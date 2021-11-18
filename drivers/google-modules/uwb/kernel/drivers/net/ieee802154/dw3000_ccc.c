/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo.
 * Please contact Qorvo to inquire about licensing terms.
 */

#include "dw3000.h"
#include "dw3000_core.h"
#include "dw3000_ccc_mailbox.h"
#include "dw3000_ccc.h"
#include "dw3000_trc.h"

/* TLVs types. */
#define TLV_SESSION_TIME0 1
#define TLV_SLOT_LIST 2
#define TLV_UWBCNT_OFFS 3
#define TLV_ERROR 4

/* TLVs len helpers. */
#define TLV_TYPELEN_LEN 2 /* type 1 byte, len 1 byte. */
#define TLV_U32_LEN (4 + 1) /* u32 + ack/nack. */
#define TLV_SLOTS_LEN(nbslots) \
	(1 + (8 * (nbslots)) + 1) /* nslots + slots + ack/nack. */
#define TLV_MAX_NB_SLOTS 4

/* Error codes for TLV_ERROR type. */
#define CCC_ERR_LATE_SPIMAVAIL 0
#define CCC_ERR_SLOT_CONFLICT 1
#define CCC_ERR_CODE_SZ 2

#define MSG_HEADER_LEN (DW3000_CCC_SIGNATURE_LEN + 3)
#define MSG_LEN(x) (MSG_HEADER_LEN + (x).tlvs_len)
#define MSG_NEXT_TLV(buffer, offset) \
	(struct dw3000_nfcc_coex_tlv *)((buffer)->msg.tlvs + (offset))

#define CCC_TEST_SLOT_DELTA_PC 15 /* Percent. */

/* Request an end of access without error on current access. */
#define DW3000_CCC_RETURN_STOP 1

/* doc/nfcc_coex/architecture.rst specify DW3000_ANTICIP_DTU as default value.
 * But the jitter on wakeup is too high. Patch with higher value from 16ms to 100ms. */
static unsigned dw3000_nfcc_coex_margin_dtu = 100 * (DW3000_DTU_FREQ / 1000);
module_param_named(nfcc_coex_margin_dtu, dw3000_nfcc_coex_margin_dtu, uint,
		   0444);
MODULE_PARM_DESC(
	nfcc_coex_margin_dtu,
	"Margin in dtu needed to give access to the NFCC controller and for the NFCC controller"
	" to wake up (default is anticip_dtu)");

/**
 * struct dw3000_nfcc_coex_tlv - Type Length Value.
 */
struct dw3000_nfcc_coex_tlv {
	/**
	 * @type: Identifier of TLV.
	 */
	u8 type;
	/**
	 * @len: Number of byte of TLV array.
	 */
	u8 len;
	/**
	 * @tlv: Value of the TLV.
	 */
	u8 tlv[];
} __attribute__((packed));

/**
 * struct dw3000_nfcc_coex_tlv_slots - TLV slots.
 */
struct dw3000_nfcc_coex_tlv_slots {
	/**
	 * @nb_slots: Number of slots used.
	 */
	u8 nb_slots;
	/**
	 * @slots: array of start/end session time.
	 */
	u32 slots[TLV_MAX_NB_SLOTS][2];
} __attribute__((packed));

/**
 * dw3000_nfcc_coex_header_put() - Fill NFCC frame header.
 * @dw: Driver context.
 * @buffer: Message buffer to fill.
 */
static void dw3000_nfcc_coex_header_put(struct dw3000 *dw,
					struct dw3000_nfcc_coex_buffer *buffer)
{
	struct dw3000_nfcc_coex_msg *msg = &buffer->msg;

	memcpy(msg->signature, DW3000_CCC_SIGNATURE_STR,
	       DW3000_CCC_SIGNATURE_LEN);
	msg->ver_id = DW3000_CCC_VER_ID;
	msg->seqnum = dw->nfcc_coex.seqnum;
	msg->nb_tlv = 0;
	buffer->tlvs_len = 0;
}

/**
 * dw3000_nfcc_coex_tlv_u32_put() - Fill buffer payload for a TLV.
 * @buffer: Message buffer to fill.
 * @type: Type id of the TLV.
 * @value: Value of TLV.
 */
static void dw3000_nfcc_coex_tlv_u32_put(struct dw3000_nfcc_coex_buffer *buffer,
					 u8 type, u32 value)
{
	struct dw3000_nfcc_coex_msg *msg = &buffer->msg;
	struct dw3000_nfcc_coex_tlv *tlv;
	u32 *v;

	tlv = MSG_NEXT_TLV(buffer, buffer->tlvs_len);
	msg->nb_tlv++;
	tlv->type = type;
	tlv->len = 4;
	v = (u32 *)&tlv->tlv;
	*v = value;
	buffer->tlvs_len += TLV_TYPELEN_LEN + TLV_U32_LEN;
}

/**
 * dw3000_nfcc_coex_single_tlv_slot_put() - Fill buffer payload for a single TLV slot.
 * @buffer: Message buffer to fill.
 * @start: Start date of NFCC session in ms.
 * @end: End date of NFCC session in ms.
 */
static void
dw3000_nfcc_coex_single_tlv_slot_put(struct dw3000_nfcc_coex_buffer *buffer,
				     u32 start, u32 end)
{
	struct dw3000_nfcc_coex_msg *msg = &buffer->msg;
	struct dw3000_nfcc_coex_tlv *tlv;
	struct dw3000_nfcc_coex_tlv_slots *slots;

	tlv = MSG_NEXT_TLV(buffer, buffer->tlvs_len);
	msg->nb_tlv++;
	tlv->type = TLV_SLOT_LIST;
	tlv->len = TLV_SLOTS_LEN(1);
	slots = (struct dw3000_nfcc_coex_tlv_slots *)&tlv->tlv;
	slots->nb_slots = 1;
	slots->slots[0][0] = start;
	slots->slots[0][1] = end;
	buffer->tlvs_len += TLV_TYPELEN_LEN + tlv->len;
}

/**
 * dw3000_nfcc_coex_tlv_slots_put() - Fill buffer payload for a TLV slots.
 * @buffer: Message buffer to fill.
 * @slots: TLV slots.
 */
static void
dw3000_nfcc_coex_tlv_slots_put(struct dw3000_nfcc_coex_buffer *buffer,
			       const struct dw3000_nfcc_coex_tlv_slots *slots)
{
	struct dw3000_nfcc_coex_msg *msg = &buffer->msg;
	struct dw3000_nfcc_coex_tlv *tlv;

	tlv = MSG_NEXT_TLV(buffer, buffer->tlvs_len);
	msg->nb_tlv++;
	tlv->type = TLV_SLOT_LIST;
	tlv->len = TLV_SLOTS_LEN(slots->nb_slots);
	memcpy(&tlv->tlv, slots, sizeof(*slots));
	buffer->tlvs_len += TLV_TYPELEN_LEN + tlv->len;
}

/**
 * dw3000_nfcc_coex_check_buffer() - Check buffer content.
 * @dw: Driver context.
 * @buffer: Buffer to check.
 *
 * Return: 0 when buffer contain a message, else an error.
 */
static int
dw3000_nfcc_coex_check_buffer(struct dw3000 *dw,
			      const struct dw3000_nfcc_coex_buffer *buffer)
{
	const struct dw3000_nfcc_coex_msg *msg;

	if (!buffer)
		return -EINVAL;

	msg = &buffer->msg;
	/* check signature */
	if (memcmp(msg->signature, DW3000_CCC_SIGNATURE_STR,
		   DW3000_CCC_SIGNATURE_LEN)) {
		trace_dw3000_nfcc_coex_err(
			dw, "signature doesn't match with expected");
		return -EINVAL;
	}

	/* check AP_NFCC Interface Version ID */
	if (msg->ver_id != DW3000_CCC_VER_ID)
		trace_dw3000_nfcc_coex_warn(dw, "version id doesn't match");

	/* Read number of TLVs */
	if (msg->nb_tlv > DW3000_CCC_MAX_NB_TLV) {
		trace_dw3000_nfcc_coex_err(dw, "nb_tlv exceeds max");
		return -EINVAL;
	}

	return 0;
}

/**
 * dw3000_nfcc_coex_process_tlvs() - Set data from message parsing.
 * @dw: Driver context.
 * @buffer: Buffer to read.
 * @data: Result of message parsed updated on success.
 * @disable_allowed: When true, disable nfcc coex on TLV absence.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_process_tlvs(
	struct dw3000 *dw, const struct dw3000_nfcc_coex_buffer *buffer,
	struct dw3000_nfcc_coex_data *data, bool disable_allowed)
{
	const struct dw3000_nfcc_coex_msg *msg = &buffer->msg;
	int i;
	int tlvs_len = 0; /* Start parsing at first TLV. */

	/* If no Tlv, assume the session ended. */
	if (msg->nb_tlv == 0) {
		if (disable_allowed)
			return dw3000_nfcc_coex_disable(dw);
		return 0;
	}

	/* Process tlvs. */
	for (i = 0; i < msg->nb_tlv; i++) {
		struct dw3000_nfcc_coex_tlv *tlv;

		tlv = MSG_NEXT_TLV(buffer, tlvs_len);
		tlvs_len += tlv->len;

		switch (tlv->type) {
		case TLV_SLOT_LIST:
			data->slots =
				(struct dw3000_nfcc_coex_tlv_slots *)&tlv->tlv;
			if (data->slots->nb_slots > 0)
				data->nextslot = data->slots->slots[0][0];
			break;
		case TLV_ERROR:
			trace_dw3000_nfcc_coex_err(dw, "nfcc sent an error");
			/* FIXME: return -1; ???? */
			break;
		default:
			trace_dw3000_nfcc_coex_warn(
				dw, "ignoring unexpected TLV type");
			break;
		}
	}

	return 0;
}

/**
 * dw3000_nfcc_coex_generate_test_slots() - Generates a series of valid, interleaved slots.
 * @in: Slots found in incomming message.
 * @out: Slots output to write.
 * @margin_pc: Margin in pourcent.
 * @slot_after: True when reuse last free slot duration is permit.
 *
 * Return: 0 on success, else an error.
 *
 * For instance, if given the input:
 *   [t_start0, t_end0], [t_start1, t_end1], [t_start2, t_end2]
 * We generate the output:
 *   [t_end0 + DELTA0, t_start1 - DELTA0], [t_end1 + DELTA1, t_start2 - DELTA1]
 * Where DELTAn = (( t_startn+1 - t_endn ) * margin_pc) / 100
 *
 * NB :
 * - assumes input slots are valid, i.e in->nb_slots <= TLV_MAX_NB_SLOTS
 * - If slot_after is passed true, append a last slot AFTER all NFCC slots
 */
static int dw3000_nfcc_coex_generate_test_slots(
	const struct dw3000_nfcc_coex_tlv_slots *in,
	struct dw3000_nfcc_coex_tlv_slots *out, u32 margin_pc, bool slot_after)
{
	int i;
	u32 free_slot_duration;
	u32 delta_i;

	if (in->nb_slots == 0)
		return -1;

	out->nb_slots = in->nb_slots - 1;
	for (i = 0; i < out->nb_slots; i++) {
		free_slot_duration = in->slots[i + 1][1] - in->slots[i][0];
		delta_i = (free_slot_duration * margin_pc) / 100;
		/* gen slot : [t_endi + DELTAi, t_starti+1 - DELTAi] */
		out->slots[i][0] = in->slots[i][1] + delta_i;
		out->slots[i][1] = in->slots[i + 1][0] - delta_i;
	}

	if (slot_after) {
		if (out->nb_slots == 0) {
			/* we need a value for free_slot_duration and delta_i. */
			free_slot_duration = in->slots[0][1] - in->slots[0][0];
			delta_i = (free_slot_duration * margin_pc) / 100;
		}
		/* Add a last slot reusing last free_slot_duration and delta_i. */
		out->nb_slots++;
		out->slots[i][0] = in->slots[i][1] + delta_i;
		out->slots[i][1] =
			in->slots[i][1] - delta_i + free_slot_duration;
	}
	return 0;
}

/**
 * dw3000_nfcc_coex_sleep() - Entering in deep sleep.
 * @dw: Driver context.
 * @sleep_dtu: Sleep duration in dtu.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_sleep(struct dw3000 *dw, u32 sleep_dtu)
{
	const u32 dtu_per_ms = dw->llhw->dtu_freq_hz / 1000;

	return dw3000_go_to_deep_sleep_and_wakeup_after_ms(
		dw, sleep_dtu / dtu_per_ms);
}

/**
 * dw3000_nfcc_coex_testmode_session_spi_avail_handler() - Start testmode callback.
 * @dw: Driver context.
 * @buffer: Memory read from scratch memory.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_testmode_session_spi_avail_handler(
	struct dw3000 *dw, const struct dw3000_nfcc_coex_buffer *buffer)
{
	struct dw3000_nfcc_coex_data data = {};
	struct dw3000_nfcc_coex_buffer out = {};
	u32 curtime = 0;
	u32 diff_dtu;
	int rc = -1;

	if (!buffer)
		return -1;

	rc = dw3000_nfcc_coex_check_buffer(dw, buffer);
	if (rc)
		return rc;

	rc = dw3000_nfcc_coex_process_tlvs(dw, buffer, &data, true);
	if (rc)
		return rc;

	rc = dw3000_read_sys_time(dw, &curtime);
	if (rc)
		return rc;

	diff_dtu = data.nextslot > 0 ? data.nextslot - curtime : 0;
	data.diff_ms = diff_dtu / (dw->llhw->dtu_freq_hz / 1000);
	if (diff_dtu > dw3000_nfcc_coex_margin_dtu) {
		/* Wait for next slot, use a margin, as it looks like we do have a
		 * lot of fluctuations on NFCC side, leading to TXLATE when trying small
		 * delay.
		 * The next message is send in dw3000_isr_handle_spi_ready() on the wake up.
		 */
		rc = dw3000_nfcc_coex_sleep(
			dw, diff_dtu - dw3000_nfcc_coex_margin_dtu);
	} else {
		/* The delay is too short to sleep before respond to the NFCC. */
		dw3000_nfcc_coex_header_put(dw, &out);
		rc = dw3000_nfcc_coex_write_buffer(dw, &out, MSG_LEN(out));
	}
	return rc;
}

/**
 * dw3000_nfcc_coex_testmode_config_spi_avail_handler() - Config testmode callback.
 * @dw: Driver context.
 * @buffer: Memory read from scratch memory.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_testmode_config_spi_avail_handler(
	struct dw3000 *dw, const struct dw3000_nfcc_coex_buffer *buffer)
{
	const struct dw3000_nfcc_coex_testmode_config *test_conf =
		dw->nfcc_coex.testmode_config;
	const u32 dtu_per_ms = dw->llhw->dtu_freq_hz / 1000;
	struct dw3000_nfcc_coex_data data = {};
	struct dw3000_nfcc_coex_buffer out = {};
	struct dw3000_nfcc_coex_tlv_slots out_slots = {};
	u32 curtime = 0;
	u32 diff_dtu;
	int rc = -1;

	dw->nfcc_coex.testmode_round_count++;

	if (!buffer)
		return DW3000_CCC_RETURN_STOP;

	rc = dw3000_nfcc_coex_check_buffer(dw, buffer);
	if (rc)
		return rc;

	rc = dw3000_nfcc_coex_process_tlvs(dw, buffer, &data, true);
	if (rc)
		return rc;

	dw3000_nfcc_coex_header_put(dw, &out);

	rc = dw3000_read_sys_time(dw, &curtime);
	if (rc)
		return rc;

	/* Behavior depends on test mode. */
	switch (test_conf->mode) {
	case DW3000_CCC_TEST_DIRECT:
		break;
	case DW3000_CCC_TEST_WAIT:
		/* Margin is typically 100ms hence msleep is used. */
		msleep(test_conf->margin_ms);
		break;
	case DW3000_CCC_TEST_LATE:
		if (data.nextslot == 0)
			break;
		diff_dtu = data.nextslot - curtime;
		data.diff_ms = diff_dtu / dtu_per_ms;
		if (test_conf->RRcount == 0 ||
		    dw->nfcc_coex.testmode_round_count++ % test_conf->RRcount ==
			    0) {
			/* Margin is typically 100ms and diff_ms is a
			 * multiple of 96ms, hence msleep is used. */
			msleep(data.diff_ms + test_conf->margin_ms);
			break;
		}
		if (diff_dtu > dw3000_nfcc_coex_margin_dtu) {
			/* Difference is a multiple of 96ms, hence msleep
			 * is used. */
			msleep((diff_dtu - dw3000_nfcc_coex_margin_dtu) /
			       dtu_per_ms);
		}
		break;
	case DW3000_CCC_TEST_CONFLICT:
		if (dw3000_nfcc_coex_generate_test_slots(data.slots, &out_slots,
							 CCC_TEST_SLOT_DELTA_PC,
							 true))
			break;
		if ((test_conf->RRcount == 0 ||
		     (dw->nfcc_coex.testmode_round_count++ %
			      test_conf->RRcount ==
		      0)) &&
		    test_conf->conflit_slot_idx < out_slots.nb_slots) {
			/* We want a conflict this round
			 * and we have enough slot, given conflit_slot_idx.
			 * So, just copy the input slot values. */
			out_slots.slots[test_conf->conflit_slot_idx][0] =
				data.slots
					->slots[test_conf->conflit_slot_idx][0];
			out_slots.slots[test_conf->conflit_slot_idx][1] =
				data.slots
					->slots[test_conf->conflit_slot_idx][1];
		}
		dw3000_nfcc_coex_tlv_slots_put(&out, &out_slots);
		break;
	case DW3000_CCC_TEST_SLEEP_OFFSET:
		dw3000_nfcc_coex_tlv_u32_put(&out, TLV_UWBCNT_OFFS,
					     test_conf->offset_ms);
		break;
	default:
		return DW3000_CCC_RETURN_STOP;
	}

	return dw3000_nfcc_coex_write_buffer(dw, &out, MSG_LEN(out));
}

/**
 * dw3000_nfcc_coex_clock_sync_frame_payload_put() - Fill clock sync frame payload.
 * @dw: Driver context.
 * @handle_access: Handle access informations.
 * @buffer: Buffer to set with help of handle_access.
 */
static void dw3000_nfcc_coex_clock_sync_frame_payload_put(
	struct dw3000 *dw,
	const struct dw3000_vendor_cmd_nfcc_coex_handle_access *handle_access,
	struct dw3000_nfcc_coex_buffer *buffer)
{
	u32 session_time0 = handle_access->timestamp_dtu;

	dw3000_nfcc_coex_header_put(dw, buffer);

	trace_dw3000_nfcc_coex_clock_sync_frame_payload_put(dw, session_time0);
	dw3000_nfcc_coex_tlv_u32_put(buffer, TLV_SESSION_TIME0, session_time0);
}

/**
 * dw3000_nfcc_coex_req_time_interval_frame_payload_put() - Fill request time interval payload.
 * @dw: Driver context.
 * @handle_access: Handle access informations.
 * @buffer: Buffer to set with help of handle_access.
 */
static void dw3000_nfcc_coex_req_time_interval_frame_payload_put(
	struct dw3000 *dw,
	const struct dw3000_vendor_cmd_nfcc_coex_handle_access *handle_access,
	struct dw3000_nfcc_coex_buffer *buffer)
{
	u32 start_dtu = handle_access->timestamp_dtu;
	u32 end_dtu = start_dtu + handle_access->duration_dtu;

	dw3000_nfcc_coex_header_put(dw, buffer);

	trace_dw3000_nfcc_coex_req_time_interval_frame_payload_put(
		dw, start_dtu, end_dtu);
	if (handle_access->duration_dtu)
		dw3000_nfcc_coex_single_tlv_slot_put(buffer, start_dtu,
						     end_dtu);
}

/**
 * dw3000_nfcc_coex_clk_offset_frame_payload_put() - Fill clock offset payload.
 * @dw: Driver context.
 * @offset_dtu: Offset to add to next ccc schedule.
 * @buffer: Buffer to set with help of handle_access.
 */
static void dw3000_nfcc_coex_clk_offset_frame_payload_put(
	struct dw3000 *dw, u32 offset_dtu,
	struct dw3000_nfcc_coex_buffer *buffer)
{
	dw3000_nfcc_coex_header_put(dw, buffer);

	trace_dw3000_nfcc_coex_clk_offset_frame_payload_put(dw, offset_dtu);
	dw3000_nfcc_coex_tlv_u32_put(buffer, TLV_UWBCNT_OFFS, offset_dtu);
}

/**
 * dw3000_nfcc_coex_testmode_write_msg() - Write custom message for NFCC.
 * @dw: Driver context.
 * @session_time0: Offset in ms to add to current time.
 * @start: Start date of NFCC session in ms.
 * @end: End date of NFCC session in ms.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_testmode_write_msg(struct dw3000 *dw,
					       u32 session_time0, u32 start,
					       u32 end)
{
	int rc;
	u32 start_time_dtu;
	u32 absolute_session_time0_dtu;
	u64 session_time0_dtu;
	struct dw3000_nfcc_coex_buffer buffer;

	rc = dw3000_read_sys_time(dw, &start_time_dtu);
	if (rc)
		return rc;

	dw3000_nfcc_coex_header_put(dw, &buffer);
	/* session_time0 is the offset of the NFCC ranging start time in
	 * microseconds.  The absolute NFCC ranging start time is transmitted
	 * to the NFCC in DTU. */
	session_time0_dtu =
		((u64)session_time0 * (u64)(dw->llhw->dtu_freq_hz)) / 1000000;
	absolute_session_time0_dtu = start_time_dtu + session_time0_dtu;

	dw3000_nfcc_coex_tlv_u32_put(&buffer, TLV_SESSION_TIME0,
				     absolute_session_time0_dtu);

	if (start && end) {
		start *= (u64)(dw->llhw->dtu_freq_hz) / 1000000;
		start += start_time_dtu;
		end *= (u64)(dw->llhw->dtu_freq_hz) / 1000000;
		end += start_time_dtu;
		dw3000_nfcc_coex_single_tlv_slot_put(&buffer, start, end);
	}

	return dw3000_nfcc_coex_write_buffer(dw, &buffer, MSG_LEN(buffer));
}

int dw3000_nfcc_coex_testmode_session(struct dw3000 *dw, u8 chan,
				      u32 session_time0, u32 start, u32 end)
{
	int r;

	r = dw3000_nfcc_coex_enable(
		dw, chan, dw3000_nfcc_coex_testmode_session_spi_avail_handler);
	if (r)
		return r;

	trace_dw3000_nfcc_coex_warn(dw, "testmode start");
	return dw3000_nfcc_coex_testmode_write_msg(dw, session_time0, start,
						   end);
}

int dw3000_nfcc_coex_testmode_config(
	struct dw3000 *dw,
	const struct dw3000_nfcc_coex_testmode_config *testmode_config)
{
	int r;

	if (!testmode_config) {
		trace_dw3000_nfcc_coex_err(
			dw, "can't start testmode with a NULL conf");
		return -EINVAL;
	}

	dw->nfcc_coex.testmode_config = testmode_config;
	dw->nfcc_coex.testmode_round_count = 0;
	r = dw3000_nfcc_coex_enable(
		dw, testmode_config->channel,
		dw3000_nfcc_coex_testmode_config_spi_avail_handler);
	if (r)
		return r;

	trace_dw3000_nfcc_coex_warn(dw, "testmode run config");
	return dw3000_nfcc_coex_testmode_write_msg(
		dw, testmode_config->session_time0, testmode_config->start,
		testmode_config->end);
}

int dw3000_nfcc_coex_write_msg_on_wakeup(struct dw3000 *dw)
{
	struct dw3000_nfcc_coex_buffer buffer = {};
	/* Update the NFCC clock reference:
	 * As DW clock have been cleared with deep sleep event, then update
	 * value to forward to NFCC is negative.
	 * NFCC will add the offset_dtu value to maintain the same base
	 * time. */
	u32 offset_dtu = -dw->deep_sleep_state.dtu_wakeup_offset;

	dw3000_nfcc_coex_clk_offset_frame_payload_put(dw, offset_dtu, &buffer);
	return dw3000_nfcc_coex_write_buffer(dw, &buffer, MSG_LEN(buffer));
}

/**
 * dw3000_nfcc_coex_update_access_info() - Update access info cache.
 * @dw: Driver context.
 * @buffer: buffer to read.
 */
static void dw3000_nfcc_coex_update_access_info(
	struct dw3000 *dw, const struct dw3000_nfcc_coex_buffer *buffer)
{
	struct dw3000_vendor_cmd_nfcc_coex_get_access_info *access_info =
		&dw->nfcc_coex.access_info;
	struct dw3000_nfcc_coex_data data = {};
	int r;

	if (!buffer)
		goto failure;

	r = dw3000_nfcc_coex_check_buffer(dw, buffer);
	if (r)
		goto failure;

	r = dw3000_nfcc_coex_process_tlvs(dw, buffer, &data, false);
	if (r)
		goto failure;

	access_info->stop = !buffer->msg.nb_tlv;
	access_info->next_timestamp_dtu = data.nextslot;
	return;

failure:
	/* When buffer content is unexpected then request a stop. */
	memset(access_info, 0, sizeof(*access_info));
	access_info->stop = true;
}

/**
 * dw3000_nfcc_coex_spi_avail_handler() - main spi available handler.
 * @dw: Driver context.
 * @buffer: buffer to read.
 *
 * Return: 0 on success, else an error.
 */
static int
dw3000_nfcc_coex_spi_avail_handler(struct dw3000 *dw,
				   const struct dw3000_nfcc_coex_buffer *buffer)
{
	dw3000_nfcc_coex_update_access_info(dw, buffer);

	if (dw->nfcc_coex.access_info.stop) {
		int r = dw3000_nfcc_coex_disable(dw);
		if (r)
			return r;
	}

	mcps802154_tx_done(dw->llhw);
	return 0;
}

/**
 * dw3000_nfcc_coex_handle_access() - handle access to provide to NFCC.
 * @dw: Driver context.
 * @data: Adress of handle access information.
 * @data_len: Number of byte of the data object.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_handle_access(struct dw3000 *dw, void *data,
					  int data_len)
{
	const struct dw3000_vendor_cmd_nfcc_coex_handle_access *info = data;
	struct dw3000_nfcc_coex_buffer buffer = {};
	u32 cur_time_dtu;
	u32 diff_dtu;
	int r;

	if (!info || data_len != sizeof(*info))
		return -EINVAL;

	r = dw3000_read_sys_time(dw, &cur_time_dtu);
	if (r)
		return r;

	if (info->start) {
		r = dw3000_nfcc_coex_enable(dw, info->chan,
					    dw3000_nfcc_coex_spi_avail_handler);
		if (r)
			return r;
	}

	diff_dtu = info->timestamp_dtu - cur_time_dtu;

	if (diff_dtu > dw3000_nfcc_coex_margin_dtu)
		return dw3000_nfcc_coex_sleep(
			dw, diff_dtu - dw3000_nfcc_coex_margin_dtu);

	if (info->start)
		dw3000_nfcc_coex_clock_sync_frame_payload_put(dw, info,
							      &buffer);
	else
		dw3000_nfcc_coex_req_time_interval_frame_payload_put(dw, info,
								     &buffer);
	return dw3000_nfcc_coex_write_buffer(dw, &buffer, MSG_LEN(buffer));
}

/**
 * dw3000_nfcc_coex_get_access_information() - Forward access info cached.
 * @dw: Driver context.
 * @data: Adress where to write access information.
 * @data_len: Number of byte of the data object.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_get_access_information(struct dw3000 *dw,
						   void *data, int data_len)
{
	const struct dw3000_vendor_cmd_nfcc_coex_get_access_info *access_info =
		&dw->nfcc_coex.access_info;

	if (!data || data_len != sizeof(*access_info))
		return -EINVAL;

	memcpy(data, access_info, data_len);
	return 0;
}

/**
 * dw3000_nfcc_coex_stop() - Stop NFCC.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_stop(struct dw3000 *dw)
{
	return dw3000_nfcc_coex_disable(dw);
}

int dw3000_nfcc_coex_vendor_cmd(struct dw3000 *dw, u32 vendor_id, u32 subcmd,
				void *data, size_t data_len)
{
	switch (subcmd) {
	case DW3000_VENDOR_CMD_NFCC_COEX_HANDLE_ACCESS:
		return dw3000_nfcc_coex_handle_access(dw, data, data_len);
	case DW3000_VENDOR_CMD_NFCC_COEX_GET_ACCESS_INFORMATION:
		return dw3000_nfcc_coex_get_access_information(dw, data,
							       data_len);
	case DW3000_VENDOR_CMD_NFCC_COEX_STOP:
		return dw3000_nfcc_coex_stop(dw);
	default:
		return -EINVAL;
	}
}
