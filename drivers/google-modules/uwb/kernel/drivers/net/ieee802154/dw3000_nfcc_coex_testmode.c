/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2021 Qorvo US, Inc.
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
 * Qorvo. Please contact Qorvo to inquire about licensing terms.
 */

#include "dw3000_nfcc_coex_testmode.h"
#include "dw3000_nfcc_coex_msg.h"
#include "dw3000_nfcc_coex_core.h"
#include "dw3000_nfcc_coex_buffer.h"
#include "dw3000_nfcc_coex.h"
#include "dw3000.h"
#include "dw3000_trc.h"
#include "dw3000_core.h"

#define CCC_TEST_SLOT_DELTA_PC 15 /* Percent. */
/* Request an end of access without error on current access. */
#define DW3000_CCC_RETURN_STOP 1

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
	const struct dw3000_nfcc_coex_tlv_slot_list *in,
	struct dw3000_nfcc_coex_tlv_slot_list *out, u32 margin_pc,
	bool slot_after)
{
	int i;
	u32 free_slot_duration;
	u32 delta_i;

	if (in->nb_slots == 0)
		return -1;

	out->nb_slots = in->nb_slots - 1;
	for (i = 0; i < out->nb_slots; i++) {
		free_slot_duration = in->slots[i + 1].end_sys_time -
				     in->slots[i].start_sys_time;
		delta_i = (free_slot_duration * margin_pc) / 100;
		/* gen slot : [t_endi + DELTAi, t_starti+1 - DELTAi] */
		out->slots[i].start_sys_time =
			in->slots[i].end_sys_time + delta_i;
		out->slots[i].end_sys_time =
			in->slots[i + 1].start_sys_time - delta_i;
	}

	if (slot_after) {
		if (out->nb_slots == 0) {
			/* we need a value for free_slot_duration and delta_i. */
			free_slot_duration = in->slots[0].end_sys_time -
					     in->slots[0].start_sys_time;
			delta_i = (free_slot_duration * margin_pc) / 100;
		}
		/* Add a last slot reusing last free_slot_duration and delta_i. */
		out->nb_slots++;
		out->slots[i].start_sys_time =
			in->slots[i].end_sys_time + delta_i;
		out->slots[i].end_sys_time = in->slots[i].end_sys_time -
					     delta_i + free_slot_duration;
	}
	return 0;
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
	struct dw3000_nfcc_coex_rx_msg_info rx_msg_info = {};
	struct dw3000_nfcc_coex_buffer out = {};
	struct dw3000_nfcc_coex_tlv_slot_list out_slot_list = {};
	u32 cur_time_dtu, next_slot_dtu;
	u32 diff_dtu, diff_ms;
	int rc;

	dw->nfcc_coex.testmode_round_count++;

	rc = dw3000_nfcc_coex_message_check(dw, buffer, &rx_msg_info);
	if (rc)
		return rc;

	if (!rx_msg_info.next_slot_found)
		return dw3000_nfcc_coex_disable(dw);

	dw3000_nfcc_coex_header_put(dw, &out);
	cur_time_dtu = dw3000_get_dtu_time(dw);

	/* Behavior depends on test mode. */
	switch (test_conf->mode) {
	case DW3000_CCC_TEST_DIRECT:
		break;
	case DW3000_CCC_TEST_WAIT:
		/* Margin is typically 100ms hence msleep is used. */
		msleep(test_conf->margin_ms);
		break;
	case DW3000_CCC_TEST_LATE:
		if (rx_msg_info.next_timestamp_sys_time == 0)
			break;
		next_slot_dtu = dw3000_sys_time_to_dtu(
			dw, rx_msg_info.next_timestamp_sys_time, cur_time_dtu);
		diff_dtu = next_slot_dtu - cur_time_dtu;
		diff_ms = diff_dtu / dtu_per_ms;
		if (test_conf->RRcount == 0 ||
		    dw->nfcc_coex.testmode_round_count++ % test_conf->RRcount ==
			    0) {
			/* Margin is typically 100ms and diff_ms is a
			 * multiple of 96ms, hence msleep is used. */
			msleep(diff_ms + test_conf->margin_ms);
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
		if (dw3000_nfcc_coex_generate_test_slots(
			    rx_msg_info.slot_list, &out_slot_list,
			    CCC_TEST_SLOT_DELTA_PC, true))
			break;
		if ((test_conf->RRcount == 0 ||
		     (dw->nfcc_coex.testmode_round_count++ %
			      test_conf->RRcount ==
		      0)) &&
		    test_conf->conflict_slot_idx < out_slot_list.nb_slots) {
			/* We want a conflict this round
			 * and we have enough slot, given conflict_slot_idx.
			 * So, just copy the input slot values. */
			const int n = test_conf->conflict_slot_idx;
			const struct dw3000_nfcc_coex_tlv_slot *slot_b =
				&rx_msg_info.slot_list->slots[n];
			struct dw3000_nfcc_coex_tlv_slot *slot_a =
				&out_slot_list.slots[n];

			*slot_a = *slot_b;
		}
		dw3000_nfcc_coex_tlv_slots_put(&out, &out_slot_list);
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
	u32 absolute_session_time0_sys;
	u32 start_time_dtu;
	u64 session_time0_dtu;
	struct dw3000_nfcc_coex_buffer buffer;

	start_time_dtu = dw3000_get_dtu_time(dw);
	dw3000_nfcc_coex_header_put(dw, &buffer);
	/* session_time0 is the offset of the NFCC ranging start time in
	 * microseconds.  The absolute NFCC ranging start time is transmitted
	 * to the NFCC in SYS_TIME. */
	session_time0_dtu =
		((u64)session_time0 * (u64)(dw->llhw->dtu_freq_hz)) / 1000000;
	absolute_session_time0_sys = (start_time_dtu + session_time0_dtu)
				     << DW3000_DTU_PER_SYS_POWER;

	dw3000_nfcc_coex_tlv_u32_put(&buffer, TLV_SESSION_TIME0,
				     absolute_session_time0_sys);

	if (start && end) {
		start *= (u64)(dw->llhw->dtu_freq_hz) / 1000000;
		start += start_time_dtu;
		end *= (u64)(dw->llhw->dtu_freq_hz) / 1000000;
		end += start_time_dtu;
		dw3000_nfcc_coex_single_tlv_slot_put(&buffer, start, end);
	}

	return dw3000_nfcc_coex_write_buffer(dw, &buffer, MSG_LEN(buffer));
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
	struct dw3000_nfcc_coex_rx_msg_info rx_msg_info = {};
	struct dw3000_nfcc_coex_buffer out = {};
	u32 curtime_dtu = 0;
	u32 nextslot_dtu = 0;
	u32 diff_dtu;
	int rc = -1;

	rc = dw3000_nfcc_coex_message_check(dw, buffer, &rx_msg_info);
	if (rc)
		return rc;

	if (!rx_msg_info.next_slot_found)
		return dw3000_nfcc_coex_disable(dw);

	curtime_dtu = dw3000_get_dtu_time(dw);
	nextslot_dtu = dw3000_sys_time_to_dtu(
		dw, rx_msg_info.next_timestamp_sys_time, curtime_dtu);
	diff_dtu = nextslot_dtu - curtime_dtu;
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
 * dw3000_nfcc_coex_testmode_session() - Start one custom NFCC session.
 * @dw: Driver context.
 * @chan: Channel number.
 * @session_time0: Offset in ms to add to current time.
 * @start: Start date of NFCC session in ms.
 * @end: End date of NFCC session in ms.
 *
 * Return: 0 on success, else an error.
 */
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

/**
 * dw3000_nfcc_coex_testmode_config() - Process testmode config.
 * @dw: Driver context.
 * @testmode_config: Pointer on persistente config to use.
 *
 * Return: 0 on success, else an error.
 */
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
