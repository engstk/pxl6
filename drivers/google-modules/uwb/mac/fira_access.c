/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2021 Qorvo US, Inc.
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

#include "fira_access.h"
#include "fira_session.h"
#include "fira_frame.h"
#include "fira_trace.h"

#include <asm/unaligned.h>
#include <linux/string.h>
#include <linux/ieee802154.h>
#include <linux/math64.h>
#include <linux/limits.h>
#include <net/mcps802154_frame.h>

#include "warn_return.h"

#define FIRA_STS_FOM_THRESHOLD 153

static struct mcps802154_access *
fira_access_controlee(struct fira_local *local, struct fira_session *session);

/**
 * sat_fp() - Saturate the range of fixed-point
 * @x: fixed-point value on s32.
 *
 * Return: value saturate to s16 range
 */
static s16 sat_fp(s32 x)
{
	if (x > S16_MAX)
		return S16_MAX;
	else if (x < S16_MIN)
		return S16_MIN;
	else
		return x;
}

/**
 * map_q11_to_2pi() - Map a Fixed Point angle to an signed 16 bit interger
 * @x: angle as Q11 fixed_point value in range [-PI, PI]
 *
 * Return: the angle mapped to [INT16_MIN, INT16_MAX]
 */
static s16 map_q11_to_2pi(s16 x)
{
	const s16 pi = 6434; /* Same as round(M_PI * (1 << 11)). */

	s32 temp = (s32)x * S16_MAX;
	temp /= pi;

	return sat_fp(temp);
}

/**
 * fira_access_setup_frame() - Fill an access frame from a FiRa slot.
 * @local: FiRa context.
 * @session: Session.
 * @frame: Access frame.
 * @sts_params: Where to store STS parameters.
 * @slot: Corresponding slot.
 * @frame_dtu: frame transmission or reception date.
 * @is_tx: true for TX.
 */
static void fira_access_setup_frame(struct fira_local *local,
				    struct fira_session *session,
				    struct mcps802154_access_frame *frame,
				    struct mcps802154_sts_params *sts_params,
				    const struct fira_slot *slot, u32 frame_dtu,
				    bool is_tx)
{
	const struct fira_session_params *params = &session->params;
	struct mcps802154_sts_params *sts_params_for_access = NULL;
	int rframe_config = session->params.rframe_config;

	const struct fira_measurement_sequence_step *current_ms_step =
		fira_session_get_current_meas_seq_step(session);

	bool is_rframe = slot->message_id <= FIRA_MESSAGE_ID_RFRAME_MAX;
	bool is_last_rframe = slot->message_id == FIRA_MESSAGE_ID_RANGING_FINAL;
	bool is_first_frame = slot->message_id == FIRA_MESSAGE_ID_CONTROL;

	if (is_rframe) {
		memcpy(sts_params->v, session->crypto.sts_v, AES_BLOCK_SIZE);
		put_unaligned_be32(slot->index,
				   sts_params->v + AES_BLOCK_SIZE / 2);
		memcpy(sts_params->key,
		       session->crypto.derived_authentication_key,
		       AES_KEYSIZE_128);
		/* Constant for the moment. */
		sts_params->n_segs = 1;
		sts_params->seg_len = 64;
		sts_params->sp2_tx_gap_4chips = 0;
		sts_params->sp2_rx_gap_4chips[0] = 0;
		sts_params->sp2_rx_gap_4chips[1] = 0;
		sts_params->sp2_rx_gap_4chips[2] = 0;
		sts_params->sp2_rx_gap_4chips[3] = 0;
		sts_params_for_access = sts_params;
	}

	if (is_tx) {
		u8 flags = MCPS802154_TX_FRAME_TIMESTAMP_DTU;

		/* Add a small margin to the Tx timestamps. */
		if (!is_first_frame)
			frame_dtu += FIRA_TX_MARGIN_US *
				     local->llhw->dtu_freq_hz / 1000000;

		if (is_rframe) {
			struct fira_ranging_info *ranging_info;

			ranging_info =
				&local->ranging_info[slot->ranging_index];
			ranging_info->timestamps_rctu[slot->message_id] =
				mcps802154_tx_timestamp_dtu_to_rmarker_rctu(
					local->llhw, frame_dtu,
					slot->tx_ant_set);

			flags |= MCPS802154_TX_FRAME_RANGING;
			if (rframe_config == FIRA_RFRAME_CONFIG_SP3)
				flags |= MCPS802154_TX_FRAME_SP3;
			else
				flags |= MCPS802154_TX_FRAME_SP1;
			if (!is_last_rframe)
				flags |= MCPS802154_TX_FRAME_KEEP_RANGING_CLOCK;
		} else if (is_first_frame) {
			flags |= MCPS802154_TX_FRAME_RANGING_ROUND;
		}
		*frame = (struct mcps802154_access_frame){
			.is_tx = true,
			.tx_frame_info = {
				.timestamp_dtu = frame_dtu,
				.flags = flags,
				.ant_set_id = slot->tx_ant_set,
			},
			.sts_params = sts_params_for_access,
		};
	} else {
		u8 flags = MCPS802154_RX_INFO_TIMESTAMP_DTU;
		u16 request = 0;

		if (is_rframe) {
			flags |= MCPS802154_RX_INFO_RANGING;
			if (rframe_config == FIRA_RFRAME_CONFIG_SP3)
				flags |= MCPS802154_RX_INFO_SP3;
			else
				flags |= MCPS802154_RX_INFO_SP1;
			if (!is_last_rframe)
				flags |= MCPS802154_RX_INFO_KEEP_RANGING_CLOCK;
			request = MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU |
				  MCPS802154_RX_FRAME_INFO_RANGING_STS_FOM;
			if (current_ms_step->type !=
			    FIRA_MEASUREMENT_TYPE_RANGE) {
				flags |= MCPS802154_RX_INFO_RANGING_PDOA;
				request |=
					MCPS802154_RX_FRAME_INFO_RANGING_PDOA |
					MCPS802154_RX_FRAME_INFO_RANGING_PDOA_FOM;
			}
			if (params->ranging_round_usage ==
				    FIRA_RANGING_ROUND_USAGE_SSTWR &&
			    session->params.device_type ==
				    FIRA_DEVICE_TYPE_CONTROLEE)
				request |=
					MCPS802154_RX_FRAME_INFO_RANGING_OFFSET;
		}
		*frame = (struct mcps802154_access_frame){
			.is_tx = false,
			.rx = {
				.info = {
					.timestamp_dtu = frame_dtu,
					.flags = flags,
					.ant_set_id = slot->rx_ant_set,
				},
				.frame_info_flags_request = request,
			},
			.sts_params = sts_params_for_access,
		};
	}
}

static bool fira_rx_sts_good(struct fira_local *local,
			     const struct mcps802154_rx_frame_info *info)
{
	if (!(info->flags & MCPS802154_RX_FRAME_INFO_RANGING_STS_FOM))
		return false;
	/* Only one segment for the moment. */
	if (info->ranging_sts_fom[0] < FIRA_STS_FOM_THRESHOLD)
		return false;
	return true;
}

static void fira_ranging_info_set_status(struct fira_ranging_info *ranging_info,
					 enum fira_ranging_status status,
					 u8 slot_index)
{
	/* Report first error. */
	if (ranging_info->status)
		return;
	ranging_info->status = status;
	ranging_info->slot_index = slot_index;
}

static void fira_rx_frame_ranging(struct fira_local *local,
				  const struct fira_slot *slot,
				  struct sk_buff *skb,
				  const struct mcps802154_rx_frame_info *info)
{
	const struct fira_session *session = local->current_session;
	struct fira_ranging_info *ranging_info =
		&local->ranging_info[slot->ranging_index];
	struct mcps802154_ie_get_context ie_get = {};

	bool pdoa_info_present;

	if (!fira_rx_sts_good(local, info)) {
		fira_ranging_info_set_status(
			ranging_info, FIRA_STATUS_RANGING_RX_PHY_STS_FAILED,
			slot->index);
		return;
	}

	if (!(info->flags & MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU)) {
		fira_ranging_info_set_status(
			ranging_info, FIRA_STATUS_RANGING_RX_PHY_TOA_FAILED,
			slot->index);
		return;
	}
	ranging_info->timestamps_rctu[slot->message_id] = info->timestamp_rctu;

	pdoa_info_present = info->flags & MCPS802154_RX_FRAME_INFO_RANGING_PDOA;

	if (pdoa_info_present) {
		struct fira_local_aoa_info *local_aoa;
		bool pdoa_fom_info_present =
			info->flags & MCPS802154_RX_FRAME_INFO_RANGING_PDOA_FOM;
		s16 local_pdoa_q11 = info->ranging_pdoa_rad_q11;
		s16 local_aoa_q11 = info->ranging_aoa_rad_q11;
		const struct fira_measurement_sequence_step *current_step =
			fira_session_get_current_meas_seq_step(session);

		switch (current_step->type) {
		case FIRA_MEASUREMENT_TYPE_AOA:
			local_aoa = &ranging_info->local_aoa;
			break;
		case FIRA_MEASUREMENT_TYPE_AOA_AZIMUTH:
			local_aoa = &ranging_info->local_aoa_azimuth;
			break;
		case FIRA_MEASUREMENT_TYPE_AOA_ELEVATION:
			local_aoa = &ranging_info->local_aoa_elevation;
			break;
		case FIRA_MEASUREMENT_TYPE_AOA_AZIMUTH_ELEVATION:
			local_aoa = (slot->message_id ==
				     FIRA_MESSAGE_ID_RANGING_FINAL) ?
					    &ranging_info->local_aoa_elevation :
					    &ranging_info->local_aoa_azimuth;
			break;
		default: /* LCOV_EXCL_START */
			local_aoa = NULL;
			/* LCOV_EXCL_STOP */
		}

		/* LCOV_EXCL_START */
		if (local_aoa) {
			/* LCOV_EXCL_STOP */
			local_aoa->present = true;
			local_aoa->rx_ant_set = slot->rx_ant_set;
			local_aoa->pdoa_2pi = map_q11_to_2pi(local_pdoa_q11);
			local_aoa->aoa_2pi = map_q11_to_2pi(local_aoa_q11);
			/* LCOV_EXCL_START */
			/* FoM is always expected when PDoA present */
			if (pdoa_fom_info_present)
				/* LCOV_EXCL_STOP */
				local_aoa->aoa_fom = info->ranging_pdoa_fom;
		}
	}

	if (info->flags & MCPS802154_RX_FRAME_INFO_RANGING_OFFSET) {
		ranging_info->clock_offset_q26 =
			div64_s64((s64)info->ranging_offset_rctu << 26,
				  info->ranging_tracking_interval_rctu);
		ranging_info->clock_offset_present = true;
	}

	if (skb) {
		if (fira_frame_header_check_decrypt(local, slot, skb, &ie_get,
						    NULL, NULL) ||
		    !fira_frame_rframe_payload_check(local, slot, skb,
						     &ie_get)) {
			fira_ranging_info_set_status(
				ranging_info,
				FIRA_STATUS_RANGING_RX_MAC_IE_DEC_FAILED,
				slot->index);
		}
	}
}

static void fira_rx_frame_control(struct fira_local *local,
				  const struct fira_slot *slot,
				  struct sk_buff *skb,
				  const struct mcps802154_rx_frame_info *info)
{
	struct fira_ranging_info *ranging_info =
		&local->ranging_info[slot->ranging_index];
	struct fira_session *session = local->current_session;
	struct fira_session *allow_resync_session = NULL;
	struct mcps802154_ie_get_context ie_get = {};
	u32 sts_index;
	unsigned int n_slots;
	int last_slot_index, block_stride_len;
	int i;
	bool stop_ranging;

	if (!(info->flags & MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU)) {
		fira_ranging_info_set_status(
			ranging_info, FIRA_STATUS_RANGING_RX_PHY_DEC_FAILED,
			slot->index);
		return;
	}
	if (fira_frame_header_check_decrypt(
		    local, slot, skb, &ie_get, &sts_index,
		    session->synchronised ? NULL : &allow_resync_session))
		goto failed;
	if (allow_resync_session) {
		session = local->current_session = allow_resync_session;
		fira_session_prepare(session);
		fira_access_controlee(local, session);
	}

	if (!fira_frame_control_payload_check(local, skb, &ie_get, &n_slots,
					      &stop_ranging, &block_stride_len))
		goto failed;

	fira_session_resync(session, sts_index, info->timestamp_dtu);

	if (stop_ranging) {
		session->stop_inband = true;
		return;
	}

	last_slot_index = 0;
	for (i = 1; i < n_slots; i++) {
		const struct fira_slot *slot = &local->slots[i];
		struct mcps802154_access_frame *frame = &local->frames[i];
		struct mcps802154_sts_params *sts_params =
			&local->sts_params[i];
		bool is_tx;
		u32 frame_dtu;

		is_tx = slot->tx_controlee_index != -1;
		frame_dtu = info->timestamp_dtu +
			    session->params.slot_duration_dtu * slot->index;
		last_slot_index = slot->index;

		fira_access_setup_frame(local, session, frame, sts_params, slot,
					frame_dtu, is_tx);
	}
	local->access.timestamp_dtu = info->timestamp_dtu;
	local->access.duration_dtu =
		session->params.slot_duration_dtu * (last_slot_index + 1);
	local->access.n_frames = n_slots;

	session->next_block_stride_len = block_stride_len;

	return;
failed:
	fira_ranging_info_set_status(ranging_info,
				     FIRA_STATUS_RANGING_RX_MAC_IE_DEC_FAILED,
				     slot->index);
	local->access.timestamp_dtu = info->timestamp_dtu;
	local->access.duration_dtu = session->params.slot_duration_dtu;
}

static void fira_rx_frame_measurement_report(
	struct fira_local *local, const struct fira_slot *slot,
	struct sk_buff *skb, const struct mcps802154_rx_frame_info *info)
{
	struct fira_ranging_info *ranging_info =
		&local->ranging_info[slot->ranging_index];
	struct mcps802154_ie_get_context ie_get = {};

	if (fira_frame_header_check_decrypt(local, slot, skb, &ie_get, NULL,
					    NULL))
		goto failed;

	if (!fira_frame_measurement_report_payload_check(local, slot, skb,
							 &ie_get))
		goto failed;

	return;
failed:
	fira_ranging_info_set_status(ranging_info,
				     FIRA_STATUS_RANGING_RX_MAC_IE_DEC_FAILED,
				     slot->index);
}

static void
fira_rx_frame_result_report(struct fira_local *local,
			    const struct fira_slot *slot, struct sk_buff *skb,
			    const struct mcps802154_rx_frame_info *info)
{
	struct fira_ranging_info *ranging_info =
		&local->ranging_info[slot->ranging_index];
	struct mcps802154_ie_get_context ie_get = {};

	if (fira_frame_header_check_decrypt(local, slot, skb, &ie_get, NULL,
					    NULL))
		goto failed;

	if (!fira_frame_result_report_payload_check(local, slot, skb, &ie_get))
		goto failed;

	return;
failed:
	fira_ranging_info_set_status(ranging_info,
				     FIRA_STATUS_RANGING_RX_MAC_IE_DEC_FAILED,
				     slot->index);
}

static bool fira_do_process_rx_frame(enum mcps802154_rx_error_type error,
				     struct fira_ranging_info *ranging_info,
				     u8 slot_index)
{
	enum fira_ranging_status status = FIRA_STATUS_RANGING_INTERNAL_ERROR;

	switch (error) {
	case MCPS802154_RX_ERROR_NONE:
		return true;
	case MCPS802154_RX_ERROR_SFD_TIMEOUT:
	case MCPS802154_RX_ERROR_TIMEOUT:
	case MCPS802154_RX_ERROR_HPDWARN:
		status = FIRA_STATUS_RANGING_RX_TIMEOUT;
		break;
	case MCPS802154_RX_ERROR_FILTERED:
	case MCPS802154_RX_ERROR_BAD_CKSUM:
		status = FIRA_STATUS_RANGING_RX_MAC_DEC_FAILED;
		break;
	case MCPS802154_RX_ERROR_UNCORRECTABLE:
	case MCPS802154_RX_ERROR_OTHER:
		status = FIRA_STATUS_RANGING_RX_PHY_DEC_FAILED;
		break;
	}
	fira_ranging_info_set_status(ranging_info, status, slot_index);
	return false;
}

static void fira_rx_frame(struct mcps802154_access *access, int frame_idx,
			  struct sk_buff *skb,
			  const struct mcps802154_rx_frame_info *info,
			  enum mcps802154_rx_error_type error)
{
	struct fira_local *local = access_to_local(access);
	const struct fira_slot *slot = &local->slots[frame_idx];
	struct fira_session *session = local->current_session;
	struct fira_ranging_info *ranging_info =
		&local->ranging_info[slot->ranging_index];

	if (fira_do_process_rx_frame(error, ranging_info, slot->index)) {
		switch (slot->message_id) {
		case FIRA_MESSAGE_ID_RANGING_INITIATION:
		case FIRA_MESSAGE_ID_RANGING_RESPONSE:
		case FIRA_MESSAGE_ID_RANGING_FINAL:
			fira_rx_frame_ranging(local, slot, skb, info);
			break;
		case FIRA_MESSAGE_ID_CONTROL:
			fira_rx_frame_control(local, slot, skb, info);
			break;
		case FIRA_MESSAGE_ID_MEASUREMENT_REPORT:
			fira_rx_frame_measurement_report(local, slot, skb,
							 info);
			break;
		case FIRA_MESSAGE_ID_RESULT_REPORT:
			fira_rx_frame_result_report(local, slot, skb, info);
			break;
		case FIRA_MESSAGE_ID_CONTROL_UPDATE:
			break;
		default:
			WARN_UNREACHABLE_DEFAULT();
		}
	}

	if (skb)
		kfree_skb(skb);

	trace_region_fira_rx_message(
		session, slot->message_id,
		local->ranging_info[slot->ranging_index].status);

	/* Controlee: Stop round on error.
	   Controller: Stop when all ranging fails. */
	if (local->ranging_info[slot->ranging_index].status)
		if (session->params.device_type == FIRA_DEVICE_TYPE_CONTROLEE ||
		    (slot->message_id <= FIRA_MESSAGE_ID_RFRAME_MAX &&
		     --local->n_ranging_valid == 0))
			access->n_frames = frame_idx + 1;

	if (session->params.device_type == FIRA_DEVICE_TYPE_CONTROLEE &&
	    session->stop_inband) {
		access->n_frames = frame_idx + 1;
	}
}

static struct sk_buff *fira_tx_get_frame(struct mcps802154_access *access,
					 int frame_idx)
{
	struct fira_local *local = access_to_local(access);
	const struct fira_slot *slot = &local->slots[frame_idx];
	struct sk_buff *skb;
	int rframe = local->current_session->params.rframe_config;

	trace_region_fira_tx_message(local->current_session, slot->message_id);
	if (rframe == FIRA_RFRAME_CONFIG_SP3 &&
	    slot->message_id <= FIRA_MESSAGE_ID_RFRAME_MAX)
		return NULL;

	skb = mcps802154_frame_alloc(local->llhw, IEEE802154_MTU, GFP_KERNEL);
	WARN_RETURN_ON(!skb, NULL);

	fira_frame_header_put(local, slot, skb);

	switch (slot->message_id) {
	case FIRA_MESSAGE_ID_RANGING_INITIATION:
	case FIRA_MESSAGE_ID_RANGING_RESPONSE:
		fira_frame_rframe_payload_put(local, skb);
		break;
	case FIRA_MESSAGE_ID_RANGING_FINAL:
		break;
	case FIRA_MESSAGE_ID_CONTROL:
		fira_frame_control_payload_put(local, slot, skb);
		break;
	case FIRA_MESSAGE_ID_MEASUREMENT_REPORT:
		fira_frame_measurement_report_payload_put(local, slot, skb);
		break;
	case FIRA_MESSAGE_ID_RESULT_REPORT:
		fira_frame_result_report_payload_put(local, slot, skb);
		break;
	case FIRA_MESSAGE_ID_CONTROL_UPDATE:
		break;
	default: /* LCOV_EXCL_START */
		kfree_skb(skb);
		WARN_UNREACHABLE_DEFAULT();
		return NULL;
		/* LCOV_EXCL_STOP */
	}

	if (fira_frame_encrypt(local, slot, skb)) {
		kfree_skb(skb);
		return NULL;
	}

	return skb;
}

static void fira_tx_return(struct mcps802154_access *access, int frame_idx,
			   struct sk_buff *skb,
			   enum mcps802154_access_tx_return_reason reason)
{
	struct fira_local *local = access_to_local(access);
	int i;

	kfree_skb(skb);

	/* Error on TX. */
	if (reason == MCPS802154_ACCESS_TX_RETURN_REASON_CANCEL) {
		for (i = 0; i < local->n_ranging_info; i++) {
			local->ranging_info[i].status =
				FIRA_STATUS_RANGING_TX_FAILED;
		}
	}
}

static void fira_access_done(struct mcps802154_access *access, bool error)
{
	struct fira_local *local = access_to_local(access);
	struct fira_session *session = local->current_session;
	int i;

	if (error)
		for (i = 0; i < local->n_ranging_info; i++)
			local->ranging_info[i].status =
				FIRA_STATUS_RANGING_INTERNAL_ERROR;
	fira_session_access_done(local, session, true);

	local->current_session = NULL;
}

struct mcps802154_access_ops fira_access_ops = {
	.common = {
		.access_done = fira_access_done,
	},
	.rx_frame = fira_rx_frame,
	.tx_get_frame = fira_tx_get_frame,
	.tx_return = fira_tx_return,
};

static __le16 fira_access_set_short_address(struct fira_local *local,
					    struct fira_session *session,
					    struct mcps802154_access *access)
{
	__le16 src_short_addr = mcps802154_get_short_addr(local->llhw);

	if (session->params.short_addr != IEEE802154_ADDR_SHORT_BROADCAST &&
	    src_short_addr != session->params.short_addr) {
		access->hw_addr_filt = (struct ieee802154_hw_addr_filt){
			.short_addr = session->params.short_addr,
		};
		access->hw_addr_filt_changed = IEEE802154_AFILT_SADDR_CHANGED;
		return session->params.short_addr;
	} else {
		access->hw_addr_filt_changed = 0;
		return src_short_addr;
	}
}

static const struct mcps802154_channel *
fira_access_channel(struct fira_local *local,
		    const struct fira_session *session)
{
	if (session->params.channel_number ||
	    session->params.preamble_code_index) {
		const struct mcps802154_channel *channel =
			mcps802154_get_current_channel(local->llhw);

		local->channel = *channel;
		if (session->params.channel_number)
			local->channel.channel = session->params.channel_number;
		if (session->params.preamble_code_index)
			local->channel.preamble_code =
				session->params.preamble_code_index;
		return &local->channel;
	}

	return NULL;
}

static struct mcps802154_access *
fira_access_controller(struct fira_local *local, struct fira_session *session)
{
	struct mcps802154_access *access;
	struct mcps802154_access_frame *frame;
	struct mcps802154_sts_params *sts_params;
	struct fira_slot *s;
	struct fira_ranging_info *ri;
	struct fira_controlees_array *controlees_array =
		&session->current_controlees;
	const struct fira_measurement_sequence_data *meas_seq =
		&session->params.meas_seq;
	const struct fira_measurement_sequence_step *current_ms_step =
		&meas_seq->active->steps[meas_seq->current_step];
	int i, j;
	u32 frame_dtu;
	int index = 0;
	bool double_sided = (session->params.ranging_round_usage ==
			     FIRA_RANGING_ROUND_USAGE_DSTWR);

	access = &local->access;
	local->src_short_addr =
		fira_access_set_short_address(local, session, access);
	local->dst_short_addr = (controlees_array->size == 1) ?
					controlees_array->data[0].short_addr :
					IEEE802154_ADDR_SHORT_BROADCAST;

	access->method = MCPS802154_ACCESS_METHOD_MULTI;
	access->ops = &fira_access_ops;
	access->timestamp_dtu = session->block_start_dtu +
				fira_session_get_round_slot(session) *
					session->params.slot_duration_dtu;
	access->frames = local->frames;
	access->channel = fira_access_channel(local, session);

	local->n_stopped_controlees_short_addr = 0;
	for (i = 0; i < controlees_array->size; i++) {
		const struct fira_controlee *c = &controlees_array->data[i];

		if (c->state != FIRA_CONTROLEE_STATE_RUNNING) {
			local->stopped_controlees_short_addr
				[local->n_stopped_controlees_short_addr] =
				c->short_addr;
			local->n_stopped_controlees_short_addr++;
		}
	}

	local->n_ranging_valid = local->n_ranging_info =
		controlees_array->size - local->n_stopped_controlees_short_addr;
	ri = local->ranging_info;

	memset(ri, 0, local->n_ranging_info * sizeof(*ri));

	s = local->slots;

	s->index = index++;
	s->tx_controlee_index = -1;
	s->ranging_index = 0;
	s->tx_ant_set = current_ms_step->tx_ant_set_nonranging;
	s->message_id = FIRA_MESSAGE_ID_CONTROL;
	s++;

	if (local->n_ranging_info) {
		s->index = index++;
		s->tx_controlee_index = -1;
		s->ranging_index = 0;
		s->tx_ant_set = current_ms_step->tx_ant_set_ranging;
		s->message_id = FIRA_MESSAGE_ID_RANGING_INITIATION;

		s++;
		for (i = 0, j = 0; i < controlees_array->size; i++) {
			if (controlees_array->data[i].state !=
			    FIRA_CONTROLEE_STATE_RUNNING)
				continue;
			ri->short_addr = controlees_array->data[i].short_addr;
			/* Requested in fira_report_aoa function. */
			ri++;
			s->index = index++;
			s->tx_controlee_index = i;
			s->ranging_index = j++;
			s->rx_ant_set = fira_session_get_rx_ant_set(
				session, FIRA_MESSAGE_ID_RANGING_RESPONSE);
			s->message_id = FIRA_MESSAGE_ID_RANGING_RESPONSE;
			s++;
		}

		if (double_sided) {
			s->index = index++;
			s->tx_controlee_index = -1;
			s->ranging_index = 0;
			s->tx_ant_set = current_ms_step->tx_ant_set_ranging;
			s->message_id = FIRA_MESSAGE_ID_RANGING_FINAL;
			s++;
		}

		s->index = index++;
		s->tx_controlee_index = -1;
		s->ranging_index = 0;
		s->tx_ant_set = current_ms_step->tx_ant_set_nonranging;
		s->message_id = FIRA_MESSAGE_ID_MEASUREMENT_REPORT;

		s++;
		if (session->params.result_report_phase) {
			for (i = 0, j = 0; i < controlees_array->size; i++) {
				if (controlees_array->data[i].state !=
				    FIRA_CONTROLEE_STATE_RUNNING)
					continue;
				s->index = index++;
				s->tx_controlee_index = i;
				s->ranging_index = j++;
				s->rx_ant_set =
					current_ms_step->rx_ant_set_nonranging;
				s->message_id = FIRA_MESSAGE_ID_RESULT_REPORT;
				s++;
			}
		}
	}

	access->n_frames = index;

	frame_dtu = access->timestamp_dtu;

	for (i = 0; i < access->n_frames; i++) {
		bool is_tx;

		s = &local->slots[i];
		frame = &local->frames[i];
		sts_params = &local->sts_params[i];

		is_tx = s->tx_controlee_index == -1;

		fira_access_setup_frame(local, session, frame, sts_params, s,
					frame_dtu, is_tx);

		frame_dtu += session->params.slot_duration_dtu;
	}
	access->duration_dtu = frame_dtu - access->timestamp_dtu;

	return access;
}

static struct mcps802154_access *
fira_access_controlee(struct fira_local *local, struct fira_session *session)
{
	struct mcps802154_access *access;
	struct mcps802154_access_frame *frame;
	struct fira_slot *s;
	struct fira_ranging_info *ri;
	int timeout_dtu;

	access = &local->access;
	local->src_short_addr =
		fira_access_set_short_address(local, session, access);
	local->dst_short_addr = session->params.controller_short_addr;

	access->method = MCPS802154_ACCESS_METHOD_MULTI;
	access->ops = &fira_access_ops;
	access->timestamp_dtu = session->last_access_timestamp_dtu;
	access->duration_dtu = session->last_access_duration_dtu;
	access->frames = local->frames;
	access->n_frames = 1;
	access->channel = fira_access_channel(local, session);

	ri = local->ranging_info;
	memset(ri, 0, sizeof(*ri));
	ri->short_addr = session->params.controller_short_addr;
	local->n_ranging_info = 1;
	local->n_stopped_controlees_short_addr = 0;

	s = local->slots;
	s->index = 0;
	s->tx_controlee_index = -1;
	s->ranging_index = 0;
	s->rx_ant_set = fira_session_get_current_meas_seq_step(session)
				->rx_ant_set_nonranging;
	s->message_id = FIRA_MESSAGE_ID_CONTROL;

	if (session->synchronised)
		timeout_dtu = 2 * fira_session_get_block_duration_margin(
					  local, session);
	else if (!access->duration_dtu)
		timeout_dtu = -1;
	else
		timeout_dtu = access->duration_dtu -
			      session->params.round_duration_slots *
				      session->params.slot_duration_dtu;

	frame = local->frames;
	*frame = (struct mcps802154_access_frame){
		.is_tx = false,
		.rx = {
			.info = {
				.timestamp_dtu = access->timestamp_dtu,
				.timeout_dtu = timeout_dtu,
				.flags = MCPS802154_RX_INFO_TIMESTAMP_DTU |
					MCPS802154_RX_INFO_RANGING_ROUND,
				.ant_set_id = s->rx_ant_set,
			},
			.frame_info_flags_request
				= MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU,
		},
	};

	return access;
}

struct mcps802154_access *fira_get_access(struct mcps802154_region *region,
					  u32 next_timestamp_dtu,
					  int next_in_region_dtu,
					  int region_duration_dtu)
{
	struct fira_local *local = region_to_local(region);
	struct fira_session *session;

	if (!local->current_session) {
		session = fira_session_next(
			local, next_timestamp_dtu + local->llhw->anticip_dtu,
			region_duration_dtu - next_in_region_dtu);
		local->current_session = session;
	} else {
		session = local->current_session;
	}

	if (!session)
		return NULL;
	return fira_compute_access(local, session);
}

struct mcps802154_access *fira_compute_access(struct fira_local *local,
					      struct fira_session *session)
{
	fira_session_prepare(session);
	if (session->params.device_type == FIRA_DEVICE_TYPE_CONTROLLER)
		return fira_access_controller(local, session);
	else
		return fira_access_controlee(local, session);
}

void fira_session_get_demand(struct fira_local *local,
			     struct fira_session *session,
			     struct mcps802154_region_demand *demand)
{
	u32 base_timestamp_dtu = session->block_start_dtu +
				 fira_session_get_round_slot(session) *
					 session->params.slot_duration_dtu;
	if (session->params.device_type == FIRA_DEVICE_TYPE_CONTROLLER) {
		demand->timestamp_dtu = base_timestamp_dtu;
		demand->max_duration_dtu =
			(4 + 2 * session->current_controlees.size) *
			session->params.slot_duration_dtu;
	} else {
		int block_duration_margin_dtu =
			fira_session_get_block_duration_margin(local, session);
		demand->timestamp_dtu =
			base_timestamp_dtu - block_duration_margin_dtu;
		demand->max_duration_dtu =
			session->params.round_duration_slots *
				session->params.slot_duration_dtu +
			block_duration_margin_dtu;
	}
}
