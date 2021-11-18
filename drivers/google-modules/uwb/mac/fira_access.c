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
 *
 * FiRa ranging, access.
 *
 */

#include "fira_access.h"
#include "fira_session.h"
#include "fira_frame.h"

#include <asm/unaligned.h>
#include <linux/bitops.h>
#include <linux/string.h>
#include <linux/ieee802154.h>
#include <net/mcps802154_frame.h>

#include "warn_return.h"
#include "utils_fixed_point.h"

#define FIRA_FRAME_MAX_SIZE 127

#define FIRA_STS_FOM_THRESHOLD 153

static void fira_update_antennas_id(struct fira_session *session);
static struct mcps802154_access *
fira_access_controlee(struct fira_local *local, struct fira_session *session);

/**
 * phase_to_rad_fp() - compute the angle(AoA) from phase(PDOA) with fixed-point.
 * @pdoa_rad_q11: phase of arrival in fixed-point.
 * @spacing_mm_q11: spacing between antenna in mm in fixed-point.
 *
 * Return: Angle of Arrival in fixed-point too.
 */
static s16 phase_to_rad_fp(s16 pdoa_rad_q11, int spacing_mm_q11)
{
	/**
	 * Speed of light in air.
	 * static const long long speed_of_light_m_per_s = 299702547ull;
	 * static const long long freq_hz = 6.5e9;
	 * Constant to amplify/decrease value A and B, and so decrease
	 * error added by fixed-point.
	 *  Through N, A and B are closed to INT16_MAX.
	 * static const int N = 762;
	 * static const long long num = N * K * speed_of_light_m_per_s;
	 * static const double dem = freq_hz * 2.0 * M_PI;
	 * static const s16 A = num / dem;
	 * s16 B = N * spacing_mm_q11 / 1000;
	 *
	 * To be sure to have a optimize code, inline A declaration.
	 * A = 11452
	 *
	 * A will change with CHAN index.
	 * B is calculated using given antenna spacing that come from the mcps
	 * driver from the calibration table.
	 */
	static const s16 A = 11452;
	s16 B = (s16)(762 * spacing_mm_q11 / 1000);
	s16 x = div_fp(mult_fp(pdoa_rad_q11, A), B);
	/* Saturate between -1 to +1 for asyn. */
	x = x > K ? K : x < -K ? -K : x;
	return asin_fp(x);
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
 * @is_rframe: true if a ranging frame.
 */
static void fira_access_setup_frame(struct fira_local *local,
				    struct fira_session *session,
				    struct mcps802154_access_frame *frame,
				    struct mcps802154_sts_params *sts_params,
				    const struct fira_slot *slot, u32 frame_dtu,
				    bool is_tx, bool is_rframe)
{
	const struct fira_session_params *params = &session->params;
	struct mcps802154_sts_params *sts_params_for_access = NULL;

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

		if (is_rframe) {
			struct fira_ranging_info *ranging_info;

			ranging_info =
				&local->ranging_info[slot->ranging_index];
			ranging_info->timestamps_rctu[slot->message_id] =
				mcps802154_tx_timestamp_dtu_to_rmarker_rctu(
					local->llhw, frame_dtu, slot->tx_ant);

			flags |= MCPS802154_TX_FRAME_RANGING |
				 MCPS802154_TX_FRAME_SP3;
		}
		*frame = (struct mcps802154_access_frame){
			.is_tx = true,
			.tx_frame_info = {
				.timestamp_dtu = frame_dtu,
				.flags = flags,
				.ant_id = slot->tx_ant,
			},
			.sts_params = sts_params_for_access,
		};
	} else {
		u8 flags = MCPS802154_RX_INFO_TIMESTAMP_DTU;
		u16 request = 0;

		if (is_rframe) {
			flags |= MCPS802154_RX_INFO_RANGING |
				 MCPS802154_RX_INFO_SP3;
			request =
				MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU |
				MCPS802154_RX_FRAME_INFO_RANGING_STS_FOM |
				(params->aoa_result_req ?
					 MCPS802154_RX_FRAME_INFO_RANGING_PDOA |
						 MCPS802154_RX_FRAME_INFO_RANGING_PDOA_FOM :
					 0);
		}
		*frame = (struct mcps802154_access_frame){
			.is_tx = false,
			.rx = {
				.info = {
					.timestamp_dtu = frame_dtu,
					.flags = flags,
					.ant_pair_id = slot->rx_ant_pair,
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

static void fira_rx_frame_ranging(struct fira_local *local,
				  const struct fira_slot *slot,
				  const struct mcps802154_rx_frame_info *info)
{
	const struct fira_session *session = local->current_session;
	const struct fira_session_params *params = &session->params;
	struct fira_ranging_info *ranging_info =
		&local->ranging_info[slot->ranging_index];

	bool timestamp_rctu_present;
	bool pdoa_info_present;

	if (!info || !fira_rx_sts_good(local, info)) {
		ranging_info->failed = true;
		return;
	}

	pdoa_info_present = info->flags & MCPS802154_RX_FRAME_INFO_RANGING_PDOA;
	timestamp_rctu_present = info->flags &
				 MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU;

	if (pdoa_info_present) {
		struct fira_local_aoa_info *local_aoa;
		bool pdoa_fom_info_present =
			info->flags & MCPS802154_RX_FRAME_INFO_RANGING_PDOA_FOM;
		s16 local_pdoa_q11 = info->ranging_pdoa_rad_q11;
		s16 local_aoa_q11 = phase_to_rad_fp(
			local_pdoa_q11, info->ranging_pdoa_spacing_mm_q11);
		if (params->rx_antenna_pair_azimuth == slot->rx_ant_pair) {
			local_aoa = &ranging_info->local_aoa_azimuth;
		} else if (params->rx_antenna_pair_elevation ==
			   slot->rx_ant_pair) {
			local_aoa = &ranging_info->local_aoa_elevation;
		} else {
			local_aoa = &ranging_info->local_aoa;
		}

		local_aoa->present = true;
		local_aoa->rx_ant_pair = slot->rx_ant_pair;
		local_aoa->pdoa_2pi = map_q11_to_2pi(local_pdoa_q11);
		local_aoa->aoa_2pi = map_q11_to_2pi(local_aoa_q11);
		if (pdoa_fom_info_present)
			local_aoa->aoa_fom = info->ranging_pdoa_fom;
	}

	if (timestamp_rctu_present) {
		ranging_info->timestamps_rctu[slot->message_id] =
			info->timestamp_rctu;
	} else {
		ranging_info->failed = true;
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
	int last_slot_index;
	int i;

	if (!info || !(info->flags & MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU))
		goto failed_without_info;
	if (fira_frame_header_check_decrypt(
		    local, slot, skb, &ie_get, &sts_index,
		    session->synchronised ? NULL : &allow_resync_session))
		goto failed;
	if (allow_resync_session) {
		session = local->current_session = allow_resync_session;
		fira_access_controlee(local, session);
	}

	if (!fira_frame_control_payload_check(local, skb, &ie_get, &n_slots))
		goto failed;

	fira_session_resync(local, session, sts_index, info->timestamp_dtu);

	last_slot_index = 0;
	for (i = 1; i < n_slots; i++) {
		const struct fira_slot *slot = &local->slots[i];
		struct mcps802154_access_frame *frame = &local->frames[i];
		struct mcps802154_sts_params *sts_params =
			&local->sts_params[i];
		bool is_tx, is_rframe;
		u32 frame_dtu;

		is_tx = slot->tx_controlee_index != -1;
		is_rframe = slot->message_id <= FIRA_MESSAGE_ID_RFRAME_MAX;
		frame_dtu = info->timestamp_dtu +
			    session->params.slot_duration_dtu * slot->index;
		last_slot_index = slot->index;

		fira_access_setup_frame(local, session, frame, sts_params, slot,
					frame_dtu, is_tx, is_rframe);
	}
	local->access.timestamp_dtu = info->timestamp_dtu;
	local->access.duration_dtu =
		session->params.slot_duration_dtu * (last_slot_index + 1);
	local->access.n_frames = n_slots;

	return;
failed:
	local->access.timestamp_dtu = info->timestamp_dtu;
	local->access.duration_dtu = session->params.slot_duration_dtu;
failed_without_info:
	ranging_info->failed = true;
}

static void fira_rx_frame_measurement_report(
	struct fira_local *local, const struct fira_slot *slot,
	struct sk_buff *skb, const struct mcps802154_rx_frame_info *info)
{
	struct fira_ranging_info *ranging_info =
		&local->ranging_info[slot->ranging_index];
	struct mcps802154_ie_get_context ie_get = {};

	if (!info)
		goto failed;
	if (fira_frame_header_check_decrypt(local, slot, skb, &ie_get, NULL,
					    NULL))
		goto failed;

	if (!fira_frame_measurement_report_payload_check(local, slot, skb,
							 &ie_get))
		goto failed;

	return;
failed:
	ranging_info->failed = true;
}

static void
fira_rx_frame_result_report(struct fira_local *local,
			    const struct fira_slot *slot, struct sk_buff *skb,
			    const struct mcps802154_rx_frame_info *info)
{
	struct fira_ranging_info *ranging_info =
		&local->ranging_info[slot->ranging_index];
	struct mcps802154_ie_get_context ie_get = {};

	if (!info)
		goto failed;
	if (fira_frame_header_check_decrypt(local, slot, skb, &ie_get, NULL,
					    NULL))
		goto failed;

	if (!fira_frame_result_report_payload_check(local, slot, skb, &ie_get))
		goto failed;

	return;
failed:
	ranging_info->failed = true;
}

static void fira_rx_frame(struct mcps802154_access *access, int frame_idx,
			  struct sk_buff *skb,
			  const struct mcps802154_rx_frame_info *info)
{
	struct fira_local *local = access_to_local(access);
	const struct fira_slot *slot = &local->slots[frame_idx];
	struct fira_session *session = local->current_session;

	switch (slot->message_id) {
	case FIRA_MESSAGE_ID_RANGING_INITIATION:
	case FIRA_MESSAGE_ID_RANGING_RESPONSE:
	case FIRA_MESSAGE_ID_RANGING_FINAL:
		fira_rx_frame_ranging(local, slot, info);
		break;
	case FIRA_MESSAGE_ID_CONTROL:
		fira_rx_frame_control(local, slot, skb, info);
		break;
	case FIRA_MESSAGE_ID_MEASUREMENT_REPORT:
		fira_rx_frame_measurement_report(local, slot, skb, info);
		break;
	case FIRA_MESSAGE_ID_RESULT_REPORT:
		fira_rx_frame_result_report(local, slot, skb, info);
		break;
	case FIRA_MESSAGE_ID_CONTROL_UPDATE:
		break;
	default:
		WARN_UNREACHABLE_DEFAULT();
	}

	if (skb)
		kfree_skb(skb);

	/* Controlee: Stop round on error.
	   Controller: Stop when all ranging fails. */
	if (local->ranging_info[slot->ranging_index].failed)
		if (session->params.device_type == FIRA_DEVICE_TYPE_CONTROLEE ||
		    (slot->message_id <= FIRA_MESSAGE_ID_RFRAME_MAX &&
		     --local->n_ranging_valid == 0))
			access->n_frames = frame_idx + 1;
}

static struct sk_buff *fira_tx_get_frame(struct mcps802154_access *access,
					 int frame_idx)
{
	struct fira_local *local = access_to_local(access);
	const struct fira_slot *slot = &local->slots[frame_idx];
	struct sk_buff *skb;

	if (slot->message_id <= FIRA_MESSAGE_ID_RFRAME_MAX)
		return NULL;

	skb = mcps802154_frame_alloc(local->llhw, FIRA_FRAME_MAX_SIZE,
				     GFP_KERNEL);
	WARN_RETURN_ON(!skb, NULL);

	fira_frame_header_put(local, slot, skb);

	switch (slot->message_id) {
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
			local->ranging_info[i].failed = true;
		}
	}
}

static void fira_access_done(struct mcps802154_access *access)
{
	struct fira_local *local = access_to_local(access);
	struct fira_session *session = local->current_session;

	if (!session)
		return;

	fira_session_access_done(local, session);

	local->current_session = NULL;
}

struct mcps802154_access_ops fira_access_ops = {
	.rx_frame = fira_rx_frame,
	.tx_get_frame = fira_tx_get_frame,
	.tx_return = fira_tx_return,
	.access_done = fira_access_done,
};

static struct mcps802154_access *fira_access_nothing(struct fira_local *local)
{
	struct mcps802154_access *access;

	access = &local->access;
	access->method = MCPS802154_ACCESS_METHOD_NOTHING;
	access->ops = &fira_access_ops;
	access->duration_dtu = 0;

	return access;
}

static int fira_next_in_bitfield_u8(u8 bitfield, int prev)
{
	u32 padded_bitfield = 1 << 16 | bitfield << 8 | bitfield;
	u32 rotated_bitfield = padded_bitfield >> (prev + 1);

	return (ffs(rotated_bitfield) + prev) % 8;
}

static void fira_update_antennas_id(struct fira_session *session)
{
	const struct fira_session_params *p = &session->params;

	session->tx_ant = fira_next_in_bitfield_u8(p->tx_antenna_selection,
						   session->tx_ant);

	switch (p->rx_antenna_switch) {
	default:
	case FIRA_RX_ANTENNA_SWITCH_BETWEEN_ROUND:
		/* Switch pairs between round. */
		session->rx_ant_pair[0] = session->rx_ant_pair[1] =
			fira_next_in_bitfield_u8(p->rx_antenna_selection,
						 session->rx_ant_pair[0]);
		break;
	case FIRA_RX_ANTENNA_SWITCH_DURING_ROUND:
		session->rx_ant_pair[0] = p->rx_antenna_pair_azimuth;
		session->rx_ant_pair[1] = p->rx_antenna_pair_elevation;
		break;
	case FIRA_RX_ANTENNA_SWITCH_TWO_RANGING:
		/* Switch from one ranging to another. */
		if (session->rx_ant_pair[0] == p->rx_antenna_pair_azimuth)
			session->rx_ant_pair[0] = p->rx_antenna_pair_elevation;
		else
			session->rx_ant_pair[0] = p->rx_antenna_pair_azimuth;
		session->rx_ant_pair[1] = session->rx_ant_pair[0];
		break;
	}
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
		&session->params.current_controlees;
	int i;
	u32 frame_dtu;
	int index = 0;

	local->src_short_addr = mcps802154_get_short_addr(local->llhw);
	local->dst_short_addr = (controlees_array->size == 1) ?
					controlees_array->data[0].short_addr :
					IEEE802154_ADDR_SHORT_BROADCAST;

	access = &local->access;
	access->method = MCPS802154_ACCESS_METHOD_MULTI;
	access->ops = &fira_access_ops;
	access->timestamp_dtu = session->block_start_dtu +
				fira_session_get_round_slot(session) *
					session->params.slot_duration_dtu;
	access->frames = local->frames;

	fira_update_antennas_id(session);
	local->n_ranging_info = controlees_array->size;
	local->n_ranging_valid = controlees_array->size;
	ri = local->ranging_info;

	memset(ri, 0, local->n_ranging_info * sizeof(*ri));

	s = local->slots;

	s->index = index++;
	s->tx_controlee_index = -1;
	s->ranging_index = 0;
	s->tx_ant = session->tx_ant;
	s->message_id = FIRA_MESSAGE_ID_CONTROL;

	s++;
	s->index = index++;
	s->tx_controlee_index = -1;
	s->ranging_index = 0;
	s->tx_ant = session->tx_ant;
	s->message_id = FIRA_MESSAGE_ID_RANGING_INITIATION;

	s++;
	for (i = 0; i < local->n_ranging_info; i++) {
		ri->short_addr = controlees_array->data[i].short_addr;
		/* Requested in fira_report_aoa function. */
		ri++;
		s->index = index++;
		s->tx_controlee_index = i;
		s->ranging_index = i;
		s->rx_ant_pair = session->rx_ant_pair[0];
		s->message_id = FIRA_MESSAGE_ID_RANGING_RESPONSE;
		s++;
	}
	s->index = index++;
	s->tx_controlee_index = -1;
	s->ranging_index = 0;
	s->tx_ant = session->tx_ant;
	s->message_id = FIRA_MESSAGE_ID_RANGING_FINAL;

	s++;
	s->index = index++;
	s->tx_controlee_index = -1;
	s->ranging_index = 0;
	s->tx_ant = session->tx_ant;
	s->message_id = FIRA_MESSAGE_ID_MEASUREMENT_REPORT;

	s++;
	for (i = 0; i < local->n_ranging_info; i++) {
		s->index = index++;
		s->tx_controlee_index = i;
		s->ranging_index = i;
		s->rx_ant_pair = session->rx_ant_pair[0];
		s->message_id = FIRA_MESSAGE_ID_RESULT_REPORT;
		s++;
	}
	access->n_frames = index;

	frame_dtu = access->timestamp_dtu;

	for (i = 0; i < access->n_frames; i++) {
		bool is_tx, is_rframe;

		s = &local->slots[i];
		frame = &local->frames[i];
		sts_params = &local->sts_params[i];

		is_tx = s->tx_controlee_index == -1;
		is_rframe = s->message_id <= FIRA_MESSAGE_ID_RFRAME_MAX;

		fira_access_setup_frame(local, session, frame, sts_params, s,
					frame_dtu, is_tx, is_rframe);

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

	local->src_short_addr = mcps802154_get_short_addr(local->llhw);
	local->dst_short_addr = session->params.controller_short_addr;

	access = &local->access;
	access->method = MCPS802154_ACCESS_METHOD_MULTI;
	access->ops = &fira_access_ops;
	access->timestamp_dtu = session->last_access_timestamp_dtu;
	access->duration_dtu = session->last_access_duration_dtu;
	access->frames = local->frames;
	access->n_frames = 1;
	fira_update_antennas_id(session);

	ri = local->ranging_info;
	memset(ri, 0, sizeof(*ri));
	ri->short_addr = session->params.controller_short_addr;
	local->n_ranging_info = 1;

	s = local->slots;
	s->index = 0;
	s->tx_controlee_index = -1;
	s->ranging_index = 0;
	s->rx_ant_pair = session->rx_ant_pair[0];
	s->message_id = FIRA_MESSAGE_ID_CONTROL;

	frame = local->frames;
	*frame = (struct mcps802154_access_frame){
		.is_tx = false,
		.rx = {
			.info = {
				.timestamp_dtu = access->timestamp_dtu,
				.timeout_dtu = access->duration_dtu ? access->duration_dtu : -1,
				.flags = MCPS802154_RX_INFO_TIMESTAMP_DTU,
				.ant_pair_id = s->rx_ant_pair,
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

	session = fira_session_next(local, next_timestamp_dtu +
						   local->llhw->anticip_dtu);
	local->current_session = session;

	if (!session)
		return fira_access_nothing(local);
	return fira_compute_access(local, session);
}

struct mcps802154_access *fira_compute_access(struct fira_local *local,
					      struct fira_session *session)
{
	if (session->params.device_type == FIRA_DEVICE_TYPE_CONTROLLER)
		return fira_access_controller(local, session);
	else
		return fira_access_controlee(local, session);
}

void fira_get_demand(struct fira_local *local, struct fira_session *session,
		     struct mcps802154_region_demand *demand)
{
	demand->timestamp_dtu = session->block_start_dtu +
				fira_session_get_round_slot(session) *
					session->params.slot_duration_dtu;
	if (session->params.device_type == FIRA_DEVICE_TYPE_CONTROLLER)
		demand->duration_dtu =
			(4 + 2 * session->params.current_controlees.size) *
			session->params.slot_duration_dtu;
	else
		/* Min size for DSTWR. */
		demand->duration_dtu = 6 * session->params.slot_duration_dtu;
}
