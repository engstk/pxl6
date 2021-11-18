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

#include "fira_frame.h"
#include "fira_session.h"

#include <asm/unaligned.h>
#include <linux/bitfield.h>
#include <linux/errno.h>
#include <linux/ieee802154.h>
#include <linux/math64.h>
#include <net/af_ieee802154.h>
#include <net/mcps802154_frame.h>

#include "warn_return.h"

#define FIRA_IE_VENDOR_OUI_LEN 3
#define FIRA_IE_HEADER_PADDING_LEN 8
#define FIRA_IE_HEADER_SESSION_ID_LEN 4
#define FIRA_IE_HEADER_STS_INDEX_LEN 4
#define FIRA_IE_HEADER_LEN                                     \
	(FIRA_IE_VENDOR_OUI_LEN + FIRA_IE_HEADER_PADDING_LEN + \
	 FIRA_IE_HEADER_SESSION_ID_LEN + FIRA_IE_HEADER_STS_INDEX_LEN)

#define FIRA_IE_PAYLOAD_CONTROL_LEN(n_mngt) \
	(FIRA_IE_VENDOR_OUI_LEN + 4 + 4 * (n_mngt))
#define FIRA_IE_PAYLOAD_MEASUREMENT_REPORT_LEN(round_index_present,   \
					       n_reply_time)          \
	(FIRA_IE_VENDOR_OUI_LEN + 2 + 2 * (round_index_present) + 4 + \
	 6 * (n_reply_time))
#define FIRA_IE_PAYLOAD_RESULT_REPORT_LEN(tof_present, aoa_azimuth_present, \
					  aoa_elevation_present,            \
					  aoa_fom_present)                  \
	(FIRA_IE_VENDOR_OUI_LEN + 2 + 4 * (tof_present) +                   \
	 2 * (aoa_azimuth_present) + 2 * (aoa_elevation_present) +          \
	 (aoa_fom_present) *                                                \
		 (1 * (aoa_azimuth_present) + 1 * (aoa_elevation_present)))

#define FIRA_IE_VENDOR_OUI 0x5a18ff
#define FIRA_IE_HEADER_PADDING 0x08

#define FIRA_MNGT_RANGING_ROLE (1 << 0)
#define FIRA_MNGT_SLOT_INDEX (0xff << 1)
#define FIRA_MNGT_SHORT_ADDR (0xffff << 9)
#define FIRA_MNGT_MESSAGE_ID (0xf << 25)
#define FIRA_MNGT_STOP (1 << 29)
#define FIRA_MNGT_RESERVED (0x3 << 30)

#define FIRA_MEASUREMENT_REPORT_CONTROL_HOPPING_MODE (1 << 0)
#define FIRA_MEASUREMENT_REPORT_CONTROL_ROUND_INDEX_PRESENT (1 << 1)
#define FIRA_MEASUREMENT_REPORT_CONTROL_N_REPLY_TIME (0x3f << 2)

#define FIRA_RESULT_REPORT_CONTROL_TOF_PRESENT (1 << 0)
#define FIRA_RESULT_REPORT_CONTROL_AOA_AZIMUTH_PRESENT (1 << 1)
#define FIRA_RESULT_REPORT_CONTROL_AOA_ELEVATION_PRESENT (1 << 2)
#define FIRA_RESULT_REPORT_CONTROL_AOA_FOM_PRESENT (1 << 3)

void fira_frame_header_put(const struct fira_local *local,
			   const struct fira_slot *slot, struct sk_buff *skb)
{
	const struct fira_session *session = local->current_session;
	u16 fc = (IEEE802154_FC_TYPE_DATA | IEEE802154_FC_SECEN |
		  IEEE802154_FC_INTRA_PAN | IEEE802154_FC_NO_SEQ |
		  (IEEE802154_ADDR_SHORT << IEEE802154_FC_DAMODE_SHIFT) |
		  (2 << IEEE802154_FC_VERSION_SHIFT) |
		  (IEEE802154_ADDR_NONE << IEEE802154_FC_SAMODE_SHIFT));
	u8 *p;
	int i;

	p = skb_put(skb, IEEE802154_FC_LEN + IEEE802154_SHORT_ADDR_LEN +
				 IEEE802154_SCF_LEN);
	put_unaligned_le16(fc, p);
	p += IEEE802154_FC_LEN;
	put_unaligned_le16(local->dst_short_addr, p);
	p += IEEE802154_SHORT_ADDR_LEN;
	*p = IEEE802154_SCF_NO_FRAME_COUNTER;

	mcps802154_ie_put_begin(skb);
	p = mcps802154_ie_put_header_ie(skb, IEEE802154_IE_HEADER_VENDOR_ID,
					FIRA_IE_HEADER_LEN);
	put_unaligned_le24(FIRA_IE_VENDOR_OUI, p);
	p += FIRA_IE_VENDOR_OUI_LEN;
	for (i = 0; i < FIRA_IE_HEADER_PADDING_LEN; i++)
		*p++ = FIRA_IE_HEADER_PADDING;
	put_unaligned_le32(session->id, p);
	p += FIRA_IE_HEADER_SESSION_ID_LEN;
	put_unaligned_le32(
		fira_session_get_round_sts_index(session) + slot->index, p);
}

static u8 *fira_frame_common_payload_put(struct sk_buff *skb, unsigned int len,
					 enum fira_message_id message_id)
{
	u8 *p;

	p = mcps802154_ie_put_payload_ie(skb, IEEE802154_IE_PAYLOAD_VENDOR_GID,
					 len);
	WARN_RETURN_ON(!p, NULL);

	put_unaligned_le24(FIRA_IE_VENDOR_OUI, p);
	p += FIRA_IE_VENDOR_OUI_LEN;
	*p++ = message_id;

	return p;
}

void fira_frame_control_payload_put(const struct fira_local *local,
				    const struct fira_slot *slot,
				    struct sk_buff *skb)
{
	const struct fira_session *session = local->current_session;
	int n_mngt;
	u8 *p;
	int i;

	n_mngt = local->access.n_frames - 1 +
		 local->n_stopped_controlees_short_addr;

	p = fira_frame_common_payload_put(skb,
					  FIRA_IE_PAYLOAD_CONTROL_LEN(n_mngt),
					  FIRA_MESSAGE_ID_CONTROL);

	*p++ = n_mngt;
	*p++ = 0;
	*p++ = session->next_block_stride_len;

	for (i = 0; i < local->access.n_frames - 1; i++) {
		const struct fira_slot *slot = &local->slots[i + 1];
		int initiator = slot->tx_controlee_index == -1;
		int slot_index = slot->index;
		__le16 short_addr =
			slot->tx_controlee_index == -1 ?
				local->src_short_addr :
				session->current_controlees
					.data[slot->tx_controlee_index]
					.short_addr;
		int message_id = slot->message_id;
		u32 mngt = FIELD_PREP(FIRA_MNGT_RANGING_ROLE, initiator) |
			   FIELD_PREP(FIRA_MNGT_SLOT_INDEX, slot_index) |
			   FIELD_PREP(FIRA_MNGT_SHORT_ADDR, short_addr) |
			   FIELD_PREP(FIRA_MNGT_MESSAGE_ID, message_id);
		put_unaligned_le32(mngt, p);
		p += sizeof(u32);
	}

	for (i = 0; i < local->n_stopped_controlees_short_addr; i++) {
		__le16 short_addr = local->stopped_controlees_short_addr[i];
		u32 mngt = FIELD_PREP(FIRA_MNGT_SHORT_ADDR, short_addr) |
			   FIELD_PREP(FIRA_MNGT_STOP, 1);
		put_unaligned_le32(mngt, p);
		p += sizeof(u32);
	}
}

void fira_frame_measurement_report_payload_put(const struct fira_local *local,
					       const struct fira_slot *slot,
					       struct sk_buff *skb)
{
	const struct fira_session *session = local->current_session;
	const struct fira_ranging_info *ranging_info =
		&local->ranging_info[slot->ranging_index];
	u8 *p;
	int hopping_mode = session->params.round_hopping;
	int round_index_present = 1;
	int n_reply_time = local->n_ranging_valid;
	int i;
	u32 first_round_trip_time;
	u32 reply_time;
	u64 initiation_rctu, response_rctu, final_rctu;

	p = fira_frame_common_payload_put(
		skb,
		FIRA_IE_PAYLOAD_MEASUREMENT_REPORT_LEN(round_index_present,
						       n_reply_time),
		FIRA_MESSAGE_ID_MEASUREMENT_REPORT);

	*p++ = FIELD_PREP(FIRA_MEASUREMENT_REPORT_CONTROL_HOPPING_MODE,
			  hopping_mode) |
	       FIELD_PREP(FIRA_MEASUREMENT_REPORT_CONTROL_ROUND_INDEX_PRESENT,
			  round_index_present) |
	       FIELD_PREP(FIRA_MEASUREMENT_REPORT_CONTROL_N_REPLY_TIME,
			  n_reply_time);

	put_unaligned_le16(session->next_round_index, p);
	p += sizeof(u16);

	/* No handling for failed measurement, as there is only one, a failed
	 * measurement will cancel the ranging round.
	 * With several measurements, make sure a later measurement can still be
	 * done if an earlier one is failed. */
	initiation_rctu =
		ranging_info
			->timestamps_rctu[FIRA_MESSAGE_ID_RANGING_INITIATION];
	final_rctu =
		ranging_info->timestamps_rctu[FIRA_MESSAGE_ID_RANGING_FINAL];

	/* Retrieve first measurement. */
	for (i = 0; i < local->n_ranging_info; i++) {
		ranging_info = &local->ranging_info[i];
		if (!ranging_info->status)
			break;
	}
	/* Add first round trip measurement. */
	response_rctu =
		ranging_info->timestamps_rctu[FIRA_MESSAGE_ID_RANGING_RESPONSE];
	first_round_trip_time = mcps802154_difference_timestamp_rctu(
		local->llhw, response_rctu, initiation_rctu);
	put_unaligned_le32(first_round_trip_time, p);
	p += sizeof(u32);
	/* Retrieve reply measurement. */
	for (; i < local->n_ranging_info; i++) {
		ranging_info = &local->ranging_info[i];
		if (ranging_info->status)
			continue;
		put_unaligned_le16(ranging_info->short_addr, p);
		p += sizeof(u16);
		response_rctu = ranging_info->timestamps_rctu
					[FIRA_MESSAGE_ID_RANGING_RESPONSE];
		reply_time = mcps802154_difference_timestamp_rctu(
			local->llhw, final_rctu, response_rctu);
		put_unaligned_le32(reply_time, p);
		p += sizeof(u32);
	}
}

void fira_frame_result_report_payload_put(const struct fira_local *local,
					  const struct fira_slot *slot,
					  struct sk_buff *skb)
{
	const struct fira_session *session = local->current_session;
	const struct fira_session_params *params = &session->params;
	const struct fira_ranging_info *ranging_info =
		&local->ranging_info[slot->ranging_index];
	bool tof_present, aoa_azimuth_present, aoa_elevation_present,
		aoa_fom_present;
	u8 *p;

	tof_present = ranging_info->tof_present && params->report_tof;
	aoa_azimuth_present = ranging_info->local_aoa_azimuth.present &&
			      params->report_aoa_azimuth;
	aoa_elevation_present = ranging_info->local_aoa_elevation.present &&
				params->report_aoa_elevation;
	aoa_fom_present = (ranging_info->local_aoa_azimuth.aoa_fom ||
			   ranging_info->local_aoa_elevation.aoa_fom) &&
			  params->report_aoa_fom;

	p = fira_frame_common_payload_put(
		skb,
		FIRA_IE_PAYLOAD_RESULT_REPORT_LEN(
			tof_present, aoa_azimuth_present, aoa_elevation_present,
			aoa_fom_present),
		FIRA_MESSAGE_ID_RESULT_REPORT);

	*p++ = FIELD_PREP(FIRA_RESULT_REPORT_CONTROL_TOF_PRESENT, tof_present) |
	       FIELD_PREP(FIRA_RESULT_REPORT_CONTROL_AOA_AZIMUTH_PRESENT,
			  aoa_azimuth_present) |
	       FIELD_PREP(FIRA_RESULT_REPORT_CONTROL_AOA_ELEVATION_PRESENT,
			  aoa_elevation_present) |
	       FIELD_PREP(FIRA_RESULT_REPORT_CONTROL_AOA_FOM_PRESENT,
			  aoa_fom_present);

	if (tof_present) {
		put_unaligned_le32(
			ranging_info->tof_rctu > 0 ? ranging_info->tof_rctu : 0,
			p);
		p += sizeof(u32);
	}
	if (aoa_azimuth_present) {
		put_unaligned_le16(ranging_info->local_aoa_azimuth.aoa_2pi, p);
		p += sizeof(u16);
		if (aoa_fom_present) {
			*p = ranging_info->local_aoa_azimuth.aoa_fom;
			p++;
		}
	}
	if (aoa_elevation_present) {
		put_unaligned_le16(
			ranging_info->local_aoa_elevation.aoa_2pi * 2, p);
		p += sizeof(u16);
		if (aoa_fom_present) {
			*p = ranging_info->local_aoa_elevation.aoa_fom;
			p++;
		}
	}
}

void fira_frame_rframe_payload_put(struct fira_local *local,
				   struct sk_buff *skb)
{
	struct fira_session *session = local->current_session;
	struct fira_session_params *params = &local->current_session->params;
	u8 *p;

	if (params->data_payload_len == 0)
		return;

	p = mcps802154_ie_put_payload_ie(skb, IEEE802154_IE_PAYLOAD_VENDOR_GID,
					 FIRA_IE_VENDOR_OUI_LEN +
						 params->data_payload_len);
	WARN_RETURN_VOID_ON(!p);

	put_unaligned_le24(params->data_vendor_oui, p);
	p += FIRA_IE_VENDOR_OUI_LEN;
	memcpy(p, params->data_payload, params->data_payload_len);
	params->data_payload_len = 0;
	session->data_payload_seq_sent = params->data_payload_seq;
}

bool fira_frame_header_check(const struct fira_local *local,
			     struct sk_buff *skb,
			     struct mcps802154_ie_get_context *ie_get,
			     u32 *sts_index, u32 *session_id)
{
	u16 fc = (IEEE802154_FC_TYPE_DATA | IEEE802154_FC_SECEN |
		  IEEE802154_FC_INTRA_PAN | IEEE802154_FC_NO_SEQ |
		  IEEE802154_FC_IE_PRESENT |
		  (IEEE802154_ADDR_SHORT << IEEE802154_FC_DAMODE_SHIFT) |
		  (2 << IEEE802154_FC_VERSION_SHIFT) |
		  (IEEE802154_ADDR_NONE << IEEE802154_FC_SAMODE_SHIFT));
	bool fira_header_seen = false;
	int r;
	u8 *p;

	p = skb->data;
	if (!skb_pull(skb, IEEE802154_FC_LEN + IEEE802154_SHORT_ADDR_LEN +
				   IEEE802154_SCF_LEN) ||
	    get_unaligned_le16(p) != fc ||
	    !fira_aead_decrypt_scf_check(
		    p[IEEE802154_FC_LEN + IEEE802154_SHORT_ADDR_LEN]))
		return false;

	if (fira_aead_decrypt_prepare(skb))
		return false;

	for (r = mcps802154_ie_get(skb, ie_get); r == 0 && !ie_get->in_payload;
	     r = mcps802154_ie_get(skb, ie_get)) {
		p = skb->data;
		skb_pull(skb, ie_get->len);
		ie_get->mlme_len = 0;

		if (ie_get->id == IEEE802154_IE_HEADER_VENDOR_ID &&
		    ie_get->len >= FIRA_IE_VENDOR_OUI_LEN) {
			u32 vendor;

			vendor = get_unaligned_le24(p);
			p += FIRA_IE_VENDOR_OUI_LEN;
			if (vendor != FIRA_IE_VENDOR_OUI)
				continue;
			if (fira_header_seen)
				return false;
			if (ie_get->len != FIRA_IE_HEADER_LEN)
				return false;
			p += FIRA_IE_HEADER_PADDING_LEN;
			*session_id = get_unaligned_le32(p);
			p += FIRA_IE_HEADER_SESSION_ID_LEN;
			*sts_index = get_unaligned_le32(p);
			p += FIRA_IE_HEADER_STS_INDEX_LEN;
			fira_header_seen = true;
		}
	}

	return r >= 0 && fira_header_seen;
}

static bool fira_frame_control_read(struct fira_local *local, u8 *p,
				    unsigned int ie_len, unsigned int *n_slots,
				    bool *stop, int *block_stride_len)
{
	const struct fira_session *session = local->current_session;
	struct fira_slot *slot, last;
	int n_mngt, i;
	u16 msg_ids = 0;
	bool stop_found = false;

	n_mngt = *p++;
	if (ie_len < FIRA_IE_PAYLOAD_CONTROL_LEN(n_mngt))
		return false;
	p++;

	*block_stride_len = *p++;

	slot = local->slots;
	last = *slot++;
	for (i = 0; i < n_mngt; i++) {
		u32 mngt;
		bool initiator;
		int slot_index;
		__le16 short_addr;
		enum fira_message_id message_id;
		bool stop_ranging;

		mngt = get_unaligned_le32(p);
		p += sizeof(u32);

		initiator = !!(mngt & FIRA_MNGT_RANGING_ROLE);
		slot_index = FIELD_GET(FIRA_MNGT_SLOT_INDEX, mngt);
		short_addr = FIELD_GET(FIRA_MNGT_SHORT_ADDR, mngt);
		message_id = FIELD_GET(FIRA_MNGT_MESSAGE_ID, mngt);
		stop_ranging = !!(mngt & FIRA_MNGT_STOP);

		if (stop_ranging) {
			if (short_addr == local->src_short_addr) {
				stop_found = true;
			}
			continue;
		}

		if (slot_index <= last.index ||
		    slot_index >= session->params.round_duration_slots)
			return false;
		if (initiator && short_addr == local->src_short_addr)
			return false;

		last.index = slot_index;
		if (message_id <= FIRA_MESSAGE_ID_MAX &&
		    (initiator || short_addr == local->src_short_addr)) {
			u16 msg_id = 1 << message_id;
			if (message_id == FIRA_MESSAGE_ID_CONTROL_UPDATE &&
			    !initiator)
				msg_id <<= 1;
			if (msg_id < msg_ids || msg_id & msg_ids)
				return false;
			msg_ids |= msg_id;
			if (slot == local->slots + FIRA_CONTROLEE_FRAMES_MAX)
				return false;
			if (!initiator)
				last.tx_controlee_index = 0;
			else
				last.tx_controlee_index = -1;
			last.ranging_index = 0;
			last.message_id = message_id;
			if (!initiator) {
				last.tx_ant = session->tx_ant;
			} else {
				if (message_id >= FIRA_MESSAGE_ID_RANGING_FINAL)
					last.rx_ant_pair =
						session->rx_ant_pair[1];
				else
					last.rx_ant_pair =
						session->rx_ant_pair[0];
			}
			*slot++ = last;
		}
	}
	*stop = stop_found;
	*n_slots = slot - local->slots;

	return true;
}

bool fira_frame_control_payload_check(struct fira_local *local,
				      struct sk_buff *skb,
				      struct mcps802154_ie_get_context *ie_get,
				      unsigned int *n_slots, bool *stop_ranging,
				      int *block_stride_len)
{
	bool fira_payload_seen = false;
	int r;
	u8 *p;

	for (r = mcps802154_ie_get(skb, ie_get); r == 0;
	     r = mcps802154_ie_get(skb, ie_get)) {
		p = skb->data;
		skb_pull(skb, ie_get->len);

		if (ie_get->id == IEEE802154_IE_PAYLOAD_VENDOR_GID &&
		    ie_get->len >= FIRA_IE_VENDOR_OUI_LEN) {
			u32 vendor;
			int message_id;

			vendor = get_unaligned_le24(p);
			p += FIRA_IE_VENDOR_OUI_LEN;
			if (vendor != FIRA_IE_VENDOR_OUI)
				continue;

			if (ie_get->len < FIRA_IE_PAYLOAD_CONTROL_LEN(0))
				return false;
			message_id = (*p++) & 0xf;
			if (message_id != FIRA_MESSAGE_ID_CONTROL)
				return false;

			if (fira_payload_seen)
				return false;

			if (!fira_frame_control_read(local, p, ie_get->len,
						     n_slots, stop_ranging,
						     block_stride_len))
				return false;

			fira_payload_seen = true;
		}
	}

	return r >= 0 && fira_payload_seen;
}

static bool
fira_frame_measurement_report_fill_ranging_info(struct fira_local *local,
						const struct fira_slot *slot,
						u8 *p, unsigned int ie_len)
{
	struct fira_session *session = local->current_session;
	struct fira_ranging_info *ranging_info =
		&local->ranging_info[slot->ranging_index];
	u8 control;
	bool hopping_mode, round_index_present;
	unsigned int n_reply_time;
	u32 remote_round_trip_rctu, remote_reply_rctu;
	u64 rx_initiation_rctu, tx_response_rctu, rx_final_rctu;
	u32 local_round_trip_rctu, local_reply_rctu;
	int tof_rctu, i;

	control = *p++;
	hopping_mode =
		!!(control & FIRA_MEASUREMENT_REPORT_CONTROL_HOPPING_MODE);
	round_index_present = !!(
		control & FIRA_MEASUREMENT_REPORT_CONTROL_ROUND_INDEX_PRESENT);
	n_reply_time = FIELD_GET(FIRA_MEASUREMENT_REPORT_CONTROL_N_REPLY_TIME,
				 control);
	if (ie_len < FIRA_IE_PAYLOAD_MEASUREMENT_REPORT_LEN(round_index_present,
							    n_reply_time))
		return false;

	session->hopping_sequence_generation = hopping_mode &&
					       !round_index_present;
	if (round_index_present) {
		int next_round_index;

		next_round_index = get_unaligned_le16(p);
		p += sizeof(u16);
		session->next_round_index = next_round_index;
	}

	/* Remote_round_trip = first_round_trip + first_reply - my_reply. */
	remote_round_trip_rctu = get_unaligned_le32(p);
	p += sizeof(u32);
	/* Add first_reply. */
	remote_round_trip_rctu += get_unaligned_le32(p + sizeof(u16));

	for (i = 0; i < n_reply_time; i++) {
		__le16 short_addr = get_unaligned_le16(p);
		p += sizeof(u16);
		if (local->src_short_addr == short_addr) {
			remote_reply_rctu = get_unaligned_le32(p);
			break;
		}
		p += sizeof(u32);
	}
	/* Reply time not found. */
	if (i == n_reply_time)
		return false;
	/* Subtract my_reply. */
	remote_round_trip_rctu -= remote_reply_rctu;

	rx_initiation_rctu =
		ranging_info
			->timestamps_rctu[FIRA_MESSAGE_ID_RANGING_INITIATION];
	tx_response_rctu =
		ranging_info->timestamps_rctu[FIRA_MESSAGE_ID_RANGING_RESPONSE];
	rx_final_rctu =
		ranging_info->timestamps_rctu[FIRA_MESSAGE_ID_RANGING_FINAL];
	local_reply_rctu = mcps802154_difference_timestamp_rctu(
		local->llhw, tx_response_rctu, rx_initiation_rctu);
	local_round_trip_rctu = mcps802154_difference_timestamp_rctu(
		local->llhw, rx_final_rctu, tx_response_rctu);
	tof_rctu =
		div64_s64((s64)remote_round_trip_rctu * local_round_trip_rctu -
				  (s64)remote_reply_rctu * local_reply_rctu,
			  (s64)remote_round_trip_rctu + local_round_trip_rctu +
				  remote_reply_rctu + local_reply_rctu);
	ranging_info->tof_rctu = tof_rctu > 0 ? tof_rctu : 0;
	ranging_info->tof_present = true;

	return true;
}

bool fira_frame_measurement_report_payload_check(
	struct fira_local *local, const struct fira_slot *slot,
	struct sk_buff *skb, struct mcps802154_ie_get_context *ie_get)
{
	bool fira_payload_seen = false;
	int r;
	u8 *p;

	for (r = mcps802154_ie_get(skb, ie_get); r == 0;
	     r = mcps802154_ie_get(skb, ie_get)) {
		p = skb->data;
		skb_pull(skb, ie_get->len);

		if (ie_get->id == IEEE802154_IE_PAYLOAD_VENDOR_GID &&
		    ie_get->len >= FIRA_IE_VENDOR_OUI_LEN) {
			u32 vendor;
			int message_id;

			vendor = get_unaligned_le24(p);
			p += FIRA_IE_VENDOR_OUI_LEN;
			if (vendor != FIRA_IE_VENDOR_OUI)
				continue;

			if (ie_get->len <
			    FIRA_IE_PAYLOAD_MEASUREMENT_REPORT_LEN(false, 0))
				return false;
			message_id = (*p++) & 0xf;
			if (message_id != FIRA_MESSAGE_ID_MEASUREMENT_REPORT)
				return false;

			if (fira_payload_seen)
				return false;

			if (!fira_frame_measurement_report_fill_ranging_info(
				    local, slot, p, ie_get->len))
				return false;

			fira_payload_seen = true;
		}
	}

	return r >= 0 && fira_payload_seen;
}

static bool
fira_frame_result_report_fill_ranging_info(struct fira_local *local,
					   const struct fira_slot *slot, u8 *p,
					   unsigned int ie_len)
{
	struct fira_ranging_info *ranging_info =
		&local->ranging_info[slot->ranging_index];
	u8 control;
	bool tof_present, aoa_azimuth_present, aoa_elevation_present,
		aoa_fom_present;

	control = *p++;
	tof_present = !!(control & FIRA_RESULT_REPORT_CONTROL_TOF_PRESENT);
	aoa_azimuth_present =
		!!(control & FIRA_RESULT_REPORT_CONTROL_AOA_AZIMUTH_PRESENT);
	aoa_elevation_present =
		!!(control & FIRA_RESULT_REPORT_CONTROL_AOA_ELEVATION_PRESENT);
	aoa_fom_present =
		!!(control & FIRA_RESULT_REPORT_CONTROL_AOA_FOM_PRESENT);
	if (ie_len < FIRA_IE_PAYLOAD_RESULT_REPORT_LEN(
			     tof_present, aoa_azimuth_present,
			     aoa_elevation_present, aoa_fom_present))
		return false;

	if (tof_present) {
		ranging_info->tof_present = true;
		ranging_info->tof_rctu = get_unaligned_le32(p);
		p += sizeof(u32);
	}
	if (aoa_azimuth_present) {
		ranging_info->remote_aoa_azimuth_present = true;
		ranging_info->remote_aoa_azimuth_2pi = get_unaligned_le16(p);
		p += sizeof(s16);
	}
	if (aoa_elevation_present) {
		ranging_info->remote_aoa_elevation_present = true;
		ranging_info->remote_aoa_elevation_pi = get_unaligned_le16(p);
		p += sizeof(s16);
	}
	if (aoa_fom_present) {
		ranging_info->remote_aoa_fom_present = true;
		if (aoa_azimuth_present)
			ranging_info->remote_aoa_azimuth_fom = *p++;
		if (aoa_elevation_present)
			ranging_info->remote_aoa_elevation_fom = *p++;
	}

	return true;
}

bool fira_frame_result_report_payload_check(
	struct fira_local *local, const struct fira_slot *slot,
	struct sk_buff *skb, struct mcps802154_ie_get_context *ie_get)
{
	bool fira_payload_seen = false;
	int r;
	u8 *p;

	for (r = mcps802154_ie_get(skb, ie_get); r == 0;
	     r = mcps802154_ie_get(skb, ie_get)) {
		p = skb->data;
		skb_pull(skb, ie_get->len);

		if (ie_get->id == IEEE802154_IE_PAYLOAD_VENDOR_GID &&
		    ie_get->len >= FIRA_IE_VENDOR_OUI_LEN) {
			u32 vendor;
			int message_id;

			vendor = get_unaligned_le24(p);
			p += FIRA_IE_VENDOR_OUI_LEN;
			if (vendor != FIRA_IE_VENDOR_OUI)
				continue;

			if (ie_get->len < FIRA_IE_PAYLOAD_RESULT_REPORT_LEN(
						  false, false, false, false))
				return false;
			message_id = (*p++) & 0xf;
			if (message_id != FIRA_MESSAGE_ID_RESULT_REPORT)
				return false;

			if (fira_payload_seen)
				return false;

			if (!fira_frame_result_report_fill_ranging_info(
				    local, slot, p, ie_get->len))
				return false;

			fira_payload_seen = true;
		}
	}

	return r >= 0 && fira_payload_seen;
}

bool fira_frame_rframe_payload_check(struct fira_local *local,
				     const struct fira_slot *slot,
				     struct sk_buff *skb,
				     struct mcps802154_ie_get_context *ie_get)
{
	struct fira_ranging_info *ranging_info =
		&local->ranging_info[slot->ranging_index];
	struct fira_session_params *params = &local->current_session->params;
	bool rframe_payload_seen = false;
	int r;
	u8 *p;

	for (r = mcps802154_ie_get(skb, ie_get); r == 0;
	     r = mcps802154_ie_get(skb, ie_get)) {
		p = skb->data;
		skb_pull(skb, ie_get->len);

		if (ie_get->id == IEEE802154_IE_PAYLOAD_VENDOR_GID &&
		    ie_get->len >= FIRA_IE_VENDOR_OUI_LEN) {
			u32 vendor;
			unsigned int data_len;

			vendor = get_unaligned_le24(p);
			p += FIRA_IE_VENDOR_OUI_LEN;
			if (vendor != params->data_vendor_oui)
				continue;

			if (ie_get->len < FIRA_IE_VENDOR_OUI_LEN + 1)
				continue;

			if (rframe_payload_seen)
				return false;

			data_len = ie_get->len - FIRA_IE_VENDOR_OUI_LEN;
			memcpy(&ranging_info->data_payload, p, data_len);
			ranging_info->data_payload_len = data_len;

			rframe_payload_seen = true;
		}
	}

	return r >= 0;
}

int fira_frame_encrypt(struct fira_local *local, const struct fira_slot *slot,
		       struct sk_buff *skb)
{
	struct fira_session *session = local->current_session;
	int header_len;

	/* No payload, can not fail. */
	header_len = mcps802154_ie_put_end(skb, false);
	WARN_RETURN_ON(header_len < 0, header_len);

	return fira_aead_encrypt(&session->crypto.aead, skb, header_len,
				 local->src_short_addr, slot->index);
}

int fira_frame_decrypt(struct fira_local *local, struct fira_session *session,
		       const struct fira_slot *slot, struct sk_buff *skb,
		       unsigned int header_len)
{
	__le16 src_short_addr;

	if (slot->tx_controlee_index == -1)
		src_short_addr = local->dst_short_addr;
	else
		src_short_addr = session->current_controlees
					 .data[slot->tx_controlee_index]
					 .short_addr;

	return fira_aead_decrypt(&session->crypto.aead, skb, header_len,
				 src_short_addr, slot->index);
}

int fira_frame_header_check_decrypt(struct fira_local *local,
				    const struct fira_slot *slot,
				    struct sk_buff *skb,
				    struct mcps802154_ie_get_context *ie_get,
				    u32 *allow_resync_sts_index,
				    struct fira_session **allow_resync_session)
{
	struct fira_session *session = local->current_session;
	u32 sts_index;
	u32 session_id;
	u8 *header;
	unsigned int header_len;
	bool active;

	header = skb->data;

	if (!fira_frame_header_check(local, skb, ie_get, &sts_index,
				     &session_id))
		return -EBADMSG;

	if (allow_resync_session && session_id != session->id) {
		session = fira_session_get(local, session_id, &active);
		if (!session ||
		    session->params.device_type != FIRA_DEVICE_TYPE_CONTROLEE ||
		    !active || session->synchronised)
			return -EBADMSG;
		*allow_resync_session = session;
	} else if (session_id != session->id) {
		return -EBADMSG;
	}

	if (allow_resync_sts_index) {
		*allow_resync_sts_index = sts_index - slot->index;
	} else if (sts_index !=
		   fira_session_get_round_sts_index(session) + slot->index) {
		return -EBADMSG;
	}

	header_len = skb->data - header;

	return fira_frame_decrypt(local, session, slot, skb, header_len);
}
