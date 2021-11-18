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

#include <asm/unaligned.h>
#include <linux/errno.h>
#include <linux/ieee802154.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/limits.h>
#include <net/af_ieee802154.h>

#include <net/mcps802154_schedule.h>
#include <net/mcps802154_frame.h>
#include <net/simple_ranging_region_nl.h>

#include "nl.h"
#include "warn_return.h"

#define TWR_SLOT_MS_TO_RCTU 67108864ull
#define TWR_SLOT_MS_MAX 64
#define TWR_SLOT_DEFAULT_RCTU (16 * TWR_SLOT_MS_TO_RCTU)

#define TWR_FUNCTION_CODE_POLL 0x40
#define TWR_FUNCTION_CODE_RESP 0x41
#define TWR_FUNCTION_CODE_FINAL 0x42
#define TWR_FUNCTION_CODE_REPORT 0x43

enum twr_frames {
	TWR_FRAME_POLL,
	TWR_FRAME_RESP,
	TWR_FRAME_FINAL,
	TWR_FRAME_REPORT,
	N_TWR_FRAMES,
};

struct simple_ranging_initiator {
	u64 poll_tx_timestamp_rctu;
	u64 final_tx_timestamp_rctu;
	int tof_half_tag_rctu;
	int local_pdoa_rad_q11;
	s16 remote_pdoa_rad_q11;
	int local_pdoa_elevation_rad_q11;
	s16 remote_pdoa_elevation_rad_q11;
};

struct simple_ranging_responder {
	u64 poll_rx_timestamp_rctu;
	u64 resp_tx_timestamp_rctu;
	s16 local_pdoa_rad_q11;
	s16 local_pdoa_elevation_rad_q11;
	int tof_x4_rctu;
};

struct simple_ranging_local {
	struct mcps802154_scheduler scheduler;
	struct mcps802154_region region_init;
	struct mcps802154_region region_resp;
	struct mcps802154_llhw *llhw;
	struct mcps802154_access access;
	struct mcps802154_access_frame frames[N_TWR_FRAMES];
	struct mcps802154_sts_params sts_params;
	struct mcps802154_nl_ranging_request
		requests[MCPS802154_NL_RANGING_REQUESTS_MAX];
	unsigned int n_requests;
	unsigned int request_idx;
	int frequency_hz;
	struct mcps802154_nl_ranging_request current_request;
	int slot_duration_dtu;
	bool is_responder;
	unsigned int tx_ant;
	unsigned int rx_ant_pair_azimuth;
	unsigned int rx_ant_pair_elevation;
	bool is_same_rx_ant;
	union {
		struct simple_ranging_initiator initiator;
		struct simple_ranging_responder responder;
	};
};

static inline struct simple_ranging_local *
scheduler_to_local(struct mcps802154_scheduler *scheduler)
{
	return container_of(scheduler, struct simple_ranging_local, scheduler);
}

static inline struct simple_ranging_local *
access_to_local(struct mcps802154_access *access)
{
	return container_of(access, struct simple_ranging_local, access);
}

static inline struct simple_ranging_local *
region_init_to_local(struct mcps802154_region *region_init)
{
	return container_of(region_init, struct simple_ranging_local,
			    region_init);
}

static inline struct simple_ranging_local *
region_resp_to_local(struct mcps802154_region *region_resp)
{
	return container_of(region_resp, struct simple_ranging_local,
			    region_resp);
}

/* Requests and reports. */

static void twr_requests_clear(struct simple_ranging_local *local)
{
	local->n_requests = 0;
	local->request_idx = 0;
	local->frequency_hz = 1;
}

static void twr_request_start(struct simple_ranging_local *local)
{
	if (local->request_idx >= local->n_requests)
		local->request_idx = 0;
	local->current_request = local->requests[local->request_idx];
}

static void twr_report(struct simple_ranging_local *local,
		       const struct mcps802154_nl_ranging_report *report)
{
	struct mcps802154_nl_ranging_request *request = &local->current_request;
	struct mcps802154_nl_ranging_report invalid = {
		.tof_rctu = INT_MIN,
		.local_pdoa_rad_q11 = INT_MIN,
		.remote_pdoa_rad_q11 = INT_MIN,
		.local_pdoa_elevation_rad_q11 = INT_MIN,
		.remote_pdoa_elevation_rad_q11 = INT_MIN,
		.is_same_rx_ant = local->is_same_rx_ant,
	};
	int r;

	if (!report)
		report = &invalid;

	r = mcps802154_nl_ranging_report(local->llhw, request->id, report);

	if (r == -ECONNREFUSED)
		twr_requests_clear(local);
	else
		local->request_idx++;
}

/* Frames. */

#define TWR_FRAME_HEADER_SIZE                                             \
	(IEEE802154_FC_LEN + IEEE802154_SEQ_LEN + IEEE802154_PAN_ID_LEN + \
	 IEEE802154_EXTENDED_ADDR_LEN * 2)
#define TWR_FRAME_POLL_SIZE 1
#define TWR_FRAME_RESP_SIZE 3
#define TWR_FRAME_FINAL_SIZE 5
#define TWR_FRAME_REPORT_SIZE 7
#define TWR_FRAME_MAX_SIZE (TWR_FRAME_HEADER_SIZE + TWR_FRAME_REPORT_SIZE)

void twr_frame_header_fill_buf(u8 *buf, __le16 pan_id, __le64 dst, __le64 src)
{
	u16 fc = (IEEE802154_FC_TYPE_DATA | IEEE802154_FC_INTRA_PAN |
		  (IEEE802154_ADDR_LONG << IEEE802154_FC_DAMODE_SHIFT) |
		  (1 << IEEE802154_FC_VERSION_SHIFT) |
		  (IEEE802154_ADDR_LONG << IEEE802154_FC_SAMODE_SHIFT));
	u8 seq = 0;
	size_t pos = 0;

	put_unaligned_le16(fc, buf + pos);
	pos += IEEE802154_FC_LEN;
	buf[pos] = seq;
	pos += IEEE802154_SEQ_LEN;
	memcpy(buf + pos, &pan_id, sizeof(pan_id));
	pos += IEEE802154_PAN_ID_LEN;
	memcpy(buf + pos, &dst, sizeof(dst));
	pos += IEEE802154_EXTENDED_ADDR_LEN;
	memcpy(buf + pos, &src, sizeof(src));
}

void twr_frame_header_put(struct sk_buff *skb, __le16 pan_id, __le64 dst,
			  __le64 src)
{
	twr_frame_header_fill_buf(skb_put(skb, TWR_FRAME_HEADER_SIZE), pan_id,
				  dst, src);
}

bool twr_frame_header_check(struct sk_buff *skb, __le16 pan_id, __le64 dst,
			    __le64 src)
{
	u8 buf[TWR_FRAME_HEADER_SIZE];

	if (!pskb_may_pull(skb, TWR_FRAME_HEADER_SIZE))
		return false;
	twr_frame_header_fill_buf(buf, pan_id, dst, src);
	if (memcmp(skb->data, buf, TWR_FRAME_HEADER_SIZE) != 0)
		return false;
	return true;
}

bool twr_frame_header_check_no_src(struct sk_buff *skb, __le16 pan_id,
				   __le64 dst)
{
	/* Ignore source. */
	const int check_size =
		TWR_FRAME_HEADER_SIZE - IEEE802154_EXTENDED_ADDR_LEN;
	u8 buf[TWR_FRAME_HEADER_SIZE];

	if (!pskb_may_pull(skb, TWR_FRAME_HEADER_SIZE))
		return false;
	twr_frame_header_fill_buf(buf, pan_id, dst, 0);
	if (memcmp(skb->data, buf, check_size) != 0)
		return false;
	return true;
}

void twr_frame_poll_put(struct sk_buff *skb)
{
	u8 function_code = TWR_FUNCTION_CODE_POLL;

	skb_put_u8(skb, function_code);
}

bool twr_frame_poll_check(struct sk_buff *skb)
{
	u8 function_code;

	if (!pskb_may_pull(skb, TWR_FRAME_POLL_SIZE))
		return false;
	function_code = skb->data[0];
	if (function_code != TWR_FUNCTION_CODE_POLL)
		return false;
	return true;
}

void twr_frame_resp_put(struct sk_buff *skb, s16 local_pdoa_rad_q11)
{
	u8 function_code = TWR_FUNCTION_CODE_RESP;

	skb_put_u8(skb, function_code);
	put_unaligned_le16(local_pdoa_rad_q11, skb_put(skb, 2));
}

bool twr_frame_resp_check(struct sk_buff *skb, s16 *remote_pdoa_rad_q11)
{
	u8 function_code;

	if (!pskb_may_pull(skb, TWR_FRAME_RESP_SIZE))
		return false;
	function_code = skb->data[0];
	if (function_code != TWR_FUNCTION_CODE_RESP)
		return false;
	*remote_pdoa_rad_q11 = get_unaligned_le16(skb->data + 1);
	return true;
}

void twr_frame_final_put(struct sk_buff *skb, s32 tof_half_tag_rctu)
{
	u8 function_code = TWR_FUNCTION_CODE_FINAL;

	skb_put_u8(skb, function_code);
	put_unaligned_le32(tof_half_tag_rctu, skb_put(skb, 4));
}

bool twr_frame_final_check(struct sk_buff *skb, s32 *tof_half_tag_rctu)
{
	u8 function_code;

	if (!pskb_may_pull(skb, TWR_FRAME_FINAL_SIZE))
		return false;
	function_code = skb->data[0];
	if (function_code != TWR_FUNCTION_CODE_FINAL)
		return false;
	*tof_half_tag_rctu = get_unaligned_le32(skb->data + 1);
	return true;
}

void twr_frame_report_put(struct sk_buff *skb, s32 tof_x4_rctu,
			  s16 local_pdoa_rad_q11)
{
	u8 function_code = TWR_FUNCTION_CODE_REPORT;

	skb_put_u8(skb, function_code);
	put_unaligned_le32(tof_x4_rctu, skb_put(skb, 4));
	put_unaligned_le16(local_pdoa_rad_q11, skb_put(skb, 2));
}

bool twr_frame_report_check(struct sk_buff *skb, s32 *tof_x4_rctu,
			    s16 *remote_pdoa_rad_q11)
{
	u8 function_code;

	if (!pskb_may_pull(skb, TWR_FRAME_REPORT_SIZE))
		return false;
	function_code = skb->data[0];
	if (function_code != TWR_FUNCTION_CODE_REPORT)
		return false;
	*tof_x4_rctu = get_unaligned_le32(skb->data + 1);
	*remote_pdoa_rad_q11 = get_unaligned_le16(skb->data + 5);
	return true;
}

/* Access responder. */

static void twr_responder_rx_frame(struct mcps802154_access *access,
				   int frame_idx, struct sk_buff *skb,
				   const struct mcps802154_rx_frame_info *info,
				   enum mcps802154_rx_error_type error)
{
	struct simple_ranging_local *local = access_to_local(access);
	struct mcps802154_nl_ranging_request *request = &local->current_request;

	if (!skb)
		goto fail;

	if (frame_idx == TWR_FRAME_POLL) {
		u32 resp_tx_dtu;

		if (!twr_frame_header_check_no_src(
			    skb, mcps802154_get_pan_id(local->llhw),
			    mcps802154_get_extended_addr(local->llhw)))
			goto fail_free_skb;
		request->peer_extended_addr =
			get_unaligned_le64(skb->data + TWR_FRAME_HEADER_SIZE -
					   IEEE802154_EXTENDED_ADDR_LEN);
		skb_pull(skb, TWR_FRAME_HEADER_SIZE);

		if (!twr_frame_poll_check(skb))
			goto fail_free_skb;
		if (!(info->flags & MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU))
			goto fail_free_skb;
		if (!(info->flags & MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU))
			goto fail_free_skb;
		if (!(info->flags & MCPS802154_RX_FRAME_INFO_RANGING_PDOA))
			local->responder.local_pdoa_rad_q11 = S16_MIN;
		else
			local->responder.local_pdoa_rad_q11 =
				info->ranging_pdoa_rad_q11;
		local->responder.poll_rx_timestamp_rctu = info->timestamp_rctu;
		resp_tx_dtu = info->timestamp_dtu + local->slot_duration_dtu;
		local->responder.resp_tx_timestamp_rctu =
			mcps802154_tx_timestamp_dtu_to_rmarker_rctu(
				local->llhw, resp_tx_dtu, local->tx_ant);
		/* Set the timings for the next frames. */
		access->frames[TWR_FRAME_RESP].tx_frame_info.timestamp_dtu =
			resp_tx_dtu;
		access->frames[TWR_FRAME_FINAL].rx.info.timestamp_dtu =
			resp_tx_dtu + local->slot_duration_dtu;
		access->frames[TWR_FRAME_REPORT].tx_frame_info.timestamp_dtu =
			resp_tx_dtu + 2 * local->slot_duration_dtu;
	} else {
		s32 tof_half_rctu;
		u64 final_rx_timestamp_rctu;

		WARN_ON(frame_idx != TWR_FRAME_FINAL);
		if (!twr_frame_header_check(
			    skb, mcps802154_get_pan_id(local->llhw),
			    mcps802154_get_extended_addr(local->llhw),
			    request->peer_extended_addr))
			goto fail_free_skb;
		skb_pull(skb, TWR_FRAME_HEADER_SIZE);
		if (!twr_frame_final_check(skb, &tof_half_rctu))
			goto fail_free_skb;
		if (!(info->flags & MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU))
			goto fail_free_skb;
		if (!(info->flags & MCPS802154_RX_FRAME_INFO_RANGING_PDOA))
			local->responder.local_pdoa_elevation_rad_q11 = S16_MIN;
		else
			local->responder.local_pdoa_elevation_rad_q11 =
				info->ranging_pdoa_rad_q11;
		final_rx_timestamp_rctu = info->timestamp_rctu;
		local->responder.tof_x4_rctu =
			tof_half_rctu -
			mcps802154_difference_timestamp_rctu(
				local->llhw,
				local->responder.resp_tx_timestamp_rctu,
				local->responder.poll_rx_timestamp_rctu) +
			mcps802154_difference_timestamp_rctu(
				local->llhw, final_rx_timestamp_rctu,
				local->responder.resp_tx_timestamp_rctu);
	}

	kfree_skb(skb);
	return;
fail_free_skb:
	kfree_skb(skb);
fail:
	access->n_frames = frame_idx + 1;
}

static struct sk_buff *
twr_responder_tx_get_frame(struct mcps802154_access *access, int frame_idx)
{
	struct simple_ranging_local *local = access_to_local(access);
	struct mcps802154_nl_ranging_request *request = &local->current_request;
	struct sk_buff *skb = mcps802154_frame_alloc(
		local->llhw, TWR_FRAME_MAX_SIZE, GFP_KERNEL);

	twr_frame_header_put(skb, mcps802154_get_pan_id(local->llhw),
			     request->peer_extended_addr,
			     mcps802154_get_extended_addr(local->llhw));
	if (frame_idx == TWR_FRAME_RESP) {
		twr_frame_resp_put(skb, local->responder.local_pdoa_rad_q11);
	} else {
		WARN_ON(frame_idx != TWR_FRAME_REPORT);
		twr_frame_report_put(
			skb, local->responder.tof_x4_rctu,
			local->responder.local_pdoa_elevation_rad_q11);
	}
	return skb;
}

static void
twr_responder_tx_return(struct mcps802154_access *access, int frame_idx,
			struct sk_buff *skb,
			enum mcps802154_access_tx_return_reason reason)
{
	kfree_skb(skb);
}

struct mcps802154_access_ops twr_responder_access_ops = {
	.rx_frame = twr_responder_rx_frame,
	.tx_get_frame = twr_responder_tx_get_frame,
	.tx_return = twr_responder_tx_return,
};

/* Access initiator. */

static void twr_rx_frame(struct mcps802154_access *access, int frame_idx,
			 struct sk_buff *skb,
			 const struct mcps802154_rx_frame_info *info,
			 enum mcps802154_rx_error_type error)
{
	struct simple_ranging_local *local = access_to_local(access);
	struct mcps802154_nl_ranging_request *request = &local->current_request;
	u64 resp_rx_timestamp_rctu;
	struct mcps802154_nl_ranging_report report;

	if (!skb)
		goto fail;

	if (!twr_frame_header_check(skb, mcps802154_get_pan_id(local->llhw),
				    mcps802154_get_extended_addr(local->llhw),
				    request->peer_extended_addr))
		goto fail_free_skb;

	skb_pull(skb, TWR_FRAME_HEADER_SIZE);

	if (frame_idx == TWR_FRAME_RESP) {
		if (!twr_frame_resp_check(
			    skb, &local->initiator.remote_pdoa_rad_q11))
			goto fail_free_skb;
		if (!(info->flags & MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU))
			goto fail_free_skb;
		if (!(info->flags & MCPS802154_RX_FRAME_INFO_RANGING_PDOA))
			local->initiator.local_pdoa_rad_q11 = INT_MIN;
		else
			local->initiator.local_pdoa_rad_q11 =
				info->ranging_pdoa_rad_q11;

		resp_rx_timestamp_rctu = info->timestamp_rctu;
		local->initiator.tof_half_tag_rctu =
			mcps802154_difference_timestamp_rctu(
				local->llhw, resp_rx_timestamp_rctu,
				local->initiator.poll_tx_timestamp_rctu) -
			mcps802154_difference_timestamp_rctu(
				local->llhw,
				local->initiator.final_tx_timestamp_rctu,
				resp_rx_timestamp_rctu);
	} else {
		s32 report_tof_x4_rctu;

		WARN_ON(frame_idx != TWR_FRAME_REPORT);
		if (!twr_frame_report_check(
			    skb, &report_tof_x4_rctu,
			    &local->initiator.remote_pdoa_elevation_rad_q11))
			goto fail_free_skb;

		if (!(info->flags & MCPS802154_RX_FRAME_INFO_RANGING_PDOA))
			local->initiator.local_pdoa_elevation_rad_q11 = INT_MIN;
		else
			local->initiator.local_pdoa_elevation_rad_q11 =
				info->ranging_pdoa_rad_q11;

		report.tof_rctu = report_tof_x4_rctu / 4;
		report.local_pdoa_rad_q11 = local->initiator.local_pdoa_rad_q11;
		report.remote_pdoa_rad_q11 =
			local->initiator.remote_pdoa_rad_q11;
		report.local_pdoa_elevation_rad_q11 =
			local->initiator.local_pdoa_elevation_rad_q11;
		report.remote_pdoa_elevation_rad_q11 =
			local->initiator.remote_pdoa_elevation_rad_q11;
		report.is_same_rx_ant = local->is_same_rx_ant;

		twr_report(local, &report);
	}

	kfree_skb(skb);
	return;
fail_free_skb:
	kfree_skb(skb);
fail:
	twr_report(local, NULL);
	access->n_frames = frame_idx + 1;
}

static struct sk_buff *twr_tx_get_frame(struct mcps802154_access *access,
					int frame_idx)
{
	struct simple_ranging_local *local = access_to_local(access);
	struct mcps802154_nl_ranging_request *request = &local->current_request;
	struct sk_buff *skb = mcps802154_frame_alloc(
		local->llhw, TWR_FRAME_MAX_SIZE, GFP_KERNEL);

	twr_frame_header_put(skb, mcps802154_get_pan_id(local->llhw),
			     request->peer_extended_addr,
			     mcps802154_get_extended_addr(local->llhw));
	if (frame_idx == TWR_FRAME_POLL) {
		twr_frame_poll_put(skb);
	} else {
		WARN_ON(frame_idx != TWR_FRAME_FINAL);
		twr_frame_final_put(skb, local->initiator.tof_half_tag_rctu);
	}
	return skb;
}

static void twr_tx_return(struct mcps802154_access *access, int frame_idx,
			  struct sk_buff *skb,
			  enum mcps802154_access_tx_return_reason reason)
{
	kfree_skb(skb);
}

struct mcps802154_access_ops twr_access_ops = {
	.rx_frame = twr_rx_frame,
	.tx_get_frame = twr_tx_get_frame,
	.tx_return = twr_tx_return,
};

/* Region responder. */

static struct mcps802154_access *
twr_responder_get_access(struct mcps802154_region *region,
			 u32 next_timestamp_dtu, int next_in_region_dtu,
			 int region_duration_dtu)
{
	struct simple_ranging_local *local = region_resp_to_local(region);
	struct mcps802154_access *access = &local->access;
	u32 start_dtu = next_timestamp_dtu;

	access->method = MCPS802154_ACCESS_METHOD_MULTI;
	access->ops = &twr_responder_access_ops;
	access->n_frames = ARRAY_SIZE(local->frames);
	access->frames = local->frames;

	access->frames[TWR_FRAME_POLL] = (struct mcps802154_access_frame){
		.is_tx = false,
		.rx = {
			.info = {
				.timestamp_dtu = start_dtu,
				.timeout_dtu = -1,
				.flags = MCPS802154_RX_INFO_TIMESTAMP_DTU |
					MCPS802154_RX_INFO_RANGING |
					MCPS802154_RX_INFO_KEEP_RANGING_CLOCK |
					MCPS802154_RX_INFO_RANGING_PDOA |
					MCPS802154_RX_INFO_SP1,
				.ant_pair_id = local->rx_ant_pair_azimuth,
			},
			.frame_info_flags_request =
				MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU |
				MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU |
				MCPS802154_RX_FRAME_INFO_RANGING_PDOA,
		},
		.sts_params = &local->sts_params,
	};

	access->frames[TWR_FRAME_RESP] = (struct mcps802154_access_frame){
		.is_tx = true,
		.tx_frame_info = {
			.flags = MCPS802154_TX_FRAME_TIMESTAMP_DTU |
				MCPS802154_TX_FRAME_RANGING |
				MCPS802154_TX_FRAME_KEEP_RANGING_CLOCK |
				MCPS802154_TX_FRAME_SP1,
			.ant_id = local->tx_ant,
		},
	};

	access->frames[TWR_FRAME_FINAL] = (struct mcps802154_access_frame){
		.is_tx = false,
		.rx = {
			.info = {
				.flags = MCPS802154_RX_INFO_TIMESTAMP_DTU |
					MCPS802154_RX_INFO_RANGING |
					MCPS802154_RX_INFO_RANGING_PDOA |
					MCPS802154_RX_INFO_SP1,
				.ant_pair_id = local->rx_ant_pair_elevation,
			},
			.frame_info_flags_request =
				MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU |
				MCPS802154_RX_FRAME_INFO_RANGING_PDOA,
		},
	};

	access->frames[TWR_FRAME_REPORT] = (struct mcps802154_access_frame){
		.is_tx = true,
		.tx_frame_info = {
			.flags = MCPS802154_TX_FRAME_TIMESTAMP_DTU |
				MCPS802154_TX_FRAME_RANGING |
				MCPS802154_TX_FRAME_SP1,
			.ant_id = local->tx_ant,
		},
	};

	return access;
}

static const struct mcps802154_region_ops
	simple_ranging_twr_responder_region_ops = {
		.name = "twr_responder",
		.get_access = twr_responder_get_access,
	};

/* Region initiator. */

static struct mcps802154_access *
twr_get_access(struct mcps802154_region *region, u32 next_timestamp_dtu,
	       int next_in_region_dtu, int region_duration_dtu)
{
	struct simple_ranging_local *local = region_init_to_local(region);
	struct mcps802154_access *access = &local->access;
	const int slots_dtu = local->slot_duration_dtu * N_TWR_FRAMES;

	/* Do nothing if nothing to do. */
	if (local->n_requests == 0)
		return NULL;

	/* Only start a ranging request if we have enough time to end it. */
	if (next_in_region_dtu + slots_dtu > region_duration_dtu)
		return NULL;

	twr_request_start(local);
	access->method = MCPS802154_ACCESS_METHOD_MULTI;
	access->ops = &twr_access_ops;
	access->n_frames = ARRAY_SIZE(local->frames);
	access->frames = local->frames;

	access->frames[TWR_FRAME_POLL] = (struct mcps802154_access_frame){
		.is_tx = true,
		.tx_frame_info = {
			.timestamp_dtu = next_timestamp_dtu,
			.flags = MCPS802154_TX_FRAME_TIMESTAMP_DTU |
				MCPS802154_TX_FRAME_RANGING |
				MCPS802154_TX_FRAME_KEEP_RANGING_CLOCK |
				MCPS802154_TX_FRAME_SP1,
			.ant_id = local->tx_ant,
		},
		.sts_params = &local->sts_params,
	};
	local->initiator.poll_tx_timestamp_rctu =
		mcps802154_tx_timestamp_dtu_to_rmarker_rctu(
			local->llhw, next_timestamp_dtu, local->tx_ant);

	access->frames[TWR_FRAME_RESP] = (struct mcps802154_access_frame){
		.is_tx = false,
		.rx = {
			.info = {
				.timestamp_dtu = next_timestamp_dtu +
					local->slot_duration_dtu,
				.flags = MCPS802154_RX_INFO_TIMESTAMP_DTU |
					MCPS802154_RX_INFO_RANGING |
					MCPS802154_RX_INFO_KEEP_RANGING_CLOCK |
					MCPS802154_RX_INFO_RANGING_PDOA |
					MCPS802154_RX_INFO_SP1,
				.ant_pair_id = local->rx_ant_pair_azimuth,
			},
			.frame_info_flags_request =
				MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU |
				MCPS802154_RX_FRAME_INFO_RANGING_PDOA,
		},
	};

	access->frames[TWR_FRAME_FINAL] = (struct mcps802154_access_frame){
		.is_tx = true,
		.tx_frame_info = {
			.timestamp_dtu = next_timestamp_dtu +
				2 * local->slot_duration_dtu,
			.flags = MCPS802154_TX_FRAME_TIMESTAMP_DTU |
				MCPS802154_TX_FRAME_RANGING |
				MCPS802154_TX_FRAME_SP1,
			.ant_id = local->tx_ant,
		},
	};
	local->initiator.final_tx_timestamp_rctu =
		mcps802154_tx_timestamp_dtu_to_rmarker_rctu(
			local->llhw,
			next_timestamp_dtu + 2 * local->slot_duration_dtu,
			local->tx_ant);

	access->frames[TWR_FRAME_REPORT] = (struct mcps802154_access_frame){
		.is_tx = false,
		.rx = {
			.info = {
				.timestamp_dtu = next_timestamp_dtu +
					3 * local->slot_duration_dtu,
				.flags = MCPS802154_RX_INFO_TIMESTAMP_DTU |
					MCPS802154_RX_INFO_RANGING |
					MCPS802154_RX_INFO_RANGING_PDOA |
					MCPS802154_RX_INFO_SP1,
				.ant_pair_id = local->rx_ant_pair_elevation,
			},
			.frame_info_flags_request =
				MCPS802154_RX_FRAME_INFO_RANGING_PDOA,
		},
	};

	return access;
}

static const struct mcps802154_region_ops simple_ranging_twr_region_ops = {
	.name = "twr_initiator",
	.get_access = twr_get_access,
};

/* Scheduler. */

static struct mcps802154_scheduler *
simple_ranging_scheduler_open(struct mcps802154_llhw *llhw)
{
	struct simple_ranging_local *local;

	local = kzalloc(sizeof(*local), GFP_KERNEL);
	if (!local)
		return NULL;
	local->region_resp.ops = &simple_ranging_twr_responder_region_ops;
	local->region_init.ops = &simple_ranging_twr_region_ops;
	local->llhw = llhw;
	local->sts_params.n_segs = 1;
	local->sts_params.seg_len = 256;
	local->slot_duration_dtu = TWR_SLOT_DEFAULT_RCTU / llhw->dtu_rctu;
	local->is_responder = false;
	local->tx_ant = TX_ANT_ID_DEFAULT;
	local->rx_ant_pair_azimuth = RX_ANT_PAIR_ID_DEFAULT;
	local->rx_ant_pair_elevation = RX_ANT_PAIR_ID_DEFAULT;
	local->is_same_rx_ant = true;

	twr_requests_clear(local);
	return &local->scheduler;
}

static void
simple_ranging_scheduler_close(struct mcps802154_scheduler *scheduler)
{
	struct simple_ranging_local *local = scheduler_to_local(scheduler);

	kfree(local);
}

static int simple_ranging_scheduler_update_schedule(
	struct mcps802154_scheduler *scheduler,
	const struct mcps802154_schedule_update *schedule_update,
	u32 next_timestamp_dtu)
{
	struct simple_ranging_local *local = scheduler_to_local(scheduler);
	const int slot_dtu = local->slot_duration_dtu;
	int twr_slots = local->n_requests * N_TWR_FRAMES;
	int r;

	if (schedule_update->n_regions) {
		int schedule_duration_slots = local->llhw->dtu_freq_hz /
					      slot_dtu / local->frequency_hz;
		/* This treatment is done only for initiator.
		 * Responder region never enters here. As it is an infinite
		 * region. */
		WARN_ON(local->is_responder);
		if (schedule_duration_slots < twr_slots)
			schedule_duration_slots = twr_slots;

		r = mcps802154_schedule_set_start(
			schedule_update,
			schedule_update->expected_start_timestamp_dtu +
				(schedule_duration_slots - twr_slots) *
					slot_dtu);
		WARN_RETURN(r);
	}

	r = mcps802154_schedule_recycle(schedule_update, 0,
					MCPS802154_DURATION_NO_CHANGE);
	WARN_RETURN(r);

	if (local->is_responder) {
		r = mcps802154_schedule_add_region(schedule_update,
						   &local->region_resp, 0, 0);
	} else {
		r = mcps802154_schedule_add_region(schedule_update,
						   &local->region_init, 0,
						   twr_slots * slot_dtu);
	}
	return r;
}

static int simple_ranging_scheduler_ranging_setup(
	struct mcps802154_scheduler *scheduler,
	const struct mcps802154_nl_ranging_request *requests,
	unsigned int n_requests)
{
	struct simple_ranging_local *local = scheduler_to_local(scheduler);
	bool need_invalidate = local->n_requests == 0;
	int max_frequency_hz = 1;
	int i;

	if (local->is_responder)
		return -EOPNOTSUPP;

	if (n_requests > MCPS802154_NL_RANGING_REQUESTS_MAX)
		return -EINVAL;

	for (i = 0; i < n_requests; i++) {
		if (requests[i].remote_peer_extended_addr)
			return -EOPNOTSUPP;
		local->requests[i] = requests[i];
		if (requests[i].frequency_hz > max_frequency_hz)
			max_frequency_hz = requests[i].frequency_hz;
	}
	local->n_requests = n_requests;
	local->frequency_hz = max_frequency_hz;

	if (need_invalidate)
		mcps802154_schedule_invalidate(local->llhw);

	return 0;
}

static int
simple_ranging_scheduler_set_parameters(struct mcps802154_scheduler *scheduler,
					const struct nlattr *params_attr,
					struct netlink_ext_ack *extack)
{
	struct simple_ranging_local *local = scheduler_to_local(scheduler);
	struct nlattr *attrs[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_MAX + 1];
	int r;
	static const struct nla_policy nla_policy[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_MAX +
						  1] = {
		[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_SLOT_DURATION_MS] = { .type = NLA_U32 },
		[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_NODE_TYPE] = { .type = NLA_U32,
									  .validation_type =
										  NLA_VALIDATE_MAX,
									  .max = 1 },
		[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_TX_ANTENNA] = { .type = NLA_U8 },
		[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_RX_ANTENNA_PAIR_AZIMUTH] = { .type = NLA_U8 },
		[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_RX_ANTENNA_PAIR_ELEVATION] = { .type = NLA_U8 },
	};

	r = nla_parse_nested(attrs,
			     SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_MAX,
			     params_attr, nla_policy, extack);
	if (r)
		return r;

	if (attrs[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_SLOT_DURATION_MS]) {
		int slot_duration_ms = nla_get_u32(
			attrs[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_SLOT_DURATION_MS]);

		if (!slot_duration_ms ||
		    (slot_duration_ms & (slot_duration_ms - 1)) ||
		    slot_duration_ms > TWR_SLOT_MS_MAX)
			return -EINVAL;

		local->slot_duration_dtu = slot_duration_ms *
					   TWR_SLOT_MS_TO_RCTU /
					   local->llhw->dtu_rctu;
	}
	if (attrs[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_NODE_TYPE]) {
		u32 type = nla_get_u32(
			attrs[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_NODE_TYPE]);

		local->is_responder = type == 1 ? true : false;
		mcps802154_schedule_invalidate(local->llhw);
	}

	if (attrs[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_TX_ANTENNA]) {
		u8 id = nla_get_u8(
			attrs[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_TX_ANTENNA]);
		local->tx_ant = id;
	}
	if (attrs[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_RX_ANTENNA_PAIR_AZIMUTH]) {
		u8 id = nla_get_u8(
			attrs[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_RX_ANTENNA_PAIR_AZIMUTH]);
		local->rx_ant_pair_azimuth = id;
	}
	if (attrs[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_RX_ANTENNA_PAIR_ELEVATION]) {
		u8 id = nla_get_u8(
			attrs[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_RX_ANTENNA_PAIR_ELEVATION]);
		local->rx_ant_pair_elevation = id;
	}

	local->is_same_rx_ant = local->rx_ant_pair_azimuth ==
				local->rx_ant_pair_elevation;

	return 0;
}

static struct mcps802154_scheduler_ops simple_ranging_scheduler_ops = {
	.owner = THIS_MODULE,
	.name = "simple-ranging",
	.open = simple_ranging_scheduler_open,
	.close = simple_ranging_scheduler_close,
	.update_schedule = simple_ranging_scheduler_update_schedule,
	.ranging_setup = simple_ranging_scheduler_ranging_setup,
	.set_parameters = simple_ranging_scheduler_set_parameters,
};

int __init simple_ranging_region_init(void)
{
	int r = mcps802154_scheduler_register(&simple_ranging_scheduler_ops);
	/* TODO: register regions when they can be used from another scheduler. */
	return r;
}

void __exit simple_ranging_region_exit(void)
{
	mcps802154_scheduler_unregister(&simple_ranging_scheduler_ops);
}
