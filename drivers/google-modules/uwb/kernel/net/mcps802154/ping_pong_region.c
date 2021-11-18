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
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/ieee802154.h>
#include <net/af_ieee802154.h>
#include <net/mcps802154_schedule.h>
#include <net/mcps802154_frame.h>
#include <net/simple_ranging_region_nl.h>
#include "nl.h"
#include "warn_return.h"
#include "ping_pong_region.h"

/* t_reply1 in RCTU
 * t_reply1 is 400 microseconds, in RCTU:
 * 400000000/15.6500400641026 = 25559039.99999994
 */
#define TWR_FACTORY_TEST_T_REPLY1_RCTU (25559040ull * 2)

/* t_reply2 in RCTU
 * t_reply2 is 17000 microseconds, in RCTU:
 * 17000000000/15.6500400641026 = 1086259199.999998
 */
#define TWR_FACTORY_TEST_T_REPLY2_RCTU (1086259200ull)

#define TWR_FACTORY_TEST_FUNCTION_CODE_POLL 0x0110
#define TWR_FACTORY_TEST_FUNCTION_CODE_RESP 0x0111
#define TWR_FACTORY_TEST_FUNCTION_CODE_FINAL 0x0112

#define PING_PONG_FRAME_SIZE 2

#define PING_PONG_FRAME_HEADER_SIZE                                       \
	(IEEE802154_FC_LEN + IEEE802154_SEQ_LEN + IEEE802154_PAN_ID_LEN + \
	 IEEE802154_SHORT_ADDR_LEN * 2)
#define PING_PONG_FRAME_MAX_SIZE \
	(PING_PONG_FRAME_HEADER_SIZE + PING_PONG_FRAME_SIZE)

enum twr_factory_tests_frames {
	TWR_FACTORY_TEST_FRAME_POLL,
	TWR_FACTORY_TEST_FRAME_RESP,
	TWR_FACTORY_TEST_FRAME_FINAL,
	N_TWR_FACTORY_TEST_FRAMES,
};

struct ping_pong_initiator_time {
	u64 t_0;
	u64 t_3;
	u64 t_4;
};

struct ping_pong_local {
	struct mcps802154_scheduler scheduler;
	struct mcps802154_llhw *llhw;
	struct mcps802154_region region_init_active;
	struct mcps802154_region region_init_idle;
	struct mcps802154_region region_resp;
	struct mcps802154_access access;
	struct mcps802154_access_frame frames[N_TWR_FACTORY_TEST_FRAMES];
	struct mcps802154_nl_ranging_request ping_pong_request;
	int region_init_active_duration_dtu;
	bool enable_init_tx;
	bool is_responder;
	struct ping_pong_initiator_time initiator_time;
};

static inline struct ping_pong_local *
scheduler_to_ping_pong_local(const struct mcps802154_scheduler *scheduler)
{
	return container_of(scheduler, struct ping_pong_local, scheduler);
}

static inline struct ping_pong_local *
region_init_active_to_ping_pong_local(struct mcps802154_region *region)
{
	return container_of(region, struct ping_pong_local, region_init_active);
}

static inline struct ping_pong_local *
region_init_idle_to_ping_pong_local(struct mcps802154_region *region)
{
	return container_of(region, struct ping_pong_local, region_init_idle);
}

static inline struct ping_pong_local *
region_resp_to_ping_pong_local(struct mcps802154_region *region)
{
	return container_of(region, struct ping_pong_local, region_resp);
}

static inline struct ping_pong_local *
access_to_ping_pong_local(const struct mcps802154_access *access)
{
	return container_of(access, struct ping_pong_local, access);
}

static void ping_pong_report(struct ping_pong_local *local, u64 t_0, u64 t_3,
			     u64 t_4)
{
	struct mcps802154_nl_ranging_request *request =
		&local->ping_pong_request;
	int r;

	r = mcps802154_nl_ping_pong_report(local->llhw, request->id, t_0, t_3,
					   t_4);
	/* Disable TX and force schedule update to change the region */
	local->enable_init_tx = false;
	mcps802154_schedule_invalidate(local->llhw);
}

void ping_pong_frame_header_fill_buf(u8 *buf, __le16 pan_id, __le16 dst,
				     __le16 src)
{
	u16 fc = (IEEE802154_FC_TYPE_DATA | IEEE802154_FC_INTRA_PAN |
		  (IEEE802154_ADDR_SHORT << IEEE802154_FC_DAMODE_SHIFT) |
		  (1 << IEEE802154_FC_VERSION_SHIFT) |
		  (IEEE802154_ADDR_SHORT << IEEE802154_FC_SAMODE_SHIFT));
	u8 seq = 0;
	size_t pos = 0;

	put_unaligned_le16(fc, buf + pos);
	pos += IEEE802154_FC_LEN;
	buf[pos] = seq;
	pos += IEEE802154_SEQ_LEN;
	memcpy(buf + pos, &pan_id, sizeof(pan_id));
	pos += IEEE802154_PAN_ID_LEN;
	memcpy(buf + pos, &dst, sizeof(dst));
	pos += IEEE802154_SHORT_ADDR_LEN;
	memcpy(buf + pos, &src, sizeof(src));
}

void ping_pong_frame_header_put(struct sk_buff *skb, __le16 pan_id, __le16 dst,
				__le16 src)
{
	ping_pong_frame_header_fill_buf(
		skb_put(skb, PING_PONG_FRAME_HEADER_SIZE), pan_id, dst, src);
}

static void ping_pong_resp_rx_frame(struct mcps802154_access *access,
				    int frame_idx, struct sk_buff *skb,
				    const struct mcps802154_rx_frame_info *info,
				    enum mcps802154_rx_error_type error)
{
	struct ping_pong_local *local = access_to_ping_pong_local(access);
	struct mcps802154_nl_ranging_request *request =
		&local->ping_pong_request;
	u32 resp_tx_start_dtu;

	if (!skb) {
		/* In case of NULL skb, to avoid the next TX, we adjust
		 * the frames count of region access. */
		access->n_frames = frame_idx + 1;
		return;
	}
	request->peer_extended_addr =
		get_unaligned_le64(skb->data + PING_PONG_FRAME_HEADER_SIZE -
				   IEEE802154_SHORT_ADDR_LEN);
	resp_tx_start_dtu =
		info->timestamp_dtu +
		TWR_FACTORY_TEST_T_REPLY1_RCTU / local->llhw->dtu_rctu;
	/* Set the timings for the next frames. */
	access->frames[TWR_FACTORY_TEST_FRAME_RESP].tx_frame_info.timestamp_dtu =
		resp_tx_start_dtu;
	access->frames[TWR_FACTORY_TEST_FRAME_FINAL].rx.info.timestamp_dtu =
		resp_tx_start_dtu +
		TWR_FACTORY_TEST_T_REPLY2_RCTU / local->llhw->dtu_rctu;

	kfree_skb(skb);
	return;
}

static struct sk_buff *
ping_pong_resp_tx_get_frame(struct mcps802154_access *access, int frame_idx)
{
	struct ping_pong_local *local = access_to_ping_pong_local(access);
	struct mcps802154_nl_ranging_request *request =
		&local->ping_pong_request;
	struct sk_buff *skb = mcps802154_frame_alloc(
		local->llhw, PING_PONG_FRAME_MAX_SIZE, GFP_KERNEL);
	/* Extended address from mcps802154_nl_ranging_request are used as
	 * short address */
	ping_pong_frame_header_put(skb, mcps802154_get_pan_id(local->llhw),
				   request->peer_extended_addr,
				   mcps802154_get_short_addr(local->llhw));

	if (WARN_ON(frame_idx != TWR_FACTORY_TEST_FRAME_RESP))
		return skb;

	skb_put_u8(skb, (u8)TWR_FACTORY_TEST_FUNCTION_CODE_RESP);
	skb_put_u8(skb, (u8)(TWR_FACTORY_TEST_FUNCTION_CODE_RESP >> 8));
	return skb;
}

static void ping_pong_tx_return(struct mcps802154_access *access, int frame_idx,
				struct sk_buff *skb,
				enum mcps802154_access_tx_return_reason reason)
{
	kfree_skb(skb);
}

struct mcps802154_access_ops ping_pong_resp_access_ops = {
	.rx_frame = ping_pong_resp_rx_frame,
	.tx_get_frame = ping_pong_resp_tx_get_frame,
	.tx_return = ping_pong_tx_return,
};

static struct mcps802154_access *
ping_pong_resp_get_access(struct mcps802154_region *region,
			  u32 next_timestamp_dtu, int next_in_region_dtu,
			  int region_duration_dtu)
{
	struct ping_pong_local *local = region_resp_to_ping_pong_local(region);
	struct mcps802154_access *access = &local->access;
	u32 start_dtu = next_timestamp_dtu;

	access->method = MCPS802154_ACCESS_METHOD_MULTI;
	access->ops = &ping_pong_resp_access_ops;
	access->n_frames = ARRAY_SIZE(local->frames);
	access->frames = local->frames;

	access->frames[TWR_FACTORY_TEST_FRAME_POLL].is_tx = false;
	access->frames[TWR_FACTORY_TEST_FRAME_POLL].rx.info.timestamp_dtu =
		start_dtu;
	access->frames[TWR_FACTORY_TEST_FRAME_POLL].rx.info.timeout_dtu = -1;
	access->frames[TWR_FACTORY_TEST_FRAME_POLL].rx.info.flags =
		MCPS802154_RX_INFO_TIMESTAMP_DTU | MCPS802154_RX_INFO_RANGING |
		MCPS802154_RX_INFO_KEEP_RANGING_CLOCK;
	access->frames[TWR_FACTORY_TEST_FRAME_POLL].rx.frame_info_flags_request =
		MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU;

	access->frames[TWR_FACTORY_TEST_FRAME_RESP].is_tx = true;
	access->frames[TWR_FACTORY_TEST_FRAME_RESP].tx_frame_info.flags =
		MCPS802154_TX_FRAME_TIMESTAMP_DTU |
		MCPS802154_TX_FRAME_RANGING |
		MCPS802154_TX_FRAME_KEEP_RANGING_CLOCK;
	access->frames[TWR_FACTORY_TEST_FRAME_RESP]
		.tx_frame_info.rx_enable_after_tx_dtu = 0;
	access->frames[TWR_FACTORY_TEST_FRAME_RESP]
		.tx_frame_info.rx_enable_after_tx_timeout_dtu = 0;

	access->frames[TWR_FACTORY_TEST_FRAME_FINAL].is_tx = false;
	access->frames[TWR_FACTORY_TEST_FRAME_FINAL].rx.info.flags =
		MCPS802154_RX_INFO_TIMESTAMP_DTU | MCPS802154_RX_INFO_RANGING;
	access->frames[TWR_FACTORY_TEST_FRAME_FINAL].rx.info.timeout_dtu = 0;
	access->frames[TWR_FACTORY_TEST_FRAME_FINAL]
		.rx.frame_info_flags_request = 0;

	return access;
}

static struct mcps802154_region_ops ping_pong_region_resp_ops = {
	.owner = THIS_MODULE,
	.name = "ping-pong-resp",
	.get_access = ping_pong_resp_get_access,
};

static void
ping_pong_init_idle_rx_frame(struct mcps802154_access *access, int frame_idx,
			     struct sk_buff *skb,
			     const struct mcps802154_rx_frame_info *info,
			     enum mcps802154_rx_error_type error)
{
	if (!skb)
		return;
	kfree_skb(skb);
}

struct mcps802154_access_ops ping_pong_init_idle_access_ops = {
	.rx_frame = ping_pong_init_idle_rx_frame,
};

static struct mcps802154_access *
ping_pong_init_idle_get_access(struct mcps802154_region *region,
			       u32 next_timestamp_dtu, int next_in_region_dtu,
			       int region_duration_dtu)
{
	struct ping_pong_local *local =
		region_init_idle_to_ping_pong_local(region);
	struct mcps802154_access *access = &local->access;

	access->method = MCPS802154_ACCESS_METHOD_IMMEDIATE_RX;
	access->ops = &ping_pong_init_idle_access_ops;
	return &local->access;
}

static struct mcps802154_region_ops ping_pong_region_init_idle_ops = {
	.owner = THIS_MODULE,
	.name = "ping-pong-init-idle",
	.get_access = ping_pong_init_idle_get_access,
};

static void
ping_pong_init_active_rx_frame(struct mcps802154_access *access, int frame_idx,
			       struct sk_buff *skb,
			       const struct mcps802154_rx_frame_info *info,
			       enum mcps802154_rx_error_type error)
{
	struct ping_pong_local *local = access_to_ping_pong_local(access);
	u32 final_timestamp_dtu;

	if (!skb) {
		/* Remote peer time out: set all time markers to 0 */
		local->initiator_time.t_0 = 0;
		local->initiator_time.t_3 = 0;
		local->initiator_time.t_4 = 0;
		/* Avoid the next TX */
		access->n_frames = frame_idx + 1;
		return;
	}
	final_timestamp_dtu =
		info->timestamp_dtu +
		TWR_FACTORY_TEST_T_REPLY2_RCTU / local->llhw->dtu_rctu;
	local->initiator_time.t_3 = info->timestamp_rctu;
	local->initiator_time.t_4 = mcps802154_tx_timestamp_dtu_to_rmarker_rctu(
		local->llhw, final_timestamp_dtu, 0);
	/* Set the timing for the final frame */
	access->frames[TWR_FACTORY_TEST_FRAME_FINAL]
		.tx_frame_info.timestamp_dtu = final_timestamp_dtu;

	kfree_skb(skb);
}

static struct sk_buff *
ping_pong_init_active_tx_get_frame(struct mcps802154_access *access,
				   int frame_idx)
{
	struct ping_pong_local *local = access_to_ping_pong_local(access);
	struct mcps802154_nl_ranging_request *request =
		&local->ping_pong_request;
	struct sk_buff *skb = mcps802154_frame_alloc(
		local->llhw, PING_PONG_FRAME_MAX_SIZE, GFP_KERNEL);

	ping_pong_frame_header_put(skb, mcps802154_get_pan_id(local->llhw),
				   request->peer_extended_addr,
				   mcps802154_get_short_addr(local->llhw));
	if (frame_idx == TWR_FACTORY_TEST_FRAME_POLL) {
		skb_put_u8(skb, (u8)TWR_FACTORY_TEST_FUNCTION_CODE_POLL);
		skb_put_u8(skb, (u8)(TWR_FACTORY_TEST_FUNCTION_CODE_POLL >> 8));
	} else {
		WARN_ON(frame_idx != TWR_FACTORY_TEST_FRAME_FINAL);
		skb_put_u8(skb, (u8)TWR_FACTORY_TEST_FUNCTION_CODE_FINAL);
		skb_put_u8(skb,
			   (u8)(TWR_FACTORY_TEST_FUNCTION_CODE_FINAL >> 8));
	}
	return skb;
}

static void ping_pong_init_active_access_done(struct mcps802154_access *access,
					      int error)
{
	struct ping_pong_local *local = access_to_ping_pong_local(access);

	ping_pong_report(local, local->initiator_time.t_0,
			 local->initiator_time.t_3, local->initiator_time.t_4);
}

struct mcps802154_access_ops ping_pong_init_active_access_ops = {
	.common = {
		.access_done = ping_pong_init_active_access_done,
	},
	.rx_frame = ping_pong_init_active_rx_frame,
	.tx_get_frame = ping_pong_init_active_tx_get_frame,
	.tx_return = ping_pong_tx_return,
};

static struct mcps802154_access *
ping_pong_init_active_get_access(struct mcps802154_region *region,
				 u32 next_timestamp_dtu, int next_in_region_dtu,
				 int region_duration_dtu)
{
	struct ping_pong_local *local =
		region_init_active_to_ping_pong_local(region);
	struct mcps802154_access *access = &local->access;
	u32 start_dtu;

	/* Only send frames on time per schedule */
	if (next_in_region_dtu != 0) {
		return NULL;
	}
	start_dtu = next_timestamp_dtu;
	access->method = MCPS802154_ACCESS_METHOD_MULTI;
	access->n_frames = ARRAY_SIZE(local->frames);
	access->frames = local->frames;
	/* Hard-coded! */
	access->n_frames = ARRAY_SIZE(local->frames);
	access->frames[TWR_FACTORY_TEST_FRAME_POLL].is_tx = true;
	access->frames[TWR_FACTORY_TEST_FRAME_POLL].tx_frame_info.timestamp_dtu =
		start_dtu;
	access->frames[TWR_FACTORY_TEST_FRAME_POLL].tx_frame_info.flags =
		MCPS802154_TX_FRAME_TIMESTAMP_DTU |
		MCPS802154_TX_FRAME_RANGING |
		MCPS802154_TX_FRAME_KEEP_RANGING_CLOCK;
	access->frames[TWR_FACTORY_TEST_FRAME_POLL]
		.tx_frame_info.rx_enable_after_tx_dtu = 0;
	access->frames[TWR_FACTORY_TEST_FRAME_POLL]
		.tx_frame_info.rx_enable_after_tx_timeout_dtu = 0;
	local->initiator_time.t_0 = mcps802154_tx_timestamp_dtu_to_rmarker_rctu(
		local->llhw, start_dtu, 0);

	access->frames[TWR_FACTORY_TEST_FRAME_RESP].is_tx = false;
	access->frames[TWR_FACTORY_TEST_FRAME_RESP].rx.info.timestamp_dtu =
		start_dtu +
		TWR_FACTORY_TEST_T_REPLY1_RCTU / local->llhw->dtu_rctu;
	access->frames[TWR_FACTORY_TEST_FRAME_RESP].rx.info.timeout_dtu = 0;
	access->frames[TWR_FACTORY_TEST_FRAME_RESP].rx.info.flags =
		MCPS802154_RX_INFO_TIMESTAMP_DTU | MCPS802154_RX_INFO_RANGING |
		MCPS802154_RX_INFO_KEEP_RANGING_CLOCK;
	access->frames[TWR_FACTORY_TEST_FRAME_RESP].rx.frame_info_flags_request =
		MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU |
		MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU;

	access->frames[TWR_FACTORY_TEST_FRAME_FINAL].is_tx = true;
	access->frames[TWR_FACTORY_TEST_FRAME_FINAL]
		.tx_frame_info.timestamp_dtu =
		start_dtu + (TWR_FACTORY_TEST_T_REPLY1_RCTU +
			     TWR_FACTORY_TEST_T_REPLY2_RCTU) /
				    local->llhw->dtu_rctu;
	access->frames[TWR_FACTORY_TEST_FRAME_FINAL].tx_frame_info.flags =
		MCPS802154_TX_FRAME_TIMESTAMP_DTU | MCPS802154_TX_FRAME_RANGING;
	access->frames[TWR_FACTORY_TEST_FRAME_FINAL]
		.tx_frame_info.rx_enable_after_tx_dtu = 0;
	access->frames[TWR_FACTORY_TEST_FRAME_FINAL]
		.tx_frame_info.rx_enable_after_tx_timeout_dtu = 0;
	access->ops = &ping_pong_init_active_access_ops;
	return &local->access;
}

static struct mcps802154_region_ops ping_pong_region_init_active_ops = {
	.owner = THIS_MODULE,
	.name = "ping-pong-init-active",
	.get_access = ping_pong_init_active_get_access,
};

static struct mcps802154_scheduler *
ping_pong_scheduler_open(struct mcps802154_llhw *llhw)
{
	struct ping_pong_local *local;

	local = kzalloc(sizeof(*local), GFP_KERNEL);
	if (!local)
		return NULL;
	local->llhw = llhw;
	local->region_init_active.ops = &ping_pong_region_init_active_ops;
	local->region_init_idle.ops = &ping_pong_region_init_idle_ops;
	local->region_resp.ops = &ping_pong_region_resp_ops;
	local->is_responder = false;
	local->enable_init_tx = false;
	local->region_init_active_duration_dtu =
		(TWR_FACTORY_TEST_T_REPLY1_RCTU +
		 TWR_FACTORY_TEST_T_REPLY2_RCTU) /
		local->llhw->dtu_rctu;
	return &local->scheduler;
}

static void ping_pong_scheduler_close(struct mcps802154_scheduler *scheduler)
{
	struct ping_pong_local *ping_pong_local =
		scheduler_to_ping_pong_local(scheduler);
	kfree(ping_pong_local);
}

static int ping_pong_scheduler_update_schedule(
	struct mcps802154_scheduler *scheduler,
	const struct mcps802154_schedule_update *schedule_update,
	u32 next_timestamp_dtu)
{
	struct ping_pong_local *local = scheduler_to_ping_pong_local(scheduler);
	int r;

	/* Remove the region in the schedule */
	r = mcps802154_schedule_recycle(schedule_update, 0,
					MCPS802154_DURATION_NO_CHANGE);
	WARN_RETURN(r);

	if (local->is_responder) {
		r = mcps802154_schedule_add_region(schedule_update,
						   &local->region_resp, 0, 0);
	} else {
		if (local->enable_init_tx) {
			r = mcps802154_schedule_add_region(
				schedule_update, &local->region_init_active, 0,
				local->region_init_active_duration_dtu);
		} else {
			r = mcps802154_schedule_add_region(
				schedule_update, &local->region_init_idle, 0,
				0);
		}
	}
	return r;
}

static int ping_pong_scheduler_ping_pong_setup(
	struct mcps802154_scheduler *scheduler,
	const struct mcps802154_nl_ranging_request *requests,
	unsigned int n_requests)
{
	struct ping_pong_local *local = scheduler_to_ping_pong_local(scheduler);

	if (local->is_responder)
		return -EOPNOTSUPP;
	if (n_requests != 1)
		return -EINVAL;
	if (requests[0].remote_peer_extended_addr)
		return -EOPNOTSUPP;
	local->ping_pong_request = requests[0];
	/* Enable TX and force schedule update to change the region */
	local->enable_init_tx = true;
	mcps802154_schedule_invalidate(local->llhw);
	return 0;
}

static int
ping_pong_scheduler_set_parameters(struct mcps802154_scheduler *scheduler,
				   const struct nlattr *params_attr,
				   struct netlink_ext_ack *extack)
{
	struct ping_pong_local *local = scheduler_to_ping_pong_local(scheduler);
	struct nlattr *attrs[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_MAX + 1];
	int r;
	static const struct nla_policy nla_policy[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_MAX +
						  1] = {
		[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_NODE_TYPE] = { .type = NLA_U32 },
	};

	r = nla_parse_nested(attrs,
			     SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_MAX,
			     params_attr, nla_policy, extack);
	if (r)
		return r;

	if (attrs[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_NODE_TYPE]) {
		u32 type = nla_get_u32(
			attrs[SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_NODE_TYPE]);

		if (type > 1)
			return -EINVAL;

		local->is_responder = type == 1 ? true : false;
		mcps802154_schedule_invalidate(local->llhw);
	}

	return 0;
}

static struct mcps802154_scheduler_ops ping_pong_region_scheduler = {
	.owner = THIS_MODULE,
	.name = "ping-pong",
	.open = ping_pong_scheduler_open,
	.close = ping_pong_scheduler_close,
	.update_schedule = ping_pong_scheduler_update_schedule,
	.ranging_setup = ping_pong_scheduler_ping_pong_setup,
	.set_parameters = ping_pong_scheduler_set_parameters,
};

int __init ping_pong_region_init(void)
{
	return mcps802154_scheduler_register(&ping_pong_region_scheduler);
}

void __exit ping_pong_region_exit(void)
{
	mcps802154_scheduler_unregister(&ping_pong_region_scheduler);
}
