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
 * 802.15.4 mac common part sublayer, default data path regions.
 *
 */
#include <linux/slab.h>
#include <linux/errno.h>

#include <linux/netdevice.h>

#include <net/mcps802154_schedule.h>

#include "mcps802154_i.h"

#include "default_region.h"
#include "warn_return.h"

struct mcps802154_default_local {
	struct mcps802154_scheduler scheduler;
	struct mcps802154_llhw *llhw;
	struct mcps802154_region region;
	struct mcps802154_access access;
};

static inline struct mcps802154_default_local *
scheduler_to_dlocal(const struct mcps802154_scheduler *scheduler)
{
	return container_of(scheduler, struct mcps802154_default_local,
			    scheduler);
}

static inline struct mcps802154_default_local *
region_to_dlocal(struct mcps802154_region *region)
{
	return container_of(region, struct mcps802154_default_local, region);
}

static inline struct mcps802154_default_local *
access_to_dlocal(const struct mcps802154_access *access)
{
	return container_of(access, struct mcps802154_default_local, access);
}

static void simple_rx_frame(struct mcps802154_access *access, int frame_idx,
			    struct sk_buff *skb,
			    const struct mcps802154_rx_frame_info *info)
{
	struct mcps802154_default_local *dlocal = access_to_dlocal(access);
	struct mcps802154_local *local = llhw_to_local(dlocal->llhw);

	ieee802154_rx_irqsafe(local->hw, skb, info->lqi);
}

static struct sk_buff *simple_tx_get_frame(struct mcps802154_access *access,
					   int frame_idx)
{
	struct mcps802154_default_local *dlocal = access_to_dlocal(access);
	struct mcps802154_local *local = llhw_to_local(dlocal->llhw);

	return skb_dequeue(&local->ca.queue);
}

static void simple_tx_return(struct mcps802154_access *access, int frame_idx,
			     struct sk_buff *skb,
			     enum mcps802154_access_tx_return_reason reason)
{
	struct mcps802154_default_local *dlocal = access_to_dlocal(access);
	struct mcps802154_local *local = llhw_to_local(dlocal->llhw);

	if (reason == MCPS802154_ACCESS_TX_RETURN_REASON_FAILURE) {
		local->ca.retries++;
		if (local->ca.retries <= local->pib.mac_max_frame_retries) {
			/* Retry the frame. */
			skb_queue_head(&local->ca.queue, skb);
		} else {
			local->ca.retries = 0;
			ieee802154_wake_queue(local->hw);
			dev_kfree_skb_any(skb);
			atomic_dec(&local->ca.n_queued);
		}
	} else if (reason == MCPS802154_ACCESS_TX_RETURN_REASON_CANCEL) {
		skb_queue_head(&local->ca.queue, skb);
	} else {
		local->ca.retries = 0;
		ieee802154_xmit_complete(local->hw, skb, false);
		atomic_dec(&local->ca.n_queued);
	}
}

static void simple_access_done(struct mcps802154_access *access)
{
	/* Nothing. */
}

struct mcps802154_access_ops simple_access_ops = {
	.rx_frame = simple_rx_frame,
	.tx_get_frame = simple_tx_get_frame,
	.tx_return = simple_tx_return,
	.access_done = simple_access_done,
};

static struct mcps802154_access *
simple_default_get_access(struct mcps802154_region *region,
			  u32 next_timestamp_dtu, int next_in_region_dtu,
			  int region_duration_dtu)
{
	struct mcps802154_default_local *dlocal = region_to_dlocal(region);
	struct mcps802154_local *local = llhw_to_local(dlocal->llhw);

	dlocal->access.method = skb_queue_empty(&local->ca.queue) ?
					MCPS802154_ACCESS_METHOD_IMMEDIATE_RX :
					MCPS802154_ACCESS_METHOD_IMMEDIATE_TX;
	dlocal->access.ops = &simple_access_ops;
	return &dlocal->access;
}

static struct mcps802154_region_ops mcps802154_default_simple_region_ops = {
	.owner = THIS_MODULE,
	.name = "simple",
	.get_access = simple_default_get_access,
};

static struct mcps802154_scheduler *
mcps802154_default_scheduler_open(struct mcps802154_llhw *llhw)
{
	struct mcps802154_default_local *dlocal;

	dlocal = kmalloc(sizeof(*dlocal), GFP_KERNEL);
	if (!dlocal)
		return NULL;
	dlocal->llhw = llhw;
	dlocal->region.ops = &mcps802154_default_simple_region_ops;
	return &dlocal->scheduler;
}

static void
mcps802154_default_scheduler_close(struct mcps802154_scheduler *scheduler)
{
	struct mcps802154_default_local *dlocal =
		scheduler_to_dlocal(scheduler);
	kfree(dlocal);
}

static int mcps802154_default_scheduler_update_schedule(
	struct mcps802154_scheduler *scheduler,
	const struct mcps802154_schedule_update *schedule_update,
	u32 next_timestamp_dtu)
{
	struct mcps802154_default_local *dlocal =
		scheduler_to_dlocal(scheduler);
	int r;

	r = mcps802154_schedule_set_start(
		schedule_update, schedule_update->expected_start_timestamp_dtu);
	/* Can not fail, only possible error is invalid parameters. */
	WARN_RETURN(r);

	r = mcps802154_schedule_recycle(schedule_update, 0,
					MCPS802154_DURATION_NO_CHANGE);
	/* Can not fail, only possible error is invalid parameters. */
	WARN_RETURN(r);

	r = mcps802154_schedule_add_region(schedule_update, &dlocal->region, 0,
					   0);

	return r;
}

static struct mcps802154_scheduler_ops mcps802154_default_region_scheduler = {
	.owner = THIS_MODULE,
	.name = "default",
	.open = mcps802154_default_scheduler_open,
	.close = mcps802154_default_scheduler_close,
	.update_schedule = mcps802154_default_scheduler_update_schedule,
};

int __init mcps802154_default_region_init(void)
{
	return mcps802154_scheduler_register(
		&mcps802154_default_region_scheduler);
}

void __exit mcps802154_default_region_exit(void)
{
	mcps802154_scheduler_unregister(&mcps802154_default_region_scheduler);
}
