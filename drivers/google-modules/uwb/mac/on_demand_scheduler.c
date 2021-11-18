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
 * Qorvo.
 * Please contact Qorvo to inquire about licensing terms.
 *
 * 802.15.4 mac common part sublayer, on_demand scheduler.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <net/mcps802154_schedule.h>

#include "mcps802154_i.h"
#include "on_demand_scheduler.h"
#include "warn_return.h"

#define REGIONS_ID_MAX 2

/**
 * struct mcps802154_private_local - local context for on demand scheduler.
 */
struct mcps802154_private_local {
	/**
	 * @scheduler: Common scheduler context.
	 */
	struct mcps802154_scheduler scheduler;
	/**
	 * @llhw: Low layer hardware attached.
	 */
	struct mcps802154_llhw *llhw;
	/**
	 * @regions: Array of regions.
	 */
	struct mcps802154_region *regions[REGIONS_ID_MAX];
};

static inline struct mcps802154_private_local *
scheduler_to_plocal(const struct mcps802154_scheduler *scheduler)
{
	return container_of(scheduler, struct mcps802154_private_local,
			    scheduler);
}

static struct mcps802154_scheduler *
mcps802154_on_demand_scheduler_open(struct mcps802154_llhw *llhw)
{
	struct mcps802154_private_local *plocal;

	plocal = kmalloc(sizeof(*plocal), GFP_KERNEL);
	if (!plocal)
		return NULL;
	plocal->llhw = llhw;
	memset(plocal->regions, 0, sizeof(plocal->regions));
	return &plocal->scheduler;
}

static void
mcps802154_on_demand_scheduler_close(struct mcps802154_scheduler *scheduler)
{
	struct mcps802154_private_local *plocal =
		scheduler_to_plocal(scheduler);
	int i;

	for (i = 0; i < REGIONS_ID_MAX; i++) {
		struct mcps802154_region *region = plocal->regions[i];
		if (region)
			mcps802154_region_close(plocal->llhw, region);
	}

	kfree(plocal);
}

static void mcps802154_on_demand_scheduler_notify_stop(
	struct mcps802154_scheduler *scheduler)
{
	struct mcps802154_private_local *plocal =
		scheduler_to_plocal(scheduler);
	int i;

	for (i = 0; i < REGIONS_ID_MAX; i++) {
		struct mcps802154_region *region = plocal->regions[i];

		if (!region)
			continue;

		mcps802154_region_notify_stop(plocal->llhw, region);
	}
}

static int mcps802154_on_demand_scheduler_set_region_parameters(
	struct mcps802154_scheduler *scheduler, u32 region_id,
	const char *region_name, const struct nlattr *attrs,
	struct netlink_ext_ack *extack)
{
	struct mcps802154_private_local *plocal =
		scheduler_to_plocal(scheduler);

	if (region_id >= REGIONS_ID_MAX)
		return -ENOENT;

	/* Close current region. */
	if (plocal->regions[region_id])
		mcps802154_region_close(plocal->llhw,
					plocal->regions[region_id]);

	/* Open region, and set its parameters. */
	plocal->regions[region_id] = mcps802154_region_open(
		plocal->llhw, region_name, attrs, extack);

	if (!plocal->regions[region_id])
		return -EINVAL;

	return 0;
}

static int mcps802154_on_demand_scheduler_call_region(
	struct mcps802154_scheduler *scheduler, u32 region_id,
	const char *region_name, u32 call_id, const struct nlattr *attrs,
	const struct genl_info *info)
{
	struct mcps802154_private_local *plocal =
		scheduler_to_plocal(scheduler);
	struct mcps802154_region *region;

	if (region_id >= REGIONS_ID_MAX)
		return -ENOENT;

	region = plocal->regions[region_id];
	if (!region)
		return -ENOENT;

	if (strcmp(region_name, region->ops->name))
		return -EINVAL;

	return mcps802154_region_call(plocal->llhw, region, call_id, attrs,
				      info);
}

static int mcps802154_on_demand_scheduler_update_schedule(
	struct mcps802154_scheduler *scheduler,
	const struct mcps802154_schedule_update *schedule_update,
	u32 next_timestamp_dtu)
{
	struct mcps802154_private_local *plocal =
		scheduler_to_plocal(scheduler);
	struct mcps802154_region_demand demand;
	struct mcps802154_region *next_region = NULL;
	int max_duration_dtu = 0;
	u32 start_dtu;
	int r, i;

	for (i = 0; i < REGIONS_ID_MAX; i++) {
		struct mcps802154_region *region = plocal->regions[i];
		struct mcps802154_region_demand candidate = {};

		if (!region)
			continue;

		r = mcps802154_region_get_demand(
			plocal->llhw, region, next_timestamp_dtu, &candidate);
		switch (r) {
		case 0:
			/* The region doesn't have a demand. */
			continue;
		case 1:
			/* The region have a demand. */
			break;
		default:
			return r;
		}

		/* Reduce duration of candidate region with less priority. */
		if (max_duration_dtu &&
		    (!candidate.duration_dtu ||
		     is_before_dtu(next_timestamp_dtu + max_duration_dtu,
				   candidate.timestamp_dtu +
					   candidate.duration_dtu)))
			candidate.duration_dtu = max_duration_dtu -
						 candidate.timestamp_dtu +
						 next_timestamp_dtu;

		/* Arbitrate between regions. */
		if (!next_region || is_before_dtu(candidate.timestamp_dtu,
						  demand.timestamp_dtu)) {
			next_region = region;
			demand = candidate;
			/* Is there some time remaining for a region with
			 * less priority? */
			if (!is_before_dtu(next_timestamp_dtu,
					   demand.timestamp_dtu))
				break;
			else
				max_duration_dtu = demand.timestamp_dtu -
						   next_timestamp_dtu;
		}
	}

	/* FIXME:
	 * - With duration_dtu = 0 (Infinite), the region interleaving is blocked.
	 * - Without candidate, the CA will go in stop instead of idle.
	 *   Which block the session start.
	 *
	 * Workaround:
	 * Select the first region, and set the duration to idle time.
	 * So the scheduler can be set, and the region start later.
	 */
	if (!next_region) {
		for (i = 0; i < REGIONS_ID_MAX; i++) {
			struct mcps802154_region *region = plocal->regions[i];

			if (!region)
				continue;

			next_region = region;
			demand.timestamp_dtu = next_timestamp_dtu;
			demand.duration_dtu = plocal->llhw->idle_dtu + 1;
			break;
		}
	}

	if (!next_region)
		return -ENOENT;

	start_dtu = demand.timestamp_dtu -
		    schedule_update->expected_start_timestamp_dtu;

	r = mcps802154_schedule_set_start(
		schedule_update, schedule_update->expected_start_timestamp_dtu);
	/* Can not fail, only possible error is invalid parameters. */
	WARN_RETURN(r);

	r = mcps802154_schedule_recycle(schedule_update, 0,
					MCPS802154_DURATION_NO_CHANGE);
	/* Can not fail, only possible error is invalid parameters. */
	WARN_RETURN(r);

	r = mcps802154_schedule_add_region(schedule_update, next_region,
					   start_dtu, demand.duration_dtu);

	return r;
}

static struct mcps802154_scheduler_ops
	mcps802154_on_demand_scheduler_scheduler = {
		.owner = THIS_MODULE,
		.name = "on_demand",
		.open = mcps802154_on_demand_scheduler_open,
		.close = mcps802154_on_demand_scheduler_close,
		.notify_stop = mcps802154_on_demand_scheduler_notify_stop,
		.set_parameters = NULL, /* No scheduler parameters for now. */
		.set_region_parameters =
			mcps802154_on_demand_scheduler_set_region_parameters,
		.call_region = mcps802154_on_demand_scheduler_call_region,
		.update_schedule =
			mcps802154_on_demand_scheduler_update_schedule,
	};

int __init mcps802154_on_demand_scheduler_init(void)
{
	return mcps802154_scheduler_register(
		&mcps802154_on_demand_scheduler_scheduler);
}

void __exit mcps802154_on_demand_scheduler_exit(void)
{
	mcps802154_scheduler_unregister(
		&mcps802154_on_demand_scheduler_scheduler);
}
