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

/**
 * struct mcps802154_on_demand_local - local context for on demand scheduler.
 */
struct mcps802154_on_demand_local {
	/**
	 * @scheduler: Common scheduler context.
	 */
	struct mcps802154_scheduler scheduler;
	/**
	 * @llhw: Low layer hardware attached.
	 */
	struct mcps802154_llhw *llhw;
};

static inline struct mcps802154_on_demand_local *
scheduler_to_plocal(const struct mcps802154_scheduler *scheduler)
{
	return container_of(scheduler, struct mcps802154_on_demand_local,
			    scheduler);
}

static struct mcps802154_scheduler *
mcps802154_on_demand_scheduler_open(struct mcps802154_llhw *llhw)
{
	struct mcps802154_on_demand_local *plocal;

	plocal = kmalloc(sizeof(*plocal), GFP_KERNEL);
	if (!plocal)
		return NULL;
	plocal->llhw = llhw;
	plocal->scheduler.n_regions = 0;
	return &plocal->scheduler;
}

static void
mcps802154_on_demand_scheduler_close(struct mcps802154_scheduler *scheduler)
{
	struct mcps802154_on_demand_local *plocal =
		scheduler_to_plocal(scheduler);

	kfree(plocal);
}

static int mcps802154_on_demand_scheduler_update_schedule(
	struct mcps802154_scheduler *scheduler,
	const struct mcps802154_schedule_update *schedule_update,
	u32 next_timestamp_dtu)
{
	struct mcps802154_on_demand_local *plocal =
		scheduler_to_plocal(scheduler);
	struct mcps802154_region_demand demand;
	struct mcps802154_region *region, *next_region = NULL;
	struct list_head *regions;
	int max_duration_dtu = 0;
	u32 start_dtu;
	int r;

	mcps802154_schedule_get_regions(plocal->llhw, &regions);

	list_for_each_entry (region, regions, ca_entry) {
		struct mcps802154_region_demand candidate = {};

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
		    (!candidate.max_duration_dtu ||
		     is_before_dtu(next_timestamp_dtu + max_duration_dtu,
				   candidate.timestamp_dtu +
					   candidate.max_duration_dtu)))
			candidate.max_duration_dtu = max_duration_dtu -
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
					   start_dtu, demand.max_duration_dtu);

	return r;
}

static struct mcps802154_scheduler_ops
	mcps802154_on_demand_scheduler_scheduler = {
		.owner = THIS_MODULE,
		.name = "on_demand",
		.open = mcps802154_on_demand_scheduler_open,
		.close = mcps802154_on_demand_scheduler_close,
		.set_parameters = NULL, /* No scheduler parameters for now. */
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
