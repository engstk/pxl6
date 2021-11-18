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
 * 802.15.4 mac common part sublayer, channel access.
 *
 */

#include <linux/errno.h>
#include <linux/string.h>

#include "mcps802154_i.h"
#include "schedulers.h"
#include "trace.h"

static void mcps802154_ca_close_scheduler(struct mcps802154_local *local)
{
	mcps802154_schedule_clear(local);
	if (local->ca.scheduler) {
		mcps802154_scheduler_close(local->ca.scheduler);
		local->ca.scheduler = NULL;
	}
}

void mcps802154_ca_init(struct mcps802154_local *local)
{
	local->ca.held = false;
	local->ca.reset = false;
	skb_queue_head_init(&local->ca.queue);
	atomic_set(&local->ca.n_queued, 0);
	local->ca.retries = 0;
}

void mcps802154_ca_uninit(struct mcps802154_local *local)
{
	skb_queue_purge(&local->ca.queue);
	atomic_set(&local->ca.n_queued, 0);
}

int mcps802154_ca_start(struct mcps802154_local *local)
{
	int r;

	if (!local->ca.scheduler) {
		r = mcps802154_ca_set_scheduler(local, "default", NULL, NULL);
		if (r)
			return r;
	}

	local->start_stop_request = true;
	local->fproc.state->schedule_change(local);

	return local->started ? 0 : -EIO;
}

void mcps802154_ca_stop(struct mcps802154_local *local)
{
	local->start_stop_request = false;
	local->fproc.state->schedule_change(local);
}

void mcps802154_ca_close(struct mcps802154_local *local)
{
	mcps802154_ca_close_scheduler(local);
}

int mcps802154_ca_set_scheduler(struct mcps802154_local *local,
				const char *name,
				const struct nlattr *params_attr,
				struct netlink_ext_ack *extack)
{
	struct mcps802154_scheduler *scheduler;

	trace_ca_set_scheduler(local, name);

	if (local->started)
		return -EBUSY;
	/* Open new scheduler. */
	scheduler = mcps802154_scheduler_open(local, name, params_attr, extack);
	if (!scheduler)
		return -EINVAL;
	/* Close previous scheduler and set the new one. */
	mcps802154_ca_close_scheduler(local);
	local->ca.scheduler = scheduler;

	return 0;
}

int mcps802154_ca_scheduler_set_parameters(struct mcps802154_local *local,
					   const char *name,
					   const struct nlattr *params_attr,
					   struct netlink_ext_ack *extack)
{
	struct mcps802154_scheduler *scheduler;

	trace_ca_set_scheduler_parameters(local, name);

	scheduler = local->ca.scheduler;

	/* Check scheduler is the correct one. */
	if (!scheduler || strcmp(scheduler->ops->name, name))
		return -EINVAL;

	return mcps802154_scheduler_set_parameters(scheduler, params_attr,
						   extack);
}

int mcps802154_ca_scheduler_set_region_parameters(
	struct mcps802154_local *local, const char *scheduler_name,
	u32 region_id, const char *region_name,
	const struct nlattr *params_attr, struct netlink_ext_ack *extack)
{
	struct mcps802154_scheduler *scheduler;

	trace_ca_scheduler_set_region_parameters(local, scheduler_name,
						 region_id, region_name);

	scheduler = local->ca.scheduler;
	/* Check scheduler is the correct one. */
	if (!scheduler || strcmp(scheduler->ops->name, scheduler_name) ||
	    !region_name)
		return -EINVAL;
	return mcps802154_scheduler_set_region_parameters(
		scheduler, region_id, region_name, params_attr, extack);
}

int mcps802154_ca_scheduler_call(struct mcps802154_local *local,
				 const char *scheduler_name, u32 call_id,
				 const struct nlattr *params_attr,
				 const struct genl_info *info)
{
	struct mcps802154_scheduler *scheduler;

	trace_ca_scheduler_call(local, scheduler_name, call_id);

	scheduler = local->ca.scheduler;
	/* Check scheduler is the correct one. */
	if (!scheduler || strcmp(scheduler->ops->name, scheduler_name))
		return -EINVAL;
	return mcps802154_scheduler_call(scheduler, call_id, params_attr, info);
}

int mcps802154_ca_scheduler_call_region(struct mcps802154_local *local,
					const char *scheduler_name,
					u32 region_id, const char *region_name,
					u32 call_id,
					const struct nlattr *params_attr,
					const struct genl_info *info)
{
	struct mcps802154_scheduler *scheduler;

	trace_ca_scheduler_call_region(local, scheduler_name, region_id,
				       region_name, call_id);

	scheduler = local->ca.scheduler;
	/* Check scheduler is the correct one. */
	if (!scheduler || strcmp(scheduler->ops->name, scheduler_name) ||
	    !region_name)
		return -EINVAL;
	return mcps802154_scheduler_call_region(
		scheduler, region_id, region_name, call_id, params_attr, info);
}

/**
 * mcps802154_ca_next_region() - Check the current region is still valid, if not
 * change region.
 * @local: MCPS private data.
 * @next_timestamp_dtu: Date of next access opportunity.
 *
 * Return: 0 if unchanged, 1 if changed, or negative error.
 */
static int mcps802154_ca_next_region(struct mcps802154_local *local,
				     u32 next_timestamp_dtu)
{
	struct mcps802154_schedule *sched = &local->ca.schedule;
	struct mcps802154_schedule_region *sched_region;
	int next_dtu = next_timestamp_dtu - sched->start_timestamp_dtu;
	bool changed = 0;

	sched_region = &sched->regions[sched->current_index];

	/* If not an endless region, need to test if still inside. */
	while (sched_region->duration_dtu != 0 &&
	       next_dtu - sched_region->start_dtu >=
		       sched_region->duration_dtu) {
		sched->current_index++;
		changed = 1;

		/* No more region, need a new schedule. */
		if (sched->current_index >= sched->n_regions) {
			int r = mcps802154_schedule_update(local,
							   next_timestamp_dtu);
			if (r)
				return -EOPNOTSUPP;
			return 1;
		}

		sched_region = &sched->regions[sched->current_index];
	}

	return changed;
}

struct mcps802154_access *
mcps802154_ca_get_access(struct mcps802154_local *local, u32 next_timestamp_dtu)
{
	struct mcps802154_schedule *sched = &local->ca.schedule;
	struct mcps802154_schedule_region *sched_region;
	struct mcps802154_region *region;
	struct mcps802154_access *access;
	int next_in_region_dtu, region_duration_dtu;
	int r, changed;

	local->ca.held = false;

	trace_ca_get_access(local, next_timestamp_dtu);

	if (local->ca.reset) {
		mcps802154_schedule_clear(local);
		local->ca.reset = false;
	}
	/* Need a schedule. */
	if (!sched->n_regions) {
		r = mcps802154_schedule_update(local, next_timestamp_dtu);
		if (r)
			return NULL;
		changed = 1;
	} else {
	get_region:
		/* Need a region. */
		changed = mcps802154_ca_next_region(local, next_timestamp_dtu);
		if (changed < 0)
			return NULL;
	}

	sched_region = &sched->regions[sched->current_index];
	region = sched_region->region;
	/* If the region has changed, access date may be postponed. */
	if (changed) {
		u32 region_start_timestamp_dtu =
			sched->start_timestamp_dtu + sched_region->start_dtu;
		if (is_before_dtu(next_timestamp_dtu,
				  region_start_timestamp_dtu))
			next_timestamp_dtu = region_start_timestamp_dtu;
	}
	region_duration_dtu = sched_region->duration_dtu;
	/* Get access. */
	if (region_duration_dtu)
		next_in_region_dtu = next_timestamp_dtu -
				     sched->start_timestamp_dtu -
				     sched_region->start_dtu;
	else
		next_in_region_dtu = 0;
	trace_region_get_access(local, region, next_timestamp_dtu,
				next_in_region_dtu, region_duration_dtu);
	access = region->ops->get_access(region, next_timestamp_dtu,
					 next_in_region_dtu,
					 region_duration_dtu);

	/* If no access is granted, look for next region.
	 * This is only accepted when we are in the middle of a region. */
	if (!access && next_in_region_dtu) {
		next_timestamp_dtu = next_timestamp_dtu +
				     sched_region->duration_dtu -
				     next_in_region_dtu;
		goto get_region;
	}

	return access;
}

void mcps802154_ca_may_reschedule(struct mcps802154_local *local)
{
	if (!local->ca.held)
		local->fproc.state->schedule_change(local);
}

void mcps802154_ca_access_hold(struct mcps802154_local *local)
{
	local->ca.held = true;
}

void mcps802154_ca_invalidate_schedule(struct mcps802154_local *local)
{
	local->ca.reset = true;

	local->fproc.state->schedule_change(local);
}
