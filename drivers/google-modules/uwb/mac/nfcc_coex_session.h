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
 * NFCC coexistence sessions management.
 */

#ifndef NET_MCPS802154_NFCC_COEX_SESSION_H
#define NET_MCPS802154_NFCC_COEX_SESSION_H

#include <linux/kernel.h>
#include <net/mcps802154_schedule.h>
#include <net/vendor_cmd.h>

#define NS_PER_SECOND 1000000000ull

/**
 * struct nfcc_coex_session_params - Session parameters.
 */
struct nfcc_coex_session_params {
	/**
	 * @time0_ns: Timestamp in nanoseconds in the ``CLOCK_MONOTONIC`` time.
	 */
	u64 time0_ns;
	/**
	 * @channel_number: Channel to use for the session, 5 or 9.
	 */
	u8 channel_number;
};

/**
 * struct nfcc_coex_session - Session information.
 */
struct nfcc_coex_session {
	/**
	 * @params: Session parameters, mostly read only while the session is
	 * active.
	 */
	struct nfcc_coex_session_params params;
	/**
	 * @event_portid: Port identifier to use for notifications.
	 */
	u32 event_portid;
	/**
	 * @get_access_info: Next access feedback get through a vendor command.
	 */
	struct dw3000_vendor_cmd_nfcc_coex_get_access_info get_access_info;
	/**
	 * @region_demand: Region access demand which contains start and duration.
	 */
	struct mcps802154_region_demand region_demand;
	/**
	 * @first_access: True on the first access.
	 */
	bool first_access;
};

/* Forward declaration. */
struct nfcc_coex_local;

/**
 * nfcc_coex_session_init() - Initialize session parameters to default value.
 * @local: NFCC coex context.
 */
void nfcc_coex_session_init(struct nfcc_coex_local *local);

/**
 * nfcc_coex_session_next() - Find the next session to use after the given timestamp.
 * @local: NFCC coex context.
 * @next_timestamp_dtu: Next start access opportunity.
 * @region_duration_dtu: Region duration, or 0 for endless region.
 *
 * Return: The session or NULL if none.
 */
struct nfcc_coex_session *nfcc_coex_session_next(struct nfcc_coex_local *local,
						 u32 next_timestamp_dtu,
						 int region_duration_dtu);

#endif /* NET_MCPS802154_NFCC_COEX_SESSION_H */
