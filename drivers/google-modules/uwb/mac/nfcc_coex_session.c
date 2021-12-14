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

#include "nfcc_coex_session.h"
#include "nfcc_coex_region.h"

#include <linux/errno.h>
#include <linux/ieee802154.h>
#include <linux/string.h>

void nfcc_coex_session_init(struct nfcc_coex_local *local)
{
	struct nfcc_coex_session_params *p = &local->session.params;

	memset(p, 0, sizeof(*p));
	p->time0_ns = (NS_PER_SECOND * local->llhw->anticip_dtu) /
		      local->llhw->dtu_freq_hz;
}

static void nfcc_coex_session_update(struct nfcc_coex_session *session,
				     u32 next_timestamp_dtu,
				     int region_duration_dtu)
{
	s32 diff_dtu =
		session->region_demand.timestamp_dtu - next_timestamp_dtu;

	if (diff_dtu < 0) {
		session->region_demand.timestamp_dtu = next_timestamp_dtu;
#if 0
		/**
		 * TODO: Update duration with futur scheduler (UWB-1101)
		 * With endless scheduler, region_duration_dtu is always 0.
		 */
		if (region_duration_dtu &&
		    session->region_demand.duration_dtu) {
			session->region_demand.duration_dtu =
				min(session->region_demand.duration_dtu,
				    region_duration_dtu);
		} else if (region_duration_dtu) {
			session->region_demand.duration_dtu =
				region_duration_dtu;
		}
#endif
	}
}

struct nfcc_coex_session *nfcc_coex_session_next(struct nfcc_coex_local *local,
						 u32 next_timestamp_dtu,
						 int region_duration_dtu)
{
	struct nfcc_coex_session *session;

	switch (local->state) {
	case NFCC_COEX_STATE_STARTED:
	case NFCC_COEX_STATE_ACCESSING:
		/* Get unique session. */
		session = &local->session;

		nfcc_coex_session_update(session, next_timestamp_dtu,
					 region_duration_dtu);
		return session;

	default:
		return NULL;
	}
}
