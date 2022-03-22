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

#include "nfcc_coex_access.h"
#include "nfcc_coex_session.h"
#include "nfcc_coex_region.h"
#include "nfcc_coex_trace.h"
#include "llhw-ops.h"
#include "mcps802154_qorvo.h"

#include <linux/string.h>
#include <linux/ieee802154.h>
#include <net/mcps802154_frame.h>
#include <net/vendor_cmd.h>

#include "warn_return.h"

static void nfcc_coex_access_done(struct mcps802154_access *access, int error)
{
	struct nfcc_coex_local *local = access_to_local(access);
	struct nfcc_coex_session *session = &local->session;

	if (error) {
		const struct dw3000_vendor_cmd_nfcc_coex_get_access_info stop = {
			.stop = true,
		};

		local->session.get_access_info = stop;
		nfcc_coex_set_state(local, NFCC_COEX_STATE_STOPPING);
	}

	switch (session->state) {
	case NFCC_COEX_STATE_STOPPING:
		mcps802154_vendor_cmd(local->llhw, VENDOR_QORVO_OUI,
				      DW3000_VENDOR_CMD_NFCC_COEX_STOP, NULL,
				      0);
		nfcc_coex_report(local);
		session->started = false;
		break;
	case NFCC_COEX_STATE_ACCESSING:
		if (session->get_access_info.stop ||
		    session->get_access_info.watchdog_timeout)
			session->started = false;
		nfcc_coex_report(local);
		break;
	default:
		WARN_UNREACHABLE_DEFAULT();
	}
	nfcc_coex_set_state(local, NFCC_COEX_STATE_IDLE);
}

static int nfcc_coex_handle(struct mcps802154_access *access)
{
	struct nfcc_coex_local *local = access_to_local(access);
	struct nfcc_coex_session *session = &local->session;
	struct dw3000_vendor_cmd_nfcc_coex_handle_access handle_access = {};

	handle_access.start = session->first_access;
	handle_access.timestamp_dtu = access->timestamp_dtu;
	handle_access.duration_dtu = access->duration_dtu;
	handle_access.chan = session->params.channel_number;

	nfcc_coex_set_state(local, NFCC_COEX_STATE_ACCESSING);
	session->first_access = false;

	return mcps802154_vendor_cmd(local->llhw, VENDOR_QORVO_OUI,
				     DW3000_VENDOR_CMD_NFCC_COEX_HANDLE_ACCESS,
				     &handle_access, sizeof(handle_access));
}

static int nfcc_coex_tx_done(struct mcps802154_access *access)
{
	struct nfcc_coex_local *local = access_to_local(access);
	struct nfcc_coex_session *session = &local->session;
	struct dw3000_vendor_cmd_nfcc_coex_get_access_info *get_access_info =
		&session->get_access_info;
	struct mcps802154_region_demand *rd = &session->region_demand;
	int r;

	r = mcps802154_vendor_cmd(
		local->llhw, VENDOR_QORVO_OUI,
		DW3000_VENDOR_CMD_NFCC_COEX_GET_ACCESS_INFORMATION,
		get_access_info, sizeof(*get_access_info));
	if (r)
		return r;

	/* Update region demand for next access. */
	rd->timestamp_dtu = get_access_info->next_timestamp_dtu;
	rd->duration_dtu = get_access_info->next_duration_dtu;
	/* Request end of current access. */
	return 1;
}

struct mcps802154_access_vendor_ops nfcc_coex_ops = {
	.common = {
		.access_done = nfcc_coex_access_done,
	},
	.handle = nfcc_coex_handle,
	.tx_done = nfcc_coex_tx_done,
};

static struct mcps802154_access *
nfcc_coex_access_controller(struct nfcc_coex_local *local,
			    struct nfcc_coex_session *session)
{
	struct mcps802154_access *access = &local->access;
	const struct mcps802154_region_demand *rd = &session->region_demand;

	access->method = MCPS802154_ACCESS_METHOD_VENDOR;
	access->vendor_ops = &nfcc_coex_ops;
	access->duration_dtu = rd->duration_dtu;
	access->timestamp_dtu = rd->timestamp_dtu;
	access->n_frames = 0;
	access->frames = NULL;

	return access;
}

struct mcps802154_access *nfcc_coex_get_access(struct mcps802154_region *region,
					       u32 next_timestamp_dtu,
					       int next_in_region_dtu,
					       int region_duration_dtu)
{
	struct nfcc_coex_local *local = region_to_local(region);
	struct nfcc_coex_session *session = &local->session;

	if (session->started) {
		nfcc_coex_session_update(session, next_timestamp_dtu,
					 region_duration_dtu);
		return nfcc_coex_access_controller(local, session);
	}
	return NULL;
}
