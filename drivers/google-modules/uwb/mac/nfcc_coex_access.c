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
 * NFCC coexistence, access.
 */

#include "nfcc_coex_access.h"
#include "nfcc_coex_session.h"
#include "nfcc_coex_region.h"
#include "llhw-ops.h"

#include <linux/string.h>
#include <linux/ieee802154.h>
#include <net/mcps802154_frame.h>
#include <net/vendor_cmd.h>

#include "warn_return.h"

static void nfcc_coex_stop_by_vendor_cmd_failure(struct nfcc_coex_local *local)
{
	static const struct dw3000_vendor_cmd_nfcc_coex_get_access_info error = {
		.stop = 1
	};

	local->session.get_access_info = error;
	local->state = NFCC_COEX_STATE_STOPPING;
}

static int nfcc_coex_vendor_cmd(struct mcps802154_llhw *llhw, u32 subcmd,
				void *data, size_t data_len)
{
	struct mcps802154_local *local = llhw_to_local(llhw);
	/* Qorvo OUI in big endian. */
	static const u32 qorvo_oui = 0xc8b1ee00;

	return llhw_vendor_cmd(local, qorvo_oui, subcmd, data, data_len);
}

void nfcc_coex_access_done(struct mcps802154_access *access)
{
	struct nfcc_coex_local *local = access_to_local(access);
	struct nfcc_coex_session *session = &local->session;

	switch (local->state) {
	case NFCC_COEX_STATE_STOPPING:
		nfcc_coex_vendor_cmd(local->llhw,
				     DW3000_VENDOR_CMD_NFCC_COEX_STOP, NULL, 0);
		nfcc_coex_report(local);
		break;
	case NFCC_COEX_STATE_ACCESSING:
		if (session->get_access_info.stop ||
		    session->get_access_info.watchdog_timeout)
			local->state = NFCC_COEX_STATE_STOPPING;
		nfcc_coex_report(local);
		break;
	default:
		break;
	}
}

int nfcc_coex_handle(struct mcps802154_access *access)
{
	struct nfcc_coex_local *local = access_to_local(access);
	struct nfcc_coex_session *session = &local->session;
	struct dw3000_vendor_cmd_nfcc_coex_handle_access handle_access = {};
	int r;

	handle_access.start = session->first_access;
	handle_access.timestamp_dtu = access->timestamp_dtu;
	handle_access.duration_dtu = access->duration_dtu;
	handle_access.chan = session->params.channel_number;

	session->first_access = false;

	r = nfcc_coex_vendor_cmd(local->llhw,
				 DW3000_VENDOR_CMD_NFCC_COEX_HANDLE_ACCESS,
				 &handle_access, sizeof(handle_access));
	if (r)
		nfcc_coex_stop_by_vendor_cmd_failure(local);
	return r;
}

static int nfcc_coex_tx_done(struct mcps802154_access *access)
{
	struct nfcc_coex_local *local = access_to_local(access);
	struct nfcc_coex_session *session = &local->session;
	struct dw3000_vendor_cmd_nfcc_coex_get_access_info *get_access_info =
		&session->get_access_info;
	struct mcps802154_region_demand *rd = &session->region_demand;
	int r;

	r = nfcc_coex_vendor_cmd(
		local->llhw, DW3000_VENDOR_CMD_NFCC_COEX_GET_ACCESS_INFORMATION,
		get_access_info, sizeof(*get_access_info));
	if (r) {
		nfcc_coex_stop_by_vendor_cmd_failure(local);
		return r;
	}

	rd->timestamp_dtu = get_access_info->next_timestamp_dtu;
	rd->duration_dtu = get_access_info->next_duration_dtu;

	/* Request end of current access. */
	return 1;
}

struct mcps802154_access_vendor_ops nfcc_coex_ops = {
	.access_done = nfcc_coex_access_done,
	.handle = nfcc_coex_handle,
	.tx_done = nfcc_coex_tx_done,
};

static struct mcps802154_access *
nfcc_coex_access_nothing(struct nfcc_coex_local *local)
{
	struct mcps802154_access *access = &local->access;

	access->method = MCPS802154_ACCESS_METHOD_NOTHING;
	access->vendor_ops = &nfcc_coex_ops;
	access->duration_dtu = 0;

	return access;
}

static struct mcps802154_access *
nfcc_coex_access_controller(struct nfcc_coex_local *local,
			    struct nfcc_coex_session *session)
{
	struct mcps802154_access *access = &local->access;

	access->method = MCPS802154_ACCESS_METHOD_VENDOR;
	access->vendor_ops = &nfcc_coex_ops;
	access->duration_dtu = session->region_demand.duration_dtu;
	access->timestamp_dtu = session->region_demand.timestamp_dtu;
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
	struct nfcc_coex_session *session;

	WARN_ON(next_in_region_dtu);

	/* Get unique session. */
	session = nfcc_coex_session_next(local, next_timestamp_dtu,
					 region_duration_dtu);

	if (!session) {
		local->state = NFCC_COEX_STATE_UNUSED;
		return nfcc_coex_access_nothing(local);
	} else {
		local->state = NFCC_COEX_STATE_ACCESSING;
		return nfcc_coex_access_controller(local, session);
	}
}
