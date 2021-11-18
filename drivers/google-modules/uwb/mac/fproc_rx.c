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
 * 802.15.4 mac common part sublayer, FProc state RX.
 *
 */
#include <linux/errno.h>

#include "mcps802154_i.h"
#include "llhw-ops.h"

static void
mcps802154_fproc_rx_wait_tx_done_tx_done(struct mcps802154_local *local)
{
	/* End current access and ask for next one. */
	mcps802154_fproc_access_done(local);
	mcps802154_fproc_access_now(local);
}

static void
mcps802154_fproc_rx_wait_tx_done_schedule_change(struct mcps802154_local *local)
{
	/* Nothing, wait for tx_done. */
}

static const struct mcps802154_fproc_state mcps802154_fproc_rx_wait_tx_done = {
	.name = "rx_wait_tx_done",
	.tx_done = mcps802154_fproc_rx_wait_tx_done_tx_done,
	.schedule_change = mcps802154_fproc_rx_wait_tx_done_schedule_change,
};

static void mcps802154_fproc_rx_rx_frame(struct mcps802154_local *local)
{
	struct mcps802154_access *access = local->fproc.access;
	int r;

	/* Read frame. */
	struct sk_buff *skb = NULL;
	struct mcps802154_rx_frame_info info = {
		.flags = MCPS802154_RX_FRAME_INFO_LQI,
	};
	r = llhw_rx_get_frame(local, &skb, &info);
	if (!r) {
		access->ops->rx_frame(access, 0, skb, &info);
		/* If auto-ack was sent, wait for tx_done. */
		if (info.flags & MCPS802154_RX_FRAME_INFO_AACK) {
			mcps802154_fproc_change_state(
				local, &mcps802154_fproc_rx_wait_tx_done);
			return;
		}
	}
	mcps802154_fproc_access_done(local);

	if (r && r != -EBUSY)
		mcps802154_fproc_broken_handle(local);
	else
		/* Next access. */
		mcps802154_fproc_access_now(local);
}

static void mcps802154_fproc_rx_rx_error(struct mcps802154_local *local,
					 enum mcps802154_rx_error_type error)
{
	mcps802154_fproc_access_done(local);

	/* Next access. */
	mcps802154_fproc_access_now(local);
}

static void mcps802154_fproc_rx_schedule_change(struct mcps802154_local *local)
{
	int r;

	/* Disable RX. */
	r = llhw_rx_disable(local);
	if (r == -EBUSY)
		/* Wait for RX result. */
		return;

	mcps802154_fproc_access_done(local);
	if (r)
		mcps802154_fproc_broken_handle(local);
	else
		/* Next access. */
		mcps802154_fproc_access_now(local);
}

static const struct mcps802154_fproc_state mcps802154_fproc_rx = {
	.name = "rx",
	.rx_frame = mcps802154_fproc_rx_rx_frame,
	.rx_error = mcps802154_fproc_rx_rx_error,
	.schedule_change = mcps802154_fproc_rx_schedule_change,
};

int mcps802154_fproc_rx_handle(struct mcps802154_local *local,
			       struct mcps802154_access *access)
{
	int r;
	struct mcps802154_rx_info rx_info = {
		.flags = MCPS802154_RX_INFO_AACK,
		.timeout_dtu = -1,
	};
	r = llhw_rx_enable(local, &rx_info, 0);
	if (r)
		return r;

	mcps802154_fproc_change_state(local, &mcps802154_fproc_rx);

	return 0;
}
