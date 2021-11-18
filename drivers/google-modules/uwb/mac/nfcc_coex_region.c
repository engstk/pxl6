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
 * 802.15.4 mac common part sublayer, nfcc_coex ranging region.
 */

#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/math64.h>

#include <linux/netdevice.h>

#include <net/mcps802154_schedule.h>
#include "net/nfcc_coex_region_nl.h"

#include "nfcc_coex_region.h"
#include "nfcc_coex_region_call.h"
#include "nfcc_coex_access.h"
#include "nfcc_coex_session.h"

static struct mcps802154_region_ops nfcc_coex_region_ops;

static struct mcps802154_region *nfcc_coex_open(struct mcps802154_llhw *llhw)
{
	struct nfcc_coex_local *local;

	local = kmalloc(sizeof(*local), GFP_KERNEL);
	if (!local)
		return NULL;

	local->llhw = llhw;
	local->region.ops = &nfcc_coex_region_ops;
	local->state = NFCC_COEX_STATE_UNUSED;

	return &local->region;
}

static void nfcc_coex_close(struct mcps802154_region *region)
{
	struct nfcc_coex_local *local = region_to_local(region);

	kfree(local);
}

static int nfcc_coex_call(struct mcps802154_region *region, u32 call_id,
			  const struct nlattr *attrs,
			  const struct genl_info *info)
{
	struct nfcc_coex_local *local = region_to_local(region);

	switch (call_id) {
	case NFCC_COEX_CALL_CCC_SESSION_START:
	case NFCC_COEX_CALL_CCC_SESSION_STOP:
		return nfcc_coex_session_control(local, call_id, attrs, info);
	default:
		return -EINVAL;
	}
}

void nfcc_coex_report(struct nfcc_coex_local *local)
{
	struct nfcc_coex_session *session = &local->session;
	const struct dw3000_vendor_cmd_nfcc_coex_get_access_info
		*get_access_info = &session->get_access_info;
	struct sk_buff *msg;
	int r;

	msg = mcps802154_region_event_alloc_skb(
		local->llhw, &local->region,
		NFCC_COEX_CALL_CCC_SESSION_NOTIFICATION, session->event_portid,
		NLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (!msg)
		return;

#define P(attr, type, value)                                            \
	do {                                                            \
		if (nla_put_##type(msg, NFCC_COEX_CALL_ATTR_CCC_##attr, \
				   value)) {                            \
			goto nla_put_failure;                           \
		}                                                       \
	} while (0)

	P(WATCHDOG_TIMEOUT, u8, get_access_info->watchdog_timeout);
	P(STOPPED, u8, get_access_info->stop);
#undef P

	r = mcps802154_region_event(local->llhw, msg);
	if (r == -ECONNREFUSED)
		/* TODO stop. */
		;
	return;

nla_put_failure:
	kfree_skb(msg);
}

static struct mcps802154_region_ops nfcc_coex_region_ops = {
	/* clang-format off */
	.owner = THIS_MODULE,
	.name = "nfcc_coex",
	.open = nfcc_coex_open,
	.close = nfcc_coex_close,
	.call = nfcc_coex_call,
	.get_access = nfcc_coex_get_access,
	/* clang-format on */
};

int __init nfcc_coex_region_init(void)
{
	return mcps802154_region_register(&nfcc_coex_region_ops);
}

void __exit nfcc_coex_region_exit(void)
{
	mcps802154_region_unregister(&nfcc_coex_region_ops);
}

module_init(nfcc_coex_region_init);
module_exit(nfcc_coex_region_exit);

MODULE_DESCRIPTION("Vendor Region for IEEE 802.15.4 MCPS");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
