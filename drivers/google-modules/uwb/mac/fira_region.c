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
 * 802.15.4 mac common part sublayer, fira ranging region.
 *
 */

#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/math64.h>

#include <linux/netdevice.h>

#include <net/mcps802154_schedule.h>
#include <net/fira_region_nl.h>

#include "fira_region.h"
#include "fira_region_call.h"
#include "fira_access.h"
#include "fira_session.h"

#include "warn_return.h"

static struct mcps802154_region_ops fira_region_ops;

static struct mcps802154_region *fira_open(struct mcps802154_llhw *llhw)
{
	struct fira_local *local;

	local = kmalloc(sizeof(*local), GFP_KERNEL);
	if (!local)
		return NULL;
	local->llhw = llhw;
	local->region.ops = &fira_region_ops;
	INIT_LIST_HEAD(&local->inactive_sessions);
	INIT_LIST_HEAD(&local->active_sessions);
	return &local->region;
}

static void fira_close(struct mcps802154_region *region)
{
	struct fira_local *local = region_to_local(region);

	kfree_sensitive(local);
}

static int fira_call(struct mcps802154_region *region, u32 call_id,
		     const struct nlattr *attrs, const struct genl_info *info)
{
	struct fira_local *local = region_to_local(region);

	switch (call_id) {
	case FIRA_CALL_GET_CAPABILITIES:
		return fira_get_capabilities(local, info);
	case FIRA_CALL_SESSION_INIT:
	case FIRA_CALL_SESSION_START:
	case FIRA_CALL_SESSION_STOP:
	case FIRA_CALL_SESSION_DEINIT:
	case FIRA_CALL_SESSION_SET_PARAMS:
	case FIRA_CALL_NEW_CONTROLEE:
	case FIRA_CALL_DEL_CONTROLEE:
		return fira_session_control(local, call_id, attrs, info);
	default:
		return -EINVAL;
	}
}

static int fira_report_local_aoa(struct fira_local *local, struct sk_buff *msg,
				 const struct fira_local_aoa_info *info)
{
#define A(x) FIRA_RANGING_DATA_MEASUREMENTS_AOA_ATTR_##x
	if (nla_put_u8(msg, A(RX_ANTENNA_PAIR), info->rx_ant_pair))
		goto nla_put_failure;
	if (nla_put_s16(msg, A(AOA_2PI), info->aoa_2pi))
		goto nla_put_failure;
	if (nla_put_s16(msg, A(PDOA_2PI), info->pdoa_2pi))
		goto nla_put_failure;
	if (nla_put_u8(msg, A(AOA_FOM), info->aoa_fom))
		goto nla_put_failure;
#undef A
	return 0;
nla_put_failure:
	return -EMSGSIZE;
}

static int fira_report_measurement(struct fira_local *local,
				   struct sk_buff *msg,
				   const struct fira_ranging_info *ranging_info)
{
	struct nlattr *aoa;
#define A(x) FIRA_RANGING_DATA_MEASUREMENTS_ATTR_##x

	if (nla_put_u16(msg, A(SHORT_ADDR), ranging_info->short_addr) ||
	    nla_put_u8(msg, A(STATUS), ranging_info->failed ? 1 : 0))
		goto nla_put_failure;

	if (ranging_info->failed)
		return 0;

	if (ranging_info->tof_present) {
		static const u64 speed_of_light_mm_per_s = 299702547000ull;
		u32 distance_mm = div64_u64(
			ranging_info->tof_rctu * speed_of_light_mm_per_s,
			(u64)local->llhw->dtu_freq_hz * local->llhw->dtu_rctu);
		if (nla_put_u32(msg, A(DISTANCE_MM), distance_mm))
			goto nla_put_failure;
	}

	if (ranging_info->local_aoa.present) {
		aoa = nla_nest_start(msg, A(LOCAL_AOA));
		if (!aoa)
			goto nla_put_failure;
		if (fira_report_local_aoa(local, msg, &ranging_info->local_aoa))
			goto nla_put_failure;
		nla_nest_end(msg, aoa);
	}
	if (ranging_info->local_aoa_azimuth.present) {
		aoa = nla_nest_start(msg, A(LOCAL_AOA_AZIMUTH));
		if (!aoa)
			goto nla_put_failure;
		if (fira_report_local_aoa(local, msg,
					  &ranging_info->local_aoa_azimuth))
			goto nla_put_failure;
		nla_nest_end(msg, aoa);
	}
	if (ranging_info->local_aoa_elevation.present) {
		aoa = nla_nest_start(msg, A(LOCAL_AOA_ELEVATION));
		if (!aoa)
			goto nla_put_failure;
		if (fira_report_local_aoa(local, msg,
					  &ranging_info->local_aoa_elevation))
			goto nla_put_failure;
		nla_nest_end(msg, aoa);
	}
	if (ranging_info->remote_aoa_azimuth_present) {
		if (nla_put_s16(msg, A(REMOTE_AOA_AZIMUTH_2PI),
				ranging_info->remote_aoa_azimuth_2pi))
			goto nla_put_failure;
		if (ranging_info->remote_aoa_fom_present) {
			if (nla_put_u8(msg, A(REMOTE_AOA_AZIMUTH_FOM),
				       ranging_info->remote_aoa_azimuth_fom))
				goto nla_put_failure;
		}
	}
	if (ranging_info->remote_aoa_elevation_present) {
		if (nla_put_s16(msg, A(REMOTE_AOA_ELEVATION_PI),
				ranging_info->remote_aoa_elevation_pi))
			goto nla_put_failure;
		if (ranging_info->remote_aoa_fom_present) {
			if (nla_put_u8(msg, A(REMOTE_AOA_ELEVATION_FOM),
				       ranging_info->remote_aoa_elevation_fom))
				goto nla_put_failure;
		}
	}

#undef A
	return 0;
nla_put_failure:
	return -EMSGSIZE;
}

void fira_report(struct fira_local *local, struct fira_session *session,
		 bool add_measurements)
{
	struct sk_buff *msg;
	struct nlattr *data, *measurements, *measurement;
	int block_duration_ms, i, r;

	msg = mcps802154_region_event_alloc_skb(local->llhw, &local->region,
						FIRA_CALL_SESSION_NOTIFICATION,
						session->event_portid,
						NLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (!msg)
		return;

	if (nla_put_u32(msg, FIRA_CALL_ATTR_SESSION_ID, session->id))
		goto nla_put_failure;

	data = nla_nest_start(msg, FIRA_CALL_ATTR_RANGING_DATA);
	if (!data)
		goto nla_put_failure;

	block_duration_ms = session->params.block_duration_dtu /
			    (local->llhw->dtu_freq_hz / 1000);
	if (nla_put_u32(msg, FIRA_RANGING_DATA_ATTR_BLOCK_INDEX,
			session->block_index) ||
	    nla_put_u32(msg, FIRA_RANGING_DATA_ATTR_RANGING_INTERVAL_MS,
			block_duration_ms))
		goto nla_put_failure;

	if (session->stop_request) {
		if (nla_put_u8(msg, FIRA_RANGING_DATA_ATTR_STOPPED,
			       FIRA_RANGING_DATA_ATTR_STOPPED_REQUEST))
			goto nla_put_failure;
	}

	if (add_measurements) {
		measurements = nla_nest_start(
			msg, FIRA_RANGING_DATA_ATTR_MEASUREMENTS);
		if (!measurements)
			goto nla_put_failure;

		for (i = 0; i < local->n_ranging_info; i++) {
			measurement = nla_nest_start(msg, 1);
			if (fira_report_measurement(local, msg,
						    &local->ranging_info[i]))
				goto nla_put_failure;
			nla_nest_end(msg, measurement);
		}

		nla_nest_end(msg, measurements);
	}

	nla_nest_end(msg, data);

	r = mcps802154_region_event(local->llhw, msg);
	if (r == -ECONNREFUSED)
		/* TODO stop. */
		;
	return;
nla_put_failure:
	kfree_skb(msg);
}

static struct mcps802154_region_ops fira_region_ops = {
	.owner = THIS_MODULE,
	.name = "fira",
	.open = fira_open,
	.close = fira_close,
	.call = fira_call,
	.get_access = fira_get_access,
};

int __init fira_region_init(void)
{
	int r;

	r = fira_crypto_test();
	WARN_RETURN(r);

	return mcps802154_region_register(&fira_region_ops);
}

void __exit fira_region_exit(void)
{
	mcps802154_region_unregister(&fira_region_ops);
}

module_init(fira_region_init);
module_exit(fira_region_exit);

MODULE_DESCRIPTION("FiRa Region for IEEE 802.15.4 MCPS");
MODULE_AUTHOR("Nicolas Schodet <nicolas.schodet@qorvo.com>");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
