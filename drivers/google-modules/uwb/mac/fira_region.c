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

	local = kzalloc(sizeof(*local), GFP_KERNEL);
	if (!local)
		return NULL;
	local->llhw = llhw;
	local->region.ops = &fira_region_ops;
	local->current_session = NULL;
	INIT_LIST_HEAD(&local->inactive_sessions);
	INIT_LIST_HEAD(&local->active_sessions);
	local->block_duration_rx_margin_ppm = UWB_BLOCK_DURATION_MARGIN_PPM;
	return &local->region;
}

static void fira_close(struct mcps802154_region *region)
{
	struct fira_local *local = region_to_local(region);

	kfree_sensitive(local);
}

static void fira_notify_stop(struct mcps802154_region *region)
{
	struct fira_local *local = region_to_local(region);
	struct fira_session *session, *s;

	list_for_each_entry_safe (session, s, &local->active_sessions, entry) {
		session->stop_request = true;
		fira_session_access_done(local, session, false);
	}
}

static int fira_call(struct mcps802154_region *region, u32 call_id,
		     const struct nlattr *attrs, const struct genl_info *info)
{
	struct fira_local *local = region_to_local(region);

	switch (call_id) {
	case FIRA_CALL_GET_CAPABILITIES:
		return fira_get_capabilities(local, info);
	default:
		return fira_session_control(local, call_id, attrs, info);
	}
}

static int fira_get_demand(struct mcps802154_region *region,
			   u32 next_timestamp_dtu,
			   struct mcps802154_region_demand *demand)
{
	struct fira_local *local = region_to_local(region);
	struct fira_session *session;

	session = fira_session_next(
		local, next_timestamp_dtu + local->llhw->anticip_dtu, 0);

	if (session) {
		fira_session_get_demand(local, session, demand);
		demand->max_duration_dtu = session->last_access_duration_dtu;
		local->current_session = session;
		return 1;
	}
	return 0;
}

static int fira_report_local_aoa(struct sk_buff *msg,
				 const struct fira_local_aoa_info *info)
{
#define A(x) FIRA_RANGING_DATA_MEASUREMENTS_AOA_ATTR_##x
	if (nla_put_u8(msg, A(RX_ANTENNA_SET), info->rx_ant_set))
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
	const struct fira_session *session = local->current_session;
	struct nlattr *aoa;
#define A(x) FIRA_RANGING_DATA_MEASUREMENTS_ATTR_##x

	if (nla_put_u16(msg, A(SHORT_ADDR), ranging_info->short_addr) ||
	    nla_put_u8(msg, A(STATUS), ranging_info->status))
		goto nla_put_failure;

	if (ranging_info->status) {
		if (nla_put_u8(msg, A(SLOT_INDEX), ranging_info->slot_index))
			goto nla_put_failure;
		return 0;
	}

	if (ranging_info->tof_present) {
		static const s64 speed_of_light_mm_per_s = 299702547000ull;
		s32 distance_mm = div64_s64(
			ranging_info->tof_rctu * speed_of_light_mm_per_s,
			(s64)local->llhw->dtu_freq_hz * local->llhw->dtu_rctu);
		if (nla_put_s32(msg, A(DISTANCE_MM), distance_mm))
			goto nla_put_failure;
	}

	if (ranging_info->local_aoa.present) {
		aoa = nla_nest_start(msg, A(LOCAL_AOA));
		if (!aoa)
			goto nla_put_failure;
		if (fira_report_local_aoa(msg, &ranging_info->local_aoa))
			goto nla_put_failure;
		nla_nest_end(msg, aoa);
	}

	if (ranging_info->local_aoa_azimuth.present) {
		aoa = nla_nest_start(msg, A(LOCAL_AOA_AZIMUTH));
		if (!aoa)
			goto nla_put_failure;
		if (fira_report_local_aoa(msg,
					  &ranging_info->local_aoa_azimuth))
			goto nla_put_failure;
		nla_nest_end(msg, aoa);
	}
	if (ranging_info->local_aoa_elevation.present) {
		aoa = nla_nest_start(msg, A(LOCAL_AOA_ELEVATION));
		if (!aoa)
			goto nla_put_failure;
		if (fira_report_local_aoa(msg,
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
	if (ranging_info->data_payload_len > 0) {
		if (nla_put(msg, A(DATA_PAYLOAD_RECV),
			    ranging_info->data_payload_len,
			    ranging_info->data_payload))
			goto nla_put_failure;
	}
	if (session->data_payload_seq_sent > 0) {
		if (nla_put_u32(msg, A(DATA_PAYLOAD_SEQ_SENT),
				session->data_payload_seq_sent))
			goto nla_put_failure;
	}

#undef A
	return 0;
nla_put_failure:
	return -EMSGSIZE;
}

static int fira_report_measurement_stopped_controlee(struct fira_local *local,
						     struct sk_buff *msg,
						     __le16 short_addr)
{
#define A(x) FIRA_RANGING_DATA_MEASUREMENTS_ATTR_##x

	if (nla_put_u16(msg, A(SHORT_ADDR), short_addr) ||
	    nla_put_u8(msg, A(STOPPED), 1))
		goto nla_put_failure;

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
	int ranging_interval_ms, i, r;
	bool stop_completed;

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

	ranging_interval_ms = session->params.block_duration_dtu *
			      (session->block_stride_len + 1) /
			      (local->llhw->dtu_freq_hz / 1000);
	if (nla_put_u32(msg, FIRA_RANGING_DATA_ATTR_BLOCK_INDEX,
			session->block_index) ||
	    nla_put_u32(msg, FIRA_RANGING_DATA_ATTR_RANGING_INTERVAL_MS,
			ranging_interval_ms))
		goto nla_put_failure;

	stop_completed = (session->max_number_of_measurements_reached ||
			  session->stop_request) &&
			 !(session->controlee_management_flags &
			   FIRA_SESSION_CONTROLEE_MANAGEMENT_FLAG_STOP);
	if (stop_completed || session->stop_inband ||
	    session->stop_no_response) {
		enum fira_ranging_data_attrs_stopped_values stopped_value =
			stop_completed ?
				FIRA_RANGING_DATA_ATTR_STOPPED_REQUEST :
				session->stop_inband ?
				FIRA_RANGING_DATA_ATTR_STOPPED_IN_BAND :
				FIRA_RANGING_DATA_ATTR_STOPPED_NO_RESPONSE;

		if (nla_put_u8(msg, FIRA_RANGING_DATA_ATTR_STOPPED,
			       stopped_value))
			goto nla_put_failure;
	}

	if (add_measurements && (local->n_ranging_info +
				 local->n_stopped_controlees_short_addr) != 0) {
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

		for (i = 0; i < local->n_stopped_controlees_short_addr; i++) {
			measurement = nla_nest_start(msg, 1);
			if (fira_report_measurement_stopped_controlee(
				    local, msg,
				    local->stopped_controlees_short_addr[i]))
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
	.notify_stop = fira_notify_stop,
	.call = fira_call,
	.get_access = fira_get_access,
	.get_demand = fira_get_demand,
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
