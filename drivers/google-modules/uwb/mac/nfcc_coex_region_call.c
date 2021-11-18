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
 * 802.15.4 mac common part sublayer, nfcc_coex call procedures.
 */

#include <linux/errno.h>
#include <linux/string.h>

#include <net/nfcc_coex_region_nl.h>
#include <net/mcps802154_frame.h>

#include "nfcc_coex_session.h"
#include "nfcc_coex_region_call.h"

static const struct nla_policy nfcc_coex_call_nla_policy[NFCC_COEX_CALL_ATTR_MAX +
							 1] = {
	[NFCC_COEX_CALL_ATTR_CCC_SESSION_PARAMS] = { .type = NLA_NESTED },
};

static const struct nla_policy nfcc_coex_session_param_nla_policy
	[NFCC_COEX_CCC_SESSION_PARAM_ATTR_MAX + 1] = {
		[NFCC_COEX_CCC_SESSION_PARAM_ATTR_TIME0_NS] = { .type = NLA_U64 },
		[NFCC_COEX_CCC_SESSION_PARAM_ATTR_CHANNEL_NUMBER] = { .type = NLA_U8 },
	};

/**
 * nfcc_coex_session_set_parameters() - Set NFCC coexistence session parameters.
 * @local: NFCC coexistence context.
 * @params: Nested attribute containing session parameters.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int nfcc_coex_session_set_parameters(struct nfcc_coex_local *local,
					    const struct nlattr *params,
					    const struct genl_info *info)
{
	struct nlattr *attrs[NFCC_COEX_CCC_SESSION_PARAM_ATTR_MAX + 1];
	struct nfcc_coex_session *session = &local->session;
	struct nfcc_coex_session_params *p = &session->params;
	int r;

	r = nla_parse_nested(attrs, NFCC_COEX_CCC_SESSION_PARAM_ATTR_MAX,
			     params, nfcc_coex_session_param_nla_policy,
			     info->extack);
	if (r)
		return r;

#define P(attr, member, type, conv)                                              \
	do {                                                                     \
		type x;                                                          \
		if (attrs[NFCC_COEX_CCC_SESSION_PARAM_ATTR_##attr]) {            \
			x = nla_get_##type(                                      \
				attrs[NFCC_COEX_CCC_SESSION_PARAM_ATTR_##attr]); \
			p->member = conv;                                        \
		}                                                                \
	} while (0)

	P(TIME0_NS, time0_ns, u64, x);
	P(CHANNEL_NUMBER, channel_number, u8, x);
#undef P

	return 0;
}

/**
 * nfcc_coex_session_start() - Start NFCC coex session.
 * @local: NFCC coexistence context.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int nfcc_coex_session_start(struct nfcc_coex_local *local,
				   const struct genl_info *info)
{
	struct nfcc_coex_session *session = &local->session;
	int initiation_time_dtu;
	u32 now_dtu;
	int r;

	WARN_ON(local->state != NFCC_COEX_STATE_UNUSED);

	r = mcps802154_get_current_timestamp_dtu(local->llhw, &now_dtu);
	if (r)
		return r;

	initiation_time_dtu =
		(session->params.time0_ns * local->llhw->dtu_freq_hz) /
		NS_PER_SECOND;
	session->region_demand.timestamp_dtu = now_dtu + initiation_time_dtu;
	session->region_demand.duration_dtu = 0;
	session->event_portid = info->snd_portid;
	session->first_access = true;
	local->state = NFCC_COEX_STATE_STARTED;

	mcps802154_reschedule(local->llhw);

	return 0;
}

/**
 * nfcc_coex_session_start_all() - Start all for a NFCC coex session.
 * @local: NFCC coexistence context.
 * @params: Call parameters.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int nfcc_coex_session_start_all(struct nfcc_coex_local *local,
				       const struct nlattr *params,
				       const struct genl_info *info)
{
	struct nlattr *attrs[NFCC_COEX_CALL_ATTR_MAX + 1];
	int r;

	if (!params)
		return -EINVAL;

	r = nla_parse_nested(attrs, NFCC_COEX_CALL_ATTR_MAX, params,
			     nfcc_coex_call_nla_policy, info->extack);
	if (r)
		return r;

	if (local->state != NFCC_COEX_STATE_UNUSED)
		return -EBUSY;

	nfcc_coex_session_init(local);

	r = nfcc_coex_session_set_parameters(
		local, attrs[NFCC_COEX_CALL_ATTR_CCC_SESSION_PARAMS], info);
	if (r)
		return r;

	r = nfcc_coex_session_start(local, info);
	if (r)
		return r;

	return 0;
}

/**
 * nfcc_coex_session_stop() - Stop NFCC coex session.
 * @local: NFCC coexistence context.
 *
 * Return: 0 or error.
 */
static int nfcc_coex_session_stop(struct nfcc_coex_local *local)
{
	switch (local->state) {
	case NFCC_COEX_STATE_STARTED:
		local->state = NFCC_COEX_STATE_UNUSED;
		break;
	case NFCC_COEX_STATE_ACCESSING:
		/* The transition from ACCESSING to UNUSED will be done
		 * in nfcc_coex_get_access function. */
		local->state = NFCC_COEX_STATE_STOPPING;
		break;
	default:
		break;
	}

	return 0;
}

int nfcc_coex_session_control(struct nfcc_coex_local *local, u32 call_id,
			      const struct nlattr *params,
			      const struct genl_info *info)
{
	switch (call_id) {
	case NFCC_COEX_CALL_CCC_SESSION_START:
		return nfcc_coex_session_start_all(local, params, info);
	default:
	case NFCC_COEX_CALL_CCC_SESSION_STOP:
		return nfcc_coex_session_stop(local);
	}
}
