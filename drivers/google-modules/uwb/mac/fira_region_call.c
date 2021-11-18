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
 * 802.15.4 mac common part sublayer, fira ranging call procedures.
 *
 */

#include <linux/errno.h>
#include <linux/string.h>

#include <net/fira_region_nl.h>
#include <net/mcps802154_frame.h>

#include "fira_session.h"
#include "fira_access.h"
#include "fira_region_call.h"
#include "fira_trace.h"

static const struct nla_policy fira_call_nla_policy[FIRA_CALL_ATTR_MAX + 1] = {
	[FIRA_CALL_ATTR_SESSION_ID] = { .type = NLA_U32 },
	[FIRA_CALL_ATTR_SESSION_PARAMS] = { .type = NLA_NESTED },
	[FIRA_CALL_ATTR_CONTROLEES] = { .type = NLA_NESTED_ARRAY },
};

static const struct nla_policy fira_session_param_nla_policy[FIRA_SESSION_PARAM_ATTR_MAX +
							     1] = {
	/* clang-format off */
	[FIRA_SESSION_PARAM_ATTR_DEVICE_TYPE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_DEVICE_TYPE_CONTROLLER
	},
	[FIRA_SESSION_PARAM_ATTR_DEVICE_ROLE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_DEVICE_ROLE_INITIATOR
	},
	[FIRA_SESSION_PARAM_ATTR_RANGING_ROUND_USAGE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_RANGING_ROUND_USAGE_DSTWR
	},
	[FIRA_SESSION_PARAM_ATTR_MULTI_NODE_MODE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_MULTI_NODE_MODE_MANY_TO_MANY
	},
	[FIRA_SESSION_PARAM_ATTR_SHORT_ADDR] = { .type = NLA_U16 },
	[FIRA_SESSION_PARAM_ATTR_DESTINATION_SHORT_ADDR] = { .type = NLA_U16 },
	[FIRA_SESSION_PARAM_ATTR_INITIATION_TIME_MS] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_SLOT_DURATION_RSTU] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_BLOCK_DURATION_MS] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_ROUND_DURATION_SLOTS] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_BLOCK_STRIDING_VALUE] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_MAX_RR_RETRY] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_ROUND_HOPPING] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX
	},
	[FIRA_SESSION_PARAM_ATTR_BLOCK_STRIDING] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX
	},
	[FIRA_SESSION_PARAM_ATTR_PRIORITY] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_PRIORITY_MAX
	},
	[FIRA_SESSION_PARAM_ATTR_RESULT_REPORT_PHASE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX
	},
	[FIRA_SESSION_PARAM_ATTR_MR_AT_INITIATOR] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_MEASUREMENT_REPORT_AT_INITIATOR
	},
	[FIRA_SESSION_PARAM_ATTR_EMBEDDED_MODE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_EMBEDDED_MODE_NON_DEFERRED
	},
	[FIRA_SESSION_PARAM_ATTR_IN_BAND_TERMINATION_ATTEMPT_COUNT] = {
		.type = NLA_U32, .validation_type = NLA_VALIDATE_RANGE,
		.min = FIRA_IN_BAND_TERMINATION_ATTEMPT_COUNT_MIN,
		.max = FIRA_IN_BAND_TERMINATION_ATTEMPT_COUNT_MAX
	},
	[FIRA_SESSION_PARAM_ATTR_CHANNEL_NUMBER] = { .type = NLA_U8 },
	[FIRA_SESSION_PARAM_ATTR_PREAMBLE_CODE_INDEX] = { .type = NLA_U8 },
	[FIRA_SESSION_PARAM_ATTR_RFRAME_CONFIG] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_RFRAME_CONFIG_SP3
	},
	[FIRA_SESSION_PARAM_ATTR_PRF_MODE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_PRF_MODE_HPRF
	},
	[FIRA_SESSION_PARAM_ATTR_PREAMBLE_DURATION] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_PREAMBULE_DURATION_64
	},
	[FIRA_SESSION_PARAM_ATTR_SFD_ID] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_SFD_ID_4
	},
	[FIRA_SESSION_PARAM_ATTR_NUMBER_OF_STS_SEGMENTS] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_STS_SEGMENTS_2
	},
	[FIRA_SESSION_PARAM_ATTR_PSDU_DATA_RATE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_PSDU_DATA_RATE_31M2
	},
	[FIRA_SESSION_PARAM_ATTR_BPRF_PHR_DATA_RATE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_PHR_DATA_RATE_6M81
	},
	[FIRA_SESSION_PARAM_ATTR_MAC_FCS_TYPE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_MAC_FCS_TYPE_CRC_32
	},
	[FIRA_SESSION_PARAM_ATTR_TX_ADAPTIVE_PAYLOAD_POWER] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX
	},
	[FIRA_SESSION_PARAM_ATTR_RX_ANTENNA_SELECTION] = { .type = NLA_U8 },
	[FIRA_SESSION_PARAM_ATTR_RX_ANTENNA_PAIR_AZIMUTH] = { .type = NLA_U8 },
	[FIRA_SESSION_PARAM_ATTR_RX_ANTENNA_PAIR_ELEVATION] = { .type = NLA_U8 },
	[FIRA_SESSION_PARAM_ATTR_TX_ANTENNA_SELECTION] = { .type = NLA_U8,
		.validation_type = NLA_VALIDATE_MIN, .min = 1 },
	[FIRA_SESSION_PARAM_ATTR_RX_ANTENNA_SWITCH] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_RX_ANTENNA_SWITCH_TWO_RANGING
	},
	[FIRA_SESSION_PARAM_ATTR_STS_CONFIG] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_STS_CONFIG_DYNAMIC_INDIVIDUAL_KEY
	},
	[FIRA_SESSION_PARAM_ATTR_SUB_SESSION_ID] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_VUPPER64] =
		NLA_POLICY_EXACT_LEN(FIRA_VUPPER64_SIZE),
	[FIRA_SESSION_PARAM_ATTR_SESSION_KEY] = {
		.type = NLA_BINARY, .len = FIRA_KEY_SIZE_MAX },
	[FIRA_SESSION_PARAM_ATTR_SUB_SESSION_KEY] = {
		.type = NLA_BINARY, .len = FIRA_KEY_SIZE_MAX },
	[FIRA_SESSION_PARAM_ATTR_KEY_ROTATION] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX },
	[FIRA_SESSION_PARAM_ATTR_KEY_ROTATION_RATE] = { .type = NLA_U8 },
	[FIRA_SESSION_PARAM_ATTR_AOA_RESULT_REQ] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX
	},
	[FIRA_SESSION_PARAM_ATTR_REPORT_TOF] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX
	},
	[FIRA_SESSION_PARAM_ATTR_REPORT_AOA_AZIMUTH] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX
	},
	[FIRA_SESSION_PARAM_ATTR_REPORT_AOA_ELEVATION] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX
	},
	[FIRA_SESSION_PARAM_ATTR_REPORT_AOA_FOM] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX
	},
	/* clang-format on */
};

/**
 * fira_session_init() - Initialize Fira session.
 * @local: FiRa context.
 * @session_id: Fira session id.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int fira_session_init(struct fira_local *local, u32 session_id,
			     const struct genl_info *info)
{
	bool active;
	struct fira_session *session;

	session = fira_session_get(local, session_id, &active);
	if (session)
		return -EBUSY;

	session = fira_session_new(local, session_id);
	if (!session)
		return -ENOMEM;

	return 0;
}

/**
 * fira_session_start() - Start Fira session.
 * @local: FiRa context.
 * @session_id: Fira session id.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int fira_session_start(struct fira_local *local, u32 session_id,
			      const struct genl_info *info)
{
	bool active;
	struct fira_session *session;

	session = fira_session_get(local, session_id, &active);
	if (!session)
		return -ENOENT;

	trace_region_fira_session_params(session, &session->params);

	if (!fira_session_is_ready(local, session))
		return -EINVAL;

	session->event_portid = info->snd_portid;

	if (!active) {
		u32 now_dtu;
		int initiation_time_dtu;
		int r;

		r = fira_crypto_derive_per_session(local, session);
		if (r)
			return r;
		r = fira_crypto_derive_per_rotation(local, session, 0);
		if (r)
			return r;

		r = mcps802154_get_current_timestamp_dtu(local->llhw, &now_dtu);
		if (r)
			return r;

		initiation_time_dtu = session->params.initiation_time_ms *
				      (local->llhw->dtu_freq_hz / 1000);
		session->block_start_dtu = now_dtu + initiation_time_dtu;
		session->block_index = 0;
		session->sts_index = session->crypto.sts_index_init;
		session->hopping_sequence_generation =
			session->params.round_hopping;
		session->round_index = 0;
		session->next_round_index = 0;
		session->synchronised = false;
		session->last_access_timestamp_dtu = -1;
		session->last_access_duration_dtu = 0;
		fira_session_round_hopping(session);
		session->tx_ant = -1;
		session->rx_ant_pair[0] = -1;
		session->rx_ant_pair[1] = -1;
		list_move(&session->entry, &local->active_sessions);

		mcps802154_reschedule(local->llhw);
	}

	return 0;
}

/**
 * fira_session_stop() - Stop Fira session.
 * @local: FiRa context.
 * @session_id: Fira session id.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int fira_session_stop(struct fira_local *local, u32 session_id,
			     const struct genl_info *info)
{
	bool active;
	struct fira_session *session;

	session = fira_session_get(local, session_id, &active);
	if (!session)
		return -ENOENT;

	if (active) {
		session->stop_request = true;
		if (local->current_session != session)
			fira_session_access_done(local, session);
		else
			mcps802154_reschedule(local->llhw);
	}

	return 0;
}

/**
 * fira_session_deinit() - Deinitialize Fira session.
 * @local: FiRa context.
 * @session_id: Fira session id.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int fira_session_deinit(struct fira_local *local, u32 session_id,
			       const struct genl_info *info)
{
	bool active;
	struct fira_session *session;

	session = fira_session_get(local, session_id, &active);
	if (!session)
		return -ENOENT;
	if (active)
		return -EBUSY;

	fira_session_free(local, session);
	return 0;
}

/**
 * fira_session_set_parameters() - Set Fira session parameters.
 * @local: FiRa context.
 * @session_id: Fira session id.
 * @params: Nested attribute containing session parameters.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int fira_session_set_parameters(struct fira_local *local, u32 session_id,
				       const struct nlattr *params,
				       const struct genl_info *info)
{
	struct nlattr *attrs[FIRA_SESSION_PARAM_ATTR_MAX + 1];
	bool active;
	struct fira_session *session;
	struct fira_session_params *p;
	int r;

	session = fira_session_get(local, session_id, &active);
	if (!session)
		return -ENOENT;
	if (active)
		return -EBUSY;

	if (!params)
		return -EINVAL;

	r = nla_parse_nested(attrs, FIRA_SESSION_PARAM_ATTR_MAX, params,
			     fira_session_param_nla_policy, info->extack);
	if (r)
		return r;

#define P(attr, member, type, conv)                                     \
	do {                                                            \
		int x;                                                  \
		if (attrs[FIRA_SESSION_PARAM_ATTR_##attr]) {            \
			x = nla_get_##type(                             \
				attrs[FIRA_SESSION_PARAM_ATTR_##attr]); \
			session->params.member = conv;                  \
		}                                                       \
	} while (0)
#define PMEMCPY(attr, member)                                          \
	do {                                                           \
		if (attrs[FIRA_SESSION_PARAM_ATTR_##attr]) {           \
			struct nlattr *attr =                          \
				attrs[FIRA_SESSION_PARAM_ATTR_##attr]; \
			memcpy(session->params.member, nla_data(attr), \
			       nla_len(attr));                         \
		}                                                      \
	} while (0)
	/* Main session parameters. */
	P(DEVICE_TYPE, device_type, u8, x);
	P(RANGING_ROUND_USAGE, ranging_round_usage, u8, x);
	P(MULTI_NODE_MODE, multi_node_mode, u8, x);
	P(DESTINATION_SHORT_ADDR, controller_short_addr, u16, x);
	/* Timings parameters. */
	P(INITIATION_TIME_MS, initiation_time_ms, u32, x);
	P(SLOT_DURATION_RSTU, slot_duration_dtu, u32,
	  x * local->llhw->rstu_dtu);
	P(BLOCK_DURATION_MS, block_duration_dtu, u32,
	  x * (local->llhw->dtu_freq_hz / 1000));
	P(ROUND_DURATION_SLOTS, round_duration_slots, u32, x);
	/* Behaviour parameters. */
	P(ROUND_HOPPING, round_hopping, u8, !!x);
	P(PRIORITY, priority, u8, x);
	/* Radio parameters. */
	P(CHANNEL_NUMBER, channel_number, u8, x);
	P(PREAMBLE_CODE_INDEX, preamble_code_index, u8, x);
	P(RFRAME_CONFIG, rframe_config, u8, x);
	P(PREAMBLE_DURATION, preamble_duration, u8, x);
	P(SFD_ID, sfd_id, u8, x);
	P(PSDU_DATA_RATE, psdu_data_rate, u8, x);
	P(MAC_FCS_TYPE, mac_fcs_type, u8, x);
	/* Antenna parameters. */
	P(RX_ANTENNA_SELECTION, rx_antenna_selection, u8, x);
	P(RX_ANTENNA_PAIR_AZIMUTH, rx_antenna_pair_azimuth, u8, x);
	P(RX_ANTENNA_PAIR_ELEVATION, rx_antenna_pair_elevation, u8, x);
	P(TX_ANTENNA_SELECTION, tx_antenna_selection, u8, x);
	P(RX_ANTENNA_SWITCH, rx_antenna_switch, u8, x);
	/* STS and crypto parameters. */
	PMEMCPY(VUPPER64, vupper64);
	/* Report parameters. */
	P(AOA_RESULT_REQ, aoa_result_req, u8, !!x);
	P(REPORT_TOF, report_tof, u8, !!x);
	P(REPORT_AOA_AZIMUTH, report_aoa_azimuth, u8, !!x);
	P(REPORT_AOA_ELEVATION, report_aoa_elevation, u8, !!x);
	P(REPORT_AOA_FOM, report_aoa_fom, u8, !!x);
	/* TODO: set all fira session parameters. */
#undef P
#undef PMEMCPY

	p = &session->params;
	switch (p->rx_antenna_switch) {
	case FIRA_RX_ANTENNA_SWITCH_BETWEEN_ROUND:
		if (p->rx_antenna_pair_azimuth != FIRA_RX_ANTENNA_PAIR_INVALID)
			p->rx_antenna_selection |=
				1 << p->rx_antenna_pair_azimuth;
		if (p->rx_antenna_pair_elevation !=
		    FIRA_RX_ANTENNA_PAIR_INVALID)
			p->rx_antenna_selection |=
				1 << p->rx_antenna_pair_elevation;
		break;
	case FIRA_RX_ANTENNA_SWITCH_DURING_ROUND:
	case FIRA_RX_ANTENNA_SWITCH_TWO_RANGING:
		if (p->rx_antenna_pair_azimuth ==
			    FIRA_RX_ANTENNA_PAIR_INVALID ||
		    p->rx_antenna_pair_elevation ==
			    FIRA_RX_ANTENNA_PAIR_INVALID)
			return -EINVAL;
		break;
	}

	return 0;
}

/**
 * fira_manage_controlees() - Manage controlees.
 * @local: FiRa context.
 * @call_id: Fira call id.
 * @session_id: Fira session id.
 * @params: Nested attribute containing controlee parameters.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int fira_manage_controlees(struct fira_local *local, u32 call_id,
				  u32 session_id, const struct nlattr *params,
				  const struct genl_info *info)
{
	static const struct nla_policy new_controlee_nla_policy[FIRA_CALL_CONTROLEE_ATTR_MAX +
								1] = {
		[FIRA_CALL_CONTROLEE_ATTR_SHORT_ADDR] = { .type = NLA_U16 },
		[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_ID] = { .type = NLA_U32 },
		[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_KEY] = { .type = NLA_BINARY,
							       .len = FIRA_KEY_SIZE_MAX },
	};
	struct nlattr *request;
	struct fira_controlee controlees[FIRA_CONTROLEES_MAX];
	struct nlattr *attrs[FIRA_CALL_CONTROLEE_ATTR_MAX + 1];
	int r, rem, i, n_controlees = 0;
	struct fira_session *session;
	struct fira_controlees_array *controlees_array;
	bool active;

	if (!params)
		return -EINVAL;

	nla_for_each_nested (request, params, rem) {
		if (n_controlees >= FIRA_CONTROLEES_MAX)
			return -EINVAL;

		r = nla_parse_nested(attrs, FIRA_CALL_CONTROLEE_ATTR_MAX,
				     request, new_controlee_nla_policy,
				     info->extack);
		if (r)
			return r;

		if (!attrs[FIRA_CALL_CONTROLEE_ATTR_SHORT_ADDR] ||
		    (!attrs[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_ID] ^
		     !attrs[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_KEY]))
			return -EINVAL;

		controlees[n_controlees].short_addr = nla_get_le16(
			attrs[FIRA_CALL_CONTROLEE_ATTR_SHORT_ADDR]);
		if (attrs[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_ID]) {
			if (call_id == FIRA_CALL_DEL_CONTROLEE)
				return -EINVAL;
			controlees[n_controlees].sub_session = true;
			controlees[n_controlees].sub_session_id = nla_get_u32(
				attrs[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_ID]);
			memcpy(controlees[n_controlees].sub_session_key,
			       nla_data(
				       attrs[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_KEY]),
			       nla_len(attrs[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_KEY]));
			controlees[n_controlees].sub_session_key_len = nla_len(
				attrs[FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_KEY]);
		} else
			controlees[n_controlees].sub_session = false;

		for (i = 0; i < n_controlees; i++) {
			if (controlees[n_controlees].short_addr ==
			    controlees[i].short_addr)
				return -EINVAL;
		}

		n_controlees++;
	}
	if (!n_controlees)
		return -EINVAL;

	session = fira_session_get(local, session_id, &active);
	if (!session)
		return -ENOENT;

	if (session->params.update_controlees) {
		/* Many changes on same round is not supported. */
		return -EBUSY;
	} else if (active) {
		fira_session_copy_controlees(
			&session->params.new_controlees,
			&session->params.current_controlees);
		/* Use second array to not disturbe active session. */
		controlees_array = &session->params.new_controlees;
	} else {
		/* No risk to disturbe this session. */
		controlees_array = &session->params.current_controlees;
	}

	if (call_id == FIRA_CALL_DEL_CONTROLEE) {
		r = fira_session_del_controlees(local, session,
						controlees_array, controlees,
						n_controlees);
	} else {
		r = fira_session_new_controlees(local, session,
						controlees_array, controlees,
						n_controlees);
	}
	if (r)
		return r;

	if (active)
		session->params.update_controlees = true;

	return 0;
}

int fira_get_capabilities(struct fira_local *local,
			  const struct genl_info *info)
{
	/* TODO: inform netlink about capabilities. */
	return 0;
}

int fira_session_control(struct fira_local *local, u32 call_id,
			 const struct nlattr *params,
			 const struct genl_info *info)
{
	u32 session_id;
	struct nlattr *attrs[FIRA_CALL_ATTR_MAX + 1];
	int r;

	if (!params)
		return -EINVAL;
	r = nla_parse_nested(attrs, FIRA_CALL_ATTR_MAX, params,
			     fira_call_nla_policy, info->extack);

	if (r)
		return r;

	if (!attrs[FIRA_CALL_ATTR_SESSION_ID])
		return -EINVAL;

	session_id = nla_get_u32(attrs[FIRA_CALL_ATTR_SESSION_ID]);

	switch (call_id) {
	case FIRA_CALL_SESSION_INIT:
		return fira_session_init(local, session_id, info);
	case FIRA_CALL_SESSION_START:
		return fira_session_start(local, session_id, info);
	case FIRA_CALL_SESSION_STOP:
		return fira_session_stop(local, session_id, info);
	case FIRA_CALL_SESSION_DEINIT:
		return fira_session_deinit(local, session_id, info);
	case FIRA_CALL_SESSION_SET_PARAMS:
		return fira_session_set_parameters(
			local, session_id, attrs[FIRA_CALL_ATTR_SESSION_PARAMS],
			info);
	/* FIRA_CALL_NEW_CONTROLEE and FIRA_CALL_DEL_CONTROLEE. */
	default:
		return fira_manage_controlees(local, call_id, session_id,
					      attrs[FIRA_CALL_ATTR_CONTROLEES],
					      info);
	}
}
