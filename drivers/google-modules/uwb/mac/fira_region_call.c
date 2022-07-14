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

#include <asm/unaligned.h>

#include <linux/errno.h>
#include <linux/ieee802154.h>
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
	[FIRA_SESSION_PARAM_ATTR_DEVICE_TYPE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_DEVICE_TYPE_CONTROLLER,
	},
	[FIRA_SESSION_PARAM_ATTR_DEVICE_ROLE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_DEVICE_ROLE_INITIATOR,
	},
	[FIRA_SESSION_PARAM_ATTR_RANGING_ROUND_USAGE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_RANGING_ROUND_USAGE_DSTWR,
	},
	[FIRA_SESSION_PARAM_ATTR_MULTI_NODE_MODE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_MULTI_NODE_MODE_MANY_TO_MANY,
	},
	[FIRA_SESSION_PARAM_ATTR_SHORT_ADDR] = { .type = NLA_U16 },
	[FIRA_SESSION_PARAM_ATTR_DESTINATION_SHORT_ADDR] = { .type = NLA_U16 },
	[FIRA_SESSION_PARAM_ATTR_INITIATION_TIME_MS] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_SLOT_DURATION_RSTU] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_BLOCK_DURATION_MS] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_ROUND_DURATION_SLOTS] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_BLOCK_STRIDE_LENGTH] = {
		.type = NLA_U32, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BLOCK_STRIDE_LEN_MAX, },
	[FIRA_SESSION_PARAM_ATTR_MAX_NUMBER_OF_MEASUREMENTS] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_MAX_RR_RETRY] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_ROUND_HOPPING] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX,
	},
	[FIRA_SESSION_PARAM_ATTR_PRIORITY] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_PRIORITY_MAX,
	},
	[FIRA_SESSION_PARAM_ATTR_RESULT_REPORT_PHASE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX,
	},
	[FIRA_SESSION_PARAM_ATTR_MR_AT_INITIATOR] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_MEASUREMENT_REPORT_AT_INITIATOR,
	},
	[FIRA_SESSION_PARAM_ATTR_EMBEDDED_MODE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_EMBEDDED_MODE_NON_DEFERRED,
	},
	[FIRA_SESSION_PARAM_ATTR_IN_BAND_TERMINATION_ATTEMPT_COUNT] = {
		.type = NLA_U32, .validation_type = NLA_VALIDATE_RANGE,
		.min = FIRA_IN_BAND_TERMINATION_ATTEMPT_COUNT_MIN,
		.max = FIRA_IN_BAND_TERMINATION_ATTEMPT_COUNT_MAX,
	},
	[FIRA_SESSION_PARAM_ATTR_CHANNEL_NUMBER] = { .type = NLA_U8 },
	[FIRA_SESSION_PARAM_ATTR_PREAMBLE_CODE_INDEX] = { .type = NLA_U8 },
	[FIRA_SESSION_PARAM_ATTR_RFRAME_CONFIG] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_RFRAME_CONFIG_SP3,
	},
	[FIRA_SESSION_PARAM_ATTR_PRF_MODE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_PRF_MODE_HPRF,
	},
	[FIRA_SESSION_PARAM_ATTR_PREAMBLE_DURATION] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_PREAMBULE_DURATION_64,
	},
	[FIRA_SESSION_PARAM_ATTR_SFD_ID] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_SFD_ID_4,
	},
	[FIRA_SESSION_PARAM_ATTR_NUMBER_OF_STS_SEGMENTS] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_STS_SEGMENTS_2,
	},
	[FIRA_SESSION_PARAM_ATTR_PSDU_DATA_RATE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_PSDU_DATA_RATE_31M2,
	},
	[FIRA_SESSION_PARAM_ATTR_BPRF_PHR_DATA_RATE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_PHR_DATA_RATE_6M81,
	},
	[FIRA_SESSION_PARAM_ATTR_MAC_FCS_TYPE] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_MAC_FCS_TYPE_CRC_32,
	},
	[FIRA_SESSION_PARAM_ATTR_TX_ADAPTIVE_PAYLOAD_POWER] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX,
	},
	[FIRA_SESSION_PARAM_ATTR_MEASUREMENT_SEQUENCE] = { .type = NLA_NESTED_ARRAY },
	[FIRA_SESSION_PARAM_ATTR_STS_CONFIG] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_STS_CONFIG_DYNAMIC_INDIVIDUAL_KEY,
	},
	[FIRA_SESSION_PARAM_ATTR_SUB_SESSION_ID] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_VUPPER64] =
		NLA_POLICY_EXACT_LEN(FIRA_VUPPER64_SIZE),
	[FIRA_SESSION_PARAM_ATTR_SESSION_KEY] = {
		.type = NLA_BINARY, .len = FIRA_KEY_SIZE_MAX, },
	[FIRA_SESSION_PARAM_ATTR_SUB_SESSION_KEY] = {
		.type = NLA_BINARY, .len = FIRA_KEY_SIZE_MAX, },
	[FIRA_SESSION_PARAM_ATTR_KEY_ROTATION] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX, },
	[FIRA_SESSION_PARAM_ATTR_KEY_ROTATION_RATE] = { .type = NLA_U8 },
	[FIRA_SESSION_PARAM_ATTR_AOA_RESULT_REQ] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX,
	},
	[FIRA_SESSION_PARAM_ATTR_REPORT_TOF] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX,
	},
	[FIRA_SESSION_PARAM_ATTR_REPORT_AOA_AZIMUTH] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX,
	},
	[FIRA_SESSION_PARAM_ATTR_REPORT_AOA_ELEVATION] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX,
	},
	[FIRA_SESSION_PARAM_ATTR_REPORT_AOA_FOM] = {
		.type = NLA_U8, .validation_type = NLA_VALIDATE_MAX,
		.max = FIRA_BOOLEAN_MAX,
	},
	[FIRA_SESSION_PARAM_ATTR_DATA_VENDOR_OUI] = { .type = NLA_U32 },
	[FIRA_SESSION_PARAM_ATTR_DATA_PAYLOAD] = {
		.type = NLA_BINARY, .len = FIRA_DATA_PAYLOAD_SIZE_MAX, },
};

/**
 * get_session_state() - Get state of the session.
 * @local: FiRa context.
 * @session_id: Fira session id.
 *
 * Return: current session state.
 */
static enum fira_session_state get_session_state(struct fira_local *local,
						 u32 session_id)
{
	struct fira_session *session;
	bool active;
	enum fira_session_state state;

	/* Determine if session exists already. */
	session = fira_session_get(local, session_id, &active);
	if (!session) {
		state = FIRA_SESSION_STATE_DEINIT;
	} else {
		/* Is session active? */
		if (active) {
			state = FIRA_SESSION_STATE_ACTIVE;
		} else {
			/* Is session ready? */
			if (fira_session_is_ready(local, session))
				state = FIRA_SESSION_STATE_IDLE;
			else
				state = FIRA_SESSION_STATE_INIT;
		}
	}

	return state;
}

/**
 * fira_session_init() - Initialize Fira session.
 * @local: FiRa context.
 * @session_id: Fira session id.
 *
 * Return: 0 or error.
 */
static int fira_session_init(struct fira_local *local, u32 session_id)
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
	int n;
	bool active;
	struct fira_session *session;

	session = fira_session_get(local, session_id, &active);
	if (!session)
		return -ENOENT;

	trace_region_fira_session_params(session, &session->params);
	for (n = 0; n < session->params.meas_seq.active->n_steps; n++)
		trace_region_fira_meas_seq_step(
			session, &(session->params.meas_seq.active->steps[n]),
			n);

	if (!fira_session_is_ready(local, session))
		return -EINVAL;

	session->event_portid = info->snd_portid;

	if (!active) {
		u32 now_dtu;
		int initiation_time_dtu;
		int block_stride_len;
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

		if (session->params.initiation_time_ms) {
			initiation_time_dtu =
				session->params.initiation_time_ms *
				(local->llhw->dtu_freq_hz / 1000);
		} else if (session->params.device_type ==
				   FIRA_DEVICE_TYPE_CONTROLLER &&
			   local->llhw->anticip_dtu) {
			/* In order to be able to generate a frame for
			   sts_index = sts_index_init, add anticip_dtu two
			   times, for mcps802154_fproc_access_now and for
			   fira_get_access.
			   Also add 5 ms delay since now_dtu will change between
			   fira_session_start and fira_get_access. */
			initiation_time_dtu =
				2 * local->llhw->anticip_dtu +
				5 * (local->llhw->dtu_freq_hz / 1000);
		} else {
			initiation_time_dtu = 0;
		}

		if (session->params.device_type == FIRA_DEVICE_TYPE_CONTROLLER)
			block_stride_len = session->params.block_stride_len;
		else
			block_stride_len = 0;

		session->block_start_dtu = now_dtu + initiation_time_dtu;
		session->block_index = 0;
		session->sts_index = session->crypto.sts_index_init;
		session->hopping_sequence_generation =
			session->params.round_hopping;
		session->round_index = 0;
		session->next_round_index = 0;
		session->block_stride_len = block_stride_len;
		session->next_block_stride_len = block_stride_len;
		session->synchronised = false;
		session->last_access_timestamp_dtu = -1;
		session->last_access_duration_dtu = 0;
		session->last_block_index = -(block_stride_len + 1);
		fira_session_update_round_index(session);
		session->number_of_measurements = 0;

		session->params.meas_seq.current_step = 0;
		session->params.meas_seq.n_measurements_achieved = 0;

		list_move(&session->entry, &local->active_sessions);

		mcps802154_reschedule(local->llhw);
	}

	return 0;
}

/**
 * fira_session_stop() - Stop Fira session.
 * @local: FiRa context.
 * @session_id: Fira session id.
 *
 * Return: 0 or error.
 */
static int fira_session_stop(struct fira_local *local, u32 session_id)
{
	bool active;
	struct fira_session *session;

	session = fira_session_get(local, session_id, &active);
	if (!session)
		return -ENOENT;

	if (active) {
		session->stop_request = true;
		if (session->params.device_type == FIRA_DEVICE_TYPE_CONTROLEE) {
			if (local->current_session != session)
				fira_session_access_done(local, session, false);
			else
				mcps802154_reschedule(local->llhw);
		} else {
			if (local->current_session == NULL ||
			    (local->current_session != session &&
			     !session->current_controlees.size)) {
				fira_session_access_done(local, session, false);
			} else {
				if (session->controlee_management_flags) {
					/* Many changes on same round or while a controlee is stopping is not supported. */
					return -EBUSY;
				}
				fira_session_copy_controlees(
					&session->new_controlees,
					&session->current_controlees);
				fira_session_stop_controlees(
					session, &session->new_controlees);
				session->controlee_management_flags =
					FIRA_SESSION_CONTROLEE_MANAGEMENT_FLAG_UPDATE |
					FIRA_SESSION_CONTROLEE_MANAGEMENT_FLAG_STOP;
			}
		}
	}

	return 0;
}

/**
 * fira_session_deinit() - Deinitialize Fira session.
 * @local: FiRa context.
 * @session_id: Fira session id.
 *
 * Return: 0 or error.
 */
static int fira_session_deinit(struct fira_local *local, u32 session_id)
{
	bool active;
	struct fira_session *session;

	session = fira_session_get(local, session_id, &active);
	if (!session)
		return -ENOENT;
	if (active)
		return -EBUSY;

	fira_session_free(session);
	return 0;
}

static int is_allowed_param_active(enum fira_device_type dev_type, int param)
{
	/* These arrays contain attributes which can be changed in an active
	   controller or controlee session */
	static const int allowed_controller_param_active[] = {
		FIRA_SESSION_PARAM_ATTR_DATA_PAYLOAD,
		FIRA_SESSION_PARAM_ATTR_BLOCK_STRIDE_LENGTH,
		FIRA_SESSION_PARAM_ATTR_MEASUREMENT_SEQUENCE,
	};
	static const int allowed_controlee_param_active[] = {
		FIRA_SESSION_PARAM_ATTR_DATA_PAYLOAD,
		FIRA_SESSION_PARAM_ATTR_MEASUREMENT_SEQUENCE,
	};
	const int *allowed_param_active =
		dev_type == FIRA_DEVICE_TYPE_CONTROLLER ?
			allowed_controller_param_active :
			allowed_controlee_param_active;
	const int num_allowed =
		dev_type == FIRA_DEVICE_TYPE_CONTROLLER ?
			ARRAY_SIZE(allowed_controller_param_active) :
			ARRAY_SIZE(allowed_controlee_param_active);
	int i;

	for (i = 0; i < num_allowed; i++)
		if (allowed_param_active[i] == param)
			return 1;

	return 0;
}

/**
 * fira_session_params_set_measurement_sequence_step() - Retrieve a
 * measurement sequence step from a NL message and store it in the session
 * parameters.
 * @params: The parameters contained in the NL message.
 * @info: NL context info.
 * @step: The step where to store the information.
 *
 * Return: 0 or error.
 */
static int fira_session_params_set_measurement_sequence_step(
	const struct nlattr *params, const struct genl_info *info,
	struct fira_measurement_sequence_step *step)
{
#define STEP_ATTR(x) FIRA_SESSION_PARAM_MEAS_SEQ_STEP_ATTR_##x
#define ASR_ATTR(x) \
	FIRA_SESSION_PARAM_MEAS_SEQ_STEP_RX_ANT_SETS_RANGING_ATTR_##x
	static const struct nla_policy meas_seq_step_policy[STEP_ATTR(MAX) +
							    1] = {
		[STEP_ATTR(MEASUREMENT_TYPE)] = { .type = NLA_U8 },
		[STEP_ATTR(N_MEASUREMENTS)] = { .type = NLA_U8 },
		[STEP_ATTR(RX_ANT_SET_NONRANGING)] = { .type = NLA_U8 },
		[STEP_ATTR(RX_ANT_SETS_RANGING)] = { .type = NLA_NESTED },
		[STEP_ATTR(TX_ANT_SET_NONRANGING)] = { .type = NLA_U8 },
		[STEP_ATTR(TX_ANT_SET_RANGING)] = { .type = NLA_U8 },
	};

	static const struct nla_policy
		rx_ant_sets_ranging_policy[ASR_ATTR(MAX) + 1] = {
			[ASR_ATTR(0)] = { .type = NLA_U8 },
			[ASR_ATTR(1)] = { .type = NLA_U8 },
		};

	struct nlattr *step_attrs[STEP_ATTR(MAX) + 1];
	struct nlattr *rx_ant_sets_attrs[ASR_ATTR(MAX) + 1];
	int r = 0;
	u8 n_measurements = 0;

	enum fira_measurement_type type = __FIRA_MEASUREMENT_TYPE_AFTER_LAST;

	r = nla_parse_nested(step_attrs, STEP_ATTR(MAX), params,
			     meas_seq_step_policy, info->extack);
	/* LCOV_EXCL_START */
	/* defensive check, should not happen */
	if (r)
		return r;
	/* LCOV_EXCL_STOP */

	memset((void *)step, 0, sizeof(struct fira_measurement_sequence_step));
	if (!step_attrs[STEP_ATTR(MEASUREMENT_TYPE)] ||
	    !step_attrs[STEP_ATTR(N_MEASUREMENTS)])
		return -EINVAL;

	type = nla_get_u8(step_attrs[STEP_ATTR(MEASUREMENT_TYPE)]);
	if (type >= __FIRA_MEASUREMENT_TYPE_AFTER_LAST)
		return -EINVAL;
	step->type = type;

	n_measurements = nla_get_u8(step_attrs[STEP_ATTR(N_MEASUREMENTS)]);
	if (n_measurements == 0)
		return -EINVAL;
	step->n_measurements = n_measurements;

#define GET_ANTENNA(nl_attr, ant_set) \
	(ant_set) = !(nl_attr) ? -1 : nla_get_u8(nl_attr);

	GET_ANTENNA(step_attrs[STEP_ATTR(RX_ANT_SET_NONRANGING)],
		    step->rx_ant_set_nonranging);
	GET_ANTENNA(step_attrs[STEP_ATTR(TX_ANT_SET_NONRANGING)],
		    step->tx_ant_set_nonranging);
	GET_ANTENNA(step_attrs[STEP_ATTR(TX_ANT_SET_RANGING)],
		    step->tx_ant_set_ranging);

	r = nla_parse_nested(rx_ant_sets_attrs, ASR_ATTR(MAX),
			     step_attrs[STEP_ATTR(RX_ANT_SETS_RANGING)],
			     rx_ant_sets_ranging_policy, info->extack);
	/* LCOV_EXCL_START */
	/* defensive check, should not happen */
	if (r)
		return r;
	/* LCOV_EXCL_STOP */

	GET_ANTENNA(rx_ant_sets_attrs[ASR_ATTR(0)],
		    step->rx_ant_sets_ranging[0]);
	GET_ANTENNA(rx_ant_sets_attrs[ASR_ATTR(1)],
		    step->rx_ant_sets_ranging[1]);

#undef GET_ANTENNA
#undef STEP_ATTR
#undef ASR_ATTR

	if (step->rx_ant_sets_ranging[0] == -1)
		step->rx_ant_sets_ranging[0] = 0;
	if (step->rx_ant_sets_ranging[1] == -1)
		step->rx_ant_sets_ranging[1] = 0;
	if (step->rx_ant_set_nonranging == -1)
		step->rx_ant_set_nonranging = step->rx_ant_sets_ranging[0];
	if (step->tx_ant_set_ranging == -1)
		step->tx_ant_set_ranging = 0;
	if (step->tx_ant_set_nonranging == -1)
		step->tx_ant_set_nonranging = step->tx_ant_set_ranging;

	return 0;
}

/**
 * fira_session_params_set_measurement_sequence() - Retrieve the measurement
 * schedule from a NL message and store it in the session parameters.
 * @params: The parameters contained in the NL message.
 * @info: NL context info.
 * @meas_seq: The measurement sequence where to store the information.
 *
 * Return: 0 or error.
 */
static int fira_session_params_set_measurement_sequence(
	const struct nlattr *params, const struct genl_info *info,
	struct fira_measurement_sequence *meas_seq)
{
	struct nlattr *request = NULL;
	int r, rem = 0;
	size_t n_steps = 0;

	/* LCOV_EXCL_START */
	/* defensive check, not easy to trigger using unit test */
	if (!params)
		return -EINVAL;
	if (!meas_seq)
		return -EINVAL;
	/* LCOV_EXCL_STOP */

	nla_for_each_nested (request, params, rem) {
		if (n_steps >= FIRA_MEASUREMENT_SEQUENCE_STEP_MAX)
			return -EINVAL;

		r = fira_session_params_set_measurement_sequence_step(
			request, info, meas_seq->steps + n_steps);
		if (r)
			return r;

		++n_steps;
	}

	if (0 == n_steps)
		return -EINVAL;
	meas_seq->n_steps = n_steps;

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
	struct fira_measurement_sequence *meas_seq;
	int r;

	session = fira_session_get(local, session_id, &active);
	if (!session)
		return -ENOENT;
	if (!params)
		return -EINVAL;

	r = nla_parse_nested(attrs, FIRA_SESSION_PARAM_ATTR_MAX, params,
			     fira_session_param_nla_policy, info->extack);
	if (r)
		return r;

	p = &session->params;
	if (p->meas_seq.active == &(p->_meas_seq_1))
		meas_seq = &(p->_meas_seq_2);
	else
		meas_seq = &(p->_meas_seq_1);

	if (active) {
		int i;
		for (i = FIRA_SESSION_PARAM_ATTR_UNSPEC + 1;
		     i <= FIRA_SESSION_PARAM_ATTR_MAX; i++) {
			if (attrs[i] &&
			    !is_allowed_param_active(p->device_type, i))
				return -EBUSY;
		}
	}
#define P(attr, member, type, conv)                                     \
	do {                                                            \
		int x;                                                  \
		if (attrs[FIRA_SESSION_PARAM_ATTR_##attr]) {            \
			x = nla_get_##type(                             \
				attrs[FIRA_SESSION_PARAM_ATTR_##attr]); \
			p->member = conv;                               \
		}                                                       \
	} while (0)
#define PMEMCPY(attr, member)                                             \
	do {                                                              \
		if (attrs[FIRA_SESSION_PARAM_ATTR_##attr]) {              \
			struct nlattr *attr =                             \
				attrs[FIRA_SESSION_PARAM_ATTR_##attr];    \
			memcpy(p->member, nla_data(attr), nla_len(attr)); \
		}                                                         \
	} while (0)
#define PMEMNCPY(attr, member, size)                                   \
	do {                                                           \
		if (attrs[FIRA_SESSION_PARAM_ATTR_##attr]) {           \
			struct nlattr *attr =                          \
				attrs[FIRA_SESSION_PARAM_ATTR_##attr]; \
			int len = nla_len(attr);                       \
			memcpy(p->member, nla_data(attr), len);        \
			p->size = len;                                 \
		}                                                      \
	} while (0)
	/* Main session parameters. */
	P(DEVICE_TYPE, device_type, u8, x);
	P(RANGING_ROUND_USAGE, ranging_round_usage, u8, x);
	P(MULTI_NODE_MODE, multi_node_mode, u8, x);
	P(SHORT_ADDR, short_addr, u16, x);
	P(DESTINATION_SHORT_ADDR, controller_short_addr, u16, x);
	/* Timings parameters. */
	P(INITIATION_TIME_MS, initiation_time_ms, u32, x);
	P(SLOT_DURATION_RSTU, slot_duration_dtu, u32,
	  x * local->llhw->rstu_dtu);
	P(BLOCK_DURATION_MS, block_duration_dtu, u32,
	  x * (local->llhw->dtu_freq_hz / 1000));
	P(ROUND_DURATION_SLOTS, round_duration_slots, u32, x);
	/* Behaviour parameters. */
	P(BLOCK_STRIDE_LENGTH, block_stride_len, u32, x);
	P(MAX_NUMBER_OF_MEASUREMENTS, max_number_of_measurements, u32, x);
	P(MAX_RR_RETRY, max_rr_retry, u32, x);
	P(ROUND_HOPPING, round_hopping, u8, !!x);
	P(PRIORITY, priority, u8, x);
	P(RESULT_REPORT_PHASE, result_report_phase, u8, !!x);
	/* Radio parameters. */
	P(CHANNEL_NUMBER, channel_number, u8, x);
	P(PREAMBLE_CODE_INDEX, preamble_code_index, u8, x);
	P(RFRAME_CONFIG, rframe_config, u8, x);
	P(PREAMBLE_DURATION, preamble_duration, u8, x);
	P(SFD_ID, sfd_id, u8, x);
	P(PSDU_DATA_RATE, psdu_data_rate, u8, x);
	P(MAC_FCS_TYPE, mac_fcs_type, u8, x);
	/* Measurement Sequence */
	if (attrs[FIRA_SESSION_PARAM_ATTR_MEASUREMENT_SEQUENCE]) {
		if (fira_session_params_set_measurement_sequence(
			    attrs[FIRA_SESSION_PARAM_ATTR_MEASUREMENT_SEQUENCE],
			    info, meas_seq)) {
			return -EINVAL;
		} else if (!active) {
			/* immediate update */
			p->meas_seq.active = meas_seq;
		} else {
			/* deferred update when session is running */
			p->meas_seq.update_flag = true;
		}
	}
	/* STS and crypto parameters. */
	PMEMCPY(VUPPER64, vupper64);
	/* Report parameters. */
	P(AOA_RESULT_REQ, aoa_result_req, u8, !!x);
	P(REPORT_TOF, report_tof, u8, !!x);
	P(REPORT_AOA_AZIMUTH, report_aoa_azimuth, u8, !!x);
	P(REPORT_AOA_ELEVATION, report_aoa_elevation, u8, !!x);
	P(REPORT_AOA_FOM, report_aoa_fom, u8, !!x);
	/* Custom data */
	P(DATA_VENDOR_OUI, data_vendor_oui, u32, x);

	PMEMNCPY(DATA_PAYLOAD, data_payload, data_payload_len);
	/* Increment sequence number if a new data is received. */
	if (attrs[FIRA_SESSION_PARAM_ATTR_DATA_PAYLOAD]) {
		p->data_payload_seq++;
	}
	/* TODO: set all fira session parameters. */
#undef P
#undef PMEMCPY
#undef PMEMNCPY

	return 0;
}

/**
 * fira_session_get_state() - Get state of the session.
 * @local: FiRa context.
 * @session_id: Fira session id.
 *
 * Return: 0 or error.
 */
static int fira_session_get_state(struct fira_local *local, u32 session_id)
{
	struct sk_buff *msg;
	enum fira_session_state state;

	state = get_session_state(local, session_id);

	msg = mcps802154_region_call_alloc_reply_skb(
		local->llhw, &local->region, FIRA_CALL_SESSION_GET_STATE,
		NLMSG_DEFAULT_SIZE);
	if (!msg)
		return -ENOMEM;

	if (nla_put_u32(msg, FIRA_CALL_ATTR_SESSION_ID, session_id))
		goto nla_put_failure;

	if (nla_put_u8(msg, FIRA_CALL_ATTR_SESSION_STATE, state))
		goto nla_put_failure;

	return mcps802154_region_call_reply(local->llhw, msg);

nla_put_failure:
	kfree_skb(msg);
	return -ENOBUFS;
}

/**
 * fira_session_params_get_measurement_sequence_step() - Retrieve a
 * measurement sequence step in a NL message.
 * @step: The measurement sequence step to add to the message.
 * @msg_buf: NL message buffer.
 * @idx: Index of the step.
 *
 * Return: 0 or error.
 */
static int fira_session_params_get_measurement_sequence_step(
	const struct fira_measurement_sequence_step *step,
	struct sk_buff *msg_buf, size_t idx)
{
#define P(attr, member, type, conv)                      \
	do {                                             \
		type x = member;                         \
		if (nla_put_##type(msg_buf, attr, conv)) \
			return -ENOBUFS;                 \
	} while (0)
#define STEP_ATTR(x) FIRA_SESSION_PARAM_MEAS_SEQ_STEP_ATTR_##x
#define ASR_ATTR(x) \
	FIRA_SESSION_PARAM_MEAS_SEQ_STEP_RX_ANT_SETS_RANGING_ATTR_##x

	struct nlattr *step_params = NULL;
	struct nlattr *rx_ant_sets_ranging_params = NULL;

	step_params = nla_nest_start(msg_buf, idx);

	if (!step_params)
		return -ENOBUFS;
	P(STEP_ATTR(MEASUREMENT_TYPE), step->type, u8, x);
	P(STEP_ATTR(N_MEASUREMENTS), step->n_measurements, u8, x);
	P(STEP_ATTR(RX_ANT_SET_NONRANGING), step->rx_ant_set_nonranging, u8, x);

	rx_ant_sets_ranging_params =
		nla_nest_start(msg_buf, STEP_ATTR(RX_ANT_SETS_RANGING));
	if (!rx_ant_sets_ranging_params)
		return -ENOBUFS;

	P(ASR_ATTR(0), step->rx_ant_sets_ranging[0], u8, x);
	P(ASR_ATTR(1), step->rx_ant_sets_ranging[1], u8, x);

	nla_nest_end(msg_buf, rx_ant_sets_ranging_params);

	P(STEP_ATTR(TX_ANT_SET_NONRANGING), step->tx_ant_set_nonranging, u8, x);
	P(STEP_ATTR(TX_ANT_SET_RANGING), step->tx_ant_set_ranging, u8, x);

	nla_nest_end(msg_buf, step_params);

#undef STEP_ATTR
#undef ASR_ATTR
#undef P
	return 0;
}

/**
 * fira_session_params_get_measurement_sequence() - Retrieve a
 * measurement sequence in a NL message.
 * @meas_seq: The measurement sequence to add to the message.
 * @msg_buf: NL message buffer.
 *
 * Return: 0 or error.
 */
static int fira_session_params_get_measurement_sequence(
	const struct fira_measurement_sequence *meas_seq,
	struct sk_buff *msg_buf)
{
	struct nlattr *meas_seq_params = NULL;
	size_t i;

	/* LCOV_EXCL_START */
	/* defensive check, should not happen */
	if (!meas_seq || !msg_buf)
		return -EINVAL;
	/* LCOV_EXCL_STOP */

	meas_seq_params = nla_nest_start(
		msg_buf, FIRA_SESSION_PARAM_ATTR_MEASUREMENT_SEQUENCE);
	if (!meas_seq_params)
		return -ENOBUFS;

	for (i = 0; i < meas_seq->n_steps; ++i) {
		int r = 0;
		r = fira_session_params_get_measurement_sequence_step(
			meas_seq->steps + i, msg_buf, i);
		if (r)
			return r;
	}

	nla_nest_end(msg_buf, meas_seq_params);

	return 0;
}

/**
 * fira_session_get_parameters() - Get Fira session parameters.
 * @local: FiRa context.
 * @session_id: Fira session id.
 *
 * Return: 0 or error.
 */
static int fira_session_get_parameters(struct fira_local *local, u32 session_id)
{
	bool active;
	const struct fira_session *session;
	const struct fira_session_params *p;
	struct sk_buff *msg;
	struct nlattr *params;

	session = fira_session_get(local, session_id, &active);
	if (!session)
		return -ENOENT;

	p = &session->params;

	msg = mcps802154_region_call_alloc_reply_skb(
		local->llhw, &local->region, FIRA_CALL_SESSION_GET_PARAMS,
		NLMSG_DEFAULT_SIZE);
	if (!msg)
		return -ENOMEM;

	if (nla_put_u32(msg, FIRA_CALL_ATTR_SESSION_ID, session->id))
		goto nla_put_failure;

	params = nla_nest_start(msg, FIRA_CALL_ATTR_SESSION_PARAMS);
	if (!params)
		goto nla_put_failure;

#define P(attr, member, type, conv)                                            \
	do {                                                                   \
		type x = p->member;                                            \
		if (nla_put_##type(msg, FIRA_SESSION_PARAM_ATTR_##attr, conv)) \
			goto nla_put_failure;                                  \
	} while (0)
#define PMEMCPY(attr, member)                                    \
	do {                                                     \
		if (nla_put(msg, FIRA_SESSION_PARAM_ATTR_##attr, \
			    sizeof(p->member), p->member))       \
			goto nla_put_failure;                    \
	} while (0)
	/* Main session parameters. */
	P(DEVICE_TYPE, device_type, u8, x);
	P(RANGING_ROUND_USAGE, ranging_round_usage, u8, x);
	P(MULTI_NODE_MODE, multi_node_mode, u8, x);
	if (p->short_addr != IEEE802154_ADDR_SHORT_BROADCAST)
		P(SHORT_ADDR, short_addr, u16, x);
	if (p->controller_short_addr != IEEE802154_ADDR_SHORT_BROADCAST)
		P(DESTINATION_SHORT_ADDR, controller_short_addr, u16, x);
	/* Timings parameters. */
	P(INITIATION_TIME_MS, initiation_time_ms, u32, x);
	P(SLOT_DURATION_RSTU, slot_duration_dtu, u32,
	  x / local->llhw->rstu_dtu);
	P(BLOCK_DURATION_MS, block_duration_dtu, u32,
	  x / (local->llhw->dtu_freq_hz / 1000));
	P(ROUND_DURATION_SLOTS, round_duration_slots, u32, x);
	/* Behaviour parameters. */
	P(BLOCK_STRIDE_LENGTH, block_stride_len, u32, x);
	P(MAX_NUMBER_OF_MEASUREMENTS, max_number_of_measurements, u32, x);
	P(MAX_RR_RETRY, max_rr_retry, u32, x);
	P(ROUND_HOPPING, round_hopping, u8, !!x);
	P(PRIORITY, priority, u8, x);
	P(RESULT_REPORT_PHASE, result_report_phase, u8, !!x);
	/* Radio parameters. */
	if (p->channel_number)
		P(CHANNEL_NUMBER, channel_number, u8, x);
	if (p->preamble_code_index)
		P(PREAMBLE_CODE_INDEX, preamble_code_index, u8, x);
	P(RFRAME_CONFIG, rframe_config, u8, x);
	P(PREAMBLE_DURATION, preamble_duration, u8, x);
	P(SFD_ID, sfd_id, u8, x);
	P(PSDU_DATA_RATE, psdu_data_rate, u8, x);
	P(MAC_FCS_TYPE, mac_fcs_type, u8, x);
	/* Measurement Sequence */
	if (fira_session_params_get_measurement_sequence(
		    session->params.meas_seq.active, msg))
		goto nla_put_failure;

	/* STS and crypto parameters. */
	PMEMCPY(VUPPER64, vupper64);
	/* Report parameters. */
	P(AOA_RESULT_REQ, aoa_result_req, u8, !!x);
	P(REPORT_TOF, report_tof, u8, !!x);
	P(REPORT_AOA_AZIMUTH, report_aoa_azimuth, u8, !!x);
	P(REPORT_AOA_ELEVATION, report_aoa_elevation, u8, !!x);
	P(REPORT_AOA_FOM, report_aoa_fom, u8, !!x);
	/* Custom data */
	if (p->data_vendor_oui)
		P(DATA_VENDOR_OUI, data_vendor_oui, u32, x);
#undef P
#undef PMEMCPY

	nla_nest_end(msg, params);

	return mcps802154_region_call_reply(local->llhw, msg);
nla_put_failure:
	kfree_skb(msg);
	return -ENOBUFS;
}

/**
 * fira_manage_controlees() - Manage controlees.
 * @local: FiRa context.
 * @call_id: Fira call id.
 * @session_id: Fira session id.
 * @params: Nested attribute containing controlee parameters.
 * @info: Request information.
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
		controlees[n_controlees].state = FIRA_CONTROLEE_STATE_RUNNING;

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

	if (session->controlee_management_flags) {
		/* Many changes on same round or while a controlee is stopping is not supported. */
		return -EBUSY;
	} else if (active) {
		/* If unicast refuse to add more than one controlee. */
		if (call_id == FIRA_CALL_NEW_CONTROLEE &&
		    session->params.multi_node_mode ==
			    FIRA_MULTI_NODE_MODE_UNICAST &&
		    (session->current_controlees.size > 0 || n_controlees > 1))
			return -EINVAL;
		if (call_id == FIRA_CALL_SET_CONTROLEE)
			return -EBUSY;
		fira_session_copy_controlees(&session->new_controlees,
					     &session->current_controlees);
		/* Use second array to not disturbe active session. */
		controlees_array = &session->new_controlees;
	} else {
		/* No risk to disturbe this session. */
		controlees_array = &session->current_controlees;
	}

	switch (call_id) {
	case FIRA_CALL_SET_CONTROLEE:
		r = fira_session_set_controlees(local, session,
						controlees_array, controlees,
						n_controlees);
		break;
	case FIRA_CALL_DEL_CONTROLEE:
		if (active)
			r = fira_session_async_del_controlees(session,
							      controlees_array,
							      controlees,
							      n_controlees);
		else
			r = fira_session_del_controlees(
				controlees_array, controlees, n_controlees);
		break;
	/* FIRA_CALL_NEW_CONTROLEE. */
	default:
		r = fira_session_new_controlees(session, active,
						controlees_array, controlees,
						n_controlees);
	}
	if (r)
		return r;

	if (active)
		session->controlee_management_flags |=
			FIRA_SESSION_CONTROLEE_MANAGEMENT_FLAG_UPDATE;

	return 0;
}

/**
 * fira_session_get_controlees() - Get list of controlees.
 * @local: FiRa context.
 * @session_id: Fira session id.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int fira_session_get_controlees(struct fira_local *local, u32 session_id,
				       const struct genl_info *info)
{
	bool active;
	const struct fira_session *session;
	struct sk_buff *msg;
	struct nlattr *controlees, *controlee;
	int i;

	session = fira_session_get(local, session_id, &active);
	if (!session)
		return -ENOENT;

	msg = mcps802154_region_call_alloc_reply_skb(local->llhw,
						     &local->region,
						     FIRA_CALL_GET_CONTROLEES,
						     NLMSG_DEFAULT_SIZE);
	if (!msg)
		return -ENOMEM;

	if (nla_put_u32(msg, FIRA_CALL_ATTR_SESSION_ID, session->id))
		goto nla_put_failure;

	controlees = nla_nest_start(msg, FIRA_CALL_ATTR_CONTROLEES);
	if (!controlees)
		goto nla_put_failure;

	for (i = 0; i < session->current_controlees.size; i++) {
		const struct fira_controlee *c =
			&session->current_controlees.data[i];
		controlee = nla_nest_start(msg, 1);
		if (!controlee)
			goto nla_put_failure;
		if (nla_put_le16(msg, FIRA_CALL_CONTROLEE_ATTR_SHORT_ADDR,
				 c->short_addr))
			goto nla_put_failure;
		if (c->sub_session &&
		    nla_put_u32(msg, FIRA_CALL_CONTROLEE_ATTR_SUB_SESSION_ID,
				c->sub_session_id))
			goto nla_put_failure;
		nla_nest_end(msg, controlee);
	}

	nla_nest_end(msg, controlees);

	return mcps802154_region_call_reply(local->llhw, msg);
nla_put_failure:
	kfree_skb(msg);
	return -ENOBUFS;
}

int fira_get_capabilities(struct fira_local *local,
			  const struct genl_info *info)
{
	struct sk_buff *msg;
	struct nlattr *capabilities;

	if (!info)
		return 0;

	msg = mcps802154_region_call_alloc_reply_skb(local->llhw,
						     &local->region,
						     FIRA_CALL_GET_CAPABILITIES,
						     NLMSG_DEFAULT_SIZE);
	if (!msg)
		return -ENOMEM;

	capabilities = nla_nest_start(msg, FIRA_CALL_ATTR_CAPABILITIES);
	if (!capabilities) {
		goto nla_put_failure;
	}

#define P(name, type, value)                                                   \
	do {                                                                   \
		if (nla_put_##type(msg, FIRA_CAPABILITY_ATTR_##name, value)) { \
			goto nla_put_failure;                                  \
		}                                                              \
	} while (0)
#define F(name)                                                       \
	do {                                                          \
		if (nla_put_flag(msg, FIRA_CAPABILITY_ATTR_##name)) { \
			goto nla_put_failure;                         \
		}                                                     \
	} while (0)

	/* Main session capabilities. */
	P(FIRA_PHY_VERSION_RANGE, u32, 0x01010101);
	P(FIRA_MAC_VERSION_RANGE, u32, 0x01010101);
	P(DEVICE_CLASS, u8, 1);
	F(DEVICE_TYPE_CONTROLEE_RESPONDER);
	F(DEVICE_TYPE_CONTROLLER_INITIATOR);
	F(MULTI_NODE_MODE_UNICAST);
	F(MULTI_NODE_MODE_ONE_TO_MANY);
	F(RANGING_ROUND_USAGE_DS_TWR);
	P(NUMBER_OF_CONTROLEES_MAX, u32, FIRA_CONTROLEES_MAX);
	/* Behaviour. */
	F(ROUND_HOPPING);
	F(BLOCK_STRIDING);
	/* Radio. */
	P(CHANNEL_NUMBER, u16,
	  local->llhw->hw->phy->supported
		  .channels[local->llhw->hw->phy->current_page]);
	F(RFRAME_CONFIG_SP1);
	F(RFRAME_CONFIG_SP3);
	F(PRF_MODE_BPRF);
	F(PREAMBLE_DURATION_64);
	F(SFD_ID_2);
	F(NUMBER_OF_STS_SEGMENTS_0);
	F(NUMBER_OF_STS_SEGMENTS_1);
	F(PSDU_DATA_RATE_6M81);
	F(BPRF_PHR_DATA_RATE_850K);
	F(TX_ADAPTIVE_PAYLOAD_POWER);
	/* Antenna. */
	P(RX_ANTENNA_PAIRS, u32, local->llhw->rx_antenna_pairs);
	P(TX_ANTENNAS, u32, local->llhw->tx_antennas);
	/* STS and crypto capabilities. */
	F(STS_CONFIG_STATIC);
	/* Report. */
	F(AOA_AZIMUTH);
	F(AOA_ELEVATION);
	F(AOA_FOM);
#undef F
#undef P

	nla_nest_end(msg, capabilities);
	return mcps802154_region_call_reply(local->llhw, msg);
nla_put_failure:
	kfree_skb(msg);
	return -ENOBUFS;
}

int fira_session_get_count(struct fira_local *local)
{
	struct fira_session *session;
	struct sk_buff *msg;
	u32 count = 0;

	list_for_each_entry (session, &local->inactive_sessions, entry) {
		count++;
	}

	list_for_each_entry (session, &local->active_sessions, entry) {
		count++;
	}

	msg = mcps802154_region_call_alloc_reply_skb(
		local->llhw, &local->region, FIRA_CALL_SESSION_GET_COUNT,
		NLMSG_DEFAULT_SIZE);
	if (!msg)
		return -ENOMEM;

	if (nla_put_u32(msg, FIRA_CALL_ATTR_SESSION_COUNT, count))
		goto nla_put_failure;

	return mcps802154_region_call_reply(local->llhw, msg);

nla_put_failure:
	kfree_skb(msg);
	return -ENOBUFS;
}

int fira_session_control(struct fira_local *local, u32 call_id,
			 const struct nlattr *params,
			 const struct genl_info *info)
{
	u32 session_id;
	struct nlattr *attrs[FIRA_CALL_ATTR_MAX + 1];
	int r;

	/* Special case of get count that doesn't need params. */
	if (call_id == FIRA_CALL_SESSION_GET_COUNT)
		return fira_session_get_count(local);

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
		return fira_session_init(local, session_id);
	case FIRA_CALL_SESSION_START:
		return fira_session_start(local, session_id, info);
	case FIRA_CALL_SESSION_STOP:
		return fira_session_stop(local, session_id);
	case FIRA_CALL_SESSION_DEINIT:
		return fira_session_deinit(local, session_id);
	case FIRA_CALL_SESSION_SET_PARAMS:
		return fira_session_set_parameters(
			local, session_id, attrs[FIRA_CALL_ATTR_SESSION_PARAMS],
			info);
	case FIRA_CALL_SET_CONTROLEE:
	case FIRA_CALL_NEW_CONTROLEE:
	case FIRA_CALL_DEL_CONTROLEE:
		return fira_manage_controlees(local, call_id, session_id,
					      attrs[FIRA_CALL_ATTR_CONTROLEES],
					      info);
	case FIRA_CALL_GET_CONTROLEES:
		return fira_session_get_controlees(local, session_id, info);
	case FIRA_CALL_SESSION_GET_PARAMS:
		return fira_session_get_parameters(local, session_id);
	case FIRA_CALL_SESSION_GET_STATE:
		return fira_session_get_state(local, session_id);
	default:
		return -EINVAL;
	}
}
