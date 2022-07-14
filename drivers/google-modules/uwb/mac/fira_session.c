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

#include "fira_session.h"
#include "fira_crypto.h"
#include "fira_round_hopping_sequence.h"
#include "fira_access.h"
#include "fira_frame.h"
#include "fira_trace.h"

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/ieee802154.h>
#include <linux/string.h>
#include <linux/limits.h>

#define FIRA_DRIFT_TOLERANCE_PPM 30

struct fira_session *fira_session_new(struct fira_local *local, u32 session_id)
{
	struct fira_session *session;

	session = kzalloc(sizeof(*session), GFP_KERNEL);
	if (!session)
		return NULL;

	session->id = session_id;
	session->params.ranging_round_usage = FIRA_RANGING_ROUND_USAGE_DSTWR;
	session->params.short_addr = IEEE802154_ADDR_SHORT_BROADCAST;
	session->params.controller_short_addr = IEEE802154_ADDR_SHORT_BROADCAST;
	session->params.slot_duration_dtu =
		FIRA_SLOT_DURATION_RSTU_DEFAULT * local->llhw->rstu_dtu;
	session->params.block_duration_dtu = FIRA_BLOCK_DURATION_MS_DEFAULT *
					     (local->llhw->dtu_freq_hz / 1000);
	session->params.round_duration_slots =
		FIRA_ROUND_DURATION_SLOTS_DEFAULT;
	session->params.max_rr_retry = FIRA_MAX_RR_RETRY_DEFAULT;
	session->params.round_hopping = false;
	session->params.priority = FIRA_PRIORITY_DEFAULT;
	session->params.result_report_phase = true;
	session->params.rframe_config = FIRA_RFRAME_CONFIG_SP3;
	session->params.preamble_duration = FIRA_PREAMBULE_DURATION_64;
	session->params.sfd_id = FIRA_SFD_ID_2;

	session->params._meas_seq_1.n_steps = 1;
	session->params._meas_seq_1.steps[0].type = FIRA_MEASUREMENT_TYPE_RANGE;
	session->params._meas_seq_1.steps[0].n_measurements = 1;
	session->params._meas_seq_1.steps[0].rx_ant_set_nonranging = 0;
	session->params._meas_seq_1.steps[0].rx_ant_sets_ranging[0] = 0;
	session->params._meas_seq_1.steps[0].rx_ant_sets_ranging[1] = 0;
	session->params._meas_seq_1.steps[0].tx_ant_set_nonranging = 0;
	session->params._meas_seq_1.steps[0].tx_ant_set_ranging = 0;
	session->params._meas_seq_2.n_steps = 0;
	session->params.meas_seq.active = &(session->params._meas_seq_1);
	session->params.meas_seq.current_step = 0;
	session->params.meas_seq.n_measurements_achieved = 0;

	/* Report parameters. */
	session->params.aoa_result_req = true;
	session->params.report_tof = true;

	if (fira_round_hopping_sequence_init(session))
		goto failed;

	list_add(&session->entry, &local->inactive_sessions);

	return session;
failed:
	kfree(session);
	return NULL;
}

void fira_session_free(struct fira_session *session)
{
	fira_round_hopping_sequence_destroy(session);
	list_del(&session->entry);
	fira_aead_destroy(&session->crypto.aead);
	/* The session structure contains the Crypto context. This needs to be
	 * cleared. */
	kfree_sensitive(session);
}

struct fira_session *fira_session_get(struct fira_local *local, u32 session_id,
				      bool *active)
{
	struct fira_session *session;

	list_for_each_entry (session, &local->inactive_sessions, entry) {
		if (session->id == session_id) {
			*active = false;
			return session;
		}
	}

	list_for_each_entry (session, &local->active_sessions, entry) {
		if (session->id == session_id) {
			*active = true;
			return session;
		}
	}

	return NULL;
}

void fira_session_copy_controlees(struct fira_controlees_array *to,
				  const struct fira_controlees_array *from)
{
	/* Copy only valid entries. */
	memcpy(to->data, from->data, from->size * sizeof(from->data[0]));
	to->size = from->size;
}

int fira_session_set_controlees(struct fira_local *local,
				struct fira_session *session,
				struct fira_controlees_array *controlees_array,
				const struct fira_controlee *controlees,
				size_t n_controlees)
{
	int i;

	if (!fira_frame_check_n_controlees(session, n_controlees, false))
		return -EINVAL;

	for (i = 0; i < n_controlees; i++)
		controlees_array->data[i] = controlees[i];

	controlees_array->size = n_controlees;

	return 0;
}

int fira_session_new_controlees(struct fira_session *session, bool active,
				struct fira_controlees_array *controlees_array,
				const struct fira_controlee *controlees,
				size_t n_controlees)
{
	int i, j;

	if (!fira_frame_check_n_controlees(
		    session, controlees_array->size + n_controlees, active))
		return -EINVAL;

	for (i = 0; i < n_controlees; i++) {
		for (j = 0; j < controlees_array->size; j++) {
			if (controlees[i].short_addr ==
			    controlees_array->data[j].short_addr)
				return -EINVAL;
		}
	}

	for (i = 0; i < n_controlees; i++)
		controlees_array->data[controlees_array->size++] =
			controlees[i];

	return 0;
}

static void
fira_session_update_stopping_controlees(struct fira_session *session)
{
	size_t ii, io;
	struct fira_controlees_array *controlees_array =
		&session->current_controlees;

	for (ii = 0, io = 0; ii < controlees_array->size; ii++) {
		struct fira_controlee *c = &controlees_array->data[ii];

		if (c->state != FIRA_CONTROLEE_STATE_PENDING_DEL) {
			if (io != ii)
				controlees_array->data[io] = *c;
			controlees_array->data[io].state =
				FIRA_CONTROLEE_STATE_RUNNING;
			io++;
		}
	}
	controlees_array->size = io;
}

int fira_session_del_controlees(struct fira_controlees_array *controlees_array,
				const struct fira_controlee *controlees,
				size_t n_controlees)
{
	size_t ii, io, j;

	for (ii = 0, io = 0; ii < controlees_array->size; ii++) {
		bool remove = false;
		struct fira_controlee *c = &controlees_array->data[ii];

		for (j = 0; j < n_controlees && !remove; j++) {
			if (c->short_addr == controlees[j].short_addr)
				remove = true;
		}

		if (!remove) {
			if (io != ii)
				controlees_array->data[io] = *c;
			io++;
		}
	}
	controlees_array->size = io;

	return 0;
}

int fira_session_async_del_controlees(
	struct fira_session *session,
	struct fira_controlees_array *controlees_array,
	const struct fira_controlee *controlees, size_t n_controlees)
{
	size_t i, j;

	for (i = 0; i < controlees_array->size; i++) {
		struct fira_controlee *c = &controlees_array->data[i];
		enum fira_controlee_state state = FIRA_CONTROLEE_STATE_RUNNING;

		for (j = 0; j < n_controlees; j++) {
			if (c->short_addr == controlees[j].short_addr) {
				state = FIRA_CONTROLEE_STATE_PENDING_DEL;
				session->controlee_management_flags |=
					FIRA_SESSION_CONTROLEE_MANAGEMENT_FLAG_STOP;
				break;
			}
		}
		c->state = state;
	}

	return 0;
}

void fira_session_stop_controlees(struct fira_session *session,
				  struct fira_controlees_array *controlees_array)
{
	size_t i;

	for (i = 0; i < controlees_array->size; i++) {
		controlees_array->data[i].state =
			FIRA_CONTROLEE_STATE_PENDING_STOP;
	}
}

bool fira_session_is_ready(struct fira_local *local,
			   struct fira_session *session)
{
	int round_duration_dtu;
	struct fira_session_params *params = &session->params;

	if (params->multi_node_mode == FIRA_MULTI_NODE_MODE_UNICAST) {
		if (session->current_controlees.size > 1)
			return false;
	} else {
		/* on success, session will become active, so assume it is */
		if (!fira_frame_check_n_controlees(
			    session, session->current_controlees.size, true))
			return false;
	}

	round_duration_dtu =
		params->slot_duration_dtu * params->round_duration_slots;
	return params->slot_duration_dtu != 0 &&
	       params->block_duration_dtu != 0 &&
	       params->round_duration_slots != 0 &&
	       round_duration_dtu < params->block_duration_dtu;
}

inline static void fira_update_meas_seq(struct fira_session *session)
{
	struct fira_session_params *p = &session->params;
	if (p->meas_seq.update_flag) {
		if (p->meas_seq.active == &p->_meas_seq_1)
			p->meas_seq.active = &p->_meas_seq_2;
		else
			p->meas_seq.active = &p->_meas_seq_1;
		p->meas_seq.current_step = 0;
		p->meas_seq.n_measurements_achieved = 0;
		p->meas_seq.update_flag = false;
	} else {
		if (p->meas_seq.n_measurements_achieved >=
		    p->meas_seq.active->steps[p->meas_seq.current_step]
			    .n_measurements) {
			p->meas_seq.n_measurements_achieved = 0;
			p->meas_seq.current_step++;
			p->meas_seq.current_step %= p->meas_seq.active->n_steps;
		}
	}
	trace_region_fira_meas_seq_step(
		session, &(p->meas_seq.active->steps[p->meas_seq.current_step]),
		p->meas_seq.current_step);
}

void fira_session_prepare(struct fira_session *session)
{
	fira_update_meas_seq(session);
	if (session->params.device_type == FIRA_DEVICE_TYPE_CONTROLLER) {
		session->next_block_stride_len =
			session->params.block_stride_len;
		if (session->params.round_hopping) {
			u32 next_block_index = session->block_index +
					       session->next_block_stride_len +
					       1;
			session->next_round_index =
				fira_round_hopping_sequence_get(
					session, next_block_index);
		}
	}
}

void fira_session_update_round_index(struct fira_session *session)
{
	if (session->hopping_sequence_generation) {
		session->round_index = fira_round_hopping_sequence_get(
			session, session->block_index);
	} else {
		session->round_index = session->next_round_index;
		session->hopping_sequence_generation =
			session->params.round_hopping;
	}
}

static void fira_session_update(struct fira_local *local,
				struct fira_session *session,
				u32 next_timestamp_dtu)
{
	u32 access_dtu;
	s32 diff_dtu;
	int block_duration_margin_dtu = 0;

	if (session->params.device_type == FIRA_DEVICE_TYPE_CONTROLEE)
		block_duration_margin_dtu =
			fira_session_get_block_duration_margin(local, session);

	/* Do we have the time to participate in the current block? */
	access_dtu = session->block_start_dtu +
		     fira_session_get_round_slot(session) *
			     session->params.slot_duration_dtu -
		     block_duration_margin_dtu;
	diff_dtu = access_dtu - next_timestamp_dtu;

	if (diff_dtu < 0) {
		int block_duration_dtu = session->params.block_duration_dtu;
		int block_duration_slots =
			block_duration_dtu / session->params.slot_duration_dtu;
		int block_stride_len = session->block_stride_len;
		int block_stride_duration_dtu =
			block_duration_dtu * (block_stride_len + 1);
		int add_blocks, add_strides;

		/*
		 * No time in current block, which block should we try? The
		 * result of this can be 0, meaning that we are still in the
		 * same block, but the access was earlier in this block.
		 */
		diff_dtu = session->block_start_dtu -
			   block_duration_margin_dtu - next_timestamp_dtu;
		add_strides = -diff_dtu / block_stride_duration_dtu;
		add_blocks = add_strides * (block_stride_len + 1);

		session->block_start_dtu += add_blocks * block_duration_dtu;
		session->block_index += add_blocks;
		session->sts_index += add_blocks * block_duration_slots;
		if (add_blocks != 0) {
			/*
			 * More than one ranging round skipped, can not trust
			 * last hopping instructions.
			 */
			if (add_blocks > block_stride_len + 1)
				session->hopping_sequence_generation =
					session->params.round_hopping;
			fira_session_update_round_index(session);
		}

		/* Retry in the found block. */
		access_dtu = session->block_start_dtu +
			     fira_session_get_round_slot(session) *
				     session->params.slot_duration_dtu -
			     block_duration_margin_dtu;
		diff_dtu = access_dtu - next_timestamp_dtu;

		if (diff_dtu < 0) {
			/* Still no time, next one will be OK. */
			add_blocks = block_stride_len + 1;
			session->block_start_dtu +=
				add_blocks * block_duration_dtu;
			session->block_index += add_blocks;
			session->sts_index += add_blocks * block_duration_slots;
			fira_session_update_round_index(session);
		}
	}
}

static inline bool
fira_session_has_higher_priority(const struct fira_session *session,
				 const struct fira_session *selected_session)
{
	return session->params.priority > selected_session->params.priority ||
	       (session->params.priority == selected_session->params.priority &&
		is_before_dtu(session->last_access_timestamp_dtu,
			      selected_session->last_access_timestamp_dtu));
}

static struct fira_session *fira_session_find_next(struct fira_local *local,
						   u32 next_timestamp_dtu,
						   u32 max_access_duration_dtu,
						   u32 *timestamp_dtu,
						   u32 *duration_dtu)
{
	struct fira_session *selected_session = NULL;
	struct fira_session *session;
	u32 selected_timestamp_dtu = 0;
	u32 selected_duration_dtu = 0;
	u32 access_timestamp_dtu;
	u32 access_duration_dtu;
	u32 unsync_access_duration_dtu;
	u32 selected_unsync_access_duration_dtu = 0;
	u32 max_unsync_access_duration_dtu = 0;
	bool found_sync_session = false;
	struct mcps802154_region_demand demand;

	/* Select the next synchronised session that can be scheduled without
	 * overlapping any other synchronised sessions or if they are
	 * overlapping, the session with the highest priority. */
	list_for_each_entry (session, &local->active_sessions, entry) {
		if (session->params.device_type == FIRA_DEVICE_TYPE_CONTROLEE &&
		    !session->synchronised)
			continue;
		fira_session_update(local, session, next_timestamp_dtu);
		fira_session_get_demand(local, session, &demand);
		access_timestamp_dtu = demand.timestamp_dtu;
		access_duration_dtu = demand.max_duration_dtu;
		if ((!selected_session ||
		     is_before_dtu(access_timestamp_dtu + access_duration_dtu +
					   local->llhw->anticip_dtu,
				   selected_timestamp_dtu + 1) ||
		     (is_before_dtu(access_timestamp_dtu,
				    selected_timestamp_dtu +
					    selected_duration_dtu +
					    local->llhw->anticip_dtu) &&
		      fira_session_has_higher_priority(session,
						       selected_session))) &&
		    (!max_access_duration_dtu ||
		     access_duration_dtu <= max_access_duration_dtu)) {
			found_sync_session = true;
			selected_session = session;
			selected_timestamp_dtu = access_timestamp_dtu;
			selected_duration_dtu = access_duration_dtu;
		}
	}

	if (found_sync_session)
		max_unsync_access_duration_dtu =
			max((s32)(selected_timestamp_dtu - next_timestamp_dtu -
				  local->llhw->anticip_dtu),
			    0);

	/* Select a session that is not synchronised if there is enough time to
	 * schedule it before the synchronised session currently selected. */
	list_for_each_entry (session, &local->active_sessions, entry) {
		if (session->params.device_type != FIRA_DEVICE_TYPE_CONTROLEE ||
		    session->synchronised)
			continue;
		fira_session_update(local, session, next_timestamp_dtu);
		fira_session_get_demand(local, session, &demand);
		access_duration_dtu = demand.max_duration_dtu;
		unsync_access_duration_dtu = U32_MAX;
		if (session->params.max_rr_retry) {
			int nb_blocks =
				session->params.max_rr_retry *
					(session->block_stride_len + 1) +
				session->last_block_index -
				session->block_index;

			unsync_access_duration_dtu =
				min((u32)session->params.block_duration_dtu *
					    max(nb_blocks, 1),
				    unsync_access_duration_dtu);
		}
		if (found_sync_session)
			unsync_access_duration_dtu =
				min(max_unsync_access_duration_dtu,
				    unsync_access_duration_dtu);
		if (max_access_duration_dtu)
			unsync_access_duration_dtu =
				min(max_access_duration_dtu,
				    unsync_access_duration_dtu);
		/* Among the sessions that are not synchronised, select the one for which the
		 * shortest access needs to be generated. */
		if (access_duration_dtu <= unsync_access_duration_dtu &&
		    (!selected_unsync_access_duration_dtu ||
		     unsync_access_duration_dtu <
			     selected_unsync_access_duration_dtu)) {
			selected_session = session;
			selected_timestamp_dtu = next_timestamp_dtu;
			if (unsync_access_duration_dtu != U32_MAX) {
				selected_unsync_access_duration_dtu =
					selected_duration_dtu =
						unsync_access_duration_dtu;
			} else {
				selected_unsync_access_duration_dtu =
					selected_duration_dtu = 0;
			}
		}
	}

	*timestamp_dtu = selected_timestamp_dtu;
	*duration_dtu = selected_duration_dtu;
	return selected_session;
}

static void
fira_session_check_max_number_of_measurements(struct fira_local *local,
					      struct fira_session *session)
{
	if (!session->max_number_of_measurements_reached &&
	    session->params.max_number_of_measurements &&
	    ((s32)(session->params.max_number_of_measurements -
		   session->number_of_measurements) <= 0)) {
		session->max_number_of_measurements_reached = true;
		session->controlee_management_flags =
			FIRA_SESSION_CONTROLEE_MANAGEMENT_FLAG_STOP;
		fira_session_stop_controlees(session,
					     &session->current_controlees);
	}
}

static bool fira_session_check_max_rr_retry(struct fira_session *session)
{
	if (session->params.max_rr_retry &&
	    !((s32)((session->block_index - session->last_block_index) /
			    (session->block_stride_len + 1) -
		    session->params.max_rr_retry) < 0)) {
		session->stop_no_response = true;
		return true;
	}
	return false;
}

static void
fira_session_send_collision_reports(struct fira_local *local,
				    struct fira_session *selected_session,
				    u32 selected_end_dtu)
{
	struct fira_session *session;
	struct fira_session *tmp_session;
	struct mcps802154_region_demand demand;
	int i;

	list_for_each_entry_safe (session, tmp_session, &local->active_sessions,
				  entry) {
		if (session == selected_session ||
		    (session->params.device_type ==
			     FIRA_DEVICE_TYPE_CONTROLEE &&
		     !session->synchronised))
			continue;
		fira_session_get_demand(local, session, &demand);
		if (is_before_dtu(demand.timestamp_dtu, selected_end_dtu)) {
			fira_compute_access(local, session);
			for (i = 0; i < local->n_ranging_info; i++) {
				local->ranging_info[i].status =
					FIRA_STATUS_RANGING_TX_FAILED;
			}
			fira_session_access_done(local, session, true);
		}
	}
}

static void fira_session_stop_expired_sessions(struct fira_local *local)
{
	struct fira_session *session;
	struct fira_session *tmp_session;
	int i;

	list_for_each_entry_safe (session, tmp_session, &local->active_sessions,
				  entry) {
		if (session == local->current_session ||
		    !fira_session_check_max_rr_retry(session))
			continue;
		fira_compute_access(local, session);
		for (i = 0; i < local->n_ranging_info; i++) {
			local->ranging_info[i].status =
				FIRA_STATUS_RANGING_RX_TIMEOUT;
		}
		fira_session_access_done(local, session, true);
	}
}

static void fira_session_check_unsync(struct fira_local *local,
				      struct fira_session *session)
{
	int nb_blocks;
	int unsync_drift_dtu;
	int block_margin_dtu;

	if ((session->params.device_type != FIRA_DEVICE_TYPE_CONTROLEE) ||
	    !session->synchronised)
		return;

	nb_blocks = session->block_index - session->last_block_index;
	unsync_drift_dtu = (long long)nb_blocks *
			   session->params.block_duration_dtu *
			   FIRA_DRIFT_TOLERANCE_PPM / 1000000;
	block_margin_dtu =
		fira_session_get_block_duration_margin(local, session);
	if (unsync_drift_dtu >= block_margin_dtu)
		session->synchronised = false;
}

struct fira_session *fira_session_next(struct fira_local *local,
				       u32 next_timestamp_dtu,
				       u32 max_access_duration_dtu)
{
	struct fira_session *selected_session;
	u32 selected_timestamp_dtu = 0;
	u32 selected_duration_dtu = 0;
	u32 selected_end_dtu;

	if (list_empty(&local->active_sessions))
		return NULL;

	selected_session = fira_session_find_next(local, next_timestamp_dtu,
						  max_access_duration_dtu,
						  &selected_timestamp_dtu,
						  &selected_duration_dtu);
	if (!selected_session)
		return NULL;
	selected_end_dtu = selected_timestamp_dtu + selected_duration_dtu +
			   local->llhw->anticip_dtu;
	fira_session_send_collision_reports(local, selected_session,
					    selected_end_dtu);
	selected_session->last_access_timestamp_dtu = selected_timestamp_dtu;
	selected_session->last_access_duration_dtu = selected_duration_dtu;
	return selected_session;
}

void fira_session_resync(struct fira_session *session, u32 sts_index,
			 u32 timestamp_dtu)
{
	int block_duration_slots = session->params.block_duration_dtu /
				   session->params.slot_duration_dtu;
	int slot_index = sts_index - session->crypto.sts_index_init;
	int block_index = slot_index / block_duration_slots;
	int round_slot_index = slot_index - block_index * block_duration_slots;

	session->block_index = block_index;
	session->block_start_dtu =
		timestamp_dtu -
		round_slot_index * session->params.slot_duration_dtu;
	session->sts_index = sts_index - round_slot_index;
	session->round_index =
		round_slot_index / session->params.round_duration_slots;
	session->synchronised = true;
	session->last_access_timestamp_dtu = timestamp_dtu;
}

void fira_session_access_done(struct fira_local *local,
			      struct fira_session *session,
			      bool add_measurements)
{
	if (session->controlee_management_flags ==
	    FIRA_SESSION_CONTROLEE_MANAGEMENT_FLAG_STOP) {
		fira_session_update_stopping_controlees(session);
		session->controlee_management_flags = 0;
	}

	if (session == local->current_session) {
		if (!(session->params.device_type ==
			      FIRA_DEVICE_TYPE_CONTROLEE &&
		      local->ranging_info[0].status) &&
		    !(session->params.device_type ==
			      FIRA_DEVICE_TYPE_CONTROLLER &&
		      local->n_ranging_valid != local->n_ranging_info)) {
			session->last_block_index = session->block_index;
		} else {
			fira_session_check_unsync(local, session);
		}
		session->number_of_measurements++;
		session->params.meas_seq.n_measurements_achieved++;
	}

	fira_session_check_max_number_of_measurements(local, session);
	fira_session_check_max_rr_retry(session);
	fira_report(local, session, add_measurements);

	if (session->controlee_management_flags &
	    FIRA_SESSION_CONTROLEE_MANAGEMENT_FLAG_UPDATE) {
		fira_session_copy_controlees(&session->current_controlees,
					     &session->new_controlees);
		session->controlee_management_flags &=
			~FIRA_SESSION_CONTROLEE_MANAGEMENT_FLAG_UPDATE;
	}

	if (((session->stop_request ||
	      session->max_number_of_measurements_reached) &&
	     !(session->controlee_management_flags &
	       FIRA_SESSION_CONTROLEE_MANAGEMENT_FLAG_STOP)) ||
	    session->stop_inband || session->stop_no_response) {
		list_move(&session->entry, &local->inactive_sessions);
		session->stop_request = false;
		session->stop_inband = false;
		session->stop_no_response = false;
		session->max_number_of_measurements_reached = false;
		/* Reset data parameters. */
		session->params.data_payload_seq = 0;
		session->params.data_payload_len = 0;
	}

	if (session == local->current_session) {
		fira_session_stop_expired_sessions(local);
		session->block_stride_len = session->next_block_stride_len;
	}
}
