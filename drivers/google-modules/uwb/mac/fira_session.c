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
 * FiRa sessions management.
 *
 */

#include "fira_session.h"
#include "fira_crypto.h"
#include "fira_round_hopping_sequence.h"
#include "fira_access.h"

#include <linux/errno.h>
#include <linux/ieee802154.h>
#include <linux/string.h>

/**
 * fira_session_controlees_max() - Calculate the maximum number of controlees
 * for current session.
 * @params: Session params.
 *
 * Return: Maximum number of controlees possible with current parameters.
 */
static size_t fira_session_controlees_max(struct fira_session_params *params)
{
	/* TODO: use parameters (embedded mode, ranging mode, device type...)
	   to calculate the size of frames, number of messages...
	   Currently using default parameters configuration. */
	static const u8 mrm_size_without_delays = 49;
	static const u8 delay_size_per_controlee = 6;
	static const u8 rcm_size_without_slots = 45;
	static const u8 slots_size = 4;
	static const u8 controller_messages = 4;
	static const u8 controlee_messages = 2;
	static const u8 frame_size_max = 125;

	static const size_t mrm_max_controlees =
		(frame_size_max - mrm_size_without_delays) /
		delay_size_per_controlee;

	static const size_t rcm_max_controlees =
		(frame_size_max - rcm_size_without_slots -
		 slots_size * controller_messages) /
		(slots_size * controlee_messages);

	const size_t controlees_max =
		min(mrm_max_controlees, rcm_max_controlees);

	return controlees_max;
}

struct fira_session *fira_session_new(struct fira_local *local, u32 session_id)
{
	struct fira_session *session;

	session = kzalloc(sizeof(*session), GFP_KERNEL);
	if (!session)
		return NULL;

	session->id = session_id;
	session->params.ranging_round_usage = FIRA_RANGING_ROUND_USAGE_DSTWR;
	session->params.controller_short_addr = IEEE802154_ADDR_SHORT_BROADCAST;
	session->params.initiation_time_ms =
		local->llhw->anticip_dtu / (local->llhw->dtu_freq_hz / 1000);
	session->params.slot_duration_dtu =
		FIRA_SLOT_DURATION_RSTU_DEFAULT * local->llhw->rstu_dtu;
	session->params.block_duration_dtu = FIRA_BLOCK_DURATION_MS_DEFAULT *
					     (local->llhw->dtu_freq_hz / 1000);
	session->params.round_duration_slots =
		FIRA_ROUND_DURATION_SLOTS_DEFAULT;
	session->params.priority = FIRA_PRIORITY_DEFAULT;
	session->params.round_hopping = false;
	session->params.rframe_config = FIRA_RFRAME_CONFIG_SP3;
	session->params.preamble_duration = FIRA_PREAMBULE_DURATION_64;
	session->params.sfd_id = FIRA_SFD_ID_2;

	/* Antenna parameters which have a default value not equal to zero. */
	session->params.rx_antenna_pair_azimuth = FIRA_RX_ANTENNA_PAIR_INVALID;
	session->params.rx_antenna_pair_elevation =
		FIRA_RX_ANTENNA_PAIR_INVALID;
	session->params.tx_antenna_selection = 0x01;
	/* Report parameters. */
	session->params.aoa_result_req = true;
	session->params.report_tof = true;
	session->params.n_controlees_max = FIRA_CONTROLEES_MAX;

	if (fira_round_hopping_sequence_init(session))
		goto failed;

	list_add(&session->entry, &local->inactive_sessions);

	return session;
failed:
	kfree(session);
	return NULL;
}

void fira_session_free(struct fira_local *local, struct fira_session *session)
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

int fira_session_new_controlees(struct fira_local *local,
				struct fira_session *session,
				struct fira_controlees_array *controlees_array,
				const struct fira_controlee *controlees,
				size_t n_controlees)
{
	int i, j;

	/* On inactive session, the max is the size of the array.
	 * And on active session, the size depend to the config. */
	if (controlees_array->size + n_controlees >
	    session->params.n_controlees_max)
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

int fira_session_del_controlees(struct fira_local *local,
				struct fira_session *session,
				struct fira_controlees_array *controlees_array,
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

bool fira_session_is_ready(struct fira_local *local,
			   struct fira_session *session)
{
	int round_duration_dtu;
	struct fira_session_params *params = &session->params;

	if (params->multi_node_mode == FIRA_MULTI_NODE_MODE_UNICAST) {
		if (params->current_controlees.size > 1)
			return false;
	} else {
		params->n_controlees_max = fira_session_controlees_max(params);
		if (params->current_controlees.size > params->n_controlees_max)
			return false;
	}
	/* RFRAME (INITIATION and FINAL) reception on different antenna is
	 * not implemented on CONTROLLER. */
	if (params->rx_antenna_switch == FIRA_RX_ANTENNA_SWITCH_DURING_ROUND &&
	    params->device_type == FIRA_DEVICE_TYPE_CONTROLLER)
		return false;

	round_duration_dtu =
		params->slot_duration_dtu * params->round_duration_slots;
	return params->slot_duration_dtu != 0 &&
	       params->block_duration_dtu != 0 &&
	       params->round_duration_slots != 0 &&
	       round_duration_dtu < params->block_duration_dtu;
}

void fira_session_round_hopping(struct fira_session *session)
{
	if (session->hopping_sequence_generation) {
		session->round_index = fira_round_hopping_sequence_get(
			session, session->block_index);
		session->next_round_index = fira_round_hopping_sequence_get(
			session, session->block_index + 1);
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
	s32 diff_dtu = session->block_start_dtu - next_timestamp_dtu;

	if (diff_dtu < 0) {
		int block_duration_dtu = session->params.block_duration_dtu;
		int block_duration_slots =
			block_duration_dtu / session->params.slot_duration_dtu;
		int add_blocks;

		add_blocks =
			(-diff_dtu + block_duration_dtu) / block_duration_dtu;
		session->block_start_dtu += add_blocks * block_duration_dtu;
		session->block_index += add_blocks;
		session->sts_index += add_blocks * block_duration_slots;
		if (add_blocks != 1)
			session->hopping_sequence_generation =
				session->params.round_hopping;
		fira_session_round_hopping(session);
	}
}

static struct fira_session *
fira_session_find_next(struct fira_local *local, u32 next_timestamp_dtu,
		       u32 *timestamp_dtu, u32 *duration_dtu, bool *bounded)
{
	struct fira_session *selected_session = NULL;
	struct fira_session *session;
	u32 selected_timestamp_dtu = 0;
	u32 selected_duration_dtu = 0;
	u32 access_timestamp_dtu;
	u32 access_duration_dtu;
	struct mcps802154_region_demand demand;

	*bounded = false;
	list_for_each_entry (session, &local->active_sessions, entry) {
		if (session->params.device_type == FIRA_DEVICE_TYPE_CONTROLEE &&
		    !session->synchronised)
			continue;
		*bounded = true;
		fira_session_update(local, session, next_timestamp_dtu);
		fira_get_demand(local, session, &demand);
		access_timestamp_dtu = demand.timestamp_dtu;
		access_duration_dtu = demand.duration_dtu;
		if (!selected_session ||
		    (s32)(access_timestamp_dtu + access_duration_dtu +
			  local->llhw->anticip_dtu - selected_timestamp_dtu) <=
			    0 ||
		    ((s32)(access_timestamp_dtu - selected_timestamp_dtu -
			   selected_duration_dtu) < 0 &&
		     (session->params.priority >
			      selected_session->params.priority ||
		      (session->params.priority ==
			       selected_session->params.priority &&
		       (s32)(session->last_access_timestamp_dtu -
			     selected_session->last_access_timestamp_dtu) <
			       0)))) {
			selected_session = session;
			selected_timestamp_dtu = access_timestamp_dtu;
			selected_duration_dtu = access_duration_dtu;
		}
	}

	list_for_each_entry (session, &local->active_sessions, entry) {
		if (session->params.device_type != FIRA_DEVICE_TYPE_CONTROLEE ||
		    session->synchronised)
			continue;
		fira_session_update(local, session, next_timestamp_dtu);
		fira_get_demand(local, session, &demand);
		access_timestamp_dtu = demand.timestamp_dtu;
		access_duration_dtu = demand.duration_dtu;
		if (!selected_session) {
			selected_session = session;
			selected_timestamp_dtu = next_timestamp_dtu;
			selected_duration_dtu = access_duration_dtu;
			break;
		} else if ((s32)(selected_timestamp_dtu - next_timestamp_dtu -
				 access_duration_dtu -
				 local->llhw->anticip_dtu) >= 0) {
			selected_session = session;
			selected_duration_dtu = selected_timestamp_dtu -
						next_timestamp_dtu -
						local->llhw->anticip_dtu;
			selected_timestamp_dtu = next_timestamp_dtu;
			break;
		}
	}

	*timestamp_dtu = selected_timestamp_dtu;
	*duration_dtu = selected_duration_dtu;
	return selected_session;
}

static void
fira_session_send_collision_reports(struct fira_local *local,
				    struct fira_session *selected_session,
				    u32 selected_end_dtu)
{
	struct fira_session *session;
	struct mcps802154_region_demand demand;
	int i;

	list_for_each_entry (session, &local->active_sessions, entry) {
		if (session == selected_session ||
		    (session->params.device_type ==
			     FIRA_DEVICE_TYPE_CONTROLEE &&
		     !session->synchronised))
			continue;
		fira_get_demand(local, session, &demand);
		if (demand.timestamp_dtu < selected_end_dtu) {
			fira_compute_access(local, session);
			for (i = 0; i < local->n_ranging_info; i++) {
				local->ranging_info[i].failed = true;
			}
			fira_report(local, session, true);
		}
	}
}

struct fira_session *fira_session_next(struct fira_local *local,
				       u32 next_timestamp_dtu)
{
	struct fira_session *selected_session;
	u32 selected_timestamp_dtu = 0;
	u32 selected_duration_dtu = 0;
	bool bounded = false;
	u32 selected_end_dtu;

	if (list_empty(&local->active_sessions))
		return NULL;

	selected_session = fira_session_find_next(local, next_timestamp_dtu,
						  &selected_timestamp_dtu,
						  &selected_duration_dtu,
						  &bounded);
	selected_end_dtu = selected_timestamp_dtu + selected_duration_dtu +
			   local->llhw->anticip_dtu;
	fira_session_send_collision_reports(local, selected_session,
					    selected_end_dtu);
	selected_session->last_access_timestamp_dtu = selected_timestamp_dtu;
	if (selected_session->params.device_type ==
		    FIRA_DEVICE_TYPE_CONTROLEE &&
	    (list_is_singular(&local->active_sessions) || !bounded))
		selected_session->last_access_duration_dtu = 0;
	else
		selected_session->last_access_duration_dtu =
			selected_duration_dtu;
	return selected_session;
}

void fira_session_resync(struct fira_local *local, struct fira_session *session,
			 u32 sts_index, u32 timestamp_dtu)
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
			      struct fira_session *session)
{
	fira_report(local, session, session == local->current_session);

	if (session->params.update_controlees) {
		fira_session_copy_controlees(
			&session->params.current_controlees,
			&session->params.new_controlees);
		session->params.update_controlees = false;
	}

	if (session->stop_request) {
		list_move(&session->entry, &local->inactive_sessions);
		session->stop_request = false;
		/* Reset to max value as it will be recomputed on session start
		 * with fira_session_is_ready call. */
		session->params.n_controlees_max = FIRA_CONTROLEES_MAX;
	}
}
