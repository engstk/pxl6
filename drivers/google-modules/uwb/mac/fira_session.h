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

#ifndef NET_MCPS802154_FIRA_SESSION_H
#define NET_MCPS802154_FIRA_SESSION_H

#include "fira_region.h"
#include "fira_crypto.h"
#include "fira_round_hopping_crypto_impl.h"

struct fira_controlees_array {
	struct fira_controlee data[FIRA_CONTROLEES_MAX];
	/* Number of data valid. */
	size_t size;
};

struct fira_session_params {
	/* Main parameters. */
	enum fira_device_type device_type;
	enum fira_ranging_round_usage ranging_round_usage;
	enum fira_multi_node_mode multi_node_mode;
	__le16 controller_short_addr;
	/* Timings parameters. */
	int initiation_time_ms;
	int slot_duration_dtu;
	int block_duration_dtu;
	int round_duration_slots;
	/* Behaviour parameters. */
	bool round_hopping;
	int priority;
	/* Radio. */
	int channel_number;
	int preamble_code_index;
	enum fira_rframe_config rframe_config;
	enum fira_preambule_duration preamble_duration;
	enum fira_sfd_id sfd_id;
	enum fira_psdu_data_rate psdu_data_rate;
	enum fira_mac_fcs_type mac_fcs_type;
	/* STS and crypto. */
	enum fira_sts_config sts_config;
	u8 vupper64[FIRA_VUPPER64_SIZE];
	/* List of controlees to applies on next ca. */
	struct fira_controlees_array new_controlees;
	/* List of controlees currently applied. */
	struct fira_controlees_array current_controlees;
	size_t n_controlees_max;
	bool update_controlees;
	bool aoa_result_req;
	bool report_tof;
	bool report_aoa_azimuth;
	bool report_aoa_elevation;
	bool report_aoa_fom;
	u8 rx_antenna_selection;
	u8 rx_antenna_pair_azimuth;
	u8 rx_antenna_pair_elevation;
	u8 tx_antenna_selection;
	u8 rx_antenna_switch;
};

/**
 * struct fira_session - Session information.
 */
struct fira_session {
	/**
	 * @id: Session identifier.
	 */
	u32 id;
	/**
	 * @entry: Entry in list of sessions.
	 */
	struct list_head entry;
	/**
	 * @params: Session parameters, mostly read only while the session is
	 * active.
	 */
	struct fira_session_params params;
	/**
	 * @block_start_dtu: Timestamp of the current or previous block. All
	 * other fields are referring to this same block.
	 */
	u32 block_start_dtu;
	/**
	 * @block_index: Block index of the reference block.
	 */
	u32 block_index;
	/**
	 * @sts_index: STS index value at reference block start.
	 */
	u32 sts_index;
	/**
	 * @hopping_sequence_generation: Whether to compute round index from ranging round sequence.
	 */
	bool hopping_sequence_generation;
	/**
	 * @round_index: Round index of the reference block.
	 */
	int round_index;
	/**
	 * @next_round_index: Round index of the block after the reference block.
	 */
	int next_round_index;
	/**
	 * @tx_ant: Antenna index to use for transmission in the reference
	 * block.
	 */
	int tx_ant;
	/**
	 * @rx_ant_pair: Antenna pair indexes to use for reception in the
	 * reference block.
	 */
	int rx_ant_pair[2];
	/**
	 * @stop_request: Session has been requested to stop.
	 */
	bool stop_request;
	/**
	 * @crypto: Crypto context.
	 */
	struct fira_crypto crypto;
	/**
	 * @round_hopping_sequence: Round hopping sequence generation context.
	 */
	struct fira_round_hopping_sequence round_hopping_sequence;
	/**
	 * @event_portid: Port identifier to use for notifications.
	 */
	u32 event_portid;
	/**
	 * @synchronised: Whether a controlee session was synchronised. This
	 * field is not used for controller sessions.
	 */
	bool synchronised;
	/**
	 * @last_access_timestamp_dtu: Timestamp of the last computed access.
	 */
	u32 last_access_timestamp_dtu;
	/**
	 * @last_access_duration_dtu: Duration of the last computed access.
	 */
	u32 last_access_duration_dtu;
};

/**
 * fira_session_new() - Create a new session.
 * @local: FiRa context.
 * @session_id: Session identifier, must be unique.
 *
 * Return: The new session or NULL on error.
 */
struct fira_session *fira_session_new(struct fira_local *local, u32 session_id);

/**
 * fira_session_free() - Remove a session.
 * @local: FiRa context.
 * @session: Session to remove, must be inactive.
 */
void fira_session_free(struct fira_local *local, struct fira_session *session);

/**
 * fira_session_get() - Get a session by its identifier.
 * @local: FiRa context.
 * @session_id: Session identifier.
 * @active: When session is found set to true if active, false if inactive.
 *
 * Return: The session or NULL if not found.
 */
struct fira_session *fira_session_get(struct fira_local *local, u32 session_id,
				      bool *active);

/**
 * fira_session_copy_controlees() - copy controlees array between two array.
 * @to: FiRa controlees array to write.
 * @from: FiRa controlees array to read.
 */
void fira_session_copy_controlees(struct fira_controlees_array *to,
				  const struct fira_controlees_array *from);

/**
 * fira_session_new_controlees() - Add new controlees.
 * @local: FiRa context.
 * @session: Session.
 * @controlees_array: Destination array where store new controlees list.
 * @controlees: Controlees information.
 * @n_controlees: Number of controlees.
 *
 * Return: 0 or error.
 */
int fira_session_new_controlees(struct fira_local *local,
				struct fira_session *session,
				struct fira_controlees_array *controlees_array,
				const struct fira_controlee *controlees,
				size_t n_controlees);

/**
 * fira_session_new_controlees() - Remove controlees.
 * @local: FiRa context.
 * @session: Session.
 * @controlees_array: Destination array where store new controlees list.
 * @controlees: Controlees information.
 * @n_controlees: Number of controlees.
 *
 * Return: 0 or error.
 */
int fira_session_del_controlees(struct fira_local *local,
				struct fira_session *session,
				struct fira_controlees_array *controlees_array,
				const struct fira_controlee *controlees,
				size_t n_controlees);

/**
 * fira_session_is_ready() - Test whether a session is ready to be started.
 * @local: FiRa context.
 * @session: Session to test.
 *
 * Return: true if the session can be started.
 */
bool fira_session_is_ready(struct fira_local *local,
			   struct fira_session *session);

/**
 * fira_session_next() - Find the next session to use after the given timestamp.
 * @local: FiRa context.
 * @next_timestamp_dtu: Next access opportunity.
 *
 * Return: The session or NULL if none.
 */
struct fira_session *fira_session_next(struct fira_local *local,
				       u32 next_timestamp_dtu);

/**
 * fira_session_round_hopping() - Update round index for round hopping.
 * @session: Session to update.
 */
void fira_session_round_hopping(struct fira_session *session);

/**
 * fira_session_resync() - Resync session parameters on control message.
 * @local: FiRa context.
 * @session: Session to synchronize.
 * @sts_index: STS index of control message.
 * @timestamp_dtu: Timestamp of control message.
 */
void fira_session_resync(struct fira_local *local, struct fira_session *session,
			 u32 sts_index, u32 timestamp_dtu);

/**
 * fira_session_access_done() - Update session at end of access, or on event
 * when no access is active.
 * @local: FiRa context.
 * @session: Session.
 */
void fira_session_access_done(struct fira_local *local,
			      struct fira_session *session);

/**
 * fira_session_get_round_slot() - Get current round's slot.
 * @session: Session.
 *
 * Return: The first slot of the current round.
 */
static inline u32
fira_session_get_round_slot(const struct fira_session *session)
{
	return session->round_index * session->params.round_duration_slots;
}

/**
 * fira_session_get_round_sts_index() - Get current round's STS index.
 * @session: Session.
 *
 * Return: The STS of the first slot of the current round.
 */
static inline u32
fira_session_get_round_sts_index(const struct fira_session *session)
{
	return session->sts_index + fira_session_get_round_slot(session);
}

#endif /* NET_MCPS802154_FIRA_SESSION_H */
