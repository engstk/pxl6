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
 * 802.15.4 mac common part sublayer, pctt ranging region.
 *
 */

#ifndef NET_PCTT_REGION_PARAMS_H
#define NET_PCTT_REGION_PARAMS_H

#include <linux/types.h>

#define PCTT_VUPPER64_SIZE 8
#define PCTT_KEY_SIZE_MAX 32
#define PCTT_KEY_SIZE_MIN 16
#define PCTT_CONTROLEES_MAX 16
#define PCTT_RX_ANTENNA_PAIR_INVALID 0xff
/*
 * In BPRF, frame is at most 127
 * 127 - (MHR + HIE + HT + PIE_Header + V_OUI + MIC + CRC)
 */
#define PCTT_DATA_PAYLOAD_SIZE_MAX 84

enum pctt_device_role {
	PCTT_DEVICE_ROLE_RESPONDER,
	PCTT_DEVICE_ROLE_INITIATOR,
};

enum pctt_rframe_config {
	PCTT_RFRAME_CONFIG_SP0,
	PCTT_RFRAME_CONFIG_SP1,
	PCTT_RFRAME_CONFIG_SP2,
	PCTT_RFRAME_CONFIG_SP3,
};

enum pctt_prf_mode {
	PCTT_PRF_MODE_BPRF,
	PCTT_PRF_MODE_HPRF,
};

enum pctt_preamble_duration {
	PCTT_PREAMBLE_DURATION_32,
	PCTT_PREAMBLE_DURATION_64,
};

enum pctt_sfd_id {
	PCTT_SFD_ID_0,
	PCTT_SFD_ID_1,
	PCTT_SFD_ID_2,
	PCTT_SFD_ID_3,
	PCTT_SFD_ID_4,
};

enum pctt_number_of_sts_segments {
	PCTT_NUMBER_OF_STS_SEGMENTS_NONE,
	PCTT_NUMBER_OF_STS_SEGMENTS_1_SEGMENT,
	PCTT_NUMBER_OF_STS_SEGMENTS_2_SEGMENTS,
};

enum pctt_psdu_data_rate {
	PCTT_PSDU_DATA_RATE_6M81,
	PCTT_PSDU_DATA_RATE_7M80,
	PCTT_PSDU_DATA_RATE_27M2,
	PCTT_PSDU_DATA_RATE_31M2,
};

enum pctt_mac_fcs_type {
	PCTT_MAC_FCS_TYPE_CRC_16,
	PCTT_MAC_FCS_TYPE_CRC_32,
};

/**
 * enum pctt_status_ranging - Ranging status: success or failure reason.
 * @PCTT_STATUS_RANGING_INTERNAL_ERROR: Implementation specific error.
 * @PCTT_STATUS_RANGING_SUCCESS: Ranging info are valid.
 * @PCTT_STATUS_RANGING_TX_FAILED: Failed to transmit UWB packet.
 * @PCTT_STATUS_RANGING_RX_TIMEOUT: No UWB packet detected by the receiver.
 * @PCTT_STATUS_RANGING_RX_PHY_DEC_FAILED: UWB packet channel decoding error.
 * @PCTT_STATUS_RANGING_RX_PHY_TOA_FAILED: Failed to detect time of arrival of
 * the UWB packet from CIR samples.
 * @PCTT_STATUS_RANGING_RX_PHY_STS_FAILED: UWB packet STS segment mismatch.
 * @PCTT_STATUS_RANGING_RX_MAC_DEC_FAILED: MAC CRC or syntax error.
 * @PCTT_STATUS_RANGING_RX_MAC_IE_DEC_FAILED: IE syntax error.
 * @PCTT_STATUS_RANGING_RX_MAC_IE_MISSING: Expected IE missing in the packet.
 */
enum pctt_status_ranging {
	PCTT_STATUS_RANGING_INTERNAL_ERROR = -1,
	PCTT_STATUS_RANGING_SUCCESS = 0,
	PCTT_STATUS_RANGING_TX_FAILED = 1,
	PCTT_STATUS_RANGING_RX_TIMEOUT = 2,
	PCTT_STATUS_RANGING_RX_PHY_DEC_FAILED = 3,
	PCTT_STATUS_RANGING_RX_PHY_TOA_FAILED = 4,
	PCTT_STATUS_RANGING_RX_PHY_STS_FAILED = 5,
	PCTT_STATUS_RANGING_RX_MAC_DEC_FAILED = 6,
	PCTT_STATUS_RANGING_RX_MAC_IE_DEC_FAILED = 7,
	PCTT_STATUS_RANGING_RX_MAC_IE_MISSING = 8,
};

/**
 * enum pctt_session_state - Session state.
 * @PCTT_SESSION_STATE_INIT: Initial state, session is not ready yet.
 * @PCTT_SESSION_STATE_DEINIT: Session does not exist.
 * @PCTT_SESSION_STATE_ACTIVE: Session is currently active.
 * @PCTT_SESSION_STATE_IDLE: Session is ready to start, but not currently
 * active.
 */
enum pctt_session_state {
	PCTT_SESSION_STATE_INIT,
	PCTT_SESSION_STATE_DEINIT,
	PCTT_SESSION_STATE_ACTIVE,
	PCTT_SESSION_STATE_IDLE,
};

#endif /* NET_PCTT_REGION_PARAMS_H */
