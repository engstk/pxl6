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

#ifndef NET_FIRA_REGION_PARAMS_H
#define NET_FIRA_REGION_PARAMS_H

#include <linux/types.h>

#define FIRA_VUPPER64_SIZE 8
#define FIRA_KEY_SIZE_MAX 32
#define FIRA_KEY_SIZE_MIN 16
#define FIRA_CONTROLEES_MAX 16
#define FIRA_RX_ANTENNA_PAIR_INVALID 0xff
/*
 * In BPRF, frame is at most 127
 * 127 - (MHR + HIE + HT + PIE_Header + V_OUI + MIC + CRC)
 */
#define FIRA_DATA_PAYLOAD_SIZE_MAX 84

enum fira_device_type {
	FIRA_DEVICE_TYPE_CONTROLEE,
	FIRA_DEVICE_TYPE_CONTROLLER,
};

enum fira_device_role {
	FIRA_DEVICE_ROLE_RESPONDER,
	FIRA_DEVICE_ROLE_INITIATOR,
};

enum fira_ranging_round_usage {
	FIRA_RANGING_ROUND_USAGE_OWR,
	FIRA_RANGING_ROUND_USAGE_SSTWR,
	FIRA_RANGING_ROUND_USAGE_DSTWR,
};

enum fira_multi_node_mode {
	FIRA_MULTI_NODE_MODE_UNICAST,
	FIRA_MULTI_NODE_MODE_ONE_TO_MANY,
	FIRA_MULTI_NODE_MODE_MANY_TO_MANY,
};

enum fira_measurement_report {
	FIRA_MEASUREMENT_REPORT_AT_RESPONDER,
	FIRA_MEASUREMENT_REPORT_AT_INITIATOR,
};

enum fira_embedded_mode {
	FIRA_EMBEDDED_MODE_DEFERRED,
	FIRA_EMBEDDED_MODE_NON_DEFERRED,
};

enum fira_rframe_config {
	FIRA_RFRAME_CONFIG_SP0,
	FIRA_RFRAME_CONFIG_SP1,
	FIRA_RFRAME_CONFIG_SP2,
	FIRA_RFRAME_CONFIG_SP3,
};

enum fira_prf_mode {
	FIRA_PRF_MODE_BPRF,
	FIRA_PRF_MODE_HPRF,
};

enum fira_preambule_duration {
	FIRA_PREAMBULE_DURATION_32,
	FIRA_PREAMBULE_DURATION_64,
};

enum fira_sfd_id {
	FIRA_SFD_ID_0,
	FIRA_SFD_ID_1,
	FIRA_SFD_ID_2,
	FIRA_SFD_ID_3,
	FIRA_SFD_ID_4,
};

enum fira_sts_segments {
	FIRA_STS_SEGMENTS_0,
	FIRA_STS_SEGMENTS_1,
	FIRA_STS_SEGMENTS_2,
};

enum fira_psdu_data_rate {
	FIRA_PSDU_DATA_RATE_6M81,
	FIRA_PSDU_DATA_RATE_7M80,
	FIRA_PSDU_DATA_RATE_27M2,
	FIRA_PSDU_DATA_RATE_31M2,
};

enum fira_phr_data_rate {
	FIRA_PHR_DATA_RATE_850k,
	FIRA_PHR_DATA_RATE_6M81,
};

enum fira_mac_fcs_type {
	FIRA_MAC_FCS_TYPE_CRC_16,
	FIRA_MAC_FCS_TYPE_CRC_32,
};

enum fira_rx_antenna_switch {
	FIRA_RX_ANTENNA_SWITCH_BETWEEN_ROUND,
	FIRA_RX_ANTENNA_SWITCH_DURING_ROUND,
	FIRA_RX_ANTENNA_SWITCH_TWO_RANGING,
};

enum fira_sts_config {
	FIRA_STS_CONFIG_STATIC,
	FIRA_STS_CONFIG_DYNAMIC,
	FIRA_STS_CONFIG_DYNAMIC_INDIVIDUAL_KEY,
};

enum fira_controlee_state {
	FIRA_CONTROLEE_STATE_RUNNING,
	FIRA_CONTROLEE_STATE_PENDING_STOP,
	FIRA_CONTROLEE_STATE_PENDING_DEL,
};

struct fira_controlee {
	u32 sub_session_id;
	__le16 short_addr;
	u16 sub_session_key_len;
	char sub_session_key[FIRA_KEY_SIZE_MAX];
	bool sub_session;
	enum fira_controlee_state state;
};

enum fira_session_controlee_management_flags {
	FIRA_SESSION_CONTROLEE_MANAGEMENT_FLAG_UPDATE = 1,
	FIRA_SESSION_CONTROLEE_MANAGEMENT_FLAG_STOP = 2,
};

/**
 * enum fira_ranging_status - Ranging status: success or failure reason.
 * @FIRA_STATUS_RANGING_INTERNAL_ERROR: Implementation specific error.
 * @FIRA_STATUS_RANGING_SUCCESS: Ranging info are valid.
 * @FIRA_STATUS_RANGING_TX_FAILED: Failed to transmit UWB packet.
 * @FIRA_STATUS_RANGING_RX_TIMEOUT: No UWB packet detected by the receiver.
 * @FIRA_STATUS_RANGING_RX_PHY_DEC_FAILED: UWB packet channel decoding error.
 * @FIRA_STATUS_RANGING_RX_PHY_TOA_FAILED: Failed to detect time of arrival of
 * the UWB packet from CIR samples.
 * @FIRA_STATUS_RANGING_RX_PHY_STS_FAILED: UWB packet STS segment mismatch.
 * @FIRA_STATUS_RANGING_RX_MAC_DEC_FAILED: MAC CRC or syntax error.
 * @FIRA_STATUS_RANGING_RX_MAC_IE_DEC_FAILED: IE syntax error.
 * @FIRA_STATUS_RANGING_RX_MAC_IE_MISSING: Expected IE missing in the packet.
 */
enum fira_ranging_status {
	FIRA_STATUS_RANGING_INTERNAL_ERROR = -1,
	FIRA_STATUS_RANGING_SUCCESS = 0,
	FIRA_STATUS_RANGING_TX_FAILED = 1,
	FIRA_STATUS_RANGING_RX_TIMEOUT = 2,
	FIRA_STATUS_RANGING_RX_PHY_DEC_FAILED = 3,
	FIRA_STATUS_RANGING_RX_PHY_TOA_FAILED = 4,
	FIRA_STATUS_RANGING_RX_PHY_STS_FAILED = 5,
	FIRA_STATUS_RANGING_RX_MAC_DEC_FAILED = 6,
	FIRA_STATUS_RANGING_RX_MAC_IE_DEC_FAILED = 7,
	FIRA_STATUS_RANGING_RX_MAC_IE_MISSING = 8,
};

#endif /* NET_FIRA_REGION_PARAMS_H */
