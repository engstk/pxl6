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

#undef TRACE_SYSTEM
#define TRACE_SYSTEM mcps802154_region_fira

#if !defined(FIRA_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define FIRA_TRACE_H

#include <linux/tracepoint.h>
#include "fira_session.h"
#include "fira_region_params.h"

/* clang-format off */

#define FIRA_SESSION_ENTRY __field(u32, session_id)
#define FIRA_SESSION_ASSIGN __entry->session_id = session->id
#define FIRA_SESSION_PR_FMT "session_id=%d"
#define FIRA_SESSION_PR_ARG __entry->session_id

#define FIRA_DEVICE_TYPE_SYMBOLS                      \
	{ FIRA_DEVICE_TYPE_CONTROLEE, "controlee" },  \
	{ FIRA_DEVICE_TYPE_CONTROLLER, "controller" }

#define FIRA_DEVICE_ROLE_SYMBOLS                      \
	{ FIRA_DEVICE_ROLE_RESPONDER, "responder" },  \
	{ FIRA_DEVICE_ROLE_INITIATOR, "initiator" }

#define FIRA_RANGING_ROUND_SYMBOLS                    \
	{ FIRA_RANGING_ROUND_USAGE_OWR, "OWR" },      \
	{ FIRA_RANGING_ROUND_USAGE_SSTWR, "SSTWR" },  \
	{ FIRA_RANGING_ROUND_USAGE_DSTWR, "DSTWR" }

#define FIRA_MULTI_NODE_MODE_SYMBOLS                          \
	{ FIRA_MULTI_NODE_MODE_UNICAST, "UNICAST" },          \
	{ FIRA_MULTI_NODE_MODE_ONE_TO_MANY, "ONE_TO_MANY" },  \
	{ FIRA_MULTI_NODE_MODE_MANY_TO_MANY, "MANY_TO_MANY" }

#define FIRA_RFRAME_CONFIG_SYMBOLS          \
	{ FIRA_RFRAME_CONFIG_SP0, "SP0" },  \
	{ FIRA_RFRAME_CONFIG_SP1, "SP1" },  \
	{ FIRA_RFRAME_CONFIG_SP2, "SP2" },  \
	{ FIRA_RFRAME_CONFIG_SP3, "SP3" }

#define FIRA_PREAMBULE_DURATION_SYMBOLS        \
	{ FIRA_PREAMBULE_DURATION_32, "32" },  \
	{ FIRA_PREAMBULE_DURATION_64, "64" }

#define FIRA_PSDU_DATA_RATE_SYMBOLS            \
	{ FIRA_PSDU_DATA_RATE_6M81, "6M81" },  \
	{ FIRA_PSDU_DATA_RATE_7M80, "7M80" },  \
	{ FIRA_PSDU_DATA_RATE_27M2, "27M2" },  \
	{ FIRA_PSDU_DATA_RATE_31M2, "31M2" }

#define FIRA_PHR_DATA_RATE_SYMBOLS            \
	{ FIRA_PHR_DATA_RATE_850k, "850k" },  \
	{ FIRA_PHR_DATA_RATE_6M81, "6M81" }

#define FIRA_MAC_FCS_TYPE_CRC_SYMBOLS        \
	{ FIRA_MAC_FCS_TYPE_CRC_16, "16" },  \
	{ FIRA_MAC_FCS_TYPE_CRC_32, "32" }

#define FIRA_STS_CONFIG_SYMBOLS                  \
	{ FIRA_STS_CONFIG_STATIC, "static" },    \
	{ FIRA_STS_CONFIG_DYNAMIC, "dynamic" },  \
	{ FIRA_STS_CONFIG_DYNAMIC_INDIVIDUAL_KEY, "dynamic_individual_key" }

#define FIRA_RX_ANTENNA_SWITCH_SYMBOLS                              \
	{ FIRA_RX_ANTENNA_SWITCH_BETWEEN_ROUND, "between_round" },  \
	{ FIRA_RX_ANTENNA_SWITCH_DURING_ROUND, "during_round" },    \
	{ FIRA_RX_ANTENNA_SWITCH_TWO_RANGING, "two_ranging" }

#define FIRA_MESSAGE_TYPE                              \
	{ FIRA_MESSAGE_ID_RANGING_INITIATION, "RIM" }, \
	{ FIRA_MESSAGE_ID_RANGING_RESPONSE, "RRM" },   \
	{ FIRA_MESSAGE_ID_RANGING_FINAL, "RFM" },      \
	{ FIRA_MESSAGE_ID_CONTROL, "RCM" },            \
	{ FIRA_MESSAGE_ID_MEASUREMENT_REPORT, "MRM" }, \
	{ FIRA_MESSAGE_ID_RESULT_REPORT, "RRRM" },     \
	{ FIRA_MESSAGE_ID_CONTROL_UPDATE, "CMU" }

#define FIRA_RANGING_STATUS                                                   \
	{ FIRA_STATUS_RANGING_SUCCESS, "success" },                           \
	{ FIRA_STATUS_RANGING_TX_FAILED, "tx_failed" },                       \
	{ FIRA_STATUS_RANGING_RX_TIMEOUT, "rx_timeout" },                     \
	{ FIRA_STATUS_RANGING_RX_PHY_DEC_FAILED, "rx_phy_dec_failed" },       \
	{ FIRA_STATUS_RANGING_RX_PHY_TOA_FAILED, "rx_phy_toa_failed" },       \
	{ FIRA_STATUS_RANGING_RX_PHY_STS_FAILED, "rx_phy_sts_failed" },       \
	{ FIRA_STATUS_RANGING_RX_MAC_DEC_FAILED, "rx_mac_dec_failed" },       \
	{ FIRA_STATUS_RANGING_RX_MAC_IE_DEC_FAILED, "rx_mac_ie_dec_failed" }, \
	{ FIRA_STATUS_RANGING_RX_MAC_IE_MISSING, "rx_mac_ie_missing" }

TRACE_EVENT(region_fira_session_params,
	TP_PROTO(const struct fira_session *session,
		 const struct fira_session_params *params),
	TP_ARGS(session, params),
	TP_STRUCT__entry(
		FIRA_SESSION_ENTRY
		__field(enum fira_device_type, device_type)
		__field(enum fira_ranging_round_usage, ranging_round_usage)
		__field(enum fira_multi_node_mode, multi_node_mode)
		__field(__le16, controller_short_addr)
		__field(int, initiation_time_ms)
		__field(int, slot_duration_dtu)
		__field(int, block_duration_dtu)
		__field(int, round_duration_slots)
		__field(bool, round_hopping)
		__field(int, priority)
		__field(int, channel_number)
		__field(int, preamble_code_index)
		__field(enum fira_rframe_config, rframe_config)
		__field(enum fira_preambule_duration, preamble_duration)
		__field(enum fira_sfd_id, sfd_id)
		__field(enum fira_psdu_data_rate, psdu_data_rate)
		__field(enum fira_mac_fcs_type, mac_fcs_type)
		__field(enum fira_sts_config, sts_config)
		__array(u8, vupper64, FIRA_VUPPER64_SIZE)
		__field(bool, aoa_result_req)
		__field(bool, report_tof)
		__field(bool, report_aoa_azimuth)
		__field(bool, report_aoa_elevation)
		__field(bool, report_aoa_fom)
		__field(u8, rx_antenna_selection)
		__field(u8, rx_antenna_pair_azimuth)
		__field(u8, rx_antenna_pair_elevation)
		__field(u8, tx_antenna_selection)
		__field(u8, rx_antenna_switch)
		),
	TP_fast_assign(
		FIRA_SESSION_ASSIGN;
		__entry->device_type = params->device_type;
		__entry->ranging_round_usage = params->ranging_round_usage;
		__entry->multi_node_mode = params->multi_node_mode;
		__entry->controller_short_addr = params->controller_short_addr;
		__entry->initiation_time_ms = params->initiation_time_ms;
		__entry->slot_duration_dtu = params->slot_duration_dtu;
		__entry->block_duration_dtu = params->block_duration_dtu;
		__entry->round_duration_slots = params->round_duration_slots;
		__entry->round_hopping = params->round_hopping;
		__entry->priority = params->priority;
		__entry->channel_number = params->channel_number;
		__entry->preamble_code_index = params->preamble_code_index;
		__entry->rframe_config = params->rframe_config;
		__entry->preamble_duration = params->preamble_duration;
		__entry->sfd_id = params->sfd_id;
		__entry->psdu_data_rate = params->psdu_data_rate;
		__entry->mac_fcs_type = params->mac_fcs_type;
		__entry->sts_config = params->sts_config;
		memcpy(__entry->vupper64, params->vupper64, FIRA_VUPPER64_SIZE);
		__entry->aoa_result_req = params->aoa_result_req;
		__entry->report_tof = params->report_tof;
		__entry->report_aoa_azimuth = params->report_aoa_azimuth;
		__entry->report_aoa_elevation = params->report_aoa_elevation;
		__entry->report_aoa_fom = params->report_aoa_fom;
		__entry->rx_antenna_selection = params->rx_antenna_selection;
		__entry->rx_antenna_pair_azimuth = params->rx_antenna_pair_azimuth;
		__entry->rx_antenna_pair_elevation = params->rx_antenna_pair_elevation;
		__entry->tx_antenna_selection = params->tx_antenna_selection;
		__entry->rx_antenna_switch = params->rx_antenna_switch;
		),
	TP_printk(FIRA_SESSION_PR_FMT " device_type=%s ranging_round_usage=%s multi_node_mode=%s "
		  "controller_short_addr=0x%x initiation_time_ms=%d slot_duration_dtu=%d "
		  "block_duration_dtu=%d round_duration_slots=%d round_hopping=%d "
		  "priority=%d channel_number=%d preamble_code_index=%d rframe_config=%s "
		  "preamble_duration=%s sfd_id=%d psdu_data_rate=%s mac_fcs_type=%s "
		  "sts_config=%s vupper64=%s aoa_result_req=%d report_tof=%d report_aoa_azimuth=%d "
		  "report_aoa_elevation=%d report_aoa_fom=%d rx_antenna_selection=0x%x "
		  "rx_antenna_pair_azimuth=%d rx_antenna_pair_elevation=%d "
		  "tx_antenna_selection=%d rx_antenna_switch=%s",
		  FIRA_SESSION_PR_ARG,
		  __print_symbolic(__entry->device_type, FIRA_DEVICE_TYPE_SYMBOLS),
		  __print_symbolic(__entry->ranging_round_usage, FIRA_RANGING_ROUND_SYMBOLS),
		  __print_symbolic(__entry->multi_node_mode, FIRA_MULTI_NODE_MODE_SYMBOLS),
		  __entry->controller_short_addr,
		  __entry->initiation_time_ms,
		  __entry->slot_duration_dtu,
		  __entry->block_duration_dtu,
		  __entry->round_duration_slots,
		  __entry->round_hopping,
		  __entry->priority,
		  __entry->channel_number,
		  __entry->preamble_code_index,
		  __print_symbolic(__entry->rframe_config, FIRA_RFRAME_CONFIG_SYMBOLS),
		  __print_symbolic(__entry->preamble_duration, FIRA_PREAMBULE_DURATION_SYMBOLS),
		  __entry->sfd_id,
		  __print_symbolic(__entry->psdu_data_rate, FIRA_PSDU_DATA_RATE_SYMBOLS),
		  __print_symbolic(__entry->mac_fcs_type, FIRA_MAC_FCS_TYPE_CRC_SYMBOLS),
		  __print_symbolic(__entry->sts_config, FIRA_STS_CONFIG_SYMBOLS),
		  __print_hex(__entry->vupper64, FIRA_VUPPER64_SIZE),
		  __entry->aoa_result_req,
		  __entry->report_tof,
		  __entry->report_aoa_azimuth,
		  __entry->report_aoa_elevation,
		  __entry->report_aoa_fom,
		  __entry->rx_antenna_selection,
		  __entry->rx_antenna_pair_azimuth,
		  __entry->rx_antenna_pair_elevation,
		  __entry->tx_antenna_selection,
		  __print_symbolic(__entry->rx_antenna_switch, FIRA_RX_ANTENNA_SWITCH_SYMBOLS)
		)
	);

TRACE_EVENT(region_fira_rx_message_type,
	TP_PROTO(const struct fira_session *session,
		 enum fira_message_id msg_id,
		 enum fira_ranging_status status),
	TP_ARGS(session, msg_id, status),
	TP_STRUCT__entry(
		FIRA_SESSION_ENTRY
		__field(enum fira_message_id, msg_id)
		__field(enum fira_ranging_status, status)
		),
	TP_fast_assign(
		FIRA_SESSION_ASSIGN;
		__entry->msg_id = msg_id;
		__entry->status = status;
		),
	TP_printk(FIRA_SESSION_PR_FMT " message_type=%s status=%s",
		FIRA_SESSION_PR_ARG,
		__print_symbolic(__entry->msg_id, FIRA_MESSAGE_TYPE),
		__print_symbolic(__entry->status, FIRA_RANGING_STATUS)
		)
	);

TRACE_EVENT(region_fira_tx_message_type,
	TP_PROTO(const struct fira_session *session,
		 enum fira_message_id msg_id),
	TP_ARGS(session, msg_id),
	TP_STRUCT__entry(
		FIRA_SESSION_ENTRY
		__field(enum fira_message_id, msg_id)
		),
	TP_fast_assign(
		FIRA_SESSION_ASSIGN;
		__entry->msg_id = msg_id;
		),
	TP_printk(FIRA_SESSION_PR_FMT " message_type=%s",
		FIRA_SESSION_PR_ARG,
		__print_symbolic(__entry->msg_id, FIRA_MESSAGE_TYPE)
		)
	);

#endif /* !FIRA_TRACE_H || TRACE_HEADER_MULTI_READ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE fira_trace
#include <trace/define_trace.h>
