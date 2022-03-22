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
#define TRACE_SYSTEM mcps802154

#if !defined(TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define TRACE_H

#include <linux/tracepoint.h>
#include "mcps802154_i.h"

/* clang-format off */

#define LOCAL_ENTRY __field(int, hw_idx)
#define LOCAL_ASSIGN __entry->hw_idx = local->hw_idx
#define LOCAL_PR_FMT "hw%d"
#define LOCAL_PR_ARG __entry->hw_idx

TRACE_DEFINE_ENUM(MCPS802154_TX_FRAME_TIMESTAMP_DTU);
TRACE_DEFINE_ENUM(MCPS802154_TX_FRAME_CCA);
TRACE_DEFINE_ENUM(MCPS802154_TX_FRAME_RANGING);
TRACE_DEFINE_ENUM(MCPS802154_TX_FRAME_KEEP_RANGING_CLOCK);
TRACE_DEFINE_ENUM(MCPS802154_TX_FRAME_RANGING_PDOA);
TRACE_DEFINE_ENUM(MCPS802154_TX_FRAME_SP1);
TRACE_DEFINE_ENUM(MCPS802154_TX_FRAME_SP2);
TRACE_DEFINE_ENUM(MCPS802154_TX_FRAME_SP3);
TRACE_DEFINE_ENUM(MCPS802154_TX_FRAME_STS_MODE_MASK);

TRACE_DEFINE_ENUM(MCPS802154_RX_INFO_TIMESTAMP_DTU);
TRACE_DEFINE_ENUM(MCPS802154_RX_INFO_AACK);
TRACE_DEFINE_ENUM(MCPS802154_RX_INFO_RANGING);
TRACE_DEFINE_ENUM(MCPS802154_RX_INFO_KEEP_RANGING_CLOCK);
TRACE_DEFINE_ENUM(MCPS802154_RX_INFO_RANGING_PDOA);
TRACE_DEFINE_ENUM(MCPS802154_RX_INFO_SP1);
TRACE_DEFINE_ENUM(MCPS802154_RX_INFO_SP2);
TRACE_DEFINE_ENUM(MCPS802154_RX_INFO_SP3);
TRACE_DEFINE_ENUM(MCPS802154_RX_INFO_STS_MODE_MASK);

TRACE_DEFINE_ENUM(MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU);
TRACE_DEFINE_ENUM(MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU);
TRACE_DEFINE_ENUM(MCPS802154_RX_FRAME_INFO_LQI);
TRACE_DEFINE_ENUM(MCPS802154_RX_FRAME_INFO_RSSI);
TRACE_DEFINE_ENUM(MCPS802154_RX_FRAME_INFO_RANGING_FOM);
TRACE_DEFINE_ENUM(MCPS802154_RX_FRAME_INFO_RANGING_OFFSET);
TRACE_DEFINE_ENUM(MCPS802154_RX_FRAME_INFO_RANGING_PDOA);
TRACE_DEFINE_ENUM(MCPS802154_RX_FRAME_INFO_RANGING_PDOA_FOM);
TRACE_DEFINE_ENUM(MCPS802154_RX_FRAME_INFO_RANGING_STS_TIMESTAMP_RCTU);
TRACE_DEFINE_ENUM(MCPS802154_RX_FRAME_INFO_RANGING_STS_FOM);
TRACE_DEFINE_ENUM(MCPS802154_RX_FRAME_INFO_AACK);

TRACE_DEFINE_ENUM(MCPS802154_RX_ERROR_BAD_CKSUM);
TRACE_DEFINE_ENUM(MCPS802154_RX_ERROR_UNCORRECTABLE);
TRACE_DEFINE_ENUM(MCPS802154_RX_ERROR_FILTERED);
TRACE_DEFINE_ENUM(MCPS802154_RX_ERROR_SFD_TIMEOUT);
TRACE_DEFINE_ENUM(MCPS802154_RX_ERROR_OTHER);

TRACE_DEFINE_ENUM(IEEE802154_AFILT_SADDR_CHANGED);
TRACE_DEFINE_ENUM(IEEE802154_AFILT_IEEEADDR_CHANGED);
TRACE_DEFINE_ENUM(IEEE802154_AFILT_PANID_CHANGED);
TRACE_DEFINE_ENUM(IEEE802154_AFILT_PANC_CHANGED);

#define RX_FRAME_INFO_FLAGS_ENTRY \
	__field(u16, flags)
#define RX_FRAME_INFO_FLAGS_ASSIGN \
	__entry->flags = info->flags
#define RX_FRAME_INFO_FLAGS_PR_FMT \
	"flags=%s"
#define RX_FRAME_INFO_FLAGS                                            \
	{ MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU, "TIMESTAMP_DTU" },   \
	{ MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU, "TIMESTAMP_RCTU" }, \
	{ MCPS802154_RX_FRAME_INFO_LQI, "LQI" },                       \
	{ MCPS802154_RX_FRAME_INFO_RSSI, "RSSI" },                     \
	{ MCPS802154_RX_FRAME_INFO_RANGING_FOM, "RANGING_FOM" },       \
	{ MCPS802154_RX_FRAME_INFO_RANGING_OFFSET, "RANGING_OFFSET" }, \
	{ MCPS802154_RX_FRAME_INFO_RANGING_PDOA, "RANGING_PDOA" },     \
	{ MCPS802154_RX_FRAME_INFO_RANGING_PDOA_FOM,                   \
	  "RANGING_PDOA_FOM" },                                        \
	{ MCPS802154_RX_FRAME_INFO_RANGING_STS_TIMESTAMP_RCTU,         \
	  "RANGING_STS_TIMESTAMP_RCTU" },                              \
	{ MCPS802154_RX_FRAME_INFO_RANGING_STS_FOM,                    \
	  "RANGING_STS_FOM" },                                         \
	{ MCPS802154_RX_FRAME_INFO_AACK, "AACK" }
#define RX_FRAME_INFO_FLAGS_PR_ARG \
	__print_flags(__entry->flags, "|", RX_FRAME_INFO_FLAGS)

#define RX_FRAME_INFO_ENTRY              \
	__field(u32, timestamp_dtu)      \
	__field(u64, timestamp_rctu)     \
	__field(int, frame_duration_dtu) \
	__field(u8, ranging_sts_fom0)    \
	RX_FRAME_INFO_FLAGS_ENTRY
#define RX_FRAME_INFO_ASSIGN                                    \
	__entry->timestamp_dtu = info->timestamp_dtu;           \
	__entry->timestamp_rctu = info->timestamp_rctu;         \
	__entry->frame_duration_dtu = info->frame_duration_dtu; \
	__entry->ranging_sts_fom0 = info->ranging_sts_fom[0];   \
	RX_FRAME_INFO_FLAGS_ASSIGN
#define RX_FRAME_INFO_PR_FMT                                                \
	"timestamp_dtu=0x%08x timestamp_rctu=%#llx frame_duration_dtu=%d " \
	"ranging_sts_fom[0]=%#02x "                                        \
	RX_FRAME_INFO_FLAGS_PR_FMT
#define RX_FRAME_INFO_PR_ARG         \
	__entry->timestamp_dtu,      \
	__entry->timestamp_rctu,     \
	__entry->frame_duration_dtu, \
	__entry->ranging_sts_fom0,   \
	RX_FRAME_INFO_FLAGS_PR_ARG

#define KEY_ENTRY	__string(key, key)
#define KEY_ASSIGN	__assign_str(key, key)
#define KEY_PR_FMT	"key=%s"
#define KEY_PR_ARG	__get_str(key)

DECLARE_EVENT_CLASS(local_only_evt,
	TP_PROTO(const struct mcps802154_local *local),
	TP_ARGS(local),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		),
	TP_printk(LOCAL_PR_FMT, LOCAL_PR_ARG)
	);

DEFINE_EVENT(local_only_evt, llhw_return_void,
	TP_PROTO(const struct mcps802154_local *local),
	TP_ARGS(local)
	);

TRACE_EVENT(llhw_return_int,
	TP_PROTO(const struct mcps802154_local *local, int ret),
	TP_ARGS(local, ret),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(int, ret)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->ret = ret;
		),
	TP_printk(LOCAL_PR_FMT " returned=%d", LOCAL_PR_ARG, __entry->ret)
	);

TRACE_EVENT(llhw_return_rx_frame,
	TP_PROTO(const struct mcps802154_local *local, int ret,
		 const struct mcps802154_rx_frame_info *info),
	TP_ARGS(local, ret, info),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(int, ret)
		RX_FRAME_INFO_ENTRY
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->ret = ret;
		RX_FRAME_INFO_ASSIGN;
		),
	TP_printk(LOCAL_PR_FMT " returned=%d " RX_FRAME_INFO_PR_FMT,
		  LOCAL_PR_ARG, __entry->ret, RX_FRAME_INFO_PR_ARG)
	);

TRACE_EVENT(llhw_return_timestamp_dtu,
	TP_PROTO(const struct mcps802154_local *local, int ret,
		 u32 timestamp_dtu),
	TP_ARGS(local, ret, timestamp_dtu),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(int, ret)
		__field(u32, timestamp_dtu)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->ret = ret;
		__entry->timestamp_dtu = timestamp_dtu;
		),
	TP_printk(LOCAL_PR_FMT " returned=%d timestamp_dtu=%#08x",
		  LOCAL_PR_ARG, __entry->ret, __entry->timestamp_dtu)
	);

TRACE_EVENT(llhw_return_timestamp_rctu,
	TP_PROTO(const struct mcps802154_local *local, int ret,
		 u64 timestamp_rctu),
	TP_ARGS(local, ret, timestamp_rctu),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(int, ret)
		__field(u64, timestamp_rctu)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->ret = ret;
		__entry->timestamp_rctu = timestamp_rctu;
		),
	TP_printk(LOCAL_PR_FMT " returned=%d timestamp_rctu=%#llx",
		  LOCAL_PR_ARG, __entry->ret, __entry->timestamp_rctu)
	);

DEFINE_EVENT(local_only_evt, llhw_start,
	TP_PROTO(const struct mcps802154_local *local),
	TP_ARGS(local)
	);

DEFINE_EVENT(local_only_evt, llhw_stop,
	TP_PROTO(const struct mcps802154_local *local),
	TP_ARGS(local)
	);

TRACE_EVENT(llhw_tx_frame,
	TP_PROTO(const struct mcps802154_local *local,
		 const struct mcps802154_tx_frame_info *info,
		 int frame_idx, int next_delay_dtu),
	TP_ARGS(local, info, frame_idx, next_delay_dtu),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(u32, timestamp_dtu)
		__field(int, rx_enable_after_tx_dtu)
		__field(int, rx_enable_after_tx_timeout_dtu)
		__field(int, ant_id)
		__field(u8, flags)
		__field(int, frame_idx)
		__field(int, next_delay_dtu)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->timestamp_dtu = info->timestamp_dtu;
		__entry->rx_enable_after_tx_dtu = info->rx_enable_after_tx_dtu;
		__entry->rx_enable_after_tx_timeout_dtu = info->rx_enable_after_tx_timeout_dtu;
		__entry->ant_id = info->ant_id;
		__entry->flags = info->flags;
		__entry->frame_idx = frame_idx;
		__entry->next_delay_dtu = next_delay_dtu;
		),
	TP_printk(LOCAL_PR_FMT " timestamp_dtu=%#08x rx_enable_after_tx_dtu=%d rx_enable_after_tx_timeout_dtu=%d"
		  " ant_id=%d flags=%s frame_idx=%d next_delay_dtu=%d", LOCAL_PR_ARG,
		  __entry->timestamp_dtu, __entry->rx_enable_after_tx_dtu,
		  __entry->rx_enable_after_tx_timeout_dtu, __entry->ant_id,
		  __print_flags(__entry->flags, "|",
			{ MCPS802154_TX_FRAME_TIMESTAMP_DTU, "TIMESTAMP_DTU" },
			{ MCPS802154_TX_FRAME_CCA, "CCA" },
			{ MCPS802154_TX_FRAME_RANGING, "RANGING" },
			{ MCPS802154_TX_FRAME_KEEP_RANGING_CLOCK, "KEEP_RANGING_CLOCK" },
			{ MCPS802154_TX_FRAME_RANGING_PDOA, "RANGING_PDOA" },
			{ MCPS802154_TX_FRAME_SP3, "SP3" },
			{ MCPS802154_TX_FRAME_SP2, "SP2" },
			{ MCPS802154_TX_FRAME_SP1, "SP1" }),
		  __entry->frame_idx,
		  __entry->next_delay_dtu
		  )
	);

TRACE_EVENT(llhw_rx_enable,
	TP_PROTO(const struct mcps802154_local *local,
		 const struct mcps802154_rx_info *info,
		 int frame_idx, int next_delay_dtu),
	TP_ARGS(local, info, frame_idx, next_delay_dtu),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(u32, timestamp_dtu)
		__field(int, timeout_dtu)
		__field(u8, flags)
		__field(u8, ant_pair_id)
		__field(int, frame_idx)
		__field(int, next_delay_dtu)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->timestamp_dtu = info->timestamp_dtu;
		__entry->timeout_dtu = info->timeout_dtu;
		__entry->flags = info->flags;
		__entry->ant_pair_id = info->ant_pair_id;
		__entry->frame_idx = frame_idx;
		__entry->next_delay_dtu = next_delay_dtu;
		),
	TP_printk(LOCAL_PR_FMT " timestamp_dtu=%#08x timeout_dtu=%d ant_pair_id=%d"
		  " flags=%s frame_idx=%d next_delay_dtu=%d",
		  LOCAL_PR_ARG,
		  __entry->timestamp_dtu, __entry->timeout_dtu,
		  __entry->ant_pair_id,
		  __print_flags(__entry->flags, "|",
			{ MCPS802154_RX_INFO_TIMESTAMP_DTU, "TIMESTAMP_DTU" },
			{ MCPS802154_RX_INFO_AACK, "AACK" },
			{ MCPS802154_RX_INFO_RANGING, "RANGING" },
			{ MCPS802154_RX_INFO_KEEP_RANGING_CLOCK, "KEEP_RANGING_CLOCK" },
			{ MCPS802154_RX_INFO_RANGING_PDOA, "RANGING_PDOA" },
			{ MCPS802154_RX_INFO_SP3, "SP3" },
			{ MCPS802154_RX_INFO_SP2, "SP2" },
			{ MCPS802154_RX_INFO_SP1, "SP1" }),
		  __entry->frame_idx,
		  __entry->next_delay_dtu
		  )
	);

DEFINE_EVENT(local_only_evt, llhw_rx_disable,
	TP_PROTO(const struct mcps802154_local *local),
	TP_ARGS(local)
	);

DECLARE_EVENT_CLASS(rx_frame_info_evt,
	TP_PROTO(const struct mcps802154_local *local,
		 const struct mcps802154_rx_frame_info *info),
	TP_ARGS(local, info),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		RX_FRAME_INFO_FLAGS_ENTRY
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		RX_FRAME_INFO_FLAGS_ASSIGN;
		),
	TP_printk(LOCAL_PR_FMT " " RX_FRAME_INFO_FLAGS_PR_FMT, LOCAL_PR_ARG,
		  RX_FRAME_INFO_FLAGS_PR_ARG)
	);

DEFINE_EVENT(rx_frame_info_evt, llhw_rx_get_frame,
	TP_PROTO(const struct mcps802154_local *local,
		 const struct mcps802154_rx_frame_info *info),
	TP_ARGS(local, info)
	);

DEFINE_EVENT(rx_frame_info_evt, llhw_rx_get_error_frame,
	TP_PROTO(const struct mcps802154_local *local,
		 const struct mcps802154_rx_frame_info *info),
	TP_ARGS(local, info)
	);

DEFINE_EVENT(local_only_evt, llhw_idle,
	TP_PROTO(const struct mcps802154_local *local),
	TP_ARGS(local)
	);

TRACE_EVENT(llhw_idle_timestamp,
	TP_PROTO(const struct mcps802154_local *local,
		 u32 timestamp_dtu),
	TP_ARGS(local, timestamp_dtu),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(u32, timestamp_dtu)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->timestamp_dtu = timestamp_dtu;
		),
	TP_printk(LOCAL_PR_FMT " timestamp_dtu=%#08x",
		  LOCAL_PR_ARG,
		  __entry->timestamp_dtu
		  )
	);

DEFINE_EVENT(local_only_evt, llhw_reset,
	TP_PROTO(const struct mcps802154_local *local),
	TP_ARGS(local)
	);

DEFINE_EVENT(local_only_evt, llhw_get_current_timestamp_dtu,
	TP_PROTO(const struct mcps802154_local *local),
	TP_ARGS(local)
	);

TRACE_EVENT(llhw_set_channel,
	TP_PROTO(const struct mcps802154_local *local, u8 page, u8 channel,
		 u8 preamble_code),
	TP_ARGS(local, page, channel, preamble_code),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(u8, page)
		__field(u8, channel)
		__field(u8, preamble_code)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->page = page;
		__entry->channel = channel;
		__entry->preamble_code = preamble_code;
		),
	TP_printk(LOCAL_PR_FMT " page=%d channel=%d preamble_code=%d",
		  LOCAL_PR_ARG, __entry->page, __entry->channel,
		  __entry->preamble_code)
	);

TRACE_EVENT(llhw_set_hrp_uwb_params,
	TP_PROTO(const struct mcps802154_local *local, int prf, int psr,
		 int sfd_selector, int phr_rate, int data_rate),
	TP_ARGS(local, prf, psr, sfd_selector, phr_rate, data_rate),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(int, prf)
		__field(int, psr)
		__field(int, sfd_selector)
		__field(int, phr_rate)
		__field(int, data_rate)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->prf = prf;
		__entry->psr = psr;
		__entry->sfd_selector = sfd_selector;
		__entry->phr_rate = phr_rate;
		__entry->data_rate = data_rate;
		),
	TP_printk(LOCAL_PR_FMT " prf=%d psr=%d sfd_selector=%d phr_rate=%d data_rate=%d",
		  LOCAL_PR_ARG, __entry->prf, __entry->psr,
		  __entry->sfd_selector, __entry->phr_rate, __entry->data_rate)
	);

TRACE_EVENT(llhw_set_sts_params,
	TP_PROTO(const struct mcps802154_local *local, const struct
		 mcps802154_sts_params *params),
	TP_ARGS(local, params),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(int, n_segs)
		__field(int, seg_len)
		__field(int, sp2_tx_gap_4chips)
		__array(int, sp2_rx_gap_4chips, MCPS802154_STS_N_SEGS_MAX)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->n_segs = params->n_segs;
		__entry->seg_len = params->seg_len;
		__entry->sp2_tx_gap_4chips = params->sp2_tx_gap_4chips;
		memcpy(__entry->sp2_rx_gap_4chips, params->sp2_rx_gap_4chips,
		       sizeof(params->sp2_rx_gap_4chips));
		),
	TP_printk(LOCAL_PR_FMT " n_segs=%d seg_len=%d sp2_tx_gap_4chips=%d"
		  " sp2_rx_gap_4chips=%d,%d,%d,%d",
		  LOCAL_PR_ARG, __entry->n_segs, __entry->seg_len,
		  __entry->sp2_tx_gap_4chips, __entry->sp2_rx_gap_4chips[0],
		  __entry->sp2_rx_gap_4chips[1], __entry->sp2_rx_gap_4chips[2],
		  __entry->sp2_rx_gap_4chips[3])
	);

TRACE_EVENT(llhw_set_hw_addr_filt,
	TP_PROTO(const struct mcps802154_local *local,
		 const struct ieee802154_hw_addr_filt *filt,
		 unsigned long changed),
	TP_ARGS(local, filt, changed),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(__le16, pan_id)
		__field(__le16, short_addr)
		__field(__le64, extended_addr)
		__field(bool, pan_coord)
		__field(unsigned long, changed)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->pan_id = filt->pan_id;
		__entry->short_addr = filt->short_addr;
		__entry->extended_addr = filt->ieee_addr;
		__entry->pan_coord = filt->pan_coord;
		__entry->changed = changed;
		),
	TP_printk(LOCAL_PR_FMT " pan_id=%#04x short_addr=%#04x extended_addr=%#016llx pan_coord=%s changed=%s",
		  LOCAL_PR_ARG, __entry->pan_id, __entry->short_addr,
		  __entry->extended_addr,
		  __entry->pan_coord ? "true" : "false",
		  __print_flags(__entry->changed, "|",
			{ IEEE802154_AFILT_SADDR_CHANGED, "SHORT_ADDR" },
			{ IEEE802154_AFILT_IEEEADDR_CHANGED, "EXTENDED_ADDR" },
			{ IEEE802154_AFILT_PANID_CHANGED, "PAN_ID" },
			{ IEEE802154_AFILT_PANC_CHANGED, "PAN_COORD" })
		 )
	);

TRACE_EVENT(llhw_set_txpower,
	TP_PROTO(const struct mcps802154_local *local, s32 mbm),
	TP_ARGS(local, mbm),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(s32, mbm)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->mbm = mbm;
		),
	TP_printk(LOCAL_PR_FMT " mbm=%d", LOCAL_PR_ARG, __entry->mbm)
	);

TRACE_EVENT(llhw_set_cca_mode,
	TP_PROTO(const struct mcps802154_local *local,
		 const struct wpan_phy_cca *cca),
	TP_ARGS(local, cca),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(enum nl802154_cca_modes, cca_mode)
		__field(enum nl802154_cca_opts, cca_opt)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->cca_mode = cca->mode;
		__entry->cca_opt = cca->opt;
		),
	TP_printk(LOCAL_PR_FMT " cca_mode=%d cca_opt=%d", LOCAL_PR_ARG,
		  __entry->cca_mode, __entry->cca_opt)
	);

TRACE_EVENT(llhw_set_cca_ed_level,
	TP_PROTO(const struct mcps802154_local *local, s32 mbm),
	TP_ARGS(local, mbm),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(s32, mbm)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->mbm = mbm;
		),
	TP_printk(LOCAL_PR_FMT " mbm=%d", LOCAL_PR_ARG, __entry->mbm)
	);

DECLARE_EVENT_CLASS(bool_on_evt,
	TP_PROTO(const struct mcps802154_local *local, bool on),
	TP_ARGS(local, on),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(bool, on)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->on = on;
		),
	TP_printk(LOCAL_PR_FMT " %s", LOCAL_PR_ARG, __entry->on ? "on" : "off")
	);

DEFINE_EVENT(bool_on_evt, llhw_set_promiscuous_mode,
	TP_PROTO(const struct mcps802154_local *local, bool on),
	TP_ARGS(local, on)
	);

DEFINE_EVENT(bool_on_evt, llhw_set_scanning_mode,
	TP_PROTO(const struct mcps802154_local *local, bool on),
	TP_ARGS(local, on)
	);

DEFINE_EVENT(local_only_evt, llhw_testmode_cmd,
	TP_PROTO(const struct mcps802154_local *local),
	TP_ARGS(local)
	);

DEFINE_EVENT(local_only_evt, llhw_event_rx_frame,
	TP_PROTO(const struct mcps802154_local *local),
	TP_ARGS(local)
	);

DEFINE_EVENT(local_only_evt, llhw_event_rx_timeout,
	TP_PROTO(const struct mcps802154_local *local),
	TP_ARGS(local)
	);

DECLARE_EVENT_CLASS(str_key_evt,
	TP_PROTO(const struct mcps802154_local *local, const char *const key),
	TP_ARGS(local, key),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		KEY_ENTRY
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		KEY_ASSIGN;
		),
	TP_printk(LOCAL_PR_FMT " " KEY_PR_FMT, LOCAL_PR_ARG, KEY_PR_ARG)
	);

DEFINE_EVENT(str_key_evt, llhw_set_calibration,
	TP_PROTO(const struct mcps802154_local *local, const char *const key),
	TP_ARGS(local, key)
	);

DEFINE_EVENT(str_key_evt, llhw_get_calibration,
	TP_PROTO(const struct mcps802154_local *local, const char *const key),
	TP_ARGS(local, key)
	);

DEFINE_EVENT(local_only_evt, llhw_list_calibration,
	TP_PROTO(const struct mcps802154_local *local),
	TP_ARGS(local)
	);

TRACE_EVENT(llhw_vendor_cmd,
	TP_PROTO(const struct mcps802154_local *local, u32 vendor_id,
		 u32 subcmd, size_t data_len),
	TP_ARGS(local, vendor_id, subcmd, data_len),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(u32, vendor_id)
		__field(u32, subcmd)
		__field(u32, data_len)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->vendor_id = vendor_id;
		__entry->subcmd = subcmd;
		__entry->data_len = data_len;
		),
	TP_printk(LOCAL_PR_FMT " vendor_id=0x%06x subcmd=0x%x data_len=%d",
		  LOCAL_PR_ARG, __entry->vendor_id, __entry->subcmd,
		  __entry->data_len)
	);

TRACE_EVENT(llhw_event_rx_error,
	TP_PROTO(const struct mcps802154_local *local,
		 enum mcps802154_rx_error_type error),
	TP_ARGS(local, error),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(enum mcps802154_rx_error_type, error)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->error = error;
		),
	TP_printk(LOCAL_PR_FMT " error=%s", LOCAL_PR_ARG,
		  __print_symbolic(__entry->error,
			{ MCPS802154_RX_ERROR_BAD_CKSUM, "BAD_CKSUM" },
			{ MCPS802154_RX_ERROR_UNCORRECTABLE, "UNCORRECTABLE" },
			{ MCPS802154_RX_ERROR_FILTERED, "FILTERED" },
			{ MCPS802154_RX_ERROR_SFD_TIMEOUT, "SFD_TIMEOUT" },
			{ MCPS802154_RX_ERROR_OTHER, "OTHER" }))
	);

DEFINE_EVENT(local_only_evt, llhw_event_tx_done,
	TP_PROTO(const struct mcps802154_local *local),
	TP_ARGS(local)
	);

DEFINE_EVENT(local_only_evt, llhw_event_broken,
	TP_PROTO(const struct mcps802154_local *local),
	TP_ARGS(local)
	);

DEFINE_EVENT(local_only_evt, llhw_event_timer_expired,
	TP_PROTO(const struct mcps802154_local *local),
	TP_ARGS(local)
	);

DEFINE_EVENT(local_only_evt, llhw_event_done,
	TP_PROTO(const struct mcps802154_local *local),
	TP_ARGS(local)
	);

TRACE_EVENT(ca_set_scheduler,
	TP_PROTO(const struct mcps802154_local *local, const char *name),
	TP_ARGS(local, name),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__string(name, name)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__assign_str(name, name);
		),
	TP_printk(LOCAL_PR_FMT " name=%s", LOCAL_PR_ARG, __get_str(name))
	);

TRACE_EVENT(ca_set_scheduler_parameters,
	TP_PROTO(const struct mcps802154_local *local, const char *name),
	TP_ARGS(local, name),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__string(name, name)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__assign_str(name, name);
		),
	TP_printk(LOCAL_PR_FMT " name=%s", LOCAL_PR_ARG, __get_str(name))
	);

TRACE_EVENT(ca_scheduler_set_region_parameters,
	TP_PROTO(const struct mcps802154_local *local,
		 const char *scheduler_name, u32 region_id,
		 const char *region_name),
	TP_ARGS(local, scheduler_name, region_id, region_name),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__string(scheduler_name, scheduler_name)
		__field(u32, region_id)
		__string(region_name, region_name)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__assign_str(scheduler_name, scheduler_name);
		__entry->region_id = region_id;
		__assign_str(region_name, region_name);
		),
	TP_printk(LOCAL_PR_FMT " scheduler=%s region_id=%u region_name=%s",
		  LOCAL_PR_ARG, __get_str(scheduler_name), __entry->region_id,
		  __get_str(region_name))
	);

TRACE_EVENT(ca_scheduler_call,
	TP_PROTO(const struct mcps802154_local *local,
		const char *scheduler_name, u32 call_id),
	TP_ARGS(local, scheduler_name, call_id),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__string(scheduler_name, scheduler_name)
		__field(u32, call_id)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__assign_str(scheduler_name, scheduler_name);
		__entry->call_id = call_id;
		),
	TP_printk(LOCAL_PR_FMT" scheduler=%s call_id=%#x",
		  LOCAL_PR_ARG, __get_str(scheduler_name), __entry->call_id)
	);

TRACE_EVENT(ca_scheduler_call_region,
	TP_PROTO(const struct mcps802154_local *local,
		const char *scheduler_name, u32 region_id,
		const char *region_name, u32 call_id),
	TP_ARGS(local, scheduler_name, region_id, region_name, call_id),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__string(scheduler_name, scheduler_name)
		__field(u32, region_id)
		__string(region_name, region_name)
		__field(u32, call_id)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__assign_str(scheduler_name, scheduler_name);
		__entry->region_id = region_id;
		__assign_str(region_name, region_name);
		__entry->call_id = call_id;
		),
	TP_printk(LOCAL_PR_FMT " scheduler=%s region_id=%u region_name=%s call_id=%#x",
		  LOCAL_PR_ARG, __get_str(scheduler_name), __entry->region_id,
		  __get_str(region_name), __entry->call_id)
	);

TRACE_EVENT(ca_get_access,
	TP_PROTO(const struct mcps802154_local *local, u32 next_timestamp_dtu),
	TP_ARGS(local, next_timestamp_dtu),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(u32, next_timestamp_dtu)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->next_timestamp_dtu = next_timestamp_dtu;
		),
	TP_printk(LOCAL_PR_FMT " next_timestamp_dtu=%#08x", LOCAL_PR_ARG,
		  __entry->next_timestamp_dtu)
	);

TRACE_EVENT(ca_return_int,
	TP_PROTO(const struct mcps802154_local *local, int r),
	TP_ARGS(local, r),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(int, r)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->r = r;
		),
	TP_printk(LOCAL_PR_FMT " r=%d", LOCAL_PR_ARG, __entry->r)
	);

TRACE_EVENT(schedule_update,
	TP_PROTO(const struct mcps802154_local *local, u32 next_timestamp_dtu),
	TP_ARGS(local, next_timestamp_dtu),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(u32, next_timestamp_dtu)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->next_timestamp_dtu = next_timestamp_dtu;
		),
	TP_printk(LOCAL_PR_FMT " next_timestamp_dtu=%#08x", LOCAL_PR_ARG,
		  __entry->next_timestamp_dtu)
	);

TRACE_EVENT(schedule_update_done,
	TP_PROTO(const struct mcps802154_local *local,
		 const struct mcps802154_schedule *sched),
	TP_ARGS(local, sched),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(u32, start_timestamp_dtu)
		__field(int, duration_dtu)
		__field(size_t, n_regions)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->start_timestamp_dtu = sched->start_timestamp_dtu;
		__entry->duration_dtu = sched->duration_dtu;
		__entry->n_regions = sched->n_regions;
		),
	TP_printk(LOCAL_PR_FMT " start_timestamp_dtu=%#08x duration_dtu=%d n_regions=%lu",
		  LOCAL_PR_ARG, __entry->start_timestamp_dtu,
		  __entry->duration_dtu, __entry->n_regions)
	);

TRACE_EVENT(region_get_access,
	TP_PROTO(const struct mcps802154_local *local,
		 const struct mcps802154_region *region,
		 u32 next_timestamp_dtu, int next_in_region_dtu,
		 int region_duration_dtu),
	TP_ARGS(local, region, next_timestamp_dtu, next_in_region_dtu,
		region_duration_dtu),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__string(region_name, region->ops->name)
		__field(u32, next_timestamp_dtu)
		__field(int, next_in_region_dtu)
		__field(int, region_duration_dtu)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__assign_str(region_name, region->ops->name);
		__entry->next_timestamp_dtu = next_timestamp_dtu;
		__entry->next_in_region_dtu = next_in_region_dtu;
		__entry->region_duration_dtu = region_duration_dtu;
		),
	TP_printk(LOCAL_PR_FMT " region=%s next_timestamp_dtu=%#08x next_in_region_dtu=%d region_duration_dtu=%d",
		  LOCAL_PR_ARG,
		  __get_str(region_name), __entry->next_timestamp_dtu,
		  __entry->next_in_region_dtu, __entry->region_duration_dtu)
	);

#endif /* !TRACE_H || TRACE_HEADER_MULTI_READ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace
#include <trace/define_trace.h>
