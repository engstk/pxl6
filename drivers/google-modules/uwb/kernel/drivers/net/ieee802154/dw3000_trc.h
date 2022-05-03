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
#define TRACE_SYSTEM dw3000

#if !defined(__DW3000_TRACE) || defined(TRACE_HEADER_MULTI_READ)
#define __DW3000_TRACE

#include <linux/tracepoint.h>

#include "dw3000.h"
#include "dw3000_nfcc_coex.h"
#include "dw3000_testmode_nl.h"
#include "dw3000_core_reg.h"
#include "dw3000_core.h"

#define MAXNAME 32
#define DW_ENTRY __array(char, dw_name, MAXNAME)
#define DW_ASSIGN strlcpy(__entry->dw_name, dw->dev->kobj.name, MAXNAME)
#define DW_PR_FMT "%s"
#define DW_PR_ARG __entry->dw_name

#ifdef CONFIG_MCPS802154_TESTMODE
#define DW_TM_CMD_ENTRY __field(u32, cmd)
#define DW_TM_CMD_ASSIGN __entry->cmd = cmd
#define DW_TM_CMD_PR_FMT "cmd: %s"
#define dw3000_tm_cmd_name(name)            \
	{                                   \
		DW3000_TM_CMD_##name, #name \
	}
/* clang-format off */
#define DW_TM_CMD_SYMBOLS                    \
	dw3000_tm_cmd_name(START_RX_DIAG),   \
	dw3000_tm_cmd_name(STOP_RX_DIAG),    \
	dw3000_tm_cmd_name(GET_RX_DIAG),     \
	dw3000_tm_cmd_name(CLEAR_RX_DIAG),   \
	dw3000_tm_cmd_name(START_TX_CWTONE), \
	dw3000_tm_cmd_name(STOP_TX_CWTONE),  \
	dw3000_tm_cmd_name(OTP_READ),        \
	dw3000_tm_cmd_name(OTP_WRITE)
/* clang-format on */
TRACE_DEFINE_ENUM(DW3000_TM_CMD_START_RX_DIAG);
TRACE_DEFINE_ENUM(DW3000_TM_CMD_STOP_RX_DIAG);
TRACE_DEFINE_ENUM(DW3000_TM_CMD_GET_RX_DIAG);
TRACE_DEFINE_ENUM(DW3000_TM_CMD_CLEAR_RX_DIAG);
TRACE_DEFINE_ENUM(DW3000_TM_CMD_START_TX_CWTONE);
TRACE_DEFINE_ENUM(DW3000_TM_CMD_STOP_TX_CWTONE);
TRACE_DEFINE_ENUM(DW3000_TM_CMD_OTP_READ);
TRACE_DEFINE_ENUM(DW3000_TM_CMD_OTP_WRITE);
#define DW_TM_CMD_PR_ARG __print_symbolic(__entry->cmd, DW_TM_CMD_SYMBOLS)
#endif

#define DW_PWR_ENTRY __field(int, state)
#define DW_PWR_ASSIGN __entry->state = state
#define DW_PWR_PR_FMT "state: %s"

TRACE_DEFINE_ENUM(DW3000_OP_STATE_OFF);
TRACE_DEFINE_ENUM(DW3000_OP_STATE_DEEP_SLEEP);
TRACE_DEFINE_ENUM(DW3000_OP_STATE_SLEEP);
TRACE_DEFINE_ENUM(DW3000_OP_STATE_WAKE_UP);
TRACE_DEFINE_ENUM(DW3000_OP_STATE_INIT_RC);
TRACE_DEFINE_ENUM(DW3000_OP_STATE_IDLE_RC);
TRACE_DEFINE_ENUM(DW3000_OP_STATE_IDLE_PLL);
TRACE_DEFINE_ENUM(DW3000_OP_STATE_TX_WAIT);
TRACE_DEFINE_ENUM(DW3000_OP_STATE_TX);
TRACE_DEFINE_ENUM(DW3000_OP_STATE_RX_WAIT);
TRACE_DEFINE_ENUM(DW3000_OP_STATE_RX);

#define dw3000_op_state_name(name)            \
	{                                     \
		DW3000_OP_STATE_##name, #name \
	}

/* clang-format off */
#define DW3000_OP_STATE_FLAGS             \
	dw3000_op_state_name(OFF),        \
	dw3000_op_state_name(DEEP_SLEEP), \
	dw3000_op_state_name(SLEEP),      \
	dw3000_op_state_name(WAKE_UP),    \
	dw3000_op_state_name(INIT_RC),    \
	dw3000_op_state_name(IDLE_RC),    \
	dw3000_op_state_name(IDLE_PLL),   \
	dw3000_op_state_name(TX_WAIT),    \
	dw3000_op_state_name(TX),         \
	dw3000_op_state_name(RX_WAIT),    \
	dw3000_op_state_name(RX)
/* clang-format on */

#define dw3000_pwr_name(name)            \
	{                                \
		DW3000_PWR_##name, #name \
	}
/* clang-format off */
#define DW_PWR_SYMBOLS          \
	dw3000_pwr_name(OFF),       \
	dw3000_pwr_name(DEEPSLEEP), \
	dw3000_pwr_name(RUN),       \
	dw3000_pwr_name(IDLE),      \
	dw3000_pwr_name(RX),        \
	dw3000_pwr_name(TX)
/* clang-format on */
TRACE_DEFINE_ENUM(DW3000_PWR_OFF);
TRACE_DEFINE_ENUM(DW3000_PWR_DEEPSLEEP);
TRACE_DEFINE_ENUM(DW3000_PWR_RUN);
TRACE_DEFINE_ENUM(DW3000_PWR_IDLE);
TRACE_DEFINE_ENUM(DW3000_PWR_RX);
TRACE_DEFINE_ENUM(DW3000_PWR_TX);
#define DW_PWR_PR_ARG __print_symbolic(__entry->state, DW_PWR_SYMBOLS)

#define dw3000_sys_status_name(name)                       \
	{                                                  \
		DW3000_SYS_STATUS_##name##_BIT_MASK, #name \
	}

#define DW3000_SYS_STATUS_FLAGS_ENTRY __field(u64, status)
#define DW3000_SYS_STATUS_FLAGS_ASSIGN __entry->status = status
#define DW3000_SYS_STATUS_FLAGS_PR_FMT "status: %s"
/* clang-format off */
#define DW3000_SYS_STATUS_FLAGS                                                \
	dw3000_sys_status_name(TIMER1),                                        \
	dw3000_sys_status_name(TIMER0),                                        \
	dw3000_sys_status_name(ARFE),                                          \
	dw3000_sys_status_name(CPERR),                                         \
	dw3000_sys_status_name(HPDWARN),                                       \
	dw3000_sys_status_name(RXSTO),                                         \
	dw3000_sys_status_name(PLL_HILO),                                      \
	dw3000_sys_status_name(RCINIT),                                        \
	dw3000_sys_status_name(SPIRDY),                                        \
	dw3000_sys_status_name(LCSSERR),                                       \
	dw3000_sys_status_name(RXPTO),                                         \
	dw3000_sys_status_name(RXOVRR),                                        \
	dw3000_sys_status_name(VWARN),                                         \
	dw3000_sys_status_name(CIAERR),                                        \
	dw3000_sys_status_name(RXFTO),                                         \
	dw3000_sys_status_name(RXFSL),                                         \
	dw3000_sys_status_name(RXFCE),                                         \
	dw3000_sys_status_name(RXFCG),                                         \
	dw3000_sys_status_name(RXFR),                                          \
	dw3000_sys_status_name(RXPHE),                                         \
	dw3000_sys_status_name(RXPHD),                                         \
	dw3000_sys_status_name(CIA_DONE),                                      \
	dw3000_sys_status_name(RXSFDD),                                        \
	dw3000_sys_status_name(RXPRD),                                         \
	dw3000_sys_status_name(TXFRS),                                         \
	dw3000_sys_status_name(TXPHS),                                         \
	dw3000_sys_status_name(TXPRS),                                         \
	dw3000_sys_status_name(TXFRB),                                         \
	dw3000_sys_status_name(AAT),                                           \
	dw3000_sys_status_name(SPICRCERR),                                     \
	dw3000_sys_status_name(CLK_PLL_LOCK),                                  \
	dw3000_sys_status_name(IRQS)
/* clang-format on */
#define DW3000_SYS_STATUS_FLAGS_PR_ARG \
	__print_flags(__entry->status, "|", DW3000_SYS_STATUS_FLAGS)

#define RX_INFO_FLAGS_ENTRY __field(u8, flags)
#define RX_INFO_FLAGS_ASSIGN entry->flags = flags
#define RX_INFO_FLAGS_PR_FMT "flags: %s"

#define mcps802154_rx_info_name(name)            \
	{                                        \
		MCPS802154_RX_INFO_##name, #name \
	}

/* clang-format off */
#define RX_INFO_FLAGS                                \
	mcps802154_rx_info_name(TIMESTAMP_DTU),      \
	mcps802154_rx_info_name(AACK),               \
	mcps802154_rx_info_name(RANGING),            \
	mcps802154_rx_info_name(KEEP_RANGING_CLOCK), \
	mcps802154_rx_info_name(RANGING_PDOA),       \
	mcps802154_rx_info_name(SP3),                \
	mcps802154_rx_info_name(SP2),                \
	mcps802154_rx_info_name(SP1),                \
	mcps802154_rx_info_name(STS_MODE_MASK)
/* clang-format on */

#define RX_INFO_FLAGS_PR_ARG __print_flags(__entry->flags, "|", RX_INFO_FLAGS)

#define RX_FRAME_INFO_FLAGS_ENTRY __field(u16, flags)
#define RX_FRAME_INFO_FLAGS_ASSIGN entry->flags = flags
#define RX_FRAME_INFO_FLAGS_PR_FMT "flags: %s"

#define mcps802154_rx_frame_info_name(name)            \
	{                                              \
		MCPS802154_RX_FRAME_INFO_##name, #name \
	}

/* clang-format off */
#define RX_FRAME_INFO_FLAGS						\
	mcps802154_rx_frame_info_name(TIMESTAMP_DTU),			\
	mcps802154_rx_frame_info_name(TIMESTAMP_RCTU),			\
	mcps802154_rx_frame_info_name(LQI),				\
	mcps802154_rx_frame_info_name(RSSI),				\
	mcps802154_rx_frame_info_name(RANGING_FOM),			\
	mcps802154_rx_frame_info_name(RANGING_OFFSET),			\
	mcps802154_rx_frame_info_name(RANGING_PDOA),			\
	mcps802154_rx_frame_info_name(RANGING_PDOA_FOM),		\
	mcps802154_rx_frame_info_name(RANGING_STS_TIMESTAMP_RCTU),	\
	mcps802154_rx_frame_info_name(RANGING_STS_FOM),			\
	mcps802154_rx_frame_info_name(AACK)
/* clang-format on */

#define RX_FRAME_INFO_FLAGS_PR_ARG \
	__print_flags(__entry->flags, "|", RX_FRAME_INFO_FLAGS)

#define TX_FRAME_INFO_FLAGS_ENTRY __field(u8, flags)
#define TX_FRAME_INFO_FLAGS_ASSIGN entry->flags = flags
#define TX_FRAME_INFO_FLAGS_PR_FMT "flags: %s"

#define mcps802154_tx_frame_info_name(name)       \
	{                                         \
		MCPS802154_TX_FRAME_##name, #name \
	}

/* clang-format off */
#define TX_FRAME_INFO_FLAGS						\
	mcps802154_tx_frame_info_name(TIMESTAMP_DTU),			\
	mcps802154_tx_frame_info_name(CCA),				\
	mcps802154_tx_frame_info_name(RANGING),				\
	mcps802154_tx_frame_info_name(KEEP_RANGING_CLOCK),		\
	mcps802154_tx_frame_info_name(SP3),				\
	mcps802154_tx_frame_info_name(SP2),				\
	mcps802154_tx_frame_info_name(SP1),				\
	mcps802154_tx_frame_info_name(STS_MODE_MASK)
/* clang-format on */

#define TX_FRAME_INFO_FLAGS_PR_ARG \
	__print_flags(__entry->flags, "|", TX_FRAME_INFO_FLAGS)

/* We don't want clang-format to modify the following events definition!
   Look at net/wireless/trace.h for the required format. */
/* clang-format off */

/*************************************************************
 *		dw3000 functions return traces		     *
 *************************************************************/

DECLARE_EVENT_CLASS(dw_only_evt,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw),
	TP_STRUCT__entry(
		DW_ENTRY
	),
	TP_fast_assign(
		DW_ASSIGN;
	),
	TP_printk(DW_PR_FMT, DW_PR_ARG)
);

DEFINE_EVENT(dw_only_evt, dw3000_return_void,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw)
);

TRACE_EVENT(dw3000_return_int,
	TP_PROTO(struct dw3000 *dw, int ret),
	TP_ARGS(dw, ret),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(int, ret)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->ret = ret;
	),
	TP_printk(DW_PR_FMT ", return %d", DW_PR_ARG, __entry->ret)
);

TRACE_EVENT(dw3000_return_int_u32,
	TP_PROTO(struct dw3000 *dw, int ret, u32 val),
	TP_ARGS(dw, ret, val),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(int, ret)
		__field(u32, val)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->ret = ret;
		__entry->val = val;
	),
	TP_printk(DW_PR_FMT ", return %d, value %#x", DW_PR_ARG,
		  __entry->ret, __entry->val)
);

TRACE_EVENT(dw3000_return_int_u64,
	TP_PROTO(struct dw3000 *dw, int ret, u64 val),
	TP_ARGS(dw, ret, val),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(int, ret)
		__field(u64, val)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->ret = ret;
		__entry->val = val;
	),
	TP_printk(DW_PR_FMT ", return %d, value %#llx", DW_PR_ARG,
		  __entry->ret, __entry->val)
);

/*************************************************************
 *		dw3000 mcps functions traces		     *
 *************************************************************/

DEFINE_EVENT(dw_only_evt, dw3000_mcps_start,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw)
);

DEFINE_EVENT(dw_only_evt, dw3000_mcps_stop,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw)
);

TRACE_EVENT(dw3000_mcps_tx_frame,
	TP_PROTO(struct dw3000 *dw, u8 flags, u16 len),
	TP_ARGS(dw, flags, len),
	TP_STRUCT__entry(
		DW_ENTRY
		TX_FRAME_INFO_FLAGS_ENTRY
		__field(u16, len)
	),
	TP_fast_assign(
		DW_ASSIGN;
		TX_FRAME_INFO_FLAGS_ASSIGN;
		__entry->len = len;
	),
	TP_printk(DW_PR_FMT ", " TX_FRAME_INFO_FLAGS_PR_FMT ", skb->len: %d",
		  DW_PR_ARG, TX_FRAME_INFO_FLAGS_PR_ARG,__entry->len)
);

TRACE_EVENT(dw3000_mcps_rx_enable,
	TP_PROTO(struct dw3000 *dw, u8 flags, int timeout),
	TP_ARGS(dw, flags, timeout),
	TP_STRUCT__entry(
		DW_ENTRY
		RX_INFO_FLAGS_ENTRY
		__field(int, timeout)
	),
	TP_fast_assign(
		DW_ASSIGN;
		RX_INFO_FLAGS_ASSIGN;
		__entry->timeout = timeout;
	),
	TP_printk(DW_PR_FMT ", " RX_INFO_FLAGS_PR_FMT ", timeout: %d",
		  DW_PR_ARG, RX_INFO_FLAGS_PR_ARG, __entry->timeout)
);

DEFINE_EVENT(dw_only_evt, dw3000_mcps_rx_disable,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw)
);

TRACE_EVENT(dw3000_mcps_rx_get_frame,
	TP_PROTO(struct dw3000 *dw, u16 flags),
	TP_ARGS(dw, flags),
	TP_STRUCT__entry(
		DW_ENTRY
		RX_FRAME_INFO_FLAGS_ENTRY
	),
	TP_fast_assign(
		DW_ASSIGN;
		RX_FRAME_INFO_FLAGS_ASSIGN;
	),
	TP_printk(DW_PR_FMT ", " RX_FRAME_INFO_FLAGS_PR_FMT,
		  DW_PR_ARG, RX_FRAME_INFO_FLAGS_PR_ARG)
);

TRACE_EVENT(dw3000_mcps_rx_get_error_frame,
	TP_PROTO(struct dw3000 *dw, u16 flags),
	TP_ARGS(dw, flags),
	TP_STRUCT__entry(
		DW_ENTRY
		RX_FRAME_INFO_FLAGS_ENTRY
	),
	TP_fast_assign(
		DW_ASSIGN;
		RX_FRAME_INFO_FLAGS_ASSIGN;
	),
	TP_printk(DW_PR_FMT ", " RX_FRAME_INFO_FLAGS_PR_FMT,
		  DW_PR_ARG, RX_FRAME_INFO_FLAGS_PR_ARG)
	);

TRACE_EVENT(dw3000_mcps_idle,
	TP_PROTO(struct dw3000 *dw, bool timeout, u32 timeout_dtu),
	TP_ARGS(dw, timeout, timeout_dtu),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(bool, timeout)
		__field(u32, timeout_dtu)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->timeout = timeout;
		__entry->timeout_dtu = timeout_dtu;
	),
	TP_printk(DW_PR_FMT ", timeout: %s, timeout_dtu: %#x", DW_PR_ARG,
		__entry->timeout ? "true" : "false", __entry->timeout_dtu)
);

DEFINE_EVENT(dw_only_evt, dw3000_mcps_reset,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw)
);

DEFINE_EVENT(dw_only_evt, dw3000_mcps_get_timestamp,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw)
);

TRACE_EVENT(dw3000_mcps_set_channel,
	TP_PROTO(struct dw3000 *dw, u8 page, u8 channel, u8 pcode),
	TP_ARGS(dw, page, channel, pcode),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u8, page)
		__field(u8, channel)
		__field(u8, pcode)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->page = page;
		__entry->channel = channel;
		__entry->pcode = pcode;
	),
	TP_printk(DW_PR_FMT ", page: %u, channel: %u, preamble_code: %u",
		  DW_PR_ARG, __entry->page, __entry->channel,
		  __entry->pcode)
);

TRACE_EVENT(dw3000_mcps_set_hw_addr_filt,
	TP_PROTO(struct dw3000 *dw, u8 changed),
	TP_ARGS(dw, changed),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u8, changed)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->changed = changed;
	),
	TP_printk(DW_PR_FMT ", changed: %#x", DW_PR_ARG,
		  __entry->changed)
);

TRACE_EVENT(dw3000_mcps_set_sts_params,
	TP_PROTO(struct dw3000 *dw, const struct mcps802154_sts_params *p),
	TP_ARGS(dw, p),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u8, n_segs)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->n_segs = p->n_segs;
	),
	TP_printk(DW_PR_FMT ", n_segs: %u", DW_PR_ARG,
		  __entry->n_segs)
);

/*************************************************************
 *	    dw3000 RCTU converter functions traces	     *
 *************************************************************/
TRACE_EVENT(dw3000_rctu_convert_align,
	TP_PROTO(struct dw3000 *dw, u32 rmarker_dtu),
	TP_ARGS(dw, rmarker_dtu),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u32, rmarker_dtu)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->rmarker_dtu = rmarker_dtu;
	),
	TP_printk(DW_PR_FMT ", rmarker_dtu: %#x", DW_PR_ARG,
		  __entry->rmarker_dtu)
);

TRACE_EVENT(dw3000_rctu_convert_synced,
	TP_PROTO(struct dw3000 *dw, u64 rmarker_rctu),
	TP_ARGS(dw, rmarker_rctu),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u64, rmarker_rctu)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->rmarker_rctu = rmarker_rctu;
	),
	TP_printk(DW_PR_FMT ", rmarker_rctu: %#llx", DW_PR_ARG,
		  __entry->rmarker_rctu)
);

TRACE_EVENT(dw3000_rctu_convert_rx,
	TP_PROTO(struct dw3000 *dw, u32 rmarker_dtu, u64 ts_rctu, u64 rmarker_rctu),
	TP_ARGS(dw, rmarker_dtu, ts_rctu, rmarker_rctu),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u32, rmarker_dtu)
		__field(u64, ts_rctu)
		__field(u64, rmarker_rctu)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->rmarker_dtu = rmarker_dtu;
		__entry->ts_rctu = ts_rctu;
		__entry->rmarker_rctu = rmarker_rctu;
	),
	TP_printk(DW_PR_FMT ", rmarker_dtu: %#x, ts_rctu: %#llx, "
		  "rmarker_rctu: %#llx",
		  DW_PR_ARG, __entry->rmarker_dtu, __entry->ts_rctu,
		  __entry->rmarker_rctu)
);

TRACE_EVENT(dw3000_rctu_convert_tx,
	TP_PROTO(struct dw3000 *dw, u32 ts_dtu, u32 ant_offset, u64 rmarker_rctu),
	TP_ARGS(dw, ts_dtu, ant_offset, rmarker_rctu),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u32, ts_dtu)
		__field(u32, ant_offset)
		__field(u64, rmarker_rctu)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->ts_dtu = ts_dtu;
		__entry->ant_offset = ant_offset;
		__entry->rmarker_rctu = rmarker_rctu;
	),
	TP_printk(DW_PR_FMT ", ts_dtu: %#x, ant_offset: %#x, rmarker_rctu: %#llx",
		  DW_PR_ARG, __entry->ts_dtu, __entry->ant_offset,
		  __entry->rmarker_rctu)
);

/*************************************************************
 *		dw3000 core functions traces		     *
 *************************************************************/
TRACE_EVENT(dw3000_isr,
	TP_PROTO(struct dw3000 *dw, u64 status),
	TP_ARGS(dw, status),
	TP_STRUCT__entry(
		DW_ENTRY
		DW3000_SYS_STATUS_FLAGS_ENTRY
	),
	TP_fast_assign(
		DW_ASSIGN;
		DW3000_SYS_STATUS_FLAGS_ASSIGN;
	),
	TP_printk(DW_PR_FMT ", " DW3000_SYS_STATUS_FLAGS_PR_FMT,
		  DW_PR_ARG, DW3000_SYS_STATUS_FLAGS_PR_ARG)
);

TRACE_EVENT(dw3000_set_operational_state,
	TP_PROTO(struct dw3000 *dw, enum operational_state operational_state),
	TP_ARGS(dw, operational_state),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(enum operational_state, operational_state)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->operational_state = operational_state;
	),
	TP_printk(DW_PR_FMT ", operational_state: %s", DW_PR_ARG,
		  __print_symbolic(__entry->operational_state,
				   DW3000_OP_STATE_FLAGS))
);

TRACE_EVENT(dw3000_check_operational_state,
	TP_PROTO(struct dw3000 *dw, int delay_dtu,
		 enum operational_state current_operational_state,
		 enum operational_state next_operational_state),
	TP_ARGS(dw, delay_dtu, current_operational_state,
		next_operational_state),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(int, delay_dtu)
		__field(enum operational_state, current_operational_state)
		__field(enum operational_state, next_operational_state)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->delay_dtu = delay_dtu;
		__entry->current_operational_state = current_operational_state;
		__entry->next_operational_state = next_operational_state;
	),
	TP_printk(DW_PR_FMT ", delay_dtu: %d, current_operational_state: %s, "
		  "next_operational_state: %s", DW_PR_ARG, __entry->delay_dtu,
		  __print_symbolic(__entry->current_operational_state,
				   DW3000_OP_STATE_FLAGS),
		  __print_symbolic(__entry->next_operational_state,
				   DW3000_OP_STATE_FLAGS))
);

DEFINE_EVENT(dw_only_evt, dw3000_read_rx_timestamp,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw)
);

TRACE_EVENT(dw3000_resync_dtu_sys_time,
	TP_PROTO(struct dw3000 *dw, u32 sys_time_sync, u32 dtu_sync),
	TP_ARGS(dw, sys_time_sync, dtu_sync),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u32, sys_time_sync)
		__field(u32, dtu_sync)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->sys_time_sync = sys_time_sync;
		__entry->dtu_sync = dtu_sync;
	),
	TP_printk(DW_PR_FMT ", sys_time_sync: %#08x, dtu_sync: %#08x",
		  DW_PR_ARG, __entry->sys_time_sync, __entry->dtu_sync)
);

TRACE_EVENT(dw3000_deep_sleep,
	TP_PROTO(struct dw3000 *dw, int rc),
	TP_ARGS(dw, rc),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(int, result)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->result = rc;
	),
	TP_printk(DW_PR_FMT ", result: %d", DW_PR_ARG,
		  __entry->result)
);

TRACE_EVENT(dw3000_deep_sleep_enter,
	TP_PROTO(struct dw3000 *dw, s64 time_before),
	TP_ARGS(dw, time_before),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(s64, time_before)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->time_before = time_before;
	),
	TP_printk(DW_PR_FMT ", time_ns: %lld", DW_PR_ARG, __entry->time_before)
);

TRACE_EVENT(dw3000_wakeup_timer_start,
	TP_PROTO(struct dw3000 *dw, int delay_us),
	TP_ARGS(dw, delay_us),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(int, delay_us)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->delay_us = delay_us;
	),
	TP_printk(DW_PR_FMT ", delay: %dus", DW_PR_ARG, __entry->delay_us)
);

DEFINE_EVENT(dw_only_evt, dw3000_wakeup,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw)
);

TRACE_EVENT(dw3000_wakeup_done,
	TP_PROTO(struct dw3000 *dw, s64 sleep_time_us, u32 sleep_enter_dtu,
		 u32 dtu_next_op, enum operational_state next_op),
	TP_ARGS(dw, sleep_time_us, sleep_enter_dtu, dtu_next_op, next_op),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(s64, sleep_time_us)
		__field(u32, sleep_enter_dtu)
		__field(u32, dtu_next_op)
		__field(enum operational_state, next_op)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->sleep_time_us = sleep_time_us;
		__entry->sleep_enter_dtu = sleep_enter_dtu;
		__entry->dtu_next_op  = dtu_next_op;
		__entry->next_op = next_op;
	),
	TP_printk(DW_PR_FMT ", sleep_us: %lld, sleep_enter_dtu: %#x, "
		  "next_op_date: %#x, next_op: %s", DW_PR_ARG,
		  __entry->sleep_time_us, __entry->sleep_enter_dtu,
		  __entry->dtu_next_op,
		  __print_symbolic(__entry->next_op, DW3000_OP_STATE_FLAGS))
);

TRACE_EVENT(dw3000_power_stats,
	TP_PROTO(struct dw3000 *dw, int state, u64 boot_time_ns, int len_or_date),
	TP_ARGS(dw, state, boot_time_ns, len_or_date),
	TP_STRUCT__entry(
		DW_ENTRY
		DW_PWR_ENTRY
		__field(u64, boot_time_ns)
		__field(int, len_or_date)
	),
	TP_fast_assign(
		DW_ASSIGN;
		DW_PWR_ASSIGN;
		__entry->boot_time_ns = boot_time_ns;
		__entry->len_or_date = len_or_date;
	),
	TP_printk(DW_PR_FMT ", " DW_PWR_PR_FMT ", boot_time_ns: %llu, len_or_date: %u",
		  DW_PR_ARG, DW_PWR_PR_ARG, __entry->boot_time_ns,
		  (unsigned)__entry->len_or_date)
);

TRACE_EVENT(dw3000_set_antenna_gpio,
	TP_PROTO(struct dw3000 *dw, int res, u8 gpio, u8 value),
	TP_ARGS(dw, res, gpio, value),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(int, res)
		__field(u8, gpio)
		__field(u8, value)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->res = res;
		__entry->gpio = gpio;
		__entry->value = value;
	),
	TP_printk(DW_PR_FMT ", res: %d, gpio: %u, value: %u",
		DW_PR_ARG, __entry->res, __entry->gpio, __entry->value)
);

TRACE_EVENT(dw3000_coex_gpio,
	TP_PROTO(struct dw3000 *dw, bool state, int delay_us, u32 expire, bool status),
	TP_ARGS(dw, state, delay_us, expire, status),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(bool, state)
		__field(int, delay_us)
		__field(u32, expire)
		__field(bool, status)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->state = state;
		__entry->delay_us = delay_us;
		__entry->expire = expire;
		__entry->status = status;
	),
	TP_printk(DW_PR_FMT ", current state: %s, new state: %s, delay_us: %d, expire: %u",
		DW_PR_ARG, __entry->status ? "ON" : "OFF", __entry->state ? "ON" : "OFF",
		__entry->delay_us, (unsigned)__entry->expire)
);

TRACE_EVENT(dw3000_coex_gpio_start,
	TP_PROTO(struct dw3000 *dw, int delay_us, bool status, int coex_interval_us),
	TP_ARGS(dw, delay_us, status, coex_interval_us),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(int, delay_us)
		__field(bool, status)
		__field(int, coex_interval_us)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->delay_us = delay_us;
		__entry->status = status;
		__entry->coex_interval_us = coex_interval_us;
	),
	TP_printk(DW_PR_FMT ", delay_us: %d, status: %s, coex_interval_us: %d",
		DW_PR_ARG, __entry->delay_us,
		__entry->status ? "ON" : "OFF", __entry->coex_interval_us)
);

TRACE_EVENT(dw3000_coex_gpio_stop,
	TP_PROTO(struct dw3000 *dw, bool status),
	TP_ARGS(dw, status),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(bool, status)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->status = status;
	),
	TP_printk(DW_PR_FMT ", status: %s",
		DW_PR_ARG,
		__entry->status ? "ON" : "OFF")
);

TRACE_EVENT(dw3000_adjust_tx_power,
	TP_PROTO(struct dw3000 *dw, u32 base_power, u32 adjusted_power,
		 u32 frm_dur, u16 pl_len, u8 chan, u16 th_boost,
		 u16 app_boost),
	TP_ARGS(dw, base_power, adjusted_power, frm_dur, pl_len, chan,
		th_boost, app_boost),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u32, base_power)
		__field(u32, adjusted_power)
		__field(u32, frm_dur)
		__field(u16, pl_len)
		__field(u8, chan)
		__field(u16, th_boost)
		__field(u16, app_boost)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->base_power = base_power;
		__entry->adjusted_power = adjusted_power;
		__entry->frm_dur = frm_dur;
		__entry->pl_len = pl_len;
		__entry->chan = chan;
		__entry->th_boost = th_boost;
		__entry->app_boost = app_boost;
	),
	TP_printk(DW_PR_FMT " base pwr: 0x%08x, adjusted: 0x%08x (chan: %u,"
		  "frm_dur: %u, PL_len: %u, th. boost: %u, applied boost: %u)",
		  DW_PR_ARG, __entry->base_power, __entry->adjusted_power,
		  __entry->chan, __entry->frm_dur, __entry->pl_len,
		  __entry->th_boost, __entry->app_boost)
);

TRACE_EVENT(dw3000_rx_rssi,
	TP_PROTO(struct dw3000 *dw, const char *chip_name, bool sts, u32 cir_pwr,
		u16 pacc_cnt, u8 prf_64mhz, u8 dgc_dec),
	TP_ARGS(dw, chip_name, sts, cir_pwr, pacc_cnt, prf_64mhz, dgc_dec),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(const char *, chip_name)
		__field(bool, sts)
		__field(u32, cir_pwr)
		__field(u16, pacc_cnt)
		__field(u8, prf_64mhz)
		__field(u8, dgc_dec)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->chip_name = chip_name;
		__entry->sts = sts;
		__entry->cir_pwr = cir_pwr;
		__entry->pacc_cnt = pacc_cnt;
		__entry->prf_64mhz = prf_64mhz;
		__entry->dgc_dec = dgc_dec;
	),
	TP_printk(DW_PR_FMT ", chip: %s sts: %u cir_pwr: %u pacc_cnt: %u"
		" prf_64mhz: %u dgc_dec: %u",
		DW_PR_ARG, __entry->chip_name, __entry->sts,
		__entry->cir_pwr, __entry->pacc_cnt, __entry->prf_64mhz,
		__entry->dgc_dec)
	);

TRACE_EVENT(dw3000_nfcc_coex_isr,
	TP_PROTO(struct dw3000 *dw, u8 dss_stat),
	TP_ARGS(dw, dss_stat),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u8, dss_stat)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->dss_stat = dss_stat;
	),
	TP_printk(DW_PR_FMT ", dss_stat=%#x", DW_PR_ARG, __entry->dss_stat)
);

TRACE_EVENT(dw3000_nfcc_coex_sleep,
	TP_PROTO(struct dw3000 *dw, u32 sleep_dtu),
	TP_ARGS(dw, sleep_dtu),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u32, sleep_dtu)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->sleep_dtu = sleep_dtu;
	),
	TP_printk(DW_PR_FMT ", sleep_dtu=%u", DW_PR_ARG, __entry->sleep_dtu)
);

TRACE_EVENT(dw3000_nfcc_coex_clock_sync_payload_put,
	TP_PROTO(struct dw3000 *dw, u32 session_time0_sys_time),
	TP_ARGS(dw, session_time0_sys_time),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u32, session_time0_sys_time)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->session_time0_sys_time = session_time0_sys_time;
	),
	TP_printk(DW_PR_FMT ", session_time0_sys_time=%#08x", DW_PR_ARG,
		  __entry->session_time0_sys_time)
);

TRACE_EVENT(dw3000_nfcc_coex_clock_offset_payload_put,
	TP_PROTO(struct dw3000 *dw, u32 clock_offset_sys_time),
	TP_ARGS(dw, clock_offset_sys_time),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u32, clock_offset_sys_time)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->clock_offset_sys_time = clock_offset_sys_time;
	),
	TP_printk(DW_PR_FMT ", clock_offset_sys_time=%#08x", DW_PR_ARG,
		  __entry->clock_offset_sys_time)
);

TRACE_EVENT(dw3000_nfcc_coex_rx_msg_info,
	TP_PROTO(struct dw3000 *dw, u32 next_timestamp_sys_time,
		 int next_duration_sys_time),
	TP_ARGS(dw, next_timestamp_sys_time, next_duration_sys_time),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u32, next_timestamp_sys_time)
		__field(int, next_duration_sys_time)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->next_timestamp_sys_time = next_timestamp_sys_time;
		__entry->next_duration_sys_time = next_duration_sys_time;
	),
	TP_printk(DW_PR_FMT ", next_timestamp_sys_time=%#08x"
		  ", next_duration_sys_time=%#08x", DW_PR_ARG,
		  __entry->next_timestamp_sys_time,
		  __entry->next_duration_sys_time)
);

TRACE_EVENT(dw3000_nfcc_coex_header_check,
	TP_PROTO(struct dw3000 *dw, const char *signature,
		 u8 ver_id, u8 seq_num, u8 nb_tlv),
	TP_ARGS(dw, signature, ver_id, seq_num, nb_tlv),
	TP_STRUCT__entry(
		DW_ENTRY
		__array(u8, signature, DW3000_NFCC_COEX_SIGNATURE_LEN)
		__field(u8, ver_id)
		__field(u8, seq_num)
		__field(u8, nb_tlv)
	),
	TP_fast_assign(
		DW_ASSIGN;
		memcpy(__entry->signature, signature,
		       DW3000_NFCC_COEX_SIGNATURE_LEN);
		__entry->ver_id = ver_id;
		__entry->seq_num = seq_num;
		__entry->nb_tlv = nb_tlv;
	),
	TP_printk(DW_PR_FMT ", signature=%s, ver_id=%u, seq_num=%u, nb_tlv=%u", DW_PR_ARG,
		  __print_array(__entry->signature, DW3000_NFCC_COEX_SIGNATURE_LEN, sizeof(u8)),
		  __entry->ver_id, __entry->seq_num, __entry->nb_tlv)
);

TRACE_EVENT(dw3000_nfcc_coex_err,
	TP_PROTO(struct dw3000 *dw, const char *err),
	TP_ARGS(dw, err),
	TP_STRUCT__entry(
		DW_ENTRY
		__string(err, err)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__assign_str(err, err);
	),
	TP_printk(DW_PR_FMT ", err=\"%s\"", DW_PR_ARG, __get_str(err))
);

TRACE_EVENT(dw3000_nfcc_coex_warn,
	TP_PROTO(struct dw3000 *dw, const char *warn),
	TP_ARGS(dw, warn),
	TP_STRUCT__entry(
		DW_ENTRY
		__string(warn, warn)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__assign_str(warn, warn);
	),
	TP_printk(DW_PR_FMT ", warn=\"%s\"", DW_PR_ARG, __get_str(warn))
);

DEFINE_EVENT(dw_only_evt, dw3000_nfcc_coex_watchdog,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw)
);

TRACE_EVENT(dw3000_nfcc_coex_enable,
	TP_PROTO(struct dw3000 *dw, int channel),
	TP_ARGS(dw, channel),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(int, channel)
	),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->channel = channel;
	),
	TP_printk(DW_PR_FMT ", channel=%d", DW_PR_ARG, __entry->channel)
);

DEFINE_EVENT(dw_only_evt, dw3000_nfcc_coex_disable,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw)
);

/*************************************************************
 *		dw3000 optional functions traces	     *
 *************************************************************/

#ifdef CONFIG_MCPS802154_TESTMODE
TRACE_EVENT(dw3000_tm_cmd,
	TP_PROTO(struct dw3000 *dw, u32 cmd),
	TP_ARGS(dw, cmd),
	TP_STRUCT__entry(
		DW_ENTRY
		DW_TM_CMD_ENTRY
	),
	TP_fast_assign(
		DW_ASSIGN;
		DW_TM_CMD_ASSIGN;
	),
	TP_printk(DW_PR_FMT ", " DW_TM_CMD_PR_FMT, DW_PR_ARG, DW_TM_CMD_PR_ARG)
);
#endif



TRACE_EVENT(dw3000_set_pdoa,
	TP_PROTO(struct dw3000 *dw, u32 mode),
	TP_ARGS(dw, mode),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u32, mode)
		),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->mode = mode;
		),
	TP_printk(DW_PR_FMT " Set PDoA mode to %d", DW_PR_ARG,
		  __entry->mode)
	);

TRACE_EVENT(dw3000_read_pdoa,
	TP_PROTO(struct dw3000 *dw, u32 pdoa),
	TP_ARGS(dw, pdoa),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u32, pdoa)
		),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->pdoa = pdoa;
		),
	TP_printk(DW_PR_FMT " pdoa=%#08x", DW_PR_ARG,
		  __entry->pdoa)
	);

TRACE_EVENT(dw3000_testmode_continuous_tx_start,
	TP_PROTO(struct dw3000 *dw, u8 chan, u32 len, u32 rate),
	TP_ARGS(dw, chan, len, rate),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(u8, chan)
		__field(u32, len)
		__field(u32, rate)
		),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->chan = chan;
		__entry->len = len;
		__entry->rate = rate;
		),
	TP_printk(DW_PR_FMT " chan=%d, len=%d, rate=%d", DW_PR_ARG,
		  __entry->chan,
		  __entry->len,
		  __entry->rate)
	);

TRACE_EVENT(dw3000_testmode_continuous_tx_stop,
	TP_PROTO(struct dw3000 *dw),
	TP_ARGS(dw),
	TP_STRUCT__entry(
		DW_ENTRY
		),
	TP_fast_assign(
		DW_ASSIGN;
		),
	TP_printk(DW_PR_FMT, DW_PR_ARG)
	);

TRACE_EVENT(dw3000_read_clockoffset,
	TP_PROTO(struct dw3000 *dw, s16 cfo),
	TP_ARGS(dw, cfo),
	TP_STRUCT__entry(
		DW_ENTRY
		__field(s16, cfo)
		),
	TP_fast_assign(
		DW_ASSIGN;
		__entry->cfo = cfo;
		),
	TP_printk(DW_PR_FMT " clockoffset=%d", DW_PR_ARG,
		  __entry->cfo)
	);

TRACE_EVENT(dw3000_get_counters_first_part,
		TP_PROTO(struct dw3000 *dw, u16 rxphe, u16 rxfsl, u16 rxcfg, u16 rxovrr, u16 rxsto, u16 rxpto, u16 fwto, u16 txfrs, u16 hpwarn, u16 spicrc),
		TP_ARGS(dw, rxphe, rxfsl, rxcfg, rxovrr, rxsto, rxpto, fwto, txfrs, hpwarn, spicrc),
		TP_STRUCT__entry(
			DW_ENTRY
			__field(u16, rxphe)
			__field(u16, rxfsl)
			__field(u16, rxcfg)
			__field(u16, rxovrr)
			__field(u16, rxsto)
			__field(u16, rxpto)
			__field(u16, fwto)
			__field(u16, txfrs)
			__field(u16, hpwarn)
			__field(u16, spicrc)
			),
		TP_fast_assign(
			DW_ASSIGN;
			__entry->rxphe = rxphe;
			__entry->rxfsl = rxfsl;
			__entry->rxcfg = rxcfg;
			__entry->rxovrr = rxovrr;
			__entry->rxsto = rxsto;
			__entry->rxpto = rxpto;
			__entry->fwto = fwto;
			__entry->txfrs = txfrs;
			__entry->hpwarn = hpwarn;
			__entry->spicrc = spicrc;
			),
		TP_printk(DW_PR_FMT " rxphe=%d rxfsl=%d rxcfg=%d, rxovrr=%d, rxsto=%d, rxpto=%d, fwto,=%d txfrs=%d, hpwarn=%d, spicrc=%d", DW_PR_ARG,
			__entry->rxphe,
			__entry->rxfsl,
			__entry->rxcfg,
			__entry->rxovrr,
			__entry->rxsto,
			__entry->rxpto,
			__entry->fwto,
			__entry->txfrs,
			__entry->hpwarn,
			__entry->spicrc)
	   );

TRACE_EVENT(dw3000_get_counters_second_part,
		TP_PROTO(struct dw3000 *dw, u16 rxprej, u16 cperr, u16 vwarn ),
		TP_ARGS(dw, rxprej, cperr, vwarn),
		TP_STRUCT__entry(
			DW_ENTRY
			__field(u16, rxprej)
			__field(u16, cperr)
			__field(u16, vwarn)
			),
		TP_fast_assign(
			DW_ASSIGN;
			__entry->rxprej = rxprej;
			__entry->cperr = cperr;
			__entry->vwarn = vwarn;
			),
		TP_printk(DW_PR_FMT " rxfce=%d, sts_qual_err=%d, vwarn=%d", DW_PR_ARG,
			__entry->rxprej,
			__entry->cperr,
			__entry->vwarn)
	   );

/* clang-format on */
#endif /* !__DW3000_TRACE || TRACE_HEADER_MULTI_READ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE dw3000_trc
#include <trace/define_trace.h>
