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
#define TRACE_SYSTEM mcps802154_region_nfcc_coex

#if !defined(NFCC_COEX_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define NFCC_COEX_TRACE_H

#include <linux/tracepoint.h>
#include <net/nfcc_coex_region_nl.h>
#include "mcps802154_i.h"
#include "nfcc_coex_region.h"
#include "nfcc_coex_session.h"

/* clang-format off */
#define nfcc_coex_call_name(name)                       \
	{                                               \
		NFCC_COEX_CALL_##name, #name            \
	}
#define NFCC_COEX_CALL_SYMBOLS                          \
	nfcc_coex_call_name(CCC_SESSION_START),         \
	nfcc_coex_call_name(CCC_SESSION_STOP),          \
	nfcc_coex_call_name(CCC_SESSION_NOTIFICATION)
TRACE_DEFINE_ENUM(NFCC_COEX_CALL_CCC_SESSION_START);
TRACE_DEFINE_ENUM(NFCC_COEX_CALL_CCC_SESSION_STOP);
TRACE_DEFINE_ENUM(NFCC_COEX_CALL_CCC_SESSION_NOTIFICATION);

#define nfcc_coex_state_name(name)                       \
	{                                                \
		NFCC_COEX_STATE_##name, #name            \
	}
#define NFCC_COEX_STATE_SYMBOLS                          \
	nfcc_coex_state_name(IDLE),                      \
	nfcc_coex_state_name(ACCESSING),                 \
	nfcc_coex_state_name(STOPPING)
TRACE_DEFINE_ENUM(NFCC_COEX_STATE_IDLE);
TRACE_DEFINE_ENUM(NFCC_COEX_STATE_ACCESSING);
TRACE_DEFINE_ENUM(NFCC_COEX_STATE_STOPPING);

#define NFCC_COEX_LOCAL_ENTRY __field(enum nfcc_coex_state, state)
#define NFCC_COEX_LOCAL_ASSIGN __entry->state = local->session.state
#define NFCC_COEX_LOCAL_PR_FMT "state=%s"
#define NFCC_COEX_LOCAL_PR_ARG \
	__print_symbolic(__entry->state, NFCC_COEX_STATE_SYMBOLS)

DECLARE_EVENT_CLASS(
	local_only_evt,
	TP_PROTO(const struct nfcc_coex_local *local),
	TP_ARGS(local),
	TP_STRUCT__entry(
		NFCC_COEX_LOCAL_ENTRY
	),
	TP_fast_assign(
		NFCC_COEX_LOCAL_ASSIGN;
	),
	TP_printk(NFCC_COEX_LOCAL_PR_FMT, NFCC_COEX_LOCAL_PR_ARG)
);

TRACE_EVENT(
	region_nfcc_coex_session_start,
	TP_PROTO(const struct nfcc_coex_local *local,
		 const struct nfcc_coex_session_params *p),
	TP_ARGS(local, p),
	TP_STRUCT__entry(
		NFCC_COEX_LOCAL_ENTRY
		__field(u64, time0_ns)
		__field(u8, channel_number)
	),
	TP_fast_assign(
		NFCC_COEX_LOCAL_ASSIGN;
		__entry->time0_ns = p->time0_ns;
		__entry->channel_number = p->channel_number;
	),
	TP_printk(NFCC_COEX_LOCAL_PR_FMT " time0_ns=%llu channel_number=%d",
		  NFCC_COEX_LOCAL_PR_ARG, __entry->time0_ns,
		  __entry->channel_number)
);

DEFINE_EVENT(
	local_only_evt, region_nfcc_coex_session_stop,
	TP_PROTO(const struct nfcc_coex_local *local),
	TP_ARGS(local)
);

DEFINE_EVENT(
	local_only_evt, region_nfcc_coex_notify_stop,
	TP_PROTO(const struct nfcc_coex_local *local),
	TP_ARGS(local)
);

TRACE_EVENT(
	region_nfcc_coex_call,
	TP_PROTO(const struct nfcc_coex_local *local,
		 enum nfcc_coex_call call_id),
	TP_ARGS(local, call_id),
	TP_STRUCT__entry(
		NFCC_COEX_LOCAL_ENTRY
		__field(enum nfcc_coex_call, call_id)
		),
	TP_fast_assign(
		NFCC_COEX_LOCAL_ASSIGN;
		__entry->call_id = call_id;
		),
	TP_printk(NFCC_COEX_LOCAL_PR_FMT " call_id=%s",
		  NFCC_COEX_LOCAL_PR_ARG,
		  __print_symbolic(__entry->call_id, NFCC_COEX_CALL_SYMBOLS))
);

TRACE_EVENT(
	region_nfcc_coex_get_demand,
	TP_PROTO(const struct nfcc_coex_local *local,
		 u32 next_timestamp_dtu,
		 const struct mcps802154_region_demand *rd),
	TP_ARGS(local, next_timestamp_dtu, rd),
	TP_STRUCT__entry(
		NFCC_COEX_LOCAL_ENTRY
		__field(u32, next_timestamp_dtu)
		__field(u32, timestamp_dtu)
		__field(int, duration_dtu)
		),
	TP_fast_assign(
		NFCC_COEX_LOCAL_ASSIGN;
		__entry->next_timestamp_dtu = next_timestamp_dtu;
		__entry->timestamp_dtu = rd->timestamp_dtu;
		__entry->duration_dtu = rd->duration_dtu;
		),
	TP_printk(NFCC_COEX_LOCAL_PR_FMT " next_timestamp_dtu=0x%08x "
		  "rd.timestamp_dtu=0x%08x rd.duration_dtu=0x%08x",
		  NFCC_COEX_LOCAL_PR_ARG,
		  __entry->next_timestamp_dtu,
		  __entry->timestamp_dtu,
		  __entry->duration_dtu)
);

TRACE_EVENT(
	region_nfcc_coex_set_state,
	TP_PROTO(const struct nfcc_coex_local *local,
		 enum nfcc_coex_state new_state),
	TP_ARGS(local, new_state),
	TP_STRUCT__entry(
		NFCC_COEX_LOCAL_ENTRY
		__field(enum nfcc_coex_state, new_state)
		),
	TP_fast_assign(
		NFCC_COEX_LOCAL_ASSIGN;
		__entry->new_state = new_state;
		),
	TP_printk(NFCC_COEX_LOCAL_PR_FMT " new_state=%s",
		  NFCC_COEX_LOCAL_PR_ARG,
		  __print_symbolic(__entry->new_state,
				       NFCC_COEX_STATE_SYMBOLS))
);

TRACE_EVENT(
	region_nfcc_coex_report,
	TP_PROTO(const struct nfcc_coex_local *local,
		 const struct dw3000_vendor_cmd_nfcc_coex_get_access_info *info),
	TP_ARGS(local, info),
	TP_STRUCT__entry(
		NFCC_COEX_LOCAL_ENTRY
		__field(bool, watchdog_timeout)
		__field(bool, stop)
		),
	TP_fast_assign(
		NFCC_COEX_LOCAL_ASSIGN;
		__entry->watchdog_timeout = info->watchdog_timeout;
		__entry->stop = info->stop;
		),
	TP_printk(NFCC_COEX_LOCAL_PR_FMT " watchdog_timeout=%s stop=%s",
		  NFCC_COEX_LOCAL_PR_ARG,
		  __entry->watchdog_timeout ? "true": "false",
		  __entry->stop ? "true": "false")
);

DEFINE_EVENT(
	local_only_evt, region_nfcc_coex_report_nla_put_failure,
	TP_PROTO(const struct nfcc_coex_local *local),
	TP_ARGS(local)
);

/* clang-format on */

#endif /* !NFCC_COEX_TRACE_H || TRACE_HEADER_MULTI_READ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE nfcc_coex_trace
#include <trace/define_trace.h>
