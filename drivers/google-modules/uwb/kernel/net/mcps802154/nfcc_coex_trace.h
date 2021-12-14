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
#include "mcps802154_i.h"
#include <net/vendor_cmd.h>

/* clang-format off */

#define LOCAL_ENTRY __field(int, hw_idx)
#define LOCAL_ASSIGN __entry->hw_idx = local->hw_idx
#define LOCAL_PR_FMT "hw%d"
#define LOCAL_PR_ARG __entry->hw_idx

TRACE_EVENT(nfcc_coex_llhw_return_int,
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

TRACE_EVENT(nfcc_coex_llhw_vendor_cmd,
	TP_PROTO(const struct mcps802154_local *local, u32 vendor_id,
		 u32 subcmd),
	TP_ARGS(local, vendor_id, subcmd),
	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(u32, vendor_id)
		__field(u32, subcmd)
		),
	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->vendor_id = vendor_id;
		__entry->subcmd = subcmd;
		),
	TP_printk(LOCAL_PR_FMT " vendor_id=%06x subcmd=%s", LOCAL_PR_ARG,
		  __entry->vendor_id,
		  __print_symbolic(__entry->subcmd,
			{ DW3000_VENDOR_CMD_NFCC_COEX_HANDLE_ACCESS, "HANDLE_ACCESS"},
			{ DW3000_VENDOR_CMD_NFCC_COEX_GET_ACCESS_INFORMATION, "GET_ACCESS_INFORMATION"},
			{ DW3000_VENDOR_CMD_NFCC_COEX_STOP, "STOP"}))
	);

#endif /* !NFCC_COEX_TRACE_H || TRACE_HEADER_MULTI_READ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE nfcc_coex_trace
#include <trace/define_trace.h>
