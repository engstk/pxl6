/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM remoteproc

#define TRACE_INCLUDE_PATH trace/hooks

#if !defined(_TRACE_HOOK_RPROC_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HOOK_RPROC_H

#include <trace/hooks/vendor_hooks.h>

struct rproc;

DECLARE_HOOK(android_vh_rproc_recovery,
	TP_PROTO(struct rproc *rproc),
	TP_ARGS(rproc));

#endif /* _TRACE_HOOK_RPROC_H */
/* This part must be outside protection */
#include <trace/define_trace.h>
