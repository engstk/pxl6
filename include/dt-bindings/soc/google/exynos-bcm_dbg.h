/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Device Tree binding constants for Exynos BCM Debug
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef _DT_BINDINGS_EXYNOS_BCM_DBG_H
#define _DT_BINDINGS_EXYNOS_BCM_DBG_H

/* BCM Pre-defined event type */
#define NO_PRE_DEFINE_EVT		0
#define LATENCY_FMT_EVT			1
#define MO_FMT_EVT			2
#define BURST_LENGTH_FMT_EVT		3
#define REQ_BLOCK_FMT_EVT		4
#define DATA_BLOCK_FMT_EVT		5
#define REQ_TYPE_FMT_EVT		6
#define PRE_DEFINE_EVT_MAX		7

#define BCM_STOP			0
#define BCM_RUN				1

#define BCM_IP_DIS			0
#define BCM_IP_EN			1

#define BCM_MODE_INTERVAL		0
#define BCM_MODE_ONCE			1
#define BCM_MODE_USERCTRL		2
#define BCM_MODE_ACCUMULATOR		3
#define BCM_MODE_MAX			4

#define PANIC_HANDLE			0
#define CAMERA_DRIVER			1
#define MODEM_IF			2
#define ITMON_HANDLE			3
#define STOP_OWNER_MAX			4

#endif	/* _DT_BINDINGS_EXYNOS_BCM_DBG_H */
