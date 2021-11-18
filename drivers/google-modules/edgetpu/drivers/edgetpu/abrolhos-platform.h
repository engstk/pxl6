/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Abrolhos device driver for the Google EdgeTPU ML accelerator.
 *
 * Copyright (C) 2019 Google, Inc.
 */
#ifndef __ABROLHOS_PLATFORM_H__
#define __ABROLHOS_PLATFORM_H__

#include <linux/device.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <soc/google/bcl.h>
#include <soc/google/bts.h>
#include <soc/google/exynos_pm_qos.h>

#include "abrolhos-debug-dump.h"
#include "abrolhos-pm.h"
#include "edgetpu-internal.h"

#define to_abrolhos_dev(etdev)                                                 \
	container_of(etdev, struct abrolhos_platform_dev, edgetpu_dev)

struct abrolhos_platform_pwr {
	struct mutex policy_lock;
	enum tpu_pwr_state curr_policy;
	struct mutex state_lock;
	u64 min_state;
	u64 requested_state;
	/* INT/MIF requests for memory bandwidth */
	struct exynos_pm_qos_request int_min;
	struct exynos_pm_qos_request mif_min;
	/* BTS */
	unsigned int performance_scenario;
	int scenario_count;
	struct mutex scenario_lock;
};

struct abrolhos_platform_dev {
	struct edgetpu_dev edgetpu_dev;
	struct abrolhos_platform_pwr platform_pwr;
	int irq;
	phys_addr_t fw_region_paddr;
	size_t fw_region_size;
	void *shared_mem_vaddr;
	phys_addr_t shared_mem_paddr;
	size_t shared_mem_size;
	struct device *gsa_dev;
	void __iomem *ssmt_base;
	struct edgetpu_coherent_mem log_mem;
	struct edgetpu_coherent_mem trace_mem;
	struct abrolhos_sscd_info sscd_info;
#if IS_ENABLED(CONFIG_GOOGLE_BCL)
	struct bcl_device *bcl_dev;
#endif
	/* Protects TZ Mailbox client pointer */
	struct mutex tz_mailbox_lock;
	/* TZ mailbox client */
	struct edgetpu_client *secure_client;
};

#endif /* __ABROLHOS_PLATFORM_H__ */
