// SPDX-License-Identifier: GPL-2.0
/*
 * Abrolhos EdgeTPU power management support
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/delay.h>
#include <linux/gsa/gsa_tpu.h>
#include <linux/iopoll.h>

#include "edgetpu-config.h"
#include "edgetpu-firmware.h"
#include "edgetpu-internal.h"
#include "edgetpu-pm.h"
#include "mobile-pm.h"

#define TPU_DEFAULT_POWER_STATE		TPU_DEEP_SLEEP_CLOCKS_SLOW

#include "mobile-pm.c"

static int abrolhos_core_pwr_get(void *data, u64 *val)
{
	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN,
			TPU_DEBUG_REQ | TPU_CORE_PWR_DEBUG);
	return 0;
}

static int abrolhos_core_pwr_set(void *data, u64 val)
{
	int ret;
	unsigned long dbg_rate_req;

	dbg_rate_req = TPU_DEBUG_REQ | TPU_CORE_PWR_DEBUG;
	dbg_rate_req |= val;

	ret = exynos_acpm_set_rate(TPU_ACPM_DOMAIN, dbg_rate_req);
	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_core_pwr, abrolhos_core_pwr_get,
		abrolhos_core_pwr_set, "%llu\n");

static int abrolhos_pm_after_create(struct edgetpu_dev *etdev)
{
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct edgetpu_mobile_platform_pwr *platform_pwr = &etmdev->platform_pwr;

	debugfs_create_file("core_pwr", 0660, platform_pwr->debugfs_dir, etdev, &fops_tpu_core_pwr);

	return 0;
}

static void abrolhos_firmware_down(struct edgetpu_dev *etdev)
{
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);

	if (!edgetpu_pchannel_power_down(etdev, false))
		return;

	etdev_warn(etdev, "Firmware shutdown request failed!\n");
	etdev_warn(etdev, "Attempting firmware restart\n");
	if (!edgetpu_firmware_restart_locked(etdev, true) &&
	    !edgetpu_pchannel_power_down(etdev, false))
		return;

	if (etmdev->gsa_dev) {
		etdev_warn(etdev, "Requesting early GSA reset\n");
		/*
		 * p-channel failed, request GSA shutdown to make sure the CPU is
		 * reset.
		 * The GSA->APM request will clear any pending DVFS status from the
		 * CPU.
		 */
		gsa_send_tpu_cmd(etmdev->gsa_dev, GSA_TPU_SHUTDOWN);
	}
}

int edgetpu_chip_pm_create(struct edgetpu_dev *etdev)
{
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct edgetpu_mobile_platform_pwr *platform_pwr = &etmdev->platform_pwr;

	platform_pwr->firmware_down = abrolhos_firmware_down;
	platform_pwr->after_create = abrolhos_pm_after_create;
	platform_pwr->acpm_set_rate = exynos_acpm_set_rate;

	return edgetpu_mobile_pm_create(etdev);
}
