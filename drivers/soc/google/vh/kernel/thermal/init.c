// SPDX-License-Identifier: GPL-2.0-only
/* init.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2021 Google LLC
 */

#include <linux/module.h>
#include <linux/thermal.h>
#include <trace/hooks/thermal.h>

extern void vh_enable_thermal_genl_check(void *data, int event, int tz_id, int *enable_thermal_genl);
extern void vh_thermal_pm_notify_suspend(void *data, struct thermal_zone_device *tz,
					 int *is_wakeable);

static int vh_thermal_init(void)
{
	int ret = 0;
	ret = register_trace_android_vh_enable_thermal_genl_check(
						vh_enable_thermal_genl_check, NULL);
	if (ret)
		return ret;

	ret =  register_trace_android_vh_thermal_pm_notify_suspend(
						vh_thermal_pm_notify_suspend, NULL);
	return ret;
}

module_init(vh_thermal_init);
MODULE_LICENSE("GPL v2");
