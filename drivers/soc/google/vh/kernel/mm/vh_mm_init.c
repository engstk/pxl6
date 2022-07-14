// SPDX-License-Identifier: GPL-2.0-only
/* vendor_mm_init.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2022 Google LLC
 */

#include <linux/kobject.h>
#include <linux/module.h>
#include <trace/hooks/gup.h>
#include "../../include/gup.h"

struct kobject *vendor_mm_kobj;
EXPORT_SYMBOL_GPL(vendor_mm_kobj);

extern int pixel_mm_cma_sysfs(struct kobject *parent);

static int vh_mm_init(void)
{
	int ret;

	vendor_mm_kobj = kobject_create_and_add("vendor_mm", kernel_kobj);
	if (!vendor_mm_kobj)
		return -ENOMEM;

	ret = pixel_mm_cma_sysfs(vendor_mm_kobj);
	if (ret) {
		kobject_put(vendor_mm_kobj);
		return ret;
	}

	/*
	 * Not sure this error handling is meaningful for vendor hook.
	 * Maybe better to rely on the just BUG_ON?
	 */
	ret = register_trace_android_vh_try_grab_compound_head(
				vh_try_grab_compound_head, NULL);
	if (ret)
		return ret;
	ret = register_trace_android_vh___get_user_pages_remote(
				vh___get_user_pages_remote, NULL);
	if (ret)
		return ret;
	ret = register_trace_android_vh_get_user_pages(
				vh_android_vh_get_user_pages, NULL);
	if (ret)
		return ret;
	ret = register_trace_android_vh_internal_get_user_pages_fast(
				vh_internal_get_user_pages_fast, NULL);
	if (ret)
		return ret;
	ret = register_trace_android_vh_pin_user_pages(
				vh_pin_user_pages, NULL);
	return ret;
}
module_init(vh_mm_init);
MODULE_LICENSE("GPL v2");
