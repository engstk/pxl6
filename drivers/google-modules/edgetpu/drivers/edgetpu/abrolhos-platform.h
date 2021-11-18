/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Abrolhos device driver for the Google EdgeTPU ML accelerator.
 *
 * Copyright (C) 2019 Google, Inc.
 */

#ifndef __ABROLHOS_PLATFORM_H__
#define __ABROLHOS_PLATFORM_H__

#include "edgetpu-internal.h"
#include "edgetpu-mobile-platform.h"
#include "mobile-debug-dump.h"

#define to_abrolhos_dev(etdev)                                                                     \
	container_of((to_mobile_dev(etdev)), struct abrolhos_platform_dev, mobile_dev)

struct abrolhos_platform_dev {
	struct edgetpu_mobile_platform_dev mobile_dev;
	/* subsystem coredump info struct */
	struct mobile_sscd_info sscd_info;
};

#endif /* __ABROLHOS_PLATFORM_H__ */
