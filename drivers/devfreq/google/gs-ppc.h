/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * GS101 - PPC header
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __DEVFREQ_GS101_PPC_H
#define __DEVFREQ_GS101_PPC_H __FILE__

#include <linux/ktime.h>
#include <linux/devfreq.h>
#include <soc/google/exynos-devfreq.h>

#if IS_ENABLED(CONFIG_EXYNOS_ALT_DVFS)
int exynos_devfreq_um_init(struct exynos_devfreq_data *data);
void exynos_devfreq_um_exit(struct exynos_devfreq_data *data);
void register_get_dev_status(struct exynos_devfreq_data *data);
#else
#define exynos_devfreq_um_init(a)                                              \
	do {                                                                   \
	} while (0)
#define exynos_devfreq_um_exit(a)                                              \
	do {                                                                   \
	} while (0)
#define register_get_dev_status(a)                                             \
	do {                                                                   \
	} while (0)
#endif

#endif /* __DEVFREQ_GS101_PPC_H */
