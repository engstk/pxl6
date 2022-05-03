/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS GS201 Platform-Specific Functions
 *
 * Copyright (c) 2021 Google, LLC
 */

#ifndef LWIS_PLATFORM_GS201_H_
#define LWIS_PLATFORM_GS201_H_

#include <soc/google/exynos_pm_qos.h>

struct lwis_platform {
	struct exynos_pm_qos_request pm_qos_int_cam;
	struct exynos_pm_qos_request pm_qos_int;
	struct exynos_pm_qos_request pm_qos_cam;
	struct exynos_pm_qos_request pm_qos_mem;
	struct exynos_pm_qos_request pm_qos_tnr;
	/* struct exynos_pm_qos_request pm_qos_hpg; */
};

#endif /* LWIS_PLATFORM_GS201_H_ */
