/*
 * Google LWIS Exynos Platform-specific Functions
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_PLATFORM_EXYNOS_H_
#define LWIS_PLATFORM_EXYNOS_H_

#include <linux/pm_qos.h>

struct lwis_platform {
	struct pm_qos_request pm_qos_int_cam;
	struct pm_qos_request pm_qos_int;
	struct pm_qos_request pm_qos_cam;
	struct pm_qos_request pm_qos_mem;
	struct pm_qos_request pm_qos_hpg;
	struct pm_qos_request pm_qos_tnr;
};

#endif /* LWIS_PLATFORM_H_ */
