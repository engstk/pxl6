/*
 * Google LWIS Lwis Exynos-specific platform functions
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "lwis_platform_exynos.h"

#include <linux/exynos_iovmm.h>
#include <linux/iommu.h>
#include <linux/of.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <soc/samsung/bts.h>

#include "lwis_device_dpm.h"
#include "lwis_debug.h"
#include "lwis_platform.h"

/* Uncomment to let kernel panic when IOMMU hits a page fault. */
/* TODO: Add error handling to propagate SysMMU errors back to userspace,
 * so we don't need to panic here. */
#define ENABLE_PAGE_FAULT_PANIC

int lwis_platform_probe(struct lwis_device *lwis_dev)
{
	struct lwis_platform *platform;
	BUG_ON(!lwis_dev);
	platform = kzalloc(sizeof(struct lwis_platform), GFP_KERNEL);
	if (IS_ERR_OR_NULL(platform)) {
		return -ENOMEM;
	}
	lwis_dev->platform = platform;

	/* Enable runtime power management for the platform device */
	pm_runtime_enable(&lwis_dev->plat_dev->dev);

	lwis_dev->bts_index = BTS_UNSUPPORTED;
	/* Only IOREG devices will access DMA resources */
	if (lwis_dev->type != DEVICE_TYPE_IOREG) {
		return 0;
	}
	/* Register to bts */
	lwis_dev->bts_index = bts_get_bwindex(lwis_dev->name);
	if (lwis_dev->bts_index < 0) {
		dev_err(lwis_dev->dev, "Failed to register to BTS, ret: %d\n", lwis_dev->bts_index);
		lwis_dev->bts_index = BTS_UNSUPPORTED;
	}

	return 0;
}

static int __attribute__((unused))
iovmm_fault_handler(struct iommu_domain *domain, struct device *dev, unsigned long fault_addr,
		    int fault_flag, void *token)
{
	struct lwis_device *lwis_dev = (struct lwis_device *)token;

	pr_err("############ LWIS IOVMM PAGE FAULT ############\n");
	pr_err("\n");
	pr_err("Device: %s IOVMM Page Fault at Address: 0x%016lx Flag: 0x%08x\n", lwis_dev->name,
	       fault_addr, fault_flag);
	pr_err("\n");
	lwis_debug_print_transaction_info(lwis_dev);
	pr_err("\n");
	lwis_debug_print_event_states_info(lwis_dev);
	pr_err("\n");
	lwis_debug_print_buffer_info(lwis_dev);
	pr_err("\n");
	pr_err("###############################################\n");

#ifdef ENABLE_PAGE_FAULT_PANIC
	return NOTIFY_BAD;
#else
	return NOTIFY_OK;
#endif
}

int lwis_platform_device_enable(struct lwis_device *lwis_dev)
{
	int ret;
	struct lwis_platform *platform;
	const int core_clock_qos = 67000;
	const int hpg_qos = 1;

	BUG_ON(!lwis_dev);
	platform = lwis_dev->platform;
	if (!platform) {
		return -ENODEV;
	}

	/* Upref the runtime power management controls for the platform dev */
	ret = pm_runtime_get_sync(&lwis_dev->plat_dev->dev);
	if (ret < 0) {
		pr_err("Unable to enable platform device\n");
		return ret;
	}
	if (lwis_dev->has_iommu) {
		/* Activate IOMMU/SYSMMU for the platform device */
		ret = iovmm_activate(&lwis_dev->plat_dev->dev);
		if (ret < 0) {
			pr_err("Failed to enable IOMMU for the device: %d\n", ret);
			return ret;
		}
		/* Set SYSMMU fault handler */
		iovmm_set_fault_handler(&lwis_dev->plat_dev->dev, iovmm_fault_handler, lwis_dev);
	}

	/* Set hardcoded DVFS levels */
	if (!pm_qos_request_active(&platform->pm_qos_hpg))
		pm_qos_add_request(&platform->pm_qos_hpg, PM_QOS_CPU_ONLINE_MIN, hpg_qos);

	if (lwis_dev->clock_family != CLOCK_FAMILY_INVALID &&
	    lwis_dev->clock_family < NUM_CLOCK_FAMILY) {
		ret = lwis_platform_update_qos(lwis_dev, core_clock_qos, lwis_dev->clock_family);
		if (ret < 0) {
			dev_err(lwis_dev->dev, "Failed to enable core clock\n");
			return ret;
		}
	}

	if (lwis_dev->bts_scenario_name) {
		lwis_dev->bts_scenario = bts_get_scenindex(lwis_dev->bts_scenario_name);
		if (!lwis_dev->bts_scenario) {
			dev_err(lwis_dev->dev, "Failed to get default camera BTS scenario.\n");
			return -EINVAL;
		}
		bts_add_scenario(lwis_dev->bts_scenario);
	}
	return 0;
}

int lwis_platform_device_disable(struct lwis_device *lwis_dev)
{
	int ret;
	struct lwis_platform *platform;
	BUG_ON(!lwis_dev);
	platform = lwis_dev->platform;
	if (!platform) {
		return -ENODEV;
	}

	if (lwis_dev->bts_scenario_name) {
		bts_del_scenario(lwis_dev->bts_scenario);
	}

	/* We can't remove fault handlers, so there's no call corresponding
	 * to the iovmm_set_fault_handler above */

	lwis_platform_remove_qos(lwis_dev);

	if (lwis_dev->has_iommu) {
		/* Deactivate IOMMU/SYSMMU */
		iovmm_deactivate(&lwis_dev->plat_dev->dev);
	}

	/* Disable platform device */
	ret = pm_runtime_put_sync(&lwis_dev->plat_dev->dev);

	return ret;
}

int lwis_platform_update_qos(struct lwis_device *lwis_dev, int value,
			     int32_t clock_family)
{
	struct lwis_platform *platform;
	struct pm_qos_request *qos_req;
	int qos_class;

	BUG_ON(!lwis_dev);
	platform = lwis_dev->platform;
	if (!platform) {
		return -ENODEV;
	}

	switch (clock_family) {
	case CLOCK_FAMILY_INTCAM:
		qos_req = &platform->pm_qos_int_cam;
		qos_class = PM_QOS_INTCAM_THROUGHPUT;
		break;
	case CLOCK_FAMILY_CAM:
		qos_req = &platform->pm_qos_cam;
		qos_class = PM_QOS_CAM_THROUGHPUT;
		break;
	case CLOCK_FAMILY_TNR:
#if defined(CONFIG_SOC_GS101)
		qos_req = &platform->pm_qos_tnr;
		qos_class = PM_QOS_TNR_THROUGHPUT;
#endif
		break;
	case CLOCK_FAMILY_MIF:
		qos_req = &platform->pm_qos_mem;
		qos_class = PM_QOS_BUS_THROUGHPUT;
		break;
	case CLOCK_FAMILY_INT:
		qos_req = &platform->pm_qos_int;
		qos_class = PM_QOS_DEVICE_THROUGHPUT;
		break;
	default:
		dev_err(lwis_dev->dev, "%s clk family %d is invalid\n", lwis_dev->name,
			lwis_dev->clock_family);
		return -EINVAL;
	}

	if (!pm_qos_request_active(qos_req))
		pm_qos_add_request(qos_req, qos_class, value);
	else
		pm_qos_update_request(qos_req, value);

	dev_info(lwis_dev->dev, "Updating clock for clock_family %d, freq to %u\n", clock_family,
		 value);

	return 0;
}

int lwis_platform_remove_qos(struct lwis_device *lwis_dev)
{
	struct lwis_platform *platform;
	BUG_ON(!lwis_dev);
	platform = lwis_dev->platform;
	if (!platform) {
		return -ENODEV;
	}

	if (pm_qos_request_active(&platform->pm_qos_int))
		pm_qos_remove_request(&platform->pm_qos_int);
	if (pm_qos_request_active(&platform->pm_qos_mem))
		pm_qos_remove_request(&platform->pm_qos_mem);
	if (pm_qos_request_active(&platform->pm_qos_hpg))
		pm_qos_remove_request(&platform->pm_qos_hpg);

	switch (lwis_dev->clock_family) {
	case CLOCK_FAMILY_INTCAM:
		if (pm_qos_request_active(&platform->pm_qos_int_cam)) {
			pm_qos_remove_request(&platform->pm_qos_int_cam);
		}
		break;
	case CLOCK_FAMILY_CAM:
		if (pm_qos_request_active(&platform->pm_qos_cam)) {
			pm_qos_remove_request(&platform->pm_qos_cam);
		}
		break;
	case CLOCK_FAMILY_TNR:
#if defined(CONFIG_SOC_GS101)
		if (pm_qos_request_active(&platform->pm_qos_tnr)) {
			pm_qos_remove_request(&platform->pm_qos_tnr);
		}
#endif
		break;
	default:
		break;
	}
	return 0;
}

int lwis_platform_update_bts(struct lwis_device *lwis_dev, unsigned int bw_kb_peak,
			     unsigned int bw_kb_read, unsigned int bw_kb_write)
{
	int ret = 0;
	struct bts_bw bts_request;

	if (lwis_dev->bts_index == BTS_UNSUPPORTED) {
		dev_info(lwis_dev->dev, "%s doesn't support bts\n", lwis_dev->name);
		return ret;
	}

	bts_request.peak = bw_kb_peak;
	bts_request.read = bw_kb_read;
	bts_request.write = bw_kb_write;
	ret = bts_update_bw(lwis_dev->bts_index, bts_request);
	if (ret < 0) {
		dev_err(lwis_dev->dev, "Failed to update bandwidth to bts, ret: %d\n", ret);
	} else {
		dev_info(lwis_dev->dev, "Updated bandwidth to bts, peak: %u, read: %u, write: %u\n",
			 bw_kb_peak, bw_kb_read, bw_kb_write);
	}

	return ret;
}
