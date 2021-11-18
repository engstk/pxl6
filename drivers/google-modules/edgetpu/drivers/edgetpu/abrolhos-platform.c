// SPDX-License-Identifier: GPL-2.0
/*
 * Abrolhos device driver for the Google EdgeTPU ML accelerator.
 *
 * Copyright (C) 2019 Google, Inc.
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/gsa/gsa_tpu.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_data/sscoredump.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "abrolhos-platform.h"
#include "abrolhos-pm.h"
#include "edgetpu-config.h"
#include "edgetpu-firmware.h"
#include "edgetpu-internal.h"
#include "edgetpu-iremap-pool.h"
#include "edgetpu-mmu.h"
#include "edgetpu-pm.h"
#include "edgetpu-telemetry.h"
#include "mobile-firmware.h"

static const struct of_device_id edgetpu_of_match[] = {
	/* TODO(b/190677977): remove  */
	{ .compatible = "google,darwinn", },
	{ .compatible = "google,edgetpu-gs101", },
	{ /* end of list */ },
};

MODULE_DEVICE_TABLE(of, edgetpu_of_match);

static void sscd_release(struct device *dev)
{
	pr_debug(DRIVER_NAME " release\n");
}
static struct sscd_platform_data sscd_pdata;
static struct platform_device sscd_dev = {
	.name            = DRIVER_NAME,
	.driver_override = SSCD_NAME,
	.id              = -1,
	.dev             = {
		.platform_data = &sscd_pdata,
		.release       = sscd_release,
	},
};

/*
 * Log and trace buffers at the beginning of the remapped region,
 * pool memory afterwards.
 */

#define EDGETPU_POOL_MEM_OFFSET (EDGETPU_TELEMETRY_BUFFER_SIZE * 2)

static void abrolhos_get_telemetry_mem(struct abrolhos_platform_dev *etpdev,
				       enum edgetpu_telemetry_type type,
				       struct edgetpu_coherent_mem *mem)
{
	int offset = type == EDGETPU_TELEMETRY_TRACE ?
				   EDGETPU_TELEMETRY_BUFFER_SIZE :
				   0;
	mem->vaddr = etpdev->shared_mem_vaddr + offset;
	mem->dma_addr = EDGETPU_REMAPPED_DATA_ADDR + offset;
	mem->tpu_addr = EDGETPU_REMAPPED_DATA_ADDR + offset;
	mem->host_addr = 0;
	mem->size = EDGETPU_TELEMETRY_BUFFER_SIZE;
}

/* Setup the firmware region carveout. */
static int
edgetpu_platform_setup_fw_region(struct abrolhos_platform_dev *etpdev)
{
	struct edgetpu_dev *etdev = &etpdev->edgetpu_dev;
	struct platform_device *gsa_pdev;
	struct device *dev = etdev->dev;
	struct resource r;
	struct device_node *np;
	int err;
	size_t region_map_size =
		EDGETPU_FW_SIZE_MAX + EDGETPU_REMAPPED_DATA_SIZE;

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np) {
		dev_err(dev, "No memory region for firmware\n");
		return -ENODEV;
	}

	err = of_address_to_resource(np, 0, &r);
	of_node_put(np);
	if (err) {
		dev_err(dev, "No memory address assigned to firmware region\n");
		return err;
	}

	if (resource_size(&r) < region_map_size) {
		dev_err(dev,
			"Memory region for firmware too small (%zu bytes needed, got %llu)\n",
			region_map_size, resource_size(&r));
		return -ENOSPC;
	}

	/* Get GSA device from device tree */
	np = of_parse_phandle(dev->of_node, "gsa-device", 0);
	if (!np) {
		dev_err(dev, "No gsa-dev in device tree\n");
		return -ENODEV;
	}
	gsa_pdev = of_find_device_by_node(np);
	if (!gsa_pdev) {
		dev_err(dev, "GSA device not found\n");
		of_node_put(np);
		return -ENODEV;
	}
	etpdev->gsa_dev = &gsa_pdev->dev;
	of_node_put(np);

	etpdev->fw_region_paddr = r.start;
	etpdev->fw_region_size = EDGETPU_FW_SIZE_MAX;

	etpdev->shared_mem_vaddr =
		memremap(r.start + EDGETPU_REMAPPED_DATA_OFFSET,
			 EDGETPU_REMAPPED_DATA_SIZE, MEMREMAP_WC);
	if (!etpdev->shared_mem_vaddr) {
		dev_err(dev, "Shared memory remap failed\n");
		return -EINVAL;
	}
	etpdev->shared_mem_paddr = r.start + EDGETPU_REMAPPED_DATA_OFFSET;

	return 0;
}

static void edgetpu_platform_cleanup_fw_region(
	struct abrolhos_platform_dev *etpdev)
{
	gsa_unload_tpu_fw_image(etpdev->gsa_dev);

	if (!etpdev->shared_mem_vaddr)
		return;
	memunmap(etpdev->shared_mem_vaddr);
	etpdev->shared_mem_vaddr = NULL;
}

int edgetpu_chip_setup_mmu(struct edgetpu_dev *etdev)
{
	int ret;

	/* No MMU info to pass to attach, IOMMU API will handle. */
	ret = edgetpu_mmu_attach(etdev, NULL);
	if (ret)
		dev_err(etdev->dev, "failed to attach IOMMU: %d\n", ret);
	return ret;
}

void edgetpu_chip_remove_mmu(struct edgetpu_dev *etdev)
{
	edgetpu_mmu_detach(etdev);
}

static int abrolhos_parse_ssmt(struct abrolhos_platform_dev *etpdev)
{
	struct edgetpu_dev *etdev = &etpdev->edgetpu_dev;
	struct platform_device *pdev = to_platform_device(etdev->dev);
	struct resource *res;
	int rc;
	void __iomem *ssmt_base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ssmt");
	if (!res) {
		etdev_warn(etdev, "Failed to find SSMT register base");
		return -EINVAL;
	}
	ssmt_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ssmt_base)) {
		rc = PTR_ERR(ssmt_base);
		etdev_warn(etdev, "Failed to map SSMT register base: %d\n", rc);
		return rc;
	}
	etpdev->ssmt_base = ssmt_base;
	return 0;
}

static int edgetpu_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct abrolhos_platform_dev *abpdev;
	struct resource *r;
	struct edgetpu_mapped_resource regs;
	int ret;
	struct edgetpu_iface_params iface_params[] = {
		/* Default interface  */
		{ .name = NULL },
		/* Common name for embedded SoC devices */
		{ .name = "edgetpu-soc" },
	};

	abpdev = devm_kzalloc(dev, sizeof(*abpdev), GFP_KERNEL);
	if (!abpdev)
		return -ENOMEM;

	mutex_init(&abpdev->tz_mailbox_lock);

	platform_set_drvdata(pdev, &abpdev->edgetpu_dev);
	abpdev->edgetpu_dev.dev = dev;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(r)) {
		dev_err(dev, "failed to get memory resource\n");
		return -ENODEV;
	}
	regs.phys = r->start;
	regs.size = resource_size(r);

	regs.mem = devm_ioremap_resource(dev, r);
	if (IS_ERR_OR_NULL(regs.mem)) {
		dev_err(dev, "failed to map registers\n");
		return -ENODEV;
	}

	mutex_init(&abpdev->platform_pwr.policy_lock);
	abpdev->platform_pwr.curr_policy = TPU_POLICY_MAX;

	ret = abrolhos_pm_create(&abpdev->edgetpu_dev);

	if (ret) {
		dev_err(dev, "Failed to initialize PM interface (%d)\n", ret);
		return ret;
	}

	ret = edgetpu_platform_setup_fw_region(abpdev);
	if (ret) {
		dev_err(dev, "%s setup fw regions failed: %d\n", DRIVER_NAME,
			ret);
		goto out_shutdown;
	}

	ret = edgetpu_iremap_pool_create(
		&abpdev->edgetpu_dev,
		/* Base virtual address (kernel address space) */
		abpdev->shared_mem_vaddr + EDGETPU_POOL_MEM_OFFSET,
		/* Base DMA address */
		EDGETPU_REMAPPED_DATA_ADDR + EDGETPU_POOL_MEM_OFFSET,
		/* Base TPU address */
		EDGETPU_REMAPPED_DATA_ADDR + EDGETPU_POOL_MEM_OFFSET,
		/* Base physical address */
		abpdev->shared_mem_paddr + EDGETPU_POOL_MEM_OFFSET,
		/* Size */
		EDGETPU_REMAPPED_DATA_SIZE - EDGETPU_POOL_MEM_OFFSET,
		/* Granularity */
		PAGE_SIZE);
	if (ret) {
		dev_err(dev,
			"%s failed to initialize remapped memory pool: %d\n",
			DRIVER_NAME, ret);
		goto out_cleanup_fw;
	}

	abpdev->edgetpu_dev.mcp_id = -1;
	abpdev->edgetpu_dev.mcp_die_index = 0;
	abpdev->irq = platform_get_irq(pdev, 0);
	ret = edgetpu_device_add(&abpdev->edgetpu_dev, &regs, iface_params,
				 ARRAY_SIZE(iface_params));

	if (!ret && abpdev->irq >= 0)
		ret = edgetpu_register_irq(&abpdev->edgetpu_dev, abpdev->irq);

	if (ret) {
		dev_err(dev, "%s edgetpu setup failed: %d\n", DRIVER_NAME,
			ret);
		goto out_destroy_iremap;
	}

	ret = abrolhos_parse_ssmt(abpdev);
	if (ret)
		dev_warn(
			dev,
			"SSMT setup failed (%d). Context isolation not enforced\n",
			ret);

	abrolhos_get_telemetry_mem(abpdev, EDGETPU_TELEMETRY_LOG,
				   &abpdev->log_mem);
	abrolhos_get_telemetry_mem(abpdev, EDGETPU_TELEMETRY_TRACE,
				   &abpdev->trace_mem);

	ret = edgetpu_telemetry_init(&abpdev->edgetpu_dev, &abpdev->log_mem,
				     &abpdev->trace_mem);
	if (ret)
		goto out_remove_device;

	ret = mobile_edgetpu_firmware_create(&abpdev->edgetpu_dev);
	if (ret) {
		dev_err(dev,
			"%s initialize firmware downloader failed: %d\n",
			DRIVER_NAME, ret);
		goto out_tel_exit;
	}

	dev_dbg(dev, "Creating thermal device\n");
	abpdev->edgetpu_dev.thermal =
			devm_tpu_thermal_create(dev, &abpdev->edgetpu_dev);

	dev_info(dev, "%s edgetpu initialized. Build: %s\n",
		 abpdev->edgetpu_dev.dev_name, GIT_REPO_TAG);

	dev_dbg(dev, "Probe finished, powering down\n");
	/* Turn the device off unless a client request is already received. */
	edgetpu_pm_shutdown(&abpdev->edgetpu_dev, false);

	abpdev->sscd_info.pdata = &sscd_pdata;
	abpdev->sscd_info.dev = &sscd_dev;

	return ret;
out_tel_exit:
	edgetpu_telemetry_exit(&abpdev->edgetpu_dev);
out_remove_device:
	edgetpu_device_remove(&abpdev->edgetpu_dev);
out_destroy_iremap:
	edgetpu_iremap_pool_destroy(&abpdev->edgetpu_dev);
out_cleanup_fw:
	edgetpu_platform_cleanup_fw_region(abpdev);
out_shutdown:
	dev_dbg(dev, "Probe finished with error %d, powering down\n", ret);
	edgetpu_pm_shutdown(&abpdev->edgetpu_dev, true);
	return ret;
}

static int edgetpu_platform_remove(struct platform_device *pdev)
{
	struct edgetpu_dev *etdev = platform_get_drvdata(pdev);
	struct abrolhos_platform_dev *abpdev = to_abrolhos_dev(etdev);

	mobile_edgetpu_firmware_destroy(etdev);
	if (abpdev->irq >= 0)
		edgetpu_unregister_irq(etdev, abpdev->irq);

	edgetpu_pm_get(etdev->pm);
	edgetpu_telemetry_exit(etdev);
	edgetpu_device_remove(etdev);
	edgetpu_iremap_pool_destroy(etdev);
	edgetpu_platform_cleanup_fw_region(abpdev);
	edgetpu_pm_put(etdev->pm);
	edgetpu_pm_shutdown(etdev, true);
	abrolhos_pm_destroy(etdev);
	return 0;
}

#if IS_ENABLED(CONFIG_PM_SLEEP)

static int edgetpu_platform_suspend(struct device *dev)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_pm_suspend(etdev);
}

static int edgetpu_platform_resume(struct device *dev)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);

	return edgetpu_pm_resume(etdev);
}

#endif /* IS_ENABLED(CONFIG_PM_SLEEP) */

static const struct dev_pm_ops edgetpu_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(edgetpu_platform_suspend,
				edgetpu_platform_resume)
};

static struct platform_driver edgetpu_platform_driver = {
	.probe = edgetpu_platform_probe,
	.remove = edgetpu_platform_remove,
	.driver = {
		.name = "edgetpu_platform",
		.of_match_table = edgetpu_of_match,
		.pm = &edgetpu_pm_ops,
	},
};

static int __init edgetpu_platform_init(void)
{
	int ret;

	ret = edgetpu_init();
	if (ret)
		return ret;

	/* Register SSCD platform device */
	ret = platform_device_register(&sscd_dev);
	if (ret)
		pr_err(DRIVER_NAME " SSCD platform device registration failed: %d\n",
		       ret);
	return platform_driver_register(&edgetpu_platform_driver);
}

static void __exit edgetpu_platform_exit(void)
{
	platform_driver_unregister(&edgetpu_platform_driver);
	platform_device_unregister(&sscd_dev);
	edgetpu_exit();
}

MODULE_DESCRIPTION("Google EdgeTPU platform driver");
MODULE_LICENSE("GPL v2");
module_init(edgetpu_platform_init);
module_exit(edgetpu_platform_exit);
MODULE_FIRMWARE(EDGETPU_DEFAULT_FIRMWARE_NAME);
