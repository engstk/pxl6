// SPDX-License-Identifier: GPL-2.0
/*
 * Abrolhos device driver for the Google EdgeTPU ML accelerator.
 *
 * Copyright (C) 2019 Google, Inc.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>

#include "abrolhos-platform.h"
#include "edgetpu-config.h"
#include "edgetpu-internal.h"
#include "edgetpu-mobile-platform.h"
#include "edgetpu-pm.h"

#include "edgetpu-mobile-platform.c"

static const struct of_device_id edgetpu_of_match[] = {
	{ .compatible = "google,edgetpu-gs101", },
	{ /* end of list */ },
};

MODULE_DEVICE_TABLE(of, edgetpu_of_match);


static int edgetpu_platform_probe(struct platform_device *pdev)
{
	struct abrolhos_platform_dev *abpdev;
	struct edgetpu_mobile_platform_dev *etmdev;

	abpdev = devm_kzalloc(&pdev->dev, sizeof(*abpdev), GFP_KERNEL);
	if (!abpdev)
		return -ENOMEM;

	etmdev = &abpdev->mobile_dev;
	return edgetpu_mobile_platform_probe(pdev, etmdev);
}

static int edgetpu_platform_remove(struct platform_device *pdev)
{
	return edgetpu_mobile_platform_remove(pdev);
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
	return platform_driver_register(&edgetpu_platform_driver);
}

static void __exit edgetpu_platform_exit(void)
{
	platform_driver_unregister(&edgetpu_platform_driver);
	edgetpu_exit();
}

MODULE_DESCRIPTION("Google EdgeTPU platform driver");
MODULE_LICENSE("GPL v2");
module_init(edgetpu_platform_init);
module_exit(edgetpu_platform_exit);
MODULE_FIRMWARE(EDGETPU_DEFAULT_FIRMWARE_NAME);
