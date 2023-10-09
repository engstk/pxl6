/*
 * Copyright (c) 2017 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#if 0
#include <linux/debug-snapshot.h>
#endif

#include <soc/google/cmu_ewf.h>

static void __iomem *cmu_cmu;
static spinlock_t cmuewf_lock;
static int ewf_refcnt[EWF_MAX_INDEX];

static struct clk_hw __maybe_unused ewf_clk;

int get_cmuewf_index(struct device_node *np, unsigned int *index)
{
	int len;
	const __be32 *prop;

	prop = of_get_property(np, "ewf-index", &len);
	if (!prop)
		return -ENOENT;
	*index = be32_to_cpup(prop);

	return 0;

}

static void __set_cmuewf(unsigned int index, unsigned int en)
{
	unsigned int reg;
	unsigned int reg_idx;

	if (index >= 32) {
		reg_idx = EARLY_WAKEUP_FORCED_ENABLE1;
		index -= 32;
	} else {
		reg_idx = EARLY_WAKEUP_FORCED_ENABLE0;
	}

	reg = __raw_readl(cmu_cmu + reg_idx);

	if (en)
		reg |= 1 << index;
	else
		reg &= ~(1 << index);

	__raw_writel(reg, cmu_cmu + reg_idx);
}

int set_cmuewf(unsigned int index, unsigned int en)
{
	int ret = 0;
	int tmp;
	unsigned long flags = 0;

	if (!cmu_cmu)
		return -EINVAL;

	spin_lock_irqsave(&cmuewf_lock, flags);
#if 0
	dbg_snapshot_clk(&ewf_clk, __func__, 1, DSS_FLAG_IN);
#endif

	if (wa_set_cmuewf) {
		ret = wa_set_cmuewf(index, en, cmu_cmu, ewf_refcnt);
		if (ret)
#if 0
			dbg_snapshot_clk(&ewf_clk, __func__, 1, DSS_FLAG_ON);
#endif
	} else {
		if (en) {
			__set_cmuewf(index, en);
			ewf_refcnt[index] += 1;
		} else {
			tmp = ewf_refcnt[index] - 1;

			if (tmp == 0) {
				__set_cmuewf(index, en);
			} else if (tmp < 0) {
				pr_err("[EWF]%s ref count mismatch. ewf_index:%u\n", __func__,  index);
#if 0
				dbg_snapshot_clk(&ewf_clk, __func__, 1, DSS_FLAG_ON);
#endif
				ret = -EINVAL;
				goto exit;
			}

			ewf_refcnt[index] -= 1;
		}
	}
#if 0
	dbg_snapshot_clk(&ewf_clk, __func__, 1, DSS_FLAG_OUT);
#endif
exit:
	spin_unlock_irqrestore(&cmuewf_lock, flags);

	return ret;
}

static int cmuewf_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct resource *res;

	dev_info(&pdev->dev, "cmuewf probe\n");

	if (!node) {
		dev_err(&pdev->dev, "driver doesn't support"
				"non-dt devices\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cmu_cmu = devm_ioremap_resource(&pdev->dev, res);

	spin_lock_init(&cmuewf_lock);

	return 0;
}

static int cmuewf_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id cmuewf_match[] = {
	{ .compatible = "samsung,exynos-cmuewf" },
	{},
};

static struct platform_driver samsung_cmuewf_driver = {
	.probe	= cmuewf_probe,
	.remove	= cmuewf_remove,
	.driver	= {
		.name = "exynos-cmuewf",
		.owner	= THIS_MODULE,
		.of_match_table	= cmuewf_match,
	},
};

static int early_wakeup_forced_enable_init(void)
{

	return platform_driver_register(&samsung_cmuewf_driver);
}

arch_initcall(early_wakeup_forced_enable_init);
