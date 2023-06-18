/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2021 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <misc/gvotable.h>
#include "gbms_power_supply.h"
#include "google_bms.h"
#include "google_psy.h"

#define MAX(x, y)	((x) < (y) ? (y) : (x))
#define DUAL_FG_DELAY_INIT_MS	500
#define DUAL_FG_WORK_PERIOD_MS	10000
#define DUAL_BATT_TEMP_VOTER	"daul_batt_temp"

#define DUAL_BATT_VFLIP_OFFSET		50000
#define DUAL_BATT_VFLIP_OFFSET_IDX	0

struct dual_fg_drv {
	struct device *device;
	struct power_supply *psy;

	const char *first_fg_psy_name;
	const char *second_fg_psy_name;

	struct power_supply *first_fg_psy;
	struct power_supply *second_fg_psy;

	struct mutex fg_lock;

	struct delayed_work init_work;
	struct delayed_work gdbatt_work;
	struct gvotable_election *fcc_votable;

	struct gbms_chg_profile chg_profile;

	u32 battery_capacity;
	int cc_max;

	int base_charge_full;
	int flip_charge_full;

	bool init_complete;
	bool resume_complete;
	bool cable_in;

	u32 vflip_offset;
};

static int gdbatt_resume_check(struct dual_fg_drv *dual_fg_drv) {
	int ret = 0;

	pm_runtime_get_sync(dual_fg_drv->device);
	if (!dual_fg_drv->init_complete || !dual_fg_drv->resume_complete)
		ret = -EAGAIN;
	pm_runtime_put_sync(dual_fg_drv->device);

	return ret;
}

static enum power_supply_property gdbatt_fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,		/* replace with _RAW */
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,	/* used from gbattery */
	POWER_SUPPLY_PROP_CURRENT_AVG,		/* candidate for tier switch */
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static int gdbatt_get_temp(struct power_supply *fg_psy, int *temp)
{
	int err = 0;
	union power_supply_propval val;

	if (!fg_psy)
		return -EINVAL;

	err = power_supply_get_property(fg_psy,
					POWER_SUPPLY_PROP_TEMP, &val);
	if (err == 0)
		*temp = val.intval;

	return err;
}

static int gdbatt_select_temp_idx(struct gbms_chg_profile *profile, int temp)
{
	if (temp < profile->temp_limits[0] ||
	    temp > profile->temp_limits[profile->temp_nb_limits - 1])
		return -1;
	else
		return gbms_msc_temp_idx(profile, temp);
}

static int gdbatt_select_voltage_idx(struct gbms_chg_profile *profile, int vbatt)
{
	int vbatt_idx = 0;

	while (vbatt_idx < profile->volt_nb_limits - 1 &&
	       vbatt > profile->volt_limits[vbatt_idx])
		vbatt_idx++;

	return vbatt_idx;
}

static void gdbatt_select_cc_max(struct dual_fg_drv *dual_fg_drv)
{
	struct gbms_chg_profile *profile = &dual_fg_drv->chg_profile;
	int base_temp, flip_temp, base_vbatt, flip_vbatt;
	int base_temp_idx, flip_temp_idx, base_vbatt_idx, flip_vbatt_idx;
	int base_cc_max, flip_cc_max, cc_max;
	struct power_supply *base_psy = dual_fg_drv->first_fg_psy;
	struct power_supply *flip_psy = dual_fg_drv->second_fg_psy;
	int ret = 0;

	if (!dual_fg_drv->cable_in)
		goto check_done;

	ret = gdbatt_get_temp(base_psy, &base_temp);
	if (ret < 0)
		goto check_done;

	ret = gdbatt_get_temp(flip_psy, &flip_temp);
	if (ret < 0)
		goto check_done;

	base_vbatt = GPSY_GET_PROP(base_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	if (base_vbatt < 0)
		goto check_done;

	flip_vbatt = GPSY_GET_PROP(flip_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	if (flip_vbatt < 0)
		goto check_done;

	base_temp_idx = gdbatt_select_temp_idx(profile, base_temp);
	flip_temp_idx = gdbatt_select_temp_idx(profile, flip_temp);

	base_vbatt_idx = gdbatt_select_voltage_idx(profile, base_vbatt);
	flip_vbatt_idx = gdbatt_select_voltage_idx(profile,
					  flip_vbatt - dual_fg_drv->vflip_offset);

	/* only apply offset in allowed idx */
	if (flip_vbatt_idx > DUAL_BATT_VFLIP_OFFSET_IDX)
		flip_vbatt_idx = gdbatt_select_voltage_idx(profile, flip_vbatt);

	base_cc_max = GBMS_CCCM_LIMITS(profile, base_temp_idx, base_vbatt_idx);
	flip_cc_max = GBMS_CCCM_LIMITS(profile, flip_temp_idx, flip_vbatt_idx);
	cc_max = (base_cc_max <= flip_cc_max) ? base_cc_max : flip_cc_max;
	if (cc_max == dual_fg_drv->cc_max)
		goto check_done;

	if (!dual_fg_drv->fcc_votable)
		dual_fg_drv->fcc_votable =
			gvotable_election_get_handle(VOTABLE_MSC_FCC);
	if (dual_fg_drv->fcc_votable) {
		pr_info("temp:%d/%d(%d/%d), vbatt:%d/%d(%d/%d), cc_max:%d/%d(%d)\n",
			base_temp, flip_temp, base_temp_idx, flip_temp_idx, base_vbatt,
			flip_vbatt, base_vbatt_idx, flip_vbatt_idx, base_cc_max,
			flip_cc_max, cc_max);
		gvotable_cast_int_vote(dual_fg_drv->fcc_votable,
				       DUAL_BATT_TEMP_VOTER, cc_max, true);
		dual_fg_drv->cc_max = cc_max;
		power_supply_changed(dual_fg_drv->psy);
	}

check_done:
	pr_debug("check done. cable_in=%d (%d)\n", dual_fg_drv->cable_in, ret);
}

static int gdbatt_get_capacity(struct dual_fg_drv *dual_fg_drv, int base_soc, int flip_soc)
{
	const int base_full = dual_fg_drv->base_charge_full / 1000;
	const int flip_full = dual_fg_drv->flip_charge_full / 1000;
	const int full_sum = base_full + flip_full;

	if (!base_full || !flip_full)
		return (base_soc + flip_soc) / 2;

	return (base_soc * base_full + flip_soc * flip_full) / full_sum;
}

static void google_dual_batt_work(struct work_struct *work)
{
	struct dual_fg_drv *dual_fg_drv = container_of(work, struct dual_fg_drv,
						 gdbatt_work.work);
	struct power_supply *base_psy = dual_fg_drv->first_fg_psy;
	struct power_supply *flip_psy = dual_fg_drv->second_fg_psy;
	int base_data, flip_data;

	mutex_lock(&dual_fg_drv->fg_lock);

	if (!dual_fg_drv->init_complete)
		goto error_done;

	if (!base_psy || !flip_psy)
		goto error_done;

	gdbatt_select_cc_max(dual_fg_drv);

	base_data = GPSY_GET_PROP(base_psy, POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN);
	flip_data = GPSY_GET_PROP(flip_psy, POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN);

	if (base_data <= 0 || flip_data <= 0)
		goto error_done;

	dual_fg_drv->base_charge_full = base_data;
	dual_fg_drv->flip_charge_full = flip_data;

error_done:
	mod_delayed_work(system_wq, &dual_fg_drv->gdbatt_work,
			 msecs_to_jiffies(DUAL_FG_WORK_PERIOD_MS));

	mutex_unlock(&dual_fg_drv->fg_lock);
}

static int gdbatt_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct dual_fg_drv *dual_fg_drv = (struct dual_fg_drv *)
					power_supply_get_drvdata(psy);
	int err = 0;
	union power_supply_propval fg_1;
	union power_supply_propval fg_2;

	err = gdbatt_resume_check(dual_fg_drv);
	if (err != 0)
		return err;

	if (!dual_fg_drv->first_fg_psy && !dual_fg_drv->second_fg_psy)
		return -EAGAIN;

	if (!dual_fg_drv->first_fg_psy || !dual_fg_drv->second_fg_psy)
		goto single_fg;

	mutex_lock(&dual_fg_drv->fg_lock);

	err = power_supply_get_property(dual_fg_drv->first_fg_psy, psp, &fg_1);
	if (err != 0) {
		pr_debug("error %d reading first fg prop %d\n", err, psp);
		mutex_unlock(&dual_fg_drv->fg_lock);
		return err;
	}

	err = power_supply_get_property(dual_fg_drv->second_fg_psy, psp, &fg_2);
	if (err != 0) {
		pr_debug("error %d reading second fg prop %d\n", err, psp);
		mutex_unlock(&dual_fg_drv->fg_lock);
		return err;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	case POWER_SUPPLY_PROP_CURRENT_AVG:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = fg_1.intval + fg_2.intval;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = MAX(fg_1.intval, fg_2.intval);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		val->intval = MAX(fg_1.intval, fg_2.intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = (fg_1.intval + fg_2.intval)/2;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = (fg_1.intval + fg_2.intval)/2;
		break;
	case GBMS_PROP_CAPACITY_RAW:
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = gdbatt_get_capacity(dual_fg_drv, fg_1.intval, fg_2.intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		/* larger one is bad. TODO: confirm its priority */
		val->intval = MAX(fg_1.intval, fg_2.intval);
		break;
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = fg_1.intval;
		if (fg_1.intval != fg_2.intval)
			pr_debug("case %d not align: %d/%d",
				psp, fg_1.intval, fg_2.intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = fg_1.intval && fg_2.intval;
		if (fg_1.intval != fg_2.intval)
			pr_debug("PRESENT different: %d/%d", fg_1.intval, fg_2.intval);
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		/* TODO: need hash SN */
		val->strval = fg_1.strval;
		break;
	default:
		pr_debug("getting unsupported property: %d\n", psp);
		break;
	}

	mutex_unlock(&dual_fg_drv->fg_lock);

	return 0;

single_fg:
	mutex_lock(&dual_fg_drv->fg_lock);
	if (dual_fg_drv->first_fg_psy)
		err = power_supply_get_property(dual_fg_drv->first_fg_psy, psp, val);
	else if (dual_fg_drv->second_fg_psy)
		err = power_supply_get_property(dual_fg_drv->second_fg_psy, psp, val);
	mutex_unlock(&dual_fg_drv->fg_lock);

	if (err < 0)
		pr_debug("error %d reading single prop %d\n", err, psp);

	return 0;
}

static int gdbatt_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *val)
{
	struct dual_fg_drv *dual_fg_drv = (struct dual_fg_drv *)
					power_supply_get_drvdata(psy);
	int ret = 0;

	ret = gdbatt_resume_check(dual_fg_drv);
	if (ret != 0)
		return ret;

	switch (psp) {
	case GBMS_PROP_BATT_CE_CTRL:
		if (dual_fg_drv->first_fg_psy) {
			ret = GPSY_SET_PROP(dual_fg_drv->first_fg_psy, psp, val->intval);
			if (ret < 0)
				pr_err("Cannot set the first BATT_CE_CTRL, ret=%d\n", ret);
		}
		if (dual_fg_drv->second_fg_psy) {
			ret = GPSY_SET_PROP(dual_fg_drv->second_fg_psy, psp, val->intval);
			if (ret < 0)
				pr_err("Cannot set the second BATT_CE_CTRL, ret=%d\n", ret);
		}
		dual_fg_drv->cable_in = !!val->intval;
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0) {
		pr_debug("gdbatt: set_prop cannot write psp=%d\n", psp);
		return ret;
	}

	return 0;
}

static int gdbatt_property_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	switch (psp) {
	case GBMS_PROP_BATT_CE_CTRL:
		return 1;
	default:
		break;
	}

	return 0;
}

static struct power_supply_desc gdbatt_psy_desc = {
	.name = "dualbatt",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.get_property = gdbatt_get_property,
	.set_property = gdbatt_set_property,
	.property_is_writeable = gdbatt_property_is_writeable,
	.properties = gdbatt_fg_props,
	.num_properties = ARRAY_SIZE(gdbatt_fg_props),
};

/* ------------------------------------------------------------------------ */

static int gdbatt_init_chg_profile(struct dual_fg_drv *dual_fg_drv)
{
	struct device_node *node = of_find_node_by_name(NULL, "google,battery");
	struct gbms_chg_profile *profile = &dual_fg_drv->chg_profile;
	int ret = 0;

	if (profile->cccm_limits)
		return 0;

	ret = gbms_init_chg_profile(profile, node);
	if (ret < 0)
		return -EINVAL;

	ret = of_property_read_u32(node, "google,chg-battery-capacity",
				   &dual_fg_drv->battery_capacity);
	if (ret < 0)
		pr_warn("battery not present, no default capacity, zero charge table\n");

	gbms_init_chg_table(profile, node, dual_fg_drv->battery_capacity);

	return ret;
}

static void google_dual_batt_gauge_init_work(struct work_struct *work)
{
	struct dual_fg_drv *dual_fg_drv = container_of(work, struct dual_fg_drv,
						 init_work.work);
	struct power_supply *first_fg_psy = dual_fg_drv->first_fg_psy;
	struct power_supply *second_fg_psy = dual_fg_drv->second_fg_psy;
	union power_supply_propval val;
	int err = 0;

	if (!dual_fg_drv->first_fg_psy && dual_fg_drv->first_fg_psy_name) {
		first_fg_psy = power_supply_get_by_name(dual_fg_drv->first_fg_psy_name);
		if (!first_fg_psy) {
			dev_info(dual_fg_drv->device,
				"failed to get \"%s\" power supply, retrying...\n",
				dual_fg_drv->first_fg_psy_name);
			goto retry_init_work;
		}

		dual_fg_drv->first_fg_psy = first_fg_psy;

		/* Don't use it if battery not present */
		err = power_supply_get_property(first_fg_psy,
						POWER_SUPPLY_PROP_PRESENT, &val);
		if (err == -EAGAIN)
			goto retry_init_work;
		if (err == 0 && val.intval == 0) {
			dev_info(dual_fg_drv->device, "First battery not PRESENT\n");
			dual_fg_drv->first_fg_psy_name = NULL;
			dual_fg_drv->first_fg_psy = NULL;
		}
	}

	if (!dual_fg_drv->second_fg_psy && dual_fg_drv->second_fg_psy_name) {
		second_fg_psy = power_supply_get_by_name(dual_fg_drv->second_fg_psy_name);
		if (!second_fg_psy) {
			pr_info("failed to get \"%s\" power supply, retrying...\n",
				dual_fg_drv->second_fg_psy_name);
			goto retry_init_work;
		}

		dual_fg_drv->second_fg_psy = second_fg_psy;

		/* Don't use it if battery not present */
		err = power_supply_get_property(second_fg_psy,
						POWER_SUPPLY_PROP_PRESENT, &val);
		if (err == -EAGAIN)
			goto retry_init_work;
		if (err == 0 && val.intval == 0) {
			dev_info(dual_fg_drv->device, "Second battery not PRESENT\n");
			dual_fg_drv->second_fg_psy_name = NULL;
			dual_fg_drv->second_fg_psy = NULL;
		}
	}

	dual_fg_drv->cc_max = -1;
	err = gdbatt_init_chg_profile(dual_fg_drv);
	if (err < 0)
		dev_info(dual_fg_drv->device,"fail to init chg profile (%d)\n", err);

	dual_fg_drv->init_complete = true;
	dual_fg_drv->resume_complete = true;
	mod_delayed_work(system_wq, &dual_fg_drv->gdbatt_work, 0);
	dev_info(dual_fg_drv->device, "google_dual_batt_gauge_init_work done\n");

	return;

retry_init_work:
	schedule_delayed_work(&dual_fg_drv->init_work,
			      msecs_to_jiffies(DUAL_FG_DELAY_INIT_MS));
}

static int google_dual_batt_gauge_probe(struct platform_device *pdev)
{
	const char *first_fg_psy_name;
	const char *second_fg_psy_name;
	struct dual_fg_drv *dual_fg_drv;
	struct power_supply_config psy_cfg = {};
	int ret;

	dual_fg_drv = devm_kzalloc(&pdev->dev, sizeof(*dual_fg_drv), GFP_KERNEL);
	if (!dual_fg_drv)
		return -ENOMEM;

	dual_fg_drv->device = &pdev->dev;

	ret = of_property_read_string(pdev->dev.of_node, "google,first-fg-psy-name",
				      &first_fg_psy_name);
	if (ret == 0) {
		pr_info("google,first-fg-psy-name=%s\n", first_fg_psy_name);
		dual_fg_drv->first_fg_psy_name = devm_kstrdup(&pdev->dev,
							first_fg_psy_name, GFP_KERNEL);
		if (!dual_fg_drv->first_fg_psy_name)
			return -ENOMEM;
	}

	ret = of_property_read_string(pdev->dev.of_node, "google,second-fg-psy-name",
				      &second_fg_psy_name);
	if (ret == 0) {
		pr_info("google,second-fg-psy-name=%s\n", second_fg_psy_name);
		dual_fg_drv->second_fg_psy_name = devm_kstrdup(&pdev->dev, second_fg_psy_name,
							GFP_KERNEL);
		if (!dual_fg_drv->second_fg_psy_name)
			return -ENOMEM;
	}

	if (!dual_fg_drv->first_fg_psy_name && !dual_fg_drv->second_fg_psy_name) {
		pr_err("no dual gauge setting\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "google,vflip-offset",
				   &dual_fg_drv->vflip_offset);
	if (ret < 0) {
		pr_debug("Couldn't set vflip_offset (%d)\n", ret);
		dual_fg_drv->vflip_offset = DUAL_BATT_VFLIP_OFFSET;
	}

	INIT_DELAYED_WORK(&dual_fg_drv->init_work, google_dual_batt_gauge_init_work);
	INIT_DELAYED_WORK(&dual_fg_drv->gdbatt_work, google_dual_batt_work);
	mutex_init(&dual_fg_drv->fg_lock);
	platform_set_drvdata(pdev, dual_fg_drv);

	psy_cfg.drv_data = dual_fg_drv;
	psy_cfg.of_node = pdev->dev.of_node;

	if (of_property_read_bool(pdev->dev.of_node, "google,psy-type-unknown"))
		gdbatt_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;

	dual_fg_drv->psy = devm_power_supply_register(dual_fg_drv->device,
						   &gdbatt_psy_desc, &psy_cfg);
	if (IS_ERR(dual_fg_drv->psy)) {
		ret = PTR_ERR(dual_fg_drv->psy);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		/* TODO: fail with -ENODEV */
		dev_err(dual_fg_drv->device,
			"Couldn't register as power supply, ret=%d\n", ret);
	}

	schedule_delayed_work(&dual_fg_drv->init_work,
					msecs_to_jiffies(DUAL_FG_DELAY_INIT_MS));

	pr_info("google_dual_batt_gauge_probe done\n");

	return 0;
}

static int google_dual_batt_gauge_remove(struct platform_device *pdev)
{
	struct dual_fg_drv *dual_fg_drv = platform_get_drvdata(pdev);

	gbms_free_chg_profile(&dual_fg_drv->chg_profile);

	return 0;
}

static int __maybe_unused google_dual_batt_pm_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dual_fg_drv *dual_fg_drv = platform_get_drvdata(pdev);

	pm_runtime_get_sync(dual_fg_drv->device);
	dual_fg_drv->resume_complete = false;
	pm_runtime_put_sync(dual_fg_drv->device);

	return 0;
}

static int __maybe_unused google_dual_batt_pm_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dual_fg_drv *dual_fg_drv = platform_get_drvdata(pdev);

	pm_runtime_get_sync(dual_fg_drv->device);
	dual_fg_drv->resume_complete = true;
	pm_runtime_put_sync(dual_fg_drv->device);

	return 0;
}

static SIMPLE_DEV_PM_OPS(google_dual_batt_pm_ops,
			 google_dual_batt_pm_suspend,
			 google_dual_batt_pm_resume);

static const struct of_device_id google_dual_batt_gauge_of_match[] = {
	{.compatible = "google,dual_batt_gauge"},
	{},
};
MODULE_DEVICE_TABLE(of, google_dual_batt_gauge_of_match);

static struct platform_driver google_dual_batt_gauge_driver = {
	.driver = {
		   .name = "google,dual_batt_gauge",
		   .owner = THIS_MODULE,
		   .of_match_table = google_dual_batt_gauge_of_match,
		   .probe_type = PROBE_PREFER_ASYNCHRONOUS,
		   .pm = &google_dual_batt_pm_ops,
		   },
	.probe = google_dual_batt_gauge_probe,
	.remove = google_dual_batt_gauge_remove,
};

module_platform_driver(google_dual_batt_gauge_driver);

MODULE_DESCRIPTION("Google Dual Gauge Driver");
MODULE_AUTHOR("Jenny Ho <hsiufangho@google.com>");
MODULE_LICENSE("GPL");
