/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2020 Google, LLC
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

#include <linux/ctype.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/thermal.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/pmic_class.h>
#include <misc/gvotable.h>
#include "gbms_power_supply.h"
#include "google_bms.h"
#include "max_m5.h"
#include "max77759.h"
#include <soc/google/bcl.h>

#define VD_BATTERY_VOLTAGE 4200
#define VD_UPPER_LIMIT 3350
#define VD_LOWER_LIMIT 2600
#define VD_STEP 50
#define VD_DELAY 300
#define BO_UPPER_LIMIT 6800
#define BO_LOWER_LIMIT 3800
#define BO_STEP 200
#define THERMAL_IRQ_COUNTER_LIMIT 5
#define THERMAL_HYST_LEVEL 100
#define BATOILO_DET_30US 0x4
#define MAX77759_DEFAULT_MODE	MAX77759_CHGR_MODE_ALL_OFF
#define CHG_TERM_VOLT_DEBOUNCE	200

/* CHG_DETAILS_01:CHG_DTLS */
#define CHGR_DTLS_DEAD_BATTERY_MODE			0x00
#define CHGR_DTLS_FAST_CHARGE_CONST_CURRENT_MODE	0x01
#define CHGR_DTLS_FAST_CHARGE_CONST_VOLTAGE_MODE	0x02
#define CHGR_DTLS_TOP_OFF_MODE				0x03
#define CHGR_DTLS_DONE_MODE				0x04
#define CHGR_DTLS_TIMER_FAULT_MODE			0x06
#define CHGR_DTLS_DETBAT_HIGH_SUSPEND_MODE		0x07
#define CHGR_DTLS_OFF_MODE				0x08
#define CHGR_DTLS_OFF_HIGH_TEMP_MODE			0x0a
#define CHGR_DTLS_OFF_WATCHDOG_MODE			0x0b
#define CHGR_DTLS_OFF_JEITA				0x0c
#define CHGR_DTLS_OFF_TEMP				0x0d

struct max77759_chgr_data {
	struct device *dev;

	struct power_supply *psy;
	struct power_supply *wcin_psy;
	struct power_supply *chgin_psy;

	struct power_supply *wlc_psy;
	struct regmap *regmap;

	struct gvotable_election *mode_votable;
	struct max77759_usecase_data uc_data;

	struct gvotable_election *dc_icl_votable;
	struct gvotable_election *dc_suspend_votable;

	bool charge_done;
	bool chgin_input_suspend;
	bool wcin_input_suspend;

	int irq_gpio;

	struct i2c_client *fg_i2c_client;
	struct i2c_client *pmic_i2c_client;

	struct dentry *de;

	atomic_t insel_cnt;
	bool insel_clear;	/* when set, irq clears CHGINSEL_MASK */

	atomic_t early_topoff_cnt;

	struct mutex io_lock;
	bool resume_complete;
	bool init_complete;
	struct wakeup_source *usecase_wake_lock;

	int fship_dtls;
	bool online;
	bool wden;

	/* debug interface, register to read or write */
	u32 debug_reg_address;

	/* thermal BCL */
#if IS_ENABLED(CONFIG_GOOGLE_BCL)
	struct thermal_zone_device *tz_miti[IFPMIC_SENSOR_MAX];
	int triggered_counter[IFPMIC_SENSOR_MAX];
	unsigned int triggered_lvl[IFPMIC_SENSOR_MAX];
	unsigned int triggered_irq[IFPMIC_SENSOR_MAX];
	struct mutex triggered_irq_lock[IFPMIC_SENSOR_MAX];
	struct delayed_work triggered_irq_work[IFPMIC_SENSOR_MAX];
	struct bcl_device *bcl_dev;
#endif

	int chg_term_voltage;
	int chg_term_volt_debounce;
};

static inline int max77759_reg_read(struct regmap *regmap, uint8_t reg,
				    uint8_t *val)
{
	int ret, ival;

	ret = regmap_read(regmap, reg, &ival);
	if (ret == 0)
		*val = 0xFF & ival;

	return ret;
}

static inline int max77759_reg_write(struct regmap *regmap, uint8_t reg,
				     uint8_t val)
{
	return regmap_write(regmap, reg, val);
}

static inline int max77759_readn(struct regmap *regmap, uint8_t reg,
				 uint8_t *val, int count)
{
	return regmap_bulk_read(regmap, reg, val, count);
}

static inline int max77759_writen(struct regmap *regmap, uint8_t reg,
				  const uint8_t *val, int count)
{
	return regmap_bulk_write(regmap, reg, val, count);
}

static inline int max77759_reg_update(struct max77759_chgr_data *data,
				      uint8_t reg, uint8_t msk, uint8_t val)
{
	int ret;
	unsigned tmp;

	mutex_lock(&data->io_lock);
	ret = regmap_read(data->regmap, reg, &tmp);
	if (!ret) {
		tmp &= ~msk;
		tmp |= val;
		ret = regmap_write(data->regmap, reg, tmp);
	}
	mutex_unlock(&data->io_lock);

	return ret;
}

/* ----------------------------------------------------------------------- */

int max77759_chg_reg_write(struct i2c_client *client, u8 reg, u8 value)
{
	struct max77759_chgr_data *data;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	return max77759_reg_write(data->regmap, reg, value);
}
EXPORT_SYMBOL_GPL(max77759_chg_reg_write);

int max77759_chg_reg_read(struct i2c_client *client, u8 reg, u8 *value)
{
	struct max77759_chgr_data *data;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	return max77759_reg_read(data->regmap, reg, value);
}
EXPORT_SYMBOL_GPL(max77759_chg_reg_read);

int max77759_chg_reg_update(struct i2c_client *client,
			    u8 reg, u8 mask, u8 value)
{
	struct max77759_chgr_data *data;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	return regmap_write_bits(data->regmap, reg, mask, value);
}
EXPORT_SYMBOL_GPL(max77759_chg_reg_update);

int max77759_chg_mode_write(struct i2c_client *client,
			    enum max77759_charger_modes mode)
{
	struct max77759_chgr_data *data;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	return regmap_write_bits(data->regmap, MAX77759_CHG_CNFG_00,
				 MAX77759_CHG_CNFG_00_MODE_MASK,
				 mode);
}
EXPORT_SYMBOL_GPL(max77759_chg_mode_write);

/* 1 if changed, 0 if not changed, or < 0 on error */
static int max77759_chg_prot(struct regmap *regmap, bool enable)
{
	u8 value = enable ? 0 : MAX77759_CHG_CNFG_06_CHGPROT_MASK;
	u8 prot;
	int ret;

	ret = max77759_reg_read(regmap, MAX77759_CHG_CNFG_06, &prot);
	if (ret < 0)
		return -EIO;

	if ((prot & MAX77759_CHG_CNFG_06_CHGPROT_MASK) == value)
		return 0;

	ret = regmap_write_bits(regmap, MAX77759_CHG_CNFG_06,
				MAX77759_CHG_CNFG_06_CHGPROT_MASK,
				value);
	if (ret < 0)
		return -EIO;

	return 1;
}

int max77759_chg_insel_write(struct i2c_client *client, u8 mask, u8 value)
{
	struct max77759_chgr_data *data;
	int ret, prot;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	prot = max77759_chg_prot(data->regmap, false);
	if (prot < 0)
		return -EIO;

	/* changing [CHGIN|WCIN]_INSEL: works when protection is disabled  */
	ret = regmap_write_bits(data->regmap, MAX77759_CHG_CNFG_12, mask, value);
	if (ret < 0 || prot == 0)
		return ret;

	prot = max77759_chg_prot(data->regmap, true);
	if (prot < 0) {
		pr_err("%s: cannot restore protection bits (%d)\n",
		       __func__, prot);
		return prot;
	};

	return ret;
}
EXPORT_SYMBOL_GPL(max77759_chg_insel_write);

int max77759_chg_insel_read(struct i2c_client *client, u8 *value)
{
	struct max77759_chgr_data *data;

	if (!client)
		return -ENODEV;

	data = i2c_get_clientdata(client);
	if (!data || !data->regmap)
		return -ENODEV;

	return  max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_12, value);
}
EXPORT_SYMBOL_GPL(max77759_chg_insel_read);

/* ----------------------------------------------------------------------- */

static int max77759_find_pmic(struct max77759_chgr_data *data)
{
	struct device_node *dn;

	if (data->pmic_i2c_client)
		return 0;

	dn = of_parse_phandle(data->dev->of_node, "max77759,pmic", 0);
	if (!dn)
		return -ENXIO;

	data->pmic_i2c_client = of_find_i2c_device_by_node(dn);
	if (!data->pmic_i2c_client)
		return -EAGAIN;

	return 0;
}

static int max77759_find_fg(struct max77759_chgr_data *data)
{
	struct device_node *dn;

	if (data->fg_i2c_client)
		return 0;

	dn = of_parse_phandle(data->dev->of_node, "max77759,max_m5", 0);
	if (!dn)
		return -ENXIO;

	data->fg_i2c_client = of_find_i2c_device_by_node(dn);
	if (!data->fg_i2c_client)
		return -EAGAIN;

	return 0;
}

static int max77759_read_vbatt(struct max77759_chgr_data *data, int *vbatt)
{
	int ret;

	ret = max77759_find_fg(data);
	if (ret == 0)
		ret = max1720x_get_voltage_now(data->fg_i2c_client, vbatt);

	return ret;
}

/* ----------------------------------------------------------------------- */

/* set WDTEN in CHG_CNFG_18 (0xCB), tWD = 80s */
static int max77759_wdt_enable(struct max77759_chgr_data *data, bool enable)
{
	int ret;
	u8 reg;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_18, &reg);
	if (ret < 0)
		return -EIO;

	if ((!!_chg_cnfg_18_wdten_get(reg)) == enable)
		return 0;

	/* this register is protected, read back to check if it worked */
	reg = _chg_cnfg_18_wdten_set(reg, enable);
	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_18, reg);
	if (ret < 0)
		return -EIO;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_18, &reg);
	if (ret < 0)
		return -EIO;

	return (ret == 0 && (!!_chg_cnfg_18_wdten_get(reg)) == enable) ?
		0 : -EINVAL;
}

/* First step to convert votes to a usecase and a setting for mode */
static int max77759_foreach_callback(void *data, const char *reason,
				     void *vote)
{
	struct max77759_foreach_cb_data *cb_data = data;
	int mode = (long)vote; /* max77759_mode is an int election */

	switch (mode) {
	/* Direct raw modes last come fist served */
	case MAX77759_CHGR_MODE_ALL_OFF:
	case MAX77759_CHGR_MODE_BUCK_ON:
	case MAX77759_CHGR_MODE_CHGR_BUCK_ON:
	case MAX77759_CHGR_MODE_BOOST_UNO_ON:
	case MAX77759_CHGR_MODE_BOOST_ON:
	case MAX77759_CHGR_MODE_OTG_BOOST_ON:
	case MAX77759_CHGR_MODE_BUCK_BOOST_UNO_ON:
	case MAX77759_CHGR_MODE_CHGR_BUCK_BOOST_UNO_ON:
	case MAX77759_CHGR_MODE_OTG_BUCK_BOOST_ON:
	case MAX77759_CHGR_MODE_CHGR_OTG_BUCK_BOOST_ON:
		pr_debug("%s: RAW vote=0x%x\n", __func__, mode);
		if (cb_data->use_raw)
			break;
		cb_data->raw_value = mode;
		cb_data->reason = reason;
		cb_data->use_raw = true;
		break;

	/* temporary, can be used to program the WLC chip, remove */
	case GBMS_CHGR_MODE_BOOST_UNO_ON:
		if (!cb_data->boost_on || !cb_data->uno_on)
			cb_data->reason = reason;
		pr_debug("%s: BOOST_UNO vote=0x%x\n", __func__, mode);
		cb_data->boost_on += 1;
		cb_data->uno_on += 1;
		break;

	/* SYSTEM modes can add complex transactions */

	/* MAX77759: on disconnect */
	case GBMS_CHGR_MODE_STBY_ON:
		if (!cb_data->stby_on)
			cb_data->reason = reason;
		pr_debug("%s: STBY_ON %s vote=0x%x\n",
			 __func__, reason ? reason : "<>", mode);
		cb_data->stby_on += 1;
		break;
	/* USB+WLCIN, factory only */
	case GBMS_CHGR_MODE_USB_WLC_RX:
		pr_debug("%s: USB_WLC_RX %s vote=0x%x\n",
			 __func__, reason ? reason : "<>", mode);
		if (!cb_data->usb_wlc)
			cb_data->reason = reason;
		cb_data->usb_wlc += 1;
		break;

	/* input_suspend => 0 ilim */
	case GBMS_CHGR_MODE_CHGIN_OFF:
		if (!cb_data->chgin_off)
			cb_data->reason = reason;
		pr_debug("%s: CHGIN_OFF %s vote=0x%x\n", __func__,
			 reason ? reason : "<>", mode);
		cb_data->chgin_off += 1;
		break;
	/* input_suspend => DC_SUSPEND */
	case GBMS_CHGR_MODE_WLCIN_OFF:
		if (!cb_data->wlcin_off)
			cb_data->reason = reason;
		pr_debug("%s: WLCIN_OFF %s vote=0x%x\n", __func__,
			 reason ? reason : "<>", mode);
		cb_data->wlcin_off += 1;
		break;
	/* MAX77759: charging on via CC_MAX (needs inflow, buck_on on) */
	case GBMS_CHGR_MODE_CHGR_BUCK_ON:
		if (!cb_data->chgr_on)
			cb_data->reason = reason;
		pr_debug("%s: CHGR_BUCK_ON %s vote=0x%x\n", __func__,
			 reason ? reason : "<>", mode);
		cb_data->chgr_on += 1;
		break;

	/* USB: present, charging controlled via GBMS_CHGR_MODE_CHGR_BUCK_ON */
	case GBMS_USB_BUCK_ON:
		if (!cb_data->buck_on)
			cb_data->reason = reason;
		pr_debug("%s: BUCK_ON %s vote=0x%x\n", __func__,
			 reason ? reason : "<>", mode);
		cb_data->buck_on += 1;
		break;
	/* USB: OTG, source, fast role swap case */
	case GBMS_USB_OTG_FRS_ON:
		if (!cb_data->frs_on)
			cb_data->reason = reason;
		pr_debug("%s: FRS_ON vote=0x%x\n", __func__, mode);
		cb_data->frs_on += 1;
		break;
	/* USB: boost mode, source, normally external boost */
	case GBMS_USB_OTG_ON:
		if (!cb_data->otg_on)
			cb_data->reason = reason;
		pr_debug("%s: OTG_ON %s vote=0x%x\n", __func__,
			 reason ? reason : "<>", mode);
		cb_data->otg_on += 1;
		break;
	/* DC Charging: mode=0, set CP_EN */
	case GBMS_CHGR_MODE_CHGR_DC:
		if (!cb_data->dc_on)
			cb_data->reason = reason;
		pr_debug("%s: DC_ON vote=0x%x\n", __func__, mode);
		cb_data->dc_on += 1;
		break;
	/* WLC Tx */
	case GBMS_CHGR_MODE_WLC_TX:
		if (!cb_data->wlc_tx)
			cb_data->reason = reason;
		pr_debug("%s: WLC_TX vote=%x\n", __func__, mode);
		cb_data->wlc_tx += 1;
		break;

	default:
		pr_err("mode=%x not supported\n", mode);
		break;
	}

	return 0;
}

/* control VENDOR_EXTBST_CTRL (from TCPCI module) */
static int max77759_ls_mode(struct max77759_usecase_data *uc_data, int mode)
{
	int ret;

	pr_debug("%s: mode=%d ext_bst_ctl=%d lsw1_c=%d lsw1_o=%d\n", __func__, mode,
		uc_data->ext_bst_ctl, uc_data->lsw1_is_closed,
		uc_data->lsw1_is_open);

	if (uc_data->ext_bst_ctl < 0)
		return 0;

	/* VENDOR_EXTBST_CTRL control LSW1, the read will check the state */
	gpio_set_value_cansleep(uc_data->ext_bst_ctl, mode);

	/* b/182953320 load switch is optional */
	if (uc_data->lsw1_is_open < 0 || uc_data->lsw1_is_closed < 0)
		return 0;

	/* ret <= 0 if *_is* is not true and > 1 if true */
	switch (mode) {
	case 0:
		/* the OVP open right away */
		ret = gpio_get_value_cansleep(uc_data->lsw1_is_open);
		if (ret <= 0 && uc_data->ls1_en > 0) {
			const int max_count = 3;
			int loops;

			/*  do it manually and re-read after 20ms */
			for (loops = 0; loops < max_count; loops++) {
				gpio_set_value_cansleep(uc_data->ls1_en, 0);
				msleep(20);

				ret = gpio_get_value_cansleep(uc_data->lsw1_is_open);
				pr_debug("%s: open lsw1 attempt %d/%d ret=%d\n",
					 __func__, loops, max_count, ret);
				if (ret > 0)
					break;
			}
		}
		break;
	case 1:
		/* it takes 11 ms to turn on the OVP */
		msleep(11);
		ret = gpio_get_value_cansleep(uc_data->lsw1_is_closed);
		break;
	default:
		return -EINVAL;
	}

	return (ret <= 0) ? -EIO : 0;
}

/* OVP LS2 */
static int max77759_ls2_mode(struct max77759_usecase_data *uc_data, int mode)
{
	pr_debug("%s: ls2_en=%d mode=%d\n", __func__, uc_data->ls2_en, mode);

	if (uc_data->ls2_en >= 0)
		gpio_set_value_cansleep(uc_data->ls2_en, !!mode);

	return 0;
}

/* control external boost mode
 * can be done controlling ls1, ls2
 */
#define EXT_MODE_OFF		0
#define EXT_MODE_OTG_5_0V	1
#define EXT_MODE_OTG_7_5V	2

/*
 * bst_on=GPIO5 on Max77759 on canopy and on all whitefins,
 * bst_sel=Granville
 */
static int max77759_ext_mode(struct max77759_usecase_data *uc_data, int mode)
{
	int ret = 0;

	pr_debug("%s: mode=%d on=%d sel=%d\n", __func__, mode,
		 uc_data->bst_on, uc_data->bst_sel);

	if (uc_data->bst_on < 0 || uc_data->bst_sel < 0)
		return 0;

	switch (mode) {
	case EXT_MODE_OFF:
		gpio_set_value_cansleep(uc_data->bst_on, 0);
		break;
	case EXT_MODE_OTG_5_0V:
		gpio_set_value_cansleep(uc_data->bst_sel, 0);
		mdelay(100);
		gpio_set_value_cansleep(uc_data->bst_on, 1);
		break;
	case EXT_MODE_OTG_7_5V: /* TODO: verify this */
		gpio_set_value_cansleep(uc_data->bst_sel, 1);
		mdelay(100);
		gpio_set_value_cansleep(uc_data->bst_on, 1);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int gs101_wlc_en(struct max77759_usecase_data *uc_data, bool wlc_on)
{
	int ret = 0;

	pr_debug("%s: cpout_en=%d wlc_en=%d wlc_vbus_en=%d wlc_on=%d\n", __func__,
		 uc_data->cpout_en, uc_data->wlc_en, uc_data->wlc_vbus_en, wlc_on);

	if (uc_data->cpout_en >= 0) {
		gpio_set_value_cansleep(uc_data->cpout_en, wlc_on);
	} else if (!wlc_on) {
		/*
		 * when
		 *   uc_data->cpout_en != -EPROBE_DEFER && uc_data->wlc_en
		 * could use uc_data->wlc_en with:
		 *   gpio_set_value_cansleep(uc_data->wlc_en, !!wlc_on);
		 *
		 * BUT need to resolve tjhe race on start since toggling
		 * ->wlc_en might not be undone by using ->cpout_en
		 */
		pr_debug("%s: no toggle for WLC on\n", __func__);
	}

	/* b/202526678 */
	if (uc_data->wlc_vbus_en >= 0)
		gpio_set_value_cansleep(uc_data->wlc_vbus_en, wlc_on);

	return ret;
}

/* RTX reverse wireless charging */
static int gs101_wlc_tx_enable(struct max77759_usecase_data *uc_data,
			       bool enable)
{
	int ret;

	if (enable) {

		ret = max77759_ls2_mode(uc_data, 1);
		if (ret == 0)
			ret = max77759_ext_mode(uc_data, EXT_MODE_OTG_7_5V);
		if (ret < 0)
			return ret;

		mdelay(100);

		/* p9412 will not be in RX when powered from EXT */
		ret = gs101_wlc_en(uc_data, true);
		if (ret < 0)
			return ret;

		if (uc_data->cpout21_en >= 0)
			gpio_set_value_cansleep(uc_data->cpout21_en, 0);
	} else {
		/* p9412 is already off from insel */
		ret = gs101_wlc_en(uc_data, false);
		if (ret < 0)
			return ret;

		/* NOTE: turn off WLC, no need to reset cpout */
		ret = max77759_ext_mode(uc_data, EXT_MODE_OFF);
		if (ret == 0)
			ret = max77759_ls2_mode(uc_data, 0);

		/* STBY will re-enable WLC */
	}

	return ret;
}

/* change p9412 CPOUT and adjust WCIN_REG */
#define GS101_WLCRX_CPOUT_DFLT	0
#define GS101_WLCRX_CPOUT_5_2V	1

static int gs101_cpout_mode(struct max77759_usecase_data *uc_data, int mode)
{
	int ret;

	/* do not change MW unless p9412 can be changed as well */
	if (uc_data->cpout_ctl < 0)
		return 0;

	if (mode == GS101_WLCRX_CPOUT_5_2V) {
		/* p9412: set CPOUT==5.2 only if on BPP */
		gpio_set_value_cansleep(uc_data->cpout_ctl, 1);

		/* NOTE: no DC_IN to MW when WCIN_REG==4_85 unless CPOUT==5.2 */
		ret =  max77759_chg_reg_update(uc_data->client, MAX77759_CHG_CNFG_12,
					       MAX77759_CHG_CNFG_12_WCIN_REG_MASK,
					       MAX77759_CHG_CNFG_12_WCIN_REG_4_85);
	} else {
		/* p9412: reset CPOUT to default */
		gpio_set_value_cansleep(uc_data->cpout_ctl, 0);

		ret =  max77759_chg_reg_update(uc_data->client, MAX77759_CHG_CNFG_12,
					       MAX77759_CHG_CNFG_12_WCIN_REG_MASK,
					       MAX77759_CHG_CNFG_12_WCIN_REG_4_5);
	}

	return ret;
}

static int max7759_otg_update_ilim(struct max77759_usecase_data *uc_data, int enable)
{
	u8 ilim;

	if (uc_data->otg_orig == uc_data->otg_ilim)
		return 0;

	if (enable) {
		int rc;

		rc = max77759_chg_reg_read(uc_data->client, MAX77759_CHG_CNFG_05,
					   &uc_data->otg_orig);
		if (rc < 0) {
			pr_err("%s: cannot read otg_ilim (%d), use default\n",
			       __func__, rc);
			uc_data->otg_orig = MAX77759_CHG_CNFG_05_OTG_ILIM_1500MA;
		} else {
			uc_data->otg_orig &= MAX77759_CHG_CNFG_05_OTG_ILIM_MASK;
		}

		ilim = uc_data->otg_ilim;
	} else {
		ilim = uc_data->otg_orig;
	}

	return max77759_chg_reg_update(uc_data->client, MAX77759_CHG_CNFG_05,
				      MAX77759_CHG_CNFG_05_OTG_ILIM_MASK,
				      ilim);
}

static int max7759_otg_frs(struct max77759_usecase_data *uc_data, int enable)
{
	int ret;

	ret = max77759_chg_mode_write(uc_data->client, MAX77759_CHGR_MODE_OTG_BOOST_ON);
	if (ret < 0) {
		pr_err("%s: cannot set CNFG_00 to 0xa ret:%d\n", __func__, ret);
		return ret;
	}

	ret = max7759_otg_update_ilim(uc_data, enable);
	if (ret < 0)
		pr_err("%s: cannot update otg_ilim: %d\n", __func__, ret);

	return ret;
}

/*
 * Transition to standby (if needed) at the beginning of the sequences
 * @return <0 on error, 0 on success. ->use_case becomes GSU_MODE_STANDBY
 * if the transition is necessary (and successful).
 */
static int max77759_to_standby(struct max77759_usecase_data *uc_data,
			       int use_case)
{
	const int from_uc = uc_data->use_case;
	bool need_stby = false;
	bool from_otg = false;
	int ret;

	switch (from_uc) {
		case GSU_MODE_USB_CHG:
			need_stby = use_case != GSU_MODE_USB_CHG_WLC_TX &&
				    use_case != GSU_MODE_WLC_RX &&
				    use_case != GSU_MODE_USB_DC &&
				    use_case != GSU_MODE_USB_OTG_FRS;
			break;
		case GSU_MODE_WLC_RX:
			need_stby = use_case != GSU_MODE_USB_OTG_WLC_RX &&
				    use_case != GSU_MODE_WLC_DC;
			break;
		case GSU_MODE_WLC_TX:
			need_stby = use_case != GSU_MODE_USB_OTG_WLC_TX &&
				    use_case != GSU_MODE_USB_CHG_WLC_TX &&
				    use_case != GSU_MODE_USB_DC_WLC_TX &&
				    use_case != GSU_MODE_USB_OTG_FRS;
			break;
		case GSU_MODE_USB_CHG_WLC_TX:
			need_stby = use_case != GSU_MODE_USB_CHG &&
				    use_case != GSU_MODE_USB_OTG_WLC_TX &&
				    use_case != GSU_MODE_USB_DC;
			break;

		case GSU_MODE_USB_OTG:
			from_otg = true;
			if (use_case == GSU_MODE_USB_OTG_FRS)
				break;
			if (use_case == GSU_MODE_USB_OTG_WLC_TX)
				break;
			if (use_case == GSU_MODE_USB_OTG_WLC_RX)
				break;
			if (use_case == GSU_MODE_USB_OTG_WLC_DC)
				break;

			/* From 5. USB OTG to anything else, go to stby */
			ret = max77759_ls_mode(uc_data, 0);
			if (ret == 0)
				ret = max77759_ext_mode(uc_data, 0);
			if (ret < 0)
				return -EIO;

			/* TODO:Discharge IN/OUT with AO37 is done in TCPM */
			mdelay(100);

			need_stby = true;
			break;

		case GSU_MODE_USB_OTG_WLC_RX:
			from_otg = true;
			need_stby = use_case != GSU_MODE_WLC_RX &&
				    use_case != GSU_MODE_USB_OTG;
			break;
		case GSU_MODE_USB_DC:
			need_stby = use_case != GSU_MODE_USB_DC;
			break;
		case GSU_MODE_WLC_DC:
			need_stby = use_case != GSU_MODE_WLC_DC;
			break;

		case GSU_MODE_USB_OTG_FRS:
			from_otg = true;
			need_stby = use_case != GSU_MODE_USB_OTG_FRS &&
				    use_case != GSU_MODE_USB_OTG_WLC_TX;
			break;
			/*
			 *  if (use_case == GSU_MODE_USB_OTG)
			 * 	break;
			 */
		case GSU_RAW_MODE:
			need_stby = true;
			break;
		case GSU_MODE_USB_OTG_WLC_TX:
			from_otg = true;
			need_stby = false;
			break;
		case GSU_MODE_STANDBY:
		default:
			need_stby = false;
			break;
	}

	if (use_case == GSU_MODE_USB_WLC_RX || use_case == GSU_RAW_MODE)
		need_stby = true;

	pr_info("%s: use_case=%d->%d from_otg=%d need_stby=%d\n", __func__,
		 from_uc, use_case, from_otg, need_stby);

	if (!need_stby)
		return 0;

	/* there are no ways out of OTG FRS */
	if (from_uc == GSU_MODE_USB_OTG_FRS) {
		ret = max7759_otg_frs(uc_data, true);
		if (ret < 0) {
			pr_err("%s: cannot turn off OTG_FRS (%d)\n", __func__, ret);
			return ret;
		}
	}

	/* from WLC_TX to STBY */
	if (from_uc == GSU_MODE_WLC_TX) {
		ret = gs101_wlc_tx_enable(uc_data, false);
		if (ret < 0) {
			pr_err("%s: cannot tun off wlc_tx (%d)\n", __func__, ret);
			return ret;
		}

		/* re-enable wlc IC if disabled */
		ret = gs101_wlc_en(uc_data, true);
		if (ret < 0)
			pr_err("%s: cannot enable WLC (%d)\n", __func__, ret);
	}

	/*
	 * There is no direct transition to STBY from BPP_RX+OTG  but we might
	 * get here on error and when forcing raw values. This makes sure that
	 * CPOUT is set to default.
	 */
	if (from_uc == GSU_MODE_USB_OTG_WLC_RX) {
		ret = gs101_cpout_mode(uc_data, GS101_WLCRX_CPOUT_DFLT);
		if (ret < 0)
			pr_err("%s: cannot reset cpout (%d)\n", __func__, ret);
	}

	/* b/178458456 exit from all OTG cases need to reset the limit */
	if (uc_data->otg_enable > 0)
		gpio_set_value_cansleep(uc_data->otg_enable, 0);

	/* transition to STBY (might need to be up) */
	ret = max77759_chg_mode_write(uc_data->client, MAX77759_CHGR_MODE_ALL_OFF);
	if (ret < 0)
		return -EIO;

	uc_data->use_case = GSU_MODE_STANDBY;
	return ret;
}

/* enable/disable soft-start. No need soft start from OTG->OTG_FRS */
static int max7759_ramp_bypass(struct max77759_usecase_data *uc_data, bool enable)
{
	const u8 value = enable ? MAX77759_CHG_CNFG_00_BYPV_RAMP_BYPASS : 0;

	if (uc_data->is_a1 <= 0)
		return 0;

	return max77759_chg_reg_update(uc_data->client, MAX77759_CHG_CNFG_00,
				       MAX77759_CHG_CNFG_00_BYPV_RAMP_BYPASS,
				       value);
}

/* cleanup from every usecase */
static int max77759_force_standby(struct max77759_chgr_data *data)
{
	struct max77759_usecase_data *uc_data = &data->uc_data;
	const u8 insel_mask = MAX77759_CHG_CNFG_12_CHGINSEL_MASK |
			      MAX77759_CHG_CNFG_12_WCINSEL_MASK;
	const u8 insel_value = MAX77759_CHG_CNFG_12_CHGINSEL |
			       MAX77759_CHG_CNFG_12_WCINSEL;
	int ret;

	pr_debug("%s: recovery\n", __func__);

	ret = max77759_ls_mode(uc_data, 0);
	if (ret < 0)
		dev_err(data->dev, "%s: cannot change ls_mode (%d)\n",
			__func__, ret);

	ret = max77759_ext_mode(uc_data, 0);
	if (ret < 0)
		dev_err(data->dev, "%s: cannot change ext mode (%d)\n",
			__func__, ret);

	ret = max7759_ramp_bypass(uc_data, false);
	if (ret < 0)
		dev_err(data->dev, "%s: cannot reset ramp_bypass (%d)\n",
			__func__, ret);

	ret = max77759_chg_mode_write(uc_data->client, MAX77759_CHGR_MODE_ALL_OFF);
	if (ret < 0)
		dev_err(data->dev, "%s: cannot reset mode register (%d)\n",
			__func__, ret);

	ret = max77759_chg_insel_write(uc_data->client, insel_mask, insel_value);
	if (ret < 0)
		dev_err(data->dev, "%s: cannot reset insel (%d)\n",
			__func__, ret);

	return 0;
}

/* b/188488966 */
static int gs101_frs_to_otg(struct max77759_usecase_data *uc_data)
{
	int closed, ret;

	ret = max77759_ext_mode(uc_data, EXT_MODE_OTG_5_0V);
	if (ret < 0)
		goto exit_done;

	mdelay(100);

	if (uc_data->ls1_en > 0)
		gpio_set_value_cansleep(uc_data->ls1_en, 1);

	mdelay(100);

	if (uc_data->lsw1_is_closed >= 0)
		closed = gpio_get_value_cansleep(uc_data->lsw1_is_closed);

exit_done:
	pr_debug("%s: ls1_en=%d lsw1_is_closed=%d closed=%d ret=%d\n",
		 __func__, uc_data->ls1_en, uc_data->lsw1_is_closed,
		 closed, ret);
	return ret;
}

/* From OTG <-> OTG_FRS */
static int gs101_otg_mode(struct max77759_usecase_data *uc_data, int to)
{
	int ret = -EINVAL;

	pr_debug("%s: to=%d\n", __func__, to);

	if (to == GSU_MODE_USB_OTG) {

		ret = max77759_ext_mode(uc_data, EXT_MODE_OTG_5_0V);
		if (ret < 0)
			return ret;

		mdelay(5);

		ret = max77759_chg_mode_write(uc_data->client,
					      MAX77759_CHGR_MODE_ALL_OFF);

	} else if (to == GSU_MODE_USB_OTG_FRS) {
		int rc;

		ret = max7759_ramp_bypass(uc_data, true);
		if (ret == 0)
			ret = max77759_chg_mode_write(uc_data->client,
						      MAX77759_CHGR_MODE_OTG_BOOST_ON);
		if (ret < 0)
			return ret;

		mdelay(5);

		rc =  max7759_ramp_bypass(uc_data, false);
		if (rc < 0)
			pr_err("%s: cannot clear bypass rc:%d\n",  __func__, rc);

		/* b/192986752 make sure that LSW1 is open before going to FRS */
		rc = max77759_ls_mode(uc_data, 0);
		if (rc < 0)
			pr_err("%s: cannot clear lsw1 rc:%d\n",  __func__, rc);

		ret = max77759_ext_mode(uc_data, EXT_MODE_OFF);
		if (ret < 0)
			return ret;
	}

	return ret;
}

/*
 * This must follow different paths depending on the platforms.
 *
 * When vin_is_valid is implemented the code uses the NBC workaround for MW
 * and the OVP. The code defaults to just setting the MODE register (at the
 * end of the use case) and toggling bst_on/bst_sel and setting ext_bst_ctl
 * otherwise.
 *
 * NOTE: the USB stack expects VBUS to be on after voting for the usecase.
 */
static int max7759_otg_enable(struct max77759_usecase_data *uc_data, int mode)
{
	int ret;

	/* the code default to write to the MODE register */
	if (uc_data->vin_is_valid >= 0) {

		/* b/178458456 */
		if (uc_data->otg_enable > 0)
			gpio_set_value_cansleep(uc_data->otg_enable, 1);

		/* NBC workaround */
		ret = max77759_ls_mode(uc_data, 1);
		if (ret < 0) {
			pr_debug("%s: cannot close load switch (%d)\n",
				 __func__, ret);
			return ret;
		}

		ret = max77759_chg_mode_write(uc_data->client,
					      MAX77759_CHGR_MODE_OTG_BOOST_ON);
		if (ret < 0) {
			pr_debug("%s: cannot set CNFG_00 to 0xa ret:%d\n",
				 __func__, ret);
			return ret;
		}

		ret = gpio_get_value_cansleep(uc_data->vin_is_valid);
		if (ret == 0) {
			pr_debug("%s: VIN not VALID\n",  __func__);
			return -EIO;
		}

		ret = max77759_ext_mode(uc_data, mode);
		if (ret < 0)
			pr_debug("%s: cannot change extmode ret:%d\n",
				 __func__, ret);

	} else {

		/* ext mode is defined when ext boost is avalaible */
		ret = max77759_ext_mode(uc_data, mode);
		if (ret < 0) {
			pr_debug("%s: cannot change extmode ret:%d\n",
				 __func__, ret);
			return ret;
		}

		mdelay(5);

		/* load switch from MW */
		ret = max77759_ls_mode(uc_data, 1);
		if (ret < 0) {
			pr_debug("%s: cannot close load switch (%d)\n",
				 __func__, ret);
			return ret;
		}

		/* time for VBUS to be on (could check PWRSTAT in MW) */
		mdelay(30);

		/* b/178458456 */
		if (uc_data->otg_enable > 0)
			gpio_set_value_cansleep(uc_data->otg_enable, 0);
	}

	return ret;
}

/* configure ilim wlctx */
static int gs101_wlctx_otg_en(struct max77759_usecase_data *uc_data, bool enable)
{
	int ret;

	if (enable) {
		/* this should be already set */
		if (uc_data->sw_en >= 0)
			gpio_set_value_cansleep(uc_data->sw_en, 1);

		ret = max7759_otg_update_ilim(uc_data, true);
		if (ret < 0)
			pr_err("%s: cannot update otg_ilim (%d)\n", __func__, ret);

		/* need this to be able to program ->otg_vbyp */
		ret = max77759_chg_reg_update(uc_data->client, MAX77759_CHG_CNFG_18,
					      MAX77759_CHG_CNFG_18_OTG_V_PGM,
					      MAX77759_CHG_CNFG_18_OTG_V_PGM);
		if (ret < 0) {
			pr_err("%s: cannot set otg_v_pgm (%d)\n", __func__, ret);
			return ret;
                }

		ret = max77759_chg_reg_write(uc_data->client, MAX77759_CHG_CNFG_11,
					     uc_data->otg_vbyp);
	} else {
		/* TODO: Discharge IN/OUT nodes with AO37 should be done in TCPM */

		mdelay(100);

		/* "bad things will happen (tm)" if you force off ->sw_en */

		ret = max7759_otg_update_ilim(uc_data, false);
		if (ret < 0)
			pr_err("%s: cannot restore otg_ilim (%d)\n", __func__, ret);

		/* TODO: restore initial value on !MAX77759_CHG_CNFG_11 */

		ret = max77759_chg_reg_update(uc_data->client, MAX77759_CHG_CNFG_18,
					      MAX77759_CHG_CNFG_18_OTG_V_PGM, 0);
		if (ret < 0)
			pr_err("%s: cannot reset otg_v_pgm (%d)\n", __func__, ret);

		ret = 0;
        }

	return ret;
}

static int gs101_ext_bst_mode(struct max77759_usecase_data *uc_data, int mode)
{

	pr_debug("%s: ext_bst_mode=%d mode=%d\n", __func__, uc_data->ext_bst_mode, mode);

	if (uc_data->ext_bst_mode >= 0)
		gpio_set_value_cansleep(uc_data->ext_bst_mode, mode);

	return 0;
}

/*
 * Case	USB_chg USB_otg	WLC_chg	WLC_TX	PMIC_Charger	Ext_B	LSxx	Name
 * -------------------------------------------------------------------------------------
 * 4-1	0	1	10	0	IF-PMIC-WCIN	1	1/0	USB_OTG_WLC_RX
 * 4-2	0	1	01	0	DC WCIN		1	1/0	USB_OTG_WLC_DC
 * 5-1	0	1	0	0	0		1	1/0	USB_OTG
 * 5-2	0	1	0	0	OTG_5V		0	0/0	USB_OTG_FRS
 * 7-2	0	1	0	1	OTG_5V		2	0/1	USB_OTG_WLC_TX
 * -------------------------------------------------------------------------------------
 * WLC_chg = 0 off, 1 = on, 2 = PPS
 * Ext_Boost = 0 off, 1 = OTG 5V, 2 = WTX 7.5
 *
 * 5-1: mode=0x0 in MW, EXT_B=1, LS1=1, LS2=0, IDLE <-> OTG (ext)
 * 5-2: mode=0xa in MW, EXT_B=0, LS1=0, LS2=0, IDLE <-> OTG_FRS
 * 7-2: mode=0xa in MW, EXT_B=2, LS1=0, LS2=1
 *
 * AO37 + GPIO5 MW (canopy 3, whitev2p2)
 * . AO_ls1 <&max20339_gpio 0 GPIO_ACTIVE_HIGH> - bit0
 * . AO_ls2 <&max20339_gpio 1 GPIO_ACTIVE_HIGH> - bit4
 *
 * ls1 can be controlled poking the AO37 OR using a MW_GPIO _> EXT_BST_EN
 *
 * max77759,bst_on = <&max777x9_gpio 4 GPIO_ACTIVE_HIGH>
 * max77759,bst-sel = <&gpp27 3 GPIO_ACTIVE_HIGH>
 * max77759,bst_on=0, max77759,bst_sel=x => OFF
 * max77759,bst_on=1, max77759,bst_sel=0 => 5V
 * max77759,bst_on=1, max77759,bst_sel=1 => 7.5V
 *
 * Ext_Boost = 0 off
 * 	MW_gpio5	: Ext_B = 0, MW_gpio5 -> LOW
 * 	AO_ls1/ls2	: 0/0
 *
 * Ext_Boost = 1 = OTG 5V
 * 	MW_gpio5	: Ext_B = 1, MW_gpio5 -> HIGH
 * 	AO_ls1/ls2	: 1/0
 *
 * Ext_Boost = 2 WTX 7.5
 * 	MW_gpio5	: Ext_B = 2, MW_gpio5 -> HIGH
 * 	AO_ls1/ls2	: 0/1
 *
 * NOTE: do not call with (cb_data->wlc_rx && cb_data->wlc_tx)
 */

/* was b/179816224 WLC_RX -> WLC_RX + OTG (Transition #10) */
static int gs101_wlcrx_to_wlcrx_otg(struct max77759_usecase_data *uc_data)
{
	pr_warn("%s: disabled\n", __func__);
	return -ENOTSUPP;
}

static int max77759_to_otg_usecase(struct max77759_usecase_data *uc_data, int use_case)
{
	const int from_uc = uc_data->use_case;
	int ret = 0;

	switch (from_uc) {
	/* 5-1: #3: stby to USB OTG, mode = 1 */
	/* 5-2: #3: stby to USB OTG_FRS, mode = 0 */
	case GSU_MODE_STANDBY: {
		const int mode = use_case == GSU_MODE_USB_OTG_FRS ?
					     EXT_MODE_OFF :
					     EXT_MODE_OTG_5_0V;

		/* NBC workaround */
		ret = max7759_otg_enable(uc_data, mode);
		if (ret < 0)
			break;

		mdelay(5);

		/*
		 * Assumption: max77759_to_usecase() will write back cached values to
		 * CHG_CNFG_00.Mode. At the moment, the cached value at
		 * max77759_mode_callback is 0. If the cached value changes to someting
		 * other than 0, then, the code has to be revisited.
		 */
	} break;

	/* b/186535439 : USB_CHG->USB_OTG_FRS*/
	case GSU_MODE_USB_CHG:
	case GSU_MODE_USB_CHG_WLC_TX:
		/* need to go through stby out of this */
		if (use_case != GSU_MODE_USB_OTG_FRS && use_case != GSU_MODE_USB_OTG_WLC_TX)
			return -EINVAL;

		ret = max7759_otg_frs(uc_data, true);
	break;


	case GSU_MODE_WLC_TX:
		/* b/179820595: WLC_TX -> WLC_TX + OTG */
		if (use_case == GSU_MODE_USB_OTG_WLC_TX) {
			ret = max77759_chg_mode_write(uc_data->client, MAX77759_CHGR_MODE_OTG_BOOST_ON);
			if (ret < 0) {
				pr_err("%s: cannot set CNFG_00 to 0xa ret:%d\n",  __func__, ret);
				return ret;
			}
		}
	break;

	case GSU_MODE_WLC_RX:
		if (use_case == GSU_MODE_USB_OTG_WLC_RX)
			ret = gs101_wlcrx_to_wlcrx_otg(uc_data);
	break;

	case GSU_MODE_USB_OTG:
		/* b/179820595: OTG -> WLC_TX + OTG (see b/181371696) */
		if (use_case == GSU_MODE_USB_OTG_WLC_TX) {
			ret = gs101_otg_mode(uc_data, GSU_MODE_USB_OTG_FRS);
			if (ret == 0)
				ret = gs101_wlc_tx_enable(uc_data, true);
		}
		/* b/179816224: OTG -> WLC_RX + OTG */
		if (use_case == GSU_MODE_USB_OTG_WLC_RX) {
			ret = gs101_cpout_mode(uc_data, GS101_WLCRX_CPOUT_5_2V);
			if (ret == 0)
				ret = gs101_ext_bst_mode(uc_data, 1);
		}
	break;
	case GSU_MODE_USB_OTG_WLC_TX:
		/*  b/179820595: WLC_TX + OTG -> OTG */
		if (use_case == GSU_MODE_USB_OTG) {
			ret = gs101_wlc_tx_enable(uc_data, false);
			if (ret == 0)
				ret = gs101_frs_to_otg(uc_data);
		}
	break;
	case GSU_MODE_USB_OTG_WLC_RX:
		/* b/179816224: WLC_RX + OTG -> OTG */
		if (use_case == GSU_MODE_USB_OTG) {
			/* it's in STBY, no need to reset gs101_otg_mode()  */
			ret = gs101_ext_bst_mode(uc_data, 0);
			if (ret == 0)
				ret = gs101_cpout_mode(uc_data, GS101_WLCRX_CPOUT_DFLT);
		}
	break;
	/* TODO: */
	case GSU_MODE_USB_OTG_FRS: {
		if (use_case == GSU_MODE_USB_OTG_WLC_TX) {
			ret = gs101_wlc_tx_enable(uc_data, true);
			break;
		}
		/*
		 * OTG source handover: OTG_FRS -> OTG
		 * from EXT_BST (Regular OTG) to IF-PMIC OTG (FRS OTG)
		 */
		if (use_case != GSU_MODE_USB_OTG)
			return -EINVAL;

		/* TODO: */
	} break;

	default:
		return -ENOTSUPP;
	}

	return ret;
}

/* handles the transition data->use_case ==> use_case */
static int max77759_to_usecase(struct max77759_usecase_data *uc_data, int use_case)
{
	const int from_uc = uc_data->use_case;
	int ret = 0;

	switch (use_case) {
	case GSU_MODE_USB_OTG:
	case GSU_MODE_USB_OTG_FRS:
	case GSU_MODE_USB_OTG_WLC_RX:
	case GSU_MODE_USB_OTG_WLC_DC:
	case GSU_MODE_USB_OTG_WLC_TX:
		ret = max77759_to_otg_usecase(uc_data, use_case);
		if (ret < 0)
			return ret;
		break;
	case GSU_MODE_WLC_TX:
	case GSU_MODE_USB_CHG_WLC_TX:
		/* Coex Case #4, WLC_TX + OTG -> WLC_TX */
		if (from_uc == GSU_MODE_USB_OTG_WLC_TX) {
			ret = max77759_chg_mode_write(uc_data->client,
						      MAX77759_CHGR_MODE_ALL_OFF);
			if (ret == 0)
				ret = gs101_wlctx_otg_en(uc_data, false);
		} else {
			ret = gs101_wlc_tx_enable(uc_data, true);
		}

		break;
	case GSU_MODE_WLC_RX:
		if (from_uc == GSU_MODE_USB_OTG_WLC_RX) {
			/* to_stby brought to stby */
			ret = gs101_ext_bst_mode(uc_data, 0);
			if (ret == 0)
				ret = gs101_cpout_mode(uc_data, GS101_WLCRX_CPOUT_DFLT);
			if (ret == 0)
				ret = gs101_otg_mode(uc_data, GSU_MODE_USB_OTG);
		}
		break;
	case GSU_MODE_USB_CHG:
	case GSU_MODE_USB_DC:
		if (from_uc == GSU_MODE_WLC_TX || from_uc == GSU_MODE_USB_CHG_WLC_TX) {
			ret = gs101_wlc_tx_enable(uc_data, false);
			if (ret < 0)
				return ret;
		}
		break;
	case GSU_MODE_USB_WLC_RX:
	case GSU_RAW_MODE:
		/* just write the value to the register (it's in stby) */
		break;
	default:
		break;
	}

	return ret;
}

#define cb_data_is_inflow_off(cb_data) \
	((cb_data)->chgin_off && (cb_data)->wlcin_off)

/*
 * It could use cb_data->charge_done to turn off charging.
 * TODO: change chgr_on=>2 to (cc_max && chgr_ena)
 */
static bool cb_data_is_chgr_on(struct max77759_foreach_cb_data *cb_data)
{
	return cb_data->stby_on ? 0 : (cb_data->chgr_on >= 2);
}

/*
 * Case	USB_chg USB_otg	WLC_chg	WLC_TX	PMIC_Charger	Ext_B	LSxx	Name
 * -------------------------------------------------------------------------------------
 * 4-1	0	1	10	0	IF-PMIC-WCIN	1	1/0	USB_OTG_WLC_RX
 * 4-2	0	1	01	0	DC WCIN		1	1/0	USB_OTG_WLC_DC
 * 5-1	0	1	0	0	0		1	1/0	USB_OTG
 * 5-2	0	1	0	0	OTG_5V		0	0/0	USB_OTG_FRS
 * 7-2	0	1	0	1	OTG_5V		2	0/1	USB_OTG_WLC_TX
 * -------------------------------------------------------------------------------------
 * Ext_Boost = 0 off, 1 = OTG 5V, 2 = WTX 7.5
 * WLC_chg = 0 off, 1 = on, 2 = PPS
 *
 * NOTE: do not call with (cb_data->wlc_rx && cb_data->wlc_tx)
 */
static int max77759_get_otg_usecase(struct max77759_foreach_cb_data *cb_data)
{
	const int chgr_on = cb_data_is_chgr_on(cb_data);
	bool dc_on = cb_data->dc_on; /* && !cb_data->charge_done */
	int usecase;
	u8 mode;

	/* invalid, cannot do OTG stuff with USB power */
	if (cb_data->buck_on) {
		pr_err("%s: buck_on with OTG\n", __func__);
		return -EINVAL;
	}

	/* pure OTG defaults to ext boost */
	if (!cb_data->wlc_rx && !cb_data->wlc_tx) {
		/* 5-1: USB_OTG or  5-2: USB_OTG_FRS */

		if (cb_data->frs_on) {
			usecase = GSU_MODE_USB_OTG_FRS;
			mode = MAX77759_CHGR_MODE_OTG_BOOST_ON;
		} else {
			usecase = GSU_MODE_USB_OTG;
			mode = MAX77759_CHGR_MODE_ALL_OFF;
		}

		/* b/188730136  OTG cases with DC on */
		if (dc_on)
			pr_err("%s: TODO enable pps+OTG\n", __func__);
	} else if (cb_data->wlc_tx) {
		/* 7-2: WLC_TX -> WLC_TX + OTG */
		usecase = GSU_MODE_USB_OTG_WLC_TX;
		mode = MAX77759_CHGR_MODE_OTG_BOOST_ON;
	} else if (cb_data->wlc_rx) {
		usecase = GSU_MODE_USB_OTG_WLC_RX;
		if (chgr_on)
			mode = MAX77759_CHGR_MODE_CHGR_BUCK_ON;
		else
			mode = MAX77759_CHGR_MODE_BUCK_ON;
	} else if (dc_on) {
		return -EINVAL;
	} else {
		return -EINVAL;
	}

	cb_data->reg = _chg_cnfg_00_cp_en_set(cb_data->reg, dc_on);
	cb_data->reg = _chg_cnfg_00_mode_set(cb_data->reg, mode);
	return usecase;
}

/*
 * Determines the use case to switch to. This is device/system dependent and
 * will likely be factored to a separate file (compile module).
 */
static int max77759_get_usecase(struct max77759_foreach_cb_data *cb_data)
{
	const int buck_on = cb_data->chgin_off ? 0 : cb_data->buck_on;
	const int chgr_on = cb_data_is_chgr_on(cb_data);
	bool wlc_tx = cb_data->wlc_tx != 0;
	bool wlc_rx = cb_data->wlc_rx != 0;
	bool dc_on = cb_data->dc_on; /* && !cb_data->charge_done */
	int usecase;
	u8 mode;

	/* consistency check, TOD: add more */
	if (wlc_tx) {
		if (wlc_rx) {
			pr_err("%s: wlc_tx and wlc_rx\n", __func__);
			return -EINVAL;
		}

		if (dc_on) {
			pr_warn("%s: no wlc_tx with dc_on for now\n", __func__);
			/* TODO: GSU_MODE_USB_DC_WLC_TX */
			wlc_tx = 0;
		}
	}

	/* OTG modes override the others, might need to move under usb_wlc */
	if (cb_data->otg_on || cb_data->frs_on)
		return max77759_get_otg_usecase(cb_data);

	/* USB will disable wlc_rx */
	if (cb_data->buck_on)
		wlc_rx = false;

	/* buck_on is wired, wlc_rx is wireless, might still need rTX */
	if (cb_data->usb_wlc) {
		/* USB+WLC for factory and testing */
		usecase = GSU_MODE_USB_WLC_RX;
		mode = MAX77759_CHGR_MODE_CHGR_BUCK_ON;
	} else if (!buck_on && !wlc_rx) {
		mode = MAX77759_CHGR_MODE_ALL_OFF;

		/* Rtx using the internal battery */
		usecase = GSU_MODE_STANDBY;
		if (wlc_tx)
			usecase = GSU_MODE_WLC_TX;

		/* here also on WLC_DC->WLC_DC+USB */
		dc_on = false;
	} else if (wlc_tx) {

		if (!buck_on) {
			mode = MAX77759_CHGR_MODE_ALL_OFF;
			usecase = GSU_MODE_WLC_TX;
		} else if (dc_on) {
			/* TODO: turn off DC and run off MW */
			pr_err("WLC_TX+DC is not supported yet\n");
			mode = MAX77759_CHGR_MODE_ALL_OFF;
			usecase = GSU_MODE_USB_DC;
		} else if (chgr_on) {
			mode = MAX77759_CHGR_MODE_CHGR_BUCK_ON;
			usecase = GSU_MODE_USB_CHG_WLC_TX;
		} else {
			mode = MAX77759_CHGR_MODE_BUCK_ON;
			usecase = GSU_MODE_USB_CHG_WLC_TX;
		}

	} else if (wlc_rx) {

		/* will be in mode 4 if in stby unless dc is enabled */
		if (chgr_on) {
			mode = MAX77759_CHGR_MODE_CHGR_BUCK_ON;
			usecase = GSU_MODE_WLC_RX;
		} else {
			mode = MAX77759_CHGR_MODE_BUCK_ON;
			usecase = GSU_MODE_WLC_RX;
		}

		/* wired input should be disabled here */
		if (dc_on) {
			mode = MAX77759_CHGR_MODE_ALL_OFF;
			usecase = GSU_MODE_WLC_DC;
		}

	} else {

		/* MODE_BUCK_ON is inflow */
		if (chgr_on) {
			mode = MAX77759_CHGR_MODE_CHGR_BUCK_ON;
			usecase = GSU_MODE_USB_CHG;
		} else {
			mode = MAX77759_CHGR_MODE_BUCK_ON;
			usecase = GSU_MODE_USB_CHG;
		}

		/*
		 * NOTE: OTG cases handled in max77759_get_otg_usecase()
		 * NOTE: usecases with !(buck|wlc)_on same as.
		 * NOTE: mode=0 if standby, mode=5 if charging, mode=0xa on otg
		 * TODO: handle rTx + DC and some more.
		 */
		if (dc_on && cb_data->wlc_rx) {
			/* WLC_DC->WLC_DC+USB -> ignore dc_on */
		} else if (dc_on) {
			mode = MAX77759_CHGR_MODE_ALL_OFF;
			usecase = GSU_MODE_USB_DC;
		} else if (cb_data->stby_on && !chgr_on) {
			mode = MAX77759_CHGR_MODE_ALL_OFF;
			usecase = GSU_MODE_STANDBY;
		}

	}

	/* reg might be ignored later */
	cb_data->reg = _chg_cnfg_00_cp_en_set(cb_data->reg, dc_on);
	cb_data->reg = _chg_cnfg_00_mode_set(cb_data->reg, mode);

	return usecase;
}

static int max77759_otg_ilim_ma_to_code(u8 *code, int otg_ilim)
{
	if (otg_ilim == 0)
		*code = 0;
	else if (otg_ilim >= 500 && otg_ilim <= 1500)
		*code = 1 + (otg_ilim - 500) / 100;
	else
		return -EINVAL;

	return 0;
}

static int max77759_otg_vbyp_mv_to_code(u8 *code, int vbyp)
{
	if (vbyp >= 12000)
		*code = 0x8c;
	else if (vbyp > 5000)
		*code = (vbyp - 5000) / 20;
	else
		return -EINVAL;

	return 0;
}

#define GS101_OTG_ILIM_DEFAULT_MA	1500
#define GS101_OTG_VBYPASS_DEFAULT_MV	5100

/* lazy init on the switches */


static bool gs101_setup_usecases_done(struct max77759_usecase_data *uc_data)
{
	return (uc_data->cpout_en != -EPROBE_DEFER) &&
	       (uc_data->cpout_ctl != -EPROBE_DEFER) &&
	       (uc_data->wlc_vbus_en != -EPROBE_DEFER);

	/* TODO: handle platform specific differences..
	       uc_data->ls2_en != -EPROBE_DEFER &&
	       uc_data->lsw1_is_closed != -EPROBE_DEFER &&
	       uc_data->lsw1_is_open != -EPROBE_DEFER &&
	       uc_data->vin_is_valid != -EPROBE_DEFER &&
	       uc_data->cpout_ctl != -EPROBE_DEFER &&
	       uc_data->cpout_en != -EPROBE_DEFER &&
	       uc_data->cpout21_en != -EPROBE_DEFER
	       uc_data->bst_on != -EPROBE_DEFER &&
	       uc_data->bst_sel != -EPROBE_DEFER &&
	       uc_data->ext_bst_ctl != -EPROBE_DEFER;
	*/
}


static bool max77759_setup_usecases(struct max77759_usecase_data *uc_data,
				    struct device_node *node)
{
	int ret;

	if (!node) {
		uc_data->is_a1 = -1;

		uc_data->bst_on = -EPROBE_DEFER;
		uc_data->bst_sel = -EPROBE_DEFER;
		uc_data->ext_bst_ctl = -EPROBE_DEFER;

		uc_data->ls1_en = -EPROBE_DEFER;
		uc_data->ls2_en = -EPROBE_DEFER;
		uc_data->sw_en = -EPROBE_DEFER;

		uc_data->vin_is_valid = -EPROBE_DEFER;
		uc_data->lsw1_is_closed = -EPROBE_DEFER;
		uc_data->lsw1_is_open = -EPROBE_DEFER;

		uc_data->otg_enable = -EPROBE_DEFER;

		uc_data->wlc_en = -EPROBE_DEFER;
		uc_data->wlc_vbus_en = -EPROBE_DEFER;
		uc_data->cpout_en = -EPROBE_DEFER;
		uc_data->cpout_ctl = -EPROBE_DEFER;
		uc_data->cpout21_en = -EPROBE_DEFER;

		uc_data->ext_bst_mode = -EPROBE_DEFER;

		uc_data->init_done = false;

		/* TODO: override in bootloader and remove */
		ret = max77759_otg_ilim_ma_to_code(&uc_data->otg_ilim,
						   GS101_OTG_ILIM_DEFAULT_MA);
		if (ret < 0)
			uc_data->otg_ilim = MAX77759_CHG_CNFG_05_OTG_ILIM_1500MA;
		ret = max77759_chg_reg_read(uc_data->client, MAX77759_CHG_CNFG_05,
					    &uc_data->otg_orig);
		if (ret == 0) {
			uc_data->otg_orig &= MAX77759_CHG_CNFG_05_OTG_ILIM_MASK;
		} else {
			uc_data->otg_orig = uc_data->otg_ilim;
		}

		ret = max77759_otg_vbyp_mv_to_code(&uc_data->otg_vbyp,
						   GS101_OTG_VBYPASS_DEFAULT_MV);
		if (ret < 0)
			uc_data->otg_vbyp = MAX77759_CHG_CNFG_11_OTG_VBYP_5100MV;

		return false;
	}

	/* control external boost if present */
	if (uc_data->bst_on == -EPROBE_DEFER)
		uc_data->bst_on = of_get_named_gpio(node, "max77759,bst-on", 0);
	if (uc_data->bst_sel == -EPROBE_DEFER)
		uc_data->bst_sel = of_get_named_gpio(node, "max77759,bst-sel", 0);
	if (uc_data->ext_bst_ctl == -EPROBE_DEFER)
		uc_data->ext_bst_ctl = of_get_named_gpio(node, "max77759,extbst-ctl", 0);

	/* NBC workaround */
	if (uc_data->vin_is_valid == -EPROBE_DEFER)
		uc_data->vin_is_valid = of_get_named_gpio(node, "max77759,vin-is_valid", 0);
	if (uc_data->lsw1_is_closed == -EPROBE_DEFER)
		uc_data->lsw1_is_closed = of_get_named_gpio(node, "max77759,lsw1-is_closed", 0);
	if (uc_data->lsw1_is_open == -EPROBE_DEFER)
		uc_data->lsw1_is_open = of_get_named_gpio(node, "max77759,lsw1-is_open", 0);

	/* all OTG cases, change INOVLO */
	if (uc_data->otg_enable == -EPROBE_DEFER)
		uc_data->otg_enable = of_get_named_gpio(node, "max77759,otg-enable", 0);

	/*  wlc_rx: disable when chgin, CPOUT is safe */
	if (uc_data->wlc_en == -EPROBE_DEFER)
		uc_data->wlc_en = of_get_named_gpio(node, "max77759,wlc-en", 0);
	if (uc_data->wlc_vbus_en == -EPROBE_DEFER)
		uc_data->wlc_vbus_en = of_get_named_gpio(node, "max77759,wlc-vbus_en", 0);
	/*  wlc_rx -> wlc_rx+otg disable cpout */
	if (uc_data->cpout_en == -EPROBE_DEFER)
		uc_data->cpout_en = of_get_named_gpio(node, "max77759,cpout-en", 0);
	/* to 5.2V in p9412 */
	if (uc_data->cpout_ctl == -EPROBE_DEFER)
		uc_data->cpout_ctl = of_get_named_gpio(node, "max77759,cpout-ctl", 0);
	/* ->wlc_tx disable 2:1 cpout */
	if (uc_data->cpout21_en == -EPROBE_DEFER)
		uc_data->cpout21_en = of_get_named_gpio(node, "max77759,cpout_21-en", 0);

	if (uc_data->ls1_en == -EPROBE_DEFER)
		uc_data->ls1_en = of_get_named_gpio(node, "max77759,ls1-en", 0);
	if (uc_data->ls2_en == -EPROBE_DEFER)
		uc_data->ls2_en = of_get_named_gpio(node, "max77759,ls2-en", 0);
	/* OTG+RTXL: IN-OUT switch of AO37 (forced always) */
	if (uc_data->sw_en == -EPROBE_DEFER)
		uc_data->sw_en = of_get_named_gpio(node, "max77759,sw-en", 0);
	/* OPTIONAL: only in P1.1+ (TPS61372) */
	if (uc_data->ext_bst_mode == -EPROBE_DEFER)
		uc_data->ext_bst_mode = of_get_named_gpio(node, "max77759,extbst-mode", 0);

	return gs101_setup_usecases_done(uc_data);
}

static void max77759_dump_usecasase_config(struct max77759_usecase_data *uc_data)
{
	pr_info("bst_on:%d, bst_sel:%d, ext_bst_ctl:%d\n",
		 uc_data->bst_on, uc_data->bst_sel, uc_data->ext_bst_ctl);
	pr_info("vin_valid:%d lsw1_o:%d lsw1_c:%d\n", uc_data->vin_is_valid,
		 uc_data->lsw1_is_open, uc_data->lsw1_is_closed);
	pr_info("wlc_en:%d wlc_vbus_en:%d cpout_en:%d cpout_ctl:%d cpout21_en=%d\n",
		uc_data->wlc_en, uc_data->wlc_vbus_en,
		uc_data->cpout_en, uc_data->cpout_ctl, uc_data->cpout21_en);
	pr_info("ls2_en:%d sw_en:%d ext_bst_mode:%d\n",
		uc_data->ls2_en, uc_data->sw_en, uc_data->ext_bst_mode);
}

/*
 * adjust *INSEL (only one source can be enabled at a given time)
 * NOTE: providing compatibility with input_suspend makes this more complex
 * that it needs to be.
 */
static int max77759_set_insel(struct max77759_usecase_data *uc_data,
			      struct max77759_foreach_cb_data *cb_data,
			      int from_uc, int use_case)
{
	const u8 insel_mask = MAX77759_CHG_CNFG_12_CHGINSEL_MASK |
			      MAX77759_CHG_CNFG_12_WCINSEL_MASK;
	int wlc_on = cb_data->wlc_tx && !cb_data->dc_on;
	bool force_wlc = false;
	u8 insel_value = 0;
	int ret;

	if (cb_data->usb_wlc) {
		insel_value |= MAX77759_CHG_CNFG_12_WCINSEL;
		force_wlc = true;
	} else if (cb_data_is_inflow_off(cb_data)) {
		/*
		 * input_suspend masks both inputs but must still allow
		 * TODO: use a separate use case for usb + wlc
		 */
	} else if (cb_data->buck_on && !cb_data->chgin_off) {
		insel_value |= MAX77759_CHG_CNFG_12_CHGINSEL;
	} else if (cb_data->wlc_rx && !cb_data->wlcin_off) {

		/* always disable WLC when USB is present */
		if (!cb_data->buck_on)
			insel_value |= MAX77759_CHG_CNFG_12_WCINSEL;
		else
			force_wlc = true;

	} else if (cb_data->otg_on) {
		/* all OTG cases MUST mask CHGIN */
		insel_value |= MAX77759_CHG_CNFG_12_WCINSEL;
	} else {
		/* disconnected, do not enable chgin if in input_suspend */
		if (!cb_data->chgin_off)
			insel_value |= MAX77759_CHG_CNFG_12_CHGINSEL;

		/* disconnected, do not enable wlc_in if in input_suspend */
		if (!cb_data->buck_on && (!cb_data->wlcin_off || cb_data->wlc_tx))
			insel_value |= MAX77759_CHG_CNFG_12_WCINSEL;

		force_wlc = true;
	}

	if (from_uc != use_case || force_wlc || wlc_on) {
		wlc_on = wlc_on || (insel_value & MAX77759_CHG_CNFG_12_WCINSEL) != 0;

		/* b/182973431 disable WLC_IC while CHGIN, rtx will enable WLC later */
		ret = gs101_wlc_en(uc_data, wlc_on);
		if (ret < 0)
			pr_err("%s: error wlc_en=%d ret:%d\n", __func__,
			       wlc_on, ret);
	} else {
		u8 value = 0;

		wlc_on = max77759_chg_insel_read(uc_data->client, &value);
		if (wlc_on == 0)
			wlc_on = (value & MAX77759_CHG_CNFG_12_WCINSEL) != 0;
	}

	/* changing [CHGIN|WCIN]_INSEL: works when protection is disabled  */
	ret = max77759_chg_insel_write(uc_data->client, insel_mask, insel_value);

	pr_debug("%s: usecase=%d->%d mask=%x insel=%x wlc_on=%d force_wlc=%d (%d)\n",
		 __func__, from_uc, use_case, insel_mask, insel_value, wlc_on,
		 force_wlc, ret);

	return ret;
}

/* switch to a use case, handle the transitions */
static int max77759_set_usecase(struct max77759_chgr_data *data,
				struct max77759_foreach_cb_data *cb_data,
				int use_case)
{
	struct max77759_usecase_data *uc_data = &data->uc_data;
	const int from_uc = uc_data->use_case;
	int ret;

	if (uc_data->is_a1 == -1) {

		ret = max77759_find_pmic(data);
		if (ret == 0) {
			u8 id, rev;

			ret = max777x9_pmic_get_id(data->pmic_i2c_client, &id, &rev);
			if (ret == 0)
				uc_data->is_a1 = id == MAX77759_PMIC_PMIC_ID_MW &&
						 rev >= MAX77759_PMIC_REV_A0;
		}
	}

	/* Need this only for for usecases that control the switches */
	if (!uc_data->init_done) {
		uc_data->init_done = max77759_setup_usecases(uc_data, data->dev->of_node);

		dev_info(data->dev, "bst_on:%d, bst_sel:%d, ext_bst_ctl:%d lsw1_o:%d lsw1_c:%d\n",
			 uc_data->bst_on, uc_data->bst_sel, uc_data->ext_bst_ctl,
			 uc_data->lsw1_is_open, uc_data->lsw1_is_closed);
	}

	/* always fix/adjust insel (solves multiple input_suspend) */
	ret = max77759_set_insel(uc_data, cb_data, from_uc, use_case);
	if (ret < 0) {
		dev_err(data->dev, "use_case=%d->%d set_insel failed ret:%d\n",
			from_uc, use_case, ret);
		return ret;
	}

	/* usbchg+wlctx will call _set_insel() multiple times. */
	if (from_uc == use_case)
		goto exit_done;

	/* transition to STBY if requested from the use case. */
	ret = max77759_to_standby(uc_data, use_case);
	if (ret < 0) {
		dev_err(data->dev, "use_case=%d->%d to_stby failed ret:%d\n",
			from_uc, use_case, ret);
		return ret;
	}

	/* transition from data->use_case to use_case */
	ret = max77759_to_usecase(uc_data, use_case);
	if (ret < 0) {
		dev_err(data->dev, "use_case=%d->%d to_usecase failed ret:%d\n",
			from_uc, use_case, ret);
		return ret;
	}

exit_done:

	/* finally set mode register */
	ret = max77759_chg_reg_write(uc_data->client, MAX77759_CHG_CNFG_00,
				     cb_data->reg);
	pr_debug("%s: CHARGER_MODE=%x ret:%x\n", __func__, cb_data->reg, ret);
	if (ret < 0) {
		dev_err(data->dev,  "use_case=%d->%d CNFG_00=%x failed ret:%d\n",
			from_uc, use_case, cb_data->reg, ret);
		return ret;
	}

	return ret;
}

static int max77759_wcin_is_online(struct max77759_chgr_data *data);

/*
 * I am using a the comparator_none, need scan all the votes to determine
 * the actual.
 */
static void max77759_mode_callback(struct gvotable_election *el,
				   const char *trigger, void *value)
{
	struct max77759_chgr_data *data = gvotable_get_data(el);
	const int from_use_case = data->uc_data.use_case;
	struct max77759_foreach_cb_data cb_data = { 0 };
	const char *reason;
	int use_case, ret;
	bool nope;
	u8 reg;

	__pm_stay_awake(data->usecase_wake_lock);
	mutex_lock(&data->io_lock);

	reason = trigger;
	use_case = data->uc_data.use_case;

	/* no caching */
	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_00, &reg);
	if (ret < 0) {
		dev_err(data->dev, "cannot read CNFG_00 (%d)\n", ret);
		goto unlock_done;
	}

	/* Need to switch to MW (turn off dc_on) and enforce no charging  */
	cb_data.charge_done = data->charge_done;

	/* this is the last vote of the election */
	cb_data.reg = reg;	/* current */
	cb_data.el = el;	/* election */

	/* read directly instead of using the vote */
	cb_data.wlc_rx = max77759_wcin_is_online(data) &&
			 !data->wcin_input_suspend;
	cb_data.wlcin_off = !!data->wcin_input_suspend;
	/* now scan all the reasons, accumulate in cb_data */
	gvotable_election_for_each(el, max77759_foreach_callback, &cb_data);

	nope = !cb_data.use_raw && !cb_data.stby_on && !cb_data.dc_on &&
	       !cb_data.chgr_on && !cb_data.buck_on && ! cb_data.boost_on &&
	       !cb_data.otg_on && !cb_data.uno_on && !cb_data.wlc_tx &&
	       !cb_data.wlc_rx && !cb_data.wlcin_off && !cb_data.chgin_off &&
	       !cb_data.usb_wlc;
	if (nope) {
		pr_debug("%s: nope callback\n", __func__);
		goto unlock_done;
	}

	dev_info(data->dev, "%s:%s full=%d raw=%d stby_on=%d, dc_on=%d, chgr_on=%d, buck_on=%d,"
		" boost_on=%d, otg_on=%d, uno_on=%d wlc_tx=%d wlc_rx=%d usb_wlc=%d"
		" chgin_off=%d wlcin_off=%d frs_on=%d\n",
		__func__, trigger ? trigger : "<>",
		data->charge_done, cb_data.use_raw, cb_data.stby_on, cb_data.dc_on,
		cb_data.chgr_on, cb_data.buck_on, cb_data.boost_on, cb_data.otg_on,
		cb_data.uno_on, cb_data.wlc_tx, cb_data.wlc_rx, cb_data.usb_wlc,
		cb_data.chgin_off, cb_data.wlcin_off, cb_data.frs_on);

	/* just use raw "as is", no changes to switches etc */
	if (cb_data.use_raw) {
		cb_data.reg = cb_data.raw_value;
		use_case = GSU_RAW_MODE;
	} else {
		struct max77759_usecase_data *uc_data = &data->uc_data;
		bool use_internal_bst;

		/* insel needs it, otg usecases needs it */
		if (!uc_data->init_done) {
			uc_data->init_done = max77759_setup_usecases(uc_data,
						data->dev->of_node);
			max77759_dump_usecasase_config(uc_data);
		}

		/*
		 * force FRS if ext boost or NBC is not enabled
		 * TODO: move to setup_usecase
		 */
		use_internal_bst = uc_data->vin_is_valid < 0 &&
				   uc_data->bst_on < 0;
		if (cb_data.otg_on && use_internal_bst)
			cb_data.frs_on = cb_data.otg_on;

		/* figure out next use case if not in raw mode */
		use_case = max77759_get_usecase(&cb_data);
		if (use_case < 0) {
			dev_err(data->dev, "no valid use case %d\n", use_case);
			goto unlock_done;
		}
	}

	/* state machine that handle transition between states */
	ret = max77759_set_usecase(data, &cb_data, use_case);
	if (ret < 0) {
		ret = max77759_force_standby(data);
		if (ret < 0) {
			dev_err(data->dev, "use_case=%d->%d force_stby failed ret:%d\n",
				data->uc_data.use_case, use_case, ret);
			goto unlock_done;
		}

		cb_data.reg = MAX77759_CHGR_MODE_ALL_OFF;
		cb_data.reason = "error";
		use_case = GSU_MODE_STANDBY;
	}

	/* the election is an int election */
	if (cb_data.reason)
		reason = cb_data.reason;
	if (!reason)
		reason = "<>";

	/* this changes the trigger */
	ret = gvotable_election_set_result(el, reason, (void*)(uintptr_t)cb_data.reg);
	if (ret < 0) {
		dev_err(data->dev, "cannot update election %d\n", ret);
		goto unlock_done;
	}

	/* mode */
	data->uc_data.use_case = use_case;

unlock_done:
	dev_info(data->dev, "%s:%s use_case=%d->%d CHG_CNFG_00=%x->%x\n",
		 __func__, trigger ? trigger : "<>",
		 from_use_case, use_case,
		 reg, cb_data.reg);
	mutex_unlock(&data->io_lock);
	__pm_relax(data->usecase_wake_lock);
}

static int max77759_get_charge_enabled(struct max77759_chgr_data *data,
				       int *enabled)
{
	int ret;
	const void *vote = (const void *)0;

	ret = gvotable_get_current_vote(data->mode_votable, &vote);
	if (ret < 0)
		return ret;

	switch ((uintptr_t)vote) {
	case MAX77759_CHGR_MODE_CHGR_BUCK_ON:
	case MAX77759_CHGR_MODE_CHGR_BUCK_BOOST_UNO_ON:
	case MAX77759_CHGR_MODE_CHGR_OTG_BUCK_BOOST_ON:
		*enabled = 1;
		break;
	default:
		*enabled = 0;
		break;
	}

	return ret;
}

/* reset charge_done if needed on cc_max!=0 and on charge_disable(false) */
static int max77759_enable_sw_recharge(struct max77759_chgr_data *data,
				       bool force)
{
	struct max77759_usecase_data *uc_data = &data->uc_data;
	const bool charge_done = data->charge_done;
	bool needs_restart = force || data->charge_done;
	uint8_t reg;
	int ret;

	if (!needs_restart) {
		ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_01, &reg);
		needs_restart = (ret < 0) ||
				_chg_details_01_chg_dtls_get(reg) == CHGR_DTLS_DONE_MODE;
		if (!needs_restart)
			return 0;
	}

	/* This: will not trigger the usecase state machine */
	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_00, &reg);
	if (ret == 0)
		ret = max77759_chg_mode_write(uc_data->client, MAX77759_CHGR_MODE_ALL_OFF);
	if (ret == 0)
		ret = max77759_chg_mode_write(uc_data->client, reg);

	data->charge_done = false;

	pr_debug("%s charge_done=%d->0, reg=%hhx (%d)\n", __func__,
		 charge_done, reg, ret);

	return ret;
}

/* called from gcpm and for CC_MAX == 0 */
static int max77759_set_charge_enabled(struct max77759_chgr_data *data,
				       int enabled, const char *reason)
{
	/* ->charge_done is reset in max77759_enable_sw_recharge() */

	return gvotable_cast_vote(data->mode_votable, reason,
				  (void*)GBMS_CHGR_MODE_CHGR_BUCK_ON,
				  enabled);
}

/* google_charger on disconnect */
static int max77759_set_charge_disable(struct max77759_chgr_data *data,
				       int enabled, const char *reason)
{
	/* make sure charging is restarted on enable */
	if (enabled) {
		int ret;

		ret = max77759_enable_sw_recharge(data, false);
		if (ret < 0)
			dev_err(data->dev, "%s cannot re-enable charging (%d)\n",
				__func__, ret);
	}

	return gvotable_cast_vote(data->mode_votable, reason,
				  (void*)GBMS_CHGR_MODE_STBY_ON,
				  enabled);
}

static int max77759_chgin_input_suspend(struct max77759_chgr_data *data,
					bool enabled, const char *reason)
{
	int ret;

	ret = gvotable_cast_vote(data->mode_votable, "CHGIN_SUSP",
				 (void*)GBMS_CHGR_MODE_CHGIN_OFF,
				 enabled);
	if (ret == 0)
		data->chgin_input_suspend = enabled; /* cache */

	return ret;
}

static int max77759_wcin_input_suspend(struct max77759_chgr_data *data,
				       bool enabled, const char *reason)
{
	int ret;

	ret = gvotable_cast_vote(data->mode_votable, "WCIN_SUSP",
				 (void*)GBMS_CHGR_MODE_WLCIN_OFF,
				 enabled);
	if (ret == 0)
		data->wcin_input_suspend = enabled; /* cache */

	return ret;
}

static int max77759_set_regulation_voltage(struct max77759_chgr_data *data,
					   int voltage_uv)
{
	u8 value;

	if (voltage_uv >= 4500000)
		value = 0x32;
	else if (voltage_uv < 4000000)
		value = 0x38 + (voltage_uv - 3800000) / 100000;
	else
		value = (voltage_uv - 4000000) / 10000;

	value = VALUE2FIELD(MAX77759_CHG_CNFG_04_CHG_CV_PRM, value);
	return max77759_reg_update(data, MAX77759_CHG_CNFG_04,
				   MAX77759_CHG_CNFG_04_CHG_CV_PRM_MASK,
				   value);
}

static int max77759_get_regulation_voltage_uv(struct max77759_chgr_data *data,
					      int *voltage_uv)
{
	u8 value;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_04, &value);
	if (ret < 0)
		return ret;

	if  (value < 0x33)
		*voltage_uv = (4000 + value * 10) * 1000;
	else if (value == 0x38)
		*voltage_uv = 3800 * 1000;
	else if (value == 0x39)
		*voltage_uv = 3900 * 1000;
	else
		return -EINVAL;

	return 0;
}

/* set charging current to 0 to disable charging (MODE=0) */
static int max77759_set_charger_current_max_ua(struct max77759_chgr_data *data,
					       int current_ua)
{
	const int disabled = current_ua == 0;
	u8 value;
	int ret;

	if (current_ua < 0)
		return 0;

	/* ilim=0 -> switch to mode 0 and suspend charging */
	if  (current_ua == 0)
		value = 0x0;
	else if (current_ua <= 200000)
		value = 0x03;
	else if (current_ua >= 4000000)
		value = 0x3c;
	else
		value = 0x3 + (current_ua - 200000) / 66670;

	/*
	 * cc_max > 0 might need to restart charging: the usecase state machine
	 * will be triggered in max77759_set_charge_enabled()
	 */
	if (current_ua) {
		ret = max77759_enable_sw_recharge(data, false);
		if (ret < 0)
			dev_err(data->dev, "cannot re-enable charging (%d)\n", ret);
	}

	value = VALUE2FIELD(MAX77759_CHG_CNFG_02_CHGCC, value);
	ret = max77759_reg_update(data, MAX77759_CHG_CNFG_02,
				   MAX77759_CHG_CNFG_02_CHGCC_MASK,
				   value);
	if (ret == 0)
		ret = max77759_set_charge_enabled(data, !disabled, "CC_MAX");

	return ret;
}

static int max77759_get_charger_current_max_ua(struct max77759_chgr_data *data,
					       int *current_ua)
{
	u8 value;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_02,
				&value);
	if (ret < 0)
		return ret;

	/* TODO: fix the rounding */
	value = VALUE2FIELD(MAX77759_CHG_CNFG_02_CHGCC, value);

	/* ilim=0 -> mode 0 with charging suspended */
	if (value == 0)
		*current_ua = 0;
	else if (value < 3)
		*current_ua = 133 * 1000;
	else if (value >= 0x3C)
		*current_ua = 4000 * 1000;
	else
		*current_ua = 133000 + (value - 2) * 66670;

	return 0;
}

/* enable autoibus and charger mode */
static int max77759_chgin_set_ilim_max_ua(struct max77759_chgr_data *data,
					  int ilim_ua)
{
	const bool suspend = ilim_ua == 0;
	u8 value;
	int ret;

	/* TODO: disable charging */
	if (ilim_ua < 0)
		return 0;

	if (ilim_ua == 0)
		value = 0x00;
	else if (ilim_ua > 3200000)
		value = 0x7f;
	else
		value = 0x04 + (ilim_ua - 125000) / 25000;

	value = VALUE2FIELD(MAX77759_CHG_CNFG_09_NO_AUTOIBUS, 1) |
		VALUE2FIELD(MAX77759_CHG_CNFG_09_CHGIN_ILIM, value);
	ret = max77759_reg_update(data, MAX77759_CHG_CNFG_09,
					MAX77759_CHG_CNFG_09_NO_AUTOIBUS |
					MAX77759_CHG_CNFG_09_CHGIN_ILIM_MASK,
					value);
	if (ret == 0)
		ret = max77759_chgin_input_suspend(data, suspend, "ILIM");

	return ret;
}

static int max77759_chgin_get_ilim_max_ua(struct max77759_chgr_data *data,
					  int *ilim_ua)
{
	int icl, ret;
	u8 value;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_09, &value);
	if (ret < 0)
		return ret;

	value = FIELD2VALUE(MAX77759_CHG_CNFG_09_CHGIN_ILIM, value);
	if (value == 0)
		icl = 0;
	else if (value > 3)
		icl = 100 + (value - 3) * 25;
	else
		icl = 100;

	*ilim_ua = icl * 1000;

	if (data->chgin_input_suspend)
		*ilim_ua = 0;

	return 0;
}

static int max77759_wcin_set_ilim_max_ua(struct max77759_chgr_data *data,
					 int ilim_ua)
{
	u8 value;
	int ret;

	if (ilim_ua < 0)
		return -EINVAL;

	if (ilim_ua == 0)
		value = 0x00;
	else if (ilim_ua <= 125000)
		value = 0x01;
	else
		value = 0x3 + (ilim_ua - 125000) / 31250;

	value = VALUE2FIELD(MAX77759_CHG_CNFG_10_WCIN_ILIM, value);
	ret = max77759_reg_update(data, MAX77759_CHG_CNFG_10,
					MAX77759_CHG_CNFG_10_WCIN_ILIM_MASK,
					value);

	/* Legacy: DC_ICL doesn't suspend on ilim_ua == 0 (it should) */

	return ret;
}

static int max77759_wcin_get_ilim_max_ua(struct max77759_chgr_data *data,
					 int *ilim_ua)
{
	int ret;
	u8 value;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_10, &value);
	if (ret < 0)
		return ret;

	value = FIELD2VALUE(MAX77759_CHG_CNFG_10_WCIN_ILIM, value);
	if (value == 0)
		*ilim_ua = 0;
	else if (value < 4)
		*ilim_ua = 125000;
	else
		*ilim_ua = 125000 + (value - 3) * 31250;

	if (data->wcin_input_suspend)
		*ilim_ua = 0;

	return 0;
}

/* default is no suspend, any valid vote will suspend  */
static void max77759_dc_suspend_vote_callback(struct gvotable_election *el,
					      const char *reason, void *value)
{
	struct max77759_chgr_data *data = gvotable_get_data(el);
	int ret, suspend = (long)value > 0;

	/* will trigger a CHARGER_MODE callback */
	ret = max77759_wcin_input_suspend(data, suspend, "DC_SUSPEND");
	if (ret < 0)
		return;

	pr_debug("%s: DC_SUSPEND reason=%s, value=%ld suspend=%d (%d)\n",
		 __func__, reason ? reason : "", (long)value, suspend, ret);
}

static void max77759_dcicl_callback(struct gvotable_election *el,
				    const char *reason,
				    void *value)
{
	struct max77759_chgr_data *data = gvotable_get_data(el);
	int dc_icl = (long)value;
	const bool suspend = dc_icl == 0;
	int ret;

	pr_debug("%s: DC_ICL reason=%s, value=%ld suspend=%d\n",
		 __func__, reason ? reason : "", (long)value, suspend);

	ret = max77759_wcin_set_ilim_max_ua(data, dc_icl);
	if (ret < 0)
		dev_err(data->dev, "cannot set dc_icl=%d (%d)\n",
			dc_icl, ret);

	/* will trigger a CHARGER_MODE callback */
	ret = max77759_wcin_input_suspend(data, suspend, "DC_ICL");
	if (ret < 0)
		dev_err(data->dev, "cannot set suspend=%d (%d)\n",
			suspend, ret);

}

/*************************
 * WCIN PSY REGISTRATION   *
 *************************/
static enum power_supply_property max77759_wcin_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

static int max77759_wcin_is_valid(struct max77759_chgr_data *data)
{
	uint8_t int_ok;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_INT_OK, &int_ok);
	return (ret == 0) && _chg_int_ok_wcin_ok_get(int_ok);
}

static int max77759_wcin_is_online(struct max77759_chgr_data *data)
{
	uint8_t val;
	int ret;

	ret = max77759_wcin_is_valid(data);
	if (ret <= 0)
		return ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_02, &val);
	return (ret == 0) && _chg_details_02_wcin_sts_get(val);
}

/* TODO: make this configurable */
static struct power_supply* max77759_get_wlc_psy(struct max77759_chgr_data *chg)
{
	if (!chg->wlc_psy)
		chg->wlc_psy = power_supply_get_by_name("wireless");
	return chg->wlc_psy;
}

static int max77759_wcin_voltage_max(struct max77759_chgr_data *chg,
				     union power_supply_propval *val)
{
	struct power_supply *wlc_psy;
	int rc;

	if (!max77759_wcin_is_valid(chg)) {
		val->intval = 0;
		return 0;
	}

	wlc_psy = max77759_get_wlc_psy(chg);
	if (!wlc_psy)
		return max77759_get_regulation_voltage_uv(chg, &val->intval);

	rc = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, val);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get VOLTAGE_MAX, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int max77759_wcin_voltage_now(struct max77759_chgr_data *chg,
				     union power_supply_propval *val)
{
	struct power_supply *wlc_psy;
	int rc;

	if (!max77759_wcin_is_valid(chg)) {
		val->intval = 0;
		return 0;
	}

	wlc_psy = max77759_get_wlc_psy(chg);
	if (!wlc_psy)
		return max77759_read_vbatt(chg, &val->intval);

	rc = power_supply_get_property(wlc_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get VOLTAGE_NOW, rc=%d\n", rc);

	return rc;
}

#define MAX77759_WCIN_RAW_TO_UA	156

/* current is valid only when charger mode is one of the following */
static bool max77759_current_check_mode(struct max77759_chgr_data *data)
{
	int ret;
	u8 reg;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_00, &reg);
	if (ret < 0)
		return false;

	return reg == 5 || reg == 6 || reg == 7 || reg == 0xe || reg == 0xf;
}

/* only valid in mode 5, 6, 7, e, f */
static int max77759_wcin_current_now(struct max77759_chgr_data *data, int *iic)
{
	int ret, iic_raw;

	ret = max77759_find_fg(data);
	if (ret < 0)
		return ret;

	ret = max_m5_read_actual_input_current_ua(data->fg_i2c_client, &iic_raw);
	if (ret < 0)
		return ret;

	*iic = iic_raw * MAX77759_WCIN_RAW_TO_UA;
	return 0;
}

static int max77759_wcin_get_prop(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct max77759_chgr_data *chgr = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = max77759_wcin_is_valid(chgr);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = max77759_wcin_is_online(chgr);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = max77759_wcin_voltage_now(chgr, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = max77759_wcin_get_ilim_max_ua(chgr, &val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = max77759_wcin_voltage_max(chgr, val);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = 0;
		if (max77759_wcin_is_online(chgr))
			rc = max77759_wcin_current_now(chgr, &val->intval);
		break;
	default:
		return -EINVAL;
	}
	if (rc < 0) {
		pr_debug("Couldn't get prop %d rc = %d\n", psp, rc);
		return -ENODATA;
	}
	return 0;
}

static int max77759_wcin_set_prop(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct max77759_chgr_data *chgr = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = max77759_wcin_set_ilim_max_ua(chgr, val->intval);
		break;
	/* called from google_cpm when switching chargers */
	case GBMS_PROP_CHARGING_ENABLED:
		rc = max77759_set_charge_enabled(chgr, val->intval > 0,
						 "DC_PSP_ENABLED");
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int max77759_wcin_prop_is_writeable(struct power_supply *psy,
					   enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case GBMS_PROP_CHARGING_ENABLED:
		return 1;
	default:
		break;
	}

	return 0;
}

static struct power_supply_desc max77759_wcin_psy_desc = {
	.name = "dc",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = max77759_wcin_props,
	.num_properties = ARRAY_SIZE(max77759_wcin_props),
	.get_property = max77759_wcin_get_prop,
	.set_property = max77759_wcin_set_prop,
	.property_is_writeable = max77759_wcin_prop_is_writeable,
};

static int max77759_init_wcin_psy(struct max77759_chgr_data *data)
{
	struct power_supply_config wcin_cfg = {};
	struct device *dev = data->dev;
	const char *name;
	int ret;

	wcin_cfg.drv_data = data;
	wcin_cfg.of_node = dev->of_node;

	if (of_property_read_bool(dev->of_node, "max77759,dc-psy-type-wireless"))
		max77759_wcin_psy_desc.type = POWER_SUPPLY_TYPE_WIRELESS;

	ret = of_property_read_string(dev->of_node, "max77759,dc-psy-name", &name);
	if (ret == 0) {
		max77759_wcin_psy_desc.name = devm_kstrdup(dev, name, GFP_KERNEL);
		if (!max77759_wcin_psy_desc.name)
			return -ENOMEM;
	}

	data->wcin_psy = devm_power_supply_register(data->dev,
					&max77759_wcin_psy_desc, &wcin_cfg);
	if (IS_ERR(data->wcin_psy))
		return PTR_ERR(data->wcin_psy);

	return 0;
}


static int max77759_chg_is_valid(struct max77759_chgr_data *data)
{
	uint8_t int_ok;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_INT_OK, &int_ok);
	return (ret == 0) && _chg_int_ok_chgin_ok_get(int_ok);
}

static int max77759_chgin_is_online(struct max77759_chgr_data *data)
{
	uint8_t val;
	int ret;

	ret = max77759_chg_is_valid(data);
	if (ret <= 0)
		return ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_02, &val);
	return (ret == 0) && _chg_details_02_chgin_sts_get(val);
}

/*
 * NOTE: could also check aicl to determine whether the adapter is, in fact,
 * at fault. Possibly qualify this with battery voltage as subpar adapters
 * are likely to flag AICL when the battery is at high voltage.
 */
static int max77759_is_limited(struct max77759_chgr_data *data)
{
	int ret;
	u8 value;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_INT_OK, &value);
	return (ret == 0) && _chg_int_ok_inlim_ok_get(value) == 0;
}

static int max77759_is_valid(struct max77759_chgr_data *data)
{
	uint8_t int_ok;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_INT_OK, &int_ok);
	return (ret == 0) && (_chg_int_ok_chgin_ok_get(int_ok) ||
	       _chg_int_ok_wcin_ok_get(int_ok));
}

/* WCIN || CHGIN present, valid  && CHGIN FET is closed */
static int max77759_is_online(struct max77759_chgr_data *data)
{
	uint8_t val;
	int ret;

	ret = max77759_is_valid(data);
	if (ret <= 0)
		return 0;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_02, &val);
	return (ret == 0) && (_chg_details_02_chgin_sts_get(val) ||
	       _chg_details_02_wcin_sts_get(val));
}

static int max77759_get_charge_type(struct max77759_chgr_data *data)
{
	int ret;
	uint8_t reg;

	if (!max77759_is_online(data))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_01, &reg);
	if (ret < 0)
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	switch(_chg_details_01_chg_dtls_get(reg)) {
	case CHGR_DTLS_DEAD_BATTERY_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case CHGR_DTLS_FAST_CHARGE_CONST_CURRENT_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case CHGR_DTLS_FAST_CHARGE_CONST_VOLTAGE_MODE:
	case CHGR_DTLS_TOP_OFF_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT;

	case CHGR_DTLS_DONE_MODE:
	case CHGR_DTLS_TIMER_FAULT_MODE:
	case CHGR_DTLS_DETBAT_HIGH_SUSPEND_MODE:
	case CHGR_DTLS_OFF_MODE:
	case CHGR_DTLS_OFF_HIGH_TEMP_MODE:
	case CHGR_DTLS_OFF_WATCHDOG_MODE:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		break;
	}

	return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
}

static bool max77759_is_full(struct max77759_chgr_data *data)
{
	int vlimit = data->chg_term_voltage;
	int ret, vbatt = 0;

	/*
	 * Set voltage level to leave CHARGER_DONE (BATT_RL_STATUS_DISCHARGE)
	 * and enter BATT_RL_STATUS_RECHARGE. It sets STATUS_DISCHARGE again
	 * once CHARGER_DONE flag set (return true here)
	 */
	ret = max77759_read_vbatt(data, &vbatt);
	if (ret == 0)
		vbatt = vbatt / 1000;

	if (data->charge_done)
		vlimit -= data->chg_term_volt_debounce;

	/* true when chg_term_voltage==0, false if read error (vbatt==0) */
	return vbatt >= vlimit;
}

static int max77759_get_status(struct max77759_chgr_data *data)
{
	uint8_t val;
	int ret;

	if (!max77759_is_online(data))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	/*
	 * EOC can be made sticky returning POWER_SUPPLY_STATUS_FULL on
	 * ->charge_done. Also need a check on max77759_is_full() or
	 * google_charger will fail to restart charging.
	 */
	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_01, &val);
	if (ret < 0)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	switch (_chg_details_01_chg_dtls_get(val)) {
		case CHGR_DTLS_DEAD_BATTERY_MODE:
		case CHGR_DTLS_FAST_CHARGE_CONST_CURRENT_MODE:
		case CHGR_DTLS_FAST_CHARGE_CONST_VOLTAGE_MODE:
		case CHGR_DTLS_TOP_OFF_MODE:
			return POWER_SUPPLY_STATUS_CHARGING;
		case CHGR_DTLS_DONE_MODE:
			/* same as POWER_SUPPLY_PROP_CHARGE_DONE */
			if (!max77759_is_full(data))
				data->charge_done = false;
			if (data->charge_done)
				return POWER_SUPPLY_STATUS_FULL;
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
		case CHGR_DTLS_TIMER_FAULT_MODE:
		case CHGR_DTLS_DETBAT_HIGH_SUSPEND_MODE:
		case CHGR_DTLS_OFF_MODE:
		case CHGR_DTLS_OFF_HIGH_TEMP_MODE:
		case CHGR_DTLS_OFF_WATCHDOG_MODE:
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
		default:
			break;
	}

	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static int max77759_get_chg_chgr_state(struct max77759_chgr_data *data,
				       union gbms_charger_state *chg_state)
{
	int usb_present, usb_valid, dc_present, dc_valid;
	const char *source = "";
	uint8_t int_ok, dtls;
	int vbatt, icl = 0;
	int rc;

	chg_state->v = 0;
	chg_state->f.chg_status = max77759_get_status(data);
	chg_state->f.chg_type = max77759_get_charge_type(data);
	chg_state->f.flags = gbms_gen_chg_flags(chg_state->f.chg_status,
						chg_state->f.chg_type);

	rc = max77759_reg_read(data->regmap, MAX77759_CHG_INT_OK, &int_ok);
	if (rc == 0)
		rc = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_02,
					&dtls);

	/* present when connected, valid when FET is closed */
	usb_present = (rc == 0) && _chg_int_ok_chgin_ok_get(int_ok);
	usb_valid = usb_present && _chg_details_02_chgin_sts_get(dtls);

	/* present if in field, valid when FET is closed */
	dc_present = (rc == 0) && _chg_int_ok_wcin_ok_get(int_ok);
	dc_valid = dc_present && _chg_details_02_wcin_sts_get(dtls);

	rc = max77759_read_vbatt(data, &vbatt);
	if (rc == 0)
		chg_state->f.vchrg = vbatt / 1000;

	if (chg_state->f.chg_status == POWER_SUPPLY_STATUS_DISCHARGING)
		goto exit_done;

	rc = max77759_is_limited(data);
	if (rc > 0)
		chg_state->f.flags |= GBMS_CS_FLAG_ILIM;

	/* TODO: b/ handle input MUX corner cases */
	if (usb_valid) {
		max77759_chgin_get_ilim_max_ua(data, &icl);
		/* TODO: 'u' only when in sink */
		if (!dc_present)
			source = "U";
		 else if (dc_valid)
			source = "UW";
		 else
			source = "Uw";

	} else if (dc_valid) {
		max77759_wcin_get_ilim_max_ua(data, &icl);

		/* TODO: 'u' only when in sink */
		source = usb_present ? "uW" : "W";
	} else if (usb_present && dc_present) {
		source = "uw";
	} else if (usb_present) {
		source = "u";
	} else if (dc_present) {
		source = "w";
	}

	chg_state->f.icl = icl / 1000;

exit_done:
	pr_debug("MSC_PCS chg_state=%lx [0x%x:%d:%d:%d:%d] chg=%s\n",
		 (unsigned long)chg_state->v,
		 chg_state->f.flags,
		 chg_state->f.chg_type,
		 chg_state->f.chg_status,
		 chg_state->f.vchrg,
		 chg_state->f.icl,
		 source);

	return 0;
}

#define MAX77759_CHGIN_RAW_TO_UA	125

/* only valid in mode 5, 6, 7, e, f */
static int max77759_chgin_current_now(struct max77759_chgr_data *data, int *iic)
{
	int ret, iic_raw;

	ret = max77759_find_fg(data);
	if (ret < 0)
		return ret;

	ret = max_m5_read_actual_input_current_ua(data->fg_i2c_client, &iic_raw);
	if (ret < 0)
		return ret;

	*iic = iic_raw * MAX77759_CHGIN_RAW_TO_UA;
	return 0;
}

static int max77759_wd_tickle(struct max77759_chgr_data *data)
{
	int ret;
	u8 reg, reg_new;

	mutex_lock(&data->io_lock);
	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_00, &reg);
	if (ret == 0) {
		reg_new  = _chg_cnfg_00_wdtclr_set(reg, 0x1);
		ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_00,
					 reg_new);
	}

	if (ret < 0)
		pr_err("WD Tickle failed %d\n", ret);

	mutex_unlock(&data->io_lock);
	return ret;
}

/* online is used from DC charging to tickle the watchdog (if enabled) */
static int max77759_set_online(struct max77759_chgr_data *data, bool online)
{
	int ret = 0;

	if (data->wden) {
		ret = max77759_wd_tickle(data);
		if (ret < 0)
			pr_err("cannot tickle the watchdog\n");
	}

	if (data->online != online) {
		ret = gvotable_cast_vote(data->mode_votable, "OFFLINE",
					 (void *)GBMS_CHGR_MODE_STBY_ON,
					 !online);
		data->online = online;
	}

	return ret;
}

static int max77759_psy_set_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     const union power_supply_propval *pval)
{
	struct max77759_chgr_data *data = power_supply_get_drvdata(psy);
	int ret = -EINVAL;

	pm_runtime_get_sync(data->dev);
	if (!data->init_complete || !data->resume_complete) {
		pm_runtime_put_sync(data->dev);
		return -EAGAIN;
	}
	pm_runtime_put_sync(data->dev);

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = max77759_chgin_set_ilim_max_ua(data, pval->intval);
		pr_debug("%s: icl=%d (%d)\n", __func__, pval->intval, ret);
		break;
	/* Charge current is set to 0 to EOC */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = max77759_set_charger_current_max_ua(data, pval->intval);
		pr_debug("%s: charge_current=%d (%d)\n",
			__func__, pval->intval, ret);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = max77759_set_regulation_voltage(data, pval->intval);
		pr_debug("%s: charge_voltage=%d (%d)\n",
			__func__, pval->intval, ret);
		break;
	/* called from google_cpm when switching chargers */
	case GBMS_PROP_CHARGING_ENABLED:
		ret = max77759_set_charge_enabled(data, pval->intval,
						  "PSP_ENABLED");
		pr_debug("%s: charging_enabled=%d (%d)\n",
			__func__, pval->intval, ret);
		break;
	/* called from google_charger on disconnect */
	case GBMS_PROP_CHARGE_DISABLE:
		ret = max77759_set_charge_disable(data, pval->intval,
						  "PSP_DISABLE");
		pr_debug("%s: charge_disable=%d (%d)\n",
			__func__, pval->intval, ret);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = max77759_set_online(data, pval->intval != 0);
		break;
	default:
		break;
	}

	if (ret == 0 && data->wden)
		max77759_wd_tickle(data);


	return ret;
}

static int max77759_psy_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *pval)
{
	struct max77759_chgr_data *data = power_supply_get_drvdata(psy);
	union gbms_charger_state chg_state;
	int rc, ret = 0;

	pm_runtime_get_sync(data->dev);
	if (!data->init_complete || !data->resume_complete) {
		pm_runtime_put_sync(data->dev);
		return -EAGAIN;
	}
	pm_runtime_put_sync(data->dev);

	switch (psp) {
	case GBMS_PROP_CHARGE_DISABLE:
		rc = max77759_get_charge_enabled(data, &pval->intval);
		if (rc == 0)
			pval->intval = !pval->intval;
		else
			pval->intval = rc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		pval->intval = max77759_get_charge_type(data);
		break;
	case GBMS_PROP_CHARGING_ENABLED:
		ret = max77759_get_charge_enabled(data, &pval->intval);
		break;
	case GBMS_PROP_CHARGE_CHARGER_STATE:
		rc = max77759_get_chg_chgr_state(data, &chg_state);
		if (rc == 0)
			gbms_propval_int64val(pval) = chg_state.v;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = max77759_get_charger_current_max_ua(data, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = max77759_get_regulation_voltage_uv(data, &pval->intval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		pval->intval = max77759_is_online(data);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		pval->intval = max77759_is_valid(data);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = max77759_chgin_get_ilim_max_ua(data, &pval->intval);
		break;
	case GBMS_PROP_INPUT_CURRENT_LIMITED:
		pval->intval = max77759_is_limited(data);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		pval->intval = max77759_get_status(data);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = max77759_read_vbatt(data, &pval->intval);
		if (rc < 0)
			pval->intval = -1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (!max77759_current_check_mode(data)) {
			pval->intval = 0;
			break;
		}

		if (max77759_wcin_is_online(data))
			rc = max77759_wcin_current_now(data, &pval->intval);
		else if (max77759_chgin_is_online(data))
			rc = max77759_chgin_current_now(data, &pval->intval);
		else
			rc = -EINVAL;

		if (rc < 0)
			pval->intval = rc;
		break;
	default:
		dev_err(data->dev, "property (%d) unsupported.\n", psp);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int max77759_psy_is_writeable(struct power_supply *psy,
				 enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX: /* compat, same the next */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case GBMS_PROP_CHARGE_DISABLE:
		return 1;
	default:
		break;
	}

	return 0;
}

/*
 * TODO: POWER_SUPPLY_PROP_RERUN_AICL, POWER_SUPPLY_PROP_TEMP
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX
 */
static enum power_supply_property max77759_psy_props[] = {
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,		/* compat */
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_STATUS,
};

static struct power_supply_desc max77759_psy_desc = {
	.name = "max77759-charger",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = max77759_psy_props,
	.num_properties = ARRAY_SIZE(max77759_psy_props),
	.get_property = max77759_psy_get_property,
	.set_property = max77759_psy_set_property,
	.property_is_writeable = max77759_psy_is_writeable,
};

static ssize_t show_fship_dtls(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct max77759_chgr_data *data = dev_get_drvdata(dev);
	static char *fship_reason[] = {"None", "PWRONB1", "PWRONB1", "PWR"};
	u8 pmic_rd;
	int ret;

	if (data->fship_dtls != -1)
		goto exit_done;


	ret = max77759_find_pmic(data);
	if (ret < 0)
		return ret;

	ret = max777x9_pmic_reg_read(data->pmic_i2c_client,
				     MAX77759_FSHIP_EXIT_DTLS,
				     &pmic_rd, 1);
	if (ret < 0)
		return -EIO;

	if (pmic_rd & MAX77759_FSHIP_EXIT_DTLS_RD) {
		u8 fship_dtls;

		ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_03,
					&fship_dtls);
		if (ret < 0)
			return -EIO;

		data->fship_dtls =
			_chg_details_03_fship_exit_dtls_get(fship_dtls);

		pmic_rd &= ~MAX77759_FSHIP_EXIT_DTLS_RD;
		ret = max777x9_pmic_reg_write(data->pmic_i2c_client,
					      MAX77759_FSHIP_EXIT_DTLS,
					      &pmic_rd, 1);
		if (ret < 0)
			pr_err("FSHIP: cannot update RD (%d)\n", ret);

	} else {
		data->fship_dtls = 0;
	}

exit_done:
	return scnprintf(buf, PAGE_SIZE, "%d %s\n", data->fship_dtls,
			 fship_reason[data->fship_dtls]);
}

static DEVICE_ATTR(fship_dtls, 0444, show_fship_dtls, NULL);

#if IS_ENABLED(CONFIG_GOOGLE_BCL)
static int vdroop2_ok_get(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_dtls1;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_01, &chg_dtls1);
	if (ret < 0)
		return -ENODEV;

	*val = _chg_details_01_vdroop2_ok_get(chg_dtls1);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vdroop2_ok_fops, vdroop2_ok_get, NULL, "%llu\n");

static int vdp1_stp_bst_get(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg18;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_18, &chg_cnfg18);
	if (ret < 0)
		return -ENODEV;

	*val = _chg_cnfg_18_vdp1_stp_bst_get(chg_cnfg18);
	return 0;
}

static int vdp1_stp_bst_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg18;
	const u8 vdp1_stp_bst = (val > 0)? 0x1 : 0x0;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_18, &chg_cnfg18);
	if (ret < 0)
		return -ENODEV;

	chg_cnfg18 = _chg_cnfg_18_vdp1_stp_bst_set(chg_cnfg18, vdp1_stp_bst);
	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_18, chg_cnfg18);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(vdp1_stp_bst_fops, vdp1_stp_bst_get, vdp1_stp_bst_set, "%llu\n");

static int vdp2_stp_bst_get(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg18;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_18, &chg_cnfg18);
	if (ret < 0)
		return -ENODEV;

	*val = _chg_cnfg_18_vdp2_stp_bst_get(chg_cnfg18);
	return 0;
}

static int vdp2_stp_bst_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg18;
	const u8 vdp2_stp_bst = (val > 0)? 0x1 : 0x0;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_18, &chg_cnfg18);
	if (ret < 0)
		return -ENODEV;

	chg_cnfg18 = _chg_cnfg_18_vdp2_stp_bst_set(chg_cnfg18, vdp2_stp_bst);
	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_18, chg_cnfg18);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(vdp2_stp_bst_fops, vdp2_stp_bst_get, vdp2_stp_bst_set, "%llu\n");

static int bat_oilo_get(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg14;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_14, &chg_cnfg14);
	if (ret < 0)
		return -EIO;

	*val = chg_cnfg14;
	return 0;
}

static int bat_oilo_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	int ret;

	if (val > 0xf)
		return -EINVAL;

	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_14, (u8) val);
	if (ret < 0)
		return -EIO;

	data->triggered_lvl[BATOILO] = (BO_STEP * ((val & MAX77759_CHG_CNFG_14_BAT_OILO_MASK)
			>> MAX77759_CHG_CNFG_14_BAT_OILO_SHIFT) + BO_LOWER_LIMIT)
			- THERMAL_HYST_LEVEL;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(bat_oilo_fops, bat_oilo_get, bat_oilo_set, "0x%llx\n");

static int sys_uvlo1_get(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg15;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_15, &chg_cnfg15);
	if (ret < 0)
		return -EIO;

	*val = chg_cnfg15;
	return 0;
}

static int sys_uvlo1_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	int ret;

	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_15, (u8) val);
	if (ret < 0)
		return -EIO;

	data->triggered_lvl[VDROOP1] = VD_BATTERY_VOLTAGE - (VD_STEP *
			((val & MAX77759_CHG_CNFG_15_SYS_UVLO1_MASK)
			>> MAX77759_CHG_CNFG_15_SYS_UVLO1_SHIFT) + VD_LOWER_LIMIT)
			- THERMAL_HYST_LEVEL;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(sys_uvlo1_fops, sys_uvlo1_get, sys_uvlo1_set, "0x%llx\n");

static int sys_uvlo2_get(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	int ret = 0;
	u8 chg_cnfg16;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_16, &chg_cnfg16);
	if (ret < 0)
		return -EIO;

	*val = chg_cnfg16;
	return 0;
}

static int sys_uvlo2_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	int ret;

	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_16, (u8) val);
	if (ret < 0)
		return -EIO;

	data->triggered_lvl[VDROOP2] = VD_BATTERY_VOLTAGE - (VD_STEP *
			((val & MAX77759_CHG_CNFG_16_SYS_UVLO2_MASK)
			>> MAX77759_CHG_CNFG_16_SYS_UVLO2_SHIFT) + VD_LOWER_LIMIT)
			- THERMAL_HYST_LEVEL;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(sys_uvlo2_fops, sys_uvlo2_get, sys_uvlo2_set, "0x%llx\n");
#endif

/* write to INPUT_MASK_CLR in to re-enable detection */
static int max77759_chgr_input_mask_clear(struct max77759_chgr_data *data)
{
	u8 value;
	int ret;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_10, &value);
	if (ret < 0)
		return -ENODEV;

	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_10,
				 _chg_cnfg_10_input_mask_clr_set(value, 1));
	if (ret < 0)
		pr_err("%s: cannot clear input_mask ret=%d\n", __func__, ret);

	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_10,
				 _chg_cnfg_10_input_mask_clr_set(value, 0));
	if (ret < 0)
		pr_err("%s: cannot reset input_mask ret=%d\n", __func__, ret);

	return ret;
}


static int input_mask_clear_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;

	return max77759_chgr_input_mask_clear(data);
}

DEFINE_SIMPLE_ATTRIBUTE(input_mask_clear_fops, NULL, input_mask_clear_set, "%llu\n");

static int charger_restart_set(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	int ret;

	ret = max77759_enable_sw_recharge(data, !!val);
	dev_info(data->dev, "triggered recharge(force=%d) %d\n", !!val, ret);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(charger_restart_fops, NULL, charger_restart_set, "%llu\n");


static int max77759_chg_debug_reg_read(void *d, u64 *val)
{
	struct max77759_chgr_data *data = d;
	u8 reg = 0;
	int ret;

	ret = max77759_reg_read(data->regmap, data->debug_reg_address, &reg);
	if (ret)
		return ret;

	*val = reg;
	return 0;
}

static int max77759_chg_debug_reg_write(void *d, u64 val)
{
	struct max77759_chgr_data *data = d;
	u8 reg = (u8) val;

	pr_warn("debug write reg 0x%x, 0x%x", data->debug_reg_address, reg);
	return max77759_reg_write(data->regmap, data->debug_reg_address, reg);
}
DEFINE_SIMPLE_ATTRIBUTE(debug_reg_rw_fops, max77759_chg_debug_reg_read,
			max77759_chg_debug_reg_write, "%02llx\n");

static int dbg_init_fs(struct max77759_chgr_data *data)
{
	int ret;

	ret = device_create_file(data->dev, &dev_attr_fship_dtls);
	if (ret != 0)
		pr_err("Failed to create fship_dtls, ret=%d\n", ret);

	data->de = debugfs_create_dir("max77759_chg", 0);
	if (IS_ERR_OR_NULL(data->de))
		return -EINVAL;

	debugfs_create_atomic_t("insel_cnt", 0644, data->de, &data->insel_cnt);
	debugfs_create_bool("insel_clear", 0644, data->de, &data->insel_clear);

	debugfs_create_atomic_t("early_topoff_cnt", 0644, data->de,
				&data->early_topoff_cnt);

#if IS_ENABLED(CONFIG_GOOGLE_BCL)
	debugfs_create_file("vdroop2_ok", 0400, data->de, data,
			    &vdroop2_ok_fops);
	debugfs_create_file("vdp1_stp_bst", 0600, data->de, data,
			    &vdp1_stp_bst_fops);
	debugfs_create_file("vdp2_stp_bst", 0600, data->de, data,
			    &vdp2_stp_bst_fops);
	debugfs_create_file("sys_uvlo1", 0600, data->de, data,
			    &sys_uvlo1_fops);
	debugfs_create_file("sys_uvlo2", 0600, data->de, data,
			    &sys_uvlo2_fops);
	debugfs_create_file("bat_oilo", 0600, data->de, data,
			    &bat_oilo_fops);
#endif

	debugfs_create_file("input_mask_clear", 0600, data->de, data,
			    &input_mask_clear_fops);
	debugfs_create_file("chg_restart", 0600, data->de, data,
			    &charger_restart_fops);

	debugfs_create_u32("address", 0600, data->de, &data->debug_reg_address);
	debugfs_create_file("data", 0600, data->de, data, &debug_reg_rw_fops);
	return 0;
}

static bool max77759_chg_is_reg(struct device *dev, unsigned int reg)
{
	return (reg >= MAX77759_CHG_INT) && (reg <= MAX77759_CHG_CNFG_19);
}

static const struct regmap_config max77759_chg_regmap_cfg = {
	.name = "max77759_charger",
	.reg_bits = 8,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = MAX77759_CHG_CNFG_19,
	.readable_reg = max77759_chg_is_reg,
	.volatile_reg = max77759_chg_is_reg,

};

#if IS_ENABLED(CONFIG_GOOGLE_BCL)
static int max77759_irq_work(struct max77759_chgr_data *data, u8 idx)
{
	u8 chg_dtls1;
	int ret;
	u64 val;
	struct delayed_work *irq_wq = &data->triggered_irq_work[idx];

	mutex_lock(&data->triggered_irq_lock[idx]);
	ret = max77759_reg_read(data->regmap, MAX77759_CHG_DETAILS_01, &chg_dtls1);
	if (ret < 0) {
		mutex_unlock(&data->triggered_irq_lock[idx]);
		return -ENODEV;
	}
	val = _chg_details_01_vdroop2_ok_get(chg_dtls1);

	if (((val >> 7) & 0x1) == 0) {
		/* Still below threshold */
		mod_delayed_work(system_wq, irq_wq,
				   msecs_to_jiffies(VD_DELAY));
		mutex_unlock(&data->triggered_irq_lock[idx]);
	} else {
		data->triggered_counter[idx] = 0;
		mutex_unlock(&data->triggered_irq_lock[idx]);
		enable_irq(data->triggered_irq[idx]);
	}

	return 0;
}

static void max77759_vdroop1_work(struct work_struct *work)
{
	struct max77759_chgr_data *chg_data =
	    container_of(work, struct max77759_chgr_data, triggered_irq_work[VDROOP1].work);

	max77759_irq_work(chg_data, VDROOP1);
}

static void max77759_vdroop2_work(struct work_struct *work)
{
	struct max77759_chgr_data *chg_data =
	    container_of(work, struct max77759_chgr_data, triggered_irq_work[VDROOP2].work);

	max77759_irq_work(chg_data, VDROOP2);
}

static void max77759_batoilo_work(struct work_struct *work)
{
	struct max77759_chgr_data *chg_data =
	    container_of(work, struct max77759_chgr_data, triggered_irq_work[BATOILO].work);

	max77759_irq_work(chg_data, BATOILO);
}

static void max77759_triggered_irq_work(void *data, int id)
{
	struct max77759_chgr_data *chg_data = data;
	struct thermal_zone_device *tvid = chg_data->tz_miti[id];

	disable_irq_nosync(chg_data->triggered_irq[id]);
	mutex_lock(&chg_data->triggered_irq_lock[id]);
	if (chg_data->triggered_counter[id] == 0) {
		chg_data->triggered_counter[id] += 1;
		if (tvid)
			thermal_zone_device_update(tvid, THERMAL_EVENT_UNSPECIFIED);
	}
	mod_delayed_work(system_wq, &chg_data->triggered_irq_work[id],
			 msecs_to_jiffies(VD_DELAY));
	mutex_unlock(&chg_data->triggered_irq_lock[id]);
}

static int uvilo_read_stats(struct ocpsmpl_stats *dst, struct max77759_chgr_data *data)
{
	int ret;

	if (!dst || !data)
		return -EINVAL;

	ret = max77759_find_fg(data);
	if (ret == 0)
		ret = max1720x_get_capacity(data->fg_i2c_client, &dst->capacity);
	if (ret == 0)
		ret = max1720x_get_voltage_now(data->fg_i2c_client, &dst->voltage);
	if (ret < 0)
		return -EINVAL;

	dst->_time = ktime_to_ms(ktime_get());
	return (dst->capacity == -1 || dst->voltage == -1) ? -EIO : 0;
}
#endif

/*
 * int[0]
 *  CHG_INT_AICL_I	(0x1 << 7)
 *  CHG_INT_CHGIN_I	(0x1 << 6)
 *  CHG_INT_WCIN_I	(0x1 << 5)
 *  CHG_INT_CHG_I	(0x1 << 4)
 *  CHG_INT_BAT_I	(0x1 << 3)
 *  CHG_INT_INLIM_I	(0x1 << 2)
 *  CHG_INT_THM2_I	(0x1 << 1)
 *  CHG_INT_BYP_I	(0x1 << 0)
 *
 * int[1]
 *  CHG_INT2_INSEL_I		(0x1 << 7)
 *  CHG_INT2_SYS_UVLO1_I	(0x1 << 6)
 *  CHG_INT2_SYS_UVLO2_I	(0x1 << 5)
 *  CHG_INT2_BAT_OILO_I		(0x1 << 4)
 *  CHG_INT2_CHG_STA_CC_I	(0x1 << 3)
 *  CHG_INT2_CHG_STA_CV_I	(0x1 << 2)
 *  CHG_INT2_CHG_STA_TO_I	(0x1 << 1)
 *  CHG_INT2_CHG_STA_DONE_I	(0x1 << 0)
 *
 * these 3 cause un-necessary chatter at EOC due to the interaction between
 * the CV and the IIN loop:
 *   MAX77759_CHG_INT2_MASK_CHG_STA_CC_M |
 *   MAX77759_CHG_INT2_MASK_CHG_STA_CV_M |
 *   MAX77759_CHG_INT_MASK_CHG_M
 */
static u8 max77759_int_mask[MAX77759_CHG_INT_COUNT] = {
	~(MAX77759_CHG_INT_MASK_CHGIN_M |
	  MAX77759_CHG_INT_MASK_WCIN_M |
	  MAX77759_CHG_INT_MASK_BAT_M),
	(u8)~(MAX77759_CHG_INT2_MASK_INSEL_M |
	  MAX77759_CHG_INT2_MASK_SYS_UVLO1_M |
	  MAX77759_CHG_INT2_MASK_SYS_UVLO2_M |
	  MAX77759_CHG_INT2_MASK_CHG_STA_TO_M |
	  MAX77759_CHG_INT2_MASK_CHG_STA_DONE_M),
};

static irqreturn_t max77759_chgr_irq(int irq, void *client)
{
	struct max77759_chgr_data *data = client;
	u8 chg_int[MAX77759_CHG_INT_COUNT];
	bool broadcast;
	int ret;

	ret = max77759_readn(data->regmap, MAX77759_CHG_INT, chg_int,
			     sizeof(chg_int));
	if (ret < 0)
		return IRQ_NONE;

	if ((chg_int[0] & ~max77759_int_mask[0]) == 0 &&
	    (chg_int[1] & ~max77759_int_mask[1]) == 0)
		return IRQ_NONE;

	ret = max77759_writen(data->regmap, MAX77759_CHG_INT, chg_int,
			      sizeof(chg_int));
	if (ret < 0)
		return IRQ_NONE;

	pr_debug("INT : %02x %02x\n", chg_int[0], chg_int[1]);

	/* always broadcast battery events */
	broadcast = chg_int[0] & MAX77759_CHG_INT_MASK_BAT_M;

	if (chg_int[1] & MAX77759_CHG_INT2_MASK_INSEL_M) {

		if (data->insel_clear)
			ret = max77759_chgr_input_mask_clear(data);

		pr_debug("%s: INSEL insel_auto_clear=%d (%d)\n", __func__,
			 data->insel_clear, data->insel_clear ? ret : 0);
		atomic_inc(&data->insel_cnt);
	}

#if IS_ENABLED(CONFIG_GOOGLE_BCL)
	if (chg_int[1] & MAX77759_CHG_INT2_SYS_UVLO1_I) {
		pr_debug("%s: SYS_UVLO1\n", __func__);

		if (data->bcl_dev) {
			atomic_inc(&data->bcl_dev->if_triggered_cnt[VDROOP1]);
			uvilo_read_stats(&data->bcl_dev->if_triggered_stats[VDROOP1], data);
		}
		max77759_triggered_irq_work(data, VDROOP1);
	}

	if (chg_int[1] & MAX77759_CHG_INT2_SYS_UVLO2_I) {
		pr_debug("%s: SYS_UVLO2\n", __func__);

		if (data->bcl_dev) {
			atomic_inc(&data->bcl_dev->if_triggered_cnt[VDROOP2]);
			uvilo_read_stats(&data->bcl_dev->if_triggered_stats[VDROOP2], data);
		}
		max77759_triggered_irq_work(data, VDROOP2);
	}

	if (chg_int[1] & MAX77759_CHG_INT2_BAT_OILO_I) {
		pr_debug("%s: BAT_OILO\n", __func__);

		if (data->bcl_dev) {
			atomic_inc(&data->bcl_dev->if_triggered_cnt[BATOILO]);
			uvilo_read_stats(&data->bcl_dev->if_triggered_stats[BATOILO], data);
		}
		max77759_triggered_irq_work(data, BATOILO);
	}
#endif

	if (chg_int[1] & MAX77759_CHG_INT2_MASK_CHG_STA_TO_M) {
		pr_debug("%s: TOP_OFF\n", __func__);

		if (!max77759_is_full(data)) {
			/*
			 * on small adapter  might enter top-off far from the
			 * last charge tier due to system load.
			 * TODO: check inlim (maybe) and rewrite fv_uv
			 */
			atomic_inc(&data->early_topoff_cnt);
		}

	}

	if (chg_int[1] & MAX77759_CHG_INT2_MASK_CHG_STA_CC_M)
		pr_debug("%s: CC_MODE\n", __func__);

	if (chg_int[1] & MAX77759_CHG_INT2_MASK_CHG_STA_CV_M)
		pr_debug("%s: CV_MODE\n", __func__);

	if (chg_int[1] & MAX77759_CHG_INT2_MASK_CHG_STA_DONE_M) {
		const bool charge_done = data->charge_done;

		/* reset on disconnect or toggles of enable/disable */
		if (max77759_is_full(data))
			data->charge_done = true;
		broadcast = true;

		pr_debug("%s: CHARGE DONE charge_done=%d->%d\n", __func__,
			 charge_done, data->charge_done);
	}

	/* wired input is changed */
	if (chg_int[0] & MAX77759_CHG_INT_MASK_CHGIN_M) {
		pr_debug("%s: CHGIN charge_done=%d\n", __func__, data->charge_done);

		data->charge_done = false;
		broadcast = true;

		if (data->chgin_psy)
			power_supply_changed(data->chgin_psy);
	}

	/* wireless input is changed */
	if (chg_int[0] & MAX77759_CHG_INT_MASK_WCIN_M) {
		pr_debug("%s: WCIN charge_done=%d\n", __func__, data->charge_done);

		data->charge_done = false;
		broadcast = true;

		if (data->wcin_psy)
			power_supply_changed(data->wcin_psy);
	}

	/* someting is changed */
	if (data->psy && broadcast)
		power_supply_changed(data->psy);

	return IRQ_HANDLED;
}

static int max77759_setup_votables(struct max77759_chgr_data *data)
{
	int ret;

	/* votes might change mode */
	data->mode_votable = gvotable_create_int_election(NULL, NULL,
					max77759_mode_callback,
					data);
	if (IS_ERR_OR_NULL(data->mode_votable)) {
		ret = PTR_ERR(data->mode_votable);
		dev_err(data->dev, "no mode votable (%d)\n", ret);
		return ret;
	}

	gvotable_set_vote2str(data->mode_votable, gvotable_v2s_uint);
	/* will use gvotable_get_default() when available */
	gvotable_set_default(data->mode_votable, (void *)GBMS_CHGR_MODE_STBY_ON);
	gvotable_election_set_name(data->mode_votable, GBMS_MODE_VOTABLE);

	/* Wireless charging, DC name is for compat */
	data->dc_suspend_votable =
		gvotable_create_bool_election(NULL,
					     max77759_dc_suspend_vote_callback,
					     data);
	if (IS_ERR_OR_NULL(data->dc_suspend_votable)) {
		ret = PTR_ERR(data->dc_suspend_votable);
		dev_err(data->dev, "no dc_suspend votable (%d)\n", ret);
		return ret;
	}

	gvotable_set_vote2str(data->dc_suspend_votable, gvotable_v2s_int);
	gvotable_election_set_name(data->dc_suspend_votable, "DC_SUSPEND");

	data->dc_icl_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_min,
					     max77759_dcicl_callback,
					     data);
	if (IS_ERR_OR_NULL(data->dc_icl_votable)) {
		ret = PTR_ERR(data->dc_icl_votable);
		dev_err(data->dev, "no dc_icl votable (%d)\n", ret);
		return ret;
	}

	gvotable_set_vote2str(data->dc_icl_votable, gvotable_v2s_uint);
	gvotable_set_default(data->dc_icl_votable, (void *)700000);
	gvotable_election_set_name(data->dc_icl_votable, "DC_ICL");
	gvotable_use_default(data->dc_icl_votable, true);

	return 0;
}

#if IS_ENABLED(CONFIG_GOOGLE_BCL)
static int max77759_miti_read_level(void *data, int *val, int id)
{
	struct max77759_chgr_data *chg_data = data;
	int triggered_counter = chg_data->triggered_counter[id];
	unsigned int triggered_lvl = chg_data->triggered_lvl[id];

	if ((triggered_counter != 0) && (triggered_counter < THERMAL_IRQ_COUNTER_LIMIT)) {
		*val = triggered_lvl + THERMAL_HYST_LEVEL;
		triggered_counter += 1;
	} else {
		*val = triggered_lvl;
		triggered_counter = 0;
	}
	chg_data->triggered_counter[id] = triggered_counter;

	return 0;
}

static int max77759_vdroop1_read_temp(void *data, int *val)
{
	return max77759_miti_read_level(data, val, VDROOP1);
}

static int max77759_vdroop2_read_temp(void *data, int *val)
{
	return max77759_miti_read_level(data, val, VDROOP2);
}

static int max77759_batoilo_read_temp(void *data, int *val)
{
	return max77759_miti_read_level(data, val, BATOILO);
}

static const struct thermal_zone_of_device_ops vdroop1_tz_ops = {
	.get_temp = max77759_vdroop1_read_temp,
};

static const struct thermal_zone_of_device_ops vdroop2_tz_ops = {
	.get_temp = max77759_vdroop2_read_temp,
};

static const struct thermal_zone_of_device_ops batoilo_tz_ops = {
	.get_temp = max77759_batoilo_read_temp,
};

static int max77759_init_vdroop(void *data_)
{
	struct max77759_chgr_data *data = data_;
	int ret = 0;
	u8 regdata;

	INIT_DELAYED_WORK(&data->triggered_irq_work[VDROOP1], max77759_vdroop1_work);
	INIT_DELAYED_WORK(&data->triggered_irq_work[VDROOP2], max77759_vdroop2_work);
	INIT_DELAYED_WORK(&data->triggered_irq_work[BATOILO], max77759_batoilo_work);
	data->triggered_counter[VDROOP1] = 0;
	data->triggered_counter[VDROOP2] = 0;
	data->triggered_counter[BATOILO] = 0;
	mutex_init(&data->triggered_irq_lock[VDROOP1]);
	mutex_init(&data->triggered_irq_lock[VDROOP2]);
	mutex_init(&data->triggered_irq_lock[BATOILO]);

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_17, &regdata);
	if (ret < 0)
		return -ENODEV;

	ret = max77759_reg_write(data->regmap, MAX77759_CHG_CNFG_17,
				 (u8) regdata | BATOILO_DET_30US);
	if (ret < 0)
		return -EIO;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_15, &regdata);
	if (ret < 0)
		return -ENODEV;

	data->triggered_lvl[VDROOP1] = VD_BATTERY_VOLTAGE
			- (VD_STEP * ((regdata & MAX77759_CHG_CNFG_15_SYS_UVLO1_MASK)
			>> MAX77759_CHG_CNFG_15_SYS_UVLO1_SHIFT)
			+ VD_LOWER_LIMIT) - THERMAL_HYST_LEVEL;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_16, &regdata);
	if (ret < 0)
		return -ENODEV;

	data->triggered_lvl[VDROOP2] = VD_BATTERY_VOLTAGE
			- (VD_STEP * ((regdata & MAX77759_CHG_CNFG_16_SYS_UVLO2_MASK)
			>> MAX77759_CHG_CNFG_16_SYS_UVLO2_SHIFT)
			+ VD_LOWER_LIMIT) - THERMAL_HYST_LEVEL;

	ret = max77759_reg_read(data->regmap, MAX77759_CHG_CNFG_14, &regdata);
	if (ret < 0)
		return -ENODEV;

	data->triggered_lvl[BATOILO] = (BO_STEP * ((regdata & MAX77759_CHG_CNFG_14_BAT_OILO_MASK)
			>> MAX77759_CHG_CNFG_14_BAT_OILO_SHIFT) + BO_LOWER_LIMIT)
			- THERMAL_HYST_LEVEL;


	data->tz_miti[VDROOP1] = thermal_zone_of_sensor_register(data->dev, VDROOP1, data,
								 &vdroop1_tz_ops);
	if (IS_ERR(data->tz_miti[VDROOP1])) {
		dev_err(data->dev, "TZ register vdroop%d failed, err:%ld\n", VDROOP1,
			PTR_ERR(data->tz_miti[VDROOP1]));
	} else {
		thermal_zone_device_enable(data->tz_miti[VDROOP1]);
		thermal_zone_device_update(data->tz_miti[VDROOP1], THERMAL_DEVICE_UP);
	}

	data->tz_miti[VDROOP2] = thermal_zone_of_sensor_register(data->dev, VDROOP2, data,
								 &vdroop2_tz_ops);
	if (IS_ERR(data->tz_miti[VDROOP2])) {
		dev_err(data->dev, "TZ register vdroop%d failed, err:%ld\n", VDROOP2,
			PTR_ERR(data->tz_miti[VDROOP2]));
	} else {
		thermal_zone_device_enable(data->tz_miti[VDROOP2]);
		thermal_zone_device_update(data->tz_miti[VDROOP2], THERMAL_DEVICE_UP);
	}

	data->tz_miti[BATOILO] = thermal_zone_of_sensor_register(data->dev, BATOILO, data,
								 &batoilo_tz_ops);
	if (IS_ERR(data->tz_miti[BATOILO])) {
		dev_err(data->dev, "TZ register BATOILO failed, err:%ld\n",
			PTR_ERR(data->tz_miti[BATOILO]));
	} else {
		thermal_zone_device_enable(data->tz_miti[BATOILO]);
		thermal_zone_device_update(data->tz_miti[BATOILO], THERMAL_DEVICE_UP);
	}

	return 0;
}
#else
static int max77759_init_vdroop(void *data_)
{
	return 0;
}
#endif


static int max77759_charger_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct power_supply_config chgr_psy_cfg = { 0 };
	struct device *dev = &client->dev;
	struct max77759_chgr_data *data;
	struct regmap *regmap;
	const char *tmp;
	int ret = 0;
	u8 ping;

	regmap = devm_regmap_init_i2c(client, &max77759_chg_regmap_cfg);
	if (IS_ERR(regmap)) {
		dev_err(dev, "Failed to initialize regmap\n");
		return -EINVAL;
	}

	ret = max77759_reg_read(regmap, MAX77759_CHG_CNFG_00, &ping);
	if (ret < 0)
		return -ENODEV;

	/* TODO: PING or read HW version from PMIC */

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	data->regmap = regmap;
	data->fship_dtls = -1;
	data->wden = false; /* TODO: read from DT */
	mutex_init(&data->io_lock);
	atomic_set(&data->insel_cnt, 0);
	atomic_set(&data->early_topoff_cnt, 0);
	i2c_set_clientdata(client, data);

	data->usecase_wake_lock = wakeup_source_register(NULL, "max77759-usecase");
	if (!data->usecase_wake_lock) {
		pr_err("Failed to register wakeup source\n");
		return -ENODEV;
	}

	/* NOTE: only one instance */
	ret = of_property_read_string(dev->of_node, "max77759,psy-name", &tmp);
	if (ret == 0)
		max77759_psy_desc.name = devm_kstrdup(dev, tmp, GFP_KERNEL);

	chgr_psy_cfg.drv_data = data;
	chgr_psy_cfg.supplied_to = NULL;
	chgr_psy_cfg.num_supplicants = 0;
	data->psy = devm_power_supply_register(dev, &max77759_psy_desc,
		&chgr_psy_cfg);
	if (IS_ERR(data->psy)) {
		dev_err(dev, "Failed to register psy rc = %ld\n",
			PTR_ERR(data->psy));
		return -EINVAL;
	}

	/* CHARGER_MODE needs this (initialized to -EPROBE_DEFER) */
	max77759_setup_usecases(&data->uc_data, NULL);
	data->uc_data.client = client;

	/* other drivers (ex tcpci) need this. */
	ret = max77759_setup_votables(data);
	if (ret < 0)
		return ret;

	if (max77759_init_vdroop(data) < 0)
		dev_err(dev, "vdroop initialization failed %d.\n", ret);

	data->irq_gpio = of_get_named_gpio(dev->of_node, "max77759,irq-gpio", 0);
	if (data->irq_gpio < 0) {
		dev_err(dev, "failed get irq_gpio\n");
	} else {
		client->irq = gpio_to_irq(data->irq_gpio);

		ret = devm_request_threaded_irq(data->dev, client->irq, NULL,
						max77759_chgr_irq,
						IRQF_TRIGGER_LOW |
						IRQF_SHARED |
						IRQF_ONESHOT,
						"max77759_charger",
						data);
		if (ret == 0) {
			enable_irq_wake(client->irq);

			/* might cause the isr to be called */
			max77759_chgr_irq(-1, data);
			ret = max77759_writen(regmap, MAX77759_CHG_INT_MASK,
					      max77759_int_mask,
					      sizeof(max77759_int_mask));
			if (ret < 0)
				dev_err(dev, "cannot set irq_mask (%d)\n", ret);
		}
	}

	ret = dbg_init_fs(data);
	if (ret < 0)
		dev_err(dev, "Failed to initialize debug fs\n");

	mutex_lock(&data->io_lock);
	ret = max77759_wdt_enable(data, data->wden);
	if (ret < 0)
		dev_err(dev, "wd enable=%d failed %d\n", data->wden, ret);

	/* disable fast charge safety timer */
	max77759_chg_reg_update(data->uc_data.client, MAX77759_CHG_CNFG_01,
				MAX77759_CHG_CNFG_01_FCHGTIME_MASK,
				MAX77759_CHG_CNFG_01_FCHGTIME_CLEAR);

	/* b/193355117 disable THM2 monitoring */
	if (!of_property_read_bool(dev->of_node, "max77759,usb-mon")) {
		max77759_chg_reg_update(data->uc_data.client, MAX77759_CHG_CNFG_13,
					MAX77759_CHG_CNFG_13_THM2_HW_CTRL |
					MAX77759_CHG_CNFG_13_USB_TEMP_MASK,
					0);
	}

	mutex_unlock(&data->io_lock);

	ret = of_property_read_u32(dev->of_node, "max77759,chg-term-voltage",
				   &data->chg_term_voltage);
	if (ret < 0)
		data->chg_term_voltage = 0;

	ret = of_property_read_u32(dev->of_node, "max77759,chg-term-volt-debounce",
				   &data->chg_term_volt_debounce);
	if (ret < 0)
		data->chg_term_volt_debounce = CHG_TERM_VOLT_DEBOUNCE;
	if (data->chg_term_voltage == 0)
		data->chg_term_volt_debounce = 0;

	data->init_complete = 1;
	data->resume_complete = 1;

	ret = max77759_init_wcin_psy(data);
	if (ret < 0)
		pr_err("Couldn't register dc power supply (%d)\n", ret);

#if IS_ENABLED(CONFIG_GOOGLE_BCL)
	data->bcl_dev = google_retrieve_bcl_handle();
#endif

	dev_info(dev, "registered as %s\n", max77759_psy_desc.name);
	return 0;
}

static int max77759_charger_remove(struct i2c_client *client)
{
#if IS_ENABLED(CONFIG_GOOGLE_BCL)
	struct max77759_chgr_data *data = i2c_get_clientdata(client);

	if (data->tz_miti[VDROOP1])
		thermal_zone_of_sensor_unregister(data->dev, data->tz_miti[VDROOP2]);
	if (data->tz_miti[VDROOP2])
		thermal_zone_of_sensor_unregister(data->dev, data->tz_miti[VDROOP2]);
	if (data->tz_miti[BATOILO])
		thermal_zone_of_sensor_unregister(data->dev, data->tz_miti[BATOILO]);
#endif

	wakeup_source_unregister(data->usecase_wake_lock);

	return 0;
}


static const struct of_device_id max77759_charger_of_match_table[] = {
	{ .compatible = "maxim,max77759chrg"},
	{},
};
MODULE_DEVICE_TABLE(of, max77759_charger_of_match_table);

static const struct i2c_device_id max77759_id[] = {
	{"max77759_charger", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max77759_id);

#if defined CONFIG_PM
static int max77759_charger_pm_suspend(struct device *dev)
{
	/* TODO: is there anything to do here? */
	return 0;
}

static int max77759_charger_pm_resume(struct device *dev)
{
	/* TODO: is there anything to do here? */
	return 0;
}
#endif

static const struct dev_pm_ops max77759_charger_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(
		max77759_charger_pm_suspend,
		max77759_charger_pm_resume)
};

static struct i2c_driver max77759_charger_i2c_driver = {
	.driver = {
		.name = "max77759-charger",
		.owner = THIS_MODULE,
		.of_match_table = max77759_charger_of_match_table,
#ifdef CONFIG_PM
		.pm = &max77759_charger_pm_ops,
#endif
	},
	.id_table = max77759_id,
	.probe    = max77759_charger_probe,
	.remove   = max77759_charger_remove,
};

module_i2c_driver(max77759_charger_i2c_driver);

MODULE_DESCRIPTION("Maxim 77759 Charger Driver");
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_LICENSE("GPL");
