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

#ifndef MAX77759_H_
#define MAX77759_H_

#include <linux/i2c.h>
#include <linux/interrupt.h>

#define SCNPRINTF(...) scnprintf(__VA_ARGS__)
#include "max77759_regs.h"

#define MAX77759_CHG_INT_COUNT 2

#define MAX77759_PMIC_REV_A0		0x01
#define MAX77759_PMIC_REV_A1		0x02

#define MAX77759_PMIC_PMIC_ID_MW	0x3b

int max777x9_pmic_get_id(struct i2c_client *client, u8 *id, u8 *rev);
int max777x9_pmic_reg_read(struct i2c_client *client,
			   u8 addr, u8 *val, int len);
int max777x9_pmic_reg_write(struct i2c_client *client,
			    u8 addr, const u8 *val, int len);
int max777x9_pmic_reg_update(struct i2c_client *client,
			     u8 reg, u8 mask, u8 value);

/* write to a register */
int max77759_chg_reg_write(struct i2c_client *client, u8 reg, u8 value);
/* write to a register */
int max77759_chg_reg_read(struct i2c_client *client, u8 reg, u8 *value);
/* udate a register */
int max77759_chg_reg_update(struct i2c_client *client, u8 reg, u8 mask, u8 value);
/* change the mode register */
int max77759_chg_mode_write(struct i2c_client *client, enum max77759_charger_modes mode);
/* change the insel register */
int max77759_chg_insel_write(struct i2c_client *client, u8 mask, u8 value);
/* read the insel register */
int max77759_chg_insel_read(struct i2c_client *client, u8 *value);

#if IS_ENABLED(CONFIG_PMIC_MAX77729)
extern int max77759_read_batt_conn(struct i2c_client *client, int *temp);
extern int max77759_read_usb_temp(struct i2c_client *client, int *temp);
extern int max77759_read_batt_id(struct i2c_client *client, unsigned int *id);
#else
static inline int max77759_read_batt_conn(struct i2c_client *client, int *temp)
{
	return -ENODEV;
}
static inline int max77759_read_usb_temp(struct i2c_client *client, int *temp)
{
	return -ENODEV;
}
static inline int max77759_read_batt_id(struct i2c_client *client,
					unsigned int *id)
{
	return -ENODEV;
}
#endif

/* ----------------------------------------------------------------------------
 * GS101 usecases
 * Platform specific, will need to be moved outside the driver.
 *
 * Case	USB_chg USB_otg	WLC_chg	WLC_TX	PMIC_Charger	Ext_B	LSx	Name
 * ----------------------------------------------------------------------------
 * 1-1	1	0	x	0	IF-PMIC-VBUS	0	0/0	USB_CHG
 * 1-2	2	0	x	0	DC VBUS		0	0/0	USB_DC
 * 2-1	1	0	0	1	IF-PMIC-VBUS	2	0/1	USB_CHG_WLC_TX
 * 2-2	2	0	0	1	DC CHG		2	0/1	USB_DC_WLC_TX
 * 3-1	0	0	1	0	IF-PMIC-WCIN	0	0/0	WLC_RX
 * 3-2	0	0	2	0	DC WCIN		0	0/0	WLC_DC
 * 4-1	0	1	1	0	IF-PMIC-WCIN	1	1/0	USB_OTG_WLC_RX
 * 4-2	0	1	2	0	DC WCIN		1	1/0	USB_OTG_WLC_DC
 * 5-1	0	1	0	0	0		1	1/0	USB_OTG
 * 5-2	0	1	0	0	OTG 5V		0	0/0	USB_OTG_FRS
 * 6-2	0	0	0	1	0		2	0/1	WLC_TX
 * 7-2	0	1	0	1	MW OTG 5V	2	0/1	USB_OTG_WLC_TX
 * 8	0	0	0	0	0		0	0/0	IDLE
 * ----------------------------------------------------------------------------
 *
 * Ext_Boost = 0 off, 1 = OTG 5V, 2 = WTX 7.5
 * USB_chg = 0 off, 1 = on, 2 = PPS
 * WLC_chg = 0 off, 1 = on, 2 = PPS
 */
struct max77759_foreach_cb_data {
	struct gvotable_election *el;

	const char *reason;

	int chgr_on;	/* CC_MAX != 0 */
	bool stby_on;	/* on disconnect, mode=0 */
	bool charge_done;

	int chgin_off;	/* input_suspend, mode=0 */
	int wlcin_off;	/* input_suspend, mode=0 */
	int usb_wlc;	/* input_suspend, mode=0 */

	/* wlc_on is the same as wlc_rx */
	bool buck_on;	/* wired power in (chgin_on) from TCPCI */

	bool otg_on;	/* power out, usually external */
	bool frs_on;	/* power out, fast role swap (internal) */

	bool wlc_rx;	/* charging wireless */
	bool wlc_tx;	/* battery share */

	bool dc_on;	/* DC requested - wired or wireless */

	bool boost_on;	/* Compat: old for WLC program */
	bool uno_on;	/* Compat: old for WLC program */

	u8 raw_value;	/* hard override */
	bool use_raw;

	u8 reg;
};

struct max77759_usecase_data {
	int is_a1;

	int bst_on;		/* ext boost */
	int bst_sel;		/* 5V or 7.5V */
	int ext_bst_ctl;	/* MW VENDOR_EXTBST_CTRL */
	int otg_enable;		/* enter/exit from OTG cases */

	int ls2_en;		/* OVP LS2, rtx case */
	int sw_en;		/* OVP SW Enable, rtx+otg case */

	int vin_is_valid;	/* MAX20339 STATUS1.vinvalid */
	int lsw1_is_open;	/* MAX20339 STATUS2.lsw1open */
	int lsw1_is_closed;	/* MAX20339 STATUS2.lsw1closed */
	int ls1_en;		/* MAX20339 close LSW1 directly */

	int wlc_en;		/* wlcrx/chgin coex */
	int ext_bst_mode;	/* wlcrx+otg: b/175706836, TPS61372 P1.1+ */
	int cpout_en;		/* wlcrx+otg: CPOUT enabled/disabled */
	int cpout_ctl;		/* wlcrx+otg: CPOUT level 5.3V or DFLT */

	int cpout21_en;		/* wlctx: CPOUT 2:1 converter enable/disable */

	u8 otg_ilim;		/* TODO: TCPM to control this? */
	u8 otg_vbyp;		/* TODO: TCPM to control this? */
	u8 otg_orig;		/* restore value */

	struct i2c_client *client;
	bool init_done;
	int use_case;
};

enum gsu_usecases {
	GSU_RAW_MODE 		= -1,	/* raw mode, default, */

	GSU_MODE_STANDBY	= 0,	/* 8, PMIC mode 0 */
	GSU_MODE_USB_CHG	= 1,	/* 1-1 wired mode 0x4, mode 0x5 */
	GSU_MODE_USB_DC 	= 2,	/* 1-2 wired mode 0x0 */
	GSU_MODE_USB_CHG_WLC_TX = 3,	/* 2-1, 1041, */
	GSU_MODE_USB_DC_WLC_TX	= 4,	/* 2-2 1042, */

	GSU_MODE_WLC_RX		= 5,	/* 3-1, mode 0x4, mode 0x5 */
	GSU_MODE_WLC_DC		= 6,	/* 3-2, mode 0x0 */

	GSU_MODE_USB_OTG_WLC_RX = 7,	/* 4-1, 524, */
	GSU_MODE_USB_OTG_WLC_DC = 8,	/* 4-2, 532, */
	GSU_MODE_USB_OTG 	= 9,	/* 5-1, 516,*/
	GSU_MODE_USB_OTG_FRS	= 10,	/* 5-2, PMIC mode 0x0a */

	GSU_MODE_WLC_TX 	= 11,	/* 6-2, 1056, */
	GSU_MODE_USB_OTG_WLC_TX	= 12,	/* 7-2, 1060, */

	GSU_MODE_USB_WLC_RX	= 13,
};

/* internal system values */
enum {
	/* Charging disabled (go to mode 0) */
	GBMS_CHGR_MODE_STBY_ON		= 0x10 + MAX77759_CHGR_MODE_ALL_OFF,
	/* USB inflow off */
	GBMS_CHGR_MODE_CHGIN_OFF	= 0x11 + MAX77759_CHGR_MODE_ALL_OFF,
	/* WCIN inflow off */
	GBMS_CHGR_MODE_WLCIN_OFF	= 0x12 + MAX77759_CHGR_MODE_ALL_OFF,
	/* USB + WLC_RX mode */
	GBMS_CHGR_MODE_USB_WLC_RX	= 0x13 + MAX77759_CHGR_MODE_ALL_OFF,

	/* charging enabled (charging current != 0) */
	GBMS_CHGR_MODE_CHGR_BUCK_ON	= 0x10 + MAX77759_CHGR_MODE_CHGR_BUCK_ON,
	/* Compat: old for programmging */
	GBMS_CHGR_MODE_BOOST_UNO_ON	= 0x10 + MAX77759_CHGR_MODE_BOOST_UNO_ON,
};


#endif
