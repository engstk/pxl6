/* SPDX-License-Identifier: GPL-2.0-only */
//
// UFS Host Controller driver for Exynos specific extensions
//
// Copyright (C) 2013-2014 Samsung Electronics Co., Ltd.
//
// Authors:
//	Kiwoong <kwmad.kim@samsung.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
#ifndef _GS101_UFS_CAL_IF_H
#define _GS101_UFS_CAL_IF_H

/* If not matched with ufs-cal-if, compling would fail */
#define UFS_CAL_IF_VER 4
#define UFS_CAL_IF_COMPAT_MMIO_VER 2

#undef BIT
#define BIT(a)	(1UL << (a))

struct uic_pwr_mode {
	u8 lane;
	u8 gear;
	u8 mode;
	u8 hs_series;
};

enum {
	HOST_EMBD = 0,
	HOST_CARD = 1,
};

enum {
	EOM_RTY_G1 = 8,
	EOM_RTY_G2 = 4,
	EOM_RTY_G3 = 2,
	EOM_RTY_G4 = 1,
	EOM_RTY_MAX = 8,
};

enum {
	GEAR_1 = 1,
	GEAR_2,
	GEAR_3,
	GEAR_4,
	GEAR_MAX = GEAR_4,
};

#define MAX_LANE		4

#define EOM_PH_SEL_MAX		72
#define EOM_DEF_VREF_MAX	256
#define EOM_MAX_SIZE		(EOM_RTY_MAX * EOM_PH_SEL_MAX * \
					EOM_DEF_VREF_MAX)

static const u32 ufs_s_eom_repeat[GEAR_MAX + 1] = {
		0, EOM_RTY_G1, EOM_RTY_G2, EOM_RTY_G3, EOM_RTY_G4
};

struct ufs_eom_result_s {
	u32 v_phase;
	u32 v_vref;
	u32 v_err;
};

/* interface */
struct ufs_cal_param {
	/* input */
	struct ufs_vs_handle *handle;
	u8 available_lane;
	u8 connected_tx_lane;
	u8 connected_rx_lane;
	u8 active_tx_lane;
	u8 active_rx_lane;
	u32 mclk_rate;
	u8 tbl;
	u8 board;
	u8 evt_ver;
	u8 max_gear;
	struct uic_pwr_mode *pmd;

	/* output */
	u32 eom_sz;
	struct ufs_eom_result_s *eom[MAX_LANE];	/* per lane */

	/* private data */
	u32 mclk_period;
	u32 mclk_period_rnd_off;
	u32 mclk_period_unipro_18;
};

enum ufs_cal_errno {
	UFS_CAL_NO_ERROR = 0,
	UFS_CAL_TIMEOUT,
	UFS_CAL_ERROR,
	UFS_CAL_INV_ARG,
	UFS_CAL_INV_CONF,
};

enum {
	__BRD_SMDK,
	__BRD_ASB,
	__BRD_HSIE,
	__BRD_ZEBU,
	__BRD_UNIV,
	__BRD_MAX,
};

#define BRD_SMDK	BIT(__BRD_SMDK)
#define BRD_ASB		BIT(__BRD_ASB)
#define BRD_HSIE	BIT(__BRD_HSIE)
#define BRD_ZEBU	BIT(__BRD_ZEBU)
#define BRD_UNIV	BIT(__BRD_UNIV)
#define BRD_MAX		BIT(__BRD_MAX)
#define BRD_ALL		(BIT(__BRD_MAX) - 1)

/* UFS CAL interface */
typedef enum ufs_cal_errno (*cal_if_func_init) (struct ufs_cal_param *, int);
typedef enum ufs_cal_errno (*cal_if_func) (struct ufs_cal_param *);
enum ufs_cal_errno ufs_cal_post_h8_enter(struct ufs_cal_param *p);
enum ufs_cal_errno ufs_cal_pre_h8_exit(struct ufs_cal_param *p);
enum ufs_cal_errno ufs_cal_post_pmc(struct ufs_cal_param *p);
enum ufs_cal_errno ufs_cal_pre_pmc(struct ufs_cal_param *p);
enum ufs_cal_errno ufs_cal_post_link(struct ufs_cal_param *p);
enum ufs_cal_errno ufs_cal_pre_link(struct ufs_cal_param *p);
enum ufs_cal_errno ufs_cal_init(struct ufs_cal_param *p, int idx);
enum ufs_cal_errno ufs_cal_eom(struct ufs_cal_param *p);

enum ufs_cal_errno ufs_cal_loopback_init(struct ufs_cal_param *p);
enum ufs_cal_errno ufs_cal_loopback_set_1(struct ufs_cal_param *p);
enum ufs_cal_errno ufs_cal_loopback_set_2(struct ufs_cal_param *p);
#endif /*_UFS_CAL_ */
