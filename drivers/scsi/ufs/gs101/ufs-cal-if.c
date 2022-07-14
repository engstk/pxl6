// SPDX-License-Identifier: GPL-2.0-only
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
#if defined(__UFS_CAL_LK__)		/* LK */
#include <types.h>
#include <reg.h>
#include <ufs-vs-mmio.h>
#include <ufs-cal-if.h>

#elif defined(__UFS_CAL_FW__)		/* FW */
#include "../ufs_util.h"
#include "ufs-vs-mmio.h"
#include "ufs-cal-if.h"
#else					/* Kernel */
#include <linux/io.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include "ufs-vs-mmio.h"
#include "ufs-cal-if.h"

#endif

struct ufs_cal_phy_cfg {
	u32 mib;
	u32 addr;
	u32 val;
	u32 flg;
	u32 lyr;
	u8 board;
};

enum {
	PHY_CFG_NONE = 0,
	PHY_PCS_COMN,
	PHY_PCS_RXTX,
	PHY_PMA_COMN,
	PHY_PMA_TRSV,
	PHY_PLL_WAIT,
	PHY_CDR_WAIT,
	PHY_CDR_AFC_WAIT,
	UNIPRO_STD_MIB,
	UNIPRO_DBG_MIB,
	UNIPRO_DBG_APB,

	PHY_PCS_RX,
	PHY_PCS_TX,
	PHY_PCS_RX_PRD,
	PHY_PCS_TX_PRD,
	UNIPRO_DBG_PRD,
	PHY_PMA_TRSV_LANE1_SQ_OFF,
	PHY_PMA_TRSV_SQ,
	COMMON_WAIT,

	PHY_PCS_RX_LR_PRD,
	PHY_PCS_TX_LR_PRD,
	PHY_PCS_RX_PRD_ROUND_OFF,
	PHY_PCS_TX_PRD_ROUND_OFF,
	UNIPRO_ADAPT_LENGTH,
	PHY_EMB_CDR_WAIT,
	PHY_EMB_CAL_WAIT,
};

enum {
	PMD_PWM_G1 = 0,
	PMD_PWM_G2,
	PMD_PWM_G3,
	PMD_PWM_G4,
	PMD_PWM_G5,
	PMD_PWM,

	PMD_HS_G1,
	PMD_HS_G2,
	PMD_HS_G3,
	PMD_HS_G4,
	PMD_HS,

	PMD_ALL,
};

/*
 * CAL table, project specifics
 *
 * This is supposed to be in here, i.e.
 * right before definitions for version check.
 *
 * DO NOT MOVE THIS TO ANYWHERE RANDOMLY !!!
 */
#include "ufs-cal.h"

/* Version check */
#if (UFS_CAL_IF_COMPAT_MMIO_VER != UFS_VS_MMIO_VER)
#error "UFS_CAL_IF_COMPAT_MMIO_VER and UFS_VS_MMIO_VER aren't matched"
#endif
#if (UFS_CAL_IF_VER != UFS_CAL_TABLE_COMPAT_IF_VER)
#error "UFS_CAL_IF_VER and UFS_CAL_TABLE_COMPAT_IF_VER aren't matched"
#endif

/*
 * UFS CAL just requires some things that don't have an impact on
 * external components, such as macros, not functions.
 * The requirement list is below: readl, writel, udelay
 */

#define NUM_OF_UFS_HOST	2

enum {
	PA_HS_MODE_A	= 1,
	PA_HS_MODE_B	= 2,
};

enum {
	FAST_MODE	= 1,
	SLOW_MODE	= 2,
	FASTAUTO_MODE	= 4,
	SLOWAUTO_MODE	= 5,
	UNCHANGED	= 7,
};

enum {
	TX_LANE_0 = 0,
	RX_LANE_0 = 4,
};

#define IS_PWR_MODE_HS(m)						\
		(((m) == FAST_MODE) || ((m) == FASTAUTO_MODE))
#define IS_PWR_MODE_PWM(m)						\
		(((m) == SLOW_MODE) || ((m) == SLOWAUTO_MODE))

#define TX_LINE_RESET_TIME		3200
#define RX_LINE_RESET_DETECT_TIME	1000

#define DELAY_PERIOD_IN_US		40		/* 1us */
#define TIMEOUT_IN_US			(40 * 1000)	/* 40ms */

#define UNIP_DL_ERROR_IRQ_MASK		0x4844	/* shadow of DL error */
#define PA_ERROR_IND_RECEIVED		BIT(15)

#define PHY_PMA_COMN_ADDR(reg)			(reg)
#define PHY_PMA_TRSV_ADDR(reg, lane)		((reg) + (0x800 * (lane)))

#define UNIP_COMP_AXI_AUX_FIELD			0x040
#define __WSTRB					(0xF << 24)
#define __SEL_IDX(L)				((L) & 0xFFFF)

/*
 * private data
 */
static struct ufs_cal_param *ufs_cal[NUM_OF_UFS_HOST];

/*
 * inline functions
 */
static inline u32 __get_mclk_period(struct ufs_cal_param *p)
{
	return (1000000000U / p->mclk_rate);
}

static inline u32 __get_mclk_period_unipro_18(struct ufs_cal_param *p)
{
	return (16 * 1000 * 1000000UL / p->mclk_rate);
}

static inline u32 __get_round_off(u32 value, u32 divider)
{
	return value / divider + ((value % divider) > (divider >> 1));
}

static inline u32 __get_mclk_period_rnd_off(struct ufs_cal_param *p)
{
#if defined(__UFS_CAL_FW__)
	return (u32)(((float)1000000000L / (float)p->mclk_rate) + 0.5);
#else
	/* assume that mclk_rate is supposed to be unsigned */
	return __get_round_off(1000000000U, p->mclk_rate);
#endif
}

/*
 * This function returns how many ticks is required to line reset
 * for predefined time value.
 */
static inline u32 __get_line_reset_ticks(struct ufs_cal_param *p,
					u32 time_in_us)
{
#if defined(__UFS_CAL_FW__)
	return (u32)((float)time_in_us *
			((float)p->mclk_rate / 1000000));
#else
	u64 val = (u64)p->mclk_rate;

	val *= time_in_us;
	return (u32)(val / 1000000U);
#endif
}

static inline enum ufs_cal_errno __match_board_by_cfg(u8 board, u8 cfg_board)
{
	enum ufs_cal_errno match = UFS_CAL_ERROR;

	if (board & cfg_board)
		match = UFS_CAL_NO_ERROR;

	return match;
}

static enum ufs_cal_errno __match_mode_by_cfg(struct uic_pwr_mode *pmd,
					     int mode)
{
	enum ufs_cal_errno match;
	u8 _m, _g;

	_m = pmd->mode;
	_g = pmd->gear;

	if (mode == PMD_ALL) {
		match = UFS_CAL_NO_ERROR;
	} else if (IS_PWR_MODE_HS(_m) && mode >= PMD_HS_G1 &&
		   mode <= PMD_HS) {
		match = UFS_CAL_NO_ERROR;
		if (mode != PMD_HS && _g != (mode - PMD_HS_G1 + 1))
			match = UFS_CAL_ERROR;
	} else if (IS_PWR_MODE_PWM(_m) && mode >= PMD_PWM_G1 &&
		   mode <= PMD_PWM) {
		match = UFS_CAL_NO_ERROR;
		if (mode != PMD_PWM && _g != (mode - PMD_PWM_G1 + 1))
			match = UFS_CAL_ERROR;
	} else {
		/* invalid lanes */
		match = UFS_CAL_ERROR;
	}

	return match;
}

static enum ufs_cal_errno ufs_cal_wait_pll_lock(struct ufs_vs_handle *handle,
						u32 addr, u32 mask)
{
	u32 reg;
#if !defined(__UFS_CAL_FW__)
	u32 residue = TIMEOUT_IN_US;
#endif

#if defined(__UFS_CAL_FW__)
	while (1)
#else
	while (residue--)
#endif
	{
		reg = pma_readl(handle, PHY_PMA_COMN_ADDR(addr));
		if (mask == (reg & mask))
			return UFS_CAL_NO_ERROR;
		if (handle->udelay)
			handle->udelay(DELAY_PERIOD_IN_US);
	}

	return UFS_CAL_ERROR;
}

static enum ufs_cal_errno ufs_cal_wait_cdr_lock(struct ufs_vs_handle *handle,
						u32 addr, u32 mask, int lane)
{
	u32 reg;
#if !defined(__UFS_CAL_FW__)
	u32 residue = TIMEOUT_IN_US;
#endif

#if defined(__UFS_CAL_FW__)
	while (1)
#else
	while (residue--)
#endif
	{
		reg = pma_readl(handle, PHY_PMA_TRSV_ADDR(addr, lane));
		if (mask == (reg & mask))
			return UFS_CAL_NO_ERROR;
		if (handle->udelay)
			handle->udelay(DELAY_PERIOD_IN_US);
	}

	return UFS_CAL_ERROR;
}

static enum ufs_cal_errno ufs30_cal_wait_cdr_lock(struct ufs_vs_handle *handle,
						  u32 addr, u32 mask, int lane)
{
	u32 reg;
	u32 i;

	for (i = 0; i < 100; i++) {
		if (handle->udelay)
			handle->udelay(DELAY_PERIOD_IN_US);

		reg = pma_readl(handle, PHY_PMA_TRSV_ADDR(addr, lane));
		if (mask == (reg & mask))
			return UFS_CAL_NO_ERROR;
#if defined(__UFS_CAL_FW__)
		else
			return UFS_CAL_ERROR;
#endif
		if (handle->udelay)
			handle->udelay(DELAY_PERIOD_IN_US);

		pma_writel(handle, 0x10, PHY_PMA_TRSV_ADDR(0x888, lane));
		pma_writel(handle, 0x18, PHY_PMA_TRSV_ADDR(0x888, lane));
	}

	return UFS_CAL_ERROR;
}

static enum ufs_cal_errno ufs_cal_wait_cdr_afc_check(
					struct ufs_vs_handle *handle,
					u32 addr, u32 mask, int lane)
{
	u32 i;

	for (i = 0; i < 100; i++) {
		u32 reg = 0;

		if (handle->udelay)
			handle->udelay(DELAY_PERIOD_IN_US);

		reg = pma_readl(handle, PHY_PMA_TRSV_ADDR(addr, lane));
		if (mask == (reg & mask))
			return UFS_CAL_NO_ERROR;
#if defined(__UFS_CAL_FW__)
		else
			return UFS_CAL_ERROR;
#endif
		if (handle->udelay)
			handle->udelay(DELAY_PERIOD_IN_US);

		pma_writel(handle, 0x7F, PHY_PMA_TRSV_ADDR(0xF0, lane));
		pma_writel(handle, 0xFF, PHY_PMA_TRSV_ADDR(0xF0, lane));
	}

	return UFS_CAL_ERROR;
}

static enum ufs_cal_errno ufs30_cal_done_wait(struct ufs_vs_handle *handle,
					      u32 addr, u32 mask, int lane)
{
	u32 i;

	for (i = 0; i < 100; i++) {
		u32 reg = 0;

		if (handle->udelay)
			handle->udelay(DELAY_PERIOD_IN_US);

		reg = pma_readl(handle, PHY_PMA_TRSV_ADDR(addr, lane));
		if (mask == (reg & mask))
			return UFS_CAL_NO_ERROR;
	}

	return UFS_CAL_NO_ERROR;
}

static inline void __set_pcs(struct ufs_vs_handle *handle,
			     u8 lane, u32 offset, u32 value)
{
	unipro_writel(handle, __WSTRB | __SEL_IDX(lane),
				       UNIP_COMP_AXI_AUX_FIELD);
	unipro_writel(handle, value, offset);
	unipro_writel(handle, __WSTRB, UNIP_COMP_AXI_AUX_FIELD);
}

static enum ufs_cal_errno __config_uic(struct ufs_vs_handle *handle, u8 lane,
				       const struct ufs_cal_phy_cfg *cfg,
				       struct ufs_cal_param *p)
{
	u32 value;
	u32 ticks;
	enum ufs_cal_errno ret = UFS_CAL_NO_ERROR;

	switch (cfg->lyr) {
	/* unipro */
	case UNIPRO_STD_MIB:
	case UNIPRO_DBG_MIB:
		unipro_writel(handle, cfg->val, cfg->addr);
		break;
	case UNIPRO_ADAPT_LENGTH:
		value = unipro_readl(handle, cfg->addr);
		if (value & 0x80) {
			if ((value & 0x7F) < 2)
				unipro_writel(handle, 0x82, cfg->addr);
		} else {
			if (((value + 1) % 4) != 0) {
				do {
					value++;
				} while (((value + 1) % 4) != 0);
				unipro_writel(handle, value, cfg->addr);
			}
		}
		break;
	case UNIPRO_DBG_PRD:
		unipro_writel(handle, p->mclk_period_unipro_18, cfg->addr);
		break;
	case UNIPRO_DBG_APB:
		unipro_writel(handle, cfg->val, cfg->addr);
		break;

	/* pcs */
	case PHY_PCS_COMN:
		unipro_writel(handle, cfg->val, cfg->addr);
		break;
	case PHY_PCS_RX:
		__set_pcs(handle, RX_LANE_0 + lane, cfg->addr, cfg->val);
		break;
	case PHY_PCS_TX:
		__set_pcs(handle, TX_LANE_0 + lane, cfg->addr, cfg->val);
		break;
	case PHY_PCS_RX_PRD:
		__set_pcs(handle, RX_LANE_0 + lane, cfg->addr, p->mclk_period);
		break;
	case PHY_PCS_TX_PRD:
		__set_pcs(handle, TX_LANE_0 + lane, cfg->addr, p->mclk_period);
		break;
	case PHY_PCS_RX_PRD_ROUND_OFF:
		__set_pcs(handle, RX_LANE_0 + lane, cfg->addr,
			  p->mclk_period_rnd_off);
		break;
	case PHY_PCS_TX_PRD_ROUND_OFF:
		__set_pcs(handle, TX_LANE_0 + lane, cfg->addr,
			  p->mclk_period_rnd_off);
		break;
	case PHY_PCS_RX_LR_PRD:
		ticks = __get_line_reset_ticks(p, RX_LINE_RESET_DETECT_TIME);
		__set_pcs(handle, RX_LANE_0 + lane, cfg->addr,
			  (ticks >> 16) & 0xFF);
		__set_pcs(handle, RX_LANE_0 + lane, cfg->addr + 4,
			  (ticks >> 8) & 0xFF);
		__set_pcs(handle, RX_LANE_0 + lane, cfg->addr + 8,
			  (ticks >> 0) & 0xFF);
		break;
	case PHY_PCS_TX_LR_PRD:
		ticks = __get_line_reset_ticks(p, TX_LINE_RESET_TIME);
		__set_pcs(handle, TX_LANE_0 + lane, cfg->addr,
			  (ticks >> 16) & 0xFF);
		__set_pcs(handle, TX_LANE_0 + lane, cfg->addr + 4,
			  (ticks  >> 8) & 0xFF);
		__set_pcs(handle, TX_LANE_0 + lane, cfg->addr + 8,
			  (ticks >> 0) & 0xFF);
		break;

	/* pma */
	case PHY_PMA_COMN:
		pma_writel(handle, cfg->val, PHY_PMA_COMN_ADDR(cfg->addr));
		break;
	case PHY_PMA_TRSV:
		pma_writel(handle, cfg->val,
			   PHY_PMA_TRSV_ADDR(cfg->addr, lane));
		break;
	case PHY_PLL_WAIT:
		if (ufs_cal_wait_pll_lock(handle, cfg->addr, cfg->val)
		    == UFS_CAL_ERROR)
			ret = UFS_CAL_TIMEOUT;
		break;
	case PHY_CDR_WAIT:
		/* after gear change */
		if (ufs_cal_wait_cdr_lock(handle, cfg->addr, cfg->val, lane)
		    == UFS_CAL_ERROR)
			ret = UFS_CAL_TIMEOUT;
		break;
	case PHY_EMB_CDR_WAIT:
		/* after gear change */
		if (ufs30_cal_wait_cdr_lock(p->handle, cfg->addr, cfg->val,
					    lane)
		    == UFS_CAL_ERROR)
			ret = UFS_CAL_TIMEOUT;
		break;
		/* after gear change */
	case PHY_CDR_AFC_WAIT:
		if (p->tbl == HOST_CARD) {
			if (ufs_cal_wait_cdr_afc_check(p->handle,
						       cfg->addr,
						       cfg->val, lane)
						       == UFS_CAL_ERROR)
				ret = UFS_CAL_TIMEOUT;
		}
		break;
	case PHY_EMB_CAL_WAIT:
		if (ufs30_cal_done_wait(p->handle, cfg->addr, cfg->val, lane)
		    == UFS_CAL_ERROR)
			ret = UFS_CAL_TIMEOUT;
		break;
	case COMMON_WAIT:
		if (handle->udelay)
			handle->udelay(cfg->val);
		break;
	case PHY_PMA_TRSV_SQ:
		/* for hibern8 time */
		pma_writel(handle, cfg->val, PHY_PMA_TRSV_ADDR(cfg->addr,
			   lane));
		break;
	case PHY_PMA_TRSV_LANE1_SQ_OFF:
		/* for hibern8 time */
		pma_writel(handle, cfg->val, PHY_PMA_TRSV_ADDR(cfg->addr,
			   lane));
		break;
	default:
		break;
	}

	return ret;
}

static enum ufs_cal_errno ufs_cal_config_uic(struct ufs_cal_param *p,
					     const struct ufs_cal_phy_cfg *cfg,
					     struct uic_pwr_mode *pmd)
{
	struct ufs_vs_handle *handle = p->handle;
	u8 i = 0;
	int skip;
	enum ufs_cal_errno ret = UFS_CAL_INV_ARG;

	if (!cfg)
		goto out;

	ret = UFS_CAL_NO_ERROR;
	for (; cfg->lyr != PHY_CFG_NONE; cfg++) {
		for (i = 0; i < p->available_lane; i++) {
			if (p->board && UFS_CAL_ERROR ==
				__match_board_by_cfg(p->board, cfg->board))
				continue;
			if (pmd && UFS_CAL_ERROR ==
				__match_mode_by_cfg(pmd, cfg->flg))
				continue;

			if (i > 0) {
				skip = 0;
				switch (cfg->lyr) {
				case PHY_PCS_COMN:
				case UNIPRO_STD_MIB:
				case UNIPRO_DBG_MIB:
				case UNIPRO_ADAPT_LENGTH:
				case UNIPRO_DBG_PRD:
				case PHY_PMA_COMN:
				case UNIPRO_DBG_APB:
				case PHY_PLL_WAIT:
				case COMMON_WAIT:
					skip = 1;
				default:
					break;
				}
				if (skip)
					continue;
			}
			if (i >= p->active_rx_lane) {
				skip = 0;
				switch (cfg->lyr) {
				case PHY_CDR_WAIT:
				case PHY_EMB_CDR_WAIT:
				case PHY_CDR_AFC_WAIT:
					skip = 1;
				default:
					break;
				}
				if (skip)
					continue;
			}
			if (cfg->lyr == PHY_PMA_TRSV_LANE1_SQ_OFF &&
			    i < p->connected_rx_lane)
				continue;
			if (cfg->lyr == PHY_PMA_TRSV_SQ &&
			    i >= p->connected_rx_lane)
				continue;

			ret = __config_uic(handle, i, cfg, p);
			if (ret)
				goto out;
		}
	}
out:
	return ret;
}

/*
 * public functions
 */
#if defined(__UFS_CAL_FW__)
enum ufs_cal_errno ufs_cal_loopback_init(struct ufs_cal_param *p)
{
	enum ufs_cal_errno ret = UFS_CAL_NO_ERROR;
	static const struct ufs_cal_phy_cfg *cfg;

	cfg = (p->tbl == HOST_CARD) ? loopback_init_card : loopback_init;
	ret = ufs_cal_config_uic(p, cfg, NULL);

	return ret;
}

enum ufs_cal_errno ufs_cal_loopback_set_1(struct ufs_cal_param *p)
{
	enum ufs_cal_errno ret = UFS_CAL_NO_ERROR;
	static const struct ufs_cal_phy_cfg *cfg;

	cfg = (p->tbl == HOST_CARD) ? loopback_set_1_card : loopback_set_1;
	ret = ufs_cal_config_uic(p, cfg, NULL);

	return ret;
}

enum ufs_cal_errno ufs_cal_loopback_set_2(struct ufs_cal_param *p)
{
	enum ufs_cal_errno ret = UFS_CAL_NO_ERROR;
	static const struct ufs_cal_phy_cfg *cfg;

	cfg = (p->tbl == HOST_CARD) ? loopback_set_2_card : loopback_set_2;
	ret = ufs_cal_config_uic(p, cfg, NULL);

	return ret;
}
#endif

enum ufs_cal_errno ufs_cal_post_h8_enter(struct ufs_cal_param *p)
{
	enum ufs_cal_errno ret = UFS_CAL_NO_ERROR;
	struct ufs_cal_phy_cfg *cfg;

	cfg = (p->tbl == HOST_CARD) ? post_h8_enter_card : post_h8_enter;
	ret = ufs_cal_config_uic(p, cfg, p->pmd);

	return ret;
}

enum ufs_cal_errno ufs_cal_pre_h8_exit(struct ufs_cal_param *p)
{
	enum ufs_cal_errno ret = UFS_CAL_NO_ERROR;
	struct ufs_cal_phy_cfg *cfg;

	cfg = (p->tbl == HOST_CARD) ? pre_h8_exit_card : pre_h8_exit;
	ret = ufs_cal_config_uic(p, cfg, p->pmd);

	return ret;
}

/*
 * This currently uses only SLOW_MODE and FAST_MODE.
 * If you want others, you should modify this function.
 */
enum ufs_cal_errno ufs_cal_pre_pmc(struct ufs_cal_param *p)
{
	enum ufs_cal_errno ret = UFS_CAL_NO_ERROR;
	struct ufs_cal_phy_cfg *cfg;
	struct ufs_vs_handle *handle = p->handle;
	u32 dl_error;

	/* block PA_ERROR_IND_RECEIVED */
	dl_error = unipro_readl(handle, UNIP_DL_ERROR_IRQ_MASK) |
						PA_ERROR_IND_RECEIVED;
	unipro_writel(handle, dl_error, UNIP_DL_ERROR_IRQ_MASK);

	if (p->pmd->mode == SLOW_MODE || p->pmd->mode == SLOWAUTO_MODE)
		cfg = (p->tbl == HOST_CARD) ? calib_of_pwm_card : calib_of_pwm;
	else if (p->pmd->hs_series == PA_HS_MODE_B)
		cfg = (p->tbl == HOST_CARD) ? calib_of_hs_rate_b_card :
							calib_of_hs_rate_b;
	else if (p->pmd->hs_series == PA_HS_MODE_A)
		cfg = (p->tbl == HOST_CARD) ? calib_of_hs_rate_a_card :
							calib_of_hs_rate_a;
	else
		return UFS_CAL_INV_ARG;

	ret = ufs_cal_config_uic(p, cfg, p->pmd);

	return ret;
}

/*
 * This currently uses only SLOW_MODE and FAST_MODE.
 * If you want others, you should modify this function.
 */
enum ufs_cal_errno ufs_cal_post_pmc(struct ufs_cal_param *p)
{
	enum ufs_cal_errno ret = UFS_CAL_NO_ERROR;
	struct ufs_cal_phy_cfg *cfg;

	if (p->pmd->mode == SLOWAUTO_MODE || p->pmd->mode == SLOW_MODE)
		cfg = (p->tbl == HOST_CARD) ? post_calib_of_pwm_card :
					post_calib_of_pwm;
	else if (p->pmd->hs_series == PA_HS_MODE_B)
		cfg = (p->tbl == HOST_CARD) ? post_calib_of_hs_rate_b_card :
					post_calib_of_hs_rate_b;
	else if (p->pmd->hs_series == PA_HS_MODE_A)
		cfg = (p->tbl == HOST_CARD) ? post_calib_of_hs_rate_a_card :
					post_calib_of_hs_rate_a;
	else
		return UFS_CAL_INV_ARG;

	ret = ufs_cal_config_uic(p, cfg, p->pmd);

	return ret;
}

enum ufs_cal_errno ufs_cal_post_link(struct ufs_cal_param *p)
{
	enum ufs_cal_errno ret = UFS_CAL_NO_ERROR;
	static struct ufs_cal_phy_cfg *cfg;

	switch (p->max_gear) {
	case GEAR_1:
	case GEAR_2:
	case GEAR_3:
	case GEAR_4:
		if (p->evt_ver == 0)
			cfg = (p->tbl == HOST_CARD) ? post_init_cfg_card :
						post_init_cfg_evt0;
		else
			cfg = (p->tbl == HOST_CARD) ? post_init_cfg_card :
						post_init_cfg_evt1;
		break;
	default:
		ret = UFS_CAL_INV_ARG;
		break;
	}

	if (ret)
		return ret;

	ret = ufs_cal_config_uic(p, cfg, NULL);

	/*
	 * If a number of target lanes is 1 and a host's
	 * a number of available lanes is 2,
	 * you should turn off phy power of lane #1.
	 *
	 * This must be modified when a number of available lanes
	 * would grow in the future.
	 */
	if (ret == UFS_CAL_NO_ERROR) {
		if (p->available_lane == 2 && p->connected_rx_lane == 1) {
			cfg = (p->tbl == HOST_CARD) ?
				lane1_sq_off_card : lane1_sq_off;
			ret = ufs_cal_config_uic(p, cfg, NULL);
		}
	}

	/* eom */
	p->eom_sz = EOM_PH_SEL_MAX * EOM_DEF_VREF_MAX *
		ufs_s_eom_repeat[p->max_gear];

	return ret;
}

enum ufs_cal_errno ufs_cal_pre_link(struct ufs_cal_param *p)
{
	enum ufs_cal_errno ret = UFS_CAL_NO_ERROR;
	static const struct ufs_cal_phy_cfg *cfg;

	/* preset mclk periods */
	p->mclk_period = __get_mclk_period(p);
	p->mclk_period_rnd_off = __get_mclk_period_rnd_off(p);
	p->mclk_period_unipro_18 = __get_mclk_period_unipro_18(p);

	if (p->evt_ver == 0)
		cfg = (p->tbl == HOST_CARD) ? init_cfg_card : init_cfg_evt0;
	else
		cfg = (p->tbl == HOST_CARD) ? init_cfg_card : init_cfg_evt1;

	ret = ufs_cal_config_uic(p, cfg, NULL);

	return ret;
}

static enum ufs_cal_errno ufs_cal_eom_prepare(struct ufs_cal_param *p)
{
	enum ufs_cal_errno ret = UFS_CAL_NO_ERROR;
	struct ufs_cal_phy_cfg *cfg;

	cfg = eom_prepare;
	ret = ufs_cal_config_uic(p, cfg, p->pmd);
	return ret;
}

static u32 ufs_cal_get_eom_err_cnt(struct ufs_vs_handle *handle, u32 lane_loop)
{
	return (u32)((pma_readl(handle,
		PHY_PMA_TRSV_ADDR(0xD20, lane_loop)) << 16)
		+ (pma_readl(handle, PHY_PMA_TRSV_ADDR(0xD24, lane_loop)) << 8)
		+ (pma_readl(handle, PHY_PMA_TRSV_ADDR(0xD28, lane_loop))));
}

static void ufs_cal_sweep_get_eom_data(struct ufs_vs_handle *handle, u32 *cnt,
				       struct ufs_cal_param *p, u32 lane,
				       u32 repeat)
{
	u32 phase, vref;
	u32 errors;
	struct ufs_eom_result_s *data = p->eom[lane];

	for (phase = 0; phase < EOM_PH_SEL_MAX; phase++) {
		pma_writel(handle, phase, PHY_PMA_TRSV_ADDR(0xB78, lane));

		for (vref = 0; vref < EOM_DEF_VREF_MAX; vref++) {
			pma_writel(handle, 0x18,
				   PHY_PMA_TRSV_ADDR(0xB5C, lane));
			pma_writel(handle, vref,
				   PHY_PMA_TRSV_ADDR(0xB74, lane));
			pma_writel(handle, 0x19,
				   PHY_PMA_TRSV_ADDR(0xB5C, lane));

			errors = ufs_cal_get_eom_err_cnt(handle, lane);

			if (handle->udelay)
				handle->udelay(1);

			data[*cnt].v_phase =
					phase + (repeat * EOM_PH_SEL_MAX);
			data[*cnt].v_vref = vref;
			data[*cnt].v_err = errors;
			(*cnt)++;
		}
	}
}

enum ufs_cal_errno ufs_cal_eom(struct ufs_cal_param *p)
{
	u32 repeat;
	u32 lane;
	u32 i;
	u32 cnt;
	struct ufs_vs_handle *handle = p->handle;
	u32 num_of_active_rx = p->available_lane;
	enum ufs_cal_errno res = UFS_CAL_NO_ERROR;

	ufs_cal_eom_prepare(p);

	repeat = (p->max_gear < GEAR_MAX) ? ufs_s_eom_repeat[p->max_gear] : 0;
	if (repeat == 0) {
		res = UFS_CAL_ERROR;
		goto end;
	} else {
		for (i = GEAR_1 ; i < GEAR_MAX ; i++) {
			if (repeat > EOM_RTY_MAX) {
				res = UFS_CAL_INV_CONF;
				goto end;
			}
		}
	}

	for (lane = 0; lane < num_of_active_rx; lane++) {
		cnt = 0;
		for (i = 0; i < repeat; i++)
			ufs_cal_sweep_get_eom_data(handle, &cnt, p, lane, i);
	}
end:
	return res;
}

enum ufs_cal_errno ufs_cal_init(struct ufs_cal_param *p, int idx)
{
	/*
	 * Return if innput index is greater than
	 * the maximum that cal supports
	 */
	if (idx >= NUM_OF_UFS_HOST)
		return UFS_CAL_INV_ARG;

	ufs_cal[idx] = p;

	return UFS_CAL_NO_ERROR;
}
