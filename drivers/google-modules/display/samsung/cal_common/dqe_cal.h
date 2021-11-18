/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for GS101 DQE CAL
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SAMSUNG_DQE_CAL_H__
#define __SAMSUNG_DQE_CAL_H__

#include <linux/types.h>
#include <drm/samsung_drm.h>
#include <drm/drm_mode.h>
#include <drm/drm_print.h>

#define DEGAMMA_LUT_SIZE		65
#define REGAMMA_LUT_SIZE		65
#define CGC_LUT_SIZE			4913
#define HIST_BIN_SIZE			256
#define LPD_ATC_REG_CNT			45
#define GAMMA_MATRIX_COEFFS_CNT		9
#define GAMMA_MATRIX_OFFSETS_CNT	3
#define LINEAR_MATRIX_COEFFS_CNT	9
#define LINEAR_MATRIX_OFFSETS_CNT	3

enum dqe_version {
	DQE_V1,
	DQE_V2,
};

enum dqe_dither_type {
	CGC_DITHER = 0,
	DISP_DITHER = 1,
};

enum histogram_state {
	HISTOGRAM_OFF,
	HISTOGRAM_FULL,
	HISTOGRAM_ROI,
};

struct exynos_atc {
	bool en;
	bool dirty;
	__u8 lt;
	__u8 ns;
	__u8 st;
	bool dither;
	__u8 pl_w1;
	__u8 pl_w2;
	__u8 ctmode;
	bool pp_en;
	__u8 upgrade_on;
	__u16 tdr_max;
	__u16 tdr_min;
	__u8 ambient_light;
	__u8 back_light;
	__u8 dstep;
	__u8 actual_dstep;
	__u8 scale_mode;
	__u8 threshold_1;
	__u8 threshold_2;
	__u8 threshold_3;
	__u16 gain_limit;
	__u8 lt_calc_ab_shift;
};

void dqe_regs_desc_init(void __iomem *regs, phys_addr_t start,
			const char *name, enum dqe_version ver);
void dqe_reg_init(u32 width, u32 height);
void dqe_reg_set_degamma_lut(const struct drm_color_lut *lut);
void dqe_reg_set_cgc_lut(const struct cgc_lut *lut);
void dqe_reg_set_regamma_lut(const struct drm_color_lut *lut);
void dqe_reg_set_cgc_dither(struct dither_config *config);
void dqe_reg_set_disp_dither(struct dither_config *config);
void dqe_reg_set_linear_matrix(const struct exynos_matrix *lm);
void dqe_reg_set_gamma_matrix(const struct exynos_matrix *matrix);
void dqe_reg_set_atc(const struct exynos_atc *atc);
void dqe_reg_print_dither(enum dqe_dither_type dither, struct drm_printer *p);
void dqe_reg_print_degamma_lut(struct drm_printer *p);
void dqe_reg_print_cgc_lut(u32 count, struct drm_printer *p);
void dqe_reg_print_regamma_lut(struct drm_printer *p);
void dqe_reg_print_hist(struct drm_printer *p);
void dqe_reg_print_gamma_matrix(struct drm_printer *p);
void dqe_reg_print_linear_matrix(struct drm_printer *p);
void dqe_reg_print_atc(struct drm_printer *p);
void dqe_reg_save_lpd_atc(u32 *lpd_atc_regs);
void dqe_reg_restore_lpd_atc(u32 *lpd_atc_regs);
bool dqe_reg_dimming_in_progress(void);
void dqe_reg_set_histogram_roi(struct histogram_roi *roi);
void dqe_reg_set_histogram_weights(struct histogram_weights *weights);
void dqe_reg_set_histogram_threshold(u32 threshold);
void dqe_reg_set_histogram(enum histogram_state state);
void dqe_reg_get_histogram_bins(struct histogram_bins *bins);
void dqe_reg_set_size(u32 width, u32 height);
void dqe_dump(void);
void dqe_reg_set_drm_write_protected(bool protected);
#endif /* __SAMSUNG_DQE_CAL_H__ */
