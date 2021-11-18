/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Power management header for Abrolhos.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __ABROLHOS_PM_H__
#define __ABROLHOS_PM_H__

#include "edgetpu-kci.h"
#include "edgetpu-internal.h"

/* Can't build out of tree with acpm_dvfs unless kernel supports ACPM */
#if IS_ENABLED(CONFIG_ACPM_DVFS) || IS_ENABLED(CONFIG_EDGETPU_TEST)

#include <linux/acpm_dvfs.h>

#else

static unsigned long exynos_acpm_rate;
static inline int exynos_acpm_set_rate(unsigned int id, unsigned long rate)
{
	exynos_acpm_rate = rate;
	return 0;
}
static inline int exynos_acpm_set_init_freq(unsigned int dfs_id,
					    unsigned long freq)
{
	return 0;
}
static inline unsigned long exynos_acpm_get_rate(unsigned int id,
						 unsigned long dbg_val)
{
	return exynos_acpm_rate;
}
static inline int exynos_acpm_set_policy(unsigned int id, unsigned long policy)
{
	return 0;
}
#endif /* IS_ENABLED(CONFIG_ACPM_DVFS) || IS_ENABLED(CONFIG_EDGETPU_TEST) */

/*
 * TPU Power States:
 * 0:		Off
 * 1:		Deep Sleep Clocks Off
 * 2:		Deep Sleep Clocks Slow
 * 3:		Deep Sleep Clocks Fast
 * 4:		Sleep Clocks Off
 * 5:		Sleep Clocks Slow
 * 6:		Retention Clocks Slow
 * 226000:	Ultra Underdrive @226MHz
 * 500000:	Super Underdrive @500MHz
 * 800000:	Underdrive @800MHz
 * 1000000:	Nominal @1066MHz
 * 1200000:	Overdrive @1230MHz
 */
enum tpu_pwr_state {
	TPU_OFF = 0,
	TPU_DEEP_SLEEP_CLOCKS_OFF  = 1,
	TPU_DEEP_SLEEP_CLOCKS_SLOW = 2,
	TPU_DEEP_SLEEP_CLOCKS_FAST = 3,
	TPU_SLEEP_CLOCKS_OFF       = 4,
	TPU_SLEEP_CLOCKS_SLOW      = 5,
	TPU_RETENTION_CLOCKS_SLOW  = 6,
	TPU_ACTIVE_UUD = 226000,
	TPU_ACTIVE_SUD = 500000,
	TPU_ACTIVE_UD  = 800000,
	TPU_ACTIVE_NOM = 1066000,
	TPU_ACTIVE_OD  = 1230000,
};

/*
 * Request codes from firmware
 * Values must match with firmware code base
 */
enum abrolhos_reverse_kci_code {
	RKCI_CODE_PM_QOS = RKCI_CHIP_CODE_FIRST + 1,
	RKCI_CODE_BTS = RKCI_CHIP_CODE_FIRST + 2,
};

#define TPU_POLICY_MAX	TPU_ACTIVE_OD

#define TPU_ACPM_DOMAIN			7

#define TPU_DEBUG_REQ			(1 << 31)
#define TPU_VDD_TPU_DEBUG		(0 << 27)
#define TPU_VDD_TPU_M_DEBUG		(1 << 27)
#define TPU_VDD_INT_M_DEBUG		(2 << 27)
#define TPU_CLK_CORE_DEBUG		(3 << 27)
#define TPU_CLK_CTL_DEBUG		(4 << 27)
#define TPU_CLK_AXI_DEBUG		(5 << 27)
#define TPU_CLK_APB_DEBUG		(6 << 27)
#define TPU_CLK_UART_DEBUG		(7 << 27)
#define TPU_CORE_PWR_DEBUG		(8 << 27)
#define TPU_DEBUG_VALUE_MASK		((1 << 27) - 1)

#define OSCCLK_RATE			24576
#define PLL_SHARED0_DIV0		1066000
#define PLL_SHARED1_DIV2		933000
#define PLL_SHARED2			800000
#define PLL_SHARED3			666000
#define PLL_SHARED0_DIV3		711000
#define PLL_SHARED1_DIV3		622000
#define PLL_SHARED0_DIV4		533000
#define PLL_SHARED2_DIV2		400000
#define PLL_SHARED3_DIV2		333000

#define TPU_CMU_TOP_REG			0x1E080000
#define MUX_CLKCMU_TPU_TPU		(TPU_CMU_TPU_REG + 0x10FC)
#define DIV_CLKCMU_TPU_TPU		(TPU_CMU_TPU_REG + 0x18EC)

#define MUX_CLKCMU_TPU_TPUCTL		(TPU_CMU_TPU_REG + 0x1100)
#define DIV_CLKCMU_TPU_TPUCTL		(TPU_CMU_TPU_REG + 0x18F0)

#define MUX_CLKCMU_TPU_BUS		(TPU_CMU_TPU_REG + 0x10F8)
#define DIV_CLKCMU_TPU_BUS		(TPU_CMU_TPU_REG + 0x18E8)

#define MUX_CLKCMU_TPU_UART		(TPU_CMU_TPU_REG + 0x1104)
#define DIV_CLKCMU_TPU_UART		(TPU_CMU_TPU_REG + 0x18F4)

#define TPU_CMU_TPU_REG			0x1CC00000

#define PLL_CON0_PLL_TPU		(TPU_CMU_TPU_REG + 0x0100)
#define PLL_CON2_PLL_TPU		(TPU_CMU_TPU_REG + 0x0108)
#define MUX_CLK_TPU_TPU			(TPU_CMU_TPU_REG + 0x1000)
#define MUX_CLKCMU_TPU_TPU_USER		(TPU_CMU_TPU_REG + 0x0620)
#define DIV_CLK_TPU_TPU			(TPU_CMU_TPU_REG + 0x1804)

#define MUX_CLK_TPU_TPUCTL		(TPU_CMU_TPU_REG + 0x1004)
#define MUX_CLKCMU_TPU_TPUCTL_USER	(TPU_CMU_TPU_REG + 0x0610)
#define DIV_CLK_TPU_TPUCTL		(TPU_CMU_TPU_REG + 0x1808)

#define MUX_CLKCMU_TPU_BUS_USER		(TPU_CMU_TPU_REG + 0x0600)

#define DIV_CLK_TPU_BUSP		(TPU_CMU_TPU_REG + 0x1800)

#define MUX_CLKCMU_TPU_UART_USER	(TPU_CMU_TPU_REG + 0x0630)

#define MUX_USER_SEL_MASK		0x1
#define MUX_USER_OSCCLK_SEL		0
#define MUX_USER_CMUCLK_SEL		1

#define MUX_CMU_SEL_MASK		0x7
#define MUX_CMU_S0_D0_SEL		0
#define MUX_CMU_S1_D2_SEL		1
#define MUX_CMU_S2_SEL			2
#define MUX_CMU_S3_SEL			3
#define MUX_CMU_S0_D3_SEL		4
#define MUX_CMU_S1_D3_SEL		5
#define MUX_CMU_S0_D4_SEL		6

#define MUX_CMU_UART_SEL_MASK		0x3
#define MUX_CMU_UART_S0_D4_SEL		0
#define MUX_CMU_UART_S2_D2_SEL		1
#define MUX_CMU_UART_S3_D2_SEL		2

#define DIV_CMU_RATIO_MASK		0xf
#define DIV_USER_RATIO_MASK		0x7

int abrolhos_pm_create(struct edgetpu_dev *etdev);

void abrolhos_pm_destroy(struct edgetpu_dev *etdev);

void abrolhos_pm_set_pm_qos(struct edgetpu_dev *etdev, u32 pm_qos_val);

void abrolhos_pm_set_bts(struct edgetpu_dev *etdev, u32 bts_val);

#endif /* __ABROLHOS_PM_H__ */
