/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2021 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo. Please contact Qorvo to inquire about licensing terms.
 */
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/bitfield.h>

#include "dw3000.h"
#include "dw3000_core.h"
#define DEFINE_COMPAT_REGISTERS
#include "dw3000_core_reg.h"
#include "dw3000_stm.h"
#include "dw3000_trc.h"
#include "dw3000_perf.h"
#include "dw3000_pm.h"
#include "dw3000_coex.h"
#include "dw3000_nfcc_coex_core.h"
#include "dw3000_txpower_adjustment.h"

/* Table of supported chip version and associated chip operations */
const struct dw3000_chip_version dw3000_chip_versions[] = {
	{ .id = DW3000_C0_DEV_ID,
	  .ver = DW3000_C0_VERSION,
	  .ops = &dw3000_chip_c0_ops,
	  .name = "C0" },
	{ .id = DW3000_C0_PDOA_DEV_ID,
	  .ver = DW3000_C0_VERSION,
	  .ops = &dw3000_chip_c0_ops,
	  .name = "C0" },
	{ .id = DW3000_D0_DEV_ID,
	  .ver = DW3000_D0_VERSION,
	  .ops = &dw3000_chip_d0_ops,
	  .name = "D0" },
	{ .id = DW3000_D0_PDOA_DEV_ID,
	  .ver = DW3000_D0_VERSION,
	  .ops = &dw3000_chip_d0_ops,
	  .name = "D0" },
	{ .id = DW3000_E0_PDOA_DEV_ID,
	  .ver = DW3000_E0_VERSION,
	  .ops = &dw3000_chip_e0_ops,
	  .name = "E0" },
};

/* DW3000 hard reset delay (us) */
#define DW3000_HARD_RESET_DELAY_US 10000

/* DW3000 soft reset delay (us) */
#define DW3000_SOFT_RESET_DELAY_US 1000

/* DW3000 pgf calibration delay (us) */
#define DW3000_PGFCAL_DELAY_US 1000

/* The pin operates as the EXTTXE output (C0, output TX state) */
#define DW3000_GPIO_PIN0_EXTTXE \
	(((u32)0x2) << DW3000_GPIO_MODE_MSGP0_MODE_BIT_OFFSET)
/* The pin operates as the EXTRXE output (C0, output RX state) */
#define DW3000_GPIO_PIN1_EXTRXE \
	(((u32)0x2) << DW3000_GPIO_MODE_MSGP1_MODE_BIT_OFFSET)

/* The pin operates as the RXLED output (C0 & D0) */
#define DW3000_GPIO_PIN2_RXLED \
	(((u32)0x1) << DW3000_GPIO_MODE_MSGP2_MODE_BIT_OFFSET)
/* The pin operates as the TXLED output (C0 & D0) */
#define DW3000_GPIO_PIN3_TXLED \
	(((u32)0x1) << DW3000_GPIO_MODE_MSGP3_MODE_BIT_OFFSET)

/* The pin operates as the EXTTXE output (D0, output TX state) */
#define DW3000_GPIO_PIN4_EXTTXE \
	(((u32)0x2) << DW3000_GPIO_MODE_MSGP4_MODE_BIT_OFFSET)
/* The pin operates as the EXTRXE output (D0, output RX state) */
#define DW3000_GPIO_PIN5_EXTRXE \
	(((u32)0x2) << DW3000_GPIO_MODE_MSGP5_MODE_BIT_OFFSET)

/* The pin operates to support external DA/PA (C0) */
#define DW3000_GPIO_PIN4_EXTDA \
	(((u32)0x1) << DW3000_GPIO_MODE_MSGP4_MODE_BIT_OFFSET)
/* The pin operates to support external PA (C0) */
#define DW3000_GPIO_PIN5_EXTTX \
	(((u32)0x1) << DW3000_GPIO_MODE_MSGP5_MODE_BIT_OFFSET)
/* The pin operates to support external LNA (C0) */
#define DW3000_GPIO_PIN6_EXTRX \
	(((u32)0x1) << DW3000_GPIO_MODE_MSGP6_MODE_BIT_OFFSET)

/* Defined constants for "mode" bit field parameter passed to dw3000_set_leds()
   function. */
#define DW3000_LEDS_DISABLE 0x00
#define DW3000_LEDS_ENABLE 0x01
#define DW3000_LEDS_INIT_BLINK 0x02
/* Default blink time. Blink time is expressed in multiples of 14 ms.
   The value defined here is ~225 ms. */
#define DW3000_LEDS_BLINK_TIME_DEF 0x10

/* Defined constants for "lna_pa" bit field parameter passed to
   dw3000_set_lna_pa_mode() function */
#define DW3000_LNA_PA_DISABLE 0x00
#define DW3000_LNA_ENABLE 0x01
#define DW3000_PA_ENABLE 0x02
#define DW3000_TXRX_ENABLE 0x04

/* Duration of the negative pulse of the SPI CS signal
   to wake up the chip in microseconds */
#define DW3000_SPI_CS_WAKEUP_DELAY_US 500

/* DW3000 double buffered receiver mode */
#define DW3000_DBL_BUFF_OFF 0x0
#define DW3000_DBL_BUFF_SWAP 0x2
#define DW3000_DBL_BUFF_ACCESS_BUFFER_A 0x1
#define DW3000_DBL_BUFF_ACCESS_BUFFER_B 0x3

#define DW3000_SFDTOC_DEF 129 /* default SFD timeout value */

/* DW3000 OTP operating parameter set selection */
#define DW3000_OPSET_LONG (0x0 << 11)
#define DW3000_OPSET_SCP (0x1 << 11)
#define DW3000_OPSET_SHORT (0x2 << 11)

#define DW3000_RX_FINFO_STD_RXFLEN_MASK \
	0x0000007FUL /* Receive Frame Length (0 to 127) */

/* Receive frame information register */
#define DW3000_RX_FINFO_RXFLEN(val) (((val) >> 0) & 0x7ff)
#define DW3000_RX_FINFO_RXNSPL(val) (((val) >> 11) & 0x3)
#define DW3000_RX_FINFO_RXPSR(val) (((val) >> 18) & 0x3)
#define DW3000_RX_FINFO_RXPACC(val) (((val) >> 20) & 0xfff)

/* RX events mask relating to reception into RX buffer A & B,  when double
   buffer is used */
#define DW3000_RDB_STATUS_CLEAR_BUFF0_EVENTS \
	((u8)0xf << DW3000_RDB_STATUS_RXFCG0_BIT_OFFSET)
#define DW3000_RDB_STATUS_CLEAR_BUFF1_EVENTS \
	((u8)0xf << DW3000_RDB_STATUS_RXFCG1_BIT_OFFSET)

#define DW3000_RDB_STATUS_RXOK               \
	(DW3000_RDB_STATUS_RXFR0_BIT_MASK |  \
	 DW3000_RDB_STATUS_RXFCG0_BIT_MASK | \
	 DW3000_RDB_STATUS_RXFR1_BIT_MASK | DW3000_RDB_STATUS_RXFCG1_BIT_MASK)

#define DW3000_SYS_STATUS_TX (DW3000_SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK)
#define DW3000_SYS_STATUS_RX                          \
	(DW3000_SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK | \
	 DW3000_SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK | \
	 DW3000_SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK | \
	 DW3000_SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK | \
	 DW3000_SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK | \
	 DW3000_SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK | \
	 DW3000_SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK | \
	 DW3000_SYS_ENABLE_LO_ARFE_ENABLE_BIT_MASK)
#define DW3000_SYS_STATUS_TRX (DW3000_SYS_STATUS_TX | DW3000_SYS_STATUS_RX)

/* All RX errors mask */
#define DW3000_SYS_STATUS_ALL_RX_ERR                                           \
	(DW3000_SYS_STATUS_RXPHE_BIT_MASK | DW3000_SYS_STATUS_RXFCE_BIT_MASK | \
	 DW3000_SYS_STATUS_RXFSL_BIT_MASK | DW3000_SYS_STATUS_RXSTO_BIT_MASK | \
	 DW3000_SYS_STATUS_ARFE_BIT_MASK | DW3000_SYS_STATUS_CIAERR_BIT_MASK | \
	 DW3000_SYS_STATUS_CPERR_BIT_MASK |                                    \
	 DW3000_SYS_STATUS_LCSSERR_BIT_MASK)

/* User defined RX timeouts (frame wait timeout and preamble detect timeout)
   mask. */
#define DW3000_SYS_STATUS_ALL_RX_TO \
	(DW3000_SYS_STATUS_RXFTO_BIT_MASK | DW3000_SYS_STATUS_RXPTO_BIT_MASK)

/* All RX events after a correct packet reception mask */
#define DW3000_SYS_STATUS_ALL_RX_GOOD                                         \
	(DW3000_SYS_STATUS_RXFR_BIT_MASK | DW3000_SYS_STATUS_RXFCG_BIT_MASK | \
	 DW3000_SYS_STATUS_RXPRD_BIT_MASK |                                   \
	 DW3000_SYS_STATUS_RXSFDD_BIT_MASK |                                  \
	 DW3000_SYS_STATUS_RXPHD_BIT_MASK |                                   \
	 DW3000_SYS_STATUS_CIA_DONE_BIT_MASK)

/* All TX events mask */
#define DW3000_SYS_STATUS_ALL_TX                                               \
	(DW3000_SYS_STATUS_AAT_BIT_MASK | DW3000_SYS_STATUS_TXFRB_BIT_MASK |   \
	 DW3000_SYS_STATUS_TXPRS_BIT_MASK | DW3000_SYS_STATUS_TXPHS_BIT_MASK | \
	 DW3000_SYS_STATUS_TXFRS_BIT_MASK)

#define DW3000_RX_BUFFER_MAX_LEN (1023)
#define DW3000_TX_BUFFER_MAX_LEN (1024)
#define DW3000_REG_DIRECT_OFFSET_MAX_LEN (127)
/* Transmit Data Buffer */
#define DW3000_TX_BUFFER_ID 0x140000
/* pointer to access indirect access buffer A */
#define DW3000_INDIRECT_POINTER_A_ID 0x1D0000
/* pointer to access indirect access buffer B */
#define DW3000_INDIRECT_POINTER_B_ID 0x1E0000
/* Receive Data Buffer (in double buffer set) */
#define DW3000_RX_BUFFER_A_ID 0x120000
/* Receive Data Buffer (in double buffer set) */
#define DW3000_RX_BUFFER_B_ID 0x130000
#define DW3000_PHRMODE_STD 0x0 /* standard PHR mode */
#define DW3000_PHRMODE_EXT 0x1 /* DW proprietary extended frames PHR mode */
#define DW3000_PHRRATE_STD 0x0 /* standard PHR rate */
#define DW3000_PHRRATE_DTA 0x1 /* PHR at data rate (6M81) */
#define DW3000_CONFIG \
	0x0001 /* download the AON array into the HIF
				(configuration download) */

/* Enable/disable TX fine grain power sequencing */
#define DW3000_PMSC_TXFINESEQ_ENABLE 0x4d28874
#define DW3000_PMSC_TXFINESEQ_DISABLE 0x0d20874
/* TXRX switch control register's configurations */
#define DW3000_TXRXSWITCH_TX 0x01011100
#define DW3000_TXRXSWITCH_AUTO 0x1C000000

#define DW3000_RF_TXCTRL_CH5 0x1C071134UL
#define DW3000_RF_TXCTRL_CH9 0x1C010034UL
#define DW3000_RF_TXCTRL_LO_B2 0x0E
#define DW3000_RF_PLL_CFG_CH5 0x1F3C
#define DW3000_RF_PLL_CFG_CH9 0x0F3C
#define DW3000_RF_PLL_CFG_LD 0x81
#define DW3000_LDO_RLOAD_VAL_B1 0x14

/* PD threshold for no data STS mode */
#define DW3000_PD_THRESH_NO_DATA 0xAF5F35CC
#define DW3000_PD_THRESH_DEFAULT 0xAF5F584C

#define DW3000_NUM_DW_DEV (1)

#define DW3000_SPI_FAC (0 << 6 | 1 << 0)
#define DW3000_SPI_FARW (0 << 6 | 0 << 0)
#define DW3000_SPI_EAMRW (1 << 6)

/* Power management control's SYSCLK field */
#define DW3000_FORCE_SYSCLK_FOSCDIV4 (1)
#define DW3000_FORCE_SYSCLK_PLL (2)
#define DW3000_FORCE_SYSCLK_FOSC (3)
/* RX and TX CLK field */
#define DW3000_FORCE_CLK_PLL (2)

/* Defines for dw3000_force_clocks() function */
#define DW3000_FORCE_CLK_SYS_TX (1)
#define DW3000_FORCE_CLK_AUTO (5)

/* Bit fields to select information to retrieve from OTP memory */
#define DW3000_READ_OTP_PID 0x10 /* read part ID from OTP */
#define DW3000_READ_OTP_LID 0x20 /* read lot ID from OTP */
#define DW3000_READ_OTP_BAT 0x40 /* read ref voltage from OTP */
#define DW3000_READ_OTP_TMP 0x80 /* read ref temperature from OTP */

#define DW3000_STD_FRAME_LEN (127)
#define DW3000_EXT_FRAME_LEN (1023)

/* CIA lower bound threshold values for 64 MHz PRF */
#define DW3000_CIA_MANUALLOWERBOUND_TH_64 (0x10)

/**
 * DW3000_STSQUAL_THRESH_64() - get STS quality thereshold value for 64 MHz PRF
 * @x: STS length in unit of 8-symbols block
 *
 * When using 64 MHz PRF the stsCpQual should be > 60 % of STS length
 */
#define DW3000_STSQUAL_THRESH_64(x) (((x)*8) * 6 / 10)
/* Factor of sqrt(2) for calculation */
#define DW3000_SQRT2_FACTOR 181
#define DW3000_SQRT2_SHIFT_VAL 7
#define DW3000_STS_MNTH_SHIFT 11
#define DW3000_STS_MNTH_ROUND_SHIFT 1024
/* The supported STS length options */
#define DW3000_STS_LEN_SUPPORTED 9

static const u16 dw3000_sts_length_factors[DW3000_STS_LEN_SUPPORTED] = {
	0, 0, 1024, 1448, 2048, 2896, 4096, 5793, 8192
};

#define DW3000_STS_ACC_CP_QUAL_SIGNTST 0x0800 /* sign test */
#define DW3000_STS_ACC_CP_QUAL_SIGNEXT \
	0xF000 /* 12 bit to 16 bit sign extension */

/**
 * DW3000_GET_STS_LEN_REG_VALUE() - Convert STS length enum into register value
 * @x: value from enum dw3000_sts_lengths
 *
 * Return: the value to set in CP_CFG0_ID for STS length.
 */
#define DW3000_GET_STS_LEN_REG_VALUE(x) ((u16)((1 << (x)) - 1))

/* Delay in symbol used for auto-ack.
 * The IEEE 802.15.4 standard specifies a 12 symbol +/- 0.5 symbols
 * turnaroud time for ACK transmission. */
#define DW3000_NUMBER_OF_SYMBOL_DELAY_AUTO_ACK (12)

/* The PLL calibration should take less that 400us, typically it is < 100us. */
#define DW3000_MAX_RETRIES_FOR_PLL (20)

/* The empirical delay to lock the PLL after its activation */
#define DW3000_PLL_LOCK_DELAY_US (10)

/* The mean XTAL TRIM value measured on multiple E0 samples.
 * During the initialization the XTAL TRIM value can be read from the OTP and
 * in case it is not present, the default would be used instead. */
#define DW3000_DEFAULT_XTAL_TRIM 0x1f

/* SYS_STATE_LO register errors */
/* TSE is in TX but TX is in IDLE in SYS_STATE_LO register */
#define DW3000_SYS_STATE_TXERR 0xD0000
/* TSE is in IDLE (IDLE_PLL) */
#define DW3000_SYS_STATE_IDLE 0x3

/**
 * enum ciadiag_dbl_options - Enable CIA diagnostic's data log level options.
 * @DW3000_CIA_DIAG_LOG_DBL_OFF:
 *	No CIA diagnostic option
 * @DW3000_CIA_DIAG_LOG_DBL_MIN:
 *	CIA to copy to swinging set a minimal set of diagnostic registers
 *	in Double Buffer mode.
 * @DW3000_CIA_DIAG_LOG_DBL_MID:
 *	CIA to copy to swinging set a medium set of diagnostic registers
 *	in Double Buffer mode.
 * @DW3000_CIA_DIAG_LOG_DBL_MAX:
 *	CIA to copy to swinging set a maximum set of diagnostic registers
 *	in Double Buffer mode.
 */
enum ciadiag_dbl_options {
	DW3000_CIA_DIAG_LOG_DBL_OFF = 0,
	DW3000_CIA_DIAG_LOG_DBL_MIN = BIT(0),
	DW3000_CIA_DIAG_LOG_DBL_MID = BIT(1),
	DW3000_CIA_DIAG_LOG_DBL_MAX = BIT(2)
};

/* Disable CIA diagnostic. CIACONFIG's bit-4 in RX_ANTENNA_DELAY + 1 */
#define DW3000_CIA_CONFIG_DIAG_OFF (0x1 << 4)

struct dw3000_ciadiag_reg_info {
	u32 diag1;
	u32 diag12;
};

/* CIA diagnostic register selection according DW3000's configuration */
static const struct dw3000_ciadiag_reg_info _ciadiag_reg_info[] = {
	/* Without STS */
	{
		.diag1 = DW3000_IP_DIAG1_ID,
		.diag12 = DW3000_IP_DIAG12_ID,
	},
	/* With STS */
	{
		.diag1 = DW3000_CP0_DIAG1_ID,
		.diag12 = DW3000_CP0_DIAG12_ID,
	},
	/* PDOA Mode 3 */
	{
		.diag1 = DW3000_CP1_DIAG1_ID,
		.diag12 = DW3000_CP1_DIAG12_ID,
	},
};

/* sysfs variables handling */
static ssize_t dw3000_sysfs_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf);
static ssize_t dw3000_sysfs_store(struct kobject *kobj,
				  struct kobj_attribute *attr, const char *buf,
				  size_t length);
const struct kobj_attribute dw3000_attribute =
	__ATTR(power_stats, 0664, dw3000_sysfs_show, dw3000_sysfs_store);

/* Interrupt working options */
enum int_options {
	DW3000_DISABLE_INT = 0,
	DW3000_ENABLE_INT,
	DW3000_ENABLE_INT_ONLY
};

/* Indexes are DWT_PLEN_NNNN values - 1 */
const struct dw3000_plen_info _plen_info[] = {
	{
		.symb = 64,
		.pac_symb = 8,
		.dw_reg = DW3000_PLEN_64,
		.dw_pac_reg = DW3000_PAC8,
	},
	{
		.symb = 1024,
		.pac_symb = 32,
		.dw_reg = DW3000_PLEN_1024,
		.dw_pac_reg = DW3000_PAC32,
	},
	{
		.symb = 4096,
		.pac_symb = 64,
		.dw_reg = DW3000_PLEN_4096,
		.dw_pac_reg = DW3000_PAC32,
	},
	{
		.symb = 32,
		.pac_symb = 8,
		.dw_reg = DW3000_PLEN_32,
		.dw_pac_reg = DW3000_PAC8,
	},
	{
		.symb = 128,
		.pac_symb = 8,
		.dw_reg = DW3000_PLEN_128,
		.dw_pac_reg = DW3000_PAC8,
	},
	{
		.symb = 1536,
		.pac_symb = 64,
		.dw_reg = DW3000_PLEN_1536,
		.dw_pac_reg = DW3000_PAC32,
	},
	{
		.symb = 72,
		.pac_symb = 8,
		.dw_reg = DW3000_PLEN_72,
		.dw_pac_reg = DW3000_PAC8,
	},
	{
		/* Invalid */
		.symb = 0,
		.pac_symb = 0,
		.dw_reg = 0,
		.dw_pac_reg = 0,
	},
	{
		.symb = 256,
		.pac_symb = 16,
		.dw_reg = DW3000_PLEN_256,
		.dw_pac_reg = DW3000_PAC16,
	},
	{
		.symb = 2048,
		.pac_symb = 64,
		.dw_reg = DW3000_PLEN_2048,
		.dw_pac_reg = DW3000_PAC32,
	},
	{
		/* Invalid */
		.symb = 0,
		.pac_symb = 0,
		.dw_reg = 0,
		.dw_pac_reg = 0,
	},
	{
		/* Invalid */
		.symb = 0,
		.pac_symb = 0,
		.dw_reg = 0,
		.dw_pac_reg = 0,
	},
	{
		.symb = 512,
		.pac_symb = 16,
		.dw_reg = DW3000_PLEN_512,
		.dw_pac_reg = DW3000_PAC16,
	},
};

const struct dw3000_bitrate_info _bitrate_info[] = {
	{
		/*  850k */
		.phr_chip_per_symb = 512,
		.data_chip_per_symb = 512,
	},
	{
		/* 6M8 */
		.phr_chip_per_symb = 512,
		.data_chip_per_symb = 64,
	},
};

const struct dw3000_prf_info _prf_info[] = {
	{
		/* Invalid PRF */
		0,
	},
	{
		/* 16 MHz */
		.chip_per_symb = 496,
	},
	{
		/* 64 MHz */
		.chip_per_symb = 508,
	},
};

static int dw3000_transfers_reset(struct dw3000 *dw);
static int dw3000_change_speed(struct dw3000 *dw, u32 new_speed);
static int dw3000_wakeup(struct dw3000 *dw);
static inline bool _dw3000_sts_is_enabled(struct dw3000 *dw);
static u8 dw3000_calc_bandwithadj(struct dw3000 *dw, int target_count);
static u32 dw3000_calc_pgcount(struct dw3000 *dw, u32 pg_delay);


void dw3000_enable_counters(struct dw3000 *dw)
{
	dw3000_reg_write32(dw, 0xf0000, 0, 1);
}

void dw3000_reset_counters(struct dw3000 *dw)
{
	dw3000_reg_write32(dw, 0xf0000, 0, 2);
}

void dw3000_read_counters(struct dw3000 *dw)
{
	uint16_t buffer[16];

	dw3000_reg_read_fast(dw, 0xf0004, 0, sizeof(buffer), buffer);

	trace_dw3000_get_counters_first_part(dw,
			buffer[0],
			buffer[1],
			buffer[2],
			buffer[5],
			buffer[6],
			buffer[7],
			buffer[8],
			buffer[9],
			buffer[10],
			buffer[11]);
	trace_dw3000_get_counters_second_part(dw,
			buffer[3],
			buffer[14],
			buffer[15]);
}

/**
 * dw3000_get_dtu_time() - compute current DTU time
 * @dw: the DW device
 *
 * This function compute the current DTU time, based on ktime,
 * at 15.6MHz rate.
 *
 * Return: the current DTU time
 */
u32 dw3000_get_dtu_time(struct dw3000 *dw)
{
	s64 bt_ns = ktime_get_boottime_ns();
	return dw3000_ktime_to_dtu(dw, bt_ns);
}

/**
 * dw3000_resync_dtu_sys_time() - resync DTU time and SYS_TIME
 * @dw: the DW device
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_resync_dtu_sys_time(struct dw3000 *dw)
{
	int rc;
	/* Read SYS_TIME */
	rc = dw3000_read_sys_time(dw, &dw->sys_time_sync);
	if (rc)
		dw->sys_time_sync = 1000; /* TODO: check value */
	/* Save synchronisation time DTU */
	dw->dtu_sync = dw3000_get_dtu_time(dw);
	trace_dw3000_resync_dtu_sys_time(dw, dw->sys_time_sync, dw->dtu_sync);
	/* Invalidate RCTU synchronisation */
	dw3000_resync_rctu_conv_state(dw);
	return rc;
}

/**
 * dw3000_may_resync() - check if a resync is needed, if yes, do it
 * @dw: the DW device
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_may_resync(struct dw3000 *dw)
{
	int rc = 0;
	u32 now_dtu;

	now_dtu = dw3000_get_dtu_time(dw);
	if (now_dtu - dw->dtu_sync > DW3000_DTU_FREQ)
		rc = dw3000_resync_dtu_sys_time(dw);
	return rc;
}

/**
 * dw3000_power_stats() - compute time elapsed in dw3000 states
 * @dw: the DW device on which state is changed
 * @state: the new state
 * @len_or_date: frame length to be transmitted or RX dates in DTU
 *
 * Update power statistics according current state leaved and new state.
 * States > RUN are handled specifically.
 */
static inline void dw3000_power_stats(struct dw3000 *dw, int state,
				      int len_or_date)
{
	struct dw3000_power *pw = &dw->power;
	int cstate = min(pw->cur_state, DW3000_PWR_RUN);
	int nstate = min(state, DW3000_PWR_RUN);
	u64 boot_time_ns = ktime_get_boottime_ns();
	s64 duration;
	s64 adjust;
	u32 cur_dtu_time;

	/* Trace call */
	trace_dw3000_power_stats(dw, state, boot_time_ns, len_or_date);
	/* Sanity checks first */
	if (state < 0 || state >= DW3000_PWR_MAX)
		return;
	/* Calculate duration of current state. */
	duration = boot_time_ns - pw->start_time;
	/* Update basic state statistics */
	pw->stats[cstate].dur += duration;
	if (nstate != cstate)
		pw->stats[nstate].count++;
	/* Handle specific states */
	cstate = pw->cur_state;
	switch (state) {
	case DW3000_PWR_IDLE:
		adjust = 0;
		if (cstate == DW3000_PWR_TX) {
			/* TX duration is just pw->tx_adjust */
			adjust = pw->tx_adjust;
		} else if (cstate == DW3000_PWR_RX) {
			/* RX duration is just cur_dtu_time-pw->trx_adjust,
			 * remain is IDLE time, so remove duration previously added */
			cur_dtu_time = (u32)len_or_date;
			if (!cur_dtu_time) {
				/* RX frame end time is not given, so get current DTU. */
				cur_dtu_time = dw3000_get_dtu_time(dw);
			}
			if (cur_dtu_time < pw->rx_start)
				/* Handle chip time overflow.
				 * Through 'll' type avoid computation problem. */
				adjust = (1ll << 32) + cur_dtu_time -
					 pw->rx_start;
			else
				adjust = cur_dtu_time - pw->rx_start;
		}
		pw->stats[cstate].dur += (u32)adjust;
		if (state != cstate)
			pw->stats[DW3000_PWR_IDLE].count++;
		break;
	case DW3000_PWR_TX:
		/* TX time is calculated according frame len only */
		pw->tx_adjust =
			dw3000_frame_duration_dtu(dw, len_or_date, true);
		pw->stats[DW3000_PWR_TX].count++;
		break;
	case DW3000_PWR_RX:
		/* RX time is calculated using start time and reception time */
		cur_dtu_time = (u32)len_or_date;
		if (!cur_dtu_time) {
			/* Start time is unknown for immediate RX but we need
			   it, so get current DTU time. */
			cur_dtu_time = dw3000_get_dtu_time(dw);
		}
		pw->rx_start = cur_dtu_time;
		pw->stats[DW3000_PWR_RX].count++;
		break;
	case DW3000_PWR_RUN:
		/* Entering RUN state also enter IDLE state */
		if (state != cstate)
			pw->stats[DW3000_PWR_IDLE].count++;
		break;
	default:
		break;
	}
	/* Update current information */
	pw->start_time = boot_time_ns;
	pw->cur_state = state;
}

/**
 * dw3000_alloc_xfer() - Allocate a new spi_message including spi_tranfer.
 * @trcount: the number of transfer to allocate with the new spi_message
 * @len: the length of TX buffer to allocate for the first transfer
 *
 * Return: the spi_message struct or NULL if error.
 */
static inline struct spi_message *dw3000_alloc_xfer(unsigned int trcount,
						    unsigned int len)
{
	struct spi_message *msg = spi_message_alloc(trcount, GFP_KERNEL);

	if (msg && len) {
		struct spi_transfer *transfer = list_first_entry(
			&msg->transfers, struct spi_transfer, transfer_list);
		transfer->tx_buf = kzalloc(len, GFP_KERNEL | GFP_DMA);
		if (!transfer->tx_buf) {
			/* failure to allocate requested buffer for header */
			spi_message_free(msg);
			msg = NULL;
		} else {
			transfer->len = len;
		}
	}
	return msg;
}

/**
 * dw3000_free_xfer() - Free an spi_message allocated by @dw3000_alloc_xfer
 * @msg: the SPI message to free
 * @len: the same length of TX buffer to automatically free it
 */
static inline void dw3000_free_xfer(struct spi_message *msg, unsigned int len)
{
	if (!msg)
		return;
	if (len) {
		struct spi_transfer *tr = list_first_entry(
			&msg->transfers, struct spi_transfer, transfer_list);
		if (tr->rx_buf && tr->rx_buf != tr->tx_buf)
			kfree(tr->rx_buf);
		kfree(tr->tx_buf);
	}
	spi_message_free(msg);
}

/**
 * dw3000_prepare_xfer() - Initialise an spi_message allocated by @dw3000_alloc_xfer
 * @msg: the SPI message to initialise
 * @reg_fileid: the register fileID to read/write
 * @index: the index where to read/write
 * @length: the length of provided buffer and SPI data transfer
 * @buffer: the address where to read/write data
 * @mode: operation mode, ie. RW, WR, AND_OR8, etc.
 *
 * Return: zero on success, else a negative error code.
 */
static inline int dw3000_prepare_xfer(struct spi_message *msg, u32 reg_fileid,
				      u16 index, u16 length, void *buffer,
				      enum spi_modes mode)
{
	struct spi_transfer *tr = list_first_entry(
		&msg->transfers, struct spi_transfer, transfer_list);
	u8 *header_buf = (u8 *)tr->tx_buf;
	u16 header_len;

	/* Extract register file and sub-address (+ offset) */
	u16 reg_file = 0x1F & ((reg_fileid + index) >> 16);
	u16 reg_offset = 0x7F & (reg_fileid + index);

	/* Fast command not supported by this function */
	if (unlikely(length == 0 && mode == DW3000_SPI_WR_BIT))
		return -EINVAL;

	/* Set header buffer */
	if (reg_offset || (mode & DW3000_SPI_AND_OR_MSK)) {
		/* 2-byte header */
		u16 param = (reg_file << 9) | (reg_offset << 2) | mode;

		header_buf[0] = (u8)(param >> 8) | DW3000_SPI_EAMRW;
		header_buf[1] = (u8)param;
		header_len = 2;
	} else {
		/* 1-byte header */
		u8 param = reg_file << 1 | mode >> 8;

		header_buf[0] = (u8)param | DW3000_SPI_FARW;
		header_len = 1;
	}
	/* Adjust header len in the SPI message */
	if (unlikely(header_len > tr->len))
		return -EINVAL;
	tr->len = header_len;

	/* Set the data buffer in second transfer */
	if (likely(!buffer)) {
		/* Single spi_tranfer messages are used for full-fuplex register
		   read/write. Just update the transfer length.
		   The rx_buf is already set dw3000_alloc_prepare_xfer(). */
		tr->len += length;
	} else {
		struct spi_transfer *tr2;

		tr2 = list_next_entry(tr, transfer_list);
		if (mode == DW3000_SPI_RD_BIT) {
			/* RX transaction */
			tr2->rx_buf = buffer;
		} else {
			/* TX transaction */
			tr2->tx_buf = buffer;
		}
		/* Add data to the SPI message */
		tr2->len = length;
	}
	return 0;
}

/**
 * dw3000_alloc_prepare_xfer() - Allocate and prepare an spi_message
 * @dw: the DW device on which the SPI transfer will occurs
 * @reg_fileid: the register fileID to read/write
 * @index: the index where to read/write
 * @length: expected length of data to read/write
 * @mode: operation mode, ie. RW, WR, AND_OR8, etc.
 *
 * The prepared spi_message will have only one spi_transfer with enough space
 * for read/write 16 bytes at least in full-duplex mode (including header).
 *
 * Return: the spi_message struct or NULL if error.
 */
static struct spi_message *dw3000_alloc_prepare_xfer(struct dw3000 *dw,
						     u32 reg_fileid, u16 index,
						     u16 length,
						     enum spi_modes mode)
{
	struct spi_message *msg;
	int len = length < 16 ? 16 : length;
	int rc;

	/* Allocate a new SPI message with one transfer. */
	msg = dw3000_alloc_xfer(1, len);
	if (!msg)
		goto err_alloc;
	/* Prepare this message for given register access */
	rc = dw3000_prepare_xfer(msg, reg_fileid, index, length, NULL, mode);
	if (rc)
		goto err_prepare;
	/* Need to have a separated TX/RX buffer because initialised TX
	   buffer will be clobbered during first exchange if RX buffer
	   is the same. rx_buf = tx_buf is only right if message is used
	   once, or is re-initialised before each transfer like in
	   dw3000_reg_read_fast(). */
	if (mode == DW3000_SPI_RD_BIT) {
		struct spi_transfer *tr = list_first_entry(
			&msg->transfers, struct spi_transfer, transfer_list);
		tr->rx_buf = kzalloc(len, GFP_KERNEL | GFP_DMA);
		if (!tr->rx_buf)
			goto err_rxalloc;
	}
	return msg;

err_rxalloc:
err_prepare:
	dw3000_free_xfer(msg, len);
err_alloc:
	dev_err(dw->dev,
		"Failure to allocate message for reg 0x%x (index %u, len %d, mode %d)\n",
		reg_fileid, index, length, mode);
	return NULL;
}

/**
 * dw3000_alloc_prepare_fastcmd() - Allocate and prepare a fastcmd spi_message
 *
 * Return: the spi_message struct or NULL if error.
 */
static struct spi_message *dw3000_alloc_prepare_fastcmd(void)
{
	/* Allocate a new SPI message with only one transfer. */
	return dw3000_alloc_xfer(1, 1);
}

/**
 * dw3000_free_fastcmd() - Free a fastcmd spi_message
 * @msg: the SPI message to free
 */
static void dw3000_free_fastcmd(struct spi_message *msg)
{
	/* Free SPI message and tx_buf of included transfer */
	dw3000_free_xfer(msg, 1);
}

/**
 * dw3000_spi_sync() - Call spi_sync with error message
 * @dw: the DW device on which the SPI transfer will occurs
 * @msg: the SPI message to sync
 *
 * Return: 0 on success, else a negative error code.
 */
static inline int dw3000_spi_sync(const struct dw3000 *dw,
				  struct spi_message *msg)
{
	int rc;
	rc = spi_sync(dw->spi, msg);
	if (rc)
		dev_err(dw->dev, "could not transfer : %d\n", rc);
	return rc;
}

/**
 * dw3000_xfer() - Generic low-level slow transfer
 * @dw: the DW device on which the SPI transfer will occurs
 * @reg_fileid: the fileID to read/write
 * @reg_offset: the offset where to read/write
 * @length: the length of provided buffer and SPI data transfer
 * @buffer: the address where to read/write data
 * @mode: operation mode, ie. RW, WR, AND_OR8, etc.
 *
 * This function initialise a new SPI message and two SPI transfer on the stack.
 * First transfer is use for TX header to device. Second transfer is used for RX
 * or TX data. Then spi_sync() is called.
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_xfer(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset, u16 length,
		void *buffer, enum spi_modes mode)
{
	struct {
		struct spi_message msg;
		struct spi_transfer header;
		struct spi_transfer data;
		u8 header_buf[2];
	} xfer;

	/* Init transfers first because spi_message_init_with_transfer don't! */
	memset(&xfer.header, 0, sizeof(xfer.header) * 2);
	xfer.header.tx_buf = xfer.header_buf;
	xfer.header.len = sizeof(xfer.header_buf);

	/* Then construct SPI message */
	spi_message_init_with_transfers(&xfer.msg, &xfer.header, 2);

	/* Prepare header & data transfers */
	dw3000_prepare_xfer(&xfer.msg, reg_fileid, reg_offset, length, buffer,
			    mode);
	/* Now execute this spi message synchronously */
	return dw3000_spi_sync(dw, &xfer.msg);
}

/**
 * dw3000_write_fastcmd() - Send a fast command to the device
 * @dw: the DW device on which the SPI transfer will occurs
 * @cmd: the fastcommand to send to the device
 *
 * This function use a prebuilt SPI message with a single transfer which is
 * modified to send the required command.
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_write_fastcmd(struct dw3000 *dw, u8 cmd)
{
	/* Use a prebuilt SPI message to be as fast as possible. */
	struct spi_message *msg = dw->msg_fast_command;
	struct spi_transfer *tr = list_first_entry(
		&msg->transfers, struct spi_transfer, transfer_list);
	u8 *header_buf = (u8 *)tr->tx_buf;

	/* Set the command to send directly in first transfer TX buffer */
	*header_buf =
		(u8)((DW3000_SPI_WR_BIT >> 8) | (cmd << 1) | DW3000_SPI_FAC);
	/* Now execute this spi message synchronously */
	return dw3000_spi_sync(dw, msg);
}

/**
 * dw3000_reg_read_fast() - Generic full-duplex register read
 * @dw: the DW device on which the SPI transfer will occurs
 * @reg_fileid: the register fileID to read
 * @reg_offset: the register offset to read
 * @length: the length of provided buffer and SPI data transfer
 * @buffer: the address where to store the read data
 *
 * This function read the specified register using a dedicated full-duplex
 * message. It configure it first, call spi_sync() and then copy read data
 * from the spi_tranfer RX buffer to the provided buffer.
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_reg_read_fast(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
			 u16 length, void *buffer)
{
	struct spi_message *msg = dw->msg_readwrite_fdx;
	struct spi_transfer *tr = list_first_entry(
		&msg->transfers, struct spi_transfer, transfer_list);
	int hlen;
	int rc;

	mutex_lock(&dw->msg_mutex);
	/* Update header and length */
	dw3000_prepare_xfer(msg, reg_fileid, reg_offset, length, NULL,
			    DW3000_SPI_RD_BIT);
	/* Retrieve header length */
	hlen = tr->len - length;
	/* Ensure all data bits are 0 */
	memset((void *)tr->tx_buf + hlen, 0, length);
	/* Execute SPI transfer */
	rc = dw3000_spi_sync(dw, msg);
	if (!rc)
		/* Get back the data that are after the header in RX buffer */
		memcpy(buffer, tr->rx_buf + hlen, length);
	mutex_unlock(&dw->msg_mutex);
	return rc;
}

/**
 * dw3000_reg_read32() - 32 bits register read
 * @dw: the DW device on which the SPI transfer will occurs
 * @reg_fileid: the register fileID to read
 * @reg_offset: the register offset to read
 * @val: the u32 value's pointer
 *
 * Just a little wrapper to the dw3000_reg_read_fast() function.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_reg_read32(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
		      u32 *val)
{
	__le32 buffer;
	int rc;
	/* Read 4 bytes (32-bits) register into buffer */
	rc = dw3000_reg_read_fast(dw, reg_fileid, reg_offset, sizeof(buffer),
				  &buffer);
	if (likely(!rc))
		*val = le32_to_cpu(buffer);
	return rc;
}

/**
 * dw3000_reg_read16() - 16 bits register read
 * @dw: the DW device on which the SPI transfer will occurs
 * @reg_fileid: the register fileID to read
 * @reg_offset: the register offset to read
 * @val: the u16 value's pointer
 *
 * Just a little wrapper to the dw3000_reg_read_fast() function.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_reg_read16(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
		      u16 *val)
{
	__le16 buffer;
	int rc;
	/* Read 2 bytes (16-bits) register into buffer */
	rc = dw3000_reg_read_fast(dw, reg_fileid, reg_offset, sizeof(buffer),
				  &buffer);
	if (likely(!rc))
		*val = le16_to_cpu(buffer);
	return rc;
}

/**
 * dw3000_reg_read8() - 8 bits register read
 * @dw: the DW device on which the SPI transfer will occurs
 * @reg_fileid: the register fileID to read
 * @reg_offset: the register offset to read
 * @val: the u8 value's pointer
 *
 * Just a little wrapper to the dw3000_reg_read_fast() function.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_reg_read8(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset, u8 *val)
{
	/* Read 1 byte (8-bits) register into data */
	return dw3000_reg_read_fast(dw, reg_fileid, reg_offset, sizeof(*val),
				    val);
}

/**
 * dw3000_reg_write_fast() - Generic single-transfer register write
 * @dw: the DW device on which the SPI transfer will occurs
 * @reg_fileid: the register fileID to read
 * @reg_offset: the register offset to read
 * @length: the length of provided buffer and SPI data transfer
 * @buffer: the address where to store the read data
 * @mode: the write operation mode to use (direct, bitmask and/or)
 *
 * This function write the specified register using a dedicated single-transfer
 * message. It configure it first, copy data from provided buffer in it, then
 * call spi_sync().
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_reg_write_fast(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
			  u16 length, const void *buffer, enum spi_modes mode)
{
	struct spi_message *msg = dw->msg_readwrite_fdx;
	struct spi_transfer *tr = list_first_entry(
		&msg->transfers, struct spi_transfer, transfer_list);
	void *rx_buf;
	int hlen;
	int rc;

	mutex_lock(&dw->msg_mutex);
	/* Update header and length */
	dw3000_prepare_xfer(msg, reg_fileid, reg_offset, length, NULL, mode);
	/* Data are after the header in TX buffer */
	hlen = tr->len - length;
	memcpy((void *)tr->tx_buf + hlen, buffer, length);
	/* We don't want to receive data, so remove unused RX buffer to avoid
	   unrequired fifo read in controller. */
	rx_buf = tr->rx_buf; /* save RX buffer */
	tr->rx_buf = NULL;
	/* Execute SPI transfer */
	rc = dw3000_spi_sync(dw, msg);
	/* Restore RX buffer */
	tr->rx_buf = rx_buf;
	mutex_unlock(&dw->msg_mutex);
	return rc;
}

/**
 * _dw3000_reg_write - Generic register write
 * @dw: the DW device on which the SPI transfer will occurs
 * @reg_fileid: the register fileID to write
 * @reg_offset: the register offset to write
 * @length: the length of provided buffer and SPI data transfer
 * @buffer: the address of the data to write
 *
 * Just a little wrapper to the dw3000_reg_write_fast() function.
 *
 * Return: zero on success, else a negative error code.
 */
static inline int _dw3000_reg_write(struct dw3000 *dw, u32 reg_fileid,
				    u16 reg_offset, u16 length,
				    const void *buffer)
{
	return dw3000_reg_write_fast(dw, reg_fileid, reg_offset, length, buffer,
				     DW3000_SPI_WR_BIT);
}

/**
 * dw3000_reg_write32() - 32 bits register write
 * @dw: the DW device on which the SPI transfer will occurs
 * @reg_fileid: the register fileID to write
 * @reg_offset: the register offset to write
 * @data: value to write to the register
 *
 * Just a little wrapper to the dw3000_reg_write() function.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_reg_write32(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
		       u32 data)
{
	__le32 buffer = cpu_to_le32(data);

	return _dw3000_reg_write(dw, reg_fileid, reg_offset, sizeof(buffer),
				 &buffer);
}

/**
 * dw3000_reg_write16() - 16 bits register write
 * @dw: the DW device on which the SPI transfer will occurs
 * @reg_fileid: the register fileID to write
 * @reg_offset: the register offset to write
 * @data: value to write to the register
 *
 * Just a little wrapper to the dw3000_reg_write() function.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_reg_write16(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
		       u16 data)
{
	__le16 buffer = cpu_to_le16(data);

	return _dw3000_reg_write(dw, reg_fileid, reg_offset, sizeof(buffer),
				 &buffer);
}

/**
 * dw3000_reg_write8() - 8 bits register write
 * @dw: the DW device on which the SPI transfer will occurs
 * @reg_fileid: the register fileID to write
 * @reg_offset: the register offset to write
 * @data: value to write to the register
 *
 * Just a little wrapper to the dw3000_reg_write() function.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_reg_write8(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
		      u8 data)
{
	return _dw3000_reg_write(dw, reg_fileid, reg_offset, sizeof(data),
				 &data);
}

/**
 * dw3000_reg_modify32() - 32 bits register modify
 * @dw: the DW device on which the SPI transfer will occurs
 * @reg_fileid: the register fileID to modify
 * @reg_offset: the register offset to modify
 * @_and: bitmask to clear
 * @_or: bitmask to set
 *
 * Just a little wrapper to the dw3000_reg_write_fast() function.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_reg_modify32(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
			u32 _and, u32 _or)
{
	__le32 buffer[2];

	buffer[0] = cpu_to_le32(_and);
	buffer[1] = cpu_to_le32(_or);
	return dw3000_reg_write_fast(dw, reg_fileid, reg_offset, sizeof(buffer),
				     buffer, DW3000_SPI_AND_OR_32);
}

/**
 * dw3000_reg_modify16() - 16 bits register modify
 * @dw: the DW device on which the SPI transfer will occurs
 * @reg_fileid: the register fileID to modify
 * @reg_offset: the register offset to modify
 * @_and: bitmask to clear
 * @_or: bitmask to set
 *
 * Just a little wrapper to the dw3000_reg_write_fast() function.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_reg_modify16(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
			u16 _and, u16 _or)
{
	__le16 buffer[2];

	buffer[0] = cpu_to_le16(_and);
	buffer[1] = cpu_to_le16(_or);
	return dw3000_reg_write_fast(dw, reg_fileid, reg_offset, sizeof(buffer),
				     buffer, DW3000_SPI_AND_OR_16);
}

/**
 * dw3000_reg_modify8() - 8 bits register modify
 * @dw: the DW device on which the SPI transfer will occurs
 * @reg_fileid: the register fileID to modify
 * @reg_offset: the register offset to modify
 * @_and: bitmask to clear
 * @_or: bitmask to set
 *
 * Just a little wrapper to the dw3000_reg_write_fast() function.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_reg_modify8(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
		       u8 _and, u8 _or)
{
	u8 buffer[2];

	buffer[0] = _and;
	buffer[1] = _or;
	return dw3000_reg_write_fast(dw, reg_fileid, reg_offset, sizeof(buffer),
				     buffer, DW3000_SPI_AND_OR_8);
}

/**
 * dw3000_clear_sys_status() - Fast clearing of SYS_STATUS register
 * @dw: the DW device on which the SPI transfer will occurs
 * @clear_bits: the bitmask of bits to clear
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_clear_sys_status(struct dw3000 *dw, u32 clear_bits)
{
	/* Use a prebuilt SPI message to be as fast as possible. */
	struct spi_message *msg = dw->msg_write_sys_status;
	struct spi_transfer *tr = list_first_entry(
		&msg->transfers, struct spi_transfer, transfer_list);
	const int hlen = tr->len - sizeof(clear_bits);
	/* Prepared message have only header & length set, need to set data part */
	put_unaligned_le32(clear_bits, (void *)tr->tx_buf + hlen);
	return dw3000_spi_sync(dw, msg);
}

static int dw3000_clear_all_sys_status(struct dw3000 *dw, u64 clear_bits)
{
	/* Use a prebuilt SPI message to be as fast as possible. */
	struct spi_message *msg = dw->msg_write_all_sys_status;
	struct spi_transfer *tr = list_first_entry(
		&msg->transfers, struct spi_transfer, transfer_list);
	const int hlen = tr->len - sizeof(clear_bits);
	/* Prepared message have only header & length set, need to set data part */
	put_unaligned_le64(clear_bits, (void *)tr->tx_buf + hlen);
	return dw3000_spi_sync(dw, msg);
}

/**
 * dw3000_read_sys_status() - Fast read of SYS_STATUS register (4 low bytes upon 6)
 * @dw: the DW device on which the SPI transfer will occurs
 * @status: address where to put read status
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_read_sys_status(struct dw3000 *dw, u32 *status)
{
	/* Use a prebuilt SPI message to be as fast as possible. */
	struct spi_message *msg = dw->msg_read_sys_status;
	struct spi_transfer *tr = list_first_entry(
		&msg->transfers, struct spi_transfer, transfer_list);
	const int hlen = tr->len - sizeof(*status);
	int rc = dw3000_spi_sync(dw, msg);
	if (!rc)
		*status = get_unaligned_le32(tr->rx_buf + hlen);
	return rc;
}

/**
 * dw3000_clear_dss_status() - Fast clearing of DSS_STAT register
 * @dw: the DW device on which the SPI transfer will occurs
 * @clear_bits: the bitmask of bits to clear
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_clear_dss_status(struct dw3000 *dw, u8 clear_bits)
{
	/* Use a prebuilt SPI message to be as fast as possible. */
	struct spi_message *msg = dw->msg_write_dss_status;
	struct spi_transfer *tr = list_first_entry(
		&msg->transfers, struct spi_transfer, transfer_list);
	const int hlen = tr->len - sizeof(clear_bits);
	int rc;
	/* Prepared message have only header & length set, need to set data part */
	*((u8 *)tr->tx_buf + hlen) = clear_bits;
	rc = spi_sync(dw->spi, msg);
	if (rc)
		dev_err(dw->dev, "could not transfer : %d\n", rc);
	return rc;
}

/**
 * dw3000_read_dss_status() - Fast read of DSS_STAT register
 * @dw: the DW device on which the SPI transfer will occurs
 * @status: address where to put read status
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_read_dss_status(struct dw3000 *dw, u8 *status)
{
	/* Use a prebuilt SPI message to be as fast as possible. */
	struct spi_message *msg = dw->msg_read_dss_status;
	struct spi_transfer *tr = list_last_entry(
		&msg->transfers, struct spi_transfer, transfer_list);
	const int hlen = tr->len - sizeof(*status);
	int rc = spi_sync(dw->spi, msg);

	if (rc)
		dev_err(dw->dev, "could not transfer : %d\n", rc);
	else
		*status = *((u8 *)tr->rx_buf + hlen);
	return rc;
}

/**
 * dw3000_clear_spi_collision_status() - Fast clearing of SPI_COLLISION_STATUS register
 * @dw: the DW device on which the SPI transfer will occurs
 * @clear_bits: the bitmask of bits to clear
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_clear_spi_collision_status(struct dw3000 *dw, u8 clear_bits)
{
	/* Use a prebuilt SPI message to be as fast as possible. */
	struct spi_message *msg = dw->msg_write_spi_collision_status;
	struct spi_transfer *tr = list_first_entry(
		&msg->transfers, struct spi_transfer, transfer_list);
	const int hlen = tr->len - sizeof(clear_bits);
	int rc;
	/* Prepared message have only header & length set, need to set data part */
	*((u8 *)tr->tx_buf + hlen) = clear_bits;
	rc = spi_sync(dw->spi, msg);
	if (rc)
		dev_err(dw->dev, "could not transfer : %d\n", rc);
	return rc;
}

/*
 * dw3000_read_all_sys_status() - Fast read of SYS_STATUS register
 * @dw: the DW device on which the SPI transfer will occurs
 * @status: address where to put read status
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_read_all_sys_status(struct dw3000 *dw, u64 *status)
{
	/* Use a prebuilt SPI message to be as fast as possible. */
	struct spi_message *msg = dw->msg_read_all_sys_status;
	struct spi_transfer *tr = list_first_entry(
		&msg->transfers, struct spi_transfer, transfer_list);
	const int hlen = tr->len - sizeof(*status);
	int rc = dw3000_spi_sync(dw, msg);
	if (!rc)
		*status = get_unaligned_le64(tr->rx_buf + hlen);
	return rc;
}

/**
 * dw3000_read_rdb_status() - Fast read of RDB_STATUS register
 * @dw: the DW device on which the SPI transfer will occurs
 * @status: address where to put read status
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_read_rdb_status(struct dw3000 *dw, u8 *status)
{
	/* Use a prebuilt SPI message to be as fast as possible. */
	struct spi_message *msg = dw->msg_read_rdb_status;
	struct spi_transfer *tr = list_last_entry(
		&msg->transfers, struct spi_transfer, transfer_list);
	const int hlen = tr->len - sizeof(*status);
	int rc = dw3000_spi_sync(dw, msg);
	if (!rc)
		*status = *((u8 *)tr->rx_buf + hlen);
	return rc;
}

/**
 * dw3000_check_devid() - Read and check the DEVID register
 * @dw: the DW device on which the SPI transfer will occurs
 *
 * Return: 0 on success, else -ENODEV error code.
 */
int dw3000_check_devid(struct dw3000 *dw)
{
	u32 devid;
	int i;

	int rc = dw3000_reg_read32(dw, DW3000_DEV_ID_ID, 0, &devid);
	if (unlikely(rc))
		return rc;
	for (i = 0; i < ARRAY_SIZE(dw3000_chip_versions); i++) {
		if (devid == dw3000_chip_versions[i].id) {
			if (!dw->chip_dev_id) {
				dev_warn(dw->dev, "chip version found : %x\n",
					 devid);
				dw->chip_dev_id = devid;
				dw->chip_idx = i;
			}
			__dw3000_chip_version = dw3000_chip_versions[i].ver;
			dw->chip_ops = dw3000_chip_versions[i].ops;
			return 0;
		}
	}
	dev_warn(dw->dev, "unknown DEV_ID : %x\n", devid);
	return -ENODEV;
}

/**
 * dw3000_check_idlerc() - Read and check the RCINIT bit in SYS_STATUS register
 * @dw: the DW device on which the SPI transfer will occurs
 *
 * Return: 1 if bit is set else 0 if unset or SPI error.
 */
static inline u8 dw3000_check_idlerc(struct dw3000 *dw)
{
	u32 reg;

	if (dw3000_read_sys_status(dw, &reg))
		return 0;
	dw3000_reset_counters(dw);
	dw3000_enable_counters(dw);
	dev_notice(dw->dev, "sys_status : 0x%x\n", reg);

	return ((reg & (DW3000_SYS_STATUS_RCINIT_BIT_MASK)) ==
		(DW3000_SYS_STATUS_RCINIT_BIT_MASK));
}

/**
 * dw3000_read_sys_time() - Read current system time
 * @dw: the DW device on which the SPI transfer will occurs
 * @sys_time: the address where to store current time
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_read_sys_time(struct dw3000 *dw, u32 *sys_time)
{
	/* Use a prebuilt SPI message to be as fast as possible. */
	struct spi_message *msg = dw->msg_read_sys_time;
	struct spi_transfer *tr = list_first_entry(
		&msg->transfers, struct spi_transfer, transfer_list);
	const int hlen = tr->len - sizeof(*sys_time);
	int rc;

	if (dw->chip_ops->pre_read_sys_time) {
		rc = dw->chip_ops->pre_read_sys_time(dw);
		if (rc)
			return rc;
	}
	rc = dw3000_spi_sync(dw, msg);
	if (!rc)
		*sys_time = get_unaligned_le32(tr->rx_buf + hlen);
	return rc;
}

/**
 * dw3000_read_rx_timestamp() - Read precise RX timestamp
 * @dw: the DW device on which the SPI transfer will occurs
 * @rx_ts: the address where to store RX timestamp value
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_read_rx_timestamp(struct dw3000 *dw, u64 *rx_ts)
{
	/* Use a prebuilt SPI message to be as fast as possible. */
	struct spi_message *msg;
	struct spi_transfer *tr;
	int hlen; /* depend on selected message */
	int rc;

	trace_dw3000_read_rx_timestamp(dw);
	switch (dw->data.dblbuffon) {
	case DW3000_DBL_BUFF_ACCESS_BUFFER_A:
		msg = dw->msg_read_rx_timestamp_a;
		break;
	case DW3000_DBL_BUFF_ACCESS_BUFFER_B:
		msg = dw->msg_read_rx_timestamp_b;
		break;
	default:
		msg = dw->msg_read_rx_timestamp;
	}
	tr = list_first_entry(&msg->transfers, struct spi_transfer,
			      transfer_list);
	/* Calc header len using known data size (smaller than sizeof(*rx_ts)) */
	hlen = tr->len - DW3000_RX_TIME_RX_STAMP_LEN;
	/* Execute this spi message synchronously */
	rc = dw3000_spi_sync(dw, msg);
	if (!rc)
		*rx_ts = get_unaligned_le64(tr->rx_buf + hlen) &
			 ((1ull << (DW3000_RX_TIME_RX_STAMP_LEN * 8)) - 1);
	trace_dw3000_return_int_u64(dw, rc, *rx_ts);
	return rc;
}

/**
 * dw3000_configure_ciadiag() - Enable CIA diagnostic data
 * @dw: the DW device
 * @on: Enable/disable CIA to log all diagnostic registers
 * @opt: option specified in ciadiag_dbl_options enum.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_configure_ciadiag(struct dw3000 *dw, bool on,
				    enum ciadiag_dbl_options opt)
{
	struct dw3000_local_data *data = &dw->data;
	int rc;

	if (on) {
		rc = dw3000_reg_and8(dw, DW3000_CIA_CONF_ID, 2,
				     ~(DW3000_CIA_CONFIG_DIAG_OFF));

	} else {
		rc = dw3000_reg_or8(dw, DW3000_CIA_CONF_ID, 2,
				    DW3000_CIA_CONFIG_DIAG_OFF);
	}
	if (unlikely(rc))
		return rc;

	if (!data->dblbuffon) {
		if (opt != 0) {
			dev_warn(
				dw->dev,
				"cannot set CIA diagnostic option while the double buffering is disabled\n");
		}
		goto skip_opt;
	}
	/* Apply double buffering option */
	rc = dw3000_reg_write8(dw, DW3000_RDB_DIAG_MODE_ID, 0, opt);
	if (unlikely(rc))
		return rc;
skip_opt:
	data->ciadiag_enabled = on;
	return rc;
}

/**
 * dw3000_stats_enable() - Enable or disable all statistics
 * @dw:	the DW device
 * @on: true to enable statistics, otherwise disable it.
 *
 * Return: 0 on success, else a negative error code.
 */
inline int dw3000_rx_stats_enable(struct dw3000 *dw, const bool on)
{
	dw->stats.enabled = on;
	/**
	 * Enable the CIA diagnostic to get diagnostic values as CIR power
	 * and PACC count needed to calculate the RSSI in userspace.
	 */
	return dw3000_configure_ciadiag(dw, on, DW3000_CIA_DIAG_LOG_DBL_OFF);
}

/**
 * dw3000_rx_stats_clear() - Clear all statistics
 * @dw:	the DW device
 */
inline void dw3000_rx_stats_clear(struct dw3000 *dw)
{
	struct dw3000_stats *stats = &dw->stats;

	memset(stats->count, 0, sizeof(stats->count));
}

static bool dw3000_stats_enabled = false;
module_param_named(stats_enabled, dw3000_stats_enabled, bool, 0644);
MODULE_PARM_DESC(stats_enabled,
		 "Enable statistics gathering, to be used with traces, only provides RSSI at this time");

/**
 * dw3000_rx_store_rssi() - Get RSSI data from last good RX frame
 * @dw: the DW device
 *
 * The RSSI data must be read before incrementing counter.
 * It requires at least one received frame to get any data from
 * register. Both 'cir_pwr' and 'pacc_cnt' values cannot be null
 * regarding the RSSI formula in the DW3700 User Manual v0.1 section 4.6.2.
 *
 * Return: 0 on success, else a negative error code.
 */
static int dw3000_rx_store_rssi(struct dw3000 *dw)
{
	struct dw3000_config *config = &dw->config;
	struct dw3000_stats *stats = &dw->stats;
	/* Only read RSSI after good RX frame */
	int idx = stats->count[DW3000_STATS_RX_GOOD] - 1;
	bool sts = _dw3000_sts_is_enabled(dw);
	const char *chip_name = dw3000_get_chip_name(dw);
	struct dw3000_rssi *rssi;
	int rc;
	u32 cir_pwr;
	u16 pacc_cnt;
	u8 dgc_dec;

	/* No data available */
	if (idx < 0)
		return -EAGAIN;
	/* Get RSSI data pointer to store data */
	rssi = &stats->rssi[idx];
	/* Read CIR power value */
	rc = dw3000_reg_read32(
		dw, _ciadiag_reg_info[dw->data.ciadiag_reg_select].diag1, 0,
		&cir_pwr);
	if (unlikely(rc))
		return rc;
	/* Read preamble accumulation count value */
	rc = dw3000_reg_read16(
		dw, _ciadiag_reg_info[dw->data.ciadiag_reg_select].diag12, 0,
		&pacc_cnt);
	if (unlikely(rc))
		return rc;
	/* Avoid "nan" and "-inf" in userspace when calculating average RSSI */
	if (cir_pwr == 0 || pacc_cnt == 0)
		dev_info(dw->dev,
			"one or both values from CIA registers are null\n");

	rssi->cir_pwr = cir_pwr;
	rssi->pacc_cnt = pacc_cnt;
	rssi->prf_64mhz = ((config->rxCode >= 9) && (config->rxCode <= 24));

	/* Read chip specific DGC_DBG */
	rc = dw->chip_ops->get_dgc_dec(dw, &dgc_dec);
	if (unlikely(rc))
		return rc;
	rssi->dgc_dec = dgc_dec;

	trace_dw3000_rx_rssi(dw, chip_name, sts, rssi->cir_pwr, rssi->pacc_cnt,
			     rssi->prf_64mhz, rssi->dgc_dec);
	return 0;
}

/**
 * dw3000_rx_stats_inc() - Increment statistics
 * @dw: the DW device
 * @item: statistics item
 *
 * Return: 0 on success, else a negative error code.
 */
static inline int dw3000_rx_stats_inc(struct dw3000 *dw,
				      const enum dw3000_stats_items item)
{
	int rc = 0;
	if (dw->stats.enabled) {
		if (dw->stats.count[item] >= DW3000_RSSI_REPORTS_MAX)
			dw->stats.count[item] = 0;
		dw->stats.count[item]++;
		if (item == DW3000_STATS_RX_GOOD) {
			rc = dw3000_rx_store_rssi(dw);
		}
	}
	return rc;
}

static int dw3000_power_supply(struct dw3000 *dw,
			       struct dw3000_power_control *power, int onoff)
{
	int rc;

	if (!power->regulator_1p8 && !power->regulator_2p5) {
		dev_warn(dw->dev, "No regulators, assuming always on\n");
		return 0;
	}

	if (power->regulator_1p8) {
		if (onoff)
			rc = regulator_enable(power->regulator_1p8);
		else
			rc = regulator_disable(power->regulator_1p8);
	}

	if (power->regulator_2p5) {
		if (onoff)
			rc = regulator_enable(power->regulator_2p5);
		else
			rc = regulator_disable(power->regulator_2p5);
	}
	/* Ensure RESET is asserted at least the required time */
	usleep_range(DW3000_HARD_RESET_DELAY_US,
		     DW3000_HARD_RESET_DELAY_US + 500);
	return rc;
}

/**
 * dw3000_reset_assert() - reset gpio control
 * @dw: the DW device
*
 * The configured reset gpio is switched to input to ensure it is not
 * driven low.
 *
 * Context: Must be call outside the thread handling device IRQ.
 * Return: 0 on success, else a negative error code.
 */
static int dw3000_reset_assert(struct dw3000 *dw, bool reset)
{
	int rc;

	if (!gpio_is_valid(dw->reset_gpio)) {
		dev_err(dw->dev, "invalid reset gpio\n");
		return -EINVAL;
	}

	if (reset) {
		/* Assert RESET GPIO */
		rc = gpio_direction_output(dw->reset_gpio, 0);
		if (rc)
			dev_err(dw->dev, "Could not set reset gpio as output\n");
	} else {
		/* Release RESET GPIO.
		 * Reset should be open drain, or switched to input whenever not driven
		 * low. It should not be driven high. */
		rc = gpio_direction_input(dw->reset_gpio);
		if (rc)
			dev_err(dw->dev, "Could not set reset gpio as input\n");
	}
	return rc;
}




/**
 * dw3000_set_operational_state() - Set new operational state for the device
 * @dw: the DW device
 * @st: the new operational state to set
 *
 * This ensure waiting thread are wake-up when operational state is changed.
 */
static void dw3000_set_operational_state(struct dw3000 *dw,
					 enum operational_state st)
{
	trace_dw3000_set_operational_state(dw, st);
	dw->current_operational_state = st;
	wake_up(&dw->operational_state_wq);
}


/**
 * dw3000_poweron() - Power-on device using configured reset gpio
 * @dw: the DW device to power-on
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_poweron(struct dw3000 *dw)
{
	int timeout;
	int rc;

	if (dw->is_powered) {
		dev_info(dw->dev, "device already powered on\n");
		return 0;
	}

	rc = dw3000_power_supply(dw, &dw->regulators, true);
	if (rc) {
		dev_err(dw->dev, "Could not enable regulator\n");
		return rc;
	}

	dw->is_powered = true;
	/* HW may rely only on regulator, so exit without error if no GPIO */
	if (!gpio_is_valid(dw->reset_gpio))
		goto stats;

	rc = dw3000_reset_assert(dw, false);
	if (rc)
		return rc;

stats:
	/* Force slow SPI clock speed (at device level) */
	dw3000_change_speed(dw, DW3000_SPI_SLOW_HZ);
	/* Enable interrupt so we can catch the SPI ready IRQ */
	enable_irq(dw->spi->irq);

	/* Now, wait for SPI ready interrupt */
	timeout = msecs_to_jiffies(500);

	/* No IRQs after this point until device is enabled */
	disable_irq(dw->spi->irq);
	/* Restore max SPI clock speed */
	dw3000_change_speed(dw, dw->of_max_speed_hz);

	if (!rc)
		dw3000_power_stats(dw, DW3000_PWR_RUN, 0);
	return rc;
}

/**
 * dw3000_poweroff() - Power-off device using configured reset gpio
 * @dw: the DW device to power-on
 *
 * The configured reset gpio is switched to output, at low state.
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_poweroff(struct dw3000 *dw)
{
	struct dw3000_local_data *local = &dw->data;
	int rc;

	if (!dw->is_powered) {
		dev_info(dw->dev, "device already powered off\n");
		return 0;
	}

	rc = dw3000_power_supply(dw, &dw->regulators, false);
	if (rc) {
		dev_err(dw->dev, "Could not disable regulator\n");
		return rc;
	}

	dw->is_powered = false;

	/* Clear security registers related cache */
	memset(local->sts_key, 0, AES_KEYSIZE_128);
	memset(local->sts_iv, 0, AES_BLOCK_SIZE);
	/* Reset STS mode */
	dw->config.stsMode = DW3000_STS_MODE_OFF | DW3000_STS_MODE_SDC;



	/* HW may rely only on regulator, so exit without error if no GPIO */
	if (!gpio_is_valid(dw->reset_gpio))
		goto stats;

	/* Assert RESET GPIO */
	rc = dw3000_reset_assert(dw, true);
	if (rc)
		return rc;

stats:
	dw3000_set_operational_state(dw, DW3000_OP_STATE_OFF);
	dw3000_power_stats(dw, DW3000_PWR_OFF, 0);

	/* Ensure RESET is asserted at least the required time */
	usleep_range(DW3000_HARD_RESET_DELAY_US,
		     DW3000_HARD_RESET_DELAY_US + 100);
	return 0;
}

/**
 * dw3000_forcetrxoff() - Force device in idle mode, TX/RX off
 * @dw: the DW device
 *
 * According to the DW3000 manual, this command must be send with IRQ disable
 * to avoid a race condition.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_forcetrxoff(struct dw3000 *dw)
{
	int rc;
	u8 idle = 0;

	/* Check if in TX or RX state before forcing device into IDLE state */
	rc = dw3000_reg_read8(dw, DW3000_SYS_STATE_LO_ID, 2, &idle);
	if (idle <= DW3000_SYS_STATE_IDLE) {
		/* Device is already in IDLE or IDLE_RC ... must not force to IDLE if in IDLE_RC */
		return rc;
	}
	disable_irq(dw->spi->irq);
	rc = dw3000_write_fastcmd(dw, DW3000_CMD_TXRXOFF);
	enable_irq(dw->spi->irq);
	if (!rc)
		dw3000_power_stats(dw, DW3000_PWR_IDLE, 0);
	return rc;
}

/**
 * dw3000_setpreambledetecttimeout() - Set the preamble detection timeout
 * @dw: the DW device
 * @timeout: preamble detection timeout in units of PAC size symbols.
 *
 * The counter automatically adds 1 PAC size to the value set. Min value that
 * can be set is 1 (i.e. a timeout of 2 PAC size).
 *
 * The default/reset value is zero which disables the preamble detection
 * timeout.
 *
 * Return: zero on success, else a negative error code.
 */
static inline int dw3000_setpreambledetecttimeout(struct dw3000 *dw,
						  u16 timeout)
{
	struct dw3000_local_data *local = &dw->data;
	int rc;
	/*
	 * Compare with the previous value stored if register access
	 * is necessary.
	 */
	if (local->rx_timeout_pac == timeout)
		return 0;
	rc = dw3000_reg_write16(dw, DW3000_DRX_PRETOC_ID, 0, timeout);
	if (unlikely(rc))
		return rc;
	local->rx_timeout_pac = timeout;
	return 0;
}

static inline int dw3000_setdelayedtrxtime(struct dw3000 *dw, u32 starttime)
{
	u32 sys_starttime = dw3000_dtu_to_sys_time(dw, starttime);
	return dw3000_reg_write32(dw, DW3000_DX_TIME_ID, 0, sys_starttime);
}

/**
 * dw3000_can_deep_sleep() - check delay before next operation
 * @dw: the DW device
 * @delay_us: the delay before which RX/TX must be executed
 *
 * Return: zero if not enough time to enter deep sleep, else a positive delay
 *  in us.
 */
int dw3000_can_deep_sleep(struct dw3000 *dw, int delay_us)
{
	/* We can't enter DEEP_SLEEP if previous operation has
	 * required ranging clock or if deep-sleep is disabled. */
	if (dw->need_ranging_clock || (dw->auto_sleep_margin_us < 0))
		return 0;
	if (delay_us < max(DW3000_WAKEUP_LATENCY_US, dw->auto_sleep_margin_us))
		return 0;
	/* Take care of wakeup latency in returned result */
	return delay_us - DW3000_WAKEUP_LATENCY_US;
}

/**
 * dw3000_wakeup() - wake-up device by forcing CS line down long enough
 * @dw: the DW device
 *
 * Return: 0 on success, else a negative error code.
 */
static int dw3000_wakeup(struct dw3000 *dw)
{
	/* Wake UP require a minimum time of 500us CS low. Use one of prebuilt
	   SPI messages to be as fast as possible and modify it to include CS
	   required delay in microseconds. */
	struct spi_message *msg = dw->msg_read_sys_status;
	struct spi_transfer *tr = list_first_entry(
		&msg->transfers, struct spi_transfer, transfer_list);
	int rc;

	/* Avoid race condition with dw3000_poweroff while chip is stopped just
	   when dw3000_wakeup_timer() HR timer callback is executed. Do nothing
	   if not in DEEP-SLEEP state. */
	if (dw->current_operational_state != DW3000_OP_STATE_DEEP_SLEEP)
		return 0;

	trace_dw3000_wakeup(dw);

	/* Add a delay after transfer. See spi_transfer_delay_exec() called by
	   spi_transfer_one_message(). */
	tr->delay_usecs = DW3000_SPI_CS_WAKEUP_DELAY_US;
	/* Now, execute SPI modified message/transfer */
	rc = dw3000_spi_sync(dw, msg);
	if (!rc) {
		/* We are waking up the chip, update state according */
		dw3000_set_operational_state(dw, DW3000_OP_STATE_WAKE_UP);
		/* Re-enable spi's irqs. deepsleep disable them */
		enable_irq(dw->spi->irq);
	}
	/* Reset delay in transfer */
	tr->delay_usecs = 0;
	return rc;
	/* The next part of the wake process is located in
	   dw3000_isr_handle_spi_ready(), executed when SPIRDY interrupt is
	   triggered */
}

static int do_wakeup(struct dw3000 *dw, const void *in, void *out)
{
	return dw3000_wakeup(dw);
}

static void dw3000_timer_expired(struct work_struct *work)
{
	struct dw3000 *dw =
		container_of(work, struct dw3000, timer_expired_work);
	/* Entered DEEP SLEEP from mcps802154_ops.idle() */
	mcps802154_timer_expired(dw->llhw);
}

/**
 * dw3000_wakeup_timer() - wake-up timer handler
 * @timer: the deep_sleep_timer field in struct dw3000
 *
 * Return: 0 on success, else a negative error code.
 */
enum hrtimer_restart dw3000_wakeup_timer(struct hrtimer *timer)
{
	struct dw3000 *dw =
		container_of(timer, struct dw3000, deep_sleep_timer);
	if (dw->nfcc_coex.enabled ||
	    (dw->deep_sleep_state.next_operational_state >
	     DW3000_OP_STATE_IDLE_PLL)) {
		struct dw3000_stm_command cmd = { do_wakeup, NULL, NULL };
		/* The chip is about to wake up, let's request the best QoS
		   latency early */
		dw3000_pm_qos_update_request(dw, dw3000_qos_latency);
		/* Must wakeup to execute stored operation, so run the
		   wake up function in state machine thread. */
		dw3000_enqueue_timer(dw, &cmd);
	} else if (dw->call_timer_expired) {
		/* Timer launched by idle(), just inform MCPS timer has expired */
		schedule_work(&dw->timer_expired_work);
	}
	hrtimer_try_to_cancel(timer);
	return HRTIMER_NORESTART;
}

/**
 * dw3000_wakeup_cancel() - Cancel wake-up timer
 * @dw: the DW device
 */
void dw3000_wakeup_cancel(struct dw3000 *dw)
{
	hrtimer_try_to_cancel(&dw->deep_sleep_timer);
}

/**
 * dw3000_wakeup_and_wait() - Check current device operational state
 * @dw: the DW device to execute an RX/TX/configuration operation
 *
 * Can't program parameters in device if not in IDLE_PLL state, so
 * we need to wakeup it at different places.
 */
void dw3000_wakeup_and_wait(struct dw3000 *dw)
{
	if (dw->current_operational_state >= DW3000_OP_STATE_IDLE_PLL)
		return;
	/* Ensure wakeup ISR don't call the mcps802154_timer_expired() since
	   this will result in dead lock! */
	dw->call_timer_expired = false;
	/* Ensure force call to dw3000_wakeup() is made. */
	dw->deep_sleep_state.next_operational_state = DW3000_OP_STATE_MAX;
	/* Now wakeup device, and stop timer if running */
	dw3000_wakeup_timer(&dw->deep_sleep_timer);
	dw->deep_sleep_state.next_operational_state = DW3000_OP_STATE_IDLE_PLL;
	/* And wait for good state */
	if (current != dw->stm.mthread)
		wait_event(dw->operational_state_wq,
			   dw->current_operational_state ==
				   DW3000_OP_STATE_IDLE_PLL);
	else
		/* TODO: find a better solution to allow wakeup isr processing
		   recursively.*/
		mdelay(2);
}

/**
 * dw3000_check_operational_state() - Check current device operational state
 * @dw: the DW device to execute an RX/TX/configuration operation
 * @delay_dtu: delay before expected operation
 * @can_sync: true when it's possible to sync the clock
 *
 * This function will decide if device should enter/leave DEEP SLEEP
 * state according current state and delay before next operation.
 *
 * Context: called from dw3000-spi thread only.
 * Return: 0 if ready, 1 if in deep-sleep or waking-up, or a negative error
 *         code.
 */
int dw3000_check_operational_state(struct dw3000 *dw, int delay_dtu,
				   bool can_sync)
{
	int delay_us = DTU_TO_US(delay_dtu);
	int rc;

	trace_dw3000_check_operational_state(
		dw, delay_dtu, dw->current_operational_state,
		dw->deep_sleep_state.next_operational_state);

	if (dw->current_operational_state == DW3000_OP_STATE_OFF)
		return -EIO;
	/* In deep sleep or wake up in progress, we can store parameters only
	   if no other operation queued. */
	if ((dw->current_operational_state < DW3000_OP_STATE_IDLE_PLL) &&
	    (dw->deep_sleep_state.next_operational_state !=
	     DW3000_OP_STATE_IDLE_PLL))
		return -EIO;

	switch (dw->current_operational_state) {
	case DW3000_OP_STATE_DEEP_SLEEP:
		/* Update delay_us with wakeup margin */
		delay_us = dw3000_can_deep_sleep(dw, delay_us);
		if (delay_us) {
			/* No need to wakeup now! Reprogram timer only. */
			dw3000_wakeup_timer_start(dw, delay_us);
			/* Stay in deep sleep, inform caller to save params. */
			return 1;
		}
		/* The chip is about to wake up, request the best QoS latency */
		dw3000_pm_qos_update_request(dw, dw3000_qos_latency);
		/* Wakeup now since delay isn't enough. */
		rc = dw3000_wakeup(dw);
		if (unlikely(rc))
			return rc;
		/* fallthrough */
	case DW3000_OP_STATE_WAKE_UP:
		/* Inform caller to save parameters. Stored operation will redo
		   deep sleep if needed. */
		return 1;
	default:
		/* May need to resync when deep sleep is not enabled */
		if (can_sync)
			dw3000_may_resync(dw);
		/* Update delay_us with wakeup margin */
		delay_us = dw3000_can_deep_sleep(dw, delay_us);
		if (!delay_us)
			break;
		/* Enter DEEP SLEEP and setup wakeup timer */
		rc = dw3000_deep_sleep_and_wakeup(dw, delay_us);
		if (!rc)
			return 1; /* And save params. */
		/* Failure to enter deep sleep, continue without it */
		break;
	}
	return 0;
}

/**
 * dw3000_check_hpdwarn() - check HPDWARN event status bit
 * @dw: the DW device
 *
 * The HPDWARN event status bit relates to the use of delayed transmit and
 * delayed receive functionality. It indicates the delay is more than half
 * a period of the system clock. The HPDWARN status flag can be polled after a
 * delayed transmission or reception is commanded, to check whether the delayed
 * send/receive invocation was given in time (0) or not (1).
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_check_hpdwarn(struct dw3000 *dw)
{
	u8 status;
	int rc;

	rc = dw3000_reg_read8(dw, DW3000_SYS_STATUS_ID, 3, &status);
	if (rc)
		return rc;
	if (status & (DW3000_SYS_STATUS_HPDWARN_BIT_MASK >> 24)) {
		dw3000_forcetrxoff(dw);
		return -ETIME;
	}
	return 0;
}

/**
 * dw3000_rx_disable() - Disable RX
 * @dw: the DW device to put back in IDLE state
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_rx_disable(struct dw3000 *dw)
{
	return dw3000_forcetrxoff(dw);
}

/**
 * dw3000_rx_enable() - Enable RX
 * @dw: the DW device to put in RX mode
 * @rx_delayed:  true if RX delayed must be use
 * @date_dtu: the date at which RX must be enabled if @rx_delayed is true
 * @timeout_pac: the preamble detect timeout
 *
 * This function enable RX on DW3000, delayed or not, with a configurable
 * preamble detection timeout.
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_rx_enable(struct dw3000 *dw, bool rx_delayed, u32 date_dtu,
		     u32 timeout_pac)
{
	u32 cur_time_dtu = 0;
	int rc;

	/* Read current DTU time to compare with next transfer time */
	if (dw->coex_gpio >= 0)
		cur_time_dtu = dw3000_get_dtu_time(dw);

	/* Configure timeout */
	rc = dw3000_setpreambledetecttimeout(dw, timeout_pac);
	if (unlikely(rc))
		return rc;

	/* Update RX parameters according to WiFi coexistence */
	rc = dw3000_coex_start(dw, &rx_delayed, &date_dtu, cur_time_dtu);
	if (unlikely(rc))
		return rc;

	if (!rx_delayed) {
		rc = dw3000_write_fastcmd(dw, DW3000_CMD_RX);
		if (unlikely(rc))
			goto stop_coex;
		dw3000_power_stats(dw, DW3000_PWR_RX, 0);
		return 0;
	}

	rc = dw3000_setdelayedtrxtime(dw, date_dtu);
	if (unlikely(rc))
		goto stop_coex;
	rc = dw3000_write_fastcmd(dw, DW3000_CMD_DRX);
	if (unlikely(rc))
		goto stop_coex;
	/* Apply power stats now, will goes back to IDLE in dw3000_forcetrxoff() */
	dw3000_power_stats(dw, DW3000_PWR_RX, date_dtu);
	/* Check if late */
	rc = dw3000_check_hpdwarn(dw);
	if (unlikely(rc)) {
		if (rc == -ETIME) {
			cur_time_dtu = dw3000_get_dtu_time(dw);
			dev_err(dw->dev,
				"cannot program delayed rx date_dtu=%x current_dtu=%x\n",
				date_dtu, cur_time_dtu);
		}
		goto stop_coex;
	}
	return 0;
stop_coex:
	dw3000_coex_stop(dw);
	return rc;
}

/**
 * dw3000_do_rx_enable() - handle RX enable MCPS operation
 * @dw: the DW device to put in RX mode
 * @info: RX enable parameters from MCPS
 * @frame_idx: Frame index in a continuous block
 *
 * This function is called to execute all required operation to enable RX on
 * the device using provided parameters.
 *
 * Since RX enable may be deferred until device is wokenup, it may be also
 * called a second time with same parameters from the wakeup ISR.
 *
 * This function calls dw3000_check_operational_state() and save and defer
 * the operation if DEEP-SLEEP is possible according delay.
 *
 * If RX must be enable immediately, this function configure STS, PDOA and
 * Antenna pair before enable RX by calling dw3000_rx_enable().
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_do_rx_enable(struct dw3000 *dw,
			const struct mcps802154_rx_info *info, int frame_idx)
{
	struct dw3000_deep_sleep_state *dss = &dw->deep_sleep_state;
	struct mcps802154_llhw *llhw = dw->llhw;
	u32 cur_time_dtu = 0;
	u32 date_dtu = 0;
	u32 timeout_pac = 0;
	bool rx_delayed = true;
	int delay_dtu = 0;
	bool can_sync = false;
	bool override_rx_antenna = dw->sp0_rx_antenna > -1 &&
		!(info-> flags & MCPS802154_RX_INFO_RANGING);
	u8 rx_antenna = info->ant_pair_id;
	int rc;
	bool pdoa_enabled;
	u8 sts_mode;

	trace_dw3000_mcps_rx_enable(dw, info->flags, info->timeout_dtu);

	/* Calculate the transfer date. */
	if (info->flags & MCPS802154_RX_INFO_TIMESTAMP_DTU)
		date_dtu = info->timestamp_dtu - DW3000_RX_ENABLE_STARTUP_DTU;
	else
		/* Receive immediately. */
		rx_delayed = false;

	/* Calculate the timeout. Do nothing if no timeout */
	if (info->timeout_dtu == 0)
		timeout_pac = dw->pre_timeout_pac;
	else if (info->timeout_dtu != -1)
		timeout_pac = dw->pre_timeout_pac + dtu_to_pac(llhw, info->timeout_dtu);

	if (rx_delayed) {
		cur_time_dtu = dw3000_get_dtu_time(dw);
		delay_dtu = (int)(date_dtu - cur_time_dtu);
		if (delay_dtu < 0) {
			dev_err(dw->dev,
				"too late to program delayed rx date_dtu=%x current_dtu=%x\n",
				date_dtu, cur_time_dtu);
			return -ETIME;
		}
	}

	/* We are always allowed to sleep & sync at the beginning of a block. */
	if (frame_idx == 0) {
		dw->need_ranging_clock = false;
		can_sync = true;
	}

	/* For delayed RX, where delay_dtu != 0, enter/leave deep sleep */
	rc = dw3000_check_operational_state(dw, delay_dtu, can_sync);
	if (rc) {
		/* Handle error cases first */
		if (rc < 0)
			return rc;
		/* Save parameters to activate RX delayed when
		   wakeup later */
		dss->next_operational_state = DW3000_OP_STATE_RX;
		dss->rx_info = *info;
		dss->frame_idx = frame_idx;
		return 0;
	}
	/* All operation below require the DW chip is in IDLE_PLL state */

	/* Enable STS */
	pdoa_enabled = !!(info->flags & MCPS802154_RX_INFO_RANGING_PDOA);
	sts_mode = FIELD_GET(MCPS802154_RX_INFO_STS_MODE_MASK, info->flags);
	rc = dw3000_set_sts_pdoa(dw, sts_mode,
				 sts_to_pdoa(sts_mode, pdoa_enabled));
	if (unlikely(rc))
		goto fail;
	/* Ensure correct RX antenna are selected. */
	if (override_rx_antenna)
		rx_antenna = ANTPAIR_OFFSET(dw->sp0_rx_antenna);
	rc = dw3000_set_rx_antennas(dw, rx_antenna, pdoa_enabled);
	if (unlikely(rc))
		goto fail;

	if (info->flags & MCPS802154_RX_INFO_AACK)
		dw3000_enable_autoack(dw, false);
	else
		dw3000_disable_autoack(dw, false);

	rc = dw3000_rx_enable(dw, rx_delayed, date_dtu, timeout_pac);
	if (unlikely(rc))
		goto fail;

	/* Store ranging clock requirement for next operation */
	dw->need_ranging_clock =
		(info->flags & MCPS802154_RX_INFO_KEEP_RANGING_CLOCK) != 0;

fail:
	trace_dw3000_return_int(dw, rc);
	return rc;
}

static irqreturn_t dw3000_irq_handler(int irq, void *context)
{
	struct dw3000 *dw = context;

	dw3000_enqueue_irq(dw);

	return IRQ_HANDLED;
}

/**
 * dw3000_setup_regulators() - request regulator
 * @dw: the DW device to request regulators.
 */

void dw3000_setup_regulators(struct dw3000 *dw)
{
	struct regulator *regulator_1p8, *regulator_2p5;

	regulator_1p8 = devm_regulator_get_optional(dw->dev, "power_reg_1p8");
	if (IS_ERR_OR_NULL(regulator_1p8)) {
		dev_dbg(dw->dev, "No regulator 1.8V found in DT\n");
		regulator_1p8 = NULL;
	}

	regulator_2p5 = devm_regulator_get_optional(dw->dev, "power_reg_2p5");
	if (IS_ERR_OR_NULL(regulator_2p5)) {
		dev_dbg(dw->dev, "No regulator 2.5V found in DT\n");
		regulator_2p5 = NULL;
	}

	dw->regulators.regulator_1p8 = regulator_1p8;
	dw->regulators.regulator_2p5 = regulator_2p5;
}

/**
 * dw3000_setup_reset_gpio() - request reset GPIO
 * @dw: the DW device to get reset GPIO config from DT
 *
 * Get the reset GPIO to use from the DT and configure it in OUTPUT
 * open-drain mode and activate it. This ensure the DW device is
 * in reset state, IRQ pin low from the end of this function.
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_setup_reset_gpio(struct dw3000 *dw)
{
	/* Initialise reset GPIO pin as output */
	dw->reset_gpio =
		of_get_named_gpio(dw->dev->of_node, "uwbhal,reset-gpio", 0);
	if (!gpio_is_valid(dw->reset_gpio)) {
		dev_warn(dw->dev, "device does not support GPIO RESET control");
		return 0;
	}
	return devm_gpio_request_one(dw->dev, dw->reset_gpio,
				     GPIOF_DIR_OUT | GPIOF_OPEN_DRAIN |
					     GPIOF_INIT_LOW,
				     "dw3000-reset");
}

/**
 * dw3000_setup_irq() - request IRQ for the device
 * @dw: the DW device
 *
 * Ensure the IRQ is correctly configured and install the hard IRQ handler.
 * The IRQ is immediately disabled, waiting the device to be started by MCPS.
 *
 * Note: If the reset GPIO is deasserted before this function, a spurious
 *  IRQ may be handled and dw3000_irq_handler() called. This IRQ is ignored
 *  if occurs before dw3000_init() call.
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_setup_irq(struct dw3000 *dw)
{
	int rc;
	int irq_flags, irq_gpio;

	/* Check the presence of irq-gpio in DT. If so,
	 * use it as irq, if not, "interrupt-parent" or
	 * "interrupt-extended" should be provided and then used.
	 */
	irq_gpio = of_get_named_gpio(dw->dev->of_node, "irq-gpio", 0);
	if (irq_gpio > 0) {
		if (!gpio_is_valid(irq_gpio))
			return -EINVAL;

		devm_gpio_request_one(dw->dev, irq_gpio, GPIOF_IN,
				      dev_name(dw->dev));
		dw->spi->irq = gpio_to_irq(irq_gpio);
	}

	irq_flags = irq_get_trigger_type(dw->spi->irq);
	if (!irq_flags) {
		irq_flags = IRQF_TRIGGER_HIGH;
	}

	/* Hook interruption */
	rc = devm_request_irq(dw->dev, dw->spi->irq, dw3000_irq_handler,
			      irq_flags, dev_name(dw->dev), dw);
	if (rc) {
		dev_err(dw->dev, "could not request the IRQ %d: %d\n",
			dw->spi->irq, rc);
		return rc;
	}

	/* Disable interrupt before enabling the device */
	disable_irq_nosync(dw->spi->irq);

	return 0;
}

int dw3000_hardreset(struct dw3000 *dw)
{
	int rc;

	rc = dw3000_reset_assert(dw, true);
	if (rc)
		return rc;

	usleep_range(DW3000_HARD_RESET_DELAY_US, DW3000_HARD_RESET_DELAY_US + 100);

	rc = dw3000_reset_assert(dw, false);
	if (rc)
		return rc;

	usleep_range(DW3000_HARD_RESET_DELAY_US, DW3000_HARD_RESET_DELAY_US + 100);

	return 0;
}

static inline int dw3000_clear_aonconfig(struct dw3000 *dw)
{
	int rc;
	/* Clear any AON auto download bits (as reset will trigger AON
	 * download). */
	rc = dw3000_reg_write16(dw, DW3000_AON_DIG_CFG_ID, 0, 0x00);
	if (rc)
		return rc;
	/* Clear the wake-up configuration */
	rc = dw3000_reg_write8(dw, DW3000_AON_CFG_ID, 0, 0x00);
	if (rc)
		return rc;
	/* Upload the new configuration */
	rc = dw3000_reg_write8(dw, DW3000_AON_CTRL_ID, 0, 0);
	if (rc)
		return rc;
	return dw3000_reg_write8(dw, DW3000_AON_CTRL_ID, 0,
				 DW3000_AON_CTRL_ARRAY_UPLOAD_BIT_MASK);
}

static void _dw3000_softreset(struct dw3000 *dw)
{
	/* Clear any AON configurations (this will leave the device at FOSC/4,
	 * thus we need low SPI rate)
	 */
	dw3000_clear_aonconfig(dw);
	/* Make sure the new AON array config has been set */
	usleep_range(DW3000_SOFT_RESET_DELAY_US,
		     DW3000_SOFT_RESET_DELAY_US + 100);
	/* Need to make sure clock is not PLL as the PLL will be switched off
	 * as part of reset.
	 */
	dw3000_reg_or8(dw, DW3000_CLK_CTRL_ID, 0, DW3000_FORCE_SYSCLK_FOSC);

	/* Call version specific softreset */
	dw->chip_ops->softreset(dw);

	/* DW3000 needs a 10us sleep to let clk PLL lock after reset
	 * - the PLL will automatically lock after the reset
	 * Could also have polled the PLL lock flag,
	 * but then the SPI needs to be <= 7MHz !! So a simple delay is easier.
	 */
	usleep_range(DW3000_SOFT_RESET_DELAY_US,
		     DW3000_SOFT_RESET_DELAY_US + 100);
	/* DW3000 not in sleep_mode anymore */
	dw->data.sleep_mode = 0;
}

static int dw3000_tx_write_data(struct dw3000 *dw, u8 *buffer, u16 len,
				u16 offset)
{
	if ((offset + len) >= DW3000_TX_BUFFER_MAX_LEN)
		return -1;
	if (offset <= DW3000_REG_DIRECT_OFFSET_MAX_LEN) {
		/* Directly write the data to the IC TX buffer */
		dw3000_xfer(dw, DW3000_TX_BUFFER_ID, offset, len, buffer,
			    DW3000_SPI_WR_BIT);
	} else {
		/* Program the indirect offset register A for
			* specified offset to TX buffer
			*/
		dw3000_reg_write32(dw, DW3000_INDIRECT_ADDR_A_ID, 0,
				   (DW3000_TX_BUFFER_ID >> 16));
		dw3000_reg_write32(dw, DW3000_INDIRECT_ADDR_A_ID, 0, offset);

		/* Indirectly write the data to the IC TX buffer */
		dw3000_xfer(dw, DW3000_INDIRECT_POINTER_A_ID, 0, len, buffer,
			    DW3000_SPI_WR_BIT);
	}
	return 0;
}

/**
 * dw3000_change_speed() - change SPI speed and update transfers
 * @dw: the DW device to change the speed
 * @new_speed: the new speed to use for all transfers
 *
 * The speed is first changed into the SPI device structure than
 * all pre-computed SPI transfers are updated if their speed isn't
 * the same.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_change_speed(struct dw3000 *dw, u32 new_speed)
{
	/* Setup new SPI speed only if changed */
	if (new_speed == dw->spi->max_speed_hz)
		return 0;
	/* Change SPI max speed */
	dw->spi->max_speed_hz = new_speed;
	/* Need to reset pre-allocated message which hold speed */
	return dw3000_transfers_reset(dw);
}

/**
 * dw3000_softreset() - perform soft-reset of the device
 * @dw: the device to soft-reset
 *
 * The SPI speed is changed to low-speed before doing the soft-reset and is
 * restored to full-speed (the speed configured in device tree) after.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_softreset(struct dw3000 *dw)
{
	int rc;
	/* Force slow SPI clock speed (at device level) */
	dw3000_change_speed(dw, DW3000_SPI_SLOW_HZ);

	/* Soft reset (require to known chip version) */
	_dw3000_softreset(dw);

	/* Re-read device ID to ensure bus is operational at low-speed */
	rc = dw3000_check_devid(dw);
	if (rc)
		return rc;

	/* Switch back to full SPI clock speed */
	dw3000_change_speed(dw, dw->of_max_speed_hz);

	/* Check device ID to ensure bus is operational at high-speed */
	return dw3000_check_devid(dw);
}

static inline int dw3000_ctrl_rftx_blocks(struct dw3000 *dw, u32 chan,
					  int reg_fileid)
{
	u32 val;

	if (reg_fileid != DW3000_RF_ENABLE_ID &&
	    reg_fileid != DW3000_RF_CTRL_MASK_ID)
		return -EINVAL;
	val = DW3000_RF_ENABLE_TX_SW_EN_BIT_MASK |
	      DW3000_RF_ENABLE_TX_EN_BIT_MASK |
	      DW3000_RF_ENABLE_TX_EN_BUF_BIT_MASK |
	      DW3000_RF_ENABLE_TX_BIAS_EN_BIT_MASK |
	      DW3000_RF_ENABLE_TX_CH_ALL_EN_BIT_MASK;
	return dw3000_reg_or32(dw, reg_fileid, 0, val);
}

/**
 * dw3000_rftx_blocks_autoseq_disable() - Disables automatic sequencing
 * of the tx-blocks
 * @dw: the DW device
 * @chan: specifies the operating channel (e.g. 5 or 9)
 *
 * Return: zero on success, else a negative error code.
 */
static inline int dw3000_rftx_blocks_autoseq_disable(struct dw3000 *dw,
						     u32 chan)
{
	return dw3000_ctrl_rftx_blocks(dw, chan, DW3000_RF_CTRL_MASK_ID);
}

/**
 * dw3000_rftx_blocks_enable() - Enable RF blocks for TX
 * @dw: the DW device
 * @chan: specifies the operating channel (e.g. 5 or 9)
 *
 * Return: zero on success, else a negative error code.
 */
static inline int dw3000_rftx_blocks_enable(struct dw3000 *dw, u32 chan)
{
	return dw3000_ctrl_rftx_blocks(dw, chan, DW3000_RF_ENABLE_ID);
}

/**
 * dw3000_enab_blocks_autoseq_disablele_rf_tx() - Turn on TX LDOs and enable RF blocks for TX
 * @dw: the DW device
 * @chan: specifies the operating channel (e.g. 5 or 9)
 * @switch_ctrl: specifies whether the switch needs to be configured for TX
 *
 * Enable TX LDOs and allow TX blocks to be manually turned on by
 * dw3000_rftx for a given channel.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_enable_rf_tx(struct dw3000 *dw, u32 chan, u8 switch_ctrl)
{
	int rc;
	/* Turn on TX LDOs */
	rc = dw3000_reg_or32(dw, DW3000_LDO_CTRL_ID, 0,
			     (DW3000_LDO_CTRL_LDO_VDDHVTX_VREF_BIT_MASK |
			      DW3000_LDO_CTRL_LDO_VDDHVTX_EN_BIT_MASK));
	rc = dw3000_reg_or32(dw, DW3000_LDO_CTRL_ID, 0,
			     (DW3000_LDO_CTRL_LDO_VDDTX2_VREF_BIT_MASK |
			      DW3000_LDO_CTRL_LDO_VDDTX1_VREF_BIT_MASK |
			      DW3000_LDO_CTRL_LDO_VDDTX2_EN_BIT_MASK |
			      DW3000_LDO_CTRL_LDO_VDDTX1_EN_BIT_MASK));
	/* Enable RF blocks for TX (configure RF_ENABLE_ID register) */
	if (chan == 5) {
		dw3000_reg_or32(dw, DW3000_RF_ENABLE_ID, 0,
				(0x2000000UL
				| 0x2000U | 0x1000U
				| 0x800U | 0x400U));
	} else {
		dw3000_reg_or32(dw, DW3000_RF_ENABLE_ID, 0,
				(0x2000000UL
				| 0x1000U
				| 0x800U | 0x400U));
	}
	if (rc)
		return rc;
	if (switch_ctrl) {
		/* Configure the TXRX switch for TX mode */
		return dw3000_reg_write32(dw, DW3000_RF_SWITCH_CTRL_ID, 0x0,
					  DW3000_TXRXSWITCH_TX);
	}
	return 0;
}

/**
 * dw3000_force_clocks() - Enable/disable clocks to particular digital blocks/system
 * @dw: the DW device
 * @clocks: set of clocks to enable/disable
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_force_clocks(struct dw3000 *dw, int clocks)
{
	if (clocks == DW3000_FORCE_CLK_SYS_TX) {
		/* TX_BUF_CLK = ON & RX_BUF_CLK = ON */
		u16 regvalue0 = DW3000_CLK_CTRL_TX_BUF_CLK_ON_BIT_MASK |
				DW3000_CLK_CTRL_RX_BUF_CLK_ON_BIT_MASK;
		/* SYS_CLK_SEL = PLL */
		regvalue0 |= (u16)DW3000_FORCE_SYSCLK_PLL
			     << DW3000_CLK_CTRL_SYS_CLK_SEL_BIT_OFFSET;
		/* TX_CLK_SEL = ON */
		regvalue0 |= (u16)DW3000_FORCE_CLK_PLL
			     << DW3000_CLK_CTRL_TX_CLK_SEL_BIT_OFFSET;
		return dw3000_reg_write16(dw, DW3000_CLK_CTRL_ID, 0x0,
					  regvalue0);
	}
	if (clocks == DW3000_FORCE_CLK_AUTO) {
		/* Restore auto clock mode */
		return dw3000_reg_write16(
			dw, DW3000_CLK_CTRL_ID, 0x0,
			(u16)(DW3000_CLK_CTRL_FORCE_NVM_CLK_EN_BIT_MASK |
			      DW3000_CLK_CTRL_RX_BUFF_AUTO_CLK_BIT_MASK |
			      DW3000_CLK_CTRL_CODE_MEM_AUTO_CLK_BIT_MASK));
	}
	return -EINVAL;
}

/**
 * dw3000_setfinegraintxseq() - Enable/disable the fine grain TX sequencing
 * @dw: the DW device
 * @on: true to enable fine grain TX sequencing, false to disable it.
 *
 * This is enabled by default in the DW3000.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_setfinegraintxseq(struct dw3000 *dw, bool on)
{
	if (on) {
		return dw3000_reg_write32(dw, DW3000_PWR_UP_TIMES_LO_ID, 2,
					  DW3000_PMSC_TXFINESEQ_ENABLE);
	}
	return dw3000_reg_write32(dw, DW3000_PWR_UP_TIMES_LO_ID, 2,
				  DW3000_PMSC_TXFINESEQ_DISABLE);
}

/**
 * dw3000_repeated_cw() - Enable a repeated continuous waveform on the device
 * @dw: the DW device
 * @cw_enable: CW mode enable
 * @cw_mode_config: CW configuration mode.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_repeated_cw(struct dw3000 *dw, int cw_enable,
			      int cw_mode_config)
{
	int rc;
	/* Turn off TX Seq */
	dw3000_setfinegraintxseq(dw, 0);
	/* Correct if passing a threshold */
	if (cw_mode_config > 0xF)
		cw_mode_config = 0xF;
	if (cw_enable > 3 || cw_enable < 1)
		cw_enable = 4;

	rc = dw3000_reg_write32(dw, DW3000_TX_TEST_ID, 0x0, 0x10 >> cw_enable);
	if (unlikely(rc))
		return rc;
	return dw3000_reg_write32(dw, DW3000_PG_TEST_ID, 0x0,
				  cw_mode_config << ((cw_enable - 1) * 4));
}

/**
 * dw3000_tx_setcwtone() - Start TX CW signal at specific channel frequency
 * @dw: the DW device
 * @on: true, enable the test mode and start transmitting a CW signal on the
 * device, otherwise disable it and back to normal mode.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_tx_setcwtone(struct dw3000 *dw, bool on)
{
	struct dw3000_txconfig *txconfig = &dw->txconfig;
	int rc;
	/* Enable test mode */
	if (on) {
		u8 chan = dw->config.chan;

		rc = dw3000_enable_rf_tx(dw, chan, 1);
		if (unlikely(rc))
			return rc;
		rc = dw3000_rftx_blocks_autoseq_disable(dw, chan);
		if (unlikely(rc))
			return rc;
		rc = dw3000_force_clocks(dw, DW3000_FORCE_CLK_SYS_TX);
		if (unlikely(rc))
			return rc;
		/* Go to test mode. PulseGen Channel 1, full power. */
		rc = dw3000_repeated_cw(dw, 1, 0xF);
		if (unlikely(rc))
			return rc;
		txconfig->testmode_enabled = true;
		return 0;
	}
	/* Back to normal mode */
	rc = dw3000_repeated_cw(dw, false, 0);
	if (unlikely(rc))
		return rc;
	txconfig->testmode_enabled = false;
	return 0;
}

static int dw3000_writetxfctrl(struct dw3000 *dw, u16 txFrameLength,
			       u16 txBufferOffset, bool ranging)
{
	struct dw3000_local_data *local = &dw->data;
	u32 fctrl =
		txFrameLength |
		((u32)txBufferOffset << DW3000_TX_FCTRL_TXB_OFFSET_BIT_OFFSET) |
		((u32)ranging << DW3000_TX_FCTRL_TR_BIT_OFFSET);
	int rc;
	/*
	 * Compare with the previous value stored if register access
	 * is necessary.
	 */
	if (local->tx_fctrl == fctrl)
		return 0;
	/* Write the frame length to the TX frame control register */
	rc = dw3000_reg_modify32(dw, DW3000_TX_FCTRL_ID, 0,
				 ~(u32)(DW3000_TX_FCTRL_TXB_OFFSET_BIT_MASK |
					DW3000_TX_FCTRL_TR_BIT_MASK |
					DW3000_TX_FCTRL_TXFLEN_BIT_MASK),
				 fctrl);
	if (unlikely(rc))
		return rc;
	local->tx_fctrl = fctrl;
	return 0;
}

/** dw3000_write_txctrl - Runtime configuration of TX parameters
 * @dw: the DW device
 *
 * This function is called before packet transmission in order to set TX
 * parameters (pgdelay, channel, pulse shape) according to the current antenna.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_write_txctrl(struct dw3000 *dw)
{
	struct dw3000_txconfig *txconfig = &dw->txconfig;
	struct dw3000_config *config = &dw->config;
	u32 txctrl;

	/* Get default values and insert wanted pgdelay */
	txctrl = config->chan == 9 ? DW3000_RF_TXCTRL_CH9 :
				     DW3000_RF_TXCTRL_CH5;
	txctrl = (txctrl & ~DW3000_TX_CTRL_HI_TX_PG_DELAY_BIT_MASK) |
		 (txconfig->PGdly & DW3000_TX_CTRL_HI_TX_PG_DELAY_BIT_MASK);

	/* Configure pulse shape */
	if (config->alternate_pulse_shape) {
		txctrl |= DW3000_TX_CTRL_HI_TX_PULSE_SHAPE_BIT_MASK;
	} else {
		txctrl &= ~DW3000_TX_CTRL_HI_TX_PULSE_SHAPE_BIT_MASK;
	}

	return dw3000_reg_write32(dw, DW3000_TX_CTRL_HI_ID, 0, txctrl);
}

/**
 * dw3000_setrxaftertxdelay() - Set time Wait-for-Response Time
 * @dw: the DW device
 * @rx_delay_time: Wait-for-Response time in units of approx 1us
 * or 128 system clock cycles
 *
 * It is used to configure the turn-around time between TX complete and RX
 * enable when the wait for response function is being used.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_setrxaftertxdelay(struct dw3000 *dw, u32 rx_delay_time)
{
	struct dw3000_local_data *local = &dw->data;
	int rc;

	if (unlikely(rx_delay_time > DW3000_ACK_RESP_WAIT4RESP_TIM_BIT_MASK))
		return -EINVAL;
	if (local->w4r_time == rx_delay_time)
		return 0;
	rc = dw3000_reg_write32(
		dw, DW3000_ACK_RESP_ID, 0,
		local->ack_time << DW3000_ACK_RESP_ACK_TIM_BIT_OFFSET |
			rx_delay_time);
	if (unlikely(rc))
		return rc;
	local->w4r_time = rx_delay_time;
	return 0;
}

/**
 * dw3000_tx_frame() - prepare, execute or program TX
 * @dw: the DW device
 * @skb: TX socket buffer
 * @tx_delayed: true if TX delayed must be use
 * @tx_date_dtu: the date at which TX must be executed
 * @rx_delay_dly: positive if needed to active RX after TX otherwise null
 * @rx_timeout_pac: the preamble detect timeout
 * @ranging: true if transmitting ranging frame
 *
 * This function prepares, executes or programs TX according to a given socket
 * buffer pointer provided by the MCPS.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_tx_frame(struct dw3000 *dw, struct sk_buff *skb, bool tx_delayed,
		    u32 tx_date_dtu, int rx_delay_dly, u32 rx_timeout_pac,
		    bool ranging)
{
	u32 cur_time_dtu = 0;
	int rc, len;
	u8 cmd;

	/* Print the transmitted frame in hexadecimal characters */
	if (unlikely(DEBUG)) {
		if (skb)
			print_hex_dump_bytes(
				"dw3000: ieee802154: transmitted frame:",
				DUMP_PREFIX_NONE, skb->data, skb->len);
		else
			dev_dbg(dw->dev,
				"dw3000: ieee802154: transmitted frame without data");
	}

	/* Read current DTU time to compare with next transfer time */
	if (dw->coex_gpio >= 0)
		cur_time_dtu = dw3000_get_dtu_time(dw);

	/* Activate RX after TX ? */
	if (rx_delay_dly >= 0) {
		rc = dw3000_setrxaftertxdelay(dw, rx_delay_dly);
		if (unlikely(rc))
			return rc;
		rc = dw3000_setpreambledetecttimeout(dw, rx_timeout_pac);
		if (unlikely(rc))
			return rc;
	}

	if (skb) {
		len = skb->len + IEEE802154_FCS_LEN;
		/* Write frame properties to the transmit frame control register */
		if (WARN_ON(len > dw->data.max_frames_len))
			return -EINVAL;
		/* Write frame data to the DW IC buffer */
		if (dw3000_tx_write_data(dw, skb->data, skb->len, 0) != 0) {
			dev_err(dw->dev, "cannot write frame data to DW IC\n");
			return -EINVAL;
		}
	} else
		len = 0;

	if (dw->txconfig.smart)
		dw3000_adjust_tx_power(dw, len);

	rc = dw3000_writetxfctrl(dw, len, 0, ranging);
	if (unlikely(rc))
		return rc;

	rc = dw3000_write_txctrl(dw);
	if (unlikely(rc))
		return rc;

	/* Update TX parameters according to Wifi coexistence */
	rc = dw3000_coex_start(dw, &tx_delayed, &tx_date_dtu, cur_time_dtu);
	if (unlikely(rc))
		return rc;

	if (!tx_delayed) {
		/* Program immediate transmission. */
		cmd = rx_delay_dly >= 0 ? DW3000_CMD_TX_W4R : DW3000_CMD_TX;
		rc = dw3000_write_fastcmd(dw, cmd);
		if (unlikely(rc))
			goto stop_coex;
		/* W4R mode are handled by TX event IRQ handler */
		dw3000_power_stats(dw, DW3000_PWR_TX,
				   skb ? skb->len + IEEE802154_FCS_LEN : 0);
		return 0;
	}

	/* Set transmission date. */
	rc = dw3000_setdelayedtrxtime(dw, tx_date_dtu);
	if (unlikely(rc))
		goto stop_coex;
	/* Program delayed transmission. */
	cmd = rx_delay_dly >= 0 ? DW3000_CMD_DTX_W4R : DW3000_CMD_DTX;
	rc = dw3000_write_fastcmd(dw, cmd);
	if (unlikely(rc))
		goto stop_coex;
	/* W4R mode are handled by TX event IRQ handler */
	dw3000_power_stats(dw, DW3000_PWR_TX,
			   skb ? skb->len + IEEE802154_FCS_LEN : 0);
	/* Check if late */
	rc = dw3000_check_hpdwarn(dw);
	if (unlikely(rc)) {
		if (rc == -ETIME) {
			cur_time_dtu = dw3000_get_dtu_time(dw);
			dev_err(dw->dev,
				"cannot program delayed tx date_dtu=%x current_dtu=%x\n",
				tx_date_dtu, cur_time_dtu);
			/* dw3000_forcetrxoff() already stop coex, return now */
			return rc;
		}
		goto stop_coex;
	}
	return 0;
stop_coex:
	dw3000_coex_stop(dw);
	return rc;
}

/**
 * dw3000_do_tx_frame() - handle TX frame MCPS operation
 * @dw: the device on which transmit frame
 * @info: TX parameters from MCPS
 * @skb: the frame to transmit
 * @frame_idx: Frame index in a continuous block
 *
 * This function is called to execute all required operation to transmit the
 * given frame using provided parameters.
 *
 * Since TX may be deferred until device is wokenup, it may be also called a
 * second time with same parameters from the wakeup ISR.
 *
 * This function calls dw3000_check_operational_state() and save and defer
 * the operation if DEEP-SLEEP is possible according delay.
 *
 * If TX frame must be done immediately, this function configure STS, PDOA,
 * Antenna pair, and delay for auto-RX before sending SKB by calling
 * dw3000_tx_frame().
 *
 * Return: 0 on success, else a negative error code.
 */
int dw3000_do_tx_frame(struct dw3000 *dw,
		       const struct mcps802154_tx_frame_info *info,
		       struct sk_buff *skb, int frame_idx)
{
	struct dw3000_deep_sleep_state *dss = &dw->deep_sleep_state;
	struct mcps802154_llhw *llhw = dw->llhw;
	u32 cur_time_dtu = 0;
	u32 tx_date_dtu = 0;
	int rx_delay_dly = -1;
	u32 rx_timeout_pac = 0;
	bool tx_delayed = true;
	bool ranging = false;
	int delay_dtu = 0;
	bool can_sync = false;
	int rc;
	u8 sts_mode;

	trace_dw3000_mcps_tx_frame(dw, info->flags, skb ? skb->len : 0);

	/* Calculate the transfer date.*/
	if (info->flags & MCPS802154_TX_FRAME_TIMESTAMP_DTU) {
		tx_date_dtu = info->timestamp_dtu + llhw->shr_dtu;
	} else {
		/* Send immediately. */
		tx_delayed = false;
	}
	if (info->flags & MCPS802154_TX_FRAME_RANGING)
		ranging = true;

	if (tx_delayed) {
		cur_time_dtu = dw3000_get_dtu_time(dw);
		delay_dtu = (int)(tx_date_dtu - cur_time_dtu);
		if (delay_dtu < 0) {
			dev_err(dw->dev,
				"too late to program delayed tx date_dtu=%x current_dtu=%x\n",
				tx_date_dtu, cur_time_dtu);
			return -ETIME;
		}
	}

	/* We are always allowed to sleep & sync at the beginning of a block. */
	if (!frame_idx) {
		dw->need_ranging_clock = false;
		can_sync = true;
	}

	/* For delayed TX, where delay_dtu != 0, enter/leave deep sleep */
	rc = dw3000_check_operational_state(dw, delay_dtu, can_sync);
	if (rc) {
		/* Handle error cases first */
		if (rc < 0)
			return rc;
		/* Save parameters to activate TX delayed when
		   wakeup later */
		dss->next_operational_state = DW3000_OP_STATE_TX;
		dss->tx_info = *info;
		dss->tx_skb = skb;
		dss->frame_idx = frame_idx;
		return 0;
	}
	/* All operation below require the DW chip is in IDLE_PLL state */

	/* Enable STS */
	sts_mode = FIELD_GET(MCPS802154_TX_FRAME_STS_MODE_MASK, info->flags);
	rc = dw3000_set_sts_pdoa(
		dw, sts_mode,
		sts_to_pdoa(sts_mode,
			    info->flags & MCPS802154_TX_FRAME_RANGING_PDOA));
	if (unlikely(rc))
		goto fail;
	/* Ensure correct TX antenna is selected. */
	rc = dw3000_set_tx_antenna(dw, info->ant_id);
	if (unlikely(rc))
		goto fail;

	if (info->rx_enable_after_tx_dtu > 0) {
		/* Disable auto-ack if it was previously enabled. */
		dw3000_disable_autoack(dw, false);
		/* Calculate the after tx rx delay. */
		rx_delay_dly = dtu_to_dly(llhw, info->rx_enable_after_tx_dtu) -
			       DW3000_RX_ENABLE_STARTUP_DLY;
		rx_delay_dly = rx_delay_dly >= 0 ? rx_delay_dly : 0;
		/* Calculate the after tx rx timeout. DO nothing if no timeout */
		if (info->rx_enable_after_tx_timeout_dtu == 0) {
			rx_timeout_pac = dw->pre_timeout_pac;
		} else if (info->rx_enable_after_tx_timeout_dtu != -1) {
			rx_timeout_pac =
				dw->pre_timeout_pac +
				dtu_to_pac(
					llhw,
					info->rx_enable_after_tx_timeout_dtu);
		}
	}
	rc = dw3000_tx_frame(dw, skb, tx_delayed, tx_date_dtu, rx_delay_dly,
			     rx_timeout_pac, ranging);
	if (unlikely(rc))
		goto fail;

	/* Store ranging clock requirement for next operation */
	dw->need_ranging_clock =
		(info->flags & MCPS802154_TX_FRAME_KEEP_RANGING_CLOCK) != 0;
fail:
	trace_dw3000_return_int(dw, rc);
	return rc;
}

static int dw3000_rx_read_data(struct dw3000 *dw, u8 *buffer, u16 len,
			       u16 offset)
{
	u32 rx_buff_addr;

	/* If the flag is 0x4 we are reading from RX_BUFFER_B */
	if (dw->data.dblbuffon == DW3000_DBL_BUFF_ACCESS_BUFFER_B) {
		rx_buff_addr = DW3000_RX_BUFFER_B_ID;
		/* Reading from RX_BUFFER_A - also when non-double buffer mode */
	} else {
		rx_buff_addr = DW3000_RX_BUFFER_A_ID;
	}
	if ((offset + len) > DW3000_RX_BUFFER_MAX_LEN)
		return -EINVAL;
	if (offset <= DW3000_REG_DIRECT_OFFSET_MAX_LEN) {
		/* Directly read data from the IC to the buffer */
		return dw3000_xfer(dw, rx_buff_addr, offset, len, buffer,
				   DW3000_SPI_RD_BIT);
	} else {
		/* Program the indirect offset registers B
		* for specified offset to RX buffer
		*/
		dw3000_reg_write32(dw, DW3000_INDIRECT_ADDR_A_ID, 0,
				   (rx_buff_addr >> 16));
		dw3000_reg_write32(dw, DW3000_ADDR_OFFSET_A_ID, 0, offset);

		/* Indirectly read data from the IC to the buffer */
		return dw3000_xfer(dw, DW3000_INDIRECT_POINTER_A_ID, 0, len,
				   buffer, DW3000_SPI_RD_BIT);
	}
}

static int dw3000_rx_frame(struct dw3000 *dw,
			   const struct dw3000_isr_data *data)
{
	struct dw3000_rx *rx = &dw->rx;
	size_t len = data->datalength;
	struct sk_buff *skb;
	unsigned long flags;
	u8 *buffer;
	int rc;

	/* Read frame data into skb */
	if (len) {
		/* Allocate new skb (including space for FCS added by ieee802154_rx) */
		skb = dev_alloc_skb(len + IEEE802154_FCS_LEN);
		if (!skb) {
			dev_err(dw->dev, "RX buffer allocation failed\n");
			return -ENOMEM;
		}
		buffer = skb_put(skb, len);
		/* Directly read data from the IC to the buffer */
		rc = dw3000_rx_read_data(dw, buffer, len, 0);
		if (rc)
			goto err_spi;
	} else {
		/* SP3 case: Frame with STS and without data */
		skb = NULL;
	}
	/* Store received frame */
	spin_lock_irqsave(&rx->lock, flags);
	WARN_ON(rx->skb);
	rx->skb = skb;
	rx->flags = data->rx_flags | (skb ? 0 : DW3000_RX_FLAG_ND);
	rx->ts_rctu = data->ts_rctu;
	spin_unlock_irqrestore(&rx->lock, flags);
	/* Print the received frame in hexadecimal characters */
	if (unlikely(DEBUG)) {
		dev_dbg(dw->dev, "frame info: len=%lu, rxflags=0x%.2x", len,
			data->rx_flags);
		if (skb)
			print_hex_dump_bytes("dw3000: frame data: ",
					     DUMP_PREFIX_NONE, skb->data, len);
	}
	/* Inform MCPS 802.15.4 that we received a frame */
	mcps802154_rx_frame(dw->llhw);
	return 0;

err_spi:
	/* Release the socker buffer */
	if (skb)
		dev_kfree_skb_any(skb);
	return rc;
}

/**
 * dw3000_set_interrupt() - Select the interruption's events to mask or unmask
 * @dw: the DW device
 * @bitmask: bitmap of selected events
 * @opt:
 *  - DW3000_DISABLE_INT: unmask the selected events
 *  - DW3000_ENABLE_INT: mask the selected events
 *  - DW3000_ENABLE_INT_ONLY: mask the selected events and unmask the rest
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_set_interrupt(struct dw3000 *dw, u32 bitmask,
				const enum int_options opt)
{
	if (opt == DW3000_ENABLE_INT_ONLY) {
		/* Overriding */
		return dw3000_reg_write32(dw, DW3000_SYS_ENABLE_LO_ID, 0,
					  bitmask);
	} else {
		if (opt == DW3000_ENABLE_INT) {
			/* Set the bits */
			return dw3000_reg_or32(dw, DW3000_SYS_ENABLE_LO_ID, 0,
					       bitmask);
		} else {
			/* Clear the bits */
			return dw3000_reg_and32(dw, DW3000_SYS_ENABLE_LO_ID, 0,
						(u32)(~bitmask));
		}
	}
}

/**
 * dw3000_setplenfine() - Configure frame preamble length
 * @dw: the DW device
 * @preamble_len: length of the preamble
 *
 * A preamble_len value of 0 disables this setting and the length of the frame
 * will be dependent on the TXPSR_PE setting as configured
 * by dw3000_configure() function.
 * The frame premable length can be configured in steps of 8, from 16
 * to 2048 symbols. If a non-zero value is configured, then the TXPSR_PE
 * setting is ignored.
 *
 * Return: zero on success, else a negative error code.
 */
static inline int dw3000_setplenfine(struct dw3000 *dw, u8 preamble_len)
{
	return dw3000_reg_write8(dw, DW3000_TX_FCTRL_HI_ID, 1, preamble_len);
}

/**
 * _dw3000_get_sts_mnth() - Calculate the adjusted STS minimum threshold
 * @cipher: the STS length's factor
 * @threshold: the STS default threshold
 * @shift_val: shift value for adjustment
 *
 * Return: the value of the adjusted STS minimum threshold.
 */
static u16 _dw3000_get_sts_mnth(u16 cipher, u8 threshold, u8 shift_val)
{
	u32 value = cipher * (u32)threshold;

	if (shift_val == 3) {
		/* Factor to sqrt(2) */
		value *= DW3000_SQRT2_FACTOR;
		value >>= DW3000_SQRT2_SHIFT_VAL;
	}
	/* Round the result of the shift by 11 (or division by 2048) */
	return (u16)((value + DW3000_STS_MNTH_ROUND_SHIFT) >>
		     DW3000_STS_MNTH_SHIFT);
}

/**
 * _dw3000_sts_is_enabled() - Return true if STS is enabled
 * @dw: the DW device
 *
 * Return: true is STS is enable.
 */
static inline bool _dw3000_sts_is_enabled(struct dw3000 *dw)
{
	return (dw->config.stsMode & DW3000_STS_BASIC_MODES_MASK) !=
	       DW3000_STS_MODE_OFF;
}

/**
 * dw3000_configure_otp() - set device's OTP configuration
 * @dw: the DW device
 * @config: the DW device's configuration
 *
 * This function depend on stsLength if STS is enabled, so require
 * stsLength is set before stsMode is updated.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_configure_otp(struct dw3000 *dw, struct dw3000_config *config)
{
	u16 preamble_len = _plen_info[config->txPreambLength - 1].symb;
	/* Update the preamble length regarding STS mode */
	if (_dw3000_sts_is_enabled(dw))
		preamble_len +=
			DW3000_GET_STS_LEN_UNIT_VALUE(config->stsLength) * 8;
	/* Configure gearing tables for non-SCP mode */
	if (preamble_len >= 256) {
		dw->data.sleep_mode |= DW3000_ALT_GEAR | DW3000_SEL_GEAR0;
		return dw3000_reg_modify32(
			dw, DW3000_NVM_CFG_ID, 0,
			~(DW3000_NVM_CFG_GEAR_ID_BIT_MASK),
			DW3000_OPSET_LONG | DW3000_NVM_CFG_GEAR_KICK_BIT_MASK);
	} else {
		return dw3000_reg_modify32(
			dw, DW3000_NVM_CFG_ID, 0,
			~(DW3000_NVM_CFG_GEAR_ID_BIT_MASK),
			DW3000_OPSET_SHORT | DW3000_NVM_CFG_GEAR_KICK_BIT_MASK);
	}
}

/**
 * dw3000_configure_sts() - set device's STS configuration
 * @dw: the DW device
 * @config: the DW device's configuration
 *
 * STS Minimum Threshold `sts_mnth` needs to be adjusted with changing
 * STS length. To adjust the `sts_mnth` following formula can be used:
 *
 *   sts_mnth = sqrt(x/y)*default_sts_mnth
 *
 *   where:
 *	 - `default_sts_mnth` is 0x10
 *       - `x` is the length of the STS in units of 8  (i.e. 8 for 64 length,
 *         16 for 128 length etc..)
 *       - `y` is either 8 or 16, 8 when no PDOA or PDOA mode 1 and 16
 *         for PDOA mode 3.
 *
 * The API does not use the formula and the `sts_mnth` value is derived
 * from approximation formula as given by _dw3000_get_sts_mnth() function.
 * The API here supports STS lengths as listed in `dw3000_sts_lengths enum`,
 * which are 32, 64, 128, 256, 512, 1024 and 2048.
 * The enum value is used as the index into`dw3000_sts_length_factors` array.
 * The array has values which are generated by:
 *
 *   val = sqrt(stsLength/16)*2048
 *
 *   where:
 *	 - `stsLength` value of the given `dw3000_sts_lengths enum`
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_configure_sts(struct dw3000 *dw, struct dw3000_config *config)
{
	u16 sts_mnth;

	if (!_dw3000_sts_is_enabled(dw))
		return 0;
	/* Update default minimum STS threshold according STS length. */
	dw->data.ststhreshold = (s16)DW3000_STSQUAL_THRESH_64(
		(u32)(DW3000_GET_STS_LEN_UNIT_VALUE(config->stsLength)));
	/* Configure CIA STS lower bound */
	if ((config->pdoaMode == DW3000_PDOA_M1) ||
	    (config->pdoaMode == DW3000_PDOA_M0)) {
		/**
		 * In PDOA mode 1, number of accumulated symbols
		 * is the whole length of the STS
		 */
		sts_mnth = _dw3000_get_sts_mnth(
			dw3000_sts_length_factors[(u8)(config->stsLength)],
			DW3000_CIA_MANUALLOWERBOUND_TH_64, 3);
	} else {
		/**
		 * In PDOA mode 3 number of accumulated symbols
		 * is half of the length of STS symbols
		 */
		sts_mnth = _dw3000_get_sts_mnth(
			dw3000_sts_length_factors[(u8)(config->stsLength)],
			DW3000_CIA_MANUALLOWERBOUND_TH_64, 4);
	}
	/* TODO: put register value in cache */
	return dw3000_reg_modify16(
		dw, DW3000_CY_CONFIG_LO_ID, 2,
		(u16) ~(DW3000_CY_CONFIG_LO_MANUALLOWERBOUND_BIT_MASK >> 16),
		sts_mnth & 0x7F);
}

/**
 * _swap128() - swap bytes of a 128 bits unaligned buffer
 * @dst: the in-stack 64 bits aligned 128 bits destination buffer
 * @src: the 128 bits buffer with MSB first to swap
 */
static inline void _swap128(__le64 *dst, const void *src)
{
	u64 val = get_unaligned_be64(src);
	dst[1] = cpu_to_le64(val);
	val = get_unaligned_be64(src + sizeof(u64));
	dst[0] = cpu_to_le64(val);
}

/**
 * dw3000_configure_sts_key() - set device's STS KEY
 * @dw: the DW device
 * @key: the 128 bits STS KEY to configure (MSB first)
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_configure_sts_key(struct dw3000 *dw, const u8 *key)
{
	static const u8 nul_key[AES_KEYSIZE_128] = {
		0,
	};
	/* TODO: Transfer the STS key securely to DW3000 using the AES engine.
	 * See 5.8.4.3 Decrypt STS KEY into STS KEY registers. */
	struct dw3000_local_data *data = &dw->data;
	struct dw3000_config *config = &dw->config;
	bool changed;
	bool is_nul_key;
	int rc;
	/* Check NUL key */
	is_nul_key = memcmp(key, nul_key, AES_KEYSIZE_128) == 0;
	if (is_nul_key) {
		/* DW3000 specific. Use Super Deterministic Code if NUL key is
		   set. Check if change required. */
		changed = (config->stsMode & DW3000_STS_MODE_SDC) == 0;
		/* No need to send key. */
	} else {
		bool kc = true;
		/* Need to remove SDC flag. Check if change is required. */
		changed = (config->stsMode & DW3000_STS_MODE_SDC) != 0;
		/* Update Key. */
		if (!changed)
			kc = memcmp(key, data->sts_key, AES_KEYSIZE_128) != 0;
		if (kc) {
			__le64 swapped_key[AES_KEYSIZE_128 / sizeof(__le64)];
			_swap128(swapped_key, key);
			/* Key has changed, update it. */
			rc = _dw3000_reg_write(dw, DW3000_STS_KEY_ID, 0,
					       AES_KEYSIZE_128, swapped_key);
			if (rc)
				return rc;
			/* Update cached key. */
			memcpy(data->sts_key, key, AES_KEYSIZE_128);
		}
	}
	if (changed) {
		/* SDC bit had changed. Sync register. */
		rc = dw3000_reg_modify8(
			dw, DW3000_SYS_CFG_ID, 1,
			(u8)(~DW3000_SYS_CFG_CP_SDC_BIT_MASK >> 8),
			is_nul_key ? (DW3000_SYS_CFG_CP_SDC_BIT_MASK >> 8) : 0);
		if (rc)
			return rc;
		/* Finally, save configured STS mode. */
		if (is_nul_key)
			config->stsMode |= DW3000_STS_MODE_SDC;
		else
			config->stsMode &= ~DW3000_STS_MODE_SDC;
	}
	return 0;
}

/**
 * dw3000_configure_sts_iv() - set device's STS IV
 * @dw: the DW device
 * @iv: the 128 bits STS IV to configure (MSB first)
 *
 * The four least significant bytes are always sent.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_configure_sts_iv(struct dw3000 *dw, const u8 *iv)
{
	struct dw3000_local_data *data = &dw->data;
	__le64 swapped_iv[AES_BLOCK_SIZE / sizeof(__le64)];
	bool changed;
	int rc;
	/* Check if IV MSB had changed. */
	changed = memcmp(iv, data->sts_iv, AES_BLOCK_SIZE - sizeof(u32)) != 0;
	/* Convert to Little-endian. */
	_swap128(swapped_iv, iv);
	/* Update it (Only reset counter in LSB if unchanged). */
	rc = _dw3000_reg_write(dw, DW3000_STS_IV_ID, 0,
			       changed ? AES_BLOCK_SIZE : sizeof(u32),
			       swapped_iv);
	if (rc)
		return rc;
	/* Update cached IV. */
	memcpy(data->sts_iv, iv, AES_KEYSIZE_128);
	return 0;
}

/**
 * dw3000_load_sts_iv() - force device's STS IV to be loaded
 * @dw: the DW device
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_load_sts_iv(struct dw3000 *dw)
{
	return dw3000_reg_or8(dw, DW3000_STS_CTRL_ID, 0,
			      DW3000_STS_CTRL_LOAD_IV_BIT_MASK);
}

/**
 * _dw3000_ciadiag_update_reg_select() - Update CIA diagnostic register selector
 * @dw: the DW device
 *
 * According to DW3000's configuration, we must read some values
 * (e.g: channel impulse response power, preamble accumulation count)
 * in different registers in the CIA interface. It on depending the STS
 * and PDOA configurations.
 */
static void _dw3000_ciadiag_update_reg_select(struct dw3000 *dw)
{
	struct dw3000_config *config = &dw->config;
	struct dw3000_local_data *data = &dw->data;

	if (config->pdoaMode == DW3000_PDOA_M3) {
		data->ciadiag_reg_select =
			DW3000_CIA_DIAG_REG_SELECT_WITH_PDAO_M3;
	} else {
		if (_dw3000_sts_is_enabled(dw))
			data->ciadiag_reg_select =
				DW3000_CIA_DIAG_REG_SELECT_WITH_STS;
		else
			data->ciadiag_reg_select =
				DW3000_CIA_DIAG_REG_SELECT_WITHOUT_STS;
	}
}

static int dw3000_configure_sys_cfg(struct dw3000 *dw,
				    struct dw3000_config *config)
{
	u8 mode = (config->phrMode == DW3000_PHRMODE_EXT) ?
			  DW3000_SYS_CFG_PHR_MODE_BIT_MASK :
			  0;
	u8 dw_pac_reg = _plen_info[config->txPreambLength - 1].dw_pac_reg;
	int rc;
	/*
	 * SYS_CFG :
	 * - Clear the PHR Mode, PHR Rate, STS Protocol, SDC, PDOA Mode,
	 * - Set the relevant bits according to configuration of the PHR Mode,
	 *   PHR Rate, STS Protocol, SDC, PDOA Mode
	 */
	rc = dw3000_reg_modify32(
		dw, DW3000_SYS_CFG_ID, 0,
		~(u32)(DW3000_SYS_CFG_PHR_MODE_BIT_MASK |
		       DW3000_SYS_CFG_PHR_6M8_BIT_MASK |
		       DW3000_SYS_CFG_CP_PROTOCOL_BIT_MASK |
		       DW3000_SYS_CFG_PDOA_MODE_BIT_MASK |
		       DW3000_SYS_CFG_CP_SDC_BIT_MASK),
		(((u32)config->pdoaMode)
		 << DW3000_SYS_CFG_PDOA_MODE_BIT_OFFSET) |
			(((u16)config->stsMode & DW3000_STS_CONFIG_MASK)
			 << DW3000_SYS_CFG_CP_PROTOCOL_BIT_OFFSET) |
			(DW3000_SYS_CFG_PHR_6M8_BIT_MASK &
			 ((u32)config->phrRate
			  << DW3000_SYS_CFG_PHR_6M8_BIT_OFFSET)) |
			mode);
	if (rc)
		return rc;
	/* Update CIA diagnostic register selection */
	_dw3000_ciadiag_update_reg_select(dw);
	/* Configure OTP */
	rc = dw3000_configure_otp(dw, config);
	if (rc)
		return rc;
	/* Configure PAC */
	rc = dw3000_reg_modify8(dw, DW3000_DRX_TUNE0_ID, 0,
				(u8)~DW3000_DRX_TUNE0_PRE_PAC_SYM_BIT_MASK,
				dw_pac_reg);
	if (rc)
		return rc;
	if (config->txPreambLength == DW3000_PLEN_72) {
		/**
		 * Value 9 sets fine premable length to 72 symbols
		 * This is needed to set 72 length.
		 */
		rc = dw3000_setplenfine(dw, 9);
		if (rc)
			return rc;
	} else {
		/* Clear the setting in the FINE_PLEN register. */
		rc = dw3000_setplenfine(dw, 0);
		if (rc)
			return rc;
	}
	if ((config->stsMode & DW3000_STS_MODE_ND) == DW3000_STS_MODE_ND) {
		/**
		 * Configure lower preamble detection threshold for no data
		 * STS mode.
		 */
		return dw3000_reg_write32(dw, DW3000_DRX_TUNE3_ID, 0,
					  DW3000_PD_THRESH_NO_DATA);
	} else {
		/**
		 * Configure default preamble detection threshold for other
		 * modes.
		 */
		return dw3000_reg_write32(dw, DW3000_DRX_TUNE3_ID, 0,
					  DW3000_PD_THRESH_DEFAULT);
	}
}

/**
 * dw3000_configure_chan_ctrl() - configure the channel control register
 * @dw: the DW device
 * @config: the DW device's configuration
 *
 * Set the DW3000_CHAN_CTRL_ID register.
 *
 * Return: zero on success, else a negative error code.
 */
static inline int dw3000_configure_chan_ctrl(struct dw3000 *dw,
					     struct dw3000_config *config)
{
	u8 chan = config->chan;
	u32 temp;
	int rc;

	/* Adjust configuration according channel/txCode and calib_data */
	dw3000_calib_update_config(dw);

	/* Read current CHAN_CTRL register value*/
	rc = dw3000_reg_read32(dw, DW3000_CHAN_CTRL_ID, 0, &temp);
	if (rc)
		return rc;
	/* Change value */
	temp &= (~(DW3000_CHAN_CTRL_RX_PCODE_BIT_MASK |
		   DW3000_CHAN_CTRL_TX_PCODE_BIT_MASK |
		   DW3000_CHAN_CTRL_SFD_TYPE_BIT_MASK |
		   DW3000_CHAN_CTRL_RF_CHAN_BIT_MASK));
	if (chan == 9)
		temp |= DW3000_CHAN_CTRL_RF_CHAN_BIT_MASK;
	temp |= (DW3000_CHAN_CTRL_RX_PCODE_BIT_MASK &
		 ((u32)config->rxCode << DW3000_CHAN_CTRL_RX_PCODE_BIT_OFFSET));
	temp |= (DW3000_CHAN_CTRL_TX_PCODE_BIT_MASK &
		 ((u32)config->txCode << DW3000_CHAN_CTRL_TX_PCODE_BIT_OFFSET));
	temp |= (DW3000_CHAN_CTRL_SFD_TYPE_BIT_MASK &
		 ((u32)config->sfdType
		  << DW3000_CHAN_CTRL_SFD_TYPE_BIT_OFFSET));
	/* Reprogram new value */
	return dw3000_reg_write32(dw, DW3000_CHAN_CTRL_ID, 0, temp);
}

/**
 * dw3000_configure_rf() - configure the device's radio frequency
 * @dw: the DW device
 *
 * According to the channel (ex: 5 or 9), it sets the device's configuration
 * of the radio frequency.
 *
 * Return: zero on success, else a negative error code.
 */
static inline int dw3000_configure_rf(struct dw3000 *dw)
{
	struct dw3000_config *config = &dw->config;
	struct dw3000_txconfig *txconfig = &dw->txconfig;
	u8 chan = config->chan;
	u32 rf_pll_cfg;
	int rc;
	/* Get default values */

	if (chan == 9) {
		rf_pll_cfg = DW3000_RF_PLL_CFG_CH9;
	} else {
		rf_pll_cfg = DW3000_RF_PLL_CFG_CH5;
	}
	rc = dw3000_write_txctrl(dw);
	if (rc)
		return rc;

	rc = dw3000_reg_write16(dw, DW3000_PLL_CFG_ID, 0, rf_pll_cfg);
	if (rc)
		return rc;
	rc = dw3000_reg_write8(dw, DW3000_LDO_RLOAD_ID, 1,
			       DW3000_LDO_RLOAD_VAL_B1);
	if (rc)
		return rc;
	rc = dw3000_reg_write8(dw, DW3000_TX_CTRL_LO_ID, 2,
			       DW3000_RF_TXCTRL_LO_B2);
	if (rc)
		return rc;
	/* Setup TX power */
	rc = dw3000_reg_write32(dw, DW3000_TX_POWER_ID, 0, txconfig->power);

	if (unlikely(rc))
		return rc;

	/* Extend the lock delay */
	rc = dw3000_reg_write8(dw, DW3000_PLL_CAL_ID, 0, DW3000_RF_PLL_CFG_LD);
	if (unlikely(rc))
		return rc;

	/* Configure PLL coarse code, if needed. */
	if (dw->chip_ops->pll_coarse_code) {
		rc = dw->chip_ops->pll_coarse_code(dw);
		if (rc)
			return rc;
	}
	return 0;
}

static int dw3000_configmrxlut(struct dw3000 *dw)
{
	struct dw3000_config *config = &dw->config;
	u8 chan = config->chan;
	const u32 *lut;
	int rc;

	lut = dw->chip_ops->get_config_mrxlut_chan(dw, chan);
	if (!lut)
		return -EINVAL;
	/* Update LUT registers */
	rc = dw3000_reg_write32(dw, DW3000_DGC_CFG_ID, 0x0, 0x19c7f1e5);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_LUT_0_CFG_ID, 0x0, lut[0]);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_LUT_1_CFG_ID, 0x0, lut[1]);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_LUT_2_CFG_ID, 0x0, lut[2]);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_LUT_3_CFG_ID, 0x0, lut[3]);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_LUT_4_CFG_ID, 0x0, lut[4]);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_LUT_5_CFG_ID, 0x0, lut[5]);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_LUT_6_CFG_ID, 0x0, lut[6]);
	if (rc)
		return rc;
	rc = dw3000_reg_write32(dw, DW3000_DGC_CFG0_ID, 0x0, DW3000_DGC_CFG0);
	if (rc)
		return rc;

	return dw3000_reg_write32(dw, DW3000_DGC_CFG1_ID, 0x0, DW3000_DGC_CFG1);
}

int dw3000_configure_dgc(struct dw3000 *dw)
{
	struct dw3000_config *config = &dw->config;
	/* Only enable DGC for PRF 64. */
	if ((config->rxCode >= 9) && (config->rxCode <= 24)) {
		struct dw3000_local_data *local = &dw->data;
		int rc;
		/* Load RX LUTs */
		if (local->dgc_otp_set) {
			/* If the OTP has DGC info programmed into it, do a
			 * manual kick from OTP. */
			u16 dgc_sel = (config->chan == 5 ? 0 : 1)
				      << DW3000_NVM_CFG_DGC_SEL_BIT_OFFSET;
			rc = dw3000_reg_modify16(
				dw, DW3000_DGC_CFG_ID, 0,
				(u16) ~(DW3000_NVM_CFG_DGC_SEL_BIT_MASK),
				dgc_sel | DW3000_NVM_CFG_DGC_KICK_BIT_MASK);
			if (rc)
				return rc;
			/* Configure kick bits for when waking up. */
			local->sleep_mode |= DW3000_LOADDGC;
		} else {
			/* Else we manually program hard-coded values into the
			 * DGC registers. */
			rc = dw3000_configmrxlut(dw);
			if (rc)
				return rc;
			local->sleep_mode &= ~DW3000_LOADDGC;
		}
		return dw3000_reg_modify16(
			dw, DW3000_DGC_CFG_ID, 0x0,
			(u16)~DW3000_DGC_CFG_THR_64_BIT_MASK,
			DW3000_DGC_CFG << DW3000_DGC_CFG_THR_64_BIT_OFFSET);
	} else {
		return dw3000_reg_and8(dw, DW3000_DGC_CFG_ID, 0x0,
				       (u8)~DW3000_DGC_CFG_RX_TUNE_EN_BIT_MASK);
	}
}

static int dw3000_restore_dgc(struct dw3000 *dw)
{
	struct dw3000_config *config = &dw->config;
	int rc = 0;
	/* Only enable DGC for PRF 64. */
	if ((config->rxCode >= 9) && (config->rxCode <= 24)) {
		struct dw3000_local_data *local = &dw->data;
		/* Load RX LUTs only if not already loaded from OTP */
		if (!local->dgc_otp_set)
			rc = dw3000_configmrxlut(dw);
	}
	return rc;
}

/**
 * dw3000_configure_chan() - configure the device's RF channel
 * @dw: the DW device
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_configure_chan(struct dw3000 *dw)
{
	struct dw3000_config *config = &dw->config;
	int rc;
	/* Configure the CHAN_CTRL register */
	rc = dw3000_configure_chan_ctrl(dw, config);
	if (rc)
		return rc;
	/* Set TX/RX analogs for given channel */
	rc = dw3000_configure_rf(dw);
	if (rc)
		return rc;
	/* Configure DGC. */
	return dw3000_configure_dgc(dw);
}

/**
 * dw3000_configure_pcode() - change the device's RF preamble code
 * @dw: the DW device
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_configure_pcode(struct dw3000 *dw)
{
	/* Just reconfigure the CHAN_CTRL register */
	return dw3000_configure_chan_ctrl(dw, &dw->config);
}

static int dw3000_setdwstate(struct dw3000 *dw, enum operational_state state)
{
	int rc;
	if (state == dw->current_operational_state)
		return 0;

	switch (state) {
	case DW3000_OP_STATE_DEEP_SLEEP:
		/* Disable IRQ to ensure no IRQ storm in case pull-down is too weak */
		disable_irq(dw->spi->irq);

		/* Clear SPIRDY and RCINIT interrupts to avoid spurious SPIRDY and
		 * RCINIT interrupts (at startup, for example) that will trigger the
		 * wake up process of the chip too early.
		 */
		rc = dw3000_clear_sys_status(
			dw, DW3000_SYS_STATUS_SPIRDY_BIT_MASK |
				    DW3000_SYS_STATUS_RCINIT_BIT_MASK);
		if (rc)
			return rc;
		/* Enable the RCINIT interrupt for wake up. */
		rc = dw3000_reg_or8(dw, DW3000_SYS_ENABLE_LO_ID, 3,
				    DW3000_SYS_STATUS_RCINIT_BIT_MASK >> 24);
		if (rc)
			return rc;
		/* Store the current DTU  */
		dw->sleep_enter_dtu = dw3000_get_dtu_time(dw);
		trace_dw3000_deep_sleep_enter(dw, dw->sleep_enter_dtu);
		/**
		 * Set up the deep sleep configuration:
		 * Step 1:
		 * - enable the sleep configuration bit.
		 * - disable sleep counter to get deep sleep.
		 * - enable wake up using SPI access.
		 */
		rc = dw3000_reg_write8(dw, DW3000_AON_CFG_ID, 0,
				       DW3000_AON_SLEEP_EN_MASK |
					       DW3000_AON_WAKE_CSN_MASK);
		if (rc)
			return rc;
		/* Step 2: Set INIT2IDLE bit, to get DW3000_OP_STATE_IDLE_PLL on wakeup. */
		rc = dw3000_reg_or8(dw, DW3000_SEQ_CTRL_ID, 0x01,
				    DW3000_SEQ_CTRL_AUTO_INIT2IDLE_BIT_MASK >>
					    8);
		if (rc)
			return rc;
		/* Step 3:
		 * - enable configuration copy from AON memory to host registers.
		 * - set ONW_GO2IDLE to get DW3000_OP_STATE_IDLE_PLL on wakeup.
		 */
		rc = dw3000_reg_or16(
			dw, DW3000_AON_DIG_CFG_ID, 0,
			dw->data.sleep_mode |
				DW3000_AON_DIG_CFG_ONW_AONDLD_MASK |
				DW3000_AON_DIG_CFG_ONW_GO2IDLE_MASK);
		if (rc)
			return rc;
		/* Step 4: backup ALL registers to AON memory */
		rc = dw3000_reg_write8(dw, DW3000_AON_CTRL_ID, 0,
				       DW3000_AON_CTRL_ARRAY_UPLOAD_BIT_MASK);
		if (rc)
			return rc;
		/* Loop until backup is complete (bit has been reset) */
		do {
			u8 val;
			udelay(10);
			rc = dw3000_reg_read8(dw, DW3000_AON_CTRL_ID, 0, &val);
			if (rc)
				return rc;
			if (val & DW3000_AON_CTRL_ARRAY_UPLOAD_BIT_MASK)
				rc = -EAGAIN;
		} while (rc);
		/* Step 5: Upload the AON block configurations to the AON to go in DW3000_OP_STATE_DEEP_SLEEP. */
		rc = dw3000_reg_or8(dw, DW3000_AON_CTRL_ID, 0,
				    DW3000_AON_CTRL_CONFIG_UPLOAD_BIT_MASK);
		if (rc)
			return rc;
		/* Step 6: Here the chip is in DW3000_OP_STATE_DEEP_SLEEP.
		 * Update power statistics and operational state.
		 */
		dw3000_power_stats(dw, DW3000_PWR_DEEPSLEEP, 0);
		dw3000_set_operational_state(dw, DW3000_OP_STATE_DEEP_SLEEP);
		break;

	case DW3000_OP_STATE_IDLE_PLL:
		/**
		 * Set the auto INIT2IDLE bit so that DW3000 enters DW3000_OP_STATE_IDLE_PLL mode before
		 * switching clocks to system_PLL.
		 *
		 * NOTE: PLL should be configured prior to this, and the device
		 * should be in DW3000_OP_STATE_IDLE_RC (if the PLL does not lock device will
		 * remain in DW3000_OP_STATE_IDLE_RC)
		 *
		 * Switch clock to auto, if coming here from DW3000_OP_STATE_IDLE_RC the clock
		 * will be FOSC/4, need to switch to auto prior to setting auto
		 * INIT2IDLE bit
		 */
		rc = dw3000_force_clocks(dw, DW3000_FORCE_CLK_AUTO);
		if (rc)
			return rc;
		/* Calibrate the PLL from scratch, if needed. */
		if (dw->chip_ops->pll_calibration_from_scratch) {
			rc = dw->chip_ops->pll_calibration_from_scratch(dw);
			if (rc)
				return rc;
		}
		rc = dw3000_reg_or8(dw, DW3000_SEQ_CTRL_ID, 0x01,
				    DW3000_SEQ_CTRL_AUTO_INIT2IDLE_BIT_MASK >>
					    8);
		if (rc)
			return rc;
		dw3000_set_operational_state(dw, DW3000_OP_STATE_IDLE_PLL);
		break;

	case DW3000_OP_STATE_IDLE_RC:
		/**
		 * Change state to IDLE_RC and clear auto INIT2IDLE bit
		 * switch clock to FOSC
		 */
		rc = dw3000_reg_or8(dw, DW3000_CLK_CTRL_ID, 0,
				    DW3000_FORCE_SYSCLK_FOSC);
		if (rc)
			return rc;
		/* Clear the auto INIT2IDLE bit and set FORCE2INIT */
		rc = dw3000_reg_modify32(
			dw, DW3000_SEQ_CTRL_ID, 0x0,
			(u32)~DW3000_SEQ_CTRL_AUTO_INIT2IDLE_BIT_MASK,
			DW3000_SEQ_CTRL_FORCE2INIT_BIT_MASK);
		if (rc)
			return rc;
		/* Clear force bits (device will stay in IDLE_RC) */
		rc = dw3000_reg_and8(
			dw, DW3000_SEQ_CTRL_ID, 0x2,
			(u8) ~(DW3000_SEQ_CTRL_FORCE2INIT_BIT_MASK >> 16));
		if (rc)
			return rc;
		/* Switch clock to auto */
		rc = dw3000_force_clocks(dw, DW3000_FORCE_CLK_AUTO);
		if (rc)
			return rc;
		dw3000_set_operational_state(dw, DW3000_OP_STATE_IDLE_RC);
		break;

	case DW3000_OP_STATE_INIT_RC:
		/**
		 * The SPI rate needs to be <= 7MHz as device is switching
		 * to INIT_RC state
		 */
		rc = dw3000_reg_or8(dw, DW3000_CLK_CTRL_ID, 0,
				    DW3000_FORCE_SYSCLK_FOSCDIV4);
		if (rc)
			return rc;
		/* Clear the auto INIT2IDLE bit and set FORCE2INIT */
		rc = dw3000_reg_modify32(
			dw, DW3000_SEQ_CTRL_ID, 0x0,
			(u32)~DW3000_SEQ_CTRL_AUTO_INIT2IDLE_BIT_MASK,
			DW3000_SEQ_CTRL_FORCE2INIT_BIT_MASK);
		if (rc)
			return rc;
		/* Clear force bits (device will stay in IDLE_RC) */
		rc = dw3000_reg_and8(
			dw, DW3000_SEQ_CTRL_ID, 0x2,
			(u8) ~(DW3000_SEQ_CTRL_FORCE2INIT_BIT_MASK >> 16));
		if (rc)
			return rc;
		dw3000_set_operational_state(dw, DW3000_OP_STATE_INIT_RC);
		break;

	default:
		/* Invalid or unmanaged state */
		dev_err(dw->dev,
			"Invalid or unmanaged operational state %d, current state: %d\n",
			state, dw->current_operational_state);
		rc = -EINVAL;
		break;
	}
	return rc;
}

#ifdef CONFIG_DW3000_DEBUG
static unsigned dw3000_check_fileid = 0;
module_param_named(checkfid, dw3000_check_fileid, uint, 0644);
MODULE_PARM_DESC(checkfid,
		 "Selected register fileid to backup and check on wakeup");

static void dw3000_wakeup_compare(struct work_struct *work)
{
	struct dw3000 *dw = container_of(work, struct dw3000,
					 deep_sleep_state.compare_work);
	u32 *before = (u32 *)dw->deep_sleep_state.regbackup;
	u32 *after = (u32 *)(dw->deep_sleep_state.regbackup + 512);
	unsigned offset = dw3000_check_fileid << 16;
	const struct dw3000_chip_register *reg;
	size_t count;
	int length;
	/* retrieve registers length */
	reg = dw->chip_ops->get_registers(dw, &count);
	while (reg->address != dw3000_check_fileid && count) {
		reg++;
		count--;
	}
	length = (reg->size + 3) & ~4;
	dev_notice(dw->dev, "compare registers in fileid %u on wakeup\n",
		   dw3000_check_fileid);
	while (length > 0) {
		if (*after != *before)
			dev_notice(dw->dev, "0x%05x : 0x%x -> 0x%x\n", offset,
				   *before, *after);
		after++;
		before++;
		offset += 4;
		length -= 4;
	}
	return;
}

static int dw3000_backup_registers(struct dw3000 *dw, bool after)
{
	const struct dw3000_chip_register *reg;
	size_t count;
	int offset = after ? 512 : 0;
	int rc;

	reg = dw->chip_ops->get_registers(dw, &count);
	while (reg->address != dw3000_check_fileid && count) {
		reg++;
		count--;
	}

	rc = dw3000_xfer(dw, dw3000_check_fileid << 16, 0, reg->size,
			 dw->deep_sleep_state.regbackup + offset,
			 DW3000_SPI_RD_BIT);
	if (rc)
		return rc;

	/* Now compare and dump both parts */
	if (after)
		schedule_work(&dw->deep_sleep_state.compare_work);

	return 0;
}
#endif /* CONFIG_DW3000_DEBUG */

/**
 * dw3000_wakeup_timer_start() - Program wakeup timer
 * @dw: the DW device on which the SPI transfer will occurs
 * @delay_us: the delay after the DW device is woken up
 *
 * The timer is configured to wake-up the device after the specified delay.
 * Caller should take care of wake-up latency.
 *
 * Return: zero on success, else a negative error code.
 */
void dw3000_wakeup_timer_start(struct dw3000 *dw, int delay_us)
{
	hrtimer_start(&dw->deep_sleep_timer, ns_to_ktime(delay_us * 1000),
		      HRTIMER_MODE_REL);
	dw->call_timer_expired = true;
	trace_dw3000_wakeup_timer_start(dw, delay_us);
}

/**
 * dw3000_deep_sleep_and_wakeup() - Put device in DEEP SLEEP state
 * @dw: the DW device on which the SPI transfer will occurs
 * @delay_us: the delay after the DW device is woken up
 *
 * The caller must save the next operational state and required data before
 * the DW device is woken-up by the timer.
 *
 * The timer is configured to wake-up the device after the specified delay.
 * Caller should take care of wake-up latency.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_deep_sleep_and_wakeup(struct dw3000 *dw, int delay_us)
{
	int rc;
#ifdef CONFIG_DW3000_DEBUG
	/* Backup config before changing state */
	rc = dw3000_backup_registers(dw, false);
	if (rc)
		return rc;
#endif
	/* Always launch timer, even if entering deep-sleep fail. */
	if (delay_us)
		dw3000_wakeup_timer_start(dw, delay_us);
	dw3000_pm_qos_update_request(dw, PM_QOS_RESUME_LATENCY_NO_CONSTRAINT);
	/* Put the chip in deep sleep immediately */
	rc = dw3000_setdwstate(dw, DW3000_OP_STATE_DEEP_SLEEP);
	trace_dw3000_deep_sleep(dw, rc);
	return rc;
}

/**
 * dw3000_lock_pll() - Auto calibrate the PLL and change to IDLE_PLL state
 * @dw: the DW device on which the SPI transfer will occurs
 * @sys_status: the last known sys_status register LSB value
 *
 * It also checks on the CPLOCK bit field to be set in the SYS_STATUS register
 * which means the PLL is locked (for D0/E0).
 *
 * Return: zero on success, else a negative error code.
 */
static inline int dw3000_lock_pll(struct dw3000 *dw, u8 sys_status)
{
	u8 flag;
	int cnt;
	int rc;

	if (sys_status & DW3000_SYS_STATUS_CLK_PLL_LOCK_BIT_MASK) {
		/* PLL already locked! Just change current_operational_state */
		dw3000_set_operational_state(dw, DW3000_OP_STATE_IDLE_PLL);
		goto resync_dtu;
	}
	if (__dw3000_chip_version == DW3000_E0_VERSION) {
		/* Verify PLL lock bit is cleared */
		int rc = dw3000_reg_write8(
			dw, DW3000_SYS_STATUS_ID, 0,
			DW3000_SYS_STATUS_CLK_PLL_LOCK_BIT_MASK);
		if (rc)
			return rc;
	}
	rc = dw3000_setdwstate(dw, DW3000_OP_STATE_IDLE_PLL);
	if (rc)
		return rc;

	/* For C0, wait for PLL lock, else SYS_TIME is 0 */
	if (__dw3000_chip_version == DW3000_C0_VERSION) {
		udelay(DW3000_PLL_LOCK_DELAY_US);
		goto resync_dtu;
	}
	/* For D0/E0, wait PLL locked bit in SYS_STATUS */
	for (flag = 1, cnt = 0; cnt < DW3000_MAX_RETRIES_FOR_PLL; cnt++) {
		u8 status;

		usleep_range(10, 40);
		/* Verify PLL lock bit is locked */
		dw3000_reg_read8(dw, DW3000_SYS_STATUS_ID, 0, &status);
		if ((status & DW3000_SYS_STATUS_CLK_PLL_LOCK_BIT_MASK)) {
			/* PLL is locked. */
			flag = 0;
			break;
		}
	}
	if (flag)
		return -EAGAIN;
resync_dtu:
	/* Re-sync SYS_TIME and DTU when chip is in IDLE_PLL */
	rc = dw3000_resync_dtu_sys_time(dw);
	return rc;
}

/**
 * dw3000_run_pgfcal() - Run PGF calibration
 * @dw: the DW device
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_run_pgfcal(struct dw3000 *dw)
{
	u32 val = 0;
	u8 cal;
	/* Put into calibration mode and turn on delay mode */
	u32 data = (((u32)0x02) << DW3000_PGF_CAL_CFG_COMP_DLY_BIT_OFFSET) |
		   (DW3000_PGF_CAL_CFG_PGF_MODE_BIT_MASK & 0x1);
	int rc;

	rc = dw3000_reg_write32(dw, DW3000_PGF_CAL_CFG_ID, 0x0, data);
	if (rc)
		return rc;
	/* Trigger PGF calibration */
	rc = dw3000_reg_or8(dw, DW3000_PGF_CAL_CFG_ID, 0x0,
			    DW3000_PGF_CAL_CFG_CAL_EN_BIT_MASK);
	if (rc)
		return rc;
	/* Calibration will be done within ~30 us (add some margin) */
	/* TODO: On D0 active wait with lower delays. */
	usleep_range(DW3000_PGFCAL_DELAY_US, DW3000_PGFCAL_DELAY_US + 100);
	/* Check if calibration is done.*/
	rc = dw3000_reg_read8(dw, DW3000_PGF_CAL_STS_ID, 0x0, &cal);
	if (rc)
		return rc;
	if (cal != 1) {
		goto calib_err;
	}
	/* Put into normal mode */
	rc = dw3000_reg_write8(dw, DW3000_PGF_CAL_CFG_ID, 0x0, 0);
	if (rc)
		return rc;
	/* Clear the status */
	rc = dw3000_reg_write8(dw, DW3000_PGF_CAL_STS_ID, 0x0, 1);
	if (rc)
		return rc;
	/* Enable reading */
	rc = dw3000_reg_or8(dw, DW3000_PGF_CAL_CFG_ID, 0x2, 0x1);
	if (rc)
		return rc;
	/* PFG I calibration */
	rc = dw3000_reg_read32(dw, DW3000_PGF_I_CTRL1_ID, 0x0, &val);
	if (rc)
		return rc;
	if (val == 0x1fffffff) {
		goto calib_err;
	}
	/* PFG Q calibration */
	rc = dw3000_reg_read32(dw, DW3000_PGF_Q_CTRL1_ID, 0x0, &val);
	if (rc)
		return rc;
	if (val == 0x1fffffff) {
		goto calib_err;
	}
	return 0;
calib_err:
	dev_err(dw->dev, "PGF calibration failed\n");
	return -EREMOTEIO;
}

/**
 * dw3000_pgf_cal() - Run PGF calibration
 * @dw: the DW device
 * @ldoen: if set to true the function will enable LDOs prior to calibration
 * and disable afterwards.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_pgf_cal(struct dw3000 *dw, bool ldoen)
{
	u16 val;
	int rc;

	if (__dw3000_chip_version >= DW3000_D0_VERSION) {
		u32 resi;
		/* Enable reading of CAL result */
		rc = dw3000_reg_or8(dw, DW3000_RX_CAL_CFG_ID, 0x2, 0x1);
		if (rc)
			return rc;
		/* Read calibration */
		rc = dw3000_reg_read32(dw, DW3000_RX_CAL_RESI_ID, 0x0, &resi);
		if (rc)
			return rc;
		/* If not D0 and not soft reset, no need to continue */
		if ((__dw3000_chip_version != DW3000_D0_VERSION) && (resi != 0))
			return 0;
	}

	/* PGF needs LDOs turned on - ensure PGF LDOs are enabled */
	if (ldoen) {
		rc = dw3000_reg_read16(dw, DW3000_LDO_CTRL_ID, 0, &val);
		if (rc)
			return rc;
		rc = dw3000_reg_or16(dw, DW3000_LDO_CTRL_ID, 0,
				     (DW3000_LDO_CTRL_LDO_VDDIF2_EN_BIT_MASK |
				      DW3000_LDO_CTRL_LDO_VDDMS3_EN_BIT_MASK |
				      DW3000_LDO_CTRL_LDO_VDDMS1_EN_BIT_MASK));
		if (rc)
			return rc;
	}
	dw->pgf_cal_running = true;
	/* Run PGF Cal */
	rc = dw3000_run_pgfcal(dw);
	dw->pgf_cal_running = false;
	/* Turn off RX LDOs if previously off */
	if (ldoen) {
		/* restore LDO values */
		return dw3000_reg_and16(dw, DW3000_LDO_CTRL_ID, 0, val);
	}
	return rc;
}

/**
 * dw3000_configure() - configure the whole device
 * @dw: the DW device
 *
 * Configure with no STS and SCP mode and without API errors.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_configure(struct dw3000 *dw)
{
	struct dw3000_config *config = &dw->config;
	int symb = _plen_info[config->txPreambLength - 1].symb;
	int pac_symb = _plen_info[config->txPreambLength - 1].pac_symb;
	int sfd_symb = dw3000_get_sfd_symb(dw);
	int pg_count = 0;
	int rc;
	/* Clear the sleep mode ALT_GEAR bit */
	dw->data.sleep_mode &= (~(DW3000_ALT_GEAR | DW3000_SEL_GEAR3));
	dw->data.max_frames_len = config->phrMode ? DW3000_EXT_FRAME_LEN :
						    DW3000_STD_FRAME_LEN;
	/* Configure the SYS_CFG register */
	rc = dw3000_configure_sys_cfg(dw, config);
	if (rc)
		return rc;
	/* Configure the RF channel */
	rc = dw3000_configure_chan(dw);
	if (rc)
		return rc;
	/* Setup TX preamble size, PRF and data rate */
	rc = dw3000_reg_modify32(
		dw, DW3000_TX_FCTRL_ID, 0,
		~(DW3000_TX_FCTRL_TXBR_BIT_MASK |
		  DW3000_TX_FCTRL_TXPSR_PE_BIT_MASK),
		((u32)config->dataRate << DW3000_TX_FCTRL_TXBR_BIT_OFFSET) |
			((u32)config->txPreambLength)
				<< DW3000_TX_FCTRL_TXPSR_PE_BIT_OFFSET);
	if (rc)
		return rc;
	/**
	 * DTUNE (SFD timeout):
	 * SFD timeout = PLEN + 1 + sfd len - pac size
	 */
	config->sfdTO = symb + 1 + sfd_symb - pac_symb;

	rc = dw3000_reg_write16(dw, DW3000_DRX_SFDTOC_ID, 0, config->sfdTO);
	if (rc)
		return rc;
	/* Auto calibrate the PLL and change to IDLE_PLL state */
	rc = dw3000_lock_pll(dw, 0);
	if (rc)
		return rc;
	/**
	 * PGF: If the RX calibration routine fails the device receiver
	 * performance will be severely affected, the application should reset
	 * and try again.
	 * PGF NOT needed for E0 as routine automatically runs on startup/hw reset, on wakeup
	 * however if reconfiguring after soft reset then the calibration needs to be run
	 */
	rc = dw3000_pgf_cal(dw, 1);
	if (rc)
		return rc;
	/* Calibrate ADC offset, if needed, after DGC configuration and after PLL lock.
	 * If this calibration is executed before the PLL lock, the PLL lock failed.
	 */

	dw3000_configmrxlut(dw);
	if (dw->chip_ops->adc_offset_calibration)
		rc = dw->chip_ops->adc_offset_calibration(dw);
	/* Update configuration dependent timings */
	if (dw->bw_comp) {
		pg_count = dw3000_calc_pgcount(dw, dw->txconfig.PGdly);
		 if (pg_count !=0) {
			 dw3000_calc_bandwithadj(dw, pg_count);
		 }
	}
	dw3000_update_timings(dw);
	return rc;
}

static int dw3000_setrxantennadelay(struct dw3000 *dw, u16 rxDelay)
{
	/* Set the RX antenna delay for auto TX timestamp adjustment */
	return dw3000_reg_write16(dw, DW3000_RX_ANTENNA_DELAY_ID, 0, rxDelay);
}

static int dw3000_settxantennadelay(struct dw3000 *dw, u16 txDelay)
{
	/* Set the TX antenna delay for auto TX timestamp adjustment */
	return dw3000_reg_write16(dw, DW3000_TX_ANTD_ID, 0, txDelay);
}

static int dw3000_set_antenna_delay(struct dw3000 *dw, u16 delay)
{
	int rc = dw3000_setrxantennadelay(dw, delay);

	if (rc)
		return rc;
	return dw3000_settxantennadelay(dw, delay);
}

/**
 * dw3000_set_eui64() - Set device's Extended Unique Identifier
 * @dw: the DW device
 * @val: the EUI
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_set_eui64(struct dw3000 *dw, __le64 val)
{
	return dw3000_reg_write_fast(dw, DW3000_EUI_64_ID, 0, 8, &val,
				     DW3000_SPI_WR_BIT);
}

/**
 * dw3000_set_pancoord() - Enable/disable the device as PAN coordinator
 * @dw: the DW device
 * @active: enables the device as PAN coordinator if true, otherwise disables
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_set_pancoord(struct dw3000 *dw, bool active)
{
	if (active) {
		return dw3000_reg_or8(dw, DW3000_ADR_FILT_CFG_ID, 1,
				      DW3000_AS_PANCOORD);
	} else {
		return dw3000_reg_and8(dw, DW3000_ADR_FILT_CFG_ID, 1,
				       ~DW3000_AS_PANCOORD);
	}
}

/**
 * dw3000_set_panid() - Set device's PAN Identifier
 * @dw: the DW device
 * @val: the PAN ID
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_set_panid(struct dw3000 *dw, __le16 val)
{
	return dw3000_reg_write_fast(dw, DW3000_PANADR_ID,
				     (DW3000_PANADR_PAN_ID_BIT_OFFSET / 8),
				     DW3000_PANADR_PAN_ID_BIT_LEN, &val,
				     DW3000_SPI_WR_BIT);
}

/**
 * dw3000_set_shortaddr() - Set device's short address
 * @dw: the DW device
 * @val: the short address
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_set_shortaddr(struct dw3000 *dw, __le16 val)
{
	return dw3000_reg_write_fast(dw, DW3000_PANADR_ID,
				     (DW3000_PANADR_SHORT_ADDR_BIT_OFFSET / 8),
				     DW3000_PANADR_SHORT_ADDR_BIT_LEN, &val,
				     DW3000_SPI_WR_BIT);
}

/**
 * dw3000_configure_hw_addr_filt() - Set device's hardware address filtering
 * parameters
 * @dw: the DW device
 * @changed: bitfields of flags indicating parameters which have changed
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_configure_hw_addr_filt(struct dw3000 *dw, unsigned long changed)
{
	struct ieee802154_hw_addr_filt *filt = &dw->config.hw_addr_filt;
	int rc;

	if (!changed)
		return 0;

	if (changed & IEEE802154_AFILT_SADDR_CHANGED) {
		rc = dw3000_set_shortaddr(dw, filt->short_addr);
		if (rc)
			return rc;
	}
	if (changed & IEEE802154_AFILT_PANID_CHANGED) {
		rc = dw3000_set_panid(dw, filt->pan_id);
		if (rc)
			return rc;
	}
	if (changed & IEEE802154_AFILT_PANC_CHANGED) {
		rc = dw3000_set_pancoord(dw, filt->pan_coord);
		if (rc)
			return rc;
	}
	if (changed & IEEE802154_AFILT_IEEEADDR_CHANGED) {
		rc = dw3000_set_eui64(dw, filt->ieee_addr);
		if (rc)
			return rc;
	}

	return 0;
}

static int dw3000_reconfigure_hw_addr_filt(struct dw3000 *dw)
{
	struct dw3000_deep_sleep_state *dss = &dw->deep_sleep_state;
	struct ieee802154_hw_addr_filt *filt = &dw->config.hw_addr_filt;
	unsigned long changed = dss->config_changed;

	/* Always force setup of ieee_addr if non 0 on wakeup
	   since not saved in AON. */
	if (filt->ieee_addr)
		changed |= DW3000_AFILT_IEEEADDR_CHANGED;
	dss->config_changed &=
		~(DW3000_AFILT_SADDR_CHANGED | DW3000_AFILT_IEEEADDR_CHANGED |
		  DW3000_AFILT_PANID_CHANGED | DW3000_AFILT_PANC_CHANGED);
	return dw3000_configure_hw_addr_filt(dw, changed);
}

/**
 * dw3000_framefilter_enable() - Enable and set the device's frame filter
 * @dw: the DW device
 * @filtermode: filter mode
 *
 * Return: zero on success, else a negative error code.
 */
static inline int dw3000_framefilter_enable(struct dw3000 *dw, u16 filtermode)
{
	/* Use 802.15.4 filtering rules */
	int rc = dw3000_reg_or8(dw, DW3000_SYS_CFG_ID, 0,
				(u8)(DW3000_SYS_CFG_FFEN_BIT_MASK));
	if (rc)
		return rc;
	/* Set the frame filter configuration */
	return dw3000_reg_write16(dw, DW3000_ADR_FILT_CFG_ID, 0, filtermode);
}

/**
 * dw3000_framefilter_disable() - Disable the device's frame filter
 * @dw: the DW device
 *
 * Return: zero on success, else a negative error code.
 */
static inline int dw3000_framefilter_disable(struct dw3000 *dw)
{
	/* Disable frame filter */
	int rc = dw3000_reg_and8(dw, DW3000_SYS_CFG_ID, 0,
				 (u8)(~(DW3000_SYS_CFG_FFEN_BIT_MASK)));
	if (rc)
		return rc;
	/* Clear the configuration */
	return dw3000_reg_write16(dw, DW3000_ADR_FILT_CFG_ID, 0, 0x0);
}

/**
* dw3000_read_pdoa() - Read the PDOA result.
* @dw: The DW device.
*
* This is used to read the PDOA result, it is the phase difference between
* either the Ipatov and STS POA, or the two STS POAs, depending on the PDOA
* mode of operation. (PDoA - Phase Difference On Arrival).
* NOTE: To convert to degrees: float pdoa_deg =
* ((float)pdoa / (1 << 11)) * 180 / M_PI.
*
* Return: The PDOA result (signed in [1:-11] radian units).
*/
s16 dw3000_read_pdoa(struct dw3000 *dw)
{
	const u16 b12_sign_extend_test = 0x2000;
	const u16 b12_sign_extend_mask = 0xc000;
	const u16 pi_floored_q11 = 3.141592653589 * (1 << 11);
	const u16 pix2_rounded_q11 = 3.141592653589 * 2 * (1 << 11) + 0.5;
	u16 val;
	s16 pdoa;

	switch (dw->data.dblbuffon) {
	case DW3000_DBL_BUFF_ACCESS_BUFFER_B:
		dw3000_reg_read16(dw, DW3000_INDIRECT_POINTER_B_ID,
				  DW3000_DB_DIAG_PDOA + 2, &val);
		pdoa = val & (DW3000_CIA_TDOA_1_PDOA_RX_PDOA_BIT_MASK >> 16);
		break;
	case DW3000_DBL_BUFF_ACCESS_BUFFER_A:
		dw3000_reg_read16(dw, DW3000_DB_DIAG_SET_1,
				  DW3000_DB_DIAG_PDOA + 2, &val);
		pdoa = val & (DW3000_CIA_TDOA_1_PDOA_RX_PDOA_BIT_MASK >> 16);
		break;
	default:
		dw3000_reg_read16(dw, DW3000_CIA_TDOA_1_PDOA_ID, 2, &val);
		/* Phase difference of the 2 POAs. */
		pdoa = val & (DW3000_CIA_TDOA_1_PDOA_RX_PDOA_BIT_MASK >> 16);
	}
	trace_dw3000_read_pdoa(dw, pdoa);
	if (pdoa & b12_sign_extend_test)
		pdoa |= b12_sign_extend_mask;
	pdoa -= dw->config.pdoaOffset;
	if (pdoa < -pi_floored_q11)
		pdoa += pix2_rounded_q11;
	if (pdoa > pi_floored_q11)
		pdoa -= pix2_rounded_q11;
	return pdoa;
}

/**
 * dw3000_pdoa_to_aoa_lut() - Convert PDoA to AoA.
 * @dw: the DW device
 * @pdoa_rad_q11: the PDoA value as returned by dw3000_read_pdoa()
 *
 * Convert PDoA (in radian, encoded as a Q11 fixed
 * point number) to AoA value using calibration look-up table.
 *
 * Return: AoA value interpolated from LUT values.
 */
s16 dw3000_pdoa_to_aoa_lut(struct dw3000 *dw, s16 pdoa_rad_q11)
{
	const dw3000_pdoa_lut_t *lut = dw->config.pdoaLut;
	int a = 0, b = DW3000_CALIBRATION_PDOA_LUT_MAX - 1;
	s16 delta_pdoa, delta_aoa;

	if (pdoa_rad_q11 < (*lut)[0][0])
		return (*lut)[0][1];
	if (pdoa_rad_q11 >= (*lut)[DW3000_CALIBRATION_PDOA_LUT_MAX - 1][0])
		return (*lut)[DW3000_CALIBRATION_PDOA_LUT_MAX - 1][1];

	while (a != b) {
		int m = (a + b) / 2;
		if (pdoa_rad_q11 < (*lut)[m][0])
			b = m;
		else
			a = m + 1;
	}

	delta_pdoa = (*lut)[a][0] - (*lut)[a - 1][0];
	delta_aoa = (*lut)[a][1] - (*lut)[a - 1][1];

	return (*lut)[a][1] +
	       (delta_aoa * (pdoa_rad_q11 - (*lut)[a][0])) / delta_pdoa;
}

/**
 * dw3000_set_sts_pdoa() - set device's STS & PDOA mode
 * @dw: the DW device
 * @sts_mode: the STS mode
 * @pdoa_mode: the PDOA mode
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_set_sts_pdoa(struct dw3000 *dw, u8 sts_mode, u8 pdoa_mode)
{
	struct dw3000_config *config = &dw->config;
	bool changed = false;
	u8 prev_sts = config->stsMode;
	u8 prev_pdoa = config->pdoaMode;
	int rc = 0;

	/* This configuration is reserved or not supported
	 * (c.f DW3000 User Manual) */
	if (pdoa_mode == DW3000_PDOA_M2)
		return -EOPNOTSUPP;

	if ((config->stsMode ^ sts_mode) & DW3000_STS_BASIC_MODES_MASK) {
		/**
		 * Enable the Super Deterministic Code according current STS
		 * key value set by the MCPS. Setting NUL key will set the
		 * SDC flag in stsMode.
		 */
		u8 sts_sdc = config->stsMode & DW3000_STS_MODE_SDC;
		config->stsMode = sts_mode | sts_sdc;
		changed = true;
	}
	if (config->pdoaMode != pdoa_mode) {
		config->pdoaMode = pdoa_mode;
		changed = true;
	}
	/* Re-configure the device with new STS & PDOA mode */
	if (changed) {
		rc = dw3000_configure_sys_cfg(dw, config);
		if (rc) {
			/* Restore initial value */
			config->stsMode = prev_sts;
			config->pdoaMode = prev_pdoa;
		}
	}
	return rc;
}

/**
 * dw3000_set_sts_length() - set device's STS length
 * @dw: the DW device
 * @len: length of the STS in blocks of 8 symbols
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_set_sts_length(struct dw3000 *dw, enum dw3000_sts_lengths len)
{
	struct dw3000_config *config = &dw->config;
	bool changed = false;
	int rc;

	if (config->stsLength != len) {
		rc = dw3000_reg_write8(dw, DW3000_CP_CFG0_ID, 0,
				       DW3000_GET_STS_LEN_REG_VALUE(len));
		if (unlikely(rc))
			return rc;
		config->stsLength = len;
		changed = true;
	}
	/* Re-configure the device with new STS length */
	if (changed)
		return dw3000_configure_sts(dw, config);
	return 0;
}

/**
 * dw3000_read_sts_timestamp() - read frame STS timestamp
 * @dw: the DW device
 * @sts_ts: the address where to store STS timestamp value
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_read_sts_timestamp(struct dw3000 *dw, u64 *sts_ts)
{
	int rc;
	int fileid;
	int offset;
	__le64 buffer;

	switch (dw->data.dblbuffon) {
	case DW3000_DBL_BUFF_ACCESS_BUFFER_A:
		fileid = DW3000_DB_DIAG_SET_1;
		offset = DW3000_DB_DIAG_STS_TS;
		break;
	case DW3000_DBL_BUFF_ACCESS_BUFFER_B:
		fileid = DW3000_INDIRECT_POINTER_B_ID;
		offset = DW3000_DB_DIAG_STS_TS;
		break;
	default:
		fileid = DW3000_STS_TOA_LO_ID;
		offset = 0;
	}
	/* Read 8 bytes (64-bits) register into buffer. */
	rc = dw3000_reg_read_fast(dw, fileid, offset, sizeof(buffer), &buffer);
	if (rc)
		return rc;
	*sts_ts = le64_to_cpu(buffer) &
		  ((u64)DW3000_STS_TOA_LO_STS_TOA_BIT_MASK |
		   ((u64)DW3000_STS_TOA_HI_STS_TOA_BIT_MASK
		    << DW3000_STS_TOA_LO_STS_TOA_BIT_LEN));
	return 0;
}

/**
 * dw3000_read_sts_quality() - read received frame STS quality
 * @dw: the DW device
 * @acc_qual: the STS accumulation quality of the frame
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_read_sts_quality(struct dw3000 *dw, s16 *acc_qual)
{
	int rc;
	u16 qual;
	rc = dw3000_reg_read16(dw, DW3000_STS_STS_ID, 0, &qual);
	if (rc)
		return rc;
	if (qual & DW3000_STS_ACC_CP_QUAL_SIGNTST)
		qual |= DW3000_STS_ACC_CP_QUAL_SIGNEXT;
	*acc_qual = (s16)qual;
	return 0;
}

/**
 * dw3000_set_promiscuous() - Enable or disable the promiscuous mode
 * @dw: the DW device
 * @on: enable the promiscuous mode if true, otherwise disable it
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_set_promiscuous(struct dw3000 *dw, bool on)
{
	if (on)
		return dw3000_framefilter_disable(dw);
	return dw3000_framefilter_enable(
		dw, DW3000_FF_BEACON_EN | DW3000_FF_DATA_EN | DW3000_FF_ACK_EN |
			    DW3000_FF_COORD_EN);
}

/**
 * dw3000_set_autoack_reply_delay() - Select the delay used for auto-ack.
 * @dw: The DW device.
 * @response_delay_time_symbols: Delay in symbols.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_set_autoack_reply_delay(struct dw3000 *dw,
				   u8 response_delay_time_symbols)
{
	struct dw3000_local_data *local = &dw->data;
	int rc;

	if (local->ack_time == response_delay_time_symbols)
		return 0;
	rc = dw3000_reg_write8(dw, DW3000_ACK_RESP_ID,
			       DW3000_ACK_RESP_ACK_TIM_BIT_OFFSET / 8,
			       response_delay_time_symbols);
	if (unlikely(rc))
		return rc;
	local->ack_time = response_delay_time_symbols;
	return 0;
}

/**
 * dw3000_enable_autoack() - Enable the autoack for futures rx.
 * @dw: The DW device.
 * @force: Enable even if it was already in enable state.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_enable_autoack(struct dw3000 *dw, bool force)
{
	int r = 0;

	if (!dw->autoack || force) {
		/* Set the AUTO_ACK bit. */
		r = dw3000_reg_or32(
			dw, DW3000_SYS_CFG_ID, 0,
			DW3000_SYS_CFG_AUTO_ACK_BIT_MASK |
				DW3000_SYS_CFG_FAST_AAT_EN_BIT_MASK);
		if (!r)
			dw->autoack = true;
	}
	return r;
}

/**
 * dw3000_disable_autoack() - Disable the autoack for futures rx.
 * @dw: The DW device.
 * @force: Disable even if it was already in disable state.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_disable_autoack(struct dw3000 *dw, bool force)
{
	int r = 0;

	if (dw->autoack || force) {
		/* Clear the AUTO_ACK bit.*/
		r = dw3000_reg_and32(dw, DW3000_SYS_CFG_ID, 0,
				     (~DW3000_SYS_CFG_AUTO_ACK_BIT_MASK |
				      DW3000_SYS_CFG_FAST_AAT_EN_BIT_MASK));
		if (!r)
			dw->autoack = false;
	}
	return r;
}

/**
 * dw3000_otp_read32() - 32 bits OTP memory read.
 * @dw: the DW device
 * @addr: address in the OTP memory
 * @val: value read from the OTP memory
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_otp_read32(struct dw3000 *dw, u16 addr, u32 *val)
{
	int rc;
	u32 tmp = 0;

	/* Set manual access mode */
	/* TODO: avoid hard number, and replace it. */
	rc = dw3000_reg_write16(dw, DW3000_NVM_CFG_ID, 0, 0x0001);
	if (unlikely(rc))
		return rc;
	/* Set the address */
	rc = dw3000_reg_write16(dw, DW3000_NVM_ADDR_ID, 0, addr);
	if (unlikely(rc))
		return rc;
	/* Assert the read strobe */
	/* TODO: avoid hard number, and replace it. */
	rc = dw3000_reg_write16(dw, DW3000_NVM_CFG_ID, 0, 0x0002);
	if (unlikely(rc))
		return rc;
	/* Attempt a read from OTP address */
	rc = dw3000_reg_read32(dw, DW3000_NVM_RDATA_ID, 0, &tmp);
	if (unlikely(rc))
		return rc;
	/* Return the 32 bits of read data */
	*val = tmp;
	return 0;
}

/**
 * dw3000_read_otp() - Read all required data from OTP.
 * @dw: The DW device.
 * @mode: bitfield to select information to retrieve from OTP memory
 *	See @DW3000_READ_OTP_PID and other values.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_read_otp(struct dw3000 *dw, int mode)
{
	struct dw3000_otp_data *otp = &dw->otp_data;
	u32 val;
	int rc;

	/* Check if recalled with same parameters */
	if ((mode | 1) == otp->mode)
		return 0; /* No need to read OTP again. */

	/* Reset OTP local data */
	memset(otp, 0, sizeof(*otp));
	otp->mode = mode | 1;
	/* Values used by dw3000_prog_ldo_and_bias_tune()  */
	rc = dw3000_otp_read32(dw, DW3000_LDOTUNELO_ADDRESS, &otp->ldo_tune_lo);
	if (rc)
		return rc;
	rc = dw3000_otp_read32(dw, DW3000_LDOTUNEHI_ADDRESS, &otp->ldo_tune_hi);
	if (rc)
		return rc;
	rc = dw3000_otp_read32(dw, DW3000_BIAS_TUNE_ADDRESS, &otp->bias_tune);
	if (rc)
		return rc;
	/* Values used by dw3000_prog_xtrim() */
	rc = dw3000_otp_read32(dw, DW3000_XTRIM_ADDRESS, &val);
	if (unlikely(rc))
		return rc;
	/* TODO: avoid hard number, and replace it. */
	otp->xtal_trim = val & 0x7f;
	/* Load optional values according to mode parameter */
	if (mode & DW3000_READ_OTP_PID) {
		rc = dw3000_otp_read32(dw, DW3000_PARTID_ADDRESS, &otp->partID);
		if (unlikely(rc))
			return rc;
	}
	if (mode & DW3000_READ_OTP_LID) {
		rc = dw3000_otp_read32(dw, DW3000_LOTID_ADDRESS, &otp->lotID);
		if (unlikely(rc))
			return rc;
	}
	if (mode & DW3000_READ_OTP_BAT) {
		rc = dw3000_otp_read32(dw, DW3000_VBAT_ADDRESS, &val);
		if (unlikely(rc))
			return rc;
		/* TODO: avoid hard number, and replace it. */
		otp->vBatP = val & 0xff;
	}
	if (mode & DW3000_READ_OTP_TMP) {
		rc = dw3000_otp_read32(dw, DW3000_VTEMP_ADDRESS, &val);
		if (unlikely(rc))
			return rc;
		/* TODO: avoid hard number, and replace it. */
		otp->tempP = val & 0xff;
	}
	/* If the reference temperature has not been programmed in OTP (early
	   eng samples) set to default value */
	/* TODO: avoid hard number, and replace it. */
	if (otp->tempP == 0)
		otp->tempP = 0x85; /* @temp of 20 deg */
	/* If the reference voltage has not been programmed in OTP (early eng
	   samples) set to default value */
	/* TODO: avoid hard number, and replace it. */
	if (otp->vBatP == 0)
		otp->vBatP = 0x74; /* @Vref of 3.0V */
	/* OTP revision */
	rc = dw3000_otp_read32(dw, DW3000_OTPREV_ADDRESS, &val);
	if (unlikely(rc))
		return rc;
	otp->rev = val & 0xff;
	/* Some chip depending adjustment */
	if (__dw3000_chip_version >= DW3000_D0_VERSION) {
		if (otp->xtal_trim == 0)
			/* Set the default value for D0 if none set in OTP. */
			otp->xtal_trim = DW3000_DEFAULT_XTAL_TRIM;
		/* Read DGC tune addr */
		rc = dw3000_otp_read32(dw, DW3000_DGC_TUNE_ADDRESS,
				       &otp->dgc_addr);
		if (rc)
			return rc;
	}
	/* PLL coarse code : starting code for calibration procedure */
	return dw3000_otp_read32(dw, DW3000_PLL_CC_ADDRESS,
				 &otp->pll_coarse_code);
}

/**
 * _dw3000_otp_wdata_write() - Configure and write value to the OTP
 * memory block.
 * @dw: The DW device.
 * @val: 16-bit value to write to the OTP block.
 *
 * This function is used to configure the OTM memory block for memory
 * writing operations and also to store the data value to be programmed
 * into an OTP location. It is involved in the OTP memory writing procedure
 * that is implemented into the '_dw3000_otp_write32()' function.
 *
 * Return: zero on success, else a negative error code.
 */
static inline int _dw3000_otp_wdata_write(struct dw3000 *dw, u16 val)
{
	/**
	 * Pull the CS high to enable user interface for programming.
	 * 'val' is ignored in this instance by the OTP block.
	 */
	/* TODO: avoid hard number, and replace it. */
	int rc = dw3000_reg_write16(dw, DW3000_NVM_WDATA_ID, 0, 0x0200 | val);

	if (unlikely(rc))
		return rc;
	/* Send the relevant command to the OTP block */
	/* TODO: avoid hard number, and replace it. */
	return dw3000_reg_write16(dw, DW3000_NVM_WDATA_ID, 0, 0x0000 | val);
}

/**
 * _dw3000_otp_write32() - Program 32-bit value in OTP memory.
 * @dw: The DW device.
 * @data: data to write to given address.
 * @addr: address to write to.
 *
 * Return: zero on success, else a negative error code.
 */
static int _dw3000_otp_write32(struct dw3000 *dw, u16 addr, u32 data)
{
	u32 ldo_tune;
	u32 rd_buf;
	u16 wr_buf[4];
	u16 i;
	int rc;

	/* Read current register value */
	rc = dw3000_reg_read32(dw, DW3000_LDO_TUNE_HI_ID, 0, &ldo_tune);
	if (rc)
		return rc;

	/* Set VDDHV_TX LDO to max */
	rc = dw3000_reg_or32(dw, DW3000_LDO_TUNE_HI_ID, 0,
			     DW3000_LDO_TUNE_HI_LDO_HVAUX_TUNE_BIT_MASK);
	if (rc)
		return rc;

	/* Configure mode for programming */
	/* TODO: avoid hard number, and replace it. */
	rc = dw3000_reg_write16(dw, DW3000_NVM_CFG_ID, 0, 0x018);
	if (rc)
		return rc;

	/* Select fast programming */
	/* TODO: avoid hard number, and replace it. */
	rc = _dw3000_otp_wdata_write(dw, 0x0025);
	if (rc)
		return rc;

	/* Apply instruction to write the address */
	/* TODO: avoid hard number, and replace it. */
	rc = _dw3000_otp_wdata_write(dw, 0x0002);
	if (rc)
		return rc;
	/* TODO: avoid hard number, and replace it. */
	rc = _dw3000_otp_wdata_write(dw, 0x01fc);
	if (rc)
		return rc;

	/* Sending the OTP address data (2 bytes) */
	/* TODO: avoid hard number, and replace it. */
	wr_buf[0] = 0x0100 | (addr & 0xff);
	rc = _dw3000_otp_wdata_write(dw, wr_buf[0]);
	if (rc)
		return rc;

	/* Write data (upper byte of address) */
	/* TODO: avoid hard number, and replace it. */
	rc = _dw3000_otp_wdata_write(dw, 0x0100);
	if (rc)
		return rc;

	/* Clean up */
	/* TODO: avoid hard number, and replace it. */
	rc = _dw3000_otp_wdata_write(dw, 0x0000);
	if (rc)
		return rc;

	/* Apply instruction to write data */
	/* TODO: avoid hard number, and replace it. */
	rc = _dw3000_otp_wdata_write(dw, 0x0002);
	if (rc)
		return rc;
	/* TODO: avoid hard number, and replace it. */
	rc = _dw3000_otp_wdata_write(dw, 0x01c0);
	if (rc)
		return rc;

	/* Write the data */
	/* TODO: use standard function to get expected endianness. */
	wr_buf[0] = 0x100 | ((data >> 24) & 0xff);
	wr_buf[1] = 0x100 | ((data >> 16) & 0xff);
	wr_buf[2] = 0x100 | ((data >> 8) & 0xff);
	wr_buf[3] = 0x100 | (data & 0xff);
	rc = _dw3000_otp_wdata_write(dw, wr_buf[3]);
	if (rc)
		return rc;
	rc = _dw3000_otp_wdata_write(dw, wr_buf[2]);
	if (rc)
		return rc;
	rc = _dw3000_otp_wdata_write(dw, wr_buf[1]);
	if (rc)
		return rc;
	rc = _dw3000_otp_wdata_write(dw, wr_buf[0]);
	if (rc)
		return rc;

	/* Clean up */
	/* TODO: avoid hard number, and replace it. */
	rc = _dw3000_otp_wdata_write(dw, 0x0000);
	if (rc)
		return rc;

	/* Enter prog mode */
	/* TODO: avoid hard number, and replace it. */
	rc = _dw3000_otp_wdata_write(dw, 0x003a);
	if (rc)
		return rc;
	/* TODO: avoid hard number, and replace it. */
	rc = _dw3000_otp_wdata_write(dw, 0x01ff);
	if (rc)
		return rc;
	/* TODO: avoid hard number, and replace it. */
	rc = _dw3000_otp_wdata_write(dw, 0x010a);
	if (rc)
		return rc;
	/* Clean up */
	/* TODO: avoid hard number, and replace it. */
	rc = _dw3000_otp_wdata_write(dw, 0x0000);
	if (rc)
		return rc;

	/* Enable state/status output */
	/* TODO: avoid hard number, and replace it. */
	_dw3000_otp_wdata_write(dw, 0x003a);
	_dw3000_otp_wdata_write(dw, 0x01bf);
	_dw3000_otp_wdata_write(dw, 0x0100);

	/* Start prog mode */
	/* TODO: avoid hard number, and replace it. */
	rc = _dw3000_otp_wdata_write(dw, 0x003a);
	if (rc)
		return rc;
	/* TODO: avoid hard number, and replace it. */
	rc = _dw3000_otp_wdata_write(dw, 0x0101);
	if (rc)
		return rc;

	/* Different to previous one */
	/* TODO: avoid hard number, and replace it. */
	rc = dw3000_reg_write16(dw, DW3000_NVM_WDATA_ID, 0, 0x0002);
	if (rc)
		return rc;
	/* TODO: avoid hard number, and replace it. */
	rc = dw3000_reg_write16(dw, DW3000_NVM_WDATA_ID, 0, 0x0000);
	if (rc)
		return rc;

	/**
	 * Read status after program command.
	 * 1000 is more than sufficient for max OTP programming delay and max
	 * supported DW3000 SPI rate.
	 * Instead a delay of 2ms (as commented out below) can be used.
	 * Burn time is about 1.76ms
	 */
	/* TODO: avoid hard number, and replace it. */
	for (i = 0; i < 1000; i++) {
		dw3000_reg_read32(dw, DW3000_NVM_STATUS_ID, 0, &rd_buf);
		if (!(rd_buf & DW3000_NVM_STATUS_NVM_PROG_DONE_BIT_MASK))
			break;
	}

	/* Stop prog mode */
	/* TODO: avoid hard number, and replace it. */
	rc = _dw3000_otp_wdata_write(dw, 0x003a);
	if (rc)
		return rc;
	/* TODO: avoid hard number, and replace it. */
	rc = _dw3000_otp_wdata_write(dw, 0x0102);
	if (rc)
		return rc;
	/* Different to previous one */
	/* TODO: avoid hard number, and replace it. */
	rc = dw3000_reg_write16(dw, DW3000_NVM_WDATA_ID, 0, 0x0002);
	if (rc)
		return rc;
	/* TODO: avoid hard number, and replace it. */
	rc = dw3000_reg_write16(dw, DW3000_NVM_WDATA_ID, 0, 0x0000);
	if (rc)
		return rc;

	/* Configure mode for reading */
	/* TODO: avoid hard number, and replace it. */
	rc = dw3000_reg_write16(dw, DW3000_NVM_CFG_ID, 0, 0x0000);
	if (rc)
		return rc;

	/* Restore LDO tune register */
	return dw3000_reg_write32(dw, DW3000_LDO_TUNE_HI_ID, 0, ldo_tune);
}

/**
 * dw3000_otp_write32() - 32 bits OTP memory write.
 * @dw: The DW device.
 * @addr: 16-bit OTP address into which the 32-bit data is programmed.
 * @data: 32-bit data to be programmed into OTP.
 *
 * This function write a 32-bit data at a given address in the OTP memory
 * and checks that that data is correctly written.
 *
 * Return: zero on success, one if the given data does not match the current
 * register value, else a negative error code.
 */
int dw3000_otp_write32(struct dw3000 *dw, u16 addr, u32 data)
{
	u32 tmp;
	int rc;

	/* Program the word */
	rc = _dw3000_otp_write32(dw, addr, data);
	if (rc)
		return rc;

	/* Check it is programmed correctly */
	rc = dw3000_otp_read32(dw, addr, &tmp);
	if (rc)
		return rc;

	return data != tmp;
}

/**
 * dw3000_prog_xtrim() - Programs the device's crystal frequency
 * @dw: The DW device.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_prog_xtrim(struct dw3000 *dw)
{
	struct dw3000_otp_data *otp = &dw->otp_data;
	int rc;

	if (otp->xtal_trim) {
		/* set the XTAL trim value as read from OTP */
		rc = dw3000_reg_write8(dw, DW3000_XTAL_ID, 0, otp->xtal_trim);
		if (unlikely(rc))
			return rc;
	}
	return 0;
}

/**
 * dw3000_set_gpio_mode() - Configure GPIO for selected device
 * @dw: The DW device.
 * @mask: the bits to reset
 * @mode: the bits to set
 *
 * Clear GPIOs which have been specified by the mask and set GPIOs mode as
 * specified.
 *
 * Return: 0 on success or errno.
 */
int dw3000_set_gpio_mode(struct dw3000 *dw, u32 mask, u32 mode)
{
	return dw3000_reg_modify32(dw, DW3000_GPIO_MODE_ID, 0, ~mask, mode);
}

/**
 * dw3000_set_gpio_dir() - Configure GPIO direction for selected device
 * @dw: The DW device.
 * @mask: the direction bits to reset
 * @dir: the direction bits to set, 1 for input, 0 for output
 *
 * Return: 0 on success or errno.
 */
int dw3000_set_gpio_dir(struct dw3000 *dw, u16 mask, u16 dir)
{
	return dw3000_reg_modify16(dw, DW3000_GPIO_DIR_ID, 0, ~mask, dir);
}

/**
 * dw3000_set_gpio_out() - Set GPIO output state for selected device
 * @dw: The DW device.
 * @reset: the output bits to reset
 * @set: the output bits to set
 *
 * Return: 0 on success or errno.
 */
int dw3000_set_gpio_out(struct dw3000 *dw, u16 reset, u16 set)
{
	return dw3000_reg_modify16(dw, DW3000_GPIO_OUT_ID, 0, ~reset, set);
}

/**
 * dw3000_set_lna_pa_mode() - Enable GPIO for external LNA or PA functionality
 * @dw: The DW device.
 * @lna_pa: LNA/PA configuration to set.
 *	Can be DW3000_LNA_PA_DISABLE, or an or-ed combination of
 *	DW3000_LNA_ENABLE, DW3000_PA_ENABLE and DW3000_TXRX_ENABLE.
 *
 * This is HW dependent, consult the DW3000/DW3700 User Manual.
 *
 * This can also be used for debug as enabling TX and RX GPIOs is quite handy
 * to monitor DW3000/DW3700's activity.
 *
 * NOTE: Enabling PA functionality requires that fine grain TX sequencing is
 *	 deactivated. This can be done using d3000_setfinegraintxseq().
 *
 * Return: 0 on success or errno.
 */
static int dw3000_set_lna_pa_mode(struct dw3000 *dw, int lna_pa)
{
	/* Clear GPIO 4, 5 configuration */
	u32 gpio_mask = (DW3000_GPIO_MODE_MSGP4_MODE_BIT_MASK |
			 DW3000_GPIO_MODE_MSGP5_MODE_BIT_MASK);
	u32 gpio_mode = 0;

	if (__dw3000_chip_version >= DW3000_D0_VERSION) {
		if (lna_pa & (DW3000_LNA_ENABLE | DW3000_TXRX_ENABLE))
			gpio_mode |= DW3000_GPIO_PIN5_EXTRXE;
		if (lna_pa & (DW3000_PA_ENABLE | DW3000_TXRX_ENABLE))
			gpio_mode |= DW3000_GPIO_PIN4_EXTTXE;
	} else {
		/* Also clear GPIO 0, 1, 6 configuration */
		gpio_mask |= (DW3000_GPIO_MODE_MSGP0_MODE_BIT_MASK |
			      DW3000_GPIO_MODE_MSGP1_MODE_BIT_MASK |
			      DW3000_GPIO_MODE_MSGP6_MODE_BIT_MASK);
		if (lna_pa & DW3000_LNA_ENABLE)
			gpio_mode |= DW3000_GPIO_PIN6_EXTRX;
		if (lna_pa & DW3000_PA_ENABLE)
			gpio_mode |= (DW3000_GPIO_PIN4_EXTDA |
				      DW3000_GPIO_PIN5_EXTTX);
		if (lna_pa & DW3000_TXRX_ENABLE)
			gpio_mode |= (DW3000_GPIO_PIN0_EXTTXE |
				      DW3000_GPIO_PIN1_EXTRXE);
	}
	return dw3000_set_gpio_mode(dw, gpio_mask, gpio_mode);
}

/**
 * dw3000_set_leds() - Configure TX/RX GPIOs to control LEDs
 * @dw: The DW device.
 * @mode: LEDs configuration to set
 *
 * This is used to set up Tx/Rx GPIOs which could be used to control LEDs
 *
 * Note: not completely IC dependent, also needs board with LEDS fitted on
 * right I/O lines this function enables GPIOs 2 and 3 which are connected
 * to LED3 and LED4 on EVB1000
 *
 * Return: 0 on success or errno.
 */
static int dw3000_set_leds(struct dw3000 *dw, u8 mode)
{
	u32 gpio_mask = (DW3000_GPIO_MODE_MSGP3_MODE_BIT_MASK |
			 DW3000_GPIO_MODE_MSGP2_MODE_BIT_MASK);
	int rc;

	if (mode & DW3000_LEDS_ENABLE) {
		u32 reg;
		/* Set up GPIO for LED output. */
		rc = dw3000_set_gpio_mode(dw, gpio_mask,
					  DW3000_GPIO_PIN2_RXLED |
						  DW3000_GPIO_PIN3_TXLED);
		if (unlikely(rc))
			return rc;
		/* Enable LP Oscillator to run from counter and turn on
		   de-bounce clock. */
		rc = dw3000_reg_or32(
			dw, DW3000_CLK_CTRL_ID, 0,
			DW3000_CLK_CTRL_LP_CLK_EN_BIT_MASK |
				DW3000_CLK_CTRL_GPIO_DBNC_CLK_EN_BIT_MASK);
		if (unlikely(rc))
			return rc;
		/* Enable LEDs to blink and set default blink time. */
		reg = DW3000_LED_CTRL_BLINK_EN_BIT_MASK |
		      DW3000_LEDS_BLINK_TIME_DEF;
		/* Make LEDs blink once if requested. */
		if (mode & DW3000_LEDS_INIT_BLINK) {
			reg |= DW3000_LED_CTRL_FORCE_TRIGGER_BIT_MASK;
		}
		rc = dw3000_reg_write32(dw, DW3000_LED_CTRL_ID, 0, reg);
		/* Clear force blink bits if needed. */
		if (!rc && (mode & DW3000_LEDS_INIT_BLINK)) {
			reg &= ~DW3000_LED_CTRL_FORCE_TRIGGER_BIT_MASK;
			rc = dw3000_reg_write32(dw, DW3000_LED_CTRL_ID, 0, reg);
		}
	} else {
		/* Clear the GPIO bits that are used for LED control. */
		rc = dw3000_set_gpio_mode(dw, gpio_mask, 0);
		if (unlikely(rc))
			return rc;
		rc = dw3000_reg_and16(dw, DW3000_LED_CTRL_ID, 0,
				      (u16)~DW3000_LED_CTRL_BLINK_EN_BIT_MASK);
	}
	return rc;
}

/**
 * dw3000_config_antenna_gpio() - Set configuration for the given GPIO
 * @dw: The DW device.
 * @gpio: GPIO number to configure for antenna switching.
 *
 * The given GPIO is configured for GPIO mode in output direction.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_config_antenna_gpio(struct dw3000 *dw, int gpio)
{
	int rc = 0;
	u32 modemask;
	u32 modegpio;
	u16 dirmask;

	if (gpio < 4)
		modegpio = 2 << (DW3000_GPIO_MODE_MSGP0_MODE_BIT_LEN * gpio);
	else if (gpio > 6)
		modegpio = 1 << (DW3000_GPIO_MODE_MSGP0_MODE_BIT_LEN * gpio);
	else
		modegpio = 0;
	/* Configure selected GPIO for GPIO mode */
	modemask = DW3000_GPIO_MODE_MSGP0_MODE_BIT_MASK
		   << (DW3000_GPIO_MODE_MSGP0_MODE_BIT_LEN * gpio);
	rc = dw3000_set_gpio_mode(dw, modemask, modegpio);
	if (rc)
		return rc;
	/* Configure selected GPIO for output direction */
	dirmask = DW3000_GPIO_DIR_GDP0_BIT_MASK
		  << (DW3000_GPIO_DIR_GDP0_BIT_LEN * gpio);
	rc = dw3000_set_gpio_dir(dw, dirmask, 0);
	return rc;
}

/**
 * dw3000_config_antenna_gpios() - Set configuration for all used GPIO
 * @dw: The DW device.
 *
 * This function configure all GPIO found in antenna table.
 * It is called before enabling the DW device in start() MCPS API.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_config_antenna_gpios(struct dw3000 *dw)
{
	char used_gpios[DW3000_GPIO_COUNT] = { 0 };
	int rc, i;
	/* Read all used GPIO used as antenna selectors */
	for (i = 0; i < DW3000_CALIBRATION_ANTENNA_MAX; i++) {
		u8 selector = dw->calib_data.ant[i].selector_gpio;
		if ((selector < DW3000_GPIO_COUNT) && !used_gpios[selector]) {
			/* Ensure selected GPIO is well configured */
			rc = dw3000_config_antenna_gpio(dw, selector);
			if (rc)
				return rc;
			/* Ensure it is configured only once in this loop */
			used_gpios[selector] = 1;
		}
	}
	return 0;
}

/**
 * dw3000_set_antenna_gpio() - Set GPIO for the given antenna
 * @dw: The DW device.
 * @ant_calib: Calibration data for the selected antenna.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_set_antenna_gpio(struct dw3000 *dw,
				   struct dw3000_antenna_calib *ant_calib)
{
	int gpio = ant_calib->selector_gpio;
	int value = ant_calib->selector_gpio_value;
	int rc = 0;
	if (gpio < DW3000_GPIO_COUNT) {
		int offset = DW3000_GPIO_OUT_GOP0_BIT_LEN * gpio;
		/* Set GPIO state according config to select this antenna */
		rc = dw3000_set_gpio_out(dw, !value << offset, value << offset);
	}
	trace_dw3000_set_antenna_gpio(dw, rc, gpio, value);
	return rc;
}

/**
 * dw3000_set_tx_antenna() - Configure device to use selected antenna for TX
 * @dw: The DW device.
 * @antidx: The antenna to use for next TX
 *
 * Prepare the DW device to transmit frame using the specified antenna.
 * The required HW information (port, gpio and gpio value) must be set
 * correctly inside calibration data structure.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_set_tx_antenna(struct dw3000 *dw, int antidx)
{
	struct dw3000_config *config = &dw->config;
	struct dw3000_antenna_calib *ant_calib;
	int rc;
	/* Early return if no change */
	if (antidx == config->ant[0])
		return 0;
	/* Retrieve antenna GPIO configuration from calibration data */
	ant_calib = &dw->calib_data.ant[antidx];
	if (ant_calib->port != 0) {
		/* TX always use RF1 port */
		dev_warn(
			dw->dev,
			"Bad antenna selected or bad configuration ant=%d, port=%d\n",
			antidx, ant_calib->port);
		return -EINVAL;
	}
	/* Set GPIO state according config to select this antenna */
	rc = dw3000_set_antenna_gpio(dw, ant_calib);
	if (rc)
		return rc;
	config->ant[0] = antidx;
	/* Switching antenna require changing some calibration parameters */
	return dw3000_calib_update_config(dw);
}

/**
 * dw3000_set_rx_antennas() - Set GPIOs to use selected antennas for RX
 * @dw: The DW device.
 * @ant_pair: The antennas pair to use
 * @pdoa_enabled: True if PDoA is enabled
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_set_rx_antennas(struct dw3000 *dw, int ant_pair, bool pdoa_enabled)
{
	struct dw3000_config *config = &dw->config;
	struct dw3000_antenna_calib *ant_calib;
	int rc = 0, port, changed = 0;
	u8 antidx1, antidx2;
	/* Sanity checks first */
	if (ant_pair < 0 || ant_pair >= ANTPAIR_MAX)
		return -EINVAL;
	/* Retrieve first antenna GPIO configuration from calibration data */
	dw3000_calib_antpair_to_ant(ant_pair, &antidx1, &antidx2);
	ant_calib = &dw->calib_data.ant[antidx1];
	port = ant_calib->port; /* Save port for later check */
	if (antidx1 != config->ant[port]) {
		/* Set GPIO state according config for this first antenna */
		rc = dw3000_set_antenna_gpio(dw, ant_calib);
		if (rc)
			return rc;
		config->ant[port] = antidx1;
		changed++;
	}
	/* Retrieve second antenna GPIO configuration from calibration data */
	if (pdoa_enabled) {
		ant_calib = &dw->calib_data.ant[antidx2];
		if (port == ant_calib->port) {
			/* Specified RX antenna must be on different port */
			dev_warn(
				dw->dev,
				"Bad antennas selected or bad configuration ant1=%d (port=%d), ant2=%d (port=%d)\n",
				antidx1, port, antidx2, ant_calib->port);
			return -EINVAL;
		}
		port = ant_calib->port;
		if (antidx2 != config->ant[port]) {
			/* Set GPIO state according config for this first antenna */
			rc = dw3000_set_antenna_gpio(dw, ant_calib);
			if (rc)
				return rc;
			config->ant[port] = antidx2;
			changed++;
		}
	}
	/* Switching antenna require changing some calibration parameters */
	if (changed)
		rc = dw3000_calib_update_config(dw);
	return rc;
}

/**
 * dw3000_initialise() - Initialise the DW local data.
 * @dw: The DW device.
 *
 * Make sure the local data is completely reset before starting initialisation.
 */
static void dw3000_initialise(struct dw3000 *dw)
{
	struct dw3000_local_data *local = &dw->data;
	struct dw3000_stats *stats = &dw->stats;
	struct dw3000_deep_sleep_state *dss = &dw->deep_sleep_state;

	/* Double buffer mode off by default / clear the flag */
	local->dblbuffon = DW3000_DBL_BUFF_OFF;
	local->sleep_mode = DW3000_RUNSAR;
	local->spicrc = DW3000_SPI_CRC_MODE_NO;
	/* Clear all register cache variables */
	local->rx_timeout_pac = 0;
	local->w4r_time = 0;
	local->ack_time = 0;
	local->tx_fctrl = 0;
	/* Initialise statistics, disabled by default, can be enabled by module
	 * parameter on load, or via testmode */
	stats->enabled = dw3000_stats_enabled;
	memset(stats->count, 0, sizeof(stats->count));
	/* Reset Deep Sleep state */
	INIT_WORK(&dw->timer_expired_work, dw3000_timer_expired);
#ifdef CONFIG_DW3000_DEBUG
	if (dss->regbackup)
		kfree(dss->regbackup);
	memset(dss, 0, sizeof(*dss));
	dss->regbackup = kzalloc(1024, GFP_KERNEL);
	INIT_WORK(&dss->compare_work, dw3000_wakeup_compare);
#else
	memset(dss, 0, sizeof(*dss));
#endif
	/* Reset time origin for DTU calculation */
	dw->time_zero_ns = ktime_get_boottime_ns();
}

/**
 * dw3000_transfers_free() - Free allocated SPI messages
 * @dw: the DW device to free the SPI messages
 */
void dw3000_transfers_free(struct dw3000 *dw)
{
	/* fast command message, only one transfer */
	dw3000_free_fastcmd(dw->msg_fast_command);
	dw->msg_fast_command = NULL;
	/* specific pre-computed read/write full-duplex messages */
	dw3000_free_xfer(dw->msg_read_rdb_status, 1);
	dw->msg_read_rdb_status = NULL;
	dw3000_free_xfer(dw->msg_read_rx_timestamp, 1);
	dw->msg_read_rx_timestamp = NULL;
	dw3000_free_xfer(dw->msg_read_rx_timestamp_a, 1);
	dw->msg_read_rx_timestamp_a = NULL;
	dw3000_free_xfer(dw->msg_read_rx_timestamp_b, 1);
	dw->msg_read_rx_timestamp_b = NULL;
	dw3000_free_xfer(dw->msg_read_sys_status, 1);
	dw->msg_read_sys_status = NULL;
	dw3000_free_xfer(dw->msg_read_all_sys_status, 1);
	dw->msg_read_all_sys_status = NULL;
	dw3000_free_xfer(dw->msg_read_sys_time, 1);
	dw->msg_read_sys_time = NULL;
	dw3000_free_xfer(dw->msg_write_sys_status, 1);
	dw->msg_write_sys_status = NULL;
	dw3000_free_xfer(dw->msg_write_all_sys_status, 1);
	dw->msg_write_all_sys_status = NULL;
	dw3000_free_xfer(dw->msg_read_dss_status, 1);
	dw->msg_read_dss_status = NULL;
	dw3000_free_xfer(dw->msg_write_dss_status, 1);
	dw->msg_write_dss_status = NULL;
	dw3000_free_xfer(dw->msg_write_spi_collision_status, 1);
	dw->msg_write_spi_collision_status = NULL;
	/* generic read/write full-duplex message */
	dw3000_free_xfer(dw->msg_readwrite_fdx, 1);
	dw->msg_readwrite_fdx = NULL;
}

/**
 * dw3000_transfers_init() - Allocate SPI messages
 * @dw: the DW device to allocate SPI message for
 *
 * Allocate and pre-compute SPI messages to allow minimum overhead
 * each time a specific read/write operation occurs.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_transfers_init(struct dw3000 *dw)
{
	/* fast command message, only one transfer */
	dw->msg_fast_command = dw3000_alloc_prepare_fastcmd();
	if (!dw->msg_fast_command)
		goto alloc_err;
	/* specific pre-computed read/write full-duplex messages */
	dw->msg_read_rdb_status = dw3000_alloc_prepare_xfer(
		dw, DW3000_RDB_STATUS_ID, 0, 1, DW3000_SPI_RD_BIT);
	if (!dw->msg_read_rdb_status)
		goto alloc_err;
	dw->msg_read_rx_timestamp = dw3000_alloc_prepare_xfer(
		dw, DW3000_RX_TIME_0_ID, 0, DW3000_RX_TIME_RX_STAMP_LEN,
		DW3000_SPI_RD_BIT);
	if (!dw->msg_read_rx_timestamp)
		goto alloc_err;
	dw->msg_read_rx_timestamp_a = dw3000_alloc_prepare_xfer(
		dw, DW3000_DB_DIAG_SET_1, DW3000_DB_DIAG_RX_TIME,
		DW3000_RX_TIME_RX_STAMP_LEN, DW3000_SPI_RD_BIT);
	if (!dw->msg_read_rx_timestamp_a)
		goto alloc_err;
	dw->msg_read_rx_timestamp_b = dw3000_alloc_prepare_xfer(
		dw, DW3000_INDIRECT_POINTER_B_ID, DW3000_DB_DIAG_RX_TIME,
		DW3000_RX_TIME_RX_STAMP_LEN, DW3000_SPI_RD_BIT);
	if (!dw->msg_read_rx_timestamp_b)
		goto alloc_err;
	dw->msg_read_sys_status = dw3000_alloc_prepare_xfer(
		dw, DW3000_SYS_STATUS_ID, 0, 4, DW3000_SPI_RD_BIT);
	if (!dw->msg_read_sys_status)
		goto alloc_err;
	dw->msg_read_all_sys_status = dw3000_alloc_prepare_xfer(
		dw, DW3000_SYS_STATUS_ID, 0, 8, DW3000_SPI_RD_BIT);
	if (!dw->msg_read_sys_status)
		goto alloc_err;
	dw->msg_read_sys_time = dw3000_alloc_prepare_xfer(
		dw, DW3000_SYS_TIME_ID, 0, 4, DW3000_SPI_RD_BIT);
	if (!dw->msg_read_sys_time)
		goto alloc_err;
	dw->msg_write_sys_status = dw3000_alloc_prepare_xfer(
		dw, DW3000_SYS_STATUS_ID, 0, 4, DW3000_SPI_WR_BIT);
	if (!dw->msg_write_sys_status)
		goto alloc_err;
	dw->msg_write_all_sys_status = dw3000_alloc_prepare_xfer(
		dw, DW3000_SYS_STATUS_ID, 0, 8, DW3000_SPI_WR_BIT);
	if (!dw->msg_write_all_sys_status)
		goto alloc_err;
	dw->msg_read_dss_status = dw3000_alloc_prepare_xfer(
		dw, DW3000_DSS_STAT_ID, 0, 1, DW3000_SPI_RD_BIT);
	if (!dw->msg_read_dss_status)
		goto alloc_err;
	dw->msg_write_dss_status = dw3000_alloc_prepare_xfer(
		dw, DW3000_DSS_STAT_ID, 0, 1, DW3000_SPI_WR_BIT);
	if (!dw->msg_write_dss_status)
		goto alloc_err;
	dw->msg_write_spi_collision_status = dw3000_alloc_prepare_xfer(
		dw, DW3000_SPI_COLLISION_STATUS_ID, 0, 1, DW3000_SPI_WR_BIT);
	if (!dw->msg_write_spi_collision_status)
		goto alloc_err;
	/* generic read/write full-duplex message */
	dw->msg_readwrite_fdx =
		dw3000_alloc_prepare_xfer(dw, 0, 0, 16, DW3000_SPI_RD_BIT);
	if (!dw->msg_readwrite_fdx)
		goto alloc_err;
	/* mutex protecting msg_readwrite_fdx */
	mutex_init(&dw->msg_mutex);
	return 0;

alloc_err:
	dw3000_transfers_free(dw);
	return -ENOMEM;
}

/**
 * dw3000_transfers_reset() - Reset allocated SPI messages
 * @dw: the DW device for which SPI messages speed must be reset
 *
 * This ensure ALL transfers are clean, without an existing SPI speed.
 * This function must be called each time after SPI speed is changed.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_transfers_reset(struct dw3000 *dw)
{
	dw3000_transfers_free(dw);
	return dw3000_transfers_init(dw);
}

/**
 * dw3000_init() - Initialise device
 * @dw: DW3000 device
 * @check_idlerc: Check if the device is in IDLE_RC state
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_init(struct dw3000 *dw, bool check_idlerc)
{
	int rc;

	/* Initialise device structure first */
	dw3000_initialise(dw);

	if (check_idlerc) {
		/* The DW IC should be in IDLE_RC state and ready */
		if (!dw3000_check_idlerc(dw)) {
			dev_err(dw->dev, "device not in IDLE_RC state\n");
			return -EINVAL;
		}
	}

	/* Read OTP data (first call only) */
	rc = dw3000_read_otp(dw, DW3000_READ_OTP_PID | DW3000_READ_OTP_LID);
	if (unlikely(rc)) {
		dev_err(dw->dev, "device OTP read has failed (%d)\n", rc);
		return rc;
	}

	/* Read LDO_TUNE and BIAS_TUNE from OTP */
	/* This is device specific */
	rc = dw->chip_ops->prog_ldo_and_bias_tune(dw);
	if (unlikely(rc)) {
		dev_err(dw->dev,
			"device LDO & bias tune setup has failed (%d)\n", rc);
		return rc;
	}
	/* Read and init XTRIM */
	rc = dw3000_prog_xtrim(dw);
	if (unlikely(rc)) {
		dev_err(dw->dev, "device XTRIM setup has failed (%d)\n", rc);
		return rc;
	}
	/* Read and init coarse code from OTP*/
	/* Configure PLL coarse code, if needed. */
	if (dw->chip_ops->prog_pll_coarse_code) {
		rc = dw->chip_ops->prog_pll_coarse_code(dw);
		if (unlikely(rc)) {
			dev_err(dw->dev,
				"device coarse code setup has failed (%d)\n",
				rc);
			return rc;
		}
	}
	/* Ensure STS fields are double-buffered if enabled, also enable stats
	 * if configured in module parameters */
	rc = dw3000_configure_ciadiag(dw, dw->stats.enabled,
				      dw->data.dblbuffon ?
					      DW3000_CIA_DIAG_LOG_DBL_MID :
					      DW3000_CIA_DIAG_LOG_DBL_OFF);
	if (unlikely(rc)) {
		dev_err(dw->dev, "device CIA DIAG setup has failed (%d)\n", rc);
		return rc;
	}
	/* Do some device specific initialisation if any */
	rc = dw->chip_ops->init(dw);
	if (unlikely(rc)) {
		dev_err(dw->dev, "device chip specific init has failed (%d)\n",
			rc);
		return rc;
	}
	/* Configure radio frequency */
	rc = dw3000_configure(dw);
	if (unlikely(rc)) {
		dev_err(dw->dev, "device configuration has failed (%d)\n", rc);
		return rc;
	}
	/* Initialise LEDs */
	rc = dw3000_set_leds(dw, DW3000_LEDS_DISABLE);
	if (unlikely(rc))
		return rc;
	/* Initialise LNA/PA modes */
	rc = dw3000_set_lna_pa_mode(dw, dw->lna_pa_mode);
	if (unlikely(rc))
		return rc;
	/* Configure delays */
	rc = dw3000_set_antenna_delay(dw, 0);
	if (unlikely(rc))
		return rc;
	/* Set auto-ack delay. */
	rc = dw3000_set_autoack_reply_delay(
		dw, DW3000_NUMBER_OF_SYMBOL_DELAY_AUTO_ACK);
	if (unlikely(rc))
		return rc;
	rc = dw3000_disable_autoack(dw, true);
	if (unlikely(rc))
		return rc;
	/* WiFi coexistence initialisation */
	rc = dw3000_coex_init(dw);
	if (unlikely(rc))
		return rc;
	/* Configure antenna selection GPIO if any */
	return dw3000_config_antenna_gpios(dw);
}

void dw3000_remove(struct dw3000 *dw)
{
	struct dw3000_rx *rx = &dw->rx;
	unsigned long flags;

	/* Free RX's socket buffer if not claimed */
	spin_lock_irqsave(&rx->lock, flags);
	if (rx->skb)
		dev_kfree_skb_any(rx->skb);
	rx->skb = NULL;
	spin_unlock_irqrestore(&rx->lock, flags);

	/* Stop device */
	dw3000_disable(dw);
}

/**
 * dw3000_enable() - Enable the device
 * @dw: the DW device
 *
 * This function masks the device's internal interruptions (fixed
 * configuration) then enables the IRQ linked to the device interruption line.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_enable(struct dw3000 *dw)
{
	/* Select the events that will generate an interruption */
	int rc = dw3000_set_interrupt(dw, DW3000_SYS_STATUS_TRX,
				      DW3000_ENABLE_INT_ONLY);
	if (rc)
		return rc;
	/* Enable interrupt that will call the worker */
	enable_irq(dw->spi->irq);

	/* Update interface status */
	atomic_set(&dw->iface_is_started, true);
	return 0;
}

/**
 * dw3000_disable() - Disable the device
 * @dw: the DW device
 *
 * This function disables the IRQ linked to the device interruption line,
 * unmasks the device's internal interruptions then clears its pending
 * interruptions.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_disable(struct dw3000 *dw)
{
	int rc;
	/* Update internal device's status */
	atomic_set(&dw->iface_is_started, false);
	/* Handle DEEP-SLEEP active case early */
	if (dw->current_operational_state < DW3000_OP_STATE_INIT_RC) {
		/* Seems chip is sleeping or already power-off. Just ensure
		   wake-up timer will not fire since not required anymore. */
		dw3000_wakeup_cancel(dw);
		/* No need to call disable_irq() since already called and this
		   will require calling two times enable_irq() later because
		   enable/disable_irq are nested! */
		return 0;
	}
	/* No IRQs after this point */
	disable_irq(dw->spi->irq);
	/* Disable further interrupt generation */
	rc = dw3000_set_interrupt(dw, 0, DW3000_ENABLE_INT_ONLY);
	if (rc)
		goto err_spi;
	/* Release Wifi coexistence. */
	dw3000_coex_stop(dw);
	/* Disable receiver and transmitter */
	rc = dw3000_forcetrxoff(dw);
	if (rc)
		goto err_spi;
	/* Clear pending ALL interrupts */
	rc = dw3000_clear_sys_status(dw, (u32)-1);
	if (rc)
		goto err_spi;
	return 0;

err_spi:
	return rc;
}

void dw3000_init_config(struct dw3000 *dw)
{
	int i, j, k;
	/* Default configuration */
	const struct dw3000_txconfig txconfig = {
		.PGdly = 0x34,
		.PGcount = 0,
		.power = 0xfefefefe,
		.smart = false,
		.testmode_enabled = false,
	};
	const struct dw3000_config config = {
		.chan = 5,
		.txPreambLength = DW3000_PLEN_64,
		.txCode = 9,
		.rxCode = 9,
		.sfdType = DW3000_SFD_TYPE_4Z,
		.dataRate = DW3000_BR_6M8,
		.phrMode = DW3000_PHRMODE_STD,
		.phrRate = DW3000_PHRRATE_STD,
		.sfdTO = DW3000_SFDTOC_DEF,
		.stsMode = DW3000_STS_MODE_OFF | DW3000_STS_MODE_SDC,
		.stsLength = DW3000_STS_LEN_64,
		.pdoaMode = DW3000_PDOA_M0,
		.pdoaOffset = 0,
		.ant = { -1, -1 },
		.antpair_spacing_mm_q11 = DW3000_DEFAULT_ANTPAIR_SPACING,
	};
	/* Set default configuration */
	dw->config = config;
	dw->txconfig = txconfig;
	for (i = 0; i < DW3000_CALIBRATION_ANTENNA_MAX; i++) {
		/* Ensure no GPIO pin are configured by default */
		dw->calib_data.ant[i].selector_gpio = 0xff;
		/* Ensure all antennas have the default antenna delay */
		for (j = 0; j < DW3000_CALIBRATION_CHANNEL_MAX; j++) {
			for (k = 0; k < DW3000_CALIBRATION_PRF_MAX; k++)
				dw->calib_data.ant[i].ch[j].prf[k].ant_delay =
					DW3000_DEFAULT_ANT_DELAY;
		}
	}
	for (i = 0; i < ANTPAIR_MAX; i++) {
		/* Ensure default antennas pair spacing is configured */
		dw->calib_data.antpair[i].spacing_mm_q11 =
			DW3000_DEFAULT_ANTPAIR_SPACING;
		memcpy(dw->calib_data.antpair[i]
			       .ch[DW3000_CALIBRATION_CHANNEL_5]
			       .pdoa_lut,
		       dw3000_default_lut_ch5, sizeof(dw3000_pdoa_lut_t));
		memcpy(dw->calib_data.antpair[i]
			       .ch[DW3000_CALIBRATION_CHANNEL_9]
			       .pdoa_lut,
		       dw3000_default_lut_ch9, sizeof(dw3000_pdoa_lut_t));
	}
	/* Set default antenna ports configuration */
	dw->calib_data.ant[0].port = 0;
	dw->calib_data.ant[1].port = 1;
	/* Ensure power stats timing start at load time */
	dw->power.cur_state = DW3000_PWR_OFF;
	dw->power.stats[DW3000_PWR_OFF].count = 1;
	dw->power.start_time = ktime_get_boottime_ns();
	dw3000_nfcc_coex_init(dw);
	/* Set interface status */
	atomic_set(&dw->iface_is_started, false);

	/* Initialize dw3000_rx spinlock */
	spin_lock_init(&dw->rx.lock);
}

static inline int dw3000_isr_handle_spi_ready(struct dw3000 *dw,
					      struct dw3000_isr_data *isr)
{
	struct dw3000_deep_sleep_state *dss = &dw->deep_sleep_state;
	int rc;

	if (dw->current_operational_state != DW3000_OP_STATE_WAKE_UP) {
		/* Not waking-up from DEEP SLEEP state, just change state and
		   return */
		if (dw->current_operational_state < DW3000_OP_STATE_IDLE_RC &&
		    isr->status & DW3000_SYS_STATUS_SPIRDY_BIT_MASK) {
			/* This is a normal SPIRDY IRQ after power-on. Wake-up
			 * thread waiting for it. Chip is at least in IDLE_RC */
			dw3000_set_operational_state(dw,
						     DW3000_OP_STATE_IDLE_RC);
		}
		return 0;
	}

	/*
	 * Second part of the Wake-up from DEEP SLEEP below.
	 *
	 * TODO: The RX with auto ACK is broken after DEEP SLEEP and WAKEUP.
	 * Please see UWB-1037.
	 */

	/* Update power statistics */
	dw3000_power_stats(dw, DW3000_PWR_RUN, 0);

	/* TODO: A full initialization with dw3000_init() CANNOT be used
	 * because it reset all cached value and won't allow put back DW
	 * in exactly same state than before entering DEEP SLEEP.
	 *
	 * dw3000_configure_chan() can be enough, but it hangs the remote
	 * host on TX. Please see UWB-1032.
	 *
	 * The AON memory was copied to register, so only STS & AES keys
	 * and some others non saved register need to be re-initialised.
	 */

	if (dss->config_changed &
	    (DW3000_CHANNEL_CHANGED | DW3000_PCODE_CHANGED)) {
		/* Channel or preamble code was changed during DEEP-SLEEP.
		 * Need to apply required configuration BEFORE PLL LOCK */
		if (dss->config_changed & DW3000_CHANNEL_CHANGED)
			/* Reconfigure all channel dependent */
			rc = dw3000_configure_chan(dw);
		else if (dss->config_changed & DW3000_PCODE_CHANGED)
			/* Only change CHAN_CTRL with new code */
			rc = dw3000_configure_pcode(dw);
		else
			rc = 0; /* remove uninit variable error */
		if (rc)
			return rc;
		dss->config_changed &=
			~(DW3000_CHANNEL_CHANGED | DW3000_PCODE_CHANGED);
		/* TODO: If channel is changed, the ongoing automatic PLL
		 * locking (see SEQ_CTRL & AON_DIG_CFG) might be stopped!
		 * If required, do it here and let the call below redo it
		 * properly. */
	}

	/* Auto calibrate the PLL and change to IDLE_PLL state */
	rc = dw3000_lock_pll(dw, isr->status);
	if (rc)
		return rc;

	/* Trace Wakeup after DTU/SYS_TIME resync */
	{
		u32 next_date_dtu;
		s64 sleep_ns;
		sleep_ns = dw3000_dtu_to_ktime(dw, dw->dtu_sync) -
			   dw3000_dtu_to_ktime(dw, dw->sleep_enter_dtu);
		next_date_dtu = dss->next_operational_state ==
						DW3000_OP_STATE_RX ?
					dss->rx_info.timestamp_dtu :
					dss->tx_info.timestamp_dtu;
		trace_dw3000_wakeup_done(dw, sleep_ns / 1000,
					 dw->sleep_enter_dtu, next_date_dtu,
					 dss->next_operational_state);
	}

	/* PGF calibration */
	rc = dw3000_pgf_cal(dw, 1);
	if (rc)
		return rc;
	/* DGC LUT */
	rc = dw3000_restore_dgc(dw);
	if (rc)
		return rc;
	/* Calibrate ADC offset, if needed, after DGC configuration and after PLL lock.
	 * If this calibration is executed before the PLL lock, the PLL lock failed.
	 */
	if (dw->chip_ops->adc_offset_calibration) {
		rc = dw->chip_ops->adc_offset_calibration(dw);
		if (rc)
			goto setuperror;
	}
	/* WiFi coexistence initialisation */
	rc = dw3000_coex_init(dw);
	if (rc)
		goto setuperror;
	/* Configure antenna selection GPIO if any */
	rc = dw3000_config_antenna_gpios(dw);
	if (rc)
		goto setuperror;
	/* Reset cached antenna config to ensure GPIO are well reconfigured */
	dw->config.ant[0] = -1;
	dw->config.ant[1] = -1;

	/* Select the events that will generate an interruption */
	rc = dw3000_set_interrupt(dw, DW3000_SYS_STATUS_TRX,
				  DW3000_ENABLE_INT_ONLY);
	if (rc)
		goto setuperror;

	rc = dw3000_reconfigure_hw_addr_filt(dw);
	if (rc)
		goto setuperror;

	if ((dw->config.stsMode & DW3000_STS_BASIC_MODES_MASK) &&
	    !(dw->config.stsMode & DW3000_STS_MODE_SDC)) {
		/* Resend key non-NUL STS KEY */
		__le64 swapped_key[AES_KEYSIZE_128 / sizeof(__le64)];
		_swap128(swapped_key, dw->data.sts_key);
		rc = _dw3000_reg_write(dw, DW3000_STS_KEY_ID, 0,
				       AES_KEYSIZE_128, swapped_key);
		if (rc)
			goto setuperror;
	} else {
		/* STS wasn't activate before entering DEEP-SLEEP, invalidate
		   STS key to ensure it is resent to the chip the next time it
		   is changed by MCPS. */
		memset(dw->data.sts_key, 0, AES_KEYSIZE_128);
		dw->config.stsMode |= DW3000_STS_MODE_SDC;
	}

	/* TODO: So, just add below this line more required unsaved registers
	 * setup. */

setuperror:
#ifdef CONFIG_DW3000_DEBUG
	/* Read and check configuration */
	rc = dw3000_backup_registers(dw, true);
	if (rc)
		return rc;
#endif
	/* Do the required operation after wakeup according states */
	if (dw->nfcc_coex.enabled) {
		rc = dw3000_nfcc_coex_handle_spi1_ready_isr(dw);
	} else if (dss->next_operational_state == DW3000_OP_STATE_RX) {
		/* Entered DEEP SLEEP from dw3000_do_rx_enable() */
		dss->next_operational_state = dw->current_operational_state;
		rc = dw3000_do_rx_enable(dw, &dss->rx_info, dss->frame_idx);
	} else if (dss->next_operational_state == DW3000_OP_STATE_TX) {
		/* Entered DEEP SLEEP from dw3000_do_tx_frame() */
		dss->next_operational_state = dw->current_operational_state;
		rc = dw3000_do_tx_frame(dw, &dss->tx_info, dss->tx_skb,
					dss->frame_idx);
	} else if (dw->call_timer_expired) {
		/* Entered DEEP SLEEP from do_idle() */
		schedule_work(&dw->timer_expired_work);
	}
	return rc;
}

static inline int dw3000_isr_handle_timer_events(struct dw3000 *dw)
{
	/* TODO: call chip specific method if special work is required */
	return 0;
}

static inline int dw3000_isr_handle_spi_error(struct dw3000 *dw)
{
	dev_warn(dw->dev, "no support for callback %s", __func__);
	return 0;
}

/**
 * dw3000_signal_rx_buff_free() - signal the current RX buffer is free
 * @dw: the DW device
 * @dblbuffon: double buffer mode
 *
 * Check if in double buffer mode and if so which buffer host is currently
 * accessing. Then Free up the current buffer and let the device know that
 * it can receive into this buffer again
 *
 * Return: zero on success, else a negative error code.
 */
static inline int dw3000_signal_rx_buff_free(struct dw3000 *dw, u8 *dblbuffon)
{
	if (*dblbuffon != DW3000_DBL_BUFF_OFF) {
		/* Toggle buffer on chip */
		int rc = dw3000_write_fastcmd(dw, DW3000_CMD_DB_TOGGLE);

		if (likely(!rc)) {
			/* Update the current buffer status */
			*dblbuffon ^= DW3000_DBL_BUFF_SWAP;
		}
	}
	return 0;
}

/**
 * dw3000_read_frame_info16() - read the 16-bit frame information
 * @dw: the DW device
 * @dblbuffon: double buffer mode
 * @finfo: the 16-bit value of the RX frame information
 *
 * Check if in double buffer mode and if so which buffer host is
 * currently accessing to clear corresponding DB status register
 * and return lower 16 bits of RX frame info register.
 *
 * Return: zero on success, else a negative error code.
 */
static inline int dw3000_read_frame_info16(struct dw3000 *dw, u8 dblbuffon,
					   u16 *finfo)
{
	int regfile_id;
	int rc;

	switch (dblbuffon) {
	case DW3000_DBL_BUFF_ACCESS_BUFFER_B:
		/* clear DB status register bits corresponding to RX_BUFFER_B */
		rc = dw3000_reg_write8(dw, DW3000_RDB_STATUS_ID, 0, 0x70);
		if (unlikely(rc))
			return rc;
		/* accessing frame info relating to the second buffer (RX_BUFFER_B) */
		regfile_id = DW3000_INDIRECT_POINTER_B_ID;
		break;
	case DW3000_DBL_BUFF_ACCESS_BUFFER_A:
		/* clear DB status register bits corresponding to RX_BUFFER_A */
		rc = dw3000_reg_write8(dw, DW3000_RDB_STATUS_ID, 0, 0x7);
		if (unlikely(rc))
			return rc;
		/* accessing frame info relating to the first buffer (RX_BUFFER_A) */
		regfile_id = DW3000_DB_DIAG_SET_1;
		break;
	default:
		/* accessing frame info in single buffer mode */
		regfile_id = DW3000_RX_FINFO_ID;
		break;
	}
	return dw3000_reg_read16(dw, regfile_id, 0, finfo);
}

static inline int dw3000_isr_handle_rx_call_handler(struct dw3000 *dw,
						    struct dw3000_isr_data *isr)
{
	u32 eof_dtu;
	int rc;

	/* Report statistics */
	rc = dw3000_rx_stats_inc(dw, DW3000_STATS_RX_GOOD);
	if (unlikely(rc))
		return rc;
	/* Store LDE/STS RX errors in rx_flags */
	if (isr->status & DW3000_SYS_STATUS_CIAERR_BIT_MASK)
		isr->rx_flags |= DW3000_RX_FLAG_CER;
	else if (isr->status & DW3000_SYS_STATUS_CIA_DONE_BIT_MASK)
		isr->rx_flags |= DW3000_RX_FLAG_CIA;
	if (isr->status & DW3000_SYS_STATUS_CPERR_BIT_MASK)
		isr->rx_flags |= DW3000_RX_FLAG_CPER;
	/* In case of automatic ack reply. */
	if (isr->status & DW3000_SYS_STATUS_AAT_BIT_MASK)
		isr->rx_flags |= DW3000_RX_FLAG_AACK;
	/* Read frame timestamp */
	rc = dw3000_read_rx_timestamp(dw, &isr->ts_rctu);
	if (unlikely(rc))
		return rc;
	isr->rx_flags |= DW3000_RX_FLAG_TS; /* don't read it again later */
	eof_dtu = (u32)(isr->ts_rctu / DW3000_RCTU_PER_DTU) +
		  dw3000_frame_duration_dtu(dw, isr->datalength, false);
	/* Update power statistics */
	dw3000_power_stats(dw, DW3000_PWR_IDLE, eof_dtu);
	/* Report received frame */
	rc = dw3000_rx_frame(dw, isr);
	if (unlikely(rc))
		return rc;
	/* Handle double buffering */
	return dw3000_signal_rx_buff_free(dw, &dw->data.dblbuffon);
}

static inline int dw3000_isr_handle_rxfcg_event(struct dw3000 *dw,
						struct dw3000_isr_data *isr)
{
	const u32 clear = DW3000_SYS_STATUS_ALL_RX_GOOD |
			  DW3000_SYS_STATUS_CIAERR_BIT_MASK |
			  DW3000_SYS_STATUS_CPERR_BIT_MASK;
	u16 finfo16;
	int rc;

	/* Read frame info, only the first two bytes of the register are used
	   here. */
	rc = dw3000_read_frame_info16(dw, dw->data.dblbuffon, &finfo16);
	if (unlikely(rc)) {
		dev_err(dw->dev, "could not read the frame info : %d\n", rc);
		return rc;
	}
	/* Report frame length, standard frame length up to 127, extended frame
	   length up to 1023 bytes */
	isr->datalength =
		(finfo16 & dw->data.max_frames_len) - IEEE802154_FCS_LEN;
	/* Report ranging bit */
	if (finfo16 & DW3000_RX_FINFO_RNG_BIT_MASK)
		isr->rx_flags = DW3000_RX_FLAG_RNG;
	else
		isr->rx_flags = 0;
	rc = dw3000_isr_handle_rx_call_handler(dw, isr);
	/* Clear errors (as we do not want to go back into cbRxErr) */
	isr->status &= ~clear;
	return rc;
}

static inline int dw3000_isr_handle_rxfr_sts_event(struct dw3000 *dw,
						   struct dw3000_isr_data *isr)
{
	const u32 clear = DW3000_SYS_STATUS_ALL_RX_GOOD |
			  DW3000_SYS_STATUS_RXFCE_BIT_MASK |
			  DW3000_SYS_STATUS_CIAERR_BIT_MASK |
			  DW3000_SYS_STATUS_CPERR_BIT_MASK;
	int rc;
	isr->datalength = 0;
	rc = dw3000_isr_handle_rx_call_handler(dw, isr);
	/* Clear errors (as we do not want to go back into cbRxErr) */
	isr->status &= ~clear;
	return rc;
}

static inline int dw3000_isr_handle_rxto_event(struct dw3000 *dw, u32 status)
{
	u32 end_dtu;
	int rc;
	/* Update power statistics */
	end_dtu = dw->power.rx_start + (dw->data.rx_timeout_pac + 1) *
					       dw->chips_per_pac *
					       DW3000_CHIP_PER_DTU;
	dw3000_power_stats(dw, DW3000_PWR_IDLE, end_dtu);
	/* Report statistics */
	rc = dw3000_rx_stats_inc(dw, DW3000_STATS_RX_TO);
	if (unlikely(rc))
		return rc;
	/* Inform upper layer */
	if (status & DW3000_SYS_STATUS_RXFTO_BIT_MASK)
		dev_dbg(dw->dev, "rx frame timeout");
	else
		dev_dbg(dw->dev, "rx preamble timeout");
	mcps802154_rx_timeout(dw->llhw);
	return 0;
}

static inline int dw3000_isr_handle_rxerr_event(struct dw3000 *dw, u32 status)
{
	struct mcps802154_llhw *llhw = dw->llhw;
	enum mcps802154_rx_error_type error;
	int rc;

	/* Update power statistics */
	dw3000_power_stats(dw, DW3000_PWR_IDLE, 0);
	/* Map error to mcps802154_rx_error_type enum */
	if (status & DW3000_SYS_STATUS_RXSTO_BIT_MASK) {
		dev_dbg(dw->dev, "rx sfd timeout\n");
		error = MCPS802154_RX_ERROR_SFD_TIMEOUT;
	} else if (status & DW3000_SYS_STATUS_ARFE_BIT_MASK) {
		u32 time;

		dw3000_reg_read32(dw, DW3000_RX_TIME_0_ID, 0, &time);
		dev_dbg(dw->dev, "rx rejected %08x\n", time);
		error = MCPS802154_RX_ERROR_FILTERED;
	} else if (status & DW3000_SYS_STATUS_RXFCE_BIT_MASK) {
		dev_dbg(dw->dev, "bad checksum\n");
		error = MCPS802154_RX_ERROR_BAD_CKSUM;
	} else if (status & DW3000_SYS_STATUS_RXPHE_BIT_MASK) {
		dev_dbg(dw->dev, "rx phr error\n");
		error = MCPS802154_RX_ERROR_OTHER;
	} else if (status & DW3000_SYS_STATUS_RXFSL_BIT_MASK) {
		dev_dbg(dw->dev, "rx sync loss\n");
		error = MCPS802154_RX_ERROR_OTHER;
	} else {
		dev_dbg(dw->dev, "rx error 0x%x\n", status);
		error = MCPS802154_RX_ERROR_OTHER;
	}
	/* Report statistics */
	rc = dw3000_rx_stats_inc(dw, DW3000_STATS_RX_ERROR);
	if (unlikely(rc))
		return rc;
	/* Report RX error event */
	mcps802154_rx_error(llhw, error);
	return 0;
}

static inline int dw3000_isr_handle_tx_event(struct dw3000 *dw,
					     struct dw3000_isr_data *isr)
{
	/* Update power statistics */
	dw3000_power_stats(dw, DW3000_PWR_IDLE, 0);
	if (dw->data.w4r_time) {
		/* W4R configured, switch automatically to RX state.
		   RX start time is unknown here but we need it, so get current
		   DTU time. This is subject to IRQ delay but don't known better
		   solution here. */
		u32 cur_dtu_time = dw3000_get_dtu_time(dw);
		dw3000_power_stats(dw, DW3000_PWR_RX,
				   cur_dtu_time + dw->data.w4r_time *
							  DW3000_DTU_PER_DLY);
	}
	/* Report completion to MCPS 802.15.4 stack */
	mcps802154_tx_done(dw->llhw);
	/* Clear TXFRS status to not handle it a second time. */
	isr->status &= ~DW3000_SYS_STATUS_TXFRS_BIT_MASK;
	return 0;
}

static inline int dw3000_clear_db_events(struct dw3000 *dw)
{
	struct dw3000_local_data *local = &dw->data;

	if (local->dblbuffon == DW3000_DBL_BUFF_ACCESS_BUFFER_A) {
		/* clear RX events relating to buffer A */
		return dw3000_reg_write8(dw, DW3000_RDB_STATUS_ID, 0,
					 DW3000_RDB_STATUS_CLEAR_BUFF0_EVENTS);
	} else if (local->dblbuffon == DW3000_DBL_BUFF_ACCESS_BUFFER_B) {
		/* clear RX events relating to buffer B */
		return dw3000_reg_write8(dw, DW3000_RDB_STATUS_ID, 0,
					 DW3000_RDB_STATUS_CLEAR_BUFF1_EVENTS);
	}
	return 0;
}

void dw3000_isr(struct dw3000 *dw)
{
	struct dw3000_local_data *local = &dw->data;
	struct dw3000_isr_data isr; /* in-stack */
	int rc = 0;
	bool stsnd = ((dw->config.stsMode & DW3000_STS_BASIC_MODES_MASK) ==
		      DW3000_STS_MODE_ND);
	if (dw3000_stats_enabled)
		dw3000_read_counters(dw);

	/* Don't read if spurious DEEP-SLEEP IRQ */
	if (dw->current_operational_state == DW3000_OP_STATE_DEEP_SLEEP) {
		trace_dw3000_isr(dw, 0);
		return;
	}

	/* Read status register(64bits). */
	rc = dw3000_read_all_sys_status(dw, &isr.status);
	if (rc)
		goto spi_err;
	trace_dw3000_isr(dw, isr.status);
	if (dw->nfcc_coex.enabled) {
		rc = dw3000_read_dss_status(dw, &isr.dss_stat);
		if (rc)
			goto spi_err;
		trace_dw3000_nfcc_coex_isr(dw, isr.dss_stat);
	}
	/* Early clear all status bits since saved locally */
	rc = dw3000_clear_all_sys_status(dw, isr.status);
	if (rc)
		goto spi_err;
	/* RX double-buffering enabled */
	if (local->dblbuffon) {
		u8 status_db;
		/* RDB status register */
		rc = dw3000_read_rdb_status(dw, &status_db);
		if (rc)
			goto spi_err;
		/*
		 * If accessing the second buffer (RX_BUFFER_B then read second
		 * nibble of the DB status reg)
		 */
		if (local->dblbuffon == DW3000_DBL_BUFF_ACCESS_BUFFER_B)
			status_db >>= 4;
		/*
		 * Setting the relevant bits in the main status register
		 * according to RDB status register.
		 */
		if (status_db & DW3000_RDB_STATUS_RXFCG0_BIT_MASK)
			isr.status |= DW3000_SYS_STATUS_RXFCG_BIT_MASK;
		if (status_db & DW3000_RDB_STATUS_RXFR0_BIT_MASK)
			isr.status |= DW3000_SYS_STATUS_RXFR_BIT_MASK;
		if (status_db & DW3000_RDB_STATUS_CIADONE0_BIT_MASK)
			isr.status |= DW3000_SYS_STATUS_CIA_DONE_BIT_MASK;
		if (status_db & DW3000_RDB_STATUS_CP_ERR0_BIT_MASK)
			isr.status |= DW3000_SYS_STATUS_CPERR_BIT_MASK;
		/* We can clear event early since converted to status */
		rc = dw3000_clear_db_events(dw);
		if (unlikely(rc))
			goto spi_err;
	}

	/* If automatic acknowledgement is not enabled, then the AAT status bit
	   must be ignored */
	if (!dw->autoack)
		isr.status &= ~DW3000_SYS_STATUS_AAT_BIT_MASK;

	/* Handle TX confirmation event before RX in case of not an ACK. */
	if ((isr.status & (DW3000_SYS_STATUS_AAT_BIT_MASK |
			   DW3000_SYS_STATUS_TXFRS_BIT_MASK)) ==
	    DW3000_SYS_STATUS_TXFRS_BIT_MASK) {
		/* Report TX completion */
		rc = dw3000_isr_handle_tx_event(dw, &isr);
		if (unlikely(rc))
			goto spi_err;
	}

	/* Handle RX good frame event */
	/* When using No Data STS mode, we do not get RXFCG but RXFR */
	if (stsnd && (isr.status & DW3000_SYS_STATUS_RXFR_BIT_MASK)) {
		rc = dw3000_isr_handle_rxfr_sts_event(dw, &isr);
		if (unlikely(rc))
			goto spi_err;
	} else if (isr.status & DW3000_SYS_STATUS_RXFCG_BIT_MASK) {
		/* Handle RX frame */
		rc = dw3000_isr_handle_rxfcg_event(dw, &isr);
		if (unlikely(rc))
			goto spi_err;
	}

	/* Handle TX confirmation event after RX in case of an ACK. */
	if (isr.status & DW3000_SYS_STATUS_TXFRS_BIT_MASK) {
		/* Report TX completion */
		rc = dw3000_isr_handle_tx_event(dw, &isr);
		if (unlikely(rc))
			goto spi_err;
	}

	/* Handle frame reception/preamble detect timeout events */
	if (isr.status & DW3000_SYS_STATUS_ALL_RX_TO) {
		/* Report RX timeout */
		rc = dw3000_isr_handle_rxto_event(dw, isr.status);
		if (unlikely(rc))
			goto spi_err;
	}

	/* Handle RX errors events */
	if (isr.status & DW3000_SYS_STATUS_ALL_RX_ERR) {
		if (isr.status & DW3000_SYS_STATUS_LCSSERR_BIT_MASK) {
			/* LCSS error will not stop the receiver, however
			   because STS timestamp will be wrong the reception
			   is aborted */
			rc = dw3000_forcetrxoff(dw);
			if (unlikely(rc))
				goto spi_err;
		}
		/* Report RX error */
		rc = dw3000_isr_handle_rxerr_event(dw, isr.status);
		if (unlikely(rc))
			goto spi_err;
	}

	/* Handle SPI CRC errors events */
	if (local->spicrc &&
	    (isr.status & DW3000_SYS_STATUS_SPICRCERR_BIT_MASK)) {
		/* Handle SPI error */
		rc = dw3000_isr_handle_spi_error(dw);
		if (unlikely(rc))
			goto spi_err;
	}

	/* SPI ready and IDLE_RC bit gets set when device powers on, or on wake
	   up */
	if (isr.status & (DW3000_SYS_STATUS_SPIRDY_BIT_MASK |
			  DW3000_SYS_STATUS_RCINIT_BIT_MASK)) {
		/* Handle SPI ready */
		rc = dw3000_isr_handle_spi_ready(dw, &isr);
		if (unlikely(rc))
			goto spi_err;
	}

	/* Handle the SPI1 Available events in CCC mode only */
	if (dw->nfcc_coex.enabled &&
	    (isr.dss_stat & DW3000_DSS_STAT_SPI1_AVAIL_BIT_MASK)) {
		rc = dw3000_clear_dss_status(
			dw, DW3000_DSS_STAT_SPI1_AVAIL_BIT_MASK);
		if (rc)
			goto spi_err;
		/* Handle SPI available. */
		rc = dw3000_nfcc_coex_handle_spi1_avail_isr(dw);
		if (rc)
			goto spi_err;
	}

	/* TIMER0/1 event will also set the SYS_EVENT bit */
	if (isr.status & (DW3000_SYS_STATUS_TIMER0_BIT_MASK |
			  DW3000_SYS_STATUS_TIMER1_BIT_MASK)) {
		/* Handle SPI ready */
		rc = dw3000_isr_handle_timer_events(dw);
		if (unlikely(rc))
			goto spi_err;
	}

	trace_dw3000_return_int(dw, 0);
	return;

spi_err:
	/* TODO: handle SPI error */
	trace_dw3000_return_int(dw, rc);
	return;
}

static u32 dw3000_calc_pgcount(struct dw3000 *dw, u32 pg_delay)
{
	u8 val;
	u16 count;
	int chan = dw->config.chan;

	dw3000_rftx_blocks_autoseq_disable(dw, chan);
	dw3000_enable_rf_tx(dw, chan, 0);
	dw3000_force_clocks(dw, DW3000_FORCE_CLK_SYS_TX);
	usleep_range(DW3000_HARD_RESET_DELAY_US,
		     DW3000_HARD_RESET_DELAY_US + 500);

	dw3000_reg_write8(dw, DW3000_TX_CTRL_HI_ID, 0,
			   pg_delay & DW3000_TX_CTRL_HI_TX_PG_DELAY_BIT_MASK);

	dw3000_reg_write8(dw, DW3000_PGC_CTRL_ID, 0, 0x91);

	do {
		dw3000_reg_read8(dw, DW3000_PGC_CTRL_ID, 0, &val);
	} while (val & DW3000_PGC_CTRL_PGC_START_BIT_MASK);

	dw3000_reg_read16(dw, DW3000_PGC_STATUS_ID, 0, &count);
	count &= DW3000_PGC_STATUS_PG_DELAY_COUNT_BIT_MASK;

	dw3000_reg_write32(dw, DW3000_RF_CTRL_MASK_ID, 0, 0);
	dw3000_rftx_blocks_autoseq_disable(dw, chan);
	dw3000_force_clocks(dw, DW3000_FORCE_CLK_AUTO);

	return (u32) count;
}

static u8 dw3000_calc_bandwithadj(struct dw3000 *dw, int target_count)
{
	int chan = dw->config.chan;
	int timeout = 1000;
	u8 val;

	if (target_count == 0)
		return 0;

	dw3000_rftx_blocks_autoseq_disable(dw, chan);
	dw3000_enable_rf_tx(dw, chan, 0);
	dw3000_force_clocks(dw, DW3000_FORCE_CLK_SYS_TX);

	dw3000_reg_write16(dw, DW3000_PG_CAL_TARGET_ID, 0,
			target_count & DW3000_PG_CAL_TARGET_TARGET_BIT_MASK);

	dw3000_reg_write8(dw, DW3000_PGC_CTRL_ID, 0x0, 0x92);
	do {
		dw3000_reg_read8(dw, DW3000_PGC_CTRL_ID, 0, &val);
	} while((val & DW3000_PGC_CTRL_PGC_START_BIT_MASK) && timeout--);

	/* dw3000_disable_rf */
	dw3000_reg_write32(dw, DW3000_RF_CTRL_MASK_ID, 0, 0);
	dw3000_rftx_blocks_autoseq_disable(dw, chan);
	dw3000_force_clocks(dw, DW3000_FORCE_CLK_AUTO);

	dw3000_reg_read8(dw, DW3000_TX_CTRL_HI_ID, 0, &val);
	val &= DW3000_TX_CTRL_HI_TX_PG_DELAY_BIT_MASK;

	return val;

}

int dw3000_testmode_continuous_tx_start(struct dw3000 *dw, u32 frame_length,
					u32 rate)
{
	int rc;
	int i;
	static u8 tx_buf[DW3000_EXT_FRAME_LEN] = { 0 };

	tx_buf[0] = 0xC5; /* 802.15.4 "blink" frame */

	if (dw->txconfig.smart) {
		rc = dw3000_adjust_tx_power(dw, frame_length);
		if (rc)
			return rc;
	}

	if (frame_length > dw->data.max_frames_len)
		return -EINVAL;

	for (i = 2; i < frame_length - IEEE802154_FCS_LEN; i++)
		tx_buf[i] = i & 0xFF;

	rc = dw3000_enable_rf_tx(dw, dw->config.chan, 1);
	if (rc)
		return rc;
	rc = dw3000_ctrl_rftx_blocks(dw, dw->config.chan,
				     DW3000_RF_CTRL_MASK_ID);
	if (rc)
		return rc;
	rc = dw3000_force_clocks(dw, DW3000_FORCE_CLK_SYS_TX);
	if (rc)
		return rc;

	/* enable repeated frames */
	rc = dw3000_reg_or8(dw, DW3000_TEST_CTRL0_ID, 0x0,
			    DW3000_TEST_CTRL0_TX_PSTM_BIT_MASK);
	if (rc)
		return rc;

	if (rate < 2)
		rate = 2;

	rc = dw3000_reg_write32(dw, DW3000_DX_TIME_ID, 0x0, rate);
	if (rc)
		return rc;
	rc = dw3000_tx_write_data(dw, tx_buf, frame_length, 0);
	if (rc)
		return rc;
	rc = dw3000_writetxfctrl(dw, frame_length, 0, false);
	if (rc)
		return rc;

	trace_dw3000_testmode_continuous_tx_start(dw, dw->config.chan,
						  frame_length, rate);
	/* start TX immediately */
	return dw3000_write_fastcmd(dw, DW3000_CMD_TX);
}

int dw3000_testmode_continuous_tx_stop(struct dw3000 *dw)
{
	int rc;

	trace_dw3000_testmode_continuous_tx_stop(dw);
	/* disable repeated frames */
	rc = dw3000_reg_and8(dw, DW3000_TEST_CTRL0_ID, 0x0,
			     (uint8_t)(~DW3000_TEST_CTRL0_TX_PSTM_BIT_MASK));
	rc |= dw3000_force_clocks(dw, DW3000_FORCE_CLK_AUTO);
	rc |= dw3000_reg_write32(dw, DW3000_LDO_CTRL_ID, 0, 0x00000000);
	/* Disable RF blocks for TX (configure RF_ENABLE_ID reg) */
	rc |= dw3000_reg_write32(dw, DW3000_RF_ENABLE_ID, 0, 0x00000000);
	/* Restore the TXRX switch to auto */
	rc |= dw3000_reg_write32(dw, DW3000_RF_SWITCH_CTRL_ID, 0x0,
				 DW3000_TXRXSWITCH_AUTO);
	rc |= dw3000_reg_write32(dw, DW3000_RF_CTRL_MASK_ID, 0x0, 0x00000000);
	return rc;
}

static int dw3000_spi_tests;
module_param_named(spitests, dw3000_spi_tests, int, 0644);
MODULE_PARM_DESC(spitests, "Activate SPI & GPIO test mode loop in RT thread");

bool dw3000_spitests_enabled(struct dw3000 *dw)
{
	((void)dw);
	return dw3000_spi_tests != 0;
}

void dw3000_spitests(struct dw3000 *dw)
{
	const int count = 16384;
	u32 mode_mask, dir_mask;
	int test = 0;

	if (!dw3000_spi_tests)
		return;

	perf_event_create_all(dw);

	/* Setup DW3000 GPIO 4-6 in GPIO mode in output direction */
	mode_mask = (DW3000_GPIO_MODE_MSGP4_MODE_BIT_MASK |
		     DW3000_GPIO_MODE_MSGP5_MODE_BIT_MASK |
		     DW3000_GPIO_MODE_MSGP6_MODE_BIT_MASK);
	dir_mask =
		(DW3000_GPIO_DIR_GDP6_BIT_MASK | DW3000_GPIO_DIR_GDP5_BIT_MASK |
		 DW3000_GPIO_DIR_GDP4_BIT_MASK);
	dw3000_set_gpio_mode(dw, mode_mask, 0);
	dw3000_set_gpio_dir(dw, dir_mask, 0);

	/* Loop until SPI test mode is disabled */
	while (dw3000_spi_tests) {
		u64 perfval[PERF_EVT_COUNT];
		u64 start, duration;
		u32 status = 0;
		int i;
		/* Bypass current test if not selected */
		if (!(dw3000_spi_tests & (1 << test))) {
			test = (test + 1) % 3;
			continue;
		}
		dev_warn(dw->dev, "test mode: start test %d\n", test);
		start = get_jiffies_64();
		perf_event_start_all();
		dw3000_set_gpio_out(
			dw, 0, 1 << (test + DW3000_GPIO_DIR_GDP4_BIT_OFFSET));
		switch (test) {
		case 0:
			/* 32bit register read loop */
			for (i = 0; i < count; i++)
				dw3000_reg_read_fast(dw, DW3000_SYS_STATUS_ID,
						     0, sizeof(status),
						     &status);
			break;
		case 1:
			/* 32bit optimised read loop */
			for (i = 0; i < count; i++)
				dw3000_read_sys_status(dw, &status);
			break;
		case 2:
			/* 32bit generic read loop */
			for (i = 0; i < count; i++)
				dw3000_xfer(dw, DW3000_SYS_STATUS_ID, 0,
					    sizeof(status), &status,
					    DW3000_SPI_RD_BIT);
			break;
		}
		dw3000_set_gpio_out(
			dw, 1 << (test + DW3000_GPIO_DIR_GDP4_BIT_OFFSET), 0);
		perf_event_stop_all(perfval);
		duration = jiffies_to_usecs(get_jiffies_64() - start);
		dev_warn(
			dw->dev,
			"test mode: test %d done in %llu ms, %llu us per read (status %x)\n",
			test, duration / 1000, duration / count, status);
		for (i = 0; i < PERF_EVT_COUNT; i++)
			dev_warn(dw->dev, "\t%s: %llu\n", perf_hw_evt_name[i],
				 perfval[i]);
		/* Set next test */
		test = (test + 1) % 3;
	}
	perf_event_release_all();
}

static ssize_t dw3000_sysfs_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	struct dw3000 *dw = container_of(kobj, struct dw3000, sysfs_power_dir);
	u64 idle_dur, rx_ns, tx_ns;
	int ret;
	if (dw->power.cur_state <= DW3000_PWR_IDLE)
		dw3000_power_stats(dw, dw->power.cur_state, 0);
	/* TX/RX are kept in DTU unit. Convert it here to limit conversion error */
	rx_ns = dw->power.stats[DW3000_PWR_RX].dur * 10000 /
		(DW3000_DTU_FREQ / 100000);
	tx_ns = dw->power.stats[DW3000_PWR_TX].dur * 10000 /
		(DW3000_DTU_FREQ / 100000);
	idle_dur = dw->power.stats[DW3000_PWR_RUN].dur - tx_ns - rx_ns;
	ret = scnprintf(buf, PAGE_SIZE,
			"Off state:\n\tcount:\t%llu\n\tdur ns:\t%llu\n"
			"Deep sleep state:\n\tcount:\t%llu\n\tdur ns:\t%llu\n"
			"Run state:\n\tcount:\t%llu\n\tdur ns:\t%llu\n"
			"Idle state:\n\tcount:\t%llu\n\tdur ns:\t%llu\n"
			"Tx state:\n\tcount:\t%llu\n\tdur ns:\t%llu\n"
			"Rx state:\n\tcount:\t%llu\n\tdur ns:\t%llu\n",
			dw->power.stats[DW3000_PWR_OFF].count,
			dw->power.stats[DW3000_PWR_OFF].dur,
			dw->power.stats[DW3000_PWR_DEEPSLEEP].count,
			dw->power.stats[DW3000_PWR_DEEPSLEEP].dur,
			dw->power.stats[DW3000_PWR_RUN].count,
			dw->power.stats[DW3000_PWR_RUN].dur,
			dw->power.stats[DW3000_PWR_IDLE].count, idle_dur,
			dw->power.stats[DW3000_PWR_TX].count, tx_ns,
			dw->power.stats[DW3000_PWR_RX].count, rx_ns);
	return ret;
}

static ssize_t dw3000_sysfs_store(struct kobject *kobj,
				  struct kobj_attribute *attr, const char *buf,
				  size_t length)
{
	struct dw3000 *dw = container_of(kobj, struct dw3000, sysfs_power_dir);
	if (attr == &dw3000_attribute) {
		/* Reset statistics on buffer write */
		int cstate = min(dw->power.cur_state, DW3000_PWR_RUN);
		memset(dw->power.stats, 0, sizeof(dw->power.stats));
		dw->power.start_time = ktime_get_boottime_ns();
		dw->power.stats[cstate].count = 1;
		if (dw->power.cur_state > DW3000_PWR_RUN)
			dw->power.stats[dw->power.cur_state].count = 1;
	}
	return length;
}

static struct kobj_type dw3000_kobj_type = {
	.sysfs_ops = &kobj_sysfs_ops,
};

void dw3000_sysfs_init(struct dw3000 *dw)
{
	int rc;
	dev_dbg(dw->dev, "creating sysfs\n");
	rc = kobject_init_and_add(&dw->sysfs_power_dir, &dw3000_kobj_type,
				  &dw->dev->kobj, "uwb");
	if (rc)
		return;
	rc = sysfs_create_file(&dw->sysfs_power_dir, &dw3000_attribute.attr);
	if (rc)
		kobject_del(&dw->sysfs_power_dir);
}

void dw3000_sysfs_remove(struct dw3000 *dw)
{
	if (dw->sysfs_power_dir.state_in_sysfs) {
		sysfs_remove_file(&dw->sysfs_power_dir, &dw3000_attribute.attr);
		kobject_del(&dw->sysfs_power_dir);
	}
}
