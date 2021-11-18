/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020 Qorvo US, Inc.
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
 * Qorvo.
 * Please contact Qorvo to inquire about licensing terms.
 */
#include "dw3000.h"
#include "dw3000_core.h"
#include "dw3000_core_reg.h"

static const struct dw3000_chip_register c0_registers[] = {
	/* registres virtuels pour dump des fileID */
	{ "GEN_CFG0   ", 0x00, 0x79, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "GEN_CFG1   ", 0x01, 0x64, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "STS_CFG    ", 0x02, 0x2c, 0, DW3000_CHIPREG_DUMP, NULL },
	/* No fileID 0x03 documented in DW3000 user manual v0.7. */
	{ "EXT_SYNC   ", 0x04, 0x04, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "GPIO_CTRL  ", 0x05, 0x2e, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "DRX_CONF   ", 0x06, 0x10, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "RF_CONF    ", 0x07, 0x4c, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "TX_CAL     ", 0x08, 0x1e, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "FS_CTRL    ", 0x09, 0x15, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "AON        ", 0x0a, 0x15, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "OTP_IF     ", 0x0b, 0x18, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "CIA_1      ", 0x0c, 0x6c, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "CIA_2      ", 0x0d, 0x6c, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "CIA_3      ", 0x0e, 0x20, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "DIG_DIAG   ", 0x0f, 0x51, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "PMSC       ", 0x11, 0x4a, 0, DW3000_CHIPREG_DUMP, NULL },
	/* TODO: RX/TX buffers limited to first 128bytes only.
	   Shall we dump the whole 1024 bytes? */
	{ "RX_BUFFER  ", 0x12, 0x80, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "RX_BUFFER1 ", 0x13, 0x80, 0, DW3000_CHIPREG_DUMP, NULL },
	/* No TX_BUFFER as read isn't supported */
	/* CIR memory require 2 bits configured elsewhere, so removed. */
	{ "SCRATCH_RAM", 0x16, 0x7f, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "AES RAM    ", 0x17, 0x80, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "SET_X      ", 0x18, 0x1d0, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "IN_PTR_CFG ", 0x1f, 0x12, 0, DW3000_CHIPREG_DUMP, NULL },
};
const struct dw3000_chip_register *dw3000_c0_get_registers(struct dw3000 *dw,
							   size_t *count)
{
	*count = ARRAY_SIZE(c0_registers);
	return c0_registers;
}

const u32 *dw3000_c0_get_config_mrxlut_chan(struct dw3000 *dw, u8 channel)
{
	/* Lookup table default values for channel 5 */
	static const u32 dw3000_c0_configmrxlut_ch5[DW3000_CONFIGMRXLUT_MAX] = {
		0x1c0fd, 0x1c47d, 0x1c67d, 0x1c7fd, 0x1cf7d, 0x1cffd, 0x0fffd
	};

	/* Lookup table default values for channel 9 */
	static const u32 dw3000_c0_configmrxlut_ch9[DW3000_CONFIGMRXLUT_MAX] = {
		0x2a07d, 0x2a3fd, 0x2a57d, 0x2a77d, 0x2a7fd, 0x2ad7d, 0x2affd
	};

	switch (channel) {
	case 5:
		return dw3000_c0_configmrxlut_ch5;
	case 9:
		return dw3000_c0_configmrxlut_ch9;
	default:
		return NULL;
	}
}

static int dw3000_c0_softreset(struct dw3000 *dw)
{
	/* Reset HIF, TX, RX and PMSC */
	return dw3000_reg_write8(dw, DW3000_SOFT_RST_ID, 0, DW3000_RESET_ALL);
}

static int dw3000_c0_init(struct dw3000 *dw)
{
	/* TODO */
	return 0;
}

static int dw3000_c0_coex_init(struct dw3000 *dw)
{
	/* TODO */
	return 0;
}

static int dw3000_c0_coex_gpio(struct dw3000 *dw, bool state, int delay_us)
{
	/* TODO */
	return 0;
}

/**
 * dw3000_prog_ldo_and_bias_tune() - Programs the device's LDO and BIAS tuning
 * @dw: The DW device.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_c0_prog_ldo_and_bias_tune(struct dw3000 *dw)
{
	const u16 bias_mask = DW3000_BIAS_CTRL_DIG_BIAS_DAC_ULV_BIT_MASK;
	struct dw3000_local_data *local = &dw->data;
	struct dw3000_otp_data *otp = &dw->otp_data;
	int rc;
	u16 bias_tune = (otp->bias_tune >> 16) & bias_mask;
	if (otp->ldo_tune_lo && otp->ldo_tune_hi && bias_tune) {
		rc = dw3000_reg_or16(dw, DW3000_NVM_CFG_ID, 0,
				     DW3000_LDO_BIAS_KICK);
		if (rc)
			return rc;
		rc = dw3000_reg_modify16(dw, DW3000_BIAS_CTRL_ID, 0, ~bias_mask,
					 bias_tune);
		if (rc)
			return rc;
	}
	local->dgc_otp_set = DW3000_DGC_LOAD_FROM_SW;
	return 0;
}

/**
 * dw3000_c0_pre_read_sys_time() - Ensure SYS_TIME register is cleared
 * @dw: The DW device.
 *
 * On C0 chips, the SYS_TIME register value is latched and any subsequent read
 * will return the same value. To clear the current value in the register an SPI
 * write transaction is necessary, the following read of the SYS_TIME register will
 * return a new value.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_c0_pre_read_sys_time(struct dw3000 *dw)
{
	/* The SPI_COLLISION register is choose to make this SPI write
	 * transaction because it is unused and it is a small 8 bits register.
	 */
	return dw3000_clear_spi_collision_status(
		dw, DW3000_SPI_COLLISION_STATUS_BIT_MASK);
}

const struct dw3000_chip_ops dw3000_chip_c0_ops = {
	.softreset = dw3000_c0_softreset,
	.init = dw3000_c0_init,
	.coex_init = dw3000_c0_coex_init,
	.coex_gpio = dw3000_c0_coex_gpio,
	.prog_ldo_and_bias_tune = dw3000_c0_prog_ldo_and_bias_tune,
	.get_config_mrxlut_chan = dw3000_c0_get_config_mrxlut_chan,
	.pre_read_sys_time = dw3000_c0_pre_read_sys_time,
	.get_registers = dw3000_c0_get_registers,
};
