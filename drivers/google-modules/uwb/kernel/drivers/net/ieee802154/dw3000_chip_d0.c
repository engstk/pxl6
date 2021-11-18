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
#include "dw3000_ccc_mailbox.h"

static const struct dw3000_chip_register d0_registers[] = {
	/* registres virtuels pour dump des fileID */
	{ "GEN_CFG0   ", 0x00, 0x7e, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "GEN_CFG1   ", 0x01, 0x64, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "STS_CFG    ", 0x02, 0x2c, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "RX_TUNE    ", 0x03, 0x54, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "EXT_SYNC   ", 0x04, 0x21, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "GPIO_CTRL  ", 0x05, 0x2e, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "DRX        ", 0x06, 0x2c, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "RF_CONF    ", 0x07, 0x51, 0, DW3000_CHIPREG_DUMP, NULL },
	{ "RF_CAL     ", 0x08, 0x1e, 0, DW3000_CHIPREG_DUMP, NULL },
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

const struct dw3000_chip_register *dw3000_d0_get_registers(struct dw3000 *dw,
							   size_t *count)
{
	*count = ARRAY_SIZE(d0_registers);
	return d0_registers;
}

const u32 *dw3000_d0_get_config_mrxlut_chan(struct dw3000 *dw, u8 channel)
{
	/* Lookup table default values for channel 5 */
	static const u32 dw3000_d0_configmrxlut_ch5[DW3000_CONFIGMRXLUT_MAX] = {
		0x1c0fd, 0x1c43e, 0x1c6be, 0x1c77e, 0x1cf36, 0x1cfb5, 0x1cff5
	};

	/* Lookup table default values for channel 9 */
	static const u32 dw3000_d0_configmrxlut_ch9[DW3000_CONFIGMRXLUT_MAX] = {
		0x2a8fe, 0x2ac36, 0x2a5fe, 0x2af3e, 0x2af7d, 0x2afb5, 0x2afb5
	};

	switch (channel) {
	case 5:
		return dw3000_d0_configmrxlut_ch5;
	case 9:
		return dw3000_d0_configmrxlut_ch9;
	default:
		return NULL;
	}
}

/**
 * dw3000_d0_softreset() - D0 chip specific software reset
 * @dw: The DW device.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_d0_softreset(struct dw3000 *dw)
{
	/* D0 require a FAST command to start soft-reset */
	return dw3000_write_fastcmd(dw, DW3000_CMD_SEMA_RESET);
}

/**
 * dw3000_d0_init() - D0 chip specific initialisation
 * @dw: The DW device.
 *
 * Note: Still used by dw3000_e0_init().
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_d0_init(struct dw3000 *dw)
{
	int rc = 0;

	if (dw->current_operational_state != DW3000_OP_STATE_DEEP_SLEEP) {
		/* Disable CCC Mailbox only if the chip is not in deep sleep.
		 * Else, on wake up, the CCC state will be not detectable. */
		rc = dw3000_nfcc_coex_disable(dw);
	}
	return rc;
}

/**
 * dw3000_d0_coex_init() - Configure the device's WiFi coexistence GPIO
 * @dw: The DW device.
 *
 * Note: Still used by dw3000_e0_coex_init() as GPIO pin need to be configured.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_d0_coex_init(struct dw3000 *dw)
{
	u32 modemask;
	u16 dirmask;
	int rc;

	if (dw->coex_gpio < 0)
		return 0;
	/* Ensure selected GPIO is well configured */
	modemask = DW3000_GPIO_MODE_MSGP0_MODE_BIT_MASK
		   << (DW3000_GPIO_MODE_MSGP0_MODE_BIT_LEN * dw->coex_gpio);
	rc = dw3000_set_gpio_mode(dw, modemask, 0);
	if (rc)
		return rc;
	dirmask = DW3000_GPIO_DIR_GDP0_BIT_MASK
		  << (DW3000_GPIO_DIR_GDP0_BIT_LEN * dw->coex_gpio);
	rc = dw3000_set_gpio_dir(dw, dirmask, 0);
	return rc;
}

/**
 * dw3000_d0_coex_gpio() - Update the device's WiFi coexistence GPIO
 * @dw: The DW device.
 * @state: The WiFi coexistence GPIO state to apply.
 * @delay_us: The delay in us before changing GPIO state.
 *
 * Return: zero on success, else a negative error code.
 */
static int dw3000_d0_coex_gpio(struct dw3000 *dw, bool state, int delay_us)
{
	int offset;
	/* /!\ could be called first with (true, 1000), then before end of 1000
	   microseconds could be called with (false, 0), should handle this case
	   with stopping the timer if any */
	if (delay_us) {
		/* Wait to ensure GPIO is toggle on time */
		if (delay_us > 10)
			usleep_range(delay_us - 10, delay_us);
		else
			udelay(delay_us);
	}
	offset = DW3000_GPIO_OUT_GOP0_BIT_LEN * dw->coex_gpio;
	dw3000_set_gpio_out(dw, !state << offset, state << offset);
	return 0;
}

/**
 * dw3000_d0_prog_ldo_and_bias_tune() - Programs the device's LDO and BIAS tuning
 * @dw: The DW device.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_d0_prog_ldo_and_bias_tune(struct dw3000 *dw)
{
	struct dw3000_local_data *local = &dw->data;
	struct dw3000_otp_data *otp = &dw->otp_data;
	if (otp->ldo_tune_lo && otp->ldo_tune_hi) {
		dw3000_reg_or16(dw, DW3000_NVM_CFG_ID, 0, DW3000_LDO_BIAS_KICK);
		/* Save the kicks for the on-wake configuration */
		local->sleep_mode |= DW3000_LOADLDO;
	}
	/* Use DGC_CFG from OTP */
	local->dgc_otp_set = otp->dgc_addr == DW3000_DGC_CFG0 ?
				     DW3000_DGC_LOAD_FROM_OTP :
				     DW3000_DGC_LOAD_FROM_SW;
	return 0;
}

const struct dw3000_chip_ops dw3000_chip_d0_ops = {
	.softreset = dw3000_d0_softreset,
	.init = dw3000_d0_init,
	.coex_init = dw3000_d0_coex_init,
	.coex_gpio = dw3000_d0_coex_gpio,
	.prog_ldo_and_bias_tune = dw3000_d0_prog_ldo_and_bias_tune,
	.get_config_mrxlut_chan = dw3000_d0_get_config_mrxlut_chan,
	.get_registers = dw3000_d0_get_registers,
};
