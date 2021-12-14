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
#ifndef __DW3000_COEX_H
#define __DW3000_COEX_H

#include "dw3000.h"

#define COEX_TIME_US (dw->coex_delay_us)
#define COEX_MARGIN_US (dw->coex_margin_us)

static inline int dw3000_coex_stop(struct dw3000 *dw);

/**
 * dw3000_coex_gpio - change the state of the GPIO used for WiFi coexistence
 * @dw: the DW device
 * @state: the new GPIO state to set
 * @delay_us: delay before toggling the GPIO.
 *
 * This function only call the version dependent coex_gpio function.
 *
 * It cannot be called if dw3000_coex_init() has fail or if no coex gpio
 * is defined. So no need to test chip_ops.
 *
 * Return: 0 on success, else a negative error code.
 */
static inline int dw3000_coex_gpio(struct dw3000 *dw, int state, int delay_us)
{
	int ret;

	ret = dw->chip_ops->coex_gpio(dw, state, delay_us);
	if (!ret)
		dw->coex_status = state;

	return ret;
}

/**
 * dw3000_coex_start - Handle WiFi coex gpio at start of uwb exchange.
 * @dw: the DW device
 * @trx_delayed: pointer to tx/rx_delayed parameter to update
 * @trx_date_dtu: pointer to tx/rx_date_dtu parameter to update
 * @cur_time_dtu: current device time in DTU
 *
 * Return: 0 on success, else a negative error code.
 */
static inline int dw3000_coex_start(struct dw3000 *dw, bool *trx_delayed,
				    u32 *trx_date_dtu, u32 cur_time_dtu)
{
	int delay_us;

	if (dw->coex_gpio < 0)
		return 0;
	delay_us = COEX_TIME_US + COEX_MARGIN_US;
	if (*trx_delayed == false) {
		/* Change to delayed TX/RX with the configured delay */
		*trx_date_dtu = cur_time_dtu + US_TO_DTU(delay_us);
		*trx_delayed = true;
		/* Set gpio now */
		delay_us = 0;
	} else {
		/* Calculate when we need to toggle the gpio */
		int time_difference_dtu = *trx_date_dtu - cur_time_dtu;
		int time_difference_us = DTU_TO_US(time_difference_dtu);
		if (time_difference_us <= delay_us)
			delay_us = 0;
		else
			delay_us = time_difference_us - delay_us;
	}
	trace_dw3000_coex_gpio_start(dw, delay_us, dw->coex_status,
				     dw->coex_interval_us);
	if (dw->coex_status) {
		if (delay_us < dw->coex_interval_us)
			return 0; /* Nothing more to do */
		dw3000_coex_stop(dw);
	}
	/* Set coexistence gpio on chip */
	return dw3000_coex_gpio(dw, true, delay_us);
}

/**
 * dw3000_coex_stop - Handle WiFi coex gpio at end of uwb exchange.
 * @dw: the DW device
 *
 * Return: 0 on success, else a negative error code.
 */
static inline int dw3000_coex_stop(struct dw3000 *dw)
{
	if (dw->coex_gpio < 0)
		return 0;

	trace_dw3000_coex_gpio_stop(dw, dw->coex_status);
	if (!dw->coex_status)
		return 0;
	/* Reset coex GPIO on chip */
	return dw3000_coex_gpio(dw, false, 0);
}

/**
 * dw3000_coex_init - Initialise WiFi coex gpio
 * @dw: the DW device
 *
 * Return: 0 on success, else a negative error code.
 */
static inline int dw3000_coex_init(struct dw3000 *dw)
{
	int rc;

	if (dw->coex_gpio < 0)
		return 0;
	/* Sanity check chip dependent functions */
	if (!dw->chip_ops || !dw->chip_ops->coex_gpio ||
	    !dw->chip_ops->coex_init)
		return -ENOTSUPP;
	/* Call chip dependent initialisation */
	rc = dw->chip_ops->coex_init(dw);
	if (unlikely(rc)) {
		dev_err(dw->dev,
			"WiFi coexistence configuration has failed (%d)\n", rc);
	}
	dw->coex_status = false;
	return rc;
}

#endif /* __DW3000_COEX_H */
