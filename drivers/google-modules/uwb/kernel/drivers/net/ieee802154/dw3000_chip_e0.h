/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2021 Qorvo US, Inc.
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
#ifndef __DW3000_CHIP_E0_H
#define __DW3000_CHIP_E0_H

/* Forward declaration */
struct dw3000;

enum dw3000_timer { DW3000_TIMER0 = 0, DW3000_TIMER1 };

enum dw3000_timer_mode { DW3000_SINGLE_MODE = 0, DW3000_REPEAT_MODE };

/* Delay to wait after RX enable to calibrate the ADC on E0 chip */
#define DW3000_E0_ADC_CALIBRATION_DELAY_US (200)

/* Loops to compute the ADC threshold average on E0 chip */
#define DW3000_E0_ADC_THRESHOLD_AVERAGE_LOOPS (4)
/* Time to wait before reading the calibration status register
 * when a calibration from scratch is executed */
#define DW3000_E0_PLL_CALIBRATION_FROM_SCRATCH_DELAY_US (400)

#define DW3000_TIMER_FREQ 38400000

enum dw3000_timer_period {
	/* 38.4 MHz */
	DW3000_TIMER_XTAL_NODIV = 0,
	/* 19.2 MHz */
	DW3000_TIMER_XTAL_DIV2,
	/* 9.6 MHz */
	DW3000_TIMER_XTAL_DIV4,
	/* 4.8 MHz */
	DW3000_TIMER_XTAL_DIV8,
	/* 2.4 MHz */
	DW3000_TIMER_XTAL_DIV16,
	/* 1.2 MHz */
	DW3000_TIMER_XTAL_DIV32,
	/* 0.6 MHz */
	DW3000_TIMER_XTAL_DIV64,
	/* 0.3 MHz */
	DW3000_TIMER_XTAL_DIV128
};

struct dw3000_timer_cfg {
	/* Select the timer frequency (divider) */
	enum dw3000_timer_period divider;
	/* Select the timer mode */
	enum dw3000_timer_mode mode;
	/* Set to '1' to halt this timer on GPIO interrupt */
	u8 gpio_stop;
	/* Configure GPIO for WiFi co-ex */
	u8 coex_out;
};

/* Hardware timer functions */
int dw3000_timers_enable(struct dw3000 *dw);
int dw3000_timers_reset(struct dw3000 *dw);
int dw3000_timers_read_and_clear_events(struct dw3000 *dw, u8 *evt0, u8 *evt1);

int dw3000_timer_configure(struct dw3000 *dw, enum dw3000_timer timer,
			   struct dw3000_timer_cfg *cfg);
int dw3000_timer_set_expiration(struct dw3000 *dw, enum dw3000_timer timer,
				u32 exp);
int dw3000_timer_get_counter(struct dw3000 *dw, enum dw3000_timer timer,
			     u32 *counter);
int dw3000_timer_start(struct dw3000 *dw, enum dw3000_timer timer);

#endif /* __DW3000_CHIP_E0_H */
