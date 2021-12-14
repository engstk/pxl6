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
 * Qorvo. Please contact Qorvo to inquire about licensing terms.
 */
#ifndef __DW3000_NFCC_COEX_TESTMODE_H
#define __DW3000_NFCC_COEX_TESTMODE_H

#include <linux/module.h>

/* Forward declaration. */
struct dw3000;

/**
 * struct dw3000_nfcc_coex_testmode_config - Test config used in testmode.
 */
struct dw3000_nfcc_coex_testmode_config {
	/**
	 * @mode: test mode identifier.
	 */
	enum { DW3000_CCC_TEST_DIRECT,
	       DW3000_CCC_TEST_WAIT,
	       DW3000_CCC_TEST_LATE,
	       DW3000_CCC_TEST_CONFLICT,
	       DW3000_CCC_TEST_SLEEP_OFFSET,
	} mode;
	/**
	 * @margin_ms: Delay in ms to apply after Ranging Round(RR).
	 */
	u32 margin_ms;
	/**
	 * @RRcount: Ranging Round count.
	 */
	u32 RRcount;
	/**
	 * @conflict_slot_idx: slot index with conflict on date.
	 */
	u32 conflict_slot_idx;
	/**
	 * @offset_ms: Duration in ms to add to NFCC base time;
	 */
	u32 offset_ms;
	/**
	 * @channel: Channel number.
	 */
	u8 channel;
	/**
	 * @session_time0: Offset in ms to add to SESSION_TIME0.
	 */
	u32 session_time0;
	/**
	 * @start: Start offset in ms for the first slot.
	 */
	u32 start;
	/**
	 * @end: End offset in ms for the first slot.
	 */
	u32 end;
};

int dw3000_nfcc_coex_testmode_session(struct dw3000 *dw, u8 chan,
				      u32 session_time0, u32 start, u32 end);
int dw3000_nfcc_coex_testmode_config(
	struct dw3000 *dw,
	const struct dw3000_nfcc_coex_testmode_config *testmode_config);

#endif /* __DW3000_NFCC_COEX_TESTMODE_H */
