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

#ifndef NET_VENDOR_CMD_H
#define NET_VENDOR_CMD_H

#include <linux/kernel.h>

/**
 * enum dw3000_vendor_cmd - Vendor command identifiers.
 * @DW3000_VENDOR_CMD_NFCC_COEX_HANDLE_ACCESS:
 *     NFCC Coex: handle access.
 * @DW3000_VENDOR_CMD_NFCC_COEX_GET_ACCESS_INFORMATION:
 *     NFCC Coex: get access information.
 * @DW3000_VENDOR_CMD_NFCC_COEX_STOP:
 *     NFCC Coex: stop.
 */
enum dw3000_vendor_cmd {
	DW3000_VENDOR_CMD_NFCC_COEX_HANDLE_ACCESS,
	DW3000_VENDOR_CMD_NFCC_COEX_GET_ACCESS_INFORMATION,
	DW3000_VENDOR_CMD_NFCC_COEX_STOP,
};

/**
 * struct dw3000_vendor_cmd_nfcc_coex_handle_access - NFCC Coex: handle access
 * vendor command.
 */
struct dw3000_vendor_cmd_nfcc_coex_handle_access {
	/**
	 * @start: True to start a new session.
	 */
	bool start;
	/**
	 * @timestamp_dtu:
	 *     Access start date. If this is a new session, this also defines
	 *     TIME0.
	 */
	u32 timestamp_dtu;
	/**
	 * @duration_dtu:
	 *     Duration of the access, or 0 if unknown (this is the case when
	 *     starting a new session).
	 */
	int duration_dtu;
	/**
	 * @chan: Channel number, 5 or 9.
	 */
	int chan;
};

/**
 * struct dw3000_vendor_cmd_nfcc_coex_get_access_info - NFCC Coex: get access
 * info vendor command.
 */
struct dw3000_vendor_cmd_nfcc_coex_get_access_info {
	/**
	 * @stop: If true, the NFCC did not give a next access.
	 */
	bool stop;
	/**
	 * @watchdog_timeout:
	 *     If true, Watchdog triggered before NFCC released the SPI.
	 */
	bool watchdog_timeout;
	/**
	 * @duration_dtu:
	 *     Effective duration of the access. If 0, the current date will be
	 *     read to continue.
	 */
	int duration_dtu;
	/**
	 * @next_timestamp_dtu: Next access date.
	 */
	u32 next_timestamp_dtu;
	/**
	 * @next_duration_dtu: Next access duration, or 0 if unknown.
	 */
	int next_duration_dtu;
};

#endif /* NET_VENDOR_CMD_H */
