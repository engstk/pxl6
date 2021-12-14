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
#include "dw3000_nfcc_coex_mcps.h"
#include "dw3000_nfcc_coex_msg.h"
#include "dw3000_nfcc_coex_buffer.h"
#include "dw3000_nfcc_coex_core.h"
#include "dw3000.h"
#include "dw3000_trc.h"
#include "dw3000_core.h"

#define DW3000_NFCC_COEX_WATCHDOG_DEFAULT_DURATION_MS 200

/**
 * dw3000_nfcc_coex_update_access_info() - Update access info cache.
 * @dw: Driver context.
 * @buffer: buffer to read.
 */
static void dw3000_nfcc_coex_update_access_info(
	struct dw3000 *dw, const struct dw3000_nfcc_coex_buffer *buffer)
{
	struct dw3000_nfcc_coex *nfcc_coex = &dw->nfcc_coex;
	struct dw3000_vendor_cmd_nfcc_coex_get_access_info *access_info =
		&dw->nfcc_coex.access_info;
	struct dw3000_nfcc_coex_rx_msg_info rx_msg_info = {};
	int r;

	r = dw3000_nfcc_coex_message_check(dw, buffer, &rx_msg_info);
	if (r)
		goto failure;

	/* Update access_info. */
	access_info->stop = !rx_msg_info.next_slot_found;
	access_info->watchdog_timeout = false;
	if (rx_msg_info.next_slot_found) {
		u32 diff_sys_time;

		/* Build the next date with the difference, because the sys_time is
		 * faster than dtu time. */
		diff_sys_time = rx_msg_info.next_timestamp_sys_time -
				dw3000_dtu_to_sys_time(
					dw, nfcc_coex->session_start_dtu);
		access_info->next_timestamp_dtu =
			nfcc_coex->session_start_dtu +
			(diff_sys_time >> DW3000_DTU_PER_SYS_POWER);
		access_info->next_duration_dtu =
			rx_msg_info.next_duration_sys_time >>
			DW3000_DTU_PER_SYS_POWER;
	}
	return;

failure:
	/* When buffer content is unexpected then request a stop. */
	memset(access_info, 0, sizeof(*access_info));
	access_info->stop = true;
}

/**
 * dw3000_nfcc_coex_spi_avail_handler() - main spi available handler.
 * @dw: Driver context.
 * @buffer: buffer to read.
 *
 * Return: 0 on success, else an error.
 */
static int
dw3000_nfcc_coex_spi_avail_handler(struct dw3000 *dw,
				   const struct dw3000_nfcc_coex_buffer *buffer)
{
	struct dw3000_nfcc_coex *nfcc_coex = &dw->nfcc_coex;

	/* Is watchdog timer running? */
	if (!del_timer(&nfcc_coex->watchdog_timer)) {
		trace_dw3000_nfcc_coex_warn(
			dw, "spi available without timer pending");
		return 0;
	}

	dw3000_nfcc_coex_update_access_info(dw, buffer);

	if (dw->nfcc_coex.access_info.stop) {
		int r = dw3000_nfcc_coex_disable(dw);
		if (r)
			return r;
	}

	mcps802154_tx_done(dw->llhw);
	return 0;
}

/**
 * dw3000_nfcc_coex_handle_access() - handle access to provide to NFCC.
 * @dw: Driver context.
 * @data: Adress of handle access information.
 * @data_len: Number of byte of the data object.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_handle_access(struct dw3000 *dw, void *data,
					  int data_len)
{
	const struct dw3000_vendor_cmd_nfcc_coex_handle_access *info = data;
	struct dw3000_nfcc_coex *nfcc_coex = &dw->nfcc_coex;
	const u32 dtu_per_ms = dw->llhw->dtu_freq_hz / 1000;
	u32 cur_time_dtu;
	u32 diff_dtu, diff_ms;
	int r;

	if (!info || data_len != sizeof(*info))
		return -EINVAL;

	cur_time_dtu = dw3000_get_dtu_time(dw);

	if (timer_pending(&nfcc_coex->watchdog_timer)) {
		trace_dw3000_nfcc_coex_err(dw, "watchdog timer is pending");
		return -EBUSY;
	}

	if (info->start) {
		r = dw3000_nfcc_coex_enable(dw, info->chan,
					    dw3000_nfcc_coex_spi_avail_handler);
		if (r)
			return r;
	}

	/* Save start session date, to retrieve MSB bits lost for next date. */
	nfcc_coex->session_start_dtu = info->timestamp_dtu;
	/* Setup watchdog timer. */
	diff_dtu = info->timestamp_dtu - cur_time_dtu;
	diff_ms = diff_dtu / dtu_per_ms;
	nfcc_coex->watchdog_timer.expires =
		jiffies +
		msecs_to_jiffies(diff_ms +
				 DW3000_NFCC_COEX_WATCHDOG_DEFAULT_DURATION_MS);
	add_timer(&nfcc_coex->watchdog_timer);

	if (diff_dtu > dw3000_nfcc_coex_margin_dtu)
		return dw3000_nfcc_coex_sleep(
			dw, diff_dtu - dw3000_nfcc_coex_margin_dtu);
	return dw3000_nfcc_coex_message_send(dw);
}

/**
 * dw3000_nfcc_coex_get_access_information() - Forward access info cached.
 * @dw: Driver context.
 * @data: Adress where to write access information.
 * @data_len: Number of byte of the data object.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_get_access_information(struct dw3000 *dw,
						   void *data, int data_len)
{
	const struct dw3000_vendor_cmd_nfcc_coex_get_access_info *access_info =
		&dw->nfcc_coex.access_info;

	if (!data || data_len != sizeof(*access_info))
		return -EINVAL;

	memcpy(data, access_info, data_len);
	return 0;
}

/**
 * dw3000_nfcc_coex_stop() - Stop NFCC.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_stop(struct dw3000 *dw)
{
	return dw3000_nfcc_coex_disable(dw);
}

/**
 * dw3000_nfcc_coex_vendor_cmd() - Vendor NFCC coexistence command processing.
 *
 * @dw: Driver context.
 * @vendor_id: Vendor Identifier on 3 bytes.
 * @subcmd: Sub-command identifier.
 * @data: Null or data related with the sub-command.
 * @data_len: Length of the data
 *
 * Return: 0 on success, 1 to request a stop, error on other value.
 */
int dw3000_nfcc_coex_vendor_cmd(struct dw3000 *dw, u32 vendor_id, u32 subcmd,
				void *data, size_t data_len)
{
	switch (subcmd) {
	case DW3000_VENDOR_CMD_NFCC_COEX_HANDLE_ACCESS:
		return dw3000_nfcc_coex_handle_access(dw, data, data_len);
	case DW3000_VENDOR_CMD_NFCC_COEX_GET_ACCESS_INFORMATION:
		return dw3000_nfcc_coex_get_access_information(dw, data,
							       data_len);
	case DW3000_VENDOR_CMD_NFCC_COEX_STOP:
		return dw3000_nfcc_coex_stop(dw);
	default:
		return -EINVAL;
	}
}
