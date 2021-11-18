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

#ifndef __DW3000_CCC_MAILBOX_H
#define __DW3000_CCC_MAILBOX_H

#include "dw3000.h"
#include "dw3000_ccc.h"

/**
 * dw3000_nfcc_coex_write_buffer() - Write buffer into scratch memory.
 * @dw: Driver context.
 * @buffer: buffer to write in scratch memory.
 * @len: byte length of the buffer.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_write_buffer(struct dw3000 *dw,
				  const struct dw3000_nfcc_coex_buffer *buffer,
				  u16 len);

/**
 * dw3000_nfcc_coex_read_buffer() - Read buffer from scratch memory.
 * @dw: Driver context.
 * @buffer: buffer fill with content of the scratch memory.
 * @len: byte length of the buffer.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_read_buffer(struct dw3000 *dw,
				 struct dw3000_nfcc_coex_buffer *buffer,
				 u16 len);

/**
 * dw3000_nfcc_coex_handle_spi_ready_isr() - Handle SPI ready interrupt.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_handle_spi_ready_isr(struct dw3000 *dw);

/**
 * dw3000_nfcc_coex_handle_spi1_avail_isr() - Handle SPI1 available interrupt.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_handle_spi1_avail_isr(struct dw3000 *dw);

/**
 * dw3000_nfcc_coex_init() - Initialize NFCC coexistence context.
 * @dw: Driver context.
 */
void dw3000_nfcc_coex_init(struct dw3000 *dw);

/**
 * dw3000_nfcc_coex_enable() - Enable NFCC coexistence.
 * @dw: Driver context.
 * @channel: Channel number (5 or 9).
 * @cb: Callback to use on spi available interrupt.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_enable(struct dw3000 *dw, u8 channel,
			    dw3000_nfcc_coex_spi_avail_cb cb);

/**
 * dw3000_nfcc_coex_disable() - Disable NFCC coexistence.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_disable(struct dw3000 *dw);

#endif /* __DW3000_CCC_MAILBOX_H */
