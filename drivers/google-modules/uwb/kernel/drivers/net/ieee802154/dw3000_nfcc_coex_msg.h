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
#ifndef __DW3000_NFCC_COEX_MSG_H
#define __DW3000_NFCC_COEX_MSG_H

#include <linux/module.h>
#include "dw3000_nfcc_coex.h"

/* Forward declaration. */
struct dw3000;

/* TLVs types. */
#define TLV_SESSION_TIME0 1
#define TLV_SLOT_LIST 2
#define TLV_UWBCNT_OFFS 3
#define TLV_ERROR 4

#define TLV_MAX_NB_SLOTS 4

/**
 * struct dw3000_nfcc_coex_tlv_slot - TLV slot definition.
 */
struct dw3000_nfcc_coex_tlv_slot {
	/**
	 * @start_sys_time: Start date in sys time unit.
	 */
	u32 start_sys_time;
	/**
	 * @end_sys_time: End date in sys time unit.
	 */
	u32 end_sys_time;
};

/**
 * struct dw3000_nfcc_coex_tlv_slot_list - TLV slots.
 */
struct dw3000_nfcc_coex_tlv_slot_list {
	/**
	 * @nb_slots: Number of slots used.
	 */
	u8 nb_slots;
	/**
	 * @slots: array of start/end session time.
	 */
	struct dw3000_nfcc_coex_tlv_slot slots[TLV_MAX_NB_SLOTS];
} __attribute__((packed));

void dw3000_nfcc_coex_header_put(struct dw3000 *dw,
				 struct dw3000_nfcc_coex_buffer *buffer);
void dw3000_nfcc_coex_single_tlv_slot_put(
	struct dw3000_nfcc_coex_buffer *buffer, u32 start, u32 end);
void dw3000_nfcc_coex_tlv_u32_put(struct dw3000_nfcc_coex_buffer *buffer,
				  u8 type, u32 value);
void dw3000_nfcc_coex_tlv_slots_put(
	struct dw3000_nfcc_coex_buffer *buffer,
	const struct dw3000_nfcc_coex_tlv_slot_list *slot_list);
int dw3000_nfcc_coex_message_send(struct dw3000 *dw);
int dw3000_nfcc_coex_message_check(
	struct dw3000 *dw, const struct dw3000_nfcc_coex_buffer *buffer,
	struct dw3000_nfcc_coex_rx_msg_info *rx_msg_info);

#endif /* __DW3000_NFCC_COEX_MSG_H */
