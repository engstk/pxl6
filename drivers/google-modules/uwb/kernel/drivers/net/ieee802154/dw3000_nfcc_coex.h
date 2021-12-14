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
#ifndef __DW3000_NFCC_COEX_H
#define __DW3000_NFCC_COEX_H

#include <linux/module.h>
#include <net/vendor_cmd.h>

#include "dw3000_nfcc_coex_testmode.h"

/* Main defines */
#define DW3000_NFCC_COEX_VER_ID 1
#define DW3000_NFCC_COEX_SIGNATURE_STR "QORVO"
#define DW3000_NFCC_COEX_SIGNATURE_LEN 5
#define DW3000_NFCC_COEX_MAX_NB_TLV 12

/* For TLV messages written by AP / read by NFCC,
 * the scratch memory region is SCRATCH_RAM[0-63]. */
#define DW3000_NFCC_COEX_MSG_OUT_OFFSET 0
#define DW3000_NFCC_COEX_MSG_OUT_SIZE 64
/* For TLV messages read by AP / written by NFCC,
 * the scratch memory region is SCRATCH_RAM[64-127]. */
#define DW3000_NFCC_COEX_MSG_IN_OFFSET 64
#define DW3000_NFCC_COEX_MSG_IN_SIZE 63

#define DW3000_NFCC_COEX_MSG_MAX_SIZE DW3000_NFCC_COEX_MSG_OUT_SIZE

/* MSG_HEADER_LEN is also the sizeof of dw3000_nfcc_coex_msg structure. */
#define MSG_HEADER_LEN (DW3000_NFCC_COEX_SIGNATURE_LEN + 3)
#define MSG_LEN(x) (MSG_HEADER_LEN + (x).tlvs_len)

/**
 * struct dw3000_nfcc_coex_msg - Message read/write from/to scratch memory.
 */
struct dw3000_nfcc_coex_msg {
	/**
	 * @signature: String signature, example "QORVO".
	 */
	u8 signature[DW3000_NFCC_COEX_SIGNATURE_LEN];
	/**
	 * @ver_id: Version identifier.
	 */
	u8 ver_id;
	/**
	 * @seqnum: Sequence number.
	 */
	u8 seqnum;
	/**
	 * @nb_tlv: Number of TLV object in the body message.
	 */
	u8 nb_tlv;
	/**
	 * @tlvs: Body message. Its addr points to TLVs start.
	 */
	u8 tlvs[];
} __attribute__((packed));

/**
 * struct dw3000_nfcc_coex_buffer - Memory buffer to read/write a NFCC message.
 */
struct dw3000_nfcc_coex_buffer {
	/* Unamed union for structured access or raw access. */
	union {
		/**
		 * @raw: Byte memory.
		 */
		u8 raw[DW3000_NFCC_COEX_MSG_MAX_SIZE];
		/**
		 * @msg: CCC Message.
		 */
		struct dw3000_nfcc_coex_msg msg;
	};
	/**
	 * @tlvs_len: Len of TLVs in bytes.
	 */
	u8 tlvs_len;
} __attribute__((packed));

/**
 * struct dw3000_nfcc_coex_rx_msg_info - Result of message parsed.
 */
struct dw3000_nfcc_coex_rx_msg_info {
	/**
	 * @next_timestamp_sys_time: Next NFCC access requested.
	 */
	u32 next_timestamp_sys_time;
	/**
	 * @next_duration_sys_time: Next NFCC duration access.
	 */
	int next_duration_sys_time;
	/**
	 * @next_slot_found: True when first next slot is found.
	 */
	bool next_slot_found;
	/**
	 * @slot_list: Pointer to array of slot.
	 */
	const struct dw3000_nfcc_coex_tlv_slot_list *slot_list;
};

/**
 * typedef dw3000_nfcc_coex_spi_avail_cb - SPI available isr handler.
 * @dw: Driver context.
 * @buffer: Memory read from scratch memory.
 *
 * Return: 0 on success, else an error.
 */
typedef int (*dw3000_nfcc_coex_spi_avail_cb)(
	struct dw3000 *dw, const struct dw3000_nfcc_coex_buffer *buffer);

/**
 * struct dw3000_nfcc_coex - NFCC coexistence context.
 */
struct dw3000_nfcc_coex {
	/**
	 * @testmode_config: config with testmode enabled.
	 */
	const struct dw3000_nfcc_coex_testmode_config *testmode_config;
	/**
	 * @access_info: Access information to provide to upper layer.
	 */
	struct dw3000_vendor_cmd_nfcc_coex_get_access_info access_info;
	/**
	 * @spi_avail_cb: callback used on SPI available isr.
	 */
	dw3000_nfcc_coex_spi_avail_cb spi_avail_cb;
	/**
	 * @testmode_round_count: Index number of NFCC session.
	*/
	u32 testmode_round_count;
	/**
	 * @session_start_dtu: start date of nfcc session in DTU.
	 */
	u32 session_start_dtu;
	/**
	 * @prev_offset_sys_time: Previous offset between local and decawave time.
	 */
	u32 prev_offset_sys_time;
	/**
	 * @original_channel: channel number to be restored after NFCC session.
	 */
	u8 original_channel;
	/**
	 * @rx_seq_num: Sequence number of last valid message check.
	 */
	u8 rx_seq_num;
	/**
	 * @tx_seq_num: Sequence message counter increased on outgoing message.
	 */
	u8 tx_seq_num;
	/**
	 * @enabled: True when nfcc coex is enabled.
	 */
	bool enabled;
	/**
	 * @sync_time_needed: True when clock_sync frame must be send.
	 */
	bool sync_time_needed;
	/**
	 * @first_rx_message: False after the first valid msg received.
	 */
	bool first_rx_message;
	/**
	 * @watchdog_timer: Watchdog timer to detect spi not bring back.
	 */
	struct timer_list watchdog_timer;
};

#endif /* __DW3000_NFCC_COEX_H */
