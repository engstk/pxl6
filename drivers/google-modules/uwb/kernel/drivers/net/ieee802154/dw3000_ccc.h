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
#ifndef __DW3000_CCC_H
#define __DW3000_CCC_H

#include <net/vendor_cmd.h>

/* Main defines */
#define DW3000_CCC_VER_ID 1
#define DW3000_CCC_SIGNATURE_STR "QORVO"
#define DW3000_CCC_SIGNATURE_LEN 5
#define DW3000_CCC_MAX_NB_TLV 12

/* Scratch memory */
#define DW3000_CCC_SCRATCH_AP_OFFSET (0U)
#define DW3000_CCC_SCRATCH_AP_SIZE (64U)
#define DW3000_CCC_SCRATCH_NFCC_OFFSET (64U)
#define DW3000_CCC_SCRATCH_NFCC_SIZE (63U)

/**
 * struct dw3000_nfcc_coex_msg - Message read/write from/to scratch memory.
 */
struct dw3000_nfcc_coex_msg {
	/**
	 * @signature: String signature, example "QORVO".
	 */
	u8 signature[DW3000_CCC_SIGNATURE_LEN];
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
		u8 raw[DW3000_CCC_SCRATCH_AP_SIZE];
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
 * struct dw3000_nfcc_coex_data - Result of message parsed.
 */
struct dw3000_nfcc_coex_data {
	/**
	 * @nextslot: current round data.
	 */
	u32 nextslot;
	/**
	 * @diff_ms: Delta between two date.
	 */
	u32 diff_ms;
	/**
	 * @slots: Pointer to array of slot.
	 */
	struct dw3000_nfcc_coex_tlv_slots *slots;
};

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
	 * @conflit_slot_idx: slot index with conflict on date.
	 */
	u32 conflit_slot_idx;
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
	 * @seqnum: sequence message counter increased on outgoing message.
	 */
	int seqnum;
	/**
	 * @testmode_round_count: Index number of NFCC session.
	*/
	u32 testmode_round_count;
	/**
	 * @original_channel: channel number to be restored after NFCC session.
	 */
	u8 original_channel;
	/**
	 * @enabled: True when nfcc coex is enabled.
	 */
	bool enabled;
};

/**
 * dw3000_nfcc_coex_testmode_session() - Start one custom NFCC session.
 * @dw: Driver context.
 * @chan: Channel number.
 * @session_time0: Offset in ms to add to current time.
 * @start: Start date of NFCC session in ms.
 * @end: End date of NFCC session in ms.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_testmode_session(struct dw3000 *dw, u8 chan,
				      u32 session_time0, u32 start, u32 end);

/**
 * dw3000_nfcc_coex_testmode_config() - Process testmode config.
 * @dw: Driver context.
 * @testmode_config: Pointer on persistente config to use.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_testmode_config(
	struct dw3000 *dw,
	const struct dw3000_nfcc_coex_testmode_config *testmode_config);

/**
 * dw3000_nfcc_coex_write_msg_on_wakeup() - Send clock offset message to NFCC.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_write_msg_on_wakeup(struct dw3000 *dw);

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
				void *data, size_t data_len);

#endif /* __DW3000_CCC_H */
