/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Chip-dependent configuration for mailbox.
 *
 * Copyright (C) 2019 Google, Inc.
 */

#ifndef __ABROLHOS_CONFIG_MAILBOX_H__
#define __ABROLHOS_CONFIG_MAILBOX_H__

#include <linux/types.h> /* u32 */

#define EDGETPU_NUM_MAILBOXES 8
#define EDGETPU_NUM_VII_MAILBOXES (EDGETPU_NUM_MAILBOXES - 1)
#define EDGETPU_NUM_P2P_MAILBOXES 0
#define EDGETPU_NUM_EXT_MAILBOXES 0

/*
 * The TZ mailbox is not managed by the kernel, but we still need to tell firmware to enable it,
 * so its index is placed after the kernel managed mailboxes.
 */
#define EDGETPU_TZ_MAILBOX_ID	8

#define ABROLHOS_CSR_MBOX2_CONTEXT_ENABLE 0xe0000
#define ABROLHOS_CSR_MBOX2_CMD_QUEUE_DOORBELL_SET 0xe1000
#define ABROLHOS_CSR_MBOX2_RESP_QUEUE_DOORBELL_SET 0xe1800
#define EDGETPU_MBOX_CSRS_SIZE 0x2000	/* CSR size of each mailbox */

#define EDGETPU_MBOX_BASE ABROLHOS_CSR_MBOX2_CONTEXT_ENABLE

/* CSR storing mailbox response queue doorbell status */
#define HOST_NONSECURE_INT_SRC_STATUS_REG 0x000f0000
#define HOST_NONSECURE_INT_SRC_CLEAR_REG 0x000f0008

static inline u32 edgetpu_mailbox_get_context_csr_base(u32 index)
{
	return ABROLHOS_CSR_MBOX2_CONTEXT_ENABLE +
		index * EDGETPU_MBOX_CSRS_SIZE;
}

static inline u32 edgetpu_mailbox_get_cmd_queue_csr_base(u32 index)
{
	return ABROLHOS_CSR_MBOX2_CMD_QUEUE_DOORBELL_SET +
		index * EDGETPU_MBOX_CSRS_SIZE;
}

static inline u32 edgetpu_mailbox_get_resp_queue_csr_base(u32 index)
{
	return ABROLHOS_CSR_MBOX2_RESP_QUEUE_DOORBELL_SET +
		index * EDGETPU_MBOX_CSRS_SIZE;
}

#endif /* __ABROLHOS_CONFIG_MAILBOX_H__ */
