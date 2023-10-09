// SPDX-License-Identifier: GPL-2.0
/*
 * Abrolhos Edge TPU ML accelerator device host support.
 *
 * Copyright (C) 2019 Google, Inc.
 */

#include <linux/irqreturn.h>

#include "abrolhos-platform.h"
#include "edgetpu-config.h"
#include "edgetpu-debug-dump.h"
#include "edgetpu-internal.h"
#include "edgetpu-mailbox.h"
#include "edgetpu-telemetry.h"
#include "edgetpu-wakelock.h"
#include "mobile-pm.h"

#define HOST_NONSECURE_INTRSRCMASKREG	0x000f0004

/*
 * The interrupt handler for mailboxes.
 *
 * This handler reads the IntSrcStatusReg available on Abrolhos to get the
 * response queue doorbell status of mailboxes 0-7 in one read. It loops
 * through the bits to check pending interrupts and invokes their IRQ
 * handlers. This allows for more efficient handling of interrupts than
 * edgetpu_mailbox_handle_irq handler.
 */
static irqreturn_t
abrolhos_mailbox_handle_irq(struct edgetpu_mailbox_manager *mgr)
{
	struct edgetpu_mailbox *mailbox;
	uint i;
	u32 val, clear_val;
	u8 mailbox_rsp_queue_status_bits;

	if (!mgr)
		return IRQ_NONE;

	read_lock(&mgr->mailboxes_lock);
	val = edgetpu_dev_read_32(mgr->etdev,
				  HOST_NONSECURE_INT_SRC_STATUS_REG);
	clear_val = val & (0xff << 1);
	edgetpu_dev_write_32(mgr->etdev,
			     HOST_NONSECURE_INT_SRC_CLEAR_REG, clear_val);

	mailbox_rsp_queue_status_bits = val >> 1;
	for (i = 0; i < mgr->num_mailbox; i++) {
		mailbox = mgr->mailboxes[i];
		if (mailbox && (mailbox_rsp_queue_status_bits & 0x1) &&
		    mailbox->handle_irq)
			mailbox->handle_irq(mailbox);
		mailbox_rsp_queue_status_bits >>= 1;
		if (!mailbox_rsp_queue_status_bits)
			break;
	}
	read_unlock(&mgr->mailboxes_lock);

	return IRQ_HANDLED;
}

irqreturn_t edgetpu_chip_irq_handler(int irq, void *arg)
{
	struct edgetpu_dev *etdev = arg;

	edgetpu_telemetry_irq_handler(etdev);
	edgetpu_debug_dump_resp_handler(etdev);

	return abrolhos_mailbox_handle_irq(etdev->mailbox_manager);
}

u64 edgetpu_chip_tpu_timestamp(struct edgetpu_dev *etdev)
{
	return edgetpu_dev_read_64(etdev, EDGETPU_REG_CPUNS_TIMESTAMP);
}

void edgetpu_chip_init(struct edgetpu_dev *etdev)
{
	/* Disable the CustomBlock Interrupt. */
	edgetpu_dev_write_32(etdev, HOST_NONSECURE_INTRSRCMASKREG, 0x1);
}

void edgetpu_chip_exit(struct edgetpu_dev *etdev)
{
}

void edgetpu_mark_probe_fail(struct edgetpu_dev *etdev)
{
}

void edgetpu_chip_handle_reverse_kci(struct edgetpu_dev *etdev,
				     struct edgetpu_kci_response_element *resp)
{
	switch (resp->code) {
	case RKCI_CODE_PM_QOS_BTS:
		/* FW indicates to ignore the request by setting them to undefined values. */
		if (resp->retval != (typeof(resp->retval))~0ull)
			edgetpu_mobile_pm_set_pm_qos(etdev, resp->retval);
		if (resp->status != (typeof(resp->status))~0ull)
			edgetpu_mobile_pm_set_bts(etdev, resp->status);
		break;
	default:
		etdev_warn(etdev, "%s: Unrecognized KCI request: %u\n",
			   __func__, resp->code);
		break;
	}
}
