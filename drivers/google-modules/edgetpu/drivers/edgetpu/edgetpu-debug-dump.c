// SPDX-License-Identifier: GPL-2.0
/*
 * Module that defines structures and functions to retrieve debug dump segments
 * from edgetpu firmware.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#include <linux/workqueue.h>

#include "edgetpu-config.h"
#include "edgetpu-debug-dump.h"
#include "edgetpu-iremap-pool.h"
#include "edgetpu-kci.h"

static inline u64 word_align_offset(u64 offset)
{
	return offset/sizeof(u64) +
	       (((offset % sizeof(u64)) == 0) ? 0 : 1);
}

int edgetpu_get_debug_dump(struct edgetpu_dev *etdev, u64 type)
{
	int ret;
	struct edgetpu_debug_dump_setup *dump_setup;

	if (!etdev->debug_dump_mem.vaddr) {
		etdev_err(etdev, "Debug dump not allocated");
		return -EINVAL;
	}

	dump_setup =
		(struct edgetpu_debug_dump_setup *)etdev->debug_dump_mem.vaddr;
	dump_setup->type = type;
	/* Signal the type of dump and buffer address to firmware */
	ret = edgetpu_kci_get_debug_dump(etdev->kci,
					 etdev->debug_dump_mem.tpu_addr,
					 etdev->debug_dump_mem.size);
	etdev_dbg(etdev, "Sent debug dump request, tpu addr: %llx",
		  (u64)etdev->debug_dump_mem.tpu_addr);
	if (ret)
		etdev_err(etdev, "KCI dump info req failed: %d", ret);

	return ret;
}

static void edgetpu_debug_dump_work(struct work_struct *work)
{
	struct edgetpu_dev *etdev;
	struct edgetpu_debug_dump_setup *dump_setup;
	struct edgetpu_debug_dump *debug_dump;
	int ret;
	u64 offset, dump_reason;

	etdev = container_of(work, struct edgetpu_dev, debug_dump_work);
	dump_setup =
		(struct edgetpu_debug_dump_setup *)etdev->debug_dump_mem.vaddr;
	offset = sizeof(struct edgetpu_debug_dump_setup);
	debug_dump = (struct edgetpu_debug_dump *)((u64 *)dump_setup +
		     word_align_offset(offset));

	if (!etdev->debug_dump_handlers) {
		etdev_err(etdev,
			  "Failed to generate coredump as handler is NULL");
		goto debug_dump_work_done;
	}

	dump_reason = dump_setup->dump_req_reason;
	if (dump_reason >= DUMP_REQ_REASON_NUM ||
	    !etdev->debug_dump_handlers[dump_reason]) {
		etdev_err(etdev,
			  "Failed to generate coredump as handler is NULL for dump request reason: 0x%llx",
			  dump_reason);
		goto debug_dump_work_done;
	}

	ret = etdev->debug_dump_handlers[dump_reason]
	      ((void *)etdev, (void *)dump_setup);
	if (ret) {
		etdev_err(etdev, "Failed to generate coredump: %d\n", ret);
		goto debug_dump_work_done;
	}

debug_dump_work_done:
	debug_dump->host_dump_available_to_read = false;
}

void edgetpu_debug_dump_resp_handler(struct edgetpu_dev *etdev)
{
	struct edgetpu_debug_dump_setup *dump_setup;
	struct edgetpu_debug_dump *debug_dump;
	u64 offset;

	if (!etdev->debug_dump_mem.vaddr) {
		etdev_err(etdev, "Debug dump memory not allocated");
		return;
	}
	dump_setup =
		(struct edgetpu_debug_dump_setup *)etdev->debug_dump_mem.vaddr;
	offset = sizeof(struct edgetpu_debug_dump_setup);
	debug_dump = (struct edgetpu_debug_dump *)((u64 *)dump_setup +
		     word_align_offset(offset));
	if (!debug_dump->host_dump_available_to_read)
		return;

	if (!etdev->debug_dump_work.func)
		INIT_WORK(&etdev->debug_dump_work, edgetpu_debug_dump_work);

	schedule_work(&etdev->debug_dump_work);
}
