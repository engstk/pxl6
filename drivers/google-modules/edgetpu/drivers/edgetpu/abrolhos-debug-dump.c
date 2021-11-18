// SPDX-License-Identifier: GPL-2.0

#include <linux/platform_data/sscoredump.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "abrolhos-platform.h"

#include "edgetpu-debug-dump.c"

static int abrolhos_sscd_generate_coredump(void *p_etdev, void *p_dump_setup)
{
	struct edgetpu_dev *etdev;
	struct edgetpu_debug_dump_setup *dump_setup;
	struct abrolhos_platform_dev *pdev;
	struct sscd_platform_data *pdata;
	struct platform_device *sscd_dev;
	struct sscd_segment *segs;
	struct edgetpu_debug_dump *debug_dump;
	struct edgetpu_crash_reason *crash_reason;
	struct edgetpu_dump_segment *dump_seg;
	char crash_info[128];
	int dump_segments_num;
	int i, ret;
	u64 offset;

	if (!p_etdev || !p_dump_setup)
		return -EINVAL;

	etdev = (struct edgetpu_dev *)p_etdev;
	dump_setup = (struct edgetpu_debug_dump_setup *)p_dump_setup;
	pdev = container_of(etdev, struct abrolhos_platform_dev, edgetpu_dev);
	pdata = (struct sscd_platform_data *)pdev->sscd_info.pdata;
	sscd_dev = (struct platform_device *)pdev->sscd_info.dev;
	if (!pdata->sscd_report) {
		etdev_err(etdev, "failed to generate coredump");
		return -ENOENT;
	}

	offset = sizeof(struct edgetpu_debug_dump_setup);
	debug_dump = (struct edgetpu_debug_dump *)((u64 *)dump_setup +
		      word_align_offset(offset));

	/* Populate crash reason */
	crash_reason = (struct edgetpu_crash_reason *)((u64 *)dump_setup +
			word_align_offset(debug_dump->crash_reason_offset));
	scnprintf(crash_info, sizeof(crash_info),
		  "[edgetpu_coredump] error code: 0x%llx", crash_reason->code);

	/* Populate dump segments */
	dump_segments_num = debug_dump->dump_segments_num;
	segs = kmalloc_array(dump_segments_num,
			     sizeof(struct sscd_segment),
			     GFP_KERNEL);
	if (!segs)
		return -ENOMEM;

	dump_seg = (struct edgetpu_dump_segment *)((u64 *)dump_setup +
		    word_align_offset(debug_dump->dump_segments_offset));
	offset = debug_dump->dump_segments_offset +
		 sizeof(struct edgetpu_dump_segment);
	for (i = 0; i < dump_segments_num; i++) {
		segs[i].addr = &dump_seg[i].src_addr + 1;
		segs[i].size = dump_seg[i].size;
		segs[i].paddr = (void *)(etdev->debug_dump_mem.tpu_addr +
					 offset);
		segs[i].vaddr = (void *)(etdev->debug_dump_mem.vaddr +
					 offset);
		offset += sizeof(struct edgetpu_dump_segment) + dump_seg->size;
		dump_seg = (struct edgetpu_dump_segment *)
			   ((u64 *)dump_seg + word_align_offset(
			   sizeof(struct edgetpu_dump_segment) +
			   dump_seg->size));
	}

	/* Pass dump data to SSCD daemon */
	etdev_dbg(etdev, "report: %d segments", dump_segments_num);
	ret = pdata->sscd_report(sscd_dev, segs, dump_segments_num,
				 SSCD_FLAGS_ELFARM64HDR, crash_info);

	kfree(segs);

	return ret;
}

int edgetpu_debug_dump_init(struct edgetpu_dev *etdev)
{
	size_t size;
	int ret;
	struct edgetpu_debug_dump_setup *dump_setup;

	size = EDGETPU_DEBUG_DUMP_MEM_SIZE;

	/*
	 * Allocate a buffer for various dump segments
	 */
	ret = edgetpu_alloc_coherent(etdev, size, &etdev->debug_dump_mem,
				     EDGETPU_CONTEXT_KCI);
	if (ret) {
		etdev_err(etdev, "Debug dump seg alloc failed");
		etdev->debug_dump_mem.vaddr = NULL;
		return ret;
	}
	dump_setup =
		(struct edgetpu_debug_dump_setup *)etdev->debug_dump_mem.vaddr;
	dump_setup->dump_mem_size = size;
	memset(dump_setup, 0, dump_setup->dump_mem_size);

	/*
	 * Allocate memory for debug dump handlers
	 */
	etdev->debug_dump_handlers = kcalloc(DUMP_REQ_REASON_NUM,
				     sizeof(*etdev->debug_dump_handlers),
				     GFP_KERNEL);
	if (!etdev->debug_dump_handlers)
		return -ENOMEM;
	etdev->debug_dump_handlers[DUMP_REQ_REASON_BY_USER] =
		abrolhos_sscd_generate_coredump;

	return ret;
}

void edgetpu_debug_dump_exit(struct edgetpu_dev *etdev)
{
	if (!etdev->debug_dump_mem.vaddr) {
		etdev_dbg(etdev, "Debug dump not allocated");
		return;
	}
	/*
	 * Free the memory assigned for debug dump
	 */
	edgetpu_free_coherent(etdev, &etdev->debug_dump_mem,
			      EDGETPU_CONTEXT_KCI);
	kfree(etdev->debug_dump_handlers);
}
