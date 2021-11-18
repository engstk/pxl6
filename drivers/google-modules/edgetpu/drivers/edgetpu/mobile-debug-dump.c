// SPDX-License-Identifier: GPL-2.0
/*
 * Implements methods common to the family of EdgeTPUs for mobile devices to retrieve host side
 * debug dump segments and report them to SSCD.
 *
 * Copyright (C) 2021 Google, Inc.
 */

#include <linux/mutex.h>
#include <linux/platform_data/sscoredump.h>
#include <linux/rbtree.h>
#include <linux/slab.h>

#include "edgetpu-device-group.h"
#include "edgetpu-mailbox.h"
#include "mobile-debug-dump.h"

#include "edgetpu-debug-dump.c"

struct mobile_sscd_mappings_dump *
mobile_sscd_collect_mappings_segment(struct edgetpu_device_group **groups, size_t num_groups,
				     struct sscd_segment *sscd_seg)
{
	struct mobile_sscd_mappings_dump *mappings_dump;
	struct edgetpu_mapping_root *mappings;
	struct rb_node *node;
	void *resized_arr;
	size_t idx = 0, mappings_num = 0, new_size = 0;

	mappings_dump = kmalloc(sizeof(struct mobile_sscd_mappings_dump), GFP_KERNEL);
	for (idx = 0; idx < num_groups; idx++) {
		mutex_lock(&groups[idx]->lock);
		new_size += groups[idx]->host_mappings.count *
			    sizeof(struct mobile_sscd_mappings_dump);
		resized_arr = krealloc(mappings_dump, new_size, GFP_KERNEL);
		if (!resized_arr) {
			kfree(mappings_dump);
			mutex_unlock(&groups[idx]->lock);
			return NULL;
		}

		mappings = &groups[idx]->host_mappings;
		for (node = rb_first(&mappings->rb); node; node = rb_next(node)) {
			struct edgetpu_mapping *map =
				container_of(node, struct edgetpu_mapping, node);

			mappings_dump[mappings_num].host_address = map->host_address;
			mappings_dump[mappings_num].device_address = map->device_address;
			mappings_dump[mappings_num].alloc_iova = map->alloc_iova;
			mappings_dump[mappings_num].size = (u64)map->alloc_size;
			mappings_num++;
		}
		new_size += groups[idx]->dmabuf_mappings.count *
			    sizeof(struct mobile_sscd_mappings_dump);
		resized_arr = krealloc(mappings_dump, new_size, GFP_KERNEL);
		if (!resized_arr) {
			kfree(mappings_dump);
			mutex_unlock(&groups[idx]->lock);
			return NULL;
		}

		mappings = &groups[idx]->dmabuf_mappings;
		for (node = rb_first(&mappings->rb); node; node = rb_next(node)) {
			struct edgetpu_mapping *map =
				container_of(node, struct edgetpu_mapping, node);

			mappings_dump[mappings_num].host_address = map->host_address;
			mappings_dump[mappings_num].device_address = map->device_address;
			mappings_dump[mappings_num].alloc_iova = map->alloc_iova;
			mappings_dump[mappings_num].size = (u64)map->alloc_size;
			mappings_num++;
		}
		mutex_unlock(&groups[idx]->lock);
	}

	sscd_seg->addr = mappings_dump;
	sscd_seg->size = new_size;
	sscd_seg->vaddr = mappings_dump;

	return mappings_dump;
}

size_t mobile_sscd_collect_cmd_resp_queues(struct edgetpu_dev *etdev,
					   struct edgetpu_device_group **groups, size_t num_groups,
					   struct sscd_segment *sscd_seg_arr)
{
	struct edgetpu_kci *kci;
	size_t idx;
	u16 num_queues = 0;

	// Collect VII cmd and resp queues
	for (idx = 0; idx < num_groups; idx++) {
		mutex_lock(&groups[idx]->lock);
		if (!edgetpu_group_mailbox_detached_locked(groups[idx])) {
			sscd_seg_arr[num_queues].addr =
						(void *)groups[idx]->vii.cmd_queue_mem.vaddr;
			sscd_seg_arr[num_queues].size = groups[idx]->vii.cmd_queue_mem.size;
			sscd_seg_arr[num_queues].paddr =
						(void *)groups[idx]->vii.cmd_queue_mem.tpu_addr;
			sscd_seg_arr[num_queues].vaddr =
						(void *)groups[idx]->vii.cmd_queue_mem.vaddr;
			num_queues++;

			sscd_seg_arr[num_queues].addr =
						(void *)groups[idx]->vii.resp_queue_mem.vaddr;
			sscd_seg_arr[num_queues].size = groups[idx]->vii.resp_queue_mem.size;
			sscd_seg_arr[num_queues].paddr =
						(void *)groups[idx]->vii.resp_queue_mem.tpu_addr;
			sscd_seg_arr[num_queues].vaddr =
						(void *)groups[idx]->vii.resp_queue_mem.vaddr;
			num_queues++;
		}
		mutex_unlock(&groups[idx]->lock);
	}

	// Collect KCI cmd and resp queues
	kci = etdev->kci;
	sscd_seg_arr[num_queues].addr = (void *)kci->cmd_queue_mem.vaddr;
	sscd_seg_arr[num_queues].size = MAX_QUEUE_SIZE * sizeof(struct edgetpu_command_element);
	sscd_seg_arr[num_queues].paddr = (void *)kci->cmd_queue_mem.tpu_addr;
	sscd_seg_arr[num_queues].vaddr = (void *)kci->cmd_queue_mem.vaddr;
	num_queues++;

	sscd_seg_arr[num_queues].addr = (void *)kci->resp_queue_mem.vaddr;
	sscd_seg_arr[num_queues].size = MAX_QUEUE_SIZE *
					sizeof(struct edgetpu_kci_response_element);
	sscd_seg_arr[num_queues].paddr = (void *)kci->resp_queue_mem.tpu_addr;
	sscd_seg_arr[num_queues].vaddr = (void *)kci->resp_queue_mem.vaddr;
	num_queues++;

	return num_queues;
}
