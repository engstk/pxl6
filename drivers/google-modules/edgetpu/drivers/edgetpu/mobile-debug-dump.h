/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Module that defines structure to retrieve debug dump segments
 * specific to the family of EdgeTPUs for mobile devices.
 *
 * Copyright (C) 2021 Google, Inc.
 */

#ifndef __MOBILE_DEBUG_DUMP_H__
#define __MOBILE_DEBUG_DUMP_H__

#include "edgetpu-debug-dump.h"

struct mobile_sscd_info {
	void *pdata; /* SSCD platform data */
	void *dev; /* SSCD platform device */
};

struct mobile_sscd_mappings_dump {
	u64 host_address;
	u64 device_address;
	u64 alloc_iova;
	u64 size;
};

struct sscd_segment;

/*
 * Collects the mapping information of all the host mapping and dmabuf mapping buffers of all
 * @groups as an array of struct mobile_sscd_mappings_dump and populates the @sscd_seg.
 *
 * Returns the pointer to the first element of the mappings dump array. The allocated array should
 * be freed by the caller after the sscd segment is reported.
 * Returns NULL in case of failure.
 */
struct mobile_sscd_mappings_dump *
mobile_sscd_collect_mappings_segment(struct edgetpu_device_group **groups, size_t num_groups,
				     struct sscd_segment *sscd_seg);

/*
 * Collects the VII cmd and resp queues of all @groups that @etdev belongs to and the KCI cmd and
 * resp queues and populates them as @sscd_seg_arr elements.
 *
 * Returns the total number of queues collected since some queues may have been released for groups
 * with detached mailboxes. The return value is less than or equal to the total number of queues
 * expected based on @num_groups i.e (2 * @num_groups +2).
 */
size_t mobile_sscd_collect_cmd_resp_queues(struct edgetpu_dev *etdev,
					   struct edgetpu_device_group **groups, size_t num_groups,
					   struct sscd_segment *sscd_seg_arr);

#endif /* MOBILE_DEBUG_DUMP_H_ */
