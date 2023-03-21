/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Include all configuration files for Abrolhos.
 *
 * Copyright (C) 2019 Google, Inc.
 */

#ifndef __ABROLHOS_CONFIG_H__
#define __ABROLHOS_CONFIG_H__

#define DRIVER_NAME "abrolhos"

#define EDGETPU_DEV_MAX		1

#define EDGETPU_HAS_MULTI_GROUPS
/* 1 context per VII/group plus 1 for KCI */
#define EDGETPU_NCONTEXTS 8
/* Max number of virtual context IDs that can be allocated for one device. */
#define EDGETPU_NUM_VCIDS 16
/* Reserved VCID that uses the extra partition. */
#define EDGETPU_VCID_EXTRA_PARTITION 0

/* Is a "mobile" style device. */
#define EDGETPU_FEATURE_MOBILE
#define EDGETPU_HAS_WAKELOCK

/*
 * A remapped data region is available. This will be accessible by the TPU
 * CPU regardless of active context and is typically used for logging buffer and
 * non-secure mailbox queues.
 */
#define EDGETPU_HAS_REMAPPED_DATA

/*
 * The TPU VA where the firmware is located.
 *
 * The address is chosen to not overlap with any memory region specified in the
 * firmware's linker file.
 */
#define FW_IOVA 0x16000000u
/*
 * Size of the area in remapped DRAM reserved for firmware code and internal
 * data. This must match the firmware's linker file.
 */
#define EDGETPU_FW_SIZE_MAX			0x100000

/* Data in remapped DRAM starts after firmware code and internal data */
#define EDGETPU_REMAPPED_DATA_OFFSET		EDGETPU_FW_SIZE_MAX

/*
 * Size of remapped DRAM data region. This must match the firmware's linker
 * file
 */
#define EDGETPU_REMAPPED_DATA_SIZE		0x100000

/*
 * Instruction remap registers make carveout memory appear at address
 * 0x10000000 from the TPU CPU perspective
 */
#define EDGETPU_INSTRUCTION_REMAP_BASE		0x10000000

/* Address from which the TPU CPU can access data in the remapped region */
#define EDGETPU_REMAPPED_DATA_ADDR                                        \
	(EDGETPU_INSTRUCTION_REMAP_BASE + EDGETPU_REMAPPED_DATA_OFFSET)

/*
 * Size of memory for FW accessible debug dump segments
 */
#define EDGETPU_DEBUG_DUMP_MEM_SIZE 0x4E0000

#include "config-mailbox.h"
#include "config-pwr-state.h"
#include "config-tpu-cpu.h"
#include "csrs.h"

#endif /* __ABROLHOS_CONFIG_H__ */
