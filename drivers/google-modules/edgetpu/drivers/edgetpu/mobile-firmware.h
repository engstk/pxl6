/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Implements utilities for firmware management of mobile chipsets.
 *
 * Copyright (C) 2021 Google, Inc.
 */
#ifndef __MOBILE_FIRMWARE_H__
#define __MOBILE_FIRMWARE_H__

#include <linux/sizes.h>

#include "edgetpu-internal.h"
#include "edgetpu.h"

/* mobile FW header size */
#define MOBILE_FW_HEADER_SIZE SZ_4K
/* The offset to the signed firmware header. */
#define MOBILE_HEADER_OFFSET 0x400
/* The offset to image configuration. */
#define MOBILE_IMAGE_CONFIG_OFFSET (MOBILE_HEADER_OFFSET + 0x160)

/*
 * The image configuration attached to the signed firmware.
 */
struct mobile_image_config {
	__u32 carveout_base;
	__u32 firmware_base;
	__u32 firmware_size;
	struct edgetpu_fw_version firmware_versions;
} __packed;

/*
 * Mobile firmware header.
 */
struct mobile_image_header {
	char sig[512];
	char pub[512];
	int Magic;
	int Generation;
	int RollbackInfo;
	int Length;
	char Flags[16];
	char BodyHash[32];
	char ChipId[32];
	char AuthConfig[256];
	struct mobile_image_config ImageConfig;
};

int mobile_edgetpu_firmware_create(struct edgetpu_dev *etdev);
void mobile_edgetpu_firmware_destroy(struct edgetpu_dev *etdev);

#endif /* __MOBILE_FIRMWARE_H__ */
