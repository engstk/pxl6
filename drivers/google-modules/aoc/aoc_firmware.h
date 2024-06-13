// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Whitechapel AoC Firmware loading support
 *
 * Copyright (c) 2019 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/firmware.h>

#define AOC_FIRMWARE_OFFSET_INVALID 0xffffffff

#define AOC_AUTH_HEADER_SIZE 4096

struct sysmmu_entry {
    uint64_t value;
};

/* Access macros to decode SYSMMU entry */
#define SYSMMU_VADDR(v) ((((v) >> 0) & 0x000FFFFF) << 12)
#define SYSMMU_PADDR(v) ((((v) >> 20) & 0x00FFFFFF) << 12)
#define SYSMMU_SIZE(v) ((((v) >> 44) & 0x000FFFFF) << 12)

/* Dev builds bypass the UUID check on load */
bool _aoc_fw_is_release(const struct firmware *fw);

bool _aoc_fw_is_signed(const struct firmware *fw);

bool _aoc_fw_is_compatible(const struct firmware *fw);

bool _aoc_fw_is_valid(const struct firmware *fw);

u32 _aoc_fw_bootloader_offset(const struct firmware *fw);

u32 _aoc_fw_ipc_offset(const struct firmware *fw);

/* Returns firmware version, or NULL on error */
const char *_aoc_fw_version(const struct firmware *fw);

bool _aoc_fw_commit(const struct firmware *fw, void *dest);

uint16_t _aoc_fw_sysmmu_offset(const struct firmware *fw);

uint16_t _aoc_fw_sysmmu_size(const struct firmware *fw);

bool _aoc_fw_is_valid_sysmmu_size(const struct firmware *fw);

struct sysmmu_entry *_aoc_fw_sysmmu_entry(const struct firmware *fw);

struct aoc_image_config *_aoc_fw_image_config(const struct firmware *fw);

u32 _aoc_fw_get_header_version(const struct firmware *fw);
