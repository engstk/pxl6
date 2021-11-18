/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Implements utilities for shared firmware management of EdgeTPU.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __EDGETPU_SHARED_FW_H__
#define __EDGETPU_SHARED_FW_H__

#include <linux/device.h>

#include "edgetpu-internal.h"

struct edgetpu_shared_fw_buffer;

/*
 * name for this firmware in null terminated string, the same as which loaded by
 * linux request_firmware API
 */
const char *
edgetpu_shared_fw_buffer_name(const struct edgetpu_shared_fw_buffer *buffer);
/* host address for this firmware */
void *
edgetpu_shared_fw_buffer_vaddr(const struct edgetpu_shared_fw_buffer *buffer);
/* size in bytes for this firmware */
size_t
edgetpu_shared_fw_buffer_size(const struct edgetpu_shared_fw_buffer *buffer);

struct edgetpu_shared_fw_init_data {
	/* firmware size alignment in bytes */
	size_t size_align;
};

/* Initializes structures for shared firmware management. */
int
edgetpu_shared_fw_init(const struct edgetpu_shared_fw_init_data *init_data);
/* Finalizes structures for shared firmware management. */
void edgetpu_shared_fw_exit(void);

/*
 * Load reference counted shared firmware from file system. Increase reference
 * count by 1 if the firmware is already loaded before.
 *
 * Firmware loaded by this function should be released by
 * edgetpu_shared_fw_put().
 *
 * @name: firmware path to be loaded
 * @etdev: requesting edgetpu_dev, if any, for logging
 */
struct edgetpu_shared_fw_buffer *edgetpu_shared_fw_load(
	const char *name, struct edgetpu_dev *etdev);

/*
 * Increase the reference count of the buffer by 1.
 *
 * returns the buffer, behave the same as other *_get/put() functions
 */
struct edgetpu_shared_fw_buffer *
edgetpu_shared_fw_get(struct edgetpu_shared_fw_buffer *buffer);
/*
 * Find the shared firmware by name and increase the reference count of the
 * found buffer by 1.
 *
 * returns NULL on error or not found
 */
struct edgetpu_shared_fw_buffer *
edgetpu_shared_fw_get_by_name(const char *name);

/*
 * Decrease the reference count by 1 and free the shared buffer if its
 * reference count reaches 0.
 */
void edgetpu_shared_fw_put(struct edgetpu_shared_fw_buffer *buffer);

/*
 * (Add/Remove) driver-wide sysfs attributes for development and debug.
 */
int edgetpu_shared_fw_add_driver_attrs(struct device_driver *driver);
void edgetpu_shared_fw_remove_driver_attrs(struct device_driver *driver);

#endif /* __EDGETPU_SHARED_FW_H__ */
