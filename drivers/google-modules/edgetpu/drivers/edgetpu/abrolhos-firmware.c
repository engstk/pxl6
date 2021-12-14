// SPDX-License-Identifier: GPL-2.0
/*
 * Abrolhos Edge TPU ML accelerator firmware download support.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/dma-mapping.h>
#include <linux/gsa/gsa_tpu.h>
#include <linux/slab.h>

#include "abrolhos-platform.h"
#include "edgetpu-config.h"
#include "edgetpu-firmware.h"
#include "edgetpu-internal.h"
#include "edgetpu-kci.h"
#include "edgetpu-mailbox.h"
#include "mobile-firmware.h"

static int abrolhos_firmware_alloc_buffer(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_buffer *fw_buf)
{
	struct edgetpu_dev *etdev = et_fw->etdev;
	struct abrolhos_platform_dev *abpdev = to_abrolhos_dev(etdev);

	/* Allocate extra space the image header */
	size_t buffer_size =
		abpdev->fw_region_size + MOBILE_FW_HEADER_SIZE;

	fw_buf->vaddr = vmalloc(buffer_size);
	if (!fw_buf->vaddr) {
		etdev_err(etdev, "%s: failed to allocate buffer (%zu bytes)\n",
			  __func__, buffer_size);
		return -ENOMEM;
	}
	fw_buf->dma_addr = 0;
	fw_buf->alloc_size = buffer_size;
	fw_buf->used_size_align = 16;
	return 0;
}

static void abrolhos_firmware_free_buffer(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_buffer *fw_buf)
{
	vfree(fw_buf->vaddr);
	fw_buf->vaddr = NULL;
	fw_buf->dma_addr = 0;
	fw_buf->alloc_size = 0;
	fw_buf->used_size_align = 0;
}

static int abrolhos_firmware_setup_buffer(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_buffer *fw_buf)
{
	return 0;
}

static void abrolhos_firmware_teardown_buffer(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_buffer *fw_buf)
{
}

static int abrolhos_firmware_restart(struct edgetpu_firmware *et_fw,
				     bool force_reset)
{
	struct edgetpu_dev *etdev = et_fw->etdev;
	struct abrolhos_platform_dev *edgetpu_pdev = to_abrolhos_dev(etdev);
	int tpu_state;

	/* We are in a bad state, send shutdown command and hope the device recovers */
	if (force_reset)
		gsa_send_tpu_cmd(edgetpu_pdev->gsa_dev, GSA_TPU_SHUTDOWN);

	tpu_state = gsa_send_tpu_cmd(edgetpu_pdev->gsa_dev, GSA_TPU_START);

	if (tpu_state < 0) {
		etdev_err(etdev, "GSA restart firmware failed: %d\n",
			  tpu_state);
		return -EIO;
	}

	etdev_dbg(etdev, "Firmware restart successful\n");

	return 0;
}

static int abrolhos_firmware_prepare_run(struct edgetpu_firmware *et_fw,
					 struct edgetpu_firmware_buffer *fw_buf)
{
	struct edgetpu_dev *etdev = et_fw->etdev;
	struct abrolhos_platform_dev *abpdev = to_abrolhos_dev(etdev);
	void *image_vaddr, *header_vaddr;
	struct mobile_image_config *image_config;
	phys_addr_t image_start, image_end, carveout_start, carveout_end;
	dma_addr_t header_dma_addr;
	int ret, tpu_state;

	if (fw_buf->used_size < MOBILE_FW_HEADER_SIZE) {
		etdev_err(etdev, "Invalid buffer size: %zu < %d\n",
			  fw_buf->used_size, MOBILE_FW_HEADER_SIZE);
		return -EINVAL;
	}

	tpu_state = gsa_send_tpu_cmd(abpdev->gsa_dev, GSA_TPU_GET_STATE);

	if (tpu_state < GSA_TPU_STATE_INACTIVE) {
		etdev_err(etdev, "GSA failed to retrieve current status: %d\n", tpu_state);
		return tpu_state;
	}

	etdev_dbg(etdev, "GSA Reports TPU state: %d\n", tpu_state);

	if (tpu_state > GSA_TPU_STATE_INACTIVE) {
		ret = gsa_unload_tpu_fw_image(abpdev->gsa_dev);
		if (ret) {
			etdev_warn(etdev, "GSA release failed: %d\n", ret);
			return -EIO;
		}
	}

	image_vaddr = memremap(abpdev->fw_region_paddr, abpdev->fw_region_size,
			       MEMREMAP_WC);

	if (!image_vaddr) {
		etdev_err(etdev, "memremap failed\n");
		return -ENOMEM;
	}

	/* Skip the header */
	memcpy(image_vaddr, fw_buf->vaddr + MOBILE_FW_HEADER_SIZE,
	       fw_buf->used_size - MOBILE_FW_HEADER_SIZE);

	/* Allocate coherent memory for the image header */
	header_vaddr = dma_alloc_coherent(abpdev->gsa_dev,
					  MOBILE_FW_HEADER_SIZE,
					  &header_dma_addr, GFP_KERNEL);
	if (!header_vaddr) {
		etdev_err(etdev,
			  "Failed to allocate coherent memory for header\n");
		ret = -ENOMEM;
		goto out_unmap;
	}

	memcpy(header_vaddr, fw_buf->vaddr, MOBILE_FW_HEADER_SIZE);
	etdev_dbg(etdev,
		  "Requesting GSA image load. meta = %llX payload = %llX",
		  header_dma_addr, (u64)abpdev->fw_region_paddr);

	ret = gsa_load_tpu_fw_image(abpdev->gsa_dev, header_dma_addr,
				    abpdev->fw_region_paddr);
	if (ret) {
		etdev_err(etdev, "GSA authentication failed: %d\n", ret);
		ret = -EIO;
		goto out_free_gsa;
	}

	/* fetch the firmware versions */
	image_config = fw_buf->vaddr + MOBILE_IMAGE_CONFIG_OFFSET;
	memcpy(&etdev->fw_version, &image_config->firmware_versions,
	       sizeof(etdev->fw_version));

	/*
	 * GSA verifies the image config addresses and sizes are valid,
	 * so we don't perform overflow checks here.
	 */
	image_start = (phys_addr_t)image_config->carveout_base;
	image_end = (phys_addr_t)(image_config->firmware_base +
				  image_config->firmware_size - 1);
	carveout_start = abpdev->fw_region_paddr;
	carveout_end = carveout_start + abpdev->fw_region_size - 1;

	/* Image must fit within the carveout */
	if (image_start < carveout_start || image_end > carveout_end) {
		etdev_err(etdev, "Firmware image doesn't fit in carveout\n");
		etdev_err(etdev, "Image config: %pap - %pap\n", &image_start,
			  &image_end);
		etdev_err(etdev, "Carveout: %pap - %pap\n", &carveout_start,
			  &carveout_end);
		ret = -ERANGE;
		goto out_free_gsa;
	}

	/* Reset KCI mailbox before starting f/w, don't process anything old.*/
	edgetpu_mailbox_reset(etdev->kci->mailbox);

	tpu_state = gsa_send_tpu_cmd(abpdev->gsa_dev, GSA_TPU_START);

	if (tpu_state < 0) {
		etdev_err(etdev, "GSA start firmware failed: %d\n", tpu_state);
		ret = -EIO;
	}

out_free_gsa:
	dma_free_coherent(abpdev->gsa_dev, MOBILE_FW_HEADER_SIZE,
			  header_vaddr, header_dma_addr);
out_unmap:
	memunmap(image_vaddr);
	return ret;
}

static const struct edgetpu_firmware_chip_data abrolhos_firmware_chip_data = {
	.default_firmware_name = EDGETPU_DEFAULT_FIRMWARE_NAME,
	.alloc_buffer = abrolhos_firmware_alloc_buffer,
	.free_buffer = abrolhos_firmware_free_buffer,
	.setup_buffer = abrolhos_firmware_setup_buffer,
	.teardown_buffer = abrolhos_firmware_teardown_buffer,
	.prepare_run = abrolhos_firmware_prepare_run,
	.restart = abrolhos_firmware_restart,
};

int mobile_edgetpu_firmware_create(struct edgetpu_dev *etdev)
{
	return edgetpu_firmware_create(etdev, &abrolhos_firmware_chip_data);
}

void mobile_edgetpu_firmware_destroy(struct edgetpu_dev *etdev)
{
	edgetpu_firmware_destroy(etdev);
}

unsigned long edgetpu_chip_firmware_iova(struct edgetpu_dev *etdev)
{
	/*
	 * There is no IOVA in Abrolhos, since firmware the IOMMU is
	 * bypassed and the only translation in effect is the one
	 * done by instruction remap registers
	 */
	return EDGETPU_INSTRUCTION_REMAP_BASE;
}
