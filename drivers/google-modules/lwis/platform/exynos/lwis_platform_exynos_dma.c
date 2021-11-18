/*
 * Google LWIS Exynos Platform-specific DMA Functions
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/ion_exynos.h>
#include <linux/exynos_iovmm.h>

#include "lwis_commands.h"
#include "lwis_init.h"
#include "lwis_platform.h"
#include "lwis_platform_dma.h"
#include "lwis_platform_exynos.h"

#define ION_SYSTEM_HEAP_NAME "ion_system_heap"
#define ION_SYSTEM_CONTIG_HEAP_NAME "ion_system_contig_heap"

// ION allocation flags, imported from "/drivers/staging/android/uapi/ion.h".
#define ION_FLAG_CACHED 1
#define ION_FLAG_NOZEROED 8

struct dma_buf *lwis_platform_dma_buffer_alloc(size_t len, unsigned int flags)
{
	unsigned int ion_flags = 0;

	// "system_contig_heap" does not seemed to be used in the exynos driver,
	// that's why system heap is used by default.
	const char *ion_heap_name = ION_SYSTEM_HEAP_NAME;

	if (flags & LWIS_DMA_BUFFER_CACHED) {
		ion_flags |= ION_FLAG_CACHED;
	}
	if (flags & LWIS_DMA_BUFFER_UNINITIALIZED) {
		ion_flags |= ION_FLAG_NOZEROED;
	}

	return ion_alloc_dmabuf(ion_heap_name, len, ion_flags);
}

dma_addr_t lwis_platform_dma_buffer_map(struct lwis_device *lwis_dev,
					struct dma_buf_attachment *attachment, off_t offset,
					size_t size, enum dma_data_direction direction, int flags)
{
	return ion_iovmm_map(attachment, offset, size, direction, flags);
}

int lwis_platform_dma_buffer_unmap(struct lwis_device *lwis_dev,
				   struct dma_buf_attachment *attachment, dma_addr_t address)
{
	ion_iovmm_unmap(attachment, address);
	return 0;
}
