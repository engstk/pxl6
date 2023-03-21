// SPDX-License-Identifier: GPL-2.0-only
/* memory.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2022 Google LLC
 */

#include <linux/mm.h>

void vh_zap_pte_range_tlb_start(void *data, void *preempt_off)
{
	preempt_disable();
	*(int *)preempt_off = 1;
}

void vh_zap_pte_range_tlb_force_flush(void *data, struct page *page, bool *flush)
{
	if (is_migrate_cma_page(page))
		*flush = true;
}

void vh_zap_pte_range_tlb_end(void *data, void *preempt_off)
{
	if (likely(*(int *)preempt_off == 1)) {
		preempt_enable();
		*(int *)preempt_off = 0;
	}
}

void vh_skip_lru_disable(void *data, bool *skip)
{
	*skip = true;
}
