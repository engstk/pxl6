/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Module that defines structure to retrieve debug dump segments
 * from abrolhos firmware.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __ABROLHOS_DEBUG_DUMP_H__
#define __ABROLHOS_DEBUG_DUMP_H__

struct abrolhos_sscd_info {
	void *pdata; /* SSCD platform data */
	void *dev; /* SSCD platform device */
};

#endif /* ABROLHOS_DEBUG_DUMP_H_ */
