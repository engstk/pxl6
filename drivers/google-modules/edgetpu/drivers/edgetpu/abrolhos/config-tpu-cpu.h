/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Chip-dependent configuration for TPU CPU.
 *
 * Copyright (C) 2019 Google, Inc.
 */

#ifndef __ABROLHOS_CONFIG_TPU_CPU_H__
#define __ABROLHOS_CONFIG_TPU_CPU_H__

#define EDGETPU_REG_RESET_CONTROL			0x90010
#define CPUPORESET					(1 << 1)

#define EDGETPU_REG_INSTRUCTION_REMAP_CONTROL		0x90050
#define EDGETPU_REG_INSTRUCTION_REMAP_BASE		0x90058
#define EDGETPU_REG_INSTRUCTION_REMAP_LIMIT		0x90060
#define EDGETPU_REG_INSTRUCTION_REMAP_NEW_BASE		0x90068
#define EDGETPU_REG_INSTRUCTION_REMAP_SECURITY		0x90070
#define EDGETPU_REG_SECURITY				0x90048

/* Power Control signals for P-channel interface. */
#define EDGETPU_REG_POWER_CONTROL			0xA0008
#define PSTATE_SHIFT					0
#define PSTATE						(1 << PSTATE_SHIFT)
#define PREQ						(1 << 1)
#define PDENY						(1 << 2)
#define PACCEPT						(1 << 3)
#define PACTIVE						(1 << 5)

#endif /* __ABROLHOS_CONFIG_TPU_CPU_H__ */
