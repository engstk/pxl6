/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2015-2019, Samsung Electronics.
 *
 */

#ifndef __EXYNOS_MODEM_CTRL_H__
#define __EXYNOS_MODEM_CTRL_H__

#include <linux/skbuff.h>

#define MODEM_CTRL_UART_AP 0
#define MODEM_CTRL_UART_CP 1

struct vnet {
	void *iod;
	bool enable_zerocopy;
	bool (*free_head)(struct sk_buff *skb);
};

#if IS_ENABLED(CONFIG_EXYNOS_MODEM_IF)
extern int modem_force_crash_exit_ext(void);
#else
static inline int modem_force_crash_exit_ext(void) { return 0; }
#endif

#if IS_ENABLED(CONFIG_CP_UART_NOTI)
extern void send_uart_noti_to_modem(int val);
#else
static inline void send_uart_noti_to_modem(int val) { return; }
#endif

#endif /* __EXYNOS_MODEM_CTRL_H__ */
