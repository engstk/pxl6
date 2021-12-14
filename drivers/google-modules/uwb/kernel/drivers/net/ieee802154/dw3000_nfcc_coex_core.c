/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2021 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo. Please contact Qorvo to inquire about licensing terms.
 */
#include "dw3000_nfcc_coex_core.h"
#include "dw3000_nfcc_coex_buffer.h"
#include "dw3000_nfcc_coex_msg.h"
#include "dw3000_nfcc_coex.h"
#include "dw3000_core.h"
#include "dw3000_trc.h"
#include "dw3000_mcps.h"

#include <linux/module.h>

unsigned dw3000_nfcc_coex_margin_dtu = DW3000_ANTICIP_DTU;
module_param_named(nfcc_coex_margin_dtu, dw3000_nfcc_coex_margin_dtu, uint,
		   0444);
MODULE_PARM_DESC(
	nfcc_coex_margin_dtu,
	"Margin in dtu needed to give access to the NFCC controller and for the NFCC controller to wake up (default is anticip_dtu)");

/**
 * dw3000_nfcc_coex_is_SPIxMAVAIL_interrupts_enabled() - Is all SPI interrupts enabled?
 * @dw: Driver context.
 * @val: Boolean updated on success.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_is_SPIxMAVAIL_interrupts_enabled(struct dw3000 *dw,
							     bool *val)
{
	int rc;
	u8 reg_sem;
	u32 reg_sys;

	rc = dw3000_reg_read8(dw, DW3000_SPI_SEM_ID, 1, &reg_sem);
	if (rc)
		return rc;
	rc = dw3000_reg_read32(dw, DW3000_SYS_CFG_ID, 0, &reg_sys);
	if (rc)
		return rc;

	*val = reg_sem & (DW3000_SPI_SEM_SPI1MAVAIL_BIT_MASK >> 8) &&
	       reg_sem & (DW3000_SPI_SEM_SPI2MAVAIL_BIT_MASK >> 8) &&
	       reg_sys & DW3000_SYS_CFG_DS_IE2_BIT_MASK;
	return 0;
}

/**
 * dw3000_nfcc_coex_enable_SPIxMAVAIL_interrupts() - Enable all SPI available interrupts.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_enable_SPIxMAVAIL_interrupts(struct dw3000 *dw)
{
	int rc;
	u8 reg;

	/* Clear SPI1MAVAIL and SPI2MAVAIL interrupt status. */
	rc = dw3000_clear_dss_status(
		dw, DW3000_DSS_STAT_SPI1_AVAIL_BIT_MASK |
			    DW3000_DSS_STAT_SPI2_AVAIL_BIT_MASK);
	if (rc)
		return rc;
	/* Disable SPIRDY in SYS_MASK. If it is enabled, the IRQ2 will not work.
	 * It is an undocumented feature.
	 */
	rc = dw3000_reg_modify32(
		dw, DW3000_SYS_ENABLE_LO_ID, 0,
		(u32)~DW3000_SYS_ENABLE_LO_SPIRDY_ENABLE_BIT_MASK, 0);
	if (rc)
		return rc;
	/* Enable the dual SPI interrupt for SPI */
	rc = dw3000_reg_modify32(dw, DW3000_SYS_CFG_ID, 0, U32_MAX,
				 (u32)DW3000_SYS_CFG_DS_IE2_BIT_MASK);
	if (rc)
		return rc;
	/* The masked write transactions do not work on the SPI_SEM register.
	 * So, a read, modify, write sequence is mandatory on this register.
	 *
	 * The 16 bits SPI_SEM register can be accessed as two 8 bits registers.
	 * So, only read the upper 8 bits for performance.
	 */
	rc = dw3000_reg_read8(dw, DW3000_SPI_SEM_ID, 1, &reg);
	if (rc)
		return rc;
	/* Set SPI1MAVAIL and SPI2MAVAIL masks */
	reg |= (DW3000_SPI_SEM_SPI1MAVAIL_BIT_MASK >> 8) |
	      (DW3000_SPI_SEM_SPI2MAVAIL_BIT_MASK >> 8);
	return dw3000_reg_write8(dw, DW3000_SPI_SEM_ID, 1, reg);
}

/**
 * dw3000_nfcc_coex_disable_SPIxMAVAIL_interrupts() - Disable all SPI available interrupts.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_disable_SPIxMAVAIL_interrupts(struct dw3000 *dw)
{
	int rc;
	u8 reg;

	/* Please read the comment in enable_SPIxMAVAIL_interrupts() for SPI_SEM access. */
	rc = dw3000_reg_read8(dw, DW3000_SPI_SEM_ID, 1, &reg);
	if (rc)
		return rc;
	/* Reset SPI1MAVAIL and SPI2MAVAIL masks. */
	reg = reg & ~(DW3000_SPI_SEM_SPI1MAVAIL_BIT_MASK >> 8) &
	      ~(DW3000_SPI_SEM_SPI2MAVAIL_BIT_MASK >> 8);
	rc = dw3000_reg_write8(dw, DW3000_SPI_SEM_ID, 1, reg);
	if (rc)
		return rc;
	/* Disable the dual SPI interrupt for SPI. */
	return dw3000_reg_modify32(dw, DW3000_SYS_CFG_ID, 0,
				   (u32)~DW3000_SYS_CFG_DS_IE2_BIT_MASK, 0);
}

/**
 * dw3000_nfcc_coex_do_watchdog_timeout() - Do watchdog timeout event in workqueue.
 * @dw: Driver context.
 * @in: Data to read.
 * @out: Data to write.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_do_watchdog_timeout(struct dw3000 *dw,
						const void *in, void *out)
{
	/* Update status for GET_ACCESS_INFORMATION vendor. */
	dw->nfcc_coex.access_info.stop = true;
	/* FIXME: The expected is a hard reset of decawave when the semaphore
	 * haven't been released.
	 * But CCC firmware can not currently provide a message with nb_tlv=0. */
	dw3000_nfcc_coex_disable(dw);
	mcps802154_tx_done(dw->llhw);

	return 0;
}

/**
 * dw3000_nfcc_coex_do_spi1_avail() - Do SPI1 available irq event in workqueue.
 * @dw: Driver context.
 * @in: Data to read.
 * @out: Data to write.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_do_spi1_avail(struct dw3000 *dw, const void *in,
					  void *out)
{
	int rc;
	struct dw3000_nfcc_coex_buffer buffer;

	if (!dw->nfcc_coex.spi_avail_cb) {
		trace_dw3000_nfcc_coex_err(
			dw, "No callback defined to handle received buffer");
		return -EOPNOTSUPP;
	}

	rc = dw3000_nfcc_coex_read_buffer(dw, &buffer,
					  DW3000_NFCC_COEX_MSG_IN_SIZE);
	if (rc)
		return rc;

	return dw->nfcc_coex.spi_avail_cb(dw, &buffer);
}

/**
 * dw3000_nfcc_coex_watchdog_handler() - Watchdog timeout event.
 * @timer: Timer context.
 */
static void dw3000_nfcc_coex_watchdog_handler(struct timer_list *timer)
{
	struct dw3000_nfcc_coex *nfcc_coex =
		from_timer(nfcc_coex, timer, watchdog_timer);
	struct dw3000 *dw = nfcc_coex_to_dw(nfcc_coex);
	struct dw3000_stm_command cmd = { dw3000_nfcc_coex_do_watchdog_timeout,
					  NULL, NULL };

	trace_dw3000_nfcc_coex_watchdog(dw);
	dw3000_enqueue_timer(dw, &cmd);
}

/**
 * dw3000_nfcc_coex_handle_spi1_avail_isr() - Handle SPI1 available interrupt.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_handle_spi1_avail_isr(struct dw3000 *dw)
{
	struct dw3000_stm_command cmd = { dw3000_nfcc_coex_do_spi1_avail, NULL,
					  NULL };
	return dw3000_enqueue_generic(dw, &cmd);
}

/**
 * dw3000_nfcc_coex_handle_spi1_ready_isr() - Handle SPI ready interrupt.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_handle_spi1_ready_isr(struct dw3000 *dw)
{
	int r;

	if (__dw3000_chip_version == DW3000_C0_VERSION)
		return -EOPNOTSUPP;

	/* NFCC was enabled before sleeping. Enable the NFCC mailbox
	 * interrupt and send the right message to the NFCC. */
	r = dw3000_nfcc_coex_enable_SPIxMAVAIL_interrupts(dw);
	if (r)
		return r;
	return dw3000_nfcc_coex_message_send(dw);
}

/**
 * dw3000_nfcc_coex_init() - Initialize NFCC coexistence context.
 * @dw: Driver context.
 */
void dw3000_nfcc_coex_init(struct dw3000 *dw)
{
	struct dw3000_nfcc_coex *nfcc_coex = &dw->nfcc_coex;

	memset(nfcc_coex, 0, sizeof(*nfcc_coex));
	timer_setup(&nfcc_coex->watchdog_timer,
		    dw3000_nfcc_coex_watchdog_handler, 0);
}

/**
 * dw3000_nfcc_coex_enable() - Enable NFCC coexistence.
 * @dw: Driver context.
 * @channel: Channel number (5 or 9).
 * @cb: Callback to use on spi available interrupt.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_enable(struct dw3000 *dw, u8 channel,
			    dw3000_nfcc_coex_spi_avail_cb cb)
{
	int rc;

	trace_dw3000_nfcc_coex_enable(dw, channel);

	/* NFCC needs a D0 chip or above. C0 does not have 2 SPI interfaces. */
	if (__dw3000_chip_version == DW3000_C0_VERSION) {
		trace_dw3000_nfcc_coex_err(dw, "C0 chip is not supported");
		return -EOPNOTSUPP;
	}

	/* Need to wake-up device and wait it before continuing. */
	dw3000_wakeup_and_wait(dw);

	/* Set the channel for NFCC and save current config. */
	/* FIXME: original_channel lost on second call. */
	dw->nfcc_coex.original_channel = dw->config.chan;
	dw->nfcc_coex.sync_time_needed = true;
	dw->config.chan = channel;
	rc = dw3000_configure_chan(dw);
	if (rc) {
		trace_dw3000_nfcc_coex_err(dw, "configure channel fails");
		return rc;
	}

	/* Disable rx during NFCC. */
	rc = dw3000_rx_disable(dw);
	if (rc)
		trace_dw3000_nfcc_coex_warn(dw, "rx disable failed");

	rc = dw3000_nfcc_coex_enable_SPIxMAVAIL_interrupts(dw);
	if (rc)
		trace_dw3000_nfcc_coex_err(
			dw, "SPIxMAVAIL interrupts enable failed");

	dw->nfcc_coex.enabled = true;
	dw->nfcc_coex.spi_avail_cb = cb;
	return rc;
}

/**
 * dw3000_nfcc_coex_disable() - Disable NFCC coexistence.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_disable(struct dw3000 *dw)
{
	int rc;
	bool val = false;

	trace_dw3000_nfcc_coex_disable(dw);

	if (__dw3000_chip_version == DW3000_C0_VERSION)
		return -EOPNOTSUPP;

	rc = dw3000_nfcc_coex_is_SPIxMAVAIL_interrupts_enabled(dw, &val);
	if (rc) {
		trace_dw3000_nfcc_coex_err(
			dw, "SPIxMAVAIL interrupts read enable status failed");
		return rc;
	}
	if (val) {
		rc = dw3000_nfcc_coex_disable_SPIxMAVAIL_interrupts(dw);
		if (rc) {
			trace_dw3000_nfcc_coex_err(
				dw, "SPIxMAVAIL interrupts disable failed");
			return rc;
		}
	}

	if (dw->nfcc_coex.original_channel == 5 ||
	    dw->nfcc_coex.original_channel == 9) {
		dw->config.chan = dw->nfcc_coex.original_channel;
		rc = dw3000_configure_chan(dw);
		if (rc) {
			trace_dw3000_nfcc_coex_err(
				dw,
				"ccc_disable error while restoring channel");
			return rc;
		}
	}

	/* TODO: Inform HAL that NFCC session is complete. */
	dw->nfcc_coex.enabled = false;
	dw->nfcc_coex.spi_avail_cb = NULL;
	return rc;
}

/**
 * dw3000_nfcc_coex_sleep() - Entering in deep sleep.
 * @dw: Driver context.
 * @sleep_dtu: Sleep duration in dtu.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_sleep(struct dw3000 *dw, u32 sleep_dtu)
{
	trace_dw3000_nfcc_coex_sleep(dw, sleep_dtu);
	dw->deep_sleep_state.next_operational_state = DW3000_OP_STATE_IDLE_PLL;
	return dw3000_deep_sleep_and_wakeup(dw, DTU_TO_US(sleep_dtu));
}
