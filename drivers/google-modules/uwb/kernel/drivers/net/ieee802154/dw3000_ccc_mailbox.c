/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020 Qorvo US, Inc.
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
 * Qorvo.
 * Please contact Qorvo to inquire about licensing terms.
 */

#include "dw3000.h"
#include "dw3000_core.h"
#include "dw3000_core_reg.h"
#include "dw3000_ccc.h"
#include "dw3000_trc.h"

/**
 * dw3000_nfcc_coex_read_scratch_ram() - Copy from scratch ram memory to buffer.
 * @dw: Driver context.
 * @buffer: Buffer to write.
 * @len: Number of byte to copy.
 * @offset: Number of byte to skip.
 *
 * Return: 0 on success, else an error.
 */
static int
dw3000_nfcc_coex_read_scratch_ram(struct dw3000 *dw,
				  struct dw3000_nfcc_coex_buffer *buffer,
				  u16 len, u16 offset)
{
	int end_offset = len + offset;

	if (end_offset > DW3000_SCRATCH_RAM_LEN) {
		trace_dw3000_nfcc_coex_err(dw, "scratch ram bad address");
		return -EINVAL;
	}
	return dw3000_xfer(dw, DW3000_SCRATCH_RAM_ID, offset, len, buffer->raw,
			   DW3000_SPI_RD_BIT);
}

/**
 * dw3000_nfcc_coex_read_scratch_ram() - Copy from buffer to scratch ram memory.
 * @dw: Driver context.
 * @buffer: Buffer to read.
 * @len: Number of byte to copy.
 * @offset: Number of byte to skip.
 *
 * Return: 0 on success, else an error.
 */
static int
dw3000_nfcc_coex_write_scratch_ram(struct dw3000 *dw,
				   const struct dw3000_nfcc_coex_buffer *buffer,
				   u16 len, u16 offset)
{
	int end_offset = len + offset;
	/* TODO: Avoid the cast, which remove the const with better code.
	 * Idea to dig: define dw3000_xfer with 2 pointers(read/write).
	 * And develop the idea of full duplex API. */
	void *raw = (void *)buffer->raw;

	if (end_offset > DW3000_SCRATCH_RAM_LEN) {
		trace_dw3000_nfcc_coex_err(dw, "scratch ram bad address");
		return -EINVAL;
	}
	return dw3000_xfer(dw, DW3000_SCRATCH_RAM_ID, offset, len, raw,
			   DW3000_SPI_WR_BIT);
}

/**
 * dw3000_nfcc_coex_is_spi1_reserved() - Get the status of SPI1 reserved status.
 * @dw: Driver context.
 * @val: Boolean updated on success.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_is_spi1_reserved(struct dw3000 *dw, bool *val)
{
	u8 reg;
	int rc;

	/* Check if SPI1 is reserved by reading SPI_SEM register */
	rc = dw3000_reg_read8(dw, DW3000_SPI_SEM_ID, 0, &reg);
	if (rc)
		return rc;

	*val = reg & DW3000_SPI_SEM_SPI1_RG_BIT_MASK;
	return 0;
}

/**
 * dw3000_nfcc_coex_release_spi1() - Release the SPI1.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_release_spi1(struct dw3000 *dw)
{
	int rc;
	bool is_spi1_enabled;

	rc = dw3000_write_fastcmd(dw, DW3000_CMD_SEMA_REL);
	if (rc)
		return rc;
	rc = dw3000_nfcc_coex_is_spi1_reserved(dw, &is_spi1_enabled);
	if (rc)
		return rc;

	return is_spi1_enabled ? -EBUSY : 0;
}

/**
 * dw3000_nfcc_coex_reserve_spi1() - Reserve the SPI1.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_reserve_spi1(struct dw3000 *dw)
{
	int rc;
	bool is_spi1_reserved;

	rc = dw3000_write_fastcmd(dw, DW3000_CMD_SEMA_REQ);
	if (rc)
		return rc;
	/* Check if the SPI1 is really reserved.
	 * Indeed, if SPI2 is already reserved, SPI1 could not be reserved. */
	rc = dw3000_nfcc_coex_is_spi1_reserved(dw, &is_spi1_reserved);
	if (rc)
		return rc;

	return is_spi1_reserved ? 0 : -EBUSY;
}

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

	*val = reg_sem & ((DW3000_SPI_SEM_SPI1MAVAIL_BIT_MASK |
			   DW3000_SPI_SEM_SPI1MAVAIL_BIT_MASK) >>
			  8) &&
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

	/* Clear SPI1MAVAIL interrupt. */
	rc = dw3000_clear_dss_status(dw, DW3000_DSS_STAT_SPI1_AVAIL_BIT_MASK);
	if (rc)
		return rc;
	/* Clear SPI2MAVAIL interrupt. */
	rc = dw3000_clear_dss_status(dw, DW3000_DSS_STAT_SPI2_AVAIL_BIT_MASK);
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
	reg = reg | (DW3000_SPI_SEM_SPI1MAVAIL_BIT_MASK >> 8) |
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

int dw3000_nfcc_coex_write_buffer(struct dw3000 *dw,
				  const struct dw3000_nfcc_coex_buffer *buffer,
				  u16 len)
{
	int rc;

	if (!dw->nfcc_coex.enabled)
		return -EOPNOTSUPP;
	if (len > DW3000_CCC_SCRATCH_AP_SIZE) {
		trace_dw3000_nfcc_coex_err(
			dw, "writing to nfcc exceed SCRATCH_AP_SIZE");
		return -EINVAL;
	}
	rc = dw3000_nfcc_coex_reserve_spi1(dw);
	if (rc)
		return rc;
	rc = dw3000_nfcc_coex_write_scratch_ram(dw, buffer, len,
						DW3000_CCC_SCRATCH_AP_OFFSET);
	if (rc)
		return rc;
	dw->nfcc_coex.seqnum++;
	/* Trigger IRQ2 to inform NFCC. */
	return dw3000_nfcc_coex_release_spi1(dw);
}

int dw3000_nfcc_coex_read_buffer(struct dw3000 *dw,
				 struct dw3000_nfcc_coex_buffer *buffer,
				 u16 len)
{
	int rc;

	if (!dw->nfcc_coex.enabled)
		return -EOPNOTSUPP;
	if (len > DW3000_CCC_SCRATCH_NFCC_SIZE) {
		trace_dw3000_nfcc_coex_err(
			dw, "Reading from NFCC exceed SCRATCH_AP_SIZE");
		return -EINVAL;
	}
	rc = dw3000_nfcc_coex_read_scratch_ram(dw, buffer, len,
					       DW3000_CCC_SCRATCH_NFCC_OFFSET);
	if (rc)
		trace_dw3000_nfcc_coex_err(
			dw, "Error while reading NFCC scratch RAM");
	return rc;
}

int dw3000_nfcc_coex_handle_spi_ready_isr(struct dw3000 *dw)
{
	int r;

	if (__dw3000_chip_version == 0 || !dw->nfcc_coex.enabled)
		return -EOPNOTSUPP;

	/* NFCC was enabled before sleeping. Enable the NFCC mailbox
	 * interrupt and send the right message to the NFCC. */
	r = dw3000_nfcc_coex_enable_SPIxMAVAIL_interrupts(dw);
	if (r)
		return r;
	return dw3000_nfcc_coex_write_msg_on_wakeup(dw);
}

int dw3000_nfcc_coex_handle_spi1_avail_isr(struct dw3000 *dw)
{
	int rc;
	struct dw3000_nfcc_coex_buffer buffer;

	if (!dw->nfcc_coex.enabled)
		return -EOPNOTSUPP;

	rc = dw3000_nfcc_coex_read_buffer(dw, &buffer,
					  DW3000_CCC_SCRATCH_NFCC_SIZE);
	/* Clear SPI1MAVAIL interrupt on error to avoid isr in loop. */
	rc &= dw3000_clear_dss_status(dw, DW3000_DSS_STAT_SPI1_AVAIL_BIT_MASK);
	if (rc)
		/* FIXME: rc doesn't matches with errno defines. */
		return rc;

	if (!dw->nfcc_coex.spi_avail_cb) {
		trace_dw3000_nfcc_coex_err(
			dw, "No callback defined to handle received buffer");
		return -EOPNOTSUPP;
	}

	return dw->nfcc_coex.spi_avail_cb(dw, &buffer);
}

void dw3000_nfcc_coex_init(struct dw3000 *dw)
{
	struct dw3000_nfcc_coex *nfcc_coex = &dw->nfcc_coex;

	memset(nfcc_coex, 0, sizeof(*nfcc_coex));
}

int dw3000_nfcc_coex_enable(struct dw3000 *dw, u8 channel,
			    dw3000_nfcc_coex_spi_avail_cb cb)
{
	int rc;

	trace_dw3000_nfcc_coex_enable(dw, channel);

	/* NFCC needs a D0 chip or above. C0 does not have 2 SPI interfaces. */
	if (__dw3000_chip_version == 0) {
		trace_dw3000_nfcc_coex_err(dw, "C0 chip is not supported");
		return -EOPNOTSUPP;
	}

	/* Set the channel for NFCC and save current config. */
	/* FIXME: original_channel lost on second call. */
	dw->nfcc_coex.original_channel = dw->config.chan;
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

int dw3000_nfcc_coex_disable(struct dw3000 *dw)
{
	int rc;
	bool val = false;

	trace_dw3000_nfcc_coex_disable(dw);

	if (__dw3000_chip_version == 0)
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
