/* Copyright (c) 2008-2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * SPI driver for Qualcomm MSM platforms
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/pm_runtime.h>
#include <linux/spi/qcom-spi.h>
#include <linux/msm-sps.h>
#include <linux/msm-bus.h>
#include <linux/msm-bus-board.h>
#include "spi-qsd-qcom-lowlatency.h"

#define SPI_MAX_BYTES_PER_WORD (4)

static int msm_spi_pm_resume_runtime(struct device *device);
static int msm_spi_pm_suspend_runtime(struct device *device);
static int get_local_resources(struct msm_spi *dd);
static void put_local_resources(struct msm_spi *dd);

static inline int msm_spi_configure_gsbi(struct msm_spi *dd,
					 struct platform_device *pdev)
{
	struct resource *resource;
	unsigned long gsbi_mem_phys_addr;
	size_t gsbi_mem_size;
	void __iomem *gsbi_base;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!resource)
		return 0;

	gsbi_mem_phys_addr = resource->start;
	gsbi_mem_size = resource_size(resource);
	if (!devm_request_mem_region(&pdev->dev, gsbi_mem_phys_addr,
				     gsbi_mem_size, SPI_DRV_NAME))
		return -ENXIO;

	gsbi_base = devm_ioremap(&pdev->dev, gsbi_mem_phys_addr, gsbi_mem_size);
	if (!gsbi_base)
		return -ENXIO;

	/* Set GSBI to SPI mode */
	writel_relaxed(GSBI_SPI_CONFIG, gsbi_base + GSBI_CTRL_REG);

	return 0;
}

static inline void msm_spi_register_init(struct msm_spi *dd)
{
	writel_relaxed(0x00000001, dd->base + SPI_SW_RESET);
	msm_spi_set_state(dd, SPI_OP_STATE_RESET);
	writel_relaxed(0x00000000, dd->base + SPI_OPERATIONAL);
	writel_relaxed(0x00000000, dd->base + SPI_CONFIG);
	writel_relaxed(0x00000000, dd->base + SPI_IO_MODES);
	if (dd->qup_ver)
		writel_relaxed(0x00000000, dd->base + QUP_OPERATIONAL_MASK);
}

static int msm_spi_pinctrl_init(struct msm_spi *dd)
{
	dd->pinctrl = devm_pinctrl_get(dd->dev);
	if (IS_ERR_OR_NULL(dd->pinctrl)) {
		dev_err(dd->dev, "Failed to get pin ctrl\n");
		return PTR_ERR(dd->pinctrl);
	}
	dd->pins_active =
		pinctrl_lookup_state(dd->pinctrl, SPI_PINCTRL_STATE_DEFAULT);
	if (IS_ERR_OR_NULL(dd->pins_active)) {
		dev_err(dd->dev, "Failed to lookup pinctrl default state\n");
		return PTR_ERR(dd->pins_active);
	}

	dd->pins_sleep =
		pinctrl_lookup_state(dd->pinctrl, SPI_PINCTRL_STATE_SLEEP);
	if (IS_ERR_OR_NULL(dd->pins_sleep)) {
		dev_err(dd->dev, "Failed to lookup pinctrl sleep state\n");
		return PTR_ERR(dd->pins_sleep);
	}

	return 0;
}

static inline int msm_spi_request_gpios(struct msm_spi *dd)
{
	int i = 0;
	int result = 0;

	if (!dd->pdata->use_pinctrl) {
		for (i = 0; i < ARRAY_SIZE(spi_rsrcs); ++i) {
			if (dd->spi_gpios[i] >= 0) {
				result = gpio_request(dd->spi_gpios[i],
						      spi_rsrcs[i]);
				if (result) {
					dev_err(dd->dev,
						"%s: gpio_request for pin %d "
						"failed with error %d\n",
						__func__, dd->spi_gpios[i],
						result);
					goto error;
				}
			}
		}
	} else {
		result = pinctrl_select_state(dd->pinctrl, dd->pins_active);
		if (result) {
			dev_err(dd->dev, "%s: Can not set %s pins\n", __func__,
				SPI_PINCTRL_STATE_DEFAULT);
			goto error;
		}
	}
	return 0;
error:
	if (!dd->pdata->use_pinctrl) {
		for (; --i >= 0;) {
			if (dd->spi_gpios[i] >= 0)
				gpio_free(dd->spi_gpios[i]);
		}
	}
	return result;
}

static inline void msm_spi_free_gpios(struct msm_spi *dd)
{
	int i;
	int result = 0;

	if (!dd->pdata->use_pinctrl) {
		for (i = 0; i < ARRAY_SIZE(spi_rsrcs); ++i) {
			if (dd->spi_gpios[i] >= 0)
				gpio_free(dd->spi_gpios[i]);
		}

		for (i = 0; i < ARRAY_SIZE(spi_cs_rsrcs); ++i) {
			if (dd->cs_gpios[i].valid) {
				gpio_free(dd->cs_gpios[i].gpio_num);
				dd->cs_gpios[i].valid = 0;
			}
		}
	} else {
		result = pinctrl_select_state(dd->pinctrl, dd->pins_sleep);
		if (result)
			dev_err(dd->dev, "%s: Can not set %s pins\n", __func__,
				SPI_PINCTRL_STATE_SLEEP);
	}
}

/**
 * msm_spi_clk_max_rate: finds the nearest lower rate for a clk
 * @clk the clock for which to find nearest lower rate
 * @rate clock frequency in Hz
 * @return nearest lower rate or negative error value
 *
 * Public clock API extends clk_round_rate which is a ceiling function. This
 * function is a floor function implemented as a binary search using the
 * ceiling function.
 */
static long msm_spi_clk_max_rate(struct clk *clk, unsigned long rate)
{
	long lowest_available, nearest_low, step_size, cur;
	long step_direction = -1;
	long guess = rate;
	int max_steps = 10;

	cur = clk_round_rate(clk, rate);
	if (cur == rate)
		return rate;

	/* if we got here then: cur > rate */
	lowest_available = clk_round_rate(clk, 0);
	if (lowest_available > rate)
		return -EINVAL;

	step_size = (rate - lowest_available) >> 1;
	nearest_low = lowest_available;

	while (max_steps-- && step_size) {
		guess += step_size * step_direction;

		cur = clk_round_rate(clk, guess);

		if ((cur < rate) && (cur > nearest_low))
			nearest_low = cur;

		/*
		 * if we stepped too far, then start stepping in the other
		 * direction with half the step size
		 */
		if (((cur > rate) && (step_direction > 0)) ||
		    ((cur < rate) && (step_direction < 0))) {
			step_direction = -step_direction;
			step_size >>= 1;
		}
	}
	return nearest_low;
}

static void msm_spi_clock_set(struct msm_spi *dd, int speed)
{
	long rate;
	int rc;

	rate = msm_spi_clk_max_rate(dd->clk, speed);
	if (rate < 0) {
		dev_err(dd->dev,
			"%s: no match found for requested clock frequency:%d",
			__func__, speed);
		return;
	}
	rc = clk_set_rate(dd->clk, rate);
	if (!rc) {
		dd->clock_speed = rate;
		dd->last_required_clock_speed = speed;
	}
}

static void msm_spi_clk_path_vote(struct msm_spi *dd, u32 rate)
{
	if (dd->bus_cl_hdl) {
		u64 ib = rate * dd->pdata->bus_width;

		msm_bus_scale_update_bw(dd->bus_cl_hdl, 0, ib);
	}
}

static void msm_spi_clk_path_teardown(struct msm_spi *dd)
{
	msm_spi_clk_path_vote(dd, 0);

	if (dd->bus_cl_hdl) {
		msm_bus_scale_unregister(dd->bus_cl_hdl);
		dd->bus_cl_hdl = NULL;
	}
}

/**
 * msm_spi_clk_path_postponed_register: reg with bus-scaling after it is probed
 *
 * @return zero on success
 *
 * Workaround: SPI driver may be probed before the bus scaling driver. Calling
 * msm_bus_scale_register_client() will fail if the bus scaling driver is not
 * ready yet. Thus, this function should be called not from probe but from a
 * later context. Also, this function may be called more then once before
 * register succeed. At this case only one error message will be logged. At boot
 * time all clocks are on, so earlier SPI transactions should succeed.
 */
static int msm_spi_clk_path_postponed_register(struct msm_spi *dd)
{
	int ret = 0;

	dd->bus_cl_hdl = msm_bus_scale_register(dd->pdata->master_id,
						MSM_BUS_SLAVE_EBI_CH0,
						(char *)dev_name(dd->dev),
						false);

	if (IS_ERR_OR_NULL(dd->bus_cl_hdl)) {
		ret = (dd->bus_cl_hdl ? PTR_ERR(dd->bus_cl_hdl) : -EAGAIN);
		dev_err(dd->dev, "Failed bus registration Err %d", ret);
	}

	return ret;
}

static void msm_spi_clk_path_init(struct msm_spi *dd)
{
	/*
	 * bail out if path voting is diabled (master_id == 0) or if it is
	 * already registered (client_hdl != 0)
	 */
	if (!dd->pdata->master_id || dd->bus_cl_hdl)
		return;

	/* on failure try again later */
	if (msm_spi_clk_path_postponed_register(dd))
		return;
}

static int msm_spi_calculate_size(int *fifo_size, int *block_size, int block,
				  int mult)
{
	int words;

	switch (block) {
	case 0:
		words = 1; /* 4 bytes */
		break;
	case 1:
		words = 4; /* 16 bytes */
		break;
	case 2:
		words = 8; /* 32 bytes */
		break;
	default:
		return -EINVAL;
	}

	switch (mult) {
	case 0:
		*fifo_size = words * 2;
		break;
	case 1:
		*fifo_size = words * 4;
		break;
	case 2:
		*fifo_size = words * 8;
		break;
	case 3:
		*fifo_size = words * 16;
		break;
	default:
		return -EINVAL;
	}

	*block_size = words * sizeof(u32); /* in bytes */
	return 0;
}

static void msm_spi_calculate_fifo_size(struct msm_spi *dd)
{
	u32 spi_iom;
	int block;
	int mult;

	spi_iom = readl_relaxed(dd->base + SPI_IO_MODES);

	block = (spi_iom & SPI_IO_M_INPUT_BLOCK_SIZE) >> INPUT_BLOCK_SZ_SHIFT;
	mult = (spi_iom & SPI_IO_M_INPUT_FIFO_SIZE) >> INPUT_FIFO_SZ_SHIFT;
	if (msm_spi_calculate_size(&dd->input_fifo_size, &dd->input_block_size,
				   block, mult)) {
		goto fifo_size_err;
	}

	block = (spi_iom & SPI_IO_M_OUTPUT_BLOCK_SIZE) >> OUTPUT_BLOCK_SZ_SHIFT;
	mult = (spi_iom & SPI_IO_M_OUTPUT_FIFO_SIZE) >> OUTPUT_FIFO_SZ_SHIFT;
	if (msm_spi_calculate_size(&dd->output_fifo_size,
				   &dd->output_block_size, block, mult)) {
		goto fifo_size_err;
	}
	return;

fifo_size_err:
	pr_err("%s: invalid FIFO size, SPI_IO_MODES=0x%x\n", __func__, spi_iom);
	return;
}

static void msm_spi_read_word_from_fifo(struct msm_spi *dd)
{
	u32 data_in;
	int i;
	int shift;
	int read_bytes =
		(dd->pack_words ? SPI_MAX_BYTES_PER_WORD : dd->bytes_per_word);

	data_in = readl_relaxed(dd->base + SPI_INPUT_FIFO);
	if (dd->read_buf) {
		for (i = 0; (i < read_bytes) && dd->rx_bytes_remaining; i++) {
			/* The data format depends on bytes_per_word:
			   4 bytes: 0x12345678
			   3 bytes: 0x00123456
			   2 bytes: 0x00001234
			   1 byte : 0x00000012
			*/
			shift = BITS_PER_BYTE * i;
			*dd->read_buf++ = (data_in & (0xFF << shift)) >> shift;
			dd->rx_bytes_remaining--;
		}
	} else {
		if (dd->rx_bytes_remaining >= read_bytes)
			dd->rx_bytes_remaining -= read_bytes;
		else
			dd->rx_bytes_remaining = 0;
	}

	dd->read_xfr_cnt++;
}

static inline bool msm_spi_is_valid_state(struct msm_spi *dd)
{
	u32 spi_op = readl_relaxed(dd->base + SPI_STATE);

	return spi_op & SPI_OP_STATE_VALID;
}

static inline int msm_spi_set_state(struct msm_spi *dd,
				    enum msm_spi_state state)
{
	enum msm_spi_state cur_state;

	cur_state = readl_relaxed(dd->base + SPI_STATE);
	/* Per spec:
	   For PAUSE_STATE to RESET_STATE, two writes of (10) are required */
	if (((cur_state & SPI_OP_STATE) == SPI_OP_STATE_PAUSE) &&
	    (state == SPI_OP_STATE_RESET)) {
		writel_relaxed(SPI_OP_STATE_CLEAR_BITS, dd->base + SPI_STATE);
		writel_relaxed(SPI_OP_STATE_CLEAR_BITS, dd->base + SPI_STATE);
	} else {
		writel_relaxed((cur_state & ~SPI_OP_STATE) | state,
			       dd->base + SPI_STATE);
	}
	return 0;
}

/**
 * msm_spi_set_bpw_and_no_io_flags: configure N, and no-input/no-output flags
 */
static inline void msm_spi_set_bpw_and_no_io_flags(struct msm_spi *dd,
						   u32 *config, int n)
{
	*config &= ~(SPI_NO_INPUT | SPI_NO_OUTPUT);

	if (n != (*config & SPI_CFG_N))
		*config = (*config & ~SPI_CFG_N) | n;
}

/**
 * msm_spi_calc_spi_config_loopback_and_input_first: Calculate the values that
 * should be updated into SPI_CONFIG's LOOPBACK and INPUT_FIRST flags
 * @return calculatd value for SPI_CONFIG
 */
static u32 msm_spi_calc_spi_config_loopback_and_input_first(u32 spi_config,
							    u8 mode)
{
	if (mode & SPI_LOOP)
		spi_config |= SPI_CFG_LOOPBACK;
	else
		spi_config &= ~SPI_CFG_LOOPBACK;

	if (mode & SPI_CPHA)
		spi_config &= ~SPI_CFG_INPUT_FIRST;
	else
		spi_config |= SPI_CFG_INPUT_FIRST;

	return spi_config;
}

/**
 * msm_spi_set_spi_config: prepares register SPI_CONFIG to process the
 * next transfer
 */
static void msm_spi_set_spi_config(struct msm_spi *dd, int bpw)
{
	u32 spi_config;

	if ((dd->last_spi_dev_mode == dd->spi->mode) &&
	    (dd->clock_speed < SPI_HS_MIN_RATE))
		return;
	spi_config = readl_relaxed(dd->base + SPI_CONFIG);
	spi_config = msm_spi_calc_spi_config_loopback_and_input_first(
		spi_config, dd->spi->mode);

	if (dd->qup_ver == SPI_QUP_VERSION_NONE)
		/* flags removed from SPI_CONFIG in QUP version-2 */
		msm_spi_set_bpw_and_no_io_flags(dd, &spi_config, bpw - 1);

	/*
	 * HS_MODE improves signal stability for spi-clk high rates
	 * but is invalid in LOOPBACK mode.
	 */
	if ((dd->clock_speed >= SPI_HS_MIN_RATE) && !(dd->spi->mode & SPI_LOOP))
		spi_config |= SPI_CFG_HS_MODE;
	else
		spi_config &= ~SPI_CFG_HS_MODE;

	writel_relaxed(spi_config, dd->base + SPI_CONFIG);
	dd->last_spi_dev_mode = dd->spi->mode;
}

/**
 * msm_spi_set_mx_counts: set SPI_MX_INPUT_COUNT and SPI_MX_INPUT_COUNT
 * for FIFO-mode.
 * @n_words The number of reads/writes of size N.
 */
static void msm_spi_set_mx_counts(struct msm_spi *dd, u32 n_words)
{
	/*
	 * For FIFO mode:
	 *   - Set the MX_OUTPUT_COUNT/MX_INPUT_COUNT registers to 0
	 *   - Set the READ/WRITE_COUNT registers to 0 (infinite mode)
	 *     or num bytes (finite mode) if less than fifo worth of data.
	 */
	if (n_words <= dd->input_fifo_size)
		msm_spi_set_write_count(dd, n_words);
	else
		msm_spi_set_write_count(dd, 0);
	writel_relaxed(0, dd->base + SPI_MX_OUTPUT_COUNT);

	if (dd->cur_transfer->rx_buf) {
		if (n_words <= dd->input_fifo_size)
			writel_relaxed(n_words, dd->base + SPI_MX_READ_COUNT);
		else
			writel_relaxed(0, dd->base + SPI_MX_READ_COUNT);
		writel_relaxed(0, dd->base + SPI_MX_INPUT_COUNT);
	} else {
		/* Dirty hack: if the transfer is TX only, the RX FIFO
		 * must be read, else the controller SPI hangs. So one
		 * byte is read in this case.
		 */
		writel_relaxed(1, dd->base + SPI_MX_READ_COUNT);
		writel_relaxed(1, dd->base + SPI_MX_INPUT_COUNT);
	}
}

static void msm_spi_write_word_to_fifo(struct msm_spi *dd)
{
	u32 word;
	u8 byte;
	int i;
	int write_bytes =
		(dd->pack_words ? SPI_MAX_BYTES_PER_WORD : dd->bytes_per_word);

	word = 0;
	if (dd->write_buf) {
		for (i = 0; (i < write_bytes) && dd->tx_bytes_remaining; i++) {
			dd->tx_bytes_remaining--;
			byte = *dd->write_buf++;
			word |= (byte << (BITS_PER_BYTE * i));
		}
	} else if (dd->tx_bytes_remaining > write_bytes)
		dd->tx_bytes_remaining -= write_bytes;
	else
		dd->tx_bytes_remaining = 0;
	dd->write_xfr_cnt++;

	writel_relaxed(word, dd->base + SPI_OUTPUT_FIFO);
}

static inline void msm_spi_write_rmn_to_fifo(struct msm_spi *dd)
{
	int count = 0;

	if (dd->tx_mode == SPI_FIFO_MODE) {
		/* Here the TX FIFO is supposed to be empty, so we don't waste time to
		 * test SPI_OPERATIONAL	& SPI_OP_OUTPUT_FIFO_FULL. We just write
		 * output_fifo_size bytes at max.
		 */
		while ((dd->tx_bytes_remaining > 0) &&
		       (count < dd->output_fifo_size)) {
			msm_spi_write_word_to_fifo(dd);
			count++;
		}
	}

	if (dd->tx_mode == SPI_BLOCK_MODE) {
		while (dd->tx_bytes_remaining &&
		       (count < dd->output_block_size)) {
			msm_spi_write_word_to_fifo(dd);
			count += SPI_MAX_BYTES_PER_WORD;
		}
	}
}

/**
 * msm_spi_set_transfer_mode: Chooses optimal transfer mode. Sets dd->mode and
 * prepares to process a transfer.
 */
static void msm_spi_set_transfer_mode(struct msm_spi *dd, u8 bpw,
				      u32 read_count)
{
	dd->rx_mode = SPI_FIFO_MODE;
	if (dd->cur_transfer->rx_buf)
		dd->read_len = dd->cur_transfer->len;
	else {
		/* Dirty hack: if the transfer is TX only, the RX FIFO
		 * must be read, else the controller SPI hangs. So one
		 * byte is read in this case.
		 */
		dd->read_len = 1;
		dd->rx_bytes_remaining = 1;
	}
	dd->tx_mode = SPI_FIFO_MODE;
	dd->write_len = dd->cur_transfer->len;
	dd->last_tx_mode = dd->tx_mode;
	dd->last_rx_mode = dd->rx_mode;
}

/**
 * msm_spi_set_qup_io_modes: prepares register QUP_IO_MODES to process a
 * transfer
 */
static void msm_spi_set_qup_io_modes(struct msm_spi *dd)
{
	u32 spi_iom;

	if ((dd->cur_msg_len % SPI_MAX_BYTES_PER_WORD == 0) &&
	    (dd->cur_transfer->bits_per_word) &&
	    (dd->cur_transfer->bits_per_word <= 32) &&
	    (dd->cur_transfer->bits_per_word % 8 == 0))
		dd->pack_words = true;
	else
		dd->pack_words = false;
	if ((dd->pack_words == dd->last_pack_words) &&
	    (dd->tx_mode == dd->last_tx_mode) &&
	    (dd->rx_mode == dd->last_rx_mode))
		return;

	spi_iom = readl_relaxed(dd->base + SPI_IO_MODES);
	/* Set input and output transfer mode: FIFO, DMOV, or BAM */
	spi_iom &= ~(SPI_IO_M_INPUT_MODE | SPI_IO_M_OUTPUT_MODE);
	spi_iom = (spi_iom | (dd->tx_mode << OUTPUT_MODE_SHIFT));
	spi_iom = (spi_iom | (dd->rx_mode << INPUT_MODE_SHIFT));

	/* Always enable packing for the BAM mode and for non BAM mode only
	 * if bpw is % 8 and transfer length is % 4 Bytes.
	 */
	if (dd->pack_words)
		spi_iom |= SPI_IO_M_PACK_EN | SPI_IO_M_UNPACK_EN;
	else {
		spi_iom &= ~(SPI_IO_M_PACK_EN | SPI_IO_M_UNPACK_EN);
		spi_iom |= SPI_IO_M_OUTPUT_BIT_SHIFT_EN;
	}
	writel_relaxed(spi_iom, dd->base + SPI_IO_MODES);
	dd->last_pack_words = dd->pack_words;
}

static u32 msm_spi_calc_spi_ioc_clk_polarity(u32 spi_ioc, u8 mode)
{
	if (mode & SPI_CPOL)
		spi_ioc |= SPI_IO_C_CLK_IDLE_HIGH;
	else
		spi_ioc &= ~SPI_IO_C_CLK_IDLE_HIGH;
	return spi_ioc;
}

/**
 * msm_spi_set_spi_io_control: prepares register SPI_IO_CONTROL to process the
 * next transfer
 * @return the new set value of SPI_IO_CONTROL
 */
static u32 msm_spi_set_spi_io_control(struct msm_spi *dd)
{
	u32 spi_ioc, spi_ioc_orig, chip_select;

	if (dd->cur_transfer->cs_change == dd->last_cs_change)
		return dd->last_spi_ioc;
	spi_ioc = readl_relaxed(dd->base + SPI_IO_CONTROL);
	spi_ioc_orig = spi_ioc;
	spi_ioc = msm_spi_calc_spi_ioc_clk_polarity(spi_ioc, dd->spi->mode);
	/* Set chip-select */
	chip_select = dd->spi->chip_select << 2;
	if ((spi_ioc & SPI_IO_C_CS_SELECT) != chip_select)
		spi_ioc = (spi_ioc & ~SPI_IO_C_CS_SELECT) | chip_select;
	if (!dd->cur_transfer->cs_change)
		spi_ioc |= SPI_IO_C_MX_CS_MODE;
	if (spi_ioc != spi_ioc_orig)
		writel_relaxed(spi_ioc, dd->base + SPI_IO_CONTROL);
	dd->last_spi_ioc = spi_ioc;
	dd->last_cs_change = dd->cur_transfer->cs_change;
	/*
	 * Ensure that the IO control mode register gets written
	 * before proceeding with the transfer.
	 */
	mb();
	return spi_ioc;
}

/**
 * msm_spi_set_qup_op_mask: prepares register QUP_OPERATIONAL_MASK to process
 * the next transfer
 */
static void msm_spi_set_qup_op_mask(struct msm_spi *dd)
{
	u32 mask;

	if ((dd->tx_mode == dd->last_tx_mode) && dd->op_mask)
		return;
	/* mask INPUT and OUTPUT service flags in to prevent IRQs on FIFO status
	 * change in BAM mode */
	mask = (dd->tx_mode == SPI_BAM_MODE) ?
		       QUP_OP_MASK_OUTPUT_SERVICE_FLAG |
			       QUP_OP_MASK_INPUT_SERVICE_FLAG :
		       0;
	writel_relaxed(mask, dd->base + QUP_OPERATIONAL_MASK);
	dd->op_mask = true;
}

static void get_transfer_length(struct msm_spi *dd)
{
	struct spi_transfer *xfer = dd->cur_transfer;

	dd->cur_msg_len = 0;
	dd->read_len = dd->write_len = 0;

	if (xfer->tx_buf)
		dd->write_len = xfer->len;
	if (xfer->rx_buf)
		dd->read_len = xfer->len;
	dd->cur_msg_len = xfer->len;
}

static int msm_spi_process_transfer(struct msm_spi *dd)
{
	u8 bpw;
	u32 max_speed;
	u32 read_count;
	u32 timeout;
	u32 spi_ioc;
	u32 int_loopback = 0;
	int status = 0;
	int run = 0;
	int count;

	get_transfer_length(dd);
	dd->cur_tx_transfer = dd->cur_transfer;
	dd->cur_rx_transfer = dd->cur_transfer;
	dd->write_xfr_cnt = dd->read_xfr_cnt = 0;
	dd->tx_bytes_remaining = dd->cur_msg_len;
	dd->rx_bytes_remaining = dd->cur_msg_len;
	dd->read_buf = dd->cur_transfer->rx_buf;
	dd->write_buf = dd->cur_transfer->tx_buf;
	dd->tx_done = false;
	dd->rx_done = false;
	if (dd->cur_transfer->bits_per_word)
		bpw = dd->cur_transfer->bits_per_word;
	else
		bpw = 8;
	dd->bytes_per_word = (bpw + 7) / 8;

	if (dd->cur_transfer->speed_hz)
		max_speed = dd->cur_transfer->speed_hz;
	else
		max_speed = dd->spi->max_speed_hz;
	if (!dd->clock_speed || max_speed != dd->last_required_clock_speed)
		msm_spi_clock_set(dd, max_speed);

	timeout = 100 * msecs_to_jiffies(DIV_ROUND_UP(
				dd->cur_msg_len * 8,
				DIV_ROUND_UP(max_speed, MSEC_PER_SEC)));

	read_count = DIV_ROUND_UP(dd->cur_msg_len, dd->bytes_per_word);
	if (dd->spi->mode & SPI_LOOP)
		int_loopback = 1;

	msm_spi_set_transfer_mode(dd, bpw, read_count);
	msm_spi_set_mx_counts(dd, read_count);
	msm_spi_set_qup_io_modes(dd);
	msm_spi_set_spi_config(dd, bpw);
	msm_spi_set_qup_config(dd, bpw);
	spi_ioc = msm_spi_set_spi_io_control(dd);
	msm_spi_set_qup_op_mask(dd);

	/* Begin of the polling transfer */
	do {
		count = 0;
		if (!run) {
			msm_spi_set_state(dd, SPI_OP_STATE_RUN);
			run = 1;
		}
		/* Here the TX is supposed empty. */
		msm_spi_write_rmn_to_fifo(dd);
		if (dd->rx_mode == SPI_FIFO_MODE) {
			/* Active wait of the end of RX operation in the RX FIFO */
			while (!(readl_relaxed(dd->base + SPI_OPERATIONAL) &
				 SPI_OP_INPUT_SERVICE_FLAG))
				;
			do {
				msm_spi_read_word_from_fifo(dd);
				count++;
			} while ((dd->rx_bytes_remaining > 0) &&
				 (count < dd->input_fifo_size));
		}
	} while ((dd->tx_bytes_remaining > 0));
	dd->tx_done = false;
	dd->rx_done = false;
	dd->tx_mode = SPI_MODE_NONE;
	dd->rx_mode = SPI_MODE_NONE;

	msm_spi_set_state(dd, SPI_OP_STATE_RESET);
	if (!dd->cur_transfer->cs_change)
		writel_relaxed(spi_ioc & ~SPI_IO_C_MX_CS_MODE,
			       dd->base + SPI_IO_CONTROL);
	return status;
}

static inline void msm_spi_set_cs(struct spi_device *spi, bool set_flag)
{
	struct msm_spi *dd = spi_master_get_devdata(spi->master);
	u32 spi_ioc;
	u32 spi_ioc_orig;
	int rc;
	static int count = 0;

	if (dd->suspended) {
		dev_err(dd->dev, "%s: SPI operational state not valid %d\n",
			__func__, dd->suspended);
		return;
	}

	if (dd->pdata->is_shared) {
		rc = get_local_resources(dd);
		if (rc)
			return;
	}

	msm_spi_clk_path_vote(dd, spi->max_speed_hz);

	if (!(spi->mode & SPI_CS_HIGH))
		set_flag = !set_flag;
	if (!dd->spi_ioc_set_cs_set || !dd->spi_ioc_set_cs_unset) {
		spi_ioc = readl_relaxed(dd->base + SPI_IO_CONTROL);
		/* These are 2 spurious calls to msm_spi_set_cs before real transfers. */
		if (count >= 2) {
			if (set_flag)
				dd->spi_ioc_set_cs_set = spi_ioc;
			else
				dd->spi_ioc_set_cs_unset = spi_ioc;
		}
		count++;
	} else {
		if (set_flag)
			spi_ioc = dd->spi_ioc_set_cs_set;
		else
			spi_ioc = dd->spi_ioc_set_cs_unset;
	}
	spi_ioc_orig = spi_ioc;
	if (set_flag)
		spi_ioc |= SPI_IO_C_FORCE_CS;
	else
		spi_ioc &= ~SPI_IO_C_FORCE_CS;

	if (spi_ioc != spi_ioc_orig)
		writel_relaxed(spi_ioc, dd->base + SPI_IO_CONTROL);
	if (dd->pdata->is_shared)
		put_local_resources(dd);
}

static void reset_core(struct msm_spi *dd)
{
	u32 spi_ioc;
	msm_spi_register_init(dd);
	/*
	 * The SPI core generates a bogus input overrun error on some targets,
	 * when a transition from run to reset state occurs and if the FIFO has
	 * an odd number of entries. Hence we disable the INPUT_OVER_RUN_ERR_EN
	 * bit.
	 */
	msm_spi_enable_error_flags(dd);

	spi_ioc = readl_relaxed(dd->base + SPI_IO_CONTROL);
	spi_ioc |= SPI_IO_C_NO_TRI_STATE;
	writel_relaxed(spi_ioc, dd->base + SPI_IO_CONTROL);
	/*
	 * Ensure that the IO control is written to before returning.
	 */
	mb();
	msm_spi_set_state(dd, SPI_OP_STATE_RESET);
}

static void put_local_resources(struct msm_spi *dd)
{
	if (IS_ERR_OR_NULL(dd->clk) || IS_ERR_OR_NULL(dd->pclk)) {
		dev_err(dd->dev, "%s: error clk put\n", __func__);
		return;
	}
	clk_disable_unprepare(dd->clk);
	dd->clock_speed = 0;
	clk_disable_unprepare(dd->pclk);

	/* Free  the spi clk, miso, mosi, cs gpio */
	if (dd->pdata && dd->pdata->gpio_release)
		dd->pdata->gpio_release();

	msm_spi_free_gpios(dd);
}

static int get_local_resources(struct msm_spi *dd)
{
	int ret = -EINVAL;

	if (IS_ERR_OR_NULL(dd->clk) || IS_ERR_OR_NULL(dd->pclk)) {
		dev_err(dd->dev, "%s: error clk put\n", __func__);
		return ret;
	}

	/* Configure the spi clk, miso, mosi and cs gpio */
	if (dd->pdata->gpio_config) {
		ret = dd->pdata->gpio_config();
		if (ret) {
			dev_err(dd->dev, "%s: error configuring GPIOs\n",
				__func__);
			return ret;
		}
	}

	ret = msm_spi_request_gpios(dd);
	if (ret)
		return ret;

	ret = clk_prepare_enable(dd->clk);
	if (ret)
		goto clk0_err;
	ret = clk_prepare_enable(dd->pclk);
	if (ret)
		goto clk1_err;

	return 0;

clk1_err:
	clk_disable_unprepare(dd->clk);
clk0_err:
	msm_spi_free_gpios(dd);
	return ret;
}

/**
 * msm_spi_transfer_one: To process one spi transfer at a time
 * @master: spi master controller reference
 * @msg: one multi-segment SPI transaction
 * @return zero on success or negative error value
 *
 */
static int msm_spi_transfer_one(struct spi_master *master,
				struct spi_device *spi,
				struct spi_transfer *xfer)
{
	struct msm_spi *dd;
	unsigned long flags;
	u32 status_error = 0;

	dd = spi_master_get_devdata(master);

	/* Check message parameters */
	if (xfer->speed_hz > dd->pdata->max_clock_speed ||
	    (xfer->bits_per_word &&
	     (xfer->bits_per_word < 4 || xfer->bits_per_word > 32)) ||
	    (xfer->tx_buf == NULL && xfer->rx_buf == NULL)) {
		dev_err(dd->dev,
			"Invalid transfer: %d Hz, %d bpw tx=%p, rx=%p\n",
			xfer->speed_hz, xfer->bits_per_word, xfer->tx_buf,
			xfer->rx_buf);
		return -EINVAL;
	}
	dd->spi = spi;
	dd->cur_transfer = xfer;

	mutex_lock(&dd->core_lock);

	spin_lock_irqsave(&dd->queue_lock, flags);
	dd->transfer_pending = 1;
	spin_unlock_irqrestore(&dd->queue_lock, flags);
	/*
	 * get local resources for each transfer to ensure we're in a good
	 * state and not interfering with other EE's using this device
	 */
	if (dd->pdata->is_shared) {
		if (get_local_resources(dd)) {
			mutex_unlock(&dd->core_lock);
			spi_finalize_current_message(master);
			return -EINVAL;
		}

		reset_core(dd);
	}

	if (dd->suspended || !msm_spi_is_valid_state(dd)) {
		if (!msm_spi_is_valid_state(dd)) {
			dev_err(dd->dev,
				"%s: SPI operational state not valid\n",
				__func__);
			status_error = 1;
		}
	}

	if (!status_error)
		status_error = msm_spi_process_transfer(dd);

	spin_lock_irqsave(&dd->queue_lock, flags);
	dd->transfer_pending = 0;
	spin_unlock_irqrestore(&dd->queue_lock, flags);

	/*
	 * Put local resources prior to calling finalize to ensure the hw
	 * is in a known state before notifying the calling thread (which is a
	 * different context since we're running in the spi kthread here) to
	 * prevent race conditions between us and any other EE's using this hw.
	 */
	if (dd->pdata->is_shared)
		put_local_resources(dd);

	mutex_unlock(&dd->core_lock);
	if (dd->suspended)
		wake_up_interruptible(&dd->continue_suspend);
	return status_error;
}

static int msm_spi_prepare_transfer_hardware(struct spi_master *master)
{
	return 0;
}

static int msm_spi_unprepare_transfer_hardware(struct spi_master *master)
{
	return 0;
}

static int msm_spi_setup(struct spi_device *spi)
{
	struct msm_spi *dd;
	int rc = 0;
	u32 spi_ioc;
	u32 spi_config;
	u32 mask;

	if (spi->bits_per_word < 4 || spi->bits_per_word > 32) {
		dev_err(&spi->dev, "%s: invalid bits_per_word %d\n", __func__,
			spi->bits_per_word);
		return -EINVAL;
	}
	if (spi->chip_select > SPI_NUM_CHIPSELECTS - 1) {
		dev_err(&spi->dev, "%s, chip select %d exceeds max value %d\n",
			__func__, spi->chip_select, SPI_NUM_CHIPSELECTS - 1);
		return -EINVAL;
	}

	dd = spi_master_get_devdata(spi->master);

	rc = pm_runtime_get_sync(dd->dev);
	if (rc < 0 && !dd->is_init_complete && pm_runtime_enabled(dd->dev)) {
		pm_runtime_set_suspended(dd->dev);
		pm_runtime_put_sync(dd->dev);
		rc = 0;
		goto err_setup_exit;
	} else
		rc = 0;

	mutex_lock(&dd->core_lock);

	/* Counter-part of system-suspend when runtime-pm is not enabled. */
	if (!pm_runtime_enabled(dd->dev)) {
		rc = msm_spi_pm_resume_runtime(dd->dev);
		if (rc < 0 && !dd->is_init_complete) {
			rc = 0;
			mutex_unlock(&dd->core_lock);
			goto err_setup_exit;
		}
	}

	if (dd->suspended) {
		rc = -EBUSY;
		mutex_unlock(&dd->core_lock);
		goto err_setup_exit;
	}

	if (dd->pdata->is_shared) {
		rc = get_local_resources(dd);
		if (rc)
			goto no_resources;
	}

	spi_ioc = readl_relaxed(dd->base + SPI_IO_CONTROL);
	mask = SPI_IO_C_CS_N_POLARITY_0 << spi->chip_select;
	if (spi->mode & SPI_CS_HIGH)
		spi_ioc |= mask;
	else
		spi_ioc &= ~mask;
	spi_ioc = msm_spi_calc_spi_ioc_clk_polarity(spi_ioc, spi->mode);

	writel_relaxed(spi_ioc, dd->base + SPI_IO_CONTROL);

	spi_config = readl_relaxed(dd->base + SPI_CONFIG);
	spi_config = msm_spi_calc_spi_config_loopback_and_input_first(
		spi_config, spi->mode);
	writel_relaxed(spi_config, dd->base + SPI_CONFIG);

	/* Ensure previous write completed before disabling the clocks */
	mb();
	if (dd->pdata->is_shared)
		put_local_resources(dd);
	/* Counter-part of system-resume when runtime-pm is not enabled. */
	if (!pm_runtime_enabled(dd->dev))
		msm_spi_pm_suspend_runtime(dd->dev);

no_resources:
	mutex_unlock(&dd->core_lock);
err_setup_exit:
	return rc;
}

#ifdef CONFIG_DEBUG_FS

static int debugfs_iomem_x32_set(void *data, u64 val)
{
	struct msm_spi_debugfs_data *reg = (struct msm_spi_debugfs_data *)data;
	struct msm_spi *dd = reg->dd;
	int ret;

	ret = pm_runtime_get_sync(dd->dev);
	if (ret < 0)
		return ret;

	writel_relaxed(val, (dd->base + reg->offset));
	/* Ensure the previous write completed. */
	mb();

	pm_runtime_mark_last_busy(dd->dev);
	pm_runtime_put_autosuspend(dd->dev);
	return 0;
}

static int debugfs_iomem_x32_get(void *data, u64 *val)
{
	struct msm_spi_debugfs_data *reg = (struct msm_spi_debugfs_data *)data;
	struct msm_spi *dd = reg->dd;
	int ret;

	ret = pm_runtime_get_sync(dd->dev);
	if (ret < 0)
		return ret;
	*val = readl_relaxed(dd->base + reg->offset);
	/* Ensure the previous read completed. */
	mb();

	pm_runtime_mark_last_busy(dd->dev);
	pm_runtime_put_autosuspend(dd->dev);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_iomem_x32, debugfs_iomem_x32_get,
			debugfs_iomem_x32_set, "0x%08llx\n");

static void spi_debugfs_init(struct msm_spi *dd)
{
	char dir_name[20];

	scnprintf(dir_name, sizeof(dir_name), "%s_dbg", dev_name(dd->dev));
	dd->dent_spi = debugfs_create_dir(dir_name, NULL);
	if (dd->dent_spi) {
		int i;

		for (i = 0; i < ARRAY_SIZE(debugfs_spi_regs); i++) {
			dd->reg_data[i].offset = debugfs_spi_regs[i].offset;
			dd->reg_data[i].dd = dd;
			dd->debugfs_spi_regs[i] = debugfs_create_file(
				debugfs_spi_regs[i].name,
				debugfs_spi_regs[i].mode, dd->dent_spi,
				&dd->reg_data[i], &fops_iomem_x32);
		}
	}
}

static void spi_debugfs_exit(struct msm_spi *dd)
{
	if (dd->dent_spi) {
		int i;

		debugfs_remove_recursive(dd->dent_spi);
		dd->dent_spi = NULL;
		for (i = 0; i < ARRAY_SIZE(debugfs_spi_regs); i++)
			dd->debugfs_spi_regs[i] = NULL;
	}
}
#else
static void spi_debugfs_init(struct msm_spi *dd)
{
}
static void spi_debugfs_exit(struct msm_spi *dd)
{
}
#endif

/* ===Device attributes begin=== */
static ssize_t show_stats(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct msm_spi *dd = spi_master_get_devdata(master);

	return snprintf(buf, PAGE_SIZE,
			"Device       %s\n"
			"rx fifo_size = %d spi words\n"
			"tx fifo_size = %d spi words\n"
			"rx block size = %d bytes\n"
			"tx block size = %d bytes\n"
			"input burst size = %d bytes\n"
			"output burst size = %d bytes\n"
			"--statistics--\n"
			"Rx isrs  = %d\n"
			"Tx isrs  = %d\n"
			"--debug--\n"
			"NA yet\n",
			dev_name(dev), dd->input_fifo_size,
			dd->output_fifo_size, dd->input_block_size,
			dd->output_block_size, dd->input_burst_size,
			dd->output_burst_size, dd->stat_rx, dd->stat_tx);
}

/* Reset statistics on write */
static ssize_t set_stats(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct msm_spi *dd = dev_get_drvdata(dev);
	dd->stat_rx = 0;
	dd->stat_tx = 0;
	return count;
}

static DEVICE_ATTR(stats, S_IRUGO | S_IWUSR, show_stats, set_stats);

static struct attribute *dev_attrs[] = {
	&dev_attr_stats.attr,
	NULL,
};

static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};
/* ===Device attributes end=== */

enum msm_spi_dt_entry_status {
	DT_REQ, /* Required:  fail if missing */
	DT_SGST, /* Suggested: warn if missing */
	DT_OPT, /* Optional:  don't warn if missing */
};

enum msm_spi_dt_entry_type {
	DT_U32,
	DT_GPIO,
	DT_BOOL,
};

struct msm_spi_dt_to_pdata_map {
	const char *dt_name;
	void *ptr_data;
	enum msm_spi_dt_entry_status status;
	enum msm_spi_dt_entry_type type;
	int default_val;
};

static int msm_spi_dt_to_pdata_populate(struct platform_device *pdev,
					struct msm_spi_platform_data *pdata,
					struct msm_spi_dt_to_pdata_map *itr)
{
	int ret, err = 0;
	struct device_node *node = pdev->dev.of_node;

	for (; itr->dt_name; ++itr) {
		switch (itr->type) {
		case DT_GPIO:
			ret = of_get_named_gpio(node, itr->dt_name, 0);
			if (ret >= 0) {
				*((int *)itr->ptr_data) = ret;
				ret = 0;
			}
			break;
		case DT_U32:
			ret = of_property_read_u32(node, itr->dt_name,
						   (u32 *)itr->ptr_data);
			break;
		case DT_BOOL:
			*((bool *)itr->ptr_data) =
				of_property_read_bool(node, itr->dt_name);
			ret = 0;
			break;
		default:
			dev_err(&pdev->dev, "%d is an unknown DT entry type\n",
				itr->type);
			ret = -EBADE;
		}

		dev_dbg(&pdev->dev, "DT entry ret:%d name:%s val:%d\n", ret,
			itr->dt_name, *((int *)itr->ptr_data));

		if (ret) {
			*((int *)itr->ptr_data) = itr->default_val;

			if (itr->status < DT_OPT) {
				dev_err(&pdev->dev, "Missing '%s' DT entry\n",
					itr->dt_name);

				/* cont on err to dump all missing entries */
				if (itr->status == DT_REQ && !err)
					err = ret;
			}
		}
	}

	return err;
}

/**
 * msm_spi_dt_to_pdata: create pdata and read gpio config from device tree
 */
struct msm_spi_platform_data *msm_spi_dt_to_pdata(struct platform_device *pdev,
						  struct msm_spi *dd)
{
	struct msm_spi_platform_data *pdata;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("Unable to allocate platform data\n");
		return NULL;
	} else {
		struct msm_spi_dt_to_pdata_map map[] = {
			{ "spi-max-frequency", &pdata->max_clock_speed, DT_SGST,
			  DT_U32, 0 },
			{ "qcom,infinite-mode", &pdata->infinite_mode, DT_OPT,
			  DT_U32, 0 },
			{ "qcom,master-id", &pdata->master_id, DT_SGST, DT_U32,
			  0 },
			{ "qcom,bus-width", &pdata->bus_width, DT_OPT, DT_U32,
			  8 },
			{ "qcom,ver-reg-exists", &pdata->ver_reg_exists, DT_OPT,
			  DT_BOOL, 0 },
			{ "qcom,use-bam", &pdata->use_bam, DT_OPT, DT_U32, 0 },
			{ "qcom,use-pinctrl", &pdata->use_pinctrl, DT_OPT,
			  DT_BOOL, 0 },
			{ "qcom,bam-consumer-pipe-index",
			  &pdata->bam_consumer_pipe_index, DT_OPT, DT_U32, 0 },
			{ "qcom,bam-producer-pipe-index",
			  &pdata->bam_producer_pipe_index, DT_OPT, DT_U32, 0 },
			{ "qcom,gpio-clk", &dd->spi_gpios[0], DT_OPT, DT_GPIO,
			  -1 },
			{ "qcom,gpio-miso", &dd->spi_gpios[1], DT_OPT, DT_GPIO,
			  -1 },
			{ "qcom,gpio-mosi", &dd->spi_gpios[2], DT_OPT, DT_GPIO,
			  -1 },
			{ "qcom,gpio-cs0", &dd->cs_gpios[0].gpio_num, DT_OPT,
			  DT_GPIO, -1 },
			{ "qcom,gpio-cs1", &dd->cs_gpios[1].gpio_num, DT_OPT,
			  DT_GPIO, -1 },
			{ "qcom,gpio-cs2", &dd->cs_gpios[2].gpio_num, DT_OPT,
			  DT_GPIO, -1 },
			{ "qcom,gpio-cs3", &dd->cs_gpios[3].gpio_num, DT_OPT,
			  DT_GPIO, -1 },
			{ "qcom,rt-priority", &pdata->rt_priority, DT_OPT,
			  DT_BOOL, 0 },
			{ "qcom,shared", &pdata->is_shared, DT_OPT, DT_BOOL,
			  0 },
			{ NULL, NULL, 0, 0, 0 },
		};

		if (msm_spi_dt_to_pdata_populate(pdev, pdata, map)) {
			devm_kfree(&pdev->dev, pdata);
			return NULL;
		}
	}

	if (pdata->use_bam) {
		if (!pdata->bam_consumer_pipe_index) {
			dev_warn(
				&pdev->dev,
				"missing qcom,bam-consumer-pipe-index entry in device-tree\n");
			pdata->use_bam = false;
		}

		if (!pdata->bam_producer_pipe_index) {
			dev_warn(
				&pdev->dev,
				"missing qcom,bam-producer-pipe-index entry in device-tree\n");
			pdata->use_bam = false;
		}
	}
	return pdata;
}

static int msm_spi_get_qup_hw_ver(struct device *dev, struct msm_spi *dd)
{
	u32 data = readl_relaxed(dd->base + QUP_HARDWARE_VER);
	return (data >= QUP_HARDWARE_VER_2_1_1) ? SPI_QUP_VERSION_BFAM :
						  SPI_QUP_VERSION_NONE;
}

static int init_resources(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct msm_spi *dd;
	int rc = -ENXIO;
	int clk_enabled = 0;
	int pclk_enabled = 0;

	dd = spi_master_get_devdata(master);

	if (dd->pdata && dd->pdata->use_pinctrl) {
		rc = msm_spi_pinctrl_init(dd);
		if (rc) {
			dev_err(&pdev->dev, "%s: pinctrl init failed\n",
				__func__);
			return rc;
		}
	}

	mutex_lock(&dd->core_lock);

	dd->clk = clk_get(&pdev->dev, "core_clk");
	if (IS_ERR(dd->clk)) {
		dev_err(&pdev->dev, "%s: unable to get core_clk\n", __func__);
		rc = PTR_ERR(dd->clk);
		goto err_clk_get;
	}

	dd->pclk = clk_get(&pdev->dev, "iface_clk");
	if (IS_ERR(dd->pclk)) {
		dev_err(&pdev->dev, "%s: unable to get iface_clk\n", __func__);
		rc = PTR_ERR(dd->pclk);
		goto err_pclk_get;
	}

	if (dd->pdata && dd->pdata->max_clock_speed)
		msm_spi_clock_set(dd, dd->pdata->max_clock_speed);

	rc = clk_prepare_enable(dd->clk);
	if (rc) {
		dev_err(&pdev->dev, "%s: unable to enable core_clk\n",
			__func__);
		goto err_clk_enable;
	}

	clk_enabled = 1;
	rc = clk_prepare_enable(dd->pclk);
	if (rc) {
		dev_err(&pdev->dev, "%s: unable to enable iface_clk\n",
			__func__);
		goto err_pclk_enable;
	}

	pclk_enabled = 1;

	if (dd->pdata && dd->pdata->ver_reg_exists) {
		enum msm_spi_qup_version ver =
			msm_spi_get_qup_hw_ver(&pdev->dev, dd);
		if (dd->qup_ver != ver)
			dev_warn(
				&pdev->dev,
				"%s: HW version different then initially assumed by probe",
				__func__);
	}

	/* GSBI dose not exists on B-family MSM-chips */
	if (dd->qup_ver != SPI_QUP_VERSION_BFAM) {
		rc = msm_spi_configure_gsbi(dd, pdev);
		if (rc)
			goto err_config_gsbi;
	}

	msm_spi_calculate_fifo_size(dd);
	msm_spi_register_init(dd);
	/*
	 * The SPI core generates a bogus input overrun error on some targets,
	 * when a transition from run to reset state occurs and if the FIFO has
	 * an odd number of entries. Hence we disable the INPUT_OVER_RUN_ERR_EN
	 * bit.
	 */
	msm_spi_enable_error_flags(dd);

	writel_relaxed(SPI_IO_C_NO_TRI_STATE, dd->base + SPI_IO_CONTROL);
	rc = msm_spi_set_state(dd, SPI_OP_STATE_RESET);
	if (rc)
		goto err_spi_state;

	clk_disable_unprepare(dd->clk);
	clk_disable_unprepare(dd->pclk);
	clk_enabled = 0;
	pclk_enabled = 0;

	dd->transfer_pending = 0;
	dd->tx_mode = SPI_MODE_NONE;
	dd->rx_mode = SPI_MODE_NONE;
	dd->last_spi_dev_mode = 0xFFFFFFFF;
	dd->last_bpw = 0;
	dd->last_cs_change = 0xFFFFFFFF;
	dd->op_mask = false;
	dd->spi_ioc_set_cs_set = 0;
	dd->spi_ioc_set_cs_unset = 0;

	mutex_unlock(&dd->core_lock);
	return 0;

err_spi_state:
err_config_gsbi:
	if (pclk_enabled)
		clk_disable_unprepare(dd->pclk);
err_pclk_enable:
	if (clk_enabled)
		clk_disable_unprepare(dd->clk);
err_clk_enable:
	clk_put(dd->pclk);
err_pclk_get:
	clk_put(dd->clk);
err_clk_get:
	mutex_unlock(&dd->core_lock);
	return rc;
}

static int msm_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct msm_spi *dd;
	struct resource *resource;
	int i = 0;
	int rc = -ENXIO;
	struct msm_spi_platform_data *pdata;

	master = spi_alloc_master(&pdev->dev, sizeof(struct msm_spi));
	if (!master) {
		rc = -ENOMEM;
		dev_err(&pdev->dev, "master allocation failed\n");
		goto err_probe_exit;
	}

	master->bus_num = pdev->id;
	master->mode_bits = SPI_SUPPORTED_MODES;
	master->num_chipselect = SPI_NUM_CHIPSELECTS;
	master->set_cs = msm_spi_set_cs;
	master->setup = msm_spi_setup;
	master->prepare_transfer_hardware = msm_spi_prepare_transfer_hardware;
	master->transfer_one = msm_spi_transfer_one;
	master->unprepare_transfer_hardware =
		msm_spi_unprepare_transfer_hardware;

	platform_set_drvdata(pdev, master);
	dd = spi_master_get_devdata(master);

	if (pdev->dev.of_node) {
		dd->qup_ver = SPI_QUP_VERSION_BFAM;
		master->dev.of_node = pdev->dev.of_node;
		pdata = msm_spi_dt_to_pdata(pdev, dd);
		if (!pdata) {
			rc = -ENOMEM;
			goto err_probe_exit;
		}

		rc = of_alias_get_id(pdev->dev.of_node, "spi");
		if (rc < 0)
			dev_warn(&pdev->dev, "using default bus_num %d\n",
				 pdev->id);
		else
			master->bus_num = pdev->id = rc;
	} else {
		pdata = pdev->dev.platform_data;
		dd->qup_ver = SPI_QUP_VERSION_NONE;

		for (i = 0; i < ARRAY_SIZE(spi_rsrcs); ++i) {
			resource =
				platform_get_resource(pdev, IORESOURCE_IO, i);
			dd->spi_gpios[i] = resource ? resource->start : -1;
		}

		for (i = 0; i < ARRAY_SIZE(spi_cs_rsrcs); ++i) {
			resource = platform_get_resource(
				pdev, IORESOURCE_IO, i + ARRAY_SIZE(spi_rsrcs));
			dd->cs_gpios[i].gpio_num = resource ? resource->start :
							      -1;
		}
	}

	for (i = 0; i < ARRAY_SIZE(spi_cs_rsrcs); ++i)
		dd->cs_gpios[i].valid = 0;

	dd->pdata = pdata;
	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource) {
		rc = -ENXIO;
		goto err_probe_res;
	}

	dd->mem_phys_addr = resource->start;
	dd->mem_size = resource_size(resource);
	dd->dev = &pdev->dev;

	spin_lock_init(&dd->queue_lock);
	mutex_init(&dd->core_lock);
	init_waitqueue_head(&dd->continue_suspend);

	if (!devm_request_mem_region(&pdev->dev, dd->mem_phys_addr,
				     dd->mem_size, SPI_DRV_NAME)) {
		rc = -ENXIO;
		goto err_probe_reqmem;
	}

	dd->base = devm_ioremap(&pdev->dev, dd->mem_phys_addr, dd->mem_size);
	if (!dd->base) {
		rc = -ENOMEM;
		goto err_probe_reqmem;
	}

	pm_runtime_set_autosuspend_delay(&pdev->dev, MSEC_PER_SEC);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	dd->suspended = 1;
	rc = spi_register_master(master);
	if (rc)
		goto err_probe_reg_master;

	rc = sysfs_create_group(&(dd->dev->kobj), &dev_attr_grp);
	if (rc) {
		dev_err(&pdev->dev, "failed to create dev. attrs : %d\n", rc);
		goto err_attrs;
	}
	spi_debugfs_init(dd);

	return 0;

err_attrs:
	spi_unregister_master(master);
err_probe_reg_master:
	pm_runtime_disable(&pdev->dev);
err_probe_reqmem:
err_probe_res:
	spi_master_put(master);
err_probe_exit:
	return rc;
}

static int msm_spi_pm_suspend_runtime(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct spi_master *master = platform_get_drvdata(pdev);
	struct msm_spi *dd;
	unsigned long flags;

	dev_dbg(device, "pm_runtime: suspending...\n");
	if (!master)
		goto suspend_exit;
	dd = spi_master_get_devdata(master);
	if (!dd)
		goto suspend_exit;

	if (dd->suspended)
		return 0;

	/*
	 * Make sure nothing is added to the queue while we're
	 * suspending
	 */
	spin_lock_irqsave(&dd->queue_lock, flags);
	dd->suspended = 1;
	spin_unlock_irqrestore(&dd->queue_lock, flags);

	/* Wait for transactions to end, or time out */
	wait_event_interruptible(dd->continue_suspend, !dd->transfer_pending);

	if (dd->pdata && !dd->pdata->is_shared)
		put_local_resources(dd);

	if (dd->pdata)
		msm_spi_clk_path_vote(dd, 0);

suspend_exit:
	return 0;
}

static int msm_spi_pm_resume_runtime(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct spi_master *master = platform_get_drvdata(pdev);
	struct msm_spi *dd;
	int ret = 0;

	dev_dbg(device, "pm_runtime: resuming...\n");
	if (!master)
		goto resume_exit;
	dd = spi_master_get_devdata(master);
	if (!dd)
		goto resume_exit;

	if (!dd->suspended)
		return 0;
	if (!dd->is_init_complete) {
		ret = init_resources(pdev);
		if (ret != 0)
			return ret;
		else
			dd->is_init_complete = true;
	}
	msm_spi_clk_path_init(dd);
	msm_spi_clk_path_vote(dd, dd->pdata->max_clock_speed);

	if (!dd->pdata->is_shared) {
		ret = get_local_resources(dd);
		if (ret)
			return ret;
	}
	dd->suspended = 0;

resume_exit:
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int msm_spi_suspend(struct device *device)
{
	if (!pm_runtime_status_suspended(device)) {
		dev_err(device, "Runtime not suspended, deny sys suspend");
		return -EBUSY;
	}
	return 0;
}

static int msm_spi_resume(struct device *device)
{
	/*
	 * Rely on runtime-PM to call resume in case it is enabled
	 * Even if it's not enabled, rely on 1st client transaction to do
	 * clock ON and gpio configuration
	 */
	dev_dbg(device, "system resume");
	return 0;
}
#else
#define msm_spi_suspend NULL
#define msm_spi_resume NULL
#endif

static int msm_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct msm_spi *dd = spi_master_get_devdata(master);

	spi_debugfs_exit(dd);
	sysfs_remove_group(&pdev->dev.kobj, &dev_attr_grp);

	pm_runtime_put_sync(dd->dev);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	clk_put(dd->clk);
	clk_put(dd->pclk);
	msm_spi_clk_path_teardown(dd);
	platform_set_drvdata(pdev, 0);
	spi_unregister_master(master);
	spi_master_put(master);

	return 0;
}

static struct of_device_id msm_spi_dt_match[] = {
	{
		.compatible = "qorvo,spi-qup-v2-lowlatency",
	},
	{}
};

static const struct dev_pm_ops msm_spi_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(msm_spi_suspend, msm_spi_resume)
		SET_RUNTIME_PM_OPS(msm_spi_pm_suspend_runtime,
				   msm_spi_pm_resume_runtime, NULL)
};

static struct platform_driver msm_spi_driver = {
	.driver		= {
		.name	= SPI_DRV_NAME,
		.owner	= THIS_MODULE,
		.pm		= &msm_spi_dev_pm_ops,
		.of_match_table = msm_spi_dt_match,
	},
	.probe		= msm_spi_probe,
	.remove		= msm_spi_remove,
	.probe		= msm_spi_probe,
};

module_platform_driver(msm_spi_driver);

MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.4");
MODULE_ALIAS("platform:" SPI_DRV_NAME);
