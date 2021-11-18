// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2017-2018, The Linux foundation. All rights reserved.

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/qcom-geni-se.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>
#include <linux/gpio/consumer.h>
#include <linux/sched.h>
#include <uapi/linux/sched/types.h>
#include <linux/delay.h>

/* About the low latency mode:
 * The Linux kernel is not real-time and a 4 bytes SPI transfer can take
 * randomly between 150탎 and more than 2.5ms on a DrangonBoard 845c.
 * This is due to the context changes of the interrupt mode of the driver:
 * - One interrupt to enable CS.
 * - One interrupt for the first TX FIFO Watermark (TX FIFO empty).
 * - One interrupt for the RX bytes.
 * - One interrupt to disable CS.
 * - Two context changes between the caller and the SPI controller thread.
 * With the "Low latency mode", the same transfer takes 25탎 in an almost
 * constant manner.
 * But to achieve this low latency, the following changes were made:
 * - Interrupts of the SPI controller are completely disabled.
 * - Transfers are made in polling mode, active wait, in the function
 * setup_fifo_xfer().
 * - The CS signal is controlled directly with a GPIO to avoid CS interrupts.
 * - The SPI mode (CPOL, CPHA), the bit per word are are configured
 * only once at the first transfer in the life of the SPI controller driver.
 * - The SPI clock speed is nevertheless checked at each transfer.
 *
 * The device tree should be changed as follows:
 * - In the spi2 node, add this to reserve the CS GPIO:
 * cs-gpio = <&tlmm 30 GPIO_ACTIVE_LOW>;
 * Note: It is not necessary to disable this GPIO in the spi2 qup_spi2_default
 * pinmux node.
 */

static bool low_latency = true;
module_param(low_latency, bool, 0440);
MODULE_PARM_DESC(low_latency, "Low latency mode");

/* SPI SE specific registers and respective register fields */
#define SE_SPI_CPHA 0x224
#define CPHA BIT(0)

#define SE_SPI_LOOPBACK 0x22c
#define LOOPBACK_ENABLE 0x1
#define NORMAL_MODE 0x0
#define LOOPBACK_MSK GENMASK(1, 0)

#define SE_SPI_CPOL 0x230
#define CPOL BIT(2)

#define SE_SPI_DEMUX_OUTPUT_INV 0x24c
#define CS_DEMUX_OUTPUT_INV_MSK GENMASK(3, 0)

#define SE_SPI_DEMUX_SEL 0x250
#define CS_DEMUX_OUTPUT_SEL GENMASK(3, 0)

#define SE_SPI_TRANS_CFG 0x25c
#define CS_TOGGLE BIT(0)

#define SE_SPI_WORD_LEN 0x268
#define WORD_LEN_MSK GENMASK(9, 0)
#define MIN_WORD_LEN 4

#define SE_SPI_TX_TRANS_LEN 0x26c
#define SE_SPI_RX_TRANS_LEN 0x270
#define TRANS_LEN_MSK GENMASK(23, 0)

#define SE_SPI_PRE_POST_CMD_DLY 0x274

#define SE_SPI_DELAY_COUNTERS 0x278
#define SPI_INTER_WORDS_DELAY_MSK GENMASK(9, 0)
#define SPI_CS_CLK_DELAY_MSK GENMASK(19, 10)
#define SPI_CS_CLK_DELAY_SHFT 10

/* M_CMD OP codes for SPI */
#define SPI_TX_ONLY 1
#define SPI_RX_ONLY 2
#define SPI_FULL_DUPLEX 3
#define SPI_TX_RX 7
#define SPI_CS_ASSERT 8
#define SPI_CS_DEASSERT 9
#define SPI_SCK_ONLY 10
/* M_CMD params for SPI */
#define SPI_PRE_CMD_DELAY BIT(0)
#define TIMESTAMP_BEFORE BIT(1)
#define FRAGMENTATION BIT(2)
#define TIMESTAMP_AFTER BIT(3)
#define POST_CMD_DELAY BIT(4)

enum spi_m_cmd_opcode {
	CMD_NONE,
	CMD_XFER,
	CMD_CS,
	CMD_CANCEL,
};

struct spi_geni_master {
	struct geni_se se;
	struct device *dev;
	u32 tx_fifo_depth;
	u32 fifo_width_bits;
	u32 tx_wm;
	unsigned long cur_speed_hz;
	unsigned long cur_real_speed_hz;
	unsigned int cur_bits_per_word;
	unsigned int tx_rem_bytes;
	unsigned int rx_rem_bytes;
	const struct spi_transfer *cur_xfer;
	struct completion xfer_done;
	unsigned int oversampling;
	spinlock_t lock;
	enum spi_m_cmd_opcode cur_mcmd;
	int irq;
	bool first_transfert;
	unsigned int tx_fifo_watermark_delay_us;
	struct gpio_desc *gpio_cs;
	unsigned int max_fifo_bytes;
};

static void geni_spi_handle_tx(struct spi_geni_master *mas);
static void geni_spi_handle_rx(struct spi_geni_master *mas);

static void get_tx_fifo_watermark_delay_us(struct spi_geni_master *mas)
{
	/* - Compute the new tx_fifo_watermark_delay_us.
	 * Due to the lack of documentation on the GENI Serial Engine,
	 * the delay values come from empirical tests that have been validated
	 * on a Snapdragon SDM845:
	 * - SPI speed 3MHz: delay of 150us
	 * - SPI speed 20MHz: delay of 20us
	 * - SPI speed 33.333333MHz: delay of 12us
	 * Please see the comment "Wait for reload the TX FIFO" in setup_fifo_xfer()
	 * below for the use of this delay.
	 */
	switch (mas->cur_real_speed_hz) {
	case 3000000:
		mas->tx_fifo_watermark_delay_us = 150;
		break;
	case 20000000:
		mas->tx_fifo_watermark_delay_us = 20;
		break;
	case 33333333:
		mas->tx_fifo_watermark_delay_us = 12;
		break;
	default:
		mas->tx_fifo_watermark_delay_us = 150;
		break;
	}
	dev_dbg(mas->dev,
		"TX FIFO watermark delay: %u us for %lu Hz transfer.\n",
		mas->tx_fifo_watermark_delay_us, mas->cur_real_speed_hz);
}

static int get_spi_clk_cfg(unsigned int speed_hz, struct spi_geni_master *mas,
			   unsigned int *clk_idx, unsigned int *clk_div)
{
	unsigned long sclk_freq;
	unsigned int actual_hz;
	struct geni_se *se = &mas->se;
	int ret;

	ret = geni_se_clk_freq_match(&mas->se, speed_hz * mas->oversampling,
				     clk_idx, &sclk_freq, false);
	if (ret) {
		dev_err(mas->dev, "Failed(%d) to find src clk for %dHz\n", ret,
			speed_hz);
		return ret;
	}

	*clk_div = DIV_ROUND_UP(sclk_freq, mas->oversampling * speed_hz);
	actual_hz = sclk_freq / (mas->oversampling * *clk_div);

	/* Store the real effective clock speed. It is needed for the
	 * tx_fifo_watermark_delay_us calculation.
	 */
	if (likely(low_latency)) {
		mas->cur_real_speed_hz = actual_hz;
	}
	dev_dbg(mas->dev, "req %u=>%u sclk %lu, idx %d, div %d\n", speed_hz,
		actual_hz, sclk_freq, *clk_idx, *clk_div);
	ret = clk_set_rate(se->clk, sclk_freq);
	if (ret)
		dev_err(mas->dev, "clk_set_rate failed %d\n", ret);
	return ret;
}

static void handle_fifo_timeout(struct spi_master *spi, struct spi_message *msg)
{
	struct spi_geni_master *mas = spi_master_get_devdata(spi);
	unsigned long time_left, flags;
	struct geni_se *se = &mas->se;

	spin_lock_irqsave(&mas->lock, flags);
	reinit_completion(&mas->xfer_done);
	mas->cur_mcmd = CMD_CANCEL;
	geni_se_cancel_m_cmd(se);
	writel(0, se->base + SE_GENI_TX_WATERMARK_REG);
	spin_unlock_irqrestore(&mas->lock, flags);
	time_left = wait_for_completion_timeout(&mas->xfer_done, HZ);
	if (time_left)
		return;

	spin_lock_irqsave(&mas->lock, flags);
	reinit_completion(&mas->xfer_done);
	geni_se_abort_m_cmd(se);
	spin_unlock_irqrestore(&mas->lock, flags);
	time_left = wait_for_completion_timeout(&mas->xfer_done, HZ);
	if (!time_left)
		dev_err(mas->dev, "Failed to cancel/abort m_cmd\n");
}

static void spi_geni_set_cs(struct spi_device *slv, bool set_flag)
{
	struct spi_geni_master *mas = spi_master_get_devdata(slv->master);
	struct spi_master *spi = dev_get_drvdata(mas->dev);
	struct geni_se *se = &mas->se;
	unsigned long time_left;

	pm_runtime_get_sync(mas->dev);
	if (!(slv->mode & SPI_CS_HIGH))
		set_flag = !set_flag;
	if (likely(low_latency)) {
		/* Toggle the CS signal directly to avoid interrupt latency */
		gpiod_set_value(mas->gpio_cs, set_flag);
	} else {
		reinit_completion(&mas->xfer_done);
		mas->cur_mcmd = CMD_CS;
		if (set_flag)
			geni_se_setup_m_cmd(se, SPI_CS_ASSERT, 0);
		else
			geni_se_setup_m_cmd(se, SPI_CS_DEASSERT, 0);

		time_left = wait_for_completion_timeout(&mas->xfer_done, HZ);
		if (!time_left)
			handle_fifo_timeout(spi, NULL);
	}
	pm_runtime_put(mas->dev);
}

static void spi_setup_word_len(struct spi_geni_master *mas, u16 mode,
			       unsigned int bits_per_word)
{
	unsigned int pack_words;
	bool msb_first = (mode & SPI_LSB_FIRST) ? false : true;
	struct geni_se *se = &mas->se;
	u32 word_len;

	word_len = readl(se->base + SE_SPI_WORD_LEN);

	/*
	 * If bits_per_word isn't a byte aligned value, set the packing to be
	 * 1 SPI word per FIFO word.
	 */
	if (!(mas->fifo_width_bits % bits_per_word))
		pack_words = mas->fifo_width_bits / bits_per_word;
	else
		pack_words = 1;
	word_len &= ~WORD_LEN_MSK;
	word_len |= ((bits_per_word - MIN_WORD_LEN) & WORD_LEN_MSK);
	geni_se_config_packing(&mas->se, bits_per_word, pack_words, msb_first,
			       true, true);
	writel(word_len, se->base + SE_SPI_WORD_LEN);
}

static int setup_fifo_params(struct spi_device *spi_slv, struct spi_master *spi)
{
	struct spi_geni_master *mas = spi_master_get_devdata(spi);
	struct geni_se *se = &mas->se;
	u32 loopback_cfg, cpol, cpha, demux_output_inv;
	u32 demux_sel, clk_sel, m_clk_cfg, idx, div;
	int ret;

	loopback_cfg = readl(se->base + SE_SPI_LOOPBACK);
	cpol = readl(se->base + SE_SPI_CPOL);
	cpha = readl(se->base + SE_SPI_CPHA);
	demux_output_inv = 0;
	loopback_cfg &= ~LOOPBACK_MSK;
	cpol &= ~CPOL;
	cpha &= ~CPHA;

	if (spi_slv->mode & SPI_LOOP)
		loopback_cfg |= LOOPBACK_ENABLE;

	if (spi_slv->mode & SPI_CPOL)
		cpol |= CPOL;

	if (spi_slv->mode & SPI_CPHA)
		cpha |= CPHA;

	if (spi_slv->mode & SPI_CS_HIGH)
		demux_output_inv = BIT(spi_slv->chip_select);

	demux_sel = spi_slv->chip_select;
	mas->cur_speed_hz = spi_slv->max_speed_hz;
	mas->cur_bits_per_word = spi_slv->bits_per_word;

	ret = get_spi_clk_cfg(mas->cur_speed_hz, mas, &idx, &div);
	if (ret) {
		dev_err(mas->dev, "Err setting clks ret(%d) for %ld\n", ret,
			mas->cur_speed_hz);
		return ret;
	}

	clk_sel = idx & CLK_SEL_MSK;
	m_clk_cfg = (div << CLK_DIV_SHFT) | SER_CLK_EN;
	spi_setup_word_len(mas, spi_slv->mode, spi_slv->bits_per_word);
	writel(loopback_cfg, se->base + SE_SPI_LOOPBACK);
	writel(demux_sel, se->base + SE_SPI_DEMUX_SEL);
	writel(cpha, se->base + SE_SPI_CPHA);
	writel(cpol, se->base + SE_SPI_CPOL);
	writel(demux_output_inv, se->base + SE_SPI_DEMUX_OUTPUT_INV);
	writel(clk_sel, se->base + SE_GENI_CLK_SEL);
	writel(m_clk_cfg, se->base + GENI_SER_M_CLK_CFG);
	/* In low latancy mode, compute the mandatory tx_fifo_watermark_delay_us. */
	if (likely(low_latency)) {
		get_tx_fifo_watermark_delay_us(mas);
	}
	return 0;
}

static int spi_geni_prepare_message(struct spi_master *spi,
				    struct spi_message *spi_msg)
{
	int ret = 0;
	struct spi_geni_master *mas = spi_master_get_devdata(spi);
	/* This function accesses the hardware and takes about 50탎.
	 * In low_latency mode, the hardware setup is made only at the first transfer.
	 */
	if (unlikely(mas->first_transfert || !low_latency)) {
		struct geni_se *se = &mas->se;

		geni_se_select_mode(se, GENI_SE_FIFO);
		ret = setup_fifo_params(spi_msg->spi, spi);
		if (ret)
			dev_err(mas->dev, "Couldn't select mode %d\n", ret);
	}
	return ret;
}

static int spi_geni_init(struct spi_geni_master *mas)
{
	struct geni_se *se = &mas->se;
	unsigned int proto, major, minor, ver;

	pm_runtime_get_sync(mas->dev);

	proto = geni_se_read_proto(se);
	if (proto != GENI_SE_SPI) {
		dev_err(mas->dev, "Invalid proto %d\n", proto);
		pm_runtime_put(mas->dev);
		return -ENXIO;
	}
	mas->tx_fifo_depth = geni_se_get_tx_fifo_depth(se);

	/* Width of Tx and Rx FIFO is same */
	mas->fifo_width_bits = geni_se_get_tx_fifo_width(se);

	/*
	 * Hardware programming guide suggests to configure
	 * RX FIFO RFR level to fifo_depth-2.
	 */
	geni_se_init(se, 0x0, mas->tx_fifo_depth - 2);
	/* Transmit an entire FIFO worth of data per IRQ */
	mas->tx_wm = 1;
	ver = geni_se_get_qup_hw_version(se);
	major = GENI_SE_VERSION_MAJOR(ver);
	minor = GENI_SE_VERSION_MINOR(ver);

	if (major == 1 && minor == 0)
		mas->oversampling = 2;
	else
		mas->oversampling = 1;

	pm_runtime_put(mas->dev);
	return 0;
}

static inline u32 geni_spi_wait_irq_status(struct spi_geni_master *mas,
					   u32 mask, unsigned int tx_remain)
{
	void __iomem *addr = mas->se.base + SE_GENI_M_IRQ_STATUS;
	u32 m_irq;
	/* Active wait for selected bits in IRQ status register.
	 *
	 * If waiting for reload the TX FIFO, the active udelay() wait is
	 * mandatory. Don't ask why, the datasheet of the the GENI Serial
	 * Engine is not publicly available.
	 *
	 * But, if SE_GENI_M_IRQ_STATUS is accessed in loop without delay,
	 * the last M_CMD_DONE_EN interrupt flag will never happen and this
	 * function will remain stuck endlessly while waiting for this flag.
	 *
	 * This delay is not necessary for the last reload, don't ask for it
	 * either!
	 */
	do {
		if (tx_remain)
			udelay(mas->tx_fifo_watermark_delay_us);
		m_irq = readl(addr);
	} while (!(m_irq & mask));
	return m_irq;
}

static void setup_fifo_xfer(struct spi_transfer *xfer,
			    struct spi_geni_master *mas, u16 mode,
			    struct spi_master *spi)
{
	u32 m_cmd = 0;
	u32 spi_tx_cfg, len;
	struct geni_se *se = &mas->se;
	u32 m_irq;

	/* These calls accesses the hardware and takes about 20탎.
	 * In low_latency mode, the hardware setup is made only at the first transfer.
	 */
	if (unlikely(mas->first_transfert || !low_latency)) {
		spi_tx_cfg = readl(se->base + SE_SPI_TRANS_CFG);
		/* Bits per word can be overridden per transfer */
		if (xfer->bits_per_word != mas->cur_bits_per_word) {
			spi_setup_word_len(mas, mode, xfer->bits_per_word);
			mas->cur_bits_per_word = xfer->bits_per_word;
		}
		spi_tx_cfg &= ~CS_TOGGLE;
		writel(spi_tx_cfg, se->base + SE_SPI_TRANS_CFG);
		mas->first_transfert = false;
	}
	/* Speed can be overridden per transfer */
	if (unlikely(xfer->speed_hz != mas->cur_speed_hz)) {
		int ret;
		u32 clk_sel, m_clk_cfg;
		unsigned int idx, div;

		ret = get_spi_clk_cfg(xfer->speed_hz, mas, &idx, &div);
		if (ret) {
			dev_err(mas->dev, "Err setting clks:%d\n", ret);
			return;
		}
		/*
		 * SPI core clock gets configured with the requested frequency
		 * or the frequency closer to the requested frequency.
		 * For that reason requested frequency is stored in the
		 * cur_speed_hz and referred in the consecutive transfer instead
		 * of calling clk_get_rate() API.
		 */
		mas->cur_speed_hz = xfer->speed_hz;
		clk_sel = idx & CLK_SEL_MSK;
		m_clk_cfg = (div << CLK_DIV_SHFT) | SER_CLK_EN;
		writel(clk_sel, se->base + SE_GENI_CLK_SEL);
		writel(m_clk_cfg, se->base + GENI_SER_M_CLK_CFG);
		/* Update tx_fifo_watermark_delay_us according to the new clock speed. */
		if (likely(low_latency)) {
			get_tx_fifo_watermark_delay_us(mas);
		}
	}

	mas->tx_rem_bytes = 0;
	mas->rx_rem_bytes = 0;
	if (xfer->tx_buf && xfer->rx_buf)
		m_cmd = SPI_FULL_DUPLEX;
	else if (xfer->tx_buf)
		m_cmd = SPI_TX_ONLY;
	else if (xfer->rx_buf)
		m_cmd = SPI_RX_ONLY;

	if (!(mas->cur_bits_per_word % MIN_WORD_LEN))
		len = xfer->len * BITS_PER_BYTE / mas->cur_bits_per_word;
	else
		len = xfer->len / (mas->cur_bits_per_word / BITS_PER_BYTE + 1);
	len &= TRANS_LEN_MSK;

	mas->cur_xfer = xfer;
	if (m_cmd & SPI_TX_ONLY) {
		mas->tx_rem_bytes = xfer->len;
		writel(len, se->base + SE_SPI_TX_TRANS_LEN);
	}

	if (m_cmd & SPI_RX_ONLY) {
		writel(len, se->base + SE_SPI_RX_TRANS_LEN);
		mas->rx_rem_bytes = xfer->len;
	}
	mas->cur_mcmd = CMD_XFER;
	geni_se_setup_m_cmd(se, m_cmd, FRAGMENTATION);

	if (likely(low_latency)) {
		if ((m_cmd & SPI_TX_ONLY) && (len > mas->max_fifo_bytes)) {
			/* TX transfer and the size of the data are greater of the TX FIFO,
			 * Enable the M_TX_FIFO_WATERMARK_EN interrupt flag. This is mandatory
			 * when the TX FIFO is reloaded several times.
			 */
			writel(mas->tx_wm, se->base + SE_GENI_TX_WATERMARK_REG);
		}
		do {
			m_irq = 0;
			/* If TX transfer, push the data in the TX FIFO */
			if (m_cmd & SPI_TX_ONLY) {
				const u32 mask = M_TX_FIFO_WATERMARK_EN;
				/* Load or reload the TX FIFO */
				geni_spi_handle_tx(mas);
				/* If TX only transfer and remaining data, active wait the end of the sending
				 * of the current data in the TX FIFO, aka the M_TX_FIFO_WATERMARK_EN flag.
				 * If TX / RX transfer or no remaining data, just wait for the RX data.
				 */
				if (mas->tx_rem_bytes && !(m_cmd & SPI_RX_ONLY))
					m_irq = geni_spi_wait_irq_status(
						mas, mask, mas->tx_rem_bytes);
			}
			/* If RX transfer, active wait and get the data in the RX FIFO */
			if (m_cmd & SPI_RX_ONLY) {
				const u32 mask = M_RX_FIFO_WATERMARK_EN |
						 M_RX_FIFO_LAST_EN;
				if (!(m_irq & mask))
					m_irq = geni_spi_wait_irq_status(
						mas, mask, 0);
				/* Get the data from the RX FIFO */
				geni_spi_handle_rx(mas);
			}
		} while (mas->tx_rem_bytes || mas->rx_rem_bytes);
		/* Here, all the data are sent and/or received. */
		/* Active wait of the end of the SPI transfer. */
		if (!(m_irq & M_CMD_DONE_EN))
			m_irq = geni_spi_wait_irq_status(mas, M_CMD_DONE_EN, 0);
		/* RAZ the IRQ flags status and the current command */
		mas->cur_mcmd = CMD_NONE;
		writel(m_irq, se->base + SE_GENI_M_IRQ_CLEAR);
	} else {
		/*
		 * TX_WATERMARK_REG should be set after SPI configuration and
		 * setting up GENI SE engine, as driver starts data transfer
		 * for the watermark interrupt.
		 */
		if (m_cmd & SPI_TX_ONLY)
			writel(mas->tx_wm, se->base + SE_GENI_TX_WATERMARK_REG);
	}
}

static int spi_geni_transfer_one(struct spi_master *spi, struct spi_device *slv,
				 struct spi_transfer *xfer)
{
	struct spi_geni_master *mas = spi_master_get_devdata(spi);

	/* Terminate and return success for 0 byte length transfer */
	if (!xfer->len)
		return 0;

	setup_fifo_xfer(xfer, mas, slv->mode, spi);
	return !low_latency;
}

static unsigned int geni_byte_per_fifo_word(struct spi_geni_master *mas)
{
	/*
	 * Calculate how many bytes we'll put in each FIFO word.  If the
	 * transfer words don't pack cleanly into a FIFO word we'll just put
	 * one transfer word in each FIFO word.  If they do pack we'll pack 'em.
	 */
	if (mas->fifo_width_bits % mas->cur_bits_per_word)
		return roundup_pow_of_two(
			DIV_ROUND_UP(mas->cur_bits_per_word, BITS_PER_BYTE));

	return mas->fifo_width_bits / BITS_PER_BYTE;
}

static void geni_spi_handle_tx(struct spi_geni_master *mas)
{
	struct geni_se *se = &mas->se;
	unsigned int max_bytes;
	const u8 *tx_buf;
	unsigned int bytes_per_fifo_word = geni_byte_per_fifo_word(mas);
	unsigned int i = 0;

	max_bytes = (mas->tx_fifo_depth - mas->tx_wm) * bytes_per_fifo_word;
	if (mas->tx_rem_bytes < max_bytes)
		max_bytes = mas->tx_rem_bytes;

	tx_buf = mas->cur_xfer->tx_buf + mas->cur_xfer->len - mas->tx_rem_bytes;
	while (i < max_bytes) {
		unsigned int j;
		unsigned int bytes_to_write;
		u32 fifo_word = 0;
		u8 *fifo_byte = (u8 *)&fifo_word;

		bytes_to_write = min(bytes_per_fifo_word, max_bytes - i);
		for (j = 0; j < bytes_to_write; j++)
			fifo_byte[j] = tx_buf[i++];
		iowrite32_rep(se->base + SE_GENI_TX_FIFOn, &fifo_word, 1);
	}
	mas->tx_rem_bytes -= max_bytes;
	if (!mas->tx_rem_bytes)
		writel(0, se->base + SE_GENI_TX_WATERMARK_REG);
}

static void geni_spi_handle_rx(struct spi_geni_master *mas)
{
	struct geni_se *se = &mas->se;
	u32 rx_fifo_status;
	unsigned int rx_bytes;
	unsigned int rx_last_byte_valid;
	u8 *rx_buf;
	unsigned int bytes_per_fifo_word = geni_byte_per_fifo_word(mas);
	unsigned int i = 0;

	rx_fifo_status = readl(se->base + SE_GENI_RX_FIFO_STATUS);
	rx_bytes = (rx_fifo_status & RX_FIFO_WC_MSK) * bytes_per_fifo_word;
	if (rx_fifo_status & RX_LAST) {
		rx_last_byte_valid = rx_fifo_status & RX_LAST_BYTE_VALID_MSK;
		rx_last_byte_valid >>= RX_LAST_BYTE_VALID_SHFT;
		if (rx_last_byte_valid && rx_last_byte_valid < 4)
			rx_bytes -= bytes_per_fifo_word - rx_last_byte_valid;
	}
	if (mas->rx_rem_bytes < rx_bytes)
		rx_bytes = mas->rx_rem_bytes;

	rx_buf = mas->cur_xfer->rx_buf + mas->cur_xfer->len - mas->rx_rem_bytes;
	while (i < rx_bytes) {
		u32 fifo_word = 0;
		u8 *fifo_byte = (u8 *)&fifo_word;
		unsigned int bytes_to_read;
		unsigned int j;

		bytes_to_read = min(bytes_per_fifo_word, rx_bytes - i);
		ioread32_rep(se->base + SE_GENI_RX_FIFOn, &fifo_word, 1);
		for (j = 0; j < bytes_to_read; j++)
			rx_buf[i++] = fifo_byte[j];
	}
	mas->rx_rem_bytes -= rx_bytes;
}

static irqreturn_t geni_spi_isr(int irq, void *data)
{
	struct spi_master *spi = data;
	struct spi_geni_master *mas = spi_master_get_devdata(spi);
	struct geni_se *se = &mas->se;
	u32 m_irq;
	unsigned long flags;

	if (mas->cur_mcmd == CMD_NONE)
		return IRQ_NONE;

	spin_lock_irqsave(&mas->lock, flags);
	m_irq = readl(se->base + SE_GENI_M_IRQ_STATUS);

	if ((m_irq & M_RX_FIFO_WATERMARK_EN) || (m_irq & M_RX_FIFO_LAST_EN))
		geni_spi_handle_rx(mas);

	if (m_irq & M_TX_FIFO_WATERMARK_EN)
		geni_spi_handle_tx(mas);

	if (m_irq & M_CMD_DONE_EN) {
		if (mas->cur_mcmd == CMD_XFER)
			spi_finalize_current_transfer(spi);
		else if (mas->cur_mcmd == CMD_CS)
			complete(&mas->xfer_done);
		mas->cur_mcmd = CMD_NONE;
		/*
		 * If this happens, then a CMD_DONE came before all the Tx
		 * buffer bytes were sent out. This is unusual, log this
		 * condition and disable the WM interrupt to prevent the
		 * system from stalling due an interrupt storm.
		 * If this happens when all Rx bytes haven't been received, log
		 * the condition.
		 * The only known time this can happen is if bits_per_word != 8
		 * and some registers that expect xfer lengths in num spi_words
		 * weren't written correctly.
		 */
		if (mas->tx_rem_bytes) {
			writel(0, se->base + SE_GENI_TX_WATERMARK_REG);
			dev_err(mas->dev, "Premature done. tx_rem = %d bpw%d\n",
				mas->tx_rem_bytes, mas->cur_bits_per_word);
		}
		if (mas->rx_rem_bytes)
			dev_err(mas->dev, "Premature done. rx_rem = %d bpw%d\n",
				mas->rx_rem_bytes, mas->cur_bits_per_word);
	}

	if ((m_irq & M_CMD_CANCEL_EN) || (m_irq & M_CMD_ABORT_EN)) {
		mas->cur_mcmd = CMD_NONE;
		complete(&mas->xfer_done);
	}

	writel(m_irq, se->base + SE_GENI_M_IRQ_CLEAR);
	spin_unlock_irqrestore(&mas->lock, flags);
	return IRQ_HANDLED;
}

static int spi_geni_probe(struct platform_device *pdev)
{
	int ret, irq;
	struct spi_master *spi;
	struct spi_geni_master *mas;
	void __iomem *base;
	struct clk *clk;
	struct device *dev = &pdev->dev;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	clk = devm_clk_get(dev, "se");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	spi = spi_alloc_master(dev, sizeof(*mas));
	if (!spi)
		return -ENOMEM;

	platform_set_drvdata(pdev, spi);
	mas = spi_master_get_devdata(spi);
	mas->irq = irq;
	mas->dev = dev;
	mas->se.dev = dev;
	mas->se.wrapper = dev_get_drvdata(dev->parent);
	mas->se.base = base;
	mas->se.clk = clk;
	mas->first_transfert = true;
	spi->bus_num = -1;
	spi->dev.of_node = dev->of_node;
	spi->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LOOP | SPI_CS_HIGH;
	spi->bits_per_word_mask = SPI_BPW_RANGE_MASK(4, 32);
	spi->num_chipselect = 4;
	spi->max_speed_hz = 50000000;
	spi->prepare_message = spi_geni_prepare_message;
	spi->transfer_one = spi_geni_transfer_one;
	spi->auto_runtime_pm = true;
	spi->handle_err = handle_fifo_timeout;
	spi->set_cs = spi_geni_set_cs;

	init_completion(&mas->xfer_done);
	spin_lock_init(&mas->lock);
	pm_runtime_enable(dev);

	ret = spi_geni_init(mas);
	if (ret)
		goto spi_geni_probe_runtime_disable;
	/* In low latency mode, active polling used instead of irq */
	if (unlikely(!low_latency)) {
		ret = request_irq(mas->irq, geni_spi_isr, 0, dev_name(dev),
				  spi);
		if (ret)
			goto spi_geni_probe_runtime_disable;
	}

	if (likely(low_latency)) {
		/* Set the max bytes in the FIFO.
		 * This calculation comes from geni_spi_handle_tx() with the right
		 * bytes per fifo word. Calling geni_byte_per_fifo_word() returns a erroneous value.
		 * So we use mas->fifo_width_bits instead.
		 */
		mas->max_fifo_bytes = ((mas->tx_fifo_depth - mas->tx_wm) *
				       mas->fifo_width_bits) /
				      BITS_PER_BYTE;
	}

	/* Reserve the GPIO for the CS signal */
	if (likely(low_latency)) {
		mas->gpio_cs = devm_gpiod_get(dev, "cs", GPIOD_OUT_HIGH);
		if (IS_ERR(mas->gpio_cs)) {
			dev_err(mas->dev,
				"devm_gpiod_get for the CS GPIO error\n");
			goto spi_geni_probe_free_irq;
		}
	}
	if (likely(low_latency)) {
		dev_warn(mas->dev, "Low latency mode enabled.\n");
		dev_dbg(mas->dev, "FIFO size: %d bytes\n", mas->max_fifo_bytes);
	}

	/* Register the contoller when all is done */
	ret = spi_register_master(spi);
	if (ret)
		goto spi_geni_probe_free_irq;
	return 0;

spi_geni_probe_free_irq:
	if (unlikely(!low_latency)) {
		free_irq(mas->irq, spi);
	}
spi_geni_probe_runtime_disable:
	pm_runtime_disable(dev);
	spi_master_put(spi);
	return ret;
}

static int spi_geni_remove(struct platform_device *pdev)
{
	struct spi_master *spi = platform_get_drvdata(pdev);
	struct spi_geni_master *mas = spi_master_get_devdata(spi);

	/* Unregister _before_ disabling pm_runtime() so we stop transfers */
	spi_unregister_master(spi);

	if (unlikely(!low_latency)) {
		free_irq(mas->irq, spi);
	}
	pm_runtime_disable(&pdev->dev);
	return 0;
}

static int __maybe_unused spi_geni_runtime_suspend(struct device *dev)
{
	struct spi_master *spi = dev_get_drvdata(dev);
	struct spi_geni_master *mas = spi_master_get_devdata(spi);

	return geni_se_resources_off(&mas->se);
}

static int __maybe_unused spi_geni_runtime_resume(struct device *dev)
{
	struct spi_master *spi = dev_get_drvdata(dev);
	struct spi_geni_master *mas = spi_master_get_devdata(spi);

	return geni_se_resources_on(&mas->se);
}

static int __maybe_unused spi_geni_suspend(struct device *dev)
{
	struct spi_master *spi = dev_get_drvdata(dev);
	int ret;

	ret = spi_master_suspend(spi);
	if (ret)
		return ret;

	ret = pm_runtime_force_suspend(dev);
	if (ret)
		spi_master_resume(spi);

	return ret;
}

static int __maybe_unused spi_geni_resume(struct device *dev)
{
	struct spi_master *spi = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		return ret;

	ret = spi_master_resume(spi);
	if (ret)
		pm_runtime_force_suspend(dev);

	return ret;
}

static const struct dev_pm_ops spi_geni_pm_ops = { SET_RUNTIME_PM_OPS(
	spi_geni_runtime_suspend, spi_geni_runtime_resume,
	NULL) SET_SYSTEM_SLEEP_PM_OPS(spi_geni_suspend, spi_geni_resume) };

static const struct of_device_id spi_geni_dt_match[] = {
	{ .compatible = "qorvo,geni-spi-lowlatency" },
	{}
};
MODULE_DEVICE_TABLE(of, spi_geni_dt_match);

static struct platform_driver spi_geni_driver = {
	.probe  = spi_geni_probe,
	.remove = spi_geni_remove,
	.driver = {
		.name = "geni_spi_lowlatency",
		.pm = &spi_geni_pm_ops,
		.of_match_table = spi_geni_dt_match,
	},
};
module_platform_driver(spi_geni_driver);

MODULE_DESCRIPTION(
	"SPI driver for GENI based QUP cores with low-latency support");
MODULE_LICENSE("GPL v2");
