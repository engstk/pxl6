/* drivers/input/touchscreen/sec_ts.c
 *
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 * http://www.samsungsemi.com/
 *
 * Core file for Samsung TSC driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

struct sec_ts_data *tsp_info;

#include "sec_ts.h"
#include <samsung/exynos_drm_connector.h>

/* init the kfifo for health check. */
#define SEC_TS_HC_KFIFO_LEN 4 /* must be power of 2. */
DEFINE_KFIFO(hc_fifo, struct sec_ts_health_check, SEC_TS_HC_KFIFO_LEN);

/* init the kfifo for debug used. */
#define SEC_TS_DEBUG_KFIFO_LEN 4 /* must be power of 2. */
DEFINE_KFIFO(debug_fifo, struct sec_ts_coordinate, SEC_TS_DEBUG_KFIFO_LEN);
static struct sec_ts_coordinate last_coord[SEC_TS_DEBUG_KFIFO_LEN];

/* Switch GPIO values */
#define SEC_SWITCH_GPIO_VALUE_SLPI_MASTER	1
#define SEC_SWITCH_GPIO_VALUE_AP_MASTER		0

struct sec_ts_data *ts_dup;

#ifndef CONFIG_SEC_SYSFS
/* Declare extern sec_class */
struct class *sec_class;
#endif

#ifdef USE_POWER_RESET_WORK
static void sec_ts_reset_work(struct work_struct *work);
#endif
static void sec_ts_fw_update_work(struct work_struct *work);
static void sec_ts_suspend_work(struct work_struct *work);
static void sec_ts_resume_work(struct work_struct *work);
#ifdef USE_CHARGER_WORK
static void sec_ts_charger_work(struct work_struct *work);
#endif

#ifdef USE_OPEN_CLOSE
static int sec_ts_input_open(struct input_dev *dev);
static void sec_ts_input_close(struct input_dev *dev);
#endif

static int register_panel_bridge(struct sec_ts_data *ts);
static void unregister_panel_bridge(struct drm_bridge *bridge);

int sec_ts_read_information(struct sec_ts_data *ts);

#ifndef I2C_INTERFACE
u32 sec_ts_spi_delay(u8 reg, u32 data_len)
{
	u32 delay_us = 100;

	switch (reg) {
	case SEC_TS_READ_TOUCH_RAWDATA:
		delay_us = 500;
		break;
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	case SEC_TS_CMD_HEATMAP_READ:
		delay_us = 500;
		break;
#endif
	case SEC_TS_CMD_CUSTOMLIB_READ_PARAM:
		delay_us = min(120 + (data_len >> 2), (u32) 500);
		break;
	case SEC_TS_READ_ALL_EVENT:
		delay_us = 550;
		break;
	case SEC_TS_READ_CSRAM_RTDP_DATA:
		delay_us = 550;
		break;
	case SEC_TS_CAAT_READ_STORED_DATA:
		delay_us = 550;
		break;
	case SEC_TS_CMD_FLASH_READ_DATA:
		delay_us = 2000;
		break;
	case SEC_TS_READ_FIRMWARE_INTEGRITY:
		delay_us = 20*1000;
		break;
	case SEC_TS_READ_SELFTEST_RESULT:
		delay_us = 4000;
		break;
	}

	usleep_range(delay_us, delay_us + 1);

	return delay_us;
}

int sec_ts_spi_post_delay(u8 reg)
{
	switch (reg) {
	case SEC_TS_READ_TOUCH_RAWDATA:
	case SEC_TS_CMD_FLASH_READ_DATA:
	case SEC_TS_READ_SELFTEST_RESULT:
		return 500;
	default: return 0;
	}
}
#endif

int sec_ts_write(struct sec_ts_data *ts, u8 reg, u8 *data, int len)
{
	u8 *buf;
	int ret;
	unsigned char retry;
#ifdef I2C_INTERFACE
	struct i2c_msg msg;
#else
	struct spi_message msg;
	struct spi_transfer transfer[1] = { { 0 } };
	unsigned int i;
	unsigned int spi_len = 0;
	unsigned char checksum = 0x0;
#endif

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			"%s: POWER_STATUS: OFF\n", __func__);
		goto err;
	}

#ifdef I2C_INTERFACE
	if (len + 1 > sizeof(ts->io_write_buf)) {
		input_err(true, &ts->client->dev,
			"%s: len is larger than buffer size\n", __func__);
		return -EINVAL;
	}
#else
	/* add 3 zero stuffing tx bytes at last */
	if (SEC_TS_SPI_HEADER_SIZE + 1 + len + SEC_TS_SPI_CHECKSUM_SIZE + 3 >
		sizeof(ts->io_write_buf)) {
		input_err(true, &ts->client->dev,
			"%s: len is larger than buffer size\n", __func__);
		return -EINVAL;
	}
#endif

	mutex_lock(&ts->io_mutex);

	buf = ts->io_write_buf;
#ifdef I2C_INTERFACE
	buf[0] = reg;
	memcpy(buf + 1, data, len);

	msg.addr = ts->client->addr;
	msg.flags = 0;
	msg.len = len + 1;
	msg.buf = buf;
#else

	buf[0] = SEC_TS_SPI_SYNC_CODE;
	buf[1] = ((len + 1) >> 8) & 0xFF;
	buf[2] = (len + 1) & 0xFF;
	buf[3] = 0x00;
	buf[4] = 0x00;
	buf[5] = reg;
	memcpy(buf + SEC_TS_SPI_HEADER_SIZE + 1, data, len);

	spi_len = SEC_TS_SPI_HEADER_SIZE + 1 + len;
	// spi_len = SPI header size(5)+register(1)+data size(len)
	for (i = 0; i < spi_len ; i++)
		checksum += buf[i];
	buf[spi_len++] = checksum;
	// spi_len += checksum(1)

	spi_message_init(&msg);

	/* add 3 zero stuffing tx bytes at last */
	memset(ts->io_write_buf + spi_len, 0x00, 3);
	/* spi transfer size should be multiple of 4
	 **/
	spi_len = (spi_len + 3) & ~3;
	transfer[0].len = spi_len;
	transfer[0].tx_buf = buf;
	transfer[0].rx_buf = NULL;
	spi_message_add_tail(&transfer[0], &msg);

#ifdef SEC_TS_DEBUG_IO
	input_info(true, &ts->client->dev, "%s: ", __func__);
//	for (i = 0; i < SEC_TS_SPI_HEADER_SIZE + 1 + len + 1; i++)
	for (i = 0; i < 8; i++)
		input_info(true, &ts->client->dev, "%X ", buf[i]);
	input_info(true, &ts->client->dev, "\n");
#endif

#endif

	for (retry = 0; retry < SEC_TS_IO_RETRY_CNT; retry++) {
#ifdef I2C_INTERFACE
		if ((ret = i2c_transfer(ts->client->adapter, &msg, 1)) == 1)
			break;
#else
		if ((ret = spi_sync(ts->client, &msg)) == 0)
			break;
#endif

		if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
			input_err(true, &ts->client->dev,
				  "%s: POWER_STATUS: OFF, retry: %d\n",
				  __func__, retry);
			mutex_unlock(&ts->io_mutex);
			goto err;
		}

		usleep_range(1 * 1000, 1 * 1000);

		if (retry > 1) {
			input_err(true, &ts->client->dev,
				  "%s: retry %d\n", __func__, retry + 1);
			ts->comm_err_count++;
		}
	}

	mutex_unlock(&ts->io_mutex);

	if (retry == SEC_TS_IO_RETRY_CNT) {
		input_err(true, &ts->client->dev,
			  "%s: write over retry limit\n", __func__);
		ret = -EIO;
#ifdef USE_POR_AFTER_I2C_RETRY
		if (ts->probe_done && !ts->reset_is_on_going)
			schedule_delayed_work(&ts->reset_work,
				msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
#endif
	}

#ifdef I2C_INTERFACE
	if (ret == 1)
#else
	if (ret == 0)
#endif
		return 0;
err:
	return -EIO;
}

static int sec_ts_read_internal(struct sec_ts_data *ts, u8 reg,
			     u8 *data, int len, bool dma_safe)
{
	u8 *buf;
	int ret;
	unsigned char retry;
#ifdef I2C_INTERFASCE
	struct i2c_msg msg[2];
#else
	struct spi_message msg;
	struct spi_transfer transfer[1] = { { 0 } };
	u32 spi_delay_us = 0;
	unsigned int i;
	unsigned int spi_write_len = 0, spi_read_len = 0;
	unsigned char write_checksum = 0x0, read_checksum = 0x0;
	int copy_size = 0, copy_cur = 0;
#endif
	int remain = len;

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			"%s: POWER_STATUS: OFF\n", __func__);
		goto err;
	}

#ifndef I2C_INTERFACE
	/* add 3 zero stuffing tx bytes at last */
	if (SEC_TS_SPI_HEADER_SIZE + 1 + SEC_TS_SPI_CHECKSUM_SIZE + 3 >
		sizeof(ts->io_write_buf)) {
		input_err(true, &ts->client->dev,
			"%s: len is larger than buffer size\n", __func__);
		return -EINVAL;
	}
#endif

	if (len > sizeof(ts->io_read_buf) && dma_safe == false) {
		input_err(true, &ts->client->dev,
			"%s: len %d over pre-allocated size %d\n",
			__func__, len, IO_PREALLOC_READ_BUF_SZ);
		return -ENOSPC;
	}

	mutex_lock(&ts->io_mutex);

	buf = ts->io_write_buf;
#ifdef I2C_INTERFACE
	buf[0] = reg;

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	if (dma_safe == false)
		msg[1].buf = ts->io_read_buf;
	else
		msg[1].buf = data;
#else

	buf[0] = SEC_TS_SPI_SYNC_CODE;
	buf[1] = 0x00;
	buf[2] = 0x01;
	buf[3] = (len >> 8) & 0xFF;
	buf[4] = len & 0xFF;
	buf[5] = reg;

	spi_write_len = SEC_TS_SPI_HEADER_SIZE + 1;
	for (i = 0; i < spi_write_len; i++)
		write_checksum += buf[i];
	buf[spi_write_len] = write_checksum;
	spi_write_len += SEC_TS_SPI_CHECKSUM_SIZE;
	/* add 3 zero stuffing tx bytes at last */
	memset(ts->io_write_buf + spi_write_len, 0x00, 3);
	spi_write_len = (spi_write_len + 3) & ~3;

	spi_read_len = len +
		SEC_TS_SPI_READ_HEADER_SIZE + SEC_TS_SPI_CHECKSUM_SIZE;
	spi_read_len = (spi_read_len + 3) & ~3;
#endif
	if (len <= ts->io_burstmax) {
#ifdef I2C_INTERFACE
		for (retry = 0; retry < SEC_TS_IO_RETRY_CNT; retry++) {
			ret = i2c_transfer(ts->client->adapter, msg, 2);
			if (ret == 2)
				break;

			usleep_range(1 * 1000, 1 * 1000);
			if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
				input_err(true, &ts->client->dev,
					  "%s: POWER_STATUS: OFF, retry: %d\n",
					  __func__, retry);
				mutex_unlock(&ts->io_mutex);
				goto err;
			}

			if (retry > 1) {
				input_err(true, &ts->client->dev,
					"%s: retry %d\n", __func__, retry + 1);
				ts->comm_err_count++;
			}
		}
		if (ret == 2 && dma_safe == false)
			memcpy(data,
			       ts->io_read_buf[SEC_TS_SPI_READ_HEADER_SIZE],
			       len);
#else
		for (retry = 0; retry < SEC_TS_IO_RETRY_CNT; retry++) {
			spi_message_init(&msg);
			// spi transfer size should be multiple of 4
			transfer[0].len = spi_write_len;
			transfer[0].tx_buf = buf;
			transfer[0].rx_buf = NULL;
			spi_message_add_tail(&transfer[0], &msg);

			ret = spi_sync(ts->client, &msg);
#ifdef SEC_TS_DEBUG_IO
			input_info(true, &ts->client->dev,
				"%s: spi write buf %X %X %X %X %X %X %X\n",
				__func__, buf[0], buf[1], buf[2],
				buf[3], buf[4], buf[5], buf[6]);
#endif
			// write fail
			if (ret != 0) {
				ret = -EIO;

				input_err(true, &ts->client->dev,
					"%s: spi write retry %d\n",
					__func__, retry + 1);
				ts->comm_err_count++;

				usleep_range(1 * 1000, 1 * 1000);
				if (ts->power_status ==
					SEC_TS_STATE_POWER_OFF) {
					input_err(true, &ts->client->dev,
						"%s: POWER_STATUS: OFF, retry: %d\n",
						__func__, retry);
					mutex_unlock(&ts->io_mutex);
					goto err;
				}

				if (retry == SEC_TS_IO_RETRY_CNT - 1) {
					input_err(true, &ts->client->dev,
						"%s: write reg retry over retry limit, skip read\n",
						__func__);
					goto skip_spi_read;
				}

				continue;
			}

			spi_delay_us = sec_ts_spi_delay(reg, len);

			// read sequence start
			spi_message_init(&msg);
			transfer[0].len = spi_read_len;
			transfer[0].tx_buf = NULL;
			transfer[0].rx_buf = ts->io_read_buf;
			spi_message_add_tail(&transfer[0], &msg);
			ret = spi_sync(ts->client, &msg);

			for (i = 0, read_checksum = 0x0;
				i < (SEC_TS_SPI_READ_HEADER_SIZE + len);
				i++)
				read_checksum += ts->io_read_buf[i];

#ifdef SEC_TS_DEBUG_IO
			input_info(true, &ts->client->dev, "%s: ", __func__);
//			for (i = 0; i < spi_read_len; i++)
			for (i = 0; i < 8; i++)
				input_info(true, &ts->client->dev,
					"%X ",
					ts->io_read_buf[i]);
			input_info(true, &ts->client->dev,
				"\n%s: checksum = %X",
				__func__, read_checksum);
#endif
			// read fail
			if (ret != 0 ||
				ts->io_read_buf[0] != SEC_TS_SPI_SYNC_CODE ||
				reg != ts->io_read_buf[5] ||
				// ts->io_read_buf[6] != SEC_TS_SPI_CMD_OK ||
				read_checksum !=
					ts->io_read_buf[
						SEC_TS_SPI_READ_HEADER_SIZE +
					len]) {

				ret = -EIO;

				/*
				 * For LP idle state from AOD to normal screen-on,
				 * T-IC needs one SPI cmds to wake-up.
				 * Therefore, change the log level to warning
				 * for this intended behavir.
				 */
				if (!completion_done(&ts->bus_resumed) &&
					reg == SEC_TS_CMD_CHG_SYSMODE) {
					dev_warn(&ts->client->dev,
						"%s: wake-up touch(#%d) by 0x%02X cmd delay_us(%d)\n",
						__func__, retry + 1, reg, spi_delay_us);
				} else {
					input_err(true, &ts->client->dev,
						"%s: retry %d for 0x%02X size(%d) delay_us(%d)\n",
						__func__, retry + 1, reg, len, spi_delay_us);
						ts->comm_err_count++;
				}

				usleep_range(1 * 1000, 1 * 1000);
				if (ts->power_status ==
					SEC_TS_STATE_POWER_OFF) {
					input_err(true, &ts->client->dev,
						"%s: POWER_STATUS: OFF, retry: %d\n",
						__func__, retry);
					mutex_unlock(&ts->io_mutex);
					goto err;
				}
				continue;
			} else
				break;
		}
		if (ret == 0)
			memcpy(data, ts->io_read_buf +
			       SEC_TS_SPI_READ_HEADER_SIZE, len);

		usleep_range(sec_ts_spi_post_delay(reg),
			     sec_ts_spi_post_delay(reg) + 1);

#endif //I2C_INTERFACE
	} else {
		/*
		 * read buffer is 256 byte. do not support long buffer over
		 * than 256. So, try to separate reading data about 256 bytes.
		 **/
#ifdef I2C_INTERFACE
		for (retry = 0; retry < SEC_TS_IO_RETRY_CNT; retry++) {
			ret = i2c_transfer(ts->client->adapter, msg, 1);
			if (ret == 1)
				break;

			usleep_range(1 * 1000, 1 * 1000);
			if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
				input_err(true, &ts->client->dev,
					"%s: POWER_STATUS: OFF, retry: %d\n",
					__func__, retry);
				mutex_unlock(&ts->io_mutex);
				goto err;
			}

			if (retry > 1) {
				input_err(true, &ts->client->dev,
					"%s: retry %d\n",
					__func__, retry + 1);
				ts->comm_err_count++;
			}
		}

		do {
			if (remain > ts->io_burstmax)
				msg[1].len = ts->io_burstmax;
			else
				msg[1].len = remain;

			remain -= ts->io_burstmax;

			for (retry = 0; retry < SEC_TS_IO_RETRY_CNT; retry++) {
				ret = i2c_transfer(ts->client->adapter,
						   &msg[1], 1);
				if (ret == 1)
					break;
				usleep_range(1 * 1000, 1 * 1000);
				if (ts->power_status ==
					SEC_TS_STATE_POWER_OFF) {
					input_err(true, &ts->client->dev,
						"%s: POWER_STATUS: OFF, retry: %d\n",
						__func__, retry);
					mutex_unlock(&ts->io_mutex);
					goto err;
				}

				if (retry > 1) {
					input_err(true, &ts->client->dev,
						"%s: retry %d\n",
						__func__, retry + 1);
					ts->comm_err_count++;
				}
			}

			msg[1].buf += msg[1].len;

		} while (remain > 0);

		if (ret == 1 && dma_safe == false)
			memcpy(data, ts->io_read_buf, len);
#else
		for (retry = 0; retry < SEC_TS_IO_RETRY_CNT; retry++) {
			spi_message_init(&msg);
			// spi transfer size should be multiple of 4
			transfer[0].len = spi_write_len;
			transfer[0].tx_buf = buf;
			transfer[0].rx_buf = NULL;
			spi_message_add_tail(&transfer[0], &msg);

			ret = spi_sync(ts->client, &msg);

			// write fail
			if (ret != 0) {
				ret = -EIO;

				input_err(true, &ts->client->dev,
					"%s: spi write retry %d\n",
					__func__, retry + 1);
				ts->comm_err_count++;

				usleep_range(1 * 1000, 1 * 1000);

				if (ts->power_status ==
					SEC_TS_STATE_POWER_OFF) {
					input_err(true, &ts->client->dev,
						"%s: POWER_STATUS: OFF, retry: %d\n",
						__func__, retry);
					mutex_unlock(&ts->io_mutex);
					goto err;
				}

				if (retry == SEC_TS_IO_RETRY_CNT - 1) {
					input_err(true, &ts->client->dev,
						"%s: write reg retry over retry limit, skip read\n",
						__func__);
					goto skip_spi_read;
				}

				continue;
			}

			sec_ts_spi_delay(reg, len);

			copy_size = 0;
			remain = spi_read_len;
			do {
				if (remain > ts->io_burstmax)
					copy_cur = ts->io_burstmax;
				else
					copy_cur = remain;

				spi_message_init(&msg);

				transfer[0].len = copy_cur;
				transfer[0].tx_buf = NULL;
				transfer[0].rx_buf =
					&ts->io_read_buf[copy_size];
				// CS needs to stay low until read seq. is done
				transfer[0].cs_change =
					(remain > ts->io_burstmax) ? 1 : 0;

				spi_message_add_tail(&transfer[0], &msg);

				copy_size += copy_cur;
				remain -= copy_cur;

				ret = spi_sync(ts->client, &msg);
#ifdef SEC_TS_DEBUG_IO
				input_info(true, &ts->client->dev,
					"%s: ", __func__);
				for (i = 0; i < 8; i++)
					input_info(true,
						   &ts->client->dev, "%X ",
						   ts->io_read_buf[i]);
				input_info(true, &ts->client->dev,
					"\n%s: checksum = %X",
					__func__, read_checksum);
#endif

				if (ret != 0) {
					ret = -EIO;

					input_err(true, &ts->client->dev,
						"%s: retry %d\n",
						__func__, retry + 1);
					ts->comm_err_count++;

					usleep_range(1 * 1000, 1 * 1000);
					if (ts->power_status
						== SEC_TS_STATE_POWER_OFF) {
						input_err(true,
							&ts->client->dev,
							"%s: POWER_STATUS: OFF, retry: %d\n",
							__func__, retry);
						mutex_unlock(&ts->io_mutex);
						goto err;
					}
					break;
				}
			} while (remain > 0);
			if (ret != 0) { // read fail, retry
				ret = -EIO;
				continue;
			}

			for (i = 0, read_checksum = 0x0;
				i < SEC_TS_SPI_READ_HEADER_SIZE + len; i++)
				read_checksum += ts->io_read_buf[i];
			//read success
			if (ts->io_read_buf[0] == SEC_TS_SPI_SYNC_CODE &&
				// ts->io_read_buf[6] == SEC_TS_SPI_CMD_OK &&
				reg == ts->io_read_buf[5] &&
				read_checksum ==
				ts->io_read_buf[SEC_TS_SPI_READ_HEADER_SIZE +
				len])
				break;
			//read data fail
			else if (ts->io_read_buf[6]
				 == SEC_TS_SPI_CMD_UNKNOWN ||
				 ts->io_read_buf[6]
				 == SEC_TS_SPI_CMD_BAD_PARAM) {
				input_info(true, &ts->client->dev,
					"%s: CMD_NG cmd(M) = %X, cmd(S) = %X, cmd_result = %X\n",
					__func__, reg, ts->io_read_buf[5],
					ts->io_read_buf[6]);
				ret = -EIO;
				continue;
			} else {
				input_info(true, &ts->client->dev,
					"%s: spi fail, ret %d, sync code %X, reg(M) %X, reg(S) %X, cmd_result %X, chksum(M) %X, chksum(S) %X\n",
					__func__, ret, ts->io_read_buf[0],
					reg, ts->io_read_buf[5],
					ts->io_read_buf[6], read_checksum,
					ts->io_read_buf[
						SEC_TS_SPI_READ_HEADER_SIZE +
					len]);
				ret = -EIO;
				continue;
			}
		}
		if (ret == 0)
			memcpy(data, ts->io_read_buf +
				SEC_TS_SPI_READ_HEADER_SIZE, len);

		usleep_range(sec_ts_spi_post_delay(reg),
			     sec_ts_spi_post_delay(reg) + 1);
#endif
	}
skip_spi_read:
	mutex_unlock(&ts->io_mutex);

	if (retry == SEC_TS_IO_RETRY_CNT) {
		input_err(true, &ts->client->dev,
			"%s: read reg(%#x) over retry limit, comm_err_count %d, io_err_count %d\n",
			__func__, reg, ts->comm_err_count, ts->io_err_count);
		ret = -EIO;
		ts->io_err_count++;
#ifdef USE_POR_AFTER_I2C_RETRY
		if (ts->probe_done && !ts->reset_is_on_going)
			schedule_delayed_work(&ts->reset_work,
				msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
#endif

	} else
		ts->io_err_count = 0;

	/*
	 * Do hw reset if continuously failed over SEC_TS_IO_RESET_CNT times
	 * except for FW update process.
	 */
	if (ts->io_err_count >= SEC_TS_IO_RESET_CNT &&
	    (ts->bus_refmask & SEC_TS_BUS_REF_FW_UPDATE) == 0) {
		ts->hw_reset_count++;
		ts->io_err_count = 0;
		sec_ts_system_reset(ts, RESET_MODE_HW, false, true);
	}

	return ret;

err:
	return -EIO;
}

static int sec_ts_write_burst_internal(struct sec_ts_data *ts,
					   u8 *data, int len, bool dma_safe)
{
	int ret;
	int retry;
#ifndef I2C_INTERFACE
	struct spi_message msg;
	struct spi_transfer transfer[1] = { { 0 } };
	unsigned int i;
	unsigned int spi_len = 0;
	unsigned char checksum = 0x0;
#endif

#ifdef I2C_INTERFACE
	if (len > sizeof(ts->io_write_buf) && dma_safe == false) {
		input_err(true, &ts->client->dev,
			"%s: len %d over pre-allocated size %d\n",
			__func__, len, sizeof(ts->io_write_buf));
		return -ENOSPC;
	}
#else
	/* add 3 zero stuffing tx bytes at last */
	if (SEC_TS_SPI_HEADER_SIZE + len + SEC_TS_SPI_CHECKSUM_SIZE + 3 >
		sizeof(ts->io_write_buf)) {
		input_err(true, &ts->client->dev,
			"%s: len is larger than buffer size\n", __func__);
		return -EINVAL;
	}
#endif

	mutex_lock(&ts->io_mutex);
#ifdef I2C_INTERFACE
	if (dma_safe == false) {
		memcpy(ts->io_write_buf, data, len);
		data = ts->io_write_buf;
	}
#else

	ts->io_write_buf[0] = SEC_TS_SPI_SYNC_CODE;
	ts->io_write_buf[1] = (len >> 8) & 0xFF;
	ts->io_write_buf[2] = len & 0xFF;
	ts->io_write_buf[3] = 0x0;
	ts->io_write_buf[4] = 0x0;

	memcpy(ts->io_write_buf + SEC_TS_SPI_HEADER_SIZE, data, len);


	spi_len = SEC_TS_SPI_HEADER_SIZE + len;
	for (i = 0; i < spi_len; i++)
		checksum += ts->io_write_buf[i];

	ts->io_write_buf[spi_len] = checksum;
	spi_len += SEC_TS_SPI_CHECKSUM_SIZE;

	spi_message_init(&msg);

	/* add 3 zero stuffing tx bytes at last */
	memset(ts->io_write_buf + spi_len, 0x00, 3);
	spi_len = (spi_len + 3) & ~3;
	transfer[0].len = spi_len;
	transfer[0].tx_buf = ts->io_write_buf;
	transfer[0].rx_buf = NULL;
	spi_message_add_tail(&transfer[0], &msg);

#ifdef SEC_TS_DEBUG_IO
	input_info(true, &ts->client->dev, "%s:\n", __func__);
	for (i = 0; i < spi_len; i++)
		input_info(true, &ts->client->dev, "%X ", ts->io_write_buf[i]);
	input_info(true, &ts->client->dev, "\n");
#endif

#endif

	for (retry = 0; retry < SEC_TS_IO_RETRY_CNT; retry++) {
#ifdef I2C_INTERFACE
		if ((ret = i2c_master_send(ts->client, data, len)) == len)
			break;
#else
		if ((ret = spi_sync(ts->client, &msg)) == 0)
			break;
#endif

		usleep_range(1 * 1000, 1 * 1000);

		if (retry > 1) {
			input_err(true, &ts->client->dev,
				  "%s: retry %d\n", __func__, retry + 1);
			ts->comm_err_count++;
		}
	}

	mutex_unlock(&ts->io_mutex);
	if (retry == SEC_TS_IO_RETRY_CNT) {
		input_err(true, &ts->client->dev,
			  "%s: write over retry limit\n", __func__);
		ret = -EIO;
	}

	return ret;
}

static int sec_ts_read_bulk_internal(struct sec_ts_data *ts,
					 u8 *data, int len, bool dma_safe)
{
	int ret;
	unsigned char retry;
	int remain = len;
#ifdef I2C_INTERFACE
	struct i2c_msg msg;
#else
	struct spi_message msg;
	struct spi_transfer transfer[1] = { { 0 } };
	unsigned int i;
	unsigned int spi_len = 0;
	unsigned char checksum = 0x0;
	int copy_size = 0, copy_cur = 0;
	int retry_msg = 0;
#endif

	if (len > sizeof(ts->io_read_buf) && dma_safe == false) {
		input_err(true, &ts->client->dev,
			  "%s: len %d over pre-allocated size %d\n", __func__,
			  len, sizeof(ts->io_read_buf));
		return -ENOSPC;
	}

	mutex_lock(&ts->io_mutex);

#ifdef I2C_INTERFACE
	msg.addr = ts->client->addr;
	msg.flags = I2C_M_RD;
	msg.len = len;
	if (dma_safe == false)
		msg.buf = ts->io_read_buf;
	else
		msg.buf = data;

	do {
		if (remain > ts->io_burstmax)
			msg.len = ts->io_burstmax;
		else
			msg.len = remain;

		remain -= ts->io_burstmax;

		for (retry = 0; retry < SEC_TS_IO_RETRY_CNT; retry++) {
			ret = i2c_transfer(ts->client->adapter, &msg, 1);
			if (ret == 1)
				break;
			usleep_range(1 * 1000, 1 * 1000);

			if (retry > 1) {
				input_err(true, &ts->client->dev,
					  "%s: retry %d\n",
					  __func__, retry + 1);
				ts->comm_err_count++;
			}
		}

		if (retry == SEC_TS_IO_RETRY_CNT) {
			input_err(true, &ts->client->dev,
				  "%s: read over retry limit\n", __func__);
			ret = -EIO;
			break;
		}

		msg.buf += msg.len;

	} while (remain > 0);

	if (ret == 1 && dma_safe == false)
		memcpy(data, ts->io_read_buf, len);
#else
retry_message:
	remain = spi_len = (SEC_TS_SPI_READ_HEADER_SIZE + len +
			    SEC_TS_SPI_CHECKSUM_SIZE + 3) & ~3;
	do {
		if (remain > ts->io_burstmax)
			copy_cur = ts->io_burstmax;
		else
			copy_cur = remain;

		spi_message_init(&msg);
		transfer[0].len = copy_cur;
		transfer[0].tx_buf = NULL;
		transfer[0].rx_buf = &ts->io_read_buf[copy_size];
		/* CS needs to stay low until read seq. is done
		 */
		transfer[0].cs_change = (remain > ts->io_burstmax) ? 1 : 0;

		spi_message_add_tail(&transfer[0], &msg);

		copy_size += copy_cur;
		remain -= copy_cur;

		for (retry = 0; retry < SEC_TS_IO_RETRY_CNT; retry++) {
			ret = spi_sync(ts->client, &msg);
			if (ret == 0)
				break;

			usleep_range(1 * 1000, 1 * 1000);
			if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
				input_err(true, &ts->client->dev,
					  "%s: POWER_STATUS: OFF, retry: %d\n",
					  __func__, retry);
				mutex_unlock(&ts->io_mutex);
				goto err;
			}

			if (retry > 1) {
				input_err(true, &ts->client->dev,
					"%s: retry %d\n", __func__, retry + 1);
				ts->comm_err_count++;
			}
		}
	} while (remain > 0);

	for (i = 0, checksum = 0; i < SEC_TS_SPI_READ_HEADER_SIZE + len; i++)
		checksum += ts->io_read_buf[i];

	if (ret == 0 && ts->io_read_buf[0] == SEC_TS_SPI_SYNC_CODE &&
		checksum == ts->io_read_buf[SEC_TS_SPI_READ_HEADER_SIZE + len])
		memcpy(data, ts->io_read_buf + SEC_TS_SPI_READ_HEADER_SIZE,
		       len);
	else {
		input_info(true, &ts->client->dev,
			   "%s: spi fail, ret %d, sync code %X, reg(S) %X, chksum(M) %X, chksum(S) %X\n",
			__func__, ret, ts->io_read_buf[0], ts->io_read_buf[5],
			checksum,
			ts->io_read_buf[SEC_TS_SPI_READ_HEADER_SIZE + len]);
		if (retry_msg++ < SEC_TS_IO_RETRY_CNT)
			goto retry_message;
	}
#endif
	mutex_unlock(&ts->io_mutex);

#ifdef I2C_INTERFACE
	if (ret == 1)
#else
	if (ret == 0)
#endif
		return 0;
err:
	return -EIO;
}

/* Wrapper API for read and write */
int sec_ts_read(struct sec_ts_data *ts, u8 reg, u8 *data, int len)
{
	return sec_ts_read_internal(ts, reg, data, len, false);
}

int sec_ts_read_heap(struct sec_ts_data *ts, u8 reg, u8 *data, int len)
{
	return sec_ts_read_internal(ts, reg, data, len, true);
}

int sec_ts_write_burst(struct sec_ts_data *ts, u8 *data, int len)
{
	return sec_ts_write_burst_internal(ts, data, len, false);
}

int sec_ts_write_burst_heap(struct sec_ts_data *ts, u8 *data, int len)
{
	return sec_ts_write_burst_internal(ts, data, len, true);
}

int sec_ts_read_bulk(struct sec_ts_data *ts, u8 *data, int len)
{
	return sec_ts_read_bulk_internal(ts, data, len, false);
}

int sec_ts_read_bulk_heap(struct sec_ts_data *ts, u8 *data, int len)
{
	return sec_ts_read_bulk_internal(ts, data, len, true);
}

static int sec_ts_read_from_customlib(struct sec_ts_data *ts, u8 *data, int len)
{
	int ret;

	ret = sec_ts_write(ts, SEC_TS_CMD_CUSTOMLIB_READ_PARAM, data, 2);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			"%s: fail to write custom library command\n", __func__);

	usleep_range(100, 100);

	ret = sec_ts_read(ts, SEC_TS_CMD_CUSTOMLIB_READ_PARAM, (u8 *)data, len);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			"%s: fail to read custom library command\n", __func__);

	return ret;
}

#if defined(CONFIG_TOUCHSCREEN_DUMP_MODE)
#include <linux/sec_debug.h>
extern struct tsp_dump_callbacks dump_callbacks;
static struct delayed_work *p_ghost_check;

static void sec_ts_check_rawdata(struct work_struct *work)
{
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data,
					      ghost_check.work);

	if (ts->tsp_dump_lock == 1) {
		input_err(true, &ts->client->dev,
			  "%s: ignored ## already checking..\n", __func__);
		return;
	}
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: ignored ## IC is power off\n", __func__);
		return;
	}

	ts->tsp_dump_lock = 1;
	input_info(true, &ts->client->dev, "%s: start ##\n", __func__);
	sec_ts_run_rawdata_all((void *)ts, false);
	msleep(100);

	input_info(true, &ts->client->dev, "%s: done ##\n", __func__);
	ts->tsp_dump_lock = 0;

}

static void dump_tsp_log(void)
{
	pr_info("%s: %s %s: start\n", SEC_TS_NAME, SECLOG, __func__);

#ifdef CONFIG_BATTERY_SAMSUNG
	if (lpcharge == 1) {
		pr_err("%s: %s %s: ignored ## lpm charging Mode!!\n",
		       SEC_TS_NAME, SECLOG, __func__);
		return;
	}
#endif

	if (p_ghost_check == NULL) {
		pr_err("%s: %s %s: ignored ## tsp probe fail!!\n",
		       SEC_TS_NAME, SECLOG, __func__);
		return;
	}
	schedule_delayed_work(p_ghost_check, msecs_to_jiffies(100));
}
#endif


void sec_ts_delay(unsigned int ms)
{
	if (ms < 20)
		usleep_range(ms * 1000, ms * 1000);
	else
		msleep(ms);
}

int sec_ts_wait_for_ready(struct sec_ts_data *ts, unsigned int ack)
{
	return sec_ts_wait_for_ready_with_count(ts, ack,
						SEC_TS_WAIT_RETRY_CNT);
}

int sec_ts_wait_for_ready_with_count(struct sec_ts_data *ts, unsigned int ack,
				     unsigned int count)
{
	int rc = -1;
	int retry = 0;
	u8 tBuff[SEC_TS_EVENT_BUFF_SIZE] = {0,};

	while (retry < count) {
		if (sec_ts_read(ts, SEC_TS_READ_ONE_EVENT, tBuff,
			SEC_TS_EVENT_BUFF_SIZE) >= 0) {
			if (((tBuff[0] >> 2) & 0xF) == TYPE_STATUS_EVENT_INFO) {
				if (tBuff[1] == ack) {
					rc = 0;
					break;
				}
			} else if (((tBuff[0] >> 2) & 0xF) ==
				   TYPE_STATUS_EVENT_VENDOR_INFO) {
				if (tBuff[1] == ack) {
					rc = 0;
					break;
				}
			}
		}
		sec_ts_delay(20);
		retry++;
	}
	if (retry == count)
		input_err(true, &ts->client->dev, "%s: Time Over\n",
			__func__);

	input_info(true, &ts->client->dev,
		"%s: %02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X [%d]\n",
		__func__, tBuff[0], tBuff[1], tBuff[2], tBuff[3],
		tBuff[4], tBuff[5], tBuff[6], tBuff[7], retry);

	return rc;
}

int sec_ts_read_calibration_report(struct sec_ts_data *ts)
{
	int ret;

	memset(ts->cali_report, 0, sizeof(ts->cali_report));
	ret = sec_ts_read(ts, SEC_TS_READ_CALIBRATION_REPORT,
		ts->cali_report, sizeof(ts->cali_report));
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: failed to read, %d\n", __func__, ret);
		return ret;
	}

	input_info(true, &ts->client->dev,
		"%s: count: %d, pass cnt: %d, fail cnt: %d, status: %X, param ver: %X %X %X %X\n",
		__func__, ts->cali_report_try_cnt, ts->cali_report_pass_cnt,
		ts->cali_report_fail_cnt, ts->cali_report_status,
		ts->cali_report_param_ver[0], ts->cali_report_param_ver[1],
		ts->cali_report_param_ver[2], ts->cali_report_param_ver[3]);

	return ts->cali_report_status;
}

#define PTFLIB_ENCODED_ENABLED_OFFSET_LSB	0xA0
#define PTFLIB_ENCODED_ENABLED_OFFSET_MSB	0x00
#define PTFLIB_ENCODED_ENABLED_TRUE		0x01
#define PTFLIB_ENCODED_ENABLED_FALSE		0x00
static int sec_ts_ptflib_reinit(struct sec_ts_data *ts)
{
	u8 r_data[2] = {0x00, 0x00};
	u8 w_data[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int ret = 0;

	/* Check whether encoded heatmap is inited in custom library. */
	r_data[0] = PTFLIB_ENCODED_ENABLED_OFFSET_LSB;
	r_data[1] = PTFLIB_ENCODED_ENABLED_OFFSET_MSB;
        ret = sec_ts_read_from_customlib(ts, r_data, 2);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: Read encoded heatmap's inited failed.\n",
			  __func__);
		return -EIO;
	}

	if (r_data[1] != 0x01) {
		input_err(true, &ts->client->dev,
			  "%s: Encoded heatmap was not initialized.\n",
			  __func__);
		return -EIO;
	}

	/* Return if encoded heatmap is already enabled. */
	if (r_data[0] == 0x01) {
		input_info(true, &ts->client->dev,
			   "%s: Encoded heatmap is already enabled.\n",
			   __func__);
		return 0;
	}

	/* Enable encoded heatmap. */
	w_data[0] = PTFLIB_ENCODED_ENABLED_OFFSET_LSB;
	w_data[1] = PTFLIB_ENCODED_ENABLED_OFFSET_MSB;
	w_data[2] = PTFLIB_ENCODED_ENABLED_TRUE;
	ret = sec_ts_write(ts, SEC_TS_CMD_CUSTOMLIB_WRITE_PARAM, w_data, 3);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: ptlib - Writing encoded heatmap's enabled register failed.\n",
			  __func__);
		return -EIO;
	}

	/* Check whether encoded heatmap is enabled in customlib. */
	r_data[0] = PTFLIB_ENCODED_ENABLED_OFFSET_LSB;
	r_data[1] = PTFLIB_ENCODED_ENABLED_OFFSET_MSB;
	ret = sec_ts_read_from_customlib(ts, r_data, 2);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: ptlib - Read encoded heatmap's enabled status failed.\n",
			  __func__);
		return -EIO;
	}

	if (r_data[0] != PTFLIB_ENCODED_ENABLED_TRUE) {
		input_err(true, &ts->client->dev,
			  "%s: ptlib - Enabling encoded heatmap failed.\n",
				  __func__);
		return -EIO;
	}

	return 0;
}

#define PTFLIB_GRIP_ENABLED_OFFSET_LSB	0x80
#define PTFLIB_GRIP_ENABLED_OFFSET_MSB	0x00
static int sec_ts_ptflib_grip_prescreen_enable(struct sec_ts_data *ts,
					       int grip_prescreen_mode) {
	u8 r_data[2] = {0x00, 0x00};
	u8 w_data[3] = {0x00, 0x00, 0x00};
	int result;

	input_info(true, &ts->client->dev, "%s: set mode %d.\n",
	    __func__, grip_prescreen_mode);

	if (grip_prescreen_mode < GRIP_PRESCREEN_OFF ||
	    grip_prescreen_mode > GRIP_PRESCREEN_MODE_3) {
		input_err(true, &ts->client->dev,
		    "%s: invalid grip_prescreen_mode value %d.\n",
		    __func__, grip_prescreen_mode);
		return -EINVAL;
	}

	w_data[0] = PTFLIB_GRIP_ENABLED_OFFSET_LSB;
	w_data[1] = PTFLIB_GRIP_ENABLED_OFFSET_MSB;
	w_data[2] = grip_prescreen_mode;
	result = ts->sec_ts_write(ts, SEC_TS_CMD_CUSTOMLIB_WRITE_PARAM,
	    w_data, 3);
	if (result < 0) {
		input_err(true, &ts->client->dev,
		    "%s: Write grip_prescreen_mode register failed.\n",
		    __func__);
		return -EIO;
	}

	r_data[0] = PTFLIB_GRIP_ENABLED_OFFSET_LSB;
	r_data[1] = PTFLIB_GRIP_ENABLED_OFFSET_MSB;
	result = ts->sec_ts_read_customlib(ts, r_data, 2);
	if (result < 0) {
		input_err(true, &ts->client->dev,
		    "%s: Read grip_prescreen_mode register failed.\n",
		    __func__);
		return -EIO;
	}

	if (r_data[0] != grip_prescreen_mode) {
		input_err(true, &ts->client->dev,
		    "%s: Configure grip_prescreen_mode register failed %d.\n",
		    __func__, r_data[0]);
		return -EIO;
	}

	return 0;
}

#define PTFLIB_GRIP_PRESCREEN_TIMEOUT_OFFSET_LSB	0x9A
#define PTFLIB_GRIP_PRESCREEN_TIMEOUT_OFFSET_MSB	0x00
static int sec_ts_ptflib_grip_prescreen_timeout(struct sec_ts_data *ts,
					        int grip_prescreen_timeout) {
	u8 r_data[2] = {0x00, 0x00};
	u8 w_data[4] = {0x00, 0x00, 0x00, 0x00};
	u16 timeout;
	int result;

	input_info(true, &ts->client->dev, "%s: set timeout %d.\n",
	    __func__, grip_prescreen_timeout);

	if (grip_prescreen_timeout < GRIP_PRESCREEN_TIMEOUT_MIN ||
	    grip_prescreen_timeout > GRIP_PRESCREEN_TIMEOUT_MAX) {
		input_err(true, &ts->client->dev,
		    "%s: invalid grip_prescreen_timeout value %d.\n",
		    __func__, grip_prescreen_timeout);
		return -EINVAL;
	}

	w_data[0] = PTFLIB_GRIP_PRESCREEN_TIMEOUT_OFFSET_LSB;
	w_data[1] = PTFLIB_GRIP_PRESCREEN_TIMEOUT_OFFSET_MSB;
	w_data[2] = grip_prescreen_timeout & 0xFF;
	w_data[3] = (grip_prescreen_timeout >> 8) & 0xFF;
	result = ts->sec_ts_write(ts, SEC_TS_CMD_CUSTOMLIB_WRITE_PARAM,
	    w_data, 4);
	if (result < 0) {
		input_err(true, &ts->client->dev,
		    "%s: Write grip_prescreen_timeout register failed.\n",
		    __func__);
		return -EIO;
	}

	r_data[0] = PTFLIB_GRIP_PRESCREEN_TIMEOUT_OFFSET_LSB;
	r_data[1] = PTFLIB_GRIP_PRESCREEN_TIMEOUT_OFFSET_MSB;
	result = ts->sec_ts_read_customlib(ts, r_data, 2);
	if (result < 0) {
		input_err(true, &ts->client->dev,
		    "%s: Read grip_prescreen_timeout register failed.\n",
		    __func__);
		return -EIO;
	}

	timeout = le16_to_cpup((uint16_t *) r_data);
	if (timeout != grip_prescreen_timeout) {
		input_err(true, &ts->client->dev,
		    "%s: Configure grip_prescreen_timeout register failed %d.\n",
		    __func__, timeout);
		return -EIO;
	}

	return 0;
}

#define PTFLIB_GRIP_PRESCREEN_FRAMES_OFFSET_LSB	0x9C
#define PTFLIB_GRIP_PRESCREEN_FRAMES_OFFSET_MSB	0x00
static int sec_ts_ptflib_get_grip_prescreen_frames(struct sec_ts_data *ts) {
	u8 r_data[4] = {0x00, 0x00, 0x00, 0x00};
	int result;

	r_data[0] = PTFLIB_GRIP_PRESCREEN_FRAMES_OFFSET_LSB;
	r_data[1] = PTFLIB_GRIP_PRESCREEN_FRAMES_OFFSET_MSB;
	result = ts->sec_ts_read_customlib(ts, r_data, sizeof(r_data));
	if (result < 0) {
		input_err(true, &ts->client->dev,
		    "%s: Read grip prescreen frames register failed.\n",
		    __func__);
		return -EIO;
	}

	return le32_to_cpup((uint32_t *) r_data);
}

static int sec_ts_ptflib_decoder(struct sec_ts_data *ts, const u16 *in_array,
				 const int in_array_size, u16 *out_array,
				 const int out_array_max_size)
{
	const u16 ESCAPE_MASK = 0xF000;
	const u16 ESCAPE_BIT = 0x8000;

	int i;
	int j;
	int out_array_size = 0;
	u16 prev_word = 0;
	u16 repetition = 0;

	for (i = 0; i < in_array_size; i++) {
		u16 curr_word = in_array[i];
		if ((curr_word & ESCAPE_MASK) == ESCAPE_BIT) {
			repetition = (curr_word & ~ESCAPE_MASK) - 1;
			if (out_array_size + repetition > out_array_max_size)
				break;

			for (j = 0; j < repetition; j++) {
				*out_array++ = prev_word;
				out_array_size++;
			}
		} else {
			if (out_array_size >= out_array_max_size)
				break;

			*out_array++ = curr_word;
			out_array_size++;
			prev_word = curr_word;
		}
	}

	if (i != in_array_size || out_array_size != out_array_max_size) {
		input_info(true, &ts->client->dev,
			   "%s: %d (in=%d, out=%d, rep=%d, out_max=%d).\n",
			   __func__, i, in_array_size, out_array_size,
			   repetition, out_array_max_size);
		return -1;
	}

	return out_array_size;
}

static void sec_ts_reinit(struct sec_ts_data *ts)
{
	u8 w_data[2] = {0x00, 0x00};
	int ret = 0;

	input_info(true, &ts->client->dev,
		"%s : charger=0x%x, Cover=0x%x, Power mode=0x%x\n",
		__func__, ts->charger_mode, ts->touch_functions,
		ts->lowpower_status);

	/* charger mode */
	if (ts->charger_mode != SEC_TS_BIT_CHARGER_MODE_NO) {
		w_data[0] = ts->charger_mode;
		ret = ts->sec_ts_write(ts, SET_TS_CMD_SET_CHARGER_MODE,
				       (u8 *)&w_data[0], 1);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				  "%s: Failed to send command(0x%x)",
				__func__, SET_TS_CMD_SET_CHARGER_MODE);
	}

	/* Cover mode */
	if (ts->touch_functions & SEC_TS_BIT_SETFUNC_COVER) {
		w_data[0] = ts->cover_cmd;
		ret = sec_ts_write(ts, SEC_TS_CMD_SET_COVERTYPE,
				   (u8 *)&w_data[0], 1);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				"%s: Failed to send command(0x%x)",
				__func__, SEC_TS_CMD_SET_COVERTYPE);

		ret = sec_ts_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION,
				   (u8 *)&(ts->touch_functions), 2);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				"%s: Failed to send command(0x%x)",
				__func__, SEC_TS_CMD_SET_TOUCHFUNCTION);
	}

	#ifdef SEC_TS_SUPPORT_CUSTOMLIB
	if (ts->use_customlib)
		sec_ts_set_custom_library(ts);
	#endif

	/* Power mode */
	if (ts->lowpower_status == TO_LOWPOWER_MODE) {
		w_data[0] = (ts->lowpower_mode &
			     SEC_TS_MODE_LOWPOWER_FLAG) >> 1;
		ret = sec_ts_write(ts, SEC_TS_CMD_WAKEUP_GESTURE_MODE,
				   (u8 *)&w_data[0], 1);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				"%s: Failed to send command(0x%x)",
				__func__, SEC_TS_CMD_WAKEUP_GESTURE_MODE);

		w_data[0] = TO_LOWPOWER_MODE;
		ret = sec_ts_write(ts, SEC_TS_CMD_SET_POWER_MODE,
				   (u8 *)&w_data[0], 1);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				"%s: Failed to send command(0x%x)",
				__func__, SEC_TS_CMD_SET_POWER_MODE);

		sec_ts_delay(50);

		if (ts->lowpower_mode & SEC_TS_MODE_CUSTOMLIB_AOD) {
			int i, ret;
			u8 data[10] = {0x02, 0};

			for (i = 0; i < 4; i++) {
				data[i * 2 + 2] = ts->rect_data[i] & 0xFF;
				data[i * 2 + 3] =
					(ts->rect_data[i] >> 8) & 0xFF;
			}

			ret = ts->sec_ts_write(ts,
					SEC_TS_CMD_CUSTOMLIB_WRITE_PARAM,
					&data[0], 10);
			if (ret < 0)
				input_err(true, &ts->client->dev,
					"%s: Failed to write offset\n",
					__func__);

			ret = ts->sec_ts_write(ts,
				SEC_TS_CMD_CUSTOMLIB_NOTIFY_PACKET, NULL, 0);
			if (ret < 0)
				input_err(true, &ts->client->dev,
					"%s: Failed to send notify\n",
					__func__);

		}

	} else {

		sec_ts_set_grip_type(ts, ONLY_EDGE_HANDLER);

		if (ts->dex_mode) {
			input_info(true, &ts->client->dev,
				   "%s: set dex mode\n", __func__);
			ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_DEX_MODE,
					       &ts->dex_mode, 1);
			if (ret < 0)
				input_err(true, &ts->client->dev,
					"%s: failed to set dex mode %x\n",
					__func__, ts->dex_mode);
		}

		if (ts->brush_mode) {
			input_info(true, &ts->client->dev,
				   "%s: set brush mode\n", __func__);
			ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_BRUSH_MODE,
					       &ts->brush_mode, 1);
			if (ret < 0)
				input_err(true, &ts->client->dev,
					"%s: failed to set brush mode\n",
					__func__);
		}

		if (ts->touchable_area) {
			input_info(true, &ts->client->dev,
				   "%s: set 16:9 mode\n", __func__);
			ret = ts->sec_ts_write(ts,
					SEC_TS_CMD_SET_TOUCHABLE_AREA,
					&ts->touchable_area, 1);
			if (ret < 0)
				input_err(true, &ts->client->dev,
					"%s: failed to set 16:9 mode\n",
					__func__);
		}

	}
}

#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
/* Update a state machine used to toggle control of the touch IC's motion
 * filter.
 */
static void update_motion_filter(struct sec_ts_data *ts, unsigned long touch_id)
{
	/* Motion filter timeout, in milliseconds */
	const u32 mf_timeout_ms = 500;
	u8 next_state;
	/* Count the active touches */
	u8 touches = hweight32(touch_id);

	if (ts->use_default_mf)
		return;

	/* Determine the next filter state. The motion filter is enabled by
	 * default and it is disabled while a single finger is touching the
	 * screen. If another finger is touched down or if a timeout expires,
	 * the motion filter is reenabled and remains enabled until all fingers
	 * are lifted.
	 */
	next_state = ts->mf_state;
	switch (ts->mf_state) {
	case SEC_TS_MF_FILTERED:
		if (touches == 1) {
			next_state = SEC_TS_MF_UNFILTERED;
			ts->mf_downtime = ktime_get();
		}
		break;
	case SEC_TS_MF_UNFILTERED:
		if (touches == 0) {
			next_state = SEC_TS_MF_FILTERED;
		} else if (touches > 1 ||
			   ktime_after(ktime_get(),
				       ktime_add_ms(ts->mf_downtime,
						    mf_timeout_ms))) {
			next_state = SEC_TS_MF_FILTERED_LOCKED;
		}
		break;
	case SEC_TS_MF_FILTERED_LOCKED:
		if (touches == 0)
			next_state = SEC_TS_MF_FILTERED;
		break;
	}

	/* Send command to update filter state */
	if ((next_state == SEC_TS_MF_UNFILTERED) !=
	    (ts->mf_state == SEC_TS_MF_UNFILTERED)) {
		int ret;
		u8 para;

		pr_debug("%s: setting motion filter = %s.\n", __func__,
			 (next_state == SEC_TS_MF_UNFILTERED) ?
			 "false" : "true");
		para = (next_state == SEC_TS_MF_UNFILTERED) ? 0x01 : 0x00;
		ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_CONT_REPORT,
				       &para, 1);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
			 "%s: write reg %#x para %#x failed, returned %i\n",
			__func__, SEC_TS_CMD_SET_CONT_REPORT, para, ret);
		}
	}
	ts->mf_state = next_state;
}

static bool read_heatmap_raw(struct v4l2_heatmap *v4l2)
{
	struct sec_ts_data *ts = container_of(v4l2, struct sec_ts_data, v4l2);
	struct sec_ts_plat_data *pdata = ts->plat_data;
	int result;
	int max_x = v4l2->format.width;
	int max_y = v4l2->format.height;

	if (ts->v4l2_mutual_strength_updated &&
	    ts->mutual_strength_heatmap.size_x == max_x &&
	    ts->mutual_strength_heatmap.size_y == max_y) {
		memcpy(v4l2->frame, ts->mutual_strength_heatmap.data,
		       max_x * max_y * 2);
		ts->v4l2_mutual_strength_updated = false;
		return true;
	}

	if (ts->tsp_dump_lock == 1) {
		input_info(true, &ts->client->dev,
			"%s: drop this because raw data reading by others\n",
			__func__);
		return false;
	}

	if (pdata->heatmap_mode == HEATMAP_PARTIAL) {
		strength_t heatmap_value;
		int heatmap_x, heatmap_y;
		/* index for through the heatmap buffer read over the bus */
		unsigned int local_i;
		/* final position of the heatmap value in the full frame */
		unsigned int frame_i;
		unsigned int num_elements;
		u8 enable;
		struct heatmap_report report = {0};

		if (!pdata->is_heatmap_enabled) {
			result = sec_ts_read(ts,
				SEC_TS_CMD_HEATMAP_ENABLE, &enable, 1);
			if (result < 0) {
				input_err(true, &ts->client->dev,
					  "%s: read reg %#x failed, returned %i\n",
					  __func__,
					  SEC_TS_CMD_HEATMAP_ENABLE, result);
				return false;
			}

			if (!enable) {
				enable = 1;
				result = sec_ts_write(ts,
					SEC_TS_CMD_HEATMAP_ENABLE, &enable, 1);
				if (result < 0)
					input_err(true, &ts->client->dev,
						"%s: enable local heatmap failed, returned %i\n",
						__func__, result);
				else
					pdata->is_heatmap_enabled = true;
				/*
				 * After local heatmap enabled, it takes
				 * `1/SCAN_RATE` time to make data ready. But,
				 * we don't want to wait here to cause
				 * overhead. Just drop this and wait for next
				 * reading.
				 */
				return false;
			} else {
				ts->plat_data->is_heatmap_enabled = true;
			}
		}

		result = sec_ts_read(ts, SEC_TS_CMD_HEATMAP_READ,
			(uint8_t *) &report, sizeof(report));
		if (result < 0) {
			input_err(true, &ts->client->dev,
				 "%s: read failed, returned %i\n",
				__func__, result);
			return false;
		}

		num_elements = report.size_x * report.size_y;
		if (num_elements > LOCAL_HEATMAP_WIDTH * LOCAL_HEATMAP_HEIGHT) {
			input_err(true, &ts->client->dev,
				"Unexpected heatmap size: %i x %i",
				report.size_x, report.size_y);
				return false;
		}

		/*
		 * Set all to zero, will only write to non-zero locations
		 * in the loop.
		 */
		memset(v4l2->frame, 0, v4l2->format.sizeimage);
		/* populate the data buffer, rearranging into final locations */
		for (local_i = 0; local_i < num_elements; local_i++) {
			/* big-endian order raw data into heatmap data type */
			be16_to_cpus(&report.data[local_i]);
			heatmap_value = report.data[local_i];

			if (heatmap_value == 0) {
				/*
				 * Already initialized to zero. More
				 * importantly, samples around edges may go out
				 * of bounds.
				 * If their value is zero, this is ok.
				 */
				continue;
			}
			heatmap_x = report.offset_x + (local_i % report.size_x);
			heatmap_y = report.offset_y + (local_i / report.size_x);

			if (heatmap_x < 0 || heatmap_x >= max_x ||
				heatmap_y < 0 || heatmap_y >= max_y) {
				input_err(true, &ts->client->dev,
					"Invalid x or y: (%i, %i), value=%i, ending loop\n",
					heatmap_x, heatmap_y,
					heatmap_value);
					return false;
			}
			frame_i = heatmap_y * max_x + heatmap_x;
			v4l2->frame[frame_i] = heatmap_value;
		}
	} else if (pdata->heatmap_mode == HEATMAP_FULL) {
		int i, j, index = 0;
		int ret = 0;
		u8 type;

		if (!ts->heatmap_buff) {
			ts->heatmap_buff = kmalloc(
				sizeof(strength_t) * max_x * max_y, GFP_KERNEL);
			if (!ts->heatmap_buff) {
				input_err(true, &ts->client->dev,
				"%s: alloc heatmap_buff failed\n", __func__);
				return false;
			}
		}

		ret = sec_ts_read(ts,
			SEC_TS_CMD_MUTU_RAW_TYPE, &ts->ms_frame_type, 1);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				"%s: read rawdata type failed\n",
				__func__);
			return false;
		}

		/* Check raw type is TYPE_SIGNAL_DATA */
		if (ts->ms_frame_type != TYPE_SIGNAL_DATA) {
			input_info(true, &ts->client->dev,
				"%s: ms_frame_type change from %#x\n",
				__func__, ts->ms_frame_type);

			/* Check raw type is TYPE_INVALID_DATA */
			if (ts->ms_frame_type != TYPE_INVALID_DATA) {
				type = TYPE_INVALID_DATA;
				ret = sec_ts_write(ts,
					SEC_TS_CMD_MUTU_RAW_TYPE, &type, 1);
				if (ret < 0) {
					input_err(true, &ts->client->dev,
						"%s: recover rawdata type failed\n",
						__func__);
					return false;
				}
				ts->ms_frame_type = type;
			}

			/* Set raw type to TYPE_SIGNAL_DATA */
			type = TYPE_SIGNAL_DATA;
			ret = sec_ts_write(ts, SEC_TS_CMD_MUTU_RAW_TYPE,
				&type, 1);
			if (ret < 0) {
				input_err(true, &ts->client->dev,
					"%s: Set rawdata type failed\n",
					__func__);
				return false;
			}
			ts->ms_frame_type = type;

			/*
			 * If raw type change, need to wait 50 ms to read data
			 * back. But, we don't wanto to wait here to cause
			 * overhead. Just drop this and wait for next reading.
			 */
			return false;
		}

		ret = sec_ts_read_heap(ts, SEC_TS_READ_TOUCH_RAWDATA,
			(u8 *)ts->heatmap_buff,
			sizeof(strength_t) * max_x * max_y);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				"%s: Read delta frame failed\n", __func__);
			return false;
		}

		/* big-endian order raw data into heatmap data type */
		for (i = max_y - 1; i >= 0; i--)
			for (j = max_x - 1; j >= 0 ; j--)
				v4l2->frame[index++] = be16_to_cpup(
					ts->heatmap_buff + (j * max_y) + i);
	} else
		return false;

	return true;
}
#endif

#ifdef SEC_TS_SUPPORT_CUSTOMLIB
/* WARNING: touch_offload does not currently support the custom library
 * interface!
 * TODO: when custom library support is enabled, ensure that the output is
 * routed through touch_offload.
 */
static void sec_ts_handle_lib_status_event(struct sec_ts_data *ts,
				struct sec_ts_event_status *p_event_status)
{
	if ((p_event_status->stype == TYPE_STATUS_EVENT_CUSTOMLIB_INFO) &&
	    (p_event_status->status_id == SEC_TS_EVENT_CUSTOMLIB_FORCE_KEY)) {
		if (ts->power_status == SEC_TS_STATE_POWER_ON) {
			if (p_event_status->status_data_1 &
			    SEC_TS_CUSTOMLIB_EVENT_PRESSURE_TOUCHED) {
				ts->all_force_count++;
				ts->scrub_id =
					CUSTOMLIB_EVENT_TYPE_PRESSURE_TOUCHED;
			} else {
				if (ts->scrub_id ==
				    CUSTOMLIB_EVENT_TYPE_AOD_HOMEKEY_PRESS) {
					input_report_key(ts->input_dev,
							 KEY_HOMEPAGE,
							 0);
					ts->scrub_id =
				CUSTOMLIB_EVENT_TYPE_AOD_HOMEKEY_RELEASE;
				} else {
					ts->scrub_id =
					CUSTOMLIB_EVENT_TYPE_PRESSURE_RELEASED;
				}
			}

			input_report_key(ts->input_dev,
					 KEY_BLACK_UI_GESTURE, 1);
		} else {
			if (p_event_status->status_data_1 &
			    SEC_TS_CUSTOMLIB_EVENT_PRESSURE_RELEASED) {
				input_report_key(ts->input_dev,
						 KEY_HOMEPAGE, 0);
				input_report_key(ts->input_dev,
						 KEY_BLACK_UI_GESTURE, 1);
				ts->scrub_id =
				CUSTOMLIB_EVENT_TYPE_AOD_HOMEKEY_RLS_NO_HAPTIC;
				input_sync(ts->input_dev);
				haptic_homekey_release();
			} else {
				input_report_key(ts->input_dev,
						 KEY_HOMEPAGE, 1);
				input_sync(ts->input_dev);
				ts->scrub_id =
					CUSTOMLIB_EVENT_TYPE_AOD_HOMEKEY_PRESS;
				haptic_homekey_press();
				ts->all_force_count++;
			}
		}

		ts->scrub_x =
			((p_event_status->status_data_4 >> 4) & 0xF) << 8 |
			(p_event_status->status_data_3 & 0xFF);
		ts->scrub_y =
			((p_event_status->status_data_4 >> 0) & 0xF) << 8 |
			(p_event_status->status_data_2 & 0xFF);

		input_info(true, &ts->client->dev, "%s: PRESSURE[%d]\n",
			   __func__, ts->scrub_id);

		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, KEY_BLACK_UI_GESTURE, 0);
	}
}
#endif


#ifdef SEC_TS_HC_KFIFO_LEN
inline void sec_ts_hc_update_and_push(struct sec_ts_data *ts, struct sec_ts_health_check *hc)
{
	hc->int_ktime = ts->timestamp;
	hc->int_idx = ts->int_cnt;
	hc->coord_idx = ts->coord_event_cnt;
	hc->status_idx = ts->status_event_cnt;
	hc->active_bit = ts->tid_touch_state;

	if (kfifo_is_full(&hc_fifo))
		kfifo_skip(&hc_fifo);
	kfifo_in(&hc_fifo, hc, 1);
}

inline void sec_ts_hc_dump(struct sec_ts_data *ts)
{
	int i;
	s64 delta;
	s64 sec_delta;
	u32 ms_delta;
	ktime_t current_time = ktime_get();
	struct sec_ts_health_check last_hc[SEC_TS_HC_KFIFO_LEN];

	kfifo_out_peek(&hc_fifo, last_hc, kfifo_size(&hc_fifo));
	for (i = 0 ; i < ARRAY_SIZE(last_hc) ; i++) {
		sec_delta = 0;
		ms_delta = 0;
		delta = ktime_ms_delta(current_time, last_hc[i].int_ktime);
		if (delta > 0)
			sec_delta = div_u64_rem(delta, MSEC_PER_SEC, &ms_delta);

		input_info(true, &ts->client->dev,
			"dump-int: #%llu(%llu.%u): S#%llu%s C#%llu(0x%lx)%s.\n",
			last_hc[i].int_idx,
			sec_delta, ms_delta,
			last_hc[i].status_idx,
			(last_hc[i].status_updated) ? "(+)" : "   ",
			last_hc[i].coord_idx,
			last_hc[i].active_bit,
			(last_hc[i].coord_updated) ? "(+)" : ""
			);
	}
}
#else
#define sec_ts_hc_update_and_push(ts, hc) do {} while (0)
#define sec_ts_hc_dump(ts) do {} while (0)
#endif /* #ifdef SEC_TS_HC_KFIFO_LEN */

#ifdef SEC_TS_DEBUG_KFIFO_LEN
inline void sec_ts_kfifo_push_coord(struct sec_ts_data *ts, u8 slot)
{
	if (slot < MAX_SUPPORT_TOUCH_COUNT) {
		/*
		 * Use kfifo as circular buffer by skipping one element
		 * when fifo is full.
		 */
		if (kfifo_is_full(&debug_fifo))
			kfifo_skip(&debug_fifo);
		kfifo_in(&debug_fifo, &ts->coord[slot], 1);
	}
}

inline void sec_ts_kfifo_pop_all_coords(struct sec_ts_data *ts)
{
	/*
	 * Keep coords without pop-out to support different timing
	 * print-out by each caller.
	 */
	kfifo_out_peek(&debug_fifo, last_coord, kfifo_size(&debug_fifo));
}

inline void sec_ts_debug_dump(struct sec_ts_data *ts)
{
	int i;
	s64 delta;
	s64 sec_longest_duration;
	u32 ms_longest_duration;
	s64 sec_delta_down;
	u32 ms_delta_down;
	s64 sec_delta_duration;
	u32 ms_delta_duration;
	s32 px_delta_x, px_delta_y;
	ktime_t current_time = ktime_get();

	sec_longest_duration = div_u64_rem(ts->longest_duration,
				MSEC_PER_SEC, &ms_longest_duration);

	sec_ts_kfifo_pop_all_coords(ts);
	for (i = 0 ; i < ARRAY_SIZE(last_coord) ; i++) {
		if (last_coord[i].action == SEC_TS_COORDINATE_ACTION_NONE) {
			input_dbg(true, &ts->client->dev,
				"dump: #%d: N/A!\n", last_coord[i].id);
			continue;
		}
		sec_delta_down = -1;
		ms_delta_down = 0;
		/* calculate the delta of finger down from current time. */
		delta = ktime_ms_delta(current_time, last_coord[i].ktime_pressed);
		if (delta > 0)
			sec_delta_down = div_u64_rem(delta, MSEC_PER_SEC, &ms_delta_down);

		/* calculate the delta of finger duration between finger up and down. */
		sec_delta_duration = -1;
		ms_delta_duration = 0;
		px_delta_x = 0;
		px_delta_y = 0;
		if (last_coord[i].action == SEC_TS_COORDINATE_ACTION_RELEASE) {
			delta = ktime_ms_delta(last_coord[i].ktime_released,
					last_coord[i].ktime_pressed);
			if (delta > 0) {
				sec_delta_duration = div_u64_rem(delta, MSEC_PER_SEC,
									&ms_delta_duration);
				px_delta_x = last_coord[i].x - last_coord[i].x_pressed;
				px_delta_y = last_coord[i].y - last_coord[i].y_pressed;
			}
		}
		input_info(true, &ts->client->dev,
			"dump: #%d: %lld.%u(%lld.%u) D(%d, %d).\n",
			last_coord[i].id,
			sec_delta_down, ms_delta_down,
			sec_delta_duration, ms_delta_duration,
			px_delta_x, px_delta_y);
		input_dbg(true, &ts->client->dev,
			"dump-dbg: #%d: (%d, %d) (%d, %d).\n",
			last_coord[i].id,
			last_coord[i].x_pressed, last_coord[i].y_pressed,
			last_coord[i].x, last_coord[i].y);
	}
	input_info(true, &ts->client->dev,
		"dump: i/o %u, comm %u, reset %u, longest %lld.%u.\n",
		ts->io_err_count, ts->comm_err_count, ts->hw_reset_count,
		sec_longest_duration, ms_longest_duration);
	input_info(true, &ts->client->dev,
		"dump: cnt %u, active %u, wet %u, palm %u.\n",
		ts->pressed_count, ts->touch_count, ts->wet_count, ts->palm_count);
}
#else
#define sec_ts_kfifo_push_coord(ts, slot) do {} while (0)
#define sec_ts_kfifo_pop_all_coords(ts) do {} while (0)
#define sec_ts_debug_dump(ts) do {} while (0)
#endif /* #ifdef SEC_TS_DEBUG_KFIFO_LEN */

static void sec_ts_handle_coord_event(struct sec_ts_data *ts,
				struct sec_ts_event_coordinate *p_event_coord)
{
	u8 t_id;

	if (ts->input_closed) {
		input_err(true, &ts->client->dev, "%s: device is closed\n",
			__func__);
		return;
	}

	t_id = (p_event_coord->tid - 1);

	if (t_id < MAX_SUPPORT_TOUCH_COUNT + MAX_SUPPORT_HOVER_COUNT) {
		ts->coord[t_id].id = t_id;
		ts->coord[t_id].action = p_event_coord->tchsta;
		ts->coord[t_id].x = (p_event_coord->x_11_4 << 4) |
					(p_event_coord->x_3_0);
		ts->coord[t_id].y = (p_event_coord->y_11_4 << 4) |
					(p_event_coord->y_3_0);
		ts->coord[t_id].z = p_event_coord->z &
					SEC_TS_PRESSURE_MAX;
		ts->coord[t_id].ttype = p_event_coord->ttype_3_2 << 2 |
					p_event_coord->ttype_1_0 << 0;
		ts->coord[t_id].major = p_event_coord->major *
						ts->plat_data->mm2px;
		ts->coord[t_id].minor = p_event_coord->minor *
						ts->plat_data->mm2px;

		if (!ts->coord[t_id].palm &&
			(ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_PALM))
			ts->palm_count++;

		ts->coord[t_id].palm =
			(ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_PALM);

		ts->coord[t_id].grip =
			(ts->coord[t_id].ttype == SEC_TS_TOUCHTYPE_GRIP);

		ts->coord[t_id].left_event = p_event_coord->left_event;

		if (ts->coord[t_id].z <= 0)
			ts->coord[t_id].z = 1;

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
		ts->offload.coords[t_id].x = ts->coord[t_id].x;
		ts->offload.coords[t_id].y = ts->coord[t_id].y;
		ts->offload.coords[t_id].major = ts->coord[t_id].major;
		ts->offload.coords[t_id].minor = ts->coord[t_id].minor;
		ts->offload.coords[t_id].pressure = ts->coord[t_id].z;
		ts->offload.coords[t_id].rotation = 0;
#endif

		if ((ts->coord[t_id].ttype ==
		     SEC_TS_TOUCHTYPE_NORMAL) ||
		    (ts->coord[t_id].ttype ==
		     SEC_TS_TOUCHTYPE_PALM) ||
		    (ts->coord[t_id].ttype ==
		     SEC_TS_TOUCHTYPE_GRIP) ||
		    (ts->coord[t_id].ttype ==
		     SEC_TS_TOUCHTYPE_WET) ||
		    (ts->coord[t_id].ttype ==
		     SEC_TS_TOUCHTYPE_GLOVE)) {

			if (ts->coord[t_id].action == SEC_TS_COORDINATE_ACTION_RELEASE) {
				s64 ms_delta;

				ts->coord[t_id].ktime_released = ktime_get();
				ms_delta = ktime_ms_delta(ts->coord[t_id].ktime_released,
							ts->coord[t_id].ktime_pressed);
				if (ts->longest_duration < ms_delta)
					ts->longest_duration = ms_delta;

				if (ts->touch_count > 0)
					ts->touch_count--;
				if (ts->touch_count == 0 ||
					ts->tid_touch_state == 0) {
					ts->check_multi = 0;
				}
				__clear_bit(t_id, &ts->tid_palm_state);
				__clear_bit(t_id, &ts->tid_grip_state);
				__clear_bit(t_id, &ts->tid_touch_state);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
				ts->offload.coords[t_id].status =
					COORD_STATUS_INACTIVE;
				if (!ts->offload.offload_running) {
#endif
				input_mt_slot(ts->input_dev, t_id);
				if (ts->plat_data->support_mt_pressure)
					input_report_abs(ts->input_dev,
						ABS_MT_PRESSURE, 0);
				input_mt_report_slot_state(ts->input_dev,
					MT_TOOL_FINGER, 0);

				if (ts->touch_count == 0 ||
					ts->tid_touch_state == 0) {
					input_report_key(ts->input_dev,
						BTN_TOUCH, 0);
					input_report_key(ts->input_dev,
						BTN_TOOL_FINGER, 0);
				}
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
				}
#endif
			} else if (ts->coord[t_id].action ==
					SEC_TS_COORDINATE_ACTION_PRESS) {
				ts->coord[t_id].ktime_pressed = ktime_get();
				ts->pressed_count++;
				ts->touch_count++;
				if ((ts->touch_count > 4) &&
					(ts->check_multi == 0)) {
					ts->check_multi = 1;
					ts->multi_count++;
				}
				ts->all_finger_count++;

				ts->coord[t_id].x_pressed = ts->coord[t_id].x;
				ts->coord[t_id].y_pressed = ts->coord[t_id].y;
				ts->max_z_value = max_t(unsigned int,
							ts->coord[t_id].z,
							ts->max_z_value);
				ts->min_z_value = min_t(unsigned int,
							ts->coord[t_id].z,
							ts->min_z_value);
				ts->sum_z_value +=
						(unsigned int)ts->coord[t_id].z;

				__set_bit(t_id, &ts->tid_touch_state);
				__clear_bit(t_id, &ts->tid_palm_state);
				__clear_bit(t_id, &ts->tid_grip_state);
				if (ts->coord[t_id].palm)
					__set_bit(t_id, &ts->tid_palm_state);
				else if (ts->coord[t_id].grip)
					__set_bit(t_id, &ts->tid_grip_state);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
				ts->offload.coords[t_id].status =
					COORD_STATUS_FINGER;
				if (!ts->offload.offload_running) {
#endif
				input_mt_slot(ts->input_dev, t_id);
				if (ts->coord[t_id].palm)
					input_mt_report_slot_state(
						ts->input_dev, MT_TOOL_PALM, 1);
				else if (ts->coord[t_id].grip)
					input_mt_report_slot_state(
						ts->input_dev, MT_TOOL_PALM, 1);
				else
					input_mt_report_slot_state(
						ts->input_dev,
						MT_TOOL_FINGER, 1);

				input_report_key(ts->input_dev, BTN_TOUCH, 1);
				input_report_key(ts->input_dev,
							BTN_TOOL_FINGER, 1);

				input_report_abs(ts->input_dev,
					ABS_MT_POSITION_X, ts->coord[t_id].x);
				input_report_abs(ts->input_dev,
					ABS_MT_POSITION_Y, ts->coord[t_id].y);
				input_report_abs(ts->input_dev,
						ABS_MT_TOUCH_MAJOR,
						ts->coord[t_id].major);
				input_report_abs(ts->input_dev,
						ABS_MT_TOUCH_MINOR,
						ts->coord[t_id].minor);
#ifdef ABS_MT_CUSTOM
				if (ts->brush_mode)
					input_report_abs(ts->input_dev,
						ABS_MT_CUSTOM,
						(ts->coord[t_id].z << 1) |
							ts->coord[t_id].palm);
				else
					input_report_abs(ts->input_dev,
						ABS_MT_CUSTOM,
						(BRUSH_Z_DATA << 1) |
							ts->coord[t_id].palm);
#endif
				if (ts->plat_data->support_mt_pressure)
					input_report_abs(ts->input_dev,
						ABS_MT_PRESSURE,
						ts->coord[t_id].z);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
				}
#endif
			} else if (ts->coord[t_id].action ==
					SEC_TS_COORDINATE_ACTION_MOVE) {

				ts->coord[t_id].mcount++;

#ifdef SW_GLOVE
				if ((ts->coord[t_id].ttype ==
					SEC_TS_TOUCHTYPE_GLOVE) &&
				    !ts->touchkey_glove_mode_status) {
					ts->touchkey_glove_mode_status = true;
				} else if ((ts->coord[t_id].ttype !=
						SEC_TS_TOUCHTYPE_GLOVE) &&
					   ts->touchkey_glove_mode_status) {
					ts->touchkey_glove_mode_status = false;
				}
#endif
				__set_bit(t_id, &ts->tid_touch_state);
				__clear_bit(t_id, &ts->tid_palm_state);
				__clear_bit(t_id, &ts->tid_grip_state);
				if (ts->coord[t_id].palm)
					__set_bit(t_id, &ts->tid_palm_state);
				else if (ts->coord[t_id].grip)
					__set_bit(t_id, &ts->tid_grip_state);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
				ts->offload.coords[t_id].status =
					COORD_STATUS_FINGER;
				if (!ts->offload.offload_running) {
#endif
#ifdef SW_GLOVE
				if ((ts->coord[t_id].ttype ==
					SEC_TS_TOUCHTYPE_GLOVE) &&
				    !ts->touchkey_glove_mode_status) {
					input_report_switch(ts->input_dev,
						SW_GLOVE, 1);
				} else if ((ts->coord[t_id].ttype !=
						SEC_TS_TOUCHTYPE_GLOVE) &&
					   ts->touchkey_glove_mode_status) {
					input_report_switch(ts->input_dev,
						SW_GLOVE, 0);
				}
#endif
				input_mt_slot(ts->input_dev, t_id);
				if (ts->coord[t_id].palm)
					input_mt_report_slot_state(
						ts->input_dev, MT_TOOL_PALM, 1);
				else if (ts->coord[t_id].grip)
					input_mt_report_slot_state(
						ts->input_dev, MT_TOOL_PALM, 1);
				else
					input_mt_report_slot_state(
						ts->input_dev,
						MT_TOOL_FINGER, 1);

				input_report_key(ts->input_dev, BTN_TOUCH, 1);
				input_report_key(ts->input_dev,
							BTN_TOOL_FINGER, 1);

				input_report_abs(ts->input_dev,
					ABS_MT_POSITION_X, ts->coord[t_id].x);
				input_report_abs(ts->input_dev,
					ABS_MT_POSITION_Y, ts->coord[t_id].y);
				input_report_abs(ts->input_dev,
						ABS_MT_TOUCH_MAJOR,
						ts->coord[t_id].major);
				input_report_abs(ts->input_dev,
						ABS_MT_TOUCH_MINOR,
						ts->coord[t_id].minor);
#ifdef ABS_MT_CUSTOM
				if (ts->brush_mode)
					input_report_abs(ts->input_dev,
						ABS_MT_CUSTOM,
						(ts->coord[t_id].z << 1) |
							ts->coord[t_id].palm);
				else
					input_report_abs(ts->input_dev,
						ABS_MT_CUSTOM,
						(BRUSH_Z_DATA << 1) |
							ts->coord[t_id].palm);
#endif
				if (ts->plat_data->support_mt_pressure)
					input_report_abs(ts->input_dev,
							ABS_MT_PRESSURE,
							ts->coord[t_id].z);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
				}
#endif
			} else
				input_dbg(true, &ts->client->dev,
					"%s: do not support coordinate action(%d)\n",
					__func__, ts->coord[t_id].action);
		} else
			input_dbg(true, &ts->client->dev,
				"%s: do not support coordinate type(%d)\n",
				__func__, ts->coord[t_id].ttype);
	} else
		input_err(true, &ts->client->dev,
				"%s: tid(%d) is out of range\n",
				__func__, t_id);

	if (t_id < MAX_SUPPORT_TOUCH_COUNT + MAX_SUPPORT_HOVER_COUNT) {

		if (ts->coord[t_id].action == SEC_TS_COORDINATE_ACTION_PRESS) {
			input_dbg(false, &ts->client->dev,
				"%s[P] tID: %d x: %d y: %d z: %d major: %d minor: %d tc: %u type: %X\n",
				ts->dex_name,
				t_id, ts->coord[t_id].x,
				ts->coord[t_id].y, ts->coord[t_id].z,
				ts->coord[t_id].major,
				ts->coord[t_id].minor,
				ts->touch_count,
				ts->coord[t_id].ttype);

		} else if (ts->coord[t_id].action ==
			   SEC_TS_COORDINATE_ACTION_RELEASE) {
			sec_ts_kfifo_push_coord(ts, t_id);
			input_dbg(false, &ts->client->dev,
				"%s[R] tID: %d mc: %d tc: %u lx: %d ly: %d v: %02X%02X cal: %02X(%02X) id(%d,%d)\n",
				ts->dex_name,
				t_id, ts->coord[t_id].mcount,
				ts->touch_count,
				ts->coord[t_id].x, ts->coord[t_id].y,
				ts->plat_data->img_version_of_ic[2],
				ts->plat_data->img_version_of_ic[3],
				ts->cal_status, ts->nv, ts->tspid_val,
				ts->tspicid_val);

			ts->coord[t_id].mcount = 0;
		}
	}
}

#ifdef SEC_TS_SUPPORT_CUSTOMLIB
static void sec_ts_handle_gesture_event(struct sec_ts_data *ts,
				struct sec_ts_gesture_status *p_gesture_status)
{
	if ((p_gesture_status->eid == 0x02) &&
	    (p_gesture_status->stype == 0x00)) {
		u8 customlib[3] = { 0 };

		ret = sec_ts_read_from_customlib(ts, customlib, 3);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				"%s: fail to read custom library data\n",
				__func__);

		input_info(true, &ts->client->dev,
			"%s: Custom Library, %x, %x, %x\n",
			__func__, customlib[0], customlib[1], customlib[2]);

		if (p_gesture_status->gesture_id == SEC_TS_GESTURE_CODE_SPAY ||
		    p_gesture_status->gesture_id ==
			SEC_TS_GESTURE_CODE_DOUBLE_TAP) {
			/* will be fixed to data structure */
			if (customlib[1] & SEC_TS_MODE_CUSTOMLIB_AOD) {
				u8 data[5] = { 0x0A, 0x00, 0x00, 0x00, 0x00 };

				ret = sec_ts_read_from_customlib(ts, data, 5);
				if (ret < 0)
					input_err(true, &ts->client->dev,
						"%s: fail to read custom library data\n",
						__func__);

				if (data[4] & SEC_TS_AOD_GESTURE_DOUBLETAB)
					ts->scrub_id =
					CUSTOMLIB_EVENT_TYPE_AOD_DOUBLETAB;

				ts->scrub_x = (data[1] & 0xFF) << 8 |
						(data[0] & 0xFF);
				ts->scrub_y = (data[3] & 0xFF) << 8 |
						(data[2] & 0xFF);
				input_info(true, &ts->client->dev,
					"%s: aod: %d\n",
					__func__, ts->scrub_id);
				ts->all_aod_tap_count++;
			}
			if (customlib[1] & SEC_TS_MODE_CUSTOMLIB_SPAY) {
				ts->scrub_id = CUSTOMLIB_EVENT_TYPE_SPAY;
				input_info(true, &ts->client->dev,
					"%s: SPAY: %d\n",
					__func__, ts->scrub_id);
				ts->all_spay_count++;
			}
			input_report_key(ts->input_dev,
					 KEY_BLACK_UI_GESTURE, 1);
			input_sync(ts->input_dev);
			input_report_key(ts->input_dev,
					 KEY_BLACK_UI_GESTURE, 0);
		}
	}
}
#endif

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)

static void sec_ts_populate_coordinate_channel(struct sec_ts_data *ts,
					struct touch_offload_frame *frame,
					int channel)
{
	int j;

	struct TouchOffloadDataCoord *dc =
		(struct TouchOffloadDataCoord *)frame->channel_data[channel];
	memset(dc, 0, frame->channel_data_size[channel]);
	dc->header.channel_type = TOUCH_DATA_TYPE_COORD;
	dc->header.channel_size = TOUCH_OFFLOAD_FRAME_SIZE_COORD;

	for (j = 0; j < MAX_COORDS; j++) {
		dc->coords[j].x = ts->offload.coords[j].x;
		dc->coords[j].y = ts->offload.coords[j].y;
		dc->coords[j].major = ts->offload.coords[j].major;
		dc->coords[j].minor = ts->offload.coords[j].minor;
		dc->coords[j].pressure = ts->offload.coords[j].pressure;
		dc->coords[j].rotation = ts->offload.coords[j].rotation;
		dc->coords[j].status = ts->offload.coords[j].status;
	}
}

static void sec_ts_update_v4l2_mutual_strength(struct sec_ts_data *ts,
					       uint32_t size_x,
					       uint32_t size_y,
					       int16_t *heatmap)
{
	if (!ts->mutual_strength_heatmap.data) {
		ts->mutual_strength_heatmap.data = devm_kmalloc(
			&ts->client->dev, size_x * size_y * 2, GFP_KERNEL);
		if (!ts->mutual_strength_heatmap.data) {
			input_err(true, &ts->client->dev,
				  "%s: kmalloc for mutual_strength_heatmap (%d) failed.\n",
				  __func__, size_x * size_y * 2);
		} else {
			ts->mutual_strength_heatmap.size_x = size_x;
			ts->mutual_strength_heatmap.size_y = size_y;
			input_info(true, &ts->client->dev,
				   "%s: kmalloc for mutual_strength_heatmap (%d).\n",
				   __func__, size_x * size_y * 2);
		}
	}

	if (ts->mutual_strength_heatmap.data) {
		if (ts->mutual_strength_heatmap.size_x == size_x &&
		    ts->mutual_strength_heatmap.size_y == size_y) {
			memcpy(ts->mutual_strength_heatmap.data,
			       heatmap, size_x * size_y * 2);
			ts->mutual_strength_heatmap.timestamp =
				ts->timestamp;
			ts->v4l2_mutual_strength_updated = true;
		} else {
			input_info(true, &ts->client->dev,
				   "%s: unmatched heatmap size (%d,%d) (%d,%d).\n",
				   __func__, size_x, size_y,
				   ts->mutual_strength_heatmap.size_x,
				   ts->mutual_strength_heatmap.size_y);
		}
	}
}

#define PTFLIB_ENCODED_COUNTER_OFFSET		0x00A8
#define PTFLIB_ENCODED_COUNTER_READ_SIZE	6
#define PTFLIB_ENCODED_DATA_READ_SIZE		338
static int sec_ts_populate_encoded_channel(struct sec_ts_data *ts,
					   struct touch_offload_frame *frame,
					   int channel)
{
	u32 heatmap_array_len = 0;
	u32 decoded_size = 0;
	u32 encoded_counter = 0;
	u16 read_src_offset = 0;
	int read_src_size = 0;
	u8 *r_data;
	u16 encoded_data_size = 0;
	u16 first_word = 0;
	int i;
	int x;
	int y;
	int ret = 0;
	ktime_t timestamp_read_start;
	ktime_t timestamp_read_end;
	struct TouchOffloadData2d *mutual_strength =
		(struct TouchOffloadData2d *)frame->channel_data[channel];

	mutual_strength->tx_size = ts->tx_count;
	mutual_strength->rx_size = ts->rx_count;
	mutual_strength->header.channel_type = frame->channel_type[channel];
	mutual_strength->header.channel_size =
		TOUCH_OFFLOAD_FRAME_SIZE_2D(mutual_strength->rx_size,
					    mutual_strength->tx_size);
	heatmap_array_len = ts->tx_count * ts->rx_count;

	if (!ts->encoded_buff) {
		ts->encoded_buff = kmalloc(
		    heatmap_array_len * 2 + PTFLIB_ENCODED_COUNTER_READ_SIZE,
		    GFP_KERNEL);
		if (!ts->encoded_buff) {
			input_err(true, &ts->client->dev,
				  "%s: kmalloc for encoded_buff failed.\n",
				  __func__);
			return -ENOMEM;
		}
	}

	/* Read encoded heatmap from customlib. */
	read_src_offset = PTFLIB_ENCODED_COUNTER_OFFSET;
	read_src_size = PTFLIB_ENCODED_COUNTER_READ_SIZE +
	    PTFLIB_ENCODED_DATA_READ_SIZE;
	r_data = (u8 *) ts->encoded_buff;
	r_data[0] = read_src_offset & 0xFF;
	r_data[1] = (read_src_offset >> 8) & 0xFF;
	timestamp_read_start = ktime_get();
	ret = sec_ts_read_from_customlib(ts, r_data, read_src_size);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: Read customlib failed, offset(0x%04X)) size(%d)\n",
			  __func__, read_src_offset, read_src_size);
		return -EIO;
	}
	timestamp_read_end = ktime_get();

	encoded_counter = le32_to_cpup((uint32_t *) r_data);
	encoded_data_size = le16_to_cpup((uint16_t *) &r_data[4]);
	first_word = le16_to_cpup((uint16_t *) &r_data[6]);

	if (encoded_counter == 0 || encoded_data_size == 0 ||
	    first_word == 0x8FFF ||
	    encoded_data_size > PTFLIB_ENCODED_DATA_READ_SIZE) {
		decoded_size = 0;
	} else {
		decoded_size = sec_ts_ptflib_decoder(ts, (u16 *) (r_data + 6),
						     encoded_data_size / 2,
						     (u16 *) ts->heatmap_buff,
						     heatmap_array_len);
	}

	ts->plat_data->encoded_frame_counter++;
	if (decoded_size != heatmap_array_len) {
		ts->plat_data->encoded_skip_counter++;
		input_info(true, &ts->client->dev,
			  "%s: %d (%d,0x%04X,0x%04X,%d) ts(%lld,%lld)\n",
			  __func__, encoded_counter,
			  encoded_data_size & 0x0FFF, encoded_data_size,
			  first_word, decoded_size,
			  ktime_us_delta(timestamp_read_start, ts->timestamp),
			  ktime_us_delta(timestamp_read_end,
					 timestamp_read_start));
		return -EIO;
	}

	i = 0;
	for (y = mutual_strength->rx_size - 1; y >= 0; y--)
		for (x = mutual_strength->tx_size - 1; x >= 0; x--)
			((uint16_t *) mutual_strength->data)[i++] =
			    ts->heatmap_buff[x * mutual_strength->rx_size + y];

	sec_ts_update_v4l2_mutual_strength(ts, mutual_strength->tx_size,
					   mutual_strength->rx_size,
					   (int16_t *) mutual_strength->data);

	return 0;
}

static void sec_ts_populate_mutual_channel(struct sec_ts_data *ts,
					struct touch_offload_frame *frame,
					int channel)
{
	uint32_t frame_index = 0;
	int32_t x, y;
	uint16_t heatmap_value;
	int ret = 0;
	u8 target_data_type, type;
	struct TouchOffloadData2d *mutual_strength =
		(struct TouchOffloadData2d *)frame->channel_data[channel];

	switch (frame->channel_type[channel] & ~TOUCH_SCAN_TYPE_MUTUAL) {
	case TOUCH_DATA_TYPE_RAW:
		target_data_type = TYPE_DECODED_DATA;
		break;
	case TOUCH_DATA_TYPE_FILTERED:
		target_data_type = TYPE_REMV_AMB_DATA;
		break;
	case TOUCH_DATA_TYPE_STRENGTH:
		target_data_type = TYPE_SIGNAL_DATA;
		break;
	case TOUCH_DATA_TYPE_BASELINE:
		target_data_type = TYPE_AMBIENT_DATA;
		break;
	}

	mutual_strength->tx_size = ts->tx_count;
	mutual_strength->rx_size = ts->rx_count;
	mutual_strength->header.channel_type = frame->channel_type[channel];
	mutual_strength->header.channel_size =
		TOUCH_OFFLOAD_FRAME_SIZE_2D(mutual_strength->rx_size,
					    mutual_strength->tx_size);

	ret = sec_ts_read(ts,
		SEC_TS_CMD_MUTU_RAW_TYPE, &ts->ms_frame_type, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			"%s: read rawdata type failed\n",
			__func__);
		return;
	}

	/* Check raw type is correct */
	if (ts->ms_frame_type != target_data_type) {
		input_info(true, &ts->client->dev,
			"%s: ms_frame_type change from %#x\n",
			__func__, ts->ms_frame_type);

		/* Check raw type is TYPE_INVALID_DATA */
		if (ts->ms_frame_type != TYPE_INVALID_DATA) {
			type = TYPE_INVALID_DATA;
			ret = sec_ts_write(ts,
				SEC_TS_CMD_MUTU_RAW_TYPE, &type, 1);
			if (ret < 0) {
				input_err(true, &ts->client->dev,
					"%s: recover rawdata type failed\n",
					__func__);
				return;
			}
			ts->ms_frame_type = type;
		}

		/* Set the targeted data type */
		ret = sec_ts_write(ts, SEC_TS_CMD_MUTU_RAW_TYPE,
			&target_data_type, 1);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				"%s: Set rawdata type failed\n",
				__func__);
			return;
		}
		ts->ms_frame_type = target_data_type;

		/*
		 * If raw type change, need to wait 50 ms to read data
		 * back. But, we don't wanto to wait here to cause
		 * overhead. Just drop this and wait for next reading.
		 */

		return;
	}

	ret = sec_ts_read_heap(ts, SEC_TS_READ_TOUCH_RAWDATA,
		(u8 *)ts->heatmap_buff,
		mutual_strength->header.channel_size);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			"%s: Read mutual frame failed\n", __func__);
		return;
	}

	for (y = mutual_strength->rx_size - 1; y >= 0; y--) {
		for (x = mutual_strength->tx_size - 1; x >= 0; x--) {
			heatmap_value =
			    ts->heatmap_buff[x * mutual_strength->rx_size + y];
			((uint16_t *)
			 mutual_strength->data)[frame_index++] =
			    be16_to_cpu(heatmap_value);
		}
	}

	if (target_data_type == TYPE_SIGNAL_DATA) {
		sec_ts_update_v4l2_mutual_strength(ts,
		    mutual_strength->tx_size, mutual_strength->rx_size,
		    (int16_t *) mutual_strength->data);
	}
}

static void sec_ts_populate_self_channel(struct sec_ts_data *ts,
					struct touch_offload_frame *frame,
					int channel)
{
	uint32_t frame_index = 0;
	int32_t x, y;
	uint16_t heatmap_value;
	int ret = 0;
	u8 target_data_type, type;
	struct TouchOffloadData1d *self_strength =
		(struct TouchOffloadData1d *)frame->channel_data[channel];

	switch (frame->channel_type[channel] & ~TOUCH_SCAN_TYPE_SELF) {
	case TOUCH_DATA_TYPE_RAW:
		target_data_type = TYPE_DECODED_DATA;
		break;
	case TOUCH_DATA_TYPE_FILTERED:
		target_data_type = TYPE_REMV_AMB_DATA;
		break;
	case TOUCH_DATA_TYPE_STRENGTH:
		target_data_type = TYPE_SIGNAL_DATA;
		break;
	case TOUCH_DATA_TYPE_BASELINE:
		target_data_type = TYPE_AMBIENT_DATA;
		break;
	}

	self_strength->tx_size = ts->tx_count;
	self_strength->rx_size = ts->rx_count;
	self_strength->header.channel_type = frame->channel_type[channel];
	self_strength->header.channel_size =
		TOUCH_OFFLOAD_FRAME_SIZE_1D(self_strength->rx_size,
					    self_strength->tx_size);

	ret = sec_ts_read(ts,
		SEC_TS_CMD_SELF_RAW_TYPE, &ts->ss_frame_type, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			"%s: read rawdata type failed\n",
			__func__);
		return;
	}

	/* Check raw type is TYPE_SIGNAL_DATA */
	if (ts->ss_frame_type != target_data_type) {
		input_info(true, &ts->client->dev,
			"%s: ss_frame_type change from %#x\n",
			__func__, ts->ss_frame_type);

		/* Check raw type is TYPE_INVALID_DATA */
		if (ts->ss_frame_type != TYPE_INVALID_DATA) {
			type = TYPE_INVALID_DATA;
			ret = sec_ts_write(ts,
				SEC_TS_CMD_SELF_RAW_TYPE, &type, 1);
			if (ret < 0) {
				input_err(true, &ts->client->dev,
					"%s: recover rawdata type failed\n",
					__func__);
				return;
			}
			ts->ss_frame_type = type;
		}

		/* Set the targeted data type */
		ret = sec_ts_write(ts, SEC_TS_CMD_SELF_RAW_TYPE,
			&target_data_type, 1);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				"%s: Set rawdata type failed\n",
				__func__);
			return;
		}
		ts->ss_frame_type = target_data_type;

		/*
		 * If raw type change, need to wait 50 ms to read data
		 * back. But, we don't wanto to wait here to cause
		 * overhead. Just drop this and wait for next reading.
		 */

		return;
	}

	ret = sec_ts_read_heap(ts, SEC_TS_READ_TOUCH_SELF_RAWDATA,
		(u8 *)ts->heatmap_buff,
		self_strength->header.channel_size);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			"%s: Read self frame failed\n", __func__);
		return;
	}

	for (x = self_strength->tx_size - 1; x >= 0; x--) {
		heatmap_value = ts->heatmap_buff[x];
		((uint16_t *)
		 self_strength->data)[frame_index++] =
		    be16_to_cpu(heatmap_value);
	}
	for (y = self_strength->rx_size - 1; y >= 0; y--) {
		heatmap_value = ts->heatmap_buff[self_strength->tx_size + y];
		((uint16_t *)
		 self_strength->data)[frame_index++] =
		    be16_to_cpu(heatmap_value);
	}
}

static void sec_ts_populate_driver_status_channel(struct sec_ts_data *ts,
					struct touch_offload_frame *frame,
					int channel)
{
	struct TouchOffloadDriverStatus *ds =
		(struct TouchOffloadDriverStatus *)frame->channel_data[channel];
	memset(ds, 0, frame->channel_data_size[channel]);
	ds->header.channel_type = (u32)CONTEXT_CHANNEL_TYPE_DRIVER_STATUS;
	ds->header.channel_size = sizeof(struct TouchOffloadDriverStatus);

	ds->contents.screen_state = 1;
	ds->screen_state = (ts->power_status == SEC_TS_STATE_POWER_ON) ? 1 : 0;

	ds->display_refresh_rate = ts->display_refresh_rate;
	ds->contents.display_refresh_rate = 1;
}

static void sec_ts_populate_frame(struct sec_ts_data *ts,
				struct touch_offload_frame *frame)
{
	static u64 index;
	int i;
	int retval = -1;
	const struct sec_ts_plat_data *pdata = ts->plat_data;

	frame->header.index = index++;
	frame->header.timestamp = ts->timestamp;

	if (!ts->heatmap_buff) {
		ts->heatmap_buff = kmalloc(
			ts->rx_count * ts->rx_count * 2, GFP_KERNEL);
	}

	/* Populate all channels */
	for (i = 0; i < frame->num_channels; i++) {
		u8 channel_type = frame->channel_type[i];

		if (channel_type == TOUCH_DATA_TYPE_COORD) {
			sec_ts_populate_coordinate_channel(ts, frame, i);
		} else if ((channel_type & TOUCH_SCAN_TYPE_MUTUAL) != 0) {
			if ((pdata->encoded_enable == ENCODED_ENABLE_ON) &&
			    ((channel_type & ~TOUCH_SCAN_TYPE_MUTUAL) ==
			    TOUCH_DATA_TYPE_STRENGTH))
				retval = sec_ts_populate_encoded_channel(
				    ts, frame, i);
			if (retval < 0)
				sec_ts_populate_mutual_channel(ts, frame, i);
		} else if ((channel_type & TOUCH_SCAN_TYPE_SELF) != 0) {
			sec_ts_populate_self_channel(ts, frame, i);
		} else if ((frame->channel_type[i] ==
			    CONTEXT_CHANNEL_TYPE_DRIVER_STATUS) != 0)
			sec_ts_populate_driver_status_channel(ts, frame, i);
		else if ((frame->channel_type[i] ==
			  CONTEXT_CHANNEL_TYPE_STYLUS_STATUS) != 0) {
			/* Stylus context is not required by this driver */
			input_err(true, &ts->client->dev,
				  "%s: Driver does not support stylus status",
				  __func__);
		}
	}
}

void sec_ts_enable_ptflib(struct sec_ts_data *ts, bool enable)
{
	struct sec_ts_plat_data *pdata = ts->plat_data;

	input_info(true, &ts->client->dev,
		"%s: enable %d.\n", __func__, enable);

	if (enable) {
		sec_ts_ptflib_reinit(ts);
		if (pdata->grip_prescreen_mode == GRIP_PRESCREEN_MODE_2) {
			sec_ts_ptflib_grip_prescreen_timeout(ts,
				pdata->grip_prescreen_timeout);
		}
		sec_ts_ptflib_grip_prescreen_enable(ts,
			pdata->grip_prescreen_mode);
	} else {
		sec_ts_ptflib_grip_prescreen_enable(ts, GRIP_PRESCREEN_OFF);
	}
}

int sec_ts_enable_fw_grip(struct sec_ts_data *ts, bool enable)
{
	struct sec_ts_plat_data *pdata = ts->plat_data;
	u8 value;
	int ret;
	int final_result = 0;

	input_info(true, &ts->client->dev,
		"%s: enable %d.\n", __func__, enable);

	/* Set grip */
	value = enable ? 0x1F : 0;
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_GRIP_DETEC, &value, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			 "%s: SEC_TS_CMD_SET_GRIP_DETEC failed with ret=%d\n",
			__func__, ret);
		final_result = ret;
	} else {
		/* Configure grip */
		if (enable) {
			u8 mm = 10;
			u8 px_lo = (mm * pdata->mm2px) & 0xFF;
			u8 px_hi = ((mm * pdata->mm2px) >> 8) & 0xFF;
			u8 long_press_zone[10] = {0x02, 0x00,	/* long press reject zone type */
						px_hi, px_lo,	/* left edge */
						0x00, 0x00,	/* top edge */
						px_hi, px_lo,	/* right edge */
						0x00, 0x00};	/* bottom edge */
			ret = ts->sec_ts_write(ts, SEC_TS_CMD_LONGPRESS_DROP_AREA,
					long_press_zone, sizeof(long_press_zone));
			if (ret < 0) {
				input_err(true, &ts->client->dev,
					"%s: SEC_TS_CMD_LONGPRESS_DROP_AREA failed with ret=%d\n",
					__func__, ret);
			}
		}
	}

	return final_result;
}

static void sec_ts_offload_set_running(struct sec_ts_data *ts, bool running)
{
	if (ts->offload.offload_running != running) {
		ts->offload.offload_running = running;
		if (running && ts->offload.config.filter_grip) {
			sec_ts_enable_fw_grip(ts, false);
			sec_ts_enable_ptflib(ts, true);
		} else {
			sec_ts_enable_fw_grip(ts, true);
			sec_ts_enable_ptflib(ts, false);
		}
	}
}

#endif /* CONFIG_TOUCHSCREEN_OFFLOAD */

#define FOD_CANCEL_EVENT_DELTA_TIME 500
static void sec_ts_handle_fod_event(struct sec_ts_data *ts,
					struct sec_ts_event_status *p_event_status)
{
	struct sec_ts_fod_event *p_fod =
				(struct sec_ts_fod_event *)p_event_status;
	int x = p_fod->x_b11_b8 << 8 | p_fod->x_b7_b0;
	int y = p_fod->y_b11_b8 << 8 | p_fod->y_b7_b0;
	s64 delta_ms = ktime_ms_delta(ktime_get(), ts->ktime_resume);

	if (test_bit(0, &ts->tid_touch_state)) {
		input_info(true, &ts->client->dev,
			   "%s: slot 0 is in use!", __func__);
		return;
	}

	if (!x || !y) {
		input_info(true, &ts->client->dev,
			   "%s: one of coords is ZERO(%d, %d)!",
			   __func__, x, y);
		x = ts->plat_data->fod_x;
		y = ts->plat_data->fod_y;
	}

	input_info(true, &ts->client->dev,
		   "STATUS: FoD: %s, X,Y: %d, %d\n", p_fod->status ? "ON" : "OFF", x, y);
	/*
	 * Send input cancel event to tunr off HBM if the following conditions match:
	 * 1. FoD status is off.
	 * 2. FoD event is before the time of (driver reusme + FOD_CANCEL_EVENT_DELTA_TIME).
	 */
	if (p_fod->status == false && delta_ms < FOD_CANCEL_EVENT_DELTA_TIME) {
		input_info(true, &ts->client->dev, "FoD: send input cancel event.\n");
		mutex_lock(&ts->eventlock);
		input_mt_slot(ts->input_dev, 0);
		input_report_key(ts->input_dev, BTN_TOUCH, 1);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 140);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MINOR, 140);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 1);
		input_sync(ts->input_dev);

		/* Report MT_TOOL_PALM for canceling the touch event. */
		input_mt_slot(ts->input_dev, 0);
		input_report_key(ts->input_dev, BTN_TOUCH, 1);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_PALM, 1);
		input_sync(ts->input_dev);

		/* Release slot 0. */
		input_mt_slot(ts->input_dev, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev,
					   MT_TOOL_FINGER, 0);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_sync(ts->input_dev);
		mutex_unlock(&ts->eventlock);
	}
}

static void sec_ts_read_vendor_event(struct sec_ts_data *ts,
					struct sec_ts_event_status *p_event_status)
{
	if (p_event_status->stype ==
		TYPE_STATUS_EVENT_VENDOR_INFO) {

		struct sec_ts_event_hopping *p_hopping_event;

		u8 status_id =
			p_event_status->status_id;
		u8 status_data_1 =
			p_event_status->status_data_1;
		u8 status_data_2 =
			p_event_status->status_data_2;

		switch (status_id) {
		case SEC_TS_EVENT_STATUS_ID_HOPPING:
			p_hopping_event =
				(struct sec_ts_event_hopping *)p_event_status;

			input_info(true,
				&ts->client->dev,
				"STATUS: hopping %d -> %d by %d with lvl %#x %#x\n",
					p_hopping_event->prev_id,
					p_hopping_event->id,
					p_hopping_event->cause,
					p_hopping_event->noise_lvl[0],
					p_hopping_event->noise_lvl[1]);
			break;

		case SEC_TS_EVENT_STATUS_ID_REPORT_RATE:
			ts->report_rate = status_data_1;
			if (ts->debug_status)
				input_info(true,
					&ts->client->dev,
					"STATUS: rate %d -> %d\n",
					status_data_2, status_data_1);
			break;

		case SEC_TS_EVENT_STATUS_ID_VSYNC:
			ts->vsync = status_data_1;
			if (ts->debug_status)
				input_info(true,
					&ts->client->dev,
					"STATUS: vsync %d -> %d\n",
					status_data_2, status_data_1);
			break;

		case SEC_TS_EVENT_STATUS_ID_WLC:
			input_info(true,
				&ts->client->dev,
				"STATUS: WLC: %#x\n",
				status_data_1);
			break;

		case SEC_TS_EVENT_STATUS_ID_NOISE:
			input_info(true,
				&ts->client->dev,
				"STATUS: noise: %#x\n",
				status_data_1);
			break;

		case SEC_TS_EVENT_STATUS_ID_GRIP:
			if (ts->debug_status)
				input_info(true,
					&ts->client->dev,
					"STATUS: grip: %d.\n",
					status_data_1);
			break;

		case SEC_TS_EVENT_STATUS_ID_PALM:
			input_info(true, &ts->client->dev,
				"STATUS: palm: %d.\n",
				status_data_1);
				if (status_data_1)
					ts->palm_count++;
			break;

		case SEC_TS_EVENT_STATUS_ID_FOD:
			sec_ts_handle_fod_event(ts, p_event_status);
			break;

		default:
			break;
		}
	} else {
		input_info(true, &ts->client->dev,
				"STATUS: %#x %#x %#x %#x %#x %#x %#x %#x\n",
				p_event_status->data[0],
				p_event_status->data[1],
				p_event_status->data[2],
				p_event_status->data[3],
				p_event_status->data[4],
				p_event_status->data[5],
				p_event_status->data[6],
				p_event_status->data[7]);
	}
}

#define MAX_EVENT_COUNT 32
static void sec_ts_read_event(struct sec_ts_data *ts)
{
	int ret;
	u8 event_id;
	u8 left_event_count;
	u8 read_event_buff[MAX_EVENT_COUNT][SEC_TS_EVENT_BUFF_SIZE] = { { 0 } };
	u8 *event_buff;
	struct sec_ts_gesture_status *p_gesture_status;
	struct sec_ts_event_status *p_event_status;
	int curr_pos;
	int remain_event_count = 0;
	bool processed_pointer_event = false;
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	unsigned long last_tid_palm_state = ts->tid_palm_state;
	unsigned long last_tid_grip_state = ts->tid_grip_state;
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
	struct touch_offload_frame *frame = NULL;
#endif
	struct sec_ts_health_check hc[1] = {0};

	if (ts->power_status == SEC_TS_STATE_LPM) {

		pm_wakeup_event(&ts->client->dev, 3 * MSEC_PER_SEC);
		/* waiting for blsp block resuming, if not occurs error */
		ret = wait_for_completion_interruptible_timeout(
				&ts->resume_done,
				msecs_to_jiffies(3 * MSEC_PER_SEC));
		if (ret == 0) {
			input_err(true, &ts->client->dev,
				  "%s: LPM: pm resume is not handled\n",
				  __func__);
			return;
		}

		if (ret < 0) {
			input_err(true, &ts->client->dev,
				  "%s: LPM: -ERESTARTSYS if interrupted, %d\n",
				  __func__, ret);
			return;
		}

		input_info(true, &ts->client->dev,
			"%s: run LPM interrupt handler, %d\n", __func__, ret);
		/* run lpm interrupt handler */
	}

	ret = event_id = curr_pos = remain_event_count = 0;
	/* repeat READ_ONE_EVENT until buffer is empty(No event) */
	ret = sec_ts_read(ts, SEC_TS_READ_ONE_EVENT,
			  (u8 *)read_event_buff[0], SEC_TS_EVENT_BUFF_SIZE);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: read one event failed\n", __func__);
		return;
	}

	if (ts->debug_events)
		input_info(true, &ts->client->dev,
			"ONE: %02X %02X %02X %02X %02X %02X %02X %02X\n",
			read_event_buff[0][0], read_event_buff[0][1],
			read_event_buff[0][2], read_event_buff[0][3],
			read_event_buff[0][4], read_event_buff[0][5],
			read_event_buff[0][6], read_event_buff[0][7]);

	if (read_event_buff[0][0] == 0) {
		input_info(true, &ts->client->dev,
			"%s: event buffer is empty\n", __func__);
		return;
	}

	left_event_count = read_event_buff[0][7] & 0x3F;
	remain_event_count = left_event_count;

	if (left_event_count > MAX_EVENT_COUNT - 1 ||
		left_event_count == 0xFF) {
		input_err(true, &ts->client->dev,
			"%s: event buffer overflow %d\n",
			__func__, left_event_count);

		/* write clear event stack command
		 * when read_event_count > MAX_EVENT_COUNT
		 **/
		ret = sec_ts_write(ts, SEC_TS_CMD_CLEAR_EVENT_STACK, NULL, 0);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				  "%s: write clear event failed\n", __func__);
		return;
	}

	if (left_event_count > 0) {
		ret = sec_ts_read(ts, SEC_TS_READ_ALL_EVENT,
			(u8 *)read_event_buff[1],
			sizeof(u8) * (SEC_TS_EVENT_BUFF_SIZE) *
				(left_event_count));
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				  "%s: read one event failed\n", __func__);
			return;
		}
	}

	do {
		event_buff = read_event_buff[curr_pos];
		event_id = event_buff[0] & 0x3;

		if (ts->debug_events && curr_pos > 0)
			input_info(true, &ts->client->dev,
				 "ALL: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				event_buff[0], event_buff[1], event_buff[2],
				event_buff[3], event_buff[4], event_buff[5],
				event_buff[6], event_buff[7]);

		switch (event_id) {
		case SEC_TS_STATUS_EVENT:
			hc->status_updated = true;
			ts->status_event_cnt++;
			p_event_status =
				(struct sec_ts_event_status *)event_buff;

			if (p_event_status->stype)
				sec_ts_read_vendor_event(ts, p_event_status);

			if ((p_event_status->stype ==
					TYPE_STATUS_EVENT_INFO) &&
				(p_event_status->status_id ==
					SEC_TS_ACK_BOOT_COMPLETE)) {
				u8 status_data_1 =
					p_event_status->status_data_1;

				switch (status_data_1) {
				case 0x20:
					/* watchdog reset !? */
					input_err(true, &ts->client->dev,
						"Touch - unexpected reset! Reason : WDT \n");

					sec_ts_locked_release_all_finger(ts);
					ret = sec_ts_write(ts,
						SEC_TS_CMD_SENSE_ON, NULL, 0);
					if (ret < 0)
						input_err(true,
							&ts->client->dev,
							"%s: fail to write Sense_on\n",
							__func__);
						sec_ts_reinit(ts);
					break;
				case 0x40:
					input_info(true, &ts->client->dev,
						"%s: sw_reset ack.\n",
						__func__);
					sec_ts_locked_release_all_finger(ts);
					complete_all(&ts->boot_completed);
					break;
				case 0x10:
					input_info(true, &ts->client->dev,
						"%s: hw_reset ack.\n",
						__func__);
					sec_ts_locked_release_all_finger(ts);
					complete_all(&ts->boot_completed);
					break;
				default:
					break;
				}

			}

			/* event queue full-> all finger release */
			if ((p_event_status->stype == TYPE_STATUS_EVENT_ERR) &&
				(p_event_status->status_id ==
					SEC_TS_ERR_EVENT_QUEUE_FULL)) {
				input_err(true, &ts->client->dev,
					"%s: IC Event Queue is full\n",
					__func__);
				sec_ts_locked_release_all_finger(ts);
			}

			if ((p_event_status->stype ==
				TYPE_STATUS_EVENT_ERR) &&
			    (p_event_status->status_id ==
				SEC_TS_ERR_EVENT_ESD)) {
				input_err(true, &ts->client->dev,
					  "%s: ESD detected. run reset\n",
					  __func__);
#ifdef USE_RESET_DURING_POWER_ON
				schedule_work(&ts->reset_work.work);
#endif
			}

			if ((p_event_status->stype ==
				TYPE_STATUS_EVENT_INFO) &&
			    (p_event_status->status_id ==
				SEC_TS_ACK_WET_MODE)) {
				ts->wet_mode = p_event_status->status_data_1;
				input_info(true, &ts->client->dev,
					"%s: water wet mode %d\n",
					__func__, ts->wet_mode);
				if (ts->wet_mode)
					ts->wet_count++;

				}

#ifdef SEC_TS_SUPPORT_CUSTOMLIB
			mutex_lock(&ts->eventlock);
			sec_ts_handle_lib_status_event(ts, p_event_status);
			mutex_unlock(&ts->eventlock);
#endif
			break;

		case SEC_TS_COORDINATE_EVENT:
			hc->coord_updated = true;
			ts->coord_event_cnt++;
			processed_pointer_event = true;
			mutex_lock(&ts->eventlock);
			sec_ts_handle_coord_event(ts,
				(struct sec_ts_event_coordinate *)event_buff);
			mutex_unlock(&ts->eventlock);
			break;

		case SEC_TS_GESTURE_EVENT:
			p_gesture_status =
				(struct sec_ts_gesture_status *)event_buff;
#ifdef SEC_TS_SUPPORT_CUSTOMLIB
			mutex_lock(&ts->eventlock);
			sec_ts_handle_gesture_event(ts, p_gesture_status);
			mutex_unlock(&ts->eventlock);
#endif
			break;

		default:
			input_err(true, &ts->client->dev,
				"%s: unknown event %x %x %x %x %x %x\n",
				__func__,
				event_buff[0], event_buff[1], event_buff[2],
				event_buff[3], event_buff[4], event_buff[5]);
			break;
		}
		curr_pos++;
		remain_event_count--;
	} while (remain_event_count >= 0);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
	if (!ts->offload.offload_running) {
#endif
	mutex_lock(&ts->eventlock);
	input_set_timestamp(ts->input_dev, ts->timestamp);
	input_sync(ts->input_dev);
	mutex_unlock(&ts->eventlock);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
	}

	if (processed_pointer_event) {
		ret = touch_offload_reserve_frame(&ts->offload, &frame);
		if (ret != 0) {
			input_dbg(true, &ts->client->dev,
				  "Could not reserve a frame: ret=%d.\n", ret);

			/* Stop offload when there are no buffers available */
			sec_ts_offload_set_running(ts, false);
		} else {
			sec_ts_offload_set_running(ts, true);

			sec_ts_populate_frame(ts, frame);

			ret = touch_offload_queue_frame(&ts->offload, frame);
			if (ret != 0) {
				pr_err("%s: Failed to queue reserved frame: ret=%d.\n",
				       __func__, ret);
			}
		}
	}
#endif

	/* TODO: If the mutual strength heatmap was already read into the touch
	 * offload interface, use it here instead of reading again.
	 */
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	if (processed_pointer_event) {
		if (ts->heatmap_init_done && !ts->offload.offload_running) {
			heatmap_read(&ts->v4l2, ktime_to_ns(ts->timestamp));
		}

		/* palm */
		if (last_tid_palm_state == 0 &&
			ts->tid_palm_state >= 1) {
			input_info(true, &ts->client->dev,
				"COORD: detect palm enter(tid 0x0 -> %#x)\n",
				ts->tid_palm_state);
		}
		if (last_tid_palm_state >= 1 &&
			ts->tid_palm_state == 0) {
			input_info(true, &ts->client->dev,
				"COORD: detect palm leave(tid %#x -> 0x0), tid_touch %#x\n",
				last_tid_palm_state, ts->tid_touch_state);
			if (ts->touch_count || ts->tid_touch_state) {
				ts->palms_leaved_once = true;
				input_dbg(true, &ts->client->dev,
					"COORD: wait all finger(s) release after palm entered\n");
			}
		}
		/* grip */
		if (last_tid_grip_state == 0 &&
			ts->tid_grip_state >= 1) {
			input_info(true, &ts->client->dev,
				"COORD: detect grip enter(tid 0x0 -> %#x)\n",
				ts->tid_grip_state);
		}
		if (last_tid_grip_state >= 1 &&
			ts->tid_grip_state == 0) {
			input_info(true, &ts->client->dev,
				"COORD: detect grip leave(tid %#x -> 0x0), tid_touch %#x\n",
				last_tid_grip_state, ts->tid_touch_state);
			if (ts->touch_count || ts->tid_touch_state) {
				ts->grips_leaved_once = true;
				input_dbg(true, &ts->client->dev,
					"COORD: wait all finger(s) release after grip entered\n");
			}
		}
		if ((ts->touch_count == 0 || ts->tid_touch_state == 0) &&
			(ts->palms_leaved_once || ts->grips_leaved_once)) {
			ts->palms_leaved_once = false;
			ts->grips_leaved_once = false;
			input_info(true, &ts->client->dev,
				"COORD: all fingers released with palm(s)/grip(s) leaved once\n");
		}
	}

	/* Disable the firmware motion filter during single touch */
	if (!ts->offload.offload_running)
		update_motion_filter(ts, ts->tid_touch_state);
#endif

	/* Update the health check info. */
	sec_ts_hc_update_and_push(ts, hc);
}

static irqreturn_t sec_ts_isr(int irq, void *handle)
{
	struct sec_ts_data *ts = (struct sec_ts_data *)handle;

	ts->timestamp = ktime_get();
	ts->int_cnt++;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t sec_ts_irq_thread(int irq, void *ptr)
{
	struct sec_ts_data *ts = (struct sec_ts_data *)ptr;

	if (sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_IRQ, true) < 0) {
		/* Interrupt during bus suspend */
		input_info(true, &ts->client->dev,
			"%s: Skipping stray interrupt since bus is suspended(power_status: %d)\n",
			__func__, ts->power_status);
		return IRQ_HANDLED;
	}

	/* prevent CPU from entering deep sleep */
	cpu_latency_qos_update_request(&ts->pm_qos_req, 100);
	pm_wakeup_event(&ts->client->dev, MSEC_PER_SEC);

	sec_ts_read_event(ts);

	cpu_latency_qos_update_request(&ts->pm_qos_req, PM_QOS_DEFAULT_VALUE);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_IRQ, false);

	return IRQ_HANDLED;
}

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
static void sec_ts_offload_report(void *handle,
				  struct TouchOffloadIocReport *report)
{
	struct sec_ts_data *ts = (struct sec_ts_data *)handle;
	bool touch_down = 0;
	unsigned long touch_id = 0;
	int i;

	mutex_lock(&ts->eventlock);

	input_set_timestamp(ts->input_dev, report->timestamp);

	for (i = 0; i < MAX_COORDS; i++) {
		if (report->coords[i].status == COORD_STATUS_FINGER) {
			input_mt_slot(ts->input_dev, i);
			touch_down = 1;
			__set_bit(i, &touch_id);
			input_report_key(ts->input_dev, BTN_TOUCH,
					 touch_down);
			input_mt_report_slot_state(ts->input_dev,
						   MT_TOOL_FINGER, 1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					 report->coords[i].x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					 report->coords[i].y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
					 report->coords[i].major);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MINOR,
					 report->coords[i].minor);
			if (ts->plat_data->support_mt_pressure)
				input_report_abs(ts->input_dev,
					ABS_MT_PRESSURE,
					report->coords[i].pressure);
			input_report_abs(ts->input_dev, ABS_MT_ORIENTATION,
					 report->coords[i].rotation);
		} else {
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev,
						   MT_TOOL_FINGER, 0);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID,
					 -1);
			input_report_abs(ts->input_dev, ABS_MT_ORIENTATION, 0);
		}
	}

	input_report_key(ts->input_dev, BTN_TOUCH, touch_down);

	input_sync(ts->input_dev);

	mutex_unlock(&ts->eventlock);

	if (touch_down)
		heatmap_read(&ts->v4l2, ktime_to_ns(report->timestamp));

	/* Disable the firmware motion filter during single touch */
	update_motion_filter(ts, touch_id);
}
#endif /* CONFIG_TOUCHSCREEN_OFFLOAD */

int get_tsp_status(void)
{
	return 0;
}
EXPORT_SYMBOL(get_tsp_status);

int sec_ts_glove_mode_enables(struct sec_ts_data *ts, int mode)
{
	int ret;

	if (mode)
		ts->touch_functions = (ts->touch_functions |
				       SEC_TS_BIT_SETFUNC_GLOVE |
				       SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC);
	else
		ts->touch_functions = ((ts->touch_functions &
					(~SEC_TS_BIT_SETFUNC_GLOVE)) |
				       SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			"%s: pwr off, glove: %d, status: %x\n", __func__,
			mode, ts->touch_functions);
		goto glove_enable_err;
	}

	ret = sec_ts_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION,
			   (u8 *)&ts->touch_functions, 2);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: Failed to send command", __func__);
		goto glove_enable_err;
	}

	input_info(true, &ts->client->dev,
		"%s: glove: %d, status: %x\n", __func__,
		mode, ts->touch_functions);

	return 0;

glove_enable_err:
	return -EIO;
}
EXPORT_SYMBOL(sec_ts_glove_mode_enables);

int sec_ts_set_cover_type(struct sec_ts_data *ts, bool enable)
{
	int ret;

	input_info(true, &ts->client->dev, "%s: %d\n",
		   __func__, ts->cover_type);


	switch (ts->cover_type) {
	case SEC_TS_VIEW_WIRELESS:
	case SEC_TS_VIEW_COVER:
	case SEC_TS_VIEW_WALLET:
	case SEC_TS_FLIP_WALLET:
	case SEC_TS_LED_COVER:
	case SEC_TS_MONTBLANC_COVER:
	case SEC_TS_CLEAR_FLIP_COVER:
	case SEC_TS_QWERTY_KEYBOARD_EUR:
	case SEC_TS_QWERTY_KEYBOARD_KOR:
		ts->cover_cmd = (u8)ts->cover_type;
		break;
	case SEC_TS_CHARGER_COVER:
	case SEC_TS_COVER_NOTHING1:
	case SEC_TS_COVER_NOTHING2:
	default:
		ts->cover_cmd = 0;
		input_err(true, &ts->client->dev,
			 "%s: not chage touch state, %d\n",
			__func__, ts->cover_type);
		break;
	}

	if (enable)
		ts->touch_functions = (ts->touch_functions |
				       SEC_TS_BIT_SETFUNC_COVER |
				       SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC);
	else
		ts->touch_functions = ((ts->touch_functions &
					(~SEC_TS_BIT_SETFUNC_COVER)) |
				       SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: pwr off, close: %d, status: %x\n", __func__,
			enable, ts->touch_functions);
		goto cover_enable_err;
	}

	if (enable) {
		ret = sec_ts_write(ts, SEC_TS_CMD_SET_COVERTYPE,
				   &ts->cover_cmd, 1);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				  "%s: Failed to send covertype command: %d",
				  __func__, ts->cover_cmd);
			goto cover_enable_err;
		}
	}

	ret = sec_ts_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION,
			   (u8 *)&(ts->touch_functions), 2);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: Failed to send command", __func__);
		goto cover_enable_err;
	}

	input_info(true, &ts->client->dev,
		"%s: close: %d, status: %x\n", __func__,
		enable, ts->touch_functions);

	return 0;

cover_enable_err:
	return -EIO;


}
EXPORT_SYMBOL(sec_ts_set_cover_type);

void sec_ts_set_grip_type(struct sec_ts_data *ts, u8 set_type)
{
	u8 mode = G_NONE;

	input_info(true, &ts->client->dev,
		"%s: re-init grip(%d), edh: %d, edg: %d, lan: %d\n", __func__,
		set_type, ts->grip_edgehandler_direction, ts->grip_edge_range,
		ts->grip_landscape_mode);

	/* edge handler */
	if (ts->grip_edgehandler_direction != 0)
		mode |= G_SET_EDGE_HANDLER;

	if (set_type == GRIP_ALL_DATA) {
		/* edge */
		if (ts->grip_edge_range != 60)
			mode |= G_SET_EDGE_ZONE;

		/* dead zone */
		if (ts->grip_landscape_mode == 1)	/* default 0 mode, 32 */
			mode |= G_SET_LANDSCAPE_MODE;
		else
			mode |= G_SET_NORMAL_MODE;
	}

	if (mode)
		set_grip_data_to_ic(ts, mode);

}

/* for debugging--------------------------------------------------------------*/

static int sec_ts_pinctrl_configure(struct sec_ts_data *ts, bool enable)
{
	struct pinctrl_state *state;

	input_info(true, &ts->client->dev, "%s: %s\n",
		   __func__, enable ? "ACTIVE" : "SUSPEND");

	if (enable) {
		state = pinctrl_lookup_state(ts->plat_data->pinctrl,
					     "on_state");
		if (IS_ERR(ts->plat_data->pinctrl))
			input_err(true, &ts->client->dev,
				"%s: could not get active pinstate\n",
				__func__);
	} else {
		state = pinctrl_lookup_state(ts->plat_data->pinctrl,
					     "off_state");
		if (IS_ERR(ts->plat_data->pinctrl))
			input_err(true, &ts->client->dev,
				"%s: could not get suspend pinstate\n",
				__func__);
	}

	if (!IS_ERR_OR_NULL(state))
		return pinctrl_select_state(ts->plat_data->pinctrl, state);

	return 0;

}

/* Return true if you should defer sec_ts probe waiting for
 * avdd or vdd regulators.
 */
static int sec_ts_check_for_deferred_regulators(struct device *dev)
{
	struct regulator *reg = regulator_get(dev, "vdd");

	if (reg == ERR_PTR(-EPROBE_DEFER))
		return true;
	if (!IS_ERR_OR_NULL(reg))
		regulator_put(reg);

	reg = regulator_get(dev, "avdd");
	if (reg == ERR_PTR(-EPROBE_DEFER))
		return true;
	if (!IS_ERR_OR_NULL(reg))
		regulator_put(reg);
	return false;
}

static int sec_ts_power(void *data, bool on)
{
	struct sec_ts_data *ts = (struct sec_ts_data *)data;
	struct sec_ts_plat_data *pdata = ts->plat_data;
	static bool vdd_enabled, avdd_enabled;
	int ret = 0;

	if (!pdata->regulator_vdd) {
		pdata->regulator_vdd = regulator_get(&ts->client->dev, "vdd");
		if (IS_ERR_OR_NULL(pdata->regulator_vdd)) {
			pdata->regulator_vdd = NULL;
			input_err(true, &ts->client->dev,
				"%s: Failed to get vdd regulator.\n",
				__func__);
		}
	}

	if (!pdata->regulator_avdd) {
		pdata->regulator_avdd = regulator_get(&ts->client->dev, "avdd");
		if (IS_ERR_OR_NULL(pdata->regulator_avdd)) {
			pdata->regulator_avdd = NULL;
			input_err(true, &ts->client->dev,
				"%s: Failed to get avdd regulator.\n",
				__func__);
		}
	}

	if (pdata->regulator_vdd) {
		if (vdd_enabled != on){
			ret = (on) ? regulator_enable(pdata->regulator_vdd) :
				regulator_disable(pdata->regulator_vdd);
			if (ret)
				input_err(true, &ts->client->dev,
					"%s: Failed to control vdd: %d\n",
					__func__, ret);
			else {
				input_info(true, &ts->client->dev, "%s: %s vdd\n",
					__func__, on ? "enable" : "disable");
				sec_ts_delay(on ? 1 : 4);
				vdd_enabled = on;
			}
		}

		if (!vdd_enabled) {
			regulator_put(pdata->regulator_vdd);
			pdata->regulator_vdd = NULL;
		}
	}

	if (pdata->regulator_avdd) {
		if (avdd_enabled != on) {
			ret = (on) ? regulator_enable(pdata->regulator_avdd) :
				regulator_disable(pdata->regulator_avdd);
			if (ret)
				input_err(true, &ts->client->dev,
					"%s: Failed to control avdd: %d\n",
					__func__, ret);
			else {
				input_info(true, &ts->client->dev, "%s: %s avdd\n",
					__func__, on ? "enable" : "disable");
				avdd_enabled = on;
			}
		}

		if (!avdd_enabled) {
			regulator_put(pdata->regulator_avdd);
			pdata->regulator_avdd = NULL;
		}
	}

	return ret;
}

#ifdef I2C_INTERFACE
static int sec_ts_parse_dt(struct i2c_client *client)
#else
static int sec_ts_parse_dt(struct spi_device *client)
#endif
{
	struct device *dev = &client->dev;
	struct sec_ts_plat_data *pdata = dev->platform_data;
	struct device_node *np = dev->of_node;
	u32 coords[2];
	u8 offload_id[4];
	int ret = 0;
	int count = 0;
	u32 ic_match_value;
	int lcdtype = 0;
#if defined(CONFIG_EXYNOS_DECON_FB)
	int connected;
#endif
	int index;
	struct of_phandle_args panelmap;
	struct drm_panel *panel = NULL;

	if (of_property_read_bool(np, "sec,panel_map")) {
		for (index = 0 ;; index++) {
			ret = of_parse_phandle_with_fixed_args(np,
					"sec,panel_map",
					1,
					index,
					&panelmap);
			if (ret)
				return -EPROBE_DEFER;
			panel = of_drm_find_panel(panelmap.np);
			of_node_put(panelmap.np);
			if (!IS_ERR_OR_NULL(panel)) {
				pdata->panel = panel;
				pdata->initial_panel_index = panelmap.args[0];
				break;
			}
		}
	}

	pdata->tsp_icid = of_get_named_gpio(np, "sec,tsp-icid_gpio", 0);
	if (gpio_is_valid(pdata->tsp_icid)) {
		input_info(true, dev, "%s: TSP_ICID : %d\n",
			   __func__, gpio_get_value(pdata->tsp_icid));
		if (of_property_read_u32(np, "sec,icid_match_value",
					 &ic_match_value)) {
			input_err(true, dev,
				"%s: Failed to get icid match value\n",
				__func__);
			return -EINVAL;
		}

		if (gpio_get_value(pdata->tsp_icid) != ic_match_value) {
			input_err(true, dev,
				  "%s: Do not match TSP_ICID\n", __func__);
			return -EINVAL;
		}
	} else {
		input_dbg(true, dev,
			  "%s: Failed to get tsp-icid gpio\n", __func__);
	}

	pdata->vsync_gpio = of_get_named_gpio(np, "sec,tsp_vsync_gpio", 0);
	if (gpio_is_valid(pdata->vsync_gpio))
		input_info(true, &client->dev, "%s: vsync %s\n", __func__,
			gpio_get_value(pdata->vsync_gpio) ?
				"disable" : "enable");

	pdata->irq_gpio = of_get_named_gpio(np, "sec,irq_gpio", 0);
	if (gpio_is_valid(pdata->irq_gpio)) {
		ret = gpio_request_one(pdata->irq_gpio, GPIOF_DIR_IN,
				       "sec,tsp_int");
		if (ret) {
			input_err(true, &client->dev,
				  "%s: Unable to request tsp_int [%d]\n",
				  __func__, pdata->irq_gpio);
			return -EINVAL;
		}
	} else {
		input_err(true, &client->dev,
			  "%s: Failed to get irq gpio\n", __func__);
		return -EINVAL;
	}

	client->irq = gpio_to_irq(pdata->irq_gpio);

	if (of_property_read_u32(np, "sec,irq_type", &pdata->irq_type)) {
		input_dbg(true, dev,
			"%s: no irq_type property, set to default!\n",
			__func__);
		pdata->irq_type = IRQF_TRIGGER_LOW | IRQF_ONESHOT;
	}

	if (of_property_read_u32(np, "sec,i2c-burstmax", &pdata->io_burstmax)) {
		input_dbg(false, &client->dev,
			  "%s: Failed to get io_burstmax property\n", __func__);
		pdata->io_burstmax = 1024; //TODO: check this
	}
	if (pdata->io_burstmax > IO_PREALLOC_READ_BUF_SZ ||
	    pdata->io_burstmax > IO_PREALLOC_WRITE_BUF_SZ) {
		input_err(true, &client->dev,
			  "%s: io_burstmax is larger than io_read_buf and/or io_write_buf.\n",
			  __func__);
//TODO: check this
//		return -EINVAL;
	}

	if (of_property_read_u32_array(np, "sec,max_coords", coords, 2)) {
		input_err(true, &client->dev,
			  "%s: Failed to get max_coords property\n", __func__);
		return -EINVAL;
	}
	pdata->max_x = coords[0] - 1;
	pdata->max_y = coords[1] - 1;

	if (of_property_read_u32_array(np, "sec,fod_coords", coords, 2)) {
		input_info(true, &client->dev,
			  "%s: sec,fod_coords not found!\n", __func__);
		coords[0] = 0;
		coords[1] = 0;
	}
	pdata->fod_x = coords[0];
	pdata->fod_y = coords[1];

#ifdef PAT_CONTROL
	if (of_property_read_u32(np, "sec,pat_function",
				 &pdata->pat_function) < 0) {
		pdata->pat_function = 0;
		input_err(true, dev,
			"%s: Failed to get pat_function property\n", __func__);
	} else {
		input_info(true, dev,
			"%s: pat_function: %#x\n", __func__, pdata->pat_function);
	}

	if (of_property_read_u32(np, "sec,afe_base", &pdata->afe_base) < 0) {
		pdata->afe_base = 0;
		input_err(true, dev,
			  "%s: Failed to get afe_base property\n", __func__);
	}
#endif

	pdata->tsp_id = of_get_named_gpio(np, "sec,tsp-id_gpio", 0);
	if (gpio_is_valid(pdata->tsp_id))
		input_info(true, dev, "%s: TSP_ID : %d\n", __func__,
			   gpio_get_value(pdata->tsp_id));
	else
		input_dbg(true, dev,
			  "%s: Failed to get tsp-id gpio\n", __func__);

	pdata->switch_gpio = of_get_named_gpio(np,
					       "sec,switch_gpio", 0);
	if (gpio_is_valid(pdata->switch_gpio)) {
		ret = gpio_request_one(pdata->switch_gpio,
				       GPIOF_OUT_INIT_LOW,
				       "sec,touch_i2c_switch");
		if (ret) {
			input_err(true, dev,
				  "%s: Failed to request gpio %d\n",
				  __func__, pdata->switch_gpio);
			return -EINVAL;
		}

		ret = gpio_direction_output(pdata->switch_gpio,
					    SEC_SWITCH_GPIO_VALUE_AP_MASTER);
		if (ret) {
			input_err(true, dev,
				  "%s: Failed to set gpio %d direction\n",
				  __func__, pdata->switch_gpio);
			return -EINVAL;
		}
	} else {
		input_info(true, dev, "%s: unavailable switch_gpio!\n",
			  __func__);
	}

	pdata->reset_gpio = of_get_named_gpio(np, "sec,reset_gpio", 0);
	if (gpio_is_valid(pdata->reset_gpio)) {
		ret = gpio_request_one(pdata->reset_gpio,
					GPIOF_OUT_INIT_HIGH,
					"sec,touch_reset_gpio");
		if (ret) {
			input_err(true, dev,
				  "%s: Failed to request gpio %d, ret %d\n",
				  __func__, pdata->reset_gpio, ret);
			pdata->reset_gpio = -1;
		}
		ret = gpio_direction_output(pdata->reset_gpio, 0);

	} else
		input_err(true, dev, "%s: Failed to get reset_gpio\n",
			__func__);

	count = of_property_count_strings(np, "sec,firmware_name");
	if (count <= 0) {
		pdata->firmware_name = NULL;
	} else {
		if (gpio_is_valid(pdata->tsp_id))
			of_property_read_string_index(np, "sec,firmware_name",
						gpio_get_value(pdata->tsp_id),
						&pdata->firmware_name);
		else
			of_property_read_string_index(np, "sec,firmware_name",
						      0, &pdata->firmware_name);
	}

	if (of_property_read_string_index(np, "sec,project_name", 0,
					  &pdata->project_name))
		input_dbg(true, &client->dev,
			"%s: skipped to get project_name property\n", __func__);
	if (of_property_read_string_index(np, "sec,model_name",
					  1, &pdata->model_name))
		input_dbg(true, &client->dev,
			  "%s: skipped to get model_name property\n", __func__);

#if defined(CONFIG_FB_MSM_MDSS_SAMSUNG)
	lcdtype = get_lcd_attached("GET");
	if (lcdtype < 0) {
		input_err(true, &client->dev,
			  "%s: lcd is not attached\n", __func__);
		return -ENODEV;
	}
#endif

#if defined(CONFIG_EXYNOS_DECON_FB)
	connected = get_lcd_info("connected");
	if (connected < 0) {
		input_err(true, dev, "%s: Failed to get lcd info\n", __func__);
		return -EINVAL;
	}

	if (!connected) {
		input_err(true, &client->dev,
			  "%s: lcd is disconnected\n", __func__);
		return -ENODEV;
	}

	input_info(true, &client->dev, "%s: lcd is connected\n", __func__);

	lcdtype = get_lcd_info("id");
	if (lcdtype < 0) {
		input_err(true, dev, "%s: Failed to get lcd info\n", __func__);
		return -EINVAL;
	}
#endif

	input_info(true, &client->dev,
		   "%s: lcdtype 0x%08X\n", __func__, lcdtype);

	if (pdata->model_name && strncmp(pdata->model_name, "G950", 4) == 0)
		pdata->panel_revision = 0;
	else
		pdata->panel_revision = ((lcdtype >> 8) & 0xFF) >> 4;

	pdata->power = sec_ts_power;

	if (of_property_read_u32(np, "sec,always_lpmode",
				 &pdata->always_lpmode) < 0)
		pdata->always_lpmode = 0;

	if (of_property_read_u32(np, "sec,bringup", &pdata->bringup) < 0)
		pdata->bringup = 0;

	if (of_property_read_u32(np, "sec,mis_cal_check",
				 &pdata->mis_cal_check) < 0)
		pdata->mis_cal_check = 0;

	if (of_property_read_u32(np, "sec,encoded_enable",
		&pdata->encoded_enable) < 0)
		pdata->encoded_enable = 0;

	pdata->grip_prescreen_mode = GRIP_PRESCREEN_MODE_2;
	pdata->grip_prescreen_timeout = 120;
	pdata->is_heatmap_enabled = false;
	pdata->encoded_frame_counter = 0;
	pdata->encoded_skip_counter = 0;

	if (of_property_read_u32(np, "sec,heatmap_mode",
		&pdata->heatmap_mode) < 0)
		pdata->heatmap_mode = 0;

	pdata->regulator_boot_on = of_property_read_bool(np,
						"sec,regulator_boot_on");
	pdata->support_sidegesture = of_property_read_bool(np,
						"sec,support_sidegesture");
	pdata->support_dex = of_property_read_bool(np, "support_dex_mode");

	pdata->support_mt_pressure = true;

	pdata->offload_id = 0;
	if (of_property_read_u8_array(np, "sec,touch_offload_id",
				      offload_id, 4) == -EINVAL)
		input_err(true, &client->dev,
			  "%s: Failed to read sec,touch_offload_id\n");
	else {
		pdata->offload_id = *(u32 *)offload_id;
		input_info(true, &client->dev,
			   "%s: Offload device ID = \"%c%c%c%c\" / 0x%08X\n",
			   __func__, offload_id[0], offload_id[1], offload_id[2],
			   offload_id[3], pdata->offload_id);
	}

	if (of_property_read_u8(np, "sec,mm2px", &pdata->mm2px) < 0)
		pdata->mm2px = 1;
	input_info(true, &client->dev,
		   "%s: mm2px %d\n", __func__, pdata->mm2px);

	input_info(true, &client->dev,
		"%s: io_burstmax: %d, bringup: %d, FW: %s, mis_cal_check: %d\n",
		__func__, pdata->io_burstmax, pdata->bringup,
		pdata->firmware_name, pdata->mis_cal_check);
	return ret;
}

int sec_ts_read_information(struct sec_ts_data *ts)
{
	unsigned char data[13] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_READ_INFO, true);

	memset(data, 0x0, 3);
	ret = sec_ts_read(ts, SEC_TS_READ_ID, data, 3);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
					"%s: failed to read device id(%d)\n",
					__func__, ret);
		goto out;
	}

	input_info(true, &ts->client->dev,
				"%s: %X, %X, %X\n",
				__func__, data[0], data[1], data[2]);
	memset(data, 0x0, 11);
	ret = sec_ts_read(ts,  SEC_TS_READ_PANEL_INFO, data, 11);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
					"%s: failed to read sub id(%d)\n",
					__func__, ret);
		goto out;
	}

	input_info(true, &ts->client->dev,
		   "%s: nTX: %d, nRX: %d, rY: %d, rX: %d\n",
		   __func__, data[8], data[9],
		   (data[2] << 8) | data[3], (data[0] << 8) | data[1]);

	/* Set X,Y Resolution from IC information. */
	if (((data[0] << 8) | data[1]) > 0)
		ts->plat_data->max_x = ((data[0] << 8) | data[1]) - 1;

	if (((data[2] << 8) | data[3]) > 0)
		ts->plat_data->max_y = ((data[2] << 8) | data[3]) - 1;

	ts->tx_count = data[8];
	ts->rx_count = data[9];

	data[0] = 0;
	ret = sec_ts_read(ts, SEC_TS_READ_BOOT_STATUS, data, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
					"%s: failed to read sub id(%d)\n",
					__func__, ret);
		goto out;
	}

	input_info(true, &ts->client->dev,
				"%s: BOOT_STATUS: %X\n",
				__func__, data[0]);

	memset(data, 0x0, 4);
	ret = sec_ts_read(ts, SEC_TS_READ_TS_STATUS, data, 4);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
					"%s: failed to read sub id(%d)\n",
					__func__, ret);
		goto out;
	}

	input_info(true, &ts->client->dev,
		   "%s: TS_STATUS: %02X, %02X, %02X, %02X\n",
		   __func__, data[0], data[1], data[2], data[3]);
	ret = sec_ts_read(ts, SEC_TS_CMD_SET_TOUCHFUNCTION,
			  (u8 *)&(ts->touch_functions), 2);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			"%s: failed to read touch functions(%d)\n",
			__func__, ret);
		goto out;
	}

	input_info(true, &ts->client->dev,
				"%s: Functions: %02X\n",
				__func__, ts->touch_functions);

out:
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_READ_INFO, false);
	return ret;
}

#ifdef SEC_TS_SUPPORT_CUSTOMLIB
int sec_ts_set_custom_library(struct sec_ts_data *ts)
{
	u8 data[3] = { 0 };
	int ret;

	input_err(true, &ts->client->dev, "%s: Custom Library (0x%02x)\n",
				__func__, ts->lowpower_mode);

	data[2] = ts->lowpower_mode;

	ret = sec_ts_write(ts, SEC_TS_CMD_CUSTOMLIB_WRITE_PARAM, &data[0], 3);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: Failed to Custom Library\n", __func__);

	ret = sec_ts_write(ts, SEC_TS_CMD_CUSTOMLIB_NOTIFY_PACKET, NULL, 0);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			"%s: Failed to send NOTIFY Custom Library\n", __func__);

	return ret;
}

int sec_ts_check_custom_library(struct sec_ts_data *ts)
{
	u8 data[10] = { 0 };
	int ret = -1;

	ret = ts->sec_ts_read(ts, SEC_TS_CMD_CUSTOMLIB_GET_INFO, &data[0], 10);

	input_info(true, &ts->client->dev,
		"%s: (%d) %c%c%c%c, || %02X, %02X, %02X, %02X, || %02X, %02X\n",
		__func__, ret, data[0], data[1], data[2], data[3], data[4],
		data[5], data[6], data[7], data[8], data[9]);

	/* compare model name with device tree */
	if (ts->plat_data->model_name)
		ret = strncmp(data, ts->plat_data->model_name, 4);

	if (ret == 0)
		ts->use_customlib = true;
	else
		ts->use_customlib = false;

	input_err(true, &ts->client->dev, "%s: use %s\n",
		  __func__, ts->use_customlib ? "CUSTOMLIB" : "VENDOR");

	return ret;
}
#endif

static void sec_ts_set_input_prop(struct sec_ts_data *ts,
				  struct input_dev *dev, u8 propbit)
{
	static char sec_ts_phys[64] = { 0 };

	snprintf(sec_ts_phys, sizeof(sec_ts_phys), "%s/input1",
			dev->name);
	dev->phys = sec_ts_phys;
#ifdef I2C_INTERFACE
	dev->id.bustype = BUS_I2C;
#else
	dev->id.bustype = BUS_SPI;
#endif
	dev->dev.parent = &ts->client->dev;

	set_bit(EV_SYN, dev->evbit);
	set_bit(EV_KEY, dev->evbit);
	set_bit(EV_ABS, dev->evbit);
	set_bit(EV_SW, dev->evbit);
	set_bit(BTN_TOUCH, dev->keybit);
	set_bit(BTN_TOOL_FINGER, dev->keybit);
#ifdef SEC_TS_SUPPORT_CUSTOMLIB
	set_bit(KEY_BLACK_UI_GESTURE, dev->keybit);
#endif
#ifdef SEC_TS_SUPPORT_TOUCH_KEY
	if (ts->plat_data->support_mskey) {
		int i;

		for (i = 0 ; i < ts->plat_data->num_touchkey ; i++)
			set_bit(ts->plat_data->touchkey[i].keycode,
				dev->keybit);

		set_bit(EV_LED, dev->evbit);
		set_bit(LED_MISC, dev->ledbit);
	}
#endif
#ifdef KEY_SIDE_GESTURE
	if (ts->plat_data->support_sidegesture) {
		set_bit(KEY_SIDE_GESTURE, dev->keybit);
		set_bit(KEY_SIDE_GESTURE_RIGHT, dev->keybit);
		set_bit(KEY_SIDE_GESTURE_LEFT, dev->keybit);
	}
#endif
	set_bit(propbit, dev->propbit);
	set_bit(KEY_HOMEPAGE, dev->keybit);

#ifdef SW_GLOVE
	input_set_capability(dev, EV_SW, SW_GLOVE);
#endif
	input_set_abs_params(dev, ABS_MT_POSITION_X, 0, ts->plat_data->max_x,
			     0, 0);
	input_set_abs_params(dev, ABS_MT_POSITION_Y, 0, ts->plat_data->max_y,
			     0, 0);
	input_set_abs_params(dev, ABS_MT_TOUCH_MAJOR, 0,
			255 * ts->plat_data->mm2px,
			0, 0);
	input_set_abs_params(dev, ABS_MT_TOUCH_MINOR, 0,
			255 * ts->plat_data->mm2px,
			0, 0);
	input_set_abs_params(dev, ABS_MT_TOOL_TYPE, MT_TOOL_FINGER,
			     MT_TOOL_FINGER, 0, 0);
#ifdef ABS_MT_CUSTOM
	input_set_abs_params(dev, ABS_MT_CUSTOM, 0, 0xFFFF, 0, 0);
#endif
	if (ts->plat_data->support_mt_pressure)
		input_set_abs_params(dev, ABS_MT_PRESSURE, 0,
				     SEC_TS_PRESSURE_MAX, 0, 0);

	/* Units are (-4096, 4096), representing the range between rotation
	 * 90 degrees to left and 90 degrees to the right.
	 */
	input_set_abs_params(dev, ABS_MT_ORIENTATION, -4096, 4096, 0, 0);

	if (propbit == INPUT_PROP_POINTER)
		input_mt_init_slots(dev, MAX_SUPPORT_TOUCH_COUNT,
				    INPUT_MT_POINTER);
	else
		input_mt_init_slots(dev, MAX_SUPPORT_TOUCH_COUNT,
				    INPUT_MT_DIRECT);

	input_set_drvdata(dev, ts);
}

static int sec_ts_fw_init(struct sec_ts_data *ts)
{
	int ret = SEC_TS_ERR_NA;
	bool force_update = false;
	bool valid_firmware_integrity = false;
	unsigned char data[5] = { 0 };
	unsigned char deviceID[5] = { 0 };
	unsigned char result = 0;

	ret = sec_ts_read(ts, SEC_TS_READ_DEVICE_ID, deviceID, 5);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: failed to read device ID(%d)\n",
			  __func__, ret);
	else
		input_info(true, &ts->client->dev,
			"%s: DEVICE ID: %02X, %02X, %02X, %02X, %02X\n",
			__func__, deviceID[0], deviceID[1], deviceID[2],
			deviceID[3], deviceID[4]);

	ret = sec_ts_read(ts, SEC_TS_READ_FIRMWARE_INTEGRITY, &result, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: failed to integrity check (%d)\n",
			  __func__, ret);
	} else {
		if (result & 0x80)
			valid_firmware_integrity = true;
		else
			input_err(true, &ts->client->dev,
				  "%s: invalid integrity result (0x%x)\n",
				  __func__, result);
	}

	ret = sec_ts_read(ts, SEC_TS_READ_BOOT_STATUS, &data[0], 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: failed to read sub id(%d)\n", __func__, ret);
	} else {
		ret = sec_ts_read(ts, SEC_TS_READ_TS_STATUS, &data[1], 4);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				  "%s: failed to touch status(%d)\n",
				  __func__, ret);
	}
	input_info(true, &ts->client->dev,
		"%s: TOUCH STATUS: %02X || %02X, %02X, %02X, %02X\n",
		__func__, data[0], data[1], data[2], data[3], data[4]);

	if (data[0] == SEC_TS_STATUS_BOOT_MODE)
		ts->checksum_result = 1;

	if (((data[0] == SEC_TS_STATUS_APP_MODE &&
	      data[2] == TOUCH_SYSTEM_MODE_FLASH) || ret < 0) &&
	    (valid_firmware_integrity == false))
		force_update = true;

	ret = sec_ts_read_information(ts);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: fail to read information 0x%x\n",
			  __func__, ret);
		return SEC_TS_ERR_INIT;
	}

	ts->touch_functions |= SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC;
	ret = sec_ts_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION,
			       (u8 *)&ts->touch_functions, 2);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: Failed to send touch func_mode command",
			  __func__);

	/* Sense_on */
	ret = sec_ts_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: fail to write Sense_on 0x%x\n",
			  __func__, ret);
		return SEC_TS_ERR_INIT;
	}

	ts->pFrame = kzalloc(ts->tx_count * ts->rx_count * 2, GFP_KERNEL);
	if (!ts->pFrame)
		return SEC_TS_ERR_ALLOC_FRAME;


	ts->pFrameSS = kzalloc((ts->tx_count + ts->rx_count) * 2, GFP_KERNEL);
	if (!ts->pFrameSS) {
		kfree(ts->pFrame);
		ts->pFrame = NULL;
		return SEC_TS_ERR_ALLOC_FRAME_SS;
	}

#ifdef USE_STIM_PAD
	ts->gainTable = kzalloc(ts->tx_count * ts->rx_count, GFP_KERNEL);
	if (!ts->gainTable) {
		kfree(ts->pFrame);
		kfree(ts->pFrameSS);
		ts->pFrame = NULL;
		ts->pFrameSS = NULL;
		return SEC_TS_ERR_ALLOC_GAINTABLE;
	}
#endif

	if (ts->plat_data->support_dex) {
		ts->input_dev_pad->name = "sec_touchpad";
		sec_ts_set_input_prop(ts, ts->input_dev_pad,
				      INPUT_PROP_POINTER);
	}
	ts->dex_name = "";

	ts->input_dev->name = "sec_touchscreen";
	sec_ts_set_input_prop(ts, ts->input_dev, INPUT_PROP_DIRECT);
#ifdef USE_OPEN_CLOSE
	ts->input_dev->open = sec_ts_input_open;
	ts->input_dev->close = sec_ts_input_close;
#endif
	ts->input_dev_touch = ts->input_dev;

	ret = input_register_device(ts->input_dev);
	if (ret) {
		input_err(true, &ts->client->dev,
			  "%s: Unable to register %s input device 0x%x\n",
			  __func__, ts->input_dev->name, ret);
		return SEC_TS_ERR_REG_INPUT_DEV;
	}

	if (ts->plat_data->support_dex) {
		ret = input_register_device(ts->input_dev_pad);
		if (ret) {
			input_err(true, &ts->client->dev,
				  "%s: Unable to register %s input device 0x%x\n",
				  __func__, ts->input_dev_pad->name, ret);
			return SEC_TS_ERR_REG_INPUT_PAD_DEV;
		}
	}

	return SEC_TS_ERR_NA;
}

static void sec_ts_device_init(struct sec_ts_data *ts)
{
#if (1) //!defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
	sec_ts_raw_device_init(ts);
#endif
	sec_ts_fn_init(ts);

#ifdef SEC_TS_SUPPORT_CUSTOMLIB
	sec_ts_check_custom_library(ts);
	if (ts->use_customlib)
		sec_ts_set_custom_library(ts);
#endif
}

static int sec_ts_heatmap_init(struct sec_ts_data *ts)
{
	int ret = 0;

	if (ts->heatmap_init_done) {
		input_info(true, &ts->client->dev, "%s: already init done!\n",
			__func__);
		return ret;
	}

	input_info(true, &ts->client->dev, "%s\n", __func__);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	/*
	 * Heatmap_probe must be called before irq routine is registered,
	 * because heatmap_read is called from the irq context.
	 * If the ISR runs before heatmap_probe is finished, it will invoke
	 * heatmap_read and cause NPE, since read_frame would not yet be set.
	 */
	ts->v4l2.parent_dev = &ts->client->dev;
	ts->v4l2.input_dev = ts->input_dev;
	ts->v4l2.read_frame = read_heatmap_raw;
	ts->v4l2.width = ts->tx_count;
	ts->v4l2.height = ts->rx_count;
	/* 120 Hz operation */
	ts->v4l2.timeperframe.numerator = 1;
	ts->v4l2.timeperframe.denominator = 120;
	ret = heatmap_probe(&ts->v4l2);
	if (ret == 0) {
		ts->heatmap_init_done = true;
	} else {
		input_err(true, &ts->client->dev,
			"%s: fail! ret %d\n", __func__, ret);
	}
#endif
	return ret;
}

#ifdef USE_CHARGER_WORK
static struct notifier_block sec_ts_psy_nb;
#endif

#ifdef I2C_INTERFACE
static int sec_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
#else
static int sec_ts_probe(struct spi_device *client)
#endif
{
	struct sec_ts_data *ts;
	struct sec_ts_plat_data *pdata;
	int ret = 0;

	input_info(true, &client->dev, "%s\n", __func__);

#ifdef I2C_INTERFACE
	input_info(true, &client->dev, "%s: I2C interface\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		input_err(true, &client->dev, "%s: EIO err!\n", __func__);
		return -EIO;
	}
#else
	if (client->controller->rt == false) {
		client->rt = true;
		ret = spi_setup(client);
		if (ret < 0) {
			input_err(true, &client->dev, "%s: setup SPI rt failed(%d)\n",
				  __func__, ret);
		}
	}
	input_info(true, &client->dev, "%s: SPI interface(%d Hz)\n",
		   __func__, client->max_speed_hz);
#endif
	/* parse dt */
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct sec_ts_plat_data), GFP_KERNEL);

		if (!pdata) {
			input_err(true, &client->dev,
				"%s: Failed to allocate platform data\n",
				__func__);
			goto error_allocate_pdata;
		}

		client->dev.platform_data = pdata;

		ret = sec_ts_parse_dt(client);
		if (ret) {
			input_err(true, &client->dev,
				  "%s: Failed to parse dt\n", __func__);
			goto error_allocate_mem;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			input_err(true, &client->dev,
				  "%s: No platform data found\n", __func__);
			goto error_allocate_pdata;
		}
	}

	if (sec_ts_check_for_deferred_regulators(&client->dev)) {
		input_err(true, &client->dev,
				"sec_ts deferring for power regulators\n");
		ret = -EPROBE_DEFER;
		goto error_allocate_mem;
	}

	if (!pdata->power) {
		input_err(true, &client->dev, "%s: No power contorl found\n",
			  __func__);
		goto error_allocate_mem;
	}

	pdata->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pdata->pinctrl))
		input_err(true, &client->dev, "%s: could not get pinctrl\n",
			  __func__);

	ts = kzalloc(sizeof(struct sec_ts_data), GFP_KERNEL);
	if (!ts)
		goto error_allocate_mem;

	ts->client = client;
	ts->plat_data = pdata;
	ts->crc_addr = 0x0001FE00;
	ts->fw_addr = 0x00002000;
	ts->para_addr = 0x18000;
	ts->flash_page_size = SEC_TS_FW_BLK_SIZE_DEFAULT;
	ts->sec_ts_read = sec_ts_read;
	ts->sec_ts_read_heap = sec_ts_read_heap;
	ts->sec_ts_write = sec_ts_write;
	ts->sec_ts_write_burst = sec_ts_write_burst;
	ts->sec_ts_write_burst_heap = sec_ts_write_burst_heap;
	ts->sec_ts_read_bulk = sec_ts_read_bulk;
	ts->sec_ts_read_bulk_heap = sec_ts_read_bulk_heap;
	ts->io_burstmax = pdata->io_burstmax;
#ifdef USE_POWER_RESET_WORK
	INIT_DELAYED_WORK(&ts->reset_work, sec_ts_reset_work);
#endif
	INIT_WORK(&ts->suspend_work, sec_ts_suspend_work);
	INIT_WORK(&ts->resume_work, sec_ts_resume_work);
#ifdef USE_CHARGER_WORK
	INIT_WORK(&ts->charger_work, sec_ts_charger_work);
#endif
	ts->event_wq = alloc_workqueue("sec_ts-event-queue", WQ_UNBOUND |
					 WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!ts->event_wq) {
		input_err(true, &ts->client->dev,
			"%s: Cannot create work thread\n", __func__);
		ret = -ENOMEM;
		goto error_alloc_workqueue;
	}

	init_completion(&ts->bus_resumed);
	complete_all(&ts->bus_resumed);

#ifdef SEC_TS_FW_UPDATE_ON_PROBE
	INIT_WORK(&ts->fw_update_work, sec_ts_fw_update_work);
#else
	input_info(true, &ts->client->dev, "%s: fw update on probe disabled!\n",
		   __func__);
	ts->fw_update_wq = alloc_workqueue("sec_ts-fw-update-queue",
					    WQ_UNBOUND | WQ_HIGHPRI |
					    WQ_CPU_INTENSIVE, 1);
	if (!ts->fw_update_wq) {
		input_err(true, &ts->client->dev,
			  "%s: Can't alloc fw update work thread\n",
			  __func__);
		ret = -ENOMEM;
		goto error_alloc_fw_update_wq;
	}
	INIT_DELAYED_WORK(&ts->fw_update_work, sec_ts_fw_update_work);
#endif

	ts->is_fw_corrupted = false;

	/* Assume screen is on throughout probe */
	ts->bus_refmask = SEC_TS_BUS_REF_SCREEN_ON;
#ifdef I2C_INTERFACE
	i2c_set_clientdata(client, ts);
#else
	spi_set_drvdata(client, ts);
#endif

#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
	if (register_tbn(&ts->tbn_register_mask)) {
		ret = -ENODEV;
		input_err(true, &ts->client->dev,
			   "%s: Failed to register tbn context.\n", __func__);
		goto err_init_tbn;
	}
	input_info(true, &ts->client->dev, "%s: tbn_register_mask = %#x.\n",
		   __func__, ts->tbn_register_mask);
#endif

	if (gpio_is_valid(ts->plat_data->tsp_id))
		ts->tspid_val = gpio_get_value(ts->plat_data->tsp_id);

	if (gpio_is_valid(ts->plat_data->tsp_icid))
		ts->tspicid_val = gpio_get_value(ts->plat_data->tsp_icid);

	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		input_err(true, &ts->client->dev,
			  "%s: allocate device err!\n", __func__);
		ret = -ENOMEM;
		goto err_allocate_input_dev;
	}

	if (ts->plat_data->support_dex) {
		ts->input_dev_pad = input_allocate_device();
		if (!ts->input_dev_pad) {
			input_err(true, &ts->client->dev,
				  "%s: allocate device err!\n", __func__);
			ret = -ENOMEM;
			goto err_allocate_input_dev_pad;
		}
	}

	ts->touch_count = 0;
	ts->tid_palm_state = 0;
	ts->tid_grip_state = 0;
	ts->tid_touch_state = 0;
	ts->palms_leaved_once = false;
	ts->grips_leaved_once = false;

	ts->sec_ts_write = sec_ts_write;
	ts->sec_ts_read = sec_ts_read;
	ts->sec_ts_read_heap = sec_ts_read_heap;
	ts->sec_ts_read_customlib = sec_ts_read_from_customlib;

	ts->max_z_value = 0;
	ts->min_z_value = 0xFFFFFFFF;
	ts->sum_z_value = 0;

	mutex_init(&ts->bus_mutex);
	mutex_init(&ts->lock);
	mutex_init(&ts->device_mutex);
	mutex_init(&ts->io_mutex);
	mutex_init(&ts->eventlock);

	init_completion(&ts->resume_done);
	complete_all(&ts->resume_done);

	init_completion(&ts->boot_completed);
	complete_all(&ts->boot_completed);

	if (pdata->always_lpmode)
		ts->lowpower_mode |= SEC_TS_MODE_CUSTOMLIB_FORCE_KEY;
	else
		ts->lowpower_mode &= ~SEC_TS_MODE_CUSTOMLIB_FORCE_KEY;

	sec_ts_pinctrl_configure(ts, true);

	/* power enable */
	sec_ts_power(ts, true);
	mdelay(10);
	ret = gpio_direction_output(pdata->reset_gpio, 1);
	mdelay(10);
	ret = gpio_direction_output(pdata->reset_gpio, 0);
	mdelay(10);
	ret = gpio_direction_output(pdata->reset_gpio, 1);
	mdelay(10);
	if (!pdata->regulator_boot_on)
		sec_ts_delay(70);
	ts->power_status = SEC_TS_STATE_POWER_ON;
	ts->external_factory = false;
	ts->heatmap_init_done = false;
	ts->mutual_strength_heatmap.timestamp = 0;
	ts->mutual_strength_heatmap.size_x = 0;
	ts->mutual_strength_heatmap.size_y = 0;
	ts->mutual_strength_heatmap.data = NULL;
	ts->v4l2_mutual_strength_updated = false;

	ret = sec_ts_wait_for_ready(ts, SEC_TS_ACK_BOOT_COMPLETE);
	if (ret < 0) {
		u8 boot_status;
		/* Read the boot status in case device is in bootloader mode */
		ret = ts->sec_ts_read(ts, SEC_TS_READ_BOOT_STATUS,
					  &boot_status, 1);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				  "%s: could not read boot status. Assuming no device connected.\n",
				  __func__);
			ret = -EPROBE_DEFER;
			goto err_init;
		}

		switch (boot_status) {
		case SEC_TS_STATUS_BOOT_MODE:
			input_err(true, &ts->client->dev,
				"%s: boot timeout(status %#x)! Reflash FW to recover.\n",
				__func__, boot_status);
			sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_FW_UPDATE, true);
			ret = sec_ts_firmware_update_on_probe(ts, true);
			sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_FW_UPDATE, false);
			if (ret) {
				ts->is_fw_corrupted = true;
				ret = -EPROBE_DEFER;
				goto err_init;
			}
			break;
		case SEC_TS_STATUS_APP_MODE:
		default:
			input_err(true, &ts->client->dev,
				"%s: boot timeout(status %#x)! Reset system to recover.\n",
				__func__, boot_status);
			sec_ts_system_reset(ts, RESET_MODE_HW, true, false);
			break;
		}
	}

	input_info(true, &client->dev, "%s: power enable\n", __func__);

	if (ts->is_fw_corrupted == false) {
		switch (sec_ts_fw_init(ts)) {
		case SEC_TS_ERR_INIT:
			ret = -EPROBE_DEFER;
			goto err_init;
		case SEC_TS_ERR_ALLOC_FRAME:
			goto err_allocate_frame;
		case SEC_TS_ERR_ALLOC_FRAME_SS:
			goto err_allocate_frame_ss;
		case SEC_TS_ERR_ALLOC_GAINTABLE:
			goto err_allocate_gaintable;
		case SEC_TS_ERR_REG_INPUT_DEV:
			goto err_input_register_device;
		case SEC_TS_ERR_REG_INPUT_PAD_DEV:
			goto err_input_pad_register_device;
		}
	}

	cpu_latency_qos_add_request(&ts->pm_qos_req, PM_QOS_DEFAULT_VALUE);

	ts->ignore_charger_nb = 0;
	/* init motion filter mode */
	ts->use_default_mf = 0;
	ts->mf_state = SEC_TS_MF_FILTERED;

	/* init heatmap */
	if (ts->is_fw_corrupted == false) {
		ret = sec_ts_heatmap_init(ts);
		if (ret)
			goto err_irq;
	}

	input_info(true, &ts->client->dev, "%s: request_irq = %d\n", __func__,
			client->irq);

	ret = request_threaded_irq(client->irq, sec_ts_isr, sec_ts_irq_thread,
			ts->plat_data->irq_type, SEC_TS_NAME, ts);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			"%s: Unable to request threaded irq\n", __func__);
		goto err_heatmap;
	}

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
	ts->offload.caps.touch_offload_major_version =
			TOUCH_OFFLOAD_INTERFACE_MAJOR_VERSION;
	ts->offload.caps.touch_offload_minor_version =
			TOUCH_OFFLOAD_INTERFACE_MINOR_VERSION;
	ts->offload.caps.device_id = ts->plat_data->offload_id;
	ts->offload.caps.display_width = ts->plat_data->max_x + 1;
	ts->offload.caps.display_height = ts->plat_data->max_y + 1;
	ts->offload.caps.tx_size = ts->tx_count;
	ts->offload.caps.rx_size = ts->rx_count;
	ts->offload.caps.heatmap_size = HEATMAP_SIZE_FULL;
#ifdef I2C_INTERFACE
	ts->offload.caps.bus_type = BUS_TYPE_I2C;
	ts->offload.caps.bus_speed_hz = 1000000;
#else
	ts->offload.caps.bus_type = BUS_TYPE_SPI;
	ts->offload.caps.bus_speed_hz = client->max_speed_hz;
#endif

	/* Currently can only reliably read mutual and self strength heatmaps
	 * each frame. Cannot support other formats due to penalties associated
	 * with switching data types.
	 */
	ts->offload.caps.touch_data_types =
	    TOUCH_DATA_TYPE_COORD | TOUCH_DATA_TYPE_STRENGTH;
	ts->offload.caps.touch_scan_types =
	    TOUCH_SCAN_TYPE_MUTUAL | TOUCH_SCAN_TYPE_SELF;
	ts->offload.caps.context_channel_types =
			CONTEXT_CHANNEL_TYPE_DRIVER_STATUS;

	ts->offload.caps.continuous_reporting = true;
	ts->offload.caps.noise_reporting = false;
	ts->offload.caps.cancel_reporting = false;
	ts->offload.caps.rotation_reporting = true;
	ts->offload.caps.size_reporting = true;
	ts->offload.caps.auto_reporting = false;
	ts->offload.caps.filter_grip = true;
	ts->offload.caps.filter_palm = true;
	ts->offload.caps.num_sensitivity_settings = 1;

	ts->offload.hcallback = (void *)ts;
	ts->offload.report_cb = sec_ts_offload_report;
	touch_offload_init(&ts->offload);
#endif

#ifndef CONFIG_SEC_SYSFS
	sec_class = class_create(THIS_MODULE, "sec");
#endif

	device_init_wakeup(&client->dev, true);

	if (ts->is_fw_corrupted == false)
		sec_ts_device_init(ts);

#ifdef SEC_TS_FW_UPDATE_ON_PROBE
	schedule_work(&ts->fw_update_work);

	/* Do not finish probe without checking and flashing the firmware */
	flush_work(&ts->fw_update_work);
#else
	queue_delayed_work(ts->fw_update_wq, &ts->fw_update_work,
		    msecs_to_jiffies(SEC_TS_FW_UPDATE_DELAY_MS_AFTER_PROBE));
#endif

#if defined(CONFIG_TOUCHSCREEN_DUMP_MODE)
	dump_callbacks.inform_dump = dump_tsp_log;
	INIT_DELAYED_WORK(&ts->ghost_check, sec_ts_check_rawdata);
	p_ghost_check = &ts->ghost_check;
#endif

	ts_dup = ts;
	ts->probe_done = true;

	ts->wlc_online = false;
	ts->usb_present = false;
	ts->charger_mode = SEC_TS_BIT_CHARGER_MODE_NO;
	ts->wireless_psy = power_supply_get_by_name("wireless");
	ts->usb_psy = power_supply_get_by_name("usb");
#ifdef USE_CHARGER_WORK
	ts->psy_nb = sec_ts_psy_nb;
	ret = power_supply_reg_notifier(&ts->psy_nb);
	if (ret < 0)
		input_err(true, &ts->client->dev, "psy notifier register failed\n");
#endif
	input_info(true, &ts->client->dev, "%s: done\n", __func__);
	input_log_fix();

	return 0;

	/* need to be enabled when new goto statement is added */
/*
 *	sec_ts_fn_remove(ts);
 *	free_irq(client->irq, ts);
 **/
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
	touch_offload_cleanup(&ts->offload);
#endif

err_heatmap:
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	heatmap_remove(&ts->v4l2);
err_irq:
#endif
	cpu_latency_qos_remove_request(&ts->pm_qos_req);
	if (ts->plat_data->support_dex) {
		input_unregister_device(ts->input_dev_pad);
		ts->input_dev_pad = NULL;
	}
err_input_pad_register_device:
	input_unregister_device(ts->input_dev);
	ts->input_dev = NULL;
	ts->input_dev_touch = NULL;
err_input_register_device:
#ifdef USE_STIM_PAD
	kfree(ts->gainTable);
#endif
err_allocate_gaintable:
	kfree(ts->pFrameSS);
err_allocate_frame_ss:
	kfree(ts->pFrame);
err_allocate_frame:
err_init:
	sec_ts_power(ts, false);
	if (ts->plat_data->support_dex) {
		if (ts->input_dev_pad)
			input_free_device(ts->input_dev_pad);
	}
err_allocate_input_dev_pad:
	if (ts->input_dev)
		input_free_device(ts->input_dev);
err_allocate_input_dev:
#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
	if (ts->tbn_register_mask)
		unregister_tbn(&ts->tbn_register_mask);
err_init_tbn:
#endif

#ifndef SEC_TS_FW_UPDATE_ON_PROBE
	if (ts->fw_update_wq)
		destroy_workqueue(ts->fw_update_wq);
error_alloc_fw_update_wq:
#endif

	if (ts->event_wq)
		destroy_workqueue(ts->event_wq);
error_alloc_workqueue:
	kfree(ts);

error_allocate_mem:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
	if (gpio_is_valid(pdata->tsp_id))
		gpio_free(pdata->tsp_id);
	if (gpio_is_valid(pdata->tsp_icid))
		gpio_free(pdata->tsp_icid);
	if (gpio_is_valid(pdata->switch_gpio))
		gpio_free(pdata->switch_gpio);
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);

error_allocate_pdata:
	if (ret == -ECONNREFUSED)
		sec_ts_delay(100);
	if (ret != -EPROBE_DEFER)
		ret = -ENODEV;
#ifdef CONFIG_TOUCHSCREEN_DUMP_MODE
	p_ghost_check = NULL;
#endif
	ts_dup = NULL;
	input_err(true, &client->dev, "%s: failed(%d)\n", __func__, ret);
	input_log_fix();
	return ret;
}

void sec_ts_unlocked_release_all_finger(struct sec_ts_data *ts)
{
	int i;
	s64 ms_delta;

	for (i = 0; i < MAX_SUPPORT_TOUCH_COUNT; i++) {
		input_mt_slot(ts->input_dev, i);
		if (ts->plat_data->support_mt_pressure)
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
					   false);

		if ((ts->coord[i].action == SEC_TS_COORDINATE_ACTION_PRESS) ||
			(ts->coord[i].action ==
			 SEC_TS_COORDINATE_ACTION_MOVE)) {

			input_info(true, &ts->client->dev,
				"%s: [RA] tID: %d mc: %d tc: %u v: %02X%02X cal: %02X(%02X) id(%d,%d)\n",
				__func__, i,
				ts->coord[i].mcount, ts->touch_count,
				ts->plat_data->img_version_of_ic[2],
				ts->plat_data->img_version_of_ic[3],
				ts->cal_status, ts->nv, ts->tspid_val,
				ts->tspicid_val);

			ts->coord[i].ktime_released = ktime_get();
			ms_delta = ktime_ms_delta(ts->coord[i].ktime_released,
						ts->coord[i].ktime_pressed);
			if (ts->longest_duration < ms_delta)
				ts->longest_duration = ms_delta;

			/* special case to push into kfifo during release all fingers. */
			sec_ts_kfifo_push_coord(ts, i);
		}

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
		ts->offload.coords[i].status = COORD_STATUS_INACTIVE;
		ts->offload.coords[i].major = 0;
		ts->offload.coords[i].minor = 0;
		ts->offload.coords[i].pressure = 0;
		ts->offload.coords[i].rotation = 0;
#endif
		ts->coord[i].action = SEC_TS_COORDINATE_ACTION_RELEASE;
		ts->coord[i].mcount = 0;
	}

	input_mt_slot(ts->input_dev, 0);

	input_report_key(ts->input_dev, BTN_TOUCH, false);
	input_report_key(ts->input_dev, BTN_TOOL_FINGER, false);
#ifdef SW_GLOVE
	input_report_switch(ts->input_dev, SW_GLOVE, false);
#endif
	ts->touchkey_glove_mode_status = false;
	ts->touch_count = 0;
	ts->check_multi = 0;
	ts->tid_palm_state = 0;
	ts->tid_grip_state = 0;
	ts->tid_touch_state = 0;
	ts->palms_leaved_once = false;
	ts->grips_leaved_once = false;

#ifdef KEY_SIDE_GESTURE
	if (ts->plat_data->support_sidegesture) {
		input_report_key(ts->input_dev, KEY_SIDE_GESTURE, 0);
		input_report_key(ts->input_dev, KEY_SIDE_GESTURE_LEFT, 0);
		input_report_key(ts->input_dev, KEY_SIDE_GESTURE_RIGHT, 0);
	}
#endif
	input_report_key(ts->input_dev, KEY_HOMEPAGE, 0);
	input_sync(ts->input_dev);
}

void sec_ts_locked_release_all_finger(struct sec_ts_data *ts)
{
	mutex_lock(&ts->eventlock);
	sec_ts_unlocked_release_all_finger(ts);
	mutex_unlock(&ts->eventlock);
}

#ifdef USE_POWER_RESET_WORK
static void sec_ts_reset_work(struct work_struct *work)
{
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data,
							reset_work.work);

	ts->reset_is_on_going = true;
	input_info(true, &ts->client->dev, "%s\n", __func__);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_RESET, true);

	sec_ts_stop_device(ts);

	sec_ts_delay(30);

	sec_ts_start_device(ts);

	ts->reset_is_on_going = false;
	ts->plat_data->is_heatmap_enabled = false;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_RESET, false);
}
#endif

void sec_ts_read_init_info(struct sec_ts_data *ts)
{
#ifndef CONFIG_SEC_FACTORY
	struct sec_ts_test_mode mode;
	char para = TO_TOUCH_MODE;
#endif
#ifdef USE_PRESSURE_SENSOR
	unsigned char data[18] = { 0 };
#endif
	int ret;

	ts->nv = get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_FAC_RESULT);
	ts->cal_count = get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_CAL_COUNT);
	ts->pressure_cal_base = get_tsp_nvm_data(ts,
				SEC_TS_NVM_OFFSET_PRESSURE_BASE_CAL_COUNT);
	ts->pressure_cal_delta = get_tsp_nvm_data(ts,
				SEC_TS_NVM_OFFSET_PRESSURE_DELTA_CAL_COUNT);

	input_info(true, &ts->client->dev,
		    "%s: fac_nv: %02X, cal_count: %02X\n",
		    __func__, ts->nv, ts->cal_count);

#ifdef PAT_CONTROL
	ts->tune_fix_ver = (get_tsp_nvm_data(ts,
				SEC_TS_NVM_OFFSET_TUNE_VERSION) << 8) |
			    get_tsp_nvm_data(ts,
				SEC_TS_NVM_OFFSET_TUNE_VERSION + 1);
	input_info(true, &ts->client->dev,
	    "%s: tune_fix_ver [%04X]\n", __func__, ts->tune_fix_ver);
#endif

#ifdef USE_PRESSURE_SENSOR
	ret = ts->sec_ts_read(ts, SEC_TS_CMD_SET_GET_PRESSURE, data, 18);
	if (ret < 0)
		return;

	ts->pressure_left = ((data[16] << 8) | data[17]);
	ts->pressure_center = ((data[8] << 8) | data[9]);
	ts->pressure_right = ((data[0] << 8) | data[1]);
	input_info(true, &ts->client->dev,
		"%s: left: %d, center: %d, right: %d\n", __func__,
		ts->pressure_left, ts->pressure_center, ts->pressure_right);
#endif

#ifndef CONFIG_SEC_FACTORY
	/* run self-test */
	disable_irq(ts->client->irq);
	execute_selftest(ts,
		TEST_OPEN | TEST_NODE_VARIANCE |
		TEST_SHORT | TEST_SELF_NODE | TEST_NOT_SAVE);
	enable_irq(ts->client->irq);

	input_info(true, &ts->client->dev, "%s: %02X %02X %02X %02X\n",
		__func__, ts->ito_test[0], ts->ito_test[1]
		, ts->ito_test[2], ts->ito_test[3]);

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
	if (ret < 0)
		input_err(true, &ts->client->dev, "%s: Failed to set\n",
				__func__);

	sec_ts_delay(350);

	/* run ambient read */
	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_AMBIENT_DATA;
	mode.allnode = TEST_MODE_ALL_NODE;

	sec_ts_read_raw_data(ts, NULL, &mode);
#endif

	input_log_fix();
}

static void sec_ts_fw_update_work(struct work_struct *work)
{
#ifdef SEC_TS_FW_UPDATE_ON_PROBE
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data,
					      fw_update_work);
#else
	struct delayed_work *fw_update_work = container_of(work,
					struct delayed_work, work);
	struct sec_ts_data *ts = container_of(fw_update_work,
					struct sec_ts_data, fw_update_work);
#endif

	int ret;

	input_info(true, &ts->client->dev,
		   "%s: Beginning firmware update after probe.\n", __func__);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_FW_UPDATE, true);

	ret = sec_ts_firmware_update_on_probe(ts, false);
	if (ret < 0)
		input_info(true, &ts->client->dev,
			   "%s: firmware update was unsuccessful.\n",
			   __func__);

	if (ts->is_fw_corrupted == true && ret == 0) {
		ret = sec_ts_fw_init(ts);
		if (ret == SEC_TS_ERR_NA) {
			ts->is_fw_corrupted = false;
			sec_ts_device_init(ts);
		} else {
			input_info(true, &ts->client->dev,
				"%s: fail to sec_ts_fw_init 0x%x\n",
				__func__, ret);
		}
	}

	if (ts->is_fw_corrupted == false)
		sec_ts_read_init_info(ts);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_FW_UPDATE, false);

	ret = register_panel_bridge(ts);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: register_panel_bridge failed. ret = 0x%08X\n",
			  __func__, ret);
}

int sec_ts_set_lowpowermode(struct sec_ts_data *ts, u8 mode)
{
	int ret;
	int retrycnt = 0;
	u8 data;
	char para = 0;

	input_err(true, &ts->client->dev, "%s: %s(%X)\n", __func__,
			mode == TO_LOWPOWER_MODE ? "ENTER" : "EXIT",
			ts->lowpower_mode);

	if (mode) {
		#ifdef SEC_TS_SUPPORT_CUSTOMLIB
		if (ts->use_customlib)
			sec_ts_set_custom_library(ts);
		#endif

		data = (ts->lowpower_mode & SEC_TS_MODE_LOWPOWER_FLAG) >> 1;
		ret = sec_ts_write(ts, SEC_TS_CMD_WAKEUP_GESTURE_MODE,
				   &data, 1);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				  "%s: Failed to set\n", __func__);
	}

retry_pmode:
	ret = sec_ts_write(ts, SEC_TS_CMD_SET_POWER_MODE, &mode, 1);
	if (ret < 0)
		input_err(true, &ts->client->dev,
				"%s: failed\n", __func__);
	sec_ts_delay(50);

	/* read data */

	ret = sec_ts_read(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: read power mode failed!\n", __func__);
	else
		input_info(true, &ts->client->dev,
			   "%s: power mode - write(%d) read(%d)\n",
			   __func__, mode, para);

	if (mode != para) {
		retrycnt++;
		if (retrycnt < 5)
			goto retry_pmode;
	}

	ret = sec_ts_write(ts, SEC_TS_CMD_CLEAR_EVENT_STACK, NULL, 0);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: write clear event failed\n", __func__);


	sec_ts_locked_release_all_finger(ts);

	if (device_may_wakeup(&ts->client->dev)) {
		if (mode)
			enable_irq_wake(ts->client->irq);
		else
			disable_irq_wake(ts->client->irq);
	}

	ts->lowpower_status = mode;
	input_info(true, &ts->client->dev, "%s: end\n", __func__);

	return ret;
}

#ifdef USE_OPEN_CLOSE
static int sec_ts_input_open(struct input_dev *dev)
{
	struct sec_ts_data *ts = input_get_drvdata(dev);
	int ret;

	ts->input_closed = false;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_INPUT_DEV, true);

	if (ts->lowpower_status) {
#ifdef USE_RESET_EXIT_LPM
		schedule_delayed_work(&ts->reset_work,
				      msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
#else
		sec_ts_set_lowpowermode(ts, TO_TOUCH_MODE);
#endif
		ts->power_status = SEC_TS_STATE_POWER_ON;
	} else {
		ret = sec_ts_start_device(ts);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				  "%s: Failed to start device\n", __func__);
	}

	/* because edge and dead zone will recover soon */
	sec_ts_set_grip_type(ts, ONLY_EDGE_HANDLER);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_INPUT_DEV, false);

	return 0;
}

static void sec_ts_input_close(struct input_dev *dev)
{
	struct sec_ts_data *ts = input_get_drvdata(dev);

	ts->input_closed = true;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_INPUT_DEV, true);

	cancel_work_sync(&ts->suspend_work);
	cancel_work_sync(&ts->resume_work);

#ifdef USE_POWER_RESET_WORK
	cancel_delayed_work(&ts->reset_work);
#endif

#ifndef CONFIG_SEC_FACTORY
	ts->lowpower_mode |= SEC_TS_MODE_CUSTOMLIB_FORCE_KEY;
#endif
	if (ts->lowpower_mode) {
		sec_ts_set_lowpowermode(ts, TO_LOWPOWER_MODE);
		ts->power_status = SEC_TS_STATE_LPM;
	} else {
		sec_ts_stop_device(ts);
	}

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_INPUT_DEV, false);
}
#endif

#ifdef I2C_INTERFACE
static int sec_ts_remove(struct i2c_client *client)
#else
static int sec_ts_remove(struct spi_device *client)
#endif
{
#ifdef I2C_INTERFACE
	struct sec_ts_data *ts = i2c_get_clientdata(client);
#else
	struct sec_ts_data *ts = spi_get_drvdata(client);
#endif
	/* const struct sec_ts_plat_data *pdata = ts->plat_data; */
	bool fw_update_cancelled = false;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	if (ts_dup == NULL || ts->probe_done == false)
		return 0;

	/* Force the bus active throughout removal of the client */
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_FORCE_ACTIVE, true);

	power_supply_unreg_notifier(&ts->psy_nb);

	cancel_work_sync(&ts->suspend_work);
	cancel_work_sync(&ts->resume_work);
#ifdef USE_CHARGER_WORK
	cancel_work_sync(&ts->charger_work);
#endif
	destroy_workqueue(ts->event_wq);

#ifdef SEC_TS_FW_UPDATE_ON_PROBE
	fw_update_cancelled = cancel_work_sync(&ts->fw_update_work);
#else
	fw_update_cancelled = cancel_delayed_work_sync(&ts->fw_update_work);
	destroy_workqueue(ts->fw_update_wq);
#endif
	if (!fw_update_cancelled)
		unregister_panel_bridge(&ts->panel_bridge);

	disable_irq_nosync(ts->client->irq);
	free_irq(ts->client->irq, ts);
	input_info(true, &ts->client->dev, "%s: irq disabled\n", __func__);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
	touch_offload_cleanup(&ts->offload);
#endif

#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	heatmap_remove(&ts->v4l2);
#endif

	cpu_latency_qos_remove_request(&ts->pm_qos_req);

#ifdef USE_POWER_RESET_WORK
	cancel_delayed_work_sync(&ts->reset_work);
	flush_delayed_work(&ts->reset_work);

	input_info(true, &ts->client->dev, "%s: flush queue\n", __func__);

#endif

	sec_ts_fn_remove(ts);

#ifdef CONFIG_TOUCHSCREEN_DUMP_MODE
	p_ghost_check = NULL;
#endif
	device_init_wakeup(&client->dev, false);

	ts->lowpower_mode = false;
	ts->probe_done = false;

	if (ts->plat_data->support_dex) {
		input_mt_destroy_slots(ts->input_dev_pad);
		input_unregister_device(ts->input_dev_pad);
	}

	ts->input_dev = ts->input_dev_touch;
	input_mt_destroy_slots(ts->input_dev);
	input_unregister_device(ts->input_dev);

	ts->input_dev_pad = NULL;
	ts->input_dev = NULL;
	ts->input_dev_touch = NULL;
	ts_dup = NULL;

	/* need to do software reset for next sec_ts_probe() without error */
	ts->sec_ts_write(ts, SEC_TS_CMD_SW_RESET, NULL, 0);

	ts->plat_data->power(ts, false);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
	if (ts->tbn_register_mask)
		unregister_tbn(&ts->tbn_register_mask);
#endif

	if (gpio_is_valid(ts->plat_data->irq_gpio))
		gpio_free(ts->plat_data->irq_gpio);
	if (gpio_is_valid(ts->plat_data->switch_gpio))
		gpio_free(ts->plat_data->switch_gpio);
	if (gpio_is_valid(ts->plat_data->reset_gpio))
		gpio_free(ts->plat_data->reset_gpio);

	sec_ts_raw_device_exit(ts);
#ifndef CONFIG_SEC_SYSFS
	class_destroy(sec_class);
#endif

#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	kfree(ts->heatmap_buff);
	kfree(ts->encoded_buff);
#endif
#ifdef USE_STIM_PAD
	kfree(ts->gainTable);
#endif
	kfree(ts->pFrameSS);
	kfree(ts->pFrame);
	kfree(ts);
	return 0;
}

#ifdef I2C_INTERFACE
static void sec_ts_shutdown(struct i2c_client *client)
#else
static void sec_ts_shutdown(struct spi_device *client)
#endif
{
	pr_info("%s\n", __func__);
	if (ts_dup)
		sec_ts_remove(client);
}

int sec_ts_stop_device(struct sec_ts_data *ts)
{
	input_info(true, &ts->client->dev, "%s\n", __func__);

	mutex_lock(&ts->device_mutex);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: already power off\n", __func__);
		goto out;
	}

	ts->power_status = SEC_TS_STATE_POWER_OFF;

	disable_irq(ts->client->irq);
	sec_ts_locked_release_all_finger(ts);

	ts->plat_data->power(ts, false);

	if (ts->plat_data->enable_sync)
		ts->plat_data->enable_sync(false);

	sec_ts_pinctrl_configure(ts, false);

out:
	mutex_unlock(&ts->device_mutex);
	return 0;
}

int sec_ts_start_device(struct sec_ts_data *ts)
{
	int ret;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	sec_ts_pinctrl_configure(ts, true);

	mutex_lock(&ts->device_mutex);

	if (ts->power_status == SEC_TS_STATE_POWER_ON) {
		input_info(true, &ts->client->dev,
			  "%s: already power on\n", __func__);
		goto out;
	}

	sec_ts_locked_release_all_finger(ts);

	ts->plat_data->power(ts, true);
	sec_ts_delay(70);
	ts->power_status = SEC_TS_STATE_POWER_ON;
	sec_ts_wait_for_ready(ts, SEC_TS_ACK_BOOT_COMPLETE);

	if (ts->plat_data->enable_sync)
		ts->plat_data->enable_sync(true);

	if (ts->flip_enable) {
		ret = sec_ts_write(ts, SEC_TS_CMD_SET_COVERTYPE,
				   &ts->cover_cmd, 1);

		ts->touch_functions = ts->touch_functions |
				SEC_TS_BIT_SETFUNC_COVER;
		input_info(true, &ts->client->dev,
				"%s: cover cmd write type: %d, mode: %x, ret: %d",
				__func__, ts->touch_functions,
				ts->cover_cmd, ret);
	} else {
		ts->touch_functions = (ts->touch_functions &
				       (~SEC_TS_BIT_SETFUNC_COVER));
		input_info(true, &ts->client->dev,
			"%s: cover open, not send cmd", __func__);
	}

	ts->touch_functions = ts->touch_functions |
				SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC;
	ret = sec_ts_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION,
			   (u8 *)&ts->touch_functions, 2);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			"%s: Failed to send touch function command", __func__);

	#ifdef SEC_TS_SUPPORT_CUSTOMLIB
	if (ts->use_customlib)
		sec_ts_set_custom_library(ts);
	#endif

	sec_ts_set_grip_type(ts, ONLY_EDGE_HANDLER);

	if (ts->dex_mode) {
		input_info(true, &ts->client->dev,
			   "%s: set dex mode\n", __func__);
		ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_DEX_MODE,
				       &ts->dex_mode, 1);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				"%s: failed to set dex mode %x\n",
				__func__, ts->dex_mode);
	}

	if (ts->brush_mode) {
		input_info(true, &ts->client->dev,
			   "%s: set brush mode\n", __func__);
		ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_BRUSH_MODE,
				       &ts->brush_mode, 1);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				"%s: failed to set brush mode\n", __func__);
	}

	if (ts->touchable_area) {
		input_info(true, &ts->client->dev,
			   "%s: set 16:9 mode\n", __func__);
		ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_TOUCHABLE_AREA,
				       &ts->touchable_area, 1);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				"%s: failed to set 16:9 mode\n", __func__);
	}

	/* Sense_on */
	ret = sec_ts_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: fail to write Sense_on\n", __func__);

	enable_irq(ts->client->irq);

out:
	mutex_unlock(&ts->device_mutex);
	return 0;
}

#ifdef CONFIG_PM
static int sec_ts_pm_suspend(struct device *dev)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (ts->bus_refmask) {
		input_info(true, &ts->client->dev,
			"%s: bus_refmask 0x%X\n", __func__, ts->bus_refmask);
	}

	/* Flush work in case a suspend is in progress */
	flush_workqueue(ts->event_wq);

	if (ts->power_status != SEC_TS_STATE_SUSPEND) {
		input_err(true, &ts->client->dev,
			"%s: can't suspend because touch bus is in use!\n",
			__func__);
		if (ts->bus_refmask == SEC_TS_BUS_REF_BUGREPORT) {
			s64 delta_ms = ktime_ms_delta(ktime_get(),
						      ts->bugreport_ktime_start);

			if (delta_ms > 30 * MSEC_PER_SEC) {
				sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_BUGREPORT, false);
				pm_relax(&ts->client->dev);
				ts->bugreport_ktime_start = 0;
				input_err(true, &ts->client->dev,
					  "%s: force release SEC_TS_BUS_REF_BUGREPORT(delta: %lld)!\n",
					   __func__, delta_ms);
			}
		}
		return -EBUSY;
	}

	if (ts->lowpower_mode)
		reinit_completion(&ts->resume_done);

	return 0;
}

static int sec_ts_pm_resume(struct device *dev)
{
	struct sec_ts_data *ts = dev_get_drvdata(dev);

	if (ts->lowpower_mode)
		complete_all(&ts->resume_done);

	return 0;
}
#endif

static const struct i2c_device_id sec_ts_id[] = {
	{ SEC_TS_NAME, 0 },
	{ },
};

#ifdef CONFIG_PM
static const struct dev_pm_ops sec_ts_dev_pm_ops = {
	.suspend = sec_ts_pm_suspend,
	.resume = sec_ts_pm_resume,
};
#endif

/*
 * Configure the switch GPIO to toggle bus master between AP and SLPI.
 * gpio_value takes one of
 * { SEC_SWITCH_GPIO_VALUE_SLPI_MASTER, SEC_SWITCH_GPIO_VALUE_AP_MASTER }
 */
static void sec_set_switch_gpio(struct sec_ts_data *ts, int gpio_value)
{
	int retval;
	unsigned int gpio = ts->plat_data->switch_gpio;

	if (!gpio_is_valid(gpio))
		return;

	input_info(true, &ts->client->dev, "%s: toggling switch to %s\n",
		   __func__, gpio_value == SEC_SWITCH_GPIO_VALUE_AP_MASTER ?
		   "AP" : "SLPI");

	retval = gpio_direction_output(gpio, gpio_value);
	if (retval < 0)
		input_err(true, &ts->client->dev,
			  "%s: Failed to toggle switch_gpio, err = %d\n",
			  __func__, retval);
}

static void sec_ts_suspend_work(struct work_struct *work)
{
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data,
					      suspend_work);
	int ret = 0;

	input_info(true, &ts->client->dev, "%s: int_cnt %llu.\n", __func__, ts->int_cnt);
	input_info(true, &ts->client->dev, "%s: encoded skipped %d/%d\n",
		   __func__, ts->plat_data->encoded_skip_counter,
		   ts->plat_data->encoded_frame_counter);
	if (ts->plat_data->grip_prescreen_mode != GRIP_PRESCREEN_OFF) {
		input_info(true, &ts->client->dev, "%s: grip prescreened frames %d.\n",
			__func__, sec_ts_ptflib_get_grip_prescreen_frames(ts));
	}

	if (ts->power_status == SEC_TS_STATE_SUSPEND) {
		input_err(true, &ts->client->dev, "%s: already suspended.\n",
			  __func__);
		return;
	}

	mutex_lock(&ts->device_mutex);
	/*
	 * Do the system reset to initialize the FW to the default state
	 * before handing over to AOC. And, recover the charger mode to
	 * have the AFE setting as the original one.
	 */
	sec_ts_system_reset(ts, RESET_MODE_AUTO, true, false);
	ret = ts->sec_ts_write(ts, SET_TS_CMD_SET_CHARGER_MODE,
			       &ts->charger_mode, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: write reg %#x %#x failed, returned %i\n",
			__func__, SET_TS_CMD_SET_CHARGER_MODE, ts->charger_mode,
			ret);
	} else {
		input_info(true, &ts->client->dev, "%s: set charger mode %#x\n",
			__func__, ts->charger_mode);
	}

	reinit_completion(&ts->bus_resumed);
	sec_ts_enable_fw_grip(ts, true);

	/* Stop T-IC */
	sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_SLEEP, TOUCH_MODE_STATE_STOP);
	ret = sec_ts_write(ts, SEC_TS_CMD_CLEAR_EVENT_STACK, NULL, 0);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			"%s: write clear event failed\n", __func__);

	disable_irq_nosync(ts->client->irq);
	sec_ts_locked_release_all_finger(ts);

	if (ts->plat_data->enable_sync)
		ts->plat_data->enable_sync(false);

	ts->power_status = SEC_TS_STATE_SUSPEND;

	sec_ts_pinctrl_configure(ts, false);

	sec_set_switch_gpio(ts, SEC_SWITCH_GPIO_VALUE_SLPI_MASTER);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
	if (ts->tbn_register_mask)
		tbn_release_bus(ts->tbn_register_mask);
#endif
	mutex_unlock(&ts->device_mutex);

	sec_ts_hc_dump(ts);
	sec_ts_debug_dump(ts);
}

static void sec_ts_resume_work(struct work_struct *work)
{
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data,
					      resume_work);
	u8 touch_mode[2] = {0};
	int ret = 0;

	input_info(true, &ts->client->dev, "%s: int_cnt %llu.\n", __func__, ts->int_cnt);
	ts->ktime_resume = ktime_get();
	ts->comm_err_count = 0;
	ts->hw_reset_count = 0;
	ts->longest_duration = 0;
	ts->pressed_count = 0;
	ts->palm_count = 0;
	ts->wet_count = 0;

	mutex_lock(&ts->device_mutex);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
	if (ts->tbn_register_mask)
		tbn_request_bus(ts->tbn_register_mask);
#endif

	sec_set_switch_gpio(ts, SEC_SWITCH_GPIO_VALUE_AP_MASTER);

	sec_ts_pinctrl_configure(ts, true);

	if (ts->power_status == SEC_TS_STATE_POWER_ON) {
		input_err(true, &ts->client->dev, "%s: already resumed.\n",
			  __func__);
		mutex_unlock(&ts->device_mutex);
		return;
	}

	sec_ts_locked_release_all_finger(ts);

	ts->power_status = SEC_TS_STATE_POWER_ON;

	ret = ts->sec_ts_read(ts, SEC_TS_CMD_CHG_SYSMODE, touch_mode,
			       sizeof(touch_mode));
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			"%s: read touch mode failed(%d)\n",
			__func__, ret);
		ret = sec_ts_system_reset(ts, RESET_MODE_HW, false, false);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				  "%s: reset failed! ret %d\n", __func__, ret);
		}
	} else {
		u8 power_mode = TO_TOUCH_MODE;
		u8 state_manage_on = { STATE_MANAGE_ON };

		input_info(true, &ts->client->dev,
			"%s: before resume: mode %#x, state %#x.\n",
			__func__, touch_mode[0], touch_mode[1]);

		/* Enable Normal scan. */
		ret = sec_ts_write(ts, SEC_TS_CMD_SET_POWER_MODE,
				   &power_mode, sizeof(power_mode));
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				  "%s: set power mode failed(%d)\n",
				  __func__, ret);
			ret = sec_ts_system_reset(ts, RESET_MODE_HW, false, false);
			if (ret < 0) {
				input_err(true, &ts->client->dev,
					  "%s: reset failed! ret %d\n", __func__, ret);
			}
		} else {
			/* Wait at least 50 ms for mode change. */
			sec_ts_delay(50);
		}

		ret = ts->sec_ts_read(ts, SEC_TS_CMD_CHG_SYSMODE, touch_mode,
				      sizeof(touch_mode));
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				  "%s: read touch mode failed(%d)\n",
				  __func__, ret);
		} else {
			input_info(true, &ts->client->dev,
				   "%s: after resume: mode %#x, state %#x.\n",
				   __func__, touch_mode[0], touch_mode[1]);
		}

		ret = sec_ts_write(ts, SEC_TS_CMD_STATEMANAGE_ON, &state_manage_on,
				sizeof(state_manage_on));
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				"%s: SEC_TS_CMD_STATEMANAGE_ON failed! ret %d\n",
				__func__, ret);
		}
	}

	if (ts->plat_data->enable_sync)
		ts->plat_data->enable_sync(true);

	ts->touch_functions =
	    ts->touch_functions | SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC;
	ret = sec_ts_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION,
			       (u8 *)&ts->touch_functions, 2);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: Failed to send touch function command.",
			  __func__);

#ifdef SEC_TS_SUPPORT_CUSTOMLIB
	if (ts->use_customlib)
		sec_ts_set_custom_library(ts);
#endif

	ts->plat_data->is_heatmap_enabled = false;
	ts->plat_data->encoded_frame_counter = 0;
	ts->plat_data->encoded_skip_counter = 0;

	if (ts->dex_mode) {
		input_info(true, &ts->client->dev, "%s: set dex mode.\n",
			   __func__);
		ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_DEX_MODE,
					   &ts->dex_mode, 1);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				  "%s: failed to set dex mode %x.\n", __func__,
				  ts->dex_mode);
	}

	if (ts->brush_mode) {
		input_info(true, &ts->client->dev, "%s: set brush mode.\n",
			   __func__);
		ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_BRUSH_MODE,
					   &ts->brush_mode, 1);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				  "%s: failed to set brush mode.\n", __func__);
	}

	if (ts->touchable_area) {
		input_info(true, &ts->client->dev, "%s: set 16:9 mode.\n",
			   __func__);
		ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_TOUCHABLE_AREA,
					   &ts->touchable_area, 1);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				  "%s: failed to set 16:9 mode.\n", __func__);
	}

	/* set charger mode */
	ret = ts->sec_ts_write(ts, SET_TS_CMD_SET_CHARGER_MODE,
			       &ts->charger_mode, 1);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: write reg %#x %#x failed, returned %i\n",
			__func__, SET_TS_CMD_SET_CHARGER_MODE, ts->charger_mode,
			ret);
	else
		input_info(true, &ts->client->dev, "%s: set charger mode %#x\n",
			__func__, ts->charger_mode);
#ifdef USE_CHARGER_WORK
	queue_work(ts->event_wq, &ts->charger_work);
#endif

	/* Sense_on */
	ret = sec_ts_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: failed to write Sense_on.\n", __func__);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
	/* Set touch_offload configuration */
	if (ts->offload.offload_running) {
		input_info(true, &ts->client->dev,
			   "applying touch_offload settings.\n");

		if (ts->offload.config.filter_grip) {
			sec_ts_enable_fw_grip(ts, false);
			sec_ts_enable_ptflib(ts, true);
		}
	}
#endif

	enable_irq(ts->client->irq);

	complete_all(&ts->bus_resumed);

	mutex_unlock(&ts->device_mutex);
}

#ifdef USE_CHARGER_WORK
static void sec_ts_charger_work(struct work_struct *work)
{
	int ret;
	union power_supply_propval prop = {0,};
	struct sec_ts_data *ts = container_of(work, struct sec_ts_data,
					      charger_work);
	u8 charger_mode = SEC_TS_BIT_CHARGER_MODE_NO;
	bool usb_present = ts->usb_present;
	bool wlc_online = ts->wlc_online;
	bool force_wlc = ts->force_wlc;
	const u64 debounce_ms = 500;

	/* usb case */
	if (ts->usb_psy != NULL) {
		ret = power_supply_get_property(ts->usb_psy,
						POWER_SUPPLY_PROP_PRESENT, &prop);
		if (ret == 0) {
			usb_present = !!prop.intval;
			if (usb_present)
				charger_mode = SEC_TS_BIT_CHARGER_MODE_WIRE_CHARGER;
		}
	}

	/* wlc case */
	wlc_online = false;
	if (ts->wireless_psy != NULL) {
		ret = power_supply_get_property(ts->wireless_psy,
					POWER_SUPPLY_PROP_ONLINE, &prop);
		if (ret == 0) {
			wlc_online = !!prop.intval;
			if (wlc_online)
				charger_mode =
				    SEC_TS_BIT_CHARGER_MODE_WIRELESS_CHARGER;
		}
	}

	/*
	* RTX case
	*    ret = power_supply_get_property(ts->wireless_psy,
	*                                    POWER_SUPPLY_PROP_RTX, &prop);
	* if (ret == 0)
	*  pr_debug("%s: RTX %s", __func__,
	*          (!!prop.intval) ? "ON" : "OFF");
	*/

	/* Check if any change for usb and wlc */
	if (usb_present == ts->usb_present &&
	    wlc_online == ts->wlc_online) {
		input_dbg(true, &ts->client->dev,
			"%s: usb_present(%d) and wlc_online(%d) no changed!",
			__func__, usb_present, wlc_online);
		return;
	}

	/* Force wlc case */
	if (usb_present &&
	    !wlc_online && ts->wlc_online &&
	    ktime_before(ts->wlc_changed_ktime,
		ktime_add_ms(ts->usb_changed_ktime, debounce_ms))) {
		force_wlc = true;
		charger_mode = SEC_TS_BIT_CHARGER_MODE_WIRELESS_CHARGER;
		input_info(true, &ts->client->dev,
			"%s: force wlc mode if usb present during wlc online.",
			__func__);
	} else {
		force_wlc = false;
	}

	input_info(true, &ts->client->dev,
		"%s: force_wlc(%d->%d), usb_present(%d->%d), wlc_online(%d->%d), charger_mode(%#x->%#x)",
		__func__,
		ts->force_wlc, force_wlc,
		ts->usb_present, usb_present,
		ts->wlc_online, wlc_online,
		ts->charger_mode, charger_mode);

	if (ts->charger_mode != charger_mode) {
		if (ts->power_status == SEC_TS_STATE_POWER_ON) {
			ret = ts->sec_ts_write(ts, SET_TS_CMD_SET_CHARGER_MODE,
				       &charger_mode, 1);
			if (ret < 0) {
				input_err(true, &ts->client->dev,
				"%s: write reg %#x %#x failed, returned %i\n",
				__func__, SET_TS_CMD_SET_CHARGER_MODE,
				charger_mode, ret);
				return;
			}

			input_info(true, &ts->client->dev,
				"%s: charger_mode change from %#x to %#x\n",
				__func__, ts->charger_mode, charger_mode);
		} else {
			input_info(true, &ts->client->dev,
				"%s: ONLY update charger_mode status from %#x to %#x, then will apply during resume\n",
				__func__, ts->charger_mode, charger_mode);
		}
		ts->charger_mode = charger_mode;
	}

	/* update final charger state */
	ts->wlc_online = wlc_online;
	ts->usb_present = usb_present;
	ts->force_wlc = force_wlc;
}
#endif

static void sec_ts_aggregate_bus_state(struct sec_ts_data *ts)
{
	input_dbg(true, &ts->client->dev, "%s: bus_refmask = 0x%02X.\n",
		  __func__, ts->bus_refmask);

	/* Complete or cancel any outstanding transitions */
	cancel_work_sync(&ts->suspend_work);
	cancel_work_sync(&ts->resume_work);

	if ((ts->bus_refmask == 0 &&
		ts->power_status == SEC_TS_STATE_SUSPEND) ||
	    (ts->bus_refmask != 0 &&
		ts->power_status != SEC_TS_STATE_SUSPEND))
		return;

	if (ts->bus_refmask == 0)
		queue_work(ts->event_wq, &ts->suspend_work);
	else
		queue_work(ts->event_wq, &ts->resume_work);
}

int sec_ts_set_bus_ref(struct sec_ts_data *ts, u16 ref, bool enable)
{
	int result = 0;

	mutex_lock(&ts->bus_mutex);

	input_dbg(true, &ts->client->dev, "%s: bus_refmask = 0x%02X.\n",
		  __func__, ref);

	if ((enable && (ts->bus_refmask & ref)) ||
	    (!enable && !(ts->bus_refmask & ref))) {
		input_dbg(true, &ts->client->dev,
			"%s: reference is unexpectedly set: mask=0x%04X, ref=0x%04X, enable=%d\n",
			__func__, ts->bus_refmask, ref, enable);
		mutex_unlock(&ts->bus_mutex);
		return -EINVAL;
	}

	if (enable) {
		/* IRQs can only keep the bus active. IRQs received while the
		 * bus is transferred to SLPI should be ignored.
		 */
		if (ref == SEC_TS_BUS_REF_IRQ && ts->bus_refmask == 0)
			result = -EAGAIN;
		else
			ts->bus_refmask |= ref;
	} else
		ts->bus_refmask &= ~ref;
	sec_ts_aggregate_bus_state(ts);

	mutex_unlock(&ts->bus_mutex);

	/* When triggering a wake, wait up to one second to resume. SCREEN_ON
	 * and IRQ references do not need to wait.
	 */
	if (enable &&
	    ref != SEC_TS_BUS_REF_SCREEN_ON && ref != SEC_TS_BUS_REF_IRQ) {
		wait_for_completion_timeout(&ts->bus_resumed, HZ);
		if (ts->power_status != SEC_TS_STATE_POWER_ON) {
			input_info(true, &ts->client->dev,
				   "%s: Failed to wake the touch bus.\n",
				   __func__);
			result = -ETIMEDOUT;
		}
	}

	return result;
}

struct drm_connector *get_bridge_connector(struct drm_bridge *bridge)
{
	struct drm_connector *connector;
	struct drm_connector_list_iter conn_iter;

	drm_connector_list_iter_begin(bridge->dev, &conn_iter);
	drm_for_each_connector_iter(connector, &conn_iter) {
		if (connector->encoder == bridge->encoder)
			break;
	}
	drm_connector_list_iter_end(&conn_iter);
	return connector;
}

static bool bridge_is_lp_mode(struct drm_connector *connector)
{
	if (connector && connector->state) {
		struct exynos_drm_connector_state *s =
			to_exynos_connector_state(connector->state);
		return s->exynos_mode.is_lp_mode;
	}
	return false;
}

static void panel_bridge_enable(struct drm_bridge *bridge)
{
	struct sec_ts_data *ts =
		container_of(bridge, struct sec_ts_data, panel_bridge);

	pr_debug("%s\n", __func__);
	if (!ts->is_panel_lp_mode)
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SCREEN_ON, true);
}

static void panel_bridge_disable(struct drm_bridge *bridge)
{
	struct sec_ts_data *ts =
		container_of(bridge, struct sec_ts_data, panel_bridge);

	if (bridge->encoder && bridge->encoder->crtc) {
		const struct drm_crtc_state *crtc_state = bridge->encoder->crtc->state;

		if (drm_atomic_crtc_effectively_active(crtc_state))
			return;
	}

	pr_debug("%s\n", __func__);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SCREEN_ON, false);
}

static void panel_bridge_mode_set(struct drm_bridge *bridge,
				  const struct drm_display_mode *mode,
				  const struct drm_display_mode *adjusted_mode)
{
	struct sec_ts_data *ts =
		container_of(bridge, struct sec_ts_data, panel_bridge);

	if (!ts->connector || !ts->connector->state)
		ts->connector = get_bridge_connector(bridge);

	ts->is_panel_lp_mode = bridge_is_lp_mode(ts->connector);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SCREEN_ON, !ts->is_panel_lp_mode);

	if (adjusted_mode) {
		int vrefresh = drm_mode_vrefresh(adjusted_mode);

		if (ts->display_refresh_rate != vrefresh) {
			input_dbg(true, &ts->client->dev,
				"%s: refresh rate(Hz) changed to %d from %d\n",
				__func__,  vrefresh, ts->display_refresh_rate);
			ts->display_refresh_rate = vrefresh;
		}
	}
}

static const struct drm_bridge_funcs panel_bridge_funcs = {
	.enable = panel_bridge_enable,
	.disable = panel_bridge_disable,
	.mode_set = panel_bridge_mode_set,
};

static int register_panel_bridge(struct sec_ts_data *ts)
{
	pr_debug("%s\n", __func__);
#ifdef CONFIG_OF
	ts->panel_bridge.of_node = ts->client->dev.of_node;
#endif
	ts->panel_bridge.funcs = &panel_bridge_funcs;
	drm_bridge_add(&ts->panel_bridge);

	return 0;
}

static void unregister_panel_bridge(struct drm_bridge *bridge)
{
	struct drm_bridge *node;

	pr_debug("%s\n", __func__);
	drm_bridge_remove(bridge);

	if (!bridge->dev) /* not attached */
		return;

	drm_modeset_lock(&bridge->dev->mode_config.connection_mutex, NULL);
	list_for_each_entry(node, &bridge->encoder->bridge_chain, chain_node)
		if (node == bridge) {
			if (bridge->funcs->detach)
				bridge->funcs->detach(bridge);
			list_del(&bridge->chain_node);
			break;
		}
	drm_modeset_unlock(&bridge->dev->mode_config.connection_mutex);
	bridge->dev = NULL;
}

/*
 * power supply callback
 */
#ifdef USE_CHARGER_WORK
static int sec_ts_psy_cb(struct notifier_block *nb,
			       unsigned long val, void *data)
{
	struct sec_ts_data *ts = container_of(nb, struct sec_ts_data, psy_nb);

	pr_debug("%s: val %lu", __func__, val);

	if (val != PSY_EVENT_PROP_CHANGED ||
	    ts->usb_psy == NULL ||
	    (ts->wireless_psy != data && ts->usb_psy != data) ||
	    ts->ignore_charger_nb == 1)
		return NOTIFY_OK;

	if (ts->usb_psy == data) {
		ts->usb_changed_ktime = ktime_get();
	}

	if (ts->wireless_psy != NULL && ts->wireless_psy == data) {
		ts->wlc_changed_ktime = ktime_get();
	}

	if (ts->power_status == SEC_TS_STATE_POWER_ON)
		queue_work(ts->event_wq, &ts->charger_work);

	return NOTIFY_OK;
}

static struct notifier_block sec_ts_psy_nb = {
	.notifier_call = sec_ts_psy_cb,
};
#endif

#ifdef CONFIG_OF
static const struct of_device_id sec_ts_match_table[] = {
	{ .compatible = "sec,sec_ts",},
	{ },
};
#else
#define sec_ts_match_table NULL
#endif

#ifdef I2C_INTERFACE
static struct i2c_driver sec_ts_driver = {
	.probe		= sec_ts_probe,
	.remove		= sec_ts_remove,
	.shutdown	= sec_ts_shutdown,
	.id_table	= sec_ts_id,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SEC_TS_NAME,
#ifdef CONFIG_OF
		.of_match_table = sec_ts_match_table,
#endif
#ifdef CONFIG_PM
		.pm = &sec_ts_dev_pm_ops,
#endif
	},
};
#else
static struct spi_driver sec_ts_driver = {
	.probe    = sec_ts_probe,
	.remove   = sec_ts_remove,
	.shutdown   = sec_ts_shutdown,
	.driver   = {
		.owner  = THIS_MODULE,
		.name = SEC_TS_NAME,
#ifdef CONFIG_OF
		.of_match_table = sec_ts_match_table,
#endif
#ifdef CONFIG_PM
		.pm = &sec_ts_dev_pm_ops,
#endif
	},
};
#endif


static int __init sec_ts_init(void)
{
#ifdef CONFIG_BATTERY_SAMSUNG
	if (lpcharge == 1) {
		pr_err("%s %s: Do not load driver due to lpm %d\n",
				SECLOG, __func__, lpcharge);
		return -ENODEV;
	}
#endif

#ifdef I2C_INTERFACE
	return i2c_add_driver(&sec_ts_driver);
#else
	return spi_register_driver(&sec_ts_driver);
#endif
}

static void __exit sec_ts_exit(void)
{

#ifdef I2C_INTERFACE
	i2c_del_driver(&sec_ts_driver);
#else
	spi_unregister_driver(&sec_ts_driver);
#endif
}

MODULE_AUTHOR("Hyobae, Ahn<hyobae.ahn@samsung.com>");
MODULE_DESCRIPTION("Samsung Electronics TouchScreen driver");
MODULE_LICENSE("GPL");

module_init(sec_ts_init);
module_exit(sec_ts_exit);
