/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * s2mpg1x-meter.h
 *
 * Copyright (C) 2015 Samsung Electronics
 *
 * header file including shared meter functions of s2mpg1x
 */

#ifndef __LINUX_MFD_S2MPG1X_METER_H
#define __LINUX_MFD_S2MPG1X_METER_H

#include <linux/i2c.h>
#include <linux/mfd/samsung/s2mpg1x.h>
#include <linux/mfd/samsung/s2mpg1x-register.h>
#include <linux/fs.h>

typedef enum {
	ADDRESS_CTRL1,
	ADDRESS_CTRL2,
	ADDRESS_BUCKEN1,
	ADDRESS_LPF_C0_0,
	ADDRESS_ACC_MODE,
	ADDRESS_ACC_DATA,
	ADDRESS_ACC_COUNT,
	ADDRESS_LPF_MODE,
	ADDRESS_LPF_DATA,
	ADDRESS_COUNT,
} address_t;

#if IS_ENABLED(CONFIG_SOC_GS101)
static const int ADDRESS_AT[ADDRESS_COUNT][ID_COUNT] = {
	[ADDRESS_CTRL1] = { S2MPG10_METER_CTRL1, S2MPG11_METER_CTRL1 },
	[ADDRESS_CTRL2] = { S2MPG10_METER_CTRL2, S2MPG11_METER_CTRL2 },
	[ADDRESS_BUCKEN1] = { S2MPG10_METER_BUCKEN1, S2MPG11_METER_BUCKEN1 },
	[ADDRESS_LPF_C0_0] = { S2MPG10_METER_LPF_C0_0, S2MPG11_METER_LPF_C0_0 },
	[ADDRESS_ACC_MODE] = { S2MPG10_METER_CTRL3, S2MPG11_METER_CTRL4 },
	[ADDRESS_ACC_DATA] = {
		S2MPG10_METER_ACC_DATA_CH0_1,
		S2MPG11_METER_ACC_DATA_CH0_1
	},
	[ADDRESS_ACC_COUNT] = {
		S2MPG10_METER_ACC_COUNT_1,
		S2MPG11_METER_ACC_COUNT_1
	},
	[ADDRESS_LPF_MODE] = { S2MPG10_METER_CTRL4, S2MPG11_METER_CTRL5 },
	[ADDRESS_LPF_DATA] = {
		S2MPG10_METER_LPF_DATA_CH0_1,
		S2MPG11_METER_LPF_DATA_CH0_1
	},
};
#elif IS_ENABLED(CONFIG_SOC_GS201)
static const int ADDRESS_AT[ADDRESS_COUNT][ID_COUNT] = {
	[ADDRESS_CTRL1] = { S2MPG12_METER_CTRL1, S2MPG13_METER_CTRL1 },
	[ADDRESS_CTRL2] = { S2MPG12_METER_CTRL2, S2MPG13_METER_CTRL2 },
	[ADDRESS_BUCKEN1] = { S2MPG12_METER_BUCKEN1, S2MPG13_METER_BUCKEN1 },
	[ADDRESS_LPF_C0_0] = { S2MPG12_METER_LPF_C0_0, S2MPG13_METER_LPF_C0_0 },
	[ADDRESS_ACC_MODE] = { S2MPG12_METER_CTRL4, S2MPG13_METER_CTRL4 },
	[ADDRESS_ACC_DATA] = {
		S2MPG12_METER_ACC_DATA_CH0_1,
		S2MPG13_METER_ACC_DATA_CH0_1
	},
	[ADDRESS_ACC_COUNT] = {
		S2MPG12_METER_ACC_COUNT_1,
		S2MPG13_METER_ACC_COUNT_1
	},
	[ADDRESS_LPF_MODE] = { S2MPG12_METER_CTRL6, S2MPG13_METER_CTRL6 },
	[ADDRESS_LPF_DATA] = {
		S2MPG12_METER_LPF_DATA_CH0_1,
		S2MPG13_METER_LPF_DATA_CH0_1
	},
};
#endif

static const u32 s2mpg1x_int_sample_rate_uhz[S2MPG1X_INT_FREQ_COUNT] = {
	[INT_7P_8125HZ] = 7812500, [INT_15P_625HZ] = 15625000,
	[INT_31P_25HZ] = 31250000, [INT_62P_5HZ] = 62500000,
	[INT_125HZ] = 125000000,   [INT_250HZ] = 250000000,
#if IS_ENABLED(CONFIG_SOC_GS101)
	[INT_500HZ] = 500000000,
#endif
	[INT_1000HZ] = 1000000000,
};

#if IS_ENABLED(CONFIG_SOC_GS101)
static const u32 s2mpg1x_ext_sample_rate_uhz[S2MPG1X_EXT_FREQ_COUNT] = {
	[EXT_7P_628125HZ] = 7628125, [EXT_15P_25625HZ] = 15256250,
	[EXT_30P_5125HZ] = 30512500, [EXT_61P_025HZ] = 61025000,
	[EXT_122P_05HZ] = 122050000,
};
#elif IS_ENABLED(CONFIG_SOC_GS201)
static const u32 s2mpg1x_ext_sample_rate_uhz[S2MPG1X_EXT_FREQ_COUNT] = {
	[EXT_7P_8125HZ] = 7812500, [EXT_15P_625HZ] = 15625000,
	[EXT_31P_25HZ] = 31250000, [EXT_62P_5HZ] = 62500000,
	[EXT_125HZ] = 125000000,
};
#endif

#if IS_ENABLED(CONFIG_SOC_GS201)
static const u32 s2mpg1x_int_acquisition_time_us[S2MPG1X_INT_FREQ_COUNT] = {
	[INT_7P_8125HZ] = 128000, [INT_15P_625HZ] = 64000,
	[INT_31P_25HZ] = 32000, [INT_62P_5HZ] = 16000,
	[INT_125HZ] = 8000, [INT_250HZ] = 4000,
	[INT_1000HZ] = 1000,
};
#endif

static inline int s2mpg1x_meter_get_acquisition_time_us(s2mpg1x_int_samp_rate samp_rate)
{
#if IS_ENABLED(CONFIG_SOC_GS101)
	/* Based on the s2mpg1x datasheets, (40 us * channel count) is the
	 * maximum time required for acquisition of all samples across all
	 * channels.
	 */
	return (40 * S2MPG1X_METER_CHANNEL_MAX);
#elif IS_ENABLED(CONFIG_SOC_GS201)
	/* The internal data and sample count are not updated at the same timing
	 * on Pro due to the duty cycling function, and the ASYNC_RD will depend
	 * on internal sampling rate.
	 * So at least waiting for internal rail to complete a sampling will be
	 * necessary to prevent the mismatch of irregular timing. (b/209886118)
	 */
	return s2mpg1x_int_acquisition_time_us[samp_rate];
#endif
}

#if IS_ENABLED(CONFIG_SOC_GS101)
#define ACQUISITION_TIME_DIVISOR 1
#elif IS_ENABLED(CONFIG_SOC_GS201)
#define ACQUISITION_TIME_DIVISOR 16
#endif
static inline int s2mpg1x_meter_set_async_blocking(enum s2mpg1x_id id,
						   struct i2c_client *i2c,
						   u64 *timestamp_capture,
						   s2mpg1x_int_samp_rate samp_rate)
{
	u8 val = 0xFF;
	int ret;
	u8 reg = ADDRESS_AT[ADDRESS_CTRL2][id];

	const u32 acquisition_time_us =
		s2mpg1x_meter_get_acquisition_time_us(samp_rate);
	const u32 min_acquisition_time_us = acquisition_time_us /
		ACQUISITION_TIME_DIVISOR;
	int acquisition_delay_count = 0;

	/* When 1 is written into ASYNC_RD bit, */
	/* transfer the accumulator data to readable registers->self-cleared */
	ret = s2mpg1x_update_reg(id, i2c, reg, ASYNC_RD_MASK, ASYNC_RD_MASK);
	if (ret != 0) {
		// Immediate return if failed
		return ret;
	}

	if (timestamp_capture)
		*timestamp_capture = ktime_get_boottime_ns();

	/* Verify if acquisition is already complete before a polled delay.
	 * Return immediately to reduce refresh time if so.
	 */
	ret = s2mpg1x_read_reg(id, i2c, reg, &val);

	if (ret == 0 && (val & ASYNC_RD_MASK) == 0x00)
		return ret; /* Read success */

	/* Reading has failed OR we sampled during acquisition, so wait the
	 * acquisition time and return based on the values read.
	 */
	do {
		usleep_range(min_acquisition_time_us,
			     min_acquisition_time_us + 100);
		ret = s2mpg1x_read_reg(id, i2c, reg, &val);
		if (ret != 0 || (val & ASYNC_RD_MASK) == 0x00)
			return ret;
		acquisition_delay_count++;
	} while (acquisition_delay_count < ACQUISITION_TIME_DIVISOR);

	return -1; /* ASYNC value has not changed */
}

static inline ssize_t s2mpg1x_meter_format_channel(char *buf, ssize_t count,
						   int ch, const char *name,
						   const char *units,
						   u64 acc_data, u32 resolution,
						   u32 acc_count)
{
	// Note: resolution is in iq30
	// --> Convert it back to a human readable decimal value
	const u32 one_billion = 1000000000;
	u64 resolution_max = _IQ30_to_int((u64)resolution * one_billion);

	return scnprintf(buf + count, PAGE_SIZE - count,
			 "CH%d[%s]: 0x%016llx * %lld.%09llu / 0x%08x %s\n", ch,
			 name, acc_data, resolution_max / one_billion,
			 resolution_max % one_billion, acc_count, units);
}

static inline int s2mpg1x_meter_set_buck_channel_en(enum s2mpg1x_id id,
						    struct i2c_client *i2c,
						    u8 *channels, int num_bytes)
{
	if (num_bytes > S2MPG1X_METER_BUCKEN_BUF)
		return -EINVAL;

	return s2mpg1x_bulk_write(id, i2c, ADDRESS_AT[ADDRESS_BUCKEN1][id],
				  num_bytes, channels);
}

static inline int s2mpg1x_meter_set_ext_channel_en(enum s2mpg1x_id id,
						   struct i2c_client *i2c,
						   u8 channels)
{
	return s2mpg1x_update_reg(id, i2c, ADDRESS_AT[ADDRESS_CTRL2][id],
				  channels << EXT_METER_CHANNEL_EN_OFFSET,
				  EXT_METER_CHANNEL_EN_MASK);
}

static inline int s2mpg1x_meter_set_int_samp_rate(enum s2mpg1x_id id,
						  struct i2c_client *i2c,
						  s2mpg1x_int_samp_rate hz)
{
	return s2mpg1x_update_reg(id, i2c, ADDRESS_AT[ADDRESS_CTRL1][id],
				  hz << INT_SAMP_RATE_SHIFT,
				  INT_SAMP_RATE_MASK);
}

static inline int s2mpg1x_meter_set_ext_samp_rate(enum s2mpg1x_id id,
						  struct i2c_client *i2c,
						  s2mpg1x_ext_samp_rate hz)
{
	return s2mpg1x_update_reg(id, i2c, ADDRESS_AT[ADDRESS_CTRL2][id],
				  hz, EXT_SAMP_RATE_MASK);
}

static inline int s2mpg1x_meter_set_lpf_coefficient(enum s2mpg1x_id id,
						    struct i2c_client *i2c,
						    int ch,
						    u32 val)
{
	if (ch >= S2MPG1X_METER_CHANNEL_MAX)
		return -EINVAL;

	return s2mpg1x_write_reg(id, i2c, ADDRESS_AT[ADDRESS_LPF_C0_0][id] +
				 ch, val);
}

static inline const u32 *s2mpg1x_meter_get_int_samping_rate_table(void)
{
	return s2mpg1x_int_sample_rate_uhz;
}

static inline const u32 *s2mpg1x_meter_get_ext_samping_rate_table(void)
{
	return s2mpg1x_ext_sample_rate_uhz;
}

static inline void s2mpg1x_meter_set_mode(enum s2mpg1x_id id,
					  struct i2c_client *i2c,
					  s2mpg1x_meter_mode mode,
					  bool is_acc_mode) // else lpf

{
	address_t mode_addr = ADDRESS_ACC_MODE;

	if (!is_acc_mode)
		mode_addr = ADDRESS_LPF_MODE;

	switch (mode) {
	case S2MPG1X_METER_POWER:
		s2mpg1x_write_reg(id, i2c, ADDRESS_AT[mode_addr][id], 0x00);
#if IS_ENABLED(CONFIG_SOC_GS201)
		s2mpg1x_update_reg(id, i2c, ADDRESS_AT[mode_addr][id] + 1, 0x00,
				   /* mask= */ 0x0F);
#endif
		break;

	case S2MPG1X_METER_CURRENT:
		s2mpg1x_write_reg(id, i2c, ADDRESS_AT[mode_addr][id], 0xFF);
#if IS_ENABLED(CONFIG_SOC_GS201)
		s2mpg1x_update_reg(id, i2c, ADDRESS_AT[mode_addr][id] + 1, 0x0F,
				   /* mask= */ 0x0F);
#endif
		break;
	}
}

static inline void s2mpg1x_meter_set_acc_mode(enum s2mpg1x_id id,
					      struct i2c_client *i2c,
					      s2mpg1x_meter_mode mode)
{
	s2mpg1x_meter_set_mode(id, i2c, mode, /* is_acc_mode= */ true);
}

static inline void s2mpg1x_meter_set_lpf_mode(enum s2mpg1x_id id,
					      struct i2c_client *i2c,
					      s2mpg1x_meter_mode mode)
{
	s2mpg1x_meter_set_mode(id, i2c, mode, /* is_acc_mode= */ false);
}

static inline void s2mpg1x_meter_read_acc_data_reg(enum s2mpg1x_id id,
						   struct i2c_client *i2c,
						   u64 *data)
{
	int i;
	u8 buf[S2MPG1X_METER_ACC_BUF];
	u8 reg = ADDRESS_AT[ADDRESS_ACC_DATA][id]; /* first acc data register */

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg1x_bulk_read(id, i2c, reg, S2MPG1X_METER_ACC_BUF, buf);

		/* 41 bits of data */
		data[i] = ((u64)buf[0] << 0) | ((u64)buf[1] << 8) |
			  ((u64)buf[2] << 16) | ((u64)buf[3] << 24) |
			  ((u64)buf[4] << 32) | (((u64)buf[5] & 0x1) << 40);

		reg += S2MPG1X_METER_ACC_BUF;
	}
}

static inline void s2mpg1x_meter_read_acc_count(enum s2mpg1x_id id,
						struct i2c_client *i2c,
						u32 *count)
{
	u8 data[S2MPG1X_METER_COUNT_BUF];
	u8 reg = ADDRESS_AT[ADDRESS_ACC_COUNT][id]; /* first count register */

	s2mpg1x_bulk_read(id, i2c, reg, S2MPG1X_METER_COUNT_BUF, data);

	/* ACC_COUNT is 20-bit data */
	*count = (data[0] << 0) | (data[1] << 8) | ((data[2] & 0x0F) << 16);
}

static inline void s2mpg1x_meter_read_lpf_data_reg(enum s2mpg1x_id id,
						   struct i2c_client *i2c,
						   u32 *data)
{
	int i;
	u8 buf[S2MPG1X_METER_LPF_BUF];
	u8 reg = ADDRESS_AT[ADDRESS_LPF_DATA][id]; /* first lpf data register */

	for (i = 0; i < S2MPG1X_METER_CHANNEL_MAX; i++) {
		s2mpg1x_bulk_read(id, i2c, reg, S2MPG1X_METER_LPF_BUF, buf);

		/* LPF is 21-bit data */
		data[i] = buf[0] + (buf[1] << 8) + ((buf[2] & 0x1F) << 16);

		reg += S2MPG1X_METER_LPF_BUF;
	}
}

/**
 * Load measurement into registers and read measurement from the registers
 *
 * Note: data must be an array with length S2MPG1X_METER_CHANNEL_MAX
 */
static inline int s2mpg1x_meter_measure_acc(enum s2mpg1x_id id,
					    struct i2c_client *i2c,
					    struct mutex *meter_lock,
					    s2mpg1x_meter_mode mode,
					    u64 *data,
					    u32 *count,
					    u64 *timestamp_capture,
					    s2mpg1x_int_samp_rate samp_rate)
{
	mutex_lock(meter_lock);

	s2mpg1x_meter_set_acc_mode(id, i2c, mode);

	s2mpg1x_meter_set_async_blocking(id, i2c,
					 timestamp_capture,
					 samp_rate);

	if (data)
		s2mpg1x_meter_read_acc_data_reg(id, i2c, data);

	if (count)
		s2mpg1x_meter_read_acc_count(id, i2c, count);

	mutex_unlock(meter_lock);

	return 0;
}

#if IS_ENABLED(CONFIG_SOC_GS201)
#define SW_RESET_DELAYTIME_US 2
static inline int s2mpg1x_meter_sw_reset(enum s2mpg1x_id id,
					  struct i2c_client *i2c,
					  struct i2c_client *mt_trim,
					  u8 mt_trim_reg)
{
	int ret;

	ret = s2mpg1x_update_reg(id, mt_trim, mt_trim_reg, 0x00,
				 /* mask= */ BIT(7));
	if (ret != 0) {
		pr_err("odpm: s2mpg1%d-odpm: failed to update mt_trim bit_7 to 0\n", id + 2);
		return ret;
	}
	ret = s2mpg1x_update_reg(id, mt_trim, mt_trim_reg, 0x80,
				 /* mask= */ BIT(7));
	if (ret != 0) {
		pr_err("odpm: s2mpg1%d-odpm: failed to update mt_trim bit_7 to 1\n", id + 2);
		return ret;
	}

	usleep_range(SW_RESET_DELAYTIME_US, SW_RESET_DELAYTIME_US + 100);

	ret = s2mpg1x_update_reg(id, i2c, ADDRESS_AT[ADDRESS_CTRL1][id], 0x01,
				 METER_EN_MASK);
	if (ret != 0)
		pr_err("odpm: s2mpg1%d-odpm: failed to update meter_ctrl1 bit_0 to 1\n", id + 2);

	return ret;
}
#endif

#endif /* __LINUX_MFD_S2MPG1X_METER_H */
