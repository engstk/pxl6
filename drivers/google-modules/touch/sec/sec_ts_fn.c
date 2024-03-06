/* drivers/input/touchscreen/sec_ts_fn.c
 *
 * Copyright (C) 2015 Samsung Electronics Co., Ltd.
 * http://www.samsungsemi.com/
 *
 * Core file for Samsung TSC driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "sec_ts.h"
#ifdef USE_SPEC_CHECK
#include "sec_ts_fac_spec.h"
#endif

static void fw_update(void *device_data);
static void get_fw_ver_bin(void *device_data);
static void get_fw_ver_ic(void *device_data);
static void get_config_ver(void *device_data);
#ifdef PAT_CONTROL
static void get_pat_information(void *device_data);
static void set_external_factory(void *device_data);
#endif
static void get_threshold(void *device_data);
static void module_off_master(void *device_data);
static void module_on_master(void *device_data);
static void get_chip_vendor(void *device_data);
static void get_chip_name(void *device_data);
static void set_mis_cal_spec(void *device_data);
static void get_mis_cal_info(void *device_data);
static void get_wet_mode(void *device_data);
static void get_x_num(void *device_data);
static void get_y_num(void *device_data);
static void get_x_cross_routing(void *device_data);
static void get_y_cross_routing(void *device_data);
static void get_checksum_data(void *device_data);
static void run_reference_read(void *device_data);
static void run_reference_read_all(void *device_data);
static void get_reference(void *device_data);
static void run_rawcap_read(void *device_data);
static void run_rawcap_read_all(void *device_data);
static void run_rawcap_combo_read_all(void *device_data);
static void run_rawcap_gap_combo_read_all(void *device_data);
static void run_rawcap_factory_read_all(void *device_data);
static void run_rawcap_high_freq_read_all(void *device_data);
static void get_rawcap(void *device_data);
static void run_rawcap_gap_read_all(void *device_data);
static void run_delta_read(void *device_data);
static void run_delta_read_all(void *device_data);
static void get_delta(void *device_data);
static void run_rawdata_stdev_read(void *device_data);
static void run_rawdata_p2p_read_all(void *device_data);
static void run_rawdata_read_type(void *device_data);
static void run_rawdata_read_all(void *device_data);
static void run_self_reference_read(void *device_data);
static void run_self_reference_read_all(void *device_data);
static void run_self_rawcap_read(void *device_data);
static void run_self_rawcap_read_all(void *device_data);
static void run_self_rawcap_gap_read_all(void *device_data);
static void run_self_delta_read(void *device_data);
static void run_self_delta_read_all(void *device_data);
static void run_force_calibration(void *device_data);
static void get_force_calibration(void *device_data);
#ifdef USE_PRESSURE_SENSOR
static void run_force_pressure_calibration(void *device_data);
static void set_pressure_test_mode(void *device_data);
static void run_pressure_filtered_strength_read_all(void *device_data);
static void run_pressure_strength_read_all(void *device_data);
static void run_pressure_rawdata_read_all(void *device_data);
static void run_pressure_offset_read_all(void *device_data);
static void set_pressure_strength(void *device_data);
static void set_pressure_rawdata(void *device_data);
static void set_pressure_data_index(void *device_data);
static void get_pressure_strength(void *device_data);
static void get_pressure_rawdata(void *device_data);
static void get_pressure_data_index(void *device_data);
static void set_pressure_strength_clear(void *device_data);
static void get_pressure_threshold(void *device_data);
static void set_pressure_user_level(void *device_data);
static void get_pressure_user_level(void *device_data);
#endif
#ifdef USE_STIM_PAD
static void run_fs_cal_pre_press(void *device_data);
static void run_fs_cal_get_data(void *device_data);
static void run_fs_cal_post_press(void *device_data);
static void enable_fs_cal_table(void *device_data);
static void enable_gain_limit(void *device_data);
#endif
static void enable_coordinate_report(void *device_data);
static void run_trx_short_test(void *device_data);
static void set_tsp_test_result(void *device_data);
static void get_tsp_test_result(void *device_data);
static void increase_disassemble_count(void *device_data);
static void get_disassemble_count(void *device_data);
static void glove_mode(void *device_data);
static void clear_cover_mode(void *device_data);
static void dead_zone_enable(void *device_data);
static void drawing_test_enable(void *device_data);
static void set_lowpower_mode(void *device_data);
static void set_wirelesscharger_mode(void *device_data);
static void spay_enable(void *device_data);
static void set_aod_rect(void *device_data);
static void get_aod_rect(void *device_data);
static void aod_enable(void *device_data);
static void set_grip_data(void *device_data);
static void dex_enable(void *device_data);
static void brush_enable(void *device_data);
static void force_touch_active(void *device_data);
static void set_touchable_area(void *device_data);
static void set_log_level(void *device_data);
static void debug(void *device_data);
static void set_touch_mode(void *device_data);
static void not_support_cmd(void *device_data);
static void set_palm_detection_enable(void *device_data);
static void set_grip_detection_enable(void *device_data);
static void set_wet_mode_enable(void *device_data);
static void set_noise_mode_enable(void *device_data);
static void set_continuous_report_enable(void *device_data);
static void set_charger_nb_enable(void *device_data);
static void set_print_format(void *device_data);
static void run_cal_check(void *device_data);

static struct sec_cmd sec_cmds[] = {
	{SEC_CMD("fw_update", fw_update),},
	{SEC_CMD("get_fw_ver_bin", get_fw_ver_bin),},
	{SEC_CMD("get_fw_ver_ic", get_fw_ver_ic),},
	{SEC_CMD("get_config_ver", get_config_ver),},
#ifdef PAT_CONTROL
	{SEC_CMD("get_pat_information", get_pat_information),},
	{SEC_CMD("set_external_factory", set_external_factory),},
#endif
	{SEC_CMD("get_threshold", get_threshold),},
	{SEC_CMD("module_off_master", module_off_master),},
	{SEC_CMD("module_on_master", module_on_master),},
	{SEC_CMD("get_chip_vendor", get_chip_vendor),},
	{SEC_CMD("get_chip_name", get_chip_name),},
	{SEC_CMD("set_mis_cal_spec", set_mis_cal_spec),},
	{SEC_CMD("get_mis_cal_info", get_mis_cal_info),},
	{SEC_CMD("get_wet_mode", get_wet_mode),},
	{SEC_CMD("get_x_num", get_x_num),},
	{SEC_CMD("get_y_num", get_y_num),},
	{SEC_CMD("get_x_cross_routing", get_x_cross_routing),},
	{SEC_CMD("get_y_cross_routing", get_y_cross_routing),},
	{SEC_CMD("get_checksum_data", get_checksum_data),},
	{SEC_CMD("run_reference_read", run_reference_read),},
	{SEC_CMD("run_reference_read_all", run_reference_read_all),},
	{SEC_CMD("get_reference", get_reference),},
	{SEC_CMD("run_rawcap_read", run_rawcap_read),},
	{SEC_CMD("run_rawcap_read_all", run_rawcap_read_all),},
	{SEC_CMD("run_rawcap_combo_read_all", run_rawcap_combo_read_all),},
	{SEC_CMD("run_rawcap_gap_combo_read_all", run_rawcap_gap_combo_read_all),},
	{SEC_CMD("run_rawcap_factory_read_all",
		 run_rawcap_factory_read_all),},
	{SEC_CMD("get_rawcap", get_rawcap),},
	{SEC_CMD("run_rawcap_gap_read_all", run_rawcap_gap_read_all),},
	{SEC_CMD("run_delta_read", run_delta_read),},
	{SEC_CMD("run_delta_read_all", run_delta_read_all),},
	{SEC_CMD("get_delta", get_delta),},
	{SEC_CMD("run_rawdata_stdev_read", run_rawdata_stdev_read),},
	{SEC_CMD("run_rawdata_p2p_read_all", run_rawdata_p2p_read_all),},
	{SEC_CMD("run_rawdata_read_type", run_rawdata_read_type),},
	{SEC_CMD("run_rawdata_read_all", run_rawdata_read_all),},
	{SEC_CMD("run_self_reference_read", run_self_reference_read),},
	{SEC_CMD("run_self_reference_read_all", run_self_reference_read_all),},
	{SEC_CMD("run_self_rawcap_read", run_self_rawcap_read),},
	{SEC_CMD("run_self_rawcap_read_all", run_self_rawcap_read_all),},
	{SEC_CMD("run_self_rawcap_gap_read_all",
		 run_self_rawcap_gap_read_all),},
	{SEC_CMD("run_rawcap_high_freq_read_all",
		 run_rawcap_high_freq_read_all),},
	{SEC_CMD("run_self_delta_read", run_self_delta_read),},
	{SEC_CMD("run_self_delta_read_all", run_self_delta_read_all),},
	{SEC_CMD("run_force_calibration", run_force_calibration),},
	{SEC_CMD("get_force_calibration", get_force_calibration),},
#ifdef USE_PRESSURE_SENSOR
	{SEC_CMD("run_force_pressure_calibration",
		 run_force_pressure_calibration),},
	{SEC_CMD("set_pressure_test_mode", set_pressure_test_mode),},
	{SEC_CMD("run_pressure_filtered_strength_read_all",
		 run_pressure_filtered_strength_read_all),},
	{SEC_CMD("run_pressure_strength_read_all",
		 run_pressure_strength_read_all),},
	{SEC_CMD("run_pressure_rawdata_read_all",
		 run_pressure_rawdata_read_all),},
	{SEC_CMD("run_pressure_offset_read_all",
		 run_pressure_offset_read_all),},
	{SEC_CMD("set_pressure_strength", set_pressure_strength),},
	{SEC_CMD("set_pressure_rawdata", set_pressure_rawdata),},
	{SEC_CMD("set_pressure_data_index", set_pressure_data_index),},
	{SEC_CMD("get_pressure_strength", get_pressure_strength),},
	{SEC_CMD("get_pressure_rawdata", get_pressure_rawdata),},
	{SEC_CMD("get_pressure_data_index", get_pressure_data_index),},
	{SEC_CMD("set_pressure_strength_clear", set_pressure_strength_clear),},
	{SEC_CMD("get_pressure_threshold", get_pressure_threshold),},
	{SEC_CMD("set_pressure_user_level", set_pressure_user_level),},
	{SEC_CMD("get_pressure_user_level", get_pressure_user_level),},
#endif
#ifdef USE_STIM_PAD
	{SEC_CMD("run_fs_cal_pre_press", run_fs_cal_pre_press),},
	{SEC_CMD("run_fs_cal_get_data", run_fs_cal_get_data),},
	{SEC_CMD("run_fs_cal_post_press", run_fs_cal_post_press),},
	{SEC_CMD("enable_fs_cal_table", enable_fs_cal_table),},
	{SEC_CMD("enable_gain_limit", enable_gain_limit),},
#endif
	{SEC_CMD("enable_coordinate_report", enable_coordinate_report),},
	{SEC_CMD("run_trx_short_test", run_trx_short_test),},
	{SEC_CMD("set_tsp_test_result", set_tsp_test_result),},
	{SEC_CMD("get_tsp_test_result", get_tsp_test_result),},
	{SEC_CMD("increase_disassemble_count", increase_disassemble_count),},
	{SEC_CMD("get_disassemble_count", get_disassemble_count),},
	{SEC_CMD("glove_mode", glove_mode),},
	{SEC_CMD("clear_cover_mode", clear_cover_mode),},
	{SEC_CMD("dead_zone_enable", dead_zone_enable),},
	{SEC_CMD("drawing_test_enable", drawing_test_enable),},
	{SEC_CMD("set_lowpower_mode", set_lowpower_mode),},
	{SEC_CMD("set_wirelesscharger_mode", set_wirelesscharger_mode),},
	{SEC_CMD("spay_enable", spay_enable),},
	{SEC_CMD("set_aod_rect", set_aod_rect),},
	{SEC_CMD("get_aod_rect", get_aod_rect),},
	{SEC_CMD("aod_enable", aod_enable),},
	{SEC_CMD("set_grip_data", set_grip_data),},
	{SEC_CMD("dex_enable", dex_enable),},
	{SEC_CMD("brush_enable", brush_enable),},
	{SEC_CMD("force_touch_active", force_touch_active),},
	{SEC_CMD("set_touchable_area", set_touchable_area),},
	{SEC_CMD("set_log_level", set_log_level),},
	{SEC_CMD("debug", debug),},
	{SEC_CMD("set_touch_mode", set_touch_mode),},
	{SEC_CMD("set_palm_detection_enable", set_palm_detection_enable),},
	{SEC_CMD("set_grip_detection_enable", set_grip_detection_enable),},
	{SEC_CMD("set_wet_mode_enable", set_wet_mode_enable),},
	{SEC_CMD("set_noise_mode_enable", set_noise_mode_enable),},
	{SEC_CMD("set_continuous_report_enable",
		set_continuous_report_enable),},
	{SEC_CMD("set_charger_nb_enable", set_charger_nb_enable),},
	{SEC_CMD("set_print_format", set_print_format),},
	{SEC_CMD("run_cal_check", run_cal_check),},
	{SEC_CMD("not_support_cmd", not_support_cmd),},
};

static void set_palm_detection_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[4] = { 0 };
	u8 para = 0x0;
	int ret;

	input_info(true, &ts->client->dev,
		"%s: %d\n", __func__, sec->cmd_param[0]);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);
	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] == 1)
		para = 0x1;
	else if (sec->cmd_param[0] == 0)
		para = 0x0;
	else {
		input_info(true, &ts->client->dev,
			"%s: param error! param = %d\n",
			__func__, sec->cmd_param[0]);
		goto err_out;
	}

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_PALM_DETEC, &para, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			 "%s: write reg %#x para %#x failed, returned %i\n",
			__func__, SEC_TS_CMD_SET_PALM_DETEC, para, ret);
		goto err_out;
	}

	scnprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

err_out:
	scnprintf(buff, sizeof(buff), "%s", "NG");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void set_grip_detection_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[4] = { 0 };
	u8 para = 0x0;
	int ret;

	input_info(true, &ts->client->dev,
		"%s: %d\n", __func__, sec->cmd_param[0]);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);
	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] == 1)
		para = 0x1F;
	else if (sec->cmd_param[0] == 0)
		para = 0x0;
	else {
		input_info(true, &ts->client->dev,
			"%s: param error! param = %d\n",
			__func__, sec->cmd_param[0]);
		goto err_out;
	}

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_GRIP_DETEC, &para, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			 "%s: write reg %#x para %#x failed, returned %i\n",
			__func__, SEC_TS_CMD_SET_GRIP_DETEC, para, ret);
		goto err_out;
	}

	scnprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

err_out:
	scnprintf(buff, sizeof(buff), "%s", "NG");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void set_wet_mode_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[4] = { 0 };
	u8 para = 0x0;
	int ret;

	input_info(true, &ts->client->dev,
		"%s: %d\n", __func__, sec->cmd_param[0]);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);
	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] == 1)
		para = 0x0;
	else if (sec->cmd_param[0] == 0)
		para = 0x1;
	else {
		input_info(true, &ts->client->dev,
			"%s: param error! param = %d\n",
			__func__, sec->cmd_param[0]);
		goto err_out;
	}

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_WET_MODE, &para, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			 "%s: write reg %#x para %#x failed, returned %i\n",
			__func__, SEC_TS_CMD_SET_WET_MODE, para, ret);
		goto err_out;
	}

	scnprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

err_out:
	scnprintf(buff, sizeof(buff), "%s", "NG");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void set_noise_mode_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[4] = { 0 };
	u8 para = 0x0;
	int ret;

	input_info(true, &ts->client->dev,
		"%s: %d\n", __func__, sec->cmd_param[0]);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);
	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] == 1)
		para = NOISE_MODE_DEFALUT;
	else if (sec->cmd_param[0] == 0)
		para = NOISE_MODE_OFF;
	else {
		input_info(true, &ts->client->dev,
			"%s: param error! param = %d\n",
			__func__, sec->cmd_param[0]);
		goto err_out;
	}

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_NOISE_MODE, &para, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			 "%s: write reg %#x para %#x failed, returned %i\n",
			__func__, SEC_TS_CMD_SET_NOISE_MODE, para, ret);
		goto err_out;
	}

	scnprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

err_out:
	scnprintf(buff, sizeof(buff), "%s", "NG");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void set_continuous_report_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[4] = { 0 };
	u8 para = 0x0;
	int ret;

	input_info(true, &ts->client->dev,
		"%s: %d\n", __func__, sec->cmd_param[0]);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);
	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] == 1)
		para = 0x1;
	else if (sec->cmd_param[0] == 0)
		para = 0x0;
	else {
		input_info(true, &ts->client->dev,
			"%s: param error! param = %d\n",
			__func__, sec->cmd_param[0]);
		goto err_out;
	}

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_CONT_REPORT, &para, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			 "%s: write reg %#x para %#x failed, returned %i\n",
			__func__, SEC_TS_CMD_SET_CONT_REPORT, para, ret);
		goto err_out;
	}

	ts->use_default_mf = para;
	scnprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

err_out:
	scnprintf(buff, sizeof(buff), "%s", "NG");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void set_charger_nb_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[4] = { 0 };

	input_info(true, &ts->client->dev,
		"%s: %d\n", __func__, sec->cmd_param[0]);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);
	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] == 1)
		ts->ignore_charger_nb = 0;
	else if (sec->cmd_param[0] == 0)
		ts->ignore_charger_nb = 1;
	else {
		input_info(true, &ts->client->dev,
			"%s: param error! param = %d\n",
			__func__, sec->cmd_param[0]);
		goto err_out;
	}

	scnprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

err_out:
	scnprintf(buff, sizeof(buff), "%s", "NG");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static ssize_t scrub_pos_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[256] = { 0 };

#ifdef CONFIG_SAMSUNG_PRODUCT_SHIP
	input_info(true, &ts->client->dev,
			"%s: scrub_id: %d\n", __func__, ts->scrub_id);
#else
	input_info(true, &ts->client->dev,
			"%s: scrub_id: %d, X: %d, Y: %d\n", __func__,
			ts->scrub_id, ts->scrub_x, ts->scrub_y);
#endif
	snprintf(buff, sizeof(buff), "%d %d %d",
		 ts->scrub_id, ts->scrub_x, ts->scrub_y);

	ts->scrub_x = 0;
	ts->scrub_y = 0;

	return snprintf(buf, PAGE_SIZE, "%s", buff);
}

static DEVICE_ATTR_RO(scrub_pos);

static ssize_t ito_check_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[256] = { 0 };

	input_info(true, &ts->client->dev, "%s: %02X%02X%02X%02X\n", __func__,
		ts->ito_test[0], ts->ito_test[1],
		ts->ito_test[2], ts->ito_test[3]);

	snprintf(buff, sizeof(buff), "%02X%02X%02X%02X",
		ts->ito_test[0], ts->ito_test[1],
		ts->ito_test[2], ts->ito_test[3]);

	return snprintf(buf, SEC_CMD_BUF_SIZE, "%s", buff);
}

static ssize_t raw_check_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	int ii, ret = 0;
	char *buffer = NULL;
	char temp[CMD_RESULT_WORD_LEN] = { 0 };

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	buffer = vzalloc(ts->rx_count * ts->tx_count * 6);
	if (!buffer) {
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return -ENOMEM;
	}

	memset(buffer, 0x00, ts->rx_count * ts->tx_count * 6);

	for (ii = 0; ii < (ts->rx_count * ts->tx_count - 1); ii++) {
		snprintf(temp, CMD_RESULT_WORD_LEN, "%d ", ts->pFrame[ii]);
		strncat(buffer, temp, CMD_RESULT_WORD_LEN);

		memset(temp, 0x00, CMD_RESULT_WORD_LEN);
	}

	snprintf(temp, CMD_RESULT_WORD_LEN, "%d", ts->pFrame[ii]);
	strncat(buffer, temp, CMD_RESULT_WORD_LEN);

	ret = snprintf(buf, ts->rx_count * ts->tx_count * 6, buffer);
	vfree(buffer);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return ret;
}

static ssize_t multi_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);

	input_info(true, &ts->client->dev, "%s: %d\n", __func__,
		ts->multi_count);

	return snprintf(buf, SEC_CMD_BUF_SIZE, "%d", ts->multi_count);
}

static ssize_t multi_count_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);

	ts->multi_count = 0;

	input_info(true, &ts->client->dev, "%s: clear\n", __func__);

	return count;
}

static ssize_t wet_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);

	input_info(true, &ts->client->dev, "%s: %d, %d\n", __func__,
		ts->wet_count, ts->dive_count);

	return snprintf(buf, SEC_CMD_BUF_SIZE, "%d", ts->wet_count);
}

static ssize_t wet_mode_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);

	ts->wet_count = 0;
	ts->dive_count = 0;

	input_info(true, &ts->client->dev, "%s: clear\n", __func__);

	return count;
}

static ssize_t comm_err_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);

	input_info(true, &ts->client->dev, "%s: %d\n", __func__,
		ts->multi_count);

	return snprintf(buf, SEC_CMD_BUF_SIZE, "%d", ts->comm_err_count);
}

static ssize_t comm_err_count_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);

	ts->comm_err_count = 0;

	input_info(true, &ts->client->dev, "%s: clear\n", __func__);

	return count;
}

static ssize_t module_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[256] = { 0 };

	input_info(true, &ts->client->dev, "%s: %d\n", __func__,
		ts->multi_count);

	snprintf(buff, sizeof(buff), "SE%02X%02X%02X%02X%02X%02X%02X",
		ts->plat_data->panel_revision,
		ts->plat_data->img_version_of_bin[2],
		ts->plat_data->img_version_of_bin[3], ts->nv, ts->cal_count,
		ts->pressure_cal_base, ts->pressure_cal_delta);

	return snprintf(buf, SEC_CMD_BUF_SIZE, "%s", buff);
}

static ssize_t vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	unsigned char buffer[10] = { 0 };

	snprintf(buffer, 5, ts->plat_data->firmware_name + 8);

	return snprintf(buf, SEC_CMD_BUF_SIZE, "LSI_%s", buffer);
}

static ssize_t checksum_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);

	ts->checksum_result = 0;

	input_info(true, &ts->client->dev, "%s: clear\n", __func__);

	return count;
}

static ssize_t checksum_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);

	input_info(true, &ts->client->dev, "%s: %d\n", __func__,
		ts->checksum_result);

	return snprintf(buf, SEC_CMD_BUF_SIZE, "%d", ts->checksum_result);
}

static ssize_t holding_time_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);

	ts->longest_duration = 0;

	input_info(true, &ts->client->dev, "%s: clear\n", __func__);

	return count;
}

static ssize_t holding_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);

	input_info(true, &ts->client->dev, "%s: %lld ms\n",
		__func__, ts->longest_duration);

	return snprintf(buf, SEC_CMD_BUF_SIZE, "%lld ms", ts->longest_duration);
}

static ssize_t all_touch_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);

	input_info(true, &ts->client->dev,
		"%s: touch: %d, force: %d, aod: %d, spay: %d\n", __func__,
		ts->all_finger_count, ts->all_force_count,
		ts->all_aod_tap_count, ts->all_spay_count);

	return snprintf(buf, SEC_CMD_BUF_SIZE,
			"\"TTCN\":\"%d\",\"TFCN\":\"%d\",\"TACN\":\"%d\",\"TSCN\":\"%d\"",
			ts->all_finger_count, ts->all_force_count,
			ts->all_aod_tap_count, ts->all_spay_count);
}

static ssize_t all_touch_count_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);

	ts->all_force_count = 0;
	ts->all_aod_tap_count = 0;
	ts->all_spay_count = 0;

	input_info(true, &ts->client->dev, "%s: clear\n", __func__);

	return count;
}

static ssize_t z_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);

	input_info(true, &ts->client->dev, "%s: max: %d, min: %d, avg: %d\n",
		   __func__, ts->max_z_value, ts->min_z_value, ts->sum_z_value);

	if (ts->all_finger_count)
		return snprintf(buf, SEC_CMD_BUF_SIZE,
				"\"TMXZ\":\"%d\",\"TMNZ\":\"%d\",\"TAVZ\":\"%d\"",
				ts->max_z_value, ts->min_z_value,
				ts->sum_z_value / ts->all_finger_count);
	else
		return snprintf(buf, SEC_CMD_BUF_SIZE,
				"\"TMXZ\":\"%d\",\"TMNZ\":\"%d\"",
				ts->max_z_value, ts->min_z_value);

}

static ssize_t z_value_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);

	ts->max_z_value = 0;
	ts->min_z_value = 0xFFFFFFFF;
	ts->sum_z_value = 0;
	ts->all_finger_count = 0;

	input_info(true, &ts->client->dev, "%s: clear\n", __func__);

	return count;
}

static ssize_t pressure_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[256] = { 0 };

	if (ts->lowpower_mode & SEC_TS_MODE_CUSTOMLIB_FORCE_KEY)
		snprintf(buff, sizeof(buff), "1");
	else
		snprintf(buff, sizeof(buff), "0");

	return snprintf(buf, SEC_CMD_BUF_SIZE, "%s\n", buff);
}

static ssize_t pressure_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	int ret;
	unsigned long value = 0;

	if (count > 2)
		return -EINVAL;

	ret = kstrtoul(buf, 10, &value);
	if (ret != 0)
		return ret;

	if (!ts->use_customlib)
		return -EINVAL;

	if (value == 1)
		ts->lowpower_mode |= SEC_TS_MODE_CUSTOMLIB_FORCE_KEY;
	else
		ts->lowpower_mode &= ~SEC_TS_MODE_CUSTOMLIB_FORCE_KEY;

	#ifdef SEC_TS_SUPPORT_CUSTOMLIB
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);
	sec_ts_set_custom_library(ts);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	#endif

	return count;
}

static ssize_t get_lp_dump_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	u8 string_data[8] = {0, };
	u16 current_index;
	int i, ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: Touch is stopped!\n",
			  __func__);
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return snprintf(buf, SEC_CMD_BUF_SIZE, "TSP turned off");
	}

	string_data[0] = SEC_TS_CMD_CUSTOMLIB_LP_DUMP & 0xFF;
	string_data[1] = (SEC_TS_CMD_CUSTOMLIB_LP_DUMP & 0xFF00) >> 8;

	disable_irq(ts->client->irq);

	ret = ts->sec_ts_read_customlib(ts, string_data, 2);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: Failed to read rect\n",
			  __func__);
		snprintf(buf, SEC_CMD_BUF_SIZE, "NG, Failed to read rect");
		goto out;
	}

	current_index = (string_data[1] & 0xFF) << 8 | (string_data[0] & 0xFF);
	if (current_index > 1000 || current_index < 500) {
		input_err(true, &ts->client->dev,
			"Failed to Custom Library LP log %d\n", current_index);
		snprintf(buf, SEC_CMD_BUF_SIZE,
			"NG, Failed to Custom Library LP log, current_index=%d",
			current_index);
		goto out;
	}

	input_info(true, &ts->client->dev,
		"%s: DEBUG current_index = %d\n", __func__, current_index);

	/* Custom Library has 62 stacks for LP dump */
	for (i = 61; i >= 0; i--) {
		u16 data0, data1, data2, data3;
		char buff[30] = {0, };
		u16 string_addr;

		string_addr = current_index - (8 * i);
		if (string_addr < 500)
			string_addr += SEC_TS_CMD_CUSTOMLIB_LP_DUMP;
		string_data[0] = string_addr & 0xFF;
		string_data[1] = (string_addr & 0xFF00) >> 8;

		ret = ts->sec_ts_read_customlib(ts, string_data, 8);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
					"%s: Failed to read rect\n", __func__);
			snprintf(buf, SEC_CMD_BUF_SIZE,
					"NG, Failed to read rect, addr=%d",
					string_addr);
			goto out;
		}

		data0 = (string_data[1] & 0xFF) << 8 | (string_data[0] & 0xFF);
		data1 = (string_data[3] & 0xFF) << 8 | (string_data[2] & 0xFF);
		data2 = (string_data[5] & 0xFF) << 8 | (string_data[4] & 0xFF);
		data3 = (string_data[7] & 0xFF) << 8 | (string_data[6] & 0xFF);
		if (data0 || data1 || data2 || data3) {
			snprintf(buff, sizeof(buff),
				"%d: %04x%04x%04x%04x\n",
				string_addr, data0, data1, data2, data3);
			strncat(buf, buff, SEC_CMD_BUF_SIZE);
		}
	}

out:
	enable_irq(ts->client->irq);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return strlen(buf);
}

static ssize_t force_recal_count_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	u8 rbuf[4] = {0, };
	u32 recal_count;
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: Touch is stopped!\n", __func__);
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return snprintf(buf, SEC_CMD_BUF_SIZE, "%d", -ENODEV);
	}

	ret = ts->sec_ts_read(ts, SEC_TS_READ_FORCE_RECAL_COUNT, rbuf, 4);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
				"%s: Failed to read\n", __func__);
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return snprintf(buf, SEC_CMD_BUF_SIZE, "%d", -EIO);
	}

	recal_count = (rbuf[0] & 0xFF) << 24 | (rbuf[1] & 0xFF) << 16 |
			(rbuf[2] & 0xFF) << 8 | (rbuf[3] & 0xFF);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return snprintf(buf, SEC_CMD_BUF_SIZE, "%d", recal_count);
}


/* sysfs file node to store heatmap mode
 * "echo cmd > heatmap_mode" to change
 * Possible commands:
 * 0 = HEATMAP_OFF
 * 1 = HEATMAP_PARTIAL
 * 2 = HEATMAP_FULL
 */
static ssize_t heatmap_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_plat_data *pdata = ts->plat_data;
	int result;
	int val;
	u8 config;

	result = kstrtoint(buf, 10, &val);
	if (result < 0 || val < HEATMAP_OFF || val > HEATMAP_FULL) {
		input_err(true, &ts->client->dev,
			"%s: Invalid input.\n", __func__);
		return -EINVAL;
	}

	pdata->heatmap_mode = val;

	/* reset all heatmap settings when any change */
	config = 0;
	result = ts->sec_ts_write(ts,
			SEC_TS_CMD_HEATMAP_ENABLE, &config, 1);
	if (result < 0)
		input_err(true, &ts->client->dev,
			 "%s: write reg %#x failed, returned %i\n",
			__func__, SEC_TS_CMD_HEATMAP_ENABLE, result);
	config = TYPE_INVALID_DATA;
	result = ts->sec_ts_write(ts,
			SEC_TS_CMD_MUTU_RAW_TYPE, &config, 1);
	if (result < 0)
		input_err(true, &ts->client->dev,
			 "%s: write reg %#x failed, returned %i\n",
			__func__, SEC_TS_CMD_MUTU_RAW_TYPE, result);

	return count;
#else
	return 0;
#endif
}

static ssize_t heatmap_mode_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	const struct sec_ts_plat_data *pdata = ts->plat_data;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pdata->heatmap_mode);
#else
	return scnprintf(buf, PAGE_SIZE, "N/A\n");
#endif
}

/* sysfs file node to store grip prescreen mode
 * "echo cmd > grip_prescreen_mode" to change
 * Possible commands:
 * 0 = GRIP_PRESCREEN_OFF
 * 1 = GRIP_PRESCREEN_MODE_1
 * 2 = GRIP_PRESCREEN_MODE_2
 * 3 = GRIP_PRESCREEN_MODE_3
 */
static ssize_t grip_prescreen_mode_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_plat_data *pdata = ts->plat_data;
	int result;
	int val;

	result = kstrtoint(buf, 10, &val);
	if (result < 0 || val < GRIP_PRESCREEN_OFF ||
	    val > GRIP_PRESCREEN_MODE_3) {
		input_err(true, &ts->client->dev,
			"%s: Invalid input.\n", __func__);
		return -EINVAL;
	}

	pdata->grip_prescreen_mode = val;

	return count;
}

static ssize_t grip_prescreen_mode_show(struct device *dev,
				        struct device_attribute *attr,
				        char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	const struct sec_ts_plat_data *pdata = ts->plat_data;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pdata->grip_prescreen_mode);
}

/* sysfs file node to store grip prescreen timeout
 * "echo timeout > grip_prescreen_timeout" to change
 * Possible timeout range:
 *   GRIP_PRESCREEN_TIMEOUT_MIN ~ GRIP_PRESCREEN_TIMEOUT_MAX
 */
static ssize_t grip_prescreen_timeout_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_plat_data *pdata = ts->plat_data;
	int result;
	int val;

	result = kstrtoint(buf, 10, &val);
	if (result < 0 || val < GRIP_PRESCREEN_TIMEOUT_MIN ||
	    val > GRIP_PRESCREEN_TIMEOUT_MAX) {
		input_err(true, &ts->client->dev,
			"%s: Invalid input.\n", __func__);
		return -EINVAL;
	}

	pdata->grip_prescreen_timeout = val;

	return count;
}

static ssize_t grip_prescreen_timeout_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	const struct sec_ts_plat_data *pdata = ts->plat_data;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 pdata->grip_prescreen_timeout);
}

/* sysfs file node to store encoded_enable control
 * "echo cmd > encoded_enable" to change
 * Possible commands:
 * 0 = ENCODED_ENABLE_OFF
 * 1 = ENCODED_ENABLE_ON
 */
static ssize_t encoded_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_plat_data *pdata = ts->plat_data;
	int result;
	int val;

	result = kstrtoint(buf, 10, &val);
	if (result < 0 || val < ENCODED_ENABLE_OFF ||
	    val > ENCODED_ENABLE_ON) {
		input_err(true, &ts->client->dev,
			"%s: Invalid input.\n", __func__);
		return -EINVAL;
	}
	pdata->encoded_enable = val;

	return count;
#else
	return 0;
#endif
}

static ssize_t encoded_enable_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	const struct sec_ts_plat_data *pdata = ts->plat_data;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pdata->encoded_enable);
#else
	return scnprintf(buf, PAGE_SIZE, "N/A\n");
#endif
}


/* sysfs file node to dump heatmap
 * "echo cmd > heatmap_dump" to change
 * Possible commands:
 * 0 = disable
 * 1 = enable
 */
static ssize_t heatmap_dump_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	int result;
	int val;

	result = kstrtoint(buf, 10, &val);
	if (result < 0 || val < 0 || val > 1) {
		input_err(true, &ts->client->dev,
			"%s: Invalid input.\n", __func__);
		return -EINVAL;
	}
	ts->heatmap_dump = val;
	return count;
#else
	return 0;
#endif
}

static ssize_t heatmap_dump_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_plat_data *pdata = ts->plat_data;
	int x, y, max_x, max_y, index = 0;

	index += scnprintf(buf + index, PAGE_SIZE - index,
			"heatmap dump(mode %d) %s\n",
			pdata->heatmap_mode,
			(ts->heatmap_dump) ? "ENABLE" : "DISABLE");

	if (!ts->heatmap_dump)
		return index;

	max_x = ts->tx_count;
	max_y = ts->rx_count;

	for (y = 0 ; y < max_y ; y++) {
		for (x = 0 ; x < max_x ; x++) {
			index += scnprintf(buf + index,
				PAGE_SIZE - index,
				" %3d,",
				ts->v4l2.frame[y * max_x  + x]);
			if (x == max_x - 1)
				index += scnprintf(buf + index,
					PAGE_SIZE - index, "\n");
		}
	}

	return index;
#else
	return scnprintf(buf, PAGE_SIZE, "N/A\n");
#endif
}

static ssize_t fw_version_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	int ret, written = 0;
	u8 data[3];

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);
	/* If there is no FW file avaiable,
	 * sec_ts_save_version_of_ic() and sec_ts_save_version_of_bin() will
	 * no be called. Need to get through SEC_TS_READ_IMG_VERSION cmd.
	 */
	if (ts->plat_data->panel_revision == 0 &&
		ts->plat_data->img_version_of_bin[2] == 0 &&
		ts->plat_data->img_version_of_bin[3] == 0) {
		u8 fw_ver[4];

		ret = ts->sec_ts_read(ts, SEC_TS_READ_IMG_VERSION, fw_ver, 4);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				"%s: firmware version read error\n", __func__);
			goto out;
		}
		written += scnprintf(buf + written, PAGE_SIZE - written,
			"SE-V%02X.%02X.%02X\n",
			ts->plat_data->panel_revision,
			fw_ver[2],
			fw_ver[3]);
		written += scnprintf(buf + written, PAGE_SIZE - written,
			"FW file: N/A\n");
	} else {
		written += scnprintf(buf + written, PAGE_SIZE - written,
			"SE-V%02X.%02X.%02X\n",
			ts->plat_data->panel_revision,
			ts->plat_data->img_version_of_ic[2],
			ts->plat_data->img_version_of_ic[3]);
		written += scnprintf(buf + written, PAGE_SIZE - written,
			"FW file: %s\n",
			ts->plat_data->firmware_name);
	}

	written += scnprintf(buf + written, PAGE_SIZE - written,
		"Cal: %02X %02X %02X %02X %02X %02X %02X %02X\n",
		ts->cali_report[0], ts->cali_report[1], ts->cali_report[2],
		ts->cali_report[3], ts->cali_report[4], ts->cali_report[5],
		ts->cali_report[6], ts->cali_report[7]);

	ret = ts->sec_ts_read(ts, SEC_TS_READ_ID, data, sizeof(data));
	if (ret < 0) {
		input_err(true, &ts->client->dev,
					"%s: failed to read device id(%d)\n",
					__func__, ret);
		goto out;
	}
	written += scnprintf(buf + written, PAGE_SIZE - written,
		"ID: %02X %02X %02X\n",
		data[0], data[1], data[2]);
out:
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);

	return written;
}

static ssize_t status_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	int written = 0;
	unsigned char data[4] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	data[0] = 0;
	ret = ts->sec_ts_read(ts, SEC_TS_READ_BOOT_STATUS, data, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
					"%s: failed to read boot status(%d)\n",
					__func__, ret);
		goto out;
	}
	written += scnprintf(buf + written, PAGE_SIZE - written,
			     "Boot status: %#x\n", data[0]);

	memset(data, 0x0, 4);
	ret = ts->sec_ts_read(ts, SEC_TS_READ_TS_STATUS, data, 4);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
					"%s: failed to touch status(%d)\n",
					__func__, ret);
		goto out;
	}
	written += scnprintf(buf + written, PAGE_SIZE - written,
			     "Touch status: %#x, %#x, %#x, %#x\n",
			     data[0], data[1], data[2], data[3]);

	memset(data, 0x0, 2);
	ret = ts->sec_ts_read(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, data, 2);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
				"%s: failed to read touch functions(%d)\n",
				__func__, ret);
		goto out;
	}
	written += scnprintf(buf + written, PAGE_SIZE - written,
			     "Functions: %#x, %#x\n", data[0], data[1]);
	written += scnprintf(buf + written, PAGE_SIZE - written,
			     "Charger mode: %#x\n", ts->charger_mode);
	written += scnprintf(buf + written, PAGE_SIZE - written,
			     "Wet mode: %d\n", ts->wet_mode);
	written += scnprintf(buf + written, PAGE_SIZE - written,
			     "Fingers#: %d\n", ts->touch_count);
	written += scnprintf(buf + written, PAGE_SIZE - written,
			     "Report rate: %d\n", ts->report_rate);
	written += scnprintf(buf + written, PAGE_SIZE - written,
			     "Vsync: %d\n", ts->vsync);
out:

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return written;
}

static DEVICE_ATTR_RO(ito_check);
static DEVICE_ATTR_RO(raw_check);
static DEVICE_ATTR_RW(multi_count);
static DEVICE_ATTR_RW(wet_mode);
static DEVICE_ATTR_RW(comm_err_count);
static DEVICE_ATTR_RW(checksum);
static DEVICE_ATTR_RW(holding_time);
static DEVICE_ATTR_RW(all_touch_count);
static DEVICE_ATTR_RW(z_value);
static DEVICE_ATTR_RO(module_id);
static DEVICE_ATTR_RO(vendor);
static DEVICE_ATTR_RW(pressure_enable);
static DEVICE_ATTR_RO(get_lp_dump);
static DEVICE_ATTR_RO(force_recal_count);
static DEVICE_ATTR_RW(heatmap_mode);
static DEVICE_ATTR_RW(grip_prescreen_mode);
static DEVICE_ATTR_RW(grip_prescreen_timeout);
static DEVICE_ATTR_RW(encoded_enable);
static DEVICE_ATTR_RW(heatmap_dump);
static DEVICE_ATTR_RO(fw_version);
static DEVICE_ATTR_RO(status);


static struct attribute *cmd_attributes[] = {
	&dev_attr_scrub_pos.attr,
	&dev_attr_ito_check.attr,
	&dev_attr_raw_check.attr,
	&dev_attr_multi_count.attr,
	&dev_attr_wet_mode.attr,
	&dev_attr_comm_err_count.attr,
	&dev_attr_checksum.attr,
	&dev_attr_holding_time.attr,
	&dev_attr_all_touch_count.attr,
	&dev_attr_z_value.attr,
	&dev_attr_module_id.attr,
	&dev_attr_vendor.attr,
	&dev_attr_pressure_enable.attr,
	&dev_attr_get_lp_dump.attr,
	&dev_attr_force_recal_count.attr,
	&dev_attr_heatmap_mode.attr,
	&dev_attr_grip_prescreen_mode.attr,
	&dev_attr_grip_prescreen_timeout.attr,
	&dev_attr_encoded_enable.attr,
	&dev_attr_heatmap_dump.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_status.attr,
	NULL,
};

static struct attribute_group cmd_attr_group = {
	.attrs = cmd_attributes,
};

static int sec_ts_check_index(struct sec_ts_data *ts)
{
	struct sec_cmd_data *sec = &ts->sec;
	char buff[SEC_CMD_STR_LEN] = { 0 };
	int node;

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > ts->tx_count
		|| sec->cmd_param[1] < 0 || sec->cmd_param[1] > ts->rx_count) {

		snprintf(buff, sizeof(buff), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		input_info(true, &ts->client->dev,
			   "%s: parameter error: %u, %u\n",
			   __func__, sec->cmd_param[0], sec->cmd_param[0]);
		node = -1;
		return node;
	}
	node = sec->cmd_param[1] * ts->tx_count + sec->cmd_param[0];
	input_info(true, &ts->client->dev, "%s: node = %d\n", __func__, node);

	return node;
}
static void fw_update(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[64] = { 0 };
	int retval = 0;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: [ERROR] Touch is stopped\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;

		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	retval = sec_ts_firmware_update_on_hidden_menu(ts, sec->cmd_param[0]);
	if (retval < 0) {
		snprintf(buff, sizeof(buff), "%s", "NA");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		input_err(true, &ts->client->dev, "%s: failed [%d]\n",
			  __func__, retval);
	} else {
		snprintf(buff, sizeof(buff), "%s", "OK");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_OK;
		input_info(true, &ts->client->dev, "%s: success [%d]\n",
			   __func__, retval);
	}

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

int sec_ts_fix_tmode(struct sec_ts_data *ts, u8 mode, u8 state)
{
	int ret;
	u8 onoff[1] = {STATE_MANAGE_OFF};
	u8 tBuff[2] = { mode, state };

	input_info(true, &ts->client->dev, "%s: mode %d state %d\n",
		__func__, mode, state);

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_STATEMANAGE_ON, onoff, 1);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: write reg %#x failed, return %i\n",
			  __func__, SEC_TS_CMD_STATEMANAGE_ON, ret);
	sec_ts_delay(20);

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_CHG_SYSMODE, tBuff,
			       sizeof(tBuff));
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: write reg %#x failed, return %i\n",
			  __func__, SEC_TS_CMD_CHG_SYSMODE, ret);
	sec_ts_delay(20);

	return ret;
}

int sec_ts_release_tmode(struct sec_ts_data *ts)
{
	int ret;
	u8 onoff[1] = {STATE_MANAGE_ON};

	input_info(true, &ts->client->dev, "%s\n", __func__);

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_STATEMANAGE_ON, onoff, 1);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: write reg %#x failed, return %i\n",
			  __func__, SEC_TS_CMD_STATEMANAGE_ON, ret);
	sec_ts_delay(20);

	return ret;
}

/* sec_ts_cm_spec_over_check :
 * apply gap calculation with ts->pFrameMS data
 * gap = abs(N1 - N2) / MAX(N1, N2) * 100 (%)
 */
static int sec_ts_cm_spec_over_check(struct sec_ts_data *ts, short *gap,
				     bool gap_dir)
{
	int i = 0;
	int j = 0;
	int gapx, gapy, pos1, pos2;
	short dpos1, dpos2;
	int specover_count = 0;

	/* Get x-direction cm gap */
	if (!gap_dir) {
		input_dbg(true, &ts->client->dev, "gapX TX\n");

		for (i = 0; i < ts->rx_count; i++) {
			for (j = 0; j < ts->tx_count - 1; j++) {
				/* Exclude last line to get gap between two
				 * lines.
				 */
				pos1 = (i * ts->tx_count) + j;
				pos2 = (i * ts->tx_count) + (j + 1);

				dpos1 = ts->pFrameMS[pos1];
				dpos2 = ts->pFrameMS[pos2];

				if (dpos1 > dpos2)
					gapx = 100 - (dpos2 * 100 / dpos1);
				else
					gapx = 100 - (dpos1 * 100 / dpos2);

				gap[pos1] = gapx;
#ifdef USE_SPEC_CHECK
				if (gapx > cm_gap[i][j])
					specover_count++;
#endif
			}
		}
	}

	/* get y-direction cm gap */
	else {
		input_dbg(true, &ts->client->dev, "gapY RX\n");

		for (i = 0; i < ts->rx_count - 1; i++) {
			for (j = 0; j < ts->tx_count; j++) {
				pos1 = (i * ts->tx_count) + j;
				pos2 = ((i + 1) * ts->tx_count) + j;

				dpos1 = ts->pFrameMS[pos1];
				dpos2 = ts->pFrameMS[pos2];

				if (dpos1 > dpos2)
					gapy = 100 - (dpos2 * 100 / dpos1);
				else
					gapy = 100 - (dpos1 * 100 / dpos2);

				gap[pos1] = gapy;
#ifdef USE_SPEC_CHECK
				if (gapy > cm_gap[i][j])
					specover_count++;
#endif
			}
		}
	}

#ifdef USE_SPEC_CHECK
	input_info(true, &ts->client->dev, "%s: Gap NG for %d node(s)\n",
			gap_dir == 0 ? "gapX" : "gapY", specover_count);
#else
	input_info(true, &ts->client->dev, "%s\n", __func__);
#endif

	return specover_count;
}

/* sec_ts_cs_spec_over_check :
 * apply gap calculation with `pFrame` data.
 * Please notice that `pFrame` will be changed dynamically by request.
 * It could be `ts->pFrameSS` or `ts->pFrameMS` for corresponding purpose.
 */
static int sec_ts_cs_spec_over_check(struct sec_ts_data *ts, short *gap,
				     short *pFrame)
{
	int i;
	int specover_count = 0;
	short dTmp;

	for (i = 0; i < ts->tx_count - 1; i++) {
		dTmp = pFrame[i] - pFrame[i + 1];
		if (dTmp < 0)
			dTmp *= -1;

		gap[i] = dTmp;
#ifdef USE_SPEC_CHECK
		if (dTmp > cs_tx_gap)
			specover_count++;
#endif
	}

	for (i = ts->tx_count; i < ts->tx_count + ts->rx_count - 1; i++) {
		dTmp = pFrame[i] - pFrame[i + 1];
		if (dTmp < 0)
			dTmp *= -1;

		gap[i] = dTmp;
#ifdef USE_SPEC_CHECK
		if (dTmp > cs_rx_gap)
			specover_count++;
#endif
	}

#ifdef USE_SPEC_CHECK
	input_info(true, &ts->client->dev, "%s: Gap NG for %d node(s)\n",
			__func__, specover_count);
#else
	input_info(true, &ts->client->dev, "%s\n", __func__);
#endif

	return specover_count;
}

#ifdef USE_STIM_PAD
static int sec_ts_get_gain_table(struct sec_ts_data *ts)
{
	int i, j;
	int temp;
	int tmp_dv;
	unsigned int str_size, str_len = 0;
	unsigned char *pStr = NULL;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	for (i = 0; i < ts->rx_count; i++) {
		for (j = 0; j < ts->tx_count; j++) {
			tmp_dv = ts->pFrame[i * ts->tx_count + j];

			/* skip notch area */
			if (cm_region[i][j] == REGION_NOTCH) {
				ts->gainTable[j * ts->rx_count + i] = 0;
				continue;
			}

			if (tmp_dv <= 0) {
				input_info(true, &ts->client->dev,
					"%s: node[%d,%d] == 0\n", __func__, i,
					j);
				tmp_dv = 1;
			}

			temp = (fs_target[i][j] * 1000) / (tmp_dv) * 64;
			/* Add 500 to round the result */
			temp = (temp + 500) / 1000;
			if (temp > 255)
				temp = 255;
			ts->gainTable[j * ts->rx_count + i] = (temp & 0xFF);
		}
	}

	str_size = 6 * (ts->tx_count + 1);
	pStr = kzalloc(str_size, GFP_KERNEL);
	if (pStr == NULL)
		return -ENOMEM;

	input_info(true, &ts->client->dev, "%s: Gain Table\n", __func__);

	for (i = 0; i < ts->rx_count; i++) {
		pStr[0] = 0;
		str_len = 0;
		for (j = 0; j < ts->tx_count; j++) {
			str_len += scnprintf(pStr + str_len, str_size - str_len,
					" %3d",
					ts->gainTable[(j * ts->rx_count) + i]);
		}
		input_info(true, &ts->client->dev, "%s\n", pStr);
	}

	kfree(pStr);

	return 0;
}

static int sec_ts_write_gain_table(struct sec_ts_data *ts)
{
	int node_cnt = ts->tx_count * ts->rx_count;
	u8 *gainTable = NULL;
	u8 *tCmd = NULL;
	int copy_max, copy_left, copy_size, copy_cur;
	int ret = -1;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	/* Write norm table to ic
	 * divide data into 256 bytes:
	 * buffer size limit 256 bytes
	 */
	gainTable = ts->gainTable;

	copy_max = ts->io_burstmax - 3;
	copy_left = node_cnt;
	copy_size = 0;
	copy_cur = (copy_left > copy_max) ? copy_max : copy_left;

	tCmd = kzalloc(copy_cur + 3, GFP_KERNEL);
	if (!tCmd)
		goto ErrorAlloc;

	while (copy_left > 0) {
		tCmd[0] = SEC_TS_CMD_WRITE_NORM_TABLE;
		tCmd[1] = (copy_size >> 8) & 0xFF;
		tCmd[2] = (copy_size >> 0) & 0xFF;

		memcpy(&tCmd[3], &gainTable[copy_size], copy_cur);

		input_info(true, &ts->client->dev,
			   "%s: left = %d, cur = %d, size = %d\n",
			   __func__, copy_left, copy_cur, copy_size);

		ret = ts->sec_ts_write_burst_heap(ts, tCmd, 3 + copy_cur);
		if (ret < 0)
			input_err(true, &ts->client->dev,
					"%s: table write failed\n", __func__);

		copy_size += copy_cur;
		copy_left -= copy_cur;
		copy_cur = (copy_left > copy_max) ? copy_max : copy_left;
	}

ErrorAlloc:
	kfree(tCmd);

	return ret;
}

/* sec_ts_get_postcal_mean : get mean value for all nodes */
static int sec_ts_get_postcal_mean(struct sec_ts_data *ts)
{
	int i, j;
	int sum = 0;
	int nCnt = 0;

	for (i = 0; i < ts->rx_count; i++) {
		for (j = 0; j < ts->tx_count; j++) {
			if (cm_region[i][j] == REGION_NOTCH) {
				/* Count notch nodes, where fs target is 0 */
				nCnt++;
				continue;
			}
			sum += ts->pFrame[(i * ts->tx_count) + j];
		}
	}

	/* exclude notch area from average */
	sum = sum / (ts->tx_count * ts->rx_count - nCnt);

	return sum;
}

static int sec_ts_get_postcal_uniformity(struct sec_ts_data *ts, short *diff)
{
	int pos1, pos2;
	short dpos1, dpos2, gap;
	int i = 0;
	int j = 0;
	int specover_cnt = 0;

	for (i = 0; i < ts->rx_count; i++) {
		for (j = 0; j < ts->tx_count - 1; j++) {
			/* At the notch boundary, skip (leave gap as 0)
			 * if node[row][col] or node[row][col+1] is 0,
			 * it is notch boundary for column direction
			 */
			if ((cm_region[i][j] == REGION_NOTCH) ||
			    (cm_region[i][j + 1] == REGION_NOTCH))
				continue;

			pos1 = (i * ts->tx_count) + j;
			pos2 = (i * ts->tx_count) + (j + 1);

			dpos1 = ts->pFrame[pos1];
			dpos2 = ts->pFrame[pos2];

			gap = (dpos1 > dpos2) ? (dpos1 - dpos2) :
						(dpos2 - dpos1);

			diff[pos1] = gap;
		}
	}

	for (i = 0; i < ts->rx_count - 1; i++) {
		for (j = 0; j < ts->tx_count; j++) {
			/* At the notch boundary, skip (leave gap as 0)
			 * if node[row][col] or node[row+1][col] is 0,
			 * it is notch boundary for row direction
			 */
			if ((cm_region[i][j] == REGION_NOTCH) ||
			    (cm_region[i + 1][j] == REGION_NOTCH))
				continue;

			pos1 = (i * ts->tx_count) + j;
			pos2 = ((i + 1) * ts->tx_count) + j;

			dpos1 = ts->pFrame[pos1];
			dpos2 = ts->pFrame[pos2];

			gap = (dpos1 > dpos2) ? (dpos1 - dpos2) :
						(dpos2 - dpos1);

			/* find max gap between x and y direction */
			if (diff[pos1] < gap)
				diff[pos1] = gap;
		}
	}

	for (i = 0; i < ts->rx_count * ts->tx_count; i++) {
		/* since spec is in % unit, multiply 100 */
		diff[i] *= 100;
		diff[i] /= (ts->fs_postcal_mean);
		if (diff[i] > fs_postcal_uniform_spec)
			specover_cnt++;
	}

	return specover_cnt;
}
#endif

static void sec_ts_print_frame(struct sec_ts_data *ts, short *min, short *max)
{
	int i = 0;
	int j = 0;
	const unsigned int buff_size = 6 * (ts->tx_count + 1);
	unsigned int buff_len = 0;
	unsigned char *pStr = NULL;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	pStr = kzalloc(buff_size, GFP_KERNEL);
	if (pStr == NULL)
		return;

	buff_len += scnprintf(pStr + buff_len, buff_size - buff_len,
				"      TX");

	for (i = 0; i < ts->tx_count; i++)
		buff_len += scnprintf(pStr + buff_len, buff_size - buff_len,
				" %02d ", i);

	input_info(true, &ts->client->dev, "%s\n", pStr);
	buff_len = 0;
	memset(pStr, 0x0, buff_size);
	buff_len += scnprintf(pStr + buff_len, buff_size - buff_len, " +");

	for (i = 0; i < ts->tx_count; i++)
		buff_len += scnprintf(pStr + buff_len, buff_size - buff_len,
				"----");

	input_info(true, &ts->client->dev, "%s\n", pStr);

	for (i = 0; i < ts->rx_count; i++) {
		buff_len = 0;
		memset(pStr, 0x0, buff_size);
		buff_len += scnprintf(pStr + buff_len, buff_size - buff_len,
				"Rx%02d | ", i);

		for (j = 0; j < ts->tx_count; j++) {
			buff_len += scnprintf(pStr + buff_len,
				buff_size - buff_len,
				" %3d", ts->pFrame[(j * ts->rx_count) + i]);

			if (i > 0) {
				if (ts->pFrame[(j * ts->rx_count) + i] < *min)
					*min = ts->pFrame[(j * ts->rx_count) +
								i];

				if (ts->pFrame[(j * ts->rx_count) + i] > *max)
					*max = ts->pFrame[(j * ts->rx_count) +
								i];
			}
		}
		input_info(true, &ts->client->dev, "%s\n", pStr);
	}
	kfree(pStr);
}

static int sec_ts_read_frame(struct sec_ts_data *ts, u8 type, short *min,
		short *max, enum spec_check_type *spec_check)
{
	unsigned int readbytes = 0xFF;
	unsigned char *pRead = NULL;
	u8 mode = TYPE_INVALID_DATA;
	int ret = 0;
	int i = 0;
	int j = 0;
	short *temp = NULL;
	u8 w_type;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	/* set data length, allocation buffer memory */
	readbytes = ts->rx_count * ts->tx_count * 2;

	pRead = kzalloc(readbytes, GFP_KERNEL);
	if (!pRead)
		return -ENOMEM;

	/* set OPCODE and data type */
	if (type == TYPE_OFFSET_DATA_SDC_CM2)
		w_type = TYPE_OFFSET_DATA_SDC;
	else if (type == TYPE_OFFSET_DATA_SDC_NOT_SAVE)
		w_type = TYPE_OFFSET_DATA_SDC;
	else
		w_type = type;

	/* Set raw type to TYPE_INVALID_DATA if change before */
	ret = ts->sec_ts_read(ts,
		SEC_TS_CMD_MUTU_RAW_TYPE, &ts->ms_frame_type, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			"%s: read rawdata type failed\n",
			__func__);
		goto ErrorExit;
	}

	if (ts->ms_frame_type != TYPE_INVALID_DATA) {
		ret = ts->sec_ts_write(ts, SEC_TS_CMD_MUTU_RAW_TYPE, &mode, 1);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				"%s: recover rawdata type failed\n", __func__);
	}

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_MUTU_RAW_TYPE, &w_type, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: Set rawdata type failed\n", __func__);
		goto ErrorExit;
	}
	ts->ms_frame_type = w_type;

	sec_ts_delay(50);

	if (type == TYPE_OFFSET_DATA_SDC || type == TYPE_OFFSET_DATA_SDC_CM2
		|| type == TYPE_OFFSET_DATA_SDC_NOT_SAVE) {
		/* excute selftest for real cap offset data, because real cap
		 * data is not memory data in normal touch.
		 **/
		char para = TO_TOUCH_MODE;

		disable_irq(ts->client->irq);

		if (type == TYPE_OFFSET_DATA_SDC)
			execute_selftest(ts, TEST_OPEN |
				TEST_NODE_VARIANCE);
		else if (type == TYPE_OFFSET_DATA_SDC_CM2)
			execute_selftest(ts, TEST_OPEN |
				TEST_NOT_SAVE | TEST_HIGH_FREQ);
		else if (type == TYPE_OFFSET_DATA_SDC_NOT_SAVE)
			execute_selftest(ts, TEST_OPEN |
				TEST_NODE_VARIANCE | TEST_NOT_SAVE);

		ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				  "%s: Set powermode failed\n", __func__);
			enable_irq(ts->client->irq);
			goto ErrorRelease;
		}

		/* read data and check ret later */
		ret = ts->sec_ts_read_heap(ts, SEC_TS_READ_TOUCH_RAWDATA, pRead,
					readbytes);
		enable_irq(ts->client->irq);
	} else
		/* read data and check ret later */
		ret = ts->sec_ts_read_heap(ts, SEC_TS_READ_TOUCH_RAWDATA, pRead,
					readbytes);

	/* check read data */
	if (ret < 0) {
		input_err(true, &ts->client->dev,
				  "%s: read rawdata failed!\n", __func__);
		goto ErrorRelease;
	}

	memset(ts->pFrame, 0x00, readbytes);

	for (i = 0; i < readbytes; i += 2)
		ts->pFrame[i / 2] = pRead[i + 1] + (pRead[i] << 8);

#ifdef DEBUG_MSG
	input_info(true, &ts->client->dev,
		   "%s: 02X%02X%02X readbytes=%d\n", __func__,
		   pRead[0], pRead[1], pRead[2], readbytes);
#endif
	sec_ts_print_frame(ts, min, max);

	temp = kzalloc(readbytes, GFP_KERNEL);
	if (!temp)
		goto ErrorRelease;

	memcpy(temp, ts->pFrame, ts->tx_count * ts->rx_count * 2);
	memset(ts->pFrame, 0x00, ts->tx_count * ts->rx_count * 2);

	for (i = 0; i < ts->tx_count; i++) {
		for (j = 0; j < ts->rx_count; j++)
			ts->pFrame[(j * ts->tx_count) + i] =
					temp[(i * ts->rx_count) + j];
	}

#ifdef USE_SPEC_CHECK
	/* spec check */
	if (*spec_check == SPEC_CHECK) {
		int specover_count = 0;
		short dTmp = 0;

		if (type == TYPE_OFFSET_DATA_SDC) {
			unsigned int region = 0;
			/* set initial value for min, max */
			for (i = 0; i < REGION_TYPE_COUNT; i++) {
				min[i] = SHRT_MAX;
				max[i] = SHRT_MIN;
			}

			for (i = 0; i < ts->rx_count; i++) {
				for (j = 0; j < ts->tx_count; j++) {
					dTmp = ts->pFrame[i * ts->tx_count + j];
					region = cm_region[i][j];

					if (region == REGION_NOTCH)
						continue;

					min[region] = min(min[region], dTmp);
					max[region] = max(max[region], dTmp);

					if (dTmp > cm_max[region])
						specover_count++;
					if (dTmp < cm_min[region])
						specover_count++;
				}
			}
			input_info(true, &ts->client->dev,
				   "%s: type = %d, specover = %d\n",
				   __func__, type, specover_count);

			if (specover_count == 0 &&
			    (max[REGION_NORMAL] - min[REGION_NORMAL] <
			     cm_mm[REGION_NORMAL]) &&
			    (max[REGION_EDGE] - min[REGION_EDGE] <
			     cm_mm[REGION_EDGE]) &&
			    (max[REGION_CORNER] - min[REGION_CORNER] <
			     cm_mm[REGION_CORNER]))
				*spec_check = SPEC_PASS;
			else
				*spec_check = SPEC_FAIL;
		} else if (type == TYPE_NOI_P2P_MIN) {
			for (i = 0; i < ts->rx_count; i++) {
				for (j = 0; j < ts->tx_count; j++) {
					dTmp = ts->pFrame[i * ts->tx_count + j];
					if (cm_region[i][j] != REGION_NOTCH &&
					    dTmp < noi_min[i][j])
						specover_count++;
				}
			}
			input_info(true, &ts->client->dev,
				   "%s: type = %d, specover = %d\n",
				   __func__, type, specover_count);

			if (specover_count == 0)
				*spec_check = SPEC_PASS;
			else
				*spec_check = SPEC_FAIL;
		} else if (type == TYPE_NOI_P2P_MAX) {
			for (i = 0; i < ts->rx_count; i++) {
				for (j = 0; j < ts->tx_count; j++) {
					dTmp = ts->pFrame[i * ts->tx_count + j];
					if (cm_region[i][j] != REGION_NOTCH &&
					    dTmp > noi_max[i][j])
						specover_count++;
				}
			}
			input_info(true, &ts->client->dev,
				   "%s: type = %d, specover = %d\n",
				   __func__, type, specover_count);

			if (specover_count == 0)
				*spec_check = SPEC_PASS;
			else
				*spec_check = SPEC_FAIL;
		}
	}
#else
	*spec_check = SPEC_PASS;
#endif

	kfree(temp);

ErrorRelease:
	/* release data monitory (unprepare AFE data memory) */
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_MUTU_RAW_TYPE, &mode, 1);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: Set rawdata type failed\n", __func__);
	else
		ts->ms_frame_type = mode;

ErrorExit:
	kfree(pRead);

	return ret;
}

/* sec_ts_print_channel :
 *  output self raw data to kernel logs.
 *  Please notice that `pFrame` will be changed dynamically by request.
 *  It could be `ts->pFrame` or `ts->pFrameSS` for corresponding purpose.
 */
static void sec_ts_print_channel(struct sec_ts_data *ts, short *pFrame)
{
	unsigned char *pStr = NULL;
	unsigned int str_size, str_len = 0;
	int i = 0, j = 0, k = 0;

	if (!ts->tx_count)
		return;

	str_size = 7 * (ts->tx_count + 1);
	pStr = vzalloc(str_size);
	if (!pStr)
		return;

	str_len = scnprintf(pStr, str_size, " TX");

	for (k = 0; k < ts->tx_count; k++) {
		str_len += scnprintf(pStr + str_len, str_size - str_len,
				     "    %02d", k);
	}
	input_info(true, &ts->client->dev, "%s\n", pStr);

	str_len = scnprintf(pStr, str_size, " +");

	for (k = 0; k < ts->tx_count; k++) {
		str_len += scnprintf(pStr + str_len, str_size - str_len,
				     "------");
	}
	input_info(true, &ts->client->dev, "%s\n", pStr);

	str_len = scnprintf(pStr, str_size, " | ");

	for (i = 0; i < (ts->tx_count + ts->rx_count) * 2; i += 2) {
		if (j == ts->tx_count) {
			input_info(true, &ts->client->dev, "%s\n", pStr);
			input_info(true, &ts->client->dev, "\n");
			str_len = scnprintf(pStr, str_size, " RX");

			for (k = 0; k < ts->tx_count; k++) {
				str_len += scnprintf(pStr + str_len,
						     str_size - str_len,
						     "    %02d", k);
			}

			input_info(true, &ts->client->dev, "%s\n", pStr);

			str_len = scnprintf(pStr, str_size, " +");

			for (k = 0; k < ts->tx_count; k++) {
				str_len += scnprintf(pStr + str_len,
						     str_size - str_len,
						     "------");
			}
			input_info(true, &ts->client->dev, "%s\n", pStr);

			str_len = scnprintf(pStr, str_size, " | ");
		} else if (j && !(j % ts->tx_count)) {
			input_info(true, &ts->client->dev, "%s\n", pStr);
			str_len = scnprintf(pStr, str_size, " | ");
		}

		str_len += scnprintf(pStr + str_len, str_size - str_len, " %5d",
				     pFrame[j]);

		j++;
	}
	input_info(true, &ts->client->dev, "%s\n", pStr);
	vfree(pStr);
}

static int sec_ts_read_channel(struct sec_ts_data *ts, u8 type, short *min,
			       short *max, enum spec_check_type *spec_check)
{
	unsigned char *pRead = NULL;
	u8 mode = TYPE_INVALID_DATA;
	int ret = 0;
	int ii = 0;
	int jj = 0;
	unsigned int data_length = (ts->tx_count + ts->rx_count) * 2;
	u8 w_data;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	pRead = kzalloc(data_length, GFP_KERNEL);
	if (!pRead)
		return -ENOMEM;

	/* set OPCODE and data type */
	if (type == TYPE_OFFSET_DATA_SDC_NOT_SAVE)
		w_data = TYPE_OFFSET_DATA_SDC;
	else
		w_data = type;
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SELF_RAW_TYPE, &w_data, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			"%s: Set rawdata type failed\n", __func__);
		goto out_read_channel;
	}

	sec_ts_delay(50);

	if (type == TYPE_OFFSET_DATA_SDC ||
		type == TYPE_OFFSET_DATA_SDC_NOT_SAVE) {
		/* excute selftest for real cap offset data, because real cap
		 * data is not memory data in normal touch.
		 **/
		char para = TO_TOUCH_MODE;

		disable_irq(ts->client->irq);
		if (type == TYPE_OFFSET_DATA_SDC)
			execute_selftest(ts, TEST_SELF_NODE);
		else if (type == TYPE_OFFSET_DATA_SDC_NOT_SAVE)
			execute_selftest(ts, TEST_SELF_NODE | TEST_NOT_SAVE);
		ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				  "%s: set rawdata type failed!\n", __func__);
			enable_irq(ts->client->irq);
			goto err_read_data;
		}

		/* read data and check ret later */
		ret = ts->sec_ts_read_heap(ts, SEC_TS_READ_TOUCH_SELF_RAWDATA,
					pRead, data_length);
		enable_irq(ts->client->irq);
		/* end */
	} else
		/* read data and check ret later */
		ret = ts->sec_ts_read_heap(ts, SEC_TS_READ_TOUCH_SELF_RAWDATA,
					pRead, data_length);

	/* check read data */
	if (ret < 0) {
		input_err(true, &ts->client->dev,
				  "%s: read rawdata failed!\n", __func__);
		goto err_read_data;
	}

	/* clear all pFrame data */
	memset(ts->pFrame, 0x00, data_length);

/* d[00] ~ d[14] : TX channel
 * d[15] ~ d[51] : none
 * d[52] ~ d[77] : RX channel
 * d[78] ~ d[103] : none
 */
	for (ii = 0; ii < data_length; ii += 2) {
		ts->pFrame[jj] = ((pRead[ii] << 8) | pRead[ii + 1]);
		jj++;
	}

	sec_ts_print_channel(ts, ts->pFrame);

#ifdef USE_SPEC_CHECK
	if (*spec_check == SPEC_CHECK) {
		int specover_count = 0;

		if (type == TYPE_OFFSET_DATA_SDC) {
			min[0] = min[1] = SHRT_MAX;
			max[0] = max[1] = SHRT_MIN;

			for (ii = 0; ii < ts->tx_count; ii++) {
				if (ts->pFrame[ii] > cs_tx_max)
					specover_count++;
				if (ts->pFrame[ii] < cs_tx_min)
					specover_count++;
				min[0] = min(min[0], ts->pFrame[ii]);
				max[0] = max(max[0], ts->pFrame[ii]);
			}

			for (ii = ts->tx_count;
			     ii < ts->tx_count + ts->rx_count; ii++) {
				if (ts->pFrame[ii] > cs_rx_max)
					specover_count++;
				if (ts->pFrame[ii] < cs_rx_min)
					specover_count++;
				min[1] = min(min[1], ts->pFrame[ii]);
				max[1] = max(max[1], ts->pFrame[ii]);
			}
		}

		input_info(true, &ts->client->dev,
			   "%s: type : %d, specover = %d\n",
			   __func__, type, specover_count);
		if (specover_count == 0 &&
			(max[0] - min[0]) < cs_tx_mm &&
			(max[1] - min[1]) < cs_rx_mm)
			*spec_check = SPEC_PASS;
		else
			*spec_check = SPEC_FAIL;
	}
#else
	*spec_check = SPEC_PASS;
#endif

err_read_data:
	/* release data monitory (unprepare AFE data memory) */
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SELF_RAW_TYPE, &mode, 1);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: Set rawdata type failed\n", __func__);

out_read_channel:
	kfree(pRead);

	return ret;
}

int sec_ts_save_mutual_raw_to_buffer(struct sec_ts_data *ts,
		struct sec_cmd_data *sec, struct sec_ts_test_mode *mode,
		char *buff, const unsigned int buff_size)
{
	int ii, jj;
	unsigned int buff_len = 0;

	if (mode->spec_check == SPEC_NO_CHECK)
		buff_len += scnprintf(buff + buff_len,
				      buff_size - buff_len, "\n");
	else if (mode->spec_check == SPEC_PASS) {
		buff_len += scnprintf(buff + buff_len,
				      buff_size - buff_len,
				      "OK %d %d\n",
				      ts->rx_count, ts->tx_count);
	} else {	/* mode->spec_check == SPEC_FAIL) */
		buff_len += scnprintf(buff + buff_len,
				      buff_size - buff_len,
				      "NG %d %d\n", ts->rx_count,
				      ts->tx_count);
	}
	if (!ts->print_format) {
		for (ii = 0;
		     ii < (ts->rx_count * ts->tx_count); ii++) {
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len,
					      "%3d,", ts->pFrame[ii]);
			if (ii % ts->tx_count == (ts->tx_count - 1))
				buff_len += scnprintf(buff + buff_len,
						      buff_size - buff_len,
						      "\n");
		}
	} else {
		for (ii = 0; ii < ts->tx_count; ii++) {
			for (jj = 0; jj < ts->rx_count; jj++) {
				buff_len += scnprintf(
						buff + buff_len,
						buff_size - buff_len,
						"%3d,",
						ts->pFrame[(jj *
							ts->tx_count) + ii]);
			}
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len,
					      "\n");
		}
	}

	return buff_len;
}

int sec_ts_save_self_raw_to_buffer(struct sec_ts_data *ts,
		struct sec_cmd_data *sec, struct sec_ts_test_mode *mode,
		char *buff, const unsigned int buff_size)
{
	int ii;
	unsigned int buff_len = 0;
	short *pFrame = ts->pFrame;

	if (mode->frame_channel == TEST_MODE_READ_ALL)
		pFrame = ts->pFrameSS;

	if (mode->spec_check == SPEC_NO_CHECK)
		buff_len += scnprintf(buff + buff_len,
				      buff_size - buff_len, "\n");
	else if (mode->spec_check == SPEC_PASS) {
		buff_len += scnprintf(buff + buff_len,
				      buff_size - buff_len,
				      "OK %d %d\n",
				      ts->rx_count, ts->tx_count);
	} else {	/* mode->spec_check == SPEC_FAIL) */
		buff_len += scnprintf(buff + buff_len,
				      buff_size - buff_len,
				      "NG %d %d\n",
				      ts->rx_count, ts->tx_count);
	}
	buff_len += scnprintf(buff + buff_len,
			      buff_size - buff_len, "      ");
	if (!ts->print_format) {
		for (ii = 0;
		     ii < (ts->rx_count + ts->tx_count);
		     ii++) {
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len,
					      "%3d,", pFrame[ii]);
			if (ii >= ts->tx_count - 1)
				buff_len += scnprintf(
						buff + buff_len,
						buff_size - buff_len,
						"\n");
		}
	} else {
		for (ii = ts->tx_count;
		     ii < (ts->rx_count + ts->tx_count);
		     ii++) {
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len,
					      "%3d,", pFrame[ii]);
		}
		buff_len += scnprintf(buff + buff_len,
				      buff_size - buff_len, "\n");
		for (ii = 0; ii < ts->tx_count; ii++) {
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len,
					      "%3d,\n", pFrame[ii]);
		}
	}

	return buff_len;
}

static int sec_ts_read_frame_and_channel(struct sec_ts_data *ts, u8 type,
		short *min, short *max, enum spec_check_type *spec_check)
{
	unsigned int readbytes_mutual = 0xFF;
	unsigned int readbytes_self = 0xFF;
	u8 *pReadMutual = NULL;
	u8 *pReadSelf = NULL;
	u8 mode = TYPE_INVALID_DATA;
	int ret = 0;
	int i = 0;
	int j = 0;
	short *temp = NULL;
	short *pFrame = NULL;
	u8 w_type;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	/* set data length, allocation buffer memory */
	readbytes_mutual = ts->rx_count * ts->tx_count * 2;
	readbytes_self = (ts->rx_count + ts->tx_count) * 2;

	pReadMutual = kzalloc(readbytes_mutual, GFP_KERNEL);
	pReadSelf = kzalloc(readbytes_self, GFP_KERNEL);
	if (!pReadMutual || !pReadSelf) {
		kfree(pReadMutual);
		kfree(pReadSelf);
		return -ENOMEM;
	}

	/* set OPCODE and data type */
	if (type == TYPE_OFFSET_DATA_SDC_CM2)
		w_type = TYPE_OFFSET_DATA_SDC;
	else if (type == TYPE_OFFSET_DATA_SDC_NOT_SAVE)
		w_type = TYPE_OFFSET_DATA_SDC;
	else
		w_type = type;

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_MUTU_RAW_TYPE, &w_type, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: Set rawdata type failed\n", __func__);
		goto ErrorExit;
	}
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SELF_RAW_TYPE, &w_type, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: Set rawdata type failed\n", __func__);
		goto ErrorExit;
	}

	ts->ms_frame_type = w_type;
	ts->ss_frame_type = w_type;

	sec_ts_delay(50);

	if (type == TYPE_OFFSET_DATA_SDC || type == TYPE_OFFSET_DATA_SDC_CM2
		|| type == TYPE_OFFSET_DATA_SDC_NOT_SAVE) {
		/* excute selftest for real cap offset data, because real cap
		 * data is not memory data in normal touch.
		 **/
		char para = TO_TOUCH_MODE;

		disable_irq(ts->client->irq);

		if (type == TYPE_OFFSET_DATA_SDC)
			execute_selftest(ts, TEST_OPEN |
				TEST_NODE_VARIANCE | TEST_SELF_NODE);
		else if (type == TYPE_OFFSET_DATA_SDC_CM2)
			execute_selftest(ts, TEST_OPEN |
				TEST_NOT_SAVE | TEST_HIGH_FREQ |
				TEST_SELF_NODE);
		else if (type == TYPE_OFFSET_DATA_SDC_NOT_SAVE)
			execute_selftest(ts, TEST_OPEN |
				TEST_NODE_VARIANCE | TEST_NOT_SAVE |
				TEST_SELF_NODE);

		ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				  "%s: Set powermode failed\n", __func__);
			enable_irq(ts->client->irq);
			goto ErrorRelease;
		}

		/* read data and check ret later */
		ret = ts->sec_ts_read_heap(ts, SEC_TS_READ_TOUCH_RAWDATA,
					pReadMutual, readbytes_mutual);
		ret |= ts->sec_ts_read_heap(ts, SEC_TS_READ_TOUCH_SELF_RAWDATA,
					pReadSelf, readbytes_self);
		enable_irq(ts->client->irq);

	} else {
		/* read data and check ret later */
		ret = ts->sec_ts_read_heap(ts, SEC_TS_READ_TOUCH_RAWDATA,
					pReadMutual, readbytes_mutual);
		ret |= ts->sec_ts_read_heap(ts, SEC_TS_READ_TOUCH_SELF_RAWDATA,
					pReadSelf, readbytes_self);
	}
	/* check read data */
	if (ret < 0) {
		input_err(true, &ts->client->dev,
				  "%s: read rawdata failed!\n", __func__);
		goto ErrorRelease;
	}

/* handle mutual raw data */
	pFrame = ts->pFrameMS;
	/* mutual data decode and print screen */
	memset(pFrame, 0x00, readbytes_mutual);

	for (i = 0; i < readbytes_mutual; i += 2)
		pFrame[i / 2] = pReadMutual[i + 1] + (pReadMutual[i] << 8);

#ifdef DEBUG_MSG
	input_info(true, &ts->client->dev,
		"%s: 02X%02X%02X readbytes=%d\n", __func__,
		pReadMutual[0], pReadMutual[1], pReadMutual[2],
		readbytes_mutual);
#endif
	sec_ts_print_frame(ts, min, max);

	temp = kzalloc(readbytes_mutual, GFP_KERNEL);
	if (!temp)
		goto ErrorRelease;

	memcpy(temp, pFrame, ts->tx_count * ts->rx_count * 2);
	memset(pFrame, 0x00, ts->tx_count * ts->rx_count * 2);

	for (i = 0; i < ts->tx_count; i++) {
		for (j = 0; j < ts->rx_count; j++)
			pFrame[(j * ts->tx_count) + i] =
				temp[(i * ts->rx_count) + j];
	}

#ifdef USE_SPEC_CHECK
	/* spec check */
	if (*spec_check == SPEC_CHECK) {
		int specover_count = 0;
		short dTmp = 0;

		if (type == TYPE_OFFSET_DATA_SDC) {
			unsigned int region = 0;
			/* set initial value for min, max */
			for (i = 0; i < REGION_TYPE_COUNT; i++) {
				min[i] = SHRT_MAX;
				max[i] = SHRT_MIN;
			}

			for (i = 0; i < ts->rx_count; i++) {
				for (j = 0; j < ts->tx_count; j++) {
					dTmp = pFrame[i * ts->tx_count + j];
					region = cm_region[i][j];

					if (region == REGION_NOTCH)
						continue;

					min[region] = min(min[region], dTmp);
					max[region] = max(max[region], dTmp);

					if (dTmp > cm_max[region])
						specover_count++;
					if (dTmp < cm_min[region])
						specover_count++;
				}
			}
			input_info(true, &ts->client->dev,
				   "%s: type = %d, specover = %d\n",
				   __func__, type, specover_count);

			if (specover_count == 0 &&
			    (max[REGION_NORMAL] - min[REGION_NORMAL] <
			     cm_mm[REGION_NORMAL]) &&
			    (max[REGION_EDGE] - min[REGION_EDGE] <
			     cm_mm[REGION_EDGE]) &&
			    (max[REGION_CORNER] - min[REGION_CORNER] <
			     cm_mm[REGION_CORNER]))
				*spec_check = SPEC_PASS;
			else
				*spec_check = SPEC_FAIL;
		} else if (type == TYPE_NOI_P2P_MIN) {
			for (i = 0; i < ts->rx_count; i++) {
				for (j = 0; j < ts->tx_count; j++) {
					dTmp = pFrame[i * ts->tx_count + j];
					if (cm_region[i][j] != REGION_NOTCH &&
					    dTmp < noi_min[i][j])
						specover_count++;
				}
			}
			input_info(true, &ts->client->dev,
				   "%s: type = %d, specover = %d\n",
				   __func__, type, specover_count);

			if (specover_count == 0)
				*spec_check = SPEC_PASS;
			else
				*spec_check = SPEC_FAIL;
		} else if (type == TYPE_NOI_P2P_MAX) {
			for (i = 0; i < ts->rx_count; i++) {
				for (j = 0; j < ts->tx_count; j++) {
					dTmp = pFrame[i * ts->tx_count + j];
					if (cm_region[i][j] != REGION_NOTCH &&
					    dTmp > noi_max[i][j])
						specover_count++;
				}
			}
			input_info(true, &ts->client->dev,
				   "%s: type = %d, specover = %d\n",
				   __func__, type, specover_count);

			if (specover_count == 0)
				*spec_check = SPEC_PASS;
			else
				*spec_check = SPEC_FAIL;
		}
	}
#else
		*spec_check = SPEC_PASS;
#endif

	kfree(temp);

/* handle self raw data */
	pFrame = ts->pFrameSS;
	/* clear all pFrame data */
	memset(pFrame, 0x00, readbytes_self);

/* d[00] ~ d[14] : TX channel
 * d[15] ~ d[51] : none
 * d[52] ~ d[77] : RX channel
 * d[78] ~ d[103] : none
 */
	for (i = 0, j = 0; i < readbytes_self; i += 2) {
		pFrame[j] = ((pReadSelf[i] << 8) | pReadSelf[i + 1]);
		j++;
	}

	sec_ts_print_channel(ts, pFrame);

#ifdef USE_SPEC_CHECK
	if (*spec_check == SPEC_CHECK) {
		int specover_count = 0;

		if (type == TYPE_OFFSET_DATA_SDC) {
			min[0] = min[1] = SHRT_MAX;
			max[0] = max[1] = SHRT_MIN;

			for (i = 0; i < ts->tx_count; i++) {
				if (pFrame[i] > cs_tx_max)
					specover_count++;
				if (pFrame[i] < cs_tx_min)
					specover_count++;
				min[0] = min(min[0], pFrame[i]);
				max[0] = max(max[0], pFrame[i]);
			}
			for (i = ts->tx_count;
			     i < ts->tx_count + ts->rx_count; i++) {
				if (pFrame[i] > cs_rx_max)
					specover_count++;
				if (pFrame[i] < cs_rx_min)
					specover_count++;
				min[1] = min(min[1], pFrame[i]);
				max[1] = max(max[1], pFrame[i]);
			}
		}

		input_info(true, &ts->client->dev,
			   "%s: type : %d, specover = %d\n",
			   __func__, type, specover_count);
		if (specover_count == 0 &&
			(max[0] - min[0]) < cs_tx_mm &&
			(max[1] - min[1]) < cs_rx_mm)
			*spec_check = SPEC_PASS;
		else
			*spec_check = SPEC_FAIL;
	}
#else
	*spec_check = SPEC_PASS;
#endif


ErrorRelease:
	/* release data monitory (unprepare AFE data memory) */
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_MUTU_RAW_TYPE, &mode, 1);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: Set rawdata type failed\n", __func__);
	else
		ts->ms_frame_type = mode;

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SELF_RAW_TYPE, &mode, 1);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: Set rawdata type failed\n", __func__);
	else
		ts->ss_frame_type = mode;
ErrorExit:
	kfree(pReadMutual);
	kfree(pReadSelf);

	return ret;
}

#ifdef USE_STIM_PAD
static int sec_ts_read_gain_table(struct sec_ts_data *ts)
{
	int readbytes = ts->tx_count * ts->rx_count;
	unsigned char *pRead = NULL;
	short min = 0;
	short max = 0;
	int ret;
	int i;

	/* readbytes : 1 byte for enable/disable info + 1 byte per node */
	pRead = kzalloc(1 + readbytes, GFP_KERNEL);
	if (!pRead)
		return -ENOMEM;

	ret = ts->sec_ts_read_heap(ts, SEC_TS_CMD_READ_NORM_TABLE, pRead,
				1 + readbytes);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: read rawdata failed!\n",
			  __func__);
		goto ErrorRead;
	}

	input_info(true, &ts->client->dev, "%s: gain table is %s\n",
		   __func__, pRead[0] ? "On." : "Off.");

	for (i = 0; i < readbytes; i++)
		ts->pFrame[i] = (short)pRead[i + 1];

	sec_ts_print_frame(ts, &min, &max);

ErrorRead:
	kfree(pRead);

	return ret;
}

int sec_ts_check_fs_precal(struct sec_ts_data *ts)
{
	int i, j;
	int fail_count = 0;
	short temp;

	for (i = 0; i < ts->rx_count; i++) {
		for (j = 0; j < ts->tx_count; j++) {
			temp = ts->pFrame[i * ts->tx_count + j];
			if (cm_region[i][j] == REGION_NOTCH)
				continue;
			/* check whether fs_precal data is within range */
			if ((temp > fs_precal_h[i][j]) ||
				(temp < fs_precal_l[i][j]))
				fail_count++;
		}
	}

	return fail_count;
}
#endif

int sec_ts_read_raw_data(struct sec_ts_data *ts,
		struct sec_cmd_data *sec, struct sec_ts_test_mode *mode)
{
	int ret = 0;
	const unsigned int buff_size = ts->tx_count * ts->rx_count *
					CMD_RESULT_WORD_LEN;
	unsigned int buff_len = 0;
	char *buff;

	buff = kzalloc(buff_size, GFP_KERNEL);
	if (!buff)
		goto error_alloc_mem;

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: [ERROR] Touch is stopped\n", __func__);
		goto error_power_state;
	}

	input_info(true, &ts->client->dev, "%s: %d, %s\n",
			__func__, mode->type, mode->allnode ? "ALL" : "");

	ret = sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH,
			       TOUCH_MODE_STATE_TOUCH);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: failed to fix tmode\n",
				__func__);
		goto error_test_fail;
	}

	if (mode->frame_channel == TEST_MODE_READ_CHANNEL)
		ret = sec_ts_read_channel(ts, mode->type, mode->min,
					  mode->max, &mode->spec_check);
	else if (mode->frame_channel == TEST_MODE_READ_FRAME)
		ret = sec_ts_read_frame(ts, mode->type, mode->min,
					mode->max, &mode->spec_check);
	else if (mode->frame_channel == TEST_MODE_READ_ALL)
		ret = sec_ts_read_frame_and_channel(ts, mode->type, mode->min,
				      mode->max, &mode->spec_check);
	else
		ret = -EINVAL;

	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: failed to read frame\n",
				__func__);
		goto error_test_fail;
	}

	if (mode->allnode) {
		if (mode->frame_channel == TEST_MODE_READ_FRAME ||
		    mode->frame_channel == TEST_MODE_READ_ALL) {
			buff_len += sec_ts_save_mutual_raw_to_buffer(ts, sec,
				mode, buff + buff_len, buff_size - buff_len);
		}

		if (mode->frame_channel == TEST_MODE_READ_CHANNEL ||
		    mode->frame_channel == TEST_MODE_READ_ALL) {
			buff_len += sec_ts_save_self_raw_to_buffer(ts, sec,
				mode, buff + buff_len, buff_size - buff_len);
		}
	} else {
#ifdef USE_SPEC_CHECK
		buff_len += scnprintf(buff + buff_len, buff_size - buff_len,
				      "%3d,%3d", mode->min[0], mode->max[0]);
#else
		buff_len += scnprintf(buff + buff_len, buff_size - buff_len,
				      "OK");
#endif
	}

	ret = sec_ts_release_tmode(ts);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: failed to release tmode\n", __func__);
		goto error_test_fail;
	}

	if (!sec)
		goto out_rawdata;
	sec_cmd_set_cmd_result(sec, buff, buff_len);
	sec->cmd_state = SEC_CMD_STATUS_OK;

out_rawdata:
	kfree(buff);

	sec_ts_locked_release_all_finger(ts);

	return ret;

error_test_fail:
error_power_state:
	kfree(buff);
error_alloc_mem:
	if (!sec)
		return ret;

	sec_cmd_set_cmd_result(sec, "FAIL", 4);
	sec->cmd_state = SEC_CMD_STATUS_FAIL;

	sec_ts_locked_release_all_finger(ts);

	return ret;
}

static void get_fw_ver_bin(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[16] = { 0 };

	sec_cmd_set_default_result(sec);

	snprintf(buff, sizeof(buff), "SE-V%02X.%02X.%02X",
		ts->plat_data->panel_revision,
		ts->plat_data->img_version_of_bin[2],
		ts->plat_data->img_version_of_bin[3]);

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
}

static void get_fw_ver_ic(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[16] = { 0 };
	int ret;
	u8 fw_ver[4];

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: [ERROR] Touch is stopped\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	ret = ts->sec_ts_read(ts, SEC_TS_READ_IMG_VERSION, fw_ver, 4);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: firmware version read error\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	snprintf(buff, sizeof(buff), "SE-V%02X.%02X.%02X",
			ts->plat_data->panel_revision, fw_ver[2], fw_ver[3]);

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void get_config_ver(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[22] = { 0 };

	sec_cmd_set_default_result(sec);

	snprintf(buff, sizeof(buff), "%s_SE_%02X%02X",
		ts->plat_data->model_name,
		ts->plat_data->config_version_of_ic[2],
		ts->plat_data->config_version_of_ic[3]);

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
}

#ifdef PAT_CONTROL
static void get_pat_information(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[22] = { 0 };

	sec_cmd_set_default_result(sec);

	/* fixed tune version will be saved at excute autotune */
	snprintf(buff, sizeof(buff), "P%02XT%04X",
		ts->cal_count, ts->tune_fix_ver);

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
}

static void set_external_factory(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[22] = { 0 };

	sec_cmd_set_default_result(sec);

	ts->external_factory = true;
	snprintf(buff, sizeof(buff), "OK");

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
}
#endif

static void get_threshold(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[20] = { 0 };
	char threshold[2] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: [ERROR] Touch is stopped\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		goto err;
	}

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_TOUCH_MODE_FOR_THRESHOLD,
			       threshold, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: threshold write type failed. ret: %d\n",
			  __func__, ret);
		snprintf(buff, sizeof(buff), "%s", "NG");
		goto err;
	}

	ret = ts->sec_ts_read(ts, SEC_TS_CMD_TOUCH_THRESHOLD, threshold, 2);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: read threshold fail!\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "NG");
		goto err;
	}

	input_info(true, &ts->client->dev, "0x%02X, 0x%02X\n",
				threshold[0], threshold[1]);

	snprintf(buff, sizeof(buff), "%d", (threshold[0] << 8) | threshold[1]);

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;
err:
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void module_off_master(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[3] = { 0 };
	int ret = 0;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	ret = sec_ts_stop_device(ts);

	if (ret == 0)
		snprintf(buff, sizeof(buff), "%s", "OK");
	else
		snprintf(buff, sizeof(buff), "%s", "NG");

	sec_cmd_set_default_result(sec);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	if (strncmp(buff, "OK", 2) == 0)
		sec->cmd_state = SEC_CMD_STATUS_OK;
	else
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void module_on_master(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[3] = { 0 };
	int ret = 0;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	ret = sec_ts_start_device(ts);

/* TODO: check this for SPI case
 *	if (ts->input_dev->disabled) {
 *		sec_ts_set_lowpowermode(ts, TO_LOWPOWER_MODE);
 *		ts->power_status = SEC_TS_STATE_LPM;
 *	}
 **/

	if (ret == 0)
		snprintf(buff, sizeof(buff), "%s", "OK");
	else
		snprintf(buff, sizeof(buff), "%s", "NG");

	sec_cmd_set_default_result(sec);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	if (strncmp(buff, "OK", 2) == 0)
		sec->cmd_state = SEC_CMD_STATUS_OK;
	else
		sec->cmd_state = SEC_CMD_STATUS_FAIL;

	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void get_chip_vendor(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[16] = { 0 };

	strncpy(buff, "SEC", sizeof(buff));
	sec_cmd_set_default_result(sec);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
}

static void get_chip_name(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[16] = { 0 };

	if (ts->plat_data->img_version_of_ic[0] == 0x02)
		strncpy(buff, "MC44", sizeof(buff));
	else if (ts->plat_data->img_version_of_ic[0] == 0x05)
		strncpy(buff, "A552", sizeof(buff));
	else if (ts->plat_data->img_version_of_ic[0] == 0x09)
		strncpy(buff, "Y661", sizeof(buff));
	else if (ts->plat_data->img_version_of_ic[0] == 0x10)
		strncpy(buff, "Y761", sizeof(buff));
	else
		strncpy(buff, "N/A", sizeof(buff));

	sec_cmd_set_default_result(sec);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
}

static void set_mis_cal_spec(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[16] = { 0 };
	char wreg[5] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->plat_data->mis_cal_check == 0) {
		input_err(true, &ts->client->dev,
			  "%s: [ERROR] not support, %d\n", __func__);
		goto NG;
	} else if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: [ERROR] Touch is stopped\n", __func__);
		goto NG;
	} else {
		if ((sec->cmd_param[0] < 0 || sec->cmd_param[0] > 255) ||
			(sec->cmd_param[1] < 0 || sec->cmd_param[1] > 255) ||
			(sec->cmd_param[2] < 0 || sec->cmd_param[2] > 255)) {
			snprintf(buff, sizeof(buff), "%s", "NG");
			goto NG;
		} else {
			wreg[0] = sec->cmd_param[0];
			wreg[1] = sec->cmd_param[1];
			wreg[2] = sec->cmd_param[2];

			ret = ts->sec_ts_write(ts, SEC_TS_CMD_MIS_CAL_SPEC,
					       wreg, 3);
			if (ret < 0) {
				input_err(true, &ts->client->dev,
					  "%s: nvm write failed. ret: %d\n",
					  __func__, ret);
				goto NG;
			} else {
				input_info(true, &ts->client->dev,
					   "%s: tx gap=%d, rx gap=%d, peak=%d\n",
					   __func__, wreg[0], wreg[1], wreg[2]);
				sec_ts_delay(20);
			}
		}
	}

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

NG:
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;


}

/*
 *	## Mis Cal result ##
 *	FF : initial value in Firmware.
 *	FD : Cal fail case
 *	F4 : fail case (5F)
 *	F3 : fail case (5E)
 *	F2 : power off state
 *	F1 : not support mis cal concept
 *	F0 : initial value in fucntion
 *	08 : Ambient Ambient condition check(PEAK) result 0 (PASS), 1(FAIL)
 *	04 : Ambient Ambient condition check(DIFF MAX TX)
 *	     result 0 (PASS), 1(FAIL)
 *	02 : Ambient Ambient condition check(DIFF MAX RX)
 *	     result 0 (PASS), 1(FAIL)
 *	01 : Wet Wet mode result 0 (PASS), 1(FAIL)
 *	00 : Pass
 **/
static void get_mis_cal_info(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[16] = { 0 };
	char mis_cal_data = 0xF0;
	char wreg[5] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->plat_data->mis_cal_check == 0) {
		input_err(true, &ts->client->dev,
			  "%s: [ERROR] not support, %d\n", __func__);
		mis_cal_data = 0xF1;
		goto NG;
	} else if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: [ERROR] Touch is stopped\n", __func__);
		mis_cal_data = 0xF2;
		goto NG;
	} else {
		ret = ts->sec_ts_read(ts, SEC_TS_CMD_MIS_CAL_READ,
				      &mis_cal_data, 1);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				  "%s: fail!, %d\n", __func__, ret);
			mis_cal_data = 0xF3;
			goto NG;
		} else {
			input_info(true, &ts->client->dev,
				   "%s: miss cal data : %d\n",
				   __func__, mis_cal_data);
		}

		ret = ts->sec_ts_read(ts, SEC_TS_CMD_MIS_CAL_SPEC, wreg, 3);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				  "%s: fail!, %d\n", __func__, ret);
			mis_cal_data = 0xF4;
			goto NG;
		} else {
			input_info(true, &ts->client->dev,
				"%s: miss cal spec : %d,%d,%d\n", __func__,
				wreg[0], wreg[1], wreg[2]);
		}
	}

	snprintf(buff, sizeof(buff), "%d,%d,%d,%d",
		 mis_cal_data, wreg[0], wreg[1], wreg[2]);

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

NG:
	snprintf(buff, sizeof(buff), "%d,%d,%d,%d", mis_cal_data, 0, 0, 0);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void get_wet_mode(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[16] = { 0 };
	char wet_mode_info = 0;
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	ret = ts->sec_ts_read(ts, SEC_TS_CMD_WET_MODE, &wet_mode_info, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: fail!, %d\n", __func__, ret);
		goto NG;
	}

	snprintf(buff, sizeof(buff), "%d", wet_mode_info);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

NG:
	snprintf(buff, sizeof(buff), "NG");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void get_x_num(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[16] = { 0 };

	sec_cmd_set_default_result(sec);
	snprintf(buff, sizeof(buff), "%d", ts->tx_count);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
}

static void get_y_num(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[16] = { 0 };

	sec_cmd_set_default_result(sec);
	snprintf(buff, sizeof(buff), "%d", ts->rx_count);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
}

static void get_x_cross_routing(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[16] = { 0 };

	sec_cmd_set_default_result(sec);
	snprintf(buff, sizeof(buff), "NG");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
}

static void get_y_cross_routing(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[16] = { 0 };
	int ret;

	sec_cmd_set_default_result(sec);

	ret = strncmp(ts->plat_data->model_name, "G935", 4)
			&& strncmp(ts->plat_data->model_name, "N930", 4);
	if (ret == 0)
		snprintf(buff, sizeof(buff), "13,14");
	else
		snprintf(buff, sizeof(buff), "NG");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
}

static void get_checksum_data(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[16] = { 0 };
	char csum_result[4] = { 0 };
	u8 cal_result;
	u8 nv_result;
	u8 temp;
	u8 csum = 0;
	int ret, i;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: [ERROR] Touch is stopped\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		goto err;
	}

	temp = DO_FW_CHECKSUM | DO_PARA_CHECKSUM;
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_GET_CHECKSUM, &temp, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: send get_checksum_cmd fail!\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "SendCMDfail");
		goto err;
	}

	sec_ts_delay(20);

#ifdef I2C_INTERFACE
	ret = ts->sec_ts_read_bulk(ts, csum_result, 4);
#else
	ret = ts->sec_ts_read(ts, SEC_TS_CMD_GET_CHECKSUM, csum_result, 4);
#endif
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: read get_checksum result fail!\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "ReadCSUMfail");
		goto err;
	}

	nv_result = get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_FAC_RESULT);
	nv_result += get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_CAL_COUNT);

	cal_result = sec_ts_read_calibration_report(ts);

	for (i = 0; i < 4; i++)
		csum += csum_result[i];

	csum += nv_result;
	csum += cal_result;

	csum = ~csum;

	input_info(true, &ts->client->dev,
		   "%s: checksum = %02X\n", __func__, csum);
	snprintf(buff, sizeof(buff), "%02X", csum);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

err:
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_reference_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_OFFSET_DATA_SEC;

	sec_ts_read_raw_data(ts, sec, &mode);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_reference_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_OFFSET_DATA_SEC;
	mode.allnode = TEST_MODE_ALL_NODE;

	sec_ts_read_raw_data(ts, sec, &mode);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void get_reference(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	short val = 0;
	int node = 0;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: [ERROR] Touch is stopped\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;

		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	node = sec_ts_check_index(ts);
	if (node < 0) {
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	val = ts->pFrame[node];
	snprintf(buff, sizeof(buff), "%d", val);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_rawcap_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_OFFSET_DATA_SDC;
#ifdef USE_SPEC_CHECK
	mode.spec_check = SPEC_CHECK;
#endif

	sec_ts_read_raw_data(ts, sec, &mode);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_rawcap_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_OFFSET_DATA_SDC;
	mode.allnode = TEST_MODE_ALL_NODE;
#ifdef USE_SPEC_CHECK
	mode.spec_check = SPEC_CHECK;
#endif

	sec_ts_read_raw_data(ts, sec, &mode);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

/* run_rawcap_combo_read_all :
 * Combine Cm/Cs offset test to merge
 * run_rawcap_read_all() and run_self_rawcap_read_all().
 */
static void run_rawcap_combo_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_OFFSET_DATA_SDC;
	mode.frame_channel = TEST_MODE_READ_ALL;
	mode.allnode = TEST_MODE_ALL_NODE;
#ifdef USE_SPEC_CHECK
	mode.spec_check = SPEC_CHECK;
#endif

	sec_ts_read_raw_data(ts, sec, &mode);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

int sec_ts_save_self_gap_raw_to_buffer(struct sec_ts_data *ts,
		struct sec_ts_test_mode *mode, short *gap,
		char *buff, const unsigned int buff_size)
{
	int i;
	unsigned int buff_len = 0;

	if (mode->spec_check == SPEC_NO_CHECK)
		buff_len = scnprintf(buff + buff_len,
				      buff_size - buff_len, "\n");
	else if (mode->spec_check == SPEC_PASS) {
		buff_len = scnprintf(buff + buff_len,
				      buff_size - buff_len,
				      "OK\n");
	} else {	/* mode->spec_check == SPEC_FAIL) */
		buff_len = scnprintf(buff + buff_len,
				      buff_size - buff_len,
				      "NG\n");
	}

	for (i = 0; i < (ts->tx_count - 1); i++) {
		buff_len += scnprintf(buff + buff_len, buff_size - buff_len,
				      "%6d,", gap[i]);
	}

	buff_len += scnprintf(buff + buff_len, buff_size - buff_len, "\n");

	for (i = ts->tx_count; i < ts->tx_count + (ts->rx_count - 1); i++) {
		buff_len += scnprintf(buff + buff_len, buff_size - buff_len,
				      "%6d,\n", gap[i]);
	}
	return buff_len;
}

int sec_ts_save_mutual_gap_raw_to_buffer(struct sec_ts_data *ts,
		struct sec_ts_test_mode *mode, short *gap_x, short *gap_y,
		char *buff, const unsigned int buff_size)
{
	int i;
	unsigned int buff_len = 0;
	short dTmp;

	if (mode->spec_check == SPEC_NO_CHECK)
		buff_len = scnprintf(buff + buff_len,
				      buff_size - buff_len, "\n");
	else if (mode->spec_check == SPEC_PASS) {
		buff_len = scnprintf(buff + buff_len,
				      buff_size - buff_len,
				      "OK\n");
	} else {	/* mode->spec_check == SPEC_FAIL) */
		buff_len = scnprintf(buff + buff_len,
				      buff_size - buff_len,
				      "NG\n");
	}

	for (i = 0; i < (ts->tx_count * ts->rx_count); i++) {

		dTmp = (gap_x[i] > gap_y[i]) ? gap_x[i] : gap_y[i];
		buff_len += scnprintf(buff + buff_len, buff_size - buff_len,
					  "%3d,", dTmp);

		if (i % ts->tx_count == (ts->tx_count - 1))
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len, "\n");
	}
	return buff_len;
}
/* run_rawcap_factory_read_all :
 * Combine Cm/Cs offset and gap test to merge
 * run_rawcap_read_all(), run_self_rawcap_read_all().
 * run_rawcap_gap_read_all() and run_self_rawcap_gap_read_all().
 */
static void run_rawcap_factory_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;

	int ret_x, ret_y, ret;

	unsigned int raw_buff_len = 0;
	unsigned int gap_buff_len = 0;

	short *gap, *gap_x, *gap_y = NULL;
	char *gap_buff = NULL;
	char *raw_buff = NULL;

	const unsigned int raw_buff_size = ts->tx_count * ts->rx_count *
		CMD_RESULT_WORD_LEN + (ts->tx_count + ts->rx_count) *
		CMD_RESULT_WORD_LEN;

	const unsigned int mutual_gap_buff_size =
		ts->tx_count * ts->rx_count * 2;
	const unsigned int mutual_buff_size = ts->tx_count * ts->rx_count * 2
		* CMD_RESULT_WORD_LEN + 4 * CMD_RESULT_WORD_LEN;

	const int self_gap_buff_size = (ts->tx_count - 1) + (ts->rx_count - 1);
	const int self_buff_size = self_gap_buff_size * CMD_RESULT_WORD_LEN + 4;

	const int gap_buff_size = mutual_buff_size + self_buff_size;

	const unsigned int X_DIR = 0;
	const unsigned int Y_DIR = 1;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);
	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));

	mode.type = TYPE_OFFSET_DATA_SDC;
	mode.frame_channel = TEST_MODE_READ_ALL;
	mode.allnode = TEST_MODE_ALL_NODE;
#ifdef USE_SPEC_CHECK
	mode.spec_check = SPEC_CHECK;
#endif

	raw_buff = kzalloc(raw_buff_size, GFP_KERNEL);
	gap_x = kzalloc(mutual_gap_buff_size, GFP_KERNEL);
	gap_y = kzalloc(mutual_gap_buff_size, GFP_KERNEL);
	gap = kzalloc(self_gap_buff_size, GFP_KERNEL);
	gap_buff = kzalloc(gap_buff_size, GFP_KERNEL);


	if (!raw_buff || !gap_x || !gap_y || !gap || !gap_buff) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		sec_cmd_set_cmd_result(sec, "FAIL", 4);
		goto ErrorAlloc;
	}

	ret = sec_ts_read_frame_and_channel(ts, mode.type, mode.min, mode.max,
				&mode.spec_check);

	if (ret < 0) {
		sec_cmd_set_cmd_result(sec, "FAIL", 4);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto ErrorAlloc;
	}
	ret_x = sec_ts_cm_spec_over_check(ts, gap_x, X_DIR);
	ret_y = sec_ts_cm_spec_over_check(ts, gap_y, Y_DIR);

	ret = sec_ts_cs_spec_over_check(ts, gap, ts->pFrameSS);

#ifdef USE_SPEC_CHECK
	if (mode.spec_check == SPEC_CHECK) {
		if (0 != (ret_x + ret_y + ret))
			mode.spec_check = SPEC_FAIL;
	}
#else
	mode.spec_check = SPEC_PASS;
#endif

	raw_buff_len = sec_ts_save_mutual_raw_to_buffer(ts, sec,
		&mode, raw_buff + raw_buff_len, raw_buff_size - raw_buff_len);
	raw_buff_len += sec_ts_save_self_raw_to_buffer(ts, sec,
		&mode, raw_buff + raw_buff_len, raw_buff_size - raw_buff_len);

#ifdef USE_SPEC_CHECK
		raw_buff_len += scnprintf(raw_buff + raw_buff_len,
					raw_buff_size - raw_buff_len,
					"%3d,%3d", mode->min[0], mode->max[0]);
#else
		raw_buff_len += scnprintf(raw_buff + raw_buff_len,
					raw_buff_size - raw_buff_len,
					"OK");
#endif

	sec_cmd_set_cmd_result(sec, raw_buff, raw_buff_len);

	gap_buff_len = sec_ts_save_mutual_gap_raw_to_buffer(ts,
		&mode, gap_x, gap_y, gap_buff + gap_buff_len,
		gap_buff_size - gap_buff_len);
	gap_buff_len += sec_ts_save_self_gap_raw_to_buffer(ts, &mode, gap,
		gap_buff + gap_buff_len, gap_buff_size - gap_buff_len);

	sec_cmd_set_cmd_result_2(sec, gap_buff, gap_buff_len);

ErrorAlloc:

	kfree(raw_buff);
	kfree(gap);
	kfree(gap_x);
	kfree(gap_y);
	kfree(gap_buff);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);

}

/* run_rawcap_gap_combo_read_all :
 * Combine Cm/Cs gap test to merge
 * run_rawcap_gap_read_all() and run_self_rawcap_gap_read_all()
 */
static void run_rawcap_gap_combo_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;

	int ret_x, ret_y, ret;

	unsigned int buff_len = 0;

	short *gap, *gap_x, *gap_y = NULL;
	char *buff = NULL;

	const unsigned int mutual_gap_buff_size =
		ts->tx_count * ts->rx_count * 2;
	const unsigned int mutual_buff_size = ts->tx_count * ts->rx_count * 2
		* CMD_RESULT_WORD_LEN + 4 * CMD_RESULT_WORD_LEN;

	const int self_gap_buff_size = (ts->tx_count - 1) + (ts->rx_count - 1);
	const int self_buff_size = self_gap_buff_size * CMD_RESULT_WORD_LEN + 4;

	const int buff_size = mutual_buff_size + self_buff_size;

	const unsigned int X_DIR = 0;
	const unsigned int Y_DIR = 1;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);
	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));

	mode.type = TYPE_OFFSET_DATA_SDC;
	mode.frame_channel = TEST_MODE_READ_ALL;
	mode.allnode = TEST_MODE_ALL_NODE;
#ifdef USE_SPEC_CHECK
	mode.spec_check = SPEC_CHECK;
#endif

	gap_x = kzalloc(mutual_gap_buff_size, GFP_KERNEL);
	gap_y = kzalloc(mutual_gap_buff_size, GFP_KERNEL);
	gap = kzalloc(self_gap_buff_size, GFP_KERNEL);
	buff = kzalloc(buff_size, GFP_KERNEL);


	if (!gap_x || !gap_y || !gap || !buff) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		sec_cmd_set_cmd_result(sec, "FAIL", 4);
		goto ErrorAlloc;
	}

	ret = sec_ts_read_frame_and_channel(ts, mode.type, mode.min, mode.max,
				&mode.spec_check);

	if (ret < 0) {
		sec_cmd_set_cmd_result(sec, "FAIL", 4);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto ErrorAlloc;
	}

	ret_x = sec_ts_cm_spec_over_check(ts, gap_x, X_DIR);
	ret_y = sec_ts_cm_spec_over_check(ts, gap_y, Y_DIR);

	ret = sec_ts_cs_spec_over_check(ts, gap, ts->pFrameSS);


#ifdef USE_SPEC_CHECK
	if (mode.spec_check == SPEC_CHECK) {
		if (0 == (ret_x + ret_y + ret))
			mode.spec_check = SPEC_PASS;
		else
			mode.spec_check = SPEC_FAIL;
	}
#else
	mode.spec_check = SPEC_PASS;
#endif

	buff_len = sec_ts_save_mutual_gap_raw_to_buffer(ts, &mode, gap_x, gap_y,
		buff + buff_len, buff_size - buff_len);
	buff_len += sec_ts_save_self_gap_raw_to_buffer(ts, &mode, gap,
		buff + buff_len, buff_size - buff_len);

	sec_cmd_set_cmd_result(sec, buff, buff_len);

ErrorAlloc:

	kfree(gap);
	kfree(gap_x);
	kfree(gap_y);
	kfree(buff);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);

}

static void get_rawcap(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	short val = 0;
	int node = 0;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: [ERROR] Touch is stopped\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	node = sec_ts_check_index(ts);
	if (node < 0) {
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	val = ts->pFrame[node];
	snprintf(buff, sizeof(buff), "%d", val);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_rawcap_gap_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;
	int ret_x, ret_y;
	short *gap_x, *gap_y;
	char *buff;
	const unsigned int buff_size = ts->tx_count * ts->rx_count * 2
		* CMD_RESULT_WORD_LEN + 4 * CMD_RESULT_WORD_LEN;
	const unsigned int readbytes = ts->tx_count * ts->rx_count * 2;
	const unsigned int X_DIR = 0;
	const unsigned int Y_DIR = 1;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));

	gap_x = kzalloc(readbytes, GFP_KERNEL);
	gap_y = kzalloc(readbytes, GFP_KERNEL);
	buff = kzalloc(buff_size, GFP_KERNEL);

	if (!gap_x || !gap_y || !buff) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		sec_cmd_set_cmd_result(sec, "FAIL", 4);
		goto ErrorAlloc;
	}

	mode.type = TYPE_OFFSET_DATA_SDC;
	mode.allnode = TEST_MODE_ALL_NODE;
#ifdef USE_SPEC_CHECK
	mode.spec_check = SPEC_CHECK;
#endif

	sec_ts_read_frame(ts, mode.type, mode.min, mode.max,
			  &mode.spec_check);

	ret_x = sec_ts_cm_spec_over_check(ts, gap_x, X_DIR);

	ret_y = sec_ts_cm_spec_over_check(ts, gap_y, Y_DIR);

#ifdef USE_SPEC_CHECK
	if (0 == (ret_x + ret_y)) {
		mode.spec_check = SPEC_PASS;
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		mode.spec_check = SPEC_FAIL;
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}
#else
	mode.spec_check = SPEC_PASS;
#endif

	sec_ts_save_mutual_gap_raw_to_buffer(ts, &mode, gap_x, gap_y,
		buff, buff_size);

	sec_cmd_set_cmd_result(sec, buff, buff_size);

ErrorAlloc:
	kfree(buff);
	kfree(gap_y);
	kfree(gap_x);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_rawcap_high_freq_read_all(void *device_data)
{
	struct sec_cmd_data *sec =
		(struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts =
		container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_OFFSET_DATA_SDC_CM2;
	mode.allnode = TEST_MODE_ALL_NODE;

	sec_ts_read_raw_data(ts, sec, &mode);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_delta_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_SIGNAL_DATA;

	sec_ts_read_raw_data(ts, sec, &mode);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_delta_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_SIGNAL_DATA;
	mode.allnode = TEST_MODE_ALL_NODE;

	sec_ts_read_raw_data(ts, sec, &mode);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void get_delta(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	short val = 0;
	int node = 0;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: [ERROR] Touch is stopped\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;

		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	node = sec_ts_check_index(ts);
	if (node < 0) {
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	val = ts->pFrame[node];
	snprintf(buff, sizeof(buff), "%d", val);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static int sec_ts_read_frame_stdev(struct sec_ts_data *ts,
	struct sec_cmd_data *sec, u8 type, short *min, short *max,
	enum spec_check_type *spec_check, bool get_average_only)
{
	unsigned char *pRead = NULL;
	short *pFrameAll = NULL;
	int *pFrameAvg = NULL;
	u64 *pFrameStd = NULL;
	u8 inval_type = TYPE_INVALID_DATA;
	int node_tot = 0;
	int ret = 0;
	int i = 0;
	int j = 0;
	int frame_len_byte = 0;
	int frame_cnt = 0;
	int frame_tot = 0;
	int tmp = 0;

	const unsigned int buff_size = ts->tx_count * ts->rx_count *
					CMD_RESULT_WORD_LEN;
	unsigned int buff_len = 0;
	char *pBuff;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	frame_tot = 100;

	/* set data length, allocation buffer memory */

	ret = -ENOMEM;
	pBuff = kzalloc(buff_size, GFP_KERNEL);
	if (!pBuff)
		goto ErrorAlloc;

	/* each node data 2bytes : 1frame bytes = node_tot * 2 */
	node_tot = ts->rx_count * ts->tx_count;
	frame_len_byte = node_tot * 2;

	pRead = kzalloc(frame_len_byte, GFP_KERNEL);
	if (!pRead)
		goto ErrorAlloc;

	/* memory whole frame data : 1frame bytes * total frame */
	pFrameAll = kzalloc(frame_len_byte * frame_tot, GFP_KERNEL);
	if (!pFrameAll)
		goto ErrorAlloc;

	/* float type : type size is double */
	pFrameAvg = kzalloc(frame_len_byte * 2, GFP_KERNEL);
	if (!pFrameAvg)
		goto ErrorAlloc;

	/* 64-bit to prevent overflow */
	pFrameStd = kzalloc(frame_len_byte * 4, GFP_KERNEL);
	if (!pFrameStd)
		goto ErrorAlloc;

	/* fix touch mode */
	ret = sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH,
			       TOUCH_MODE_STATE_TOUCH);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: failed to fix tmode\n",
				__func__);
		goto ErrorAlloc;
	}

	/* set OPCODE and data type */
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_MUTU_RAW_TYPE, &type, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: Set rawdata type failed\n", __func__);
		goto ErrorDataType;
	}

	sec_ts_delay(50);

	for (frame_cnt = 0; frame_cnt < frame_tot; frame_cnt++) {
		/* read data */
		ret = ts->sec_ts_read_heap(ts, SEC_TS_READ_TOUCH_RAWDATA,
					pRead, frame_len_byte);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				  "%s: read rawdata failed!\n", __func__);
			goto ErrorRelease;
		}

		memset(ts->pFrame, 0x00, frame_len_byte);

		for (i = 0; i < frame_len_byte; i += 2) {
			ts->pFrame[i / 2] = pRead[i + 1] + (pRead[i] << 8);
			pFrameAvg[i / 2] += ts->pFrame[i / 2];
		}

		memcpy(pFrameAll + (frame_len_byte * frame_cnt) / sizeof(short),
		       ts->pFrame, frame_len_byte);
	}

	/* get total frame average of each node */
	/* in the case of getting only average, *1000 not needed */
	for (j = 0; j < node_tot; j++) {
		if (!get_average_only)
			pFrameAvg[j] = pFrameAvg[j] * 1000;
		pFrameAvg[j] = pFrameAvg[j] / frame_tot;
	}

	input_info(true, &ts->client->dev, "%s: FrameAvg x 1000\n", __func__);

	/* print frame average x 1000 of each node */
	for (i = 0; i < ts->rx_count; i++) {
		buff_len = scnprintf(pBuff, buff_size, "Rx%02d | ", i);

		for (j = 0; j < ts->tx_count; j++) {
			buff_len += scnprintf(pBuff + buff_len,
					buff_size - buff_len,
					" %6d",
					pFrameAvg[(j * ts->rx_count) + i]);
		}
		input_info(true, &ts->client->dev, "%s\n", pBuff);
	}

	/* when only getting average, put average in
	 * ts->pFrame and goto set cmd_result
	 */
	if (get_average_only) {
		for (i = 0; i < ts->tx_count; i++) {
			for (j = 0; j < ts->rx_count; j++) {
				ts->pFrame[(j * ts->tx_count) + i] =
					(short)(pFrameAvg[(i * ts->rx_count) +
							  j]);
			}
		}
		goto OnlyAverage;
	}

	/* get standard deviation */
	for (i = 0; i < frame_tot; i++) {
		for (j = 0; j < node_tot; j++) {
			tmp = pFrameAll[node_tot * i + j] * 1000;
			pFrameStd[j] = pFrameStd[j] +
			    (tmp - pFrameAvg[j]) * (tmp - pFrameAvg[j]);
		}
	}

	for (j = 0; j < node_tot; j++)
		pFrameStd[j] = int_sqrt(pFrameStd[j] / frame_tot);

	/* print standard deviation x 1000 of each node */
	input_info(true, &ts->client->dev, "%s: FrameStd x 1000\n", __func__);

	*min = *max = pFrameStd[0];

	for (i = 0; i < ts->rx_count; i++) {
		buff_len = scnprintf(pBuff, buff_size, "Rx%02d | ", i);

		for (j = 0; j < ts->tx_count; j++) {
			if (i > 0) {
				if (pFrameStd[(j * ts->rx_count) + i] < *min)
					*min =
					    pFrameStd[(j * ts->rx_count) + i];

				if (pFrameStd[(j * ts->rx_count) + i] > *max)
					*max =
					    pFrameStd[(j * ts->rx_count) + i];
			}
			buff_len += scnprintf(pBuff + buff_len,
					buff_size - buff_len,
					" %6d",
					(int)pFrameStd[(j * ts->rx_count) + i]);
		}
		input_info(true, &ts->client->dev, "%s\n", pBuff);
	}
	// SQRT(VAR)

	/* Rotate 90 degrees for readability */
	for (i = 0; i < ts->tx_count; i++) {
		for (j = 0; j < ts->rx_count; j++) {
			if (pFrameStd[(i * ts->rx_count) + j] > 32767)
				/* Reduce to short data type and allow high
				 * values to saturate.
				 */
				ts->pFrame[(j * ts->tx_count) + i] = 32767;
			else {
				ts->pFrame[(j * ts->tx_count) + i] =
					(short)(pFrameStd[(i * ts->rx_count) +
						j]);
			}
		}
	}

#ifdef USE_SPEC_CHECK
	if (*spec_check == SPEC_CHECK) {
		int specover_count = 0;

		for (i = 0; i < ts->tx_count; i++) {
			for (j = 0; j < ts->rx_count; j++) {
				if (ts->pFrame[(j * ts->tx_count) + i] >
				    cm_stdev_max)
					specover_count++;
			}
		}

		if (specover_count == 0)
			*spec_check = SPEC_PASS;
		else
			*spec_check = SPEC_FAIL;
	}
#else
	*spec_check = SPEC_PASS;
#endif

	if (*spec_check == SPEC_PASS)
		buff_len = scnprintf(pBuff, buff_size, "OK %d %d\n",
				ts->rx_count, ts->tx_count);
	else if (*spec_check == SPEC_FAIL)
		buff_len = scnprintf(pBuff, buff_size, "NG %d %d\n",
				ts->rx_count, ts->tx_count);
	else
		buff_len = scnprintf(pBuff, buff_size, "\n");

	for (i = 0; i < node_tot; i++) {
		buff_len += scnprintf(pBuff + buff_len, buff_size - buff_len,
				      "%4d,", ts->pFrame[i]);

		if (i % ts->tx_count == ts->tx_count - 1)
			buff_len += scnprintf(pBuff + buff_len,
					      buff_size - buff_len, "\n");
	}

	if (!sec)
		goto ErrorRelease;

	sec_cmd_set_cmd_result(sec, pBuff, buff_len);
	sec->cmd_state = SEC_CMD_STATUS_OK;

ErrorRelease:
OnlyAverage:
	/* release data monitory (unprepare AFE data memory) */
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_MUTU_RAW_TYPE, &inval_type,
				   1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: Set rawdata type failed\n", __func__);
		goto ErrorAlloc;
	}

ErrorDataType:
	/* release mode fix */
	ret = sec_ts_release_tmode(ts);
	if (ret < 0) {
		input_err(true,
			&ts->client->dev, "%s: failed to release tmode\n",
			__func__);
	}

ErrorAlloc:
	kfree(pFrameStd);
	kfree(pFrameAvg);
	kfree(pFrameAll);
	kfree(pRead);
	kfree(pBuff);

	return ret;
}

static void run_rawdata_stdev_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_REMV_AMB_DATA;
#ifdef USE_SPEC_CHECK
	mode.spec_check = SPEC_CHECK;
#endif

	sec_ts_read_frame_stdev(ts, sec, mode.type, mode.min, mode.max,
				&mode.spec_check, false);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static int sec_ts_read_frame_p2p(struct sec_ts_data *ts,
		struct sec_cmd_data *sec, struct sec_ts_test_mode mode)
{
	const unsigned int frame_size = ts->rx_count * ts->tx_count * 2;
	short *temp = NULL;
	unsigned short readbytes;
	int i;
	int ret = -1;
	char para = TO_TOUCH_MODE;
	const unsigned int buff_size = ts->tx_count * ts->rx_count *
					CMD_RESULT_WORD_LEN;
	unsigned int buff_len = 0;
	char *buff;
	u8 result = 0x0;

	buff = kzalloc(buff_size, GFP_KERNEL);
	if (!buff)
		goto ErrorAllocbuff;

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: [ERROR] Touch is stopped\n", __func__);
		goto ErrorPowerState;
	}

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));

	disable_irq(ts->client->irq);

	ret = execute_p2ptest(ts);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: P2P test failed\n",
			  __func__);
		goto ErrorP2PTest;
	}

	/* get min data */
	mode.type = TYPE_NOI_P2P_MIN;
#ifdef USE_SPEC_CHECK
	mode.spec_check = SPEC_CHECK;
#endif
	sec_ts_read_frame(ts, mode.type, mode.min, mode.max,
			  &mode.spec_check);
	if (mode.spec_check == SPEC_FAIL)
		result |= 0x1;

	readbytes = ts->rx_count * ts->tx_count;

	/* 2 bytes for each node data */
	temp = kzalloc(frame_size, GFP_KERNEL);
	if (!temp)
		goto ErrorAlloctemp;

	memcpy(temp, ts->pFrame, frame_size);
	memset(ts->pFrame, 0x00, frame_size);

	/* get max data */
	mode.type = TYPE_NOI_P2P_MAX;
#ifdef USE_SPEC_CHECK
	mode.spec_check = SPEC_CHECK;
#endif
	sec_ts_read_frame(ts, mode.type, mode.min, mode.max,
			  &mode.spec_check);
	if (mode.spec_check == SPEC_FAIL)
		result |= 0x2;

	for (i = 0; i < readbytes; i++) {
		/* get p2p by subtract min from max data */
		ts->pFrame[i] = ts->pFrame[i] - temp[i];
#ifdef USE_SPEC_CHECK
		if (ts->pFrame[i] > noi_mm)
			result |= 0x4;
#endif
	}

	if (result != 0x0)
		buff_len += scnprintf(buff + buff_len, buff_size - buff_len,
				      "NG\n");
	else
		buff_len += scnprintf(buff + buff_len, buff_size - buff_len,
				      "OK\n");

	for (i = 0; i < readbytes; i++) {
		buff_len += scnprintf(buff + buff_len, buff_size - buff_len,
				      "%3d,", ts->pFrame[i]);
		if (i % ts->tx_count == (ts->tx_count - 1))
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len, "\n");
	}

	if (sec) {
		sec_cmd_set_cmd_result(sec, buff, buff_len);
		sec->cmd_state = SEC_CMD_STATUS_OK;
	}

	kfree(temp);
ErrorAlloctemp:
ErrorP2PTest:
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
	if (ret < 0)
		input_err(true, &ts->client->dev, "%s: Set powermode failed\n",
			  __func__);

	enable_irq(ts->client->irq);
ErrorPowerState:
	kfree(buff);
ErrorAllocbuff:

	if (sec && ret < 0) {
		sec_cmd_set_cmd_result(sec, "NG", 3);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	return ret;
}

static void run_rawdata_p2p_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));

	sec_ts_read_frame_p2p(ts, sec, mode);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

/* self reference : send TX power in TX channel, receive in TX channel */
static void run_self_reference_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_OFFSET_DATA_SEC;
	mode.frame_channel = TEST_MODE_READ_CHANNEL;

	sec_ts_read_raw_data(ts, sec, &mode);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_self_reference_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_OFFSET_DATA_SEC;
	mode.frame_channel = TEST_MODE_READ_CHANNEL;
	mode.allnode = TEST_MODE_ALL_NODE;

	sec_ts_read_raw_data(ts, sec, &mode);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_self_rawcap_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_OFFSET_DATA_SDC;
	mode.frame_channel = TEST_MODE_READ_CHANNEL;
#ifdef USE_SPEC_CHECK
	mode.spec_check = SPEC_CHECK;
#endif

	sec_ts_read_raw_data(ts, sec, &mode);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_self_rawcap_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_OFFSET_DATA_SDC;
	mode.frame_channel = TEST_MODE_READ_CHANNEL;
	mode.allnode = TEST_MODE_ALL_NODE;
#ifdef USE_SPEC_CHECK
	mode.spec_check = SPEC_CHECK;
#endif

	sec_ts_read_raw_data(ts, sec, &mode);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_self_rawcap_gap_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;
	int ret = 0;
	char *buff = NULL;
	short *gap = NULL;
	const int gap_buff_size = (ts->tx_count - 1) + (ts->rx_count - 1);
	const int buff_size = gap_buff_size * CMD_RESULT_WORD_LEN + 4;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_OFFSET_DATA_SDC;
	mode.frame_channel = TEST_MODE_READ_CHANNEL;
	mode.allnode = TEST_MODE_ALL_NODE;
#ifdef USE_SPEC_CHECK
	mode.spec_check = SPEC_CHECK;
#endif

	gap = kzalloc(gap_buff_size, GFP_KERNEL);
	buff = kzalloc(buff_size, GFP_KERNEL);
	if (!gap || !buff) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		sec_cmd_set_cmd_result(sec, "FAIL", 4);
		goto ErrorAlloc;
	}

	sec_ts_read_channel(ts, mode.type, mode.min, mode.max,
			    &mode.spec_check);

	/* ret is number of spec over channel */
	ret = sec_ts_cs_spec_over_check(ts, gap, ts->pFrame);

#ifdef USE_SPEC_CHECK
	if (ret == 0) {
		mode.spec_check = SPEC_PASS;
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		mode.spec_check = SPEC_FAIL;
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}
#else
	mode.spec_check = SPEC_PASS;
#endif

	sec_ts_save_self_gap_raw_to_buffer(ts, &mode, gap,
		buff, buff_size);

	sec_cmd_set_cmd_result(sec, buff, buff_size);

ErrorAlloc:
	kfree(buff);
	kfree(gap);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_self_delta_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_SIGNAL_DATA;
	mode.frame_channel = TEST_MODE_READ_CHANNEL;

	sec_ts_read_raw_data(ts, sec, &mode);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_self_delta_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));
	mode.type = TYPE_SIGNAL_DATA;
	mode.frame_channel = TEST_MODE_READ_CHANNEL;
	mode.allnode = TEST_MODE_ALL_NODE;

	sec_ts_read_raw_data(ts, sec, &mode);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

/* Use TSP NV area
 * buff[0] : offset from user NVM storage
 * buff[1] : length of stroed data - 1 (ex. using 1byte, value is  1 - 1 = 0)
 * buff[2] : write data
 * buff[..] : cont.
 */
void set_tsp_nvm_data_clear(struct sec_ts_data *ts, u8 offset)
{
	char buff[4] = { 0 };
	int ret;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	buff[0] = offset;

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_NVM, buff, 3);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: nvm write failed. ret: %d\n", __func__, ret);

	sec_ts_delay(20);
}

int get_tsp_nvm_data(struct sec_ts_data *ts, u8 offset)
{
	char buff[2] = { 0 };
	int ret;

	/* SENSE OFF -> CELAR EVENT STACK -> READ NV -> SENSE ON */
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SENSE_OFF, NULL, 0);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: fail to write Sense_off\n", __func__);
		goto out_nvm;
	}

	input_dbg(true, &ts->client->dev, "%s: SENSE OFF\n", __func__);

	sec_ts_delay(100);

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_CLEAR_EVENT_STACK, NULL, 0);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: write clear event failed\n", __func__);
		goto out_nvm;
	}

	input_dbg(true, &ts->client->dev, "%s: CLEAR EVENT STACK\n", __func__);

	sec_ts_delay(100);

	sec_ts_locked_release_all_finger(ts);

	/* send NV data using command
	 * Use TSP NV area : in this model, use only one byte
	 * buff[0] : offset from user NVM storage
	 * buff[1] : length of stroed data - 1 (ex. using 1byte,
	 *           value is  1 - 1 = 0)
	 **/
	memset(buff, 0x00, 2);
	buff[0] = offset;
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_NVM, buff, 2);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			"%s: nvm send command failed. ret: %d\n",
			__func__, ret);
		goto out_nvm;
	}

	sec_ts_delay(20);

	/* read NV data
	 * Use TSP NV area : in this model, use only one byte
	 */
#ifdef I2C_INTERFACE
	ret = ts->sec_ts_read_bulk(ts, buff, 1);
#else
	ret = ts->sec_ts_read(ts, SEC_TS_CMD_NVM, buff, 1);
#endif
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			"%s: nvm send command failed. ret: %d\n",
			__func__, ret);
		goto out_nvm;
	}

	input_info(true, &ts->client->dev,
		   "%s: offset: %u data: %02X\n", __func__, offset, buff[0]);

out_nvm:
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: fail to write Sense_on\n", __func__);

	input_dbg(true, &ts->client->dev, "%s: SENSE ON\n", __func__);

	return buff[0];
}

int get_tsp_nvm_data_by_size(struct sec_ts_data *ts, u8 offset,
			     int length, u8 *data)
{
	char *buff = NULL;
	int ret;

	buff = kzalloc(length, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	input_info(true, &ts->client->dev,
		   "%s: offset: %u, length: %d, size: %d\n",
		   __func__, offset, length, sizeof(data));

	/* SENSE OFF -> CELAR EVENT STACK -> READ NV -> SENSE ON */
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SENSE_OFF, NULL, 0);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: fail to write Sense_off\n", __func__);
		goto out_nvm;
	}

	input_dbg(true, &ts->client->dev, "%s: SENSE OFF\n", __func__);

	sec_ts_delay(100);

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_CLEAR_EVENT_STACK, NULL, 0);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: write clear event failed\n", __func__);
		goto out_nvm;
	}

	input_dbg(true, &ts->client->dev, "%s: CLEAR EVENT STACK\n", __func__);

	sec_ts_delay(100);

	sec_ts_locked_release_all_finger(ts);

	/* send NV data using command
	 * Use TSP NV area : in this model, use only one byte
	 * buff[0] : offset from user NVM storage
	 * buff[1] : length of stroed data - 1 (ex. using 1byte,
	 *           value is  1 - 1 = 0)
	 **/
	memset(buff, 0x00, 2);
	buff[0] = offset;
	buff[1] = length - 1;
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_NVM, buff, 2);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			"%s: nvm send command failed. ret: %d\n",
			__func__, ret);
		goto out_nvm;
	}

	sec_ts_delay(20);

	/* read NV data
	 * Use TSP NV area : in this model, use only one byte
	 */
#ifdef I2C_INTERFACE
	ret = ts->sec_ts_read_bulk_heap(ts, buff, length);
#else
	ret = ts->sec_ts_read(ts, SEC_TS_CMD_NVM, buff, length);
#endif
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			"%s: nvm send command failed. ret: %d\n",
			__func__, ret);
		goto out_nvm;
	}

	memcpy(data, buff, length);

out_nvm:
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: fail to write Sense_on\n", __func__);

	input_dbg(true, &ts->client->dev, "%s: SENSE ON\n", __func__);

	kfree(buff);

	return ret;
}

#ifdef PAT_CONTROL
void set_pat_magic_number(struct sec_ts_data *ts)
{
	char buff[4] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	input_info(true, &ts->client->dev, "%s\n", __func__);
	buff[0] = SEC_TS_NVM_OFFSET_CAL_COUNT;
	buff[1] = 0;
	buff[2] = PAT_MAGIC_NUMBER;

	input_info(true, &ts->client->dev, "%s: %02X\n", __func__, buff[2]);

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_NVM, buff, 3);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			"%s: nvm write failed. ret: %d\n", __func__, ret);
	}
	sec_ts_delay(20);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}
#endif


/* FACTORY TEST RESULT SAVING FUNCTION
 * bit 3 ~ 0 : OCTA Assy
 * bit 7 ~ 4 : OCTA module
 * param[0] : OCTA modue(1) / OCTA Assy(2)
 * param[1] : TEST NONE(0) / TEST FAIL(1) / TEST PASS(2) : 2 bit
 */

#define TEST_OCTA_MODULE	1
#define TEST_OCTA_ASSAY		2

#define TEST_OCTA_NONE		0
#define TEST_OCTA_FAIL		1
#define TEST_OCTA_PASS		2

static void set_tsp_test_result(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_result *result;
	char buff[SEC_CMD_STR_LEN] = { 0 };
	char r_data[1] = { 0 };
	int ret = 0;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: [ERROR] Touch is stopped\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP_truned off");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	r_data[0] = get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_FAC_RESULT);
	if (r_data[0] == 0xFF)
		r_data[0] = 0;

	result = (struct sec_ts_test_result *)r_data;

	if (sec->cmd_param[0] == TEST_OCTA_ASSAY) {
		result->assy_result = sec->cmd_param[1];
		if (result->assy_count < 3)
			result->assy_count++;
	}

	if (sec->cmd_param[0] == TEST_OCTA_MODULE) {
		result->module_result = sec->cmd_param[1];
		if (result->module_count < 3)
			result->module_count++;
	}

	input_info(true, &ts->client->dev,
		"%s: %d, %d, %d, %d, 0x%X\n", __func__,
		result->module_result, result->module_count,
		result->assy_result, result->assy_count, result->data[0]);

	/* Use TSP NV area : in this model, use only one byte
	 * buff[0] : offset from user NVM storage
	 * buff[1] : length of stroed data - 1 (ex. using 1byte,
	 *           value is  1 - 1 = 0)
	 * buff[2] : write data
	 **/
	memset(buff, 0x00, SEC_CMD_STR_LEN);
	buff[2] = *result->data;

	input_info(true, &ts->client->dev, "%s: command (1)%X, (2)%X: %X\n",
		__func__, sec->cmd_param[0], sec->cmd_param[1], buff[2]);

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_NVM, buff, 3);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: nvm write failed. ret: %d\n", __func__, ret);

	sec_ts_delay(20);

	ts->nv = get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_FAC_RESULT);

	snprintf(buff, sizeof(buff), "OK");
	sec_cmd_set_cmd_result(sec, buff, strlen(buff));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void get_tsp_test_result(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	struct sec_ts_test_result *result;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: [ERROR] Touch is stopped\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP_truned off");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;

		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	memset(buff, 0x00, SEC_CMD_STR_LEN);
	buff[0] = get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_FAC_RESULT);
	if (buff[0] == 0xFF) {
		set_tsp_nvm_data_clear(ts, SEC_TS_NVM_OFFSET_FAC_RESULT);
		buff[0] = 0;
	}

	ts->nv = buff[0];

	result = (struct sec_ts_test_result *)buff;

	input_info(true, &ts->client->dev,
			 "%s: [0x%X][0x%X] M: %d, M: %d, A: %d, A: %d\n",
			__func__, *result->data, buff[0],
			result->module_result, result->module_count,
			result->assy_result, result->assy_count);

	snprintf(buff, sizeof(buff), "M: %s, M: %d, A: %s, A: %d",
			result->module_result == 0 ? "NONE" :
			result->module_result == 1 ? "FAIL" : "PASS",
			result->module_count,
			result->assy_result == 0 ? "NONE" :
			result->assy_result == 1 ? "FAIL" : "PASS",
			result->assy_count);

	sec_cmd_set_cmd_result(sec, buff, strlen(buff));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void increase_disassemble_count(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[3] = { 0 };
	int ret = 0;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: [ERROR] Touch is stopped\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP_truned off");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;

		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	buff[2] = get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_DISASSEMBLE_COUNT);

	input_info(true, &ts->client->dev,
		   "%s: disassemble count is #1 %d\n", __func__, buff[2]);

	if (buff[2] == 0xFF)
		buff[2] = 0;

	if (buff[2] < 0xFE)
		buff[2]++;

	/* Use TSP NV area : in this model, use only one byte
	 * buff[0] : offset from user NVM storage
	 * buff[1] : length of stroed data - 1 (ex. using 1byte,
	 *           value is  1 - 1 = 0)
	 * buff[2] : write data
	 **/
	buff[0] = SEC_TS_NVM_OFFSET_DISASSEMBLE_COUNT;
	buff[1] = 0;

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_NVM, buff, 3);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			  "%s: nvm write failed. ret: %d\n", __func__, ret);

	sec_ts_delay(20);

	memset(buff, 0x00, 3);
	buff[0] = get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_DISASSEMBLE_COUNT);
	input_info(true, &ts->client->dev,
		   "%s: check disassemble count: %d\n", __func__, buff[0]);

	snprintf(buff, sizeof(buff), "OK");
	sec_cmd_set_cmd_result(sec, buff, strlen(buff));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void get_disassemble_count(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };

	input_info(true, &ts->client->dev, "%s\n", __func__);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: [ERROR] Touch is stopped\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP_truned off");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;

		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	memset(buff, 0x00, SEC_CMD_STR_LEN);
	buff[0] = get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_DISASSEMBLE_COUNT);
	if (buff[0] == 0xFF) {
		set_tsp_nvm_data_clear(ts, SEC_TS_NVM_OFFSET_DISASSEMBLE_COUNT);
		buff[0] = 0;
	}

	input_info(true, &ts->client->dev,
		   "%s: read disassemble count: %d\n", __func__, buff[0]);

	snprintf(buff, sizeof(buff), "%d", buff[0]);

	sec_cmd_set_cmd_result(sec, buff, strlen(buff));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

#define GLOVE_MODE_EN		(1 << 0)
#define CLEAR_COVER_EN		(1 << 1)
#define FAST_GLOVE_MODE_EN	(1 << 2)

static void glove_mode(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	int glove_mode_enables = 0;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1) {
		snprintf(buff, sizeof(buff), "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	} else {
		int retval;

		if (sec->cmd_param[0])
			glove_mode_enables |= GLOVE_MODE_EN;
		else
			glove_mode_enables &= ~(GLOVE_MODE_EN);

		retval = sec_ts_glove_mode_enables(ts, glove_mode_enables);

		if (retval < 0) {
			input_err(true, &ts->client->dev,
				"%s: failed, retval = %d\n", __func__, retval);
			snprintf(buff, sizeof(buff), "NG");
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
		} else {
			snprintf(buff, sizeof(buff), "OK");
			sec->cmd_state = SEC_CMD_STATUS_OK;
		}
	}

	sec_cmd_set_cmd_result(sec, buff, strlen(buff));
	sec->cmd_state = SEC_CMD_STATUS_WAITING;
	sec_cmd_set_cmd_exit(sec);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void clear_cover_mode(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };

	input_info(true, &ts->client->dev,
		   "%s...\n", __func__, buff);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 3) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	} else {
		if (sec->cmd_param[0] > 1) {
			ts->flip_enable = true;
			ts->cover_type = sec->cmd_param[1];
			ts->cover_cmd = (u8)ts->cover_type;
		} else {
			ts->flip_enable = false;
		}

		if (!(ts->power_status == SEC_TS_STATE_POWER_OFF) &&
		    ts->reinit_done) {
			if (ts->flip_enable)
				sec_ts_set_cover_type(ts, true);
			else
				sec_ts_set_cover_type(ts, false);
		}

		snprintf(buff, sizeof(buff), "%s", "OK");
		sec->cmd_state = SEC_CMD_STATUS_OK;
	}
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_WAITING;
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
};

static void dead_zone_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	int ret;
	char data = 0;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	} else {
		data = sec->cmd_param[0];

		ret = ts->sec_ts_write(ts, SEC_TS_CMD_EDGE_DEADZONE, &data, 1);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				"%s: failed to set deadzone\n", __func__);
			snprintf(buff, sizeof(buff), "%s", "NG");
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			goto err_set_dead_zone;
		}

		snprintf(buff, sizeof(buff), "%s", "OK");
		sec->cmd_state = SEC_CMD_STATUS_OK;
	}

err_set_dead_zone:
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
};


static void drawing_test_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	} else {
		if (ts->use_customlib) {
			if (sec->cmd_param[0])
				ts->lowpower_mode &=
					~SEC_TS_MODE_CUSTOMLIB_FORCE_KEY;
			else
				ts->lowpower_mode |=
					SEC_TS_MODE_CUSTOMLIB_FORCE_KEY;

			#ifdef SEC_TS_SUPPORT_CUSTOMLIB
			ret = sec_ts_set_custom_library(ts);
			if (ret < 0) {
				snprintf(buff, sizeof(buff), "%s", "NG");
				sec->cmd_state = SEC_CMD_STATUS_FAIL;
			} else {
				snprintf(buff, sizeof(buff), "%s", "OK");
				sec->cmd_state = SEC_CMD_STATUS_OK;
			}
			#endif

		} else {
			snprintf(buff, sizeof(buff), "%s", "NA");
			sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
		}
	}

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
};

static void sec_ts_swap(u8 *a, u8 *b)
{
	u8 temp = *a;

	*a = *b;
	*b = temp;
}

static void rearrange_sft_result(u8 *data, int length)
{
	int i;

	for (i = 0; i < length; i += 4) {
		sec_ts_swap(&data[i], &data[i + 3]);
		sec_ts_swap(&data[i + 1], &data[i + 2]);
	}
}

int execute_p2ptest(struct sec_ts_data *ts)
{
	int rc;
	u8 tpara[2] = {0x0F, 0x11};

	input_info(true, &ts->client->dev, "%s: P2P test start!\n", __func__);
	rc = ts->sec_ts_write(ts, SEC_TS_CMD_SET_P2PTEST_MODE, tpara, 2);
	if (rc < 0) {
		input_err(true, &ts->client->dev,
			  "%s: Send P2Ptest Mode cmd failed!\n", __func__);
		goto err_exit;
	}

	sec_ts_delay(15);

	tpara[0] = 0x00;
	tpara[1] = 0x64;
	rc = ts->sec_ts_write(ts, SEC_TS_CMD_P2PTEST, tpara, 2);
	if (rc < 0) {
		input_err(true, &ts->client->dev,
			  "%s: Send P2Ptest cmd failed!\n", __func__);
		goto err_exit;
	}

	sec_ts_delay(1500);

	rc = sec_ts_wait_for_ready(ts, SEC_TS_VENDOR_ACK_P2P_TEST_DONE);
	if (rc < 0) {
		input_err(true, &ts->client->dev,
			  "%s: P2Ptest execution time out!\n", __func__);
		goto err_exit;
	}

	input_info(true, &ts->client->dev, "%s: P2P test done!\n", __func__);

err_exit:
	return rc;
}

/* execute_selftest options
 * bit[8] : Enable/disable the panel crack test
 * bit[7] : Do NOT save
 * bit[6] : Load self-test configuration only
 * bit[5] : Get Self capacitance
 * bit[4] : Reserved
 * bit[3] : Reserved
 * bit[2] : Enable/disable the short test
 * bit[1] : Enable/disable the node variance test
 * bit[0] : Enable/disable the open test
 */
int execute_selftest(struct sec_ts_data *ts, u32 option)
{
	int rc;
	/* Selftest setting
	 * Get self capacitance
	 * Enable/disable the short test
	 * Enable/disable the node variance test
	 * Enable/disable the open test
	 */
	u8 tpara[2] = {(u8)(option & 0xff), (u8)((option & 0xff00) >> 8)};
	u8 *rBuff;
	int i;
	int result_size = SEC_TS_SELFTEST_REPORT_SIZE +
				ts->tx_count * ts->rx_count * 2;

	rBuff = kzalloc(result_size, GFP_KERNEL);
	if (!rBuff)
		return -ENOMEM;

	input_info(true, &ts->client->dev, "%s: Self test start!\n", __func__);
	rc = ts->sec_ts_write(ts, SEC_TS_CMD_SELFTEST, tpara, 2);
	if (rc < 0) {
		input_err(true, &ts->client->dev,
			  "%s: Send selftest cmd failed!\n", __func__);
		goto err_exit;
	}

	sec_ts_delay(1500);

	rc = sec_ts_wait_for_ready(ts, SEC_TS_VENDOR_ACK_SELF_TEST_DONE);
	if (rc < 0) {
		input_err(true, &ts->client->dev,
			  "%s: Selftest execution time out!\n", __func__);
		goto err_exit;
	}

	input_info(true, &ts->client->dev, "%s: Self test done!\n", __func__);

	rc = ts->sec_ts_read_heap(ts, SEC_TS_READ_SELFTEST_RESULT, rBuff,
				result_size);
	if (rc < 0) {
		input_err(true, &ts->client->dev,
			  "%s: Selftest execution time out!\n", __func__);
		goto err_exit;
	}
	rearrange_sft_result(rBuff, result_size);

	for (i = 0; i < 80; i += 4) {
		if (i % 8 == 0) pr_cont("\n");
		if (i % 4 == 0) pr_cont("%s sec_ts : ", SECLOG);

		if (i / 4 == 0) pr_cont("SIG");
		else if (i / 4 == 1) pr_cont("VER");
		else if (i / 4 == 2) pr_cont("SIZ");
		else if (i / 4 == 3) pr_cont("CRC");
		else if (i / 4 == 4) pr_cont("RES");
		else if (i / 4 == 5) pr_cont("COU");
		else if (i / 4 == 6) pr_cont("PAS");
		else if (i / 4 == 7) pr_cont("FAI");
		else if (i / 4 == 8) pr_cont("CHA");
		else if (i / 4 == 9) pr_cont("AMB");
		else if (i / 4 == 10) pr_cont("RXS");
		else if (i / 4 == 11) pr_cont("TXS");
		else if (i / 4 == 12) pr_cont("RXO");
		else if (i / 4 == 13) pr_cont("TXO");
		else if (i / 4 == 14) pr_cont("RXG");
		else if (i / 4 == 15) pr_cont("TXG");
		else if (i / 4 == 16) pr_cont("RXR");
		else if (i / 4 == 17) pr_cont("TXT");
		else if (i / 4 == 18) pr_cont("RXT");
		else if (i / 4 == 19) pr_cont("TXR");

		pr_cont(" %2X, %2X, %2X, %2X  ", rBuff[i], rBuff[i + 1], rBuff[i + 2], rBuff[i + 3]);


		if (i / 4 == 4) {
			/* RX, RX open check. */
			if ((rBuff[i + 3] & 0x30) != 0)
				rc = 0;
			/* TX, RX GND(VDD) short check. */
			else if ((rBuff[i + 3] & 0xC0) != 0)
				rc = 0;
			/* RX-RX, TX-TX short check. */
			else if ((rBuff[i + 2] & 0x03) != 0)
				rc = 0;
			/* TX-RX short check. */
			else if ((rBuff[i + 2] & 0x04) != 0)
				rc = 0;
			else
				rc = 1;

			ts->ito_test[0] = rBuff[i];
			ts->ito_test[1] = rBuff[i + 1];
			ts->ito_test[2] = rBuff[i + 2];
			ts->ito_test[3] = rBuff[i + 3];
		}

	}

err_exit:
	kfree(rBuff);
	return rc;
}

#ifdef USE_STIM_PAD
static void run_fs_cal_pre_press(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	int ret = 0;
	u8 off[1] = {STATE_MANAGE_OFF};

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	input_info(true, &ts->client->dev, "%s: initial sequence for fs cal\n",
		   __func__);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: Touch is stopped!\n",
			  __func__);
		goto ErrorPowerOff;
	}

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_DISABLE_GAIN_LIMIT, off, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			   "%s: fail to disable gain limit\n", __func__);
		goto ErrorSendingCmd;
	}

	ret = sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH,
			       TOUCH_MODE_STATE_TOUCH);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: fail to fix tmode\n",
			  __func__);
		goto ErrorSendingCmd;
	}

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_DISABLE_BASELINE_ADAPT, off,
				   1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: fail to disable baselineAdapt\n", __func__);
		goto ErrorSendingCmd;
	}

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_DISABLE_DF, off, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: fail to disable df\n",
			  __func__);
		goto ErrorSendingCmd;
	}

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_TOUCH_ENGINE_MODE,
				   off, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: fail to disable touch engine\n", __func__);
		goto ErrorSendingCmd;
	}

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_RESET_BASELINE, NULL, 0);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: fail to fix tmode\n",
			  __func__);
		goto ErrorSendingCmd;
	}

	sec_ts_delay(50);

	input_info(true, &ts->client->dev, "%s: ready to press\n", __func__);

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

ErrorSendingCmd:
ErrorPowerOff:
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_fs_cal_get_data(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	struct sec_ts_test_mode mode;
	short *diff_table = NULL;
	const bool only_average = true;
	char *buff;
	int ret;
	int i, j;
	const unsigned int buff_size = ts->tx_count * ts->rx_count *
				       CMD_RESULT_WORD_LEN;
	unsigned int buff_len = 0;

	input_info(true, &ts->client->dev, "%s: fs cal with stim pad\n",
		   __func__);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: Touch is stopped!\n",
			  __func__);
		sec_cmd_set_cmd_result(sec, "NG", 2);
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 3) {
		input_err(true, &ts->client->dev,
			  "%s: Parameter Error\n", __func__);
		sec_cmd_set_cmd_result(sec, "NG", 2);
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	buff = kzalloc(buff_size, GFP_KERNEL);
	if (!buff) {
		sec_cmd_set_cmd_result(sec, "NG", 2);
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	memset(&mode, 0x00, sizeof(struct sec_ts_test_mode));

	if (sec->cmd_param[0] == 0) {
		mode.type = TYPE_REMV_AMB_DATA;
		mode.spec_check = SPEC_NO_CHECK;
		ret = sec_ts_read_frame_stdev(ts, sec, mode.type, mode.min,
				mode.max, &mode.spec_check, only_average);
		if (ret < 0) {
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len, "NG");
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			goto SetCmdResult;
		}
		ret = sec_ts_check_fs_precal(ts);

		/* ret = NG node count */
		if (ret > 0) {
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len, "NG\n");
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
		} else {
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len, "OK\n");
			sec->cmd_state = SEC_CMD_STATUS_OK;
		}

		for (i = 0; i < ts->tx_count * ts->rx_count; i++) {
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len, "%4d,",
					      ts->pFrame[i]);

			if (i % ts->tx_count == ts->tx_count - 1)
				buff_len += scnprintf(buff + buff_len,
						      buff_size - buff_len,
						      "\n");
		}
		/* calculate gain table, and store it to ts->gainTable */
		sec_ts_get_gain_table(ts);

	} else if (sec->cmd_param[0] == 1) {
		/* write gaintable(ts->gainTable) to ic */
		ret = sec_ts_write_gain_table(ts);
		if (ret < 0) {
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len,
					      "NG %d %d\n",
					      ts->rx_count, ts->tx_count);
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
		} else {
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len,
					      "OK %d %d\n",
					      ts->rx_count, ts->tx_count);
			sec->cmd_state = SEC_CMD_STATUS_OK;
		}

		for (j = 0; j < ts->rx_count; j++) {
			for (i = 0; i < ts->tx_count; i++) {
				buff_len += scnprintf(buff + buff_len,
						buff_size - buff_len,
						"%4d,",
						ts->gainTable[i*ts->rx_count
								+ j]);
			}
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len, "\n");
		}
	} else if (sec->cmd_param[0] == 2) {
		int mean;
		/* for stim pad fixture 2 */
		mode.type = TYPE_NORM2_DATA;
		mode.spec_check = SPEC_NO_CHECK;

		ret = sec_ts_read_frame_stdev(ts, sec, mode.type, mode.min,
				mode.max, &mode.spec_check, only_average);
		if (ret < 0) {
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len, "NG");
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			goto SetCmdResult;
		}

		mean = sec_ts_get_postcal_mean(ts);
		ts->fs_postcal_mean = mean;

		input_info(true, &ts->client->dev,
			   "%s : FS mean = %d\n", __func__, mean);

		if ((mean > fs_mean_target_h) || (mean < fs_mean_target_l)) {
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len,
					      "NG %d %d\n%d\n",
					      ts->rx_count, ts->tx_count,
					      ts->fs_postcal_mean);
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
		} else {
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len,
					      "OK %d %d\n%d\n",
					      ts->rx_count, ts->tx_count,
					      ts->fs_postcal_mean);
			sec->cmd_state = SEC_CMD_STATUS_OK;
		}

		for (i = 0; i < ts->tx_count * ts->rx_count; i++) {
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len, "%4d,",
					      ts->pFrame[i]);

			if (i % ts->tx_count == ts->tx_count - 1)
				buff_len += scnprintf(buff + buff_len,
						      buff_size - buff_len,
						      "\n");
		}
	} else { //(sec->cmd_param[0] == 3)
		/* get fs_uniformity */

		diff_table = kzalloc(ts->tx_count * ts->rx_count * 2,
				GFP_KERNEL);

		if ((diff_table == NULL) || (ts->fs_postcal_mean == 0)) {
			input_err(true, &ts->client->dev,
					"%s: fail to alloc diffTable, postcal mean = %d\n",
					__func__, ts->fs_postcal_mean);
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len, "NG %d %d",
					      ts->rx_count, ts->tx_count);
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			goto SetCmdResult;
		}

		ret = sec_ts_get_postcal_uniformity(ts, diff_table);

		if (ret == 0) {
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len,
					      "OK %d %d\n",
					      ts->rx_count, ts->tx_count);
			sec->cmd_state = SEC_CMD_STATUS_OK;
		} else {
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len,
					      "NG %d %d\n",
					      ts->rx_count, ts->tx_count);
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
		}

		for (i = 0; i < ts->tx_count * ts->rx_count; i++) {
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len, "%4d,",
					      diff_table[i]);

			if (i % ts->tx_count == ts->tx_count - 1)
				buff_len += scnprintf(buff + buff_len,
						      buff_size - buff_len,
						      "\n");
		}
	}

SetCmdResult:

	sec_cmd_set_cmd_result(sec, buff, buff_len);

	kfree(diff_table);
	kfree(buff);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_fs_cal_post_press(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	int ret = 0;
	u8 on[1] = {STATE_MANAGE_ON};

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: Touch is stopped!\n",
			  __func__);
		goto ErrorPowerOff;
	}

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_TOUCH_ENGINE_MODE, on, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: fail to enable touch engine\n", __func__);
		goto ErrorSendingCmd;
	}

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_DISABLE_DF, on, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: fail to enable df\n",
			  __func__);
		goto ErrorSendingCmd;
	}

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_DISABLE_BASELINE_ADAPT, on,
				   1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: fail to enable baselineAdapt\n", __func__);
		goto ErrorSendingCmd;
	}

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_DISABLE_GAIN_LIMIT, on, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: fail to enable gain limit\n", __func__);
		goto ErrorSendingCmd;
	}

	ret = sec_ts_release_tmode(ts);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: fail to release tmode\n",
			  __func__);
		goto ErrorSendingCmd;
	}

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

ErrorSendingCmd:
ErrorPowerOff:
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

/* enable_fs_cal_table : enable or disable fs cal table
 * cmd_param : 0 to disable, 1 to enable
 * touch mode and state should be fixed before enable or disable
 */
static void enable_fs_cal_table(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	u8 tPara;
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		sec_cmd_set_cmd_result(sec, "NG", 2);
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	ret = sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH,
				TOUCH_MODE_STATE_TOUCH);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: fail to fix tmode\n",
			  __func__);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		sec_cmd_set_cmd_result(sec, "NG", 2);
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	tPara = sec->cmd_param[0];

	input_info(true, &ts->client->dev, "%s: fs cal table %s\n",
		   __func__, ((tPara == 0) ? "disable" : "enable"));

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_DISABLE_NORM_TABLE,
				   &tPara, 1);

	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: cmd write failed\n", __func__);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		sec_cmd_set_cmd_result(sec, "NG", 2);
	} else {
		sec->cmd_state = SEC_CMD_STATUS_OK;
		sec_cmd_set_cmd_result(sec, "OK", 2);
	}

	ret = sec_ts_release_tmode(ts);
	if (ret < 0)
		input_err(true, &ts->client->dev, "%s: fail to release tmode\n",
			  __func__);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void enable_gain_limit(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	int ret = 0;
	u8 tPara;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: Touch is stopped!\n",
			  __func__);
		sec_cmd_set_cmd_result(sec, "TSP turned off", 14);
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		sec_cmd_set_cmd_result(sec, "NG", 2);
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	tPara = sec->cmd_param[0];

	input_info(true, &ts->client->dev, "%s: gain limit %s\n",
		   __func__, ((tPara == 0) ? "disable" : "enable"));

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_DISABLE_GAIN_LIMIT, &tPara,
				   1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: cmd write failed\n", __func__);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		sec_cmd_set_cmd_result(sec, "NG", 2);
	} else {
		sec->cmd_state = SEC_CMD_STATUS_OK;
		sec_cmd_set_cmd_result(sec, "OK", 2);
	}

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}
#endif

static void enable_coordinate_report(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	int ret = 0;
	u8 tPara;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: Touch is stopped!\n",
			  __func__);
		sec_cmd_set_cmd_result(sec, "TSP turned off", 14);
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		sec_cmd_set_cmd_result(sec, "NG", 2);
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	tPara = sec->cmd_param[0];

	input_info(true, &ts->client->dev, "%s: coordinate report %s\n",
		   __func__, ((tPara == 0) ? "disable" : "enable"));

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_TOUCH_ENGINE_MODE,
				   &tPara, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: cmd write failed\n", __func__);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		sec_cmd_set_cmd_result(sec, "NG", 2);
	} else {
		sec->cmd_state = SEC_CMD_STATUS_OK;
		sec_cmd_set_cmd_result(sec, "OK", 2);
	}

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_trx_short_test(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	int rc;
	char para = TO_TOUCH_MODE;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: Touch is stopped!\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	disable_irq(ts->client->irq);

	rc = execute_selftest(ts, TEST_SHORT | TEST_OPEN | TEST_NODE_VARIANCE);
	if (rc > 0) {
		ts->sec_ts_write(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
		enable_irq(ts->client->irq);
		snprintf(buff, sizeof(buff), "%s", "OK");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_OK;

		input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	ts->sec_ts_write(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
	enable_irq(ts->client->irq);

	snprintf(buff, sizeof(buff), "%s", "NG");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_FAIL;

	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}


int sec_ts_execute_force_calibration(struct sec_ts_data *ts, int cal_mode)
{
	int rc = -1;
	u8 cmd;

	input_info(true, &ts->client->dev, "%s: %d\n", __func__, cal_mode);

	if (cal_mode == OFFSET_CAL_SEC)
		cmd = SEC_TS_CMD_FACTORY_PANELCALIBRATION;
	else if (cal_mode == AMBIENT_CAL)
		cmd = SEC_TS_CMD_CALIBRATION_AMBIENT;
#ifdef USE_PRESSURE_SENSOR
	else if (cal_mode == PRESSURE_CAL)
		cmd = SEC_TS_CMD_CALIBRATION_PRESSURE;
#endif
	else
		return rc;

	if (ts->sec_ts_write(ts, cmd, NULL, 0) < 0) {
		input_err(true, &ts->client->dev,
			  "%s: Write Cal commend failed!\n", __func__);
		return rc;
	}

	sec_ts_delay(4000);

	rc = sec_ts_wait_for_ready(ts, SEC_TS_VENDOR_ACK_OFFSET_CAL_DONE);

	return rc;
}

static void run_force_calibration(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	int rc;
#ifdef PAT_CONTROL
	u8 img_ver[4];
#endif
	struct sec_ts_test_mode mode;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: Touch is stopped!\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;

		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	sec_ts_read_calibration_report(ts);

	if (ts->touch_count > 0) {
		snprintf(buff, sizeof(buff), "%s", "NG_FINGER_ON");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto out_force_cal;
	}

	disable_irq(ts->client->irq);

	rc = sec_ts_execute_force_calibration(ts, OFFSET_CAL_SEC);
	if (rc < 0) {
		snprintf(buff, sizeof(buff), "%s", "FAIL");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	} else {
#ifdef USE_PRESSURE_SENSOR
		rc = sec_ts_execute_force_calibration(ts, PRESSURE_CAL);
		if (rc < 0)
			input_err(true, &ts->client->dev,
				"%s: fail to write PRESSURE CAL!\n", __func__);
#endif

		if (ts->plat_data->mis_cal_check &&
		    sec_ts_run_cal_check(ts)) {
			memset(&mode, 0x00,
			       sizeof(struct sec_ts_test_mode));
			mode.type = TYPE_AMBIENT_DATA;
			mode.allnode = TEST_MODE_ALL_NODE;

			sec_ts_read_raw_data(ts, NULL, &mode);
			snprintf(buff, sizeof(buff), "%s", "MIS CAL");
			sec->cmd_state = SEC_CMD_STATUS_FAIL;

			enable_irq(ts->client->irq);
			goto out_force_cal;
		}

#ifdef PAT_CONTROL
		ts->cal_count = get_tsp_nvm_data(ts,
					SEC_TS_NVM_OFFSET_CAL_COUNT);

		if (ts->external_factory == true) {
			/* for external factory mode */
			if (ts->cal_count == PAT_MAX_EXT)
				ts->cal_count = PAT_MAX_EXT;
			else if (ts->cal_count >= PAT_EXT_FACT &&
				 ts->cal_count < PAT_MAX_EXT)
				ts->cal_count++;
			else
				ts->cal_count = PAT_EXT_FACT;

			/* not to enter external factory mode without setting
			 * everytime
			 **/
			ts->external_factory = false;
		} else {
			/* change from (virtual pat or vpat by external fatory)
			 * to real pat by forced calibarion by LCIA
			 **/
			if (ts->cal_count >= PAT_MAGIC_NUMBER)
				ts->cal_count = 1;
			else if (ts->cal_count == PAT_MAX_LCIA)
				ts->cal_count = PAT_MAX_LCIA;
			else
				ts->cal_count++;
		}

		/* Use TSP NV area : in this model, use only one byte
		 * buff[0] : offset from user NVM storage
		 * buff[1] : length of stroed data - 1 (ex. using 1byte,
				value is 1 - 1 = 0)
		 * buff[2] : write data
		 **/
		buff[0] = SEC_TS_NVM_OFFSET_CAL_COUNT;
		buff[1] = 0;
		buff[2] = ts->cal_count;
		input_info(true, &ts->client->dev,
			   "%s: write to nvm cal_count(%2X)\n",
			   __func__, buff[2]);

		rc = ts->sec_ts_write(ts, SEC_TS_CMD_NVM, buff, 3);
		if (rc < 0) {
			input_err(true, &ts->client->dev,
				"%s: nvm write failed. ret: %d\n",
				__func__, rc);
		}

		sec_ts_delay(20);

		ts->cal_count = get_tsp_nvm_data(ts,
						 SEC_TS_NVM_OFFSET_CAL_COUNT);

		rc = ts->sec_ts_read(ts, SEC_TS_READ_IMG_VERSION, img_ver, 4);
		if (rc < 0) {
			input_err(true, &ts->client->dev,
				  "%s: Image version read error\n", __func__);
		} else {
			memset(buff, 0x00, SEC_CMD_STR_LEN);
			buff[0] = get_tsp_nvm_data(ts,
						SEC_TS_NVM_OFFSET_TUNE_VERSION);
			if (buff[0] == 0xFF) {
				set_tsp_nvm_data_clear(ts,
					SEC_TS_NVM_OFFSET_TUNE_VERSION);
				set_tsp_nvm_data_clear(ts,
					SEC_TS_NVM_OFFSET_TUNE_VERSION + 1);
			}

			ts->tune_fix_ver = (img_ver[2]<<8 | img_ver[3]);
			buff[0] = SEC_TS_NVM_OFFSET_TUNE_VERSION;
			buff[1] = 1;// 2bytes
			buff[2] = img_ver[2];
			buff[3] = img_ver[3];
			input_info(true, &ts->client->dev,
				   "%s: write tune_ver to nvm (%2X %2X)\n",
				   __func__, buff[2], buff[3]);

			rc = ts->sec_ts_write(ts, SEC_TS_CMD_NVM, buff, 4);
			if (rc < 0) {
				input_err(true, &ts->client->dev,
					"%s: nvm write failed. ret: %d\n",
					__func__, rc);
			}
			sec_ts_delay(20);

			buff[0] = get_tsp_nvm_data(ts,
					SEC_TS_NVM_OFFSET_TUNE_VERSION);
			buff[1] = get_tsp_nvm_data(ts,
					SEC_TS_NVM_OFFSET_TUNE_VERSION + 1);
			ts->tune_fix_ver = buff[0]<<8 | buff[1];
			input_info(true, &ts->client->dev,
				"%s: cal_count [%2X] tune_fix_ver [%04X]\n",
				__func__, ts->cal_count, ts->tune_fix_ver);
		}
#endif
		snprintf(buff, sizeof(buff), "%s", "OK");
		sec->cmd_state = SEC_CMD_STATUS_OK;
	}

	enable_irq(ts->client->irq);

out_force_cal:
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));

	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void get_force_calibration(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	int buff_len = sizeof(buff);
	int written = 0;
	int rc;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: Touch is stopped!\n",
			  __func__);
		written += scnprintf(buff + written, buff_len - written,
			"%s", "TSP turned off");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;

		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	rc = sec_ts_read_calibration_report(ts);
	if (rc < 0) {
		written += scnprintf(buff + written, buff_len - written,
			"%s\n", "FAIL");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	} else if (rc == SEC_TS_STATUS_CALIBRATION_SDC) {
		written += scnprintf(buff + written, buff_len - written,
			"%s\n", "OK(MODULE)");
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else if (rc == SEC_TS_STATUS_CALIBRATION_SEC) {
		written += scnprintf(buff + written, buff_len - written,
			"%s\n", "OK");
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		written += scnprintf(buff + written, buff_len - written,
			"%s\n", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}

	written += scnprintf(buff + written, buff_len - written,
		"%02X, %02X, %02X, %02X, %02X, %02X, %02X, %02X\n",
		ts->cali_report[0], ts->cali_report[1], ts->cali_report[2],
		ts->cali_report[3], ts->cali_report[4], ts->cali_report[5],
		ts->cali_report[6], ts->cali_report[7]);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));

	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

#ifdef USE_PRESSURE_SENSOR
static void run_force_pressure_calibration(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	int rc;
	char data[3] = { 0 };

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: Touch is stopped!\n",
			  __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;

		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	if (ts->touch_count > 0) {
		snprintf(buff, sizeof(buff), "%s", "NG_FINGER_ON");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto out_force_pressure_cal;
	}

	disable_irq(ts->client->irq);

	rc = sec_ts_execute_force_calibration(ts, PRESSURE_CAL);
	if (rc < 0) {
		snprintf(buff, sizeof(buff), "%s", "FAIL");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	} else {
		snprintf(buff, sizeof(buff), "%s", "OK");
		sec->cmd_state = SEC_CMD_STATUS_OK;
	}

	ts->pressure_cal_base = get_tsp_nvm_data(ts,
				SEC_TS_NVM_OFFSET_PRESSURE_BASE_CAL_COUNT);
	if (ts->pressure_cal_base == 0xFF)
		ts->pressure_cal_base = 0;
	if (ts->pressure_cal_base > 0xFD)
		ts->pressure_cal_base = 0xFD;

	/* Use TSP NV area : in this model, use only one byte
	 * data[0] : offset from user NVM storage
	 * data[1] : length of stroed data - 1 (ex. using 1byte,
	 *           value is  1 - 1 = 0)
	 * data[2] : write data
	 **/
	data[0] = SEC_TS_NVM_OFFSET_PRESSURE_BASE_CAL_COUNT;
	data[1] = 0;
	data[2] = ts->pressure_cal_base + 1;

	rc = ts->sec_ts_write(ts, SEC_TS_CMD_NVM, data, 3);
	if (rc < 0)
		input_err(true, &ts->client->dev,
			"%s: nvm write failed. ret: %d\n", __func__, rc);

	ts->pressure_cal_base = get_tsp_nvm_data(ts,
				SEC_TS_NVM_OFFSET_PRESSURE_BASE_CAL_COUNT);

	input_info(true, &ts->client->dev, "%s: count: %d\n",
		   __func__, ts->pressure_cal_base);

	enable_irq(ts->client->irq);

out_force_pressure_cal:
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));

	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void set_pressure_test_mode(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	int ret;
	unsigned char data = TYPE_INVALID_DATA;
	unsigned char enable = 0;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: Touch is stopped!\n",
			  __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;

		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	if (sec->cmd_param[0] == 1) {
		enable = 0x1;
		ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_TEMPERATURE_COMP_MODE,
				       &enable, 1);
		if (ret < 0) {
			snprintf(buff, sizeof(buff), "%s", "NG");
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			goto out_test_mode;
		}

		ret = sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH,
				       TOUCH_MODE_STATE_TOUCH);
		if (ret < 0) {
			snprintf(buff, sizeof(buff), "%s", "NG");
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			goto out_test_mode;
		}

	} else {
		ret = ts->sec_ts_write(ts, SEC_TS_CMD_SELECT_PRESSURE_TYPE,
				       &data, 1);
		if (ret < 0) {
			snprintf(buff, sizeof(buff), "%s", "NG");
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			goto out_test_mode;
		}

		ret = sec_ts_release_tmode(ts);
		if (ret < 0) {
			snprintf(buff, sizeof(buff), "%s", "NG");
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			goto out_test_mode;
		}

		enable = 0x0;
		ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_TEMPERATURE_COMP_MODE,
				       &enable, 1);
		if (ret < 0) {
			snprintf(buff, sizeof(buff), "%s", "NG");
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			goto out_test_mode;
		}
	}

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;

out_test_mode:
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));

	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static int read_pressure_data(struct sec_ts_data *ts, u8 type, short *value)
{
	unsigned char data[6] = { 0 };
	short pressure[3] = { 0 };
	int ret;

	if (ts->power_status == SEC_TS_STATE_POWER_OFF)
		return -ENODEV;

	ret = ts->sec_ts_read(ts, SEC_TS_CMD_SELECT_PRESSURE_TYPE, data, 1);
	if (ret < 0)
		return -EIO;

	if (data[0] != type) {
		input_info(true, &ts->client->dev, "%s: type change to %02X\n",
			   __func__, type);

		data[1] = type;

		ret = ts->sec_ts_write(ts, SEC_TS_CMD_SELECT_PRESSURE_TYPE,
				       &data[1], 1);
		if (ret < 0)
			return -EIO;

		sec_ts_delay(30);
	}

	memset(data, 0x00, 6);

	ret = ts->sec_ts_read(ts, SEC_TS_CMD_READ_PRESSURE_DATA, data, 6);
	if (ret < 0)
		return -EIO;

	pressure[0] = (data[0] << 8 | data[1]);
	pressure[1] = (data[2] << 8 | data[3]);
	pressure[2] = (data[4] << 8 | data[5]);

	input_info(true, &ts->client->dev,
		   "%s: Left: %d, Center: %d, Rignt: %d\n",
		   __func__, pressure[2], pressure[1], pressure[0]);

	memcpy(value, pressure, 3 * 2);

	return ret;
}

static void run_pressure_filtered_strength_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	short pressure[3] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	ret = read_pressure_data(ts, TYPE_SIGNAL_DATA, pressure);
	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "WRITE FAILED");
		goto error_read_str;
	}

	snprintf(buff, sizeof(buff), "%d,%d,%d",
		 pressure[2], pressure[1], pressure[0]);

	sec->cmd_state = SEC_CMD_STATUS_OK;

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));

	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

error_read_str:
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_pressure_strength_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	short pressure[3] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	ret = read_pressure_data(ts, TYPE_REMV_AMB_DATA, pressure);
	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "WRITE FAILED");
		goto error_read_str;
	}

	snprintf(buff, sizeof(buff), "%d,%d,%d",
		 pressure[2], pressure[1], pressure[0]);

	sec->cmd_state = SEC_CMD_STATUS_OK;

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));

	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

error_read_str:
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_pressure_rawdata_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	short pressure[3] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	ret = read_pressure_data(ts, TYPE_RAW_DATA, pressure);
	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "WRITE FAILED");
		goto error_read_rawdata;
	}

	snprintf(buff, sizeof(buff), "%d,%d,%d",
		 pressure[2], pressure[1], pressure[0]);

	sec->cmd_state = SEC_CMD_STATUS_OK;

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));

	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

error_read_rawdata:
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void run_pressure_offset_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	short pressure[3] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	ret = read_pressure_data(ts, TYPE_OFFSET_DATA_SEC, pressure);
	if (ret < 0) {
		snprintf(buff, sizeof(buff), "%s", "WRITE FAILED");
		goto error_read_str;
	}

	snprintf(buff, sizeof(buff), "%d,%d,%d",
		 pressure[2], pressure[1], pressure[0]);

	sec->cmd_state = SEC_CMD_STATUS_OK;

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));

	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

error_read_str:
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void set_pressure_strength(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	u8 data[8] = { 0 };
	u8 cal_data[18] = { 0 };
	int index;
	int ret;
	short pressure[3] = { 0 };

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if ((sec->cmd_param[0] < 1) || (sec->cmd_param[0] > 4))
		goto err_cmd_param_str;

	index = sec->cmd_param[0] - 1;

	/* RIGHT */
	cal_data[0] = (sec->cmd_param[3] >> 8);
	cal_data[1] = (sec->cmd_param[3] & 0xFF);
	/* CENTER */
	cal_data[8] = (sec->cmd_param[2] >> 8);
	cal_data[9] = (sec->cmd_param[2] & 0xFF);
	/* LEFT */
	cal_data[16] = (sec->cmd_param[1] >> 8);
	cal_data[17] = (sec->cmd_param[1] & 0xFF);

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_GET_PRESSURE, cal_data, 18);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: cmd write failed. ret: %d\n", __func__, ret);
		goto err_comm_str;
	}

	sec_ts_delay(30);

	memset(cal_data, 0x00, 18);
	ret = ts->sec_ts_read(ts, SEC_TS_CMD_SET_GET_PRESSURE, cal_data, 18);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: cmd write failed. ret: %d\n", __func__, ret);
		goto err_comm_str;
	}

	pressure[0] = ((cal_data[16] << 8) | cal_data[17]);
	pressure[1] = ((cal_data[8] << 8) | cal_data[9]);
	pressure[2] = ((cal_data[0] << 8) | cal_data[1]);

	input_info(true, &ts->client->dev, "%s: [%d] : %d, %d, %d\n",
		   __func__, pressure[0], pressure[1], pressure[2]);

	/* Use TSP NV area : in this model, use only one byte
	 * buff[0] : offset from user NVM storage
	 * buff[1] : [n] length of stroed data - 1 (ex. using 1byte,
	 *           value is  1 - 1 = 0)
	 * buff[2] ... [n] : write data ...
	 **/
	data[0] = SEC_TS_NVM_OFFSET_PRESSURE_STRENGTH +
			(index * SEC_TS_NVM_SIZE_PRESSURE_BLOCK);
	data[1] = SEC_TS_NVM_SIZE_PRESSURE_BLOCK - 1;
	/* RIGHT */
	data[2] = (sec->cmd_param[3] >> 8);
	data[3] = (sec->cmd_param[3] & 0xFF);
	/* CENTER */
	data[4] = (sec->cmd_param[2] >> 8);
	data[5] = (sec->cmd_param[2] & 0xFF);
	/*LEFT */
	data[6] = (sec->cmd_param[1] >> 8);
	data[7] = (sec->cmd_param[1] & 0xFF);

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_NVM, data, 8);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: nvm write failed. ret: %d\n", __func__, ret);
		goto err_comm_str;
	}

	sec_ts_delay(20);

	input_info(true, &ts->client->dev, "%s: [%d] : %d, %d, %d\n",
		__func__, index,
		(data[6] << 8) + data[7],
		(data[4] << 8) + data[5],
		(data[0] << 8) + data[1]);

	memset(data, 0x00, 8);

	data[0] = SEC_TS_NVM_OFFSET_PRESSURE_INDEX;
	data[1] = 0;
	data[2] = (u8)(index & 0xFF);

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_NVM, data, 3);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: nvm write failed. ret: %d\n", __func__, ret);
		goto err_comm_str;
	}

	sec_ts_delay(20);

	snprintf(buff, sizeof(buff), "%s", "OK");

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	ts->pressure_cal_delta = get_tsp_nvm_data(ts,
				SEC_TS_NVM_OFFSET_PRESSURE_DELTA_CAL_COUNT);
	if (ts->pressure_cal_delta == 0xFF)
		ts->pressure_cal_delta = 0;

	if (ts->pressure_cal_delta > 0xFD)
		ts->pressure_cal_delta = 0xFD;

	/* Use TSP NV area : in this model, use only one byte
	 * data[0] : offset from user NVM storage
	 * data[1] : length of stroed data - 1 (ex. using 1byte,
	 *           value is  1 - 1 = 0)
	 * data[2] : write data
	 **/
	data[0] = SEC_TS_NVM_OFFSET_PRESSURE_DELTA_CAL_COUNT;
	data[1] = 0;
	data[2] = ts->pressure_cal_delta + 1;

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_NVM, data, 3);
	if (ret < 0)
		input_err(true, &ts->client->dev,
			"%s: nvm write failed. ret: %d\n", __func__, ret);

	ts->pressure_cal_delta = get_tsp_nvm_data(ts,
				SEC_TS_NVM_OFFSET_PRESSURE_DELTA_CAL_COUNT);

	input_info(true, &ts->client->dev,
		   "%s: count: %d\n", __func__, ts->pressure_cal_delta);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

err_cmd_param_str:
	input_info(true, &ts->client->dev, "%s: parameter error: %u\n",
			__func__, sec->cmd_param[0]);
err_comm_str:
	snprintf(buff, sizeof(buff), "NG");
	sec_cmd_set_cmd_result(sec, buff, strlen(buff));

	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void set_pressure_rawdata(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	u8 data[8] = { 0 };
	int index;
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if ((sec->cmd_param[0] < 1) || (sec->cmd_param[0] > 4))
		goto err_cmd_param_raw;

	index = sec->cmd_param[0] - 1;

	/* Use TSP NV area : in this model, use only one byte
	 * buff[0] : offset from user NVM storage
	 * buff[1] : [n] length of stroed data - 1 (ex. using 1byte,
	 *           value is  1 - 1 = 0)
	 * buff[2] ... [n] : write data ...
	 **/
	data[0] = SEC_TS_NVM_OFFSET_PRESSURE_RAWDATA +
			(index * SEC_TS_NVM_SIZE_PRESSURE_BLOCK);
	data[1] = SEC_TS_NVM_SIZE_PRESSURE_BLOCK - 1;
	data[2] = (sec->cmd_param[3] >> 8);
	data[3] = (sec->cmd_param[3] & 0xFF);
	data[4] = (sec->cmd_param[2] >> 8);
	data[5] = (sec->cmd_param[2] & 0xFF);
	data[6] = (sec->cmd_param[1] >> 8);
	data[7] = (sec->cmd_param[1] & 0xFF);

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_NVM, data, 8);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: nvm write failed. ret: %d\n", __func__, ret);
		goto err_comm_raw;
	}
	sec_ts_delay(20);

	memset(data, 0x00, 8);

	ret = get_tsp_nvm_data_by_size(ts, SEC_TS_NVM_OFFSET_PRESSURE_RAWDATA +
				(index * SEC_TS_NVM_SIZE_PRESSURE_BLOCK),
				SEC_TS_NVM_SIZE_PRESSURE_BLOCK, data);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: nvm read failed. ret: %d\n", __func__, ret);
		goto err_comm_raw;
	}

	snprintf(buff, sizeof(buff), "%s", "OK");

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &ts->client->dev, "%s: [%d] : %d, %d, %d\n",
		__func__, index,
		(data[4] << 8) + data[5],
		(data[2] << 8) + data[3],
		(data[1] << 8) + data[0]);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

err_cmd_param_raw:
	input_info(true, &ts->client->dev, "%s: parameter error: %u\n",
			__func__, sec->cmd_param[0]);
err_comm_raw:

	snprintf(buff, sizeof(buff), "NG");
	sec_cmd_set_cmd_result(sec, buff, strlen(buff));

	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void set_pressure_data_index(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	u8 data[8] = { 0 };
	u8 cal_data[18] = { 0 };
	int index;
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if ((sec->cmd_param[0] < 0) || (sec->cmd_param[0] > 4))
		goto err_set_cmd_param_index;

	if (sec->cmd_param[0] == 0) {
		input_info(true, &ts->client->dev,
			   "%s: clear calibration result\n", __func__);
		/* clear pressure calibrated data */
		ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_GET_PRESSURE,
				       cal_data, 18);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				"%s: cmd write failed. ret: %d\n",
				__func__, ret);
			goto err_set_comm_index;
		}

		sec_ts_delay(30);

		goto clear_index;
	}

	index = sec->cmd_param[0] - 1;

	ret = get_tsp_nvm_data_by_size(ts, SEC_TS_NVM_OFFSET_PRESSURE_STRENGTH,
				       24, data);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: nvm read failed. ret: %d\n", __func__, ret);
		goto err_set_comm_index;
	}

	cal_data[16] = data[6 * index + 4];
	cal_data[17] = data[6 * index + 5];	/* LEFT */

	cal_data[8] = data[6 * index + 2];
	cal_data[9] = data[6 * index + 3];	/* CENTER */

	cal_data[0] = data[6 * index + 0];
	cal_data[1] = data[6 * index + 1];	/* RIGHT */

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_GET_PRESSURE, cal_data, 18);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: cmd write failed. ret: %d\n", __func__, ret);
		goto err_set_comm_index;
	}

	sec_ts_delay(30);

	/* Use TSP NV area : in this model, use only one byte
	 * buff[0] : offset from user NVM storage
	 * buff[1] : length of stroed data - 1 (ex. using 1byte,
	 *           value is  1 - 1 = 0)
	 * buff[2] : write data
	 **/
	memset(data, 0x00, 8);
	data[0] = SEC_TS_NVM_OFFSET_PRESSURE_INDEX;
	data[1] = 0;
	data[2] = (u8)(index & 0xFF);

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_NVM, data, 3);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: nvm write failed. ret: %d\n", __func__, ret);
		goto err_set_comm_index;
	}

	sec_ts_delay(20);

clear_index:
	snprintf(buff, sizeof(buff), "%s", "OK");

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

err_set_cmd_param_index:
	input_info(true, &ts->client->dev, "%s: parameter error: %u\n",
			__func__, sec->cmd_param[0]);
err_set_comm_index:

	snprintf(buff, sizeof(buff), "NG");
	sec_cmd_set_cmd_result(sec, buff, strlen(buff));

	sec->cmd_state = SEC_CMD_STATUS_FAIL;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void get_pressure_strength(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	int index;
	u8 data[24] = { 0 };
	short pressure[3] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if ((sec->cmd_param[0] < 1) || (sec->cmd_param[0] > 4))
		goto err_get_cmd_param_str;

	index = sec->cmd_param[0] - 1;

	ret = get_tsp_nvm_data_by_size(ts, SEC_TS_NVM_OFFSET_PRESSURE_STRENGTH,
				       24, data);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: nvm read failed. ret: %d\n", __func__, ret);
		goto err_get_comm_str;
	}

	pressure[0] = ((data[6 * index + 4] << 8) + data[6 * index + 5]);
	pressure[1] = ((data[6 * index + 2] << 8) + data[6 * index + 3]);
	pressure[2] = ((data[6 * index + 0] << 8) + data[6 * index + 1]);

	snprintf(buff, sizeof(buff), "%d,%d,%d",
		 pressure[0], pressure[1], pressure[2]);

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &ts->client->dev, "%s: [%d] : %d, %d, %d\n",
		__func__, index, pressure[0], pressure[1], pressure[2]);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

err_get_comm_str:
	input_info(true, &ts->client->dev, "%s: parameter error: %u\n",
			__func__, sec->cmd_param[0]);
err_get_cmd_param_str:

	snprintf(buff, sizeof(buff), "NG");
	sec_cmd_set_cmd_result(sec, buff, strlen(buff));

	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void get_pressure_rawdata(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	int index;
	u8 data[24] = { 0 };
	short pressure[3] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if ((sec->cmd_param[0] < 1) || (sec->cmd_param[0] > 4))
		goto err_get_cmd_param_raw;

	index = sec->cmd_param[0] - 1;

	ret = get_tsp_nvm_data_by_size(ts, SEC_TS_NVM_OFFSET_PRESSURE_RAWDATA,
				       24, data);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: nvm read failed. ret: %d\n", __func__, ret);
		goto err_get_comm_raw;
	}

	pressure[0] = ((data[6 * index + 4] << 8) + data[6 * index + 5]);
	pressure[1] = ((data[6 * index + 2] << 8) + data[6 * index + 3]);
	pressure[2] = ((data[6 * index + 0] << 8) + data[6 * index + 1]);

	snprintf(buff, sizeof(buff), "%d,%d,%d",
		 pressure[0], pressure[1], pressure[2]);

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &ts->client->dev, "%s: [%d] : %d, %d, %d\n",
		__func__, index, pressure[0], pressure[1], pressure[2]);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;
err_get_cmd_param_raw:
	input_info(true, &ts->client->dev, "%s: parameter error: %u\n",
			__func__, sec->cmd_param[0]);
err_get_comm_raw:

	snprintf(buff, sizeof(buff), "NG");
	sec_cmd_set_cmd_result(sec, buff, strlen(buff));

	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void get_pressure_data_index(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	int index = 0;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	index = get_tsp_nvm_data(ts, SEC_TS_NVM_OFFSET_PRESSURE_INDEX);
	if (index < 0) {
		goto err_get_index;
	} else {
		if (index == 0xFF)
			snprintf(buff, sizeof(buff), "%d", 0);
		else
			snprintf(buff, sizeof(buff), "%d", index + 1);
	}

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &ts->client->dev, "%s: %d\n",
		__func__, index);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

err_get_index:

	snprintf(buff, sizeof(buff), "NG");
	sec_cmd_set_cmd_result(sec, buff, strlen(buff));

	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}
static void set_pressure_strength_clear(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	u8 *data;
	u8 cal_data[18] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	/* clear pressure calibrated data */
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_GET_PRESSURE, cal_data, 18);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: cmd write failed. ret: %d\n", __func__, ret);
		goto err_comm_str;
	}

	sec_ts_delay(30);

	/* Use TSP NV area : in this model, use only one byte
	 * buff[0] : offset from user NVM storage
	 * buff[1] : length of stroed data - 1 (ex. using 1byte,
	 *           value is  1 - 1 = 0)
	 * buff[2] : write data
	 **/

	/* strength 6 * 4,  rawdata 6 * 4, buff[0], buff[1] */
	data = kzalloc(50, GFP_KERNEL);
	if (!data) {
		input_err(true, &ts->client->dev,
			"%s failed to allocate memory. ret: %d\n",
			__func__, ret);
		goto err_comm_str;
	}

	data[0] = SEC_TS_NVM_OFFSET_PRESSURE_INDEX;
	data[1] = (SEC_TS_NVM_SIZE_PRESSURE_BLOCK * 8) - 1;

	/* remove calicated strength, rawdata in NVM */
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_NVM, data, 50);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: nvm write failed. ret: %d\n", __func__, ret);
		goto err_mem_str;
	}

	sec_ts_delay(20);

	snprintf(buff, sizeof(buff), "%s", "OK");

	kfree(data);

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

err_mem_str:
	kfree(data);
err_comm_str:
	snprintf(buff, sizeof(buff), "NG");
	sec_cmd_set_cmd_result(sec, buff, strlen(buff));

	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void get_pressure_threshold(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	char buff[SEC_CMD_STR_LEN] = {0};

	sec_cmd_set_default_result(sec);

	snprintf(buff, sizeof(buff), "300");

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
}

/* low level is more sensitivity, except level-0(value 0) */
static void set_pressure_user_level(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	int ret;
	char addr[3] = { 0 };
	char data[2] = { 0 };

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: Touch is stopped!\n",
			  __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;

		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	if ((sec->cmd_param[0] < 1) || (sec->cmd_param[0] > 5))
		goto out_set_user_level;

	/*
	 * byte[0]: m_customlib_ifpacket_addr[7:0]
	 * byte[1]: m_customlib_ifpacket_addr[15:8]
	 * byte[n] : user data (max 32 bytes)
	 */
	addr[0] = SEC_TS_CMD_CUSTOMLIB_OFFSET_PRESSURE_LEVEL;
	addr[1] = 0x00;
	addr[2] = sec->cmd_param[0];

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_CUSTOMLIB_WRITE_PARAM, addr, 3);
	if (ret < 0)
		goto out_set_user_level;

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_CUSTOMLIB_NOTIFY_PACKET, NULL, 0);
	if (ret < 0)
		goto out_set_user_level;

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_CUSTOMLIB_READ_PARAM, addr, 2);
	if (ret < 0)
		goto out_set_user_level;

	ret = ts->sec_ts_read(ts, SEC_TS_CMD_CUSTOMLIB_READ_PARAM, data, 1);
	if (ret < 0)
		goto out_set_user_level;

	input_info(true, &ts->client->dev, "%s: set user level: %d\n",
		   __func__, data[0]);

	ts->pressure_user_level = data[0];

	addr[0] = SEC_TS_CMD_CUSTOMLIB_OFFSET_PRESSURE_THD_HIGH;
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_CUSTOMLIB_READ_PARAM, addr, 2);
	if (ret < 0)
		goto out_set_user_level;

	ret = ts->sec_ts_read(ts, SEC_TS_CMD_CUSTOMLIB_READ_PARAM, data, 1);
	if (ret < 0)
		goto out_set_user_level;

	input_info(true, &ts->client->dev, "%s: HIGH THD: %d\n",
		   __func__, data[0]);

	addr[0] = SEC_TS_CMD_CUSTOMLIB_OFFSET_PRESSURE_THD_LOW;

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_CUSTOMLIB_READ_PARAM, addr, 2);
	if (ret < 0)
		goto out_set_user_level;

	ret = ts->sec_ts_read(ts, SEC_TS_CMD_CUSTOMLIB_READ_PARAM, data, 1);
	if (ret < 0)
		goto out_set_user_level;

	input_info(true, &ts->client->dev, "%s: LOW THD: %d\n",
		   __func__, data[0]);

	snprintf(buff, sizeof(buff), "%s", "OK");

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

out_set_user_level:
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void get_pressure_user_level(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = {0};
	char addr[3] = { 0 };
	char data[2] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	snprintf(buff, sizeof(buff), "%d", ts->pressure_user_level);

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));

	addr[0] = SEC_TS_CMD_CUSTOMLIB_OFFSET_PRESSURE_LEVEL;
	addr[1] = 0x00;

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_CUSTOMLIB_READ_PARAM, addr, 2);
	if (ret < 0)
		goto out_get_user_level;

	ret = ts->sec_ts_read(ts, SEC_TS_CMD_CUSTOMLIB_READ_PARAM, data, 1);
	if (ret < 0)
		goto out_get_user_level;

	input_err(true, &ts->client->dev, "%s: set user level: %d\n",
		  __func__, data[0]);
	ts->pressure_user_level = data[0];

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

out_get_user_level:
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_FAIL;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}
#endif

static void set_lowpower_mode(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	} else {
		snprintf(buff, sizeof(buff), "%s", "OK");
		sec->cmd_state = SEC_CMD_STATUS_OK;
	}

/* set lowpower mode by spay, edge_swipe function.
 *	ts->lowpower_mode = sec->cmd_param[0];
 **/
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));

	sec_cmd_set_cmd_exit(sec);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void set_wirelesscharger_mode(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	int ret;
	bool mode;
	u8 w_data[1] = {0x00};

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 3)
		goto OUT;

	if (sec->cmd_param[0] == 0) {
		ts->charger_mode |= SEC_TS_BIT_CHARGER_MODE_NO;
		mode = false;
	} else {
		ts->charger_mode &= (~SEC_TS_BIT_CHARGER_MODE_NO);
		mode = true;
	}

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			  "%s: fail to enable w-charger status, POWER_STATUS=OFF\n",
			  __func__);
		goto NG;
	}

	if (sec->cmd_param[0] == 1)
		ts->charger_mode = ts->charger_mode |
				SEC_TS_BIT_CHARGER_MODE_WIRELESS_CHARGER;
	else if (sec->cmd_param[0] == 3)
		ts->charger_mode = ts->charger_mode |
				SEC_TS_BIT_CHARGER_MODE_WIRELESS_BATTERY_PACK;
	else if (mode == false)
		ts->charger_mode = ts->charger_mode &
			(~SEC_TS_BIT_CHARGER_MODE_WIRELESS_CHARGER) &
			(~SEC_TS_BIT_CHARGER_MODE_WIRELESS_BATTERY_PACK);

	w_data[0] = ts->charger_mode;
	ret = ts->sec_ts_write(ts, SET_TS_CMD_SET_CHARGER_MODE, w_data, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			  "%s: Failed to send command 74\n", __func__);
		goto NG;
	}

	input_err(true, &ts->client->dev, "%s: %s, status =%x\n",
		__func__, (mode) ? "wireless enable" : "wireless disable",
		ts->charger_mode);

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

NG:
	input_err(true, &ts->client->dev, "%s: %s, status =%x\n",
		__func__, (mode) ? "wireless enable" : "wireless disable",
		ts->charger_mode);

OUT:
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void spay_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1)
		goto NG;

	if (sec->cmd_param[0]) {
		if (ts->use_customlib)
			ts->lowpower_mode |= SEC_TS_MODE_CUSTOMLIB_SPAY;
	} else {
		if (ts->use_customlib)
			ts->lowpower_mode &= ~SEC_TS_MODE_CUSTOMLIB_SPAY;
	}

	input_info(true, &ts->client->dev,
		   "%s: %02X\n", __func__, ts->lowpower_mode);

	#ifdef SEC_TS_SUPPORT_CUSTOMLIB
	if (ts->use_customlib)
		sec_ts_set_custom_library(ts);
	#endif

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

NG:
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void set_aod_rect(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	u8 data[10] = {0x02, 0};
	int ret, i;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	input_info(true, &ts->client->dev, "%s: w: %d, h: %d, x: %d, y: %d\n",
			__func__, sec->cmd_param[0], sec->cmd_param[1],
			sec->cmd_param[2], sec->cmd_param[3]);

	for (i = 0; i < 4; i++) {
		data[i * 2 + 2] = sec->cmd_param[i] & 0xFF;
		data[i * 2 + 3] = (sec->cmd_param[i] >> 8) & 0xFF;
		ts->rect_data[i] = sec->cmd_param[i];
	}

	if (ts->use_customlib) {
		disable_irq(ts->client->irq);
		ret = ts->sec_ts_write(ts, SEC_TS_CMD_CUSTOMLIB_WRITE_PARAM,
				       &data[0], 10);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				  "%s: Failed to write offset\n", __func__);
			goto NG;
		}

		ret = ts->sec_ts_write(ts, SEC_TS_CMD_CUSTOMLIB_NOTIFY_PACKET,
				       NULL, 0);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				  "%s: Failed to send notify\n", __func__);
			goto NG;
		}
		enable_irq(ts->client->irq);
	}

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;
NG:
	enable_irq(ts->client->irq);
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}


static void get_aod_rect(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	u8 data[8] = {0x02, 0};
	u16 rect_data[4] = {0, };
	int ret, i;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->use_customlib) {
		disable_irq(ts->client->irq);
		ret = ts->sec_ts_read_customlib(ts, data, 8);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				  "%s: Failed to read rect\n", __func__);
			goto NG;
		}
		enable_irq(ts->client->irq);
	}

	for (i = 0; i < 4; i++)
		rect_data[i] = (data[i * 2 + 1] & 0xFF) << 8 |
					(data[i * 2] & 0xFF);

	input_info(true, &ts->client->dev, "%s: w: %d, h: %d, x: %d, y: %d\n",
			__func__,
			rect_data[0], rect_data[1], rect_data[2], rect_data[3]);

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;
NG:
	enable_irq(ts->client->irq);
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void aod_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1)
		goto NG;

	if (sec->cmd_param[0]) {
		if (ts->use_customlib)
			ts->lowpower_mode |= SEC_TS_MODE_CUSTOMLIB_AOD;
	} else {
		if (ts->use_customlib)
			ts->lowpower_mode &= ~SEC_TS_MODE_CUSTOMLIB_AOD;
	}

	input_info(true, &ts->client->dev,
		   "%s: %02X\n", __func__, ts->lowpower_mode);

	#ifdef SEC_TS_SUPPORT_CUSTOMLIB
	if (ts->use_customlib)
		sec_ts_set_custom_library(ts);
	#endif

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

NG:
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

/*
 *	flag     1  :  set edge handler
 *		2  :  set (portrait, normal) edge zone data
 *		4  :  set (portrait, normal) dead zone data
 *		8  :  set landscape mode data
 *		16 :  mode clear
 *	data
 *		0x30, FFF (y start), FFF (y end),  FF(direction)
 *		0x31, FFFF (edge zone)
 *		0x32, FF (up x), FF (down x), FFFF (y)
 *		0x33, FF (mode), FFF (edge), FFF (dead zone)
 *	case
 *		edge handler set :  0x30....
 *		booting time :  0x30...  + 0x31...
 *		normal mode : 0x32...  (+0x31...)
 *		landscape mode : 0x33...
 *		landscape -> normal (if same with old data) : 0x33, 0
 *		landscape -> normal (etc) : 0x32....  + 0x33, 0
 */

void set_grip_data_to_ic(struct sec_ts_data *ts, u8 flag)
{
	u8 data[8] = { 0 };

	input_info(true, &ts->client->dev, "%s: flag: %02X (clr,lan,nor,edg,han)\n",
		   __func__, flag);

	if (flag & G_SET_EDGE_HANDLER) {
		if (ts->grip_edgehandler_direction == 0) {
			data[0] = 0x0;
			data[1] = 0x0;
			data[2] = 0x0;
			data[3] = 0x0;
		} else {
			data[0] = (ts->grip_edgehandler_start_y >> 4) & 0xFF;
			data[1] = (ts->grip_edgehandler_start_y << 4 & 0xF0) |
				((ts->grip_edgehandler_end_y >> 8) & 0xF);
			data[2] = ts->grip_edgehandler_end_y & 0xFF;
			data[3] = ts->grip_edgehandler_direction & 0x3;
		}
		ts->sec_ts_write(ts, SEC_TS_CMD_EDGE_HANDLER, data, 4);
		input_info(true, &ts->client->dev, "%s: 0x%02X %02X,%02X,%02X,%02X\n",
			__func__, SEC_TS_CMD_EDGE_HANDLER,
			data[0], data[1], data[2], data[3]);
	}

	if (flag & G_SET_EDGE_ZONE) {
		data[0] = (ts->grip_edge_range >> 8) & 0xFF;
		data[1] = ts->grip_edge_range  & 0xFF;
		ts->sec_ts_write(ts, SEC_TS_CMD_EDGE_AREA, data, 2);
		input_info(true, &ts->client->dev, "%s: 0x%02X %02X,%02X\n",
			__func__, SEC_TS_CMD_EDGE_AREA, data[0], data[1]);
	}

	if (flag & G_SET_NORMAL_MODE) {
		data[0] = ts->grip_deadzone_up_x & 0xFF;
		data[1] = ts->grip_deadzone_dn_x & 0xFF;
		data[2] = (ts->grip_deadzone_y >> 8) & 0xFF;
		data[3] = ts->grip_deadzone_y & 0xFF;
		ts->sec_ts_write(ts, SEC_TS_CMD_DEAD_ZONE, data, 4);
		input_info(true, &ts->client->dev, "%s: 0x%02X %02X,%02X,%02X,%02X\n",
			__func__, SEC_TS_CMD_DEAD_ZONE,
			data[0], data[1], data[2], data[3]);
	}

	if (flag & G_SET_LANDSCAPE_MODE) {
		data[0] = ts->grip_landscape_mode & 0x1;
		data[1] = (ts->grip_landscape_edge >> 4) & 0xFF;
		data[2] = (ts->grip_landscape_edge << 4 & 0xF0) |
				((ts->grip_landscape_deadzone >> 8) & 0xF);
		data[3] = ts->grip_landscape_deadzone & 0xFF;
		ts->sec_ts_write(ts, SEC_TS_CMD_LANDSCAPE_MODE, data, 4);
		input_info(true, &ts->client->dev, "%s: 0x%02X %02X,%02X,%02X,%02X\n",
			__func__, SEC_TS_CMD_LANDSCAPE_MODE,
			data[0], data[1], data[2], data[3]);
	}

	if (flag & G_CLR_LANDSCAPE_MODE) {
		data[0] = ts->grip_landscape_mode;
		ts->sec_ts_write(ts, SEC_TS_CMD_LANDSCAPE_MODE, data, 1);
		input_info(true, &ts->client->dev, "%s: 0x%02X %02X\n",
			__func__, SEC_TS_CMD_LANDSCAPE_MODE, data[0]);
	}
}

/*
 * index  0 :  set edge handler
 *  1 :  portrait (normal) mode
 *  2 :  landscape mode
 *
 * data
 *  0, X (direction), X (y start), X (y end)
 *  direction : 0 (off), 1 (left), 2 (right)
 *	ex) echo set_grip_data,0,2,600,900 > cmd
 *
 *  1, X (edge zone), X (dead zone up x), X (dead zone down x), X (dead zone y)
 *	ex) echo set_grip_data,1,200,10,50,1500 > cmd
 *
 *  2, 1 (landscape mode), X (edge zone), X (dead zone)
 *	ex) echo set_grip_data,2,1,200,100 > cmd
 *
 *2, 0 (portrait mode)
 *	ex) echo set_grip_data,2,0  > cmd
 */

static void set_grip_data(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	/* u8 mode = G_NONE; */
	u8 tPara[2] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	memset(buff, 0, sizeof(buff));

	mutex_lock(&ts->device_mutex);

	tPara[0] = sec->cmd_param[0] & 0xFF;
	tPara[1] = (sec->cmd_param[0] >> 8) & 0xFF;

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_DEADZONE_RANGE, tPara, 2);
	if (ret < 0)
		goto err_grip_data;
/*
 *	if (sec->cmd_param[0] == 0) {	// edge handler
 *		if (sec->cmd_param[1] == 0) {	// clear
 *			ts->grip_edgehandler_direction = 0;
 *		} else if (sec->cmd_param[1] < 3) {
 *			ts->grip_edgehandler_direction = sec->cmd_param[1];
 *			ts->grip_edgehandler_start_y = sec->cmd_param[2];
 *			ts->grip_edgehandler_end_y = sec->cmd_param[3];
 *		} else {
 *			input_err(true, &ts->client->dev,
 *				"%s: cmd1 is abnormal, %d (%d)\n",
 *				__func__, sec->cmd_param[1], __LINE__);
 *			goto err_grip_data;
 *		}
 *
 *		mode = mode | G_SET_EDGE_HANDLER;
 *		set_grip_data_to_ic(ts, mode);
 *
 *	} else if (sec->cmd_param[0] == 1) {	// normal mode
 *		if (ts->grip_edge_range != sec->cmd_param[1])
 *			mode = mode | G_SET_EDGE_ZONE;
 *
 *		ts->grip_edge_range = sec->cmd_param[1];
 *		ts->grip_deadzone_up_x = sec->cmd_param[2];
 *		ts->grip_deadzone_dn_x = sec->cmd_param[3];
 *		ts->grip_deadzone_y = sec->cmd_param[4];
 *		mode = mode | G_SET_NORMAL_MODE;
 *
 *		if (ts->grip_landscape_mode == 1) {
 *			ts->grip_landscape_mode = 0;
 *			mode = mode | G_CLR_LANDSCAPE_MODE;
 *		}
 *		set_grip_data_to_ic(ts, mode);
 *	} else if (sec->cmd_param[0] == 2) {	// landscape mode
 *		if (sec->cmd_param[1] == 0) {	// normal mode
 *			ts->grip_landscape_mode = 0;
 *			mode = mode | G_CLR_LANDSCAPE_MODE;
 *		} else if (sec->cmd_param[1] == 1) {
 *			ts->grip_landscape_mode = 1;
 *			ts->grip_landscape_edge = sec->cmd_param[2];
 *			ts->grip_landscape_deadzone	= sec->cmd_param[3];
 *			mode = mode | G_SET_LANDSCAPE_MODE;
 *		} else {
 *			input_err(true, &ts->client->dev,
 *				"%s: cmd1 is abnormal, %d (%d)\n",
 *				__func__, sec->cmd_param[1], __LINE__);
 *			goto err_grip_data;
 *		}
 *		set_grip_data_to_ic(ts, mode);
 *	} else {
 *		input_err(true, &ts->client->dev,
 *			"%s: cmd0 is abnormal, %d",
 *			__func__, sec->cmd_param[0]);
 *		goto err_grip_data;
 *	}
 **/

	mutex_unlock(&ts->device_mutex);

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

err_grip_data:
	mutex_unlock(&ts->device_mutex);

	snprintf(buff, sizeof(buff), "%s", "NG");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

/*
 * Set/Get Dex Mode 0xE7
 *  0: Disable dex mode
 *  1: Full screen mode
 *  2: Iris mode
 */
static void dex_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (!ts->plat_data->support_dex) {
		input_err(true, &ts->client->dev, "%s: not support DeX mode\n",
			  __func__);
		goto NG;
	}

	if ((sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1) &&
		(sec->cmd_param[1] < 0 || sec->cmd_param[1] > 1)) {
		input_err(true, &ts->client->dev, "%s: not support param\n",
			  __func__);
		goto NG;
	}

	ts->dex_mode = sec->cmd_param[0];
	if (ts->dex_mode) {
		input_err(true, &ts->client->dev, "%s: set DeX touch_pad mode%s\n",
			__func__, sec->cmd_param[1] ? " & Iris mode" : "");
		ts->input_dev = ts->input_dev_pad;
		if (sec->cmd_param[1]) {
			/* Iris mode */
			ts->dex_mode = 0x02;
			ts->dex_name = "[DeXI]";
		} else {
			ts->dex_name = "[DeX]";
		}
	} else {
		input_err(true, &ts->client->dev, "%s: set touch mode\n",
			  __func__);
		ts->input_dev = ts->input_dev_touch;
		ts->dex_name = "";
	}

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: Touch is stopped!\n",
			  __func__);
		goto NG;
	}

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_DEX_MODE, &ts->dex_mode, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
				"%s: failed to set dex %smode\n", __func__,
				sec->cmd_param[1] ? "iris " : "");
		goto NG;
	}

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;

NG:
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void brush_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto out;
	}

	ts->brush_mode = sec->cmd_param[0];

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			"%s: Touch is stopped!\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
		goto out;
	}

	input_info(true, &ts->client->dev,
		"%s: set brush mode %s\n", __func__,
		ts->brush_mode ? "enable" : "disable");

	/* - 0: Disable Artcanvas min phi mode
	 * - 1: Enable Artcanvas min phi mode
	 **/
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_BRUSH_MODE,
				&ts->brush_mode, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
				"%s: failed to set brush mode\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto out;
	}

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
out:
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void force_touch_active(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	int active, ret;
	u16 bus_ref = SEC_TS_BUS_REF_FORCE_ACTIVE;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 2) {
		sec_cmd_set_cmd_result(sec, "NG", 2);
		sec_cmd_set_cmd_exit(sec);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;

		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	/* Specific case for bugreport. */
	if (sec->cmd_param[0] == 2) {
		bus_ref = SEC_TS_BUS_REF_BUGREPORT;
		active = (sec->cmd_param[1]) ? true : false;
		if (active) {
			sec_ts_hc_dump(ts);
			sec_ts_debug_dump(ts);
			ts->bugreport_ktime_start = ktime_get();
		} else {
			ts->bugreport_ktime_start = 0;
		}
	} else {
		active = sec->cmd_param[0];
	}
	input_info(true, &ts->client->dev,
		   "%s: %s %#x\n", __func__, active ? "enable" : "disable",
		   bus_ref);

	if (active)
		pm_stay_awake(&ts->client->dev);
	else
		pm_relax(&ts->client->dev);

	ret = sec_ts_set_bus_ref(ts, bus_ref, active);
	if (ret == 0) {
		sec_cmd_set_cmd_result(sec, "OK", 2);
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		input_info(true, &ts->client->dev,
			"%s: failed! ret %d\n", __func__, ret);
		sec_cmd_set_cmd_result(sec, "NG", 2);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}
	sec_cmd_set_cmd_exit(sec);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void set_touchable_area(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto out;
	}

	ts->touchable_area = sec->cmd_param[0];

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
			"%s: Touch is stopped!\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
		goto out;
	}

	input_info(true, &ts->client->dev,
		"%s: set 16:9 mode %s\n", __func__,
		ts->touchable_area ? "enable" : "disable");

	/*  - 0: Disable 16:9 mode
	 *  - 1: Enable 16:9 mode
	 **/
	ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_TOUCHABLE_AREA,
				&ts->touchable_area, 1);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
				"%s: failed to set 16:9 mode\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto out;
	}

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
out:
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void set_log_level(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	char tBuff[2] = { 0 };
	u8 w_data[1] = {0x00};
	int ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev,
				"%s: Touch is stopped!\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;

		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	if ((sec->cmd_param[0] < 0 || sec->cmd_param[0] > 1) ||
		(sec->cmd_param[1] < 0 || sec->cmd_param[1] > 1) ||
		(sec->cmd_param[2] < 0 || sec->cmd_param[2] > 1) ||
		(sec->cmd_param[3] < 0 || sec->cmd_param[3] > 1) ||
		(sec->cmd_param[4] < 0 || sec->cmd_param[4] > 1) ||
		(sec->cmd_param[5] < 0 || sec->cmd_param[5] > 1) ||
		(sec->cmd_param[6] < 0 || sec->cmd_param[6] > 1) ||
		(sec->cmd_param[7] < 0 || sec->cmd_param[7] > 1)) {
		input_err(true, &ts->client->dev,
				"%s: para out of range\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "Para out of range");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;

		sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
		return;
	}

	ret = ts->sec_ts_read(ts, SEC_TS_CMD_STATUS_EVENT_TYPE, tBuff, 2);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			"%s: Read Event type enable status fail\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "Read Stat Fail");
		goto err;
	}

	input_info(true, &ts->client->dev, "%s: STATUS_EVENT enable = 0x%02X, 0x%02X\n",
		__func__, tBuff[0], tBuff[1]);

	tBuff[0] = BIT_STATUS_EVENT_VENDOR_INFO(sec->cmd_param[6]);
	tBuff[1] = BIT_STATUS_EVENT_ERR(sec->cmd_param[0]) |
			BIT_STATUS_EVENT_INFO(sec->cmd_param[1]) |
			BIT_STATUS_EVENT_USER_INPUT(sec->cmd_param[2]);

	ret = ts->sec_ts_write(ts, SEC_TS_CMD_STATUS_EVENT_TYPE, tBuff, 2);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
			"%s: Write Event type enable status fail\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "Write Stat Fail");
		goto err;
	}

	if (sec->cmd_param[0] == 1 && sec->cmd_param[1] == 1 &&
		sec->cmd_param[2] == 1  && sec->cmd_param[3] == 1 &&
		sec->cmd_param[4] == 1 &&  sec->cmd_param[5] == 1 &&
		sec->cmd_param[6] == 1 && sec->cmd_param[7] == 1) {
		w_data[0] = 0x1;
		ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_VENDOR_EVENT_LEVEL,
				       w_data, 1);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				  "%s: Write Vendor Event Level fail\n",
				  __func__);
			snprintf(buff, sizeof(buff), "%s", "Write Stat Fail");
			goto err;
		}
	} else {
		w_data[0] = 0x0;
		ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_VENDOR_EVENT_LEVEL,
				       w_data, 0);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				"%s: Write Vendor Event Level fail\n",
				__func__);
			snprintf(buff, sizeof(buff), "%s", "Write Stat Fail");
			goto err;
		}
	}

	input_info(true, &ts->client->dev,
		"%s: ERROR : %d, INFO : %d, USER_INPUT : %d, INFO_CUSTOMLIB : %d, VENDOR_INFO : %d, VENDOR_EVENT_LEVEL : %d\n",
		__func__, sec->cmd_param[0], sec->cmd_param[1],
		sec->cmd_param[2], sec->cmd_param[5],
		sec->cmd_param[6], w_data[0]);

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
	return;
err:
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void debug(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };

	sec_cmd_set_default_result(sec);

	ts->debug = sec->cmd_param[0];

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
}

static void set_print_format(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };

	sec_cmd_set_default_result(sec);

	ts->print_format = !!sec->cmd_param[0];

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
}

u8 sec_ts_run_cal_check(struct sec_ts_data *ts)
{
	int rc = 0;
	u8 mis_cal_data = 0xF0;

	if (ts->plat_data->mis_cal_check) {
		rc = sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH,
				      TOUCH_MODE_STATE_TOUCH);
		if (rc < 0)
			input_err(true, &ts->client->dev,
				  "%s: failed#1 ret: %d\n", __func__, rc);

		rc = ts->sec_ts_write(ts, SEC_TS_CMD_MIS_CAL_CHECK,
				      NULL, 0);
		if (rc < 0)
			input_err(true, &ts->client->dev,
				  "%s: failed#2 ret: %d\n", __func__, rc);
		sec_ts_delay(200);

		rc = ts->sec_ts_read(ts, SEC_TS_CMD_MIS_CAL_READ,
				     &mis_cal_data, 1);
		if (rc < 0) {
			input_err(true, &ts->client->dev,
				  "%s: failed#3 ret: %d\n", __func__, rc);
			mis_cal_data = 0xF3;
		} else {
			input_info(true, &ts->client->dev,
				   "%s: mis cal data : %d\n",
				   __func__, mis_cal_data);
		}

		rc = sec_ts_release_tmode(ts);
		if (rc < 0)
			input_err(true, &ts->client->dev,
				  "%s: failed#4 ret: %d\n", __func__, rc);
	} else {
		input_info(true, &ts->client->dev,
			  "%s: not support!\n", __func__);
		mis_cal_data = 0xF1;
	}

	return mis_cal_data;
}

static void run_cal_check(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	u8 ret;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);
	sec_cmd_set_default_result(sec);

	ret = sec_ts_run_cal_check(ts);
	if (ret)
		scnprintf(buff, sizeof(buff), "FAIL(%#x)\n", ret);
	else
		scnprintf(buff, sizeof(buff), "OK\n");

	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

static void not_support_cmd(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	char buff[SEC_CMD_STR_LEN] = { 0 };

	sec_cmd_set_default_result(sec);
	snprintf(buff, sizeof(buff), "%s", "NA");

	sec_cmd_set_cmd_result(sec, buff, strlen(buff));
	sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
	sec_cmd_set_cmd_exit(sec);
}

static void set_touch_mode(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	int ret = 0;
	u8 para[4] = { 0 };

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);
	sec_cmd_set_default_result(sec);

	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: POWER off!\n", __func__);
		goto err_out;
	}

	switch (sec->cmd_param[0]) {
	case 1:
		input_info(true, &ts->client->dev,
			"%s: param = %d, set Normal ACTIVE mode\n",
			__func__, sec->cmd_param[0]);
		sec_ts_fix_tmode(ts,
			TOUCH_SYSTEM_MODE_TOUCH, TOUCH_MODE_STATE_TOUCH);
		break;
	case 2:
		input_info(true, &ts->client->dev,
			"%s: param = %d, set Normal IDLE mode\n",
			__func__, sec->cmd_param[0]);
		sec_ts_fix_tmode(ts,
			TOUCH_SYSTEM_MODE_TOUCH, TOUCH_MODE_STATE_IDLE);
		break;
	case 3:
		input_info(true, &ts->client->dev,
			"%s: param = %d, set Lowpower ACTIVE mode\n",
			__func__, sec->cmd_param[0]);
		sec_ts_fix_tmode(ts,
			TOUCH_SYSTEM_MODE_LOWPOWER, TOUCH_MODE_STATE_TOUCH);
		break;
	case 4:
		input_info(true, &ts->client->dev,
			"%s: param = %d, set Lowpower IDLE mode\n",
			__func__, sec->cmd_param[0]);
		sec_ts_fix_tmode(ts,
			TOUCH_SYSTEM_MODE_LOWPOWER, TOUCH_MODE_STATE_IDLE);
		break;
	case 5:
		input_info(true, &ts->client->dev,
			"%s: param = %d, Sense On\n",
			__func__, sec->cmd_param[0]);
		ret = ts->sec_ts_write(ts,
			SEC_TS_CMD_SENSE_ON, NULL, 0);
		if (ret < 0)
			input_err(true, &ts->client->dev,
				"%s: fail to write Sense_on\n", __func__);
		sec_ts_delay(300);
		break;
	case 6:
		input_info(true, &ts->client->dev,
			"%s: param = %d, Sense Off\n",
			__func__, sec->cmd_param[0]);
		sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_SLEEP,
			TOUCH_MODE_STATE_STOP);
		break;
	case 7:
		input_info(true, &ts->client->dev,
			"%s: param = %d %d, do touch system reset\n",
			__func__, sec->cmd_param[0], sec->cmd_param[1]);
		switch (sec->cmd_param[1]) {
		case RESET_MODE_SW:
			sec_ts_system_reset(ts, RESET_MODE_SW, true, true);
			break;
		case RESET_MODE_HW:
			sec_ts_system_reset(ts, RESET_MODE_HW, true, true);
			break;
		default:
			sec_ts_system_reset(ts, RESET_MODE_AUTO, true, true);
		}
		break;
	case 8:
		input_info(true, &ts->client->dev,
			"%s: Toggle Sense On/Off\n",
			__func__, sec->cmd_param[0]);
		ret = ts->sec_ts_read(ts, SEC_TS_READ_TS_STATUS, para, 4);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				"%s: failed to read status(%d)\n", __func__,
				ret);
			goto err_out;
		}

		if (para[1] == 6) {// have to sense on
			input_info(true, &ts->client->dev,
				"%s: param = %d, Sense On\n",
				__func__, sec->cmd_param[0]);
			ret = ts->sec_ts_write(ts,
				SEC_TS_CMD_SENSE_ON, NULL, 0);
			if (ret < 0)
				input_err(true, &ts->client->dev,
					"%s: fail to write Sense_on\n",
					__func__);

			sec_ts_delay(300);

			input_dbg(false, &ts->client->dev,
				"%s: SENSE ON\n", __func__);
		} else {// have to sense off
			input_info(true, &ts->client->dev,
				"%s: param = %d, Sense Off\n",
				__func__, sec->cmd_param[0]);
			sec_ts_fix_tmode(ts, 0x6, 0x1);
		}

		break;
	default:
		input_info(true, &ts->client->dev,
			"%s: param error! param = %d\n",
			__func__, sec->cmd_param[0]);
		goto err_out;
	}

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);

	return;

err_out:
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

int sec_ts_fn_init(struct sec_ts_data *ts)
{
	int retval;

	retval = sec_cmd_init(&ts->sec, sec_cmds,
			ARRAY_SIZE(sec_cmds), SEC_CLASS_DEVT_TSP);
	if (retval < 0) {
		input_err(true, &ts->client->dev,
			"%s: Failed to sec_cmd_init\n", __func__);
		goto exit;
	}

	retval = sysfs_create_group(&ts->sec.fac_dev->kobj,
			&cmd_attr_group);
	if (retval < 0) {
		input_err(true, &ts->client->dev,
			"%s: Failed to create sysfs attributes\n",
			__func__);
		goto exit;
	}

	retval = sysfs_create_link(&ts->sec.fac_dev->kobj,
				&ts->input_dev->dev.kobj, "input");
	if (retval < 0) {
		input_err(true, &ts->client->dev,
			"%s: Failed to create input symbolic link\n",
			__func__);
		goto exit;
	}

	ts->reinit_done = true;

	return 0;

exit:
	return retval;
}

void sec_ts_fn_remove(struct sec_ts_data *ts)
{
	input_err(true, &ts->client->dev, "%s\n", __func__);

	sysfs_remove_link(&ts->sec.fac_dev->kobj, "input");

	sysfs_remove_group(&ts->sec.fac_dev->kobj,
			   &cmd_attr_group);

	sec_cmd_exit(&ts->sec, SEC_CLASS_DEVT_TSP);

}

int sec_ts_run_rawdata_type(struct sec_ts_data *ts, struct sec_cmd_data *sec)
{
	short min[REGION_TYPE_COUNT], max[REGION_TYPE_COUNT];
	enum spec_check_type spec_check = SPEC_NO_CHECK;
	int i, ii, jj;
	int ret = -1;
	u8 data_type = 0;
	u8 touch_type = sec->cmd_param[1];
	u8 read_type[9] = {TYPE_RAW_DATA, TYPE_AMBIENT_DATA,
		TYPE_DECODED_DATA, TYPE_REMV_AMB_DATA,
		TYPE_SIGNAL_DATA, TYPE_OFFSET_DATA_SEC, TYPE_OFFSET_DATA_SDC,
		TYPE_NOI_P2P_MIN, TYPE_NOI_P2P_MAX};
	const unsigned int buff_size = ts->tx_count * ts->rx_count *
					CMD_RESULT_WORD_LEN;
	unsigned int buff_len = 0;
	char *buff;
	char para = TO_TOUCH_MODE;

#ifdef USE_PRESSURE_SENSOR
	short pressure[3] = { 0 };
	u8 cal_data[18] = { 0 };
#endif

	buff = kzalloc(buff_size, GFP_KERNEL);
	if (!buff)
		goto error_alloc_mem;

	for (i = 0; i < 9; i++) {
		if (read_type[i] == sec->cmd_param[0])
			break;
	}
	if (i == 9) {
		input_err(true, &ts->client->dev, "%s: invalid data type\n",
				__func__);
		goto out;
	}

	ts->tsp_dump_lock = 1;
	input_info(true, &ts->client->dev,
			"%s: start (wet: %d)##\n",
			__func__, ts->wet_mode);


	if (sec->cmd_param[0] == TYPE_OFFSET_DATA_SDC)
		data_type = TYPE_OFFSET_DATA_SDC_NOT_SAVE;
	else
		data_type = (u8)sec->cmd_param[0];

	if (data_type == TYPE_NOI_P2P_MIN
		|| data_type == TYPE_NOI_P2P_MAX) {
		disable_irq(ts->client->irq);

		ret = execute_p2ptest(ts);
		if (ret < 0) {
			input_err(true, &ts->client->dev, "%s: P2P test failed\n",
				__func__);
		}

		ret = ts->sec_ts_write(ts, SEC_TS_CMD_SET_POWER_MODE, &para, 1);
		if (ret < 0) {
			input_err(true, &ts->client->dev,
				"%s: Set powermode failed\n", __func__);
			enable_irq(ts->client->irq);
			goto out;
		}

		enable_irq(ts->client->irq);
	} else {
		ret = sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH,
				TOUCH_MODE_STATE_TOUCH);
		if (ret < 0) {
			input_err(true, &ts->client->dev, "%s: failed to fix tmode\n",
					__func__);
			goto out;
		}
	}

	if (touch_type == 0) {
		ret = sec_ts_read_frame(ts, data_type, min, max,
					&spec_check);

		if (ret < 0)
			input_info(true, &ts->client->dev,
					"%s: mutual %d : error ## ret: %d\n",
					__func__, sec->cmd_param[0], ret);
		else
#ifdef USE_SPEC_CHECK
			input_info(true, &ts->client->dev,
					"%s: mutual %d : Max/Min %d,%d ##\n",
					__func__, sec->cmd_param[0],
					max[0], min[0]);
#else
			input_info(true, &ts->client->dev,
					"%s: mutual %d ##\n",
					__func__, sec->cmd_param[0]);
#endif

		sec_ts_delay(20);

		buff_len += scnprintf(buff + buff_len,
					buff_size - buff_len, "\n");
		if (!ts->print_format) {
			for (ii = 0; ii < (ts->rx_count * ts->tx_count); ii++) {
				buff_len += scnprintf(buff + buff_len,
					buff_size - buff_len,
					"%3d,", ts->pFrame[ii]);
				if (ii % ts->tx_count == (ts->tx_count - 1))
					buff_len += scnprintf(buff + buff_len,
						buff_size - buff_len,
						"\n");
			}
		} else {
			for (ii = 0; ii < ts->tx_count; ii++) {
				for (jj = 0; jj < ts->rx_count; jj++) {
					buff_len += scnprintf(buff + buff_len,
						buff_size - buff_len,
						"%3d,",
						ts->pFrame[(jj * ts->tx_count)
								+ ii]);
				}
				buff_len += scnprintf(buff + buff_len,
						buff_size - buff_len,
						"\n");
			}
		}
	} else if (touch_type > 0) {
		ret = sec_ts_read_channel(ts, data_type, min,
					max, &spec_check);
		if (ret < 0)
			input_info(true, &ts->client->dev,
					"%s: self %d : error ## ret: %d\n",
					__func__, sec->cmd_param[0], ret);
		else
#ifdef USE_SPEC_CHECK
			input_info(true, &ts->client->dev,
					"%s: self %d : Max/Min %d,%d ##\n",
					__func__, sec->cmd_param[0], max[0],
					min[0]);
#else
			input_info(true, &ts->client->dev,
					"%s: self %d ##\n",
					__func__, sec->cmd_param[0]);
#endif

		sec_ts_delay(20);

		buff_len += scnprintf(buff + buff_len,
				buff_size - buff_len, "\n      ");
		if (!ts->print_format) {
			for (ii = 0; ii < (ts->rx_count + ts->tx_count); ii++) {
				buff_len += scnprintf(buff + buff_len,
					buff_size - buff_len,
					"%3d,", ts->pFrame[ii]);
				if (ii >= ts->tx_count - 1)
					buff_len += scnprintf(buff + buff_len,
							buff_size - buff_len,
							"\n");
			}
		} else {
			for (ii = ts->tx_count;
			     ii < (ts->rx_count + ts->tx_count); ii++) {
				buff_len += scnprintf(buff + buff_len,
						buff_size - buff_len,
						"%3d,", ts->pFrame[ii]);
			}
			buff_len += scnprintf(buff + buff_len,
					      buff_size - buff_len, "\n");
			for (ii = 0; ii < ts->tx_count; ii++) {
				buff_len += scnprintf(buff + buff_len,
						buff_size - buff_len,
						"%3d,\n", ts->pFrame[ii]);
			}
		}
	}

#ifdef USE_PRESSURE_SENSOR
	ret = sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH,
			       TOUCH_MODE_STATE_TOUCH);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: failed to fix tmode\n",
				__func__);
		goto out;
	}

	/* run pressure offset data read */
	read_pressure_data(ts, TYPE_OFFSET_DATA_SEC, pressure);
	sec_ts_delay(20);

	/* run pressure rawdata read */
	read_pressure_data(ts, TYPE_RAW_DATA, pressure);
	sec_ts_delay(20);

	/* run pressure raw delta read  */
	read_pressure_data(ts, TYPE_REMV_AMB_DATA, pressure);
	sec_ts_delay(20);

	/* run pressure sigdata read */
	read_pressure_data(ts, TYPE_SIGNAL_DATA, pressure);
	sec_ts_delay(20);

	ret = ts->sec_ts_read(ts, SEC_TS_CMD_SET_GET_PRESSURE, cal_data,
				  18);
	ts->pressure_left = ((cal_data[16] << 8) | cal_data[17]);
	ts->pressure_center = ((cal_data[8] << 8) | cal_data[9]);
	ts->pressure_right = ((cal_data[0] << 8) | cal_data[1]);
	input_info(true, &ts->client->dev, "%s: pressure cal data - Left: %d, Center: %d, Right: %d\n",
			__func__, ts->pressure_left, ts->pressure_center,
			ts->pressure_right);
#endif
	sec_ts_release_tmode(ts);

	ret = sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH,
			       TOUCH_MODE_STATE_TOUCH);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: failed to fix tmode.\n",
			  __func__);
		goto out;
	}

#ifdef USE_STIM_PAD
	sec_ts_read_gain_table(ts);
#endif

	sec_ts_release_tmode(ts);
out:
	input_info(true, &ts->client->dev, "%s: ito: %02X %02X %02X %02X\n",
			__func__, ts->ito_test[0], ts->ito_test[1]
			, ts->ito_test[2], ts->ito_test[3]);

	input_info(true, &ts->client->dev, "%s: done (wet: %d)##\n",
			__func__, ts->wet_mode);
	ts->tsp_dump_lock = 0;

	sec_ts_locked_release_all_finger(ts);

	sec_cmd_set_cmd_result(sec, buff, buff_len);
	kfree(buff);

	return ret;

error_alloc_mem:
	sec_cmd_set_cmd_result(sec, "FAIL", 4);
	return ret;
}

static void run_rawdata_read_type(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[16] = { 0 };
	int ret = -1;

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->tsp_dump_lock == 1) {
		input_err(true, &ts->client->dev, "%s: already checking now\n",
			  __func__);
		scnprintf(buff, sizeof(buff), "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto out;
	}
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: IC is power off\n",
			  __func__);
		scnprintf(buff, sizeof(buff), "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto out;
	}

	ret = sec_ts_run_rawdata_type(ts, sec);
	if (ret < 0) {
		scnprintf(buff, sizeof(buff), "NA");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	} else {
		scnprintf(buff, sizeof(buff), "OK");
		sec->cmd_state = SEC_CMD_STATUS_OK;
	}
out:
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}

/*
 * sec_ts_run_rawdata_all : read all raw data
 *
 * when you want to read full raw data (full_read : true)
 * "mutual/self 3, 5, 29, 1, 19" data will be saved in log
 *
 * otherwise, (full_read : false, especially on boot time)
 * only "mutual 3, 5, 29" data will be saved in log
 */
void sec_ts_run_rawdata_all(struct sec_ts_data *ts, bool full_read)
{
	short min[REGION_TYPE_COUNT], max[REGION_TYPE_COUNT];
	enum spec_check_type spec_check = SPEC_NO_CHECK;
	int ret, i, read_num;
	u8 test_type[5] = {TYPE_AMBIENT_DATA, TYPE_DECODED_DATA,
		TYPE_SIGNAL_DATA, TYPE_OFFSET_DATA_SEC, TYPE_OFFSET_DATA_SDC};
#ifdef USE_PRESSURE_SENSOR
	short pressure[3] = { 0 };
	u8 cal_data[18] = { 0 };
#endif

	ts->tsp_dump_lock = 1;
	input_info(true, &ts->client->dev,
			"%s: start (wet: %d)##\n",
			__func__, ts->wet_mode);

	ret = sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH,
			       TOUCH_MODE_STATE_TOUCH);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: failed to fix tmode\n",
				__func__);
		goto out;
	}

	if (full_read) {
		read_num = 5;
	} else {
		read_num = 3;
		test_type[read_num - 1] = TYPE_OFFSET_DATA_SDC;
	}

	for (i = 0; i < read_num; i++) {
		ret = sec_ts_read_frame(ts, test_type[i], min, max,
					&spec_check);
		if (ret < 0)
			input_info(true, &ts->client->dev,
					"%s: mutual %d : error ## ret: %d\n",
					__func__, test_type[i], ret);
		else
#ifdef USE_SPEC_CHECK
			input_info(true, &ts->client->dev,
					"%s: mutual %d : Max/Min %d,%d ##\n",
					__func__, test_type[i], max[0], min[0]);
#else
			input_info(true, &ts->client->dev,
					"%s: mutual %d ##\n",
					__func__, test_type[i]);
#endif
		sec_ts_delay(20);

		if (full_read) {
			ret = sec_ts_read_channel(ts, test_type[i], min,
						  max, &spec_check);
			if (ret < 0)
				input_info(true, &ts->client->dev,
						"%s: self %d : error ## ret: %d\n",
						__func__, test_type[i], ret);
			else
#ifdef USE_SPEC_CHECK
				input_info(true, &ts->client->dev,
						"%s: self %d : Max/Min %d,%d ##\n",
						__func__, test_type[i], max[0],
						min[0]);
#else
				input_info(true, &ts->client->dev,
						"%s: self %d ##\n",
						__func__, test_type[i]);
#endif
			sec_ts_delay(20);
		}
	}

#ifdef USE_PRESSURE_SENSOR
	ret = sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH,
			       TOUCH_MODE_STATE_TOUCH);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: failed to fix tmode\n",
				__func__);
		goto out;
	}

	/* run pressure offset data read */
	read_pressure_data(ts, TYPE_OFFSET_DATA_SEC, pressure);
	sec_ts_delay(20);

	/* run pressure rawdata read */
	read_pressure_data(ts, TYPE_RAW_DATA, pressure);
	sec_ts_delay(20);

	/* run pressure raw delta read  */
	read_pressure_data(ts, TYPE_REMV_AMB_DATA, pressure);
	sec_ts_delay(20);

	/* run pressure sigdata read */
	read_pressure_data(ts, TYPE_SIGNAL_DATA, pressure);
	sec_ts_delay(20);

	ret = ts->sec_ts_read(ts, SEC_TS_CMD_SET_GET_PRESSURE, cal_data,
				  18);
	ts->pressure_left = ((cal_data[16] << 8) | cal_data[17]);
	ts->pressure_center = ((cal_data[8] << 8) | cal_data[9]);
	ts->pressure_right = ((cal_data[0] << 8) | cal_data[1]);
	input_info(true, &ts->client->dev, "%s: pressure cal data - Left: %d, Center: %d, Right: %d\n",
			__func__, ts->pressure_left, ts->pressure_center,
			ts->pressure_right);
#endif
	sec_ts_release_tmode(ts);

	ret = sec_ts_fix_tmode(ts, TOUCH_SYSTEM_MODE_TOUCH,
			       TOUCH_MODE_STATE_TOUCH);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: failed to fix tmode.\n",
			  __func__);
		goto out;
	}

#ifdef USE_STIM_PAD
	sec_ts_read_gain_table(ts);
#endif

	sec_ts_release_tmode(ts);
out:
	input_info(true, &ts->client->dev, "%s: ito: %02X %02X %02X %02X\n",
			__func__, ts->ito_test[0], ts->ito_test[1]
			, ts->ito_test[2], ts->ito_test[3]);

	input_info(true, &ts->client->dev, "%s: done (wet: %d)##\n",
			__func__, ts->wet_mode);
	ts->tsp_dump_lock = 0;

	sec_ts_locked_release_all_finger(ts);
}

static void run_rawdata_read_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct sec_ts_data *ts = container_of(sec, struct sec_ts_data, sec);
	char buff[16] = { 0 };

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, true);

	sec_cmd_set_default_result(sec);

	if (ts->tsp_dump_lock == 1) {
		input_err(true, &ts->client->dev, "%s: already checking now\n",
			  __func__);
		snprintf(buff, sizeof(buff), "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto out;
	}
	if (ts->power_status == SEC_TS_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: IC is power off\n",
			  __func__);
		snprintf(buff, sizeof(buff), "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto out;
	}

	sec_ts_run_rawdata_all(ts, true);

	snprintf(buff, sizeof(buff), "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;
out:
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	input_info(true, &ts->client->dev, "%s: %s\n", __func__, buff);

	sec_ts_set_bus_ref(ts, SEC_TS_BUS_REF_SYSFS, false);
}
