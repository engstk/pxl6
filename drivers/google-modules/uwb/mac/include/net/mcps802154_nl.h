/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2021 Qorvo US, Inc.
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

#ifndef NET_MCPS802154_NL_H
#define NET_MCPS802154_NL_H

#define MCPS802154_GENL_NAME "mcps802154"

/**
 * enum mcps802154_commands - MCPS net link commands.
 *
 * @MCPS802154_CMD_GET_HW:
 *	Request information about a device, or dump request to get a list of all
 *	devices.
 * @MCPS802154_CMD_NEW_HW:
 *	Result from information request.
 * @MCPS802154_CMD_SET_SCHEDULER:
 *	Set the scheduler used to manage the schedule.
 * @MCPS802154_CMD_SET_SCHEDULER_PARAMS:
 *	Set the parameters of the current scheduler.
 * @MCPS802154_CMD_SET_SCHEDULER_REGIONS:
 *	Set and configure the scheduler regions.
 * @MCPS802154_CMD_CALL_SCHEDULER:
 *	Call specific scheduler procedure.
 * @MCPS802154_CMD_CALL_REGION:
 *	Call specific region procedure.
 * @MCPS802154_CMD_SET_CALIBRATIONS:
 *	Attempts to write the given value to the indicated calibration item.
 * @MCPS802154_CMD_GET_CALIBRATIONS:
 *	Requests information about a given calibration items.
 * @MCPS802154_CMD_LIST_CALIBRATIONS:
 *	Reports all calibrations existing.
 * @MCPS802154_CMD_TESTMODE:
 *	Run a testmode command with TESTDATA blob attribute to pass through
 *	to the driver.
 * @MCPS802154_CMD_SET_RANGING_REQUESTS:
 *	Set the list of ranging requests.
 * @MCPS802154_CMD_RANGING_REPORT:
 *	Result of ranging.
 * @MCPS802154_CMD_PING_PONG_REPORT:
 *	Result of a ping pong request.
 *
 * @MCPS802154_CMD_UNSPEC: Invalid command.
 * @__MCPS802154_CMD_AFTER_LAST: Internal use.
 * @MCPS802154_CMD_MAX: Internal use.
 */
enum mcps802154_commands {
	MCPS802154_CMD_UNSPEC,

	MCPS802154_CMD_GET_HW, /* can dump */
	MCPS802154_CMD_NEW_HW,

	MCPS802154_CMD_SET_SCHEDULER,
	MCPS802154_CMD_SET_SCHEDULER_PARAMS,
	MCPS802154_CMD_SET_SCHEDULER_REGIONS,

	MCPS802154_CMD_CALL_SCHEDULER,
	MCPS802154_CMD_CALL_REGION,

	MCPS802154_CMD_SET_CALIBRATIONS,
	MCPS802154_CMD_GET_CALIBRATIONS,
	MCPS802154_CMD_LIST_CALIBRATIONS,

	MCPS802154_CMD_TESTMODE,

	/* Temporary ranging interface. */
	MCPS802154_CMD_SET_RANGING_REQUESTS,
	MCPS802154_CMD_RANGING_REPORT,
	MCPS802154_CMD_PING_PONG_REPORT,

	__MCPS802154_CMD_AFTER_LAST,
	MCPS802154_CMD_MAX = __MCPS802154_CMD_AFTER_LAST - 1
};

/**
 * enum mcps802154_attrs - Top level MCPS net link attributes.
 *
 * @MCPS802154_ATTR_HW:
 *	Hardware device index, internal to MCPS.
 * @MCPS802154_ATTR_WPAN_PHY_NAME:
 *	Name of corresponding wpan_phy device.
 * @MCPS802154_ATTR_SCHEDULER_NAME:
 *	Name of the scheduler.
 * @MCPS802154_ATTR_SCHEDULER_PARAMS:
 *	Parameters of the scheduler.
 * @MCPS802154_ATTR_SCHEDULER_REGIONS:
 *	Parameters of the regions.
 * @MCPS802154_ATTR_SCHEDULER_CALL:
 *	Call id of the scheduler's procedure, scheduler specific.
 * @MCPS802154_ATTR_SCHEDULER_CALL_PARAMS:
 *	Parameters of the scheduler's procedure, scheduler specific.
 * @MCPS802154_ATTR_SCHEDULER_REGION_CALL:
 *	Parameters of the region call, scheduler specific.
 * @MCPS802154_ATTR_TESTDATA:
 *	Testmode's data blob, passed through to the driver. It contains
 *	driver-specific attributes.
 * @MCPS802154_ATTR_CALIBRATIONS:
 *	Nested array of calibrations.
 * @MCPS802154_ATTR_RANGING_REQUESTS:
 *	List of ranging requests. This is a nested attribute containing an array
 *	of nested attributes.
 * @MCPS802154_ATTR_RANGING_RESULT:
 *	Ranging result, this is a nested attribute.
 * @MCPS802154_ATTR_PING_PONG_RESULT:
 *	Ping pong result, this is a nested attribute.
 *
 * @MCPS802154_ATTR_UNSPEC: Invalid command.
 * @__MCPS802154_ATTR_AFTER_LAST: Internal use.
 * @MCPS802154_ATTR_MAX: Internal use.
 */
enum mcps802154_attrs {
	MCPS802154_ATTR_UNSPEC,

	MCPS802154_ATTR_HW,
	MCPS802154_ATTR_WPAN_PHY_NAME,

	MCPS802154_ATTR_SCHEDULER_NAME,
	MCPS802154_ATTR_SCHEDULER_PARAMS,
	MCPS802154_ATTR_SCHEDULER_REGIONS,

	MCPS802154_ATTR_SCHEDULER_CALL,
	MCPS802154_ATTR_SCHEDULER_CALL_PARAMS,

	MCPS802154_ATTR_SCHEDULER_REGION_CALL,

	MCPS802154_ATTR_TESTDATA,

	MCPS802154_ATTR_CALIBRATIONS,

	/* Temporary ranging interface. */
	MCPS802154_ATTR_RANGING_REQUESTS,
	MCPS802154_ATTR_RANGING_RESULT,
	MCPS802154_ATTR_PING_PONG_RESULT,

	__MCPS802154_ATTR_AFTER_LAST,
	MCPS802154_ATTR_MAX = __MCPS802154_ATTR_AFTER_LAST - 1
};

/**
 * enum mcps802154_region_attrs - Regions attributes.
 *
 * @MCPS802154_REGION_ATTR_ID:
 *	ID of the region, scheduler specific.
 * @MCPS802154_REGION_ATTR_NAME:
 *	Name of the region, caller specific.
 * @MCPS802154_REGION_ATTR_PARAMS:
 *	Parameters of the region.
 * @MCPS802154_REGION_ATTR_CALL:
 *	Call id of the region's procedure, scheduler specific.
 * @MCPS802154_REGION_ATTR_CALL_PARAMS:
 *	Parameters of the region's procedure, scheduler specific.
 *
 * @MCPS802154_REGION_UNSPEC: Invalid command.
 * @__MCPS802154_REGION_AFTER_LAST: Internal use.
 * @MCPS802154_REGION_MAX: Internal use.
 */
enum mcps802154_region_attrs {
	MCPS802154_REGION_UNSPEC,

	MCPS802154_REGION_ATTR_ID,
	MCPS802154_REGION_ATTR_NAME,
	MCPS802154_REGION_ATTR_PARAMS,
	MCPS802154_REGION_ATTR_CALL,
	MCPS802154_REGION_ATTR_CALL_PARAMS,

	__MCPS802154_REGION_AFTER_LAST,
	MCPS802154_REGION_MAX = __MCPS802154_REGION_AFTER_LAST - 1
};

/**
 * enum mcps802154_ranging_request_attrs - Ranging request.
 *
 * @MCPS802154_RANGING_REQUEST_ATTR_ID:
 *	Request identifier, returned in report.
 * @MCPS802154_RANGING_REQUEST_ATTR_FREQUENCY_HZ:
 *	Ranging frequency in Hz.
 * @MCPS802154_RANGING_REQUEST_ATTR_PEER:
 *	Ranging peer extended address.
 * @MCPS802154_RANGING_REQUEST_ATTR_REMOTE_PEER:
 *	Ranging remote peer extended address.
 *
 * @MCPS802154_RANGING_REQUEST_ATTR_UNSPEC: Invalid command.
 * @__MCPS802154_RANGING_REQUEST_ATTR_AFTER_LAST: Internal use.
 * @MCPS802154_RANGING_REQUEST_ATTR_MAX: Internal use.
 */
enum mcps802154_ranging_request_attrs {
	MCPS802154_RANGING_REQUEST_ATTR_UNSPEC,

	MCPS802154_RANGING_REQUEST_ATTR_ID,
	MCPS802154_RANGING_REQUEST_ATTR_FREQUENCY_HZ,
	MCPS802154_RANGING_REQUEST_ATTR_PEER,
	MCPS802154_RANGING_REQUEST_ATTR_REMOTE_PEER,

	__MCPS802154_RANGING_REQUEST_ATTR_AFTER_LAST,
	MCPS802154_RANGING_REQUEST_ATTR_MAX =
		__MCPS802154_RANGING_REQUEST_ATTR_AFTER_LAST - 1
};

/**
 * enum mcps802154_calibrations_attrs - Calibration item.
 *
 * @MCPS802154_CALIBRATIONS_ATTR_KEY:
 *	Calibration name.
 * @MCPS802154_CALIBRATIONS_ATTR_VALUE:
 *	Calibration value.
 * @MCPS802154_CALIBRATIONS_ATTR_STATUS:
 *	Status in case of error on write or read.
 *
 * @MCPS802154_CALIBRATIONS_ATTR_UNSPEC: Invalid command.
 * @__MCPS802154_CALIBRATIONS_ATTR_AFTER_LAST: Internal use.
 * @MCPS802154_CALIBRATIONS_ATTR_MAX: Internal use.
 */
enum mcps802154_calibrations_attrs {
	MCPS802154_CALIBRATIONS_ATTR_UNSPEC,

	MCPS802154_CALIBRATIONS_ATTR_KEY,
	MCPS802154_CALIBRATIONS_ATTR_VALUE,
	MCPS802154_CALIBRATIONS_ATTR_STATUS,

	__MCPS802154_CALIBRATIONS_ATTR_AFTER_LAST,
	MCPS802154_CALIBRATIONS_ATTR_MAX =
		__MCPS802154_CALIBRATIONS_ATTR_AFTER_LAST - 1
};

/**
 * enum mcps802154_ranging_result_attrs - Ranging result.
 *
 * @MCPS802154_RANGING_RESULT_ATTR_ID:
 *	Identifier of request.
 * @MCPS802154_RANGING_RESULT_ATTR_TOF_RCTU:
 *	Time of flight in RCTU.
 * @MCPS802154_RANGING_RESULT_ATTR_LOCAL_PDOA_RAD_Q11:
 *	Local Phase Difference Of Arrival, unit is multiple of 2048.
 * @MCPS802154_RANGING_RESULT_ATTR_REMOTE_PDOA_RAD_Q11:
 *	Remote Phase Difference Of Arrival, unit is multiple of 2048.
 * @MCPS802154_RANGING_RESULT_ATTR_LOCAL_PDOA_ELEVATION_RAD_Q11:
 *	Local Phase Difference Of Arrival with second pair antenna, unit is multiple of 2048.
 * @MCPS802154_RANGING_RESULT_ATTR_REMOTE_PDOA_ELEVATION_RAD_Q11:
 *	Remote Phase Difference Of Arrival with second pair antenna, unit is multiple of 2048.
 *
 * @MCPS802154_RANGING_RESULT_ATTR_UNSPEC: Invalid command.
 * @__MCPS802154_RANGING_RESULT_ATTR_AFTER_LAST: Internal use.
 * @MCPS802154_RANGING_RESULT_ATTR_MAX: Internal use.
 */
enum mcps802154_ranging_result_attrs {
	MCPS802154_RANGING_RESULT_ATTR_UNSPEC,

	MCPS802154_RANGING_RESULT_ATTR_ID,
	MCPS802154_RANGING_RESULT_ATTR_TOF_RCTU,
	MCPS802154_RANGING_RESULT_ATTR_LOCAL_PDOA_RAD_Q11,
	MCPS802154_RANGING_RESULT_ATTR_REMOTE_PDOA_RAD_Q11,
	MCPS802154_RANGING_RESULT_ATTR_LOCAL_PDOA_ELEVATION_RAD_Q11,
	MCPS802154_RANGING_RESULT_ATTR_REMOTE_PDOA_ELEVATION_RAD_Q11,

	__MCPS802154_RANGING_RESULT_ATTR_AFTER_LAST,
	MCPS802154_RANGING_RESULT_ATTR_MAX =
		__MCPS802154_RANGING_RESULT_ATTR_AFTER_LAST - 1
};

/**
 * enum mcps802154_ping_pong_result_attrs - Ping pong result.
 *
 * @MCPS802154_PING_PONG_RESULT_ATTR_ID:
 *	Identifier of request.
 * @MCPS802154_PING_PONG_RESULT_ATTR_T_0:
 *	t_0 of ping pong
 * @MCPS802154_PING_PONG_RESULT_ATTR_T_3:
 *	t_3 of ping pong.
 * @MCPS802154_PING_PONG_RESULT_ATTR_T_4:
 *	t_4 of ping pong.
 *
 * @MCPS802154_PING_PONG_RESULT_ATTR_UNSPEC: Invalid command.
 * @__MCPS802154_PING_PONG_RESULT_ATTR_AFTER_LAST: Internal use.
 * @MCPS802154_PING_PONG_RESULT_ATTR_MAX: Internal use.
 */
enum mcps802154_ping_pong_result_attrs {
	MCPS802154_PING_PONG_RESULT_ATTR_UNSPEC,

	MCPS802154_PING_PONG_RESULT_ATTR_ID,
	MCPS802154_PING_PONG_RESULT_ATTR_T_0,
	MCPS802154_PING_PONG_RESULT_ATTR_T_3,
	MCPS802154_PING_PONG_RESULT_ATTR_T_4,

	__MCPS802154_PING_PONG_RESULT_ATTR_AFTER_LAST,
	MCPS802154_PING_PONG_RESULT_ATTR_MAX =
		__MCPS802154_PING_PONG_RESULT_ATTR_AFTER_LAST - 1
};

#endif /* NET_MCPS802154_NL_H */
