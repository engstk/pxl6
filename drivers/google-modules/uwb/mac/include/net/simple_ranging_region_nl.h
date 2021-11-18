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

#ifndef SIMPLE_RANGING_REGION_NL_H
#define SIMPLE_RANGING_REGION_NL_H

/**
 * enum simple_ranging_region_set_parameters_attrs - Simple ranging params.
 *
 * @SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_SLOT_DURATION_MS:
 *	Slot duration in milliseconds.
 * @SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_NODE_TYPE:
 *	The node type, either 0 for initiator, or 1 for responder.
 * @SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_TX_ANTENNA:
 *	The antenna index for transmit.
 * @SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_RX_ANTENNA_PAIR_AZIMUTH:
 *	The antenna pair index for receive with azimuth AoA.
 * @SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_RX_ANTENNA_PAIR_ELEVATION:
 *	The antenna pair index for receive with elevation AoA.
 *
 * @SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_UNSPEC: Invalid command.
 * @__SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_AFTER_LAST: Internal use.
 * @SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_MAX: Internal use.
 */
enum simple_ranging_region_set_parameters_attrs {
	SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_UNSPEC,

	SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_SLOT_DURATION_MS,
	SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_NODE_TYPE,
	SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_TX_ANTENNA,
	SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_RX_ANTENNA_PAIR_AZIMUTH,
	SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_RX_ANTENNA_PAIR_ELEVATION,

	__SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_AFTER_LAST,
	SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_MAX =
		__SIMPLE_RANGING_REGION_SET_PARAMETERS_ATTR_AFTER_LAST - 1
};

#endif /* SIMPLE_RANGING_REGION_NL_H */
