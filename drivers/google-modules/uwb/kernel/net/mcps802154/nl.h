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
 *
 * 802.15.4 mac common part sublayer, net link definitions.
 */

#ifndef MCPS802154_NL_H
#define MCPS802154_NL_H

#include <linux/types.h>

struct mcps802154_llhw;

#define MCPS802154_NL_RANGING_REQUESTS_MAX 16

struct mcps802154_nl_ranging_request {
	int id;
	int frequency_hz;
	__le64 peer_extended_addr;
	__le64 remote_peer_extended_addr;
};

/**
 * struct mcps802154_nl_ranging_report - Measures report.
 */
struct mcps802154_nl_ranging_report {
	/** @tof_rctu: Time of Flight, or INT_MIN. */
	int tof_rctu;
	/** @local_pdoa_rad_q11: Local Phase Difference Of Arrival, or INT_MIN. */
	int local_pdoa_rad_q11;
	/** @remote_pdoa_rad_q11: Remote Phase Difference  Of Arrival, or INT_MIN. */
	int remote_pdoa_rad_q11;
	/** @local_pdoa_elevation_rad_q11: Local Phase Difference Of Arrival, or INT_MIN. */
	int local_pdoa_elevation_rad_q11;
	/** @remote_pdoa_elevation_rad_q11: Remote Phase Difference Of Arrival, or INT_MIN. */
	int remote_pdoa_elevation_rad_q11;
	/** @is_same_rx_ant: Has azimuth and elevation AoA been done with same antenna? */
	bool is_same_rx_ant;
};

/**
 * mcps802154_nl_ranging_report() - Report a ranging result, called from ranging
 * code.
 * @llhw: Low-level device pointer.
 * @id: Ranging identifier.
 * @report: Phase Difference Of Arrival and Time of Flight.
 *
 * If this returns -ECONNREFUSED, the receiver is not listening anymore, ranging
 * can be stopped.
 *
 * Return: 0 or error.
 */
int mcps802154_nl_ranging_report(
	struct mcps802154_llhw *llhw, int id,
	const struct mcps802154_nl_ranging_report *report);

#ifdef CONFIG_MCPS802154_TESTMODE
/**
 * mcps802154_nl_ping_pong_report() - Report a ping pong result, called from
 * factory tests code.
 * @llhw: Low-level device pointer.
 * @id: ping pong identifier.
 * @t_0: t_0 of ping pong
 * @t_3: t_3 of ping pong
 * @t_4: t_4 of ping pong
 *
 * If this returns -ECONNREFUSED, the receiver is not listening anymore,
 * ping pong can be stopped.
 *
 * Return: 0 or error.
 */
int mcps802154_nl_ping_pong_report(struct mcps802154_llhw *llhw, int id,
				   u64 t_0, u64 t_3, u64 t_4);
#endif

int mcps802154_nl_init(void);
void mcps802154_nl_exit(void);

#endif /* MCPS802154_NL_H */
