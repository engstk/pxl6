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

#ifndef NET_MCPS802154_H
#define NET_MCPS802154_H

#include <net/mac802154.h>
#include <crypto/aes.h>

/* Antenna index to use for transmission by default. */
#define TX_ANT_ID_DEFAULT 0
/* Antenna pair index to use for reception by default. */
#define RX_ANT_PAIR_ID_DEFAULT 0

/** Maximum number of STS segments. */
#define MCPS802154_STS_N_SEGS_MAX 4

/**
 * struct mcps802154_channel - Channel parameters.
 */
struct mcps802154_channel {
	/**
	 * @page: Channel page used in conjunction with channel to uniquely
	 * identify the channel.
	 */
	int page;
	/**
	 * @channel: RF channel to use for all transmissions and receptions.
	 */
	int channel;
	/**
	 * @preamble_code: Preamble code index for HRP UWB. Must be zero for
	 * other PHYs.
	 */
	int preamble_code;
};

/**
 * enum mcps802154_llhw_flags - Low-level hardware without MCPS flags.
 * @MCPS802154_LLHW_RDEV:
 *	Support for ranging (RDEV). TODO: move to &ieee802154_hw.
 * @MCPS802154_LLHW_ERDEV:
 *	Support for enhanced ranging (ERDEV). TODO: move to &ieee802154_hw.
 */
enum mcps802154_llhw_flags {
	MCPS802154_LLHW_RDEV = BIT(0),
	MCPS802154_LLHW_ERDEV = BIT(1),
};

/**
 * struct mcps802154_llhw - Low-level hardware without MCPS.
 *
 * This must be allocated with mcps802154_alloc_llhw(). The low-level driver
 * should then initialize it.
 */
struct mcps802154_llhw {
	/**
	 * @dtu_freq_hz: Inverse of device time unit duration, in Hz.
	 */
	int dtu_freq_hz;
	/**
	 * @symbol_dtu: Symbol duration in device time unit, can change if radio
	 * parameters are changed. Can be set to 1 if device time unit is the
	 * symbol.
	 */
	int symbol_dtu;
	/**
	 * @cca_dtu: CCA duration in device time unit, can change if radio
	 * parameters or CCA modes are changed.
	 */
	int cca_dtu;
	/**
	 * @shr_dtu: Synchronisation header duration in device time unit, can
	 * change if radio parameters are changed. If ranging is supported, this
	 * is the difference between the RMARKER and the first frame symbol.
	 */
	int shr_dtu;
	/**
	 * @dtu_rctu: Duration of one device time unit in ranging counter time
	 * unit (RDEV only).
	 */
	int dtu_rctu;
	/**
	 * @rstu_dtu: Duration of ranging slot time unit in device time unit
	 * (ERDEV only).
	 */
	int rstu_dtu;
	/**
	 * @anticip_dtu: Reasonable delay between reading the current timestamp
	 * and doing an operation in device time unit.
	 */
	int anticip_dtu;
	/**
	 * @idle_dtu: Duration long enough to prefer entering the idle mode
	 * rather than trying to find a valid access.
	 */
	int idle_dtu;
	/**
	 * @current_preamble_code: Current value of preamble code index for HRP
	 * UWB. Must be zero for other PHYs.
	 */
	int current_preamble_code;
	/**
	 * @flags: Low-level hardware flags, see &enum mcps802154_llhw_flags.
	 */
	u32 flags;
	/**
	 * @hw: Pointer to IEEE802154 hardware exposed by MCPS. The low-level
	 * driver needs to update this and hw->phy according to supported
	 * hardware features and radio parameters. More specifically:
	 *
	 * * &ieee802154_hw.extra_tx_headroom
	 * * in &ieee802154_hw.flags:
	 *
	 *     * IEEE802154_HW_TX_OMIT_CKSUM
	 *     * IEEE802154_HW_RX_OMIT_CKSUM
	 *     * IEEE802154_HW_RX_DROP_BAD_CKSUM
	 *
	 * * &wpan_phy.flags
	 * * &wpan_phy_supported
	 * * &wpan_phy.symbol_duration
	 */
	struct ieee802154_hw *hw;
	/**
	 * @priv: Driver private data.
	 */
	void *priv;
};

/**
 * enum mcps802154_tx_frame_info_flags - Flags for transmitting a frame.
 * @MCPS802154_TX_FRAME_TIMESTAMP_DTU:
 *	Start transmission at given timestamp in device time unit.
 * @MCPS802154_TX_FRAME_CCA:
 *	Use CCA before transmission using the programmed mode.
 * @MCPS802154_TX_FRAME_RANGING:
 *	Enable precise timestamping for the transmitted frame and its response
 *	(RDEV only).
 * @MCPS802154_TX_FRAME_KEEP_RANGING_CLOCK:
 *      Request that the ranging clock be kept valid after the transmission of
 *      this frame (RDEV only).
 * @MCPS802154_TX_FRAME_RANGING_PDOA:
 *	Enable phase difference of arrival measurement for the response frame
 *	(RDEV only).
 * @MCPS802154_TX_FRAME_SP1:
 *	Enable STS for the transmitted frame and its response, mode 1 (STS after
 *	SFD and before PHR, ERDEV only).
 * @MCPS802154_TX_FRAME_SP2:
 *	Enable STS for the transmitted frame and its response, mode 2 (STS after
 *	the payload, ERDEV only).
 * @MCPS802154_TX_FRAME_SP3:
 *	Enable STS for the transmitted frame and its response, mode 3 (STS after
 *	SFD, no PHR, no payload, ERDEV only).
 * @MCPS802154_TX_FRAME_STS_MODE_MASK:
 *      Mask covering all the STS mode configuration values.
 *
 * If no timestamp flag is given, transmit as soon as possible.
 */
enum mcps802154_tx_frame_info_flags {
	MCPS802154_TX_FRAME_TIMESTAMP_DTU = BIT(0),
	MCPS802154_TX_FRAME_CCA = BIT(1),
	MCPS802154_TX_FRAME_RANGING = BIT(2),
	MCPS802154_TX_FRAME_KEEP_RANGING_CLOCK = BIT(3),
	MCPS802154_TX_FRAME_RANGING_PDOA = BIT(4),
	MCPS802154_TX_FRAME_SP1 = BIT(5),
	MCPS802154_TX_FRAME_SP2 = BIT(6),
	MCPS802154_TX_FRAME_SP3 = BIT(5) | BIT(6),
	MCPS802154_TX_FRAME_STS_MODE_MASK = BIT(5) | BIT(6),
};

/**
 * struct mcps802154_tx_frame_info - Information for transmitting a frame.
 */
struct mcps802154_tx_frame_info {
	/**
	 * @timestamp_dtu: If timestamped, date of transmission start.
	 */
	u32 timestamp_dtu;
	/**
	 * @rx_enable_after_tx_dtu: If positive, enable receiver this number of
	 * device time unit after the end of the transmitted frame.
	 */
	int rx_enable_after_tx_dtu;
	/**
	 * @rx_enable_after_tx_timeout_dtu: When receiver is enabled after the
	 * end of the transmitted frame: if negative, no timeout, if zero, use
	 * a default timeout value, else this is the timeout value in device
	 * time unit.
	 */
	int rx_enable_after_tx_timeout_dtu;
	/**
	 * @flags: See &enum mcps802154_tx_frame_info_flags.
	 */
	u8 flags;
	/**
	 * @ant_id: antenna index to use for transmit.
	 */
	int ant_id;
};

/**
 * enum mcps802154_rx_info_flags - Flags for enabling the receiver.
 * @MCPS802154_RX_INFO_TIMESTAMP_DTU:
 *	Enable receiver at given timestamp in device time unit.
 * @MCPS802154_RX_INFO_AACK:
 *	Enable automatic acknowledgment.
 * @MCPS802154_RX_INFO_RANGING:
 *	Enable precise timestamping for the received frame (RDEV only).
 * @MCPS802154_RX_INFO_KEEP_RANGING_CLOCK:
 *      Request that the ranging clock be kept valid after the reception of the
 *      frame (RDEV only).
 * @MCPS802154_RX_INFO_RANGING_PDOA:
 *	Enable phase difference of arrival measurement (RDEV only).
 * @MCPS802154_RX_INFO_SP1:
 *	Enable STS for the received frame, mode 1 (STS after SFD and before PHR,
 *	ERDEV only).
 * @MCPS802154_RX_INFO_SP2:
 *	Enable STS for the received frame, mode 2 (STS after the payload, ERDEV
 *	only).
 * @MCPS802154_RX_INFO_SP3:
 *	Enable STS for the received frame, mode 3 (STS after SFD, no PHR, no
 *	payload, ERDEV only).
 * @MCPS802154_RX_INFO_STS_MODE_MASK:
 *      Mask covering all the STS mode configuration values.
 *
 * If no timestamp flag is given, enable receiver as soon as possible.
 */
enum mcps802154_rx_info_flags {
	MCPS802154_RX_INFO_TIMESTAMP_DTU = BIT(0),
	MCPS802154_RX_INFO_AACK = BIT(1),
	MCPS802154_RX_INFO_RANGING = BIT(2),
	MCPS802154_RX_INFO_KEEP_RANGING_CLOCK = BIT(3),
	MCPS802154_RX_INFO_RANGING_PDOA = BIT(4),
	MCPS802154_RX_INFO_SP1 = BIT(5),
	MCPS802154_RX_INFO_SP2 = BIT(6),
	MCPS802154_RX_INFO_SP3 = BIT(5) | BIT(6),
	MCPS802154_RX_INFO_STS_MODE_MASK = BIT(5) | BIT(6),
};

/**
 * struct mcps802154_rx_info - Information for enabling the receiver.
 */
struct mcps802154_rx_info {
	/**
	 * @timestamp_dtu: If timestamped, date to enable the receiver.
	 */
	u32 timestamp_dtu;
	/**
	 * @timeout_dtu: If negative, no timeout, if zero, use a default timeout
	 * value, else this is the timeout value in device time unit.
	 */
	int timeout_dtu;
	/**
	 * @flags: See &enum mcps802154_rx_info_flags.
	 */
	u8 flags;
	/**
	 * @ant_pair_id: Antenna pair index to use for reception.
	 */
	u8 ant_pair_id;
};

/**
 * enum mcps802154_rx_frame_info_flags - Flags for a received frame.
 * @MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU:
 *	Set by MCPS to request timestamp in device time unit.
 * @MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU:
 *	Set by MCPS to request RMARKER timestamp in ranging counter time unit
 *	(RDEV only).
 * @MCPS802154_RX_FRAME_INFO_LQI:
 *	Set by MCPS to request link quality indicator (LQI).
 * @MCPS802154_RX_FRAME_INFO_RSSI:
 *	Set by MCPS to request RSSI.
 * @MCPS802154_RX_FRAME_INFO_RANGING_FOM:
 *	Set by MCPS to request ranging figure of merit (FoM, RDEV only).
 * @MCPS802154_RX_FRAME_INFO_RANGING_OFFSET:
 *	Set by MCPS to request clock characterization data (RDEV only).
 * @MCPS802154_RX_FRAME_INFO_RANGING_PDOA:
 *	Set by MCPS to request phase difference of arrival (RDEV only).
 * @MCPS802154_RX_FRAME_INFO_RANGING_PDOA_FOM:
 *	Set by MCPS to request phase difference of arrival figure of merit (FoM,
 *	RDEV only).
 * @MCPS802154_RX_FRAME_INFO_RANGING_STS_TIMESTAMP_RCTU:
 *	Set by MCPS to request SRMARKERx timestamps for each STS segments in
 *	ranging counter time unit (ERDEV only).
 * @MCPS802154_RX_FRAME_INFO_RANGING_STS_FOM:
 *	Set by MCPS to request STS segments figure of merit measuring the
 *	correlation strength between the received STS segment and the expected
 *	one (FoM, ERDEV only).
 * @MCPS802154_RX_FRAME_INFO_AACK:
 *	Set by low-level driver if an automatic acknowledgment was sent or is
 *	being sent.
 *
 * The low-level driver must clear the corresponding flag if the information is
 * not available.
 */
enum mcps802154_rx_frame_info_flags {
	MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU = BIT(0),
	MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU = BIT(1),
	MCPS802154_RX_FRAME_INFO_LQI = BIT(2),
	MCPS802154_RX_FRAME_INFO_RSSI = BIT(3),
	MCPS802154_RX_FRAME_INFO_RANGING_FOM = BIT(4),
	MCPS802154_RX_FRAME_INFO_RANGING_OFFSET = BIT(5),
	MCPS802154_RX_FRAME_INFO_RANGING_PDOA = BIT(6),
	MCPS802154_RX_FRAME_INFO_RANGING_PDOA_FOM = BIT(7),
	MCPS802154_RX_FRAME_INFO_RANGING_STS_TIMESTAMP_RCTU = BIT(8),
	MCPS802154_RX_FRAME_INFO_RANGING_STS_FOM = BIT(9),
	MCPS802154_RX_FRAME_INFO_AACK = BIT(10),
};

/**
 * struct mcps802154_rx_frame_info - Information on a received frame.
 */
struct mcps802154_rx_frame_info {
	/**
	 * @timestamp_dtu: Timestamp of start of frame in device time unit.
	 */
	u32 timestamp_dtu;
	/**
	 * @timestamp_rctu: Timestamp of RMARKER in ranging count time unit
	 * (RDEV only).
	 */
	u64 timestamp_rctu;
	/**
	 * @frame_duration_dtu: Duration of the whole frame in device time unit
	 * or 0 if unknown.
	 */
	int frame_duration_dtu;
	/**
	 * @rssi: Received signal strength indication (RSSI).
	 */
	int rssi;
	/**
	 * @ranging_tracking_interval_rctu: Interval on which tracking offset
	 * was measured (RDEV only).
	 */
	int ranging_tracking_interval_rctu;
	/**
	 * @ranging_offset_rctu: Difference between the transmitter and the
	 * receiver clock measure over the tracking interval, if positive, the
	 * transmitter operates at a higher frequency (RDEV only).
	 */
	int ranging_offset_rctu;
	/**
	 * @ranging_pdoa_rad_q11: Phase difference of arrival, unit is radian
	 * multiplied by 2048 (RDEV only).
	 */
	int ranging_pdoa_rad_q11;
	/**
	 * @ranging_pdoa_spacing_mm_q11: Spacing between antennas, unit is mm
	 * multiplied by 2048 (RDEV only).
	 */
	int ranging_pdoa_spacing_mm_q11;
	/**
	 * @ranging_aoa_rad_q11: AoA interpolated by the driver from its
	 * calibration LUT. unit is rad multiplied by 2048 (RDEV only).
	 */
	int ranging_aoa_rad_q11;
	/**
	 * @ranging_sts_timestamp_diffs_rctu: For each SRMARKERx, difference
	 * between the measured timestamp and the expected timestamp relative to
	 * RMARKER in ranging count time unit (ERDEV only). When STS mode is
	 * 1 or 3, SRMARKER0 is the same as RMARKER and difference is always 0.
	 */
	s16 ranging_sts_timestamp_diffs_rctu[MCPS802154_STS_N_SEGS_MAX + 1];
	/**
	 * @lqi: Link quality indicator (LQI).
	 */
	u8 lqi;
	/**
	 * @ranging_fom: Ranging figure of merit (FoM, RDEV only). Should be
	 * formatted according to 802.15.4.
	 */
	u8 ranging_fom;
	/**
	 * @ranging_pdoa_fom: Phase difference of arrival figure of merit (FoM,
	 * RDEV only). Range is 0 to 255, with 0 being an invalid measure and
	 * 255 being a 100% confidence.
	 */
	u8 ranging_pdoa_fom;
	/**
	 * @ranging_sts_fom: Table of figures of merit measuring the correlation
	 * strength between the received STS segment and the expected one (FoM,
	 * ERDEV only). Range is 0 to 255, with 0 being an invalid measure and
	 * 255 being a 100% confidence.
	 */
	u8 ranging_sts_fom[MCPS802154_STS_N_SEGS_MAX];
	/**
	 * @flags: See &enum mcps802154_rx_frame_info_flags.
	 */
	u16 flags;
};

/**
 * struct mcps802154_sts_params - STS parameters for HRP UWB.
 */
struct mcps802154_sts_params {
	/**
	 * @v: Value V used in DRBG for generating the STS. The 32 LSB are the
	 * VCounter which is incremented every 128 generated pulse.
	 */
	u8 v[AES_BLOCK_SIZE];
	/**
	 * @key: STS AES key used in DRBG for generating the STS.
	 */
	u8 key[AES_KEYSIZE_128];
	/**
	 * @n_segs: Number of STS segments.
	 */
	int n_segs;
	/**
	 * @seg_len: Length of STS segments.
	 */
	int seg_len;
	/**
	 * @sp2_tx_gap_4chips: For SP2 frame format, additional gap in unit of
	 * 4 chips between the end of the payload and the start of the STS, used
	 * for TX.
	 */
	int sp2_tx_gap_4chips;
	/**
	 * @sp2_rx_gap_4chips: For SP2 frame format, additional gap in unit of
	 * 4 chips between the end of the payload and the start of the STS, used
	 * for RX. A0 and A1 bits in PHR are used to index the array.
	 */
	int sp2_rx_gap_4chips[MCPS802154_STS_N_SEGS_MAX];
};

/**
 * struct mcps802154_ops - Callback from MCPS to the driver.
 */
struct mcps802154_ops {
	/**
	 * @start: Initialize device. Reception should not be activated.
	 *
	 * Return: 0 or error.
	 */
	int (*start)(struct mcps802154_llhw *llhw);
	/**
	 * @stop: Stop device. Should stop any transmission or reception and put
	 * the device in a low power mode.
	 */
	void (*stop)(struct mcps802154_llhw *llhw);
	/**
	 * @tx_frame: Transmit a frame. skb contains the buffer starting from
	 * the IEEE 802.15.4 header. The low-level driver should send the frame
	 * as specified in info. Receiver should be disabled automatically
	 * unless a frame is being received.
	 *
	 * The &frame_idx parameter gives the index of the frame in a "block".
	 * Frames from the same block (aka frame_idx > 0) should maintain the
	 * same synchronization.
	 *
	 * The &next_delay_dtu parameter gives the expected delay between the
	 * start of the transmitted frame and the next action.
	 *
	 * Return: 0, -ETIME if frame can not be sent at specified timestamp,
	 * -EBUSY if a reception is happening right now, or any other error.
	 */
	int (*tx_frame)(struct mcps802154_llhw *llhw, struct sk_buff *skb,
			const struct mcps802154_tx_frame_info *info,
			int frame_idx, int next_delay_dtu);
	/**
	 * @rx_enable: Enable receiver.
	 *
	 * The &frame_idx parameter gives the index of the frame in a "block".
	 * Frames from the same block (aka frame_idx > 0) should maintain the
	 * same synchronization.
	 *
	 * The &next_delay_dtu parameter gives the expected delay between the
	 * start of the received frame or timeout event and the next action.
	 *
	 * Return: 0, -ETIME if receiver can not be enabled at specified
	 * timestamp, or any other error.
	 */
	int (*rx_enable)(struct mcps802154_llhw *llhw,
			 const struct mcps802154_rx_info *info, int frame_idx,
			 int next_delay_dtu);
	/**
	 * @rx_disable: Disable receiver, or a programmed receiver enabling,
	 * unless a frame reception is happening right now.
	 *
	 * Return: 0, -EBUSY if a reception is happening right now, or any other
	 * error.
	 */
	int (*rx_disable)(struct mcps802154_llhw *llhw);
	/**
	 * @rx_get_frame: Get previously received frame. MCPS calls this handler
	 * after a frame reception has been signaled by the low-level driver.
	 *
	 * The received buffer is owned by MCPS after this call. Only the
	 * requested information need to be filled in the information structure.
	 *
	 * Return: 0, -EBUSY if no longer available, or any other error.
	 */
	int (*rx_get_frame)(struct mcps802154_llhw *llhw, struct sk_buff **skb,
			    struct mcps802154_rx_frame_info *info);
	/**
	 * @rx_get_error_frame: Get information on rejected frame. MCPS can call
	 * this handler after a frame rejection has been signaled by the
	 * low-level driver.
	 *
	 * In case of error, info flags must be cleared by this callback.
	 *
	 * Return: 0, -EBUSY if no longer available, or any other error.
	 */
	int (*rx_get_error_frame)(struct mcps802154_llhw *llhw,
				  struct mcps802154_rx_frame_info *info);
	/**
	 * @idle: Put the device into idle mode without time limit or until the
	 * given timestamp.  The driver should call &mcps802154_timer_expired()
	 * before the given timestamp so that an action can be programmed at the
	 * given timestamp.
	 *
	 * The &mcps802154_timer_expired() function must not be called
	 * immediately from this callback, but should be scheduled to be called
	 * later.
	 *
	 * If the driver is late, the regular handling of late actions will take
	 * care of the situation.
	 *
	 * Return: 0 or error.
	 */
	int (*idle)(struct mcps802154_llhw *llhw, bool timestamp,
		    u32 timestamp_dtu);
	/**
	 * @reset: Reset device after an unrecoverable error.
	 *
	 * Return: 0 or error.
	 */
	int (*reset)(struct mcps802154_llhw *llhw);
	/**
	 * @get_current_timestamp_dtu: Get current timestamp in device time
	 * unit.
	 *
	 * If the device is currently in a low power state, the eventual wake up
	 * delay should be added to the returned timestamp.
	 *
	 * If the current timestamp can not be determined precisely, it should
	 * return a pessimistic value, i.e. rounded up.
	 *
	 * Return: 0 or error.
	 */
	int (*get_current_timestamp_dtu)(struct mcps802154_llhw *llhw,
					 u32 *timestamp_dtu);
	/**
	 * @tx_timestamp_dtu_to_rmarker_rctu: Compute the RMARKER timestamp in
	 * ranging counter time unit for a frame transmitted at given timestamp
	 * in device time unit (RDEV only).
	 *
	 * Return: The RMARKER timestamp.
	 */
	u64 (*tx_timestamp_dtu_to_rmarker_rctu)(struct mcps802154_llhw *llhw,
						u32 tx_timestamp_dtu,
						int ant_id);
	/**
	 * @difference_timestamp_rctu: Compute the difference between two
	 * timestamp values.
	 *
	 * Return: The difference between A and B.
	 */
	s64 (*difference_timestamp_rctu)(struct mcps802154_llhw *llhw,
					 u64 timestamp_a_rctu,
					 u64 timestamp_b_rctu);
	/**
	 * @compute_frame_duration_dtu: Compute the duration of a frame with
	 * given payload length (header and checksum included) using the current
	 * radio parameters.
	 *
	 * Return: The duration in device time unit.
	 */
	int (*compute_frame_duration_dtu)(struct mcps802154_llhw *llhw,
					  int payload_bytes);
	/**
	 * @set_channel: Set channel parameters.
	 *
	 * Return: 0 or error.
	 */
	int (*set_channel)(struct mcps802154_llhw *llhw, u8 page, u8 channel,
			   u8 preamble_code);
	/**
	 * @set_hrp_uwb_params: Set radio parameters for HRP UWB.
	 *
	 * The parameters in &mcps802154_llhw can change according to radio
	 * parameters.
	 *
	 * Return: 0 or error.
	 */
	int (*set_hrp_uwb_params)(struct mcps802154_llhw *llhw, int prf,
				  int psr, int sfd_selector, int phr_rate,
				  int data_rate);
	/**
	 * @set_sts_params: Set STS parameters (ERDEV only).
	 *
	 * Return: 0 or error.
	 */
	int (*set_sts_params)(struct mcps802154_llhw *llhw,
			      const struct mcps802154_sts_params *params);
	/**
	 * @set_hw_addr_filt: Set hardware filter parameters.
	 *
	 * Return: 0 or error.
	 */
	int (*set_hw_addr_filt)(struct mcps802154_llhw *llhw,
				struct ieee802154_hw_addr_filt *filt,
				unsigned long changed);
	/**
	 * @set_txpower: Set transmission power.
	 *
	 * Return: 0 or error.
	 */
	int (*set_txpower)(struct mcps802154_llhw *llhw, s32 mbm);
	/**
	 * @set_cca_mode: Set CCA mode.
	 *
	 * The CCA duration in &mcps802154_llhw can change according to CCA
	 * mode.
	 *
	 * Return: 0 or error.
	 */
	int (*set_cca_mode)(struct mcps802154_llhw *llhw,
			    const struct wpan_phy_cca *cca);
	/**
	 * @set_cca_ed_level: Set CCA energy detection threshold.
	 *
	 * Return: 0 or error.
	 */
	int (*set_cca_ed_level)(struct mcps802154_llhw *llhw, s32 mbm);
	/**
	 * @set_promiscuous_mode: Set promiscuous mode.
	 *
	 * Return: 0 or error.
	 */
	int (*set_promiscuous_mode)(struct mcps802154_llhw *llhw, bool on);
	/**
	 * @set_scanning_mode: Set SW scanning mode.
	 *
	 * Return: 0 or error.
	 */
	int (*set_scanning_mode)(struct mcps802154_llhw *llhw, bool on);
	/**
	 * @set_calibration: Set calibration value.
	 *
	 * Set the calibration parameter specified by the key string with the
	 * value specified in the provided buffer. The provided length must
	 * match the length returned by the @get_calibration() callback.
	 *
	 * Return: 0 or error.
	 */
	int (*set_calibration)(struct mcps802154_llhw *llhw, const char *key,
			       void *value, size_t length);
	/**
	 * @get_calibration: Get calibration value.
	 *
	 * Get the calibration parameter specified by the key string into the
	 * provided buffer.
	 *
	 * Return: size of parameter written in buffer or error.
	 */
	int (*get_calibration)(struct mcps802154_llhw *llhw, const char *key,
			       void *value, size_t length);
	/**
	 * @list_calibration: Returns list of accepted calibration key strings
	 *
	 * Return: NULL terminated strings pointer array.
	 */
	const char *const *(*list_calibration)(struct mcps802154_llhw *llhw);
	/**
	 * @vendor_cmd: Run a vendor specific command.
	 *
	 * Do not (ab)use this feature to implement features that could be
	 * openly shared across drivers.
	 *
	 * Return: 0 or error.
	 */
	int (*vendor_cmd)(struct mcps802154_llhw *llhw, u32 vendor_id,
			  u32 subcmd, void *data, size_t data_len);
#ifdef CONFIG_MCPS802154_TESTMODE
	/**
	 * @testmode_cmd: Run a testmode command.
	 *
	 * Return: 0 or error.
	 */
	int (*testmode_cmd)(struct mcps802154_llhw *llhw, void *data, int len);
#endif
};

#ifdef CONFIG_MCPS802154_TESTMODE
#define MCPS802154_TESTMODE_CMD(cmd) .testmode_cmd = (cmd),
#else
#define MCPS802154_TESTMODE_CMD(cmd)
#endif

/**
 * enum mcps802154_rx_error_type - Type of reception errors.
 * @MCPS802154_RX_ERROR_NONE:
 *      RX successful.
 * @MCPS802154_RX_ERROR_TIMEOUT:
 *      RX timeout.
 * @MCPS802154_RX_ERROR_BAD_CKSUM:
 *	Checksum is not correct.
 * @MCPS802154_RX_ERROR_UNCORRECTABLE:
 *	During reception, the error correction code detected an uncorrectable
 *	error.
 * @MCPS802154_RX_ERROR_FILTERED:
 *	A received frame was rejected due to frame filter.
 * @MCPS802154_RX_ERROR_SFD_TIMEOUT:
 *	A preamble has been detected but no SFD.
 * @MCPS802154_RX_ERROR_OTHER:
 *	Other error, frame reception is aborted.
 */
enum mcps802154_rx_error_type {
	MCPS802154_RX_ERROR_NONE = 0,
	MCPS802154_RX_ERROR_TIMEOUT = 1,
	MCPS802154_RX_ERROR_BAD_CKSUM = 2,
	MCPS802154_RX_ERROR_UNCORRECTABLE = 3,
	MCPS802154_RX_ERROR_FILTERED = 4,
	MCPS802154_RX_ERROR_SFD_TIMEOUT = 5,
	MCPS802154_RX_ERROR_OTHER = 6,
};

/**
 * mcps802154_alloc_llhw() - Allocate a new low-level hardware device.
 * @priv_data_len: Length of private data.
 * @ops: Callbacks for this device.
 *
 * Return: A pointer to the new low-level hardware device, or %NULL on error.
 */
struct mcps802154_llhw *mcps802154_alloc_llhw(size_t priv_data_len,
					      const struct mcps802154_ops *ops);

/**
 * mcps802154_free_llhw() - Free low-level hardware descriptor.
 * @llhw: Low-level device pointer.
 *
 * You must call mcps802154_unregister_hw() before calling this function.
 */
void mcps802154_free_llhw(struct mcps802154_llhw *llhw);

/**
 * mcps802154_register_llhw() - Register low-level hardware device.
 * @llhw: Low-level device pointer.
 *
 * Return: 0 or error.
 */
int mcps802154_register_llhw(struct mcps802154_llhw *llhw);

/**
 * mcps802154_unregister_llhw() - Unregister low-level hardware device.
 * @llhw: Low-level device pointer.
 */
void mcps802154_unregister_llhw(struct mcps802154_llhw *llhw);

/**
 * mcps802154_rx_frame() - Signal a frame reception.
 * @llhw: Low-level device this frame came in on.
 *
 * The MCPS will call the &mcps802154_ops.rx_get_frame() handler to retrieve
 * frame.
 */
void mcps802154_rx_frame(struct mcps802154_llhw *llhw);

/**
 * mcps802154_rx_timeout() - Signal a reception timeout.
 * @llhw: Low-level device pointer.
 */
void mcps802154_rx_timeout(struct mcps802154_llhw *llhw);

/**
 * mcps802154_rx_error() - Signal a reception error.
 * @llhw: Low-level device pointer.
 * @error: Type of detected error.
 *
 * In case of filtered frame, the MCPS can call the
 * &mcps802154_ops.rx_get_error_frame() handler to retrieve frame information.
 */
void mcps802154_rx_error(struct mcps802154_llhw *llhw,
			 enum mcps802154_rx_error_type error);

/**
 * mcps802154_tx_done() - Signal the end of an MCPS transmission.
 * @llhw: Low-level device pointer.
 */
void mcps802154_tx_done(struct mcps802154_llhw *llhw);

/**
 * mcps802154_broken() - Signal an unrecoverable error, device needs to be
 * reset.
 * @llhw: Low-level device pointer.
 */
void mcps802154_broken(struct mcps802154_llhw *llhw);

/**
 * mcps802154_timer_expired() - Signal that a programmed timer expired.
 * @llhw: Low-level device pointer.
 *
 * To be called before the timestamp given to &mcps802154_ops.idle() callback.
 */
void mcps802154_timer_expired(struct mcps802154_llhw *llhw);

#ifdef CONFIG_MCPS802154_TESTMODE
/**
 * mcps802154_testmode_alloc_reply_skb() - Allocate testmode reply.
 * @llhw: Low-level device pointer.
 * @approxlen: an upper bound of the length of the data that will
 * be put into the skb.
 *
 * This function allocates and pre-fills an skb for a reply to
 * the testmode command. Since it is intended for a reply, calling
 * it outside of the @testmode_cmd operation is invalid.
 *
 * The returned skb is pre-filled with the netlink message's header
 * and attribute's data and set up in a way that any data that is
 * put into the skb (with skb_put(), nla_put() or similar) will end up
 * being within the %MCPS802154_ATTR_TESTDATA attribute, so all
 * that needs to be done with the skb is adding data for
 * the corresponding userspace tool which can then read that data
 * out of the testdata attribute. You must not modify the skb
 * in any other way.
 *
 * When done, call mcps802154_testmode_reply() with the skb and return
 * its error code as the result of the @testmode_cmd operation.
 *
 * Return: An allocated and pre-filled skb. %NULL if any errors happen.
 */
struct sk_buff *
mcps802154_testmode_alloc_reply_skb(struct mcps802154_llhw *llhw,
				    int approxlen);

/**
 * mcps802154_testmode_reply() - Send the reply skb.
 * @llhw: Low-level device pointer.
 * @skb: The skb, must have been allocated with
 * mcps802154_testmode_alloc_reply_skb().
 *
 * Since calling this function will usually be the last thing
 * before returning from the @testmode_cmd you should return
 * the error code.  Note that this function consumes the skb
 * regardless of the return value.
 *
 * Return: 0 or error.
 */
int mcps802154_testmode_reply(struct mcps802154_llhw *llhw,
			      struct sk_buff *skb);
#endif

#endif /* NET_MCPS802154_H */
