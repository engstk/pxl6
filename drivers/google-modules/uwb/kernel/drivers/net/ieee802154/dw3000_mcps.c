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
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/pm_runtime.h>
#include <linux/bitfield.h>
#include <net/mcps802154.h>

#include "dw3000.h"
#include "dw3000_pm.h"
#include "dw3000_core.h"
#include "dw3000_mcps.h"
#include "dw3000_testmode.h"
#include "dw3000_trc.h"

#define sts_to_pdoa(s) ((s) ? DW3000_PDOA_M1 : DW3000_PDOA_M0)

static inline u64 timestamp_dtu_to_rctu(struct mcps802154_llhw *llhw,
					u32 timestamp_dtu);

static inline u32 timestamp_rctu_to_dtu(struct mcps802154_llhw *llhw,
					u64 timestamp_rctu);

static inline int dtu_to_pac(struct mcps802154_llhw *llhw, int timeout_dtu)
{
	struct dw3000 *dw = llhw->priv;

	return (timeout_dtu * DW3000_CHIP_PER_DTU + dw->chips_per_pac - 1) /
	       dw->chips_per_pac;
}

static inline int dtu_to_dly(struct mcps802154_llhw *llhw, int dtu)
{
	return (dtu * DW3000_CHIP_PER_DTU / DW3000_CHIP_PER_DLY);
}

static inline int rctu_to_dly(struct mcps802154_llhw *llhw, int rctu)
{
	return (rctu / DW3000_RCTU_PER_CHIP / DW3000_CHIP_PER_DLY);
}

static inline u32 tx_rmarker_offset(struct dw3000 *dw, int ant_id)
{
	struct dw3000_config *config = &dw->config;
	const struct dw3000_antenna_calib *ant_calib;
	const struct dw3000_antenna_calib_prf *ant_calib_prf;
	int chanidx;
	int prfidx;

	if (ant_id >= ANTMAX || ant_id < 0) {
		dev_err(dw->dev, "ant_id %d is out of antenna range, max is %d",
			ant_id, ANTMAX);
		return 0;
	}
	/* Current configured ant_id. */
	if (ant_id == config->ant[0])
		return config->rmarkerOffset;

	ant_calib = &dw->calib_data.ant[ant_id];

	chanidx = config->chan == 9 ? DW3000_CALIBRATION_CHANNEL_9 :
				      DW3000_CALIBRATION_CHANNEL_5;
	prfidx = config->txCode >= 9 ? DW3000_CALIBRATION_PRF_64MHZ :
				       DW3000_CALIBRATION_PRF_16MHZ;

	ant_calib_prf = &ant_calib->ch[chanidx].prf[prfidx];

	return ant_calib_prf->ant_delay;
}

static int do_set_hw_addr_filt(struct dw3000 *dw, const void *in, void *out);
static int do_set_promiscuous_mode(struct dw3000 *dw, const void *in,
				   void *out);

static int do_start(struct dw3000 *dw, const void *in, void *out)
{
#if (KERNEL_VERSION(4, 13, 0) <= LINUX_VERSION_CODE)
	struct spi_controller *ctlr = dw->spi->controller;
#else
	struct spi_master *ctlr = dw->spi->master;
#endif
	const unsigned long changed = (unsigned long)-1;
	int rc;

	dw3000_pm_qos_update_request(dw, dw3000_qos_latency);

	/* Lock power management of SPI controller */
	rc = pm_runtime_get_sync(ctlr->dev.parent);
	if (rc < 0) {
		pm_runtime_put_noidle(ctlr->dev.parent);
		dev_err(&ctlr->dev, "Failed to power device: %d\n", rc);
	}
	dw->has_lock_pm = !rc;

	/* Since device is power-off when interface is down, we need to redo hard
	 * and soft resets to ensure good state of the device */

	/* Turn on power (with RST GPIO) */
	rc = dw3000_poweron(dw);
	if (rc) {
		dev_err(dw->dev, "device power on failed: %d\n", rc);
		return rc;
	}

	rc = dw3000_hardreset(dw);
	if (rc) {
		dev_err(dw->dev, "hard reset failed: %d\n", rc);
		return rc;
	}

	/* Soft reset */
	rc = dw3000_softreset(dw);
	if (rc) {
		dev_err(dw->dev, "device reset failed: %d\n", rc);
		return rc;
	}

	/* Initialize & configure the device */
	rc = dw3000_init(dw, true);
	if (rc) {
		dev_err(dw->dev, "device init failed: %d\n", rc);
		return rc;
	}

	/* Configure antenna selection GPIO if any */
	rc = dw3000_config_antenna_gpios(dw);
	if (rc)
		return rc;

	/* Apply other configuration not done by dw3000_init() */
	rc = do_set_hw_addr_filt(dw, &changed, NULL);
	if (rc)
		return rc;

	rc = do_set_promiscuous_mode(dw, NULL, NULL);
	if (rc)
		return rc;

	/* Enable the device */
	return dw3000_enable(dw);
}

static int start(struct mcps802154_llhw *llhw)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_stm_command cmd = { do_start, NULL, NULL };
	int ret;

	trace_dw3000_mcps_start(dw);
	ret = dw3000_enqueue_generic(dw, &cmd);
	trace_dw3000_return_int(dw, ret);
	return ret;
}

static int do_stop(struct dw3000 *dw, const void *in, void *out)
{
	int rc;

	/* Disable the device */
	rc = dw3000_disable(dw);
	if (rc)
		dev_warn(dw->dev, "device disable failed: %d\n", rc);

	/* Power-off */
	rc = dw3000_poweroff(dw);
	if (rc)
		dev_err(dw->dev, "device power-off failed: %d\n", rc);

	/* Unlock power management of SPI controller */
	if (dw->has_lock_pm) {
#if (KERNEL_VERSION(4, 13, 0) <= LINUX_VERSION_CODE)
		struct spi_controller *ctlr = dw->spi->controller;
#else
		struct spi_master *ctlr = dw->spi->master;
#endif
		pm_runtime_put(ctlr->dev.parent);
		dw->has_lock_pm = false;
	}

	dw3000_pm_qos_update_request(dw, FREQ_QOS_MAX_DEFAULT_VALUE);
	return 0;
}

static void stop(struct mcps802154_llhw *llhw)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_stm_command cmd = { do_stop, NULL, NULL };

	trace_dw3000_mcps_stop(dw);
	dw3000_enqueue_generic(dw, &cmd);
	trace_dw3000_return_void(dw);
}

struct do_tx_frame_params {
	struct sk_buff *skb;
	const struct mcps802154_tx_frame_info *info;
};

static int do_tx_frame(struct dw3000 *dw, const void *in, void *out)
{
	struct do_tx_frame_params *params = (struct do_tx_frame_params *)in;
	const struct mcps802154_tx_frame_info *info = params->info;
	struct mcps802154_llhw *llhw = dw->llhw;
	u32 tx_date_dtu = 0;
	int rx_delay_dly = -1;
	u32 rx_timeout_pac = 0;
	bool tx_delayed = true;
	int rc;
	u8 sts_mode;

	/* Check data : no data if SP3, must have data otherwise */
	if (((info->flags & MCPS802154_TX_FRAME_STS_MODE_MASK) ==
	     MCPS802154_TX_FRAME_SP3) != !params->skb)
		return -EINVAL;

	/* Enable STS */
	sts_mode = FIELD_GET(MCPS802154_TX_FRAME_STS_MODE_MASK, info->flags);
	rc = dw3000_set_sts(dw, sts_mode);
	if (unlikely(rc))
		return rc;
	rc = dw3000_set_pdoa(dw, sts_to_pdoa(sts_mode));
	if (unlikely(rc))
		return rc;
	/* Ensure correct TX antenna is selected. */
	rc = dw3000_set_tx_antenna(dw, info->ant_id);
	if (unlikely(rc))
		return rc;
	/* Calculate the transfer date.*/
	if (info->flags & MCPS802154_TX_FRAME_TIMESTAMP_DTU) {
		tx_date_dtu = info->timestamp_dtu + llhw->shr_dtu;
	} else {
		/* Send immediately. */
		tx_delayed = false;
	}

	if (info->rx_enable_after_tx_dtu > 0) {
		/* Disable auto-ack if it was previously enabled. */
		dw3000_disable_autoack(dw, false);
		/* Calculate the after tx rx delay. */
		rx_delay_dly = dtu_to_dly(llhw, info->rx_enable_after_tx_dtu) -
			       DW3000_RX_ENABLE_STARTUP_DLY;
		rx_delay_dly = rx_delay_dly >= 0 ? rx_delay_dly : 0;
		/* Calculate the after tx rx timeout. */
		if (info->rx_enable_after_tx_timeout_dtu == 0) {
			rx_timeout_pac = dw->pre_timeout_pac;
		} else if (info->rx_enable_after_tx_timeout_dtu != -1) {
			rx_timeout_pac =
				dw->pre_timeout_pac +
				dtu_to_pac(
					llhw,
					info->rx_enable_after_tx_timeout_dtu);
		} else {
			/* No timeout. */
		}
	}
	return dw3000_tx_frame(dw, params->skb, tx_delayed, tx_date_dtu,
			       rx_delay_dly, rx_timeout_pac);
}

static int tx_frame(struct mcps802154_llhw *llhw, struct sk_buff *skb,
		    const struct mcps802154_tx_frame_info *info,
		    int next_delay_dtu)
{
	struct dw3000 *dw = llhw->priv;
	struct do_tx_frame_params params = { skb, info };
	struct dw3000_stm_command cmd = { do_tx_frame, &params, NULL };
	int ret;

	trace_dw3000_mcps_tx_frame(dw, skb ? skb->len : 0);
	ret = dw3000_enqueue_generic(dw, &cmd);
	trace_dw3000_return_int(dw, ret);
	return ret;
}

static int do_rx_enable(struct dw3000 *dw, const void *in, void *out)
{
	struct mcps802154_rx_info *info = (struct mcps802154_rx_info *)(in);
	struct mcps802154_llhw *llhw = dw->llhw;
	u32 date_dtu = 0;
	u32 timeout_pac = 0;
	bool rx_delayed = true;
	int rc;
	u8 sts_mode;

	/* Enable STS */
	sts_mode = FIELD_GET(MCPS802154_RX_INFO_STS_MODE_MASK, info->flags);
	rc = dw3000_set_sts(dw, sts_mode);
	if (unlikely(rc))
		return rc;
	rc = dw3000_set_pdoa(dw, sts_to_pdoa(sts_mode));
	if (unlikely(rc))
		return rc;
	/* Ensure correct RX antenna are selected. */
	rc = dw3000_set_rx_antennas(dw, info->ant_pair_id);
	if (unlikely(rc))
		return rc;
	/* Calculate the transfer date. */
	if (info->flags & MCPS802154_RX_INFO_TIMESTAMP_DTU) {
		date_dtu = info->timestamp_dtu - DW3000_RX_ENABLE_STARTUP_DTU;
	} else {
		/* Receive immediately. */
		rx_delayed = false;
	}

	if (info->flags & MCPS802154_RX_INFO_AACK) {
		dw3000_enable_autoack(dw, false);
	} else {
		dw3000_disable_autoack(dw, false);
	}

	/* Calculate the timeout. */
	if (info->timeout_dtu == 0) {
		timeout_pac = dw->pre_timeout_pac;
	} else if (info->timeout_dtu != -1) {
		timeout_pac = dw->pre_timeout_pac +
			      dtu_to_pac(llhw, info->timeout_dtu);
	} else {
		/* No timeout. */
	}
	return dw3000_rx_enable(dw, rx_delayed, date_dtu, timeout_pac);
}

static int do_rx_disable(struct dw3000 *dw, const void *in, void *out)
{
	return dw3000_rx_disable(dw);
}

static int rx_enable(struct mcps802154_llhw *llhw,
		     const struct mcps802154_rx_info *info, int next_delay_dtu)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_stm_command cmd = { do_rx_enable, (void *)info, NULL };
	int ret;

	trace_dw3000_mcps_rx_enable(dw, info->flags, info->timeout_dtu);
	ret = dw3000_enqueue_generic(dw, &cmd);
	trace_dw3000_return_int(dw, ret);
	return ret;
}

static int rx_disable(struct mcps802154_llhw *llhw)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_stm_command cmd = { do_rx_disable, NULL, NULL };
	int ret;

	trace_dw3000_mcps_rx_disable(dw);
	ret = dw3000_enqueue_generic(dw, &cmd);
	trace_dw3000_return_int(dw, ret);
	return ret;
}

/**
 * get_ranging_pdoa_fom() - compute the figure of merit of the PDoA.
 * @sts_fom: STS FoM on a received frame.
 *
 * If the STS FoM is less than sts_fom_threshold, PDoA FoM is 1, the worst.
 * If the STS FoM is greater or equal than sts_fom_threshold,
 * sts_fom_threshold to 255 values are mapped to 2 to 255.
 *
 * Return: the PDoA FoM value.
 */
static u8 get_ranging_pdoa_fom(u8 sts_fom)
{
	/* For a normalized STS FoM in 0 to 255, the STS is not reliable if
	 * the STS FoM is less than 60 percents of its maximum value.
	 */
	static const int sts_fom_threshold = 153;
	/* sts_fom_threshold .. sts_fom_max values are mapped to pdoa_fom_min .. pdoa_fom_max.
	 * The relation is pdoa_fom = a * sts_fom + b, with
	 * pdoa_fom_min =  sts_fom_threshold * a + b
	 * pdoa_fom_max = sts_fom_max * a + b
	 * So:
	 * a = (pdoa_fom_max - pdoa_fom_min) / (sts_fom_max - sts_fom_threshold)
	 * b = pdoa_fom_min - sts_fom_threshold * a
	 */
	static const int sts_fom_max = 255;
	static const int pdoa_fom_min = 2;
	static const int pdoa_fom_max = 255;
	static const int a_numerator = pdoa_fom_max - pdoa_fom_min;
	static const int a_denominator = sts_fom_max - sts_fom_threshold;
	static const int b = pdoa_fom_min - ((sts_fom_threshold * a_numerator) /
					     a_denominator);

	if (sts_fom < sts_fom_threshold)
		return 1;
	return ((a_numerator * sts_fom) / a_denominator) + b;
}

static int get_ranging_sts_fom(struct mcps802154_llhw *llhw,
			       struct mcps802154_rx_frame_info *info)
{
	int ret;
	struct dw3000 *dw = llhw->priv;
	struct dw3000_config *config = &dw->config;
	/* Max sts_acc_qual value depend on STS length */
	int sts_acc_max = DW3000_GET_STS_LEN_UNIT_VALUE(config->stsLength) * 8;
	s16 sts_acc_qual;

	/* TODO: Reading TOAST disabled. According to hardware team,
	 * this needs more tuning. They suggest to use quality only for
	 * now. See UWB-940 and commit "disable TOAST quality checking
	 * for STS". */

	ret = dw3000_read_sts_quality(dw, &sts_acc_qual);
	if (ret)
		return ret;
	/* DW3000 only support one STS segment. */
	info->ranging_sts_fom[0] =
		clamp(1 + sts_acc_qual * 254 / sts_acc_max, 1, 255);
	return ret;
}

static int rx_get_frame(struct mcps802154_llhw *llhw, struct sk_buff **skb,
			struct mcps802154_rx_frame_info *info)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_config *config = &dw->config;
	struct dw3000_rx *rx = &dw->rx;
	unsigned long flags;
	u64 timestamp_rctu;
	int ret = 0;
	u8 rx_flags;

	trace_dw3000_mcps_rx_get_frame(dw, info->flags);

	/* Sanity check parameters */
	if (unlikely(!info || !skb)) {
		ret = -EINVAL;
		goto error;
	}

	/* Acquire RX lock */
	spin_lock_irqsave(&rx->lock, flags);
	/* Check buffer available */
	if (unlikely(!rx->skb && !(rx->flags & DW3000_RX_FLAG_ND))) {
		spin_unlock_irqrestore(&rx->lock, flags);
		ret = -EAGAIN;
		goto error;
	}
	/* Give the last received frame we stored */
	*skb = rx->skb;
	rx->skb = NULL;
	rx_flags = rx->flags;
	timestamp_rctu = rx->ts_rctu;
	/* Release RX lock */
	spin_unlock_irqrestore(&rx->lock, flags);

	if (info->flags & (MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU |
			   MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU)) {
		if (!(rx_flags & DW3000_RX_FLAG_TS))
			info->flags &= ~(
				MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU |
				MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU |
				MCPS802154_RX_FRAME_INFO_RANGING_STS_TIMESTAMP_RCTU);
		else {
			info->timestamp_rctu =
				timestamp_rctu - config->rmarkerOffset;
			info->timestamp_dtu =
				timestamp_rctu_to_dtu(llhw, timestamp_rctu) -
				llhw->shr_dtu;
		}
	}
	/* In case of auto-ack send. */
	if (rx_flags & DW3000_RX_FLAG_AACK)
		info->flags |= MCPS802154_RX_FRAME_INFO_AACK;
	/* In case of PDoA. */
	if (info->flags & MCPS802154_RX_FRAME_INFO_RANGING_PDOA) {
		info->ranging_pdoa_rad_q11 = dw3000_read_pdoa(dw);
		info->ranging_pdoa_spacing_mm_q11 =
			config->antpair_spacing_mm_q11;
	}
	/* In case of STS */
	if (info->flags & MCPS802154_RX_FRAME_INFO_RANGING_STS_TIMESTAMP_RCTU) {
		u64 sts_ts_rctu;

		ret = dw3000_read_sts_timestamp(dw, &sts_ts_rctu);
		if (ret)
			goto error;
		/* DW3000 only support one STS segment. */
		info->ranging_sts_timestamp_diffs_rctu[0] = 0;
		info->ranging_sts_timestamp_diffs_rctu[1] =
			sts_ts_rctu - timestamp_rctu;
		if ((config->stsMode & DW3000_STS_BASIC_MODES_MASK) ==
		    DW3000_STS_MODE_2) {
			/* TODO: calc SRMARKER0 */
		}
	}
	info->ranging_sts_fom[0] = 0;
	if (info->flags & MCPS802154_RX_FRAME_INFO_RANGING_STS_FOM) {
		ret = get_ranging_sts_fom(llhw, info);
		if (ret)
			goto error;
	}
	if (info->flags & MCPS802154_RX_FRAME_INFO_RANGING_PDOA_FOM) {
		if (!(info->flags & MCPS802154_RX_FRAME_INFO_RANGING_STS_FOM)) {
			ret = get_ranging_sts_fom(llhw, info);
			if (ret)
				goto error;
		}
		if (info->ranging_sts_fom[0])
			info->ranging_pdoa_fom =
				get_ranging_pdoa_fom(info->ranging_sts_fom[0]);
		else
			info->ranging_pdoa_fom = 0;
	}
	/* Keep only implemented. */
	info->flags &= (MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU |
			MCPS802154_RX_FRAME_INFO_TIMESTAMP_DTU |
			MCPS802154_RX_FRAME_INFO_AACK |
			MCPS802154_RX_FRAME_INFO_RANGING_PDOA |
			MCPS802154_RX_FRAME_INFO_RANGING_PDOA_FOM |
			MCPS802154_RX_FRAME_INFO_RANGING_STS_FOM |
			MCPS802154_RX_FRAME_INFO_RANGING_STS_TIMESTAMP_RCTU);
	trace_dw3000_return_int_u32(dw, info->flags, *skb ? (*skb)->len : 0);
	return 0;

error:
	trace_dw3000_return_int_u32(dw, ret, 0);
	return ret;
}

static int rx_get_error_frame(struct mcps802154_llhw *llhw,
			      struct mcps802154_rx_frame_info *info)
{
	struct dw3000 *dw = llhw->priv;
	int ret = 0;

	trace_dw3000_mcps_rx_get_error_frame(dw, info->flags);
	/* Sanity check */
	if (unlikely(!info)) {
		ret = -EINVAL;
		goto error;
	}
	if (info->flags & MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU) {
		if (dw3000_read_rx_timestamp(dw, &info->timestamp_rctu))
			info->flags &= ~MCPS802154_RX_FRAME_INFO_TIMESTAMP_RCTU;
	} else {
		/* Not implemented */
		info->flags = 0;
	}
error:
	trace_dw3000_return_int_u32(dw, ret, info->flags);
	return ret;
}

static int do_reset(struct dw3000 *dw, const void *in, void *out)
{
	int rc;
	/* Disable the device before resetting it */
	rc = dw3000_disable(dw);
	if (rc) {
		dev_err(dw->dev, "device disable failed: %d\n", rc);
		return rc;
	}
	/* Soft reset */
	rc = dw3000_softreset(dw);
	if (rc != 0) {
		dev_err(dw->dev, "device reset failed: %d\n", rc);
		return rc;
	}
	/* Initialize & configure the device */
	rc = dw3000_init(dw, true);
	if (rc != 0) {
		dev_err(dw->dev, "device reset failed: %d\n", rc);
		return rc;
	}
	/* Enable the device */
	rc = dw3000_enable(dw);
	if (rc) {
		dev_err(dw->dev, "device enable failed: %d\n", rc);
		return rc;
	}
	return 0;
}

static int reset(struct mcps802154_llhw *llhw)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_stm_command cmd = { do_reset, NULL, NULL };
	int ret;

	trace_dw3000_mcps_reset(dw);
	ret = dw3000_enqueue_generic(dw, &cmd);
	trace_dw3000_return_int(dw, ret);
	return ret;
}

static int do_get_dtu(struct dw3000 *dw, const void *in, void *out)
{
	return dw3000_read_sys_time(dw, (u32 *)out);
}

static int get_current_timestamp_dtu(struct mcps802154_llhw *llhw,
				     u32 *timestamp_dtu)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_stm_command cmd = { do_get_dtu, NULL, timestamp_dtu };
	int ret;

	trace_dw3000_mcps_get_timestamp(dw);
	/* Must be called after start() */
	ret = dw->started ? dw3000_enqueue_generic(dw, &cmd) : -EBUSY;
	trace_dw3000_return_int_u32(dw, ret, *timestamp_dtu);
	return ret;
}

static inline u64 timestamp_dtu_to_rctu(struct mcps802154_llhw *llhw,
					u32 timestamp_dtu)
{
	return ((u64)timestamp_dtu) * DW3000_RCTU_PER_DTU;
}

static inline u32 timestamp_rctu_to_dtu(struct mcps802154_llhw *llhw,
					u64 timestamp_rctu)
{
	return (u32)(timestamp_rctu / DW3000_RCTU_PER_DTU);
}

static inline u64 tx_timestamp_dtu_to_rmarker_rctu(struct mcps802154_llhw *llhw,
						   u32 tx_timestamp_dtu,
						   int ant_id)
{
	struct dw3000 *dw = llhw->priv;
	/* LSB is ignored. */
	const u32 bit_mask = ~1;
	const u64 rctu = timestamp_dtu_to_rctu(
		llhw, (tx_timestamp_dtu + llhw->shr_dtu) & bit_mask);
	return rctu + tx_rmarker_offset(dw, ant_id);
}

static inline s64 difference_timestamp_rctu(struct mcps802154_llhw *llhw,
					    u64 timestamp_a_rctu,
					    u64 timestamp_b_rctu)
{
	/* RCTU time is an unsigned encoded over 40 bytes. This function
	   calculate the signed difference between two unsigned 40 bytes */
	static const u64 rctu_rollover = 1ll << 40;
	static const u64 rctu_mask = rctu_rollover - 1;
	s64 diff_rctu = (timestamp_a_rctu - timestamp_b_rctu) & rctu_mask;

	if (diff_rctu & (rctu_rollover >> 1))
		diff_rctu = diff_rctu | ~rctu_mask;
	return diff_rctu;
}

static int compute_frame_duration_dtu(struct mcps802154_llhw *llhw,
				      int payload_bytes)
{
	struct dw3000 *dw = llhw->priv;
	return dw3000_frame_duration_dtu(dw, payload_bytes, true);
}

static int do_set_channel(struct dw3000 *dw, const void *in, void *out)
{
	return dw3000_configure_chan(dw);
}

static int set_channel(struct mcps802154_llhw *llhw, u8 page, u8 channel,
		       u8 preamble_code)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_config *config = &dw->config;
	struct dw3000_stm_command cmd = { do_set_channel, NULL, NULL };
	int ret;

	trace_dw3000_mcps_set_channel(dw, page, channel, preamble_code);
	/* Check parameters early */
	if (page != 4 || (channel != 5 && channel != 9)) {
		ret = -EINVAL;
		goto error;
	}
	switch (preamble_code) {
	case 0:
		/* Set default value if MCPS don't give one to driver */
		preamble_code = 9;
		break;
	/* DW3000_PRF_16M */
	case 3:
	case 4:
	/* DW3000_PRF_64M */
	case 9:
	case 10:
	case 11:
	case 12:
		break;
	default:
		ret = -EINVAL;
		goto error;
	}
	/* Update configuration structure */
	config->chan = channel;
	config->txCode = preamble_code;
	config->rxCode = preamble_code;
	/* Reconfigure the chip with it if needed */
	ret = dw->started ? dw3000_enqueue_generic(dw, &cmd) : 0;
error:
	trace_dw3000_return_int(dw, ret);
	return ret;
}

static int set_hrp_uwb_params(struct mcps802154_llhw *llhw, int prf, int psr,
			      int sfd_selector, int phr_rate, int data_rate)
{
	struct dw3000 *dw = llhw->priv;

	dev_dbg(dw->dev, "%s called\n", __func__);
	return 0;
}

static int do_set_sts_params(struct dw3000 *dw, const void *in, void *out)
{
	const struct mcps802154_sts_params *params =
		(const struct mcps802154_sts_params *)in;
	enum dw3000_sts_lengths len;
	int rc;
	/* Set STS segment(s) length */
	/* ffs(x) return 1 for bit0, 2 for bit1... */
	len = (enum dw3000_sts_lengths)(ffs(params->seg_len) - 4);
	rc = dw3000_set_sts_length(dw, len);
	if (rc)
		return rc;
	/* Send KEY & IV */
	rc = dw3000_configure_sts_key(dw, params->key);
	if (rc)
		return rc;
	rc = dw3000_configure_sts_iv(dw, params->v);
	if (rc)
		return rc;
	return dw3000_load_sts_iv(dw);
}

static int set_sts_params(struct mcps802154_llhw *llhw,
			  const struct mcps802154_sts_params *params)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_stm_command cmd = { do_set_sts_params, params, NULL };
	int ret;
	/* Must be called after start() */
	if (!dw->started)
		return -EBUSY;
	/* Check parameters */
	if (params->n_segs > 1)
		return -EINVAL;
	trace_dw3000_mcps_set_sts_params(dw, params);
	ret = dw3000_enqueue_generic(dw, &cmd);
	trace_dw3000_return_int(dw, ret);
	return ret;
}

struct do_set_hw_addr_filt_params {
	struct ieee802154_hw_addr_filt *filt;
	unsigned long changed;
};

static int do_set_hw_addr_filt(struct dw3000 *dw, const void *in, void *out)
{
	struct ieee802154_hw_addr_filt *filt = &dw->config.hw_addr_filt;
	unsigned long changed = *(unsigned long *)in;
	int rc;

	if (changed & IEEE802154_AFILT_SADDR_CHANGED) {
		rc = dw3000_set_shortaddr(dw, filt->short_addr);
		if (rc)
			goto spi_err;
	}
	if (changed & IEEE802154_AFILT_IEEEADDR_CHANGED) {
		rc = dw3000_set_eui64(dw, filt->ieee_addr);
		if (rc)
			return rc;
	}
	if (changed & IEEE802154_AFILT_PANID_CHANGED) {
		rc = dw3000_set_panid(dw, filt->pan_id);
		if (rc)
			goto spi_err;
	}
	if (changed & IEEE802154_AFILT_PANC_CHANGED) {
		rc = dw3000_set_pancoord(dw, filt->pan_coord);
		if (rc)
			goto spi_err;
	}
	return 0;

spi_err:
	return rc;
}

static int set_hw_addr_filt(struct mcps802154_llhw *llhw,
			    struct ieee802154_hw_addr_filt *filt,
			    unsigned long changed)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_config *config = &dw->config;
	struct ieee802154_hw_addr_filt *cfilt = &config->hw_addr_filt;
	struct dw3000_stm_command cmd = { do_set_hw_addr_filt, &changed, NULL };
	int ret;

	if (changed & IEEE802154_AFILT_SADDR_CHANGED)
		cfilt->short_addr = filt->short_addr;
	if (changed & IEEE802154_AFILT_IEEEADDR_CHANGED)
		cfilt->ieee_addr = filt->ieee_addr;
	if (changed & IEEE802154_AFILT_PANID_CHANGED)
		cfilt->pan_id = filt->pan_id;
	if (changed & IEEE802154_AFILT_PANC_CHANGED)
		cfilt->pan_coord = filt->pan_coord;

	trace_dw3000_mcps_set_hw_addr_filt(dw, (u8)changed);
	ret = dw->started ? dw3000_enqueue_generic(dw, &cmd) : 0;
	trace_dw3000_return_int(dw, ret);
	return ret;
}

static int set_txpower(struct mcps802154_llhw *llhw, s32 mbm)
{
	struct dw3000 *dw = llhw->priv;

	dev_dbg(dw->dev, "%s called\n", __func__);
	return 0;
}

static int set_cca_mode(struct mcps802154_llhw *llhw,
			const struct wpan_phy_cca *cca)
{
	struct dw3000 *dw = llhw->priv;

	dev_dbg(dw->dev, "%s called\n", __func__);
	return 0;
}

static int set_cca_ed_level(struct mcps802154_llhw *llhw, s32 mbm)
{
	struct dw3000 *dw = llhw->priv;

	dev_dbg(dw->dev, "%s called\n", __func__);
	return 0;
}

static int do_set_promiscuous_mode(struct dw3000 *dw, const void *in, void *out)
{
	return dw3000_set_promiscuous(dw, dw->config.promisc);
}

static int set_promiscuous_mode(struct mcps802154_llhw *llhw, bool on)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_stm_command cmd = { do_set_promiscuous_mode, NULL, NULL };

	dev_dbg(dw->dev, "%s called, (mode: %sabled)\n", __func__,
		(on) ? "en" : "dis");
	dw->config.promisc = on;
	return dw->started ? dw3000_enqueue_generic(dw, &cmd) : 0;
}

static int set_calibration(struct mcps802154_llhw *llhw, const char *key,
			   void *value, size_t length)
{
	struct dw3000 *dw = llhw->priv;
	void *param;
	int len;
	/* Sanity checks */
	if (!key || !value || !length)
		return -EINVAL;
	/* Search parameter */
	len = dw3000_calib_parse_key(dw, key, &param);
	if (len < 0)
		return len;
	if (len > length)
		return -EINVAL;
	/* FIXME: This copy isn't big-endian compatible. */
	memcpy(param, value, len);
	/* One parameter has changed. */
	dw3000_calib_update_config(dw);
	/* TODO: need reconfiguration? */
	return 0;
}

static int get_calibration(struct mcps802154_llhw *llhw, const char *key,
			   void *value, size_t length)
{
	struct dw3000 *dw = llhw->priv;
	void *param;
	int len;
	/* Sanity checks */
	if (!key)
		return -EINVAL;
	/* Calibration parameters */
	len = dw3000_calib_parse_key(dw, key, &param);
	if (len < 0)
		return len;
	if (len <= length)
		memcpy(value, param, len);
	else if (value && length)
		/* Provided buffer size isn't enough, return an error */
		return -ENOSPC;
	/* Return selected parameter length or error */
	return len;
}

static const char *const *list_calibration(struct mcps802154_llhw *llhw)
{
	return dw3000_calib_list_keys(llhw->priv);
}

/**
 * vendor_cmd() - Forward vendor commands processing to dw3000 function.
 *
 * @llhw: Low-level hardware without MCPS.
 * @vendor_id: Vendor Identifier on 3 bytes.
 * @subcmd: Sub-command identifier.
 * @data: Null or data related with the sub-command.
 * @data_len: Length of the data
 *
 * Return: 0 on success, 1 to request a stop, error on other value.
 */
static int vendor_cmd(struct mcps802154_llhw *llhw, u32 vendor_id, u32 subcmd,
		      void *data, size_t data_len)
{
	struct dw3000 *dw = llhw->priv;

	/* There is only NFCC coexitence vendor command for now. */
	return dw3000_nfcc_coex_vendor_cmd(dw, vendor_id, subcmd, data,
					   data_len);
}

static const struct mcps802154_ops dw3000_mcps_ops = {
	.start = start,
	.stop = stop,
	.tx_frame = tx_frame,
	.rx_enable = rx_enable,
	.rx_disable = rx_disable,
	.rx_get_frame = rx_get_frame,
	.rx_get_error_frame = rx_get_error_frame,
	.reset = reset,
	.get_current_timestamp_dtu = get_current_timestamp_dtu,
	.timestamp_dtu_to_rctu = timestamp_dtu_to_rctu,
	.timestamp_rctu_to_dtu = timestamp_rctu_to_dtu,
	.tx_timestamp_dtu_to_rmarker_rctu = tx_timestamp_dtu_to_rmarker_rctu,
	.difference_timestamp_rctu = difference_timestamp_rctu,
	.compute_frame_duration_dtu = compute_frame_duration_dtu,
	.set_channel = set_channel,
	.set_hrp_uwb_params = set_hrp_uwb_params,
	.set_sts_params = set_sts_params,
	.set_hw_addr_filt = set_hw_addr_filt,
	.set_txpower = set_txpower,
	.set_cca_mode = set_cca_mode,
	.set_cca_ed_level = set_cca_ed_level,
	.set_promiscuous_mode = set_promiscuous_mode,
	.set_calibration = set_calibration,
	.get_calibration = get_calibration,
	.list_calibration = list_calibration,
	.vendor_cmd = vendor_cmd,
	MCPS802154_TESTMODE_CMD(dw3000_tm_cmd)
};

struct dw3000 *dw3000_mcps_alloc(struct device *dev)
{
	struct mcps802154_llhw *llhw;
	struct dw3000 *dw;

	dev_dbg(dev, "%s called\n", __func__);
	llhw = mcps802154_alloc_llhw(sizeof(*dw), &dw3000_mcps_ops);
	if (llhw == NULL)
		return NULL;
	dw = llhw->priv;
	dw->llhw = llhw;
	dw->dev = dev;
	dw3000_init_config(dw);

	/* Configure IEEE802154 HW capabilities */
	llhw->hw->flags =
		(IEEE802154_HW_TX_OMIT_CKSUM | IEEE802154_HW_AFILT |
		 IEEE802154_HW_PROMISCUOUS | IEEE802154_HW_RX_OMIT_CKSUM);
	llhw->flags = llhw->hw->flags;

	/* UWB High band 802.15.4a-2007. Only channels 5 & 9 for DW3000. */
	llhw->hw->phy->supported.channels[4] = (1 << 5) | (1 << 9);

	/* Set time related fields */
	llhw->dtu_freq_hz = DW3000_DTU_FREQ;
	llhw->dtu_rctu = DW3000_RCTU_PER_DTU;
	llhw->rstu_dtu = DW3000_DTU_PER_RSTU;
	llhw->anticip_dtu = DW3000_ANTICIP_DTU;
	/* Set time related field that are configuration dependent */
	dw3000_update_timings(dw);
	/* Symbol is ~0.994us @ PRF16 or ~1.018us @ PRF64. Use 1. */
	llhw->hw->phy->symbol_duration = 1;

	/* Set extended address. */
	llhw->hw->phy->perm_extended_addr = 0xd6552cd6e41ceb57;

	/* Driver phy page 4 as default, channel is copied from init config. */
	llhw->hw->phy->current_channel = dw->config.chan;
	llhw->hw->phy->current_page = 4;

	return dw;
}

void dw3000_mcps_free(struct dw3000 *dw)
{
	dev_dbg(dw->dev, "%s called\n", __func__);
	if (dw->llhw) {
		mcps802154_free_llhw(dw->llhw);
		dw->llhw = NULL;
	}
}

int dw3000_mcps_register(struct dw3000 *dw)
{
	dev_dbg(dw->dev, "%s called\n", __func__);
	return mcps802154_register_llhw(dw->llhw);
}

void dw3000_mcps_unregister(struct dw3000 *dw)
{
	dev_dbg(dw->dev, "%s called\n", __func__);
	mcps802154_unregister_llhw(dw->llhw);
}
