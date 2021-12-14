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
#include "dw3000_nfcc_coex_mcps.h"
#include "dw3000_coex.h"

/* UWB High band 802.15.4a-2007. Only channels 5 & 9 for DW3000. */
#define DW3000_SUPPORTED_CHANNELS ((1 << 5) | (1 << 9))

static inline u32 timestamp_rctu_to_dtu(struct dw3000 *dw, u64 timestamp_rctu);
static inline u64 timestamp_rctu_to_rmarker_rctu(struct dw3000 *dw,
						 u64 timestamp_rctu,
						 u32 rmarker_dtu);

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

	trace_dw3000_mcps_start(dw);

	/* Enforce required CPU latency */
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
		goto fail;
	}
	/* Initialize & configure the device */
	rc = dw3000_init(dw, true);
	if (rc) {
		dev_err(dw->dev, "device init failed: %d\n", rc);
		goto fail;
	}
	/* Apply other configuration not done by dw3000_init() */
	rc = do_set_hw_addr_filt(dw, &changed, NULL);
	if (rc)
		goto fail;
	rc = do_set_promiscuous_mode(dw, NULL, NULL);
	if (rc)
		goto fail;
	/* Reset ranging clock requirement */
	dw->need_ranging_clock = false;
	/* Enable the device */
	rc = dw3000_enable(dw);
fail:
	trace_dw3000_return_int(dw, rc);
	return rc;
}

static int start(struct mcps802154_llhw *llhw)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_stm_command cmd = { do_start, NULL, NULL };
	int rc;

	/* Turn on power (with RST GPIO) and wait SPI ready IRQ */
	rc = dw3000_hardreset(dw);
	if (rc) {
		dev_err(dw->dev, "device power on failed: %d\n", rc);
		return rc;
	}
	/* Do soft reset and all initialization from the high-prio thread */
	return dw3000_enqueue_generic(dw, &cmd);
}

static int do_stop(struct dw3000 *dw, const void *in, void *out)
{
	int rc;

	trace_dw3000_mcps_stop(dw);

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
	/* Reset ranging clock requirement */
	dw->need_ranging_clock = false;
	dw3000_reset_rctu_conv_state(dw);
	/* Reset cached antenna config to ensure GPIO are well reconfigured */
	dw->config.ant[0] = -1;
	dw->config.ant[1] = -1;
	/* Relax CPU latency requirement */
	dw3000_pm_qos_update_request(dw, PM_QOS_RESUME_LATENCY_NO_CONSTRAINT);
	trace_dw3000_return_void(dw);
	return 0;
}

static void stop(struct mcps802154_llhw *llhw)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_stm_command cmd = { do_stop, NULL, NULL };

	dw3000_enqueue_generic(dw, &cmd);
}

struct do_tx_frame_params {
	struct sk_buff *skb;
	const struct mcps802154_tx_frame_info *info;
	int frame_idx;
};

static int do_tx_frame(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_tx_frame_params *params = in;

	return dw3000_do_tx_frame(dw, params->info, params->skb,
				  params->frame_idx);
}

static int tx_frame(struct mcps802154_llhw *llhw, struct sk_buff *skb,
		    const struct mcps802154_tx_frame_info *info, int frame_idx,
		    int next_delay_dtu)
{
	struct dw3000 *dw = llhw->priv;
	struct do_tx_frame_params params = { .skb = skb,
					     .info = info,
					     .frame_idx = frame_idx };
	struct dw3000_stm_command cmd = { do_tx_frame, &params, NULL };

	/* Check data : no data if SP3, must have data otherwise */
	if (((info->flags & MCPS802154_TX_FRAME_STS_MODE_MASK) ==
	     MCPS802154_TX_FRAME_SP3) != !skb)
		return -EINVAL;

	return dw3000_enqueue_generic(dw, &cmd);
}

struct do_rx_frame_params {
	const struct mcps802154_rx_info *info;
	int frame_idx;
};

static int do_rx_enable(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_rx_frame_params *params =
		(const struct do_rx_frame_params *)in;

	return dw3000_do_rx_enable(dw, params->info, params->frame_idx);
}

static int rx_enable(struct mcps802154_llhw *llhw,
		     const struct mcps802154_rx_info *info, int frame_idx,
		     int next_delay_dtu)
{
	struct dw3000 *dw = llhw->priv;
	struct do_rx_frame_params params = { .info = info,
					     .frame_idx = frame_idx };
	struct dw3000_stm_command cmd = { do_rx_enable, &params, NULL };

	return dw3000_enqueue_generic(dw, &cmd);
}

static int do_rx_disable(struct dw3000 *dw, const void *in, void *out)
{
	int ret;

	trace_dw3000_mcps_rx_disable(dw);
	ret = dw3000_rx_disable(dw);
	/* Reset ranging clock requirement */
	dw->need_ranging_clock = false;
	dw3000_reset_rctu_conv_state(dw);
	trace_dw3000_return_int(dw, ret);
	return ret;
}

static int rx_disable(struct mcps802154_llhw *llhw)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_stm_command cmd = { do_rx_disable, NULL, NULL };

	return dw3000_enqueue_generic(dw, &cmd);
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
			u32 rmarker_dtu =
				timestamp_rctu_to_dtu(dw, timestamp_rctu);
			u64 rmarker_rctu = timestamp_rctu_to_rmarker_rctu(
				dw, timestamp_rctu, rmarker_dtu);
			info->timestamp_rctu =
				rmarker_rctu - config->rmarkerOffset;
			info->timestamp_dtu = rmarker_dtu - llhw->shr_dtu;
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
		info->ranging_aoa_rad_q11 =
			dw3000_pdoa_to_aoa_lut(dw, info->ranging_pdoa_rad_q11);
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

static int do_idle(struct dw3000 *dw, const void *in, void *out)
{
	struct dw3000_deep_sleep_state *dss = &dw->deep_sleep_state;
	u32 cur_time_dtu = 0;
	int delay_us = 0, rc;

	trace_dw3000_mcps_idle(dw, in != NULL, in ? *(const u32 *)in : 0);

	/* Reset ranging clock requirement */
	dw->need_ranging_clock = false;
	dw3000_reset_rctu_conv_state(dw);
	/* Ensure dw3000_wakeup_timer() handler does the right thing */
	dss->next_operational_state = DW3000_OP_STATE_IDLE_PLL;

	/* Release Wifi coexistence. */
	dw3000_coex_stop(dw);
	/* Check if enough idle time to enter DEEP SLEEP */
	if (in) {
		u32 date_dtu = *(const u32 *)in;
		int effective_delay_us;
		cur_time_dtu = dw3000_get_dtu_time(dw);
		effective_delay_us = DTU_TO_US(date_dtu - cur_time_dtu);
		delay_us = dw3000_can_deep_sleep(dw, effective_delay_us);
		if (!delay_us) {
			/* Provided date isn't far enough to enter deep-sleep,
			   just launch the timer to have it call the MCPS timer
			   expired event function. */
			dw3000_wakeup_timer_start(dw, effective_delay_us);
			rc = 0;
			goto eof;
		}
	} else if (dw->auto_sleep_margin_us < 0) {
		/* Deep-sleep is completly disable, so do nothing here! */
		rc = 0;
		goto eof;
	}
	/* Enter DEEP SLEEP */
	rc = dw3000_deep_sleep_and_wakeup(dw, delay_us);
eof:
	trace_dw3000_return_int(dw, rc);
	return rc;
}

static int idle(struct mcps802154_llhw *llhw, bool timestamp, u32 timestamp_dtu)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_stm_command cmd = { do_idle, NULL, NULL };

	if (timestamp)
		cmd.in = &timestamp_dtu;
	return dw3000_enqueue_generic(dw, &cmd);
}

static int do_reset(struct dw3000 *dw, const void *in, void *out)
{
	int rc;

	trace_dw3000_mcps_reset(dw);
	/* Disable the device before resetting it */
	rc = dw3000_disable(dw);
	if (rc) {
		dev_err(dw->dev, "device disable failed: %d\n", rc);
		goto fail;
	}
	/* Soft reset */
	rc = dw3000_softreset(dw);
	if (rc != 0) {
		dev_err(dw->dev, "device reset failed: %d\n", rc);
		goto fail;
	}
	/* Initialize & configure the device */
	rc = dw3000_init(dw, true);
	if (rc != 0) {
		dev_err(dw->dev, "device reset failed: %d\n", rc);
		goto fail;
	}
	/* Enable the device */
	rc = dw3000_enable(dw);
	if (rc) {
		dev_err(dw->dev, "device enable failed: %d\n", rc);
		goto fail;
	}
fail:
	dw3000_reset_rctu_conv_state(dw);
	trace_dw3000_return_int(dw, rc);
	return rc;
}

static int reset(struct mcps802154_llhw *llhw)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_stm_command cmd = { do_reset, NULL, NULL };

	return dw3000_enqueue_generic(dw, &cmd);
}

static int get_current_timestamp_dtu(struct mcps802154_llhw *llhw,
				     u32 *timestamp_dtu)
{
	struct dw3000 *dw = llhw->priv;
	int ret = 0;

	trace_dw3000_mcps_get_timestamp(dw);
	/* Must be called after start() */
	if (!dw3000_is_active(dw)) {
		ret = -EBUSY;
		goto fail;
	}

	*timestamp_dtu = dw3000_get_dtu_time(dw);
	if (dw->current_operational_state < DW3000_OP_STATE_IDLE_PLL)
		*timestamp_dtu += US_TO_DTU(DW3000_WAKEUP_LATENCY_US);

fail:
	trace_dw3000_return_int_u32(dw, ret, *timestamp_dtu);
	return ret;
}

static inline u32 timestamp_rctu_to_dtu(struct dw3000 *dw, u64 timestamp_rctu)
{
	u32 sys_time = (u32)(timestamp_rctu / DW3000_RCTU_PER_SYS);
	u32 dtu_near = dw3000_get_dtu_time(dw) - DW3000_DTU_FREQ;

	return dw3000_sys_time_to_dtu(dw, sys_time, dtu_near);
}

static inline u64 timestamp_rctu_to_rmarker_rctu(struct dw3000 *dw,
						 u64 timestamp_rctu,
						 u32 rmarker_dtu)
{
	struct dw3000_rctu_conv *rctu = &dw->rctu_conv;
	static const u64 rctu_mask = (1ll << 40) - 1;
	u64 rmarker_rctu;

	if (rctu->state == UNALIGNED) {
		rctu->alignment_rmarker_dtu = rmarker_dtu;
		rctu->state = ALIGNED;
		trace_dw3000_rctu_convert_align(dw, rmarker_dtu);
	}
	if (rctu->state == ALIGNED) {
		u64 alignment_rmarker_rctu =
			((u64) dw3000_dtu_to_sys_time(dw, rctu->alignment_rmarker_dtu)) * DW3000_RCTU_PER_SYS;
		rctu->synced_rmarker_rctu = alignment_rmarker_rctu;
		rctu->state = ALIGNED_SYNCED;
		trace_dw3000_rctu_convert_synced(dw, alignment_rmarker_rctu);
	}
	rmarker_rctu = (timestamp_rctu - rctu->synced_rmarker_rctu) & rctu_mask;
	trace_dw3000_rctu_convert_rx(dw, rmarker_dtu, timestamp_rctu,
				     rmarker_rctu);
	return rmarker_rctu;
}

static u64 tx_timestamp_dtu_to_rmarker_rctu(struct mcps802154_llhw *llhw,
					    u32 tx_timestamp_dtu, int ant_id)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_rctu_conv *rctu = &dw->rctu_conv;

	const u32 rmarker_dtu = tx_timestamp_dtu + llhw->shr_dtu;
	const u32 ant_offset = tx_rmarker_offset(dw, ant_id);
	u64 rmarker_rctu;
	s64 rmarker_diff_dtu;

	if (rctu->state == UNALIGNED) {
		rctu->alignment_rmarker_dtu = rmarker_dtu;
		rctu->state = ALIGNED;
		trace_dw3000_rctu_convert_align(dw, rmarker_dtu);
	}
	rmarker_diff_dtu = rmarker_dtu - rctu->alignment_rmarker_dtu;
	rmarker_rctu = rmarker_diff_dtu * DW3000_RCTU_PER_DTU + ant_offset;
	trace_dw3000_rctu_convert_tx(dw, tx_timestamp_dtu, ant_offset,
				     rmarker_rctu);
	return rmarker_rctu;
}

static s64 difference_timestamp_rctu(struct mcps802154_llhw *llhw,
				     u64 timestamp_a_rctu, u64 timestamp_b_rctu)
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
	struct dw3000_deep_sleep_state *dss = &dw->deep_sleep_state;
	unsigned long changed = *(const unsigned long *)in;

	if (dw->current_operational_state < DW3000_OP_STATE_IDLE_RC) {
		/* Cannot configure device, save info and ensure it will be
		   configured on wakeup */
		dss->config_changed |= changed;
		return 0;
	}
	if (changed & DW3000_CHANNEL_CHANGED)
		/* Reconfigure all channel dependent */
		return dw3000_configure_chan(dw);
	else if (changed & DW3000_PCODE_CHANGED)
		/* Only change CHAN_CTRL with new code */
		return dw3000_configure_pcode(dw);
	return 0;
}

static int set_channel(struct mcps802154_llhw *llhw, u8 page, u8 channel,
		       u8 preamble_code)
{
	unsigned long changed = 0;
	struct dw3000 *dw = llhw->priv;
	struct dw3000_config *config = &dw->config;
	struct dw3000_stm_command cmd = { do_set_channel, &changed, NULL };
	int ret = 0;

	trace_dw3000_mcps_set_channel(dw, page, channel, preamble_code);
	/* Check parameters early */
	if (page != 4 || (channel != 5 && channel != 9) ||
	    (dw->restricted_channels & (1 << (channel % 16)))) {
		ret = -EINVAL;
		goto trace;
	}
	switch (preamble_code) {
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
		goto trace;
	}
	/* Detect configuration change(s) */
	if (config->chan != channel)
		changed |= DW3000_CHANNEL_CHANGED;
	if ((config->txCode != preamble_code) ||
	    (config->rxCode != preamble_code))
		changed |= DW3000_PCODE_CHANGED;
	if (!changed)
		goto trace;
	/* Update configuration structure */
	config->chan = channel;
	config->txCode = preamble_code;
	config->rxCode = preamble_code;
	/* Reconfigure the chip with it if needed */
	ret = dw3000_is_active(dw) ? dw3000_enqueue_generic(dw, &cmd) : 0;
trace:
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
	trace_dw3000_mcps_set_sts_params(dw, params);
	/* Set STS segment(s) length */
	/* ffs(x) return 1 for bit0, 2 for bit1... */
	len = (enum dw3000_sts_lengths)(ffs(params->seg_len) - 4);
	rc = dw3000_set_sts_length(dw, len);
	if (rc)
		goto fail;
	/* Send KEY & IV */
	rc = dw3000_configure_sts_key(dw, params->key);
	if (rc)
		goto fail;
	rc = dw3000_configure_sts_iv(dw, params->v);
	if (rc)
		goto fail;
	rc = dw3000_load_sts_iv(dw);
fail:
	trace_dw3000_return_int(dw, rc);
	return rc;
}

static int set_sts_params(struct mcps802154_llhw *llhw,
			  const struct mcps802154_sts_params *params)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_stm_command cmd = { do_set_sts_params, params, NULL };

	/* Must be called after start() */
	if (!dw3000_is_active(dw))
		return -EBUSY;
	/* Check parameters */
	if (params->n_segs > 1)
		return -EINVAL;
	/* TODO: HACK TO REMOVE: store parameters and setup change on wakeup */
	dw3000_wakeup_and_wait(dw);
	return dw3000_enqueue_generic(dw, &cmd);
}

struct do_set_hw_addr_filt_params {
	struct ieee802154_hw_addr_filt *filt;
	unsigned long changed;
};

static int do_set_hw_addr_filt(struct dw3000 *dw, const void *in, void *out)
{
	struct dw3000_deep_sleep_state *dss = &dw->deep_sleep_state;
	const unsigned long changed = *(unsigned long *)in;

	if (dw->current_operational_state < DW3000_OP_STATE_IDLE_RC) {
		/* Cannot configure device, save info and ensure it will be
		   configured on wakeup */
		dss->config_changed |= changed;
		return 0;
	}

	return dw3000_configure_hw_addr_filt(dw, changed);
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
	ret = dw3000_is_active(dw) ? dw3000_enqueue_generic(dw, &cmd) : 0;
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
	return dw3000_is_active(dw) ? dw3000_enqueue_generic(dw, &cmd) : 0;
}

static int check_calibration_value(struct mcps802154_llhw *llhw,
				   const char *key, void *value)
{
	struct dw3000 *dw = llhw->priv;
	struct dw3000_config *config = &dw->config;

	/* Prevent restricted channels from containing current channel. */
	if ((!strcmp(key, "restricted_channels")) &&
	    ((1 << llhw->hw->phy->current_channel) & *(u16 *)value) &&
	    ((1 << dw->llhw->hw->phy->current_channel) & dw->restricted_channels)) {
		/* Change channel if the current one is restricted. */
		config->chan = ffs(dw->llhw->hw->phy->supported.channels[4]) - 1;
		dw->llhw->hw->phy->current_channel = config->chan;
	}
	if ((strlen(key) > 22) && !strcmp(key + 13, ".pdoa_lut")) {
		int i;
		dw3000_pdoa_lut_t *lut = value;
		for (i = 1; i < DW3000_CALIBRATION_PDOA_LUT_MAX; i++)
			if ((*lut)[i][0] <= (*lut)[i - 1][0])
				return -EINVAL;
	}
	return 0;
}

static int set_calibration(struct mcps802154_llhw *llhw, const char *key,
			   void *value, size_t length)
{
	struct dw3000 *dw = llhw->priv;
	void *param;
	int len;
	int r;
	/* Sanity checks */
	if (!key || !value || !length)
		return -EINVAL;
	/* Search parameter */
	len = dw3000_calib_parse_key(dw, key, &param);
	if (len < 0)
		return len;
	if (len > length)
		return -EINVAL;
	r = check_calibration_value(llhw, key, value);
	if (r)
		return r;
	/* FIXME: This copy isn't big-endian compatible. */
	memcpy(param, value, len);

	/* One parameter has changed. */
	dw3000_calib_update_config(dw);
	llhw->hw->phy->supported.channels[4] = DW3000_SUPPORTED_CHANNELS &
					       ~dw->restricted_channels;
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

struct do_vendor_params {
	u32 vendor_id;
	u32 subcmd;
	void *data;
	size_t data_len;
};

static int do_vendor_cmd(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_vendor_params *params = in;
	/* There is only NFCC coexitence vendor command for now. */
	return dw3000_nfcc_coex_vendor_cmd(dw, params->vendor_id,
					   params->subcmd, params->data,
					   params->data_len);
}

/**
 * vendor_cmd() - Forward vendor commands processing to dw3000 function.
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
	struct do_vendor_params params = {
		.vendor_id = vendor_id,
		.subcmd = subcmd,
		.data = data,
		.data_len = data_len,
	};
	struct dw3000_stm_command cmd = { do_vendor_cmd, &params, NULL };
	return dw3000_enqueue_generic(dw, &cmd);
}

static const struct mcps802154_ops dw3000_mcps_ops = {
	.start = start,
	.stop = stop,
	.tx_frame = tx_frame,
	.rx_enable = rx_enable,
	.rx_disable = rx_disable,
	.rx_get_frame = rx_get_frame,
	.rx_get_error_frame = rx_get_error_frame,
	.idle = idle,
	.reset = reset,
	.get_current_timestamp_dtu = get_current_timestamp_dtu,
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

	llhw->hw->phy->supported.channels[4] = DW3000_SUPPORTED_CHANNELS;

	/* Set time related fields */
	llhw->dtu_freq_hz = DW3000_DTU_FREQ;
	llhw->dtu_rctu = DW3000_RCTU_PER_DTU;
	llhw->rstu_dtu = DW3000_DTU_PER_RSTU;
	llhw->anticip_dtu = DW3000_ANTICIP_DTU;
	llhw->idle_dtu = DW3000_DTU_FREQ;
	/* Set time related field that are configuration dependent */
	dw3000_update_timings(dw);
	/* Symbol is ~0.994us @ PRF16 or ~1.018us @ PRF64. Use 1. */
	llhw->hw->phy->symbol_duration = 1;

	/* Set extended address. */
	llhw->hw->phy->perm_extended_addr = 0xd6552cd6e41ceb57;

	/* Driver phy page 4 as default, channel is copied from init config. */
	llhw->hw->phy->current_channel = dw->config.chan;
	llhw->hw->phy->current_page = 4;
	llhw->current_preamble_code = dw->config.txCode;

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

void dw3000_mcps_reset(struct dw3000 *dw)
{
	do_stop(dw, NULL, NULL);
	do_start(dw, NULL, NULL);
}
