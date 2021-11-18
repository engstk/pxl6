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

#include <linux/slab.h>
#include <net/netlink.h>

#include "dw3000.h"
#include "dw3000_core.h"
#include "dw3000_trc.h"
#include "dw3000_testmode.h"
#include "dw3000_testmode_nl.h"
#include "dw3000_ccc.h"

static const struct nla_policy dw3000_tm_policy[DW3000_TM_ATTR_MAX + 1] = {
	[DW3000_TM_ATTR_CMD] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_RX_GOOD_CNT] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_RX_BAD_CNT] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_RSSI_DATA] = { .type = NLA_BINARY,
				       .len = DW3000_TM_RSSI_DATA_MAX_LEN },
	[DW3000_TM_ATTR_OTP_ADDR] = { .type = NLA_U16 },
	[DW3000_TM_ATTR_OTP_VAL] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_OTP_DONE] = { .type = NLA_U8 },
	[DW3000_TM_ATTR_CCC_TIME0] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_CCC_CHANNEL] = { .type = NLA_U8 },
	[DW3000_TM_ATTR_CCC_TSTART] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_CCC_TEND] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_CCC_MARGIN_MS] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_CCC_RR_COUNT] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_CCC_OFFSET_MS] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_CCC_CONFLICT_SLOT_IDX] = { .type = NLA_U32 },
	[DW3000_TM_ATTR_DEEP_SLEEP_DELAY_MS] = { .type = NLA_U32 },
};

struct do_tm_cmd_params {
	struct mcps802154_llhw *llhw;
	struct nlattr **nl_attr;
};

static int do_tm_cmd_start_rx_diag(struct dw3000 *dw, const void *in, void *out)
{
	int rc;
	/**
	 * Since the MCPS enables RX by default on the device at startup before
	 * running any test, enabling it once again manually via testmode
	 * command will cause the device to raise repeated IRQ and flood
	 * this driver's event thread.
	 * As a workaround, we disable RX before performing the testmode
	 * command.
	 */
	rc = dw3000_rx_disable(dw);
	if (rc)
		return rc;
	/* Enable receiver and promiscuous mode */
	rc = dw3000_rx_enable(dw, 0, 0, 0);
	if (rc)
		return rc;
	rc = dw3000_set_promiscuous(dw, true);
	if (rc)
		return rc;
	/* Enable statistics */
	return dw3000_rx_stats_enable(dw, true);
}

static int do_tm_cmd_stop_rx_diag(struct dw3000 *dw, const void *in, void *out)
{
	int rc;
	/* Disable receiver and promiscuous mode */
	rc = dw3000_rx_disable(dw);
	if (rc)
		return rc;
	rc = dw3000_set_promiscuous(dw, false);
	if (rc)
		return rc;
	/* Disable statistics */
	return dw3000_rx_stats_enable(dw, false);
}

static int do_tm_cmd_get_rx_diag(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_tm_cmd_params *params = in;
	struct dw3000_stats *stats = &dw->stats;
	size_t rssi_len =
		stats->count[DW3000_STATS_RX_GOOD] * sizeof(struct dw3000_rssi);
	struct sk_buff *nl_skb;
	int rc;

	/**
	 * Allocate netlink message. The approximated size includes
	 * the testmode's command id and data.
	 */
	nl_skb = mcps802154_testmode_alloc_reply_skb(
		params->llhw, 3 * sizeof(u32) + rssi_len);
	if (!nl_skb) {
		dev_err(dw->dev, "failed to alloc skb reply\n");
		return -ENOMEM;
	}
	/* Append good and bad RX counters to the netlink message */
	rc = nla_put_u32(nl_skb, DW3000_TM_ATTR_RX_GOOD_CNT,
			 stats->count[DW3000_STATS_RX_GOOD]);
	if (rc) {
		dev_err(dw->dev, "failed to put testmode rx counter: %d\n", rc);
		goto nla_put_failure;
	}
	rc = nla_put_u32(nl_skb, DW3000_TM_ATTR_RX_BAD_CNT,
			 stats->count[DW3000_STATS_RX_ERROR] +
				 stats->count[DW3000_STATS_RX_TO]);
	if (rc) {
		dev_err(dw->dev, "failed to put testmode rx counter: %d\n", rc);
		goto nla_put_failure;
	}
	/* Append RSSI data to the netlink message */
	rc = nla_put(nl_skb, DW3000_TM_ATTR_RSSI_DATA, rssi_len, stats->rssi);
	if (rc) {
		dev_err(dw->dev, "failed to copy testmode data: %d\n", rc);
		goto nla_put_failure;
	}
	return mcps802154_testmode_reply(params->llhw, nl_skb);

nla_put_failure:
	nlmsg_free(nl_skb);
	return rc;
}

static int do_tm_cmd_clear_rx_diag(struct dw3000 *dw, const void *in, void *out)
{
	/* Clear statistics */
	dw3000_rx_stats_clear(dw);
	return 0;
}

static int do_tm_cmd_start_tx_cwtone(struct dw3000 *dw, const void *in,
				     void *out)
{
	int rc;
	/* Disable receiver */
	rc = dw3000_rx_disable(dw);
	if (rc)
		return rc;
	/* Play repeated CW tone */
	return dw3000_tx_setcwtone(dw, true);
}

static int do_tm_cmd_stop_tx_cwtone(struct dw3000 *dw, const void *in,
				    void *out)
{
	/* Stop repeated CW tone */
	return dw3000_tx_setcwtone(dw, false);
}

static int do_tm_cmd_otp_read(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_tm_cmd_params *params = in;
	struct sk_buff *msg;
	u32 otp_val;
	u16 otp_addr;
	int rc;

	/* Verify the OTP address attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_OTP_ADDR])
		return -EINVAL;

	otp_addr = nla_get_u16(params->nl_attr[DW3000_TM_ATTR_OTP_ADDR]);
	/* Verify if the given OTP address exceeds the limit */
	if (otp_addr > DW3000_OTP_ADDRESS_LIMIT)
		return -EINVAL;
	/* Read at OTP address */
	rc = dw3000_otp_read32(dw, otp_addr, &otp_val);
	if (rc)
		return rc;
	/**
	 * Allocate netlink message. The approximated size includes
	 * the testmode's command id and data.
	 */
	msg = mcps802154_testmode_alloc_reply_skb(params->llhw,
						  2 * sizeof(u32));
	if (!msg) {
		dev_err(dw->dev, "failed to alloc skb reply\n");
		return -ENOMEM;
	}
	/* Append OTP memory's value to the netlink message */
	rc = nla_put_u32(msg, DW3000_TM_ATTR_OTP_VAL, otp_val);
	if (rc) {
		dev_err(dw->dev, "failed to put testmode otp val: %d\n", rc);
		goto nla_put_failure;
	}
	return mcps802154_testmode_reply(params->llhw, msg);

nla_put_failure:
	nlmsg_free(msg);
	return rc;
}

static int do_tm_cmd_otp_write(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_tm_cmd_params *params = in;
	struct sk_buff *msg;
	u32 otp_val;
	u16 otp_addr;
	u8 otp_done;
	int rc;

	/* Verify the OTP address attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_OTP_ADDR] ||
	    !params->nl_attr[DW3000_TM_ATTR_OTP_VAL])
		return -EINVAL;

	otp_addr = nla_get_u16(params->nl_attr[DW3000_TM_ATTR_OTP_ADDR]);
	/* Verify if the given OTP address exceeds the limit */
	if (otp_addr > DW3000_OTP_ADDRESS_LIMIT)
		return -EINVAL;
	otp_val = nla_get_u32(params->nl_attr[DW3000_TM_ATTR_OTP_VAL]);
	/* Write at OTP address */
	rc = dw3000_otp_write32(dw, otp_addr, otp_val);
	otp_done = (rc) ? 0 : 1;
	/**
	 * Allocate netlink message. The approximated size includes
	 * the testmode's command id and data.
	 */
	msg = mcps802154_testmode_alloc_reply_skb(params->llhw,
						  2 * sizeof(u32));
	if (!msg) {
		dev_err(dw->dev, "failed to alloc skb reply\n");
		return -ENOMEM;
	}
	/* Append OTP memory's value to the netlink message */
	rc = nla_put_u8(msg, DW3000_TM_ATTR_OTP_DONE, otp_done);
	if (rc) {
		dev_err(dw->dev, "failed to put testmode otp done: %d\n", rc);
		goto nla_put_failure;
	}
	return mcps802154_testmode_reply(params->llhw, msg);

nla_put_failure:
	nlmsg_free(msg);
	return rc;
}

/* helper function for common CCC command returning their rc code */
static int tm_ccc_cmd_done(struct dw3000 *dw, struct mcps802154_llhw *llhw,
			   u8 ccc_cmd_rc)
{
	struct sk_buff *msg;
	int rc;

	msg = mcps802154_testmode_alloc_reply_skb(llhw, 2 * sizeof(u32));
	if (!msg) {
		dev_err(dw->dev, "failed to alloc skb reply\n");
		return -ENOMEM;
	}

	/* Append ccc cmd rc to the netlink message */
	rc = nla_put_u8(msg, DW3000_TM_ATTR_CCC_CMD_RC, ccc_cmd_rc);
	if (rc) {
		dev_err(dw->dev, "failed to put testmode ccc cmd rc: %d\n", rc);
		goto nla_put_failure;
	}
	return mcps802154_testmode_reply(llhw, msg);

nla_put_failure:
	nlmsg_free(msg);
	return rc;
}

static int do_tm_cmd_ccc_start(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_tm_cmd_params *params = in;
	u8 channel;
	u32 session_time0;
	u32 start = 0;
	u32 end = 0;
	u8 ccc_cmd_rc;

	/* Verify the ccc channel attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_CCC_CHANNEL])
		return -EINVAL;
	channel = nla_get_u8(params->nl_attr[DW3000_TM_ATTR_CCC_CHANNEL]);

	/* Verify the ccc time0 attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_CCC_TIME0])
		return -EINVAL;
	session_time0 = nla_get_u32(params->nl_attr[DW3000_TM_ATTR_CCC_TIME0]);

	if (params->nl_attr[DW3000_TM_ATTR_CCC_TSTART] &&
	    params->nl_attr[DW3000_TM_ATTR_CCC_TEND]) {
		start = nla_get_u32(params->nl_attr[DW3000_TM_ATTR_CCC_TSTART]);
		end = nla_get_u32(params->nl_attr[DW3000_TM_ATTR_CCC_TEND]);
	}

	if (dw3000_nfcc_coex_testmode_session(dw, channel, session_time0, start,
					      end))
		ccc_cmd_rc = 0;
	else
		ccc_cmd_rc = 1;

	return tm_ccc_cmd_done(dw, params->llhw, ccc_cmd_rc);
}

static int do_tm_cmd_ccc_test_scratch(struct dw3000 *dw, const void *in,
				      void *out)
{
	const struct do_tm_cmd_params *params = in;
	u8 ccc_cmd_rc;

	/* TODO: write to the whole memory, read back, validate */
	ccc_cmd_rc = 1;

	return tm_ccc_cmd_done(dw, params->llhw, ccc_cmd_rc);
}

static int do_tm_cmd_ccc_test_spi1(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_tm_cmd_params *params = in;
	u8 ccc_cmd_rc;

	/**
	 * TODO: test SPI1
	 * - enable SPI1
	 * - check SPI1 is enabled
	 * - release SPI1
	 * - check SPI1 is disabled
        */
	ccc_cmd_rc = 1;

	return tm_ccc_cmd_done(dw, params->llhw, ccc_cmd_rc);
}

static int do_tm_cmd_ccc_test_spi2(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_tm_cmd_params *params = in;
	u8 ccc_cmd_rc;

	/**
	 * TODO: test SPI2
	 * - enable SPI2
	 * - wait for release (from other side code or watchdog?)
        */
	ccc_cmd_rc = 1;

	return tm_ccc_cmd_done(dw, params->llhw, ccc_cmd_rc);
}

static int do_tm_cmd_ccc_test_direct(struct dw3000 *dw, const void *in,
				     void *out)
{
	const struct do_tm_cmd_params *params = in;
	u8 ccc_cmd_rc;
	static struct dw3000_nfcc_coex_testmode_config conf = {
		.mode = DW3000_CCC_TEST_DIRECT,
		.start = 0,
		.end = 0,
	};

	/* Verify the ccc channel attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_CCC_CHANNEL])
		return -EINVAL;
	conf.channel = nla_get_u8(params->nl_attr[DW3000_TM_ATTR_CCC_CHANNEL]);

	/* Verify the ccc time0 attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_CCC_TIME0])
		return -EINVAL;
	conf.session_time0 =
		nla_get_u32(params->nl_attr[DW3000_TM_ATTR_CCC_TIME0]);

	if (dw3000_nfcc_coex_testmode_config(dw, &conf))
		ccc_cmd_rc = 0;
	else
		ccc_cmd_rc = 1;

	return tm_ccc_cmd_done(dw, params->llhw, ccc_cmd_rc);
}

static int do_tm_cmd_ccc_test_wait(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_tm_cmd_params *params = in;
	u8 ccc_cmd_rc;
	static struct dw3000_nfcc_coex_testmode_config conf = {
		.mode = DW3000_CCC_TEST_WAIT,
		.margin_ms = 0,
		.start = 0,
		.end = 0,
	};

	/* Verify the ccc channel attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_CCC_CHANNEL])
		return -EINVAL;
	conf.channel = nla_get_u8(params->nl_attr[DW3000_TM_ATTR_CCC_CHANNEL]);

	/* Verify the ccc time0 attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_CCC_TIME0])
		return -EINVAL;
	conf.session_time0 =
		nla_get_u32(params->nl_attr[DW3000_TM_ATTR_CCC_TIME0]);

	/* margin_ms is optional */
	if (params->nl_attr[DW3000_TM_ATTR_CCC_MARGIN_MS])
		conf.margin_ms = nla_get_u32(
			params->nl_attr[DW3000_TM_ATTR_CCC_MARGIN_MS]);

	if (dw3000_nfcc_coex_testmode_config(dw, &conf))
		ccc_cmd_rc = 0;
	else
		ccc_cmd_rc = 1;

	return tm_ccc_cmd_done(dw, params->llhw, ccc_cmd_rc);
}

static int do_tm_cmd_ccc_test_late(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_tm_cmd_params *params = in;
	u8 ccc_cmd_rc;
	static struct dw3000_nfcc_coex_testmode_config conf = {
		.mode = DW3000_CCC_TEST_LATE,
		.margin_ms = 0,
		.RRcount = 0,
		.start = 0,
		.end = 0,
	};

	/* Verify the ccc channel attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_CCC_CHANNEL])
		return -EINVAL;
	conf.channel = nla_get_u8(params->nl_attr[DW3000_TM_ATTR_CCC_CHANNEL]);

	/* Verify the ccc time0 attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_CCC_TIME0])
		return -EINVAL;
	conf.session_time0 =
		nla_get_u32(params->nl_attr[DW3000_TM_ATTR_CCC_TIME0]);

	/* margin_ms is mandatory */
	if (!params->nl_attr[DW3000_TM_ATTR_CCC_MARGIN_MS])
		return -EINVAL;
	conf.margin_ms =
		nla_get_u32(params->nl_attr[DW3000_TM_ATTR_CCC_MARGIN_MS]);

	/* RRcount is optional */
	if (params->nl_attr[DW3000_TM_ATTR_CCC_RR_COUNT])
		conf.RRcount = nla_get_u32(
			params->nl_attr[DW3000_TM_ATTR_CCC_RR_COUNT]);

	if (dw3000_nfcc_coex_testmode_config(dw, &conf))
		ccc_cmd_rc = 0;
	else
		ccc_cmd_rc = 1;

	return tm_ccc_cmd_done(dw, params->llhw, ccc_cmd_rc);
}

static int do_tm_cmd_ccc_test_conflict(struct dw3000 *dw, const void *in,
				       void *out)
{
	const struct do_tm_cmd_params *params = in;
	u8 ccc_cmd_rc;
	static struct dw3000_nfcc_coex_testmode_config conf = {
		.mode = DW3000_CCC_TEST_CONFLICT,
		.RRcount = 0,
		.start = 0,
		.end = 0,
	};

	/* Verify the ccc channel attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_CCC_CHANNEL])
		return -EINVAL;
	conf.channel = nla_get_u8(params->nl_attr[DW3000_TM_ATTR_CCC_CHANNEL]);

	/* Verify the ccc time0 attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_CCC_TIME0])
		return -EINVAL;
	conf.session_time0 =
		nla_get_u32(params->nl_attr[DW3000_TM_ATTR_CCC_TIME0]);

	/* conflit_slot_idx is mandatory */
	if (!params->nl_attr[DW3000_TM_ATTR_CCC_CONFLICT_SLOT_IDX])
		return -EINVAL;
	conf.conflit_slot_idx = nla_get_u32(
		params->nl_attr[DW3000_TM_ATTR_CCC_CONFLICT_SLOT_IDX]);

	/* RRcount is optional */
	if (params->nl_attr[DW3000_TM_ATTR_CCC_RR_COUNT])
		conf.RRcount = nla_get_u32(
			params->nl_attr[DW3000_TM_ATTR_CCC_RR_COUNT]);

	if (dw3000_nfcc_coex_testmode_config(dw, &conf))
		ccc_cmd_rc = 0;
	else
		ccc_cmd_rc = 1;

	return tm_ccc_cmd_done(dw, params->llhw, ccc_cmd_rc);
}

static int do_tm_cmd_ccc_test_offset(struct dw3000 *dw, const void *in,
				     void *out)
{
	const struct do_tm_cmd_params *params = in;
	u8 ccc_cmd_rc;
	static struct dw3000_nfcc_coex_testmode_config conf = {
		.mode = DW3000_CCC_TEST_SLEEP_OFFSET,
		.offset_ms = 0,
		.start = 0,
		.end = 0,
	};

	/* Verify the ccc channel attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_CCC_CHANNEL])
		return -EINVAL;
	conf.channel = nla_get_u8(params->nl_attr[DW3000_TM_ATTR_CCC_CHANNEL]);

	/* Verify the ccc time0 attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_CCC_TIME0])
		return -EINVAL;
	conf.session_time0 =
		nla_get_u32(params->nl_attr[DW3000_TM_ATTR_CCC_TIME0]);

	/* offset_ms is mandatory */
	if (!params->nl_attr[DW3000_TM_ATTR_CCC_OFFSET_MS])
		return -EINVAL;
	conf.offset_ms =
		nla_get_u32(params->nl_attr[DW3000_TM_ATTR_CCC_OFFSET_MS]);
	if (dw3000_nfcc_coex_testmode_config(dw, &conf))
		ccc_cmd_rc = 0;
	else
		ccc_cmd_rc = 1;

	return tm_ccc_cmd_done(dw, params->llhw, ccc_cmd_rc);
}

static int do_tm_cmd_ccc_read_tlvs(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_tm_cmd_params *params = in;
	u8 ccc_cmd_rc;

	/* TODO: read tlvs values */
	ccc_cmd_rc = 1;

	return tm_ccc_cmd_done(dw, params->llhw, ccc_cmd_rc);
}

static int do_tm_cmd_ccc_write_tlvs(struct dw3000 *dw, const void *in,
				    void *out)
{
	const struct do_tm_cmd_params *params = in;
	u8 ccc_cmd_rc;

	/* TODO: write tlvs values */
	ccc_cmd_rc = 1;

	return tm_ccc_cmd_done(dw, params->llhw, ccc_cmd_rc);
}

static int do_tm_cmd_deep_sleep(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_tm_cmd_params *params = in;
	u32 delay;

	/* Verify the delay attribute exists */
	if (!params->nl_attr[DW3000_TM_ATTR_DEEP_SLEEP_DELAY_MS])
		return -EINVAL;
	delay = nla_get_u32(
		params->nl_attr[DW3000_TM_ATTR_DEEP_SLEEP_DELAY_MS]);
	return dw3000_go_to_deep_sleep_and_wakeup_after_ms(dw, delay);
}

int dw3000_tm_cmd(struct mcps802154_llhw *llhw, void *data, int len)
{
	struct dw3000 *dw = llhw->priv;
	struct do_tm_cmd_params params;
	struct dw3000_stm_command cmd = { NULL, &params, NULL };
	struct nlattr *attr[DW3000_TM_ATTR_MAX + 1];

	static const cmd_func cmds[__DW3000_TM_CMD_AFTER_LAST] = {
		[DW3000_TM_CMD_START_RX_DIAG] = do_tm_cmd_start_rx_diag,
		[DW3000_TM_CMD_STOP_RX_DIAG] = do_tm_cmd_stop_rx_diag,
		[DW3000_TM_CMD_GET_RX_DIAG] = do_tm_cmd_get_rx_diag,
		[DW3000_TM_CMD_CLEAR_RX_DIAG] = do_tm_cmd_clear_rx_diag,
		[DW3000_TM_CMD_START_TX_CWTONE] = do_tm_cmd_start_tx_cwtone,
		[DW3000_TM_CMD_STOP_TX_CWTONE] = do_tm_cmd_stop_tx_cwtone,
		[DW3000_TM_CMD_OTP_READ] = do_tm_cmd_otp_read,
		[DW3000_TM_CMD_OTP_WRITE] = do_tm_cmd_otp_write,
		[DW3000_TM_CMD_CCC_START] = do_tm_cmd_ccc_start,
		[DW3000_TM_CMD_CCC_TEST_SCRATCH] = do_tm_cmd_ccc_test_scratch,
		[DW3000_TM_CMD_CCC_TEST_SPI1] = do_tm_cmd_ccc_test_spi1,
		[DW3000_TM_CMD_CCC_TEST_SPI2] = do_tm_cmd_ccc_test_spi2,
		[DW3000_TM_CMD_CCC_READ_TLVS] = do_tm_cmd_ccc_read_tlvs,
		[DW3000_TM_CMD_CCC_WRITE_TLVS] = do_tm_cmd_ccc_write_tlvs,
		[DW3000_TM_CMD_CCC_TEST_DIRECT] = do_tm_cmd_ccc_test_direct,
		[DW3000_TM_CMD_CCC_TEST_WAIT] = do_tm_cmd_ccc_test_wait,
		[DW3000_TM_CMD_CCC_TEST_LATE] = do_tm_cmd_ccc_test_late,
		[DW3000_TM_CMD_CCC_TEST_CONFLICT] = do_tm_cmd_ccc_test_conflict,
		[DW3000_TM_CMD_CCC_TEST_OFFSET] = do_tm_cmd_ccc_test_offset,
		[DW3000_TM_CMD_DEEP_SLEEP] = do_tm_cmd_deep_sleep,
	};
	u32 tm_cmd;
	int ret;

	ret = nla_parse(attr, DW3000_TM_ATTR_MAX, data, len, dw3000_tm_policy,
			NULL);
	if (ret)
		return ret;

	if (!attr[DW3000_TM_ATTR_CMD])
		return -EINVAL;

	params = (struct do_tm_cmd_params){ llhw, attr };

	tm_cmd = nla_get_u32(attr[DW3000_TM_ATTR_CMD]);
	/* Share the testmode's command with each thread-safe function */
	trace_dw3000_tm_cmd(dw, tm_cmd);

	if (tm_cmd < __DW3000_TM_CMD_AFTER_LAST && cmds[tm_cmd]) {
		cmd.cmd = cmds[tm_cmd];
		ret = dw3000_enqueue_generic(dw, &cmd);
	} else
		ret = -EOPNOTSUPP;

	trace_dw3000_return_int(dw, ret);
	return ret;
}
