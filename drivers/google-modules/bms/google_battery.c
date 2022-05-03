/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#ifdef CONFIG_PM_SLEEP
#define SUPPORT_PM_SLEEP 1
#endif

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/slab.h>
#include "gbms_power_supply.h"
#include "pmic-voter.h" /* TODO(b/163679860): use gvotables */
#include "google_bms.h"
#include "google_psy.h"
#include "qmath.h"
#include <misc/logbuffer.h>
#include <crypto/hash.h>

#include <linux/debugfs.h>

#define BATT_DELAY_INIT_MS		250
#define BATT_WORK_FAST_RETRY_CNT	30
#define BATT_WORK_FAST_RETRY_MS		1000
#define BATT_WORK_DEBOUNCE_RETRY_MS	3000
#define BATT_WORK_ERROR_RETRY_MS	1000

#define DEFAULT_BATT_FAKE_CAPACITY		50
#define DEFAULT_BATT_UPDATE_INTERVAL		30000
#define DEFAULT_BATT_DRV_RL_SOC_THRESHOLD	97
#define DEFAULT_BD_TRICKLE_RL_SOC_THRESHOLD	90
#define DEFAULT_BD_TRICKLE_RESET_SEC		(5 * 60)
#define DEFAULT_HIGH_TEMP_UPDATE_THRESHOLD	550

#define DEFAULT_HEALTH_SAFETY_MARGIN	(30 * 60)

#define MSC_ERROR_UPDATE_INTERVAL		5000
#define MSC_DEFAULT_UPDATE_INTERVAL		30000

/* qual time is 0 minutes of charge or 0% increase in SOC */
#define DEFAULT_CHG_STATS_MIN_QUAL_TIME		0
#define DEFAULT_CHG_STATS_MIN_DELTA_SOC		0

/* Voters */
#define MSC_LOGIC_VOTER	"msc_logic"
#define SW_JEITA_VOTER	"sw_jeita"
#define RL_STATE_VOTER	"rl_state"
#define MSC_HEALTH_VOTER "chg_health"

#define UICURVE_MAX	3

/* sync from google/logbuffer.c */
#define LOG_BUFFER_ENTRY_SIZE	256

/* Initial data of history cycle count */
#define HCC_DEFAULT_DELTA_CYCLE_CNT	10
#define HCC_DELAY_INIT_MS	30000

/* Interval value used when health is settings disabled when not running */
#define CHG_DEADLINE_SETTING -1
/* Internal value used when health is settings disabled while running */
#define CHG_DEADLINE_SETTING_STOP -2

#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX	"androidboot."
#define DEV_SN_LENGTH		20
static char dev_sn[DEV_SN_LENGTH];
module_param_string(serialno, dev_sn, DEV_SN_LENGTH, 0000);

#if (GBMS_CCBIN_BUCKET_COUNT < 1) || (GBMS_CCBIN_BUCKET_COUNT > 100)
#error "GBMS_CCBIN_BUCKET_COUNT needs to be a value from 1-100"
#endif

#define get_boot_sec() div_u64(ktime_to_ns(ktime_get_boottime()), NSEC_PER_SEC)

struct ssoc_uicurve {
	qnum_t real;
	qnum_t ui;
};

enum batt_rl_status {
	BATT_RL_STATUS_NONE = 0,
	BATT_RL_STATUS_DISCHARGE = -1,
	BATT_RL_STATUS_RECHARGE = 1,
};

#define RL_DELTA_SOC_MAX	8

struct batt_ssoc_rl_state {
	/* rate limiter state */
	qnum_t rl_ssoc_target;
	ktime_t rl_ssoc_last_update;

	/* rate limiter flags */
	bool rl_no_zero;
	int rl_fast_track;
	int rl_track_target;
	/* rate limiter config */
	int rl_delta_max_time;
	qnum_t rl_delta_max_soc;

	int rl_delta_soc_ratio[RL_DELTA_SOC_MAX];
	qnum_t rl_delta_soc_limit[RL_DELTA_SOC_MAX];
	int rl_delta_soc_cnt;

	qnum_t rl_ft_low_limit;
	qnum_t rl_ft_delta_limit;
};

#define SSOC_STATE_BUF_SZ 128
#define SSOC_DELTA 3

struct batt_ssoc_state {
	/* output of gauge data filter */
	qnum_t ssoc_gdf;
	/*  UI Curves */
	int ssoc_curve_type;    /*<0 dsg, >0 chg, 0? */
	struct ssoc_uicurve ssoc_curve[UICURVE_MAX];
	qnum_t ssoc_uic;
	/* output of rate limiter */
	qnum_t ssoc_rl;
	struct batt_ssoc_rl_state ssoc_rl_state;
	int ssoc_delta;

	/* output of rate limiter */
	int rl_rate;
	int rl_last_ssoc;
	ktime_t rl_last_update;

	/* connected or disconnected */
	ktime_t disconnect_time;
	int buck_enabled;

	/* recharge logic */
	int rl_soc_threshold;
	enum batt_rl_status rl_status;

	/* trickle defender */
	bool bd_trickle_enable;
	int bd_trickle_recharge_soc;
	int bd_trickle_cnt;
	bool bd_trickle_dry_run;
	u32 bd_trickle_reset_sec;

	/* buff */
	char ssoc_state_cstr[SSOC_STATE_BUF_SZ];
};

struct gbatt_ccbin_data {
	u16 count[GBMS_CCBIN_BUCKET_COUNT];
	char cyc_ctr_cstr[GBMS_CCBIN_CSTR_SIZE];
	struct mutex lock;
	int prev_soc;
};

#define DEFAULT_RES_TEMP_HIGH	390
#define DEFAULT_RES_TEMP_LOW	350
#define DEFAULT_RES_SSOC_THR	75
#define DEFAULT_RES_FILT_LEN	10

struct batt_res {
	bool estimate_requested;

	/* samples */
	int sample_accumulator;
	int sample_count;

	/* registers */
	int filter_count;
	int resistance_avg;

	/* configuration */
	int estimate_filter;
	int ssoc_threshold;
	int res_temp_low;
	int res_temp_high;
};

enum batt_paired_state {
	BATT_PAIRING_WRITE_ERROR = -4,
	BATT_PAIRING_READ_ERROR = -3,
	BATT_PAIRING_MISMATCH = -2,
	BATT_PAIRING_DISABLED = -1,
	BATT_PAIRING_ENABLED = 0,
	BATT_PAIRING_PAIRED = 1,
	BATT_PAIRING_RESET = 2,
};

enum batt_lfcollect_status {
	BATT_LFCOLLECT_NOT_AVAILABLE = -1,
	BATT_LFCOLLECT_DISABLED = 0,
	BATT_LFCOLLECT_ENABLED = 1,
	BATT_LFCOLLECT_COLLECT = 2,
};

/* battery driver state */
struct batt_drv {
	struct device *device;
	struct power_supply *psy;

	const char *fg_psy_name;
	struct power_supply *fg_psy;
	struct notifier_block fg_nb;

	struct delayed_work init_work;
	struct delayed_work batt_work;
	struct delayed_work init_hist_work;

	struct wakeup_source *msc_ws;
	struct wakeup_source *batt_ws;
	struct wakeup_source *taper_ws;
	struct wakeup_source *poll_ws;
	bool hold_taper_ws;

	/* TODO: b/111407333, will likely need to adjust SOC% on wakeup */
	bool init_complete;
	bool resume_complete;
	bool batt_present;
	u32 fake_battery_present;

	struct mutex batt_lock;
	struct mutex chg_lock;

	/* battery work */
	int fg_status;
	int batt_fast_update_cnt;
	u32 batt_update_interval;
	/* update high temperature in time */
	int batt_temp;
	u32 batt_update_high_temp_threshold;
	/* fake battery temp for thermal testing */
	int fake_temp;
	/* triger for recharge logic next update from charger */
	bool batt_full;
	struct batt_ssoc_state ssoc_state;
	/* bin count */
	struct gbatt_ccbin_data cc_data;
	/* fg cycle count */
	int cycle_count;

	/* props */
	int soh;
	int fake_capacity;
	int batt_health;
	bool dead_battery;
	int capacity_level;
	bool chg_done;

	/* temp outside the charge table */
	int jeita_stop_charging;
	/* health based charging */
	struct batt_chg_health chg_health;

	/* MSC charging */
	u32 battery_capacity;
	struct gbms_chg_profile chg_profile;
	union gbms_charger_state chg_state;

	int temp_idx;
	int vbatt_idx;
	int checked_cv_cnt;
	int checked_ov_cnt;
	int checked_tier_switch_cnt;
	int last_log_cnt;

	int fv_uv;
	int cc_max;
	int msc_update_interval;

	bool disable_votes;
	struct votable	*msc_interval_votable;
	struct votable	*fcc_votable;
	struct votable	*fv_votable;

	/* FAN level */
	struct votable	*fan_level_votable;
	int fan_last_level;

	/* stats */
	int msc_state;
	int msc_irdrop_state;
	struct mutex stats_lock;
	struct gbms_charging_event ce_data;
	struct gbms_charging_event ce_qual;
	uint32_t chg_sts_qual_time;
	uint32_t chg_sts_delta_soc;

	/* health charge margin time */
	int health_safety_margin;

	/* time to full */
	struct batt_ttf_stats ttf_stats;
	bool ttf_debounce;

	/* logging */
	struct logbuffer *ssoc_log;

	/* thermal */
	struct thermal_zone_device *tz_dev;

	/* Resistance */
	struct batt_res res_state;

	/* used to detect battery replacements and reset statistics */
	enum batt_paired_state pairing_state;

	/* collect battery history/lifetime data (history) */
	enum batt_lfcollect_status blf_state;
	int hist_delta_cycle_cnt;
	int hist_data_max_cnt;
	int hist_data_saved_cnt;
	void *hist_data;

	/* Battery device info */
	u8 dev_info_check[GBMS_DINF_LEN];

	/* History Device */
	struct gbms_storage_device *history;

	/* Fan control */
	int fan_level;
};

static int gbatt_get_temp(const struct batt_drv *batt_drv, int *temp);

static int batt_chg_tier_stats_cstr(char *buff, int size,
				    const struct gbms_ce_tier_stats *tier_stat,
				    bool verbose);

static inline void batt_update_cycle_count(struct batt_drv *batt_drv)
{
	batt_drv->cycle_count = GPSY_GET_PROP(batt_drv->fg_psy,
					      POWER_SUPPLY_PROP_CYCLE_COUNT);
}

static int google_battery_tz_get_cycle_count(void *data, int *cycle_count)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	if (!cycle_count) {
		pr_err("Cycle Count NULL");
		return -EINVAL;
	}

	if (batt_drv->cycle_count < 0)
		return batt_drv->cycle_count;

	*cycle_count = batt_drv->cycle_count;

	return 0;
}

static int psy_changed(struct notifier_block *nb,
		       unsigned long action, void *data)
{
	struct power_supply *psy = data;
	struct batt_drv *batt_drv = container_of(nb, struct batt_drv, fg_nb);

	pr_debug("name=%s evt=%lu\n", psy->desc->name, action);

	if ((action != PSY_EVENT_PROP_CHANGED) ||
	    (psy == NULL) || (psy->desc == NULL) || (psy->desc->name == NULL))
		return NOTIFY_OK;

	if (action == PSY_EVENT_PROP_CHANGED &&
	    (!strcmp(psy->desc->name, batt_drv->fg_psy_name))) {
		mod_delayed_work(system_wq, &batt_drv->batt_work, 0);
	}

	return NOTIFY_OK;
}

/* ------------------------------------------------------------------------- */


#define BATT_PRLOG_DEBUG  0
#define BATT_PRLOG_ALWAYS 1
#define BATT_PRLOG_LAST_LOG_COUNT 10

static int debug_printk_prlog = LOGLEVEL_INFO;

static inline int batt_prlog_level(bool level)
{
	return level ? BATT_PRLOG_ALWAYS : BATT_PRLOG_DEBUG;
}

__printf(2,3)
static void batt_prlog__(int info_level, const char *format, ...)
{
	const int level = info_level == BATT_PRLOG_ALWAYS ? LOGLEVEL_INFO : LOGLEVEL_DEBUG;

	if (level <= debug_printk_prlog) {
		va_list args;

		va_start(args, format);
			vprintk(format, args);
		va_end(args);
	}

}

#define batt_prlog(l, fmt, ...) batt_prlog__(l, pr_fmt(fmt), ##__VA_ARGS__)

/* ------------------------------------------------------------------------- */

#define SSOC_TRUE 15
#define SSOC_SPOOF 95
#define SSOC_FULL 100
#define UICURVE_BUF_SZ	(UICURVE_MAX * 15 + 1)
#define SSOC_HIGH_SOC 90

enum ssoc_uic_type {
	SSOC_UIC_TYPE_DSG  = -1,
	SSOC_UIC_TYPE_NONE = 0,
	SSOC_UIC_TYPE_CHG  = 1,
};

const qnum_t ssoc_point_true = qnum_rconst(SSOC_TRUE);
const qnum_t ssoc_point_spoof = qnum_rconst(SSOC_SPOOF);
const qnum_t ssoc_point_full = qnum_rconst(SSOC_FULL);

static struct ssoc_uicurve chg_curve[UICURVE_MAX] = {
	{ ssoc_point_true, ssoc_point_true },
	{ ssoc_point_spoof, ssoc_point_spoof },
	{ ssoc_point_full, ssoc_point_full },
};

static struct ssoc_uicurve dsg_curve[UICURVE_MAX] = {
	{ ssoc_point_true, ssoc_point_true },
	{ ssoc_point_spoof, ssoc_point_full },
	{ ssoc_point_full, ssoc_point_full },
};

static char *ssoc_uicurve_cstr(char *buff, size_t size,
			       struct ssoc_uicurve *curve)
{
	int i, len = 0;

	for (i = 0; i < UICURVE_MAX ; i++) {
		len += scnprintf(&buff[len], size - len,
				"[" QNUM_CSTR_FMT " " QNUM_CSTR_FMT "]",
				qnum_toint(curve[i].real),
				qnum_fracdgt(curve[i].real),
				qnum_toint(curve[i].ui),
				qnum_fracdgt(curve[i].ui));
		if (len >= size)
			break;
	}

	buff[len] = 0;
	return buff;
}

/* NOTE: no bounds checks on this one */
static int ssoc_uicurve_find(qnum_t real, struct ssoc_uicurve *curve)
{
	int i;

	for (i = 1; i < UICURVE_MAX ; i++) {
		if (real == curve[i].real)
			return i;
		if (real > curve[i].real)
			continue;
		break;
	}

	return i-1;
}

static qnum_t ssoc_uicurve_map(qnum_t real, struct ssoc_uicurve *curve)
{
	qnum_t slope = 0, delta_ui, delta_re;
	int i;

	if (real < curve[0].real)
		return real;
	if (real >= curve[UICURVE_MAX - 1].ui)
		return curve[UICURVE_MAX - 1].ui;

	i = ssoc_uicurve_find(real, curve);
	if (curve[i].real == real)
		return curve[i].ui;

	delta_ui = curve[i + 1].ui - curve[i].ui;
	delta_re =  curve[i + 1].real - curve[i].real;
	if (delta_re)
		slope = qnum_div(delta_ui, delta_re);

	return curve[i].ui + qnum_mul(slope, (real - curve[i].real));
}

/* "optimized" to work on 3 element curves */
static void ssoc_uicurve_splice(struct ssoc_uicurve *curve, qnum_t real,
				qnum_t ui)
{
	if (real < curve[0].real || real > curve[2].real)
		return;

#if UICURVE_MAX != 3
#error ssoc_uicurve_splice() only support UICURVE_MAX == 3
#endif

	/* splice only when real is within the curve range */
	curve[1].real = real;
	curve[1].ui = ui;
}

static void ssoc_uicurve_dup(struct ssoc_uicurve *dst,
			     struct ssoc_uicurve *curve)
{
	if (dst != curve)
		memcpy(dst, curve, sizeof(*dst)*UICURVE_MAX);
}


/* ------------------------------------------------------------------------- */

/* could also use the rate of change for this */
static qnum_t ssoc_rl_max_delta(const struct batt_ssoc_rl_state *rls,
				int bucken, ktime_t delta_time)
{
	int i;
	const qnum_t max_delta = ((qnumd_t)rls->rl_delta_max_soc * delta_time) /
				  rls->rl_delta_max_time;

	if (rls->rl_fast_track)
		return max_delta;

	/* might have one table for charging and one for discharging */
	for (i = 0; i < rls->rl_delta_soc_cnt; i++) {
		if (rls->rl_delta_soc_limit[i] == 0)
			break;

		if (rls->rl_ssoc_target < rls->rl_delta_soc_limit[i])
			return ((qnumd_t)max_delta * 10) /
				rls->rl_delta_soc_ratio[i];
	}

	return max_delta;
}

static qnum_t ssoc_apply_rl(struct batt_ssoc_state *ssoc)
{
	const ktime_t now = get_boot_sec();
	struct batt_ssoc_rl_state *rls = &ssoc->ssoc_rl_state;
	qnum_t rl_val;

	/* track ssoc_uic when buck is enabled or the minimum value of uic */
	if (ssoc->buck_enabled ||
	    (!ssoc->buck_enabled && ssoc->ssoc_uic < rls->rl_ssoc_target))
		rls->rl_ssoc_target = ssoc->ssoc_uic;

	/* sanity on the target */
	if (rls->rl_ssoc_target > qnum_fromint(100))
		rls->rl_ssoc_target = qnum_fromint(100);
	if (rls->rl_ssoc_target < qnum_fromint(0))
		rls->rl_ssoc_target = qnum_fromint(0);

	/* closely track target */
	if (rls->rl_track_target) {
		rl_val = rls->rl_ssoc_target;
	} else {
		qnum_t step;
		const ktime_t delta_time = now - rls->rl_ssoc_last_update;
		const ktime_t max_delta = ssoc_rl_max_delta(rls,
							   ssoc->buck_enabled,
							   delta_time);

		/* apply the rate limiter, delta_soc to target */
		step = rls->rl_ssoc_target - ssoc->ssoc_rl;
		if (step < -max_delta)
			step = -max_delta;
		else if (step > max_delta)
			step = max_delta;

		rl_val = ssoc->ssoc_rl + step;
	}

	/* do not increase when not connected */
	if (!ssoc->buck_enabled && rl_val > ssoc->ssoc_rl)
		rl_val = ssoc->ssoc_rl;

	/* will report 0% when rl_no_zero clears */
	if (rls->rl_no_zero && rl_val <= qnum_fromint(1))
		rl_val = qnum_fromint(1);

	rls->rl_ssoc_last_update = now;
	return rl_val;
}

/* ------------------------------------------------------------------------- */

static int ssoc_get_real_raw(const struct batt_ssoc_state *ssoc)
{
	return ssoc->ssoc_gdf;
}

/* a statement :-) */
static qnum_t ssoc_get_capacity_raw(const struct batt_ssoc_state *ssoc)
{
	return ssoc->ssoc_rl;
}

static int ssoc_get_real(const struct batt_ssoc_state *ssoc)
{
	const qnum_t real_raw = ssoc_get_real_raw(ssoc);

	return qnum_toint(real_raw);
}

#define SOC_ROUND_BASE	0.5

/* reported to userspace: call while holding batt_lock */
static int ssoc_get_capacity(const struct batt_ssoc_state *ssoc)
{
	const qnum_t raw = ssoc_get_capacity_raw(ssoc);

	return qnum_roundint(raw, SOC_ROUND_BASE);
}

/* ------------------------------------------------------------------------- */

static void dump_ssoc_state(struct batt_ssoc_state *ssoc_state,
			    struct logbuffer *log)
{
	char buff[UICURVE_BUF_SZ] = { 0 };

	scnprintf(ssoc_state->ssoc_state_cstr,
		  sizeof(ssoc_state->ssoc_state_cstr),
		  "SSOC: l=%d%% gdf=%d.%02d uic=%d.%02d rl=%d.%02d ct=%d curve:%s rls=%d bd_cnt=%d",
		  ssoc_get_capacity(ssoc_state),
		  qnum_toint(ssoc_state->ssoc_gdf),
		  qnum_fracdgt(ssoc_state->ssoc_gdf),
		  qnum_toint(ssoc_state->ssoc_uic),
		  qnum_fracdgt(ssoc_state->ssoc_uic),
		  qnum_toint(ssoc_state->ssoc_rl),
		  qnum_fracdgt(ssoc_state->ssoc_rl),
		  ssoc_state->ssoc_curve_type,
		  ssoc_uicurve_cstr(buff, sizeof(buff), ssoc_state->ssoc_curve),
		  ssoc_state->rl_status,
		  ssoc_state->bd_trickle_cnt);

	logbuffer_log(log, "%s", ssoc_state->ssoc_state_cstr);
	pr_debug("%s\n", ssoc_state->ssoc_state_cstr);
}

/* ------------------------------------------------------------------------- */

/* call while holding batt_lock */
static void ssoc_update(struct batt_ssoc_state *ssoc, qnum_t soc)
{
	struct batt_ssoc_rl_state *rls =  &ssoc->ssoc_rl_state;
	qnum_t delta;

	/* low pass filter */
	ssoc->ssoc_gdf = soc;
	/* spoof UI @ EOC */
	ssoc->ssoc_uic = ssoc_uicurve_map(ssoc->ssoc_gdf, ssoc->ssoc_curve);

	/* first target is current UIC */
	if (rls->rl_ssoc_target == -1) {
		rls->rl_ssoc_target = ssoc->ssoc_uic;
		ssoc->ssoc_rl = ssoc->ssoc_uic;
	}

	/* enable fast track when target under configured limit */
	rls->rl_fast_track |= rls->rl_ssoc_target < rls->rl_ft_low_limit;

	/*
	 * delta fast tracking during charge
	 * NOTE: might use the stats from TTF to determine the maximum rate
	 */
	delta = rls->rl_ssoc_target - ssoc->ssoc_rl;
	if (rls->rl_ft_delta_limit && ssoc->buck_enabled && delta > 0) {
		/* only when SOC increase */
		rls->rl_fast_track |= delta > rls->rl_ft_delta_limit;
	} else if (rls->rl_ft_delta_limit && !ssoc->buck_enabled && delta < 0) {
		/* enable fast track when target under configured limit */
		rls->rl_fast_track |= -delta > rls->rl_ft_delta_limit;
	}

	/*
	 * Right now a simple test on target metric falling under 0.5%
	 * TODO: add a filter that decrements no_zero when a specific
	 * condition is met (ex rl_ssoc_target < 1%).
	 */
	if (rls->rl_no_zero)
		rls->rl_no_zero = rls->rl_ssoc_target > qnum_from_q8_8(128);

	/*  monotonicity and rate of change */
	ssoc->ssoc_rl = ssoc_apply_rl(ssoc);
}

/*
 * Maxim could need:
 *	1fh AvCap, 10h FullCap. 23h FullCapNom
 * QC could need:
 *	QG_CC_SOC, QG_Raw_SOC, QG_Bat_SOC, QG_Sys_SOC, QG_Mon_SOC
 */
static int ssoc_work(struct batt_ssoc_state *ssoc_state,
		     struct power_supply *fg_psy)
{
	int soc_q8_8;
	qnum_t soc_raw;

	/*
	 * TODO: GBMS_PROP_CAPACITY_RAW should return a qnum_t
	 * TODO: add an array here configured in DT with the properties
	 * to query and their weights, make soc_raw come from fusion.
	 */
	soc_q8_8 = GPSY_GET_PROP(fg_psy, GBMS_PROP_CAPACITY_RAW);
	if (soc_q8_8 < 0)
		return -EINVAL;

	/*
	 * soc_raw can come from fusion:
	 *    soc_raw = m1 * w1 + m2 * w2 + ...
	 *
	 * where m1, m2 are gauge metrics, w1,w1 are weights that change
	 * with temperature, state of charge, battery health etc.
	 */
	soc_raw = qnum_from_q8_8(soc_q8_8);

	ssoc_update(ssoc_state, soc_raw);
	return 0;
}

static void ssoc_change_curve_at_gdf(struct batt_ssoc_state *ssoc_state,
				     qnum_t gdf, qnum_t capacity,
				     enum ssoc_uic_type type)
{
	struct ssoc_uicurve *new_curve;

	new_curve = (type == SSOC_UIC_TYPE_DSG) ? dsg_curve : chg_curve;
	ssoc_uicurve_dup(ssoc_state->ssoc_curve, new_curve);
	ssoc_state->ssoc_curve_type = type;

	/* splice at (->ssoc_gdf,->ssoc_rl) because past spoof */
	ssoc_uicurve_splice(ssoc_state->ssoc_curve, gdf, capacity);
}

/*
 * Called on connect and disconnect to adjust the UI curve. On disconnect
 * splice at GDF less a fixed delta while UI is at 100% (i.e. in RL) to
 * avoid showing 100% for "too long" after disconnect.
 */
static void ssoc_change_curve(struct batt_ssoc_state *ssoc_state, qnum_t delta,
			      enum ssoc_uic_type type)
{
	qnum_t ssoc_level = ssoc_get_capacity(ssoc_state);
	qnum_t gdf = ssoc_state->ssoc_gdf; /* actual battery level */

	/* force dsg curve when connect/disconnect with battery at 100% */
	if (ssoc_level >= SSOC_FULL) {
		const qnum_t rlt = qnum_fromint(ssoc_state->rl_soc_threshold);

		/* bounds GDF - DELTA to prevent SSOC/GDF from diverging significantly */
		gdf = gdf > rlt ? gdf : rlt;

		type = SSOC_UIC_TYPE_DSG;
		gdf -=  delta;
	}

	/* adjust gdf to update curve[1].real in ssoc_uicurve_splice() */
	if (gdf > ssoc_point_full)
		gdf = ssoc_point_full;

	ssoc_change_curve_at_gdf(ssoc_state, gdf,
				 ssoc_get_capacity_raw(ssoc_state), type);
}

/* Fan levels limits from battery temperature */
#define FAN_BT_LIMIT_NOT_CARE	320
#define FAN_BT_LIMIT_LOW	420
#define FAN_BT_LIMIT_MED	460
#define FAN_BT_LIMIT_HIGH	480
/* Fan levels limits from charge rate */
#define FAN_CHG_LIMIT_LOW	50
#define FAN_CHG_LIMIT_MED	70
/* Fan levels limits from health charge */
#define FAN_HEALTH_LIMIT_LVL	FAN_LVL_LOW

static int fan_bt_calculate_level(const struct batt_drv *batt_drv)
{
	int level, temp, ret;

	ret = gbatt_get_temp(batt_drv, &temp);
	if (ret < 0) {

		if (batt_drv->temp_idx < 2)
			level = FAN_LVL_NOT_CARE;
		else if (batt_drv->temp_idx == 3)
			level = FAN_LVL_MED;
		else
			level = FAN_LVL_HIGH;

		pr_warn("FAN_LEVEL: level=%d from temp_idx=%d (%d)\n",
			level, batt_drv->temp_idx, ret);
		return level;
	}

	if (temp <= FAN_BT_LIMIT_NOT_CARE)
		level = FAN_LVL_NOT_CARE;
	else if (temp <= FAN_BT_LIMIT_LOW)
		level = FAN_LVL_LOW;
	else if (temp <= FAN_BT_LIMIT_MED)
		level = FAN_LVL_LOW;
	else if (temp <= FAN_BT_LIMIT_HIGH)
		level = FAN_LVL_HIGH;
	else
		level = FAN_LVL_ALARM;

	return level;
}

static int fan_calculate_level(const struct batt_drv *batt_drv)
{
	int charging_rate, fan_level, chg_fan_level;
	int health_fan_level = FAN_LVL_NOT_CARE;

	if (batt_drv->jeita_stop_charging == 1)
		return FAN_LVL_ALARM;

	/* defender limits from google_charger */
	fan_level = fan_bt_calculate_level(batt_drv);
	if (batt_drv->cc_max == 0 || batt_drv->battery_capacity == 0)
		return fan_level;

	if (batt_drv->msc_state == MSC_HEALTH_PAUSE ||
	    batt_drv->msc_state == MSC_HEALTH)
		health_fan_level = FAN_HEALTH_LIMIT_LVL;

	if (health_fan_level > fan_level)
		fan_level = health_fan_level;

	/* cc_max should be 0 when disconnected */
	charging_rate = batt_drv->cc_max / batt_drv->battery_capacity / 10;
	if (charging_rate <= FAN_CHG_LIMIT_LOW)
		chg_fan_level = FAN_LVL_LOW;
	else if (charging_rate <= FAN_CHG_LIMIT_MED)
		chg_fan_level = FAN_LVL_MED;
	else
		chg_fan_level = FAN_LVL_HIGH;

	/* Charge rate can increase the level */
	if (chg_fan_level > fan_level)
		fan_level = chg_fan_level;

	return fan_level;
}

static void fan_level_reset(const struct batt_drv *batt_drv)
{

	if (batt_drv->fan_level_votable)
		vote(batt_drv->fan_level_votable, "MSC_BATT", false, 0);
}

static int fan_level_cb(struct votable *votable, void *data,
			int lvl, const char *client)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	if (!batt_drv)
		return 0;

	if (batt_drv->fan_last_level != lvl) {
		pr_debug("FAN_LEVEL %d->%d\n", batt_drv->fan_last_level, lvl);

		batt_drv->fan_last_level = lvl;
		if (batt_drv->psy)
			power_supply_changed(batt_drv->psy);
	}

	return 0;
}
/* ------------------------------------------------------------------------- */

/*
 * enter recharge logic in BATT_RL_STATUS_DISCHARGE on charger_DONE,
 * enter BATT_RL_STATUS_RECHARGE on Fuel Gauge FULL
 * NOTE: batt_rl_update_status() doesn't call this, it flip from DISCHARGE
 * to recharge on its own.
 * NOTE: call holding chg_lock
 * FIX: BatteryDefenderUI different rules when battery defender is enabled
 * @pre rl_status != BATT_RL_STATUS_NONE
 */
static bool batt_rl_enter(struct batt_ssoc_state *ssoc_state,
			  enum batt_rl_status rl_status)
{
	const int rl_current = ssoc_state->rl_status;
	const bool enable = ssoc_state->bd_trickle_enable;
	const bool dry_run = ssoc_state->bd_trickle_dry_run;

	/*
	 * NOTE: NO_OP when RL=DISCHARGE since batt_rl_update_status() flip
	 * between BATT_RL_STATUS_DISCHARGE and BATT_RL_STATUS_RECHARGE
	 * directly.
	 */
	if (rl_current == rl_status || rl_current == BATT_RL_STATUS_DISCHARGE)
		return false;

	/* bd_trickle_cnt -1 if the rl_status change does not happen at 100% */
	if (rl_current == BATT_RL_STATUS_RECHARGE &&
	    rl_status == BATT_RL_STATUS_DISCHARGE) {
		if (ssoc_get_real(ssoc_state) != SSOC_FULL) {
			if (ssoc_state->bd_trickle_cnt > 0)
				ssoc_state->bd_trickle_cnt--;
		}
	}

	/*
	 * NOTE: rl_status transition from *->DISCHARGE on charger FULL (during
	 * charge or at the end of recharge) and transition from
	 * NONE->RECHARGE when battery is full (SOC==100%) before charger is.
	 */
	if (rl_status == BATT_RL_STATUS_DISCHARGE) {
		if (enable && !dry_run && ssoc_state->bd_trickle_cnt > 0) {
			ssoc_change_curve(ssoc_state, 0, SSOC_UIC_TYPE_DSG);
		} else {
			ssoc_uicurve_dup(ssoc_state->ssoc_curve, dsg_curve);
			ssoc_state->ssoc_curve_type = SSOC_UIC_TYPE_DSG;
		}
	}

	ssoc_update(ssoc_state, ssoc_state->ssoc_gdf);
	ssoc_state->rl_status = rl_status;

	return true;
}

static int ssoc_rl_read_dt(struct batt_ssoc_rl_state *rls,
			   struct device_node *node)
{
	u32 tmp, delta_soc[RL_DELTA_SOC_MAX];
	int ret, i;

	ret = of_property_read_u32(node, "google,rl_delta-max-soc", &tmp);
	if (ret == 0)
		rls->rl_delta_max_soc = qnum_fromint(tmp);

	ret = of_property_read_u32(node, "google,rl_delta-max-time", &tmp);
	if (ret == 0)
		rls->rl_delta_max_time = tmp;

	if (!rls->rl_delta_max_soc || !rls->rl_delta_max_time)
		return -EINVAL;

	rls->rl_no_zero = of_property_read_bool(node, "google,rl_no-zero");
	rls->rl_track_target = of_property_read_bool(node,
						     "google,rl_track-target");

	ret = of_property_read_u32(node, "google,rl_ft-low-limit", &tmp);
	if (ret == 0)
		rls->rl_ft_low_limit = qnum_fromint(tmp);

	ret = of_property_read_u32(node, "google,rl_ft-delta-limit", &tmp);
	if (ret == 0)
		rls->rl_ft_delta_limit = qnum_fromint(tmp);

	rls->rl_delta_soc_cnt = of_property_count_elems_of_size(node,
					      "google,rl_soc-limits",
					      sizeof(u32));
	tmp = of_property_count_elems_of_size(node, "google,rl_soc-rates",
					      sizeof(u32));
	if (rls->rl_delta_soc_cnt != tmp || tmp == 0) {
		rls->rl_delta_soc_cnt = 0;
		goto done;
	}

	if (rls->rl_delta_soc_cnt > RL_DELTA_SOC_MAX)
		return -EINVAL;

	ret = of_property_read_u32_array(node, "google,rl_soc-limits",
					 delta_soc,
					 rls->rl_delta_soc_cnt);
	if (ret < 0)
		return ret;

	for (i = 0; i < rls->rl_delta_soc_cnt; i++)
		rls->rl_delta_soc_limit[i] = qnum_fromint(delta_soc[i]);

	ret = of_property_read_u32_array(node, "google,rl_soc-rates",
					 delta_soc,
					 rls->rl_delta_soc_cnt);
	if (ret < 0)
		return ret;

	for (i = 0; i < rls->rl_delta_soc_cnt; i++)
		rls->rl_delta_soc_ratio[i] = delta_soc[i];

done:
	return 0;
}


/*
 * NOTE: might need to use SOC from bootloader as starting point to avoid UI
 * SSOC jumping around or taking long time to coverge. Could technically read
 * charger voltage and estimate SOC% based on empty and full voltage.
 */
static int ssoc_init(struct batt_ssoc_state *ssoc_state,
		     struct device_node *node,
		     struct power_supply *fg_psy)
{
	int ret, capacity;

	ret = ssoc_rl_read_dt(&ssoc_state->ssoc_rl_state, node);
	if (ret < 0)
		ssoc_state->ssoc_rl_state.rl_track_target = 1;
	ssoc_state->ssoc_rl_state.rl_ssoc_target = -1;

	/*
	 * ssoc_work() needs a curve: start with the charge curve to prevent
	 * SSOC% from increasing after a reboot. Curve type must be NONE until
	 * battery knows the charger BUCK_EN state.
	 */
	ssoc_uicurve_dup(ssoc_state->ssoc_curve, chg_curve);
	ssoc_state->ssoc_curve_type = SSOC_UIC_TYPE_NONE;

	ret = ssoc_work(ssoc_state, fg_psy);
	if (ret < 0)
		return -EIO;

	capacity = ssoc_get_capacity(ssoc_state);
	if (capacity >= SSOC_FULL) {
		/* consistent behavior when booting without adapter */
		ssoc_uicurve_dup(ssoc_state->ssoc_curve, dsg_curve);
	} else if (capacity < SSOC_TRUE) {
		/* no split */
	} else if (capacity < SSOC_SPOOF) {
		/* mark the initial point if under spoof */
		ssoc_uicurve_splice(ssoc_state->ssoc_curve,
						ssoc_state->ssoc_gdf,
						ssoc_state->ssoc_rl);

	}

	return 0;
}

/* ------------------------------------------------------------------------- */

/*
 * just reset state, no PS notifications no changes in the UI curve. This is
 * called on startup and on disconnect when the charge driver state is reset
 * NOTE: call holding chg_lock
 */
static void batt_rl_reset(struct batt_drv *batt_drv)
{
	batt_drv->ssoc_state.rl_status = BATT_RL_STATUS_NONE;
}

/*
 * RL recharge: call after SSOC work, restart charging when gdf hit the
 * recharge threshold.
 * NOTE: call holding chg_lock
 */
static void batt_rl_update_status(struct batt_drv *batt_drv)
{
	struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	const bool bd_dry_run = ssoc_state->bd_trickle_dry_run;
	const int bd_cnt = ssoc_state->bd_trickle_cnt;
	int soc, rl_soc_threshold;

	/* already in _RECHARGE or _NONE, done */
	if (ssoc_state->rl_status != BATT_RL_STATUS_DISCHARGE)
		return;
	/* no threashold (why I am here???) */
	if (!ssoc_state->rl_soc_threshold)
		return;
	/* recharge logic work on real soc */
	soc = ssoc_get_real(ssoc_state);

	if (ssoc_state->bd_trickle_enable)
		rl_soc_threshold = ((bd_cnt > 0) && !bd_dry_run) ?
			ssoc_state->bd_trickle_recharge_soc :
			ssoc_state->rl_soc_threshold;
	else
		rl_soc_threshold = ssoc_state->rl_soc_threshold;

	if (soc > rl_soc_threshold)
		return;

	/* change state (will restart charge) on trigger */
	ssoc_state->rl_status = BATT_RL_STATUS_RECHARGE;
	if (batt_drv->psy)
		power_supply_changed(batt_drv->psy);

	ssoc_state->bd_trickle_cnt++;
}

/* ------------------------------------------------------------------------- */

/*
 * msc_logic_health() sync ce_data->ce_health to batt_drv->chg_health
 * . return -EINVAL when the device is not connected to power -ERANGE when
 *   ttf_soc_estimate() returns a negative value (invalid parameters, or
 *   corrupted internal data)
 * . the estimate is 0 when the device is at 100%.
 * . the estimate is negative during debounce, when in overheat, when
 *   custom charge levels are active.
 */
static int batt_ttf_estimate(ktime_t *res, const struct batt_drv *batt_drv)
{
	qnum_t raw_full = ssoc_point_full - qnum_rconst(SOC_ROUND_BASE);
	qnum_t soc_raw = ssoc_get_real_raw(&batt_drv->ssoc_state);
	ktime_t estimate;
	int rc;

	if (batt_drv->ssoc_state.buck_enabled != 1)
		return -EINVAL;

	if (batt_drv->ttf_stats.ttf_fake != -1) {
		estimate = batt_drv->ttf_stats.ttf_fake;
		goto done;
	}

	/* TTF is 0 when UI shows 100% */
	if (ssoc_get_capacity(&batt_drv->ssoc_state) == SSOC_FULL) {
		estimate = 0;
		goto done;
	}

	/* no estimates during debounce or with special profiles */
	if (batt_drv->ttf_debounce ||
	    batt_drv->batt_health == POWER_SUPPLY_HEALTH_OVERHEAT ||
	    batt_drv->chg_state.f.flags & GBMS_CS_FLAG_CCLVL) {
		estimate = -1;
		goto done;
	}

	/*
	 * Handle rounding (removing it from the end)
	 * example: 96.64% with SOC_ROUND_BASE = 0.5 -> UI = 97
	 *    ttf = elap[96] * 0.36 + elap[97] + elap[98] +
	 * 	    elap[99] * (1 - 0.5)
	 *
	 * negative return value (usually) means data corruption
	 */
	rc = ttf_soc_estimate(&estimate, &batt_drv->ttf_stats,
			      &batt_drv->ce_data, soc_raw, raw_full);
	if (rc < 0)
		estimate = -1;

done:
	*res = estimate;
	return 0;
}

/* ------------------------------------------------------------------------- */

static void cev_ts_init(struct gbms_ce_tier_stats *stats, int8_t idx)
{
	stats->vtier_idx = idx;
	stats->temp_idx = -1;
	stats->soc_in = -1;
}

/* CEV = Charging EVent */
static void cev_stats_init(struct gbms_charging_event *ce_data,
			   const struct gbms_chg_profile *profile)
{
	int i;

	memset(ce_data, 0, sizeof(*ce_data));

	ce_data->chg_profile = profile;
	ce_data->charging_stats.voltage_in = -1;
	ce_data->charging_stats.ssoc_in = -1;
	ce_data->charging_stats.voltage_out = -1;
	ce_data->charging_stats.ssoc_out = -1;

	ttf_soc_init(&ce_data->soc_stats);
	ce_data->last_soc = -1;

	for (i = 0; i < GBMS_STATS_TIER_COUNT ; i++)
		cev_ts_init(&ce_data->tier_stats[i], i);

	/* batt_chg_health_stats_close() will fix this */
	cev_ts_init(&ce_data->health_stats, GBMS_STATS_AC_TI_INVALID);
	cev_ts_init(&ce_data->health_pause_stats, GBMS_STATS_AC_TI_PAUSE);

	cev_ts_init(&ce_data->full_charge_stats, GBMS_STATS_AC_TI_FULL_CHARGE);
	cev_ts_init(&ce_data->high_soc_stats, GBMS_STATS_AC_TI_HIGH_SOC);
	cev_ts_init(&ce_data->overheat_stats, GBMS_STATS_BD_TI_OVERHEAT_TEMP);
	cev_ts_init(&ce_data->cc_lvl_stats, GBMS_STATS_BD_TI_CUSTOM_LEVELS);

}

static void batt_chg_stats_start(struct batt_drv *batt_drv)
{
	union gbms_ce_adapter_details ad;
	struct gbms_charging_event *ce_data = &batt_drv->ce_data;
	const ktime_t now = get_boot_sec();
	int vin, cc_in;

	mutex_lock(&batt_drv->stats_lock);
	ad.v = batt_drv->ce_data.adapter_details.v;
	cev_stats_init(ce_data, &batt_drv->chg_profile);
	batt_drv->ce_data.adapter_details.v = ad.v;

	vin = GPSY_GET_PROP(batt_drv->fg_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW);
	ce_data->charging_stats.voltage_in = (vin < 0) ? -1 : vin / 1000;
	ce_data->charging_stats.ssoc_in =
				ssoc_get_capacity(&batt_drv->ssoc_state);
	cc_in = GPSY_GET_PROP(batt_drv->fg_psy,
				POWER_SUPPLY_PROP_CHARGE_COUNTER);
	ce_data->charging_stats.cc_in = (cc_in < 0) ? -1 : cc_in / 1000;

	ce_data->charging_stats.ssoc_out = -1;
	ce_data->charging_stats.voltage_out = -1;

	ce_data->first_update = now;
	ce_data->last_update = now;

	mutex_unlock(&batt_drv->stats_lock);
}

/* call holding stats_lock */
static bool batt_chg_stats_qual(const struct batt_drv *batt_drv)
{
	const struct gbms_charging_event *ce_data = &batt_drv->ce_data;
	const long elap = ce_data->last_update - ce_data->first_update;
	const long ssoc_delta = ce_data->charging_stats.ssoc_out -
				ce_data->charging_stats.ssoc_in;

	return elap >= batt_drv->chg_sts_qual_time ||
	    ssoc_delta >= batt_drv->chg_sts_delta_soc;
}

/* call holding stats_lock */
static void batt_chg_stats_tier(struct gbms_ce_tier_stats *tier,
				int msc_state,
				ktime_t elap)
{
	if (msc_state < 0 || msc_state >= MSC_STATES_COUNT)
		return;

	tier->msc_cnt[msc_state] += 1;
	tier->msc_elap[msc_state] += elap;
}

/* call holding stats_lock */
static void batt_chg_stats_soc_update(struct gbms_charging_event *ce_data,
				      qnum_t soc, ktime_t elap, int tier_index,
				      int cc)
{
	int index;
	const int last_soc = ce_data->last_soc;

	index = qnum_toint(soc);
	if (index < 0)
		index = 0;
	if (index > 100)
		index = 100;
	if (index < last_soc)
		return;

	if (ce_data->soc_stats.elap[index] == 0) {
		ce_data->soc_stats.ti[index] = tier_index;
		ce_data->soc_stats.cc[index] = cc;
	}

	if (last_soc != -1)
		ce_data->soc_stats.elap[last_soc] += elap;

	ce_data->last_soc = index;
}

static void batt_chg_stats_update_tier(const struct batt_drv *const batt_drv,
				       int temp_idx, int ibatt_ma, int temp,
				       ktime_t elap, int cc,
				       struct gbms_ce_tier_stats *tier)
{
	const uint16_t icl_settled = batt_drv->chg_state.f.icl;

	/*
	 * book time to previous msc_state for this tier, there is an
	 * interesting wrinkle here since some tiers (health, full, etc)
	 * might be entered and exited multiple times.
	 */
	batt_chg_stats_tier(tier, batt_drv->msc_state, elap);

	if (tier->soc_in == -1) {
		int soc_in;

		soc_in = GPSY_GET_PROP(batt_drv->fg_psy,
				       GBMS_PROP_CAPACITY_RAW);
		if (soc_in < 0) {
			pr_info("MSC_STAT cannot read soc_in=%d\n", soc_in);
			return;
		}

		tier->temp_idx = temp_idx;

		tier->temp_in = temp;
		tier->temp_min = temp;
		tier->temp_max = temp;

		tier->ibatt_min = ibatt_ma;
		tier->ibatt_max = ibatt_ma;

		tier->icl_min = icl_settled;
		tier->icl_max = icl_settled;

		tier->soc_in = soc_in;
		tier->cc_in = cc;
		tier->cc_total = 0;
	} else {
		const u8 flags = batt_drv->chg_state.f.flags;

		/* crossed temperature tier */
		if (temp_idx != tier->temp_idx)
			tier->temp_idx = -1;

		if (flags & GBMS_CS_FLAG_CC) {
			tier->time_fast += elap;
		} else if (flags & GBMS_CS_FLAG_CV) {
			tier->time_taper += elap;
		} else {
			tier->time_other += elap;
		}

		/*
		 * averages: temp < 100. icl_settled < 3000, sum(ibatt)
		 * is bound to battery capacity, elap in seconds, sums
		 * are stored in an s64. For icl_settled I need a tier
		 * to last for more than ~97M years.
		 */
		if (temp < tier->temp_min)
			tier->temp_min = temp;
		if (temp > tier->temp_max)
			tier->temp_max = temp;
		tier->temp_sum += temp * elap;

		if (icl_settled < tier->icl_min)
			tier->icl_min = icl_settled;
		if (icl_settled > tier->icl_max)
			tier->icl_max = icl_settled;
		tier->icl_sum += icl_settled * elap;

		if (ibatt_ma < tier->ibatt_min)
			tier->ibatt_min = ibatt_ma;
		if (ibatt_ma > tier->ibatt_max)
			tier->ibatt_max = ibatt_ma;
		tier->ibatt_sum += ibatt_ma * elap;

		tier->cc_total = cc - tier->cc_in;
	}

	tier->sample_count += 1;
}

/* call holding stats_lock */
static void batt_chg_stats_update(struct batt_drv *batt_drv, int temp_idx,
				  int tier_idx, int ibatt_ma, int temp,
				  ktime_t elap)
{
	const int soc_real = ssoc_get_real(&batt_drv->ssoc_state);
	const int msc_state = batt_drv->msc_state; /* last msc_state */
	struct gbms_charging_event *ce_data = &batt_drv->ce_data;
	struct gbms_ce_tier_stats *tier = NULL;
	int cc;

	if (elap == 0)
		return;

	/* TODO: read at start of tier and update cc_total of previous */
	cc = GPSY_GET_PROP(batt_drv->fg_psy, POWER_SUPPLY_PROP_CHARGE_COUNTER);
	if (cc < 0) {
		pr_debug("MSC_STAT cannot read cc=%d\n", cc);
		return;
	}
	cc = cc / 1000;

	/* Note: To log new voltage tiers, add to list in go/pixel-vtier-defs */
	/* ---  Log tiers in PARALLEL below ---  */

	if (soc_real >= SSOC_HIGH_SOC)
		batt_chg_stats_update_tier(batt_drv, temp_idx, ibatt_ma, temp,
					   elap, cc,
					   &ce_data->high_soc_stats);

	/* --- Log tiers in SERIES below --- */
	if (batt_drv->batt_full) {

		/* Override regular charge tiers when fully charged */
		batt_chg_stats_update_tier(batt_drv, temp_idx, ibatt_ma,
					   temp, elap, cc,
					   &ce_data->full_charge_stats);

	} else if (msc_state == MSC_HEALTH_PAUSE) {

		/*
		 * We log the pause tier in different AC tier groups so that we
		 * can capture pause time separately.
		 */
		batt_chg_stats_update_tier(batt_drv, temp_idx, ibatt_ma, temp,
					   elap, cc,
					   &ce_data->health_pause_stats);

	} else if (msc_state == MSC_HEALTH) {
		/*
		 * It works because msc_logic call BEFORE updating msc_state.
		 * NOTE: that OVERHEAT and CCLVL disable AC, I should not be
		 * here if either of them are set.
		 * NOTE: We currently only log time when AC is ACTIVE.
		 * Thus, when disconnecting in ENABLED state, we will log a
		 * GBMS_STATS_AC_TI_ENABLED tier with no time, and the regular
		 * charge time is accumulated in normal charge tiers.
		 * Similarly, once we reach 100%, we stop counting time in the
		 * health tier and we rely on the full_charge_stats.
		 */

		/* tier used for TTF during HC, check msc_logic_health() */
		batt_chg_stats_update_tier(batt_drv, temp_idx, ibatt_ma,
					   temp, elap, cc,
					   &ce_data->health_stats);
	} else {
		const qnum_t soc = ssoc_get_capacity_raw(&batt_drv->ssoc_state);

		/* book to previous soc unless discharging */
		if (msc_state != MSC_DSG) {
			/* TODO: should I use ssoc instead? */
			batt_chg_stats_soc_update(ce_data, soc, elap,
						  tier_idx, cc);
		}

		/*
		 * ce_data.tier_stats[tier_idx] are used for time to full.
		 * Do not book to them if we are in overheat or LVL
		 */
		tier = &ce_data->tier_stats[tier_idx];
	}

	/* --- Log tiers in PARALLEL that MUST NULL normal tiers below --- */

	/* batt_drv->batt_health is protected with chg_lock, */
	if (batt_drv->batt_health == POWER_SUPPLY_HEALTH_OVERHEAT) {
		batt_chg_stats_update_tier(batt_drv, temp_idx, ibatt_ma, temp,
					   elap, cc,
					   &ce_data->overheat_stats);
		tier = NULL;
	}

	/* custom charge levels (DWELL-DEFEND or RETAIL) */
	if (batt_drv->chg_state.f.flags & GBMS_CS_FLAG_CCLVL) {
		batt_chg_stats_update_tier(batt_drv, temp_idx, ibatt_ma, temp,
					   elap, cc,
					   &ce_data->cc_lvl_stats);
		tier = NULL;
	}

	/*
	 * Time/current spent in OVERHEAT or at CustomLevel should not
	 * be booked to ce_data.tier_stats[tier_idx]
	 */
	if (!tier)
		return;

	batt_chg_stats_update_tier(batt_drv, temp_idx, ibatt_ma, temp,
				   elap, cc, tier);
}

static int batt_chg_health_vti(const struct batt_chg_health *chg_health)
{
	enum chg_health_state rest_state = chg_health->rest_state;
	ktime_t rest_deadline = chg_health->rest_deadline;
	int tier_idx = GBMS_STATS_AC_TI_INVALID;
	bool aon_enabled = chg_health->always_on_soc != -1;

	switch (rest_state) {
	/* battery defender did it */
	case CHG_HEALTH_BD_DISABLED:
	case CHG_HEALTH_CCLVL_DISABLED:
		tier_idx = GBMS_STATS_AC_TI_DEFENDER;
		break;
	/* user disabled with deadline */
	case CHG_HEALTH_USER_DISABLED:
		if (rest_deadline == CHG_DEADLINE_SETTING)
			tier_idx = GBMS_STATS_AC_TI_DISABLE_SETTING;
		else if (rest_deadline == CHG_DEADLINE_SETTING_STOP)
			tier_idx = GBMS_STATS_AC_TI_DISABLE_SETTING_STOP;
		else
			tier_idx = GBMS_STATS_AC_TI_DISABLE_MISC;
		break;
	/* missed the deadline, TODO: log the deadline */
	case CHG_HEALTH_DISABLED:
		tier_idx = GBMS_STATS_AC_TI_DISABLED;
		break;
	/* disconnected in active mode, TODO: log the deadline */
	case CHG_HEALTH_ACTIVE:
	case CHG_HEALTH_PAUSE:
		if (aon_enabled)
			tier_idx = GBMS_STATS_AC_TI_ACTIVE_AON;
		else
			tier_idx = GBMS_STATS_AC_TI_ACTIVE;
		break;
	/* never became active */
	case CHG_HEALTH_ENABLED:
		if (aon_enabled)
			tier_idx = GBMS_STATS_AC_TI_ENABLED_AON;
		else
			tier_idx = GBMS_STATS_AC_TI_ENABLED;
		break;
	/* active, worked */
	case CHG_HEALTH_DONE:
		tier_idx = GBMS_STATS_AC_TI_VALID;
		break;
	default:
		break;
	}

	return tier_idx;
}

static int batt_chg_vbat2tier(const int vbatt_idx)
{
	return vbatt_idx < GBMS_STATS_TIER_COUNT ?
		vbatt_idx : GBMS_STATS_TIER_COUNT - 1;
}

/* Only the qualified copy gets the timestamp and the exit voltage. */
static bool batt_chg_stats_close(struct batt_drv *batt_drv,
				 char *reason,
				 bool force)
{
	bool publish;
	const int vout = GPSY_GET_PROP(batt_drv->fg_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW);
	const int cc_out = GPSY_GET_PROP(batt_drv->fg_psy,
				POWER_SUPPLY_PROP_CHARGE_COUNTER);

	/* book last period to the current tier
	 * NOTE: vbatt_idx != -1 -> temp_idx != -1
	 */
	if (batt_drv->vbatt_idx != -1 && batt_drv->temp_idx != -1) {
		const ktime_t now = get_boot_sec();
		const ktime_t elap = now - batt_drv->ce_data.last_update;
		const int tier_idx = batt_chg_vbat2tier(batt_drv->vbatt_idx);
		const int ibatt = GPSY_GET_PROP(batt_drv->fg_psy,
						POWER_SUPPLY_PROP_CURRENT_NOW);
		const int temp = GPSY_GET_PROP(batt_drv->fg_psy,
					       POWER_SUPPLY_PROP_TEMP);

		batt_chg_stats_update(batt_drv,
				      batt_drv->temp_idx, tier_idx,
				      ibatt / 1000, temp, elap);
		batt_drv->ce_data.last_update = now;
	}

	/* record the closing in data (and qual) */
	batt_drv->ce_data.charging_stats.voltage_out =
				(vout < 0) ? -1 : vout / 1000;
	batt_drv->ce_data.charging_stats.ssoc_out =
				ssoc_get_capacity(&batt_drv->ssoc_state);
	batt_drv->ce_data.charging_stats.cc_out =
				(cc_out < 0) ? -1 : cc_out / 1000;

	/* close/fix heath charge data (if enabled) */
	memcpy(&batt_drv->ce_data.ce_health, &batt_drv->chg_health,
	       sizeof(batt_drv->ce_data.ce_health));
	batt_drv->ce_data.health_stats.vtier_idx =
				batt_chg_health_vti(&batt_drv->chg_health);

	/* TODO: add a field to ce_data to qual weird charge sessions */
	publish = force || batt_chg_stats_qual(batt_drv);
	if (publish) {
		struct gbms_charging_event *ce_qual = &batt_drv->ce_qual;

		/* all charge tiers including health */
		memcpy(ce_qual, &batt_drv->ce_data, sizeof(*ce_qual));

		pr_info("MSC_STAT %s: elap=%lld ssoc=%d->%d v=%d->%d c=%d->%d hdl=%lld hrs=%d hti=%d/%d\n",
			reason,
			ce_qual->last_update - ce_qual->first_update,
			ce_qual->charging_stats.ssoc_in,
			ce_qual->charging_stats.ssoc_out,
			ce_qual->charging_stats.voltage_in,
			ce_qual->charging_stats.voltage_out,
			ce_qual->charging_stats.cc_in,
			ce_qual->charging_stats.cc_out,
			ce_qual->ce_health.rest_deadline,
			ce_qual->ce_health.rest_state,
			ce_qual->health_stats.vtier_idx,
			ce_qual->health_pause_stats.vtier_idx);
	}

	return publish;
}

static int batt_chg_stats_soc_next(const struct gbms_charging_event *ce_data,
				   int i)
{
	int soc_next;

	if (i == GBMS_STATS_TIER_COUNT -1)
		return ce_data->last_soc;

	soc_next = ce_data->tier_stats[i + 1].soc_in >> 8;
	if (soc_next <= 0)
		return ce_data->last_soc;

	return soc_next;
}

static void bat_log_cstr_handler(struct logbuffer *log, char *buf, int len)
{
	int i, j = 0;
	char tmp[LOG_BUFFER_ENTRY_SIZE];

	buf[len] = '\n';
	for (i = 0; i <= len; i++) {
		if (buf[i] == '\n') {
			tmp[j] = '\0';
			/* skip first blank line */
			if (i != 0)
				logbuffer_log(log, "%s", tmp);
			j = 0;
		} else if (j >= LOG_BUFFER_ENTRY_SIZE - 1) {
			tmp[j] = '\0';
			logbuffer_log(log, "%s", tmp);
			i--;
			j = 0;
		} else {
			tmp[j] = buf[i];
			j++;
		}
	}
}

static void bat_log_chg_stats(struct logbuffer *log,
			      const struct gbms_charging_event *ce_data)
{
	const char *adapter_name =
		gbms_chg_ev_adapter_s(ce_data->adapter_details.ad_type);
	int i;

	logbuffer_log(log, "A: %s,%d,%d,%d",
			adapter_name,
			ce_data->adapter_details.ad_type,
			ce_data->adapter_details.ad_voltage * 100,
			ce_data->adapter_details.ad_amperage * 100);

	logbuffer_log(log, "S: %hu,%hu, %hu,%hu %hu,%hu %ld,%ld",
			ce_data->charging_stats.ssoc_in,
			ce_data->charging_stats.voltage_in,
			ce_data->charging_stats.ssoc_out,
			ce_data->charging_stats.voltage_out,
			ce_data->charging_stats.cc_in,
			ce_data->charging_stats.cc_out,
			ce_data->first_update,
			ce_data->last_update);

	for (i = 0; i < GBMS_STATS_TIER_COUNT; i++) {
		const int soc_next = batt_chg_stats_soc_next(ce_data, i);
		const int soc_in = ce_data->tier_stats[i].soc_in >> 8;
		const long elap = ce_data->tier_stats[i].time_fast +
				  ce_data->tier_stats[i].time_taper +
				  ce_data->tier_stats[i].time_other;
		/* retrun len in below functions sometimes more than 256 */
		char buff[LOG_BUFFER_ENTRY_SIZE * 2] = {0};
		int len = 0;

		/* Do not output tiers without time */
		if (!elap)
			continue;

		len = batt_chg_tier_stats_cstr(buff, sizeof(buff),
						&ce_data->tier_stats[i], true);
		bat_log_cstr_handler(log, buff, len);

		if (soc_next) {
			len = ttf_soc_cstr(buff, sizeof(buff),
					   &ce_data->soc_stats,
					   soc_in, soc_next);
			bat_log_cstr_handler(log, buff, len);
		}
	}
}

/* End of charging: close stats, qualify event publish data */
static void batt_chg_stats_pub(struct batt_drv *batt_drv, char *reason,
			       bool force, bool skip_uevent)
{
	bool publish;

	mutex_lock(&batt_drv->stats_lock);
	publish = batt_chg_stats_close(batt_drv, reason, force);
	if (publish) {
		ttf_stats_update(&batt_drv->ttf_stats,
				 &batt_drv->ce_qual, false);

		if (skip_uevent == false)
			kobject_uevent(&batt_drv->device->kobj, KOBJ_CHANGE);
	}

	bat_log_chg_stats(batt_drv->ttf_stats.ttf_log, &batt_drv->ce_data);
	mutex_unlock(&batt_drv->stats_lock);
}

/* Log only when elap != 0 add a special meaning for health status */
static int batt_chg_tier_stats_cstr(char *buff, int size,
				    const struct gbms_ce_tier_stats *tier_stat,
				    bool verbose)
{
	const int soc_in = tier_stat->soc_in >> 8;
	const long elap = tier_stat->time_fast + tier_stat->time_taper +
			  tier_stat->time_other;
	const static char *codes[] = {"n", "s", "d", "l", "v", "vo", "p", "f",
					"t", "dl", "st", "tc", "r", "w", "rs",
					"n", "ny", "h", "hp"};
	long temp_avg, ibatt_avg, icl_avg;
	int j, len = 0;

	if (elap) {
		temp_avg = tier_stat->temp_sum / elap;
		ibatt_avg = tier_stat->ibatt_sum / elap;
		icl_avg = tier_stat->icl_sum / elap;
	} else {
		temp_avg = 0;
		ibatt_avg = 0;
		icl_avg = 0;
	}

	len += scnprintf(&buff[len], size - len, "\n%d%c ",
		tier_stat->vtier_idx,
		(verbose) ? ':' : ',');

	len += scnprintf(&buff[len], size - len,
		"%d.%d,%d,%d, %d,%d,%d, %d,%ld,%d, %d,%ld,%d, %d,%ld,%d",
		soc_in,
		tier_stat->soc_in & 0xff,
		tier_stat->cc_in,
		tier_stat->temp_in,
		tier_stat->time_fast,
		tier_stat->time_taper,
		tier_stat->time_other,
		tier_stat->temp_min,
		temp_avg,
		tier_stat->temp_max,
		tier_stat->ibatt_min,
		ibatt_avg,
		tier_stat->ibatt_max,
		tier_stat->icl_min,
		icl_avg,
		tier_stat->icl_max);

	if (!verbose || !elap)
		return len;

	/* time spent in every multi step charging state */
	len += scnprintf(&buff[len], size - len, "\n%d:",
			tier_stat->vtier_idx);

	for (j = 0; j < MSC_STATES_COUNT; j++)
		len += scnprintf(&buff[len], size - len, " %s=%d",
			codes[j], tier_stat->msc_elap[j]);

	/* count spent in each step charging state */
	len += scnprintf(&buff[len], size - len, "\n%d:",
			tier_stat->vtier_idx);

	for (j = 0; j < MSC_STATES_COUNT; j++)
		len += scnprintf(&buff[len], size - len, " %s=%d",
			codes[j], tier_stat->msc_cnt[j]);

	return len;
}

/* health_stats->tier_index is set on stats_close() */
static int batt_health_stats_cstr(char *buff, int size,
				  const struct gbms_charging_event *ce_data,
				  bool verbose)
{
	const struct gbms_ce_tier_stats *health_stats = &ce_data->health_stats;
	const int vti = batt_chg_health_vti(&ce_data->ce_health);
	int len = 0;

	len += scnprintf(&buff[len], size - len, "\nH: %d %d %lld %d\n",
			 ce_data->ce_health.rest_state, vti,
			 ce_data->ce_health.rest_deadline,
			 ce_data->ce_health.always_on_soc);

	/* no additional tier stats when vti is invalid */
	if (vti == GBMS_STATS_AC_TI_INVALID)
		return len;

	len += batt_chg_tier_stats_cstr(&buff[len], size - len,
					health_stats,
					verbose);

	/* Only add pause tier logging if there is pause time */
	if (ce_data->health_pause_stats.soc_in != -1)
		len += batt_chg_tier_stats_cstr(&buff[len], size - len,
						&ce_data->health_pause_stats,
						verbose);
	return len;
}

/* doesn't output hc stats */
static int batt_chg_stats_cstr(char *buff, int size,
			       const struct gbms_charging_event *ce_data,
			       bool verbose)
{
	int i, len = 0;

	if (verbose) {
		const char *adapter_name =
			gbms_chg_ev_adapter_s(ce_data->adapter_details.ad_type);

		len += scnprintf(&buff[len], size - len, "A: %s,",
				adapter_name);
	}

	len += scnprintf(&buff[len], size - len, "%d,%d,%d",
				ce_data->adapter_details.ad_type,
				ce_data->adapter_details.ad_voltage * 100,
				ce_data->adapter_details.ad_amperage * 100);

	len += scnprintf(&buff[len], size - len, "%s%hu,%hu, %hu,%hu",
				(verbose) ?  "\nS: " : ", ",
				ce_data->charging_stats.ssoc_in,
				ce_data->charging_stats.voltage_in,
				ce_data->charging_stats.ssoc_out,
				ce_data->charging_stats.voltage_out);


	if (verbose) {
		len += scnprintf(&buff[len], size - len, " %hu,%hu",
				ce_data->charging_stats.cc_in,
				ce_data->charging_stats.cc_out);

		len += scnprintf(&buff[len], size - len, " %lld,%lld",
				ce_data->first_update,
				ce_data->last_update);
	}

	for (i = 0; i < GBMS_STATS_TIER_COUNT; i++) {
		const int soc_next = batt_chg_stats_soc_next(ce_data, i);
		const int soc_in = ce_data->tier_stats[i].soc_in >> 8;
		const long elap = ce_data->tier_stats[i].time_fast +
				  ce_data->tier_stats[i].time_taper +
				  ce_data->tier_stats[i].time_other;

		/* Do not output tiers without time */
		if (!elap)
			continue;

		len += batt_chg_tier_stats_cstr(&buff[len], size - len,
						&ce_data->tier_stats[i],
						verbose);

		if (soc_next)
			len += ttf_soc_cstr(&buff[len], size - len,
					    &ce_data->soc_stats,
					    soc_in, soc_next);
	}

	if (ce_data->full_charge_stats.soc_in != -1)
		len += batt_chg_tier_stats_cstr(&buff[len], size - len,
						&ce_data->full_charge_stats,
						verbose);

	if (ce_data->high_soc_stats.soc_in != -1)
		len += batt_chg_tier_stats_cstr(&buff[len], size - len,
						&ce_data->high_soc_stats,
						verbose);

	if (ce_data->overheat_stats.soc_in != -1)
		len += batt_chg_tier_stats_cstr(&buff[len], size - len,
						&ce_data->overheat_stats,
						verbose);

	if (ce_data->cc_lvl_stats.soc_in != -1)
		len += batt_chg_tier_stats_cstr(&buff[len], size - len,
						&ce_data->cc_lvl_stats,
						verbose);

	return len;
}

/* ------------------------------------------------------------------------- */

static void batt_res_dump_logs(struct batt_res *rstate)
{
	pr_info("RES: req:%d, sample:%d[%d], filt_cnt:%d, res_avg:%d\n",
		rstate->estimate_requested, rstate->sample_accumulator,
		rstate->sample_count, rstate->filter_count,
		rstate->resistance_avg);
}

static void batt_res_state_set(struct batt_res *rstate, bool breq)
{
	rstate->estimate_requested = breq;
	rstate->sample_accumulator = 0;
	rstate->sample_count = 0;
	batt_res_dump_logs(rstate);
}

static void batt_res_store_data(struct batt_res *rstate,
				struct power_supply *fg_psy)
{
	int ret = 0;
	int filter_estimate = 0;
	int total_estimate = 0;
	long new_estimate = 0;

	new_estimate = rstate->sample_accumulator / rstate->sample_count;
	filter_estimate = rstate->resistance_avg * rstate->filter_count;

	rstate->filter_count++;
	if (rstate->filter_count > rstate->estimate_filter) {
		rstate->filter_count = rstate->estimate_filter;
		filter_estimate -= rstate->resistance_avg;
	}
	total_estimate = filter_estimate + new_estimate;
	rstate->resistance_avg = total_estimate / rstate->filter_count;

	ret = gbms_storage_write(GBMS_TAG_RAVG, &rstate->resistance_avg,
				 sizeof(rstate->resistance_avg));
	if (ret < 0)
		pr_err("failed to write resistance_avg\n");

	ret = gbms_storage_write(GBMS_TAG_RFCN, &rstate->filter_count,
				 sizeof(rstate->filter_count));
	if (ret < 0)
		pr_err("failed to write resistenace filt_count\n");

	batt_res_dump_logs(rstate);
}

static int batt_res_load_data(struct batt_res *rstate,
			      struct power_supply *fg_psy)
{
	int ret = 0;

	ret = gbms_storage_read(GBMS_TAG_RAVG, &rstate->resistance_avg,
				sizeof(rstate->resistance_avg));
	if (ret < 0) {
		pr_err("failed to get resistance_avg(%d)\n", ret);
		return ret;
	}

	ret = gbms_storage_read(GBMS_TAG_RFCN, &rstate->filter_count,
				sizeof(rstate->filter_count));
	if (ret < 0) {
		rstate->resistance_avg = 0;
		pr_err("failed to get resistance filt_count(%d)\n", ret);
		return ret;
	}

	batt_res_dump_logs(rstate);
	return 0;
}

static void batt_res_work(struct batt_drv *batt_drv)
{
	u32 data32;
	int temp, ret, resistance;
	struct batt_res *rstate = &batt_drv->res_state;
	const int ssoc_threshold = rstate->ssoc_threshold;
	const int res_temp_low = rstate->res_temp_low;
	const int res_temp_high = rstate->res_temp_high;

	temp = GPSY_GET_INT_PROP(batt_drv->fg_psy,
				 POWER_SUPPLY_PROP_TEMP, &ret);
	if (ret < 0 || temp < res_temp_low || temp > res_temp_high) {
		if (ssoc_get_real(&batt_drv->ssoc_state) > ssoc_threshold) {
			if (rstate->sample_count > 0) {
				/* update the filter */
				batt_res_store_data(&batt_drv->res_state,
						    batt_drv->fg_psy);
				batt_res_state_set(rstate, false);
			}
		}
		return;
	}

	ret = gbms_storage_read(GBMS_TAG_BRES, &data32, sizeof(data32));
	if (ret < 0)
		return;
	resistance = data32;

	if (ssoc_get_real(&batt_drv->ssoc_state) < ssoc_threshold) {
		rstate->sample_accumulator += resistance / 100;
		rstate->sample_count++;
		batt_res_dump_logs(rstate);
	} else {
		if (rstate->sample_count > 0) {
			/* update the filter here */
			batt_res_store_data(&batt_drv->res_state,
					    batt_drv->fg_psy);
		}
		batt_res_state_set(rstate, false);
	}
}

/* ------------------------------------------------------------------------- */

/* NOTE: should not reset always_on_soc */
static inline void batt_reset_rest_state(struct batt_chg_health *chg_health)
{
	chg_health->rest_cc_max = -1;
	chg_health->rest_fv_uv = -1;

	/* keep negative deadlines (they mean USER disabled) */
	if (chg_health->rest_deadline < 0) {
		chg_health->rest_state = CHG_HEALTH_USER_DISABLED;
	} else {
		chg_health->rest_state = CHG_HEALTH_INACTIVE;
		chg_health->rest_deadline = 0;
	}
}

/* should not reset rl state */
static inline void batt_reset_chg_drv_state(struct batt_drv *batt_drv)
{
	/* the wake assertion will be released on disconnect and on SW JEITA */
	if (batt_drv->hold_taper_ws) {
		batt_drv->hold_taper_ws = false;
		__pm_relax(batt_drv->taper_ws);
	}

	/* polling */
	batt_drv->batt_fast_update_cnt = 0;
	batt_drv->ttf_debounce = 1;
	batt_drv->fg_status = POWER_SUPPLY_STATUS_UNKNOWN;
	batt_drv->chg_done = false;
	/* algo */
	batt_drv->temp_idx = -1;
	batt_drv->vbatt_idx = -1;
	batt_drv->fv_uv = -1;
	batt_drv->cc_max = -1;
	batt_drv->msc_update_interval = -1;
	batt_drv->jeita_stop_charging = -1;
	/* timers */
	batt_drv->checked_cv_cnt = 0;
	batt_drv->checked_ov_cnt = 0;
	batt_drv->checked_tier_switch_cnt = 0;
	/* stats and logs */
	batt_drv->msc_state = -1;
	batt_drv->last_log_cnt = 0;
	/* health */
	batt_reset_rest_state(&batt_drv->chg_health);
	/* fan level */
	fan_level_reset(batt_drv);
}

/*
 * software JEITA, disable charging when outside the charge table.
 * NOTE: ->jeita_stop_charging is either -1 (init or disable) or 0
 * TODO: need to be able to disable (leave to HW)
 */
static bool msc_logic_soft_jeita(const struct batt_drv *batt_drv, int temp)
{
	const struct gbms_chg_profile *profile = &batt_drv->chg_profile;

	if (temp < profile->temp_limits[0] ||
	    temp > profile->temp_limits[profile->temp_nb_limits - 1]) {
		if (batt_drv->jeita_stop_charging < 0) {
			batt_prlog(BATT_PRLOG_ALWAYS,
				   "MSC_JEITA temp=%d off limits, do not enable charging\n",
				   temp);
		} else if (batt_drv->jeita_stop_charging == 0) {
			batt_prlog(BATT_PRLOG_ALWAYS,
				   "MSC_JEITA temp=%d off limits, disabling charging\n",
				    temp);
		}

		return true;
	}

	return false;
}

/* TODO: only change batt_drv->checked_ov_cnt, an */
static int msc_logic_irdrop(struct batt_drv *batt_drv,
			    int vbatt, int ibatt, int temp_idx,
			    int *vbatt_idx, int *fv_uv,
			    int *update_interval)
{
	const struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	const int vtier = profile->volt_limits[*vbatt_idx];
	const int chg_type = batt_drv->chg_state.f.chg_type;
	const int utv_margin = profile->cv_range_accuracy;
	const int otv_margin = profile->cv_otv_margin;
	const int switch_cnt = profile->cv_tier_switch_cnt;
	int vchg = batt_drv->chg_state.f.vchrg;
	int msc_state = MSC_NONE;
	bool match_enable;

	if (batt_drv->chg_state.f.flags & GBMS_CS_FLAG_NOCOMP)
		vchg = 0;
	match_enable = vchg != 0;

	if ((vbatt - vtier) > otv_margin) {
		/* OVER: vbatt over vtier for more than margin */
		const int cc_max = GBMS_CCCM_LIMITS(profile, temp_idx,
						    *vbatt_idx);

		/*
		 * pullback when over tier voltage, fast poll, penalty
		 * on TAPER_RAISE and no cv debounce (so will consider
		 * switching voltage tiers if the current is right).
		 * NOTE: lowering voltage might cause a small drop in
		 * current (we should remain  under next tier)
		 */
		*fv_uv = gbms_msc_round_fv_uv(profile, vtier,
			*fv_uv - profile->fv_uv_resolution);
		if (*fv_uv < vtier)
			*fv_uv = vtier;

		*update_interval = profile->cv_update_interval;
		batt_drv->checked_ov_cnt = profile->cv_tier_ov_cnt;
		batt_drv->checked_cv_cnt = 0;

		if (batt_drv->checked_tier_switch_cnt > 0 || !match_enable) {
			/* no pullback, next tier if already counting */
			msc_state = MSC_VSWITCH;
			*vbatt_idx = batt_drv->vbatt_idx + 1;

			batt_prlog(BATT_PRLOG_ALWAYS,
				   "MSC_VSWITCH vt=%d vb=%d ibatt=%d me=%d\n",
				   vtier, vbatt, ibatt, match_enable);
		} else if (-ibatt == cc_max) {
			/* pullback, double penalty if at full current */
			msc_state = MSC_VOVER;
			batt_drv->checked_ov_cnt *= 2;

			batt_prlog(BATT_PRLOG_ALWAYS,
				   "MSC_VOVER vt=%d  vb=%d ibatt=%d fv_uv=%d->%d\n",
				   vtier, vbatt, ibatt,
				   batt_drv->fv_uv, *fv_uv);
		} else {
			/* simple pullback */
			msc_state = MSC_PULLBACK;
			batt_prlog(BATT_PRLOG_ALWAYS,
				  "MSC_PULLBACK vt=%d vb=%d ibatt=%d fv_uv=%d->%d\n",
				  vtier, vbatt, ibatt,
				  batt_drv->fv_uv, *fv_uv);
		}

		/*
		 * might get here after windup because algo will track the
		 * voltage drop caused from load as IRDROP.
		 * TODO: make sure that being current limited clear
		 * the taper condition.
		 */

	} else if (chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST) {
		/*
		 * FAST: usual compensation (vchrg is vqcom)
		 * NOTE: there is a race in reading from charger and
		 * data might not be consistent (b/110318684)
		 * NOTE: could add PID loop for management of thermals
		 */
		const int vchrg_ua = vchg * 1000;

		msc_state = MSC_FAST;

		/* invalid or 0 vchg disable IDROP compensation */
		if (vchrg_ua <= 0) {
			/* could keep it steady instead */
			*fv_uv = vtier;
		} else if (vchrg_ua > vbatt) {
			*fv_uv = gbms_msc_round_fv_uv(profile, vtier,
				vtier + (vchrg_ua - vbatt));
		}

		/* no tier switch in fast charge (TODO unless close to tier) */
		if (batt_drv->checked_cv_cnt == 0)
			batt_drv->checked_cv_cnt = 1;

		batt_prlog(BATT_PRLOG_ALWAYS,
			   "MSC_FAST vt=%d vb=%d ib=%d fv_uv=%d->%d vchrg=%d cv_cnt=%d\n",
			   vtier, vbatt, ibatt, batt_drv->fv_uv, *fv_uv,
			   batt_drv->chg_state.f.vchrg,
			   batt_drv->checked_cv_cnt);

	} else if (chg_type == POWER_SUPPLY_CHARGE_TYPE_TRICKLE) {
		/*
		 * Precharge: charging current/voltage are limited in
		 * hardware, no point in applying irdrop compensation.
		 * Just wait for battery voltage to raise over the
		 * precharge to fast charge threshold.
		 */
		msc_state = MSC_TYPE;

		/* no tier switching in trickle */
		if (batt_drv->checked_cv_cnt == 0)
			batt_drv->checked_cv_cnt = 1;

		batt_prlog(BATT_PRLOG_ALWAYS, "MSC_PRE vt=%d vb=%d fv_uv=%d chg_type=%d\n",
			   vtier, vbatt, *fv_uv, chg_type);
	} else if (chg_type != POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT) {
		const int type_margin = utv_margin;

		/*
		 * Not fast, taper or precharge: in *_UNKNOWN and *_NONE.
		 * Set checked_cv_cnt=0 when voltage is withing utv_margin of
		 * vtier (tune marging) to force checking current and avoid
		 * early termination for lack of headroom. Carry on at the
		 * same update_interval otherwise.
		 */
		msc_state = MSC_TYPE;
		if (vbatt > (vtier - type_margin)) {
			*update_interval = profile->cv_update_interval;
			batt_drv->checked_cv_cnt = 0;
		} else {
			batt_drv->checked_cv_cnt = 1;
		}

		batt_prlog(BATT_PRLOG_ALWAYS,
			   "MSC_TYPE vt=%d margin=%d cv_cnt=%d vb=%d fv_uv=%d chg_type=%d\n",
			   vtier, type_margin, batt_drv->checked_cv_cnt, vbatt,
			   *fv_uv, chg_type);

	} else if (batt_drv->checked_ov_cnt) {
		/*
		 * TAPER_DLY: countdown to raise fv_uv and/or check
		 * for tier switch, will keep steady...
		 */
		batt_prlog(BATT_PRLOG_ALWAYS,
			   "MSC_DLY vt=%d vb=%d fv_uv=%d margin=%d cv_cnt=%d, ov_cnt=%d\n",
			   vtier, vbatt, *fv_uv, profile->cv_range_accuracy,
			   batt_drv->checked_cv_cnt,
			   batt_drv->checked_ov_cnt);

		msc_state = MSC_DLY;
		batt_drv->checked_ov_cnt -= 1;
		*update_interval = profile->cv_update_interval;

	} else if ((vtier - vbatt) < utv_margin) {
		const bool log_level = batt_drv->msc_state != MSC_STEADY &&
				      batt_drv->msc_state != MSC_RSTC;

		/* TAPER_STEADY: close enough to tier */

		msc_state = MSC_STEADY;
		*update_interval = profile->cv_update_interval;

		batt_prlog(batt_prlog_level(log_level),
			   "MSC_STEADY vt=%d vb=%d fv_uv=%d margin=%d\n",
			   vtier, vbatt, *fv_uv,
			   profile->cv_range_accuracy);
	} else if (batt_drv->checked_tier_switch_cnt >= (switch_cnt - 1)) {
		/*
		 * TAPER_TIERCNTING: prepare to switch to next tier
		 * so not allow to raise vfloat to prevent battery
		 * voltage over than tier
		 */
		msc_state = MSC_TIERCNTING;
		*update_interval = profile->cv_update_interval;

		batt_prlog(BATT_PRLOG_ALWAYS,
			   "MSC_TIERCNTING vt=%d vb=%d fv_uv=%d margin=%d\n",
			   vtier, vbatt, *fv_uv,
			   profile->cv_range_accuracy);
	} else if (match_enable) {
		/*
		 * TAPER_RAISE: under tier vlim, raise one click &
		 * debounce taper (see above handling of STEADY)
		 */
		msc_state = MSC_RAISE;
		*fv_uv = gbms_msc_round_fv_uv(profile, vtier,
			*fv_uv + profile->fv_uv_resolution);
		*update_interval = profile->cv_update_interval;

		/* debounce next taper voltage adjustment */
		batt_drv->checked_cv_cnt = profile->cv_debounce_cnt;

		batt_prlog(BATT_PRLOG_ALWAYS, "MSC_RAISE vt=%d vb=%d fv_uv=%d->%d\n",
			   vtier, vbatt, batt_drv->fv_uv, *fv_uv);
	} else {
		msc_state = MSC_STEADY;
		batt_prlog(BATT_PRLOG_DEBUG, "MSC_DISB vt=%d vb=%d fv_uv=%d->%d\n",
			vtier, vbatt, batt_drv->fv_uv, *fv_uv);
	}

	return msc_state;
}

/* battery health based charging on SOC */
static enum chg_health_state msc_health_active(const struct batt_drv *batt_drv)
{
	int ssoc, ssoc_threshold = -1;

	ssoc_threshold = CHG_HEALTH_REST_SOC(&batt_drv->chg_health);
	if (ssoc_threshold < 0)
		return CHG_HEALTH_INACTIVE;

	ssoc = ssoc_get_capacity(&batt_drv->ssoc_state);
	if (ssoc >= ssoc_threshold)
		return CHG_HEALTH_ACTIVE;

	return CHG_HEALTH_ENABLED;
}

#define HEALTH_PAUSE_DEBOUNCE 180
#define HEALTH_PAUSE_MAX_SSOC 95
static bool msc_health_pause(struct batt_drv *batt_drv, const ktime_t ttf,
			      const ktime_t now,
			      const enum chg_health_state rest_state) {
	const struct gbms_charging_event *ce_data = &batt_drv->ce_data;
	const struct gbms_ce_tier_stats	*h = &ce_data->health_stats;
	const struct batt_chg_health *rest = &batt_drv->chg_health;
	const ktime_t deadline = rest->rest_deadline;
	const ktime_t safety_margin = (ktime_t)batt_drv->health_safety_margin;
	/* Note: We only capture ACTIVE time in health stats */
	const ktime_t elap_h = h->time_fast + h->time_taper + h->time_other;
	const int ssoc = ssoc_get_capacity(&batt_drv->ssoc_state);

	/*
	 * the safety marging cannot be less than 0 (it would subtract time from TTF and would
         * cause AC to never meet 100% in time). Use 0<= to disable PAUSE.
	 */
	if (safety_margin <= 0)
		return false;

	/*
	 * Expected behavior:
	 * 1. ACTIVE: small current run a while for ttf
	 * 2. PAUSE: when time is enough to pause
	 * 3. ACTIVE: when time out and back to ACTIVE charge
	 */
	if (rest_state != CHG_HEALTH_ACTIVE && rest_state != CHG_HEALTH_PAUSE)
		return false;

	/*
	 * elap_h: running active for a while wait status and current stable
	 * ssoc: transfer in high soc impact charge full condition, disable pause
	 * behavior in high soc
	 */
	if (elap_h < HEALTH_PAUSE_DEBOUNCE || ssoc > HEALTH_PAUSE_MAX_SSOC)
		return false;

	/* check if time meets the PAUSE condition or not */
	if (ttf > 0 && deadline > now + ttf + safety_margin)
		return true;

	return false;
}


/*
 * for logging, userspace should use
 *   deadline == 0 on fast replug (leave initial deadline ok)
 *   deadline == -1 when the feature is disabled
 *       if charge health was active/enabled, set to -2
 *   deadline == absolute requested deadline (if always_on is set)
 * return true if there was a change
 */
static bool batt_health_set_chg_deadline(struct batt_chg_health *chg_health,
					 long long deadline_s)
{
	enum chg_health_state rest_state = chg_health->rest_state;
	bool new_deadline;

	/* disabled in settings */
	if (deadline_s < 0) {
		new_deadline = chg_health->rest_deadline != deadline_s;
		chg_health->rest_state = CHG_HEALTH_USER_DISABLED;

		if (chg_health->rest_deadline > 0) /* was active */
			chg_health->rest_deadline = CHG_DEADLINE_SETTING_STOP;
		else
			chg_health->rest_deadline = CHG_DEADLINE_SETTING;

	/* disabled with replug */
	} else if (deadline_s == 0) {
		new_deadline = chg_health->rest_deadline != deadline_s;
		/* ->rest_deadline will be reset to 0 on disconnect */

		/* Don't disable A/C if already done */
		if (chg_health->rest_state != CHG_HEALTH_DONE)
			chg_health->rest_state = CHG_HEALTH_USER_DISABLED;

		/* enabled from any previous state */
	} else {
		const ktime_t rest_deadline = get_boot_sec() + deadline_s;

		/* ->always_on SOC overrides the deadline */
		new_deadline = chg_health->rest_deadline != rest_deadline;
		chg_health->rest_state = CHG_HEALTH_ENABLED;
		chg_health->rest_deadline = rest_deadline;
	}

	return new_deadline || rest_state != chg_health->rest_state;
}

/* cc_max in ua: capacity in mAh, rest_rate in deciPct */
static int msc_logic_health_get_rate(const struct batt_chg_health *rest,
				     int capacity_ma)
{
	return capacity_ma * rest->rest_rate * 10;
}

/* health based charging trade charging speed for battery cycle life. */
static bool msc_logic_health(struct batt_drv *batt_drv)
{
	const struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	struct batt_chg_health *rest = &batt_drv->chg_health;
	const ktime_t deadline = rest->rest_deadline;
	enum chg_health_state rest_state = rest->rest_state;
	const bool aon_enabled = rest->always_on_soc != -1;
	const ktime_t now = get_boot_sec();
	int fv_uv = -1, cc_max = -1;
	bool changed = false;
	ktime_t ttf = 0;
	int ret;

	/* move to ENABLED if INACTIVE when aon_enabled is set */
	if (aon_enabled && rest_state == CHG_HEALTH_INACTIVE)
		rest_state = CHG_HEALTH_ENABLED;

	/*
	 * on disconnect batt_reset_rest_state() will set rest_state to
	 * CHG_HEALTH_USER_DISABLED if the deadline is negative.
	 */
	if (rest_state == CHG_HEALTH_CCLVL_DISABLED ||
	    rest_state == CHG_HEALTH_BD_DISABLED ||
	    rest_state == CHG_HEALTH_USER_DISABLED ||
	    rest_state == CHG_HEALTH_DISABLED ||
	    rest_state == CHG_HEALTH_INACTIVE)
		goto done_no_op;

	/* Keeps AC enabled after DONE */
	if (rest_state == CHG_HEALTH_DONE)
		goto done_exit;

	/* disable AC because we are running custom charging levels */
	if (batt_drv->chg_state.f.flags & GBMS_CS_FLAG_CCLVL) {
		rest_state = CHG_HEALTH_CCLVL_DISABLED;
		goto done_exit;
	}

	/* disable AC because BD-TEMP triggered */
	if (batt_drv->batt_health == POWER_SUPPLY_HEALTH_OVERHEAT) {
		rest_state = CHG_HEALTH_BD_DISABLED;
		goto done_exit;
	}

	/*
	 * ret < 0 right after plug-in or when the device is discharging due
	 * to a large sysload or an underpowered adapter (or both). Current
	 * strategy leaves everything as is (hoping) that the load is temporary.
	 * The estimate will be negative when BD is triggered and during the
	 * debounce period.
	 */
	ret = batt_ttf_estimate(&ttf, batt_drv);
	if (ret < 0)
		return false;

	/* estimate is 0 at 100%: set to done and keep AC enabled in RL */
	if (ttf == 0) {
		rest_state = CHG_HEALTH_DONE;
		goto done_exit;
	}

	/*
	 * rest_state here is either ENABLED or ACTIVE, transition to DISABLED
	 * when the deadline cannot be met with the current rate. set a new
	 * deadline or reset always_on_soc to re-enable AC for this session.
	 * NOTE: A device with AON enabled might (will) receive a deadline if
	 * plugged in within the AC window: ignore it.
	 * NOTE: cannot have a negative deadline with rest_state different
	 * from CHG_HEALTH_USER_DISABLED.
	 * TODO: consider adding a margin or debounce it.
	 */
	if (aon_enabled == false && rest_state == CHG_HEALTH_ACTIVE &&
	    deadline > 0 && ttf != -1 && now + ttf > deadline) {
		rest_state = CHG_HEALTH_DISABLED;
		goto done_exit;
	}

	/* Decide enter PAUSE state or not by time */
	if (msc_health_pause(batt_drv, ttf, now, rest_state)) {
		rest_state = CHG_HEALTH_PAUSE;
		goto done_exit;
	}

	/*
	 * rest_state here is either ENABLED or ACTIVE,
	 * NOTE: State might transition from _ACTIVE to _ENABLED after a
	 * discharge cycle that makes the battery fall under the threshold.
	 * State will transition back to _ENABLED after some time unless
	 * the deadline is met.
	 */
	rest_state = msc_health_active(batt_drv);

done_exit:
	if (rest_state == CHG_HEALTH_ACTIVE || rest_state == CHG_HEALTH_DONE) {
		const int capacity_ma = batt_drv->battery_capacity;

		cc_max = msc_logic_health_get_rate(rest, capacity_ma);

		/*
		 * default FV_UV to the last charge tier since fv_uv will be
		 * set to that on _DONE.
		 * NOTE this might need to be adjusted for the actual charge
		 * tiers that have nonzero charging current
		 */
		fv_uv = profile->volt_limits[profile->volt_nb_limits - 1];

		/* TODO: make sure that we wakeup when we are close to ttf */
	} else if (rest_state == CHG_HEALTH_PAUSE) {
		/*
		 * pause charging behavior when the the deadline is longer than
		 * expected charge time. return back to CHG_HEALTH_ACTIVE and
		 * start health charge when now + ttf + margine close to deadline
		*/
		cc_max = 0;
	}

done_no_op:
	/* send a power supply event when rest_state changes */
	changed = rest->rest_state != rest_state;

	/* msc_logic_* will vote on cc_max and fv_uv. */
	rest->rest_cc_max = cc_max;
	rest->rest_fv_uv = fv_uv;

	if (!changed)
		return false;

	batt_prlog(BATT_PRLOG_ALWAYS,
		   "MSC_HEALTH: now=%lld deadline=%lld aon_soc=%d ttf=%lld state=%d->%d fv_uv=%d, cc_max=%d\n",
		   now, rest->rest_deadline, rest->always_on_soc, ttf,
		   rest->rest_state, rest_state, fv_uv, cc_max);
	logbuffer_log(batt_drv->ttf_stats.ttf_log,
		      "MSC_HEALTH: now=%lld deadline=%lld aon_soc=%d ttf=%lld state=%d->%d fv_uv=%d, cc_max=%d\n",
		      now, rest->rest_deadline, rest->always_on_soc,
		      ttf, rest->rest_state, rest_state, fv_uv, cc_max);

	rest->rest_state = rest_state;
	memcpy(&batt_drv->ce_data.ce_health, &batt_drv->chg_health,
			sizeof(batt_drv->ce_data.ce_health));
	return true;
}

static int msc_pm_hold(int msc_state)
{
	int pm_state = -1;

	switch (msc_state) {
	case MSC_RAISE:
	case MSC_VOVER:
	case MSC_PULLBACK:
		pm_state = 1; /* __pm_stay_awake */
		break;
	case MSC_SEED:
	case MSC_DSG:
	case MSC_VSWITCH:
	case MSC_NEXT:
	case MSC_LAST:
	case MSC_RSTC:
	case MSC_HEALTH:
	case MSC_WAIT:
	case MSC_FAST:
	case MSC_NYET:
	case MSC_STEADY:
		pm_state = 0;  /* pm_relax */
		break;
	default:
		pr_debug("hold not defined for msc_state=%d\n", msc_state);
		pm_state = 0;  /* pm_relax */
		break;
	}

	return pm_state;
}

/* TODO: factor msc_logic_irdop from the logic about tier switch */
static int msc_logic(struct batt_drv *batt_drv)
{
	bool sw_jeita;
	int msc_state = MSC_NONE;
	struct power_supply *fg_psy = batt_drv->fg_psy;
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	int vbatt_idx = batt_drv->vbatt_idx, fv_uv = batt_drv->fv_uv, temp_idx;
	int temp, ibatt, vbatt, ioerr;
	int update_interval = MSC_DEFAULT_UPDATE_INTERVAL;
	const ktime_t now = get_boot_sec();
	ktime_t elap = now - batt_drv->ce_data.last_update;
	bool changed;

	temp = GPSY_GET_INT_PROP(fg_psy, POWER_SUPPLY_PROP_TEMP, &ioerr);
	if (ioerr < 0)
		return -EIO;

	/*
	 * driver state is (was) reset when we hit the SW jeita limit.
	 * NOTE: resetting driver state will release the wake assertion
	 */
	sw_jeita = msc_logic_soft_jeita(batt_drv, temp);
	if (sw_jeita) {
		/* reset batt_drv->jeita_stop_charging to -1 */
		if (batt_drv->jeita_stop_charging == 0)
			batt_reset_chg_drv_state(batt_drv);

		return 0;
	} else if (batt_drv->jeita_stop_charging) {
		batt_prlog(BATT_PRLOG_ALWAYS,
			   "MSC_JEITA temp=%d ok, enabling charging\n",
			   temp);
		batt_drv->jeita_stop_charging = 0;
	}

	ibatt = GPSY_GET_INT_PROP(fg_psy, POWER_SUPPLY_PROP_CURRENT_NOW,
					  &ioerr);
	if (ioerr < 0)
		return -EIO;

	vbatt = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	if (vbatt < 0)
		return -EIO;

	/*
	 * Multi Step Charging with IRDROP compensation when vchrg is != 0
	 * vbatt_idx = batt_drv->vbatt_idx, fv_uv = batt_drv->fv_uv
	 */
	temp_idx = gbms_msc_temp_idx(profile, temp);
	if (temp_idx != batt_drv->temp_idx || batt_drv->fv_uv == -1 ||
		batt_drv->vbatt_idx == -1) {

		msc_state = MSC_SEED;

		/* seed voltage only on connect, book 0 time */
		if (batt_drv->vbatt_idx == -1)
			vbatt_idx = gbms_msc_voltage_idx(profile, vbatt);

		batt_prlog(BATT_PRLOG_ALWAYS,
			   "MSC_SEED temp=%d vb=%d temp_idx:%d->%d, vbatt_idx:%d->%d\n",
			   temp, vbatt, batt_drv->temp_idx, temp_idx,
			   batt_drv->vbatt_idx, vbatt_idx);

		/* Debounce tier switch only when not already switching */
		if (batt_drv->checked_tier_switch_cnt == 0)
			batt_drv->checked_cv_cnt = profile->cv_debounce_cnt;
	} else if (ibatt > 0) {
		const int vtier = profile->volt_limits[vbatt_idx];
		const bool log_level = batt_drv->msc_state != MSC_DSG ||
				       batt_drv->cc_max != 0;

		/*
		 * Track battery voltage if discharging is due to system load,
		 * low ILIM or lack of headroom; stop charging work and reset
		 * batt_drv state() when discharging is due to disconnect.
		 * NOTE: POWER_SUPPLY_PROP_STATUS return *_DISCHARGING only on
		 * disconnect.
		 * NOTE: same vbat_idx will not change fv_uv
		 */
		msc_state = MSC_DSG;
		vbatt_idx = gbms_msc_voltage_idx(profile, vbatt);

		batt_prlog(batt_prlog_level(log_level),
			   "MSC_DSG vbatt_idx:%d->%d vt=%d fv_uv=%d vb=%d ib=%d cv_cnt=%d ov_cnt=%d\n",
			   batt_drv->vbatt_idx, vbatt_idx, vtier, fv_uv, vbatt, ibatt,
			   batt_drv->checked_cv_cnt, batt_drv->checked_ov_cnt);

	} else if (batt_drv->vbatt_idx == profile->volt_nb_limits - 1) {
		const int chg_type = batt_drv->chg_state.f.chg_type;
		const int vtier = profile->volt_limits[vbatt_idx];
		int log_level;

		/*
		 * will not adjust charger voltage only in the configured
		 * last tier.
		 * NOTE: might not be the "real" last tier since can I have
		 * tiers with max charge current == 0.
		 * NOTE: should I use a voltage limit instead?
		 */

		if (chg_type == POWER_SUPPLY_CHARGE_TYPE_FAST) {
			msc_state = MSC_FAST;
		} else if (chg_type != POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT) {
			msc_state = MSC_TYPE;
		} else {
			msc_state = MSC_LAST;
		}

		log_level = batt_prlog_level(batt_drv->msc_state != msc_state);
		if (log_level != BATT_PRLOG_ALWAYS && msc_state == MSC_LAST) {

			if (batt_drv->last_log_cnt > 0)
				batt_drv->last_log_cnt--;
			if (batt_drv->last_log_cnt == 0) {
				batt_drv->last_log_cnt = BATT_PRLOG_LAST_LOG_COUNT;
				log_level = batt_prlog_level(true);
			}
		}

		batt_prlog(log_level, "MSC_LAST vt=%d fv_uv=%d vb=%d ib=%d\n",
			   vtier, fv_uv, vbatt, ibatt);

	} else {
		const int tier_idx = batt_drv->vbatt_idx;
		const int vtier = profile->volt_limits[vbatt_idx];
		const int switch_cnt = profile->cv_tier_switch_cnt;
		const int cc_next_max = GBMS_CCCM_LIMITS(profile, temp_idx,
							vbatt_idx + 1);

		/* book elapsed time to previous tier & msc_irdrop_state */
		msc_state = msc_logic_irdrop(batt_drv,
					     vbatt, ibatt, temp_idx,
					     &vbatt_idx, &fv_uv,
					     &update_interval);

		if (msc_pm_hold(msc_state) == 1 && !batt_drv->hold_taper_ws) {
			__pm_stay_awake(batt_drv->taper_ws);
			batt_drv->hold_taper_ws = true;
		}

		mutex_lock(&batt_drv->stats_lock);
		batt_chg_stats_tier(&batt_drv->ce_data.tier_stats[tier_idx],
				    batt_drv->msc_irdrop_state, elap);
		batt_drv->msc_irdrop_state = msc_state;
		mutex_unlock(&batt_drv->stats_lock);

		/*
		 * Basic multi step charging: switch to next tier when ibatt
		 * is under next tier cc_max.
		 */
		if (batt_drv->checked_cv_cnt > 0) {
			/* debounce period on tier switch */
			batt_drv->checked_cv_cnt -= 1;

			batt_prlog(batt_prlog_level(msc_state != MSC_FAST),
				   "MSC_WAIT s:%d->%d vt=%d fv_uv=%d vb=%d ib=%d cv_cnt=%d ov_cnt=%d t_cnt=%d\n",
				   msc_state, MSC_WAIT, vtier, fv_uv, vbatt, ibatt,
				   batt_drv->checked_cv_cnt, batt_drv->checked_ov_cnt,
				   batt_drv->checked_tier_switch_cnt);

			if (-ibatt > cc_next_max)
				batt_drv->checked_tier_switch_cnt = 0;

			msc_state = MSC_WAIT;
		} else if (-ibatt > cc_next_max) {

			/* current over next tier, reset tier switch count */
			batt_prlog(BATT_PRLOG_ALWAYS,
				   "MSC_RSTC s:%d->%d vt=%d fv_uv=%d vb=%d ib=%d cc_next_max=%d t_cnt=%d->0\n",
				   msc_state, MSC_RSTC, vtier, fv_uv, vbatt, ibatt, cc_next_max,
				   batt_drv->checked_tier_switch_cnt);

			batt_drv->checked_tier_switch_cnt = 0;
			msc_state = MSC_RSTC;
		} else if (batt_drv->checked_tier_switch_cnt >= switch_cnt) {
			/* next tier, fv_uv detemined at MSC_SET */
			vbatt_idx = batt_drv->vbatt_idx + 1;

			batt_prlog(BATT_PRLOG_ALWAYS,
				   "MSC_NEXT s:%d->%d tier vb=%d ib=%d vbatt_idx=%d->%d\n",
				   msc_state, MSC_NEXT, vbatt, ibatt,
				   batt_drv->vbatt_idx, vbatt_idx);

			msc_state = MSC_NEXT;
		} else {
			/* current under next tier, +1 on tier switch count */
			batt_drv->checked_tier_switch_cnt++;

			batt_prlog(BATT_PRLOG_ALWAYS,
				   "MSC_NYET s:%d->%d vt=%d vb=%d ib=%d cc_next_max=%d t_cnt=%d\n",
				   msc_state, MSC_NYET, vtier, vbatt, ibatt, cc_next_max,
				   batt_drv->checked_tier_switch_cnt);

			msc_state = MSC_NYET;
		}

	}

	if (msc_pm_hold(msc_state) == 0 && batt_drv->hold_taper_ws) {
		batt_drv->hold_taper_ws = false;
		__pm_relax(batt_drv->taper_ws);
	}

	/* need a new fv_uv only on a new voltage tier.  */
	if (vbatt_idx != batt_drv->vbatt_idx) {
		fv_uv = profile->volt_limits[vbatt_idx];
		batt_drv->checked_tier_switch_cnt = 0;
		batt_drv->checked_ov_cnt = 0;
	}

	/*
	 * book elapsed time to previous tier & msc_state
	 * NOTE: temp_idx != -1 but batt_drv->msc_state could be -1
	 */
	mutex_lock(&batt_drv->stats_lock);
	if (vbatt_idx != -1 && vbatt_idx < profile->volt_nb_limits) {
		int tier_idx = batt_chg_vbat2tier(batt_drv->vbatt_idx);

		/* this is the seed after the connect */
		if (tier_idx == -1) {
			tier_idx = batt_chg_vbat2tier(vbatt_idx);
			elap = 0;
		}

		batt_chg_stats_update(batt_drv, temp_idx, tier_idx,
				      ibatt / 1000, temp,
				      elap);

	}

	batt_drv->msc_state = msc_state;
	batt_drv->ce_data.last_update = now;
	mutex_unlock(&batt_drv->stats_lock);

	changed = batt_drv->temp_idx != temp_idx ||
		  batt_drv->vbatt_idx != vbatt_idx ||
		  batt_drv->fv_uv != fv_uv;
	batt_prlog(batt_prlog_level(changed),
		   "MSC_LOGIC temp_idx:%d->%d, vbatt_idx:%d->%d, fv=%d->%d, ui=%d->%d cv_cnt=%d ov_cnt=%d\n",
		   batt_drv->temp_idx, temp_idx, batt_drv->vbatt_idx, vbatt_idx,
		   batt_drv->fv_uv, fv_uv, batt_drv->cc_max, update_interval,
		   batt_drv->checked_cv_cnt, batt_drv->checked_ov_cnt);

	/* next update */
	batt_drv->msc_update_interval = update_interval;
	batt_drv->vbatt_idx = vbatt_idx;
	batt_drv->temp_idx = temp_idx;
	batt_drv->cc_max = GBMS_CCCM_LIMITS(profile, temp_idx, vbatt_idx);
	batt_drv->fv_uv = fv_uv;

	return 0;
}

/* no ssoc_delta when in overheat */
static int ssoc_get_delta(struct batt_drv *batt_drv)
{
	const bool overheat = batt_drv->batt_health ==
			      POWER_SUPPLY_HEALTH_OVERHEAT;

	return overheat ? 0 : qnum_fromint(batt_drv->ssoc_state.ssoc_delta);
}

/* TODO: handle the whole state buck_enable state */
static void ssoc_change_state(struct batt_ssoc_state *ssoc_state, bool ben)
{
	const ktime_t now = get_boot_sec();

	if (!ben) {
		ssoc_state->disconnect_time = now;
	} else if (ssoc_state->disconnect_time) {
		const u32 trickle_reset = ssoc_state->bd_trickle_reset_sec;
		const long long elap = now - ssoc_state->disconnect_time;

		if (trickle_reset && elap > trickle_reset)
			ssoc_state->bd_trickle_cnt = 0;

		pr_debug("MSC_BD: bd_trickle_cnt=%d dsc_time=%lld elap=%lld\n",
			 ssoc_state->bd_trickle_cnt,
			 ssoc_state->disconnect_time,
			 elap);

		ssoc_state->disconnect_time = 0;
	}

	ssoc_state->buck_enabled = ben;
}

static void bd_trickle_reset(struct batt_ssoc_state *ssoc_state)
{
	ssoc_state->bd_trickle_cnt = 0;
	ssoc_state->disconnect_time = 0;
}

static void batt_prlog_din(union gbms_charger_state *chg_state, int log_level)
{
	batt_prlog(log_level,
		   "MSC_DIN chg_state=%lx f=0x%x chg_s=%s chg_t=%s vchg=%d icl=%d\n",
		   (unsigned long)chg_state->v,
		   chg_state->f.flags,
		   gbms_chg_status_s(chg_state->f.chg_status),
		   gbms_chg_type_s(chg_state->f.chg_type),
		   chg_state->f.vchrg,
		   chg_state->f.icl);
}

/* called holding chg_lock */
static int batt_chg_logic(struct batt_drv *batt_drv)
{
	int err = 0;
	bool jeita_stop;
	bool changed = false;
	const bool disable_votes = batt_drv->disable_votes;
	union gbms_charger_state *chg_state = &batt_drv->chg_state;
	int log_vote_level = BATT_PRLOG_DEBUG;

	if (!batt_drv->chg_profile.cccm_limits)
		return -EINVAL;

	__pm_stay_awake(batt_drv->msc_ws);

	batt_prlog_din(chg_state, BATT_PRLOG_ALWAYS);

	/* disconnect! */
	if ((batt_drv->chg_state.f.flags & GBMS_CS_FLAG_BUCK_EN) == 0) {
		const qnum_t ssoc_delta = ssoc_get_delta(batt_drv);

		if (batt_drv->ssoc_state.buck_enabled == 0)
			goto msc_logic_exit;

		/* here on: disconnect */
		batt_chg_stats_pub(batt_drv, "disconnect", false, false);
		batt_res_state_set(&batt_drv->res_state, false);

		/* change curve before changing the state. */
		ssoc_change_curve(&batt_drv->ssoc_state, ssoc_delta,
				  SSOC_UIC_TYPE_DSG);

		batt_drv->chg_health.rest_deadline = 0;
		batt_reset_chg_drv_state(batt_drv);
		batt_update_cycle_count(batt_drv);
		batt_rl_reset(batt_drv);

		/* this will trigger another capacity learning. */
		err = GPSY_SET_PROP(batt_drv->fg_psy,
				    GBMS_PROP_BATT_CE_CTRL,
				    false);
		if (err < 0)
			pr_err("Cannot set the BATT_CE_CTRL.\n");

		/* TODO: move earlier and include the change to the curve */
		ssoc_change_state(&batt_drv->ssoc_state, 0);
		changed = true;

		goto msc_logic_done;
	}

	/*
	 * here when connected to power supply
	 * The following block one only on start.
	 */
	if (batt_drv->ssoc_state.buck_enabled <= 0) {
		const qnum_t ssoc_delta = ssoc_get_delta(batt_drv);

		/*
		 * FIX: BatteryDefenderUI needs use a different curve because
		 * bd->bd_voltage_trigger needs now to be 100%. In alternative
		 * we use the regular charge curve and show that charging stop
		 * BEFORE reaching 100%. This is similar to what we do if BD
		 * trigger over bd->bd_voltage_trigger BUT under SSOC=100%
		 */
		ssoc_change_curve(&batt_drv->ssoc_state, ssoc_delta,
				  SSOC_UIC_TYPE_CHG);

		if (batt_drv->res_state.estimate_filter)
			batt_res_state_set(&batt_drv->res_state, true);

		batt_chg_stats_start(batt_drv);
		err = GPSY_SET_PROP(batt_drv->fg_psy,
				    GBMS_PROP_BATT_CE_CTRL,
				    true);
		if (err < 0)
			pr_err("Cannot set the BATT_CE_CTRL.\n");

		/* released in battery_work() */
		__pm_stay_awake(batt_drv->poll_ws);
		batt_drv->batt_fast_update_cnt = BATT_WORK_FAST_RETRY_CNT;
		mod_delayed_work(system_wq, &batt_drv->batt_work,
				 msecs_to_jiffies(BATT_WORK_FAST_RETRY_MS));

		/* TODO: move earlier and include the change to the curve */
		ssoc_change_state(&batt_drv->ssoc_state, 1);
		changed = true;
	}

	/*
	 * enter RL in DISCHARGE on charger DONE and enter RL in RECHARGE on
	 * battery FULL (i.e. SSOC==100%). charger DONE forces the discharge
	 * curve while RECHARGE will not modify the current curve.
	 */
	if ((batt_drv->chg_state.f.flags & GBMS_CS_FLAG_DONE) != 0) {
		changed = batt_rl_enter(&batt_drv->ssoc_state,
					BATT_RL_STATUS_DISCHARGE);

		batt_drv->chg_done = true;
	} else if (batt_drv->batt_full) {
		changed = batt_rl_enter(&batt_drv->ssoc_state,
					BATT_RL_STATUS_RECHARGE);
	}

	err = msc_logic(batt_drv);
	if (err < 0) {
		/* NOTE: google charger will poll again. */
		batt_drv->msc_update_interval = -1;

		batt_prlog(BATT_PRLOG_ALWAYS,
			   "MSC_DOUT ERROR=%d fv_uv=%d cc_max=%d update_interval=%d\n",
			   err, batt_drv->fv_uv, batt_drv->cc_max,
			   batt_drv->msc_update_interval);

		goto msc_logic_exit;
	}

	/*
	 * TODO: might need to behave in a different way when health based
	 * charging is active
	 */
	changed |= msc_logic_health(batt_drv);
	if (CHG_HEALTH_REST_IS_ACTIVE(&batt_drv->chg_health)) {
		batt_drv->msc_state = MSC_HEALTH;
		/* make sure using rest_fv_uv when HEALTH_ACTIVE */
		batt_drv->fv_uv = 0;
	} else if (CHG_HEALTH_REST_IS_PAUSE(&batt_drv->chg_health)) {
		batt_drv->msc_state = MSC_HEALTH_PAUSE;
	}

msc_logic_done:

	/* set ->cc_max = 0 on RL and SW_JEITA, no vote on interval in RL_DSG */
	if (batt_drv->ssoc_state.rl_status == BATT_RL_STATUS_DISCHARGE) {
		log_vote_level = batt_prlog_level(batt_drv->cc_max != 0);
		batt_drv->msc_update_interval = -1;
		batt_drv->cc_max = 0;
	}

	jeita_stop = batt_drv->jeita_stop_charging == 1;
	if (jeita_stop) {
		log_vote_level = batt_prlog_level(batt_drv->cc_max != 0);
		batt_drv->cc_max = 0;
	}

	/* Fan level can be updated only during power transfer */
	if (batt_drv->fan_level_votable) {
		int level = fan_calculate_level(batt_drv);

		vote(batt_drv->fan_level_votable, "MSC_BATT", true, level);
		pr_debug("MSC_FAN_LVL: level=%d\n", level);
	}

	if (changed)
		log_vote_level = BATT_PRLOG_ALWAYS;
	batt_prlog(log_vote_level,
		   "%s msc_state=%d cv_cnt=%d ov_cnt=%d rl_sts=%d temp_idx:%d, vbatt_idx:%d  fv_uv=%d cc_max=%d update_interval=%d\n",
		   (disable_votes) ? "MSC_DOUT" : "MSC_VOTE",
		   batt_drv->msc_state,
		   batt_drv->checked_cv_cnt, batt_drv->checked_ov_cnt,
		   batt_drv->ssoc_state.rl_status,
		   batt_drv->temp_idx, batt_drv->vbatt_idx,
		   batt_drv->fv_uv, batt_drv->cc_max,
		   batt_drv->msc_update_interval);

	 /*
	  * google_charger has voted(<=0) on msc_interval_votable and the
	  * votes on fcc and fv_uv will not be applied until google_charger
	  * votes a non-zero value.
	  *
	  * SW_JEITA: ->jeita_stop_charging != 0
	  * . ->msc_update_interval = -1 , fv_uv = -1 and ->cc_max = 0
	  * . vote(0) on ->fcc_votable with SW_JEITA_VOTER
	  * BATT_RL: rl_status == BATT_RL_STATUS_DISCHARGE
	  * . ->msc_update_interval = -1 , fv_uv = -1 and ->cc_max = 0
	  * . vote(0) on ->fcc_votable with SW_JEITA_VOTER
	  *
	  * Votes for MSC_LOGIC_VOTER will be all disabled.
	  */
	if (!batt_drv->fv_votable)
		batt_drv->fv_votable = find_votable(VOTABLE_MSC_FV);
	if (batt_drv->fv_votable) {
		const int rest_fv_uv = batt_drv->chg_health.rest_fv_uv;

		vote(batt_drv->fv_votable, MSC_LOGIC_VOTER,
			!disable_votes && (batt_drv->fv_uv > 0),
			batt_drv->fv_uv);

		vote(batt_drv->fv_votable, MSC_HEALTH_VOTER,
			!disable_votes && (rest_fv_uv > 0),
			rest_fv_uv);
	}

	if (!batt_drv->fcc_votable)
		batt_drv->fcc_votable = find_votable(VOTABLE_MSC_FCC);
	if (batt_drv->fcc_votable) {
		enum batt_rl_status rl_status = batt_drv->ssoc_state.rl_status;
		const int rest_cc_max = batt_drv->chg_health.rest_cc_max;

		/* while in RL => ->cc_max != -1 && ->fv_uv != -1 */
		vote(batt_drv->fcc_votable, RL_STATE_VOTER,
			!disable_votes &&
			(rl_status == BATT_RL_STATUS_DISCHARGE),
			0);

		/* jeita_stop_charging != 0 => ->fv_uv = -1 && cc_max == -1 */
		vote(batt_drv->fcc_votable, SW_JEITA_VOTER,
			!disable_votes && jeita_stop,
			0);

		/* health based charging */
		vote(batt_drv->fcc_votable, MSC_HEALTH_VOTER,
			!disable_votes && (rest_cc_max != -1),
			rest_cc_max);

		vote(batt_drv->fcc_votable, MSC_LOGIC_VOTER,
			!disable_votes && (batt_drv->cc_max != -1),
			batt_drv->cc_max);

	}

	if (!batt_drv->msc_interval_votable)
		batt_drv->msc_interval_votable =
			find_votable(VOTABLE_MSC_INTERVAL);
	if (batt_drv->msc_interval_votable)
		vote(batt_drv->msc_interval_votable, MSC_LOGIC_VOTER,
			!disable_votes && (batt_drv->msc_update_interval != -1),
			batt_drv->msc_update_interval);

msc_logic_exit:

	if (changed) {
		dump_ssoc_state(&batt_drv->ssoc_state, batt_drv->ssoc_log);
		if (batt_drv->psy)
			power_supply_changed(batt_drv->psy);
	}

	__pm_relax(batt_drv->msc_ws);
	return err;
}

/* charge profile not in battery */
static int batt_init_chg_profile(struct batt_drv *batt_drv)
{
	struct device_node *node = batt_drv->device->of_node;
	struct gbms_chg_profile *profile = &batt_drv->chg_profile;
	int ret = 0;

	/* handle retry */
	if (!profile->cccm_limits) {
		ret = gbms_init_chg_profile(profile, node);
		if (ret < 0)
			return -EINVAL;
	}

	ret = of_property_read_u32(node, "google,chg-battery-capacity",
				   &batt_drv->battery_capacity);
	if (ret < 0)
		pr_warn("read chg-battery-capacity from gauge\n");

	/*
	 * use battery FULL design when is not specified in DT. When battery is
	 * not present use default capacity from DT (if present) or disable
	 * charging altogether.
	 */
	if (batt_drv->battery_capacity == 0) {
		u32 fc = 0;
		struct power_supply *fg_psy = batt_drv->fg_psy;

		if (batt_drv->batt_present) {
			fc = GPSY_GET_PROP(fg_psy,
					POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN);
			if (fc == -EAGAIN)
				return -EPROBE_DEFER;
			if (fc > 0) {
				pr_info("successfully read charging profile:\n");
				/* convert uA to mAh*/
				batt_drv->battery_capacity = fc / 1000;
			}

		}

		if (batt_drv->battery_capacity == 0) {
			struct device_node *node = batt_drv->device->of_node;

			ret = of_property_read_u32(node,
					"google,chg-battery-default-capacity",
						&batt_drv->battery_capacity);
			if (ret < 0)
				pr_warn("battery not present, no default capacity, zero charge table\n");
			else
				pr_warn("battery not present, using default capacity:\n");
		}
	}

	/* NOTE: with NG charger tolerance is applied from "charger" */
	gbms_init_chg_table(&batt_drv->chg_profile, batt_drv->battery_capacity);

	return 0;
}

/* ------------------------------------------------------------------------- */

/* call holding mutex_unlock(&ccd->lock); */
static int batt_cycle_count_store(struct gbatt_ccbin_data *ccd)
{
	int ret;

	ret = gbms_storage_write(GBMS_TAG_BCNT, ccd->count, sizeof(ccd->count));
	if (ret < 0 && ret != -ENOENT) {
		pr_err("failed to set bin_counts ret=%d\n", ret);
		return ret;
	}

	return 0;
}

/* call holding mutex_unlock(&ccd->lock); */
static int batt_cycle_count_load(struct gbatt_ccbin_data *ccd)
{
	int ret;

	ret = gbms_storage_read(GBMS_TAG_BCNT, ccd->count, sizeof(ccd->count));
	if (ret < 0 && ret != -ENOENT) {
		pr_err("failed to get bin_counts ret=%d\n", ret);
		return ret;
	}

	ccd->prev_soc = -1;
	return 0;
}

/* update only when SSOC is increasing, not need to check charging */
static void batt_cycle_count_update(struct batt_drv *batt_drv, int soc)
{
	struct gbatt_ccbin_data *ccd = &batt_drv->cc_data;

	if (soc < 0 || soc > 100)
		return;

	mutex_lock(&ccd->lock);

	if (ccd->prev_soc != -1 && soc > ccd->prev_soc) {
		int bucket, cnt;

		for (cnt = soc ; cnt > ccd->prev_soc ; cnt--) {
			/* cnt decremented by 1 for bucket symmetry */
			bucket = (cnt - 1) * GBMS_CCBIN_BUCKET_COUNT / 100;
			ccd->count[bucket]++;
		}

		/* NOTE: could store on FULL or disconnect instead */
		(void)batt_cycle_count_store(ccd);
	}

	ccd->prev_soc = soc;

	mutex_unlock(&ccd->lock);
}

/* ------------------------------------------------------------------------- */

#ifdef CONFIG_DEBUG_FS

#define BATTERY_DEBUG_ATTRIBUTE(name, fn_read, fn_write) \
static const struct file_operations name = {	\
	.open	= simple_open,			\
	.llseek	= no_llseek,			\
	.read	= fn_read,			\
	.write	= fn_write,			\
}

static ssize_t cycle_counts_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int ret;

	mutex_lock(&batt_drv->cc_data.lock);

	ret = gbms_cycle_count_sscan(batt_drv->cc_data.count, buf);
	if (ret == 0) {
		ret = batt_cycle_count_store(&batt_drv->cc_data);
		if (ret < 0)
			pr_err("cannot store bin count ret=%d\n", ret);
	}

	if (ret == 0)
		ret = count;

	mutex_unlock(&batt_drv->cc_data.lock);

	return ret;
}

static ssize_t cycle_counts_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buff)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int len;

	mutex_lock(&batt_drv->cc_data.lock);
	len = gbms_cycle_count_cstr(buff, PAGE_SIZE, batt_drv->cc_data.count);
	mutex_unlock(&batt_drv->cc_data.lock);

	return len;
}

static const DEVICE_ATTR_RW(cycle_counts);

/* Was POWER_SUPPLY_PROP_RESISTANCE */
static ssize_t resistance_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buff)
{
	u32 data;
	int ret;

	ret = gbms_storage_read(GBMS_TAG_BRES, &data, sizeof(data));

	return scnprintf(buff, PAGE_SIZE, "%d\n", ret < 0 ? ret : data);
}

static const DEVICE_ATTR_RO(resistance);

/* Was POWER_SUPPLY_PROP_RESISTANCE_AVG */
static ssize_t resistance_avg_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buff)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct batt_res *res_state = &batt_drv->res_state;
	int value = 0;

	if (res_state->filter_count >= res_state->estimate_filter)
		value = res_state->resistance_avg;

	return scnprintf(buff, PAGE_SIZE, "%d\n", value);
}

static const DEVICE_ATTR_RO(resistance_avg);

/* Was POWER_SUPPLY_PROP_CHARGE_FULL_ESTIMATE */
static ssize_t charge_full_estimate_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buff)
{
	u16 data;
	int ret;

	ret = gbms_storage_read(GBMS_TAG_GCFE, &data, sizeof(data));

	return scnprintf(buff, PAGE_SIZE, "%d\n", ret < 0 ? ret : data * 1000);
}

static const DEVICE_ATTR_RO(charge_full_estimate);


static int cycle_count_bins_store(void *data, u64 val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;
	int ret;

	mutex_lock(&batt_drv->cc_data.lock);
	ret = batt_cycle_count_store(&batt_drv->cc_data);
	if (ret < 0)
		pr_err("cannot store bin count ret=%d\n", ret);
	mutex_unlock(&batt_drv->cc_data.lock);

	return ret;
}

static int cycle_count_bins_reload(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;
	int ret;

	mutex_lock(&batt_drv->cc_data.lock);
	ret = batt_cycle_count_load(&batt_drv->cc_data);
	if (ret < 0)
		pr_err("cannot restore bin count ret=%d\n", ret);
	mutex_unlock(&batt_drv->cc_data.lock);
	*val = ret;

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(cycle_count_bins_sync_fops,
				cycle_count_bins_reload,
				cycle_count_bins_store, "%llu\n");


static int debug_get_ssoc_gdf(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;
	*val = batt_drv->ssoc_state.ssoc_gdf;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_ssoc_gdf_fops, debug_get_ssoc_gdf, NULL, "%llu\n");


static int debug_get_ssoc_uic(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;
	*val = batt_drv->ssoc_state.ssoc_uic;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_ssoc_uic_fops, debug_get_ssoc_uic, NULL, "%llu\n");

static int debug_get_ssoc_rls(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	mutex_lock(&batt_drv->chg_lock);
	*val = batt_drv->ssoc_state.rl_status;
	mutex_unlock(&batt_drv->chg_lock);

	return 0;
}

static int debug_set_ssoc_rls(void *data, u64 val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	if (val < 0 || val > 2)
		return -EINVAL;

	mutex_lock(&batt_drv->chg_lock);
	batt_drv->ssoc_state.rl_status = val;
	if (!batt_drv->fcc_votable)
		batt_drv->fcc_votable = find_votable(VOTABLE_MSC_FCC);
	if (batt_drv->fcc_votable)
		vote(batt_drv->fcc_votable, RL_STATE_VOTER,
			batt_drv->ssoc_state.rl_status ==
						BATT_RL_STATUS_DISCHARGE,
			0);
	mutex_unlock(&batt_drv->chg_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_ssoc_rls_fops,
				debug_get_ssoc_rls, debug_set_ssoc_rls, "%llu\n");


static ssize_t debug_get_ssoc_uicurve(struct file *filp,
					   char __user *buf,
					   size_t count, loff_t *ppos)
{
	struct batt_drv *batt_drv = (struct batt_drv *)filp->private_data;
	char tmp[UICURVE_BUF_SZ] = { 0 };

	mutex_lock(&batt_drv->chg_lock);
	ssoc_uicurve_cstr(tmp, sizeof(tmp), batt_drv->ssoc_state.ssoc_curve);
	mutex_unlock(&batt_drv->chg_lock);

	return simple_read_from_buffer(buf, count, ppos, tmp, strlen(tmp));
}

static ssize_t debug_set_ssoc_uicurve(struct file *filp,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct batt_drv *batt_drv = (struct batt_drv *)filp->private_data;
	int ret, curve_type;
	char buf[8];

	ret = simple_write_to_buffer(buf, sizeof(buf), ppos, user_buf, count);
	if (!ret)
		return -EFAULT;

	mutex_lock(&batt_drv->chg_lock);

	/* FIX: BatteryDefenderUI doesn't really handle this yet */
	curve_type = (int)simple_strtoull(buf, NULL, 10);
	if (curve_type >= -1 && curve_type <= 1)
		ssoc_change_curve(&batt_drv->ssoc_state, 0, curve_type);
	else
		ret = -EINVAL;

	mutex_unlock(&batt_drv->chg_lock);

	if (ret == 0)
		ret = count;

	return 0;
}

BATTERY_DEBUG_ATTRIBUTE(debug_ssoc_uicurve_cstr_fops,
					debug_get_ssoc_uicurve,
					debug_set_ssoc_uicurve);

static int debug_force_psy_update(void *data, u64 val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	if (!batt_drv->psy)
		return -EINVAL;

	power_supply_changed(batt_drv->psy);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_force_psy_update_fops,
				NULL, debug_force_psy_update, "%llu\n");

/* Adaptive Charging */
static int debug_chg_health_rest_rate_read(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	if (!batt_drv->psy)
		return -EINVAL;

	*val = batt_drv->chg_health.rest_rate;
	return 0;
}

/* Adaptive Charging */
static int debug_chg_health_rest_rate_write(void *data, u64 val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	if (!batt_drv->psy)
		return -EINVAL;

	batt_drv->chg_health.rest_rate = val;
	return 0;
}

/* Adaptive Charging */
DEFINE_SIMPLE_ATTRIBUTE(debug_chg_health_rest_rate_fops,
			debug_chg_health_rest_rate_read,
			debug_chg_health_rest_rate_write, "%llu\n");

/* Adaptive Charging */
static int debug_chg_health_thr_soc_read(void *data, u64 *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	if (!batt_drv->psy)
		return -EINVAL;

	*val = batt_drv->chg_health.rest_soc;
	return 0;
}

/* Adaptive Charging */
static int debug_chg_health_thr_soc_write(void *data, u64 val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	if (!batt_drv->psy)
		return -EINVAL;

	batt_drv->chg_health.rest_soc = val;
	return 0;
}

/* Adaptive Charging */
DEFINE_SIMPLE_ATTRIBUTE(debug_chg_health_thr_soc_fops,
			debug_chg_health_thr_soc_read,
			debug_chg_health_thr_soc_write, "%llu\n");

/* Adaptive Charging */
static int debug_chg_health_set_stage(void *data, u64 val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)data;

	if (!batt_drv->psy)
		return -EINVAL;

	switch (val) {
	case CHG_HEALTH_DISABLED:
	case CHG_HEALTH_INACTIVE:
	case CHG_HEALTH_ENABLED:
	case CHG_HEALTH_ACTIVE:
	case CHG_HEALTH_DONE:
		break;
	default:
		return -EINVAL;
	}

	batt_drv->chg_health.rest_state = val;
	return 0;
}

/* Adaptive Charging */
DEFINE_SIMPLE_ATTRIBUTE(debug_chg_health_stage_fops, NULL,
			debug_chg_health_set_stage, "%llu\n");
#endif

/* ------------------------------------------------------------------------- */

static ssize_t debug_get_fake_temp(struct file *filp,
					   char __user *buf,
					   size_t count, loff_t *ppos)
{
	struct batt_drv *batt_drv = (struct batt_drv *)filp->private_data;
	char tmp[8];

	mutex_lock(&batt_drv->chg_lock);
	scnprintf(tmp, sizeof(tmp), "%d\n", batt_drv->fake_temp);
	mutex_unlock(&batt_drv->chg_lock);

	return simple_read_from_buffer(buf, count, ppos, tmp, strlen(tmp));
}

static ssize_t debug_set_fake_temp(struct file *filp,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct batt_drv *batt_drv = (struct batt_drv *)filp->private_data;
	int ret = 0, val;
	char buf[8];

	ret = simple_write_to_buffer(buf, sizeof(buf), ppos, user_buf, count);
	if (!ret)
		return -EFAULT;

	buf[ret] = '\0';
	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	mutex_lock(&batt_drv->chg_lock);
	batt_drv->fake_temp = val;
	mutex_unlock(&batt_drv->chg_lock);

	return count;
}

BATTERY_DEBUG_ATTRIBUTE(debug_fake_temp_fops, debug_get_fake_temp,
			debug_set_fake_temp);


static enum batt_paired_state
batt_reset_pairing_state(const struct batt_drv *batt_drv)
{
	char dev_info[GBMS_DINF_LEN];
	int ret = 0;

	memset(dev_info, 0xff, sizeof(dev_info));
	ret = gbms_storage_write(GBMS_TAG_DINF, dev_info, sizeof(dev_info));
	if (ret < 0)
		return -EIO;

	return 0;
}

static ssize_t debug_set_pairing_state(struct file *filp,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct batt_drv *batt_drv = (struct batt_drv *)filp->private_data;
	int ret = 0, val;
	char buf[8];

	ret = simple_write_to_buffer(buf, sizeof(buf), ppos, user_buf, count);
	if (ret <= 0)
		return ret;

	buf[ret] = '\0';
	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	mutex_lock(&batt_drv->chg_lock);

	if (val == BATT_PAIRING_ENABLED) {
		batt_drv->pairing_state = BATT_PAIRING_ENABLED;
		mod_delayed_work(system_wq, &batt_drv->batt_work, 0);
	} else if (val == BATT_PAIRING_RESET) {

		/* send a paring enable to re-pair OR reboot */
		ret = batt_reset_pairing_state(batt_drv);
		if (ret == 0)
			batt_drv->pairing_state = BATT_PAIRING_DISABLED;
		else
			count = -EIO;
	} else {
		count = -EINVAL;
	}
	mutex_unlock(&batt_drv->chg_lock);

	return count;
}

BATTERY_DEBUG_ATTRIBUTE(debug_pairing_fops, 0, debug_set_pairing_state);

/* TODO: add write to stop/start collection, erase history etc. */
static ssize_t debug_get_blf_state(struct file *filp, char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct batt_drv *batt_drv = (struct batt_drv *)filp->private_data;
	char tmp[8];

	mutex_lock(&batt_drv->chg_lock);
	scnprintf(tmp, sizeof(tmp), "%d\n", batt_drv->blf_state);
	mutex_unlock(&batt_drv->chg_lock);

	return simple_read_from_buffer(buf, count, ppos, tmp, strlen(tmp));
}
BATTERY_DEBUG_ATTRIBUTE(debug_blf_state_fops, debug_get_blf_state, 0);

/* TODO: add writes to restart pairing (i.e. provide key) */
static ssize_t batt_pairing_state_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int len;

	mutex_lock(&batt_drv->chg_lock);
	len = scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->pairing_state);
	mutex_unlock(&batt_drv->chg_lock);
	return len;
}

static const DEVICE_ATTR(pairing_state, 0444, batt_pairing_state_show, NULL);


static ssize_t batt_ctl_chg_stats_actual(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);

	if (count < 1)
		return -ENODATA;

	switch (buf[0]) {
	case 'p': /* publish data to qual */
	case 'P': /* force publish data to qual */
		batt_chg_stats_pub(batt_drv, "debug cmd", buf[0] == 'P', false);
		break;
	default:
		count = -EINVAL;
		break;
	}

	return count;
}

static ssize_t batt_show_chg_stats_actual(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	int len;

	mutex_lock(&batt_drv->stats_lock);
	len = batt_chg_stats_cstr(buf, PAGE_SIZE, &batt_drv->ce_data, false);
	mutex_unlock(&batt_drv->stats_lock);

	return len;
}

static const DEVICE_ATTR(charge_stats_actual, 0664,
					     batt_show_chg_stats_actual,
					     batt_ctl_chg_stats_actual);

static ssize_t batt_ctl_chg_stats(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);

	if (count < 1)
		return -ENODATA;

	mutex_lock(&batt_drv->stats_lock);
	switch (buf[0]) {
	case 0:
	case '0': /* invalidate current qual */
		cev_stats_init(&batt_drv->ce_qual, &batt_drv->chg_profile);
		break;
	}
	mutex_unlock(&batt_drv->stats_lock);

	return count;
}

/* regular and health stats */
static ssize_t batt_chg_qual_stats_cstr(char *buff, int size,
					struct gbms_charging_event *ce_qual,
					bool verbose)
{
	ssize_t len = 0;

	len += batt_chg_stats_cstr(&buff[len], size - len, ce_qual, verbose);
	if (ce_qual->ce_health.rest_state != CHG_HEALTH_INACTIVE)
		len += batt_health_stats_cstr(&buff[len], size - len,
					      ce_qual, verbose);
	return len;
}

static ssize_t batt_show_chg_stats(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	struct gbms_charging_event *ce_qual = &batt_drv->ce_qual;
	int len = -ENODATA;

	mutex_lock(&batt_drv->stats_lock);
	if (ce_qual->last_update - ce_qual->first_update)
		len = batt_chg_qual_stats_cstr(buf, PAGE_SIZE, ce_qual, false);
	mutex_unlock(&batt_drv->stats_lock);

	return len;
}

static const DEVICE_ATTR(charge_stats, 0664, batt_show_chg_stats,
					     batt_ctl_chg_stats);

/* show current/active and qual data */
static ssize_t batt_show_chg_details(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	struct gbms_charging_event *ce_data = &batt_drv->ce_data;
	const bool qual_valid = (batt_drv->ce_qual.last_update -
				batt_drv->ce_qual.first_update) != 0;
	int len = 0;

	mutex_lock(&batt_drv->stats_lock);

	/* this is the current one */
	len += batt_chg_stats_cstr(&buf[len], PAGE_SIZE - len, ce_data, true);

	/*
	 * stats are accumulated in ce_data->health_stats, rest_* fields
	 * are set on stats_close()
	 */
	if (batt_drv->chg_health.rest_state != CHG_HEALTH_INACTIVE) {
		const struct gbms_ce_tier_stats *h = &batt_drv->ce_data.health_stats;
		const struct gbms_ce_tier_stats *p = &batt_drv->ce_data.health_pause_stats;
		const long elap_h = h->time_fast + h->time_taper + h->time_other;
		const long elap_p = p->time_fast + p->time_taper + p->time_other;
		const ktime_t now = get_boot_sec();
		int vti;

		vti = batt_chg_health_vti(&batt_drv->chg_health);
		len += scnprintf(&buf[len], PAGE_SIZE - len,
				"\nH: %d %d %ld %ld %lld %lld %d",
				batt_drv->chg_health.rest_state,
				vti, elap_h, elap_p, now,
				batt_drv->chg_health.rest_deadline,
				batt_drv->chg_health.always_on_soc);

		/* NOTE: vtier_idx is -1, can also check elap  */
		if (h->soc_in != -1)
			len += batt_chg_tier_stats_cstr(&buf[len],
							PAGE_SIZE - len, h, !!elap_h);
		if (p->soc_in != -1)
			len += batt_chg_tier_stats_cstr(&buf[len],
							PAGE_SIZE - len, p, !!elap_p);
	}

	len += scnprintf(&buf[len], PAGE_SIZE - len, "\n");

	/* this was the last one (if present) */
	if (qual_valid) {
		len += batt_chg_qual_stats_cstr(&buf[len], PAGE_SIZE - len,
						&batt_drv->ce_qual, true);
		len += scnprintf(&buf[len], PAGE_SIZE - len, "\n");
	}

	mutex_unlock(&batt_drv->stats_lock);

	return len;
}

static const DEVICE_ATTR(charge_details, 0444, batt_show_chg_details,
					       NULL);

/* tier and soc details */
static ssize_t batt_show_ttf_details(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = (struct batt_drv *)
					power_supply_get_drvdata(psy);
	struct batt_ttf_stats *ttf_stats;
	int len;

	if (!batt_drv->ssoc_state.buck_enabled)
		return -ENODATA;

	ttf_stats = kzalloc(sizeof(*ttf_stats), GFP_KERNEL);
	if (!ttf_stats)
		return -ENOMEM;

	mutex_lock(&batt_drv->stats_lock);
	/* update a private copy of ttf stats */
	ttf_stats_update(ttf_stats_dup(ttf_stats, &batt_drv->ttf_stats),
			 &batt_drv->ce_data, false);
	mutex_unlock(&batt_drv->stats_lock);

	len = ttf_dump_details(buf, PAGE_SIZE, ttf_stats,
			       batt_drv->ce_data.last_soc);
	kfree(ttf_stats);

	return len;
}

static const DEVICE_ATTR(ttf_details, 0444, batt_show_ttf_details, NULL);

/* house stats */
static ssize_t batt_show_ttf_stats(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	const int verbose = true;
	int i, len = 0;

	mutex_lock(&batt_drv->stats_lock);

	for (i = 0; i < GBMS_STATS_TIER_COUNT; i++)
		len += ttf_tier_cstr(&buf[len], PAGE_SIZE,
				     &batt_drv->ttf_stats.tier_stats[i]);

	len += scnprintf(&buf[len], PAGE_SIZE - len, "\n");

	if (verbose)
		len += ttf_soc_cstr(&buf[len], PAGE_SIZE - len,
				    &batt_drv->ttf_stats.soc_stats,
				    0, 99);

	mutex_unlock(&batt_drv->stats_lock);

	return len;
}

/* userspace restore the TTF data with this */
static ssize_t batt_ctl_ttf_stats(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int res;
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);

	if (count < 1)
		return -ENODATA;
	if (!batt_drv->ssoc_state.buck_enabled)
		return -ENODATA;

	mutex_lock(&batt_drv->stats_lock);
	switch (buf[0]) {
	case 'u':
	case 'U': /* force update */
		ttf_stats_update(&batt_drv->ttf_stats, &batt_drv->ce_data,
				 (buf[0] == 'U'));
		break;
	default:
		/* TODO: userspace restore of the data */
		res = ttf_stats_sscan(&batt_drv->ttf_stats, buf, count);
		if (res < 0)
			count = res;
		break;
	}
	mutex_unlock(&batt_drv->stats_lock);

	return count;
}

static const DEVICE_ATTR(ttf_stats, 0664, batt_show_ttf_stats,
					  batt_ctl_ttf_stats);

/* ------------------------------------------------------------------------- */

static ssize_t chg_health_show_stage(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = (struct batt_drv *)
					power_supply_get_drvdata(psy);
	const char *s = "Inactive";

	mutex_lock(&batt_drv->chg_lock);
	switch (batt_drv->chg_health.rest_state) {
	case CHG_HEALTH_DISABLED:
		s = "Disabled";
		break;
	case CHG_HEALTH_ENABLED:
		s = "Enabled";
		break;
	case CHG_HEALTH_PAUSE:
	case CHG_HEALTH_ACTIVE:
		s = "Active";
		break;
	case CHG_HEALTH_DONE:
		s = "Done";
		break;
	default:
		break;
	}
	mutex_unlock(&batt_drv->chg_lock);

	return scnprintf(buf, PAGE_SIZE, "%s\n", s);
}

static const DEVICE_ATTR(charge_stage, 0444, chg_health_show_stage, NULL);

static ssize_t chg_health_charge_limit_get(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 batt_drv->chg_health.always_on_soc);
}
/* setting disable (deadline = -1) or replug (deadline == 0) will disable */
static ssize_t chg_health_charge_limit_set(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	const int always_on_soc = simple_strtol(buf, NULL, 10);
	enum chg_health_state rest_state;

	/* Always enable AC when SOC is over trigger */
	if (always_on_soc < -1 || always_on_soc > 99)
		return -EINVAL;

	mutex_lock(&batt_drv->chg_lock);

	/*
	 * There are interesting overlaps with the AC standard behavior since
	 * the aon limit can be set at any time (and while AC limit is active)
	 * TODO: fully document the state machine
	 */
	rest_state = batt_drv->chg_health.rest_state;

	if (always_on_soc != -1) {
		switch (rest_state) {
		case CHG_HEALTH_DISABLED: /* didn't meet deadline */
		case CHG_HEALTH_INACTIVE: /* deadline was not provided */
			rest_state = CHG_HEALTH_ENABLED;
			break;
		default:
			/* _DONE, _ENABLED, _ACTIVE, _USER_DISABLED */
			break;
		}
	} else if (batt_drv->chg_health.always_on_soc != -1) {

		switch (rest_state) {
		case CHG_HEALTH_ENABLED: /* waiting for always_on_soc */
		case CHG_HEALTH_ACTIVE: /* activated at always_on_soc */
			if (batt_drv->chg_health.rest_deadline > 0)
				rest_state = CHG_HEALTH_ENABLED;
			else
				rest_state = CHG_HEALTH_INACTIVE;
			break;
		default:
			/* _DONE, _DISABLED, _USER_DISABLED */
			break;
		}
	}

	batt_drv->chg_health.always_on_soc = always_on_soc;
	batt_drv->chg_health.rest_state = rest_state;

	mutex_unlock(&batt_drv->chg_lock);
	power_supply_changed(batt_drv->psy);
	return count;
}

static DEVICE_ATTR(charge_limit, 0660, chg_health_charge_limit_get,
		   chg_health_charge_limit_set);

static ssize_t batt_show_chg_deadline(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	const ktime_t now = get_boot_sec();
	long long deadline = 0;

	mutex_lock(&batt_drv->chg_lock);

	/*
	 * = (rest_deadline <= 0) means state is either Inactive or Disabled
	 * = (rest_deadline < now) means state is either Done or Disabled
	 *
	 * State becomes Disabled from Enabled or Active when/if msc_logic()
	 * determines that the device cannot reach full before the deadline.
	 *
	 * UI checks for:
	 *   (stage == 'Active' || stage == 'Enabled') && deadline > 0
	 */
	deadline = batt_drv->chg_health.rest_deadline;
	if (deadline > 0 && deadline > now)
		deadline -= now;
	else if (deadline > 0)
		deadline = 0;

	mutex_unlock(&batt_drv->chg_lock);

	/*
	 * deadline < 0 feature disabled. deadline = 0 expired or disabled for
	 * this session, deadline > 0 time to deadline otherwise.
	 */
	return scnprintf(buf, PAGE_SIZE, "%lld\n", (long long)deadline);
}

/* userspace restore the TTF data with this */
static ssize_t batt_set_chg_deadline(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	long long deadline_s;
	bool changed;

	/* API works in seconds */
	deadline_s = simple_strtoll(buf, NULL, 10);

	mutex_lock(&batt_drv->chg_lock);
	/* Let deadline < 0 pass to set stats */
	if (!batt_drv->ssoc_state.buck_enabled && deadline_s >= 0) {
		mutex_unlock(&batt_drv->chg_lock);
		return -EINVAL;
	}

	changed = batt_health_set_chg_deadline(&batt_drv->chg_health,
					       deadline_s);
	mutex_unlock(&batt_drv->chg_lock);

	if (changed)
		power_supply_changed(batt_drv->psy);

	pr_info("MSC_HEALTH deadline_s=%lld deadline at %lld\n",
		deadline_s, batt_drv->chg_health.rest_deadline);
	logbuffer_log(batt_drv->ttf_stats.ttf_log,
		     "MSC_HEALTH: deadline_s=%ld deadline at %ld",
		     deadline_s, batt_drv->chg_health.rest_deadline);

	return count;
}

static const DEVICE_ATTR(charge_deadline, 0664, batt_show_chg_deadline,
						batt_set_chg_deadline);

enum batt_ssoc_status {
	BATT_SSOC_STATUS_UNKNOWN = 0,
	BATT_SSOC_STATUS_CONNECTED = 1,
	BATT_SSOC_STATUS_DISCONNECTED = 2,
	BATT_SSOC_STATUS_FULL = 3,
};

static ssize_t ssoc_details_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	int len = 0;
	enum batt_ssoc_status status = BATT_SSOC_STATUS_UNKNOWN;
	char buff[UICURVE_BUF_SZ] = { 0 };

	mutex_lock(&batt_drv->chg_lock);

	if (ssoc_state->buck_enabled == 0) {
		status = BATT_SSOC_STATUS_DISCONNECTED;
	} else if (ssoc_state->buck_enabled == 1) {
		if (batt_drv->batt_full)
			status = BATT_SSOC_STATUS_FULL;
		else
			status = BATT_SSOC_STATUS_CONNECTED;
	}

	len = scnprintf(
		buf, sizeof(ssoc_state->ssoc_state_cstr),
		"soc: l=%d%% gdf=%d.%02d uic=%d.%02d rl=%d.%02d\n"
		"curve:%s\n"
		"status: ct=%d rl=%d s=%d\n",
		ssoc_get_capacity(ssoc_state), qnum_toint(ssoc_state->ssoc_gdf),
		qnum_fracdgt(ssoc_state->ssoc_gdf),
		qnum_toint(ssoc_state->ssoc_uic),
		qnum_fracdgt(ssoc_state->ssoc_uic),
		qnum_toint(ssoc_state->ssoc_rl),
		qnum_fracdgt(ssoc_state->ssoc_rl),
		ssoc_uicurve_cstr(buff, sizeof(buff), ssoc_state->ssoc_curve),
		ssoc_state->ssoc_curve_type, ssoc_state->rl_status, status);

	mutex_unlock(&batt_drv->chg_lock);

	return len;
}

static const DEVICE_ATTR_RO(ssoc_details);

static ssize_t show_bd_trickle_enable(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 batt_drv->ssoc_state.bd_trickle_enable);
}

static ssize_t set_bd_trickle_enable(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	batt_drv->ssoc_state.bd_trickle_enable = !!val;

	return count;
}

static DEVICE_ATTR(bd_trickle_enable, 0660,
		   show_bd_trickle_enable, set_bd_trickle_enable);

static ssize_t show_bd_trickle_cnt(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 batt_drv->ssoc_state.bd_trickle_cnt);
}

static ssize_t set_bd_trickle_cnt(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	batt_drv->ssoc_state.bd_trickle_cnt = val;

	return count;
}

static DEVICE_ATTR(bd_trickle_cnt, 0660,
		   show_bd_trickle_cnt, set_bd_trickle_cnt);

static ssize_t show_bd_trickle_recharge_soc(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 batt_drv->ssoc_state.bd_trickle_recharge_soc);
}

#define BD_RL_SOC_FULL		100
#define BD_RL_SOC_LOW		50
static ssize_t set_bd_trickle_recharge_soc(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if ((val >= BD_RL_SOC_FULL) || (val < BD_RL_SOC_LOW))
		return count;

	batt_drv->ssoc_state.bd_trickle_recharge_soc = val;

	return count;
}

static DEVICE_ATTR(bd_trickle_recharge_soc, 0660,
		   show_bd_trickle_recharge_soc, set_bd_trickle_recharge_soc);

static ssize_t show_bd_trickle_dry_run(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 batt_drv->ssoc_state.bd_trickle_dry_run);
}

static ssize_t set_bd_trickle_dry_run(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	batt_drv->ssoc_state.bd_trickle_dry_run = val ? true : false;

	return count;
}

static DEVICE_ATTR(bd_trickle_dry_run, 0660,
		   show_bd_trickle_dry_run, set_bd_trickle_dry_run);

static ssize_t show_bd_trickle_reset_sec(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 batt_drv->ssoc_state.bd_trickle_reset_sec);
}

static ssize_t set_bd_trickle_reset_sec(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	unsigned int val;
	int ret = 0;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	batt_drv->ssoc_state.bd_trickle_reset_sec = val;

	return count;
}

static DEVICE_ATTR(bd_trickle_reset_sec, 0660,
		   show_bd_trickle_reset_sec, set_bd_trickle_reset_sec);

static ssize_t bd_clear_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int ret = 0, val = 0;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val)
		bd_trickle_reset(&batt_drv->ssoc_state);

	return count;
}

static DEVICE_ATTR_WO(bd_clear);

static ssize_t batt_show_time_to_ac(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);
	const int soc = CHG_HEALTH_REST_SOC(&batt_drv->chg_health);
	qnum_t soc_raw = ssoc_get_capacity_raw(&batt_drv->ssoc_state);
	qnum_t soc_health = qnum_fromint(soc);
	ktime_t estimate;
	int rc;

	rc = ttf_soc_estimate(&estimate, &batt_drv->ttf_stats,
			      &batt_drv->ce_data, soc_raw,
			      soc_health - qnum_rconst(SOC_ROUND_BASE));
	if (rc < 0)
		estimate = -1;

	if (estimate == -1)
		return -ERANGE;

	return scnprintf(buf, PAGE_SIZE, "%lld\n", (long long)estimate);
}

static const DEVICE_ATTR(time_to_ac, 0444, batt_show_time_to_ac, NULL);

static ssize_t batt_show_ac_soc(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv =(struct batt_drv *)
					power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 CHG_HEALTH_REST_SOC(&batt_drv->chg_health));
}

static const DEVICE_ATTR(ac_soc, 0444, batt_show_ac_soc, NULL);


static ssize_t batt_show_charger_state(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "0x%llx\n", batt_drv->chg_state.v);
}

static const DEVICE_ATTR(charger_state, 0444, batt_show_charger_state, NULL);


static ssize_t batt_show_charge_type(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->chg_state.f.chg_type);
}

static const DEVICE_ATTR(charge_type, 0444, batt_show_charge_type, NULL);


static ssize_t batt_show_constant_charge_current(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->cc_max);
}

static const DEVICE_ATTR(constant_charge_current, 0444,
			 batt_show_constant_charge_current, NULL);


static ssize_t batt_show_constant_charge_voltage(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n", batt_drv->fv_uv);
}

static const DEVICE_ATTR(constant_charge_voltage, 0444,
			 batt_show_constant_charge_voltage, NULL);

static ssize_t fan_level_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count) {
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int ret = 0;
	int level;

	ret = kstrtoint(buf, 0, &level);
	if (ret < 0)
		return ret;

	if ((level < FAN_LVL_UNKNOWN) || (level > FAN_LVL_ALARM))
		return -ERANGE;

	batt_drv->fan_level = level;

	/* always send a power supply event when forcing the value */
	if (batt_drv->psy)
		power_supply_changed(batt_drv->psy);

	return count;
}

static ssize_t fan_level_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int result = 0;

	if (batt_drv->fan_level == -1 && batt_drv->fan_level_votable)
		result = get_effective_result_locked(batt_drv->fan_level_votable);
	else
		result = batt_drv->fan_level;

	return scnprintf(buf, PAGE_SIZE, "%d\n", result);
}

static const DEVICE_ATTR_RW(fan_level);

static ssize_t show_health_safety_margin(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 batt_drv->health_safety_margin);
}

static ssize_t set_health_safety_margin(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct power_supply *psy = container_of(dev, struct power_supply, dev);
	struct batt_drv *batt_drv = power_supply_get_drvdata(psy);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	/*
	 * less than 0 is not accaptable: we will not reach full in time.
	 * set to 0 to disable PAUSE but keep AC charge
	 */
	if (val < 0)
		val = 0;

	batt_drv->health_safety_margin = val;

	return count;
}

static DEVICE_ATTR(health_safety_margin, 0660,
		    show_health_safety_margin, set_health_safety_margin);

/* ------------------------------------------------------------------------- */

static int batt_init_fs(struct batt_drv *batt_drv)
{
	struct dentry *de = NULL;
	int ret;

	/* stats */
	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_charge_stats);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create charge_stats\n");

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_charge_stats_actual);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create charge_stats_actual\n");

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_charge_details);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create charge_details\n");

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_ssoc_details);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create ssoc_details\n");

	/* health based charging */
	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_charge_deadline);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create chg_deadline\n");

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_charge_stage);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create charge_stage\n");

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_charge_limit);
	if (ret != 0)
		dev_err(&batt_drv->psy->dev, "Failed to create charge_limit\n");

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_time_to_ac);
	if (ret != 0)
		dev_err(&batt_drv->psy->dev, "Failed to create time_to_ac\n");

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_ac_soc);
	if (ret != 0)
		dev_err(&batt_drv->psy->dev, "Failed to create ac_soc\n");

	/* time to full */
	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_ttf_stats);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create ttf_stats\n");

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_ttf_details);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create ttf_details\n");

	/* TRICKLE-DEFEND */
	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_bd_trickle_enable);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create bd_trickle_enable\n");

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_bd_trickle_cnt);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create bd_trickle_cnt\n");

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_bd_trickle_recharge_soc);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create bd_trickle_recharge_soc\n");

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_bd_trickle_dry_run);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create bd_trickle_dry_run\n");

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_bd_trickle_reset_sec);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create bd_trickle_reset_sec\n");

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_bd_clear);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create bd_clear\n");

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_pairing_state);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create pairing_state\n");

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_cycle_counts);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create cycle_counts\n");

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_charge_full_estimate);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create chage_full_estimate\n");

	/* google_resistance and resistance */
	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_resistance_avg);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create resistance_avg\n");

	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_resistance);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create resistance\n");

	/* monitoring */
	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_charger_state);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create charger state\n");
	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_charge_type);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create charge_type\n");
	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_constant_charge_current);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create constant charge current\n");
	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_constant_charge_voltage);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create constant charge voltage\n");
	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_fan_level);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create fan level\n");
	ret = device_create_file(&batt_drv->psy->dev, &dev_attr_health_safety_margin);
	if (ret)
		dev_err(&batt_drv->psy->dev, "Failed to create health safety margin\n");

	de = debugfs_create_dir("google_battery", 0);
	if (IS_ERR_OR_NULL(de))
		return 0;

	debugfs_create_u32("debug_level", 0644, de, &debug_printk_prlog);
	debugfs_create_file("cycle_count_sync", 0600, de, batt_drv,
			    &cycle_count_bins_sync_fops);
	debugfs_create_file("ssoc_gdf", 0600, de, batt_drv, &debug_ssoc_gdf_fops);
	debugfs_create_file("ssoc_uic", 0600, de, batt_drv, &debug_ssoc_uic_fops);
	debugfs_create_file("ssoc_rls", 0400, de, batt_drv, &debug_ssoc_rls_fops);
	debugfs_create_file("ssoc_uicurve", 0600, de, batt_drv,
			    &debug_ssoc_uicurve_cstr_fops);
	debugfs_create_file("force_psy_update", 0400, de, batt_drv,
			    &debug_force_psy_update_fops);
	debugfs_create_file("pairing_state", 0200, de, batt_drv, &debug_pairing_fops);
	debugfs_create_file("blf_state", 0400, de, batt_drv, &debug_blf_state_fops);
	debugfs_create_file("temp", 0400, de, batt_drv, &debug_fake_temp_fops);
	debugfs_create_u32("battery_present", 0600, de,
			   &batt_drv->fake_battery_present);

	/* defender */
	debugfs_create_u32("fake_capacity", 0600, de,
			    &batt_drv->fake_capacity);

	/* health charging */
	debugfs_create_file("chg_health_thr_soc", 0600, de, batt_drv,
			    &debug_chg_health_thr_soc_fops);
	debugfs_create_file("chg_health_rest_rate", 0600, de, batt_drv,
			    &debug_chg_health_rest_rate_fops);
	debugfs_create_file("chg_health_stage", 0600, de, batt_drv,
			    &debug_chg_health_stage_fops);

	return 0;
}

/* ------------------------------------------------------------------------- */

/* could also use battery temperature, age */
static bool gbatt_check_dead_battery(const struct batt_drv *batt_drv)
{
	return ssoc_get_capacity(&batt_drv->ssoc_state) == 0;
}

#define SSOC_LEVEL_FULL		SSOC_SPOOF
#define SSOC_LEVEL_HIGH		80
#define SSOC_LEVEL_NORMAL	30
#define SSOC_LEVEL_LOW		0

/*
 * could also use battery temperature, age.
 * NOTE: this implementation looks at the SOC% but it might be looking to
 * other quantities or flags.
 * NOTE: CRITICAL_LEVEL implies BATTERY_DEAD but BATTERY_DEAD doesn't imply
 * CRITICAL.
 */
static int gbatt_get_capacity_level(struct batt_ssoc_state *ssoc_state,
				    int fg_status)
{
	const int ssoc = ssoc_get_capacity(ssoc_state);
	int capacity_level;

	if (ssoc >= SSOC_LEVEL_FULL) {
		capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	} else if (ssoc > SSOC_LEVEL_HIGH) {
		capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
	} else if (ssoc > SSOC_LEVEL_NORMAL) {
		capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	} else if (ssoc > SSOC_LEVEL_LOW) {
		capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	} else if (ssoc_state->buck_enabled == 0) {
		capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	} else if (ssoc_state->buck_enabled == -1) {
		/* only at startup, this should not happen */
		capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
	} else if (fg_status == POWER_SUPPLY_STATUS_DISCHARGING ||
		   fg_status == POWER_SUPPLY_STATUS_UNKNOWN) {
		capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	} else {
		capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	}

	return capacity_level;
}

static int gbatt_get_temp(const struct batt_drv *batt_drv, int *temp)
{
	int err = 0;
	union power_supply_propval val;

	if (batt_drv->fake_temp) {
		*temp = batt_drv->fake_temp;
	} else if (!batt_drv->fg_psy) {
		err = -EINVAL;
	} else {
		err = power_supply_get_property(batt_drv->fg_psy,
						POWER_SUPPLY_PROP_TEMP, &val);
		if (err == 0)
			*temp = val.intval;
	}

	return err;
}

static void bat_log_ttf_estimate(const char *label, int ssoc,
				 struct batt_drv *batt_drv)
{
	int cc, err;
	ktime_t res = 0;

	err = batt_ttf_estimate(&res, batt_drv);
	if (err < 0) {
		logbuffer_log(batt_drv->ttf_stats.ttf_log,
			      "%s ssoc=%d time=%ld err=%d",
			      (label) ? label : "", ssoc, get_boot_sec(), err);
		return;
	}

	cc = GPSY_GET_PROP(batt_drv->fg_psy, POWER_SUPPLY_PROP_CHARGE_COUNTER);
	logbuffer_log(batt_drv->ttf_stats.ttf_log,
		      "%s ssoc=%d cc=%d time=%ld %d:%d:%d (est=%ld)",
		      (label) ? label : "", ssoc, cc / 1000, get_boot_sec(),
		      res / 3600, (res % 3600) / 60, (res % 3600) % 60,
		      res);
}

static int batt_do_sha256(const u8 *data, unsigned int len, u8 *result)
{
	struct crypto_shash *tfm;
	struct shash_desc *shash;
	int size, ret = 0;

	tfm = crypto_alloc_shash("sha256", 0, 0);
	if (IS_ERR(tfm)) {
		pr_err("Error SHA-256 transform: %ld\n", PTR_ERR(tfm));
		return PTR_ERR(tfm);
	}

	size = sizeof(struct shash_desc) + crypto_shash_descsize(tfm);
	shash = kmalloc(size, GFP_KERNEL);
	if (!shash)
		return -ENOMEM;

	shash->tfm = tfm;
	ret = crypto_shash_digest(shash, data, len, result);
	kfree(shash);
	crypto_free_shash(tfm);

	return ret;
}


/* called with a lock on ->chg_lock */
static enum batt_paired_state
batt_check_pairing_state(struct batt_drv *batt_drv)
{
	const int len = strlen(dev_sn);
	char dev_info[GBMS_DINF_LEN];
	char mfg_info[GBMS_MINF_LEN];
	u8 *dev_info_check = batt_drv->dev_info_check;
	int ret;

	ret = gbms_storage_read(GBMS_TAG_DINF, dev_info, GBMS_DINF_LEN);
	if (ret < 0) {
		pr_err("Read device pairing info failed, ret=%d\n", ret);
		return BATT_PAIRING_READ_ERROR;
	}

	if (batt_drv->dev_info_check[0] == 0) {
		char data[DEV_SN_LENGTH + GBMS_MINF_LEN];

		ret = gbms_storage_read(GBMS_TAG_MINF, mfg_info, GBMS_MINF_LEN);
		if (ret < 0) {
			pr_err("read mfg info. fail, ret=%d\n", ret);
			return BATT_PAIRING_READ_ERROR;
		}

		memcpy(data, dev_sn, len);
		memcpy(&data[len], mfg_info, GBMS_MINF_LEN);

		ret = batt_do_sha256(data, len + GBMS_MINF_LEN, dev_info_check);
		if (ret < 0) {
			pr_err("execute batt_do_sha256 fail, ret=%d\n", ret);
			return BATT_PAIRING_MISMATCH;
		}

		pr_info("dev_info_check: %s\n", dev_info_check);
	}

	/* new battery: pair the battery to this device */
	if (dev_info[0] == 0xFF) {

		ret = gbms_storage_write(GBMS_TAG_DINF, dev_info_check,
					 strlen(dev_info_check));
		if (ret < 0) {
			pr_err("Pairing to this device failed, ret=%d\n", ret);
			return BATT_PAIRING_WRITE_ERROR;
		}

	/* recycled battery */
	} else if (strncmp(dev_info, dev_info_check, strlen(dev_info_check))) {
		pr_warn("dev_sn=%*s\n", (int)strlen(dev_sn), dev_sn);
		pr_warn("dev_info=%*s\n", GBMS_DINF_LEN, dev_info);
		pr_warn("Battery paired to a different device\n");

		return BATT_PAIRING_MISMATCH;
	}

	return BATT_PAIRING_PAIRED;
}

/*  TODO: handle history collection, use storage */
static int batt_hist_data_collect(void *h, int idx)
{
	int cnt;

	cnt = gbms_storage_read(GBMS_TAG_CLHI, h, 0);
	if (cnt > 0)
		cnt = gbms_storage_write_data(GBMS_TAG_HIST, h, cnt, idx);

	return cnt;
}

/* TODO: handle history collection, use storage */
static void batt_hist_free_data(void *p)
{
	if (p)
		kfree(p);
}

/* battery history data collection */
static int batt_history_data_work(struct batt_drv *batt_drv)
{
	int cycle_cnt, idx, ret;

	/* TODO: google_battery caches cycle count, should use that */
	cycle_cnt = GPSY_GET_PROP(batt_drv->fg_psy,
				  POWER_SUPPLY_PROP_CYCLE_COUNT);
	if (cycle_cnt < 0)
		return -EIO;

	if (cycle_cnt <= batt_drv->hist_data_saved_cnt)
		return 0;

	idx = cycle_cnt / batt_drv->hist_delta_cycle_cnt;

	/* check if the cycle_cnt is valid */
	if (idx >= batt_drv->hist_data_max_cnt)
		return -ENOENT;

	ret = batt_hist_data_collect(batt_drv->hist_data, idx);
	if (ret < 0)
		return ret;

	batt_drv->hist_data_saved_cnt = cycle_cnt;
	pr_debug("MSC_HIST Update data with cnt:%d\n", cycle_cnt);


	return 0;
}

/*
 * poll the battery, run SOC%, dead battery, critical.
 * scheduled from psy_changed and from timer
 */

#define UPDATE_INTERVAL_AT_FULL_FACTOR	4

static void google_battery_work(struct work_struct *work)
{
	struct batt_drv *batt_drv =
	    container_of(work, struct batt_drv, batt_work.work);
	struct power_supply *fg_psy = batt_drv->fg_psy;
	struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	int update_interval = batt_drv->batt_update_interval;
	const int prev_ssoc = ssoc_get_capacity(ssoc_state);
	int present, fg_status, batt_temp, ret;
	bool notify_psy_changed = false;

	pr_debug("battery work item\n");

	__pm_stay_awake(batt_drv->batt_ws);

	/* chg_lock protect msc_logic */
	mutex_lock(&batt_drv->chg_lock);

	present = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_PRESENT);
	if (present && !batt_drv->batt_present) {
		pr_debug("%s: change of battery state %d->%d\n",
			 __func__, batt_drv->batt_present, present);

		batt_drv->batt_present = true;
		notify_psy_changed = true;
	} else if (!present && batt_drv->batt_present) {
		pr_debug("%s: change of battery state %d->%d\n",
			 __func__, batt_drv->batt_present, present);

		batt_drv->batt_present = false;

		/* add debounce? */
		notify_psy_changed = true;
		mutex_unlock(&batt_drv->chg_lock);
		goto reschedule;
	}

	fg_status = GPSY_GET_INT_PROP(fg_psy, POWER_SUPPLY_PROP_STATUS, &ret);
	if (ret < 0) {
		mutex_unlock(&batt_drv->chg_lock);
		goto reschedule;
	}

	/* batt_lock protect SSOC code etc. */
	mutex_lock(&batt_drv->batt_lock);

	/* TODO: poll rate should be min between ->batt_update_interval and
	 * whatever ssoc_work() decides (typically rls->rl_delta_max_time)
	 */
	ret = ssoc_work(ssoc_state, fg_psy);
	if (ret < 0) {
		update_interval = BATT_WORK_ERROR_RETRY_MS;
	} else {
		bool full;
		int ssoc, level;

		/* handle charge/recharge */
		batt_rl_update_status(batt_drv);

		ssoc = ssoc_get_capacity(ssoc_state);
		if (prev_ssoc != ssoc) {
			pr_debug("%s: change of ssoc %d->%d\n", __func__,
				 prev_ssoc, ssoc);

			if (ssoc > prev_ssoc)
				bat_log_ttf_estimate("SSOC", ssoc, batt_drv);

			dump_ssoc_state(ssoc_state, batt_drv->ssoc_log);
			notify_psy_changed = true;
		}

		/* TODO(b/138860602): clear ->chg_done to enforce the
		 * same behavior during the transition 99 -> 100 -> Full
		 */

		level = gbatt_get_capacity_level(&batt_drv->ssoc_state,
						 fg_status);
		if (level != batt_drv->capacity_level) {
			pr_debug("%s: change of capacity level %d->%d\n",
				 __func__, batt_drv->capacity_level,
				 level);

			batt_drv->capacity_level = level;
			notify_psy_changed = true;
		}

		if (batt_drv->dead_battery) {
			batt_drv->dead_battery = gbatt_check_dead_battery(batt_drv);
			if (!batt_drv->dead_battery) {
				pr_debug("%s: dead_battery 1->0\n", __func__);
				notify_psy_changed = true;
			}
		}

		/* fuel gauge triggered recharge logic. */
		full = (ssoc == SSOC_FULL);
		if (full && !batt_drv->batt_full) {
			bat_log_ttf_estimate("Full", ssoc, batt_drv);
			batt_chg_stats_pub(batt_drv, "100%", false, true);
		}
		batt_drv->batt_full = full;

		/* debounce fg_status changes at 100% */
		if (fg_status != batt_drv->fg_status) {

			pr_debug("%s: ssoc=%d full=%d change of fg_status %d->%d\n",
				 __func__, ssoc, full, batt_drv->fg_status, fg_status);
			if (!full)
				notify_psy_changed = true;
		}

		/* slow down the updates at full */
		if (full && batt_drv->chg_done)
			update_interval *= UPDATE_INTERVAL_AT_FULL_FACTOR;
	}

	/* notifications for this are debounced  */
	batt_drv->fg_status = fg_status;

	/* TODO: poll other data here if needed */

	ret = gbatt_get_temp(batt_drv, &batt_temp);
	if (ret == 0 && batt_temp != batt_drv->batt_temp) {
		const int limit = batt_drv->batt_update_high_temp_threshold;

		batt_drv->batt_temp = batt_temp;
		if (batt_drv->batt_temp > limit) {
			pr_debug("%s: temperature over limit %d > %d\n",
				 __func__, batt_temp, limit);
			notify_psy_changed = true;
		}
	}

	mutex_unlock(&batt_drv->batt_lock);

	/*
	 * wait for timeout or state equal to CHARGING, FULL or UNKNOWN
	 * (which will likely not happen) even on ssoc error. msc_logic
	 * hold poll_ws wakelock during this time.
	 * Delay the estimates for time to full for BATT_WORK_DEBOUNCE_RETRY_MS
	 * after the device start charging.
	 */
	if (batt_drv->batt_fast_update_cnt) {

		if (fg_status != POWER_SUPPLY_STATUS_DISCHARGING &&
		    fg_status != POWER_SUPPLY_STATUS_NOT_CHARGING) {
			batt_drv->batt_fast_update_cnt = 0;
			update_interval = BATT_WORK_DEBOUNCE_RETRY_MS;
		} else {
			update_interval = BATT_WORK_FAST_RETRY_MS;
			batt_drv->batt_fast_update_cnt -= 1;
		}
	} else if (batt_drv->ttf_debounce) {
		batt_drv->ttf_debounce = 0;
		bat_log_ttf_estimate("Start", prev_ssoc, batt_drv);
	}

	/* acquired in msc_logic */
	if (batt_drv->batt_fast_update_cnt == 0)
		__pm_relax(batt_drv->poll_ws);

	if (batt_drv->res_state.estimate_requested)
		batt_res_work(batt_drv);

	/* check only once and when/if the pairing state is reset */
	if (batt_drv->pairing_state == BATT_PAIRING_ENABLED) {
		enum batt_paired_state state;

		state = batt_check_pairing_state(batt_drv);
		switch (state) {
		/* somethig is wrong with eeprom comms, HW problem? */
		case BATT_PAIRING_READ_ERROR:
			break;
		/* somethig is wrong with eeprom, HW problem? */
		case BATT_PAIRING_WRITE_ERROR:
			break;
		default:
			batt_drv->pairing_state = state;
			break;
		}
	}

	mutex_unlock(&batt_drv->chg_lock);

	/* TODO: we might not need to do this all the time */
	batt_cycle_count_update(batt_drv, ssoc_get_real(ssoc_state));

reschedule:

	if (notify_psy_changed)
		power_supply_changed(batt_drv->psy);

	if (batt_drv->blf_state == BATT_LFCOLLECT_COLLECT) {
		ret = batt_history_data_work(batt_drv);
		if (ret == -ENOENT) {
			batt_drv->blf_state = BATT_LFCOLLECT_NOT_AVAILABLE;
			pr_info("MSC_HIST Battery data collection disabled\n");
		}  else if (ret < 0) {
			pr_debug("MSC_HIST cannot collect battery data %d\n", ret);
		}
	}

	if (update_interval) {
		pr_debug("rerun battery work in %d ms\n", update_interval);
		schedule_delayed_work(&batt_drv->batt_work,
				      msecs_to_jiffies(update_interval));
	}

	__pm_relax(batt_drv->batt_ws);
}

/* ------------------------------------------------------------------------- */

/*
 * Keep the number of properties under UEVENT_NUM_ENVP (minus # of
 * standard uevent variables) i.e 26.
 *
 * Removed the following from sysnodes
 * GBMS_PROP_ADAPTER_DETAILS			gbms
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT	gbms
 * POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE	gbms
 *
 * POWER_SUPPLY_PROP_CHARGE_TYPE,
 * POWER_SUPPLY_PROP_CURRENT_AVG,
 * POWER_SUPPLY_PROP_VOLTAGE_AVG,
 * POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
 * POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
 * POWER_SUPPLY_PROP_VOLTAGE_OCV,
 */

static enum power_supply_property gbatt_battery_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,	/* No need for this? */
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,		/* 23 */
	POWER_SUPPLY_PROP_VOLTAGE_OCV,

	/*  hard limit to 26 */
};

/*
 * status is:
 * . _UNKNOWN during init
 * . _DISCHARGING when not connected
 * when connected to a power supply status is
 * . _FULL (until disconnect) after the charger flags DONE if SSOC=100%
 * . _CHARGING if FG reports _FULL but SSOC < 100% (should not happen)
 * . _CHARGING if FG reports _NOT_CHARGING
 * . _NOT_CHARGING if FG report _DISCHARGING
 * . same as FG state otherwise
 */
static int gbatt_get_status(struct batt_drv *batt_drv,
			    union power_supply_propval *val)
{
	int err, ssoc;

	if (batt_drv->ssoc_state.buck_enabled == 0) {
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	if (batt_drv->ssoc_state.buck_enabled == -1) {
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		return 0;
	}

	/* ->buck_enabled = 1, from here ownward device is connected */

	if (batt_drv->batt_health == POWER_SUPPLY_HEALTH_OVERHEAT) {
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	}

	if (batt_drv->msc_state == MSC_HEALTH_PAUSE) {
		/* Expect AC to discharge in PAUSE. However, UI must persist */
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		return 0;
	}

	if (!batt_drv->fg_psy)
		return -EINVAL;

	ssoc = ssoc_get_capacity(&batt_drv->ssoc_state);

	/* FULL when the charger said so and SSOC == 100% */
	if (batt_drv->chg_done && ssoc == SSOC_FULL) {
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	err = power_supply_get_property(batt_drv->fg_psy,
					POWER_SUPPLY_PROP_STATUS,
					val);
	if (err != 0)
		return err;

	if (val->intval == POWER_SUPPLY_STATUS_FULL) {

		/* not full unless the charger says so */
		if (!batt_drv->chg_done)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;

		/* NOTE: FG driver could flag FULL before GDF is at 100% when
		 * gauge is not tuned or when capacity estimates are wrong.
		 */
		if (ssoc != SSOC_FULL)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;

	} else if (val->intval == POWER_SUPPLY_STATUS_NOT_CHARGING) {
		/* smooth transition between charging and full */
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
	} else if (val->intval == POWER_SUPPLY_STATUS_DISCHARGING) {
		/* connected and discharging is NOT charging */
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	return 0;
}

/* lock batt_drv->batt_lock */
static int gbatt_get_capacity(struct batt_drv *batt_drv)
{
	struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
	int capacity;

	if (batt_drv->fake_capacity >= 0 && batt_drv->fake_capacity <= 100)
		capacity = batt_drv->fake_capacity;
	else
		capacity = ssoc_get_capacity(ssoc_state);

	return capacity;
}

/* splice the curve at point when the SSOC is removed */
static void gbatt_set_capacity(struct batt_drv *batt_drv, int capacity)
{
	if (capacity < 0)
		capacity = -EINVAL;

	if (batt_drv->batt_health != POWER_SUPPLY_HEALTH_OVERHEAT) {
		/* just set the value if not in overheat  */
	} else if (capacity < 0 && batt_drv->fake_capacity >= 0) {
		struct batt_ssoc_state *ssoc_state = &batt_drv->ssoc_state;
		const qnum_t cap = qnum_fromint(batt_drv->fake_capacity);
		const qnum_t gdf = ssoc_state->ssoc_gdf;
		enum ssoc_uic_type type;

		if (gdf < qnum_fromint(batt_drv->fake_capacity)) {
			type = SSOC_UIC_TYPE_DSG;
		} else {
			type = SSOC_UIC_TYPE_CHG;
		}

		pr_info("reset curve at gdf=%d.%d cap=%d.%d type=%d\n",
			qnum_toint(gdf), qnum_fracdgt(gdf),
			qnum_toint(cap), qnum_fracdgt(cap),
			type);

		/* current is the drop point on the discharge curve */
		ssoc_change_curve_at_gdf(ssoc_state, gdf, cap, type);
		ssoc_work(ssoc_state, batt_drv->fg_psy);
		dump_ssoc_state(ssoc_state, batt_drv->ssoc_log);
	} else if (capacity > 0) {
		/* TODO: convergence to the new capacity? */
	}

	batt_drv->fake_capacity = capacity;
}

static int gbatt_set_health(struct batt_drv *batt_drv, int health)
{
	if (health > POWER_SUPPLY_HEALTH_HOT ||
	    health < POWER_SUPPLY_HEALTH_UNKNOWN)
		return -EINVAL;

	batt_drv->batt_health = health;

	/* disable health charging if in overheat */
	if (health == POWER_SUPPLY_HEALTH_OVERHEAT)
		msc_logic_health(batt_drv);

	return 0;
}

static int gbatt_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)
					power_supply_get_drvdata(psy);
	int rc, err = 0;

	pm_runtime_get_sync(batt_drv->device);
	if (!batt_drv->init_complete || !batt_drv->resume_complete) {
		pm_runtime_put_sync(batt_drv->device);
		return -EAGAIN;
	}
	pm_runtime_put_sync(batt_drv->device);

	switch (psp) {
	case GBMS_PROP_ADAPTER_DETAILS:
		val->intval = batt_drv->ce_data.adapter_details.v;
		break;

	case GBMS_PROP_DEAD_BATTERY:
		val->intval = batt_drv->dead_battery;
		break;
	/*
	 * ng charging:
	 * 1) write to GBMS_PROP_CHARGE_CHARGER_STATE,
	 * 2) read POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT and
	 *    POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE
	 */
	case GBMS_PROP_CHARGE_CHARGER_STATE:
		val->intval = batt_drv->chg_state.v;
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		if (batt_drv->cycle_count < 0)
			err = batt_drv->cycle_count;
		else
			val->intval = batt_drv->cycle_count;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		mutex_lock(&batt_drv->batt_lock);
		val->intval = gbatt_get_capacity(batt_drv);
		mutex_unlock(&batt_drv->batt_lock);
		break;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		if (batt_drv->fake_capacity >= 0 &&
				batt_drv->fake_capacity <= 100)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		else
			val->intval = batt_drv->capacity_level;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		mutex_lock(&batt_drv->chg_lock);
		val->intval = batt_drv->cc_max;
		mutex_unlock(&batt_drv->chg_lock);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		mutex_lock(&batt_drv->chg_lock);
		val->intval = batt_drv->fv_uv;
		mutex_unlock(&batt_drv->chg_lock);
		break;

	/*
	 * POWER_SUPPLY_PROP_CHARGE_DONE comes from the charger BUT battery
	 * has also an idea about it.
	 *	mutex_lock(&batt_drv->chg_lock);
	 *	val->intval = batt_drv->chg_done;
	 *	mutex_unlock(&batt_drv->chg_lock);
	 */

	/*
	 * compat: POWER_SUPPLY_PROP_CHARGE_TYPE comes from the charger so
	 * using the last value reported from the CHARGER. This (of course)
	 * means that NG charging needs to be enabled.
	 */
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		mutex_lock(&batt_drv->chg_lock);
		val->intval = batt_drv->chg_state.f.chg_type;
		mutex_unlock(&batt_drv->chg_lock);
		break;

	case POWER_SUPPLY_PROP_STATUS:
		err = gbatt_get_status(batt_drv, val);
		break;

	/* health */
	case POWER_SUPPLY_PROP_HEALTH:
		if (batt_drv->batt_health == POWER_SUPPLY_HEALTH_OVERHEAT &&
		    gbms_temp_defend_dry_run(false, false)) {
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		} else if (batt_drv->batt_health !=
			   POWER_SUPPLY_HEALTH_UNKNOWN) {
			val->intval = batt_drv->batt_health;
		} else if (!batt_drv->fg_psy) {
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		} else {
			rc = power_supply_get_property(batt_drv->fg_psy,
							psp, val);
			if (rc < 0)
				val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
			batt_drv->soh = val->intval;
		}
		break;

	/* cannot set err, negative estimate will revert to HAL */
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW: {
		ktime_t res;

		rc = batt_ttf_estimate(&res, batt_drv);
		if (rc == 0) {
			if (res < 0)
				res = 0;
			val->intval = res;
		} else if (!batt_drv->fg_psy) {
			val->intval = -1;
		} else {
			rc = power_supply_get_property(batt_drv->fg_psy,
							psp, val);
			if (rc < 0)
				val->intval = -1;
		}
	} break;

	case POWER_SUPPLY_PROP_TEMP:
		err = gbatt_get_temp(batt_drv, &val->intval);
		break;

	case POWER_SUPPLY_PROP_CURRENT_AVG:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (!batt_drv->fg_psy)
			return -EINVAL;
		err = power_supply_get_property(batt_drv->fg_psy, psp, val);
		if (err == 0)
			val->intval = -val->intval;
		break;

	/* Can force the state here */
	case POWER_SUPPLY_PROP_PRESENT:
		if (batt_drv->fake_battery_present != -1) {
			val->intval = batt_drv->fake_battery_present;
		} else if (batt_drv->fg_psy) {

			/* TODO: use the cached value? */
			rc = power_supply_get_property(batt_drv->fg_psy,
						       psp, val);
			if (rc < 0)
				val->intval = 0;
		} else {
			err = -EINVAL;
		}
		break;

	default:
		if (!batt_drv->fg_psy)
			return -EINVAL;
		err = power_supply_get_property(batt_drv->fg_psy, psp, val);
		break;
	}

	if (err < 0) {
		pr_debug("gbatt: get_prop cannot read psp=%d\n", psp);
		return err;
	}

	return 0;
}

static int gbatt_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *val)
{
	struct batt_drv *batt_drv = (struct batt_drv *)
					power_supply_get_drvdata(psy);
	int ret = 0;

	pm_runtime_get_sync(batt_drv->device);
	if (!batt_drv->init_complete || !batt_drv->resume_complete) {
		pm_runtime_put_sync(batt_drv->device);
		return -EAGAIN;
	}
	pm_runtime_put_sync(batt_drv->device);

	switch (psp) {
	case GBMS_PROP_ADAPTER_DETAILS:
		mutex_lock(&batt_drv->stats_lock);
		batt_drv->ce_data.adapter_details.v = val->intval;
		mutex_unlock(&batt_drv->stats_lock);
	break;

	/* NG Charging, where it all begins */
	case GBMS_PROP_CHARGE_CHARGER_STATE:
		mutex_lock(&batt_drv->chg_lock);
		batt_drv->chg_state.v = gbms_propval_int64val(val);
		ret = batt_chg_logic(batt_drv);
		mutex_unlock(&batt_drv->chg_lock);
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		mutex_lock(&batt_drv->chg_lock);
		if (val->intval != batt_drv->fake_capacity) {
			gbatt_set_capacity(batt_drv, val->intval);
			if (batt_drv->psy)
				power_supply_changed(batt_drv->psy);
		}
		mutex_unlock(&batt_drv->chg_lock);
		break;

	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		if (val->intval <= 0)
			batt_drv->ttf_stats.ttf_fake = -1;
		else
			batt_drv->ttf_stats.ttf_fake = val->intval;
		pr_info("time_to_full = %lld\n", batt_drv->ttf_stats.ttf_fake);
		if (batt_drv->psy)
			power_supply_changed(batt_drv->psy);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		mutex_lock(&batt_drv->chg_lock);
		if (batt_drv->batt_health != val->intval) {
			ret = gbatt_set_health(batt_drv, val->intval);
			if (ret == 0 && batt_drv->psy)
				power_supply_changed(batt_drv->psy);
		}
		mutex_unlock(&batt_drv->chg_lock);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret < 0) {
		pr_debug("gbatt: get_prop cannot write psp=%d\n", psp);
		return ret;
	}


	return 0;
}

static int gbatt_property_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	switch (psp) {
	case GBMS_PROP_CHARGE_CHARGER_STATE:
	case POWER_SUPPLY_PROP_CAPACITY:
	case GBMS_PROP_ADAPTER_DETAILS:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
	case POWER_SUPPLY_PROP_HEALTH:
		return 1;
	default:
		break;
	}

	return 0;
}

static struct power_supply_desc gbatt_psy_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.get_property = gbatt_get_property,
	.set_property = gbatt_set_property,
	.property_is_writeable = gbatt_property_is_writeable,
	.properties = gbatt_battery_props,
	.num_properties = ARRAY_SIZE(gbatt_battery_props),
};

/* ------------------------------------------------------------------------ */
#define BATT_ONE_HIST_LEN	12
static void google_battery_init_hist_work(struct work_struct *work)
{
	struct batt_drv *batt_drv = container_of(work, struct batt_drv,
						 init_hist_work.work);
	int cnt;

	/* gbms_storage will return -EPROBE_DEFER during init */
	cnt = gbms_storage_read_data(GBMS_TAG_HIST, NULL, 0, 0);
	if (cnt == -EPROBE_DEFER) {
		/* wait until storage is up */
		schedule_delayed_work(&batt_drv->init_hist_work,
				      msecs_to_jiffies(BATT_DELAY_INIT_MS));
		return;
	}

	if (cnt <= 0) {
		batt_drv->blf_state = BATT_LFCOLLECT_NOT_AVAILABLE;
		pr_err("MSC_HIST collect history data not available (%d)\n", cnt);
		return;
	}

	batt_drv->hist_data = kzalloc(BATT_ONE_HIST_LEN, GFP_KERNEL);
	if (!batt_drv->hist_data) {
		batt_drv->blf_state = BATT_LFCOLLECT_DISABLED;
	} else {
		batt_drv->blf_state = BATT_LFCOLLECT_COLLECT;
		batt_drv->hist_data_max_cnt = cnt;
		batt_drv->hist_data_saved_cnt = -1;
	}

	pr_info("MSC_HIST init_hist_work done, state:%d, cnt:%d",
		 batt_drv->blf_state, cnt);
}

static void google_battery_init_work(struct work_struct *work)
{
	struct batt_drv *batt_drv = container_of(work, struct batt_drv,
						 init_work.work);
	struct device_node *node = batt_drv->device->of_node;
	struct power_supply *fg_psy = batt_drv->fg_psy;
	int ret = 0;

	batt_rl_reset(batt_drv);
	batt_drv->dead_battery = true; /* clear in batt_work() */
	batt_drv->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
	batt_drv->ssoc_state.buck_enabled = -1;
	batt_drv->hold_taper_ws = false;
	batt_drv->fake_temp = 0;
	batt_drv->fake_battery_present = -1;
	batt_reset_chg_drv_state(batt_drv);

	mutex_init(&batt_drv->chg_lock);
	mutex_init(&batt_drv->batt_lock);
	mutex_init(&batt_drv->stats_lock);
	mutex_init(&batt_drv->cc_data.lock);

	if (!batt_drv->fg_psy) {

		fg_psy = power_supply_get_by_name(batt_drv->fg_psy_name);
		if (!fg_psy) {
			pr_info("failed to get \"%s\" power supply, retrying...\n",
				batt_drv->fg_psy_name);
			goto retry_init_work;
		}

		batt_drv->fg_psy = fg_psy;
	}

	if (!batt_drv->batt_present) {
		ret = GPSY_GET_PROP(fg_psy, POWER_SUPPLY_PROP_PRESENT);
		if (ret == -EAGAIN)
			goto retry_init_work;

		batt_drv->batt_present = (ret > 0);
		if (!batt_drv->batt_present)
			pr_warn("battery not present (ret=%d)\n", ret);
	}

	ret = of_property_read_u32(node, "google,recharge-soc-threshold",
				   &batt_drv->ssoc_state.rl_soc_threshold);
	if (ret < 0)
		batt_drv->ssoc_state.rl_soc_threshold =
				DEFAULT_BATT_DRV_RL_SOC_THRESHOLD;

	ret = of_property_read_u32(node, "google,bd-trickle-recharge-soc",
				   &batt_drv->ssoc_state.bd_trickle_recharge_soc);
	if (ret < 0)
		batt_drv->ssoc_state.bd_trickle_recharge_soc =
				DEFAULT_BD_TRICKLE_RL_SOC_THRESHOLD;

	batt_drv->ssoc_state.bd_trickle_dry_run = false;

	ret = of_property_read_u32(node, "google,bd-trickle-reset-sec",
				   &batt_drv->ssoc_state.bd_trickle_reset_sec);
	if (ret < 0)
		batt_drv->ssoc_state.bd_trickle_reset_sec =
				DEFAULT_BD_TRICKLE_RESET_SEC;

	batt_drv->ssoc_state.bd_trickle_enable =
		of_property_read_bool(node, "google,bd-trickle-enable");

	ret = of_property_read_u32(node, "google,ssoc-delta",
				   &batt_drv->ssoc_state.ssoc_delta);
	if (ret < 0)
		batt_drv->ssoc_state.ssoc_delta = SSOC_DELTA;

	ret = of_property_read_u32(node, "google,health-safety-margin",
				   &batt_drv->health_safety_margin);
	if (ret < 0)
		batt_drv->health_safety_margin = DEFAULT_HEALTH_SAFETY_MARGIN;

	/* cycle count is cached: read here bc SSOC, chg_profile might use it */
	batt_update_cycle_count(batt_drv);

	ret = ssoc_init(&batt_drv->ssoc_state, node, fg_psy);
	if (ret < 0 && batt_drv->batt_present)
		goto retry_init_work;

	dump_ssoc_state(&batt_drv->ssoc_state, batt_drv->ssoc_log);

	/* could read EEPROM and history here */

	ret = batt_init_chg_profile(batt_drv);
	if (ret == -EPROBE_DEFER)
		goto retry_init_work;

	if (ret < 0) {
		pr_err("charging profile disabled, ret=%d\n", ret);
	} else if (batt_drv->battery_capacity) {
		gbms_dump_chg_profile(&batt_drv->chg_profile);
	}

	cev_stats_init(&batt_drv->ce_data, &batt_drv->chg_profile);
	cev_stats_init(&batt_drv->ce_qual, &batt_drv->chg_profile);

	batt_drv->fg_nb.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&batt_drv->fg_nb);
	if (ret < 0)
		pr_err("cannot register power supply notifer, ret=%d\n",
			ret);

	batt_drv->batt_ws = wakeup_source_register(NULL, gbatt_psy_desc.name);
	batt_drv->taper_ws = wakeup_source_register(NULL, "Taper");
	batt_drv->poll_ws = wakeup_source_register(NULL, "Poll");
	batt_drv->msc_ws = wakeup_source_register(NULL, "MSC");
	if (!batt_drv->batt_ws || !batt_drv->taper_ws ||
			!batt_drv->poll_ws || !batt_drv->msc_ws)
		pr_err("failed to register wakeup sources\n");

	mutex_lock(&batt_drv->cc_data.lock);
	ret = batt_cycle_count_load(&batt_drv->cc_data);
	if (ret < 0)
		pr_err("cannot restore bin count ret=%d\n", ret);
	mutex_unlock(&batt_drv->cc_data.lock);

	batt_drv->fake_capacity = (batt_drv->batt_present) ? -EINVAL
						: DEFAULT_BATT_FAKE_CAPACITY;

	/* charging configuration */
	ret = of_property_read_u32(node, "google,update-interval",
				   &batt_drv->batt_update_interval);
	if (ret < 0)
		batt_drv->batt_update_interval = DEFAULT_BATT_UPDATE_INTERVAL;

	/* high temperature notify configuration */
	ret = of_property_read_u32(batt_drv->device->of_node,
				   "google,update-high-temp-threshold",
				   &batt_drv->batt_update_high_temp_threshold);
	if (ret < 0)
		batt_drv->batt_update_high_temp_threshold =
					DEFAULT_HIGH_TEMP_UPDATE_THRESHOLD;
	/* charge statistics */
	ret = of_property_read_u32(node, "google,chg-stats-qual-time",
				   &batt_drv->chg_sts_qual_time);
	if (ret < 0)
		batt_drv->chg_sts_qual_time =
					DEFAULT_CHG_STATS_MIN_QUAL_TIME;

	ret = of_property_read_u32(node, "google,chg-stats-delta-soc",
				   &batt_drv->chg_sts_delta_soc);
	if (ret < 0)
		batt_drv->chg_sts_delta_soc =
					DEFAULT_CHG_STATS_MIN_DELTA_SOC;

	/* time to full */
	ret = ttf_stats_init(&batt_drv->ttf_stats, batt_drv->device,
			     batt_drv->battery_capacity);
	if (ret < 0) {
		pr_info("time to full not available\n");
	} else {
		batt_drv->ttf_stats.ttf_log = logbuffer_register("ttf");
		if (IS_ERR(batt_drv->ttf_stats.ttf_log)) {
			ret = PTR_ERR(batt_drv->ttf_stats.ttf_log);
			dev_err(batt_drv->device,
				"failed to create ttf_log, ret=%d\n", ret);

			batt_drv->ttf_stats.ttf_log = NULL;
		}
	}

	/* google_resistance  */
	batt_res_load_data(&batt_drv->res_state, batt_drv->fg_psy);

	/* health based charging, triggers */
	batt_drv->chg_health.always_on_soc = -1;

	ret = of_property_read_u32(batt_drv->device->of_node,
				   "google,chg-rest-soc",
				   &batt_drv->chg_health.rest_soc);
	if (ret < 0)
		batt_drv->chg_health.rest_soc = -1;

	ret = of_property_read_u32(batt_drv->device->of_node,
				   "google,chg-rest-rate",
				   &batt_drv->chg_health.rest_rate);
	if (ret < 0)
		batt_drv->chg_health.rest_rate = 0;

	/* override setting google,battery-roundtrip = 0 in device tree */
	batt_drv->disable_votes =
		of_property_read_bool(node, "google,disable-votes");
	if (batt_drv->disable_votes)
		pr_info("battery votes disabled\n");

	/* pairing battery vs. device */
	if (of_property_read_bool(node, "google,eeprom-pairing"))
		batt_drv->pairing_state = BATT_PAIRING_ENABLED;
	else
		batt_drv->pairing_state = BATT_PAIRING_DISABLED;

	/* use delta cycle count to adjust collecting period */
	ret = of_property_read_u32(batt_drv->device->of_node,
					"google,history-delta-cycle-count",
					&batt_drv->hist_delta_cycle_cnt);
	if (ret < 0)
		batt_drv->hist_delta_cycle_cnt = HCC_DEFAULT_DELTA_CYCLE_CNT;

	/* use delta cycle count to enable collecting history */
	if (batt_drv->hist_delta_cycle_cnt)
		batt_drv->blf_state = BATT_LFCOLLECT_ENABLED;

	if (batt_drv->blf_state == BATT_LFCOLLECT_ENABLED)
		schedule_delayed_work(&batt_drv->init_hist_work,
				      msecs_to_jiffies(HCC_DELAY_INIT_MS));

	/* google_battery expose history via a standard device */
	batt_drv->history = gbms_storage_create_device("battery_history",
						       GBMS_TAG_HIST);
	if (!batt_drv->history)
		pr_err("history not available\n");

	/* debugfs */
	(void)batt_init_fs(batt_drv);

	pr_info("google_battery init_work done\n");

	batt_drv->init_complete = true;
	batt_drv->resume_complete = true;

	schedule_delayed_work(&batt_drv->batt_work, 0);

	return;

retry_init_work:
	schedule_delayed_work(&batt_drv->init_work,
			      msecs_to_jiffies(BATT_DELAY_INIT_MS));
}

static struct thermal_zone_of_device_ops google_battery_tz_ops = {
	.get_temp = google_battery_tz_get_cycle_count,
};

static int google_battery_probe(struct platform_device *pdev)
{
	const char *fg_psy_name, *psy_name = NULL;
	struct batt_drv *batt_drv;
	int ret;
	struct power_supply_config psy_cfg = {};

	batt_drv = devm_kzalloc(&pdev->dev, sizeof(*batt_drv), GFP_KERNEL);
	if (!batt_drv)
		return -ENOMEM;

	batt_drv->device = &pdev->dev;

	ret = of_property_read_string(pdev->dev.of_node, "google,fg-psy-name",
				      &fg_psy_name);
	if (ret != 0) {
		pr_err("cannot read google,fg-psy-name, ret=%d\n", ret);
		return -EINVAL;
	}

	batt_drv->fg_psy_name = devm_kstrdup(&pdev->dev, fg_psy_name,
					     GFP_KERNEL);
	if (!batt_drv->fg_psy_name)
		return -ENOMEM;

	/* change name and type for debug/test */
	if (of_property_read_bool(pdev->dev.of_node, "google,psy-type-unknown"))
		gbatt_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;

	ret = of_property_read_string(pdev->dev.of_node,
				      "google,psy-name", &psy_name);
	if (ret == 0) {
		gbatt_psy_desc.name =
		    devm_kstrdup(&pdev->dev, psy_name, GFP_KERNEL);
	}

	INIT_DELAYED_WORK(&batt_drv->init_work, google_battery_init_work);
	INIT_DELAYED_WORK(&batt_drv->batt_work, google_battery_work);
	INIT_DELAYED_WORK(&batt_drv->init_hist_work, google_battery_init_hist_work);
	platform_set_drvdata(pdev, batt_drv);

	psy_cfg.drv_data = batt_drv;
	psy_cfg.of_node = pdev->dev.of_node;

	batt_drv->psy = devm_power_supply_register(batt_drv->device,
						   &gbatt_psy_desc, &psy_cfg);
	if (IS_ERR(batt_drv->psy)) {
		ret = PTR_ERR(batt_drv->psy);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		/* TODO: fail with -ENODEV */
		dev_err(batt_drv->device,
			"Couldn't register as power supply, ret=%d\n", ret);
	}

	batt_drv->ssoc_log = logbuffer_register("ssoc");
	if (IS_ERR(batt_drv->ssoc_log)) {
		ret = PTR_ERR(batt_drv->ssoc_log);
		dev_err(batt_drv->device,
			"failed to create ssoc_log, ret=%d\n", ret);
		batt_drv->ssoc_log = NULL;
	}

	/* Resistance Estimation configuration */
	ret = of_property_read_u32(pdev->dev.of_node, "google,res-temp-hi",
				   &batt_drv->res_state.res_temp_high);
	if (ret < 0)
		batt_drv->res_state.res_temp_high = DEFAULT_RES_TEMP_HIGH;

	ret = of_property_read_u32(pdev->dev.of_node, "google,res-temp-lo",
				   &batt_drv->res_state.res_temp_low);
	if (ret < 0)
		batt_drv->res_state.res_temp_low = DEFAULT_RES_TEMP_LOW;

	ret = of_property_read_u32(pdev->dev.of_node, "google,res-soc-thresh",
				   &batt_drv->res_state.ssoc_threshold);
	if (ret < 0)
		batt_drv->res_state.ssoc_threshold = DEFAULT_RES_SSOC_THR;

	ret = of_property_read_u32(pdev->dev.of_node, "google,res-filt-length",
				   &batt_drv->res_state.estimate_filter);
	if (ret < 0)
		batt_drv->res_state.estimate_filter = DEFAULT_RES_FILT_LEN;

	batt_drv->tz_dev = thermal_zone_of_sensor_register(batt_drv->device,
				0, batt_drv, &google_battery_tz_ops);
	if (IS_ERR(batt_drv->tz_dev)) {
		pr_err("battery tz register failed. err:%ld\n",
			PTR_ERR(batt_drv->tz_dev));
		ret = PTR_ERR(batt_drv->tz_dev);
		batt_drv->tz_dev = NULL;
	} else {
		thermal_zone_device_update(batt_drv->tz_dev, THERMAL_DEVICE_UP);
	}

	batt_drv->fan_level = -1;
	batt_drv->fan_last_level = -1;
	batt_drv->fan_level_votable =
		create_votable(VOTABLE_FAN_LEVEL, VOTE_MAX, fan_level_cb, batt_drv);
	if (IS_ERR(batt_drv->fan_level_votable)) {
		ret = PTR_ERR(batt_drv->fan_level_votable);
		dev_err(batt_drv->device, "Fail to create fan_level_votable\n");
		batt_drv->fan_level_votable = NULL;
	} else {
		vote(batt_drv->fan_level_votable, "DEFAULT", true, FAN_LVL_UNKNOWN);
	}

	/* give time to fg driver to start */
	schedule_delayed_work(&batt_drv->init_work,
					msecs_to_jiffies(BATT_DELAY_INIT_MS));

	return 0;
}

static int google_battery_remove(struct platform_device *pdev)
{
	struct batt_drv *batt_drv = platform_get_drvdata(pdev);

	if (!batt_drv)
		return 0;

	if (batt_drv->ssoc_log)
		logbuffer_unregister(batt_drv->ssoc_log);
	if (batt_drv->ttf_stats.ttf_log)
		logbuffer_unregister(batt_drv->ttf_stats.ttf_log);
	if (batt_drv->tz_dev)
		thermal_zone_of_sensor_unregister(batt_drv->device,
				batt_drv->tz_dev);
	if (batt_drv->history)
		gbms_storage_cleanup_device(batt_drv->history);

	if (batt_drv->fg_psy)
		power_supply_put(batt_drv->fg_psy);

	batt_hist_free_data(batt_drv->hist_data);

	gbms_free_chg_profile(&batt_drv->chg_profile);

	wakeup_source_unregister(batt_drv->msc_ws);
	wakeup_source_unregister(batt_drv->batt_ws);
	wakeup_source_unregister(batt_drv->taper_ws);
	wakeup_source_unregister(batt_drv->poll_ws);

	destroy_votable(batt_drv->fan_level_votable);
	batt_drv->fan_level_votable = NULL;

	return 0;
}

#ifdef SUPPORT_PM_SLEEP
static int gbatt_pm_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct batt_drv *batt_drv = platform_get_drvdata(pdev);

	pm_runtime_get_sync(batt_drv->device);
	batt_drv->resume_complete = false;
	pm_runtime_put_sync(batt_drv->device);

	return 0;
}

static int gbatt_pm_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct batt_drv *batt_drv = platform_get_drvdata(pdev);

	pm_runtime_get_sync(batt_drv->device);
	batt_drv->resume_complete = true;
	pm_runtime_put_sync(batt_drv->device);

	mod_delayed_work(system_wq, &batt_drv->batt_work, 0);

	return 0;
}

static const struct dev_pm_ops gbatt_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(gbatt_pm_suspend, gbatt_pm_resume)
};
#endif


static const struct of_device_id google_charger_of_match[] = {
	{.compatible = "google,battery"},
	{},
};
MODULE_DEVICE_TABLE(of, google_charger_of_match);


static struct platform_driver google_battery_driver = {
	.driver = {
		   .name = "google,battery",
		   .owner = THIS_MODULE,
		   .of_match_table = google_charger_of_match,
		   .probe_type = PROBE_PREFER_ASYNCHRONOUS,
#ifdef SUPPORT_PM_SLEEP
		   .pm = &gbatt_pm_ops,
#endif
		   },
	.probe = google_battery_probe,
	.remove = google_battery_remove,
};

module_platform_driver(google_battery_driver);

MODULE_DESCRIPTION("Google Battery Driver");
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_LICENSE("GPL");
