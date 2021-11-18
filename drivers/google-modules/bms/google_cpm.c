/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2020 Google, LLC
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
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/slab.h>
#include <misc/gvotable.h>
#include "gbms_power_supply.h"
#include "google_bms.h"
#include "google_dc_pps.h"
#include "google_psy.h"
#include <misc/logbuffer.h>

#include <linux/debugfs.h>

#define get_boot_sec() div_u64(ktime_to_ns(ktime_get_boottime()), NSEC_PER_SEC)


/* Non DC Charger is the default */
#define GCPM_DEFAULT_CHARGER	0
/* TODO: handle capabilities based on index number */
#define GCPM_INDEX_DC_DISABLE	-1
#define GCPM_INDEX_DC_ENABLE	1
#define GCPM_MAX_CHARGERS	4

/* tier based, disabled now */
#define GCPM_DEFAULT_DC_LIMIT_DEMAND	0
/* thermal will change this */
#define GCPM_DEFAULT_DC_LIMIT_CC_MIN		1000000
#define GCPM_DEFAULT_DC_LIMIT_CC_MIN_WLC	2000000

/* voltage based */
#define GCPM_DEFAULT_DC_LIMIT_VBATT_MIN		3600000
#define GCPM_DEFAULT_DC_LIMIT_DELTA_LOW		200000

/* demand based limits */
#define GCPM_DEFAULT_DC_LIMIT_VBATT_MAX		4450000
#define GCPM_DEFAULT_DC_LIMIT_DELTA_HIGH	200000

/* behavior in taper */
#define GCPM_TAPER_STEP_FV_MARGIN	0
#define GCPM_TAPER_STEP_CC_STEP		0
#define GCPM_TAPER_STEP_COUNT		0
#define GCPM_TAPER_STEP_GRACE		1
#define GCPM_TAPER_STEP_VOLTAGE		0
#define GCPM_TAPER_STEP_CURRENT		0
/* enough time for the charger to settle to a new limit */
#define GCPM_TAPER_STEP_INTERVAL_S	120

/* TODO: move to configuration */
#define DC_TA_VMAX_MV		9800000
/* TODO: move to configuration */
#define DC_TA_VMIN_MV		8000000
/* TODO: move to configuration */
#define DC_VBATT_HEADROOM_MV	500000

enum gcpm_dc_state_t {
	DC_DISABLED = -1,
	DC_IDLE = 0,
	DC_ENABLE,
	DC_RUNNING,
	DC_ENABLE_PASSTHROUGH,
	DC_PASSTHROUGH,
};

/* DC_ERROR_RETRY_MS <= DC_RUN_DELAY_MS */
#define DC_ENABLE_DELAY_MS	500
#define DC_RUN_DELAY_MS		9000
#define DC_ERROR_RETRY_MS	PPS_ERROR_RETRY_MS

#define PPS_PROG_TIMEOUT_S	10
#define PPS_PROG_RETRY_MS	2000
#define PPS_ACTIVE_RETRY_MS	1500
#define PPS_ACTIVE_USB_TIMEOUT_S	25
#define PPS_ACTIVE_WLC_TIMEOUT_S	500
#define PPS_READY_DELTA_TIMEOUT_S	10

#define PPS_ERROR_RETRY_MS	1000

enum {
	PPS_INDEX_NOT_SUPP = -1,
	PPS_INDEX_TCPM = 1,
	PPS_INDEX_WLC = 2,
	PPS_INDEX_MAX = 2,
};

struct gcpm_drv  {
	struct device *device;
	struct power_supply *psy;
	struct delayed_work init_work;

	struct gvotable_election *fcc_votable;
	/* charge limit for wireless DC */
	struct gvotable_election *dc_fcc_votable;
	int dc_fcc_limit;
	bool dc_fcc_hold;

	/* combine PPS, route to the active PPS source */
	struct power_supply *pps_psy;

	int chg_psy_retries;
	struct power_supply *chg_psy_avail[GCPM_MAX_CHARGERS];
	const char *chg_psy_names[GCPM_MAX_CHARGERS];
	struct mutex chg_psy_lock;
	int chg_psy_active;
	int chg_psy_count;

	/* force a charger, this might have side effects */
	int force_active;

	struct logbuffer *log;

	/* TCPM state for wired PPS charging */
	const char *tcpm_psy_name;
	struct power_supply *tcpm_psy;
	struct pd_pps_data tcpm_pps_data;
	int log_psy_ratelimit;
	u32 tcpm_phandle;

	/* TCPM state for wireless PPS charging */
	const char *wlc_dc_name;
	struct power_supply *wlc_dc_psy;
	struct pd_pps_data wlc_pps_data;
	u32 wlc_phandle;

	struct delayed_work select_work;

	/* set to force PPS negoatiation */
	bool force_pps;
	/* pps state and detect */
	struct delayed_work pps_work;
	/* request of output ua, */
	int out_ua;
	int out_uv;

	int dcen_gpio;
	u32 dcen_gpio_default;

	/* >0 when enabled, pps charger to use */
	int pps_index;
	/* >0 when enabled, dc_charger */
	int dc_index;
	/* dc_charging state */
	int dc_state;

	ktime_t dc_start_time;

	/* Disable DC control */
	int dc_ctl;

	/* force check of the DC limit again (debug) */
	bool new_dc_limit;

	/* taper off of current at tier, voltage */
	u32 taper_step_interval;	/* countdown interval in seconds */
	u32 taper_step_voltage;		/* voltage before countdown */
	u32 taper_step_current;		/* current before countdown */
	u32 taper_step_grace;		/* steps from voltage before countdown */
	u32 taper_step_count;		/* countdown steps before dc_done */
	u32 taper_step_fv_margin;		/* countdown steps before dc_done */
	u32 taper_step_cc_step;		/* countdown steps before dc_done */
	int taper_step;			/* actual countdown */

	/* policy: power demand limit for DC charging */
	u32 dc_limit_vbatt_low;		/* DC will not stop until low */
	u32 dc_limit_vbatt_min;		/* DC will start at min */
	u32 dc_limit_vbatt_high;	/* DC will not start over high */
	u32 dc_limit_vbatt_max;		/* DC stop at max */
	u32 dc_limit_demand;

	/* TODO: keep TCPM/DC state in a structure add there */
	u32 dc_limit_cc_min;		/* PPS_DC stop if CC_MAX is under this */
	u32 dc_limit_cc_min_wlc;	/* WLC_DC stop if CC_MAX is under this */

	/* cc_max and fv_uv are the demand from google_charger */
	int cc_max;
	int fv_uv;

	bool dc_init_complete;
	bool init_complete;
	bool resume_complete;
	struct notifier_block chg_nb;

	/* tie up to charger mode */
	struct gvotable_election *gbms_mode;

	/* debug fs */
	struct dentry *debug_entry;
};

#define gcpm_psy_name(psy) \
	((psy) && (psy)->desc && (psy)->desc->name ? (psy)->desc->name : "???")

/* TODO: handle capabilities based on index number */
#define gcpm_is_dc(gcpm, index) \
	((index) >= GCPM_INDEX_DC_ENABLE)


/* Logging ----------------------------------------------------------------- */

int debug_printk_prlog = LOGLEVEL_INFO;

/*
 * A \n in a logbuffer string has a special meaning and is usually not included
 * in the debug messages for it. The \n is necessary for printk.
 */
static void logbuffer_prlog(struct gcpm_drv *gcpm, int level, const char *f, ...)
{
	va_list args;

	va_start(args, f);

	logbuffer_vlog(gcpm->log, f, args);
	if (level <= debug_printk_prlog)
		vprintk(f, args);

	va_end(args);
}

/* ------------------------------------------------------------------------- */

static int gcpm_dc_fcc_update(struct gcpm_drv *gcpm, int limit);

static struct power_supply *gcpm_chg_get_charger(const struct gcpm_drv *gcpm, int index)
{
	return (index < 0 || index >= gcpm->chg_psy_count) ? NULL : gcpm->chg_psy_avail[index];
}

static struct power_supply *gcpm_chg_get_default(const struct gcpm_drv *gcpm)
{
	return gcpm_chg_get_charger(gcpm, GCPM_DEFAULT_CHARGER);
}

/* TODO: place a lock around the operation? */
static struct power_supply *gcpm_chg_get_active(const struct gcpm_drv *gcpm)
{
	return gcpm_chg_get_charger(gcpm, gcpm->chg_psy_active);
}

static int gcpm_chg_ping(struct gcpm_drv *gcpm, int index, bool online)
{
	struct power_supply *chg_psy;
	int ret;

	chg_psy = gcpm->chg_psy_avail[index];
	if (!chg_psy)
		return 0;

	ret = GPSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_ONLINE, 0);
	if (ret < 0)
		pr_debug("adapter %d cannot ping (%d)", index, ret);

	return 0;
}

/* use the charger one when avalaible or fallback to the generated one */
static uint64_t gcpm_get_charger_state(const struct gcpm_drv *gcpm,
				       struct power_supply *chg_psy)
{
	union gbms_charger_state chg_state;
	int rc;

	rc = gbms_read_charger_state(&chg_state, chg_psy);
	if (rc < 0)
		return 0;

	return chg_state.v;
}

/*
 * chg_psy_active==-1 if index was active
 * NOTE: GBMS_PROP_CHARGING_ENABLED will be pinged later on
 */
static int gcpm_chg_offline(struct gcpm_drv *gcpm, int index)
{
	const int active_index = gcpm->chg_psy_active;
	struct power_supply *chg_psy;
	int ret;

	chg_psy = gcpm_chg_get_charger(gcpm, index);
	if (!chg_psy)
		return 0;

	/* OFFLINE should stop charging */
	ret = GPSY_SET_PROP(chg_psy, GBMS_PROP_CHARGING_ENABLED, 0);
	if (ret == 0)
		ret = GPSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_ONLINE, 0);
	if (ret == 0 && gcpm->chg_psy_active == index)
		gcpm->chg_psy_active = -1;

	pr_info("%s: %s active=%d->%d offline_ok=%d\n", __func__,
		 pps_name(chg_psy), active_index, gcpm->chg_psy_active, ret == 0);

	return ret;
}

/* preset charging parameters */
static int gcpm_chg_preset(struct power_supply *chg_psy, int fv_uv, int cc_max)
{
	const char *name = gcpm_psy_name(chg_psy);
	int ret;

	pr_debug("%s: %s fv_uv=%d cc_max=%d\n", __func__, name, fv_uv, cc_max);

	ret = GPSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
			    fv_uv);
	if (ret < 0) {
		pr_err("%s: %s no fv_uv (%d)\n", __func__, name, ret);
		return ret;
	}

	ret = GPSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
			    cc_max);
	if (ret < 0)
		pr_err("%s: %s no cc_max (%d)\n", __func__, name, ret);

	return ret;
}

/* setting online might start charging (if ENABLE is set) */
static int gcpm_chg_online(struct gcpm_drv *gcpm, struct power_supply *chg_psy)
{
	const char *name = gcpm_psy_name(chg_psy);
	bool preset_ok = true;
	int ret;

	if (!gcpm) {
		pr_err("%s: invalid charger\n", __func__);
		return -EINVAL;
	}

	/* preset the new charger */
	ret = gcpm_chg_preset(chg_psy, gcpm->fv_uv, gcpm->cc_max);
	if (ret < 0)
		preset_ok = false;

	/* online (but so we can enable it) */
	ret = GPSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_ONLINE, 1);
	if (ret < 0) {
		pr_debug("%s: %s online failed (%d)\n", __func__, name, ret);
		return ret;
	}

	/* retry preset if failed */
	if (!preset_ok)
		ret = gcpm_chg_preset(chg_psy, gcpm->fv_uv, gcpm->cc_max);
	if (ret < 0) {
		int rc;

		pr_err("%s: %s preset failed (%d)\n", __func__, name, ret);

		rc = GPSY_SET_PROP(chg_psy, POWER_SUPPLY_PROP_ONLINE, 0);
		if (rc < 0)
			pr_err("%s: %s offline failed (%d)\n", __func__, name, rc);
	}

	return ret;
}

/*
 * gcpm->chg_psy_active == gcpm->dc_index on success.
 * NOTE: call with a lock around gcpm->chg_psy_lock
 */
static int gcpm_chg_start(struct gcpm_drv *gcpm, int index)
{
	const int active_index = gcpm->chg_psy_active;
	struct power_supply *chg_psy;
	int ret = -EINVAL;

	if (index == active_index)
		return 0;

	if (active_index != -1)
		pr_err("%s: %d->%d not idle\n", __func__, active_index, index);

	/* validate the index before switch */
	chg_psy = gcpm_chg_get_charger(gcpm, index);
	if (chg_psy)
		ret = gcpm_chg_online(gcpm, chg_psy);
	if (ret < 0) {
		/* TODO: force active_index if != -1 */
		pr_debug("%s: index=%d not online (%d)\n",
			 __func__, index, ret);
		return ret;
	}

	pr_debug("%s: active=%d->%d\n", __func__, active_index, index);

	gcpm->chg_psy_active = index;
	return ret;
}

/*
 * Enable DirectCharge mode, PPS and DC charger must be already initialized
 * NOTE: disable might restart the default charger with stale settings
 */
static int gcpm_dc_enable(struct gcpm_drv *gcpm, bool enabled)
{
	if (gcpm->dcen_gpio >= 0 && !gcpm->dcen_gpio_default)
		gpio_set_value(gcpm->dcen_gpio, enabled);

	if (!gcpm->gbms_mode) {
		struct gvotable_election *v;

		v = gvotable_election_get_handle(GBMS_MODE_VOTABLE);
		if (IS_ERR_OR_NULL(v))
			return -ENODEV;
		gcpm->gbms_mode = v;
	}

	return gvotable_cast_vote(gcpm->gbms_mode, "GCPM",
				  (void*)GBMS_CHGR_MODE_CHGR_DC,
				  enabled);
}

/*
 * disable DC and switch back to the default charger. Final DC statate is
 * DC_IDLE (i.e. this can be used to reset dc_state from DC_DISABLED).
 * NOTE: call with a lock around gcpm->chg_psy_lock
 * NOTE: I could pass in and return dc_state instead of changing gcpm
 * must  hold a lock on mutex_lock(&gcpm->chg_psy_lock);
 */
static int gcpm_dc_stop(struct gcpm_drv *gcpm, int index)
{
	int ret = 0;

	if (!gcpm_is_dc(gcpm, index))
		return 0;

	switch (gcpm->dc_state) {
	case DC_RUNNING:
	case DC_PASSTHROUGH:
		ret = gcpm_chg_offline(gcpm, index);
		if (ret < 0)
			pr_warn("DC_PPS: Cannot offline DC index=%d (%d)",
				index, ret);
		else
			gcpm->dc_state = DC_ENABLE;
		/* Fall Through */
	case DC_ENABLE:
	case DC_ENABLE_PASSTHROUGH:
		ret = gcpm_dc_enable(gcpm, false);
		if (ret < 0) {
			pr_err("DC_PPS: Cannot disable DC (%d)", ret);
			break;
		}
		/* Fall Through */
	default:
		gcpm->dc_state = DC_DISABLED;
		break;
	}

	return ret;
}

/* NOTE: call with a lock around gcpm->chg_psy_lock */
static int gcpm_dc_start(struct gcpm_drv *gcpm, int index)
{
	struct power_supply *dc_psy;
	int ret;

	/* ENABLE will be called by the dc_pps workloop */
	ret = gcpm_chg_start(gcpm, index);
	if (ret < 0) {
		pr_err("PPS_DC: index=%d not started (%d)\n", index, ret);
		return ret;
	}

	/*
	 * Restoring the DC_FCC limit might change charging current and cause
	 * demand to fall under dc_limit_demand. The possible resulting loop
	 * (enable/disable) is solved in gcpm_chg_select_work().
	 */
	ret = gcpm_dc_fcc_update(gcpm, gcpm->dc_fcc_limit);
	if (ret < 0)
		pr_err("PPS_Work: cannot update DC_FCC limit (%d)\n", ret);

	dc_psy = gcpm_chg_get_active(gcpm);
	if (!dc_psy) {
		pr_err("PPS_DC: gcpm->dc_state == DC_READY, no adapter\n");
		return -ENODEV;
	}

	/* set IIN_CFG (might not need) */
	ret = GPSY_SET_PROP(dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX,
			    gcpm->out_ua);
	if (ret < 0) {
		pr_err("PPS_DC: no IIN (%d)\n", ret);
		return ret;
	}

	/* vote on MODE */
	ret = gcpm_dc_enable(gcpm, true);
	if (ret < 0) {
		pr_err("PPS_DC: dc_ready failed=%d\n", ret);
		return ret;
	}

	pr_debug("PPS_DC: dc_ready ok state=%d fv_uv=%d cc_max=%d, out_ua=%d\n",
		gcpm->dc_state, gcpm->fv_uv, gcpm->cc_max, gcpm->out_ua);

	return 0;
}

/*
 * Select the DC charger using the thermal policy.
 * DC charging is enabled when demand is over dc_limit (default 0) and
 * vbatt > vbatt_min (default or device tree). DC is not disabled when
 * vbatt is over vbat low.
 * DC is stopped when vbatt is over vbatt_max (default or DT) and not started
 * when vbatt is over vbatt_high (some default 200mV under vbatt_max).
 * NOTE: program target before enabling chaging.
 */
enum gcpm_dc_ctl_t {
	GCPM_DC_CTL_DEFAULT = 0,
	GCPM_DC_CTL_DISABLE_WIRED,
	GCPM_DC_CTL_DISABLE_WIRELESS,
	GCPM_DC_CTL_DISABLE_BOTH,
};

/* call holding mutex_lock(&gcpm->chg_psy_lock) */
static int gcpm_chg_select(struct gcpm_drv *gcpm)
{
	struct power_supply *chg_psy;
	bool use_dc_limits = gcpm->pps_index == PPS_INDEX_WLC;
	int batt_demand, index = GCPM_DEFAULT_CHARGER;
	int cc_min = gcpm->dc_limit_cc_min;
	int cc_max = gcpm->cc_max;
	int vbatt = -1;

	if (!gcpm->dc_init_complete)
		return GCPM_DEFAULT_CHARGER;

	/* overrides dc_fcc_hold, might trigger taper_control */
	if (gcpm->force_active >= 0)
		return gcpm->force_active;

	/* kill switch */
	if (gcpm->dc_ctl == GCPM_DC_CTL_DISABLE_BOTH)
		return GCPM_DEFAULT_CHARGER;

	 /*
	  * ->dc_fcc_hold is cleared int the dc_fcc callback when the
	  * dc_fcc_limit changes and is nonzero. The hold keeps the
	  * default charger enabled when charging from WLC.
	  */
	if (gcpm->dc_fcc_hold) {
		pr_debug("%s: CPM_THERM_DC_FCC hold, use_dc_limits:%d->%d\n",
			 __func__, use_dc_limits, gcpm->wlc_pps_data.pd_online);

		use_dc_limits = gcpm->wlc_pps_data.pd_online;
	}

	/*
	 * Prevents enable/disable loops in gcpm_pps_wlc_dc_work().
	 * Need to check the value directly because source selection is done
	 * holding a lock on &gcpm->chg_psy_lock (cc_max will become the same
	 * as gcpm->dc_fcc_limit on exit).
	 */
	if (use_dc_limits) {
		pr_debug("%s: CPM_THERM_DC_FCC use_dc_limits cc_max=%d->%d cc_min=%d->%d\n",
			 __func__, cc_max, gcpm->dc_fcc_limit,
			 cc_min, gcpm->dc_limit_cc_min_wlc);

		/* set dc_limit_cc_min_wlc to 0 in DT to disable WLC_DC */
		if (gcpm->dc_limit_cc_min_wlc >= 0)
			cc_min = gcpm->dc_limit_cc_min_wlc;

		/* 0 dc_fcc limit forces the default charger */
		if (gcpm->dc_fcc_limit >= 0)
			cc_max = gcpm->dc_fcc_limit;
	}

	/* keeps on default charger until we have valid charging parameters */
	if (cc_max <= 0 || gcpm->fv_uv <= 0)
		return GCPM_DEFAULT_CHARGER;

	/*
	 * battery demand comes from charging tier or thermal limit: leave
	 * dc_limit_demand to 0 to use only cc_min. The demand is the same
	 * for wired and wireless: we might need to change it.
	 *
	 * TODO: must make DEFAULT_CHARGER to be more likely to be selected
	 * at higher voltage. Can do using (max_fv_uv - gcpm->fv_uv) instead
	 * of simply using gcpm->fv_uv when calculating the limit.
	 *
	 * TODO: handle capabilities based on index number
	 */
	batt_demand = (cc_max / 1000) * (gcpm->fv_uv / 1000);
	if (batt_demand > gcpm->dc_limit_demand)
		index = GCPM_INDEX_DC_ENABLE;

	pr_debug("%s: index=%d cc_max=%d gcpm->fv_uv=%d demand=%d, dc_limit=%d\n", __func__,
		 index, cc_max / 1000, gcpm->fv_uv / 1000,
		 batt_demand, gcpm->dc_limit_demand);

	/* TODO: add debounce on demand */

	/*
	 * min and max are hard limits, low and high are debounce.
	 * 	low < MIN < high < MAX
	 * TODO: factor to a separate function, pass vbatt in
	 * NOTE: check the current charger, should check battery?
	 */
	chg_psy = gcpm_chg_get_default(gcpm);
	if (chg_psy) {
		const int vbatt_min = gcpm->dc_limit_vbatt_min;
		const int vbatt_max = gcpm->dc_limit_vbatt_max;
		const int vbatt_high = gcpm->dc_limit_vbatt_high;

		/* NOTE: check the current charger, should check battery? */
		vbatt = GPSY_GET_PROP(chg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
		if (vbatt < 0) {
			pr_err("CHG_CHK cannot read vbatt %d\n", vbatt);
			return gcpm->dc_index;
		}

		/*
		 * re-enable DC_ICL if it was disabled by taper control.
		 * TODO: Move to gcpm_chg_select_work() next to the taper
		 * control logic.
		 * NOTE: the check in gcpm_chg_dc_check_source() prevents this
		 * from retrying to enable DC while on NON PPS adapters.
		 */
		if (gcpm->fv_uv < vbatt_high && gcpm->dc_state == DC_DISABLED)
			gcpm->dc_state = DC_IDLE;

		/*
		 * Need to keep checking when vbatt is under low to make sure
		 * that DC starts at min. No keeps the current
		 */
		if (vbatt_max || vbatt_min) {
			const int vbatt_low = gcpm->dc_limit_vbatt_low;

			/* -EAGAIN will check for it */
			if (vbatt_low && vbatt < vbatt_low)
				return gcpm->dc_index == GCPM_DEFAULT_CHARGER ?
				      -EAGAIN : gcpm->dc_index; /* debounce */

			if (vbatt_min && vbatt < vbatt_min)
				index = gcpm->dc_index == GCPM_DEFAULT_CHARGER ?
					-EAGAIN : gcpm->dc_index; /* debounce? */
			else if (vbatt_max && vbatt > vbatt_max)
				index = GCPM_DEFAULT_CHARGER; /* disable */
			else if (vbatt_high && vbatt > vbatt_high)
				index = gcpm->dc_index; /* debounce */
			else if (vbatt_min && vbatt < vbatt_min)
				index = GCPM_DEFAULT_CHARGER;

			/* demand decides DC unless under vmin */
			pr_debug("%s: index=%d->%d vbatt=%d: low=%d min=%d high=%d max=%d\n",
				 __func__, gcpm->dc_index, index, vbatt, vbatt_low, vbatt_min,
				 vbatt_high, vbatt_max);
		}
	}

	if (index >= gcpm->chg_psy_count) {
		pr_err("CHG_CHK index=%d out of bounds %d\n", index,
		       gcpm->chg_psy_count);
		index = GCPM_DEFAULT_CHARGER;
	}

	/* thermals might have reduced demand, use the cc_min limit */
	if (cc_max <= cc_min && index != GCPM_DEFAULT_CHARGER) {
		pr_debug("%s: cc_max=%d under cc_min=%d, ->hold=%d:%d index:%d->%d\n",
			 __func__, cc_max, cc_min, gcpm->dc_fcc_hold,
			 use_dc_limits, index, GCPM_DEFAULT_CHARGER);

		/* reset when the limit changes */
		gcpm->dc_fcc_hold = use_dc_limits;
		index = GCPM_DEFAULT_CHARGER;
	}

	/* TODO: more qualifiers here */

	if (index != gcpm->dc_index)
		logbuffer_log(gcpm->log,
			      "%s: index:%d->%d vbatt=%d demand=%d,limit=%d cc_max=%d,cc_min=%d, hold=%d",
			      __func__, gcpm->dc_index, index, vbatt,
			      batt_demand, gcpm->dc_limit_demand,
			      cc_max, cc_min, gcpm->dc_fcc_hold);

	return index;
}

static bool gcpm_chg_dc_check_source(const struct gcpm_drv *gcpm, int index)
{
	/* Will run detection only the first time */
	if (gcpm->tcpm_pps_data.stage == PPS_NOTSUPP &&
	    gcpm->wlc_pps_data.stage == PPS_NOTSUPP )
		return false;

	return gcpm_is_dc(gcpm, index);
}

/* reset gcpm pps state */
static void gcpm_pps_online(struct gcpm_drv *gcpm)
{
	/* reset setpoint */
	gcpm->out_ua = -1;
	gcpm->out_uv = -1;

	/* reset detection */
	if (gcpm->tcpm_pps_data.pps_psy) {
		pps_init_state(&gcpm->tcpm_pps_data);
		if (gcpm->dc_ctl & GCPM_DC_CTL_DISABLE_WIRED)
			gcpm->tcpm_pps_data.stage = PPS_NOTSUPP;
	}
	if (gcpm->wlc_pps_data.pps_psy) {
		pps_init_state(&gcpm->wlc_pps_data);
		if (gcpm->dc_ctl & GCPM_DC_CTL_DISABLE_WIRELESS)
			gcpm->wlc_pps_data.stage = PPS_NOTSUPP;
	}
	gcpm->pps_index = 0;
}

static struct pd_pps_data *gcpm_pps_data(struct gcpm_drv *gcpm)
{
	struct pd_pps_data *pps_data = NULL;

	if (gcpm->pps_index == PPS_INDEX_TCPM)
		pps_data = &gcpm->tcpm_pps_data;
	else if (gcpm->pps_index == PPS_INDEX_WLC)
		pps_data = &gcpm->wlc_pps_data;

	return pps_data;
}

/* Wait for a source to become ready for the handoff */
static int gcpm_pps_wait_for_ready(struct gcpm_drv *gcpm)
{
	struct pd_pps_data *pps_data = gcpm_pps_data(gcpm);
	int pps_ui, vout = -1, iout = -1;
	bool pwr_ok = false;

	if (!pps_data)
		return -ENODEV;

	/* determine the limit/levels if needed */
	if (gcpm->pps_index == PPS_INDEX_WLC) {
		struct power_supply *chg_psy = gcpm_chg_get_active(gcpm);
		int vbatt = -1;

		if (chg_psy)
			vbatt = GPSY_GET_PROP(chg_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
		if (vbatt > 0)
			vout = vbatt * 4;
	} else {
		pwr_ok = true;
	}

	/* always need to ping */
	pps_ui = pps_update_adapter(pps_data, vout, iout, pps_data->pps_psy);
	if (pps_ui < 0) {
		pr_err("PPS_Work: pps update, dc_state=%d (%d)\n",
			gcpm->dc_state, pps_ui);
		return pps_ui;
	}

	/* wait until adapter is at or over request */
	pwr_ok |= (vout <=0 || pps_data->out_uv >= vout) &&
		  (iout <=0 || pps_data->op_ua >= iout );

	pr_info("PPS_Work: pwr_ok=%d pps_ui=%d vout=%d out_uv=%d iout=%d op_ua=%d\n",
		pwr_ok, pps_ui, vout, pps_data->out_uv, iout, pps_data->op_ua);

	return pwr_ok ? pps_ui : -EAGAIN;
}

/*
 * Pick the first PPS source that transition to PPS_ACTIVE:
 *
 * ->stage ==
 *	DISABLED => NONE -> AVAILABLE -> ACTIVE -> DISABLED
 *		 -> DISABLED
 *		 -> NOTSUPP
 *
 * return 0 if needs to continue polling, -ENODEV if none of the sources
 * support pps.
 */
static int gcpm_pps_work(struct gcpm_drv *gcpm)
{
	int ret = 0, pps_index = 0;
	int not_supported = 0;

	if (gcpm->tcpm_pps_data.stage != PPS_NOTSUPP) {
		struct pd_pps_data *pps_data = &gcpm->tcpm_pps_data;
		int pps_ui;

		pps_ui = pps_work(pps_data, pps_data->pps_psy);
		if (pps_ui >= 0 && pps_data->stage == PPS_ACTIVE)
			pps_index = PPS_INDEX_TCPM;

		if (pps_data->pd_online < PPS_PSY_PROG_ONLINE)
			pr_debug("PPS_Work: TCPM Wait pps_ui=%d online=%d, stage=%d\n",
				pps_ui, pps_data->pd_online, pps_data->stage);
	} else {
		not_supported += 1;
	}

	if (gcpm->wlc_pps_data.stage != PPS_NOTSUPP) {
		struct pd_pps_data *pps_data = &gcpm->wlc_pps_data;
		int pps_ui;

		pps_ui = pps_work(pps_data, pps_data->pps_psy);
		if (pps_ui >= 0 && pps_data->stage == PPS_ACTIVE)
			pps_index = PPS_INDEX_WLC;

		if (pps_data->pd_online < PPS_PSY_PROG_ONLINE)
			pr_debug("PPS_Work: WLC Wait pps_ui=%d online=%d, stage=%d\n",
				pps_ui, pps_data->pd_online, pps_data->stage);
	} else {
		not_supported += 1;
	}

	pr_debug("PPS_Work: tcpm[online=%d, stage=%d] wlc[online=%d, stage=%d] ns=%d pps_index=%d\n",
		 gcpm->tcpm_pps_data.pd_online, gcpm->tcpm_pps_data.stage,
		 gcpm->wlc_pps_data.pd_online, gcpm->wlc_pps_data.stage,
		 not_supported, pps_index);

	/* 2 sources */
	if (not_supported == PPS_INDEX_MAX)
		return -ENODEV;

	/* index==0 mean detecting */
	if (gcpm->pps_index != pps_index)
		pr_debug("PPS_Work: pps_index %d->%d\n",
			gcpm->pps_index, pps_index);

	/* went away! */
	if (gcpm->pps_index && !pps_index)
		ret = -ENODEV;

	gcpm->pps_index = pps_index;
	return ret;
}

static int gcpm_pps_timeout(struct gcpm_drv *gcpm)
{
	struct pd_pps_data *wlc_pps_data = &gcpm->wlc_pps_data;

	return (wlc_pps_data->stage != PPS_NOTSUPP && wlc_pps_data->pd_online)
		? PPS_ACTIVE_WLC_TIMEOUT_S : PPS_ACTIVE_USB_TIMEOUT_S;
}

static int gcpm_pps_offline(struct gcpm_drv *gcpm)
{
	int ret;

	/* TODO: migh be a no-op when pps_index == 0 */

	if (gcpm->tcpm_pps_data.pps_psy) {
		ret = pps_prog_offline(&gcpm->tcpm_pps_data,
				       gcpm->tcpm_pps_data.pps_psy);
		if (ret < 0)
			pr_err("PPS_DC: fail tcpm offline (%d)\n", ret);
	}

	if (gcpm->wlc_pps_data.pps_psy) {
		ret = pps_prog_offline(&gcpm->wlc_pps_data,
				       gcpm->wlc_pps_data.pps_psy);
		if (ret < 0)
			pr_err("PPS_DC: fail wlc offline (%d)\n", ret);
	}

	gcpm->pps_index = 0;
	return 0;
}

/* <=0 to disable, > 0 to enable "n" counts */
static bool gcpm_taper_ctl(struct gcpm_drv *gcpm, int count)
{
	bool changed = false;

	if (count <= 0) {
		changed = gcpm->taper_step != 0;
		gcpm->taper_step = 0;
	} else if (gcpm->taper_step == 0) {
		gcpm->taper_step = count;
		changed = true;
	}

	return changed;
}

static bool gcpm_taper_step(const struct gcpm_drv *gcpm, int taper_step)
{
	const int delta = gcpm->taper_step_count - taper_step;
	int fv_uv = gcpm->fv_uv, cc_max = gcpm->cc_max;
	struct power_supply *dc_psy;

	if (taper_step <= 0)
		return true;
	/*
	 * TODO: on a race between TAPER and select, active might not
	 * be a DC source. Force done to prevent voltage spikes.
	 */
	dc_psy = gcpm_chg_get_active(gcpm);
	if (!dc_psy)
		return true;

	/* Optional dc voltage limit */
	if (gcpm->taper_step_voltage) {
		int vbatt;

		vbatt = GPSY_GET_PROP(dc_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
		if (vbatt < 0)
			pr_err("%s: cannot read voltage (%d)", __func__, vbatt);
		else if (vbatt < gcpm->taper_step_voltage)
			return false;
	}

	/* Optional dc current limit */
	if (gcpm->taper_step_current) {
		int ret, ibatt;

		/* TODO: use current average if available */
		ret = GPSY_GET_INT_PROP(dc_psy, POWER_SUPPLY_PROP_CURRENT_NOW,
					&ibatt);
		if (ret < 0)
			pr_err("%s: cannot read current (%d)", __func__, ret);
		else if (ibatt > gcpm->taper_step_current)
			return false;
	}

	/* delta < 0 during the grace period, which will increase cc_max */
	fv_uv -= gcpm->taper_step_fv_margin;
	if (gcpm->taper_step_cc_step) {
		cc_max -= delta * gcpm->taper_step_cc_step;
		if (cc_max < gcpm->cc_max / 2)
			cc_max = gcpm->cc_max / 2;
	}

	/* increase of cc_max due to delta < 0 are ignored */
	if (cc_max < gcpm->cc_max) {
		int ret;

		/* failure to preset stop taper and revert to main */
		ret = gcpm_chg_preset(dc_psy, fv_uv, cc_max);
		pr_info("CHG_CHK: taper_step=%d fv_uv=%d->%d, cc_max=%d->%d\n",
			 taper_step, gcpm->fv_uv, fv_uv, gcpm->cc_max, cc_max);
		if (ret < 0) {
			pr_err("CHG_CHK: taper_step=%d failed, revert (%d)\n",
			       taper_step, ret);
			return true;
		}

	} else {
		pr_debug("CHG_CHK: grace taper_step=%d fv_uv=%d, cc_max=%d\n",
			 taper_step, gcpm->fv_uv, gcpm->cc_max);
	}

	logbuffer_log(gcpm->log, "taper_step=%d delta=%d fv_uv=%d->%d, cc_max=%d->%d",
		      taper_step, delta, gcpm->fv_uv, fv_uv, gcpm->cc_max, cc_max);


	/* not done */
	return false;
}

/*
 * triggered on every FV_UV and in DC_PASSTHROUGH
 * will keep polling if in -EAGAIN
 */
static void gcpm_chg_select_work(struct work_struct *work)
{
	struct gcpm_drv *gcpm =
		container_of(work, struct gcpm_drv, select_work.work);
	int index, schedule_pps_interval = -1;
	bool dc_done = false, dc_ena;

	mutex_lock(&gcpm->chg_psy_lock);

	pr_debug("%s: on=%d dc_state=%d dc_index=%d\n", __func__,
		 gcpm->dc_init_complete, gcpm->dc_state, gcpm->dc_index);

	if (!gcpm->dc_init_complete) {
		const int interval = 5; /* 5 seconds */

		mod_delayed_work(system_wq, &gcpm->select_work,
				 msecs_to_jiffies(interval * 1000));
		goto unlock_done;
	}

	index = gcpm_chg_select(gcpm);
	if (index < 0) {
		const int interval = 5; /* 5 seconds */

		pr_debug("%s: index=%d dc_state=%d dc_index=%d\n",
			 __func__, index, gcpm->dc_state, gcpm->dc_index);

		/* -EAGAIN is used between low and min */
		if (index == -EAGAIN)
			mod_delayed_work(system_wq, &gcpm->select_work,
					 msecs_to_jiffies(interval * 1000));

		goto unlock_done;
	}

	/* will not try to enable if the source cannot do PPS */
	dc_ena = gcpm_chg_dc_check_source(gcpm, index);
	if (dc_ena && gcpm->dc_fcc_hold) {
		pr_debug("%s: CPM_THERM_DC_FCC hold from limit\n", __func__);
		dc_ena = false;
	}

	/*
	 * taper control reduces cc_max every gcpm->taper_step_interval seconds
	 * by a fixed amount for gcpm->taper_step_count seconds. fv_uv might
	 * also be lowered by a fixed amount.
	 */
	if (dc_ena && gcpm->taper_step > 0) {
		const int interval = msecs_to_jiffies(gcpm->taper_step_interval * 1000);

		dc_done = gcpm_taper_step(gcpm, gcpm->taper_step - 1);
		if (!dc_done) {
			mod_delayed_work(system_wq, &gcpm->select_work, interval);
			gcpm->taper_step -= 1;
		}

		pr_debug("%s: taper_step=%d done=%d\n", __func__,
			 gcpm->taper_step, dc_done);
	} else if (gcpm->taper_step != 0) {
		gcpm_taper_ctl(gcpm, 0);

		/*
		 * TODO: move the reset of DC state to DC_ENABLE when vbatt
		 * fall under vbatt_high here.
		 */
	}

	pr_debug("%s: DC dc_ena=%d dc_state=%d dc_index=%d->%d\n", __func__,
		 dc_ena, gcpm->dc_state, gcpm->dc_index, index);

	/*
	 * NOTE: disabling DC might need to transition to charger mode 0
	 * same might apply when switching between WLC-DC and PPS-DC.
	 * Figure out a way to do this if needed.
	 */
	if (!dc_ena || dc_done) {

		if (gcpm->dc_state > DC_IDLE && gcpm->dc_index > 0) {
			pr_info("CHG_CHK: dc_ena=%d dc_done=%d stop PPS_Work for dc_index=%d\n",
				dc_ena, dc_done, gcpm->dc_index);

			/*
			 * dc_done will prevent DC to restart until disconnect
			 * or voltage goes over _high.
			 */
			gcpm->dc_index = dc_done ? GCPM_INDEX_DC_DISABLE :
					 GCPM_DEFAULT_CHARGER;
			gcpm_taper_ctl(gcpm, 0);
			schedule_pps_interval = 0;
		}
	} else if (gcpm->dc_state == DC_DISABLED) {
		pr_debug("%s: PPS_Work disabled for the session\n", __func__);
	} else if (gcpm->dc_state == DC_IDLE) {
		pr_info("CHG_CHK: start PPS_Work for dc_index=%d\n", index);

		/* reset pps state to re-enable detection */
		gcpm_pps_online(gcpm);

		/* TODO: DC_ENABLE or DC_PASSTHROUGH depending on index */
		gcpm->dc_state = DC_ENABLE_PASSTHROUGH;
		gcpm->dc_index = index;

		/* grace period of 500ms, PPS Work not called during grace */
		gcpm->dc_start_time = get_boot_sec();
		schedule_pps_interval = DC_ENABLE_DELAY_MS;
	}

	if (schedule_pps_interval >= 0) {
		pr_debug("%s: DC schedule pps_work in %ds\n", __func__,
			 schedule_pps_interval / 1000);

		mod_delayed_work(system_wq, &gcpm->pps_work,
				 msecs_to_jiffies(schedule_pps_interval));
	}

unlock_done:
	mutex_unlock(&gcpm->chg_psy_lock);
}

static int gcpm_enable_default(struct gcpm_drv *gcpm)
{
	struct power_supply *chg_psy = gcpm_chg_get_default(gcpm);
	int ret;

	/* gcpm_chg_offline set GBMS_PROP_CHARGING_ENABLED = 0 */
	ret = GPSY_SET_PROP(chg_psy, GBMS_PROP_CHARGING_ENABLED, 1);
	if (ret < 0) {
		pr_debug("%s: failed 2 enable charging (%d)\n", __func__, ret);
		return ret;
	}

	/* (re) online and start the default charger */
	ret = gcpm_chg_start(gcpm, GCPM_DEFAULT_CHARGER);
	if (ret < 0) {
		pr_debug("%s: failed 2 start (%d)\n", __func__, ret);
		return ret;
	}

	return 0;
}

/* online the default charger (do not change active, nor enable) */
static int gcpm_online_default(struct gcpm_drv *gcpm)
{
	return gcpm_chg_online(gcpm, gcpm_chg_get_default(gcpm));
}

/*
 * restart the default charger after DC or while trying to start it.
 * Can come here during DC_ENABLE_PASSTHROUGH, with PPS enabled and
 * after a failure to start DC or on a failure to disable the default
 * charger.
 *
 * NOTE: the caller needs to reset gcpm->dc_index
 */
static int gcpm_pps_wlc_dc_restart_default(struct gcpm_drv *gcpm)
{
	const int active_index = gcpm->chg_psy_active; /* will change */
	const int dc_state = gcpm->dc_state; /* will change */
	int pps_done, ret;

	/* DC_FCC limit might be enabled as soon as we enter WLC_DC */
	ret = gcpm_dc_fcc_update(gcpm, -1);
	if (ret < 0)
		pr_err("PPS_Work: cannot update DC_FCC limit (%d)\n", ret);

	/* Clear taper count if not complete */
	gcpm_taper_ctl(gcpm, 0);

	/*
	 * in dc_state=DC_ENABLE_PASSTHROUGH it might  be able to take
	 * the current charger offline BUT might fail to start DC.
	 */
	if (dc_state <= DC_IDLE)
		return 0;

	/* online the default charger (do not change active, nor enable)
	 * TODO: possibly do nothing if the current charger is not DC.
	 */
	ret = gcpm_online_default(gcpm);
	if (ret < 0)
		pr_warn("%s: Cannot online default (%d)", __func__, ret);

	/*
	 * dc_state=DC_DISABLED, chg_psy_active==-1 a DC charger was active.
	 * in DC_ENABLE_PASSTHROUGH, gcpm_dc_stop() will vote on charger mode.
	 */
	ret = gcpm_dc_stop(gcpm, active_index);
	if (ret < 0) {
		pr_debug("%s: retry disable, dc_state=%d->%d (%d)\n",
			 __func__, dc_state, gcpm->dc_state, ret);
		return -EAGAIN;
	}

	/*
	 * Calling pps_offline is not really needed becasuse the adapter will
	 * revert to fixed once ping stops (pps state is re-initialized on
	 * DC start). I clear it to keep things neat and tidy.
	 *
	 * NOTE: Make sure that pps_prog_offline only changes from PROG to
	 * FIXED and not from OFFLINE to FIXED. Setting WLC from OFFLINE
	 * to FIXED (online) at the wrong time might interfere with
	 * the usecases that need to disable charging explicitly.
	 */
	pps_done = gcpm_pps_offline(gcpm);
	if (pps_done < 0)
		pr_debug("%s: fail 2 offline pps, dc_state=%d (%d)\n",
			__func__, gcpm->dc_state, pps_done);

	ret = gcpm_enable_default(gcpm);
	if (ret < 0) {
		pr_err("%s: fail 2 restart default, dc_state=%d pps_done=%d (%d)\n",
		       __func__, gcpm->dc_state, pps_done >= 0 ? : pps_done, ret);
		return -EAGAIN;
	}

	return 0;
}

/*
 * hold a lock on mutex_lock(&gcpm->chg_psy_lock);
 * @return <0 on error, 0 on limit not applied, 1 on limit applied
 */
static int gcpm_dc_fcc_update(struct gcpm_drv *gcpm, int value)
{
	int limit = value;
	int ret = -ENODEV;

	if (!gcpm->fcc_votable) {
		struct gvotable_election *v;

		v = gvotable_election_get_handle("MSC_FCC");
		if (IS_ERR_OR_NULL(v))
			goto error_exit;

		gcpm->fcc_votable = v;
	}

	/* apply/enable DC_FCC only when a WLC_DC source is selected */
	if (gcpm->pps_index != PPS_INDEX_WLC || limit < 0)
		limit = -1;

	/*
	 * The thermal voter for FCC wired must be disabled to allow higher
	 * charger rates for DC_FCC than for the wired case.
	 */
	ret = gvotable_cast_vote(gcpm->fcc_votable, "DC_FCC",
				 (void*)(uintptr_t)limit,
				 limit >= 0);
	if (ret < 0)
		pr_err("%s: vote %d on MSC_FCC failed (%d)\n",  __func__,
		       limit, ret);
	else
		ret = limit >= 0;

error_exit:
	pr_debug("%s: CPM_THERM_DC_FCC pps_index=%d value=%d limit=%d applied=%d\n",
		 __func__, gcpm->pps_index, value, limit, ret);

	return ret;
}

/*
 * pps_data->stage:
 *  PPS_NONE -> PPS_AVAILABLE -> PPS_ACTIVE
 * 	     -> PPS_DISABLED  -> PPS_DISABLED
 */
static void gcpm_pps_wlc_dc_work(struct work_struct *work)
{
	struct gcpm_drv *gcpm =
		container_of(work, struct gcpm_drv, pps_work.work);
	const ktime_t elap = get_boot_sec() - gcpm->dc_start_time;
	struct pd_pps_data *pps_data;
	int ret, pps_ui = -ENODEV;

	pr_debug("%s: ok=%d dc_index=%d dc_state=%d\n", __func__,
		 gcpm->resume_complete && gcpm->init_complete,
		 gcpm->dc_index, gcpm->dc_state);

	/* spurious during init */
	mutex_lock(&gcpm->chg_psy_lock);
	if (!gcpm->resume_complete || !gcpm->init_complete) {
		/* TODO: should probably reschedule */
		goto pps_dc_done;
	}

	/* disconnect, gcpm_chg_check() and most errors reset ->dc_index */
	if (gcpm->dc_index <= 0) {
		const int active_index = gcpm->chg_psy_active; /* will change */
		const bool dc_disable = gcpm->dc_index == GCPM_INDEX_DC_DISABLE;

		/* will leave gcpm->dc_state in DC_DISABLED */
		ret = gcpm_pps_wlc_dc_restart_default(gcpm);
		if (ret < 0) {
			pr_warn("PPS_Work: retry restart elap=%lld dc_state=%d %d->%d (%d)\n",
				elap, gcpm->dc_state, active_index,
				gcpm->chg_psy_active, ret);

			pps_ui = DC_ERROR_RETRY_MS;
			goto pps_dc_reschedule;
		}

		/* Re-enable DC if just switching to the default charger */
		if (!dc_disable)
			gcpm->dc_state = DC_IDLE;

		logbuffer_prlog(gcpm, LOGLEVEL_INFO,
			       "PPS_Work: done elap=%lld dc_state=%d %d->%d\n",
			       elap, gcpm->dc_state, active_index,
			       gcpm->chg_psy_active);

		/* TODO: send a ps event? */
		goto pps_dc_done;
	}

	/* PPS was handed over to the DC driver, just monitor it... */
	if (gcpm->dc_state == DC_PASSTHROUGH) {
		struct power_supply *dc_psy;
		bool prog_online = false;
		int index;

		/* the dc driver needs to keep the source online */
		pps_data = gcpm_pps_data(gcpm);
		if (pps_data)
			prog_online = pps_check_prog_online(pps_data);
		if (!prog_online) {
			pr_err("PPS_Work: PPS offline, elap=%lld dc_index:%d->0\n",
			       elap, gcpm->dc_index);

			gcpm->dc_index = GCPM_DEFAULT_CHARGER;
			pps_ui = DC_ERROR_RETRY_MS;
			goto pps_dc_reschedule;
		}

		/* likely changed from debug, bail */
		dc_psy = gcpm_chg_get_active(gcpm);
		if (!dc_psy) {
			pr_err("PPS_Work: No adapter, elap=%lld in PASSTHROUGH\n",
			       elap);

			pps_ui = DC_ERROR_RETRY_MS;
			goto pps_dc_reschedule;
		}

		/* something is changed: kick the revert to default */
		index = gcpm_chg_select(gcpm);
		if (index != gcpm->dc_index)
			mod_delayed_work(system_wq, &gcpm->select_work, 0);

		/* ->pps_index valid: set/ping source to DC, ping watchdog */
		ret = GPSY_SET_PROP(dc_psy, GBMS_PROP_CHARGING_ENABLED,
				    gcpm->pps_index);
		if (ret == 0) {
			ret = gcpm_chg_ping(gcpm, GCPM_DEFAULT_CHARGER, 0);
			if (ret < 0)
				pr_err("PPS_Work: ping failed, elap=%lld with %d\n",
				       elap, ret);

			/* keep running to ping the adapters */
			pps_ui = DC_RUN_DELAY_MS;
		} else if (ret == -EBUSY || ret == -EAGAIN) {
			pps_ui = DC_ERROR_RETRY_MS;
		} else {
			pr_err("PPS_Work: ping DC failed, elap=%lld (%d)\n", elap, ret);

			ret = gcpm_chg_offline(gcpm, gcpm->dc_index);
			if (ret == 0)
				ret = gcpm_enable_default(gcpm);
			if (ret < 0) {
				pr_err("PPS_Work: cannot online default %d\n", ret);
				pps_ui = DC_ERROR_RETRY_MS;
			 } else {
				pr_err("PPS_Work: dc offline\n");
				pps_ui = 0;
			}
		}

		goto pps_dc_reschedule;
	}

	/*
	 * Wait until one of the sources come online, <0 when PPS is not
	 * supported from ANY source. Deadline at PPS_PROG_TIMEOUT_S.
	 */
	ret = gcpm_pps_work(gcpm);
	if (ret < 0) {
		if (elap < PPS_PROG_TIMEOUT_S) {
			/* retry for the session  */
			pps_ui = PPS_PROG_RETRY_MS;
			gcpm_pps_online(gcpm);
		} else {
			/* TODO: abort for the session  */
			pr_err("PPS_Work: PROG timeout, elap=%lld dc_state=%d (%d)\n",
			       elap, gcpm->dc_state, ret);

			gcpm->dc_index = GCPM_DEFAULT_CHARGER;
			pps_ui = PPS_ERROR_RETRY_MS;
		}

		goto pps_dc_reschedule;
	}

	/*
	 * DC runs only when PPS is active: abort for the session if a source
	 * went PROG_ONLINE but !active within PPS_ACTIVE_TIMEOUT_S.
	 */
	pps_data = gcpm_pps_data(gcpm);
	if (!pps_data) {
		int timeout_s = gcpm_pps_timeout(gcpm);

		if (elap < timeout_s) {
			/*
			 * WLC + Auth might require a very long time to become
			 * active.
			 */
			pps_ui = PPS_ACTIVE_RETRY_MS;
		} else {
			pr_err("PPS_Work: ACTIVE timeout=%d, elap=%lld dc_state=%d (%d)\n",
			       timeout_s, elap, gcpm->dc_state, ret);

			/* TODO: abort for the session  */
			pps_ui = PPS_ERROR_RETRY_MS;
			gcpm->dc_index = 0;
		}

		goto pps_dc_reschedule;
	}

	if (gcpm->dc_state == DC_ENABLE_PASSTHROUGH) {
		int timeout_s = gcpm_pps_timeout(gcpm) + PPS_READY_DELTA_TIMEOUT_S;
		int index;

		/* Also ping the source */
		pps_ui = gcpm_pps_wait_for_ready(gcpm);
		if (pps_ui < 0) {
			pr_info("PPS_Work: wait for source timeout=%d elap=%lld, dc_state=%d (%d)\n",
				timeout_s, elap, gcpm->dc_state, pps_ui);
			if (pps_ui != -EAGAIN)
				gcpm->dc_index = GCPM_DEFAULT_CHARGER;
			if (elap > timeout_s)
				gcpm->dc_index = GCPM_DEFAULT_CHARGER;

			/* error retry */
			pps_ui = PPS_ERROR_RETRY_MS;
			goto pps_dc_reschedule;
		}

		/*
		 * source selection might have changed demand and disabled DC
		 * (WLC_DC has a different mincurrent). Revert the input
		 * selection, retry when ->dc_fcc_limit changes.
		 */
		index = gcpm_chg_select(gcpm);
		if (!gcpm_is_dc(gcpm, index)) {
			pr_info("PPS_Work: selection changed index=%d\n", index);

			gcpm->dc_index = GCPM_DEFAULT_CHARGER;
			pps_ui = PPS_ERROR_RETRY_MS;
			goto pps_dc_reschedule;
		}

		/*
		 * offine current adapter and start new. Charging is enabled
		 * in DC_PASSTHROUGH setting GBMS_PROP_CHARGING_ENABLED to
		 * the PPS source.
		 * TODO: preset the DC charger before handoff
		 * NOTE: There are a bunch of interesting recovery scenarios.
		 */
		ret = gcpm_chg_offline(gcpm, gcpm->chg_psy_active);
		if (ret == 0)
			ret = gcpm_dc_start(gcpm, gcpm->dc_index);
		if (ret == 0) {
			gcpm->dc_state = DC_PASSTHROUGH;
			pps_ui = DC_ENABLE_DELAY_MS;
		} else if (pps_ui > DC_ERROR_RETRY_MS) {
			pps_ui = DC_ERROR_RETRY_MS;
		}
	} else {
		struct power_supply *pps_psy = pps_data->pps_psy;

		/* steady on PPS, if DC state is DC_ENABLE or DC_RUNNING */
		pps_ui = pps_update_adapter(pps_data, -1, -1, pps_psy);

		pr_info("PPS_Work: STEADY pd_online=%d pps_ui=%d dc_ena=%d dc_state=%d\n",
			pps_data->pd_online, pps_ui, gcpm->dc_index,
			gcpm->dc_state);
		if (pps_ui < 0)
			pps_ui = PPS_ERROR_RETRY_MS;
	}

pps_dc_reschedule:
	if (pps_ui <= 0) {
		pr_debug("PPS_Work: pps_ui=%d dc_index=%d dc_state=%d",
			 pps_ui, gcpm->dc_index, gcpm->dc_state);
	} else {
		pr_debug("PPS_Work: reschedule in %d dc_index=%d dc_state=%d (%d:%d)",
			 pps_ui, gcpm->dc_index, gcpm->dc_state, gcpm->out_uv, gcpm->out_ua);

		schedule_delayed_work(&gcpm->pps_work, msecs_to_jiffies(pps_ui));
	}

pps_dc_done:
	mutex_unlock(&gcpm->chg_psy_lock);
}

static void gcpm_dc_fcc_callback(struct gvotable_election *el,
				 const char *reason,
				 void *value)
{
	struct gcpm_drv *gcpm = gvotable_get_data(el);
	const int limit = (long)value;
	int changed = gcpm->dc_fcc_limit != limit;
	int applied;

	mutex_lock(&gcpm->chg_psy_lock);

	gcpm->dc_fcc_limit = limit;

	/*
	 * applied=1 when the dc_limit has voted on MSC_FCC and we need to
	 * re-run the selection.
	 */
	applied = gcpm_dc_fcc_update(gcpm, limit);
	if (applied < 0)
		pr_err("%s: cannot enforce DC_FCC limit applied=%d\n",
			__func__, applied);


	/*
	 * ->dc_fcc_hold is set in select_work() for WLC_DC sessions when
	 * the dc alternate min limit disables DC charging. Clearing ->hold
	 * re-enable
	 */
	if (limit != 0 && gcpm->dc_fcc_hold) {
		gcpm->dc_fcc_hold = false;
		changed += 1;
	}

	pr_debug("%s: CPM_THERM_DC_FCC limit=%d hold=%d applied=%d changed=%d\n",
		 __func__, limit, gcpm->dc_fcc_hold, applied, changed);

	/* ->dc_fcc_hold forces GCPM_DEFAULT_CHARGER in gcpm_chg_select() */
	if (applied || changed)
		mod_delayed_work(system_wq, &gcpm->select_work,
				 msecs_to_jiffies(DC_ENABLE_DELAY_MS));

	mutex_unlock(&gcpm->chg_psy_lock);
}

/* --------------------------------------------------------------------- */

static int gcpm_psy_set_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *pval)
{
	struct gcpm_drv *gcpm = power_supply_get_drvdata(psy);
	struct power_supply *chg_psy = NULL;
	bool ta_check = false;
	bool route = true;
	int ret = 0;

	pm_runtime_get_sync(gcpm->device);
	if (!gcpm->init_complete || !gcpm->resume_complete) {
		pm_runtime_put_sync(gcpm->device);
		return -EAGAIN;
	}
	pm_runtime_put_sync(gcpm->device);

	mutex_lock(&gcpm->chg_psy_lock);
	switch (psp) {
	/* do not route to the active charger */
	case GBMS_PROP_TAPER_CONTROL: {
		int count = 0;

		if (pval->intval != GBMS_TAPER_CONTROL_OFF)
			count = gcpm->taper_step_count + gcpm->taper_step_grace;

		/* ta_check is set when taper control changes value */
		ta_check = gcpm_taper_ctl(gcpm, count);
		route = false;
	} break;

	/* also route to the active charger */
	case GBMS_PROP_CHARGE_DISABLE:

		/* google_charger send this on disconnect and input_suspend. */
		pr_info("%s: ChargeDisable value=%d dc_index=%d dc_state=%d\n",
			__func__, pval->intval, gcpm->dc_index, gcpm->dc_state);

		if (pval->intval) {
			/*
			 * more or less the same as gcpm_pps_wlc_dc_work() when
			 * dc_index <= 0. But the default charger must not be
			 * restarted in this case though.
			 * TODO: factor the code with gcpm_pps_wlc_dc_work().
			 */

			/*
			 * No op if the current source is not DC, ->dc_state
			 * will be DC_DISABLED if this was actually disabled.
			 */
			ret = gcpm_dc_stop(gcpm,  gcpm->chg_psy_active);
			if (ret == -EAGAIN) {
				mutex_unlock(&gcpm->chg_psy_lock);
				return -EAGAIN;
			}

			ret = gcpm_pps_offline(gcpm);
			if (ret < 0)
				pr_debug("%s: fail 2 offline pps, dc_state=%d (%d)\n",
					__func__, gcpm->dc_state, ret);

			/* reset to the default charger, and clear taper */
			gcpm->dc_index = GCPM_DEFAULT_CHARGER;
			gcpm_taper_ctl(gcpm, 0);

			/*
			 * no-op if dc was NOT running, set online the charger
			 * but do not start it otherwise.
			 */
			ret = gcpm_chg_start(gcpm, GCPM_DEFAULT_CHARGER);
			if (ret < 0)
				pr_err("%s: cannot start default (%d)\n",
				       __func__, ret);

			pr_info("%s: ChargeDisable value=%d dc_index=%d dc_state=%d\n",
				__func__, pval->intval, gcpm->dc_index, gcpm->dc_state);

			/*
			 * route = true so active will get the property.
			 * No need to re-check the TA selection on disable.
			 */
			ta_check = false;
		} else if (gcpm->dc_state <= DC_IDLE) {
			/*
			 * ->dc_state will be DC_DISABLED if DC was disabled
			 * via GBMS_PROP_CHARGE_DISABLE(1) of from other
			 * conditions such as taper control.
			 */
			if (gcpm->dc_state == DC_DISABLED)
				gcpm->dc_state = DC_IDLE;

			pr_info("%s: ChargeDisable value=%d dc_index=%d dc_state=%d\n",
				__func__, pval->intval, gcpm->dc_index, gcpm->dc_state);

			gcpm_pps_online(gcpm);
			ta_check = true;
		}

		break;
	case POWER_SUPPLY_PROP_ONLINE:
		pr_info("%s: ONLINE value=%d dc_index=%d dc_state=%d\n",
			__func__, pval->intval, gcpm->dc_index,
			gcpm->dc_state);
		ta_check = true;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		psp = POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX;
		/* compat, fall through */
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ta_check = gcpm->fv_uv != pval->intval;
		gcpm->fv_uv = pval->intval;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ta_check = gcpm->cc_max != pval->intval;
		gcpm->cc_max = pval->intval;
		break;

	/* just route to the active charger */
	default:
		break;
	}

	/* used only for debug */
	if (gcpm->new_dc_limit) {
		gcpm->new_dc_limit = false;
		ta_check = true;
	}

	/* logic that select the active charging */
	if (gcpm->dc_init_complete && ta_check)
		mod_delayed_work(system_wq, &gcpm->select_work, 0);

	/*  route to active charger when needed */
	if (!route)
		goto done;

	chg_psy = gcpm_chg_get_active(gcpm);
	if (chg_psy) {
		ret = power_supply_set_property(chg_psy, psp, pval);
		if (ret < 0 && ret != -EAGAIN) {
			pr_err("cannot route prop=%d to %d:%s (%d)\n", psp,
				gcpm->chg_psy_active, gcpm_psy_name(chg_psy),
				ret);
		}
	} else {
		pr_err("invalid active charger = %d for prop=%d\n",
			gcpm->chg_psy_active, psp);
	}

done:
	/* the charger should not call into gcpm: this can change though */
	mutex_unlock(&gcpm->chg_psy_lock);
	return ret;
}

static int gcpm_psy_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *pval)
{
	struct gcpm_drv *gcpm = power_supply_get_drvdata(psy);
	union gbms_charger_state chg_state;
	struct power_supply *chg_psy;
	bool route = false;
	int ret = 0;

	pm_runtime_get_sync(gcpm->device);
	if (!gcpm->init_complete || !gcpm->resume_complete) {
		pm_runtime_put_sync(gcpm->device);
		return -EAGAIN;
	}
	pm_runtime_put_sync(gcpm->device);

	mutex_lock(&gcpm->chg_psy_lock);
	chg_psy = gcpm_chg_get_active(gcpm);
	if (!chg_psy) {
		pr_err("invalid active charger = %d for prop=%d\n",
			gcpm->chg_psy_active, psp);
		mutex_unlock(&gcpm->chg_psy_lock);
		return -ENODEV;
	}

	switch (psp) {
	/* handle locally for now */
	case GBMS_PROP_CHARGE_CHARGER_STATE:
		chg_state.v = gcpm_get_charger_state(gcpm, chg_psy);
		gbms_propval_int64val(pval) = chg_state.v;
		break;

	/* route to the active charger */
	default:
		route = true;
		break;
	}

	if (route)
		ret = power_supply_get_property(chg_psy, psp, pval);

	mutex_unlock(&gcpm->chg_psy_lock);
	return ret;
}

static int gcpm_psy_is_writeable(struct power_supply *psy,
				 enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case GBMS_PROP_CHARGE_DISABLE:
	case GBMS_PROP_TAPER_CONTROL:
		return 1;
	default:
		break;
	}

	return 0;
}

/*
 * TODO: POWER_SUPPLY_PROP_RERUN_AICL, POWER_SUPPLY_PROP_TEMP
 */
static enum power_supply_property gcpm_psy_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	/* pixel battery management subsystem */
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,	/* cc_max */
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,	/* fv_uv */
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,	/* input current limit */
	POWER_SUPPLY_PROP_VOLTAGE_MAX,	/* set float voltage, compat */
	POWER_SUPPLY_PROP_STATUS,
};

static struct power_supply_desc gcpm_psy_desc = {
	.name = "gcpm",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.get_property = gcpm_psy_get_property,
	.set_property = gcpm_psy_set_property,
	.property_is_writeable = gcpm_psy_is_writeable,
	.properties = gcpm_psy_properties,
	.num_properties = ARRAY_SIZE(gcpm_psy_properties),
};

#define gcpm_psy_changed_tickle_pps(gcpm) \
	((gcpm)->dc_state == DC_PASSTHROUGH || (gcpm)->dc_state == DC_RUNNING)

static int gcpm_psy_changed(struct notifier_block *nb, unsigned long action,
			    void *data)
{
	struct gcpm_drv *gcpm = container_of(nb, struct gcpm_drv, chg_nb);
	const int index = gcpm->chg_psy_active;
	struct power_supply *psy = data;
	bool tickle_pps_work = false;

	if (index == -1)
		return NOTIFY_OK;

	if ((action != PSY_EVENT_PROP_CHANGED) ||
	    (psy == NULL) || (psy->desc == NULL) || (psy->desc->name == NULL))
		return NOTIFY_OK;

	if (strcmp(psy->desc->name, gcpm->chg_psy_names[index]) == 0) {
		/* route upstream when the charger active and found */
		if (gcpm->chg_psy_avail[index])
			power_supply_changed(gcpm->psy);

		tickle_pps_work = gcpm_psy_changed_tickle_pps(gcpm);
	} else if (strcmp(psy->desc->name, gcpm->chg_psy_names[0]) == 0) {
		/* possibly JEITA or other violation, check PPS */
		tickle_pps_work = gcpm_psy_changed_tickle_pps(gcpm);
	} else if (gcpm->tcpm_psy_name &&
		   !strcmp(psy->desc->name, gcpm->tcpm_psy_name)) {

		/* from tcpm source (even if not selected) */
		tickle_pps_work = gcpm_psy_changed_tickle_pps(gcpm);
	} else if (gcpm->wlc_dc_name &&
	      !strcmp(psy->desc->name, gcpm->wlc_dc_name)) {

		/* from wc source (even if not selected) */
		tickle_pps_work = gcpm_psy_changed_tickle_pps(gcpm);
	}

	/* should tickle the PPS loop only when is running */
	if (tickle_pps_work)
		mod_delayed_work(system_wq, &gcpm->pps_work, 0);

	return NOTIFY_OK;
}

static ssize_t dc_limit_demand_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", gcpm->dc_limit_demand);
}
static ssize_t dc_limit_demand_store(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf, size_t count)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);
	int ret = 0;
	u32 val;

	ret = kstrtou32(buf, 0, &val);
	if (ret < 0)
		return ret;

	mutex_lock(&gcpm->chg_psy_lock);
	if (gcpm->dc_limit_demand != val) {
		gcpm->dc_limit_demand = val;
		gcpm->new_dc_limit = true;
	}

	mutex_unlock(&gcpm->chg_psy_lock);

	return count;
}
static DEVICE_ATTR_RW(dc_limit_demand);

static ssize_t dc_limit_vbatt_max_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", gcpm->dc_limit_vbatt_max);
}
static ssize_t dc_limit_vbatt_max_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);
	int ret = 0;
	u32 val;

	ret = kstrtou32(buf, 0, &val);
	if (ret < 0)
		return ret;

	gcpm->dc_limit_vbatt_max = val;

	return count;
}
static DEVICE_ATTR_RW(dc_limit_vbatt_max);

static ssize_t dc_limit_vbatt_min_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", gcpm->dc_limit_vbatt_min);
}
static ssize_t dc_limit_vbatt_min_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);
	int ret = 0;
	u32 val;

	ret = kstrtou32(buf, 0, &val);
	if (ret < 0)
		return ret;

	gcpm->dc_limit_vbatt_min = val;

	return count;
}
static DEVICE_ATTR_RW(dc_limit_vbatt_min);

static ssize_t dc_ctl_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", gcpm->dc_ctl);
}

static ssize_t dc_ctl_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct gcpm_drv *gcpm = dev_get_drvdata(dev);
	int ret = 0, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret < 0)
		return ret;

	/*
	 * 0: enable both (Default)
	 * 1: disable wired-DC
	 * 2: disable wireless-DC
	 * 3: disable both
	 */
	switch (val) {
		case GCPM_DC_CTL_DEFAULT:
		case GCPM_DC_CTL_DISABLE_WIRED:
		case GCPM_DC_CTL_DISABLE_WIRELESS:
		case GCPM_DC_CTL_DISABLE_BOTH:
			gcpm->dc_ctl = val;
			break;
		default:
			return -EINVAL;
	};

	return count;
}
static DEVICE_ATTR_RW(dc_ctl);

#define INIT_DELAY_MS 100
#define INIT_RETRY_DELAY_MS 1000

#define GCPM_TCPM_PSY_MAX 2

/*thiscan run */
static void gcpm_init_work(struct work_struct *work)
{
	struct gcpm_drv *gcpm = container_of(work, struct gcpm_drv,
					     init_work.work);
	int i, found = 0, ret = 0;
	bool dc_not_done;

	/* might run along set_property() */
	mutex_lock(&gcpm->chg_psy_lock);

	/*
	 * could call pps_init() in probe() and use lazy init for ->tcpm_psy
	 * when the device an APDO in the sink capabilities.
	 */
	if (gcpm->tcpm_phandle && !gcpm->tcpm_psy) {
		struct power_supply *tcpm_psy;

		tcpm_psy = pps_get_tcpm_psy(gcpm->device->of_node,
					    GCPM_TCPM_PSY_MAX);
		if (!IS_ERR_OR_NULL(tcpm_psy)) {
			const char *name = tcpm_psy->desc->name;

			gcpm->tcpm_psy_name = name;
			gcpm->tcpm_psy = tcpm_psy;

			/* PPS charging: needs an APDO */
			ret = pps_init(&gcpm->tcpm_pps_data, gcpm->device,
				       gcpm->tcpm_psy);
			if (ret == 0 && gcpm->debug_entry)
				pps_init_fs(&gcpm->tcpm_pps_data, gcpm->debug_entry);
			if (ret < 0) {
				pr_err("PPS init failure for %s (%d)\n",
				       name, ret);
			} else {
				gcpm->tcpm_pps_data.port_data =
					power_supply_get_drvdata(tcpm_psy);
				pps_init_state(&gcpm->tcpm_pps_data);
				pps_set_logbuffer(&gcpm->tcpm_pps_data, gcpm->log);
				pps_log(&gcpm->tcpm_pps_data, "TCPM_PPS for %s", gcpm->tcpm_psy_name);
			}

		} else if (!tcpm_psy || !gcpm->log_psy_ratelimit) {
			/* abort on an error */
			pr_warn("PPS not available for tcpm\n");
			gcpm->tcpm_phandle = 0;
		} else {
			pr_warn("tcpm power supply not found, retrying... ret:%d\n",
				ret);
			gcpm->log_psy_ratelimit--;
		}

	}

	/* TODO: lookup by phandle as the dude above */
	if (gcpm->wlc_dc_name && !gcpm->wlc_dc_psy) {
		struct power_supply *wlc_dc_psy;

		wlc_dc_psy = power_supply_get_by_name(gcpm->wlc_dc_name);
		if (wlc_dc_psy) {
			const char *name = gcpm->wlc_dc_name;

			gcpm->wlc_dc_psy = wlc_dc_psy;

			/* PPS charging: needs an APDO */
			ret = pps_init(&gcpm->wlc_pps_data, gcpm->device,
					gcpm->wlc_dc_psy);
			if (ret == 0 && gcpm->debug_entry)
				pps_init_fs(&gcpm->wlc_pps_data, gcpm->debug_entry);
			if (ret < 0) {
				pr_err("PPS init failure for %s (%d)\n",
				       name, ret);
			} else {
				gcpm->wlc_pps_data.port_data = NULL;
				pps_init_state(&gcpm->wlc_pps_data);
				pps_set_logbuffer(&gcpm->wlc_pps_data, gcpm->log);
				pps_log(&gcpm->wlc_pps_data, "WLC_PPS for %s", gcpm->wlc_dc_name);
			}

		} else if (!gcpm->log_psy_ratelimit) {
			/* give up if wlc_dc_psy return an error */
			pr_warn("PPS not available for %s\n", gcpm->wlc_dc_name);
			gcpm->wlc_dc_name = NULL;
		} else {
			pr_warn("%s power supply not found, retrying... ret:%d\n",
				gcpm->wlc_dc_name, ret);
			gcpm->log_psy_ratelimit--;
		}
	}

	/* default is index 0 */
	for (i = 0; i < gcpm->chg_psy_count; i++) {
		if (!gcpm->chg_psy_avail[i]) {
			const char *name = gcpm->chg_psy_names[i];

			gcpm->chg_psy_avail[i] = power_supply_get_by_name(name);
			if (gcpm->chg_psy_avail[i])
				pr_info("init_work found %d:%s\n", i, name);
		}

		found += !!gcpm->chg_psy_avail[i];
	}

	/* sort of done when we have the primary, make it online */
	if (gcpm->chg_psy_avail[0] && !gcpm->init_complete) {
		struct power_supply *def_psy = gcpm->chg_psy_avail[0];

		gcpm->chg_nb.notifier_call = gcpm_psy_changed;
		ret = power_supply_reg_notifier(&gcpm->chg_nb);
		if (ret < 0)
			pr_err("%s: no ps notifier, ret=%d\n", __func__, ret);

		ret = gcpm_enable_default(gcpm);
		if (ret < 0)
			pr_err("%s: default %s not online, ret=%d\n", __func__,
			       gcpm_psy_name(def_psy), ret);

		/* this is the reason why we need a lock here */
		gcpm->resume_complete = true;
		gcpm->init_complete = true;
	}

	/* keep looking for late arrivals, TCPM and WLC if set */
	if (found == gcpm->chg_psy_count)
		gcpm->chg_psy_retries = 0;
	else if (gcpm->chg_psy_retries)
		gcpm->chg_psy_retries--;

	dc_not_done = (gcpm->tcpm_phandle && !gcpm->tcpm_psy) ||
		      (gcpm->wlc_dc_name && !gcpm->wlc_dc_psy);

	pr_warn("%s retries=%d dc_not_done=%d tcpm_ok=%d wlc_ok=%d\n",
		__func__, gcpm->chg_psy_retries, dc_not_done,
		(!gcpm->tcpm_phandle || gcpm->tcpm_psy),
		(!gcpm->wlc_dc_name || gcpm->wlc_dc_psy));

	if (gcpm->chg_psy_retries || dc_not_done) {
		const unsigned long jif = msecs_to_jiffies(INIT_RETRY_DELAY_MS);

		schedule_delayed_work(&gcpm->init_work, jif);
	} else {
		pr_info("google_cpm init_work done %d/%d pps=%d wlc_dc=%d\n",
			found, gcpm->chg_psy_count,
			!!gcpm->tcpm_psy, !!gcpm->wlc_dc_psy);

		gcpm->dc_init_complete = true;
	}

	/* might run along set_property() */
	mutex_unlock(&gcpm->chg_psy_lock);

	if (gcpm->dc_init_complete)
		mod_delayed_work(system_wq, &gcpm->select_work, 0);
}

/* ------------------------------------------------------------------------ */

static int gcpm_debug_get_active(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->dc_index;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_set_active(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	int intval = (int)val;

	if (gcpm->force_active != -1 && val == gcpm->force_active)
		intval = -1;

	pr_info("%s: val=%llu val=%lld intval=%d\n", __func__, val, val, intval);

	if (intval != -1 && (intval < 0 || intval >= gcpm->chg_psy_count))
		return -ERANGE;
	if (intval != -1 && !gcpm_chg_get_charger(gcpm, intval))
		return -EINVAL;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->force_active = intval;
	mod_delayed_work(system_wq, &gcpm->select_work, 0);
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_active_fops, gcpm_debug_get_active,
			gcpm_debug_set_active, "%lld\n");

static int gcpm_debug_dc_limit_demand_show(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	*val = gcpm->dc_limit_demand;
	return 0;
}

static int gcpm_debug_dc_limit_demand_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = val;

	mutex_lock(&gcpm->chg_psy_lock);
	if (gcpm->dc_limit_demand != intval) {
		gcpm->dc_limit_demand = intval;
		gcpm->new_dc_limit = true;
	}

	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}


DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_dc_limit_demand_fops,
			gcpm_debug_dc_limit_demand_show,
                        gcpm_debug_dc_limit_demand_set,
			"%llu\n");


static int gcpm_debug_pps_stage_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;
	struct pd_pps_data *pps_data;

	mutex_lock(&gcpm->chg_psy_lock);
	pps_data = gcpm_pps_data(gcpm);
	if (pps_data)
		*val = pps_data->stage;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_pps_stage_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;
	struct pd_pps_data *pps_data;

	if (intval < PPS_DISABLED || intval > PPS_ACTIVE)
		return -EINVAL;

	mutex_lock(&gcpm->chg_psy_lock);
	pps_data = gcpm_pps_data(gcpm);
	if (pps_data)
		pps_data->stage = intval;
	gcpm->force_pps = !pps_is_disabled(intval);
	mod_delayed_work(system_wq, &gcpm->pps_work, 0);
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_pps_stage_fops, gcpm_debug_pps_stage_get,
			gcpm_debug_pps_stage_set, "%llu\n");

static int gcpm_debug_dc_state_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->dc_state;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_dc_state_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;

	if (intval < DC_DISABLED || intval > DC_PASSTHROUGH)
		return -EINVAL;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->dc_state = intval;
	mod_delayed_work(system_wq, &gcpm->select_work, 0);
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_dc_state_fops, gcpm_debug_dc_state_get,
			gcpm_debug_dc_state_set, "%llu\n");


static int gcpm_debug_taper_ctl_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->taper_step;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_taper_ctl_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	bool ta_check;

	mutex_lock(&gcpm->chg_psy_lock);

	/* ta_check set when taper control changes value */
	ta_check = gcpm_taper_ctl(gcpm, val);
	if (ta_check)
		mod_delayed_work(system_wq, &gcpm->select_work, 0);
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_taper_ctl_fops, gcpm_debug_taper_ctl_get,
			gcpm_debug_taper_ctl_set, "%llu\n");

static int gcpm_debug_taper_step_fv_margin_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->taper_step_fv_margin;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_taper_step_fv_margin_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->taper_step_fv_margin = intval;
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_taper_step_fv_margin_fops, gcpm_debug_taper_step_fv_margin_get,
			gcpm_debug_taper_step_fv_margin_set, "%llu\n");

static int gcpm_debug_taper_step_cc_step_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->taper_step_cc_step;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_taper_step_cc_step_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->taper_step_cc_step = intval;
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_taper_step_cc_step_fops, gcpm_debug_taper_step_cc_step_get,
			gcpm_debug_taper_step_cc_step_set, "%llu\n");

static int gcpm_debug_taper_step_count_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->taper_step_count;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_taper_step_count_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->taper_step_count = intval;
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_taper_step_count_fops, gcpm_debug_taper_step_count_get,
			gcpm_debug_taper_step_count_set, "%llu\n");

static int gcpm_debug_taper_step_grace_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->taper_step_grace;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_taper_step_grace_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->taper_step_grace = intval;
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_taper_step_grace_fops, gcpm_debug_taper_step_grace_get,
			gcpm_debug_taper_step_grace_set, "%llu\n");

static int gcpm_debug_taper_step_voltage_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->taper_step_voltage;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_taper_step_voltage_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->taper_step_voltage = intval;
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_taper_step_voltage_fops, gcpm_debug_taper_step_voltage_get,
			gcpm_debug_taper_step_voltage_set, "%llu\n");

static int gcpm_debug_taper_step_current_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->taper_step_current;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_taper_step_current_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->taper_step_current = intval;
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_taper_step_current_fops, gcpm_debug_taper_step_current_get,
			gcpm_debug_taper_step_current_set, "%llu\n");

static int gcpm_debug_taper_step_interval_get(void *data, u64 *val)
{
	struct gcpm_drv *gcpm = data;

	mutex_lock(&gcpm->chg_psy_lock);
	*val = gcpm->taper_step_interval;
	mutex_unlock(&gcpm->chg_psy_lock);
	return 0;
}

static int gcpm_debug_taper_step_interval_set(void *data, u64 val)
{
	struct gcpm_drv *gcpm = data;
	const int intval = (int)val;

	mutex_lock(&gcpm->chg_psy_lock);
	gcpm->taper_step_interval = intval;
	mutex_unlock(&gcpm->chg_psy_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(gcpm_debug_taper_step_interval_fops, gcpm_debug_taper_step_interval_get,
			gcpm_debug_taper_step_interval_set, "%llu\n");

static struct dentry *gcpm_init_fs(struct gcpm_drv *gcpm)
{
	struct dentry *de;

	de = debugfs_create_dir("google_cpm", 0);
	if (IS_ERR_OR_NULL(de))
		return NULL;

	debugfs_create_file("dc_state", 0644, de, gcpm, &gcpm_debug_dc_state_fops);
	debugfs_create_file("active", 0644, de, gcpm, &gcpm_debug_active_fops);
	debugfs_create_file("dc_limit_demand", 0644, de, gcpm,
			    &gcpm_debug_dc_limit_demand_fops);
	debugfs_create_file("pps_stage", 0644, de, gcpm, &gcpm_debug_pps_stage_fops);
	debugfs_create_file("taper_ctl", 0644, de, gcpm, &gcpm_debug_taper_ctl_fops);
	debugfs_create_file("taper_step_fv_margin", 0644, de, gcpm, &gcpm_debug_taper_step_fv_margin_fops);
	debugfs_create_file("taper_step_cc_step", 0644, de, gcpm, &gcpm_debug_taper_step_cc_step_fops);
	debugfs_create_file("taper_step_count", 0644, de, gcpm, &gcpm_debug_taper_step_count_fops);
	debugfs_create_file("taper_step_grace", 0644, de, gcpm, &gcpm_debug_taper_step_grace_fops);
	debugfs_create_file("taper_step_voltage", 0644, de, gcpm, &gcpm_debug_taper_step_voltage_fops);
	debugfs_create_file("taper_step_current", 0644, de, gcpm, &gcpm_debug_taper_step_current_fops);
	debugfs_create_file("taper_step_interval", 0644, de, gcpm, &gcpm_debug_taper_step_interval_fops);

	return de;
}

/* ------------------------------------------------------------------------ */

static int gcpm_probe_psy_names(struct gcpm_drv *gcpm)
{
	struct device *dev = gcpm->device;
	int i, count, ret;

	if (!gcpm->device)
		return -EINVAL;

	count = of_property_count_strings(dev->of_node,
					  "google,chg-power-supplies");
	if (count <= 0 || count > GCPM_MAX_CHARGERS)
		return -ERANGE;

	ret = of_property_read_string_array(dev->of_node,
					    "google,chg-power-supplies",
					    (const char**)&gcpm->chg_psy_names,
					    count);
	if (ret != count)
		return -ERANGE;

	for (i = 0; i < count; i++)
		dev_info(gcpm->device, "%d:%s\n", i, gcpm->chg_psy_names[i]);

	return count;
}

/* -------------------------------------------------------------------------
 *  Use to abstract the PPS adapter if needed.
 */

static int gcpm_pps_psy_set_property(struct power_supply *psy,
				    enum power_supply_property prop,
				    const union power_supply_propval *val)
{
	struct gcpm_drv *gcpm = power_supply_get_drvdata(psy);
	struct pd_pps_data *pps_data;
	int ret = 0;

	mutex_lock(&gcpm->chg_psy_lock);

	pps_data = gcpm_pps_data(gcpm);
	if (!pps_data || !pps_data->pps_psy) {
		pr_debug("%s: no target prop=%d ret=%d\n", __func__, prop, ret);
		mutex_unlock(&gcpm->chg_psy_lock);
		return -EAGAIN;
	}

	switch (prop) {
	default:
		ret = power_supply_set_property(pps_data->pps_psy, prop, val);
		break;
	}

	mutex_unlock(&gcpm->chg_psy_lock);
	pr_debug("%s: prop=%d val=%d ret=%d\n", __func__,
		 prop, val->intval, ret);

	return ret;
}

static int gcpm_pps_psy_get_property(struct power_supply *psy,
				    enum power_supply_property prop,
				    union power_supply_propval *val)
{
	struct gcpm_drv *gcpm = power_supply_get_drvdata(psy);
	struct pd_pps_data *pps_data;
	int ret = 0;

	mutex_lock(&gcpm->chg_psy_lock);

	pps_data = gcpm_pps_data(gcpm);
	if (pps_data && pps_data->pps_psy) {
		ret = power_supply_get_property(pps_data->pps_psy, prop, val);
		pr_debug("%s: prop=%d val=%d ret=%d\n", __func__,
			 prop, val->intval, ret);
		goto done;
	}

	switch (prop) {
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		break;
	default:
		val->intval = 0;
		break;
	}

done:
	mutex_unlock(&gcpm->chg_psy_lock);
	return ret;
}

/* check pps_is_avail(), pps_prog_online() and pps_check_type() */
static enum power_supply_property gcpm_pps_psy_properties[] = {
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,	/* 17 */
	POWER_SUPPLY_PROP_ONLINE,	/* 4 */
	POWER_SUPPLY_PROP_PRESENT,	/* 3 */
	POWER_SUPPLY_PROP_TYPE,		/* */
	POWER_SUPPLY_PROP_USB_TYPE,	/* */
	POWER_SUPPLY_PROP_VOLTAGE_NOW,	/* */
};

static int gcpm_pps_psy_is_writeable(struct power_supply *psy,
				   enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_usb_type gcpm_pps_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_PD_PPS
};

static const struct power_supply_desc gcpm_pps_psy_desc = {
	.name		= "gcpm_pps",
	.type		= POWER_SUPPLY_TYPE_UNKNOWN,
	.get_property	= gcpm_pps_psy_get_property,
	.set_property 	= gcpm_pps_psy_set_property,
	.properties	= gcpm_pps_psy_properties,
	.property_is_writeable = gcpm_pps_psy_is_writeable,
	.num_properties	= ARRAY_SIZE(gcpm_pps_psy_properties),

	/* POWER_SUPPLY_PROP_USB_TYPE requires an array of these */
	.usb_types	= gcpm_pps_usb_types,
	.num_usb_types	= ARRAY_SIZE(gcpm_pps_usb_types),
};

/* ------------------------------------------------------------------------- */

#define LOG_PSY_RATELIMIT_CNT	200

static int google_cpm_probe(struct platform_device *pdev)
{
	struct power_supply_config gcpm_pps_psy_cfg = { 0 };
	struct power_supply_config psy_cfg = { 0 };
	const char *tmp_name = NULL;
	struct gcpm_drv *gcpm;
	int ret;

	gcpm = devm_kzalloc(&pdev->dev, sizeof(*gcpm), GFP_KERNEL);
	if (!gcpm)
		return -ENOMEM;

	gcpm->device = &pdev->dev;
	gcpm->force_active = -1;
	gcpm->dc_ctl = GCPM_DC_CTL_DEFAULT;
	gcpm->log_psy_ratelimit = LOG_PSY_RATELIMIT_CNT;
	gcpm->chg_psy_retries = 10; /* chg_psy_retries *  INIT_RETRY_DELAY_MS */
	gcpm->out_uv = -1;
	gcpm->out_ua = -1;
	gcpm->dc_fcc_limit = -1;

	INIT_DELAYED_WORK(&gcpm->pps_work, gcpm_pps_wlc_dc_work);
	INIT_DELAYED_WORK(&gcpm->select_work, gcpm_chg_select_work);
	INIT_DELAYED_WORK(&gcpm->init_work, gcpm_init_work);
	mutex_init(&gcpm->chg_psy_lock);

	/* this is my name */
	ret = of_property_read_string(pdev->dev.of_node, "google,psy-name",
				      &tmp_name);
	if (ret == 0) {
		gcpm_psy_desc.name = devm_kstrdup(&pdev->dev, tmp_name,
						  GFP_KERNEL);
		if (!gcpm_psy_desc.name)
			return -ENOMEM;
	}

	/* subs power supply names */
	gcpm->chg_psy_count = gcpm_probe_psy_names(gcpm);
	if (gcpm->chg_psy_count <= 0)
		return -ENODEV;

	/* DC/PPS needs at least one power supply of this type */
	ret = of_property_read_u32(pdev->dev.of_node,
				   "google,tcpm-power-supply",
				   &gcpm->tcpm_phandle);
	if (ret < 0)
		pr_warn("google,tcpm-power-supply not defined\n");

	ret = of_property_read_string(pdev->dev.of_node,
				      "google,wlc_dc-power-supply",
				      &tmp_name);
	if (ret == 0) {
		gcpm->wlc_dc_name = devm_kstrdup(&pdev->dev, tmp_name,
						     GFP_KERNEL);
		if (!gcpm->wlc_dc_name)
			return -ENOMEM;
	}

	/* GCPM might need a gpio to enable/disable DC/PPS */
	gcpm->dcen_gpio = of_get_named_gpio(pdev->dev.of_node, "google,dc-en", 0);
	if (gcpm->dcen_gpio >= 0) {
		unsigned long init_flags = GPIOF_OUT_INIT_LOW;

		of_property_read_u32(pdev->dev.of_node, "google,dc-en-value",
				     &gcpm->dcen_gpio_default);
		if (gcpm->dcen_gpio_default)
			init_flags = GPIOF_OUT_INIT_HIGH;

		ret = devm_gpio_request_one(&pdev->dev, gcpm->dcen_gpio,
					    init_flags, "dc_pu_pin");
		pr_info("google,dc-en value =%d ret=%d\n",
			gcpm->dcen_gpio_default, ret);
	}

	/* Triggers to enable dc charging */
	ret = of_property_read_u32(pdev->dev.of_node, "google,dc_limit-demand",
				   &gcpm->dc_limit_demand);
	if (ret < 0)
		gcpm->dc_limit_demand = GCPM_DEFAULT_DC_LIMIT_DEMAND;

	ret = of_property_read_u32(pdev->dev.of_node, "google,dc_limit-cc_min",
				   &gcpm->dc_limit_cc_min);
	if (ret < 0)
		gcpm->dc_limit_cc_min = GCPM_DEFAULT_DC_LIMIT_CC_MIN;

	ret = of_property_read_u32(pdev->dev.of_node, "google,dc_limit-cc_min_wlc",
				   &gcpm->dc_limit_cc_min);
	if (ret < 0)
		gcpm->dc_limit_cc_min_wlc = GCPM_DEFAULT_DC_LIMIT_CC_MIN_WLC;


	/* voltage lower bound */
	ret = of_property_read_u32(pdev->dev.of_node, "google,dc_limit-vbatt_min",
				   &gcpm->dc_limit_vbatt_min);
	if (ret < 0)
		gcpm->dc_limit_vbatt_min = GCPM_DEFAULT_DC_LIMIT_VBATT_MIN;
	ret = of_property_read_u32(pdev->dev.of_node, "google,dc_limit-vbatt_low",
				   &gcpm->dc_limit_vbatt_low);
	if (ret < 0)
		gcpm->dc_limit_vbatt_low = gcpm->dc_limit_vbatt_min -
					   GCPM_DEFAULT_DC_LIMIT_DELTA_LOW;
	if (gcpm->dc_limit_vbatt_low > gcpm->dc_limit_vbatt_min)
		gcpm->dc_limit_vbatt_low = gcpm->dc_limit_vbatt_min;

	/* voltage upper bound */
	ret = of_property_read_u32(pdev->dev.of_node, "google,dc_limit-vbatt_max",
				   &gcpm->dc_limit_vbatt_max);
	if (ret < 0)
		gcpm->dc_limit_vbatt_max = GCPM_DEFAULT_DC_LIMIT_VBATT_MAX;
	ret = of_property_read_u32(pdev->dev.of_node, "google,dc_limit-vbatt_high",
				   &gcpm->dc_limit_vbatt_high);
	if (ret < 0)
		gcpm->dc_limit_vbatt_high = gcpm->dc_limit_vbatt_max -
					    GCPM_DEFAULT_DC_LIMIT_DELTA_HIGH;
	if (gcpm->dc_limit_vbatt_high > gcpm->dc_limit_vbatt_max)
		gcpm->dc_limit_vbatt_high = gcpm->dc_limit_vbatt_max;

	/* taper control */
	gcpm->taper_step = -1;
	ret = of_property_read_u32(pdev->dev.of_node, "google,taper_step-fv-margin",
				   &gcpm->taper_step_fv_margin);
	if (ret < 0)
		gcpm->taper_step_fv_margin = GCPM_TAPER_STEP_FV_MARGIN;
	ret = of_property_read_u32(pdev->dev.of_node, "google,taper_step-cc-step",
				   &gcpm->taper_step_cc_step);
	if (ret < 0)
		gcpm->taper_step_cc_step = GCPM_TAPER_STEP_CC_STEP;
	ret = of_property_read_u32(pdev->dev.of_node, "google,taper_step-interval",
				   &gcpm->taper_step_interval);
	if (ret < 0)
		gcpm->taper_step_interval = GCPM_TAPER_STEP_INTERVAL_S;

	ret = of_property_read_u32(pdev->dev.of_node, "google,taper_step-count",
				   &gcpm->taper_step_count);
	if (ret < 0)
		gcpm->taper_step_count = GCPM_TAPER_STEP_COUNT;
	ret = of_property_read_u32(pdev->dev.of_node, "google,taper_step-grace",
				   &gcpm->taper_step_grace);
	if (ret < 0)
		gcpm->taper_step_grace = GCPM_TAPER_STEP_GRACE;
	ret = of_property_read_u32(pdev->dev.of_node, "google,taper_step-voltage",
				   &gcpm->taper_step_voltage);
	if (ret < 0)
		gcpm->taper_step_voltage = GCPM_TAPER_STEP_VOLTAGE;
	ret = of_property_read_u32(pdev->dev.of_node, "google,taper_step-current",
				   &gcpm->taper_step_current);
	if (ret < 0)
		gcpm->taper_step_current = GCPM_TAPER_STEP_CURRENT;

	dev_err(gcpm->device, "ts_m=%d ts_ccs=%d ts_i=%d ts_cnt=%d ts_g=%d ts_v=%d ts_c=%d\n",
		gcpm->taper_step_fv_margin, gcpm->taper_step_cc_step,
		gcpm->taper_step_interval, gcpm->taper_step_count,
		gcpm->taper_step_grace, gcpm->taper_step_voltage,
		gcpm->taper_step_current);

	/* Wireless charging, DC name is for compat */
	gcpm->dc_fcc_votable =
		gvotable_create_int_election(NULL, gvotable_comparator_int_min,
					     gcpm_dc_fcc_callback, gcpm);
	if (IS_ERR_OR_NULL(gcpm->dc_fcc_votable)) {
		ret = PTR_ERR(gcpm->dc_fcc_votable);
		dev_err(gcpm->device, "no dc_fcc votable (%d)\n", ret);
		return ret;
	}

	gvotable_set_default(gcpm->dc_fcc_votable, (void *)-1);
	gvotable_set_vote2str(gcpm->dc_fcc_votable, gvotable_v2s_int);
	gvotable_election_set_name(gcpm->dc_fcc_votable, "DC_FCC");

	/* sysfs & debug */
	gcpm->debug_entry = gcpm_init_fs(gcpm);
	if (!gcpm->debug_entry)
		pr_warn("No debug control\n");

	platform_set_drvdata(pdev, gcpm);

	psy_cfg.drv_data = gcpm;
	psy_cfg.of_node = pdev->dev.of_node;
	gcpm->psy = devm_power_supply_register(gcpm->device,
					       &gcpm_psy_desc,
					       &psy_cfg);
	if (IS_ERR(gcpm->psy)) {
		ret = PTR_ERR(gcpm->psy);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		dev_err(gcpm->device, "Couldn't register gcpm, (%d)\n", ret);
		return -ENODEV;
	}

	gcpm->log = logbuffer_register("cpm");
	if (IS_ERR(gcpm->log)) {
		dev_err(gcpm->device, "Couldn't register logbuffer, (%ld)\n",
			PTR_ERR(gcpm->log));
		gcpm->log = NULL;
	}

	/* gcpm_pps_psy_cfg.of_node is used to find out the snk_pdos */
	gcpm_pps_psy_cfg.drv_data = gcpm;
	gcpm_pps_psy_cfg.of_node = pdev->dev.of_node;
	gcpm->pps_psy = devm_power_supply_register(gcpm->device,
						   &gcpm_pps_psy_desc,
						   &gcpm_pps_psy_cfg);
	if (IS_ERR(gcpm->pps_psy)) {
		ret = PTR_ERR(gcpm->pps_psy);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		dev_err(gcpm->device, "Couldn't register gcpm_pps (%d)\n", ret);
		return -ENODEV;
	}

	ret = device_create_file(gcpm->device, &dev_attr_dc_limit_demand);
	if (ret)
		dev_err(gcpm->device, "Failed to create dc_limit_demand\n");

	ret = device_create_file(gcpm->device, &dev_attr_dc_limit_vbatt_max);
	if (ret)
		dev_err(gcpm->device, "Failed to create dc_limit_vbatt_max\n");

	ret = device_create_file(gcpm->device, &dev_attr_dc_limit_vbatt_min);
	if (ret)
		dev_err(gcpm->device, "Failed to create dc_limit_vbatt_min\n");

	ret = device_create_file(gcpm->device, &dev_attr_dc_ctl);
	if (ret)
		dev_err(gcpm->device, "Failed to create dc_crl\n");

	/* give time to fg driver to start */
	schedule_delayed_work(&gcpm->init_work,
			      msecs_to_jiffies(INIT_DELAY_MS));

	return 0;
}

static int google_cpm_remove(struct platform_device *pdev)
{
	struct gcpm_drv *gcpm = platform_get_drvdata(pdev);
	int i;

	if (!gcpm)
		return 0;

	gvotable_destroy_election(gcpm->dc_fcc_votable);

	for (i = 0; i < gcpm->chg_psy_count; i++) {
		if (!gcpm->chg_psy_avail[i])
			continue;

		power_supply_put(gcpm->chg_psy_avail[i]);
		gcpm->chg_psy_avail[i] = NULL;
	}

	pps_free(&gcpm->wlc_pps_data);
	pps_free(&gcpm->tcpm_pps_data);

	if (gcpm->wlc_dc_psy)
		power_supply_put(gcpm->wlc_dc_psy);
	if (gcpm->log)
		logbuffer_unregister(gcpm->log);

	return 0;
}

static const struct of_device_id google_cpm_of_match[] = {
	{.compatible = "google,cpm"},
	{},
};
MODULE_DEVICE_TABLE(of, google_cpm_of_match);


static struct platform_driver google_cpm_driver = {
	.driver = {
		   .name = "google_cpm",
		   .owner = THIS_MODULE,
		   .of_match_table = google_cpm_of_match,
		   .probe_type = PROBE_PREFER_ASYNCHRONOUS,
#ifdef SUPPORT_PM_SLEEP
		   /* .pm = &gcpm_pm_ops, */
#endif
		   },
	.probe = google_cpm_probe,
	.remove = google_cpm_remove,
};

module_platform_driver(google_cpm_driver);

MODULE_DESCRIPTION("Google Charging Policy Manager");
MODULE_AUTHOR("AleX Pelosi <apelosi@google.com>");
MODULE_LICENSE("GPL");

#if 0

/* NOTE: call with a lock around gcpm->chg_psy_lock */
static int gcpm_dc_charging(struct gcpm_drv *gcpm)
{
	struct power_supply *dc_psy;
	int vchg, ichg, status;

	dc_psy = gcpm_chg_get_active(gcpm);
	if (!dc_psy) {
		pr_err("DC_CHG: invalid charger\n");
		return -ENODEV;
	}

	vchg = GPSY_GET_PROP(dc_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	ichg = GPSY_GET_PROP(dc_psy, POWER_SUPPLY_PROP_CURRENT_NOW);
	status = GPSY_GET_PROP(dc_psy, POWER_SUPPLY_PROP_STATUS);

	pr_err("DC_CHG: vchg=%d, ichg=%d status=%d\n",
	       vchg, ichg, status);

	return 0;
}

static void gcpm_pps_dc_charging(struct gcpm_drv *gcpm)
{
	struct pd_pps_data *pps_data = &gcpm->pps_data;
	struct power_supply *pps_psy = gcpm->tcpm_psy;
	const int pre_out_ua = pps_data->op_ua;
	const int pre_out_uv = pps_data->out_uv;
	int ret, pps_ui = -ENODEV;

	if (gcpm->dc_state == DC_ENABLE) {
		struct pd_pps_data *pps_data = &gcpm->pps_data;
		bool pwr_ok;

		/* must run at the end of PPS negotiation */
		if (gcpm->out_ua == -1)
			gcpm->out_ua = min(gcpm->cc_max, pps_data->max_ua);
		if (gcpm->out_uv == -1) {
			struct power_supply *chg_psy =
						gcpm_chg_get_active(gcpm);
			unsigned long ta_max_v, value;
			int vbatt = -1;

			ta_max_v = pps_data->max_ua * pps_data->max_uv;
			ta_max_v /= gcpm->out_ua;
			if (ta_max_v > DC_TA_VMAX_MV)
				ta_max_v = DC_TA_VMAX_MV;

			if (chg_psy)
				vbatt = GPSY_GET_PROP(chg_psy,
						POWER_SUPPLY_PROP_VOLTAGE_NOW);
			if (vbatt < 0)
				vbatt = gcpm->fv_uv;
			if (vbatt < 0)
				vbatt = 0;

			/* good for pca9468 */
			value = 2 * vbatt + DC_VBATT_HEADROOM_MV;
			if (value < DC_TA_VMIN_MV)
				value = DC_TA_VMIN_MV;

			/* PPS voltage in 20mV steps */
			gcpm->out_uv = value - value % 20000;
		}

		pr_info("CHG_CHK: max_uv=%d,max_ua=%d  out_uv=%d,out_ua=%d\n",
			pps_data->max_uv, pps_data->max_ua,
			gcpm->out_uv, gcpm->out_ua);

		pps_ui = pps_update_adapter(pps_data, gcpm->out_uv,
					    gcpm->out_ua, pps_psy);
		if (pps_ui < 0)
			pps_ui = PPS_ERROR_RETRY_MS;

		/* wait until adapter is at or over request */
		pwr_ok = pps_data->out_uv == gcpm->out_uv &&
				pps_data->op_ua == gcpm->out_ua;
		if (pwr_ok) {
			ret = gcpm_chg_offline(gcpm);
			if (ret == 0)
				ret = gcpm_dc_start(gcpm, gcpm->dc_index);
			if (ret == 0) {
				gcpm->dc_state = DC_RUNNING;
				pps_ui = DC_RUN_DELAY_MS;
			}  else if (pps_ui > DC_ERROR_RETRY_MS) {
				pps_ui = DC_ERROR_RETRY_MS;
			}
		}

		/*
			* TODO: add retries and switch to DC_ENABLE again or to
			* DC_DISABLED on timeout.
			*/

		pr_info("PPS_DC: dc_state=%d out_uv=%d %d->%d, out_ua=%d %d->%d\n",
			gcpm->dc_state,
			pps_data->out_uv, pre_out_uv, gcpm->out_uv,
			pps_data->op_ua, pre_out_ua, gcpm->out_ua);
	} else if (gcpm->dc_state == DC_RUNNING)  {

		ret = gcpm_chg_ping(gcpm, 0, 0);
		if (ret < 0)
			pr_err("PPS_DC: ping failed with %d\n", ret);

		/* update gcpm->out_uv, gcpm->out_ua */
		pr_info("PPS_DC: dc_state=%d out_uv=%d %d->%d out_ua=%d %d->%d\n",
			gcpm->dc_state,
			pps_data->out_uv, pre_out_uv, gcpm->out_uv,
			pps_data->op_ua, pre_out_ua, gcpm->out_ua);

		ret = gcpm_dc_charging(gcpm);
		if (ret < 0)
			pps_ui = DC_ERROR_RETRY_MS;

		ret = pps_update_adapter(&gcpm->pps_data,
						gcpm->out_uv, gcpm->out_ua,
						pps_psy);
		if (ret < 0)
			pps_ui = PPS_ERROR_RETRY_MS;
	}

	return pps_ui;
}
#endif
