// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based Samsung common panel driver.
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_encoder.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>
#include <video/mipi_display.h>

#include <trace/dpu_trace.h>
#include "../exynos_drm_connector.h"
#include "panel-samsung-drv.h"

#define PANEL_ID_REG		0xA1
#define PANEL_ID_LEN		7
#define PANEL_ID_OFFSET		6
#define PANEL_ID_READ_SIZE	(PANEL_ID_LEN + PANEL_ID_OFFSET)

static const char ext_info_regs[] = { 0xDA, 0xDB, 0xDC };
#define EXT_INFO_SIZE ARRAY_SIZE(ext_info_regs)

#define exynos_connector_to_panel(c)					\
	container_of((c), struct exynos_panel, exynos_connector)

#define bridge_to_exynos_panel(b) \
	container_of((b), struct exynos_panel, bridge)

static void exynos_panel_set_backlight_state(struct exynos_panel *ctx,
					enum exynos_panel_state panel_state);
static ssize_t exynos_panel_parse_byte_buf(char *input_str, size_t input_len,
					   const char **out_buf);
static int parse_u32_buf(char *src, size_t src_len, u32 *out, size_t out_len);

static inline bool is_backlight_off_state(const struct backlight_device *bl)
{
	return (bl->props.state & BL_STATE_STANDBY) != 0;
}

static inline bool is_backlight_lp_state(const struct backlight_device *bl)
{
	return (bl->props.state & BL_STATE_LP) != 0;
}

int exynos_panel_configure_te2_edges(struct exynos_panel *ctx,
				     u32 *timings, bool lp_mode)
{
	struct te2_mode_data *data;
	const u32 *t;
	int i;

	if (!ctx || !timings)
		return -EINVAL;

	t = timings;

	for_each_te2_timing(ctx, lp_mode, data, i) {
		data->timing.rising_edge = t[0];
		data->timing.falling_edge = t[1];
		t += 2;
	}

	return 0;
}
EXPORT_SYMBOL(exynos_panel_configure_te2_edges);

ssize_t exynos_panel_get_te2_edges(struct exynos_panel *ctx,
				   char *buf, bool lp_mode)
{
	struct te2_mode_data *data;
	size_t len = 0;
	int i;

	if (!ctx)
		return -EINVAL;

	for_each_te2_timing(ctx, lp_mode, data, i) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%dx%d@%d",
				 data->mode->hdisplay, data->mode->vdisplay,
				 drm_mode_vrefresh(data->mode));

		if (data->binned_lp)
			len += scnprintf(buf + len, PAGE_SIZE - len, "-lp_%s",
					 data->binned_lp->name);

		len += scnprintf(buf + len, PAGE_SIZE - len,
				 " rising %u falling %u\n",
				 data->timing.rising_edge,
				 data->timing.falling_edge);
	}

	return len;
}
EXPORT_SYMBOL(exynos_panel_get_te2_edges);

int exynos_panel_get_current_mode_te2(struct exynos_panel *ctx,
				      struct exynos_panel_te2_timing *timing)
{
	struct te2_mode_data *data;
	const struct drm_display_mode *mode;
	u32 bl_th = 0;
	bool lp_mode;
	int i;

	if (!ctx)
		return -EINVAL;

	if (!ctx->current_mode)
		return -EAGAIN;

	mode = &ctx->current_mode->mode;
	lp_mode = ctx->current_mode->exynos_mode.is_lp_mode;

	if (lp_mode && !ctx->desc->num_binned_lp) {
		dev_warn(ctx->dev, "Missing LP mode command set\n");
		return -EINVAL;
	}

	if (lp_mode && !ctx->current_binned_lp)
		return -EAGAIN;

	if (ctx->current_binned_lp)
		bl_th = ctx->current_binned_lp->bl_threshold;

	for_each_te2_timing(ctx, lp_mode, data, i) {
		if (data->mode != mode)
			continue;

		if (data->binned_lp && data->binned_lp->bl_threshold != bl_th)
			continue;

		timing->rising_edge = data->timing.rising_edge;
		timing->falling_edge = data->timing.falling_edge;

		dev_dbg(ctx->dev,
			"found TE2 timing %s at %dHz: rising %u falling %u\n",
			!lp_mode ? "normal" : "LP", drm_mode_vrefresh(mode),
			timing->rising_edge, timing->falling_edge);

		return 0;
	}

	dev_warn(ctx->dev, "failed to find %s TE2 timing at %dHz\n",
		 !lp_mode ? "normal" : "LP", drm_mode_vrefresh(mode));

	return -EINVAL;
}
EXPORT_SYMBOL(exynos_panel_get_current_mode_te2);

static void exynos_panel_update_te2(struct exynos_panel *ctx)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;

	if (!ctx->initialized || !ctx->enabled || !funcs || !funcs->update_te2)
		return;

	funcs->update_te2(ctx);
}

static int exynos_panel_parse_gpios(struct exynos_panel *ctx)
{
	struct device *dev = ctx->dev;

	dev_dbg(ctx->dev, "%s +\n", __func__);

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR)) {
		dev_info(ctx->dev, "no reset/enable pins on emulator\n");
		return 0;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "failed to get reset-gpios %ld",
				PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	ctx->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enable_gpio))
		ctx->enable_gpio = NULL;

	dev_dbg(ctx->dev, "%s -\n", __func__);
	return 0;
}

static int exynos_panel_parse_regulators(struct exynos_panel *ctx)
{
	struct device *dev = ctx->dev;
	struct regulator *reg;
	int ret;

	ctx->vddi = devm_regulator_get(dev, "vddi");
	if (IS_ERR(ctx->vddi)) {
		dev_warn(ctx->dev, "failed to get panel vddi.\n");
		return -EPROBE_DEFER;
	}

	ctx->vci = devm_regulator_get(dev, "vci");
	if (IS_ERR(ctx->vci)) {
		dev_warn(ctx->dev, "failed to get panel vci.\n");
		return -EPROBE_DEFER;
	}

	reg = devm_regulator_get_optional(dev, "vddd");
	if (!PTR_ERR_OR_ZERO(reg)) {
		pr_info("panel vddd found\n");
		ctx->vddd = reg;
	}

	ret = of_property_read_u32(dev->of_node, "vddd-normal-microvolt", &ctx->vddd_normal_uV);
	if (ret)
		ctx->vddd_normal_uV = 0;

	ret = of_property_read_u32(dev->of_node, "vddd-lp-microvolt", &ctx->vddd_lp_uV);
	if (ret) {
		ctx->vddd_lp_uV = 0;
		if (ctx->vddd_normal_uV != 0) {
			pr_warn("ignore vddd normal %u\n", ctx->vddd_normal_uV);
			ctx->vddd_normal_uV = 0;
		}
	}

	reg = devm_regulator_get_optional(dev, "vddr_en");
	if (!PTR_ERR_OR_ZERO(reg)) {
		dev_dbg(ctx->dev, "panel vddr_en found\n");
		ctx->vddr_en = reg;
	}

	reg = devm_regulator_get_optional(dev, "vddr");
	if (!PTR_ERR_OR_ZERO(reg)) {
		dev_dbg(ctx->dev, "panel vddr found\n");
		ctx->vddr = reg;
	}

	return 0;
}

static int exynos_panel_read_id(struct exynos_panel *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	char buf[PANEL_ID_READ_SIZE];
	int ret;

	ret = mipi_dsi_dcs_read(dsi, PANEL_ID_REG, buf, PANEL_ID_READ_SIZE);
	if (ret != PANEL_ID_READ_SIZE) {
		dev_warn(ctx->dev, "Unable to read panel id (%d)\n", ret);
		return ret;
	}

	exynos_bin2hex(buf + PANEL_ID_OFFSET, PANEL_ID_LEN,
		       ctx->panel_id, sizeof(ctx->panel_id));

	return 0;
}

static int exynos_panel_read_extinfo(struct exynos_panel *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	char buf[EXT_INFO_SIZE];
	int i, ret;

	for (i = 0; i < EXT_INFO_SIZE; i++) {
		ret = mipi_dsi_dcs_read(dsi, ext_info_regs[i], buf + i, 1);
		if (ret != 1) {
			dev_warn(ctx->dev,
				 "Unable to read panel extinfo (0x%x: %d)\n",
				 ext_info_regs[i], ret);
			return ret;
		}

	}
	exynos_bin2hex(buf, i, ctx->panel_extinfo, sizeof(ctx->panel_extinfo));

	return 0;
}

static void exynos_panel_get_panel_rev(struct exynos_panel *ctx)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	u32 id, rev;

	if (kstrtou32(ctx->panel_extinfo, 16, &id)) {
		dev_warn(ctx->dev,
			 "failed to get panel extinfo, default to latest\n");
		ctx->panel_rev = PANEL_REV_LATEST;
		return;
	}

	if (funcs && funcs->get_panel_rev) {
		rev = funcs->get_panel_rev(id);
	} else {
		dev_warn(ctx->dev,
			 "unable to get panel rev, default to latest\n");
		ctx->panel_rev = PANEL_REV_LATEST;
		return;
	}

	switch (rev) {
	case 0:
		ctx->panel_rev = PANEL_REV_PROTO1;
		break;
	case 1:
		ctx->panel_rev = PANEL_REV_PROTO1_1;
		break;
	case 8:
		ctx->panel_rev = PANEL_REV_EVT1;
		break;
	case 9:
		ctx->panel_rev = PANEL_REV_EVT1_1;
		break;
	case 0xC:
		ctx->panel_rev = PANEL_REV_DVT1;
		break;
	case 0xD:
		ctx->panel_rev = PANEL_REV_DVT1_1;
		break;
	case 0x10:
		ctx->panel_rev = PANEL_REV_PVT;
		break;
	default:
		dev_warn(ctx->dev,
			 "unknown rev from panel (0x%x), default to latest\n",
			 rev);
		ctx->panel_rev = PANEL_REV_LATEST;
		return;
	}

	dev_info(ctx->dev, "panel_rev: 0x%x\n", ctx->panel_rev);
}

int exynos_panel_init(struct exynos_panel *ctx)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	int ret;

	if (ctx->initialized)
		return 0;

	ret = exynos_panel_read_id(ctx);
	if (ret)
		return ret;

	ret = exynos_panel_read_extinfo(ctx);
	if (!ret)
		ctx->initialized = true;

	exynos_panel_get_panel_rev(ctx);

	if (funcs && funcs->panel_init)
		funcs->panel_init(ctx);

	return ret;
}
EXPORT_SYMBOL(exynos_panel_init);

void exynos_panel_reset(struct exynos_panel *ctx)
{
	dev_dbg(ctx->dev, "%s +\n", __func__);

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR))
		return;

	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);

	dev_dbg(ctx->dev, "%s -\n", __func__);

	exynos_panel_init(ctx);
}
EXPORT_SYMBOL(exynos_panel_reset);

static int _exynos_panel_set_power(struct exynos_panel *ctx, bool on)
{
	int ret;

	if (on) {
		if (ctx->enable_gpio) {
			gpiod_set_value(ctx->enable_gpio, 1);
			usleep_range(10000, 11000);
		}

		if (ctx->vddi) {
			ret = regulator_enable(ctx->vddi);
			if (ret) {
				dev_err(ctx->dev, "vddi enable failed\n");
				return ret;
			}
			usleep_range(5000, 6000);
		}

		if (ctx->vddd) {
			ret = regulator_enable(ctx->vddd);
			if (ret) {
				dev_err(ctx->dev, "vddd enable failed\n");
				return ret;
			}
		}

		if (ctx->vci) {
			ret = regulator_enable(ctx->vci);
			if (ret) {
				dev_err(ctx->dev, "vci enable failed\n");
				return ret;
			}
		}

		if (ctx->vddr_en) {
			ret = regulator_enable(ctx->vddr_en);
			if (ret) {
				dev_err(ctx->dev, "vddr_en enable failed\n");
				return ret;
			}
			usleep_range(2 * 1000, 2 * 1000 + 10);
		}

		if (ctx->vddr) {
			ret = regulator_enable(ctx->vddr);
			if (ret) {
				dev_err(ctx->dev, "vddr enable failed\n");
				return ret;
			}
		}
	} else {
		gpiod_set_value(ctx->reset_gpio, 0);
		if (ctx->enable_gpio)
			gpiod_set_value(ctx->enable_gpio, 0);

		if (ctx->vddr) {
			ret = regulator_disable(ctx->vddr);
			if (ret) {
				dev_err(ctx->dev, "vddr disable failed\n");
				return ret;
			}
		}

		if (ctx->vddr_en) {
			ret = regulator_disable(ctx->vddr_en);
			if (ret) {
				dev_err(ctx->dev, "vddr_en disable failed\n");
				return ret;
			}
		}

		if (ctx->vddd) {
			ret = regulator_disable(ctx->vddd);
			if (ret) {
				dev_err(ctx->dev, "vddd disable failed\n");
				return ret;
			}
		}

		if (ctx->vddi) {
			ret = regulator_disable(ctx->vddi);
			if (ret) {
				dev_err(ctx->dev, "vddi disable failed\n");
				return ret;
			}
		}

		if (ctx->vci > 0) {
			ret = regulator_disable(ctx->vci);
			if (ret) {
				dev_err(ctx->dev, "vci disable failed\n");
				return ret;
			}
		}
	}

	return 0;
}

int exynos_panel_set_power(struct exynos_panel *ctx, bool on)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	int ret;

	if (IS_ENABLED(CONFIG_BOARD_EMULATOR))
		return 0;

	if (funcs && funcs->set_power)
		ret = funcs->set_power(ctx, on);
	else
		ret = _exynos_panel_set_power(ctx, on);

	if (ret) {
		dev_err(ctx->dev, "failed to set power: ret %d \n", ret);
		return ret;
	}

	ctx->bl->props.power = on ? FB_BLANK_UNBLANK : FB_BLANK_POWERDOWN;

	return 0;
}
EXPORT_SYMBOL(exynos_panel_set_power);

static void exynos_panel_handoff(struct exynos_panel *ctx)
{
	ctx->enabled = gpiod_get_raw_value(ctx->reset_gpio) > 0;
	if (ctx->enabled) {
		dev_info(ctx->dev, "panel enabled at boot\n");
		exynos_panel_set_power(ctx, true);
	} else {
		gpiod_direction_output(ctx->reset_gpio, 0);
	}
}

static int exynos_panel_parse_dt(struct exynos_panel *ctx)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(ctx->dev->of_node)) {
		dev_err(ctx->dev, "no device tree information of exynos panel\n");
		return -EINVAL;
	}

	ret = exynos_panel_parse_gpios(ctx);
	if (ret)
		goto err;

	ret = exynos_panel_parse_regulators(ctx);
	if (ret)
		goto err;

	ctx->touch_dev = of_parse_phandle(ctx->dev->of_node, "touch", 0);
	ctx->is_secondary = of_property_read_bool(ctx->dev->of_node, "is_secondary");

err:
	return ret;
}

static void exynos_panel_mode_set_name(struct drm_display_mode *mode)
{
	scnprintf(mode->name, DRM_DISPLAY_MODE_LEN, "%dx%dx%d",
		  mode->hdisplay, mode->vdisplay, drm_mode_vrefresh(mode));
}

int exynos_panel_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);
	struct drm_display_mode *preferred_mode = NULL;
	const struct exynos_panel_mode *current_mode = ctx->current_mode;
	int i;

	dev_dbg(ctx->dev, "%s +\n", __func__);

	for (i = 0; i < ctx->desc->num_modes; i++) {
		const struct exynos_panel_mode *pmode = &ctx->desc->modes[i];
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, &pmode->mode);
		if (!mode)
			return -ENOMEM;

		if (!mode->name[0])
			exynos_panel_mode_set_name(mode);

		mode->type |= DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode);

		dev_dbg(ctx->dev, "added display mode: %s\n", mode->name);

		if (!preferred_mode || (mode->type & DRM_MODE_TYPE_PREFERRED)) {
			preferred_mode = mode;
			/* if enabled at boot, assume preferred mode was set */
			if (ctx->enabled && !current_mode)
				ctx->current_mode = pmode;
		}
	}

	if (preferred_mode) {
		dev_dbg(ctx->dev, "preferred display mode: %s\n", preferred_mode->name);
		preferred_mode->type |= DRM_MODE_TYPE_PREFERRED;
		connector->display_info.width_mm = preferred_mode->width_mm;
		connector->display_info.height_mm = preferred_mode->height_mm;
	}

	dev_dbg(ctx->dev, "%s -\n", __func__);

	return i;
}
EXPORT_SYMBOL(exynos_panel_get_modes);

int exynos_panel_disable(struct drm_panel *panel)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);
	const struct exynos_panel_funcs *exynos_panel_func;

	ctx->enabled = false;
	ctx->hbm_mode = HBM_OFF;
	ctx->dimming_on = false;
	ctx->self_refresh_active = false;
	ctx->panel_idle_vrefresh = 0;

	exynos_panel_func = ctx->desc->exynos_panel_func;
	if (exynos_panel_func) {
		if (exynos_panel_func->set_local_hbm_mode) {
			ctx->hbm.local_hbm.enabled = false;
			sysfs_notify(&ctx->bl->dev.kobj, NULL, "local_hbm_mode");
			cancel_delayed_work_sync(&ctx->hbm.local_hbm.timeout_work);
		}
		if (exynos_panel_func->set_hbm_mode) {
			cancel_work_sync(&ctx->hbm.hbm_work);
			if (ctx->hbm.commit) {
				drm_crtc_commit_put(ctx->hbm.commit);
				ctx->hbm.commit = NULL;
			}
		}
	}

	mutex_lock(&ctx->mode_lock);
	exynos_panel_send_cmd_set(ctx, ctx->desc->off_cmd_set);
	mutex_unlock(&ctx->mode_lock);
	dev_dbg(ctx->dev, "%s\n", __func__);
	return 0;
}
EXPORT_SYMBOL(exynos_panel_disable);

int exynos_panel_unprepare(struct drm_panel *panel)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);

	dev_dbg(ctx->dev, "%s +\n", __func__);
	exynos_panel_set_power(ctx, false);
	dev_dbg(ctx->dev, "%s -\n", __func__);
	return 0;
}
EXPORT_SYMBOL(exynos_panel_unprepare);

int exynos_panel_prepare(struct drm_panel *panel)
{
	struct exynos_panel *ctx =
		container_of(panel, struct exynos_panel, panel);

	dev_dbg(ctx->dev, "%s +\n", __func__);
	exynos_panel_set_power(ctx, true);
	dev_dbg(ctx->dev, "%s -\n", __func__);

	return 0;
}
EXPORT_SYMBOL(exynos_panel_prepare);

void exynos_panel_send_cmd_set_flags(struct exynos_panel *ctx,
				     const struct exynos_dsi_cmd_set *cmd_set, u32 flags)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	const struct exynos_dsi_cmd *c;
	const struct exynos_dsi_cmd *last_cmd = NULL;
	const u32 async_mask = PANEL_CMD_SET_BATCH | PANEL_CMD_SET_QUEUE;
	u16 dsi_flags = 0;

	if (!cmd_set || !cmd_set->num_cmd)
		return;

	/* shouldn't have both queue and batch set together */
	WARN_ON((flags & async_mask) == async_mask);

	if (flags & PANEL_CMD_SET_IGNORE_VBLANK)
		dsi_flags |= EXYNOS_DSI_MSG_IGNORE_VBLANK;

	/* if not batched or queued, all commands should be sent out immediately */
	if (!(flags & async_mask))
		dsi_flags |= MIPI_DSI_MSG_LASTCOMMAND;

	c = &cmd_set->cmds[cmd_set->num_cmd - 1];
	if (!c->panel_rev) {
		last_cmd = c;
	} else {
		for (; c >= cmd_set->cmds; c--) {
			if (c->panel_rev & ctx->panel_rev) {
				last_cmd = c;
				break;
			}
		}
	}

	/* no commands to transfer */
	if (!last_cmd)
		return;

	for (c = cmd_set->cmds; c <= last_cmd; c++) {
		u32 delay_ms = c->delay_ms;

		if (ctx->panel_rev && !(c->panel_rev & ctx->panel_rev))
			continue;

		if ((c == last_cmd) && !(flags & PANEL_CMD_SET_QUEUE))
			dsi_flags |= MIPI_DSI_MSG_LASTCOMMAND;

		exynos_dsi_dcs_write_buffer(dsi, c->cmd, c->cmd_len, dsi_flags);
		if (delay_ms)
			usleep_range(delay_ms * 1000, delay_ms * 1000 + 10);
	}
}
EXPORT_SYMBOL(exynos_panel_send_cmd_set_flags);

void exynos_panel_set_lp_mode(struct exynos_panel *ctx, const struct exynos_panel_mode *pmode)
{
	if (!ctx->enabled)
		return;

	exynos_panel_send_cmd_set(ctx, ctx->desc->lp_cmd_set);

	dev_info(ctx->dev, "enter %dhz LP mode\n", drm_mode_vrefresh(&pmode->mode));
}
EXPORT_SYMBOL(exynos_panel_set_lp_mode);

void exynos_panel_set_binned_lp(struct exynos_panel *ctx, const u16 brightness)
{
	int i;
	const struct exynos_binned_lp *binned_lp;
	struct backlight_device *bl = ctx->bl;
	bool is_lp_state;
	enum exynos_panel_state panel_state;

	for (i = 0; i < ctx->desc->num_binned_lp; i++) {
		binned_lp = &ctx->desc->binned_lp[i];
		if (brightness <= binned_lp->bl_threshold)
			break;
	}
	if (i == ctx->desc->num_binned_lp)
		return;

	mutex_lock(&ctx->bl_state_lock);
	is_lp_state = is_backlight_lp_state(bl);
	mutex_unlock(&ctx->bl_state_lock);

	mutex_lock(&ctx->lp_state_lock);

	if (is_lp_state && ctx->current_binned_lp &&
	    binned_lp->bl_threshold == ctx->current_binned_lp->bl_threshold) {
		mutex_unlock(&ctx->lp_state_lock);
		return;
	}

	exynos_panel_send_cmd_set(ctx, &binned_lp->cmd_set);

	ctx->current_binned_lp = binned_lp;
	dev_dbg(ctx->dev, "enter lp_%s\n", ctx->current_binned_lp->name);

	mutex_unlock(&ctx->lp_state_lock);

	panel_state = !binned_lp->bl_threshold ? PANEL_STATE_OFF : PANEL_STATE_LP;
	exynos_panel_set_backlight_state(ctx, panel_state);

	if (bl)
		sysfs_notify(&bl->dev.kobj, NULL, "lp_state");

	if (panel_state != PANEL_STATE_OFF)
		exynos_panel_update_te2(ctx);
}
EXPORT_SYMBOL(exynos_panel_set_binned_lp);

int exynos_panel_set_brightness(struct exynos_panel *exynos_panel, u16 br)
{
	u16 brightness;

	if (exynos_panel->current_mode->exynos_mode.is_lp_mode) {
		const struct exynos_panel_funcs *funcs;

		funcs = exynos_panel->desc->exynos_panel_func;
		if (funcs && funcs->set_binned_lp)
			funcs->set_binned_lp(exynos_panel, br);
		return 0;
	}

	brightness = (br & 0xff) << 8 | br >> 8;

	return exynos_dcs_set_brightness(exynos_panel, brightness);
}
EXPORT_SYMBOL(exynos_panel_set_brightness);

static int exynos_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int exynos_bl_find_range(struct exynos_panel *ctx,
				int brightness, u32 *range)
{
	u32 i;

	if (!ctx->bl_notifier.num_ranges)
		return -EOPNOTSUPP;

	mutex_lock(&ctx->bl_state_lock);

	for (i = 0; i < ctx->bl_notifier.num_ranges; i++) {
		if (brightness <= ctx->bl_notifier.ranges[i]) {
			*range = i;
			mutex_unlock(&ctx->bl_state_lock);

			return 0;
		}
	}

	mutex_unlock(&ctx->bl_state_lock);

	dev_warn(ctx->dev, "failed to find bl range\n");

	return -EINVAL;
}

static int exynos_update_status(struct backlight_device *bl)
{
	struct exynos_panel *ctx = bl_get_data(bl);
	const struct exynos_panel_funcs *exynos_panel_func;
	int brightness = bl->props.brightness;
	int min_brightness = ctx->desc->min_brightness ? : 1;
	u32 bl_range = 0;

	if (!ctx->enabled || !ctx->initialized) {
		dev_dbg(ctx->dev, "panel is not enabled\n");
		return -EPERM;
	}

	/* check if backlight is forced off */
	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (brightness && brightness < min_brightness)
		brightness = min_brightness;

	dev_info(ctx->dev, "req: %d, br: %d\n", bl->props.brightness,
		brightness);

	mutex_lock(&ctx->mode_lock);
	exynos_panel_func = ctx->desc->exynos_panel_func;
	if (exynos_panel_func && exynos_panel_func->set_brightness)
		exynos_panel_func->set_brightness(ctx, brightness);
	else
		exynos_dcs_set_brightness(ctx, brightness);

	if (!ctx->hbm_mode &&
	    exynos_bl_find_range(ctx, brightness, &bl_range) >= 0 &&
	    bl_range != ctx->bl_notifier.current_range) {
		ctx->bl_notifier.current_range = bl_range;

		sysfs_notify(&ctx->bl->dev.kobj, NULL, "brightness");

		dev_dbg(ctx->dev, "bl range is changed to %d\n",
			ctx->bl_notifier.current_range);
	}
	mutex_unlock(&ctx->mode_lock);
	return 0;
}

static const struct backlight_ops exynos_backlight_ops = {
	.get_brightness = exynos_get_brightness,
	.update_status = exynos_update_status,
};

static ssize_t serial_number_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	const struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	if (!ctx->initialized)
		return -EPERM;

	return snprintf(buf, PAGE_SIZE, "%s\n", ctx->panel_id);
}

static ssize_t panel_extinfo_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	const struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	if (!ctx->initialized)
		return -EPERM;

	return snprintf(buf, PAGE_SIZE, "%s\n", ctx->panel_extinfo);
}

static ssize_t panel_name_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n", dsi->name);
}

static ssize_t gamma_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	const struct exynos_panel_funcs *funcs;
	size_t len, ret;
	char *input_buf;
	const char *out_buf;

	if (!ctx->enabled || !ctx->initialized)
		return -EPERM;

	funcs = ctx->desc->exynos_panel_func;
	if (!funcs || !funcs->gamma_store)
		return -EOPNOTSUPP;

	if (!strncmp(buf, DEFAULT_GAMMA_STR, strlen(DEFAULT_GAMMA_STR))) {
		if (!funcs->restore_native_gamma)
			return -EOPNOTSUPP;
		else
			ret = funcs->restore_native_gamma(ctx);

		return ret ? : count;
	}

	input_buf = kstrndup(buf, count, GFP_KERNEL);
	if (!input_buf)
		return -ENOMEM;

	len = exynos_panel_parse_byte_buf(input_buf, count, &out_buf);
	kfree(input_buf);
	if (len <= 0)
		return len;

	ret = funcs->gamma_store(ctx, out_buf, len);
	kfree(out_buf);

	return ret ? : count;
}

static ssize_t set_te2_timing(struct exynos_panel *ctx, size_t count,
			      const char *buf, bool lp_mode)
{
	char *buf_dup;
	ssize_t type_len, data_len;
	u32 timing[MAX_TE2_TYPE * 2] = {0};
	const struct exynos_panel_funcs *funcs;

	if (!ctx->enabled)
		return -EPERM;

	funcs = ctx->desc->exynos_panel_func;
	if (!funcs || !funcs->configure_te2_edges || !funcs->update_te2)
		return -EINVAL;

	if (!count)
		return -EINVAL;

	buf_dup = kstrndup(buf, count, GFP_KERNEL);
	if (!buf_dup)
		return -ENOMEM;

	type_len = lp_mode ? (ctx->desc->num_binned_lp - 1) :
			     ctx->desc->num_modes;
	data_len = parse_u32_buf(buf_dup, count + 1, timing, type_len * 2);
	if (data_len != type_len * 2) {
		dev_warn(ctx->dev,
			 "invalid number of TE2 %s timing: expected %ld but actual %ld\n",
			 lp_mode ? "LP" : "normal",
			 type_len * 2, data_len);
		kfree(buf_dup);
		return -EINVAL;
	}

	mutex_lock(&ctx->mode_lock);
	funcs->configure_te2_edges(ctx, timing, lp_mode);
	funcs->update_te2(ctx);
	mutex_unlock(&ctx->mode_lock);

	kfree(buf_dup);

	return count;
}

static ssize_t get_te2_timing(struct exynos_panel *ctx, char *buf, bool lp_mode)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	size_t len;

	if (!funcs || !funcs->get_te2_edges)
		return -EPERM;

	mutex_lock(&ctx->mode_lock);
	len = funcs->get_te2_edges(ctx, buf, lp_mode);
	mutex_unlock(&ctx->mode_lock);

	return len;
}

static ssize_t te2_timing_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;

	if (!ctx->initialized)
		return -EAGAIN;

	ret = set_te2_timing(ctx, count, buf, false);
	if (ret < 0)
		dev_err(ctx->dev,
			"failed to set normal mode TE2 timing: ret %ld\n", ret);

	return ret;
}

static ssize_t te2_timing_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;

	if (!ctx->initialized)
		return -EAGAIN;

	ret = get_te2_timing(ctx, buf, false);
	if (ret < 0)
		dev_err(ctx->dev,
			"failed to get normal mode TE2 timing: ret %ld\n", ret);

	return ret;
}

static ssize_t te2_lp_timing_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;

	if (!ctx->initialized)
		return -EAGAIN;

	ret = set_te2_timing(ctx, count, buf, true);
	if (ret < 0)
		dev_err(ctx->dev,
			"failed to set LP mode TE2 timing: ret %ld\n", ret);

	return ret;
}

static ssize_t te2_lp_timing_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	ssize_t ret;

	if (!ctx->initialized)
		return -EAGAIN;

	ret = get_te2_timing(ctx, buf, true);
	if (ret < 0)
		dev_err(ctx->dev,
			"failed to get LP mode TE2 timing: ret %ld\n", ret);

	return ret;
}

static bool panel_idle_queue_delayed_work(struct exynos_panel *ctx)
{
	const ktime_t now = ktime_get();
	const unsigned int delta_ms = ktime_to_ms(ktime_sub(now, ctx->last_mode_set_ts));

	if (delta_ms < ctx->idle_delay_ms) {
		const unsigned int delay_ms = ctx->idle_delay_ms - delta_ms;

		dev_dbg(ctx->dev, "%s: last mode %ums ago, schedule idle in %ums\n",
			__func__, delta_ms, delay_ms);

		mod_delayed_work(system_highpri_wq, &ctx->idle_work,
					msecs_to_jiffies(delay_ms));
		return true;
	}

	return false;
}

static void panel_update_idle_mode_locked(struct exynos_panel *ctx)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;

	WARN_ON(!mutex_is_locked(&ctx->mode_lock));

	if (unlikely(!ctx->initialized || !ctx->current_mode || !funcs))
		return;

	if (!ctx->enabled || !funcs->set_self_refresh)
		return;

	if (ctx->idle_delay_ms && ctx->self_refresh_active && panel_idle_queue_delayed_work(ctx))
		return;

	if (delayed_work_pending(&ctx->idle_work)) {
		dev_dbg(ctx->dev, "%s: cancelling delayed idle work\n", __func__);
		cancel_delayed_work(&ctx->idle_work);
	}

	if (funcs->set_self_refresh(ctx, ctx->self_refresh_active))
		exynos_panel_update_te2(ctx);
}

static void panel_idle_work(struct work_struct *work)
{
	struct exynos_panel *ctx = container_of(work, struct exynos_panel, idle_work.work);

	dev_dbg(ctx->dev, "%s\n", __func__);

	mutex_lock(&ctx->mode_lock);
	panel_update_idle_mode_locked(ctx);
	mutex_unlock(&ctx->mode_lock);
}

static ssize_t panel_idle_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	bool idle_enabled;
	int ret;

	ret = kstrtobool(buf, &idle_enabled);
	if (ret) {
		dev_err(dev, "invalid panel idle value\n");
		return ret;
	}

	mutex_lock(&ctx->mode_lock);
	if (idle_enabled != ctx->panel_idle_enabled) {
		ctx->panel_idle_enabled = idle_enabled;
		panel_update_idle_mode_locked(ctx);
	}
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t panel_idle_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->panel_idle_enabled);
}

static ssize_t min_vrefresh_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	int min_vrefresh;
	int ret;

	ret = kstrtoint(buf, 0, &min_vrefresh);
	if (ret) {
		dev_err(dev, "invalid min vrefresh value\n");
		return ret;
	}

	mutex_lock(&ctx->mode_lock);
	ctx->min_vrefresh = min_vrefresh;
	panel_update_idle_mode_locked(ctx);
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t min_vrefresh_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->min_vrefresh);
}

static ssize_t idle_delay_ms_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);
	u32 idle_delay_ms;
	int ret;

	ret = kstrtou32(buf, 0, &idle_delay_ms);
	if (ret) {
		dev_err(dev, "invalid idle delay ms\n");
		return ret;
	}

	mutex_lock(&ctx->mode_lock);
	ctx->idle_delay_ms = idle_delay_ms;
	panel_update_idle_mode_locked(ctx);
	mutex_unlock(&ctx->mode_lock);

	return count;
}

static ssize_t idle_delay_ms_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->idle_delay_ms);
}

static DEVICE_ATTR_RO(serial_number);
static DEVICE_ATTR_RO(panel_extinfo);
static DEVICE_ATTR_RO(panel_name);
static DEVICE_ATTR_WO(gamma);
static DEVICE_ATTR_RW(te2_timing);
static DEVICE_ATTR_RW(te2_lp_timing);
static DEVICE_ATTR_RW(panel_idle);
static DEVICE_ATTR_RW(min_vrefresh);
static DEVICE_ATTR_RW(idle_delay_ms);

static const struct attribute *panel_attrs[] = {
	&dev_attr_serial_number.attr,
	&dev_attr_panel_extinfo.attr,
	&dev_attr_panel_name.attr,
	&dev_attr_gamma.attr,
	&dev_attr_te2_timing.attr,
	&dev_attr_te2_lp_timing.attr,
	&dev_attr_panel_idle.attr,
	&dev_attr_min_vrefresh.attr,
	&dev_attr_idle_delay_ms.attr,
	NULL
};

static void exynos_panel_connector_print_state(struct drm_printer *p,
					       const struct exynos_drm_connector_state *state)
{
	const struct exynos_drm_connector *exynos_connector =
		to_exynos_connector(state->base.connector);
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);
	const struct exynos_panel_desc *desc = ctx->desc;
	int ret;

	ret = mutex_lock_interruptible(&ctx->mode_lock);
	if (ret)
		return;

	drm_printf(p, "\tenabled: %d\n", ctx->enabled);

	drm_puts(p, "\tidle: ");
	if (ctx->panel_idle_vrefresh)
		drm_printf(p, "%uhz", ctx->panel_idle_vrefresh);
	else
		drm_puts(p, "no");
	drm_printf(p, " (%s)  min_vrefresh: %d  idle_delay: %ums\n",
		   ctx->panel_idle_enabled ? "enabled" : "disabled",
		   ctx->min_vrefresh, ctx->idle_delay_ms);

	if (ctx->current_mode) {
		const struct drm_display_mode *m = &ctx->current_mode->mode;

		drm_printf(p, " \tcurrent mode: %dx%d@%d\n", m->hdisplay,
			   m->vdisplay, drm_mode_vrefresh(m));
	}
	drm_printf(p, "\text_info: %s\n", ctx->panel_extinfo);
	drm_printf(p, "\tluminance: [%u, %u] avg: %u\n",
		   desc->min_luminance, desc->max_luminance,
		   desc->max_avg_luminance);
	drm_printf(p, "\thdr_formats: 0x%x\n", desc->hdr_formats);
	drm_printf(p, "\thbm_mode: %u\n", ctx->hbm_mode);
	drm_printf(p, "\tdimming_on: %s\n", ctx->dimming_on ? "true" : "false");
	drm_printf(p, "\tis_partial: %s\n", desc->is_partial ? "true" : "false");

	mutex_unlock(&ctx->mode_lock);
}

static int exynos_panel_connector_get_property(
				   struct exynos_drm_connector *exynos_connector,
				   const struct exynos_drm_connector_state *exynos_state,
				   struct drm_property *property,
				   uint64_t *val)
{
	struct exynos_drm_connector_properties *p =
		exynos_drm_connector_get_properties(exynos_connector);
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);

	if (property == p->brightness_level) {
		*val = exynos_state->brightness_level;
		dev_dbg(ctx->dev, "%s: brt(%llu)\n", __func__, *val);
	} else if (property == p->global_hbm_mode) {
		*val = exynos_state->global_hbm_mode;
		dev_dbg(ctx->dev, "%s: global_hbm_mode(%llu)\n", __func__, *val);
	}  else if (property == p->local_hbm_on) {
		*val = exynos_state->local_hbm_on;
		dev_dbg(ctx->dev, "%s: local_hbm_on(%s)\n", __func__, *val ? "true" : "false");
	} else if (property == p->dimming_on) {
		*val = exynos_state->dimming_on;
		dev_dbg(ctx->dev, "%s: dimming_on(%s)\n", __func__, *val ? "true" : "false");
	} else if (property == p->sync_rr_switch) {
		*val = exynos_state->sync_rr_switch;
		dev_dbg(ctx->dev, "%s: sync_rr_switch(%s)\n", __func__, *val ? "true" : "false");
	} else
		return -EINVAL;

	return 0;
}

static int exynos_panel_connector_set_property(
				   struct exynos_drm_connector *exynos_connector,
				   struct exynos_drm_connector_state *exynos_state,
				   struct drm_property *property,
				   uint64_t val)
{
	struct exynos_drm_connector_properties *p =
		exynos_drm_connector_get_properties(exynos_connector);
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);

	if (property == p->brightness_level) {
		exynos_state->pending_update_flags |= HBM_FLAG_BL_UPDATE;
		exynos_state->brightness_level = val;
		dev_dbg(ctx->dev, "%s: brt(%u)\n", __func__, exynos_state->brightness_level);
	} else if (property == p->global_hbm_mode) {
		exynos_state->pending_update_flags |= HBM_FLAG_GHBM_UPDATE;
		exynos_state->global_hbm_mode = val;
		dev_dbg(ctx->dev, "%s: global_hbm_mode(%u)\n", __func__,
			 exynos_state->global_hbm_mode);
	} else if (property == p->local_hbm_on) {
		exynos_state->pending_update_flags |= HBM_FLAG_LHBM_UPDATE;
		exynos_state->local_hbm_on = val;
		dev_dbg(ctx->dev, "%s: local_hbm_on(%s)\n", __func__,
			 exynos_state->local_hbm_on ? "true" : "false");
	} else if (property == p->dimming_on) {
		exynos_state->pending_update_flags |= HBM_FLAG_DIMMING_UPDATE;
		exynos_state->dimming_on = val;
		dev_dbg(ctx->dev, "%s: dimming_on(%s)\n", __func__,
			 exynos_state->dimming_on ? "true" : "false");
	} else if (property == p->sync_rr_switch) {
		exynos_state->sync_rr_switch = val;
		dev_dbg(ctx->dev, "%s: sync_rr_switch(%s)\n", __func__,
			 exynos_state->sync_rr_switch ? "true" : "false");
	} else
		return -EINVAL;

	return 0;
}

static const struct exynos_drm_connector_funcs exynos_panel_connector_funcs = {
	.atomic_print_state = exynos_panel_connector_print_state,
	.atomic_get_property = exynos_panel_connector_get_property,
	.atomic_set_property = exynos_panel_connector_set_property,
};

static void exynos_panel_set_dimming(struct exynos_panel *ctx, bool dimming_on)
{
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;

	if (!funcs || !funcs->set_dimming_on)
		return;

	mutex_lock(&ctx->mode_lock);
	if (dimming_on != ctx->dimming_on) {
		funcs->set_dimming_on(ctx, dimming_on);
		panel_update_idle_mode_locked(ctx);
	}
	mutex_unlock(&ctx->mode_lock);
}

static void exynos_panel_commit_properties(
				struct exynos_panel *ctx,
				const struct exynos_drm_connector_state *conn_state)
{
	const struct exynos_panel_funcs *exynos_panel_func = ctx->desc->exynos_panel_func;
	u8 update_flags = 0;

	if (!conn_state->pending_update_flags)
		return;


	cancel_work_sync(&ctx->hbm.hbm_work);
	mutex_lock(&ctx->hbm.hbm_work_lock);
	update_flags = ctx->hbm.update_flags;
	ctx->hbm.update_flags = 0;
	mutex_unlock(&ctx->hbm.hbm_work_lock);


	if ((conn_state->pending_update_flags & HBM_FLAG_GHBM_UPDATE) &&
		exynos_panel_func->set_hbm_mode &&
		(ctx->hbm_mode != conn_state->global_hbm_mode))
		update_flags |= HBM_FLAG_GHBM_UPDATE;

	if ((conn_state->pending_update_flags & HBM_FLAG_BL_UPDATE) &&
		(ctx->bl->props.brightness != conn_state->brightness_level))
		update_flags |= HBM_FLAG_BL_UPDATE;

	if ((conn_state->pending_update_flags & HBM_FLAG_LHBM_UPDATE) &&
		exynos_panel_func->set_local_hbm_mode &&
		(ctx->hbm.local_hbm.enabled != conn_state->local_hbm_on))
		update_flags |= HBM_FLAG_LHBM_UPDATE;

	if ((conn_state->pending_update_flags & HBM_FLAG_DIMMING_UPDATE) &&
		exynos_panel_func->set_dimming_on &&
		(ctx->dimming_on != conn_state->dimming_on))
		update_flags |= HBM_FLAG_DIMMING_UPDATE;

	dev_dbg(ctx->dev, "%s: update_flags(0x%x)", __func__, update_flags);
	if (update_flags & (HBM_FLAG_GHBM_UPDATE | HBM_FLAG_LHBM_UPDATE)) {
		struct drm_crtc_commit *commit = conn_state->base.commit;
		struct drm_crtc_commit *old_commit;

		if (commit)
			drm_crtc_commit_get(commit);

		mutex_lock(&ctx->hbm.hbm_work_lock);
		old_commit = ctx->hbm.commit;
		if (WARN_ON(old_commit))
			drm_crtc_commit_put(old_commit);

		ctx->hbm.commit = commit;

		if (update_flags & HBM_FLAG_GHBM_UPDATE)
			ctx->hbm.request_global_hbm_mode = conn_state->global_hbm_mode;

		if (update_flags & HBM_FLAG_BL_UPDATE)
			ctx->bl->props.brightness = conn_state->brightness_level;

		if (update_flags & HBM_FLAG_LHBM_UPDATE)
			ctx->hbm.local_hbm.request_hbm_mode = conn_state->local_hbm_on;

		if (update_flags & HBM_FLAG_DIMMING_UPDATE)
			ctx->request_dimming_on = conn_state->dimming_on;

		ctx->hbm.update_flags = update_flags;
		update_flags = 0;
		queue_work(system_highpri_wq, &ctx->hbm.hbm_work);
		mutex_unlock(&ctx->hbm.hbm_work_lock);
	}

	if (update_flags & HBM_FLAG_BL_UPDATE) {
		ctx->bl->props.brightness = conn_state->brightness_level;
		backlight_update_status(ctx->bl);
	}


	if (update_flags & HBM_FLAG_DIMMING_UPDATE)
		exynos_panel_set_dimming(ctx, conn_state->dimming_on);

}

static void exynos_panel_connector_atomic_commit(
				struct exynos_drm_connector *exynos_connector,
			    struct exynos_drm_connector_state *exynos_old_state,
			    struct exynos_drm_connector_state *exynos_new_state)
{
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);
	const struct exynos_panel_funcs *exynos_panel_func = ctx->desc->exynos_panel_func;

	if (!exynos_panel_func)
		return;

	exynos_panel_commit_properties(ctx, exynos_new_state);

	mutex_lock(&ctx->mode_lock);
	if (exynos_panel_func->commit_done)
		exynos_panel_func->commit_done(ctx);
	mutex_unlock(&ctx->mode_lock);

	ctx->last_commit_ts = ktime_get();
}

static const struct exynos_drm_connector_helper_funcs exynos_panel_connector_helper_funcs = {
	.atomic_commit = exynos_panel_connector_atomic_commit,
};

static int exynos_drm_connector_modes(struct drm_connector *connector)
{
	struct exynos_drm_connector *exynos_connector = to_exynos_connector(connector);
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);
	int ret;

	ret = drm_panel_get_modes(&ctx->panel, connector);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to get panel display modes\n");
		return ret;
	}

	return ret;
}

static const struct exynos_panel_mode *exynos_panel_get_mode(struct exynos_panel *ctx,
							     const struct drm_display_mode *mode)
{
	const struct exynos_panel_mode *pmode;
	int i;

	for (i = 0; i < ctx->desc->num_modes; i++) {
		pmode = &ctx->desc->modes[i];

		if (drm_mode_equal(&pmode->mode, mode))
			return pmode;
	}

	pmode = ctx->desc->lp_mode;
	if (pmode && drm_mode_equal(&pmode->mode, mode))
		return pmode;

	return NULL;
}

static void exynos_drm_connector_attach_touch(struct exynos_panel *ctx,
					      const struct drm_connector_state *connector_state,
					      const struct drm_crtc_state *crtc_state)
{
	struct drm_encoder *encoder = connector_state->best_encoder;
	struct drm_bridge *bridge;

	if (!encoder) {
		dev_warn(ctx->dev, "%s encoder is null\n", __func__);
		return;
	}

	bridge = of_drm_find_bridge(ctx->touch_dev);
	if (!bridge || bridge->dev)
		return;

	drm_bridge_attach(encoder, bridge, NULL, 0);
	dev_info(ctx->dev, "attach bridge %p to encoder %p\n", bridge, encoder);
}

/*
 * Check whether transition to new mode can be done seamlessly without having
 * to turn display off before mode change. This is currently only possible if
 * only clocks/refresh rate is changing
 */
static bool exynos_panel_is_mode_seamless(const struct exynos_panel *ctx,
					  const struct exynos_panel_mode *mode)
{
	const struct exynos_panel_funcs *funcs;

	/* no need to go through seamless mode set if panel is disabled */
	if (!ctx->enabled || !ctx->initialized)
		return false;

	funcs = ctx->desc->exynos_panel_func;
	if (!funcs || !funcs->is_mode_seamless)
		return false;

	return funcs->is_mode_seamless(ctx, mode);
}

static void exynos_panel_set_partial(struct exynos_display_partial *partial,
			const struct exynos_panel_mode *pmode, bool is_partial)
{
	const struct exynos_display_dsc *dsc = &pmode->exynos_mode.dsc;
	const struct drm_display_mode *mode = &pmode->mode;

	partial->enabled = is_partial;
	if (!partial->enabled)
		return;

	if (dsc->enabled) {
		partial->min_width = DIV_ROUND_UP(mode->hdisplay, dsc->slice_count);
		partial->min_height = dsc->slice_height;
	} else {
		partial->min_width = MIN_WIN_BLOCK_WIDTH;
		partial->min_height = MIN_WIN_BLOCK_HEIGHT;
	}
}

static int exynos_drm_connector_check_mode(struct exynos_panel *ctx,
					   struct drm_connector_state *connector_state,
					   const struct drm_display_mode *mode)
{
	struct exynos_drm_connector_state *exynos_connector_state =
		to_exynos_connector_state(connector_state);
	const struct exynos_panel_mode *pmode = exynos_panel_get_mode(ctx, mode);

	if (!pmode) {
		dev_warn(ctx->dev, "invalid mode %s\n", mode->name);
		return -EINVAL;
	}

	exynos_connector_state->seamless_possible = exynos_panel_is_mode_seamless(ctx, pmode);
	exynos_connector_state->exynos_mode = pmode->exynos_mode;
	exynos_panel_set_partial(&exynos_connector_state->partial, pmode,
			ctx->desc->is_partial);

	return 0;
}

static int exynos_drm_connector_check_state(struct exynos_panel *ctx,
					   struct drm_connector_state *connector_state)
{
	struct exynos_drm_connector_state *exynos_connector_state =
		to_exynos_connector_state(connector_state);

	if (exynos_connector_state->global_hbm_mode && exynos_connector_state->local_hbm_on) {
		dev_err(ctx->dev, "invalid state - both LHBM and GHBM on");
		return -EINVAL;
	}

	return 0;
}

/*
 * this atomic check is called before adjusted mode is populated, this can be used to check only
 * connector state (without adjusted mode), or to decide if modeset may be required
 */
static int exynos_drm_connector_atomic_check(struct drm_connector *connector,
					     struct drm_atomic_state *state)
{
	struct exynos_drm_connector *exynos_connector = to_exynos_connector(connector);
	struct drm_connector_state *connector_state =
		drm_atomic_get_new_connector_state(state, connector);
	struct exynos_panel *ctx = exynos_connector_to_panel(exynos_connector);
	struct drm_crtc_state *crtc_state;

	/* nothing to do if disabled or if mode is unchanged */
	if (!connector_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_new_crtc_state(state, connector_state->crtc);

	if (ctx->touch_dev)
		exynos_drm_connector_attach_touch(ctx, connector_state, crtc_state);

	return exynos_drm_connector_check_state(ctx, connector_state);
}

static const struct drm_connector_helper_funcs exynos_connector_helper_funcs = {
	.atomic_check = exynos_drm_connector_atomic_check,
	.get_modes = exynos_drm_connector_modes,
};

#ifdef CONFIG_DEBUG_FS

static u8 panel_get_cmd_type(const struct exynos_dsi_cmd *cmd)
{
	if (cmd->type)
		return cmd->type;

	switch (cmd->cmd_len) {
	case 0:
		return -EINVAL;
	case 1:
		return MIPI_DSI_DCS_SHORT_WRITE;
	case 2:
		return MIPI_DSI_DCS_SHORT_WRITE_PARAM;
	default:
		return MIPI_DSI_DCS_LONG_WRITE;
	}
}

static int panel_cmdset_show(struct seq_file *m, void *data)
{
	const struct exynos_dsi_cmd_set *cmdset = m->private;
	const struct exynos_dsi_cmd *cmd;
	u8 type;
	int i;

	for (i = 0; i < cmdset->num_cmd; i++) {
		cmd = &cmdset->cmds[i];

		type = panel_get_cmd_type(cmd);
		seq_printf(m, "0x%02x ", type);
		seq_hex_dump(m, "\t", DUMP_PREFIX_NONE, 16, 1, cmd->cmd, cmd->cmd_len, false);

		if (cmd->delay_ms)
			seq_printf(m, "wait \t%dms\n", cmd->delay_ms);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(panel_cmdset);

void exynos_panel_debugfs_create_cmdset(struct exynos_panel *ctx,
					struct dentry *parent,
					const struct exynos_dsi_cmd_set *cmdset,
					const char *name)

{
	if (!cmdset)
		return;

	debugfs_create_file(name, 0600, parent, (void *)cmdset, &panel_cmdset_fops);
}
EXPORT_SYMBOL(exynos_panel_debugfs_create_cmdset);

static int panel_gamma_show(struct seq_file *m, void *data)
{
	struct exynos_panel *ctx = m->private;
	const struct exynos_panel_funcs *funcs;
	const struct drm_display_mode *mode;
	int i;

	funcs = ctx->desc->exynos_panel_func;
	for_each_display_mode(i, mode, ctx) {
		seq_printf(m, "\n=== %dhz Mode Gamma ===\n", drm_mode_vrefresh(mode));
		funcs->print_gamma(m, mode);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(panel_gamma);

static int panel_debugfs_add(struct exynos_panel *ctx, struct dentry *parent)
{
	const struct exynos_panel_desc *desc = ctx->desc;
	const struct exynos_panel_funcs *funcs = desc->exynos_panel_func;
	struct dentry *root;

	debugfs_create_u32("rev", 0600, parent, &ctx->panel_rev);

	if (!funcs)
		return -EINVAL;

	if (funcs->print_gamma)
		debugfs_create_file("gamma", 0600, parent, ctx, &panel_gamma_fops);

	root = debugfs_create_dir("cmdsets", ctx->debugfs_entry);
	if (!root) {
		dev_err(ctx->dev, "can't create cmdset dir\n");
		return -EFAULT;
	}
	ctx->debugfs_cmdset_entry = root;

	exynos_panel_debugfs_create_cmdset(ctx, root, desc->off_cmd_set, "off");

	if (desc->lp_mode) {
		struct dentry *lpd;
		int i;

		if (desc->binned_lp) {
			lpd = debugfs_create_dir("lp", root);
			if (!lpd) {
				dev_err(ctx->dev, "can't create lp dir\n");
				return -EFAULT;
			}

			for (i = 0; i < desc->num_binned_lp; i++) {
				const struct exynos_binned_lp *b = &desc->binned_lp[i];

				exynos_panel_debugfs_create_cmdset(ctx, lpd, &b->cmd_set, b->name);
			}
		} else {
			lpd = root;
		}
		exynos_panel_debugfs_create_cmdset(ctx, lpd, desc->lp_cmd_set, "lp_entry");
	}

	return 0;
}

static ssize_t exynos_dsi_dcs_transfer(struct mipi_dsi_device *dsi, u8 type,
				     const void *data, size_t len, u16 flags)
{
	const struct mipi_dsi_host_ops *ops = dsi->host->ops;
	struct mipi_dsi_msg msg = {
		.channel = dsi->channel,
		.tx_buf = data,
		.tx_len = len,
		.type = type,
	};

	if (!ops || !ops->transfer)
		return -ENOSYS;

	msg.flags = flags;
	if (dsi->mode_flags & MIPI_DSI_MODE_LPM)
		msg.flags |= MIPI_DSI_MSG_USE_LPM;

	return ops->transfer(dsi->host, &msg);
}

ssize_t exynos_dsi_dcs_write_buffer(struct mipi_dsi_device *dsi,
				  const void *data, size_t len, u16 flags)
{
	u8 type;

	switch (len) {
	case 0:
		return -EINVAL;

	case 1:
		type = MIPI_DSI_DCS_SHORT_WRITE;
		break;

	case 2:
		type = MIPI_DSI_DCS_SHORT_WRITE_PARAM;
		break;

	default:
		type = MIPI_DSI_DCS_LONG_WRITE;
		break;
	}

	return exynos_dsi_dcs_transfer(dsi, type, data, len, flags);
}
EXPORT_SYMBOL(exynos_dsi_dcs_write_buffer);

static int exynos_dsi_name_show(struct seq_file *m, void *data)
{
	struct mipi_dsi_device *dsi = m->private;

	seq_puts(m, dsi->name);
	seq_putc(m, '\n');

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(exynos_dsi_name);

static ssize_t parse_byte_buf(u8 *out, size_t len, char *src)
{
	const char *skip = "\n ";
	size_t i = 0;
	int rc = 0;
	char *s;

	while (src && !rc && i < len) {
		s = strsep(&src, skip);
		if (*s != '\0') {
			rc = kstrtou8(s, 16, out + i);
			i++;
		}
	}

	return rc ? : i;
}

static ssize_t exynos_panel_parse_byte_buf(char *input_str, size_t input_len,
					   const char **out_buf)
{
	size_t len = (input_len + 1) / 2;
	size_t rc;
	char *out;

	out = kzalloc(len, GFP_KERNEL);
	if (!out)
		return -ENOMEM;

	rc = parse_byte_buf(out, len, input_str);
	if (rc <= 0) {
		kfree(out);
		return rc;
	}

	*out_buf = out;

	return rc;
}

struct exynos_dsi_reg_data {
	struct mipi_dsi_device *dsi;
	u8 address;
	u8 type;
	u16 flags;
	size_t count;
};

static ssize_t exynos_dsi_payload_write(struct file *file,
			       const char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	struct seq_file *m = file->private_data;
	struct exynos_dsi_reg_data *reg_data = m->private;
	char *buf;
	char *payload;
	size_t len;
	int ret;

	buf = memdup_user_nul(user_buf, count);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	/* calculate length for worst case (1 digit per byte + whitespace) */
	len = (count + 1) / 2;
	payload = kmalloc(len, GFP_KERNEL);
	if (!payload) {
		kfree(buf);
		return -ENOMEM;
	}

	ret = parse_byte_buf(payload, len, buf);
	if (ret <= 0) {
		ret = -EINVAL;
	} else if (reg_data->type) {
		ret = exynos_dsi_dcs_transfer(reg_data->dsi, reg_data->type,
					    payload, ret, reg_data->flags);
	} else {
		ret = exynos_dsi_dcs_write_buffer(reg_data->dsi, payload, ret,
						reg_data->flags);
	}

	kfree(buf);
	kfree(payload);

	return ret ? : count;
}

static int exynos_dsi_payload_show(struct seq_file *m, void *data)
{
	struct exynos_dsi_reg_data *reg_data = m->private;
	char *buf;
	ssize_t rc;

	if (!reg_data->count)
		return -EINVAL;

	buf = kmalloc(reg_data->count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	rc = mipi_dsi_dcs_read(reg_data->dsi, reg_data->address, buf,
			       reg_data->count);
	if (rc > 0) {
		seq_hex_dump(m, "", DUMP_PREFIX_NONE, 16, 1, buf, rc, false);
		rc = 0;
	} else if (rc == 0) {
		pr_debug("no response back\n");
	}
	kfree(buf);

	return 0;
}

static int exynos_dsi_payload_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_dsi_payload_show, inode->i_private);
}

static const struct file_operations exynos_dsi_payload_fops = {
	.owner		= THIS_MODULE,
	.open		= exynos_dsi_payload_open,
	.write		= exynos_dsi_payload_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int exynos_reset_panel(struct exynos_panel *ctx)
{
	if (!ctx) {
		pr_debug("reset_panel: exynos_panel not exist\n");
		return -EPERM;
	}

	if (IS_ERR_OR_NULL(ctx->reset_gpio)) {
		pr_debug("reset_panel: reset_gpio is invalid\n");
		return -EPERM;
	}

	gpiod_set_value(ctx->reset_gpio, 0);
	pr_info("reset_panel: pull reset_gpio to low to reset panel\n");

	return 0;
}

static ssize_t exynos_debugfs_reset_panel(struct file *file,
			       const char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	bool reset_panel;
	int ret;
	struct mipi_dsi_device *dsi = file->private_data;
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	if (!ctx->initialized || !ctx->enabled)
		return -EPERM;

	ret = kstrtobool_from_user(user_buf, count, &reset_panel);
	if (ret)
		return ret;

	if (reset_panel) {
		ret = exynos_reset_panel(ctx);
		if (ret) {
			pr_debug("reset_panel: reset panel failed\n");
			return ret;
		}
	}

	return count;
}

static const struct file_operations exynos_reset_panel_fops = {
	.open = simple_open,
	.write = exynos_debugfs_reset_panel,
};

static int exynos_dsi_debugfs_add(struct mipi_dsi_device *dsi,
			 struct dentry *parent)
{
	struct dentry *reg_root;
	struct exynos_dsi_reg_data *reg_data;

	reg_root = debugfs_create_dir("reg", parent);
	if (!reg_root)
		return -EFAULT;

	reg_data = devm_kzalloc(&dsi->dev, sizeof(*reg_data), GFP_KERNEL);
	if (!reg_data)
		return -ENOMEM;

	reg_data->dsi = dsi;
	reg_data->flags = MIPI_DSI_MSG_LASTCOMMAND;

	debugfs_create_u8("address", 0600, reg_root, &reg_data->address);
	debugfs_create_u8("type", 0600, reg_root, &reg_data->type);
	debugfs_create_size_t("count", 0600, reg_root, &reg_data->count);
	debugfs_create_u16("flags", 0600, reg_root, &reg_data->flags);
	debugfs_create_file("payload", 0600, reg_root, reg_data,
			    &exynos_dsi_payload_fops);

	debugfs_create_file("name", 0600, parent, dsi, &exynos_dsi_name_fops);
	debugfs_create_file("reset_panel",0200, parent, dsi, &exynos_reset_panel_fops);

	return 0;
}

static int exynos_debugfs_panel_add(struct exynos_panel *ctx, struct dentry *parent)
{
	struct dentry *root;

	if (!parent)
		return -EINVAL;

	root = debugfs_create_dir("panel", parent);
	if (!root)
		return -EPERM;

	ctx->debugfs_entry = root;

	return 0;
}

static void exynos_debugfs_panel_remove(struct exynos_panel *ctx)
{
	if (!ctx->debugfs_entry)
		return;

	debugfs_remove_recursive(ctx->debugfs_entry);

	ctx->debugfs_entry = NULL;
}
#else
static int panel_debugfs_add(struct exynos_panel *ctx, struct dentry *parent)
{
	return 0;
}

static int exynos_dsi_debugfs_add(struct mipi_dsi_device *dsi,
			 struct dentry *parent)
{
	return 0;
}

static int exynos_debugfs_panel_add(struct exynos_panel *ctx, struct dentry *parent)
{
	return 0;
}

static void exynos_debugfs_panel_remove(struct exynos_panel *ctx)
{
	return;
}
#endif

static int exynos_panel_attach_lp_mode(struct exynos_drm_connector *exynos_conn,
				       const struct drm_display_mode *lp_mode)
{
	struct exynos_drm_connector_properties *p =
		exynos_drm_connector_get_properties(exynos_conn);
	struct drm_mode_modeinfo umode;
	struct drm_property_blob *blob;

	if (!lp_mode)
		return -ENOENT;

	drm_mode_convert_to_umode(&umode, lp_mode);
	blob = drm_property_create_blob(exynos_conn->base.dev, sizeof(umode), &umode);
	if (IS_ERR(blob))
		return PTR_ERR(blob);

	drm_object_attach_property(&exynos_conn->base.base, p->lp_mode, blob->base.id);

	return 0;
}

static ssize_t hbm_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	const struct exynos_panel_mode *pmode;
	u32 hbm_mode;
	int ret;

	if (!funcs || !funcs->set_hbm_mode) {
		dev_err(ctx->dev, "HBM is not supported\n");
		return -ENOTSUPP;
	}

	mutex_lock(&ctx->mode_lock);
	pmode = ctx->current_mode;

	if (!ctx->enabled || !ctx->initialized || !pmode) {
		dev_err(ctx->dev, "panel is not enabled\n");
		ret = -EPERM;
		goto unlock;
	}

	if (pmode->exynos_mode.is_lp_mode) {
		dev_dbg(ctx->dev, "hbm unsupported in LP mode\n");
		ret = -EPERM;
		goto unlock;
	}

	ret = kstrtouint(buf, 0, &hbm_mode);
	if (ret || (hbm_mode >= HBM_STATE_MAX)) {
		dev_err(ctx->dev, "invalid hbm_mode value\n");
		goto unlock;
	}

	funcs->set_hbm_mode(ctx, hbm_mode);

	backlight_state_changed(bd);
unlock:
	mutex_unlock(&ctx->mode_lock);

	return ret ? : count;
}

static ssize_t hbm_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);

	return scnprintf(buf, PAGE_SIZE, "%u\n", ctx->hbm_mode);
}

static DEVICE_ATTR_RW(hbm_mode);

static ssize_t dimming_on_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);
	bool dimming_on;
	int ret;

	if (!ctx->enabled || !ctx->initialized) {
		dev_err(ctx->dev, "panel is not enabled\n");
		return -EPERM;
	}

	ret = kstrtobool(buf, &dimming_on);
	if (ret) {
		dev_err(ctx->dev, "invalid dimming_on value\n");
		return ret;
	}

	exynos_panel_set_dimming(ctx, dimming_on);

	return count;
}

static ssize_t dimming_on_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->dimming_on);
}
static DEVICE_ATTR_RW(dimming_on);

static ssize_t local_hbm_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	struct drm_mode_config *config;
	struct drm_crtc *crtc = NULL;
	bool local_hbm_en;
	int ret;

	if (!ctx->enabled || !ctx->initialized) {
		dev_err(ctx->dev, "panel is not enabled\n");
		return -EPERM;
	}

	if (!funcs || !funcs->set_local_hbm_mode) {
		dev_err(ctx->dev, "Local HBM is not supported\n");
		return -ENOTSUPP;
	}

	ret = kstrtobool(buf, &local_hbm_en);
	if (ret) {
		dev_err(ctx->dev, "invalid local_hbm_mode value\n");
		return ret;
	}

	dev_info(ctx->dev, "%s: set LHBM to %d\n", __func__, local_hbm_en);
	mutex_lock(&ctx->mode_lock);
	funcs->set_local_hbm_mode(ctx, local_hbm_en);
	mutex_unlock(&ctx->mode_lock);

	config = &ctx->exynos_connector.base.dev->mode_config;
	drm_modeset_lock(&config->connection_mutex, NULL);
	if (ctx->exynos_connector.base.state)
		crtc = ctx->exynos_connector.base.state->crtc;
	drm_modeset_unlock(&config->connection_mutex);
	if (crtc) {
		drm_crtc_wait_one_vblank(crtc);
		drm_crtc_wait_one_vblank(crtc);
	}

	sysfs_notify(&bd->dev.kobj, NULL, "local_hbm_mode");
	if (local_hbm_en) {
		queue_delayed_work(ctx->hbm.wq,
			 &ctx->hbm.local_hbm.timeout_work,
			 msecs_to_jiffies(ctx->hbm.local_hbm.max_timeout_ms));
	} else {
		cancel_delayed_work(&ctx->hbm.local_hbm.timeout_work);
	}

	return count;
}

static ssize_t local_hbm_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->hbm.local_hbm.enabled);
}
static DEVICE_ATTR_RW(local_hbm_mode);

static ssize_t local_hbm_max_timeout_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);
	int ret;

	ret = kstrtou32(buf, 0, &ctx->hbm.local_hbm.max_timeout_ms);
	if (ret) {
		dev_err(ctx->dev, "invalid local_hbm_max_timeout_ms value\n");
		return ret;
	}

	return count;
}

static ssize_t local_hbm_max_timeout_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bd);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ctx->hbm.local_hbm.max_timeout_ms);
}

static DEVICE_ATTR_RW(local_hbm_max_timeout);

static ssize_t state_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bl);
	bool show_mode = true;
	const char *statestr;
	int rc, ret_cnt;

	mutex_lock(&ctx->bl_state_lock);

	if (is_backlight_off_state(bl)) {
		statestr = "Off";
		show_mode = false;
	} else if (is_backlight_lp_state(bl)) {
		statestr = "LP";
	} else if (IS_HBM_ON(ctx->hbm_mode)) {
		statestr = IS_HBM_ON_IRC_OFF(ctx->hbm_mode) ?
				"HBM IRC_OFF" : "HBM";
	} else {
		statestr = "On";
	}

	mutex_unlock(&ctx->bl_state_lock);

	ret_cnt = scnprintf(buf, PAGE_SIZE, "%s\n", statestr);
	rc = ret_cnt;

	if (rc > 0 && show_mode) {
		const struct exynos_panel_mode *pmode;

		mutex_lock(&ctx->mode_lock);
		pmode = ctx->current_mode;
		mutex_unlock(&ctx->mode_lock);
		if (pmode) {
			/* overwrite \n and continue the string */
			const u8 str_len = ret_cnt - 1;

			ret_cnt = scnprintf(buf + str_len, PAGE_SIZE - str_len,
				      ": %dx%d@%d\n",
				      pmode->mode.hdisplay, pmode->mode.vdisplay,
				      exynos_get_actual_vrefresh(ctx));
			if (ret_cnt > 0)
				rc = str_len + ret_cnt;
		}
	}

	dev_dbg(ctx->dev, "%s: %s\n", __func__, rc > 0 ? buf : "");

	return rc;
}

static DEVICE_ATTR_RO(state);

static ssize_t lp_state_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bl);
	int rc;

	mutex_lock(&ctx->bl_state_lock);

	if (!is_backlight_lp_state(bl)) {
		dev_warn(ctx->dev, "panel is not in LP mode\n");
		mutex_unlock(&ctx->bl_state_lock);
		return -EPERM;
	}

	if (!ctx->current_binned_lp) {
		dev_warn(ctx->dev, "LP state is null\n");
		mutex_unlock(&ctx->bl_state_lock);
		return -EINVAL;
	}

	mutex_lock(&ctx->lp_state_lock);
	rc = scnprintf(buf, PAGE_SIZE, "%s\n", ctx->current_binned_lp->name);
	mutex_unlock(&ctx->lp_state_lock);

	mutex_unlock(&ctx->bl_state_lock);

	dev_dbg(ctx->dev, "%s: %s\n", __func__, buf);

	return rc;
}

static DEVICE_ATTR_RO(lp_state);

static int parse_u32_buf(char *src, size_t src_len, u32 *out, size_t out_len)
{
	int rc = 0, cnt = 0;
	char *str;
	const char *delim = " ";

	if (!src || !src_len || !out || !out_len)
		return -EINVAL;

	/* src_len is the length of src including null character '\0' */
	if (strnlen(src, src_len) == src_len)
		return -EINVAL;

	for (str = strsep(&src, delim); str != NULL; str = strsep(&src, delim)) {
		rc = kstrtou32(str, 0, out + cnt);
		if (rc)
			return -EINVAL;

		cnt++;

		if (out_len == cnt)
			break;
	}

	return cnt;
}

static ssize_t als_table_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bl);
	ssize_t bl_num_ranges;
	char *buf_dup;
	u32 ranges[MAX_BL_RANGES] = {0};
	u32 i;

	if (count == 0)
		return -EINVAL;

	buf_dup = kstrndup(buf, count, GFP_KERNEL);
	if (!buf_dup)
		return -ENOMEM;

	if (strlen(buf_dup) != count) {
		kfree(buf_dup);
		return -EINVAL;
	}

	bl_num_ranges = parse_u32_buf(buf_dup, count + 1,
				      ranges, MAX_BL_RANGES);
	if (bl_num_ranges < 0 || bl_num_ranges > MAX_BL_RANGES) {
		dev_warn(ctx->dev, "exceed max number of bl range\n");
		kfree(buf_dup);
		return -EINVAL;
	}

	mutex_lock(&ctx->bl_state_lock);

	ctx->bl_notifier.num_ranges = bl_num_ranges;
	for (i = 0; i < ctx->bl_notifier.num_ranges; i++)
		ctx->bl_notifier.ranges[i] = ranges[i];

	mutex_unlock(&ctx->bl_state_lock);

	kfree(buf_dup);

	return count;
}

static ssize_t als_table_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct exynos_panel *ctx = bl_get_data(bl);
	ssize_t rc = 0;
	size_t len = 0;
	u32 i = 0;

	mutex_lock(&ctx->bl_state_lock);

	for (i = 0; i < ctx->bl_notifier.num_ranges; i++) {
		rc = scnprintf(buf + len, PAGE_SIZE - len,
			       "%u ", ctx->bl_notifier.ranges[i]);
		if (rc < 0) {
			mutex_unlock(&ctx->bl_state_lock);
			return -EINVAL;
		}

		len += rc;
	}

	mutex_unlock(&ctx->bl_state_lock);

	len += scnprintf(buf + len, PAGE_SIZE - len, "\n");

	return len;
}

static DEVICE_ATTR_RW(als_table);

static struct attribute *bl_device_attrs[] = {
	&dev_attr_hbm_mode.attr,
	&dev_attr_dimming_on.attr,
	&dev_attr_local_hbm_mode.attr,
	&dev_attr_local_hbm_max_timeout.attr,
	&dev_attr_state.attr,
	&dev_attr_lp_state.attr,
	&dev_attr_als_table.attr,
	NULL,
};
ATTRIBUTE_GROUPS(bl_device);

static int exynos_panel_attach_brightness_capability(struct exynos_drm_connector *exynos_conn,
				const struct brightness_capability *brt_capability)
{
	struct exynos_drm_connector_properties *p =
		exynos_drm_connector_get_properties(exynos_conn);
	struct drm_property_blob *blob;

	blob = drm_property_create_blob(exynos_conn->base.dev,
				 sizeof(struct brightness_capability),
				 brt_capability);
	if (IS_ERR(blob))
		return PTR_ERR(blob);
	drm_object_attach_property(&exynos_conn->base.base, p->brightness_capability, blob->base.id);

	return 0;
}

static unsigned long get_backlight_state_from_panel(struct backlight_device *bl,
					enum exynos_panel_state panel_state)
{
	unsigned long state = bl->props.state;

	switch (panel_state) {
	case PANEL_STATE_ON:
		state &= ~(BL_STATE_STANDBY | BL_STATE_LP);
		break;
	case PANEL_STATE_LP:
		state &= ~(BL_STATE_STANDBY);
		state |= BL_STATE_LP;
		break;
	case PANEL_STATE_OFF:
		state &= ~(BL_STATE_LP);
		state |= BL_STATE_STANDBY;
		break;
	}

	return state;
}

static void exynos_panel_set_backlight_state(struct exynos_panel *ctx,
					enum exynos_panel_state panel_state)
{
	struct backlight_device *bl = ctx->bl;

	if (!bl)
		return;

	mutex_lock(&ctx->bl_state_lock);

	bl->props.state = get_backlight_state_from_panel(bl, panel_state);

	mutex_unlock(&ctx->bl_state_lock);

	backlight_state_changed(bl);

	dev_info(ctx->dev, "%s: panel:%d, bl:0x%x\n", __func__,
		 panel_state, bl->props.state);
}

static int exynos_panel_attach_properties(struct exynos_panel *ctx)
{
	struct exynos_drm_connector_properties *p =
		exynos_drm_connector_get_properties(&ctx->exynos_connector);
	struct drm_mode_object *obj = &ctx->exynos_connector.base.base;
	const struct exynos_panel_desc *desc = ctx->desc;
	int ret = 0;

	if (!p || !desc)
		return -ENOENT;

	drm_object_attach_property(obj, p->min_luminance, desc->min_luminance);
	drm_object_attach_property(obj, p->max_luminance, desc->max_luminance);
	drm_object_attach_property(obj, p->max_avg_luminance, desc->max_avg_luminance);
	drm_object_attach_property(obj, p->hdr_formats, desc->hdr_formats);
	drm_object_attach_property(obj, p->brightness_level, 0);
	drm_object_attach_property(obj, p->global_hbm_mode, 0);
	drm_object_attach_property(obj, p->local_hbm_on, 0);
	drm_object_attach_property(obj, p->dimming_on, 0);
	drm_object_attach_property(obj, p->sync_rr_switch, 0);
	drm_object_attach_property(obj, p->is_partial, desc->is_partial);

	if (desc->brt_capability) {
		ret = exynos_panel_attach_brightness_capability(&ctx->exynos_connector,
				desc->brt_capability);
		if (ret)
			dev_err(ctx->dev, "Failed to attach brightness capability (%d)\n", ret);
	}

	if (desc->lp_mode) {
		ret = exynos_panel_attach_lp_mode(&ctx->exynos_connector, &desc->lp_mode->mode);
		if (ret)
			dev_err(ctx->dev, "Failed to attach lp mode (%d)\n", ret);
	}

	return ret;
}

static int exynos_panel_bridge_attach(struct drm_bridge *bridge,
				      enum drm_bridge_attach_flags flags)
{
	struct drm_device *dev = bridge->dev;
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	struct drm_connector *connector = &ctx->exynos_connector.base;
	int ret;

	ret = exynos_drm_connector_init(dev, &ctx->exynos_connector,
					&exynos_panel_connector_funcs,
					&exynos_panel_connector_helper_funcs,
					DRM_MODE_CONNECTOR_DSI);
	if (ret) {
		dev_err(ctx->dev, "failed to initialize connector with drm\n");
		return ret;
	}

	ret = exynos_panel_attach_properties(ctx);
	if (ret) {
		dev_err(ctx->dev, "failed to attach connector properties\n");
		return ret;
	}

	drm_connector_helper_add(connector, &exynos_connector_helper_funcs);

	drm_connector_register(connector);

	drm_connector_attach_encoder(connector, bridge->encoder);
	connector->funcs->reset(connector);
	connector->status = connector_status_connected;
	connector->state->self_refresh_aware = true;
	if (ctx->desc->exynos_panel_func && ctx->desc->exynos_panel_func->commit_done)
		ctx->exynos_connector.needs_commit = true;

	ret = sysfs_create_link(&connector->kdev->kobj, &ctx->dev->kobj,
				"panel");
	if (ret)
		dev_warn(ctx->dev, "unable to link panel sysfs (%d)\n", ret);

	exynos_debugfs_panel_add(ctx, connector->debugfs_entry);
	exynos_dsi_debugfs_add(to_mipi_dsi_device(ctx->dev), ctx->debugfs_entry);
	panel_debugfs_add(ctx, ctx->debugfs_entry);

	drm_kms_helper_hotplug_event(connector->dev);

	if (!ctx->is_secondary)
		ret = sysfs_create_link(&bridge->dev->dev->kobj, &ctx->dev->kobj, "primary-panel");
	else
		ret = sysfs_create_link(&bridge->dev->dev->kobj, &ctx->dev->kobj, "secondary-panel");

	if (ret)
		dev_warn(ctx->dev, "unable to link %s sysfs (%d)\n",
			(!ctx->is_secondary) ? "primary-panel" : "secondary-panel", ret);
	return 0;
}

static void exynos_panel_bridge_detach(struct drm_bridge *bridge)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	struct drm_connector *connector = &ctx->exynos_connector.base;

	if (!ctx->is_secondary)
		sysfs_remove_link(&bridge->dev->dev->kobj, "primary-panel");
	else
		sysfs_remove_link(&bridge->dev->dev->kobj, "secondary-panel");

	exynos_debugfs_panel_remove(ctx);
	sysfs_remove_link(&connector->kdev->kobj, "panel");
	drm_connector_unregister(connector);
	drm_connector_cleanup(&ctx->exynos_connector.base);
}

static void exynos_panel_bridge_enable(struct drm_bridge *bridge,
				       struct drm_bridge_state *old_bridge_state)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	bool need_update_backlight = false;

	mutex_lock(&ctx->mode_lock);
	/* avoid turning on panel again if already enabled (ex. while booting or self refresh) */
	if (!ctx->enabled || exynos_panel_init(ctx)) {
		drm_panel_enable(&ctx->panel);
		need_update_backlight = true;
	}

	if (ctx->self_refresh_active) {
		dev_dbg(ctx->dev, "self refresh state : %s\n", __func__);

		ctx->self_refresh_active = false;
		panel_update_idle_mode_locked(ctx);
	} else {
		const bool is_lp_mode = ctx->current_mode &&
					ctx->current_mode->exynos_mode.is_lp_mode;
		enum exynos_panel_state panel_state = is_lp_mode ? PANEL_STATE_LP : PANEL_STATE_ON;

		exynos_panel_set_backlight_state(ctx, panel_state);

		/* For the case of OFF->AOD, TE2 will be updated in backlight_update_status */
		if (panel_state == PANEL_STATE_ON)
			exynos_panel_update_te2(ctx);
	}
	mutex_unlock(&ctx->mode_lock);

	if (need_update_backlight && ctx->bl)
		backlight_update_status(ctx->bl);
}

/*
 * this atomic check is called after adjusted mode is populated, so it's safe to modify
 * adjusted_mode if needed at this point
 */
static int exynos_panel_bridge_atomic_check(struct drm_bridge *bridge,
					    struct drm_bridge_state *bridge_state,
					    struct drm_crtc_state *new_crtc_state,
					    struct drm_connector_state *conn_state)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	struct drm_atomic_state *state = new_crtc_state->state;
	struct drm_crtc_state *old_crtc_state;
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	int ret;

	if (unlikely(!new_crtc_state))
		return 0;

	if (funcs && funcs->atomic_check) {
		ret = funcs->atomic_check(ctx, state);
		if (ret)
			return ret;
	}

	if (!drm_atomic_crtc_needs_modeset(new_crtc_state))
		return 0;

	old_crtc_state = drm_atomic_get_old_crtc_state(state, new_crtc_state->crtc);
	if (!old_crtc_state->enable && ctx->enabled)
		old_crtc_state->self_refresh_active = true;

	return exynos_drm_connector_check_mode(ctx, conn_state, &new_crtc_state->mode);
}

static void exynos_panel_bridge_pre_enable(struct drm_bridge *bridge,
					   struct drm_bridge_state *old_bridge_state)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);

	if (ctx->enabled)
		return;

	drm_panel_prepare(&ctx->panel);
}

static void exynos_panel_bridge_disable(struct drm_bridge *bridge,
					struct drm_bridge_state *old_bridge_state)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	const struct drm_connector_state *conn_state = ctx->exynos_connector.base.state;
	const bool self_refresh_active = conn_state->crtc && conn_state->crtc->state &&
		conn_state->crtc->state->self_refresh_active;

	if (self_refresh_active) {
		mutex_lock(&ctx->mode_lock);
		dev_dbg(ctx->dev, "self refresh state : %s\n", __func__);

		ctx->self_refresh_active = true;
		panel_update_idle_mode_locked(ctx);
		mutex_unlock(&ctx->mode_lock);
	} else {
		drm_panel_disable(&ctx->panel);
	}
}

static void exynos_panel_bridge_post_disable(struct drm_bridge *bridge,
					     struct drm_bridge_state *old_bridge_state)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	const struct drm_connector_state *conn_state = ctx->exynos_connector.base.state;
	const bool self_refresh_active = conn_state->crtc && conn_state->crtc->state &&
		conn_state->crtc->state->self_refresh_active;

	if (self_refresh_active) {
		dev_dbg(ctx->dev, "self refresh state : skip %s\n", __func__);
		return;
	}

	drm_panel_unprepare(&ctx->panel);

	exynos_panel_set_backlight_state(ctx, PANEL_STATE_OFF);
}

/* Get the VSYNC start time within a TE period */
static u64 exynos_panel_vsync_start_time_us(u32 te_period_us)
{
	/* Approximate the VSYNC start time with TE falling edge. Approximate
	 * the TE falling edge with 55% TE width
	 */
	/* TODO: Change to ctx->current_mode->exynos_mode.vblank_usec when it's accurate */
	return te_period_us * 55 / 100;
}

static void exynos_panel_check_modeset_timing(struct drm_crtc *crtc,
					 const struct drm_display_mode *old_mode)
{
	u32 te_period_us;
	int retry;
	u64 left, right;

	DPU_ATRACE_BEGIN(__func__);
	pr_debug("%s: check mode_set timing enter.\n", __func__);
	te_period_us = USEC_PER_SEC / drm_mode_vrefresh(old_mode);

	/*
	 * Safe time window to send RR (refresh rate) command illustrated below. RR switch
	 * and scanout need to happen in the same VSYNC period because the frame content might
	 * be adjusted specific to this RR.
	 *
	 * An estimation is [55% * TE_duration, TE_duration - 1ms] before driver has the
	 * accurate TE pulse width (VSYNC rising is a bit ahead of TE falling edge).
	 *
	 *         -->|     |<-- safe time window to send RR
	 *
	 *        +----+     +----+     +-+
	 *        |    |     |    |     | |
	 * TE   --+    +-----+    +-----+ +---
	 *               RR  SCANOUT
	 *
	 *            |          |       |
	 *            |          |       |
	 * VSYNC------+----------+-------+----
	 *            RR1        RR2
	 */
	left = exynos_panel_vsync_start_time_us(te_period_us);
	right = te_period_us - USEC_PER_MSEC;
	/* check for next TE every 1ms */
	retry = te_period_us / USEC_PER_MSEC + 1;

	do {
		ktime_t last_te = 0, now;
		s64 since_last_te_us;

		drm_crtc_vblank_count_and_time(crtc, &last_te);
		now = ktime_get();
		since_last_te_us = ktime_us_delta(now, last_te);
		if (since_last_te_us <= right) {
			if (since_last_te_us < left) {
				u32 delay_us = left - since_last_te_us;

				usleep_range(delay_us, delay_us + 100);
			}
			break;
		}
		/* retry in 1ms */
		usleep_range(USEC_PER_MSEC, USEC_PER_MSEC + 100);
	} while (--retry > 0);

	pr_debug("%s: check mode_set timing exit.\n", __func__);
	DPU_ATRACE_END(__func__);
}

static void panel_update_local_hbm_locked(struct exynos_panel *ctx, bool enabled)
{
	if (!ctx->desc->exynos_panel_func->set_local_hbm_mode)
		return;

	ctx->desc->exynos_panel_func->set_local_hbm_mode(ctx, enabled);
	sysfs_notify(&ctx->bl->dev.kobj, NULL, "local_hbm_mode");
	if (enabled) {
		queue_delayed_work(ctx->hbm.wq,
				&ctx->hbm.local_hbm.timeout_work,
				msecs_to_jiffies(ctx->hbm.local_hbm.max_timeout_ms));
	} else {
		cancel_delayed_work(&ctx->hbm.local_hbm.timeout_work);
	}
}

static void exynos_panel_bridge_mode_set(struct drm_bridge *bridge,
				  const struct drm_display_mode *mode,
				  const struct drm_display_mode *adjusted_mode)
{
	struct exynos_panel *ctx = bridge_to_exynos_panel(bridge);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	struct drm_connector_state *connector_state = ctx->exynos_connector.base.state;
	struct drm_crtc *crtc = connector_state->crtc;
	struct exynos_drm_connector_state *exynos_connector_state =
				      to_exynos_connector_state(connector_state);
	const struct exynos_panel_mode *pmode = exynos_panel_get_mode(ctx, mode);
	const struct exynos_panel_funcs *funcs = ctx->desc->exynos_panel_func;
	const struct exynos_panel_mode *current_mode;
	bool need_update_backlight = false;

	if (WARN_ON(!pmode))
		return;

	mutex_lock(&ctx->mode_lock);
	current_mode = ctx->current_mode;

	if (!ctx->initialized && ctx->enabled) {
		/* if panel was enabled at boot and there's no mode change skip mode set */
		if (current_mode == pmode) {
			mutex_unlock(&ctx->mode_lock);
			return;
		}

		WARN(1, "mode change at boot to %s\n", adjusted_mode->name);

		/*
		 * This is unexpected, but the best we can do is to set as disable which will
		 * force panel reset on next enable. That way it will go into new mode
		 */
		ctx->enabled = false;
		exynos_panel_set_power(ctx, false);
	}

	if (current_mode == pmode) {
		mutex_unlock(&ctx->mode_lock);
		return;
	}

	dev_dbg(ctx->dev, "changing display mode to %dx%d@%d\n",
		pmode->mode.hdisplay, pmode->mode.vdisplay, drm_mode_vrefresh(&pmode->mode));

	dsi->mode_flags = pmode->exynos_mode.mode_flags;
	ctx->last_mode_set_ts = ktime_get();

	DPU_ATRACE_BEGIN(__func__);
	if (funcs) {
		const bool was_lp_mode = current_mode &&
					 current_mode->exynos_mode.is_lp_mode;
		const bool is_lp_mode = pmode->exynos_mode.is_lp_mode;
		bool state_changed = false;

		if (is_lp_mode && funcs->set_lp_mode) {
			if (ctx->hbm.local_hbm.enabled && funcs->set_local_hbm_mode) {
				dev_warn(ctx->dev,
					"LHBM is on when switching to LP mode(%s), turn off LHBM first\n",
					pmode->mode.name);
				panel_update_local_hbm_locked(ctx, false);
			}
			funcs->set_lp_mode(ctx, pmode);
			if (ctx->vddd && ctx->vddd_lp_uV)
				regulator_set_voltage(ctx->vddd, ctx->vddd_lp_uV, ctx->vddd_lp_uV);
			need_update_backlight = true;
		} else if (was_lp_mode && !is_lp_mode && funcs->set_nolp_mode) {
			if (ctx->vddd && ctx->vddd_normal_uV)
				regulator_set_voltage(ctx->vddd, ctx->vddd_normal_uV,
						      ctx->vddd_normal_uV);
			funcs->set_nolp_mode(ctx, pmode);
			state_changed = true;
			need_update_backlight = true;
		} else if (funcs->mode_set) {
			if (exynos_connector_state->sync_rr_switch && ctx->enabled)
				exynos_panel_check_modeset_timing(crtc, &current_mode->mode);
			funcs->mode_set(ctx, pmode);
			state_changed = true;
		}

		ctx->current_mode = pmode;

		if (state_changed) {
			if (was_lp_mode)
				exynos_panel_set_backlight_state(ctx, ctx->enabled ?
								 PANEL_STATE_ON : PANEL_STATE_OFF);
			else if (ctx->bl)
				backlight_state_changed(ctx->bl);

			if (!is_lp_mode)
				exynos_panel_update_te2(ctx);
		}
	} else {
		ctx->current_mode = pmode;
	}

	mutex_unlock(&ctx->mode_lock);

	if (need_update_backlight && ctx->bl)
		backlight_update_status(ctx->bl);

	DPU_ATRACE_INT("panel_fps", drm_mode_vrefresh(mode));
	DPU_ATRACE_END(__func__);
}

static void local_hbm_timeout_work(struct work_struct *work)
{
	struct exynos_panel *ctx =
			 container_of(work, struct exynos_panel, hbm.local_hbm.timeout_work.work);

	dev_dbg(ctx->dev, "%s\n", __func__);

	dev_info(ctx->dev, "%s: turn off LHBM\n", __func__);
	mutex_lock(&ctx->mode_lock);
	ctx->desc->exynos_panel_func->set_local_hbm_mode(ctx, false);
	mutex_unlock(&ctx->mode_lock);
	sysfs_notify(&ctx->bl->dev.kobj, NULL, "local_hbm_mode");
}

static void hbm_work(struct work_struct *work)
{
	struct exynos_panel *ctx =
			 container_of(work, struct exynos_panel, hbm.hbm_work);
	const struct exynos_panel_funcs *exynos_panel_func = ctx->desc->exynos_panel_func;
	struct drm_crtc_commit *commit = ctx->hbm.commit;
	u32 delay_us, timeout_ms, te_period_us;
	int fps;

	dev_dbg(ctx->dev, "%s (update_flags: 0x%02x)\n", __func__, ctx->hbm.update_flags);

	mutex_lock(&ctx->mode_lock);
	fps = drm_mode_vrefresh(&ctx->current_mode->mode);
	mutex_unlock(&ctx->mode_lock);
	WARN_ON(fps < 0);
	if (fps <= 0)
		fps = 1;
	te_period_us = USEC_PER_SEC / fps;

	/* delay begins at TE rising, ends at VSYNC rising */
	delay_us = exynos_panel_vsync_start_time_us(te_period_us);
	timeout_ms = te_period_us / USEC_PER_MSEC + 20;

	/* considering the variation */
	delay_us = delay_us * 105 / 100;

	DPU_ATRACE_BEGIN("ghbm");
	mutex_lock(&ctx->hbm.hbm_work_lock);

	WARN_ON(!commit);

	if (commit) {
		int ret;

		DPU_ATRACE_BEGIN("wait_for_flip");
		ctx->hbm.commit = NULL;
		ret = wait_for_completion_timeout(&commit->flip_done,
						  msecs_to_jiffies(timeout_ms));
		WARN_ON(ret < 0);
		drm_crtc_commit_put(commit);
		DPU_ATRACE_END("wait_for_flip");
	}

	usleep_range(delay_us, delay_us + 100);
	if (ctx->hbm.update_flags & HBM_FLAG_GHBM_UPDATE) {
		DPU_ATRACE_BEGIN("set_hbm");
		mutex_lock(&ctx->mode_lock);
		exynos_panel_func->set_hbm_mode(ctx, ctx->hbm.request_global_hbm_mode);
		backlight_state_changed(ctx->bl);
		mutex_unlock(&ctx->mode_lock);
		DPU_ATRACE_END("set_hbm");
	}

	if (ctx->hbm.update_flags & HBM_FLAG_BL_UPDATE) {
		DPU_ATRACE_BEGIN("set_bl");
		backlight_update_status(ctx->bl);
		DPU_ATRACE_END("set_bl");
	}

	if (ctx->hbm.update_flags & HBM_FLAG_LHBM_UPDATE) {
		DPU_ATRACE_BEGIN("set_lhbm");
		dev_info(ctx->dev, "%s: set LHBM to %d (%dHz)\n",
			__func__, ctx->hbm.local_hbm.request_hbm_mode, fps);
		mutex_lock(&ctx->mode_lock);
		panel_update_local_hbm_locked(ctx, ctx->hbm.local_hbm.request_hbm_mode);
		mutex_unlock(&ctx->mode_lock);
		DPU_ATRACE_END("set_lhbm");
	}

	if (ctx->hbm.update_flags & HBM_FLAG_DIMMING_UPDATE) {
		DPU_ATRACE_BEGIN("set_dimming");
		exynos_panel_set_dimming(ctx, ctx->request_dimming_on);
		DPU_ATRACE_END("set_dimming");
	}

	ctx->hbm.update_flags = 0;
	mutex_unlock(&ctx->hbm.hbm_work_lock);

	DPU_ATRACE_END("hbm");
}

static void hbm_data_init(struct exynos_panel *ctx)
{
	ctx->hbm.local_hbm.gamma_para_ready = false;
	ctx->hbm.local_hbm.max_timeout_ms = LOCAL_HBM_MAX_TIMEOUT_MS;
	ctx->hbm.local_hbm.enabled = false;
	ctx->hbm.wq = create_singlethread_workqueue("hbm_workq");
	if (!ctx->hbm.wq)
		dev_err(ctx->dev, "failed to create hbm workq!\n");
	else {
		INIT_DELAYED_WORK(&ctx->hbm.local_hbm.timeout_work, local_hbm_timeout_work);
		INIT_WORK(&ctx->hbm.hbm_work, hbm_work);
	}
	ctx->hbm.update_flags = 0;
	mutex_init(&ctx->hbm.hbm_work_lock);
}

static void exynos_panel_te2_init(struct exynos_panel *ctx)
{
	struct te2_mode_data *data;
	const struct exynos_binned_lp *binned_lp;
	int i;
	int lp_idx = ctx->desc->num_modes;

	for (i = 0; i < ctx->desc->num_modes; i++) {
		const struct exynos_panel_mode *pmode = &ctx->desc->modes[i];

		data = &ctx->te2.mode_data[i];
		data->mode = &pmode->mode;
		data->timing.rising_edge = pmode->te2_timing.rising_edge;
		data->timing.falling_edge = pmode->te2_timing.falling_edge;
	}

	for_each_exynos_binned_lp(i, binned_lp, ctx) {
		/* ignore the first binned entry (off) */
		if (i == 0)
			continue;

		data = &ctx->te2.mode_data[i - 1 + lp_idx];
		data->mode = &ctx->desc->lp_mode->mode;
		data->binned_lp = binned_lp;
		data->timing.rising_edge =
				binned_lp->te2_timing.rising_edge;
		data->timing.falling_edge =
				binned_lp->te2_timing.falling_edge;
	}
}

static const struct drm_bridge_funcs exynos_panel_bridge_funcs = {
	.attach = exynos_panel_bridge_attach,
	.detach = exynos_panel_bridge_detach,
	.atomic_check = exynos_panel_bridge_atomic_check,
	.atomic_pre_enable = exynos_panel_bridge_pre_enable,
	.atomic_enable = exynos_panel_bridge_enable,
	.atomic_disable = exynos_panel_bridge_disable,
	.atomic_post_disable = exynos_panel_bridge_post_disable,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.mode_set = exynos_panel_bridge_mode_set,
};

int exynos_panel_common_init(struct mipi_dsi_device *dsi,
				struct exynos_panel *ctx)
{
	static atomic_t panel_index = ATOMIC_INIT(-1);
	struct device *dev = &dsi->dev;
	int ret = 0;
	char name[32];
	const struct exynos_panel_funcs *exynos_panel_func;
	int i;

	dev_dbg(dev, "%s +\n", __func__);

	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dev = dev;
	ctx->desc = of_device_get_match_data(dev);

	dsi->lanes = ctx->desc->data_lane_cnt;
	dsi->format = MIPI_DSI_FMT_RGB888;

	ret = exynos_panel_parse_dt(ctx);
	if (ret)
		return ret;

	scnprintf(name, sizeof(name), "panel%d-backlight", atomic_inc_return(&panel_index));

	ctx->bl = devm_backlight_device_register(ctx->dev, name, dev,
			ctx, &exynos_backlight_ops, NULL);
	if (IS_ERR(ctx->bl)) {
		dev_err(ctx->dev, "failed to register backlight device\n");
		return PTR_ERR(ctx->bl);
	}
	ctx->bl->props.max_brightness = ctx->desc->max_brightness;
	ctx->bl->props.brightness = ctx->desc->dft_brightness;

	exynos_panel_func = ctx->desc->exynos_panel_func;
	if (exynos_panel_func && (exynos_panel_func->set_hbm_mode
				  || exynos_panel_func->set_local_hbm_mode))
		hbm_data_init(ctx);

	if (exynos_panel_func && exynos_panel_func->get_te2_edges &&
	    exynos_panel_func->configure_te2_edges &&
	    exynos_panel_func->update_te2)
		exynos_panel_te2_init(ctx);

	if (ctx->desc->bl_num_ranges) {
		ctx->bl_notifier.num_ranges = ctx->desc->bl_num_ranges;
		if (ctx->bl_notifier.num_ranges > MAX_BL_RANGES) {
			dev_warn(ctx->dev, "exceed max number of bl range\n");
			ctx->bl_notifier.num_ranges = MAX_BL_RANGES;
		}

		for (i = 0; i < ctx->bl_notifier.num_ranges; i++)
			ctx->bl_notifier.ranges[i] = ctx->desc->bl_range[i];
	}
	ctx->panel_idle_enabled = exynos_panel_func && exynos_panel_func->set_self_refresh != NULL;
	INIT_DELAYED_WORK(&ctx->idle_work, panel_idle_work);

	mutex_init(&ctx->mode_lock);
	mutex_init(&ctx->bl_state_lock);
	mutex_init(&ctx->lp_state_lock);

	drm_panel_init(&ctx->panel, dev, ctx->desc->panel_func, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ctx->bridge.funcs = &exynos_panel_bridge_funcs;
#ifdef CONFIG_OF
	ctx->bridge.of_node = ctx->dev->of_node;
#endif
	drm_bridge_add(&ctx->bridge);

	ret = sysfs_create_files(&dev->kobj, panel_attrs);
	if (ret)
		pr_warn("unable to add panel sysfs files (%d)\n", ret);

	ret = sysfs_create_groups(&ctx->bl->dev.kobj, bl_device_groups);
	if (ret)
		dev_err(ctx->dev, "unable to create bl_device_groups groups\n");

	exynos_panel_handoff(ctx);

	ret = mipi_dsi_attach(dsi);
	if (ret)
		goto err_panel;

	dev_info(ctx->dev, "samsung common panel driver has been probed\n");

	return 0;

err_panel:
	drm_panel_remove(&ctx->panel);
	dev_err(ctx->dev, "failed to probe samsung panel driver(%d)\n", ret);

	return ret;
}
EXPORT_SYMBOL(exynos_panel_common_init);

int exynos_panel_probe(struct mipi_dsi_device *dsi)
{
	struct exynos_panel *ctx;

	ctx = devm_kzalloc(&dsi->dev, sizeof(struct exynos_panel), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	return exynos_panel_common_init(dsi, ctx);
}
EXPORT_SYMBOL(exynos_panel_probe);

int exynos_panel_remove(struct mipi_dsi_device *dsi)
{
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
	drm_bridge_remove(&ctx->bridge);

	sysfs_remove_groups(&ctx->bl->dev.kobj, bl_device_groups);
	devm_backlight_device_unregister(ctx->dev, ctx->bl);

	return 0;
}
EXPORT_SYMBOL(exynos_panel_remove);

MODULE_AUTHOR("Jiun Yu <jiun.yu@samsung.com>");
MODULE_DESCRIPTION("MIPI-DSI based Samsung common panel driver");
MODULE_LICENSE("GPL");
