// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020-2021 Google LLC.
 *
 * Author: Sidath Senanayake <sidaths@google.com>
 */

/* Linux includes */
#include <linux/of_device.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#endif
#include <linux/pm_domain.h>

/* SOC includes */
#if IS_ENABLED(CONFIG_EXYNOS_PMU_IF)
#include <soc/google/exynos-pmu-if.h>
#endif
#if IS_ENABLED(CONFIG_CAL_IF)
#include <soc/google/cal-if.h>
#endif

/* Mali core includes */
#include <mali_kbase.h>

/* Pixel integration includes */
#include "mali_kbase_config_platform.h"
#include "pixel_gpu_control.h"
#include "pixel_gpu_trace.h"

/*
 * GPU_PM_DOMAIN_NAMES - names for GPU power domains.
 *
 * This array of names is used to match up devicetree defined power domains with their
 * representation in the Mali GPU driver. The names here must have a one to one mapping with the
 * 'power-domain-names' entry in the GPU's devicetree entry.
 */
static const char * const GPU_PM_DOMAIN_NAMES[GPU_PM_DOMAIN_COUNT] = {
	"top", "cores"
};

/**
 * gpu_pm_power_on_cores() - Powers on the GPU shader cores.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Powers on the CORES domain and issues trace points and events. Also powers on TOP and cancels
 * any pending suspend operations on it.
 *
 * Context: Process context. Takes and releases PM lock.
 *
 * Return: If GPU state has been lost, 1 is returned. Otherwise 0 is returned.
 */
static int gpu_pm_power_on_cores(struct kbase_device *kbdev)
{
	int ret;
	struct pixel_context *pc = kbdev->platform_context;
	u64 start_ns = ktime_get_ns();

	mutex_lock(&pc->pm.lock);

	pm_runtime_get_sync(pc->pm.domain_devs[GPU_PM_DOMAIN_TOP]);
	pm_runtime_get_sync(pc->pm.domain_devs[GPU_PM_DOMAIN_CORES]);

	/*
	 * We determine whether GPU state was lost by detecting whether the GPU state reached
	 * GPU_POWER_LEVEL_OFF before we entered this function. The GPU state is set to be
	 * GPU_POWER_LEVEL_OFF in the gpu_pm_callback_power_runtime_suspend callback which is run
	 * when autosuspend for TOP is triggered.
	 *
	 * As such, GPU state is only checked at this point in the code (and not at the start) as it
	 * is after pm_runtime_get_sync() on the TOP domain has been called. If there was an
	 * autosuspend in progress for TOP, then the call to pm_runtime_get_sync() would have
	 * blocked until it completed ensuring that the value of pc->pm.state is up-to-date.
	 */
	ret = (pc->pm.state == GPU_POWER_LEVEL_OFF);

	trace_gpu_power_state(ktime_get_ns() - start_ns,
		GPU_POWER_LEVEL_GLOBAL, GPU_POWER_LEVEL_STACKS);
#ifdef CONFIG_MALI_MIDGARD_DVFS
	gpu_dvfs_event_power_on(kbdev);
#endif

#if IS_ENABLED(CONFIG_GOOGLE_BCL)
	if (pc->pm.bcl_dev)
		google_init_gpu_ratio(pc->pm.bcl_dev);
#endif

	pc->pm.state = GPU_POWER_LEVEL_STACKS;

	mutex_unlock(&pc->pm.lock);

	return ret;
}

/**
 * gpu_pm_power_off_cores() - Powers off the GPU shader cores.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Powers off the CORES domain and issues trace points and events. Also marks the TOP domain for
 * delayed suspend. Complete power down of all GPU domains will only occur after this delayed
 * suspend, and the kernel notifies of this change via the &gpu_pm_callback_power_runtime_suspend
 * callback.
 *
 * Note: If the we have already performed these operations without an intervening call to
 *       &gpu_pm_power_on_cores, then we take no action.
 *
 * Context: Process context. Takes and releases the PM lock.
 */
static void gpu_pm_power_off_cores(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;
	u64 start_ns = ktime_get_ns();

	mutex_lock(&pc->pm.lock);

	if (pc->pm.state > GPU_POWER_LEVEL_GLOBAL) {
		pm_runtime_put_sync(pc->pm.domain_devs[GPU_PM_DOMAIN_CORES]);
		pc->pm.state = GPU_POWER_LEVEL_GLOBAL;

		pm_runtime_mark_last_busy(pc->pm.domain_devs[GPU_PM_DOMAIN_TOP]);
		pm_runtime_put_autosuspend(pc->pm.domain_devs[GPU_PM_DOMAIN_TOP]);

		trace_gpu_power_state(ktime_get_ns() - start_ns,
			GPU_POWER_LEVEL_STACKS, GPU_POWER_LEVEL_GLOBAL);
#ifdef CONFIG_MALI_MIDGARD_DVFS
		gpu_dvfs_event_power_off(kbdev);
#endif
	}

	mutex_unlock(&pc->pm.lock);
}

/**
 * gpu_pm_callback_power_on() - Called when the GPU needs to be powered on.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * This callback is called by the core Mali driver when it identifies that the GPU is about to
 * become active.
 *
 * Since we are using idle hints to power down the GPU in &gpu_pm_callback_power_off we will need to
 * power up the GPU when we receive this callback.
 *
 * If we detect that we are being called after TOP has been powered off, we indicate to the caller
 * that the GPU state has been lost.
 *
 * Return: If GPU state has been lost, 1 is returned. Otherwise 0 is returned.
 */
static int gpu_pm_callback_power_on(struct kbase_device *kbdev)
{
	dev_dbg(kbdev->dev, "%s\n", __func__);

	return gpu_pm_power_on_cores(kbdev);
}

/**
 * gpu_pm_callback_power_off() - Called when the GPU is idle and may be powered off
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * This callback is called by the core Mali driver when it identifies that the GPU is idle and may
 * be powered off.
 *
 * We take this opportunity to power down the CORES domain to allow for inter-frame power downs that
 * save power.
 */
static void gpu_pm_callback_power_off(struct kbase_device *kbdev)
{
	dev_dbg(kbdev->dev, "%s\n", __func__);

	gpu_pm_power_off_cores(kbdev);
}

/**
 * gpu_pm_callback_power_suspend() - Called when the system is going to suspend
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * This callback is called by the core Mali driver when it is notified that the system is about to
 * suspend and the GPU needs to be powered down.
 *
 * We manage 2 logical power domains; an SOC might have more physical domains, but they will be
 * grouped into these two domains.
 *
 *   1. the TOP or front-end domain, which holds useful state even when the GPU is idle.
 *   2. the CORES or back-end domain, which has no persistent state between tasks.
 *
 * GPU state is stored in the TOP power domain. This domain is powered whenever the SOC is not in
 * suspend, so that we don't have to restore state when we have new work. The CORES domain is
 * powered off when the GPU is idle in order to save power.
 *
 * This callback is called when the SOC is about to suspend which will result in GPU state being
 * lost. As the core Mali driver doesn't guarantee that &gpu_pm_callback_power_off will be called as
 * well, all operations made in that function are made in this callback too if CORES is still
 * powered. In addition, we also record that state will be lost and power down the TOP domain.
 *
 * Logging the GPU state in this way enables an optimization where GPU state is only reconstructed
 * if necessary when the GPU is powered on by &gpu_pm_callback_power_on. This saves CPU cycles and
 * reduces power on latency.
 */
static void gpu_pm_callback_power_suspend(struct kbase_device *kbdev)
{
	dev_dbg(kbdev->dev, "%s\n", __func__);

	gpu_pm_power_off_cores(kbdev);
}

#ifdef KBASE_PM_RUNTIME

/**
 * gpu_pm_callback_power_runtime_suspend() - Called when a TOP domain is going to runtime suspend
 *
 * @dev: The device that is going to runtime suspend
 *
 * This callback is made when @dev is about to enter runtime suspend. In our case, this occurs when
 * the TOP domain of GPU is about to enter runtime suspend. At this point we take the opportunity
 * to store that state will be lost and disable DVFS metrics gathering.
 *
 * Note: This function doesn't take the PM lock prior to updating GPU state as it doesn't explicitly
 *       attempt to update GPU power domain state. The caller of this function (or another function
 *       further up the callstack) will hold &power.lock for the TOP domain's &struct device and
 *       that is sufficient for ensuring serialization of the GPU power state.
 *
 * Return: Always returns 0.
 */
static int gpu_pm_callback_power_runtime_suspend(struct device *dev)
{
	struct kbase_device *kbdev = dev_get_drvdata(dev);
	struct pixel_context *pc = kbdev->platform_context;

	dev_dbg(kbdev->dev, "%s\n", __func__);

	WARN_ON(pc->pm.state > GPU_POWER_LEVEL_GLOBAL);
	pc->pm.state = GPU_POWER_LEVEL_OFF;

#ifdef CONFIG_MALI_MIDGARD_DVFS
	kbase_pm_metrics_stop(kbdev);
#endif

	return 0;
}

/**
 * gpu_pm_callback_power_runtime_resume() - Called when a TOP domain is going to runtime resume
 *
 * @dev: The device that is going to runtime suspend
 *
 * This callback is made when @dev is about to runtime resume. In our case, this occurs when
 * the TOP domain of GPU is about to runtime resume. We use this callback to enable DVFS metrics
 * gathering.
 *
 * Return: Always returns 0.
 */
static int gpu_pm_callback_power_runtime_resume(struct device *dev)
{
#ifdef CONFIG_MALI_MIDGARD_DVFS
	struct kbase_device *kbdev = dev_get_drvdata(dev);

	kbase_pm_metrics_start(kbdev);
#endif
	return 0;
}

/**
 * gpu_pm_callback_power_runtime_init() - Initialize runtime power management.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * This callback is made by the core Mali driver at the point where runtime power management is
 * being initialized early on in the probe of the Mali device.
 *
 * We enable autosuspend for the TOP domain so that after the autosuspend delay, the core Mali
 * driver knows to disable the collection of GPU utilization data used for DVFS purposes.
 *
 * Return: Returns 0 on success, or an error code on failure.
 */
static int gpu_pm_callback_power_runtime_init(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;

	dev_dbg(kbdev->dev, "%s\n", __func__);

	pm_runtime_set_autosuspend_delay(pc->pm.domain_devs[GPU_PM_DOMAIN_TOP],
		pc->pm.autosuspend_delay);
	pm_runtime_use_autosuspend(pc->pm.domain_devs[GPU_PM_DOMAIN_TOP]);

	if (!pm_runtime_enabled(pc->pm.domain_devs[GPU_PM_DOMAIN_TOP]) ||
		!pm_runtime_enabled(pc->pm.domain_devs[GPU_PM_DOMAIN_CORES])) {
		dev_warn(kbdev->dev, "pm_runtime not enabled\n");
		return -ENOSYS;
	}

	return 0;
}

/**
 * kbase_device_runtime_term() - Initialize runtime power management.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * This callback is made via the core Mali driver at the point where runtime power management needs
 * to be de-initialized. Currently this only happens if the device probe fails at a point after
 * which runtime power management has been initialized.
 */
static void gpu_pm_callback_power_runtime_term(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;

	dev_dbg(kbdev->dev, "%s\n", __func__);

	pm_runtime_disable(pc->pm.domain_devs[GPU_PM_DOMAIN_CORES]);
	pm_runtime_disable(pc->pm.domain_devs[GPU_PM_DOMAIN_TOP]);
}

#endif /* KBASE_PM_RUNTIME */

/*
 * struct pm_callbacks - Callbacks for linking to core Mali KMD power management
 *
 * Callbacks linking power management code in the core Mali driver with code in The Pixel
 * integration. For more information on the fields below, see the documentation for each function
 * assigned below, and &struct kbase_pm_callback_conf.
 *
 * Currently we power down the GPU when the core Mali driver indicates that the GPU is idle. This is
 * indicated by the core Mali driver via &power_off_callback and actioned in this integration via
 * &gpu_pm_callback_power_off. Similarly, the GPU is powered on in the mirror callback
 * &power_on_callback and actioned by &gpu_pm_callback_power_on.
 *
 * We also provide a callback for &power_suspend_callback since this call is made when the system is
 * going to suspend which will result in the GPU state being lost. We need to log this so that when
 * the GPU comes on again we can indicate to the core Mali driver that the GPU state needs to be
 * reconstructed. See the documentation for &gpu_pm_callback_power_suspend for more information.
 *
 * Since all power operations are handled in the most aggressive manner, &power_resume_callback, is
 * not needed and set to NULL.
 *
 * For runtime PM operations, we use virtual devices mapped to the two GPU power domains (TOP and
 * CORES) instead, and so all runtime PM callbacks as defined in &struct dev_pm_ops are set to NULL
 * here. Note that &power_runtime_init_callback and &power_runtime_term_callback are constructs of
 * the Mali GPU driver and not present in &struct dev_pm_ops despite their naming similarity. We do
 * define these as they link initialization in this file with the probe of the GPU device.
 *
 * Finally, we set &soft_reset_callback to NULL as we do not need to perform a custom soft reset,
 * and can rely on this being handled in the default way by the core Mali driver.
 */
struct kbase_pm_callback_conf pm_callbacks = {
	.power_off_callback = gpu_pm_callback_power_off,
	.power_on_callback = gpu_pm_callback_power_on,
	.power_suspend_callback = gpu_pm_callback_power_suspend,
	.power_resume_callback = NULL,
#ifdef KBASE_PM_RUNTIME
	.power_runtime_init_callback = gpu_pm_callback_power_runtime_init,
	.power_runtime_term_callback = gpu_pm_callback_power_runtime_term,
	.power_runtime_off_callback = NULL,
	.power_runtime_on_callback = NULL,
	.power_runtime_idle_callback = NULL,
#else /* KBASE_PM_RUNTIME */
	.power_runtime_init_callback = NULL,
	.power_runtime_term_callback = NULL,
	.power_runtime_off_callback = NULL,
	.power_runtime_on_callback = NULL,
	.power_runtime_idle_callback = NULL,
#endif /* KBASE_PM_RUNTIME */
	.soft_reset_callback = NULL
};

/**
 * gpu_pm_get_pm_cores_domain() - Find the GPU's power domain.
 *
 * @g3d_genpd_name: A string containing the name of the power domain
 *
 * Searches through the available power domains in device tree for one that
 * matched @g3d_genpd_name and returns it if found.
 *
 * Return: A pointer to the power domain if found, NULL otherwise.
 */
static struct exynos_pm_domain *gpu_pm_get_pm_cores_domain(const char *g3d_genpd_name)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct exynos_pm_domain *pd;

	for_each_compatible_node(np, NULL, "samsung,exynos-pd") {
		if (of_device_is_available(np)) {
			pdev = of_find_device_by_node(np);
			pd = (struct exynos_pm_domain *)platform_get_drvdata(pdev);
			if (strcmp(g3d_genpd_name, (const char *)(pd->genpd.name)) == 0)
				return pd;
		}
	}

	return NULL;
}

/**
 * gpu_pm_get_power_state() - Returns the current power state of a GPU.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Context: Process context. Takes and releases the power domain access lock.
 *
 * Return: Returns true if the GPU is powered on, false if not.
 */
bool gpu_pm_get_power_state(struct kbase_device *kbdev)
{
	bool ret;
	unsigned int val = 0;
	struct pixel_context *pc = kbdev->platform_context;

	mutex_lock(&pc->pm.domain->access_lock);
	exynos_pmu_read(pc->pm.status_reg_offset, &val);
	ret = ((val & pc->pm.status_local_power_mask) == pc->pm.status_local_power_mask);
	mutex_unlock(&pc->pm.domain->access_lock);

	return ret;
}


/**
 * gpu_pm_init() - Initializes power management control for a GPU.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Return: An error code, or 0 on success.
 */
int gpu_pm_init(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;
	struct device_node *np = kbdev->dev->of_node;
	const char *g3d_power_domain_name;
	int i, num_pm_domains;
	int ret = 0;

	/* Initialize lock */
	mutex_init(&pc->pm.lock);

	num_pm_domains = of_count_phandle_with_args(np, "power-domains", "#power-domain-cells");
	if (num_pm_domains != GPU_PM_DOMAIN_COUNT) {
		dev_err(kbdev->dev, "incorrect number of power domains in DT actual=%d expected=%d",
				num_pm_domains, GPU_PM_DOMAIN_COUNT);
		return -EINVAL;
	}

	for (i = 0; i < GPU_PM_DOMAIN_COUNT; i++) {
		pc->pm.domain_devs[i] = dev_pm_domain_attach_by_name(kbdev->dev,
			GPU_PM_DOMAIN_NAMES[i]);

		if (IS_ERR_OR_NULL(pc->pm.domain_devs[i])) {
			if (IS_ERR(pc->pm.domain_devs[i]))
				ret = PTR_ERR(pc->pm.domain_devs[i]);
			else
				ret = -EINVAL;

			dev_err(kbdev->dev, "failed to attach pm domain %s: %d\n",
				GPU_PM_DOMAIN_NAMES[i], ret);

			pc->pm.domain_devs[i] = NULL;
			goto error;
		}

		dev_set_drvdata(pc->pm.domain_devs[i], kbdev);

		pc->pm.domain_links[i] = device_link_add(kbdev->dev,
			pc->pm.domain_devs[i], DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME);

		if (!pc->pm.domain_links[i]) {
			dev_err(kbdev->dev, "failed to link pm domain device");
			ret = -EINVAL;
			goto error;
		}
	}

	/*
	 * We set up runtime pm callbacks specifically for the TOP domain. This is so that when we
	 * use autosupend it will only affect the TOP domain and not CORES as we control the power
	 * state of CORES directly.
	 */
	pc->pm.domain_devs[GPU_PM_DOMAIN_TOP]->pm_domain->ops.runtime_suspend =
		&gpu_pm_callback_power_runtime_suspend;
	pc->pm.domain_devs[GPU_PM_DOMAIN_TOP]->pm_domain->ops.runtime_resume =
		&gpu_pm_callback_power_runtime_resume;

	if (of_property_read_u32(np, "gpu_pm_autosuspend_delay", &pc->pm.autosuspend_delay)) {
		pc->pm.autosuspend_delay = AUTO_SUSPEND_DELAY;
		dev_info(kbdev->dev, "autosuspend delay not set in DT, using default of %dms\n",
			AUTO_SUSPEND_DELAY);
	}

	if (of_property_read_u32(np, "gpu_pmu_status_reg_offset", &pc->pm.status_reg_offset)) {
		dev_err(kbdev->dev, "PMU status register offset not set in DT\n");
		ret = -EINVAL;
		goto error;
	}

	if (of_property_read_u32(np, "gpu_pmu_status_local_pwr_mask",
		&pc->pm.status_local_power_mask)) {
		dev_err(kbdev->dev, "PMU status register power mask not set in DT\n");
		ret = -EINVAL;
		goto error;
	}

	if (of_property_read_string(np, "g3d_genpd_name", &g3d_power_domain_name)) {
		dev_err(kbdev->dev, "GPU power domain name not set in DT\n");
		ret = -EINVAL;
		goto error;
	}

	pc->pm.domain = gpu_pm_get_pm_cores_domain(g3d_power_domain_name);
	if (pc->pm.domain == NULL)
		return -ENODEV;

#if IS_ENABLED(CONFIG_GOOGLE_BCL)
	pc->pm.bcl_dev = google_retrieve_bcl_handle();
#endif

	return 0;

error:
	gpu_pm_term(kbdev);
	return ret;
}

/**
 * gpu_pm_term() - Terminates power control for a GPU
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * This function is called from the error-handling path of &gpu_pm_init, so must handle a
 * partially-initialized device.
 */
void gpu_pm_term(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;
	int i;

	for (i = 0; i < GPU_PM_DOMAIN_COUNT; i++) {
		if (pc->pm.domain_devs[i]) {
			if (pc->pm.domain_links[i])
				device_link_del(pc->pm.domain_links[i]);
			dev_pm_domain_detach(pc->pm.domain_devs[i], true);
		}
	}
}
