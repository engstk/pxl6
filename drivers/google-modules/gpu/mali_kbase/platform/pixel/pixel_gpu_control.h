/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2020-2021 Google LLC.
 *
 * Author: Sidath Senanayake <sidaths@google.com>
 */

#ifndef _PIXEL_GPU_CONTROL_H_
#define _PIXEL_GPU_CONTROL_H_

/* Power management */
bool gpu_pm_get_power_state(struct kbase_device *kbdev);
int gpu_pm_init(struct kbase_device *kbdev);
void gpu_pm_term(struct kbase_device *kbdev);

/* DVFS */
void gpu_dvfs_event_power_on(struct kbase_device *kbdev);
void gpu_dvfs_event_power_off(struct kbase_device *kbdev);
int gpu_dvfs_init(struct kbase_device *kbdev);
void gpu_dvfs_term(struct kbase_device *kbdev);

/* sysfs */
int gpu_sysfs_init(struct kbase_device *kbdev);
void gpu_sysfs_term(struct kbase_device *kbdev);

/* Kernel context callbacks */
int gpu_dvfs_kctx_init(struct kbase_context *kctx);
void gpu_dvfs_kctx_term(struct kbase_context *kctx);

#endif /* _PIXEL_GPU_CONTROL_H_ */
