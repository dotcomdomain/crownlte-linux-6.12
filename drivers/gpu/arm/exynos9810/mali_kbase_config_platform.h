/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Exynos9810 (crownlte) kbase platform configuration.
 */
#ifndef _MALI_KBASE_CONFIG_PLATFORM_H_
#define _MALI_KBASE_CONFIG_PLATFORM_H_

#include "mali_exynos_kbase_entrypoint.h"

/* No runtime-PM callbacks: the PMU G3D domain is never switched (stage 1). */
#define POWER_MANAGEMENT_CALLBACKS (&exynos9810_gpu_pm_callbacks)
#define PLATFORM_FUNCS (&exynos9810_gpu_platform_funcs)

extern struct kbase_pm_callback_conf exynos9810_gpu_pm_callbacks;
extern struct kbase_platform_funcs_conf exynos9810_gpu_platform_funcs;

#endif /* _MALI_KBASE_CONFIG_PLATFORM_H_ */
