/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Exynos9810 MIF/INT devfreq (drivers/devfreq/exynos9810-acpm-devfreq.c):
 * interface for the Exynos9810 cpufreq driver.
 */
#ifndef __SOC_SAMSUNG_EXYNOS9810_DEVFREQ_H
#define __SOC_SAMSUNG_EXYNOS9810_DEVFREQ_H

#include <linux/errno.h>

struct devfreq;

/* one hard MIF floor per CPU cluster (LITTLE, big) */
#define EXYNOS9810_MIF_HARD_SLOTS	2

#if IS_ENABLED(CONFIG_ARM_EXYNOS9810_ACPM_DEVFREQ)
int exynos9810_mif_hard_floor(struct devfreq *df, unsigned int slot,
			      unsigned long khz, unsigned long *now_khz);
#else
static inline int exynos9810_mif_hard_floor(struct devfreq *df, unsigned int slot,
					    unsigned long khz, unsigned long *now_khz)
{
	return -ENODEV;
}
#endif

#endif
