/* SPDX-License-Identifier: GPL-2.0 */
/*
 * crownlte audio stage A2 guard (drivers/soc/samsung/exynos9810-abox-a2.c).
 *
 * A device whose DT node has samsung,abox-a2-guard = <&abox_a2> (the ABOX
 * SysMMU, the ABOX itself) calls exynos9810_abox_a2_guard() first thing in
 * its probe, before it touches a register inside pd-aud. 0: the A2 gate
 * armed and verified the CL0 watchdog deadline, saw pd-aud on in the PMU
 * and took the AUD clocks through the clock framework; -EPROBE_DEFER: the
 * gate has not got that far yet; any other error: the gate refused, the
 * device must not come up in this boot. Without the A2 driver the stub
 * refuses, so a guarded device never comes up unguarded.
 */
#ifndef __SOC_SAMSUNG_EXYNOS9810_ABOX_A2_H
#define __SOC_SAMSUNG_EXYNOS9810_ABOX_A2_H

#include <linux/errno.h>

struct device;

#ifdef CONFIG_EXYNOS9810_ABOX_A2
int exynos9810_abox_a2_guard(struct device *consumer);
#else
static inline int exynos9810_abox_a2_guard(struct device *consumer)
{
	return -ENODEV;
}
#endif

#endif /* __SOC_SAMSUNG_EXYNOS9810_ABOX_A2_H */
