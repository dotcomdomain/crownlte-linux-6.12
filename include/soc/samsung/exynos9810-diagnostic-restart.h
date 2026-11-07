/* SPDX-License-Identifier: GPL-2.0 */
/*
 * crownlte diagnostic restart (drivers/soc/samsung/exynos9810-diagnostic-
 * restart.c): kernel-internal entry points for a bounded full-SoC reset
 * through the same PMU sequence as a default restart(2).
 *
 * exynos9810_diagnostic_restart_check(): read-only. NULL if the direct
 * reset can run now (context published, CAL up, same chip revision, DFD
 * dump off), else the reason it cannot.
 *
 * exynos9810_diagnostic_emergency_restart(): process context only; must
 * not be called with the CPU hotplug lock held. On the reboot CPU (chosen
 * and kept online under cpus_read_lock()), runs the restart preparation
 * (normal INFORM2/INFORM3, ACPM STOP per the reset policy), publishes its
 * own emergency preparation (which a concurrent restart(2) cannot clear)
 * and calls emergency_restart(), whose restart guard writes PMU SWRESET.
 * No device shutdown. Returns only if it refused before
 * emergency_restart(); the return value is then negative.
 *
 * Without the diagnostic restart driver the stubs refuse.
 */
#ifndef __SOC_SAMSUNG_EXYNOS9810_DIAGNOSTIC_RESTART_H
#define __SOC_SAMSUNG_EXYNOS9810_DIAGNOSTIC_RESTART_H

#include <linux/errno.h>

#ifdef CONFIG_EXYNOS9810_DIAGNOSTIC_RESTART
const char *exynos9810_diagnostic_restart_check(void);
int exynos9810_diagnostic_emergency_restart(const char *reason);
#else
static inline const char *exynos9810_diagnostic_restart_check(void)
{
	return "diagnostic restart not built";
}

static inline int exynos9810_diagnostic_emergency_restart(const char *reason)
{
	return -ENODEV;
}
#endif

#endif /* __SOC_SAMSUNG_EXYNOS9810_DIAGNOSTIC_RESTART_H */
