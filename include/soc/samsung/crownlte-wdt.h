/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * crownlte (Exynos 9810) watchdog hooks, drivers/watchdog/s3c2410_wdt.c
 */
#ifndef __SOC_SAMSUNG_CROWNLTE_WDT_H
#define __SOC_SAMSUNG_CROWNLTE_WDT_H

#include <linux/errno.h>
#include <linux/kconfig.h>

#if IS_BUILTIN(CONFIG_S3C2410_WATCHDOG)
int crownlte_wdt_stop_petting(const char *why);
#else
static inline int crownlte_wdt_stop_petting(const char *why)
{
	return -ENODEV;
}
#endif

#endif /* __SOC_SAMSUNG_CROWNLTE_WDT_H */
