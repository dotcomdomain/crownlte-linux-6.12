/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Exynos9810 diagnostic MAX77705 USB-C path switch: write accounting, so the
 * read-only battery monitor can report every MAX77705 write since boot.
 */
#ifndef __LINUX_SOC_SAMSUNG_EXYNOS9810_MAX77705_MUIC_H
#define __LINUX_SOC_SAMSUNG_EXYNOS9810_MAX77705_MUIC_H

#if IS_ENABLED(CONFIG_EXYNOS9810_DIAGNOSTIC_MUIC)
/* Number of I2C write transactions the MUIC driver issued since boot. */
int exynos9810_max77705_muic_write_count(void);
#else
static inline int exynos9810_max77705_muic_write_count(void)
{
	return -1;	/* driver not built: no writes possible */
}
#endif

#endif
