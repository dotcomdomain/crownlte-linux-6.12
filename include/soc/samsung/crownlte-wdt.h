/* SPDX-License-Identifier: GPL-2.0 */
/*
 * crownlte (Exynos 9810) CL0 watchdog hooks for the suspend driver.
 * Served by drivers/watchdog/s3c2410_wdt.c for its SBOOT-guarded instance
 * only (samsung,exynos9810-wdt); see Documentation/crownlte/suspend-s0.md.
 */
#ifndef __SOC_SAMSUNG_CROWNLTE_WDT_H
#define __SOC_SAMSUNG_CROWNLTE_WDT_H

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

struct crownlte_wdt_state {
	u32 wtcon, wtdat, wtcnt;
	unsigned int timeout;		/* seconds, current setting */
	bool enabled, reset;		/* WTCON enable / reset enable */
	bool keep_armed_suspend;	/* DT samsung,keep-armed-in-suspend */
	bool pet_stopped;		/* no longer petted (restart/selftest/stop/deadline) */
	u64 deadline_ns;		/* CLOCK_BOOTTIME reload deadline, 0: none */
};

/* s3c2410wdt_crownlte_resume_early(): what it found, and what it left */
struct crownlte_wdt_early {
	u32 found_wtcon, found_wtdat, found_wtcnt;
	u32 found_auto_disable, found_mask_reset;	/* PMU 0x408, 0x40c */
	u32 wtcon, wtdat, wtcnt;
	u32 auto_disable, mask_reset;
	unsigned int timeout;		/* seconds, kept-armed setting */
	bool reloaded;			/* counter and control written */
};

#if IS_REACHABLE(CONFIG_S3C2410_WATCHDOG)
int s3c2410wdt_crownlte_ping(void);
int s3c2410wdt_crownlte_stop_petting(const char *why);
int s3c2410wdt_crownlte_set_deadline(u64 boot_ns);
bool s3c2410wdt_crownlte_state(struct crownlte_wdt_state *st);
int s3c2410wdt_crownlte_resume_early(struct crownlte_wdt_early *e);
#else
static inline int s3c2410wdt_crownlte_ping(void)
{
	return -ENODEV;
}

static inline int s3c2410wdt_crownlte_stop_petting(const char *why)
{
	return -ENODEV;
}

static inline int s3c2410wdt_crownlte_set_deadline(u64 boot_ns)
{
	return -ENODEV;
}

static inline bool s3c2410wdt_crownlte_state(struct crownlte_wdt_state *st)
{
	return false;
}

static inline int s3c2410wdt_crownlte_resume_early(struct crownlte_wdt_early *e)
{
	return -ENODEV;
}
#endif

#endif /* __SOC_SAMSUNG_CROWNLTE_WDT_H */
