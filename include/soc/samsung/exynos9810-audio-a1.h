/* SPDX-License-Identifier: GPL-2.0 */
/*
 * crownlte (Exynos 9810) audio stage A1 guard. A device whose DT node has
 * samsung,audio-a1-guard = <&audio_a1> (the amp bus: USI07 wrapper and
 * HSI2C18) calls this first in its probe: 0 once the A1 driver installed
 * and verified the CL0 watchdog deadline, -EPROBE_DEFER before that, any
 * other error if A1 refused (the device must not come up in this boot).
 * See Documentation/crownlte/audio-a1.md.
 */
#ifndef __SOC_SAMSUNG_EXYNOS9810_AUDIO_A1_H
#define __SOC_SAMSUNG_EXYNOS9810_AUDIO_A1_H

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

struct device;

/*
 * The MAX98512 reads A1 makes, and the only ones the HSI2C18 adapter in A1
 * read-only mode passes: 7-bit addresses 0x39 (left) / 0x3a (right) and
 * these 16-bit register addresses (K49 sound/soc/codecs/max98512.h):
 * INT_RAW1-3 0x0001-0x0003 (raw status, not clear-on-read), GLOBAL_SHDN
 * 0x0400 / 0x0500 (rev A / B), REV_ID 0x0402 / 0x0600 (rev A / B). The
 * clear-on-read interrupt flags 0x0007-0x0009 and everything else are not
 * on the list.
 */
static inline bool exynos9810_audio_a1_amp_addr_ok(u16 addr)
{
	return addr == 0x39 || addr == 0x3a;
}

static inline bool exynos9810_audio_a1_amp_reg_ok(u16 reg)
{
	switch (reg) {
	case 0x0001:	/* INT_RAW1 */
	case 0x0002:	/* INT_RAW2 */
	case 0x0003:	/* INT_RAW3 */
	case 0x0400:	/* GLOBAL_SHDN (rev A) */
	case 0x0402:	/* REV_ID (rev A) */
	case 0x0500:	/* GLOBAL_SHDN (rev B) */
	case 0x0600:	/* REV_ID (rev B) */
		return true;
	default:
		return false;
	}
}

/*
 * Boot-lifetime latch for the guarded bus devices, keyed by the physical
 * base of the device's first MMIO window. It lives in the built-in A1
 * driver, so neither a rebind nor a reload of the bus driver resets it.
 *  CHECK: first thing in probe; -EPERM if this controller was already
 *         brought up (or closed) in this boot, before anything else.
 *  CLAIM: just before the first clock, SYSREG or register access; records
 *         the controller (one hardware bring-up per boot), -EPERM if it
 *         was already recorded or the table is full.
 *  CLOSE: the controller failed and is closed for the rest of the boot.
 */
enum exynos9810_a1_bus_op {
	EXYNOS9810_A1_BUS_CHECK,
	EXYNOS9810_A1_BUS_CLAIM,
	EXYNOS9810_A1_BUS_CLOSE,
};

#if IS_ENABLED(CONFIG_EXYNOS9810_AUDIO_A1)
int exynos9810_audio_a1_guard(struct device *consumer);
int exynos9810_audio_a1_bus(struct device *dev, phys_addr_t base,
			    enum exynos9810_a1_bus_op op);
#else
/* No A1 driver: a guarded device never comes up. */
static inline int exynos9810_audio_a1_guard(struct device *consumer)
{
	return -ENODEV;
}

static inline int exynos9810_audio_a1_bus(struct device *dev, phys_addr_t base,
					  enum exynos9810_a1_bus_op op)
{
	return op == EXYNOS9810_A1_BUS_CLOSE ? 0 : -ENODEV;
}
#endif

#endif /* __SOC_SAMSUNG_EXYNOS9810_AUDIO_A1_H */
