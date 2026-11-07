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

struct device;

#if IS_ENABLED(CONFIG_EXYNOS9810_AUDIO_A1)
int exynos9810_audio_a1_guard(struct device *consumer);
#else
/* No A1 driver: a guarded device never comes up. */
static inline int exynos9810_audio_a1_guard(struct device *consumer)
{
	return -ENODEV;
}
#endif

#endif /* __SOC_SAMSUNG_EXYNOS9810_AUDIO_A1_H */
