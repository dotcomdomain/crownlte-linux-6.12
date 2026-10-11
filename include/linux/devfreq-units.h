/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_DEVFREQ_UNITS_H
#define _LINUX_DEVFREQ_UNITS_H

#include <linux/errno.h>
#include <linux/limits.h>
#include <linux/types.h>
#include <linux/units.h>

/* Legacy Exynos bus sysfs uses kHz; devfreq and OPP always use Hz. */
static inline unsigned long devfreq_sysfs_freq(bool legacy_khz,
					     unsigned long hz)
{
	return legacy_khz ? hz / HZ_PER_KHZ : hz;
}

static inline int devfreq_sysfs_to_hz(bool legacy_khz, unsigned long value,
				    unsigned long *hz)
{
	if (!hz)
		return -EINVAL;
	if (legacy_khz && value > ULONG_MAX / HZ_PER_KHZ)
		return -ERANGE;
	*hz = legacy_khz ? value * HZ_PER_KHZ : value;
	return 0;
}

#endif /* _LINUX_DEVFREQ_UNITS_H */
