/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef EXYNOS9810_MIF_BTS_POLICY_H
#define EXYNOS9810_MIF_BTS_POLICY_H

#include <linux/errno.h>

#define EXYNOS9810_BTS_QMAX_CELLS 97

/* Original format: mo, threshold_khz, mo, threshold_khz, ... mo.
 * A boundary belongs to the lower-frequency entry, as in find_qmax().
 * Validate every threshold before returning a selected entry.
 */
static inline int exynos9810_bts_qmax(const unsigned int *cells,
		unsigned int count, unsigned int khz, unsigned int *result)
{
	unsigned int i, selected = 0;

	if (!cells || !result || !khz || !count ||
	    count > EXYNOS9810_BTS_QMAX_CELLS || !(count & 1))
		return -EINVAL;
	for (i = 1; i < count; i += 2) {
		if (!cells[i] || (i > 1 && cells[i] >= cells[i - 2]))
			return -EINVAL;
	}
	while (selected + 1 < count && khz <= cells[selected + 1])
		selected += 2;
	/* Preserve cal_bts9810.c set_mo(), including zero -> maximum. */
	*result = !cells[selected] || cells[selected] > 0xffff ?
		0xffff : cells[selected];
	return 0;
}

#endif
