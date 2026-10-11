/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef EXYNOS9810_MIF_BTS_H
#define EXYNOS9810_MIF_BTS_H

#include <linux/device.h>

struct exynos9810_mif_bts;

/* Mandatory managed supplier, acquired only from the consumer's probe callback.
 * Errors, including probe deferral, are explicit.
 * Call apply before a MIF decrease and after a verified MIF increase.
 * This hardware service does not itself request a frequency transition.
 */
struct exynos9810_mif_bts *exynos9810_mif_bts_get(struct device *consumer);
int exynos9810_mif_bts_apply(struct exynos9810_mif_bts *bts, unsigned int khz);

#endif
