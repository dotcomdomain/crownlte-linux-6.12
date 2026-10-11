/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef EXYNOS9810_BUS_TRANSITION_H
#define EXYNOS9810_BUS_TRANSITION_H

#include <linux/devfreq.h>
#include <linux/mutex.h>
#include <linux/pm_qos.h>
#include "exynos9810-bus-tables.h"
#include "exynos9810-bus-guard.h"

struct exynos9810_mif_bts;

/* Zero-initialized and owned by a future registered provider. Init once.
 * It must acquire the real managed
 * dependency/BTS suppliers before init and keep the immutable table alive.
 * Add its devfreq cleanup after init so devfreq is removed before this state.
 */
struct exynos9810_bus_transition {
	struct device *dev;
	const struct exynos9810_bus_table *table;
	struct devfreq *dependency;
	struct exynos9810_bus_lease dependency_lease;
	unsigned int dependency_floor;
	struct exynos9810_mif_bts *bts;
	struct mutex lock;
	bool ready;
	bool faulted;
};

int exynos9810_bus_transition_init(struct exynos9810_bus_transition *state,
		struct device *dev, const struct exynos9810_bus_table *table,
		struct devfreq *dependency, struct exynos9810_mif_bts *bts);
int exynos9810_bus_transition_rate(struct exynos9810_bus_transition *state,
		unsigned long requested_hz);
int exynos9810_bus_transition_current(struct exynos9810_bus_transition *state,
		unsigned long *actual_hz);

#endif
