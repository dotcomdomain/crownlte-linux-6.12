/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef EXYNOS9810_BUS_GUARD_H
#define EXYNOS9810_BUS_GUARD_H

#include <linux/types.h>

struct devfreq;
struct exynos9810_bus_guard;

/* Zero-initialized lease. Its static guard owns the request and supplier class
 * reference independently of the consumer. Retire before freeing the lease.
 * Caller must acquire a managed supplier link before acquire, and serialize
 * a lease's operations with its own transition lock.
 */
struct exynos9810_bus_lease {
	struct exynos9810_bus_guard *guard;
	u64 generation;
	unsigned int requested_khz;
};

int exynos9810_bus_guard_acquire(unsigned int owner_cal_id,
		struct devfreq *supplier, struct exynos9810_bus_lease *lease);
int exynos9810_bus_guard_update(struct exynos9810_bus_lease *lease,
		unsigned int floor_khz);
int exynos9810_bus_guard_retire(struct exynos9810_bus_lease *lease);
/* Supplier remove, after linked consumers retire and before dropping its own
 * devfreq class reference. The static parent/request remain pinned; releasing
 * the old class permits a later provider registration on that same parent.
 */
int exynos9810_bus_guard_forget_supplier(struct devfreq *supplier);

#endif
