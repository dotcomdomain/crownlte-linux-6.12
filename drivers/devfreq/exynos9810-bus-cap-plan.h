/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef EXYNOS9810_BUS_CAP_PLAN_H
#define EXYNOS9810_BUS_CAP_PLAN_H

#include "exynos9810-bus-tables.h"

#define EXYNOS9810_CAP_DOMAINS 3

/* Slots are MIF, INT, FSYS0. Tables must remain immutable and alive. The
 * caller pins all provider instances and holds one coordinator lock throughout
 * the transaction, outside device-QoS notifiers. Targets must cooperate with
 * that lock and cannot change the snapshot concurrently in software.
 * This API neither acquires those pins/locks nor operates hardware.
 */
struct exynos9810_cap_input {
	const struct exynos9810_bus_table *table;
	unsigned long confirmed_hz;
	unsigned long policy_max_hz;
	u64 generation;
	bool ready;
};

struct exynos9810_cap_step {
	unsigned int cal_id;
	unsigned long from_hz, target_hz;
	/* Zero means no incoming dependency ceiling. Never publish this
	 * field as a global legacy PM_QOS MAX request.
	 */
	unsigned long internal_max_hz;
	unsigned long effective_max_hz;
	u64 generation;
};

enum exynos9810_cap_phase {
	EXYNOS9810_CAP_READY, EXYNOS9810_CAP_WAITING,
	EXYNOS9810_CAP_DONE, EXYNOS9810_CAP_FAULTED,
};

struct exynos9810_cap_transaction {
	struct exynos9810_cap_step steps[EXYNOS9810_CAP_DOMAINS];
	unsigned long expected_hz[EXYNOS9810_CAP_DOMAINS];
	u64 generations[EXYNOS9810_CAP_DOMAINS];
	u64 cookie;
	unsigned int cursor;
	enum exynos9810_cap_phase phase;
};

/* A reduction-only transaction. It never raises a domain, rolls a request
 * back, releases a dependency guard or claims an IPC response proves a rate.
 * The caller supplies a unique nonzero cookie for each serialized transaction.
 * Invalid input leaves the output transaction unchanged.
 */
int exynos9810_cap_prepare(const struct exynos9810_cap_input input[3],
		u64 cookie, struct exynos9810_cap_transaction *transaction);

/* Fresh actual rates for all three pinned providers are required before each
 * step. A changed instance/rate faults the transaction. The caller must install
 * the step's internal maximum in provider-private state, run the real target
 * path, and reread the actual CAL rate, even when from_hz == target_hz.
 */
int exynos9810_cap_next(struct exynos9810_cap_transaction *transaction,
		const unsigned long actual_hz[3], const u64 generations[3],
		struct exynos9810_cap_step *step);

/* Only exact hardware confirmation advances to the next supplier. Failed or
 * uncertain operations fault without rollback; protective floors must remain.
 * Wrong/stale confirmation tokens leave the waiting transaction unchanged.
 */
int exynos9810_cap_confirm(struct exynos9810_cap_transaction *transaction,
		u64 cookie, unsigned int cal_id, u64 generation,
		unsigned long actual_hz, int operation_status);

#endif
