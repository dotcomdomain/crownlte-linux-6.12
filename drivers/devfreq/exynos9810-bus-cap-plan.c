// SPDX-License-Identifier: GPL-2.0-only
/* Pure planning/confirmation foundation. No provider or hardware callers. */
#include "exynos9810-bus-cap-plan.h"

static int cap_table_validate(const struct exynos9810_bus_table *table,
		unsigned int cal_id, unsigned long confirmed_hz)
{
	unsigned int i;
	bool found = false;

	if (!table || table->cal_id != cal_id || !table->count ||
	    table->count > EXYNOS9810_BUS_LEVELS)
		return -EINVAL;
	for (i = 0; i < table->count; i++) {
		const struct exynos9810_bus_level *row = &table->levels[i];

		if (!row->hz || row->hz % 1000 || row->hz / 1000 > 0x7fffffffUL ||
		    !row->uv || (i && row->hz >= table->levels[i - 1].hz) ||
		    (cal_id == EXYNOS9810_FSYS0_CAL_ID && row->dependency_khz) ||
		    (cal_id != EXYNOS9810_FSYS0_CAL_ID &&
		     (!row->dependency_khz || row->dependency_khz > 0x7fffffffU ||
		      (i && row->dependency_khz > table->levels[i - 1].dependency_khz))))
			return -EINVAL;
		found |= row->hz == confirmed_hz;
	}
	return found ? 0 : -ERANGE;
}

static unsigned long cap_supported_max(const struct exynos9810_bus_table *table,
		unsigned long maximum_hz)
{
	unsigned int i;

	for (i = 0; i < table->count; i++)
		if (table->levels[i].hz <= maximum_hz)
			return table->levels[i].hz;
	return 0;
}

int exynos9810_cap_prepare(const struct exynos9810_cap_input input[3],
		u64 cookie, struct exynos9810_cap_transaction *transaction)
{
	static const unsigned int ids[3] = {
		EXYNOS9810_MIF_CAL_ID, EXYNOS9810_INT_CAL_ID, EXYNOS9810_FSYS0_CAL_ID,
	};
	struct exynos9810_cap_transaction planned = { .cookie = cookie };
	unsigned long ceiling;
	unsigned int i, row;
	int ret;

	if (!input || !transaction || !cookie)
		return -EINVAL;
	for (i = 0; i < 3; i++) {
		if (!input[i].ready || !input[i].generation)
			return -ENODEV;
		if (!input[i].policy_max_hz || input[i].policy_max_hz % 1000 ||
		    input[i].policy_max_hz / 1000 > 0x7fffffffUL)
			return -EINVAL;
		ret = cap_table_validate(input[i].table, ids[i], input[i].confirmed_hz);
		if (ret)
			return ret;
		planned.generations[i] = input[i].generation;
		planned.expected_hz[i] = input[i].confirmed_hz;
		planned.steps[i].cal_id = ids[i];
		planned.steps[i].generation = input[i].generation;
		planned.steps[i].from_hz = input[i].confirmed_hz;
	}
	/* Reject an already unsafe initial clock snapshot. */
	for (i = 0; i < 2; i++)
		for (row = 0; row < input[i].table->count; row++)
			if (input[i].table->levels[row].hz == input[i].confirmed_hz &&
			    input[i].table->levels[row].dependency_khz > input[i + 1].confirmed_hz / 1000)
				return -ERANGE;
	/* Propagate the supplier ceilings upward, FSYS0 -> INT -> MIF. */
	for (i = 3; i-- > 0;) {
		ceiling = input[i].policy_max_hz;
		if (i < 2) {
			ret = exynos9810_bus_reverse_cap(input[i].table, ids[i + 1],
					planned.steps[i + 1].effective_max_hz,
					&planned.steps[i].internal_max_hz);
			if (ret)
				return ret;
			if (ceiling > planned.steps[i].internal_max_hz)
				ceiling = planned.steps[i].internal_max_hz;
		}
		planned.steps[i].effective_max_hz = cap_supported_max(input[i].table, ceiling);
		if (!planned.steps[i].effective_max_hz)
			return -ERANGE;
		planned.steps[i].target_hz = input[i].confirmed_hz;
		if (planned.steps[i].target_hz > planned.steps[i].effective_max_hz)
			planned.steps[i].target_hz = planned.steps[i].effective_max_hz;
	}
	planned.phase = EXYNOS9810_CAP_READY;
	*transaction = planned;
	return 0;
}

int exynos9810_cap_next(struct exynos9810_cap_transaction *transaction,
		const unsigned long actual_hz[3], const u64 generations[3],
		struct exynos9810_cap_step *step)
{
	unsigned int i;

	if (!transaction || !actual_hz || !generations || !step || !transaction->cookie)
		return -EINVAL;
	if (transaction->phase == EXYNOS9810_CAP_FAULTED)
		return -EIO;
	if (transaction->phase == EXYNOS9810_CAP_DONE)
		return -ENOENT;
	if (transaction->phase == EXYNOS9810_CAP_WAITING)
		return -EBUSY;
	if (transaction->phase != EXYNOS9810_CAP_READY || transaction->cursor >= 3)
		return -EINVAL;
	for (i = 0; i < 3; i++) {
		if (generations[i] != transaction->generations[i] ||
		    actual_hz[i] != transaction->expected_hz[i]) {
			transaction->phase = EXYNOS9810_CAP_FAULTED;
			return generations[i] != transaction->generations[i] ? -ESTALE : -EIO;
		}
	}
	*step = transaction->steps[transaction->cursor];
	transaction->phase = EXYNOS9810_CAP_WAITING;
	return 0;
}

int exynos9810_cap_confirm(struct exynos9810_cap_transaction *transaction,
		u64 cookie, unsigned int cal_id, u64 generation,
		unsigned long actual_hz, int operation_status)
{
	const struct exynos9810_cap_step *step;

	if (!transaction || !transaction->cookie)
		return -EINVAL;
	if (transaction->phase == EXYNOS9810_CAP_FAULTED)
		return -EIO;
	if (transaction->phase != EXYNOS9810_CAP_WAITING || transaction->cursor >= 3)
		return -EINVAL;
	step = &transaction->steps[transaction->cursor];
	if (cookie != transaction->cookie || cal_id != step->cal_id || generation != step->generation)
		return -ESTALE;
	if (operation_status || actual_hz != step->target_hz) {
		transaction->phase = EXYNOS9810_CAP_FAULTED;
		return operation_status < 0 ? operation_status : -EIO;
	}
	transaction->expected_hz[transaction->cursor++] = actual_hz;
	transaction->phase = transaction->cursor == 3 ? EXYNOS9810_CAP_DONE : EXYNOS9810_CAP_READY;
	return 0;
}
