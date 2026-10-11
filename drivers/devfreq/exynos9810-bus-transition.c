// SPDX-License-Identifier: GPL-2.0-only
/* Shared real MIF -> INT -> FSYS0 transition path for future bus providers.
 * This does not register devfreq devices, invent suppliers or enable DT nodes.
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <soc/samsung/cal-if.h>
#include <soc/samsung/exynos9810-mif-bts.h>
#include "exynos9810-bus-transition.h"

static int level_for_rate(const struct exynos9810_bus_table *table,
		unsigned long hz)
{
	unsigned int i;

	for (i = 0; i < table->count; i++)
		if (table->levels[i].hz == hz)
			return i;
	return -ERANGE;
}

static int actual_level(struct exynos9810_bus_transition *state)
{
	unsigned long rate = cal_dfs_get_rate(state->table->cal_id);

	if (!rate || rate > INT_MAX)
		return -EIO;
	return level_for_rate(state->table, rate * 1000UL);
}

/* PM QoS notifier return codes do not propagate target failures in ACK.
 * Explicitly run the supplier and check its real current-rate callback.
 * A accepted request is recorded even if hardware validation then fails.
 */
static int dependency_floor(struct exynos9810_bus_transition *state,
		unsigned int floor)
{
	int ret;

	if (!state->dependency)
		return floor ? -EINVAL : 0;
	ret = exynos9810_bus_guard_update(&state->dependency_lease, floor);
	state->dependency_floor = state->dependency_lease.requested_khz;
	return ret;
}

static void transition_cleanup(void *data)
{
	struct exynos9810_bus_transition *state = data;
	int ret = 0;

	mutex_lock(&state->lock);
	state->ready = false;
	if (state->dependency_lease.guard)
		ret = exynos9810_bus_guard_retire(&state->dependency_lease);
	mutex_unlock(&state->lock);
	if (ret)
		dev_err(state->dev, "dependency guard retained; hardware raise failed: %d\n", ret);
}

int exynos9810_bus_transition_init(struct exynos9810_bus_transition *state,
		struct device *dev, const struct exynos9810_bus_table *table,
		struct devfreq *dependency, struct exynos9810_mif_bts *bts)
{
	unsigned int i, expected_dependency = 0, dependency_id, owner_id;
	int level, ret;

	if (!state || !dev || !table || !table->count ||
	    table->count > EXYNOS9810_BUS_LEVELS)
		return -EINVAL;
	if (of_property_read_u32(dev->of_node, "samsung,cal-id", &owner_id) ||
	    owner_id != table->cal_id)
		return -EINVAL;
	switch (table->cal_id) {
	case EXYNOS9810_MIF_CAL_ID:
		expected_dependency = EXYNOS9810_INT_CAL_ID;
		if (IS_ERR_OR_NULL(bts))
			return bts ? PTR_ERR(bts) : -ENODEV;
		break;
	case EXYNOS9810_INT_CAL_ID:
		expected_dependency = EXYNOS9810_FSYS0_CAL_ID;
		if (bts)
			return -EINVAL;
		break;
	case EXYNOS9810_FSYS0_CAL_ID:
		if (bts || dependency)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}
	for (i = 0; i < table->count; i++) {
		if (!table->levels[i].hz || table->levels[i].hz % 1000 ||
		    table->levels[i].hz / 1000 > INT_MAX || !table->levels[i].uv ||
		    (i && table->levels[i].hz >= table->levels[i - 1].hz) ||
		    (expected_dependency && (!table->levels[i].dependency_khz ||
				     table->levels[i].dependency_khz > INT_MAX)) ||
		    (!expected_dependency && table->levels[i].dependency_khz))
			return -EINVAL;
	}
	if (expected_dependency) {
		if (IS_ERR_OR_NULL(dependency))
			return dependency ? PTR_ERR(dependency) : -ENODEV;
		if (!dependency->profile || !dependency->dev.parent ||
		    !dependency->profile->get_cur_freq || dependency->dev.parent == dev ||
		    of_property_read_u32(dependency->dev.parent->of_node, "samsung,cal-id", &dependency_id) ||
		    dependency_id != expected_dependency)
			return -EINVAL;
	}
	if (!cal_dfs_is_ready(table->cal_id))
		return -EPROBE_DEFER;
	state->dev = dev;
	state->table = table;
	state->dependency = dependency;
	state->bts = bts;
	state->ready = false;
	state->faulted = false;
	state->dependency_floor = 0;
	mutex_init(&state->lock);
	level = actual_level(state);
	if (level < 0)
		return level;
	if (dependency) {
		ret = exynos9810_bus_guard_acquire(table->cal_id, dependency,
						&state->dependency_lease);
		if (ret)
			return ret;
		state->dependency_floor = state->dependency_lease.requested_khz;
	}
	ret = devm_add_action_or_reset(dev, transition_cleanup, state);
	if (ret)
		return ret;
	ret = dependency_floor(state, table->levels[level].dependency_khz);
	if (ret)
		return ret;
	if (bts) {
		ret = exynos9810_mif_bts_apply(bts, table->levels[level].hz / 1000);
		if (ret)
			return ret;
	}
	state->ready = true;
	return 0;
}

int exynos9810_bus_transition_current(struct exynos9810_bus_transition *state,
		unsigned long *actual_hz)
{
	int level, ret = 0;

	if (!state || !actual_hz)
		return -EINVAL;
	mutex_lock(&state->lock);
	if (!state->ready || state->faulted) {
		ret = -EIO;
		goto out;
	}
	level = actual_level(state);
	if (level < 0)
		ret = level;
	else
		*actual_hz = state->table->levels[level].hz;
out:
	mutex_unlock(&state->lock);
	return ret;
}

int exynos9810_bus_transition_rate(struct exynos9810_bus_transition *state,
		unsigned long requested_hz)
{
	const struct exynos9810_bus_table *table;
	unsigned int protected_floor, required;
	unsigned long old_hz;
	int old, target, actual, ret, repair;

	if (!state)
		return -EINVAL;
	mutex_lock(&state->lock);
	if (!state->ready || state->faulted) {
		ret = -EIO;
		goto out;
	}
	table = state->table;
	target = level_for_rate(table, requested_hz);
	if (target < 0) {
		ret = target;
		goto out;
	}
	old = actual_level(state);
	if (old < 0) {
		state->faulted = true;
		ret = old;
		goto out;
	}
	old_hz = table->levels[old].hz;
	required = table->levels[target].dependency_khz;
	protected_floor = max(state->dependency_floor,
			max(required, table->levels[old].dependency_khz));
	ret = dependency_floor(state, protected_floor);
	if (ret)
		goto out; /* No own rate write; retain the raised request. */
	if (state->bts && requested_hz <= old_hz) {
		ret = exynos9810_mif_bts_apply(state->bts, requested_hz / 1000);
		if (ret)
			goto fault;
	}
	if (requested_hz != old_hz) {
		ret = cal_dfs_set_rate(table->cal_id, requested_hz / 1000);
		if (ret) {
			/* A timeout does not prove the rate remained unchanged. Repair
			 * BTS only from a new confirmed table row, retaining high QoS.
			 */
			actual = actual_level(state);
			if (actual >= 0) {
				dependency_floor(state, max(protected_floor,
						table->levels[actual].dependency_khz));
				if (state->bts)
					exynos9810_mif_bts_apply(state->bts, table->levels[actual].hz / 1000);
			}
			goto fault;
		}
	}
	actual = actual_level(state);
	if (actual != target) {
		if (actual >= 0) {
			dependency_floor(state, max(protected_floor,
					table->levels[actual].dependency_khz));
			if (state->bts)
				exynos9810_mif_bts_apply(state->bts, table->levels[actual].hz / 1000);
		}
		ret = actual < 0 ? actual : -EIO;
		goto fault;
	}
	if (state->bts && requested_hz > old_hz) {
		ret = exynos9810_mif_bts_apply(state->bts, requested_hz / 1000);
		if (ret)
			goto fault;
	}
	/* Reconcile even an unchanged rate: firmware may have changed a supplier
	 * autonomously, and its cached previous frequency cannot prove readiness.
	 */
	ret = dependency_floor(state, required);
	if (ret) {
		repair = dependency_floor(state, protected_floor);
		if (repair)
			dev_err(state->dev, "dependency floor restore failed: %d\n", repair);
		goto fault;
	}
	goto out;
fault:
	state->faulted = true;
out:
	mutex_unlock(&state->lock);
	return ret;
}
