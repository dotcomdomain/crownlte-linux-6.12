// SPDX-License-Identifier: GPL-2.0-only
/* Bounded, durable MIF->INT and INT->FSYS0 dependency request ownership.
 * Built-in guard requests live until reboot. An unleased guard stays at the
 * supplier's supported maximum, including failed consumer probe/removal.
 * Physical/firmware thermal coordination remains necessary before enabling
 * hardware; a QoS floor never bypasses a maximum cap or validates firmware.
 */
#include <linux/device.h>
#include <linux/devfreq.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm_qos.h>
#include "exynos9810-bus-guard.h"
#include "exynos9810-bus-tables.h"

struct exynos9810_bus_guard {
	struct mutex *lock;
	struct devfreq *supplier;
	struct device *parent;
	struct dev_pm_qos_request request;
	unsigned int expected_id;
	unsigned int maximum_khz;
	unsigned int requested_khz;
	u64 generation;
	bool leased;
};

/* Separate locks follow the hardware DAG; a supplier target may call its own
 * downstream guard while this guard is locked. A shared lock would deadlock.
 */
static DEFINE_MUTEX(mif_guard_lock);
static DEFINE_MUTEX(int_guard_lock);
static struct exynos9810_bus_guard mif_guard = {
	.lock = &mif_guard_lock, .expected_id = EXYNOS9810_INT_CAL_ID,
};
static struct exynos9810_bus_guard int_guard = {
	.lock = &int_guard_lock, .expected_id = EXYNOS9810_FSYS0_CAL_ID,
};

const struct dev_pm_qos_request *
exynos9810_bus_guard_min_request(struct device *supplier)
{
	u32 id;

	if (IS_ERR_OR_NULL(supplier) ||
	    of_property_read_u32(supplier->of_node, "samsung,cal-id", &id))
		return NULL;
	switch (id) {
	case EXYNOS9810_INT_CAL_ID:
		return &mif_guard.request;
	case EXYNOS9810_FSYS0_CAL_ID:
		return &int_guard.request;
	default:
		return NULL;
	}
}

static struct exynos9810_bus_guard *guard_for_owner(unsigned int owner)
{
	switch (owner) {
	case EXYNOS9810_MIF_CAL_ID:
		return &mif_guard;
	case EXYNOS9810_INT_CAL_ID:
		return &int_guard;
	default:
		return NULL;
	}
}

/* Caller holds guard->lock. ACK's QoS notifier can hide target errors. */
static int set_floor(struct exynos9810_bus_guard *guard, unsigned int khz)
{
	struct devfreq *df = guard->supplier;
	unsigned long actual_hz = 0;
	int ret;

	ret = dev_pm_qos_update_request(&guard->request, khz);
	if (ret < 0)
		return ret;
	guard->requested_khz = khz;
	mutex_lock(&df->lock);
	ret = update_devfreq(df);
	if (!ret)
		ret = df->profile->get_cur_freq(df->dev.parent, &actual_hz);
	mutex_unlock(&df->lock);
	if (!ret && (actual_hz < khz * 1000UL ||
		     actual_hz > guard->maximum_khz * 1000UL))
		ret = -ERANGE;
	return ret;
}

static int supplier_maximum(struct devfreq *df, unsigned int *maximum)
{
	unsigned long hz, max_hz = 0;
	unsigned int i;
	int ret = 0;

	mutex_lock(&df->lock);
	if (!df->max_state || df->max_state > EXYNOS9810_BUS_LEVELS || !df->freq_table) {
		ret = -EINVAL;
		goto out;
	}
	for (i = 0; i < df->max_state; i++) {
		hz = df->freq_table[i];
		if (!hz || hz % 1000 || hz / 1000 > INT_MAX) {
			ret = -ERANGE;
			goto out;
		}
		max_hz = max(max_hz, hz);
	}
	*maximum = max_hz / 1000;
out:
	mutex_unlock(&df->lock);
	return ret;
}

int exynos9810_bus_guard_acquire(unsigned int owner, struct devfreq *df,
		struct exynos9810_bus_lease *lease)
{
	struct exynos9810_bus_guard *guard = guard_for_owner(owner);
	struct devfreq *old = NULL;
	unsigned int id, maximum;
	int ret;

	if (!guard || !lease || lease->guard || lease->generation || lease->requested_khz ||
	    IS_ERR_OR_NULL(df) || !df->profile || !df->dev.parent ||
	    !df->profile->get_cur_freq || !df->profile->legacy_khz_sysfs ||
	    !df->profile->release ||
	    of_property_read_u32(df->dev.parent->of_node, "samsung,cal-id", &id) ||
	    id != guard->expected_id)
		return -EINVAL;
	mutex_lock(guard->lock);
	if (guard->leased) {
		ret = -EBUSY;
		goto out;
	}
	if (guard->generation == U64_MAX) {
		ret = -EOVERFLOW;
		goto out;
	}
	if (!device_is_registered(&df->dev)) {
		ret = -ENODEV;
		goto out;
	}
	if (guard->parent && guard->parent != df->dev.parent) {
		ret = -EXDEV;
		goto out;
	}
	ret = supplier_maximum(df, &maximum);
	if (ret)
		goto out;
	if (!guard->parent) {
		if (!get_device(df->dev.parent)) {
			ret = -ENODEV;
			goto out;
		}
		guard->parent = df->dev.parent;
	}
	if (df != guard->supplier) {
		if (!get_device(&df->dev)) {
			ret = -ENODEV;
			goto out;
		}
		old = guard->supplier;
		guard->supplier = df;
		guard->parent = df->dev.parent;
	}
	guard->maximum_khz = maximum;
	if (!dev_pm_qos_request_active(&guard->request)) {
		ret = dev_pm_qos_add_request(guard->parent, &guard->request,
					     DEV_PM_QOS_MIN_FREQUENCY, maximum);
		if (ret < 0)
			goto out;
		guard->requested_khz = maximum;
	}
	/* Never lower a retained protective request during consumer reacquisition. */
	ret = set_floor(guard, max(guard->requested_khz, maximum));
	if (ret)
		goto out;
	guard->generation++;
	guard->leased = true;
	lease->guard = guard;
	lease->generation = guard->generation;
	lease->requested_khz = guard->requested_khz;
out:
	mutex_unlock(guard->lock);
	if (old)
		put_device(&old->dev);
	return ret;
}

int exynos9810_bus_guard_forget_supplier(struct devfreq *df)
{
	struct exynos9810_bus_guard *guard;
	u32 id;
	int ret = 0;
	bool put = false;

	if (!df || !df->dev.parent ||
	    of_property_read_u32(df->dev.parent->of_node, "samsung,cal-id", &id))
		return -EINVAL;
	if (id == EXYNOS9810_MIF_CAL_ID)
		return 0;
	if (id == EXYNOS9810_INT_CAL_ID)
		guard = &mif_guard;
	else if (id == EXYNOS9810_FSYS0_CAL_ID)
		guard = &int_guard;
	else
		return -EINVAL;
	mutex_lock(guard->lock);
	if (!guard->supplier)
		goto out;
	if (guard->supplier != df) {
		ret = -ESTALE;
		goto out;
	}
	if (guard->leased) {
		ret = -EBUSY;
		goto out;
	}
	if (dev_pm_qos_request_active(&guard->request) &&
	    guard->requested_khz < guard->maximum_khz) {
		ret = -EPERM;
		goto out;
	}
	guard->supplier = NULL;
	put = true;
out:
	mutex_unlock(guard->lock);
	if (put)
		put_device(&df->dev);
	return ret;
}

int exynos9810_bus_guard_update(struct exynos9810_bus_lease *lease,
		unsigned int khz)
{
	struct exynos9810_bus_guard *guard;
	int ret;

	if (!lease || !lease->guard || !khz || khz > INT_MAX)
		return -EINVAL;
	guard = lease->guard;
	mutex_lock(guard->lock);
	if (!guard->leased || lease->generation != guard->generation)
		ret = -ESTALE;
	else if (khz > guard->maximum_khz)
		ret = -ERANGE;
	else {
		ret = set_floor(guard, khz);
		lease->requested_khz = guard->requested_khz;
	}
	mutex_unlock(guard->lock);
	return ret;
}

int exynos9810_bus_guard_retire(struct exynos9810_bus_lease *lease)
{
	struct exynos9810_bus_guard *guard;
	int ret;

	if (!lease || !lease->guard)
		return -EINVAL;
	guard = lease->guard;
	mutex_lock(guard->lock);
	if (!guard->leased || lease->generation != guard->generation) {
		ret = -ESTALE;
	} else {
		/* The static request is retained even if the hardware update fails. */
		ret = set_floor(guard, max(guard->requested_khz, guard->maximum_khz));
		guard->leased = false;
		lease->guard = NULL;
		lease->generation = 0;
		lease->requested_khz = 0;
	}
	mutex_unlock(guard->lock);
	return ret;
}
