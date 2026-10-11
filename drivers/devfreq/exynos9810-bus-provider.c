// SPDX-License-Identifier: GPL-2.0-only
/* Crownlte MIF/INT/FSYS0 providers. Board nodes remain disabled until the
 * complete thermal, firmware ownership and BTS bring-up is validated.
 */
#include <linux/device.h>
#include <linux/devfreq.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/pm_qos.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <soc/samsung/exynos9810-mif-bts.h>
#include <soc/samsung/exynos9810-legacy-qos.h>
#include "exynos9810-bus-transition.h"
#include "exynos9810-interactive.h"
#include "governor.h"

enum bus_pm_phase {
	BUS_ACTIVE, BUS_ENTERING_SLEEP, BUS_SLEEP, BUS_LEAVING_SLEEP, BUS_PM_FAILED,
};

struct crownlte_bus {
	struct kref refs;
	struct device *dev;
	struct devfreq *df;
	struct devfreq_dev_profile profile;
	struct exynos9810_bus_table table;
	struct exynos9810_bus_transition transition;
	struct exynos9810_interactive_config interactive;
	struct dev_pm_qos_request default_min, system_min, boot_min, reboot_max;
	struct notifier_block reboot_notifier;
	struct exynos9810_legacy_qos_bridge legacy_qos;
	struct mutex pm_lock;
	enum bus_pm_phase pm_phase;
	unsigned long forced_hz;
	struct delayed_work boot_work;
	unsigned long frequencies[EXYNOS9810_BUS_LEVELS];
	unsigned long boot_expires;
	unsigned int freq_info[6], boot_info[2], added_opps, slot;
	bool retired, listed, sysfs_ready, boot_paused, pm_frozen, reboot_registered;
};

static struct platform_driver crownlte_bus_driver;
static DEFINE_MUTEX(bus_registry_lock);
static struct crownlte_bus *bus_registry[3];
static int bus_update(struct crownlte_bus *bus);

static int bus_slot(unsigned int id)
{
	switch (id) {
	case EXYNOS9810_MIF_CAL_ID: return 0;
	case EXYNOS9810_INT_CAL_ID: return 1;
	case EXYNOS9810_FSYS0_CAL_ID: return 2;
	default: return -EINVAL;
	}
}

static void bus_free(struct kref *ref)
{
	struct crownlte_bus *bus = container_of(ref, struct crownlte_bus, refs);

	/* A final class reference may outlive driver unbind. Keep its profile,
	 * frequency table, parent and dynamic OPP rows valid until this point.
	 */
	while (bus->added_opps)
		dev_pm_opp_remove(bus->dev, bus->frequencies[--bus->added_opps]);
	put_device(bus->dev);
	kfree(bus);
}

static void bus_owner_put(void *data)
{
	struct crownlte_bus *bus = data;

	kref_put(&bus->refs, bus_free);
}

static void bus_class_acquire(struct devfreq *df)
{
	struct crownlte_bus *bus = container_of(df->profile,
						struct crownlte_bus, profile);

	kref_get(&bus->refs);
}

static void bus_class_release(struct devfreq *df)
{
	struct crownlte_bus *bus = container_of(df->profile,
						struct crownlte_bus, profile);

	kref_put(&bus->refs, bus_free);
}

static int bus_current(struct device *dev, unsigned long *hz)
{
	struct crownlte_bus *bus = dev_get_drvdata(dev);

	if (!bus || READ_ONCE(bus->retired))
		return -ENODEV;
	if (READ_ONCE(bus->pm_phase) == BUS_PM_FAILED)
		return -EIO;
	return exynos9810_bus_transition_current(&bus->transition, hz);
}

static int bus_target(struct device *dev, unsigned long *hz, u32 flags)
{
	struct crownlte_bus *bus = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;
	int ret;

	if (!bus || READ_ONCE(bus->retired))
		return -ENODEV;
	if (bus->pm_phase == BUS_PM_FAILED)
		return -EIO;
	if (bus->pm_phase == BUS_SLEEP)
		return exynos9810_bus_transition_current(&bus->transition, hz);
	if (bus->forced_hz) {
		unsigned long minimum, cap;

		/* Suspend ignores ordinary minimum requests as in the old str_freq
		 * path, but cannot override an active maximum/thermal limit.
		 */
		devfreq_get_freq_range(bus->df, &minimum, &cap);
		if (bus->forced_hz > cap)
			return -ERANGE;
		*hz = bus->forced_hz;
		ret = exynos9810_bus_transition_rate(&bus->transition, *hz);
		if (ret)
			return ret;
		return exynos9810_bus_transition_current(&bus->transition, hz);
	}
	opp = devfreq_recommended_opp(dev, hz, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	dev_pm_opp_put(opp);
	ret = exynos9810_bus_transition_rate(&bus->transition, *hz);
	if (ret)
		return ret;
	return exynos9810_bus_transition_current(&bus->transition, hz);
}

static void supplier_put(void *data)
{
	put_device(data);
}

static struct devfreq *bus_dependency(struct crownlte_bus *bus)
{
	struct device_node *node;
	struct platform_device *pdev;
	struct crownlte_bus *supplier;
	struct devfreq *df;
	u32 expected, id;
	int ret;

	if (bus->table.cal_id == EXYNOS9810_FSYS0_CAL_ID) {
		if (of_find_property(bus->dev->of_node, "samsung,bus-dependency", NULL))
			return ERR_PTR(-EINVAL);
		return NULL;
	}
	expected = bus->table.cal_id == EXYNOS9810_MIF_CAL_ID ?
		EXYNOS9810_INT_CAL_ID : EXYNOS9810_FSYS0_CAL_ID;
	node = of_parse_phandle(bus->dev->of_node, "samsung,bus-dependency", 0);
	if (!node)
		return ERR_PTR(-ENODEV);
	ret = of_property_read_u32(node, "samsung,cal-id", &id);
	if (ret || id != expected ||
	    !of_device_is_compatible(node, "samsung,exynos9810-bus-devfreq")) {
		of_node_put(node);
		return ERR_PTR(-EINVAL);
	}
	pdev = of_find_device_by_node(node);
	of_node_put(node);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);
	device_lock(&pdev->dev);
	supplier = platform_get_drvdata(pdev);
	if (!device_is_bound(&pdev->dev) ||
	    pdev->dev.driver != &crownlte_bus_driver.driver || !supplier ||
	    READ_ONCE(supplier->retired) || !supplier->df) {
		ret = -EPROBE_DEFER;
		goto unlock_put;
	}
	/* Bound private state, not devfreq_get_devfreq_by_node's unreferenced
	 * list result. The managed link protects supplier devres after unlock.
	 */
	if (!device_link_add(bus->dev, &pdev->dev, DL_FLAG_AUTOREMOVE_CONSUMER)) {
		ret = -ENOMEM;
		goto unlock_put;
	}
	df = supplier->df;
	get_device(&df->dev);
	device_unlock(&pdev->dev);
	put_device(&pdev->dev);
	ret = devm_add_action_or_reset(bus->dev, supplier_put, &df->dev);
	return ret ? ERR_PTR(ret) : df;
unlock_put:
	device_unlock(&pdev->dev);
	put_device(&pdev->dev);
	return ERR_PTR(ret);
}

static bool buses_ready(void)
{
	return bus_registry[0] && bus_registry[1] && bus_registry[2];
}

static void bus_boot_work(struct work_struct *work)
{
	struct crownlte_bus *bus = container_of(to_delayed_work(work),
						struct crownlte_bus, boot_work);
	bool ready;
	unsigned long delay, now = jiffies;
	int ret;

	mutex_lock(&bus_registry_lock);
	ready = buses_ready();
	mutex_unlock(&bus_registry_lock);
	if (READ_ONCE(bus->retired) || READ_ONCE(bus->boot_paused))
		return;
	/* An absent MIF provider may still leave firmware running at a high
	 * boot rate. Never release a supplier's bootstrap floor in that state.
	 */
	if (!ready || time_before(now, bus->boot_expires)) {
		delay = ready ? bus->boot_expires - now : HZ;
		mod_delayed_work(system_wq, &bus->boot_work, max(1UL, delay));
		return;
	}
	ret = dev_pm_qos_update_request(&bus->boot_min, 0);
	if (ret < 0)
		dev_err(bus->dev, "cannot release bus bootstrap request: %d\n", ret);
}

static void bus_remove_requests(struct crownlte_bus *bus)
{
	if (dev_pm_qos_request_active(&bus->reboot_max))
		dev_pm_qos_remove_request(&bus->reboot_max);
	if (dev_pm_qos_request_active(&bus->boot_min))
		dev_pm_qos_remove_request(&bus->boot_min);
	if (dev_pm_qos_request_active(&bus->system_min))
		dev_pm_qos_remove_request(&bus->system_min);
	if (dev_pm_qos_request_active(&bus->default_min))
		dev_pm_qos_remove_request(&bus->default_min);
}

static struct devfreq *class_devfreq(struct device *dev)
{
	return container_of(dev, struct devfreq, dev);
}

static struct crownlte_bus *class_bus(struct device *dev)
{
	return container_of(class_devfreq(dev)->profile, struct crownlte_bus, profile);
}

static ssize_t scaling_devfreq_min_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct crownlte_bus *bus = class_bus(dev);

	/* This old attribute reports its own system request, not the aggregate
	 * minimum and not the separate board default request.
	 */
	return sysfs_emit(buf, "%d\n", READ_ONCE(bus->system_min.data.freq.pnode.prio));
}

static ssize_t scaling_devfreq_min_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct crownlte_bus *bus = class_bus(dev);
	u32 value;
	int ret;
	bool changed;

	ret = kstrtou32(buf, 0, &value);
	if (ret || value > INT_MAX)
		return ret ? ret : -ERANGE;
	changed = (s32)value != READ_ONCE(bus->system_min.data.freq.pnode.prio);
	ret = dev_pm_qos_update_request(&bus->system_min, value);
	/* The old INT minimum class uses FORCE_MAX even when this system request
	 * remains below another minimum. Native device QoS suppresses that event.
	 * Preserve the old exact-repeat no-op and write-count return behavior.
	 */
	if (ret >= 0 && changed && bus->slot == 1) {
		int apply_ret = bus_update(bus);

		if (apply_ret)
			dev_err(bus->dev, "INT system request retained; apply failed: %d\n", apply_ret);
	}
	return ret < 0 ? ret : count;
}
static DEVICE_ATTR(scaling_devfreq_min, 0640,
		   scaling_devfreq_min_show, scaling_devfreq_min_store);

static ssize_t use_delay_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct crownlte_bus *bus = class_bus(dev);
	struct devfreq *df = class_devfreq(dev);
	bool enabled;

	mutex_lock(&df->lock);
	enabled = bus->interactive.use_delay_time;
	mutex_unlock(&df->lock);
	return sysfs_emit(buf, "%s\n", enabled ? "true" : "false");
}

static ssize_t use_delay_time_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct crownlte_bus *bus = class_bus(dev);
	struct devfreq *df = class_devfreq(dev);
	struct exynos9810_interactive_config next;
	bool enabled;
	int ret;

	ret = kstrtobool(buf, &enabled);
	if (ret)
		return ret;
	mutex_lock(&df->lock);
	next = bus->interactive;
	next.use_delay_time = enabled;
	ret = exynos9810_interactive_validate(&next);
	if (!ret) {
		bus->interactive = next;
		ret = update_devfreq(df);
	}
	mutex_unlock(&df->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR(use_delay_time, 0640, use_delay_time_show, use_delay_time_store);

static ssize_t delay_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct crownlte_bus *bus = class_bus(dev);
	struct devfreq *df = class_devfreq(dev);
	unsigned int i;
	int written = 0;

	mutex_lock(&df->lock);
	for (i = 0; i < bus->interactive.count; i++)
		written += sysfs_emit_at(buf, written, "%u%s", bus->interactive.tokens[i],
			(i + 1 == bus->interactive.count) ? "\n" : (i & 1) ? ":" : " ");
	mutex_unlock(&df->lock);
	return written;
}

static ssize_t delay_time_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct crownlte_bus *bus = class_bus(dev);
	struct devfreq *df = class_devfreq(dev);
	struct exynos9810_interactive_config next;
	int ret;

	mutex_lock(&df->lock);
	next = bus->interactive;
	ret = exynos9810_interactive_parse_delay(buf, count, &next);
	if (!ret) {
		bus->interactive = next;
		ret = update_devfreq(df);
	}
	mutex_unlock(&df->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR(delay_time, 0640, delay_time_show, delay_time_store);

static struct attribute *bus_attrs[] = {
	&dev_attr_scaling_devfreq_min.attr, NULL,
};
static const struct attribute_group bus_group = { .attrs = bus_attrs };
static struct attribute *interactive_attrs[] = {
	&dev_attr_use_delay_time.attr, &dev_attr_delay_time.attr, NULL,
};
static const struct attribute_group interactive_group = {
	.name = "interactive", .attrs = interactive_attrs,
};

static void bus_cleanup(void *data)
{
	struct crownlte_bus *bus = data;
	int ret;

	if (bus->reboot_registered) {
		unregister_reboot_notifier(&bus->reboot_notifier);
		bus->reboot_registered = false;
	}
	exynos9810_legacy_qos_stop(&bus->legacy_qos);

	mutex_lock(&bus_registry_lock);
	if (bus->listed) {
		bus_registry[bus->slot] = NULL;
		bus->listed = false;
	}
	mutex_unlock(&bus_registry_lock);
	cancel_delayed_work_sync(&bus->boot_work);
	if (bus->sysfs_ready) {
		sysfs_remove_group(&bus->df->dev.kobj, &interactive_group);
		sysfs_remove_group(&bus->df->dev.kobj, &bus_group);
	}
	/* Stop local target operations before retiring the outgoing guard.
	 * Incoming linked consumers have already been unbound by driver core.
	 */
	if (bus->df)
		mutex_lock(&bus->df->lock);
	WRITE_ONCE(bus->retired, true);
	if (bus->df)
		mutex_unlock(&bus->df->lock);
	exynos9810_bus_transition_fini(&bus->transition);
	if (bus->df) {
		ret = exynos9810_bus_guard_forget_supplier(bus->df);
		if (ret)
			dev_err(bus->dev, "incoming guard retained: %d\n", ret);
		devfreq_remove_device(bus->df);
		bus->df = NULL;
	}
	exynos9810_legacy_qos_detach(&bus->legacy_qos);
	bus_remove_requests(bus);
	if (dev_get_drvdata(bus->dev) == bus)
		dev_set_drvdata(bus->dev, NULL);
}

/* The registry lock protects these short reads against provider withdrawal.
 * It never spans IPC, frequency operations or work cancellation.
 */
static bool bus_pm_order(struct crownlte_bus *bus, bool resume)
{
	bool valid;
	unsigned int i;

	mutex_lock(&bus_registry_lock);
	valid = buses_ready();
	if (valid && resume) {
		for (i = bus->slot + 1; i < 3; i++)
			valid &= READ_ONCE(bus_registry[i]->pm_phase) == BUS_ACTIVE;
	} else if (valid) {
		for (i = 0; i < bus->slot; i++)
			valid &= READ_ONCE(bus_registry[i]->pm_phase) == BUS_SLEEP;
	}
	mutex_unlock(&bus_registry_lock);
	return valid;
}

static int bus_update(struct crownlte_bus *bus)
{
	int ret;

	mutex_lock(&bus->df->lock);
	ret = update_devfreq(bus->df);
	mutex_unlock(&bus->df->lock);
	return ret;
}

static void bus_pause_boot(struct crownlte_bus *bus)
{
	WRITE_ONCE(bus->boot_paused, true);
	cancel_delayed_work_sync(&bus->boot_work);
}

static int bus_unfreeze(struct crownlte_bus *bus)
{
	int ret = 0;

	if (bus->pm_frozen) {
		bus->pm_frozen = false;
		ret = devfreq_resume_device(bus->df);
	}
	if (!ret) {
		WRITE_ONCE(bus->boot_paused, false);
		if (bus->boot_min.data.freq.pnode.prio > 0)
			mod_delayed_work(system_wq, &bus->boot_work, HZ);
	}
	return ret;
}

static int bus_abort_sleep(struct crownlte_bus *bus)
{
	int ret;

	if (bus->slot < 2) {
		/* Even a failed AP-sleep command may have reached firmware. Restore
		 * acknowledged AP ownership before any ordinary CAL rate write.
		 */
		mutex_lock(&bus->df->lock);
		ret = exynos9810_bus_set_ap_running(bus->dev->of_node, bus->table.cal_id, 1);
		if (!ret) {
			bus->forced_hz = 0;
			bus->pm_phase = BUS_ACTIVE;
			ret = update_devfreq(bus->df);
		}
		if (ret)
			bus->pm_phase = BUS_PM_FAILED;
		mutex_unlock(&bus->df->lock);
	} else {
		/* Updating our own QoS request can synchronously lock df->lock. */
		ret = dev_pm_qos_update_request(&bus->default_min, bus->freq_info[1]);
		if (ret >= 0)
			ret = bus_update(bus);
		mutex_lock(&bus->df->lock);
		bus->pm_phase = ret < 0 ? BUS_PM_FAILED : BUS_ACTIVE;
		mutex_unlock(&bus->df->lock);
	}
	if (!ret)
		ret = bus_unfreeze(bus);
	if (ret) {
		mutex_lock(&bus->df->lock);
		bus->pm_phase = BUS_PM_FAILED;
		mutex_unlock(&bus->df->lock);
	}
	return ret;
}

static int __maybe_unused bus_suspend_late(struct device *dev)
{
	struct crownlte_bus *bus = dev_get_drvdata(dev);
	int ret, repair;

	if (!bus)
		return -ENODEV;
	mutex_lock(&bus->pm_lock);
	if (bus->retired || bus->pm_phase != BUS_ACTIVE || !bus_pm_order(bus, false)) {
		ret = -EBUSY;
		goto out;
	}
	bus_pause_boot(bus);
	/* ACK increments suspend_count before invoking its governor, including
	 * error returns. Balance our contribution during rollback in either case.
	 */
	bus->pm_frozen = true;
	ret = devfreq_suspend_device(bus->df);
	if (ret) {
		repair = bus_unfreeze(bus);
		if (repair) {
			mutex_lock(&bus->df->lock);
			bus->pm_phase = BUS_PM_FAILED;
			mutex_unlock(&bus->df->lock);
			dev_err(dev, "governor freeze rollback failed: %d\n", repair);
		}
		goto out;
	}
	mutex_lock(&bus->df->lock);
	bus->pm_phase = BUS_ENTERING_SLEEP;
	if (bus->slot < 2) {
		bus->forced_hz = bus->table.resume_khz * 1000UL;
		ret = exynos9810_bus_set_ap_running(dev->of_node, bus->table.cal_id, 0);
		if (!ret)
			ret = update_devfreq(bus->df);
	} else {
		ret = 0;
	}
	mutex_unlock(&bus->df->lock);
	if (bus->slot == 2) {
		ret = dev_pm_qos_update_request(&bus->default_min, bus->table.resume_khz);
		if (ret >= 0)
			ret = bus_update(bus);
	}
	if (ret) {
		repair = bus_abort_sleep(bus);
		if (repair)
			dev_err(dev, "sleep rollback failed, retained protective state: %d\n", repair);
		goto out;
	}
	mutex_lock(&bus->df->lock);
	bus->pm_phase = BUS_SLEEP;
	mutex_unlock(&bus->df->lock);
out:
	mutex_unlock(&bus->pm_lock);
	return ret;
}

static int __maybe_unused bus_resume_early(struct device *dev)
{
	struct crownlte_bus *bus = dev_get_drvdata(dev);
	int ret;

	if (!bus)
		return -ENODEV;
	mutex_lock(&bus->pm_lock);
	if (bus->retired || bus->pm_phase != BUS_SLEEP || !bus_pm_order(bus, true)) {
		ret = -EBUSY;
		goto out;
	}
	mutex_lock(&bus->df->lock);
	bus->pm_phase = BUS_LEAVING_SLEEP;
	if (bus->slot < 2) {
		ret = exynos9810_bus_set_ap_running(dev->of_node, bus->table.cal_id, 1);
		if (!ret) {
			bus->forced_hz = 0;
			ret = update_devfreq(bus->df);
		}
	} else {
		ret = 0;
	}
	mutex_unlock(&bus->df->lock);
	if (bus->slot == 2) {
		ret = dev_pm_qos_update_request(&bus->default_min, bus->freq_info[1]);
		if (ret >= 0)
			ret = bus_update(bus);
	}
	if (!ret)
		ret = bus_unfreeze(bus);
	mutex_lock(&bus->df->lock);
	bus->pm_phase = ret ? BUS_PM_FAILED : BUS_ACTIVE;
	mutex_unlock(&bus->df->lock);
out:
	mutex_unlock(&bus->pm_lock);
	return ret;
}

static int bus_reboot(struct notifier_block *nb, unsigned long action, void *data)
{
	struct crownlte_bus *bus = container_of(nb, struct crownlte_bus, reboot_notifier);
	unsigned int khz = min_t(unsigned int, bus->freq_info[5],
				bus->frequencies[bus->table.count - 1] / 1000);
	unsigned long actual;
	int ret;

	mutex_lock(&bus->pm_lock);
	if (bus->retired || bus->pm_phase != BUS_ACTIVE) {
		ret = -EBUSY;
		goto out;
	}
	bus_pause_boot(bus);
	ret = dev_pm_qos_update_request(&bus->default_min, khz);
	if (ret >= 0)
		ret = dev_pm_qos_update_request(&bus->reboot_max, khz);
	if (ret >= 0)
		ret = bus_update(bus);
	if (!ret) {
		ret = bus_current(bus->dev, &actual);
		if (!ret && actual < khz * 1000UL)
			ret = -ERANGE;
	}
out:
	mutex_unlock(&bus->pm_lock);
	if (ret)
		dev_err(bus->dev, "reboot bus request failed: %d\n", ret);
	return ret ? NOTIFY_BAD : NOTIFY_OK;
}

static int bus_read_binding(struct crownlte_bus *bus)
{
	struct device_node *np = bus->dev->of_node;
	const char *type, *text, *governor;
	u32 id, original_id, polling, channel;
	unsigned int i;
	int ret, slot;

	if (of_property_read_u32(np, "samsung,cal-id", &id) ||
	    of_property_read_u32(np, "dfs_id", &original_id) || id != original_id ||
	    of_property_read_string(np, "devfreq_type", &type))
		return -EINVAL;
	slot = bus_slot(id);
	if (slot < 0 || strcmp(type, slot == 0 ? "mif" : slot == 1 ? "int" : "fsys0"))
		return -EINVAL;
	bus->slot = slot;
	if (slot < 2) {
		if (of_property_read_string(np, "use_acpm", &text) || strcmp(text, "true") ||
		    of_property_read_u32(np, "acpm-ipc-channel", &channel) || channel != 1)
			return -EINVAL;
	} else if (!of_property_read_string(np, "use_acpm", &text) && strcmp(text, "false")) {
		return -EINVAL;
	}
	if (of_property_read_u32_array(np, "freq_info", bus->freq_info, 6) ||
	    of_property_read_u32_array(np, "boot_info", bus->boot_info, 2) ||
	    of_property_read_u32(np, "polling_ms", &polling) || polling ||
	    of_property_read_string(np, "gov_name", &governor) ||
	    strcmp(governor, "interactive"))
		return -EINVAL;
	/* The actual phone has no AP-controlled regulator, load polling,
	 * switching clock or closed-loop voltage controller for these buses.
	 */
	if (of_property_read_string(np, "use_reg", &text) || strcmp(text, "false") ||
	    of_property_read_string(np, "use_get_dev", &text) || strcmp(text, "false") ||
	    of_property_read_string(np, "use_sw_clk", &text) || strcmp(text, "false") ||
	    of_property_read_string(np, "use_cl_dvfs", &text) || strcmp(text, "false"))
		return -EINVAL;
	if (!bus->freq_info[0] || !bus->freq_info[1] || !bus->freq_info[2] ||
	    !bus->freq_info[3] || bus->freq_info[3] > bus->freq_info[4] ||
	    bus->freq_info[0] > INT_MAX || bus->freq_info[1] > INT_MAX ||
	    bus->freq_info[2] > INT_MAX ||
	    bus->freq_info[4] > INT_MAX || !bus->freq_info[5] ||
	    bus->freq_info[5] > INT_MAX || !bus->boot_info[1] ||
	    bus->boot_info[1] > INT_MAX || bus->boot_info[0] > INT_MAX / HZ)
		return -ERANGE;
	ret = exynos9810_bus_build_table(id, bus->freq_info[3],
					bus->freq_info[4], &bus->table);
	if (ret)
		return ret;
	/* Forced suspend rates must be exact calibrated rows, never extrapolated. */
	for (i = 0; i < bus->table.count; i++)
		if (bus->table.levels[i].hz == bus->table.resume_khz * 1000UL)
			break;
	if (i == bus->table.count)
		return -ERANGE;
	if (of_property_read_string(np, "use_delay_time", &text))
		return -EINVAL;
	if (!strcmp(text, "true"))
		bus->interactive.use_delay_time = true;
	else if (strcmp(text, "false"))
		return -EINVAL;
	bus->interactive.count = 1;
	bus->interactive.tokens[0] = 10;
	if (!of_property_read_string(np, "delay_time_list", &text))
		return exynos9810_interactive_parse_delay(text, strlen(text), &bus->interactive);
	return exynos9810_interactive_validate(&bus->interactive);
}

static int bus_legacy_apply(void *context)
{
	struct crownlte_bus *bus = context;

	/* Early requests are retained before the class/governor exists. Its first
	 * explicit update consumes them once registration has finished.
	 */
	if (IS_ERR_OR_NULL(READ_ONCE(bus->df)))
		return 0;
	if (READ_ONCE(bus->retired))
		return -ENODEV;
	return bus_update(bus);
}

static int crownlte_bus_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct crownlte_bus *bus;
	struct exynos9810_mif_bts *bts = NULL;
	struct devfreq *dependency;
	unsigned long actual;
	unsigned int i, maximum;
	int ret;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;
	kref_init(&bus->refs);
	mutex_init(&bus->pm_lock);
	bus->dev = get_device(dev);
	INIT_DELAYED_WORK(&bus->boot_work, bus_boot_work);
	ret = devm_add_action_or_reset(dev, bus_owner_put, bus);
	if (ret)
		return ret;
	ret = bus_read_binding(bus);
	if (ret)
		return dev_err_probe(dev, ret, "invalid calibrated bus binding\n");
	dependency = bus_dependency(bus);
	if (IS_ERR(dependency))
		return dev_err_probe(dev, PTR_ERR(dependency), "bus supplier unavailable\n");
	if (bus->slot == 0) {
		bts = exynos9810_mif_bts_get(dev);
		if (IS_ERR(bts))
			return dev_err_probe(dev, PTR_ERR(bts), "MIF BTS unavailable\n");
		ret = exynos9810_bus_upload_mif_constraints(dev->of_node, &bus->table);
		if (ret)
			return dev_err_probe(dev, ret, "MIF FVP upload failed\n");
	}
	ret = exynos9810_bus_transition_init(&bus->transition, dev,
					&bus->table, dependency, bts);
	if (ret)
		return dev_err_probe(dev, ret, "bus transition initialization failed\n");
	/* Cleanup precedes the managed transition cleanup and supplier put.
	 * Explicit fini is idempotent, preserving a failed consumer's guard.
	 */
	ret = devm_add_action_or_reset(dev, bus_cleanup, bus);
	if (ret)
		return ret;
	for (i = 0; i < bus->table.count; i++) {
		const struct exynos9810_bus_level *level =
			&bus->table.levels[bus->table.count - 1 - i];
		bus->frequencies[i] = level->hz;
		ret = dev_pm_opp_add(dev, level->hz, level->uv);
		if (ret)
			return dev_err_probe(dev, ret, "bus OPP registration failed\n");
		bus->added_opps++;
	}
	maximum = bus->frequencies[bus->table.count - 1] / 1000;
	ret = dev_pm_qos_add_request(dev, &bus->default_min,
				    DEV_PM_QOS_MIN_FREQUENCY, bus->freq_info[1]);
	if (ret < 0)
		return ret;
	ret = dev_pm_qos_add_request(dev, &bus->system_min,
				    DEV_PM_QOS_MIN_FREQUENCY, bus->frequencies[0] / 1000);
	if (ret < 0)
		return ret;
	ret = dev_pm_qos_add_request(dev, &bus->boot_min,
				    DEV_PM_QOS_MIN_FREQUENCY, maximum);
	if (ret < 0)
		return ret;
	ret = dev_pm_qos_add_request(dev, &bus->reboot_max,
				    DEV_PM_QOS_MAX_FREQUENCY, PM_QOS_MAX_FREQUENCY_DEFAULT_VALUE);
	if (ret < 0)
		return ret;
	ret = exynos9810_legacy_qos_attach(&bus->legacy_qos, dev, maximum,
					 bus_legacy_apply, bus);
	if (ret)
		return dev_err_probe(dev, ret, "legacy bus QoS bridge unavailable\n");
	bus->boot_expires = jiffies + bus->boot_info[0] * HZ;
	ret = exynos9810_bus_transition_current(&bus->transition, &actual);
	if (ret)
		return ret;
	bus->profile.initial_freq = actual;
	bus->profile.target = bus_target;
	bus->profile.get_cur_freq = bus_current;
	bus->profile.acquire = bus_class_acquire;
	bus->profile.release = bus_class_release;
	bus->profile.freq_table = bus->frequencies;
	bus->profile.max_state = bus->table.count;
	bus->profile.legacy_khz_sysfs = true;
	bus->profile.defer_qos_updates = true;
	platform_set_drvdata(pdev, bus);
	/* Core acquires its storage loan exactly when class lifetime starts.
	 * A failed registration can still have a delayed release callback, so
	 * the driver must never infer ownership from the returned error pointer.
	 */
	bus->df = devfreq_add_device(dev, &bus->profile, "interactive", &bus->interactive);
	if (IS_ERR(bus->df)) {
		ret = PTR_ERR(bus->df);
		bus->df = NULL;
		return dev_err_probe(dev, ret, "devfreq class registration failed\n");
	}
	mutex_lock(&bus->df->lock);
	ret = update_devfreq(bus->df);
	mutex_unlock(&bus->df->lock);
	if (ret)
		return dev_err_probe(dev, ret, "bus bootstrap transition failed\n");
	ret = sysfs_create_group(&bus->df->dev.kobj, &bus_group);
	if (ret)
		return ret;
	ret = sysfs_create_group(&bus->df->dev.kobj, &interactive_group);
	if (ret) {
		sysfs_remove_group(&bus->df->dev.kobj, &bus_group);
		return ret;
	}
	bus->sysfs_ready = true;
	bus->reboot_notifier.notifier_call = bus_reboot;
	ret = register_reboot_notifier(&bus->reboot_notifier);
	if (ret)
		return ret;
	bus->reboot_registered = true;
	mutex_lock(&bus_registry_lock);
	if (bus_registry[bus->slot]) {
		mutex_unlock(&bus_registry_lock);
		return -EEXIST;
	}
	bus_registry[bus->slot] = bus;
	bus->listed = true;
	mutex_unlock(&bus_registry_lock);
	mod_delayed_work(system_wq, &bus->boot_work, HZ);
	dev_info(dev, "calibrated bus registered; bootstrap retained until all buses ready\n");
	return 0;
}

static void crownlte_bus_remove(struct platform_device *pdev)
{
	/* Managed cleanup runs while private data remains valid. Do not clear
	 * drvdata before its governor, sysfs and target callbacks have stopped.
	 */
}

static const struct of_device_id crownlte_bus_match[] = {
	{ .compatible = "samsung,exynos9810-bus-devfreq" }, {}
};
MODULE_DEVICE_TABLE(of, crownlte_bus_match);
static const struct dev_pm_ops bus_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(bus_suspend_late, bus_resume_early)
};
static struct platform_driver crownlte_bus_driver = {
	.probe = crownlte_bus_probe,
	.remove = crownlte_bus_remove,
	.driver = {
		.name = "exynos9810-bus-devfreq",
		.of_match_table = crownlte_bus_match,
		.pm = &bus_pm_ops,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(crownlte_bus_driver);
MODULE_LICENSE("GPL");
