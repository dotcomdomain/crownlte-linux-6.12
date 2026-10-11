// SPDX-License-Identifier: GPL-2.0-only
/* Original six bus character controls, with real registered-provider bridges.
 * Static classes retain early/open-FD requests across provider reprobes.
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/plist.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <soc/samsung/exynos9810-legacy-qos.h>
#include "exynos9810-bus-tables.h"
#include "exynos9810-bus-guard.h"

struct legacy_qos_class {
	const char *name;
	struct miscdevice misc;
	struct mutex lock;
	struct pm_qos_constraints constraints;
	struct exynos9810_legacy_qos_bridge *bridge;
	struct exynos9810_legacy_qos_bridge *reservation;
	bool maximum, force_notify;
};

struct legacy_qos_file {
	struct legacy_qos_class *class;
	struct plist_node request;
};

static struct legacy_qos_class classes[] = {
	{ .name = "bus_throughput" },
	{ .name = "bus_throughput_max", .maximum = true },
	{ .name = "device_throughput", .force_notify = true },
	{ .name = "device_throughput_max", .maximum = true },
	{ .name = "fsys0_throughput" },
	{ .name = "fsys0_throughput_max", .maximum = true },
};
static bool frontend_ready;

/* Old signed negative maxima become very large unsigned governor limits.
 * Their readback stays signed, but the native frequency request represents
 * the resulting unlimited cap. Negative minima cannot beat the provider's
 * nonnegative system/default/boot requests.
 */
static s32 native_value(struct legacy_qos_class *class)
{
	s32 value = pm_qos_read_value(&class->constraints);

	if (value < 0)
		return class->maximum ? PM_QOS_MAX_FREQUENCY_DEFAULT_VALUE : 0;
	return value;
}

/* class->lock serializes mutations, bridge publication and callbacks.
 * Current providers update only native dependency requests from their targets,
 * so callbacks never mutate a legacy class while holding a devfreq lock.
 */
static void apply_class(struct legacy_qos_class *class)
{
	struct exynos9810_legacy_qos_bridge *bridge = class->bridge;
	struct dev_pm_qos_request *request;
	int ret;

	if (!bridge)
		return;
	request = class->maximum ? &bridge->maximum : &bridge->minimum;
	ret = dev_pm_qos_update_request(request, native_value(class));
	/* Native QoS drops unchanged aggregate notifications. Explicitly reconcile
	 * the original FORCE_MAX case as well as real target errors hidden by the
	 * native notifier return path. Original writes still return their count.
	 */
	if (ret >= 0 && bridge->apply)
		ret = bridge->apply(bridge->context);
	if (ret < 0)
		dev_err_ratelimited(bridge->dev, "%s clock request retained; apply failed: %d\n",
				    class->name, ret);
}

static void mutate(struct legacy_qos_class *class, struct plist_node *node,
		enum pm_qos_req_action action, s32 value)
{
	int changed;

	changed = pm_qos_update_target(&class->constraints, node, action, value);
	if (changed || class->force_notify)
		apply_class(class);
}

static int legacy_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct legacy_qos_class *class = container_of(misc, struct legacy_qos_class, misc);
	struct legacy_qos_file *request;

	if (!READ_ONCE(frontend_ready))
		return -ENODEV;
	request = kzalloc(sizeof(*request), GFP_KERNEL);
	if (!request)
		return -ENOMEM;
	request->class = class;
	mutex_lock(&class->lock);
	mutate(class, &request->request, PM_QOS_ADD_REQ, PM_QOS_DEFAULT_VALUE);
	mutex_unlock(&class->lock);
	file->private_data = request;
	return 0;
}

static int legacy_release(struct inode *inode, struct file *file)
{
	struct legacy_qos_file *request = file->private_data;
	struct legacy_qos_class *class;

	if (!request)
		return -EINVAL;
	class = request->class;
	mutex_lock(&class->lock);
	mutate(class, &request->request, PM_QOS_REMOVE_REQ, PM_QOS_DEFAULT_VALUE);
	mutex_unlock(&class->lock);
	kfree(request);
	file->private_data = NULL;
	return 0;
}

static s32 aggregate_value(struct legacy_qos_class *class)
{
	struct exynos9810_legacy_qos_bridge *bridge = class->bridge;
	s32 legacy = pm_qos_read_value(&class->constraints), native;

	if (!bridge)
		return legacy;
	/* Keep system/default/boot/thermal and real client votes. Original DM
	 * guidance floors were internal constraints, not global-class requests;
	 * omit exactly that incoming guard from MIN readback, never enforcement.
	 * Original MAX readback and its supported-maximum seed stay unchanged.
	 */
	if (class->maximum) {
		native = dev_pm_qos_read_value(bridge->dev, DEV_PM_QOS_MAX_FREQUENCY);
		return min(legacy, min_t(s32, native, bridge->maximum_khz));
	}
	native = dev_pm_qos_read_min_freq_excluding(bridge->dev,
			exynos9810_bus_guard_min_request(bridge->dev));
	return max(legacy, native);
}

static ssize_t legacy_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	struct legacy_qos_file *request = file->private_data;
	struct legacy_qos_class *class;
	s32 value;

	if (!request)
		return -EINVAL;
	class = request->class;
	mutex_lock(&class->lock);
	value = aggregate_value(class);
	mutex_unlock(&class->lock);
	return simple_read_from_buffer(buf, count, offset, &value, sizeof(value));
}

static ssize_t legacy_write(struct file *file, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct legacy_qos_file *request = file->private_data;
	struct legacy_qos_class *class;
	s32 value;
	int ret;

	if (!request)
		return -EINVAL;
	/* Count==4 means native binary even if all four bytes look like text. */
	if (count == sizeof(value)) {
		if (copy_from_user(&value, buf, sizeof(value)))
			return -EFAULT;
	} else {
		ret = kstrtos32_from_user(buf, count, 16, &value);
		if (ret)
			return ret;
	}
	class = request->class;
	mutex_lock(&class->lock);
	/* Preserve the old per-request no-op, including DEFAULT=-1 being compared
	 * with the stored default priority before the generic helper maps it.
	 */
	if (value != request->request.prio)
		mutate(class, &request->request, PM_QOS_UPDATE_REQ, value);
	mutex_unlock(&class->lock);
	return count;
}

static const struct file_operations legacy_fops = {
	.owner = THIS_MODULE,
	.open = legacy_open, .release = legacy_release,
	.read = legacy_read, .write = legacy_write,
	.llseek = noop_llseek,
};

static int first_class(unsigned int cal_id)
{
	switch (cal_id) {
	case EXYNOS9810_MIF_CAL_ID: return 0;
	case EXYNOS9810_INT_CAL_ID: return 2;
	case EXYNOS9810_FSYS0_CAL_ID: return 4;
	default: return -EINVAL;
	}
}

static void detach_one(struct legacy_qos_class *class,
		struct exynos9810_legacy_qos_bridge *bridge)
{
	struct dev_pm_qos_request *request = class->maximum ? &bridge->maximum : &bridge->minimum;

	mutex_lock(&class->lock);
	if (class->bridge == bridge)
		class->bridge = NULL;
	if (dev_pm_qos_request_active(request))
		dev_pm_qos_remove_request(request);
	if (class->reservation == bridge)
		class->reservation = NULL;
	mutex_unlock(&class->lock);
}

void exynos9810_legacy_qos_stop(struct exynos9810_legacy_qos_bridge *bridge)
{
	struct legacy_qos_class *class;
	unsigned int i;

	if (!bridge || !bridge->dev)
		return;
	for (i = 0; i < 2; i++) {
		class = &classes[bridge->first_class + i];
		/* Every frontend callback holds this mutex through apply(). Clearing
		 * publication therefore drains callers before devfreq can be freed.
		 */
		mutex_lock(&class->lock);
		if (class->bridge == bridge)
			class->bridge = NULL;
		mutex_unlock(&class->lock);
	}
}

void exynos9810_legacy_qos_detach(struct exynos9810_legacy_qos_bridge *bridge)
{
	if (!bridge || !bridge->dev)
		return;
	exynos9810_legacy_qos_stop(bridge);
	if (bridge->minimum_attached)
		detach_one(&classes[bridge->first_class], bridge);
	if (bridge->maximum_attached)
		detach_one(&classes[bridge->first_class + 1], bridge);
	bridge->minimum_attached = bridge->maximum_attached = false;
	bridge->dev = NULL;
}

static int attach_one(struct legacy_qos_class *class,
		struct exynos9810_legacy_qos_bridge *bridge)
{
	struct dev_pm_qos_request *request = class->maximum ? &bridge->maximum : &bridge->minimum;
	int ret;

	mutex_lock(&class->lock);
	if (class->reservation) {
		ret = -EBUSY;
		goto out;
	}
	ret = dev_pm_qos_add_request(bridge->dev, request, class->maximum ?
				    DEV_PM_QOS_MAX_FREQUENCY : DEV_PM_QOS_MIN_FREQUENCY,
				    native_value(class));
	if (ret >= 0) {
		class->bridge = bridge;
		class->reservation = bridge;
		if (class->maximum)
			bridge->maximum_attached = true;
		else
			bridge->minimum_attached = true;
		ret = 0;
	}
out:
	mutex_unlock(&class->lock);
	return ret;
}

int exynos9810_legacy_qos_attach(struct exynos9810_legacy_qos_bridge *bridge,
		struct device *dev, unsigned int maximum_khz,
		int (*apply)(void *context), void *context)
{
	u32 id;
	int first, ret;

	if (!bridge || !dev || !apply || bridge->dev || !maximum_khz ||
	    maximum_khz > INT_MAX || of_property_read_u32(dev->of_node, "samsung,cal-id", &id))
		return -EINVAL;
	first = first_class(id);
	if (first < 0)
		return first;
	if (!READ_ONCE(frontend_ready))
		return -EPROBE_DEFER;
	bridge->dev = dev;
	bridge->first_class = first;
	bridge->maximum_khz = maximum_khz;
	bridge->apply = apply;
	bridge->context = context;
	ret = attach_one(&classes[first], bridge);
	if (!ret)
		ret = attach_one(&classes[first + 1], bridge);
	if (ret)
		exynos9810_legacy_qos_detach(bridge);
	return ret;
}

static int __init legacy_qos_init(void)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(classes); i++) {
		struct legacy_qos_class *class = &classes[i];
		s32 initial = class->maximum ? INT_MAX : 0;

		mutex_init(&class->lock);
		plist_head_init(&class->constraints.list);
		class->constraints.default_value = initial;
		class->constraints.no_constraint_value = initial;
		class->constraints.target_value = initial;
		class->constraints.type = class->maximum ? PM_QOS_MIN : PM_QOS_MAX;
		class->misc.minor = MISC_DYNAMIC_MINOR;
		class->misc.name = class->name;
		class->misc.fops = &legacy_fops;
	}
	for (i = 0; i < ARRAY_SIZE(classes); i++) {
		ret = misc_register(&classes[i].misc);
		if (ret) {
			while (i)
				misc_deregister(&classes[--i].misc);
			return ret;
		}
	}
	WRITE_ONCE(frontend_ready, true);
	return 0;
}
subsys_initcall(legacy_qos_init);
MODULE_LICENSE("GPL");
