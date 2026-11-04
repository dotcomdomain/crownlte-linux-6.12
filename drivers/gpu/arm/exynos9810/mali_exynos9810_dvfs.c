// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos9810 (crownlte) Mali-G72 DVFS.
 *
 * Levels come from the DT samsung,dvfs-table (4.9 gpu_dvfs_table columns
 * clock, min_threshold, max_threshold, down_staycount, mem_freq). Every
 * level must be an exact CAL rate of dvfs_g3d inside CAL's min/max; rates go
 * to cal_dfs_set_rate() (ACPM vclk: the firmware programs PLL_G3D and picks
 * the BUCK6 voltage from the FVMap), and the applied rate is the CMU
 * readback cal_dfs_get_rate(). Policy is the 4.9 "interactive" governor fed
 * by kbase's own utilisation metrics (CONFIG_MALI_MIDGARD_DVFS). Limits:
 * thermal cooling device "thermal-gpufreq-0" (the vendor thermal HAL name)
 * and /sys/kernel/gpu/gpu_{max,min}_clock (vendor power HAL). Each level can
 * raise a MIF floor through the MIF devfreq (4.9 mem_freq column).
 */
#include <linux/bitops.h>
#include <linux/devfreq.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>

#include <mali_kbase.h>
#include <backend/gpu/mali_kbase_pm_internal.h>
#include <soc/samsung/cal-if.h>

#include "mali_exynos9810.h"

#define XG_CAL_MAX_LEVELS	48
#define XG_IDLE_DROP_MS		100
#define XG_DEFAULT_PERIOD_MS	30	/* 4.9 gpu_dvfs_polling_time */

int xg_dvfs_khz_to_step(struct exynos9810_gpu *g, unsigned int khz)
{
	unsigned int i;

	for (i = 0; i < g->nr_levels; i++)
		if (g->levels[i].khz == khz)
			return i;
	return -EINVAL;
}

static bool xg_cal_has_rate(const unsigned long *rates, int n, unsigned int khz)
{
	int i;

	for (i = 0; i < n; i++)
		if (rates[i] == khz)
			return true;
	return false;
}

int xg_dvfs_init(struct exynos9810_gpu *g)
{
	struct device *dev = g->dev;
	struct device_node *np = dev->of_node;
	struct dvfs_rate_volt rv[XG_CAL_MAX_LEVELS];
	unsigned long rates[XG_CAL_MAX_LEVELS];
	u32 tbl[XG_MAX_LEVELS * XG_TABLE_COLS];
	u32 hs[3];
	unsigned long cal_min, cal_max, cur;
	unsigned int lv;
	int n, nv, cnt, i;

	if (of_property_read_u32(np, "samsung,cal-id", &g->cal_id) || !g->cal_id) {
		dev_err(dev, XG_PFX "no samsung,cal-id\n");
		return -EINVAL;
	}
	if (!cal_if_is_initialized())
		return dev_err_probe(dev, -EPROBE_DEFER, XG_PFX "CAL not initialized\n");
	if (!cal_dfs_is_ready(g->cal_id))
		return dev_err_probe(dev, -EPROBE_DEFER,
				     XG_PFX "CAL %#x not ready (ACPM DVFS / FVMap)\n", g->cal_id);

	lv = cal_dfs_get_lv_num(g->cal_id);
	if (!lv || lv > XG_CAL_MAX_LEVELS) {
		dev_err(dev, XG_PFX "CAL %#x has %u levels\n", g->cal_id, lv);
		return -ENODEV;
	}
	n = cal_dfs_get_rate_table(g->cal_id, rates);
	if (n != lv) {
		dev_err(dev, XG_PFX "CAL %#x rate table has %d of %u levels\n", g->cal_id, n, lv);
		return -ENODEV;
	}
	cal_min = cal_dfs_get_min_freq(g->cal_id);
	cal_max = cal_dfs_get_max_freq(g->cal_id);
	nv = cal_dfs_get_rate_asv_table(g->cal_id, rv);
	for (i = 0; i < n; i++)
		dev_info(dev, XG_PFX "CAL level %d: %lu kHz %u uV\n", i, rates[i],
			 i < nv ? rv[i].volt : 0);

	/* CMU readback: the current rate must be one CAL level. */
	cur = cal_dfs_get_rate(g->cal_id);
	if (!cur || !xg_cal_has_rate(rates, n, cur)) {
		dev_err(dev, XG_PFX "CMU readback %lu kHz is not a CAL level of %#x\n", cur, g->cal_id);
		return -ENODEV;
	}
	g->boot_khz = cur;

	cnt = of_property_count_u32_elems(np, "samsung,dvfs-table");
	if (cnt <= 0) {
		dev_warn(dev, XG_PFX "no samsung,dvfs-table: DVFS off, clock stays at %lu kHz\n", cur);
		g->dvfs_enabled = false;
		return 0;
	}
	if (cnt % XG_TABLE_COLS || cnt > ARRAY_SIZE(tbl)) {
		dev_err(dev, XG_PFX "samsung,dvfs-table has %d cells\n", cnt);
		return -EINVAL;
	}
	if (of_property_read_u32_array(np, "samsung,dvfs-table", tbl, cnt))
		return -EINVAL;
	g->nr_levels = cnt / XG_TABLE_COLS;
	for (i = 0; i < g->nr_levels; i++) {
		struct xg_level *l = &g->levels[i];
		const u32 *r = &tbl[i * XG_TABLE_COLS];

		l->khz = r[0];
		l->min_util = r[1];
		l->max_util = r[2];
		l->staycount = max_t(u32, r[3], 1);
		l->mif_khz = r[4];
		if (!xg_cal_has_rate(rates, n, l->khz) || l->khz < cal_min || l->khz > cal_max ||
		    (i && l->khz >= g->levels[i - 1].khz) ||
		    l->min_util > l->max_util || l->max_util > 100) {
			dev_err(dev, XG_PFX "dvfs-table row %d (%u kHz %u-%u%%) invalid (CAL %lu-%lu kHz)\n",
				i, l->khz, l->min_util, l->max_util, cal_min, cal_max);
			return -EINVAL;
		}
	}

	g->hs_step = 0;	/* 0 = no highspeed jump */
	if (!of_property_read_u32_array(np, "samsung,interactive-info", hs, 3)) {
		int s = xg_dvfs_khz_to_step(g, hs[0]);

		if (s > 0) {
			g->hs_step = s;
			g->hs_load = hs[1];
			g->hs_delay = hs[2];
		}
	}

	g->user_max_step = 0;
	g->user_min_step = g->nr_levels - 1;
	g->thermal_cap = 0;
	i = xg_dvfs_khz_to_step(g, cur);
	g->cur_step = i >= 0 ? i : g->nr_levels;	/* nr_levels = not a table level */
	g->step = i >= 0 ? i : g->nr_levels - 1;
	g->down_req = g->levels[g->step].staycount;
	g->dvfs_enabled = true;
	return 0;
}

/* Allowed step window, g->lock held. lo = fastest, hi = slowest. */
static void xg_window(struct exynos9810_gpu *g, unsigned int *lo, unsigned int *hi)
{
	unsigned int l = max(g->thermal_cap, g->user_max_step);
	unsigned int h = max(g->user_min_step, l);

	*lo = min(l, g->nr_levels - 1);
	*hi = min(h, g->nr_levels - 1);
}

static unsigned int xg_clamp_step(struct exynos9810_gpu *g, unsigned int step)
{
	unsigned int lo, hi;

	xg_window(g, &lo, &hi);
	return clamp(step, lo, hi);
}

/* 4.9 gpu_dvfs_governor_interactive(), g->lock held. */
static void xg_governor(struct exynos9810_gpu *g, unsigned int util)
{
	unsigned int lo, hi, step;

	xg_window(g, &lo, &hi);
	step = clamp(g->step, lo, hi);

	if (step > lo && util > g->levels[step].max_util) {
		if (g->hs_step && step > g->hs_step && util > g->hs_load) {
			if (g->delay_count >= g->hs_delay) {
				step = g->hs_step;
				g->delay_count = 0;
			} else {
				g->delay_count++;
			}
		} else {
			step--;
			g->delay_count = 0;
		}
		step = max(step, lo);
		g->down_req = g->levels[step].staycount;
	} else if (step < hi && util < g->levels[step].min_util) {
		g->delay_count = 0;
		if (g->down_req)
			g->down_req--;
		if (!g->down_req) {
			step++;
			g->down_req = g->levels[step].staycount;
		}
	} else {
		g->delay_count = 0;
		g->down_req = g->levels[step].staycount;
	}
	g->step = step;
}

static void xg_kick(struct exynos9810_gpu *g)
{
	if (g->wq)
		queue_work(g->wq, &g->set_work);
}

/*
 * Called by kbase_pm_get_dvfs_action() from the metrics hrtimer (atomic):
 * only bookkeeping here, the CAL request runs on the workqueue.
 */
int kbase_platform_dvfs_event(struct kbase_device *kbdev, u32 utilisation,
			      u32 util_gl_share, u32 util_cl_share[2])
{
	struct exynos9810_gpu *g = kbdev->platform_context;
	unsigned long flags;
	bool kick;

	if (!g || !g->dvfs_enabled)
		return 1;
	spin_lock_irqsave(&g->lock, flags);
	g->last_util = utilisation;
	if (!g->hold)
		xg_governor(g, utilisation);
	kick = !g->hold && g->step != READ_ONCE(g->cur_step);
	spin_unlock_irqrestore(&g->lock, flags);
	if (kick)
		xg_kick(g);
	return 1;
}

static void xg_mif_lookup(struct exynos9810_gpu *g)
{
	struct devfreq *df;
	int ret;

	if (g->mif_lookup_done)
		return;
	if (!of_property_present(g->dev->of_node, "samsung,mif-devfreq")) {
		g->mif_lookup_done = true;
		return;
	}
	df = devfreq_get_devfreq_by_phandle(g->dev, "samsung,mif-devfreq", 0);
	if (IS_ERR(df))
		return;	/* not probed yet: retry at the next change */
	get_device(&df->dev);
	ret = dev_pm_qos_add_request(df->dev.parent, &g->mif_req,
				     DEV_PM_QOS_MIN_FREQUENCY, 0);
	if (ret < 0) {
		put_device(&df->dev);
		dev_warn(g->dev, XG_PFX "MIF floor request failed: %d\n", ret);
		g->mif_lookup_done = true;
		return;
	}
	g->mif = df;
	g->mif_lookup_done = true;
	dev_info(g->dev, XG_PFX "MIF floor through %s\n", dev_name(df->dev.parent));
}

static void xg_mif_set(struct exynos9810_gpu *g, unsigned int khz)
{
	if (g->mif && dev_pm_qos_request_active(&g->mif_req))
		dev_pm_qos_update_request(&g->mif_req, min_t(unsigned int, khz, S32_MAX));
}

/* set_lock held. */
static int xg_apply_locked(struct exynos9810_gpu *g, unsigned int step, unsigned long *readback)
{
	unsigned int khz, old_mif, new_mif;
	unsigned long rb;
	int ret, idx;

	if (step >= g->nr_levels)
		return -EINVAL;
	if (!xg_pd_is_on(g, NULL, NULL))
		return -EIO;

	khz = g->levels[step].khz;
	old_mif = g->cur_step < g->nr_levels ? g->levels[g->cur_step].mif_khz : 0;
	new_mif = g->levels[step].mif_khz;

	xg_mif_lookup(g);
	if (new_mif > old_mif)
		xg_mif_set(g, new_mif);

	ret = cal_dfs_set_rate(g->cal_id, khz);
	rb = cal_dfs_get_rate(g->cal_id);
	g->nr_sets++;
	if (readback)
		*readback = rb;
	if (ret || rb != khz) {
		g->nr_set_errors++;
		dev_warn_ratelimited(g->dev, XG_PFX "set %u kHz: ret %d, CMU readback %lu kHz\n",
				     khz, ret, rb);
		idx = xg_dvfs_khz_to_step(g, rb);
		WRITE_ONCE(g->cur_step, idx >= 0 ? idx : g->nr_levels);
		if (!ret)
			ret = -EIO;
	} else {
		WRITE_ONCE(g->cur_step, step);
	}

	xg_mif_set(g, g->cur_step < g->nr_levels ?
		   g->levels[g->cur_step].mif_khz : max(old_mif, new_mif));
	return ret;
}

int xg_dvfs_apply(struct exynos9810_gpu *g, unsigned int step, unsigned long *readback)
{
	int ret;

	if (!g->dvfs_enabled)
		return -EOPNOTSUPP;
	mutex_lock(&g->set_lock);
	ret = xg_apply_locked(g, step, readback);
	mutex_unlock(&g->set_lock);
	return ret;
}

static void xg_set_work_fn(struct work_struct *work)
{
	struct exynos9810_gpu *g = container_of(work, struct exynos9810_gpu, set_work);
	unsigned long flags;
	unsigned int target;
	bool hold;

	mutex_lock(&g->set_lock);
	spin_lock_irqsave(&g->lock, flags);
	target = xg_clamp_step(g, g->step);
	g->step = target;
	hold = g->hold;
	spin_unlock_irqrestore(&g->lock, flags);
	if (!hold && target != g->cur_step)
		xg_apply_locked(g, target, NULL);
	mutex_unlock(&g->set_lock);
}

/* GPU idle (kbase powered L2/cores off) for XG_IDLE_DROP_MS: lowest level. */
static void xg_idle_work_fn(struct work_struct *work)
{
	struct exynos9810_gpu *g = container_of(to_delayed_work(work),
						 struct exynos9810_gpu, idle_work);
	unsigned long flags;
	unsigned int lo, hi;

	spin_lock_irqsave(&g->lock, flags);
	if (g->gpu_on || g->hold) {
		spin_unlock_irqrestore(&g->lock, flags);
		return;
	}
	xg_window(g, &lo, &hi);
	g->step = hi;
	g->down_req = g->levels[hi].staycount;
	g->delay_count = 0;
	spin_unlock_irqrestore(&g->lock, flags);
	xg_set_work_fn(&g->set_work);
}

void xg_dvfs_power_on(struct exynos9810_gpu *g)
{
	unsigned long flags;

	spin_lock_irqsave(&g->lock, flags);
	g->gpu_on = true;
	spin_unlock_irqrestore(&g->lock, flags);
	if (g->wq)
		cancel_delayed_work(&g->idle_work);
}

void xg_dvfs_power_off(struct exynos9810_gpu *g)
{
	unsigned long flags;

	spin_lock_irqsave(&g->lock, flags);
	g->gpu_on = false;
	spin_unlock_irqrestore(&g->lock, flags);
	if (g->wq && g->dvfs_enabled)
		mod_delayed_work(g->wq, &g->idle_work, msecs_to_jiffies(XG_IDLE_DROP_MS));
}

void xg_dvfs_hold(struct exynos9810_gpu *g, bool hold)
{
	unsigned long flags;

	spin_lock_irqsave(&g->lock, flags);
	g->hold = hold;
	if (!hold && READ_ONCE(g->cur_step) < g->nr_levels)
		g->step = READ_ONCE(g->cur_step);
	spin_unlock_irqrestore(&g->lock, flags);
	if (!hold)
		xg_kick(g);
}

unsigned int xg_dvfs_cur_khz(struct exynos9810_gpu *g)
{
	if (!xg_pd_is_on(g, NULL, NULL))
		return 0;
	return cal_dfs_get_rate(g->cal_id);
}

/* Thermal cooling device: state N = top N levels removed. */
static int xg_cdev_get_max(struct thermal_cooling_device *cdev, unsigned long *state)
{
	struct exynos9810_gpu *g = cdev->devdata;

	*state = g->nr_levels - 1;
	return 0;
}

static int xg_cdev_get_cur(struct thermal_cooling_device *cdev, unsigned long *state)
{
	struct exynos9810_gpu *g = cdev->devdata;

	*state = READ_ONCE(g->thermal_cap);
	return 0;
}

static int xg_cdev_set_cur(struct thermal_cooling_device *cdev, unsigned long state)
{
	struct exynos9810_gpu *g = cdev->devdata;
	unsigned long flags;

	if (state >= g->nr_levels)
		return -EINVAL;
	spin_lock_irqsave(&g->lock, flags);
	if (g->thermal_cap != state)
		dev_info(g->dev, XG_PFX "thermal cap %lu: max %u kHz\n", state,
			 g->levels[state].khz);
	g->thermal_cap = state;
	g->step = xg_clamp_step(g, g->step);
	spin_unlock_irqrestore(&g->lock, flags);
	xg_kick(g);
	return 0;
}

static const struct thermal_cooling_device_ops xg_cdev_ops = {
	.get_max_state = xg_cdev_get_max,
	.get_cur_state = xg_cdev_get_cur,
	.set_cur_state = xg_cdev_set_cur,
};

int xg_dvfs_late_init(struct exynos9810_gpu *g)
{
	struct device_node *np = g->dev->of_node;
	struct workqueue_struct *wq;
	u32 period = XG_DEFAULT_PERIOD_MS;

	if (!g->dvfs_enabled)
		return 0;

	INIT_WORK(&g->set_work, xg_set_work_fn);
	INIT_DELAYED_WORK(&g->idle_work, xg_idle_work_fn);
	wq = alloc_ordered_workqueue("mali_g3d_dvfs", WQ_HIGHPRI | WQ_MEM_RECLAIM);
	if (!wq)
		return -ENOMEM;
	/* kbase power callbacks may already run: publish after INIT_*. */
	smp_store_release(&g->wq, wq);

	of_property_read_u32(np, "samsung,dvfs-period-ms", &period);
	g->kbdev->pm.dvfs_period = clamp_t(u32, period, 10, 1000);

	if (of_property_present(np, "#cooling-cells")) {
		struct thermal_cooling_device *cdev;

		cdev = thermal_of_cooling_device_register(np, "thermal-gpufreq-0", g,
							  &xg_cdev_ops);
		if (IS_ERR(cdev))
			dev_warn(g->dev, XG_PFX "cooling device not registered: %ld\n",
				 PTR_ERR(cdev));
		else
			g->cdev = cdev;
	}
	dev_info(g->dev, XG_PFX "DVFS %u-%u kHz, period %u ms, cooling %s\n",
		 g->levels[g->nr_levels - 1].khz, g->levels[0].khz,
		 g->kbdev->pm.dvfs_period, g->cdev ? "thermal-gpufreq-0" : "none");
	return 0;
}

void xg_dvfs_term(struct exynos9810_gpu *g)
{
	unsigned long flags;

	if (g->cdev) {
		thermal_cooling_device_unregister(g->cdev);
		g->cdev = NULL;
	}
	if (g->wq) {
		spin_lock_irqsave(&g->lock, flags);
		g->hold = true;
		spin_unlock_irqrestore(&g->lock, flags);
		cancel_delayed_work_sync(&g->idle_work);
		cancel_work_sync(&g->set_work);
		destroy_workqueue(g->wq);
		g->wq = NULL;
	}
	if (g->mif) {
		if (dev_pm_qos_request_active(&g->mif_req))
			dev_pm_qos_remove_request(&g->mif_req);
		put_device(&g->mif->dev);
		g->mif = NULL;
	}
}

/* /sys/kernel/gpu: the 4.9 gpex nodes the vendor power HAL uses. */
#define XG_ATTR_RO(_name) static struct kobj_attribute xg_attr_##_name = __ATTR_RO(_name)
#define XG_ATTR_RW(_name) static struct kobj_attribute xg_attr_##_name = __ATTR_RW(_name)

static ssize_t gpu_clock_show(struct kobject *k, struct kobj_attribute *a, char *buf)
{
	struct exynos9810_gpu *g = exynos9810_gpu;

	return g ? sysfs_emit(buf, "%u\n", xg_dvfs_cur_khz(g)) : -ENODEV;
}
XG_ATTR_RO(gpu_clock);

static ssize_t gpu_freq_table_show(struct kobject *k, struct kobj_attribute *a, char *buf)
{
	struct exynos9810_gpu *g = exynos9810_gpu;
	unsigned int i;
	int len = 0;

	if (!g)
		return -ENODEV;
	for (i = 0; i < g->nr_levels; i++)
		len += sysfs_emit_at(buf, len, "%u ", g->levels[i].khz);
	len += sysfs_emit_at(buf, len, "\n");
	return len;
}
XG_ATTR_RO(gpu_freq_table);

static ssize_t gpu_max_clock_show(struct kobject *k, struct kobj_attribute *a, char *buf)
{
	struct exynos9810_gpu *g = exynos9810_gpu;
	unsigned int lo, hi;
	unsigned long flags;

	if (!g || !g->dvfs_enabled)
		return -ENODEV;
	spin_lock_irqsave(&g->lock, flags);
	xg_window(g, &lo, &hi);
	spin_unlock_irqrestore(&g->lock, flags);
	return sysfs_emit(buf, "%u\n", g->levels[lo].khz);
}

/* Highest level <= value; 0 or >= top releases the limit. */
static ssize_t gpu_max_clock_store(struct kobject *k, struct kobj_attribute *a,
				   const char *buf, size_t count)
{
	struct exynos9810_gpu *g = exynos9810_gpu;
	unsigned int khz, i, step = 0;
	unsigned long flags;

	if (!g || !g->dvfs_enabled)
		return -ENODEV;
	if (kstrtouint(buf, 0, &khz))
		return -EINVAL;
	if (khz) {
		step = g->nr_levels - 1;
		for (i = 0; i < g->nr_levels; i++)
			if (g->levels[i].khz <= khz) {
				step = i;
				break;
			}
	}
	spin_lock_irqsave(&g->lock, flags);
	g->user_max_step = step;
	g->step = xg_clamp_step(g, g->step);
	spin_unlock_irqrestore(&g->lock, flags);
	xg_kick(g);
	return count;
}
XG_ATTR_RW(gpu_max_clock);

static ssize_t gpu_min_clock_show(struct kobject *k, struct kobj_attribute *a, char *buf)
{
	struct exynos9810_gpu *g = exynos9810_gpu;
	unsigned int lo, hi;
	unsigned long flags;

	if (!g || !g->dvfs_enabled)
		return -ENODEV;
	spin_lock_irqsave(&g->lock, flags);
	xg_window(g, &lo, &hi);
	spin_unlock_irqrestore(&g->lock, flags);
	return sysfs_emit(buf, "%u\n", g->levels[hi].khz);
}

/* Lowest level >= value; 0 releases the limit. */
static ssize_t gpu_min_clock_store(struct kobject *k, struct kobj_attribute *a,
				   const char *buf, size_t count)
{
	struct exynos9810_gpu *g = exynos9810_gpu;
	unsigned int khz, i, step;
	unsigned long flags;

	if (!g || !g->dvfs_enabled)
		return -ENODEV;
	if (kstrtouint(buf, 0, &khz))
		return -EINVAL;
	step = g->nr_levels - 1;
	if (khz) {
		step = 0;
		for (i = g->nr_levels; i-- > 0;)
			if (g->levels[i].khz >= khz) {
				step = i;
				break;
			}
	}
	spin_lock_irqsave(&g->lock, flags);
	g->user_min_step = step;
	g->step = xg_clamp_step(g, g->step);
	spin_unlock_irqrestore(&g->lock, flags);
	xg_kick(g);
	return count;
}
XG_ATTR_RW(gpu_min_clock);

static ssize_t gpu_governor_show(struct kobject *k, struct kobj_attribute *a, char *buf)
{
	return sysfs_emit(buf, "Interactive\n");
}
XG_ATTR_RO(gpu_governor);

static ssize_t gpu_busy_show(struct kobject *k, struct kobj_attribute *a, char *buf)
{
	struct exynos9810_gpu *g = exynos9810_gpu;

	return g ? sysfs_emit(buf, "%u\n", READ_ONCE(g->last_util)) : -ENODEV;
}
XG_ATTR_RO(gpu_busy);

static ssize_t gpu_model_show(struct kobject *k, struct kobj_attribute *a, char *buf)
{
	struct exynos9810_gpu *g = exynos9810_gpu;

	if (!g)
		return -ENODEV;
	return sysfs_emit(buf, "Mali-G72 MP%lu\n",
			  hweight64(g->kbdev->gpu_props.props.raw_props.shader_present));
}
XG_ATTR_RO(gpu_model);

static ssize_t gpu_dvfs_debug_show(struct kobject *k, struct kobj_attribute *a, char *buf)
{
	struct exynos9810_gpu *g = exynos9810_gpu;

	if (!g)
		return -ENODEV;
	return sysfs_emit(buf, "step %u cur %u thermal_cap %u user %u-%u sets %lu errors %lu power_on %lu power_off %lu mif %s\n",
			  READ_ONCE(g->step), READ_ONCE(g->cur_step), READ_ONCE(g->thermal_cap),
			  READ_ONCE(g->user_max_step), READ_ONCE(g->user_min_step),
			  g->nr_sets, g->nr_set_errors, g->nr_power_on, g->nr_power_off,
			  g->mif ? "yes" : "no");
}
XG_ATTR_RO(gpu_dvfs_debug);

static struct attribute *xg_attrs[] = {
	&xg_attr_gpu_clock.attr,
	&xg_attr_gpu_freq_table.attr,
	&xg_attr_gpu_max_clock.attr,
	&xg_attr_gpu_min_clock.attr,
	&xg_attr_gpu_governor.attr,
	&xg_attr_gpu_busy.attr,
	&xg_attr_gpu_model.attr,
	&xg_attr_gpu_dvfs_debug.attr,
	NULL,
};

static const struct attribute_group xg_attr_group = {
	.attrs = xg_attrs,
};

int xg_sysfs_init(struct exynos9810_gpu *g)
{
	int ret;

	g->kobj = kobject_create_and_add("gpu", kernel_kobj);
	if (!g->kobj)
		return -ENOMEM;
	ret = sysfs_create_group(g->kobj, &xg_attr_group);
	if (ret) {
		kobject_put(g->kobj);
		g->kobj = NULL;
	}
	return ret;
}

void xg_sysfs_term(struct exynos9810_gpu *g)
{
	if (!g->kobj)
		return;
	sysfs_remove_group(g->kobj, &xg_attr_group);
	kobject_put(g->kobj);
	g->kobj = NULL;
}
