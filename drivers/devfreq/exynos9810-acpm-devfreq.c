// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos9810 MIF/INT bus DVFS over CAL/ACPM (crownlte).
 *
 * Lean replacement for the 4.9 exynos-devfreq MIF/INT pair
 * (drivers/devfreq/exynos/exynos-devfreq.c, exynos9810_bus_{mif,int}.c).
 * What 4.9 did and is kept here:
 *  - OPPs are the ECT DVFS levels of dvfs_mif/dvfs_int, limited to the DT
 *    freq_info min/max and to CAL's min/max; rates go to cal_dfs_set_rate(),
 *    the firmware (ACPM channel 0) picks the voltage; current rate is the CMU
 *    readback cal_dfs_get_rate().
 *  - Policy is pure QoS: the rate is the highest minimum request, capped by
 *    the lowest maximum request (4.9 "interactive" governor without
 *    CONFIG_EXYNOS_WD_DVFS). Floors: default_qos, a boot floor for N seconds,
 *    the ACPM channel 5 MIF request, and consumers (CPU, GPU, display...)
 *    through dev_pm_qos DEV_PM_QOS_MIN_FREQUENCY on the platform device.
 *    MIF decreases wait delay_time_list (20 ms) after the last change.
 *  - MIF->INT minimum lock from the ECT MINLOCK dvfs_mif table (4.9 DVFS
 *    manager guidance constraint); INT is raised before MIF goes up and
 *    lowered after MIF goes down.
 *  - CPU->MIF ECT skew (the 4.9 DVFS Manager's "guidance" constraint) as a
 *    hard floor per CPU cluster (exynos9810_mif_hard_floor(), set by the
 *    Exynos9810 cpufreq driver): MIF is never programmed below it, whatever
 *    the governor or a MAX QoS request asks, only below the OPPs available.
 *  - The MIF level/constraint table is sent to the firmware FVP on ACPM
 *    channel 1 (DATA_INIT/SET_CONST), sleep entry/exit are announced with
 *    DATA_INIT/RELEASE, suspend runs at CAL's resume rate, reboot at
 *    freq_info's reboot rate.
 * Not ported: BTS MIF hook (4.9 qmax table is a single constant entry, so
 * BS_MIF_CHANGE never changes a register after init), PPMU load tracking
 * (use_get_dev = false on crownlte), TMU cold voltage offset (no regulator).
 *
 * Fail-safe: nothing is sent to the firmware and no rate changes until CAL
 * is ready, the ECT/CAL tables validate and the CMU readback matches a table
 * level. Any failure leaves the bootloader rates untouched.
 */
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_opp.h>
#include <linux/pm_qos.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <soc/samsung/acpm_ipc_ctrl.h>
#include <soc/samsung/cal-if.h>
#include <soc/samsung/ect_parser.h>
#include <soc/samsung/exynos9810-devfreq.h>
#include "governor.h"

#define XDF_GOV_NAME		"exynos9810-qos"
#define XDF_MAX_LEVELS		48

/* 4.9 include/soc/samsung/exynos-devfreq.h, FVP protocol on channel 1 */
#define FVP_DATA_INIT		5
#define FVP_SET_CONST		1
#define FVP_RELEASE		2

/* CAL ids of dvfs_mif / dvfs_int (4.9 dfs_id 184811520 / 184811521) */
#define XDF_MIF_CAL_ID		0x0b040000
#define XDF_INT_CAL_ID		0x0b040001

/* 4.9 enum exynos_devfreq_type, also the FVP domain id */
enum { XDF_MIF, XDF_INT, XDF_NR };
static const char * const xdf_names[XDF_NR] = { "mif", "int" };

struct xdf_level {
	unsigned int khz;
	unsigned int uv;
	unsigned int dep_khz;	/* MIF only: ECT minimum INT rate */
};

struct xdf_bus {
	struct device *dev;
	struct devfreq *df;
	struct devfreq_dev_profile profile;
	unsigned int type;
	unsigned int cal_id;
	const char *ect_name;
	unsigned int fvp_ch;
	unsigned int nr_levels;		/* descending */
	struct xdf_level levels[XDF_MAX_LEVELS];
	unsigned int min_khz, max_khz;
	unsigned int default_khz, suspend_khz, reboot_khz;
	unsigned int boot_khz, boot_secs, down_delay_ms;

	/* Governor inputs, written and read under df->lock. */
	unsigned long cur_hz;
	unsigned long changed_at;	/* jiffies of the last rate change */
	unsigned long dep_floor_hz;	/* INT: floor imposed by MIF */
	unsigned long forced_hz;	/* suspend / reboot rate */
	/* MIF: CPU cluster ECT skew floors (exynos9810_mif_hard_floor()) */
	unsigned long hard_floor_hz[EXYNOS9810_MIF_HARD_SLOTS];
	u32 hard_floor_raises;		/* targets the hard floor lifted */

	struct xdf_bus *dep;		/* MIF: the INT bus */
	struct delayed_work hold_work;
	struct delayed_work boot_work;
	struct work_struct noti_work;
	struct notifier_block reboot_nb;
	struct dev_pm_qos_request default_req, boot_req, acpm_req;
	unsigned int acpm_khz;
	bool acpm_notify;
	bool ready;
};

static struct platform_driver xdf_driver;
static struct xdf_bus *xdf_buses[XDF_NR];
static int xdf_probe_ret[XDF_NR] = { 1, 1 };	/* 1 = never probed */
static const char *xdf_probe_why[XDF_NR];
static bool xdf_fvp_uploaded;

static struct xdf_bus *xdf_ready(unsigned int type)
{
	struct xdf_bus *bus = READ_ONCE(xdf_buses[type]);

	return bus && smp_load_acquire(&bus->ready) ? bus : NULL;
}

static const struct xdf_level *xdf_find(const struct xdf_bus *bus,
					unsigned int khz)
{
	unsigned int i;

	for (i = 0; i < bus->nr_levels; i++)
		if (bus->levels[i].khz == khz)
			return &bus->levels[i];
	return NULL;
}

static struct ect_minlock_domain *xdf_minlock(const char *name)
{
	void *block = ect_get_block(BLOCK_MINLOCK);

	return block ? ect_minlock_get_domain(block, (char *)name) : NULL;
}

static int xdf_minlock_sub(const struct ect_minlock_domain *ml,
			   unsigned int main_khz, unsigned int *sub_khz)
{
	int i;

	for (i = 0; i < ml->num_of_level; i++) {
		if (ml->level[i].main_frequencies == main_khz) {
			*sub_khz = ml->level[i].sub_frequencies;
			return 0;
		}
	}
	return -ENOENT;
}

static struct ect_dvfs_domain *xdf_ect_dvfs(const char *name)
{
	void *block = ect_get_block(BLOCK_DVFS);
	struct ect_dvfs_domain *dom;
	int i;

	dom = block ? ect_dvfs_get_domain(block, (char *)name) : NULL;
	if (!dom || !dom->list_level || dom->num_of_level <= 0 ||
	    dom->num_of_level > XDF_MAX_LEVELS)
		return NULL;
	for (i = 0; i < dom->num_of_level; i++)
		if (!dom->list_level[i].level ||
		    (i && dom->list_level[i].level >= dom->list_level[i - 1].level))
			return NULL;
	return dom;
}

static int xdf_fvp_send(unsigned int ch, u32 c0, u32 c1, u32 c2, u32 c3)
{
	unsigned int cmd[4] = { c0, c1, c2, c3 };
	struct ipc_config cfg = { .cmd = cmd, .response = true };

	return acpm_ipc_send_data(ch, &cfg);
}

/* ------------------------------------------------------------------ */
/* Rate changes                                                        */

/*
 * Make INT follow the ECT MIF->INT minimum lock for @mif_hz. Called with the
 * MIF devfreq lock held; nests the INT lock (order is always MIF -> INT).
 */
static int xdf_set_dep(struct xdf_bus *mif, unsigned long mif_hz)
{
	struct xdf_bus *dep = mif->dep;
	const struct xdf_level *lvl = xdf_find(mif, mif_hz / 1000);
	unsigned long floor_hz, now_hz;
	int ret;

	if (!dep)
		return 0;
	if (!lvl)
		return -EINVAL;
	floor_hz = (unsigned long)lvl->dep_khz * 1000;
	mutex_lock_nested(&dep->df->lock, SINGLE_DEPTH_NESTING);
	dep->dep_floor_hz = floor_hz;
	ret = update_devfreq(dep->df);
	now_hz = dep->cur_hz;
	mutex_unlock(&dep->df->lock);
	if (ret)
		dev_err_ratelimited(mif->dev, "INT floor %lu kHz failed: %d\n",
				    floor_hz / 1000, ret);
	else if (now_hz < floor_hz)
		dev_warn_ratelimited(mif->dev,
				     "INT capped at %lu kHz below MIF lock %lu kHz\n",
				     now_hz / 1000, floor_hz / 1000);
	return ret;
}

static int xdf_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct xdf_bus *bus = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;
	unsigned long hz, old, floor;
	int i, ret;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	hz = dev_pm_opp_get_freq(opp);
	dev_pm_opp_put(opp);

	/*
	 * CPU skew floors win over the governor and over MAX QoS requests:
	 * the lowest available OPP at or above the highest one, or the top
	 * available OPP if none reaches it (the cpufreq side checks the rate
	 * reached and runs a lower CPU level then).
	 */
	floor = 0;
	for (i = 0; i < EXYNOS9810_MIF_HARD_SLOTS; i++)
		floor = max(floor, bus->hard_floor_hz[i]);
	if (hz < floor) {
		unsigned long f = floor;

		opp = dev_pm_opp_find_freq_ceil(dev, &f);
		if (IS_ERR(opp)) {
			f = ULONG_MAX;
			opp = dev_pm_opp_find_freq_floor(dev, &f);
		}
		if (!IS_ERR(opp)) {
			f = dev_pm_opp_get_freq(opp);
			dev_pm_opp_put(opp);
			if (f > hz) {
				hz = f;
				bus->hard_floor_raises++;
			}
		}
	}

	old = bus->cur_hz;
	*freq = old;
	if (hz == old)
		return 0;

	if (hz > old) {
		ret = xdf_set_dep(bus, hz);
		if (ret)
			return ret;
	}
	ret = cal_dfs_set_rate(bus->cal_id, hz / 1000);
	if (ret) {
		dev_err_ratelimited(dev, "CAL %lu -> %lu kHz failed: %d\n",
				    old / 1000, hz / 1000, ret);
		return ret;
	}
	bus->cur_hz = hz;
	bus->changed_at = jiffies;
	*freq = hz;
	if (hz < old)
		xdf_set_dep(bus, hz);
	return 0;
}

static int xdf_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct xdf_bus *bus = dev_get_drvdata(dev);
	unsigned long khz = cal_dfs_get_rate(bus->cal_id);

	if (!khz) {
		*freq = bus->cur_hz;
		return -EIO;
	}
	*freq = khz * 1000;
	return 0;
}

/* ------------------------------------------------------------------ */
/* Governor: 4.9 QoS-only "interactive" with delayed decreases          */

static struct xdf_bus *xdf_df_bus(struct devfreq *df)
{
	struct device *parent = df->dev.parent;

	/* The governor is selectable by any devfreq device; act only on ours. */
	if (!parent || parent->driver != &xdf_driver.driver)
		return NULL;
	return dev_get_drvdata(parent);
}

/**
 * exynos9810_mif_hard_floor - set a CPU cluster's hard MIF floor
 * @df: the MIF devfreq (samsung,mif-devfreq of the cpufreq node)
 * @slot: the CPU cluster, below EXYNOS9810_MIF_HARD_SLOTS
 * @khz: the ECT skew of the level the cluster runs or is about to run; 0 none
 * @now_khz: if not NULL, the MIF rate afterwards (CMU readback, else the
 *	last rate set)
 *
 * Applied synchronously under the MIF devfreq lock: MIF is updated (also
 * when the floor did not change, which retries a failed MIF/INT FREQ_REQ)
 * before this returns, and xdf_target() never programs MIF below the floor
 * whatever the governor or a MAX QoS request asks. A failed MIF update is
 * logged here; the caller judges @now_khz.
 *
 * Return: 0, -EPROBE_DEFER if MIF is not up yet, -ENODEV if @df is not the
 * Exynos9810 MIF devfreq, -EINVAL for a bad slot.
 */
int exynos9810_mif_hard_floor(struct devfreq *df, unsigned int slot,
			      unsigned long khz, unsigned long *now_khz)
{
	struct xdf_bus *bus = df ? xdf_df_bus(df) : NULL;
	unsigned long hz = 0;
	int ret;

	if (!bus || bus->type != XDF_MIF)
		return -ENODEV;
	/* pairs with the smp_store_release() in xdf_probe() */
	if (!smp_load_acquire(&bus->ready))
		return -EPROBE_DEFER;
	if (slot >= EXYNOS9810_MIF_HARD_SLOTS)
		return -EINVAL;
	mutex_lock(&df->lock);
	bus->hard_floor_hz[slot] = khz * 1000;
	ret = update_devfreq(df);
	if (now_khz) {
		if (df->profile->get_cur_freq(df->dev.parent, &hz) || !hz)
			hz = df->previous_freq;
		*now_khz = hz / 1000;
	}
	mutex_unlock(&df->lock);
	if (ret)
		dev_warn_ratelimited(bus->dev, "MIF update with CPU%u hard floor %lu kHz failed: %d\n",
				     slot, khz, ret);
	return 0;
}
EXPORT_SYMBOL_GPL(exynos9810_mif_hard_floor);

static int xdf_gov_get_target_freq(struct devfreq *df, unsigned long *freq)
{
	struct xdf_bus *bus = xdf_df_bus(df);
	unsigned long min, max, want, prev = df->previous_freq;

	devfreq_get_freq_range(df, &min, &max);
	if (!bus) {
		*freq = min;
		return 0;
	}
	want = max(min, bus->dep_floor_hz);
	if (bus->forced_hz) {
		*freq = max(bus->forced_hz, bus->dep_floor_hz);
		return 0;
	}
	if (bus->down_delay_ms && want < prev && prev <= max) {
		unsigned long until = bus->changed_at +
				      msecs_to_jiffies(bus->down_delay_ms);

		if (time_before(jiffies, until)) {
			mod_delayed_work(system_wq, &bus->hold_work,
					 until - jiffies);
			*freq = prev;
			return 0;
		}
	}
	*freq = want;
	return 0;
}

static int xdf_gov_event(struct devfreq *df, unsigned int event, void *data)
{
	struct xdf_bus *bus = xdf_df_bus(df);
	int ret = 0;

	switch (event) {
	case DEVFREQ_GOV_START:
		if (bus)
			WRITE_ONCE(bus->df, df);
		fallthrough;
	case DEVFREQ_GOV_RESUME:
		mutex_lock(&df->lock);
		ret = update_devfreq(df);
		mutex_unlock(&df->lock);
		break;
	case DEVFREQ_GOV_STOP:
	case DEVFREQ_GOV_SUSPEND:
		if (bus)
			cancel_delayed_work_sync(&bus->hold_work);
		break;
	}
	return ret;
}

static struct devfreq_governor xdf_governor = {
	.name = XDF_GOV_NAME,
	.get_target_freq = xdf_gov_get_target_freq,
	.event_handler = xdf_gov_event,
};

static void xdf_hold_work(struct work_struct *work)
{
	struct xdf_bus *bus = container_of(to_delayed_work(work),
					   struct xdf_bus, hold_work);
	struct devfreq *df = READ_ONCE(bus->df);

	if (!df)
		return;
	mutex_lock(&df->lock);
	update_devfreq(df);
	mutex_unlock(&df->lock);
}

static void xdf_boot_work(struct work_struct *work)
{
	struct xdf_bus *bus = container_of(to_delayed_work(work),
					   struct xdf_bus, boot_work);

	dev_pm_qos_update_request(&bus->boot_req,
				  PM_QOS_MIN_FREQUENCY_DEFAULT_VALUE);
}

/* ------------------------------------------------------------------ */
/* ACPM channel 5 firmware MIF floor (4.9 acpm_noti_mif_callback)       */

static void xdf_noti_work(struct work_struct *work)
{
	struct xdf_bus *bus = container_of(work, struct xdf_bus, noti_work);
	unsigned int khz = READ_ONCE(bus->acpm_khz);

	dev_pm_qos_update_request(&bus->acpm_req, min_t(unsigned int, khz, S32_MAX));
}

/* Atomic context (ACPM IRQ thread, maybe under its rx spinlock). */
static void xdf_acpm_mif_notify(void *priv, unsigned int khz)
{
	struct xdf_bus *bus = priv;

	WRITE_ONCE(bus->acpm_khz, khz);
	schedule_work(&bus->noti_work);
}

/* ------------------------------------------------------------------ */
/* Suspend / reboot (4.9 exynos_devfreq_suspend/resume, reboot notifier) */

static int xdf_force(struct xdf_bus *bus, unsigned long hz)
{
	int ret;

	mutex_lock(&bus->df->lock);
	bus->forced_hz = hz;
	ret = update_devfreq(bus->df);
	mutex_unlock(&bus->df->lock);
	return ret;
}

static int xdf_suspend(struct device *dev)
{
	struct xdf_bus *bus = dev_get_drvdata(dev);
	int ret;

	if (!bus || !smp_load_acquire(&bus->ready))
		return 0;
	/* Tell FVP the AP is going to sleep, as 4.9 did for MIF and INT. */
	ret = xdf_fvp_send(bus->fvp_ch, bus->type, 0, FVP_DATA_INIT, FVP_RELEASE);
	if (ret) {
		dev_err(dev, "FVP sleep flag failed: %d\n", ret);
		return ret;
	}
	ret = xdf_force(bus, (unsigned long)bus->suspend_khz * 1000);
	if (ret) {
		dev_err(dev, "suspend rate %u kHz failed: %d\n",
			bus->suspend_khz, ret);
		xdf_force(bus, 0);
		xdf_fvp_send(bus->fvp_ch, bus->type, 1, FVP_DATA_INIT, FVP_RELEASE);
	}
	return ret;
}

static int xdf_resume(struct device *dev)
{
	struct xdf_bus *bus = dev_get_drvdata(dev);
	int ret;

	if (!bus || !smp_load_acquire(&bus->ready))
		return 0;
	ret = xdf_fvp_send(bus->fvp_ch, bus->type, 1, FVP_DATA_INIT, FVP_RELEASE);
	if (ret)
		dev_err(dev, "FVP wake flag failed: %d\n", ret);
	xdf_force(bus, 0);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(xdf_pm_ops, xdf_suspend, xdf_resume);

static int xdf_reboot_notify(struct notifier_block *nb, unsigned long action,
			     void *data)
{
	struct xdf_bus *bus = container_of(nb, struct xdf_bus, reboot_nb);
	int ret = xdf_force(bus, (unsigned long)bus->reboot_khz * 1000);

	if (ret)
		dev_err(bus->dev, "reboot rate %u kHz failed: %d\n",
			bus->reboot_khz, ret);
	return NOTIFY_DONE;
}

/* ------------------------------------------------------------------ */
/* Probe                                                               */

static int xdf_build_table(struct xdf_bus *bus, u32 dt_min, u32 dt_max,
			   const char **why)
{
	struct dvfs_rate_volt cal[XDF_MAX_LEVELS];
	struct ect_dvfs_domain *dom;
	unsigned long cal_min, cal_max;
	int n, i, j;

	cal_min = cal_dfs_get_min_freq(bus->cal_id);
	cal_max = cal_dfs_get_max_freq(bus->cal_id);
	if (!cal_min || cal_min > cal_max || cal_max > INT_MAX) {
		*why = "CAL min/max invalid";
		return -ERANGE;
	}
	bus->min_khz = max_t(unsigned long, dt_min, cal_min);
	bus->max_khz = min_t(unsigned long, dt_max, cal_max);
	if (bus->min_khz > bus->max_khz) {
		*why = "DT and CAL ranges do not overlap";
		return -ERANGE;
	}
	n = cal_dfs_get_rate_asv_table(bus->cal_id, cal);
	if (n <= 0 || n > XDF_MAX_LEVELS) {
		*why = "CAL rate/voltage table unavailable";
		return -ENODATA;
	}
	dom = xdf_ect_dvfs(bus->ect_name);
	if (!dom) {
		*why = "ECT DVFS domain missing or not descending";
		return -ENODATA;
	}
	bus->nr_levels = 0;
	for (i = 0; i < dom->num_of_level; i++) {
		unsigned int khz = dom->list_level[i].level;

		if (khz < bus->min_khz || khz > bus->max_khz)
			continue;
		for (j = 0; j < n; j++)
			if (cal[j].rate == khz)
				break;
		if (j == n || !cal[j].volt) {
			*why = "ECT level without CAL rate/voltage";
			return -EINVAL;
		}
		bus->levels[bus->nr_levels].khz = khz;
		bus->levels[bus->nr_levels].uv = cal[j].volt;
		bus->nr_levels++;
	}
	if (!bus->nr_levels) {
		*why = "no level inside min/max";
		return -ERANGE;
	}
	return 0;
}

/*
 * 4.9 exynos9810_mif_constraint_parse(): every ECT MIF level inside CAL's
 * min/max as <index, kHz, DATA_INIT, ECT INT minimum>, then <count, CAL boot
 * kHz, DATA_INIT, SET_CONST>. Validated in full before the first send.
 */
static int xdf_fvp_upload(struct xdf_bus *bus, const struct ect_minlock_domain *ml,
			  const char **why)
{
	struct ect_dvfs_domain *dom = xdf_ect_dvfs(bus->ect_name);
	unsigned long cal_min = cal_dfs_get_min_freq(bus->cal_id);
	unsigned long cal_max = cal_dfs_get_max_freq(bus->cal_id);
	unsigned int boot = cal_dfs_get_boot_freq(bus->cal_id);
	unsigned int sub, idx;
	int i, ret;

	if (xdf_fvp_uploaded)
		return 0;
	if (!dom || !boot) {
		*why = "FVP table source missing";
		return -ENODATA;
	}
	for (i = 0; i < dom->num_of_level; i++) {
		unsigned int khz = dom->list_level[i].level;

		if (khz >= cal_min && khz <= cal_max &&
		    xdf_minlock_sub(ml, khz, &sub)) {
			*why = "MIF level missing from ECT MINLOCK";
			return -ENODATA;
		}
	}
	for (i = 0, idx = 0; i < dom->num_of_level; i++) {
		unsigned int khz = dom->list_level[i].level;

		if (khz < cal_min || khz > cal_max)
			continue;
		xdf_minlock_sub(ml, khz, &sub);
		ret = xdf_fvp_send(bus->fvp_ch, idx, khz, FVP_DATA_INIT, sub);
		if (ret) {
			*why = "FVP level upload failed";
			return ret;
		}
		idx++;
	}
	ret = xdf_fvp_send(bus->fvp_ch, idx, boot, FVP_DATA_INIT, FVP_SET_CONST);
	if (ret) {
		*why = "FVP SET_CONST failed";
		return ret;
	}
	xdf_fvp_uploaded = true;
	dev_info(bus->dev, "FVP: %u MIF levels with INT locks, boot %u kHz\n",
		 idx, boot);
	return 0;
}

static void xdf_remove_opps(void *data)
{
	dev_pm_opp_remove_all_dynamic(data);
}

static void xdf_remove_qos(void *data)
{
	struct xdf_bus *bus = data;

	if (dev_pm_qos_request_active(&bus->acpm_req))
		dev_pm_qos_remove_request(&bus->acpm_req);
	if (dev_pm_qos_request_active(&bus->boot_req))
		dev_pm_qos_remove_request(&bus->boot_req);
	if (dev_pm_qos_request_active(&bus->default_req))
		dev_pm_qos_remove_request(&bus->default_req);
}

static void xdf_stop_async(void *data)
{
	struct xdf_bus *bus = data;

	if (bus->acpm_notify)
		exynos_acpm_dvfs_mif_notify_unregister();
	cancel_work_sync(&bus->noti_work);
	cancel_delayed_work_sync(&bus->boot_work);
}

static void xdf_unregister_reboot(void *data)
{
	struct xdf_bus *bus = data;

	unregister_reboot_notifier(&bus->reboot_nb);
}

static int xdf_get_dep(struct xdf_bus *bus, const char **why)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct xdf_bus *dep;

	np = of_parse_phandle(bus->dev->of_node, "samsung,dependent-devfreq", 0);
	if (!np)
		return 0;
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev) {
		*why = "dependent devfreq device missing";
		return -EPROBE_DEFER;
	}
	dep = platform_get_drvdata(pdev);
	put_device(&pdev->dev);	/* built-in, never unbound */
	if (!dep || dep->type != XDF_INT || !smp_load_acquire(&dep->ready)) {
		*why = "waiting for INT devfreq";
		return -EPROBE_DEFER;
	}
	bus->dep = dep;
	return 0;
}

static int xdf_probe_bus(struct xdf_bus *bus, const char **why)
{
	struct device *dev = bus->dev;
	struct device_node *np = dev->of_node;
	struct ect_minlock_domain *ml = NULL;
	struct devfreq *df;
	unsigned long cur_khz;
	u32 info[6], boot[2] = { 0, 0 }, size;
	unsigned int i;
	int ret;

	if (of_property_read_u32(np, "samsung,cal-id", &bus->cal_id) ||
	    of_property_read_string(np, "samsung,ect-domain", &bus->ect_name) ||
	    of_property_read_u32_array(np, "samsung,freq-info", info, 6)) {
		*why = "DT properties missing";
		return -EINVAL;
	}
	of_property_read_u32_array(np, "samsung,boot-qos", boot, 2);
	of_property_read_u32(np, "samsung,down-delay-ms", &bus->down_delay_ms);
	bus->acpm_notify = of_property_read_bool(np, "samsung,acpm-mif-notify");

	if (!cal_dfs_is_ready(bus->cal_id)) {
		*why = "CAL/ACPM DVFS not ready";
		return -EPROBE_DEFER;
	}
	ret = acpm_ipc_request_channel(np, NULL, &bus->fvp_ch, &size);
	if (ret) {
		*why = "ACPM FVP channel unavailable";
		return ret == -ENODEV ? -EPROBE_DEFER : ret;
	}
	if (size != 4 * sizeof(u32)) {
		*why = "ACPM FVP channel element size";
		return -EPROTO;
	}
	ret = xdf_get_dep(bus, why);
	if (ret)
		return ret;

	ret = xdf_build_table(bus, info[3], info[4], why);
	if (ret)
		return ret;
	if (bus->dep) {
		ml = xdf_minlock(bus->ect_name);
		if (!ml || !ml->level || ml->num_of_level <= 0) {
			*why = "ECT MINLOCK domain missing";
			return -ENODATA;
		}
		for (i = 0; i < bus->nr_levels; i++) {
			if (xdf_minlock_sub(ml, bus->levels[i].khz,
					    &bus->levels[i].dep_khz)) {
				*why = "MIF level missing from ECT MINLOCK";
				return -ENODATA;
			}
		}
	}

	bus->default_khz = clamp(info[1], bus->min_khz, bus->max_khz);
	bus->suspend_khz = cal_dfs_get_resume_freq(bus->cal_id);
	if (!bus->suspend_khz)
		bus->suspend_khz = info[2];
	bus->reboot_khz = info[5];
	bus->boot_secs = boot[0];
	bus->boot_khz = boot[1];

	cur_khz = cal_dfs_get_rate(bus->cal_id);
	if (!xdf_find(bus, cur_khz)) {
		dev_err(dev, "CMU readback %lu kHz is not a table level\n", cur_khz);
		*why = "CMU readback not a table level";
		return -ERANGE;
	}
	bus->cur_hz = cur_khz * 1000;
	bus->changed_at = jiffies;
	/* INT stays at its maximum until MIF takes over the minimum lock. */
	bus->dep_floor_hz = bus->type == XDF_INT ?
			    (unsigned long)bus->max_khz * 1000 : 0;

	for (i = 0; i < bus->nr_levels; i++) {
		ret = dev_pm_opp_add(dev, (unsigned long)bus->levels[i].khz * 1000,
				     bus->levels[i].uv);
		if (ret) {
			dev_pm_opp_remove_all_dynamic(dev);
			*why = "OPP add failed";
			return ret;
		}
	}
	ret = devm_add_action_or_reset(dev, xdf_remove_opps, dev);
	if (ret)
		return ret;

	if (bus->dep && of_property_read_bool(np, "samsung,fvp-constraint-upload")) {
		ret = xdf_fvp_upload(bus, ml, why);
		if (ret)
			return ret;
	}

	INIT_DELAYED_WORK(&bus->hold_work, xdf_hold_work);
	INIT_DELAYED_WORK(&bus->boot_work, xdf_boot_work);
	INIT_WORK(&bus->noti_work, xdf_noti_work);

	ret = dev_pm_qos_add_request(dev, &bus->default_req,
				     DEV_PM_QOS_MIN_FREQUENCY, bus->default_khz);
	if (ret >= 0 && bus->boot_secs)
		ret = dev_pm_qos_add_request(dev, &bus->boot_req,
					     DEV_PM_QOS_MIN_FREQUENCY, bus->boot_khz);
	if (ret >= 0 && bus->acpm_notify)
		ret = dev_pm_qos_add_request(dev, &bus->acpm_req,
					     DEV_PM_QOS_MIN_FREQUENCY, 0);
	if (ret < 0) {
		xdf_remove_qos(bus);
		*why = "PM QoS request failed";
		return ret;
	}
	ret = devm_add_action_or_reset(dev, xdf_remove_qos, bus);
	if (ret)
		return ret;

	bus->profile.initial_freq = bus->cur_hz;
	bus->profile.polling_ms = 0;
	bus->profile.target = xdf_target;
	bus->profile.get_cur_freq = xdf_get_cur_freq;
	df = devm_devfreq_add_device(dev, &bus->profile, XDF_GOV_NAME, NULL);
	if (IS_ERR(df)) {
		*why = "devfreq registration failed";
		return PTR_ERR(df);
	}
	WRITE_ONCE(bus->df, df);

	if (bus->dep) {
		mutex_lock(&df->lock);
		ret = xdf_set_dep(bus, bus->cur_hz);
		mutex_unlock(&df->lock);
		if (ret) {
			*why = "initial INT lock failed";
			return ret;
		}
	}

	ret = devm_add_action_or_reset(dev, xdf_stop_async, bus);
	if (ret)
		return ret;
	if (bus->boot_secs)
		schedule_delayed_work(&bus->boot_work, bus->boot_secs * HZ);
	if (bus->acpm_notify) {
		ret = exynos_acpm_dvfs_mif_notify_register(xdf_acpm_mif_notify, bus);
		if (ret) {
			bus->acpm_notify = false;
			*why = "ACPM MIF notification already owned";
			return ret;
		}
	}
	bus->reboot_nb.notifier_call = xdf_reboot_notify;
	ret = register_reboot_notifier(&bus->reboot_nb);
	if (!ret)
		ret = devm_add_action_or_reset(dev, xdf_unregister_reboot, bus);
	if (ret) {
		*why = "reboot notifier failed";
		return ret;
	}

	dev_info(dev, "%s: %u levels %u-%u kHz, now %lu kHz, floor %u kHz, boot %u kHz for %us%s\n",
		 bus->ect_name, bus->nr_levels, bus->min_khz, bus->max_khz,
		 df->previous_freq / 1000, bus->default_khz, bus->boot_khz,
		 bus->boot_secs, bus->dep ? ", INT follows ECT MINLOCK" : "");
	return 0;
}

static int xdf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *why = NULL;
	struct xdf_bus *bus;
	u32 type;
	int ret;

	if (of_property_read_u32(dev->of_node, "samsung,devfreq-type", &type) ||
	    type >= XDF_NR)
		return dev_err_probe(dev, -EINVAL, "bad samsung,devfreq-type\n");
	bus = devm_kzalloc(dev, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;
	bus->dev = dev;
	bus->type = type;
	platform_set_drvdata(pdev, bus);

	ret = xdf_probe_bus(bus, &why);
	WRITE_ONCE(xdf_probe_why[type], why);
	WRITE_ONCE(xdf_probe_ret[type], ret);
	if (ret)
		return dev_err_probe(dev, ret, "%s\n", why ? why : "probe failed");
	WRITE_ONCE(xdf_buses[type], bus);
	smp_store_release(&bus->ready, true);
	return 0;
}

static const struct of_device_id xdf_match[] = {
	{ .compatible = "samsung,exynos9810-acpm-devfreq" },
	{ }
};

static struct platform_driver xdf_driver = {
	.probe = xdf_probe,
	.driver = {
		.name = "exynos9810-acpm-devfreq",
		.of_match_table = xdf_match,
		.pm = pm_sleep_ptr(&xdf_pm_ops),
		.suppress_bind_attrs = true,
	},
};

/* ------------------------------------------------------------------ */
/* Diagnostic self-test (DT property samsung,devfreq-selftest)          */

#define XDF_ST_PFX	"devfreq selftest: "

static char xdf_st_buf[640];

static void xdf_st_dvfs(const char *name)
{
	struct ect_dvfs_domain *dom = xdf_ect_dvfs(name);
	int i, len = 0;

	if (!dom) {
		pr_info(XDF_ST_PFX "ect %s: missing/invalid\n", name);
		return;
	}
	for (i = 0; i < dom->num_of_level; i++)
		len += scnprintf(xdf_st_buf + len, sizeof(xdf_st_buf) - len, " %u",
				 dom->list_level[i].level);
	pr_info(XDF_ST_PFX "ect %s:%s\n", name, xdf_st_buf);
}

static void xdf_st_minlock(const char *name)
{
	struct ect_minlock_domain *ml = xdf_minlock(name);
	int i, len = 0;

	if (!ml || !ml->level || ml->num_of_level <= 0) {
		pr_info(XDF_ST_PFX "minlock %s: missing\n", name);
		return;
	}
	for (i = 0; i < ml->num_of_level; i++)
		len += scnprintf(xdf_st_buf + len, sizeof(xdf_st_buf) - len,
				 " %u>%u", ml->level[i].main_frequencies,
				 ml->level[i].sub_frequencies);
	pr_info(XDF_ST_PFX "minlock %s:%s\n", name, xdf_st_buf);
}

static void xdf_st_cal(const char *name, unsigned int id)
{
	struct dvfs_rate_volt cal[XDF_MAX_LEVELS];
	int n, i, len = 0;

	n = cal_dfs_is_ready(id) ? cal_dfs_get_rate_asv_table(id, cal) : 0;
	for (i = 0; i < n && i < XDF_MAX_LEVELS; i++)
		len += scnprintf(xdf_st_buf + len, sizeof(xdf_st_buf) - len,
				 " %lu/%u", cal[i].rate, cal[i].volt);
	xdf_st_buf[len] = 0;
	pr_info(XDF_ST_PFX "cal %s ready=%d min=%lu max=%lu boot=%u resume=%u cmu=%lu kHz table:%s\n",
		name, cal_dfs_is_ready(id), cal_dfs_get_min_freq(id),
		cal_dfs_get_max_freq(id), cal_dfs_get_boot_freq(id),
		cal_dfs_get_resume_freq(id), cal_dfs_get_rate(id),
		n > 0 ? xdf_st_buf : " none");
}

static void xdf_st_noti(const char *when)
{
	unsigned long count;
	unsigned int khz;
	u64 ns;

	exynos_acpm_dvfs_mif_notify_stats(&count, &khz, &ns);
	pr_info(XDF_ST_PFX "acpm mif notifications %s: count=%lu last=%u kHz at %llu ms\n",
		when, count, khz, ns / NSEC_PER_MSEC);
}

static unsigned long xdf_st_prev(struct xdf_bus *bus)
{
	unsigned long hz;

	mutex_lock(&bus->df->lock);
	hz = bus->df->previous_freq;
	mutex_unlock(&bus->df->lock);
	return hz;
}

/* Wait (bounded) for devfreq to settle on @want_khz, or any change if 0. */
static unsigned int xdf_st_wait(struct xdf_bus *bus, unsigned int want_khz,
				unsigned long from_hz)
{
	ktime_t t0 = ktime_get();
	unsigned long hz;

	do {
		hz = xdf_st_prev(bus);
		if (want_khz ? hz == (unsigned long)want_khz * 1000 : hz != from_hz)
			break;
		usleep_range(1000, 2000);
	} while (ktime_ms_delta(ktime_get(), t0) < 300);
	return (unsigned int)ktime_us_delta(ktime_get(), t0);
}

static int xdf_st_report(const char *step, int ret, unsigned int want_khz,
			 unsigned int t_us, unsigned int wait_us)
{
	struct xdf_bus *mif = xdf_ready(XDF_MIF), *intb = xdf_ready(XDF_INT);
	unsigned long mdf = mif ? xdf_st_prev(mif) / 1000 : 0;
	unsigned long idf = intb ? xdf_st_prev(intb) / 1000 : 0;
	unsigned long mcmu = cal_dfs_get_rate(XDF_MIF_CAL_ID);
	unsigned long icmu = cal_dfs_get_rate(XDF_INT_CAL_ID);
	bool bad = ret < 0 || (mif && mdf != mcmu) || (intb && idf != icmu);

	pr_info(XDF_ST_PFX "%s ret=%d want=%u mif=%lu/cmu %lu int=%lu/cmu %lu kHz qos %u us settle %u us%s\n",
		step, ret, want_khz, mdf, mcmu, idf, icmu, t_us, wait_us,
		bad ? " MISMATCH" : "");
	return bad;
}

/*
 * Lowest level the self-test may cap a bus to: never below 4.9's awake
 * floor (default_qos; INT's absolute 107 MHz was suspend-only on 4.9 and
 * is not forced while the diagnostic USB log runs), below any consumer's
 * current PM QoS floor (CPU, firmware, ...), or below the MIF->INT lock.
 * Rounded up to a table level.
 */
static unsigned int xdf_low_target(struct xdf_bus *bus)
{
	unsigned long floor = bus->default_khz;
	unsigned int i = bus->nr_levels;
	s32 qos = dev_pm_qos_read_value(bus->dev, DEV_PM_QOS_MIN_FREQUENCY);

	if (qos > 0)
		floor = max(floor, (unsigned long)qos);
	mutex_lock(&bus->df->lock);
	floor = max(floor, bus->dep_floor_hz / 1000);
	mutex_unlock(&bus->df->lock);
	while (i-- > 1)
		if (bus->levels[i].khz >= floor)
			break;
	return bus->levels[i].khz;
}

static void xdf_selftest_fn(struct work_struct *work)
{
	struct xdf_bus *mif = xdf_ready(XDF_MIF), *intb = xdf_ready(XDF_INT);
	struct dev_pm_qos_request mmin = {}, mmax = {}, imin = {}, imax = {};
	ktime_t start = ktime_get(), t;
	unsigned int us, wait, want;
	unsigned long from;
	int i, ret, bad = 0;

	for (i = 0; i < XDF_NR; i++) {
		int pr = READ_ONCE(xdf_probe_ret[i]);
		const char *why = READ_ONCE(xdf_probe_why[i]);

		if (pr == 1)
			pr_info(XDF_ST_PFX "%s: never probed\n", xdf_names[i]);
		else
			pr_info(XDF_ST_PFX "%s: probe %d%s%s\n", xdf_names[i], pr,
				why ? " " : "", why ? why : "");
	}
	pr_info(XDF_ST_PFX "fvp upload %s\n", xdf_fvp_uploaded ? "sent" : "not sent");
	xdf_st_dvfs("dvfs_mif");
	xdf_st_dvfs("dvfs_int");
	xdf_st_cal("mif", XDF_MIF_CAL_ID);
	xdf_st_cal("int", XDF_INT_CAL_ID);
	xdf_st_minlock("dvfs_mif");
	xdf_st_minlock("dvfs_int");
	xdf_st_minlock("dvfs_cpucl0");
	xdf_st_minlock("dvfs_cpucl1");
	xdf_st_noti("before");

	for (i = 0; i < XDF_NR; i++) {
		struct xdf_bus *bus = xdf_ready(i);

		if (!bus)
			continue;
		pr_info(XDF_ST_PFX "%s now %lu kHz qos min %d max %d kHz floors default %u boot %u%s acpm %u dep %lu\n",
			xdf_names[i], xdf_st_prev(bus) / 1000,
			dev_pm_qos_read_value(bus->dev, DEV_PM_QOS_MIN_FREQUENCY),
			dev_pm_qos_read_value(bus->dev, DEV_PM_QOS_MAX_FREQUENCY),
			bus->default_khz, bus->boot_khz,
			delayed_work_pending(&bus->boot_work) ? "" : "(expired)",
			READ_ONCE(bus->acpm_khz), bus->dep_floor_hz / 1000);
	}
	bad += xdf_st_report("start", 0, 0, 0, 0);

	if (mif) {
		want = mif->levels[0].khz;
		t = ktime_get();
		ret = dev_pm_qos_add_request(mif->dev, &mmin, DEV_PM_QOS_MIN_FREQUENCY, want);
		us = ktime_us_delta(ktime_get(), t);
		wait = xdf_st_wait(mif, want, 0);
		bad += xdf_st_report("mif max", ret, want, us, wait);

		/* Release the floor: the drop should wait down-delay-ms (4.9 20 ms). */
		from = xdf_st_prev(mif);
		t = ktime_get();
		ret = dev_pm_qos_request_active(&mmin) ?
		      dev_pm_qos_update_request(&mmin, 0) : -ENODEV;
		us = ktime_us_delta(ktime_get(), t);
		wait = xdf_st_wait(mif, 0, from);
		bad += xdf_st_report("mif release (hold)", ret, 0, us, wait);

		want = xdf_low_target(mif);
		t = ktime_get();
		ret = dev_pm_qos_add_request(mif->dev, &mmax, DEV_PM_QOS_MAX_FREQUENCY, want);
		us = ktime_us_delta(ktime_get(), t);
		wait = xdf_st_wait(mif, want, 0);
		bad += xdf_st_report("mif low", ret, want, us, wait);
	}
	if (intb) {
		want = intb->levels[0].khz;
		t = ktime_get();
		ret = dev_pm_qos_add_request(intb->dev, &imin, DEV_PM_QOS_MIN_FREQUENCY, want);
		us = ktime_us_delta(ktime_get(), t);
		wait = xdf_st_wait(intb, want, 0);
		bad += xdf_st_report("int max", ret, want, us, wait);

		if (dev_pm_qos_request_active(&imin))
			dev_pm_qos_update_request(&imin, 0);
		want = xdf_low_target(intb);
		t = ktime_get();
		ret = dev_pm_qos_add_request(intb->dev, &imax, DEV_PM_QOS_MAX_FREQUENCY, want);
		us = ktime_us_delta(ktime_get(), t);
		wait = xdf_st_wait(intb, want, 0);
		bad += xdf_st_report("int low", ret, want, us, wait);
	}

	/* Restore: lift INT caps before MIF so the MIF->INT lock can follow. */
	t = ktime_get();
	if (dev_pm_qos_request_active(&imax))
		dev_pm_qos_remove_request(&imax);
	if (dev_pm_qos_request_active(&imin))
		dev_pm_qos_remove_request(&imin);
	if (dev_pm_qos_request_active(&mmax))
		dev_pm_qos_remove_request(&mmax);
	if (dev_pm_qos_request_active(&mmin))
		dev_pm_qos_remove_request(&mmin);
	us = ktime_us_delta(ktime_get(), t);
	if (mif)
		msleep(mif->down_delay_ms + 10);
	bad += xdf_st_report("restored", 0, 0, us, 0);
	xdf_st_noti("after");
	pr_info(XDF_ST_PFX "%s (%d mismatches, mif %s, int %s) in %lld ms\n",
		mif && intb && !bad ? "PASS" : "FAIL", bad,
		mif ? "ready" : "absent", intb ? "ready" : "absent",
		ktime_ms_delta(ktime_get(), start));
}

static DECLARE_DELAYED_WORK(xdf_selftest_work, xdf_selftest_fn);

static void __init xdf_selftest_schedule(void)
{
	/*
	 * 105 s: after the cpufreq self-test (100 s) so the two never step MIF
	 * and CPU at once, and well before the diagnostic's 120 s reboot.
	 */
	unsigned long at = msecs_to_jiffies(105000), up = jiffies - INITIAL_JIFFIES;
	struct device_node *np;
	bool want = false;

	for_each_compatible_node(np, NULL, "samsung,exynos9810-acpm-devfreq")
		if (of_device_is_available(np) &&
		    of_property_read_bool(np, "samsung,devfreq-selftest"))
			want = true;
	if (want)
		schedule_delayed_work(&xdf_selftest_work, at > up ? at - up : 0);
}

static int __init xdf_init(void)
{
	int ret;

	ret = devfreq_add_governor(&xdf_governor);
	if (ret)
		return ret;
	ret = platform_driver_register(&xdf_driver);
	if (ret) {
		devfreq_remove_governor(&xdf_governor);
		return ret;
	}
	xdf_selftest_schedule();
	return 0;
}
device_initcall(xdf_init);
