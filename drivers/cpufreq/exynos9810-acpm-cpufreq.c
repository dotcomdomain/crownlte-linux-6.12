// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos9810 CAL/ACPM CPUFreq for the unchanged crownlte firmware.
 *
 * This is the frequency-scaling core of Samsung's 4.9 exynos-acme driver
 * (drivers/cpufreq/exynos-acme.c, crownlte 4.9 kernel):
 *  - levels come from cal_dfs_get_rate_table() (ECT DVFS block) and
 *    voltages from cal_dfs_get_asv_table() (FVMap), as in acme init_table();
 *  - the usable range is the CAL min/max narrowed by DT min-freq/max-freq,
 *    as in acme init_domain();
 *  - policy-max is the initial user policy cap, as in acme set_policy();
 *  - a transition is one FREQ_REQ through cal_dfs_set_rate(), and firmware
 *    selects the voltage. get() reads the CMU back through cal_dfs_get_rate()
 *    and returns the last good rate while CAL reports 0 (mid-transition), as
 *    in acme get_freq().
 *
 * Samsung's DVFS Manager is not ported. When a samsung,mif-devfreq supplier
 * is present, each CPU level raises MIF before the CPU goes up, as the 4.9
 * DVFS Manager did:
 *  - the mif-perf floor (with the skew) is a device PM QoS minimum request,
 *    best effort: MIF goes as far as its maximum allows and the CPU still
 *    scales;
 *  - the ECT MINLOCK skew (chipset guidance) of the level the CPU runs is a
 *    hard floor inside the MIF devfreq driver
 *    (exynos9810_mif_hard_floor()): MIF is never programmed below it, a MAX
 *    QoS request cannot override it. It is raised, under the MIF devfreq
 *    lock, before the CPU goes up and lowered only after the CPU went down;
 *  - every transition checks the requested level's skew against the MIF
 *    rate actually reached (CMU readback). If MIF could not get there (a
 *    failed MIF/INT FREQ_REQ) the highest level that
 *    rate satisfies runs instead, and the request is retried with an
 *    exponential backoff (100 ms .. 5 s).
 * Without a supplier, MIF stays at its
 * current rate. In that case every level whose ECT MINLOCK MIF requirement
 * (acme "mif-skew", a chipset guidance constraint) the current MIF rate does
 * not meet is dropped from the table, including when the MIF rate cannot be
 * read.
 *
 * samsung,max-freq-cap is an additional hard ceiling. It is not in 4.9; it
 * keeps big at 4.9's policy-max until thermal throttling has been verified.
 *
 * Cooling devices keep the 4.9 names. 4.9 registered one cpufreq cooling
 * device per acme domain in DT order (domain@0 = cpus 0-3, domain@1 = cpus
 * 4-7) through cpu_cooling.c, which named them "thermal-cpufreq-<idr id>":
 * LITTLE = thermal-cpufreq-0, BIG = thermal-cpufreq-1. The vendor thermal HAL
 * (/vendor/etc/thermal_info_config.json "CoolingDevices", via the
 * /dev/thermal/cdev-by-name links made by /vendor/bin/thermal_symlinks from
 * cooling_deviceN/type) looks them up by that string. So the driver
 * registers them itself instead of setting CPUFREQ_IS_COOLING_DEV (which
 * names them "cpufreq-cpu0"/"cpufreq-cpu4"). The device is still bound to
 * the policy CPU's DT node, so cooling-maps <&cpu4 ...> work unchanged.
 */
#include <linux/cpu.h>
#include <linux/cpu_cooling.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/device.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>
#include <soc/samsung/cal-if.h>
#include <soc/samsung/ect_parser.h>
#include <soc/samsung/exynos9810-devfreq.h>

#define EXYNOS9810_DOMAINS 2
#define EXYNOS9810_LEVELS 48
#define EXYNOS9810_CPUCL0 0x0b040002
#define EXYNOS9810_CPUCL1 0x0b040003
/*
 * acme declares 100 us (TRANSITION_LATENCY). On crownlte one FREQ_REQ plus
 * CMU readback over ACPM channel 0 measured 0.39-0.42 ms (acpm-dvfs selftest
 * timestamps). Declaring 500 us makes schedutil rate-limit to 750 us.
 */
#define EXYNOS9810_TRANSITION_NS 500000
#define EXYNOS9810_SELFTEST_UPTIME_MS 100000
#define EXYNOS9810_SELFTEST_SETTLE_US 200000

struct exynos9810_domain {
	struct device *cpu_dev;
	cpumask_t cpus;
	unsigned int id;
	u32 cal_id;
	struct cpufreq_frequency_table *table;
	u32 *mif_khz;		/* MIF floor per table entry (supplier mode) */
	u32 *skew_khz;		/* ECT MINLOCK MIF requirement per table entry */
	u32 *perf_khz;		/* DT mif-perf-table floor per table entry */
	unsigned int levels;
	unsigned int opps_added;
	/* Diagnostics: CAL inputs and what was derived from them. */
	unsigned long cal_rate[EXYNOS9810_LEVELS];
	unsigned int cal_volt[EXYNOS9810_LEVELS];
	unsigned int cal_levels;
	u32 cal_min, cal_max, dt_min, dt_max, cap;
	unsigned int mif_dropped;
	bool ect_skew;
	u32 policy_max;
	u32 resume_khz;
	unsigned int last_khz;	/* last CMU readback or acknowledged FREQ_REQ */
	struct dev_pm_qos_request mif_request;
	u32 mif_floor;		/* this domain's MIF request, not a reached rate */
	/* ECT skew vs. MIF (supplier mode only) */
	u32 hard_khz;		/* this domain's hard MIF floor (skew) */
	bool skew_short;	/* last transition ran below the request */
	unsigned int retry_ms;	/* backoff of the re-request after a cap */
	struct delayed_work retry_work;
};

struct exynos9810_cpufreq {
	struct device *dev;
	struct devfreq *mif;	/* NULL: MIF is not scaled by anyone */
	u32 mif_cal_id;
	unsigned long mif_static_khz;
	struct exynos9810_domain domains[EXYNOS9810_DOMAINS];
};

static struct exynos9810_cpufreq *provider;

/* Late-log diagnostics: early boot records are lost on the diag setup. */
static DEFINE_MUTEX(exynos9810_status_lock);
static int exynos9810_probe_ret = 1;	/* 1: probe never ran */
static char exynos9810_probe_stage[96] = "none";
static struct delayed_work exynos9810_selftest_work;
static bool exynos9810_selftest_armed;

static __printf(2, 3) void exynos9810_note(int ret, const char *fmt, ...)
{
	va_list args;

	mutex_lock(&exynos9810_status_lock);
	exynos9810_probe_ret = ret;
	va_start(args, fmt);
	vscnprintf(exynos9810_probe_stage, sizeof(exynos9810_probe_stage), fmt, args);
	va_end(args);
	mutex_unlock(&exynos9810_status_lock);
}

/* Match Samsung ACME's descending nearest-higher performance row. */
static int exynos9810_perf_floor(const u32 *pairs, unsigned int count,
			       u32 rate, u32 *floor)
{
	unsigned int i;
	bool matched = false;

	if (!pairs || !count || count > EXYNOS9810_LEVELS)
		return -EINVAL;
	*floor = 0;
	for (i = 0; i < count; i++) {
		if (!pairs[2 * i] || pairs[2 * i + 1] > INT_MAX ||
		    (i && pairs[2 * i] >= pairs[2 * (i - 1)]))
			return -EINVAL;
	}
	for (i = 0; i < count; i++) {
		if (rate <= pairs[2 * i]) {
			*floor = pairs[2 * i + 1];
			matched = true;
		}
		if (rate >= pairs[2 * i])
			break;
	}
	return matched ? 0 : -ERANGE;
}

/*
 * ECT levels are descending. An exact row is used as is; a rate without one
 * takes the row of the nearest higher CPU rate (acme takes the top row above
 * the first match and 0 below it, so this is never lower than acme).
 */
static int exynos9810_skew_floor(struct ect_minlock_domain *ect,
			       u32 rate, u32 *floor)
{
	unsigned int i;

	if (!ect || !ect->level || ect->num_of_level <= 0 ||
	    ect->num_of_level > EXYNOS9810_LEVELS)
		return -EINVAL;
	*floor = ect->level[0].sub_frequencies;
	for (i = 0; i < ect->num_of_level; i++) {
		if (!ect->level[i].main_frequencies ||
		    ect->level[i].sub_frequencies > INT_MAX ||
		    (i && ect->level[i].main_frequencies >=
			  ect->level[i - 1].main_frequencies))
			return -EINVAL;
	}
	for (i = 0; i < ect->num_of_level; i++) {
		if (rate >= ect->level[i].main_frequencies)
			break;
		*floor = ect->level[i].sub_frequencies;
	}
	if (i < ect->num_of_level && rate == ect->level[i].main_frequencies)
		*floor = ect->level[i].sub_frequencies;
	return 0;
}

static struct exynos9810_domain *exynos9810_find_domain(unsigned int cpu)
{
	unsigned int i;

	if (!provider)
		return NULL;
	for (i = 0; i < EXYNOS9810_DOMAINS; i++)
		if (cpumask_test_cpu(cpu, &provider->domains[i].cpus))
			return &provider->domains[i];
	return NULL;
}

/*
 * Set this domain's MIF demands and report the MIF rate reached, in kHz.
 * @floor is the device PM QoS minimum (mif-perf and skew, best effort);
 * @hard is the hard floor inside the MIF devfreq driver (the skew of the
 * level the CPU runs or is about to run), which a MAX QoS request cannot
 * undercut. exynos9810_mif_hard_floor() applies it synchronously under the
 * MIF devfreq lock and re-runs the MIF update even if nothing changed, so a
 * failed MIF/INT FREQ_REQ is retried; the devfreq driver logs MIF failures,
 * the caller judges the reached rate. Only a rejected request (a driver
 * bug) returns an error.
 */
static int exynos9810_set_mif(struct exynos9810_domain *domain, u32 floor, u32 hard,
			      unsigned long *mif_khz)
{
	int ret;

	ret = dev_pm_qos_update_request(&domain->mif_request, floor);
	if (ret < 0)
		return ret;
	domain->mif_floor = floor;
	ret = exynos9810_mif_hard_floor(provider->mif, domain->id, hard, mif_khz);
	if (ret)
		return ret;
	domain->hard_khz = hard;
	return 0;
}

/*
 * The highest level at or below @index (the table is descending) whose ECT
 * skew @mif_khz satisfies; the lowest level if none does.
 */
static unsigned int exynos9810_skew_safe(const struct exynos9810_domain *domain,
					 unsigned int index, unsigned long mif_khz)
{
	unsigned int j;

	for (j = index; j < domain->levels; j++)
		if (domain->skew_khz[j] <= mif_khz)
			return j;
	return domain->levels - 1;
}

#define EXYNOS9810_RETRY_MIN_MS	100
#define EXYNOS9810_RETRY_MAX_MS	5000

/*
 * A transition ran below the governor's request because MIF had not reached
 * the requested level's skew. schedutil does not repeat a request for the
 * frequency it already asked for, so a delayed work refreshes the policy
 * limits (CPUFREQ_NEED_UPDATE_LIMITS) and the request comes back. The delay
 * doubles from 100 ms to 5 s while the shortfall lasts and resets once a
 * transition runs the requested level.
 */
static void exynos9810_skew_short(struct exynos9810_domain *domain, bool short_now)
{
	WRITE_ONCE(domain->skew_short, short_now);
	if (!short_now) {
		domain->retry_ms = 0;
		return;
	}
	if (delayed_work_pending(&domain->retry_work))
		return;
	domain->retry_ms = domain->retry_ms ?
			   min(domain->retry_ms * 2, EXYNOS9810_RETRY_MAX_MS) :
			   EXYNOS9810_RETRY_MIN_MS;
	queue_delayed_work(system_wq, &domain->retry_work,
			   msecs_to_jiffies(domain->retry_ms));
}

static void exynos9810_retry_work(struct work_struct *work)
{
	struct exynos9810_domain *domain =
		container_of(to_delayed_work(work), struct exynos9810_domain, retry_work);
	unsigned int cpu = cpumask_first_and(&domain->cpus, cpu_online_mask);

	if (READ_ONCE(domain->skew_short) && cpu < nr_cpu_ids)
		cpufreq_update_limits(cpu);
}

static int exynos9810_target_index(struct cpufreq_policy *policy,
				 unsigned int index)
{
	struct exynos9810_domain *domain = policy->driver_data;
	struct cpufreq_freqs freqs = { .old = policy->cur, .flags = 0 };
	u32 old_floor = domain->mif_floor, old_hard = domain->hard_khz;
	u32 new_floor = 0;
	unsigned int pick = index;
	unsigned long mif_khz;
	int ret;

	if (index >= domain->levels)
		return -EINVAL;
	if (provider->mif) {
		new_floor = domain->mif_khz[index];
		/*
		 * Before the CPU moves, only raise: the QoS minimum to this
		 * level's floor and the hard floor to its skew. The MIF update
		 * runs on every target, raised or not, so the decision below
		 * uses the rate MIF actually reached.
		 */
		ret = exynos9810_set_mif(domain, max(old_floor, new_floor),
					 max_t(u32, old_hard, domain->skew_khz[index]),
					 &mif_khz);
		if (ret)
			return ret;
		/*
		 * Short of the skew (a failed MIF/INT FREQ_REQ, a skew above
		 * every MIF level): run the highest level the reached rate satisfies,
		 * as the 4.9 DVFS Manager's CPU maximum from the MIF side did.
		 */
		pick = exynos9810_skew_safe(domain, index, mif_khz);
		if (pick != index)
			dev_warn_ratelimited(provider->dev, "domain%u: %u kHz needs MIF >= %u kHz (ECT skew), MIF is %lu kHz: running %u kHz\n",
					     domain->id, domain->table[index].frequency,
					     domain->skew_khz[index], mif_khz,
					     domain->table[pick].frequency);
		exynos9810_skew_short(domain, pick != index);
	}
	freqs.new = domain->table[pick].frequency;
	if (freqs.new != policy->cur) {
		/* CPUFREQ_ASYNC_NOTIFICATION: notify the level actually programmed */
		cpufreq_freq_transition_begin(policy, &freqs);
		ret = cal_dfs_set_rate(domain->cal_id, freqs.new);
		if (ret) {
			/*
			 * A failed firmware acknowledgement does not prove that
			 * the CPU stayed at its previous rate. Keep the raised
			 * MIF floors.
			 */
			pr_err_ratelimited("exynos9810-cpufreq: domain%u FREQ_REQ %u kHz failed: %d\n",
					   domain->id, freqs.new, ret);
			ret = ret > 0 ? -EIO : ret;
		} else {
			WRITE_ONCE(domain->last_khz, freqs.new);
		}
		cpufreq_freq_transition_end(policy, &freqs, ret);
		if (ret)
			return ret;
	}
	/*
	 * The CPU runs table[pick] now: the hard floor drops to its skew. The
	 * QoS minimum drops to the requested level's floor once that level
	 * runs; while capped the higher request stays, so MIF can still get
	 * there.
	 */
	if (provider->mif) {
		ret = exynos9810_set_mif(domain,
					 pick == index ? new_floor : max(old_floor, new_floor),
					 domain->skew_khz[pick], &mif_khz);
		if (ret)
			dev_warn_ratelimited(provider->dev, "MIF floor release failed: %d\n", ret);
	}
	return 0;
}

/* no re-request pending any more (cluster stops, or its policy goes away) */
static void exynos9810_domain_stop_retry(struct exynos9810_domain *domain)
{
	cancel_delayed_work(&domain->retry_work);
	WRITE_ONCE(domain->skew_short, false);
	domain->retry_ms = 0;
}

static unsigned int exynos9810_domain_get(struct exynos9810_domain *domain)
{
	unsigned long rate = cal_dfs_get_rate(domain->cal_id);

	/* acme get_freq(): CAL returns 0 while the CMU is changing state. */
	if (!rate || rate > UINT_MAX)
		return READ_ONCE(domain->last_khz);
	WRITE_ONCE(domain->last_khz, (unsigned int)rate);
	return rate;
}

static unsigned int exynos9810_get(unsigned int cpu)
{
	struct exynos9810_domain *domain = exynos9810_find_domain(cpu);

	return domain ? exynos9810_domain_get(domain) : 0;
}

/*
 * Bring a cluster's rate onto the table with its MIF floors in place: at
 * ->init and when the cluster comes back (->online). The current rate is
 * the CMU readback. On failure the hard floor that covers the current
 * rate stays: nothing has confirmed that the CPU left that rate.
 */
static int exynos9810_domain_start(struct exynos9810_domain *domain,
				   unsigned int *cur_khz)
{
	unsigned int rate, target, ti, i;
	bool listed = false;
	unsigned long mif_khz;
	u32 floor = 0, hard = 0;
	int ret;

	rate = exynos9810_domain_get(domain);
	if (!rate)
		return -EIO;
	/* Table is descending: first entry <= rate, else the lowest entry. */
	ti = domain->levels - 1;
	for (i = 0; i < domain->levels; i++) {
		if (domain->table[i].frequency == rate)
			listed = true;
		if (domain->table[i].frequency <= rate) {
			ti = i;
			break;
		}
	}
	target = domain->table[ti].frequency;
	if (provider->mif) {
		/* Floors of the current rate; the highest ones if it is not listed. */
		for (i = 0; i < domain->levels; i++) {
			if (domain->table[i].frequency == rate) {
				floor = domain->mif_khz[i];
				hard = domain->skew_khz[i];
				break;
			}
			floor = max(floor, domain->mif_khz[i]);
			hard = max(hard, domain->skew_khz[i]);
		}
		/* only raise: an earlier (failed) start may hold a higher floor */
		ret = exynos9810_set_mif(domain, max(floor, domain->mif_floor),
					 max(hard, domain->hard_khz), &mif_khz);
		if (ret)
			return ret;
		if (mif_khz < floor)
			dev_warn(provider->dev, "domain%u: start rate %u kHz with MIF %lu kHz below its %u kHz floor\n",
				 domain->id, rate, mif_khz, floor);
		/*
		 * The level the policy starts at (the current rate, or the
		 * table level an unlisted one moves to) must have its ECT skew
		 * met by the MIF rate reached; else start at the highest level
		 * that is.
		 */
		i = exynos9810_skew_safe(domain, ti, mif_khz);
		if (i != ti) {
			ti = i;
			dev_warn(provider->dev, "domain%u: start rate %u kHz needs more MIF than %lu kHz (ECT skew): starting at %u kHz\n",
				 domain->id, rate, mif_khz, domain->table[ti].frequency);
			target = domain->table[ti].frequency;
			listed = false;
		}
	}
	/*
	 * Move an unlisted rate (e.g. above samsung,max-freq-cap) onto the
	 * table here. CPUFREQ_NEED_INITIAL_FREQ_CHECK would do it in the core,
	 * which BUG()s if that transition fails.
	 */
	if (!listed) {
		ret = cal_dfs_set_rate(domain->cal_id, target);
		if (ret) {
			pr_err("exynos9810-cpufreq: domain%u rate %u kHz is not usable and FREQ_REQ %u kHz failed: %d; MIF hard floor stays at %u kHz\n",
			       domain->id, rate, target, ret, domain->hard_khz);
			return ret > 0 ? -EIO : ret;
		}
		WRITE_ONCE(domain->last_khz, target);
		rate = target;
	}
	/* confirmed: the CPU runs table[ti], its own floors only */
	if (provider->mif) {
		ret = exynos9810_set_mif(domain, domain->mif_khz[ti],
					 domain->skew_khz[ti], &mif_khz);
		if (ret)
			return ret;
	}
	*cur_khz = rate;
	return 0;
}

static int exynos9810_policy_init(struct cpufreq_policy *policy)
{
	struct exynos9810_domain *domain = exynos9810_find_domain(policy->cpu);
	unsigned int rate;
	int ret;

	if (!domain || !domain->levels)
		return -ENODEV;
	ret = exynos9810_domain_start(domain, &rate);
	if (ret) {
		/*
		 * cpufreq_online() frees the new policy; nothing was attached
		 * to it. The MIF floors stay (see exynos9810_domain_start()).
		 */
		exynos9810_domain_stop_retry(domain);
		return ret;
	}
	cpumask_copy(policy->cpus, &domain->cpus);
	policy->driver_data = domain;
	policy->freq_table = domain->table;
	policy->cur = rate;
	policy->suspend_freq = domain->resume_khz;
	policy->cpuinfo.transition_latency = EXYNOS9810_TRANSITION_NS;
	policy->dvfs_possible_from_any_cpu = true;
	return 0;
}

/*
 * The cluster comes back (inactive policy, light-weight online). The core
 * reads policy->cur back through ->get. On failure the core calls ->exit
 * and frees the policy.
 */
static int exynos9810_policy_online(struct cpufreq_policy *policy)
{
	struct exynos9810_domain *domain = policy->driver_data;
	unsigned int rate;

	return exynos9810_domain_start(domain, &rate);
}

/*
 * The last CPU of the cluster goes offline (light-weight; the policy and
 * its cooling device stay). The governor has stopped. Move the cluster
 * synchronously to its lowest level and only then drop the MIF floors to
 * that level's; if the FREQ_REQ fails they stay where they are.
 */
static int exynos9810_policy_offline(struct cpufreq_policy *policy)
{
	struct exynos9810_domain *domain = policy->driver_data;
	unsigned int low = domain->levels - 1;
	struct cpufreq_freqs freqs = {
		.old = policy->cur,
		.new = domain->table[low].frequency,
	};
	unsigned long mif_khz;
	int ret = 0;

	exynos9810_domain_stop_retry(domain);
	if (freqs.new != freqs.old) {
		cpufreq_freq_transition_begin(policy, &freqs);
		ret = cal_dfs_set_rate(domain->cal_id, freqs.new);
		if (!ret)
			WRITE_ONCE(domain->last_khz, freqs.new);
		cpufreq_freq_transition_end(policy, &freqs, ret);
	}
	if (ret) {
		pr_err("exynos9810-cpufreq: domain%u offline: FREQ_REQ %u kHz failed: %d; MIF floors kept\n",
		       domain->id, freqs.new, ret);
		return 0;
	}
	if (provider->mif)
		exynos9810_set_mif(domain, 0, domain->skew_khz[low], &mif_khz);
	return 0;
}

/*
 * The policy is destroyed (with ->offline/->online the core calls ->exit
 * only before freeing it: a failed ->online, or CPU device removal). The
 * cooling device holds a freq_qos request on the policy's constraints, so
 * it goes with it; a new policy registers a new one under the same name.
 * The MIF hard floor is dropped only if no CPU of the cluster is online:
 * otherwise nothing confirmed that the CPU left the rate it protects.
 */
static void exynos9810_policy_exit(struct cpufreq_policy *policy)
{
	struct exynos9810_domain *domain = policy->driver_data;
	unsigned long mif_khz;

	if (policy->cdev) {
		cpufreq_cooling_unregister(policy->cdev);
		policy->cdev = NULL;
	}
	if (!domain)
		return;
	exynos9810_domain_stop_retry(domain);
	if (!provider->mif || !dev_pm_qos_request_active(&domain->mif_request))
		return;
	if (cpumask_intersects(&domain->cpus, cpu_online_mask)) {
		dev_warn(provider->dev, "domain%u: policy gone with the cluster online: MIF hard floor stays at %u kHz\n",
			 domain->id, domain->hard_khz);
		return;
	}
	exynos9810_set_mif(domain, 0, 0, &mif_khz);
}

/* 4.9 cpu_cooling.c name for acme domain N: "thermal-cpufreq-N". */
static void exynos9810_cooling_register(struct cpufreq_policy *policy,
					struct exynos9810_domain *domain)
{
	char type[THERMAL_NAME_LENGTH];
	unsigned long max_state = 0;

	/*
	 * ->ready also runs when an inactive policy comes back online; the
	 * device stays registered across CPU offline (as the core's own
	 * CPUFREQ_IS_COOLING_DEV device does), so the cdev-by-name links made
	 * once at boot stay valid. The policy object survives offline
	 * (->offline); every path that frees it after ->ready calls ->exit
	 * first, which unregisters the device (exynos9810_policy_exit()).
	 */
	if (policy->cdev)
		return;
	snprintf(type, sizeof(type), "thermal-cpufreq-%u", domain->id);
	policy->cdev = of_cpufreq_cooling_register_named(policy, type);
	if (!policy->cdev) {
		dev_warn(provider->dev, "domain%u: cooling device %s not registered (cpu%u needs #cooling-cells)\n",
			 domain->id, type, policy->cpu);
		return;
	}
	policy->cdev->ops->get_max_state(policy->cdev, &max_state);
	dev_info(provider->dev, "domain%u (cpus %*pbl): cooling device %s = cooling_device%d, max_state %lu\n",
		 domain->id, cpumask_pr_args(policy->related_cpus), type,
		 policy->cdev->id, max_state);
}

static void exynos9810_policy_ready(struct cpufreq_policy *policy)
{
	struct exynos9810_domain *domain = policy->driver_data;
	int ret;

	exynos9810_cooling_register(policy, domain);

	/*
	 * acme set_policy(): policy-max is the initial user policy maximum.
	 * A later scaling_max_freq write can replace it, up to the table max.
	 */
	if (!domain->policy_max || !policy->max_freq_req)
		return;
	ret = freq_qos_update_request(policy->max_freq_req, domain->policy_max);
	if (ret < 0)
		dev_warn(provider->dev, "initial CPU policy cap failed: %d\n", ret);
}

static struct freq_attr *exynos9810_attrs[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver exynos9810_driver = {
	.name = "exynos9810-acpm",
	/*
	 * ASYNC_NOTIFICATION: target_index() may program a lower level than
	 * asked (ECT skew) and notifies the level it programmed.
	 * NEED_UPDATE_LIMITS: after such a cap, a limits refresh brings the
	 * governor's unchanged request back (exynos9810_retry_work()).
	 */
	.flags = CPUFREQ_ASYNC_NOTIFICATION | CPUFREQ_NEED_UPDATE_LIMITS,
	/* No CPUFREQ_IS_COOLING_DEV: see exynos9810_cooling_register(). */
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = exynos9810_target_index,
	.get = exynos9810_get,
	.init = exynos9810_policy_init,
	.exit = exynos9810_policy_exit,
	.online = exynos9810_policy_online,
	.offline = exynos9810_policy_offline,
	.ready = exynos9810_policy_ready,
	.suspend = cpufreq_generic_suspend,
	.attr = exynos9810_attrs,
};

static void exynos9810_domain_cleanup(void *data)
{
	struct exynos9810_domain *domain = data;

	if (dev_pm_qos_request_active(&domain->mif_request))
		dev_pm_qos_remove_request(&domain->mif_request);
	while (domain->opps_added) {
		unsigned int index = --domain->opps_added;

		dev_pm_opp_remove(domain->cpu_dev,
				  (unsigned long)domain->table[index].frequency * 1000);
	}
}

static int exynos9810_domain_cpus(struct device_node *node,
				  struct exynos9810_domain *domain, unsigned int id)
{
	struct device_node *cpu_node;
	struct opp_table *existing;
	u64 affinity;
	int count, cpu, i;

	count = of_count_phandle_with_args(node, "cpus", NULL);
	if (count != 4)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		cpu_node = of_parse_phandle(node, "cpus", i);
		if (!cpu_node)
			return -EINVAL;
		cpu = of_cpu_node_to_id(cpu_node);
		affinity = of_get_cpu_hwid(cpu_node, 0);
		of_node_put(cpu_node);
		if (cpu < 0 || !cpu_possible(cpu) || cpumask_test_cpu(cpu, &domain->cpus) ||
		    (affinity >> 8) != id || (affinity & 0xff) >= 4)
			return -EINVAL;
		cpumask_set_cpu(cpu, &domain->cpus);
	}
	/*
	 * Dynamic OPP addition accepts identical duplicates as success. Reject
	 * pre-existing tables so cleanup cannot take ownership of another table.
	 * This platform is the sole OPP creator for these CPU devices.
	 */
	for_each_cpu(cpu, &domain->cpus) {
		struct device *member = get_cpu_device(cpu);

		if (!member)
			return -EPROBE_DEFER;
		existing = dev_pm_opp_get_opp_table(member);
		if (!IS_ERR(existing)) {
			dev_pm_opp_put_opp_table(existing);
			return -EBUSY;
		}
		if (PTR_ERR(existing) != -ENODEV)
			return PTR_ERR(existing);
	}
	domain->cpu_dev = get_cpu_device(cpumask_first(&domain->cpus));
	return domain->cpu_dev ? 0 : -EPROBE_DEFER;
}

static int exynos9810_prepare_domain(struct exynos9810_cpufreq *state,
		struct device_node *node, struct exynos9810_domain *domain,
		unsigned int id)
{
	struct device *dev = state->dev;
	struct dvfs_rate_volt rates[EXYNOS9810_LEVELS];
	struct ect_minlock_domain *ect = NULL;
	const char *ect_name = NULL;
	u32 min_khz, max_khz, perf, skew;
	u32 *pairs = NULL;
	unsigned int i, j;
	cpumask_t shared;
	int cells = 0, count, ret;
	void *block;

	domain->id = id;
	ret = exynos9810_domain_cpus(node, domain, id);
	if (ret)
		return ret;

	/* acme init_domain(): CAL range narrowed by DT min-freq/max-freq. */
	domain->cal_min = cal_dfs_get_min_freq(domain->cal_id);
	domain->cal_max = cal_dfs_get_max_freq(domain->cal_id);
	if (of_property_read_u32(node, "min-freq", &domain->dt_min) ||
	    of_property_read_u32(node, "max-freq", &domain->dt_max))
		return -EINVAL;
	min_khz = max(domain->cal_min, domain->dt_min);
	/*
	 * acme lets DT max-freq replace the CAL max outright. cal_dfs_set_rate()
	 * here refuses rates above the CAL max, so take the smaller one.
	 */
	max_khz = min(domain->cal_max, domain->dt_max);
	if (!of_property_read_u32(node, "samsung,max-freq-cap", &domain->cap))
		max_khz = min(max_khz, domain->cap);
	if (!min_khz || min_khz > max_khz)
		return -EINVAL;
	if (!of_property_read_u32(node, "policy-max", &domain->policy_max)) {
		if (domain->policy_max < min_khz)
			return -EINVAL;
		if (domain->policy_max >= max_khz)
			domain->policy_max = 0;	/* the table already stops there */
	}

	/* acme init_table(): cal_dfs_get_rate_table() + cal_dfs_get_asv_table(). */
	if (cal_dfs_get_lv_num(domain->cal_id) > EXYNOS9810_LEVELS)
		return -E2BIG;
	count = cal_dfs_get_rate_asv_table(domain->cal_id, rates);
	if (count <= 0 || count > EXYNOS9810_LEVELS)
		return -ENODEV;
	domain->cal_levels = count;
	for (i = 0; i < count; i++) {
		domain->cal_rate[i] = rates[i].rate;
		domain->cal_volt[i] = rates[i].volt;
	}

	/* Optional 4.9 mif-perf table; required to drive a MIF supplier. */
	if (of_property_present(node, "samsung,mif-perf-table")) {
		cells = of_property_count_u32_elems(node, "samsung,mif-perf-table");
		if (cells <= 0 || cells % 2 || cells > EXYNOS9810_LEVELS * 2)
			return -EINVAL;
		pairs = devm_kcalloc(dev, cells, sizeof(*pairs), GFP_KERNEL);
		if (!pairs)
			return -ENOMEM;
		ret = of_property_read_u32_array(node, "samsung,mif-perf-table", pairs, cells);
		if (ret)
			return ret;
	} else if (state->mif) {
		return -EINVAL;
	}
	if (!of_property_read_string(node, "samsung,mif-ect-domain", &ect_name)) {
		block = ect_get_block(BLOCK_MINLOCK);
		ect = block ? ect_minlock_get_domain(block, (char *)ect_name) : NULL;
	}
	domain->ect_skew = !!ect;
	/*
	 * acme skips an absent ECT MINLOCK domain (crownlte reportedly has no
	 * CPUCL0 one). The supplier path keeps the stricter rule that big must
	 * have its real skew table.
	 */
	if (state->mif && !ect && id)
		return -ENODEV;

	domain->table = devm_kcalloc(dev, count + 1, sizeof(*domain->table), GFP_KERNEL);
	domain->mif_khz = devm_kcalloc(dev, count, sizeof(*domain->mif_khz), GFP_KERNEL);
	domain->skew_khz = devm_kcalloc(dev, count, sizeof(*domain->skew_khz), GFP_KERNEL);
	domain->perf_khz = devm_kcalloc(dev, count, sizeof(*domain->perf_khz), GFP_KERNEL);
	if (!domain->table || !domain->mif_khz || !domain->skew_khz || !domain->perf_khz)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, exynos9810_domain_cleanup, domain);
	if (ret)
		return ret;
	for (i = 0, j = 0; i < count; i++) {
		if (!rates[i].rate || rates[i].rate > UINT_MAX || !rates[i].volt ||
		    (i && rates[i].rate >= rates[i - 1].rate))
			return -EINVAL;
		if (rates[i].rate < min_khz || rates[i].rate > max_khz)
			continue;
		perf = 0;
		if (pairs) {
			ret = exynos9810_perf_floor(pairs, cells / 2, rates[i].rate, &perf);
			if (ret)
				return ret;
		}
		skew = 0;
		if (ect) {
			ret = exynos9810_skew_floor(ect, rates[i].rate, &skew);
			if (ret)
				return ret;
		}
		/*
		 * Nobody scales MIF: keep only levels the current MIF rate is
		 * known to satisfy. An unreadable MIF rate satisfies none.
		 */
		if (!state->mif && skew > state->mif_static_khz) {
			domain->mif_dropped++;
			continue;
		}
		domain->table[j].frequency = rates[i].rate;
		domain->table[j].driver_data = j;
		domain->perf_khz[j] = perf;
		domain->skew_khz[j] = skew;
		domain->mif_khz[j] = max(perf, skew);
		ret = dev_pm_opp_add(domain->cpu_dev, rates[i].rate * 1000, rates[i].volt);
		if (ret)
			return ret;
		domain->opps_added = ++j;
	}
	if (!j)
		return -ERANGE;
	domain->levels = j;
	domain->table[j].frequency = CPUFREQ_TABLE_END;
	/* Suspend at the CAL resume rate, or the highest table rate below it. */
	domain->resume_khz = 0;
	for (i = 0; i < j; i++) {
		u32 f = domain->table[i].frequency;

		if (f <= cal_dfs_get_resume_freq(domain->cal_id) && f > domain->resume_khz)
			domain->resume_khz = f;
	}
	ret = dev_pm_opp_set_sharing_cpus(domain->cpu_dev, &domain->cpus);
	if (ret)
		return ret;
	/*
	 * The OPP helper can skip failed member allocations without an error.
	 * All four CPUs must actually share the calibrated table.
	 */
	ret = dev_pm_opp_get_sharing_cpus(domain->cpu_dev, &shared);
	if (ret)
		return ret;
	if (!cpumask_equal(&shared, &domain->cpus))
		return -ENOMEM;
	domain->last_khz = cal_dfs_get_rate(domain->cal_id);
	if (!state->mif)
		return 0;
	return dev_pm_qos_add_request(state->mif->dev.parent, &domain->mif_request,
				     DEV_PM_QOS_MIN_FREQUENCY, 0);
}

static void exynos9810_put_mif(void *data)
{
	put_device(data);
}

static int exynos9810_get_mif_supplier(struct exynos9810_cpufreq *state)
{
	struct device *dev = state->dev;
	int ret;

	state->mif = devfreq_get_devfreq_by_phandle(dev, "samsung,mif-devfreq", 0);
	if (IS_ERR(state->mif)) {
		ret = PTR_ERR(state->mif);
		state->mif = NULL;
		/*
		 * cpufreq/ links before devfreq/, so an enabled MIF devfreq that
		 * has not probed yet reads as -ENODEV rather than a deferral.
		 */
		if (ret == -ENODEV) {
			struct device_node *np = of_parse_phandle(dev->of_node,
								  "samsung,mif-devfreq", 0);

			if (np && of_device_is_available(np))
				ret = -EPROBE_DEFER;
			of_node_put(np);
		}
		return ret;
	}
	if (!state->mif->profile->get_cur_freq) {
		state->mif = NULL;
		return -ENODEV;
	}
	get_device(&state->mif->dev);
	ret = devm_add_action_or_reset(dev, exynos9810_put_mif, &state->mif->dev);
	if (ret) {
		state->mif = NULL;
		return ret;
	}
	if (!device_link_add(dev, state->mif->dev.parent, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -ENOMEM;
	return 0;
}

/* ---------------------------------------------------------------------- */
/* DT-gated diagnostic (samsung,cpufreq-selftest). Every line is prefixed. */

static void exynos9810_selftest_levels(struct exynos9810_domain *d)
{
	char buf[256];
	unsigned int i, len;

	for (i = 0; i < d->cal_levels; i += 6) {
		unsigned int k;

		len = 0;
		for (k = i; k < d->cal_levels && k < i + 6; k++)
			len += scnprintf(buf + len, sizeof(buf) - len, " %lu@%u",
					 d->cal_rate[k], d->cal_volt[k]);
		pr_info("cpufreq selftest: domain%u cal kHz@fvmap-volt [%u..]%s\n",
			d->id, i, buf);
	}
	for (i = 0; i < d->levels; i += 6) {
		unsigned int k;

		len = 0;
		for (k = i; k < d->levels && k < i + 6; k++)
			len += scnprintf(buf + len, sizeof(buf) - len, " %u(skew%u,perf%u)",
					 d->table[k].frequency, d->skew_khz[k], d->perf_khz[k]);
		pr_info("cpufreq selftest: domain%u table[%u..]%s\n", d->id, i, buf);
	}
}

static bool exynos9810_selftest_step(struct cpufreq_policy *policy,
				     struct exynos9810_domain *d,
				     struct freq_qos_request *min_req,
				     struct freq_qos_request *max_req,
				     unsigned int *pinned, unsigned int request,
				     unsigned int expect)
{
	ktime_t start = ktime_get();
	unsigned long cmu;
	unsigned int cur;
	bool ok;
	int r1, r2;

	/* Move the bound on the far side first so min never exceeds max. */
	if (request >= *pinned) {
		r1 = freq_qos_update_request(max_req, request);
		r2 = freq_qos_update_request(min_req, request);
	} else {
		r1 = freq_qos_update_request(min_req, request);
		r2 = freq_qos_update_request(max_req, request);
	}
	*pinned = request;
	/* QoS changes are applied from the policy update work; poll for it. */
	for (;;) {
		cur = READ_ONCE(policy->cur);
		cmu = cal_dfs_get_rate(d->cal_id);
		ok = cur == expect && cmu == expect;
		if (ok || ktime_us_delta(ktime_get(), start) > EXYNOS9810_SELFTEST_SETTLE_US)
			break;
		usleep_range(200, 400);
	}
	pr_info("cpufreq selftest: policy%u request=%u expect=%u limits=%u-%u cur=%u cmu=%lu qos=%d/%d %lldus %s\n",
		policy->cpu, request, expect, READ_ONCE(policy->min),
		READ_ONCE(policy->max), cur, cmu, r1, r2,
		ktime_us_delta(ktime_get(), start), ok ? "PASS" : "FAIL");
	return ok;
}

static void exynos9810_selftest_domain(struct exynos9810_domain *d)
{
	struct freq_qos_request min_req = {}, max_req = {};
	struct cpufreq_policy *policy;
	unsigned int cpu, top, steps[4], expect[4], nsteps = 0, pass = 0, i;
	unsigned int pinned, old_min, old_max;
	ktime_t start;
	int ret;

	cpu = cpumask_first_and(&d->cpus, cpu_online_mask);
	policy = cpu < nr_cpu_ids ? cpufreq_cpu_get(cpu) : NULL;
	if (!policy) {
		pr_info("cpufreq selftest: domain%u cpus=%*pbl no policy (online=%*pbl)\n",
			d->id, cpumask_pr_args(&d->cpus), cpumask_pr_args(cpu_online_mask));
		return;
	}
	pr_info("cpufreq selftest: policy%u domain%u cal=%#x cpus=%*pbl related=%*pbl gov=%s cur=%u cmu=%lu limits=%u-%u cpuinfo=%u-%u latency=%uns delay=%uus suspend=%u cdev=%s/%d\n",
		policy->cpu, d->id, d->cal_id, cpumask_pr_args(policy->cpus),
		cpumask_pr_args(policy->related_cpus),
		policy->governor ? policy->governor->name : "none", policy->cur,
		cal_dfs_get_rate(d->cal_id), policy->min, policy->max,
		policy->cpuinfo.min_freq, policy->cpuinfo.max_freq,
		policy->cpuinfo.transition_latency,
		cpufreq_policy_transition_delay_us(policy), policy->suspend_freq,
		policy->cdev ? policy->cdev->type : "none",
		policy->cdev ? policy->cdev->id : -1);
	pr_info("cpufreq selftest: domain%u cal_min=%u cal_max=%u dt_min=%u dt_max=%u cap=%u policy_max=%u levels=%u/%u mif_dropped=%u ect_skew=%d\n",
		d->id, d->cal_min, d->cal_max, d->dt_min, d->dt_max, d->cap,
		d->policy_max, d->levels, d->cal_levels, d->mif_dropped, d->ect_skew);
	exynos9810_selftest_levels(d);

	/* The table is descending: [0] is the max, [levels - 1] the min. */
	top = d->table[0].frequency;
	if (d->policy_max && d->policy_max < top)
		top = d->policy_max;
	steps[nsteps] = d->table[d->levels - 1].frequency;
	expect[nsteps++] = steps[0];
	steps[nsteps] = d->table[d->levels / 2].frequency;
	expect[nsteps++] = min(steps[1], top);
	steps[nsteps] = top;
	expect[nsteps++] = top;
	/* Above the cap: must be clamped to the top of the table. */
	if (d->dt_max > d->table[0].frequency) {
		steps[nsteps] = d->dt_max;
		expect[nsteps++] = top;
	}

	old_min = policy->min;
	old_max = policy->max;
	start = ktime_get();
	pinned = policy->cur;
	ret = freq_qos_add_request(&policy->constraints, &min_req, FREQ_QOS_MIN,
				   FREQ_QOS_MIN_DEFAULT_VALUE);
	if (ret < 0)
		goto out;
	ret = freq_qos_add_request(&policy->constraints, &max_req, FREQ_QOS_MAX,
				   FREQ_QOS_MAX_DEFAULT_VALUE);
	if (ret < 0) {
		freq_qos_remove_request(&min_req);
		goto out;
	}
	for (i = 0; i < nsteps; i++)
		pass += exynos9810_selftest_step(policy, d, &min_req, &max_req,
						 &pinned, steps[i], expect[i]);
	freq_qos_remove_request(&min_req);
	freq_qos_remove_request(&max_req);
	/* Wait for the original limits to come back. */
	while ((READ_ONCE(policy->min) != old_min || READ_ONCE(policy->max) != old_max) &&
	       ktime_us_delta(ktime_get(), start) < 2 * EXYNOS9810_SELFTEST_SETTLE_US * (nsteps + 1))
		usleep_range(500, 1000);
	ret = 0;
out:
	pr_info("cpufreq selftest: policy%u restored ret=%d limits=%u-%u (was %u-%u) cur=%u cmu=%lu steps=%u pass=%u %lldus\n",
		policy->cpu, ret, READ_ONCE(policy->min), READ_ONCE(policy->max),
		old_min, old_max, READ_ONCE(policy->cur), cal_dfs_get_rate(d->cal_id),
		nsteps, pass, ktime_us_delta(ktime_get(), start));
	cpufreq_cpu_put(policy);
}

static void exynos9810_selftest(struct work_struct *work)
{
	struct exynos9810_cpufreq *state;
	struct thermal_zone_device *tz;
	ktime_t start = ktime_get();
	int temp = 0, ret, stage_ret;
	unsigned int i;

	mutex_lock(&exynos9810_status_lock);
	stage_ret = exynos9810_probe_ret;
	pr_info("cpufreq selftest: start probe_ret=%d stage=%s driver=%s\n",
		stage_ret, exynos9810_probe_stage, cpufreq_get_current_driver() ?: "none");
	mutex_unlock(&exynos9810_status_lock);
	state = smp_load_acquire(&provider);
	if (!state) {
		pr_info("cpufreq selftest: no registered provider; ready lit=%d big=%d cmu lit=%lu big=%lu\n",
			cal_dfs_is_ready(EXYNOS9810_CPUCL0), cal_dfs_is_ready(EXYNOS9810_CPUCL1),
			cal_dfs_get_rate(EXYNOS9810_CPUCL0), cal_dfs_get_rate(EXYNOS9810_CPUCL1));
		return;
	}
	pr_info("cpufreq selftest: mif mode=%s cal=%#x probe_cmu=%lu now_cmu=%lu\n",
		state->mif ? "devfreq" : "static", state->mif_cal_id,
		state->mif_static_khz,
		state->mif_cal_id ? cal_dfs_get_rate(state->mif_cal_id) : 0);
	tz = thermal_zone_get_zone_by_name("BIG");
	ret = IS_ERR(tz) ? PTR_ERR(tz) : thermal_zone_get_temp(tz, &temp);
	pr_info("cpufreq selftest: thermal BIG zone=%s ret=%d temp=%d mC\n",
		IS_ERR(tz) ? "absent" : "present", ret, temp);
	for (i = 0; i < EXYNOS9810_DOMAINS; i++)
		exynos9810_selftest_domain(&state->domains[i]);
	pr_info("cpufreq selftest: done %lldus\n", ktime_us_delta(ktime_get(), start));
}

static void exynos9810_selftest_arm(struct device *dev)
{
	s64 uptime_ms, delay_ms;

	if (exynos9810_selftest_armed ||
	    !of_property_read_bool(dev->of_node, "samsung,cpufreq-selftest"))
		return;
	exynos9810_selftest_armed = true;
	uptime_ms = ktime_to_ms(ktime_get_boottime());
	delay_ms = max_t(s64, EXYNOS9810_SELFTEST_UPTIME_MS - uptime_ms, 0);
	INIT_DELAYED_WORK(&exynos9810_selftest_work, exynos9810_selftest);
	schedule_delayed_work(&exynos9810_selftest_work, msecs_to_jiffies(delay_ms));
}

/* ---------------------------------------------------------------------- */

static int exynos9810_cpufreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos9810_cpufreq *state;
	struct device_node *child;
	unsigned int id = 0;
	u32 cal_id;
	int ret;

	exynos9810_selftest_arm(dev);
	if (provider)
		return -EBUSY;
	state = devm_kzalloc(dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;
	state->dev = dev;

	/* Bind each child to its CAL domain and wait for ACPM DVFS + FVMap. */
	for_each_available_child_of_node(dev->of_node, child) {
		if (id >= EXYNOS9810_DOMAINS) {
			of_node_put(child);
			exynos9810_note(-EINVAL, "too many domains");
			return -EINVAL;
		}
		if (of_property_read_u32(child, "samsung,cal-id", &cal_id) ||
		    cal_id != (id ? EXYNOS9810_CPUCL1 : EXYNOS9810_CPUCL0)) {
			of_node_put(child);
			exynos9810_note(-EINVAL, "domain%u bad samsung,cal-id", id);
			return -EINVAL;
		}
		if (!cal_dfs_is_ready(cal_id)) {
			of_node_put(child);
			exynos9810_note(-EPROBE_DEFER, "domain%u CAL %#x not ready", id, cal_id);
			return dev_err_probe(dev, -EPROBE_DEFER,
					     "CAL DVFS %#x not ready\n", cal_id);
		}
		state->domains[id++].cal_id = cal_id;
	}
	if (id != EXYNOS9810_DOMAINS) {
		exynos9810_note(-ENODEV, "found %u domains", id);
		return -ENODEV;
	}

	for (id = 0; id < EXYNOS9810_DOMAINS; id++)
		INIT_DELAYED_WORK(&state->domains[id].retry_work, exynos9810_retry_work);
	if (of_property_present(dev->of_node, "samsung,mif-devfreq")) {
		ret = exynos9810_get_mif_supplier(state);
		if (ret) {
			exynos9810_note(ret, "MIF devfreq supplier");
			return dev_err_probe(dev, ret, "MIF devfreq supplier unavailable\n");
		}
		/* the skew is a hard floor in the Exynos9810 MIF devfreq driver */
		ret = exynos9810_mif_hard_floor(state->mif, 0, 0, NULL);
		if (ret) {
			exynos9810_note(ret, "MIF hard floor");
			return dev_err_probe(dev, ret, "samsung,mif-devfreq is not the Exynos9810 MIF devfreq\n");
		}
	} else if (!of_property_read_u32(dev->of_node, "samsung,mif-cal-id",
					 &state->mif_cal_id)) {
		state->mif_static_khz = cal_dfs_get_rate(state->mif_cal_id);
	}

	id = 0;
	for_each_available_child_of_node(dev->of_node, child) {
		ret = exynos9810_prepare_domain(state, child, &state->domains[id], id);
		if (ret) {
			of_node_put(child);
			exynos9810_note(ret, "domain%u prepare", id);
			return dev_err_probe(dev, ret, "CPU domain%u unavailable\n", id);
		}
		id++;
	}
	if (cpumask_intersects(&state->domains[0].cpus, &state->domains[1].cpus)) {
		exynos9810_note(-EINVAL, "overlapping domains");
		return -EINVAL;
	}
	smp_store_release(&provider, state);
	ret = cpufreq_register_driver(&exynos9810_driver);
	if (ret) {
		provider = NULL;
		exynos9810_note(ret, "cpufreq_register_driver");
		return dev_err_probe(dev, ret, "cpufreq registration failed\n");
	}
	exynos9810_note(0, "registered mif=%s levels=%u+%u",
			state->mif ? "devfreq" : "static",
			state->domains[0].levels, state->domains[1].levels);
	dev_info(dev, "registered: domain0 %u levels %u-%u kHz, domain1 %u levels %u-%u kHz\n",
		 state->domains[0].levels,
		 state->domains[0].table[state->domains[0].levels - 1].frequency,
		 state->domains[0].table[0].frequency, state->domains[1].levels,
		 state->domains[1].table[state->domains[1].levels - 1].frequency,
		 state->domains[1].table[0].frequency);
	return 0;
}

static const struct of_device_id exynos9810_cpufreq_match[] = {
	{ .compatible = "samsung,exynos9810-acpm-cpufreq" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos9810_cpufreq_match);

/*
 * No remove: the driver is built in, unbinding is suppressed, and the late
 * selftest work reads the provider without a reference.
 */
static struct platform_driver exynos9810_cpufreq_platform_driver = {
	.probe = exynos9810_cpufreq_probe,
	.driver = {
		.name = "exynos9810-acpm-cpufreq",
		.of_match_table = exynos9810_cpufreq_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(exynos9810_cpufreq_platform_driver);
MODULE_DESCRIPTION("Exynos9810 firmware CPUFreq with original CPU/MIF constraints");
MODULE_LICENSE("GPL");
