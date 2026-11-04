// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 9810 CPU idle driver: WFI and per-core power-down (C2), staged.
 *
 * The C2 flow follows the crownlte 4.9 vendor kernel
 * (drivers/cpuidle/cpuidle-exynos64.c, drivers/soc/samsung/exynos-powermode.c
 * and drivers/soc/samsung/cal-if/pmucal_cpu.c):
 *
 *   cpu_pm_enter()
 *   cal_cpu_disable(phycpu)   CPU_INFORM[phycpu] = C2 and arm the FlexPMU
 *                             GRP2 "core down" event for this core
 *   PSCI CPU_SUSPEND(0x10000) EL3 saves state and executes WFI; the APM
 *                             firmware powers the core off on the GRP2
 *                             event and back on when GRP1 (wakeup) fires
 *   cal_cpu_enable(phycpu)    CPU_INFORM[phycpu] = 0, disarm GRP2 and
 *                             clear a stale GRP2 pending bit
 *   cpu_pm_exit()
 *
 * Staging (see Documentation/crownlte/cpuidle-staging.txt). The first trial
 * of this flow (q16) enabled C2 on every CPU from boot. Afterwards the device
 * interrupts that all land on CPU0 were no longer serviced. C2 is therefore
 * never entered unless it was armed for that CPU:
 *
 *  - Every C2 state is registered with CPUIDLE_FLAG_OFF and then held
 *    disabled through the driver bit (CPUIDLE_STATE_DISABLED_BY_DRIVER) on
 *    every CPU, so the governor never selects it.
 *  - e9810_enter_c2() also checks a per-CPU arm mask and runs WFI for any CPU
 *    that is not armed.
 *  - CPUs are armed only by the DT-gated self-test, in phases that never
 *    include CPU0 (it owns every SPI and the MCT broadcast interrupt), or by
 *    the DT property samsung,c2-boot-cpus (stage C, a CPU bitmask).
 *  - Arming also requires the MCT tick broadcast device
 *    (exynos9810-mct-bc). The hrtimer broadcast cannot wake a CPU in a low-res
 *    kernel, and it keeps the CPU that owns it out of C2.
 *
 * Any setup problem leaves the kernel on its default WFI idle. Any runtime
 * failure (cpu_pm notifier veto, CAL error, PSCI error) runs WFI for that
 * idle period. Repeated PSCI errors, a CAL error or a failed self-test
 * phase disarm every CPU.
 */

#define pr_fmt(fmt) "exynos9810-cpuidle: " fmt

#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/cpuidle.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/percpu.h>
#include <linux/platform_device.h>
#include <linux/psci.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/sched/idle.h>
#include <linux/sched/task.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include <uapi/linux/psci.h>

#include <asm/cputype.h>
#include <asm/smp_plat.h>

#include <soc/samsung/cal-if.h>
#include <soc/samsung/exynos9810-mct-bc.h>

#include "dt_idle_states.h"

#define E9810_NR_CLUSTERS		2
#define E9810_NR_PHYCPUS		8

/* PMU (0x14060000) registers, from flexpmu_cal_*_exynos9810.h */
#define PMU_CPU_INFORM(p)		(0x0860 + 4 * (p))
#define PMU_EVENT_INT_EN_GRP2		0x7f08

/* CLUSTERx_CPUy_STATUS, indexed by physical cpu (cluster << 2 | core) */
static const u16 pmu_cpu_status[E9810_NR_PHYCPUS] = {
	0x2024, 0x202c, 0x2034, 0x203c,
	0x2004, 0x200c, 0x2014, 0x201c,
};
/* core_on in the APM sequences waits for STATUS[3:0] == 0xf */
#define PMU_CPU_STATUS_ON		0xf

/* PMU_INTR_GEN (0x14070000), read only here, for diagnostics */
#define INTR_GEN_GRP1_ENABLE		0x0100
#define INTR_GEN_GRP1_PEND		0x0108
#define INTR_GEN_GRP2_ENABLE		0x0200
#define INTR_GEN_GRP2_PEND		0x0208
/* LITTLE cores use bits 4-7, BIG cores bits 0-3 */
#define INTR_GEN_BIT(phy)		BIT((phy) ^ 4)

#define E9810_FAIL_STREAK_MAX		32

/* Self-test timing. The diag PID1 restarts the phone at about 120 s. */
/* After the MCT report (86-89 s), before the DVFS self-tests (100 s). */
#define E9810_SELFTEST_MS		90000
#define E9810_PHASE_MS			2500
#define E9810_POLL_MS			20
#define E9810_PING_TIMEOUT_MS		100
#define E9810_SLEEP_MS			10
#define E9810_SLEEPER_STALE_MS		250
#define E9810_RECHECK_MS		300

struct e9810_cluster {
	struct cpuidle_driver drv;
	struct cpumask cpus;
	u32 psci_state[CPUIDLE_STATE_MAX];
	unsigned int id;
	int c2_idx;
	bool verified;
	bool registered;
	const char *why;
	char name[20];
};

struct e9810_stats {
	u64 c2_enter;		/* PSCI CPU_SUSPEND issued for C2 */
	u64 c2_resumed;		/* came back through cpu_resume */
	u64 early;		/* firmware returned without power-down */
	u64 psci_fail;		/* CPU_SUSPEND returned an error */
	u64 unarmed;		/* C2 requested while not armed: WFI */
	u64 wfi_fallback;	/* any C2 request that ran WFI instead */
	int last_err;
	unsigned int fail_streak;
};

static struct e9810_cluster e9810_clusters[E9810_NR_CLUSTERS];
static DEFINE_PER_CPU(struct e9810_stats, e9810_stats);
static unsigned int e9810_phycpu[NR_CPUS];
static unsigned int e9810_cluster_of[NR_CPUS];
static u64 e9810_live_mpidr[NR_CPUS];
static bool e9810_topology_ok;

/* CPUs that may enter C2. Written under e9810_arm_lock, read locklessly. */
static struct cpumask e9810_armed;
static DEFINE_MUTEX(e9810_arm_lock);
static atomic_t e9810_deep_off = ATOMIC_INIT(0);
static const char *e9810_deep_off_why;
static const char *e9810_setup_why;

static struct regmap *e9810_pmu;
static void __iomem *e9810_intr_gen;

static struct delayed_work e9810_selftest_work;
static bool e9810_keep_armed;

/* Called with interrupts off from the idle path: no plain printk here. */
static void e9810_disable_deep(const char *why)
{
	if (atomic_xchg(&e9810_deep_off, 1))
		return;
	WRITE_ONCE(e9810_deep_off_why, why);
	printk_deferred(KERN_WARNING pr_fmt("deep idle disabled, WFI only: %s\n"),
			why);
}

/* State 0: the cpuidle core already stopped RCU for this state. */
static __cpuidle int e9810_enter_wfi(struct cpuidle_device *dev,
				     struct cpuidle_driver *drv, int idx)
{
	cpu_do_idle();
	return 0;
}

/*
 * Deep states carry CPUIDLE_FLAG_RCU_IDLE (set by dt_init_idle_driver()), so
 * RCU is still watching here and the WFI fallback must stop it itself.
 */
static __cpuidle int e9810_fallback_wfi(void)
{
	ct_cpuidle_enter();
	cpu_do_idle();
	ct_cpuidle_exit();
	return 0;
}

static __cpuidle int e9810_enter_c2(struct cpuidle_device *dev,
				    struct cpuidle_driver *drv, int idx)
{
	struct e9810_cluster *cl = container_of(drv, struct e9810_cluster, drv);
	struct e9810_stats *st = this_cpu_ptr(&e9810_stats);
	unsigned int phy = e9810_phycpu[dev->cpu];
	int ret;

	if (!cpumask_test_cpu(dev->cpu, &e9810_armed)) {
		st->unarmed++;
		goto wfi;
	}
	if (atomic_read(&e9810_deep_off))
		goto wfi;

	/* A vetoing notifier has already been rolled back by cpu_pm. */
	if (cpu_pm_enter())
		goto wfi;

	ret = cal_cpu_disable(phy);
	if (ret) {
		cal_cpu_enable(phy);
		cpu_pm_exit();
		e9810_disable_deep("cal_cpu_disable failed");
		goto wfi;
	}

	st->c2_enter++;
	ret = psci_cpu_suspend_enter(cl->psci_state[idx]);

	cal_cpu_enable(phy);
	cpu_pm_exit();

	if (!ret) {
		st->c2_resumed++;
		st->fail_streak = 0;
		return idx;
	}

	/*
	 * arm64 cpu_suspend() reports a firmware return without a power cycle
	 * (a wakeup that was already pending) as -EOPNOTSUPP. That is not an
	 * error; the period is spent in WFI. Real PSCI errors are counted and
	 * a long streak of them switches deep idle off.
	 */
	st->last_err = ret;
	if (ret == -EOPNOTSUPP) {
		st->early++;
	} else {
		st->psci_fail++;
		if (++st->fail_streak >= E9810_FAIL_STREAK_MAX)
			e9810_disable_deep("PSCI CPU_SUSPEND kept failing");
	}
wfi:
	st->wfi_fallback++;
	return e9810_fallback_wfi();
}

static const struct of_device_id e9810_idle_state_match[] = {
	{ .compatible = "samsung,exynos9810-idle-state",
	  .data = e9810_enter_c2 },
	{ },
};

/* 4.9 linear_phycpu(): A55 MPIDRs carry the MT bit, Mongoose ones do not. */
static unsigned int e9810_linear_phycpu(u64 mpidr)
{
	unsigned int lvl = (mpidr & MPIDR_MT_BITMASK) ? 1 : 0;

	return (MPIDR_AFFINITY_LEVEL(mpidr, (1 + lvl)) << 2) |
		MPIDR_AFFINITY_LEVEL(mpidr, lvl);
}

static void e9810_read_mpidr(void *info)
{
	*(u64 *)info = read_cpuid_mpidr();
}

static const char *e9810_init_topology(void)
{
	unsigned long seen = 0;
	unsigned int cpu;

	for_each_possible_cpu(cpu) {
		u64 hwid = cpu_logical_map(cpu);
		unsigned int aff0 = MPIDR_AFFINITY_LEVEL(hwid, 0);
		unsigned int aff1 = MPIDR_AFFINITY_LEVEL(hwid, 1);
		unsigned int phy = (aff1 << 2) | aff0;

		/* Same mapping as 4.9 exynos-powermode.c cpu_disable(). */
		if ((hwid & ~0xffffULL) || aff0 > 3 ||
		    aff1 >= E9810_NR_CLUSTERS || (seen & BIT(phy)))
			return "unexpected cpu reg/MPIDR layout";
		seen |= BIT(phy);
		e9810_phycpu[cpu] = phy;
		e9810_cluster_of[cpu] = aff1;
		cpumask_set_cpu(cpu, &e9810_clusters[aff1].cpus);
	}
	e9810_topology_ok = true;
	return NULL;
}

/*
 * pmucal_powermode_hint() picks the CPU_INFORM slot from the live MPIDR while
 * the core sequences use the DT reg value. They must agree, or one core's
 * power-down event would be armed while another core idles.
 */
static void e9810_verify_clusters(void)
{
	unsigned int c, cpu;

	for (c = 0; c < E9810_NR_CLUSTERS; c++) {
		struct e9810_cluster *cl = &e9810_clusters[c];

		cl->id = c;
		cl->verified = !cpumask_empty(&cl->cpus);
		for_each_cpu(cpu, &cl->cpus) {
			u64 mpidr = 0;

			if (!cpu_online(cpu) ||
			    smp_call_function_single(cpu, e9810_read_mpidr,
						     &mpidr, 1)) {
				cl->verified = false;
				cl->why = "cpu offline at probe";
				continue;
			}
			e9810_live_mpidr[cpu] = mpidr;
			if (e9810_linear_phycpu(mpidr) != e9810_phycpu[cpu]) {
				cl->verified = false;
				cl->why = "live MPIDR does not match DT reg";
			}
		}
	}
}

/*
 * Map DT states (after WFI) to PSCI parameters. Only core power-down at
 * affinity level 0 (4.9: 0x10000) is accepted; the first other state
 * truncates the list. Every accepted state starts disabled (CPUIDLE_FLAG_OFF).
 */
static void e9810_parse_states(struct e9810_cluster *cl)
{
	struct cpuidle_driver *drv = &cl->drv;
	struct device_node *cpu_node, *np;
	int i, idx = 1;

	cl->c2_idx = -1;
	cpu_node = of_cpu_device_node_get(cpumask_first(&cl->cpus));
	for (i = 0; idx < drv->state_count; i++) {
		u32 state;
		int ret;

		np = of_get_cpu_state_node(cpu_node, i);
		if (!np)
			break;
		if (!of_device_is_available(np)) {
			of_node_put(np);
			continue;
		}
		ret = of_property_read_u32(np, "arm,psci-suspend-param", &state);
		of_node_put(np);
		if (ret || state != PSCI_0_2_POWER_STATE_TYPE_MASK) {
			pr_warn("%s: state %d (%s) psci param %#x rejected\n",
				cl->name, idx, drv->states[idx].desc,
				ret ? 0 : state);
			break;
		}
		cl->c2_idx = idx;
		drv->states[idx].enter = e9810_enter_c2;
		drv->states[idx].enter_s2idle = NULL;
		drv->states[idx].flags |= CPUIDLE_FLAG_OFF;
		cl->psci_state[idx++] = state;
	}
	of_node_put(cpu_node);
	drv->state_count = idx;
}

/*
 * CPUIDLE_FLAG_OFF only sets the "disabled by user" bit, which sysfs can
 * clear. Right after registration move the gate to the driver bit, which
 * only this driver changes.
 */
static void e9810_take_gate(struct e9810_cluster *cl)
{
	unsigned int cpu;

	for_each_cpu(cpu, &cl->cpus) {
		struct cpuidle_device *dev = per_cpu(cpuidle_devices, cpu);
		unsigned long long *d;

		if (!dev)
			continue;
		d = &dev->states_usage[cl->c2_idx].disable;
		WRITE_ONCE(*d, (*d | CPUIDLE_STATE_DISABLED_BY_DRIVER) &
			       ~CPUIDLE_STATE_DISABLED_BY_USER);
	}
}

static int e9810_register_cluster(struct e9810_cluster *cl)
{
	struct cpuidle_driver *drv = &cl->drv;
	int ret;

	if (!cl->verified)
		return -ENODEV;

	snprintf(cl->name, sizeof(cl->name), "exynos9810_cl%u", cl->id);
	drv->name = cl->name;
	drv->owner = THIS_MODULE;
	drv->cpumask = &cl->cpus;

	drv->states[0].enter = e9810_enter_wfi;
	drv->states[0].exit_latency = 1;
	drv->states[0].target_residency = 1;
	drv->states[0].power_usage = UINT_MAX;
	strscpy(drv->states[0].name, "WFI", CPUIDLE_NAME_LEN);
	strscpy(drv->states[0].desc, "c1", CPUIDLE_DESC_LEN);

	ret = dt_init_idle_driver(drv, e9810_idle_state_match, 1);
	if (ret <= 0) {
		cl->why = ret ? "DT idle states invalid" : "no DT idle states";
		return ret ? : -ENODEV;
	}

	e9810_parse_states(cl);
	if (cl->c2_idx < 1) {
		cl->why = "no usable C2 state";
		return -ENODEV;
	}

	ret = cpuidle_register(drv, NULL);
	if (ret) {
		cl->why = "cpuidle_register failed";
		return ret;
	}
	cl->registered = true;
	e9810_take_gate(cl);
	cl->why = NULL;
	return 0;
}

/* ------------------------------------------------------------------------ */
/* Arming                                                                   */
/* ------------------------------------------------------------------------ */

/*
 * The governor skips states disabled by the driver; the arm mask is checked
 * again on entry. Same direct bit update as the cpuidle sysfs "disable" file.
 */
static void e9810_state_disable(unsigned int cpu, bool disable)
{
	struct cpuidle_device *dev = per_cpu(cpuidle_devices, cpu);
	struct e9810_cluster *cl = &e9810_clusters[e9810_cluster_of[cpu]];
	unsigned long long *d;

	if (!dev || !cl->registered)
		return;
	d = &dev->states_usage[cl->c2_idx].disable;
	if (disable)
		WRITE_ONCE(*d, *d | CPUIDLE_STATE_DISABLED_BY_DRIVER);
	else
		WRITE_ONCE(*d, *d & ~CPUIDLE_STATE_DISABLED_BY_DRIVER);
}

/* Why @cpu cannot be armed, or NULL. */
static const char *e9810_arm_refused(unsigned int cpu)
{
	if (e9810_setup_why)
		return e9810_setup_why;
	if (!e9810_clusters[e9810_cluster_of[cpu]].registered)
		return "cluster not registered";
	if (!cpu_online(cpu))
		return "cpu offline";
	if (atomic_read(&e9810_deep_off))
		return "deep idle disabled";
	if (!exynos9810_mct_bc_ready())
		return "MCT tick broadcast not active";
	return NULL;
}

static const char *e9810_arm(const struct cpumask *mask)
{
	const char *why = NULL;
	unsigned int cpu;

	mutex_lock(&e9810_arm_lock);
	for_each_cpu(cpu, mask) {
		why = e9810_arm_refused(cpu);
		if (why)
			break;
	}
	if (!why) {
		cpumask_or(&e9810_armed, &e9810_armed, mask);
		/* Arm mask visible before the governor may pick C2. */
		smp_mb();
		for_each_cpu(cpu, mask)
			e9810_state_disable(cpu, false);
	}
	mutex_unlock(&e9810_arm_lock);
	return why;
}

/* Safe from any context that may sleep; a stuck CPU cannot be helped here. */
static void e9810_disarm_all(void)
{
	unsigned int cpu;

	mutex_lock(&e9810_arm_lock);
	for_each_possible_cpu(cpu)
		e9810_state_disable(cpu, true);
	cpumask_clear(&e9810_armed);
	/* Both gates closed before the idle CPUs are kicked. */
	smp_mb();
	mutex_unlock(&e9810_arm_lock);
	wake_up_all_idle_cpus();
}

static int e9810_reboot_cb(struct notifier_block *nb, unsigned long event,
			   void *data)
{
	/* Like 4.9: no deep idle while the restart/power-off path runs. */
	if (event == SYS_RESTART || event == SYS_POWER_OFF ||
	    event == SYS_HALT) {
		cpumask_clear(&e9810_armed);
		/* Disarmed before cpuidle_pause() kicks the idle CPUs. */
		smp_mb();
		cpuidle_pause();
	}
	return NOTIFY_OK;
}

/* First in the chain: CPUs leave C2 before any restart notifier runs. */
static struct notifier_block e9810_reboot_nb = {
	.notifier_call = e9810_reboot_cb,
	.priority = INT_MAX,
};

static const char *e9810_setup(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	bool any = false;
	unsigned int c;
	const char *why;

	why = e9810_init_topology();
	if (why)
		return why;

	e9810_pmu = syscon_regmap_lookup_by_phandle(np, "samsung,pmu-syscon");
	if (IS_ERR(e9810_pmu)) {
		e9810_pmu = NULL;
		return "no PMU syscon";
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return "no PMU_INTR_GEN reg";
	/* Already mapped (not requested) by PMUCAL; map, do not request. */
	e9810_intr_gen = devm_ioremap(&pdev->dev, res->start,
				      resource_size(res));
	if (!e9810_intr_gen)
		return "PMU_INTR_GEN ioremap failed";

	if (!cal_if_is_initialized())
		return "CAL/PMUCAL not initialized";
	if (!psci_ops.cpu_suspend)
		return "PSCI CPU_SUSPEND not available";

	e9810_verify_clusters();

	for (c = 0; c < E9810_NR_CLUSTERS; c++) {
		struct e9810_cluster *cl = &e9810_clusters[c];

		if (!e9810_register_cluster(cl))
			any = true;
		else
			pr_warn("cluster %u: WFI only: %s\n", c,
				cl->why ? cl->why : "error");
	}
	if (!any)
		return "no cluster registered";

	register_reboot_notifier(&e9810_reboot_nb);
	return NULL;
}

/* ------------------------------------------------------------------------ */
/* DT-gated staged self-test                                                */
/* ------------------------------------------------------------------------ */

struct e9810_regs {
	u32 status, inform, event_grp2;
	u32 grp1_en, grp1_pend, grp2_en, grp2_pend;
};

struct e9810_ping {
	call_single_data_t csd;
	u64 t_issue;
	u64 t_run;
	struct e9810_regs running;	/* sampled by the target itself */
	u64 lat_min, lat_max, lat_sum;
	unsigned int sent, answered;
	unsigned int off_seen;		/* PMU status samples not "on" */
	unsigned int armed_seen;	/* C2 hooks found armed while running */
};

struct e9810_sleeper {
	struct task_struct *task;
	unsigned int cpu;
	u64 heartbeat;			/* ktime_get_ns() of the last wakeup */
	u64 over_max;			/* worst oversleep, ns */
	unsigned int loops;
	bool stop;
	bool exited;
};

struct e9810_snap {
	u64 usage, time_ns, rejected, above, below;
	struct e9810_stats st;
};

struct e9810_phase {
	const char *name;
	struct cpumask cpus;
};

static void e9810_read_regs(unsigned int cpu, struct e9810_regs *r)
{
	unsigned int phy = e9810_phycpu[cpu];

	regmap_read(e9810_pmu, pmu_cpu_status[phy], &r->status);
	regmap_read(e9810_pmu, PMU_CPU_INFORM(phy), &r->inform);
	regmap_read(e9810_pmu, PMU_EVENT_INT_EN_GRP2, &r->event_grp2);
	r->grp1_en = readl(e9810_intr_gen + INTR_GEN_GRP1_ENABLE);
	r->grp1_pend = readl(e9810_intr_gen + INTR_GEN_GRP1_PEND);
	r->grp2_en = readl(e9810_intr_gen + INTR_GEN_GRP2_ENABLE);
	r->grp2_pend = readl(e9810_intr_gen + INTR_GEN_GRP2_PEND);
}

static void e9810_log_regs(const char *tag, unsigned int cpu,
			   const struct e9810_regs *r)
{
	u32 bit = INTR_GEN_BIT(e9810_phycpu[cpu]);

	pr_info("cpuidle selftest: %s cpu%u phy=%u status=%#x inform=%#x event_grp2=%#x(%d) grp1_en=%#x(%d) grp1_pend=%#x(%d) grp2_en=%#x(%d) grp2_pend=%#x(%d)\n",
		tag, cpu, e9810_phycpu[cpu], r->status, r->inform,
		r->event_grp2, !!(r->event_grp2 & bit),
		r->grp1_en, !!(r->grp1_en & bit), r->grp1_pend,
		!!(r->grp1_pend & bit), r->grp2_en, !!(r->grp2_en & bit),
		r->grp2_pend, !!(r->grp2_pend & bit));
}

static void e9810_ping_fn(void *info)
{
	struct e9810_ping *p = info;
	unsigned int cpu = smp_processor_id();
	u32 bit = INTR_GEN_BIT(e9810_phycpu[cpu]);

	/* Sampled while this core runs: the C2 hooks must be disarmed. */
	e9810_read_regs(cpu, &p->running);
	if (p->running.inform || (p->running.grp2_en & bit) ||
	    (p->running.event_grp2 & bit))
		p->armed_seen++;
	/* Pairs with the acquire loads in e9810_run_phase(). */
	smp_store_release(&p->t_run, ktime_get_ns());
}

/*
 * Timer wakeups of a C2 core come only through the broadcast device, so each
 * sleeper loop iteration proves one broadcast wakeup of its CPU.
 */
static int e9810_sleeper_fn(void *data)
{
	struct e9810_sleeper *s = data;

	while (!READ_ONCE(s->stop) && !kthread_should_stop()) {
		u64 t0 = ktime_get_ns(), dt;

		msleep(E9810_SLEEP_MS);
		dt = ktime_get_ns() - t0;
		/* msleep() adds a jiffy; anything beyond that is oversleep. */
		if (dt > (E9810_SLEEP_MS + 2 * jiffies_to_msecs(1)) * NSEC_PER_MSEC)
			s->over_max = max(s->over_max, dt -
				(E9810_SLEEP_MS + 2 * jiffies_to_msecs(1)) *
				NSEC_PER_MSEC);
		s->loops++;
		smp_store_release(&s->heartbeat, ktime_get_ns());
	}
	smp_store_release(&s->exited, true);
	return 0;
}

static void e9810_snapshot(struct e9810_snap *s)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu) {
		struct cpuidle_device *dev = per_cpu(cpuidle_devices, cpu);
		struct e9810_cluster *cl = &e9810_clusters[e9810_cluster_of[cpu]];

		s[cpu].st = per_cpu(e9810_stats, cpu);
		if (!dev || !cl->registered)
			continue;
		s[cpu].usage = dev->states_usage[cl->c2_idx].usage;
		s[cpu].time_ns = dev->states_usage[cl->c2_idx].time_ns;
		s[cpu].rejected = dev->states_usage[cl->c2_idx].rejected;
		s[cpu].above = dev->states_usage[cl->c2_idx].above;
		s[cpu].below = dev->states_usage[cl->c2_idx].below;
	}
}

static void e9810_log_setup(void)
{
	unsigned int c, cpu, i;

	pr_info("cpuidle selftest: setup=%s deep_off=%d%s%s mct_bc=%d armed=%*pbl\n",
		e9810_setup_why ? e9810_setup_why : "ok",
		atomic_read(&e9810_deep_off),
		e9810_deep_off_why ? " why=" : "",
		e9810_deep_off_why ? e9810_deep_off_why : "",
		exynos9810_mct_bc_ready(), cpumask_pr_args(&e9810_armed));

	for_each_possible_cpu(cpu)
		pr_info("cpuidle selftest: cpu%u reg=%#llx live_mpidr=%#llx phycpu=%u cluster=%u\n",
			cpu, cpu_logical_map(cpu), e9810_live_mpidr[cpu],
			e9810_phycpu[cpu], e9810_cluster_of[cpu]);

	for (c = 0; c < E9810_NR_CLUSTERS; c++) {
		struct e9810_cluster *cl = &e9810_clusters[c];
		struct cpuidle_driver *drv = &cl->drv;

		pr_info("cpuidle selftest: cluster%u cpus=%*pbl verified=%d registered=%d%s%s\n",
			c, cpumask_pr_args(&cl->cpus), cl->verified,
			cl->registered, cl->why ? " why=" : "",
			cl->why ? cl->why : "");
		if (!cl->registered)
			continue;
		for (i = 0; i < drv->state_count; i++)
			pr_info("cpuidle selftest: cluster%u state%u %s/%s exit=%uus residency=%uus flags=%#x psci=%#x\n",
				c, i, drv->states[i].name, drv->states[i].desc,
				drv->states[i].exit_latency,
				drv->states[i].target_residency,
				drv->states[i].flags, i ? cl->psci_state[i] : 0);
	}
}

/* Returns NULL on success or the failure reason; *failed_cpu is set. */
static const char *e9810_run_phase(const struct e9810_phase *ph,
				   struct e9810_ping *pings,
				   struct e9810_sleeper *sl,
				   struct e9810_snap *a, struct e9810_snap *b,
				   unsigned int *failed_cpu, bool *entered_all)
{
	struct exynos9810_mct_bc_stats bc0, bc1;
	unsigned int samples = 0, cpu;
	const char *why = NULL;
	u64 t_end;

	*failed_cpu = UINT_MAX;
	*entered_all = false;

	for_each_cpu(cpu, &ph->cpus) {
		struct e9810_sleeper *s = &sl[cpu];

		memset(s, 0, sizeof(*s));
		s->cpu = cpu;
		s->heartbeat = ktime_get_ns();
		s->task = kthread_create(e9810_sleeper_fn, s, "c2test/%u", cpu);
		if (IS_ERR(s->task)) {
			s->task = NULL;
			return "cannot create sleeper thread";
		}
		kthread_bind(s->task, cpu);
		get_task_struct(s->task);
		wake_up_process(s->task);
	}

	e9810_snapshot(a);
	exynos9810_mct_bc_get_stats(&bc0);

	why = e9810_arm(&ph->cpus);
	if (why) {
		pr_err("cpuidle selftest: phase %s: cannot arm %*pbl: %s\n",
		       ph->name, cpumask_pr_args(&ph->cpus), why);
		goto stop_sleepers;
	}
	pr_info("cpuidle selftest: phase %s: armed %*pbl for %d ms\n",
		ph->name, cpumask_pr_args(&ph->cpus), E9810_PHASE_MS);

	t_end = ktime_get_ns() + (u64)E9810_PHASE_MS * NSEC_PER_MSEC;
	while (!why && ktime_get_ns() < t_end) {
		u64 deadline;

		msleep(E9810_POLL_MS);

		/* Power state as the PMU sees it, sampled before the ping. */
		samples++;
		for_each_cpu(cpu, &ph->cpus) {
			u32 st;

			regmap_read(e9810_pmu,
				    pmu_cpu_status[e9810_phycpu[cpu]], &st);
			if ((st & 0xf) != PMU_CPU_STATUS_ON)
				pings[cpu].off_seen++;
		}

		for_each_cpu(cpu, &ph->cpus) {
			struct e9810_ping *p = &pings[cpu];

			WRITE_ONCE(p->t_run, 0);
			p->t_issue = ktime_get_ns();
			p->sent++;
			/* -EBUSY: an earlier request never ran; never block. */
			if (smp_call_function_single_async(cpu, &p->csd)) {
				*failed_cpu = cpu;
				why = "IPI csd still busy";
				break;
			}
		}
		if (why)
			break;

		deadline = ktime_get_ns() +
			   (u64)E9810_PING_TIMEOUT_MS * NSEC_PER_MSEC;
		for (;;) {
			bool pending = false;

			for_each_cpu(cpu, &ph->cpus)
				if (!smp_load_acquire(&pings[cpu].t_run))
					pending = true;
			if (!pending || ktime_get_ns() > deadline)
				break;
			usleep_range(50, 100);
		}

		for_each_cpu(cpu, &ph->cpus) {
			struct e9810_ping *p = &pings[cpu];
			u64 t_run = smp_load_acquire(&p->t_run), lat;
			u64 hb = smp_load_acquire(&sl[cpu].heartbeat);

			if (!t_run) {
				*failed_cpu = cpu;
				why = "IPI not answered within 100 ms";
				break;
			}
			lat = t_run > p->t_issue ? t_run - p->t_issue : 0;
			p->answered++;
			p->lat_sum += lat;
			p->lat_min = min(p->lat_min, lat);
			p->lat_max = max(p->lat_max, lat);

			if (ktime_get_ns() - hb >
			    (u64)E9810_SLEEPER_STALE_MS * NSEC_PER_MSEC) {
				*failed_cpu = cpu;
				why = "timer wakeup (broadcast) late by more than 250 ms";
				break;
			}
			/* An armed GRP2 event would let the APM cut a running core. */
			if (p->armed_seen) {
				*failed_cpu = cpu;
				why = "C2 hooks found armed while the core was running";
				break;
			}
		}
	}

	if (why) {
		struct e9810_regs r;

		/* Capture the stuck state before disarming changes it. */
		e9810_read_regs(*failed_cpu, &r);
		e9810_log_regs("FAIL-state", *failed_cpu, &r);
		e9810_disable_deep("selftest phase failed");
		e9810_disarm_all();
		msleep(E9810_RECHECK_MS);
		e9810_read_regs(*failed_cpu, &r);
		e9810_log_regs("after-disarm", *failed_cpu, &r);
		pr_err("cpuidle selftest: phase %s: FAIL cpu%u: %s; after disarm the IPI %s\n",
		       ph->name, *failed_cpu, why,
		       smp_load_acquire(&pings[*failed_cpu].t_run) ?
		       "was answered" : "is still unanswered");
	} else if (!e9810_keep_armed) {
		e9810_disarm_all();
	}

	e9810_snapshot(b);
	exynos9810_mct_bc_get_stats(&bc1);

	*entered_all = true;
	for_each_cpu(cpu, &ph->cpus) {
		const struct e9810_stats *sa = &a[cpu].st, *sb = &b[cpu].st;
		const struct e9810_ping *p = &pings[cpu];

		if (sb->c2_resumed == sa->c2_resumed)
			*entered_all = false;
		pr_info("cpuidle selftest: phase %s cpu%u c2_enter=+%llu resumed=+%llu early=+%llu psci_fail=+%llu last_err=%d unarmed=+%llu wfi_fallback=+%llu | usage=+%llu time_ms=+%llu rejected=+%llu above=+%llu below=+%llu\n",
			ph->name, cpu, sb->c2_enter - sa->c2_enter,
			sb->c2_resumed - sa->c2_resumed, sb->early - sa->early,
			sb->psci_fail - sa->psci_fail, sb->last_err,
			sb->unarmed - sa->unarmed,
			sb->wfi_fallback - sa->wfi_fallback,
			b[cpu].usage - a[cpu].usage,
			div_u64(b[cpu].time_ns - a[cpu].time_ns, NSEC_PER_MSEC),
			b[cpu].rejected - a[cpu].rejected,
			b[cpu].above - a[cpu].above,
			b[cpu].below - a[cpu].below);
		pr_info("cpuidle selftest: phase %s cpu%u ping answered=%u/%u lat_us min=%llu avg=%llu max=%llu | sleeper loops=%u oversleep_max_us=%llu | pmu_off_seen=%u/%u | running: status=%#x inform=%#x armed_seen=%u%s\n",
			ph->name, cpu, p->answered, p->sent,
			p->answered ? div_u64(p->lat_min, NSEC_PER_USEC) : 0,
			p->answered ? div_u64(div_u64(p->lat_sum, p->answered),
					      NSEC_PER_USEC) : 0,
			div_u64(p->lat_max, NSEC_PER_USEC),
			sl[cpu].loops, div_u64(sl[cpu].over_max, NSEC_PER_USEC),
			p->off_seen, samples, p->running.status,
			p->running.inform, p->armed_seen,
			p->armed_seen ? " ARMED-WHILE-RUNNING" : "");
	}
	pr_info("cpuidle selftest: phase %s mct_bc irqs=+%llu programmed=+%llu etime=+%llu write_timeouts=+%llu\n",
		ph->name, bc1.irqs - bc0.irqs,
		bc1.programmed - bc0.programmed, bc1.etime - bc0.etime,
		bc1.write_timeouts - bc0.write_timeouts);

stop_sleepers:
	for_each_cpu(cpu, &ph->cpus) {
		struct e9810_sleeper *s = &sl[cpu];
		u64 deadline = ktime_get_ns() + 500 * NSEC_PER_MSEC;

		if (!s->task)
			continue;
		WRITE_ONCE(s->stop, true);
		wake_up_process(s->task);
		while (!smp_load_acquire(&s->exited) &&
		       ktime_get_ns() < deadline)
			msleep(5);
		/* A sleeper on a stuck CPU is leaked, never waited for. */
		if (smp_load_acquire(&s->exited))
			kthread_stop(s->task);
		else
			pr_err("cpuidle selftest: sleeper on cpu%u did not exit; leaked\n",
			       cpu);
		put_task_struct(s->task);
		s->task = NULL;
	}
	return why;
}

static void e9810_selftest(struct work_struct *work)
{
	struct e9810_phase phases[3];
	struct e9810_snap *a, *b;
	struct e9810_ping *pings;
	struct e9810_sleeper *sl;
	const char *result = "PASS", *why = NULL;
	unsigned int cpu, i, nphases = 0, passed = 0, failed_cpu = UINT_MAX;
	u64 t0 = ktime_get_ns();

	a = kcalloc(nr_cpu_ids, sizeof(*a), GFP_KERNEL);
	b = kcalloc(nr_cpu_ids, sizeof(*b), GFP_KERNEL);
	/* Never freed: a CPU that does not answer may still own its csd. */
	pings = kcalloc(nr_cpu_ids, sizeof(*pings), GFP_KERNEL);
	/* Never freed: a leaked sleeper thread still points into it. */
	sl = kcalloc(nr_cpu_ids, sizeof(*sl), GFP_KERNEL);
	if (!a || !b || !pings || !sl) {
		pr_err("cpuidle selftest: done result=FAIL why=no memory\n");
		kfree(a);
		kfree(b);
		kfree(pings);
		kfree(sl);
		return;
	}
	for_each_possible_cpu(cpu) {
		INIT_CSD(&pings[cpu].csd, e9810_ping_fn, &pings[cpu]);
		pings[cpu].lat_min = U64_MAX;
	}

	e9810_log_setup();
	for_each_online_cpu(cpu) {
		struct e9810_regs r;

		if (!e9810_pmu || !e9810_topology_ok)
			break;
		e9810_read_regs(cpu, &r);
		e9810_log_regs("before", cpu, &r);
	}

	if (raw_smp_processor_id() != 0)
		pr_warn("cpuidle selftest: running on cpu%d, expected cpu0\n",
			raw_smp_processor_id());

	/*
	 * One LITTLE secondary, one BIG CPU, then every secondary. CPU0 owns
	 * every SPI (GIC default affinity) and the MCT broadcast interrupt; it
	 * is never armed here.
	 */
	phases[nphases].name = "little-1";
	cpumask_clear(&phases[nphases].cpus);
	cpumask_set_cpu(1, &phases[nphases++].cpus);
	phases[nphases].name = "big-1";
	cpumask_clear(&phases[nphases].cpus);
	cpumask_set_cpu(4, &phases[nphases++].cpus);
	phases[nphases].name = "secondaries";
	cpumask_copy(&phases[nphases].cpus, cpu_online_mask);
	cpumask_clear_cpu(0, &phases[nphases++].cpus);

	for (i = 0; i < nphases; i++) {
		bool entered_all;

		if (cpumask_empty(&phases[i].cpus) ||
		    !cpumask_subset(&phases[i].cpus, cpu_online_mask)) {
			why = "phase cpus not online";
			result = "SKIPPED";
			break;
		}
		for_each_cpu(cpu, &phases[i].cpus) {
			struct e9810_ping *p = &pings[cpu];

			p->sent = 0;
			p->answered = 0;
			p->off_seen = 0;
			p->armed_seen = 0;
			p->lat_sum = 0;
			p->lat_max = 0;
			p->lat_min = U64_MAX;
		}

		why = e9810_run_phase(&phases[i], pings, sl, a, b,
				      &failed_cpu, &entered_all);
		if (why) {
			result = failed_cpu == UINT_MAX ? "SKIPPED" : "FAIL";
			break;
		}
		if (!entered_all) {
			/* No evidence yet: do not widen the exposure. */
			why = "a CPU never resumed from C2 in this phase";
			result = "INCONCLUSIVE";
			pr_warn("cpuidle selftest: phase %s: %s; stopping\n",
				phases[i].name, why);
			e9810_disarm_all();
			break;
		}
		passed++;
		pr_info("cpuidle selftest: phase %s: PASS\n", phases[i].name);
	}

	pr_info("cpuidle selftest: done result=%s phases_passed=%u/%u%s%s deep_off=%d armed=%*pbl elapsed_ms=%llu\n",
		result, passed, nphases, why ? " why=" : "", why ? why : "",
		atomic_read(&e9810_deep_off), cpumask_pr_args(&e9810_armed),
		div_u64(ktime_get_ns() - t0, NSEC_PER_MSEC));

	kfree(a);
	kfree(b);
	if (!strcmp(result, "PASS") || !strcmp(result, "SKIPPED")) {
		kfree(pings);
		kfree(sl);
	}
}

static void e9810_arm_boot_cpus(struct device_node *np)
{
	struct cpumask mask;
	const char *why;
	u32 bits;

	if (of_property_read_u32(np, "samsung,c2-boot-cpus", &bits) || !bits)
		return;
	cpumask_clear(&mask);
	bitmap_from_arr32(cpumask_bits(&mask), &bits, 32);
	cpumask_and(&mask, &mask, cpu_possible_mask);
	why = e9810_arm(&mask);
	if (why)
		pr_warn("C2 boot arming of %*pbl refused: %s\n",
			cpumask_pr_args(&mask), why);
	else
		pr_info("C2 armed at boot for %*pbl\n", cpumask_pr_args(&mask));
}

static int e9810_cpuidle_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;

	e9810_setup_why = e9810_setup(pdev);
	if (e9810_setup_why)
		pr_warn("deep idle not available, WFI only: %s\n",
			e9810_setup_why);
	else
		pr_info("registered: cluster0=%d cluster1=%d, C2 disarmed on every CPU\n",
			e9810_clusters[0].registered,
			e9810_clusters[1].registered);

	if (!e9810_setup_why)
		e9810_arm_boot_cpus(np);

	e9810_keep_armed = of_property_read_bool(np,
					"samsung,cpuidle-selftest-keep-armed");
	if (of_property_read_bool(np, "samsung,cpuidle-selftest")) {
		unsigned long now = jiffies - INITIAL_JIFFIES;
		unsigned long at = msecs_to_jiffies(E9810_SELFTEST_MS);

		INIT_DELAYED_WORK(&e9810_selftest_work, e9810_selftest);
		/* CPU0 must drive the test: it is never armed. */
		queue_delayed_work_on(0, system_long_wq, &e9810_selftest_work,
				      at > now ? at - now : 0);
	}

	/* Failures are reported, never fatal: the CPUs keep plain WFI. */
	return 0;
}

static const struct of_device_id e9810_cpuidle_of_match[] = {
	{ .compatible = "samsung,exynos9810-cpuidle" },
	{ },
};

static struct platform_driver e9810_cpuidle_driver = {
	.probe = e9810_cpuidle_probe,
	.driver = {
		.name = "exynos9810-cpuidle",
		.of_match_table = e9810_cpuidle_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(e9810_cpuidle_driver);
