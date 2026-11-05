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
 *
 * C2 hook reset (4.9 parity, Documentation/crownlte/cpuidle-secondaries-q22b.md):
 * 4.9 runs cal_cpu_enable() on every CPU as it comes online
 * (exynos-powermode.c exynos_hotplug_in_callback(), cpuhp
 * CPUHP_AP_EXYNOS_CPU_UP_POWER_CONTROL). That clears the CPU_INFORM slot,
 * the core's PMU_EVENT_INTERRUPT_ENABLE_GRP2 bit and any latched GRP2
 * (core-down) event. The bootloader leaves EVENT_INTERRUPT_ENABLE_GRP2 at
 * 0xffffffff, so without the reset every WFI of a core latches a GRP2
 * event, and the first cal_cpu_disable() of that core hands the APM a
 * core-down request while the core is still running Linux (q22b "before"
 * dump: event_grp2=0xffffffff grp2_pend=0xef). This driver does the same
 * reset from a cpuhp online callback and again on each CPU right before it
 * is armed, and clears a latched GRP2 event of its own core (counted as
 * stale_grp2) before every cal_cpu_disable().
 */

#define pr_fmt(fmt) "exynos9810-cpuidle: " fmt

#include <linux/arm-smccc.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/cpuhotplug.h>
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

#include <asm/barrier.h>
#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/suspend.h>

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
/*
 * CLUSTERx_CPUy_CONFIGURATION sits 4 bytes below STATUS. 4.9
 * exynos-pmu.c exynos_cpu_up() sets LOCAL_PWR_CFG [3:0] = 0xf to power a
 * core on; the APM core-on sequence writes the same field.
 */
#define PMU_CPU_CONFIG(phy)		(pmu_cpu_status[phy] - 4)
#define PMU_CPU_LOCAL_PWR_CFG		0xf

/*
 * PMU_INTR_GEN (0x14070000). Read for diagnostics; the only write here is
 * a single-bit write-1-to-clear of this core's own latched GRP2 event.
 */
#define INTR_GEN_GRP1_ENABLE		0x0100
#define INTR_GEN_GRP1_PEND		0x0108
#define INTR_GEN_GRP1_CLEAR		0x010c
#define INTR_GEN_GRP2_ENABLE		0x0200
#define INTR_GEN_GRP2_PEND		0x0208
#define INTR_GEN_GRP2_CLEAR		0x020c
/* LITTLE cores use bits 4-7, BIG cores bits 0-3 */
#define INTR_GEN_BIT(phy)		BIT((phy) ^ 4)

#define E9810_FAIL_STREAK_MAX		32

/*
 * Where a CPU is in the C2 path, written by the CPU itself. A CPU that
 * stops answering IPIs is located by this marker.
 */
enum {
	E9810_ST_KERNEL = 0,	/* not in the C2 path */
	E9810_ST_CAL_OFF,	/* in cal_cpu_disable(): holds pmucal_cpu_lock */
	E9810_ST_FIRMWARE,	/* PSCI CPU_SUSPEND issued: EL3, or powered off */
	E9810_ST_CAL_ON,	/* back from EL3, in cal_cpu_enable() */
	E9810_ST_PM_EXIT,	/* in cpu_pm_exit() */
};

static const char * const e9810_stage_name[] = {
	[E9810_ST_KERNEL]	= "kernel",
	[E9810_ST_CAL_OFF]	= "cal_cpu_disable",
	[E9810_ST_FIRMWARE]	= "firmware",
	[E9810_ST_CAL_ON]	= "cal_cpu_enable",
	[E9810_ST_PM_EXIT]	= "cpu_pm_exit",
};

/* Self-test timing. The diag PID1 restarts the phone at about 120 s. */
/* After the MCT report (86-90 s); the BIG ramp and soak end at about 106 s. */
#define E9810_SELFTEST_MS		90000
/* big-4-soak length; DT samsung,cpuidle-selftest-soak-ms (1000..10000) */
#define E9810_SOAK_MS_DEFAULT		10000
#define E9810_SOAK_MS_MIN		1000
#define E9810_PHASE_MS			800
#define E9810_POLL_MS			20
#define E9810_PING_TIMEOUT_MS		100
#define E9810_SLEEP_MS			10
#define E9810_SLEEPER_STALE_MS		250
#define E9810_RECHECK_MS		300
#define E9810_RECOVER_WAIT_MS		200
#define E9810_LOST_REMINDERS		8
#define E9810_LOST_REMINDER_MS		5000

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
	u64 stale_grp2;		/* own GRP2 event latched before arming */
	u64 guarded;		/* C2 refused by the cluster-0 limit: WFI */
	int last_err;
	unsigned int fail_streak;
};

static struct e9810_cluster e9810_clusters[E9810_NR_CLUSTERS];
static DEFINE_PER_CPU(struct e9810_stats, e9810_stats);
static DEFINE_PER_CPU(u32, e9810_stage);
/* C2 hook resets (cal_cpu_enable() on the CPU) and the last CAL result. */
static DEFINE_PER_CPU(u32, e9810_hooks_reset);
static DEFINE_PER_CPU(int, e9810_hooks_ret);
static unsigned int e9810_phycpu[NR_CPUS];
static unsigned int e9810_cpu_of_phy[E9810_NR_PHYCPUS];
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

/* CPUs that never came back from C2 and could not be recovered. */
static struct cpumask e9810_lost;
static struct delayed_work e9810_lost_work;
static unsigned int e9810_lost_reminders;

/* PSCI CPU_ON function ID from the DT psci node (v0.1 IDs), 0 if unknown. */
static u32 e9810_psci_cpu_on_fn;

/*
 * Cluster-0 (LITTLE, boot cluster) C2 concurrency limit. q40: two LITTLE
 * cores in C2 together passed (430 cycles), the first time a third one went
 * down it never woke (cpu3, wake never latched in GRP1_PEND, then stuck in
 * EL3). q22b lost cpu1 and cpu3 the same way. Until the firmware side is
 * understood, at most this many cluster-0 cores are inside the C2 path
 * (cal_cpu_disable() .. cal_cpu_enable()) at once; the others idle in WFI.
 * 0 = no limit. DT: samsung,c2-little-limit. The self-test changes it per
 * phase.
 */
#define E9810_LITTLE_LIMIT_DEFAULT	2
static int e9810_little_limit = E9810_LITTLE_LIMIT_DEFAULT;
/* The configured value (default or DT), restored after each test phase. */
static int e9810_little_limit_cfg = E9810_LITTLE_LIMIT_DEFAULT;

/*
 * LITTLE (cluster 0) C2 is off unless DT samsung,c2-little-enable. With
 * three LITTLE cores armed, one core was lost in q22b, q40 and q43 (q43 even
 * with at most two in C2): powered off, its GRP1 wakeup never latched, and
 * once powered on again it kept running inside EL3 and never returned from
 * CPU_SUSPEND. The EL3 monitor is encrypted in the boot LUN, so the cause
 * is not known. BIG C2 never failed. See cpuidle-secondaries-q22b.md.
 */
static bool e9810_little_c2;
static atomic_t e9810_little_in_c2 = ATOMIC_INIT(0);

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

/*
 * The marker must be visible to CPU0 even if this core is powered off or
 * stops right after the store, hence the full DSB.
 */
static __always_inline void e9810_set_stage(u32 stage)
{
	WRITE_ONCE(*this_cpu_ptr(&e9810_stage), stage);
	dsb(ish);
}

static __cpuidle int e9810_enter_c2(struct cpuidle_device *dev,
				    struct cpuidle_driver *drv, int idx)
{
	struct e9810_cluster *cl = container_of(drv, struct e9810_cluster, drv);
	struct e9810_stats *st = this_cpu_ptr(&e9810_stats);
	unsigned int phy = e9810_phycpu[dev->cpu];
	u32 bit = INTR_GEN_BIT(phy);
	bool little = e9810_cluster_of[dev->cpu] == 0;
	int ret, lim;

	if (!cpumask_test_cpu(dev->cpu, &e9810_armed)) {
		st->unarmed++;
		goto wfi;
	}
	if (atomic_read(&e9810_deep_off))
		goto wfi;

	/* The slot is held until cal_cpu_enable() has run (see "out"). */
	if (little) {
		lim = READ_ONCE(e9810_little_limit);
		if (atomic_inc_return(&e9810_little_in_c2) > lim && lim) {
			atomic_dec(&e9810_little_in_c2);
			st->guarded++;
			goto wfi;
		}
	}

	/* A vetoing notifier has already been rolled back by cpu_pm. */
	if (cpu_pm_enter()) {
		if (little)
			atomic_dec(&e9810_little_in_c2);
		goto wfi;
	}

	/*
	 * cal_cpu_disable() enables this core's GRP2 interrupt first. A GRP2
	 * event latched earlier would reach the APM at once, while the core
	 * still runs here. The 4.9 core-on sequence clears it the same way
	 * (GRP2_INTR_BID_CLEAR, own bit); GRP2 is not enabled for this core
	 * yet, so the APM is not looking at it.
	 */
	if (readl_relaxed(e9810_intr_gen + INTR_GEN_GRP2_PEND) & bit) {
		writel(bit, e9810_intr_gen + INTR_GEN_GRP2_CLEAR);
		st->stale_grp2++;
	}

	e9810_set_stage(E9810_ST_CAL_OFF);
	ret = cal_cpu_disable(phy);
	if (ret) {
		e9810_set_stage(E9810_ST_CAL_ON);
		cal_cpu_enable(phy);
		if (little)
			atomic_dec(&e9810_little_in_c2);
		e9810_set_stage(E9810_ST_PM_EXIT);
		cpu_pm_exit();
		e9810_set_stage(E9810_ST_KERNEL);
		e9810_disable_deep("cal_cpu_disable failed");
		goto wfi;
	}

	st->c2_enter++;
	e9810_set_stage(E9810_ST_FIRMWARE);
	ret = psci_cpu_suspend_enter(cl->psci_state[idx]);
	e9810_set_stage(E9810_ST_CAL_ON);

	cal_cpu_enable(phy);
	if (little)
		atomic_dec(&e9810_little_in_c2);
	e9810_set_stage(E9810_ST_PM_EXIT);
	cpu_pm_exit();
	e9810_set_stage(E9810_ST_KERNEL);

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
		e9810_cpu_of_phy[phy] = cpu;
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
	if (e9810_cluster_of[cpu] == 0 && !e9810_little_c2)
		return "LITTLE C2 disabled (cores lost in EL3 in q22b/q40/q43; DT samsung,c2-little-enable)";
	if (!cpu_online(cpu))
		return "cpu offline";
	if (atomic_read(&e9810_deep_off))
		return "deep idle disabled";
	if (!exynos9810_mct_bc_ready())
		return "MCT tick broadcast not active";
	return NULL;
}

/*
 * 4.9 exynos_hotplug_in_callback() -> cpu_enable() -> cal_cpu_enable(): on
 * the CPU itself, since pmucal_powermode_hint_clear() picks the CPU_INFORM
 * slot from the live MPIDR. Clears CPU_INFORM, disables the core's GRP2
 * interrupt, clears a latched GRP2 event and the core's
 * PMU_EVENT_INTERRUPT_ENABLE_GRP2 bit. Never runs while the CPU is armed.
 */
static void e9810_reset_hooks_local(void *unused)
{
	unsigned int cpu = smp_processor_id();
	unsigned long flags;
	int ret;

	/* pmucal_cpu_lock is a plain spinlock also taken from idle. */
	local_irq_save(flags);
	ret = cal_cpu_enable(e9810_phycpu[cpu]);
	local_irq_restore(flags);
	this_cpu_write(e9810_hooks_ret, ret);
	this_cpu_inc(e9810_hooks_reset);
}

/* cpuhp AP online callback: runs on @cpu, like the 4.9 one. */
static int e9810_cpu_online(unsigned int cpu)
{
	if (!cpumask_test_cpu(cpu, &e9810_armed))
		e9810_reset_hooks_local(NULL);
	return 0;
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
		if (cpumask_test_cpu(cpu, &e9810_armed))
			continue;
		/* Not armed yet, so not in C2: the IPI is answered. */
		if (smp_call_function_single(cpu, e9810_reset_hooks_local,
					     NULL, 1)) {
			why = "C2 hook reset IPI failed";
			break;
		}
		if (per_cpu(e9810_hooks_ret, cpu)) {
			why = "C2 hook reset (cal_cpu_enable) failed";
			break;
		}
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
		/*
		 * cpuidle_pause() waits for an RCU grace period. A CPU lost
		 * outside an RCU idle section would block it forever, so
		 * only kick the idle CPUs then.
		 */
		if (!cpumask_empty(&e9810_lost)) {
			pr_emerg("restart with lost CPUs %*pbl: cpuidle_pause() skipped\n",
				 cpumask_pr_args(&e9810_lost));
			wake_up_all_idle_cpus();
		} else {
			cpuidle_pause();
		}
	}
	return NOTIFY_OK;
}

/*
 * CPUs leave C2 before nearly every restart notifier runs. One below
 * INT_MAX, so the watchdog's restart guard (INT_MAX, arms the final timeout)
 * always runs first, even if this one blocks.
 */
static struct notifier_block e9810_reboot_nb = {
	.notifier_call = e9810_reboot_cb,
	.priority = INT_MAX - 1,
};

static const char *e9810_setup(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node, *psci_np;
	struct resource *res;
	bool any = false;
	unsigned int c;
	const char *why;
	int ret;

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

	/*
	 * 4.9 resets the C2 hooks as each CPU comes online. The callback
	 * also runs now on every online CPU, before anything can be armed.
	 */
	ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "cpuidle/exynos9810:hooks",
				e9810_cpu_online, NULL);
	if (ret < 0)
		pr_warn("cpuhp C2 hook reset not registered (%d); reset only at arming\n",
			ret);

	psci_np = of_find_compatible_node(NULL, NULL, "arm,psci");
	if (psci_np) {
		const char *method;

		if (!of_property_read_string(psci_np, "method", &method) &&
		    !strcmp(method, "smc"))
			of_property_read_u32(psci_np, "cpu_on",
					     &e9810_psci_cpu_on_fn);
		of_node_put(psci_np);
	}

	register_reboot_notifier(&e9810_reboot_nb);
	return NULL;
}

/* ------------------------------------------------------------------------ */
/* DT-gated staged self-test                                                */
/* ------------------------------------------------------------------------ */

struct e9810_regs {
	u32 status, config, inform, event_grp2;
	u32 grp1_en, grp1_pend, grp2_en, grp2_pend;
	u32 stage;
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
	int little_limit;	/* cluster-0 C2 limit for this phase, 0 = none */
	bool busy0;		/* CPU0 busy-polls: never in WFI during the phase */
	bool bisect;		/* may lose a CPU; runs last */
	unsigned int ms;	/* armed time */
};

/*
 * Timeline of one cluster (all four cores) and the INTR_GEN group bits,
 * sampled by a kthread on a CPU outside the phase, one entry per change.
 * The ring is frozen when a phase fails, so the moments before the failure
 * survive. Printed when a phase fails.
 */
#define E9810_TL_ENTRIES		1024
#define E9810_TL_DUMP_MAX		700
#define E9810_TL_MARKS			64
#define E9810_TL_FIRST			128

struct e9810_tl_state {
	u32 status[4];
	u8 config[4], inform[4], stage[4];
	u8 grp1_en, grp1_pend, grp2_en, grp2_pend;
};

struct e9810_tl_entry {
	u64 t;
	struct e9810_tl_state s;
};

struct e9810_tl_mark {
	u64 t;
	const char *what;
	int arg;
};

struct e9810_tl {
	/* The first changes after arming are kept apart from the ring. */
	struct e9810_tl_entry first[E9810_TL_FIRST];
	struct e9810_tl_entry e[E9810_TL_ENTRIES];
	struct e9810_tl_mark m[E9810_TL_MARKS];	/* ring */
	unsigned int n, nm;
	unsigned int cluster, cpu;
	u64 samples, t0;
	struct task_struct *task;
	bool stop, exited, frozen;
};

static struct e9810_tl *e9810_tl;

static void e9810_tl_mark(const char *what, int arg)
{
	struct e9810_tl *tl = e9810_tl;
	struct e9810_tl_mark *m;

	if (!tl || !tl->task || READ_ONCE(tl->frozen))
		return;
	m = &tl->m[tl->nm % E9810_TL_MARKS];
	m->t = ktime_get_ns();
	m->what = what;
	m->arg = arg;
	tl->nm++;
}

static void e9810_tl_sample(const struct e9810_tl *tl,
			    struct e9810_tl_state *s)
{
	unsigned int i, phy, v;

	memset(s, 0, sizeof(*s));
	for (i = 0; i < 4; i++) {
		phy = tl->cluster * 4 + i;
		regmap_read(e9810_pmu, pmu_cpu_status[phy], &s->status[i]);
		regmap_read(e9810_pmu, PMU_CPU_CONFIG(phy), &v);
		s->config[i] = v;
		regmap_read(e9810_pmu, PMU_CPU_INFORM(phy), &v);
		s->inform[i] = v | ((v >> 16) ? 0x80 : 0);
		/* stage of the logical CPU on this core */
		s->stage[i] = READ_ONCE(per_cpu(e9810_stage,
						e9810_cpu_of_phy[phy]));
	}
	s->grp1_en = readl(e9810_intr_gen + INTR_GEN_GRP1_ENABLE);
	s->grp1_pend = readl(e9810_intr_gen + INTR_GEN_GRP1_PEND);
	s->grp2_en = readl(e9810_intr_gen + INTR_GEN_GRP2_ENABLE);
	s->grp2_pend = readl(e9810_intr_gen + INTR_GEN_GRP2_PEND);
}

static int e9810_tl_fn(void *data)
{
	struct e9810_tl *tl = data;
	struct e9810_tl_state cur, prev;
	bool first = true;

	while (!READ_ONCE(tl->stop) && !kthread_should_stop()) {
		u64 t = ktime_get_ns();

		if (READ_ONCE(tl->frozen)) {
			usleep_range(1000, 2000);
			continue;
		}
		e9810_tl_sample(tl, &cur);
		tl->samples++;
		if (first || memcmp(&cur, &prev, sizeof(cur))) {
			struct e9810_tl_entry *e = &tl->e[tl->n % E9810_TL_ENTRIES];

			e->t = t;
			e->s = cur;
			if (tl->n < E9810_TL_FIRST)
				tl->first[tl->n] = *e;
			prev = cur;
			first = false;
			smp_store_release(&tl->n, tl->n + 1);
		}
		cond_resched();
	}
	smp_store_release(&tl->exited, true);
	return 0;
}

/* Sample @cluster from @cpu, which must not be part of the phase. */
static void e9810_tl_start(unsigned int cluster, unsigned int cpu)
{
	struct e9810_tl *tl = e9810_tl;

	if (!tl || cpu >= nr_cpu_ids || !cpu_online(cpu))
		return;
	tl->n = 0;
	tl->nm = 0;
	tl->samples = 0;
	tl->stop = false;
	tl->exited = false;
	tl->frozen = false;
	tl->cluster = cluster;
	tl->cpu = cpu;
	tl->t0 = ktime_get_ns();
	tl->task = kthread_create(e9810_tl_fn, tl, "c2tl/%u", cpu);
	if (IS_ERR(tl->task)) {
		tl->task = NULL;
		return;
	}
	kthread_bind(tl->task, cpu);
	get_task_struct(tl->task);
	wake_up_process(tl->task);
}

/* Keep the ring as it is at the failure; marks stop too. */
static void e9810_tl_freeze(void)
{
	if (e9810_tl && e9810_tl->task)
		WRITE_ONCE(e9810_tl->frozen, true);
}

static void e9810_tl_stop(void)
{
	struct e9810_tl *tl = e9810_tl;
	u64 deadline = ktime_get_ns() + 200 * NSEC_PER_MSEC;

	if (!tl || !tl->task)
		return;
	WRITE_ONCE(tl->stop, true);
	while (!smp_load_acquire(&tl->exited) && ktime_get_ns() < deadline)
		msleep(2);
	if (smp_load_acquire(&tl->exited))
		kthread_stop(tl->task);
	else
		pr_err("cpuidle selftest: timeline thread did not stop; leaked\n");
	put_task_struct(tl->task);
	tl->task = NULL;
}

static void e9810_tl_print(const char *tag, unsigned int idx,
			   const struct e9810_tl_entry *e, u64 t0,
			   unsigned int c0)
{
	const struct e9810_tl_state *s = &e->s;

	pr_info("cpuidle selftest: %s#%u +%lluus c%u=%x/%x/%x/%u c%u=%x/%x/%x/%u c%u=%x/%x/%x/%u c%u=%x/%x/%x/%u g1e=%02x g1p=%02x g2e=%02x g2p=%02x\n",
		tag, idx, div_u64(e->t - t0, NSEC_PER_USEC),
		c0, s->status[0], s->config[0], s->inform[0], s->stage[0],
		c0 + 1, s->status[1], s->config[1], s->inform[1], s->stage[1],
		c0 + 2, s->status[2], s->config[2], s->inform[2], s->stage[2],
		c0 + 3, s->status[3], s->config[3], s->inform[3], s->stage[3],
		s->grp1_en, s->grp1_pend, s->grp2_en, s->grp2_pend);
}

/*
 * Times are microseconds after arming. "tl-first" = the first changes after
 * arming, "tl" = the last changes before the ring was frozen at the
 * failure (at most E9810_TL_DUMP_MAX). Entry numbers are global, so gaps
 * and overlaps are visible. cN = logical CPU N (cluster 0: cpu0-3, cluster
 * 1: cpu4-7, logical == physical on crownlte).
 */
static void e9810_tl_dump(const char *phase)
{
	struct e9810_tl *tl = e9810_tl;
	unsigned int n, i, first, c0, nm;

	if (!tl)
		return;
	n = smp_load_acquire(&tl->n);
	c0 = e9810_cpu_of_phy[tl->cluster * 4];
	first = n > E9810_TL_DUMP_MAX ? n - E9810_TL_DUMP_MAX : 0;
	pr_info("cpuidle selftest: timeline %s: cluster%u sampled on cpu%u samples=%llu changes=%u marks=%u frozen=%d (cN=status/config/inform[0x80=bit16]/stage 0 kernel 1 cal_off 2 firmware 3 cal_on 4 pm_exit; INTR_GEN bytes: LITTLE phyN = bit 4+N, BIG phyN = bit N-4)\n",
		phase, tl->cluster, tl->cpu, tl->samples, n, tl->nm, tl->frozen);
	nm = tl->nm;
	for (i = nm > E9810_TL_MARKS ? nm - E9810_TL_MARKS : 0; i < nm; i++) {
		const struct e9810_tl_mark *m = &tl->m[i % E9810_TL_MARKS];

		pr_info("cpuidle selftest: tl-mark#%u +%lluus %s %d\n", i,
			div_u64(m->t - tl->t0, NSEC_PER_USEC), m->what, m->arg);
	}
	for (i = 0; i < min_t(unsigned int, n, E9810_TL_FIRST); i++)
		e9810_tl_print("tl-first", i, &tl->first[i], tl->t0, c0);
	for (i = max(first, (unsigned int)E9810_TL_FIRST); i < n; i++)
		e9810_tl_print("tl", i, &tl->e[i % E9810_TL_ENTRIES], tl->t0, c0);
}

static void e9810_read_regs(unsigned int cpu, struct e9810_regs *r)
{
	unsigned int phy = e9810_phycpu[cpu];

	regmap_read(e9810_pmu, pmu_cpu_status[phy], &r->status);
	regmap_read(e9810_pmu, PMU_CPU_CONFIG(phy), &r->config);
	regmap_read(e9810_pmu, PMU_CPU_INFORM(phy), &r->inform);
	regmap_read(e9810_pmu, PMU_EVENT_INT_EN_GRP2, &r->event_grp2);
	r->grp1_en = readl(e9810_intr_gen + INTR_GEN_GRP1_ENABLE);
	r->grp1_pend = readl(e9810_intr_gen + INTR_GEN_GRP1_PEND);
	r->grp2_en = readl(e9810_intr_gen + INTR_GEN_GRP2_ENABLE);
	r->grp2_pend = readl(e9810_intr_gen + INTR_GEN_GRP2_PEND);
	r->stage = READ_ONCE(per_cpu(e9810_stage, cpu));
}

static const char *e9810_stage_str(u32 stage)
{
	return stage < ARRAY_SIZE(e9810_stage_name) ?
		e9810_stage_name[stage] : "?";
}

static void e9810_log_regs(const char *tag, unsigned int cpu,
			   const struct e9810_regs *r)
{
	u32 bit = INTR_GEN_BIT(e9810_phycpu[cpu]);

	pr_info("cpuidle selftest: %s cpu%u phy=%u stage=%s status=%#x config=%#x inform=%#x event_grp2=%#x(%d) grp1_en=%#x(%d) grp1_pend=%#x(%d) grp2_en=%#x(%d) grp2_pend=%#x(%d)\n",
		tag, cpu, e9810_phycpu[cpu], e9810_stage_str(r->stage),
		r->status, r->config, r->inform,
		r->event_grp2, !!(r->event_grp2 & bit),
		r->grp1_en, !!(r->grp1_en & bit), r->grp1_pend,
		!!(r->grp1_pend & bit), r->grp2_en, !!(r->grp2_en & bit),
		r->grp2_pend, !!(r->grp2_pend & bit));
}

static bool e9810_wait_answer(struct e9810_ping *p, unsigned int ms)
{
	u64 deadline = ktime_get_ns() + (u64)ms * NSEC_PER_MSEC;

	while (!smp_load_acquire(&p->t_run)) {
		if (ktime_get_ns() > deadline)
			return false;
		usleep_range(500, 1000);
	}
	return true;
}

#define PSCI_FN_VERSION			0x84000000
#define PSCI_FN64_AFFINITY_INFO		0xc4000004

struct e9810_smc_req {
	call_single_data_t csd;
	u32 fn;
	unsigned long a1, a2;
	long ret;
	bool done;
};

/* Runs on a healthy helper CPU, so CPU0 survives if EL3 blocks the call. */
static void e9810_smc_fn(void *info)
{
	struct e9810_smc_req *r = info;
	struct arm_smccc_res res;

	arm_smccc_smc(r->fn, r->a1, r->a2, 0, 0, 0, 0, 0, &res);
	r->ret = (long)(s32)res.a0;
	smp_store_release(&r->done, true);
}

/* First online CPU other than CPU0 that is not in @avoid, or nr_cpu_ids. */
static unsigned int e9810_pick_helper(const struct cpumask *avoid)
{
	unsigned int cpu;

	for_each_online_cpu(cpu)
		if (cpu && !(avoid && cpumask_test_cpu(cpu, avoid)))
			return cpu;
	return nr_cpu_ids;
}

/*
 * Raw SMC on @helper with a bounded wait. 0 and *ret on completion,
 * -ETIMEDOUT if the helper is still inside EL3 (the request is then leaked
 * on purpose), -EBUSY/-ENOMEM otherwise.
 */
static int e9810_smc_on(unsigned int helper, u32 fn, unsigned long a1,
			unsigned long a2, long *ret)
{
	struct e9810_smc_req *req;
	u64 t0;

	if (helper >= nr_cpu_ids)
		return -ENODEV;
	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;
	req->fn = fn;
	req->a1 = a1;
	req->a2 = a2;
	INIT_CSD(&req->csd, e9810_smc_fn, req);
	if (smp_call_function_single_async(helper, &req->csd)) {
		kfree(req);
		return -EBUSY;
	}
	t0 = ktime_get_ns();
	while (!smp_load_acquire(&req->done) &&
	       ktime_get_ns() - t0 < E9810_RECOVER_WAIT_MS * NSEC_PER_MSEC)
		usleep_range(200, 500);
	if (!smp_load_acquire(&req->done))
		return -ETIMEDOUT;
	*ret = req->ret;
	kfree(req);
	return 0;
}

/* PSCI AFFINITY_INFO (0 ON, 1 OFF, 2 ON_PENDING; <0 error), logged. */
static void e9810_log_affinity(const char *tag, unsigned int cpu,
			       unsigned int helper)
{
	long ret = 0;
	int err = e9810_smc_on(helper, PSCI_FN64_AFFINITY_INFO,
			       cpu_logical_map(cpu), 0, &ret);

	pr_info("cpuidle selftest: %s cpu%u PSCI AFFINITY_INFO=%ld%s\n", tag,
		cpu, err ? 0 : ret, err == -ETIMEDOUT ? " (helper timeout)" :
		err ? " (not issued)" : "");
}

/*
 * Try to bring back a CPU that does not answer IPIs after disarming.
 * Returns the step that brought it back, or NULL. Every step is logged.
 *  1. "ipi": another SGI (wake_up_if_idle() reschedule IPI) and a wait.
 *  2. "pmu-on": if the PMU reports the core off, request power-on the way
 *     4.9 exynos_cpu_up() and the APM core-on sequence do
 *     (CONFIGURATION[3:0] = 0xf). The core then runs the EL3 warm-boot path,
 *     which resumes it if EL3 still holds its CPU_SUSPEND state.
 *  3. "psci-cpu-on": only if the marker says the core is inside the
 *     firmware: PSCI CPU_ON(mpidr, cpu_resume) from a helper CPU. If EL3
 *     still treats the core as suspended or on, it refuses (ALREADY_ON or
 *     ON_PENDING) and nothing happens. If EL3 has it as off, the core
 *     starts in cpu_resume, which restores the context this core saved in
 *     __cpu_suspend_enter() before CPU_SUSPEND and returns from that call.
 *  4. "pmu-cycle": only if CPU_ON was accepted (0) and the core is powered:
 *     power it down and up through CONFIGURATION, see below.
 * Steps 2-4 cannot help a core that is stuck in kernel code (marker
 * cal_cpu_disable / cal_cpu_enable / cpu_pm_exit); that is reported.
 */
static const char *e9810_recover(unsigned int cpu, struct e9810_ping *p,
				 const struct cpumask *unhealthy)
{
	unsigned int phy = e9810_phycpu[cpu], helper;
	struct e9810_regs r;
	long ret = 0;
	u32 st = 0;
	int err;
	u64 t0;

	pr_warn("cpuidle selftest: recover cpu%u: step ipi\n", cpu);
	wake_up_if_idle(cpu);
	if (e9810_wait_answer(p, E9810_PING_TIMEOUT_MS))
		return "ipi";

	e9810_read_regs(cpu, &r);
	e9810_log_regs("recover-ipi", cpu, &r);
	if (r.stage != E9810_ST_FIRMWARE) {
		pr_err("cpuidle selftest: recover cpu%u: stuck in kernel code at stage=%s (pmucal_cpu_lock %s); PMU/PSCI steps cannot help\n",
		       cpu, e9810_stage_str(r.stage),
		       cal_cpu_lock_is_held() ? "HELD" : "free");
		return NULL;
	}

	if ((r.status & PMU_CPU_STATUS_ON) != PMU_CPU_STATUS_ON) {
		pr_warn("cpuidle selftest: recover cpu%u: step pmu-on (config %#x -> LOCAL_PWR_CFG=0xf)\n",
			cpu, r.config);
		regmap_update_bits(e9810_pmu, PMU_CPU_CONFIG(phy),
				   PMU_CPU_LOCAL_PWR_CFG, PMU_CPU_LOCAL_PWR_CFG);
		t0 = ktime_get_ns();
		do {
			regmap_read(e9810_pmu, pmu_cpu_status[phy], &st);
			if ((st & PMU_CPU_STATUS_ON) == PMU_CPU_STATUS_ON)
				break;
			usleep_range(100, 200);
		} while (ktime_get_ns() - t0 < 20 * NSEC_PER_MSEC);
		pr_warn("cpuidle selftest: recover cpu%u: pmu-on status=%#x after %llu us\n",
			cpu, st, div_u64(ktime_get_ns() - t0, NSEC_PER_USEC));
		wake_up_if_idle(cpu);
		if (e9810_wait_answer(p, E9810_RECOVER_WAIT_MS))
			return "pmu-on";
		e9810_read_regs(cpu, &r);
		e9810_log_regs("recover-pmu-on", cpu, &r);
	}

	if (!e9810_psci_cpu_on_fn) {
		pr_err("cpuidle selftest: recover cpu%u: no PSCI CPU_ON function ID\n",
		       cpu);
		return NULL;
	}
	helper = e9810_pick_helper(unhealthy);
	if (helper >= nr_cpu_ids) {
		pr_err("cpuidle selftest: recover cpu%u: no healthy helper CPU for PSCI CPU_ON\n",
		       cpu);
		return NULL;
	}
	/* What EL3 thinks of the lost core and of a running one. */
	e9810_log_affinity("recover", cpu, helper);
	e9810_log_affinity("reference", helper, helper);

	pr_warn("cpuidle selftest: recover cpu%u: step psci-cpu-on fn=%#x mpidr=%#llx entry=cpu_resume@%#lx via cpu%u\n",
		cpu, e9810_psci_cpu_on_fn, cpu_logical_map(cpu),
		(unsigned long)__pa_symbol(cpu_resume), helper);
	err = e9810_smc_on(helper, e9810_psci_cpu_on_fn, cpu_logical_map(cpu),
			   __pa_symbol(cpu_resume), &ret);
	if (err) {
		pr_err("cpuidle selftest: recover cpu%u: PSCI CPU_ON not completed via cpu%u (%d)\n",
		       cpu, helper, err);
		return NULL;
	}
	/* PSCI: 0 ok, -4 ALREADY_ON, -5 ON_PENDING, -2 INVALID_PARAMS */
	pr_warn("cpuidle selftest: recover cpu%u: PSCI CPU_ON returned %ld\n",
		cpu, ret);
	wake_up_if_idle(cpu);
	if (e9810_wait_answer(p, E9810_RECOVER_WAIT_MS))
		return "psci-cpu-on";
	e9810_read_regs(cpu, &r);
	e9810_log_regs("recover-psci", cpu, &r);
	e9810_log_affinity("after-cpu-on", cpu, helper);

	/*
	 * 4. "pmu-cycle": q40 had the core powered on, CPU_ON accepted (0, so
	 * EL3 holds it as OFF) and still nothing. That fits a core parked in
	 * EL3 (WFI/WFE) waiting for a power-on reset that never comes because
	 * it is already powered. Only after CPU_ON was accepted: disarm its
	 * GRP2 event (cal_cpu_enable(phy) from CPU0, which also rewrites
	 * CPU0's own INFORM slot to 0, its normal value), request power-down
	 * (CONFIGURATION[3:0] = 0, completes only if the core sits in WFI),
	 * then power-up. The reset goes through EL3 warm boot with the CPU_ON
	 * entry point recorded.
	 */
	if (ret || (r.status & PMU_CPU_STATUS_ON) != PMU_CPU_STATUS_ON)
		return NULL;
	pr_warn("cpuidle selftest: recover cpu%u: step pmu-cycle\n", cpu);
	{
		unsigned long flags;

		local_irq_save(flags);
		cal_cpu_enable(phy);
		local_irq_restore(flags);
	}
	regmap_update_bits(e9810_pmu, PMU_CPU_CONFIG(phy),
			   PMU_CPU_LOCAL_PWR_CFG, 0);
	t0 = ktime_get_ns();
	do {
		regmap_read(e9810_pmu, pmu_cpu_status[phy], &st);
		if (!(st & PMU_CPU_STATUS_ON))
			break;
		usleep_range(50, 100);
	} while (ktime_get_ns() - t0 < 5 * NSEC_PER_MSEC);
	pr_warn("cpuidle selftest: recover cpu%u: pmu-cycle off: status=%#x after %llu us%s\n",
		cpu, st, div_u64(ktime_get_ns() - t0, NSEC_PER_USEC),
		(st & PMU_CPU_STATUS_ON) ? " (did not power down: core not in WFI)" : "");
	regmap_update_bits(e9810_pmu, PMU_CPU_CONFIG(phy),
			   PMU_CPU_LOCAL_PWR_CFG, PMU_CPU_LOCAL_PWR_CFG);
	t0 = ktime_get_ns();
	do {
		regmap_read(e9810_pmu, pmu_cpu_status[phy], &st);
		if ((st & PMU_CPU_STATUS_ON) == PMU_CPU_STATUS_ON)
			break;
		usleep_range(100, 200);
	} while (ktime_get_ns() - t0 < 20 * NSEC_PER_MSEC);
	pr_warn("cpuidle selftest: recover cpu%u: pmu-cycle on: status=%#x after %llu us\n",
		cpu, st, div_u64(ktime_get_ns() - t0, NSEC_PER_USEC));
	wake_up_if_idle(cpu);
	if (e9810_wait_answer(p, E9810_RECOVER_WAIT_MS))
		return "pmu-cycle";
	e9810_read_regs(cpu, &r);
	e9810_log_regs("recover-pmu-cycle", cpu, &r);
	return NULL;
}

/* Repeated so the last lines of the console (pmsg) still carry it. */
static void e9810_lost_remind(struct work_struct *work)
{
	unsigned int cpu;

	for_each_cpu(cpu, &e9810_lost)
		pr_emerg("cpuidle: CPU-LOST cpu%u never came back from C2 (stage=%s); deep idle is off; a restart may hang until the CL0 watchdog resets the SoC\n",
			 cpu, e9810_stage_str(READ_ONCE(per_cpu(e9810_stage,
								cpu))));
	if (++e9810_lost_reminders < E9810_LOST_REMINDERS)
		queue_delayed_work_on(0, system_long_wq, &e9810_lost_work,
				      msecs_to_jiffies(E9810_LOST_REMINDER_MS));
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
		pr_info("cpuidle selftest: cpu%u reg=%#llx live_mpidr=%#llx phycpu=%u cluster=%u hooks_reset=%u ret=%d\n",
			cpu, cpu_logical_map(cpu), e9810_live_mpidr[cpu],
			e9810_phycpu[cpu], e9810_cluster_of[cpu],
			per_cpu(e9810_hooks_reset, cpu),
			per_cpu(e9810_hooks_ret, cpu));
	pr_info("cpuidle selftest: psci cpu_on fn=%#x\n", e9810_psci_cpu_on_fn);

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
	struct cpumask unanswered;
	const char *why = NULL;
	u64 t_end;

	*failed_cpu = UINT_MAX;
	*entered_all = false;
	cpumask_clear(&unanswered);

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

	WRITE_ONCE(e9810_little_limit, ph->little_limit);
	{
		/* Sample the phase's cluster from the other cluster. */
		unsigned int cl = e9810_cluster_of[cpumask_first(&ph->cpus)];
		unsigned int on = cl ? 1 : 7;

		if (!cpumask_test_cpu(on, &ph->cpus))
			e9810_tl_start(cl, on);
	}
	e9810_tl_mark("arm", ph->little_limit);
	why = e9810_arm(&ph->cpus);
	if (why) {
		pr_err("cpuidle selftest: phase %s: cannot arm %*pbl: %s\n",
		       ph->name, cpumask_pr_args(&ph->cpus), why);
		e9810_tl_stop();
		WRITE_ONCE(e9810_little_limit, e9810_little_limit_cfg);
		goto stop_sleepers;
	}
	pr_info("cpuidle selftest: phase %s: armed %*pbl for %u ms little_limit=%d cpu0=%s timeline=%s%d%s\n",
		ph->name, cpumask_pr_args(&ph->cpus), ph->ms,
		ph->little_limit, ph->busy0 ? "busy" : "idle",
		e9810_tl && e9810_tl->task ? "cpu" : "off",
		e9810_tl && e9810_tl->task ? (int)e9810_tl->cpu : -1,
		ph->bisect ? " (bisect phase)" : "");

	t_end = ktime_get_ns() + (u64)ph->ms * NSEC_PER_MSEC;
	while (!why && ktime_get_ns() < t_end) {
		u64 deadline;

		if (ph->busy0) {
			/* CPU0 must not reach WFI: no sleeping at all. */
			deadline = ktime_get_ns() +
				   (u64)E9810_POLL_MS * NSEC_PER_MSEC;
			/* cond_resched() runs other work, never idle. */
			while (ktime_get_ns() < deadline) {
				cond_resched();
				cpu_relax();
			}
		} else {
			msleep(E9810_POLL_MS);
		}

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
		e9810_tl_mark("ping", samples);

		deadline = ktime_get_ns() +
			   (u64)E9810_PING_TIMEOUT_MS * NSEC_PER_MSEC;
		for (;;) {
			bool pending = false;

			for_each_cpu(cpu, &ph->cpus)
				if (!smp_load_acquire(&pings[cpu].t_run))
					pending = true;
			if (!pending || ktime_get_ns() > deadline)
				break;
			if (ph->busy0)
				udelay(50);
			else
				usleep_range(50, 100);
		}

		/* Every CPU is checked, so all unanswered CPUs are named. */
		for_each_cpu(cpu, &ph->cpus) {
			struct e9810_ping *p = &pings[cpu];
			u64 t_run = smp_load_acquire(&p->t_run), lat;
			u64 hb = smp_load_acquire(&sl[cpu].heartbeat);

			if (!t_run) {
				cpumask_set_cpu(cpu, &unanswered);
				if (!why) {
					*failed_cpu = cpu;
					why = "IPI not answered within 100 ms";
				}
				continue;
			}
			lat = t_run > p->t_issue ? t_run - p->t_issue : 0;
			p->answered++;
			p->lat_sum += lat;
			p->lat_min = min(p->lat_min, lat);
			p->lat_max = max(p->lat_max, lat);

			if (!why && ktime_get_ns() - hb >
			    (u64)E9810_SLEEPER_STALE_MS * NSEC_PER_MSEC) {
				*failed_cpu = cpu;
				why = "timer wakeup (broadcast) late by more than 250 ms";
			}
			/* An armed GRP2 event would let the APM cut a running core. */
			if (!why && p->armed_seen) {
				*failed_cpu = cpu;
				why = "C2 hooks found armed while the core was running";
			}
		}
	}

	if (why) {
		struct cpumask lost;
		struct e9810_regs r;

		/* Capture the stuck state before disarming changes it. */
		if (!cpumask_test_cpu(*failed_cpu, &unanswered)) {
			e9810_read_regs(*failed_cpu, &r);
			e9810_log_regs("FAIL-state", *failed_cpu, &r);
		}
		for_each_cpu(cpu, &unanswered) {
			e9810_read_regs(cpu, &r);
			e9810_log_regs("FAIL-state", cpu, &r);
		}
		e9810_tl_mark("fail", *failed_cpu);
		/* Keep the moments before the failure in the ring. */
		e9810_tl_freeze();
		pr_err("cpuidle selftest: phase %s: FAIL cpu%u: %s; unanswered=%*pbl pmucal_cpu_lock=%s little_in_c2=%d\n",
		       ph->name, *failed_cpu, why, cpumask_pr_args(&unanswered),
		       cal_cpu_lock_is_held() ? "HELD" : "free",
		       atomic_read(&e9810_little_in_c2));
		e9810_disable_deep("selftest phase failed");
		e9810_disarm_all();
		e9810_tl_mark("disarmed", 0);
		msleep(E9810_RECHECK_MS);

		cpumask_clear(&lost);
		for_each_cpu(cpu, &unanswered) {
			e9810_read_regs(cpu, &r);
			e9810_log_regs("after-disarm", cpu, &r);
			if (smp_load_acquire(&pings[cpu].t_run))
				pr_warn("cpuidle selftest: cpu%u answered after disarm\n",
					cpu);
			else
				cpumask_set_cpu(cpu, &lost);
		}
		for_each_cpu(cpu, &lost) {
			const char *how;

			e9810_tl_mark("recover", cpu);
			how = e9810_recover(cpu, &pings[cpu], &unanswered);

			if (how) {
				pr_warn("cpuidle selftest: RECOVERED cpu%u via %s (stage now %s)\n",
					cpu, how,
					e9810_stage_str(READ_ONCE(per_cpu(e9810_stage,
									  cpu))));
				cpumask_clear_cpu(cpu, &lost);
				continue;
			}
			e9810_read_regs(cpu, &r);
			e9810_log_regs("LOST", cpu, &r);
			pr_emerg("cpuidle selftest: CPU-LOST cpu%u NOT RECOVERED (ipi, pmu-on, psci-cpu-on tried as applicable); stage=%s pmucal_cpu_lock=%s; deep idle stays off\n",
				 cpu, e9810_stage_str(r.stage),
				 cal_cpu_lock_is_held() ? "HELD" : "free");
		}
		if (!cpumask_empty(&lost)) {
			cpumask_or(&e9810_lost, &e9810_lost, &lost);
			e9810_lost_reminders = 0;
			queue_delayed_work_on(0, system_long_wq,
					      &e9810_lost_work,
					      msecs_to_jiffies(E9810_LOST_REMINDER_MS));
		}
		pr_err("cpuidle selftest: phase %s: FAIL cpu%u: %s; unanswered=%*pbl lost=%*pbl\n",
		       ph->name, *failed_cpu, why,
		       cpumask_pr_args(&unanswered), cpumask_pr_args(&lost));
		e9810_tl_stop();
		e9810_tl_dump(ph->name);
	} else if (!e9810_keep_armed) {
		e9810_disarm_all();
	}
	e9810_tl_stop();
	WRITE_ONCE(e9810_little_limit, e9810_little_limit_cfg);

	e9810_snapshot(b);
	exynos9810_mct_bc_get_stats(&bc1);

	*entered_all = true;
	for_each_cpu(cpu, &ph->cpus) {
		const struct e9810_stats *sa = &a[cpu].st, *sb = &b[cpu].st;
		const struct e9810_ping *p = &pings[cpu];

		if (sb->c2_resumed == sa->c2_resumed)
			*entered_all = false;
		/* last_err is cumulative: only shown if it changed this phase. */
		pr_info("cpuidle selftest: phase %s cpu%u c2_enter=+%llu resumed=+%llu early=+%llu psci_fail=+%llu last_err=%d stale_grp2=+%llu stage=%s guarded=+%llu unarmed=+%llu wfi_fallback=+%llu | usage=+%llu time_ms=+%llu rejected=+%llu above=+%llu below=+%llu\n",
			ph->name, cpu, sb->c2_enter - sa->c2_enter,
			sb->c2_resumed - sa->c2_resumed, sb->early - sa->early,
			sb->psci_fail - sa->psci_fail,
			(sb->psci_fail != sa->psci_fail ||
			 sb->early != sa->early) ? sb->last_err : 0,
			sb->stale_grp2 - sa->stale_grp2,
			e9810_stage_str(READ_ONCE(per_cpu(e9810_stage, cpu))),
			sb->guarded - sa->guarded,
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

static void e9810_add_phase(struct e9810_phase *ph, const char *name,
			    u32 cpus, int little_limit, bool busy0,
			    bool bisect, unsigned int ms)
{
	ph->name = name;
	ph->ms = ms ? ms : E9810_PHASE_MS;
	cpumask_clear(&ph->cpus);
	bitmap_from_arr32(cpumask_bits(&ph->cpus), &cpus, 32);
	ph->little_limit = little_limit;
	ph->busy0 = busy0;
	ph->bisect = bisect;
}

static u32 e9810_soak_ms = E9810_SOAK_MS_DEFAULT;

static void e9810_selftest(struct work_struct *work)
{
	struct e9810_phase phases[11];
	struct e9810_snap *a, *b;
	struct e9810_ping *pings;
	struct e9810_sleeper *sl;
	const char *result = "PASS", *why = NULL, *bisect_fail = NULL;
	unsigned int cpu, i, nphases = 0, passed = 0, failed_cpu = UINT_MAX;
	int lim = e9810_little_limit_cfg ? : E9810_LITTLE_LIMIT_DEFAULT;
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
	/*
	 * What the write-1-to-clear registers read back. PMUCAL clears them
	 * with a read-modify-write; if they read back the pending bits, that
	 * write would also clear other cores' events.
	 */
	if (e9810_intr_gen)
		pr_info("cpuidle selftest: clear-register readback grp1_clr=%#x (grp1_pend=%#x) grp2_clr=%#x (grp2_pend=%#x)\n",
			readl(e9810_intr_gen + INTR_GEN_GRP1_CLEAR),
			readl(e9810_intr_gen + INTR_GEN_GRP1_PEND),
			readl(e9810_intr_gen + INTR_GEN_GRP2_CLEAR),
			readl(e9810_intr_gen + INTR_GEN_GRP2_PEND));

	if (raw_smp_processor_id() != 0)
		pr_warn("cpuidle selftest: running on cpu%d, expected cpu0\n",
			raw_smp_processor_id());

	if (!e9810_tl)
		e9810_tl = kzalloc(sizeof(*e9810_tl), GFP_KERNEL);
	{
		long ver = 0;
		int err = e9810_smc_on(e9810_pick_helper(NULL), PSCI_FN_VERSION,
				       0, 0, &ver);

		pr_info("cpuidle selftest: PSCI_VERSION=%#lx%s little_limit(default)=%d\n",
			err ? 0 : ver, err ? " (not issued/timeout)" : "",
			e9810_little_limit_cfg);
	}

	/*
	 * q44: BIG-only C2. LITTLE C2 lost a core in q22b, q40 and q43 and is
	 * refused at arm time unless DT enables it; that refusal is checked
	 * first. Then a ramp over the BIG cluster and a 10 s soak with all four
	 * BIG cores. Overlaps the cpufreq (100 s) and devfreq/TMU (105 s)
	 * self-tests on purpose: BIG frequency changes while BIG cores power
	 * down are part of normal operation. CPU0 is never armed.
	 */
	{
		struct cpumask m;
		const char *w;

		cpumask_clear(&m);
		cpumask_set_cpu(1, &m);
		w = e9810_arm(&m);
		if (w) {
			pr_info("cpuidle selftest: LITTLE arm check: refused (%s)\n", w);
		} else {
			pr_err("cpuidle selftest: LITTLE arm check: cpu1 was ARMED; disarming\n");
			e9810_disarm_all();
		}
	}
	nphases = 0;
	e9810_add_phase(&phases[nphases++], "big-1", 0x10, lim, false, false, 1000);
	e9810_add_phase(&phases[nphases++], "big-2", 0x30, lim, false, false, 1000);
	e9810_add_phase(&phases[nphases++], "big-3", 0x70, lim, false, false, 1000);
	e9810_add_phase(&phases[nphases++], "big-4", 0xf0, lim, false, false, 2000);
	e9810_add_phase(&phases[nphases++], "big-4-soak", 0xf0, lim, false, false,
			e9810_soak_ms);

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
			if (failed_cpu == UINT_MAX)
				result = "SKIPPED";
			else
				result = phases[i].bisect ?
					 "FAIL-IN-BISECT (limited phases passed)" :
					 "FAIL";
			bisect_fail = phases[i].name;
			break;
		}
		if (!entered_all && phases[i].little_limit &&
		    cpumask_weight(&phases[i].cpus) > phases[i].little_limit) {
			/* The limit may starve one core for a whole phase. */
			pr_warn("cpuidle selftest: phase %s: a CPU never got a C2 slot (limit %d); continuing\n",
				phases[i].name, phases[i].little_limit);
		} else if (!entered_all) {
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

	pr_info("cpuidle selftest: done result=%s phases_passed=%u/%u%s%s%s%s deep_off=%d armed=%*pbl lost=%*pbl little_limit=%d elapsed_ms=%llu\n",
		result, passed, nphases, why ? " why=" : "", why ? why : "",
		bisect_fail ? " failed_phase=" : "",
		bisect_fail ? bisect_fail : "",
		atomic_read(&e9810_deep_off), cpumask_pr_args(&e9810_armed),
		cpumask_pr_args(&e9810_lost), READ_ONCE(e9810_little_limit),
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
	u32 lim;

	INIT_DELAYED_WORK(&e9810_lost_work, e9810_lost_remind);
	/* Before anything can be armed. */
	if (!of_property_read_u32(np, "samsung,c2-little-limit", &lim) &&
	    lim <= 4) {
		e9810_little_limit_cfg = lim;
		WRITE_ONCE(e9810_little_limit, lim);
	}
	e9810_little_c2 = of_property_read_bool(np, "samsung,c2-little-enable");
	pr_info("LITTLE C2 %s, cluster-0 C2 limit %d%s\n",
		e9810_little_c2 ? "enabled" : "disabled (WFI only)",
		e9810_little_limit_cfg,
		e9810_little_limit_cfg ? "" : " (none)");
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
	if (!of_property_read_u32(np, "samsung,cpuidle-selftest-soak-ms",
				  &e9810_soak_ms))
		e9810_soak_ms = clamp_t(u32, e9810_soak_ms, E9810_SOAK_MS_MIN,
					E9810_SOAK_MS_DEFAULT);
	if (of_property_read_bool(np, "samsung,cpuidle-selftest")) {
		unsigned long now = jiffies - INITIAL_JIFFIES;
		unsigned long at = msecs_to_jiffies(E9810_SELFTEST_MS);

		INIT_DELAYED_WORK(&e9810_selftest_work, e9810_selftest);
		pr_info("cpuidle selftest: scheduled at %u ms, big-4-soak %u ms\n",
			E9810_SELFTEST_MS, e9810_soak_ms);
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
