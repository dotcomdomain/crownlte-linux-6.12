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
 *
 * Self-recovery chain (S0, roadmap/c2-runtime-design.md 4.0). Runs whenever
 * the driver is set up, armed or not:
 *  - Lost-core detector: every 2 s a work on CPU0 pings (async csd) each CPU
 *    that has sat in the firmware stage for more than 1 s, and once more
 *    with a reschedule IPI. Lost: a delivered ping unanswered for 2 x 200 ms
 *    and the CPU still online in the same firmware entry (per-CPU entry
 *    generation). Then a "C2-LOST" dump, the quarantine record, everything
 *    disarmed, the watchdog's one shared final stop
 *    (s3c2410wdt_crownlte_stop_petting(): never reloaded again, reset
 *    within its timeout), then orderly_reboot().
 *  - Quarantine record (v4; v3 slots are still read): one 4 KB no-map reserved-memory page (DT
 *    memory-region of this node) that survives a watchdog reset, two
 *    alternating sequenced slots, trusted only when both are valid and
 *    chained. Fail closed: no trusted record, an armed session that never
 *    reached the last controlled point of a restart/halt/power-off
 *    (watchdog reset, panic, power loss, kexec, a hang in device shutdown)
 *    or a big loss keeps all C2 off; a LITTLE loss keeps LITTLE C2 off;
 *    until an explicit clear (root c2_quarantine_clear, or a trial token in
 *    the DT above the record's trial floor on a trusted record). "Session
 *    armed" is persisted before the first CPU is armed. At shutdown the
 *    reboot notifier closes the arm gate for good and requires every CPU to
 *    acknowledge leaving C2 (session SHUTDOWN, still unclean); the restart
 *    handler or syscore shutdown writes IDLE last.
 *    Without the page, boot arming and drills are refused.
 *  - Drills (diag DTBs only, once: consumed in the record before they run,
 *    root c2_drill_rearm re-arms): samsung,c2-lost-drill = <cpu ms>
 *    simulates a loss of <cpu> at <ms> boot time and waits for the watchdog
 *    reset; samsung,c2-loss-drill-ms arms cpu1-3 (the known EL3 failure)
 *    for that long.
 * Boot arming (S1): samsung,c2-boot-cpus is applied per CPU from a work at
 * samsung,c2-arm-delay-ms boot time. CPUs waiting for the MCT broadcast
 * device (the cpuidle probe runs first, link order) or offline are retried
 * every second for up to 60 s; an offline one is then armed when it comes
 * online.
 * S2 (samsung,c2-little-local-timer): LITTLE C2 only on CPUs whose tick runs
 * on their own MCT local timer, as on 4.9 (Documentation/crownlte/c2-s2.md).
 * The takeover of those timers is part of the trial: the boot arming work
 * persists a local-timer session (LT_ARMED) before it asks exynos9810-mct-bc
 * to take over the LITTLE CPUs it arms (never CPU0 or the big cores), and
 * only the last controlled point of a clean shutdown closes it. An unclosed
 * local-timer session or a recorded local timer fault (E9810_QF_LTIMER)
 * means no takeover and no LITTLE C2 on later boots, until a clear.
 */

#define pr_fmt(fmt) "exynos9810-cpuidle: " fmt

#include <linux/arm-smccc.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/cpuhotplug.h>
#include <linux/cpuidle.h>
#include <linux/cpumask.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/kexec.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/panic.h>
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
#include <linux/syscore_ops.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include <uapi/linux/psci.h>

#include <asm/barrier.h>
#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/suspend.h>

#include <soc/samsung/cal-if.h>
#include <soc/samsung/crownlte-wdt.h>
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
	u64 no_ltimer;		/* LITTLE C2 refused: tick not on the MCT local timer */
	int last_err;
	unsigned int fail_streak;
};

static struct e9810_cluster e9810_clusters[E9810_NR_CLUSTERS];
static DEFINE_PER_CPU(struct e9810_stats, e9810_stats);
static DEFINE_PER_CPU(u32, e9810_stage);
/* jiffies when the CPU last entered the firmware stage (lost-core detector) */
static DEFINE_PER_CPU(unsigned long, e9810_fw_jiffies);
/* Firmware entries of the CPU so far: tells one C2 period from the next. */
static DEFINE_PER_CPU(u32, e9810_fw_gen);
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
/*
 * Set by the reboot notifier, under e9810_arm_lock, before it clears the arm
 * mask; never cleared. Every arming path and the C2 entry check it.
 * reboot.c runs the reboot notifiers before it sets system_state, so
 * system_state alone cannot stop an arming work that runs right after the
 * notifier.
 */
static bool e9810_shutdown;
/*
 * 1 from before e9810_enter_c2() reads the arm gate until the CPU is back
 * in the kernel stage. The shutdown drain closes the gate, then waits until
 * it has seen 0 on every CPU (see e9810_drain()).
 */
static DEFINE_PER_CPU(u32, e9810_c2_busy);
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
static bool e9810_little_c2_cfg;	/* DT value, restored after the loss drill */
static atomic_t e9810_little_in_c2 = ATOMIC_INIT(0);

/*
 * Stage S2 (DT samsung,c2-little-local-timer): LITTLE C2 only on a CPU whose
 * tick runs on its own MCT local timer (exynos9810-mct-bc, DT
 * samsung,mct-local-timers), as on 4.9. Its C2 periods are then ended by
 * that timer's own SPI, never by an IPI_TIMER from the tick broadcast,
 * which wakes several LITTLE cores with one SGI (the suspected trigger of
 * the 3-core EL3 loss, roadmap/c2-runtime-design.md 2.4 H1). Checked at
 * arming and again at every LITTLE C2 entry (no_ltimer: WFI instead).
 */
static bool e9810_little_ltimer;

/*
 * Quarantine in force this boot, from the record an earlier boot wrote
 * (E9810_QF_*). Set at probe before anything can be armed; fail closed
 * (E9810_QF_UNINIT) until a valid record is in place.
 */
#define E9810_QF_LITTLE			BIT(0)	/* a LITTLE core was lost */
#define E9810_QF_BIG			BIT(1)	/* a big core was lost */
#define E9810_QF_DRILL			BIT(2)	/* recorded by a drill */
#define E9810_QF_UNCLEAN		BIT(3)	/* armed session, no clean restart */
#define E9810_QF_UNINIT			BIT(4)	/* no valid record, never cleared */
/*
 * Record v4: an MCT local timer fault was recorded, or a local-timer
 * session (LT_ARMED/LT_SHUTDOWN) was not closed. No local timer takeover,
 * LITTLE C2 off.
 */
#define E9810_QF_LTIMER			BIT(5)
/* Flags that keep every CPU out of C2. */
#define E9810_QF_ALL_OFF	(E9810_QF_BIG | E9810_QF_UNCLEAN | E9810_QF_UNINIT)
/* Flags that keep the LITTLE cluster out of C2 (and its timers on the arch timer). */
#define E9810_QF_LITTLE_OFF	(E9810_QF_LITTLE | E9810_QF_LTIMER)
static u32 e9810_quarantine = E9810_QF_UNINIT;

/*
 * Local-timer trial state of this boot. e9810_lt_busy: the takeover is
 * running (the final point does not close the session meanwhile).
 * e9810_lt_faulted: a local timer fault was reported (LITTLE C2 off for the
 * rest of the boot). e9810_lt_block_close: the fault could not be recorded,
 * or the fatal path was taken: the session is never closed on this boot.
 */
static bool e9810_lt_tried;
static bool e9810_lt_busy;
static bool e9810_lt_faulted;
static atomic_t e9810_lt_block_close = ATOMIC_INIT(0);

/* Compared by address: the refusals the boot arming retries. */
static const char e9810_why_no_mct[] = "MCT tick broadcast not active";
static const char e9810_why_offline[] = "cpu offline";

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

	/*
	 * Busy before the gate is read, with a full barrier in between. The
	 * drain closes the gate, issues the same barrier and then reads busy
	 * (store-buffering pattern): either this CPU sees the gate closed, or
	 * the drain sees it busy and waits until it is out again.
	 */
	this_cpu_write(e9810_c2_busy, 1);
	smp_mb();
	if (!cpumask_test_cpu(dev->cpu, &e9810_armed) ||
	    READ_ONCE(e9810_shutdown)) {
		st->unarmed++;
		goto wfi;
	}
	if (atomic_read(&e9810_deep_off))
		goto wfi;
	if (little && READ_ONCE(e9810_little_ltimer) &&
	    !exynos9810_mct_local_active(dev->cpu)) {
		st->no_ltimer++;
		goto wfi;
	}

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
	/*
	 * A new firmware-entry generation, then the time, both visible before
	 * the stage (the detector reads the stage first, then these).
	 */
	this_cpu_write(e9810_fw_gen, this_cpu_read(e9810_fw_gen) + 1);
	this_cpu_write(e9810_fw_jiffies, jiffies);
	smp_wmb();
	e9810_set_stage(E9810_ST_FIRMWARE);
	ret = psci_cpu_suspend_enter(cl->psci_state[idx]);
	e9810_set_stage(E9810_ST_CAL_ON);

	cal_cpu_enable(phy);
	if (little)
		atomic_dec(&e9810_little_in_c2);
	e9810_set_stage(E9810_ST_PM_EXIT);
	cpu_pm_exit();
	e9810_set_stage(E9810_ST_KERNEL);
	/* The positive "out of C2" acknowledgement the shutdown drain waits for. */
	smp_store_release(this_cpu_ptr(&e9810_c2_busy), 0);

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
	/* Not (or no longer) on the C2 path: the WFI below is not C2. */
	smp_store_release(this_cpu_ptr(&e9810_c2_busy), 0);
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

static const char *e9810_qm_session_arm(void);
static void e9810_qm_session_drained(const char *how);
static void e9810_qm_session_final(const char *how, bool restart);
static void e9810_boot_arm_online(unsigned int cpu);

/* Why @cpu cannot be armed, or NULL. Called with e9810_arm_lock held. */
static const char *e9810_arm_refused(unsigned int cpu)
{
	if (e9810_setup_why)
		return e9810_setup_why;
	/* CPU0 owns every SPI, the MCT broadcast and the C2-LOST detector. */
	if (cpu == 0)
		return "CPU0 is never armed";
	if (!e9810_clusters[e9810_cluster_of[cpu]].registered)
		return "cluster not registered";
	/*
	 * The reboot notifier sets e9810_shutdown under e9810_arm_lock (held
	 * here) before system_state changes; nothing arms after it.
	 */
	if (READ_ONCE(e9810_shutdown) ||
	    system_state == SYSTEM_HALT || system_state == SYSTEM_POWER_OFF ||
	    system_state == SYSTEM_RESTART)
		return "restart or power-off under way";
	if (e9810_quarantine & E9810_QF_UNINIT)
		return "C2 quarantined (all): no valid quarantine record yet (fail closed)";
	if (e9810_quarantine & E9810_QF_UNCLEAN)
		return "C2 quarantined (all): the last armed C2 session ended without a clean restart";
	if (e9810_quarantine & E9810_QF_BIG)
		return "C2 quarantined (all): a big core was lost on an earlier boot";
	if (e9810_cluster_of[cpu] == 0 && (e9810_quarantine & E9810_QF_LITTLE))
		return "C2 quarantined (LITTLE): a LITTLE core was lost on an earlier boot";
	if (e9810_cluster_of[cpu] == 0 && (e9810_quarantine & E9810_QF_LTIMER))
		return "C2 quarantined (LITTLE, local timers): an MCT local timer fault, or a local-timer session that was not closed, on an earlier boot";
	/* The fault count rises at the latch, before the fault work runs. */
	if (e9810_cluster_of[cpu] == 0 && (READ_ONCE(e9810_lt_faulted) ||
					   exynos9810_mct_local_faults()))
		return "LITTLE C2 off: an MCT local timer failed on this boot";
	if (e9810_cluster_of[cpu] == 0 && !e9810_little_c2)
		return "LITTLE C2 disabled (cores lost in EL3 in q22b/q40/q43; DT samsung,c2-little-enable)";
	if (!cpu_online(cpu))
		return e9810_why_offline;
	if (atomic_read(&e9810_deep_off))
		return "deep idle disabled";
	if (!exynos9810_mct_bc_ready())
		return e9810_why_no_mct;
	/* Final: the takeover ran (e9810_lt_takeover()) before this arming. */
	if (e9810_cluster_of[cpu] == 0 && e9810_little_ltimer &&
	    !exynos9810_mct_local_active(cpu))
		return "LITTLE C2 needs the CPU's tick on its MCT local timer (DT samsung,c2-little-local-timer), and it is not (see the exynos9810-mct-bc local timer lines)";
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
	e9810_boot_arm_online(cpu);
	return 0;
}

/*
 * Arm every CPU of @mask, or none. The armed-session record is persisted
 * (and read back) before the first CPU is armed.
 */
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
	if (!why && !cpumask_subset(mask, &e9810_armed))
		why = e9810_qm_session_arm();
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

static void e9810_disarm_mask(const struct cpumask *mask)
{
	unsigned int cpu;

	mutex_lock(&e9810_arm_lock);
	for_each_cpu(cpu, mask)
		e9810_state_disable(cpu, true);
	cpumask_andnot(&e9810_armed, &e9810_armed, mask);
	smp_mb();
	mutex_unlock(&e9810_arm_lock);
	wake_up_all_idle_cpus();
}

/* Shutdown drain: bound, and how often the idle CPUs are kicked meanwhile. */
#define E9810_DRAIN_MS			1000
#define E9810_DRAIN_KICK_MS		10

/*
 * After the gate is closed (e9810_shutdown set, arm mask cleared): wait, at
 * most E9810_DRAIN_MS, until every possible CPU has shown e9810_c2_busy == 0.
 * A 0 seen after the barrier below is a positive acknowledgement: that CPU
 * is outside e9810_enter_c2() at that moment, and every later entry sees the
 * closed gate. An RCU grace period (cpuidle_pause()) proves nothing here: a
 * CPU in the firmware stage is in an RCU idle section and does not hold
 * one up. True if all acknowledged; @stuck gets the CPUs that did not.
 */
static bool e9810_drain(struct cpumask *stuck)
{
	unsigned long end = jiffies + msecs_to_jiffies(E9810_DRAIN_MS);
	unsigned long kick = jiffies;
	unsigned int cpu;

	/* Pairs with the smp_mb() between busy and the gate in e9810_enter_c2(). */
	smp_mb();
	cpumask_copy(stuck, cpu_possible_mask);
	for (;;) {
		for_each_cpu(cpu, stuck)
			if (!smp_load_acquire(per_cpu_ptr(&e9810_c2_busy, cpu)))
				cpumask_clear_cpu(cpu, stuck);
		if (cpumask_empty(stuck))
			return true;
		if (time_after(jiffies, end))
			return false;
		if (time_after_eq(jiffies, kick)) {
			wake_up_all_idle_cpus();
			kick = jiffies + msecs_to_jiffies(E9810_DRAIN_KICK_MS);
		}
		usleep_range(500, 1000);
	}
}

/*
 * Start of a restart, halt or power-off (kernel_restart(), kernel_halt(),
 * kernel_power_off(), and kernel_kexec(): reboot(2), orderly_reboot(),
 * Android "reboot", the diag PID1 120/150 s restart, the ROM-RAM DONE
 * reboot). Closes the arm gate for good, then requires every CPU to
 * acknowledge that it left C2 (e9810_drain()). Only then is an ARMED record
 * moved to SHUTDOWN, which the next boot still treats as unclean; IDLE is
 * written only at the last controlled point (e9810_qm_session_final():
 * the restart handler, or syscore shutdown for halt and power-off). Panic,
 * emergency_restart(), a watchdog reset and power loss never get there.
 */
static int e9810_reboot_cb(struct notifier_block *nb, unsigned long event,
			   void *data)
{
	static const char * const how[] = {
		[SYS_RESTART] = "restart", [SYS_HALT] = "halt",
		[SYS_POWER_OFF] = "power-off",
	};
	struct cpumask stuck;
	unsigned int cpu;

	if (event != SYS_RESTART && event != SYS_POWER_OFF &&
	    event != SYS_HALT)
		return NOTIFY_OK;

	/* Irreversible; every arming path checks it under this lock. */
	mutex_lock(&e9810_arm_lock);
	WRITE_ONCE(e9810_shutdown, true);
	for_each_possible_cpu(cpu)
		e9810_state_disable(cpu, true);
	cpumask_clear(&e9810_armed);
	mutex_unlock(&e9810_arm_lock);

	/*
	 * Like 4.9, no deep idle while the restart/power-off path runs: the
	 * gate does that. cpuidle_pause() is not used: its RCU wait proves
	 * nothing about C2 and blocks on a CPU lost outside RCU idle.
	 */
	if (!e9810_drain(&stuck)) {
		pr_emerg("C2 session NOT closed (%s): cpu %*pbl did not acknowledge leaving C2 within %u ms; the record keeps its session (armed: next boot UNCLEAN)\n",
			 how[event], cpumask_pr_args(&stuck), E9810_DRAIN_MS);
		return NOTIFY_OK;
	}
	e9810_qm_session_drained(how[event]);
	return NOTIFY_OK;
}

/*
 * One below INT_MAX, so the watchdog's restart guard (INT_MAX, arms the
 * final timeout) always runs first, even if this one blocks.
 */
static struct notifier_block e9810_reboot_nb = {
	.notifier_call = e9810_reboot_cb,
	.priority = INT_MAX - 1,
};

/*
 * Set by syscore shutdown in an orderly kernel_restart() (not kexec): the
 * restart handler may close the session. emergency_restart() and panic
 * reach the restart handlers without passing syscore shutdown.
 */
static bool e9810_restart_final_ok;

static void e9810_syscore_shutdown(void)
{
	if (kexec_in_progress) {
		/* kexec is unclean by definition: no restart handler follows. */
		pr_emerg("C2 session: kexec: the record stays as it is (next boot: UNCLEAN if C2 was armed)\n");
		return;
	}
	if (system_state == SYSTEM_RESTART)
		WRITE_ONCE(e9810_restart_final_ok, true);
	else if (system_state == SYSTEM_POWER_OFF)
		e9810_qm_session_final("power-off", false);
	else if (system_state == SYSTEM_HALT)
		e9810_qm_session_final("halt", false);
}

static struct syscore_ops e9810_syscore_ops = {
	.shutdown = e9810_syscore_shutdown,
};

/*
 * The last controlled point of a restart: machine_restart(), IRQs off,
 * the other CPUs stopped. INT_MAX: before every handler that resets (the
 * crownlte diagnostic restart guard at 255 never returns to the chain).
 */
static int e9810_restart_cb(struct notifier_block *nb, unsigned long mode,
			    void *cmd)
{
	e9810_qm_session_final("restart", true);
	return NOTIFY_DONE;
}

static struct notifier_block e9810_restart_nb = {
	.notifier_call = e9810_restart_cb,
	.priority = INT_MAX,
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
	register_syscore_ops(&e9810_syscore_ops);
	ret = register_restart_handler(&e9810_restart_nb);
	if (ret)
		pr_warn("C2 restart handler not registered (%d): a restart never closes an armed session (next boot UNCLEAN)\n",
			ret);
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

/* ------------------------------------------------------------------------ */
/* S0: self-recovery chain                                                  */
/* ------------------------------------------------------------------------ */

#define E9810_DET_PERIOD_MS		2000
#define E9810_DET_FW_MIN_MS		1000
#define E9810_DET_PING_MS		200
#define E9810_STATS_PERIOD_MS		(10 * 60 * 1000)
#define E9810_ARM_RETRY_MS		1000
#define E9810_ARM_RETRY_MAX_MS		60000
#define E9810_ARM_DELAY_MAX_MS		600000
#define E9810_DRILL_MS_MAX		600000
/* A drill waits this long for the watchdog reset before rebooting itself. */
#define E9810_DRILL_REBOOT_MS		45000
/* Still alive this long after a loss: emergency_restart(). */
#define E9810_LAST_RESORT_MS		75000

/*
 * Quarantine record v3 (Documentation/crownlte/c2-runtime.md section 6):
 * one page of no-map reserved memory holding two record slots (byte offset
 * 0 and 2048), u32 words. Each write goes to the slot that does not hold
 * the newest record, with the sequence number one higher, so a reset in
 * the middle of a write leaves the previous record intact. A slot is valid
 * only with magic, version, its own slot number, a sequence number of its
 * parity, a known session word, no unknown flag/drill bits or reason and a
 * CRC32 over the words before it. Written through a write-combining
 * mapping, a DSB and a read-back.
 *
 * Trust: the record is trusted only when BOTH slots are valid and exactly
 * one sequence number apart (the newer one wins). Every write keeps it so,
 * and a fresh record is written twice. One valid slot, or two that do not
 * chain, is ambiguous: the missing newer record may have been ARMED or
 * quarantined, so a lone clean slot never authorizes C2. Ambiguous and
 * missing records mean all C2 off (E9810_QF_UNINIT); what survives (loss
 * flags, an armed session, consumed drills, the trial floor) is merged in.
 *
 * Fail closed until an explicit clear: root writes c2_quarantine_clear, or
 * a trial DTB carries samsung,c2-quarantine-trial-token above the record's
 * trial floor on a trusted record. A record whose session word says ARMED
 * or SHUTDOWN means the boot that armed C2 never reached the last
 * controlled point of a restart, halt or power-off: all C2 off
 * (E9810_QF_UNCLEAN).
 *
 * v4 (same 14-word layout; v3 slots are still read and chain with v4
 * ones, every write is v4, a v3 kernel treats v4 slots as invalid and
 * fails closed): two more session words for a local-timer trial,
 * LT_ARMED (persisted before the MCT local timers take over; covers an
 * armed C2 session too) and LT_SHUTDOWN (its drained form), flag
 * E9810_QF_LTIMER and reason E9810_QR_LTIMER (a local timer fault
 * recorded at runtime). A session found LT_ARMED or LT_SHUTDOWN at boot
 * adds E9810_QF_LTIMER to E9810_QF_UNCLEAN.
 */
enum {
	QM_MAGIC, QM_VERSION, QM_SLOT, QM_SEQ, QM_FLAGS, QM_SESSION, QM_BOOTS,
	QM_CPUS, QM_UPTIME_MS, QM_REASON, QM_LOSSES, QM_DRILLS, QM_TRIAL,
	QM_CRC, QM_WORDS
};
#define E9810_QM_MAGIC			0x51324339	/* "9C2Q" */
#define E9810_QM_VERSION		4
#define E9810_QM_VERSION_V3		3	/* read only */
#define E9810_QM_SLOTS			2
#define E9810_QM_SLOT_BYTES		2048
/* QM_SESSION: C2 may be armed on the boot that wrote it, or not. */
#define E9810_QS_IDLE			0x49444c45	/* "ELDI" */
#define E9810_QS_ARMED			0x41524d44	/* "DMRA" */
/* Drained at the start of a shutdown, last controlled point not reached. */
#define E9810_QS_SHUTDOWN		0x5348444e	/* "NDHS" */
/* v4: MCT local timers may run CPU ticks (and C2 may be armed). */
#define E9810_QS_LT_ARMED		0x4c41524d	/* "MRAL" */
/* v4: LT_ARMED, drained at the start of a shutdown. */
#define E9810_QS_LT_SHUTDOWN		0x4c534844	/* "DHSL" */
/*
 * QM_TRIAL: the trial floor. A DT trial token clears only if it is above it
 * (and the record is trusted); then it becomes the floor. Tokens are one
 * increasing sequence over all trial DTBs, so a token that was used, or
 * superseded by a later one, never clears again. Tokens at or below
 * E9810_QM_TRIAL_RETIRED (rounds 0 and 1) never clear. When the history is
 * lost (no trusted record), the presented token is folded into the floor
 * without clearing: a later boot needs a new DTB with a higher token.
 */
#define E9810_QM_TRIAL_RETIRED		0xc2b10001
/* QM_DRILLS: drills already run (consumed); root re-arms them. */
#define E9810_QD_LOST			BIT(0)	/* samsung,c2-lost-drill */
#define E9810_QD_LOSS			BIT(1)	/* samsung,c2-loss-drill-ms */
#define E9810_QD_ALL			(E9810_QD_LOST | E9810_QD_LOSS)

enum {
	E9810_QR_NONE,
	E9810_QR_DETECTOR,	/* the runtime lost-core detector */
	E9810_QR_SELFTEST,	/* a self-test phase, not recovered */
	E9810_QR_DRILL,		/* samsung,c2-lost-drill (simulated) */
	E9810_QR_UNCLEAN,	/* armed session without a clean restart */
	E9810_QR_INVALID,	/* no valid record found at boot */
	E9810_QR_SHUTDOWN,	/* drained, last controlled point not reached */
	E9810_QR_AMBIGUOUS,	/* one valid slot, or two that do not chain */
	E9810_QR_COUNT_V3,
	E9810_QR_LTIMER = E9810_QR_COUNT_V3,	/* v4: an MCT local timer fault */
	E9810_QR_COUNT
};

static const char * const e9810_qr_name[] = {
	[E9810_QR_NONE]		= "none",
	[E9810_QR_DETECTOR]	= "detector",
	[E9810_QR_SELFTEST]	= "selftest",
	[E9810_QR_DRILL]	= "drill",
	[E9810_QR_UNCLEAN]	= "unclean",
	[E9810_QR_INVALID]	= "invalid",
	[E9810_QR_SHUTDOWN]	= "shutdown",
	[E9810_QR_AMBIGUOUS]	= "ambiguous",
	[E9810_QR_LTIMER]	= "ltimer",
};

static void *e9810_qm;			/* mapped page, NULL if none */
static phys_addr_t e9810_qm_phys;
static const char *e9810_qm_why = "not probed";
/* Newest record written (or adopted at probe), under e9810_qm_lock. */
static u32 e9810_qm_cur[QM_WORDS];
static bool e9810_qm_found;		/* a trusted record was found at probe */
static bool e9810_qm_bad;		/* a write failed its read-back */
/* Set by the shutdown drain once it wrote SHUTDOWN: the final point closes. */
static const char *e9810_qm_close_how;
static DEFINE_SPINLOCK(e9810_qm_lock);

struct e9810_det_ping {
	call_single_data_t csd;
	unsigned long sent;
	unsigned long ack;		/* written by the target */
	u32 gen;			/* firmware entry sampled by the scan */
};
static DEFINE_PER_CPU(struct e9810_det_ping, e9810_det_ping);

static struct delayed_work e9810_det_work;
static u64 e9810_det_scans, e9810_det_pings, e9810_det_enq_fail,
	   e9810_det_revalidated, e9810_det_hp_unlocked;
static unsigned int e9810_det_hp_skips;
static unsigned long e9810_stats_next;
/* CPUs a self-test phase watches itself (20 ms pings, its own recovery). */
static struct cpumask e9810_selftest_cpus;

static atomic_t e9810_lost_fired = ATOMIC_INIT(0);
static bool e9810_lost_drill;
static struct delayed_work e9810_reboot_work;
static struct delayed_work e9810_last_resort_work;

/* samsung,c2-lost-drill = <cpu ms> */
static unsigned int e9810_drill_cpu = UINT_MAX;
static u32 e9810_drill_at_ms;
static bool e9810_drill_fired;

/* Boot arming (samsung,c2-boot-cpus) and samsung,c2-loss-drill-ms. */
static struct delayed_work e9810_arm_work;
static struct cpumask e9810_boot_mask;
/* Boot-arming CPUs not armed yet; retried, offline ones when they return. */
static struct cpumask e9810_boot_pending;
static unsigned long e9810_arm_deadline;
static bool e9810_arm_retrying;
static bool e9810_arm_done;
static u32 e9810_loss_drill_ms;
static struct delayed_work e9810_loss_drill_work;

static u64 e9810_boot_ms(void)
{
	return ktime_to_ms(ktime_get_boottime());
}

static u32 e9810_qm_crc(const u32 *w)
{
	return crc32_le(~0U, (const u8 *)w, QM_CRC * sizeof(u32)) ^ ~0U;
}

static u32 *e9810_qm_slot(unsigned int slot)
{
	return e9810_qm + slot * E9810_QM_SLOT_BYTES;
}

static void e9810_qm_read(unsigned int slot, u32 *w)
{
	const u32 *p = e9810_qm_slot(slot);
	unsigned int i;

	for (i = 0; i < QM_WORDS; i++)
		w[i] = READ_ONCE(p[i]);
}

#define E9810_QF_KNOWN_V3 (E9810_QF_LITTLE | E9810_QF_BIG | E9810_QF_DRILL | \
			   E9810_QF_UNCLEAN | E9810_QF_UNINIT)
#define E9810_QF_KNOWN	(E9810_QF_KNOWN_V3 | E9810_QF_LTIMER)

static bool e9810_qs_lt(u32 session)
{
	return session == E9810_QS_LT_ARMED || session == E9810_QS_LT_SHUTDOWN;
}

/* v4, or a v3 slot (the v3 subset of sessions, flags and reasons). */
static bool e9810_qm_valid(const u32 *w, unsigned int slot)
{
	bool v4 = w[QM_VERSION] == E9810_QM_VERSION;

	if (!v4 && w[QM_VERSION] != E9810_QM_VERSION_V3)
		return false;
	return w[QM_MAGIC] == E9810_QM_MAGIC &&
	       w[QM_SLOT] == slot && (w[QM_SEQ] & 1) == slot &&
	       (w[QM_SESSION] == E9810_QS_IDLE ||
		w[QM_SESSION] == E9810_QS_ARMED ||
		w[QM_SESSION] == E9810_QS_SHUTDOWN ||
		(v4 && e9810_qs_lt(w[QM_SESSION]))) &&
	       !(w[QM_FLAGS] & ~(v4 ? E9810_QF_KNOWN : E9810_QF_KNOWN_V3)) &&
	       !(w[QM_DRILLS] & ~E9810_QD_ALL) &&
	       w[QM_REASON] < (v4 ? E9810_QR_COUNT : E9810_QR_COUNT_V3) &&
	       w[QM_CRC] == e9810_qm_crc(w);
}

static const char *e9810_qs_str(u32 session)
{
	switch (session) {
	case E9810_QS_IDLE:
		return "idle";
	case E9810_QS_ARMED:
		return "armed";
	case E9810_QS_SHUTDOWN:
		return "shutdown";
	case E9810_QS_LT_ARMED:
		return "ltimer-armed";
	case E9810_QS_LT_SHUTDOWN:
		return "ltimer-shutdown";
	}
	return "?";
}

/*
 * Write @w as the next record: sequence number one above the newest, into
 * the other slot, then read it back. The newest record stays untouched, so
 * an interrupted write falls back to it. Caller holds e9810_qm_lock (or
 * runs at probe). On success @w is the newest record.
 */
static bool e9810_qm_commit(u32 *w)
{
	u32 rb[QM_WORDS], *p;
	unsigned int slot, i;

	w[QM_MAGIC] = E9810_QM_MAGIC;
	w[QM_VERSION] = E9810_QM_VERSION;
	w[QM_SEQ] = e9810_qm_cur[QM_SEQ] + 1;
	slot = w[QM_SEQ] & 1;
	w[QM_SLOT] = slot;
	w[QM_CRC] = e9810_qm_crc(w);
	p = e9810_qm_slot(slot);
	for (i = 0; i < QM_WORDS; i++)
		WRITE_ONCE(p[i], w[i]);
	/* Out of the write buffers before anything else happens. */
	dsb(sy);
	e9810_qm_read(slot, rb);
	if (memcmp(rb, w, sizeof(rb)) || !e9810_qm_valid(rb, slot)) {
		e9810_qm_bad = true;
		return false;
	}
	memcpy(e9810_qm_cur, w, sizeof(rb));
	return true;
}

static const char *e9810_qm_flags_str(u32 flags)
{
	if (flags & E9810_QF_ALL_OFF)
		return "all";
	if (flags & E9810_QF_LITTLE_OFF)
		return "LITTLE";
	return "none";
}

static const char *e9810_qr_str(u32 reason)
{
	return reason < ARRAY_SIZE(e9810_qr_name) ? e9810_qr_name[reason] : "?";
}

enum { E9810_QM_TRUSTED, E9810_QM_AMBIGUOUS, E9810_QM_NONE };

/*
 * Read both slots. E9810_QM_TRUSTED: both valid and exactly one sequence
 * number apart; @w is the newer one. E9810_QM_AMBIGUOUS: one valid slot, or
 * two that do not chain; the newest record may be lost, so @w is the
 * conservative merge of the valid slots (every flag, an unclean flag for a
 * session that is not IDLE, every lost CPU and consumed drill, the highest
 * counters and trial floor). E9810_QM_NONE: no valid slot, @w zeroed.
 * *seq_base: the highest valid sequence number (0 if none), so the next
 * write never reuses one.
 */
static int e9810_qm_pick(u32 *w, u32 *seq_base, const char **why)
{
	u32 s[E9810_QM_SLOTS][QM_WORDS];
	bool v[E9810_QM_SLOTS];
	unsigned int i, n = 0;

	*seq_base = 0;
	memset(w, 0, sizeof(s[0]));
	for (i = 0; i < E9810_QM_SLOTS; i++) {
		e9810_qm_read(i, s[i]);
		v[i] = e9810_qm_valid(s[i], i);
		n += v[i];
	}
	if (n == 2) {
		u32 d = s[1][QM_SEQ] - s[0][QM_SEQ];

		*seq_base = (s32)d > 0 ? s[1][QM_SEQ] : s[0][QM_SEQ];
		if (d == 1 || d == (u32)-1) {
			memcpy(w, s[d == 1 ? 1 : 0], sizeof(s[0]));
			return E9810_QM_TRUSTED;
		}
		*why = "both slots valid but not one sequence number apart";
	} else if (n == 1) {
		*why = "only one valid slot: its successor may be lost";
	} else {
		*why = "no valid slot";
		return E9810_QM_NONE;
	}

	for (i = 0; i < E9810_QM_SLOTS; i++) {
		if (!v[i])
			continue;
		if (n == 1)
			*seq_base = s[i][QM_SEQ];
		w[QM_FLAGS] |= s[i][QM_FLAGS];
		if (s[i][QM_SESSION] != E9810_QS_IDLE)
			w[QM_FLAGS] |= E9810_QF_UNCLEAN;
		if (e9810_qs_lt(s[i][QM_SESSION]))
			w[QM_FLAGS] |= E9810_QF_LTIMER;
		w[QM_CPUS] |= s[i][QM_CPUS];
		w[QM_DRILLS] |= s[i][QM_DRILLS];
		w[QM_BOOTS] = max(w[QM_BOOTS], s[i][QM_BOOTS]);
		w[QM_LOSSES] = max(w[QM_LOSSES], s[i][QM_LOSSES]);
		w[QM_UPTIME_MS] = max(w[QM_UPTIME_MS], s[i][QM_UPTIME_MS]);
		w[QM_TRIAL] = max(w[QM_TRIAL], s[i][QM_TRIAL]);
	}
	return E9810_QM_AMBIGUOUS;
}

/*
 * Map the marker page, take over its quarantine and count this boot. Runs
 * at probe, before anything can be armed. Every outcome that is not a
 * verified write of a valid record leaves all C2 off this boot.
 */
static void e9810_qm_init(struct device *dev)
{
	struct device_node *mnp;
	struct reserved_mem *rmem;
	u32 w[QM_WORDS], seq_base, token = 0;
	const char *bad = NULL;
	bool nomap, ok, cleared = false;
	int state;

	/* Fail closed until a valid record is in place. */
	e9810_quarantine = E9810_QF_UNINIT;

	mnp = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!mnp) {
		e9810_qm_why = "no memory-region in the cpuidle node";
		goto none;
	}
	rmem = of_reserved_mem_lookup(mnp);
	nomap = of_property_read_bool(mnp, "no-map");
	of_node_put(mnp);
	if (!rmem || rmem->size < E9810_QM_SLOTS * E9810_QM_SLOT_BYTES) {
		e9810_qm_why = "memory-region is not a 4 KB reserved-memory region";
		goto none;
	}
	/* Mapped write-combining; the linear map must not alias it. */
	if (!nomap) {
		e9810_qm_why = "memory-region is not no-map";
		goto none;
	}
	e9810_qm = devm_memremap(dev, rmem->base, rmem->size, MEMREMAP_WC);
	if (IS_ERR_OR_NULL(e9810_qm)) {
		e9810_qm = NULL;
		e9810_qm_why = "memremap failed";
		goto none;
	}
	e9810_qm_phys = rmem->base;
	e9810_qm_why = NULL;
	of_property_read_u32(dev->of_node, "samsung,c2-quarantine-trial-token",
			     &token);

	state = e9810_qm_pick(w, &seq_base, &bad);
	e9810_qm_found = state == E9810_QM_TRUSTED;
	e9810_qm_cur[QM_SEQ] = seq_base;
	if (state == E9810_QM_TRUSTED) {
		if (w[QM_SESSION] != E9810_QS_IDLE) {
			w[QM_FLAGS] |= E9810_QF_UNCLEAN;
			/* The local timers may have caused it: none next time. */
			if (e9810_qs_lt(w[QM_SESSION]))
				w[QM_FLAGS] |= E9810_QF_LTIMER;
			if (w[QM_REASON] == E9810_QR_NONE)
				w[QM_REASON] =
					(w[QM_SESSION] == E9810_QS_SHUTDOWN ||
					 w[QM_SESSION] == E9810_QS_LT_SHUTDOWN) ?
					E9810_QR_SHUTDOWN : E9810_QR_UNCLEAN;
		}
		/* Explicit one-shot clear by a trial DTB: a token above the floor. */
		if (token && token <= E9810_QM_TRIAL_RETIRED) {
			pr_warn("C2 quarantine: trial token %#x is retired (rounds 0/1, at or below %#x): ignored\n",
				token, E9810_QM_TRIAL_RETIRED);
		} else if (token && token <= w[QM_TRIAL]) {
			pr_warn("C2 quarantine: trial token %#x not above the trial floor %#x (already used or superseded): ignored; a clear needs a new DTB with a higher token, or root c2_quarantine_clear\n",
				token, w[QM_TRIAL]);
		} else if (token) {
			pr_warn("C2 quarantine: trial token %#x (floor %#x): one-shot clear of flags %#x (%s) by the DT\n",
				token, w[QM_TRIAL], w[QM_FLAGS],
				e9810_qm_flags_str(w[QM_FLAGS]));
			w[QM_FLAGS] = 0;
			w[QM_CPUS] = 0;
			w[QM_UPTIME_MS] = 0;
			w[QM_REASON] = E9810_QR_NONE;
			w[QM_TRIAL] = token;
			cleared = true;
		}
	} else {
		/* @w: the merge of what survived (ambiguous), or zeroes. */
		w[QM_FLAGS] |= E9810_QF_UNINIT;
		w[QM_REASON] = state == E9810_QM_AMBIGUOUS ?
			       E9810_QR_AMBIGUOUS : E9810_QR_INVALID;
		/*
		 * Whether this token was consumed before cannot be told: it
		 * is spent now, without clearing (renewed authorization: a
		 * new DTB with a higher token, or root).
		 */
		if (token > E9810_QM_TRIAL_RETIRED && token > w[QM_TRIAL]) {
			pr_warn("C2 quarantine: trial token %#x not applied: no trusted record, so its earlier use cannot be ruled out; it is now the trial floor (a clear needs a new DTB with a token above %#x, or root c2_quarantine_clear)\n",
				token, token);
			w[QM_TRIAL] = token;
		}
	}
	w[QM_SESSION] = E9810_QS_IDLE;
	w[QM_BOOTS]++;
	ok = e9810_qm_commit(w);
	/* A fresh or rebuilt record goes into both slots: trusted next time. */
	if (ok && state != E9810_QM_TRUSTED)
		ok = e9810_qm_commit(w);
	if (!ok) {
		e9810_qm_why = "record write failed its read-back";
		pr_err("C2 quarantine marker at %pa: write failed its read-back: all C2 off this boot\n",
		       &e9810_qm_phys);
		return;
	}
	e9810_quarantine = w[QM_FLAGS] & (E9810_QF_LITTLE_OFF | E9810_QF_ALL_OFF);

	if (state == E9810_QM_NONE)
		pr_warn("C2 quarantined (all): no valid record at %pa (%s): fail closed, a new UNINIT record (seq %u, both slots) was written; clear: echo 1 > /sys/bus/platform/devices/%s/c2_quarantine_clear, or a trial DTB with a token above %#x on a later boot\n",
			&e9810_qm_phys, bad, w[QM_SEQ], dev_name(dev),
			max_t(u32, w[QM_TRIAL], E9810_QM_TRIAL_RETIRED));
	else if (state == E9810_QM_AMBIGUOUS)
		pr_warn("C2 quarantined (all): ambiguous record at %pa (%s): fail closed, kept flags %#x%s, cpus %#x, %u losses, drills %#x, trial floor %#x; rewritten as UNINIT (seq %u, both slots); clear: echo 1 > /sys/bus/platform/devices/%s/c2_quarantine_clear, or a trial DTB with a token above %#x on a later boot\n",
			&e9810_qm_phys, bad, w[QM_FLAGS],
			(w[QM_FLAGS] & E9810_QF_UNCLEAN) ? " UNCLEAN (a surviving slot had a session that was not closed)" : "",
			w[QM_CPUS], w[QM_LOSSES], w[QM_DRILLS], w[QM_TRIAL],
			w[QM_SEQ], dev_name(dev),
			max_t(u32, w[QM_TRIAL], E9810_QM_TRIAL_RETIRED));
	else if (e9810_quarantine)
		pr_warn("C2 quarantined (%s): flags %#x%s%s%s%s, cpus %#x lost at %u ms (%s), %u losses, %u boots counted (seq %u); %s; clear: echo 1 > /sys/bus/platform/devices/%s/c2_quarantine_clear, or a trial DTB with a token above %#x\n",
			e9810_qm_flags_str(e9810_quarantine), w[QM_FLAGS],
			(w[QM_FLAGS] & E9810_QF_UNCLEAN) ? " UNCLEAN (the last armed session never reached the last controlled point of a restart/halt/power-off)" : "",
			(w[QM_FLAGS] & E9810_QF_UNINIT) ? " UNINIT (never cleared since a missing or ambiguous record)" : "",
			(w[QM_FLAGS] & E9810_QF_LTIMER) ? " LTIMER (an MCT local timer fault, or a local-timer session not closed: no local timer takeover, LITTLE C2 off)" : "",
			(w[QM_FLAGS] & E9810_QF_DRILL) ? " drill" : "",
			w[QM_CPUS], w[QM_UPTIME_MS], e9810_qr_str(w[QM_REASON]),
			w[QM_LOSSES], w[QM_BOOTS], w[QM_SEQ],
			(e9810_quarantine & E9810_QF_ALL_OFF) ?
			"all C2 off" : "LITTLE C2 off, big C2 allowed",
			dev_name(dev),
			max_t(u32, w[QM_TRIAL], E9810_QM_TRIAL_RETIRED));
	else
		pr_info("C2 quarantine marker at %pa: clear%s (seq %u, %u losses, %u boots counted, drills consumed %#x, trial floor %#x)\n",
			&e9810_qm_phys, cleared ? " (trial token)" : "",
			w[QM_SEQ], w[QM_LOSSES], w[QM_BOOTS], w[QM_DRILLS],
			w[QM_TRIAL]);
	return;
none:
	/* Boot arming and drills are refused; the DT-gated self-test may arm. */
	e9810_quarantine = 0;
	pr_warn("C2 quarantine marker not available: %s; boot arming and drills refused\n",
		e9810_qm_why);
}

/*
 * Persist "a C2 session is armed" before the first CPU is armed. Without
 * the marker only the DT-gated self-test may arm (boot arming and drills
 * are refused earlier), and there is nothing to persist.
 */
static const char *e9810_qm_session_arm(void)
{
	unsigned long flags;
	u32 w[QM_WORDS];
	bool ok = true;

	/* Under e9810_arm_lock: the shutdown gate cannot close meanwhile. */
	if (READ_ONCE(e9810_shutdown))
		return "restart or power-off under way";
	if (!e9810_qm)
		return NULL;
	spin_lock_irqsave(&e9810_qm_lock, flags);
	/* LT_ARMED already covers an armed C2 session: never downgrade it. */
	if (e9810_qm_cur[QM_SESSION] != E9810_QS_ARMED &&
	    e9810_qm_cur[QM_SESSION] != E9810_QS_LT_ARMED) {
		memcpy(w, e9810_qm_cur, sizeof(w));
		w[QM_SESSION] = E9810_QS_ARMED;
		ok = e9810_qm_commit(w);
	}
	spin_unlock_irqrestore(&e9810_qm_lock, flags);
	if (!ok)
		return "the armed-session record failed its read-back";
	return NULL;
}

/*
 * Persist "MCT local timers may run CPU ticks" (LT_ARMED, read back)
 * before the takeover: from here on, any end of this boot other than the
 * last controlled point of a clean shutdown leaves LT_ARMED or LT_SHUTDOWN,
 * which the next boot turns into UNCLEAN + LTIMER (no takeover, LITTLE C2
 * off). Under e9810_arm_lock. Refused without the marker.
 */
static const char *e9810_qm_session_ltimer(void)
{
	unsigned long flags;
	u32 w[QM_WORDS];
	bool ok = true;

	if (READ_ONCE(e9810_shutdown))
		return "restart or power-off under way";
	if (!e9810_qm)
		return "no quarantine record to hold the local-timer session";
	spin_lock_irqsave(&e9810_qm_lock, flags);
	if (e9810_qm_cur[QM_SESSION] != E9810_QS_LT_ARMED) {
		memcpy(w, e9810_qm_cur, sizeof(w));
		w[QM_SESSION] = E9810_QS_LT_ARMED;
		ok = e9810_qm_commit(w);
	}
	spin_unlock_irqrestore(&e9810_qm_lock, flags);
	if (!ok)
		return "the local-timer session record failed its read-back";
	return NULL;
}

/*
 * The session may not be closed on this boot: the fatal local timer path
 * ran, the fault could not be recorded, or a local timer failed and its
 * LTIMER flag is not in the record yet (the fault work has not run). Any
 * context; the last check reads e9810_qm_cur without the lock (a stale
 * read only keeps the session open).
 */
static bool e9810_lt_close_blocked(void)
{
	return atomic_read(&e9810_lt_block_close) ||
	       (exynos9810_mct_local_faults() &&
		!(READ_ONCE(e9810_qm_cur[QM_FLAGS]) & E9810_QF_LTIMER));
}

/*
 * The shutdown drain succeeded (gate closed for good, every CPU
 * acknowledged leaving C2): ARMED -> SHUTDOWN. The next boot still treats
 * SHUTDOWN as unclean; e9810_qm_session_final() closes it at the last
 * controlled point. Never after a loss (the record says why).
 */
static void e9810_qm_session_drained(const char *how)
{
	unsigned long flags;
	u32 w[QM_WORDS];
	bool ok = true, was = false;

	if (!e9810_qm)
		return;
	if (atomic_read(&e9810_lost_fired)) {
		pr_emerg("C2 session not closed (%s): a loss was recorded on this boot\n",
			 how);
		return;
	}
	if (e9810_lt_close_blocked()) {
		pr_emerg("C2 session not closed (%s): an MCT local timer fault on this boot is not safely recorded, or took the fatal path\n",
			 how);
		return;
	}
	spin_lock_irqsave(&e9810_qm_lock, flags);
	if (e9810_qm_cur[QM_SESSION] == E9810_QS_ARMED ||
	    e9810_qm_cur[QM_SESSION] == E9810_QS_LT_ARMED) {
		was = true;
		memcpy(w, e9810_qm_cur, sizeof(w));
		w[QM_SESSION] = e9810_qm_cur[QM_SESSION] == E9810_QS_LT_ARMED ?
				E9810_QS_LT_SHUTDOWN : E9810_QS_SHUTDOWN;
		ok = e9810_qm_commit(w);
		if (ok)
			WRITE_ONCE(e9810_qm_close_how, how);
	}
	spin_unlock_irqrestore(&e9810_qm_lock, flags);
	if (was)
		pr_info("C2 session drained (%s): every CPU acknowledged leaving C2; record %s (seq %u), closed only at the final %s point\n",
			how, !ok ? "WRITE FAILED, stays armed" :
			e9810_qm_cur[QM_SESSION] == E9810_QS_LT_SHUTDOWN ?
			"LT_SHUTDOWN" : "SHUTDOWN",
			e9810_qm_cur[QM_SEQ],
			strcmp(how, "restart") ? "syscore shutdown" : "restart handler");
}

/*
 * The last controlled point: the restart handler (machine_restart(), IRQs
 * off, other CPUs stopped) after an orderly kernel_restart(), or syscore
 * shutdown for halt and power-off. SHUTDOWN -> IDLE, only if this boot
 * drained cleanly, recorded no loss and is not in a panic, an oops or
 * kexec. Never waits for the lock: a stopped CPU may hold it, and then the
 * record stays SHUTDOWN (unclean). Nothing is printed after the write.
 */
static void e9810_qm_session_final(const char *how, bool restart)
{
	unsigned long flags;
	u32 w[QM_WORDS];
	bool ok = false, locked;

	if (!e9810_qm || !READ_ONCE(e9810_qm_close_how) ||
	    atomic_read(&e9810_lost_fired) ||
	    atomic_read(&panic_cpu) != PANIC_CPU_INVALID ||
	    oops_in_progress || kexec_in_progress)
		return;
	if (restart && (!READ_ONCE(e9810_restart_final_ok) ||
			system_state != SYSTEM_RESTART))
		return;
	/* A local timer fault not (safely) recorded, or a takeover running. */
	if (e9810_lt_close_blocked() || READ_ONCE(e9810_lt_busy)) {
		pr_emerg("C2 session NOT closed at the final %s point: MCT local timer fault not recorded, fatal path taken, or takeover still running; the next boot sees it as unclean\n",
			 how);
		return;
	}
	pr_emerg("C2 session: final %s point: closing the record (%s -> IDLE)\n",
		 how, e9810_qm_cur[QM_SESSION] == E9810_QS_LT_SHUTDOWN ?
		 "LT_SHUTDOWN" : "SHUTDOWN");
	local_irq_save(flags);
	locked = spin_trylock(&e9810_qm_lock);
	if (locked) {
		if (e9810_qm_cur[QM_SESSION] == E9810_QS_SHUTDOWN ||
		    e9810_qm_cur[QM_SESSION] == E9810_QS_LT_SHUTDOWN) {
			memcpy(w, e9810_qm_cur, sizeof(w));
			w[QM_SESSION] = E9810_QS_IDLE;
			ok = e9810_qm_commit(w);
		}
		spin_unlock(&e9810_qm_lock);
	}
	WRITE_ONCE(e9810_qm_close_how, NULL);
	local_irq_restore(flags);
	if (!ok)
		pr_emerg("C2 session NOT closed at the final %s point (%s): the next boot sees it as unclean\n",
			 how, locked ? "WRITE FAILED" : "record lock busy");
}

/* Persist drill bits as consumed before the drill runs. */
static bool e9810_qm_consume_drill(u32 bit)
{
	unsigned long flags;
	u32 w[QM_WORDS];
	bool ok;

	if (!e9810_qm)
		return false;
	spin_lock_irqsave(&e9810_qm_lock, flags);
	memcpy(w, e9810_qm_cur, sizeof(w));
	w[QM_DRILLS] |= bit;
	ok = e9810_qm_commit(w);
	spin_unlock_irqrestore(&e9810_qm_lock, flags);
	return ok;
}

static bool e9810_qm_drill_consumed(u32 bit)
{
	return READ_ONCE(e9810_qm_cur[QM_DRILLS]) & bit;
}

/* Record a loss; the next boot keeps the lost cluster(s) out of C2. */
static void e9810_qm_record(const struct cpumask *lost, u32 reason)
{
	unsigned long flags;
	unsigned int cpu;
	u32 w[QM_WORDS], f = 0;
	bool ok;

	if (!e9810_qm) {
		pr_emerg("C2-LOST: no quarantine marker (%s): nothing recorded\n",
			 e9810_qm_why);
		return;
	}
	for_each_cpu(cpu, lost)
		f |= e9810_cluster_of[cpu] ? E9810_QF_BIG : E9810_QF_LITTLE;
	if (reason == E9810_QR_DRILL)
		f |= E9810_QF_DRILL;

	spin_lock_irqsave(&e9810_qm_lock, flags);
	memcpy(w, e9810_qm_cur, sizeof(w));
	w[QM_FLAGS] |= f;
	w[QM_CPUS] |= (u32)(cpumask_bits(lost)[0] & 0xff);
	w[QM_UPTIME_MS] = (u32)e9810_boot_ms();
	w[QM_REASON] = reason;
	w[QM_LOSSES]++;
	ok = e9810_qm_commit(w);
	if (!ok)
		memcpy(w, e9810_qm_cur, sizeof(w));
	spin_unlock_irqrestore(&e9810_qm_lock, flags);

	pr_emerg("C2-LOST: quarantine record %s at %pa: seq=%u flags=%#x cpus=%#x reason=%s losses=%u session=%s; next boot: C2 quarantined (%s)\n",
		 ok ? "written and read back" : "WRITE FAILED (kept)",
		 &e9810_qm_phys, w[QM_SEQ], w[QM_FLAGS], w[QM_CPUS],
		 e9810_qr_str(w[QM_REASON]), w[QM_LOSSES],
		 e9810_qs_str(w[QM_SESSION]),
		 e9810_qm_flags_str(w[QM_FLAGS] |
			(w[QM_SESSION] != E9810_QS_IDLE ? E9810_QF_UNCLEAN : 0)));
}

static void e9810_reboot_fn(struct work_struct *work)
{
	if (e9810_lost_drill)
		pr_emerg("C2-LOST drill: FAIL: no watchdog reset %u s after petting stopped; orderly reboot\n",
			 E9810_DRILL_REBOOT_MS / 1000);
	orderly_reboot();
}

static void e9810_last_resort_fn(struct work_struct *work)
{
	pr_emerg("C2-LOST: still running %u s after the loss: emergency restart\n",
		 E9810_LAST_RESORT_MS / 1000);
	emergency_restart();
}

/*
 * The end of the chain: dump, record, disarm, the watchdog's final stop,
 * reboot. Process context, never called from panic. A drill does not
 * reboot by itself at first: it must prove that the watchdog resets the SoC.
 */
static void e9810_c2_lost(const struct cpumask *lost, u32 reason)
{
	bool drill = reason == E9810_QR_DRILL;
	unsigned int cpu;
	int t;

	for_each_cpu(cpu, lost) {
		struct e9810_regs r;
		u32 bit = INTR_GEN_BIT(e9810_phycpu[cpu]);

		e9810_read_regs(cpu, &r);
		pr_emerg("C2-LOST cpu%u%s: no answer from C2 (%s): stage=%s fw_for=%ums fw_gen=%u status=%#x config=%#x inform=%#x event_grp2=%d grp1_en=%d grp1_pend=%d grp2_en=%d grp2_pend=%d pmucal_cpu_lock=%s armed=%*pbl\n",
			 cpu, drill ? " (DRILL, simulated)" : "",
			 e9810_qr_str(reason), e9810_stage_str(r.stage),
			 r.stage == E9810_ST_FIRMWARE ?
			 jiffies_to_msecs(jiffies -
				READ_ONCE(per_cpu(e9810_fw_jiffies, cpu))) : 0,
			 READ_ONCE(per_cpu(e9810_fw_gen, cpu)),
			 r.status, r.config, r.inform,
			 !!(r.event_grp2 & bit), !!(r.grp1_en & bit),
			 !!(r.grp1_pend & bit), !!(r.grp2_en & bit),
			 !!(r.grp2_pend & bit),
			 cal_cpu_lock_is_held() ? "HELD" : "free",
			 cpumask_pr_args(&e9810_armed));
	}
	if (atomic_xchg(&e9810_lost_fired, 1)) {
		pr_emerg("C2-LOST: %*pbl: reboot already under way\n",
			 cpumask_pr_args(lost));
		return;
	}
	e9810_lost_drill = drill;
	/* A drill CPU is fine: the restart path may wait for it as usual. */
	if (!drill)
		cpumask_or(&e9810_lost, &e9810_lost, lost);

	e9810_qm_record(lost, reason);
	e9810_disable_deep(drill ? "C2-LOST drill" : "C2-LOST");
	e9810_disarm_all();

	/* The one shared final stop (crownlte-wdt.h); idempotent. */
	t = s3c2410wdt_crownlte_stop_petting(drill ? "C2-LOST drill" :
						     "C2-LOST");
	if (t == -EIO)
		pr_emerg("C2-LOST: watchdog final stop latched, but the counter does not run with the reset enabled now: relying on the reboot\n");
	else if (t < 0)
		pr_emerg("C2-LOST: no guarded watchdog (%d): relying on the reboot\n",
			 t);
	else
		pr_emerg("C2-LOST: watchdog final stop: never reloaded again, SoC reset within %d s\n",
			 t);

	queue_delayed_work_on(0, system_highpri_wq, &e9810_last_resort_work,
			      msecs_to_jiffies(E9810_LAST_RESORT_MS));
	if (drill) {
		pr_emerg("C2-LOST drill: waiting for the watchdog reset; orderly reboot only if none within %u s\n",
			 E9810_DRILL_REBOOT_MS / 1000);
		queue_delayed_work_on(0, system_highpri_wq, &e9810_reboot_work,
				      msecs_to_jiffies(E9810_DRILL_REBOOT_MS));
	} else {
		pr_emerg("C2-LOST: orderly reboot\n");
		orderly_reboot();
	}
}

static void e9810_det_ping_fn(void *info)
{
	struct e9810_det_ping *p = info;

	smp_store_release(&p->ack, READ_ONCE(p->sent));
}

/* Wait until every CPU in @wait answered; answered ones are cleared. */
static void e9810_det_wait(struct cpumask *wait, unsigned int ms)
{
	unsigned long end = jiffies + msecs_to_jiffies(ms);
	unsigned int cpu;

	for (;;) {
		for_each_cpu(cpu, wait) {
			struct e9810_det_ping *p = per_cpu_ptr(&e9810_det_ping,
							       cpu);

			if (smp_load_acquire(&p->ack) == READ_ONCE(p->sent))
				cpumask_clear_cpu(cpu, wait);
		}
		if (cpumask_empty(wait) || time_after(jiffies, end))
			return;
		usleep_range(1000, 2000);
	}
}

/*
 * Still in the very firmware entry the scan sampled: online, stage
 * firmware, same generation. A CPU that went offline, or left C2 (and maybe
 * entered it again) since, is not lost, whether or not it answered yet.
 */
static bool e9810_det_same_entry(unsigned int cpu)
{
	if (!cpu_online(cpu))
		return false;
	if (READ_ONCE(per_cpu(e9810_stage, cpu)) != E9810_ST_FIRMWARE)
		return false;
	/* Pairs with the smp_wmb() before the stage in e9810_enter_c2(). */
	smp_rmb();
	return READ_ONCE(per_cpu(e9810_fw_gen, cpu)) ==
	       per_cpu_ptr(&e9810_det_ping, cpu)->gen;
}

static void e9810_det_revalidate(struct cpumask *m)
{
	unsigned int cpu;

	for_each_cpu(cpu, m) {
		if (e9810_det_same_entry(cpu))
			continue;
		cpumask_clear_cpu(cpu, m);
		e9810_det_revalidated++;
		pr_info_ratelimited("C2 detector: cpu%u left the sampled firmware entry or went offline before answering: not lost\n",
				    cpu);
	}
}

/*
 * @cand sat in the firmware stage for more than E9810_DET_FW_MIN_MS. A CPU
 * that is merely in a long C2 period answers the ping within its exit
 * latency (< 1 ms). Lost: a ping was enqueued (or an earlier one is still
 * pending) and neither it nor, 200 ms later, a reschedule IPI followed by
 * another 200 ms got an answer, and the CPU is still online in the same
 * firmware entry. A ping that could not be enqueued (CPU went offline) is
 * no delivery and gives no verdict.
 */
static void e9810_det_check(const struct cpumask *cand, struct cpumask *lost)
{
	unsigned int cpu;
	int ret;

	cpumask_clear(lost);
	for_each_cpu(cpu, cand) {
		struct e9810_det_ping *p = per_cpu_ptr(&e9810_det_ping, cpu);

		/* An earlier, delivered ping still owns the csd: wait for it. */
		if (smp_load_acquire(&p->ack) != READ_ONCE(p->sent)) {
			cpumask_set_cpu(cpu, lost);
			continue;
		}
		WRITE_ONCE(p->sent, p->sent + 1);
		ret = smp_call_function_single_async(cpu, &p->csd);
		if (ret) {
			/* Not enqueued: nothing to wait for. */
			WRITE_ONCE(p->sent, p->sent - 1);
			e9810_det_enq_fail++;
			pr_info_ratelimited("C2 detector: cpu%u ping not enqueued (%d, offline?): no verdict this scan\n",
					    cpu, ret);
			continue;
		}
		e9810_det_pings++;
		cpumask_set_cpu(cpu, lost);
	}
	e9810_det_wait(lost, E9810_DET_PING_MS);
	e9810_det_revalidate(lost);
	if (cpumask_empty(lost))
		return;
	for_each_cpu(cpu, lost) {
		pr_warn("C2 detector: cpu%u silent for %u ms in the firmware stage; second ping\n",
			cpu, E9810_DET_PING_MS);
		wake_up_if_idle(cpu);
	}
	e9810_det_wait(lost, E9810_DET_PING_MS);
	e9810_det_revalidate(lost);
}

static void e9810_log_stats(void)
{
	unsigned int cpu;

	pr_info("c2 stats: armed=%*pbl lost=%*pbl deep_off=%d quarantine=%s ltimer_faults=%u det_scans=%llu det_pings=%llu det_enq_fail=%llu det_revalidated=%llu det_hp_unlocked=%llu uptime_ms=%llu\n",
		cpumask_pr_args(&e9810_armed), cpumask_pr_args(&e9810_lost),
		atomic_read(&e9810_deep_off),
		e9810_qm_flags_str(e9810_quarantine),
		exynos9810_mct_local_faults(), e9810_det_scans,
		e9810_det_pings, e9810_det_enq_fail, e9810_det_revalidated,
		e9810_det_hp_unlocked, e9810_boot_ms());
	for_each_possible_cpu(cpu) {
		struct e9810_stats st = per_cpu(e9810_stats, cpu);
		struct cpuidle_device *dev = per_cpu(cpuidle_devices, cpu);
		struct e9810_cluster *cl = &e9810_clusters[e9810_cluster_of[cpu]];
		u64 usage = 0, time_ms = 0;

		if (!st.c2_enter && !cpumask_test_cpu(cpu, &e9810_armed))
			continue;
		if (dev && cl->registered) {
			usage = dev->states_usage[cl->c2_idx].usage;
			time_ms = div_u64(dev->states_usage[cl->c2_idx].time_ns,
					  NSEC_PER_MSEC);
		}
		pr_info("c2 stats: cpu%u enter=%llu resumed=%llu early=%llu psci_fail=%llu last_err=%d unarmed=%llu guarded=%llu wfi_fallback=%llu stale_grp2=%llu stage=%s c2_usage=%llu c2_time_ms=%llu ltimer=%d no_ltimer=%llu\n",
			cpu, st.c2_enter, st.c2_resumed, st.early, st.psci_fail,
			st.last_err, st.unarmed, st.guarded, st.wfi_fallback,
			st.stale_grp2,
			e9810_stage_str(READ_ONCE(per_cpu(e9810_stage, cpu))),
			usage, time_ms, exynos9810_mct_local_active(cpu),
			st.no_ltimer);
	}
}

/*
 * Hotplug: the scan normally holds cpus_read_trylock(), so no CPU comes or
 * goes while it pings. It never waits for the hotplug lock: when hotplug
 * holds it, the scan is skipped; after E9810_DET_HP_SKIP_MAX skips in a row
 * (a hotplug operation that hangs, e.g. on a CPU lost in C2) it scans
 * without the lock, relying on the online and firmware-entry revalidation.
 */
#define E9810_DET_HP_SKIP_MAX		5

static void e9810_detect(struct work_struct *work)
{
	struct cpumask cand, lost;
	unsigned int cpu;
	bool locked;

	/*
	 * Not during a restart or power-off (the shutdown drain handles a CPU
	 * that does not come out); SYSTEM_SUSPEND passes by.
	 */
	if (READ_ONCE(e9810_shutdown) ||
	    system_state == SYSTEM_HALT || system_state == SYSTEM_POWER_OFF ||
	    system_state == SYSTEM_RESTART || atomic_read(&e9810_lost_fired))
		return;
	e9810_det_scans++;

	if (e9810_drill_cpu < nr_cpu_ids && !e9810_drill_fired &&
	    e9810_boot_ms() >= e9810_drill_at_ms) {
		e9810_drill_fired = true;
		/* Consumed before it runs: it never repeats on a later boot. */
		if (!e9810_qm_consume_drill(E9810_QD_LOST)) {
			pr_emerg("C2-LOST drill: SKIPPED: the consumed flag could not be persisted\n");
		} else {
			pr_emerg("C2-LOST drill: consumed (record seq %u); treating cpu%u as lost at %llu ms (simulated; the CPU is fine)\n",
				 READ_ONCE(e9810_qm_cur[QM_SEQ]),
				 e9810_drill_cpu, e9810_boot_ms());
			e9810_c2_lost(cpumask_of(e9810_drill_cpu),
				      E9810_QR_DRILL);
			return;
		}
	}

	locked = cpus_read_trylock();
	if (!locked && ++e9810_det_hp_skips < E9810_DET_HP_SKIP_MAX)
		goto next;
	if (locked) {
		e9810_det_hp_skips = 0;
	} else {
		e9810_det_hp_unlocked++;
		pr_warn_ratelimited("C2 detector: CPU hotplug busy for %u scans; scanning without the hotplug lock\n",
				    e9810_det_hp_skips);
	}

	cpumask_clear(&cand);
	for_each_online_cpu(cpu) {
		struct e9810_det_ping *p = per_cpu_ptr(&e9810_det_ping, cpu);
		unsigned long since;

		if (!cpu || cpumask_test_cpu(cpu, &e9810_lost) ||
		    cpumask_test_cpu(cpu, &e9810_selftest_cpus))
			continue;
		if (READ_ONCE(per_cpu(e9810_stage, cpu)) != E9810_ST_FIRMWARE)
			continue;
		/* Pairs with the smp_wmb() before the stage in e9810_enter_c2(). */
		smp_rmb();
		p->gen = READ_ONCE(per_cpu(e9810_fw_gen, cpu));
		since = READ_ONCE(per_cpu(e9810_fw_jiffies, cpu));
		if (time_before(jiffies, since +
				msecs_to_jiffies(E9810_DET_FW_MIN_MS)))
			continue;
		cpumask_set_cpu(cpu, &cand);
	}
	cpumask_clear(&lost);
	if (!cpumask_empty(&cand))
		e9810_det_check(&cand, &lost);
	if (locked)
		cpus_read_unlock();
	if (!cpumask_empty(&lost)) {
		e9810_c2_lost(&lost, E9810_QR_DETECTOR);
		return;
	}

next:
	if (time_after_eq(jiffies, e9810_stats_next)) {
		e9810_stats_next = jiffies +
				   msecs_to_jiffies(E9810_STATS_PERIOD_MS);
		e9810_log_stats();
	}
	queue_delayed_work_on(0, system_highpri_wq, &e9810_det_work,
			      msecs_to_jiffies(E9810_DET_PERIOD_MS));
}

/* samsung,c2-loss-drill-ms ran out without a loss. */
static void e9810_loss_drill_end(struct work_struct *work)
{
	struct cpumask m;

	if (atomic_read(&e9810_lost_fired))
		return;
	cpumask_clear(&m);
	cpumask_set_cpu(1, &m);
	cpumask_set_cpu(2, &m);
	cpumask_set_cpu(3, &m);
	e9810_disarm_mask(&m);
	WRITE_ONCE(e9810_little_c2, e9810_little_c2_cfg);
	WRITE_ONCE(e9810_little_limit, e9810_little_limit_cfg);
	pr_warn("C2 loss drill: no core lost in %u ms (INCONCLUSIVE); cpu1-3 disarmed\n",
		e9810_loss_drill_ms);
}

/* Arms cpu1-3, the configuration that lost a LITTLE core in q22b/q40/q43. */
static void e9810_loss_drill_start(void)
{
	struct cpumask m;
	const char *why;

	if (e9810_qm_drill_consumed(E9810_QD_LOSS)) {
		pr_warn("C2 loss drill: already run on an earlier boot (consumed); rerun: echo 1 > c2_drill_rearm (root)\n");
		return;
	}
	/* Consumed before anything is armed: it never repeats by itself. */
	if (!e9810_qm_consume_drill(E9810_QD_LOSS)) {
		pr_warn("C2 loss drill: skipped: the consumed flag could not be persisted\n");
		return;
	}
	cpumask_clear(&m);
	cpumask_set_cpu(1, &m);
	cpumask_set_cpu(2, &m);
	cpumask_set_cpu(3, &m);
	WRITE_ONCE(e9810_little_c2, true);
	WRITE_ONCE(e9810_little_limit, 0);
	why = e9810_arm(&m);
	if (why) {
		WRITE_ONCE(e9810_little_c2, e9810_little_c2_cfg);
		WRITE_ONCE(e9810_little_limit, e9810_little_limit_cfg);
		pr_warn("C2 loss drill: cannot arm 1-3 (drill consumed): %s\n",
			why);
		return;
	}
	pr_emerg("C2 loss drill: armed 1-3 (LITTLE, no concurrency limit) for %u ms at %llu ms: a lost core is expected; it must end in C2-LOST, a reboot and 'C2 quarantined' on the next boot\n",
		 e9810_loss_drill_ms, e9810_boot_ms());
	queue_delayed_work_on(0, system_highpri_wq, &e9810_loss_drill_work,
			      msecs_to_jiffies(e9810_loss_drill_ms));
}

/* Set by c2_disarm: boot arming never arms anything again this boot. */
static bool e9810_arm_killed;

/*
 * exynos9810-mct-bc fault work (process context, CPU0): the local timer of
 * @cpu was latched failed, which closed that CPU's C2 gate at once. Now
 * LITTLE C2 goes off for the rest of the boot, then the fault is recorded
 * (E9810_QF_LTIMER): later boots take no local timers and arm no LITTLE
 * C2 until a clear. If the record cannot be written, the session is never
 * closed on this boot, so the next boot still finds LT_ARMED or
 * LT_SHUTDOWN (UNCLEAN + LTIMER). The replacement by the arch timer
 * follows in the caller.
 */
static void e9810_lt_fault(unsigned int cpu, const char *why)
{
	unsigned long flags;
	u32 w[QM_WORDS];
	bool ok = false;

	WRITE_ONCE(e9810_lt_faulted, true);
	e9810_disarm_mask(&e9810_clusters[0].cpus);
	if (e9810_qm) {
		spin_lock_irqsave(&e9810_qm_lock, flags);
		memcpy(w, e9810_qm_cur, sizeof(w));
		w[QM_FLAGS] |= E9810_QF_LTIMER;
		if (w[QM_REASON] == E9810_QR_NONE) {
			w[QM_REASON] = E9810_QR_LTIMER;
			w[QM_UPTIME_MS] = (u32)e9810_boot_ms();
		}
		ok = e9810_qm_commit(w);
		if (!ok)
			memcpy(w, e9810_qm_cur, sizeof(w));
		spin_unlock_irqrestore(&e9810_qm_lock, flags);
	}
	if (!ok)
		atomic_set(&e9810_lt_block_close, 1);
	pr_emerg("C2: MCT local timer FAULT on cpu%u (%s): LITTLE C2 disarmed for this boot (armed now %*pbl); quarantine record %s (seq %u flags %#x session %s)%s\n",
		 cpu, why, cpumask_pr_args(&e9810_armed),
		 ok ? "written and read back" : "WRITE FAILED",
		 w[QM_SEQ], w[QM_FLAGS], e9810_qs_str(w[QM_SESSION]),
		 ok ? "; next boots: no local timer takeover, LITTLE C2 off" :
		 "; the session stays open: next boot UNCLEAN + LTIMER");
}

/*
 * The failed local timer of @cpu could not be replaced: @cpu may have no
 * working tick. The S0 end of chain without a loss: the session stays
 * open (next boot UNCLEAN + LTIMER: all C2 off, no takeover), everything
 * disarmed, the watchdog's final stop, orderly_reboot(), and
 * emergency_restart() as the last resort.
 */
static void e9810_lt_fatal(unsigned int cpu, const char *why)
{
	int t;

	atomic_set(&e9810_lt_block_close, 1);
	pr_emerg("C2: MCT local timer of cpu%u FAILED and was NOT replaced (%s): fatal path; the record keeps its local-timer session (next boot: UNCLEAN + LTIMER, all C2 off)\n",
		 cpu, why);
	e9810_disable_deep("MCT local timer fault, not replaced");
	e9810_disarm_all();
	t = s3c2410wdt_crownlte_stop_petting("MCT local timer fault");
	if (t == -EIO)
		pr_emerg("C2 ltimer fatal: watchdog final stop latched, but the counter does not run with the reset enabled now: relying on the reboot\n");
	else if (t < 0)
		pr_emerg("C2 ltimer fatal: no guarded watchdog (%d): relying on the reboot\n",
			 t);
	else
		pr_emerg("C2 ltimer fatal: watchdog final stop: never reloaded again, SoC reset within %d s\n",
			 t);
	queue_delayed_work_on(0, system_highpri_wq, &e9810_last_resort_work,
			      msecs_to_jiffies(E9810_LAST_RESORT_MS));
	pr_emerg("C2 ltimer fatal: orderly reboot\n");
	orderly_reboot();
}

static const struct exynos9810_mct_local_ops e9810_lt_ops = {
	.fault = e9810_lt_fault,
	.fatal = e9810_lt_fatal,
};

/* Why the LITTLE local timer takeover may not run now. e9810_arm_lock held. */
static const char *e9810_lt_refused(void)
{
	if (e9810_setup_why)
		return e9810_setup_why;
	if (!e9810_qm)
		return "no quarantine record to hold the local-timer session";
	if (READ_ONCE(e9810_shutdown) ||
	    system_state == SYSTEM_HALT || system_state == SYSTEM_POWER_OFF ||
	    system_state == SYSTEM_RESTART)
		return "restart or power-off under way";
	if (e9810_quarantine & E9810_QF_ALL_OFF)
		return "C2 quarantined (all)";
	if (e9810_quarantine & E9810_QF_LITTLE_OFF)
		return "C2 quarantined (LITTLE or local timers)";
	if (!e9810_little_c2)
		return "LITTLE C2 disabled";
	if (atomic_read(&e9810_deep_off))
		return "deep idle disabled";
	if (atomic_read(&e9810_lost_fired) || READ_ONCE(e9810_arm_killed))
		return "C2 lost or disarmed on this boot";
	if (READ_ONCE(e9810_lt_faulted))
		return "an MCT local timer failed on this boot";
	return NULL;
}

/*
 * Once per boot, from the boot arming work, before the LITTLE CPUs are
 * armed: persist LT_ARMED, then let exynos9810-mct-bc self-check and
 * register the local timers of the LITTLE CPUs to arm. Never CPU0 and
 * never the big cores: they keep the arch timer + G0 broadcast (the
 * hardware-proven S1a path). The tick core keeps both kinds side by side:
 * a local timer without C3STOP takes its CPU out of the broadcast masks,
 * the arch timer CPUs stay under the G0 broadcast. e9810_arm_lock is not
 * held across the takeover (it takes the hotplug lock); e9810_lt_busy
 * keeps the final point from closing the session meanwhile.
 */
static void e9810_lt_takeover(void)
{
	struct cpumask want, active;
	const char *why;

	e9810_lt_tried = true;
	cpumask_and(&want, &e9810_boot_pending, &e9810_clusters[0].cpus);
	cpumask_clear_cpu(0, &want);
	if (cpumask_empty(&want))
		return;

	mutex_lock(&e9810_arm_lock);
	why = e9810_lt_refused();
	if (!why)
		why = e9810_qm_session_ltimer();
	if (!why)
		WRITE_ONCE(e9810_lt_busy, true);
	mutex_unlock(&e9810_arm_lock);
	if (why) {
		pr_warn("MCT local timer takeover of %*pbl not done: %s; they keep the arch timer and are not armed\n",
			cpumask_pr_args(&want), why);
		return;
	}
	pr_info("C2 local-timer session persisted (record seq %u, LT_ARMED): MCT local timer takeover of %*pbl (CPU0 and big keep the arch timer + broadcast)\n",
		e9810_qm_cur[QM_SEQ], cpumask_pr_args(&want));
	why = exynos9810_mct_local_takeover(&want, &e9810_lt_ops, &active);
	WRITE_ONCE(e9810_lt_busy, false);
	if (why)
		pr_warn("MCT local timer takeover: %s; LITTLE CPUs without a local tick are not armed\n",
			why);
	else
		pr_info("MCT local timer takeover: %*pbl tick on their local timer\n",
			cpumask_pr_args(&active));
}

/*
 * Boot arming, one CPU at a time: a CPU that is offline or waits for the
 * MCT broadcast does not hold back the others. Both are retried every
 * second for up to E9810_ARM_RETRY_MAX_MS; after that a CPU that is still
 * offline stays pending and is armed when it comes online (cpuhp callback
 * -> this work), one still waiting for the MCT broadcast is dropped. Any
 * other refusal is final for that CPU.
 */
static void e9810_arm_boot_fn(struct work_struct *work)
{
	struct cpumask todo, armed, wait_mct, wait_off;
	const char *why;
	unsigned int cpu;

	if (atomic_read(&e9810_lost_fired) || READ_ONCE(e9810_arm_killed))
		return;
	if (!e9810_arm_retrying)
		e9810_arm_deadline = jiffies +
				     msecs_to_jiffies(E9810_ARM_RETRY_MAX_MS);
	/* S2: the LITTLE CPUs need their local tick before they can arm. */
	if (!e9810_lt_tried && e9810_little_ltimer &&
	    exynos9810_mct_bc_ready())
		e9810_lt_takeover();

	cpumask_clear(&armed);
	cpumask_clear(&wait_mct);
	cpumask_clear(&wait_off);
	cpumask_copy(&todo, &e9810_boot_pending);
	for_each_cpu(cpu, &todo) {
		why = e9810_arm(cpumask_of(cpu));
		if (!why) {
			cpumask_clear_cpu(cpu, &e9810_boot_pending);
			cpumask_set_cpu(cpu, &armed);
		} else if (why == e9810_why_no_mct) {
			cpumask_set_cpu(cpu, &wait_mct);
		} else if (why == e9810_why_offline) {
			cpumask_set_cpu(cpu, &wait_off);
		} else {
			pr_warn("C2 boot arming of cpu%u refused: %s\n",
				cpu, why);
			cpumask_clear_cpu(cpu, &e9810_boot_pending);
		}
	}
	if (!cpumask_empty(&armed))
		pr_info("C2 armed for %*pbl at %llu ms (MCT broadcast ready at %llu ms); armed now %*pbl, pending %*pbl\n",
			cpumask_pr_args(&armed), e9810_boot_ms(),
			exynos9810_mct_bc_ready_ms(),
			cpumask_pr_args(&e9810_armed),
			cpumask_pr_args(&e9810_boot_pending));

	if (!READ_ONCE(e9810_arm_done) &&
	    (!cpumask_empty(&wait_mct) || !cpumask_empty(&wait_off)) &&
	    time_before(jiffies, e9810_arm_deadline)) {
		if (!e9810_arm_retrying)
			pr_info("C2 boot arming: %*pbl wait for the MCT tick broadcast, %*pbl are offline; retry every %u ms for up to %u ms\n",
				cpumask_pr_args(&wait_mct),
				cpumask_pr_args(&wait_off),
				E9810_ARM_RETRY_MS, E9810_ARM_RETRY_MAX_MS);
		e9810_arm_retrying = true;
		queue_delayed_work_on(0, system_highpri_wq, &e9810_arm_work,
				      msecs_to_jiffies(E9810_ARM_RETRY_MS));
		return;
	}
	for_each_cpu(cpu, &wait_mct) {
		pr_warn("C2 boot arming of cpu%u refused: %s after %u ms\n",
			cpu, e9810_why_no_mct, E9810_ARM_RETRY_MAX_MS);
		cpumask_clear_cpu(cpu, &e9810_boot_pending);
	}
	if (!READ_ONCE(e9810_arm_done)) {
		if (!cpumask_empty(&wait_off))
			pr_info("C2 boot arming: %*pbl offline; armed when they come online\n",
				cpumask_pr_args(&wait_off));
		WRITE_ONCE(e9810_arm_done, true);
		if (e9810_loss_drill_ms)
			e9810_loss_drill_start();
	}
}

/* cpuhp online callback (on @cpu): a pending boot-arming CPU is back. */
static void e9810_boot_arm_online(unsigned int cpu)
{
	if (READ_ONCE(e9810_arm_done) && !READ_ONCE(e9810_arm_killed) &&
	    cpumask_test_cpu(cpu, &e9810_boot_pending))
		queue_delayed_work_on(0, system_highpri_wq, &e9810_arm_work, 0);
}

static void e9810_boot_arm_setup(struct device_node *np)
{
	u32 bits = 0, delay = 0;
	u64 now;

	of_property_read_u32(np, "samsung,c2-boot-cpus", &bits);
	if (!of_property_read_u32(np, "samsung,c2-loss-drill-ms",
				  &e9810_loss_drill_ms))
		e9810_loss_drill_ms = clamp_t(u32, e9810_loss_drill_ms, 1000,
					      E9810_DRILL_MS_MAX);
	cpumask_clear(&e9810_boot_mask);
	bitmap_from_arr32(cpumask_bits(&e9810_boot_mask), &bits, 32);
	cpumask_and(&e9810_boot_mask, &e9810_boot_mask, cpu_possible_mask);
	if (cpumask_empty(&e9810_boot_mask) && !e9810_loss_drill_ms)
		return;

	if (!e9810_qm) {
		pr_warn("C2 boot arming of %*pbl%s refused: no C2 quarantine marker (%s); a lost core could loop the reboots\n",
			cpumask_pr_args(&e9810_boot_mask),
			e9810_loss_drill_ms ? " and the loss drill" : "",
			e9810_qm_why);
		return;
	}
	if (e9810_quarantine & E9810_QF_ALL_OFF) {
		pr_warn("C2 quarantined (all, flags %#x): boot arming of %*pbl%s skipped\n",
			e9810_quarantine, cpumask_pr_args(&e9810_boot_mask),
			e9810_loss_drill_ms ? " and the loss drill" : "");
		return;
	}
	if (e9810_loss_drill_ms && e9810_qm_drill_consumed(E9810_QD_LOSS)) {
		pr_warn("C2 loss drill: already run on an earlier boot (consumed); skipped; rerun: echo 1 > c2_drill_rearm (root)\n");
		e9810_loss_drill_ms = 0;
	}
	if (e9810_quarantine & E9810_QF_LITTLE_OFF) {
		if (e9810_loss_drill_ms)
			pr_warn("C2 quarantined (LITTLE): loss drill skipped\n");
		e9810_loss_drill_ms = 0;
		if (cpumask_intersects(&e9810_boot_mask,
				       &e9810_clusters[0].cpus)) {
			cpumask_andnot(&e9810_boot_mask, &e9810_boot_mask,
				       &e9810_clusters[0].cpus);
			pr_warn("C2 quarantined (LITTLE): boot arming reduced to %*pbl\n",
				cpumask_pr_args(&e9810_boot_mask));
		}
	}
	if (cpumask_empty(&e9810_boot_mask) && !e9810_loss_drill_ms)
		return;
	cpumask_copy(&e9810_boot_pending, &e9810_boot_mask);

	of_property_read_u32(np, "samsung,c2-arm-delay-ms", &delay);
	delay = min_t(u32, delay, E9810_ARM_DELAY_MAX_MS);
	now = e9810_boot_ms();
	pr_info("C2 boot arming of %*pbl%s at %u ms boot time\n",
		cpumask_pr_args(&e9810_boot_mask),
		e9810_loss_drill_ms ? " + loss drill cpu1-3" : "", delay);
	queue_delayed_work_on(0, system_highpri_wq, &e9810_arm_work,
			      delay > now ? msecs_to_jiffies(delay - now) : 0);
}

static void e9810_drill_setup(struct device *dev)
{
	u32 v[2];

	if (of_property_read_u32_array(dev->of_node, "samsung,c2-lost-drill",
				       v, 2))
		return;
	if (!v[0] || v[0] >= nr_cpu_ids || v[1] > E9810_DRILL_MS_MAX) {
		pr_warn("C2-LOST drill: ignored: cpu%u at %u ms (cpu 1..%u, at most %u ms)\n",
			v[0], v[1], nr_cpu_ids - 1, E9810_DRILL_MS_MAX);
		return;
	}
	if (!e9810_qm) {
		pr_warn("C2-LOST drill: refused: no quarantine marker (%s)\n",
			e9810_qm_why);
		return;
	}
	if (e9810_qm_drill_consumed(E9810_QD_LOST)) {
		pr_warn("C2-LOST drill: already run on an earlier boot (consumed, record seq %u): skipped; rerun: echo 1 > /sys/bus/platform/devices/%s/c2_drill_rearm (root)\n",
			e9810_qm_cur[QM_SEQ], dev_name(dev));
		return;
	}
	e9810_drill_cpu = v[0];
	e9810_drill_at_ms = v[1];
	pr_warn("C2-LOST drill ENABLED: cpu%u is treated as lost at %u ms boot time (simulated, once: consumed in the record first); expect the record, the watchdog final stop and its reset, then 'C2 quarantined (all)' on the next boot of this kernel\n",
		e9810_drill_cpu, e9810_drill_at_ms);
}

/* Root-only sysfs under the cpuidle platform device. */
static ssize_t c2_armed_show(struct device *dev, struct device_attribute *a,
			     char *buf)
{
	return sysfs_emit(buf, "%*pbl\n", cpumask_pr_args(&e9810_armed));
}
static DEVICE_ATTR_ADMIN_RO(c2_armed);

static ssize_t c2_lost_show(struct device *dev, struct device_attribute *a,
			    char *buf)
{
	return sysfs_emit(buf, "%*pbl\n", cpumask_pr_args(&e9810_lost));
}
static DEVICE_ATTR_ADMIN_RO(c2_lost);

static ssize_t c2_stats_show(struct device *dev, struct device_attribute *a,
			     char *buf)
{
	unsigned int cpu;
	int n = 0;

	for_each_possible_cpu(cpu) {
		struct e9810_stats st = per_cpu(e9810_stats, cpu);

		n += sysfs_emit_at(buf, n, "cpu%u armed=%d stage=%s fw_gen=%u enter=%llu resumed=%llu early=%llu psci_fail=%llu unarmed=%llu guarded=%llu wfi_fallback=%llu stale_grp2=%llu last_err=%d ltimer=%d no_ltimer=%llu\n",
				   cpu, cpumask_test_cpu(cpu, &e9810_armed),
				   e9810_stage_str(READ_ONCE(per_cpu(e9810_stage, cpu))),
				   READ_ONCE(per_cpu(e9810_fw_gen, cpu)),
				   st.c2_enter, st.c2_resumed, st.early,
				   st.psci_fail, st.unarmed, st.guarded,
				   st.wfi_fallback, st.stale_grp2, st.last_err,
				   exynos9810_mct_local_active(cpu),
				   st.no_ltimer);
	}
	n += sysfs_emit_at(buf, n, "deep_off=%d ltimer_faults=%u ltimer_close_blocked=%d det_scans=%llu det_pings=%llu det_enq_fail=%llu det_revalidated=%llu det_hp_unlocked=%llu boot_pending=%*pbl\n",
			   atomic_read(&e9810_deep_off),
			   exynos9810_mct_local_faults(),
			   e9810_lt_close_blocked(), e9810_det_scans,
			   e9810_det_pings, e9810_det_enq_fail,
			   e9810_det_revalidated, e9810_det_hp_unlocked,
			   cpumask_pr_args(&e9810_boot_pending));
	return n;
}
static DEVICE_ATTR_ADMIN_RO(c2_stats);

static ssize_t c2_quarantine_show(struct device *dev,
				  struct device_attribute *a, char *buf)
{
	u32 w[QM_WORDS], s[E9810_QM_SLOTS][QM_WORDS];
	unsigned long flags;
	unsigned int i;

	if (!e9810_qm)
		return sysfs_emit(buf, "marker=none (%s) in_force=%s\n",
				  e9810_qm_why,
				  e9810_qm_flags_str(e9810_quarantine));
	spin_lock_irqsave(&e9810_qm_lock, flags);
	memcpy(w, e9810_qm_cur, sizeof(w));
	for (i = 0; i < E9810_QM_SLOTS; i++)
		e9810_qm_read(i, s[i]);
	spin_unlock_irqrestore(&e9810_qm_lock, flags);
	return sysfs_emit(buf, "record seq=%u version=%u flags=%#x (%s%s%s%s%s) session=%s cpus=%#x uptime_ms=%u reason=%s losses=%u boots=%u drills_consumed=%#x trial_floor=%#x slot0=%s/%u slot1=%s/%u trusted_at_boot=%d write_failed=%d in_force=%s (%#x)\n",
			  w[QM_SEQ], w[QM_VERSION], w[QM_FLAGS],
			  e9810_qm_flags_str(w[QM_FLAGS]),
			  (w[QM_FLAGS] & E9810_QF_UNCLEAN) ? ", unclean" : "",
			  (w[QM_FLAGS] & E9810_QF_UNINIT) ? ", uninit" : "",
			  (w[QM_FLAGS] & E9810_QF_LTIMER) ? ", ltimer" : "",
			  (w[QM_FLAGS] & E9810_QF_DRILL) ? ", drill" : "",
			  e9810_qs_str(w[QM_SESSION]),
			  w[QM_CPUS], w[QM_UPTIME_MS], e9810_qr_str(w[QM_REASON]),
			  w[QM_LOSSES], w[QM_BOOTS], w[QM_DRILLS], w[QM_TRIAL],
			  e9810_qm_valid(s[0], 0) ? "valid" : "invalid",
			  s[0][QM_SEQ],
			  e9810_qm_valid(s[1], 1) ? "valid" : "invalid",
			  s[1][QM_SEQ], e9810_qm_found, e9810_qm_bad,
			  e9810_qm_flags_str(e9810_quarantine), e9810_quarantine);
}
static DEVICE_ATTR_ADMIN_RO(c2_quarantine);

/*
 * The explicit clear (root): writes a record with no quarantine flags
 * (UNINIT and UNCLEAN included). The session word, the consumed drills and
 * the trial floor are kept. The record stays trusted (both slots valid and
 * chained), so the clear takes effect on the next
 * boot; this boot keeps the quarantine it started with.
 */
static ssize_t c2_quarantine_clear_store(struct device *dev,
					 struct device_attribute *a,
					 const char *buf, size_t count)
{
	unsigned long flags;
	u32 w[QM_WORDS];
	bool v, ok;

	if (kstrtobool(buf, &v) || !v)
		return -EINVAL;
	if (!e9810_qm)
		return -ENODEV;
	spin_lock_irqsave(&e9810_qm_lock, flags);
	memcpy(w, e9810_qm_cur, sizeof(w));
	w[QM_FLAGS] = 0;
	w[QM_CPUS] = 0;
	w[QM_UPTIME_MS] = 0;
	w[QM_REASON] = E9810_QR_NONE;
	ok = e9810_qm_commit(w);
	spin_unlock_irqrestore(&e9810_qm_lock, flags);
	if (!ok)
		return -EIO;
	pr_warn("C2 quarantine record cleared from sysfs (seq %u); takes effect on the next boot (in force now: %s)\n",
		w[QM_SEQ], e9810_qm_flags_str(e9810_quarantine));
	return count;
}
static DEVICE_ATTR_WO(c2_quarantine_clear);

/* Root: the drills run again on the next boot. */
static ssize_t c2_drill_rearm_store(struct device *dev,
				    struct device_attribute *a,
				    const char *buf, size_t count)
{
	unsigned long flags;
	u32 w[QM_WORDS];
	bool v, ok;

	if (kstrtobool(buf, &v) || !v)
		return -EINVAL;
	if (!e9810_qm)
		return -ENODEV;
	spin_lock_irqsave(&e9810_qm_lock, flags);
	memcpy(w, e9810_qm_cur, sizeof(w));
	w[QM_DRILLS] = 0;
	ok = e9810_qm_commit(w);
	spin_unlock_irqrestore(&e9810_qm_lock, flags);
	if (!ok)
		return -EIO;
	pr_warn("C2 drills re-armed from sysfs (seq %u): they run on the next boot whose DT asks for them\n",
		w[QM_SEQ]);
	return count;
}
static DEVICE_ATTR_WO(c2_drill_rearm);

/* Kill switch: disarm every CPU now (until the next boot). */
static ssize_t c2_disarm_store(struct device *dev, struct device_attribute *a,
			       const char *buf, size_t count)
{
	bool v;

	if (kstrtobool(buf, &v) || !v)
		return -EINVAL;
	/* No boot arming retry or online callback may re-arm afterwards. */
	WRITE_ONCE(e9810_arm_killed, true);
	cancel_delayed_work_sync(&e9810_arm_work);
	e9810_disarm_all();
	pr_warn("C2 disarmed on every CPU from sysfs\n");
	return count;
}
static DEVICE_ATTR_WO(c2_disarm);

static struct attribute *e9810_attrs[] = {
	&dev_attr_c2_armed.attr,
	&dev_attr_c2_lost.attr,
	&dev_attr_c2_stats.attr,
	&dev_attr_c2_quarantine.attr,
	&dev_attr_c2_quarantine_clear.attr,
	&dev_attr_c2_drill_rearm.attr,
	&dev_attr_c2_disarm.attr,
	NULL
};

static const struct attribute_group e9810_attr_group = {
	.attrs = e9810_attrs,
};

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
	/* This phase watches its CPUs itself; the runtime detector skips them. */
	cpumask_or(&e9810_selftest_cpus, &e9810_selftest_cpus, &ph->cpus);
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
		/* Not recovered: the S0 chain (marker, watchdog, reboot). */
		if (!cpumask_empty(&lost))
			e9810_c2_lost(&lost, E9810_QR_SELFTEST);
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
	cpumask_andnot(&e9810_selftest_cpus, &e9810_selftest_cpus, &ph->cpus);
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

static int e9810_cpuidle_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	unsigned int cpu;
	u32 lim;
	int ret;

	INIT_DELAYED_WORK(&e9810_lost_work, e9810_lost_remind);
	INIT_DELAYED_WORK(&e9810_det_work, e9810_detect);
	INIT_DELAYED_WORK(&e9810_arm_work, e9810_arm_boot_fn);
	INIT_DELAYED_WORK(&e9810_loss_drill_work, e9810_loss_drill_end);
	INIT_DELAYED_WORK(&e9810_reboot_work, e9810_reboot_fn);
	INIT_DELAYED_WORK(&e9810_last_resort_work, e9810_last_resort_fn);
	for_each_possible_cpu(cpu) {
		struct e9810_det_ping *p = per_cpu_ptr(&e9810_det_ping, cpu);

		INIT_CSD(&p->csd, e9810_det_ping_fn, p);
	}

	/* Before anything can be armed. */
	if (!of_property_read_u32(np, "samsung,c2-little-limit", &lim) &&
	    lim <= 4) {
		e9810_little_limit_cfg = lim;
		WRITE_ONCE(e9810_little_limit, lim);
	}
	e9810_little_c2 = of_property_read_bool(np, "samsung,c2-little-enable");
	e9810_little_c2_cfg = e9810_little_c2;
	e9810_little_ltimer = of_property_read_bool(np,
					"samsung,c2-little-local-timer");
	pr_info("LITTLE C2 %s, cluster-0 C2 limit %d%s%s\n",
		e9810_little_c2 ? "enabled" : "disabled (WFI only)",
		e9810_little_limit_cfg,
		e9810_little_limit_cfg ? "" : " (none)",
		e9810_little_ltimer ?
		", only on a CPU that ticks on its MCT local timer" : "");
	/* The quarantine an earlier boot recorded applies to every arming. */
	e9810_qm_init(&pdev->dev);
	e9810_setup_why = e9810_setup(pdev);
	if (e9810_setup_why)
		pr_warn("deep idle not available, WFI only: %s\n",
			e9810_setup_why);
	else
		pr_info("registered: cluster0=%d cluster1=%d, C2 disarmed on every CPU\n",
			e9810_clusters[0].registered,
			e9810_clusters[1].registered);

	ret = devm_device_add_group(&pdev->dev, &e9810_attr_group);
	if (ret)
		pr_warn("sysfs c2_* files not created (%d)\n", ret);

	if (!e9810_setup_why) {
		/* The S0 detector runs whether or not anything is armed. */
		e9810_drill_setup(&pdev->dev);
		queue_delayed_work_on(0, system_highpri_wq, &e9810_det_work,
				      msecs_to_jiffies(E9810_DET_PERIOD_MS));
		pr_info("C2-LOST detector running: every %u ms, ping after %u ms in firmware, %u ms per ping; watchdog stop-petting + reboot on loss\n",
			E9810_DET_PERIOD_MS, E9810_DET_FW_MIN_MS,
			E9810_DET_PING_MS);
		e9810_boot_arm_setup(np);
	}

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
