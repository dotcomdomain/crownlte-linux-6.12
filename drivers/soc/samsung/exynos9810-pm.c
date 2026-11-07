// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 9810 (crownlte) system suspend, stage S0: everything up to, but
 * never including, the firmware sleep call.
 *
 * Design: gekko roadmap/suspend-design.md (sections 3.1, 3.2, 5 S0a-S0d).
 * Self-tests, expected lines and recovery: Documentation/crownlte/suspend-s0.md.
 *
 * Port of the 4.9 vendor flow (K49 drivers/soc/samsung/exynos-pm.c and the
 * system/hotplug half of exynos-powermode.c), cut down to what S0 needs:
 *
 *  - CPU hotplug hooks (4.9 exynos_hotplug_{in,out}_callback): the dying
 *    CPU runs cal_cpu_disable() and, if it empties its cluster,
 *    cal_cluster_disable(); a starting CPU runs cal_cluster_enable() if its
 *    cluster was empty, then cal_cpu_enable(). After CPU_DEAD the core's
 *    PMU status is polled and logged (4.9 BUG()s after 2 s; here it is
 *    only recorded).
 *  - platform_suspend_ops + syscore_ops for suspend-to-RAM (mode 8,
 *    SYS_SLEEP): CP gate, wakeup masks (DT, 4.9 values), IDLE_IP masks,
 *    cal_pm_enter / cal_pm_exit / cal_pm_earlywakeup.
 *
 * S0 construction: nothing here can make the SoC sleep.
 *  1. There is no CPU_SUSPEND call. .enter runs cpu_suspend() with a dry
 *     finisher that records what the firmware call would have been (fn
 *     0xc4000001, power_state 0x03000000, entry __pa(cpu_resume)) and
 *     returns without any SMC. arm64 cpu_suspend() reports a finisher that
 *     returns as -EOPNOTSUPP: the not-slept path, which a firmware early
 *     wake takes as well; resume then runs cal_pm_earlywakeup().
 *  2. The syscore step uses cal_pm_enter_unarmed(): the 4.9 enter sequence
 *     without the INTR_GEN GRP2_INTR_BID_ENABLE write. That bit is how the
 *     CPU0 "core down" event reaches the APM firmware, which owns the SoC
 *     and MIF sleep sequencers. It is read back as 0 before syscore
 *     suspend returns; otherwise the step undoes itself and fails.
 *  3. No PMIC, charger or UFS command is issued; S0c/S0d refuse to run if
 *     a UFS host is bound or a charger watchdog kick is enabled in DT.
 *
 * Stages (DT samsung,suspend-s0, kthread on CPU0, from ~20 s uptime):
 *  S0a read-only register snapshot;
 *  S0b CPU hotplug: each core alone, the BIG cluster, then LITTLE pairs,
 *      all three LITTLE cores and all non-boot CPUs together (the C2 hang
 *      needs >= 3 LITTLE cores), with lost-core detection;
 *  S0c pm_test freezer .. core (syscore runs cal_pm_enter_unarmed and
 *      cal_pm_exit, .enter is skipped by pm_test);
 *  S0d full suspend-to-RAM with the dry .enter and a pending self-IPI.
 * Each stage runs only if the previous one passed.
 *
 * Recovery: the CL0 watchdog stays armed through suspend
 * (samsung,keep-armed-in-suspend, 60 s). A CPU lost while the kernel still
 * counts it online, or a cross-CPU request that times out, stops the
 * kernel's petting (other CPUs would otherwise keep the watchdog quiet
 * forever, as in trial c2diag-b). The DT deadline is installed at probe,
 * before any cross-CPU request, inside the watchdog's ping path: after it
 * no CPU can reload the counter. S0b and S0c/S0d start only with a live,
 * reset-enabled, still-petted watchdog carrying that deadline.
 */

#define pr_fmt(fmt) "suspend s0: " fmt

#include <linux/cpu.h>
#include <linux/cpuhotplug.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sched/clock.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <linux/workqueue.h>

#include <asm/arch_timer.h>
#include <asm/cputype.h>
#include <asm/smp.h>
#include <asm/smp_plat.h>
#include <asm/suspend.h>

#include <soc/samsung/acpm_ipc_ctrl.h>
#include <soc/samsung/cal-if.h>
#include <soc/samsung/crownlte-wdt.h>

#define E9810_NR_PHYCPUS		8
#define E9810_NR_CLUSTERS		2

/*
 * Register access rule (S0 R1, after the S0ab panic on PMU 0x30): only
 * registers that the 4.9 kernel reaches by plain non-secure MMIO on 9810;
 * table with the 4.9 file:line for each in Documentation/crownlte/
 * suspend-s0.md section 9. PMU 0x30 CP_CTRL_NS and 0x34 CP_CTRL_S are
 * secure-only: 4.9 goes through SMC 0x82000700 for them (pmucal_cp.c
 * pmucal_is_cp_smc_regs()), and a non-secure read is a synchronous
 * external abort. This driver never touches them and has no SMC.
 */

/* PMU (0x14060000) */
#define PMU_CP_STAT			0x0038
#define PMU_CENTRAL_SEQ_CP_CFG		0x0280
#define PMU_CENTRAL_SEQ_CP_STATUS	0x0284
#define PMU_IDLE_IP(n)			(0x03e0 + 4 * (n))
#define PMU_IDLE_IP_MASK(n)		(0x03f0 + 4 * (n))
#define PMU_NR_IDLE_IP			4
/* 4.9 SYS_SLEEP idle_ip_mask: init_idle_ip(), no fix-idle-ip on crownlte */
#define E9810_IDLE_IP_MASK_SLEEP	0xffffffff
#define PMU_RESET_SEQUENCER_CFG		0x0500	/* bit 0: DFD EDPCSR dump */
#define PMU_WAKEUP_STAT			0x0600
#define PMU_EINT_WAKEUP_MASK		0x060c
#define PMU_WAKEUP_STAT2		0x0640
#define PMU_CPU_INFORM(p)		(0x0860 + 4 * (p))
#define PMU_FLEXPMU_INFORM		0x0880
#define PMU_EMBEDDED_G3D_STATUS		0x4044
#define PMU_EVENT_INT_EN_GRP2		0x7f08
#define PMU_CPU_STATUS_ON		0xf

static const u16 pmu_cpu_status[E9810_NR_PHYCPUS] = {
	0x2024, 0x202c, 0x2034, 0x203c,		/* CLUSTER0 (LITTLE) CPU0..3 */
	0x2004, 0x200c, 0x2014, 0x201c,		/* CLUSTER1 (BIG) CPU0..3 */
};

/* CLUSTERx_NONCPU_STATUS as named by the two 4.9 sources (both logged) */
static const u16 pmu_noncpu_status[2] = { 0x2044, 0x204c };

#define E9810_NR_PAD_RET		6
static const u16 pmu_pad_ret[E9810_NR_PAD_RET] = {
	0x3060, 0x3080, 0x30a0, 0x30c0, 0x3120, 0x31a0,
};

/* PMU_INTR_GEN (0x14070000) */
#define INTR_GEN_EN(g)			(0x100 * (g))
#define INTR_GEN_PEND(g)		(0x100 * (g) + 0x8)
#define INTR_GEN_CLEAR(g)		(0x100 * (g) + 0xc)
/* LITTLE cores use bits 4-7, BIG cores bits 0-3 */
#define INTR_GEN_BIT(phy)		BIT((phy) ^ 4)
/* Groups whose EN and PEND the 4.9 CAL tables access (not 0, 3) */
static const u8 intr_gen_grp[] = { 1, 2, 4 };
#define E9810_NR_GRP			ARRAY_SIZE(intr_gen_grp)

/* GPIO_ALIVE (0x14050000): gpa0..gpa4 */
#define ALIVE_NR_BANKS			5
#define ALIVE_EINT_MASK(b)		(0x900 + 4 * (b))
#define ALIVE_EINT_PEND(b)		(0xa00 + 4 * (b))

/*
 * GIC-400 distributor; SGI/PPI bits are banked for the reading CPU.
 * ISPENDR0 bits 0-15 show a pending SGI from any source, so the S0d
 * self-IPI check needs no GICD_SPENDSGIRn (4.9 never reads those on 9810).
 */
#define GICD_ISPENDR(n)			(0x200 + 4 * (n))
#define GICD_NR_ISPENDR			16

/* MCT global free-running counter */
#define MCT_G_CNT_L			0x100
#define MCT_G_CNT_U			0x104
#define MCT_G_TCON			0x240
#define MCT_G_TCON_START		BIT(8)

/* APM2AP mailbox: 4.9 cluster{0,1}_{on,off} use bit 24 of +0x10 */
#define APM_MAILBOX_APM2AP		0x10

/* FlexPMU debug lines (16 bytes; words at +8, +12), 4.9 exynos-flexpmu-dbg */
static const u8 fdbg_line[] = { 10, 22, 23 };
static const char * const fdbg_name[] = { "sw_flag", "ap_sleep", "mif_sleep" };
#define E9810_NR_FDBG			ARRAY_SIZE(fdbg_line)

#define E9810_SYS_SLEEP			8	/* enum sys_powerdown SYS_SLEEP */
#define E9810_MAX_WMASK			8

/* S0 defaults (all DT-overridable) */
#define S0_START_MS_DEFAULT		20000
#define S0_END_BY_MS_DEFAULT		115000
#define S0_HP_CYCLES_DEFAULT		20
#define S0_CLUSTER_CYCLES_DEFAULT	5
#define S0_PM_TEST_DELAY_DEFAULT	5
#define S0_GROUP_CYCLES_DEFAULT		5
#define S0_HP_TIMEOUT_MS		9000	/* > arm64 5 s waits + CPU_DEAD 2 s */
#define S0_XCALL_TIMEOUT_MS		200	/* any cross-CPU request */
#define S0_WDT_LIVE_TRIES		20	/* WTCNT must move: 20 x 2-3 ms */
#define S0_DEAD_POLL_MS			2000	/* 4.9 exynos_cpu_callback() */
#define S0_DEAD_POLL_SHORT_MS		20	/* after a core stayed powered */
#define S0_REMINDERS			4
#define S0_REMINDER_MS			5000

enum s0_result { S0_NOTRUN, S0_PASS, S0_FAIL, S0_SKIPPED };
static const char * const s0_result_name[] = {
	[S0_NOTRUN] = "not-run", [S0_PASS] = "PASS",
	[S0_FAIL] = "FAIL", [S0_SKIPPED] = "SKIPPED",
};

struct e9810_snap {
	u32 cp_stat, cp_seq_cfg, cp_seq_stat;
	u32 dfd, g3d;
	u32 wakeup_stat, wakeup_stat2, eint_wmask;
	u32 wmask[E9810_MAX_WMASK];
	u32 idle_ip[PMU_NR_IDLE_IP], idle_ip_mask[PMU_NR_IDLE_IP];
	u32 inform[E9810_NR_PHYCPUS], flexpmu_inform;
	u32 pad_ret[E9810_NR_PAD_RET];
	u32 cpu_status[E9810_NR_PHYCPUS];
	u32 noncpu[2];
	u32 event_grp2;
	u32 grp_en[E9810_NR_GRP], grp_pend[E9810_NR_GRP]; /* intr_gen_grp[] */
	u32 apm2ap;
	u32 alive_mask[ALIVE_NR_BANKS], alive_pend[ALIVE_NR_BANKS];
	u32 gicd_pend[GICD_NR_ISPENDR];
	u64 frc, cntpct;
	u32 g_tcon;
	u32 fdbg[E9810_NR_FDBG][2];
};

/* Per-CPU hotplug hook record */
struct e9810_hp {
	u32 down, up;			/* hook runs on the CPU itself */
	int down_ret, up_ret;		/* last CAL results */
	u32 stale_grp2;			/* own GRP2 event cleared before arming */
	u32 cluster_off, cluster_on;	/* this CPU emptied/refilled its cluster */
	/* last CPU_DEAD poll (control CPU) */
	u32 dead;			/* CPU_DEAD polls */
	u32 dead_off_us;		/* time until the core read powered off */
	u32 dead_status, dead_inform;
	u32 still_on;			/* polls that timed out with the core on */
	bool dead_timed_out;
};

struct e9810_dry {
	u32 calls;
	u32 fn, state;
	unsigned long entry;
	u64 cntpct;
};

static struct e9810_pm {
	struct device *dev;
	struct regmap *pmu;
	void __iomem *intr_gen, *alive, *gicd, *mct, *fdbg, *mbox;

	unsigned int phycpu[NR_CPUS];
	unsigned int cluster_of[NR_CPUS];
	struct cpumask cluster_cpus[E9810_NR_CLUSTERS];
	bool topology_ok;

	/* DT */
	u32 sleep_mode;
	u32 psci_state;
	u32 psci_fn;
	u32 nr_wmask;
	u32 wmask_off[E9810_MAX_WMASK], wmask_val[E9810_MAX_WMASK];
	bool sleep_with_cp_off;
	bool want[4];			/* stages a..d */
	u32 start_ms, end_by_ms, deadline_ms;
	u32 hp_cycles, cluster_cycles, group_cycles, pm_test_delay;

	bool hooks, ops;		/* hotplug hooks / suspend ops registered */
	int dyn_dead_state;

	/* suspend path state */
	bool sys_entered;		/* syscore suspend ran cal_pm_enter_unarmed */
	bool early;			/* .enter came back without a power cycle */
	bool masks_owned;		/* sleep masks written: baseline is ours */
	u32 cnt_begin, cnt_prepare_late, cnt_enter, cnt_wake, cnt_end;
	u32 cnt_sys_suspend, cnt_sys_resume, cnt_sys_refused;
	int last_resume_ret;
	bool last_resume_early;
	bool window_grp2_latched;	/* CPU0 GRP2 event latched while entered */
	u32 enter_skipped;
	struct e9810_dry dry;
	bool ipi_pending;		/* self-IPI seen pending before the dry call */
	int enter_ret;
	u64 enter_cnt0, enter_cnt1, enter_frc0, enter_frc1;
	u32 enter_fdbg0[E9810_NR_FDBG], enter_fdbg1[E9810_NR_FDBG];

	/* S0 */
	struct e9810_snap boot, base, pre;	/* probe, S0a, before each step */
	bool boot_valid;
	struct acpm_log_snapshot *apm;
	enum s0_result res[4];
	char why[4][96];
	u32 b_ok, b_total, b_cluster_ok, b_cluster_total, b_still_on;
	u32 b_group_ok, b_group_total;
	u32 c_ok;
	struct cpumask lost;
	bool lost_online;
	struct hrtimer deadline;	/* supplement to the WDT ping-path check */
	struct task_struct *task;
} e9pm;

static DEFINE_PER_CPU(struct e9810_hp, e9810_hp);
static unsigned long e9810_dead_short;	/* bitmap: cores seen staying on */

static u32 pmu_rd(u32 off)
{
	u32 v = 0;

	regmap_read(e9pm.pmu, off, &v);
	return v;
}

static u64 mct_frc(void)
{
	u32 hi, lo, hi2;

	do {
		hi = readl(e9pm.mct + MCT_G_CNT_U);
		lo = readl(e9pm.mct + MCT_G_CNT_L);
		hi2 = readl(e9pm.mct + MCT_G_CNT_U);
	} while (hi != hi2);
	return ((u64)hi << 32) | lo;
}

static void fdbg_read(u32 *v1, u32 *v2)
{
	unsigned int i;

	for (i = 0; i < E9810_NR_FDBG; i++) {
		if (v1)
			v1[i] = readl(e9pm.fdbg + 16 * fdbg_line[i] + 8);
		if (v2)
			v2[i] = readl(e9pm.fdbg + 16 * fdbg_line[i] + 12);
	}
}

/* Read only. Usable with interrupts off. */
static void e9810_snap(struct e9810_snap *s)
{
	u32 a[E9810_NR_FDBG], b[E9810_NR_FDBG];
	unsigned int i;

	memset(s, 0, sizeof(*s));
	s->cp_stat = pmu_rd(PMU_CP_STAT);
	s->cp_seq_cfg = pmu_rd(PMU_CENTRAL_SEQ_CP_CFG);
	s->cp_seq_stat = pmu_rd(PMU_CENTRAL_SEQ_CP_STATUS);
	s->dfd = pmu_rd(PMU_RESET_SEQUENCER_CFG);
	s->g3d = pmu_rd(PMU_EMBEDDED_G3D_STATUS);
	s->wakeup_stat = pmu_rd(PMU_WAKEUP_STAT);
	s->wakeup_stat2 = pmu_rd(PMU_WAKEUP_STAT2);
	s->eint_wmask = pmu_rd(PMU_EINT_WAKEUP_MASK);
	for (i = 0; i < e9pm.nr_wmask; i++)
		s->wmask[i] = pmu_rd(e9pm.wmask_off[i]);
	for (i = 0; i < PMU_NR_IDLE_IP; i++) {
		s->idle_ip[i] = pmu_rd(PMU_IDLE_IP(i));
		s->idle_ip_mask[i] = pmu_rd(PMU_IDLE_IP_MASK(i));
	}
	for (i = 0; i < E9810_NR_PHYCPUS; i++) {
		s->inform[i] = pmu_rd(PMU_CPU_INFORM(i));
		s->cpu_status[i] = pmu_rd(pmu_cpu_status[i]);
	}
	s->flexpmu_inform = pmu_rd(PMU_FLEXPMU_INFORM);
	for (i = 0; i < E9810_NR_PAD_RET; i++)
		s->pad_ret[i] = pmu_rd(pmu_pad_ret[i]);
	for (i = 0; i < 2; i++)
		s->noncpu[i] = pmu_rd(pmu_noncpu_status[i]);
	s->event_grp2 = pmu_rd(PMU_EVENT_INT_EN_GRP2);
	for (i = 0; i < E9810_NR_GRP; i++) {
		s->grp_en[i] = readl(e9pm.intr_gen + INTR_GEN_EN(intr_gen_grp[i]));
		s->grp_pend[i] = readl(e9pm.intr_gen +
				       INTR_GEN_PEND(intr_gen_grp[i]));
	}
	s->apm2ap = readl(e9pm.mbox + APM_MAILBOX_APM2AP);
	for (i = 0; i < ALIVE_NR_BANKS; i++) {
		s->alive_mask[i] = readl(e9pm.alive + ALIVE_EINT_MASK(i));
		s->alive_pend[i] = readl(e9pm.alive + ALIVE_EINT_PEND(i));
	}
	for (i = 0; i < GICD_NR_ISPENDR; i++)
		s->gicd_pend[i] = readl(e9pm.gicd + GICD_ISPENDR(i));
	s->frc = mct_frc();
	s->g_tcon = readl(e9pm.mct + MCT_G_TCON);
	s->cntpct = __arch_counter_get_cntpct();
	fdbg_read(a, b);
	for (i = 0; i < E9810_NR_FDBG; i++) {
		s->fdbg[i][0] = a[i];
		s->fdbg[i][1] = b[i];
	}
}

static void e9810_snap_print(const char *tag, const struct e9810_snap *s)
{
	unsigned int i;

	pr_info("%s CP_STAT=%#x SEQ_CP cfg=%#x status=%#x DFD=%#x G3D=%#x\n",
		tag, s->cp_stat, s->cp_seq_cfg, s->cp_seq_stat, s->dfd, s->g3d);
	pr_info("%s INFORM0..7=%#x %#x %#x %#x %#x %#x %#x %#x FLEXPMU_INFORM=%#x\n",
		tag, s->inform[0], s->inform[1], s->inform[2], s->inform[3],
		s->inform[4], s->inform[5], s->inform[6], s->inform[7],
		s->flexpmu_inform);
	pr_info("%s CPU_STATUS0..7=%#x %#x %#x %#x %#x %#x %#x %#x NONCPU 0x2044=%#x 0x204c=%#x APM2AP=%#x\n",
		tag, s->cpu_status[0], s->cpu_status[1], s->cpu_status[2],
		s->cpu_status[3], s->cpu_status[4], s->cpu_status[5],
		s->cpu_status[6], s->cpu_status[7], s->noncpu[0], s->noncpu[1],
		s->apm2ap);
	pr_info("%s WAKEUP_MASK 0x60c=%#x %#x=%#x %#x=%#x %#x=%#x %#x=%#x WAKEUP_STAT=%#x/%#x\n",
		tag, s->eint_wmask,
		e9pm.wmask_off[0], s->wmask[0], e9pm.wmask_off[1], s->wmask[1],
		e9pm.wmask_off[2], s->wmask[2], e9pm.wmask_off[3], s->wmask[3],
		s->wakeup_stat, s->wakeup_stat2);
	pr_info("%s PAD_RET top=%#x spi=%#x bootldo0=%#x bootldo1=%#x pcie=%#x uart=%#x IDLE_IP=%#x %#x %#x %#x IDLE_IP_MASK=%#x %#x %#x %#x\n",
		tag, s->pad_ret[0], s->pad_ret[1], s->pad_ret[2], s->pad_ret[3],
		s->pad_ret[4], s->pad_ret[5], s->idle_ip[0], s->idle_ip[1],
		s->idle_ip[2], s->idle_ip[3], s->idle_ip_mask[0],
		s->idle_ip_mask[1], s->idle_ip_mask[2], s->idle_ip_mask[3]);
	pr_info("%s INTR_GEN en/pend grp1=%#x/%#x grp2=%#x/%#x grp4=%#x/%#x EVENT_INT_EN_GRP2=%#x\n",
		tag, s->grp_en[0], s->grp_pend[0], s->grp_en[1], s->grp_pend[1],
		s->grp_en[2], s->grp_pend[2], s->event_grp2);
	pr_info("%s EINT gpa0..4 mask=%#x %#x %#x %#x %#x pend=%#x %#x %#x %#x %#x\n",
		tag, s->alive_mask[0], s->alive_mask[1], s->alive_mask[2],
		s->alive_mask[3], s->alive_mask[4], s->alive_pend[0],
		s->alive_pend[1], s->alive_pend[2], s->alive_pend[3],
		s->alive_pend[4]);
	for (i = 0; i < GICD_NR_ISPENDR; i++)
		if (s->gicd_pend[i])
			pr_info("%s GICD_ISPENDR%u=%#x\n", tag, i, s->gicd_pend[i]);
	pr_info("%s FRC=%llu G_TCON=%#x(start=%d) CNTPCT=%llu (read on cpu%u)\n",
		tag, s->frc, s->g_tcon, !!(s->g_tcon & MCT_G_TCON_START),
		s->cntpct, raw_smp_processor_id());
	pr_info("%s flexpmu %s=%#x/%#x %s=%u/%u %s=%u/%u\n", tag,
		fdbg_name[0], s->fdbg[0][0], s->fdbg[0][1],
		fdbg_name[1], s->fdbg[1][0], s->fdbg[1][1],
		fdbg_name[2], s->fdbg[2][0], s->fdbg[2][1]);
}

/*
 * Kernel-owned configuration that every stage must leave as it found it:
 * the CPU_INFORM slots and GRP2 enables the CAL core/cluster/system
 * sequences write, the cluster flag in the APM mailbox and the EINT masks.
 * The PMU masks this driver writes are compared with its software-owned
 * baseline (e9810_write_sleep_masks()) once it has written them, before
 * that with @a. Each difference is printed (expected -> found); returns
 * their number. Not compared: INTR_GEN GRP1 enables and the other mailbox
 * bits (the APM firmware writes those) and PAD_RETENTION (cal_pm_exit()
 * writes the 4.9 release value; printed by step 50 instead).
 */
static unsigned int e9810_snap_diff(const char *tag, const struct e9810_snap *a,
				    const struct e9810_snap *b)
{
	unsigned int i, n = 0;

#define DIFF(name, idx, x, y) do {					\
		if ((x) != (y)) {					\
			pr_warn("%s changed %s[%u]: %#x -> %#x\n", tag, name,	\
				idx, (u32)(x), (u32)(y));		\
			n++;						\
		}							\
	} while (0)

	DIFF("0x60c", 0, a->eint_wmask, b->eint_wmask);
	for (i = 0; i < e9pm.nr_wmask; i++)
		DIFF("wakeup mask", i,
		     e9pm.masks_owned ? e9pm.wmask_val[i] : a->wmask[i],
		     b->wmask[i]);
	for (i = 0; i < PMU_NR_IDLE_IP; i++)
		DIFF("IDLE_IP_MASK", i,
		     e9pm.masks_owned ? E9810_IDLE_IP_MASK_SLEEP :
					a->idle_ip_mask[i],
		     b->idle_ip_mask[i]);
	for (i = 0; i < E9810_NR_PHYCPUS; i++)
		DIFF("INFORM", i, a->inform[i], b->inform[i]);
	DIFF("EVENT_INT_EN_GRP2", 0, a->event_grp2, b->event_grp2);
	DIFF("INTR_GEN GRP2 enable", 0, a->grp_en[1], b->grp_en[1]); /* grp 2 */
	DIFF("APM2AP bit24", 0, a->apm2ap & BIT(24), b->apm2ap & BIT(24));
	for (i = 0; i < ALIVE_NR_BANKS; i++)
		DIFF("EINT mask gpa", i, a->alive_mask[i], b->alive_mask[i]);
#undef DIFF
	return n;
}

static void e9810_apm_print(const char *tag, unsigned int last)
{
	struct acpm_log_snapshot *s = e9pm.apm;
	unsigned int i, shown = 0;
	int n;

	if (!s)
		return;
	n = acpm_ipc_log_snapshot(s);
	if (n < 0) {
		pr_info("%s apm-ring unavailable (%d)\n", tag, n);
		return;
	}
	pr_info("%s apm-ring rear=%u front=%u front_after=%u slots=%u, newest %u:\n",
		tag, s->rear, s->front, s->front_after, s->n, last);
	/* e[] is oldest first; walk back from the newest. */
	for (i = s->n; i-- > 0 && shown < last;) {
		struct acpm_log_snapshot_entry *e = &s->e[i];
		char tagbuf[9];
		unsigned int k;

		if (e->empty)
			continue;
		for (k = 0; k < 8; k++)
			tagbuf[k] = (e->tag[k] >= 0x20 && e->tag[k] < 0x7f) ?
				    e->tag[k] : '.';
		tagbuf[8] = '\0';
		pr_info("%s apm-ring #%u slot=%u t=%llu %s:%#x hdr=%#x\n", tag,
			shown, e->slot, e->time_ns, tagbuf, e->val, e->header);
		shown++;
	}
}

/* ------------------------------------------------------------------------ */
/* Bounded cross-CPU calls                                                  */
/* ------------------------------------------------------------------------ */

/*
 * Every request to another CPU, probe included, goes through
 * e9810_xcall(): asynchronous, waited for at most S0_XCALL_TIMEOUT_MS. The
 * request lives on the heap and is deliberately leaked on a timeout, so a
 * CPU that runs it late still writes valid memory. A timeout also stops
 * the watchdog petting for good: a CPU that the kernel counts online but
 * that takes no IPIs is the c2diag-b hang, and other CPUs would otherwise
 * keep the watchdog quiet. Process context only (sleeps).
 */
struct e9810_xcall {
	call_single_data_t csd;
	void (*fn)(struct e9810_xcall *x);
	u64 val;			/* set by fn */
	int ret;			/* set by fn */
	u64 t_run;			/* ktime_get_ns() | 1, published last */
};

static void e9810_xcall_run(void *info)
{
	struct e9810_xcall *x = info;

	x->fn(x);
	/* Pairs with the acquire in e9810_xcall(). */
	smp_store_release(&x->t_run, ktime_get_ns() | 1);
}

static int e9810_xcall(unsigned int cpu, void (*fn)(struct e9810_xcall *x),
		       const char *what, u64 *val, int *ret, u64 *lat_ns)
{
	struct e9810_xcall *x = kzalloc(sizeof(*x), GFP_KERNEL);
	u64 t0;
	int err;

	if (!x)
		return -ENOMEM;
	x->fn = fn;
	INIT_CSD(&x->csd, e9810_xcall_run, x);
	t0 = ktime_get_ns();
	err = smp_call_function_single_async(cpu, &x->csd);
	if (err) {
		kfree(x);
		return err;
	}
	/* Pairs with the release in e9810_xcall_run(). */
	while (!smp_load_acquire(&x->t_run)) {
		if (ktime_get_ns() - t0 > S0_XCALL_TIMEOUT_MS * NSEC_PER_MSEC) {
			pr_emerg("cpu%u did not run the %s request within %u ms (request left to it)\n",
				 cpu, what, S0_XCALL_TIMEOUT_MS);
			s3c2410wdt_crownlte_stop_petting("suspend s0: a CPU does not answer cross-CPU calls");
			return -ETIMEDOUT;
		}
		usleep_range(100, 200);
	}
	if (val)
		*val = x->val;
	if (ret)
		*ret = x->ret;
	if (lat_ns)
		*lat_ns = x->t_run - t0;
	kfree(x);
	return 0;
}

static void e9810_xcall_nop(struct e9810_xcall *x)
{
}

/* IPI round trip; -ETIMEDOUT: no answer (petting already stopped). */
static int e9810_ping(unsigned int cpu, u64 *lat_ns)
{
	return e9810_xcall(cpu, e9810_xcall_nop, "IPI ping", NULL, NULL, lat_ns);
}

/* ------------------------------------------------------------------------ */
/* CPU hotplug hooks (4.9 exynos_hotplug_{in,out}_callback)                  */
/* ------------------------------------------------------------------------ */

static bool e9810_cluster_has_online(unsigned int cpu)
{
	unsigned int c;

	for_each_cpu(c, &e9pm.cluster_cpus[e9pm.cluster_of[cpu]])
		if (c != cpu && cpu_online(c))
			return true;
	return false;
}

/*
 * STARTING teardown, on the dying CPU with interrupts off, after it left
 * cpu_online_mask; the next thing it runs is the idle loop's PSCI CPU_OFF.
 * A GRP2 event latched while the core ran (4.9 core-on clears it the same
 * way) would reach the APM the moment cal_cpu_disable() enables the core's
 * GRP2 interrupt, so it is cleared first.
 */
static int e9810_cpu_dying(unsigned int cpu)
{
	struct e9810_hp *hp = this_cpu_ptr(&e9810_hp);
	unsigned int phy = e9pm.phycpu[cpu];
	u32 bit = INTR_GEN_BIT(phy);

	if (readl(e9pm.intr_gen + INTR_GEN_PEND(2)) & bit) {
		writel(bit, e9pm.intr_gen + INTR_GEN_CLEAR(2));
		hp->stale_grp2++;
	}
	hp->down_ret = cal_cpu_disable(phy);
	/* big_reset_control(0) is a no-op without DFD dump (checked at probe) */
	if (!e9810_cluster_has_online(cpu)) {
		int ret = cal_cluster_disable(e9pm.cluster_of[cpu]);

		if (ret && !hp->down_ret)
			hp->down_ret = ret;
		hp->cluster_off++;
	}
	hp->down++;
	return 0;
}

/* STARTING startup, on the incoming CPU with interrupts off, not yet online. */
static int e9810_cpu_starting(unsigned int cpu)
{
	struct e9810_hp *hp = this_cpu_ptr(&e9810_hp);
	unsigned int phy = e9pm.phycpu[cpu];
	int ret = 0;

	if (!e9810_cluster_has_online(cpu)) {
		ret = cal_cluster_enable(e9pm.cluster_of[cpu]);
		hp->cluster_on++;
	}
	hp->up_ret = cal_cpu_enable(phy);
	if (ret && !hp->up_ret)
		hp->up_ret = ret;
	hp->up++;
	return 0;
}

/*
 * BP teardown after the CPU is dead (4.9 CPU_DEAD notifier): wait for the
 * PMU to report the core powered off. 4.9 BUG()s after 2 s; here the
 * result is only recorded. A core that stayed powered once is polled for
 * S0_DEAD_POLL_SHORT_MS afterwards, so a firmware that never powers
 * hotplugged cores off cannot stall the self-tests.
 */
static int e9810_cpu_dead(unsigned int cpu)
{
	struct e9810_hp *hp = per_cpu_ptr(&e9810_hp, cpu);
	unsigned int phy = e9pm.phycpu[cpu];
	unsigned int limit_ms = test_bit(cpu, &e9810_dead_short) ?
				S0_DEAD_POLL_SHORT_MS : S0_DEAD_POLL_MS;
	u64 t0 = ktime_get_ns(), dt;
	u32 st;

	for (;;) {
		st = pmu_rd(pmu_cpu_status[phy]);
		dt = ktime_get_ns() - t0;
		if (!(st & PMU_CPU_STATUS_ON) || dt > limit_ms * NSEC_PER_MSEC)
			break;
		usleep_range(50, 100);
	}
	hp->dead++;
	hp->dead_status = st;
	hp->dead_inform = pmu_rd(PMU_CPU_INFORM(phy));
	hp->dead_off_us = div_u64(dt, NSEC_PER_USEC);
	hp->dead_timed_out = !!(st & PMU_CPU_STATUS_ON);
	if (hp->dead_timed_out) {
		hp->still_on++;
		if (!test_and_set_bit(cpu, &e9810_dead_short))
			pr_warn("cpu%u still powered %u ms after CPU_DEAD: status=%#x inform=%#x (4.9 would BUG here); later polls %u ms\n",
				cpu, hp->dead_off_us / 1000, st,
				hp->dead_inform, S0_DEAD_POLL_SHORT_MS);
	}
	return 0;
}

static void e9810_reset_hooks_local(struct e9810_xcall *x)
{
	x->ret = cal_cpu_enable(e9pm.phycpu[smp_processor_id()]);
}

static void e9810_read_mpidr(struct e9810_xcall *x)
{
	x->val = read_cpuid_mpidr();
}

/* 4.9 linear_phycpu(): A55 MPIDRs carry the MT bit, Mongoose ones do not. */
static unsigned int e9810_linear_phycpu(u64 mpidr)
{
	unsigned int lvl = (mpidr & MPIDR_MT_BITMASK) ? 1 : 0;

	return (MPIDR_AFFINITY_LEVEL(mpidr, (1 + lvl)) << 2) |
		MPIDR_AFFINITY_LEVEL(mpidr, lvl);
}

static const char *e9810_init_topology(void)
{
	unsigned long seen = 0;
	unsigned int cpu;

	for_each_possible_cpu(cpu) {
		u64 hwid = cpu_logical_map(cpu), live = 0;
		unsigned int aff0 = MPIDR_AFFINITY_LEVEL(hwid, 0);
		unsigned int aff1 = MPIDR_AFFINITY_LEVEL(hwid, 1);
		unsigned int phy = (aff1 << 2) | aff0;

		if ((hwid & ~0xffffULL) || aff0 > 3 ||
		    aff1 >= E9810_NR_CLUSTERS || (seen & BIT(phy)))
			return "unexpected cpu reg/MPIDR layout";
		seen |= BIT(phy);
		e9pm.phycpu[cpu] = phy;
		e9pm.cluster_of[cpu] = aff1;
		cpumask_set_cpu(cpu, &e9pm.cluster_cpus[aff1]);
		/*
		 * pmucal_powermode_hint() picks the CPU_INFORM slot from the
		 * live MPIDR, the core sequences from phy: they must agree.
		 */
		if (!cpu_online(cpu))
			return "a CPU is offline at probe";
		if (e9810_xcall(cpu, e9810_read_mpidr, "MPIDR read", &live,
				NULL, NULL))
			return "MPIDR read IPI failed or timed out";
		if (e9810_linear_phycpu(live) != phy)
			return "live MPIDR does not match DT reg";
	}
	/* The S0b core lists use logical numbers: cpu0-3 LITTLE, cpu4-7 BIG. */
	if (nr_cpu_ids != E9810_NR_PHYCPUS)
		return "not 8 possible CPUs";
	for_each_possible_cpu(cpu)
		if (e9pm.cluster_of[cpu] != cpu / 4)
			return "logical CPUs are not cpu0-3 LITTLE, cpu4-7 BIG";
	e9pm.topology_ok = true;
	return NULL;
}

static const char *e9810_setup_hooks(void)
{
	unsigned int cpu;
	int ret;

	if (!e9pm.topology_ok)
		return "topology not verified";
	/* 4.9 big_reset_control() only acts with DFD dump enabled. */
	if (pmu_rd(PMU_RESET_SEQUENCER_CFG) & BIT(0))
		return "DFD EDPCSR dump enabled: big_reset_control() not ported";

	/*
	 * 4.9 resets each CPU's C2/hotplug hooks as it comes online. CPUs that
	 * booted before this driver get the same reset now, on themselves.
	 */
	for_each_online_cpu(cpu) {
		int r = -EIO;

		if (e9810_xcall(cpu, e9810_reset_hooks_local, "hook reset", NULL,
				&r, NULL) || r)
			return "hook reset (cal_cpu_enable) failed or timed out";
	}

	ret = cpuhp_setup_state_nocalls(CPUHP_AP_EXYNOS9810_CPU_POWER_STARTING,
					"soc/exynos9810-pm:power",
					e9810_cpu_starting, e9810_cpu_dying);
	if (ret)
		return "cpuhp STARTING state busy";
	ret = cpuhp_setup_state_nocalls(CPUHP_BP_PREPARE_DYN,
					"soc/exynos9810-pm:dead",
					NULL, e9810_cpu_dead);
	if (ret < 0) {
		cpuhp_remove_state_nocalls(CPUHP_AP_EXYNOS9810_CPU_POWER_STARTING);
		return "cpuhp dead state failed";
	}
	e9pm.dyn_dead_state = ret;
	e9pm.hooks = true;
	return NULL;
}

/* ------------------------------------------------------------------------ */
/* Suspend-to-RAM: platform ops and syscore                                 */
/* ------------------------------------------------------------------------ */

static int e9810_pm_begin(suspend_state_t state)
{
	e9pm.cnt_begin++;
	e9pm.early = false;
	pr_info("step 0 begin state=%d (mode %u, psci_state %#x)\n", state,
		e9pm.sleep_mode, e9pm.psci_state);
	return 0;
}

/* After dpm_suspend_noirq(), still with all CPUs online. */
static int e9810_pm_prepare_late(void)
{
	e9pm.cnt_prepare_late++;
	pr_info("step 5 prepare_late (noirq): %u CPUs online\n",
		num_online_cpus());
	return 0;
}

/*
 * Dry finisher: what 4.9 psci_suspend_customized_finisher() would pass to
 * PSCI CPU_SUSPEND, recorded instead of called. Runs in cpu_suspend()
 * after __cpu_suspend_enter() with RCU idle: no printk here.
 */
static int e9810_dry_finisher(unsigned long arg)
{
	e9pm.dry.calls++;
	e9pm.dry.fn = e9pm.psci_fn;
	e9pm.dry.state = e9pm.psci_state;
	e9pm.dry.entry = __pa_symbol(cpu_resume);
	e9pm.dry.cntpct = __arch_counter_get_cntpct();
	/* Returning at all is the not-slept path: cpu_suspend() -EOPNOTSUPP. */
	return 0;
}

static int e9810_pm_enter(suspend_state_t state)
{
	unsigned long entry = __pa_symbol(cpu_resume);
	unsigned int cpu = smp_processor_id();
	struct crownlte_wdt_state w = { };
	u32 pend0;
	int ret;

	e9pm.cnt_enter++;
	if (!e9pm.sys_entered) {
		pr_err("step 30 refused: syscore suspend did not enter mode %u\n",
		       e9pm.sleep_mode);
		return -EIO;
	}
	if (!entry) {
		pr_err("step 30 refused: cpu_resume physical address is 0\n");
		return -EINVAL;
	}

	s3c2410wdt_crownlte_ping();
	s3c2410wdt_crownlte_state(&w);
	e9pm.enter_cnt0 = __arch_counter_get_cntpct();
	e9pm.enter_frc0 = mct_frc();
	fdbg_read(NULL, e9pm.enter_fdbg0);

	/*
	 * 4.9 test_early_wakeup: a self-IPI (SGI1, call-function) pending at
	 * CPU0 makes the firmware's WFI fall through. Its banked pending bit
	 * (GICD_ISPENDR0 bit 1, any source) is read back here.
	 */
	arch_send_call_function_single_ipi(cpu);
	pend0 = readl(e9pm.gicd + GICD_ISPENDR(0));
	e9pm.ipi_pending = !!(pend0 & BIT(1));

	pr_info("step 30 enter (S0 dry run, no SMC): fn=%#x state=%#x entry=%#lx cpu%u self-IPI pending=%d (ISPENDR0=%#x) wdt=%us en=%d rst=%d cntpct=%llu frc=%llu\n",
		e9pm.psci_fn, e9pm.psci_state, entry, cpu, e9pm.ipi_pending, pend0,
		w.timeout, w.enabled, w.reset, e9pm.enter_cnt0,
		e9pm.enter_frc0);

	ret = cpu_suspend(0, e9810_dry_finisher);

	e9pm.enter_cnt1 = __arch_counter_get_cntpct();
	e9pm.enter_frc1 = mct_frc();
	fdbg_read(NULL, e9pm.enter_fdbg1);
	e9pm.enter_ret = ret;
	e9pm.early = ret != 0;
	pr_info("step 40 CPU_SUSPEND returned %d (%s) dry calls=%u fn=%#x state=%#x entry=%#lx cntpct %llu->%llu frc %llu->%llu (%s) mif_sleep %u->%u ap_sleep %u->%u (MIF blocked: no firmware call)\n",
		ret, ret ? "early" : "resumed", e9pm.dry.calls, e9pm.dry.fn,
		e9pm.dry.state, e9pm.dry.entry, e9pm.enter_cnt0, e9pm.enter_cnt1,
		e9pm.enter_frc0, e9pm.enter_frc1,
		e9pm.enter_frc1 > e9pm.enter_frc0 ? "frc running" : "FRC STOPPED",
		e9pm.enter_fdbg0[2], e9pm.enter_fdbg1[2], e9pm.enter_fdbg0[1],
		e9pm.enter_fdbg1[1]);
	return ret;
}

static void e9810_pm_wake(void)
{
	e9pm.cnt_wake++;
	pr_info("step 60 wake: %u CPUs online\n", num_online_cpus());
}

static void e9810_pm_end(void)
{
	e9pm.cnt_end++;
	pr_info("step 70 end\n");
}

static const struct platform_suspend_ops e9810_pm_ops = {
	.valid		= suspend_valid_only_mem,
	.begin		= e9810_pm_begin,
	.prepare_late	= e9810_pm_prepare_late,
	.enter		= e9810_pm_enter,
	.wake		= e9810_pm_wake,
	.end		= e9810_pm_end,
};

/*
 * The wakeup masks and IDLE_IP_MASK are written only with values this
 * driver owns, never with a value read back from the hardware: the 4.9
 * SYS_SLEEP values, DT samsung,wakeup-masks and all-ones IDLE_IP_MASK
 * (exynos-powermode.c exynos_prepare_sys_powerdown(), init_idle_ip()).
 * 4.9 leaves them in place after resume and after a failed entry; so does
 * this driver, which writes the same values again on resume and on the
 * undo path in case a CAL sequence moved them. From the first write on,
 * these values are the baseline the after-suspend register check expects.
 */
static void e9810_write_sleep_masks(void)
{
	unsigned int i;

	for (i = 0; i < e9pm.nr_wmask; i++)
		regmap_write(e9pm.pmu, e9pm.wmask_off[i], e9pm.wmask_val[i]);
	for (i = 0; i < PMU_NR_IDLE_IP; i++)
		regmap_write(e9pm.pmu, PMU_IDLE_IP_MASK(i),
			     E9810_IDLE_IP_MASK_SLEEP);
	e9pm.masks_owned = true;
}

/*
 * First syscore suspend callback to run (registered last), as 4.9's: all
 * other CPUs are offline and interrupts are off on CPU0.
 */
static int e9810_pm_syscore_suspend(void)
{
	u32 cp = pmu_rd(PMU_CP_STAT), g3d, en, ev, inform0;
	unsigned int phy0 = e9pm.phycpu[smp_processor_id()];
	int ret;

	e9pm.cnt_sys_suspend++;
	e9pm.sys_entered = false;
	e9pm.early = false;

	if (!cp && !e9pm.sleep_with_cp_off) {
		e9pm.cnt_sys_refused++;
		pr_info("step 20 sleep canceled by CP reset (CP_STAT 0, no samsung,sleep-with-cp-off)\n");
		return -EBUSY;
	}
	/*
	 * save/exit touch CMU_G3D and SYSREG_G3D without a power-domain
	 * condition. 4.9 always had G3D on here; S0 refuses otherwise.
	 */
	g3d = pmu_rd(PMU_EMBEDDED_G3D_STATUS);
	if ((g3d & 0xf) != 0xf) {
		e9pm.cnt_sys_refused++;
		pr_err("step 20 refused: EMBEDDED_G3D_STATUS=%#x (not on)\n", g3d);
		return -EBUSY;
	}

	/* 4.9 exynos_set_idle_ip_mask() + exynos_set_wakeupmask(SYS_SLEEP) */
	e9810_write_sleep_masks();

	e9pm.enter_skipped = 0;
	ret = cal_pm_enter_unarmed(e9pm.sleep_mode, &e9pm.enter_skipped);
	en = readl(e9pm.intr_gen + INTR_GEN_EN(2));
	ev = pmu_rd(PMU_EVENT_INT_EN_GRP2);
	inform0 = pmu_rd(PMU_CPU_INFORM(phy0));
	if (!ret && (en & INTR_GEN_BIT(phy0)))
		ret = -EIO;	/* armed after all: never leave it so */
	if (ret || !e9pm.enter_skipped) {
		pr_err("step 20 cal_pm_enter_unarmed(%u) failed %d (skipped %u, GRP2 en=%#x): undone\n",
		       e9pm.sleep_mode, ret, e9pm.enter_skipped, en);
		/* -EINVAL/-ENOENT: refused before the hint/save, nothing to undo */
		if (ret != -EINVAL && ret != -ENOENT)
			cal_pm_earlywakeup(e9pm.sleep_mode);
		/* The software-owned baseline, as 4.9 (no read-back restore). */
		e9810_write_sleep_masks();
		cal_cpu_enable(phy0);
		e9pm.cnt_sys_refused++;
		return ret ? ret : -EIO;
	}
	e9pm.sys_entered = true;
	pr_info("step 20 syscore suspend mode %u masks 0x60c=%#x(not written in S0) %#x=%#x %#x=%#x %#x=%#x %#x=%#x IDLE_IP_MASK=0xffffffff CP_STAT=%#x%s\n",
		e9pm.sleep_mode, pmu_rd(PMU_EINT_WAKEUP_MASK),
		e9pm.wmask_off[0], pmu_rd(e9pm.wmask_off[0]),
		e9pm.wmask_off[1], pmu_rd(e9pm.wmask_off[1]),
		e9pm.wmask_off[2], pmu_rd(e9pm.wmask_off[2]),
		e9pm.wmask_off[3], pmu_rd(e9pm.wmask_off[3]), cp,
		cp ? "" : " (CP off, allowed by DT)");
	pr_info("step 20 cal_pm_enter_unarmed(%u) ok: INFORM%u=%#x EVENT_INT_EN_GRP2=%#x GRP2 en=%#x (cpu0 bit %d, APM hand-off withheld: %u write skipped)\n",
		e9pm.sleep_mode, phy0, inform0, ev, en,
		!!(en & INTR_GEN_BIT(phy0)), e9pm.enter_skipped);
	return 0;
}

static void e9810_pm_syscore_resume(void)
{
	unsigned int i, phy0 = e9pm.phycpu[smp_processor_id()];
	u32 pend, inform0, en, ev, ws, ws2;
	int ret, ret2;

	if (!e9pm.sys_entered)
		return;
	e9pm.cnt_sys_resume++;
	e9pm.sys_entered = false;

	/*
	 * EVENT_INT_EN_GRP2 has CPU0's bit set while entered, so a WFI of
	 * CPU0 inside the window would have latched a GRP2 event.
	 */
	pend = readl(e9pm.intr_gen + INTR_GEN_PEND(2));
	e9pm.window_grp2_latched = !!(pend & INTR_GEN_BIT(phy0));
	ws = pmu_rd(PMU_WAKEUP_STAT);
	ws2 = pmu_rd(PMU_WAKEUP_STAT2);

	if (e9pm.early)
		ret = cal_pm_earlywakeup(e9pm.sleep_mode);
	else
		ret = cal_pm_exit(e9pm.sleep_mode);

	/* The software-owned baseline, as 4.9 (no read-back restore). */
	e9810_write_sleep_masks();
	/* 4.9 CPU-online reset of CPU0's slot: INFORM 0, GRP2 off and clear */
	ret2 = cal_cpu_enable(phy0);
	s3c2410wdt_crownlte_ping();

	e9pm.last_resume_ret = ret ? ret : ret2;
	e9pm.last_resume_early = e9pm.early;
	inform0 = pmu_rd(PMU_CPU_INFORM(phy0));
	en = readl(e9pm.intr_gen + INTR_GEN_EN(2));
	ev = pmu_rd(PMU_EVENT_INT_EN_GRP2);
	pr_info("step 50 syscore resume %s %s (ret %d/%d): INFORM%u=%#x GRP2 en=%#x EVENT_INT_EN_GRP2=%#x cpu0 GRP2 latched in window=%d PAD_RET top=%u spi=%u bootldo0=%u bootldo1=%u pcie=%u uart=%u WAKEUP_STAT=%#x/%#x\n",
		e9pm.early ? "earlywakeup" : "exit",
		e9pm.last_resume_ret ? "FAILED" : "ok", ret, ret2, phy0, inform0,
		en, ev, e9pm.window_grp2_latched,
		pmu_rd(pmu_pad_ret[0]) & 1, pmu_rd(pmu_pad_ret[1]) & 1,
		pmu_rd(pmu_pad_ret[2]) & 1, pmu_rd(pmu_pad_ret[3]) & 1,
		pmu_rd(pmu_pad_ret[4]) & 1, pmu_rd(pmu_pad_ret[5]) & 1,
		ws, ws2);
	if (e9pm.early) {
		/* 4.9 exynos_show_wakeup_reason(sleep_abort) */
		pr_info("step 50 early wakeup: EINT_PEND gpa0..4=%#x %#x %#x %#x %#x\n",
			readl(e9pm.alive + ALIVE_EINT_PEND(0)),
			readl(e9pm.alive + ALIVE_EINT_PEND(1)),
			readl(e9pm.alive + ALIVE_EINT_PEND(2)),
			readl(e9pm.alive + ALIVE_EINT_PEND(3)),
			readl(e9pm.alive + ALIVE_EINT_PEND(4)));
		for (i = 0; i < GICD_NR_ISPENDR; i++) {
			u32 v = readl(e9pm.gicd + GICD_ISPENDR(i));

			if (v)
				pr_info("step 50 early wakeup: GICD_ISPENDR%u=%#x\n",
					i, v);
		}
	}
}

static struct syscore_ops e9810_pm_syscore_ops = {
	.suspend	= e9810_pm_syscore_suspend,
	.resume		= e9810_pm_syscore_resume,
};

/* ------------------------------------------------------------------------ */
/* S0 self-tests                                                            */
/* ------------------------------------------------------------------------ */

static u64 boot_ms(void)
{
	return ktime_to_ms(ktime_get_boottime());
}

static void s0_set(int stage, enum s0_result r, const char *fmt, ...)
{
	va_list ap;

	e9pm.res[stage] = r;
	va_start(ap, fmt);
	vsnprintf(e9pm.why[stage], sizeof(e9pm.why[stage]), fmt, ap);
	va_end(ap);
}

static enum hrtimer_restart e9810_deadline_fn(struct hrtimer *t)
{
	pr_emerg("deadline %u ms reached and the system is still up (the vehicle restarts at 120 s): stopping watchdog petting\n",
		 e9pm.deadline_ms);
	s3c2410wdt_crownlte_stop_petting("suspend s0 deadline");
	return HRTIMER_NORESTART;
}

/*
 * The deadline lives in the watchdog's own ping path: after it, no CPU can
 * reload the counter (s3c2410wdt_crownlte_set_deadline()), so a CPU wedged
 * with interrupts off cannot be covered up by others that keep petting,
 * wherever any timer sits. Installed at probe, before the first cross-CPU
 * request; without an armed guarded watchdog the driver does not start.
 * The hrtimer only adds the log line and an earlier stop when nobody pets.
 */
static int e9810_deadline_install(void)
{
	int ret;

	ret = s3c2410wdt_crownlte_set_deadline((u64)e9pm.deadline_ms *
						NSEC_PER_MSEC);
	if (ret)
		return ret;
	hrtimer_init(&e9pm.deadline, CLOCK_BOOTTIME, HRTIMER_MODE_ABS);
	e9pm.deadline.function = e9810_deadline_fn;
	hrtimer_start(&e9pm.deadline, ms_to_ktime(e9pm.deadline_ms),
		      HRTIMER_MODE_ABS);
	return 0;
}

/*
 * Recovery armed: the guarded CL0 watchdog runs with the reset enabled, is
 * still petted, carries this driver's deadline (or an earlier one) and
 * (@live) its counter is seen moving. NULL if so, else why not.
 */
static const char *s0_recovery_refused(bool live, bool keep_armed)
{
	struct crownlte_wdt_state a, b;
	unsigned int i;

	if (!s3c2410wdt_crownlte_state(&a))
		return "no guarded CL0 watchdog";
	if (!a.enabled || !a.reset)
		return "CL0 watchdog not running with the reset enabled";
	if (a.pet_stopped)
		return "CL0 watchdog petting already stopped";
	if (!a.deadline_ns ||
	    a.deadline_ns > (u64)e9pm.deadline_ms * NSEC_PER_MSEC)
		return "watchdog deadline not installed";
	if (keep_armed && !a.keep_armed_suspend)
		return "CL0 watchdog without samsung,keep-armed-in-suspend";
	if (!live)
		return NULL;
	for (i = 0; i < S0_WDT_LIVE_TRIES; i++) {
		usleep_range(2000, 3000);
		if (!s3c2410wdt_crownlte_state(&b) || !b.enabled || !b.reset ||
		    b.pet_stopped)
			return "CL0 watchdog stopped while being checked";
		if (b.wtcnt != a.wtcnt)
			return NULL;
	}
	return "CL0 watchdog counter not moving";
}

/* Number of online CPUs that did not answer; logs each one. */
static void e9810_lost(unsigned int cpu, const char *why, bool online);

static unsigned int e9810_ping_all(const char *tag)
{
	unsigned int cpu, bad = 0;
	u64 lat;

	for_each_online_cpu(cpu) {
		int r = e9810_ping(cpu, &lat);

		if (r) {
			pr_err("%s cpu%u online but no IPI answer (%d)\n", tag,
			       cpu, r);
			if (r == -ETIMEDOUT)
				e9810_lost(cpu, "online but not answering IPIs",
					   true);
			bad++;
		}
	}
	return bad;
}

static void e9810_log_core(const char *tag, unsigned int cpu)
{
	unsigned int phy = e9pm.phycpu[cpu];
	u32 bit = INTR_GEN_BIT(phy);
	u32 g1e = readl(e9pm.intr_gen + INTR_GEN_EN(1));
	u32 g1p = readl(e9pm.intr_gen + INTR_GEN_PEND(1));
	u32 g2e = readl(e9pm.intr_gen + INTR_GEN_EN(2));
	u32 g2p = readl(e9pm.intr_gen + INTR_GEN_PEND(2));

	pr_emerg("%s cpu%u phy%u status=%#x inform=%#x EVENT_INT_EN_GRP2=%#x grp1 en/pend=%d/%d grp2 en/pend=%d/%d APM2AP=%#x online=%d\n",
		 tag, cpu, phy, pmu_rd(pmu_cpu_status[phy]),
		 pmu_rd(PMU_CPU_INFORM(phy)),
		 pmu_rd(PMU_EVENT_INT_EN_GRP2), !!(g1e & bit), !!(g1p & bit),
		 !!(g2e & bit), !!(g2p & bit),
		 readl(e9pm.mbox + APM_MAILBOX_APM2AP), cpu_online(cpu));
}

/*
 * A CPU that the kernel still counts online but that does not run: other
 * CPUs would keep the watchdog petted forever and CPU0 may block on it.
 * Stop petting; the CL0 watchdog then resets the SoC (proven q35/q37) and
 * the shim bounce returns to Lineage.
 */
static void e9810_lost(unsigned int cpu, const char *why, bool online)
{
	cpumask_set_cpu(cpu, &e9pm.lost);
	e9810_log_core("LOST", cpu);
	pr_emerg("CPU-LOST cpu%u: %s\n", cpu, why);
	e9810_apm_print("LOST", 16);
	if (online) {
		e9pm.lost_online = true;
		s3c2410wdt_crownlte_stop_petting("suspend s0: CPU lost while online");
	}
}

/*
 * Every possible CPU online again; each one that is not is recorded as
 * lost (offline: the kernel gave up on it, so nothing waits on it).
 */
static unsigned int e9810_check_all_back(const char *tag)
{
	unsigned int cpu, n = 0;

	for_each_possible_cpu(cpu) {
		if (cpu_online(cpu))
			continue;
		pr_emerg("%s cpu%u is not back online\n", tag, cpu);
		e9810_lost(cpu, "not back online", false);
		n++;
	}
	return n;
}

struct e9810_hp_req {
	unsigned int cpu;
	bool up;
	int ret;
	struct completion done;
};

static int e9810_hp_fn(void *data)
{
	struct e9810_hp_req *r = data;

	r->ret = r->up ? add_cpu(r->cpu) : remove_cpu(r->cpu);
	complete(&r->done);
	return 0;
}

/*
 * add_cpu()/remove_cpu() from a helper thread on CPU0, waited for at most
 * S0_HP_TIMEOUT_MS. Neither should block that long: arm64 __cpu_up()
 * gives up after 5 s, __cpu_die() after 5 s plus the PSCI AFFINITY_INFO
 * poll, and the CPU_DEAD poll here after 2 s. A helper still blocked holds
 * the hotplug locks, which the vehicle's restart path needs as well.
 */
static int e9810_hotplug_bounded(unsigned int cpu, bool up, int *ret)
{
	struct e9810_hp_req *r = kzalloc(sizeof(*r), GFP_KERNEL);
	struct task_struct *t;

	if (!r)
		return -ENOMEM;
	r->cpu = cpu;
	r->up = up;
	init_completion(&r->done);
	t = kthread_create(e9810_hp_fn, r, up ? "s0-up/%u" : "s0-down/%u", cpu);
	if (IS_ERR(t)) {
		kfree(r);
		return PTR_ERR(t);
	}
	kthread_bind(t, 0);
	wake_up_process(t);
	if (!wait_for_completion_timeout(&r->done,
					 msecs_to_jiffies(S0_HP_TIMEOUT_MS)))
		return -ETIMEDOUT;	/* r stays with the blocked helper */
	*ret = r->ret;
	kfree(r);
	return 0;
}

/* Offline @cpu and check what CPU_DEAD saw. 0, or a FAIL already logged. */
static int s0b_down(const char *ph, unsigned int cpu, unsigned int n)
{
	struct e9810_hp *hp = per_cpu_ptr(&e9810_hp, cpu);
	u32 down0 = hp->down, dead0 = hp->dead;
	const char *why;
	int ret = 0, err;

	/* No core goes down unless the watchdog can still reset the SoC. */
	why = s0_recovery_refused(false, false);
	if (why) {
		pr_err("S0b %s cpu%u off#%u not started: %s FAIL\n", ph, cpu, n,
		       why);
		return -ENOLINK;
	}
	err = e9810_hotplug_bounded(cpu, false, &ret);
	if (err == -ETIMEDOUT) {
		pr_emerg("S0b %s cpu%u off#%u remove_cpu() still blocked after %u ms (online=%d)\n",
			 ph, cpu, n, S0_HP_TIMEOUT_MS, cpu_online(cpu));
		e9810_lost(cpu, "tear-down never completed", true);
		return -ETIMEDOUT;
	}
	if (err) {
		pr_err("S0b %s cpu%u off#%u helper failed %d FAIL\n", ph, cpu, n,
		       err);
		return err;
	}
	if (ret || cpu_online(cpu)) {
		pr_err("S0b %s cpu%u off#%u remove_cpu=%d online=%d FAIL\n", ph,
		       cpu, n, ret, cpu_online(cpu));
		return ret ? ret : -EBUSY;
	}
	if (hp->down != down0 + 1 || hp->dead != dead0 + 1 || hp->down_ret) {
		pr_err("S0b %s cpu%u off#%u hooks: down %u->%u dead %u->%u cal=%d FAIL\n",
		       ph, cpu, n, down0, hp->down, dead0, hp->dead,
		       hp->down_ret);
		return -EIO;
	}
	if (hp->dead_timed_out)
		e9pm.b_still_on++;
	return 0;
}

/* Online @cpu, ping it. 0, or a FAIL/LOST already logged. */
static int s0b_up(const char *ph, unsigned int cpu, unsigned int n, u64 *lat)
{
	struct e9810_hp *hp = per_cpu_ptr(&e9810_hp, cpu);
	u32 up0 = hp->up;
	int ret = 0, err;

	err = e9810_hotplug_bounded(cpu, true, &ret);
	if (err == -ETIMEDOUT) {
		pr_emerg("S0b %s cpu%u on#%u add_cpu() still blocked after %u ms (online=%d)\n",
			 ph, cpu, n, S0_HP_TIMEOUT_MS, cpu_online(cpu));
		/*
		 * Online or not, the blocked helper holds the hotplug locks,
		 * which the restart path needs: treat it as lost while online.
		 */
		e9810_lost(cpu, "bring-up never completed", true);
		return -ETIMEDOUT;
	}
	if (err) {
		pr_err("S0b %s cpu%u on#%u helper failed %d FAIL\n", ph, cpu, n,
		       err);
		return err;
	}
	if (ret || !cpu_online(cpu)) {
		pr_emerg("S0b %s cpu%u on#%u add_cpu=%d online=%d\n", ph, cpu, n,
			 ret, cpu_online(cpu));
		e9810_lost(cpu, "PSCI CPU_ON bring-up failed", cpu_online(cpu));
		return ret ? ret : -EIO;
	}
	err = e9810_ping(cpu, lat);
	if (err) {
		pr_emerg("S0b %s cpu%u on#%u online but no IPI answer (%d)\n", ph,
			 cpu, n, err);
		e9810_lost(cpu, "online but not answering IPIs", true);
		return err;
	}
	if (hp->up != up0 + 1 || hp->up_ret) {
		pr_err("S0b %s cpu%u on#%u hooks: up %u->%u cal=%d FAIL\n", ph,
		       cpu, n, up0, hp->up, hp->up_ret);
		return -EIO;
	}
	return 0;
}

static int s0b_core(const char *ph, unsigned int cpu, unsigned int n)
{
	struct e9810_hp *hp = per_cpu_ptr(&e9810_hp, cpu);
	u64 t1, lat = 0;
	int ret;

	ret = s0b_down(ph, cpu, n);
	if (ret)
		return ret;
	t1 = ktime_get_ns();
	ret = s0b_up(ph, cpu, n, &lat);
	if (ret)
		return ret;
	pr_info("S0b %s cpu%u #%u off: inform=%#x status=%#x off_us=%u%s on: up_ms=%llu ping_us=%llu stale_grp2=%u ok\n",
		ph, cpu, n, hp->dead_inform, hp->dead_status, hp->dead_off_us,
		hp->dead_timed_out ? " STILL-POWERED" : "",
		div_u64(ktime_get_ns() - t1, NSEC_PER_MSEC),
		div_u64(lat, NSEC_PER_USEC), hp->stale_grp2);
	return 0;
}

/* Whole BIG cluster down (cluster_disable) and up again (cluster_enable). */
static int s0b_cluster(unsigned int n)
{
	static const unsigned int down[] = { 7, 6, 5, 4 }, up[] = { 4, 5, 6, 7 };
	struct e9810_hp *first = per_cpu_ptr(&e9810_hp, 4);
	u32 off0 = first->cluster_off, on0 = first->cluster_on;
	u32 noncpu[2], apm2ap;
	unsigned int i;
	u64 lat;
	int ret;

	for (i = 0; i < ARRAY_SIZE(down); i++) {
		ret = s0b_down("cluster", down[i], n);
		if (ret)
			return ret;
	}
	noncpu[0] = pmu_rd(pmu_noncpu_status[0]);
	noncpu[1] = pmu_rd(pmu_noncpu_status[1]);
	apm2ap = readl(e9pm.mbox + APM_MAILBOX_APM2AP);
	if (first->cluster_off != off0 + 1) {
		pr_err("S0b cluster #%u cluster_disable not run by cpu4 (%u->%u) FAIL\n",
		       n, off0, first->cluster_off);
		return -EIO;
	}
	for (i = 0; i < ARRAY_SIZE(up); i++) {
		ret = s0b_up("cluster", up[i], n, &lat);
		if (ret)
			return ret;
	}
	if (first->cluster_on != on0 + 1) {
		pr_err("S0b cluster #%u cluster_enable not run by cpu4 (%u->%u) FAIL\n",
		       n, on0, first->cluster_on);
		return -EIO;
	}
	pr_info("S0b cluster #%u BIG off (NONCPU 0x2044=%#x 0x204c=%#x APM2AP=%#x) and on again (APM2AP=%#x) ok\n",
		n, noncpu[0], noncpu[1], apm2ap,
		readl(e9pm.mbox + APM_MAILBOX_APM2AP));
	return 0;
}

/*
 * Several CPUs off at once, taken down in @cpus order and brought back in
 * the same order (freeze_secondary_cpus() / thaw_secondary_cpus() order
 * for the all-nonboot group). LITTLE groups are the point: with three
 * LITTLE cores cycling C2, EL3 lost a core (q22b, q40, q43, c2diag-a/b),
 * with two it did not (c2soak, 20 s). Hotplug uses the same APM path as C2
 * (4.9 cal_cpu_disable() hint = C2) plus PSCI CPU_OFF/CPU_ON, and system
 * suspend takes all three LITTLE cores down together.
 */
static int s0b_group(const char *ph, const unsigned int *cpus, unsigned int nr,
		     unsigned int n)
{
	char st[E9810_NR_PHYCPUS * 24];
	unsigned int i, len = 0;
	u64 t1, lat = 0;
	int ret;

	t1 = ktime_get_ns();
	for (i = 0; i < nr; i++) {
		ret = s0b_down(ph, cpus[i], n);
		if (ret)
			return ret;
	}
	/* What CPU_DEAD saw for each core, with all of them down. */
	st[0] = '\0';
	for (i = 0; i < nr; i++) {
		struct e9810_hp *hp = per_cpu_ptr(&e9810_hp, cpus[i]);

		len += scnprintf(st + len, sizeof(st) - len, " cpu%u=%#x/%#x%s",
				 cpus[i], hp->dead_status, hp->dead_inform,
				 hp->dead_timed_out ? "!" : "");
	}
	for (i = 0; i < nr; i++) {
		ret = s0b_up(ph, cpus[i], n, &lat);
		if (ret)
			return ret;
	}
	pr_info("S0b %s #%u %u cores off (status/inform:%s) and on in %llu ms ok\n",
		ph, n, nr, st, div_u64(ktime_get_ns() - t1, NSEC_PER_MSEC));
	return 0;
}

static void s0b_run(void)
{
	/* BIG first, LITTLE last (LITTLE cores have the EL3 C2 hang) */
	static const unsigned int big[] = { 7, 6, 5, 4 }, little[] = { 3, 2, 1 };
	static const unsigned int pair[3][2] = { { 1, 2 }, { 1, 3 }, { 2, 3 } };
	static const unsigned int triple[] = { 1, 2, 3 };
	static const unsigned int nonboot[] = { 1, 2, 3, 4, 5, 6, 7 };
	struct e9810_snap s;
	unsigned int i, n, diff;
	u64 t0 = boot_ms();
	int ret = 0;

	e9810_snap(&e9pm.pre);
	e9pm.b_total = (ARRAY_SIZE(big) + ARRAY_SIZE(little)) * e9pm.hp_cycles;
	e9pm.b_cluster_total = e9pm.cluster_cycles;
	e9pm.b_group_total = (ARRAY_SIZE(pair) + 2) * e9pm.group_cycles;
	pr_info("S0b start: %u x cpu7..4, %u x BIG cluster, %u x cpu3..1, %u x LITTLE pairs {1,2} {1,3} {2,3}, %u x LITTLE {1,2,3}, %u x cpu1..7 (suspend order)\n",
		e9pm.hp_cycles, e9pm.cluster_cycles, e9pm.hp_cycles,
		e9pm.group_cycles, e9pm.group_cycles, e9pm.group_cycles);

	for (i = 0; i < ARRAY_SIZE(big) && !ret; i++)
		for (n = 1; n <= e9pm.hp_cycles && !ret; n++) {
			ret = s0b_core("big", big[i], n);
			if (!ret)
				e9pm.b_ok++;
		}
	for (n = 1; n <= e9pm.cluster_cycles && !ret; n++) {
		ret = s0b_cluster(n);
		if (!ret)
			e9pm.b_cluster_ok++;
	}
	for (i = 0; i < ARRAY_SIZE(little) && !ret; i++)
		for (n = 1; n <= e9pm.hp_cycles && !ret; n++) {
			ret = s0b_core("little", little[i], n);
			if (!ret)
				e9pm.b_ok++;
		}
	/* Escalation: two LITTLE cores off together, then three, then all. */
	for (i = 0; i < ARRAY_SIZE(pair) && !ret; i++)
		for (n = 1; n <= e9pm.group_cycles && !ret; n++) {
			ret = s0b_group("little-2", pair[i], 2, n);
			if (!ret)
				e9pm.b_group_ok++;
		}
	for (n = 1; n <= e9pm.group_cycles && !ret; n++) {
		ret = s0b_group("little-3", triple, ARRAY_SIZE(triple), n);
		if (!ret)
			e9pm.b_group_ok++;
	}
	for (n = 1; n <= e9pm.group_cycles && !ret; n++) {
		ret = s0b_group("nonboot", nonboot, ARRAY_SIZE(nonboot), n);
		if (!ret)
			e9pm.b_group_ok++;
	}

	if (!ret && e9810_check_all_back("S0b end:"))
		ret = -EIO;
	if (!ret && e9810_ping_all("S0b end:"))
		ret = -EIO;
	e9810_apm_print("S0b end:", 8);
	e9810_snap(&s);
	diff = e9810_snap_diff("S0b end:", &e9pm.pre, &s);
	for_each_possible_cpu(i) {
		struct e9810_hp *hp = per_cpu_ptr(&e9810_hp, i);

		pr_info("S0b cpu%u hooks down=%u up=%u cluster off/on=%u/%u stale_grp2=%u dead=%u still_on=%u last cal %d/%d\n",
			i, hp->down, hp->up, hp->cluster_off, hp->cluster_on,
			hp->stale_grp2, hp->dead, hp->still_on, hp->down_ret,
			hp->up_ret);
	}
	if (ret)
		s0_set(1, S0_FAIL, "single %u/%u cluster %u/%u group %u/%u err %d%s",
		       e9pm.b_ok, e9pm.b_total, e9pm.b_cluster_ok,
		       e9pm.b_cluster_total, e9pm.b_group_ok, e9pm.b_group_total,
		       ret, cpumask_empty(&e9pm.lost) ? "" : " CPU-LOST");
	else if (diff)
		s0_set(1, S0_FAIL, "single %u/%u cluster %u/%u group %u/%u, %u regs not restored",
		       e9pm.b_ok, e9pm.b_total, e9pm.b_cluster_ok,
		       e9pm.b_cluster_total, e9pm.b_group_ok, e9pm.b_group_total,
		       diff);
	else
		s0_set(1, S0_PASS, "single %u/%u cluster %u/%u group %u/%u still-on %u",
		       e9pm.b_ok, e9pm.b_total, e9pm.b_cluster_ok,
		       e9pm.b_cluster_total, e9pm.b_group_ok, e9pm.b_group_total,
		       e9pm.b_still_on);
	pr_info("S0b %s single %u/%u cluster %u/%u group %u/%u in %llu ms, %u CPU_DEAD polls saw the core still powered%s\n",
		s0_result_name[e9pm.res[1]], e9pm.b_ok, e9pm.b_total,
		e9pm.b_cluster_ok, e9pm.b_cluster_total, e9pm.b_group_ok,
		e9pm.b_group_total, boot_ms() - t0, e9pm.b_still_on,
		e9pm.b_still_on ? " (WARN: 4.9 BUG()s on that; blocks S1)" : "");
}

/* S0c/S0d preconditions: nothing in device suspend may reach UFS/charger. */
static const char *s0_suspend_refused(void)
{
	struct device_node *np;
	const char *why;

	if (!e9pm.ops)
		return "suspend ops not registered";
	/* Live, reset-enabled, petted, deadline, kept armed in suspend. */
	why = s0_recovery_refused(true, true);
	if (why)
		return why;
	for_each_compatible_node(np, NULL, "samsung,exynos9810-ufs") {
		struct platform_device *pdev = of_find_device_by_node(np);

		if (pdev) {
			if (READ_ONCE(pdev->dev.driver))
				why = "a UFS host is bound (no UFS power commands in S0)";
			put_device(&pdev->dev);
		}
		if (why) {
			of_node_put(np);
			return why;
		}
	}
	np = of_find_node_with_property(NULL, "samsung,charger-wdt-kick");
	if (np) {
		bool on = of_device_is_available(np);

		of_node_put(np);
		if (on)
			return "charger WDT kick enabled (no charger writes in S0)";
	}
	if (!cpumask_empty(&e9pm.lost))
		return "a CPU was lost";
	return NULL;
}

static bool s0_regs_ok(const char *tag, unsigned int *diff)
{
	struct e9810_snap s;

	e9810_snap(&s);
	*diff = e9810_snap_diff(tag, &e9pm.pre, &s);
	return !*diff;
}

static void s0c_run(void)
{
	static const char * const level[] = {
		"freezer", "devices", "platform", "processors", "core",
	};
	unsigned int i, diff;
	const char *why;
	u64 t0;
	int ret;

	why = s0_suspend_refused();
	if (!why && !IS_ENABLED(CONFIG_PM_DEBUG))
		why = "CONFIG_PM_DEBUG off: no pm_test";
	if (why) {
		s0_set(2, S0_SKIPPED, "%s", why);
		pr_err("S0c refused: %s\n", why);
		return;
	}
	pr_info("S0c start: pm_test %s .. %s, %u s each\n", level[0],
		level[ARRAY_SIZE(level) - 1], e9pm.pm_test_delay);

	for (i = 0; i < ARRAY_SIZE(level); i++) {
		u32 b0 = e9pm.cnt_begin, p0 = e9pm.cnt_prepare_late,
		    e0 = e9pm.cnt_enter, ss0 = e9pm.cnt_sys_suspend,
		    sr0 = e9pm.cnt_sys_resume;
		bool want_begin = i >= 1, want_late = i >= 2,
		     want_sys = i == 4;
		struct crownlte_wdt_state w = { };
		const char *bad = NULL;

		t0 = boot_ms();
		e9810_snap(&e9pm.pre);
		pr_info("S0c level=%s start\n", level[i]);
		ret = pm_suspend_test_level(level[i], e9pm.pm_test_delay);
		s3c2410wdt_crownlte_state(&w);

		if (ret)
			bad = "pm_suspend returned an error";
		else if (e9810_check_all_back("S0c:"))
			bad = "not all CPUs back online";
		else if (e9810_ping_all("S0c:"))
			bad = "a CPU does not answer";
		else if ((e9pm.cnt_begin - b0) != want_begin ||
			 (e9pm.cnt_prepare_late - p0) != want_late ||
			 e9pm.cnt_enter != e0)
			bad = "unexpected platform callbacks";
		else if ((e9pm.cnt_sys_suspend - ss0) != want_sys ||
			 (e9pm.cnt_sys_resume - sr0) != want_sys)
			bad = "unexpected syscore callbacks";
		else if (want_sys && (e9pm.last_resume_ret || e9pm.last_resume_early))
			bad = "syscore resume exit failed";
		else if (!w.enabled || !w.reset || w.pet_stopped)
			bad = "watchdog not armed after resume";
		else if (!s0_regs_ok("S0c:", &diff))
			bad = "PMU state not restored";

		pr_info("S0c level=%s ret=%d %llu ms begin/prepare_late/enter +%u/+%u/+%u syscore +%u/+%u%s cpus %u/%u wdt %us en=%d -> %s%s\n",
			level[i], ret, boot_ms() - t0, e9pm.cnt_begin - b0,
			e9pm.cnt_prepare_late - p0, e9pm.cnt_enter - e0,
			e9pm.cnt_sys_suspend - ss0, e9pm.cnt_sys_resume - sr0,
			want_sys ? (e9pm.window_grp2_latched ?
				    " cpu0-WFI-in-window" : " no-cpu0-WFI") : "",
			num_online_cpus(), num_possible_cpus(), w.timeout,
			w.enabled, bad ? "FAIL: " : "PASS", bad ? bad : "");
		if (bad) {
			s0_set(2, S0_FAIL, "level %s: %s (%d)", level[i], bad,
			       ret);
			e9810_apm_print("S0c fail:", 8);
			return;
		}
		e9pm.c_ok++;
	}
	e9810_apm_print("S0c end:", 8);
	s0_set(2, S0_PASS, "%u/%u levels", e9pm.c_ok,
	       (unsigned int)ARRAY_SIZE(level));
	pr_info("S0c PASS %u/%u\n", e9pm.c_ok, (unsigned int)ARRAY_SIZE(level));
}

static void s0d_run(void)
{
	u32 e0 = e9pm.cnt_enter, d0 = e9pm.dry.calls, sr0 = e9pm.cnt_sys_resume;
	struct crownlte_wdt_state w = { };
	const char *why, *bad = NULL;
	unsigned int diff;
	u64 t0;
	int ret;

	why = s0_suspend_refused();
	if (why) {
		s0_set(3, S0_SKIPPED, "%s", why);
		pr_err("S0d refused: %s\n", why);
		return;
	}
	pr_info("S0d start: full suspend-to-RAM, .enter = dry run with a pending self-IPI (no SMC)\n");
	t0 = boot_ms();
	e9810_snap(&e9pm.pre);
	ret = pm_suspend(PM_SUSPEND_MEM);
	s3c2410wdt_crownlte_state(&w);

	if (e9pm.cnt_enter - e0 != 1 || e9pm.dry.calls - d0 != 1)
		bad = ".enter / dry finisher not run exactly once";
	else if (e9pm.enter_ret != -EOPNOTSUPP || ret != e9pm.enter_ret)
		bad = "unexpected return path";
	else if (!e9pm.ipi_pending)
		bad = "self-IPI not seen pending before the call";
	else if (e9pm.dry.fn != e9pm.psci_fn || e9pm.dry.state != e9pm.psci_state ||
		 e9pm.dry.entry != __pa_symbol(cpu_resume) || !e9pm.dry.entry)
		bad = "firmware call parameters wrong";
	else if (e9pm.cnt_sys_resume - sr0 != 1 || !e9pm.last_resume_early ||
		 e9pm.last_resume_ret)
		bad = "earlywakeup sequence failed";
	else if (e9pm.enter_frc1 <= e9pm.enter_frc0 ||
		 e9pm.enter_cnt1 <= e9pm.enter_cnt0)
		bad = "FRC/arch counter not counting";
	else if (e9810_check_all_back("S0d:"))
		bad = "not all CPUs back online";
	else if (e9810_ping_all("S0d:"))
		bad = "a CPU does not answer";
	else if (!w.enabled || !w.reset || w.pet_stopped)
		bad = "watchdog not armed after resume";
	else if (!s0_regs_ok("S0d:", &diff))
		bad = "PMU state not restored";

	pr_info("S0d pm_suspend=%d in %llu ms: enter=%d (early) ipi_pending=%d fn=%#x state=%#x entry=%#lx earlywakeup seq %s cpu0-WFI-in-window=%d frc %llu->%llu cpus %u/%u -> %s%s\n",
		ret, boot_ms() - t0, e9pm.enter_ret, e9pm.ipi_pending, e9pm.dry.fn,
		e9pm.dry.state, e9pm.dry.entry,
		e9pm.last_resume_ret ? "FAILED" : "ok", e9pm.window_grp2_latched,
		e9pm.enter_frc0, e9pm.enter_frc1, num_online_cpus(),
		num_possible_cpus(), bad ? "FAIL: " : "PASS", bad ? bad : "");
	e9810_apm_print("S0d end:", 8);
	if (bad)
		s0_set(3, S0_FAIL, "%s (pm_suspend %d)", bad, ret);
	else
		s0_set(3, S0_PASS, "early return %d, earlywakeup ok", ret);
}

static void s0a_run(void)
{
	struct e9810_snap *s = &e9pm.base;
	unsigned int diff;

	e9810_snap(s);
	if (e9pm.boot_valid) {
		e9810_snap_print("S0a boot", &e9pm.boot);
		diff = e9810_snap_diff("S0a since boot:", &e9pm.boot, s);
		pr_info("S0a %u configuration registers changed since probe (probe-time hook reset expected when hooks are on)\n",
			diff);
	}
	e9810_snap_print("S0a", s);
	e9810_apm_print("S0a", 8);
	s0_set(0, S0_PASS, "CP_STAT=%#x INFORM0=%#x 0x610=%#x mapped all",
	       s->cp_stat, s->inform[0], s->wmask[0]);
	pr_info("S0a PASS (read-only) CP_STAT=%#x\n", s->cp_stat);
}

static void s0_summary(void)
{
	pr_info("SUMMARY S0a=%s [%s] S0b=%s [%s] S0c=%s [%s] S0d=%s [%s] lost=%*pbl\n",
		s0_result_name[e9pm.res[0]], e9pm.why[0],
		s0_result_name[e9pm.res[1]], e9pm.why[1],
		s0_result_name[e9pm.res[2]], e9pm.why[2],
		s0_result_name[e9pm.res[3]], e9pm.why[3],
		cpumask_pr_args(&e9pm.lost));
}

static bool s0_time_ok(int stage, u32 need_ms)
{
	u64 now = boot_ms();

	if (now + need_ms <= e9pm.end_by_ms)
		return true;
	s0_set(stage, S0_SKIPPED, "needs ~%u ms, %llu ms left before %u ms",
	       need_ms, e9pm.end_by_ms > now ? e9pm.end_by_ms - now : 0,
	       e9pm.end_by_ms);
	pr_warn("S0%c skipped: %s\n", 'a' + stage, e9pm.why[stage]);
	return false;
}

static int s0_thread(void *unused)
{
	bool ok = true;
	unsigned int i;
	u64 now;

	while ((now = boot_ms()) < e9pm.start_ms)
		msleep(min_t(u64, e9pm.start_ms - now, 1000));
	pr_info("deadline %u ms installed at probe in the watchdog ping path\n",
		e9pm.deadline_ms);

	s0a_run();

	if (e9pm.want[1]) {
		const char *why = s0_recovery_refused(true, false);

		if (why) {
			s0_set(1, S0_SKIPPED, "refused: %s", why);
			pr_err("S0b refused (no hotplug done): %s\n", why);
		} else if (s0_time_ok(1, 40000)) {
			s0b_run();
		}
		ok = e9pm.res[1] == S0_PASS;
	}
	if (e9pm.want[2]) {
		if (!ok)
			s0_set(2, S0_SKIPPED, "S0b did not pass");
		else if (s0_time_ok(2, 5 * (e9pm.pm_test_delay * 1000 + 3000)))
			s0c_run();
		ok = ok && e9pm.res[2] == S0_PASS;
	}
	if (e9pm.want[3]) {
		if (!ok)
			s0_set(3, S0_SKIPPED, "an earlier stage did not pass");
		else if (s0_time_ok(3, 10000))
			s0d_run();
	}

	for (i = 0; i < S0_REMINDERS; i++) {
		s0_summary();
		if (!cpumask_empty(&e9pm.lost))
			pr_emerg("CPU-LOST %*pbl%s\n", cpumask_pr_args(&e9pm.lost),
				 e9pm.lost_online ?
				 ": WDT petting stopped, expect a WDT reset + shim bounce" :
				 ": offline; the normal 120 s restart applies");
		if (i + 1 < S0_REMINDERS)
			msleep(S0_REMINDER_MS);
	}
	return 0;
}

/* ------------------------------------------------------------------------ */
/* Probe                                                                    */
/* ------------------------------------------------------------------------ */

static void __iomem *e9810_map(struct platform_device *pdev, const char *name)
{
	struct resource *res = platform_get_resource_byname(pdev,
							    IORESOURCE_MEM,
							    name);

	if (!res)
		return NULL;
	/* Shared with PMUCAL, cpuidle, ACPM, pinctrl, GIC: map, never request. */
	return devm_ioremap(&pdev->dev, res->start, resource_size(res));
}

static int e9810_pm_parse(struct device_node *np)
{
	struct device_node *psci;
	struct property *prop;
	const char *st;
	int n, i;

	if (of_property_read_u32(np, "samsung,sleep-mode-idx", &e9pm.sleep_mode) ||
	    e9pm.sleep_mode != E9810_SYS_SLEEP)
		return -EINVAL;
	if (of_property_read_u32(np, "samsung,sleep-psci-state", &e9pm.psci_state))
		return -EINVAL;
	n = of_property_count_u32_elems(np, "samsung,wakeup-mask-offsets");
	if (n <= 0 || n > E9810_MAX_WMASK || n < 4 ||
	    of_property_count_u32_elems(np, "samsung,wakeup-masks") != n)
		return -EINVAL;
	e9pm.nr_wmask = n;
	of_property_read_u32_array(np, "samsung,wakeup-mask-offsets",
				   e9pm.wmask_off, n);
	of_property_read_u32_array(np, "samsung,wakeup-masks", e9pm.wmask_val, n);
	/* Only the four 4.9 wakeup-mask registers (4.9 DT reg-offset values). */
	for (i = 0; i < n; i++)
		if (e9pm.wmask_off[i] != 0x610 && e9pm.wmask_off[i] != 0x614 &&
		    e9pm.wmask_off[i] != 0x618 && e9pm.wmask_off[i] != 0x644)
			return -EINVAL;
	e9pm.sleep_with_cp_off = of_property_read_bool(np, "samsung,sleep-with-cp-off");

	psci = of_find_compatible_node(NULL, NULL, "arm,psci");
	if (psci) {
		of_property_read_u32(psci, "cpu_suspend", &e9pm.psci_fn);
		of_node_put(psci);
	}

	of_property_for_each_string(np, "samsung,suspend-s0", prop, st) {
		if (strlen(st) == 1 && st[0] >= 'a' && st[0] <= 'd')
			e9pm.want[st[0] - 'a'] = true;
	}
	e9pm.start_ms = S0_START_MS_DEFAULT;
	e9pm.end_by_ms = S0_END_BY_MS_DEFAULT;
	e9pm.hp_cycles = S0_HP_CYCLES_DEFAULT;
	e9pm.cluster_cycles = S0_CLUSTER_CYCLES_DEFAULT;
	e9pm.group_cycles = S0_GROUP_CYCLES_DEFAULT;
	e9pm.pm_test_delay = S0_PM_TEST_DELAY_DEFAULT;
	of_property_read_u32(np, "samsung,suspend-s0-start-ms", &e9pm.start_ms);
	of_property_read_u32(np, "samsung,suspend-s0-end-by-ms", &e9pm.end_by_ms);
	of_property_read_u32(np, "samsung,suspend-s0-deadline-ms", &e9pm.deadline_ms);
	of_property_read_u32(np, "samsung,suspend-s0-hotplug-cycles", &e9pm.hp_cycles);
	of_property_read_u32(np, "samsung,suspend-s0-cluster-cycles",
			     &e9pm.cluster_cycles);
	of_property_read_u32(np, "samsung,suspend-s0-group-cycles",
			     &e9pm.group_cycles);
	of_property_read_u32(np, "samsung,suspend-s0-pm-test-delay-s",
			     &e9pm.pm_test_delay);
	e9pm.hp_cycles = clamp_t(u32, e9pm.hp_cycles, 1, 100);
	e9pm.cluster_cycles = clamp_t(u32, e9pm.cluster_cycles, 0, 20);
	e9pm.group_cycles = clamp_t(u32, e9pm.group_cycles, 0, 20);
	e9pm.pm_test_delay = clamp_t(u32, e9pm.pm_test_delay, 1, 5);
	/* Mandatory: no S0 run without a watchdog deadline. */
	if (!e9pm.deadline_ms)
		return -EINVAL;
	if (e9pm.deadline_ms < e9pm.end_by_ms + 10000)
		e9pm.deadline_ms = e9pm.end_by_ms + 10000;
	return 0;
}

/*
 * A failed probe is reported again at S0 start time: the vehicle's PID1
 * may drop early boot messages before the host connects.
 */
static char e9810_probe_why[96] = "probe never ran";
static int e9810_probe_ret = -ENODEV;

static void e9810_probe_report_fn(struct work_struct *work)
{
	if (READ_ONCE(e9pm.task))
		return;
	pr_err("SUMMARY S0a=not-run S0b=not-run S0c=not-run S0d=not-run: probe failed: %s (%d)\n",
	       e9810_probe_why, e9810_probe_ret);
}
static DECLARE_DELAYED_WORK(e9810_probe_report, e9810_probe_report_fn);

static __printf(3, 4) int e9810_probe_fail(struct platform_device *pdev,
					   int err, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(e9810_probe_why, sizeof(e9810_probe_why), fmt, ap);
	va_end(ap);
	e9810_probe_ret = err;
	if (err != -EPROBE_DEFER)
		pr_err("probe FAIL: %s (%d)\n", e9810_probe_why, err);
	return dev_err_probe(&pdev->dev, err, "%s\n", e9810_probe_why);
}

static int e9810_pm_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const char *why;
	int ret;

	e9pm.dev = &pdev->dev;
	ret = e9810_pm_parse(np);
	if (ret)
		return e9810_probe_fail(pdev, ret, "bad DT properties");
	if (!e9pm.want[0] && !e9pm.want[1] && !e9pm.want[2] && !e9pm.want[3])
		return e9810_probe_fail(pdev, -ENODEV,
					"no samsung,suspend-s0 stages: this build is S0 only");
	e9pm.want[0] = true;	/* S0a always runs first */

	e9pm.pmu = syscon_regmap_lookup_by_phandle(np, "samsung,pmu-syscon");
	if (IS_ERR(e9pm.pmu))
		return e9810_probe_fail(pdev, PTR_ERR(e9pm.pmu), "no PMU syscon");
	e9pm.intr_gen = e9810_map(pdev, "intr-gen");
	e9pm.alive = e9810_map(pdev, "gpio-alive");
	e9pm.gicd = e9810_map(pdev, "gicd");
	e9pm.mct = e9810_map(pdev, "mct");
	e9pm.fdbg = e9810_map(pdev, "flexpmu-dbg");
	e9pm.mbox = e9810_map(pdev, "apm-mailbox");
	if (!e9pm.intr_gen || !e9pm.alive || !e9pm.gicd || !e9pm.mct || !e9pm.fdbg ||
	    !e9pm.mbox)
		return e9810_probe_fail(pdev, -ENOMEM, "register maps");
	if (!cal_if_is_initialized())
		return e9810_probe_fail(pdev, -EPROBE_DEFER, "CAL not ready");
	/* Optional: without it the APM ring is just not printed. */
	e9pm.apm = devm_kzalloc(&pdev->dev, sizeof(*e9pm.apm), GFP_KERNEL);

	/*
	 * Before this driver's first access to a PMU, INTR_GEN, GPIO_ALIVE,
	 * GIC, MCT, FlexPMU or mailbox register, and before the first
	 * cross-CPU request: if a diagnostic read stalls CPU0 instead of
	 * aborting, the other CPUs stop reloading the counter at the deadline
	 * whatever they do. Up to here only the DT, the CAL init flag and the
	 * syscon/ioremap lookups were used; the watchdog accesses below are
	 * its own CL0 registers.
	 */
	ret = e9810_deadline_install();
	if (ret)
		return e9810_probe_fail(pdev, ret,
					"no armed, petted CL0 watchdog to carry the %u ms deadline",
					e9pm.deadline_ms);
	/* Verified: running, reset enabled, petted, our deadline, WTCNT moving. */
	why = s0_recovery_refused(true, false);
	if (why)
		return e9810_probe_fail(pdev, -EIO,
					"watchdog deadline %u ms not verified: %s",
					e9pm.deadline_ms, why);
	pr_info("probe: watchdog deadline %u ms installed and verified before any diagnostic register read or cross-CPU request\n",
		e9pm.deadline_ms);

	/* Before any write of this driver. */
	e9810_snap(&e9pm.boot);
	e9pm.boot_valid = true;

	cpus_read_lock();
	why = e9810_init_topology();
	cpus_read_unlock();
	if (why)
		return e9810_probe_fail(pdev, -ENODEV, "topology: %s", why);

	if (e9pm.want[1] || e9pm.want[2] || e9pm.want[3]) {
		why = e9810_setup_hooks();
		if (why)
			return e9810_probe_fail(pdev, -ENODEV, "hotplug hooks: %s",
						why);
	}
	if (e9pm.want[2] || e9pm.want[3]) {
		register_syscore_ops(&e9810_pm_syscore_ops);
		suspend_set_ops(&e9810_pm_ops);
		e9pm.ops = true;
	}

	pr_info("probe: stages %s%s%s%s start %u ms end-by %u ms deadline %u ms, hooks=%d ops=%d (no CPU_SUSPEND call exists in S0), mode %u psci fn %#x state %#x entry %#lx, masks %u, cp-off %s\n",
		e9pm.want[0] ? "a" : "", e9pm.want[1] ? "b" : "",
		e9pm.want[2] ? "c" : "", e9pm.want[3] ? "d" : "", e9pm.start_ms,
		e9pm.end_by_ms, e9pm.deadline_ms, e9pm.hooks, e9pm.ops, e9pm.sleep_mode,
		e9pm.psci_fn, e9pm.psci_state,
		(unsigned long)__pa_symbol(cpu_resume), e9pm.nr_wmask,
		e9pm.sleep_with_cp_off ? "allowed" : "refused");

	e9pm.task = kthread_create(s0_thread, NULL, "suspend-s0");
	if (IS_ERR(e9pm.task)) {
		ret = PTR_ERR(e9pm.task);
		e9pm.task = NULL;
		snprintf(e9810_probe_why, sizeof(e9810_probe_why),
			 "self-test thread not created");
		e9810_probe_ret = ret;
		pr_err("self-test thread: %d\n", ret);
		return 0;
	}
	kthread_bind(e9pm.task, 0);
	wake_up_process(e9pm.task);
	return 0;
}

static const struct of_device_id e9810_pm_of_match[] = {
	{ .compatible = "samsung,exynos9810-pm" },
	{ },
};

static struct platform_driver e9810_pm_driver = {
	.probe = e9810_pm_probe,
	.driver = {
		.name = "exynos9810-pm",
		.of_match_table = e9810_pm_of_match,
		.suppress_bind_attrs = true,
	},
};

static int __init e9810_pm_init(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "samsung,exynos9810-pm");
	if (np) {
		if (of_device_is_available(np))
			queue_delayed_work(system_long_wq, &e9810_probe_report,
					   msecs_to_jiffies(S0_START_MS_DEFAULT + 1000));
		of_node_put(np);
	}
	return platform_driver_register(&e9810_pm_driver);
}
device_initcall(e9810_pm_init);
