// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 9810 MCT global comparator 0 as the tick broadcast device.
 *
 * On 6.12 the per-CPU tick runs on the arch timer, which stops when the APM
 * powers a core down (C2). The only broadcast device the kernel otherwise
 * has is the hrtimer one from tick_setup_hrtimer_broadcast(). That one is
 * unusable here:
 *
 *  - With HIGH_RES_TIMERS=n (the crownlte diag config) bc_set_next() only
 *    queues an hrtimer. __hrtimer_reprogram() does not touch the hardware in
 *    low-resolution mode, and the CPU that armed it has usually stopped its
 *    tick already, so broadcast wakeups are delayed until something else
 *    wakes that CPU.
 *  - Whichever CPU owns the broadcast hrtimer may not enter a
 *    local-timer-stop state (broadcast_needs_cpu()). With C2 enabled on a
 *    single CPU, that CPU arms the hrtimer itself and is then refused C2.
 *
 * 4.9 never needed a broadcast: its per-CPU ticks ran on the always-on MCT
 * local timers and the arch timer was a clocksource only. 4.9 also
 * registered G_COMP0 as "mct-comp" and programmed it the way this driver
 * does, so the register sequence is the one the 4.9 kernel used on this SoC.
 *
 * The upstream exynos_mct driver is not used. Its TIMER_OF_DECLARE probe
 * runs from time_init() and panics if a clock or MCT write fails, which on
 * a RECOVERY trial means a boot loop. It also registers a 32-bit sched_clock
 * that would replace the arch timer's. This driver probes as a normal
 * platform device, checks the FRC rate against the arch counter and the
 * comparator interrupt with real events before registering, and never
 * panics. If any check fails, nothing is registered and the hrtimer broadcast
 * stays in place. The Exynos 9810 cpuidle driver then refuses to arm C2.
 *
 * G_COMP0 and its interrupt (MCT G0, GIC SPI 466 on 4.9) are always used.
 * The FRC is started if it is not counting, as 4.9 exynos4_mct_frc_start()
 * did at every boot.
 *
 * Per-CPU local timers (DT samsung,mct-local-timers, runtime C2 stage S2).
 * 4.9 ran every CPU's tick on its own MCT local timer: L0-L7 at
 * 0x300 + 0x100 * N, one SPI each (470-477), "mct_tickN" in
 * /proc/interrupts, rating 500 above the arch timer, interrupt counter
 * stepping every TCNTB + 1 = 2 MCT clocks, arch timer as clocksource only.
 * Its tick broadcast never fired (0 broadcast IPIs), and every timer wake of
 * a core in C2 was a separate SPI to that core. On 6.12 the tick runs on the
 * arch timer, which stops in C2, so every timer wake is an IPI_TIMER sent by
 * the G0 broadcast handler on CPU0, often one SGI to several cores at once.
 * Three LITTLE cores cycling C2 that way lose a core in EL3
 * (roadmap/c2-runtime-design.md 2.4, H1). With the property this driver
 * registers L<cpu> as the tick device of every CPU that passes a self-check
 * (an interrupt on that CPU, at the 4.9 rate, three times), with the 4.9
 * register sequence. The MCT is outside the CPU power domains (4.9 relies on
 * it across C2), so these CPUs never need the broadcast. G0 stays the
 * broadcast device for any CPU whose local timer is not in use. Nothing
 * changes without the property.
 */

#define pr_fmt(fmt) "exynos9810-mct-bc: " fmt

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/completion.h>
#include <linux/cpu.h>
#include <linux/cpuhotplug.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/irqnr.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/percpu.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include <clocksource/arm_arch_timer.h>
#include <soc/samsung/exynos9810-mct-bc.h>

#define MCT_G_CNT_L			0x100
#define MCT_G_CNT_U			0x104
#define MCT_G_CNT_WSTAT			0x110
#define MCT_G_COMP0_L			0x200
#define MCT_G_COMP0_U			0x204
#define MCT_G_TCON			0x240
#define MCT_G_INT_CSTAT			0x244
#define MCT_G_INT_ENB			0x248
#define MCT_G_WSTAT			0x24c

#define MCT_G_TCON_START		BIT(8)
#define MCT_G_TCON_COMP0_AUTO_INC	BIT(1)
#define MCT_G_TCON_COMP0_ENABLE		BIT(0)

#define MCT_WSTAT_COMP0_L		BIT(0)
#define MCT_WSTAT_COMP0_U		BIT(1)
#define MCT_WSTAT_TCON			BIT(16)

/* Each poll is an APB read of a 26 MHz block: well over 1 ms in total. */
#define MCT_WSTAT_POLLS			20000

/*
 * The comparator matches on equality. Programming takes three synchronised
 * writes, so keep the minimum delta well above their cost (50 us at 26 MHz)
 * and re-check the counter afterwards.
 */
#define MCT_MIN_DELTA			1300
#define MCT_MAX_DELTA			0x7fffffff
#define MCT_RATING			250

#define MCT_TEST_ROUNDS			3
#define MCT_TEST_DELAY_US		5000
#define MCT_TEST_LATE_MAX_US		1000
#define MCT_RATE_TOLERANCE_PPM		10000
#define MCT_REPORT_MS			86000
#define MCT_TICK_WINDOW_MS		2000

/* Local timers L0-L7 (4.9 exynos_mct.c). */
#define MCT_NR_LOCAL			8
#define MCT_L_BASE(n)			(0x300 + 0x100 * (n))
#define MCT_L_TCNTB			0x00
#define MCT_L_ICNTB			0x08
#define MCT_L_ICNTO			0x0c
#define MCT_L_TCON			0x20
#define MCT_L_INT_CSTAT			0x30
#define MCT_L_INT_ENB			0x34
#define MCT_L_WSTAT			0x40
/* L7 ends at 0xa44: more than the 0x800 the G0-only node maps. */
#define MCT_L_REG_END			(MCT_L_BASE(MCT_NR_LOCAL - 1) + \
					 MCT_L_WSTAT + 4)

#define MCT_L_TCON_INTERVAL_MODE	BIT(2)
#define MCT_L_TCON_INT_START		BIT(1)
#define MCT_L_TCON_TIMER_START		BIT(0)
#define MCT_L_TCON_RUN			(MCT_L_TCON_INTERVAL_MODE | \
					 MCT_L_TCON_INT_START | \
					 MCT_L_TCON_TIMER_START)
#define MCT_L_ICNTB_UPDATE		BIT(31)
#define MCT_L_WSTAT_TCNTB		BIT(0)
#define MCT_L_WSTAT_ICNTB		BIT(1)
#define MCT_L_WSTAT_TCON		BIT(3)

/* DT interrupts in the 4.9 order: G0, G1, G2, G3, then L0-L7. */
#define MCT_L_IRQ_INDEX(n)		(4 + (n))
/* 4.9 TICK_BASE_CNT: the interrupt counter steps every 2 MCT clocks. */
#define MCT_L_TCNTB_VAL			1
#define MCT_L_RATING			500	/* 4.9: above the arch timer (450) */
#define MCT_L_MIN_DELTA			0xf	/* 4.9 */
#define MCT_L_MAX_DELTA			0x7fffffff

#define MCT_L_TEST_ROUNDS		3
#define MCT_L_TEST_US			10000
#define MCT_L_TEST_TOL_PCT		2
#define MCT_L_TEST_LATE_US		1000
/* Usage windows: the first 90-150 s boot time, then every 10 min. */
#define MCT_L_WINDOW_START_MS		90000
#define MCT_L_WINDOW_FIRST_MS		60000
#define MCT_L_WINDOW_MS			600000

/* IPI_TIMER of arch/arm64/kernel/smp.c: IPI n is SGI n, requested as "IPI". */
#define MCT_IPI_TIMER_HWIRQ		4

static void __iomem *mct_base;
static int mct_irq = -1;
static u32 mct_rate;
static bool mct_ready;
static u64 mct_ready_ms;		/* boot time of registration, 0 = never */
static bool mct_irq_stuck;
static const char *mct_why = "not probed";

/* Probe self-check results, re-printed by the late report. */
static struct {
	u32 tcon_at_probe;
	bool frc_started;
	u64 frc_delta, arch_delta;
	s64 rate_ppm;
	unsigned int fired;
	u64 late_max_cycles;
	bool early;
} mct_probe;

/* Self-check mode: the ISR completes this instead of calling the tick code. */
static bool mct_testing;
static DECLARE_COMPLETION(mct_test_done);
static u64 mct_test_target, mct_test_seen;

static atomic64_t mct_irqs = ATOMIC64_INIT(0);
static atomic64_t mct_programmed = ATOMIC64_INIT(0);
static atomic64_t mct_etime = ATOMIC64_INIT(0);
static atomic64_t mct_wtimeouts = ATOMIC64_INIT(0);

static struct delayed_work mct_report_work;

/* One per CPU: logical CPU N uses local timer L<N> and its SPI, as on 4.9. */
struct mct_local {
	struct clock_event_device evt;
	unsigned long base;		/* MCT_L_BASE(cpu) */
	int irq;			/* -1: none */
	bool tested;			/* passed the self-check: may register */
	bool registered;		/* registered while this CPU is up */
	bool stuck;			/* interrupt did not clear: given up */
	u64 irqs;			/* written by the CPU itself (ISR) */
	u64 wtimeouts;			/* register write never confirmed */
	/* self-check */
	bool testing;
	struct completion test_done;
	u64 test_seen;
	int test_cpu;
	u64 test_min, test_max;		/* MCT clocks from start to the ISR */
	char name[12];
};

static DEFINE_PER_CPU(struct mct_local, mct_local);
static bool mct_local_on;		/* the cpuhp state is installed */
static const char *mct_local_why = "not requested (no DT samsung,mct-local-timers)";
static u32 mct_local_rate;
static struct cpumask mct_local_tested;
static u64 mct_l_test_min = U64_MAX, mct_l_test_max;
static int mct_ipi_timer_irq = -1;

/* Usage window snapshots (mct_local_window()). */
static struct {
	bool valid;
	u64 at_ms, g0;
	u64 local[NR_CPUS];
	unsigned int ipi[NR_CPUS];
} mct_win;
static struct delayed_work mct_local_work;

static bool mct_write(u32 value, unsigned long offset, unsigned long stat,
		      u32 mask)
{
	unsigned int i;

	writel_relaxed(value, mct_base + offset);
	for (i = 0; i < MCT_WSTAT_POLLS; i++) {
		if (readl_relaxed(mct_base + stat) & mask) {
			writel_relaxed(mask, mct_base + stat);
			return true;
		}
	}
	atomic64_inc(&mct_wtimeouts);
	return false;
}

static u64 mct_read_count(void)
{
	u32 hi, lo, hi2;

	hi2 = readl_relaxed(mct_base + MCT_G_CNT_U);
	do {
		hi = hi2;
		lo = readl_relaxed(mct_base + MCT_G_CNT_L);
		hi2 = readl_relaxed(mct_base + MCT_G_CNT_U);
	} while (hi != hi2);
	return ((u64)hi << 32) | lo;
}

static bool mct_comp0_stop(void)
{
	u32 tcon = readl_relaxed(mct_base + MCT_G_TCON);
	bool ok;

	tcon &= ~(MCT_G_TCON_COMP0_ENABLE | MCT_G_TCON_COMP0_AUTO_INC);
	ok = mct_write(tcon, MCT_G_TCON, MCT_G_WSTAT, MCT_WSTAT_TCON);
	writel_relaxed(0, mct_base + MCT_G_INT_ENB);
	return ok;
}

/* Returns the programmed compare value, or 0 if a write did not complete. */
static u64 mct_comp0_start(unsigned long cycles)
{
	u32 tcon = readl_relaxed(mct_base + MCT_G_TCON);
	u64 comp = mct_read_count() + cycles;

	if (!mct_write(lower_32_bits(comp), MCT_G_COMP0_L, MCT_G_WSTAT,
		       MCT_WSTAT_COMP0_L) ||
	    !mct_write(upper_32_bits(comp), MCT_G_COMP0_U, MCT_G_WSTAT,
		       MCT_WSTAT_COMP0_U))
		return 0;
	writel_relaxed(1, mct_base + MCT_G_INT_ENB);
	tcon &= ~MCT_G_TCON_COMP0_AUTO_INC;
	tcon |= MCT_G_TCON_COMP0_ENABLE;
	if (!mct_write(tcon, MCT_G_TCON, MCT_G_WSTAT, MCT_WSTAT_TCON))
		return 0;
	return comp;
}

static int mct_set_next_event(unsigned long cycles,
			      struct clock_event_device *evt)
{
	u64 comp = mct_comp0_start(cycles);

	atomic64_inc(&mct_programmed);
	/*
	 * An equality match that was passed while programming would only fire
	 * after the counter wraps. Let the core retry with a larger delta.
	 */
	if (!comp || mct_read_count() >= comp) {
		atomic64_inc(&mct_etime);
		return -ETIME;
	}
	return 0;
}

static int mct_set_state_shutdown(struct clock_event_device *evt)
{
	mct_comp0_stop();
	return 0;
}

static struct clock_event_device mct_bc_evt = {
	.name			= "mct-bc",
	.features		= CLOCK_EVT_FEAT_ONESHOT,
	.rating			= MCT_RATING,
	.set_next_event		= mct_set_next_event,
	.set_state_shutdown	= mct_set_state_shutdown,
	.set_state_oneshot	= mct_set_state_shutdown,
	.set_state_oneshot_stopped = mct_set_state_shutdown,
	.tick_resume		= mct_set_state_shutdown,
};

static irqreturn_t mct_bc_isr(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	writel_relaxed(1, mct_base + MCT_G_INT_CSTAT);
	atomic64_inc(&mct_irqs);

	/*
	 * A level interrupt that does not clear would storm CPU0 on every
	 * boot. Give up on the device instead; C2 then stays disarmed and
	 * CPUs already in C2 are caught by the self-test as late wakeups.
	 */
	if (readl_relaxed(mct_base + MCT_G_INT_CSTAT) & 1) {
		writel_relaxed(0, mct_base + MCT_G_INT_ENB);
		if (readl_relaxed(mct_base + MCT_G_INT_CSTAT) & 1) {
			disable_irq_nosync(irq);
			WRITE_ONCE(mct_ready, false);
			WRITE_ONCE(mct_irq_stuck, true);
			if (READ_ONCE(mct_testing))
				complete(&mct_test_done);
			return IRQ_HANDLED;
		}
	}

	if (READ_ONCE(mct_testing)) {
		mct_test_seen = mct_read_count();
		mct_comp0_stop();
		complete(&mct_test_done);
		return IRQ_HANDLED;
	}
	if (evt->event_handler)
		evt->event_handler(evt);
	return IRQ_HANDLED;
}

static u64 mct_cycles_to_us(u64 cycles)
{
	return div_u64(cycles * USEC_PER_SEC, mct_rate);
}

/* FRC running and counting at the arch timer rate (both run from OSCCLK). */
static const char *mct_check_frc(void)
{
	unsigned long flags;
	u64 m0, m1, a0, a1;

	mct_probe.tcon_at_probe = readl_relaxed(mct_base + MCT_G_TCON);
	m0 = mct_read_count();
	udelay(100);
	m1 = mct_read_count();
	if (m1 == m0) {
		u32 tcon = mct_probe.tcon_at_probe | MCT_G_TCON_START;

		pr_info("FRC not counting (G_TCON=%#x), starting it as 4.9 did\n",
			mct_probe.tcon_at_probe);
		if (!mct_write(tcon, MCT_G_TCON, MCT_G_WSTAT, MCT_WSTAT_TCON))
			return "G_TCON write did not complete";
		mct_probe.frc_started = true;
		m0 = mct_read_count();
		udelay(100);
		if (mct_read_count() == m0)
			return "FRC still not counting";
	}

	local_irq_save(flags);
	m0 = mct_read_count();
	a0 = arch_timer_read_counter();
	local_irq_restore(flags);
	msleep(20);
	local_irq_save(flags);
	m1 = mct_read_count();
	a1 = arch_timer_read_counter();
	local_irq_restore(flags);

	mct_probe.frc_delta = m1 - m0;
	mct_probe.arch_delta = a1 - a0;
	if (!mct_probe.arch_delta)
		return "arch counter not counting";
	mct_probe.rate_ppm = div64_s64(((s64)mct_probe.frc_delta -
					(s64)mct_probe.arch_delta) * 1000000,
				       (s64)mct_probe.arch_delta);
	if (abs(mct_probe.rate_ppm) > MCT_RATE_TOLERANCE_PPM)
		return "FRC rate differs from the arch counter";
	return NULL;
}

/* Real comparator events on CPU0 before the tick code may rely on them. */
static const char *mct_check_irq(void)
{
	unsigned long cycles = div_u64((u64)mct_rate * MCT_TEST_DELAY_US,
				       USEC_PER_SEC);
	unsigned int i;

	WRITE_ONCE(mct_testing, true);
	for (i = 0; i < MCT_TEST_ROUNDS; i++) {
		reinit_completion(&mct_test_done);
		mct_test_target = mct_comp0_start(cycles);
		if (!mct_test_target) {
			mct_comp0_stop();
			WRITE_ONCE(mct_testing, false);
			return "comparator write did not complete";
		}
		if (!wait_for_completion_timeout(&mct_test_done,
						 msecs_to_jiffies(100))) {
			mct_comp0_stop();
			WRITE_ONCE(mct_testing, false);
			return "comparator interrupt did not arrive within 100 ms";
		}
		if (READ_ONCE(mct_irq_stuck)) {
			mct_comp0_stop();
			WRITE_ONCE(mct_testing, false);
			return "comparator interrupt does not clear; IRQ disabled";
		}
		if (mct_test_seen < mct_test_target) {
			mct_probe.early = true;
			WRITE_ONCE(mct_testing, false);
			return "comparator interrupt fired early";
		}
		mct_probe.late_max_cycles = max(mct_probe.late_max_cycles,
						mct_test_seen - mct_test_target);
		mct_probe.fired++;
	}
	WRITE_ONCE(mct_testing, false);
	if (mct_cycles_to_us(mct_probe.late_max_cycles) > MCT_TEST_LATE_MAX_US)
		return "comparator interrupt handled too late";
	return NULL;
}

/*
 * Local timers. The register sequences are 4.9 exynos_mct.c's
 * (exynos4_mct_write(), exynos4_mct_tick_stop(), exynos4_mct_tick_start(),
 * set_state_periodic(), set_state_resume(), exynos4_mct_starting_cpu() and
 * exynos4_mct_dying_cpu()). Only TCNTB, ICNTB and TCON report a write
 * status (L_WSTAT bits 0, 1 and 3); INT_ENB and INT_CSTAT are plain writes.
 */
static bool mct_l_write(struct mct_local *ml, u32 value, unsigned long reg,
			u32 wmask)
{
	void __iomem *stat = mct_base + ml->base + MCT_L_WSTAT;
	unsigned int i;

	writel_relaxed(value, mct_base + ml->base + reg);
	for (i = 0; i < MCT_WSTAT_POLLS; i++) {
		if (readl_relaxed(stat) & wmask) {
			writel_relaxed(wmask, stat);
			return true;
		}
	}
	WRITE_ONCE(ml->wtimeouts, ml->wtimeouts + 1);
	return false;
}

/* 4.9 exynos4_mct_tick_stop(): ack, then stop unless periodic (or @force). */
static void mct_l_stop(struct mct_local *ml, bool force)
{
	u32 tcon;

	writel_relaxed(1, mct_base + ml->base + MCT_L_INT_CSTAT);
	if (!force && clockevent_state_periodic(&ml->evt))
		return;
	tcon = readl_relaxed(mct_base + ml->base + MCT_L_TCON);
	tcon &= ~MCT_L_TCON_RUN;
	mct_l_write(ml, tcon, MCT_L_TCON, MCT_L_WSTAT_TCON);
}

/* 4.9 exynos4_mct_tick_start(). False if a write did not complete. */
static bool mct_l_start(struct mct_local *ml, unsigned long cycles,
			bool periodic)
{
	u32 tcon = MCT_L_TCON_INT_START | MCT_L_TCON_TIMER_START;
	bool ok;

	ok = mct_l_write(ml, MCT_L_ICNTB_UPDATE | cycles, MCT_L_ICNTB,
			 MCT_L_WSTAT_ICNTB);
	writel_relaxed(1, mct_base + ml->base + MCT_L_INT_ENB);
	if (periodic)
		tcon |= MCT_L_TCON_INTERVAL_MODE;
	ok &= mct_l_write(ml, tcon, MCT_L_TCON, MCT_L_WSTAT_TCON);
	return ok;
}

/*
 * A write that never completes is counted (write_timeouts, reported) where
 * 4.9 panicked. The usage window reports it as FAIL.
 */
static int mct_l_set_next_event(unsigned long cycles,
				struct clock_event_device *evt)
{
	struct mct_local *ml = container_of(evt, struct mct_local, evt);

	mct_l_start(ml, cycles, false);
	return 0;
}

static int mct_l_set_shutdown(struct clock_event_device *evt)
{
	mct_l_stop(container_of(evt, struct mct_local, evt), true);
	return 0;
}

static int mct_l_set_periodic(struct clock_event_device *evt)
{
	struct mct_local *ml = container_of(evt, struct mct_local, evt);
	unsigned long cycles = ((u64)NSEC_PER_SEC / HZ * evt->mult) >>
			       evt->shift;

	mct_l_stop(ml, true);
	mct_l_start(ml, cycles, true);
	return 0;
}

static int mct_l_resume(struct clock_event_device *evt)
{
	struct mct_local *ml = container_of(evt, struct mct_local, evt);

	mct_l_stop(ml, true);
	mct_l_write(ml, MCT_L_TCNTB_VAL, MCT_L_TCNTB, MCT_L_WSTAT_TCNTB);
	return 0;
}

static irqreturn_t mct_l_isr(int irq, void *dev_id)
{
	struct mct_local *ml = dev_id;
	bool testing = READ_ONCE(ml->testing);
	unsigned int i;

	if (testing) {
		ml->test_seen = mct_read_count();
		ml->test_cpu = smp_processor_id();
	}
	mct_l_stop(ml, testing);
	WRITE_ONCE(ml->irqs, ml->irqs + 1);

	/*
	 * The acknowledge is a plain write: give it the time of a few
	 * register reads. A level interrupt that never clears would storm
	 * this CPU; give up on the timer instead. The CPU then has no tick
	 * (its hrtimers stop), which the buddy hard-lockup detector turns
	 * into a panic and the C2 record into UNCLEAN: loud, not a hang.
	 */
	for (i = 0; i < MCT_WSTAT_POLLS; i++)
		if (!(readl_relaxed(mct_base + ml->base + MCT_L_INT_CSTAT) & 1))
			break;
	if (i == MCT_WSTAT_POLLS) {
		writel_relaxed(0, mct_base + ml->base + MCT_L_INT_ENB);
		writel_relaxed(1, mct_base + ml->base + MCT_L_INT_CSTAT);
		if (readl_relaxed(mct_base + ml->base + MCT_L_INT_CSTAT) & 1) {
			disable_irq_nosync(irq);
			WRITE_ONCE(ml->stuck, true);
			if (!testing)
				printk_deferred(KERN_EMERG pr_fmt("%s: interrupt does not clear; IRQ disabled, this CPU has no tick now\n"),
						ml->name);
		}
	}

	if (testing) {
		complete(&ml->test_done);
		return IRQ_HANDLED;
	}
	if (ml->evt.event_handler)
		ml->evt.event_handler(&ml->evt);
	return IRQ_HANDLED;
}

/*
 * Real local timer events before the tick code may rely on them: the
 * interrupt must reach @cpu, clear, and come after cycles x (TCNTB + 1)
 * MCT clocks (the 4.9 rate), within 2 % + 1 ms.
 */
static const char *mct_l_test(unsigned int cpu)
{
	struct mct_local *ml = per_cpu_ptr(&mct_local, cpu);
	unsigned long cycles = div_u64((u64)mct_local_rate * MCT_L_TEST_US,
				       USEC_PER_SEC);
	u64 expect = (u64)cycles * (MCT_L_TCNTB_VAL + 1);
	u64 tol = div_u64(expect * MCT_L_TEST_TOL_PCT, 100);
	u64 late = div_u64((u64)mct_rate * MCT_L_TEST_LATE_US, USEC_PER_SEC);
	const char *why = NULL;
	unsigned int i;
	u64 t0, d;

	if (irq_force_affinity(ml->irq, cpumask_of(cpu)))
		return "cannot route its interrupt to the CPU";
	if (!mct_l_write(ml, MCT_L_TCNTB_VAL, MCT_L_TCNTB, MCT_L_WSTAT_TCNTB))
		return "TCNTB write did not complete";

	ml->test_min = U64_MAX;
	ml->test_max = 0;
	WRITE_ONCE(ml->testing, true);
	enable_irq(ml->irq);
	for (i = 0; i < MCT_L_TEST_ROUNDS; i++) {
		reinit_completion(&ml->test_done);
		ml->test_cpu = -1;
		t0 = mct_read_count();
		if (!mct_l_start(ml, cycles, false)) {
			why = "start write did not complete";
			break;
		}
		if (!wait_for_completion_timeout(&ml->test_done,
						 msecs_to_jiffies(100))) {
			why = "interrupt did not arrive within 100 ms";
			break;
		}
		if (READ_ONCE(ml->stuck)) {
			why = "interrupt does not clear; IRQ disabled";
			break;
		}
		if (ml->test_cpu != cpu) {
			why = "interrupt taken on another CPU";
			break;
		}
		d = ml->test_seen - t0;
		ml->test_min = min(ml->test_min, d);
		ml->test_max = max(ml->test_max, d);
		if (d + tol < expect) {
			why = "interrupt early: the counter is not MCT clock / 2";
			break;
		}
		if (d > expect + tol + late) {
			why = "interrupt late: the counter is not MCT clock / 2, or handled late";
			break;
		}
	}
	if (!READ_ONCE(ml->stuck))
		disable_irq(ml->irq);
	mct_l_stop(ml, true);
	writel_relaxed(0, mct_base + ml->base + MCT_L_INT_ENB);
	WRITE_ONCE(ml->testing, false);
	return why;
}

/* cpuhp STARTING, on @cpu with interrupts off (4.9 exynos4_mct_starting_cpu). */
static int mct_l_starting_cpu(unsigned int cpu)
{
	struct mct_local *ml = per_cpu_ptr(&mct_local, cpu);
	struct clock_event_device *evt = &ml->evt;

	/* Untested or given up: the arch timer and the broadcast stay. */
	if (!ml->tested || READ_ONCE(ml->stuck))
		return 0;

	evt->name = ml->name;
	evt->cpumask = cpumask_of(cpu);
	evt->irq = ml->irq;
	evt->set_next_event = mct_l_set_next_event;
	evt->set_state_periodic = mct_l_set_periodic;
	evt->set_state_shutdown = mct_l_set_shutdown;
	evt->set_state_oneshot = mct_l_set_shutdown;
	evt->set_state_oneshot_stopped = mct_l_set_shutdown;
	evt->tick_resume = mct_l_resume;
	/*
	 * PERCPU (as upstream): never a broadcast device candidate, which a
	 * rating of 500 without it would make it, in place of G0.
	 */
	evt->features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT |
			CLOCK_EVT_FEAT_PERCPU;
	evt->rating = MCT_L_RATING;

	mct_l_write(ml, MCT_L_TCNTB_VAL, MCT_L_TCNTB, MCT_L_WSTAT_TCNTB);
	irq_force_affinity(ml->irq, cpumask_of(cpu));
	enable_irq(ml->irq);
	ml->registered = true;
	clockevents_config_and_register(evt, mct_local_rate, MCT_L_MIN_DELTA,
					MCT_L_MAX_DELTA);
	return 0;
}

/* cpuhp STARTING teardown, on @cpu (4.9 exynos4_mct_dying_cpu). */
static int mct_l_dying_cpu(unsigned int cpu)
{
	struct mct_local *ml = per_cpu_ptr(&mct_local, cpu);

	if (!ml->registered)
		return 0;
	ml->registered = false;
	mct_l_stop(ml, true);
	if (!READ_ONCE(ml->stuck))
		disable_irq_nosync(ml->irq);
	writel_relaxed(1, mct_base + ml->base + MCT_L_INT_CSTAT);
	return 0;
}

static const char *mct_local_setup(struct platform_device *pdev)
{
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct device *dev = &pdev->dev;
	unsigned int cpu;
	const char *why;
	int irq, ret;

	if (!res || resource_size(res) < MCT_L_REG_END)
		return "register window ends before L7 (DT reg must cover 0xa44)";
	mct_local_rate = mct_rate / (MCT_L_TCNTB_VAL + 1);

	for_each_possible_cpu(cpu) {
		struct mct_local *ml = per_cpu_ptr(&mct_local, cpu);

		ml->irq = -1;
		init_completion(&ml->test_done);
		if (cpu >= MCT_NR_LOCAL)
			continue;
		ml->base = MCT_L_BASE(cpu);
		snprintf(ml->name, sizeof(ml->name), "mct_tick%u", cpu);
		irq = platform_get_irq_optional(pdev, MCT_L_IRQ_INDEX(cpu));
		if (irq < 0) {
			pr_warn("local timer L%u: no interrupt (DT index %u): cpu%u keeps the arch timer\n",
				cpu, MCT_L_IRQ_INDEX(cpu), cpu);
			continue;
		}
		/* Stopped, acknowledged and masked before the IRQ is taken. */
		mct_l_stop(ml, true);
		writel_relaxed(0, mct_base + ml->base + MCT_L_INT_ENB);
		writel_relaxed(1, mct_base + ml->base + MCT_L_INT_CSTAT);
		ret = devm_request_irq(dev, irq, mct_l_isr,
				       IRQF_TIMER | IRQF_NOBALANCING |
				       IRQF_PERCPU | IRQF_NO_AUTOEN,
				       ml->name, ml);
		if (ret) {
			pr_warn("local timer L%u: request_irq %d failed (%d): cpu%u keeps the arch timer\n",
				cpu, irq, ret, cpu);
			continue;
		}
		ml->irq = irq;
	}

	cpus_read_lock();
	for_each_online_cpu(cpu) {
		struct mct_local *ml = per_cpu_ptr(&mct_local, cpu);

		if (ml->irq < 0)
			continue;
		why = mct_l_test(cpu);
		if (why) {
			pr_warn("local timer L%u (irq %d) not used: %s (start->irq %llu..%llu MCT clocks, expected %llu): cpu%u keeps the arch timer\n",
				cpu, ml->irq, why,
				ml->test_min == U64_MAX ? 0 : ml->test_min,
				ml->test_max,
				div_u64((u64)mct_local_rate * MCT_L_TEST_US,
					USEC_PER_SEC) * (MCT_L_TCNTB_VAL + 1),
				cpu);
			continue;
		}
		ml->tested = true;
		cpumask_set_cpu(cpu, &mct_local_tested);
		mct_l_test_min = min(mct_l_test_min, ml->test_min);
		mct_l_test_max = max(mct_l_test_max, ml->test_max);
	}
	cpus_read_unlock();
	if (cpumask_empty(&mct_local_tested))
		return "no local timer passed its self-check";

	ret = cpuhp_setup_state(CPUHP_AP_EXYNOS4_MCT_TIMER_STARTING,
				"clockevents/exynos9810_mct:starting",
				mct_l_starting_cpu, mct_l_dying_cpu);
	if (ret < 0)
		return "MCT cpuhp state already in use";
	WRITE_ONCE(mct_local_on, true);
	return NULL;
}

bool exynos9810_mct_local_active(unsigned int cpu)
{
	struct mct_local *ml;

	if (!READ_ONCE(mct_local_on) || cpu >= nr_cpu_ids)
		return false;
	ml = per_cpu_ptr(&mct_local, cpu);
	return !READ_ONCE(ml->stuck) && tick_cpu_evtdev(cpu) == &ml->evt;
}

void exynos9810_mct_local_get_stats(unsigned int cpu,
				    struct exynos9810_mct_local_stats *s)
{
	struct mct_local *ml = per_cpu_ptr(&mct_local, cpu);

	s->irqs = READ_ONCE(ml->irqs);
	s->write_timeouts = READ_ONCE(ml->wtimeouts);
	s->active = exynos9810_mct_local_active(cpu);
}

/*
 * The "Timer broadcast interrupts" line of /proc/interrupts: the IPI_TIMER
 * descriptor (IRQ_HIDDEN, but its kstat counts per CPU).
 */
static void mct_find_ipi_timer(void)
{
	struct irq_desc *desc;
	unsigned int irq;

	for_each_irq_desc(irq, desc) {
		struct irqaction *act = desc->action;

		if (act && act->name && !strcmp(act->name, "IPI") &&
		    irqd_to_hwirq(&desc->irq_data) == MCT_IPI_TIMER_HWIRQ) {
			mct_ipi_timer_irq = irq;
			return;
		}
	}
}

static unsigned int mct_bc_ipis(unsigned int cpu)
{
	return mct_ipi_timer_irq < 0 ? 0 : kstat_irqs_cpu(mct_ipi_timer_irq, cpu);
}

/* CPUs whose tick runs on their MCT local timer now (for log lines). */
static const struct cpumask *mct_local_active_mask(void)
{
	static struct cpumask m;
	unsigned int cpu;

	cpumask_clear(&m);
	for_each_online_cpu(cpu)
		if (exynos9810_mct_local_active(cpu))
			cpumask_set_cpu(cpu, &m);
	return &m;
}

/* Jiffies from now until @ms boot time (0 if that has passed). */
static unsigned long mct_jiffies_until(unsigned int ms)
{
	unsigned long now = jiffies - INITIAL_JIFFIES;
	unsigned long at = msecs_to_jiffies(ms);

	return at > now ? at - now : 0;
}

static const char *mct_tick_name(unsigned int cpu)
{
	struct clock_event_device *td = tick_cpu_evtdev(cpu);

	return td && td->name ? td->name : "none";
}

/*
 * Proof that the local timers are in use, at 150 s boot time (window from
 * 90 s) and then every 10 min: every online CPU ticks on its own
 * mct_tickN, each took local timer interrupts in the window, and the
 * broadcast stayed idle (no G0 interrupt, no timer broadcast IPI). 4.9
 * shows 0 broadcast IPIs on every CPU.
 */
static void mct_local_window(struct work_struct *work)
{
	u64 now = ktime_to_ms(ktime_get_boottime()), g0 = atomic64_read(&mct_irqs);
	u64 lmin = U64_MAX, lmax = 0, wt = 0, secs;
	unsigned int cpu, ipis = 0;
	struct cpumask active, not_local, silent;
	bool pass;

	cpumask_clear(&active);
	cpumask_clear(&not_local);
	cpumask_clear(&silent);
	if (mct_win.valid) {
		secs = max_t(u64, (now - mct_win.at_ms) / MSEC_PER_SEC, 1);
		for_each_online_cpu(cpu) {
			struct mct_local *ml = per_cpu_ptr(&mct_local, cpu);
			u64 l = READ_ONCE(ml->irqs) - mct_win.local[cpu];
			unsigned int ipi = mct_bc_ipis(cpu) - mct_win.ipi[cpu];

			pr_info("mct-local: window %llu s: cpu%u tick=%s local_irqs=+%llu (%llu/s) timer_bc_ipis=+%u write_timeouts=%llu\n",
				secs, cpu, mct_tick_name(cpu), l, div64_u64(l, secs),
				ipi, READ_ONCE(ml->wtimeouts));
			if (exynos9810_mct_local_active(cpu))
				cpumask_set_cpu(cpu, &active);
			else
				cpumask_set_cpu(cpu, &not_local);
			if (!l)
				cpumask_set_cpu(cpu, &silent);
			lmin = min(lmin, l);
			lmax = max(lmax, l);
			ipis += ipi;
			wt += READ_ONCE(ml->wtimeouts);
		}
		pass = cpumask_empty(&not_local) && cpumask_empty(&silent) &&
		       !ipis && g0 == mct_win.g0 && !wt &&
		       mct_ipi_timer_irq >= 0;
		pr_info("mct-local: %s window=%llu s: online %*pbl, on their MCT local timer %*pbl (not: %*pbl), no local irq: %*pbl, local irqs/CPU +%llu..+%llu, broadcast mct-bc G0 irqs +%llu, timer broadcast IPIs +%u%s, write_timeouts %llu\n",
			pass ? "PASS" : "FAIL", secs,
			cpumask_pr_args(cpu_online_mask),
			cpumask_pr_args(&active),
			cpumask_pr_args(&not_local), cpumask_pr_args(&silent),
			lmin == U64_MAX ? 0 : lmin, lmax, g0 - mct_win.g0, ipis,
			mct_ipi_timer_irq < 0 ? " (IPI_TIMER not found)" : "",
			wt);
	}

	mct_win.at_ms = now;
	mct_win.g0 = g0;
	for_each_possible_cpu(cpu) {
		mct_win.local[cpu] = READ_ONCE(per_cpu_ptr(&mct_local, cpu)->irqs);
		mct_win.ipi[cpu] = mct_bc_ipis(cpu);
	}
	queue_delayed_work(system_long_wq, &mct_local_work,
			   msecs_to_jiffies(mct_win.valid ? MCT_L_WINDOW_MS :
					    MCT_L_WINDOW_FIRST_MS));
	mct_win.valid = true;
}

bool exynos9810_mct_bc_ready(void)
{
	return READ_ONCE(mct_ready);
}

u64 exynos9810_mct_bc_ready_ms(void)
{
	return READ_ONCE(mct_ready_ms);
}

void exynos9810_mct_bc_get_stats(struct exynos9810_mct_bc_stats *s)
{
	s->irqs = atomic64_read(&mct_irqs);
	s->programmed = atomic64_read(&mct_programmed);
	s->etime = atomic64_read(&mct_etime);
	s->write_timeouts = atomic64_read(&mct_wtimeouts);
}

/* Interrupt count of the per-CPU arch timer (the tick device) on @cpu. */
static unsigned int mct_arch_timer_irqs(int cpu)
{
	struct irq_desc *desc;
	unsigned int irq, n = 0;

	for_each_irq_desc(irq, desc) {
		struct irqaction *act = desc->action;

		if (act && act->name && !strcmp(act->name, "arch_timer"))
			n += kstat_irqs_cpu(irq, cpu);
	}
	return n;
}

/*
 * Late report (early diag log records are lost): probe results, broadcast
 * state, FRC drift against the arch counter since probe, and the tick
 * interrupt rate of every CPU. With NO_HZ_IDLE idle CPUs take far fewer
 * than HZ ticks per second; with HZ_PERIODIC every CPU takes HZ.
 */
static void mct_report(struct work_struct *work)
{
	static unsigned int t0[NR_CPUS], t1[NR_CPUS], b0[NR_CPUS], b1[NR_CPUS];
	static u64 l0[NR_CPUS], l1[NR_CPUS];
	struct exynos9810_mct_bc_stats s;
	unsigned long flags;
	u64 m0, m1, a0, a1;
	unsigned int cpu;
	s64 ppm = 0;

	pr_info("report: ready=%d why=%s irq_stuck=%d rate=%u Hz irq=%d tcon_at_probe=%#x frc_started=%d rate_ppm=%lld test_fired=%u/%u test_late_max_us=%llu early=%d bc_state=%s\n",
		mct_ready, mct_why ? mct_why : "ok", mct_irq_stuck, mct_rate,
		mct_irq,
		mct_probe.tcon_at_probe, mct_probe.frc_started,
		mct_probe.rate_ppm, mct_probe.fired, MCT_TEST_ROUNDS,
		mct_rate ? mct_cycles_to_us(mct_probe.late_max_cycles) : 0,
		mct_probe.early,
		!mct_ready ? "unregistered" :
		clockevent_state_detached(&mct_bc_evt) ? "detached" :
		clockevent_state_oneshot(&mct_bc_evt) ? "oneshot" :
		clockevent_state_shutdown(&mct_bc_evt) ? "shutdown" : "other");

	if (mct_base && mct_rate) {
		local_irq_save(flags);
		m0 = mct_read_count();
		a0 = arch_timer_read_counter();
		local_irq_restore(flags);
		msleep(1000);
		local_irq_save(flags);
		m1 = mct_read_count();
		a1 = arch_timer_read_counter();
		local_irq_restore(flags);
		if (a1 != a0)
			ppm = div64_s64(((s64)(m1 - m0) - (s64)(a1 - a0)) * 1000000,
					(s64)(a1 - a0));
		pr_info("report: frc_drift_ppm_1s=%lld frc_delta=%llu arch_delta=%llu\n",
			ppm, m1 - m0, a1 - a0);
	}

	for_each_online_cpu(cpu) {
		t0[cpu] = mct_arch_timer_irqs(cpu);
		l0[cpu] = READ_ONCE(per_cpu_ptr(&mct_local, cpu)->irqs);
		b0[cpu] = mct_bc_ipis(cpu);
	}
	msleep(MCT_TICK_WINDOW_MS);
	for_each_online_cpu(cpu) {
		t1[cpu] = mct_arch_timer_irqs(cpu);
		l1[cpu] = READ_ONCE(per_cpu_ptr(&mct_local, cpu)->irqs);
		b1[cpu] = mct_bc_ipis(cpu);
	}
	for_each_online_cpu(cpu)
		pr_info("report: cpu%u tick=%s arch_timer irqs/s=%u mct_tick irqs/s=%llu timer_bc_ipis/s=%u (HZ=%u)\n",
			cpu, mct_tick_name(cpu),
			(t1[cpu] - t0[cpu]) * 1000 / MCT_TICK_WINDOW_MS,
			div_u64((l1[cpu] - l0[cpu]) * 1000, MCT_TICK_WINDOW_MS),
			(b1[cpu] - b0[cpu]) * 1000 / MCT_TICK_WINDOW_MS, HZ);
	pr_info("report: local timers %s (%s), self-check passed on %*pbl\n",
		READ_ONCE(mct_local_on) ? "on" : "off",
		mct_local_why ? mct_local_why : "ok",
		cpumask_pr_args(&mct_local_tested));

	exynos9810_mct_bc_get_stats(&s);
	pr_info("report: done irqs=%llu programmed=%llu etime=%llu write_timeouts=%llu\n",
		s.irqs, s.programmed, s.etime, s.write_timeouts);
}

static const char *mct_bc_setup(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk *clk;
	const char *why;
	int ret;

	mct_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mct_base)) {
		mct_base = NULL;
		return "cannot map registers";
	}

	/*
	 * 4.9 enabled the MCT gate too. It is a HWACG (Q-channel) gate, so
	 * register access works without it; holding it only keeps
	 * clk_disable_unused() away. Probe runs before that, with the
	 * bootloader's clock state.
	 */
	/*
	 * Enable once and never disable: a devm-managed enable would turn the
	 * gate off again if the device were ever unbound, and releasing a
	 * PERIS vclk can switch the PERIS user mux (GIC/MCT bus) to OSCCLK.
	 */
	clk = clk_get_optional(dev, "mct");
	if (IS_ERR(clk))
		pr_warn("mct clock lookup failed (%ld), relying on HWACG\n",
			PTR_ERR(clk));
	else if (clk && clk_prepare_enable(clk))
		pr_warn("mct clock enable failed, relying on HWACG\n");

	mct_rate = arch_timer_get_rate();
	if (!mct_rate)
		return "arch timer rate unknown";

	why = mct_check_frc();
	if (why)
		return why;

	/* Leave the comparator in a known state before taking the IRQ. */
	if (!mct_comp0_stop())
		return "G_TCON write did not complete";
	writel_relaxed(1, mct_base + MCT_G_INT_CSTAT);

	mct_irq = platform_get_irq(pdev, 0);
	if (mct_irq < 0)
		return "no interrupt";
	mct_bc_evt.irq = mct_irq;
	mct_bc_evt.cpumask = cpumask_of(0);
	ret = devm_request_irq(dev, mct_irq, mct_bc_isr,
			       IRQF_TIMER | IRQF_NOBALANCING, "mct-bc",
			       &mct_bc_evt);
	if (ret)
		return "request_irq failed";
	if (irq_force_affinity(mct_irq, cpumask_of(0)))
		return "cannot route the interrupt to CPU0";

	why = mct_check_irq();
	if (why)
		return why;

	clockevents_config_and_register(&mct_bc_evt, mct_rate, MCT_MIN_DELTA,
					MCT_MAX_DELTA);
	/* Detached means the tick code kept another broadcast device. */
	if (clockevent_state_detached(&mct_bc_evt))
		return "registered but not installed as the broadcast device";
	return NULL;
}

static int mct_bc_probe(struct platform_device *pdev)
{
	bool want_local = of_property_read_bool(pdev->dev.of_node,
						"samsung,mct-local-timers");

	mct_why = mct_bc_setup(pdev);
	if (mct_why) {
		pr_warn("not used, hrtimer broadcast stays: %s\n", mct_why);
		if (want_local) {
			mct_local_why = "the MCT failed its broadcast self-check";
			pr_warn("local timers not used: %s\n", mct_local_why);
		}
	} else {
		/*
		 * Local timers first: once mct_ready is set, cpuidle may arm,
		 * and exynos9810_mct_local_active() must be final by then.
		 */
		if (want_local) {
			mct_find_ipi_timer();
			mct_local_why = mct_local_setup(pdev);
			if (mct_local_why)
				pr_warn("local timers not used, every CPU keeps the arch timer + broadcast: %s\n",
					mct_local_why);
			else
				pr_info("local timers: self-check passed on %*pbl (start->irq %llu..%llu MCT clocks, expected %llu for %u us at %u Hz), tick device now on %*pbl; rating %u, the broadcast stays for the others; IPI_TIMER irq %d\n",
					cpumask_pr_args(&mct_local_tested),
					mct_l_test_min, mct_l_test_max,
					div_u64((u64)mct_local_rate * MCT_L_TEST_US,
						USEC_PER_SEC) * (MCT_L_TCNTB_VAL + 1),
					MCT_L_TEST_US, mct_local_rate,
					cpumask_pr_args(mct_local_active_mask()),
					MCT_L_RATING, mct_ipi_timer_irq);
			if (!mct_local_why) {
				INIT_DELAYED_WORK(&mct_local_work, mct_local_window);
				queue_delayed_work(system_long_wq, &mct_local_work,
						   mct_jiffies_until(MCT_L_WINDOW_START_MS));
			}
		}
		WRITE_ONCE(mct_ready_ms, ktime_to_ms(ktime_get_boottime()));
		WRITE_ONCE(mct_ready, true);
		pr_info("tick broadcast device registered: rate=%u Hz rate_ppm=%lld test_late_max_us=%llu irq=%d\n",
			mct_rate, mct_probe.rate_ppm,
			mct_cycles_to_us(mct_probe.late_max_cycles), mct_irq);
	}

	if (of_property_read_bool(pdev->dev.of_node, "samsung,mct-bc-report")) {
		unsigned long now = jiffies - INITIAL_JIFFIES;
		unsigned long at = msecs_to_jiffies(MCT_REPORT_MS);

		INIT_DELAYED_WORK(&mct_report_work, mct_report);
		queue_delayed_work(system_long_wq, &mct_report_work,
				   at > now ? at - now : 0);
	}

	/*
	 * Failures are reported, never fatal. Once registered, the clockevent
	 * and its IRQ belong to the tick code for good.
	 */
	return 0;
}

static const struct of_device_id mct_bc_of_match[] = {
	{ .compatible = "samsung,exynos9810-mct-broadcast" },
	{ },
};

static struct platform_driver mct_bc_driver = {
	.probe = mct_bc_probe,
	.driver = {
		.name = "exynos9810-mct-bc",
		.of_match_table = mct_bc_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(mct_bc_driver);
