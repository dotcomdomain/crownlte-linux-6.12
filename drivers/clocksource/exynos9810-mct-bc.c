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
 * Only G_COMP0 and its interrupt (MCT G0, GIC SPI 466 on 4.9) are used. The
 * FRC is started if it is not counting, as 4.9 exynos4_mct_frc_start() did
 * at every boot. The local timers are left alone.
 */

#define pr_fmt(fmt) "exynos9810-mct-bc: " fmt

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/completion.h>
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

static void __iomem *mct_base;
static int mct_irq = -1;
static u32 mct_rate;
static bool mct_ready;
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

bool exynos9810_mct_bc_ready(void)
{
	return READ_ONCE(mct_ready);
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
	static unsigned int t0[NR_CPUS], t1[NR_CPUS];
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

	for_each_online_cpu(cpu)
		t0[cpu] = mct_arch_timer_irqs(cpu);
	msleep(MCT_TICK_WINDOW_MS);
	for_each_online_cpu(cpu)
		t1[cpu] = mct_arch_timer_irqs(cpu);
	for_each_online_cpu(cpu)
		pr_info("report: cpu%u arch_timer irqs/s=%u (HZ=%u)\n", cpu,
			(t1[cpu] - t0[cpu]) * 1000 / MCT_TICK_WINDOW_MS, HZ);

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
	clk = devm_clk_get_optional_enabled(dev, "mct");
	if (IS_ERR(clk))
		pr_warn("mct clock not enabled (%ld), relying on HWACG\n",
			PTR_ERR(clk));

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
	mct_why = mct_bc_setup(pdev);
	if (mct_why) {
		pr_warn("not used, hrtimer broadcast stays: %s\n", mct_why);
	} else {
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
