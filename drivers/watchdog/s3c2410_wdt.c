// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2004 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C2410 Watchdog Timer Support
 *
 * Based on, softdog.c by Alan Cox,
 *     (c) Copyright 1996 Alan Cox <alan@lxorguk.ukuu.org.uk>
 */

#include <linux/bits.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/watchdog.h>
#include <linux/reboot.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/math64.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>
#include <linux/workqueue.h>

#include <soc/samsung/crownlte-wdt.h>

#define S3C2410_WTCON		0x00
#define S3C2410_WTDAT		0x04
#define S3C2410_WTCNT		0x08
#define S3C2410_WTCLRINT	0x0c

#define S3C2410_WTCNT_MAXCNT	0xffff

#define S3C2410_WTCON_RSTEN		BIT(0)
#define S3C2410_WTCON_INTEN		BIT(2)
#define S3C2410_WTCON_ENABLE		BIT(5)
#define S3C2410_WTCON_DBGACK_MASK	BIT(16)

#define S3C2410_WTCON_DIV16	(0 << 3)
#define S3C2410_WTCON_DIV32	(1 << 3)
#define S3C2410_WTCON_DIV64	(2 << 3)
#define S3C2410_WTCON_DIV128	(3 << 3)

#define S3C2410_WTCON_MAXDIV	0x80

#define S3C2410_WTCON_PRESCALE(x)	((x) << 8)
#define S3C2410_WTCON_PRESCALE_MASK	(0xff << 8)
#define S3C2410_WTCON_PRESCALE_MAX	0xff

#define S3C2410_WATCHDOG_ATBOOT		(0)
#define S3C2410_WATCHDOG_DEFAULT_TIME	(15)

#define EXYNOS5_RST_STAT_REG_OFFSET		0x0404
#define EXYNOS5_WDT_DISABLE_REG_OFFSET		0x0408
#define EXYNOS5_WDT_MASK_RESET_REG_OFFSET	0x040c
#define EXYNOS850_CLUSTER0_NONCPU_OUT		0x1220
#define EXYNOS850_CLUSTER0_NONCPU_INT_EN	0x1244
#define EXYNOS850_CLUSTER1_NONCPU_OUT		0x1620
#define EXYNOS850_CLUSTER1_NONCPU_INT_EN	0x1644
#define EXYNOSAUTOV9_CLUSTER1_NONCPU_OUT	0x1520
#define EXYNOSAUTOV9_CLUSTER1_NONCPU_INT_EN	0x1544

#define EXYNOS850_CLUSTER0_WDTRESET_BIT		24
#define EXYNOS850_CLUSTER1_WDTRESET_BIT		23
#define EXYNOSAUTOV9_CLUSTER0_WDTRESET_BIT	25
#define EXYNOSAUTOV9_CLUSTER1_WDTRESET_BIT	24

#define GS_CLUSTER0_NONCPU_OUT			0x1220
#define GS_CLUSTER1_NONCPU_OUT			0x1420
#define GS_CLUSTER0_NONCPU_INT_EN		0x1244
#define GS_CLUSTER1_NONCPU_INT_EN		0x1444
#define GS_CLUSTER2_NONCPU_INT_EN		0x1644
#define GS_RST_STAT_REG_OFFSET			0x3B44

/*
 * Exynos9810: CL0 watchdog at 0x10050000 uses the Exynos5-style PMU
 * DISABLE/MASK/RST_STAT registers at bit 24 (CLUSTER0_WDTRESET), as in the
 * vendor 4.9 kernel ("samsung,exynos8-wdt" variant on watchdog_cl0).
 */
#define EXYNOS9810_CLUSTER0_WDTRESET_BIT	24

/* Exynos9810 PMU registers read (never written) by the SBOOT reset guard */
#define EXYNOS9810_CPU_RESET_DISABLE_FROM_WDTRESET	0x0414
#define EXYNOS9810_RESET_SEQUENCER_CONFIGURATION	0x0500
#define EXYNOS9810_INFORM2				0x0808
#define EXYNOS9810_INFORM3				0x080c
#define EXYNOS9810_BIG_CPU0_RESET			0x3c0c
#define EXYNOS9810_LITTLE_CPU0_ETC_RESET		0x3c4c
#define EXYNOS9810_BIG_NONCPU_ETC_RESET			0x3c8c
#define EXYNOS9810_PMU_CPU_STRIDE			0x10

/* DFD dump-GPR enable and the reset-disable bits it arms (4.9 exynos-reboot) */
#define EXYNOS9810_DFD_EDPCSR_DUMP_EN		BIT(0)
#define EXYNOS9810_DFD_RESET_DISABLE		(BIT(9) | BIT(10) | BIT(11) | \
						 (0xfU << 24))
#define EXYNOS9810_WDT_RESET_PEND		0x3	/* PEND_BIG | PEND_LITTLE */
#define EXYNOS9810_BIG_CPU_RESET_DISABLE	0x3300
#define EXYNOS9810_LITTLE_CPU_RESET_DISABLE	0x1100
#define EXYNOS9810_BIG_NONCPU_RESET_DISABLE	0x84040000

/* SEC sboot normal-boot selection (4.9 sec_reboot.c, crownlte boot shim) */
#define SEC_POWER_RESET				0x12345678
#define SEC_RESET_REASON_UNKNOWN		0x12345670

/* Counter-rate check: DIV128 with prescaler 16, i.e. src_clk / 2048 */
#define S3C2410_RATE_CHECK_PRESCALE		16
#define S3C2410_RATE_CHECK_SLEEP_MS		25

/* Opt-in expiry selftest: short timeout once petting stops */
#define S3C2410_SELFTEST_EXPIRE_S		5
#define S3C2410_SELFTEST_MIN_MS			10000
#define S3C2410_SELFTEST_MAX_MS			114000

/**
 * DOC: Quirk flags for different Samsung watchdog IP-cores
 *
 * This driver supports multiple Samsung SoCs, each of which might have
 * different set of registers and features supported. As watchdog block
 * sometimes requires modifying PMU registers for proper functioning, register
 * differences in both watchdog and PMU IP-cores should be accounted for. Quirk
 * flags described below serve the purpose of telling the driver about mentioned
 * SoC traits, and can be specified in driver data for each particular supported
 * device.
 *
 * %QUIRK_HAS_WTCLRINT_REG: Watchdog block has WTCLRINT register. It's used to
 * clear the interrupt once the interrupt service routine is complete. It's
 * write-only, writing any values to this register clears the interrupt, but
 * reading is not permitted.
 *
 * %QUIRK_HAS_PMU_MASK_RESET: PMU block has the register for disabling/enabling
 * WDT reset request. On old SoCs it's usually called MASK_WDT_RESET_REQUEST,
 * new SoCs have CLUSTERx_NONCPU_INT_EN register, which 'mask_bit' value is
 * inverted compared to the former one.
 *
 * %QUIRK_HAS_PMU_RST_STAT: PMU block has RST_STAT (reset status) register,
 * which contains bits indicating the reason for most recent CPU reset. If
 * present, driver will use this register to check if previous reboot was due to
 * watchdog timer reset.
 *
 * %QUIRK_HAS_PMU_AUTO_DISABLE: PMU block has AUTOMATIC_WDT_RESET_DISABLE
 * register. If 'mask_bit' bit is set, PMU will disable WDT reset when
 * corresponding processor is in reset state.
 *
 * %QUIRK_HAS_PMU_CNT_EN: PMU block has some register (e.g. CLUSTERx_NONCPU_OUT)
 * with "watchdog counter enable" bit. That bit should be set to make watchdog
 * counter running.
 *
 * %QUIRK_HAS_DBGACK_BIT: WTCON register has DBGACK_MASK bit. Setting the
 * DBGACK_MASK bit disables the watchdog outputs when the SoC is in debug mode.
 * Debug mode is determined by the DBGACK CPU signal.
 *
 * %QUIRK_SBOOT_RESET_GUARD: Samsung SBOOT handles the watchdog reset. Before
 * anything is armed, check (read-only) that a watchdog reset would be a plain
 * full reset into the normal boot, not into Upload (ramdump) mode or a
 * recovery boot, and measure the counter rate. On any mismatch the device is
 * not registered and nothing is armed. Also verifies the PMU unmask readback.
 *
 * %QUIRK_KEEP_RUNNING_ON_SHUTDOWN: Leave a running watchdog armed (freshly
 * pinged) across shutdown/restart, so a restart path that hangs or refuses
 * still ends in a watchdog reset.
 */
#define QUIRK_HAS_WTCLRINT_REG			BIT(0)
#define QUIRK_HAS_PMU_MASK_RESET		BIT(1)
#define QUIRK_HAS_PMU_RST_STAT			BIT(2)
#define QUIRK_HAS_PMU_AUTO_DISABLE		BIT(3)
#define QUIRK_HAS_PMU_CNT_EN			BIT(4)
#define QUIRK_HAS_DBGACK_BIT			BIT(5)
#define QUIRK_SBOOT_RESET_GUARD			BIT(6)
#define QUIRK_KEEP_RUNNING_ON_SHUTDOWN		BIT(7)

/* These quirks require that we have a PMU register map */
#define QUIRKS_HAVE_PMUREG \
	(QUIRK_HAS_PMU_MASK_RESET | QUIRK_HAS_PMU_RST_STAT | \
	 QUIRK_HAS_PMU_AUTO_DISABLE | QUIRK_HAS_PMU_CNT_EN | \
	 QUIRK_SBOOT_RESET_GUARD)

static bool nowayout	= WATCHDOG_NOWAYOUT;
static int tmr_margin;
static int tmr_atboot	= S3C2410_WATCHDOG_ATBOOT;
static int soft_noboot;

module_param(tmr_margin,  int, 0);
module_param(tmr_atboot,  int, 0);
module_param(nowayout,   bool, 0);
module_param(soft_noboot, int, 0);

MODULE_PARM_DESC(tmr_margin, "Watchdog tmr_margin in seconds. (default="
		__MODULE_STRING(S3C2410_WATCHDOG_DEFAULT_TIME) ")");
MODULE_PARM_DESC(tmr_atboot,
		"Watchdog is started at boot time if set to 1, default="
			__MODULE_STRING(S3C2410_WATCHDOG_ATBOOT));
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
			__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");
MODULE_PARM_DESC(soft_noboot, "Watchdog action, set to 1 to ignore reboots, 0 to reboot (default 0)");

/**
 * struct s3c2410_wdt_variant - Per-variant config data
 *
 * @disable_reg: Offset in pmureg for the register that disables the watchdog
 * timer reset functionality.
 * @mask_reset_reg: Offset in pmureg for the register that masks the watchdog
 * timer reset functionality.
 * @mask_reset_inv: If set, mask_reset_reg value will have inverted meaning.
 * @mask_bit: Bit number for the watchdog timer in the disable register and the
 * mask reset register.
 * @rst_stat_reg: Offset in pmureg for the register that has the reset status.
 * @rst_stat_bit: Bit number in the rst_stat register indicating a watchdog
 * reset.
 * @cnt_en_reg: Offset in pmureg for the register that enables WDT counter.
 * @cnt_en_bit: Bit number for "watchdog counter enable" in cnt_en register.
 * @quirks: A bitfield of quirks.
 */

struct s3c2410_wdt_variant {
	int disable_reg;
	int mask_reset_reg;
	bool mask_reset_inv;
	int mask_bit;
	int rst_stat_reg;
	int rst_stat_bit;
	int cnt_en_reg;
	int cnt_en_bit;
	u32 quirks;
};

struct s3c2410_wdt {
	struct device		*dev;
	struct clk		*bus_clk; /* for register interface (PCLK) */
	struct clk		*src_clk; /* for WDT counter */
	void __iomem		*reg_base;
	unsigned int		count;
	spinlock_t		lock;
	unsigned long		wtcon_save;
	unsigned long		wtdat_save;
	struct watchdog_device	wdt_device;
	struct notifier_block	freq_transition;
	const struct s3c2410_wdt_variant *drv_data;
	struct regmap *pmureg;
	/* Opt-in expiry selftest (QUIRK_SBOOT_RESET_GUARD variants only) */
	struct delayed_work	selftest_work;
	bool			selftest_fired;	/* the work already ran once */
	/* Restart: short timeout, last reload, then the final stop */
	struct notifier_block	reboot_nb;
	/*
	 * DT samsung,keep-armed-in-suspend (guarded variants only): system
	 * suspend leaves the watchdog running with a longer timeout instead
	 * of stopping it, so the noirq/syscore/firmware part of a suspend that
	 * hangs still ends in a reset. resume_timeout is the run-time timeout
	 * restored on resume.
	 */
	bool			keep_armed_suspend;
	bool			suspended_armed;
	unsigned int		resume_timeout;
	/*
	 * Counter switched off by a plain (not kept-armed) system suspend,
	 * under wdt->lock; cleared by resume under the same lock.
	 */
	bool			pm_stopped;
	/*
	 * THE final-stop state, the only one: set under wdt->lock by
	 * __s3c2410wdt_final_stop() (s3c2410wdt_crownlte_stop_petting(), the
	 * restart notifier, the expiry selftest) or by s3c2410wdt_may_pet()
	 * once the deadline below has passed. Never cleared. Every path that
	 * writes WTCNT, slows the prescaler, stops the counter or masks the
	 * reset checks it under wdt->lock. stop_why: the first reason.
	 */
	bool			pet_stopped;
	const char		*stop_why;
	/*
	 * s3c2410wdt_crownlte_set_deadline(): CLOCK_BOOTTIME ns after which
	 * no path reloads the counter any more, whatever CPU asks (0: none).
	 * deadline_hit/deadline_logged: the ping path found it passed.
	 */
	u64			pet_deadline_ns;
	bool			deadline_hit;
	bool			deadline_logged;
};

/* Suspend-time timeout with samsung,keep-armed-in-suspend (capped at max) */
#define S3C2410_WDT_SUSPEND_TIMEOUT	60

/*
 * The one guarded (SBOOT reset guard) instance, for crownlte-wdt.h.
 * Published with smp_store_release() once probe has set it up; the
 * hooks read it with smp_load_acquire() and see a complete instance.
 */
static struct s3c2410_wdt *s3c2410wdt_crownlte;

static const struct s3c2410_wdt_variant drv_data_s3c2410 = {
	.quirks = 0
};

#ifdef CONFIG_OF
static const struct s3c2410_wdt_variant drv_data_s3c6410 = {
	.quirks = QUIRK_HAS_WTCLRINT_REG,
};

static const struct s3c2410_wdt_variant drv_data_exynos5250  = {
	.disable_reg = EXYNOS5_WDT_DISABLE_REG_OFFSET,
	.mask_reset_reg = EXYNOS5_WDT_MASK_RESET_REG_OFFSET,
	.mask_bit = 20,
	.rst_stat_reg = EXYNOS5_RST_STAT_REG_OFFSET,
	.rst_stat_bit = 20,
	.quirks = QUIRK_HAS_WTCLRINT_REG | QUIRK_HAS_PMU_MASK_RESET | \
		  QUIRK_HAS_PMU_RST_STAT | QUIRK_HAS_PMU_AUTO_DISABLE,
};

static const struct s3c2410_wdt_variant drv_data_exynos5420 = {
	.disable_reg = EXYNOS5_WDT_DISABLE_REG_OFFSET,
	.mask_reset_reg = EXYNOS5_WDT_MASK_RESET_REG_OFFSET,
	.mask_bit = 0,
	.rst_stat_reg = EXYNOS5_RST_STAT_REG_OFFSET,
	.rst_stat_bit = 9,
	.quirks = QUIRK_HAS_WTCLRINT_REG | QUIRK_HAS_PMU_MASK_RESET | \
		  QUIRK_HAS_PMU_RST_STAT | QUIRK_HAS_PMU_AUTO_DISABLE,
};

static const struct s3c2410_wdt_variant drv_data_exynos7 = {
	.disable_reg = EXYNOS5_WDT_DISABLE_REG_OFFSET,
	.mask_reset_reg = EXYNOS5_WDT_MASK_RESET_REG_OFFSET,
	.mask_bit = 23,
	.rst_stat_reg = EXYNOS5_RST_STAT_REG_OFFSET,
	.rst_stat_bit = 23,	/* A57 WDTRESET */
	.quirks = QUIRK_HAS_WTCLRINT_REG | QUIRK_HAS_PMU_MASK_RESET | \
		  QUIRK_HAS_PMU_RST_STAT | QUIRK_HAS_PMU_AUTO_DISABLE,
};

static const struct s3c2410_wdt_variant drv_data_exynos850_cl0 = {
	.mask_reset_reg = EXYNOS850_CLUSTER0_NONCPU_INT_EN,
	.mask_bit = 2,
	.mask_reset_inv = true,
	.rst_stat_reg = EXYNOS5_RST_STAT_REG_OFFSET,
	.rst_stat_bit = EXYNOS850_CLUSTER0_WDTRESET_BIT,
	.cnt_en_reg = EXYNOS850_CLUSTER0_NONCPU_OUT,
	.cnt_en_bit = 7,
	.quirks = QUIRK_HAS_WTCLRINT_REG | QUIRK_HAS_PMU_MASK_RESET | \
		  QUIRK_HAS_PMU_RST_STAT | QUIRK_HAS_PMU_CNT_EN,
};

static const struct s3c2410_wdt_variant drv_data_exynos850_cl1 = {
	.mask_reset_reg = EXYNOS850_CLUSTER1_NONCPU_INT_EN,
	.mask_bit = 2,
	.mask_reset_inv = true,
	.rst_stat_reg = EXYNOS5_RST_STAT_REG_OFFSET,
	.rst_stat_bit = EXYNOS850_CLUSTER1_WDTRESET_BIT,
	.cnt_en_reg = EXYNOS850_CLUSTER1_NONCPU_OUT,
	.cnt_en_bit = 7,
	.quirks = QUIRK_HAS_WTCLRINT_REG | QUIRK_HAS_PMU_MASK_RESET | \
		  QUIRK_HAS_PMU_RST_STAT | QUIRK_HAS_PMU_CNT_EN,
};

static const struct s3c2410_wdt_variant drv_data_exynosautov9_cl0 = {
	.mask_reset_reg = EXYNOS850_CLUSTER0_NONCPU_INT_EN,
	.mask_bit = 2,
	.mask_reset_inv = true,
	.rst_stat_reg = EXYNOS5_RST_STAT_REG_OFFSET,
	.rst_stat_bit = EXYNOSAUTOV9_CLUSTER0_WDTRESET_BIT,
	.cnt_en_reg = EXYNOS850_CLUSTER0_NONCPU_OUT,
	.cnt_en_bit = 7,
	.quirks = QUIRK_HAS_WTCLRINT_REG | QUIRK_HAS_PMU_MASK_RESET |
		  QUIRK_HAS_PMU_RST_STAT | QUIRK_HAS_PMU_CNT_EN,
};

static const struct s3c2410_wdt_variant drv_data_exynosautov9_cl1 = {
	.mask_reset_reg = EXYNOSAUTOV9_CLUSTER1_NONCPU_INT_EN,
	.mask_bit = 2,
	.mask_reset_inv = true,
	.rst_stat_reg = EXYNOS5_RST_STAT_REG_OFFSET,
	.rst_stat_bit = EXYNOSAUTOV9_CLUSTER1_WDTRESET_BIT,
	.cnt_en_reg = EXYNOSAUTOV9_CLUSTER1_NONCPU_OUT,
	.cnt_en_bit = 7,
	.quirks = QUIRK_HAS_WTCLRINT_REG | QUIRK_HAS_PMU_MASK_RESET |
		  QUIRK_HAS_PMU_RST_STAT | QUIRK_HAS_PMU_CNT_EN,
};

/*
 * Exynos9810 CL0 watchdog. PMU offsets and bit 24 match the vendor 4.9
 * crownlte kernel; RST_STAT bit 24 is CLUSTER0_WDTRESET in its reset-reason
 * table, the bit SBOOT tests for a watchdog reset.
 */
static const struct s3c2410_wdt_variant drv_data_exynos9810 = {
	.disable_reg = EXYNOS5_WDT_DISABLE_REG_OFFSET,
	.mask_reset_reg = EXYNOS5_WDT_MASK_RESET_REG_OFFSET,
	.mask_bit = EXYNOS9810_CLUSTER0_WDTRESET_BIT,
	.rst_stat_reg = EXYNOS5_RST_STAT_REG_OFFSET,
	.rst_stat_bit = EXYNOS9810_CLUSTER0_WDTRESET_BIT,
	.quirks = QUIRK_HAS_WTCLRINT_REG | QUIRK_HAS_PMU_MASK_RESET |
		  QUIRK_HAS_PMU_RST_STAT | QUIRK_HAS_PMU_AUTO_DISABLE |
		  QUIRK_SBOOT_RESET_GUARD | QUIRK_KEEP_RUNNING_ON_SHUTDOWN,
};

static const struct s3c2410_wdt_variant drv_data_gs101_cl0 = {
	.mask_reset_reg = GS_CLUSTER0_NONCPU_INT_EN,
	.mask_bit = 2,
	.mask_reset_inv = true,
	.rst_stat_reg = GS_RST_STAT_REG_OFFSET,
	.rst_stat_bit = 0,
	.cnt_en_reg = GS_CLUSTER0_NONCPU_OUT,
	.cnt_en_bit = 8,
	.quirks = QUIRK_HAS_PMU_RST_STAT | QUIRK_HAS_PMU_MASK_RESET |
		  QUIRK_HAS_PMU_CNT_EN | QUIRK_HAS_WTCLRINT_REG |
		  QUIRK_HAS_DBGACK_BIT,
};

static const struct s3c2410_wdt_variant drv_data_gs101_cl1 = {
	.mask_reset_reg = GS_CLUSTER1_NONCPU_INT_EN,
	.mask_bit = 2,
	.mask_reset_inv = true,
	.rst_stat_reg = GS_RST_STAT_REG_OFFSET,
	.rst_stat_bit = 1,
	.cnt_en_reg = GS_CLUSTER1_NONCPU_OUT,
	.cnt_en_bit = 7,
	.quirks = QUIRK_HAS_PMU_RST_STAT | QUIRK_HAS_PMU_MASK_RESET |
		  QUIRK_HAS_PMU_CNT_EN | QUIRK_HAS_WTCLRINT_REG |
		  QUIRK_HAS_DBGACK_BIT,
};

static const struct of_device_id s3c2410_wdt_match[] = {
	{ .compatible = "google,gs101-wdt",
	  .data = &drv_data_gs101_cl0 },
	{ .compatible = "samsung,s3c2410-wdt",
	  .data = &drv_data_s3c2410 },
	{ .compatible = "samsung,s3c6410-wdt",
	  .data = &drv_data_s3c6410 },
	{ .compatible = "samsung,exynos5250-wdt",
	  .data = &drv_data_exynos5250 },
	{ .compatible = "samsung,exynos5420-wdt",
	  .data = &drv_data_exynos5420 },
	{ .compatible = "samsung,exynos7-wdt",
	  .data = &drv_data_exynos7 },
	{ .compatible = "samsung,exynos850-wdt",
	  .data = &drv_data_exynos850_cl0 },
	{ .compatible = "samsung,exynosautov9-wdt",
	  .data = &drv_data_exynosautov9_cl0 },
	{ .compatible = "samsung,exynos9810-wdt",
	  .data = &drv_data_exynos9810 },
	{},
};
MODULE_DEVICE_TABLE(of, s3c2410_wdt_match);
#endif

static const struct platform_device_id s3c2410_wdt_ids[] = {
	{
		.name = "s3c2410-wdt",
		.driver_data = (unsigned long)&drv_data_s3c2410,
	},
	{}
};
MODULE_DEVICE_TABLE(platform, s3c2410_wdt_ids);

/* functions */

static inline unsigned long s3c2410wdt_get_freq(struct s3c2410_wdt *wdt)
{
	return clk_get_rate(wdt->src_clk ? wdt->src_clk : wdt->bus_clk);
}

static inline unsigned int s3c2410wdt_max_timeout(struct s3c2410_wdt *wdt)
{
	const unsigned long freq = s3c2410wdt_get_freq(wdt);
	const u64 n_max = (u64)(S3C2410_WTCON_PRESCALE_MAX + 1) *
			S3C2410_WTCON_MAXDIV * S3C2410_WTCNT_MAXCNT;
	u64 t_max = div64_ul(n_max, freq);

	if (t_max > UINT_MAX)
		t_max = UINT_MAX;

	return t_max;
}

static int s3c2410wdt_disable_wdt_reset(struct s3c2410_wdt *wdt, bool mask)
{
	const u32 mask_val = BIT(wdt->drv_data->mask_bit);
	const u32 val = mask ? mask_val : 0;
	int ret;

	ret = regmap_update_bits(wdt->pmureg, wdt->drv_data->disable_reg,
				 mask_val, val);
	if (ret < 0)
		dev_err(wdt->dev, "failed to update reg(%d)\n", ret);

	return ret;
}

static int s3c2410wdt_mask_wdt_reset(struct s3c2410_wdt *wdt, bool mask)
{
	const u32 mask_val = BIT(wdt->drv_data->mask_bit);
	const bool val_inv = wdt->drv_data->mask_reset_inv;
	const u32 val = (mask ^ val_inv) ? mask_val : 0;
	int ret;

	ret = regmap_update_bits(wdt->pmureg, wdt->drv_data->mask_reset_reg,
				 mask_val, val);
	if (ret < 0)
		dev_err(wdt->dev, "failed to update reg(%d)\n", ret);

	return ret;
}

static int s3c2410wdt_enable_counter(struct s3c2410_wdt *wdt, bool en)
{
	const u32 mask_val = BIT(wdt->drv_data->cnt_en_bit);
	const u32 val = en ? mask_val : 0;
	int ret;

	ret = regmap_update_bits(wdt->pmureg, wdt->drv_data->cnt_en_reg,
				 mask_val, val);
	if (ret < 0)
		dev_err(wdt->dev, "failed to update reg(%d)\n", ret);

	return ret;
}

static int s3c2410wdt_enable(struct s3c2410_wdt *wdt, bool en)
{
	int ret;

	if (wdt->drv_data->quirks & QUIRK_HAS_PMU_AUTO_DISABLE) {
		ret = s3c2410wdt_disable_wdt_reset(wdt, !en);
		if (ret < 0)
			return ret;
	}

	if (wdt->drv_data->quirks & QUIRK_HAS_PMU_MASK_RESET) {
		ret = s3c2410wdt_mask_wdt_reset(wdt, !en);
		if (ret < 0)
			return ret;
	}

	if (wdt->drv_data->quirks & QUIRK_HAS_PMU_CNT_EN) {
		ret = s3c2410wdt_enable_counter(wdt, en);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/* Disable watchdog outputs if CPU is in debug mode */
static void s3c2410wdt_mask_dbgack(struct s3c2410_wdt *wdt)
{
	unsigned long wtcon;

	if (!(wdt->drv_data->quirks & QUIRK_HAS_DBGACK_BIT))
		return;

	wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	wtcon |= S3C2410_WTCON_DBGACK_MASK;
	writel(wtcon, wdt->reg_base + S3C2410_WTCON);
}

/*
 * May the counter be reloaded now? Called with wdt->lock held, from every
 * path that would reload it. False once a restart, the expiry selftest or
 * s3c2410wdt_crownlte_stop_petting() stopped petting, and once the
 * crownlte deadline has passed: that check runs here, in the ping path, so
 * it holds whichever CPU pets and whether or not a deadline timer fires.
 * ktime_get_boot_fast_ns() is usable in any context; while timekeeping is
 * suspended it stands still, which can only delay the stop, never move it
 * earlier.
 */
static bool s3c2410wdt_may_pet(struct s3c2410_wdt *wdt)
{
	lockdep_assert_held(&wdt->lock);
	if (wdt->pet_stopped)
		return false;
	if (wdt->pet_deadline_ns &&
	    ktime_get_boot_fast_ns() >= wdt->pet_deadline_ns) {
		wdt->pet_stopped = true;
		wdt->stop_why = "deadline";
		wdt->deadline_hit = true;
		return false;
	}
	return true;
}

/*
 * The final stop, with wdt->lock held: one last reload of the running
 * counter, but only while s3c2410wdt_may_pet() still allows one (never
 * after the deadline passed or after an earlier final stop), then the
 * shared stopped state is latched for good. Idempotent. True if this call
 * made the last reload.
 */
static bool __s3c2410wdt_final_stop(struct s3c2410_wdt *wdt, const char *why)
{
	bool reload;

	lockdep_assert_held(&wdt->lock);
	reload = s3c2410wdt_may_pet(wdt) &&
		 (readl(wdt->reg_base + S3C2410_WTCON) & S3C2410_WTCON_ENABLE);
	if (reload)
		writel(wdt->count, wdt->reg_base + S3C2410_WTCNT);
	if (!wdt->pet_stopped) {
		wdt->pet_stopped = true;
		wdt->stop_why = why;
	}
	return reload;
}

/* Logs the deadline stop once, outside wdt->lock. */
static void s3c2410wdt_deadline_report(struct s3c2410_wdt *wdt)
{
	unsigned long flags;
	bool report;

	spin_lock_irqsave(&wdt->lock, flags);
	report = wdt->deadline_hit && !wdt->deadline_logged;
	if (report)
		wdt->deadline_logged = true;
	spin_unlock_irqrestore(&wdt->lock, flags);
	if (report)
		pr_emerg("s3c2410-wdt: deadline %llu ms passed: counter no longer reloaded, SoC reset in at most %u s (WTCNT %#x)\n",
			 div_u64(wdt->pet_deadline_ns, NSEC_PER_MSEC),
			 wdt->wdt_device.timeout,
			 readl(wdt->reg_base + S3C2410_WTCNT));
}

/* Reload the counter if allowed; true if it was reloaded. */
static bool __s3c2410wdt_keepalive(struct s3c2410_wdt *wdt)
{
	unsigned long flags;
	bool pet;

	spin_lock_irqsave(&wdt->lock, flags);
	/* The opt-in selftest deliberately stops petting to prove expiry. */
	pet = s3c2410wdt_may_pet(wdt);
	if (pet)
		writel(wdt->count, wdt->reg_base + S3C2410_WTCNT);
	spin_unlock_irqrestore(&wdt->lock, flags);
	if (!pet)
		s3c2410wdt_deadline_report(wdt);
	return pet;
}

static int s3c2410wdt_keepalive(struct watchdog_device *wdd)
{
	__s3c2410wdt_keepalive(watchdog_get_drvdata(wdd));
	return 0;
}

static void __s3c2410wdt_stop(struct s3c2410_wdt *wdt)
{
	unsigned long wtcon;

	wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	wtcon &= ~(S3C2410_WTCON_ENABLE | S3C2410_WTCON_RSTEN);
	writel(wtcon, wdt->reg_base + S3C2410_WTCON);
}

static int s3c2410wdt_stop(struct watchdog_device *wdd)
{
	struct s3c2410_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;

	spin_lock_irqsave(&wdt->lock, flags);
	__s3c2410wdt_stop(wdt);
	spin_unlock_irqrestore(&wdt->lock, flags);

	return 0;
}

/* Petting stopped for good (or the deadline passed): true. */
static bool s3c2410wdt_pet_refused(struct s3c2410_wdt *wdt)
{
	unsigned long flags;
	bool refused;

	spin_lock_irqsave(&wdt->lock, flags);
	refused = !s3c2410wdt_may_pet(wdt);
	spin_unlock_irqrestore(&wdt->lock, flags);
	if (refused)
		s3c2410wdt_deadline_report(wdt);
	return refused;
}

/* A timeout turned into hardware values; computed without wdt->lock. */
struct s3c2410wdt_hb {
	unsigned int count;
	unsigned int divisor;
	unsigned int timeout;
};

/* clk_get_rate() may sleep: never called with wdt->lock held. */
static int s3c2410wdt_calc_heartbeat(struct s3c2410_wdt *wdt,
				     unsigned int timeout,
				     struct s3c2410wdt_hb *hb)
{
	unsigned long freq = s3c2410wdt_get_freq(wdt);
	unsigned int count;
	unsigned int divisor = 1;

	if (timeout < 1)
		return -EINVAL;

	freq = DIV_ROUND_UP(freq, 128);
	count = timeout * freq;

	dev_dbg(wdt->dev, "Heartbeat: count=%d, timeout=%d, freq=%lu\n",
		count, timeout, freq);

	/* if the count is bigger than the watchdog register,
	   then work out what we need to do (and if) we can
	   actually make this value
	*/

	if (count >= 0x10000) {
		divisor = DIV_ROUND_UP(count, 0xffff);

		if (divisor > 0x100) {
			dev_err(wdt->dev, "timeout %d too big\n", timeout);
			return -EINVAL;
		}
	}

	dev_dbg(wdt->dev, "Heartbeat: timeout=%d, divisor=%d, count=%d (%08x)\n",
		timeout, divisor, count, DIV_ROUND_UP(count, divisor));

	hb->count = DIV_ROUND_UP(count, divisor);
	hb->divisor = divisor;
	hb->timeout = (hb->count * divisor) / freq;
	return 0;
}

/*
 * Program WTDAT and the prescaler, with wdt->lock held, so a prescaler
 * change never interleaves with a reload, a deadline installation or a
 * deadline check. The count already running keeps its value, so a larger
 * prescaler stretches the time left until the reset. Once the counter may
 * no longer be reloaded (deadline passed, petting stopped for good,
 * restart, selftest), only a change that does not slow it down is made:
 * -ETIME otherwise, nothing written. @pet: reload the counter with the new
 * count in the same critical section; refused (-ETIME, nothing written)
 * unless reloading is still allowed.
 */
static int __s3c2410wdt_apply_heartbeat(struct s3c2410_wdt *wdt,
					const struct s3c2410wdt_hb *hb,
					bool pet)
{
	unsigned long wtcon;
	unsigned int cur_div;

	lockdep_assert_held(&wdt->lock);
	wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	cur_div = ((wtcon & S3C2410_WTCON_PRESCALE_MASK) >> 8) + 1;
	if (!s3c2410wdt_may_pet(wdt) && (pet || hb->divisor > cur_div))
		return -ETIME;

	wdt->count = hb->count;
	/* update the pre-scaler */
	wtcon &= ~S3C2410_WTCON_PRESCALE_MASK;
	wtcon |= S3C2410_WTCON_PRESCALE(hb->divisor - 1);

	writel(hb->count, wdt->reg_base + S3C2410_WTDAT);
	writel(wtcon, wdt->reg_base + S3C2410_WTCON);
	if (pet)
		writel(hb->count, wdt->reg_base + S3C2410_WTCNT);

	wdt->wdt_device.timeout = hb->timeout;
	return 0;
}

static int __s3c2410wdt_set_heartbeat(struct s3c2410_wdt *wdt,
				      unsigned int timeout, bool pet)
{
	struct s3c2410wdt_hb hb;
	unsigned long flags;
	int ret;

	ret = s3c2410wdt_calc_heartbeat(wdt, timeout, &hb);
	if (ret)
		return ret;
	spin_lock_irqsave(&wdt->lock, flags);
	ret = __s3c2410wdt_apply_heartbeat(wdt, &hb, pet);
	spin_unlock_irqrestore(&wdt->lock, flags);
	if (ret == -ETIME)
		s3c2410wdt_deadline_report(wdt);
	return ret;
}

/* New timeout, no reload; never slows down a counter that is no longer petted. */
static int s3c2410wdt_set_heartbeat(struct watchdog_device *wdd,
				    unsigned int timeout)
{
	return __s3c2410wdt_set_heartbeat(watchdog_get_drvdata(wdd), timeout,
					  false);
}

/* New timeout and a reload, atomically; -ETIME if reloading is refused. */
static int s3c2410wdt_set_heartbeat_pet(struct s3c2410_wdt *wdt,
					unsigned int timeout)
{
	return __s3c2410wdt_set_heartbeat(wdt, timeout, true);
}

/*
 * watchdog core .stop / .set_timeout: once a crownlte deadline is installed
 * or petting stopped for good, the guarded watchdog can be neither stopped
 * nor given another timeout by user space. Both decide under wdt->lock,
 * the lock s3c2410wdt_crownlte_set_deadline() installs the deadline under,
 * so neither can slip in after a deadline was installed. Internal callers
 * use s3c2410wdt_stop() / s3c2410wdt_set_heartbeat() directly.
 */
static int s3c2410wdt_ops_stop(struct watchdog_device *wdd)
{
	struct s3c2410_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&wdt->lock, flags);
	if (wdt->pet_deadline_ns || !s3c2410wdt_may_pet(wdt))
		ret = -EBUSY;
	else
		__s3c2410wdt_stop(wdt);
	spin_unlock_irqrestore(&wdt->lock, flags);
	if (ret)
		s3c2410wdt_deadline_report(wdt);
	return ret;
}

static int s3c2410wdt_ops_set_timeout(struct watchdog_device *wdd,
				      unsigned int timeout)
{
	struct s3c2410_wdt *wdt = watchdog_get_drvdata(wdd);
	struct s3c2410wdt_hb hb;
	unsigned long flags;
	int ret;

	ret = s3c2410wdt_calc_heartbeat(wdt, timeout, &hb);
	if (ret)
		return ret;
	spin_lock_irqsave(&wdt->lock, flags);
	if (wdt->pet_deadline_ns || !s3c2410wdt_may_pet(wdt))
		ret = -EBUSY;
	else
		ret = __s3c2410wdt_apply_heartbeat(wdt, &hb, false);
	spin_unlock_irqrestore(&wdt->lock, flags);
	if (ret)
		s3c2410wdt_deadline_report(wdt);
	return ret;
}

static int s3c2410wdt_start(struct watchdog_device *wdd)
{
	unsigned long wtcon;
	struct s3c2410_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;

	spin_lock_irqsave(&wdt->lock, flags);

	/*
	 * After the final stop the counter is never reloaded: one running
	 * with the reset enabled is left alone; otherwise (block reset in
	 * sleep) counter and reset are switched on from whatever WTCNT holds,
	 * so the reset the final stop asked for still comes, within at most
	 * the full count.
	 */
	if (!s3c2410wdt_may_pet(wdt)) {
		wtcon = readl(wdt->reg_base + S3C2410_WTCON);
		if ((wtcon & (S3C2410_WTCON_ENABLE | S3C2410_WTCON_RSTEN)) !=
		    (S3C2410_WTCON_ENABLE | S3C2410_WTCON_RSTEN)) {
			wtcon &= ~S3C2410_WTCON_INTEN;
			wtcon |= S3C2410_WTCON_ENABLE | S3C2410_WTCON_RSTEN;
			writel(wtcon, wdt->reg_base + S3C2410_WTCON);
		}
		spin_unlock_irqrestore(&wdt->lock, flags);
		s3c2410wdt_deadline_report(wdt);
		return 0;
	}

	__s3c2410wdt_stop(wdt);

	wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	wtcon |= S3C2410_WTCON_ENABLE | S3C2410_WTCON_DIV128;

	if (soft_noboot) {
		wtcon |= S3C2410_WTCON_INTEN;
		wtcon &= ~S3C2410_WTCON_RSTEN;
	} else {
		wtcon &= ~S3C2410_WTCON_INTEN;
		wtcon |= S3C2410_WTCON_RSTEN;
	}

	dev_dbg(wdt->dev, "Starting watchdog: count=0x%08x, wtcon=%08lx\n",
		wdt->count, wtcon);

	writel(wdt->count, wdt->reg_base + S3C2410_WTDAT);
	writel(wdt->count, wdt->reg_base + S3C2410_WTCNT);
	writel(wtcon, wdt->reg_base + S3C2410_WTCON);
	spin_unlock_irqrestore(&wdt->lock, flags);

	return 0;
}

static int s3c2410wdt_restart(struct watchdog_device *wdd, unsigned long action,
			      void *data)
{
	struct s3c2410_wdt *wdt = watchdog_get_drvdata(wdd);
	void __iomem *wdt_base = wdt->reg_base;

	/* disable watchdog, to be safe  */
	writel(0, wdt_base + S3C2410_WTCON);

	/* put initial values into count and data */
	writel(0x80, wdt_base + S3C2410_WTCNT);
	writel(0x80, wdt_base + S3C2410_WTDAT);

	/* set the watchdog to go and reset... */
	writel(S3C2410_WTCON_ENABLE | S3C2410_WTCON_DIV16 |
		S3C2410_WTCON_RSTEN | S3C2410_WTCON_PRESCALE(0x20),
		wdt_base + S3C2410_WTCON);

	/* wait for reset to assert... */
	mdelay(500);

	return 0;
}

#define OPTIONS (WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE)

static const struct watchdog_info s3c2410_wdt_ident = {
	.options          =     OPTIONS,
	.firmware_version =	0,
	.identity         =	"S3C2410 Watchdog",
};

static const struct watchdog_ops s3c2410wdt_ops = {
	.owner = THIS_MODULE,
	.start = s3c2410wdt_start,
	.stop = s3c2410wdt_ops_stop,
	.ping = s3c2410wdt_keepalive,
	.set_timeout = s3c2410wdt_ops_set_timeout,
	.restart = s3c2410wdt_restart,
};

static const struct watchdog_device s3c2410_wdd = {
	.info = &s3c2410_wdt_ident,
	.ops = &s3c2410wdt_ops,
	.timeout = S3C2410_WATCHDOG_DEFAULT_TIME,
};

/* interrupt handler code */

static irqreturn_t s3c2410wdt_irq(int irqno, void *param)
{
	struct s3c2410_wdt *wdt = platform_get_drvdata(param);

	dev_info(wdt->dev, "watchdog timer expired (irq)\n");

	s3c2410wdt_keepalive(&wdt->wdt_device);

	if (wdt->drv_data->quirks & QUIRK_HAS_WTCLRINT_REG)
		writel(0x1, wdt->reg_base + S3C2410_WTCLRINT);

	return IRQ_HANDLED;
}

static inline unsigned int s3c2410wdt_get_bootstatus(struct s3c2410_wdt *wdt)
{
	unsigned int rst_stat;
	int ret;

	if (!(wdt->drv_data->quirks & QUIRK_HAS_PMU_RST_STAT))
		return 0;

	ret = regmap_read(wdt->pmureg, wdt->drv_data->rst_stat_reg, &rst_stat);
	if (ret)
		dev_warn(wdt->dev, "Couldn't get RST_STAT register\n");
	else if (rst_stat & BIT(wdt->drv_data->rst_stat_bit))
		return WDIOF_CARDRESET;

	return 0;
}

static inline int
s3c2410_get_wdt_drv_data(struct platform_device *pdev, struct s3c2410_wdt *wdt)
{
	const struct s3c2410_wdt_variant *variant;
	struct device *dev = &pdev->dev;

	variant = of_device_get_match_data(dev);
	if (!variant) {
		/* Device matched by platform_device_id */
		variant = (struct s3c2410_wdt_variant *)
			   platform_get_device_id(pdev)->driver_data;
	}

#ifdef CONFIG_OF
	/* Choose Exynos850/ExynosAutov9 driver data w.r.t. cluster index */
	if (variant == &drv_data_exynos850_cl0 ||
	    variant == &drv_data_exynosautov9_cl0 ||
	    variant == &drv_data_gs101_cl0) {
		u32 index;
		int err;

		err = of_property_read_u32(dev->of_node,
					   "samsung,cluster-index", &index);
		if (err)
			return dev_err_probe(dev, -EINVAL, "failed to get cluster index\n");

		switch (index) {
		case 0:
			break;
		case 1:
			if (variant == &drv_data_exynos850_cl0)
				variant = &drv_data_exynos850_cl1;
			else if (variant == &drv_data_exynosautov9_cl0)
				variant = &drv_data_exynosautov9_cl1;
			else if (variant == &drv_data_gs101_cl0)
				variant = &drv_data_gs101_cl1;
			break;
		default:
			return dev_err_probe(dev, -EINVAL, "wrong cluster index: %u\n", index);
		}
	}
#endif

	wdt->drv_data = variant;
	return 0;
}

/* Unbind: after the final stop the reset is neither masked nor stopped. */
static void s3c2410wdt_wdt_disable_action(void *data)
{
	if (!s3c2410wdt_pet_refused(data))
		s3c2410wdt_enable(data, false);
}

static void s3c2410wdt_hw_stop_action(void *data)
{
	if (!s3c2410wdt_pet_refused(data))
		s3c2410wdt_stop(&((struct s3c2410_wdt *)data)->wdt_device);
}

static int s3c2410wdt_pmu_read(struct s3c2410_wdt *wdt, unsigned int reg,
			       unsigned int *val)
{
	int ret = regmap_read(wdt->pmureg, reg, val);

	if (ret)
		dev_err(wdt->dev, "not arming: PMU read %#x failed (%d)\n",
			reg, ret);
	return ret;
}

static int s3c2410wdt_pmu_clear(struct s3c2410_wdt *wdt, unsigned int reg,
				u32 mask, const char *what)
{
	unsigned int val;
	int ret = s3c2410wdt_pmu_read(wdt, reg, &val);

	if (ret)
		return ret;
	if (val & mask) {
		dev_err(wdt->dev,
			"not arming: %s set (PMU %#x = %#x, mask %#x); a watchdog reset would not be a plain full reset\n",
			what, reg, val, mask);
		return -ENODEV;
	}
	return 0;
}

/*
 * Samsung SBOOT picks the next boot from PMU INFORM2/INFORM3. After a
 * watchdog reset (RST_STAT CLUSTER0_WDTRESET) it enters Upload (ramdump) mode
 * instead of booting whenever its debug policy (PARAM DEBUG_LEVEL and
 * FORCE_UPLOAD) allows uploads. SBOOT records that policy in the PMU on every
 * boot: it enables DFD "dump GPR" (RESET_SEQUENCER_CONFIGURATION bit 0 = 0xc01)
 * exactly when an upload is possible and writes 0x00f000e0 plus clears the
 * per-CPU WDT reset-disable bits otherwise. So arm only if INFORM2/3 hold the
 * normal boot selection and DFD and every WDT reset-disable bit are clear.
 * This function only reads registers.
 */
static int s3c2410wdt_sboot_guard(struct s3c2410_wdt *wdt)
{
	unsigned int inform2, inform3, rst_stat, i;
	int ret;

	ret = s3c2410wdt_pmu_read(wdt, EXYNOS9810_INFORM2, &inform2);
	if (!ret)
		ret = s3c2410wdt_pmu_read(wdt, EXYNOS9810_INFORM3, &inform3);
	if (!ret)
		ret = s3c2410wdt_pmu_read(wdt, wdt->drv_data->rst_stat_reg,
					  &rst_stat);
	if (ret)
		return ret;

	dev_info(wdt->dev, "sboot state: INFORM2=%#x INFORM3=%#x RST_STAT=%#x%s\n",
		 inform2, inform3, rst_stat,
		 rst_stat & BIT(wdt->drv_data->rst_stat_bit) ?
		 " (last reset: CLUSTER0 watchdog)" : "");

	if (inform2 != SEC_POWER_RESET || inform3 != SEC_RESET_REASON_UNKNOWN) {
		dev_err(wdt->dev,
			"not arming: boot selection is not normal boot (want INFORM2=%#x INFORM3=%#x); a watchdog reset would not return to the normal BOOT image\n",
			SEC_POWER_RESET, SEC_RESET_REASON_UNKNOWN);
		return -ENODEV;
	}

	ret = s3c2410wdt_pmu_clear(wdt, EXYNOS9810_RESET_SEQUENCER_CONFIGURATION,
				   EXYNOS9810_DFD_EDPCSR_DUMP_EN |
				   EXYNOS9810_DFD_RESET_DISABLE,
				   "DFD dump-GPR (sboot upload-capable debug policy)");
	if (ret)
		return ret;
	ret = s3c2410wdt_pmu_clear(wdt, EXYNOS9810_CPU_RESET_DISABLE_FROM_WDTRESET,
				   EXYNOS9810_WDT_RESET_PEND,
				   "CPU reset disable from WDT reset");
	if (ret)
		return ret;
	for (i = 0; i < 4; i++) {
		ret = s3c2410wdt_pmu_clear(wdt, EXYNOS9810_BIG_CPU0_RESET +
					   i * EXYNOS9810_PMU_CPU_STRIDE,
					   EXYNOS9810_BIG_CPU_RESET_DISABLE,
					   "big CPU reset disable");
		if (ret)
			return ret;
		ret = s3c2410wdt_pmu_clear(wdt, EXYNOS9810_LITTLE_CPU0_ETC_RESET +
					   i * EXYNOS9810_PMU_CPU_STRIDE,
					   EXYNOS9810_LITTLE_CPU_RESET_DISABLE,
					   "LITTLE CPU reset disable");
		if (ret)
			return ret;
	}
	return s3c2410wdt_pmu_clear(wdt, EXYNOS9810_BIG_NONCPU_ETC_RESET,
				    EXYNOS9810_BIG_NONCPU_RESET_DISABLE,
				    "big non-CPU reset disable");
}

/*
 * The timeout is computed from src_clk. Prove that WTCNT really decrements at
 * src_clk / 2048 before trusting it: a faster counter would reset a healthy
 * system, a stopped one would never reset. RSTEN and INTEN stay clear, so this
 * cannot reset or interrupt. Leaves the timer stopped.
 */
static int s3c2410wdt_check_rate(struct s3c2410_wdt *wdt)
{
	const unsigned int div = S3C2410_RATE_CHECK_PRESCALE * S3C2410_WTCON_MAXDIV;
	unsigned long rate = s3c2410wdt_get_freq(wdt);
	unsigned int wtcon, wtdat, wtcnt, i;
	u32 cnt[3];
	ktime_t t[3];
	u64 expect, ticks;
	s64 ns;

	wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	wtdat = readl(wdt->reg_base + S3C2410_WTDAT);
	wtcnt = readl(wdt->reg_base + S3C2410_WTCNT);
	dev_info(wdt->dev, "left by bootloader: WTCON=%#x WTDAT=%#x WTCNT=%#x\n",
		 wtcon, wtdat, wtcnt);

	if (!rate) {
		dev_err(wdt->dev, "not arming: source clock rate is 0\n");
		return -ENODEV;
	}

	writel(0, wdt->reg_base + S3C2410_WTCON);
	writel(S3C2410_WTCNT_MAXCNT, wdt->reg_base + S3C2410_WTDAT);
	writel(S3C2410_WTCNT_MAXCNT, wdt->reg_base + S3C2410_WTCNT);
	writel(S3C2410_WTCON_ENABLE | S3C2410_WTCON_DIV128 |
	       S3C2410_WTCON_PRESCALE(S3C2410_RATE_CHECK_PRESCALE - 1),
	       wdt->reg_base + S3C2410_WTCON);
	for (i = 0; i < ARRAY_SIZE(cnt); i++) {
		if (i)
			msleep(S3C2410_RATE_CHECK_SLEEP_MS);
		t[i] = ktime_get();
		cnt[i] = readl(wdt->reg_base + S3C2410_WTCNT);
	}
	writel(0, wdt->reg_base + S3C2410_WTCON);

	ns = ktime_to_ns(ktime_sub(t[2], t[0]));
	if (!(cnt[0] > cnt[1] && cnt[1] > cnt[2]) || ns <= 0) {
		dev_err(wdt->dev,
			"not arming: counter not counting down (WTCNT %#x %#x %#x)\n",
			cnt[0], cnt[1], cnt[2]);
		return -ENODEV;
	}
	ticks = cnt[0] - cnt[2];
	expect = div64_u64((u64)rate * ns, div * (u64)NSEC_PER_SEC);
	dev_info(wdt->dev, "counter rate check: %llu ticks in %lld us, expected %llu (%lu Hz / %u)\n",
		 ticks, div_s64(ns, NSEC_PER_USEC), expect, rate, div);
	if (!expect || ticks * 10 < expect * 9 || ticks * 10 > expect * 11) {
		dev_err(wdt->dev,
			"not arming: counter rate is not within 10%% of the source clock\n");
		return -ENODEV;
	}
	return 0;
}

/* Prove the reset request really reaches the PMU (both bits must read 0). */
static int s3c2410wdt_check_pmu_unmasked(struct s3c2410_wdt *wdt)
{
	const u32 bit = BIT(wdt->drv_data->mask_bit);
	unsigned int disable, mask;
	int ret;

	ret = s3c2410wdt_pmu_read(wdt, wdt->drv_data->disable_reg, &disable);
	if (!ret)
		ret = s3c2410wdt_pmu_read(wdt, wdt->drv_data->mask_reset_reg,
					  &mask);
	if (ret)
		return ret;
	dev_info(wdt->dev, "PMU AUTOMATIC_WDT_RESET_DISABLE=%#x MASK_WDT_RESET_REQUEST=%#x (bit %d)\n",
		 disable, mask, wdt->drv_data->mask_bit);
	if ((disable | mask) & bit) {
		dev_err(wdt->dev,
			"not arming: PMU still disables/masks the watchdog reset\n");
		return -ENODEV;
	}
	return 0;
}

/*
 * Opt-in expiry selftest, "samsung,selftest-stop-petting-ms" in DT only.
 * At that boot time, stop petting and shorten the timeout so the hardware
 * reset is due a few seconds later. A second pass reports a failure if the
 * kernel is still alive well after the expected reset.
 */
static void s3c2410wdt_selftest_work(struct work_struct *work)
{
	struct s3c2410_wdt *wdt = container_of(to_delayed_work(work),
					       struct s3c2410_wdt, selftest_work);
	struct s3c2410wdt_hb hb;
	unsigned long flags;
	int ret;

	if (wdt->selftest_fired) {
		dev_emerg(wdt->dev,
			  "watchdog selftest: FAIL, still running %d s after petting stopped (WTCON=%#x WTCNT=%#x)\n",
			  S3C2410_SELFTEST_EXPIRE_S + 5,
			  readl(wdt->reg_base + S3C2410_WTCON),
			  readl(wdt->reg_base + S3C2410_WTCNT));
		return;
	}

	/*
	 * Short timeout, last reload and the stop in one wdt->lock section;
	 * no reload if petting already stopped for good (deadline passed).
	 */
	ret = s3c2410wdt_calc_heartbeat(wdt, S3C2410_SELFTEST_EXPIRE_S, &hb);
	spin_lock_irqsave(&wdt->lock, flags);
	if (!ret)
		ret = __s3c2410wdt_apply_heartbeat(wdt, &hb, true);
	__s3c2410wdt_final_stop(wdt, "watchdog selftest");
	wdt->selftest_fired = true;
	spin_unlock_irqrestore(&wdt->lock, flags);

	dev_emerg(wdt->dev,
		  "watchdog selftest: petting stopped at %lld ms, expecting CLUSTER0 watchdog reset in ~%d s (ret %d, WTCON=%#x WTCNT=%#x)\n",
		  ktime_to_ms(ktime_get_boottime()), S3C2410_SELFTEST_EXPIRE_S,
		  ret, readl(wdt->reg_base + S3C2410_WTCON),
		  readl(wdt->reg_base + S3C2410_WTCNT));
	schedule_delayed_work(&wdt->selftest_work,
			      msecs_to_jiffies((S3C2410_SELFTEST_EXPIRE_S + 5) *
					       MSEC_PER_SEC));
}

static void s3c2410wdt_selftest_cancel(void *data)
{
	cancel_delayed_work_sync(&((struct s3c2410_wdt *)data)->selftest_work);
}

static int s3c2410wdt_selftest_setup(struct s3c2410_wdt *wdt)
{
	struct device *dev = wdt->dev;
	s64 now, delay;
	u32 at_ms;
	int ret;

	if (of_property_read_u32(dev->of_node, "samsung,selftest-stop-petting-ms",
				 &at_ms))
		return 0;
	if (at_ms < S3C2410_SELFTEST_MIN_MS || at_ms > S3C2410_SELFTEST_MAX_MS) {
		dev_warn(dev, "watchdog selftest: ignoring stop time %u ms (allowed %u..%u)\n",
			 at_ms, S3C2410_SELFTEST_MIN_MS, S3C2410_SELFTEST_MAX_MS);
		return 0;
	}

	INIT_DELAYED_WORK(&wdt->selftest_work, s3c2410wdt_selftest_work);
	ret = devm_add_action_or_reset(dev, s3c2410wdt_selftest_cancel, wdt);
	if (ret)
		return ret;
	now = ktime_to_ms(ktime_get_boottime());
	delay = at_ms > now ? at_ms - now : 0;
	schedule_delayed_work(&wdt->selftest_work, msecs_to_jiffies(delay));
	dev_warn(dev, "watchdog selftest ENABLED: will stop petting at %u ms boot time (in %lld ms)\n",
		 at_ms, delay);
	return 0;
}

static int s3c2410wdt_reboot_notify(struct notifier_block *nb,
				    unsigned long action, void *data);

static void s3c2410wdt_crownlte_clear(void *data)
{
	if (READ_ONCE(s3c2410wdt_crownlte) == data)
		WRITE_ONCE(s3c2410wdt_crownlte, NULL);
}

/*
 * crownlte-wdt.h: hooks for the crownlte suspend driver
 * (drivers/soc/samsung/exynos9810-pm.c). Only the guarded instance is
 * served; every call is safe from any context, including hardirq and
 * syscore with interrupts off.
 */

/*
 * Reload the counter now (e.g. right before the firmware sleep call).
 * -ETIME: not reloaded, petting has stopped for good (or the deadline
 * passed).
 */
int s3c2410wdt_crownlte_ping(void)
{
	struct s3c2410_wdt *wdt = smp_load_acquire(&s3c2410wdt_crownlte);

	if (!wdt)
		return -ENODEV;
	if (!(readl(wdt->reg_base + S3C2410_WTCON) & S3C2410_WTCON_ENABLE))
		return -EIO;
	return __s3c2410wdt_keepalive(wdt) ? 0 : -ETIME;
}
EXPORT_SYMBOL_GPL(s3c2410wdt_crownlte_ping);

/*
 * Absolute deadline (CLOCK_BOOTTIME ns) after which the counter is never
 * reloaded again, by any path on any CPU (s3c2410wdt_may_pet()). Only
 * accepted while the watchdog runs with the reset enabled and is still
 * petted; a later call may only move it earlier. Once set, user space can
 * no longer stop the watchdog.
 */
int s3c2410wdt_crownlte_set_deadline(u64 boot_ns)
{
	struct s3c2410_wdt *wdt = smp_load_acquire(&s3c2410wdt_crownlte);
	unsigned long flags;
	u32 wtcon;
	int ret = 0;

	if (!wdt)
		return -ENODEV;
	if (!boot_ns)
		return -EINVAL;
	spin_lock_irqsave(&wdt->lock, flags);
	wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	if (!(wtcon & S3C2410_WTCON_ENABLE) || !(wtcon & S3C2410_WTCON_RSTEN))
		ret = -EIO;
	else if (!s3c2410wdt_may_pet(wdt))
		ret = -ETIME;
	else if (!wdt->pet_deadline_ns || boot_ns < wdt->pet_deadline_ns)
		wdt->pet_deadline_ns = boot_ns;
	spin_unlock_irqrestore(&wdt->lock, flags);
	if (!ret)
		dev_info(wdt->dev, "crownlte deadline: no reload after %llu ms boot time (WTCON %#x)\n",
			 div_u64(wdt->pet_deadline_ns, NSEC_PER_MSEC), wtcon);
	return ret;
}
EXPORT_SYMBOL_GPL(s3c2410wdt_crownlte_set_deadline);

/*
 * The one public final stop, shared by every crownlte user (suspend
 * deadline, C2 lost-core chain): stop petting for good, so the running
 * watchdog resets the SoC within its current timeout. For a hang the kernel
 * can see but not undo (a CPU lost while online): other CPUs would
 * otherwise keep petting forever.
 *
 * Irreversible and idempotent, decided entirely under wdt->lock by
 * __s3c2410wdt_final_stop(): one last reload of a running counter, only
 * while s3c2410wdt_may_pet() still allows it (so the logs get a full
 * timeout to reach pstore), never after the deadline passed or after an
 * earlier final stop (restart notifier, selftest, an earlier call); then
 * the shared stopped state is latched. From then on no path reloads the
 * counter, slows its prescaler, stops it, masks its reset or lets a resume
 * reload it.
 *
 * Returns the timeout in seconds if the counter runs with the reset
 * enabled, -ENODEV without a guarded instance, -EIO if the counter is not
 * running with the reset enabled right now (e.g. switched off by a system
 * suspend that completed first). With -EIO the state is still latched: the
 * resume switches the counter back on without a reload.
 */
int s3c2410wdt_crownlte_stop_petting(const char *why)
{
	struct s3c2410_wdt *wdt = smp_load_acquire(&s3c2410wdt_crownlte);
	unsigned long flags;
	bool reload, armed;
	u32 wtcon;

	if (!wdt)
		return -ENODEV;
	spin_lock_irqsave(&wdt->lock, flags);
	reload = __s3c2410wdt_final_stop(wdt, why);
	wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	armed = (wtcon & (S3C2410_WTCON_ENABLE | S3C2410_WTCON_RSTEN)) ==
		(S3C2410_WTCON_ENABLE | S3C2410_WTCON_RSTEN);
	spin_unlock_irqrestore(&wdt->lock, flags);
	if (reload)
		pr_emerg("s3c2410-wdt: petting stopped for good (%s): SoC reset in about %u s\n",
			 why, wdt->wdt_device.timeout);
	else
		pr_emerg("s3c2410-wdt: petting already stopped (%s, first: %s): no reload, reset due within %u s (WTCON %#x WTCNT %#x)\n",
			 why, wdt->stop_why, wdt->wdt_device.timeout, wtcon,
			 readl(wdt->reg_base + S3C2410_WTCNT));
	s3c2410wdt_deadline_report(wdt);
	if (!armed) {
		pr_emerg("s3c2410-wdt: final stop (%s) latched, but the counter is not running with the reset enabled now (WTCON %#x%s)\n",
			 why, wtcon,
			 READ_ONCE(wdt->pm_stopped) ? ", stopped for system suspend: resume re-arms it without a reload" : "");
		return -EIO;
	}
	return wdt->wdt_device.timeout;
}
EXPORT_SYMBOL_GPL(s3c2410wdt_crownlte_stop_petting);

/* WTCON/WTDAT/WTCNT snapshot for logs; false if no guarded instance. */
bool s3c2410wdt_crownlte_state(struct crownlte_wdt_state *st)
{
	struct s3c2410_wdt *wdt = smp_load_acquire(&s3c2410wdt_crownlte);

	if (!wdt)
		return false;
	st->wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	st->wtdat = readl(wdt->reg_base + S3C2410_WTDAT);
	st->wtcnt = readl(wdt->reg_base + S3C2410_WTCNT);
	st->timeout = wdt->wdt_device.timeout;
	st->enabled = !!(st->wtcon & S3C2410_WTCON_ENABLE);
	st->reset = !!(st->wtcon & S3C2410_WTCON_RSTEN);
	st->keep_armed_suspend = wdt->keep_armed_suspend;
	/* Evaluates the deadline too, as the ping path does. */
	st->pet_stopped = s3c2410wdt_pet_refused(wdt);
	st->deadline_ns = READ_ONCE(wdt->pet_deadline_ns);
	return true;
}
EXPORT_SYMBOL_GPL(s3c2410wdt_crownlte_state);

static int s3c2410wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s3c2410_wdt *wdt;
	unsigned int wtcon;
	bool start = tmr_atboot;
	bool guarded;
	int wdt_irq;
	int ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->dev = dev;
	spin_lock_init(&wdt->lock);
	wdt->wdt_device = s3c2410_wdd;

	ret = s3c2410_get_wdt_drv_data(pdev, wdt);
	if (ret)
		return ret;

	if (wdt->drv_data->quirks & QUIRKS_HAVE_PMUREG) {
		wdt->pmureg = syscon_regmap_lookup_by_phandle(dev->of_node,
						"samsung,syscon-phandle");
		if (IS_ERR(wdt->pmureg))
			return dev_err_probe(dev, PTR_ERR(wdt->pmureg),
					     "syscon regmap lookup failed.\n");
	}

	wdt_irq = platform_get_irq(pdev, 0);
	if (wdt_irq < 0)
		return wdt_irq;

	/* get the memory region for the watchdog timer */
	wdt->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdt->reg_base))
		return PTR_ERR(wdt->reg_base);

	wdt->bus_clk = devm_clk_get_enabled(dev, "watchdog");
	if (IS_ERR(wdt->bus_clk))
		return dev_err_probe(dev, PTR_ERR(wdt->bus_clk), "failed to get bus clock\n");

	/*
	 * "watchdog_src" clock is optional; if it's not present -- just skip it
	 * and use "watchdog" clock as both bus and source clock.
	 */
	wdt->src_clk = devm_clk_get_optional_enabled(dev, "watchdog_src");
	if (IS_ERR(wdt->src_clk))
		return dev_err_probe(dev, PTR_ERR(wdt->src_clk), "failed to get source clock\n");

	guarded = wdt->drv_data->quirks & QUIRK_SBOOT_RESET_GUARD;
	if (guarded) {
		/* Read-only reset-path checks first, then a no-reset rate check. */
		ret = s3c2410wdt_sboot_guard(wdt);
		if (ret)
			return ret;
		ret = s3c2410wdt_check_rate(wdt);
		if (ret)
			return ret;
		/*
		 * The core stops a boot-running watchdog neither on unregister
		 * nor on probe failure; stop the hardware on any unwind.
		 */
		ret = devm_add_action_or_reset(dev, s3c2410wdt_hw_stop_action, wdt);
		if (ret)
			return ret;
		start |= of_property_read_bool(dev->of_node, "samsung,start-at-boot");
	}

	wdt->wdt_device.min_timeout = 1;
	wdt->wdt_device.max_timeout = s3c2410wdt_max_timeout(wdt);

	watchdog_set_drvdata(&wdt->wdt_device, wdt);

	/* see if we can actually set the requested timer margin, and if
	 * not, try the default value */

	watchdog_init_timeout(&wdt->wdt_device, tmr_margin, dev);
	ret = s3c2410wdt_set_heartbeat(&wdt->wdt_device,
					wdt->wdt_device.timeout);
	if (ret) {
		ret = s3c2410wdt_set_heartbeat(&wdt->wdt_device,
					       S3C2410_WATCHDOG_DEFAULT_TIME);
		if (ret == 0)
			dev_warn(dev, "tmr_margin value out of range, default %d used\n",
				 S3C2410_WATCHDOG_DEFAULT_TIME);
		else
			return dev_err_probe(dev, ret, "failed to use default timeout\n");
	}

	ret = devm_request_irq(dev, wdt_irq, s3c2410wdt_irq, 0,
			       pdev->name, pdev);
	if (ret != 0)
		return dev_err_probe(dev, ret, "failed to install irq (%d)\n", ret);

	watchdog_set_nowayout(&wdt->wdt_device, nowayout);
	watchdog_set_restart_priority(&wdt->wdt_device, 128);

	wdt->wdt_device.bootstatus = s3c2410wdt_get_bootstatus(wdt);
	wdt->wdt_device.parent = dev;

	s3c2410wdt_mask_dbgack(wdt);

	/*
	 * If "tmr_atboot" param is non-zero, start the watchdog right now. Also
	 * set WDOG_HW_RUNNING bit, so that watchdog core can kick the watchdog.
	 *
	 * If we're not enabling the watchdog, then ensure it is disabled if it
	 * has been left running from the bootloader or other source.
	 */
	if (start) {
		dev_info(dev, "starting watchdog timer\n");
		s3c2410wdt_start(&wdt->wdt_device);
		set_bit(WDOG_HW_RUNNING, &wdt->wdt_device.status);
	} else {
		s3c2410wdt_stop(&wdt->wdt_device);
	}

	ret = devm_watchdog_register_device(dev, &wdt->wdt_device);
	if (ret)
		return ret;

	if (wdt->drv_data->quirks & QUIRK_KEEP_RUNNING_ON_SHUTDOWN) {
		wdt->reboot_nb.notifier_call = s3c2410wdt_reboot_notify;
		wdt->reboot_nb.priority = INT_MAX;	/* before anything that may hang */
		ret = devm_register_reboot_notifier(dev, &wdt->reboot_nb);
		if (ret)
			return ret;
	}

	ret = s3c2410wdt_enable(wdt, true);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(dev, s3c2410wdt_wdt_disable_action, wdt);
	if (ret)
		return ret;

	if (guarded) {
		ret = s3c2410wdt_check_pmu_unmasked(wdt);
		if (ret)
			return ret;
	}

	platform_set_drvdata(pdev, wdt);

	/* print out a statement of readiness */

	wtcon = readl(wdt->reg_base + S3C2410_WTCON);

	dev_info(dev, "watchdog %sactive, reset %sabled, irq %sabled\n",
		 (wtcon & S3C2410_WTCON_ENABLE) ?  "" : "in",
		 (wtcon & S3C2410_WTCON_RSTEN) ? "en" : "dis",
		 (wtcon & S3C2410_WTCON_INTEN) ? "en" : "dis");

	if (guarded && (wtcon & S3C2410_WTCON_ENABLE)) {
		dev_info(dev, "armed: timeout %u s, kernel pings until userspace opens it\n",
			 wdt->wdt_device.timeout);
		ret = s3c2410wdt_selftest_setup(wdt);
		if (ret)
			return ret;
		wdt->keep_armed_suspend = of_property_read_bool(dev->of_node,
						"samsung,keep-armed-in-suspend");
		if (wdt->keep_armed_suspend)
			dev_info(dev, "system suspend keeps it armed (timeout %u s while suspended)\n",
				 min_t(unsigned int, S3C2410_WDT_SUSPEND_TIMEOUT,
				       wdt->wdt_device.max_timeout));
		ret = devm_add_action_or_reset(dev, s3c2410wdt_crownlte_clear,
					       wdt);
		if (ret)
			return ret;
		smp_store_release(&s3c2410wdt_crownlte, wdt);
	}

	return 0;
}

/*
 * QUIRK_KEEP_RUNNING_ON_SHUTDOWN: a restart that hangs before it reaches the
 * reset (trial q22b: a CPU lost in C2 kept the reboot from completing while
 * the kernel kept petting) must still end in a reset. At the very start of a
 * restart, set a short timeout, pet once, then stop petting for good. Power
 * off is left alone so a slow power-off is never turned into a reboot.
 */
#define S3C2410_WDT_RESTART_TIMEOUT	30

static int s3c2410wdt_reboot_notify(struct notifier_block *nb,
				    unsigned long action, void *data)
{
	struct s3c2410_wdt *wdt = container_of(nb, struct s3c2410_wdt, reboot_nb);
	struct s3c2410wdt_hb hb;
	unsigned long flags;
	bool hb_ok, reloaded;

	if (action != SYS_RESTART)
		return NOTIFY_DONE;
	if (!(readl(wdt->reg_base + S3C2410_WTCON) & S3C2410_WTCON_ENABLE))
		return NOTIFY_DONE;
	hb_ok = !s3c2410wdt_calc_heartbeat(wdt, S3C2410_WDT_RESTART_TIMEOUT, &hb);

	/*
	 * One wdt->lock section: the short timeout without a reload (after
	 * the final stop only a change that does not slow the counter down is
	 * made), then the shared final stop, whose one last reload (with the
	 * new count) only happens while reloading is still allowed.
	 */
	spin_lock_irqsave(&wdt->lock, flags);
	if (hb_ok)
		__s3c2410wdt_apply_heartbeat(wdt, &hb, false);
	reloaded = __s3c2410wdt_final_stop(wdt, "restart");
	spin_unlock_irqrestore(&wdt->lock, flags);
	if (!reloaded) {
		s3c2410wdt_deadline_report(wdt);
		dev_info(wdt->dev, "restart: petting already stopped, reset due within %u s\n",
			 wdt->wdt_device.timeout);
	} else {
		dev_info(wdt->dev, "restart: armed for %u s and no longer petted\n",
			 wdt->wdt_device.timeout);
	}
	return NOTIFY_DONE;
}

static void s3c2410wdt_shutdown(struct platform_device *dev)
{
	struct s3c2410_wdt *wdt = platform_get_drvdata(dev);

	if ((wdt->drv_data->quirks & QUIRK_KEEP_RUNNING_ON_SHUTDOWN) &&
	    (readl(wdt->reg_base + S3C2410_WTCON) & S3C2410_WTCON_ENABLE)) {
		s3c2410wdt_keepalive(&wdt->wdt_device);
		dev_info(wdt->dev, "left armed across shutdown (timeout %u s)\n",
			 wdt->wdt_device.timeout);
		return;
	}
	/* After the final stop the reset is never masked or stopped. */
	if (s3c2410wdt_pet_refused(wdt))
		return;

	s3c2410wdt_enable(wdt, false);
	s3c2410wdt_stop(&wdt->wdt_device);
}

/*
 * samsung,keep-armed-in-suspend: stay armed with the reset enabled and a
 * longer timeout. The PMU reset mask is left as it is. If the firmware
 * really stops the counter clock in system sleep, a resume re-initializes
 * it (see resume). Every failure keeps the watchdog running as it was and
 * fails the suspend; this path never stops it.
 */
static int s3c2410wdt_suspend_keep_armed(struct s3c2410_wdt *wdt, u32 wtcon)
{
	unsigned int t = min_t(unsigned int, S3C2410_WDT_SUSPEND_TIMEOUT,
			       wdt->wdt_device.max_timeout);
	struct device *dev = wdt->dev;
	int ret;

	if (!(wtcon & S3C2410_WTCON_RSTEN)) {
		dev_err(dev, "suspend refused: kept-armed watchdog runs with the reset disabled (WTCON %#x)\n",
			wtcon);
		return -EIO;
	}
	wdt->resume_timeout = wdt->wdt_device.timeout;
	/*
	 * Slower prescaler and reload in one wdt->lock section: refused, with
	 * nothing written, once petting has stopped for good or the deadline
	 * has passed, so the suspend count can never stretch a due reset.
	 */
	ret = s3c2410wdt_set_heartbeat_pet(wdt, t);
	if (ret == -ETIME) {
		dev_err(dev, "suspend refused: petting stopped for good, the reset stays due (WTCON %#x WTCNT %#x)\n",
			wtcon, readl(wdt->reg_base + S3C2410_WTCNT));
		return -EBUSY;
	}
	if (ret) {
		/* Bad timeout: nothing was written. */
		s3c2410wdt_keepalive(&wdt->wdt_device);
		dev_err(dev, "suspend refused: keep-armed timeout %u s failed (%d), left running at %u s\n",
			t, ret, wdt->wdt_device.timeout);
		return ret;
	}
	wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	if ((wtcon & (S3C2410_WTCON_ENABLE | S3C2410_WTCON_RSTEN)) !=
	    (S3C2410_WTCON_ENABLE | S3C2410_WTCON_RSTEN)) {
		/* Back to the run-time count; without a reload if refused. */
		if (s3c2410wdt_set_heartbeat_pet(wdt, wdt->resume_timeout))
			s3c2410wdt_set_heartbeat(&wdt->wdt_device,
						 wdt->resume_timeout);
		dev_err(dev, "suspend refused: WTCON %#x after setting the suspend timeout\n",
			wtcon);
		return -EIO;
	}
	wdt->suspended_armed = true;
	dev_info(dev, "suspend: kept armed, timeout %u s, WTCON %#x\n",
		 wdt->wdt_device.timeout, wtcon);
	return 0;
}

static int s3c2410wdt_resume(struct device *dev);

static int s3c2410wdt_suspend(struct device *dev)
{
	int ret;
	struct s3c2410_wdt *wdt = dev_get_drvdata(dev);
	u32 wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	unsigned long flags;

	wdt->suspended_armed = false;
	if (wdt->keep_armed_suspend && (wtcon & S3C2410_WTCON_ENABLE))
		return s3c2410wdt_suspend_keep_armed(wdt, wtcon);
	/*
	 * A crownlte deadline or the final stop: never stop it either. The
	 * check, the save and switching the counter off are one wdt->lock
	 * section, so a final stop is either seen here (suspend refused) or
	 * comes after the counter is off (pm_stopped: the stop latches, and
	 * resume re-arms without a reload). The PMU reset mask below is only
	 * set while the counter is off.
	 */
	spin_lock_irqsave(&wdt->lock, flags);
	wtcon = readl(wdt->reg_base + S3C2410_WTCON);
	if ((wtcon & S3C2410_WTCON_ENABLE) &&
	    (wdt->pet_deadline_ns || !s3c2410wdt_may_pet(wdt))) {
		spin_unlock_irqrestore(&wdt->lock, flags);
		s3c2410wdt_deadline_report(wdt);
		dev_err(dev, "suspend refused: watchdog under a deadline or no longer petted (WTCON %#x)\n",
			wtcon);
		return -EBUSY;
	}
	/* Save watchdog state, and turn it off. WTCNT is not saved. */
	wdt->wtcon_save = wtcon;
	wdt->wtdat_save = readl(wdt->reg_base + S3C2410_WTDAT);
	__s3c2410wdt_stop(wdt);
	wdt->pm_stopped = true;
	spin_unlock_irqrestore(&wdt->lock, flags);

	ret = s3c2410wdt_enable(wdt, false);
	if (ret < 0) {
		/* The PM core will not call resume for this device. */
		s3c2410wdt_resume(dev);
		return ret;
	}
	return 0;
}

static int s3c2410wdt_resume(struct device *dev)
{
	int ret;
	struct s3c2410_wdt *wdt = dev_get_drvdata(dev);
	unsigned long flags;
	bool final = false;
	u32 wtcon;

	if (wdt->suspended_armed) {
		u32 wtcon = readl(wdt->reg_base + S3C2410_WTCON);

		wdt->suspended_armed = false;
		/*
		 * Still running: back to the run-time timeout. Lost (the block
		 * was reset): start it again, as the boot-time start did.
		 */
		if (!(wtcon & S3C2410_WTCON_ENABLE) ||
		    !(wtcon & S3C2410_WTCON_RSTEN)) {
			dev_warn(dev, "resume: found WTCON %#x after a kept-armed suspend; restarting\n",
				 wtcon);
			s3c2410wdt_set_heartbeat(&wdt->wdt_device,
						 wdt->resume_timeout);
			s3c2410wdt_start(&wdt->wdt_device);
		} else if (s3c2410wdt_set_heartbeat_pet(wdt,
						       wdt->resume_timeout)) {
			/*
			 * No reload allowed any more: still go back to the
			 * faster run-time prescaler, which only shortens the
			 * time left.
			 */
			s3c2410wdt_set_heartbeat(&wdt->wdt_device,
						 wdt->resume_timeout);
		}
		dev_info(dev, "resume: armed, timeout %u s, WTCON %#x\n",
			 wdt->wdt_device.timeout,
			 readl(wdt->reg_base + S3C2410_WTCON));
		return 0;
	}

	/*
	 * PMU reset unmask first, then the counter: a counter that is due
	 * never expires while its reset is still masked.
	 */
	ret = s3c2410wdt_enable(wdt, true);

	/*
	 * Restore watchdog state, in one wdt->lock section with the final
	 * stop check. After a final stop (latched while the counter was off
	 * for suspend) WTCNT is not reloaded: the counter goes on from the
	 * value it stopped at, with the reset enabled.
	 */
	spin_lock_irqsave(&wdt->lock, flags);
	wdt->pm_stopped = false;
	if (!s3c2410wdt_may_pet(wdt)) {
		final = true;
		wtcon = wdt->wtcon_save;
		if (wtcon & S3C2410_WTCON_ENABLE) {
			wtcon &= ~S3C2410_WTCON_INTEN;
			wtcon |= S3C2410_WTCON_RSTEN;
		}
		writel(wtcon, wdt->reg_base + S3C2410_WTCON);
	} else {
		writel(wdt->wtdat_save, wdt->reg_base + S3C2410_WTDAT);
		writel(wdt->wtdat_save, wdt->reg_base + S3C2410_WTCNT);/* Reset count */
		writel(wdt->wtcon_save, wdt->reg_base + S3C2410_WTCON);
	}
	spin_unlock_irqrestore(&wdt->lock, flags);

	if (final) {
		s3c2410wdt_deadline_report(wdt);
		dev_emerg(dev, "resume after the final stop (%s): re-armed without a reload (WTCON %#x WTCNT %#x)\n",
			  wdt->stop_why, readl(wdt->reg_base + S3C2410_WTCON),
			  readl(wdt->reg_base + S3C2410_WTCNT));
	}
	if (ret < 0)
		return ret;

	dev_info(dev, "watchdog %sabled\n",
		(wdt->wtcon_save & S3C2410_WTCON_ENABLE) ? "en" : "dis");

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(s3c2410wdt_pm_ops,
				s3c2410wdt_suspend, s3c2410wdt_resume);

static struct platform_driver s3c2410wdt_driver = {
	.probe		= s3c2410wdt_probe,
	.shutdown	= s3c2410wdt_shutdown,
	.id_table	= s3c2410_wdt_ids,
	.driver		= {
		.name	= "s3c2410-wdt",
		.pm	= pm_sleep_ptr(&s3c2410wdt_pm_ops),
		.of_match_table	= of_match_ptr(s3c2410_wdt_match),
	},
};

/*
 * Built in, register at subsys_initcall so a watchdog armed at boot covers
 * every device_initcall driver (cpufreq, cpuidle, UFS, devfreq, ...). Its
 * clocks are CLK_OF_DECLARE providers and the PMU is a plain syscon, both
 * available by then; the watchdog core defers the chardev registration and
 * starts pinging at its subsys_initcall_sync.
 */
static int __init s3c2410wdt_driver_init(void)
{
	return platform_driver_register(&s3c2410wdt_driver);
}
subsys_initcall(s3c2410wdt_driver_init);

static void __exit s3c2410wdt_driver_exit(void)
{
	platform_driver_unregister(&s3c2410wdt_driver);
}
module_exit(s3c2410wdt_driver_exit);

MODULE_AUTHOR("Ben Dooks <ben@simtec.co.uk>, Dimitry Andric <dimitry.andric@tomtom.com>");
MODULE_DESCRIPTION("S3C2410 Watchdog Device Driver");
MODULE_LICENSE("GPL");
