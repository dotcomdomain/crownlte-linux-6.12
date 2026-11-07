// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 9810 (crownlte) audio stage A1: read-only hardware probe.
 *
 * Design: gekko roadmap/audio-design.md, section 5 "A1. Read-only probe".
 * Expected lines, failure signatures and the trial: Documentation/crownlte/audio-a1.md.
 *
 * At probe (device initcall, before the amp bus exists):
 *
 *  0. CL0 watchdog deadline (samsung,audio-a1-deadline-ms) installed and
 *     verified, as the suspend S0 driver does: after the deadline no CPU
 *     can reload the counter, so a CPU wedged on a bus access ends in a
 *     watchdog reset even if the vehicle's 120 s restart cannot run. Only
 *     then is the guard (exynos9810_audio_a1_guard()) opened: the USI07
 *     wrapper and the HSI2C18 controller of the amp bus call it first in
 *     their probe (DT samsung,audio-a1-guard) and defer until it opens, or
 *     fail for good if A1 refused. No deadline, no amp bus bring-up.
 *
 * One self-test, started from the DT (samsung,audio-a1-selftest-ms, ~30 s
 * uptime) on a CPU other than CPU0, that only reads. It pins itself to its
 * CPU (migrate_disable(): offlining that CPU waits for A1 to finish) and
 * refuses to touch anything if it finds itself on CPU0.
 *
 *  1. The watchdog deadline is still armed and the counter still moves.
 *  2. PMU (always on, syscon): pd-aud AUD_CONFIGURATION/AUD_STATUS and the
 *     ABOX CPU/pad-retention registers 4.9 abox.c reads or updates. Status
 *     only: nothing inside pd-aud is read (power on does not prove the
 *     block's clocks or its access path).
 *  3. GPIO CON/DAT/PUD of the codec IRQ (gpa2-2), codec reset (gpg1-0),
 *     codec SPI pins (gpp0-4..7), amp IRQs (gpm2-0, gpm3-0) and the amp I2C
 *     pins (gpp4-4/5). 6.12 gpiolib reads every bank's CON at boot.
 *  4. S2MPS18 L33CTRL 0x6a / L34CTRL 0x6b through the read-only ACPM PMIC
 *     path (s2mps18_acpm_read); the transport's write policy is sealed by
 *     the PMIC driver, this file has no PMIC write call.
 *  5. MAX98512 0x39 / 0x3a on HSI2C18: register-address write + repeated
 *     start + 1-byte read only (the normal read protocol; no data byte is
 *     ever sent), whitelist 0x0402/0x0600 (rev ID), 0x0400/0x0500
 *     (GLOBAL_SHDN, bit 0 = GLOBAL_EN), 0x0001-0x0003 (INT_RAW1-3, not
 *     clear-on-read; the flags 0x0007-0x0009 are not read). Same order as
 *     4.9 max98512_i2c_probe(): 0x0402 first, 0x0600 only below 0x40.
 *     The first failed transfer ends all amp traffic for the boot (here and
 *     in the adapter, whose DT A1 mode also refuses any other message
 *     shape, stuck-bus recovery clocks and retries).
 *  6. Codec CS47L93: no SPI traffic. The silicon ID read needs USI01 in SPI
 *     mode, the SPI pins muxed, the reset released and DCVDD (LDO34) on;
 *     any of these missing would take a write to fix. Only the
 *     preconditions are evaluated, from the reads above and USI01 SW_CONF.
 *
 * Nothing here writes MMIO, the PMIC, the codec or the amps; the amp
 * driver is never bound (the A1 DT has no amp node). Writes that do happen
 * in an A1 boot belong to other drivers: the USI07 wrapper / PERIC1 SYSREG
 * SW_CONF and the HSI2C18 controller (exynos-usi, i2c-exynos5, as for the
 * touch bus) and the gpp4-4/5 pin mux (pinctrl "default"), all after the
 * guard opened.
 */

#define pr_fmt(fmt) "audio selftest A1: " fmt

#include <linux/atomic.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/mfd/samsung/s2mps18-acpm.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/preempt.h>
#include <linux/regmap.h>
#include <linux/smp.h>
#include <linux/workqueue.h>
#include <soc/samsung/acpm_mfd.h>
#include <soc/samsung/crownlte-wdt.h>
#include <soc/samsung/exynos9810-audio-a1.h>

/* PMU (0x14060000) offsets, K49 flexpmu_cal_local_exynos9810.h and abox.c:81-93 */
#define PMU_AUD_CONFIGURATION		0x4000
#define PMU_AUD_STATUS			0x4004
#define PMU_AUD_STATUS_ON		0xf
#define PMU_ABOX_CPU_CONFIGURATION	0x415c
#define PMU_ABOX_CPU_STATUS		0x4160
#define PMU_ABOX_CPU_OPTION		0x4164
#define PMU_ABOX_CPU_STANDBY		0x3804
#define PMU_PAD_RETENTION_ABOX_OPTION	0x4170
#define PMU_GPIO_MODE_ABOX_SYS_PWR_REG	0x1424

/* S2MPS18 PMIC block, K49 s2mps18-private.h / regulator/s2mps18.c */
#define S2MPS18_L33CTRL			0x6a
#define S2MPS18_L34CTRL			0x6b
#define S2MPS18_LDO_EN_SHIFT		6
#define S2MPS18_LDO_VSEL_MASK		0x3f
#define S2MPS18_LDO_MIN_UV		700000
#define S2MPS18_L33_STEP_UV		25000	/* LDO33: _MIN1, _STEP2 */
#define S2MPS18_L34_STEP_UV		12500	/* LDO34: _MIN1, _STEP1 */
#define S2MPS18_L33_VSEL_1V8		0x2c
#define S2MPS18_L34_VSEL_1V2		0x28

/* MAX98512, K49 sound/soc/codecs/max98512.{c,h} */
#define MAX98512_INT_RAW1		0x0001
#define MAX98512_INT_RAW3		0x0003
#define MAX98512_GLOBAL_SHDN		0x0400
#define MAX98512_REV_ID			0x0402
#define MAX98512B_GLOBAL_SHDN		0x0500
#define MAX98512B_REV_ID		0x0600
#define MAX98512_REV_ID_0		0x40
#define MAX98512_GLOBAL_EN		BIT(0)

/* USIv2 SW_CONF (exynos-usi.c) */
#define USI_V2_SW_CONF_MASK		0x7
#define USI_V2_SW_CONF_SPI		BIT(1)

/* exynos850 bank layout: CON 4 bit/pin, DAT 1 bit/pin, PUD 4 bit/pin */
#define GPIO_CON			0x0
#define GPIO_DAT			0x4
#define GPIO_PUD			0x8

#define A1_START_MIN_MS			20000
#define A1_START_MAX_MS			100000
#define A1_DEADLINE_MIN_MS		121000
#define A1_DEADLINE_MAX_MS		180000
#define A1_WDT_LIVE_TRIES		20
/* the guard refuses a bring-up this close to (or past) the deadline */
#define A1_GUARD_MARGIN_MS		20000

enum a1_win {
	W_ALIVE,	/* pinctrl_alive 0x14050000: gpa2 */
	W_PERIC0,	/* pinctrl_peric0 0x10430000: gpp0, gpg1 */
	W_PERIC1,	/* pinctrl_peric1 0x10830000: gpp4 */
	W_CMGP,		/* pinctrl_cmgp 0x14220000: gpm2, gpm3 */
	W_USI01,	/* PERIC0 SYSREG USI01 SW_CONF 0x1041100c */
	W_NR,
};

static const char * const a1_win_names[W_NR] = {
	"pinctrl-alive", "pinctrl-peric0", "pinctrl-peric1", "pinctrl-cmgp",
	"usi01-swconf",
};

/* minimum window sizes, checked against the DT */
static const u32 a1_win_min[W_NR] = {
	0x80, 0xc0, 0x10, 0x80, 0x4,
};

struct a1_pin {
	const char *name;
	enum a1_win win;
	u32 bank;
	u8 pin;
};

static const struct a1_pin a1_pins[] = {
	{ "codec-irq gpa2-2",	W_ALIVE,  0x060, 2 },
	{ "codec-reset gpg1-0",	W_PERIC0, 0x0a0, 0 },
	{ "codec-spi-clk gpp0-4", W_PERIC0, 0x000, 4 },
	{ "codec-spi-miso gpp0-5", W_PERIC0, 0x000, 5 },
	{ "codec-spi-mosi gpp0-6", W_PERIC0, 0x000, 6 },
	{ "codec-spi-cs gpp0-7", W_PERIC0, 0x000, 7 },
	{ "amp-int gpm2-0",	W_CMGP,   0x040, 0 },
	{ "amp-int gpm3-0",	W_CMGP,   0x060, 0 },
	{ "amp-i2c-scl gpp4-4",	W_PERIC1, 0x000, 4 },
	{ "amp-i2c-sda gpp4-5",	W_PERIC1, 0x000, 5 },
};

/* indexes into a1_pins[] */
enum {
	P_CODEC_IRQ, P_CODEC_RST, P_SPI0, P_SPI3 = P_SPI0 + 3,
	P_AMP_INT0, P_AMP_INT1, P_I2C_SCL, P_I2C_SDA,
};

struct a1_pin_val {
	u32 con, dat, pud;	/* field values for the pin */
};

struct a1_amp {
	u16 addr;
	int err;		/* first I2C error, 0 if every read ACKed */
	bool read, rev_b, rev_known;
	u8 rev, shdn, raw[3];
};

/* guard states, published with smp_store_release() */
enum a1_state {
	A1_PENDING,	/* not probed yet (or waiting for the watchdog): defer */
	A1_ARMED,	/* deadline installed and verified: bring-up allowed */
	A1_REFUSED,	/* no deadline, no nonzero CPU or bad DT: never */
};

static struct {
	void __iomem *win[W_NR];
	struct regmap *pmu;
	struct device_node *np, *pmic_np, *bus_np;
	u32 start_ms, deadline_ms;
	unsigned int cpu;		/* nonzero CPU the self-test is queued on */
	struct delayed_work work;
	struct hrtimer deadline;
	bool hrtimer_on;
	const char *why;		/* reason for A1_REFUSED / wdt failure */
	int state;			/* enum a1_state */
} a1;

/* ---- watchdog deadline (pattern of exynos9810-pm.c) ---------------------- */

static enum hrtimer_restart a1_deadline_fn(struct hrtimer *t)
{
	pr_emerg("deadline %u ms reached and the system is still up (the vehicle restarts at 120 s): stopping watchdog petting\n",
		 a1.deadline_ms);
	s3c2410wdt_crownlte_stop_petting("audio A1 deadline");
	return HRTIMER_NORESTART;
}

static bool a1_wdt_state_ok(const struct crownlte_wdt_state *s)
{
	return s->enabled && s->reset && !s->pet_stopped && s->deadline_ns &&
	       s->deadline_ns <= (u64)a1.deadline_ms * NSEC_PER_MSEC;
}

/* NULL if the deadline is in the ping path of a live, petted, resetting WDT */
static const char *a1_wdt_verify(const char *when)
{
	struct crownlte_wdt_state a, b;
	unsigned int i;

	if (!s3c2410wdt_crownlte_state(&a))
		return "no guarded CL0 watchdog";
	if (!a.enabled || !a.reset)
		return "CL0 watchdog not running with the reset enabled";
	if (a.pet_stopped)
		return "CL0 watchdog petting stopped or deadline passed";
	if (!a1_wdt_state_ok(&a))
		return "watchdog deadline not installed";
	for (i = 0; i < A1_WDT_LIVE_TRIES; i++) {
		usleep_range(2000, 3000);
		if (!s3c2410wdt_crownlte_state(&b) || !a1_wdt_state_ok(&b))
			return "CL0 watchdog stopped while being checked";
		if (b.wtcnt != a.wtcnt) {
			pr_info("wdt (%s): deadline %llu ms armed (WTCON=%#x timeout %u s, counter moving)\n",
				when, div_u64(b.deadline_ns, NSEC_PER_MSEC), b.wtcon,
				b.timeout);
			return NULL;
		}
	}
	return "CL0 watchdog counter not moving";
}

/* ---- guard for the amp bus (USI07 wrapper, HSI2C18) ------------------------ */

/*
 * Called first thing in the probe of a device whose DT node has
 * samsung,audio-a1-guard = <&audio_a1>, before it touches SYSREG, clocks or
 * registers. 0: A1 armed and verified the CL0 watchdog deadline and it is
 * still armed; -EPROBE_DEFER: A1 has not got that far yet; any other error:
 * A1 refused, the device must not come up in this boot.
 */
int exynos9810_audio_a1_guard(struct device *consumer)
{
	struct crownlte_wdt_state st;
	struct device_node *np;
	bool ok_node;
	u64 now;
	int state;

	np = of_parse_phandle(consumer->of_node, "samsung,audio-a1-guard", 0);
	if (!np) {
		dev_err(consumer, "audio A1 guard: no samsung,audio-a1-guard phandle: bring-up refused\n");
		return -EINVAL;
	}
	ok_node = of_device_is_compatible(np, "samsung,exynos9810-audio-a1") &&
		  of_device_is_available(np);
	if (!ok_node)
		dev_err(consumer, "audio A1 guard: %pOF is not an enabled audio A1 node: bring-up refused\n",
			np);
	of_node_put(np);	/* only compared as a pointer below */
	if (!ok_node)
		return -EINVAL;

	state = smp_load_acquire(&a1.state);
	if (state == A1_PENDING)
		return -EPROBE_DEFER;
	if (state != A1_ARMED || np != a1.np) {
		dev_err(consumer, "audio A1 guard: A1 refused (%s): bring-up refused for this boot\n",
			state != A1_ARMED ? a1.why : "other A1 node");
		return -EPERM;
	}
	if (!s3c2410wdt_crownlte_state(&st) || !a1_wdt_state_ok(&st)) {
		dev_err(consumer, "audio A1 guard: CL0 watchdog deadline no longer armed: bring-up refused\n");
		return -EPERM;
	}
	now = ktime_get_boottime_ns();
	if (now + (u64)A1_GUARD_MARGIN_MS * NSEC_PER_MSEC >= st.deadline_ns) {
		dev_err(consumer, "audio A1 guard: %llu ms is within %u ms of the deadline: bring-up refused\n",
			div_u64(now, NSEC_PER_MSEC), A1_GUARD_MARGIN_MS);
		return -EPERM;
	}
	dev_info(consumer, "audio A1 guard: CL0 watchdog deadline %llu ms armed, bring-up allowed at %llu ms\n",
		 div_u64(st.deadline_ns, NSEC_PER_MSEC), div_u64(now, NSEC_PER_MSEC));
	return 0;
}
EXPORT_SYMBOL_GPL(exynos9810_audio_a1_guard);

/* ---- always-on reads ------------------------------------------------------ */

struct a1_pmu {
	int err;
	u32 aud_cfg, aud_status, cpu_cfg, cpu_status, cpu_option, cpu_standby,
	    pad_ret, gpio_mode;
};

static int a1_pmu_rd(u32 off, u32 *v)
{
	return regmap_read(a1.pmu, off, v);
}

static const char *a1_pd_aud(const struct a1_pmu *p)
{
	if (p->err)
		return "?";
	if ((p->aud_status & 0xf) == PMU_AUD_STATUS_ON)
		return "on";
	return (p->aud_status & 0xf) ? "partial" : "off";
}

static void a1_read_pmu(struct a1_pmu *p)
{
	p->err = a1_pmu_rd(PMU_AUD_CONFIGURATION, &p->aud_cfg) ?:
		 a1_pmu_rd(PMU_AUD_STATUS, &p->aud_status) ?:
		 a1_pmu_rd(PMU_ABOX_CPU_CONFIGURATION, &p->cpu_cfg) ?:
		 a1_pmu_rd(PMU_ABOX_CPU_STATUS, &p->cpu_status) ?:
		 a1_pmu_rd(PMU_ABOX_CPU_OPTION, &p->cpu_option) ?:
		 a1_pmu_rd(PMU_ABOX_CPU_STANDBY, &p->cpu_standby) ?:
		 a1_pmu_rd(PMU_PAD_RETENTION_ABOX_OPTION, &p->pad_ret) ?:
		 a1_pmu_rd(PMU_GPIO_MODE_ABOX_SYS_PWR_REG, &p->gpio_mode);
	if (p->err) {
		pr_info("pmu: read FAILED %d\n", p->err);
		return;
	}
	pr_info("pmu: AUD_CONFIGURATION=%#x AUD_STATUS=%#x (pd-aud %s, status only: nothing inside pd-aud is read) ABOX_CPU_CONFIGURATION=%#x ABOX_CPU_STATUS=%#x (cpu %s) ABOX_CPU_OPTION=%#x ABOX_CPU_STANDBY=%#x PAD_RETENTION_ABOX_OPTION=%#x GPIO_MODE_ABOX_SYS_PWR_REG=%#x\n",
		p->aud_cfg, p->aud_status, a1_pd_aud(p),
		p->cpu_cfg, p->cpu_status, p->cpu_status & 1 ? "on" : "off",
		p->cpu_option, p->cpu_standby, p->pad_ret, p->gpio_mode);
}

static void a1_read_pin(const struct a1_pin *p, struct a1_pin_val *v)
{
	void __iomem *b = a1.win[p->win] + p->bank;

	v->con = (readl(b + GPIO_CON) >> (p->pin * 4)) & 0xf;
	v->dat = (readl(b + GPIO_DAT) >> p->pin) & 1;
	v->pud = (readl(b + GPIO_PUD) >> (p->pin * 4)) & 0xf;
}

static void a1_read_pins(struct a1_pin_val *v)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(a1_pins); i++) {
		a1_read_pin(&a1_pins[i], &v[i]);
		pr_info("gpio: %s con=%#x (%s) dat=%u pud=%#x\n", a1_pins[i].name,
			v[i].con, v[i].con == 0 ? "input" : v[i].con == 1 ? "output" :
			v[i].con == 0xf ? "eint" : "func", v[i].dat, v[i].pud);
	}
}

struct a1_ldo {
	int err;
	u8 ctrl;
};

static const char *a1_ldo_state(u8 ctrl)
{
	switch (ctrl >> S2MPS18_LDO_EN_SHIFT) {
	case 3:
		return "on";
	case 0:
		return "off";
	default:
		return "en-mode-other";
	}
}

static void a1_read_pmic(struct s2mps18_acpm *pmic, struct a1_ldo *l33,
			 struct a1_ldo *l34)
{
	l33->err = s2mps18_acpm_read(pmic, ACPM_MFD_TYPE_PMIC, S2MPS18_L33CTRL,
				     &l33->ctrl);
	l34->err = s2mps18_acpm_read(pmic, ACPM_MFD_TYPE_PMIC, S2MPS18_L34CTRL,
				     &l34->ctrl);
	if (l33->err)
		pr_info("pmic: L33CTRL read FAILED %d\n", l33->err);
	else
		pr_info("pmic: L33CTRL[0x6a]=%#04x en=%u vsel=%#04x -> %u uV, %s (codec AVDD/DBVDD1/CPVDD1, 4.9: 1.8 V always-on)\n",
			l33->ctrl, l33->ctrl >> S2MPS18_LDO_EN_SHIFT,
			l33->ctrl & S2MPS18_LDO_VSEL_MASK,
			S2MPS18_LDO_MIN_UV + (l33->ctrl & S2MPS18_LDO_VSEL_MASK) *
					     S2MPS18_L33_STEP_UV,
			a1_ldo_state(l33->ctrl));
	if (l34->err)
		pr_info("pmic: L34CTRL read FAILED %d\n", l34->err);
	else
		pr_info("pmic: L34CTRL[0x6b]=%#04x en=%u vsel=%#04x -> %u uV, %s (codec DCVDD/CPVDD2, 4.9: 1.2 V, enabled by madera)\n",
			l34->ctrl, l34->ctrl >> S2MPS18_LDO_EN_SHIFT,
			l34->ctrl & S2MPS18_LDO_VSEL_MASK,
			S2MPS18_LDO_MIN_UV + (l34->ctrl & S2MPS18_LDO_VSEL_MASK) *
					     S2MPS18_L34_STEP_UV,
			a1_ldo_state(l34->ctrl));
}

/*
 * MAX98512 register read: a 2-byte register-address message, a repeated
 * start, a 1-byte read. The address message carries no data byte, so the
 * amp only moves its register pointer. Anything off the whitelist is
 * refused before the bus is touched. The adapter's A1 mode enforces the
 * same message shape and closes itself after the first failure.
 */
static int a1_amp_read(struct i2c_adapter *adap, u16 addr, u16 reg, u8 *val)
{
	u8 ra[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[2] = {
		{ .addr = addr, .flags = 0, .len = sizeof(ra), .buf = ra },
		{ .addr = addr, .flags = I2C_M_RD, .len = 1, .buf = val },
	};
	int ret;

	switch (reg) {
	case MAX98512_INT_RAW1 ... MAX98512_INT_RAW3:
	case MAX98512_GLOBAL_SHDN:
	case MAX98512_REV_ID:
	case MAX98512B_GLOBAL_SHDN:
	case MAX98512B_REV_ID:
		break;
	default:
		pr_err("amp %#04x: register %#06x is not on the A1 read whitelist, refused\n",
		       addr, reg);
		return -EPERM;
	}
	ret = i2c_transfer(adap, msgs, ARRAY_SIZE(msgs));
	if (ret == ARRAY_SIZE(msgs))
		return 0;
	return ret < 0 ? ret : -EIO;
}

/* Stops at the first failed transfer; the caller then closes the bus. */
static void a1_read_amp(struct i2c_adapter *adap, struct a1_amp *amp)
{
	u16 shdn_reg;
	int i, ret;

	amp->read = true;
	/* 4.9 max98512_i2c_probe(): rev A ID at 0x0402, else rev B at 0x0600 */
	ret = a1_amp_read(adap, amp->addr, MAX98512_REV_ID, &amp->rev);
	if (!ret && amp->rev >= MAX98512_REV_ID_0) {
		amp->rev_known = true;
	} else if (!ret) {
		ret = a1_amp_read(adap, amp->addr, MAX98512B_REV_ID, &amp->rev);
		if (!ret && amp->rev >= MAX98512_REV_ID_0)
			amp->rev_known = amp->rev_b = true;
	}
	if (ret) {
		amp->err = ret;
		pr_info("amp %#04x: rev ID read FAILED %d (%s)\n", amp->addr, ret,
			ret == -ENXIO || ret == -EREMOTEIO ? "NACK" :
			ret == -ETIMEDOUT ? "bus timeout" :
			ret == -EAGAIN ? "arbitration lost" :
			ret == -EBUSY ? "bus busy or stuck" : "error");
		return;
	}
	if (!amp->rev_known) {
		pr_info("amp %#04x: rev ID %#04x below %#x at 0x0402 and 0x0600: unknown silicon, GLOBAL_SHDN not read\n",
			amp->addr, amp->rev, MAX98512_REV_ID_0);
		return;
	}
	shdn_reg = amp->rev_b ? MAX98512B_GLOBAL_SHDN : MAX98512_GLOBAL_SHDN;
	ret = a1_amp_read(adap, amp->addr, shdn_reg, &amp->shdn);
	for (i = 0; !ret && i < 3; i++)
		ret = a1_amp_read(adap, amp->addr, MAX98512_INT_RAW1 + i,
				  &amp->raw[i]);
	if (ret) {
		amp->err = ret;
		pr_info("amp %#04x: rev %#04x (%s) then read FAILED %d\n", amp->addr,
			amp->rev, amp->rev_b ? "B" : "A", ret);
		return;
	}
	pr_info("amp %#04x: rev %#04x (rev %s, ID at %#06x) GLOBAL_SHDN[%#06x]=%#04x GLOBAL_EN=%u INT_RAW1-3=%#04x %#04x %#04x\n",
		amp->addr, amp->rev, amp->rev_b ? "B" : "A",
		amp->rev_b ? MAX98512B_REV_ID : MAX98512_REV_ID, shdn_reg,
		amp->shdn, !!(amp->shdn & MAX98512_GLOBAL_EN),
		amp->raw[0], amp->raw[1], amp->raw[2]);
}

/*
 * Codec SPI ID read (CS47L93 0x6371 at register 0): only the preconditions.
 * Returns a short verdict for the summary line.
 */
static const char *a1_codec_spi(const struct a1_ldo *l33, const struct a1_ldo *l34,
				const struct a1_pin_val *pv)
{
	u32 sw_conf = readl(a1.win[W_USI01]);
	bool spi_mode = (sw_conf & USI_V2_SW_CONF_MASK) == USI_V2_SW_CONF_SPI;
	bool l33_on = !l33->err && (l33->ctrl >> S2MPS18_LDO_EN_SHIFT) == 3 &&
		      (l33->ctrl & S2MPS18_LDO_VSEL_MASK) == S2MPS18_L33_VSEL_1V8;
	bool l34_on = !l34->err && (l34->ctrl >> S2MPS18_LDO_EN_SHIFT) == 3 &&
		      (l34->ctrl & S2MPS18_LDO_VSEL_MASK) == S2MPS18_L34_VSEL_1V2;
	bool rst_released = pv[P_CODEC_RST].con == 1 && pv[P_CODEC_RST].dat == 1;
	bool pins = true;
	const char *why;
	int i;

	for (i = P_SPI0; i <= P_SPI3; i++)
		pins &= pv[i].con == 2;

	pr_info("codec: USI01 SW_CONF[0x1041100c]=%#x (%s) spi-pins=%s reset gpg1-0 %s L33=%s L34=%s\n",
		sw_conf, spi_mode ? "spi" : "not spi", pins ? "func2" : "not muxed",
		rst_released ? "released (output high)" : "asserted or not driven",
		l33_on ? "1.8V on" : "not on", l34_on ? "1.2V on" : "not on");

	if (!l33_on)
		why = "skip(l33-off)";
	else if (!l34_on)
		why = "skip(l34-off)";
	else if (!rst_released)
		why = "skip(reset)";
	else if (!spi_mode)
		why = "skip(usi01-not-spi)";
	else if (!pins)
		why = "skip(pins)";
	else
		why = "skip(ready-no-a1-spi)";
	pr_info("codec: no SPI transfer in A1 (%s); bringing any missing precondition up is a write: A3 [G1]\n",
		why);
	return why;
}

/* ---- self-test ------------------------------------------------------------ */

static const char *a1_amp_rev(const struct a1_amp *amp)
{
	if (!amp->read)
		return "(unread)";
	if (!amp->rev_known)
		return "(?)";
	return amp->rev_b ? "(B)" : "(A)";
}

/* Runs pinned (migrate_disable()) on the nonzero CPU it was queued on. */
static void a1_selftest(unsigned int cpu)
{
	struct a1_amp amps[2] = { { .addr = 0x39 }, { .addr = 0x3a } };
	struct a1_pin_val pv[ARRAY_SIZE(a1_pins)];
	struct s2mps18_acpm *pmic = NULL;
	struct platform_device *pdev;
	struct i2c_adapter *adap;
	struct a1_ldo l33 = { .err = -ENODEV }, l34 = { .err = -ENODEV };
	struct a1_pmu pmu = { };
	char fails[160] = "";
	const char *wdt_why, *spi, *verdict;
	bool amp_on = false, bus_err = false;
	int pmic_w0 = 0, pmic_writes = 0, i;

#define A1_FAIL(s) strlcat(fails, " " s, sizeof(fails))

	pr_info("start uptime=%lld ms on CPU%u (pinned until the end of A1), read-only (no MMIO/PMIC/I2C data writes, no SPI, nothing inside pd-aud)\n",
		ktime_to_ms(ktime_get_boottime()), cpu);

	/* 1. recovery first: the deadline armed at probe must still hold */
	wdt_why = a1.why ?: a1_wdt_verify("self-test");
	if (wdt_why) {
		pr_info("wdt: NOT armed (%s): amp reads skipped\n", wdt_why);
		A1_FAIL("wdt");
	}

	/* 2. PMU */
	a1_read_pmu(&pmu);
	if (pmu.err)
		A1_FAIL("pmu");

	/* 3. GPIO */
	a1_read_pins(pv);
	pr_info("jack: detection is the codec's own JD2/MICDET (cirrus,accdet), no AP GPIO; the codec IRQ gpa2-2 above is its only AP-visible line (dat=%u)\n",
		pv[P_CODEC_IRQ].dat);

	/* 4. PMIC */
	pdev = a1.pmic_np ? of_find_device_by_node(a1.pmic_np) : NULL;
	if (pdev) {
		pmic = platform_get_drvdata(pdev);
		if (pmic) {
			pmic_w0 = atomic_read(&pmic->writes);
			a1_read_pmic(pmic, &l33, &l34);
			pmic_writes = atomic_read(&pmic->writes) - pmic_w0;
		}
		put_device(&pdev->dev);
	}
	if (!pmic) {
		pr_info("pmic: S2MPS18 ACPM driver not bound: L33/L34 not read\n");
		A1_FAIL("pmic-unbound");
	} else if (l33.err || l34.err) {
		A1_FAIL("pmic-read");
	} else if ((l33.ctrl >> S2MPS18_LDO_EN_SHIFT) != 3) {
		A1_FAIL("l33-off");
	}

	/* 5. amps: only behind a live deadline; the first failure ends the bus */
	adap = !wdt_why && a1.bus_np ? of_get_i2c_adapter_by_node(a1.bus_np) : NULL;
	if (wdt_why) {
		A1_FAIL("amp-skipped");
	} else if (!adap) {
		pr_info("amp: HSI2C18 (0x108c0000) adapter not registered: amps not read\n");
		A1_FAIL("amp-bus");
	} else {
		for (i = 0; i < ARRAY_SIZE(amps); i++) {
			if (bus_err) {
				pr_info("amp %#04x: not read: amp bus closed after the failure above (no retry, no recovery)\n",
					amps[i].addr);
				continue;
			}
			a1_read_amp(adap, &amps[i]);
			if (amps[i].err)
				bus_err = true;
			else if (amps[i].rev_known &&
				 (amps[i].shdn & MAX98512_GLOBAL_EN))
				amp_on = true;
		}
		for (i = 0; i < ARRAY_SIZE(amps); i++)
			if (amps[i].err || !amps[i].rev_known) {
				A1_FAIL("amp-read");
				break;
			}
		i2c_put_adapter(adap);
	}

	/* 6. codec */
	spi = a1_codec_spi(&l33, &l34, pv);

	if (pmic_writes)
		A1_FAIL("pmic-writes");
	if (amp_on)
		verdict = "STOP";	/* warm amp: report only, owner decides */
	else
		verdict = fails[0] ? "FAIL" : "PASS";

	pr_info("%s pd_aud=%s(status) abox_cpu=%s l33=%#04x (%s) l34=%#04x (%s) amp39 rev=%#04x%s en=%u amp3a rev=%#04x%s en=%u codec_spi=%s jack=codec-jd2 wdt=%s cpu=%u writes=%u%s%s\n",
		verdict, a1_pd_aud(&pmu),
		pmu.err ? "?" : pmu.cpu_status & 1 ? "on" : "off",
		l33.ctrl, l33.err ? "unread" : a1_ldo_state(l33.ctrl),
		l34.ctrl, l34.err ? "unread" : a1_ldo_state(l34.ctrl),
		amps[0].rev, a1_amp_rev(&amps[0]),
		!!(amps[0].shdn & MAX98512_GLOBAL_EN),
		amps[1].rev, a1_amp_rev(&amps[1]),
		!!(amps[1].shdn & MAX98512_GLOBAL_EN), spi,
		wdt_why ? "no" : "armed", cpu, pmic_writes,
		fails[0] ? " fails:" : "", fails);
	if (amp_on)
		pr_info("STOP: an amp has GLOBAL_EN=1 left from before this kernel; nothing was written, owner decides (audio-design.md A1)\n");
#undef A1_FAIL
}

static void a1_work_fn(struct work_struct *work)
{
	unsigned int cpu;

	/*
	 * Pin to the CPU we run on: from here until migrate_enable() this
	 * task cannot move, and offlining this CPU waits for it (the
	 * scheduler's hotplug path waits for pinned tasks). If the queued CPU
	 * went offline before the work ran, the workqueue ran it unbound: it
	 * may be anywhere, including CPU0, so check before any access.
	 */
	migrate_disable();
	cpu = smp_processor_id();
	if (!cpu)
		pr_info("FAIL: self-test found itself on CPU0 (queued on CPU%u, offline?): refused before any hardware access (the vehicle's PID1 restart timer lives on CPU0) fails: cpu0\n",
			a1.cpu);
	else
		a1_selftest(cpu);
	migrate_enable();
}

/* ---- probe ------------------------------------------------------------------ */

static int a1_map(struct platform_device *pdev)
{
	struct resource *r;
	int i;

	for (i = 0; i < W_NR; i++) {
		r = platform_get_resource_byname(pdev, IORESOURCE_MEM, a1_win_names[i]);
		if (!r || resource_size(r) < a1_win_min[i]) {
			dev_err(&pdev->dev, "reg \"%s\" missing or smaller than %#x\n",
				a1_win_names[i], a1_win_min[i]);
			return -EINVAL;
		}
		/* no request_mem_region: the owning drivers keep their claims */
		a1.win[i] = devm_ioremap(&pdev->dev, r->start, resource_size(r));
		if (!a1.win[i])
			return -ENOMEM;
	}
	return 0;
}

/* Bound but idle: no self-test, the guard refuses the amp bus for good. */
static int a1_refuse(struct device *dev, const char *why)
{
	a1.why = why;
	smp_store_release(&a1.state, A1_REFUSED);
	dev_err(dev, "audio A1 refused (%s): no self-test, amp bus bring-up refused\n",
		why);
	return 0;
}

static int a1_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	s64 now, delay;
	int ret;

	if (a1.np && a1.np != np)
		return dev_err_probe(dev, -EBUSY, "only one audio A1 node is supported\n");
	a1.np = np;

	if (of_property_read_u32(np, "samsung,audio-a1-selftest-ms", &a1.start_ms))
		return a1_refuse(dev, "no samsung,audio-a1-selftest-ms");
	if (a1.start_ms < A1_START_MIN_MS || a1.start_ms > A1_START_MAX_MS)
		return a1_refuse(dev, "samsung,audio-a1-selftest-ms out of range");
	if (of_property_read_u32(np, "samsung,audio-a1-deadline-ms", &a1.deadline_ms) ||
	    a1.deadline_ms < A1_DEADLINE_MIN_MS || a1.deadline_ms > A1_DEADLINE_MAX_MS)
		return a1_refuse(dev, "samsung,audio-a1-deadline-ms missing or out of range");

	/*
	 * Not on CPU0: a CPU wedged on a read must leave the vehicle's
	 * CPU0-pinned PID1 restart timer running. No other CPU: fail closed.
	 */
	a1.cpu = cpumask_next(0, cpu_online_mask);
	if (a1.cpu >= nr_cpu_ids)
		return a1_refuse(dev, "no CPU other than CPU0 online");

	a1.pmu = syscon_regmap_lookup_by_phandle(np, "samsung,pmu-syscon");
	if (a1.pmu == ERR_PTR(-EPROBE_DEFER))
		return dev_err_probe(dev, -EPROBE_DEFER, "PMU syscon\n");
	if (IS_ERR(a1.pmu))
		return a1_refuse(dev, "PMU syscon not found");
	ret = a1_map(pdev);
	if (ret)
		return a1_refuse(dev, "DT reg windows");
	of_node_put(a1.pmic_np);
	of_node_put(a1.bus_np);
	a1.pmic_np = of_parse_phandle(np, "samsung,pmic", 0);
	a1.bus_np = of_parse_phandle(np, "samsung,amp-i2c-bus", 0);

	/* The deadline goes in before the amp bus may come up. */
	ret = s3c2410wdt_crownlte_set_deadline((u64)a1.deadline_ms * NSEC_PER_MSEC);
	if (ret == -ENODEV)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "waiting for the guarded CL0 watchdog (amp bus held back)\n");
	if (ret) {
		a1.why = "watchdog refused the deadline";
	} else {
		if (!a1.hrtimer_on) {
			hrtimer_init(&a1.deadline, CLOCK_BOOTTIME, HRTIMER_MODE_ABS);
			a1.deadline.function = a1_deadline_fn;
			hrtimer_start(&a1.deadline, ms_to_ktime(a1.deadline_ms),
				      HRTIMER_MODE_ABS);
			a1.hrtimer_on = true;
		}
		a1.why = a1_wdt_verify("probe");
	}
	if (a1.why) {
		/* self-test still runs (always-on reads), the amp bus never comes up */
		smp_store_release(&a1.state, A1_REFUSED);
		dev_err(dev, "watchdog deadline NOT armed (%s): amp bus bring-up refused\n",
			a1.why);
	} else {
		smp_store_release(&a1.state, A1_ARMED);
	}

	INIT_DELAYED_WORK(&a1.work, a1_work_fn);
	now = ktime_to_ms(ktime_get_boottime());
	delay = a1.start_ms > now ? a1.start_ms - now : 0;
	queue_delayed_work_on(a1.cpu, system_long_wq, &a1.work, msecs_to_jiffies(delay));
	dev_info(dev, "audio A1 read-only probe scheduled at ~%u ms on CPU%u, watchdog deadline %u ms %s, amp bus guard %s\n",
		 a1.start_ms, a1.cpu, a1.deadline_ms, a1.why ? "NOT armed" : "armed",
		 a1.why ? "closed" : "open");
	return 0;
}

static const struct of_device_id a1_of_match[] = {
	{ .compatible = "samsung,exynos9810-audio-a1" },
	{ }
};

static struct platform_driver a1_driver = {
	.driver = {
		.name = "exynos9810-audio-a1",
		.of_match_table = a1_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = a1_probe,
};
builtin_platform_driver(a1_driver);
