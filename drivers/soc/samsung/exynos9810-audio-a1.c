// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 9810 (crownlte) audio stage A1: read-only hardware probe.
 *
 * Design: gekko roadmap/audio-design.md, section 5 "A1. Read-only probe".
 * Expected lines, failure signatures and the trial: Documentation/crownlte/audio-a1.md.
 *
 * One self-test, started from the DT (samsung,audio-a1-selftest-ms, ~30 s
 * uptime) on a CPU other than CPU0, that only reads:
 *
 *  1. CL0 watchdog deadline (samsung,audio-a1-deadline-ms) installed and
 *     verified first, as the suspend S0 driver does: after the deadline no
 *     CPU can reload the counter, so a CPU wedged on a bus read ends in a
 *     watchdog reset even if the vehicle's 120 s restart cannot run.
 *  2. PMU (always on, syscon): pd-aud AUD_CONFIGURATION/AUD_STATUS and the
 *     ABOX CPU/pad-retention registers 4.9 abox.c reads or updates.
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
 *  6. Codec CS47L93: no SPI traffic. The silicon ID read needs USI01 in SPI
 *     mode, the SPI pins muxed, the reset released and DCVDD (LDO34) on;
 *     any of these missing would take a write to fix. Only the
 *     preconditions are evaluated, from the reads above and USI01 SW_CONF.
 *  7. Only if AUD_STATUS[3:0] == 0xf (pd-aud on) and the deadline is armed:
 *     pinctrl_aud gpb0..gpb2, SYSREG_AUD MISC_CON, ABOX SFR IP_INDEX /
 *     VERSION / SYSPOWER_CTRL / SYSPOWER_STATUS, ABOX SysMMU CTRL / VERSION,
 *     then AUD_STATUS again. Each block is announced before it is read, so
 *     a stall names its block.
 *
 * Nothing here writes MMIO, the PMIC, the codec or the amps; the amp
 * driver is never bound (the A1 DT has no amp node). Writes that do happen
 * in an A1 boot belong to other drivers: the USI07 wrapper / PERIC1 SYSREG
 * SW_CONF and the HSI2C18 controller (exynos-usi, i2c-exynos5, as for the
 * touch bus) and the gpp4-4/5 pin mux (pinctrl "default").
 */

#define pr_fmt(fmt) "audio selftest A1: " fmt

#include <linux/atomic.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/mfd/samsung/s2mps18-acpm.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
#include <soc/samsung/acpm_mfd.h>
#include <soc/samsung/crownlte-wdt.h>

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

/* ABOX SFR (K49 abox.h:27-30) and SysMMU (K49 exynos-iommu.h:100-103) */
#define ABOX_IP_INDEX			0x00
#define ABOX_VERSION			0x04
#define ABOX_SYSPOWER_CTRL		0x10
#define ABOX_SYSPOWER_STATUS		0x14
#define SYSMMU_CTRL			0x00
#define SYSMMU_VERSION			0x34

/* exynos850 bank layout: CON 4 bit/pin, DAT 1 bit/pin, PUD 4 bit/pin */
#define GPIO_CON			0x0
#define GPIO_DAT			0x4
#define GPIO_PUD			0x8

#define A1_START_MIN_MS			20000
#define A1_START_MAX_MS			100000
#define A1_DEADLINE_MIN_MS		121000
#define A1_DEADLINE_MAX_MS		180000
#define A1_WDT_LIVE_TRIES		20

enum a1_win {
	W_ALIVE,	/* pinctrl_alive 0x14050000: gpa2 */
	W_PERIC0,	/* pinctrl_peric0 0x10430000: gpp0, gpg1 */
	W_PERIC1,	/* pinctrl_peric1 0x10830000: gpp4 */
	W_CMGP,		/* pinctrl_cmgp 0x14220000: gpm2, gpm3 */
	W_USI01,	/* PERIC0 SYSREG USI01 SW_CONF 0x1041100c */
	W_AUD_PIN,	/* pd-aud: pinctrl_aud 0x17c60000 */
	W_AUD_SYSREG,	/* pd-aud: SYSREG_AUD MISC_CON 0x17c1032c */
	W_ABOX,		/* pd-aud: ABOX SFR 0x17c50000 */
	W_SYSMMU,	/* pd-aud: ABOX SysMMU 0x17e00000 */
	W_NR,
};

static const char * const a1_win_names[W_NR] = {
	"pinctrl-alive", "pinctrl-peric0", "pinctrl-peric1", "pinctrl-cmgp",
	"usi01-swconf", "pinctrl-aud", "sysreg-aud-misc", "abox-sfr",
	"sysmmu-abox",
};

/* minimum window sizes, checked against the DT */
static const u32 a1_win_min[W_NR] = {
	0x80, 0xc0, 0x10, 0x80, 0x4, 0x50, 0x4, 0x18, 0x38,
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
	bool rev_b, rev_known;
	u8 rev, shdn, raw[3];
};

static struct {
	void __iomem *win[W_NR];
	struct regmap *pmu;
	struct device_node *pmic_np, *bus_np;
	u32 start_ms, deadline_ms;
	struct delayed_work work;
	struct hrtimer deadline;
	bool wdt_armed;
} a1;

/* ---- watchdog deadline (pattern of exynos9810-pm.c) ---------------------- */

static enum hrtimer_restart a1_deadline_fn(struct hrtimer *t)
{
	pr_emerg("deadline %u ms reached and the system is still up (the vehicle restarts at 120 s): stopping watchdog petting\n",
		 a1.deadline_ms);
	s3c2410wdt_crownlte_stop_petting("audio A1 deadline");
	return HRTIMER_NORESTART;
}

/* NULL if the deadline is in the ping path of a live, petted, resetting WDT */
static const char *a1_wdt_arm(void)
{
	struct crownlte_wdt_state a, b;
	unsigned int i;
	int ret;

	ret = s3c2410wdt_crownlte_set_deadline((u64)a1.deadline_ms * NSEC_PER_MSEC);
	if (ret == -ENODEV)
		return "no guarded CL0 watchdog";
	if (ret)
		return "watchdog refused the deadline";
	hrtimer_init(&a1.deadline, CLOCK_BOOTTIME, HRTIMER_MODE_ABS);
	a1.deadline.function = a1_deadline_fn;
	hrtimer_start(&a1.deadline, ms_to_ktime(a1.deadline_ms), HRTIMER_MODE_ABS);

	if (!s3c2410wdt_crownlte_state(&a))
		return "no guarded CL0 watchdog";
	if (!a.enabled || !a.reset)
		return "CL0 watchdog not running with the reset enabled";
	if (a.pet_stopped)
		return "CL0 watchdog petting already stopped";
	if (!a.deadline_ns || a.deadline_ns > (u64)a1.deadline_ms * NSEC_PER_MSEC)
		return "watchdog deadline not installed";
	for (i = 0; i < A1_WDT_LIVE_TRIES; i++) {
		usleep_range(2000, 3000);
		if (!s3c2410wdt_crownlte_state(&b) || !b.enabled || !b.reset ||
		    b.pet_stopped)
			return "CL0 watchdog stopped while being checked";
		if (b.wtcnt != a.wtcnt) {
			pr_info("wdt: deadline %u ms armed (WTCON=%#x timeout %u s, counter moving)\n",
				a1.deadline_ms, b.wtcon, b.timeout);
			return NULL;
		}
	}
	return "CL0 watchdog counter not moving";
}

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
	pr_info("pmu: AUD_CONFIGURATION=%#x AUD_STATUS=%#x (pd-aud %s) ABOX_CPU_CONFIGURATION=%#x ABOX_CPU_STATUS=%#x (cpu %s) ABOX_CPU_OPTION=%#x ABOX_CPU_STANDBY=%#x PAD_RETENTION_ABOX_OPTION=%#x GPIO_MODE_ABOX_SYS_PWR_REG=%#x\n",
		p->aud_cfg, p->aud_status,
		(p->aud_status & 0xf) == PMU_AUD_STATUS_ON ? "on" :
		(p->aud_status & 0xf) ? "partial" : "off",
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
 * refused before the bus is touched.
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

static void a1_read_amp(struct i2c_adapter *adap, struct a1_amp *amp)
{
	u16 shdn_reg;
	int i, ret;

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
			ret == -ETIMEDOUT ? "bus timeout" : "error");
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

/* ---- pd-aud reads ---------------------------------------------------------- */

static void a1_read_pd_aud(bool *still_on)
{
	void __iomem *b;
	u32 v[3], st;
	int i;

	pr_info("pd-aud: reading pinctrl_aud gpb0..gpb2 (0x17c60000)\n");
	for (i = 0; i < 3; i++) {
		b = a1.win[W_AUD_PIN] + i * 0x20;
		v[0] = readl(b + GPIO_CON);
		v[1] = readl(b + GPIO_DAT);
		v[2] = readl(b + GPIO_PUD);
		pr_info("pd-aud: gpb%d CON=%#010x DAT=%#04x PUD=%#010x\n", i,
			v[0], v[1], v[2]);
	}

	pr_info("pd-aud: reading SYSREG_AUD MISC_CON (0x17c1032c)\n");
	pr_info("pd-aud: SYSREG_AUD MISC_CON=%#x\n", readl(a1.win[W_AUD_SYSREG]));

	pr_info("pd-aud: reading ABOX SFR (0x17c50000)\n");
	b = a1.win[W_ABOX];
	pr_info("pd-aud: ABOX IP_INDEX=%#x VERSION=%#x SYSPOWER_CTRL=%#x SYSPOWER_STATUS=%#x\n",
		readl(b + ABOX_IP_INDEX), readl(b + ABOX_VERSION),
		readl(b + ABOX_SYSPOWER_CTRL), readl(b + ABOX_SYSPOWER_STATUS));

	pr_info("pd-aud: reading ABOX SysMMU (0x17e00000)\n");
	b = a1.win[W_SYSMMU];
	pr_info("pd-aud: SysMMU CTRL=%#x VERSION=%#x\n", readl(b + SYSMMU_CTRL),
		readl(b + SYSMMU_VERSION));

	if (a1_pmu_rd(PMU_AUD_STATUS, &st)) {
		*still_on = false;
		pr_info("pd-aud: AUD_STATUS re-read FAILED\n");
		return;
	}
	*still_on = (st & 0xf) == PMU_AUD_STATUS_ON;
	pr_info("pd-aud: AUD_STATUS after=%#x (%s)\n", st,
		*still_on ? "still on" : "CHANGED");
}

/* ---- self-test ------------------------------------------------------------ */

static void a1_work_fn(struct work_struct *work)
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
	bool pd_on, pd_read = false, pd_still_on = false, amp_on = false;
	int pmic_w0 = 0, pmic_writes = 0, i;

#define A1_FAIL(s) strlcat(fails, " " s, sizeof(fails))

	pr_info("start uptime=%lld ms on CPU%d, read-only (no MMIO/PMIC/I2C data writes, no SPI)\n",
		ktime_to_ms(ktime_get_boottime()), raw_smp_processor_id());

	/* 1. recovery first */
	wdt_why = a1_wdt_arm();
	a1.wdt_armed = !wdt_why;
	if (wdt_why) {
		pr_info("wdt: NOT armed (%s): pd-aud reads will be skipped\n", wdt_why);
		A1_FAIL("wdt");
	}

	/* 2. PMU */
	a1_read_pmu(&pmu);
	if (pmu.err)
		A1_FAIL("pmu");
	pd_on = !pmu.err && (pmu.aud_status & 0xf) == PMU_AUD_STATUS_ON;

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

	/* 5. amps */
	adap = a1.bus_np ? of_get_i2c_adapter_by_node(a1.bus_np) : NULL;
	if (!adap) {
		pr_info("amp: HSI2C18 (0x108c0000) adapter not registered: amps not read\n");
		A1_FAIL("amp-bus");
	} else {
		for (i = 0; i < ARRAY_SIZE(amps); i++) {
			a1_read_amp(adap, &amps[i]);
			if (amps[i].err || !amps[i].rev_known)
				A1_FAIL("amp-read");
			else if (amps[i].shdn & MAX98512_GLOBAL_EN)
				amp_on = true;
		}
		i2c_put_adapter(adap);
	}
	/* 6. codec */
	spi = a1_codec_spi(&l33, &l34, pv);

	pr_info("partial: pd_aud=%s abox_cpu=%s l33=%#04x l34=%#04x amp39 rev=%#04x en=%u amp3a rev=%#04x en=%u codec_spi=%s (pd-aud reads next)\n",
		pmu.err ? "?" : pd_on ? "on" : "off",
		pmu.err ? "?" : pmu.cpu_status & 1 ? "on" : "off", l33.ctrl, l34.ctrl,
		amps[0].rev, !!(amps[0].shdn & MAX98512_GLOBAL_EN),
		amps[1].rev, !!(amps[1].shdn & MAX98512_GLOBAL_EN), spi);

	/* 7. pd-aud: power proven on and recovery armed, else skip */
	if (!pd_on)
		pr_info("pd-aud: %s: ABOX/SYSREG_AUD/SysMMU/pinctrl_aud reads skipped\n",
			pmu.err ? "AUD_STATUS unknown" : "off");
	else if (!a1.wdt_armed)
		pr_info("pd-aud: on, but no watchdog deadline: reads skipped\n");
	else {
		a1_read_pd_aud(&pd_still_on);
		pd_read = true;
		if (!pd_still_on)
			A1_FAIL("pd-aud-changed");
	}

	if (pmic_writes)
		A1_FAIL("pmic-writes");
	if (amp_on)
		verdict = "STOP";	/* warm amp: report only, owner decides */
	else
		verdict = fails[0] ? "FAIL" : "PASS";

	pr_info("%s pd_aud=%s%s abox_cpu=%s l33=%#04x (%s) l34=%#04x (%s) amp39 rev=%#04x%s en=%u amp3a rev=%#04x%s en=%u codec_spi=%s jack=codec-jd2 wdt=%s writes=%u%s%s\n",
		verdict, pmu.err ? "?" : pd_on ? "on" : "off",
		pd_read ? "(read)" : "(skipped)",
		pmu.err ? "?" : pmu.cpu_status & 1 ? "on" : "off",
		l33.ctrl, l33.err ? "unread" : a1_ldo_state(l33.ctrl),
		l34.ctrl, l34.err ? "unread" : a1_ldo_state(l34.ctrl),
		amps[0].rev, amps[0].rev_known ? (amps[0].rev_b ? "(B)" : "(A)") : "(?)",
		!!(amps[0].shdn & MAX98512_GLOBAL_EN),
		amps[1].rev, amps[1].rev_known ? (amps[1].rev_b ? "(B)" : "(A)") : "(?)",
		!!(amps[1].shdn & MAX98512_GLOBAL_EN), spi,
		a1.wdt_armed ? "armed" : "no",
		pmic_writes,
		fails[0] ? " fails:" : "", fails);
	if (amp_on)
		pr_info("STOP: an amp has GLOBAL_EN=1 left from before this kernel; nothing was written, owner decides (audio-design.md A1)\n");
#undef A1_FAIL
}

/* ---- probe ------------------------------------------------------------------ */

static int a1_map(struct platform_device *pdev)
{
	struct resource *r;
	int i;

	for (i = 0; i < W_NR; i++) {
		r = platform_get_resource_byname(pdev, IORESOURCE_MEM, a1_win_names[i]);
		if (!r || resource_size(r) < a1_win_min[i])
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "reg \"%s\" missing or smaller than %#x\n",
					     a1_win_names[i], a1_win_min[i]);
		/* no request_mem_region: the owning drivers keep their claims */
		a1.win[i] = devm_ioremap(&pdev->dev, r->start, resource_size(r));
		if (!a1.win[i])
			return -ENOMEM;
	}
	return 0;
}

static int a1_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	unsigned int cpu;
	s64 now, delay;
	int ret;

	if (of_property_read_u32(np, "samsung,audio-a1-selftest-ms", &a1.start_ms)) {
		dev_info(dev, "no samsung,audio-a1-selftest-ms: idle\n");
		return 0;
	}
	if (a1.start_ms < A1_START_MIN_MS || a1.start_ms > A1_START_MAX_MS)
		return dev_err_probe(dev, -EINVAL, "selftest start %u ms outside %u..%u\n",
				     a1.start_ms, A1_START_MIN_MS, A1_START_MAX_MS);
	if (of_property_read_u32(np, "samsung,audio-a1-deadline-ms", &a1.deadline_ms) ||
	    a1.deadline_ms < A1_DEADLINE_MIN_MS || a1.deadline_ms > A1_DEADLINE_MAX_MS)
		return dev_err_probe(dev, -EINVAL,
				     "samsung,audio-a1-deadline-ms missing or outside %u..%u\n",
				     A1_DEADLINE_MIN_MS, A1_DEADLINE_MAX_MS);

	a1.pmu = syscon_regmap_lookup_by_phandle(np, "samsung,pmu-syscon");
	if (IS_ERR(a1.pmu))
		return dev_err_probe(dev, PTR_ERR(a1.pmu), "PMU syscon\n");
	ret = a1_map(pdev);
	if (ret)
		return ret;
	a1.pmic_np = of_parse_phandle(np, "samsung,pmic", 0);
	a1.bus_np = of_parse_phandle(np, "samsung,amp-i2c-bus", 0);

	/*
	 * Not on CPU0: a CPU wedged on a pd-aud read must leave the vehicle's
	 * CPU0-pinned PID1 restart timer running.
	 */
	cpu = cpumask_next(0, cpu_online_mask);
	if (cpu >= nr_cpu_ids)
		cpu = 0;
	INIT_DELAYED_WORK(&a1.work, a1_work_fn);
	now = ktime_to_ms(ktime_get_boottime());
	delay = a1.start_ms > now ? a1.start_ms - now : 0;
	queue_delayed_work_on(cpu, system_long_wq, &a1.work, msecs_to_jiffies(delay));
	dev_info(dev, "audio A1 read-only probe scheduled at ~%u ms on CPU%u, watchdog deadline %u ms\n",
		 a1.start_ms, cpu, a1.deadline_ms);
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
