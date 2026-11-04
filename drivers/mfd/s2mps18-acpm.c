// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S2MPS18 main PMIC on Exynos9810 (Galaxy Note9 crownlte): MFD core
 * over the ACPM IPC channel 2 PMIC transport.
 *
 * 4.9 reference (vendor kernel running on the phone):
 *   drivers/mfd/s2mps18_core.c        all accessors call exynos_acpm_*_reg()
 *                                     with the dummy I2C address as "type":
 *                                     TOP 0, PMIC 1, RTC 2 (lines 41-43, 52-120)
 *   drivers/soc/samsung/acpm/acpm_mfd.c   cmd[0]/cmd[1] encoding
 *   DT speedy@141C0000/s2mps18mfd@00     acpm-ipc-channel = <2>
 *
 * This driver is read-only by default. It never writes the TOP or PMIC
 * blocks (regulators, IRQ masks, power control) and registers no regulator.
 * The only writes it can issue are to the RTC block, and only when the DT
 * opts in:
 *   samsung,rtc-read-latch    RUDR latch write to RTC UPDATE (0x02) before
 *                             reading time, exactly as 4.9 s2m_rtc_update()
 *   samsung,rtc-write-enable  + time (0x05-0x0b) and alarm 0 (0x0c-0x12)
 *                             registers, WUDR/AUDR commits (set_time, alarms)
 * The same limits are sealed into the transport (acpm_mfd.c), which refuses
 * every other write regardless of caller.
 */

#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/mfd/core.h>
#include <linux/mfd/samsung/s2mps18-acpm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/timekeeping.h>
#include <linux/workqueue.h>
#include <soc/samsung/acpm_ipc_ctrl.h>
#include <soc/samsung/acpm_mfd.h>

#define S2MPS18_ACPM_CHANNEL		2
#define S2MPS18_SELFTEST_MS		106000

static const struct mfd_cell s2mps18_acpm_cells[] = {
	{ .name = "s2mps18-acpm-rtc", },
};

static int __s2mps18_bulk_read(u8 type, u8 reg, int count, u8 *buf)
{
	int n, ret;

	while (count > 0) {
		n = min(count, 8);
		ret = n == 1 ? exynos_acpm_read_reg(type, reg, buf) :
			       exynos_acpm_bulk_read(type, reg, n, buf);
		if (ret)
			return ret;
		reg += n;
		buf += n;
		count -= n;
	}
	return 0;
}

int s2mps18_acpm_read(struct s2mps18_acpm *pmic, u8 type, u8 reg, u8 *val)
{
	int ret;

	mutex_lock(&pmic->io_lock);
	ret = exynos_acpm_read_reg(type, reg, val);
	mutex_unlock(&pmic->io_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mps18_acpm_read);

int s2mps18_acpm_bulk_read(struct s2mps18_acpm *pmic, u8 type, u8 reg,
			   int count, u8 *buf)
{
	int ret;

	if (count < 1 || reg + count > 0x100)
		return -EINVAL;
	mutex_lock(&pmic->io_lock);
	ret = __s2mps18_bulk_read(type, reg, count, buf);
	mutex_unlock(&pmic->io_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mps18_acpm_bulk_read);

/* Driver-level gate; acpm_mfd.c enforces the same sealed policy again. */
static bool s2mps18_rtc_write_ok(struct s2mps18_acpm *pmic, u8 reg, int count,
				 const u8 *buf)
{
	int i;

	for (i = 0; i < count; i++, reg++) {
		if (reg == S2MPS18_RTC_UPDATE) {
			/*
			 * UPDATE must be exactly the probe-time base (WAKE and
			 * other static bits unchanged) plus permitted strobes:
			 * RUDR with latch, WUDR/AUDR with write. Never FREEZE.
			 */
			u8 strobes = S2MPS18_RTC_UPDATE_RUDR;

			if (!pmic->rtc_latch && !pmic->rtc_write)
				return false;
			if (pmic->rtc_write)
				strobes |= S2MPS18_RTC_UPDATE_WUDR |
					   S2MPS18_RTC_UPDATE_AUDR;
			if ((buf[i] & ~strobes) != pmic->rtc_update_base)
				return false;
		} else if (!pmic->rtc_write ||
			   reg < S2MPS18_RTC_SEC || reg > S2MPS18_RTC_A0YEAR) {
			return false;
		}
	}
	return true;
}

static int __s2mps18_rtc_write(struct s2mps18_acpm *pmic, u8 reg, int count,
			       const u8 *buf)
{
	int ret;

	if (count < 1 || count > 8 || !s2mps18_rtc_write_ok(pmic, reg, count, buf)) {
		dev_warn_ratelimited(pmic->dev,
				     "refused RTC write reg=%#04x count=%d (latch=%d write=%d)\n",
				     reg, count, pmic->rtc_latch, pmic->rtc_write);
		return -EPERM;
	}
	ret = count == 1 ? exynos_acpm_write_reg(ACPM_MFD_TYPE_RTC, reg, buf[0]) :
		exynos_acpm_bulk_write(ACPM_MFD_TYPE_RTC, reg, count, (u8 *)buf);
	if (!ret)
		atomic_inc(&pmic->writes);
	return ret;
}

int s2mps18_acpm_rtc_write(struct s2mps18_acpm *pmic, u8 reg, u8 val)
{
	int ret;

	mutex_lock(&pmic->io_lock);
	ret = __s2mps18_rtc_write(pmic, reg, 1, &val);
	mutex_unlock(&pmic->io_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mps18_acpm_rtc_write);

int s2mps18_acpm_rtc_bulk_write(struct s2mps18_acpm *pmic, u8 reg, int count,
				const u8 *buf)
{
	int ret;

	mutex_lock(&pmic->io_lock);
	ret = __s2mps18_rtc_write(pmic, reg, count, buf);
	mutex_unlock(&pmic->io_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(s2mps18_acpm_rtc_bulk_write);

bool s2mps18_acpm_rtc_mode_ok(struct s2mps18_acpm *pmic)
{
	return (pmic->rtc_ctrl & S2MPS18_RTC_CTRL_MODEL24) &&
	       !(pmic->rtc_ctrl & S2MPS18_RTC_CTRL_BCD_EN);
}
EXPORT_SYMBOL_GPL(s2mps18_acpm_rtc_mode_ok);

/*
 * 4.9 s2m_rtc_read_time(): write UPDATE = update_reg | RUDR, sleep 1-2 ms,
 * bulk-read SEC..YEAR. Without the latch permission the read buffer is read
 * as is: it holds whatever the last RUDR (sboot or 4.9) latched.
 */
int s2mps18_acpm_rtc_read_time_regs(struct s2mps18_acpm *pmic,
				    u8 data[S2MPS18_RTC_NR_TIME], bool *latched)
{
	u8 v = pmic->rtc_update_base | S2MPS18_RTC_UPDATE_RUDR;
	bool latch = pmic->rtc_latch || pmic->rtc_write;
	int ret = 0;

	mutex_lock(&pmic->io_lock);
	if (latch) {
		ret = __s2mps18_rtc_write(pmic, S2MPS18_RTC_UPDATE, 1, &v);
		if (!ret)
			usleep_range(1000, 2000);
	}
	if (!ret)
		ret = __s2mps18_bulk_read(ACPM_MFD_TYPE_RTC, S2MPS18_RTC_SEC,
					  S2MPS18_RTC_NR_TIME, data);
	mutex_unlock(&pmic->io_lock);
	if (latched)
		*latched = latch && !ret;
	return ret;
}
EXPORT_SYMBOL_GPL(s2mps18_acpm_rtc_read_time_regs);

/* 4.9 rtc-s2mps18.c s2m_data_to_tm(): binary, 24 h, one-hot weekday. */
void s2mps18_acpm_rtc_decode(const u8 d[S2MPS18_RTC_NR_TIME], struct rtc_time *tm)
{
	u8 wday = d[3] & 0x7f;

	tm->tm_sec = d[0] & 0x7f;
	tm->tm_min = d[1] & 0x7f;
	tm->tm_hour = d[2] & 0x1f;
	tm->tm_wday = wday ? __fls(wday) : 0;
	tm->tm_mday = d[4] & 0x1f;
	tm->tm_mon = (d[5] & 0x0f) - 1;
	tm->tm_year = (d[6] & 0x7f) + 100;
	tm->tm_yday = 0;
	tm->tm_isdst = 0;
}
EXPORT_SYMBOL_GPL(s2mps18_acpm_rtc_decode);

/*
 * ---- DT-gated self-test (diagnostic DT only): reads only ----
 * Scheduled from the driver init, not from probe, so that a failed or
 * deferred probe is still reported at ~106 s (the diagnostic initramfs drops
 * early kernel messages).
 */

static struct s2mps18_acpm *st_pmic;
static const char *st_probe_stage = "probe never ran";
static int st_probe_ret = -ENODEV;
static unsigned int st_errors;

static void st_dump(struct s2mps18_acpm *pmic, const char *blk, u8 type,
		    u8 first, u8 last)
{
	u8 buf[16];
	unsigned int reg, n;
	int ret;

	for (reg = first; reg <= last; reg += n) {
		n = min_t(unsigned int, 16, last - reg + 1);
		ret = s2mps18_acpm_bulk_read(pmic, type, reg, n, buf);
		if (ret) {
			st_errors++;
			pr_info("pmic selftest: %s[%#04x..%#04x] read FAILED %d\n",
				blk, reg, reg + n - 1, ret);
			continue;
		}
		pr_info("pmic selftest: %s[%#04x..%#04x]: %*ph\n",
			blk, reg, reg + n - 1, n, buf);
	}
}

static int st_read_rtc(struct s2mps18_acpm *pmic, const char *tag,
		       time64_t *secs)
{
	u8 d[S2MPS18_RTC_NR_TIME];
	struct rtc_time tm;
	bool latched;
	int ret, valid;

	ret = s2mps18_acpm_rtc_read_time_regs(pmic, d, &latched);
	if (ret) {
		st_errors++;
		pr_info("pmic selftest: rtc %s read FAILED %d\n", tag, ret);
		return ret;
	}
	s2mps18_acpm_rtc_decode(d, &tm);
	valid = rtc_valid_tm(&tm);
	*secs = valid ? 0 : rtc_tm_to_time64(&tm);
	pr_info("pmic selftest: rtc %s raw=%*ph -> %ptR wday=%d latched=%s valid=%s\n",
		tag, S2MPS18_RTC_NR_TIME, d, &tm, tm.tm_wday,
		latched ? "yes" : "no", valid ? "NO" : "yes");
	return valid;
}

static void s2mps18_selftest_fn(struct work_struct *work)
{
	struct s2mps18_acpm *pmic = READ_ONCE(st_pmic);
	time64_t t1 = 0, t2 = 0, now;
	unsigned int writes_before;
	struct rtc_time tm;
	u8 id = 0, v = 0;
	int ret, r1, r2;

	if (!pmic) {
		pr_info("pmic selftest: FAIL driver not bound: %s (ret %d); ACPM PMIC ready=%d ch%d mode=%d (1 = AP-polled)\n",
			st_probe_stage, st_probe_ret, exynos_acpm_mfd_is_ready(),
			S2MPS18_ACPM_CHANNEL,
			acpm_ipc_get_ch_mode(S2MPS18_ACPM_CHANNEL));
		return;
	}

	writes_before = atomic_read(&pmic->writes);
	pr_info("pmic selftest: start uptime=%u ms, ACPM ch%d, latch=%s rtc-write=%s\n",
		jiffies_to_msecs(jiffies - INITIAL_JIFFIES), S2MPS18_ACPM_CHANNEL,
		pmic->rtc_latch ? "on" : "off", pmic->rtc_write ? "on" : "off");

	ret = s2mps18_acpm_read(pmic, ACPM_MFD_TYPE_TOP, S2MPS18_TOP_PMICID, &id);
	if (ret)
		st_errors++;
	pr_info("pmic selftest: chip id TOP[0x00]=%#04x (probe %#04x) %s\n",
		id, pmic->chip_id, ret ? "READ FAILED" :
		id == pmic->chip_id ? "stable" : "CHANGED");

	ret = s2mps18_acpm_read(pmic, ACPM_MFD_TYPE_TOP, S2MPS18_TOP_INTSRC_MASK, &v);
	if (ret)
		st_errors++;
	else
		pr_info("pmic selftest: TOP INTSRC_MASK=%#04x\n", v);

	/* INT1M..INT7M, STATUS1/2, PWRONSRC, OFFSRC, BUCHG, RTCBUF, CTRL1-3 */
	st_dump(pmic, "pmic", ACPM_MFD_TYPE_PMIC, S2MPS18_PMIC_INT1M, S2MPS18_PMIC_CTRL3);
	/* buck CTRL/OUT, ramp, DVS; then LDO CTRL (bits 7:6 = enable mode) */
	st_dump(pmic, "pmic", ACPM_MFD_TYPE_PMIC, S2MPS18_PMIC_B1CTRL, S2MPS18_PMIC_BUCK7_DVS);
	st_dump(pmic, "pmic", ACPM_MFD_TYPE_PMIC, S2MPS18_PMIC_L1CTRL, S2MPS18_PMIC_L45CTRL);
	/* RTC CTRL WTSR_SMPL UPDATE CAPSEL MSEC, alarm 0/1, OSCCTRL */
	st_dump(pmic, "rtc", ACPM_MFD_TYPE_RTC, S2MPS18_RTC_CTRL, S2MPS18_RTC_MSEC);
	st_dump(pmic, "rtc", ACPM_MFD_TYPE_RTC, S2MPS18_RTC_A0SEC, S2MPS18_RTC_OSCCTRL);

	pr_info("pmic selftest: rtc mode CTRL=%#04x (%s) update_base=%#04x\n",
		pmic->rtc_ctrl, s2mps18_acpm_rtc_mode_ok(pmic) ?
		"binary 24h, as 4.9 init" : "NOT binary 24h", pmic->rtc_update_base);

	r1 = st_read_rtc(pmic, "t1", &t1);
	msleep(1000);
	r2 = st_read_rtc(pmic, "t2", &t2);
	if (!r1 && !r2) {
		long long adv = t2 - t1;

		if (pmic->rtc_latch)
			pr_info("pmic selftest: rtc advance %lld s over 1 s: %s\n", adv,
				adv >= 1 && adv <= 2 ? "PASS" : "FAIL");
		else
			pr_info("pmic selftest: rtc advance %lld s over 1 s: %s\n", adv,
				adv == 0 ? "frozen read buffer (expected: no RUDR latch without samsung,rtc-read-latch)" :
				"buffer moved without latch (unexpected)");
		rtc_time64_to_tm(t2, &tm);
		pr_info("pmic selftest: rtc plausibility year=%d: %s\n",
			tm.tm_year + 1900,
			tm.tm_year + 1900 >= 2026 && tm.tm_year + 1900 <= 2099 ?
			"PASS (>= 2026)" : "FAIL");
		now = ktime_get_real_seconds();
		pr_info("pmic selftest: system realtime - rtc = %lld s (system time may itself come from this RTC via hctosys)\n",
			(long long)(now - t2));
	}

	/* The rtc_class path hctosys and /dev/rtc0 use. */
	if (pmic->rtc) {
		memset(&tm, 0, sizeof(tm));
		ret = rtc_read_time(pmic->rtc, &tm);
		if (ret)
			st_errors++;
		pr_info("pmic selftest: %s rtc_read_time ret=%d %ptR\n",
			dev_name(&pmic->rtc->dev), ret, &tm);
	} else {
		st_errors++;
		pr_info("pmic selftest: RTC class device NOT registered\n");
	}

	pr_info("pmic selftest: done errors=%u writes-during-selftest=%u writes-since-probe=%u\n",
		st_errors, atomic_read(&pmic->writes) - writes_before,
		atomic_read(&pmic->writes));
}

static DECLARE_DELAYED_WORK(s2mps18_selftest_work, s2mps18_selftest_fn);

static int st_fail(const char *stage, int err)
{
	st_probe_stage = stage;
	st_probe_ret = err;
	return err;
}

static int s2mps18_acpm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct acpm_mfd_write_policy policy = { .update_reg = S2MPS18_RTC_UPDATE };
	struct s2mps18_acpm *pmic;
	u32 ch;
	u8 v;
	int ret;

	if (of_property_read_u32(np, "acpm-ipc-channel", &ch) ||
	    ch != S2MPS18_ACPM_CHANNEL)
		return dev_err_probe(dev, st_fail("bad acpm-ipc-channel", -EINVAL),
				     "acpm-ipc-channel must be <2>\n");

	/*
	 * acpm_mfd.c reads the response from cfg->cmd after acpm_ipc_send_data()
	 * returns; that only happens when the AP polls the channel. On an
	 * interrupt-mode channel every read would return 0 silently.
	 */
	ret = acpm_ipc_get_ch_mode(ch);
	if (ret < 0 || !exynos_acpm_mfd_is_ready())
		return dev_err_probe(dev, st_fail("ACPM channel not ready (deferred)",
						  -EPROBE_DEFER),
				     "ACPM channel %u not ready (mode %d)\n", ch, ret);
	if (ret != 1)
		return dev_err_probe(dev, st_fail("ACPM channel not AP-polled", -ENODEV),
				     "ACPM channel %u is not AP-polled; refusing\n", ch);

	pmic = devm_kzalloc(dev, sizeof(*pmic), GFP_KERNEL);
	if (!pmic)
		return st_fail("out of memory", -ENOMEM);
	pmic->dev = dev;
	mutex_init(&pmic->io_lock);
	atomic_set(&pmic->writes, 0);
	pmic->rtc_write = of_property_read_bool(np, "samsung,rtc-write-enable");
	/* set_time/alarms need latched reads to verify them: write implies latch */
	pmic->rtc_latch = pmic->rtc_write ||
			  of_property_read_bool(np, "samsung,rtc-read-latch");

	/*
	 * Seal the transport write policy before anything else can: zero mask
	 * (refuse all) unless the DT opted in. Never TOP/PMIC; never FREEZE;
	 * never alarm 1 (power-on alarm), WTSR/SMPL, CTRL, CAPSEL, OSCCTRL.
	 */
	if (pmic->rtc_latch) {
		policy.rtc_reg_mask |= BIT(S2MPS18_RTC_UPDATE);
		policy.update_forbidden = S2MPS18_RTC_UPDATE_WUDR |
					  S2MPS18_RTC_UPDATE_FREEZE |
					  S2MPS18_RTC_UPDATE_AUDR;
	}
	if (pmic->rtc_write) {
		policy.rtc_reg_mask |= GENMASK(S2MPS18_RTC_A0YEAR, S2MPS18_RTC_SEC);
		policy.update_forbidden = S2MPS18_RTC_UPDATE_FREEZE;
	}
	ret = exynos_acpm_mfd_seal_write_policy(&policy);
	if (ret)
		return dev_err_probe(dev, st_fail("write policy seal", ret),
				     "cannot seal PMIC write policy\n");

	ret = s2mps18_acpm_read(pmic, ACPM_MFD_TYPE_TOP, S2MPS18_TOP_PMICID,
				&pmic->chip_id);
	if (ret)
		return dev_err_probe(dev, st_fail("chip id read", ret),
				     "chip id read over ACPM failed\n");

	ret = s2mps18_acpm_read(pmic, ACPM_MFD_TYPE_RTC, S2MPS18_RTC_CTRL,
				&pmic->rtc_ctrl);
	if (!ret)
		ret = s2mps18_acpm_read(pmic, ACPM_MFD_TYPE_RTC,
					S2MPS18_RTC_UPDATE, &v);
	if (ret)
		return dev_err_probe(dev, st_fail("RTC CTRL/UPDATE read", ret),
				     "RTC CTRL/UPDATE read failed\n");
	/* 4.9 s2m_rtc_init_reg(): keep everything but the self-clearing strobes */
	pmic->rtc_update_base = v & ~(S2MPS18_RTC_UPDATE_RUDR |
				      S2MPS18_RTC_UPDATE_WUDR |
				      S2MPS18_RTC_UPDATE_FREEZE |
				      S2MPS18_RTC_UPDATE_AUDR);

	dev_info(dev, "S2MPS18 chip id %#04x via ACPM ch%u; RTC CTRL=%#04x UPDATE=%#04x; RTC latch %s, RTC write %s; TOP/PMIC writes refused, no regulators registered\n",
		 pmic->chip_id, ch, pmic->rtc_ctrl, v,
		 pmic->rtc_latch ? "on" : "off", pmic->rtc_write ? "on" : "off");

	platform_set_drvdata(pdev, pmic);

	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_NONE, s2mps18_acpm_cells,
				   ARRAY_SIZE(s2mps18_acpm_cells), NULL, 0, NULL);
	if (ret)
		return dev_err_probe(dev, st_fail("mfd cells", ret), "mfd cells\n");

	st_fail("bound", 0);
	WRITE_ONCE(st_pmic, pmic);
	return 0;
}

static const struct of_device_id s2mps18_acpm_of_match[] = {
	{ .compatible = "samsung,exynos9810-s2mps18-acpm" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mps18_acpm_of_match);

static struct platform_driver s2mps18_acpm_driver = {
	.driver = {
		.name = "s2mps18-acpm",
		.of_match_table = s2mps18_acpm_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = s2mps18_acpm_probe,
};

static int __init s2mps18_acpm_init(void)
{
	/* 106 s: after cpufreq (100 s) / devfreq (105 s), before the 120 s reboot */
	unsigned long at = msecs_to_jiffies(S2MPS18_SELFTEST_MS);
	unsigned long up = jiffies - INITIAL_JIFFIES;
	struct device_node *np;

	for_each_compatible_node(np, NULL, "samsung,exynos9810-s2mps18-acpm") {
		if (of_device_is_available(np) &&
		    of_property_read_bool(np, "samsung,pmic-selftest")) {
			queue_delayed_work(system_long_wq, &s2mps18_selftest_work,
					   at > up ? at - up : 0);
			pr_info("s2mps18-acpm: pmic selftest scheduled at ~%u s uptime\n",
				S2MPS18_SELFTEST_MS / 1000);
			of_node_put(np);
			break;
		}
	}
	return platform_driver_register(&s2mps18_acpm_driver);
}
device_initcall(s2mps18_acpm_init);

MODULE_DESCRIPTION("Samsung S2MPS18 PMIC core over Exynos9810 ACPM (read-only by default)");
MODULE_LICENSE("GPL");
