// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S2MPS18 RTC over the Exynos9810 ACPM PMIC transport.
 *
 * Register layout and sequences follow the 4.9 vendor driver
 * drivers/rtc/rtc-s2mps18.c (s2m_rtc_update, s2m_rtc_read_time,
 * s2m_rtc_set_time, s2m_rtc_read_alarm, s2m_rtc_set_alarm) with the
 * !CONFIG_RTC_DRV_S2MP register map of rtc-s2mp.h (SEC = 0x05).
 *
 * Default (diagnostic DT): read_time only. Nothing is written; time comes from
 * the RTC read buffer, which only refreshes on an RUDR latch. With
 * samsung,rtc-read-latch the RUDR strobe is written before each read (4.9
 * behaviour). set_time and alarm 0 exist only with samsung,rtc-write-enable.
 * Alarm 1 (power-on alarm), WTSR/SMPL, CTRL, CAPSEL and OSCCTRL are never
 * written. No PMIC interrupt is wired yet, so alarms are not delivered.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/mfd/samsung/s2mps18-acpm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <soc/samsung/acpm_mfd.h>

struct s2mps18_rtc {
	struct device *dev;
	struct s2mps18_acpm *pmic;
	struct rtc_device *rtc;
	struct mutex lock;		/* multi-IPC RTC sequences */
};

static int s2mps18_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct s2mps18_rtc *info = dev_get_drvdata(dev);
	u8 d[S2MPS18_RTC_NR_TIME];
	bool latched;
	int ret;

	if (!s2mps18_acpm_rtc_mode_ok(info->pmic))
		return -EINVAL;

	mutex_lock(&info->lock);
	ret = s2mps18_acpm_rtc_read_time_regs(info->pmic, d, &latched);
	mutex_unlock(&info->lock);
	if (ret)
		return ret;
	if (!latched)
		dev_warn_once(dev, "time comes from the unlatched RTC read buffer (no samsung,rtc-read-latch): it may be stale\n");

	s2mps18_acpm_rtc_decode(d, tm);
	return rtc_valid_tm(tm);
}

/* 4.9 s2m_tm_to_data() */
static void s2mps18_rtc_encode(const struct rtc_time *tm, u8 *d)
{
	d[0] = tm->tm_sec;
	d[1] = tm->tm_min;
	d[2] = tm->tm_hour | (tm->tm_hour >= 12 ? S2MPS18_RTC_HOUR_PM : 0);
	d[3] = BIT(tm->tm_wday);
	d[4] = tm->tm_mday;
	d[5] = tm->tm_mon + 1;
	d[6] = tm->tm_year - 100;
}

/* 4.9 s2m_rtc_update(): UPDATE = update_reg | strobe, then 1-2 ms. */
static int s2mps18_rtc_strobe(struct s2mps18_rtc *info, u8 strobe)
{
	int ret;

	ret = s2mps18_acpm_rtc_write(info->pmic, S2MPS18_RTC_UPDATE,
				     info->pmic->rtc_update_base | strobe);
	if (!ret)
		usleep_range(1000, 2000);
	return ret;
}

static int s2mps18_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct s2mps18_rtc *info = dev_get_drvdata(dev);
	u8 d[S2MPS18_RTC_NR_TIME];
	int ret;

	if (tm->tm_year < 100 || tm->tm_year > 199)
		return -EINVAL;
	s2mps18_rtc_encode(tm, d);
	dev_info(dev, "set_time %ptR\n", tm);

	mutex_lock(&info->lock);
	ret = s2mps18_acpm_rtc_bulk_write(info->pmic, S2MPS18_RTC_SEC,
					  S2MPS18_RTC_NR_TIME, d);
	if (!ret)
		ret = s2mps18_rtc_strobe(info, S2MPS18_RTC_UPDATE_WUDR);
	mutex_unlock(&info->lock);
	return ret;
}

static int s2mps18_rtc_read_a0(struct s2mps18_rtc *info, u8 *d)
{
	int ret;

	ret = s2mps18_rtc_strobe(info, S2MPS18_RTC_UPDATE_RUDR);
	if (!ret)
		ret = s2mps18_acpm_bulk_read(info->pmic, ACPM_MFD_TYPE_RTC,
					     S2MPS18_RTC_A0SEC,
					     S2MPS18_RTC_NR_TIME, d);
	return ret;
}

static int s2mps18_rtc_write_a0(struct s2mps18_rtc *info, const u8 *d)
{
	int ret;

	ret = s2mps18_acpm_rtc_bulk_write(info->pmic, S2MPS18_RTC_A0SEC,
					  S2MPS18_RTC_NR_TIME, d);
	if (!ret)
		ret = s2mps18_rtc_strobe(info, S2MPS18_RTC_UPDATE_AUDR);
	return ret;
}

static int s2mps18_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct s2mps18_rtc *info = dev_get_drvdata(dev);
	u8 d[S2MPS18_RTC_NR_TIME], status2;
	int ret;

	mutex_lock(&info->lock);
	ret = s2mps18_rtc_read_a0(info, d);
	if (!ret)
		ret = s2mps18_acpm_read(info->pmic, ACPM_MFD_TYPE_PMIC,
					S2MPS18_PMIC_STATUS2, &status2);
	mutex_unlock(&info->lock);
	if (ret)
		return ret;

	s2mps18_acpm_rtc_decode(d, &alrm->time);
	alrm->enabled = !!(d[0] & S2MPS18_RTC_ALARM_EN);
	alrm->pending = !!(status2 & S2MPS18_PMIC_STATUS2_RTCA0E);
	return 0;
}

/* 4.9 s2m_rtc_set_alarm() with alarm_check: disable, commit, then enable. */
static int s2mps18_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct s2mps18_rtc *info = dev_get_drvdata(dev);
	u8 d[S2MPS18_RTC_NR_TIME];
	int i, ret;

	if (alrm->time.tm_year < 100 || alrm->time.tm_year > 199)
		return -EINVAL;
	s2mps18_rtc_encode(&alrm->time, d);
	dev_info(dev, "set_alarm %ptR enabled=%d\n", &alrm->time, alrm->enabled);

	mutex_lock(&info->lock);
	ret = s2mps18_rtc_write_a0(info, d);
	if (!ret && alrm->enabled) {
		for (i = 0; i < S2MPS18_RTC_NR_TIME; i++)
			d[i] |= S2MPS18_RTC_ALARM_EN;
		ret = s2mps18_rtc_write_a0(info, d);
	}
	mutex_unlock(&info->lock);
	return ret;
}

static int s2mps18_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct s2mps18_rtc *info = dev_get_drvdata(dev);
	u8 d[S2MPS18_RTC_NR_TIME];
	int i, ret;

	mutex_lock(&info->lock);
	ret = s2mps18_rtc_read_a0(info, d);
	if (!ret) {
		for (i = 0; i < S2MPS18_RTC_NR_TIME; i++) {
			if (enabled)
				d[i] |= S2MPS18_RTC_ALARM_EN;
			else
				d[i] &= ~S2MPS18_RTC_ALARM_EN;
		}
		ret = s2mps18_rtc_write_a0(info, d);
	}
	mutex_unlock(&info->lock);
	return ret;
}

static const struct rtc_class_ops s2mps18_rtc_ro_ops = {
	.read_time = s2mps18_rtc_read_time,
};

static const struct rtc_class_ops s2mps18_rtc_rw_ops = {
	.read_time = s2mps18_rtc_read_time,
	.set_time = s2mps18_rtc_set_time,
	.read_alarm = s2mps18_rtc_read_alarm,
	.set_alarm = s2mps18_rtc_set_alarm,
	.alarm_irq_enable = s2mps18_rtc_alarm_irq_enable,
};

static int s2mps18_rtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mps18_acpm *pmic = dev_get_drvdata(dev->parent);
	struct s2mps18_rtc *info;
	int ret;

	if (!pmic)
		return -EPROBE_DEFER;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->dev = dev;
	info->pmic = pmic;
	mutex_init(&info->lock);
	platform_set_drvdata(pdev, info);

	if (!s2mps18_acpm_rtc_mode_ok(pmic))
		dev_warn(dev, "RTC CTRL=%#04x is not binary 24 h (4.9 would reinitialise it): reads will fail\n",
			 pmic->rtc_ctrl);

	info->rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(info->rtc))
		return PTR_ERR(info->rtc);

	info->rtc->ops = pmic->rtc_write ? &s2mps18_rtc_rw_ops : &s2mps18_rtc_ro_ops;
	info->rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
	info->rtc->range_max = RTC_TIMESTAMP_END_2099;
	clear_bit(RTC_FEATURE_UPDATE_INTERRUPT, info->rtc->features);
	if (!pmic->rtc_write)
		clear_bit(RTC_FEATURE_ALARM, info->rtc->features);

	ret = devm_rtc_register_device(info->rtc);
	if (ret)
		return ret;
	/* For the core's self-test; both drivers are built in and unbind-proof. */
	WRITE_ONCE(pmic->rtc, info->rtc);

	dev_info(dev, "%s: read_time%s\n", dev_name(&info->rtc->dev),
		 pmic->rtc_write ? " + set_time + alarm 0 (alarm IRQ not wired)" :
		 pmic->rtc_latch ? " only, RUDR latch on" :
		 " only, no writes (buffer not latched)");
	return 0;
}

static struct platform_driver s2mps18_rtc_driver = {
	.driver = {
		.name = "s2mps18-acpm-rtc",
		.suppress_bind_attrs = true,
	},
	.probe = s2mps18_rtc_probe,
};
builtin_platform_driver(s2mps18_rtc_driver);

MODULE_DESCRIPTION("Samsung S2MPS18 RTC over Exynos9810 ACPM");
MODULE_LICENSE("GPL");
