/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Samsung S2MPS18 main PMIC on Exynos9810 (Galaxy Note9, crownlte), reached
 * through the ACPM IPC channel 2 "PMIC" transport (drivers/soc/samsung/acpm/
 * acpm_mfd.c). The APM firmware runs the SPEEDY transaction; the AP never
 * drives the SPEEDY controller for this chip.
 *
 * Register map from the 4.9 vendor kernel:
 *   include/linux/mfd/samsung/s2mps18-private.h (TOP/PMIC blocks)
 *   include/linux/mfd/samsung/rtc-s2mp.h, !CONFIG_RTC_DRV_S2MP branch (RTC)
 */
#ifndef __LINUX_MFD_S2MPS18_ACPM_H
#define __LINUX_MFD_S2MPS18_ACPM_H

#include <linux/bits.h>
#include <linux/mutex.h>
#include <linux/types.h>

/* TOP block (type 0) */
#define S2MPS18_TOP_PMICID		0x00
#define S2MPS18_TOP_INTSRC		0x01
#define S2MPS18_TOP_INTSRC_MASK		0x02

/* PMIC block (type 1). INT1..INT7 (0x00..0x06) are clear-on-read: never read. */
#define S2MPS18_PMIC_INT1M		0x07
#define S2MPS18_PMIC_STATUS1		0x0e
#define S2MPS18_PMIC_STATUS2		0x0f
#define S2MPS18_PMIC_PWRONSRC		0x10
#define S2MPS18_PMIC_OFFSRC		0x11
#define S2MPS18_PMIC_CTRL3		0x16
#define S2MPS18_PMIC_B1CTRL		0x1c
#define S2MPS18_PMIC_BUCK7_DVS		0x48
#define S2MPS18_PMIC_L1CTRL		0x49
#define S2MPS18_PMIC_L45CTRL		0x76
#define S2MPS18_PMIC_STATUS2_RTCA0E	BIT(2)

/* RTC block (type 2) */
#define S2MPS18_RTC_CTRL		0x00
#define S2MPS18_RTC_WTSR_SMPL		0x01
#define S2MPS18_RTC_UPDATE		0x02
#define S2MPS18_RTC_CAPSEL		0x03
#define S2MPS18_RTC_MSEC		0x04
#define S2MPS18_RTC_SEC			0x05	/* SEC MIN HOUR WEEK DAY MON YEAR */
#define S2MPS18_RTC_YEAR		0x0b
#define S2MPS18_RTC_A0SEC		0x0c	/* alarm 0, same 7-byte layout */
#define S2MPS18_RTC_A0YEAR		0x12
#define S2MPS18_RTC_A1SEC		0x13	/* alarm 1 = power-on alarm (never written) */
#define S2MPS18_RTC_OSCCTRL		0x1a
#define S2MPS18_RTC_NR_TIME		7

#define S2MPS18_RTC_CTRL_BCD_EN		BIT(0)
#define S2MPS18_RTC_CTRL_MODEL24	BIT(1)
#define S2MPS18_RTC_UPDATE_RUDR		BIT(0)	/* latch counters into read buffer */
#define S2MPS18_RTC_UPDATE_WUDR		BIT(1)	/* commit time write buffer */
#define S2MPS18_RTC_UPDATE_FREEZE	BIT(2)
#define S2MPS18_RTC_UPDATE_WAKE		BIT(3)	/* alarm-boot enable (4.9 RTC_WAKE) */
#define S2MPS18_RTC_UPDATE_AUDR		BIT(4)	/* commit alarm write buffer */
#define S2MPS18_RTC_HOUR_PM		BIT(6)
#define S2MPS18_RTC_ALARM_EN		BIT(7)

struct rtc_device;

struct s2mps18_acpm {
	struct device *dev;
	struct mutex io_lock;		/* one ACPM PMIC transaction at a time */
	u8 chip_id;
	bool rtc_latch;			/* DT samsung,rtc-read-latch */
	bool rtc_write;			/* DT samsung,rtc-write-enable */
	u8 rtc_ctrl;			/* RTC CTRL at probe */
	u8 rtc_update_base;		/* UPDATE & ~(RUDR|WUDR|FREEZE|AUDR) at probe */
	atomic_t writes;		/* successful write IPCs issued by us */
	struct rtc_device *rtc;		/* set by the RTC cell once registered */
};

int s2mps18_acpm_read(struct s2mps18_acpm *pmic, u8 type, u8 reg, u8 *val);
int s2mps18_acpm_bulk_read(struct s2mps18_acpm *pmic, u8 type, u8 reg,
			   int count, u8 *buf);
int s2mps18_acpm_rtc_write(struct s2mps18_acpm *pmic, u8 reg, u8 val);
int s2mps18_acpm_rtc_bulk_write(struct s2mps18_acpm *pmic, u8 reg, int count,
				const u8 *buf);
struct rtc_time;
int s2mps18_acpm_rtc_read_time_regs(struct s2mps18_acpm *pmic,
				    u8 data[S2MPS18_RTC_NR_TIME], bool *latched);
void s2mps18_acpm_rtc_decode(const u8 data[S2MPS18_RTC_NR_TIME],
			     struct rtc_time *tm);
bool s2mps18_acpm_rtc_mode_ok(struct s2mps18_acpm *pmic);

#endif /* __LINUX_MFD_S2MPS18_ACPM_H */
