// SPDX-License-Identifier: GPL-2.0
/*
 * MAX77705 read-only battery monitor for the Exynos9810 Note9 (crownlte).
 *
 * Stage 1 of the MAX77705 charger/fuel-gauge port. This driver never writes
 * to the MAX77705. It reads the fuel gauge (I2C 0x36), the charger block
 * (0x69), the PMIC ID (0x66) and one USB-C MCU status register (0x25, which
 * is bound to the exynos9810-max77705-muic driver). From these reads it
 * builds the power_supply devices that the Lineage/Samsung health HAL scans:
 * "battery" (Battery), "ac" (Mains), "usb" (USB) and "wireless" (Wireless).
 * The power_supply core adds a tripless thermal zone named "battery" because
 * "battery" reports POWER_SUPPLY_PROP_TEMP.
 *
 * The register conversions and the capacity pipeline follow the 4.9 Samsung
 * drivers (drivers/battery_v2/max77705_fuelgauge.c, max77705_charger.c and
 * sec_battery.c in the crownlte 4.9 tree), so that the battery percentage
 * shown by Lineage matches 4.9:
 *   - SOCREP is scaled by capacity_min/capacity_max (dynamic scale);
 *   - with CAPACITY_TYPE_ATOMIC, the percentage moves by at most 1 per poll;
 *   - with CAPACITY_TYPE_SKIP_ABNORMAL, it never rises while not charging;
 *   - "battery" reports 100 while the charger says FULL.
 *
 * Everything 4.9 did with I2C writes is left out: the battery model and
 * quick-start (fg_reset_soc), CONFIG.TEX, VEMPTY/VALRT, fuel alerts,
 * QRTABLE, auto-discharge, the FG temperature write-back, DesignCap on JIG,
 * the charger initialisation, the watchdog enable and kick, input and charge
 * current, float voltage, OTG and the shutdown register writes. Interrupt
 * registers are not read either, because reading them clears them.
 *
 * Read-only by construction: every bus access goes through mx_read(), which
 * issues I2C_SMBUS_READ transfers only. The write APIs are poisoned below, so
 * a future edit that calls one fails to compile.
 */
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/soc/samsung/exynos9810-max77705-muic.h>
#include <linux/timekeeping.h>
#include <linux/workqueue.h>

#pragma GCC poison i2c_smbus_write_byte i2c_smbus_write_byte_data
#pragma GCC poison i2c_smbus_write_word_data i2c_smbus_write_block_data
#pragma GCC poison i2c_smbus_write_i2c_block_data i2c_master_send
#pragma GCC poison i2c_master_send_dmasafe i2c_transfer __i2c_transfer
#pragma GCC poison i2c_transfer_buffer_flags
#undef I2C_SMBUS_WRITE		/* a macro cannot be poisoned; drop it first */
#pragma GCC poison I2C_SMBUS_WRITE
#pragma GCC poison regmap_write regmap_update_bits regmap_bulk_write
#pragma GCC poison regmap_raw_write regmap_multi_reg_write

/* 7-bit I2C addresses (4.9 drivers/mfd/max77705.c:48-52) */
#define MX_ADDR_PMIC		0x66
#define MX_ADDR_USBC		0x25
#define MX_ADDR_CHG		0x69
#define MX_ADDR_FG		0x36

/* PMIC (0x66) */
#define PMIC_REG_PMICID1	0x00
#define PMIC_REG_PMICREV	0x01

/* USB-C MCU (0x25): BC_STATUS is a status register, not an interrupt */
#define USBC_REG_BC_STATUS	0x08
#define  BC_VBUSDET		BIT(7)
#define  BC_PRCHGTYP(x)		(((x) >> 3) & 0x7)
#define  BC_CHGTYP(x)		((x) & 0x3)
#define  CHGTYP_NONE		0
#define  CHGTYP_SDP		1
#define  CHGTYP_CDP		2
#define  CHGTYP_DCP		3

/* Charger (0x69). CHG_INT (0xb0) is clear-on-read and is never read. */
#define CHG_REG_INT_OK		0xb2
#define  INT_OK_BATP		BIT(2)
#define  INT_OK_WCIN		BIT(5)
#define  INT_OK_CHGIN		BIT(6)
#define CHG_REG_DTLS_00		0xb3
#define  DTLS00_BATP(x)		((x) & 0x1)
#define  DTLS00_WCIN(x)		(((x) >> 3) & 0x3)
#define  DTLS00_CHGIN(x)	(((x) >> 5) & 0x3)
#define  DTLS00_AUTOIBUS	BIT(7)
#define CHG_REG_DTLS_01		0xb4
#define  DTLS01_CHG(x)		((x) & 0xf)
#define  DTLS01_BAT(x)		(((x) >> 4) & 0x7)
#define CHG_REG_DTLS_02		0xb5
#define CHG_REG_CNFG_00		0xb7
#define  CNFG00_MODE(x)		((x) & 0xf)
#define  CNFG00_CHG		BIT(0)
#define  CNFG00_BUCK		BIT(2)
#define  CNFG00_WDTEN		BIT(4)
#define CHG_REG_CNFG_02		0xb9
#define CHG_REG_CNFG_04		0xbb
#define CHG_REG_CNFG_09		0xc0
#define CHG_REG_CNFG_10		0xc1
#define CHG_REG_CNFG_FIRST	0xb7	/* CNFG_00 */
#define CHG_REG_CNFG_LAST	0xc5	/* CNFG_14 */
#define CHG_NUM_CNFG		(CHG_REG_CNFG_LAST - CHG_REG_CNFG_FIRST + 1)

/* Fuel gauge (0x36), 16-bit little-endian registers */
#define FG_STATUS		0x00
#define  FG_STATUS_BST		BIT(3)
#define FG_VALRT_TH		0x01
#define FG_TALRT_TH		0x02
#define FG_SALRT_TH		0x03
#define FG_REPCAP		0x05
#define FG_REPSOC		0x06
#define FG_TEMP			0x08
#define FG_VCELL		0x09
#define FG_CURRENT		0x0a
#define FG_AVGCURRENT		0x0b
#define FG_FULLCAP		0x10
#define FG_FULLSOCTHR		0x13
#define FG_CYCLES		0x17
#define FG_DESIGNCAP		0x18
#define FG_AVGVCELL		0x19
#define FG_CONFIG		0x1d
#define  FG_CONFIG_TEX		BIT(8)
#define FG_ICHGTERM		0x1e
#define FG_FULLCAPNOM		0x23
#define FG_LEARNCFG		0x28
#define FG_FILTERCFG		0x29
#define FG_MISCCFG		0x2b
#define FG_QRTABLE20		0x32
#define FG_FULLCAPREP		0x35
#define FG_RCOMP		0x38
#define FG_VEMPTY		0x3a
#define FG_DISCHG_TH		0x40
#define FG_QRTABLE30		0x42
#define FG_ISYS			0x43
#define FG_TALRTTH2		0xb2
#define FG_VBYP			0xb3
#define FG_CONFIG2		0xbb
#define FG_IIN			0xd0
#define FG_VFOCV		0xfb
#define FG_VFSOC		0xff

/* 4.9 sec_charging_common.h */
#define CAP_TYPE_SCALE		2
#define CAP_TYPE_DYNAMIC_SCALE	4
#define CAP_TYPE_ATOMIC		8
#define CAP_TYPE_SKIP_ABNORMAL	16
#define CAP_MAX_CONTROL_THRESHOLD	300	/* max77705_fuelgauge.c */
#define CAP_SCALE_DEFAULT_CURRENT	1000
#define CAP_SCALE_HV_CURRENT		600

/* 4.9 enum sec_battery_cable, reported by battery/online as on 4.9 */
enum mx_cable {
	CABLE_NONE = 1,
	CABLE_TA = 3,
	CABLE_USB = 4,
	CABLE_USB_CDP = 5,
	CABLE_WIRELESS = 10,
};

/* FG registers that only the host changes: must not move while we run */
static const u8 fg_cfg_regs[] = {
	FG_VALRT_TH, FG_TALRT_TH, FG_SALRT_TH, FG_FULLSOCTHR, FG_DESIGNCAP,
	FG_CONFIG, FG_ICHGTERM, FG_LEARNCFG, FG_FILTERCFG, FG_MISCCFG,
	FG_VEMPTY, FG_DISCHG_TH, FG_TALRTTH2, FG_CONFIG2,
};

/* FG registers the gauge itself updates while learning (informational) */
static const u8 fg_learn_regs[] = {
	FG_QRTABLE20, FG_QRTABLE30, FG_RCOMP, FG_FULLCAPNOM, FG_FULLCAPREP,
	FG_CYCLES,
};

struct mx_snap {
	bool valid;
	/* raw */
	u16 fg_status, config;
	u8 int_ok, dtls0, dtls1, dtls2, cnfg00, cnfg02, cnfg04, cnfg09, cnfg10;
	int bc;			/* USBC BC_STATUS or -errno */
	/* converted, 4.9 units */
	int vcell_mv, avgvcell_mv, vfocv_mv, vbyp_mv;
	int inow_ma, iavg_ma, inow_ua, iavg_ua;
	int temp_dc;		/* 0.1 degC */
	int repsoc_dp;		/* 0.1 % */
	int rawsoc_cp;		/* 0.01 % */
	int vfsoc_dp;
	int fullcaprep_mah, fullcapnom_mah, repcap_mah;
	int cycles_reg;
	/* derived */
	bool present;
	int cable;
	int status;		/* POWER_SUPPLY_STATUS_* */
	int health;
	int charge_type;
	int capacity;		/* % shown by "battery" */
	int raw_capacity;	/* 0.1 %, after scaling */
	int input_ma;		/* CHGIN or WCIN input limit */
	int vmax_uv;
};

struct mx_ro {
	struct device *dev;
	struct i2c_client *fg;
	struct i2c_client *chg;
	struct i2c_client *pmic;
	struct i2c_adapter *adap;

	struct mutex lock;		/* snap, scaling state, counters */
	struct mx_snap snap;

	/* DT, 4.9 names in comments */
	u32 fg_resistor;		/* fuelgauge,fg_resistor */
	u32 design_cap;			/* fuelgauge,capacity (register units) */
	u32 cap_max_dt;			/* fuelgauge,capacity_max */
	u32 cap_max_margin;		/* fuelgauge,capacity_max_margin */
	u32 cap_min;			/* fuelgauge,capacity_min */
	u32 cap_calc_type;		/* fuelgauge,capacity_calculation_type */
	u32 full_check_1st;		/* cable-info full_check_current_1st */
	u32 technology;			/* battery,technology */
	s32 t_high, t_high_rec, t_low, t_low_rec;	/* *_normal */
	bool current_ma;		/* 4.9 sysfs: current_now/avg in mA */
	u32 selftest_ms;
	u32 poll_ms, poll_chg_ms;

	/* 4.9 max77705_fuelgauge_data state, all software */
	int capacity_max;
	int capacity_old;
	int standard_capacity;
	int scale_cnt;
	bool initial_update;
	int low_avg_cnt;
	bool temp_bad;			/* hysteresis for OVERHEAT/COLD */
	int temp_health;

	/* zero-write evidence */
	u16 fg_cfg0[ARRAY_SIZE(fg_cfg_regs)];
	u16 fg_learn0[ARRAY_SIZE(fg_learn_regs)];
	u8 chg_cfg0[CHG_NUM_CNFG];
	u16 temp_first;
	bool temp_changed;
	unsigned long reads, read_errors;
	unsigned long writes;		/* no write path exists; stays 0 */
	int muic_writes_at_probe;
	u8 pmic_id, pmic_rev;

	struct power_supply *psy_bat, *psy_ac, *psy_usb, *psy_wl;
	struct delayed_work monitor;
	struct delayed_work selftest;
};

/* The only bus access in this driver. */
static int mx_read(struct mx_ro *m, u16 addr, u8 reg, int size,
		   union i2c_smbus_data *d)
{
	int ret = i2c_smbus_xfer(m->adap, addr, 0, I2C_SMBUS_READ, reg, size, d);

	m->reads++;
	if (ret < 0)
		m->read_errors++;
	return ret;
}

static int mx_read_u8(struct mx_ro *m, u16 addr, u8 reg, u8 *val)
{
	union i2c_smbus_data d;
	int ret = mx_read(m, addr, reg, I2C_SMBUS_BYTE_DATA, &d);

	if (ret < 0)
		return ret;
	*val = d.byte;
	return 0;
}

/* max77705_bulk_read(2) in 4.9: data[1] << 8 | data[0] */
static int mx_fg_word(struct mx_ro *m, u8 reg, u16 *val)
{
	union i2c_smbus_data d;
	int ret = mx_read(m, MX_ADDR_FG, reg, I2C_SMBUS_WORD_DATA, &d);

	if (ret < 0)
		return ret;
	*val = d.word;
	return 0;
}

/* --- 4.9 max77705_fuelgauge.c conversions ------------------------------ */

/* max77705_fg_read_vcell(), _vfocv(), _avg_vcell(): 78.125 uV per LSB */
static int conv_vcell(u16 w)
{
	u32 t = (w & 0xfff) * 78125;
	u32 v = t / 1000000;

	t = ((w & 0xf000) >> 4) * 78125;
	return v + ((t / 1000000) << 4);
}

/* max77705_fg_read_vbyp(): 427.246 uV per LSB */
static int conv_vbyp(u16 w)
{
	u32 t = (w & 0xfff) * 427246;
	u32 v = t / 1000000;

	t = ((w & 0xf000) >> 4) * 427246;
	return v + ((t / 1000000) << 4);
}

/* max77705_fg_read_current(): 1.5625 uV / (fg_resistor * 5 mOhm) */
static int conv_current(struct mx_ro *m, u16 w, bool ua)
{
	u32 t = w;
	bool neg = t & BIT(15);
	s32 i;

	if (neg)
		t = (~t & 0xffff) + 1;
	if (ua)
		i = t * 15625 * m->fg_resistor / 100;
	else
		i = t * 15625 * m->fg_resistor / 100000;
	return neg ? -i : i;
}

/* max77705_fg_read_temp(): result in 0.1 degC */
static int conv_temp(u16 w)
{
	u8 lo = w & 0xff, hi = w >> 8;
	int t;

	if (hi & BIT(7)) {
		t = ((~hi) & 0xff) + 1;
		t *= -1000;
		t -= ((~((int)lo)) + 1) * 39 / 10;
	} else {
		t = (hi & 0x7f) * 1000;
		t += lo * 39 / 10;
	}
	return t / 100;
}

/* max77705_fg_read_soc(): 0.1 %, capped at 1000 */
static int conv_soc_dp(u16 w)
{
	int soc = (((w >> 8) * 100) + ((w & 0xff) * 100 / 256)) / 10;

	return min(soc, 1000);
}

/* max77705_fg_read_rawsoc(): 0.01 %, capped at 10000 */
static int conv_soc_cp(u16 w)
{
	int soc = ((w >> 8) * 100) + ((w & 0xff) * 100 / 256);

	return min(soc, 10000);
}

static int conv_cap_mah(struct mx_ro *m, u16 w)
{
	return w * m->fg_resistor / 2;
}

/* --- 4.9 max77705_charger.c conversions -------------------------------- */

/* max77705_get_input_current_type(): CHGIN_ILIM / WCIN_ILIM in mA */
static int conv_chgin_ilim(u8 r)
{
	r &= 0x7f;
	if (r <= 0x3)
		return 100;
	if (r >= 0x7f)
		return 3200;
	return (r + 1) * 25;
}

static int conv_wcin_ilim(u8 r)
{
	r &= 0x3f;
	if (r <= 0x3)
		return 100;
	if (r >= 0x3f)
		return 1600;
	return (r + 1) * 25;
}

/* max77705_get_float_voltage() */
static int conv_float_mv(u8 r)
{
	r &= 0x3f;
	return r <= 0x04 ? r * 50 + 4000 : (r - 4) * 10 + 4200;
}

/* max77705_get_charge_current() */
static int conv_chg_cc_ma(u8 r)
{
	r &= 0x3f;
	return r <= 0x2 ? 100 : r * 50;
}

/* max77705_get_charger_state() */
static int chg_dtls_status(u8 dtls)
{
	switch (dtls) {
	case 0x0: case 0x1: case 0x2:
		return POWER_SUPPLY_STATUS_CHARGING;
	case 0x3: case 0x4:
		return POWER_SUPPLY_STATUS_FULL;
	case 0x5: case 0x6: case 0x7:
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	case 0x8: case 0xa: case 0xb:
		return POWER_SUPPLY_STATUS_DISCHARGING;
	default:
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}
}

static const char *chg_dtls_name(u8 dtls)
{
	static const char * const n[16] = {
		"precharge", "cc", "cv", "top-off", "done", "timer-fault",
		"suspended(temp)", "off(input/junction)", "off", "?9",
		"off(wdt-susp)", "wdt-expired", "?c", "?d", "?e", "?f",
	};

	return n[dtls & 0xf];
}

/* --- 4.9 capacity pipeline (software only) ------------------------------ */

static int mx_check_capacity_max(struct mx_ro *m, int cap_max)
{
	int hi = m->cap_max_dt, lo = m->cap_max_dt - m->cap_max_margin;

	return cap_max < lo ? lo : (cap_max >= hi ? hi : cap_max);
}

/* max77705_fg_calculate_dynamic_scale() */
static void mx_dynamic_scale(struct mx_ro *m, int capacity, int rawsoc_cp)
{
	int raw = rawsoc_cp / 10;

	if (raw < (int)m->cap_max_dt - (int)m->cap_max_margin) {
		dev_info(m->dev, "dynamic scale: raw soc %d too low, skipped\n", raw);
		return;
	}
	m->capacity_max = mx_check_capacity_max(m, raw * 100 / (capacity + 1));
	m->capacity_old = capacity;
	dev_info(m->dev, "dynamic scale: capacity_max %d for capacity %d\n",
		 m->capacity_max, capacity);
}

/* max77705_fg_get_scaled_capacity(), input/output in 0.1 % */
static int mx_scaled_capacity(struct mx_ro *m, struct mx_snap *s, int raw)
{
	int denom;

	if (s->cable != CABLE_NONE && DTLS01_CHG(s->dtls1) == 0x2 /* CV */ &&
	    s->input_ma >= (s->vmax_uv > 5500000 ? CAP_SCALE_HV_CURRENT :
						    CAP_SCALE_DEFAULT_CURRENT)) {
		int curr = s->iavg_ma, topoff = m->full_check_1st;
		int thr = topoff + CAP_MAX_CONTROL_THRESHOLD;
		int max_temp = m->capacity_max;
		int span = m->capacity_max - m->cap_min;

		if (curr < thr && curr > topoff && span > 0) {
			int sc = raw < (int)m->cap_min ? 0 :
				 (raw - (int)m->cap_min) * 999 / span;

			if (!m->scale_cnt) {
				m->scale_cnt = 1;
				m->standard_capacity = sc;
			} else if (m->standard_capacity < 999) {
				int sample = ((thr - curr) *
					      (999 - m->standard_capacity)) /
					     (thr - topoff);
				int gap = sc - m->standard_capacity;

				/* 4.9: no change while TEMP >= SAMPLE */
				if (gap < sample)
					max_temp -= min(sample - gap, 5);
				m->capacity_max = max_temp;
			}
		} else {
			m->scale_cnt = 0;
		}
	}

	denom = m->capacity_max - (int)m->cap_min;
	if (denom <= 0)
		return raw;
	return raw < (int)m->cap_min ? 0 :
	       (raw - (int)m->cap_min) * 1000 / denom;
}

/* max77705_get_fuelgauge_soc() + max77705_fg_get_property(CAPACITY) */
static int mx_fg_capacity(struct mx_ro *m, struct mx_snap *s, bool charging)
{
	int soc = s->repsoc_dp, val;

	if (s->vcell_mv < 3400 && s->iavg_ma < 0 && soc <= 10)
		soc = 0;

	val = soc;
	if (m->cap_calc_type & (CAP_TYPE_SCALE | CAP_TYPE_DYNAMIC_SCALE))
		val = mx_scaled_capacity(m, s, soc);
	val = clamp(val, 0, 1000);
	s->raw_capacity = val;
	val /= 10;

	if (m->initial_update) {
		m->capacity_old = val;
		m->initial_update = false;
		return val;
	}
	if (m->cap_calc_type & CAP_TYPE_ATOMIC) {
		if (m->capacity_old < val)
			val = m->capacity_old + 1;
		else if (m->capacity_old > val)
			val = m->capacity_old - 1;
	}
	if ((m->cap_calc_type & CAP_TYPE_SKIP_ABNORMAL) && !charging &&
	    m->capacity_old < val)
		val = m->capacity_old;
	m->capacity_old = val;
	return val;
}

/* sec_battery temperature health, *_normal thresholds, with recovery */
static int mx_temp_health(struct mx_ro *m, int t)
{
	if (m->temp_health == POWER_SUPPLY_HEALTH_OVERHEAT && t <= m->t_high_rec)
		m->temp_health = POWER_SUPPLY_HEALTH_GOOD;
	else if (m->temp_health == POWER_SUPPLY_HEALTH_COLD && t >= m->t_low_rec)
		m->temp_health = POWER_SUPPLY_HEALTH_GOOD;
	if (t >= m->t_high)
		m->temp_health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (t <= m->t_low)
		m->temp_health = POWER_SUPPLY_HEALTH_COLD;
	return m->temp_health;
}

/* --- sampling ------------------------------------------------------------ */

static int mx_sample(struct mx_ro *m, struct mx_snap *s)
{
	u16 w[16];
	u8 bc;
	int ret, vbus, prev_status, i;
	static const u8 regs[] = {
		FG_STATUS, FG_VCELL, FG_AVGVCELL, FG_VFOCV, FG_CURRENT,
		FG_AVGCURRENT, FG_TEMP, FG_REPSOC, FG_VFSOC, FG_FULLCAPREP,
		FG_FULLCAPNOM, FG_REPCAP, FG_CYCLES, FG_VBYP, FG_CONFIG,
	};

	BUILD_BUG_ON(ARRAY_SIZE(regs) > ARRAY_SIZE(w));
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = mx_fg_word(m, regs[i], &w[i]);
		if (ret)
			return ret;
	}
	s->fg_status = w[0];
	s->vcell_mv = conv_vcell(w[1]);
	s->avgvcell_mv = conv_vcell(w[2]);
	s->vfocv_mv = conv_vcell(w[3]);
	s->inow_ma = conv_current(m, w[4], false);
	s->inow_ua = conv_current(m, w[4], true);
	s->iavg_ma = conv_current(m, w[5], false);
	s->iavg_ua = conv_current(m, w[5], true);
	s->present = !(w[0] & FG_STATUS_BST);
	s->temp_dc = s->present ? conv_temp(w[6]) : 200;
	s->repsoc_dp = conv_soc_dp(w[7]);
	s->rawsoc_cp = conv_soc_cp(w[7]);
	s->vfsoc_dp = conv_soc_dp(w[8]);
	s->fullcaprep_mah = conv_cap_mah(m, w[9]);
	s->fullcapnom_mah = conv_cap_mah(m, w[10]);
	s->repcap_mah = conv_cap_mah(m, w[11]);
	s->cycles_reg = w[12];
	s->vbyp_mv = conv_vbyp(w[13]);
	s->config = w[14];
	if (!m->temp_changed && w[6] != m->temp_first)
		m->temp_changed = true;

	ret = mx_read_u8(m, MX_ADDR_CHG, CHG_REG_INT_OK, &s->int_ok) ?:
	      mx_read_u8(m, MX_ADDR_CHG, CHG_REG_DTLS_00, &s->dtls0) ?:
	      mx_read_u8(m, MX_ADDR_CHG, CHG_REG_DTLS_01, &s->dtls1) ?:
	      mx_read_u8(m, MX_ADDR_CHG, CHG_REG_DTLS_02, &s->dtls2) ?:
	      mx_read_u8(m, MX_ADDR_CHG, CHG_REG_CNFG_00, &s->cnfg00) ?:
	      mx_read_u8(m, MX_ADDR_CHG, CHG_REG_CNFG_02, &s->cnfg02) ?:
	      mx_read_u8(m, MX_ADDR_CHG, CHG_REG_CNFG_04, &s->cnfg04) ?:
	      mx_read_u8(m, MX_ADDR_CHG, CHG_REG_CNFG_09, &s->cnfg09) ?:
	      mx_read_u8(m, MX_ADDR_CHG, CHG_REG_CNFG_10, &s->cnfg10);
	if (ret)
		return ret;

	/* BC1.2 result from the USB-C MCU; optional */
	ret = mx_read_u8(m, MX_ADDR_USBC, USBC_REG_BC_STATUS, &bc);
	s->bc = ret ? ret : bc;

	/* cable: max77705_chg_get_property(ONLINE) + BC1.2 type */
	if (s->int_ok & INT_OK_WCIN) {
		s->cable = CABLE_WIRELESS;
		s->input_ma = conv_wcin_ilim(s->cnfg10);
	} else if (s->int_ok & INT_OK_CHGIN) {
		s->cable = CABLE_TA;
		if (s->bc >= 0 && BC_CHGTYP(s->bc) == CHGTYP_SDP)
			s->cable = CABLE_USB;
		else if (s->bc >= 0 && BC_CHGTYP(s->bc) == CHGTYP_CDP)
			s->cable = CABLE_USB_CDP;
		s->input_ma = conv_chgin_ilim(s->cnfg09);
	} else {
		s->cable = CABLE_NONE;
		s->input_ma = 0;
	}
	s->vmax_uv = s->cable == CABLE_NONE ? 0 :
		     (s->vbyp_mv > 7500 ? 9000000 : 5000000);

	/* health: max77705_get_charging_health() + sec_battery temperature */
	vbus = s->cable == CABLE_WIRELESS ? DTLS00_WCIN(s->dtls0) :
					    DTLS00_CHGIN(s->dtls0);
	if (s->cable != CABLE_NONE && vbus == 0x2)
		s->health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else if (DTLS01_BAT(s->dtls1) == 0x5)
		s->health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else if (DTLS01_BAT(s->dtls1) == 0x2)
		s->health = POWER_SUPPLY_HEALTH_DEAD;
	else if (s->cable != CABLE_NONE && s->cable != CABLE_WIRELESS &&
		 vbus <= 0x1 && (DTLS01_CHG(s->dtls1) & 0x8) &&
		 (s->cnfg00 & CNFG00_BUCK) && (s->cnfg00 & CNFG00_CHG))
		/* 4.9 "Under voltage": unknown to healthd, reported as such */
		s->health = POWER_SUPPLY_HEALTH_UNKNOWN;
	else
		s->health = mx_temp_health(m, s->temp_dc);

	/* status: charger state while external power is present */
	prev_status = m->snap.valid ? m->snap.status : POWER_SUPPLY_STATUS_UNKNOWN;
	if (s->cable == CABLE_NONE) {
		s->status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else if (s->health == POWER_SUPPLY_HEALTH_OVERVOLTAGE ||
		   s->health == POWER_SUPPLY_HEALTH_UNKNOWN) {
		s->status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		s->status = chg_dtls_status(DTLS01_CHG(s->dtls1));
		/* power present but charger off: say so */
		if (s->status == POWER_SUPPLY_STATUS_DISCHARGING)
			s->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	s->charge_type = s->status == POWER_SUPPLY_STATUS_CHARGING ?
			 POWER_SUPPLY_CHARGE_TYPE_FAST : POWER_SUPPLY_CHARGE_TYPE_NONE;

	/* 4.9 max77705_fg_read_avg_current() low-voltage quirk */
	if (s->vcell_mv < 3500 && m->low_avg_cnt < 10 && s->iavg_ma < 0 &&
	    s->status == POWER_SUPPLY_STATUS_CHARGING) {
		s->iavg_ma = 1;
		s->iavg_ua = 1000;
		m->low_avg_cnt++;
	}

	/* sec_bat_set_charging_status(): leaving FULL updates capacity_max */
	if ((m->cap_calc_type & CAP_TYPE_DYNAMIC_SCALE) &&
	    prev_status == POWER_SUPPLY_STATUS_FULL &&
	    s->status != POWER_SUPPLY_STATUS_FULL &&
	    s->status != POWER_SUPPLY_STATUS_CHARGING)
		mx_dynamic_scale(m, 100, s->rawsoc_cp);

	s->capacity = mx_fg_capacity(m, s,
			s->status == POWER_SUPPLY_STATUS_CHARGING ||
			s->status == POWER_SUPPLY_STATUS_FULL);

	/* sec_bat_fullcharged_check(): entering FULL updates capacity_max */
	if ((m->cap_calc_type & CAP_TYPE_DYNAMIC_SCALE) &&
	    s->status == POWER_SUPPLY_STATUS_FULL &&
	    prev_status != POWER_SUPPLY_STATUS_FULL)
		mx_dynamic_scale(m, s->capacity, s->rawsoc_cp);

	/* sec_bat_get_property(CAPACITY) */
	if (s->status == POWER_SUPPLY_STATUS_FULL)
		s->capacity = 100;

	s->valid = true;
	return 0;
}

static void mx_update(struct mx_ro *m)
{
	struct mx_snap s = {};
	bool changed;
	int ret;

	mutex_lock(&m->lock);
	ret = mx_sample(m, &s);
	if (ret) {
		mutex_unlock(&m->lock);
		dev_warn_ratelimited(m->dev, "sample failed: %d\n", ret);
		return;
	}
	changed = !m->snap.valid || s.status != m->snap.status ||
		  s.cable != m->snap.cable || s.capacity != m->snap.capacity ||
		  s.health != m->snap.health || s.present != m->snap.present;
	m->snap = s;
	mutex_unlock(&m->lock);

	if (changed) {
		power_supply_changed(m->psy_bat);
		power_supply_changed(m->psy_ac);
		power_supply_changed(m->psy_usb);
		power_supply_changed(m->psy_wl);
	}
}

static void mx_monitor_work(struct work_struct *work)
{
	struct mx_ro *m = container_of(to_delayed_work(work), struct mx_ro,
				       monitor);
	u32 ms;

	mx_update(m);
	mutex_lock(&m->lock);
	ms = m->snap.cable != CABLE_NONE ? m->poll_chg_ms : m->poll_ms;
	mutex_unlock(&m->lock);
	queue_delayed_work(system_power_efficient_wq, &m->monitor,
			   msecs_to_jiffies(ms));
}

/* --- power_supply ------------------------------------------------------- */

static enum power_supply_property mx_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static int mx_bat_get(struct power_supply *psy, enum power_supply_property psp,
		      union power_supply_propval *val)
{
	struct mx_ro *m = power_supply_get_drvdata(psy);
	struct mx_snap s;

	mutex_lock(&m->lock);
	s = m->snap;
	mutex_unlock(&m->lock);
	if (!s.valid)
		return -ENODATA;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = s.status;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = s.charge_type;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = s.health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = s.present;
		break;
	case POWER_SUPPLY_PROP_ONLINE:		/* 4.9: sec_battery cable_type */
		val->intval = s.cable;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = m->technology;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = s.vcell_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		val->intval = s.avgvcell_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = s.vfocv_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = m->current_ma ? s.inow_ma : s.inow_ua;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = m->current_ma ? s.iavg_ma : s.iavg_ua;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:	/* FULLCAPREP, uAh */
		val->intval = (s.fullcaprep_mah > 0 ? s.fullcaprep_mah :
			       conv_cap_mah(m, m->design_cap)) * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = conv_cap_mah(m, m->design_cap) * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:	/* 4.9: Capacity * raw_capacity */
		val->intval = m->design_cap * s.raw_capacity;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:	/* 4.9: CYCLES / 100 */
		val->intval = s.cycles_reg / 100;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = s.capacity;
		break;
	case POWER_SUPPLY_PROP_TEMP:		/* 0.1 degC */
		val->intval = s.temp_dc;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property mx_src_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static enum power_supply_property mx_wl_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static bool mx_src_online(enum power_supply_type type, int cable)
{
	switch (type) {
	case POWER_SUPPLY_TYPE_MAINS:
		return cable == CABLE_TA;
	case POWER_SUPPLY_TYPE_USB:
		return cable == CABLE_USB || cable == CABLE_USB_CDP;
	case POWER_SUPPLY_TYPE_WIRELESS:
		return cable == CABLE_WIRELESS;
	default:
		return false;
	}
}

static int mx_src_get(struct power_supply *psy, enum power_supply_property psp,
		      union power_supply_propval *val)
{
	struct mx_ro *m = power_supply_get_drvdata(psy);
	struct mx_snap s;
	bool on;

	mutex_lock(&m->lock);
	s = m->snap;
	mutex_unlock(&m->lock);
	if (!s.valid)
		return -ENODATA;
	on = mx_src_online(psy->desc->type, s.cable);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		/* sec_ac_get_property(): no "online" during OVP/UVLO */
		val->intval = on && s.health != POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = on;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = on ? s.vmax_uv : 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = on ? s.input_ma * 1000 : 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct power_supply_desc mx_bat_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = mx_bat_props,
	.num_properties = ARRAY_SIZE(mx_bat_props),
	.get_property = mx_bat_get,
};

static const struct power_supply_desc mx_ac_desc = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = mx_src_props,
	.num_properties = ARRAY_SIZE(mx_src_props),
	.get_property = mx_src_get,
};

static const struct power_supply_desc mx_usb_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = mx_src_props,
	.num_properties = ARRAY_SIZE(mx_src_props),
	.get_property = mx_src_get,
};

static const struct power_supply_desc mx_wl_desc = {
	.name = "wireless",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = mx_wl_props,
	.num_properties = ARRAY_SIZE(mx_wl_props),
	.get_property = mx_src_get,
};

/* --- zero-write evidence and self-test ----------------------------------- */

static int mx_snapshot_cfg(struct mx_ro *m, u16 *fg_cfg, u16 *fg_learn,
			   u8 *chg_cfg)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(fg_cfg_regs); i++) {
		ret = mx_fg_word(m, fg_cfg_regs[i], &fg_cfg[i]);
		if (ret)
			return ret;
	}
	for (i = 0; i < ARRAY_SIZE(fg_learn_regs); i++) {
		ret = mx_fg_word(m, fg_learn_regs[i], &fg_learn[i]);
		if (ret)
			return ret;
	}
	for (i = 0; i < CHG_NUM_CNFG; i++) {
		ret = mx_read_u8(m, MX_ADDR_CHG, CHG_REG_CNFG_FIRST + i,
				 &chg_cfg[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static void mx_selftest_work(struct work_struct *work)
{
	struct mx_ro *m = container_of(to_delayed_work(work), struct mx_ro,
				       selftest);
	u16 fg_cfg[ARRAY_SIZE(fg_cfg_regs)], fg_learn[ARRAY_SIZE(fg_learn_regs)];
	u8 chg_cfg[CHG_NUM_CNFG];
	int fg_diff = 0, learn_diff = 0, chg_diff = 0, muic_now, i, ret;
	bool sane, pass;
	struct mx_snap s;

	mx_update(m);

	mutex_lock(&m->lock);
	s = m->snap;
	ret = mx_snapshot_cfg(m, fg_cfg, fg_learn, chg_cfg);
	if (!ret) {
		for (i = 0; i < ARRAY_SIZE(fg_cfg_regs); i++) {
			if (fg_cfg[i] == m->fg_cfg0[i])
				continue;
			fg_diff++;
			dev_warn(m->dev,
				 "battery selftest: FG cfg reg 0x%02x changed %04x -> %04x\n",
				 fg_cfg_regs[i], m->fg_cfg0[i], fg_cfg[i]);
		}
		for (i = 0; i < ARRAY_SIZE(fg_learn_regs); i++) {
			if (fg_learn[i] == m->fg_learn0[i])
				continue;
			learn_diff++;
			dev_info(m->dev,
				 "battery selftest: FG learned reg 0x%02x %04x -> %04x (gauge-internal)\n",
				 fg_learn_regs[i], m->fg_learn0[i], fg_learn[i]);
		}
		for (i = 0; i < CHG_NUM_CNFG; i++) {
			if (chg_cfg[i] == m->chg_cfg0[i])
				continue;
			chg_diff++;
			dev_warn(m->dev,
				 "battery selftest: CHG CNFG reg 0x%02x changed %02x -> %02x\n",
				 CHG_REG_CNFG_FIRST + i, m->chg_cfg0[i], chg_cfg[i]);
		}
	}
	mutex_unlock(&m->lock);

	muic_now = exynos9810_max77705_muic_write_count();

	dev_info(m->dev,
		 "battery selftest: soc=%d%% raw=%d.%d%% repsoc=%d.%d%% vfsoc=%d.%d%% cap_max=%d vcell=%dmV vavg=%dmV vocv=%dmV inow=%dmA iavg=%dmA\n",
		 s.capacity, s.raw_capacity / 10, s.raw_capacity % 10,
		 s.repsoc_dp / 10, s.repsoc_dp % 10, s.vfsoc_dp / 10,
		 s.vfsoc_dp % 10, m->capacity_max, s.vcell_mv, s.avgvcell_mv,
		 s.vfocv_mv, s.inow_ma, s.iavg_ma);
	dev_info(m->dev,
		 "battery selftest: temp=%d.%dC (FG TEMP, CONFIG=%04x TEX=%d, %s since probe) present=%d health=%d cycles=%d fullcaprep=%dmAh fullcapnom=%dmAh repcap=%dmAh design=%dmAh\n",
		 s.temp_dc / 10, abs(s.temp_dc % 10), s.config,
		 !!(s.config & FG_CONFIG_TEX),
		 m->temp_changed ? "changed" : "UNCHANGED", s.present, s.health,
		 s.cycles_reg / 100, s.fullcaprep_mah, s.fullcapnom_mah,
		 s.repcap_mah, conv_cap_mah(m, m->design_cap));
	dev_info(m->dev,
		 "battery selftest: charger int_ok=%02x dtls=%02x/%02x/%02x (chg=%x %s bat=%x chgin=%x wcin=%x autoibus=%d) cnfg00=%02x(mode=%x wdten=%d) cc=%dmA cv=%dmV ilim=%dmA vbyp=%dmV bc=%d\n",
		 s.int_ok, s.dtls0, s.dtls1, s.dtls2, DTLS01_CHG(s.dtls1),
		 chg_dtls_name(DTLS01_CHG(s.dtls1)), DTLS01_BAT(s.dtls1),
		 DTLS00_CHGIN(s.dtls0), DTLS00_WCIN(s.dtls0),
		 !!(s.dtls0 & DTLS00_AUTOIBUS), s.cnfg00, CNFG00_MODE(s.cnfg00),
		 !!(s.cnfg00 & CNFG00_WDTEN), conv_chg_cc_ma(s.cnfg02),
		 conv_float_mv(s.cnfg04), s.input_ma, s.vbyp_mv, s.bc);
	dev_info(m->dev,
		 "battery selftest: online cable=%d ac=%d usb=%d wireless=%d status=%d\n",
		 s.cable, mx_src_online(POWER_SUPPLY_TYPE_MAINS, s.cable),
		 mx_src_online(POWER_SUPPLY_TYPE_USB, s.cable),
		 mx_src_online(POWER_SUPPLY_TYPE_WIRELESS, s.cable), s.status);

	sane = s.present && s.vcell_mv >= 3000 && s.vcell_mv <= 4500 &&
	       s.capacity >= 0 && s.capacity <= 100 &&
	       s.temp_dc > -200 && s.temp_dc < 800;
	pass = !ret && sane && m->writes == 0 && fg_diff == 0 && chg_diff == 0;
	dev_info(m->dev,
		 "battery selftest: i2c reads=%lu read_errors=%lu writes=%lu (no write path compiled) fg_cfg_changed=%d chg_cnfg_changed=%d learned_changed=%d muic_opcode_writes=%d (at probe %d) sane=%d -> %s\n",
		 m->reads, m->read_errors, m->writes, fg_diff, chg_diff,
		 learn_diff, muic_now, m->muic_writes_at_probe, sane,
		 pass ? "PASS" : "FAIL");
}

/* --- probe ------------------------------------------------------------- */

static void mx_parse_dt(struct mx_ro *m)
{
	struct device_node *np = m->dev->of_node;

#define RD(prop, field, def) \
	do { if (of_property_read_u32(np, prop, &m->field)) m->field = def; } while (0)
	RD("maxim,fg-resistor", fg_resistor, 2);
	RD("maxim,design-capacity", design_cap, 3753);
	RD("maxim,capacity-max", cap_max_dt, 1000);
	RD("maxim,capacity-max-margin", cap_max_margin, 300);
	RD("maxim,capacity-min", cap_min, 0);
	RD("maxim,capacity-calculation-type", cap_calc_type, 28);
	RD("samsung,full-check-current-1st", full_check_1st, 400);
	RD("samsung,technology", technology, POWER_SUPPLY_TECHNOLOGY_LION);
	RD("samsung,poll-ms", poll_ms, 30000);
	RD("samsung,poll-charging-ms", poll_chg_ms, 10000);
	RD("samsung,selftest-ms", selftest_ms, 0);
#undef RD
	if (of_property_read_s32(np, "samsung,temp-high-threshold", &m->t_high))
		m->t_high = 500;
	if (of_property_read_s32(np, "samsung,temp-high-recovery", &m->t_high_rec))
		m->t_high_rec = 450;
	if (of_property_read_s32(np, "samsung,temp-low-threshold", &m->t_low))
		m->t_low = 0;
	if (of_property_read_s32(np, "samsung,temp-low-recovery", &m->t_low_rec))
		m->t_low_rec = 50;
	m->current_ma = of_property_read_bool(np, "samsung,legacy-current-ma");

	if (!m->fg_resistor)
		m->fg_resistor = 1;
	if (m->cap_max_margin >= m->cap_max_dt)
		m->cap_max_margin = m->cap_max_dt / 2;
	if (m->cap_min >= m->cap_max_dt - m->cap_max_margin)
		m->cap_min = 0;
	m->poll_ms = clamp(m->poll_ms, 1000U, 600000U);
	m->poll_chg_ms = clamp(m->poll_chg_ms, 1000U, 600000U);
}

static void mx_cancel(void *data)
{
	struct mx_ro *m = data;

	cancel_delayed_work_sync(&m->selftest);
	cancel_delayed_work_sync(&m->monitor);
}

static int mx_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config cfg = {};
	struct mx_ro *m;
	u16 designcap, w;
	int ret, raw;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return dev_err_probe(dev, -ENODEV, "adapter lacks SMBus reads\n");

	m = devm_kzalloc(dev, sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;
	m->dev = dev;
	m->fg = client;
	m->adap = client->adapter;
	mutex_init(&m->lock);
	INIT_DELAYED_WORK(&m->monitor, mx_monitor_work);
	INIT_DELAYED_WORK(&m->selftest, mx_selftest_work);
	i2c_set_clientdata(client, m);
	mx_parse_dt(m);

	if (client->addr != MX_ADDR_FG)
		return dev_err_probe(dev, -EINVAL, "expected FG at 0x%02x\n",
				     MX_ADDR_FG);

	/* Reserve the charger and PMIC addresses; the USB-C address belongs
	 * to the MUIC driver and is only read through the adapter.
	 */
	m->chg = devm_i2c_new_dummy_device(dev, m->adap, MX_ADDR_CHG);
	if (IS_ERR(m->chg))
		return dev_err_probe(dev, PTR_ERR(m->chg), "charger 0x%02x busy\n",
				     MX_ADDR_CHG);
	m->pmic = devm_i2c_new_dummy_device(dev, m->adap, MX_ADDR_PMIC);
	if (IS_ERR(m->pmic))
		return dev_err_probe(dev, PTR_ERR(m->pmic), "pmic 0x%02x busy\n",
				     MX_ADDR_PMIC);

	ret = mx_read_u8(m, MX_ADDR_PMIC, PMIC_REG_PMICID1, &m->pmic_id) ?:
	      mx_read_u8(m, MX_ADDR_PMIC, PMIC_REG_PMICREV, &m->pmic_rev);
	if (ret)
		return dev_err_probe(dev, ret, "MAX77705 PMIC does not answer\n");

	ret = mx_fg_word(m, FG_DESIGNCAP, &designcap) ?:
	      mx_fg_word(m, FG_TEMP, &m->temp_first);
	if (ret)
		return dev_err_probe(dev, ret, "MAX77705 fuel gauge does not answer\n");

	ret = mx_snapshot_cfg(m, m->fg_cfg0, m->fg_learn0, m->chg_cfg0);
	if (ret)
		return dev_err_probe(dev, ret, "config snapshot failed\n");
	m->muic_writes_at_probe = exynos9810_max77705_muic_write_count();

	/* 4.9 probe: capacity_max from DT, dynamic scale if raw > max */
	m->capacity_max = m->cap_max_dt;
	m->initial_update = true;
	m->temp_health = POWER_SUPPLY_HEALTH_GOOD;
	ret = mx_fg_word(m, FG_REPSOC, &w);
	if (ret)
		return ret;
	raw = conv_soc_cp(w) / 10;
	if (raw > m->capacity_max)
		mx_dynamic_scale(m, 100, conv_soc_cp(w));

	mutex_lock(&m->lock);
	{
		struct mx_snap first = {};

		ret = mx_sample(m, &first);
		if (!ret)
			m->snap = first;
	}
	mutex_unlock(&m->lock);
	if (ret)
		return dev_err_probe(dev, ret, "first sample failed\n");

	dev_info(dev,
		 "MAX77705 PMIC id %02x rev %02x; FG DesignCap %04x (DT %04x%s), CONFIG %04x (TEX=%d: temperature %s), fg_resistor %u, capacity max %d min %u type 0x%x; read-only, no writes\n",
		 m->pmic_id, m->pmic_rev, designcap, m->design_cap,
		 designcap == m->design_cap ? ", model present" : ", MISMATCH",
		 m->snap.config, !!(m->snap.config & FG_CONFIG_TEX),
		 (m->snap.config & FG_CONFIG_TEX) ? "is host-written (stale until a host writes it)" :
						    "measured by the gauge",
		 m->fg_resistor, m->capacity_max, m->cap_min, m->cap_calc_type);
	dev_info(dev,
		 "first sample: soc %d%% (repsoc %d.%d%%) vcell %dmV inow %dmA temp %d.%dC cable %d status %d chg_dtls %x(%s) cnfg00 %02x\n",
		 m->snap.capacity, m->snap.repsoc_dp / 10, m->snap.repsoc_dp % 10,
		 m->snap.vcell_mv, m->snap.inow_ma, m->snap.temp_dc / 10,
		 abs(m->snap.temp_dc % 10), m->snap.cable, m->snap.status,
		 DTLS01_CHG(m->snap.dtls1), chg_dtls_name(DTLS01_CHG(m->snap.dtls1)),
		 m->snap.cnfg00);

	cfg.drv_data = m;
	m->psy_bat = devm_power_supply_register(dev, &mx_bat_desc, &cfg);
	if (IS_ERR(m->psy_bat))
		return dev_err_probe(dev, PTR_ERR(m->psy_bat), "battery psy\n");
	m->psy_ac = devm_power_supply_register(dev, &mx_ac_desc, &cfg);
	if (IS_ERR(m->psy_ac))
		return dev_err_probe(dev, PTR_ERR(m->psy_ac), "ac psy\n");
	m->psy_usb = devm_power_supply_register(dev, &mx_usb_desc, &cfg);
	if (IS_ERR(m->psy_usb))
		return dev_err_probe(dev, PTR_ERR(m->psy_usb), "usb psy\n");
	m->psy_wl = devm_power_supply_register(dev, &mx_wl_desc, &cfg);
	if (IS_ERR(m->psy_wl))
		return dev_err_probe(dev, PTR_ERR(m->psy_wl), "wireless psy\n");

	/* devm unwinds in reverse: stop the works before the supplies go */
	ret = devm_add_action_or_reset(dev, mx_cancel, m);
	if (ret)
		return ret;

	queue_delayed_work(system_power_efficient_wq, &m->monitor,
			   msecs_to_jiffies(m->poll_chg_ms));

	if (m->selftest_ms) {
		u64 now = ktime_to_ms(ktime_get_boottime());
		u64 delay = now < m->selftest_ms ? m->selftest_ms - now : 0;

		dev_info(dev, "battery selftest armed for %u ms after boot\n",
			 m->selftest_ms);
		queue_delayed_work(system_power_efficient_wq, &m->selftest,
				   msecs_to_jiffies(delay));
	}
	return 0;
}

static const struct of_device_id mx_of_match[] = {
	{ .compatible = "samsung,exynos9810-max77705-battery-ro" },
	{ }
};
MODULE_DEVICE_TABLE(of, mx_of_match);

static struct i2c_driver mx_driver = {
	.driver = {
		.name = "max77705-ro-battery",
		.of_match_table = mx_of_match,
	},
	.probe = mx_probe,
};
module_i2c_driver(mx_driver);

MODULE_DESCRIPTION("MAX77705 read-only battery monitor (Exynos9810 Note9)");
MODULE_LICENSE("GPL");
