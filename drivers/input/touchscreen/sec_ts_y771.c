// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung sec_ts "Y771" touchscreen controller, stage 1 (read-mostly),
 * for the Galaxy Note9 (SM-N960F crownlte).
 *
 * Reference: the 4.9 vendor driver running on the phone,
 * drivers/input/touchscreen/sec_ts/y771/ (CONFIG_TOUCHSCREEN_SEC_TS_Y771),
 * DT crownlte_eur_open_26 hsi2c@104B0000/touchscreen@48.
 *
 * What this driver does NOT contain, by design:
 *  - no firmware update of any kind (4.9 sec_ts_fw.c, which can flash the IC
 *    from sec_ts_probe() -> sec_ts_firmware_update_on_probe()), no
 *    request_firmware(), no FLASH_ERASE/FLASH_WRITE/ENTER_FW_MODE opcodes;
 *  - no calibration, TCLM or NVM access (4.9 sec_ts_read_info_work(),
 *    SEC_TS_CMD_NVM, factory/raw-data tests), no sponge library writes;
 *  - no regulator framework use and no PMIC access other than the two
 *    touch rail registers below.
 *
 * Power (stage 2, DT samsung,acpm-touch-power, default off). SBoot leaves
 * the IC unpowered: S2MPS18 L35CTRL 0x6c = 0x2c and L43CTRL 0x74 = 0x30
 * (vsel 1.8 V / 3.0 V, enable bits 7:6 clear). With the property set, and
 * the S2MPS18 node's samsung,touch-rail-update policy, probe does what 4.9
 * sec_ts_power() + s2m_enable() did through ACPM:
 *   read 0x6c/0x74; require vsel 0x2c/0x30 and modes 00 or the 4.9 mode;
 *   LDO35 tsp_io: update 0x6c val 0x80 mask 0xc0, read back 0xac; 1-2 ms;
 *   LDO43 tsp_avdd (only if 0x6c reads 0xac): update 0x74 val 0xc0 mask
 *   0xc0, read back 0xf0; 70 ms (4.9 sec_ts_start_device()); then wait for
 *   the IC's boot-complete event as 4.9 sec_ts_wait_for_ready().
 * Off (only on a probe failure and at shutdown): LDO43 val 0x00, 4 ms,
 * LDO35 val 0x00. At most two power-on sequences per boot. The ACPM
 * transport (acpm_mfd.c) enforces the same values, order and limit.
 *
 * Every I2C transaction goes through sec_ts_read()/sec_ts_write(), which
 * refuse any opcode outside two fixed allowlists. The only opcodes ever
 * written are the two runtime commands 4.9 sends at normal boot to start
 * reporting: SET_TOUCHFUNCTION (0x30, 2 bytes, only if the default bits are
 * missing) and SENSE_ON (0x10, no data). 4.9 re-sends both after every
 * power-on (sec_ts_start_device()) and after an IC watchdog reset
 * (sec_ts_read_event()), i.e. the IC does not retain them.
 *
 * Like 4.9 on this bus (DT samsung,stop-after-trans), every register read is
 * sent as two separate transfers: the 1-byte opcode write ends with STOP,
 * then the data read.
 */

#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/ratelimit.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <soc/samsung/acpm_mfd.h>

#define SEC_TS_DRV_NAME			"sec_ts"
/* 4.9 sec_ts.c:2501 input_dev->name, kept so userspace sees the 4.9 name. */
#define SEC_TS_INPUT_NAME		"sec_touchscreen"

/* Opcodes, 4.9 y771/sec_ts.h:202-284 */
#define SEC_TS_CMD_SENSE_ON		0x10
#define SEC_TS_READ_FIRMWARE_INTEGRITY	0x21
#define SEC_TS_READ_DEVICE_ID		0x22
#define SEC_TS_READ_PANEL_INFO		0x23
#define SEC_TS_CMD_SET_TOUCHFUNCTION	0x30	/* write = set, read = get */
#define SEC_TS_READ_ID			0x52
#define SEC_TS_READ_BOOT_STATUS		0x55
#define SEC_TS_READ_ONE_EVENT		0x60
#define SEC_TS_READ_ALL_EVENT		0x61
#define SEC_TS_READ_FW_VERSION		0xA3
#define SEC_TS_READ_PARA_VERSION	0xA4
#define SEC_TS_READ_IMG_VERSION		0xA5
#define SEC_TS_READ_TS_STATUS		0xAF

/* 4.9 y771/sec_ts.h:324-414 */
#define SEC_TS_STATUS_BOOT_MODE		0x10
#define SEC_TS_STATUS_APP_MODE		0x20
#define TOUCH_SYSTEM_MODE_FLASH		4
#define SEC_TS_EVENT_BUFF_SIZE		8
#define SEC_TS_COORDINATE_EVENT		0
#define SEC_TS_STATUS_EVENT		1
#define SEC_TS_GESTURE_EVENT		2
#define SEC_TS_ACTION_PRESS		1
#define SEC_TS_ACTION_MOVE		2
#define SEC_TS_ACTION_RELEASE		3
#define SEC_TS_TOUCHTYPE_NORMAL		0
#define SEC_TS_TOUCHTYPE_GLOVE		3
#define SEC_TS_TOUCHTYPE_PALM		5
#define SEC_TS_TOUCHTYPE_WET		6
#define TYPE_STATUS_EVENT_ERR		1
#define TYPE_STATUS_EVENT_INFO		2
#define TYPE_STATUS_EVENT_VENDOR_INFO	7
#define SEC_TS_ACK_BOOT_COMPLETE	0x00
#define SEC_TS_ERR_EVENT_QUEUE_FULL	0x01
#define SEC_TS_ERR_EVENT_ESD		0x02
/* TOUCH | PALM | WET, 4.9 SEC_TS_DEFAULT_ENABLE_BIT_SETFUNC */
#define SEC_TS_DEFAULT_SETFUNC		(BIT(0) | BIT(5) | BIT(6))

#define SEC_TS_MAX_TOUCH		10	/* 4.9 MAX_SUPPORT_TOUCH_COUNT */
#define SEC_TS_MAX_EVENTS		32	/* 4.9 sec_ts.c:958 MAX_EVENT_COUNT */
#define SEC_TS_I2C_RETRY		3	/* 4.9 SEC_TS_I2C_RETRY_CNT */
#define SEC_TS_BURST_MAX		256	/* 4.9 default sec,i2c-burstmax */
#define SEC_TS_IDLE_IRQ_LIMIT		1000	/* consecutive IRQs without events */
#define SEC_TS_LOG_TOUCHES		32	/* first N press/release logged */
#define SEC_TS_SELFTEST_MS		109000
#define SEC_TS_WAIT_RETRY		100	/* 4.9 SEC_TS_WAIT_RETRY_CNT, x 20 ms */
#define SEC_TS_MAX_POWER_ON		2	/* power-on sequences per boot */

enum sec_ts_state {
	SEC_TS_ABSENT,		/* no ACK: IC not powered or not there */
	SEC_TS_REFUSED,		/* IC answers but firmware is not usable */
	SEC_TS_FAILED,		/* setup command failed */
	SEC_TS_RUNNING,
};

static const char * const sec_ts_state_names[] = {
	[SEC_TS_ABSENT] = "absent",
	[SEC_TS_REFUSED] = "refused",
	[SEC_TS_FAILED] = "failed",
	[SEC_TS_RUNNING] = "running",
};

struct sec_ts {
	struct i2c_client *client;
	struct input_dev *input;
	struct gpio_desc *irq_gpio;
	struct mutex i2c_lock;		/* one command + data pair at a time */
	struct mutex event_lock;	/* IRQ thread vs. self-test */
	struct delayed_work selftest_work;
	enum sec_ts_state state;
	bool selftest;
	bool power_ctl;			/* DT samsung,acpm-touch-power */
	bool powered;			/* rails on (by this driver) */
	int power_err;			/* last power-on result */

	u8 device_id[5];
	u8 img_ver[4], core_ver[4], para_ver[4];
	u8 touch_func[2];
	u16 max_x, max_y;		/* IC panel info, 0 if not reported */
	u8 tx_count, rx_count;
	bool irq_disabled;

	u8 *buf;			/* SEC_TS_MAX_EVENTS * 8, DMA-safe */
	u8 cmd;				/* opcode byte, DMA-safe via kzalloc */
	u8 wbuf[3];

	/* statistics, updated under event_lock */
	unsigned int irqs, idle_irqs, idle_run, overflows, i2c_errors;
	unsigned int press, move, release, status_events, gesture_events;
	unsigned int ic_resets, logged;
	unsigned int writes_sense_on, writes_touchfunc, refused;
	unsigned long active;		/* bitmap of slots down */
};

/*
 * Allowlists. A read is "write opcode, STOP, read data"; the opcode byte is
 * the only byte the host sends. 0x60/0x61 pop events from the IC's RAM
 * event queue (4.9 does this in every interrupt). Everything else is a
 * status/ID/version query.
 */
static bool sec_ts_read_allowed(u8 op)
{
	switch (op) {
	case SEC_TS_READ_FIRMWARE_INTEGRITY:
	case SEC_TS_READ_DEVICE_ID:
	case SEC_TS_READ_PANEL_INFO:
	case SEC_TS_CMD_SET_TOUCHFUNCTION:
	case SEC_TS_READ_ID:
	case SEC_TS_READ_BOOT_STATUS:
	case SEC_TS_READ_ONE_EVENT:
	case SEC_TS_READ_ALL_EVENT:
	case SEC_TS_READ_FW_VERSION:
	case SEC_TS_READ_PARA_VERSION:
	case SEC_TS_READ_IMG_VERSION:
	case SEC_TS_READ_TS_STATUS:
		return true;
	default:
		return false;
	}
}

static bool sec_ts_write_allowed(u8 op, int len)
{
	return (op == SEC_TS_CMD_SENSE_ON && len == 0) ||
	       (op == SEC_TS_CMD_SET_TOUCHFUNCTION && len == 2);
}

static int sec_ts_xfer_one(struct sec_ts *ts, struct i2c_msg *msg)
{
	int ret = i2c_transfer(ts->client->adapter, msg, 1);

	if (ret == 1)
		return 0;
	return ret < 0 ? ret : -EIO;
}

static int sec_ts_read(struct sec_ts *ts, u8 op, u8 *data, int len)
{
	struct i2c_msg wr = {
		.addr = ts->client->addr, .flags = 0, .len = 1, .buf = &ts->cmd,
	};
	struct i2c_msg rd = {
		.addr = ts->client->addr, .flags = I2C_M_RD, .len = len,
		.buf = ts->buf,
	};
	int retry, ret = -EIO;

	if (!sec_ts_read_allowed(op) || len <= 0 ||
	    len > SEC_TS_MAX_EVENTS * SEC_TS_EVENT_BUFF_SIZE ||
	    len > SEC_TS_BURST_MAX) {
		ts->refused++;
		dev_err(&ts->client->dev, "refused read op %#04x len %d\n", op, len);
		return -EPERM;
	}

	mutex_lock(&ts->i2c_lock);
	for (retry = 0; retry < SEC_TS_I2C_RETRY; retry++) {
		ts->cmd = op;
		ret = sec_ts_xfer_one(ts, &wr);
		if (!ret)
			ret = sec_ts_xfer_one(ts, &rd);
		if (!ret)
			break;
		usleep_range(1000, 1100);
	}
	if (!ret)
		memcpy(data, ts->buf, len);
	else
		ts->i2c_errors++;
	mutex_unlock(&ts->i2c_lock);

	return ret;
}

static int sec_ts_write(struct sec_ts *ts, u8 op, const u8 *data, int len)
{
	struct i2c_msg wr = {
		.addr = ts->client->addr, .flags = 0, .len = len + 1,
		.buf = ts->wbuf,
	};
	int retry, ret = -EIO;

	if (!sec_ts_write_allowed(op, len) || len >= sizeof(ts->wbuf)) {
		ts->refused++;
		dev_err(&ts->client->dev, "refused write op %#04x len %d\n", op, len);
		return -EPERM;
	}

	mutex_lock(&ts->i2c_lock);
	ts->wbuf[0] = op;
	if (len)
		memcpy(&ts->wbuf[1], data, len);
	for (retry = 0; retry < SEC_TS_I2C_RETRY; retry++) {
		ret = sec_ts_xfer_one(ts, &wr);
		if (!ret)
			break;
		usleep_range(1000, 1100);
	}
	if (ret)
		ts->i2c_errors++;
	else if (op == SEC_TS_CMD_SENSE_ON)
		ts->writes_sense_on++;
	else
		ts->writes_touchfunc++;
	mutex_unlock(&ts->i2c_lock);

	if (len)
		dev_info(&ts->client->dev, "wrote op %#04x data %*ph: %d\n", op,
			 len, data, ret);
	else
		dev_info(&ts->client->dev, "wrote op %#04x: %d\n", op, ret);
	return ret;
}

static const char *sec_ts_family(u8 img0)
{
	/* 4.9 sec_ts_fn.c get_chip_name() */
	switch (img0) {
	case 0x02: return "MC44";
	case 0x05: return "A552";
	case 0x09: return "Y661";
	case 0x10: return "Y761";
	case 0x17: return "Y771";
	default: return "unknown";
	}
}

static int sec_ts_read_versions(struct sec_ts *ts)
{
	int ret;

	/* 4.9 sec_ts_fw.c:146-174 sec_ts_save_version_of_ic() */
	ret = sec_ts_read(ts, SEC_TS_READ_IMG_VERSION, ts->img_ver, 4);
	if (!ret)
		ret = sec_ts_read(ts, SEC_TS_READ_FW_VERSION, ts->core_ver, 4);
	if (!ret)
		ret = sec_ts_read(ts, SEC_TS_READ_PARA_VERSION, ts->para_ver, 4);
	return ret;
}

static void sec_ts_release_all(struct sec_ts *ts)
{
	int i;

	if (!ts->input)
		return;
	for (i = 0; i < SEC_TS_MAX_TOUCH; i++) {
		if (!test_and_clear_bit(i, &ts->active))
			continue;
		input_mt_slot(ts->input, i);
		input_mt_report_slot_inactive(ts->input);
	}
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

static void sec_ts_coord_event(struct sec_ts *ts, const u8 *ev)
{
	struct device *dev = &ts->client->dev;
	/* 4.9 struct sec_ts_event_coordinate (sec_ts.h:664), LSB-first fields */
	unsigned int tid = ((ev[0] >> 2) & 0xf) - 1;
	unsigned int action = (ev[0] >> 6) & 0x3;
	unsigned int x = (ev[1] << 4) | (ev[3] >> 4);
	unsigned int y = (ev[2] << 4) | (ev[3] & 0xf);
	unsigned int major = ev[4], minor = ev[5];
	unsigned int ttype = ((ev[6] >> 6) << 2) | (ev[7] >> 6);

	if (tid >= SEC_TS_MAX_TOUCH)
		return;		/* hover id or tid 0 */
	if (ttype != SEC_TS_TOUCHTYPE_NORMAL && ttype != SEC_TS_TOUCHTYPE_PALM &&
	    ttype != SEC_TS_TOUCHTYPE_WET && ttype != SEC_TS_TOUCHTYPE_GLOVE)
		return;		/* 4.9 sec_ts.c:1202-1205 */

	input_mt_slot(ts->input, tid);
	if (action == SEC_TS_ACTION_RELEASE) {
		input_mt_report_slot_inactive(ts->input);
		clear_bit(tid, &ts->active);
		ts->release++;
		if (ts->logged < SEC_TS_LOG_TOUCHES) {
			ts->logged++;
			dev_info(dev, "[R] tID:%u x:%u y:%u\n", tid, x, y);
		}
		return;
	}
	if (action != SEC_TS_ACTION_PRESS && action != SEC_TS_ACTION_MOVE)
		return;

	input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
	input_report_abs(ts->input, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, major);
	input_report_abs(ts->input, ABS_MT_TOUCH_MINOR, minor);
	set_bit(tid, &ts->active);
	if (action == SEC_TS_ACTION_MOVE) {
		ts->move++;
		return;
	}
	ts->press++;
	if (ts->logged < SEC_TS_LOG_TOUCHES) {
		ts->logged++;
		dev_info(dev, "[P] tID:%u x:%u y:%u major:%u minor:%u type:%u\n",
			 tid, x, y, major, minor, ttype);
	}
}

static void sec_ts_status_event(struct sec_ts *ts, const u8 *ev)
{
	static DEFINE_RATELIMIT_STATE(rs, 5 * HZ, 10);
	struct device *dev = &ts->client->dev;
	unsigned int stype = (ev[0] >> 2) & 0xf;

	ts->status_events++;
	if (stype && __ratelimit(&rs))
		dev_info(dev, "status event %8ph\n", ev);

	/* 4.9 sec_ts.c:1063-1073: IC watchdog reset -> SENSE_ON + reinit */
	if (stype == TYPE_STATUS_EVENT_INFO && ev[1] == SEC_TS_ACK_BOOT_COMPLETE &&
	    ev[2] == 0x20) {
		ts->ic_resets++;
		dev_warn(dev, "IC reset reported, re-sending SENSE_ON and touch functions\n");
		sec_ts_release_all(ts);
		sec_ts_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
		sec_ts_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, ts->touch_func, 2);
		return;
	}
	if (stype == TYPE_STATUS_EVENT_ERR && ev[1] == SEC_TS_ERR_EVENT_QUEUE_FULL) {
		dev_warn(dev, "IC event queue full\n");
		sec_ts_release_all(ts);
	}
	/* 4.9 schedules a power-cycle reset on ESD; not done here. */
	if (stype == TYPE_STATUS_EVENT_ERR && ev[1] == SEC_TS_ERR_EVENT_ESD)
		dev_warn(dev, "IC reported ESD (no runtime power-cycle reset, not resetting)\n");
}

/* Returns the number of events handled, 0 for an empty queue, or -errno. */
static int sec_ts_read_events(struct sec_ts *ts)
{
	u8 ev[SEC_TS_MAX_EVENTS][SEC_TS_EVENT_BUFF_SIZE];
	unsigned int left, i;
	int ret;

	ret = sec_ts_read(ts, SEC_TS_READ_ONE_EVENT, ev[0], SEC_TS_EVENT_BUFF_SIZE);
	if (ret)
		return ret;
	if (!ev[0][0])
		return 0;

	left = ev[0][7] & 0x3f;
	if (left > SEC_TS_MAX_EVENTS - 1) {
		/*
		 * 4.9 sends CLEAR_EVENT_STACK here. Instead handle the one event
		 * already popped; the level IRQ stays asserted and each further
		 * READ_ONE_EVENT pops one more until the backlog fits.
		 */
		ts->overflows++;
		left = 0;
	} else if (left) {
		ret = sec_ts_read(ts, SEC_TS_READ_ALL_EVENT, ev[1],
				  left * SEC_TS_EVENT_BUFF_SIZE);
		if (ret)
			return ret;
	}

	for (i = 0; i <= left; i++) {
		switch (ev[i][0] & 0x3) {
		case SEC_TS_COORDINATE_EVENT:
			sec_ts_coord_event(ts, ev[i]);
			break;
		case SEC_TS_STATUS_EVENT:
			sec_ts_status_event(ts, ev[i]);
			break;
		case SEC_TS_GESTURE_EVENT:
			ts->gesture_events++;
			break;
		default:
			break;
		}
	}
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);

	return left + 1;
}

static irqreturn_t sec_ts_irq_thread(int irq, void *data)
{
	struct sec_ts *ts = data;
	int n;

	mutex_lock(&ts->event_lock);
	ts->irqs++;
	n = sec_ts_read_events(ts);
	if (n > 0) {
		ts->idle_run = 0;
	} else {
		ts->idle_irqs++;
		if (++ts->idle_run >= SEC_TS_IDLE_IRQ_LIMIT && !ts->irq_disabled) {
			ts->irq_disabled = true;
			disable_irq_nosync(irq);
			dev_err(&ts->client->dev,
				"%u consecutive interrupts without events (last %d), IRQ disabled\n",
				ts->idle_run, n);
		}
	}
	mutex_unlock(&ts->event_lock);

	return IRQ_HANDLED;
}

/* 4.9 sec_ts_wait_for_ready(): consume pending status events at probe. */
static void sec_ts_drain_pending(struct sec_ts *ts)
{
	u8 ev[SEC_TS_EVENT_BUFF_SIZE];
	int i;

	for (i = 0; i < SEC_TS_MAX_EVENTS; i++) {
		if (gpiod_get_raw_value_cansleep(ts->irq_gpio))
			break;		/* line high: nothing pending */
		if (sec_ts_read(ts, SEC_TS_READ_ONE_EVENT, ev, sizeof(ev)))
			break;
		dev_info(&ts->client->dev, "pending event at probe: %8ph\n", ev);
		if (!ev[0])
			break;
	}
}

/*
 * Touch rails through the ACPM PMIC transport. Built-in only: the
 * exynos_acpm_* accessors are not exported to modules.
 */
#if IS_BUILTIN(CONFIG_TOUCHSCREEN_SEC_TS_Y771) && IS_ENABLED(CONFIG_EXYNOS_ACPM)
#define SEC_TS_L35_ON_VAL	(ACPM_MFD_TOUCH_L35_VSEL | ACPM_MFD_TOUCH_L35_ON)	/* 0xac */
#define SEC_TS_L43_ON_VAL	(ACPM_MFD_TOUCH_L43_VSEL | ACPM_MFD_TOUCH_L43_ON)	/* 0xf0 */

static DEFINE_MUTEX(sec_ts_power_lock);
static unsigned int sec_ts_power_ons;	/* per boot, under sec_ts_power_lock */

static int sec_ts_rail_read(u8 reg, u8 *val)
{
	return exynos_acpm_read_reg(ACPM_MFD_TYPE_PMIC, reg, val);
}

static int sec_ts_rail_update(u8 reg, u8 val)
{
	return exynos_acpm_update_reg(ACPM_MFD_TYPE_PMIC, reg, val,
				      ACPM_MFD_TOUCH_RAIL_MASK);
}

static int sec_ts_rails_read(u8 *l35, u8 *l43)
{
	return sec_ts_rail_read(ACPM_MFD_TOUCH_L35_REG, l35) ?:
	       sec_ts_rail_read(ACPM_MFD_TOUCH_L43_REG, l43);
}

static bool sec_ts_rail_ok(u8 v, u8 vsel, u8 on)
{
	u8 mode = v & ACPM_MFD_TOUCH_RAIL_MASK;

	return (v & ACPM_MFD_TOUCH_VSEL_MASK) == vsel &&
	       (mode == ACPM_MFD_TOUCH_RAIL_OFF || mode == on);
}

/* 4.9 sec_ts_power(off): avdd off, 4 ms, dvdd off. */
static void sec_ts_power_off(struct sec_ts *ts, const char *why)
{
	struct device *dev = &ts->client->dev;
	u8 l35 = 0, l43 = 0;
	int r43, r35 = -EPERM, ret;

	if (!ts->power_ctl || !ts->powered)
		return;

	mutex_lock(&sec_ts_power_lock);
	r43 = sec_ts_rail_update(ACPM_MFD_TOUCH_L43_REG, ACPM_MFD_TOUCH_RAIL_OFF);
	usleep_range(4000, 5000);
	ret = sec_ts_rail_read(ACPM_MFD_TOUCH_L43_REG, &l43);
	/* LDO35 only after LDO43 reads off (the transport checks it again) */
	if (!ret && !(l43 & ACPM_MFD_TOUCH_RAIL_MASK))
		r35 = sec_ts_rail_update(ACPM_MFD_TOUCH_L35_REG,
					 ACPM_MFD_TOUCH_RAIL_OFF);
	ret = sec_ts_rails_read(&l35, &l43);
	ts->powered = ret || ((l35 | l43) & ACPM_MFD_TOUCH_RAIL_MASK);
	mutex_unlock(&sec_ts_power_lock);

	dev_info(dev, "power off (%s): LDO43 %d, LDO35 %d; now 0x6c=%#04x 0x74=%#04x (read %d)%s\n",
		 why, r43, r35, l35, l43, ret,
		 ts->powered ? " - RAIL STILL ON" : "");
}

/* 4.9 sec_ts_power(on) + 70 ms of sec_ts_start_device(). */
static int sec_ts_power_on(struct sec_ts *ts)
{
	struct device *dev = &ts->client->dev;
	u8 l35 = 0, l43 = 0, rb = 0;
	int ret;

	mutex_lock(&sec_ts_power_lock);
	ret = sec_ts_rails_read(&l35, &l43);
	if (ret) {
		dev_err(dev, "power: rail read failed %d, not powering\n", ret);
		goto unlock;
	}
	dev_info(dev, "power: before 0x6c=%#04x 0x74=%#04x, power-ons so far %u\n",
		 l35, l43, sec_ts_power_ons);

	ret = -EINVAL;
	if (!sec_ts_rail_ok(l35, ACPM_MFD_TOUCH_L35_VSEL, ACPM_MFD_TOUCH_L35_ON) ||
	    !sec_ts_rail_ok(l43, ACPM_MFD_TOUCH_L43_VSEL, ACPM_MFD_TOUCH_L43_ON)) {
		dev_err(dev, "power: unexpected rail state (want vsel 0x2c/0x30, modes 00 or 10/11), not powering\n");
		goto unlock;
	}
	if ((l43 & ACPM_MFD_TOUCH_RAIL_MASK) && l35 != SEC_TS_L35_ON_VAL) {
		dev_err(dev, "power: LDO43 on while LDO35 off, not touching it\n");
		goto unlock;
	}
	if (l35 == SEC_TS_L35_ON_VAL && l43 == SEC_TS_L43_ON_VAL) {
		dev_info(dev, "power: both rails already on, nothing written\n");
		ts->powered = true;
		ret = 0;
		goto unlock;
	}
	if (sec_ts_power_ons >= SEC_TS_MAX_POWER_ON) {
		dev_err(dev, "power: %u power-ons this boot, no more\n",
			sec_ts_power_ons);
		ret = -EBUSY;
		goto unlock;
	}
	sec_ts_power_ons++;
	/* from here a failure leaves something to switch off */
	ts->powered = true;

	if (l35 != SEC_TS_L35_ON_VAL) {
		ret = sec_ts_rail_update(ACPM_MFD_TOUCH_L35_REG,
					 ACPM_MFD_TOUCH_L35_ON);
		usleep_range(1000, 2000);	/* 4.9 sec_ts_delay(1) */
		if (!ret)
			ret = sec_ts_rail_read(ACPM_MFD_TOUCH_L35_REG, &rb);
		if (!ret && rb != SEC_TS_L35_ON_VAL)
			ret = -EIO;
		dev_info(dev, "power: LDO35 tsp_io on: %d, 0x6c=%#04x\n", ret, rb);
		if (ret)
			goto fail;
	}

	/* LDO43 only while LDO35 reads on */
	rb = 0;
	ret = sec_ts_rail_read(ACPM_MFD_TOUCH_L35_REG, &rb);
	if (!ret && rb != SEC_TS_L35_ON_VAL)
		ret = -EIO;
	if (ret) {
		dev_err(dev, "power: LDO35 not on before LDO43 (%d, 0x6c=%#04x)\n",
			ret, rb);
		goto fail;
	}
	rb = 0;
	ret = sec_ts_rail_update(ACPM_MFD_TOUCH_L43_REG, ACPM_MFD_TOUCH_L43_ON);
	if (!ret)
		ret = sec_ts_rail_read(ACPM_MFD_TOUCH_L43_REG, &rb);
	if (!ret && rb != SEC_TS_L43_ON_VAL)
		ret = -EIO;
	dev_info(dev, "power: LDO43 tsp_avdd on: %d, 0x74=%#04x\n", ret, rb);
	if (ret)
		goto fail;
	mutex_unlock(&sec_ts_power_lock);

	msleep(70);		/* 4.9 sec_ts_start_device() */
	return 0;

fail:
	mutex_unlock(&sec_ts_power_lock);
	sec_ts_power_off(ts, "power-on failed");
	return ret;
unlock:
	mutex_unlock(&sec_ts_power_lock);
	return ret;
}

static int sec_ts_rails_now(u8 *l35, u8 *l43)
{
	return sec_ts_rails_read(l35, l43);
}

/*
 * 4.9 sec_ts_wait_for_ready(SEC_TS_ACK_BOOT_COMPLETE): while the IRQ line
 * is low, pop events until the boot-complete status event, 20 ms apart.
 * Read-only (READ_ONE_EVENT); a timeout is logged and probe goes on.
 */
static void sec_ts_wait_for_ready(struct sec_ts *ts)
{
	u8 ev[SEC_TS_EVENT_BUFF_SIZE] = { 0 };
	unsigned int stype;
	int retry;

	for (retry = 0; retry <= SEC_TS_WAIT_RETRY; retry++) {
		if (!gpiod_get_raw_value_cansleep(ts->irq_gpio) &&
		    !sec_ts_read(ts, SEC_TS_READ_ONE_EVENT, ev, sizeof(ev))) {
			stype = (ev[0] >> 2) & 0xf;
			if ((stype == TYPE_STATUS_EVENT_INFO ||
			     stype == TYPE_STATUS_EVENT_VENDOR_INFO) &&
			    ev[1] == SEC_TS_ACK_BOOT_COMPLETE) {
				dev_info(&ts->client->dev,
					 "boot complete after %d polls: %8ph\n",
					 retry, ev);
				return;
			}
		}
		msleep(20);
	}
	dev_warn(&ts->client->dev, "no boot-complete event in %d ms (last %8ph)\n",
		 SEC_TS_WAIT_RETRY * 20, ev);
}

/*
 * DT samsung,acpm-touch-power: power the IC before identifying it. Returns
 * -EPROBE_DEFER until the S2MPS18 driver has sealed the ACPM write policy;
 * any other failure leaves the IC unpowered and probe reports "absent".
 */
static int sec_ts_power_up(struct sec_ts *ts)
{
	struct device *dev = &ts->client->dev;
	int policy = exynos_acpm_mfd_touch_rails_policy();

	if (policy == -EAGAIN)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "waiting for the S2MPS18 ACPM write policy\n");
	if (policy != 1) {
		dev_err(dev, "power: ACPM policy refuses touch rails (%d; S2MPS18 node needs samsung,touch-rail-update), not powering\n",
			policy);
		return -EPERM;
	}
	ts->power_err = sec_ts_power_on(ts);
	if (ts->power_err)
		return ts->power_err;
	sec_ts_wait_for_ready(ts);
	return 0;
}
#else
/*
 * Module build or no ACPM: no power path at all (the exynos_acpm_*
 * accessors are not exported), so the DT property only logs.
 */
static void sec_ts_power_off(struct sec_ts *ts, const char *why)
{
}

static int sec_ts_rails_now(u8 *l35, u8 *l43)
{
	return -EOPNOTSUPP;
}

static int sec_ts_power_up(struct sec_ts *ts)
{
	dev_err(&ts->client->dev,
		"power: samsung,acpm-touch-power needs a built-in driver and CONFIG_EXYNOS_ACPM, not powering\n");
	ts->power_err = -EOPNOTSUPP;
	return -EOPNOTSUPP;
}
#endif

static int sec_ts_identify(struct sec_ts *ts)
{
	struct device *dev = &ts->client->dev;
	u8 integrity = 0, boot = 0, status[4] = { 0 };
	int ret;

	ret = sec_ts_read(ts, SEC_TS_READ_DEVICE_ID, ts->device_id, 5);
	if (ret) {
		dev_err(dev,
			"no answer at %#04x (%d): IC not powered? Needs S2MPS18 LDO35 1.8 V (tsp_io) then LDO43 3.0 V (tsp_avdd); power control %s, stopping\n",
			ts->client->addr, ret,
			!ts->power_ctl ? "off (no samsung,acpm-touch-power)" :
			ts->powered ? "on, rails on" : "on, rails NOT on");
		ts->state = SEC_TS_ABSENT;
		return -ENODEV;
	}
	dev_info(dev, "device id %5ph\n", ts->device_id);
	if (!memchr_inv(ts->device_id, 0, 5) || !memchr_inv(ts->device_id, 0xff, 5)) {
		dev_err(dev, "implausible device id, stopping\n");
		ts->state = SEC_TS_REFUSED;
		return -ENODEV;
	}

	ret = sec_ts_read(ts, SEC_TS_READ_FIRMWARE_INTEGRITY, &integrity, 1);
	if (!ret)
		ret = sec_ts_read(ts, SEC_TS_READ_BOOT_STATUS, &boot, 1);
	if (!ret)
		ret = sec_ts_read(ts, SEC_TS_READ_TS_STATUS, status, 4);
	if (!ret)
		ret = sec_ts_read_versions(ts);
	if (ret) {
		dev_err(dev, "status/version read failed: %d\n", ret);
		ts->state = SEC_TS_FAILED;
		return ret;
	}
	dev_info(dev, "integrity %#04x boot %#04x status %4ph\n", integrity, boot, status);
	dev_info(dev, "chip %s, image %4ph core %4ph config %4ph\n",
		 sec_ts_family(ts->img_ver[0]), ts->img_ver, ts->core_ver, ts->para_ver);

	/*
	 * 4.9 sec_ts.c:2414-2456 forces a firmware flash in these cases.
	 * Stage 1 never flashes: refuse and leave the IC alone.
	 */
	if (!(integrity & 0x80) || boot != SEC_TS_STATUS_APP_MODE ||
	    status[1] == TOUCH_SYSTEM_MODE_FLASH) {
		dev_err(dev,
			"IC firmware not in application mode (4.9 would re-flash here); not touching it\n");
		ts->state = SEC_TS_REFUSED;
		return -ENODEV;
	}
	return 0;
}

static int sec_ts_start(struct sec_ts *ts)
{
	struct device *dev = &ts->client->dev;
	u8 info[11] = { 0 }, id[3] = { 0 };
	int ret;

	sec_ts_drain_pending(ts);

	/* 4.9 sec_ts_read_information(), sec_ts.c:2055-2140 */
	ret = sec_ts_read(ts, SEC_TS_READ_ID, id, 3);
	if (!ret)
		ret = sec_ts_read(ts, SEC_TS_READ_PANEL_INFO, info, 11);
	if (!ret)
		ret = sec_ts_read(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, ts->touch_func, 2);
	if (ret) {
		dev_err(dev, "information read failed: %d\n", ret);
		return ret;
	}
	ts->max_x = (info[0] << 8) | info[1];
	ts->max_y = (info[2] << 8) | info[3];
	ts->tx_count = info[8];
	ts->rx_count = info[9];
	dev_info(dev, "id %3ph, panel %ux%u display %ux%u tx %u rx %u, functions %2ph\n",
		 id, ts->max_x, ts->max_y, (info[4] << 8) | info[5],
		 (info[6] << 8) | info[7], ts->tx_count, ts->rx_count, ts->touch_func);

	/* 4.9 sec_ts.c:2469-2470 */
	if ((ts->touch_func[0] & SEC_TS_DEFAULT_SETFUNC) != SEC_TS_DEFAULT_SETFUNC) {
		ts->touch_func[0] |= SEC_TS_DEFAULT_SETFUNC;
		ret = sec_ts_write(ts, SEC_TS_CMD_SET_TOUCHFUNCTION, ts->touch_func, 2);
		if (ret)
			dev_err(dev, "SET_TOUCHFUNCTION failed: %d\n", ret);
	} else {
		dev_info(dev, "touch functions already enabled, not written\n");
	}

	/* 4.9 sec_ts.c:2483-2484 */
	ret = sec_ts_write(ts, SEC_TS_CMD_SENSE_ON, NULL, 0);
	if (ret)
		dev_err(dev, "SENSE_ON failed: %d\n", ret);
	return ret;
}

static int sec_ts_input_init(struct sec_ts *ts)
{
	struct device *dev = &ts->client->dev;
	struct input_dev *input;
	u32 dx = 1440, dy = 2960;
	int ret;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	/* 4.9 sec_ts_set_input_prop() */
	input->name = SEC_TS_INPUT_NAME;
	input->phys = SEC_TS_INPUT_NAME "/input1";
	input->id.bustype = BUS_I2C;

	/* Axis range from the IC (4.9 behaviour), DT size as fallback. */
	device_property_read_u32(dev, "touchscreen-size-x", &dx);
	device_property_read_u32(dev, "touchscreen-size-y", &dy);
	if (ts->max_x && ts->max_y) {
		if (ts->max_x != dx || ts->max_y != dy)
			dev_warn(dev, "IC reports %ux%u, DT says %ux%u: using the IC's\n",
				 ts->max_x, ts->max_y, dx, dy);
		dx = ts->max_x;
		dy = ts->max_y;
	}
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, dx - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, dy - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_capability(input, EV_KEY, BTN_TOOL_FINGER);

	ret = input_mt_init_slots(input, SEC_TS_MAX_TOUCH, INPUT_MT_DIRECT);
	if (ret)
		return ret;

	ts->input = input;
	input_set_drvdata(input, ts);
	ret = input_register_device(input);
	if (ret)
		return ret;
	dev_info(dev, "input %s, %ux%u, %d slots\n", input->name, dx, dy,
		 SEC_TS_MAX_TOUCH);
	return 0;
}

static void sec_ts_selftest_work(struct work_struct *work)
{
	struct sec_ts *ts = container_of(to_delayed_work(work), struct sec_ts,
					 selftest_work);
	u8 id[5] = { 0 }, img[4] = { 0 }, boot = 0, status[4] = { 0 };
	int ret = -ENODEV, line = -1;
	bool pass;

	mutex_lock(&ts->event_lock);
	pr_info("touch selftest: start uptime=%u ms state=%s addr=%#04x\n",
		jiffies_to_msecs(jiffies - INITIAL_JIFFIES),
		sec_ts_state_names[ts->state], ts->client->addr);

	if (ts->power_ctl && !ts->powered) {
		pr_info("touch selftest: IC rails off (power-on refused/failed or probe failure), no IC reads\n");
	} else if (ts->state != SEC_TS_ABSENT) {
		ret = sec_ts_read(ts, SEC_TS_READ_DEVICE_ID, id, 5);
		pr_info("touch selftest: chip id %5ph (probe %5ph) %s\n", id,
			ts->device_id, ret ? "READ FAILED" :
			memcmp(id, ts->device_id, 5) ? "CHANGED" : "stable");
		if (!ret)
			ret = sec_ts_read(ts, SEC_TS_READ_IMG_VERSION, img, 4);
		if (!ret)
			ret = sec_ts_read(ts, SEC_TS_READ_BOOT_STATUS, &boot, 1);
		if (!ret)
			ret = sec_ts_read(ts, SEC_TS_READ_TS_STATUS, status, 4);
		pr_info("touch selftest: chip %s fw image %4ph core %4ph config %4ph (now %4ph) boot %#04x status %4ph\n",
			sec_ts_family(ts->img_ver[0]), ts->img_ver, ts->core_ver,
			ts->para_ver, img, boot, status);
	} else {
		pr_info("touch selftest: IC did not answer at probe (unpowered?): LDO35 tsp_io 1.8 V + LDO43 tsp_avdd 3.0 V needed, see pmic selftest L35CTRL(0x6c)/L43CTRL(0x74)\n");
	}
	if (ts->power_ctl) {
		u8 l35 = 0, l43 = 0;
		int r = sec_ts_rails_now(&l35, &l43);

		pr_info("touch selftest: power control on, power-on result %d, rails %s, now 0x6c=%#04x 0x74=%#04x (read %d)\n",
			ts->power_err, ts->powered ? "on" : "off", l35, l43, r);
	} else {
		pr_info("touch selftest: power control off (no samsung,acpm-touch-power)\n");
	}
	if (ts->irq_gpio)
		line = gpiod_get_raw_value_cansleep(ts->irq_gpio);

	pr_info("touch selftest: events since boot: press=%u move=%u release=%u status=%u gesture=%u irqs=%u idle_irqs=%u overflows=%u ic_resets=%u\n",
		ts->press, ts->move, ts->release, ts->status_events,
		ts->gesture_events, ts->irqs, ts->idle_irqs, ts->overflows,
		ts->ic_resets);
	pr_info("touch selftest: irq line=%d (%s) irq %s, fingers down=%u, i2c errors=%u\n",
		line, line == 0 ? "asserted" : line == 1 ? "idle" : "n/a",
		ts->irq_disabled ? "DISABLED (storm guard)" : "enabled",
		(unsigned int)hweight_long(ts->active), ts->i2c_errors);
	pr_info("touch selftest: IC writes: SENSE_ON(0x10)=%u SET_TOUCHFUNCTION(0x30)=%u, refused=%u; no firmware/flash/NVM/calibration code built\n",
		ts->writes_sense_on, ts->writes_touchfunc, ts->refused);

	pass = ts->state == SEC_TS_RUNNING && !ret && !ts->irq_disabled &&
	       !ts->refused;
	pr_info("touch selftest: %s%s\n", pass ? "PASS" : "FAIL",
		pass && !ts->press ? " (driver up; no touch seen yet)" : "");
	mutex_unlock(&ts->event_lock);
}

static void sec_ts_cancel_selftest(void *data)
{
	struct sec_ts *ts = data;

	cancel_delayed_work_sync(&ts->selftest_work);
}

static int sec_ts_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sec_ts *ts;
	unsigned int now;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EIO;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->buf = devm_kzalloc(dev, SEC_TS_MAX_EVENTS * SEC_TS_EVENT_BUFF_SIZE,
			       GFP_KERNEL);
	if (!ts->buf)
		return -ENOMEM;
	ts->client = client;
	mutex_init(&ts->i2c_lock);
	mutex_init(&ts->event_lock);
	INIT_DELAYED_WORK(&ts->selftest_work, sec_ts_selftest_work);
	ts->selftest = device_property_read_bool(dev, "samsung,touch-selftest");
	ts->power_ctl = device_property_read_bool(dev, "samsung,acpm-touch-power");
	ts->power_err = -ENODATA;
	i2c_set_clientdata(client, ts);

	/* ASIS: requesting it as an input would undo the EINT pin function. */
	ts->irq_gpio = devm_gpiod_get(dev, "irq", GPIOD_ASIS);
	if (IS_ERR(ts->irq_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->irq_gpio), "irq gpio\n");
	if (client->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "no interrupt\n");

	/* before the rails go on: no early return may leave them on */
	if (ts->selftest) {
		ret = devm_add_action_or_reset(dev, sec_ts_cancel_selftest, ts);
		if (ret)
			return ret;
	}

	if (ts->power_ctl) {
		ret = sec_ts_power_up(ts);
		if (ret == -EPROBE_DEFER)
			return ret;
	}

	if (ts->selftest) {
		now = jiffies_to_msecs(jiffies - INITIAL_JIFFIES);
		schedule_delayed_work(&ts->selftest_work,
				      msecs_to_jiffies(now < SEC_TS_SELFTEST_MS ?
						       SEC_TS_SELFTEST_MS - now : 0));
	}

	dev_info(dev, "stage 1 (no firmware update), power control %s, irq %d, line %d\n",
		 ts->power_ctl ? (ts->powered ? "on (rails on)" : "on (rails off)") :
				 "off", client->irq,
		 gpiod_get_raw_value_cansleep(ts->irq_gpio));

	ret = sec_ts_identify(ts);
	/* No answer right after our own power-on: one power cycle (4.9 reset: off, 30 ms, on). */
	if (ret && ts->state == SEC_TS_ABSENT && ts->powered) {
		dev_warn(dev, "no answer after power-on, power cycling once\n");
		sec_ts_power_off(ts, "retry");
		msleep(30);
		if (!ts->powered && !sec_ts_power_up(ts))
			ret = sec_ts_identify(ts);
	}
	/* Detection failures keep the device bound so the self-test reports. */
	if (ret) {
		sec_ts_power_off(ts, "probe failed: identify");
		return 0;
	}

	ret = sec_ts_start(ts);
	if (ret) {
		ts->state = SEC_TS_FAILED;
		sec_ts_power_off(ts, "probe failed: start");
		return 0;
	}

	ret = sec_ts_input_init(ts);
	if (ret) {
		sec_ts_power_off(ts, "probe failed: input");
		return dev_err_probe(dev, ret, "input device\n");
	}

	/* Trigger (level low, 4.9 sec,irq_type 0x2008) comes from DT. */
	ret = devm_request_threaded_irq(dev, client->irq, NULL, sec_ts_irq_thread,
					IRQF_ONESHOT, SEC_TS_DRV_NAME, ts);
	if (ret) {
		sec_ts_power_off(ts, "probe failed: irq");
		return dev_err_probe(dev, ret, "irq %d\n", client->irq);
	}

	ts->state = SEC_TS_RUNNING;
	dev_info(dev, "running\n");
	return 0;
}

/* 4.9 sec_ts_shutdown() -> sec_ts_remove() -> power(off): LDO43 first. */
static void sec_ts_shutdown(struct i2c_client *client)
{
	struct sec_ts *ts = i2c_get_clientdata(client);

	if (!ts || !ts->power_ctl || !ts->powered)
		return;
	if (ts->state == SEC_TS_RUNNING)
		disable_irq(client->irq);
	cancel_delayed_work_sync(&ts->selftest_work);
	sec_ts_power_off(ts, "shutdown");
}

static const struct of_device_id sec_ts_of_match[] = {
	{ .compatible = "samsung,sec-ts-y771" },
	{ }
};
MODULE_DEVICE_TABLE(of, sec_ts_of_match);

static const struct i2c_device_id sec_ts_id[] = {
	{ SEC_TS_DRV_NAME },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sec_ts_id);

static struct i2c_driver sec_ts_driver = {
	.driver = {
		.name = SEC_TS_DRV_NAME,
		.of_match_table = sec_ts_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		/* no unbind/rebind power cycles from sysfs */
		.suppress_bind_attrs = true,
	},
	.probe = sec_ts_probe,
	.shutdown = sec_ts_shutdown,
	.id_table = sec_ts_id,
};
module_i2c_driver(sec_ts_driver);

MODULE_DESCRIPTION("Samsung sec_ts Y771 touchscreen, crownlte (no firmware update)");
MODULE_LICENSE("GPL");
