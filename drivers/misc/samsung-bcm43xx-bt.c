// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung BCM43xx (BCM4361) Bluetooth power and rfkill for crownlte.
 *
 * 6.12 port of the 4.9 "samsung,bcm43xx" driver (drivers/bluetooth/bcm43xx.c),
 * stage B1 of Documentation/crownlte/bluetooth-port.md:
 *  - one rfkill "bcm43xx Bluetooth", type bluetooth, registered blocked,
 *    on the platform device "bluetooth" so that it appears as
 *    /sys/devices/platform/bluetooth/rfkill/rfkill0 (vendor rc + sepolicy);
 *  - BT_REG_ON is pin 7 (port 0 bit 7) of the PCAL6524 at 0x22 on
 *    hsi2c@10930000. That expander also drives NFC VEN, the wireless charging
 *    enable and the DP switch, so no GPIO driver is bound to it: this driver
 *    writes exactly two of its registers (output port 0, configuration
 *    port 0) and only ever changes bit 7, read-modify-write, with readback.
 *    Before and after every write it re-reads every non-volatile register
 *    and compares it with the probe baseline; any failed read or difference
 *    latches the expander "unsafe" and nothing is ever written again;
 *  - BT_REG_ON power state OFF / ON / ON (bootloader) / UNKNOWN. The
 *    bootloader leaves P0_7 an output driven high; probe records that as
 *    "ON (bootloader)" and writes nothing. The first rfkill block or
 *    unblock (or the self-test) drives it low, as 4.9 did at probe;
 *  - BT_WAKE (gpg2-0) is held high whenever this driver has powered the
 *    chip or may have (no LPM yet: the vendor libbt never drives BT_WAKE);
 *  - BT_HOST_WAKE (gpa2-3) is requested as a both-edge interrupt that only
 *    counts edges while powered. It is not a wakeup source in B1.
 *
 * An optional one-shot self-test ("bt selftest:"), armed only by the DT
 * property samsung,bt-selftest, checks the expander bus, the UART and the
 * rfkill index and, unless samsung,bt-selftest-no-power is set, power-cycles
 * the chip, sends HCI Reset / Read Local Version / Read BD_ADDR over ttySAC1
 * at 115200 8N1 with RTS/CTS and powers it off again.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/rfkill.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/timekeeping.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_port.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

/* PCAL6524 registers (4.9 include/linux/i2c/pcal6524.h + NXP datasheet) */
#define PCAL_IN0		0x00
#define PCAL_OUT0		0x04
#define PCAL_POL0		0x08
#define PCAL_CFG0		0x0c	/* 1 = input */
#define PCAL_OUT_CFG		0x5c	/* bit n: port n open-drain */
#define BT_EXP_BIT		BIT(7)	/* P0_7 = BT_REG_ON, the only pin we touch */

struct pcal_dump_group {
	const char *name;
	u8 reg;
	u8 n;
	bool volatile_reg;	/* changes on its own: not compared */
};

/*
 * Every readable PCAL6524 register (interrupt clear 0x68-0x6a is write-only
 * and skipped). Reads have no side effects except that reading the input
 * ports clears a pending expander interrupt, which nothing uses on 6.12.
 * All of them answered on hardware (B1-read, 2026-11-07), including the
 * PCAL6524 extensions from 0x60 on that 4.9 never read. This board has a
 * PCAL6524, so every one of them is mandatory: a failed read is a transfer
 * error, never evidence that a register is absent.
 */
static const struct pcal_dump_group pcal_groups[] = {
	{ "in",		0x00, 3, true },
	{ "out",	0x04, 3, false },
	{ "pol",	0x08, 3, false },
	{ "cfg",	0x0c, 3, false },
	{ "drv",	0x40, 6, false },
	{ "latch",	0x48, 3, false },
	{ "pull_en",	0x4c, 3, false },
	{ "pull_sel",	0x50, 3, false },
	{ "int_mask",	0x54, 3, false },
	{ "int_stat",	0x58, 3, true },
	{ "out_cfg",	0x5c, 1, false },
	{ "int_edge",	0x60, 6, false },
	{ "in_stat",	0x6c, 3, true },
	{ "out_indcfg",	0x70, 3, false },
	{ "debounce",	0x74, 3, false },
};

#define PCAL_DUMP_BYTES		49	/* sum of pcal_groups[].n */
#define PCAL_LAST_BASE_GROUP	3	/* first dump line ends with "cfg" */

struct pcal_dump {
	bool valid;
	int err;
	int err_reg;		/* register whose read failed, -1: none */
	u8 v[PCAL_DUMP_BYTES];
};

/* read-only diagnostic addresses (Exynos9810, 4.9 DT; 4.9 reads them too) */
#define GPG3_BASE		0x108300c0	/* PERIC1 gpg3: CON, DAT, PUD */
#define EXP_RESET_PIN		6		/* gpg3-6, expander RESET#, active low */

#define USI_CON			0x04	/* relative to the USI wrapper node */
#define USI_OPTION		0x08
#define UART_ULCON		0x00
#define UART_UCON		0x04
#define UART_UFCON		0x08
#define UART_UMCON		0x0c
#define UART_UFSTAT		0x18
#define UART_UMSTAT		0x1c
#define UART_UBRDIV		0x28
#define UART_UFRACVAL		0x2c

#define BT_POWER_ON_DELAY_MS	100	/* 4.9 bcm43xx.c, after BT_REG_ON high */
/*
 * BT_REG_ON low time of a power cycle from "ON (bootloader)". 4.9 never
 * cycled quickly: it drove the pin low at probe and the HAL raised it
 * seconds later. 500 ms is five times the 4.9 power-on delay.
 */
#define BT_POWER_CYCLE_OFF_MS	500
#define HOST_WAKE_STORM		10000	/* edges per power session */
#define ST_RX_SIZE		64

enum bt_pwr {
	BT_PWR_OFF,		/* P0_7 driven low, or an input (not driven) */
	BT_PWR_ON,		/* powered by this driver */
	BT_PWR_ON_BOOT,		/* P0_7 driven high by the bootloader (probe) */
	BT_PWR_UNKNOWN,		/* a P0_7 write may have taken effect */
};

static const char * const bt_pwr_names[] = {
	[BT_PWR_OFF]		= "OFF",
	[BT_PWR_ON]		= "ON",
	[BT_PWR_ON_BOOT]	= "ON (bootloader)",
	[BT_PWR_UNKNOWN]	= "UNKNOWN",
};

struct bcm43xx_bt {
	struct device *dev;
	struct mutex lock;		/* power state, expander access, self-test */
	struct rfkill *rfkill;
	struct gpio_desc *bt_wake;
	struct gpio_desc *host_wake;
	int host_wake_irq;
	struct i2c_client *exp;		/* PCAL6524, no driver bound */
	struct device_node *uart_np;

	enum bt_pwr pwr;
	bool exp_unsafe;		/* latched: never write the expander again */
	unsigned int write_errors;	/* I2C write transfer errors this boot */
	struct pcal_dump baseline;	/* at probe */
	u8 exp_out0, exp_cfg0;		/* expected output/config port 0 */
	bool quiescing;			/* suspend/shutdown: no new transitions */

	spinlock_t irq_lock;
	bool irq_on;
	bool irq_storm;
	unsigned int host_wake_edges;	/* this power session */

	/* self-test */
	struct delayed_work st_work;
	u32 st_ms;
	bool st_no_power;
	bool st_verbose;		/* self-test running: print every dump */
	spinlock_t rx_lock;
	wait_queue_head_t rx_wait;
	bool rx_active;
	u8 rx[ST_RX_SIZE];
	unsigned int rx_len;
	unsigned int rx_dropped;
	unsigned int rx_flagged;
};

/* ---------------------------------------------------------------------- */
/* PCAL6524: dump all registers, validate, pin-7-only read-modify-write     */

static int pcal_read(struct bcm43xx_bt *bt, u8 reg, u8 *val)
{
	int ret = i2c_smbus_read_byte_data(bt->exp, reg);

	if (ret < 0)
		return ret;
	*val = ret;
	return 0;
}

static int pcal_dump_read(struct bcm43xx_bt *bt, struct pcal_dump *d)
{
	unsigned int g, i, k = 0;
	bool all_same = true;
	int ret;

	memset(d, 0, sizeof(*d));
	d->err_reg = -1;
	for (g = 0; g < ARRAY_SIZE(pcal_groups); g++) {
		for (i = 0; i < pcal_groups[g].n; i++, k++) {
			/* every register is mandatory (see pcal_groups) */
			ret = pcal_read(bt, pcal_groups[g].reg + i, &d->v[k]);
			if (ret) {
				d->err = ret;
				d->err_reg = pcal_groups[g].reg + i;
				return ret;
			}
			if (k && d->v[k] != d->v[0])
				all_same = false;
		}
	}
	/* a stuck bus reads one constant; a live PCAL6524 never does
	 * (polarity resets to 0x00, configuration to 0xff) */
	if (all_same) {
		d->err = -EIO;
		return -EIO;
	}
	d->valid = true;
	return 0;
}

static void pcal_dump_print(struct bcm43xx_bt *bt, const char *tag,
			    const struct pcal_dump *d)
{
	char line[256];
	unsigned int g, i, k = 0;
	int n = 0;

	if (!d->valid) {
		if (!d->err)	/* never read, e.g. no baseline at probe */
			dev_info(bt->dev, "%s exp dump not taken\n", tag);
		else if (d->err_reg >= 0)
			dev_info(bt->dev, "%s exp dump failed: reg 0x%02x read error %d\n",
				 tag, d->err_reg, d->err);
		else
			dev_info(bt->dev, "%s exp dump failed: %d (all registers read one value)\n",
				 tag, d->err);
		return;
	}
	for (g = 0; g < ARRAY_SIZE(pcal_groups); g++) {
		n += scnprintf(line + n, sizeof(line) - n, " %s=",
			       pcal_groups[g].name);
		for (i = 0; i < pcal_groups[g].n; i++, k++)
			n += scnprintf(line + n, sizeof(line) - n,
				       "%s%02x", i ? "," : "", d->v[k]);
		/* two lines: input..config, then the rest */
		if (g == PCAL_LAST_BASE_GROUP || g == ARRAY_SIZE(pcal_groups) - 1) {
			dev_info(bt->dev, "%s exp%s\n", tag, line);
			n = 0;
		}
	}
}

/* offset of register @reg inside a dump (all callers pass dumped registers) */
static int pcal_dump_index(u8 reg)
{
	unsigned int g, k = 0;

	for (g = 0; g < ARRAY_SIZE(pcal_groups); g++) {
		if (reg >= pcal_groups[g].reg &&
		    reg < pcal_groups[g].reg + pcal_groups[g].n)
			return k + reg - pcal_groups[g].reg;
		k += pcal_groups[g].n;
	}
	return 0;
}

static u8 pcal_dump_reg(const struct pcal_dump *d, u8 reg)
{
	return d->v[pcal_dump_index(reg)];
}

/* latch: from now on nothing writes the expander (until reboot) */
static void pcal_set_unsafe(struct bcm43xx_bt *bt, const char *why, int err)
{
	if (!bt->exp_unsafe)
		dev_err(bt->dev, "expander UNSAFE (%s, %d): no further expander writes, BT_REG_ON state %s\n",
			why, err, bt_pwr_names[bt->pwr]);
	bt->exp_unsafe = true;
}

/*
 * Compare a dump with what the expander must hold: the probe baseline for
 * every non-volatile register, the tracked expected value for output and
 * configuration port 0 (bit 7 ignored when @mask_bit7, i.e. after a write
 * that may or may not have taken effect). Prints each difference.
 */
static int pcal_dump_compare(struct bcm43xx_bt *bt, const char *tag,
			     const struct pcal_dump *d, bool mask_bit7)
{
	int out0 = pcal_dump_index(PCAL_OUT0), cfg0 = pcal_dump_index(PCAL_CFG0);
	unsigned int g, i, k = 0;
	int diffs = 0;

	for (g = 0; g < ARRAY_SIZE(pcal_groups); g++) {
		for (i = 0; i < pcal_groups[g].n; i++, k++) {
			u8 ref = bt->baseline.v[k], mask = 0xff;

			if (pcal_groups[g].volatile_reg)
				continue;
			if (k == out0)
				ref = bt->exp_out0;
			if (k == cfg0)
				ref = bt->exp_cfg0;
			if ((k == out0 || k == cfg0) && mask_bit7)
				mask = (u8)~BT_EXP_BIT;
			if ((d->v[k] ^ ref) & mask) {
				dev_err(bt->dev, "%s exp CHANGED %s[%u] expected=%02x now=%02x%s\n",
					tag, pcal_groups[g].name, i, ref, d->v[k],
					mask != 0xff ? " (bit 7 ignored)" : "");
				diffs++;
			}
		}
	}
	return diffs;
}

/*
 * Mandatory check before and after every expander write: no driver bound,
 * every register of pcal_groups readable (no exceptions), every
 * non-volatile register as expected, and P0_7, if it is an
 * output, at the level it is driven to. Any failure latches the expander
 * unsafe. @tag: print the dump (self-test); failures always print it.
 */
static int pcal_validate(struct bcm43xx_bt *bt, const char *tag,
			 struct pcal_dump *d, bool mask_bit7)
{
	const char *t = tag ?: "expander check:";
	u8 out0, cfg0, in0;
	int ret, diffs;

	memset(d, 0, sizeof(*d));
	if (bt->exp_unsafe) {
		dev_err(bt->dev, "%s expander is latched unsafe, not touching it\n", t);
		return -EIO;
	}
	if (!bt->baseline.valid) {
		pcal_set_unsafe(bt, "no valid probe baseline", -ENODATA);
		return -EIO;
	}
	if (bt->exp->dev.driver) {
		dev_err(bt->dev, "%s expander bound to driver %s\n", t,
			bt->exp->dev.driver->name);
		pcal_set_unsafe(bt, "driver bound", -EBUSY);
		return -EBUSY;
	}
	ret = pcal_dump_read(bt, d);
	if (tag || ret)
		pcal_dump_print(bt, t, d);
	if (ret) {
		pcal_set_unsafe(bt, "register read failed", ret);
		return ret;
	}
	diffs = pcal_dump_compare(bt, t, d, mask_bit7);
	out0 = pcal_dump_reg(d, PCAL_OUT0);
	cfg0 = pcal_dump_reg(d, PCAL_CFG0);
	in0 = pcal_dump_reg(d, PCAL_IN0);
	if (!(cfg0 & BT_EXP_BIT) && ((in0 ^ out0) & BT_EXP_BIT)) {
		dev_err(bt->dev, "%s P0_7 driven %d but reads %d\n", t,
			!!(out0 & BT_EXP_BIT), !!(in0 & BT_EXP_BIT));
		diffs++;
	}
	if (diffs) {
		if (!tag)
			pcal_dump_print(bt, t, d);
		pcal_set_unsafe(bt, "registers differ from the baseline", -EIO);
		return -EIO;
	}
	return 0;
}

/*
 * Write @reg = @val, which differs from @old only in bit 7, and read it
 * back. A readback that fails or differs latches unsafe. A failed write
 * transfer returns the I2C error with *@xfer set; the caller decides (it
 * re-validates read-only).
 */
static int pcal_write_bit7(struct bcm43xx_bt *bt, u8 reg, u8 old, u8 val,
			   bool *xfer)
{
	u8 rb;
	int ret;

	*xfer = false;
	if ((old ^ val) & ~BT_EXP_BIT) {
		pcal_set_unsafe(bt, "internal: write would change more than bit 7", -EINVAL);
		return -EINVAL;
	}
	if (old == val)
		return 0;
	ret = i2c_smbus_write_byte_data(bt->exp, reg, val);
	if (ret) {
		dev_err(bt->dev, "expander: write reg 0x%02x %02x->%02x failed: %d\n",
			reg, old, val, ret);
		*xfer = true;
		return ret;
	}
	ret = pcal_read(bt, reg, &rb);
	if (ret) {
		pcal_set_unsafe(bt, "readback failed", ret);
		return ret;
	}
	if (rb != val) {
		dev_err(bt->dev, "expander: reg 0x%02x wrote %02x, reads %02x\n",
			reg, val, rb);
		pcal_set_unsafe(bt, "readback differs", -EIO);
		return -EIO;
	}
	return 0;
}

/*
 * Drive BT_REG_ON. Validate first (no write at all if that fails), record
 * UNKNOWN (and raise BT_WAKE: power may be asserted) before the first
 * write, write output port 0 then (power on) configuration port 0, as the
 * 4.9 pcal6524 driver did, then validate again. Only a validated result
 * sets OFF/ON. No rollback writes: a failure leaves UNKNOWN.
 * bt->lock held.
 */
static int bt_reg_on_set(struct bcm43xx_bt *bt, bool on)
{
	const char *pre = NULL, *post = NULL;
	struct pcal_dump d;
	u8 out0, cfg0, want_out, want_cfg;
	bool xfer = false;
	int ret;

	if (bt->st_verbose) {
		pre = on ? "bt selftest: before power on" : "bt selftest: before power off";
		post = on ? "bt selftest: after power on" : "bt selftest: after power off";
	}
	ret = pcal_validate(bt, pre, &d, bt->pwr == BT_PWR_UNKNOWN);
	if (ret)
		return ret;
	out0 = pcal_dump_reg(&d, PCAL_OUT0);
	cfg0 = pcal_dump_reg(&d, PCAL_CFG0);
	want_out = on ? out0 | BT_EXP_BIT : out0 & ~BT_EXP_BIT;
	want_cfg = on ? cfg0 & ~BT_EXP_BIT : cfg0;	/* off: stays an output (4.9) */

	if (want_out != out0 || want_cfg != cfg0) {
		bt->pwr = BT_PWR_UNKNOWN;
		gpiod_set_value_cansleep(bt->bt_wake, 1);
		ret = pcal_write_bit7(bt, PCAL_OUT0, out0, want_out, &xfer);
		if (!ret)
			ret = pcal_write_bit7(bt, PCAL_CFG0, cfg0, want_cfg, &xfer);
		if (ret) {
			/*
			 * A failed transfer may or may not have reached the
			 * chip: check everything else read-only (bit 7
			 * ignored). One transfer error per boot is tolerated
			 * so that a validated power-off can recover; a second
			 * one, or a failed check, latches unsafe.
			 */
			if (xfer && ++bt->write_errors > 1)
				pcal_set_unsafe(bt, "second write transfer error", ret);
			if (xfer && !bt->exp_unsafe)
				pcal_validate(bt, "after failed write", &d, true);
			return ret;
		}
		/* both readbacks matched: now check everything, bit 7 included */
		bt->exp_out0 = want_out;
		bt->exp_cfg0 = want_cfg;
		ret = pcal_validate(bt, post, &d, false);
		if (ret)
			return ret;
	} else if (!(cfg0 & BT_EXP_BIT) && pre) {
		/* nothing to write: the check above is the readback */
		dev_info(bt->dev, "bt selftest: P0_7 already %s\n", on ? "high" : "low");
	}
	bt->exp_out0 = want_out;
	bt->exp_cfg0 = want_cfg;
	bt->pwr = on ? BT_PWR_ON : BT_PWR_OFF;
	return 0;
}

/* ---------------------------------------------------------------------- */
/* power                                                                    */

static irqreturn_t bt_host_wake_isr(int irq, void *data)
{
	struct bcm43xx_bt *bt = data;

	spin_lock(&bt->irq_lock);
	if (bt->irq_on && ++bt->host_wake_edges >= HOST_WAKE_STORM) {
		bt->irq_on = false;
		bt->irq_storm = true;
		disable_irq_nosync(irq);
	}
	spin_unlock(&bt->irq_lock);
	return IRQ_HANDLED;
}

static void bt_host_wake_irq_enable(struct bcm43xx_bt *bt)
{
	unsigned long flags;

	spin_lock_irqsave(&bt->irq_lock, flags);
	bt->host_wake_edges = 0;
	bt->irq_storm = false;
	bt->irq_on = bt->host_wake_irq > 0;
	spin_unlock_irqrestore(&bt->irq_lock, flags);
	if (bt->host_wake_irq > 0)
		enable_irq(bt->host_wake_irq);
}

static void bt_host_wake_irq_disable(struct bcm43xx_bt *bt)
{
	unsigned long flags;
	bool was_on;

	spin_lock_irqsave(&bt->irq_lock, flags);
	was_on = bt->irq_on;
	bt->irq_on = false;
	spin_unlock_irqrestore(&bt->irq_lock, flags);
	if (was_on)
		disable_irq(bt->host_wake_irq);
	if (bt->irq_storm)
		dev_err(bt->dev, "host-wake storm: irq disabled after %u edges\n",
			bt->host_wake_edges);
}

/*
 * Power off from ON, ON (bootloader) or UNKNOWN (the recovery path): a
 * validated pin-7-only write. Refused once the expander is latched unsafe;
 * then BT_WAKE stays as it is. bt->lock held.
 */
static int bt_power_off(struct bcm43xx_bt *bt)
{
	enum bt_pwr from = bt->pwr;
	int ret;

	if (from == BT_PWR_OFF)
		return 0;
	/* edge counting runs only in ON; it stays off if this fails */
	if (from == BT_PWR_ON)
		bt_host_wake_irq_disable(bt);
	ret = bt_reg_on_set(bt, false);
	if (ret) {
		dev_err(bt->dev, "power off from %s failed: %d, BT_REG_ON state %s%s, BT_WAKE=%d\n",
			bt_pwr_names[from], ret, bt_pwr_names[bt->pwr],
			bt->exp_unsafe ? ", expander unsafe" : "",
			gpiod_get_value_cansleep(bt->bt_wake));
		/* still ON if nothing was written: count edges again */
		if (bt->pwr == BT_PWR_ON)
			bt_host_wake_irq_enable(bt);
		return ret;
	}
	gpiod_set_value_cansleep(bt->bt_wake, 0);
	if (from == BT_PWR_ON)
		dev_info(bt->dev, "power off (BT_REG_ON=0, BT_WAKE=0), host_wake_edges=%u\n",
			 bt->host_wake_edges);
	else
		dev_info(bt->dev, "power off from %s (BT_REG_ON=0, BT_WAKE=0)\n",
			 bt_pwr_names[from]);
	return 0;
}

/*
 * Power on. From ON (bootloader) this is a power cycle (validated off,
 * BT_POWER_CYCLE_OFF_MS, validated on). From UNKNOWN it is refused: block
 * (power off) first. bt->lock held.
 */
static int bt_power_on(struct bcm43xx_bt *bt)
{
	int ret;

	switch (bt->pwr) {
	case BT_PWR_ON:
		return 0;
	case BT_PWR_UNKNOWN:
		dev_err(bt->dev, "power on refused: BT_REG_ON state UNKNOWN, block (power off) first\n");
		return -EIO;
	case BT_PWR_ON_BOOT:
		ret = bt_power_off(bt);
		if (ret)
			return ret;
		msleep(BT_POWER_CYCLE_OFF_MS);
		break;
	case BT_PWR_OFF:
		break;
	}
	gpiod_set_value_cansleep(bt->bt_wake, 1);	/* 4.9: BT_WAKE before BT_REG_ON */
	ret = bt_reg_on_set(bt, true);
	if (ret) {
		if (bt->pwr == BT_PWR_OFF)	/* nothing was written */
			gpiod_set_value_cansleep(bt->bt_wake, 0);
		dev_err(bt->dev, "power on failed: %d, BT_REG_ON state %s%s, BT_WAKE=%d\n",
			ret, bt_pwr_names[bt->pwr],
			bt->exp_unsafe ? ", expander unsafe" : "",
			gpiod_get_value_cansleep(bt->bt_wake));
		return ret;
	}
	msleep(BT_POWER_ON_DELAY_MS);
	bt_host_wake_irq_enable(bt);
	dev_info(bt->dev, "power on (BT_WAKE=1, BT_REG_ON=1)\n");
	return 0;
}

static int bt_rfkill_set_block(void *data, bool blocked)
{
	struct bcm43xx_bt *bt = data;
	int ret;

	mutex_lock(&bt->lock);
	if (READ_ONCE(bt->quiescing)) {
		dev_warn(bt->dev, "rfkill %s rejected: suspend/shutdown in progress\n",
			 blocked ? "block" : "unblock");
		ret = -EBUSY;
	} else {
		ret = blocked ? bt_power_off(bt) : bt_power_on(bt);
	}
	mutex_unlock(&bt->lock);
	return ret;
}

static const struct rfkill_ops bt_rfkill_ops = {
	.set_block = bt_rfkill_set_block,
};

/* ---------------------------------------------------------------------- */
/* self-test                                                                */

#define st_info(bt, fmt, ...) dev_info((bt)->dev, "bt selftest: " fmt, ##__VA_ARGS__)
#define st_err(bt, fmt, ...) dev_err((bt)->dev, "bt selftest: " fmt, ##__VA_ARGS__)

static u32 st_readl(phys_addr_t pa)
{
	void __iomem *va = ioremap(pa & ~(phys_addr_t)0xfff, 0x1000);
	u32 v;

	if (!va)
		return 0xdeadbeef;
	v = readl(va + (pa & 0xfff));
	iounmap(va);
	return v;
}

static int st_bus(struct bcm43xx_bt *bt)
{
	u32 con, dat, pud;
	int reset_fn, reset_level, ret;
	struct pcal_dump d;
	u8 o, c, in;

	/* GPIO bank reads only (4.9 reads gpg3 through its pinctrl driver) */
	con = st_readl(GPG3_BASE);
	dat = st_readl(GPG3_BASE + 4);
	pud = st_readl(GPG3_BASE + 8);
	reset_fn = (con >> (EXP_RESET_PIN * 4)) & 0xf;
	reset_level = (dat >> EXP_RESET_PIN) & 1;
	st_info(bt, "bus exp_reset gpg3-6 fn=%d level=%d pud=%d (gpg3 con=%08x dat=%02x)\n",
		reset_fn, reset_level, (pud >> (EXP_RESET_PIN * 4)) & 0xf, con,
		dat & 0xff);
	st_info(bt, "bus expander %s addr 0x%02x driver=%s probe_baseline=%s unsafe=%d\n",
		dev_name(&bt->exp->dev), bt->exp->addr,
		bt->exp->dev.driver ? bt->exp->dev.driver->name : "none",
		bt->baseline.valid ? "ok" : "MISSING", bt->exp_unsafe);
	pcal_dump_print(bt, "bt selftest: probe", &bt->baseline);
	ret = pcal_validate(bt, "bt selftest: now", &d,
			    bt->pwr == BT_PWR_UNKNOWN);
	if (!reset_level)
		st_err(bt, "bus expander reset line gpg3-6 reads LOW (expander held in reset?)\n");
	if (ret)
		return ret;
	o = pcal_dump_reg(&d, PCAL_OUT0);
	c = pcal_dump_reg(&d, PCAL_CFG0);
	in = pcal_dump_reg(&d, PCAL_IN0);
	st_info(bt, "bus P0_7 (BT_REG_ON) now: %s, out=%d, level=%d, state %s\n",
		(c & BT_EXP_BIT) ? "input" : "output", !!(o & BT_EXP_BIT),
		!!(in & BT_EXP_BIT), bt_pwr_names[bt->pwr]);
	return 0;
}

/* baud error in 1/100 %, as samsung_tty programs it (UBRDIV + UFRACVAL) */
static int st_baud_err(unsigned long rate, unsigned int baud)
{
	unsigned long div = rate / baud, actual;

	if (div < 16)
		return 100000;
	actual = rate / div;
	return (int)(((long)actual - (long)baud) * 10000 / (long)baud);
}

/*
 * The ttySAC1 class device. On 6.12 its parent is the serial_base port
 * device (10840000.serial:0.0, child of 10840000.serial:0), not the
 * platform device, so look it up by dev_t and check that the platform
 * device is one of its ancestors.
 */
static struct device *st_find_tty(struct device *uart_dev, bool *foreign)
{
	struct device *tty_dev, *p;
	dev_t devt;

	*foreign = false;
	if (tty_dev_name_to_number("ttySAC1", &devt))
		return NULL;
	tty_dev = class_find_device_by_devt(&tty_class, devt);
	if (!tty_dev)
		return NULL;
	for (p = tty_dev->parent; p; p = p->parent)
		if (p == uart_dev)
			return tty_dev;
	*foreign = true;
	return tty_dev;
}

static int st_uart(struct bcm43xx_bt *bt)
{
	struct platform_device *pdev;
	struct device *tty_dev = NULL;
	struct device_node *usi_np;
	struct clk *pclk = NULL, *bclk = NULL;
	void __iomem *usi = NULL, *uart = NULL;
	unsigned long rate = 0;
	bool foreign = false;
	int e1, e2, ret = 0;

	pdev = of_find_device_by_node(bt->uart_np);
	if (!pdev) {
		st_err(bt, "uart: no platform device for %pOF\n", bt->uart_np);
		return -ENODEV;
	}
	if (pdev->dev.driver)
		tty_dev = st_find_tty(&pdev->dev, &foreign);
	st_info(bt, "uart %s driver=%s tty=%s%s (parent %s)\n", dev_name(&pdev->dev),
		pdev->dev.driver ? pdev->dev.driver->name : "none",
		tty_dev ? dev_name(tty_dev) : "MISSING",
		foreign ? " NOT-THIS-UART" : "",
		tty_dev && tty_dev->parent ? dev_name(tty_dev->parent) : "-");
	if (!tty_dev || foreign)
		ret = -ENODEV;
	put_device(tty_dev);

	pclk = of_clk_get_by_name(bt->uart_np, "uart");
	bclk = of_clk_get_by_name(bt->uart_np, "clk_uart_baud0");
	if (IS_ERR(pclk) || IS_ERR(bclk) || clk_prepare_enable(pclk)) {
		st_err(bt, "uart: clocks unavailable\n");
		ret = -ENODEV;
		goto out;
	}
	if (clk_prepare_enable(bclk)) {
		clk_disable_unprepare(pclk);
		st_err(bt, "uart: baud clock enable failed\n");
		ret = -ENODEV;
		goto out;
	}
	rate = clk_get_rate(bclk);
	usi_np = of_get_parent(bt->uart_np);
	usi = of_iomap(usi_np, 0);
	of_node_put(usi_np);
	uart = of_iomap(bt->uart_np, 0);
	/*
	 * USI_CON/USI_OPTION of the UART_BT wrapper (0x108400c4/c8): 4.9 only
	 * writes them, but B1-read on hardware (2026-11-07, pack 218efc06)
	 * read con=00000000 option=00000003 here and in exynos-usi without an
	 * abort, with the UART clocks enabled as here.
	 */
	if (usi)
		st_info(bt, "uart usi con=%08x option=%08x\n",
			readl(usi + USI_CON), readl(usi + USI_OPTION));
	if (uart)
		st_info(bt, "uart regs ulcon=%08x ucon=%08x ufcon=%08x umcon=%08x ufstat=%08x umstat=%08x ubrdiv=%08x ufracval=%08x\n",
			readl(uart + UART_ULCON), readl(uart + UART_UCON),
			readl(uart + UART_UFCON), readl(uart + UART_UMCON),
			readl(uart + UART_UFSTAT), readl(uart + UART_UMSTAT),
			readl(uart + UART_UBRDIV), readl(uart + UART_UFRACVAL));
	if (usi)
		iounmap(usi);
	if (uart)
		iounmap(uart);
	clk_disable_unprepare(bclk);
	clk_disable_unprepare(pclk);

	e1 = st_baud_err(rate, 115200);
	e2 = st_baud_err(rate, 3000000);
	st_info(bt, "uart clk=%lu Hz err115200=%c%d.%02d%% err3000000=%c%d.%02d%%\n",
		rate, e1 < 0 ? '-' : '+', abs(e1) / 100, abs(e1) % 100,
		e2 < 0 ? '-' : '+', abs(e2) / 100, abs(e2) % 100);
	if (!usi || !uart || abs(e1) > 200) {
		ret = -ERANGE;
	} else if (rate < 48000000 || abs(e2) > 200) {
		/* 115200 (HCI step) still usable, libbt's 3 Mbaud is not */
		st_err(bt, "uart: 3000000 baud not reachable within 2%%\n");
		if (!ret)
			ret = 1;
	}
out:
	if (!IS_ERR_OR_NULL(bclk))
		clk_put(bclk);
	if (!IS_ERR_OR_NULL(pclk))
		clk_put(pclk);
	put_device(&pdev->dev);
	return ret;
}

static int st_match_rfkill(struct device *dev, void *data)
{
	return !strncmp(dev_name(dev), "rfkill", 6);
}

/* index N of our rfkillN class device, or -1 */
static int bt_rfkill_index(struct bcm43xx_bt *bt)
{
	struct device *rdev = device_find_child(bt->dev, NULL, st_match_rfkill);
	int idx = -1;

	if (rdev && kstrtoint(dev_name(rdev) + 6, 10, &idx))
		idx = -1;
	put_device(rdev);
	return idx;
}

static int st_rfkill(struct bcm43xx_bt *bt)
{
	struct device *rdev;
	char *path = NULL;
	int ret = 0;

	rdev = device_find_child(bt->dev, NULL, st_match_rfkill);
	if (rdev)
		path = kobject_get_path(&rdev->kobj, GFP_KERNEL);
	st_info(bt, "rfkill %s /sys%s type=bluetooth soft_blocked=%d\n",
		rdev ? dev_name(rdev) : "MISSING", path ? path : "?",
		rfkill_soft_blocked(bt->rfkill));
	if (!rdev || !path ||
	    strcmp(path, "/devices/platform/bluetooth/rfkill/rfkill0"))
		ret = -EINVAL;
	kfree(path);
	put_device(rdev);
	return ret;
}

/* tty client ops: the self-test owns the port while it holds it kopened */
static size_t st_receive_buf(struct tty_port *port, const u8 *cp,
			     const u8 *fp, size_t count)
{
	struct bcm43xx_bt *bt = port->client_data;
	unsigned long flags;
	size_t i;

	spin_lock_irqsave(&bt->rx_lock, flags);
	if (bt->rx_active) {
		for (i = 0; i < count; i++) {
			if (fp && fp[i] != TTY_NORMAL)
				bt->rx_flagged++;
			if (bt->rx_len < ST_RX_SIZE)
				bt->rx[bt->rx_len++] = cp[i];
			else
				bt->rx_dropped++;
		}
	}
	spin_unlock_irqrestore(&bt->rx_lock, flags);
	wake_up(&bt->rx_wait);
	return count;
}

/* does not use client_data: harmless if it races with the restore */
static void st_write_wakeup(struct tty_port *port)
{
	struct tty_struct *tty = tty_port_tty_get(port);

	if (tty) {
		tty_wakeup(tty);
		tty_kref_put(tty);
	}
}

static const struct tty_port_client_operations st_client_ops = {
	.receive_buf = st_receive_buf,
	.write_wakeup = st_write_wakeup,
};

static void st_rx_reset(struct bcm43xx_bt *bt)
{
	unsigned long flags;

	spin_lock_irqsave(&bt->rx_lock, flags);
	bt->rx_len = 0;
	bt->rx_dropped = 0;
	bt->rx_flagged = 0;
	spin_unlock_irqrestore(&bt->rx_lock, flags);
}

static unsigned int st_rx_len(struct bcm43xx_bt *bt)
{
	unsigned long flags;
	unsigned int n;

	spin_lock_irqsave(&bt->rx_lock, flags);
	n = bt->rx_len;
	spin_unlock_irqrestore(&bt->rx_lock, flags);
	return n;
}

static void st_rx_set_active(struct bcm43xx_bt *bt, bool on)
{
	spin_lock_irq(&bt->rx_lock);
	bt->rx_active = on;
	spin_unlock_irq(&bt->rx_lock);
}

/*
 * Send an HCI command and wait for its Command Complete event
 * (04 0e <len> <ncmd> <opcode lo> <opcode hi> <status> ...). On success
 * copies the return parameters after the status byte into @ret_par.
 * Returns -ESHUTDOWN as soon as suspend/shutdown starts quiescing.
 */
static int st_hci_cmd(struct bcm43xx_bt *bt, struct tty_struct *tty,
		      u16 opcode, u8 *ret_par, unsigned int npar,
		      unsigned int *elapsed_ms)
{
	u8 cmd[4] = { 0x01, opcode & 0xff, opcode >> 8, 0x00 };
	unsigned int want = 7 + npar, have = 0, off = 0;
	unsigned long t0 = jiffies, flags;
	u8 rx[ST_RX_SIZE];
	char hex[3 * 24 + 1];
	int ret, i, n = 0;

	if (READ_ONCE(bt->quiescing))
		return -ESHUTDOWN;
	st_rx_reset(bt);
	ret = tty->ops->write(tty, cmd, sizeof(cmd));
	if (ret != sizeof(cmd)) {
		st_err(bt, "hci 0x%04x: write returned %d\n", opcode, ret);
		return ret < 0 ? ret : -EIO;
	}
	wait_event_timeout(bt->rx_wait, st_rx_len(bt) >= want ||
			   READ_ONCE(bt->quiescing), HZ);
	*elapsed_ms = jiffies_to_msecs(jiffies - t0);
	if (READ_ONCE(bt->quiescing))
		return -ESHUTDOWN;

	spin_lock_irqsave(&bt->rx_lock, flags);
	have = bt->rx_len;
	memcpy(rx, bt->rx, have);
	spin_unlock_irqrestore(&bt->rx_lock, flags);

	/* tolerate leading bytes before the event, report them */
	for (off = 0; off + want <= have; off++)
		if (rx[off] == 0x04 && rx[off + 1] == 0x0e &&
		    rx[off + 2] == 4 + npar && rx[off + 4] == cmd[1] &&
		    rx[off + 5] == cmd[2])
			break;
	if (off + want > have) {
		for (i = 0; i < have && i < 24; i++)
			n += scnprintf(hex + n, sizeof(hex) - n, "%02x ", rx[i]);
		st_err(bt, "hci 0x%04x: no command complete after %u ms, got %u bytes [%s] dropped=%u flagged=%u\n",
		       opcode, *elapsed_ms, have, n ? hex : "", bt->rx_dropped,
		       bt->rx_flagged);
		return -ETIMEDOUT;
	}
	if (off)
		st_info(bt, "hci 0x%04x: %u junk bytes before the event\n", opcode, off);
	if (rx[off + 6]) {
		st_err(bt, "hci 0x%04x: status 0x%02x\n", opcode, rx[off + 6]);
		return -EIO;
	}
	if (npar)
		memcpy(ret_par, rx + off + 7, npar);
	return 0;
}

/*
 * Give the port back. Order: stop accepting data, flush TX, close the port
 * (the UART stops receiving), drop queued flip buffers, wait for a running
 * flip-buffer worker (the only caller of client_ops->receive_buf; this is
 * tty_buffer_flush_work(), private to drivers/tty), restore the default
 * client ops while the kopen still keeps userspace out, then tty_kclose()
 * last. tty unlocked on entry.
 */
static void st_tty_release(struct bcm43xx_bt *bt, struct tty_struct *tty)
{
	struct tty_port *port = tty->port;

	st_rx_set_active(bt, false);
	tty_driver_flush_buffer(tty);	/* nothing stuck behind CTS on close */
	tty_lock(tty);
	if (tty->ops->close)
		tty->ops->close(tty, NULL);
	tty_ldisc_flush(tty);
	tty_unlock(tty);
	flush_work(&port->buf.work);
	port->client_ops = &tty_port_default_client_ops;
	port->client_data = NULL;
	tty_kclose(tty);	/* frees tty */
}

static struct tty_struct *st_tty_open(struct bcm43xx_bt *bt)
{
	struct ktermios kt;
	struct tty_struct *tty;
	struct tty_port *port;
	dev_t devt;
	int ret;

	ret = tty_dev_name_to_number("ttySAC1", &devt);
	if (ret)
		return ERR_PTR(ret);
	tty = tty_kopen_exclusive(devt);	/* -EBUSY if userspace has it */
	if (IS_ERR(tty))
		return tty;
	/* tty_kopen_exclusive() returns with the tty locked */
	port = tty->port;
	st_rx_set_active(bt, false);
	port->client_data = bt;
	port->client_ops = &st_client_ops;
	ret = tty->ops->open ? tty->ops->open(tty, NULL) : -ENODEV;
	tty_unlock(tty);
	if (ret) {
		st_tty_release(bt, tty);
		return ERR_PTR(ret);
	}
	clear_bit(TTY_HUPPED, &tty->flags);

	/* serdev-ttyport: raw 8N1, RTS/CTS, ignore carrier; 115200 as libbt */
	kt = tty->termios;
	kt.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
			ICRNL | IXON);
	kt.c_oflag &= ~OPOST;
	kt.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	kt.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
	kt.c_cflag |= CS8 | CRTSCTS | CLOCAL | CREAD;
	tty_termios_encode_baud_rate(&kt, 115200, 115200);
	tty_set_termios(tty, &kt);
	st_info(bt, "tty ttySAC1 open: %u baud, crtscts=%d\n",
		tty_termios_baud_rate(&tty->termios),
		!!(tty->termios.c_cflag & CRTSCTS));
	st_rx_set_active(bt, true);
	return tty;
}

static int st_hci(struct bcm43xx_bt *bt, struct tty_struct *tty)
{
	u8 ver[8], addr[6];
	unsigned int ms = 0;
	int ret = -ETIMEDOUT, try;

	for (try = 1; try <= 3; try++) {
		ret = st_hci_cmd(bt, tty, 0x0c03, NULL, 0, &ms);
		if (!ret || ret == -ESHUTDOWN)
			break;
		msleep(100);
	}
	if (ret)
		return ret;
	st_info(bt, "hci reset ok in %u ms (try %d)\n", ms, try);

	/* Read Local Version: hci_ver, hci_rev(2), lmp_ver, manuf(2), lmp_subver(2) */
	ret = st_hci_cmd(bt, tty, 0x1001, ver, sizeof(ver), &ms);
	if (ret)
		return ret;
	ret = st_hci_cmd(bt, tty, 0x1009, addr, sizeof(addr), &ms);
	if (ret == -ESHUTDOWN)
		return ret;
	st_info(bt, "local version hci=0x%02x rev=0x%04x lmp=0x%02x lmp_subver=0x%04x manuf=0x%04x bdaddr_oui=%s%02x:%02x:%02x\n",
		ver[0], ver[1] | ver[2] << 8, ver[3], ver[6] | ver[7] << 8,
		ver[4] | ver[5] << 8, ret ? "(read failed) " : "",
		ret ? 0 : addr[5], ret ? 0 : addr[4], ret ? 0 : addr[3]);
	if (ret)
		return ret;
	if ((ver[4] | ver[5] << 8) != 0x000f)
		st_err(bt, "manufacturer is not Broadcom (0x000f)\n");
	return 0;
}

static void st_result(struct bcm43xx_bt *bt, const char *stage,
		      const char *pass)
{
	if (stage && !strcmp(stage, "quiesce"))
		st_info(bt, "RESULT ABORT suspend/shutdown, state %s%s\n",
			bt_pwr_names[bt->pwr], bt->exp_unsafe ? ", expander unsafe" : "");
	else if (stage)
		st_err(bt, "RESULT FAIL stage=%s, state %s%s\n", stage,
		       bt_pwr_names[bt->pwr], bt->exp_unsafe ? ", expander unsafe" : "");
	else
		st_info(bt, "RESULT %s, state %s\n", pass, bt_pwr_names[bt->pwr]);
}

static void bt_selftest_work(struct work_struct *work)
{
	struct bcm43xx_bt *bt = container_of(to_delayed_work(work),
					     struct bcm43xx_bt, st_work);
	struct tty_struct *tty;
	const char *stage = NULL;
	bool baud3m_bad = false;
	int ret, ret2;

	st_info(bt, "start at %lld ms, mode %s\n",
		ktime_to_ms(ktime_get_boottime()),
		bt->st_no_power ? "no-power" : "full");

	mutex_lock(&bt->lock);
	if (READ_ONCE(bt->quiescing)) {
		st_result(bt, "quiesce", NULL);
		goto out_unlock;
	}
	if (bt->pwr == BT_PWR_ON || !rfkill_soft_blocked(bt->rfkill)) {
		st_info(bt, "RESULT SKIP userspace owns BT (rfkill unblocked)\n");
		goto out_unlock;
	}
	if (st_bus(bt))
		stage = "bus";
	ret = st_uart(bt);
	if (ret < 0 && !stage)
		stage = "uart";
	baud3m_bad = ret > 0;
	if (st_rfkill(bt) && !stage)
		stage = "rfkill";
	if (!stage && bt->pwr == BT_PWR_UNKNOWN)
		stage = "power";
	if (stage || bt->st_no_power) {
		if (!stage && baud3m_bad)
			stage = "uart-3m";
		st_result(bt, stage, "PASS-NOPOWER");
		goto out_unlock;
	}

	tty = st_tty_open(bt);
	if (IS_ERR(tty)) {
		if (PTR_ERR(tty) == -EBUSY)
			st_info(bt, "RESULT SKIP userspace owns BT (ttySAC1 open)\n");
		else
			st_err(bt, "RESULT FAIL stage=uart (tty open %ld)\n",
			       PTR_ERR(tty));
		goto out_unlock;
	}

	bt->st_verbose = true;
	if (READ_ONCE(bt->quiescing)) {
		stage = "quiesce";
	} else if (bt_power_on(bt)) {
		stage = "power";
	} else {
		u8 in0 = 0;

		ret = pcal_read(bt, PCAL_IN0, &in0);
		st_info(bt, "power on (bt_wake=%d reg_on_level=%d%s)\n",
			gpiod_get_value_cansleep(bt->bt_wake),
			!!(in0 & BT_EXP_BIT), ret ? " read failed" : "");
		ret = st_hci(bt, tty);
		if (ret == -ESHUTDOWN)
			stage = "quiesce";
		else if (ret)
			stage = "hci";
	}

	/* cleanup: always back to OFF (validated; refused if unsafe) */
	ret2 = bt_power_off(bt);
	bt->st_verbose = false;
	st_tty_release(bt, tty);
	st_info(bt, "power off%s, host_wake_edges=%u, bt_wake=%d, tty closed, rfkill soft_blocked=%d\n",
		ret2 ? " FAILED" : "", bt->host_wake_edges,
		gpiod_get_value_cansleep(bt->bt_wake),
		rfkill_soft_blocked(bt->rfkill));
	if (!stage && ret2)
		stage = "power";
	if (!stage && baud3m_bad)
		stage = "uart-3m";	/* HCI fine at 115200, 3 Mbaud is not */
	st_result(bt, stage, "PASS");
out_unlock:
	mutex_unlock(&bt->lock);
}

/* ---------------------------------------------------------------------- */
/* suspend / shutdown                                                       */

/*
 * Stop the self-test (cancel it if pending, else make it abort at its next
 * check and join it) and wait out an rfkill transition in flight; later
 * transitions see quiescing and are rejected. The self-test's own cleanup
 * power-off still runs (validated) before it returns.
 */
static void bt_quiesce(struct bcm43xx_bt *bt, const char *why)
{
	WRITE_ONCE(bt->quiescing, true);
	wake_up(&bt->rx_wait);
	if (cancel_delayed_work_sync(&bt->st_work))
		dev_info(bt->dev, "bt selftest: cancelled before it ran (%s)\n", why);
	mutex_lock(&bt->lock);
	dev_dbg(bt->dev, "quiesced for %s, BT_REG_ON state %s\n", why,
		bt_pwr_names[bt->pwr]);
	mutex_unlock(&bt->lock);
}

static int bcm43xx_bt_prepare(struct device *dev)
{
	bt_quiesce(dev_get_drvdata(dev), "suspend");
	return 0;
}

static void bcm43xx_bt_complete(struct device *dev)
{
	struct bcm43xx_bt *bt = dev_get_drvdata(dev);

	mutex_lock(&bt->lock);
	WRITE_ONCE(bt->quiescing, false);
	mutex_unlock(&bt->lock);
}

static const struct dev_pm_ops bcm43xx_bt_pm_ops = {
	.prepare = bcm43xx_bt_prepare,
	.complete = bcm43xx_bt_complete,
};

static void bcm43xx_bt_shutdown(struct platform_device *pdev)
{
	struct bcm43xx_bt *bt = platform_get_drvdata(pdev);

	if (bt)
		bt_quiesce(bt, "shutdown");
}

/* ---------------------------------------------------------------------- */
/* sysfs, probe                                                             */

static ssize_t bt_reg_on_state_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct bcm43xx_bt *bt = dev_get_drvdata(dev);
	ssize_t n;

	mutex_lock(&bt->lock);
	n = sysfs_emit(buf, "%s%s\n", bt_pwr_names[bt->pwr],
		       bt->exp_unsafe ? " unsafe" : "");
	mutex_unlock(&bt->lock);
	return n;
}
static DEVICE_ATTR_RO(bt_reg_on_state);

static struct attribute *bcm43xx_bt_attrs[] = {
	&dev_attr_bt_reg_on_state.attr,
	NULL
};
ATTRIBUTE_GROUPS(bcm43xx_bt);

static void bt_put_i2c_client(void *data)
{
	put_device(&((struct i2c_client *)data)->dev);
}

static void bt_put_node(void *data)
{
	of_node_put(data);
}

/*
 * Read-only expander baseline and the BT_REG_ON state found at probe.
 * Nothing is written (unlike 4.9, which had just reset and reprogrammed
 * the whole expander and then drove BT_REG_ON low). If the expander is not
 * in a sane state the BT_REG_ON state stays UNKNOWN and, latched unsafe,
 * is never written (BT_WAKE stays low: this driver never asserted power).
 * bt->lock held.
 */
static void bt_probe_expander(struct bcm43xx_bt *bt)
{
	struct device *dev = bt->dev;
	u8 o, c, in, pol, ocfg;

	bt->pwr = BT_PWR_UNKNOWN;
	if (bt->exp->dev.driver) {
		dev_err(dev, "expander %s is bound to driver %s: BT power disabled\n",
			dev_name(&bt->exp->dev), bt->exp->dev.driver->name);
		pcal_set_unsafe(bt, "driver bound at probe", -EBUSY);
		return;
	}
	/*
	 * The baseline must be complete: every register, the PCAL6524
	 * extensions included. Any read error (also a transient one) leaves
	 * no baseline and latches unsafe, so nothing is ever written.
	 */
	if (pcal_dump_read(bt, &bt->baseline)) {
		if (bt->baseline.err_reg >= 0)
			dev_err(dev, "expander %s reg 0x%02x unreadable (%d): no complete baseline, BT power disabled\n",
				dev_name(&bt->exp->dev), bt->baseline.err_reg,
				bt->baseline.err);
		else
			dev_err(dev, "expander %s reads one constant value (%d): no baseline, BT power disabled\n",
				dev_name(&bt->exp->dev), bt->baseline.err);
		bt->baseline.valid = false;
		pcal_set_unsafe(bt, "baseline incomplete at probe", bt->baseline.err);
		return;
	}
	pcal_dump_print(bt, "probe", &bt->baseline);
	o = pcal_dump_reg(&bt->baseline, PCAL_OUT0);
	c = pcal_dump_reg(&bt->baseline, PCAL_CFG0);
	in = pcal_dump_reg(&bt->baseline, PCAL_IN0);
	pol = pcal_dump_reg(&bt->baseline, PCAL_POL0);
	ocfg = pcal_dump_reg(&bt->baseline, PCAL_OUT_CFG);
	bt->exp_out0 = o;
	bt->exp_cfg0 = c;

	if (pol & BT_EXP_BIT) {
		pcal_set_unsafe(bt, "P0_7 input polarity inverted at probe", -EIO);
	} else if (ocfg & BIT(0)) {
		pcal_set_unsafe(bt, "port 0 open-drain at probe", -EIO);
	} else if (!(c & BT_EXP_BIT) && ((o ^ in) & BT_EXP_BIT)) {
		dev_err(dev, "BT_REG_ON (P0_7) driven %d but reads %d at probe\n",
			!!(o & BT_EXP_BIT), !!(in & BT_EXP_BIT));
		pcal_set_unsafe(bt, "P0_7 level differs from its output at probe", -EIO);
	} else if (!(c & BT_EXP_BIT) && (o & BT_EXP_BIT)) {
		/* B1-read 2026-11-07: out=90,.. cfg=6f,.. in=90,..: P0_7 high */
		bt->pwr = BT_PWR_ON_BOOT;
		dev_info(dev, "BT_REG_ON (P0_7) driven high by the bootloader: state %s, not written at probe\n",
			 bt_pwr_names[bt->pwr]);
	} else {
		bt->pwr = BT_PWR_OFF;
		dev_info(dev, "BT_REG_ON (P0_7) %s at probe: state %s\n",
			 (c & BT_EXP_BIT) ? "an input" : "driven low",
			 bt_pwr_names[bt->pwr]);
	}
}

static int bcm43xx_bt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct bcm43xx_bt *bt;
	int ret;

	bt = devm_kzalloc(dev, sizeof(*bt), GFP_KERNEL);
	if (!bt)
		return -ENOMEM;
	bt->dev = dev;
	mutex_init(&bt->lock);
	spin_lock_init(&bt->irq_lock);
	spin_lock_init(&bt->rx_lock);
	init_waitqueue_head(&bt->rx_wait);
	INIT_DELAYED_WORK(&bt->st_work, bt_selftest_work);

	np = of_parse_phandle(dev->of_node, "samsung,bt-reg-on-expander", 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "no samsung,bt-reg-on-expander\n");
	bt->exp = of_find_i2c_device_by_node(np);
	of_node_put(np);
	if (!bt->exp)
		return dev_err_probe(dev, -EPROBE_DEFER, "expander i2c client not yet registered\n");
	ret = devm_add_action_or_reset(dev, bt_put_i2c_client, bt->exp);
	if (ret)
		return ret;

	bt->uart_np = of_parse_phandle(dev->of_node, "samsung,bt-uart", 0);
	if (!bt->uart_np)
		return dev_err_probe(dev, -EINVAL, "no samsung,bt-uart\n");
	ret = devm_add_action_or_reset(dev, bt_put_node, bt->uart_np);
	if (ret)
		return ret;

	/* 4.9 parity: BT_WAKE output low at probe (SoC pin) */
	bt->bt_wake = devm_gpiod_get(dev, "bt-wake", GPIOD_OUT_LOW);
	if (IS_ERR(bt->bt_wake))
		return dev_err_probe(dev, PTR_ERR(bt->bt_wake), "BT_WAKE\n");
	bt->host_wake = devm_gpiod_get(dev, "bt-host-wake", GPIOD_IN);
	if (IS_ERR(bt->host_wake))
		return dev_err_probe(dev, PTR_ERR(bt->host_wake), "BT_HOST_WAKE\n");
	/* B1 only counts host-wake edges: an IRQ problem must not cost the rfkill */
	ret = gpiod_to_irq(bt->host_wake);
	if (ret > 0)
		ret = devm_request_irq(dev, ret, bt_host_wake_isr,
				       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				       IRQF_NO_AUTOEN, "bt_host_wake", bt) ?: ret;
	if (ret > 0) {
		bt->host_wake_irq = ret;
	} else {
		dev_err(dev, "BT_HOST_WAKE irq unavailable (%d), edges not counted\n", ret);
		bt->host_wake_irq = -1;
	}

	mutex_lock(&bt->lock);
	bt_probe_expander(bt);
	mutex_unlock(&bt->lock);

	/* drvdata before rfkill_register: sysfs, PM and set_block use it */
	platform_set_drvdata(pdev, bt);
	bt->rfkill = rfkill_alloc("bcm43xx Bluetooth", dev, RFKILL_TYPE_BLUETOOTH,
				  &bt_rfkill_ops, bt);
	if (!bt->rfkill)
		return -ENOMEM;
	/* persistent + blocked: rfkill_register() calls no set_block() */
	rfkill_init_sw_state(bt->rfkill, true);
	ret = rfkill_register(bt->rfkill);
	if (ret) {
		rfkill_destroy(bt->rfkill);
		return dev_err_probe(dev, ret, "rfkill_register\n");
	}

	dev_info(dev, "rfkill \"bcm43xx Bluetooth\" = rfkill%d (blocked), host-wake irq %d, expander %s P0_7 state %s%s\n",
		 bt_rfkill_index(bt), bt->host_wake_irq, dev_name(&bt->exp->dev),
		 bt_pwr_names[bt->pwr], bt->exp_unsafe ? " UNUSABLE (unsafe)" : "");

	if (of_property_read_bool(dev->of_node, "samsung,bt-selftest")) {
		s64 now = ktime_to_ms(ktime_get_boottime());

		of_property_read_u32(dev->of_node, "samsung,bt-selftest-ms", &bt->st_ms);
		bt->st_no_power = of_property_read_bool(dev->of_node,
							"samsung,bt-selftest-no-power");
		queue_delayed_work(system_long_wq, &bt->st_work,
				   msecs_to_jiffies(bt->st_ms > now ? bt->st_ms - now : 0));
		dev_info(dev, "bt selftest: armed for %u ms after boot, %s\n",
			 bt->st_ms, bt->st_no_power ? "no power step" : "full");
	}
	return 0;
}

static const struct of_device_id bcm43xx_bt_of_match[] = {
	{ .compatible = "samsung,bcm43xx" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm43xx_bt_of_match);

static struct platform_driver bcm43xx_bt_driver = {
	.probe = bcm43xx_bt_probe,
	.shutdown = bcm43xx_bt_shutdown,
	.driver = {
		.name = "samsung-bcm43xx-bt",
		.of_match_table = bcm43xx_bt_of_match,
		.dev_groups = bcm43xx_bt_groups,
		.pm = pm_sleep_ptr(&bcm43xx_bt_pm_ops),
		/* never unbound: the self-test and rfkill outlive any unbind */
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(bcm43xx_bt_driver);

MODULE_DESCRIPTION("Samsung BCM43xx Bluetooth power/rfkill (crownlte)");
MODULE_LICENSE("GPL");
