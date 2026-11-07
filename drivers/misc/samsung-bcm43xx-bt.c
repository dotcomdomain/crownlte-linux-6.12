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
 *    reads/writes exactly two of its registers (output port 0, configuration
 *    port 0) and only ever changes bit 7, read-modify-write, with readback;
 *  - BT_WAKE (gpg2-0) is held high whenever the chip is powered (no LPM yet:
 *    the vendor libbt never drives BT_WAKE itself);
 *  - BT_HOST_WAKE (gpa2-3) is requested as a both-edge interrupt that only
 *    counts edges while powered. It is not a wakeup source in B1.
 *
 * An optional one-shot self-test ("bt selftest:"), armed only by the DT
 * property samsung,bt-selftest, checks the expander bus, the UART and the
 * rfkill index and, unless samsung,bt-selftest-no-power is set, powers the
 * chip, sends HCI Reset / Read Local Version / Read BD_ADDR over ttySAC1 at
 * 115200 8N1 with RTS/CTS and powers it off again.
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
#include <linux/rfkill.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timekeeping.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_port.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

/* PCAL6524 registers (4.9 include/linux/i2c/pcal6524.h + NXP datasheet) */
#define PCAL_IN0		0x00
#define PCAL_OUT0		0x04
#define PCAL_CFG0		0x0c	/* 1 = input */
#define BT_EXP_BIT		BIT(7)	/* P0_7 = BT_REG_ON, the only pin we touch */

struct pcal_dump_group {
	const char *name;
	u8 reg;
	u8 n;
	bool volatile_reg;	/* changes on its own: not compared */
	bool ext;		/* PCAL6524 extension: 4.9 never read it */
};

/*
 * Every readable PCAL6524 register (interrupt clear 0x68-0x6a is write-only
 * and skipped). Reads have no side effects except that reading the input
 * ports clears a pending expander interrupt, which nothing uses on 6.12.
 */
static const struct pcal_dump_group pcal_groups[] = {
	{ "in",		0x00, 3, true,  false },
	{ "out",	0x04, 3, false, false },
	{ "pol",	0x08, 3, false, false },
	{ "cfg",	0x0c, 3, false, false },
	{ "drv",	0x40, 6, false, false },
	{ "latch",	0x48, 3, false, false },
	{ "pull_en",	0x4c, 3, false, false },
	{ "pull_sel",	0x50, 3, false, false },
	{ "int_mask",	0x54, 3, false, false },
	{ "int_stat",	0x58, 3, true,  false },
	{ "out_cfg",	0x5c, 1, false, false },
	{ "int_edge",	0x60, 6, false, true },
	{ "in_stat",	0x6c, 3, true,  true },
	{ "out_indcfg",	0x70, 3, false, true },
	{ "debounce",	0x74, 3, false, true },
};

#define PCAL_DUMP_BYTES		49	/* sum of pcal_groups[].n */
#define PCAL_LAST_BASE_GROUP	3	/* first dump line ends with "cfg" */

struct pcal_dump {
	bool valid;
	bool ext_missing;	/* extension registers did not answer */
	int err;
	u8 v[PCAL_DUMP_BYTES];
};

/* read-only diagnostic addresses (Exynos9810, 4.9 DT) */
#define SYSREG_PERIC1_BASE	0x10811000
#define SYSREG_USI10_SW_CONF	0x3c	/* USI10 slot ("i2c" in 4.9) */
#define SYSREG_USI10I2C_SW_CONF	0x40	/* USI10_I2C slot (disabled in 4.9) */
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

#define BT_POWER_ON_DELAY_MS	100	/* 4.9 bcm43xx.c */
#define HOST_WAKE_STORM		10000	/* edges per power session */
#define ST_RX_SIZE		64

struct bcm43xx_bt {
	struct device *dev;
	struct mutex lock;		/* power state, expander access, self-test */
	struct rfkill *rfkill;
	struct gpio_desc *bt_wake;
	struct gpio_desc *host_wake;
	int host_wake_irq;
	struct i2c_client *exp;		/* PCAL6524, no driver bound */
	struct device_node *uart_np;

	bool powered;
	bool exp_tainted;		/* unexpected expander state: no more power-on */
	struct pcal_dump baseline;	/* at probe */

	spinlock_t irq_lock;
	bool irq_on;
	bool irq_storm;
	unsigned int host_wake_edges;	/* this power session */

	/* self-test */
	struct delayed_work st_work;
	u32 st_ms;
	bool st_no_power;
	spinlock_t rx_lock;
	wait_queue_head_t rx_wait;
	bool rx_active;
	u8 rx[ST_RX_SIZE];
	unsigned int rx_len;
	unsigned int rx_dropped;
	unsigned int rx_flagged;
};

/* ---------------------------------------------------------------------- */
/* PCAL6524: dump all registers, pin-7-only read-modify-write               */

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
	for (g = 0; g < ARRAY_SIZE(pcal_groups); g++) {
		for (i = 0; i < pcal_groups[g].n; i++, k++) {
			if (pcal_groups[g].ext && d->ext_missing)
				continue;
			ret = pcal_read(bt, pcal_groups[g].reg + i, &d->v[k]);
			if (ret && pcal_groups[g].ext) {
				/* never needed for P0_7: report, do not fail */
				d->ext_missing = true;
				continue;
			}
			if (ret) {
				d->err = ret;
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
		dev_info(bt->dev, "%s exp dump failed: %d\n", tag, d->err);
		return;
	}
	for (g = 0; g < ARRAY_SIZE(pcal_groups); g++) {
		n += scnprintf(line + n, sizeof(line) - n, " %s=",
			       pcal_groups[g].name);
		for (i = 0; i < pcal_groups[g].n; i++, k++) {
			if (pcal_groups[g].ext && d->ext_missing)
				n += scnprintf(line + n, sizeof(line) - n,
					       "%s--", i ? "," : "");
			else
				n += scnprintf(line + n, sizeof(line) - n,
					       "%s%02x", i ? "," : "", d->v[k]);
		}
		/* two lines: input..config, then the rest */
		if (g == PCAL_LAST_BASE_GROUP || g == ARRAY_SIZE(pcal_groups) - 1) {
			dev_info(bt->dev, "%s exp%s\n", tag, line);
			n = 0;
		}
	}
}

/* offset of register @reg inside a dump */
static int pcal_dump_index(u8 reg)
{
	unsigned int g, k = 0;

	for (g = 0; g < ARRAY_SIZE(pcal_groups); g++) {
		if (reg >= pcal_groups[g].reg &&
		    reg < pcal_groups[g].reg + pcal_groups[g].n)
			return k + reg - pcal_groups[g].reg;
		k += pcal_groups[g].n;
	}
	return -1;
}

/*
 * Compare a dump with the probe baseline. Volatile registers are skipped and
 * bit 7 of output/config port 0 may differ (our pin). Returns the number of
 * differing bytes and prints each one.
 */
static int pcal_dump_compare(struct bcm43xx_bt *bt, const char *tag,
			     const struct pcal_dump *d)
{
	int out0 = pcal_dump_index(PCAL_OUT0), cfg0 = pcal_dump_index(PCAL_CFG0);
	unsigned int g, i, k = 0;
	int diffs = 0;

	if (!bt->baseline.valid || !d->valid)
		return -ENODATA;
	for (g = 0; g < ARRAY_SIZE(pcal_groups); g++) {
		for (i = 0; i < pcal_groups[g].n; i++, k++) {
			u8 mask = 0xff;

			if (pcal_groups[g].volatile_reg)
				continue;
			if (pcal_groups[g].ext &&
			    (d->ext_missing || bt->baseline.ext_missing))
				continue;
			if (k == out0 || k == cfg0)
				mask = (u8)~BT_EXP_BIT;
			if ((d->v[k] ^ bt->baseline.v[k]) & mask) {
				dev_err(bt->dev, "%s exp CHANGED %s[%u] probe=%02x now=%02x\n",
					tag, pcal_groups[g].name, i,
					bt->baseline.v[k], d->v[k]);
				diffs++;
			}
		}
	}
	return diffs;
}

/* BT_REG_ON state from output/config/input port 0 */
static void pcal_pin_state(u8 out0, u8 cfg0, u8 in0, int *driven_high,
			   int *level)
{
	*driven_high = !(cfg0 & BT_EXP_BIT) && (out0 & BT_EXP_BIT);
	*level = !!(in0 & BT_EXP_BIT);
}

/*
 * Read output and configuration port 0 and check that every bit except
 * bit 7 still equals the probe baseline. Anything else (bus error, expander
 * reset, another writer) taints the expander: nothing more is written.
 */
static int pcal_check_port0(struct bcm43xx_bt *bt, u8 *out0, u8 *cfg0)
{
	const struct pcal_dump *b = &bt->baseline;
	u8 bo = b->v[pcal_dump_index(PCAL_OUT0)], bc = b->v[pcal_dump_index(PCAL_CFG0)];
	int ret;

	if (!b->valid) {
		dev_err(bt->dev, "expander: no valid probe baseline, not touching it\n");
		return -EIO;
	}
	if (bt->exp->dev.driver) {
		dev_err(bt->dev, "expander: bound to driver %s, not touching it\n",
			bt->exp->dev.driver->name);
		return -EBUSY;
	}
	ret = pcal_read(bt, PCAL_OUT0, out0);
	if (!ret)
		ret = pcal_read(bt, PCAL_CFG0, cfg0);
	if (ret) {
		dev_err(bt->dev, "expander: port 0 read failed: %d\n", ret);
		return ret;
	}
	if ((*out0 ^ bo) & ~BT_EXP_BIT || (*cfg0 ^ bc) & ~BT_EXP_BIT) {
		dev_err(bt->dev, "expander: port 0 changed outside P0_7 (out %02x->%02x cfg %02x->%02x), not touching it\n",
			bo, *out0, bc, *cfg0);
		bt->exp_tainted = true;
		return -EIO;
	}
	return 0;
}

/* write @reg = @val, which differs from the current value only in bit 7 */
static int pcal_write_bit7(struct bcm43xx_bt *bt, u8 reg, u8 old, u8 val)
{
	u8 rb;
	int ret;

	if ((old ^ val) & ~BT_EXP_BIT)
		return -EINVAL;	/* never: callers only flip bit 7 */
	if (old == val)
		return 0;
	ret = i2c_smbus_write_byte_data(bt->exp, reg, val);
	if (!ret)
		ret = pcal_read(bt, reg, &rb);
	if (!ret && rb != val)
		ret = -EIO;
	if (ret) {
		dev_err(bt->dev, "expander: reg 0x%02x %02x->%02x failed: %d\n",
			reg, old, val, ret);
		bt->exp_tainted = true;
	}
	return ret;
}

/* BT_REG_ON high: output bit first, then direction (4.9 pcal6524 order) */
static int bt_reg_on_set(struct bcm43xx_bt *bt, bool on)
{
	u8 out0, cfg0;
	int ret;

	ret = pcal_check_port0(bt, &out0, &cfg0);
	if (ret)
		return ret;
	if (on) {
		ret = pcal_write_bit7(bt, PCAL_OUT0, out0, out0 | BT_EXP_BIT);
		if (!ret)
			ret = pcal_write_bit7(bt, PCAL_CFG0, cfg0, cfg0 & ~BT_EXP_BIT);
	} else {
		/* stays an output, driven low (4.9: gpio_set_value(bt_en, 0)) */
		ret = pcal_write_bit7(bt, PCAL_OUT0, out0, out0 & ~BT_EXP_BIT);
	}
	return ret;
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

/* bt->lock held */
static int bt_power_on(struct bcm43xx_bt *bt)
{
	int ret;

	if (bt->powered)
		return 0;
	if (bt->exp_tainted) {
		dev_err(bt->dev, "power on refused: expander state unexpected (see above)\n");
		return -EIO;
	}
	gpiod_set_value_cansleep(bt->bt_wake, 1);
	ret = bt_reg_on_set(bt, true);
	if (ret) {
		/* undo only what is ours: P0_7 low (if still allowed), BT_WAKE low */
		if (!bt->exp_tainted)
			bt_reg_on_set(bt, false);
		gpiod_set_value_cansleep(bt->bt_wake, 0);
		dev_err(bt->dev, "power on failed: %d\n", ret);
		return ret;
	}
	msleep(BT_POWER_ON_DELAY_MS);
	bt_host_wake_irq_enable(bt);
	bt->powered = true;
	dev_info(bt->dev, "power on (BT_WAKE=1, BT_REG_ON=1)\n");
	return 0;
}

/* bt->lock held */
static int bt_power_off(struct bcm43xx_bt *bt)
{
	int ret;

	if (!bt->powered)
		return 0;
	bt_host_wake_irq_disable(bt);
	/* power-off is a pin-7-only write and is allowed even when tainted
	 * by a later check, as long as port 0 still matches the baseline */
	ret = bt_reg_on_set(bt, false);
	if (ret) {
		dev_err(bt->dev, "power off failed: %d (BT_REG_ON state unknown)\n", ret);
		bt_host_wake_irq_enable(bt);
		return ret;
	}
	gpiod_set_value_cansleep(bt->bt_wake, 0);
	bt->powered = false;
	dev_info(bt->dev, "power off (BT_REG_ON=0, BT_WAKE=0), host_wake_edges=%u\n",
		 bt->host_wake_edges);
	return 0;
}

static int bt_rfkill_set_block(void *data, bool blocked)
{
	struct bcm43xx_bt *bt = data;
	int ret;

	mutex_lock(&bt->lock);
	ret = blocked ? bt_power_off(bt) : bt_power_on(bt);
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

static void st_dump(struct bcm43xx_bt *bt, const char *when, int *diffs)
{
	struct pcal_dump d;
	char tag[48];
	int n;

	snprintf(tag, sizeof(tag), "bt selftest: %s", when);
	pcal_dump_read(bt, &d);
	pcal_dump_print(bt, tag, &d);
	n = pcal_dump_compare(bt, tag, &d);
	if (n > 0)
		*diffs += n;
	else if (n < 0)
		*diffs += 1000;
}

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

static int st_bus(struct bcm43xx_bt *bt, int *diffs)
{
	u32 con, dat, pud, sw10, sw10i2c;
	int reset_fn, reset_level, drv, lvl;
	u8 o, c, in;

	sw10 = st_readl(SYSREG_PERIC1_BASE + SYSREG_USI10_SW_CONF);
	sw10i2c = st_readl(SYSREG_PERIC1_BASE + SYSREG_USI10I2C_SW_CONF);
	con = st_readl(GPG3_BASE);
	dat = st_readl(GPG3_BASE + 4);
	pud = st_readl(GPG3_BASE + 8);
	reset_fn = (con >> (EXP_RESET_PIN * 4)) & 0xf;
	reset_level = (dat >> EXP_RESET_PIN) & 1;
	st_info(bt, "bus sysreg_peric1 usi10=0x%08x usi10_i2c=0x%08x exp_reset gpg3-6 fn=%d level=%d pud=%d (gpg3 con=%08x dat=%02x)\n",
		sw10, sw10i2c, reset_fn, reset_level,
		(pud >> (EXP_RESET_PIN * 4)) & 0xf, con, dat & 0xff);
	st_info(bt, "bus expander %s addr 0x%02x driver=%s probe_baseline=%s tainted=%d\n",
		dev_name(&bt->exp->dev), bt->exp->addr,
		bt->exp->dev.driver ? bt->exp->dev.driver->name : "none",
		bt->baseline.valid ? "ok" : "MISSING", bt->exp_tainted);
	pcal_dump_print(bt, "bt selftest: probe", &bt->baseline);
	st_dump(bt, "now", diffs);
	if (!reset_level)
		st_err(bt, "bus expander reset line gpg3-6 reads LOW (expander held in reset?)\n");
	if (!bt->baseline.valid || bt->exp_tainted || bt->exp->dev.driver || *diffs)
		return -EIO;
	o = bt->baseline.v[pcal_dump_index(PCAL_OUT0)];
	c = bt->baseline.v[pcal_dump_index(PCAL_CFG0)];
	in = bt->baseline.v[pcal_dump_index(PCAL_IN0)];
	pcal_pin_state(o, c, in, &drv, &lvl);
	st_info(bt, "bus P0_7 (BT_REG_ON) at probe: %s, out=%d, level=%d\n",
		(c & BT_EXP_BIT) ? "input" : "output", !!(o & BT_EXP_BIT), lvl);
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

static int st_uart(struct bcm43xx_bt *bt)
{
	struct platform_device *pdev;
	struct device *tty_dev = NULL;
	struct device_node *usi_np;
	struct clk *pclk = NULL, *bclk = NULL;
	void __iomem *usi = NULL, *uart = NULL;
	unsigned long rate = 0;
	int e1, e2, ret = 0;

	pdev = of_find_device_by_node(bt->uart_np);
	if (!pdev) {
		st_err(bt, "uart: no platform device for %pOF\n", bt->uart_np);
		return -ENODEV;
	}
	if (pdev->dev.driver)
		tty_dev = device_find_child_by_name(&pdev->dev, "ttySAC1");
	st_info(bt, "uart %s driver=%s tty=%s\n", dev_name(&pdev->dev),
		pdev->dev.driver ? pdev->dev.driver->name : "none",
		tty_dev ? dev_name(tty_dev) : "MISSING");
	if (!tty_dev)
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
	if (!usi || !uart || rate < 48000000 || abs(e1) > 200 || abs(e2) > 200)
		ret = -ERANGE;
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

/*
 * Send an HCI command and wait for its Command Complete event
 * (04 0e <len> <ncmd> <opcode lo> <opcode hi> <status> ...). On success
 * copies the return parameters after the status byte into @ret_par.
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

	st_rx_reset(bt);
	ret = tty->ops->write(tty, cmd, sizeof(cmd));
	if (ret != sizeof(cmd)) {
		st_err(bt, "hci 0x%04x: write returned %d\n", opcode, ret);
		return ret < 0 ? ret : -EIO;
	}
	wait_event_timeout(bt->rx_wait, st_rx_len(bt) >= want, HZ);
	*elapsed_ms = jiffies_to_msecs(jiffies - t0);

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
	bt->rx_active = false;
	port->client_data = bt;
	port->client_ops = &st_client_ops;
	ret = tty->ops->open ? tty->ops->open(tty, NULL) : -ENODEV;
	if (ret) {
		if (tty->ops->close)
			tty->ops->close(tty, NULL);
		tty_unlock(tty);
		tty_kclose(tty);	/* frees tty */
		port->client_ops = &tty_port_default_client_ops;
		port->client_data = NULL;
		return ERR_PTR(ret);
	}
	clear_bit(TTY_HUPPED, &tty->flags);
	tty_unlock(tty);

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
	spin_lock_irq(&bt->rx_lock);
	bt->rx_active = true;
	spin_unlock_irq(&bt->rx_lock);
	return tty;
}

static void st_tty_close(struct bcm43xx_bt *bt, struct tty_struct *tty)
{
	struct tty_port *port = tty->port;

	spin_lock_irq(&bt->rx_lock);
	bt->rx_active = false;
	spin_unlock_irq(&bt->rx_lock);
	/* drop anything stuck behind CTS so close does not wait closing_wait */
	tty_driver_flush_buffer(tty);
	tty_lock(tty);
	if (tty->ops->close)
		tty->ops->close(tty, NULL);
	tty_ldisc_flush(tty);
	tty_unlock(tty);
	tty_kclose(tty);
	port->client_ops = &tty_port_default_client_ops;
	port->client_data = NULL;
}

static int st_hci(struct bcm43xx_bt *bt, struct tty_struct *tty)
{
	u8 ver[8], addr[6];
	unsigned int ms = 0;
	int ret = -ETIMEDOUT, try;

	for (try = 1; try <= 3; try++) {
		ret = st_hci_cmd(bt, tty, 0x0c03, NULL, 0, &ms);
		if (!ret)
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

static void bt_selftest_work(struct work_struct *work)
{
	struct bcm43xx_bt *bt = container_of(to_delayed_work(work),
					     struct bcm43xx_bt, st_work);
	struct tty_struct *tty;
	const char *stage = NULL;
	int diffs = 0, ret, ret2;

	st_info(bt, "start at %lld ms, mode %s\n",
		ktime_to_ms(ktime_get_boottime()),
		bt->st_no_power ? "no-power" : "full");

	mutex_lock(&bt->lock);
	if (bt->powered || !rfkill_soft_blocked(bt->rfkill)) {
		st_info(bt, "RESULT SKIP userspace owns BT (rfkill unblocked)\n");
		goto out_unlock;
	}
	if (st_bus(bt, &diffs))
		stage = "bus";
	if (st_uart(bt) && !stage)
		stage = "uart";
	if (st_rfkill(bt) && !stage)
		stage = "rfkill";
	if (stage || bt->st_no_power) {
		if (stage)
			st_err(bt, "RESULT FAIL stage=%s\n", stage);
		else
			st_info(bt, "RESULT PASS-NOPOWER\n");
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

	st_dump(bt, "before power on", &diffs);
	ret = bt_power_on(bt);
	st_dump(bt, "after power on", &diffs);
	if (ret) {
		stage = "power";
	} else {
		u8 in0 = 0;

		pcal_read(bt, PCAL_IN0, &in0);
		st_info(bt, "power on (bt_wake=%d reg_on_level=%d)\n",
			gpiod_get_value_cansleep(bt->bt_wake),
			!!(in0 & BT_EXP_BIT));
		if (diffs)
			stage = "power";
		else if (st_hci(bt, tty))
			stage = "hci";
	}

	ret2 = bt_power_off(bt);
	st_dump(bt, "after power off", &diffs);
	st_tty_close(bt, tty);
	st_info(bt, "power off%s, host_wake_edges=%u, bt_wake=%d, tty closed, rfkill soft_blocked=%d\n",
		ret2 ? " FAILED" : "", bt->host_wake_edges,
		gpiod_get_value_cansleep(bt->bt_wake),
		rfkill_soft_blocked(bt->rfkill));
	if (!stage && (ret2 || diffs))
		stage = "power";
	if (stage)
		st_err(bt, "RESULT FAIL stage=%s\n", stage);
	else
		st_info(bt, "RESULT PASS\n");
out_unlock:
	mutex_unlock(&bt->lock);
}

/* ---------------------------------------------------------------------- */
/* probe                                                                    */

static void bt_put_i2c_client(void *data)
{
	put_device(&((struct i2c_client *)data)->dev);
}

static void bt_put_node(void *data)
{
	of_node_put(data);
}

static int bcm43xx_bt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct bcm43xx_bt *bt;
	u8 o, c, in;
	int ret, drv, lvl;

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

	/* 4.9 parity: BT_WAKE output low at probe (SoC pin, chip is off) */
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

	/*
	 * Expander baseline: read-only. Unlike 4.9 (which had just reset and
	 * reprogrammed the whole expander) nothing is written at probe; the
	 * first write happens at the first power-on. If the expander does not
	 * answer, reads constant garbage, has a driver bound, or BT_REG_ON is
	 * already driven high, the expander is never written.
	 */
	mutex_lock(&bt->lock);
	if (bt->exp->dev.driver) {
		dev_err(dev, "expander %s is bound to driver %s: BT power disabled\n",
			dev_name(&bt->exp->dev), bt->exp->dev.driver->name);
		bt->exp_tainted = true;
	} else if (pcal_dump_read(bt, &bt->baseline)) {
		dev_err(dev, "expander %s unreadable (%d): BT power disabled\n",
			dev_name(&bt->exp->dev), bt->baseline.err);
		bt->exp_tainted = true;
	} else {
		pcal_dump_print(bt, "probe", &bt->baseline);
		o = bt->baseline.v[pcal_dump_index(PCAL_OUT0)];
		c = bt->baseline.v[pcal_dump_index(PCAL_CFG0)];
		in = bt->baseline.v[pcal_dump_index(PCAL_IN0)];
		pcal_pin_state(o, c, in, &drv, &lvl);
		if (drv) {
			dev_err(dev, "BT_REG_ON (P0_7) is already driven high at probe: BT power disabled\n");
			bt->exp_tainted = true;
		}
	}
	mutex_unlock(&bt->lock);

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
	platform_set_drvdata(pdev, bt);

	dev_info(dev, "rfkill \"bcm43xx Bluetooth\" = rfkill%d (blocked), host-wake irq %d, expander %s P0_7%s\n",
		 bt_rfkill_index(bt), bt->host_wake_irq, dev_name(&bt->exp->dev),
		 bt->exp_tainted ? " UNUSABLE" : "");

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
	.driver = {
		.name = "samsung-bcm43xx-bt",
		.of_match_table = bcm43xx_bt_of_match,
		/* never unbound: the self-test and rfkill outlive any unbind */
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(bcm43xx_bt_driver);

MODULE_DESCRIPTION("Samsung BCM43xx Bluetooth power/rfkill (crownlte)");
MODULE_LICENSE("GPL");
