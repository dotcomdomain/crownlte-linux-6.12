// SPDX-License-Identifier: GPL-2.0
/*
 * Opt-in crownlte display PMIC snapshot. No register writes, regulator
 * operations, interrupt handler or IRQ-mask changes. IRQ is clear-on-read
 * and is attempted only in the last sample, even if that read fails.
 */
#define pr_fmt(fmt) "s2dos05-diag: " fmt

#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/workqueue.h>

static const unsigned int sample_ms[] = { 1000, 5500, 9000, 25000, 60000, 120000 };

struct s2dos05_diag {
	struct i2c_client *client;
	struct delayed_work work;
	unsigned int sample;
};

/* One register-address byte, repeated START, one data byte. The address
 * phase selects a register; it contains no register value to write.
 *
 * Call the adapter once under its segment lock, deliberately bypassing
 * i2c_transfer()'s arbitration retry loop (especially for clear-on-read IRQ).
 * This dedicated HSI2C adapter supports these two ordinary I2C messages.
 */
static int s2dos05_diag_read(struct i2c_client *client, u8 reg, u8 *value)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msgs[] = {
		{ .addr = client->addr, .len = 1, .buf = &reg },
		{ .addr = client->addr, .flags = I2C_M_RD,
		  .len = 1, .buf = value },
	};
	int ret;

	i2c_lock_bus(adap, I2C_LOCK_SEGMENT);
	ret = adap->algo->master_xfer(adap, msgs, ARRAY_SIZE(msgs));
	i2c_unlock_bus(adap, I2C_LOCK_SEGMENT);

	return ret == ARRAY_SIZE(msgs) ? 0 : ret < 0 ? ret : -EIO;
}

static void s2dos05_diag_schedule(struct s2dos05_diag *diag)
{
	s64 remaining = (s64)sample_ms[diag->sample] * NSEC_PER_MSEC -
			ktime_get_boottime_ns();

	schedule_delayed_work(&diag->work,
			      remaining > 0 ? nsecs_to_jiffies(remaining) + 1 : 0);
}

static void s2dos05_diag_work(struct work_struct *work)
{
	struct s2dos05_diag *diag =
		container_of(to_delayed_work(work), struct s2dos05_diag, work);
	unsigned int target = sample_ms[diag->sample];
	bool last = diag->sample == ARRAY_SIZE(sample_ms) - 1;
	u8 regs[0x11], irq;
	int reg, ret;

	pr_info("sample target_ms=%u uptime_ms=%llu IRQ=%s\n",
		target, div_u64(ktime_get_boottime_ns(), NSEC_PER_MSEC),
		last ? "read-once-clear-on-read" : "not-read");
	for (reg = 0; reg < ARRAY_SIZE(regs); reg++) {
		ret = s2dos05_diag_read(diag->client, reg, &regs[reg]);
		if (ret) {
			pr_info("target_ms=%u reg=0x%02x read failed err=%d; sample aborted, no retry\n",
				target, reg, ret);
			break;
		}
		pr_info("target_ms=%u reg=0x%02x value=0x%02x\n",
			target, reg, regs[reg]);
	}
	if (reg == ARRAY_SIZE(regs)) {
		/* The reference 4.9 header defines no TOPSYS_STAT/STAT bits. */
		pr_info("target_ms=%u TOPSYS_STAT=0x%02x STAT=0x%02x (raw; bit meanings unknown)\n",
			target, regs[1], regs[2]);
		pr_info("target_ms=%u EN=0x%02x LDO1=%u LDO2=%u LDO3=%u LDO4=%u BUCK1=%u\n",
			target, regs[3], !!(regs[3] & BIT(0)),
			!!(regs[3] & BIT(1)), !!(regs[3] & BIT(2)),
			!!(regs[3] & BIT(3)), !!(regs[3] & BIT(4)));
	}
	if (last) {
		ret = s2dos05_diag_read(diag->client, 0x11, &irq);
		if (ret)
			pr_info("target_ms=%u IRQ reg=0x11 clear-on-read single attempt failed err=%d; no retry\n",
				target, ret);
		else
			pr_info("target_ms=%u IRQ reg=0x11 value=0x%02x clear-on-read read-once PWRMT=%u TSD=%u SSD=%u SCP=%u UVLO=%u OCD=%u\n",
				target, irq, !!(irq & BIT(5)), !!(irq & BIT(4)),
				!!(irq & BIT(3)), !!(irq & BIT(2)),
				!!(irq & BIT(1)), !!(irq & BIT(0)));
		pr_info("complete; no further reads\n");
		return;
	}
	diag->sample++;
	s2dos05_diag_schedule(diag);
}

static int s2dos05_diag_probe(struct i2c_client *client)
{
	struct s2dos05_diag *diag;

	if (client->addr != 0x60 ||
	    !i2c_check_functionality(client->adapter, I2C_FUNC_I2C) ||
	    !client->adapter->algo->master_xfer)
		return -EOPNOTSUPP;
	diag = devm_kzalloc(&client->dev, sizeof(*diag), GFP_KERNEL);
	if (!diag)
		return -ENOMEM;
	diag->client = client;
	i2c_set_clientdata(client, diag);
	INIT_DELAYED_WORK(&diag->work, s2dos05_diag_work);
	pr_info("armed bus=%d addr=0x60 read-only targets_ms=1000,5500,9000,25000,60000,120000; IRQ 0x11 only at 120000 (clear-on-read)\n",
		client->adapter->nr);
	s2dos05_diag_schedule(diag);
	return 0;
}

static void s2dos05_diag_remove(struct i2c_client *client)
{
	struct s2dos05_diag *diag = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&diag->work);
}

static const struct of_device_id s2dos05_diag_match[] = {
	{ .compatible = "samsung,crownlte-s2dos05-diag" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2dos05_diag_match);

static struct i2c_driver s2dos05_diag_driver = {
	.driver = {
		.name = "crownlte-s2dos05-diag",
		.of_match_table = s2dos05_diag_match,
		.suppress_bind_attrs = true,
	},
	.probe = s2dos05_diag_probe,
	.remove = s2dos05_diag_remove,
};
module_i2c_driver(s2dos05_diag_driver);

MODULE_DESCRIPTION("Read-only crownlte S2DOS05 display PMIC snapshots");
MODULE_LICENSE("GPL");
