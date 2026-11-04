// SPDX-License-Identifier: GPL-2.0
/*
 * MAX77705 USB-C/MUIC path switch for the Exynos9810 diagnostic board (Note9 wiring checked against live DT).
 *
 * The MAX77705 carries an embedded MCU ("USBC") that owns the Type-C CC
 * logic, BC1.2 charger detection and the D+/D- switches between the
 * connector and the AP. The AP talks to it over I2C at address 0x25 with an
 * opcode protocol: the opcode and its data go into AP_DATAOUT0.. (0x21..),
 * a write to AP_DATAOUT32 (0x41) terminates the command, the MCU answers in
 * AP_DATAIN0.. (0x51..).
 *
 * The switches are not set by the MCU itself. Samsung's MUIC driver sends
 * COMMAND_CONTROL1_WRITE (0x06) with COM_USB (D+ and D- to the AP) whenever
 * a USB host is attached, and its shutdown path resets the MCU
 * (max77705_reset_ic: register 0x80 = 0x0f), so a kernel entered from TWRP
 * finds the switches in their post-reset state and the host sees nothing on
 * D+/D- however well the SoC PHY is configured. This driver does the one
 * thing that is needed: it routes D+/D- to the AP, and repeats that whenever
 * VBUS reappears.
 */
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/soc/samsung/exynos9810-max77705-muic.h>
#include <linux/workqueue.h>

#define REG_UIC_HW_REV		0x00
#define REG_UIC_FW_REV		0x01
#define REG_UIC_INT		0x02
#define REG_CC_INT		0x03
#define REG_PD_INT		0x04
#define REG_VDM_INT		0x05
#define REG_USBC_STATUS1	0x06
#define REG_USBC_STATUS2	0x07
#define REG_BC_STATUS		0x08
#define  BC_STATUS_VBUSDET		BIT(7)
#define  BC_STATUS_PRCHGTYP_MASK	(0x7 << 3)
#define  BC_STATUS_DCDTMO		BIT(2)
#define  BC_STATUS_CHGTYP_MASK		(0x3 << 0)
#define REG_CC_STATUS0		0x0a
#define REG_CC_STATUS1		0x0b
#define REG_PD_STATUS0		0x0c
#define REG_PD_STATUS1		0x0d
#define REG_UIC_INT_M		0x0e
#define REG_CC_INT_M		0x0f
#define REG_PD_INT_M		0x10
#define REG_VDM_INT_M		0x11
#define REG_AP_DATAOUT0		0x21	/* OPCODE_WRITE */
#define REG_AP_DATAOUT32	0x41	/* OPCODE_WRITE_END */
#define REG_AP_DATAIN0		0x51	/* OPCODE_READ */

#define OPCODE_BC_CTRL1_READ	0x01
#define OPCODE_BC_CTRL1_WRITE	0x02
#define OPCODE_CONTROL1_READ	0x05
#define OPCODE_CONTROL1_WRITE	0x06

/*
 * CONTROL1: NoBCComp[7] RCPS[6] COMP2SW(D+)[5:3] COMN1SW(D-)[2:0]
 * 001 = USB (AP), 011 = UART, 111 = open.
 */
#define COM_USB			0x09
#define COM_OPEN		0x3f

static int s9p_muic_path = COM_USB;
module_param_named(path, s9p_muic_path, int, 0644);
MODULE_PARM_DESC(path, "CONTROL1 switch value to apply (0x09 USB, 0x3f open, -1 none)");

static int s9p_muic_poll_ms = 2000;
module_param_named(poll_ms, s9p_muic_poll_ms, int, 0644);

/*
 * Every I2C write this driver issues (the CONTROL1 opcode = one block write
 * to AP_DATAOUT0 plus one byte write to AP_DATAOUT32) is counted, so that the
 * read-only battery monitor can report all MAX77705 writes since boot.
 */
static atomic_t s9p_muic_writes = ATOMIC_INIT(0);

int exynos9810_max77705_muic_write_count(void)
{
	return atomic_read(&s9p_muic_writes);
}
EXPORT_SYMBOL_GPL(exynos9810_max77705_muic_write_count);

struct s9p_muic {
	struct i2c_client *client;
	struct delayed_work work;
	u8 last_bc;
	bool applied;
};

static int muic_read(struct s9p_muic *m, u8 reg)
{
	return i2c_smbus_read_byte_data(m->client, reg);
}

static void muic_log_status(struct s9p_muic *m, const char *when)
{
	u8 st[8];
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(st); i++) {
		ret = muic_read(m, REG_USBC_STATUS1 + i);
		st[i] = ret < 0 ? 0xee : ret;
	}
	dev_info(&m->client->dev,
		 "%s: usbc1=%02x usbc2=%02x bc=%02x(vbus=%d chgtyp=%d prchg=%d) cc0=%02x cc1=%02x pd0=%02x pd1=%02x int=%02x/%02x/%02x/%02x\n",
		 when, st[0], st[1], st[2], !!(st[2] & BC_STATUS_VBUSDET),
		 st[2] & BC_STATUS_CHGTYP_MASK,
		 (st[2] & BC_STATUS_PRCHGTYP_MASK) >> 3,
		 st[4], st[5], st[6], st[7],
		 muic_read(m, REG_UIC_INT), muic_read(m, REG_CC_INT),
		 muic_read(m, REG_PD_INT), muic_read(m, REG_VDM_INT));
}

/* max77705_i2c_opcode_write(): opcode + data, then the end marker */
static int muic_opcode_write(struct s9p_muic *m, u8 opcode, const u8 *data,
			     int len)
{
	u8 buf[33];
	int ret;

	if (len > 32)
		return -EMSGSIZE;
	buf[0] = opcode;
	if (len)
		memcpy(buf + 1, data, len);
	atomic_inc(&s9p_muic_writes);
	ret = i2c_smbus_write_i2c_block_data(m->client, REG_AP_DATAOUT0,
					     len + 1, buf);
	if (ret < 0)
		return ret;
	if (len < 32) {
		atomic_inc(&s9p_muic_writes);
		ret = i2c_smbus_write_byte_data(m->client, REG_AP_DATAOUT32, 0);
	}
	return ret < 0 ? ret : 0;
}

static int muic_opcode_result(struct s9p_muic *m, u8 *out, int len)
{
	return i2c_smbus_read_i2c_block_data(m->client, REG_AP_DATAIN0, len,
					     out);
}

static int muic_set_path(struct s9p_muic *m, u8 val)
{
	u8 res[4] = {};
	int ret;

	ret = muic_opcode_write(m, OPCODE_CONTROL1_WRITE, &val, 1);
	if (ret) {
		dev_err(&m->client->dev, "CONTROL1 write failed: %d\n", ret);
		return ret;
	}
	msleep(20);
	muic_opcode_result(m, res, sizeof(res));
	dev_info(&m->client->dev,
		 "CONTROL1 <= %02x, MCU answered %02x %02x %02x %02x\n",
		 val, res[0], res[1], res[2], res[3]);
	m->applied = true;
	return 0;
}

static void muic_poll(struct work_struct *work)
{
	struct s9p_muic *m = container_of(work, struct s9p_muic, work.work);
	int bc = muic_read(m, REG_BC_STATUS);

	if (bc >= 0 && (u8)bc != m->last_bc) {
		dev_info(&m->client->dev, "BC_STATUS %02x -> %02x\n",
			 m->last_bc, bc);
		/* VBUS came back: the MCU may have dropped the switch setting */
		if ((bc & BC_STATUS_VBUSDET) && !(m->last_bc & BC_STATUS_VBUSDET)
		    && s9p_muic_path >= 0)
			muic_set_path(m, s9p_muic_path);
		m->last_bc = bc;
	}
	if (s9p_muic_poll_ms > 0)
		schedule_delayed_work(&m->work, msecs_to_jiffies(s9p_muic_poll_ms));
}

static int s9p_muic_probe(struct i2c_client *client)
{
	struct s9p_muic *m;
	int hw, fw, bc, ret;

	m = devm_kzalloc(&client->dev, sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;
	m->client = client;
	i2c_set_clientdata(client, m);

	hw = muic_read(m, REG_UIC_HW_REV);
	fw = muic_read(m, REG_UIC_FW_REV);
	if (hw < 0 || fw < 0)
		return dev_err_probe(&client->dev, hw < 0 ? hw : fw,
				     "MAX77705 USBC does not answer\n");
	dev_info(&client->dev, "MAX77705 USBC: HW rev %02x, FW rev %02x\n",
		 hw, fw);
	muic_log_status(m, "at probe");

	bc = muic_read(m, REG_BC_STATUS);
	m->last_bc = bc < 0 ? 0 : bc;

	if (s9p_muic_path >= 0) {
		ret = muic_set_path(m, s9p_muic_path);
		if (ret)
			return ret;
		muic_log_status(m, "after switch");
	}

	INIT_DELAYED_WORK(&m->work, muic_poll);
	if (s9p_muic_poll_ms > 0)
		schedule_delayed_work(&m->work, msecs_to_jiffies(s9p_muic_poll_ms));
	return 0;
}

static void s9p_muic_remove(struct i2c_client *client)
{
	struct s9p_muic *m = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&m->work);
}

static const struct of_device_id s9p_muic_of_match[] = {
	{ .compatible = "samsung,exynos9810-max77705-muic" },
	{ }
};
MODULE_DEVICE_TABLE(of, s9p_muic_of_match);

static struct i2c_driver s9p_muic_driver = {
	.driver = {
		.name		= "exynos9810-max77705-muic",
		.of_match_table	= s9p_muic_of_match,
	},
	.probe	= s9p_muic_probe,
	.remove	= s9p_muic_remove,
};
module_i2c_driver(s9p_muic_driver);

MODULE_DESCRIPTION("MAX77705 USB-C D+/D- path switch (Exynos9810 diagnostic)");
MODULE_LICENSE("GPL");
