/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bits.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <soc/samsung/acpm_mfd.h>
#include <soc/samsung/acpm_ipc_ctrl.h>

static struct device_node *acpm_mfd_node;
static int __init exynos9810_acpm_mfd_channel_init(void)
{
	static const char * const compats[] = {
		"samsung,exynos9810-s2mps18-acpm",
		"samsung,exynos9810-acpm-pmic-client",
		"samsung,s2mps18mfd",
	};
	struct device_node *np;
	int i;

	/* First available node wins; a disabled one must not hide the next. */
	for (i = 0; i < ARRAY_SIZE(compats); i++) {
		for_each_compatible_node(np, NULL, compats[i]) {
			if (of_device_is_available(np)) {
				acpm_mfd_node = np;
				return 0;
			}
		}
	}
	return -ENODEV;
}
fs_initcall(exynos9810_acpm_mfd_channel_init);

/*
 * Write gate. Every PMIC write on this phone goes through the three write
 * entry points below (4.9 had no other AP path to the S2MPS18: SPEEDY is
 * driven by the APM firmware). The default policy is all zero = refuse.
 */
static DEFINE_SPINLOCK(acpm_mfd_policy_lock);
static struct acpm_mfd_write_policy acpm_mfd_policy;
static bool acpm_mfd_policy_sealed;

int exynos_acpm_mfd_seal_write_policy(const struct acpm_mfd_write_policy *policy)
{
	unsigned long flags;
	int ret = 0;

	if (!policy)
		return -EINVAL;

	spin_lock_irqsave(&acpm_mfd_policy_lock, flags);
	if (acpm_mfd_policy_sealed)
		ret = -EBUSY;
	else {
		acpm_mfd_policy = *policy;
		acpm_mfd_policy_sealed = true;
	}
	spin_unlock_irqrestore(&acpm_mfd_policy_lock, flags);

	if (ret)
		pr_err(ACPM_MFD_PREFIX "write policy already sealed\n");
	else
		pr_info(ACPM_MFD_PREFIX "write policy sealed: rtc_reg_mask=%#010x update_reg=%#04x forbidden=%#04x g3d_rail=%d touch_rails=%d (TOP/PMIC/other writes always refused)\n",
			policy->rtc_reg_mask, policy->update_reg,
			policy->update_forbidden, policy->g3d_rail,
			policy->touch_rails);
	return ret;
}

/* -EAGAIN until a policy is sealed, then 1 if touch rail updates are allowed. */
int exynos_acpm_mfd_touch_rails_policy(void)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&acpm_mfd_policy_lock, flags);
	ret = !acpm_mfd_policy_sealed ? -EAGAIN : acpm_mfd_policy.touch_rails;
	spin_unlock_irqrestore(&acpm_mfd_policy_lock, flags);
	return ret;
}

static bool acpm_mfd_write_allowed(const char *op, u16 type, u8 reg, int count,
				   const u8 *vals, void *caller)
{
	struct acpm_mfd_write_policy p;
	unsigned long flags;
	unsigned int r;
	int i;

	spin_lock_irqsave(&acpm_mfd_policy_lock, flags);
	p = acpm_mfd_policy;
	spin_unlock_irqrestore(&acpm_mfd_policy_lock, flags);

	if (type != ACPM_MFD_TYPE_RTC || count < 1 || count > 8 || !vals)
		goto deny;
	for (i = 0; i < count; i++) {
		r = reg + i;
		if (r >= 32 || !(p.rtc_reg_mask & BIT(r)))
			goto deny;
		if (r == p.update_reg && (vals[i] & p.update_forbidden))
			goto deny;
	}
	return true;

deny:
	pr_warn_ratelimited(ACPM_MFD_PREFIX "refused %s type=%#x reg=%#04x count=%d from %pS\n",
			    op, type, reg, count, caller);
	return false;
}

static int acpm_mfd_request_channel(unsigned int *id, unsigned int *size)
{
	int ret = acpm_ipc_request_channel(acpm_mfd_node, NULL, id, size);
	if (ret)
		return ret;
	if (*id != 2 || *size != 4 * sizeof(u32)) {
		acpm_ipc_release_channel(acpm_mfd_node, *id);
		return -EPROTO;
	}
	return 0;
}


/* A read-only supplier snapshot. NULL handler does not register a callback. */
bool exynos_acpm_mfd_is_ready(void)
{
	unsigned int channel = 0, size = 0;
	return !acpm_ipc_request_channel(acpm_mfd_node, NULL, &channel, &size) &&
		channel == 2 && size == 4 * sizeof(u32);
}

int exynos_acpm_read_reg(u16 type, u8 reg, u8 *dest)
{
	unsigned int channel_num = 0, size = 0;
	struct ipc_config config;
	unsigned int command[4] = {0,};
	int ret = 0;

	if (!dest)
		return -EINVAL;

	if (!acpm_mfd_request_channel(&channel_num, &size)) {
		config.cmd = command;
		config.cmd[0] = set_protocol(type, TYPE) | set_protocol(reg, REG);
		config.cmd[1] = set_protocol(FUNC_READ, FUNC);
		config.response = true;
		config.indirection = false;

		ACPM_MFD_PRINT("%s - addr: 0x%03x\n",
				__func__, set_protocol(type, TYPE) | set_protocol(reg, REG));

		ret = acpm_ipc_send_data(channel_num, &config);
		if (ret) {
			pr_err(ACPM_MFD_PREFIX "%s - acpm_ipc_send_data fail.\n", __func__);
			return ret;
		}

		*dest = read_protocol(config.cmd[1], DEST);
		ret = read_protocol(config.cmd[1], RETURN);
		if (ret) {
			pr_err(ACPM_MFD_PREFIX "%s - APM's speedy_rx fail.\n", __func__);
			return -EREMOTEIO;
		}

		ACPM_MFD_PRINT("%s - data = 0x%02x ret = 0x%02x\n",
						__func__, *dest, ret);

		acpm_ipc_release_channel(acpm_mfd_node, channel_num);
	} else {
		pr_err(ACPM_MFD_PREFIX "%s ipc request_channel fail, id:%u, size:%u\n",
				__func__, channel_num, size);
		ret = -EBUSY;
	}

	return ret;
}

int exynos_acpm_bulk_read(u16 type, u8 reg, int count, u8 *buf)
{
	unsigned int channel_num = 0, size = 0;
	struct ipc_config config;
	unsigned int command[4] = {0,};
	int i, ret = 0;

	if (!buf || count < 1 || count > 8)
		return -EINVAL;

	if (!acpm_mfd_request_channel(&channel_num, &size)) {
		config.cmd = command;
		config.cmd[0] = set_protocol(type, TYPE) | set_protocol(reg, REG);
		config.cmd[1] = set_protocol(FUNC_BULK_READ, FUNC) | set_protocol(count, CNT);
		config.response = true;
		config.indirection = false;

		ACPM_MFD_PRINT("%s - addr: 0x%03x\n",
				__func__, set_protocol(type, TYPE) | set_protocol(reg, REG));

		ret = acpm_ipc_send_data(channel_num, &config);
		if (ret) {
			pr_err(ACPM_MFD_PREFIX "%s - acpm_ipc_send_data fail.\n", __func__);
			return ret;
		}

		ret = read_protocol(config.cmd[1], RETURN);
		if (ret) {
			pr_err(ACPM_MFD_PREFIX "%s - APM's speedy_rx fail.\n", __func__);
			return -EREMOTEIO;
		}

		for (i = 0; i < count; i++) {
			if (i < 4)
				buf[i] = read_bulk_protocol(config.cmd[2], BULK_VAL, i);
			else
				buf[i] = read_bulk_protocol(config.cmd[3], BULK_VAL , i - 4);
		}

		acpm_ipc_release_channel(acpm_mfd_node, channel_num);
	} else {
		pr_err(ACPM_MFD_PREFIX "%s ipc request_channel fail, id:%u, size:%u\n",
				__func__, channel_num, size);
		ret = -EBUSY;
	}

	return ret;
}

int exynos_acpm_write_reg(u16 type, u8 reg, u8 value)
{
	unsigned int channel_num = 0, size = 0;
	struct ipc_config config;
	unsigned int command[4] = {0,};
	int ret = 0;

	if (!acpm_mfd_write_allowed("write", type, reg, 1, &value,
				    __builtin_return_address(0)))
		return -EPERM;

	if (!acpm_mfd_request_channel(&channel_num, &size)) {
		config.cmd = command;
		config.cmd[0] = set_protocol(type, TYPE) | set_protocol(reg, REG);
		config.cmd[1] = set_protocol(FUNC_WRITE, FUNC) | set_protocol(value, WRITE_VAL);
		config.response = true;
		config.indirection = false;

		ACPM_MFD_PRINT("%s - addr: 0x%03x val: 0x%02x\n",
				__func__, set_protocol(type, TYPE) | set_protocol(reg, REG), value);

		ret = acpm_ipc_send_data(channel_num, &config);
		if (ret) {
			pr_err(ACPM_MFD_PREFIX "%s - acpm_ipc_send_data fail.\n", __func__);
			return ret;
		}

		ret = read_protocol(config.cmd[1], RETURN);
		if (ret) {
			pr_err(ACPM_MFD_PREFIX "%s - APM's speedy_tx fail.\n", __func__);
			return -EREMOTEIO;
		}

		ACPM_MFD_PRINT("%s - read(ret) val: 0x%02x\n", __func__, ret);

		acpm_ipc_release_channel(acpm_mfd_node, channel_num);
	} else {
		pr_err(ACPM_MFD_PREFIX "%s ipc request_channel fail, id:%u, size:%u\n",
				__func__, channel_num, size);
		ret = -EBUSY;
	}

	return ret;
}

int exynos_acpm_bulk_write(u16 type, u8 reg, int count, u8 *buf)
{
	unsigned int channel_num = 0, size = 0;
	struct ipc_config config;
	unsigned int command[4] = {0,};
	int ret = 0;
	int i;

	if (!buf || count < 1 || count > 8)
		return -EINVAL;

	if (!acpm_mfd_write_allowed("bulk_write", type, reg, count, buf,
				    __builtin_return_address(0)))
		return -EPERM;

	if (!acpm_mfd_request_channel(&channel_num, &size)) {
		config.cmd = command;
		config.cmd[0] = set_protocol(type, TYPE) | set_protocol(reg, REG);
		config.cmd[1] = set_protocol(FUNC_BULK_WRITE, FUNC) | set_protocol(count, CNT);
		config.response = true;
		config.indirection = false;

		ACPM_MFD_PRINT("%s - addr: 0x%03x\n cnt: 0x%02x\n",
				__func__, set_protocol(type, TYPE) | set_protocol(reg, REG), count);

		for (i = 0; i < count; i++) {
			if (i < 4)
				config.cmd[2] |= set_bulk_protocol(buf[i], BULK_VAL, i);
			else
				config.cmd[3] |= set_bulk_protocol(buf[i], BULK_VAL, i - 4);
		}

		ret = acpm_ipc_send_data(channel_num, &config);
		if (ret) {
			pr_err(ACPM_MFD_PREFIX "%s - acpm_ipc_send_data fail.\n", __func__);
			return ret;
		}

		ret = read_protocol(config.cmd[1], RETURN);
		if (ret) {
			pr_err(ACPM_MFD_PREFIX "%s - APM's speedy_tx fail.\n", __func__);
			return -EREMOTEIO;
		}

		acpm_ipc_release_channel(acpm_mfd_node, channel_num);
	} else {
		pr_err(ACPM_MFD_PREFIX "%s ipc request_channel fail, id:%u, size:%u\n",
				__func__, channel_num, size);
		ret = -EBUSY;
	}

	return ret;
}

/* Exactly the two 4.9 cal_pd_control() G3D rail requests, see acpm_mfd.h. */
static bool acpm_mfd_g3d_rail_allowed(u16 type, u8 reg, u8 value, u8 mask)
{
	unsigned long flags;
	bool allowed;

	if (type != ACPM_MFD_TYPE_G3D_RAIL || reg != ACPM_MFD_G3D_RAIL_REG ||
	    mask != ACPM_MFD_G3D_RAIL_MASK ||
	    (value != ACPM_MFD_G3D_RAIL_ON && value != ACPM_MFD_G3D_RAIL_OFF))
		return false;
	spin_lock_irqsave(&acpm_mfd_policy_lock, flags);
	allowed = acpm_mfd_policy_sealed && acpm_mfd_policy.g3d_rail;
	spin_unlock_irqrestore(&acpm_mfd_policy_lock, flags);
	return allowed;
}

/*
 * The four touch rail requests, see acpm_mfd.h. Exact match on all of
 * type, reg, value and mask.
 */
static bool acpm_mfd_touch_rail_request(u16 type, u8 reg, u8 value, u8 mask)
{
	if (type != ACPM_MFD_TYPE_PMIC || mask != ACPM_MFD_TOUCH_RAIL_MASK)
		return false;
	if (reg == ACPM_MFD_TOUCH_L35_REG)
		return value == ACPM_MFD_TOUCH_L35_ON ||
		       value == ACPM_MFD_TOUCH_RAIL_OFF;
	if (reg == ACPM_MFD_TOUCH_L43_REG)
		return value == ACPM_MFD_TOUCH_L43_ON ||
		       value == ACPM_MFD_TOUCH_RAIL_OFF;
	return false;
}

static DEFINE_MUTEX(acpm_mfd_touch_lock);	/* read-check-update, on counts */
static unsigned int acpm_mfd_touch_l35_on, acpm_mfd_touch_l43_on;

static bool acpm_mfd_touch_mode_ok(u8 v, u8 vsel, u8 on)
{
	u8 mode = v & ACPM_MFD_TOUCH_RAIL_MASK;

	return (v & ACPM_MFD_TOUCH_VSEL_MASK) == vsel &&
	       (mode == ACPM_MFD_TOUCH_RAIL_OFF || mode == on);
}

/* Caller holds acpm_mfd_touch_lock. Reads 0x6c and 0x74 before every update. */
static int acpm_mfd_touch_rail_check(u8 reg, u8 value)
{
	const char *why;
	u8 l35 = 0, l43 = 0;
	int ret;

	ret = exynos_acpm_read_reg(ACPM_MFD_TYPE_PMIC, ACPM_MFD_TOUCH_L35_REG, &l35);
	if (!ret)
		ret = exynos_acpm_read_reg(ACPM_MFD_TYPE_PMIC,
					   ACPM_MFD_TOUCH_L43_REG, &l43);
	if (ret) {
		pr_err(ACPM_MFD_PREFIX "touch rail reg %#04x val %#04x refused: precheck read failed %d\n",
		       reg, value, ret);
		return ret;
	}

	if (reg == ACPM_MFD_TOUCH_L35_REG && value == ACPM_MFD_TOUCH_L35_ON) {
		why = "LDO35 not vsel 0x2c with mode 00/10";
		if (!acpm_mfd_touch_mode_ok(l35, ACPM_MFD_TOUCH_L35_VSEL,
					    ACPM_MFD_TOUCH_L35_ON))
			goto deny;
		why = "LDO35 on-request limit reached";
		if (acpm_mfd_touch_l35_on >= ACPM_MFD_TOUCH_MAX_ON)
			goto deny;
		acpm_mfd_touch_l35_on++;
	} else if (reg == ACPM_MFD_TOUCH_L43_REG &&
		   value == ACPM_MFD_TOUCH_L43_ON) {
		why = "LDO35 not on (0x6c != 0xac)";
		if (l35 != (ACPM_MFD_TOUCH_L35_VSEL | ACPM_MFD_TOUCH_L35_ON))
			goto deny;
		why = "LDO43 not vsel 0x30 with mode 00/11";
		if (!acpm_mfd_touch_mode_ok(l43, ACPM_MFD_TOUCH_L43_VSEL,
					    ACPM_MFD_TOUCH_L43_ON))
			goto deny;
		why = "LDO43 on-request limit reached";
		if (acpm_mfd_touch_l43_on >= ACPM_MFD_TOUCH_MAX_ON)
			goto deny;
		acpm_mfd_touch_l43_on++;
	} else if (reg == ACPM_MFD_TOUCH_L35_REG) {
		why = "LDO43 still on (off order is LDO43, then LDO35)";
		if (l43 & ACPM_MFD_TOUCH_RAIL_MASK)
			goto deny;
	}
	/* LDO43 off needs no precondition: AVDD may always go first. */
	return 0;

deny:
	pr_err(ACPM_MFD_PREFIX "touch rail reg %#04x val %#04x refused: %s (0x6c=%#04x 0x74=%#04x)\n",
	       reg, value, why, l35, l43);
	return -EPERM;
}

static int acpm_mfd_send_update(u16 type, u8 reg, u8 value, u8 mask)
{
	unsigned int channel_num = 0, size = 0;
	struct ipc_config config;
	unsigned int command[4] = {0,};
	int ret;

	ret = acpm_mfd_request_channel(&channel_num, &size);
	if (ret) {
		pr_err(ACPM_MFD_PREFIX "%s ipc request_channel fail: %d\n", __func__, ret);
		return -EBUSY;
	}
	config.cmd = command;
	config.cmd[0] = set_protocol(type, TYPE) | set_protocol(reg, REG);
	config.cmd[1] = set_protocol(FUNC_UPDATE, FUNC)
			| set_protocol(value, UPDATE_VAL)
			| set_protocol(mask, UPDATE_MASK);
	config.response = true;
	config.indirection = false;

	ret = acpm_ipc_send_data(channel_num, &config);
	acpm_ipc_release_channel(acpm_mfd_node, channel_num);
	if (ret) {
		pr_err(ACPM_MFD_PREFIX "%s - acpm_ipc_send_data fail: %d\n", __func__, ret);
		return ret;
	}
	ret = read_protocol(config.cmd[1], RETURN);
	if (ret) {
		pr_err(ACPM_MFD_PREFIX "update type=%#x reg=%#04x val=%#04x: firmware status %#x\n",
		       type, reg, value, ret);
		return -EREMOTEIO;
	}
	/* Success only means "request handled": callers verify the result. */
	return 0;
}

int exynos_acpm_update_reg(u16 type, u8 reg, u8 value, u8 mask)
{
	int ret;

	/*
	 * A generic firmware-side read-modify-write (FUNC_UPDATE: cmd[1] = 2 |
	 * value << 8 | mask << 16) is never sent: the firmware would write bits
	 * the policy cannot see. The exceptions, each only if the sealed policy
	 * enables it: the G3D rail request of cal_pd_control(), which the APM
	 * firmware never forwards as a SPEEDY RMW, and the four touch rail
	 * requests, whose mask 0xc0 limits the firmware RMW to the LDO35/LDO43
	 * enable field and which are checked against the PMIC state first.
	 */
	if (acpm_mfd_touch_rail_request(type, reg, value, mask) &&
	    exynos_acpm_mfd_touch_rails_policy() == 1) {
		might_sleep();
		mutex_lock(&acpm_mfd_touch_lock);
		ret = acpm_mfd_touch_rail_check(reg, value);
		if (!ret)
			ret = acpm_mfd_send_update(type, reg, value, mask);
		mutex_unlock(&acpm_mfd_touch_lock);
		pr_info(ACPM_MFD_PREFIX "touch rail update reg=%#04x val=%#04x mask=%#04x from %pS: %d\n",
			reg, value, mask, __builtin_return_address(0), ret);
		return ret;
	}

	if (!acpm_mfd_g3d_rail_allowed(type, reg, value, mask)) {
		pr_warn_ratelimited(ACPM_MFD_PREFIX "refused update type=%#x reg=%#04x val=%#04x mask=%#04x from %pS\n",
				    type, reg, value, mask, __builtin_return_address(0));
		return -EPERM;
	}

	ret = acpm_mfd_send_update(type, reg, value, mask);
	if (ret == -EREMOTEIO)
		pr_err(ACPM_MFD_PREFIX "G3D rail %s: firmware refused the request\n",
		       value ? "on" : "off");
	return ret;
}
