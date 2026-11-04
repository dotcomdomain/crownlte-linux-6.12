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
		pr_info(ACPM_MFD_PREFIX "write policy sealed: rtc_reg_mask=%#010x update_reg=%#04x forbidden=%#04x (TOP/PMIC/other writes always refused)\n",
			policy->rtc_reg_mask, policy->update_reg,
			policy->update_forbidden);
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

int exynos_acpm_update_reg(u16 type, u8 reg, u8 value, u8 mask)
{
	/*
	 * Firmware-side read-modify-write (FUNC_UPDATE: cmd[1] = 2 | value << 8 |
	 * mask << 16) is never sent: the firmware would write bits the policy
	 * cannot see. Its only in-tree caller is cal_pd_control() (type 3,
	 * reg 0x2a), which has no callers in this tree.
	 */
	pr_warn_ratelimited(ACPM_MFD_PREFIX "refused update type=%#x reg=%#04x val=%#04x mask=%#04x from %pS\n",
			    type, reg, value, mask, __builtin_return_address(0));
	return -EPERM;
}
