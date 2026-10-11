/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <soc/samsung/acpm_mfd.h>
#include <soc/samsung/acpm_ipc_ctrl.h>

static struct device_node *acpm_mfd_node;
static int __init exynos9810_acpm_mfd_channel_init(void)
{
	acpm_mfd_node = of_find_compatible_node(NULL, NULL,
		"samsung,exynos9810-acpm-pmic-client");
	if (!acpm_mfd_node)
		acpm_mfd_node = of_find_compatible_node(NULL, NULL, "samsung,s2mps18mfd");
	if (!acpm_mfd_node)
		return -ENODEV;
	if (!of_device_is_available(acpm_mfd_node)) {
		of_node_put(acpm_mfd_node);
		acpm_mfd_node = NULL;
		return -ENODEV;
	}
	return 0;
}
fs_initcall(exynos9810_acpm_mfd_channel_init);

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
	unsigned int channel_num = 0, size = 0;
	struct ipc_config config;
	unsigned int command[4] = {0,};
	int ret = 0;

	if (!acpm_mfd_request_channel(&channel_num, &size)) {
		config.cmd = command;
		config.cmd[0] = set_protocol(type, TYPE) | set_protocol(reg, REG);
		config.cmd[1] = set_protocol(FUNC_UPDATE, FUNC)
				| set_protocol(value, UPDATE_VAL)
				| set_protocol(mask, UPDATE_MASK);
		config.response = true;
		config.indirection = false;

		ACPM_MFD_PRINT("%s - addr: 0x%03x val: 0x%02x mask: 0x%02x\n",
				__func__, set_protocol(type, TYPE) | set_protocol(reg, REG), value, mask);

		ret = acpm_ipc_send_data(channel_num, &config);
		if (ret) {
			pr_err(ACPM_MFD_PREFIX "%s - acpm_ipc_send_data fail.\n", __func__);
			return ret;
		}

		ret = read_protocol(config.cmd[1], RETURN);
		if (ret) {
			if (ret == 1)
				pr_err(ACPM_MFD_PREFIX "%s - APM's speedy_rx fail.\n", __func__);
			else if (ret == 2)
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
