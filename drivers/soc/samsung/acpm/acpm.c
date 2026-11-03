/*
 * Copyright (c) 2015 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/exynos-ss.h>
#include "../cal-if/asv.h"
#include <linux/sched/clock.h>

#include "acpm.h"
#include "acpm_ipc.h"
#include "../cal-if/fvmap.h"
#include "fw_header/framework.h"

static int ipc_done;
static unsigned long long ipc_time_start;
static unsigned long long ipc_time_end;

static struct acpm_info *exynos_acpm;
/* Always outside the IPC metadata mutex. No IPC callback takes this lock. */
static DEFINE_MUTEX(acpm_framework_lock);

static int acpm_send_data(struct device_node *node, unsigned int check_id,
		struct ipc_config *config);







static int plugins_init(struct device *framework)
{
	struct acpm_firmware_snapshot snapshot;
	int ret;

	ret = acpm_ipc_copy_firmware_snapshot(framework, &snapshot);
	if (ret) {
		dev_err(framework, "ACPM diagnostic snapshot unavailable: %d\n", ret);
		return ret;
	}
	if (!snapshot.fvmap) {
		ret = -ENODATA;
		dev_err(framework, "ACPM diagnostic snapshot has no attached DVFS FVMap\n");
	} else {
		ret = fvmap_validate_snapshot(snapshot.fvmap, ACPM_SNAPSHOT_FVMAP_BYTES);
		if (ret) {
			dev_err(framework, "ACPM owned FVMap validation failed: %d\n", ret);
		} else {
			/* 4.9: fvmap_init(plugin base + fvmap_offset) for the
			 * attached DVFS plugin. fvmap_init() re-validates the live
			 * SRAM copy before publishing voltage tables.
			 */
			void __iomem *sram = acpm_ipc_sram_ptr(snapshot.fvmap_offset,
							       ACPM_SNAPSHOT_FVMAP_BYTES);

			ret = sram ? fvmap_init(sram) : -ENODEV;
			dev_info(framework, "ACPM FVMap plugin=%u offset=%#x validated; fvmap_init=%d\n",
				 snapshot.fvmap_plugin, snapshot.fvmap_offset, ret);
		}
	}
	acpm_ipc_free_firmware_snapshot(&snapshot);
	return ret;
}

static int debug_log_level_get(void *data, unsigned long long *val)
{

	return 0;
}

static int debug_log_level_set(void *data, unsigned long long val)
{
	acpm_fw_log_level((unsigned int)val);

	return 0;
}

static int debug_ipc_loopback_test_get(void *data, unsigned long long *val)
{
	struct ipc_config config;
	int ret = 0;
	unsigned int cmd[4] = {0, };

	config.cmd = cmd;
	config.cmd[0] = (1 << ACPM_IPC_PROTOCOL_TEST);
	config.cmd[0] |= 0x3 << ACPM_IPC_PROTOCOL_ID;

	config.response = true;
	config.indirection = false;

	ret = acpm_send_data(exynos_acpm->dev->of_node, 3, &config);

	if (!ret)
		*val = ipc_time_end - ipc_time_start;
	else
		*val = 0;

	config.cmd = NULL;

	return 0;
}

static int debug_ipc_loopback_test_set(void *data, unsigned long long val)
{

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_log_level_fops,
		debug_log_level_get, debug_log_level_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(debug_ipc_loopback_test_fops,
		debug_ipc_loopback_test_get, debug_ipc_loopback_test_set, "%llu\n");

static void acpm_debugfs_init(struct acpm_info *acpm)
{
	struct dentry *den;

	den = debugfs_create_dir("acpm_framework", NULL);
	debugfs_create_file("ipc_loopback_test", 0644, den, acpm, &debug_ipc_loopback_test_fops);
	debugfs_create_file("log_level", 0644, den, NULL, &debug_log_level_fops);
}

void *memcpy_align_4(void *dest, const void *src, unsigned int n)
{
	unsigned int *dp = dest;
	const unsigned int *sp = src;
	int i;

	if ((n % 4))
		BUG();

	n = n >> 2;

	for (i = 0; i < n; i++)
		*dp++ = *sp++;

	return dest;
}

void acpm_enter_wfi(void)
{
	struct ipc_config config;
	int ret = 0;
	unsigned int cmd[4] = {0, };

	if (exynos_acpm->enter_wfi)
		return;

	config.cmd = cmd;
	config.response = true;
	config.indirection = false;
	config.cmd[0] = 1 << ACPM_IPC_PROTOCOL_STOP;

	ret = acpm_send_data(exynos_acpm->dev->of_node, ACPM_IPC_PROTOCOL_STOP, &config);

	config.cmd = NULL;

	if (ret) {
		pr_err("[ACPM] acpm enter wfi fail!!\n");
	} else {
		pr_err("[ACPM] wfi done\n");
		exynos_acpm->enter_wfi++;
	}
}

void exynos_acpm_timer_clear(void)
{
	if (!exynos_acpm || IS_ERR(exynos_acpm->timer_base))
		return;
	writel(exynos_acpm->timer_cnt, exynos_acpm->timer_base + EXYNOS_TIMER_APM_TCVR);
}

void exynos_acpm_reboot(void)
{
	u32 revision, subrevision;

	acpm_ipc_set_waiting_mode(BUSY_WAIT);

	id_get_rev(&revision, &subrevision);

	if (revision >= 1)
		acpm_enter_wfi();
}

static int acpm_send_data(struct device_node *node, unsigned int check_id,
		struct ipc_config *config)
{
	unsigned int channel_num, size;
	int ret = 0;
	int timeout_flag;
	unsigned int id = 0;

	if (!acpm_ipc_request_channel(node, NULL,
				&channel_num, &size)) {
		ipc_done = -1;

		ipc_time_start = sched_clock();
		ret = acpm_ipc_send_data(channel_num, config);

		if (config->cmd[0] & ACPM_IPC_PROTOCOL_DP_CMD) {
			id = config->cmd[0] & ACPM_IPC_PROTOCOL_IDX;
			id = id	>> ACPM_IPC_PROTOCOL_ID;
			ipc_done = id;
		} else if (config->cmd[0] & (1 << ACPM_IPC_PROTOCOL_STOP)) {
			ipc_done = ACPM_IPC_PROTOCOL_STOP;
		} else if (config->cmd[0] & (1 << ACPM_IPC_PROTOCOL_TEST)) {
			id = config->cmd[0] & ACPM_IPC_PROTOCOL_IDX;
			id = id	>> ACPM_IPC_PROTOCOL_ID;
			ipc_done = id;
		} else {
			id = config->cmd[0] & ACPM_IPC_PROTOCOL_IDX;
			id = id	>> ACPM_IPC_PROTOCOL_ID;
			ipc_done = id;
		}

		/* Response interrupt waiting */
		UNTIL_EQUAL(ipc_done, check_id, timeout_flag);

		if (timeout_flag)
			ret = -ETIMEDOUT;

		acpm_ipc_release_channel(node, channel_num);
	} else {
		pr_err("%s ipc request_channel fail, id:%u, size:%u\n",
				__func__, channel_num, size);
		ret = -EBUSY;
	}

	return ret;
}

static int acpm_probe_core(struct platform_device *pdev)
{
	struct acpm_info *acpm;
	struct device_node *node = pdev->dev.of_node;
	struct resource *res;
	int ret = 0;

	dev_info(&pdev->dev, "acpm probe\n");

	if (!node) {
		dev_err(&pdev->dev, "driver doesnt support"
				"non-dt devices\n");
		return -ENODEV;
	}

	acpm = devm_kzalloc(&pdev->dev,
			sizeof(struct acpm_info), GFP_KERNEL);
	if (IS_ERR(acpm))
		return PTR_ERR(acpm);
	if (!acpm)
		return -ENOMEM;

	acpm->dev = &pdev->dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "timer_apm");
	acpm->timer_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(acpm->timer_base))
		return PTR_ERR(acpm->timer_base);

	if (of_property_read_u32(node, "peritimer-cnt", &acpm->timer_cnt))
		pr_warn("No matching property: peritiemr_cnt\n");

	ret = acpm_ipc_bind_framework(&pdev->dev);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, acpm);
	exynos_acpm = acpm;

	acpm_debugfs_init(acpm);

	exynos_acpm_timer_clear();
	return ret;
}

static int acpm_probe(struct platform_device *pdev)
{
	int ret;

	mutex_lock(&acpm_framework_lock);
	ret = exynos_acpm ? -EBUSY : acpm_probe_core(pdev);
	mutex_unlock(&acpm_framework_lock);
	return ret;
}

static void acpm_remove(struct platform_device *pdev)
{
	mutex_lock(&acpm_framework_lock);
	if (exynos_acpm && exynos_acpm == platform_get_drvdata(pdev)) {
		acpm_ipc_unbind_framework(&pdev->dev);
		exynos_acpm = NULL;
		platform_set_drvdata(pdev, NULL);
	}
	mutex_unlock(&acpm_framework_lock);
}

static const struct of_device_id acpm_match[] = {
	{ .compatible = "samsung,exynos-acpm" },
	{},
};

static struct platform_driver samsung_acpm_driver = {
	.probe	= acpm_probe,
	.remove	= acpm_remove,
	.driver	= {
		.name = "exynos-acpm",
		.owner	= THIS_MODULE,
		.of_match_table	= acpm_match,
	},
};

static int __init exynos_acpm_init(void)
{
	return platform_driver_register(&samsung_acpm_driver);
}
arch_initcall_sync(exynos_acpm_init);

static int __init exynos_acpm_binary_update(void)
{
	int ret = -ENODEV;

	/* This one-shot diagnostic scan does not enable firmware updates or
	 * publish voltage tables, and does not pretend to implement deferral.
	 */
	mutex_lock(&acpm_framework_lock);
	if (exynos_acpm)
		ret = plugins_init(exynos_acpm->dev);
	mutex_unlock(&acpm_framework_lock);
	return ret;
}
fs_initcall_sync(exynos_acpm_binary_update);
