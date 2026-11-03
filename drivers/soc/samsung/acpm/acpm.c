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

/*
 * Dynamic plugins this kernel may load. Only plugins named in the framework
 * node's "dynamic-plugins" DT children are ever written to SRAM; 4.9 did
 * every detached stay_attached plugin, but loaded only images it shipped.
 */
#define ACPM_DYNAMIC_PLUGINS_MAX	4
struct acpm_dynamic_plugin {
	const char *fw_name;
	struct exynos_acpm_plugin_status status;
};
static struct acpm_dynamic_plugin acpm_dynamic_plugins[ACPM_DYNAMIC_PLUGINS_MAX];
static unsigned int acpm_dynamic_plugin_count;
static bool acpm_dynamic_plugins_done;

/* 4.9 handle_dynamic_plugin(), attach only; keeps the reply for logging. */
static int handle_dynamic_plugin(struct device_node *node, unsigned int id,
				 u32 response[4])
{
	struct ipc_config config;
	unsigned int cmd[4] = {0, };
	int ret;

	config.cmd = cmd;
	config.cmd[0] = (1 << ACPM_IPC_PROTOCOL_DP_A);
	config.cmd[0] |= id << ACPM_IPC_PROTOCOL_ID;
	config.response = true;
	config.indirection = false;

	ret = acpm_send_data(node, id, &config);
	memcpy(response, cmd, sizeof(cmd));
	config.cmd = NULL;
	if (!ret)
		pr_info("[ACPM] plugin(id = %u) attach done!\n", id);
	return ret;
}

static void acpm_attach_dynamic_plugin(struct device *framework,
				       const struct acpm_firmware_snapshot *snapshot,
				       struct acpm_dynamic_plugin *plugin,
				       u32 expected_base)
{
	struct exynos_acpm_plugin_status *st = &plugin->status;
	const struct acpm_plugin_snapshot *entry = NULL;
	struct acpm_firmware_snapshot after;
	const struct firmware *fw = NULL;
	unsigned int i;
	int ret;

	memset(st, 0, sizeof(*st));
	st->result = -ENOENT;
	for (i = 0; i < snapshot->num_plugins; i++) {
		if (strncmp(snapshot->plugins[i].name, plugin->fw_name,
			    ACPM_PLUGIN_NAME_SIZE))
			continue;
		if (entry) {
			st->result = -EPROTO;
			dev_err(framework, "ACPM plugin %s listed twice\n", plugin->fw_name);
			return;
		}
		entry = &snapshot->plugins[i];
		st->index = i;
	}
	if (!entry) {
		dev_err(framework, "ACPM plugin %s not in firmware table\n", plugin->fw_name);
		return;
	}
	st->found = true;
	st->id = entry->id;
	st->base = entry->base;
	st->size = entry->bytes;
	if (entry->attached && entry->stay_attached) {
		st->result = 0;
		dev_info(framework, "ACPM plugin %s index=%u id=%u already attached; nothing to load\n",
			 plugin->fw_name, st->index, st->id);
		return;
	}
	if (entry->attached || !entry->stay_attached) {
		/* 4.9 would DP_D this one; never needed for TMU, so refuse. */
		st->result = -EOPNOTSUPP;
		dev_err(framework, "ACPM plugin %s attached=%u stay=%u: not a load+attach case\n",
			plugin->fw_name, entry->attached, entry->stay_attached);
		return;
	}
	if (entry->id > (ACPM_IPC_PROTOCOL_IDX >> ACPM_IPC_PROTOCOL_ID)) {
		st->result = -EPROTO;
		return;
	}
	/* Built-in (CONFIG_EXTRA_FIRMWARE) first; no usermode helper fallback. */
	ret = request_firmware_direct(&fw, plugin->fw_name, framework);
	if (ret) {
		st->result = ret;
		dev_err(framework, "ACPM plugin %s firmware request failed: %d\n",
			plugin->fw_name, ret);
		return;
	}
	st->image_bytes = fw->size;
	ret = acpm_ipc_load_dynamic_plugin(framework, st->index, st->id, expected_base,
					   fw->data, fw->size);
	release_firmware(fw);
	if (ret) {
		st->result = ret;
		return;
	}
	st->loaded = true;
	ret = handle_dynamic_plugin(framework->of_node, st->id, st->dp_response);
	st->attach_ret = ret;
	st->result = ret;
	if (ret)
		dev_err(framework, "plugin attach/detach:1 fail! plugin_id:%u, ret:%d\n",
			st->id, ret);

	if (!acpm_ipc_copy_firmware_snapshot(framework, &after)) {
		if (st->index < after.num_plugins &&
		    after.plugins[st->index].id == st->id)
			st->post_attached = after.plugins[st->index].attached;
		acpm_ipc_free_firmware_snapshot(&after);
	}
	dev_info(framework, "ACPM plugin %s index=%u id=%u DP_A ret=%d reply=%08x %08x %08x %08x is_attached now=%u\n",
		 plugin->fw_name, st->index, st->id, ret,
		 st->dp_response[0], st->dp_response[1], st->dp_response[2],
		 st->dp_response[3], st->post_attached);
}

static void acpm_dynamic_plugins_init(struct device *framework,
				      const struct acpm_firmware_snapshot *snapshot)
{
	struct device_node *list, *child;
	struct acpm_dynamic_plugin *plugin;
	u32 expected_base;

	list = of_get_child_by_name(framework->of_node, "dynamic-plugins");
	if (!list)
		return;
	for_each_available_child_of_node(list, child) {
		if (acpm_dynamic_plugin_count >= ACPM_DYNAMIC_PLUGINS_MAX) {
			dev_err(framework, "too many dynamic-plugins entries\n");
			of_node_put(child);
			break;
		}
		plugin = &acpm_dynamic_plugins[acpm_dynamic_plugin_count];
		if (of_property_read_string(child, "fw-name", &plugin->fw_name) ||
		    of_property_read_u32(child, "samsung,load-offset", &expected_base)) {
			dev_err(framework, "%pOF: needs fw-name and samsung,load-offset\n", child);
			continue;
		}
		acpm_dynamic_plugin_count++;
		acpm_attach_dynamic_plugin(framework, snapshot, plugin, expected_base);
	}
	of_node_put(list);
}

int exynos_acpm_get_plugin_status(const char *fw_name,
				  struct exynos_acpm_plugin_status *status)
{
	unsigned int i;
	int ret = -ENOENT;

	if (!fw_name || !status)
		return -EINVAL;
	mutex_lock(&acpm_framework_lock);
	if (!acpm_dynamic_plugins_done) {
		ret = -EPROBE_DEFER;
	} else {
		for (i = 0; i < acpm_dynamic_plugin_count; i++) {
			if (strcmp(acpm_dynamic_plugins[i].fw_name, fw_name))
				continue;
			*status = acpm_dynamic_plugins[i].status;
			ret = status->result;
			break;
		}
	}
	mutex_unlock(&acpm_framework_lock);
	return ret;
}

static int plugins_init(struct device *framework)
{
	struct acpm_firmware_snapshot snapshot;
	int ret;

	ret = acpm_ipc_copy_firmware_snapshot(framework, &snapshot);
	if (ret) {
		dev_err(framework, "ACPM diagnostic snapshot unavailable: %d\n", ret);
		return ret;
	}
	acpm_dynamic_plugins_init(framework, &snapshot);
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

	/* One-shot scan: loads+attaches only DT-listed dynamic plugins (4.9
	 * firmware_update + DP_A), publishes the attached DVFS FVMap, and does
	 * not pretend to implement deferral.
	 */
	mutex_lock(&acpm_framework_lock);
	if (exynos_acpm)
		ret = plugins_init(exynos_acpm->dev);
	/* Consumers stop deferring even when the framework never probed. */
	acpm_dynamic_plugins_done = true;
	mutex_unlock(&acpm_framework_lock);
	return ret;
}
fs_initcall_sync(exynos_acpm_binary_update);
