// SPDX-License-Identifier: GPL-2.0
/* Experimental crownlte headless diagnostic return, never an Android ABI. */
#include <linux/device.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/rcupdate.h>
#include <soc/samsung/acpm_ipc_ctrl.h>
#include <soc/samsung/cal-if.h>
#include "cal-if/asv.h"

#define INFORM2 0x808
#define INFORM3 0x80c
#define NORMAL_INFORM2 0x12345678
#define NORMAL_INFORM3 0x12345670
#define SWRESET 0x400
#define DFD 0x500
#define DFD_DISABLE_RESET (BIT(9) | BIT(10) | BIT(11) | (0xfU << 24))
#define DFD_CLEAR_COMMAND 0x00f000e1

struct diagnostic_restart {
	void __iomem *pmu;
	struct device *supplier;
	struct device_node *framework;
	struct notifier_block command_notifier;
	struct sys_off_handler *prepare_handler;
	unsigned int revision;
	bool default_requested;
	bool prepared;
};

/* This builtin guard survives probe failure and consumer/supplier unbind. */
static struct diagnostic_restart __rcu *diagnostic_context;
static DEFINE_MUTEX(diagnostic_context_lock);

static bool diagnostic_default_cmd(const char *cmd)
{
	return !cmd || !cmd[0];
}

static int diagnostic_command(struct notifier_block *nb,
			      unsigned long event, void *cmd)
{
	struct diagnostic_restart *ctx =
		container_of(nb, struct diagnostic_restart, command_notifier);

	WRITE_ONCE(ctx->prepared, false);
	WRITE_ONCE(ctx->default_requested,
		   event == SYS_RESTART && diagnostic_default_cmd(cmd));
	return NOTIFY_DONE;
}

static bool diagnostic_reasons_valid(struct diagnostic_restart *ctx)
{
	return readl(ctx->pmu + INFORM2) == NORMAL_INFORM2 &&
	       readl(ctx->pmu + INFORM3) == NORMAL_INFORM3;
}

/* Diagnostic-only original reset policy. An unavailable or failed STOP
 * remains visible, but does not veto the independently checked PMU sequence.
 * The supplier reference alone does not own drvdata or devres. device_trylock
 * excludes probe/remove and their resource retirement across this entire call.
 * This function runs in the blocking restart-prepare chain, never final reset.
 */
static void diagnostic_optional_stop(struct diagnostic_restart *ctx)
{
	int ret = -ENODEV;
	bool available;

	if (!ctx->supplier) {
		pr_emerg("Experimental diagnostic STOP unavailable: no supplier device; original-style PMU reset remains permitted\n");
		return;
	}
	/* No dependency wait may prevent this best-effort fallback. */
	if (!device_trylock(ctx->supplier)) {
		pr_emerg("Experimental diagnostic STOP unavailable: supplier busy; original-style PMU reset remains permitted\n");
		return;
	}
	available = device_is_bound(ctx->supplier) &&
		    dev_get_drvdata(ctx->supplier);
	if (available)
		ret = acpm_ipc_prepare_restart(ctx->supplier, ctx->framework);
	device_unlock(ctx->supplier);
	/* Only owned scalar results survive supplier unlock/possible retirement. */
	if (!available)
		pr_emerg("Experimental diagnostic STOP unavailable: supplier unbound or unpublished; original-style PMU reset remains permitted\n");
	else if (ret)
		pr_emerg("Experimental diagnostic STOP failed: %d; original-style PMU reset remains permitted\n", ret);
	else
		pr_emerg("Experimental diagnostic STOP acknowledged\n");
}

static int diagnostic_prepare(struct sys_off_data *data)
{
	struct diagnostic_restart *ctx = data->cb_data;
	unsigned int main_rev, sub_rev;
	int ret;

	WRITE_ONCE(ctx->prepared, false);
	if (!READ_ONCE(ctx->default_requested) || !cal_if_is_initialized())
		return NOTIFY_DONE;
	id_get_rev(&main_rev, &sub_rev);
	if (main_rev != ctx->revision)
		return NOTIFY_DONE;
	/* The original SEC wrapper writes INFORM2 then INFORM3 for normal boot. */
	writel(NORMAL_INFORM2, ctx->pmu + INFORM2);
	if (readl(ctx->pmu + INFORM2) != NORMAL_INFORM2)
		return NOTIFY_DONE;
	writel(NORMAL_INFORM3, ctx->pmu + INFORM3);
	if (!diagnostic_reasons_valid(ctx))
		return NOTIFY_DONE;
	if (ctx->revision >= 1) {
		if (IS_ENABLED(CONFIG_EXYNOS9810_DIAGNOSTIC_ORIGINAL_RESET)) {
			diagnostic_optional_stop(ctx);
		} else {
			/* Keep the existing strict policy when the option is disabled. */
			ret = acpm_ipc_prepare_restart(ctx->supplier, ctx->framework);
			if (ret) {
				pr_emerg("Diagnostic restart STOP refused: %d\n", ret);
				return NOTIFY_DONE;
			}
		}
	}
	smp_store_release(&ctx->prepared, true);
	return NOTIFY_DONE;
}

static int diagnostic_clear_mask(struct diagnostic_restart *ctx,
				 u32 offset, u32 mask)
{
	u32 value = readl(ctx->pmu + offset);

	writel(value & ~mask, ctx->pmu + offset);
	return readl(ctx->pmu + offset) & mask ? -EIO : 0;
}

static int diagnostic_dfd_prepare(struct diagnostic_restart *ctx)
{
	unsigned int i;
	int ret;

	if (!(readl(ctx->pmu + DFD) & BIT(0)))
		return 0;
	/* Original Exynos9810 DFD reset preparation, fixed four cores per cluster. */
	if (ctx->revision >= 1)
		for (i = 0; i < 4; i++) {
			ret = diagnostic_clear_mask(ctx, 0x3c4c + i * 0x10, 0x1100);
			if (ret)
				return ret;
		}
	for (i = 0; i < 4; i++) {
		ret = diagnostic_clear_mask(ctx, 0x3c0c + i * 0x10, 0x3300);
		if (ret)
			return ret;
	}
	ret = diagnostic_clear_mask(ctx, 0x3c8c, 0x84040000);
	if (ret)
		return ret;
	/* Original clear strobes may self-clear; verify the reset-disable state. */
	writel(DFD_CLEAR_COMMAND, ctx->pmu + DFD);
	return readl(ctx->pmu + DFD) & DFD_DISABLE_RESET ? -EIO : 0;
}

static int diagnostic_restart_final(struct sys_off_data *data)
{
	struct diagnostic_restart *ctx = data->cb_data;

	if (!diagnostic_default_cmd(data->cmd) ||
	    !smp_load_acquire(&ctx->prepared) ||
	    !READ_ONCE(ctx->default_requested) ||
	    !cal_if_is_initialized() || !diagnostic_reasons_valid(ctx) ||
	    diagnostic_dfd_prepare(ctx)) {
		pr_emerg("Diagnostic restart prerequisites failed; physical reset required\n");
		return NOTIFY_STOP;
	}
	/* DFD preparation must not have changed either boot-selection reason. */
	if (!diagnostic_reasons_valid(ctx))
		return NOTIFY_STOP;
	writel(1, ctx->pmu + SWRESET);
	/* If hardware did not reset, do not fall through to PSCI/generic handlers. */
	return NOTIFY_STOP;
}

static int diagnostic_restart_guard(struct notifier_block *nb,
				    unsigned long mode, void *cmd)
{
	struct diagnostic_restart *ctx;
	struct sys_off_data data = { .mode = mode, .cmd = cmd };
	int ret = NOTIFY_STOP;

	(void)nb;
	rcu_read_lock();
	ctx = rcu_dereference(diagnostic_context);
	if (ctx) {
		data.cb_data = ctx;
		ret = diagnostic_restart_final(&data);
	} else {
		pr_emerg("Diagnostic restart context unavailable; physical reset required\n");
	}
	rcu_read_unlock();
	return ret;
}

static struct notifier_block diagnostic_guard = {
	.notifier_call = diagnostic_restart_guard,
	.priority = 255,
};

static bool diagnostic_topology_valid(void)
{
	struct device_node *cpu;
	unsigned long mask = 0;
	u32 reg;
	unsigned int bit;

	for_each_of_cpu_node(cpu) {
		if (of_property_read_u32(cpu, "reg", &reg))
			goto invalid;
		if (reg <= 3 && of_device_is_compatible(cpu, "arm,cortex-a55"))
			bit = reg;
		else if (reg >= 0x100 && reg <= 0x103 &&
			 of_device_is_compatible(cpu, "samsung,mongoose-m3"))
			bit = reg - 0x100 + 4;
		else
			goto invalid;
		if (mask & BIT(bit))
			goto invalid;
		mask |= BIT(bit);
	}
	return mask == 0xff;
invalid:
	of_node_put(cpu);
	return false;
}

static void diagnostic_supplier_put(void *data)
{
	put_device(data);
}

static void diagnostic_framework_put(void *data)
{
	of_node_put(data);
}

static void diagnostic_retire_context(void *data)
{
	struct diagnostic_restart *ctx = data;

	/* Blocking notifier unregistration waits for any preparation callback. */
	unregister_sys_off_handler(ctx->prepare_handler);
	mutex_lock(&diagnostic_context_lock);
	if (rcu_access_pointer(diagnostic_context) == ctx)
		rcu_assign_pointer(diagnostic_context, NULL);
	/* Final atomic callback's read-side ownership ends before devres release. */
	synchronize_rcu();
	mutex_unlock(&diagnostic_context_lock);
}

static int diagnostic_restart_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct diagnostic_restart *ctx;
	struct platform_device *supplier;
	struct device_node *np;
	struct resource res;
	unsigned int sub_rev;
	int ret;

	if (!of_machine_is_compatible("samsung,crownlte") ||
	    !diagnostic_topology_valid())
		return -EINVAL;
	if (!cal_if_is_initialized())
		return -EPROBE_DEFER;
	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	id_get_rev(&ctx->revision, &sub_rev);
	if (ctx->revision > 15)
		return -EINVAL;
	ret = of_address_to_resource(dev->of_node, 0, &res);
	if (ret || res.start != 0x14060000 || resource_size(&res) != 0x4000)
		return -EINVAL;
	/* CAL already maps this shared PMU; avoid claiming its resource twice. */
	ctx->pmu = devm_ioremap(dev, res.start, resource_size(&res));
	if (!ctx->pmu)
		return -ENOMEM;
	ctx->framework = of_parse_phandle(dev->of_node, "samsung,acpm-framework", 0);
	if (!ctx->framework ||
	    !of_device_is_compatible(ctx->framework, "samsung,exynos-acpm")) {
		of_node_put(ctx->framework);
		return -EINVAL;
	}
	ret = devm_add_action_or_reset(dev, diagnostic_framework_put, ctx->framework);
	if (ret)
		return ret;
	np = of_parse_phandle(dev->of_node, "samsung,acpm-ipc", 0);
	if (!np || !of_device_is_compatible(np, "samsung,exynos-acpm-ipc")) {
		of_node_put(np);
		return -EINVAL;
	}
	supplier = of_find_device_by_node(np);
	of_node_put(np);
	if (IS_ENABLED(CONFIG_EXYNOS9810_DIAGNOSTIC_ORIGINAL_RESET)) {
		/* Optional transport must not control the independent PMU context.
		 * Keep only a device reference here. A dependency link would make
		 * supplier unbind retire this reset context. Each STOP call instead
		 * holds the supplier lock until all owned transport access ends.
		 */
		if (supplier) {
			ctx->supplier = &supplier->dev;
			ret = devm_add_action_or_reset(dev, diagnostic_supplier_put,
						       ctx->supplier);
			if (ret)
				return ret;
		}
	} else {
		if (!supplier)
			return -EPROBE_DEFER;
		ctx->supplier = &supplier->dev;
		ret = devm_add_action_or_reset(dev, diagnostic_supplier_put, ctx->supplier);
		if (ret)
			return ret;
		device_lock(ctx->supplier);
		if (!device_is_bound(ctx->supplier) || !dev_get_drvdata(ctx->supplier))
			ret = -EPROBE_DEFER;
		else if (!device_link_add(dev, ctx->supplier, DL_FLAG_AUTOREMOVE_CONSUMER))
			ret = -EINVAL;
		else
			ret = 0;
		device_unlock(ctx->supplier);
		if (ret)
			return ret;
	}
	/* Devres unregisters callbacks before the optional supplier ref and MMIO disappear. */
	ctx->command_notifier.notifier_call = diagnostic_command;
	ret = devm_register_reboot_notifier(dev, &ctx->command_notifier);
	if (ret)
		return ret;
	ctx->prepare_handler = register_sys_off_handler(SYS_OFF_MODE_RESTART_PREPARE,
						       255, diagnostic_prepare, ctx);
	if (IS_ERR(ctx->prepare_handler))
		return PTR_ERR(ctx->prepare_handler);
	ret = devm_add_action_or_reset(dev, diagnostic_retire_context, ctx);
	if (ret)
		return ret;
	mutex_lock(&diagnostic_context_lock);
	if (rcu_access_pointer(diagnostic_context))
		ret = -EBUSY;
	else
		rcu_assign_pointer(diagnostic_context, ctx);
	mutex_unlock(&diagnostic_context_lock);
	if (!ret) {
		if (IS_ENABLED(CONFIG_EXYNOS9810_DIAGNOSTIC_ORIGINAL_RESET))
			dev_warn(dev, "Experimental original-style diagnostic reset ready; STOP best-effort, hardware unproven\n");
		else
			dev_info(dev, "Experimental checked diagnostic restart ready; guard installed\n");
	}
	return ret;
}

static const struct of_device_id diagnostic_restart_match[] = {
	{ .compatible = "samsung,exynos9810-crownlte-diagnostic-restart" },
	{}
};

static struct platform_driver diagnostic_restart_driver = {
	.probe = diagnostic_restart_probe,
	.driver = {
		.name = "exynos9810-diagnostic-restart",
		.of_match_table = diagnostic_restart_match,
	},
};

static int __init diagnostic_restart_init(void)
{
	struct device_node *np;
	int ret;
	bool enabled;

	np = of_find_compatible_node(NULL, NULL,
			"samsung,exynos9810-crownlte-diagnostic-restart");
	enabled = np && of_device_is_available(np);
	of_node_put(np);
	if (!enabled)
		return 0;
	ret = register_restart_handler(&diagnostic_guard);
	if (ret)
		return ret;
	/* If driver registration/probe fails, retain the independent refusal guard. */
	return platform_driver_register(&diagnostic_restart_driver);
}
device_initcall(diagnostic_restart_init);
