// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos9810 (crownlte) platform layer for the Samsung r38p1-01bet0 kbase.
 *
 * Replaces the 4.9 Samsung "gpex" stack (drivers/gpu/arm/exynos, ~14k
 * lines) with the parts this phone needs:
 *  - power: the PMU "embedded_g3d" domain is NOT switched in stage 1. 4.9
 *    cal_pd_control(0xb1380008) powers it with an S2MPS18 BUCK6 (vdd_g3d)
 *    B6CTRL update over ACPM, a PMU write and a DTZPC restore SMC; the 6.12
 *    ACPM PMIC write gate refuses that update by design. The GPU is used
 *    only if SBoot left the domain on, and it is never powered off here:
 *    kbase still power-gates L2 and shader cores through GPU registers.
 *  - clock: G3D DVFS through cal_dfs_set_rate() on dvfs_g3d (ACPM vclk,
 *    firmware-chosen voltage), see mali_exynos9810_dvfs.c.
 *  - the kbase core hooks of mali_exynos_kbase_entrypoint.h and the Samsung
 *    private ioctls (mali_exynos9810_entrypoint.c).
 *
 * Fail-safe: probe stops before /dev/mali0 is registered (and before kbase
 * touches a GPU register) unless the PMU reports the domain on, the CMU_G3D
 * user mux selects the G3D clock, CAL/ACPM DVFS for dvfs_g3d is ready, the
 * CMU readback is a CAL level and GPU_ID is a Mali-G72 (THEx).
 */
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/suspend.h>

#include <mali_kbase.h>
#include <mali_kbase_config.h>
#include <soc/samsung/cal-if.h>

#include "mali_exynos9810.h"

struct exynos9810_gpu *exynos9810_gpu;

bool xg_pd_is_on(struct exynos9810_gpu *g, u32 *cfg, u32 *status)
{
	unsigned int c = 0, s = 0;

	if (regmap_read(g->pmu, XG_PMU_G3D_CONFIGURATION, &c) ||
	    regmap_read(g->pmu, XG_PMU_G3D_STATUS, &s)) {
		c = 0;
		s = 0;
	}
	if (cfg)
		*cfg = c;
	if (status)
		*status = s;
	return (c & XG_PMU_G3D_LOCAL_PWR_MASK) == XG_PMU_G3D_LOCAL_PWR_MASK &&
	       (s & XG_PMU_G3D_LOCAL_PWR_MASK) == XG_PMU_G3D_LOCAL_PWR_MASK;
}

u32 xg_cmu_mux(struct exynos9810_gpu *g)
{
	return readl_relaxed(g->cmu + XG_CMU_G3D_MUX_USER);
}

/*
 * System suspend is vetoed while the GPU is bound: the 4.9 sleep path
 * powered the domain off and on through cal_pd_control (PMIC + DTZPC SMC),
 * which stage 1 does not do, and kbase would touch GPU registers on resume.
 */
static int xg_pm_notify(struct notifier_block *nb, unsigned long action, void *data)
{
	if (action == PM_SUSPEND_PREPARE || action == PM_HIBERNATION_PREPARE) {
		pr_warn_ratelimited(XG_PFX "system suspend refused: G3D domain sleep handling not ported (stage 1)\n");
		return NOTIFY_BAD;
	}
	return NOTIFY_DONE;
}

static struct notifier_block xg_pm_nb = {
	.notifier_call = xg_pm_notify,
};

static int xg_check_gpu_id(struct exynos9810_gpu *g)
{
	struct kbase_device *kbdev = g->kbdev;
	u32 id;

	if (!kbdev->reg)
		return -ENODEV;
	/* Plain MMIO read: kbase has not started its own register access yet. */
	id = readl(kbdev->reg + GPU_CONTROL_REG(GPU_ID));
	g->gpu_id = id;
	if (id == 0 || id == ~0u ||
	    (id & GPU_ID2_PRODUCT_MODEL) != GPU_ID2_PRODUCT_THEX) {
		dev_err(g->dev, XG_PFX "GPU_ID %#010x is not a Mali-G72 (THEx); /dev/mali0 not registered\n",
			id);
		return -ENODEV;
	}
	dev_info(g->dev, XG_PFX "GPU_ID %#010x: Mali-G72 arch %u.%u.%u r%up%u status %u\n",
		 id, (id & GPU_ID2_ARCH_MAJOR) >> GPU_ID2_ARCH_MAJOR_SHIFT,
		 (id & GPU_ID2_ARCH_MINOR) >> GPU_ID2_ARCH_MINOR_SHIFT,
		 (id & GPU_ID2_ARCH_REV) >> GPU_ID2_ARCH_REV_SHIFT,
		 (id & GPU_ID2_VERSION_MAJOR) >> GPU_ID2_VERSION_MAJOR_SHIFT,
		 (id & GPU_ID2_VERSION_MINOR) >> GPU_ID2_VERSION_MINOR_SHIFT,
		 (id & GPU_ID2_VERSION_STATUS) >> GPU_ID2_VERSION_STATUS_SHIFT);
	return 0;
}

static int xg_platform_init(struct kbase_device *kbdev)
{
	struct device *dev = kbdev->dev;
	struct device_node *np = dev->of_node;
	struct exynos9810_gpu *g;
	u32 cfg, st, mux;
	int ret;

	if (exynos9810_gpu)
		return -EBUSY;

	g = kzalloc(sizeof(*g), GFP_KERNEL);
	if (!g)
		return -ENOMEM;
	g->kbdev = kbdev;
	g->dev = dev;
	spin_lock_init(&g->lock);
	mutex_init(&g->set_lock);

	/* 1. PMU embedded_g3d domain (read only). */
	g->pmu = syscon_regmap_lookup_by_phandle(np, "samsung,pmu-syscon");
	if (IS_ERR(g->pmu)) {
		ret = dev_err_probe(dev, PTR_ERR(g->pmu), XG_PFX "no PMU syscon\n");
		goto fail;
	}
	if (!xg_pd_is_on(g, &cfg, &st)) {
		dev_err(dev, XG_PFX "G3D power domain is off (PMU cfg %#x status %#x); stage 1 never powers it, /dev/mali0 not registered\n",
			cfg, st);
		ret = -ENODEV;
		goto fail;
	}

	/* 2. CMU_G3D user mux must select the G3D clock (pmucal embedded_g3d_on). */
	g->cmu = of_iomap(np, 1);
	if (!g->cmu) {
		dev_err(dev, XG_PFX "no CMU_G3D register window (reg index 1)\n");
		ret = -ENODEV;
		goto fail;
	}
	mux = xg_cmu_mux(g);
	if (!(mux & XG_CMU_G3D_MUX_USER_SEL)) {
		dev_err(dev, XG_PFX "CMU_G3D MUX_CLKCMU_EMBEDDED_G3D_USER %#x selects OSCCLK; /dev/mali0 not registered\n",
			mux);
		ret = -ENODEV;
		goto fail;
	}

	/* 3. CAL/ACPM DVFS for dvfs_g3d, ECT levels and CMU readback. */
	ret = xg_dvfs_init(g);
	if (ret)
		goto fail;

	/* 4. GPU_ID: only after the domain checks above. */
	ret = xg_check_gpu_id(g);
	if (ret)
		goto fail;

	dev_info(dev, XG_PFX "G3D domain on (PMU cfg %#x status %#x), CMU mux %#x, %u kHz, %u DVFS levels%s\n",
		 cfg, st, mux, g->boot_khz, g->nr_levels,
		 g->dvfs_enabled ? "" : " (DVFS off)");

	kbdev->platform_context = g;
	exynos9810_gpu = g;
	return 0;

fail:
	if (g->cmu)
		iounmap(g->cmu);
	kfree(g);
	return ret;
}

static void xg_platform_term(struct kbase_device *kbdev)
{
	struct exynos9810_gpu *g = kbdev->platform_context;

	if (!g)
		return;
	kbdev->platform_context = NULL;
	exynos9810_gpu = NULL;
	if (g->cmu)
		iounmap(g->cmu);
	kfree(g);
}

static int xg_platform_late_init(struct kbase_device *kbdev)
{
	struct exynos9810_gpu *g = kbdev->platform_context;
	int ret;

	if (!g)
		return -ENODEV;
	ret = xg_dvfs_late_init(g);
	if (ret)
		return ret;
	if (xg_sysfs_init(g))
		dev_warn(g->dev, XG_PFX "/sys/kernel/gpu not created\n");
	ret = register_pm_notifier(&xg_pm_nb);
	if (ret) {
		xg_sysfs_term(g);
		xg_dvfs_term(g);
		return ret;
	}
	xg_selftest_schedule(g);
	return 0;
}

static void xg_platform_late_term(struct kbase_device *kbdev)
{
	struct exynos9810_gpu *g = kbdev->platform_context;

	if (!g)
		return;
	xg_selftest_cancel(g);
	unregister_pm_notifier(&xg_pm_nb);
	xg_sysfs_term(g);
	xg_dvfs_term(g);
}

struct kbase_platform_funcs_conf exynos9810_gpu_platform_funcs = {
	.platform_init_func = xg_platform_init,
	.platform_term_func = xg_platform_term,
	.platform_late_init_func = xg_platform_late_init,
	.platform_late_term_func = xg_platform_late_term,
};

/*
 * kbase power callbacks. The PMU domain stays on, so GPU state is never
 * lost across these: return 0 ("no reset needed"); kbase resets the GPU
 * itself in kbase_pm_init_hw() at power-up.
 */
static int xg_power_on(struct kbase_device *kbdev)
{
	struct exynos9810_gpu *g = kbdev->platform_context;
	u32 cfg, st;

	if (!g)
		return 0;
	g->nr_power_on++;
	if (!xg_pd_is_on(g, &cfg, &st))
		dev_err_ratelimited(g->dev, XG_PFX "G3D power domain off at GPU power-on (PMU cfg %#x status %#x)\n",
				    cfg, st);
	xg_dvfs_power_on(g);
	return 0;
}

static void xg_power_off(struct kbase_device *kbdev)
{
	struct exynos9810_gpu *g = kbdev->platform_context;

	if (!g)
		return;
	g->nr_power_off++;
	xg_dvfs_power_off(g);
}

struct kbase_pm_callback_conf exynos9810_gpu_pm_callbacks = {
	.power_on_callback = xg_power_on,
	.power_off_callback = xg_power_off,
};
