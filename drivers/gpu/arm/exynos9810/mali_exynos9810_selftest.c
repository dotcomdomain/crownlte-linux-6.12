// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos9810 (crownlte) Mali diagnostic self-test, DT property
 * samsung,mali-selftest on the gpu node. Runs once at ~109 s uptime and
 * logs with the prefix "mali selftest:".
 *
 *  1. PMU G3D domain / CMU_G3D mux / clock snapshot (reads only).
 *  2. kbase_pm_context_active(): kbase powers L2 and shader cores through
 *     the GPU's own PWRON registers; read GPU_ID, L2/shader/tiler present
 *     and ready masks, GPU_STATUS.
 *  3. GPU_COMMAND CLEAN_INV_CACHES through kbase, wait for completion.
 *  4. MMU: AS_STATUS of every address space and MMU_IRQ_RAWSTAT (reads only;
 *     no address space is programmed without a kbase context).
 *  5. DVFS: the samsung,selftest-khz levels through cal_dfs_set_rate() with
 *     CMU readback and a GPU_ID read at each level, then the original level.
 *  6. kbase_pm_context_idle(): wait until kbase powers the GPU down.
 *  7. Only with samsung,g3d-power-control: one G3D power-domain cycle
 *     (4.9 cal_pd_control off, 20 ms, on), then kbase power-up again with a
 *     GPU_ID / L2 / cache-clean check (kbase resets the GPU first) and
 *     power-down.
 * No job is submitted (that needs a user context and job memory; the first
 * real job comes from libGLES_mali). Bounded to well under 3 s.
 */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/timekeeping.h>
#include <linux/workqueue.h>

#include <mali_kbase.h>
#include <backend/gpu/mali_kbase_pm_internal.h>
#include <device/mali_kbase_device.h>
#include <soc/samsung/acpm_mfd.h>
#include <soc/samsung/cal-if.h>

#include "mali_exynos9810.h"

#define XG_PD_CAL_ID		0xb1380008	/* 4.9 pd-embedded_g3d cal_id */
#define XG_ST_AT_MS		109000
#define XG_ST_MAX_LEVELS	4
#define XG_ST_POWEROFF_MS	1000

static bool xg_st_armed;

static s64 xg_us_since(ktime_t t)
{
	return ktime_us_delta(ktime_get(), t);
}

static bool xg_gpu_powered(struct kbase_device *kbdev)
{
	unsigned long flags;
	bool on;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	on = kbdev->pm.backend.gpu_powered;
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
	return on;
}

static int xg_st_dvfs(struct exynos9810_gpu *g)
{
	struct kbase_device *kbdev = g->kbdev;
	struct device_node *np = g->dev->of_node;
	u32 khz[XG_ST_MAX_LEVELS] = { 455000, 338000 };
	unsigned int orig, i;
	int n = 2, fails = 0;

	if (!g->dvfs_enabled) {
		pr_info(XG_ST_PFX "dvfs: skipped (no DVFS table)\n");
		return 0;
	}
	if (of_property_present(np, "samsung,selftest-khz")) {
		n = of_property_read_variable_u32_array(np, "samsung,selftest-khz", khz,
							1, XG_ST_MAX_LEVELS);
		if (n < 0) {
			pr_err(XG_ST_PFX "dvfs: bad samsung,selftest-khz (%d)\n", n);
			return 1;
		}
	}

	xg_dvfs_hold(g, true);
	mutex_lock(&g->set_lock);
	orig = g->cur_step;
	mutex_unlock(&g->set_lock);

	for (i = 0; i < n; i++) {
		int step = xg_dvfs_khz_to_step(g, khz[i]);
		unsigned long rb = 0;
		ktime_t t;
		u32 id;
		int ret;

		if (step < 0) {
			pr_err(XG_ST_PFX "dvfs %u kHz: not a table level\n", khz[i]);
			fails++;
			continue;
		}
		t = ktime_get();
		ret = xg_dvfs_apply(g, step, &rb);
		id = kbase_reg_read(kbdev, GPU_CONTROL_REG(GPU_ID));
		pr_info(XG_ST_PFX "dvfs %u kHz: set %d, CMU readback %lu kHz, GPU_ID %#010x, %lld us: %s\n",
			khz[i], ret, rb, id, xg_us_since(t),
			!ret && rb == khz[i] && id == g->gpu_id ? "PASS" : "FAIL");
		if (ret || rb != khz[i] || id != g->gpu_id)
			fails++;
	}

	if (orig >= g->nr_levels)
		orig = g->nr_levels - 1;
	{
		unsigned long rb = 0;
		int ret = xg_dvfs_apply(g, orig, &rb);

		pr_info(XG_ST_PFX "dvfs restore %u kHz: set %d, CMU readback %lu kHz\n",
			g->levels[orig].khz, ret, rb);
		if (ret)
			fails++;
	}
	xg_dvfs_hold(g, false);
	return fails;
}

/* Wait until kbase has powered the GPU off; true if it did. */
static bool xg_st_wait_gpu_off(struct kbase_device *kbdev, s64 *ms)
{
	ktime_t t = ktime_get();

	while (xg_gpu_powered(kbdev) && ktime_ms_delta(ktime_get(), t) < XG_ST_POWEROFF_MS)
		msleep(10);
	*ms = ktime_ms_delta(ktime_get(), t);
	return !xg_gpu_powered(kbdev);
}

/* 7. One G3D power-domain off/on cycle with the GPU idle, then a re-check. */
static int xg_st_pd_cycle(struct exynos9810_gpu *g)
{
	struct kbase_device *kbdev = g->kbdev;
	u32 cfg, st, id, l2p, l2r;
	int ret_off, ret_on, ret;
	s64 off_us, on_us, ms;
	bool off_ok;
	ktime_t t;

	if (g->pd_mode == XG_PD_OFF) {
		pr_info(XG_ST_PFX "pd cycle: not run (G3D power control off)\n");
		return 0;
	}
	cancel_delayed_work_sync(&g->pd_off_work);
	mutex_lock(&g->pd_lock);
	if (!xg_pd_may_power_off(g)) {
		mutex_unlock(&g->pd_lock);
		pr_err(XG_ST_PFX "pd cycle: GPU not idle, not run: FAIL\n");
		return 1;
	}
	t = ktime_get();
	ret_off = xg_pd_off_locked(g, "selftest");
	off_us = xg_us_since(t);
	off_ok = !xg_pd_is_on(g, &cfg, &st);
	pr_info(XG_ST_PFX "pd cycle: off %d, %lld us, PMU G3D cfg %#x status %#x\n",
		ret_off, off_us, cfg, st);
	msleep(20);
	t = ktime_get();
	ret_on = xg_pd_on_locked(g, "selftest");
	on_us = xg_us_since(t);
	xg_pd_is_on(g, &cfg, &st);
	mutex_unlock(&g->pd_lock);
	pr_info(XG_ST_PFX "pd cycle: on %d, %lld us, PMU G3D cfg %#x status %#x, CAL pd status %d, pd on/off/errors %lu/%lu/%lu\n",
		ret_on, on_us, cfg, st, cal_pd_status(g->pd_cal_id),
		g->nr_pd_on, g->nr_pd_off, g->nr_pd_errors);
	if (ret_off || !off_ok || ret_on < 0) {
		pr_err(XG_ST_PFX "pd cycle: FAIL\n");
		return 1;
	}

	/* kbase power_on sees pd_state_lost and resets the GPU. */
	t = ktime_get();
	kbase_pm_context_active(kbdev);
	ret = kbase_pm_wait_for_desired_state(kbdev);
	id = kbase_reg_read(kbdev, GPU_CONTROL_REG(GPU_ID));
	l2p = kbase_reg_read(kbdev, GPU_CONTROL_REG(L2_PRESENT_LO));
	l2r = kbase_reg_read(kbdev, GPU_CONTROL_REG(L2_READY_LO));
	pr_info(XG_ST_PFX "pd cycle: kbase power up %d, %lld us, GPU_ID %#010x, L2 present %#x ready %#x, clock %u kHz\n",
		ret, xg_us_since(t), id, l2p, l2r, xg_dvfs_cur_khz(g));
	if (ret || id != g->gpu_id || !l2p || l2r != l2p)
		ret = ret ? ret : -EIO;
	if (!ret) {
		t = ktime_get();
		kbase_gpu_start_cache_clean(kbdev, GPU_COMMAND_CLEAN_INV_CACHES);
		ret = kbase_gpu_wait_cache_clean_timeout(kbdev, 100);
		pr_info(XG_ST_PFX "pd cycle: cache clean+invalidate %d, %lld us\n",
			ret, xg_us_since(t));
	}
	kbase_pm_context_idle(kbdev);
	if (!xg_st_wait_gpu_off(kbdev, &ms)) {
		pr_err(XG_ST_PFX "pd cycle: kbase GPU still on after %lld ms\n", ms);
		ret = ret ? ret : -EBUSY;
	}
	pr_info(XG_ST_PFX "pd cycle: %s\n", ret ? "FAIL" : "PASS");
	return ret ? 1 : 0;
}

static void xg_selftest_fn(struct work_struct *work)
{
	struct exynos9810_gpu *g = container_of(to_delayed_work(work),
						 struct exynos9810_gpu, selftest_work);
	struct kbase_device *kbdev = g->kbdev;
	u32 cfg, st, mux, id, l2p, l2r, shp_lo, shp_hi, shr_lo, shr_hi, tlp, tlr;
	u32 gst, irq, mmu_irq, as_active = 0;
	ktime_t t0 = ktime_get(), t;
	int fails = 0, ret, i;
	bool on;

	mux = xg_cmu_mux(g);
	on = xg_pd_is_on(g, &cfg, &st);
	pr_info(XG_ST_PFX "start: PMU G3D cfg %#x status %#x (CAL pd status %d), CMU_G3D mux %#x, clock %u kHz, kbase power on/off %lu/%lu\n",
		cfg, st, cal_pd_status(XG_PD_CAL_ID), mux, xg_dvfs_cur_khz(g),
		g->nr_power_on, g->nr_power_off);
	/* vdd_g3d = S2MPS18 BUCK6, B6CTRL (4.9 s2mps18-private.h); read only */
	if (exynos_acpm_mfd_is_ready()) {
		u8 b6 = 0;

		ret = exynos_acpm_read_reg(ACPM_MFD_TYPE_PMIC, XG_S2MPS18_B6CTRL, &b6);
		pr_info(XG_ST_PFX "S2MPS18 B6CTRL (vdd_g3d) read %d: %#04x\n", ret, b6);
	}
	if (!on) {
		pr_err(XG_ST_PFX "FAIL: G3D power domain off, GPU registers not read\n");
		return;
	}

	/* 2. power up through kbase */
	t = ktime_get();
	kbase_pm_context_active(kbdev);
	ret = kbase_pm_wait_for_desired_state(kbdev);
	id = kbase_reg_read(kbdev, GPU_CONTROL_REG(GPU_ID));
	l2p = kbase_reg_read(kbdev, GPU_CONTROL_REG(L2_PRESENT_LO));
	l2r = kbase_reg_read(kbdev, GPU_CONTROL_REG(L2_READY_LO));
	shp_lo = kbase_reg_read(kbdev, GPU_CONTROL_REG(SHADER_PRESENT_LO));
	shp_hi = kbase_reg_read(kbdev, GPU_CONTROL_REG(SHADER_PRESENT_HI));
	shr_lo = kbase_reg_read(kbdev, GPU_CONTROL_REG(SHADER_READY_LO));
	shr_hi = kbase_reg_read(kbdev, GPU_CONTROL_REG(SHADER_READY_HI));
	tlp = kbase_reg_read(kbdev, GPU_CONTROL_REG(TILER_PRESENT_LO));
	tlr = kbase_reg_read(kbdev, GPU_CONTROL_REG(TILER_READY_LO));
	gst = kbase_reg_read(kbdev, GPU_CONTROL_REG(GPU_STATUS));
	irq = kbase_reg_read(kbdev, GPU_CONTROL_REG(GPU_IRQ_RAWSTAT));
	pr_info(XG_ST_PFX "power up: wait %d, %lld us, GPU powered %d\n", ret, xg_us_since(t),
		xg_gpu_powered(kbdev));
	pr_info(XG_ST_PFX "GPU_ID %#010x (probe %#010x), L2 present %#x ready %#x, shader present %#010x%08x ready %#010x%08x (%u cores), tiler present %#x ready %#x, GPU_STATUS %#x, IRQ_RAWSTAT %#x\n",
		id, g->gpu_id, l2p, l2r, shp_hi, shp_lo, shr_hi, shr_lo,
		hweight32(shp_lo) + hweight32(shp_hi), tlp, tlr, gst, irq);
	if (ret || id != g->gpu_id || (id & GPU_ID2_PRODUCT_MODEL) != GPU_ID2_PRODUCT_THEX ||
	    !l2p || !(shp_lo | shp_hi) || !tlp || l2r != l2p) {
		pr_err(XG_ST_PFX "power up / ID check FAIL\n");
		fails++;
	}
	if (hweight32(shp_lo) + hweight32(shp_hi) != 18)
		pr_warn(XG_ST_PFX "expected 18 shader cores (Mali-G72 MP18)\n");
	if (shr_lo != shp_lo || shr_hi != shp_hi || tlr != tlp)
		pr_info(XG_ST_PFX "note: not all shader/tiler cores ready (kbase core policy)\n");

	/* 3. cache flush */
	t = ktime_get();
	kbase_gpu_start_cache_clean(kbdev, GPU_COMMAND_CLEAN_INV_CACHES);
	ret = kbase_gpu_wait_cache_clean_timeout(kbdev, 100);
	pr_info(XG_ST_PFX "cache clean+invalidate: %d, %lld us: %s\n", ret, xg_us_since(t),
		ret ? "FAIL" : "PASS");
	if (ret)
		fails++;

	/* 4. MMU address spaces (reads only) */
	for (i = 0; i < kbdev->nr_hw_address_spaces && i < 32; i++)
		if (kbase_reg_read(kbdev, MMU_AS_REG(i, AS_STATUS)) & AS_STATUS_AS_ACTIVE)
			as_active |= BIT(i);
	mmu_irq = kbase_reg_read(kbdev, MMU_REG(MMU_IRQ_RAWSTAT));
	pr_info(XG_ST_PFX "MMU: %d address spaces, active mask %#x, MMU_IRQ_RAWSTAT %#x (no AS programmed: needs a kbase context)\n",
		kbdev->nr_hw_address_spaces, as_active, mmu_irq);

	/* 5. DVFS */
	fails += xg_st_dvfs(g);

	pr_info(XG_ST_PFX "job submission: not run (no in-kernel job client)\n");

	/* 6. power down through kbase */
	kbase_pm_context_idle(kbdev);
	{
		s64 ms;

		on = !xg_st_wait_gpu_off(kbdev, &ms);
		xg_pd_is_on(g, &cfg, &st);
		pr_info(XG_ST_PFX "power down: kbase GPU %s after %lld ms, power on/off %lu/%lu, PMU G3D cfg %#x status %#x (G3D power control %s)\n",
			on ? "STILL ON" : "off", ms, g->nr_power_on, g->nr_power_off,
			cfg, st, xg_pd_mode_str(g));
	}
	if (on)
		fails++;

	/* 7. G3D power-domain cycle (samsung,g3d-power-control only) */
	if (!on)
		fails += xg_st_pd_cycle(g);

	pr_info(XG_ST_PFX "%s (%d failures, %lld ms)\n", fails ? "FAIL" : "PASS", fails,
		ktime_ms_delta(ktime_get(), t0));
}

void xg_selftest_schedule(struct exynos9810_gpu *g)
{
	s64 up;

	if (!of_property_read_bool(g->dev->of_node, "samsung,mali-selftest"))
		return;
	INIT_DELAYED_WORK(&g->selftest_work, xg_selftest_fn);
	up = ktime_to_ms(ktime_get_boottime());
	queue_delayed_work(system_long_wq, &g->selftest_work,
			   msecs_to_jiffies(up < XG_ST_AT_MS ? XG_ST_AT_MS - up : 0));
	xg_st_armed = true;
	pr_info(XG_ST_PFX "armed for %d s uptime\n", XG_ST_AT_MS / 1000);
}

void xg_selftest_cancel(struct exynos9810_gpu *g)
{
	if (xg_st_armed) {
		cancel_delayed_work_sync(&g->selftest_work);
		xg_st_armed = false;
	}
}
