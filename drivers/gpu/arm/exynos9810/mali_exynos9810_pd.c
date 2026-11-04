// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos9810 (crownlte) G3D power-domain control for the r38p1 kbase
 * platform layer. Off unless the gpu node has
 *
 *   samsung,g3d-power-control = "boot-on";   power the domain on at probe if
 *                                            SBoot left it off; never off
 *                                            (except the self-test cycle)
 *   samsung,g3d-power-control = "runtime";   + 4.9 runtime PM: domain off
 *                                            samsung,g3d-off-delay-ms after
 *                                            kbase powers the GPU off
 *
 * and the PMIC transport allows the G3D rail request (S2MPS18 node
 * samsung,g3d-rail-update). Without the latter cal_pd_control() fails with
 * -EPERM before any PMU write.
 *
 * The sequence is the unmodified 4.9 one, cal_pd_control(0xb1380008):
 *  on:  ACPM FUNC_UPDATE type 3 reg 0x2a val 0xc0 mask 0xc0 (APM firmware:
 *       PMU PWR_EMBEDDED_G3D 0x4678 = 1, wait 0x467c bit 0, then S2MPS18
 *       B6CTRL bits 3:2 = 0b10), PMUCAL embedded_g3d_on (CMU_G3D PLL/mux,
 *       EMBEDDED_G3D_CONFIGURATION = 0xf, wait STATUS), DTZPC restore SMC
 *       0x82000410(1, 0x17410204).
 *  off: DTZPC save SMC 0x82000410(0, 0x17410204), PMUCAL embedded_g3d_off,
 *       ACPM update val 0x00 (B6CTRL bits 3:2 = 0, PWR_EMBEDDED_G3D = 0).
 * 4.9 sets need_smc from the pd-embedded_g3d DT node (exynos-pd.c); nothing
 * does that in 6.12, so this file calls cal_pd_set_smc_id() itself.
 *
 * The APM firmware reports success even when its PWR_EMBEDDED_G3D status
 * poll times out, so every transition is verified here through the PMU
 * (G3D STATUS, PWR status bit 0) and the CMU_G3D user mux. A failed
 * transition latches pd_failed: no further power-off is attempted.
 */
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include <mali_kbase.h>
#include <soc/samsung/acpm_mfd.h>
#include <soc/samsung/cal-if.h>

#include "mali_exynos9810.h"

/* 4.9 DT pd-embedded_g3d@14064040: cal_id, need_smc */
#define XG_PD_CAL_ID_49		0xb1380008
#define XG_PD_NEED_SMC_49	0x17410204
#define XG_PD_OFF_DELAY_MS	150	/* > XG_IDLE_DROP_MS, see below */
#define XG_PD_VERBOSE		8	/* transitions logged at info level */

static const char *xg_pd_mode_name(enum xg_pd_mode m)
{
	switch (m) {
	case XG_PD_BOOT_ON:
		return "boot-on";
	case XG_PD_RUNTIME:
		return "runtime";
	default:
		return "off";
	}
}

struct xg_pd_snap {
	u32 cfg, st, pwr_cfg, pwr_st, mux;
	u8 b6;
	int b6_ret;
	bool on;
};

static void xg_pd_snapshot(struct exynos9810_gpu *g, struct xg_pd_snap *s)
{
	unsigned int v;

	memset(s, 0, sizeof(*s));
	s->on = xg_pd_is_on(g, &s->cfg, &s->st);
	if (!regmap_read(g->pmu, XG_PMU_G3D_PWR_CONFIGURATION, &v))
		s->pwr_cfg = v;
	if (!regmap_read(g->pmu, XG_PMU_G3D_PWR_STATUS, &v))
		s->pwr_st = v;
	s->mux = xg_cmu_mux(g);
	s->b6_ret = -ENODATA;
}

/* One ACPM PMIC read (S2MPS18 B6CTRL); only for transitions that are logged. */
static void xg_pd_snapshot_b6(struct xg_pd_snap *s)
{
	s->b6_ret = exynos_acpm_mfd_is_ready() ?
		exynos_acpm_read_reg(ACPM_MFD_TYPE_PMIC, XG_S2MPS18_B6CTRL, &s->b6) :
		-ENODEV;
}

static void xg_pd_log(struct exynos9810_gpu *g, bool err, const char *what,
		      const char *why, int ret, s64 us, struct xg_pd_snap *s)
{
	if (err || g->nr_pd_on + g->nr_pd_off <= XG_PD_VERBOSE)
		xg_pd_snapshot_b6(s);
	if (err)
		dev_err(g->dev, XG_PFX "G3D domain %s (%s): FAIL ret %d, %lld us, PMU cfg %#x status %#x, PWR cfg %#x status %#x, CMU mux %#x, B6CTRL %#04x (read %d)\n",
			what, why, ret, us, s->cfg, s->st, s->pwr_cfg, s->pwr_st,
			s->mux, s->b6, s->b6_ret);
	else if (g->nr_pd_on + g->nr_pd_off <= XG_PD_VERBOSE)
		dev_info(g->dev, XG_PFX "G3D domain %s (%s): %lld us, PMU cfg %#x status %#x, PWR cfg %#x status %#x, CMU mux %#x, B6CTRL %#04x (read %d)\n",
			 what, why, us, s->cfg, s->st, s->pwr_cfg, s->pwr_st,
			 s->mux, s->b6, s->b6_ret);
	else
		dev_dbg(g->dev, XG_PFX "G3D domain %s (%s): %lld us\n", what, why, us);
}

/*
 * pd_lock held. Returns 1 if the domain was switched on (GPU state lost),
 * 0 if it was already on, negative on failure.
 */
int xg_pd_on_locked(struct exynos9810_gpu *g, const char *why)
{
	struct xg_pd_snap s;
	ktime_t t;
	int ret;

	lockdep_assert_held(&g->pd_lock);
	if (xg_pd_is_on(g, NULL, NULL))
		return 0;
	if (g->pd_mode == XG_PD_OFF)
		return -EPERM;

	mutex_lock(&g->set_lock);
	t = ktime_get();
	ret = cal_pd_control(g->pd_cal_id, 1);
	/*
	 * CMU_G3D is outside the switched domain (embedded_g3d_on writes it
	 * before CONFIGURATION), so PLL_G3D keeps its rate; check anyway and
	 * let the governor re-apply its level if the readback differs.
	 */
	if (!ret && g->nr_levels) {
		unsigned long rb = cal_dfs_get_rate(g->cal_id);
		int idx = xg_dvfs_khz_to_step(g, rb);

		if (idx < 0 || (unsigned int)idx != g->cur_step) {
			dev_warn(g->dev, XG_PFX "G3D clock %lu kHz after power on, expected level %u\n",
				 rb, g->cur_step);
			WRITE_ONCE(g->cur_step, idx >= 0 ? idx : g->nr_levels);
		}
	}
	mutex_unlock(&g->set_lock);
	xg_pd_snapshot(g, &s);
	g->nr_pd_on++;
	if (ret || !s.on || !(s.pwr_st & XG_PMU_G3D_PWR_STATUS_ON) ||
	    !(s.mux & XG_CMU_G3D_MUX_USER_SEL)) {
		g->nr_pd_errors++;
		g->pd_failed = true;
		xg_pd_log(g, true, "on", why, ret, ktime_us_delta(ktime_get(), t), &s);
		return ret ? ret : -EIO;
	}
	g->pd_state_lost = true;
	xg_pd_log(g, false, "on", why, 0, ktime_us_delta(ktime_get(), t), &s);
	return 1;
}

/* pd_lock held, kbase GPU powered off. */
int xg_pd_off_locked(struct exynos9810_gpu *g, const char *why)
{
	struct xg_pd_snap s;
	ktime_t t;
	int ret;

	lockdep_assert_held(&g->pd_lock);
	if (g->pd_mode == XG_PD_OFF || g->pd_failed)
		return -EPERM;
	if (!xg_pd_is_on(g, NULL, NULL))
		return 0;

	mutex_lock(&g->set_lock);
	t = ktime_get();
	ret = cal_pd_control(g->pd_cal_id, 0);
	mutex_unlock(&g->set_lock);
	xg_pd_snapshot(g, &s);
	g->nr_pd_off++;
	if (ret || (s.st & XG_PMU_G3D_LOCAL_PWR_MASK) ||
	    (s.pwr_st & XG_PMU_G3D_PWR_STATUS_ON)) {
		g->nr_pd_errors++;
		g->pd_failed = true;
		xg_pd_log(g, true, "off", why, ret, ktime_us_delta(ktime_get(), t), &s);
		return ret ? ret : -EIO;
	}
	g->pd_state_lost = true;
	xg_pd_log(g, false, "off", why, 0, ktime_us_delta(ktime_get(), t), &s);
	return 0;
}

static bool xg_kbase_gpu_powered(struct exynos9810_gpu *g)
{
	unsigned long flags;
	bool on;

	spin_lock_irqsave(&g->kbdev->hwaccess_lock, flags);
	on = g->kbdev->pm.backend.gpu_powered;
	spin_unlock_irqrestore(&g->kbdev->hwaccess_lock, flags);
	return on;
}

/* pd_lock held: may the domain be switched off now? */
bool xg_pd_may_power_off(struct exynos9810_gpu *g)
{
	unsigned long flags;
	bool busy;

	lockdep_assert_held(&g->pd_lock);
	spin_lock_irqsave(&g->lock, flags);
	busy = g->gpu_on || g->hold;
	spin_unlock_irqrestore(&g->lock, flags);
	return !busy && !xg_kbase_gpu_powered(g);
}

static void xg_pd_off_work_fn(struct work_struct *work)
{
	struct exynos9810_gpu *g = container_of(to_delayed_work(work),
						 struct exynos9810_gpu, pd_off_work);

	mutex_lock(&g->pd_lock);
	if (g->pd_mode == XG_PD_RUNTIME && xg_pd_may_power_off(g))
		xg_pd_off_locked(g, "kbase idle");
	mutex_unlock(&g->pd_lock);
}

/*
 * kbase power_on callback. Returns true if the GPU lost its state since the
 * last power_on (the domain was cycled) and kbase must reset it.
 */
bool xg_pd_power_on(struct exynos9810_gpu *g)
{
	bool lost;
	int ret;

	if (g->pd_mode == XG_PD_OFF)
		return false;
	cancel_delayed_work(&g->pd_off_work);
	mutex_lock(&g->pd_lock);
	xg_dvfs_power_on(g);	/* gpu_on = true: the off work backs off */
	ret = xg_pd_on_locked(g, "kbase power on");
	if (ret < 0) {
		/* One retry; a second failure leaves kbase on a dead domain. */
		ret = xg_pd_on_locked(g, "kbase power on, retry");
		if (ret < 0)
			dev_crit(g->dev, XG_PFX "G3D domain did not power on (%d); GPU register access will fault\n",
				 ret);
	}
	lost = g->pd_state_lost;
	g->pd_state_lost = false;
	mutex_unlock(&g->pd_lock);
	return lost;
}

/* kbase power_off callback, after xg_dvfs_power_off(). */
void xg_pd_power_off(struct exynos9810_gpu *g)
{
	if (g->pd_mode != XG_PD_RUNTIME || g->pd_failed)
		return;
	mod_delayed_work(system_wq, &g->pd_off_work,
			 msecs_to_jiffies(g->pd_off_delay_ms));
}

/*
 * Probe: parse the DT gate, set the 4.9 DTZPC SMC id and power the domain
 * on if SBoot left it off. Called with the PMU regmap and CMU_G3D mapped.
 */
int xg_pd_init(struct exynos9810_gpu *g)
{
	struct device_node *np = g->dev->of_node;
	struct xg_pd_snap s;
	const char *mode;
	u32 v;
	int ret;

	mutex_init(&g->pd_lock);
	INIT_DELAYED_WORK(&g->pd_off_work, xg_pd_off_work_fn);
	g->pd_mode = XG_PD_OFF;
	g->pd_cal_id = XG_PD_CAL_ID_49;
	g->pd_need_smc = XG_PD_NEED_SMC_49;
	g->pd_off_delay_ms = XG_PD_OFF_DELAY_MS;

	xg_pd_snapshot(g, &s);
	xg_pd_snapshot_b6(&s);
	dev_info(g->dev, XG_PFX "G3D at probe: PMU cfg %#x status %#x, PWR cfg %#x status %#x, CMU mux %#x, B6CTRL %#04x (read %d)\n",
		 s.cfg, s.st, s.pwr_cfg, s.pwr_st, s.mux, s.b6, s.b6_ret);

	if (of_property_read_string(np, "samsung,g3d-power-control", &mode))
		return 0;
	if (!strcmp(mode, "boot-on")) {
		g->pd_mode = XG_PD_BOOT_ON;
	} else if (!strcmp(mode, "runtime")) {
		g->pd_mode = XG_PD_RUNTIME;
	} else {
		dev_err(g->dev, XG_PFX "samsung,g3d-power-control \"%s\" unknown; G3D domain not switched\n",
			mode);
		return 0;
	}

	/* Only the 4.9 values: a different domain or SMC target is refused. */
	if (!of_property_read_u32(np, "samsung,g3d-pd-cal-id", &v) && v != XG_PD_CAL_ID_49)
		goto bad_dt;
	if (!of_property_read_u32(np, "samsung,g3d-need-smc", &v) && v != XG_PD_NEED_SMC_49)
		goto bad_dt;
	if (!of_property_read_u32(np, "samsung,g3d-off-delay-ms", &v))
		g->pd_off_delay_ms = clamp_t(u32, v, 20, 10000);

	ret = cal_pd_status(g->pd_cal_id);
	if (ret < 0 || !!ret != s.on) {
		dev_err(g->dev, XG_PFX "CAL pd status %d disagrees with PMU (%d); G3D domain not switched\n",
			ret, s.on);
		g->pd_mode = XG_PD_OFF;
		return 0;
	}
	ret = cal_pd_set_smc_id(g->pd_cal_id, g->pd_need_smc);
	if (ret) {
		dev_err(g->dev, XG_PFX "cal_pd_set_smc_id: %d; G3D domain not switched\n", ret);
		g->pd_mode = XG_PD_OFF;
		return 0;
	}
	dev_info(g->dev, XG_PFX "G3D power control \"%s\": cal_id %#x, DTZPC SMC target %#x, off delay %u ms\n",
		 xg_pd_mode_name(g->pd_mode), g->pd_cal_id, g->pd_need_smc,
		 g->pd_mode == XG_PD_RUNTIME ? g->pd_off_delay_ms : 0);

	if (s.on)
		return 0;
	mutex_lock(&g->pd_lock);
	ret = xg_pd_on_locked(g, "probe");
	g->pd_state_lost = false;	/* kbase resets the GPU at init anyway */
	mutex_unlock(&g->pd_lock);
	return ret < 0 ? ret : 0;

bad_dt:
	dev_err(g->dev, XG_PFX "G3D pd cal-id/need-smc differ from 4.9 (%#x/%#x); G3D domain not switched\n",
		XG_PD_CAL_ID_49, XG_PD_NEED_SMC_49);
	g->pd_mode = XG_PD_OFF;
	return 0;
}

void xg_pd_term(struct exynos9810_gpu *g)
{
	if (g->pd_mode == XG_PD_OFF)
		return;
	g->pd_mode = XG_PD_OFF;
	cancel_delayed_work_sync(&g->pd_off_work);
}

const char *xg_pd_mode_str(struct exynos9810_gpu *g)
{
	return xg_pd_mode_name(g->pd_mode);
}
