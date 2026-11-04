/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Exynos9810 (crownlte) platform layer for the Samsung r38p1-01bet0 kbase.
 *
 * Private state shared by the platform, DVFS and self-test files.
 */
#ifndef _MALI_EXYNOS9810_H_
#define _MALI_EXYNOS9810_H_

#include <linux/kobject.h>
#include <linux/mutex.h>
#include <linux/pm_qos.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

struct device;
struct devfreq;
struct kbase_device;
struct regmap;
struct thermal_cooling_device;

#define XG_PFX			"mali exynos9810: "
#define XG_ST_PFX		"mali selftest: "

/* PMU (0x14060000), 4.9 flexpmu_cal_local_exynos9810.h embedded_g3d_* */
#define XG_PMU_G3D_CONFIGURATION	0x4040
#define XG_PMU_G3D_STATUS		0x4044
#define XG_PMU_G3D_LOCAL_PWR_MASK	0xf	/* 4.9 gpu_pmu_status_local_pwr_mask */

/* CMU_G3D (0x17400000): PLL_CON0_MUX_CLKCMU_EMBEDDED_G3D_USER, bit 4 MUX_SEL */
#define XG_CMU_G3D_MUX_USER		0x0100
#define XG_CMU_G3D_MUX_USER_SEL		BIT(4)
#define XG_CMU_G3D_MAP_SIZE		0x1000

#define XG_MAX_LEVELS			16
#define XG_TABLE_COLS			5	/* kHz, min %, max %, stay, MIF kHz */

struct xg_level {
	unsigned int khz;
	unsigned int min_util;
	unsigned int max_util;
	unsigned int staycount;
	unsigned int mif_khz;
};

struct exynos9810_gpu {
	struct kbase_device *kbdev;
	struct device *dev;
	struct regmap *pmu;
	void __iomem *cmu;	/* CMU_G3D, read only */
	u32 cal_id;
	u32 gpu_id;
	unsigned int boot_khz;	/* CMU readback at probe */

	/* DVFS. Step 0 is the highest level (4.9 gpu_dvfs_table order). */
	struct xg_level levels[XG_MAX_LEVELS];
	unsigned int nr_levels;
	bool dvfs_enabled;

	spinlock_t lock;		/* governor state below */
	unsigned int step;		/* governor target */
	unsigned int down_req;
	unsigned int hs_step;		/* interactive highspeed level */
	unsigned int hs_load;
	unsigned int hs_delay;
	unsigned int delay_count;
	unsigned int thermal_cap;	/* lowest step index thermal allows */
	unsigned int user_max_step;	/* sysfs gpu_max_clock */
	unsigned int user_min_step;	/* sysfs gpu_min_clock */
	unsigned int last_util;
	bool hold;			/* self-test owns the clock */
	bool gpu_on;			/* between power_on and power_off callbacks */

	struct mutex set_lock;		/* serializes CAL rate changes */
	unsigned int cur_step;		/* applied level (set_lock) */
	unsigned long nr_sets;
	unsigned long nr_set_errors;
	struct workqueue_struct *wq;
	struct work_struct set_work;
	struct delayed_work idle_work;

	struct thermal_cooling_device *cdev;

	struct devfreq *mif;
	struct dev_pm_qos_request mif_req;
	bool mif_lookup_done;

	struct kobject *kobj;
	struct delayed_work selftest_work;
	unsigned long nr_power_on;
	unsigned long nr_power_off;
};

extern struct exynos9810_gpu *exynos9810_gpu;

bool xg_pd_is_on(struct exynos9810_gpu *g, u32 *cfg, u32 *status);
u32 xg_cmu_mux(struct exynos9810_gpu *g);

int xg_dvfs_init(struct exynos9810_gpu *g);
int xg_dvfs_late_init(struct exynos9810_gpu *g);
void xg_dvfs_term(struct exynos9810_gpu *g);
void xg_dvfs_power_on(struct exynos9810_gpu *g);
void xg_dvfs_power_off(struct exynos9810_gpu *g);
int xg_dvfs_apply(struct exynos9810_gpu *g, unsigned int step, unsigned long *readback);
int xg_dvfs_khz_to_step(struct exynos9810_gpu *g, unsigned int khz);
unsigned int xg_dvfs_cur_khz(struct exynos9810_gpu *g);
void xg_dvfs_hold(struct exynos9810_gpu *g, bool hold);
int xg_sysfs_init(struct exynos9810_gpu *g);
void xg_sysfs_term(struct exynos9810_gpu *g);

void xg_selftest_schedule(struct exynos9810_gpu *g);
void xg_selftest_cancel(struct exynos9810_gpu *g);

#endif /* _MALI_EXYNOS9810_H_ */
