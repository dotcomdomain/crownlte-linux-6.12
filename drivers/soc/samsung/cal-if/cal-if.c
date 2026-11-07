#include <linux/module.h>
#include <linux/exynos-ss.h>
#include <soc/samsung/ect_parser.h>
#include <soc/samsung/cal-if.h>
#include <soc/samsung/acpm_mfd.h>

#include "pwrcal-env.h"
#include "pwrcal-rae.h"
#include "cmucal.h"
#include "ra.h"
#include "acpm_dvfs.h"
#include "fvmap.h"
#include "asv.h"

#include "pmucal_system.h"
#include "pmucal_local.h"
#include "pmucal_cpu.h"
#include "pmucal_cp.h"
#include "pmucal_rae.h"
#include "pmucal_powermode.h"

static DEFINE_SPINLOCK(pmucal_cpu_lock);
static bool cal_initialized;

bool cal_if_is_initialized(void)
{
	return smp_load_acquire(&cal_initialized);
}
EXPORT_SYMBOL_GPL(cal_if_is_initialized);

static unsigned int cal_hw_clock_enabled(unsigned int id)
{
	struct cmucal_clk *clk = cmucal_get_node(id);
	if (!clk)
		return 0;
	if (IS_GATE(id))
		return !!ra_get_value(id);
	if (IS_QCH(id) && clk->offset)
		return !!get_value(clk->offset, clk->shift, clk->width);
	if (IS_PLL(id) && clk->enable)
		return !!get_value(clk->enable, clk->e_shift, clk->e_width);
	return 0;
}
unsigned int cal_clk_is_enabled(unsigned int id)
{
	struct vclk *vclk;
	unsigned int i;
	if (!IS_GATE_VCLK(id))
		return cal_hw_clock_enabled(id);
	vclk = cmucal_get_node(id);
	if (!vclk || !vclk->num_list)
		return 0;
	for (i = 0; i < vclk->num_list; i++)
		if (!cal_hw_clock_enabled(vclk->list[i]))
			return 0;
	return 1;
}

unsigned long cal_dfs_get_max_freq(unsigned int id)
{
	return vclk_get_max_freq(id);
}

unsigned long cal_dfs_get_min_freq(unsigned int id)
{
	return vclk_get_min_freq(id);
}

unsigned int cal_dfs_get_lv_num(unsigned int id)
{
	return vclk_get_lv_num(id);
}

int cal_dfs_get_bigturbo_max_freq(unsigned int *table)
{
	return vclk_get_bigturbo_table(table);
}

bool cal_dfs_is_ready(unsigned int id)
{
	struct vclk *vclk;
	unsigned int voltage[48], i;
	if (!cal_if_is_initialized() || !IS_ACPM_VCLK(id) ||
	    !exynos_acpm_dvfs_is_ready())
		return false;
	vclk = cmucal_get_node(id);
	if (!vclk || !vclk->lut || !vclk->num_rates ||
	    vclk->num_rates > ARRAY_SIZE(voltage) ||
	    !vclk->max_freq || vclk->max_freq == UINT_MAX ||
	    vclk->min_freq == UINT_MAX || vclk->min_freq > vclk->max_freq ||
	    fvmap_get_voltage_table(id, voltage) != vclk->num_rates)
		return false;
	for (i = 0; i < vclk->num_rates; i++)
		if (!voltage[i])
			return false;
	return true;
}
EXPORT_SYMBOL_GPL(cal_dfs_is_ready);

int cal_dfs_set_rate(unsigned int id, unsigned long rate)
{
	struct vclk *vclk;
	int ret;

	if (IS_ACPM_VCLK(id)) {
		unsigned int voltage[48];
		unsigned int i;
		vclk = cmucal_get_node(id);
		if (!vclk || !vclk->lut || !vclk->num_rates ||
		    vclk->num_rates > ARRAY_SIZE(voltage))
			return -ENODEV;
		if (!vclk->max_freq || vclk->max_freq == UINT_MAX ||
		    vclk->min_freq == UINT_MAX || vclk->min_freq > vclk->max_freq)
			return -EINVAL;
		if (rate < vclk->min_freq || rate > vclk->max_freq)
			return -ERANGE;
		for (i = 0; i < vclk->num_rates; i++)
			if (vclk->lut[i].rate == rate)
				break;
		if (i == vclk->num_rates)
			return -EINVAL;
		if (fvmap_get_voltage_table(id, voltage) != vclk->num_rates ||
		    !voltage[i])
			return -ENODEV;
		ret = exynos_acpm_set_rate(GET_IDX(id), rate);
		if (!ret) {
			vclk = cmucal_get_node(id);
			if (vclk)
				vclk->vrate = (unsigned int)rate;
		}
	} else {
		ret = vclk_set_rate(id, rate);
	}

	return ret;
}

int cal_dfs_set_rate_switch(unsigned int id, unsigned long switch_rate)
{
	int ret = 0;

	ret = vclk_set_rate_switch(id, switch_rate);

	return ret;
}

int cal_dfs_set_rate_restore(unsigned int id, unsigned long switch_rate)
{
	int ret = 0;

	ret = vclk_set_rate_restore(id, switch_rate);

	return ret;
}

unsigned long cal_dfs_cached_get_rate(unsigned int id)
{
	int ret;

	ret = vclk_get_rate(id);

	return ret;
}

unsigned long cal_dfs_get_rate(unsigned int id)
{
	int ret;

	ret = vclk_recalc_rate(id);

	return ret;
}

int cal_dfs_get_rate_table(unsigned int id, unsigned long *table)
{
	int ret;

	ret = vclk_get_rate_table(id, table);

	return ret;
}

int cal_clk_setrate(unsigned int id, unsigned long rate)
{
	int ret = -EINVAL;

	ret = vclk_set_rate(id, rate);

	return ret;
}

unsigned long cal_clk_getrate(unsigned int id)
{
	int ret = 0;

	ret = vclk_recalc_rate(id);

	return ret;
}

int cal_clk_enable(unsigned int id)
{
	int ret = 0;

	ret = vclk_set_enable(id);

	return ret;
}

int cal_clk_disable(unsigned int id)
{
	int ret = 0;

	ret = vclk_set_disable(id);

	return ret;
}

int cal_qch_init(unsigned int id, unsigned int use_qch)
{
	int ret = 0;

	ret = ra_set_qch(id, use_qch, 0, 0);

	return ret;
}

unsigned int cal_dfs_get_boot_freq(unsigned int id)
{
	return vclk_get_boot_freq(id);
}

unsigned int cal_dfs_get_resume_freq(unsigned int id)
{
	return vclk_get_resume_freq(id);
}

int cal_pd_control(unsigned int id, int on)
{
	unsigned int index;
	int ret;

	if (!cal_if_is_initialized())
		return -ENODEV;

	if ((id & 0xFFFF0000) != BLKPWR_MAGIC)
		return -1;

	index = id & 0x0000FFFF;

	if (on) {
		if ((id & 0xffff) == 0x8) {
			ret = exynos_acpm_update_reg(0x3, 0x2a, 0x3 << 6, 0x3 << 6);
			if (ret)
				return ret;
		}
		ret = pmucal_local_enable(index);
	} else {
		ret = pmucal_local_disable(index);
		if (!ret && (id & 0xffff) == 0x8)
			ret = exynos_acpm_update_reg(0x3, 0x2a, 0x0 << 6, 0x3 << 6);
	}

	return ret;
}

int cal_pd_status(unsigned int id)
{
	unsigned int index;

	if (!cal_if_is_initialized())
		return -ENODEV;

	if ((id & 0xFFFF0000) != BLKPWR_MAGIC)
		return -1;

	index = id & 0x0000FFFF;

	return pmucal_local_is_enabled(index);
}

int cal_pd_set_smc_id(unsigned int id, int need_smc)
{
	unsigned int index;

	if (!cal_if_is_initialized())
		return -ENODEV;

	if (need_smc && ((id & 0xFFFF0000) != BLKPWR_MAGIC))
		return -1;

	index = id & 0x0000FFFF;

	pmucal_local_set_smc_id(index, need_smc);

	return 0;
}

int cal_pm_enter(int mode)
{
	return pmucal_system_enter(mode);
}

int cal_pm_exit(int mode)
{
	return pmucal_system_exit(mode);
}

int cal_pm_earlywakeup(int mode)
{
	return pmucal_system_earlywakeup(mode);
}

/* crownlte suspend S0: cal_pm_enter() minus the APM GRP2 hand-off. */
int cal_pm_enter_unarmed(int mode, unsigned int *skipped)
{
	return pmucal_system_enter_unarmed(mode, skipped);
}

int cal_cpu_enable(unsigned int cpu)
{
	int ret;

	spin_lock(&pmucal_cpu_lock);
	ret = pmucal_cpu_enable(cpu);
	spin_unlock(&pmucal_cpu_lock);

	return ret;
}

int cal_cpu_disable(unsigned int cpu)
{
	int ret;

	spin_lock(&pmucal_cpu_lock);
	ret = pmucal_cpu_disable(cpu);
	spin_unlock(&pmucal_cpu_lock);

	return ret;
}

/* Diagnostics only: true while a CPU is inside a cal_cpu_* / cal_cluster_* call. */
bool cal_cpu_lock_is_held(void)
{
	return spin_is_locked(&pmucal_cpu_lock);
}

int cal_cpu_status(unsigned int cpu)
{
	int ret;

	spin_lock(&pmucal_cpu_lock);
	ret = pmucal_cpu_is_enabled(cpu);
	spin_unlock(&pmucal_cpu_lock);

	return ret;
}

int cal_cluster_enable(unsigned int cluster)
{
	int ret;

	spin_lock(&pmucal_cpu_lock);
	ret = pmucal_cpu_cluster_enable(cluster);
	spin_unlock(&pmucal_cpu_lock);

	return ret;
}

int cal_cluster_disable(unsigned int cluster)
{
	int ret;

	spin_lock(&pmucal_cpu_lock);
	ret = pmucal_cpu_cluster_disable(cluster);
	spin_unlock(&pmucal_cpu_lock);

	return ret;
}

int cal_cluster_status(unsigned int cluster)
{
	int ret;

	spin_lock(&pmucal_cpu_lock);
	ret = pmucal_cpu_cluster_is_enabled(cluster);
	spin_unlock(&pmucal_cpu_lock);

	return ret;
}

int cal_dfs_get_asv_table(unsigned int id, unsigned int *table)
{
	return fvmap_get_voltage_table(id, table);
}

void cal_dfs_set_volt_margin(unsigned int id, int volt)
{
	if (IS_ACPM_VCLK(id))
		exynos_acpm_set_volt_margin(id, volt);
}

int cal_dfs_get_rate_asv_table(unsigned int id,
					struct dvfs_rate_volt *table)
{
	unsigned long rate[48];
	unsigned int volt[48];
	int num_of_entry;
	int idx;

	if (cal_dfs_get_lv_num(id) > ARRAY_SIZE(rate))
		return 0;
	num_of_entry = cal_dfs_get_rate_table(id, rate);
	if (num_of_entry == 0)
		return 0;

	if (num_of_entry != cal_dfs_get_asv_table(id, volt))
		return 0;

	if (!table)
		return 0;
	for (idx = 0; idx < num_of_entry; idx++)
		if (!volt[idx])
			return 0;
	for (idx = 0; idx < num_of_entry; idx++) {
		table[idx].rate = rate[idx];
		table[idx].volt = volt[idx];
	}

	return num_of_entry;
}

int cal_asv_get_ids_info(unsigned int id)
{
	return asv_get_ids_info(id);
}

int cal_asv_get_grp(unsigned int id)
{
	return asv_get_grp(id);
}

int cal_asv_get_tablever(void)
{
	return asv_get_table_ver();
}

#ifdef CONFIG_CP_PMUCAL
int cal_cp_init(void)
{
	return pmucal_cp_init();
}

int cal_cp_status(void)
{
	return pmucal_cp_status();
}

int cal_cp_reset_assert(void)
{
	return pmucal_cp_reset_assert();
}

int cal_cp_reset_release(void)
{
	return pmucal_cp_reset_release();
}

void cal_cp_active_clear(void)
{
	pmucal_cp_active_clear();
}

void cal_cp_reset_req_clear(void)
{
	pmucal_cp_reset_req_clear();
}

void cal_cp_enable_dump_pc_no_pg(void)
{
	pmucal_cp_enable_dump_pc_no_pg();
}

void cal_cp_disable_dump_pc_no_pg(void)
{
	pmucal_cp_disable_dump_pc_no_pg();
}
#endif

int __init cal_if_init(void *dev)
{
	int ret;

	if (cal_if_is_initialized())
		return 0;

	ret = ect_parse_binary_header();
	if (ret)
		return ret;

	ret = vclk_initialize();
	if (ret)
		return ret;

	if (cal_data_init)
		cal_data_init();

	ret = pmucal_rae_init();
	if (ret < 0)
		return ret;

	ret = pmucal_system_init();
	if (ret < 0)
		return ret;

	ret = pmucal_local_init();
	if (ret < 0)
		return ret;

	ret = pmucal_cpu_init();
	if (ret < 0)
		return ret;

	ret = pmucal_cpuinform_init();
	if (ret < 0)
		return ret;

#ifdef CONFIG_CP_PMUCAL
	ret = pmucal_cp_initialize();
	if (ret < 0)
		return ret;
#endif

	exynos_acpm_set_device(dev);

	smp_store_release(&cal_initialized, true);

	return 0;
}
