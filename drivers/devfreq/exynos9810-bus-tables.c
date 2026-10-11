// SPDX-License-Identifier: GPL-2.0-only
/* Real CAL/ASV and ECT bus tables plus the unchanged crownlte FVP protocol.
 * This is a table/firmware component, not a registered devfreq provider.
 * The provider still needs real BTS, thermal and dependency transitions.
 */
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/string.h>
#include <soc/samsung/acpm_ipc_ctrl.h>
#include <soc/samsung/cal-if.h>
#include <soc/samsung/ect_parser.h>
#include "exynos9810-bus-tables.h"

int exynos9810_bus_build_table(unsigned int cal_id, unsigned int min_khz,
		unsigned int max_khz, struct exynos9810_bus_table *table)
{
	struct dvfs_rate_volt rates[EXYNOS9810_BUS_LEVELS];
	struct ect_dvfs_domain *dvfs;
	struct ect_minlock_domain *minlock = NULL;
	unsigned int pairs[2 * EXYNOS9810_BUS_LEVELS];
	unsigned long cal_min, cal_max;
	const char *name;
	void *block;
	unsigned int i, j, count = 0, pair_count = 0;
	int levels, ret;

	if (!table || !min_khz || min_khz > max_khz || max_khz > INT_MAX)
		return -EINVAL;
	memset(table, 0, sizeof(*table));
	switch (cal_id) {
	case EXYNOS9810_MIF_CAL_ID:
		name = "dvfs_mif";
		break;
	case EXYNOS9810_INT_CAL_ID:
		name = "dvfs_int";
		break;
	case EXYNOS9810_FSYS0_CAL_ID:
		name = "dvfs_fsys0";
		break;
	default:
		return -EINVAL;
	}
	if (!cal_dfs_is_ready(cal_id))
		return -EPROBE_DEFER;
	cal_min = cal_dfs_get_min_freq(cal_id);
	cal_max = cal_dfs_get_max_freq(cal_id);
	if (!cal_min || cal_min > cal_max || cal_max > INT_MAX)
		return -ERANGE;
	min_khz = max_t(unsigned int, min_khz, cal_min);
	max_khz = min_t(unsigned int, max_khz, cal_max);
	if (min_khz > max_khz)
		return -ERANGE;
	levels = cal_dfs_get_rate_asv_table(cal_id, rates);
	if (levels <= 0 || levels > EXYNOS9810_BUS_LEVELS)
		return -EINVAL;
	block = ect_get_block(BLOCK_DVFS);
	dvfs = block ? ect_dvfs_get_domain(block, (char *)name) : NULL;
	if (!dvfs || !dvfs->list_level || dvfs->num_of_level <= 0 ||
		dvfs->num_of_level > EXYNOS9810_BUS_LEVELS)
		return -ENODEV;
	for (i = 0; i < dvfs->num_of_level; i++) {
		if (!dvfs->list_level[i].level ||
			(i && dvfs->list_level[i].level >= dvfs->list_level[i - 1].level))
			return -EINVAL;
	}
	if (cal_id != EXYNOS9810_FSYS0_CAL_ID) {
		block = ect_get_block(BLOCK_MINLOCK);
		minlock = block ? ect_minlock_get_domain(block, (char *)name) : NULL;
		if (!minlock || !minlock->level || minlock->num_of_level <= 0 ||
			minlock->num_of_level > EXYNOS9810_BUS_LEVELS)
			return -ENODEV;
		pair_count = minlock->num_of_level;
		for (i = 0; i < pair_count; i++) {
			pairs[2 * i] = minlock->level[i].main_frequencies;
			pairs[2 * i + 1] = minlock->level[i].sub_frequencies;
		}
	}
	for (i = 0; i < levels; i++) {
		struct exynos9810_bus_level *level;
		unsigned int dependency = 0;

		if (!rates[i].rate || rates[i].rate > INT_MAX || !rates[i].volt ||
			(i && rates[i].rate >= rates[i - 1].rate))
			return -EINVAL;
		if (rates[i].rate < min_khz || rates[i].rate > max_khz)
			continue;
		for (j = 0; j < dvfs->num_of_level; j++)
			if (rates[i].rate == dvfs->list_level[j].level)
				break;
		if (j == dvfs->num_of_level)
			return -ERANGE;
		if (minlock) {
			ret = exynos9810_bus_exact_floor(pairs, pair_count,
					rates[i].rate, &dependency);
			if (ret)
				return ret;
		}
		level = &table->levels[count++];
		level->hz = rates[i].rate * 1000UL;
		level->uv = rates[i].volt;
		level->dependency_khz = dependency;
	}
	if (!count)
		return -ERANGE;
	table->boot_khz = cal_dfs_get_boot_freq(cal_id);
	table->resume_khz = cal_dfs_get_resume_freq(cal_id);
	if (!table->boot_khz || !table->resume_khz)
		return -ERANGE;
	table->cal_id = cal_id;
	table->count = count;
	return 0;
}

int exynos9810_bus_upload_mif_constraints(struct device_node *node,
		const struct exynos9810_bus_table *table)
{
	unsigned int channel, size, requested, index, command[4];
	struct ipc_config config = {
		.cmd = command,
		.response = true,
		.indirection = false,
	};
	int ret;

	ret = exynos9810_bus_fvp_validate(table);
	if (ret)
		return ret;
	if (!node || of_property_read_u32(node, "acpm-ipc-channel", &requested) ||
		requested != 1)
		return -EINVAL;
	/* NULL callback allocates no client state in the real IPC implementation.
	 * Table initialization uses channel1, distinct from CAL's rate channel5.
	 */
	ret = acpm_ipc_request_channel(node, NULL, &channel, &size);
	if (ret)
		return ret;
	if (channel != requested || size != sizeof(command))
		return -EINVAL;
	for (index = 0; index <= table->count; index++) {
		ret = exynos9810_bus_fvp_command(table, index, command);
		if (ret)
			return ret;
		ret = acpm_ipc_send_data(channel, &config);
		if (ret)
			return ret;
	}
	return 0;
}

int exynos9810_bus_set_ap_running(struct device_node *node,
		unsigned int cal_id, unsigned int running)
{
	unsigned int channel, size, requested, command[4];
	struct ipc_config config = {
		.cmd = command,
		.response = true,
		.indirection = false,
	};
	int ret;

	ret = exynos9810_bus_ap_command(cal_id, running, command);
	if (ret)
		return ret;
	if (!node || of_property_read_u32(node, "acpm-ipc-channel", &requested) ||
	    requested != 1)
		return -EINVAL;
	ret = acpm_ipc_request_channel(node, NULL, &channel, &size);
	if (ret)
		return ret;
	if (channel != requested || size != sizeof(command))
		return -EINVAL;
	return acpm_ipc_send_data(channel, &config);
}
