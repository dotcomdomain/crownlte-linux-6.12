/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef EXYNOS9810_BUS_TABLES_H
#define EXYNOS9810_BUS_TABLES_H

#include <linux/errno.h>
#include <linux/types.h>

#define EXYNOS9810_BUS_LEVELS 48
#define EXYNOS9810_MIF_CAL_ID 0x0b040000
#define EXYNOS9810_INT_CAL_ID 0x0b040001
#define EXYNOS9810_FSYS0_CAL_ID 0x0b040006

struct device_node;

struct exynos9810_bus_level {
	unsigned long hz;
	unsigned int uv;
	unsigned int dependency_khz;
};

struct exynos9810_bus_table {
	struct exynos9810_bus_level levels[EXYNOS9810_BUS_LEVELS];
	unsigned int count;
	unsigned int cal_id;
	unsigned int boot_khz;
	unsigned int resume_khz;
};

/* Original FVP AP-state protocol uses devfreq type0/1, not an arbitrary CAL
 * domain index. FSYS0 has CAL index6 but does not use this release message.
 */
static inline int exynos9810_bus_ap_command(unsigned int cal_id,
		unsigned int running, unsigned int command[4])
{
	unsigned int type;

	if (!command || running > 1)
		return -EINVAL;
	switch (cal_id) {
	case EXYNOS9810_MIF_CAL_ID:
		type = 0;
		break;
	case EXYNOS9810_INT_CAL_ID:
		type = 1;
		break;
	default:
		return -EINVAL;
	}
	command[0] = type;
	command[1] = running;
	command[2] = 5; /* DATA_INIT */
	command[3] = 2; /* RELEASE */
	return 0;
}

/* Exact matching is intentional. Never extrapolate a missing firmware row. */
static inline int exynos9810_bus_exact_floor(const unsigned int *pairs,
		unsigned int count, unsigned int khz, unsigned int *floor)
{
	unsigned int i;

	if (!pairs || !floor || !count || count > EXYNOS9810_BUS_LEVELS || !khz)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		if (!pairs[2 * i] || !pairs[2 * i + 1] ||
			pairs[2 * i + 1] > 0x7fffffffU ||
			(i && pairs[2 * i] >= pairs[2 * (i - 1)]))
			return -EINVAL;
	}
	for (i = 0; i < count; i++) {
		if (pairs[2 * i] == khz) {
			*floor = pairs[2 * i + 1];
			return 0;
		}
	}
	return -ERANGE;
}

/* Validate the entire transaction before sending its first firmware command. */
static inline int exynos9810_bus_fvp_validate(const struct exynos9810_bus_table *table)
{
	unsigned int i;
	bool boot_found = false;

	if (!table || table->cal_id != EXYNOS9810_MIF_CAL_ID ||
		!table->count || table->count > EXYNOS9810_BUS_LEVELS)
		return -EINVAL;
	for (i = 0; i < table->count; i++) {
		const struct exynos9810_bus_level *level = &table->levels[i];

		if (!level->hz || level->hz % 1000 ||
			level->hz / 1000 > 0x7fffffffUL || !level->uv ||
			!level->dependency_khz || level->dependency_khz > 0x7fffffffU ||
			(i && level->hz >= table->levels[i - 1].hz))
			return -EINVAL;
		if (level->hz / 1000 == table->boot_khz)
			boot_found = true;
	}
	return boot_found ? 0 : -ERANGE;
}

static inline int exynos9810_bus_fvp_command(const struct exynos9810_bus_table *table,
		unsigned int index, unsigned int command[4])
{
	int ret;

	if (!command)
		return -EINVAL;
	ret = exynos9810_bus_fvp_validate(table);
	if (ret)
		return ret;
	if (index > table->count)
		return -ERANGE;
	command[0] = index;
	command[2] = 5; /* Original Samsung DATA_INIT. */
	if (index == table->count) {
		command[1] = table->boot_khz;
		command[3] = 1; /* Original Samsung SET_CONST. */
	} else {
		command[1] = table->levels[index].hz / 1000;
		command[3] = table->levels[index].dependency_khz;
	}
	return 0;
}

int exynos9810_bus_build_table(unsigned int cal_id, unsigned int min_khz,
		unsigned int max_khz, struct exynos9810_bus_table *table);
int exynos9810_bus_upload_mif_constraints(struct device_node *node,
		const struct exynos9810_bus_table *table);
int exynos9810_bus_set_ap_running(struct device_node *node,
		unsigned int cal_id, unsigned int running);

#endif
