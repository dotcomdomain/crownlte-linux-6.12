// SPDX-License-Identifier: GPL-2.0-only
/* Exynos9810 CAL/ACPM CPUFreq adapter for the unchanged crownlte firmware.
 * The original Samsung ACME driver supplies the table and MIF constraint
 * semantics. Firmware owns the paired CPU voltage/frequency transition.
 */
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/devfreq.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <soc/samsung/cal-if.h>
#include <soc/samsung/ect_parser.h>

#define EXYNOS9810_DOMAINS 2
#define EXYNOS9810_LEVELS 48
#define EXYNOS9810_CPUCL0 0x0b040002
#define EXYNOS9810_CPUCL1 0x0b040003

struct exynos9810_domain {
	struct device *cpu_dev;
	cpumask_t cpus;
	u32 cal_id;
	struct cpufreq_frequency_table *table;
	u32 *mif_khz;
	unsigned int levels;
	unsigned int opps_added;
	u32 policy_max;
	u32 resume_khz;
	struct dev_pm_qos_request mif_request;
	u32 mif_floor;
};

struct exynos9810_cpufreq {
	struct device *dev;
	struct devfreq *mif;
	struct exynos9810_domain domains[EXYNOS9810_DOMAINS];
};

static struct exynos9810_cpufreq *provider;

/* Match Samsung ACME's descending nearest-higher performance row. */
static int exynos9810_perf_floor(const u32 *pairs, unsigned int count,
			       u32 rate, u32 *floor)
{
	unsigned int i;
	bool matched = false;

	if (!pairs || !count || count > EXYNOS9810_LEVELS)
		return -EINVAL;
	*floor = 0;
	for (i = 0; i < count; i++) {
		if (!pairs[2 * i] || pairs[2 * i + 1] > INT_MAX ||
		    (i && pairs[2 * i] >= pairs[2 * (i - 1)]))
			return -EINVAL;
	}
	for (i = 0; i < count; i++) {
		if (rate <= pairs[2 * i]) {
			*floor = pairs[2 * i + 1];
			matched = true;
		}
		if (rate >= pairs[2 * i])
			break;
	}
	return matched ? 0 : -ERANGE;
}

/* ECT levels are descending. Preserve ACME's previous matched skew floor. */
static int exynos9810_skew_floor(struct ect_minlock_domain *ect,
			       u32 rate, u32 *floor)
{
	unsigned int i;

	if (!ect || !ect->level || ect->num_of_level <= 0 ||
	    ect->num_of_level > EXYNOS9810_LEVELS)
		return -EINVAL;
	*floor = ect->level[0].sub_frequencies;
	for (i = 0; i < ect->num_of_level; i++) {
		if (!ect->level[i].main_frequencies ||
		    ect->level[i].sub_frequencies > INT_MAX ||
		    (i && ect->level[i].main_frequencies >=
			  ect->level[i - 1].main_frequencies))
			return -EINVAL;
	}
	for (i = 0; i < ect->num_of_level; i++) {
		if (rate >= ect->level[i].main_frequencies)
			break;
		*floor = ect->level[i].sub_frequencies;
	}
	if (i < ect->num_of_level && rate == ect->level[i].main_frequencies)
		*floor = ect->level[i].sub_frequencies;
	return 0;
}

static struct exynos9810_domain *exynos9810_find_domain(unsigned int cpu)
{
	unsigned int i;

	if (!provider)
		return NULL;
	for (i = 0; i < EXYNOS9810_DOMAINS; i++)
		if (cpumask_test_cpu(cpu, &provider->domains[i].cpus))
			return &provider->domains[i];
	return NULL;
}

static int exynos9810_set_mif_floor(struct exynos9810_domain *domain, u32 floor)
{
	struct devfreq *mif = provider->mif;
	unsigned long actual_hz = 0;
	int ret;

	ret = dev_pm_qos_update_request(&domain->mif_request, floor);
	if (ret < 0)
		return ret;
	domain->mif_floor = floor;
	mutex_lock(&mif->lock);
	ret = update_devfreq(mif);
	if (!ret)
		ret = mif->profile->get_cur_freq(mif->dev.parent, &actual_hz);
	mutex_unlock(&mif->lock);
	if (ret)
		return ret;
	if (actual_hz < (unsigned long)floor * 1000)
		return -ERANGE;
	return 0;
}

static int exynos9810_target_index(struct cpufreq_policy *policy,
				 unsigned int index)
{
	struct exynos9810_domain *domain = policy->driver_data;
	u32 old_floor = domain->mif_floor;
	u32 new_floor;
	int ret;

	if (index >= domain->levels || !cal_dfs_is_ready(domain->cal_id))
		return -ENODEV;
	new_floor = domain->mif_khz[index];
	/* Raise dependent MIF first, then ask firmware to raise the CPU. */
	if (new_floor > old_floor) {
		ret = exynos9810_set_mif_floor(domain, new_floor);
		if (ret)
			return ret;
	}
	ret = cal_dfs_set_rate(domain->cal_id, domain->table[index].frequency);
	if (ret) {
		/* A failed firmware acknowledgement does not prove that the CPU
		 * stayed at its previous rate. Keep the raised dependency floor.
		 */
		return ret;
	}
	/* The CPU transition has succeeded. Retain a conservative old floor if
	 * releasing its MIF requirement fails; do not misreport CPU ownership.
	 */
	if (new_floor < old_floor) {
		ret = exynos9810_set_mif_floor(domain, new_floor);
		if (ret) {
			exynos9810_set_mif_floor(domain, old_floor);
			dev_warn_ratelimited(provider->dev, "MIF floor release failed: %d\n", ret);
		}
	}
	return 0;
}

static unsigned int exynos9810_get(unsigned int cpu)
{
	struct exynos9810_domain *domain = exynos9810_find_domain(cpu);

	if (!domain || !cal_dfs_is_ready(domain->cal_id))
		return 0;
	return cal_dfs_get_rate(domain->cal_id);
}

static int exynos9810_policy_init(struct cpufreq_policy *policy)
{
	struct exynos9810_domain *domain = exynos9810_find_domain(policy->cpu);
	unsigned int rate, i;
	int ret;

	if (!domain)
		return -ENODEV;
	rate = exynos9810_get(policy->cpu);
	for (i = 0; i < domain->levels; i++)
		if (domain->table[i].frequency == rate)
			break;
	if (i == domain->levels)
		return -ERANGE;
	ret = exynos9810_set_mif_floor(domain, domain->mif_khz[i]);
	if (ret)
		return ret;
	cpumask_copy(policy->cpus, &domain->cpus);
	policy->driver_data = domain;
	policy->freq_table = domain->table;
	policy->cur = rate;
	policy->suspend_freq = domain->resume_khz;
	policy->cpuinfo.transition_latency = 100000;
	policy->dvfs_possible_from_any_cpu = true;
	return 0;
}

static void exynos9810_policy_ready(struct cpufreq_policy *policy)
{
	struct exynos9810_domain *domain = policy->driver_data;
	int ret;

	/* Preserve ACME's initial policy-max through the user policy request.
	 * A later scaling_max_freq write can replace this initial setting.
	 */
	if (!domain->policy_max || !policy->max_freq_req)
		return;
	ret = freq_qos_update_request(policy->max_freq_req, domain->policy_max);
	if (ret < 0)
		dev_warn(provider->dev, "initial CPU policy cap failed: %d\n", ret);
}

static struct freq_attr *exynos9810_attrs[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver exynos9810_driver = {
	.name = "exynos9810-acpm",
	.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK | CPUFREQ_IS_COOLING_DEV,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = exynos9810_target_index,
	.get = exynos9810_get,
	.init = exynos9810_policy_init,
	.ready = exynos9810_policy_ready,
	.suspend = cpufreq_generic_suspend,
	.attr = exynos9810_attrs,
};

static void exynos9810_domain_cleanup(void *data)
{
	struct exynos9810_domain *domain = data;

	if (dev_pm_qos_request_active(&domain->mif_request))
		dev_pm_qos_remove_request(&domain->mif_request);
	while (domain->opps_added) {
		unsigned int index = --domain->opps_added;

		dev_pm_opp_remove(domain->cpu_dev,
				  (unsigned long)domain->table[index].frequency * 1000);
	}
}

static int exynos9810_prepare_domain(struct exynos9810_cpufreq *state,
		struct device_node *node, struct exynos9810_domain *domain,
		unsigned int id)
{
	struct device *dev = state->dev;
	struct dvfs_rate_volt rates[EXYNOS9810_LEVELS];
	struct ect_minlock_domain *ect;
	struct device_node *cpu_node;
	struct opp_table *existing;
	const char *ect_name;
	u32 min_khz, max_khz, perf, skew;
	u32 *pairs;
	unsigned int i, j;
	u64 affinity;
	cpumask_t shared;
	int cells, count, cpu, ret;
	void *block;

	if (of_property_read_u32(node, "samsung,cal-id", &domain->cal_id) ||
	    domain->cal_id != (id ? EXYNOS9810_CPUCL1 : EXYNOS9810_CPUCL0))
		return -EINVAL;
	if (!cal_dfs_is_ready(domain->cal_id))
		return -EPROBE_DEFER;
	count = of_count_phandle_with_args(node, "cpus", NULL);
	if (count != 4)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		cpu_node = of_parse_phandle(node, "cpus", i);
		if (!cpu_node)
			return -EINVAL;
		cpu = of_cpu_node_to_id(cpu_node);
		affinity = of_get_cpu_hwid(cpu_node, 0);
		of_node_put(cpu_node);
		if (cpu < 0 || !cpu_possible(cpu) || cpumask_test_cpu(cpu, &domain->cpus) ||
		    (affinity >> 8) != id || (affinity & 0xff) >= 4)
			return -EINVAL;
		cpumask_set_cpu(cpu, &domain->cpus);
	}
	/* Dynamic OPP addition accepts identical duplicates as success. Reject
	 * pre-existing tables so cleanup cannot take ownership of another table.
	 * This platform is the sole OPP creator for these CPU devices.
	 */
	for_each_cpu(cpu, &domain->cpus) {
		struct device *member = get_cpu_device(cpu);

		if (!member)
			return -EPROBE_DEFER;
		existing = dev_pm_opp_get_opp_table(member);
		if (!IS_ERR(existing)) {
			dev_pm_opp_put_opp_table(existing);
			return -EBUSY;
		}
		if (PTR_ERR(existing) != -ENODEV)
			return PTR_ERR(existing);
	}
	domain->cpu_dev = get_cpu_device(cpumask_first(&domain->cpus));
	if (!domain->cpu_dev)
		return -EPROBE_DEFER;
	min_khz = cal_dfs_get_min_freq(domain->cal_id);
	max_khz = cal_dfs_get_max_freq(domain->cal_id);
	if (of_property_read_u32(node, "min-freq", &perf) ||
	    of_property_read_u32(node, "max-freq", &skew))
		return -EINVAL;
	min_khz = max(min_khz, perf);
	max_khz = min(max_khz, skew);
	if (!min_khz || min_khz > max_khz)
		return -EINVAL;
	if (!of_property_read_u32(node, "policy-max", &domain->policy_max) &&
	    (domain->policy_max < min_khz || domain->policy_max > max_khz))
		return -EINVAL;
	count = cal_dfs_get_rate_asv_table(domain->cal_id, rates);
	if (count <= 0 || count > EXYNOS9810_LEVELS)
		return -ENODEV;
	cells = of_property_count_u32_elems(node, "samsung,mif-perf-table");
	if (cells <= 0 || cells % 2 || cells > EXYNOS9810_LEVELS * 2)
		return -EINVAL;
	pairs = devm_kcalloc(dev, cells, sizeof(*pairs), GFP_KERNEL);
	if (!pairs)
		return -ENOMEM;
	ret = of_property_read_u32_array(node, "samsung,mif-perf-table", pairs, cells);
	if (ret)
		return ret;
	if (of_property_read_string(node, "samsung,mif-ect-domain", &ect_name))
		return -EINVAL;
	block = ect_get_block(BLOCK_MINLOCK);
	ect = block ? ect_minlock_get_domain(block, (char *)ect_name) : NULL;
	/* This phone's live ECT contains no CPUCL0 MINLOCK domain. ACME skips
	 * that absent optional constraint, keeping the DT performance table.
	 * CPUCL1's real skew table is mandatory in this adapter.
	 */
	if (!block || (!ect && id))
		return -ENODEV;
	domain->table = devm_kcalloc(dev, count + 1, sizeof(*domain->table), GFP_KERNEL);
	domain->mif_khz = devm_kcalloc(dev, count, sizeof(*domain->mif_khz), GFP_KERNEL);
	if (!domain->table || !domain->mif_khz)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, exynos9810_domain_cleanup, domain);
	if (ret)
		return ret;
	for (i = 0, j = 0; i < count; i++) {
		if (!rates[i].rate || rates[i].rate > UINT_MAX || !rates[i].volt ||
		    (i && rates[i].rate >= rates[i - 1].rate))
			return -EINVAL;
		if (rates[i].rate < min_khz || rates[i].rate > max_khz)
			continue;
		ret = exynos9810_perf_floor(pairs, cells / 2, rates[i].rate, &perf);
		if (ret)
			return ret;
		skew = 0;
		if (ect) {
			ret = exynos9810_skew_floor(ect, rates[i].rate, &skew);
			if (ret)
				return ret;
		}
		domain->table[j].frequency = rates[i].rate;
		domain->table[j].driver_data = j;
		domain->mif_khz[j] = max(perf, skew);
		ret = dev_pm_opp_add(domain->cpu_dev, rates[i].rate * 1000, rates[i].volt);
		if (ret)
			return ret;
		domain->opps_added = ++j;
	}
	if (!j)
		return -EINVAL;
	domain->levels = j;
	domain->table[j].frequency = CPUFREQ_TABLE_END;
	domain->resume_khz = cal_dfs_get_resume_freq(domain->cal_id);
	for (i = 0; i < j; i++)
		if (domain->table[i].frequency == domain->resume_khz)
			break;
	if (i == j)
		return -EINVAL;
	ret = dev_pm_opp_set_sharing_cpus(domain->cpu_dev, &domain->cpus);
	if (ret)
		return ret;
	/* The OPP helper can skip failed member allocations without an error.
	 * All four CPUs must actually share the calibrated table.
	 */
	ret = dev_pm_opp_get_sharing_cpus(domain->cpu_dev, &shared);
	if (ret)
		return ret;
	if (!cpumask_equal(&shared, &domain->cpus))
		return -ENOMEM;
	return dev_pm_qos_add_request(state->mif->dev.parent, &domain->mif_request,
				     DEV_PM_QOS_MIN_FREQUENCY, 0);
}

static void exynos9810_put_mif(void *data)
{
	put_device(data);
}

static int exynos9810_cpufreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos9810_cpufreq *state;
	struct device_node *child;
	unsigned int id = 0;
	int ret;

	if (provider)
		return -EBUSY;
	state = devm_kzalloc(dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;
	state->dev = dev;
	state->mif = devfreq_get_devfreq_by_phandle(dev, "samsung,mif-devfreq", 0);
	if (IS_ERR(state->mif))
		return dev_err_probe(dev, PTR_ERR(state->mif), "MIF devfreq supplier unavailable\n");
	if (!state->mif->profile->get_cur_freq)
		return -ENODEV;
	get_device(&state->mif->dev);
	ret = devm_add_action_or_reset(dev, exynos9810_put_mif, &state->mif->dev);
	if (ret)
		return ret;
	if (!device_link_add(dev, state->mif->dev.parent, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -ENOMEM;
	for_each_available_child_of_node(dev->of_node, child) {
		if (id >= EXYNOS9810_DOMAINS) {
			of_node_put(child);
			return -EINVAL;
		}
		ret = exynos9810_prepare_domain(state, child, &state->domains[id], id);
		if (ret) {
			of_node_put(child);
			return dev_err_probe(dev, ret, "CPU domain%u unavailable\n", id);
		}
		id++;
	}
	if (id != EXYNOS9810_DOMAINS ||
	    cpumask_intersects(&state->domains[0].cpus, &state->domains[1].cpus))
		return -EINVAL;
	provider = state;
	ret = cpufreq_register_driver(&exynos9810_driver);
	if (ret)
		provider = NULL;
	return ret;
}

static void exynos9810_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&exynos9810_driver);
	provider = NULL;
}

static const struct of_device_id exynos9810_cpufreq_match[] = {
	{ .compatible = "samsung,exynos9810-acpm-cpufreq" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos9810_cpufreq_match);

static struct platform_driver exynos9810_cpufreq_platform_driver = {
	.probe = exynos9810_cpufreq_probe,
	.remove = exynos9810_cpufreq_remove,
	.driver = {
		.name = "exynos9810-acpm-cpufreq",
		.of_match_table = exynos9810_cpufreq_match,
	},
};
module_platform_driver(exynos9810_cpufreq_platform_driver);
MODULE_DESCRIPTION("Exynos9810 firmware CPUFreq with original CPU/MIF constraints");
MODULE_LICENSE("GPL");
