// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos 9810 TMU temperatures read through the ACPM TMU plugin.
 *
 * The protocol is Samsung 4.9 exynos_acpm_tmu.[ch]: 16-byte messages on the
 * ACPM channel named by the acpm-tmu node (12 on crownlte). TMU_IPC_INIT is
 * sent once after the plugin is attached; TMU_IPC_READ_TEMP carries the TMU
 * node's "id" as tzid and returns whole degrees Celsius.
 *
 * Unlike 4.9 exynos_tmu.c this driver never writes TMU registers: no trim,
 * threshold, interrupt, CORE_EN or THERM_TRIP_EN programming. It only reads
 * and logs what the bootloader left, once, at probe. Temperatures are only
 * reported when the plugin is attached, TMU_IPC_INIT succeeded, the channel
 * is AP-polled (so the reply really is copied back) and the TMU core is
 * enabled; otherwise get_temp fails instead of inventing a value.
 *
 * Opt-in 4.9 thermal parity, per sensor node (see
 * Documentation/crownlte/perf-tuning.md). Nothing here writes TMU or PMIC
 * registers; the bootloader-armed hardware trip stays as it is.
 *  - samsung,ect-thermal = "<ECT THERMAL function>" + samsung,cooling-cpu:
 *    once the zone's cooling maps are bound, log every trip with the CPU
 *    frequency its cooling state means and the ECT value for that
 *    temperature (4.9 took its step ladder from there).
 *  - samsung,ect-pidtm = "<ECT PIDTM block>" + samsung,cooling-cpu: switch
 *    the zone to power_allocator, but only once a power actor (a cooling
 *    device with an energy model) is bound to its control trip; then apply
 *    the ECT PID constants (k_po, k_pu, k_i, integral_cutoff), as 4.9
 *    exynos_tmu.c did. Until then, or if that never happens, the zone keeps
 *    the step_wise governor and its existing cooling map.
 *  - samsung,hotplug-temps = <out in> + samsung,cooling-cpu: 4.9 took the
 *    big cores offline at hotplug_out_threshold and brought them back below
 *    hotplug_in_threshold. Here the cluster is held at its lowest frequency
 *    instead (a freq QoS maximum), which needs no CPU hotplug.
 */

#include <linux/bitops.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <soc/samsung/acpm_ipc_ctrl.h>
#include <soc/samsung/ect_parser.h>

#include "../thermal_core.h"

#define ACPM_TMU_PLUGIN		"exynos9810_acpm_tmu.fw"
#define ACPM_TMU_MAX_SENSORS	8
#define ACPM_TMU_SELFTEST_MS	105000
#define ACPM_TMU_PARITY_PERIOD_MS	1000
#define ACPM_TMU_PARITY_TRIES		180	/* give up after ~3 min */

/* 4.9 exynos_acpm_tmu.h */
#define TMU_IPC_INIT		0x01
#define TMU_IPC_READ_TEMP	0x02
#define CAP_APM_IRQ		0x1
#define CAP_APM_DIVIDER		0x2

/*
 * 4.9 struct tmu_ipc_request/response, decoded by word so nothing depends
 * on structure layout. Word 0 (ctx/fw_use) carries the ACPM sequence number.
 * REQ  word1: type[7:0] tzid[23:16]
 * RESP word1: type[7:0] ret[15:8] (s8) tzid[23:16] temp[31:24]; word2: stat[7:0]
 */
#define TMU_W1_REQ(type, tz)	((u32)(type) | ((u32)(tz) << 16))
#define TMU_W1_TYPE(w)		((w) & 0xff)
#define TMU_W1_RET(w)		((s8)(((w) >> 8) & 0xff))
#define TMU_W1_TZID(w)		(((w) >> 16) & 0xff)
#define TMU_W1_TEMP(w)		(((w) >> 24) & 0xff)
#define TMU_W2_STAT(w)		((w) & 0xff)

/* 4.9 exynos_tmu.c register map (CONFIG_SOC_EXYNOS9810). Read only here. */
#define TMU_REG_TRIMINFO(p)	((p) <= 7 ? (p) * 4 : ((p) - 8) * 4 + 0x400)
#define TMU_REG_CONTROL		0x20
#define TMU_REG_CONTROL1	0x24
#define TMU_CORE_EN		BIT(0)
#define TMU_THERM_TRIP_EN	BIT(12)
#define TMU_TRIP_MODE(c)	(((c) >> 13) & 0x7)
#define TMU_NUM_PROBE(c)	(((c) >> 16) & 0xf)
#define TMU_REG_THD_TEMP(s)	((s) == 0 ? 0x50 : (s) < 8 ? 0x170 + ((s) - 1) * 0x20 : \
				 0x450 + ((s) - 8) * 0x20)
#define TMU_THD_FALL_OFFSET	0x10
#define TMU_RISE7_SHIFT		16
#define TMU_REG_INTEN(s)	((s) < 5 ? 0x110 + (s) * 0x10 : (s) < 8 ? 0x310 + ((s) - 5) * 0x10 : \
				 0x650 + ((s) - 8) * 0x10)
#define TMU_INTEN_RISE7		BIT(7)
#define TMU_TEMP_MASK		0x1ff
#define TMU_TRIM_85_SHIFT	9
#define TMU_CALIB_SEL(t)	(((t) >> 23) & 0x1)
#define TMU_TWO_POINT_TRIMMING	1
#define TMU_TOTAL_SENSORS	16
#define TMU_REG_EXTENT		0x700	/* highest register read: INTEN(15) */
#define TMU_NO_TEMP		(-999)

struct acpm_tmu_sensor {
	struct device *dev;
	const char *name;
	u32 id;
	u32 sensors;
	u32 control;
	u32 control1;
	bool core_en;
	bool trip_en;
	int rise7_min_c;	/* lowest decoded rise7 threshold, TMU_NO_TEMP if none */
	struct thermal_zone_device *tzd;
	unsigned int failures;	/* consecutive get_temp failures */
	/* opt-in 4.9 parity */
	const char *ect_thermal;	/* ECT THERMAL function name */
	const char *ect_pidtm;		/* ECT PIDTM block name */
	int cooling_cpu;		/* -1: none */
	struct delayed_work parity_work;
	unsigned int parity_tries;
	bool pa_done;			/* power_allocator switch decided */
	int hp_out, hp_in;		/* mC; hp_out == 0: off */
	bool hp_want;			/* cap wanted (written in get_temp) */
	bool hp_capped;			/* cap applied (hp_lock) */
	struct mutex hp_lock;
	struct work_struct hp_work;
	struct freq_qos_request hp_req;
	struct notifier_block hp_nb;
};

/*
 * A zone whose sensor stays unreadable gets no trip handling from the
 * thermal core (get_temp errors only re-arm polling), so the software
 * critical trip would silently stop protecting. After this many
 * consecutive failures (~10 s at the 1 s polling interval) treat the zone
 * as critical. The bootloader-armed TMU hardware trip stays the backstop.
 */
#define ACPM_TMU_MAX_FAILURES	10

static struct acpm_tmu_core {
	struct device *dev;
	unsigned int channel;
	int channel_ret;
	int channel_mode;
	int plugin_ret;
	struct exynos_acpm_plugin_status plugin;
	bool init_sent;
	int init_ret;
	u32 init_reply[4];
	u8 caps;
	bool ready;
	struct acpm_tmu_sensor *sensors[ACPM_TMU_MAX_SENSORS];
	unsigned int num_sensors;
	struct delayed_work selftest;
} acpm_tmu;

/* acpm_tmu.dev/ready/sensors[]; IPC is serialized separately. */
static DEFINE_MUTEX(acpm_tmu_state_lock);
static DEFINE_MUTEX(acpm_tmu_ipc_lock);

static int acpm_tmu_xfer(u32 msg[4])
{
	struct ipc_config config = {
		.cmd = msg,
		.response = true,
		.indirection = false,
	};
	int ret;

	mutex_lock(&acpm_tmu_ipc_lock);
	/* On a polled channel the reply with our sequence number replaces msg. */
	ret = acpm_ipc_send_data(acpm_tmu.channel, &config);
	mutex_unlock(&acpm_tmu_ipc_lock);
	return ret;
}

static int acpm_tmu_read(struct acpm_tmu_sensor *s, int *millicelsius, u32 raw[4])
{
	u32 msg[4] = { 0, TMU_W1_REQ(TMU_IPC_READ_TEMP, s->id), 0, 0 };
	int ret;

	if (raw)
		memset(raw, 0, 4 * sizeof(u32));
	if (!READ_ONCE(acpm_tmu.ready))
		return -ENODEV;
	if (!s->core_en)
		return -ENODATA;
	ret = acpm_tmu_xfer(msg);
	if (raw)
		memcpy(raw, msg, sizeof(msg));
	if (ret)
		return ret;
	/*
	 * Replies are matched by a 6-bit sequence number only. A late reply to
	 * an earlier (timed-out) request can carry a reused number, so accept
	 * only a reply that echoes this request's type and zone.
	 */
	if (TMU_W1_TYPE(msg[1]) != TMU_IPC_READ_TEMP || TMU_W1_TZID(msg[1]) != s->id) {
		pr_warn_ratelimited("acpm-tmu: %s: reply %08x does not match READ_TEMP tzid %u\n",
				    s->name, msg[1], s->id);
		return -EPROTO;
	}
	if (TMU_W1_RET(msg[1]) < 0)
		return -EIO;
	*millicelsius = TMU_W1_TEMP(msg[1]) * 1000;
	return 0;
}

static int acpm_tmu_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct acpm_tmu_sensor *s = thermal_zone_device_priv(tz);
	int ret = acpm_tmu_read(s, temp, NULL);

	if (!ret) {
		s->failures = 0;
		/* 4.9 exynos_throttle_cpu_hotplug(): out at >= out, back below in */
		if (READ_ONCE(s->hp_out)) {
			bool want = READ_ONCE(s->hp_want);

			if (!want && *temp >= s->hp_out)
				want = true;
			else if (want && *temp < s->hp_in)
				want = false;
			if (want != READ_ONCE(s->hp_want)) {
				WRITE_ONCE(s->hp_want, want);
				dev_info(s->dev, "%s: %d mC: %s the cpu%d cluster cap (hotplug-%s %d mC)\n",
					 s->name, *temp, want ? "applying" : "releasing",
					 s->cooling_cpu, want ? "out" : "in",
					 want ? s->hp_out : s->hp_in);
			}
			/* also retries a cap that could not be applied yet */
			if (want != READ_ONCE(s->hp_capped))
				schedule_work(&s->hp_work);
		}
		return 0;
	}
	if (++s->failures == ACPM_TMU_MAX_FAILURES) {
		dev_crit(s->dev, "%s: temperature unreadable %u times (last %d); treating as critical\n",
			 s->name, s->failures, ret);
		thermal_zone_device_critical(tz);
	}
	return ret;
}

static const struct thermal_zone_device_ops acpm_tmu_ops = {
	.get_temp = acpm_tmu_get_temp,
};

/* ---- 4.9 thermal hotplug equivalent: a freq QoS cap on the cluster ---- */

static void acpm_tmu_hp_sync(struct acpm_tmu_sensor *s)
{
	struct cpufreq_policy *policy;
	bool want;
	int ret;

	mutex_lock(&s->hp_lock);
	want = READ_ONCE(s->hp_want);
	if (!freq_qos_request_active(&s->hp_req) || want == s->hp_capped) {
		mutex_unlock(&s->hp_lock);
		return;
	}
	policy = cpufreq_cpu_get(s->cooling_cpu);
	if (want && !policy) {
		/* no policy to read the lowest level from: retried from get_temp */
		mutex_unlock(&s->hp_lock);
		return;
	}
	ret = freq_qos_update_request(&s->hp_req, want ? policy->cpuinfo.min_freq :
				      FREQ_QOS_MAX_DEFAULT_VALUE);
	if (ret >= 0)
		WRITE_ONCE(s->hp_capped, want);
	dev_info(s->dev, "%s: cpu%d cluster max %s %u kHz ret=%d\n", s->name,
		 s->cooling_cpu, want ? "capped at" : "released, cap was",
		 policy ? policy->cpuinfo.min_freq : 0, ret);
	if (policy)
		cpufreq_cpu_put(policy);
	mutex_unlock(&s->hp_lock);
}

static void acpm_tmu_hp_work(struct work_struct *work)
{
	acpm_tmu_hp_sync(container_of(work, struct acpm_tmu_sensor, hp_work));
}

/* The request lives on the policy's constraints: follow the policy. */
static void acpm_tmu_hp_attach(struct acpm_tmu_sensor *s, struct cpufreq_policy *policy)
{
	int ret;

	mutex_lock(&s->hp_lock);
	if (freq_qos_request_active(&s->hp_req) ||
	    !cpumask_test_cpu(s->cooling_cpu, policy->related_cpus)) {
		mutex_unlock(&s->hp_lock);
		return;
	}
	s->hp_capped = false;
	ret = freq_qos_add_request(&policy->constraints, &s->hp_req, FREQ_QOS_MAX,
				   FREQ_QOS_MAX_DEFAULT_VALUE);
	mutex_unlock(&s->hp_lock);
	dev_info(s->dev, "%s: hotplug-equivalent cap armed on policy%u: cap at %u kHz from %d mC, release below %d mC (ret=%d)\n",
		 s->name, policy->cpu, policy->cpuinfo.min_freq, s->hp_out, s->hp_in, ret);
	acpm_tmu_hp_sync(s);
}

static int acpm_tmu_hp_notify(struct notifier_block *nb, unsigned long event, void *data)
{
	struct acpm_tmu_sensor *s = container_of(nb, struct acpm_tmu_sensor, hp_nb);
	struct cpufreq_policy *policy = data;

	if (!cpumask_test_cpu(s->cooling_cpu, policy->related_cpus))
		return NOTIFY_DONE;
	if (event == CPUFREQ_CREATE_POLICY) {
		acpm_tmu_hp_attach(s, policy);
	} else if (event == CPUFREQ_REMOVE_POLICY) {
		mutex_lock(&s->hp_lock);
		if (freq_qos_request_active(&s->hp_req))
			freq_qos_remove_request(&s->hp_req);
		s->hp_capped = false;
		mutex_unlock(&s->hp_lock);
	}
	return NOTIFY_OK;
}

static void acpm_tmu_hp_init(struct acpm_tmu_sensor *s)
{
	struct device_node *np = s->dev->of_node;
	struct cpufreq_policy *policy;
	u32 t[2];
	int ret;

	if (of_property_read_u32_array(np, "samsung,hotplug-temps", t, 2))
		return;
	if (s->cooling_cpu < 0 || !t[0] || t[1] >= t[0] || t[0] > 125000) {
		dev_err(s->dev, "%s: samsung,hotplug-temps <%u %u> needs out > in and samsung,cooling-cpu; ignored\n",
			s->name, t[0], t[1]);
		return;
	}
	mutex_init(&s->hp_lock);
	INIT_WORK(&s->hp_work, acpm_tmu_hp_work);
	s->hp_nb.notifier_call = acpm_tmu_hp_notify;
	ret = cpufreq_register_notifier(&s->hp_nb, CPUFREQ_POLICY_NOTIFIER);
	if (ret) {
		dev_err(s->dev, "%s: hotplug-equivalent cap unavailable: %d\n", s->name, ret);
		return;
	}
	s->hp_in = t[1];
	WRITE_ONCE(s->hp_out, t[0]);	/* get_temp may run from here on */
	policy = cpufreq_cpu_get(s->cooling_cpu);
	if (policy) {
		acpm_tmu_hp_attach(s, policy);
		cpufreq_cpu_put(policy);
	}
}

/* ---- ECT parity report and the guarded power_allocator switch ---- */

static int acpm_tmu_ect_param(struct ect_pidtm_block *b, const char *name)
{
	int i;

	for (i = 0; i < b->num_of_parameter; i++)
		if (!strcmp(b->param_name_list[i], name))
			return b->param_value_list[i];
	return -1;
}

static bool acpm_tmu_cdev_is_power_actor(struct thermal_cooling_device *cdev)
{
	return cdev->ops->get_requested_power && cdev->ops->state2power &&
	       cdev->ops->power2state;
}

/*
 * Instances bound to the zone's last passive trip (power_allocator's control
 * trip): how many, and how many of them are power actors. tz->lock held.
 */
static void acpm_tmu_count_actors(struct thermal_zone_device *tz,
				  unsigned int *bound, unsigned int *actors)
{
	const struct thermal_trip_desc *td, *last = NULL;
	struct thermal_instance *ins;

	*bound = *actors = 0;
	for_each_trip_desc(tz, td)
		if (td->trip.type == THERMAL_TRIP_PASSIVE)
			last = td;
	if (!last)
		return;
	list_for_each_entry(ins, &last->thermal_instances, trip_node) {
		(*bound)++;
		if (acpm_tmu_cdev_is_power_actor(ins->cdev))
			(*actors)++;
	}
}

/* frequency of cooling state @state of a cpufreq cooling device on @policy */
static unsigned int acpm_tmu_state_khz(struct cpufreq_policy *policy, unsigned long state)
{
	unsigned int n = cpufreq_table_count_valid_entries(policy);

	if (!policy->freq_table || state >= n)
		return 0;
	/* cpufreq_cooling: state 0 is the highest frequency */
	if (policy->freq_table_sorted == CPUFREQ_TABLE_SORTED_ASCENDING)
		return policy->freq_table[n - 1 - state].frequency;
	return policy->freq_table[state].frequency;
}

static void acpm_tmu_report_trips(struct acpm_tmu_sensor *s, struct cpufreq_policy *policy)
{
	struct thermal_zone_device *tz = s->tzd;
	struct ect_ap_thermal_function *fn = NULL;
	const struct thermal_trip_desc *td;
	struct thermal_instance *ins;
	unsigned int checked = 0, worse = 0;
	void *block;

	if (s->ect_thermal) {
		block = ect_get_block(BLOCK_AP_THERMAL);
		fn = block ? ect_ap_thermal_get_function(block, (char *)s->ect_thermal) : NULL;
		if (!fn)
			dev_warn(s->dev, "%s: ECT THERMAL function %s not found\n",
				 s->name, s->ect_thermal);
	}
	if (fn) {
		int i;

		for (i = 0; i < fn->num_of_range; i++)
			dev_info(s->dev, "%s: ECT %s range %u-%u C max %u kHz flag %u\n",
				 s->name, s->ect_thermal,
				 fn->range_list[i].lower_bound_temperature,
				 fn->range_list[i].upper_bound_temperature,
				 fn->range_list[i].max_frequency, fn->range_list[i].flag);
	}

	mutex_lock(&tz->lock);
	for_each_trip_desc(tz, td) {
		const struct thermal_trip *trip = &td->trip;
		bool any = false;

		list_for_each_entry(ins, &td->thermal_instances, trip_node) {
			unsigned int khz = 0, ect = 0;
			const char *verdict = "";
			int i;

			any = true;
			if (policy && !strncmp(ins->cdev->type, "thermal-cpufreq-", 16))
				khz = acpm_tmu_state_khz(policy, ins->upper);
			/* ECT range that starts at this trip temperature */
			for (i = 0; fn && i < fn->num_of_range; i++)
				if (fn->range_list[i].lower_bound_temperature * 1000 ==
				    trip->temperature)
					ect = fn->range_list[i].max_frequency;
			if (ect && khz) {
				checked++;
				if (khz == ect) {
					verdict = " = ECT";
				} else if (khz < ect) {
					verdict = " below ECT (more throttling)";
				} else {
					verdict = " ABOVE ECT";
					worse++;
				}
			}
			dev_info(s->dev, "%s: trip %d mC %s hyst %d -> %s state %lu..%lu = %u kHz (ECT %u kHz)%s\n",
				 s->name, trip->temperature, thermal_trip_type_name(trip->type),
				 trip->hysteresis, ins->cdev->type, ins->lower, ins->upper,
				 khz, ect, verdict);
		}
		if (!any)
			dev_info(s->dev, "%s: trip %d mC %s hyst %d (no cooling device)\n",
				 s->name, trip->temperature,
				 thermal_trip_type_name(trip->type), trip->hysteresis);
	}
	dev_info(s->dev, "%s: governor %s polling %u/%u ms, sustainable %u mW, k_po %d k_pu %d k_i %d integral_cutoff %d (FRAC_BITS 10); ECT ladder %u checked, %u above ECT\n",
		 s->name, tz->governor ? tz->governor->name : "none",
		 jiffies_to_msecs(tz->passive_delay_jiffies),
		 jiffies_to_msecs(tz->polling_delay_jiffies),
		 tz->tzp ? tz->tzp->sustainable_power : 0,
		 tz->tzp ? tz->tzp->k_po : 0, tz->tzp ? tz->tzp->k_pu : 0,
		 tz->tzp ? tz->tzp->k_i : 0, tz->tzp ? tz->tzp->integral_cutoff : 0,
		 checked, worse);
	mutex_unlock(&tz->lock);
}

/* 4.9 exynos_tmu.c: PID constants from the ECT PIDTM block (<< FRAC_BITS). */
static void acpm_tmu_power_allocator(struct acpm_tmu_sensor *s)
{
	struct thermal_zone_device *tz = s->tzd;
	struct ect_pidtm_block *b;
	int k_po, k_pu, k_i, cutoff, p_control, ret;
	char gov[] = "power_allocator";
	void *block;

	block = ect_get_block(BLOCK_PIDTM);
	b = block ? ect_pidtm_get_block(block, (char *)s->ect_pidtm) : NULL;
	if (!b) {
		dev_err(s->dev, "%s: ECT PIDTM block %s not found: governor unchanged\n",
			s->name, s->ect_pidtm);
		return;
	}
	k_po = acpm_tmu_ect_param(b, "k_po");
	k_pu = acpm_tmu_ect_param(b, "k_pu");
	k_i = acpm_tmu_ect_param(b, "k_i");
	cutoff = acpm_tmu_ect_param(b, "integral_cutoff");
	p_control = acpm_tmu_ect_param(b, "p_control_t");
	dev_info(s->dev, "%s: ECT PIDTM %s: k_po %d k_pu %d k_i %d i_max %d integral_cutoff %d p_control_t %d mW hotplug out/in %d/%d C limited_frequency %d\n",
		 s->name, s->ect_pidtm, k_po, k_pu, k_i,
		 acpm_tmu_ect_param(b, "i_max"), cutoff, p_control,
		 acpm_tmu_ect_param(b, "hotplug_out_threshold"),
		 acpm_tmu_ect_param(b, "hotplug_in_threshold"),
		 acpm_tmu_ect_param(b, "limited_frequency"));
	if (k_po < 0 || k_pu < 0 || k_i < 0 || cutoff < 0 || p_control <= 0 ||
	    k_po > 10000 || k_pu > 10000 || k_i > 10000) {
		dev_err(s->dev, "%s: ECT PID constants incomplete: governor unchanged\n", s->name);
		return;
	}
	if (!tz->tzp || tz->tzp->sustainable_power != p_control) {
		dev_err(s->dev, "%s: DT sustainable-power %u mW != ECT p_control_t %d mW: governor unchanged\n",
			s->name, tz->tzp ? tz->tzp->sustainable_power : 0, p_control);
		return;
	}
	/* writable: thermal_zone_device_set_policy() strim()s its argument */
	ret = thermal_zone_device_set_policy(tz, gov);
	if (ret) {
		dev_err(s->dev, "%s: power_allocator not set (%d): governor unchanged\n",
			s->name, ret);
		return;
	}
	/* bind estimated them from sustainable power; 4.9 used the ECT ones */
	mutex_lock(&tz->lock);
	tz->tzp->k_po = k_po << 10;
	tz->tzp->k_pu = k_pu << 10;
	tz->tzp->k_i = k_i << 10;
	tz->tzp->integral_cutoff = cutoff;
	mutex_unlock(&tz->lock);
	dev_info(s->dev, "%s: governor power_allocator, ECT PID constants applied\n", s->name);
}

static void acpm_tmu_parity_work(struct work_struct *work)
{
	struct acpm_tmu_sensor *s = container_of(to_delayed_work(work),
						 struct acpm_tmu_sensor, parity_work);
	struct thermal_zone_device *tz = s->tzd;
	struct cpufreq_policy *policy = NULL;
	unsigned int bound, actors;
	bool last = ++s->parity_tries >= ACPM_TMU_PARITY_TRIES;

	mutex_lock(&tz->lock);
	acpm_tmu_count_actors(tz, &bound, &actors);
	mutex_unlock(&tz->lock);
	if (s->cooling_cpu >= 0)
		policy = cpufreq_cpu_get(s->cooling_cpu);

	/* wait for the cpufreq cooling device(s) to bind */
	if ((!bound || (s->cooling_cpu >= 0 && !policy)) && !last) {
		if (policy)
			cpufreq_cpu_put(policy);
		queue_delayed_work(system_long_wq, &s->parity_work,
				   msecs_to_jiffies(ACPM_TMU_PARITY_PERIOD_MS));
		return;
	}
	if (s->ect_pidtm && !s->pa_done) {
		s->pa_done = true;
		if (bound && actors == bound)
			acpm_tmu_power_allocator(s);
		else
			dev_err(s->dev, "%s: power_allocator NOT enabled: %u of %u cooling devices on the control trip are power actors (energy model missing?); keeping %s\n",
				s->name, actors, bound,
				tz->governor ? tz->governor->name : "none");
	}
	acpm_tmu_report_trips(s, policy);
	if (policy)
		cpufreq_cpu_put(policy);
}

static void acpm_tmu_parity_init(struct acpm_tmu_sensor *s)
{
	struct device_node *np = s->dev->of_node, *cpu_np;

	s->cooling_cpu = -1;
	cpu_np = of_parse_phandle(np, "samsung,cooling-cpu", 0);
	if (cpu_np) {
		s->cooling_cpu = of_cpu_node_to_id(cpu_np);
		of_node_put(cpu_np);
	}
	of_property_read_string(np, "samsung,ect-thermal", &s->ect_thermal);
	of_property_read_string(np, "samsung,ect-pidtm", &s->ect_pidtm);
	if (s->ect_pidtm && s->cooling_cpu < 0) {
		dev_err(s->dev, "%s: samsung,ect-pidtm needs samsung,cooling-cpu; ignored\n", s->name);
		s->ect_pidtm = NULL;
	}
	acpm_tmu_hp_init(s);
	if (!s->ect_thermal && !s->ect_pidtm)
		return;
	INIT_DELAYED_WORK(&s->parity_work, acpm_tmu_parity_work);
	queue_delayed_work(system_long_wq, &s->parity_work,
			   msecs_to_jiffies(ACPM_TMU_PARITY_PERIOD_MS));
}

static void acpm_tmu_selftest(struct work_struct *work)
{
	const struct exynos_acpm_plugin_status *p = &acpm_tmu.plugin;
	struct acpm_tmu_sensor *sensors[ACPM_TMU_MAX_SENSORS];
	unsigned int count, i;
	int round;

	pr_info("acpm-tmu selftest: plugin ret=%d found=%u index=%u id=%u base=%#x size=%#x image=%u loaded=%u dp_a=%d reply=%08x %08x %08x %08x is_attached_after=%u\n",
		acpm_tmu.plugin_ret, p->found, p->index, p->id, p->base, p->size,
		p->image_bytes, p->loaded, p->attach_ret, p->dp_response[0],
		p->dp_response[1], p->dp_response[2], p->dp_response[3],
		p->post_attached);
	pr_info("acpm-tmu selftest: channel %u ret=%d polled=%d; TMU_IPC_INIT sent=%u ret=%d reply=%08x %08x %08x %08x caps=%#x apm_irq=%u apm_divider=%u ready=%u\n",
		acpm_tmu.channel, acpm_tmu.channel_ret, acpm_tmu.channel_mode,
		acpm_tmu.init_sent, acpm_tmu.init_ret, acpm_tmu.init_reply[0],
		acpm_tmu.init_reply[1], acpm_tmu.init_reply[2], acpm_tmu.init_reply[3],
		acpm_tmu.caps, !!(acpm_tmu.caps & CAP_APM_IRQ),
		!!(acpm_tmu.caps & CAP_APM_DIVIDER), READ_ONCE(acpm_tmu.ready));

	mutex_lock(&acpm_tmu_state_lock);
	count = acpm_tmu.num_sensors;
	memcpy(sensors, acpm_tmu.sensors, sizeof(sensors));
	mutex_unlock(&acpm_tmu_state_lock);

	for (i = 0; i < count; i++)
		pr_info("acpm-tmu selftest: %s id=%u CONTROL=%#010x CORE_EN=%u THERM_TRIP_EN=%u TRIP_MODE=%u rise7_min=%d C zone=%s\n",
			sensors[i]->name, sensors[i]->id, sensors[i]->control,
			sensors[i]->core_en, sensors[i]->trip_en,
			TMU_TRIP_MODE(sensors[i]->control), sensors[i]->rise7_min_c,
			sensors[i]->tzd ? "registered" : "not-registered");

	for (round = 0; round < 3; round++) {
		for (i = 0; i < count; i++) {
			int mc = 0, zone_mc = 0, zone_ret = -ENODEV, ret;
			u32 raw[4];

			ret = acpm_tmu_read(sensors[i], &mc, raw);
			if (sensors[i]->tzd)
				zone_ret = thermal_zone_get_temp(sensors[i]->tzd, &zone_mc);
			pr_info("acpm-tmu selftest: round %d %s tzid=%u ret=%d temp=%d mC stat=%u resp_type=%#x resp_ret=%d resp_tzid=%u raw=%08x %08x %08x %08x zone_ret=%d zone=%d mC\n",
				round, sensors[i]->name, sensors[i]->id, ret,
				ret ? 0 : mc, TMU_W2_STAT(raw[2]), TMU_W1_TYPE(raw[1]),
				TMU_W1_RET(raw[1]), TMU_W1_TZID(raw[1]),
				raw[0], raw[1], raw[2], raw[3], zone_ret,
				zone_ret ? 0 : zone_mc);
		}
		if (round < 2)
			msleep(1000);
	}
	pr_info("acpm-tmu selftest: done sensors=%u\n", count);
}

static int acpm_tmu_core_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const char *fw_name = ACPM_TMU_PLUGIN;
	unsigned int size = 0;
	int ret;

	mutex_lock(&acpm_tmu_state_lock);
	ret = acpm_tmu.dev ? -EBUSY : 0;
	mutex_unlock(&acpm_tmu_state_lock);
	if (ret)
		return ret;

	of_property_read_string(np, "samsung,acpm-plugin", &fw_name);
	ret = exynos_acpm_get_plugin_status(fw_name, &acpm_tmu.plugin);
	if (ret == -EPROBE_DEFER)
		return ret;
	acpm_tmu.plugin_ret = ret;

	acpm_tmu.channel_ret = acpm_ipc_request_channel(np, NULL, &acpm_tmu.channel, &size);
	acpm_tmu.channel_mode = -ENODEV;
	if (!acpm_tmu.channel_ret && size != 4 * sizeof(u32))
		acpm_tmu.channel_ret = -EPROTO;
	if (!acpm_tmu.channel_ret)
		acpm_tmu.channel_mode = acpm_ipc_get_ch_mode(acpm_tmu.channel);

	if (acpm_tmu.plugin_ret) {
		acpm_tmu.init_ret = acpm_tmu.plugin_ret;
		dev_err(dev, "plugin %s not attached (%d); TMU_IPC_INIT not sent\n",
			fw_name, acpm_tmu.plugin_ret);
	} else if (acpm_tmu.channel_ret) {
		acpm_tmu.init_ret = acpm_tmu.channel_ret;
		dev_err(dev, "ACPM channel unavailable (%d); TMU_IPC_INIT not sent\n",
			acpm_tmu.channel_ret);
	} else if (acpm_tmu.channel_mode != 1) {
		/* An interrupt-mode reply is never copied back: refuse. */
		acpm_tmu.init_ret = -EOPNOTSUPP;
		dev_err(dev, "ACPM channel %u is not AP-polled (%d); TMU_IPC_INIT not sent\n",
			acpm_tmu.channel, acpm_tmu.channel_mode);
	} else {
		u32 msg[4] = { 0, TMU_W1_REQ(TMU_IPC_INIT, 0), 0, 0 };

		acpm_tmu.init_sent = true;
		ret = acpm_tmu_xfer(msg);
		memcpy(acpm_tmu.init_reply, msg, sizeof(msg));
		/* Only a reply that echoes INIT is INIT reply (see acpm_tmu_read). */
		if (!ret && TMU_W1_TYPE(msg[1]) != TMU_IPC_INIT)
			ret = -EPROTO;
		if (!ret && TMU_W1_RET(msg[1]) < 0)
			ret = -EIO;
		if (!ret)
			acpm_tmu.caps = TMU_W1_RET(msg[1]) & (CAP_APM_IRQ | CAP_APM_DIVIDER);
		acpm_tmu.init_ret = ret;
		dev_info(dev, "TMU_IPC_INIT on channel %u ret=%d reply=%08x %08x %08x %08x caps=%#x\n",
			 acpm_tmu.channel, ret, msg[0], msg[1], msg[2], msg[3], acpm_tmu.caps);
	}

	mutex_lock(&acpm_tmu_state_lock);
	acpm_tmu.dev = dev;
	WRITE_ONCE(acpm_tmu.ready, acpm_tmu.init_sent && !acpm_tmu.init_ret);
	mutex_unlock(&acpm_tmu_state_lock);

	if (of_property_read_bool(np, "samsung,tmu-selftest")) {
		unsigned long now = jiffies - INITIAL_JIFFIES;
		unsigned long at = msecs_to_jiffies(ACPM_TMU_SELFTEST_MS);

		INIT_DELAYED_WORK(&acpm_tmu.selftest, acpm_tmu_selftest);
		queue_delayed_work(system_long_wq, &acpm_tmu.selftest,
				   at > now ? at - now : 0);
	}
	return 0;
}

static int acpm_tmu_decode(u32 code, u32 trim, u32 cal_type, u32 efuse,
			   u32 first, u32 second)
{
	int e1 = trim & TMU_TEMP_MASK;
	int e2 = (trim >> TMU_TRIM_85_SHIFT) & TMU_TEMP_MASK;

	/* 4.9 code_to_temp_with_sensorinfo(), without the 10..125 C clamp. */
	if (!e1)
		e1 = efuse & TMU_TEMP_MASK;
	if (cal_type != TMU_TWO_POINT_TRIMMING)
		return (int)code - e1 + (int)first;
	if (!e2)
		e2 = (efuse >> TMU_TRIM_85_SHIFT) & TMU_TEMP_MASK;
	if (e2 == e1)
		return TMU_NO_TEMP;
	return ((int)code - e1) * ((int)second - (int)first) / (e2 - e1) + (int)first;
}

/* Read-only: what did the bootloader leave in the TMU hardware trip path? */
static void acpm_tmu_report_hw_trip(struct acpm_tmu_sensor *s, void __iomem *base)
{
	struct device_node *np = s->dev->of_node;
	u32 first = 25, second = 85, efuse = 55, cal_type;
	unsigned long mask = s->sensors;
	unsigned int sensor;

	of_property_read_u32(np, "samsung,tmu_first_point_trim", &first);
	of_property_read_u32(np, "samsung,tmu_second_point_trim", &second);
	of_property_read_u32(np, "samsung,tmu_efuse_value", &efuse);

	s->control = readl(base + TMU_REG_CONTROL);
	s->control1 = readl(base + TMU_REG_CONTROL1);
	s->core_en = s->control & TMU_CORE_EN;
	s->trip_en = s->control & TMU_THERM_TRIP_EN;
	s->rise7_min_c = TMU_NO_TEMP;
	cal_type = TMU_CALIB_SEL(readl(base + TMU_REG_TRIMINFO(0)));
	dev_info(s->dev, "acpm-tmu: %s id=%u hw-trip (read-only, not programmed): CONTROL=%#010x CORE_EN=%u THERM_TRIP_EN=%u TRIP_MODE=%u CONTROL1=%#010x NUM_PROBE=%u cal_type=%u\n",
		 s->name, s->id, s->control, s->core_en, s->trip_en,
		 TMU_TRIP_MODE(s->control), s->control1, TMU_NUM_PROBE(s->control1),
		 cal_type);

	for_each_set_bit(sensor, &mask, TMU_TOTAL_SENSORS) {
		u32 trim = readl(base + TMU_REG_TRIMINFO(sensor));
		u32 rise = readl(base + TMU_REG_THD_TEMP(sensor));
		u32 fall = readl(base + TMU_REG_THD_TEMP(sensor) + TMU_THD_FALL_OFFSET);
		u32 inten = readl(base + TMU_REG_INTEN(sensor));
		u32 rise7 = (rise >> TMU_RISE7_SHIFT) & TMU_TEMP_MASK;
		u32 fall7 = (fall >> TMU_RISE7_SHIFT) & TMU_TEMP_MASK;
		int rise7_c = acpm_tmu_decode(rise7, trim, cal_type, efuse, first, second);
		int fall7_c = acpm_tmu_decode(fall7, trim, cal_type, efuse, first, second);

		if (rise7 && rise7_c != TMU_NO_TEMP &&
		    (s->rise7_min_c == TMU_NO_TEMP || rise7_c < s->rise7_min_c))
			s->rise7_min_c = rise7_c;
		dev_info(s->dev, "acpm-tmu: %s sensor %u TRIMINFO=%#010x THD_RISE=%#010x rise7=%u (%d C) THD_FALL=%#010x fall7=%u (%d C) INTEN=%#010x RISE7_EN=%u\n",
			 s->name, sensor, trim, rise, rise7, rise7_c, fall, fall7,
			 fall7_c, inten, !!(inten & TMU_INTEN_RISE7));
	}
}

/*
 * Placeholder cooling device (DT samsung,stub-cooling-device = "<type>").
 * 4.9 registered isp_cooling as "thermal-isp-0"; the vendor thermal HAL
 * (thermal_info_config.json) lists it and aborts if no cooling device of
 * that type exists. There is no ISP driver on 6.12, so the stub has a
 * single state 0 ("no throttling"), refuses any other state and touches no
 * hardware. Nothing binds it to a zone (no cooling-maps reference it).
 */
static int acpm_tmu_stub_cdev_get_max_state(struct thermal_cooling_device *cdev,
					    unsigned long *state)
{
	*state = 0;
	return 0;
}

static int acpm_tmu_stub_cdev_get_cur_state(struct thermal_cooling_device *cdev,
					    unsigned long *state)
{
	*state = 0;
	return 0;
}

static int acpm_tmu_stub_cdev_set_cur_state(struct thermal_cooling_device *cdev,
					    unsigned long state)
{
	return state ? -EINVAL : 0;
}

static const struct thermal_cooling_device_ops acpm_tmu_stub_cdev_ops = {
	.get_max_state = acpm_tmu_stub_cdev_get_max_state,
	.get_cur_state = acpm_tmu_stub_cdev_get_cur_state,
	.set_cur_state = acpm_tmu_stub_cdev_set_cur_state,
};

static void acpm_tmu_register_stub_cdev(struct device *dev, struct device_node *np)
{
	struct thermal_cooling_device *cdev;
	const char *type;

	if (of_property_read_string(np, "samsung,stub-cooling-device", &type))
		return;
	cdev = devm_thermal_of_cooling_device_register(dev, NULL, type, NULL,
						       &acpm_tmu_stub_cdev_ops);
	if (IS_ERR(cdev))
		dev_warn(dev, "stub cooling device %s: %ld\n", type, PTR_ERR(cdev));
	else
		dev_info(dev, "stub cooling device %s = cooling_device%d, max_state 0 (no driver behind it, never throttles)\n",
			 type, cdev->id);
}

static int acpm_tmu_sensor_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct acpm_tmu_sensor *s;
	struct resource *res;
	void __iomem *base;
	bool ready;
	int ret;

	mutex_lock(&acpm_tmu_state_lock);
	ret = acpm_tmu.dev ? 0 : -EPROBE_DEFER;
	ready = acpm_tmu.ready;
	mutex_unlock(&acpm_tmu_state_lock);
	if (ret)
		return ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->dev = dev;
	if (of_property_read_u32(np, "id", &s->id) || s->id > 0xff)
		return dev_err_probe(dev, -EINVAL, "missing or invalid id\n");
	if (of_property_read_u32(np, "sensors", &s->sensors) || !s->sensors ||
	    s->sensors >= BIT(TMU_TOTAL_SENSORS))
		return dev_err_probe(dev, -EINVAL, "missing or invalid sensors mask\n");
	if (of_property_read_string(np, "tmu_name", &s->name))
		s->name = dev_name(dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || resource_size(res) < TMU_REG_EXTENT)
		return dev_err_probe(dev, -EINVAL, "TMU register window too small\n");
	/* BIG/LITTLE and G3D/ISP share one TMU block: map, do not claim. */
	base = devm_ioremap(dev, res->start, TMU_REG_EXTENT);
	if (!base)
		return -ENOMEM;
	acpm_tmu_report_hw_trip(s, base);
	devm_iounmap(dev, base);

	if (ready && s->core_en) {
		s->tzd = devm_thermal_of_zone_register(dev, 0, s, &acpm_tmu_ops);
		if (IS_ERR(s->tzd))
			return dev_err_probe(dev, PTR_ERR(s->tzd),
					     "%s: thermal zone registration failed\n", s->name);
		dev_info(dev, "%s tzid=%u thermal zone registered\n", s->name, s->id);
		acpm_tmu_parity_init(s);
	} else {
		s->tzd = NULL;
		dev_err(dev, "%s tzid=%u thermal zone NOT registered: %s\n", s->name, s->id,
			!ready ? "ACPM TMU not initialised" : "TMU CORE_EN is clear");
	}

	acpm_tmu_register_stub_cdev(dev, np);

	mutex_lock(&acpm_tmu_state_lock);
	if (acpm_tmu.num_sensors < ACPM_TMU_MAX_SENSORS)
		acpm_tmu.sensors[acpm_tmu.num_sensors++] = s;
	mutex_unlock(&acpm_tmu_state_lock);
	return 0;
}

static const struct of_device_id acpm_tmu_core_match[] = {
	{ .compatible = "samsung,exynos9810-acpm-tmu" },
	{ }
};

static const struct of_device_id acpm_tmu_sensor_match[] = {
	{ .compatible = "samsung,exynos9810-tmu" },
	{ }
};

static struct platform_driver acpm_tmu_core_driver = {
	.probe = acpm_tmu_core_probe,
	.driver = {
		.name = "exynos9810-acpm-tmu",
		.of_match_table = acpm_tmu_core_match,
		.suppress_bind_attrs = true,
	},
};

static struct platform_driver acpm_tmu_sensor_driver = {
	.probe = acpm_tmu_sensor_probe,
	.driver = {
		.name = "exynos9810-tmu",
		.of_match_table = acpm_tmu_sensor_match,
		.suppress_bind_attrs = true,
	},
};

static int __init acpm_tmu_init(void)
{
	int ret;

	ret = platform_driver_register(&acpm_tmu_core_driver);
	if (ret)
		return ret;
	ret = platform_driver_register(&acpm_tmu_sensor_driver);
	if (ret)
		platform_driver_unregister(&acpm_tmu_core_driver);
	return ret;
}
device_initcall(acpm_tmu_init);
