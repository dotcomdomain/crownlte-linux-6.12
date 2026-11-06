// SPDX-License-Identifier: GPL-2.0-only
/*
 * crownlte touch input booster: the 4.9 in-kernel input booster's TOUCH
 * boost, as CPU frequency QoS minimum requests.
 *
 * 4.9 (drivers/input/input.c without CONFIG_INPUT_BOOSTER, DT input_booster
 * "TOUCH", default level 2): on touch-down big >= 1066 MHz and LITTLE >=
 * 832 MHz for 130 ms (head); then big >= 858 MHz and LITTLE >= 832 MHz while
 * the finger stays down and for 500 ms after it lifts (tail). 4.9 also set
 * an EMS top-app boost (hmp_boost); that is not ported here.
 *
 * DT (all levels in kHz, one per samsung,cpus entry):
 *	input-booster {
 *		compatible = "samsung,crownlte-input-booster";
 *		samsung,cpus = <&cpu4 &cpu0>;
 *		samsung,touch-head = <130 1066000 832000>;	ms, kHz...
 *		samsung,touch-tail = <500 858000 832000>;	optional
 *		samsung,booster-selftest-ms = <95000>;		optional
 *	};
 *
 * A minimum request never raises a policy maximum: thermal and user caps
 * still win. Levels above a policy's cpuinfo.max_freq are refused.
 * Touchscreens are recognised by BTN_TOUCH + ABS_MT_POSITION_X; only
 * BTN_TOUCH is looked at.
 */
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define BOOSTER_MAX_TARGETS	2
#define BOOSTER_MAX_MS		5000

enum booster_phase { PHASE_IDLE, PHASE_HEAD, PHASE_TAIL };

struct booster_target {
	int cpu;
	u32 head_khz, tail_khz;
	struct freq_qos_request req;
	unsigned int max_khz;		/* cpuinfo.max_freq when attached */
};

struct booster {
	struct device *dev;
	struct input_handler handler;
	struct notifier_block policy_nb;
	struct booster_target t[BOOSTER_MAX_TARGETS];
	unsigned int nt;
	u32 head_ms, tail_ms;

	/* event side (atomic context) -> work */
	atomic_t downs;
	atomic_t pressed;
	struct work_struct event_work;

	struct mutex lock;		/* phase, timers, requests */
	enum booster_phase phase;
	struct delayed_work head_work;
	struct delayed_work tail_work;
	unsigned long tail_deadline;	/* jiffies: release + tail_ms */
	unsigned long head_deadline;	/* jiffies: last touch-down + head_ms (lock) */
	bool first_logged;
	bool first_event_logged;

	u32 selftest_ms;
	struct delayed_work selftest_work;
};

static unsigned int booster_boosts;
module_param_named(boosts, booster_boosts, uint, 0444);
MODULE_PARM_DESC(boosts, "touch-down boosts started since boot");

/* lock held */
static void booster_set(struct booster *b, enum booster_phase phase)
{
	unsigned int i;

	b->phase = phase;
	for (i = 0; i < b->nt; i++) {
		struct booster_target *t = &b->t[i];
		s32 khz = FREQ_QOS_MIN_DEFAULT_VALUE;

		if (!freq_qos_request_active(&t->req))
			continue;
		if (phase == PHASE_HEAD)
			khz = t->head_khz;
		else if (phase == PHASE_TAIL)
			khz = t->tail_khz;
		freq_qos_update_request(&t->req, khz);
	}
}

static void booster_log_state(struct booster *b, const char *what)
{
	char buf[160];
	unsigned int i, len = 0;

	for (i = 0; i < b->nt; i++) {
		struct cpufreq_policy *p = cpufreq_cpu_get(b->t[i].cpu);

		len += scnprintf(buf + len, sizeof(buf) - len, " cpu%d:min=%u,cur=%u",
				 b->t[i].cpu, p ? p->min : 0, p ? p->cur : 0);
		if (p)
			cpufreq_cpu_put(p);
	}
	dev_info(b->dev, "%s phase=%d boosts=%u%s\n", what, b->phase, booster_boosts, buf);
}

static void booster_event_work(struct work_struct *work)
{
	struct booster *b = container_of(work, struct booster, event_work);
	int downs = atomic_xchg(&b->downs, 0);
	bool pressed;

	mutex_lock(&b->lock);
	pressed = atomic_read(&b->pressed);
	if (downs) {
		/* touch-down: (re)start the head */
		booster_boosts++;
		cancel_delayed_work(&b->tail_work);
		booster_set(b, PHASE_HEAD);
		b->head_deadline = jiffies + msecs_to_jiffies(b->head_ms);
		mod_delayed_work(system_highpri_wq, &b->head_work,
				 msecs_to_jiffies(b->head_ms));
		if (!b->first_logged) {
			b->first_logged = true;
			mutex_unlock(&b->lock);
			/* the policy update runs from the policy's work: give it time */
			usleep_range(5000, 6000);
			booster_log_state(b, "first touch boost (head)");
			return;
		}
	} else if (!pressed && b->phase == PHASE_TAIL) {
		mod_delayed_work(system_highpri_wq, &b->tail_work,
				 msecs_to_jiffies(b->tail_ms));
	}
	mutex_unlock(&b->lock);
}

static void booster_head_end(struct work_struct *work)
{
	struct booster *b = container_of(to_delayed_work(work), struct booster, head_work);

	mutex_lock(&b->lock);
	if (b->phase == PHASE_HEAD) {
		long left = (long)(b->head_deadline - jiffies);

		if (left > 0) {
			/* an older callback raced a new touch-down: keep the new head */
			mod_delayed_work(system_highpri_wq, &b->head_work, left);
			mutex_unlock(&b->lock);
			return;
		}
		if (b->tail_ms) {
			booster_set(b, PHASE_TAIL);
			if (!atomic_read(&b->pressed)) {
				/* released during the head: the tail ends tail_ms after the release */
				long left;

				smp_rmb();
				left = (long)(READ_ONCE(b->tail_deadline) - jiffies);

				mod_delayed_work(system_highpri_wq, &b->tail_work,
						 left > 0 ? left : 0);
			}
		} else {
			booster_set(b, PHASE_IDLE);
		}
	}
	mutex_unlock(&b->lock);
}

static void booster_tail_end(struct work_struct *work)
{
	struct booster *b = container_of(to_delayed_work(work), struct booster, tail_work);

	mutex_lock(&b->lock);
	if (b->phase == PHASE_TAIL && !atomic_read(&b->pressed)) {
		long left;

		smp_rmb();
		left = (long)(READ_ONCE(b->tail_deadline) - jiffies);
		if (left > 0)
			/* a newer release moved the deadline: wait for it */
			mod_delayed_work(system_highpri_wq, &b->tail_work, left);
		else
			booster_set(b, PHASE_IDLE);
	}
	mutex_unlock(&b->lock);
}

static void booster_touch(struct booster *b, bool down)
{
	if (!down) {
		/* deadline first, then the release it belongs to (paired smp_rmb) */
		WRITE_ONCE(b->tail_deadline, jiffies + msecs_to_jiffies(b->tail_ms));
		smp_wmb();
	}
	atomic_set(&b->pressed, down);
	if (down)
		atomic_inc(&b->downs);
	queue_work(system_highpri_wq, &b->event_work);
}

/* atomic context (dev->event_lock) */
static void booster_input_event(struct input_handle *handle, unsigned int type,
				unsigned int code, int value)
{
	struct booster *b = handle->handler->private;

	if (type != EV_KEY || code != BTN_TOUCH || value > 1)
		return;
	if (!b->first_event_logged) {
		b->first_event_logged = true;
		dev_info(b->dev, "first BTN_TOUCH %d from %s\n", value,
			 handle->dev->name ?: "?");
	}
	booster_touch(b, value);
}

static int booster_input_connect(struct input_handler *handler, struct input_dev *dev,
				 const struct input_device_id *id)
{
	struct booster *b = handler->private;
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;
	handle->dev = dev;
	handle->handler = handler;
	handle->name = "crownlte-input-booster";
	ret = input_register_handle(handle);
	if (ret)
		goto err_free;
	ret = input_open_device(handle);
	if (ret)
		goto err_unregister;
	dev_info(b->dev, "boosting touches of %s\n", dev->name ?: "?");
	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return ret;
}

static void booster_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

/* direct touchscreens: BTN_TOUCH and multitouch positions */
static const struct input_device_id booster_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_KEYBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) },
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] = BIT_MASK(ABS_MT_POSITION_X) },
	},
	{ }
};

/* ---- per-policy QoS requests follow the policies ---- */

static void booster_attach(struct booster *b, struct cpufreq_policy *policy)
{
	unsigned int i;
	int ret;

	mutex_lock(&b->lock);
	for (i = 0; i < b->nt; i++) {
		struct booster_target *t = &b->t[i];

		if (freq_qos_request_active(&t->req) ||
		    !cpumask_test_cpu(t->cpu, policy->related_cpus))
			continue;
		if (t->head_khz > policy->cpuinfo.max_freq ||
		    t->tail_khz > policy->cpuinfo.max_freq) {
			dev_err(b->dev, "policy%u: boost %u/%u kHz above cpuinfo max %u kHz: not boosting it\n",
				policy->cpu, t->head_khz, t->tail_khz,
				policy->cpuinfo.max_freq);
			continue;
		}
		ret = freq_qos_add_request(&policy->constraints, &t->req, FREQ_QOS_MIN,
					   FREQ_QOS_MIN_DEFAULT_VALUE);
		t->max_khz = policy->cpuinfo.max_freq;
		dev_info(b->dev, "policy%u (cpus %*pbl): head %u kHz, tail %u kHz (ret=%d)\n",
			 policy->cpu, cpumask_pr_args(policy->related_cpus),
			 t->head_khz, t->tail_khz, ret);
	}
	mutex_unlock(&b->lock);
}

static int booster_policy_notify(struct notifier_block *nb, unsigned long event, void *data)
{
	struct booster *b = container_of(nb, struct booster, policy_nb);
	struct cpufreq_policy *policy = data;
	unsigned int i;

	if (event == CPUFREQ_CREATE_POLICY) {
		booster_attach(b, policy);
	} else if (event == CPUFREQ_REMOVE_POLICY) {
		mutex_lock(&b->lock);
		for (i = 0; i < b->nt; i++)
			if (cpumask_test_cpu(b->t[i].cpu, policy->related_cpus) &&
			    freq_qos_request_active(&b->t[i].req))
				freq_qos_remove_request(&b->t[i].req);
		mutex_unlock(&b->lock);
	}
	return NOTIFY_OK;
}

/* ---- DT-gated self-test: one synthetic touch through the same path ---- */

static bool booster_check(struct booster *b, enum booster_phase want, const char *what)
{
	unsigned int i;
	bool ok = true;

	for (i = 0; i < b->nt; i++) {
		struct booster_target *t = &b->t[i];
		struct cpufreq_policy *p = cpufreq_cpu_get(t->cpu);
		u32 expect = want == PHASE_HEAD ? t->head_khz :
			     want == PHASE_TAIL ? t->tail_khz : 0;

		if (!p) {
			ok = false;
			continue;
		}
		if (want == PHASE_IDLE) {
			u32 boost = max(t->head_khz, t->tail_khz);

			/* released: nothing holds the policy minimum at the boost */
			if (boost > p->cpuinfo.min_freq && p->min >= boost)
				ok = false;
		} else if (p->min < expect || p->cur < expect) {
			ok = false;
		}
		cpufreq_cpu_put(p);
	}
	booster_log_state(b, what);
	return ok;
}

static void booster_selftest(struct work_struct *work)
{
	struct booster *b = container_of(to_delayed_work(work), struct booster, selftest_work);
	unsigned int pass = 0, steps = 0;

	dev_info(b->dev, "selftest: synthetic touch, head %u ms, tail %u ms\n",
		 b->head_ms, b->tail_ms);
	booster_touch(b, true);
	msleep(min(b->head_ms / 2, 50U));
	steps++;
	pass += booster_check(b, PHASE_HEAD, "selftest: head");
	booster_touch(b, false);
	if (b->tail_ms) {
		ktime_t rel = ktime_get();

		msleep(b->head_ms + 30);
		steps++;
		pass += booster_check(b, PHASE_TAIL, "selftest: tail");
		/* idle 60 ms after release + tail (the old head-relative tail was still on) */
		while (ktime_ms_delta(ktime_get(), rel) < b->tail_ms + 60)
			msleep(10);
	} else {
		msleep(b->head_ms + 100);
	}
	steps++;
	pass += booster_check(b, PHASE_IDLE, "selftest: released");
	dev_info(b->dev, "selftest: %s (%u/%u)\n", pass == steps ? "PASS" : "FAIL",
		 pass, steps);
}

static int booster_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	u32 head[1 + BOOSTER_MAX_TARGETS], tail[1 + BOOSTER_MAX_TARGETS] = { 0 };
	struct booster *b;
	unsigned int i;
	int n, ret;

	b = devm_kzalloc(dev, sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;
	b->dev = dev;
	n = of_count_phandle_with_args(np, "samsung,cpus", NULL);
	if (n <= 0 || n > BOOSTER_MAX_TARGETS)
		return dev_err_probe(dev, -EINVAL, "samsung,cpus: 1..%d CPUs\n",
				     BOOSTER_MAX_TARGETS);
	b->nt = n;
	if (of_property_read_u32_array(np, "samsung,touch-head", head, 1 + n))
		return dev_err_probe(dev, -EINVAL, "samsung,touch-head: <ms kHz...>\n");
	if (of_property_present(np, "samsung,touch-tail") &&
	    of_property_read_u32_array(np, "samsung,touch-tail", tail, 1 + n))
		return dev_err_probe(dev, -EINVAL, "samsung,touch-tail: <ms kHz...>\n");
	b->head_ms = head[0];
	b->tail_ms = tail[0];
	if (!b->head_ms || b->head_ms > BOOSTER_MAX_MS || b->tail_ms > BOOSTER_MAX_MS)
		return dev_err_probe(dev, -EINVAL, "head 1..%d ms, tail 0..%d ms\n",
				     BOOSTER_MAX_MS, BOOSTER_MAX_MS);
	for (i = 0; i < b->nt; i++) {
		struct device_node *cpu_np = of_parse_phandle(np, "samsung,cpus", i);

		b->t[i].cpu = cpu_np ? of_cpu_node_to_id(cpu_np) : -ENODEV;
		of_node_put(cpu_np);
		if (b->t[i].cpu < 0)
			return dev_err_probe(dev, -EINVAL, "samsung,cpus[%u]: not a CPU\n", i);
		b->t[i].head_khz = head[1 + i];
		b->t[i].tail_khz = b->tail_ms ? tail[1 + i] : 0;
	}
	mutex_init(&b->lock);
	atomic_set(&b->downs, 0);
	atomic_set(&b->pressed, 0);
	INIT_WORK(&b->event_work, booster_event_work);
	INIT_DELAYED_WORK(&b->head_work, booster_head_end);
	INIT_DELAYED_WORK(&b->tail_work, booster_tail_end);

	b->policy_nb.notifier_call = booster_policy_notify;
	ret = cpufreq_register_notifier(&b->policy_nb, CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		return dev_err_probe(dev, ret, "cpufreq policy notifier\n");
	for (i = 0; i < b->nt; i++) {
		struct cpufreq_policy *policy = cpufreq_cpu_get(b->t[i].cpu);

		if (policy) {
			booster_attach(b, policy);
			cpufreq_cpu_put(policy);
		}
	}

	b->handler.event = booster_input_event;
	b->handler.connect = booster_input_connect;
	b->handler.disconnect = booster_input_disconnect;
	b->handler.name = "crownlte-input-booster";
	b->handler.id_table = booster_ids;
	b->handler.private = b;
	ret = input_register_handler(&b->handler);
	if (ret) {
		cpufreq_unregister_notifier(&b->policy_nb, CPUFREQ_POLICY_NOTIFIER);
		return dev_err_probe(dev, ret, "input handler\n");
	}
	dev_info(dev, "touch boost: head %u ms, tail %u ms after release, %u cluster(s) (4.9 input_booster TOUCH level 2)\n",
		 b->head_ms, b->tail_ms, b->nt);

	if (!of_property_read_u32(np, "samsung,booster-selftest-ms", &b->selftest_ms)) {
		s64 now = ktime_to_ms(ktime_get_boottime());

		INIT_DELAYED_WORK(&b->selftest_work, booster_selftest);
		schedule_delayed_work(&b->selftest_work,
				      msecs_to_jiffies(max_t(s64, b->selftest_ms - now, 0)));
	}
	return 0;
}

static const struct of_device_id booster_match[] = {
	{ .compatible = "samsung,crownlte-input-booster" },
	{ }
};

/* built in, no remove: the policy notifier and timers stay for the boot */
static struct platform_driver booster_driver = {
	.probe = booster_probe,
	.driver = {
		.name = "crownlte-input-booster",
		.of_match_table = booster_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(booster_driver);
MODULE_DESCRIPTION("crownlte touch input booster (4.9 parity, freq QoS)");
MODULE_LICENSE("GPL");
