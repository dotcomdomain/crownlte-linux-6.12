// SPDX-License-Identifier: GPL-2.0-only
/* Original QoS-only Exynos9810 bus policy, using real Hz devfreq providers. */
#include <linux/device.h>
#include <linux/devfreq.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/lockdep.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include "exynos9810-bus-tables.h"
#include "exynos9810-interactive.h"
#include "governor.h"

struct interactive_state {
	struct devfreq *df;
	struct delayed_work work;
	struct exynos9810_interactive_policy policy;
	bool frozen;
};

/* PM events may overlap a sysfs governor switch. Keep state alive through
 * synchronous work cancellation, including the unlocked portion of STOP.
 */
static DEFINE_MUTEX(interactive_events_lock);

static void interactive_work(struct work_struct *work)
{
	struct interactive_state *state = container_of(to_delayed_work(work),
						struct interactive_state, work);
	struct devfreq *df = state->df;
	int ret = 0;

	mutex_lock(&df->lock);
	if (df->governor_data == state && !state->frozen)
		ret = update_devfreq(df);
	mutex_unlock(&df->lock);
	if (ret)
		dev_err_ratelimited(&df->dev, "interactive update failed: %d\n", ret);
}

static int interactive_target(struct devfreq *df, unsigned long *freq)
{
	const struct exynos9810_interactive_config *config = df->data;
	struct interactive_state *state = df->governor_data;
	struct dev_pm_opp *opp;
	unsigned long min_hz, cap_hz, actual_hz, wait;
	s32 floor_khz;
	int ret;

	lockdep_assert_held(&df->lock);
	if (!state || !config)
		return -ENODATA;
	ret = exynos9810_interactive_validate(config);
	if (ret)
		return ret;
	ret = df->profile->get_cur_freq(df->dev.parent, &actual_hz);
	if (ret)
		return ret;
	if (!actual_hz)
		return -EIO;
	floor_khz = dev_pm_qos_read_value(df->dev.parent, DEV_PM_QOS_MIN_FREQUENCY);
	if (floor_khz < 0)
		return -ERANGE;
	devfreq_get_freq_range(df, &min_hz, &cap_hz);
	/* The original governor floors the maximum cap to an available OPP before
	 * testing the decrease delay, including the cap == actual case.
	 */
	opp = devfreq_recommended_opp(df->dev.parent, &cap_hz,
				    DEVFREQ_FLAG_LEAST_UPPER_BOUND);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	dev_pm_opp_put(opp);
	*freq = exynos9810_interactive_decide(config, &state->policy,
			actual_hz, (unsigned long)floor_khz * 1000UL,
			cap_hz, jiffies, state->frozen, &wait);
	if (wait && !state->frozen)
		mod_delayed_work_on(0, system_wq, &state->work, wait);
	else
		cancel_delayed_work(&state->work);
	return 0;
}

static int interactive_start(struct devfreq *df)
{
	const struct exynos9810_interactive_config *config = df->data;
	struct interactive_state *state;
	unsigned long actual_hz;
	u32 id;
	int ret;

	mutex_lock(&df->lock);
	/* Other governors may leave their freed governor_data pointer behind. */
	df->governor_data = NULL;
	if (!df->profile->legacy_khz_sysfs || !df->profile->get_cur_freq ||
	    of_property_read_u32(df->dev.parent->of_node, "samsung,cal-id", &id) ||
	    (id != EXYNOS9810_MIF_CAL_ID && id != EXYNOS9810_INT_CAL_ID &&
	     id != EXYNOS9810_FSYS0_CAL_ID)) {
		ret = -EINVAL;
		goto out;
	}
	ret = exynos9810_interactive_validate(config);
	if (ret)
		goto out;
	ret = df->profile->get_cur_freq(df->dev.parent, &actual_hz);
	if (ret || !actual_hz) {
		ret = ret ? ret : -EIO;
		goto out;
	}
	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state) {
		ret = -ENOMEM;
		goto out;
	}
	state->df = df;
	INIT_DELAYED_WORK(&state->work, interactive_work);
	state->policy.previous_hz = actual_hz;
	state->policy.changed = jiffies;
	state->policy.expires = jiffies;
	df->governor_data = state;
	df->stop_polling = false;
out:
	mutex_unlock(&df->lock);
	return ret;
}

static void interactive_stop(struct devfreq *df)
{
	struct interactive_state *state;

	mutex_lock(&df->lock);
	state = df->governor_data;
	if (state) {
		state->frozen = true;
		df->governor_data = NULL;
		df->stop_polling = true;
	}
	mutex_unlock(&df->lock);
	if (state) {
		/* All ACK STOP call sites release df->lock before the callback. */
		cancel_delayed_work_sync(&state->work);
		kfree(state);
	}
}

static int interactive_event(struct devfreq *df, unsigned int event, void *data)
{
	struct interactive_state *state;
	unsigned long actual_hz;
	int ret = 0;

	lockdep_assert_not_held(&df->lock);
	mutex_lock(&interactive_events_lock);
	switch (event) {
	case DEVFREQ_GOV_START:
		ret = interactive_start(df);
		break;
	case DEVFREQ_GOV_STOP:
		interactive_stop(df);
		break;
	case DEVFREQ_GOV_SUSPEND:
		mutex_lock(&df->lock);
		state = df->governor_data;
		if (state) {
			state->frozen = true;
			df->stop_polling = true;
		}
		mutex_unlock(&df->lock);
		if (state)
			cancel_delayed_work_sync(&state->work);
		break;
	case DEVFREQ_GOV_RESUME:
		mutex_lock(&df->lock);
		state = df->governor_data;
		if (!state) {
			ret = -ENODATA;
			goto out;
		}
		ret = df->profile->get_cur_freq(df->dev.parent, &actual_hz);
		if (ret || !actual_hz) {
			ret = ret ? ret : -EIO;
			goto out;
		}
		state->policy.previous_hz = actual_hz;
		state->policy.changed = jiffies;
		state->policy.expires = jiffies;
		state->frozen = false;
		df->stats.last_update = get_jiffies_64();
		df->stop_polling = false;
		ret = update_devfreq(df);
out:
		mutex_unlock(&df->lock);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}
	mutex_unlock(&interactive_events_lock);
	return ret;
}

static struct devfreq_governor interactive_governor = {
	.name = "interactive",
	.flags = DEVFREQ_GOV_FLAG_IRQ_DRIVEN,
	.get_target_freq = interactive_target,
	.event_handler = interactive_event,
};

static int __init interactive_init(void)
{
	return devfreq_add_governor(&interactive_governor);
}
subsys_initcall(interactive_init);

MODULE_DESCRIPTION("Exynos9810 QoS-only interactive bus governor");
MODULE_LICENSE("GPL");
