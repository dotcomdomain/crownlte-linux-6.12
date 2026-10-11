/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef EXYNOS9810_INTERACTIVE_H
#define EXYNOS9810_INTERACTIVE_H

#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/limits.h>
#include <linux/types.h>

#define EXYNOS9810_DELAY_TOKENS 33
#define EXYNOS9810_DELAY_MS_MAX (INT_MAX - 9)

/* Provider-owned config passed in df->data, immutable while df->lock is not held.
 * Tokens retain the old ABI: delay_ms threshold_khz:delay_ms ... . The provider
 * owns interactive/ sysfs so configuration survives governor switches.
 */
struct exynos9810_interactive_config {
	bool use_delay_time;
	unsigned int count;
	unsigned int tokens[EXYNOS9810_DELAY_TOKENS];
};

struct exynos9810_interactive_policy {
	unsigned long previous_hz;
	unsigned long changed;
	unsigned long expires;
};

static inline int exynos9810_interactive_validate(
		const struct exynos9810_interactive_config *config)
{
	unsigned int i;

	if (!config || config->count > EXYNOS9810_DELAY_TOKENS ||
	    (config->count && !(config->count & 1)) ||
	    (config->use_delay_time && !config->count))
		return -EINVAL;
	for (i = 0; i < config->count; i++) {
		if (!(i & 1)) {
			if (config->tokens[i] > EXYNOS9810_DELAY_MS_MAX)
				return -ERANGE;
		} else if (!config->tokens[i] || config->tokens[i] > INT_MAX ||
			   config->tokens[i] > ULONG_MAX / 1000UL ||
			   (i > 1 && config->tokens[i] <= config->tokens[i - 2])) {
			return -EINVAL;
		}
	}
	return 0;
}

/* Parse a counted sysfs buffer without depending on a terminating NUL. */
static inline int exynos9810_interactive_parse_delay(const char *buf, size_t len,
		struct exynos9810_interactive_config *config)
{
	struct exynos9810_interactive_config parsed = { };
	size_t pos = 0;
	unsigned int value;
	int ret;

	if (!buf || !config)
		return -EINVAL;
	parsed.use_delay_time = config->use_delay_time;
	while (pos < len) {
		while (pos < len && (buf[pos] == ' ' || buf[pos] == '\t' ||
		       buf[pos] == '\n' || buf[pos] == ':'))
			pos++;
		if (pos == len)
			break;
		if (parsed.count == EXYNOS9810_DELAY_TOKENS ||
		    buf[pos] < '0' || buf[pos] > '9')
			return -EINVAL;
		value = 0;
		while (pos < len && buf[pos] >= '0' && buf[pos] <= '9') {
			if (value > (UINT_MAX - (buf[pos] - '0')) / 10)
				return -ERANGE;
			value = value * 10 + buf[pos++] - '0';
		}
		if (pos < len && buf[pos] != ' ' && buf[pos] != '\t' &&
		    buf[pos] != '\n' && buf[pos] != ':')
			return -EINVAL;
		parsed.tokens[parsed.count++] = value;
	}
	if (!parsed.count)
		return -EINVAL;
	ret = exynos9810_interactive_validate(&parsed);
	if (!ret)
		*config = parsed;
	return ret;
}

/* Caller validates config and serializes this state with df->lock. */
static inline unsigned long exynos9810_interactive_decide(
		const struct exynos9810_interactive_config *config,
		struct exynos9810_interactive_policy *policy,
		unsigned long actual_hz, unsigned long wanted_hz,
		unsigned long cap_hz, unsigned long now, bool frozen,
		unsigned long *wait)
{
	unsigned int i = 0, delay;
	unsigned long result = wanted_hz;

	*wait = 0;
	if (!config->use_delay_time || frozen)
		goto out;
	if (policy->previous_hz != actual_hz) {
		while (i + 1 < config->count &&
		       wanted_hz >= config->tokens[i + 1] * 1000UL)
			i += 2;
		delay = config->tokens[i];
		delay = ((delay + 9) / 10) * 10;
		policy->expires = policy->changed + msecs_to_jiffies(delay);
	}
	/* Equality also bypasses the old delay. Resolve cap to a real OPP first. */
	if (cap_hz > actual_hz && wanted_hz < actual_hz &&
	    time_before(now, policy->expires)) {
		result = actual_hz;
		*wait = policy->expires - now;
	} else {
		policy->changed = now;
	}
out:
	policy->previous_hz = actual_hz;
	return result < cap_hz ? result : cap_hz;
}

#endif /* EXYNOS9810_INTERACTIVE_H */
