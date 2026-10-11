/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef EXYNOS9810_LEGACY_QOS_H
#define EXYNOS9810_LEGACY_QOS_H

#include <linux/pm_qos.h>
#include <linux/types.h>

/* Provider-owned bridge storage. Detach before freeing it. The provider's
 * parent must remain registered until detach has finished. No frontend FD
 * points at this storage; all FDs own independent static-class requests.
 */
struct exynos9810_legacy_qos_bridge {
	struct device *dev;
	struct dev_pm_qos_request minimum, maximum;
	unsigned int first_class, maximum_khz;
	int (*apply)(void *context);
	void *context;
	bool minimum_attached, maximum_attached;
};

int exynos9810_legacy_qos_attach(struct exynos9810_legacy_qos_bridge *bridge,
		struct device *dev, unsigned int maximum_khz,
		int (*apply)(void *context), void *context);
void exynos9810_legacy_qos_detach(struct exynos9810_legacy_qos_bridge *bridge);
/* Drain frontend callbacks before unregistering the provider's devfreq class.
 * Keeps the native requests and class reservation until detach after teardown.
 */
void exynos9810_legacy_qos_stop(struct exynos9810_legacy_qos_bridge *bridge);

#endif
