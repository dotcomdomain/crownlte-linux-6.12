/* SPDX-License-Identifier: GPL-2.0 */
/*
 * crownlte (Galaxy Note9) platform glue and stage-1 self-test for bcmdhd
 * 101.16 on Linux 6.12. See Documentation/crownlte/wifi-port.md.
 */
#ifndef _DHD_CROWNLTE_H_
#define _DHD_CROWNLTE_H_

#include <linux/types.h>

struct device;
struct net_device;

/* dhd_custom_exynos.c */
struct dhd_crownlte_plat_status {
	bool probed;
	int probe_ret;
	int reg_on;		/* WLAN_REG_ON level, -1 if unknown */
	int host_wake;		/* WLAN_HOST_WAKE level, -1 if unknown */
	int host_wake_irq;
	u32 power_on;
	u32 power_off;
	u32 card_on;
	u32 card_off;
	int last_card_ret;
	int prealloc_ret;
};

bool dhd_crownlte_wlan_ready(void);
void dhd_crownlte_get_plat_status(struct dhd_crownlte_plat_status *st);

/* dhd_pcie.c */
struct dhd_pub;
bool dhd_bus_sih_ok(struct dhd_pub *dhdp);

/* dhd_crownlte_selftest.c */
void dhd_crownlte_selftest_arm(struct device *dev, u32 delay_ms, bool keep_up);

/* watch window: any cfg80211 scan/sched-scan/connect while armed = userspace took over */
void wl_cfgscan_crownlte_watch(bool on);
bool wl_cfgscan_crownlte_userspace_seen(void);

#endif /* _DHD_CROWNLTE_H_ */
