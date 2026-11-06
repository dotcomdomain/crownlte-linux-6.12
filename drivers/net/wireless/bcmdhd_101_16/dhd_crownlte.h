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

/* dhd_crownlte_selftest.c */
void dhd_crownlte_selftest_arm(struct device *dev, u32 delay_ms, bool keep_up);

/* wl_cfgscan.c: one firmware escan without a cfg80211 request */
#define DHD_CROWNLTE_SCAN_TOP	5
struct dhd_crownlte_scan_report {
	int ret;
	bool timed_out;
	bool preempted;
	u32 channels;		/* channels requested */
	u32 elapsed_ms;
	u32 count;		/* BSS entries in the escan buffer */
	u32 n2g;
	u32 n5g;
	u32 ntop;
	u8 top_oui[DHD_CROWNLTE_SCAN_TOP][3];
	s16 top_rssi[DHD_CROWNLTE_SCAN_TOP];
	u16 top_chan[DHD_CROWNLTE_SCAN_TOP];
};

int wl_cfgscan_crownlte_selftest_scan(struct net_device *ndev,
	unsigned int timeout_ms, struct dhd_crownlte_scan_report *rep);

#endif /* _DHD_CROWNLTE_H_ */
