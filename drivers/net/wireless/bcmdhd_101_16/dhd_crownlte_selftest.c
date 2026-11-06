// SPDX-License-Identifier: GPL-2.0
/*
 * crownlte Wi-Fi stage-1 self-test ("wifi selftest:"), Linux 6.12.
 *
 * Armed only by the diagnostic DT ("samsung,wifi-selftest" on the
 * samsung,brcm-wlan node). It runs once, in its own kthread, at
 * "samsung,wifi-selftest-ms" after boot and only does what Android does when
 * Wi-Fi is switched on and scans, minus association:
 *
 *   1. platform: WLAN_REG_ON / WLAN_HOST_WAKE state (set at probe)
 *   2. PCIe: RC link state, GEN/width, counters (read only)
 *   3. endpoint: Broadcom function on the bus, BARs, driver bound
 *   4. chip: chipcommon chip id/rev read by DHD at attach (expect 0x4361)
 *   5. wlan0 up (dev_open): firmware + NVRAM + CLM download from /vendor via
 *      the firmware loader, dongle init, MAC read (cur_etheraddr / OTP)
 *   6. one passive escan (no probe requests), results summarised
 *   7. wlan0 down again (unless samsung,wifi-selftest-keep-up)
 *
 * It never associates, never writes NVRAM/OTP/MAC to the chip's flash (the
 * BCM4361 has none) or to any file, and leaves wlan0 alone when userspace
 * already brought it up. DHD itself is built without kernel file I/O
 * (see dhd.h, DHD_CROWNLTE_612).
 */

#include <linux/delay.h>
#include <linux/exynos-pci-ctrl.h>
#include <linux/kthread.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/pci.h>
#include <linux/rtnetlink.h>
#include <linux/sched/clock.h>
#include <linux/timekeeping.h>
#include <net/net_namespace.h>

#include <typedefs.h>
#include <linuxver.h>
#include <osl.h>
#include <bcmutils.h>
#include <bcmdevs.h>
#include <bcmdevs_legacy.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <dhd_bus.h>
#include <dhd_linux.h>
#include <dhd_linux_priv.h>
#include <net/cfg80211.h>
#include <dhd_crownlte.h>

#define WST "wifi selftest: "
#define ST_SCAN_TIMEOUT_MS	20000

extern dhd_pub_t *g_dhd_pub;
extern char fw_version[];
extern char clm_version[];

struct crownlte_selftest {
	struct device *dev;
	u32 at_ms;
	bool keep_up;
	bool no_fw;	/* samsung,wifi-selftest-no-firmware: stop after the chip id */
};

static struct crownlte_selftest st_cfg;
static bool st_armed;

static const char *st_state_name(int s)
{
	static const char * const n[] = { "down", "up-try", "down-try", "up" };

	return (s >= 0 && s < ARRAY_SIZE(n)) ? n[s] : "?";
}

static int crownlte_selftest_fn(void *arg)
{
	struct dhd_crownlte_plat_status ps;
	struct exynos_pcie_status rc;
	struct dhd_crownlte_scan_report sr;
	struct pci_dev *pdev = NULL;
	struct net_device *ndev = NULL;
	dhd_pub_t *dhdp;
	const char *fail = NULL;
	bool opened = false, shared = false, scanned = false, nofw = false;
	u64 t0;
	u32 chip = 0, rev = 0;
	int ret, i;

	memset(&sr, 0, sizeof(sr));
	while (!kthread_should_stop()) {
		u64 now = ktime_get_boottime_ns() / NSEC_PER_MSEC;

		if (now >= st_cfg.at_ms)
			break;
		msleep_interruptible(min_t(u64, st_cfg.at_ms - now, 1000));
	}
	if (kthread_should_stop())
		return 0;

	pr_info(WST "start at %llu ms (stage 1: power, link, chip id, firmware, wlan0, passive scan)\n",
		ktime_get_boottime_ns() / NSEC_PER_MSEC);

	/* 1. platform */
	dhd_crownlte_get_plat_status(&ps);
	pr_info(WST "platform probed=%d ret=%d WLAN_REG_ON=%d HOST_WAKE=%d irq=%d power on/off=%u/%u card on/off=%u/%u last_card_ret=%d prealloc=%d\n",
		ps.probed, ps.probe_ret, ps.reg_on, ps.host_wake, ps.host_wake_irq,
		ps.power_on, ps.power_off, ps.card_on, ps.card_off,
		ps.last_card_ret, ps.prealloc_ret);
	if (!ps.probed || ps.probe_ret) {
		fail = "platform";
		goto out;
	}

	/* 2. PCIe RC */
	ret = exynos_pcie_get_status(0, &rc);
	pr_info(WST "pcie ret=%d host=%d state=%s ltssm=0x%02x GEN%u x%u rc=%04x:%04x poweron ok/fail=%u/%u poweroff=%u train_retries=%u linkdown_irqs=%u l1ss_req=%u\n",
		ret, rc.host_registered, st_state_name(rc.state), rc.ltssm,
		rc.speed, rc.width, rc.rc_vendor_device & 0xffff,
		rc.rc_vendor_device >> 16, rc.poweron_ok, rc.poweron_fail,
		rc.poweroff, rc.train_retries, rc.linkdown_irqs, rc.l1ss_requests);
	if (ret || !rc.host_registered || !rc.poweron_ok) {
		fail = "pcie";
		goto out;
	}

	/* 3. endpoint */
	pdev = pci_get_device(PCI_VENDOR_ID_BROADCOM, PCI_ANY_ID, NULL);
	if (!pdev) {
		pr_info(WST "endpoint: no Broadcom function on the bus\n");
		fail = "endpoint";
		goto out;
	}
	pr_info(WST "endpoint %s %04x:%04x rev %02x class %06x irq %d bar0 %pR bar2 %pR driver %s\n",
		pci_name(pdev), pdev->vendor, pdev->device, pdev->revision,
		pdev->class, pdev->irq, &pdev->resource[0], &pdev->resource[2],
		pdev->dev.driver ? pdev->dev.driver->name : "(none)");
	if (!pdev->dev.driver) {
		fail = "endpoint-driver";
		goto out;
	}

	/* 4. chip id (read by si_attach() at DHD attach, no new access) */
	dhdp = g_dhd_pub;
	if (!dhdp || !dhdp->bus) {
		pr_info(WST "chip: DHD not attached\n");
		fail = "dhd-attach";
		goto out;
	}
	chip = dhd_bus_chip_id(dhdp);
	rev = dhd_bus_chiprev_id(dhdp);
	pr_info(WST "chip 0x%04x rev %u pkg %u busstate %d up %d\n", chip, rev,
		dhd_bus_chippkg_id(dhdp), dhdp->busstate, dhdp->up);
	if (chip != BCM4361_CHIP_ID) {
		fail = "chip-id";
		goto out;
	}
	if (st_cfg.no_fw) {
		/* kernel-only vehicles: /vendor/firmware is not reachable */
		nofw = true;
		goto out;
	}

	/* 5. wlan0 up: firmware download and dongle init */
	ndev = dev_get_by_name(&init_net, "wlan0");
	if (!ndev) {
		pr_info(WST "no wlan0\n");
		fail = "wlan0";
		goto out;
	}
	/* check and open atomically under RTNL: never adopt a userspace-owned wlan0 */
	t0 = ktime_get_boottime_ns();
	rtnl_lock();
	if (ndev->flags & IFF_UP) {
		shared = true;
		ret = 0;
	} else {
		wl_cfgscan_crownlte_watch(true);
		ret = dev_open(ndev, NULL);
		if (!ret)
			opened = true;
		else
			wl_cfgscan_crownlte_watch(false);
	}
	rtnl_unlock();
	if (shared) {
		pr_info(WST "wlan0 already up (userspace owns it): no bring-up, no scan\n");
	} else {
		pr_info(WST "wlan0 up ret=%d in %llu ms\n", ret,
			(ktime_get_boottime_ns() - t0) / NSEC_PER_MSEC);
		if (ret) {
			fail = "firmware/bring-up";
			goto out;
		}
	}

	{
		dhd_info_t *dhd = (dhd_info_t *)dhdp->info;
		const u8 *m = dhdp->mac.octet;

		pr_info(WST "fw \"%s\" clm \"%s\"\n", fw_version, clm_version);
		pr_info(WST "fw_path %s nv_path %s\n", dhd ? dhd->fw_path : "?",
			dhd ? dhd->nv_path : "?");
		pr_info(WST "mac oui %02x:%02x:%02x locally-administered=%d (read from chip, not stored)\n",
			m[0], m[1], m[2], !!(m[0] & 0x02));
	}

	/* 6. one passive escan, never handed to cfg80211 */
	if (!shared) {
		ret = wl_cfgscan_crownlte_selftest_scan(ndev, ST_SCAN_TIMEOUT_MS, &sr);
		pr_info(WST "scan ret=%d status=%d channels=%u elapsed=%u ms timed_out=%d preempted=%d bss=%u (2.4G %u, 5G %u)\n",
			sr.ret, sr.status, sr.channels, sr.elapsed_ms, sr.timed_out,
			sr.preempted, sr.count, sr.n2g, sr.n5g);
		for (i = 0; i < sr.ntop; i++)
			pr_info(WST "scan top%d oui %02x:%02x:%02x ch %u rssi %d\n",
				i + 1, sr.top_oui[i][0], sr.top_oui[i][1],
				sr.top_oui[i][2], sr.top_chan[i], sr.top_rssi[i]);
		if (ret) {
			fail = sr.preempted ? "scan-preempted" : "scan";
			goto out;
		}
		scanned = true;
	}

out:
	/* 7. leave wlan0 as we found it */
	if (opened) {
		struct wiphy *wiphy = ndev->ieee80211_ptr ? ndev->ieee80211_ptr->wiphy : NULL;
		bool user;

		/*
		 * cfg80211 ops (scan, connect, sched scan) run under the wiphy mutex,
		 * not RTNL (6.12 nl80211 drops RTNL first): decide with both held, then
		 * drop only the wiphy mutex for dev_close().
		 */
		rtnl_lock();
		if (wiphy)
			wiphy_lock(wiphy);
		user = wl_cfgscan_crownlte_userspace_seen() || sr.preempted;
		wl_cfgscan_crownlte_watch(false);
		if (wiphy)
			wiphy_unlock(wiphy);
		if (user)
			pr_info(WST "userspace used wlan0 during the test: left as is (flags 0x%x)\n", ndev->flags);
		else if (!st_cfg.keep_up && (ndev->flags & IFF_UP)) {
			dev_close(ndev);
			pr_info(WST "wlan0 down again (flags 0x%x)\n", ndev->flags);
		}
		rtnl_unlock();
	}
	if (ndev)
		dev_put(ndev);
	if (pdev)
		pci_dev_put(pdev);

	if (fail)
		pr_info(WST "RESULT FAIL stage=%s\n", fail);
	else if (nofw)
		pr_info(WST "RESULT PASS-NOFW chip=0x%04x rev=%u (power, link, enumeration, chip id; firmware step not requested)\n",
			chip, rev);
	else if (shared)
		pr_info(WST "RESULT PASS-SHARED chip=0x%04x rev=%u (wlan0 owned by userspace, scan skipped)\n",
			chip, rev);
	else if (scanned && sr.count == 0)
		pr_info(WST "RESULT PASS-NO-BSS chip=0x%04x rev=%u (scan completed, nothing heard)\n",
			chip, rev);
	else
		pr_info(WST "RESULT PASS chip=0x%04x rev=%u bss=%u\n", chip, rev, sr.count);
	return 0;
}

void dhd_crownlte_selftest_arm(struct device *dev, u32 delay_ms, bool keep_up)
{
	struct task_struct *t;

	if (st_armed)
		return;
	st_cfg.dev = dev;
	st_cfg.at_ms = delay_ms;
	st_cfg.keep_up = keep_up;
	st_cfg.no_fw = of_property_read_bool(dev->of_node,
		"samsung,wifi-selftest-no-firmware");
	t = kthread_run(crownlte_selftest_fn, NULL, "wifi_selftest");
	if (IS_ERR(t)) {
		pr_err(WST "cannot start kthread: %ld\n", PTR_ERR(t));
		return;
	}
	st_armed = true;
	pr_info(WST "armed for %u ms after boot%s%s\n", delay_ms,
		keep_up ? ", wlan0 kept up" : "",
		st_cfg.no_fw ? ", no firmware step" : "");
}
