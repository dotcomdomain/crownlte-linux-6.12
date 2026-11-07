/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Exynos 9810 PCIe RC control API for the BCM4361 Wi-Fi driver (crownlte).
 *
 * The function names and semantics follow the crownlte 4.9 Samsung driver
 * (drivers/pci/host/pci-exynos.c): the Wi-Fi platform code powers the
 * endpoint (WLAN_REG_ON) and then asks the RC to train the link
 * (exynos_pcie_pm_resume) or to take it down (exynos_pcie_pm_suspend).
 */
#ifndef __EXYNOS_PCIE_CTRL_H
#define __EXYNOS_PCIE_CTRL_H

#include <linux/errno.h>
#include <linux/types.h>

#define PCIE_L1SS_CTRL_ARGOS            (0x1 << 0)
#define PCIE_L1SS_CTRL_BOOT             (0x1 << 1)
#define PCIE_L1SS_CTRL_CAMERA           (0x1 << 2)
#define PCIE_L1SS_CTRL_MODEM_IF         (0x1 << 3)
#define PCIE_L1SS_CTRL_WIFI             (0x1 << 4)
#define PCIE_L1SS_CTRL_MST              (0x1 << 5)

/* Read-only snapshot for diagnostics (wifi selftest). */
struct exynos_pcie_status {
	bool probed;
	bool host_registered;
	int state;		/* 0 down, 1 up-try, 2 down-try, 3 up */
	u32 ltssm;		/* ELBI RDLH_LINKUP[4:0] at the time of the call */
	u32 speed;		/* negotiated GEN (LNKSTA.CLS), 0 when down */
	u32 width;		/* negotiated width (LNKSTA.NLW), 0 when down */
	u32 poweron_ok;
	u32 poweron_fail;
	u32 poweroff;
	u32 train_retries;	/* LTSSM retries summed over all power-ons */
	u32 linkdown_irqs;
	u32 l1ss_requests;	/* exynos_pcie_l1ss_ctrl() calls (ignored) */
	u32 rc_vendor_device;	/* root port vendor | device << 16 */
};

#if IS_ENABLED(CONFIG_PCIE_EXYNOS9810)
int exynos_pcie_pm_resume(int ch_num);
void exynos_pcie_pm_suspend(int ch_num);
int exynos_pcie_l1ss_ctrl(int enable, int id);
int exynos_pcie_l1_exit(int ch_num);
int exynos_pcie_get_status(int ch_num, struct exynos_pcie_status *st);
#else
static inline int exynos_pcie_l1_exit(int ch_num) { return 0; }
static inline int exynos_pcie_pm_resume(int ch_num) { return -ENODEV; }
static inline void exynos_pcie_pm_suspend(int ch_num) { }
static inline int exynos_pcie_l1ss_ctrl(int enable, int id) { return 0; }
static inline int exynos_pcie_get_status(int ch_num,
					 struct exynos_pcie_status *st)
{
	return -ENODEV;
}
#endif

#endif /* __EXYNOS_PCIE_CTRL_H */
