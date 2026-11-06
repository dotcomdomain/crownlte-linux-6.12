// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos 9810 PCIe root complex, channel 0 (BCM4361 Wi-Fi on crownlte).
 *
 * Port of the Samsung 4.9 driver that runs on the phone today
 * (crownlte 4.9: drivers/pci/host/pci-exynos.c, pci-exynos9810_cal.c,
 * pci-exynos.h) onto the 6.12 DesignWare host core. The ELBI, PHY and PCS
 * sequences and values are the 4.9 CAL ones; only the plumbing is new.
 * Documentation/crownlte/wifi-port.md lists every deviation from 4.9.
 *
 * Model (same as 4.9): the RC driver prepares the PHY at probe and leaves
 * the link down. The Wi-Fi platform code raises WLAN_REG_ON and calls
 * exynos_pcie_pm_resume(0), which trains the link. On 6.12 the host bridge
 * is registered (and the endpoint enumerated) on the first successful
 * link-up instead of the 4.9 pci_rescan_bus(). Later power cycles restore
 * the root port state like 4.9 does.
 *
 * Not ported in stage 1 (see the doc): MSI, L1 substates/ASPM, the PCIe
 * SysMMU, cache-coherent DMA, CPU idle-IP voting, the pcie_sysfs test node
 * and channel 1.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/exynos-pci-ctrl.h>
#include <linux/exynos-pci-noti.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#include "../../pci.h"
#include "pcie-designware.h"

#define EX_MAX_CH			2
#define EX_TRAIN_POLLS			2000	/* x 10 us, 4.9 MAX_TIMEOUT */
#define EX_TRAIN_TRIES			10	/* 4.9 retry count */

/* ELBI (4.9 pci-exynos.h) */
#define ELBI_IRQ_PULSE			0x000
#define  IRQ_INTA_ASSERT		BIT(0)
#define  IRQ_INTB_ASSERT		BIT(2)
#define  IRQ_INTC_ASSERT		BIT(4)
#define  IRQ_INTD_ASSERT		BIT(6)
#define  IRQ_RADM_PM_TO_ACK		BIT(18)
#define  IRQ_LINK_DOWN			BIT(30)
#define ELBI_IRQ_LEVEL			0x004
#define ELBI_IRQ_SPECIAL		0x008
#define ELBI_IRQ_EN_PULSE		0x00c
#define  IRQ_LINKDOWN_ENABLE		BIT(30)
#define ELBI_IRQ_EN_LEVEL		0x010
#define ELBI_IRQ_EN_SPECIAL		0x014
#define ELBI_SW_WAKE			0x018
#define ELBI_APP_LTSSM_ENABLE		0x02c
#define ELBI_IRQ_TOE_EN_PULSE2		0x034
#define ELBI_IRQ_TOE_EN_LEVEL		0x038
#define ELBI_SUB_CON			0x034	/* 4.9 "PCIE_SUB_CON ... Enable TOE" */
#define ELBI_APP_REQ_EXIT_L1		0x040
#define ELBI_XMIT_PME_TURNOFF		0x04c
#define ELBI_RDLH_LINKUP		0x074
#define ELBI_PM_DSTATE			0x088
#define ELBI_HISTORY_REG(x)		(0x138 + ((x) * 4))
#define  HIST_LTSSM(x)			(((x) >> 16) & 0x3f)
#define  HIST_DSTATE(x)			(((x) >> 8) & 0x7)
#define  HIST_L1SUB(x)			((x) & 0x7)
#define ELBI_LINKDOWN_RST_CTRL_SEL	0x1b8
#define  LINKDOWN_RST_MANUAL		BIT(1)
#define ELBI_SOFT_CORE_RESET		0x1d0
#define ELBI_STATE_HISTORY_CHECK	0x274
#define  HISTORY_BUFFER_ENABLE		0x3
#define ELBI_PCS_G_RST			0x288
#define ELBI_PCS_PMA_RST		0x28c
#define ELBI_MAC_RST			0x290
#define ELBI_STATE_POWER_S		0x2bc
#define ELBI_STATE_POWER_M		0x2c0
#define ELBI_QCH_SEL			0x2c8
#define  CLOCK_GATING_PMU_MASK		(0xf << 8)
#define  CLOCK_GATING_APB_MASK		(0xf << 4)
#define  CLOCK_GATING_AXI_MASK		(0xf << 0)
#define ELBI_L12ERR_CTRL		0x2f0
#define  NCLK_OFF			BIT(2)
#define ELBI_APP_REQ_EXIT_L1_MODE	0x0f4
#define  APP_REQ_EXIT_L1_MODE		BIT(0)
#define  L1_REQ_NAK_CONTROL_MASTER	BIT(4)

#define LTSSM_MASK			0x1f
#define LTSSM_L0_FIRST			0x0d
#define LTSSM_L0_LAST			0x14	/* establish_link window */
#define LTSSM_UP_LAST			0x15	/* link_up window (incl. L2/3 ready) */
#define LTSSM_L23_READY			0x15

/* PCS (4.9 cal) */
#define PCS_RX_ELECIDLE_CH0		0x0ec	/* bit 3: 1 = ignore elecidle */
#define PCS_REFCLK_CTRL0		0x100
#define PCS_REFCLK_CTRL1		0x104
#define PCS_TX_LATENCY			0x0f8
#define PCS_L1SS_TIMEOUT		0x00c
#define PCS_CDR			0x0d0

/* PMU (4.9 pci-exynos.h) */
#define PMU_WAKEUP_MASK			0x610
#define  WAKEUP_MASK_PCIE_WIFI		5
#define PMU_PCIE_PHY_CONTROL		0x71c

/* DBI, 4.9 offsets (PCIe cap at 0x70, PM cap at 0x40) */
#define DBI_PM_CAP			0x40
#define DBI_EXP_CAP			0x70
#define DBI_AUX_CLK_FREQ		0xb40
#define  AUX_CLK_FREQ_26MHZ		0x1a
#define DBI_L1_SUBSTATES		0xb44
#define  L1_SUB_VAL			0xea
#define EX_RC_VENDOR			0x144d	/* PCI_VENDOR_ID_SAMSUNG */
#define EX_RC_DEVICE			0xecec	/* 4.9 PCI_DEVICE_ID_EXYNOS */

/* 4.9 I/A sequencer, absolute addresses of the blocks it touches */
#define IA_ELBI_PA			0x116a0000
#define IA_PMA_PA			0x116d0000
#define IA_PCS_PA			0x116c0000
#define IA_SELF_PA			0x11680000

enum ex_state {
	EX_LINK_DOWN = 0,
	EX_LINK_UP_TRY,
	EX_LINK_DOWN_TRY,
	EX_LINK_UP,
};

struct exynos9810_pcie {
	struct dw_pcie pci;
	struct device *dev;
	void __iomem *elbi;
	void __iomem *phy;
	void __iomem *pcs;
	void __iomem *phy_sysreg;
	void __iomem *ia;
	struct regmap *pmu;
	struct gpio_desc *perst;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_idle;
	int irq;
	bool irq_enabled;
	u32 ch;
	bool use_ia;

	struct mutex lock;		/* power on/off, host registration */
	spinlock_t reg_lock;		/* DBI access with link down */
	enum ex_state state;
	bool host_registered;
	bool was_up_at_suspend;
	struct pci_dev *root_port;
	struct pci_saved_state *rp_saved;

	struct exynos_pcie_register_event *event_reg;
	struct work_struct linkdown_work;

	u32 poweron_ok, poweron_fail, poweroff_cnt, train_retries;
	u32 linkdown_irqs, l1ss_requests;
};

#define to_ex(x) container_of(x, struct exynos9810_pcie, pci)

static struct exynos9810_pcie *g_ex[EX_MAX_CH];
static DEFINE_SPINLOCK(g_ex_lock);

static inline u32 ex_elbi_rd(struct exynos9810_pcie *ex, u32 reg)
{
	return readl(ex->elbi + reg);
}

static inline void ex_elbi_wr(struct exynos9810_pcie *ex, u32 val, u32 reg)
{
	writel(val, ex->elbi + reg);
}

static inline u32 ex_ltssm(struct exynos9810_pcie *ex)
{
	return ex_elbi_rd(ex, ELBI_RDLH_LINKUP) & LTSSM_MASK;
}

/* ---- PHY CAL (4.9 pci-exynos9810_cal.c, channel 0 only) ---- */

/* 4.9 exynos_phy_check_rx_elecidle(): 1 = IGNORE_ELECIDLE, 0 = ENABLE */
static void ex_phy_rx_elecidle(struct exynos9810_pcie *ex, int ignore)
{
	u32 v = readl(ex->pcs + PCS_RX_ELECIDLE_CH0);

	v &= ~BIT(3);
	v |= (ignore ? 1 : 0) << 3;
	writel(v, ex->pcs + PCS_RX_ELECIDLE_CH0);
}

/* 4.9 exynos_phy_all_pwrdn() for ch0 (SOCPLL_GATING not defined on 4.9) */
static void ex_phy_all_pwrdn(struct exynos9810_pcie *ex)
{
	u32 v;

	writel(0xfe, ex->phy + 0x57 * 4);	/* Trsv */
	writel(0xc1, ex->phy + 0x20 * 4);	/* Common */
	v = readl(ex->pcs + PCS_REFCLK_CTRL0);
	writel(v | BIT(6), ex->pcs + PCS_REFCLK_CTRL0);
	v = readl(ex->pcs + PCS_REFCLK_CTRL1);
	writel(v | BIT(7), ex->pcs + PCS_REFCLK_CTRL1);
}

/* 4.9 exynos_phy_all_pwrdn_clear() for ch0 */
static void ex_phy_all_pwrdn_clear(struct exynos9810_pcie *ex)
{
	u32 v;

	v = readl(ex->pcs + PCS_REFCLK_CTRL0);
	writel(v & ~BIT(6), ex->pcs + PCS_REFCLK_CTRL0);
	v = readl(ex->pcs + PCS_REFCLK_CTRL1);
	writel(v & ~BIT(7), ex->pcs + PCS_REFCLK_CTRL1);
	writel(0xc0, ex->phy + 0x20 * 4);	/* Common */
	writel(0x7e, ex->phy + 0x57 * 4);	/* Trsv */
}

/* 26 MHz, Gen2, ch0 (4.9 cmn_config_val / trsv_config_val) */
static const u8 ex_cmn_cfg[48] = {
	0x01, 0xE1, 0x05, 0x00, 0x88, 0x88, 0x88, 0x0C,
	0x61, 0x45, 0x65, 0x24, 0x33, 0x18, 0xE3, 0xFC,
	0xD8, 0x05, 0xE6, 0x80, 0x00, 0x00, 0x00, 0x00,
	0x60, 0x11, 0x00, 0xA0, 0x05, 0x04, 0x18, 0x88,
	0xC0, 0xFF, 0x9B, 0x52, 0x22, 0x30, 0x4F, 0xDC,
	0x40, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x80,
};

static const u8 ex_trsv_cfg[80] = {
	0x31, 0x40, 0x37, 0x99, 0x85, 0x00, 0xC0, 0xFF,
	0xFF, 0x3F, 0x8C, 0xC8, 0x02, 0x01, 0x88, 0x80,
	0x06, 0x90, 0x6C, 0x66, 0x09, 0x61, 0x42, 0x44,
	0xC6, 0x50, 0x0A, 0x33, 0x58, 0xE7, 0x20, 0x22,
	0x80, 0x38, 0x05, 0x85, 0x00, 0x00, 0x00, 0x7E,
	0x00, 0x00, 0x55, 0x15, 0xAC, 0xAA, 0x3E, 0x00,
	0x00, 0x00, 0x20, 0x3F, 0x00, 0x03, 0x01, 0x00,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
	0x05, 0x85, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static void ex_pulse_rst(struct exynos9810_pcie *ex, u32 reg, unsigned int last_us)
{
	ex_elbi_wr(ex, 1, reg);
	udelay(10);
	ex_elbi_wr(ex, 0, reg);
	udelay(10);
	ex_elbi_wr(ex, 1, reg);
	udelay(last_us);
}

/* 4.9 exynos_pcie_phy_config() for ch0. CONFIG_EXYNOS_OTP was off on 4.9. */
static void ex_phy_config(struct exynos9810_pcie *ex)
{
	void __iomem *sr = ex->phy_sysreg;
	u32 v;
	int i;

	/* Set PCS_HIGH_SPEED, lane 0 enable */
	writel(readl(sr) | BIT(1), sr);
	writel((((readl(sr + 0xc) & ~(0xf << 4)) & ~(0xf << 2)) | (0x3 << 2)) &
	       ~BIT(1), sr + 0xc);
	writel(readl(sr + 0xc) | BIT(12), sr + 0xc);

	ex_pulse_rst(ex, ELBI_PCS_G_RST, 10);

	for (i = 0; i < ARRAY_SIZE(ex_cmn_cfg); i++)
		writel(ex_cmn_cfg[i], ex->phy + i * 4);
	for (i = 0; i < ARRAY_SIZE(ex_trsv_cfg); i++)
		writel(ex_trsv_cfg[i], ex->phy + (0x30 + i) * 4);

	writel(0x70, ex->pcs + PCS_TX_LATENCY);
	writel(0x87, ex->pcs + PCS_REFCLK_CTRL0);
	writel(0x50, ex->pcs + PCS_REFCLK_CTRL1);
	v = readl(ex->pcs + PCS_L1SS_TIMEOUT);
	v &= ~BIT(1);
	v |= BIT(4);
	writel(v, ex->pcs + PCS_L1SS_TIMEOUT);

	ex_pulse_rst(ex, ELBI_MAC_RST, 10);
	ex_pulse_rst(ex, ELBI_PCS_PMA_RST, 100);

	/* CDR reset */
	v = readl(ex->pcs + PCS_CDR);
	v |= 0x3 << 6;
	writel(v, ex->pcs + PCS_CDR);
	udelay(20);
	v = readl(ex->pcs + PCS_CDR);
	v &= ~(0x3 << 6);
	v |= 0x2 << 6;
	writel(v, ex->pcs + PCS_CDR);
	v &= ~(0x3 << 6);
	writel(v, ex->pcs + PCS_CDR);
}

/* 4.9 exynos_pcie_assert_phy_reset() (phy clocks: none on crownlte) */
static void ex_assert_phy_reset(struct exynos9810_pcie *ex)
{
	void __iomem *ia = ex->ia;
	u32 v;

	ex_phy_config(ex);

	if (ex->use_ia) {
		/* L1.2-exit CDR reset sequencer, verbatim from 4.9 */
		ex_elbi_wr(ex, 0x10, ELBI_SUB_CON);
		writel(IA_ELBI_PA, ia + 0x30);
		writel(IA_PMA_PA, ia + 0x34);
		writel(IA_PCS_PA, ia + 0x38);
		writel(IA_SELF_PA, ia + 0x3c);
		writel(0xb, ia + 0x10);
		writel(0x2ffff, ia + 0x40);
		writel(0x50000004, ia + 0x100);
		writel(0xffff, ia + 0x104);
		writel(0x20930014, ia + 0x108);
		writel(0xa, ia + 0x10c);
		writel(0x40030044, ia + 0x110);
		writel(0x1, ia + 0x114);
		writel(0x50000004, ia + 0x118);
		writel(0x20, ia + 0x11c);
		writel(0x209101ec, ia + 0x120);
		writel(0x20, ia + 0x124);
		writel(0x400200d0, ia + 0x128);
		writel(0xc0, ia + 0x12c);
		writel(0x400200d0, ia + 0x130);
		writel(0x80, ia + 0x134);
		writel(0x400200d0, ia + 0x138);
		writel(0x0, ia + 0x13c);
		writel(0x100101ec, ia + 0x140);
		writel(0x20, ia + 0x144);
		writel(0x70000004, ia + 0x148);
		writel(0x10, ia + 0x14c);
		writel(0x40030010, ia + 0x150);
		writel(0x1000b, ia + 0x154);
		writel(0x80000000, ia + 0x158);
		writel(0x0, ia + 0x15c);
		writel(0x1, ia + 0x0);
	}

	/* Bus number enable */
	v = ex_elbi_rd(ex, ELBI_SW_WAKE);
	v &= ~BIT(1);
	ex_elbi_wr(ex, v, ELBI_SW_WAKE);
}

/* ---- DBI access: 4.9 rd/wr_own_conf wrapping while the link is not up ---- */

static void ex_dbi_open(struct exynos9810_pcie *ex)
{
	ex_elbi_wr(ex, 0, ELBI_L12ERR_CTRL);
	ex_phy_rx_elecidle(ex, 1);
}

static void ex_dbi_close(struct exynos9810_pcie *ex)
{
	ex_phy_rx_elecidle(ex, 0);
	ex_elbi_wr(ex, NCLK_OFF, ELBI_L12ERR_CTRL);
}

static u32 ex_read_dbi(struct dw_pcie *pci, void __iomem *base, u32 reg,
		       size_t size)
{
	struct exynos9810_pcie *ex = to_ex(pci);
	unsigned long flags;
	bool wrap;
	u32 val;

	spin_lock_irqsave(&ex->reg_lock, flags);
	wrap = ex->state != EX_LINK_UP;
	if (wrap)
		ex_dbi_open(ex);
	dw_pcie_read(base + reg, size, &val);
	if (wrap)
		ex_dbi_close(ex);
	spin_unlock_irqrestore(&ex->reg_lock, flags);
	return val;
}

static void ex_write_dbi(struct dw_pcie *pci, void __iomem *base, u32 reg,
			 size_t size, u32 val)
{
	struct exynos9810_pcie *ex = to_ex(pci);
	unsigned long flags;
	bool wrap;

	spin_lock_irqsave(&ex->reg_lock, flags);
	wrap = ex->state != EX_LINK_UP;
	if (wrap)
		ex_dbi_open(ex);
	dw_pcie_write(base + reg, size, val);
	if (wrap)
		ex_dbi_close(ex);
	spin_unlock_irqrestore(&ex->reg_lock, flags);
}

static int ex_link_up(struct dw_pcie *pci)
{
	struct exynos9810_pcie *ex = to_ex(pci);
	u32 l;

	if (ex->state != EX_LINK_UP)
		return 0;
	l = ex_ltssm(ex);
	return l >= LTSSM_L0_FIRST && l <= LTSSM_UP_LAST;
}

static const struct dw_pcie_ops ex_dw_ops = {
	.read_dbi = ex_read_dbi,
	.write_dbi = ex_write_dbi,
	.link_up = ex_link_up,
};

/* The link is already trained when dw_pcie_host_init() runs. */
static int ex_host_init(struct dw_pcie_rp *pp)
{
	return 0;
}

/* 4.9 runs ch0 with use-msi = "false": INTx only, no iMSI-RX. */
static int ex_msi_init(struct dw_pcie_rp *pp)
{
	return 0;
}

static const struct dw_pcie_host_ops ex_host_ops = {
	.init = ex_host_init,
	.msi_init = ex_msi_init,
};

/* ---- RC setup (4.9 exynos_pcie_setup_rc) ---- */

static void ex_setup_rc(struct exynos9810_pcie *ex)
{
	struct dw_pcie *pci = &ex->pci;
	u32 v;

	dw_pcie_dbi_ro_wr_en(pci);

	dw_pcie_writew_dbi(pci, PCI_VENDOR_ID, EX_RC_VENDOR);
	dw_pcie_writew_dbi(pci, PCI_DEVICE_ID, EX_RC_DEVICE + ex->ch);

	/* Gen2, x1, L1 exit latency 64 us */
	v = dw_pcie_readl_dbi(pci, DBI_EXP_CAP + PCI_EXP_LNKCAP);
	v &= ~(PCI_EXP_LNKCAP_L1EL | PCI_EXP_LNKCAP_MLW | PCI_EXP_LNKCAP_SLS);
	v |= (0x7 << 15) | (0x1 << 4) | PCI_EXP_LNKCAP_SLS_5_0GB;
	dw_pcie_writel_dbi(pci, DBI_EXP_CAP + PCI_EXP_LNKCAP, v);

	dw_pcie_writel_dbi(pci, DBI_AUX_CLK_FREQ, AUX_CLK_FREQ_26MHZ);
	dw_pcie_writel_dbi(pci, DBI_L1_SUBSTATES, L1_SUB_VAL);
	dw_pcie_writel_dbi(pci, DBI_PM_CAP + PCI_PM_CTRL, 0);

	v = dw_pcie_readl_dbi(pci, DBI_EXP_CAP + PCI_EXP_LNKCTL);
	v |= PCI_EXP_LNKCTL_RL;
	dw_pcie_writel_dbi(pci, DBI_EXP_CAP + PCI_EXP_LNKCTL, v);

	v = dw_pcie_readl_dbi(pci, DBI_EXP_CAP + PCI_EXP_LNKCTL2);
	v &= ~PCI_EXP_LNKCTL2_TLS;
	v |= pci->max_link_speed;
	dw_pcie_writel_dbi(pci, DBI_EXP_CAP + PCI_EXP_LNKCTL2, v);

	dw_pcie_dbi_ro_wr_dis(pci);
}

static void ex_enable_interrupts(struct exynos9810_pcie *ex)
{
	ex_elbi_wr(ex, IRQ_INTA_ASSERT | IRQ_INTB_ASSERT | IRQ_INTC_ASSERT |
		   IRQ_INTD_ASSERT, ELBI_IRQ_EN_PULSE);
	ex_elbi_wr(ex, 0, ELBI_IRQ_TOE_EN_PULSE2);
	ex_elbi_wr(ex, 0, ELBI_IRQ_TOE_EN_LEVEL);
	ex_elbi_wr(ex, 0, ELBI_IRQ_EN_LEVEL);
	ex_elbi_wr(ex, 0, ELBI_IRQ_EN_SPECIAL);
}

static void ex_print_history(struct exynos9810_pcie *ex)
{
	int i;

	for (i = 31; i >= 0; i--) {
		u32 h = ex_elbi_rd(ex, ELBI_HISTORY_REG(i));

		dev_info(ex->dev, "LTSSM: 0x%02x, L1sub: 0x%x, D state: 0x%x\n",
			 HIST_LTSSM(h), HIST_L1SUB(h), HIST_DSTATE(h));
	}
}

static void ex_irq_enable(struct exynos9810_pcie *ex, bool on)
{
	if (on && !ex->irq_enabled) {
		enable_irq(ex->irq);
		ex->irq_enabled = true;
	} else if (!on && ex->irq_enabled) {
		disable_irq(ex->irq);
		ex->irq_enabled = false;
	}
}

/* 4.9 exynos_pcie_establish_link() */
static int ex_establish_link(struct exynos9810_pcie *ex)
{
	struct dw_pcie *pci = &ex->pci;
	int tries = 0, i;
	u32 v;

retry:
	ex_phy_rx_elecidle(ex, 1);

	ex_elbi_wr(ex, 0, ELBI_SOFT_CORE_RESET);
	udelay(20);
	ex_elbi_wr(ex, 1, ELBI_SOFT_CORE_RESET);

	/* PERST# released (pin high), then 18-20 ms as on 4.9 */
	gpiod_set_value_cansleep(ex->perst, 1);
	usleep_range(18000, 20000);

	v = ex_elbi_rd(ex, ELBI_APP_REQ_EXIT_L1_MODE);
	v |= APP_REQ_EXIT_L1_MODE | L1_REQ_NAK_CONTROL_MASTER;
	ex_elbi_wr(ex, v, ELBI_APP_REQ_EXIT_L1_MODE);
	ex_elbi_wr(ex, LINKDOWN_RST_MANUAL, ELBI_LINKDOWN_RST_CTRL_SEL);

	v = ex_elbi_rd(ex, ELBI_QCH_SEL);
	v &= ~(CLOCK_GATING_PMU_MASK | CLOCK_GATING_APB_MASK |
	       CLOCK_GATING_AXI_MASK);
	ex_elbi_wr(ex, v, ELBI_QCH_SEL);

	ex_assert_phy_reset(ex);

	/*
	 * 4.9: dw_pcie_setup_rc() + exynos_pcie_setup_rc() before LTSSM.
	 * Before the host bridge exists only the link part of the DesignWare
	 * setup can run; dw_pcie_host_init() does the rest right after.
	 */
	if (ex->host_registered) {
		dw_pcie_setup_rc(&pci->pp);
	} else {
		dw_pcie_dbi_ro_wr_en(pci);
		dw_pcie_setup(pci);
		v = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
		v |= PORT_LOGIC_SPEED_CHANGE;
		dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, v);
		dw_pcie_dbi_ro_wr_dis(pci);
	}
	ex_setup_rc(ex);

	ex_phy_rx_elecidle(ex, 0);

	dev_info(ex->dev, "D state: %x, %x\n",
		 ex_elbi_rd(ex, ELBI_PM_DSTATE) & 0x7,
		 ex_elbi_rd(ex, ELBI_RDLH_LINKUP));

	ex_elbi_wr(ex, 1, ELBI_APP_LTSSM_ENABLE);
	for (i = 0; i < EX_TRAIN_POLLS; i++) {
		v = ex_ltssm(ex);
		if (v >= LTSSM_L0_FIRST && v <= LTSSM_L0_LAST)
			break;
		udelay(10);
	}

	if (i >= EX_TRAIN_POLLS) {
		tries++;
		ex->train_retries++;
		dev_err(ex->dev, "link is not up, try count: %d, %x\n", tries,
			ex_elbi_rd(ex, ELBI_RDLH_LINKUP));
		if (tries < EX_TRAIN_TRIES) {
			gpiod_set_value_cansleep(ex->perst, 0);
			ex_elbi_wr(ex, 0, ELBI_APP_LTSSM_ENABLE);
			goto retry;
		}
		ex_print_history(ex);
		return -EPIPE;
	}

	v = dw_pcie_readw_dbi(pci, DBI_EXP_CAP + PCI_EXP_LNKSTA);
	dev_info(ex->dev, "link up: ltssm 0x%x, GEN%u x%u (max GEN%u)\n",
		 ex_elbi_rd(ex, ELBI_RDLH_LINKUP),
		 (u32)FIELD_GET(PCI_EXP_LNKSTA_CLS, v),
		 (u32)FIELD_GET(PCI_EXP_LNKSTA_NLW, v), pci->max_link_speed);

	v = ex_elbi_rd(ex, ELBI_IRQ_PULSE);
	ex_elbi_wr(ex, v, ELBI_IRQ_PULSE);
	v = ex_elbi_rd(ex, ELBI_IRQ_EN_PULSE);
	v |= IRQ_LINKDOWN_ENABLE;
	ex_elbi_wr(ex, v, ELBI_IRQ_EN_PULSE);

	return 0;
}

/* 4.9 exynos_pcie_send_pme_turn_off(), sleeping instead of IRQs-off polls */
static void ex_send_pme_turn_off(struct exynos9810_pcie *ex)
{
	int retry = 0;
	u32 v;

	v = ex_ltssm(ex);
	if (!(v >= LTSSM_L0_FIRST && v <= LTSSM_L0_LAST)) {
		dev_info(ex->dev, "pme turn off: link not up (0x%x)\n", v);
		return;
	}

	ex_elbi_wr(ex, 1, ELBI_APP_REQ_EXIT_L1);
	v = ex_elbi_rd(ex, ELBI_APP_REQ_EXIT_L1_MODE);
	v &= ~APP_REQ_EXIT_L1_MODE;
	v |= L1_REQ_NAK_CONTROL_MASTER;
	ex_elbi_wr(ex, v, ELBI_APP_REQ_EXIT_L1_MODE);

	do {
		ex_elbi_wr(ex, 1, ELBI_XMIT_PME_TURNOFF);
		if (readl_poll_timeout(ex->elbi + ELBI_IRQ_PULSE, v,
				       v & IRQ_RADM_PM_TO_ACK, 10, 20000))
			dev_err(ex->dev, "no PME_TO_Ack from the endpoint\n");
		ex_elbi_wr(ex, 0, ELBI_XMIT_PME_TURNOFF);
		if (!readl_poll_timeout(ex->elbi + ELBI_RDLH_LINKUP, v,
					(v & LTSSM_MASK) == LTSSM_L23_READY,
					10, 20000))
			return;
	} while (++retry < EX_TRAIN_TRIES);

	dev_err(ex->dev, "no L23_READY (0x%x)\n", v);
}

static void ex_notify(struct exynos9810_pcie *ex, enum exynos_pcie_event ev)
{
	struct exynos_pcie_register_event *reg = ex->event_reg;

	if (reg && reg->callback && (reg->events & ev)) {
		reg->notify.event = ev;
		reg->notify.user = reg->user;
		dev_info(ex->dev, "callback for event %d\n", ev);
		reg->callback(&reg->notify);
	} else {
		/* 4.9 power-cycled the link here; stage 1 only reports. */
		dev_info(ex->dev, "no client for event %d\n", ev);
	}
}

static void ex_linkdown_work(struct work_struct *work)
{
	struct exynos9810_pcie *ex =
		container_of(work, struct exynos9810_pcie, linkdown_work);

	if (ex->state == EX_LINK_DOWN)
		return;
	ex_print_history(ex);
	ex_notify(ex, EXYNOS_PCIE_EVENT_LINKDOWN);
}

static irqreturn_t ex_irq(int irq, void *arg)
{
	struct exynos9810_pcie *ex = arg;
	u32 v;

	v = ex_elbi_rd(ex, ELBI_IRQ_PULSE);
	ex_elbi_wr(ex, v, ELBI_IRQ_PULSE);
	if ((v & IRQ_LINK_DOWN) && ex->state == EX_LINK_UP) {
		dev_info(ex->dev, "!!!PCIE LINK DOWN (state : 0x%x)!!!\n", v);
		ex->state = EX_LINK_DOWN_TRY;
		ex->linkdown_irqs++;
		schedule_work(&ex->linkdown_work);
	}

	v = ex_elbi_rd(ex, ELBI_IRQ_LEVEL);
	ex_elbi_wr(ex, v, ELBI_IRQ_LEVEL);
	v = ex_elbi_rd(ex, ELBI_IRQ_SPECIAL);
	ex_elbi_wr(ex, v, ELBI_IRQ_SPECIAL);

	return IRQ_HANDLED;
}

static void ex_set_state(struct exynos9810_pcie *ex, enum ex_state s)
{
	unsigned long flags;

	spin_lock_irqsave(&ex->reg_lock, flags);
	ex->state = s;
	spin_unlock_irqrestore(&ex->reg_lock, flags);
}

/* 4.9 exynos_pcie_poweroff(), caller holds ex->lock */
static void ex_poweroff_locked(struct exynos9810_pcie *ex)
{
	u32 v;

	if (ex->state == EX_LINK_UP || ex->state == EX_LINK_DOWN_TRY) {
		ex_set_state(ex, EX_LINK_DOWN_TRY);
		ex_irq_enable(ex, false);

		v = ex_elbi_rd(ex, ELBI_IRQ_EN_PULSE);
		v &= ~IRQ_LINKDOWN_ENABLE;
		ex_elbi_wr(ex, v, ELBI_IRQ_EN_PULSE);

		ex_send_pme_turn_off(ex);
		ex_set_state(ex, EX_LINK_DOWN);

		v = ex_elbi_rd(ex, ELBI_STATE_HISTORY_CHECK);
		v &= ~HISTORY_BUFFER_ENABLE;
		ex_elbi_wr(ex, v, ELBI_STATE_HISTORY_CHECK);

		gpiod_set_value_cansleep(ex->perst, 0);
		ex_elbi_wr(ex, 0, ELBI_APP_LTSSM_ENABLE);
		ex_phy_all_pwrdn(ex);
		ex_elbi_wr(ex, NCLK_OFF, ELBI_L12ERR_CTRL);

		if (!IS_ERR_OR_NULL(ex->pin_idle))
			pinctrl_select_state(ex->pinctrl, ex->pin_idle);
		ex->poweroff_cnt++;
	}

	regmap_update_bits(ex->pmu, PMU_WAKEUP_MASK,
			   BIT(WAKEUP_MASK_PCIE_WIFI + ex->ch),
			   BIT(WAKEUP_MASK_PCIE_WIFI + ex->ch));
	dev_info(ex->dev, "poweroff done, state %d\n", ex->state);
}

static int ex_register_host(struct exynos9810_pcie *ex)
{
	struct dw_pcie_rp *pp = &ex->pci.pp;
	struct pci_dev *rp;
	int ret;

	/* PCIe r5.0 6.6.1: 100 ms after link-up before configuration requests */
	msleep(100);

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(ex->dev, "host init failed: %d\n", ret);
		return ret;
	}

	rp = pci_get_slot(pp->bridge->bus, PCI_DEVFN(0, 0));
	if (!rp) {
		dev_err(ex->dev, "root port not found after enumeration\n");
		return 0;	/* bridge is registered; nothing to restore later */
	}
	pci_save_state(rp);
	ex->rp_saved = pci_store_saved_state(rp);
	ex->root_port = rp;
	return 0;
}

/* 4.9 exynos_pcie_poweron(), caller holds ex->lock */
static int ex_poweron_locked(struct exynos9810_pcie *ex)
{
	int ret;
	u32 v;

	dev_info(ex->dev, "poweron start, state %d\n", ex->state);
	if (ex->state != EX_LINK_DOWN)
		return 0;

	regmap_update_bits(ex->pmu, PMU_WAKEUP_MASK,
			   BIT(WAKEUP_MASK_PCIE_WIFI + ex->ch), 0);
	if (!IS_ERR_OR_NULL(ex->pin_default))
		pinctrl_select_state(ex->pinctrl, ex->pin_default);
	regmap_update_bits(ex->pmu, PMU_PCIE_PHY_CONTROL + ex->ch * 4, 1, 1);
	ex_phy_all_pwrdn_clear(ex);

	ex_set_state(ex, EX_LINK_UP_TRY);

	v = ex_elbi_rd(ex, ELBI_STATE_HISTORY_CHECK);
	ex_elbi_wr(ex, 0, ELBI_STATE_HISTORY_CHECK);
	ex_elbi_wr(ex, HISTORY_BUFFER_ENABLE, ELBI_STATE_HISTORY_CHECK);
	ex_elbi_wr(ex, 0x200000, ELBI_STATE_POWER_S);
	ex_elbi_wr(ex, 0xffffffff, ELBI_STATE_POWER_M);

	ex_irq_enable(ex, true);
	ret = ex_establish_link(ex);
	if (ret)
		goto fail;

	ex_elbi_wr(ex, 0, ELBI_L12ERR_CTRL);
	ex_set_state(ex, EX_LINK_UP);

	if (!ex->host_registered) {
		ret = ex_register_host(ex);
		if (ret)
			goto fail;
		ex->host_registered = true;
	} else if (ex->root_port && ex->rp_saved) {
		ret = pci_load_saved_state(ex->root_port, ex->rp_saved);
		if (!ret)
			pci_restore_state(ex->root_port);
		else
			dev_err(ex->dev, "root port state restore failed: %d\n", ret);
	}

	ex->poweron_ok++;
	dev_info(ex->dev, "poweron done, state %d\n", ex->state);
	return 0;

fail:
	ex->poweron_fail++;
	dev_err(ex->dev, "poweron failed: %d\n", ret);
	ex_set_state(ex, EX_LINK_UP);
	ex_poweroff_locked(ex);
	return -EPIPE;
}

static struct exynos9810_pcie *ex_get(int ch)
{
	struct exynos9810_pcie *ex = NULL;
	unsigned long flags;

	if (ch < 0 || ch >= EX_MAX_CH)
		return NULL;
	spin_lock_irqsave(&g_ex_lock, flags);
	ex = g_ex[ch];
	spin_unlock_irqrestore(&g_ex_lock, flags);
	return ex;
}

int exynos_pcie_pm_resume(int ch_num)
{
	struct exynos9810_pcie *ex = ex_get(ch_num);
	int ret;

	if (!ex)
		return -ENODEV;
	mutex_lock(&ex->lock);
	ret = ex_poweron_locked(ex);
	mutex_unlock(&ex->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(exynos_pcie_pm_resume);

void exynos_pcie_pm_suspend(int ch_num)
{
	struct exynos9810_pcie *ex = ex_get(ch_num);

	if (!ex)
		return;
	mutex_lock(&ex->lock);
	if (ex->state == EX_LINK_DOWN)
		dev_info(ex->dev, "RC%d already off\n", ex->ch);
	else
		ex_set_state(ex, EX_LINK_DOWN_TRY);
	ex_poweroff_locked(ex);
	mutex_unlock(&ex->lock);
}
EXPORT_SYMBOL_GPL(exynos_pcie_pm_suspend);

/* L1 substates are not enabled in stage 1; keep the 4.9 API for DHD. */
int exynos_pcie_l1ss_ctrl(int enable, int id)
{
	struct exynos9810_pcie *ex = ex_get(0);

	if (ex)
		ex->l1ss_requests++;
	return 0;
}
EXPORT_SYMBOL_GPL(exynos_pcie_l1ss_ctrl);

/*
 * 4.9 forced the link out of L1 before DHD touched the dongle, but only
 * while L1 substates were enabled (l1ss_ctrl_id_state == 0). ASPM/L1SS are
 * never enabled in stage 1, the link stays in L0, so this is the 4.9
 * "L1.2 already disabled" branch.
 */
int exynos_pcie_l1_exit(int ch_num)
{
	return 0;
}
EXPORT_SYMBOL_GPL(exynos_pcie_l1_exit);

static struct exynos9810_pcie *ex_from_user(void *user)
{
	struct pci_dev *pdev = user;
	int i;

	if (!pdev)
		return NULL;
	for (i = 0; i < EX_MAX_CH; i++) {
		struct exynos9810_pcie *ex = ex_get(i);

		if (ex && ex->host_registered &&
		    pci_find_host_bridge(pdev->bus) == ex->pci.pp.bridge)
			return ex;
	}
	return NULL;
}

int exynos_pcie_register_event(struct exynos_pcie_register_event *reg)
{
	struct exynos9810_pcie *ex;

	if (!reg || !reg->user)
		return -ENODEV;
	ex = ex_from_user(reg->user);
	if (!ex) {
		pr_err("PCIe: did not find RC for pci endpoint device\n");
		return -ENODEV;
	}
	ex->event_reg = reg;
	dev_info(ex->dev, "event 0x%x registered for RC %u\n", reg->events,
		 ex->ch);
	return 0;
}
EXPORT_SYMBOL_GPL(exynos_pcie_register_event);

int exynos_pcie_deregister_event(struct exynos_pcie_register_event *reg)
{
	struct exynos9810_pcie *ex;

	if (!reg || !reg->user)
		return -ENODEV;
	ex = ex_from_user(reg->user);
	if (!ex)
		return -ENODEV;
	cancel_work_sync(&ex->linkdown_work);
	ex->event_reg = NULL;
	return 0;
}
EXPORT_SYMBOL_GPL(exynos_pcie_deregister_event);

int exynos_pcie_get_status(int ch_num, struct exynos_pcie_status *st)
{
	struct exynos9810_pcie *ex = ex_get(ch_num);
	u16 lnksta = 0;

	memset(st, 0, sizeof(*st));
	if (!ex)
		return -ENODEV;

	mutex_lock(&ex->lock);
	st->probed = true;
	st->host_registered = ex->host_registered;
	st->state = ex->state;
	st->ltssm = ex_ltssm(ex);
	if (ex->state == EX_LINK_UP) {
		lnksta = dw_pcie_readw_dbi(&ex->pci, DBI_EXP_CAP + PCI_EXP_LNKSTA);
		st->rc_vendor_device = dw_pcie_readl_dbi(&ex->pci, PCI_VENDOR_ID);
	}
	st->speed = FIELD_GET(PCI_EXP_LNKSTA_CLS, lnksta);
	st->width = FIELD_GET(PCI_EXP_LNKSTA_NLW, lnksta);
	st->poweron_ok = ex->poweron_ok;
	st->poweron_fail = ex->poweron_fail;
	st->poweroff = ex->poweroff_cnt;
	st->train_retries = ex->train_retries;
	st->linkdown_irqs = ex->linkdown_irqs;
	st->l1ss_requests = ex->l1ss_requests;
	mutex_unlock(&ex->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(exynos_pcie_get_status);

/* 4.9 exynos_pcie_resumed_phydown(): PHY configured, then powered down */
static void ex_phydown_at_probe(struct exynos9810_pcie *ex)
{
	ex_enable_interrupts(ex);
	regmap_update_bits(ex->pmu, PMU_PCIE_PHY_CONTROL + ex->ch * 4, 1, 1);
	ex_assert_phy_reset(ex);
	ex_phy_all_pwrdn(ex);
	ex_elbi_wr(ex, NCLK_OFF, ELBI_L12ERR_CTRL);
}

static int ex_check_ia(struct exynos9810_pcie *ex, struct platform_device *pdev)
{
	static const struct { const char *name; resource_size_t pa; } want[] = {
		{ "elbi", IA_ELBI_PA }, { "phy", IA_PMA_PA },
		{ "pcs", IA_PCS_PA }, { "ia", IA_SELF_PA },
	};
	struct resource *r;
	int i;

	for (i = 0; i < ARRAY_SIZE(want); i++) {
		r = platform_get_resource_byname(pdev, IORESOURCE_MEM, want[i].name);
		if (!r || r->start != want[i].pa) {
			dev_warn(ex->dev, "I/A disabled: %s not at %pa\n",
				 want[i].name, &want[i].pa);
			return -EINVAL;
		}
	}
	return 0;
}

static int exynos9810_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct exynos9810_pcie *ex;
	struct dw_pcie *pci;
	struct resource *res;
	unsigned long flags;
	int ret;

	ex = devm_kzalloc(dev, sizeof(*ex), GFP_KERNEL);
	if (!ex)
		return -ENOMEM;

	ex->dev = dev;
	pci = &ex->pci;
	pci->dev = dev;
	pci->ops = &ex_dw_ops;
	pci->pp.ops = &ex_host_ops;
	/* No eDMA on this RC: make dw_pcie_edma_detect() bail out early. */
	pci->edma.mf = EDMA_MF_EDMA_UNROLL;

	if (of_property_read_u32(np, "samsung,channel", &ex->ch) || ex->ch != 0)
		return dev_err_probe(dev, -EINVAL, "only channel 0 is supported\n");

	mutex_init(&ex->lock);
	spin_lock_init(&ex->reg_lock);
	INIT_WORK(&ex->linkdown_work, ex_linkdown_work);
	ex->state = EX_LINK_DOWN;

	ex->elbi = devm_platform_ioremap_resource_byname(pdev, "elbi");
	if (IS_ERR(ex->elbi))
		return PTR_ERR(ex->elbi);
	ex->phy = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(ex->phy))
		return PTR_ERR(ex->phy);
	ex->phy_sysreg = devm_platform_ioremap_resource_byname(pdev, "sysreg");
	if (IS_ERR(ex->phy_sysreg))
		return PTR_ERR(ex->phy_sysreg);
	ex->pcs = devm_platform_ioremap_resource_byname(pdev, "pcs");
	if (IS_ERR(ex->pcs))
		return PTR_ERR(ex->pcs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	pci->dbi_base = devm_pci_remap_cfg_resource(dev, res);
	if (IS_ERR(pci->dbi_base))
		return PTR_ERR(pci->dbi_base);
	pci->dbi_phys_addr = res->start;

	ex->use_ia = of_property_read_bool(np, "samsung,use-ia") &&
		     !ex_check_ia(ex, pdev);
	if (ex->use_ia) {
		ex->ia = devm_platform_ioremap_resource_byname(pdev, "ia");
		if (IS_ERR(ex->ia))
			return PTR_ERR(ex->ia);
	}

	ex->pmu = syscon_regmap_lookup_by_phandle(np, "samsung,pmu-syscon");
	if (IS_ERR(ex->pmu))
		return dev_err_probe(dev, PTR_ERR(ex->pmu), "no PMU syscon\n");

	/* Logical 1 = PERST# released (pin high), as 4.9 gpio_set_value(1). */
	ex->perst = devm_gpiod_get(dev, "perst", GPIOD_OUT_LOW);
	if (IS_ERR(ex->perst))
		return dev_err_probe(dev, PTR_ERR(ex->perst), "no perst gpio\n");

	ex->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(ex->pinctrl))
		return dev_err_probe(dev, PTR_ERR(ex->pinctrl), "no pinctrl\n");
	ex->pin_default = pinctrl_lookup_state(ex->pinctrl, "default");
	if (IS_ERR(ex->pin_default))
		return dev_err_probe(dev, PTR_ERR(ex->pin_default),
				     "no default pin state\n");
	ex->pin_idle = pinctrl_lookup_state(ex->pinctrl, "idle");

	pci->num_lanes = 1;
	of_property_read_u32(np, "num-lanes", &pci->num_lanes);
	pci->max_link_speed = of_pci_get_max_link_speed(np);
	if (pci->max_link_speed < 1 || pci->max_link_speed > 2)
		pci->max_link_speed = 2;

	ex->irq = platform_get_irq(pdev, 0);
	if (ex->irq < 0)
		return ex->irq;
	/* Shared with the endpoint INTx (interrupt-map -> same SPI). */
	ret = devm_request_irq(dev, ex->irq, ex_irq, IRQF_SHARED,
			       "exynos9810-pcie", ex);
	if (ret)
		return dev_err_probe(dev, ret, "irq %d\n", ex->irq);
	disable_irq(ex->irq);
	ex->irq_enabled = false;

	platform_set_drvdata(pdev, ex);

	ex_phydown_at_probe(ex);
	if (!IS_ERR(ex->pin_idle))
		pinctrl_select_state(ex->pinctrl, ex->pin_idle);

	spin_lock_irqsave(&g_ex_lock, flags);
	g_ex[ex->ch] = ex;
	spin_unlock_irqrestore(&g_ex_lock, flags);

	dev_info(dev, "RC%u ready (link down), I/A %s, max GEN%u x%u\n",
		 ex->ch, ex->use_ia ? "on" : "off", pci->max_link_speed,
		 pci->num_lanes);
	return 0;
}

static int ex_suspend_noirq(struct device *dev)
{
	struct exynos9810_pcie *ex = dev_get_drvdata(dev);

	mutex_lock(&ex->lock);
	ex->was_up_at_suspend = ex->state != EX_LINK_DOWN;
	if (ex->was_up_at_suspend) {
		dev_warn(dev, "link still up at suspend, powering off\n");
		ex_set_state(ex, EX_LINK_DOWN_TRY);
		ex_poweroff_locked(ex);
	}
	mutex_unlock(&ex->lock);
	return 0;
}

static int ex_resume_noirq(struct device *dev)
{
	struct exynos9810_pcie *ex = dev_get_drvdata(dev);

	mutex_lock(&ex->lock);
	ex_elbi_wr(ex, NCLK_OFF, ELBI_L12ERR_CTRL);
	regmap_update_bits(ex->pmu, PMU_PCIE_PHY_CONTROL + ex->ch * 4, 1, 1);
	if (ex->was_up_at_suspend)
		ex_poweron_locked(ex);
	mutex_unlock(&ex->lock);
	return 0;
}

static const struct dev_pm_ops ex_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(ex_suspend_noirq, ex_resume_noirq)
};

/*
 * 4.9 exynos9810_pcie_quirks(): no async suspend, and DMA went through the
 * PCIe SysMMU (32-bit IOVAs). The SysMMU is not used here, so endpoint DMA
 * is capped at 32 bits (swiotlb bounces anything higher): 36-bit bus
 * mastering by this RC was never exercised on 4.9. A bus limit is used
 * instead of DT dma-ranges because 6.12 turns dma-ranges into inbound iATU
 * windows, which 4.9 never programmed.
 */
static void ex_pci_fixup(struct pci_dev *pdev)
{
	struct pci_host_bridge *bridge = pci_find_host_bridge(pdev->bus);
	int i;

	for (i = 0; i < EX_MAX_CH; i++) {
		struct exynos9810_pcie *ex = ex_get(i);

		if (ex && bridge && bridge->dev.parent == ex->dev) {
			device_disable_async_suspend(&pdev->dev);
			pdev->dev.bus_dma_limit = DMA_BIT_MASK(32);
			pci_info(pdev, "exynos9810: async suspend off, DMA limit 32 bit\n");
			return;
		}
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, ex_pci_fixup);

static const struct of_device_id exynos9810_pcie_of_match[] = {
	{ .compatible = "samsung,exynos9810-pcie" },
	{ }
};

static struct platform_driver exynos9810_pcie_driver = {
	.probe = exynos9810_pcie_probe,
	.driver = {
		.name = "exynos9810-pcie",
		.of_match_table = exynos9810_pcie_of_match,
		.pm = pm_sleep_ptr(&ex_pm_ops),
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(exynos9810_pcie_driver);
