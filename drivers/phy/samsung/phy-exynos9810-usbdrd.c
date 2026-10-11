// SPDX-License-Identifier: GPL-2.0
/*
 * USB 2.0 (UTMI+) PHY of the Samsung Exynos 9810.
 *
 * The block is Samsung's "USB DRD combo PHY" in its 3.0 revision
 * (phy_version 0x300, sub_phy_version 0x400 in the vendor device tree).
 * The register sequence below is the vendor CAL for EXYNOS_USBCON_VER_03_0_0
 * (phy_exynos_usb_v3p1_enable / _disable / _pipe_ovrd / rewa_ready in
 * phy-exynos-usb3p1.c of the Samsung 4.9 kernel), transcribed step by step
 * with the board's own settings folded in:
 *
 *   common_block_disable = 1   -> HSP_COMMONONN set
 *   is_not_vbus_pad = 1        -> VBUS forced valid, bus filter bypassed
 *   use_io_for_ovc = 0         -> over-current from the hub, not a pad
 *   hs_rewa = 1                -> ReWA block parked in its "ready" state
 *   dual_phy = 0               -> DUALPHYSEL untouched
 *
 * Nothing here is borrowed from the Exynos 850/990 path of the mainline
 * driver: that path writes SSPPLLCTL FSEL and the PHY20 POR bits, neither of
 * which the 9810 vendor code touches, and it skips the Q-channel and link
 * reset steps the vendor code performs first.
 *
 * Only the high-speed side is brought up. The SuperSpeed/DP combo PMA at
 * 0x110a0000 stays powered down (PMA_LOW_PWRN), exactly as the vendor does
 * before the USB3 PHY instance is enabled.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/* USBCON register block at 0x11100000 (vendor phy-exynos-usb3p1-reg.h) */
#define USBCON_CTRL_VER			0x00
#define USBCON_LINK_CTRL		0x04
#define  LINKCTRL_PIPE3_FORCE_RX_ELEC_IDLE	BIT(18)
#define  LINKCTRL_PIPE3_FORCE_PHY_STATUS	BIT(17)
#define  LINKCTRL_PIPE3_FORCE_EN		BIT(16)
#define  LINKCTRL_DIS_QACT_BUSPEND		BIT(13)
#define  LINKCTRL_DIS_QACT_LINKGATE		BIT(12)
#define  LINKCTRL_DIS_QACT_ID0			BIT(11)
#define  LINKCTRL_DIS_QACT_VBUS_VALID		BIT(10)
#define  LINKCTRL_DIS_QACT_BVALID		BIT(9)
#define  LINKCTRL_FORCE_QACT			BIT(8)
#define  LINKCTRL_BUS_FILTER_BYPASS_MASK	(0xf << 4)
#define USBCON_LINK_PORT		0x08
#define  LINKPORT_HUB_PORT_SEL_OCD_U3		BIT(3)
#define  LINKPORT_HUB_PORT_SEL_OCD_U2		BIT(2)
#define USBCON_CLKRST			0x20
#define  CLKRST_LINK_PCLK_SEL			BIT(7)
#define  CLKRST_PHY_SW_RST			BIT(3)
#define  CLKRST_PHY_RST_SEL			BIT(2)
#define  CLKRST_PORT_RST			BIT(1)
#define  CLKRST_LINK_SW_RST			BIT(0)
#define USBCON_PWR			0x24
#define USBCON_SSP_PLL			0x30
#define USBCON_COMBO_PMA_CTRL		0x48
#define  PMA_LOW_PWRN				BIT(4)
#define USBCON_UTMI			0x50
#define  UTMI_FORCE_VBUSVALID			BIT(5)
#define  UTMI_FORCE_BVALID			BIT(4)
#define  UTMI_DP_PULLDOWN			BIT(3)
#define  UTMI_DM_PULLDOWN			BIT(2)
#define  UTMI_FORCE_SUSPEND			BIT(1)
#define  UTMI_FORCE_SLEEP			BIT(0)
#define USBCON_HSP			0x54
#define  HSP_VBUSVLDEXTSEL			BIT(13)
#define  HSP_VBUSVLDEXT				BIT(12)
#define  HSP_EN_UTMISUSPEND			BIT(9)
#define  HSP_COMMONONN				BIT(8)
#define USBCON_HSP_TUNE			0x58
#define  HSP_TUNE_TXVREF_MASK			(0xfu << 28)
#define  HSP_TUNE_TXVREF(x)			(((x) & 0xfu) << 28)
#define  HSP_TUNE_TXRISE_MASK			(0x3 << 24)
#define  HSP_TUNE_TXRISE(x)			(((x) & 0x3) << 24)
#define  HSP_TUNE_TXRES_MASK			(0x3 << 21)
#define  HSP_TUNE_TXRES(x)			(((x) & 0x3) << 21)
#define  HSP_TUNE_TXPREEMPA_MASK		(0x3 << 18)
#define  HSP_TUNE_TXPREEMPA(x)			(((x) & 0x3) << 18)
#define  HSP_TUNE_SQRX_MASK			(0x7 << 8)
#define  HSP_TUNE_SQRX(x)			(((x) & 0x7) << 8)
#define  HSP_TUNE_COMPDIS_MASK			(0x7 << 0)
#define  HSP_TUNE_COMPDIS(x)			(((x) & 0x7) << 0)
#define USBCON_HSP_TEST			0x5c
#define  HSP_TEST_SIDDQ				BIT(24)
#define USBCON_HSP_PLL_TUNE		0x60
#define USBCON_REWA_ENABLE		0x100
#define  REWA_ENABLE_HS_REWA_EN			BIT(0)
#define USBCON_HSREWA_CTRL		0x108
#define  HSREWA_CTRL_DIG_BYPASS_CON_EN		BIT(28)
#define  HSREWA_CTRL_DPDM_MON_SEL		BIT(24)
#define USBCON_HSREWA_REFTO		0x10c
#define USBCON_HSREWA_HSTK		0x110
#define USBCON_HSREWA_INT1_MASK		0x11c
#define  HSREWA_INT1_ALL			(BIT(18) | BIT(17) | BIT(16) | \
						 BIT(2) | BIT(1) | BIT(0))

/* PMU: isolation of the two PHY halves. Bit 0 set = PHY enabled. */
#define PMU_USB20_PHY_CTRL		0x72c
#define PMU_USB3DP_PHY_CTRL		0x704
#define PMU_PHY_ENABLE			BIT(0)

/*
 * Stock star2lte HS tuning, device-mode column of the vendor hs_tune node
 * (tune_value = <dev host>): tx_pre_emp 3, tx_vref 0xd, rx_sqrx 4,
 * compdis 3, tx_res 3, tx_rise 1.
 */
#define S9P_HSP_TUNE_MASK	(HSP_TUNE_TXVREF_MASK | HSP_TUNE_TXRISE_MASK | \
				 HSP_TUNE_TXRES_MASK | HSP_TUNE_TXPREEMPA_MASK | \
				 HSP_TUNE_SQRX_MASK | HSP_TUNE_COMPDIS_MASK)
#define S9P_HSP_TUNE_VAL	(HSP_TUNE_TXVREF(0xd) | HSP_TUNE_TXRISE(1) | \
				 HSP_TUNE_TXRES(3) | HSP_TUNE_TXPREEMPA(3) | \
				 HSP_TUNE_SQRX(4) | HSP_TUNE_COMPDIS(3))

static bool s9p_phy_tune = true;
module_param_named(tune, s9p_phy_tune, bool, 0444);
MODULE_PARM_DESC(tune, "apply the stock star2lte HS tuning (default on)");

static bool s9p_phy_rewa = true;
module_param_named(rewa, s9p_phy_rewa, bool, 0444);
MODULE_PARM_DESC(rewa, "park the ReWA block like the vendor does (default on)");

struct s9p_usbphy {
	struct device *dev;
	void __iomem *regs;
	struct regmap *pmu;
	struct clk_bulk_data clks[2];
	int nclks;
	struct phy *phy;
};

static inline void rmw(void __iomem *reg, u32 clear, u32 set)
{
	writel((readl(reg) & ~clear) | set, reg);
}

static void s9p_usbphy_dump(struct s9p_usbphy *p, const char *when)
{
	static const struct { u16 off; const char *name; } regs[] = {
		{ USBCON_CTRL_VER, "ver" },
		{ USBCON_LINK_CTRL, "linkctrl" },
		{ USBCON_LINK_PORT, "linkport" },
		{ USBCON_CLKRST, "clkrst" },
		{ USBCON_PWR, "pwr" },
		{ USBCON_SSP_PLL, "ssp_pll" },
		{ USBCON_COMBO_PMA_CTRL, "pma" },
		{ USBCON_UTMI, "utmi" },
		{ USBCON_HSP, "hsp" },
		{ USBCON_HSP_TUNE, "hsp_tune" },
		{ USBCON_HSP_TEST, "hsp_test" },
		{ USBCON_HSP_PLL_TUNE, "hsp_pll" },
		{ USBCON_REWA_ENABLE, "rewa" },
	};
	char buf[320];
	int i, n = 0;
	unsigned int pmu20 = 0xffffffff, pmu3 = 0xffffffff;

	for (i = 0; i < ARRAY_SIZE(regs); i++)
		n += scnprintf(buf + n, sizeof(buf) - n, " %s=%08x",
			       regs[i].name, readl(p->regs + regs[i].off));
	if (p->pmu) {
		regmap_read(p->pmu, PMU_USB20_PHY_CTRL, &pmu20);
		regmap_read(p->pmu, PMU_USB3DP_PHY_CTRL, &pmu3);
	}
	dev_info(p->dev, "%s: pmu72c=%08x pmu704=%08x%s\n", when, pmu20, pmu3,
		 buf);
}

static void s9p_usbphy_pmu(struct s9p_usbphy *p, bool on)
{
	if (!p->pmu)
		return;
	regmap_update_bits(p->pmu, PMU_USB20_PHY_CTRL, PMU_PHY_ENABLE,
			   on ? PMU_PHY_ENABLE : 0);
	regmap_update_bits(p->pmu, PMU_USB3DP_PHY_CTRL, PMU_PHY_ENABLE,
			   on ? PMU_PHY_ENABLE : 0);
}

/* exynos_cal_usbphy_q_ch(): force the Q-channel active, with its delays */
static void s9p_usbphy_q_ch(struct s9p_usbphy *p, bool enable)
{
	void __iomem *reg = p->regs + USBCON_LINK_CTRL;
	u32 v;

	if (enable) {
		v = readl(reg);
		v |= LINKCTRL_DIS_QACT_ID0 | LINKCTRL_DIS_QACT_VBUS_VALID |
		     LINKCTRL_DIS_QACT_BVALID | LINKCTRL_DIS_QACT_LINKGATE;
		v &= ~LINKCTRL_FORCE_QACT;
		udelay(500);
		writel(v, reg);
		udelay(500);
		v = readl(reg);
		v |= LINKCTRL_FORCE_QACT;
		udelay(500);
		writel(v, reg);
	} else {
		v = readl(reg);
		v &= ~LINKCTRL_FORCE_QACT;
		v |= LINKCTRL_DIS_QACT_ID0 | LINKCTRL_DIS_QACT_VBUS_VALID |
		     LINKCTRL_DIS_QACT_BVALID | LINKCTRL_DIS_QACT_LINKGATE;
		writel(v, reg);
	}
}

/* phy_exynos_usb3p1_rewa_ready() */
static void s9p_usbphy_rewa_ready(struct s9p_usbphy *p)
{
	rmw(p->regs + USBCON_REWA_ENABLE, REWA_ENABLE_HS_REWA_EN, 0);
	rmw(p->regs + USBCON_HSREWA_CTRL, HSREWA_CTRL_DPDM_MON_SEL,
	    HSREWA_CTRL_DIG_BYPASS_CON_EN);
	writel(0x1, p->regs + USBCON_HSREWA_HSTK);
	writel(0xff00, p->regs + USBCON_HSREWA_REFTO);
	rmw(p->regs + USBCON_HSREWA_INT1_MASK, 0, HSREWA_INT1_ALL);
}

static int s9p_usbphy_init(struct phy *phy)
{
	struct s9p_usbphy *p = phy_get_drvdata(phy);
	void __iomem *r = p->regs;
	int ret;

	ret = clk_bulk_prepare_enable(p->nclks, p->clks);
	if (ret)
		return ret;

	s9p_usbphy_dump(p, "before init");

	/*
	 * The vendor releases the PMU isolation in power_on, which its glue
	 * calls before init. dwc3 calls init first, so release it here too;
	 * it is idempotent.
	 */
	s9p_usbphy_pmu(p, true);

	/* --- phy_exynos_usb_v3p1_enable(), VER_03_0_0 branch --- */
	s9p_usbphy_q_ch(p, true);

	/* link reset */
	rmw(r + USBCON_CLKRST, 0, CLKRST_LINK_SW_RST);
	udelay(10);
	rmw(r + USBCON_CLKRST, CLKRST_LINK_SW_RST, 0);

	/* PHY reset high */
	rmw(r + USBCON_CLKRST, 0, CLKRST_PHY_SW_RST | CLKRST_PHY_RST_SEL);

	/* PHY power: SIDDQ off */
	rmw(r + USBCON_HSP_TEST, HSP_TEST_SIDDQ, 0);

	/* UTMI+ out of forced suspend/sleep, pull-downs off */
	rmw(r + USBCON_UTMI, UTMI_FORCE_SUSPEND | UTMI_FORCE_SLEEP |
	    UTMI_DP_PULLDOWN | UTMI_DM_PULLDOWN, 0);

	/* HS PHY clock control; common_block_disable = 1 */
	rmw(r + USBCON_HSP, 0, HSP_EN_UTMISUSPEND | HSP_COMMONONN);

	udelay(100);

	/* PHY reset low */
	rmw(r + USBCON_CLKRST, CLKRST_PHY_SW_RST | CLKRST_PORT_RST,
	    CLKRST_PHY_RST_SEL);

	/* no VBUS pad on this board: force VBUS valid, bypass the bus filter */
	rmw(r + USBCON_LINK_CTRL, 0, LINKCTRL_BUS_FILTER_BYPASS_MASK);
	rmw(r + USBCON_UTMI, 0, UTMI_FORCE_BVALID | UTMI_FORCE_VBUSVALID);
	rmw(r + USBCON_HSP, 0, HSP_VBUSVLDEXTSEL | HSP_VBUSVLDEXT);

	/* over-current from the hub, not from an IO pad */
	rmw(r + USBCON_LINK_PORT, 0, LINKPORT_HUB_PORT_SEL_OCD_U3 |
	    LINKPORT_HUB_PORT_SEL_OCD_U2);

	if (s9p_phy_rewa)
		s9p_usbphy_rewa_ready(p);

	/* --- phy_exynos_usb_v3p1_pipe_ovrd(): USB3 side forced idle --- */
	rmw(r + USBCON_LINK_CTRL, LINKCTRL_PIPE3_FORCE_PHY_STATUS,
	    LINKCTRL_PIPE3_FORCE_EN | LINKCTRL_PIPE3_FORCE_RX_ELEC_IDLE);
	rmw(r + USBCON_COMBO_PMA_CTRL, 0, PMA_LOW_PWRN);

	/* --- phy_exynos_usb_v3p1_tune(), device-mode values --- */
	if (s9p_phy_tune)
		rmw(r + USBCON_HSP_TUNE, S9P_HSP_TUNE_MASK, S9P_HSP_TUNE_VAL);

	s9p_usbphy_dump(p, "after init");
	return 0;
}

static int s9p_usbphy_exit(struct phy *phy)
{
	struct s9p_usbphy *p = phy_get_drvdata(phy);
	void __iomem *r = p->regs;

	/* phy_exynos_usb_v3p1_disable() */
	rmw(r + USBCON_UTMI, 0, UTMI_FORCE_SUSPEND | UTMI_FORCE_SLEEP);
	rmw(r + USBCON_HSP_TEST, 0, HSP_TEST_SIDDQ);
	s9p_usbphy_q_ch(p, false);

	clk_bulk_disable_unprepare(p->nclks, p->clks);
	return 0;
}

static int s9p_usbphy_power_on(struct phy *phy)
{
	struct s9p_usbphy *p = phy_get_drvdata(phy);

	s9p_usbphy_pmu(p, true);
	s9p_usbphy_dump(p, "power_on");
	return 0;
}

static int s9p_usbphy_power_off(struct phy *phy)
{
	struct s9p_usbphy *p = phy_get_drvdata(phy);

	s9p_usbphy_pmu(p, false);
	return 0;
}

static const struct phy_ops s9p_usbphy_ops = {
	.init		= s9p_usbphy_init,
	.exit		= s9p_usbphy_exit,
	.power_on	= s9p_usbphy_power_on,
	.power_off	= s9p_usbphy_power_off,
	.owner		= THIS_MODULE,
};

static struct phy *s9p_usbphy_xlate(struct device *dev,
				    const struct of_phandle_args *args)
{
	struct s9p_usbphy *p = dev_get_drvdata(dev);

	/* index 0 is the UTMI+ (USB 2.0) PHY; the SuperSpeed one is not here */
	if (args->args_count && args->args[0] != 0)
		return ERR_PTR(-ENODEV);
	return p->phy;
}

static int s9p_usbphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct s9p_usbphy *p;
	int ret;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->dev = dev;

	p->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->regs))
		return PTR_ERR(p->regs);

	p->pmu = syscon_regmap_lookup_by_phandle(dev->of_node,
						 "samsung,pmu-syscon");
	if (IS_ERR(p->pmu)) {
		dev_warn(dev, "no PMU syscon (%ld), isolation left as is\n",
			 PTR_ERR(p->pmu));
		p->pmu = NULL;
	}

	p->clks[0].id = "ref";
	p->clks[1].id = "phy";
	p->nclks = 2;
	ret = devm_clk_bulk_get_optional(dev, p->nclks, p->clks);
	if (ret)
		return dev_err_probe(dev, ret, "clocks\n");

	p->phy = devm_phy_create(dev, NULL, &s9p_usbphy_ops);
	if (IS_ERR(p->phy))
		return PTR_ERR(p->phy);
	phy_set_drvdata(p->phy, p);
	dev_set_drvdata(dev, p);

	provider = devm_of_phy_provider_register(dev, s9p_usbphy_xlate);
	if (IS_ERR(provider))
		return PTR_ERR(provider);

	dev_info(dev, "Exynos 9810 USB 2.0 PHY, USBCON version %08x\n",
		 readl(p->regs + USBCON_CTRL_VER));
	return 0;
}

static const struct of_device_id s9p_usbphy_of_match[] = {
	{ .compatible = "samsung,exynos9810-usbdrd-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, s9p_usbphy_of_match);

static struct platform_driver s9p_usbphy_driver = {
	.probe	= s9p_usbphy_probe,
	.driver	= {
		.name		= "phy-exynos9810-usbdrd",
		.of_match_table	= s9p_usbphy_of_match,
	},
};
module_platform_driver(s9p_usbphy_driver);

MODULE_DESCRIPTION("Samsung Exynos 9810 USB 2.0 PHY");
MODULE_LICENSE("GPL");
