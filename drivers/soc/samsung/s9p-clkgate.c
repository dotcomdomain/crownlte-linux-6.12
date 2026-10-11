// SPDX-License-Identifier: GPL-2.0
/*
 * Clock gates the S9+ port has to open by hand.
 *
 * There is no clock driver for the Exynos 9810 in mainline. Most of what this
 * kernel uses is left running by sboot and TWRP, but Samsung's I2C driver
 * gates its clock after every transfer and TWRP shuts the peripherals down
 * before rebooting, so the HSI2C controllers arrive without a clock and every
 * transfer times out. The CMU gate registers are plain MMIO: bit 20 selects
 * manual control, bit 21 is the clock value. Addresses come from the vendor
 * kernel's cmucal-sfr tables and are listed in the device tree so nothing
 * here is board-specific.
 *
 * Four lists, all optional:
 *   s9p,gate-registers   <addr>         set MANUAL|CG_VAL
 *   s9p,set-registers    <addr value>   written as-is
 *   s9p,or-registers     <addr bits>    OR-ed in
 *   s9p,clear-registers  <addr bits>    masked out
 *   s9p,dump-registers   <addr>         read and logged, never written
 */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define CLK_GATE_MANUAL		BIT(20)
#define CLK_GATE_CG_VAL		BIT(21)

static int __init s9p_clkgate_init(void)
{
	struct device_node *np;
	int i, n;

	np = of_find_compatible_node(NULL, NULL, "s9p,clock-gates");
	if (!np)
		return 0;

	/* read-only and first: the state everything below inherits from TWRP */
	n = of_property_count_u32_elems(np, "s9p,dump-registers");
	for (i = 0; i < n; i++) {
		u32 addr;
		void __iomem *reg;

		if (of_property_read_u32_index(np, "s9p,dump-registers", i, &addr))
			break;
		reg = ioremap(addr, 4);
		if (!reg)
			continue;
		pr_info("s9p-clkgate: dump %08x = %08x\n", addr, readl(reg));
		iounmap(reg);
	}
	n = of_property_count_u32_elems(np, "s9p,gate-registers");
	for (i = 0; i < n; i++) {
		u32 addr, val;
		void __iomem *reg;

		if (of_property_read_u32_index(np, "s9p,gate-registers", i,
					       &addr))
			break;
		reg = ioremap(addr, 4);
		if (!reg)
			continue;
		val = readl(reg);
		writel(val | CLK_GATE_MANUAL | CLK_GATE_CG_VAL, reg);
		pr_info("s9p-clkgate: %08x: %08x -> %08x\n", addr, val,
			readl(reg));
		iounmap(reg);
	}

	/* <address value> pairs, written as-is: USI SW_CONF mode, USI_CON reset. */
	n = of_property_count_u32_elems(np, "s9p,set-registers");
	for (i = 0; i + 1 < n; i += 2) {
		u32 addr, val, old;
		void __iomem *reg;

		if (of_property_read_u32_index(np, "s9p,set-registers", i, &addr) ||
		    of_property_read_u32_index(np, "s9p,set-registers", i + 1, &val))
			break;
		reg = ioremap(addr, 4);
		if (!reg)
			continue;
		old = readl(reg);
		writel(val, reg);
		udelay(1);
		pr_info("s9p-clkgate: %08x = %08x (was %08x)\n", addr, val, old);
		iounmap(reg);
	}

	/*
	 * <address bits> pairs, masked out. The PMU's watchdog reset mask is
	 * the reason this exists: with the mask bit set, an expired cluster
	 * watchdog raises nothing at all, so a boot that wedges before the
	 * watchdog driver probes at 1.3 s hangs for ever -- measured, that is
	 * exactly what every second warm reboot out of Ubuntu did.
	 */
	n = of_property_count_u32_elems(np, "s9p,clear-registers");
	for (i = 0; i + 1 < n; i += 2) {
		u32 addr, bits, val;
		void __iomem *reg;

		if (of_property_read_u32_index(np, "s9p,clear-registers", i, &addr) ||
		    of_property_read_u32_index(np, "s9p,clear-registers", i + 1, &bits))
			break;
		reg = ioremap(addr, 4);
		if (!reg)
			continue;
		val = readl(reg);
		writel(val & ~bits, reg);
		pr_info("s9p-clkgate: %08x: %08x & ~%08x -> %08x\n", addr, val,
			bits, readl(reg));
		iounmap(reg);
	}

	/* <address bits> pairs, OR-ed in: PMU PHY controls and the like. */
	n = of_property_count_u32_elems(np, "s9p,or-registers");
	for (i = 0; i + 1 < n; i += 2) {
		u32 addr, bits, val;
		void __iomem *reg;

		if (of_property_read_u32_index(np, "s9p,or-registers", i, &addr) ||
		    of_property_read_u32_index(np, "s9p,or-registers", i + 1, &bits))
			break;
		reg = ioremap(addr, 4);
		if (!reg)
			continue;
		val = readl(reg);
		writel(val | bits, reg);
		pr_info("s9p-clkgate: %08x |= %08x: %08x -> %08x\n", addr, bits,
			val, readl(reg));
		iounmap(reg);
	}

	/*
	 * <address> list of PLL_CON0 registers: a PLL the previous kernel
	 * switched off on its way out is enabled again (bit 31), waited for
	 * (STABLE, bit 29) and its output selected (MUX_SEL, bit 4). The P/M/S
	 * dividers are left as the bootloader programmed them.
	 */
	n = of_property_count_u32_elems(np, "s9p,pll-enable");
	for (i = 0; i < n; i++) {
		u32 addr, val, old;
		void __iomem *reg;
		int t;

		if (of_property_read_u32_index(np, "s9p,pll-enable", i, &addr))
			break;
		reg = ioremap(addr, 4);
		if (!reg)
			continue;
		old = val = readl(reg);
		if (!(val & BIT(31))) {
			writel(val | BIT(31), reg);
			for (t = 0; t < 5000 && !(readl(reg) & BIT(29)); t++)
				udelay(1);
			val = readl(reg);
		}
		if ((val & BIT(29)) && !(val & BIT(4)))
			writel(val | BIT(4), reg);
		pr_info("s9p-clkgate: pll %08x: %08x -> %08x (%s)\n", addr, old,
			readl(reg), (readl(reg) & BIT(29)) ? "locked" : "NOT LOCKED");
		iounmap(reg);
	}

	of_node_put(np);
	return 0;
}
postcore_initcall(s9p_clkgate_init);
