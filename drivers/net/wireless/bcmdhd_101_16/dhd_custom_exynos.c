/*
 * Platform Dependent file for Samsung Exynos
 *
 * Copyright (C) 2021, Broadcom.
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id$
 */

/*
 * crownlte (Galaxy Note9, Exynos 9810, BCM4361 on PCIe ch0), Linux 6.12.
 *
 * Same power sequence and wifi_platform_data callbacks as the crownlte 4.9
 * dhd_custom_exynos.c (dhd_wlan_init_gpio/dhd_wlan_power/
 * dhd_wlan_set_carddetect), restructured as a platform driver for the
 * "samsung,brcm-wlan" DT node so that GPIOs come from gpiod, the pin states
 * are applied by the driver core, and probe can wait for the PCIe RC
 * (-EPROBE_DEFER) instead of failing at a fixed initcall. Nothing in DHD
 * runs unless this node is enabled (dhd_crownlte_wlan_ready()).
 *
 * Dropped from 4.9: sec_device_create("wlan") and the gpio_export() links
 * under /sys/class/sec/wlan (no userspace consumer in Lineage), and the
 * SDIO/MMC branches (PCIe only).
 */

#include <linux/delay.h>
#include <linux/exynos-pci-ctrl.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <typedefs.h>
#include <linuxver.h>
#include <osl.h>
#include <bcmutils.h>
#include <linux_osl.h>
#include <dhd_dbg.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <dhd_bus.h>
#include <dhd_linux.h>
#include <dhd_crownlte.h>

#ifndef WIFI_TURNON_DELAY
#define WIFI_TURNON_DELAY	200	/* ms, 4.9 dhd_custom_exynos.c (dhd.h has the same) */
#endif
#define CROWNLTE_PCIE_CH	0	/* 4.9 EXYNOS_PCIE_CH_NUM for crownlte */

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
extern void dhd_exit_wlan_mem(void);
extern int dhd_init_wlan_mem(void);
extern void *dhd_wlan_mem_prealloc(int section, unsigned long size);
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

static struct {
	struct gpio_desc *reg_on;
	struct gpio_desc *host_wake;
	int host_wake_irq;
	bool probed;
	int probe_ret;
	u32 power_on;
	u32 power_off;
	u32 card_on;
	u32 card_off;
	int last_card_ret;
	int prealloc_ret;
} crownlte_wlan = {
	.probe_ret = -ENODEV,
};

static int
dhd_wlan_power(int onoff)
{
	printk(KERN_INFO "%s Enter: power %s\n", __FUNCTION__, onoff ? "on" : "off");

	if (!crownlte_wlan.reg_on) {
		printk(KERN_ERR "%s: no WLAN_REG_ON\n", __FUNCTION__);
		return -EIO;
	}
	if (gpiod_direction_output(crownlte_wlan.reg_on, onoff ? 1 : 0)) {
		printk(KERN_ERR "%s failed to control WLAN_REG_ON to %s\n",
			__FUNCTION__, onoff ? "HIGH" : "LOW");
		return -EIO;
	}
	if (onoff)
		crownlte_wlan.power_on++;
	else
		crownlte_wlan.power_off++;
	return 0;
}

static int
dhd_wlan_reset(int onoff)
{
	return 0;
}

static int
dhd_wlan_set_carddetect(int val)
{
	if (val) {
		crownlte_wlan.card_on++;
		crownlte_wlan.last_card_ret = exynos_pcie_pm_resume(CROWNLTE_PCIE_CH);
	} else {
		crownlte_wlan.card_off++;
		exynos_pcie_pm_suspend(CROWNLTE_PCIE_CH);
		crownlte_wlan.last_card_ret = 0;
	}
	/* 4.9 ignored the RC result here as well */
	return 0;
}

#if defined(CONFIG_BCMDHD_OOB_HOST_WAKE) && defined(CONFIG_BCMDHD_GET_OOB_STATE)
int
dhd_get_wlan_oob_gpio(void)
{
	return crownlte_wlan.host_wake ?
		gpiod_get_value(crownlte_wlan.host_wake) : -1;
}
EXPORT_SYMBOL(dhd_get_wlan_oob_gpio);

int
dhd_get_wlan_oob_gpio_number(void)
{
	return crownlte_wlan.host_wake ? desc_to_gpio(crownlte_wlan.host_wake) : -1;
}
EXPORT_SYMBOL(dhd_get_wlan_oob_gpio_number);
#endif /* CONFIG_BCMDHD_OOB_HOST_WAKE && CONFIG_BCMDHD_GET_OOB_STATE */

struct resource dhd_wlan_resources = {
	.name	= "bcmdhd_wlan_irq",
	.start	= 0,
	.end	= 0,
	.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE |
	IORESOURCE_IRQ_HIGHEDGE,
};
EXPORT_SYMBOL(dhd_wlan_resources);

struct wifi_platform_data dhd_wlan_control = {
	.set_power	= dhd_wlan_power,
	.set_reset	= dhd_wlan_reset,
	.set_carddetect	= dhd_wlan_set_carddetect,
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	.mem_prealloc	= dhd_wlan_mem_prealloc,
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */
};
EXPORT_SYMBOL(dhd_wlan_control);

bool
dhd_crownlte_wlan_ready(void)
{
	return crownlte_wlan.probed && crownlte_wlan.probe_ret == 0;
}

void
dhd_crownlte_get_plat_status(struct dhd_crownlte_plat_status *st)
{
	memset(st, 0, sizeof(*st));
	st->probed = crownlte_wlan.probed;
	st->probe_ret = crownlte_wlan.probe_ret;
	st->reg_on = crownlte_wlan.reg_on ?
		gpiod_get_value_cansleep(crownlte_wlan.reg_on) : -1;
	st->host_wake = crownlte_wlan.host_wake ?
		gpiod_get_value_cansleep(crownlte_wlan.host_wake) : -1;
	st->host_wake_irq = crownlte_wlan.host_wake_irq;
	st->power_on = crownlte_wlan.power_on;
	st->power_off = crownlte_wlan.power_off;
	st->card_on = crownlte_wlan.card_on;
	st->card_off = crownlte_wlan.card_off;
	st->last_card_ret = crownlte_wlan.last_card_ret;
	st->prealloc_ret = crownlte_wlan.prealloc_ret;
}

/* Called by dhd_linux_platdev.c only for BCMDHD_MODULAR builds. */
int
dhd_wlan_init(void)
{
	return dhd_crownlte_wlan_ready() ? 0 : -ENODEV;
}

int
dhd_wlan_deinit(void)
{
	return 0;
}

static int
dhd_crownlte_wlan_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_desc *reg_on, *wake;
	int ret, irq;

	reg_on = devm_gpiod_get(dev, "wlan-reg-on", GPIOD_ASIS);
	if (IS_ERR(reg_on))
		return dev_err_probe(dev, PTR_ERR(reg_on), "WLAN_REG_ON\n");
	wake = devm_gpiod_get(dev, "wlan-host-wake", GPIOD_IN);
	if (IS_ERR(wake))
		return dev_err_probe(dev, PTR_ERR(wake), "WLAN_HOST_WAKE\n");
	irq = gpiod_to_irq(wake);
	if (irq < 0)
		return dev_err_probe(dev, irq, "WLAN_HOST_WAKE irq\n");

	crownlte_wlan.reg_on = reg_on;
	crownlte_wlan.host_wake = wake;
	crownlte_wlan.host_wake_irq = irq;

	/* 4.9 dhd_wlan_init_gpio(): REG_ON high, 200 ms, then train the link */
	ret = gpiod_direction_output(reg_on, 1);
	if (ret)
		return dev_err_probe(dev, ret, "WLAN_REG_ON output\n");
	crownlte_wlan.power_on++;
	msleep(WIFI_TURNON_DELAY);

	crownlte_wlan.card_on++;
	ret = exynos_pcie_pm_resume(CROWNLTE_PCIE_CH);
	crownlte_wlan.last_card_ret = ret;
	if (ret == -ENODEV) {
		/* RC not probed yet: power the chip down again and retry later */
		gpiod_direction_output(reg_on, 0);
		crownlte_wlan.power_off++;
		crownlte_wlan.reg_on = NULL;
		crownlte_wlan.host_wake = NULL;
		return -EPROBE_DEFER;
	}
	if (ret) {
		dev_err(dev, "pcie link up failure: %d, WLAN_REG_ON low\n", ret);
		gpiod_direction_output(reg_on, 0);
		crownlte_wlan.power_off++;
		/* devm releases the GPIOs when probe fails */
		crownlte_wlan.reg_on = NULL;
		crownlte_wlan.host_wake = NULL;
		crownlte_wlan.probed = true;
		crownlte_wlan.probe_ret = ret;
		return ret;
	}

	dhd_wlan_resources.start = irq;
	dhd_wlan_resources.end = irq;

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	crownlte_wlan.prealloc_ret = dhd_init_wlan_mem();
	if (crownlte_wlan.prealloc_ret < 0)
		dev_err(dev, "failed to alloc reserved memory, ret=%d\n",
			crownlte_wlan.prealloc_ret);
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

	crownlte_wlan.probed = true;
	crownlte_wlan.probe_ret = 0;
	dev_info(dev, "BCM4361 powered, PCIe ch%d link up, host-wake irq %d\n",
		CROWNLTE_PCIE_CH, irq);
	return 0;
}

static const struct of_device_id dhd_crownlte_wlan_of_match[] = {
	{ .compatible = "samsung,brcm-wlan" },
	{ }
};

static struct platform_driver dhd_crownlte_wlan_driver = {
	.probe = dhd_crownlte_wlan_probe,
	.driver = {
		.name = "crownlte-brcm-wlan",
		.of_match_table = dhd_crownlte_wlan_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(dhd_crownlte_wlan_driver);
