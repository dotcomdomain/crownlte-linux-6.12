// SPDX-License-Identifier: GPL-2.0
/*
 * Read-only "panel" backlight for the Galaxy Note9 (crownlte) S6E3HA8.
 *
 * The vendor light HAL and HWC look for
 * /sys/devices/platform/panel_drv@001/backlight/panel/{brightness,
 * max_brightness}. Until the S6E3HA8 panel driver is ported, this device
 * only mirrors the brightness SBoot set (4.9 boots at UI_DEF_BRIGHTNESS,
 * BRT(128) = 12800, of max 44700) and refuses changes: no DSI command, no
 * regulator, no GPIO is touched.
 */
#include <linux/backlight.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

struct panel_ro {
	struct backlight_device *bd;
	u32 brightness;
};

static int panel_ro_update_status(struct backlight_device *bd)
{
	struct panel_ro *p = bl_get_data(bd);

	if (bd->props.brightness == p->brightness)
		return 0;
	dev_info_ratelimited(&bd->dev, "read-only: brightness %d refused, panel stays at %u\n",
			     bd->props.brightness, p->brightness);
	bd->props.brightness = p->brightness;
	return -EPERM;
}

static int panel_ro_get_brightness(struct backlight_device *bd)
{
	struct panel_ro *p = bl_get_data(bd);

	return p->brightness;
}

static const struct backlight_ops panel_ro_ops = {
	.update_status = panel_ro_update_status,
	.get_brightness = panel_ro_get_brightness,
};

static int panel_ro_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct backlight_properties props = { };
	struct panel_ro *p;
	u32 max = 44700;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->brightness = 12800;
	of_property_read_u32(dev->of_node, "samsung,max-brightness", &max);
	of_property_read_u32(dev->of_node, "samsung,boot-brightness", &p->brightness);
	p->brightness = min(p->brightness, max);

	props.type = BACKLIGHT_RAW;
	props.max_brightness = max;
	props.brightness = p->brightness;
	p->bd = devm_backlight_device_register(dev, "panel", dev, p, &panel_ro_ops, &props);
	if (IS_ERR(p->bd))
		return PTR_ERR(p->bd);
	dev_info(dev, "read-only backlight \"panel\": %u/%u (no panel driver yet)\n",
		 p->brightness, max);
	return 0;
}

static const struct of_device_id panel_ro_of_match[] = {
	{ .compatible = "samsung,exynos9810-panel-readonly" },
	{ }
};
MODULE_DEVICE_TABLE(of, panel_ro_of_match);

static struct platform_driver panel_ro_driver = {
	.probe = panel_ro_probe,
	.driver = {
		.name = "panel-readonly",
		.of_match_table = panel_ro_of_match,
	},
};
module_platform_driver(panel_ro_driver);

MODULE_DESCRIPTION("Read-only panel backlight mirror for crownlte");
MODULE_LICENSE("GPL");
