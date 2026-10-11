// SPDX-License-Identifier: GPL-2.0-only
/* Exynos9810 MIF BTS service, derived from the original bts-exynos9810.c
 * exynos_bts_mif_update() and cal_bts9810.c bts_set_qmax().
 * This supplies the real four SNode writes required by the future MIF driver.
 * Full media bandwidth requests, other BTS scenarios and thermal control are
 * separate remaining work. No enabled device-tree node is supplied here.
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <soc/samsung/cal-if.h>
#include <soc/samsung/exynos9810-mif-bts.h>
#include "exynos9810-bus-tables.h"
#include "exynos9810-mif-bts-policy.h"

#define SNODES 4
#define QMAX_READ 0x50
#define QMAX_WRITE 0x54

static const resource_size_t snode_addresses[SNODES] = {
	0x1b1b2000, 0x1b1c2000, 0x1b1d2000, 0x1b1e2000,
};
static const char * const snode_names[SNODES] = {
	"snode0", "snode1", "snode2", "snode3",
};

struct exynos9810_mif_bts {
	struct device *dev;
	void __iomem *snode[SNODES];
	struct exynos9810_bus_table rates;
	unsigned int read_cells[EXYNOS9810_BTS_QMAX_CELLS];
	unsigned int write_cells[EXYNOS9810_BTS_QMAX_CELLS];
	unsigned int read_count;
	unsigned int write_count;
	spinlock_t lock;
	bool ready;
	bool faulted;
};

static bool active_rate(struct exynos9810_mif_bts *bts, unsigned int khz)
{
	unsigned int i;

	for (i = 0; i < bts->rates.count; i++)
		if (bts->rates.levels[i].hz == khz * 1000UL)
			return true;
	return false;
}

/* Caller holds lock. Always write all registers, including after power loss;
 * unlike the original cached comparison, no stale cache can skip restoration.
 */
static int program_qmax(struct exynos9810_mif_bts *bts, unsigned int khz)
{
	unsigned int read, write, i;
	int ret;

	if (!active_rate(bts, khz))
		return -ERANGE;
	ret = exynos9810_bts_qmax(bts->read_cells, bts->read_count, khz, &read);
	if (ret)
		return ret;
	ret = exynos9810_bts_qmax(bts->write_cells, bts->write_count, khz, &write);
	if (ret)
		return ret;
	for (i = 0; i < SNODES; i++) {
		writel(read, bts->snode[i] + QMAX_READ);
		writel(write, bts->snode[i] + QMAX_WRITE);
	}
	for (i = 0; i < SNODES; i++) {
		if (readl(bts->snode[i] + QMAX_READ) != read ||
		    readl(bts->snode[i] + QMAX_WRITE) != write) {
			bts->faulted = true;
			bts->ready = false;
			return -EIO;
		}
	}
	return 0;
}

int exynos9810_mif_bts_apply(struct exynos9810_mif_bts *bts, unsigned int khz)
{
	unsigned long flags;
	int ret;

	if (!bts)
		return -EINVAL;
	spin_lock_irqsave(&bts->lock, flags);
	ret = bts->faulted ? -EIO : !bts->ready ? -ENODEV : program_qmax(bts, khz);
	spin_unlock_irqrestore(&bts->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(exynos9810_mif_bts_apply);

static void put_supplier(void *data)
{
	put_device(data);
}

struct exynos9810_mif_bts *exynos9810_mif_bts_get(struct device *consumer)
{
	struct device_node *node;
	struct platform_device *pdev;
	struct exynos9810_mif_bts *bts;
	unsigned long flags;
	int ret;

	if (!consumer || !consumer->of_node)
		return ERR_PTR(-EINVAL);
	node = of_parse_phandle(consumer->of_node, "samsung,mif-bts", 0);
	if (!node)
		return ERR_PTR(-ENODEV);
	if (!of_device_is_compatible(node, "samsung,exynos9810-mif-bts")) {
		of_node_put(node);
		return ERR_PTR(-EINVAL);
	}
	pdev = of_find_device_by_node(node);
	of_node_put(node);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);
	if (&pdev->dev == consumer) {
		put_device(&pdev->dev);
		return ERR_PTR(-EINVAL);
	}
	/* Pin bound driver state until the managed link protects the supplier.
	 * A platform-device reference alone does not pin its managed private data.
	 */
	device_lock(&pdev->dev);
	if (!device_is_bound(&pdev->dev)) {
		ret = -EPROBE_DEFER;
		goto unlock_put;
	}
	bts = platform_get_drvdata(pdev);
	if (!bts) {
		ret = -EPROBE_DEFER;
		goto unlock_put;
	}
	spin_lock_irqsave(&bts->lock, flags);
	ret = bts->faulted ? -EIO : !bts->ready ? -EPROBE_DEFER : 0;
	spin_unlock_irqrestore(&bts->lock, flags);
	if (ret) {
		goto unlock_put;
	}
	if (!device_link_add(consumer, &pdev->dev, DL_FLAG_AUTOREMOVE_CONSUMER)) {
		ret = -ENOMEM;
		goto unlock_put;
	}
	device_unlock(&pdev->dev);
	ret = devm_add_action_or_reset(consumer, put_supplier, &pdev->dev);
	return ret ? ERR_PTR(ret) : bts;

unlock_put:
	device_unlock(&pdev->dev);
	put_device(&pdev->dev);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(exynos9810_mif_bts_get);

static int read_policy(struct device *dev, const char *name, unsigned int *cells,
		unsigned int *count, unsigned int default_mo)
{
	unsigned int ignored;
	int n, ret;

	if (!of_find_property(dev->of_node, name, NULL)) {
		cells[0] = default_mo;
		*count = 1;
		return 0;
	}
	n = of_property_count_u32_elems(dev->of_node, name);
	if (n <= 0 || n > EXYNOS9810_BTS_QMAX_CELLS || !(n & 1))
		return -EINVAL;
	ret = of_property_read_u32_array(dev->of_node, name, cells, n);
	if (ret)
		return ret;
	ret = exynos9810_bts_qmax(cells, n, 1, &ignored);
	if (!ret)
		*count = n;
	return ret;
}

static int exynos9810_mif_bts_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos9810_mif_bts *bts;
	struct resource *res;
	unsigned long rate;
	unsigned int i;
	int ret;

	if (!cal_dfs_is_ready(EXYNOS9810_MIF_CAL_ID))
		return -EPROBE_DEFER;
	bts = devm_kzalloc(dev, sizeof(*bts), GFP_KERNEL);
	if (!bts)
		return -ENOMEM;
	bts->dev = dev;
	spin_lock_init(&bts->lock);
	ret = exynos9810_bus_build_table(EXYNOS9810_MIF_CAL_ID, 208000, 2093000, &bts->rates);
	if (ret)
		return dev_err_probe(dev, ret, "calibrated MIF table unavailable\n");
	ret = read_policy(dev, "samsung,qmax-read", bts->read_cells, &bts->read_count, 0x18);
	if (ret)
		return ret;
	ret = read_policy(dev, "samsung,qmax-write", bts->write_cells, &bts->write_count, 0x6);
	if (ret)
		return ret;
	/* Complete all resource checks and mappings before the first MMIO write. */
	for (i = 0; i < SNODES; i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, snode_names[i]);
		if (!res || res->start != snode_addresses[i] || resource_size(res) != 0x400)
			return dev_err_probe(dev, -EINVAL, "incorrect/missing %s resource\n", snode_names[i]);
		bts->snode[i] = devm_ioremap_resource(dev, res);
		if (IS_ERR(bts->snode[i]))
			return PTR_ERR(bts->snode[i]);
	}
	rate = cal_dfs_get_rate(EXYNOS9810_MIF_CAL_ID);
	if (!rate || rate > INT_MAX)
		return -ERANGE;
	ret = program_qmax(bts, rate);
	if (ret)
		return dev_err_probe(dev, ret, "MIF BTS initialization/readback failed\n");
	bts->ready = true;
	platform_set_drvdata(pdev, bts);
	return 0;
}

static void exynos9810_mif_bts_remove(struct platform_device *pdev)
{
	struct exynos9810_mif_bts *bts = platform_get_drvdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&bts->lock, flags);
	bts->ready = false;
	spin_unlock_irqrestore(&bts->lock, flags);
	platform_set_drvdata(pdev, NULL);
}

static int __maybe_unused exynos9810_mif_bts_suspend(struct device *dev)
{
	struct exynos9810_mif_bts *bts = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&bts->lock, flags);
	bts->ready = false;
	spin_unlock_irqrestore(&bts->lock, flags);
	return 0;
}

static int __maybe_unused exynos9810_mif_bts_resume(struct device *dev)
{
	struct exynos9810_mif_bts *bts = dev_get_drvdata(dev);
	unsigned long flags, rate;
	int ret;

	rate = cal_dfs_get_rate(EXYNOS9810_MIF_CAL_ID);
	if (!rate || rate > INT_MAX)
		return -ERANGE;
	spin_lock_irqsave(&bts->lock, flags);
	ret = bts->faulted ? -EIO : program_qmax(bts, rate);
	bts->ready = !ret;
	spin_unlock_irqrestore(&bts->lock, flags);
	return ret;
}

static const struct dev_pm_ops exynos9810_mif_bts_pm = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(exynos9810_mif_bts_suspend, exynos9810_mif_bts_resume)
};
static const struct of_device_id exynos9810_mif_bts_match[] = {
	{ .compatible = "samsung,exynos9810-mif-bts" },
	{}
};
MODULE_DEVICE_TABLE(of, exynos9810_mif_bts_match);

static struct platform_driver exynos9810_mif_bts_driver = {
	.probe = exynos9810_mif_bts_probe,
	.remove = exynos9810_mif_bts_remove,
	.driver = {
		.name = "exynos9810-mif-bts",
		.of_match_table = exynos9810_mif_bts_match,
		.pm = &exynos9810_mif_bts_pm,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(exynos9810_mif_bts_driver);
MODULE_LICENSE("GPL");
