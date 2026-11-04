#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <soc/samsung/cal-if.h>

#include <soc/samsung/acpm_ipc_ctrl.h>
#include <linux/sched/clock.h>

#include "acpm_dvfs.h"
#include "cmucal.h"

#ifndef CONFIG_ARM_EXYNOS_DEVFREQ
#define PM_QOS_BUS_THROUGHPUT (11)
#endif

static struct acpm_dvfs acpm_dvfs;
static bool acpm_dvfs_ready;

/*
 * Firmware MIF floor requests (4.9 acpm_noti_mif_callback). The firmware
 * sends a requested MIF rate in kHz in cmd[1] on channel 5; 4.9 applied it
 * as a PM_QOS_BUS_THROUGHPUT minimum (dmesg "req 1794000 KHz", "421000 KHz").
 * There is no global bus class in 6.12, so one consumer (the MIF devfreq
 * driver) registers here and turns it into a device PM QoS minimum. The last
 * request is replayed on registration so nothing sent before the devfreq
 * driver probes is lost. The callback runs from the ACPM IRQ thread, possibly
 * under the channel's rx spinlock; the consumer must not sleep.
 */
static DEFINE_SPINLOCK(mif_noti_lock);
static exynos_acpm_mif_notify_fn mif_noti_fn;
static void *mif_noti_priv;
static unsigned int mif_noti_khz;
static unsigned long mif_noti_count;
static u64 mif_noti_ns;

static void acpm_noti_mif_callback(unsigned int *cmd, unsigned int size)
{
	unsigned long flags;

	if (size < 2 * sizeof(u32))
		return;
	pr_info_ratelimited("%s : req %u KHz\n", __func__, cmd[1]);
	spin_lock_irqsave(&mif_noti_lock, flags);
	mif_noti_khz = cmd[1];
	mif_noti_count++;
	mif_noti_ns = local_clock();
	if (mif_noti_fn)
		mif_noti_fn(mif_noti_priv, mif_noti_khz);
	spin_unlock_irqrestore(&mif_noti_lock, flags);
}

int exynos_acpm_dvfs_mif_notify_register(exynos_acpm_mif_notify_fn fn,
					 void *priv)
{
	unsigned long flags;
	int ret = 0;

	if (!fn)
		return -EINVAL;
	spin_lock_irqsave(&mif_noti_lock, flags);
	if (mif_noti_fn) {
		ret = -EBUSY;
	} else {
		mif_noti_fn = fn;
		mif_noti_priv = priv;
		if (mif_noti_count)
			fn(priv, mif_noti_khz);
	}
	spin_unlock_irqrestore(&mif_noti_lock, flags);
	return ret;
}

/* After return no callback is running or will start; flush your own work. */
void exynos_acpm_dvfs_mif_notify_unregister(void)
{
	unsigned long flags;

	spin_lock_irqsave(&mif_noti_lock, flags);
	mif_noti_fn = NULL;
	mif_noti_priv = NULL;
	spin_unlock_irqrestore(&mif_noti_lock, flags);
}

void exynos_acpm_dvfs_mif_notify_stats(unsigned long *count,
				       unsigned int *last_khz, u64 *last_ns)
{
	unsigned long flags;

	spin_lock_irqsave(&mif_noti_lock, flags);
	*count = mif_noti_count;
	*last_khz = mif_noti_khz;
	*last_ns = mif_noti_ns;
	spin_unlock_irqrestore(&mif_noti_lock, flags);
}

int exynos_acpm_set_rate(unsigned int id, unsigned long rate)
{
	struct ipc_config config = { 0 };
	unsigned int cmd[4];
	unsigned long long before, after, latency;
	int ret;

	if (!smp_load_acquire(&acpm_dvfs_ready))
		return -ENODEV;
	config.cmd = cmd;
	config.response = true;
	config.indirection = false;
	config.cmd[0] = id;
	config.cmd[1] = (unsigned int)rate;
	config.cmd[2] = FREQ_REQ;
	config.cmd[3] = 0;

	before = sched_clock();
	ret = acpm_ipc_send_data(acpm_dvfs.ch_num, &config);
	after = sched_clock();
	latency = after - before;
	if (ret)
		pr_err("%s:[%d] latency = %llu ret = %d",
			__func__, id, latency, ret);

	return ret;
}

unsigned long exynos_acpm_get_rate(unsigned int id)
{
	struct ipc_config config = { 0 };
	unsigned int cmd[4];
	unsigned long long before, after, latency;
	int ret;

	if (!smp_load_acquire(&acpm_dvfs_ready))
		return 0;
	config.cmd = cmd;
	config.response = true;
	config.indirection = false;
	config.cmd[0] = id;
	config.cmd[1] = 0;
	config.cmd[2] = FREQ_GET;
	config.cmd[3] = 0;

	before = sched_clock();
	ret = acpm_ipc_send_data(acpm_dvfs.ch_num, &config);
	after = sched_clock();
	latency = after - before;
	if (ret)
		pr_err("%s:[%d] latency = %llu ret = %d", __func__,
			id, latency, ret);

	return config.cmd[1];
}

char margin_list[MAX_MARGIN_ID][10] = {"MIF", "INT", "BIG", "LIT", "G3D",
		"INTCAM", "CAM", "DISP", "G3DM",
		"CP", "FSYS0", "AUD", "IVA", "SCORE"};

int exynos_acpm_set_volt_margin(unsigned int id, int volt)
{
	struct ipc_config config = { 0 };
	unsigned int cmd[4];
	unsigned long long before, after, latency;
	int ret;
	struct vclk *vclk;

	if (!smp_load_acquire(&acpm_dvfs_ready))
		return -ENODEV;
	config.cmd = cmd;
	config.response = true;
	config.indirection = false;
	config.cmd[0] = id;
	config.cmd[1] = volt;
	config.cmd[2] = MARGIN_REQ;
	config.cmd[3] = 0;

	before = sched_clock();
	ret = acpm_ipc_send_data(acpm_dvfs.ch_num, &config);
	after = sched_clock();
	latency = after - before;
	if (ret)
		pr_err("%s:[%d] latency = %llu ret = %d",
			__func__, id, latency, ret);

	vclk = cmucal_get_node(id);
	pr_info("%s: [%s] +margin %d uV\n", __func__, margin_list[vclk->margin_id], volt);

	return ret;
}

#ifdef CONFIG_EXYNOS9810_LEGACY_THERMAL_NOTIFIERS
int exynos_acpm_set_cold_temp(unsigned int id, bool is_cold_temp)
{
	struct ipc_config config = { 0 };
	unsigned int cmd[4];
	unsigned long long before, after, latency;
	int ret;

	if (!smp_load_acquire(&acpm_dvfs_ready))
		return -ENODEV;
	config.cmd = cmd;
	config.response = true;
	config.indirection = false;
	config.cmd[0] = id;
	config.cmd[1] = is_cold_temp;
	config.cmd[2] = COLDTEMP_REQ;
	config.cmd[3] = 0;

	before = sched_clock();
	ret = acpm_ipc_send_data(acpm_dvfs.ch_num, &config);
	after = sched_clock();
	latency = after - before;
	if (ret)
		pr_err("%s:[%d] latency = %llu ret = %d",
			__func__, id, latency, ret);

	return ret;
}
#endif


#ifdef CONFIG_EXYNOS9810_LEGACY_THERMAL_NOTIFIERS
static int acpm_cpu_tmu_notifier(struct notifier_block *nb, unsigned long event, void *v)
{
	unsigned int *is_cold_temp = v;
	int i;
	int ret = NOTIFY_OK;

	if (event != TMU_COLD)
		return ret;

	if (!acpm_dvfs.cpu_coldtemp)
		return ret;

	for (i = 0; i < acpm_dvfs.cpu_len; i++)
		exynos_acpm_set_cold_temp(GET_IDX(acpm_dvfs.cpu_coldtemp[i]),
					*is_cold_temp);

	return NOTIFY_OK;
}
#endif

#ifdef CONFIG_EXYNOS9810_LEGACY_THERMAL_NOTIFIERS
static int acpm_gpu_tmu_notifier(struct notifier_block *nb, unsigned long event, void *v)
{
	unsigned int *is_cold_temp = v;
	int i;
	int ret = NOTIFY_OK;

	if (event != GPU_COLD && event != GPU_NORMAL)
		return ret;

	if (!acpm_dvfs.gpu_coldtemp)
		return ret;

	for (i = 0; i < acpm_dvfs.gpu_len; i++)
		exynos_acpm_set_cold_temp(GET_IDX(acpm_dvfs.gpu_coldtemp[i]),
					*is_cold_temp);

	return ret;
}
#endif

#ifdef CONFIG_EXYNOS9810_LEGACY_THERMAL_NOTIFIERS
static void acpm_dvfs_get_cpu_cold_temp_list(struct device *dev)
{
	struct device_node *node = dev->of_node;
	unsigned int proplen;

	proplen = of_property_count_u32_elems(node, "cpu_cold_temp_list");

	if (!proplen)
		return;

	acpm_dvfs.cpu_coldtemp = kcalloc(proplen, sizeof(u32), GFP_KERNEL);

	if (!acpm_dvfs.cpu_coldtemp) {
		pr_err("acpm_dvfs memory allocation fail\n");
		return;
	}

	of_property_read_u32_array(node, "cpu_cold_temp_list",
					acpm_dvfs.cpu_coldtemp, proplen);

	acpm_dvfs.cpu_len = proplen;

	acpm_dvfs.cpu_tmu_notifier.notifier_call = acpm_cpu_tmu_notifier;
	if (exynos_tmu_add_notifier(&acpm_dvfs.cpu_tmu_notifier))
		dev_err(dev, "failed register cpu tmu notifier\n");

}
#endif

#ifdef CONFIG_EXYNOS9810_LEGACY_THERMAL_NOTIFIERS
static void acpm_dvfs_get_gpu_cold_temp_list(struct device *dev)
{
	struct device_node *node = dev->of_node;
	int proplen;

	proplen = of_property_count_u32_elems(node, "gpu_cold_temp_list");

	if (!proplen)
		return;

	acpm_dvfs.gpu_coldtemp = kcalloc(proplen, sizeof(u32), GFP_KERNEL);
	if (!acpm_dvfs.gpu_coldtemp) {
		pr_err("acpm_dvfs memory allocation fail\n");
		return;
	}

	of_property_read_u32_array(node, "gpu_cold_temp_list",
					acpm_dvfs.gpu_coldtemp, proplen);

	acpm_dvfs.gpu_len = proplen;

	acpm_dvfs.gpu_tmu_notifier.notifier_call = acpm_gpu_tmu_notifier;
	if (exynos_gpu_add_notifier(&acpm_dvfs.gpu_tmu_notifier))
		dev_err(dev, "failed register gpu tmu notifier\n");
}
#endif

bool exynos_acpm_dvfs_is_ready(void)
{
	return smp_load_acquire(&acpm_dvfs_ready);
}

#define ACPM_DVFS_SELFTEST_LIT	0x0b040002
#define ACPM_DVFS_SELFTEST_BIG	0x0b040003
#define ACPM_DVFS_SELFTEST_KHZ	832000	/* LITTLE ASV step below the 1053000 boot rate */

static struct delayed_work acpm_dvfs_selftest_work;

/*
 * Diagnostic only, run late so the host log forwarder keeps it. Reads both
 * CPU domains from CMU registers (the 4.9 read path) and performs one LITTLE
 * FREQ_REQ round trip through cal_dfs_set_rate(), which only accepts rates in
 * the domain's ASV table. The firmware selects voltage. BIG is never set.
 */
static void acpm_dvfs_selftest(struct work_struct *work)
{
	unsigned long lit0, lit1, lit2, big0;
	int ret1 = -EINVAL, ret2 = -EINVAL;

	pr_info("acpm-dvfs selftest: ready lit=%d big=%d\n",
		cal_dfs_is_ready(ACPM_DVFS_SELFTEST_LIT),
		cal_dfs_is_ready(ACPM_DVFS_SELFTEST_BIG));
	lit0 = cal_dfs_get_rate(ACPM_DVFS_SELFTEST_LIT);
	big0 = cal_dfs_get_rate(ACPM_DVFS_SELFTEST_BIG);
	pr_info("acpm-dvfs selftest: cmu lit=%lu big=%lu\n", lit0, big0);
	if (!cal_dfs_is_ready(ACPM_DVFS_SELFTEST_LIT)) {
		pr_info("acpm-dvfs selftest: LITTLE not ready; no FREQ_REQ sent\n");
		return;
	}
	ret1 = cal_dfs_set_rate(ACPM_DVFS_SELFTEST_LIT, ACPM_DVFS_SELFTEST_KHZ);
	lit1 = cal_dfs_get_rate(ACPM_DVFS_SELFTEST_LIT);
	pr_info("acpm-dvfs selftest: set lit %u ret=%d cmu=%lu\n",
		ACPM_DVFS_SELFTEST_KHZ, ret1, lit1);
	if (lit0 && lit0 != ACPM_DVFS_SELFTEST_KHZ) {
		ret2 = cal_dfs_set_rate(ACPM_DVFS_SELFTEST_LIT, lit0);
		lit2 = cal_dfs_get_rate(ACPM_DVFS_SELFTEST_LIT);
		pr_info("acpm-dvfs selftest: restore lit %lu ret=%d cmu=%lu\n",
			lit0, ret2, lit2);
	}
}

/*
 * Samsung's 4.9 driver sends every DVFS command (FREQ_REQ/FREQ_GET/MARGIN_REQ)
 * on channel 0: exynos_acpm_dvfs_init() requests a channel for the clock
 * controller node, which has no acpm-ipc-channel property, so ch_num keeps
 * its static zero. The acpm-dvfs node's own channel (5, TYPE_BUFFER) is the
 * firmware's MIF-floor notification channel and is never used for sending.
 * Describe the command channel explicitly with a "command" child node.
 * The notification handler is attached last, once the probe cannot fail, so a
 * deferred probe never leaves a second copy on the channel's callback list.
 */
static int acpm_dvfs_probe(struct platform_device *pdev)
{
	struct device_node *cmd_np;
	unsigned int noti_ch, noti_size;
	static const u32 selftest_ids[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
	int ret, i;

	acpm_dvfs.dev = pdev->dev.of_node;
	/* Notification channel: check it exists before anything else. */
	ret = acpm_ipc_request_channel(acpm_dvfs.dev, NULL, &noti_ch, &noti_size);
	if (ret)
		return ret == -ENODEV ? -EPROBE_DEFER : ret;

	cmd_np = of_get_child_by_name(pdev->dev.of_node, "command");
	if (!cmd_np) {
		dev_err(&pdev->dev, "missing DVFS command channel node\n");
		return -EINVAL;
	}
	ret = acpm_ipc_request_channel(cmd_np, NULL, &acpm_dvfs.ch_num,
				       &acpm_dvfs.size);
	of_node_put(cmd_np);
	if (ret)
		return ret == -ENODEV ? -EPROBE_DEFER : ret;
	if (acpm_dvfs.size != 4 * sizeof(u32)) {
		dev_err(&pdev->dev, "DVFS command channel %u element %u, want 16\n",
			acpm_dvfs.ch_num, acpm_dvfs.size);
		return -EPROTO;
	}
	ret = acpm_ipc_request_channel(acpm_dvfs.dev, acpm_noti_mif_callback,
				       &noti_ch, &noti_size);
	if (ret)
		return ret == -ENODEV ? -EPROBE_DEFER : ret;
	dev_info(&pdev->dev, "DVFS commands on channel %u, MIF notifications on channel %u\n",
		 acpm_dvfs.ch_num, noti_ch);
	smp_store_release(&acpm_dvfs_ready, true);

	if (of_property_read_bool(pdev->dev.of_node, "samsung,freq-get-selftest")) {
		/* Firmware FREQ_GET is unused by 4.9 and answers 0; log once. */
		for (i = 0; i < ARRAY_SIZE(selftest_ids); i++)
			dev_info(&pdev->dev, "FREQ_GET id=%u rate=%lu\n",
				 selftest_ids[i],
				 exynos_acpm_get_rate(selftest_ids[i]));
		INIT_DELAYED_WORK(&acpm_dvfs_selftest_work, acpm_dvfs_selftest);
		schedule_delayed_work(&acpm_dvfs_selftest_work,
				      msecs_to_jiffies(100000) > jiffies - INITIAL_JIFFIES ?
				      msecs_to_jiffies(100000) - (jiffies - INITIAL_JIFFIES) : 0);
	}
	return 0;
}

static void acpm_dvfs_remove(struct platform_device *pdev)
{
	smp_store_release(&acpm_dvfs_ready, false);
}

static const struct of_device_id acpm_dvfs_match[] = {
	{ .compatible = "samsung,exynos-acpm-dvfs" },
	{},
};

static struct platform_driver samsung_acpm_dvfs_driver = {
	.probe	= acpm_dvfs_probe,
	.remove	= acpm_dvfs_remove,
	.driver	= {
		.name = "exynos-acpm-dvfs",
		.owner	= THIS_MODULE,
		.of_match_table	= acpm_dvfs_match,
	},
};

static int __init exynos_acpm_dvfs_init(void)
{
	return platform_driver_register(&samsung_acpm_dvfs_driver);
}
fs_initcall(exynos_acpm_dvfs_init);

void __init exynos_acpm_set_device(void *dev)
{
	acpm_dvfs.dev = dev;
}
