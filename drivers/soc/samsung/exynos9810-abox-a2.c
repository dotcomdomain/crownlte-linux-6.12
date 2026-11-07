// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos9810 (crownlte) audio stage A2: boot the ABOX ("Calliope") audio
 * DSP firmware in volatile DSP RAM, check that it answers, stop it again.
 *
 * This is the minimal boot path of the Samsung 4.9 ABOX driver, not the
 * driver itself (no ASoC, no DMA/UAIF/DSIF, no codec, no amps, no PM QoS):
 *   4.9 sound/soc/samsung/abox/abox.c  samsung_abox_probe() (IOMMU part),
 *       abox_enable(), abox_download_firmware(), abox_cpu_power(),
 *       abox_cpu_enable(), abox_request_dram_on(), abox_irq_handler(),
 *       abox_system_ipc_handler(), __abox_process_ipc(),
 *       abox_cpu_pm_ipc(), abox_disable()
 *   4.9 sound/soc/samsung/abox/abox_gic.c  abox_gic_init_gic(),
 *       abox_gic_irq_handler(), abox_gic_generate_interrupt()
 *   4.9 include/sound/samsung/abox_ipc.h  message layout
 * See Documentation/crownlte/audio-a2.md.
 *
 * Two platform drivers:
 *
 *  "samsung,exynos9810-abox-a2" (the gate, no registers of its own): at
 *  probe it installs and verifies the CL0 watchdog deadline, reads the
 *  pd-aud power status in the always-on PMU, then takes the five AUD
 *  clocks through the clock framework (first pd-aud access: CMU_AUD rate
 *  reads, the 4.9 AUDIF rate). Only then does it open the guard
 *  exynos9810_abox_a2_guard() for the ABOX SysMMU and the ABOX node.
 *
 *  "samsung,exynos9810-abox-boot" (the ABOX): at probe it only maps its
 *  windows, allocates the firmware DRAM/IVA buffers and builds its own
 *  IOMMU domain with the fixed firmware IOVAs (no register access: the
 *  SysMMU is runtime-suspended until the self-test). The self-test runs
 *  once per boot at samsung,abox-a2-selftest-ms on a CPU other than CPU0:
 *  firmware from /vendor/firmware (user-helper fallback, ueventd), SysMMU
 *  on, then per boot: ABOX GIC init, SYSREG MISC_CON, firmware copy,
 *  SYSPOWER_CTRL, PMU ABOX CPU power + enable, wait for "Calliope is ready
 *  to sing", ABOX_RESUME, run, ABOX_SUSPEND, wait for WFI, CPU enable and
 *  power off (the 4.9 order of the 8895 secure-boot path). The PMU
 *  registers are left as found.
 *
 * Not touched: codec, amps, PMIC, /efs, UAIF/DSIF pins and pad retention,
 * pd-aud power (no DTZPC SMC: pd-aud stays on as SBoot left it), AUD DVFS.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-map-ops.h>
#include <linux/firmware.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/ktime.h>
#include <linux/mfd/syscon.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <soc/samsung/crownlte-wdt.h>
#include <soc/samsung/exynos9810-abox-a2.h>

#define A2 "audio selftest A2: "

/* PMU (0x14060000, always on): 4.9 abox.c CONFIG_SOC_EXYNOS9810 block */
#define PMU_AUD_CONFIGURATION		0x4000
#define PMU_AUD_STATUS			0x4004
#define  PMU_AUD_STATUS_ON		0xf
#define PMU_GPIO_MODE_ABOX_SYS_PWR	0x1424
#define PMU_PAD_RETENTION_ABOX_OPTION	0x4170
#define PMU_ABOX_CPU_CONFIGURATION	0x415c
#define  ABOX_CPU_LOCAL_PWR_CFG		BIT(0)
#define PMU_ABOX_CPU_STATUS		0x4160
#define  ABOX_CPU_STATUS_ON		BIT(0)
#define PMU_ABOX_CPU_OPTION		0x4164
#define  ABOX_CPU_OPTION_ENABLE_CPU	BIT(28)
#define PMU_ABOX_CPU_STANDBY		0x3804
#define  ABOX_CPU_STANDBY_WFI		BIT(28)

/* ABOX SFR (4.9 abox.h) */
#define ABOX_VERSION			0x0004
#define ABOX_SYSPOWER_CTRL		0x0010
#define ABOX_SYSPOWER_STATUS		0x0014
#define ABOX_CPU_R(x)			(0x2c00 + (x) * 4)
#define ABOX_CPU_PC			0x2c3c
#define ABOX_CPU_L2C_STATUS		0x2c40
/* SYSREG_AUD */
#define ABOX_SYSREG_MISC_CON		0x032c

/* firmware layout (4.9 abox.h), fixed by the firmware */
#define DRAM_FIRMWARE_SIZE		(SZ_16M + SZ_2M)
#define IOVA_DRAM_FIRMWARE		0x80000000UL
#define IVA_FIRMWARE_SIZE		SZ_512K
#define IOVA_IVA_FIRMWARE		0x90000000UL
#define IOVA_CHIPID			0x10000000UL	/* 4.9: identity */
#define AUDIF_RATE_HZ			24576000

/*
 * Chip-ID registers copied into the snapshot page at IOVA_CHIPID: the ones
 * the 4.9 exynos9810 chipid driver reads (PRODUCT_ID, unique ID low/high,
 * revision). The CPU reads them once at probe; the DSP never sees the real
 * chip-ID page. Every other offset of the snapshot page reads 0.
 */
#define CHIPID_PRODUCT_ID		0x00
#define CHIPID_UNIQUE_LO		0x04
#define CHIPID_UNIQUE_HI		0x08
#define CHIPID_REV			0x10
#define CHIPID_WINDOW			0x14
static const u16 a2_chipid_regs[] = {
	CHIPID_PRODUCT_ID, CHIPID_UNIQUE_LO, CHIPID_UNIQUE_HI, CHIPID_REV,
};

/* IPC (4.9 include/sound/samsung/abox_ipc.h) */
#define IPC_SYSTEM			1
enum {
	ABOX_SUSPEND = 1,
	ABOX_RESUME,
	ABOX_BOOT_DONE,
	ABOX_CHANGE_GEAR,
	ABOX_START_L2C_CONTROL,
	ABOX_END_L2C_CONTROL,
	ABOX_REQUEST_SYSCLK,
	ABOX_REQUEST_L2C,
	ABOX_CHANGED_GEAR,
	ABOX_REPORT_LOG = 0x10,
	ABOX_FLUSH_LOG,
	ABOX_REPORT_DUMP = 0x20,
	ABOX_FLUSH_DUMP = 0x22,
	ABOX_REPORT_COMPONENT = 0xc0,
	ABOX_REPORT_FAULT = 0xfa,
};

/* ABOX_IPC_MSG with the IPC_SYSTEM_MSG member: 764 bytes (tx_ack - tx) */
struct abox_a2_msg {
	u32 ipcid;
	u32 task_id;
	u32 msgtype;
	s32 param1;
	s32 param2;
	s32 param3;
	u8 bundle[740];
};
static_assert(sizeof(struct abox_a2_msg) == 0x2fc);

/* 4.9: CALLIOPE_ENABLE_TIMEOUT_MS, IPC_TIMEOUT_US; boot wait from the design */
#define A2_CPU_ON_TIMEOUT_US		100000
#define A2_BOOT_TIMEOUT_MS		5000
#define A2_BOOT_PASS_MS			2000
#define A2_IPC_TIMEOUT_US		10000
#define A2_WFI_TIMEOUT_US		100000
#define A2_CPU_OFF_TIMEOUT_US		100000
#define A2_MAX_EXT_FW			8
#define A2_LOG_TAIL			768
#define A2_COMPAT_MSG			0x30040	/* SRAM, 4.9 */

/* the gate: deadline window and self-test start */
#define A2_DEADLINE_MIN_MS		121000
#define A2_DEADLINE_MAX_MS		900000
#define A2_START_MIN_MS			20000
#define A2_START_MAX_MS			300000
#define A2_TEST_MARGIN_MS		60000	/* start + margin <= deadline */
#define A2_GUARD_MARGIN_MS		20000
#define A2_WDT_LIVE_TRIES		50

enum { A2_PENDING, A2_ARMED, A2_REFUSED };

enum { CLK_PLL, CLK_CPU, CLK_AUDIF, CLK_DMIC, CLK_BUS, CLK_NUM };
static const char * const a2_clk_names[CLK_NUM] = {
	"pll", "cpu", "audif", "dmic", "bus",
};

static struct {
	struct device_node *np;
	struct regmap *pmu;
	u32 deadline_ms, start_ms;
	unsigned int cpu;		/* nonzero CPU for the self-test */
	struct hrtimer deadline;
	bool hrtimer_on;
	struct clk *clk[CLK_NUM];
	const char *why;
	int state;
} gate;

/* ---- watchdog deadline (pattern of exynos9810-audio-a1.c / -pm.c) ------- */

static enum hrtimer_restart a2_deadline_fn(struct hrtimer *t)
{
	pr_emerg(A2 "deadline %u ms reached and the system is still up: stopping watchdog petting\n",
		 gate.deadline_ms);
	s3c2410wdt_crownlte_stop_petting("audio A2 deadline");
	return HRTIMER_NORESTART;
}

static bool a2_wdt_state_ok(const struct crownlte_wdt_state *s)
{
	return s->enabled && s->reset && !s->pet_stopped && s->deadline_ns &&
	       s->deadline_ns <= (u64)gate.deadline_ms * NSEC_PER_MSEC;
}

/* NULL if the deadline is in the ping path of a live, petted, resetting WDT */
static const char *a2_wdt_verify(const char *when)
{
	struct crownlte_wdt_state a, b;
	unsigned int i;

	if (!s3c2410wdt_crownlte_state(&a))
		return "no guarded CL0 watchdog";
	if (!a.enabled || !a.reset)
		return "CL0 watchdog not running with the reset enabled";
	if (a.pet_stopped)
		return "CL0 watchdog petting stopped or deadline passed";
	if (!a2_wdt_state_ok(&a))
		return "watchdog deadline not installed";
	for (i = 0; i < A2_WDT_LIVE_TRIES; i++) {
		usleep_range(2000, 3000);
		if (!s3c2410wdt_crownlte_state(&b) || !a2_wdt_state_ok(&b))
			return "CL0 watchdog stopped while being checked";
		if (b.wtcnt != a.wtcnt) {
			pr_info(A2 "wdt (%s): deadline %llu ms armed (WTCON=%#x timeout %u s, counter moving)\n",
				when, div_u64(b.deadline_ns, NSEC_PER_MSEC),
				b.wtcon, b.timeout);
			return NULL;
		}
	}
	return "CL0 watchdog counter not moving";
}

/* still armed and at least @margin_ms before the deadline */
static const char *a2_wdt_live(u32 margin_ms)
{
	struct crownlte_wdt_state st;

	if (!s3c2410wdt_crownlte_state(&st) || !a2_wdt_state_ok(&st))
		return "CL0 watchdog deadline no longer armed";
	if (ktime_get_boottime_ns() + (u64)margin_ms * NSEC_PER_MSEC >= st.deadline_ns)
		return "too close to the watchdog deadline";
	return NULL;
}

/* ---- guard -------------------------------------------------------------- */

int exynos9810_abox_a2_guard(struct device *consumer)
{
	struct device_node *np;
	const char *why;
	bool ok_node;
	int state;

	np = of_parse_phandle(consumer->of_node, "samsung,abox-a2-guard", 0);
	if (!np) {
		dev_err(consumer, "audio A2 guard: no samsung,abox-a2-guard phandle: refused\n");
		return -EINVAL;
	}
	ok_node = of_device_is_compatible(np, "samsung,exynos9810-abox-a2") &&
		  of_device_is_available(np);
	if (!ok_node)
		dev_err(consumer, "audio A2 guard: %pOF is not an enabled A2 gate: refused\n", np);
	of_node_put(np);	/* only compared as a pointer below */
	if (!ok_node)
		return -EINVAL;

	state = smp_load_acquire(&gate.state);
	if (state == A2_PENDING)
		return -EPROBE_DEFER;
	if (state != A2_ARMED || np != gate.np) {
		dev_err(consumer, "audio A2 guard: gate refused (%s): not coming up in this boot\n",
			state != A2_ARMED ? gate.why : "other A2 gate node");
		return -EPERM;
	}
	why = a2_wdt_live(A2_GUARD_MARGIN_MS);
	if (why) {
		dev_err(consumer, "audio A2 guard: %s: refused\n", why);
		return -EPERM;
	}
	dev_info(consumer, "audio A2 guard: watchdog deadline %u ms armed, pd-aud on, AUD clocks held: allowed at %llu ms\n",
		 gate.deadline_ms, div_u64(ktime_get_boottime_ns(), NSEC_PER_MSEC));
	return 0;
}
EXPORT_SYMBOL_GPL(exynos9810_abox_a2_guard);

static int a2_gate_refuse(struct device *dev, const char *why)
{
	gate.why = why;
	smp_store_release(&gate.state, A2_REFUSED);
	dev_err(dev, "audio A2 refused (%s): ABOX SysMMU and ABOX stay down for this boot\n", why);
	return 0;	/* bound, guard closed */
}

static void a2_log_rates(struct device *dev, const char *when)
{
	dev_info(dev, "AUD clocks (%s): pll %lu cpu %lu audif %lu dmic %lu bus %lu Hz\n", when,
		 clk_get_rate(gate.clk[CLK_PLL]), clk_get_rate(gate.clk[CLK_CPU]),
		 clk_get_rate(gate.clk[CLK_AUDIF]), clk_get_rate(gate.clk[CLK_DMIC]),
		 clk_get_rate(gate.clk[CLK_BUS]));
}

static int a2_gate_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	u32 aud_cfg, aud_st;
	const char *why;
	int i, ret;

	if (gate.np && gate.np != np)
		return dev_err_probe(dev, -EBUSY, "only one A2 gate is supported\n");
	gate.np = np;

	if (of_property_read_u32(np, "samsung,abox-a2-deadline-ms", &gate.deadline_ms) ||
	    gate.deadline_ms < A2_DEADLINE_MIN_MS || gate.deadline_ms > A2_DEADLINE_MAX_MS)
		return a2_gate_refuse(dev, "samsung,abox-a2-deadline-ms missing or out of range");
	if (of_property_read_u32(np, "samsung,abox-a2-selftest-ms", &gate.start_ms) ||
	    gate.start_ms < A2_START_MIN_MS || gate.start_ms > A2_START_MAX_MS)
		return a2_gate_refuse(dev, "samsung,abox-a2-selftest-ms missing or out of range");
	if (gate.start_ms + A2_TEST_MARGIN_MS > gate.deadline_ms)
		return a2_gate_refuse(dev, "self-test too close to the watchdog deadline");

	gate.cpu = cpumask_next(0, cpu_online_mask);
	if (gate.cpu >= nr_cpu_ids)
		return a2_gate_refuse(dev, "no CPU other than CPU0 online");

	gate.pmu = syscon_regmap_lookup_by_phandle(np, "samsung,pmu-syscon");
	if (gate.pmu == ERR_PTR(-EPROBE_DEFER))
		return dev_err_probe(dev, -EPROBE_DEFER, "PMU syscon\n");
	if (IS_ERR(gate.pmu))
		return a2_gate_refuse(dev, "PMU syscon not found");

	for (i = 0; i < CLK_NUM; i++) {
		gate.clk[i] = devm_clk_get(dev, a2_clk_names[i]);
		if (gate.clk[i] == ERR_PTR(-EPROBE_DEFER))
			return dev_err_probe(dev, -EPROBE_DEFER, "clock %s\n", a2_clk_names[i]);
		if (IS_ERR(gate.clk[i]))
			return a2_gate_refuse(dev, "AUD clock missing in DT");
	}

	/* 1. the deadline goes in before the first pd-aud access */
	ret = s3c2410wdt_crownlte_set_deadline((u64)gate.deadline_ms * NSEC_PER_MSEC);
	if (ret == -ENODEV)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "waiting for the guarded CL0 watchdog (ABOX held back)\n");
	if (ret)
		return a2_gate_refuse(dev, "watchdog refused the deadline");
	if (!gate.hrtimer_on) {
		hrtimer_init(&gate.deadline, CLOCK_BOOTTIME, HRTIMER_MODE_ABS);
		gate.deadline.function = a2_deadline_fn;
		hrtimer_start(&gate.deadline, ms_to_ktime(gate.deadline_ms), HRTIMER_MODE_ABS);
		gate.hrtimer_on = true;
	}
	why = a2_wdt_verify("probe");
	if (why)
		return a2_gate_refuse(dev, why);

	/* 2. pd-aud power status in the always-on PMU (not inside pd-aud) */
	if (regmap_read(gate.pmu, PMU_AUD_CONFIGURATION, &aud_cfg) ||
	    regmap_read(gate.pmu, PMU_AUD_STATUS, &aud_st))
		return a2_gate_refuse(dev, "PMU read failed");
	dev_info(dev, "pd-aud: AUD_CONFIGURATION=%#x AUD_STATUS=%#x (%s)\n", aud_cfg, aud_st,
		 (aud_st & PMU_AUD_STATUS_ON) == PMU_AUD_STATUS_ON ? "on" : "NOT on");
	if ((aud_st & PMU_AUD_STATUS_ON) != PMU_AUD_STATUS_ON)
		return a2_gate_refuse(dev, "pd-aud is not on (A2 never powers it)");

	/*
	 * 3. AUD clocks through the clock framework, as 4.9 abox probe/enable:
	 * prepare all five, set AUDIF to 24.576 MHz, enable cpu and audif.
	 * On this tree "cpu" (GATE_ABOX_DUMMY) is a HWACG Q-channel gate whose
	 * enable is a no-op and enabling the AUDIF divider is a no-op too;
	 * the rate reads go through CAL into CMU_AUD (inside pd-aud).
	 */
	a2_log_rates(dev, "before");
	if (!clk_get_rate(gate.clk[CLK_PLL]) || !clk_get_rate(gate.clk[CLK_BUS]))
		return a2_gate_refuse(dev, "PLL_AUD or the ABOX bus clock reads 0 Hz");
	ret = clk_set_rate(gate.clk[CLK_AUDIF], AUDIF_RATE_HZ);
	if (ret)
		dev_warn(dev, "AUDIF rate %u Hz not set (%d): left as found\n", AUDIF_RATE_HZ, ret);
	ret = clk_prepare_enable(gate.clk[CLK_CPU]);
	if (ret)
		return a2_gate_refuse(dev, "ABOX cpu clock enable failed");
	ret = clk_prepare_enable(gate.clk[CLK_AUDIF]);
	if (ret) {
		clk_disable_unprepare(gate.clk[CLK_CPU]);
		return a2_gate_refuse(dev, "AUDIF clock enable failed");
	}
	a2_log_rates(dev, "after");

	smp_store_release(&gate.state, A2_ARMED);
	dev_info(dev, "audio A2 gate open: watchdog deadline %u ms, self-test at ~%u ms on CPU%u\n",
		 gate.deadline_ms, gate.start_ms, gate.cpu);
	return 0;
}

/* ---- the ABOX ----------------------------------------------------------- */

struct a2_chunk {
	struct page *page;
	unsigned int order;
};

struct abox_a2 {
	struct device *dev;
	void __iomem *sfr, *sysreg, *sram, *gicd, *gicc;
	resource_size_t sram_size;
	u32 tx, rx, tx_ack, rx_ack;
	int irq;
	bool irq_enabled;

	/* firmware buffers: chunks mapped at fixed IOVAs, NC kernel view */
	struct a2_chunk *dram_chunks;
	unsigned int dram_nchunks;
	void *dram;
	struct a2_chunk iva_chunk;
	void *iva;
	struct page *chipid_page;	/* RAM snapshot mapped at IOVA_CHIPID */
	struct iommu_domain *domain;
	bool attached;

	const char *ext_name[A2_MAX_EXT_FW];
	u32 ext_off[A2_MAX_EXT_FW];
	int ext_num;
	const struct firmware *fw_sram, *fw_dram, *fw_iva, *fw_ext[A2_MAX_EXT_FW];

	u32 boots_wanted, run_ms;
	struct delayed_work work;
	struct completion boot_done;
	spinlock_t ipc_lock;

	/* written by the IRQ handlers */
	u32 version;
	bool incompatible;
	u32 log_addr;
	atomic_t ipc_rx, faults_fw, faults_mmu, spi_irqs;

	bool dsp_running;	/* ABOX CPU may run (enabled and not seen off) */
	bool rpm_held;		/* SysMMU translating on our behalf */
	bool aborting;
	u32 pmu_cfg0, pmu_opt0;	/* PMU state as found */
	char result[160];
};

static void a2_dump_gpr(struct abox_a2 *a, const char *why)
{
	int i;

	dev_info(a->dev, "ABOX CPU registers (%s):\n", why);
	for (i = 0; i < 15; i += 3)	/* R0..R14 */
		dev_info(a->dev, "  R%02d %08x R%02d %08x R%02d %08x\n",
			 i, readl(a->sfr + ABOX_CPU_R(i)),
			 i + 1, readl(a->sfr + ABOX_CPU_R(i + 1)),
			 i + 2, readl(a->sfr + ABOX_CPU_R(i + 2)));
	dev_info(a->dev, "  PC %08x L2C_STATUS %08x\n",
		 readl(a->sfr + ABOX_CPU_PC), readl(a->sfr + ABOX_CPU_L2C_STATUS));
}

static int a2_iommu_fault(struct iommu_domain *domain, struct device *dev,
			  unsigned long iova, int flags, void *token)
{
	struct abox_a2 *a = token;
	int n = atomic_inc_return(&a->faults_mmu);

	if (n <= 4) {
		dev_err(dev, "abox_iommu_fault_handler: IOVA %#lx %s (fault #%d)\n", iova,
			flags & IOMMU_FAULT_WRITE ? "write" : "read", n);
		if (READ_ONCE(a->dsp_running))
			a2_dump_gpr(a, "SysMMU fault");
	}
	return 0;
}

static void a2_system_msg(struct abox_a2 *a, const struct abox_a2_msg *m)
{
	char v[4];

	switch (m->msgtype) {
	case ABOX_BOOT_DONE:
		/* 4.9 abox_is_calliope_incompatible(): param3 of the msg at 0x30040 */
		if (a->sram_size >= A2_COMPAT_MSG + 24 &&
		    (readl(a->sram + A2_COMPAT_MSG + 20) >> 24) == 'A') {
			WRITE_ONCE(a->incompatible, true);
			dev_err(a->dev, "Calliope is not compatible with the driver\n");
		}
		WRITE_ONCE(a->version, (u32)m->param3);
		memcpy(v, &m->param3, sizeof(v));
		dev_info(a->dev, "Calliope is ready to sing (version:%c%c%c%c)\n",
			 v[3], v[2], v[1], v[0]);
		complete(&a->boot_done);
		break;
	case ABOX_REPORT_LOG:
		WRITE_ONCE(a->log_addr, (u32)m->param2);
		dev_info(a->dev, "firmware log buffer %d at %#x\n", m->param1, (u32)m->param2);
		break;
	case ABOX_REPORT_FAULT:
		atomic_inc(&a->faults_fw);
		dev_err(a->dev, "fault %d (%#x, %#x, %#x) reported from calliope (1 data abort, 2 prefetch abort, 3 os error, 4 vss error, 5 undefined)\n",
			m->param1, m->param1, m->param2, m->param3);
		break;
	default:
		dev_info_ratelimited(a->dev, "system message %#x (%d, %d, %d) not handled in A2\n",
				     m->msgtype, m->param1, m->param2, m->param3);
		break;
	}
}

/* 4.9 abox_gic_irq_handler() + abox_irq_handler(), system messages only */
static irqreturn_t a2_irq(int irq, void *data)
{
	struct abox_a2 *a = data;
	struct abox_a2_msg m;
	irqreturn_t ret = IRQ_NONE;
	unsigned int loops;
	u32 iar, nr;

	for (loops = 0; loops < 64; loops++) {
		iar = readl(a->gicc + GIC_CPU_INTACK);
		nr = iar & GICC_IAR_INT_ID_MASK;
		if (nr < 16) {
			writel(iar, a->gicc + GIC_CPU_EOI);
			writel(iar, a->gicc + GIC_CPU_DEACTIVATE);
			memcpy_fromio(&m, a->sram + a->rx, sizeof(m));
			writel(0, a->sram + a->rx_ack);
			atomic_inc(&a->ipc_rx);
			if (m.ipcid == IPC_SYSTEM)
				a2_system_msg(a, &m);
			else
				dev_info_ratelimited(a->dev, "IPC %u from calliope (SGI %u) not handled in A2\n",
						     m.ipcid, nr);
			ret = IRQ_HANDLED;
		} else if (nr < 1021) {
			writel(iar, a->gicc + GIC_CPU_EOI);
			atomic_inc(&a->spi_irqs);
			ret = IRQ_HANDLED;
		} else {
			break;	/* spurious: nothing pending */
		}
	}
	return ret;
}

/* 4.9 abox_gic_init_gic() (GIC_IS_SECURE_FREE) */
static void a2_gic_init(struct abox_a2 *a)
{
	int i;

	writel(0xff, a->gicc + GIC_CPU_PRIMASK);
	writel(0x3, a->gicd + GIC_DIST_CTRL);
	for (i = 0; i < 40; i++)
		writel(0x10101010, a->gicd + GIC_DIST_PRI + i * 4);
	writel(0x3, a->gicc + GIC_CPU_CTRL);
}

/* 4.9 __abox_process_ipc(), one try, no failsafe */
static int a2_ipc_send(struct abox_a2 *a, u32 msgtype)
{
	struct abox_a2_msg m = { .ipcid = IPC_SYSTEM, .msgtype = msgtype };
	unsigned long flags;
	int i, ret = 0;

	spin_lock_irqsave(&a->ipc_lock, flags);
	memcpy_toio(a->sram + a->tx, &m, sizeof(m));
	writel(1, a->sram + a->tx_ack);
	writel((1 << 16) | (IPC_SYSTEM & 0xf), a->gicd + GIC_DIST_SOFTINT);
	for (i = A2_IPC_TIMEOUT_US; i && readl(a->sram + a->tx_ack); i--)
		udelay(1);
	if (readl(a->sram + a->tx_ack)) {
		writel(0, a->sram + a->tx_ack);
		ret = -ETIMEDOUT;
	}
	spin_unlock_irqrestore(&a->ipc_lock, flags);
	return ret;
}

static void a2_pmu_snapshot(struct abox_a2 *a, const char *when)
{
	u32 st = 0, cfg = 0, cst = 0, opt = 0, sb = 0, pr = 0, gm = 0;

	regmap_read(gate.pmu, PMU_AUD_STATUS, &st);
	regmap_read(gate.pmu, PMU_ABOX_CPU_CONFIGURATION, &cfg);
	regmap_read(gate.pmu, PMU_ABOX_CPU_STATUS, &cst);
	regmap_read(gate.pmu, PMU_ABOX_CPU_OPTION, &opt);
	regmap_read(gate.pmu, PMU_ABOX_CPU_STANDBY, &sb);
	regmap_read(gate.pmu, PMU_PAD_RETENTION_ABOX_OPTION, &pr);
	regmap_read(gate.pmu, PMU_GPIO_MODE_ABOX_SYS_PWR, &gm);
	pr_info(A2 "pmu (%s): AUD_STATUS=%#x ABOX_CPU_CONFIGURATION=%#x ABOX_CPU_STATUS=%#x ABOX_CPU_OPTION=%#x ABOX_CPU_STANDBY=%#x PAD_RETENTION_ABOX_OPTION=%#x GPIO_MODE_ABOX_SYS_PWR=%#x\n",
		when, st, cfg, cst, opt, sb, pr, gm);
}

static const struct firmware *a2_fw(struct abox_a2 *a, const char *name, bool must)
{
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, name, a->dev);
	if (ret) {
		if (must)
			dev_err(a->dev, "%s request failed (%d)\n", name, ret);
		else
			dev_warn(a->dev, "%s request failed (%d): not loaded\n", name, ret);
		return NULL;
	}
	dev_info(a->dev, "%s is loaded (%zu bytes)\n", name, fw->size);
	return fw;
}

static int a2_request_firmware(struct abox_a2 *a)
{
	int i;

	a->fw_sram = a2_fw(a, "calliope_sram.bin", true);
	if (!a->fw_sram)
		return -ENOENT;
	if (a->fw_sram->size > a->tx) {
		dev_err(a->dev, "calliope_sram.bin (%zu) overlaps the IPC area at %#x\n",
			a->fw_sram->size, a->tx);
		return -EFBIG;
	}
	dev_info(a->dev, "SRAM firmware loaded\n");
	a->fw_dram = a2_fw(a, "calliope_dram.bin", true);
	if (!a->fw_dram)
		return -ENOENT;
	if (a->fw_dram->size > DRAM_FIRMWARE_SIZE)
		return -EFBIG;
	a->fw_iva = a2_fw(a, "calliope_iva.bin", true);
	if (!a->fw_iva)
		return -ENOENT;
	if (a->fw_iva->size > IVA_FIRMWARE_SIZE)
		return -EFBIG;
	for (i = 0; i < a->ext_num; i++) {
		a->fw_ext[i] = a2_fw(a, a->ext_name[i], false);
		if (a->fw_ext[i] && a->ext_off[i] + a->fw_ext[i]->size > DRAM_FIRMWARE_SIZE) {
			dev_err(a->dev, "%s does not fit at DRAM offset %#x: not loaded\n",
				a->ext_name[i], a->ext_off[i]);
			release_firmware(a->fw_ext[i]);
			a->fw_ext[i] = NULL;
		}
	}
	return 0;
}

static void a2_release_firmware(struct abox_a2 *a)
{
	int i;

	release_firmware(a->fw_sram);
	release_firmware(a->fw_dram);
	release_firmware(a->fw_iva);
	for (i = 0; i < a->ext_num; i++)
		release_firmware(a->fw_ext[i]);
}

/* 4.9 abox_download_firmware() + abox_download_extra_firmware() (area 1) */
static int a2_download(struct abox_a2 *a)
{
	int i, n = 0;

	memcpy_toio(a->sram, a->fw_sram->data, a->fw_sram->size);
	memset_io(a->sram + a->fw_sram->size, 0, a->sram_size - a->fw_sram->size);
	memcpy(a->dram, a->fw_dram->data, a->fw_dram->size);
	memset(a->dram + a->fw_dram->size, 0, DRAM_FIRMWARE_SIZE - a->fw_dram->size);
	memcpy(a->iva, a->fw_iva->data, a->fw_iva->size);
	memset(a->iva + a->fw_iva->size, 0, IVA_FIRMWARE_SIZE - a->fw_iva->size);
	for (i = 0; i < a->ext_num; i++) {
		if (!a->fw_ext[i])
			continue;
		memcpy(a->dram + a->ext_off[i], a->fw_ext[i]->data, a->fw_ext[i]->size);
		n++;
	}
	wmb();	/* the NC buffers and SRAM are written before the CPU starts */
	return n;
}

/* firmware log ring (struct ABOX_LOG_BUFFER), DRAM only, printable tail */
static void a2_log_tail(struct abox_a2 *a)
{
	u32 addr = READ_ONCE(a->log_addr), off, wr, size, start, len, i, j;
	char *buf, *p;

	if (!addr)
		return;
	if (addr < IOVA_DRAM_FIRMWARE || addr - IOVA_DRAM_FIRMWARE > DRAM_FIRMWARE_SIZE - 12) {
		dev_info(a->dev, "firmware log at %#x is outside the DRAM image: not read\n", addr);
		return;
	}
	off = addr - IOVA_DRAM_FIRMWARE;
	wr = readl(a->dram + off);	/* index_writer (NC memory) */
	size = readl(a->dram + off + 8);
	if (!size || size > DRAM_FIRMWARE_SIZE - off - 12 || wr > size) {
		dev_info(a->dev, "firmware log header implausible (writer %u size %u)\n", wr, size);
		return;
	}
	start = wr > A2_LOG_TAIL ? wr - A2_LOG_TAIL : 0;
	len = wr - start;
	buf = kzalloc(len + 1, GFP_KERNEL);
	if (!buf || !len) {
		kfree(buf);
		return;
	}
	memcpy(buf, a->dram + off + 12 + start, len);
	for (i = 0; i < len; i++)
		if (buf[i] != '\n' && (buf[i] < 0x20 || buf[i] > 0x7e))
			buf[i] = '.';
	dev_info(a->dev, "firmware log tail (writer %u of %u):\n", wr, size);
	for (p = buf, j = 0; p && *p && j < 24; j++) {
		char *nl = strchr(p, '\n');

		if (nl)
			*nl = 0;
		if (*p)
			dev_info(a->dev, "  calliope: %s\n", p);
		p = nl ? nl + 1 : NULL;
	}
	kfree(buf);
}

enum a2_stop_result {
	A2_STOP_CLEAN,		/* SUSPEND acked, WFI seen, CPU off confirmed */
	A2_STOP_OFF_DIRTY,	/* CPU off confirmed, but no SUSPEND ack or no WFI */
	A2_STOP_UNCONFIRMED,	/* CPU off not confirmed: the DSP may still run */
};

/*
 * Stop the DSP: 4.9 abox_cpu_pm_ipc(SUSPEND) + WFI poll, then the CPU
 * enable/power clear of the 4.9 8895 path, then status off. The SUSPEND
 * ack, the WFI poll and the CPU-off poll are three separate results; only
 * all three together are a clean stop. The DRAM request (SYSPOWER_CTRL) is
 * withdrawn only once the CPU reads off. If it does not, nothing the DSP
 * may still use is given back (DRAM request, clocks, buffers, SysMMU
 * translation) and the caller ends the boot via a2_end_left_running().
 */
static enum a2_stop_result a2_stop(struct abox_a2 *a, int n)
{
	u32 sb = 0, st = 0;
	int ipc, wfi, off;

	ipc = a2_ipc_send(a, ABOX_SUSPEND);
	wfi = regmap_read_poll_timeout(gate.pmu, PMU_ABOX_CPU_STANDBY, sb,
				       sb & ABOX_CPU_STANDBY_WFI, 100, A2_WFI_TIMEOUT_US);
	pr_info(A2 "boot %d: ABOX_SUSPEND ipc %s, WFI %s (STANDBY=%#x)\n", n,
		ipc ? "NOT acked" : "acked", wfi ? "NOT seen" : "seen", sb);
	if (wfi)
		a2_dump_gpr(a, "no WFI after ABOX_SUSPEND");

	regmap_update_bits(gate.pmu, PMU_ABOX_CPU_OPTION, ABOX_CPU_OPTION_ENABLE_CPU, 0);
	regmap_update_bits(gate.pmu, PMU_ABOX_CPU_CONFIGURATION, ABOX_CPU_LOCAL_PWR_CFG, 0);
	off = regmap_read_poll_timeout(gate.pmu, PMU_ABOX_CPU_STATUS, st,
				       !(st & ABOX_CPU_STATUS_ON), 100, A2_CPU_OFF_TIMEOUT_US);
	if (off) {
		pr_err(A2 "boot %d: ABOX CPU still on after enable/power off (STATUS=%#x): DRAM request (SYSPOWER_CTRL) kept\n",
		       n, st);
		return A2_STOP_UNCONFIRMED;
	}
	writel(0, a->sfr + ABOX_SYSPOWER_CTRL);	/* 4.9 abox_request_dram_on(false) */
	WRITE_ONCE(a->dsp_running, false);
	pr_info(A2 "boot %d: ABOX CPU off (STATUS=%#x)%s\n", n, st,
		ipc || wfi ? ", stop not clean (no SUSPEND ack or no WFI)" : "");
	return ipc || wfi ? A2_STOP_OFF_DIRTY : A2_STOP_CLEAN;
}

/*
 * The ABOX CPU did not read off after a stop. The DSP may still execute
 * and DMA, so this boot keeps everything it may use: the DRAM request
 * (SYSPOWER_CTRL = 1), the AUD clocks (held by the gate), the firmware
 * buffers and the chip-ID snapshot, and the runtime-PM reference that
 * keeps the SysMMU translating with A2's domain; a2_abox_suspend() refuses
 * system suspend. The boot then ends through the CL0 watchdog: petting
 * stops for good, so the SoC resets within the watchdog timeout (the
 * counter is reloaded once, so the logs reach pstore). Only if the guarded
 * watchdog cannot do that (gone or not running), an orderly reboot.
 */
static void a2_end_left_running(struct abox_a2 *a)
{
	int ret;

	pr_emerg(A2 "ABOX CPU could not be stopped: DRAM request, AUD clocks, buffers and SysMMU translation kept, system suspend refused; ending this boot via the watchdog\n");
	ret = s3c2410wdt_crownlte_stop_petting("audio A2: ABOX CPU not stopped");
	if (ret >= 0)
		return;
	pr_emerg(A2 "CL0 watchdog final stop not possible (%d): orderly reboot\n", ret);
	orderly_reboot();
}

/* one firmware boot, 4.9 abox_enable() order; 0 = booted, answered, stopped */
static int a2_boot_once(struct abox_a2 *a, int n, unsigned int *boot_ms)
{
	ktime_t t0;
	long left;
	u32 v;
	int ret, ext;

	reinit_completion(&a->boot_done);
	WRITE_ONCE(a->version, 0);

	if (!a->irq_enabled) {
		enable_irq(a->irq);
		a->irq_enabled = true;
	}
	a2_gic_init(a);
	writel(0x1, a->sysreg + ABOX_SYSREG_MISC_CON);
	ext = a2_download(a);
	pr_info(A2 "boot %d: SRAM %zu + DRAM %zu + IVA %zu bytes copied, ext firmware %d/%d\n",
		n, a->fw_sram->size, a->fw_dram->size, a->fw_iva->size, ext, a->ext_num);
	if (READ_ONCE(a->aborting))
		return -EINTR;

	writel(1, a->sfr + ABOX_SYSPOWER_CTRL);	/* 4.9 abox_request_dram_on(true) */
	t0 = ktime_get();
	WRITE_ONCE(a->dsp_running, true);
	regmap_update_bits(gate.pmu, PMU_ABOX_CPU_CONFIGURATION, ABOX_CPU_LOCAL_PWR_CFG,
			   ABOX_CPU_LOCAL_PWR_CFG);
	regmap_update_bits(gate.pmu, PMU_ABOX_CPU_OPTION, ABOX_CPU_OPTION_ENABLE_CPU,
			   ABOX_CPU_OPTION_ENABLE_CPU);
	ret = regmap_read_poll_timeout(gate.pmu, PMU_ABOX_CPU_STATUS, v,
				       v & ABOX_CPU_STATUS_ON, 10, A2_CPU_ON_TIMEOUT_US);
	if (ret) {
		pr_err(A2 "boot %d: abox cpu enable timeout (STATUS=%#x)\n", n, v);
		a2_stop(a, n);
		return -ETIME;
	}
	pr_info(A2 "boot %d: abox_cpu_enable(1): STATUS=%#x after %lld us\n", n, v,
		ktime_us_delta(ktime_get(), t0));

	left = wait_for_completion_timeout(&a->boot_done, msecs_to_jiffies(A2_BOOT_TIMEOUT_MS));
	*boot_ms = ktime_ms_delta(ktime_get(), t0);
	if (!left) {
		pr_err(A2 "boot %d: no \"ready to sing\" within %u ms (ipc rx %d, gic spi %d)\n",
		       n, A2_BOOT_TIMEOUT_MS, atomic_read(&a->ipc_rx), atomic_read(&a->spi_irqs));
		a2_dump_gpr(a, "no boot done");
		a2_log_tail(a);
		a2_stop(a, n);
		return -ETIMEDOUT;
	}
	pr_info(A2 "boot %d: ready to sing after %u ms, SFR VERSION=%#x\n", n, *boot_ms,
		readl(a->sfr + ABOX_VERSION));

	/* 4.9 abox_boot_done_work_func(): ABOX_RESUME (no restore of mixer data) */
	ret = a2_ipc_send(a, ABOX_RESUME);
	pr_info(A2 "boot %d: ABOX_RESUME ipc %s\n", n, ret ? "NOT acked" : "acked");
	msleep(a->run_ms);
	a2_log_tail(a);

	if (a2_stop(a, n) != A2_STOP_CLEAN)
		return -EIO;
	return ret ? -EPROTO : 0;
}

static void a2_finish(struct abox_a2 *a, bool pass, const char *fails, int boots,
		      unsigned int boot_ms_max)
{
	char v[4];
	u32 ver = READ_ONCE(a->version);

	memcpy(v, &ver, sizeof(v));
	snprintf(a->result, sizeof(a->result),
		 "%s boots=%d version=%c%c%c%c boot_ms_max=%u faults=%d fw_faults=%d ipc_rx=%d dsp=%s%s%s",
		 pass ? "PASS" : "FAIL", boots,
		 ver ? v[3] : '-', ver ? v[2] : '-', ver ? v[1] : '-', ver ? v[0] : '-',
		 boot_ms_max, atomic_read(&a->faults_mmu), atomic_read(&a->faults_fw),
		 atomic_read(&a->ipc_rx), READ_ONCE(a->dsp_running) ? "LEFT-RUNNING" : "off",
		 fails ? " fails: " : "", fails ? fails : "");
	if (pass)
		pr_info(A2 "%s\n", a->result);
	else
		pr_err(A2 "%s\n", a->result);
}

static void a2_work_fn(struct work_struct *w)
{
	struct abox_a2 *a = container_of(to_delayed_work(w), struct abox_a2, work);
	unsigned int boot_ms = 0, boot_ms_max = 0;
	const char *why, *fails = NULL;
	u32 st, cst, opt, cfg, sbsp;
	bool left_running = false;
	int i, ret, boots = 0;

	migrate_disable();
	pr_info(A2 "start uptime=%llu ms on CPU%d (pinned), boots wanted %u, run %u ms each\n",
		div_u64(ktime_get_boottime_ns(), NSEC_PER_MSEC), smp_processor_id(),
		a->boots_wanted, a->run_ms);
	if (smp_processor_id() == 0) {
		a2_finish(a, false, "cpu0 (nothing touched)", 0, 0);
		goto out;
	}
	why = a2_wdt_live(A2_TEST_MARGIN_MS / 2);
	if (!why)
		why = a2_wdt_verify("self-test");
	if (why) {
		pr_err(A2 "%s: nothing touched\n", why);
		a2_finish(a, false, "wdt", 0, 0);
		goto out;
	}

	/* PMU first (always on), then the first reads inside pd-aud */
	a2_pmu_snapshot(a, "before");
	if (regmap_read(gate.pmu, PMU_AUD_STATUS, &st) ||
	    regmap_read(gate.pmu, PMU_ABOX_CPU_CONFIGURATION, &cfg) ||
	    regmap_read(gate.pmu, PMU_ABOX_CPU_STATUS, &cst) ||
	    regmap_read(gate.pmu, PMU_ABOX_CPU_OPTION, &opt)) {
		a2_finish(a, false, "pmu-read", 0, 0);
		goto out;
	}
	if ((st & PMU_AUD_STATUS_ON) != PMU_AUD_STATUS_ON) {
		a2_finish(a, false, "pd-aud-off (nothing inside pd-aud touched)", 0, 0);
		goto out;
	}
	if ((cst & ABOX_CPU_STATUS_ON) || (opt & ABOX_CPU_OPTION_ENABLE_CPU)) {
		a2_finish(a, false, "abox-cpu-already-on (nothing written)", 0, 0);
		goto out;
	}
	a->pmu_cfg0 = cfg;
	a->pmu_opt0 = opt;
	sbsp = readl(a->sfr + ABOX_SYSPOWER_STATUS);
	pr_info(A2 "abox: SFR VERSION=%#x SYSPOWER_CTRL=%#x SYSPOWER_STATUS=%#x SYSREG MISC_CON=%#x CPU PC=%#x\n",
		readl(a->sfr + ABOX_VERSION), readl(a->sfr + ABOX_SYSPOWER_CTRL), sbsp,
		readl(a->sysreg + ABOX_SYSREG_MISC_CON), readl(a->sfr + ABOX_CPU_PC));

	/* firmware before the first write (user-helper fallback: ueventd) */
	ret = a2_request_firmware(a);
	if (ret) {
		a2_finish(a, false, "firmware (nothing written)", 0, 0);
		goto out_fw;
	}

	/* SysMMU on (supplier of this device, enabled with our domain) */
	ret = pm_runtime_resume_and_get(a->dev);
	if (ret) {
		dev_err(a->dev, "runtime resume (SysMMU) failed: %d\n", ret);
		a2_finish(a, false, "sysmmu-resume (nothing written)", 0, 0);
		goto out_fw;
	}
	a->rpm_held = true;

	for (i = 1; i <= a->boots_wanted; i++) {
		if (READ_ONCE(a->aborting)) {
			fails = "aborted (shutdown)";
			break;
		}
		why = a2_wdt_live(A2_GUARD_MARGIN_MS);
		if (why) {
			pr_err(A2 "boot %d not started: %s\n", i, why);
			fails = "wdt";
			break;
		}
		ret = a2_boot_once(a, i, &boot_ms);
		boot_ms_max = max(boot_ms_max, boot_ms);
		if (ret) {
			fails = ret == -ETIME ? "cpu-enable-timeout" :
				ret == -ETIMEDOUT ? "no-ready-to-sing" :
				ret == -EIO ? (READ_ONCE(a->dsp_running) ?
					       "stop-unconfirmed" : "stop") :
				ret == -EPROTO ? "resume-ipc" : "aborted";
			break;
		}
		boots++;
	}

	if (a->irq_enabled) {
		disable_irq(a->irq);
		a->irq_enabled = false;
	}
	left_running = READ_ONCE(a->dsp_running);
	if (!left_running) {
		/* leave the PMU as found (the boot state is CPU off) */
		regmap_update_bits(gate.pmu, PMU_ABOX_CPU_CONFIGURATION, ABOX_CPU_LOCAL_PWR_CFG,
				   a->pmu_cfg0 & ABOX_CPU_LOCAL_PWR_CFG);
		pm_runtime_put_sync(a->dev);
		a->rpm_held = false;
	} else if (!fails) {
		fails = "stop-unconfirmed";	/* cannot happen: every unconfirmed stop fails */
	}
	a2_pmu_snapshot(a, "after");

	if (!fails && atomic_read(&a->faults_mmu))
		fails = "sysmmu-faults";
	if (!fails && atomic_read(&a->faults_fw))
		fails = "firmware-fault";
	/* AUD runs at its boot DVFS level (4.9 raised the gear first): note only */
	if (boot_ms_max > A2_BOOT_PASS_MS)
		pr_warn(A2 "slowest boot %u ms is over the %u ms design target (AUD not raised)\n",
			boot_ms_max, A2_BOOT_PASS_MS);
	if (!fails && READ_ONCE(a->incompatible))
		fails = "incompatible";
	a2_finish(a, !fails && !left_running && boots == a->boots_wanted, fails, boots,
		  boot_ms_max);
out_fw:
	a2_release_firmware(a);	/* CPU-side copies; the DSP never sees them */
	if (left_running)
		a2_end_left_running(a);
out:
	migrate_enable();
}

/* -- ABOX probe: map, allocate, IOMMU domain; no register access -- */

static void a2_free_buffers(void *data)
{
	struct abox_a2 *a = data;
	unsigned int i;

	if (a->dram)
		vunmap(a->dram);
	if (a->iva)
		vunmap(a->iva);
	for (i = 0; i < a->dram_nchunks; i++)
		if (a->dram_chunks[i].page)
			__free_pages(a->dram_chunks[i].page, a->dram_chunks[i].order);
	if (a->iva_chunk.page)
		__free_pages(a->iva_chunk.page, a->iva_chunk.order);
	if (a->chipid_page)
		__free_page(a->chipid_page);
}

/*
 * The firmware expects the chip-ID block at IOVA 0x10000000 (4.9 mapped
 * the real register page there, identity). The 9810 SysMMU page tables
 * carry no protection bits (SYSMMU_NO_PROT), so a read-only mapping would
 * still let the DSP write. A2 therefore maps a private RAM page instead:
 * the CPU reads the chip-ID registers once (the window described in the
 * DT, outside pd-aud, already read by exynos-chipid at boot) and copies
 * them to the same offsets; the rest of the page is zero. Whatever the DSP
 * writes there only lands in this page.
 */
static int a2_chipid_snapshot(struct abox_a2 *a)
{
	struct device_node *np;
	void __iomem *base;
	struct resource r;
	u32 *snap;
	unsigned int i;
	int ret;

	np = of_find_compatible_node(NULL, NULL, "samsung,exynos9810-chipid");
	if (!np)
		return -ENODEV;
	ret = of_address_to_resource(np, 0, &r);
	of_node_put(np);
	if (ret)
		return ret;
	if (r.start != IOVA_CHIPID || resource_size(&r) < CHIPID_WINDOW)
		return -EINVAL;

	a->chipid_page = alloc_page(GFP_KERNEL | __GFP_ZERO);
	if (!a->chipid_page)
		return -ENOMEM;
	base = ioremap(r.start, CHIPID_WINDOW);
	if (!base)
		return -ENOMEM;
	snap = page_address(a->chipid_page);
	for (i = 0; i < ARRAY_SIZE(a2_chipid_regs); i++)
		snap[a2_chipid_regs[i] / 4] = readl(base + a2_chipid_regs[i]);
	iounmap(base);
	/* the SysMMU walk and the DSP are not coherent with the CPU caches */
	arch_dma_prep_coherent(a->chipid_page, PAGE_SIZE);
	dev_info(a->dev, "chip-id snapshot: PRODUCT_ID=%#x REV=%#x (+ unique ID words) copied to a private page\n",
		 snap[CHIPID_PRODUCT_ID / 4], snap[CHIPID_REV / 4]);
	return 0;
}

/* NC kernel view of @n chunks; the cacheable linear alias is cleaned first */
static void *a2_vmap_chunks(struct a2_chunk *c, unsigned int n, size_t total)
{
	unsigned int i, j, k = 0, npages = total >> PAGE_SHIFT;
	struct page **pages;
	void *va;

	pages = kvcalloc(npages, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return NULL;
	for (i = 0; i < n; i++) {
		arch_dma_prep_coherent(c[i].page, PAGE_SIZE << c[i].order);
		for (j = 0; j < (1U << c[i].order) && k < npages; j++)
			pages[k++] = c[i].page + j;
	}
	va = vmap(pages, npages, VM_MAP, pgprot_writecombine(PAGE_KERNEL));
	kvfree(pages);
	return va;
}

static int a2_alloc_buffers(struct abox_a2 *a)
{
	unsigned int order = get_order(SZ_1M);
	size_t done = 0;
	int ret;

	a->dram_chunks = devm_kcalloc(a->dev, DRAM_FIRMWARE_SIZE / SZ_64K,
				      sizeof(*a->dram_chunks), GFP_KERNEL);
	if (!a->dram_chunks)
		return -ENOMEM;
	ret = devm_add_action(a->dev, a2_free_buffers, a);
	if (ret)
		return ret;
	/* 1 MiB chunks (SysMMU sections), falling back to 64 KiB */
	while (done < DRAM_FIRMWARE_SIZE) {
		struct page *p = alloc_pages(GFP_KERNEL | __GFP_ZERO | __GFP_NOWARN, order);

		if (!p) {
			if (order == get_order(SZ_64K))
				return -ENOMEM;
			order = get_order(SZ_64K);
			continue;
		}
		a->dram_chunks[a->dram_nchunks].page = p;
		a->dram_chunks[a->dram_nchunks++].order = order;
		done += PAGE_SIZE << order;
	}
	a->iva_chunk.order = get_order(IVA_FIRMWARE_SIZE);
	a->iva_chunk.page = alloc_pages(GFP_KERNEL | __GFP_ZERO, a->iva_chunk.order);
	if (!a->iva_chunk.page)
		return -ENOMEM;

	a->dram = a2_vmap_chunks(a->dram_chunks, a->dram_nchunks, DRAM_FIRMWARE_SIZE);
	a->iva = a2_vmap_chunks(&a->iva_chunk, 1, IVA_FIRMWARE_SIZE);
	if (!a->dram || !a->iva)
		return -ENOMEM;
	ret = a2_chipid_snapshot(a);
	if (ret)
		return dev_err_probe(a->dev, ret, "chip-id snapshot\n");
	dev_info(a->dev, "dram firmware (%#x) alloc: %u chunks, iva firmware (%#x) alloc\n",
		 DRAM_FIRMWARE_SIZE, a->dram_nchunks, IVA_FIRMWARE_SIZE);
	return 0;
}

static void a2_iommu_release(void *data)
{
	struct abox_a2 *a = data;

	if (a->attached)
		iommu_detach_device(a->domain, a->dev);
	iommu_domain_free(a->domain);
}

static int a2_iommu_setup(struct abox_a2 *a)
{
	unsigned long iova = IOVA_DRAM_FIRMWARE;
	unsigned int i;
	int ret;

	a->domain = iommu_paging_domain_alloc(a->dev);
	if (IS_ERR(a->domain))
		return PTR_ERR(a->domain);
	ret = devm_add_action_or_reset(a->dev, a2_iommu_release, a);
	if (ret)
		return ret;
	iommu_set_fault_handler(a->domain, a2_iommu_fault, a);

	for (i = 0; i < a->dram_nchunks; i++) {
		size_t sz = PAGE_SIZE << a->dram_chunks[i].order;

		ret = iommu_map(a->domain, iova, page_to_phys(a->dram_chunks[i].page), sz,
				IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
		if (ret)
			return ret;
		iova += sz;
	}
	ret = iommu_map(a->domain, IOVA_IVA_FIRMWARE, page_to_phys(a->iva_chunk.page),
			IVA_FIRMWARE_SIZE, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret)
		return ret;
	/*
	 * 4.9 mapped the real chip-ID page here; A2 maps its RAM snapshot
	 * (a2_chipid_snapshot()). IOMMU_READ is not enforced by this SysMMU.
	 */
	ret = iommu_map(a->domain, IOVA_CHIPID, page_to_phys(a->chipid_page), PAGE_SIZE,
			IOMMU_READ, GFP_KERNEL);
	if (ret)
		return ret;

	ret = iommu_attach_device(a->domain, a->dev);
	if (ret)
		return ret;
	a->attached = true;
	dev_info(a->dev, "own IOMMU domain: DRAM firmware at %#lx (%u chunks), IVA at %#lx, chip-id snapshot (RAM) at %#lx; VSS/dump not mapped\n",
		 IOVA_DRAM_FIRMWARE, a->dram_nchunks, IOVA_IVA_FIRMWARE, IOVA_CHIPID);
	return 0;
}

static ssize_t calliope_version_show(struct device *dev, struct device_attribute *attr,
				     char *buf)
{
	struct abox_a2 *a = dev_get_drvdata(dev);
	u32 v = READ_ONCE(a->version);
	char c[4];

	memcpy(c, &v, sizeof(c));
	return sysfs_emit(buf, "%c%c%c%c\n", c[3], c[2], c[1], c[0]);
}
static DEVICE_ATTR_RO(calliope_version);

static ssize_t a2_result_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct abox_a2 *a = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", a->result[0] ? a->result : "pending");
}
static DEVICE_ATTR_RO(a2_result);

static struct attribute *a2_attrs[] = {
	&dev_attr_calliope_version.attr,
	&dev_attr_a2_result.attr,
	NULL
};
ATTRIBUTE_GROUPS(a2);

static int a2_abox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *res;
	struct abox_a2 *a;
	s64 now, delay;
	int ret, i;

	ret = exynos9810_abox_a2_guard(dev);
	if (ret)
		return ret;

	a = devm_kzalloc(dev, sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;
	a->dev = dev;
	platform_set_drvdata(pdev, a);
	init_completion(&a->boot_done);
	spin_lock_init(&a->ipc_lock);
	INIT_DELAYED_WORK(&a->work, a2_work_fn);

	a->sfr = devm_platform_ioremap_resource_byname(pdev, "sfr");
	a->sysreg = devm_platform_ioremap_resource_byname(pdev, "sysreg");
	a->gicd = devm_platform_ioremap_resource_byname(pdev, "gicd");
	a->gicc = devm_platform_ioremap_resource_byname(pdev, "gicc");
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	a->sram = devm_ioremap_resource(dev, res);
	if (IS_ERR(a->sfr) || IS_ERR(a->sysreg) || IS_ERR(a->gicd) || IS_ERR(a->gicc) ||
	    IS_ERR(a->sram))
		return dev_err_probe(dev, -EINVAL, "reg windows\n");
	a->sram_size = resource_size(res);
	if (resource_size(platform_get_resource_byname(pdev, IORESOURCE_MEM, "gicc")) <
	    GIC_CPU_DEACTIVATE + 4)
		return dev_err_probe(dev, -EINVAL, "gicc window too small\n");

	if (of_property_read_u32(np, "samsung,ipc-tx-offset", &a->tx) ||
	    of_property_read_u32(np, "samsung,ipc-rx-offset", &a->rx) ||
	    of_property_read_u32(np, "samsung,ipc-tx-ack-offset", &a->tx_ack) ||
	    of_property_read_u32(np, "samsung,ipc-rx-ack-offset", &a->rx_ack))
		return dev_err_probe(dev, -EINVAL, "IPC offsets\n");
	if (a->tx + sizeof(struct abox_a2_msg) > a->tx_ack ||
	    a->rx + sizeof(struct abox_a2_msg) > a->rx_ack ||
	    a->tx_ack + 4 > a->sram_size || a->rx_ack + 4 > a->sram_size)
		return dev_err_probe(dev, -EINVAL, "IPC offsets do not fit the SRAM\n");

	if (of_property_read_u32(np, "samsung,abox-a2-boots", &a->boots_wanted) ||
	    a->boots_wanted < 1 || a->boots_wanted > 2)
		a->boots_wanted = 2;
	if (of_property_read_u32(np, "samsung,abox-a2-run-ms", &a->run_ms) ||
	    a->run_ms < 100 || a->run_ms > 5000)
		a->run_ms = 1000;

	a->ext_num = of_property_read_string_array(np, "samsung,ext-firmware-names",
						   a->ext_name, A2_MAX_EXT_FW);
	if (a->ext_num < 0)
		a->ext_num = 0;
	if (a->ext_num &&
	    of_property_read_u32_array(np, "samsung,ext-firmware-offsets", a->ext_off,
				       a->ext_num))
		return dev_err_probe(dev, -EINVAL, "ext firmware offsets\n");
	for (i = 0; i < a->ext_num; i++)
		if (a->ext_off[i] >= DRAM_FIRMWARE_SIZE)
			return dev_err_probe(dev, -EINVAL, "ext firmware offset %#x\n",
					     a->ext_off[i]);

	a->irq = platform_get_irq(pdev, 0);
	if (a->irq < 0)
		return a->irq;
	ret = devm_request_irq(dev, a->irq, a2_irq, IRQF_NO_AUTOEN, dev_name(dev), a);
	if (ret)
		return ret;
	if (irq_set_affinity(a->irq, cpumask_of(gate.cpu)))
		dev_warn(dev, "ABOX GIC irq not moved to CPU%u\n", gate.cpu);

	ret = a2_alloc_buffers(a);
	if (ret)
		return dev_err_probe(dev, ret, "firmware buffers\n");
	ret = a2_iommu_setup(a);
	if (ret)
		return dev_err_probe(dev, ret, "IOMMU domain\n");
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	now = div_u64(ktime_get_boottime_ns(), NSEC_PER_MSEC);
	delay = gate.start_ms > now ? gate.start_ms - now : 0;
	queue_delayed_work_on(gate.cpu, system_long_wq, &a->work, msecs_to_jiffies(delay));
	dev_info(dev, "audio A2 ABOX boot test scheduled at ~%u ms on CPU%u (%u boots, %d ext firmware files); nothing inside pd-aud touched yet\n",
		 gate.start_ms, gate.cpu, a->boots_wanted, a->ext_num);
	return 0;
}

static int a2_abox_suspend(struct device *dev)
{
	struct abox_a2 *a = dev_get_drvdata(dev);

	/* a running DSP behind a SysMMU in bypass would DMA to physical memory */
	if (READ_ONCE(a->dsp_running) || a->rpm_held || delayed_work_pending(&a->work) ||
	    work_busy(&a->work.work)) {
		dev_warn(dev, "audio A2: ABOX test pending or running: system suspend refused\n");
		return -EBUSY;
	}
	return 0;
}

static void a2_abox_shutdown(struct platform_device *pdev)
{
	struct abox_a2 *a = platform_get_drvdata(pdev);

	if (!a)
		return;
	WRITE_ONCE(a->aborting, true);
	cancel_delayed_work_sync(&a->work);
	if (READ_ONCE(a->dsp_running)) {
		regmap_update_bits(gate.pmu, PMU_ABOX_CPU_OPTION, ABOX_CPU_OPTION_ENABLE_CPU, 0);
		dev_info(&pdev->dev, "audio A2: ABOX CPU disabled at shutdown\n");
	}
}

static DEFINE_SIMPLE_DEV_PM_OPS(a2_abox_pm, a2_abox_suspend, NULL);

static const struct of_device_id a2_gate_match[] = {
	{ .compatible = "samsung,exynos9810-abox-a2" },
	{ }
};

static struct platform_driver a2_gate_driver = {
	.driver = {
		.name = "exynos9810-abox-a2",
		.of_match_table = a2_gate_match,
		.suppress_bind_attrs = true,
	},
	.probe = a2_gate_probe,
};

static const struct of_device_id a2_abox_match[] = {
	{ .compatible = "samsung,exynos9810-abox-boot" },
	{ }
};

static struct platform_driver a2_abox_driver = {
	.driver = {
		.name = "exynos9810-abox-boot",
		.of_match_table = a2_abox_match,
		.suppress_bind_attrs = true,
		.dev_groups = a2_groups,
		.pm = pm_sleep_ptr(&a2_abox_pm),
	},
	.probe = a2_abox_probe,
	.shutdown = a2_abox_shutdown,
	.driver_managed_dma = true,	/* own unmanaged IOMMU domain */
};

static int __init a2_init(void)
{
	int ret = platform_driver_register(&a2_gate_driver);

	if (ret)
		return ret;
	return platform_driver_register(&a2_abox_driver);
}
device_initcall(a2_init);
