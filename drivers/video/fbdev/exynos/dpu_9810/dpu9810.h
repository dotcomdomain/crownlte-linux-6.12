/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Exynos9810 DPU (DECON-F + DPP) fbdev driver, internal definitions.
 *
 * Forward port of the Samsung 4.9 dpu_9810 driver (decon_core.c,
 * decon_reg.c, dpp_reg.c), reduced to what the vendor HWC needs from
 * /dev/graphics/fb0 while SBoot's DECON/DSIM/panel state is adopted.
 */
#ifndef __DPU9810_H__
#define __DPU9810_H__

#include <linux/dma-buf.h>
#include <linux/dma-fence.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <uapi/video/exynos9810_decon.h>

#include "regs-decon.h"
#include "regs-dpp.h"

#define DPU_PFX			"decon: "
#define DPU_IOMMU_PFX		"decon iommu: "
#define DPU_ST_PFX		"decon selftest: "
#define DPU_MX_PFX		"decon matrix: "
#define DPU_PB_PFX		"panel-bright: "

/*
 * Per-packet panel command logs go to the ring buffer (and /dev/kmsg, so
 * the serial capture still sees them) at KERN_DEBUG, below the default
 * console loglevel: legacy consoles write with interrupts off
 * (kernel/printk/printk.c console_emit_next_record()), and fbcon scrolling
 * the 1440x2960 fb0 takes tens of ms per line at early boot (trial q46).
 */
#define pb_dbg(fmt, ...)	printk(KERN_DEBUG DPU_PB_PFX fmt, ##__VA_ARGS__)

#define DPU_NR_DPP		6
#define DPU_NR_SYSMMU		3

/* PMU DPU power domain status (4.9 flexpmu_cal_local_exynos9810.h) */
#define DPU_PMU_DPU_STATUS	0x4024
#define DPU_PMU_DPU_ON		0xf

/* DPU SysMMU non-secure SFRs (4.9 exynos-iommu.h), read only here */
#define DPU_SYSMMU_CTRL		0x000
#define DPU_SYSMMU_CFG		0x004
#define DPU_SYSMMU_STATUS	0x008
#define DPU_SYSMMU_PT_BASE	0x00c
#define DPU_SYSMMU_VERSION	0x034
#define DPU_SYSMMU_INT_STATUS	0x060

/* DPP-level pixel formats (after dpu_translate_fmt_to_dpp()) */
#define DPU_FMT_IS_RGB32(f)	((f) <= DECON_PIXEL_FORMAT_BGRX_8888)

/* 4.9 decon.h enum decon_win_func */
#define PD_FUNC_COPY			0x1
#define PD_FUNC_USER_DEFINED		0xd

/* 4.9 decon.h enum decon_win_alpha_coef */
#define BND_COEF_ZERO			0x0
#define BND_COEF_ONE			0x1
#define BND_COEF_AF			0x2
#define BND_COEF_1_M_AF			0x3
#define BND_COEF_PLNAE_ALPHA0		0x6
#define BND_COEF_ALPHA_MULT		0xA
#define BND_COEF_1_M_ALPHA_MULT		0xB

/* 4.9 decon.h enum decon_win_alpha_sel */
#define ALPHA_MULT_SRC_SEL_AF		2

/* 4.9 enum decon_psr_mode */
#define DPU_PSR_MIPI_COMMAND	2

struct dpu_dev;

/*
 * A dma-buf mapped through the DPU SysMMU, kept for later frames
 * (decon_drv.c dpu_iova_get()). Only with the SysMMU (d->iommu).
 */
struct dpu_iova {
	struct list_head node;		/* d->iova_lru, most recent first */
	struct dpu_dev *d;
	struct dma_buf *dmabuf;		/* the cache's own reference */
	struct dma_buf_attachment *att;
	struct sg_table *sgt;
	dma_addr_t addr;		/* one IOVA run */
	size_t len;
	u32 nsegs;			/* physical chunks behind it */
	unsigned int users;		/* commits that hold it (d->iova_lock) */
};

struct dpu_buf {
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *att;
	struct sg_table *sgt;
	dma_addr_t addr;		/* 0 = no buffer; an IOVA with the SysMMU */
	size_t len;
	bool bounce;			/* scattered: copied into a decon-pool buffer */
	u32 nsegs;
	struct dpu_iova *iova;		/* cached SysMMU mapping (att/sgt unused) */
};

/* contiguous copy target for scattered buffers until the DPU SysMMU is in */
struct dpu_bounce {
	void *vaddr;
	dma_addr_t dma;
	size_t size;
};

/* One S3CFB_WIN_CONFIG request, as queued to the update worker */
struct dpu_commit {
	struct list_head node;
	struct decon_win_config cfg[DECON_MAX_WIN];	/* user format */
	u32 dpp_fmt[DECON_MAX_WIN];			/* translated */
	struct dpu_buf buf[DECON_MAX_WIN];
	struct dma_fence *acq[DECON_MAX_WIN];
	struct dma_fence *fence;	/* retire = release fence, may be NULL */
	unsigned int nr_win;
	bool internal;			/* kernel-owned buffers (fb, self-test) */
	unsigned long bounced;		/* window i scans d->bounce[i][slot[i]] */
	u8 slot[DECON_MAX_WIN];		/* bounce slot used by window i */
	ktime_t queued;
	u32 latch_us;			/* filled by the worker */
	u32 fc_before, fc_after;
	int result;
};

struct dpu_lcd {
	u32 xres, yres, fps, mm_w, mm_h;
	u32 dsc_en, dsc_w, dsc_h;
	u32 hdr_num, hdr_type[DECON_HDR_CAPA_NUM];
	u32 hdr_max_luma, hdr_max_avg_luma, hdr_min_luma;
};

struct dpu_dev {
	struct device *dev;
	void __iomem *regs;			/* DECON-F */
	void __iomem *dma_regs;			/* DPP IDMA G0..VGF1, 0x1000 apart */
	void __iomem *dpp_regs;			/* DPP blocks G0..VGF1, 0x1000 apart */
	void __iomem *dma_com;			/* DPU DMA common */
	void __iomem *sysmmu[DPU_NR_SYSMMU];	/* read-only views */
	struct regmap *pmu;
	struct dpu_lcd lcd;

	/*
	 * DPU SysMMUs through the IOMMU API, opt-in by "iommus" on decon_f.
	 * NULL: no translation, physically contiguous buffers below 4 GiB
	 * and bounce copies (the default).
	 */
	struct iommu_domain *iommu;
	bool rpm_held;				/* pm_runtime ref keeps SysMMUs on */
	spinlock_t trig_lock;			/* HW_SW_TRIG_CONTROL RMW */
	/*
	 * Set by a SysMMU fault (trig_lock): the TE trigger stays masked
	 * until a commit unmasks it (dpu_hw_trigger_commit()); command
	 * window and other restores keep it masked.
	 */
	bool trig_fault_inhibit;
	atomic_t iommu_faults;
	unsigned long last_fault_iova;
	u32 fault_trigger_stops, fault_inhibit_holds, fault_recoveries;
	/* a shadow update held by a fault mask (dpu_pending_resolve()), hw_lock */
	u32 update_seq;				/* shadow updates requested */
	u32 fault_try_seq;			/* update the tries count for */
	u8 fault_tries;
	bool fault_quiesced;			/* tries used up, trigger stays masked */
	u32 fault_quiesces;
	u32 trig_boot;				/* trigger before the fault handler */
	struct mutex iova_lock;			/* the IOVA cache */
	struct list_head iova_lru;
	u32 iova_nr, iova_maps, iova_hits, iova_unmaps;

	/* adopted SBoot state */
	int boot_win, boot_dpp;
	u32 boot_fmt;				/* IDMA_IMG_FORMAT_* SBoot left */
	u32 fb_dpp_fmt;				/* fb0 window, DPP level (= boot) */
	bool trig_written;
	u32 trig_orig;
	bool continuous;			/* TE trigger left unmasked */
	unsigned long dpp_inited;		/* bitmap of DPPs we initialised */

	/* fb0 */
	struct fb_info *fb;
	phys_addr_t fb_phys;
	size_t fb_size;
	u32 fb_stride;
	u32 pseudo_palette[16];
	bool fb_taken;				/* window reprogrammed to fb0 (4.9 style) */

	/* commit pipeline */
	struct mutex lock;			/* ioctl serialisation */
	struct mutex hw_lock;			/* register programming */
	spinlock_t queue_lock;
	struct list_head queue;
	struct kthread_worker *worker;
	struct kthread_work work;
	struct dpu_commit *active;		/* buffers currently scanned */
	struct list_head stale;			/* maybe still scanned (latch timeout) */
	u64 fence_ctx;
	atomic64_t fence_seqno;
	spinlock_t fence_lock;
	u32 nr_commits, nr_latch_timeouts, nr_rejects;
	u32 nr_base_mismatch;			/* active IDMA base != programmed */
	struct dpu_bounce bounce[DECON_MAX_WIN][2];	/* per window, double buffered */
	u8 bounce_next[DECON_MAX_WIN];
	u32 nr_bounces, bounce_max_us, nr_afbc;
	bool diag_latched;			/* first user commit logged */
	struct kthread_delayed_work idle_work;	/* diag of the frame HWC left up */
	u32 nr_idle_snaps;
	struct delayed_work fb_snap_work;	/* fb0 picture at 5 s */
	bool fb_snap_hwc;			/* fb0 logged before HWC's first commit */
	ktime_t thumb_at[DECON_MAX_WIN];	/* last sampled window thumbnail */
	/* diagnostic text, printed from system_wq (decon_drv.c dpu_diag_post) */
	spinlock_t diag_lock;
	struct list_head diag_txt;
	struct work_struct diag_emit_work;
	/* FRAME_COUNT/TE count between two callbacks (dpu_rate_start) */
	struct delayed_work rate_work;
	unsigned long rate_busy;
	u32 rate_fc0;
	int rate_te0;
	ktime_t rate_t0;
	char rate_tag[40];
	u32 hwc_ver;
	/* ioctl numbers seen, each logged once with its first result */
	spinlock_t ioc_lock;
	u32 ioc_seen[32];
	unsigned int ioc_nseen;

	/* vsync (TE EINT, as 4.9 decon_ext_irq_handler) */
	int te_irq;
	ktime_t vsync_ts;
	atomic_t te_count;
	wait_queue_head_t vsync_wait;
	bool vsync_active;
	struct task_struct *vsync_thread;

	/* self-test */
	struct delayed_work st_work;
	u32 st_at_ms, st_flips, st_period_ms;
	bool stopping;

	/* display matrix self-test (decon_drv.c dpu_mx_thread) */
	struct task_struct *mx_thread;
	u32 mx_at_ms;

	/* DSIM0 command path (dsim_cmd.c), only inside a command window */
	void __iomem *dsim;
	bool cmd_window;			/* hw_lock held, trigger masked, no frame */
	u32 dsim_ntx, dsim_nrx, dsim_nerr, dsim_nnote;
	u32 dsim_max_tx_us, dsim_max_rx_us;
	/* IRQ-latency sentinel: longest TE interrupt interval in a window */
	bool te_gap_track;
	u32 te_gap_max_us;

	/* fault-display brightness (panel_bright.c), opt-in via DT */
	struct delayed_work pb_work;
	u32 pb_target, pb_at_ms;
	bool pb_enabled, pb_done, pb_has_id, pb_has_gamma;
	const char *pb_readback;
	u8 pb_id[3];
	u8 pb_gamma[34];
	/* read-only panel status samples after the brightness (panel_bright.c) */
	struct delayed_work pb_status_work;
	u32 pb_nstatus;
	bool pb_status_key2;			/* DT: level 2 key around EE/EA */
	/* one 4.9 fix_green() ALPM enter/exit cycle (panel_bright.c), opt-in via DT */
	struct delayed_work pb_alpm_work;
	u32 pb_alpm_at_ms;			/* 0 = off */
	bool pb_alpm_active;			/* alpm packet shapes allowed */
	/* one 4.9 screen off / on panel cycle (panel_recovery.c), opt-in via DT */
	struct delayed_work pr_work;
	u32 pr_level, pr_at_ms;			/* level 0 = off */
	bool pr_only_elvdd;			/* DT: only when keyed EE shows ELVDD */
	bool pr_done;
	bool pr_active;				/* sleep/display DCS, DSC/PPS allowed */
	/* one panel command sequence at a time: bright, status, alpm, recovery */
	struct mutex panel_lock;
};

/* decon_hw.c: register layer (ported from 4.9 decon_reg.c / dpp_reg.c) */
static inline u32 decon_rd(struct dpu_dev *d, u32 off)
{
	return readl(d->regs + off);
}

static inline void decon_wr(struct dpu_dev *d, u32 off, u32 val)
{
	writel(val, d->regs + off);
}

static inline void decon_wr_mask(struct dpu_dev *d, u32 off, u32 val, u32 mask)
{
	u32 old = decon_rd(d, off);

	decon_wr(d, off, (val & mask) | (old & ~mask));
}

bool dpu_hw_powered(struct dpu_dev *d);
u32 dpu_hw_frame_count(struct dpu_dev *d);
int dpu_hw_channel_to_dpp(u32 ch);
u32 dpu_hw_dpp_to_channel(u32 dpp);
u32 dpu_hw_dma_rd(struct dpu_dev *d, int dpp, u32 off);
void dpu_hw_trigger(struct dpu_dev *d, bool enable);
void dpu_hw_trigger_restore(struct dpu_dev *d, u32 trig);
void dpu_hw_trigger_commit(struct dpu_dev *d);
bool dpu_hw_trigger_fault_unmask(struct dpu_dev *d);
bool dpu_hw_fault_inhibited(struct dpu_dev *d);
void dpu_hw_trigger_reset_fault(struct dpu_dev *d, u32 trig);
bool dpu_hw_trigger_stop_atomic(struct dpu_dev *d);
int dpu_hw_wait_shadow(struct dpu_dev *d, unsigned int timeout_us);
void dpu_hw_update_req(struct dpu_dev *d);
u32 dpu_hw_dma_active_rd(struct dpu_dev *d, int dpp, u32 off);
void dpu_hw_dpp_init(struct dpu_dev *d, int dpp);
int dpu_hw_dpp_config(struct dpu_dev *d, int dpp, const struct decon_win_config *c,
		      u32 dpp_fmt, dma_addr_t addr);
void dpu_hw_win_config(struct dpu_dev *d, int win, const struct decon_win_config *c,
		       bool enable);
u32 dpu_translate_fmt_to_dpp(u32 format);
int dpu_fmt_bpp(u32 dpp_fmt);

/* decon_drv.c: DECON-side guard around panel commands */
int dpu_cmd_window_begin(struct dpu_dev *d, u32 *trig);
void dpu_cmd_window_end(struct dpu_dev *d, u32 trig);
void dpu_cmd_window_pause(struct dpu_dev *d, u32 trig);	/* frames run, hw_lock kept */
int dpu_cmd_window_resume(struct dpu_dev *d);

/* dsim_cmd.c: DSIM0 packets (4.9 dsim_write_data / dsim_read_data) */
int dpu_dsim_check(struct dpu_dev *d);
void dpu_dsim_clear_status(struct dpu_dev *d);
int dpu_dsim_tx(struct dpu_dev *d, u8 type, const u8 *buf, size_t len);
int dpu_dsim_dcs_read(struct dpu_dev *d, u8 cmd, u8 *buf, size_t len);
int dpu_dsim_lp11_check(struct dpu_dev *d, const char *when);
int dpu_dsim_enter_lp11(struct dpu_dev *d, bool *hs_was_on);
int dpu_dsim_leave_lp11(struct dpu_dev *d, bool hs_was_on);

/* panel_bright.c: the 4.9 brightness-44700 sequence */
int dpu_panel_bright_init(struct dpu_dev *d);
void dpu_panel_bright_work(struct work_struct *work);
void dpu_panel_status_work(struct work_struct *work);
void dpu_panel_status_schedule(struct dpu_dev *d);
void dpu_panel_alpm_work(struct work_struct *work);
u32 dpu9810_panel_brightness(void);	/* also used by panel_ro.c */

/* panel_recovery.c: one 4.9 screen off / on cycle (levels 1 and 2) */
int dpu_panel_recovery_init(struct dpu_dev *d);
void dpu_panel_recovery_work(struct work_struct *work);

#endif /* __DPU9810_H__ */
