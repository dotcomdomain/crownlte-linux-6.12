// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos9810 DECON-F / DPP register layer.
 *
 * Ported from the Samsung 4.9 kernel (drivers/video/fbdev/exynos/dpu_9810/
 * decon_reg.c, dpp_reg.c, dpp_drv.c, helper.c): RGB, RGB10 and YUV windows,
 * scaling and AFBC on VGF0/VGF1, flips everywhere, 90/270 rotation and HDR
 * on VGF1, block areas. Register sequences and values are unchanged; the
 * per-id accessors now take the device.
 */
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/regmap.h>

#include "dpu9810.h"
#include "dpp_coef.h"
#include "dpp_hdr_lut.h"

#define DPP_STRIDE	0x1000

static u32 dma_rd(struct dpu_dev *d, int id, u32 off)
{
	return readl(d->dma_regs + id * DPP_STRIDE + off);
}

static void dma_wr(struct dpu_dev *d, int id, u32 off, u32 val)
{
	writel(val, d->dma_regs + id * DPP_STRIDE + off);
}

static void dma_wr_mask(struct dpu_dev *d, int id, u32 off, u32 val, u32 mask)
{
	u32 old = dma_rd(d, id, off);

	dma_wr(d, id, off, (val & mask) | (old & ~mask));
}

static void dpp_wr(struct dpu_dev *d, int id, u32 off, u32 val)
{
	writel(val, d->dpp_regs + id * DPP_STRIDE + off);
}

static void dpp_wr_mask(struct dpu_dev *d, int id, u32 off, u32 val, u32 mask)
{
	u32 old = readl(d->dpp_regs + id * DPP_STRIDE + off);

	dpp_wr(d, id, off, (val & mask) | (old & ~mask));
}

u32 dpu_hw_dma_rd(struct dpu_dev *d, int dpp, u32 off)
{
	return dma_rd(d, dpp, off);
}

bool dpu_hw_powered(struct dpu_dev *d)
{
	u32 st = 0;

	if (regmap_read(d->pmu, DPU_PMU_DPU_STATUS, &st))
		return false;
	return (st & DPU_PMU_DPU_ON) == DPU_PMU_DPU_ON;
}

u32 dpu_hw_frame_count(struct dpu_dev *d)
{
	return decon_rd(d, FRAME_COUNT);
}

/* 4.9 helper.c dpu_dma_type_to_channel() and its inverse */
static const u8 dpp_to_ch[DPU_NR_DPP] = { 5, 3, 0, 4, 1, 2 };

u32 dpu_hw_dpp_to_channel(u32 dpp)
{
	return dpp < DPU_NR_DPP ? dpp_to_ch[dpp] : 0;
}

int dpu_hw_channel_to_dpp(u32 ch)
{
	int i;

	for (i = 0; i < DPU_NR_DPP; i++)
		if (dpp_to_ch[i] == ch)
			return i;
	return -EINVAL;
}

/*
 * 4.9 decon_reg_set_trigger(), DECON_HW_TRIG:
 * enable  -> HW_TRIG_EN=1, HW_TRIG_MASK_DECON=0
 * disable -> HW_TRIG_EN=0, HW_TRIG_MASK_DECON=1
 */
void dpu_hw_trigger(struct dpu_dev *d, bool enable)
{
	dpu_hw_trigger_restore(d, enable ? HW_TRIG_EN : HW_TRIG_MASK_DECON);
}

/*
 * Write the HW_TRIG_EN / HW_TRIG_MASK_DECON bits of @trig. The read-modify-
 * write is under trig_lock because the DPU SysMMU fault handler (hard IRQ,
 * only with the SysMMU) masks the trigger too.
 */
void dpu_hw_trigger_restore(struct dpu_dev *d, u32 trig)
{
	unsigned long flags;

	spin_lock_irqsave(&d->trig_lock, flags);
	/*
	 * After a SysMMU fault only a commit may unmask (the faulting buffer
	 * may still be programmed): any other restore, e.g. the end of a
	 * panel command window that saved an enabled trigger, keeps it masked.
	 * Never set without the SysMMU.
	 */
	if (d->trig_fault_inhibit && (trig & HW_TRIG_EN)) {
		trig = (trig & ~HW_TRIG_EN) | HW_TRIG_MASK_DECON;
		d->fault_inhibit_holds++;
	}
	decon_wr_mask(d, HW_SW_TRIG_CONTROL, trig, HW_TRIG_EN | HW_TRIG_MASK_DECON);
	spin_unlock_irqrestore(&d->trig_lock, flags);
}

/* a commit's own unmask (dpu_apply()): it replaced the faulting buffers */
void dpu_hw_trigger_commit(struct dpu_dev *d)
{
	unsigned long flags;

	spin_lock_irqsave(&d->trig_lock, flags);
	d->trig_fault_inhibit = false;
	decon_wr_mask(d, HW_SW_TRIG_CONTROL, HW_TRIG_EN, HW_TRIG_EN | HW_TRIG_MASK_DECON);
	spin_unlock_irqrestore(&d->trig_lock, flags);
}

/*
 * Recovery for a shadow update that a fault left pending behind the masked
 * trigger: one controlled unmask so that it latches at the next TE. Only if
 * a fault inhibited the trigger; returns true if it unmasked.
 */
bool dpu_hw_trigger_fault_unmask(struct dpu_dev *d)
{
	unsigned long flags;
	bool done = false;

	spin_lock_irqsave(&d->trig_lock, flags);
	if (d->trig_fault_inhibit) {
		d->trig_fault_inhibit = false;
		decon_wr_mask(d, HW_SW_TRIG_CONTROL, HW_TRIG_EN,
			      HW_TRIG_EN | HW_TRIG_MASK_DECON);
		d->fault_recoveries++;
		done = true;
	}
	spin_unlock_irqrestore(&d->trig_lock, flags);
	return done;
}

bool dpu_hw_fault_inhibited(struct dpu_dev *d)
{
	unsigned long flags;
	bool v;

	spin_lock_irqsave(&d->trig_lock, flags);
	v = d->trig_fault_inhibit;
	spin_unlock_irqrestore(&d->trig_lock, flags);
	return v;
}

/*
 * Probe failure after the fault handler was removed and synchronised: no
 * commit will ever clear the inhibit, so drop it and write @trig back.
 */
void dpu_hw_trigger_reset_fault(struct dpu_dev *d, u32 trig)
{
	unsigned long flags;

	spin_lock_irqsave(&d->trig_lock, flags);
	d->trig_fault_inhibit = false;
	decon_wr_mask(d, HW_SW_TRIG_CONTROL, trig, HW_TRIG_EN | HW_TRIG_MASK_DECON);
	spin_unlock_irqrestore(&d->trig_lock, flags);
}

/*
 * From the DPU SysMMU fault IRQ: stop TE-triggered frames so that no new
 * frame fetches a faulting buffer; the frame in progress completes. Returns
 * true if the trigger was running.
 */
bool dpu_hw_trigger_stop_atomic(struct dpu_dev *d)
{
	bool was;
	u32 v;

	spin_lock(&d->trig_lock);
	/* latched even if masked now: a saved enabled value must not undo it */
	d->trig_fault_inhibit = true;
	v = decon_rd(d, HW_SW_TRIG_CONTROL);
	was = (v & (HW_TRIG_EN | HW_TRIG_MASK_DECON)) == HW_TRIG_EN;
	if (was)
		decon_wr(d, HW_SW_TRIG_CONTROL, (v & ~HW_TRIG_EN) | HW_TRIG_MASK_DECON);
	spin_unlock(&d->trig_lock);
	return was;
}

/* 4.9 decon_reg_wait_for_update_timeout(): 0 once the shadow update latched */
int dpu_hw_wait_shadow(struct dpu_dev *d, unsigned int timeout_us)
{
	u32 v;

	return readl_poll_timeout(d->regs + SHADOW_REG_UPDATE_REQ, v, !v,
				  250, timeout_us);
}

/*
 * 4.9 __decon_update_regs(): decon_reg_all_win_shadow_update_req() (per-window
 * bits [5:0]) and then decon_reg_start() -> decon_reg_update_req_global()
 * (bit 31). The global bit only covers DECON 0x0000-0x011C and
 * 0x0230-0x209C; the per-window requests are what make the window's DPP/IDMA
 * channel take its new SFRs (base address, format, sizes). With the global
 * bit alone (trials q39/q45) the DECON shadow cleared and FRAME_COUNT
 * advanced, but IDMA kept scanning the old base.
 */
void dpu_hw_update_req(struct dpu_dev *d)
{
	decon_wr_mask(d, SHADOW_REG_UPDATE_REQ, ~0U, SHADOW_REG_UPDATE_REQ_FOR_DECON);
	decon_wr_mask(d, SHADOW_REG_UPDATE_REQ, ~0U, SHADOW_REG_UPDATE_REQ_GLOBAL);
}

/*
 * The IDMA SFRs in use: 4.9 dpp_drv.c dma_dump_regs() prints them as the
 * "SHADOW SFR DUMP" at dma_regs + 0x800 (same layout as the SFRs at +0).
 */
#define IDMA_ACTIVE_OFS	0x800

u32 dpu_hw_dma_active_rd(struct dpu_dev *d, int dpp, u32 off)
{
	return dma_rd(d, dpp, IDMA_ACTIVE_OFS + off);
}

/* ---- DPP ------------------------------------------------------------- */

/*
 * 4.9 dpp_reg_init(), except that the DPP/IDMA interrupts stay masked:
 * this driver does not handle DPP interrupts yet.
 */
void dpu_hw_dpp_init(struct dpu_dev *d, int id)
{
	dma_wr_mask(d, id, IDMA_IRQ, ~0U, IDMA_ALL_IRQ_MASK);
	dpp_wr_mask(d, id, DPP_IRQ, ~0U, DPP_ALL_IRQ_MASK);

	/* dma/dpp_reg_set_clock_gate_en_all(id, 0) */
	dma_wr_mask(d, id, IDMA_ENABLE, 0, IDMA_ALL_CLOCK_GATE_EN_MASK);
	dpp_wr_mask(d, id, DPP_ENABLE, 0, DPP_ALL_CLOCK_GATE_EN_MASK);

	/* dpp_reg_set_lookup_table(): DPU DMA common QoS LUTs */
	writel(0x44444444, d->dma_com + DPU_DMA_IN_QOS_LUT07_00);
	writel(0x44444444, d->dma_com + DPU_DMA_IN_QOS_LUT15_08);

	/* dpp_reg_set_dynamic_clock_gating(id, 0) */
	dma_wr_mask(d, id, IDMA_DYNAMIC_GATING_EN, 0, IDMA_DG_EN_ALL);
	dpp_wr_mask(d, id, DPP_DYNAMIC_GATING_EN, 0, DPP_DG_EN_ALL);

	/* dpp_reg_set_plane_alpha_fixed() */
	dma_wr_mask(d, id, IDMA_OUT_CON, IDMA_OUT_FRAME_ALPHA(0xff),
		    IDMA_OUT_FRAME_ALPHA_MASK);

	/*
	 * 4.9 dpp_reg_init(), CONFIG_EXYNOS_AFBC: dma_reg_set_recovery_num(id,
	 * INIT_RCV_NUM), AFBC error-recovery window in ACLK cycles (630000 =
	 * 1 ms at 630 MHz) in the DPU DMA common block.
	 */
	writel((readl(d->dma_com + DPU_DMA_RECOVERY_NUM_CTRL) & ~DPU_DMA_RECOVERY_NUM_MASK) |
	       DPU_DMA_RECOVERY_NUM(630000), d->dma_com + DPU_DMA_RECOVERY_NUM_CTRL);
}

/*
 * ---- DPP per-frame parameters and limits (4.9 dpp_drv.c) ---------------
 *
 * Channels (4.9 dpp_check_format()): G0/G1 RGB only; VG0/VG1 add YUV;
 * VGF0/VGF1 add scaling and AFBC; VGF1 (VGRF) adds 90/270 rotation and HDR.
 * Flips and 180 are accepted on every channel, as on 4.9.
 */

/* 4.9 decon.h enum dpp_rotate / dpp_hdr_standard / dpp_csc_eq */
#define DPU_ROT_180		3
#define DPU_ROT_270		7
#define DPU_HDR_OFF		0
#define DPU_HDR_ST2084		1
#define DPU_HDR_HLG		2
#define DPU_CSC_BT_601		0
#define DPU_CSC_BT_2020		2
#define DPU_CSC_DCI_P3		3
#define DPU_CSC_STD_UNSPEC	63
#define DPU_CSC_RANGE_SHIFT	6
#define DPU_CSC_RANGE_LIMITED	0
#define DPU_CSC_RANGE_UNSPEC	7

/* 4.9 dpp.h size constraints */
#define DPU_SRC_WIDTH_MAX	65534
#define DPU_SRC_HEIGHT_MAX	8190
#define DPU_IMG_MIN		16
#define DPU_IMG_MAX		4096
#define DPU_IMG_ROT_HEIGHT_MAX	2160
#define DPU_SCALED_MIN		16
#define DPU_SCALED_MAX		4096
#define DPU_BLK_WIDTH_MIN	4
#define DPU_BLK_HEIGHT_MIN	1

/* 4.9 include/uapi/linux/videodev2_exynos_media.h plane layouts */
#define DPU_ALIGN(x, a)			(((x) + ((a) - 1)) & ~((u64)(a) - 1))
#define NV12N_Y_SIZE(w, h)		(DPU_ALIGN((w), 16) * DPU_ALIGN((h), 16) + 256)
#define NV12N_CBCR_SIZE(w, h)		(DPU_ALIGN((DPU_ALIGN((w), 16) * (DPU_ALIGN((h), 16) / 2) + 256), 16))
#define NV12N_10B_Y_8B_SIZE(w, h)	(DPU_ALIGN((w), 64) * DPU_ALIGN((h), 16) + 256)
#define NV12N_10B_Y_2B_SIZE(w, h)	((DPU_ALIGN((w) / 4, 16) * DPU_ALIGN((h), 16) + 64))
#define NV12N_10B_CBCR_8B_SIZE(w, h)	(DPU_ALIGN((DPU_ALIGN((w), 64) * (DPU_ALIGN((h), 16) / 2) + 256), 16))
#define NV12N_10B_CBCR_2B_SIZE(w, h)	((DPU_ALIGN((w) / 4, 16) * (DPU_ALIGN((h), 16) / 2) + 64))
#define NV12M_Y_SIZE(w, h)		(DPU_ALIGN((w), 64) * DPU_ALIGN((h), 16) + 256)
#define NV12M_CBCR_SIZE(w, h)		((DPU_ALIGN((w), 64) * DPU_ALIGN((h), 16) / 2) + 256)
#define NV12M_Y_2B_SIZE(w, h)		(DPU_ALIGN((w) / 4, 16) * DPU_ALIGN((h), 16) + 256)
#define NV12M_CBCR_2B_SIZE(w, h)	((DPU_ALIGN((w) / 4, 16) * DPU_ALIGN((h), 16) / 2) + 256)
#define S10B_2B_STRIDE(w)		(DPU_ALIGN((w) / 4, 16))

/* 4.9 dpp.h is_yuv() / is_yuv420(), on the DPP-level format */
static bool fmt_is_yuv(u32 f)
{
	return f >= DECON_PIXEL_FORMAT_NV16 && f < DECON_PIXEL_FORMAT_MAX;
}

static bool fmt_is_yuv420(u32 f)
{
	return f >= DECON_PIXEL_FORMAT_NV12 && f <= DECON_PIXEL_FORMAT_YVU420M;
}

bool dpu_fmt_is_yuv(u32 dpp_fmt)
{
	return fmt_is_yuv(dpp_fmt);
}

/* 4.9 struct dpp_params_info, the parts derived from a window config */
struct dpu_dpp_p {
	u32 src_w, src_h;	/* crop, after 90/270 rotation */
	u32 h_ratio, v_ratio;	/* 12.20 fixed point, src / dst */
	bool scale, block, rot90;
};

/* 4.9 dpp_get_params() */
static void dpp_get_params(const struct decon_win_config *c, u32 fmt, struct dpu_dpp_p *p)
{
	const struct decon_win_rect *b = &c->block_area;

	p->rot90 = c->dpp_parm.rot > DPU_ROT_180;
	p->src_w = p->rot90 ? c->src.h : c->src.w;
	p->src_h = p->rot90 ? c->src.w : c->src.h;
	/* dst.w/h >= 16: checked before (4.9 decon_check_limitation()) */
	p->h_ratio = div_u64((u64)p->src_w << 20, max(c->dst.w, 1U));
	p->v_ratio = div_u64((u64)p->src_h << 20, max(c->dst.h, 1U));
	p->scale = p->h_ratio != (1 << 20) || p->v_ratio != (1 << 20);
	/*
	 * 4.9: block mode unless rotated, scaled, YUV or too small. Two
	 * deviations, both only drop a fetch optimisation: no block for AFBC
	 * (4.9 then fails the DPP check and disables the whole window) and no
	 * block that is not inside the source crop (4.9 programs it as is).
	 */
	p->block = !c->dpp_parm.rot && !p->scale && fmt < DECON_PIXEL_FORMAT_NV16 &&
		   !c->compression && b->w >= DPU_BLK_WIDTH_MIN && b->h >= DPU_BLK_HEIGHT_MIN &&
		   b->x >= 0 && b->y >= 0 && b->x + b->w <= c->src.w && b->y + b->h <= c->src.h;
}

/*
 * 4.9 dpp_check_limitation() with dpp_check_scale_ratio(), dpp_check_format()
 * and dpp_check_size(), on the DPP-level format @fmt. NULL if the DPP can
 * scan the window, else why not. On 4.9 a failure here disabled the window
 * for that frame (decon_set_dpp_config()); the caller does the same. The
 * window-level checks (4.9 decon_check_limitation()) come first.
 */
const char *dpu_hw_dpp_check(const struct decon_win_config *c, u32 fmt)
{
	const struct decon_frame *s = &c->src, *t = &c->dst;
	u32 id = c->idma_type, hdr = c->dpp_parm.hdr_std, al;
	bool comp = c->compression, vgf = id == DECON_IDMA_VGF0 || id == DECON_IDMA_VGF1;
	struct dpu_dpp_p p;

	if (id >= DPU_NR_DPP)
		return "idma_type out of range";
	if (c->dpp_parm.rot > DPU_ROT_270)
		return "rotation value out of range";
	dpp_get_params(c, fmt, &p);

	/* dpp_check_scale_ratio(): 1/2x down to 8x up */
	if (p.src_w > t->w * 2 || p.src_h > t->h * 2)
		return "scale-down below 1/2x";
	if (p.src_w < (t->w + 7) / 8 || p.src_h < (t->h + 7) / 8)
		return "scale-up above 8x";

	/* dpp_check_format() */
	if (id != DECON_IDMA_VGF1 && p.rot90)
		return "90/270 rotation outside VGF1 (VGRF)";
	if (id != DECON_IDMA_VGF1 && hdr > DPU_HDR_OFF)
		return "HDR outside VGF1 (VGRF)";
	if (hdr > DPU_HDR_HLG)
		return "unknown HDR standard";
	if ((id == DECON_IDMA_G0 || id == DECON_IDMA_G1) && fmt_is_yuv(fmt))
		return "YUV on a G channel";
	if (!vgf && comp)
		return "AFBC outside VGF0/VGF1";
	if (!vgf && p.scale)
		return "scaling outside VGF0/VGF1";
	switch (fmt) {
	case DECON_PIXEL_FORMAT_ARGB_8888:
	case DECON_PIXEL_FORMAT_ABGR_8888:
	case DECON_PIXEL_FORMAT_RGBA_8888:
	case DECON_PIXEL_FORMAT_BGRA_8888:
	case DECON_PIXEL_FORMAT_XRGB_8888:
	case DECON_PIXEL_FORMAT_XBGR_8888:
	case DECON_PIXEL_FORMAT_RGBX_8888:
	case DECON_PIXEL_FORMAT_BGRX_8888:
	case DECON_PIXEL_FORMAT_RGB_565:
	case DECON_PIXEL_FORMAT_NV12:
	case DECON_PIXEL_FORMAT_NV12M:
	case DECON_PIXEL_FORMAT_NV21:
	case DECON_PIXEL_FORMAT_NV21M:
	case DECON_PIXEL_FORMAT_NV12N:
	case DECON_PIXEL_FORMAT_NV12N_10B:
	case DECON_PIXEL_FORMAT_ARGB_2101010:
	case DECON_PIXEL_FORMAT_ABGR_2101010:
	case DECON_PIXEL_FORMAT_RGBA_1010102:
	case DECON_PIXEL_FORMAT_BGRA_1010102:
	case DECON_PIXEL_FORMAT_NV12M_P010:
	case DECON_PIXEL_FORMAT_NV21M_P010:
	case DECON_PIXEL_FORMAT_NV12M_S10B:
	case DECON_PIXEL_FORMAT_NV21M_S10B:
		break;
	default:
		return "pixel format not supported by the DPP";
	}

	/* dpp_check_limitation() combinations (block ones: see dpp_get_params) */
	if (comp && c->dpp_parm.rot)
		return "AFBC with rotation/flip";
	if (comp && fmt_is_yuv420(fmt))
		return "AFBC with YUV420";
	if (hdr != DPU_HDR_OFF && comp)
		return "HDR with AFBC";
	if (hdr != DPU_HDR_OFF && c->dpp_parm.rot)
		return "HDR with rotation/flip";

	/* dpp_check_size() + dpp_constraints_params() */
	if (s->x < 0 || s->y < 0 || t->x < 0 || t->y < 0)
		return "negative source or destination position";
	al = fmt_is_yuv(fmt) ? 2 : 1;
	if (!IS_ALIGNED(s->x, al) || !IS_ALIGNED(s->y, al) || !IS_ALIGNED(s->f_w, al) ||
	    !IS_ALIGNED(s->f_h, al) || !IS_ALIGNED(s->w, al) || !IS_ALIGNED(s->h, al))
		return "YUV source not 2-pixel aligned";
	if (s->w > DPU_IMG_MAX || s->w < DPU_IMG_MIN * al ||
	    s->h > (p.rot90 ? DPU_IMG_ROT_HEIGHT_MAX : DPU_IMG_MAX) || s->h < DPU_IMG_MIN * al)
		return "source crop size out of range";
	if (t->w > DPU_SCALED_MAX || t->w < DPU_SCALED_MIN ||
	    t->h > DPU_SCALED_MAX || t->h < DPU_SCALED_MIN)
		return "scaled (destination) size out of range";
	/* + IDMA field widths: SRC_OFFSET 13 bits, SRC_SIZE 16/14 bits */
	if (s->x + s->w > DPU_SRC_WIDTH_MAX || s->y + s->h > DPU_SRC_HEIGHT_MAX || s->x > 0x1fff ||
	    s->f_w > DPU_SRC_WIDTH_MAX || s->f_h > DPU_SRC_HEIGHT_MAX)
		return "source boundary out of range";
	if (s->x + s->w > s->f_w || s->y + s->h > s->f_h)
		return "source crop outside the buffer";
	return NULL;
}

/*
 * Planes (fds) the DPP reads for the DPP-level format @fmt, 4.9
 * dpu_get_plane_cnt(fmt, false), and the bytes each must hold for a
 * @f_w x @f_h buffer, following the 4.9 address derivation in
 * dpu_hw_dpp_config(). 4.9 only checked plane 0 >= f_w * f_h * Bpp and
 * relied on the SysMMU for the rest; without it the IDMA would read
 * physical memory beyond the buffer, so every plane is checked here.
 * Returns 0 for formats the DPP check refuses.
 */
int dpu_fmt_planes(u32 fmt, u32 f_w, u32 f_h, u64 need[2])
{
	u64 px = (u64)f_w * f_h;

	need[1] = 0;
	switch (fmt) {
	case DECON_PIXEL_FORMAT_ARGB_8888 ... DECON_PIXEL_FORMAT_BGRX_8888:
	case DECON_PIXEL_FORMAT_ARGB_2101010 ... DECON_PIXEL_FORMAT_BGRA_1010102:
		need[0] = px * 4;
		return 1;
	case DECON_PIXEL_FORMAT_RGB_565:
		need[0] = px * 2;
		return 1;
	case DECON_PIXEL_FORMAT_NV12N:
		need[0] = NV12N_Y_SIZE(f_w, f_h) + NV12N_CBCR_SIZE(f_w, f_h);
		return 1;
	case DECON_PIXEL_FORMAT_NV12N_10B:
		need[0] = NV12N_10B_Y_8B_SIZE(f_w, f_h) + NV12N_10B_Y_2B_SIZE(f_w, f_h) +
			  NV12N_10B_CBCR_8B_SIZE(f_w, f_h) + NV12N_10B_CBCR_2B_SIZE(f_w, f_h);
		return 1;
	case DECON_PIXEL_FORMAT_NV12:
	case DECON_PIXEL_FORMAT_NV21:
	case DECON_PIXEL_FORMAT_NV12M:
	case DECON_PIXEL_FORMAT_NV21M:
		need[0] = px;
		need[1] = px / 2;
		return 2;
	case DECON_PIXEL_FORMAT_NV12M_P010:
	case DECON_PIXEL_FORMAT_NV21M_P010:
		need[0] = px * 2;
		need[1] = px;
		return 2;
	case DECON_PIXEL_FORMAT_NV12M_S10B:
	case DECON_PIXEL_FORMAT_NV21M_S10B:
		need[0] = NV12M_Y_SIZE(f_w, f_h) + NV12M_Y_2B_SIZE(f_w, f_h);
		need[1] = NV12M_CBCR_SIZE(f_w, f_h) + NV12M_CBCR_2B_SIZE(f_w, f_h);
		return 2;
	default:
		need[0] = 0;
		return 0;
	}
}

static int sc_ratio_sel(u32 ratio)
{
	if (ratio <= DPP_SC_RATIO_MAX)
		return 0;
	if (ratio <= DPP_SC_RATIO_7_8)
		return 1;
	if (ratio <= DPP_SC_RATIO_6_8)
		return 2;
	if (ratio <= DPP_SC_RATIO_5_8)
		return 3;
	if (ratio <= DPP_SC_RATIO_4_8)
		return 4;
	if (ratio <= DPP_SC_RATIO_3_8)
		return 5;
	return 6;
}

/* 4.9 dpp_reg_set_scale_ratio(): ratio registers and poly-phase coefficients */
static void dpp_set_scale_ratio(struct dpu_dev *d, int id, u32 h_ratio, u32 v_ratio)
{
	int i, j, k, hs = sc_ratio_sel(h_ratio), vs = sc_ratio_sel(v_ratio);

	dpp_wr_mask(d, id, DPP_MAIN_H_RATIO, DPP_H_RATIO(h_ratio), DPP_H_RATIO_MASK);
	for (i = 0; i < 9; i++)
		for (j = 0; j < 8; j++)
			for (k = 0; k < 2; k++)
				dpp_wr(d, id, DPP_H_COEF(i, j, k),
				       (u32)(s32)h_coef_8t[hs][i][j]);

	dpp_wr_mask(d, id, DPP_MAIN_V_RATIO, DPP_V_RATIO(v_ratio), DPP_V_RATIO_MASK);
	for (i = 0; i < 9; i++)
		for (j = 0; j < 4; j++)
			for (k = 0; k < 2; k++)
				dpp_wr(d, id, DPP_V_COEF(i, j, k),
				       (u32)(s32)v_coef_4t[vs][i][j]);
}

/*
 * 4.9 dpp_reg_set_format(): IDMA format, DPP alpha select and DPP input
 * type, then (CONFIG_EXYNOS_AFBC=y) AFBC decoding and its error recovery,
 * which the IDMA only has on VGF0/VGF1 (4.9 dma_reg_set_afbc_en() returns
 * early elsewhere).
 */
static int dpp_set_format(struct dpu_dev *d, int id, u32 dpp_fmt, bool comp)
{
	u32 fmt, type = DPP_IMG_FORMAT_ARGB8888, a_sel = 0;

	switch (dpp_fmt) {
	case DECON_PIXEL_FORMAT_ARGB_8888:
		fmt = IDMA_IMG_FORMAT_ARGB8888; a_sel = 1; break;
	case DECON_PIXEL_FORMAT_ABGR_8888:
		fmt = IDMA_IMG_FORMAT_ABGR8888; a_sel = 1; break;
	case DECON_PIXEL_FORMAT_RGBA_8888:
		fmt = IDMA_IMG_FORMAT_RGBA8888; a_sel = 1; break;
	case DECON_PIXEL_FORMAT_BGRA_8888:
		fmt = IDMA_IMG_FORMAT_BGRA8888; a_sel = 1; break;
	case DECON_PIXEL_FORMAT_XRGB_8888:
		fmt = IDMA_IMG_FORMAT_XRGB8888; break;
	case DECON_PIXEL_FORMAT_XBGR_8888:
		fmt = IDMA_IMG_FORMAT_XBGR8888; break;
	case DECON_PIXEL_FORMAT_RGBX_8888:
		fmt = IDMA_IMG_FORMAT_RGBX8888; break;
	case DECON_PIXEL_FORMAT_BGRX_8888:
		fmt = IDMA_IMG_FORMAT_BGRX8888; break;
	case DECON_PIXEL_FORMAT_RGB_565:
		fmt = comp ? IDMA_IMG_FORMAT_BGR565 : IDMA_IMG_FORMAT_RGB565; break;
	case DECON_PIXEL_FORMAT_ARGB_2101010:
		fmt = IDMA_IMG_FORMAT_ARGB2101010;
		type = DPP_IMG_FORMAT_ARGB8101010; a_sel = 1; break;
	case DECON_PIXEL_FORMAT_ABGR_2101010:
		fmt = IDMA_IMG_FORMAT_ABGR2101010;
		type = DPP_IMG_FORMAT_ARGB8101010; a_sel = 1; break;
	case DECON_PIXEL_FORMAT_RGBA_1010102:
		fmt = IDMA_IMG_FORMAT_RGBA2101010;
		type = DPP_IMG_FORMAT_ARGB8101010; a_sel = 1; break;
	case DECON_PIXEL_FORMAT_BGRA_1010102:
		fmt = IDMA_IMG_FORMAT_BGRA2101010;
		type = DPP_IMG_FORMAT_ARGB8101010; a_sel = 1; break;
	case DECON_PIXEL_FORMAT_NV12:
	case DECON_PIXEL_FORMAT_NV12M:
		fmt = IDMA_IMG_FORMAT_YUV420_2P; type = DPP_IMG_FORMAT_YUV420_8P; break;
	case DECON_PIXEL_FORMAT_NV21:
	case DECON_PIXEL_FORMAT_NV21M:
	case DECON_PIXEL_FORMAT_NV12N:
		fmt = IDMA_IMG_FORMAT_YVU420_2P; type = DPP_IMG_FORMAT_YUV420_8P; break;
	case DECON_PIXEL_FORMAT_NV12N_10B:
		fmt = IDMA_IMG_FORMAT_YVU420_8P2; type = DPP_IMG_FORMAT_YUV420_8P2; break;
	case DECON_PIXEL_FORMAT_NV12M_P010:
		fmt = IDMA_IMG_FORMAT_YUV420_P010; type = DPP_IMG_FORMAT_YUV420_P010; break;
	case DECON_PIXEL_FORMAT_NV21M_P010:
		fmt = IDMA_IMG_FORMAT_YVU420_P010; type = DPP_IMG_FORMAT_YUV420_P010; break;
	case DECON_PIXEL_FORMAT_NV12M_S10B:
		fmt = IDMA_IMG_FORMAT_YVU420_8P2; type = DPP_IMG_FORMAT_YUV420_8P2; break;
	case DECON_PIXEL_FORMAT_NV21M_S10B:
		fmt = IDMA_IMG_FORMAT_YUV420_8P2; type = DPP_IMG_FORMAT_YUV420_8P2; break;
	default:
		return -EINVAL;
	}

	dma_wr_mask(d, id, IDMA_IN_CON, IDMA_IMG_FORMAT(fmt), IDMA_IMG_FORMAT_MASK);
	dpp_wr_mask(d, id, DPP_IN_CON, DPP_ALPHA_SEL(a_sel), DPP_ALPHA_SEL_MASK);
	dpp_wr_mask(d, id, DPP_IN_CON, DPP_IMG_FORMAT(type), DPP_IMG_FORMAT_MASK);

	if (id == DECON_IDMA_VGF0 || id == DECON_IDMA_VGF1) {
		dma_wr_mask(d, id, IDMA_IN_CON, comp ? ~0U : 0, IDMA_AFBC_EN);
		dma_wr_mask(d, id, IDMA_RECOVERY_CTRL, comp ? ~0U : 0, IDMA_RECOVERY_EN);
	} else if (comp) {
		return -EINVAL;		/* refused by dpu_hw_dpp_check() */
	}
	return 0;
}

/*
 * 4.9 dpp_reg_set_csc_config(), VG and VGF channels: CSC standard and range
 * from the window eq_mode, hard-wired coefficients for BT.601/709/2020 and
 * DCI-P3. 4.9 selects "customised" for any other standard but never writes
 * coefficients (SUPPORT_USER_COEF is not defined), so neither does this.
 * The DPP applies the CSC to YUV input only.
 */
static void dpp_set_csc(struct dpu_dev *d, int id, u32 eq_mode)
{
	u32 std = eq_mode & 0x3f, rng = (eq_mode >> DPU_CSC_RANGE_SHIFT) & 0x7;

	if (id == DECON_IDMA_G0 || id == DECON_IDMA_G1)
		return;
	if (std == DPU_CSC_STD_UNSPEC)
		std = DPU_CSC_BT_601;
	if (rng == DPU_CSC_RANGE_UNSPEC)
		rng = DPU_CSC_RANGE_LIMITED;
	dpp_wr_mask(d, id, DPP_IN_CON,
		    DPP_CSC_TYPE(std) | DPP_CSC_RANGE(rng) | DPP_CSC_MODE(std > DPU_CSC_DCI_P3),
		    DPP_CSC_TYPE_MASK | DPP_CSC_RANGE_MASK | DPP_CSC_MODE_MASK);
}

/* two 14-bit LUT entries per register (4.9 dpp_reg_set_eotf_lut() etc.) */
static void dpp_set_lut(struct dpu_dev *d, int id, u32 base, const u16 *lut, int n)
{
	int i;

	for (i = 0; i < n; i++)
		dpp_wr_mask(d, id, base + (i / 2) * 4, ((u32)lut[i] & 0x3fff) << (16 * (i % 2)),
			    0x3fffU << (16 * (i % 2)));
}

/*
 * 4.9 dpp_reg_set_hdr(), VGF1 (VGRF) on every frame: HDR, EOTF and tone
 * map on for ST2084/HLG windows, the gamut map enabled unless the window
 * is DCI-P3 (with HDR off it has no effect; 4.9 sets it anyway), and the
 * LUTs loaded while HDR is on.
 */
static void dpp_set_hdr(struct dpu_dev *d, int id, const struct decon_win_config *c)
{
	u32 hdr = c->dpp_parm.hdr_std, eq = c->dpp_parm.eq_mode;
	u32 maxl = c->dpp_parm.max_luminance;
	bool en = hdr == DPU_HDR_ST2084 || hdr == DPU_HDR_HLG;
	int i;

	dpp_wr_mask(d, id, DPP_VGRF_HDR_CON, en ? ~0U : 0, DPP_HDR_ON_MASK);
	dpp_wr_mask(d, id, DPP_VGRF_HDR_CON, en ? ~0U : 0, DPP_EOTF_ON_MASK);
	dpp_wr_mask(d, id, DPP_VGRF_HDR_CON, eq != DPU_CSC_DCI_P3 ? ~0U : 0, DPP_GM_ON_MASK);
	dpp_wr_mask(d, id, DPP_VGRF_HDR_CON, en ? ~0U : 0, DPP_TM_ON_MASK);
	if (!en)
		return;

	/* dpp_reg_set_eotf_lut() */
	if (hdr == DPU_HDR_ST2084 && maxl > 1000) {
		dpp_set_lut(d, id, DPP_HDR_EOTF_X_AXIS_ADDR(0), eotf_x_axis_st2084_4000, DPP_HDR_MAX_EOTF);
		dpp_set_lut(d, id, DPP_HDR_EOTF_Y_AXIS_ADDR(0), eotf_y_axis_st2084_4000, DPP_HDR_MAX_EOTF);
	} else if (hdr == DPU_HDR_ST2084) {
		dpp_set_lut(d, id, DPP_HDR_EOTF_X_AXIS_ADDR(0), eotf_x_axis_st2084_1000, DPP_HDR_MAX_EOTF);
		dpp_set_lut(d, id, DPP_HDR_EOTF_Y_AXIS_ADDR(0), eotf_y_axis_st2084_1000, DPP_HDR_MAX_EOTF);
	} else {
		dpp_set_lut(d, id, DPP_HDR_EOTF_X_AXIS_ADDR(0), eotf_x_axis_hlg, DPP_HDR_MAX_EOTF);
		dpp_set_lut(d, id, DPP_HDR_EOTF_Y_AXIS_ADDR(0), eotf_y_axis_hlg, DPP_HDR_MAX_EOTF);
	}

	/* dpp_reg_set_gm_lut(): BT.2020 only; DCI-P3 and others write nothing */
	if (eq == DPU_CSC_BT_2020)
		for (i = 0; i < DPP_HDR_MAX_GM; i++)
			dpp_wr_mask(d, id, DPP_HDR_GM_COEF_ADDR(i), (u32)gm_coef_2020_p3[i],
				    DPP_HDR_GM_COEF_MASK);

	/* dpp_reg_set_tm_lut(), tunable off */
	if (maxl > 1000 && maxl < 10000) {
		dpp_set_lut(d, id, DPP_HDR_TM_X_AXIS_ADDR(0), tm_x_axis_gamma_2P2_4000, DPP_HDR_MAX_TM);
		dpp_set_lut(d, id, DPP_HDR_TM_Y_AXIS_ADDR(0), tm_y_axis_gamma_2P2_4000, DPP_HDR_MAX_TM);
	} else {
		dpp_set_lut(d, id, DPP_HDR_TM_X_AXIS_ADDR(0), tm_x_axis_gamma_2P2_1000, DPP_HDR_MAX_TM);
		dpp_set_lut(d, id, DPP_HDR_TM_Y_AXIS_ADDR(0), tm_y_axis_gamma_2P2_1000, DPP_HDR_MAX_TM);
	}
}

/*
 * 4.9 dpp_reg_configure_params() for a window dpu_hw_dpp_check() accepted:
 * CSC, scale ratio and coefficients, sizes, rotation, plane addresses,
 * block area, format/AFBC and (VGF1) HDR. @y is plane 0 (fd_idma[0]),
 * @cb plane 1 (fd_idma[1]) of the two-plane YUV formats; the other planes
 * are derived as 4.9 dpp_get_params() does. The initial scaler phase is
 * not written, as on 4.9.
 */
int dpu_hw_dpp_config(struct dpu_dev *d, int id, const struct decon_win_config *c,
		      u32 dpp_fmt, dma_addr_t y, dma_addr_t cb)
{
	const struct decon_frame *s = &c->src, *t = &c->dst;
	const struct decon_win_rect *b = &c->block_area;
	u64 a[4] = { y, y, 0, 0 };
	bool is_4p = false;
	struct dpu_dpp_p p;
	u32 strd2 = 0;

	if (id < 0 || id >= DPU_NR_DPP)
		return -EINVAL;
	dpp_get_params(c, dpp_fmt, &p);

	switch (dpp_fmt) {
	case DECON_PIXEL_FORMAT_NV12:
	case DECON_PIXEL_FORMAT_NV21:
	case DECON_PIXEL_FORMAT_NV12M:
	case DECON_PIXEL_FORMAT_NV21M:
	case DECON_PIXEL_FORMAT_NV12M_P010:
	case DECON_PIXEL_FORMAT_NV21M_P010:
		a[1] = cb;
		break;
	case DECON_PIXEL_FORMAT_NV12N:
		a[1] = y + NV12N_Y_SIZE(s->f_w, s->f_h);
		break;
	case DECON_PIXEL_FORMAT_NV12M_S10B:
	case DECON_PIXEL_FORMAT_NV21M_S10B:
		a[1] = cb;
		a[2] = y + NV12M_Y_SIZE(s->f_w, s->f_h);
		a[3] = cb + NV12M_CBCR_SIZE(s->f_w, s->f_h);
		is_4p = true;
		break;
	case DECON_PIXEL_FORMAT_NV12N_10B:
		a[1] = y + NV12N_10B_Y_8B_SIZE(s->f_w, s->f_h) + NV12N_10B_Y_2B_SIZE(s->f_w, s->f_h);
		a[2] = y + NV12N_10B_Y_8B_SIZE(s->f_w, s->f_h);
		a[3] = a[1] + NV12N_10B_CBCR_8B_SIZE(s->f_w, s->f_h);
		is_4p = true;
		break;
	default:
		break;	/* one plane: C mirrors Y (4.9 1p address for AFBC) */
	}
	if (is_4p)
		strd2 = S10B_2B_STRIDE(s->f_w);
	if (upper_32_bits(a[0]) || upper_32_bits(a[1]) || upper_32_bits(a[2]) ||
	    upper_32_bits(a[3]))
		return -EINVAL;

	if (!test_bit(id, &d->dpp_inited) && id != d->boot_dpp) {
		dpu_hw_dpp_init(d, id);
		set_bit(id, &d->dpp_inited);
	}

	dpp_set_csc(d, id, c->dpp_parm.eq_mode);
	dpp_set_scale_ratio(d, id, p.h_ratio, p.v_ratio);

	/* dpp_reg_set_size() */
	dma_wr(d, id, IDMA_SRC_OFFSET, IDMA_SRC_OFFSET_Y(s->y) | IDMA_SRC_OFFSET_X(s->x));
	dma_wr(d, id, IDMA_SRC_SIZE, IDMA_SRC_HEIGHT(s->f_h) | IDMA_SRC_WIDTH(s->f_w));
	dma_wr(d, id, IDMA_IMG_SIZE, IDMA_IMG_HEIGHT(s->h) | IDMA_IMG_WIDTH(s->w));
	dpp_wr(d, id, DPP_IMG_SIZE, DPP_IMG_HEIGHT(p.src_h) | DPP_IMG_WIDTH(p.src_w));
	if (id == DECON_IDMA_VGF0 || id == DECON_IDMA_VGF1)
		dpp_wr(d, id, DPP_SCALED_IMG_SIZE,
		       DPP_SCALED_IMG_HEIGHT(t->h) | DPP_SCALED_IMG_WIDTH(t->w));

	/* dma_reg_set_rotation() */
	dma_wr_mask(d, id, IDMA_IN_CON, IDMA_ROTATION(c->dpp_parm.rot), IDMA_ROTATION_MASK);

	/* dpp_reg_set_buf_addr(): 4p with the 2-bit strides, else Y and C */
	dma_wr(d, id, IDMA_IN_BASE_ADDR_Y, lower_32_bits(a[0]));
	dma_wr(d, id, IDMA_IN_BASE_ADDR_C, lower_32_bits(a[1]));
	if (is_4p) {
		dma_wr(d, id, IDMA_IN_BASE_ADDR_Y2, lower_32_bits(a[2]));
		dma_wr(d, id, IDMA_IN_BASE_ADDR_C2, lower_32_bits(a[3]));
		dma_wr_mask(d, id, IDMA_2BIT_STRIDE,
			    IDMA_LUMA_2B_STRIDE(strd2) | IDMA_CHROMA_2B_STRIDE(strd2),
			    IDMA_LUMA_2B_STRIDE_MASK | IDMA_CHROMA_2B_STRIDE_MASK);
	}

	/* dpp_reg_set_block_area() */
	if (p.block) {
		dma_wr(d, id, IDMA_BLOCK_OFFSET, IDMA_BLK_OFFSET_Y(b->y) | IDMA_BLK_OFFSET_X(b->x));
		dma_wr(d, id, IDMA_BLOCK_SIZE, IDMA_BLK_HEIGHT(b->h) | IDMA_BLK_WIDTH(b->w));
	}
	dma_wr_mask(d, id, IDMA_IN_CON, p.block ? ~0U : 0, IDMA_BLOCK_EN);

	if (dpp_set_format(d, id, dpp_fmt, c->compression))
		return -EINVAL;
	if (id == DECON_IDMA_VGF1)
		dpp_set_hdr(d, id, c);
	return 0;
}

/* scaler state as programmed, for logs: H/V ratio and the scaled size */
void dpu_hw_dpp_scale_rd(struct dpu_dev *d, int id, u32 *h, u32 *v, u32 *scaled)
{
	*h = readl(d->dpp_regs + id * DPP_STRIDE + DPP_MAIN_H_RATIO) & DPP_H_RATIO_MASK;
	*v = readl(d->dpp_regs + id * DPP_STRIDE + DPP_MAIN_V_RATIO) & DPP_V_RATIO_MASK;
	*scaled = (id == DECON_IDMA_VGF0 || id == DECON_IDMA_VGF1) ?
		  readl(d->dpp_regs + id * DPP_STRIDE + DPP_SCALED_IMG_SIZE) : 0;
}

/* ---- DECON windows (4.9 decon_reg_set_window_control) ---------------- */

static void win_set_bnd_function(struct dpu_dev *d, int win, int plane_a, u32 blend)
{
	u32 pd_func = PD_FUNC_USER_DEFINED;
	u8 alpha0 = 0xff, alpha1 = 0xff;
	bool is_plane_a = false;
	u32 af_d = BND_COEF_ONE, ab_d = BND_COEF_ZERO,
	    af_a = BND_COEF_ONE, ab_a = BND_COEF_ZERO;

	if (blend == DECON_BLENDING_NONE)
		pd_func = PD_FUNC_COPY;

	if (plane_a >= 0 && plane_a <= 0xff) {
		alpha0 = plane_a;
		alpha1 = 0;
		is_plane_a = true;
	}

	if (blend == DECON_BLENDING_COVERAGE && !is_plane_a) {
		af_d = BND_COEF_AF;
		ab_d = BND_COEF_1_M_AF;
		af_a = BND_COEF_AF;
		ab_a = BND_COEF_1_M_AF;
	} else if (blend == DECON_BLENDING_COVERAGE && is_plane_a) {
		af_d = BND_COEF_ALPHA_MULT;
		ab_d = BND_COEF_1_M_ALPHA_MULT;
		af_a = BND_COEF_ALPHA_MULT;
		ab_a = BND_COEF_1_M_ALPHA_MULT;
	} else if (blend == DECON_BLENDING_PREMULT && !is_plane_a) {
		af_d = BND_COEF_ONE;
		ab_d = BND_COEF_1_M_AF;
		af_a = BND_COEF_ONE;
		ab_a = BND_COEF_1_M_AF;
	} else if (blend == DECON_BLENDING_PREMULT && is_plane_a) {
		af_d = BND_COEF_PLNAE_ALPHA0;
		ab_d = BND_COEF_1_M_ALPHA_MULT;
		af_a = BND_COEF_PLNAE_ALPHA0;
		ab_a = BND_COEF_1_M_ALPHA_MULT;
	}

	decon_wr_mask(d, WIN_CONTROL_0(win), WIN_ALPHA1_F(alpha1) | WIN_ALPHA0_F(alpha0),
		      WIN_ALPHA1_MASK | WIN_ALPHA0_MASK);
	decon_wr_mask(d, WIN_CONTROL_0(win),
		      WIN_ALPHA_MULT_SRC_SEL_F(ALPHA_MULT_SRC_SEL_AF),
		      WIN_ALPHA_MULT_SRC_SEL_MASK);
	decon_wr_mask(d, WIN_CONTROL_0(win), WIN_FUNC_F(pd_func), WIN_FUNC_MASK);
	if (pd_func == PD_FUNC_USER_DEFINED)
		decon_wr_mask(d, WIN_CONTROL_1(win),
			      WIN_FG_ALPHA_D_SEL_F(af_d) | WIN_BG_ALPHA_D_SEL_F(ab_d) |
			      WIN_FG_ALPHA_A_SEL_F(af_a) | WIN_BG_ALPHA_A_SEL_F(ab_a),
			      WIN_FG_ALPHA_D_SEL_MASK | WIN_BG_ALPHA_D_SEL_MASK |
			      WIN_FG_ALPHA_A_SEL_MASK | WIN_BG_ALPHA_A_SEL_MASK);
}

void dpu_hw_win_config(struct dpu_dev *d, int win, const struct decon_win_config *c,
		       bool enable)
{
	bool colormap = enable && c->state == DECON_WIN_STATE_COLOR;
	u32 ch = dpu_hw_dpp_to_channel(c->idma_type < DPU_NR_DPP ? c->idma_type : 0);

	if (enable) {
		const struct decon_frame *f = &c->dst;

		win_set_bnd_function(d, win, c->plane_alpha, c->blending);
		decon_wr(d, WIN_START_POSITION(win),
			 WIN_STRPTR_Y_F(f->y) | WIN_STRPTR_X_F(f->x));
		decon_wr(d, WIN_END_POSITION(win),
			 WIN_ENDPTR_Y_F(f->y + f->h - 1) |
			 WIN_ENDPTR_X_F(f->x + f->w - 1));
		decon_wr(d, WIN_START_TIME_CONTROL(win), 0);
	}

	decon_wr_mask(d, DATA_PATH_CONTROL_0, enable ? ~0U : 0, WIN_EN_F(win));
	if (enable) {
		/* 4.9: colormap = color | 0xff000000 for COLOR, 0 otherwise */
		u32 argb = colormap ? (c->color | (0xffU << 24)) : 0;

		decon_wr_mask(d, DATA_PATH_CONTROL_0, colormap ? ~0U : 0,
			      WIN_MAPCOLOR_EN_F(win));
		decon_wr_mask(d, WIN_COLORMAP_0(win),
			      WIN_MAPCOLOR_A_F((argb >> 24) & 0xff) |
			      WIN_MAPCOLOR_R_F((argb >> 16) & 0xff),
			      WIN_MAPCOLOR_A_MASK | WIN_MAPCOLOR_R_MASK);
		decon_wr_mask(d, WIN_COLORMAP_1(win),
			      WIN_MAPCOLOR_G_F((argb >> 8) & 0xff) |
			      WIN_MAPCOLOR_B_F(argb & 0xff),
			      WIN_MAPCOLOR_G_MASK | WIN_MAPCOLOR_B_MASK);
	}
	decon_wr_mask(d, DATA_PATH_CONTROL_1, WIN_CHMAP_F(win, ch), WIN_CHMAP_MASK(win));
}

/* 4.9 helper.c dpu_translate_fmt_to_dpp(): its own inverse */
u32 dpu_translate_fmt_to_dpp(u32 format)
{
	switch (format) {
	case DECON_PIXEL_FORMAT_NV12: return DECON_PIXEL_FORMAT_NV21;
	case DECON_PIXEL_FORMAT_NV21: return DECON_PIXEL_FORMAT_NV12;
	case DECON_PIXEL_FORMAT_NV12M: return DECON_PIXEL_FORMAT_NV21M;
	case DECON_PIXEL_FORMAT_NV21M: return DECON_PIXEL_FORMAT_NV12M;
	case DECON_PIXEL_FORMAT_YUV420: return DECON_PIXEL_FORMAT_YVU420;
	case DECON_PIXEL_FORMAT_YVU420: return DECON_PIXEL_FORMAT_YUV420;
	case DECON_PIXEL_FORMAT_YUV420M: return DECON_PIXEL_FORMAT_YVU420M;
	case DECON_PIXEL_FORMAT_YVU420M: return DECON_PIXEL_FORMAT_YUV420M;
	case DECON_PIXEL_FORMAT_ARGB_8888: return DECON_PIXEL_FORMAT_BGRA_8888;
	case DECON_PIXEL_FORMAT_ABGR_8888: return DECON_PIXEL_FORMAT_RGBA_8888;
	case DECON_PIXEL_FORMAT_RGBA_8888: return DECON_PIXEL_FORMAT_ABGR_8888;
	case DECON_PIXEL_FORMAT_BGRA_8888: return DECON_PIXEL_FORMAT_ARGB_8888;
	case DECON_PIXEL_FORMAT_XRGB_8888: return DECON_PIXEL_FORMAT_BGRX_8888;
	case DECON_PIXEL_FORMAT_XBGR_8888: return DECON_PIXEL_FORMAT_RGBX_8888;
	case DECON_PIXEL_FORMAT_RGBX_8888: return DECON_PIXEL_FORMAT_XBGR_8888;
	case DECON_PIXEL_FORMAT_BGRX_8888: return DECON_PIXEL_FORMAT_XRGB_8888;
	default: return format;
	}
}
