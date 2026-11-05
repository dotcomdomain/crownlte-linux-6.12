// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos9810 DECON-F / DPP register layer.
 *
 * Ported from the Samsung 4.9 kernel (drivers/video/fbdev/exynos/dpu_9810/
 * decon_reg.c, dpp_reg.c, helper.c), restricted to unscaled, unrotated,
 * uncompressed RGB windows. Register sequences and values are unchanged;
 * the per-id accessors now take the device.
 */
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/regmap.h>

#include "dpu9810.h"
#include "dpp_coef.h"

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

/* 4.9 dpp_reg_set_format(), RGB entries only: IDMA format, DPP alpha sel */
static int dpp_set_format(struct dpu_dev *d, int id, u32 dpp_fmt)
{
	u32 fmt, a_sel = 0;

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
		fmt = IDMA_IMG_FORMAT_RGB565; break;
	default:
		return -EINVAL;
	}

	dma_wr_mask(d, id, IDMA_IN_CON, IDMA_IMG_FORMAT(fmt), IDMA_IMG_FORMAT_MASK);
	dpp_wr_mask(d, id, DPP_IN_CON, DPP_ALPHA_SEL(a_sel), DPP_ALPHA_SEL_MASK);
	dpp_wr_mask(d, id, DPP_IN_CON, DPP_IMG_FORMAT(DPP_IMG_FORMAT_ARGB8888),
		    DPP_IMG_FORMAT_MASK);
	return 0;
}

/*
 * 4.9 dpp_reg_configure_params() for an unscaled RGB window:
 * scale ratio 1:1, sizes, rotation 0, base address (Y = C), block off,
 * format. The CSC and HDR steps do not apply to RGB input.
 */
int dpu_hw_dpp_config(struct dpu_dev *d, int id, const struct decon_win_config *c,
		      u32 dpp_fmt, dma_addr_t addr)
{
	if (id < 0 || id >= DPU_NR_DPP || upper_32_bits(addr))
		return -EINVAL;

	if (!test_bit(id, &d->dpp_inited) && id != d->boot_dpp) {
		dpu_hw_dpp_init(d, id);
		set_bit(id, &d->dpp_inited);
	}

	dpp_set_scale_ratio(d, id, 1 << 20, 1 << 20);

	/* dpp_reg_set_size() */
	dma_wr(d, id, IDMA_SRC_OFFSET,
	       IDMA_SRC_OFFSET_Y(c->src.y) | IDMA_SRC_OFFSET_X(c->src.x));
	dma_wr(d, id, IDMA_SRC_SIZE,
	       IDMA_SRC_HEIGHT(c->src.f_h) | IDMA_SRC_WIDTH(c->src.f_w));
	dma_wr(d, id, IDMA_IMG_SIZE,
	       IDMA_IMG_HEIGHT(c->src.h) | IDMA_IMG_WIDTH(c->src.w));
	dpp_wr(d, id, DPP_IMG_SIZE,
	       DPP_IMG_HEIGHT(c->src.h) | DPP_IMG_WIDTH(c->src.w));
	if (id == DECON_IDMA_VGF0 || id == DECON_IDMA_VGF1)
		dpp_wr(d, id, DPP_SCALED_IMG_SIZE,
		       DPP_SCALED_IMG_HEIGHT(c->dst.h) |
		       DPP_SCALED_IMG_WIDTH(c->dst.w));

	/* dma_reg_set_rotation(id, DPP_ROT_NORMAL) */
	dma_wr_mask(d, id, IDMA_IN_CON, IDMA_ROTATION(0), IDMA_ROTATION_MASK);

	/* dpp_reg_set_buf_2p_addr(): RGB uses Y; C mirrors Y as on 4.9 */
	dma_wr(d, id, IDMA_IN_BASE_ADDR_Y, lower_32_bits(addr));
	dma_wr(d, id, IDMA_IN_BASE_ADDR_C, lower_32_bits(addr));

	/* dpp_reg_set_block_area(): no block */
	dma_wr_mask(d, id, IDMA_IN_CON, 0, IDMA_BLOCK_EN);

	if (dpp_set_format(d, id, dpp_fmt))
		return -EINVAL;

	/*
	 * 4.9 dpp_reg_set_format() tail (CONFIG_EXYNOS_AFBC=y): AFBC decoding
	 * and its recovery on the VGF channels only, cleared otherwise. An AFBC
	 * stream uses one plane and BASE_ADDR_C = BASE_ADDR_Y (set above).
	 */
	if (id == DECON_IDMA_VGF0 || id == DECON_IDMA_VGF1) {
		bool afbc = c->compression;

		dma_wr_mask(d, id, IDMA_IN_CON, afbc ? ~0U : 0, IDMA_AFBC_EN);
		dma_wr_mask(d, id, IDMA_RECOVERY_CTRL, afbc ? ~0U : 0, IDMA_RECOVERY_EN);
	} else if (c->compression) {
		return -EINVAL;		/* 4.9: AFBC decoding is VGF only */
	}
	return 0;
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

/* 4.9 helper.c dpu_translate_fmt_to_dpp(), RGB entries */
u32 dpu_translate_fmt_to_dpp(u32 format)
{
	switch (format) {
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

/* bits per pixel of the supported DPP-level formats, 0 = unsupported */
int dpu_fmt_bpp(u32 dpp_fmt)
{
	if (DPU_FMT_IS_RGB32(dpp_fmt))
		return 32;
	if (dpp_fmt == DECON_PIXEL_FORMAT_RGB_565)
		return 16;
	return 0;
}
