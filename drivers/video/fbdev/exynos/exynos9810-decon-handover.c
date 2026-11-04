// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos9810 DECON-F boot-handover helper (Galaxy Note9 "crownlte").
 *
 * SBoot initialises DSIM, the S6E3HA8 panel (MIPI command mode, TE on)
 * and DECON-F, and scans out a 1440x2960 ARGB8888 buffer at 0xcc000000
 * through one DPP IDMA channel with the DPU SysMMU in bypass (static SBoot
 * analysis: DPP IDMA_IN_BASE_ADDR_Y = 0xcc000000, IN_CON format 0, SRC and
 * IMG size 1440x2960, offset 0). simplefb draws into that buffer.
 *
 * On a command-mode panel new pixels only reach the panel GRAM when DECON
 * transfers a frame. With trig_mode = HW (4.9 DT decon_f), DECON transfers
 * one frame per TE pulse while HW_TRIG_EN is set and HW_TRIG_MASK_DECON is
 * clear; 4.9 dpu_9810 decon_reg_set_trigger() sets exactly that before a
 * frame and masks it again afterwards. If SBoot leaves the trigger masked,
 * the panel keeps showing the SBoot logo from its GRAM whatever the kernel
 * writes into the buffer.
 *
 * This driver:
 *  - checks the PMU DPU power domain status before touching any DPU
 *    register (an access to a powered-down block stalls the bus);
 *  - reads and logs what SBoot left in DECON-F, the scanning DPP channel and
 *    the DPU SysMMUs (which must be in bypass for 0xcc000000 to be a PA);
 *  - only when the DT asks for it (samsung,decon-trigger), and only when the
 *    scanned-out window is verified to be the simplefb buffer, enables the
 *    trigger with the same read-modify-write as 4.9's decon_reg_set_trigger();
 *    the original value is restored on shutdown;
 *  - optionally (samsung,display-selftest) draws a test pattern at ~108 s.
 *
 * It never touches clocks, power domains, regulators, GPIOs, DSIM, the MIPI
 * PHY, the panel, video timing, the window/DPP configuration or the SysMMU.
 */

#include <linux/console.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/font.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/timekeeping.h>
#include <linux/workqueue.h>

#define HO_PFX	"display handover: "
#define ST_PFX	"display selftest: "

/* PMU (0x14060000): DPU power domain, 4.9 flexpmu_cal_local_exynos9810.h */
#define PMU_DPU_STATUS			0x4024
#define PMU_DPU_STATUS_ON		0xf

/* DECON-F (0x16030000), 4.9 dpu_9810/regs-decon.h */
#define DECON_GLOBAL_CONTROL		0x0000
#define  GC_DECON_EN_F			BIT(0)
#define  GC_DECON_EN			BIT(1)
#define  GC_RUN_STATUS			BIT(4)
#define  GC_IDLE_STATUS			BIT(5)
#define  GC_OPMODE_CMD			BIT(8)
#define DECON_INTERRUPT_ENABLE		0x0040
#define DECON_INTERRUPT_PENDING		0x004c
#define DECON_HW_SW_TRIG_CONTROL	0x0070
#define  TRIG_HW_EN			BIT(0)
#define  TRIG_HW_MASK_DECON		BIT(4)
#define  TRIG_SW_EN			BIT(8)
#define DECON_DATA_PATH_CONTROL_0	0x0214	/* WIN_EN_F(w) = bit 4w */
#define DECON_DATA_PATH_CONTROL_1	0x0218	/* WIN_CHMAP(w) = bits 4w+2..4w */
#define DECON_DATA_PATH_CONTROL_2	0x0230
#define DECON_FRAME_COUNT		0x02a0
#define DECON_SHADOW_OFFSET		0x7000
#define DECON_MAX_WIN			6

/* DPP IDMA channels: G0 at 0x16021000, then G1, VG0, VG1, VGF0, VGF1 */
#define DPP_STRIDE			0x1000
#define DPP_NR				6
#define IDMA_ENABLE			0x0000
#define IDMA_IN_CON			0x0008
#define  IDMA_IN_FMT(v)			(((v) >> 11) & 0x1f)
#define  IDMA_IN_FMT_ARGB8888		0
#define IDMA_SRC_SIZE			0x0010	/* h [29:16], w [15:0] */
#define IDMA_SRC_OFFSET			0x0014
#define IDMA_IMG_SIZE			0x0018	/* h [28:16], w [12:0] */
#define IDMA_IN_BASE_ADDR_Y		0x0040

/* DPU SysMMUs (4.9 sysmmu_dpu0..2), non-secure SFRs, 4.9 exynos-iommu.h */
#define SYSMMU_NR			3
#define SYSMMU_CTRL			0x000	/* 0x5 enable, 0x0 disable */
#define  SYSMMU_CTRL_ENABLE		BIT(0)
#define SYSMMU_CFG			0x004
#define SYSMMU_STATUS			0x008
#define SYSMMU_PT_BASE_PPN		0x00c
#define SYSMMU_VERSION			0x034

/*
 * DECON channel-map value -> DPP index (alias dppN), the inverse of 4.9
 * helper.c dpu_dma_type_to_channel(): G0->5, G1->3, VG0->0, VG1->4,
 * VGF0->1, VGF1->2.
 */
static const u8 decon_ch_to_dpp[DPP_NR] = { 2, 4, 5, 1, 3, 0 };
static const char * const dpp_names[DPP_NR] = {
	"G0", "G1", "VG0", "VG1", "VGF0", "VGF1"
};

enum ho_trigger {
	HO_TRIG_NONE,
	HO_TRIG_HW_TE,
	HO_TRIG_SW_PERIODIC,
};

struct ho_dev {
	struct device *dev;
	void __iomem *decon;
	void __iomem *dpp;
	void __iomem *sysmmu[SYSMMU_NR];
	struct regmap *pmu;
	struct device_node *fb_np;
	phys_addr_t fb_base;
	u32 fb_w, fb_h, fb_stride;
	enum ho_trigger trig;
	u32 sw_period_ms;
	bool trig_written;
	u32 trig_orig;
	bool scanout_ok;
	bool stopping;
	int scan_dpp;
	struct delayed_work sw_work;
	struct delayed_work st_work;
	u32 st_at_ms, st_hold_s;
};

static bool ho_dpu_powered(struct ho_dev *ho)
{
	u32 st = 0;

	if (regmap_read(ho->pmu, PMU_DPU_STATUS, &st))
		return false;
	return (st & PMU_DPU_STATUS_ON) == PMU_DPU_STATUS_ON;
}

static u32 ho_rd(struct ho_dev *ho, u32 off)
{
	return readl(ho->decon + off);
}

static u32 ho_dpp_rd(struct ho_dev *ho, int dpp, u32 off)
{
	return readl(ho->dpp + dpp * DPP_STRIDE + off);
}

/*
 * Read-only survey of what SBoot left. Returns true when an enabled window
 * scans the simplefb buffer with the expected format and geometry.
 */
static bool ho_survey(struct ho_dev *ho, const char *pfx)
{
	u32 gc, trig, dp0, dp1, dp1s, fc, ie, ip;
	bool match = false;
	int w;

	if (!ho_dpu_powered(ho)) {
		pr_info("%sPMU DPU domain is off; DPU registers not read\n", pfx);
		ho->scanout_ok = false;
		return false;
	}

	gc = ho_rd(ho, DECON_GLOBAL_CONTROL);
	trig = ho_rd(ho, DECON_HW_SW_TRIG_CONTROL);
	dp0 = ho_rd(ho, DECON_DATA_PATH_CONTROL_0);
	dp1 = ho_rd(ho, DECON_DATA_PATH_CONTROL_1);
	dp1s = ho_rd(ho, DECON_SHADOW_OFFSET + DECON_DATA_PATH_CONTROL_1);
	fc = ho_rd(ho, DECON_FRAME_COUNT);
	ie = ho_rd(ho, DECON_INTERRUPT_ENABLE);
	ip = ho_rd(ho, DECON_INTERRUPT_PENDING);

	pr_info("%sDECON-F GLOBAL_CONTROL=%#010x (en=%d/%d run=%d idle=%d %s mode)\n",
		pfx, gc, !!(gc & GC_DECON_EN), !!(gc & GC_DECON_EN_F),
		!!(gc & GC_RUN_STATUS), !!(gc & GC_IDLE_STATUS),
		(gc & GC_OPMODE_CMD) ? "command" : "video");
	pr_info("%sHW_SW_TRIG_CONTROL=%#010x (hw_trig_en=%d mask=%d sw_trig=%d) FRAME_COUNT=%u\n",
		pfx, trig, !!(trig & TRIG_HW_EN), !!(trig & TRIG_HW_MASK_DECON),
		!!(trig & TRIG_SW_EN), fc);
	pr_info("%sDATA_PATH_CONTROL_0=%#010x _1=%#010x (shadow %#010x) _2=%#010x INT_EN=%#010x INT_PEND=%#010x\n",
		pfx, dp0, dp1, dp1s, ho_rd(ho, DECON_DATA_PATH_CONTROL_2),
		ie, ip);

	ho->scan_dpp = -1;
	for (w = 0; w < DECON_MAX_WIN; w++) {
		u32 ch, dpp, en, in, src, img, off, base;

		if (!(dp0 & BIT(4 * w)))
			continue;
		ch = (dp1 >> (4 * w)) & 0x7;
		if (ch >= DPP_NR) {
			pr_info("%swin%d enabled, channel %u out of range\n",
				pfx, w, ch);
			continue;
		}
		dpp = decon_ch_to_dpp[ch];
		en = ho_dpp_rd(ho, dpp, IDMA_ENABLE);
		in = ho_dpp_rd(ho, dpp, IDMA_IN_CON);
		src = ho_dpp_rd(ho, dpp, IDMA_SRC_SIZE);
		img = ho_dpp_rd(ho, dpp, IDMA_IMG_SIZE);
		off = ho_dpp_rd(ho, dpp, IDMA_SRC_OFFSET);
		base = ho_dpp_rd(ho, dpp, IDMA_IN_BASE_ADDR_Y);
		pr_info("%swin%d -> ch%u dpp%u(%s): ENABLE=%#x IN_CON=%#x (fmt %u) SRC=%ux%u IMG=%ux%u OFF=%u,%u BASE=%#010x\n",
			pfx, w, ch, dpp, dpp_names[dpp], en, in, IDMA_IN_FMT(in),
			src & 0xffff, (src >> 16) & 0x3fff,
			img & 0x1fff, (img >> 16) & 0x1fff,
			off & 0x1fff, (off >> 16) & 0x1fff, base);
		if (base == ho->fb_base &&
		    IDMA_IN_FMT(in) == IDMA_IN_FMT_ARGB8888 &&
		    (src & 0xffff) * 4 == ho->fb_stride &&
		    ((src >> 16) & 0x3fff) >= ho->fb_h &&
		    (img & 0x1fff) == ho->fb_w &&
		    ((img >> 16) & 0x1fff) == ho->fb_h && off == 0) {
			match = true;
			ho->scan_dpp = dpp;
		}
	}

	/*
	 * The scanout address is only a physical address while the DPU
	 * SysMMUs are disabled (bypass). Read-only.
	 */
	for (w = 0; w < SYSMMU_NR; w++) {
		u32 ctrl;

		if (!ho->sysmmu[w])
			continue;
		ctrl = readl(ho->sysmmu[w] + SYSMMU_CTRL);
		pr_info("%ssysmmu_dpu%d CTRL=%#x CFG=%#x STATUS=%#x PT_BASE=%#x VERSION=%#x (%s)\n",
			pfx, w, ctrl, readl(ho->sysmmu[w] + SYSMMU_CFG),
			readl(ho->sysmmu[w] + SYSMMU_STATUS),
			readl(ho->sysmmu[w] + SYSMMU_PT_BASE_PPN),
			readl(ho->sysmmu[w] + SYSMMU_VERSION),
			(ctrl & SYSMMU_CTRL_ENABLE) ? "ENABLED" : "bypass");
		if (ctrl & SYSMMU_CTRL_ENABLE)
			match = false;
	}

	ho->scanout_ok = match && (gc & GC_DECON_EN) && (gc & GC_DECON_EN_F);
	pr_info("%sscanout of %pa %ux%u stride %u: %s\n", pfx, &ho->fb_base,
		ho->fb_w, ho->fb_h, ho->fb_stride,
		ho->scanout_ok ? "verified" : "NOT verified");
	return ho->scanout_ok;
}

/*
 * 4.9 decon_reg_set_trigger(DECON_HW_TRIG, DECON_TRIG_ENABLE):
 * mask = HW_TRIG_EN | HW_TRIG_MASK_DECON, val = HW_TRIG_EN.
 */
static void ho_trigger_hw_te(struct ho_dev *ho)
{
	u32 v = ho_rd(ho, DECON_HW_SW_TRIG_CONTROL), nv;

	if ((v & (TRIG_HW_EN | TRIG_HW_MASK_DECON)) == TRIG_HW_EN) {
		pr_info(HO_PFX "TE trigger already enabled and unmasked (%#010x); no write\n",
			v);
		return;
	}
	nv = (v & ~TRIG_HW_MASK_DECON) | TRIG_HW_EN;
	ho->trig_orig = v;
	ho->trig_written = true;
	writel(nv, ho->decon + DECON_HW_SW_TRIG_CONTROL);
	pr_info(HO_PFX "WRITE DECON-F+0x70 HW_SW_TRIG_CONTROL %#010x -> %#010x (readback %#010x)\n",
		v, nv, ho_rd(ho, DECON_HW_SW_TRIG_CONTROL));
}

/*
 * Fallback, not enabled in any DT: one software-triggered frame per period,
 * 4.9 decon_reg_set_trigger(DECON_SW_TRIG, ENABLE) then DISABLE one frame
 * time later. Not TE-synchronised, so it may tear.
 */
static void ho_sw_work(struct work_struct *work)
{
	struct ho_dev *ho = container_of(to_delayed_work(work), struct ho_dev,
					 sw_work);
	u32 v;

	if (!ho_dpu_powered(ho))
		return;
	v = ho_rd(ho, DECON_HW_SW_TRIG_CONTROL) & ~(TRIG_HW_EN | TRIG_SW_EN);
	writel(v | TRIG_SW_EN, ho->decon + DECON_HW_SW_TRIG_CONTROL);
	usleep_range(17000, 20000);
	writel(v, ho->decon + DECON_HW_SW_TRIG_CONTROL);
	schedule_delayed_work(&ho->sw_work, msecs_to_jiffies(ho->sw_period_ms));
}

/* ---------------------------------------------------------------------- */
/* Self-test pattern                                                       */
/* ---------------------------------------------------------------------- */

#define C_BLACK		0xff000000
#define C_WHITE		0xffffffff
#define C_RED		0xffff0000
#define C_GREEN		0xff00ff00
#define C_BLUE		0xff0000ff
#define C_YELLOW	0xffffff00
#define C_CYAN		0xff00ffff
#define C_MAGENTA	0xffff00ff
#define C_GREY		0xff202020

struct st_canvas {
	u8 __iomem *base;
	u32 w, h, stride;
	u32 *row;
	const struct font_desc *font;
};

static void st_fill(struct st_canvas *cv, u32 x, u32 y, u32 w, u32 h, u32 c)
{
	u32 i;

	if (x >= cv->w || y >= cv->h)
		return;
	w = min(w, cv->w - x);
	h = min(h, cv->h - y);
	for (i = 0; i < w; i++)
		cv->row[i] = c;
	for (i = 0; i < h; i++)
		memcpy_toio(cv->base + (y + i) * cv->stride + x * 4, cv->row,
			    w * 4);
}

static u32 st_text(struct st_canvas *cv, u32 x, u32 y, u32 s, u32 fg,
		   const char *str)
{
	const struct font_desc *f = cv->font;
	u32 pitch, cx, gy, gx;

	if (!f)
		return x;
	pitch = DIV_ROUND_UP(f->width, 8);
	for (cx = x; *str; str++, cx += f->width * s) {
		const u8 *g = (const u8 *)f->data +
			      (u8)*str * pitch * f->height;

		for (gy = 0; gy < f->height; gy++)
			for (gx = 0; gx < f->width; gx++)
				if (g[gy * pitch + gx / 8] & (0x80 >> (gx % 8)))
					st_fill(cv, cx + gx * s, y + gy * s,
						s, s, fg);
	}
	return cx;
}

static const struct { u32 c; const char *n; } st_bars[] = {
	{ C_WHITE, "WHT" }, { C_YELLOW, "YEL" }, { C_CYAN, "CYN" },
	{ C_GREEN, "GRN" }, { C_MAGENTA, "MAG" }, { C_RED, "RED" },
	{ C_BLUE, "BLU" }, { C_BLACK, "BLK" },
};

#define ST_TICK_Y	1880
#define ST_TICK_H	140

static void st_draw(struct st_canvas *cv, phys_addr_t base)
{
	u32 bw = cv->w / ARRAY_SIZE(st_bars), i, x, y, fh;
	char buf[48];

	fh = cv->font ? cv->font->height : 32;

	st_fill(cv, 0, 0, cv->w, cv->h, C_GREY);

	/* colour bars with labels (an R/B swap shows as RED<->BLU) */
	for (i = 0; i < ARRAY_SIZE(st_bars); i++) {
		st_fill(cv, i * bw, 200, bw, 560, st_bars[i].c);
		st_fill(cv, i * bw + 12, 770, bw - 24, fh * 2 + 16, C_BLACK);
		st_text(cv, i * bw + (bw - 3 * 16 * 2) / 2, 778, 2, C_WHITE,
			st_bars[i].n);
	}
	/* grey ramp 0..255 */
	for (x = 0; x < cv->w; x++) {
		u32 l = x * 256 / cv->w;

		st_fill(cv, x, 880, 1, 160, 0xff000000 | l << 16 | l << 8 | l);
	}

	st_text(cv, 80, 1100, 5, C_WHITE, "DISPLAY SELFTEST");
	st_text(cv, 80, 1300, 3, C_CYAN, "LINUX 6.12 CROWNLTE");
	snprintf(buf, sizeof(buf), "SIMPLEFB %pa", &base);
	st_text(cv, 80, 1420, 3, C_CYAN, buf);
	snprintf(buf, sizeof(buf), "%ux%u STRIDE %u", cv->w, cv->h,
		 cv->stride);
	st_text(cv, 80, 1540, 3, C_CYAN, buf);
	st_text(cv, 80, 1700, 2, C_YELLOW, "TIME BELOW MUST COUNT UP");

	/* 16 px checkerboard */
	for (y = 2200; y < 2600; y += 16)
		for (x = 40; x < cv->w - 40; x += 16)
			st_fill(cv, x, y, 16, 16,
				((x / 16) ^ (y / 16)) & 1 ? C_WHITE : C_BLACK);
	/* diagonal, 3 px wide: a wrong stride shears it */
	for (y = 2640; y < cv->h - 160; y++) {
		x = (y - 2640) * (cv->w - 1) / (cv->h - 160 - 2640);
		st_fill(cv, x > 1 ? x - 1 : 0, y, 3, 1, C_MAGENTA);
	}
	/* 8-pixel white frame on top: shows cropping or offset of the scanout */
	st_fill(cv, 0, 0, cv->w, 8, C_WHITE);
	st_fill(cv, 0, cv->h - 8, cv->w, 8, C_WHITE);
	st_fill(cv, 0, 0, 8, cv->h, C_WHITE);
	st_fill(cv, cv->w - 8, 0, 8, cv->h, C_WHITE);
	/* corner markers: TL red, TR green, BL blue, BR yellow */
	st_fill(cv, 16, 16, 120, 120, C_RED);
	st_fill(cv, cv->w - 136, 16, 120, 120, C_GREEN);
	st_fill(cv, 16, cv->h - 136, 120, 120, C_BLUE);
	st_fill(cv, cv->w - 136, cv->h - 136, 120, 120, C_YELLOW);
	wmb();
}

static void st_tick(struct st_canvas *cv, u64 ms, u32 fc, bool have_fc,
		    unsigned int n)
{
	char buf[48];

	st_fill(cv, 40, ST_TICK_Y, cv->w - 80, ST_TICK_H, C_BLACK);
	if (have_fc)
		snprintf(buf, sizeof(buf), "T+%llu.%03lluS F%u", ms / 1000,
			 ms % 1000, fc);
	else
		snprintf(buf, sizeof(buf), "T+%llu.%03lluS", ms / 1000,
			 ms % 1000);
	st_text(cv, 60, ST_TICK_Y + 6, 4, C_WHITE, buf);
	/* heartbeat square alternates green/red every tick */
	st_fill(cv, cv->w / 2 - 60, 2040, 120, 120, (n & 1) ? C_RED : C_GREEN);
	wmb();
}

static struct fb_info *st_find_fb(struct ho_dev *ho, struct platform_device **out)
{
	struct platform_device *pdev = of_find_device_by_node(ho->fb_np);
	struct fb_info *info;

	*out = NULL;
	if (!pdev)
		return NULL;
	device_lock(&pdev->dev);
	info = (pdev->dev.driver &&
		!strcmp(pdev->dev.driver->name, "simple-framebuffer")) ?
		platform_get_drvdata(pdev) : NULL;
	device_unlock(&pdev->dev);
	if (!info) {
		put_device(&pdev->dev);
		return NULL;
	}
	*out = pdev;
	return info;
}

static void ho_st_work(struct work_struct *work)
{
	struct ho_dev *ho = container_of(to_delayed_work(work), struct ho_dev,
					 st_work);
	struct platform_device *fbpdev;
	struct st_canvas cv = { };
	struct fb_info *info;
	u32 fc0 = 0, fc = 0, prev = 0, adv = 0, rb_ok = 0, rb_n = 0;
	bool have_fc, pass;
	unsigned int i;
	u64 ms;

	ms = ktime_to_ms(ktime_get_boottime());
	pr_info(ST_PFX "start at %llu.%03llu s\n", ms / 1000, ms % 1000);
	ho_survey(ho, ST_PFX);
	have_fc = ho_dpu_powered(ho);
	if (have_fc)
		fc0 = prev = ho_rd(ho, DECON_FRAME_COUNT);

	info = st_find_fb(ho, &fbpdev);
	if (!info) {
		pr_err(ST_PFX "FAIL: simplefb %pOF not bound, nothing drawn\n",
		       ho->fb_np);
		return;
	}
	pr_info(ST_PFX "fb%d \"%s\" smem %#lx+%#x %ux%u bpp %u line %u\n",
		info->node, info->fix.id, info->fix.smem_start,
		info->fix.smem_len, info->var.xres, info->var.yres,
		info->var.bits_per_pixel, info->fix.line_length);
	if (info->fix.smem_start != ho->fb_base || info->var.xres != ho->fb_w ||
	    info->var.yres != ho->fb_h || info->var.bits_per_pixel != 32 ||
	    info->fix.line_length != ho->fb_stride || !info->screen_base) {
		pr_err(ST_PFX "FAIL: fb geometry differs from DT, nothing drawn\n");
		goto out_put;
	}

	cv.base = (u8 __iomem *)info->screen_base;
	cv.w = ho->fb_w;
	cv.h = ho->fb_h;
	cv.stride = ho->fb_stride;
	cv.font = find_font("TER16x32");
	if (!cv.font)
		cv.font = find_font("VGA8x16");
	cv.row = kmalloc_array(cv.w, sizeof(u32), GFP_KERNEL);
	if (!cv.row)
		goto out_put;
	pr_info(ST_PFX "font %s\n", cv.font ? cv.font->name : "none (no text)");

	/* stop fbcon drawing while the pattern is up */
	console_lock();
	fb_set_suspend(info, 1);
	console_unlock();

	st_draw(&cv, ho->fb_base);
	{
		static const struct { u32 x, y, c; } chk[] = {
			{ 0, 0, C_WHITE }, { 20, 20, C_RED },
			{ 1420, 20, C_GREEN }, { 20, 2940, C_BLUE },
			{ 1420, 2940, C_YELLOW }, { 90, 400, C_WHITE },
			{ 990, 400, C_RED }, { 1170, 400, C_BLUE },
		};

		for (i = 0; i < ARRAY_SIZE(chk); i++, rb_n++) {
			u32 v = readl(cv.base + chk[i].y * cv.stride +
				      chk[i].x * 4);

			if (v == chk[i].c)
				rb_ok++;
			else
				pr_err(ST_PFX "readback (%u,%u) %#010x != %#010x\n",
				       chk[i].x, chk[i].y, v, chk[i].c);
		}
	}
	ms = ktime_to_ms(ktime_get_boottime());
	pr_info(ST_PFX "pattern drawn at %llu.%03llu s: grey bg, white frame, corners TL red/TR green/BL blue/BR yellow, 8 bars WHT YEL CYN GRN MAG RED BLU BLK, ramp, 'DISPLAY SELFTEST', ticking clock; readback %u/%u\n",
		ms / 1000, ms % 1000, rb_ok, rb_n);

	for (i = 0; i <= ho->st_hold_s; i++) {
		ms = ktime_to_ms(ktime_get_boottime());
		if (have_fc && ho_dpu_powered(ho)) {
			fc = ho_rd(ho, DECON_FRAME_COUNT);
			if (i && fc != prev)
				adv++;
			prev = fc;
		}
		st_tick(&cv, ms, fc, have_fc, i);
		pr_info(ST_PFX "tick %u t=%llu.%03llu s FRAME_COUNT=%u\n",
			i, ms / 1000, ms % 1000, fc);
		if (READ_ONCE(ho->stopping))
			break;
		if (i < ho->st_hold_s)
			msleep(1000);
	}

	console_lock();
	fb_set_suspend(info, 0);
	console_unlock();

	pass = rb_ok == rb_n && (!have_fc || adv > 0);
	pr_info(ST_PFX "%s: readback %u/%u, FRAME_COUNT %u -> %u (advanced in %u/%u ticks)%s; trigger %s; fbcon resumed\n",
		pass ? "PASS" : "FAIL", rb_ok, rb_n, fc0, fc, adv,
		ho->st_hold_s, have_fc ? "" : " [DPU off: not read]",
		ho->trig_written ? "written by kernel" :
		ho->trig == HO_TRIG_NONE ? "untouched (read-only DT)" :
		"untouched");
	pr_info(ST_PFX "a person must confirm the panel showed the pattern and that the clock counted up\n");
	kfree(cv.row);
out_put:
	put_device(&fbpdev->dev);
}

/* ---------------------------------------------------------------------- */

static int ho_parse_fb(struct ho_dev *ho)
{
	struct device_node *np = ho->dev->of_node;
	struct resource res;
	int ret;

	ho->fb_np = of_parse_phandle(np, "samsung,simplefb", 0);
	if (!ho->fb_np)
		return dev_err_probe(ho->dev, -EINVAL, "no samsung,simplefb\n");
	if (!of_device_is_compatible(ho->fb_np, "simple-framebuffer"))
		return dev_err_probe(ho->dev, -EINVAL,
				     "%pOF is not a simple-framebuffer\n",
				     ho->fb_np);
	ret = of_address_to_resource(ho->fb_np, 0, &res);
	if (ret)
		return dev_err_probe(ho->dev, ret, "simplefb reg\n");
	ho->fb_base = res.start;
	if (of_property_read_u32(ho->fb_np, "width", &ho->fb_w) ||
	    of_property_read_u32(ho->fb_np, "height", &ho->fb_h) ||
	    of_property_read_u32(ho->fb_np, "stride", &ho->fb_stride))
		return dev_err_probe(ho->dev, -EINVAL, "simplefb geometry\n");
	if (resource_size(&res) < (resource_size_t)ho->fb_stride * ho->fb_h ||
	    ho->fb_stride < ho->fb_w * 4)
		return dev_err_probe(ho->dev, -EINVAL, "simplefb size\n");
	return 0;
}

static int ho_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct ho_dev *ho;
	const char *mode = "none";
	int ret, i;

	ho = devm_kzalloc(dev, sizeof(*ho), GFP_KERNEL);
	if (!ho)
		return -ENOMEM;
	ho->dev = dev;
	ho->scan_dpp = -1;
	INIT_DELAYED_WORK(&ho->sw_work, ho_sw_work);
	INIT_DELAYED_WORK(&ho->st_work, ho_st_work);
	platform_set_drvdata(pdev, ho);

	ret = ho_parse_fb(ho);
	if (ret)
		goto err_put;

	ho->pmu = syscon_regmap_lookup_by_phandle(np, "samsung,pmu-syscon");
	if (IS_ERR(ho->pmu)) {
		ret = dev_err_probe(dev, PTR_ERR(ho->pmu), "pmu syscon\n");
		goto err_put;
	}
	ho->decon = devm_platform_ioremap_resource_byname(pdev, "decon");
	if (IS_ERR(ho->decon)) {
		ret = PTR_ERR(ho->decon);
		goto err_put;
	}
	ho->dpp = devm_platform_ioremap_resource_byname(pdev, "dpp");
	if (IS_ERR(ho->dpp)) {
		ret = PTR_ERR(ho->dpp);
		goto err_put;
	}
	for (i = 0; i < SYSMMU_NR; i++) {
		char name[8];
		struct resource *r;

		snprintf(name, sizeof(name), "sysmmu%d", i);
		r = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
		if (!r)
			continue;
		ho->sysmmu[i] = devm_ioremap_resource(dev, r);
		if (IS_ERR(ho->sysmmu[i])) {
			ret = PTR_ERR(ho->sysmmu[i]);
			goto err_put;
		}
	}

	of_property_read_string(np, "samsung,decon-trigger", &mode);
	if (!strcmp(mode, "hw-te"))
		ho->trig = HO_TRIG_HW_TE;
	else if (!strcmp(mode, "sw-periodic"))
		ho->trig = HO_TRIG_SW_PERIODIC;
	else
		ho->trig = HO_TRIG_NONE;
	ho->sw_period_ms = 100;
	of_property_read_u32(np, "samsung,decon-sw-trigger-ms",
			     &ho->sw_period_ms);
	ho->sw_period_ms = clamp(ho->sw_period_ms, 17U, 1000U);

	pr_info(HO_PFX "fb %pa %ux%u stride %u, trigger mode \"%s\"\n",
		&ho->fb_base, ho->fb_w, ho->fb_h, ho->fb_stride, mode);
	ho_survey(ho, HO_PFX);

	if (ho->trig != HO_TRIG_NONE) {
		if (!ho->scanout_ok)
			pr_warn(HO_PFX "refusing trigger write: DPU off, a DPU SysMMU enabled, or SBoot scanout of the simplefb buffer not verified\n");
		else if (!(ho_rd(ho, DECON_GLOBAL_CONTROL) & GC_OPMODE_CMD))
			pr_info(HO_PFX "video mode: no trigger needed, no write\n");
		else if (ho->trig == HO_TRIG_HW_TE)
			ho_trigger_hw_te(ho);
		else {
			ho->trig_orig = ho_rd(ho, DECON_HW_SW_TRIG_CONTROL);
			ho->trig_written = true;
			pr_warn(HO_PFX "UNTESTED sw-periodic trigger every %u ms\n",
				ho->sw_period_ms);
			schedule_delayed_work(&ho->sw_work, 0);
		}
	}

	if (ho->trig_written && ho_dpu_powered(ho)) {
		u32 f0 = ho_rd(ho, DECON_FRAME_COUNT), f1;

		msleep(100);
		f1 = ho_rd(ho, DECON_FRAME_COUNT);
		pr_info(HO_PFX "FRAME_COUNT %u -> %u over 100 ms after trigger change\n",
			f0, f1);
	}

	if (of_property_read_bool(np, "samsung,display-selftest")) {
		u64 now = ktime_to_ms(ktime_get_boottime());

		ho->st_at_ms = 108000;
		ho->st_hold_s = 10;
		of_property_read_u32(np, "samsung,display-selftest-at-ms",
				     &ho->st_at_ms);
		of_property_read_u32(np, "samsung,display-selftest-hold-s",
				     &ho->st_hold_s);
		ho->st_hold_s = min(ho->st_hold_s, 60U);
		pr_info(ST_PFX "scheduled at %u ms uptime, pattern held %u s\n",
			ho->st_at_ms, ho->st_hold_s);
		queue_delayed_work(system_long_wq, &ho->st_work,
				   msecs_to_jiffies(ho->st_at_ms > now ?
						    ho->st_at_ms - now : 0));
	}
	return 0;

err_put:
	of_node_put(ho->fb_np);
	return ret;
}

static void ho_restore(struct ho_dev *ho)
{
	cancel_delayed_work_sync(&ho->sw_work);
	if (ho->trig_written && ho_dpu_powered(ho)) {
		writel(ho->trig_orig, ho->decon + DECON_HW_SW_TRIG_CONTROL);
		ho->trig_written = false;
		pr_info(HO_PFX "restored HW_SW_TRIG_CONTROL=%#010x\n",
			ho->trig_orig);
	}
}

static void ho_shutdown(struct platform_device *pdev)
{
	struct ho_dev *ho = platform_get_drvdata(pdev);

	WRITE_ONCE(ho->stopping, true);
	cancel_delayed_work_sync(&ho->st_work);
	ho_restore(ho);
}

static void ho_remove(struct platform_device *pdev)
{
	struct ho_dev *ho = platform_get_drvdata(pdev);

	WRITE_ONCE(ho->stopping, true);
	cancel_delayed_work_sync(&ho->st_work);
	ho_restore(ho);
	of_node_put(ho->fb_np);
}

static const struct of_device_id ho_of_match[] = {
	{ .compatible = "samsung,exynos9810-decon-handover" },
	{ }
};
MODULE_DEVICE_TABLE(of, ho_of_match);

static struct platform_driver ho_driver = {
	.probe = ho_probe,
	.remove = ho_remove,
	.shutdown = ho_shutdown,
	.driver = {
		.name = "exynos9810-decon-handover",
		.of_match_table = ho_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(ho_driver);

MODULE_DESCRIPTION("Exynos9810 DECON-F SBoot scanout handover helper");
MODULE_LICENSE("GPL");
