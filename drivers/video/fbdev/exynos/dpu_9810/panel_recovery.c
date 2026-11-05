// SPDX-License-Identifier: GPL-2.0
/*
 * Galaxy Note9 (crownlte) S6E3HA8 "crown_a3_s0": one 4.9 screen off /
 * screen on panel command cycle for a panel SBoot left dark, opt-in via DT
 * (owner approval 2026-11-05 for levels 1 and 2; no PMIC or regulator
 * access at any level).
 *
 * Fault seen since 2026-11-05 02:48: the panel stays black from SBoot's
 * logo on, while the DDI reports normal display (RDDPM 0x9f), TE on, OTP
 * loaded, and under the level 2 key ERR_FG EE = 0x08 (ELVDD error). On the
 * owner's 4.9 Lineage a screen off / on relights the panel. This file
 * sends the panel commands of that cycle, once, in the owner's 4.9 build
 * configuration (SUPPORT_DSU, SUPPORT_AFC, EXYNOS_DECON_LCD_COPR,
 * DISPLAY_USE_INFO = y; OLD_DISP_TIMING, ACTIVE_CLOCK, SUPPORT_AOD_BL = n):
 *
 *   panel_sleep_in()  = display off table + PANEL_EXIT_SEQ
 *   level 2 only: the panel reset pulse of __set_panel_power()
 *   panel_sleep_out() = PANEL_INIT_SEQ (with PANEL_SET_BL_SEQ and
 *                       COPR_SET_SEQ inside it)
 *   first frame done  -> panel_display_on() = display on table
 *
 * Packet list, bytes and 4.9 sources: Documentation/crownlte/
 * panel-recovery.md. Not sent (no panel commands in the 4.9 screen-off
 * path, or not part of "lighting the panel"): the S2DOS05 rails and its
 * short-detect current (level 3, not approved), the DSIM PHY / link
 * power-down and re-init (adopt mode keeps DSIM and DECON running), the
 * mDNIe tuning tables mdnie_enable() re-sends after display on, and the
 * COPR counter clear copr_enable() may send.
 *
 * The DSIM link stays in command mode with the DECON TE trigger masked
 * (dpu_cmd_window_begin()) while panel commands go out. 4.9 sends the
 * display on only after the first frame done; so does this file: window 1
 * (checks, off, reset, init) is closed, frames run until FRAME_COUNT moves,
 * then window 2 sends the display on table.
 *
 * Preconditions, all read only, checked before the first write (a refusal
 * logs why and sends nothing but the level 2 key around the reads):
 * DSIM command mode and idle (dpu_dsim_check()); DECON data path
 * DSCC + DSC ENC0/1 + DSIM IF0 with both encoders' PPS registers matching
 * the format fields of the PPS sent here; the panel_bright.c checks
 * (normal display RDDPM, panel id 04 = DT samsung,panel-id, HBM gamma vs
 * DT, F4); with "samsung,panel-recovery-only-on-elvdd-error" the keyed EE
 * must show bit 3; at level 2 the reset GPIO must be gpa3-4 of
 * pinctrl@14050000, an output driven high.
 *
 * Every key on is followed by its key off, also after errors: each table
 * tracks the levels it turned on and sends the outstanding key offs at its
 * end. Within a table every packet is tried (4.9 would retry the whole
 * power cycle instead; there is no retry here), and every later phase
 * still runs, so the panel ends as far "on" as the link allows.
 */
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <video/mipi_display.h>

#include "panel_bright.h"

#define DPU_PR_PFX		"panel-recovery: "
#define prc_dbg(fmt, ...)	printk(KERN_DEBUG DPU_PR_PFX fmt, ##__VA_ARGS__)

#define PR_PPS_LEN		128
#define PR_FRAME_WAIT_MS	500	/* first frame after the init table */
#define PR_SETTLE_MS		100	/* display on -> status read */

/* level 2 reset pin: 4.9 DT gpio,lcd-reset = <gpa3 4>, ALIVE pin controller */
#define PR_ALIVE_BASE		0x14050000	/* pinctrl@14050000 */
#define PR_GPA3_CON		0x080		/* exynos9810_pin_banks0 "gpa3" */
#define PR_GPA3_DAT		0x084
#define PR_RESET_PIN		4
#define PR_RESET_LOW_US		10000	/* __set_panel_power(): rails on, 10 ms, reset high */
#define PR_RESET_HIGH_US	5000	/* __set_panel_power(): reset high, 5 ms */

enum pr_kind {
	PR_WR,		/* DCS write (4.9 mipi_write(), "B0 <ofs>" first if ofs) */
	PR_KEY_ON,	/* DCS write of a key on, level in .slot */
	PR_KEY_OFF,	/* DCS write of a key off, level in .slot */
	PR_COMP,	/* DSC compression mode, MIPI_DSI_DSC_PRA 0x07 */
	PR_PPS,		/* DSC PPS, MIPI_DSI_DSC_PPS 0x0A, 128 bytes */
	PR_RD,		/* 4.9 panel dump read of .len bytes of .reg into slot */
	PR_DELAY,	/* 4.9 panel_do_delay(): usleep_range(us, us + 10) */
};

struct pr_step {
	u8 kind, slot, ofs, reg, len;
	const u8 *data;
	u32 us;
	const char *what;
};

/* reads of the 4.9 tables (DISPLAY_USE_INFO dumps), kept for the log */
enum { PR_RD_EXIT_EE, PR_RD_EXIT_DSIE, PR_RD_INIT_DIAG, PR_NRD };

#define PR_W(_d, _what)		{ .kind = PR_WR, .len = sizeof(_d), .data = _d, .what = _what }
#define PR_WO(_o, _d, _what)	{ .kind = PR_WR, .ofs = _o, .len = sizeof(_d), .data = _d, \
				  .what = _what }
#define PR_KON(_l, _d)		{ .kind = PR_KEY_ON, .slot = _l, .len = 3, .data = _d, \
				  .what = "level " #_l " key on" }
#define PR_KOFF(_l, _d)		{ .kind = PR_KEY_OFF, .slot = _l, .len = 3, .data = _d, \
				  .what = "level " #_l " key off" }
#define PR_DLY(_us, _what)	{ .kind = PR_DELAY, .us = _us, .what = _what }
#define PR_DUMP(_r, _s, _what)	{ .kind = PR_RD, .reg = _r, .slot = _s, .len = 1, .what = _what }

/* s6e3ha8_crown_a3_s0_panel.h:648-657 */
static const u8 pr_key1_on[] = { 0x9F, 0xA5, 0xA5 };
static const u8 pr_key1_off[] = { 0x9F, 0x5A, 0x5A };
static const u8 pr_key3_on[] = { 0xFC, 0x5A, 0x5A };
static const u8 pr_key3_off[] = { 0xFC, 0xA5, 0xA5 };
static const u8 pr_sleep_out[] = { MIPI_DCS_EXIT_SLEEP_MODE };
static const u8 pr_sleep_in[] = { MIPI_DCS_ENTER_SLEEP_MODE };
static const u8 pr_display_off[] = { MIPI_DCS_SET_DISPLAY_OFF };
static const u8 pr_display_on[] = { MIPI_DCS_SET_DISPLAY_ON };
/* s6e3ha8_crown_a3_s0_panel_mdnie.h:2056, S6E3HA8_AFC_REG = 0xE2 (s6e3ha8.h:274) */
static const u8 pr_afc_off[] = { 0xE2, 0x00 };
/* crown_a3_s0_dsc_table[1] (panel.h:416-419): getidx_dsc_table(), dsc_en = 1 */
static const u8 pr_dsc[] = { 0x01 };
/*
 * crown_a3_s0_pps_table[0] (panel.h:421-440), "DSU MODE 1: 1440x2960
 * (Original) Slice Info: 720x40": getidx_resolution_table() row 0 for
 * mres_mode DSU_MODE_1. (The static CROWN_A3_S0_PPS array, panel.h:682,
 * holds the 1080x2220 row and is overwritten by the maptbl.)
 */
static const u8 pr_pps[PR_PPS_LEN] = {
	0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x0B, 0x90,
	0x05, 0xA0, 0x00, 0x28, 0x02, 0xD0, 0x02, 0xD0,
	0x02, 0x00, 0x02, 0x68, 0x00, 0x20, 0x04, 0x6C,
	0x00, 0x0A, 0x00, 0x0C, 0x02, 0x77, 0x01, 0xE9,
	0x18, 0x00, 0x10, 0xF0, 0x03, 0x0C, 0x20, 0x00,
	0x06, 0x0B, 0x0B, 0x33, 0x0E, 0x1C, 0x2A, 0x38,
	0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B,
	0x7D, 0x7E, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
	0x09, 0xBE, 0x19, 0xFC, 0x19, 0xFA, 0x19, 0xF8,
	0x1A, 0x38, 0x1A, 0x78, 0x1A, 0xB6, 0x2A, 0xF6,
	0x2B, 0x34, 0x2B, 0x74, 0x3B, 0x74, 0x6B, 0xF4,
	/* bytes 88..127 zero */
};
/* crown_a3_s0_scaler/caset/paset_table[0] (panel.h:481-506), DSU MODE 1 */
static const u8 pr_scaler[] = { 0xBA, 0x01, 0x26, 0x08, 0x08, 0xF3 };
static const u8 pr_caset[] = { MIPI_DCS_SET_COLUMN_ADDRESS, 0x00, 0x00, 0x05, 0x9F };
static const u8 pr_paset[] = { MIPI_DCS_SET_PAGE_ADDRESS, 0x00, 0x00, 0x0B, 0x8F };
/* panel.h:702-704, 745-756, 827-829 */
static const u8 pr_set_area[] = { 0x1A, 0x1F, 0x00, 0x00, 0x00, 0x00 };
static const u8 pr_te_on[] = { MIPI_DCS_SET_TEAR_ON, 0x00 };
static const u8 pr_err_fg[] = { 0xED, 0x4C };
static const u8 pr_ffc[] = { 0xC5, 0x0D, 0x10, 0xB4, 0x3E, 0x01 };
static const u8 pr_tsp_hsync[] = { 0xB9, 0x01, 0xB0, 0x91, 0x09, 0x00, 0x00, 0x00, 0x11, 0x03 };
static const u8 pr_isc[] = { 0xF6, 0x43 };		/* packet offset 3, panel.h:1125 */
static const u8 pr_edge_dimming[] = { 0xC2, 0x09, 0x00, 0xD8, 0xD8, 0xF4, 0xF4 };
/*
 * COPR_SET_SEQ (s6e3ha8_crown_a3_s0_panel_copr.h:42-63): E1 + copy_copr_maptbl()
 * (s6e3ha8.c:1519) of the crown_a3_s0 defaults (panel_copr.h:98-122):
 * ilc, gamma 2.2, enable; E{R,G,B}{,C} 0x100; max count 0xFFFF; no ROI.
 */
static const u8 pr_copr[] = { 0xE1, 0x07, 0x15, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			      0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* crown_a3_s0_display_off_cmdtbl, panel.h:1359 */
static const struct pr_step pr_tbl_display_off[] = {
	PR_KON(1, pr_key1_on),
	PR_W(pr_display_off, "DISPLAY_OFF 28"),
	PR_KOFF(1, pr_key1_off),
};

/* crown_a3_s0_exit_cmdtbl, panel.h:1365-1380 (SUPPORT_AFC, DISPLAY_USE_INFO) */
static const struct pr_step pr_tbl_exit[] = {
	PR_KON(2, pb_key2_on),				/* MDNIE_AFC_OFF_SEQ */
	PR_W(pr_afc_off, "AFC off E2 00"),
	PR_KOFF(2, pb_key2_off),
	PR_DLY(20000, "wait_afc_off 20 ms"),
	PR_KON(1, pr_key1_on),
	PR_KON(2, pb_key2_on),
	PR_DUMP(0xEE, PR_RD_EXIT_EE, "dump ERR_FG EE"),
	PR_KOFF(2, pb_key2_off),
	PR_DUMP(MIPI_DCS_GET_ERROR_COUNT_ON_DSI, PR_RD_EXIT_DSIE, "dump DSI_ERR 05"),
	PR_W(pr_sleep_in, "SLEEP_IN 10"),
	PR_KOFF(1, pr_key1_off),
	PR_DLY(100000, "wait_sleep_in 100 ms"),
};

/* crown_a3_s0_init_cmdtbl, panel.h:1224-1260, up to PANEL_SET_BL_SEQ */
static const struct pr_step pr_tbl_init[] = {
	PR_DLY(5000, "wait_5msec"),
	PR_KON(1, pr_key1_on),
	{ .kind = PR_COMP, .len = sizeof(pr_dsc), .data = pr_dsc,
	  .what = "DSC compression mode (0x07) 01" },
	{ .kind = PR_PPS, .len = sizeof(pr_pps), .data = pr_pps,
	  .what = "DSC PPS (0x0A) 1440x2960 slice 720x40" },
	PR_KOFF(1, pr_key1_off),
	PR_W(pr_sleep_out, "SLEEP_OUT 11"),
	PR_DLY(10000, "wait_sleep_out 10 ms"),
	PR_DUMP(MIPI_DCS_GET_DIAGNOSTIC_RESULT, PR_RD_INIT_DIAG, "dump SELF_DIAG 0F"),
	PR_KON(2, pb_key2_on),				/* SUPPORT_DSU */
	PR_W(pr_scaler, "SCALER BA (1440x2960)"),
	PR_KOFF(2, pb_key2_off),
	PR_W(pr_caset, "CASET 2A 0..1439"),
	PR_W(pr_paset, "PASET 2B 0..2959"),
	PR_KON(1, pr_key1_on),
	PR_W(pr_set_area, "SET_AREA 1A"),
	PR_W(pr_te_on, "TE_ON 35 00"),
	PR_KOFF(1, pr_key1_off),
	PR_KON(2, pb_key2_on),
	PR_W(pr_err_fg, "ERR_FG ED 4C"),
	PR_KON(3, pr_key3_on),
	PR_W(pr_ffc, "FFC C5"),
	PR_KOFF(3, pr_key3_off),
	PR_W(pr_tsp_hsync, "TSP_HSYNC B9"),
	PR_WO(3, pr_isc, "ISC F6 43 (B0 03)"),
	PR_W(pr_edge_dimming, "EDGE_DIMMING C2"),
	PR_KOFF(2, pb_key2_off),
	/* PANEL_SET_BL_SEQ: panel_bright.c pb_send_bl() */
};

/* crown_a3_s0_set_copr_cmdtbl (PKTINFO keys), the init table's last entry */
static const struct pr_step pr_tbl_copr[] = {
	PR_KON(2, pb_key2_on),
	PR_W(pr_copr, "COPR E1"),
	PR_KOFF(2, pb_key2_off),
};

/* crown_a3_s0_display_on_cmdtbl, panel.h:1353 */
static const struct pr_step pr_tbl_display_on[] = {
	PR_KON(1, pr_key1_on),
	PR_W(pr_display_on, "DISPLAY_ON 29"),
	PR_KOFF(1, pr_key1_off),
};

static const struct {
	const struct pr_step *st;
	int n;
} pr_tables[] = {
	{ pr_tbl_display_off, ARRAY_SIZE(pr_tbl_display_off) },
	{ pr_tbl_exit, ARRAY_SIZE(pr_tbl_exit) },
	{ pr_tbl_init, ARRAY_SIZE(pr_tbl_init) },
	{ pr_tbl_copr, ARRAY_SIZE(pr_tbl_copr) },
	{ pr_tbl_display_on, ARRAY_SIZE(pr_tbl_display_on) },
};

/*
 * The per-packet allowlist: a write must equal, in kind, offset, length
 * and every byte, one packet of the tables above. Open only while the
 * recovery runs (d->pr_active also opens dsim_cmd.c's gate).
 */
static bool pr_allowed(struct dpu_dev *d, u8 kind, u8 ofs, const u8 *buf, size_t len)
{
	int t, i;

	if (!d->pr_active)
		return false;
	if (kind == PR_KEY_ON || kind == PR_KEY_OFF)
		kind = PR_WR;
	for (t = 0; t < ARRAY_SIZE(pr_tables); t++) {
		for (i = 0; i < pr_tables[t].n; i++) {
			const struct pr_step *s = &pr_tables[t].st[i];
			u8 k = s->kind;

			if (k == PR_KEY_ON || k == PR_KEY_OFF)
				k = PR_WR;
			if (k == kind && s->ofs == ofs && s->len == len && !memcmp(s->data, buf, len))
				return true;
		}
	}
	return false;
}

static int pr_tx(struct dpu_dev *d, const struct pr_step *st)
{
	int ret;

	if (!pr_allowed(d, st->kind, st->ofs, st->data, st->len)) {
		pr_err(DPU_PR_PFX "write %*ph (ofs %#x, kind %u) not in the 4.9 tables or recovery not active, refused\n",
		       (int)min_t(size_t, st->len, 8), st->data, st->ofs, st->kind);
		return -EPERM;
	}
	switch (st->kind) {
	case PR_COMP:
		return dpu_dsim_tx(d, MIPI_DSI_COMPRESSION_MODE, st->data, st->len);
	case PR_PPS:
		return dpu_dsim_tx(d, MIPI_DSI_PICTURE_PARAMETER_SET, st->data, st->len);
	default:
		if (st->ofs) {
			u8 gpara[2] = { 0xB0, st->ofs };

			ret = dpu_dsim_tx(d, MIPI_DSI_DCS_SHORT_WRITE_PARAM, gpara, 2);
			if (ret)
				return ret;
		}
		return dpu_dsim_tx(d, st->len == 1 ? MIPI_DSI_DCS_SHORT_WRITE :
				   MIPI_DSI_DCS_LONG_WRITE, st->data, st->len);
	}
}

struct pr_result {
	int err;			/* first error of any phase */
	u32 sent, failed;		/* packets attempted / failed */
	unsigned long keys;		/* key levels owed a key off */
	u8 rd[PR_NRD];
	int rd_ret[PR_NRD];
	u32 frames, frame_wait_ms;	/* between init and display on */
	int ret_bl;			/* PANEL_SET_BL_SEQ */
	int reset;			/* level 2 pulse: 0 ok, <0 error, 1 not run */
};

static void pr_note_err(struct pr_result *r, int ret)
{
	if (ret && !r->err)
		r->err = ret;
}

/* the key offs a table still owes, highest level first */
static void pr_keys_off(struct dpu_dev *d, struct pr_result *r, const char *tbl)
{
	static const struct pr_step off[] = {
		PR_KOFF(1, pr_key1_off), PR_KOFF(2, pb_key2_off), PR_KOFF(3, pr_key3_off),
	};
	int l;

	for (l = 3; l >= 1; l--) {
		int ret;

		if (!test_bit(l, &r->keys))
			continue;
		r->sent++;
		ret = pr_tx(d, &off[l - 1]);
		if (ret) {
			r->failed++;
			pr_err(DPU_PR_PFX "%s: level %d key off after an error: %d\n", tbl, l, ret);
			pr_note_err(r, ret);
			continue;
		}
		clear_bit(l, &r->keys);
	}
}

/* one 4.9 table, every packet tried; outstanding key offs at the end */
static void pr_run(struct dpu_dev *d, const char *tbl, const struct pr_step *st, int n,
		   struct pr_result *r)
{
	int i, ret;

	for (i = 0; i < n; i++) {
		const struct pr_step *s = &st[i];

		prc_dbg("%s %d/%d: %s\n", tbl, i + 1, n, s->what);
		switch (s->kind) {
		case PR_DELAY:
			usleep_range(s->us, s->us + 10);
			continue;
		case PR_RD:
			r->rd_ret[s->slot] = pb_read(d, s->reg, 0, &r->rd[s->slot], s->len);
			continue;	/* a dump read failing stops nothing (4.9 too) */
		case PR_KEY_ON:
			set_bit(s->slot, &r->keys);	/* owed from the attempt on */
			break;
		default:
			break;
		}
		r->sent++;
		ret = pr_tx(d, s);
		if (ret) {
			r->failed++;
			pr_err(DPU_PR_PFX "%s %d/%d (%s): %d\n", tbl, i + 1, n, s->what, ret);
			pr_note_err(r, ret);
			continue;
		}
		if (s->kind == PR_KEY_OFF)
			clear_bit(s->slot, &r->keys);
	}
	if (r->keys)
		pr_keys_off(d, r, tbl);
}

/* ---- preconditions, read only ------------------------------------------ */

static u16 pr_be16(const u8 *b)
{
	return (b[0] << 8) | b[1];
}

/*
 * The DECON DSC encoders must produce the stream the PPS sent here
 * describes. 4.9 (decon_reg.c dsc_calc_pps_info() / dsc_reg_set_pps())
 * programs each of the two encoders (DSC combiner on) with half the
 * picture width and its own rate-control values, so the PPS registers
 * (DSC_PPS00_03 .. DSC_PPS84_87, big endian per word) are not a copy of
 * the panel PPS. Required equal: comp_cfg and bits per pixel (4-5),
 * picture height (6-7), slice height (10-11), slice width (12-13) and
 * chunk size (14-15); picture width (8-9) times two = the PPS width.
 * The remaining bytes are logged, not compared.
 */
static int pr_check_dsc(struct dpu_dev *d)
{
	const struct dpu_lcd *l = &d->lcd;
	u32 path = decon_rd(d, DATA_PATH_CONTROL_2);
	u8 enc[88];
	int e, i, diff;

	if (!l->dsc_en || pr_be16(&pr_pps[6]) != l->yres || pr_be16(&pr_pps[8]) != l->xres ||
	    pr_be16(&pr_pps[10]) != l->dsc_h || pr_be16(&pr_pps[12]) != l->dsc_w) {
		pr_err(DPU_PR_PFX "DT display %ux%u dsc %u slice %ux%u, the 4.9 PPS is 1440x2960 slice 720x40\n",
		       l->xres, l->yres, l->dsc_en, l->dsc_w, l->dsc_h);
		return -EINVAL;
	}
	if (COMP_OUTIF_PATH_GET(path) != 0xB1) {
		pr_err(DPU_PR_PFX "DATA_PATH_CONTROL_2 %#x: data path %#x, 4.9 DSCC + DSC ENC0/1 + DSIM IF0 is 0xb1\n",
		       path, COMP_OUTIF_PATH_GET(path));
		return -EINVAL;
	}
	for (e = 0; e < 2; e++) {
		u32 base = (e ? DSC1_OFFSET : DSC0_OFFSET) + DSC_PPS00_03;

		for (i = 0; i < ARRAY_SIZE(enc); i += 4) {
			u32 w = decon_rd(d, base + i);

			enc[i] = w >> 24;
			enc[i + 1] = w >> 16;
			enc[i + 2] = w >> 8;
			enc[i + 3] = w;
		}
		prc_dbg("DSC%d PPS 00-43 %*ph\n", e, 44, enc);
		prc_dbg("DSC%d PPS 44-87 %*ph\n", e, 44, enc + 44);
		if (memcmp(&enc[4], &pr_pps[4], 4) || memcmp(&enc[10], &pr_pps[10], 6) ||
		    pr_be16(&enc[8]) * 2 != pr_be16(&pr_pps[8])) {
			pr_err(DPU_PR_PFX "DSC%d encoder PPS %*ph (bytes 4-15) does not fit the 4.9 PPS %*ph\n",
			       e, 12, &enc[4], 12, &pr_pps[4]);
			return -EINVAL;
		}
		for (diff = 0, i = 0; i < ARRAY_SIZE(enc); i++)
			diff += enc[i] != pr_pps[i];
		pr_info(DPU_PR_PFX "DSC%d encoder fits the PPS (%d of 88 bytes differ, width per encoder %u)\n",
			e, diff, pr_be16(&enc[8]));
	}
	return 0;
}

/*
 * Level 2 reset pin: the descriptor must be gpa3 pin 4 of the ALIVE pin
 * controller, configured as an output (CON[19:16] = 1) and driven high,
 * as SBoot leaves it. CON/DAT are read through a private read-only
 * mapping; the pulse itself goes through gpiolib (samsung_gpio_set(): a
 * read-modify-write of DAT bit 4 under the bank lock).
 */
struct pr_gpio {
	struct gpio_desc *desc;
	void __iomem *regs;		/* gpa3 CON, DAT; read only */
	u32 con0, dat0, out_mask;	/* other output pins of gpa3 */
};

static void pr_gpio_put(struct pr_gpio *g)
{
	if (g->regs)
		iounmap(g->regs);
	if (!IS_ERR_OR_NULL(g->desc))
		gpiod_put(g->desc);
	g->regs = NULL;
	g->desc = NULL;
}

static int pr_gpio_get(struct dpu_dev *d, struct pr_gpio *g)
{
	struct device_node *np = of_parse_phandle(d->dev->of_node, "samsung,panel", 0);
	struct gpio_device *gdev;
	struct gpio_chip *gc;
	struct resource *res;
	const char *label;
	int ofs, i;

	memset(g, 0, sizeof(*g));
	if (!np)
		return -ENODEV;
	g->desc = fwnode_gpiod_get_index(of_fwnode_handle(np), "samsung,panel-reset", 0,
					 GPIOD_ASIS, "panel-reset");
	of_node_put(np);
	if (IS_ERR(g->desc)) {
		int ret = PTR_ERR(g->desc);

		pr_err(DPU_PR_PFX "samsung,panel-reset-gpios: %d\n", ret);
		g->desc = NULL;
		return ret;
	}
	gdev = gpiod_to_gpio_device(g->desc);
	gc = gpio_device_get_chip(gdev);
	label = gpio_device_get_label(gdev);
	ofs = desc_to_gpio(g->desc) - gpio_device_get_base(gdev);
	res = gc && gc->parent && dev_is_platform(gc->parent) ?
	      platform_get_resource(to_platform_device(gc->parent), IORESOURCE_MEM, 0) : NULL;
	if (!label || strcmp(label, "gpa3") || ofs != PR_RESET_PIN || !res ||
	    res->start != PR_ALIVE_BASE) {
		pr_err(DPU_PR_PFX "reset gpio is %s-%d of %pR, expected gpa3-%d of pinctrl@%x\n",
		       label ?: "?", ofs, res, PR_RESET_PIN, PR_ALIVE_BASE);
		pr_gpio_put(g);
		return -EINVAL;
	}
	g->regs = ioremap(PR_ALIVE_BASE + PR_GPA3_CON, 8);
	if (!g->regs) {
		pr_gpio_put(g);
		return -ENOMEM;
	}
	g->con0 = readl(g->regs);
	g->dat0 = readl(g->regs + PR_GPA3_DAT - PR_GPA3_CON);
	for (i = 0; i < 8; i++)
		if (i != PR_RESET_PIN && ((g->con0 >> (4 * i)) & 0xf) == 1)
			g->out_mask |= BIT(i);
	if (((g->con0 >> (4 * PR_RESET_PIN)) & 0xf) != 1 || !(g->dat0 & BIT(PR_RESET_PIN)) ||
	    gpiod_get_raw_value(g->desc) != 1) {
		pr_err(DPU_PR_PFX "gpa3 CON %#010x DAT %#04x: pin 4 is not an output driven high\n",
		       g->con0, g->dat0);
		pr_gpio_put(g);
		return -EINVAL;
	}
	pr_info(DPU_PR_PFX "reset gpio gpa3-4: CON %#010x DAT %#04x (output, high)\n",
		g->con0, g->dat0);
	return 0;
}

/* after a DAT write: pin 4 as asked, CON and the other output pins untouched */
static int pr_gpio_verify(struct pr_gpio *g, int want, const char *when)
{
	u32 con = readl(g->regs), dat = readl(g->regs + PR_GPA3_DAT - PR_GPA3_CON);

	if (con != g->con0 || ((dat ^ g->dat0) & g->out_mask) ||
	    !!(dat & BIT(PR_RESET_PIN)) != want) {
		pr_err(DPU_PR_PFX "reset %s: gpa3 CON %#010x DAT %#04x (before %#010x %#04x), want pin 4 = %d\n",
		       when, con, dat, g->con0, g->dat0, want);
		return -EIO;
	}
	return 0;
}

/*
 * 4.9 __set_panel_power() (CONFIG_OLD_DISP_TIMING off, panel_drv.c:119-160):
 * power off drives reset low before the rails go off; power on enables the
 * rails, waits 10 ms, drives reset high and waits 5 ms; then the DSIM is
 * re-initialised (LP-11) and panel_sleep_out() sends the init table. Here
 * the rails and the DSIM link stay on: reset low, 10 ms, reset high, 5 ms.
 * Reset is driven high again whatever happened.
 */
static int pr_reset_pulse(struct dpu_dev *d, struct pr_gpio *g)
{
	int ret, ret2;

	gpiod_set_raw_value(g->desc, 0);
	ret = pr_gpio_verify(g, 0, "low");
	usleep_range(PR_RESET_LOW_US, PR_RESET_LOW_US + 10);
	gpiod_set_raw_value(g->desc, 1);
	ret2 = pr_gpio_verify(g, 1, "high");
	usleep_range(PR_RESET_HIGH_US, PR_RESET_HIGH_US + 10);
	prc_dbg("reset pulse: low %d, high %d\n", ret, ret2);
	return ret ? ret : ret2;
}

/* ---- the work ----------------------------------------------------------- */

static int pr_ee_elvdd(const struct pb_status *s)
{
	return s->keyed && !s->ret_k && (s->ee_k & 0x08);
}

/* window 1: checks, then display off, exit, (reset), init, set-bl, COPR */
static int pr_window_off_on(struct dpu_dev *d, struct pb_status *sb, struct pb_calib *c,
			    struct pr_gpio *g, struct pr_result *r, const char **why)
{
	int ret;

	ret = dpu_dsim_check(d);
	if (ret) {
		*why = "DSIM state";
		return ret;
	}
	dpu_dsim_clear_status(d);
	ret = pr_check_dsc(d);
	if (ret) {
		*why = "DECON DSC configuration";
		return ret;
	}
	if (!d->pb_has_id || !d->pb_has_gamma) {	/* dpu_panel_recovery_init() requires both */
		*why = "no DT panel id / HBM gamma cross-check";
		return -EINVAL;
	}
	ret = pb_prepare(d, sb, c);	/* RDDPM, panel id, calibration, keyed EE */
	if (ret) {
		*why = "panel state / id / calibration";
		return ret;
	}
	if (d->pr_only_elvdd && !pr_ee_elvdd(sb)) {
		*why = "no ELVDD error in the keyed EE";
		return -ECANCELED;
	}
	if (d->pr_level >= 2) {
		ret = pr_gpio_get(d, g);
		if (ret) {
			*why = "reset gpio";
			return ret;
		}
	}

	/* from here on writes go out: no refusal, every phase runs */
	*why = NULL;
	d->pr_active = true;
	pr_run(d, "display off", pr_tbl_display_off, ARRAY_SIZE(pr_tbl_display_off), r);
	pr_run(d, "exit", pr_tbl_exit, ARRAY_SIZE(pr_tbl_exit), r);
	if (d->pr_level >= 2) {
		r->reset = pr_reset_pulse(d, g);
		pr_note_err(r, r->reset);
	}
	pr_run(d, "init", pr_tbl_init, ARRAY_SIZE(pr_tbl_init), r);
	r->ret_bl = pb_send_bl(d, c);
	pr_note_err(r, r->ret_bl);
	pr_run(d, "copr", pr_tbl_copr, ARRAY_SIZE(pr_tbl_copr), r);
	d->pr_active = false;
	return 0;
}

/* window 2: the display on table, once a frame went out (4.9 FRAME_DONE) */
static int pr_display_on_window(struct dpu_dev *d, struct pr_result *r)
{
	u32 trig = 0;
	int tries, ret = -EBUSY;

	for (tries = 0; tries < 3; tries++) {
		ret = dpu_cmd_window_begin(d, &trig);
		if (!ret)
			break;
		msleep(20);
	}
	if (ret) {
		pr_err(DPU_PR_PFX "no command window for the display on (%d)\n", ret);
		return ret;
	}
	ret = dpu_dsim_check(d);
	if (!ret) {
		dpu_dsim_clear_status(d);
		d->pr_active = true;
		pr_run(d, "display on", pr_tbl_display_on, ARRAY_SIZE(pr_tbl_display_on), r);
		d->pr_active = false;
	}
	dpu_cmd_window_end(d, trig);
	return ret;
}

void dpu_panel_recovery_work(struct work_struct *work)
{
	struct dpu_dev *d = container_of(to_delayed_work(work), struct dpu_dev, pr_work);
	u64 ms = ktime_to_ms(ktime_get_boottime());
	u32 ntx0, nerr0, trig = 0, us, fc, i;
	struct pb_status sb, sa;
	struct pr_result r = { .reset = 1, .ret_bl = -ENODATA };
	struct pr_gpio g = { };
	struct pb_calib c;
	const char *why = NULL;
	char eeb[8] = "-", eea[8] = "-";
	bool pm_ok, ee_ok;
	ktime_t t0;
	int ret;

	if (d->pr_done || READ_ONCE(d->stopping))
		return;
	d->pr_done = true;		/* once, whatever the outcome */
	for (i = 0; i < PR_NRD; i++)
		r.rd_ret[i] = -ENODATA;
	pb_status_init(&sb);
	pb_status_init(&sa);
	mutex_lock(&d->panel_lock);
	ntx0 = d->dsim_ntx;
	nerr0 = d->dsim_nerr;
	pr_info(DPU_PR_PFX "start at %llu.%03llu s: level %u%s, DECON FRAME_COUNT=%u TE=%d\n",
		ms / 1000, ms % 1000, d->pr_level,
		d->pr_only_elvdd ? " (only on ELVDD error)" : "", dpu_hw_frame_count(d),
		atomic_read(&d->te_count));

	ret = dpu_cmd_window_begin(d, &trig);
	if (ret) {
		pr_err(DPU_PR_PFX "RESULT REFUSED: no command window (%d), nothing sent\n", ret);
		goto out;
	}
	t0 = ktime_get();
	ret = pr_window_off_on(d, &sb, &c, &g, &r, &why);
	us = ktime_us_delta(ktime_get(), t0);
	fc = dpu_hw_frame_count(d);
	dpu_cmd_window_end(d, trig);
	pb_status_log("before recovery", &sb);
	if (why) {
		pr_err(DPU_PR_PFX "RESULT REFUSED (%s): %d, only reads sent\n", why, ret);
		goto out;
	}
	if (r.reset <= 0)
		pr_info(DPU_PR_PFX "reset pulse: %d\n", r.reset);

	/* frames run again: wait for the first one (4.9 sends display on at FRAME_DONE) */
	for (i = 0; i < PR_FRAME_WAIT_MS / 2; i++) {
		if (dpu_hw_frame_count(d) != fc)
			break;
		usleep_range(2000, 2100);
	}
	r.frames = dpu_hw_frame_count(d) - fc;
	r.frame_wait_ms = i * 2;
	if (!r.frames)
		pr_err(DPU_PR_PFX "no frame within %u ms after the init table; display on anyway\n",
		       PR_FRAME_WAIT_MS);
	ret = pr_display_on_window(d, &r);
	pr_note_err(&r, ret);

	/* status after: the read-back of the set-bl registers, then RDDPM/EE/EA */
	msleep(PR_SETTLE_MS);
	d->pb_readback = "not run";
	if (!dpu_cmd_window_begin(d, &trig)) {
		if (!dpu_dsim_check(d)) {
			dpu_dsim_clear_status(d);
			pb_readback(d, &sa, &c);
		}
		dpu_cmd_window_end(d, trig);
	}
	pb_status_log("after recovery", &sa);
	if (!r.ret_bl)
		pb_publish(d->pb_target);

	pm_ok = !sa.ret && (sa.pm[0] & 0x7c) == 0x1c;
	ee_ok = sa.keyed && !sa.ret_k && !(sa.ee_k & 0x4c);
	if (sb.keyed && !sb.ret_k)
		snprintf(eeb, sizeof(eeb), "%#04x", sb.ee_k);
	if (sa.keyed && !sa.ret_k)
		snprintf(eea, sizeof(eea), "%#04x", sa.ee_k);
	pr_info(DPU_PR_PFX "4.9 dumps: exit EE %#04x (%d), exit 05 %#04x (%d), init 0F %#04x (%d)\n",
		r.rd[PR_RD_EXIT_EE], r.rd_ret[PR_RD_EXIT_EE], r.rd[PR_RD_EXIT_DSIE],
		r.rd_ret[PR_RD_EXIT_DSIE], r.rd[PR_RD_INIT_DIAG], r.rd_ret[PR_RD_INIT_DIAG]);
	pr_info(DPU_PR_PFX "RESULT %s: level %u, first error %d, %u packets (%u failed), set-bl %d, reset %s, %u frames in %u ms before display on, read-back %s, RDDPM after %#04x, EE (key2) before %s after %s, window 1 %u us, %u DSIM tx, %u DSIM errors\n",
		!r.err && pm_ok && ee_ok ? "PASS" : "FAIL", d->pr_level, r.err, r.sent, r.failed,
		r.ret_bl, r.reset > 0 ? "not run" : r.reset ? "FAILED" : "ok", r.frames,
		r.frame_wait_ms, d->pb_readback, sa.ret ? 0 : sa.pm[0], eeb, eea, us,
		d->dsim_ntx - ntx0, d->dsim_nerr - nerr0);
out:
	pr_gpio_put(&g);
	mutex_unlock(&d->panel_lock);
}

/* decon_drv.c probe, after dpu_panel_bright_init() succeeded */
int dpu_panel_recovery_init(struct dpu_dev *d)
{
	struct device_node *np = of_parse_phandle(d->dev->of_node, "samsung,panel", 0);
	int ret = -ENODEV;

	if (!np)
		return -ENODEV;
	if (of_property_read_u32(np, "samsung,panel-recovery-level", &d->pr_level))
		goto out;
	ret = -EINVAL;
	if (d->pr_level != 1 && d->pr_level != 2) {
		dev_err(d->dev, "panel-recovery: level %u not implemented (1 = DCS sleep cycle, 2 = plus reset pulse)\n",
			d->pr_level);
		goto out;
	}
	/*
	 * Sol review R1 #4: the set-bl gamma comes from the panel's B3 read;
	 * pb_prepare() compares it with DT only when the DT value exists, so
	 * both cross-checks are mandatory here, with their exact lengths.
	 */
	if (!d->pb_has_id || of_property_count_u8_elems(np, "samsung,panel-id") != 3) {
		dev_err(d->dev, "panel-recovery: needs samsung,panel-id (3 bytes)\n");
		goto out;
	}
	if (!d->pb_has_gamma ||
	    of_property_count_u8_elems(np, "samsung,hbm-gamma-expect") != PB_HBM_GAMMA_LEN) {
		dev_err(d->dev, "panel-recovery: needs samsung,hbm-gamma-expect (%d bytes)\n",
			PB_HBM_GAMMA_LEN);
		goto out;
	}
	if (d->pr_level >= 2 && !of_property_present(np, "samsung,panel-reset-gpios")) {
		dev_err(d->dev, "panel-recovery: level 2 needs samsung,panel-reset-gpios\n");
		goto out;
	}
	d->pr_at_ms = 8000;
	of_property_read_u32(np, "samsung,panel-recovery-at-ms", &d->pr_at_ms);
	d->pr_only_elvdd = of_property_read_bool(np, "samsung,panel-recovery-only-on-elvdd-error");
	ret = 0;
out:
	of_node_put(np);
	return ret;
}
