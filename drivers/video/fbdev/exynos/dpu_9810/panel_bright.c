// SPDX-License-Identifier: GPL-2.0
/*
 * Galaxy Note9 (crownlte) S6E3HA8 "crown_a3_s0" fault-display brightness.
 *
 * The owner's panel looks black below a high brightness. Their 4.9 kernel
 * (panel_bl.c, commit cf5bb86c "keep crownlte normal display at calibrated
 * maximum brightness") forces PANEL_FAULT_DISPLAY_BRIGHTNESS = 44700 in the
 * normal display state. This file sends the one panel command sequence 4.9
 * sends for that brightness, once, after the DECON driver adopted SBoot's
 * display. It never sends power, sleep, display on/off, reset, timing,
 * DSC/PPS, MTP/OTP-write or POC/flash commands.
 *
 * 44700 = BRT(447) = crown_a3_s0_brt_tbl[86], the extended-HBM row
 * (720 nit). For that row 4.9 (s6e3ha8_crown_a3_s0_panel.h
 * crown_a3_s0_set_bl_cmdtbl, PANEL_SET_BL_SEQ) sends, through
 * common_mipi_lcd.c mipi_write() (size 1 = DCS short 0x05, else DCS long
 * 0x39, offset > 0 = DCS short-param 0x15 "B0 <offset>" first):
 *
 *   F0 5A 5A                          level 2 key on
 *   CA <34 bytes>                     gamma = HBM gamma read from B3 (OTP)
 *   B1 00 0C                          AOR, aor_table[86]
 *   B5 19 DC 00                       TSET 25 C, MPS row 1, ELVSS[86]
 *   B0 16 / B5 <elvss_temp_1>         ELVSS temp, OTP B5[23] (rows >= 74)
 *   F4 EB 26                          VGH (fixed) / VINT[86]
 *   B4 00 55 80 65 26 26              ACL control (pwrsave on, HBM, 8%)
 *   B0 0F / B4 20                     ACL dimming frames
 *   55 02                             ACL on
 *   B0 12 / B8 25 69 D6 62 06 57 53 89 83 83 83 2C 2C 2C 2C 2C 2C   IRC
 *   B8 15                             IRC on
 *   B0 06 / B1 01 AF 54 68 CC 78 30 CC 64 FF                           POC comp 1
 *   B0 53 / B8 00 00                  POC comp 2, poc_comp_table[86]
 *   F7 03                             gamma/LTPS update
 *   F0 A5 A5                          level 2 key off
 *
 * Fixed bytes were checked against the compiled 4.9 object
 * (build/close-range-fix-4.9 .../s6e3ha8/s6e3ha8.o). Two values are per
 * panel and are read here first, the way 4.9 reads them at probe
 * (s6e3ha8.h READ_HBM_GAMMA / READ_ELVSS_TEMP_1, panel.c panel_rx_nbytes):
 * the 34-byte HBM gamma (B0 02, DCS read B3) and ELVSS temp 1 (B0 17,
 * DCS read B5, 1 byte). The gamma is cross-checked against the value the
 * running 4.9 kernel printed for gamma[720] when DT provides it.
 *
 * IRC is the table-mode value (crown_a3_s0_dimming_irc_table row 73 scaled
 * to 700 nit by irc_interpolation(), = rows 85/86). 4.9 sends exactly this
 * until its dim-flash work replaces row 73 with the panel flash copy
 * ("dim:table" in 4.9 logs); reading that flash needs POC commands, which
 * this driver does not send.
 */
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <video/mipi_display.h>

#include "dpu9810.h"

#define PB_BRT_EXT_HBM		44700	/* BRT(447), crown_a3_s0_brt_tbl[86] */
#define PB_HBM_GAMMA_LEN	34	/* S6E3HA8_HBM_GAMMA_LEN */
#define PB_HBM_GAMMA_OFS	2	/* S6E3HA8_HBM_GAMMA_OFS */
#define PB_ELVSS_TEMP_0_OFS	22	/* S6E3HA8_ELVSS_TEMP_0_OFS */
#define PB_ELVSS_TEMP_1_OFS	23	/* S6E3HA8_ELVSS_TEMP_1_OFS */
#define PB_IRC_VALUE_OFS	18	/* S6E3HA8_IRC_VALUE_OFS */
#define PB_MAX_PKT		35

static u32 pb_reported;		/* brightness the panel was set to, 0 = untouched */

u32 dpu9810_panel_brightness(void)
{
	return READ_ONCE(pb_reported);
}

struct pb_pkt {
	u8 ofs;
	u8 len;
	u8 data[PB_MAX_PKT];
	const char *what;
};

/*
 * Packets this file may write, by first byte, length and global-parameter
 * offset: exactly the shapes of crown_a3_s0_set_bl_cmdtbl.
 */
static bool pb_write_allowed(u8 ofs, const u8 *b, size_t len)
{
	switch (b[0]) {
	case 0xF0:
		return !ofs && len == 3 && b[1] == b[2] && (b[1] == 0x5A || b[1] == 0xA5);
	case 0xCA:
		return !ofs && len == 1 + PB_HBM_GAMMA_LEN;
	case 0xB1:
		return (!ofs && len == 3) || (ofs == 0x06 && len == 11);
	case 0xB5:
		return (!ofs && len == 4) || (ofs == PB_ELVSS_TEMP_0_OFS && len == 2);
	case 0xF4:
		return !ofs && len == 3 && b[1] == 0xEB;
	case 0xB4:
		return (!ofs && len == 7) || (ofs == 0x0F && len == 2);
	case 0x55:
		return !ofs && len == 2;
	case 0xB8:
		return (ofs == PB_IRC_VALUE_OFS && len == 18) ||
		       (!ofs && len == 2 && b[1] == 0x15) || (ofs == 0x53 && len == 3);
	case 0xF7:
		return !ofs && len == 2 && b[1] == 0x03;
	default:
		return false;
	}
}

static bool pb_read_allowed(u8 reg)
{
	switch (reg) {
	case MIPI_DCS_GET_POWER_MODE:	/* 0x0A */
	case 0x04:			/* ID */
	case 0xB1: case 0xB3: case 0xB4: case 0xB5: case 0xB8: case 0xCA: case 0xF4:
		return true;
	default:
		return false;
	}
}

/* 4.9 common_mipi_lcd.c mipi_write(), without the reset-and-retry loop */
static int pb_write(struct dpu_dev *d, u8 ofs, const u8 *buf, size_t len)
{
	int ret;

	if (!len || !pb_write_allowed(ofs, buf, len)) {
		pr_err(DPU_PB_PFX "write %*ph (ofs %#x) not in the 4.9 brightness sequence, refused\n",
		       (int)min_t(size_t, len, 8), buf, ofs);
		return -EPERM;
	}
	if (ofs) {
		u8 gpara[2] = { 0xB0, ofs };

		ret = dpu_dsim_tx(d, MIPI_DSI_DCS_SHORT_WRITE_PARAM, gpara, 2);
		if (ret)
			return ret;
	}
	return dpu_dsim_tx(d, len == 1 ? MIPI_DSI_DCS_SHORT_WRITE : MIPI_DSI_DCS_LONG_WRITE,
			   buf, len);
}

/* 4.9 mipi_read(): optional "B0 <offset>", then dsim_read_data(DCS_READ) */
static int pb_read(struct dpu_dev *d, u8 reg, u8 ofs, u8 *buf, size_t len)
{
	int ret;

	if (!pb_read_allowed(reg))
		return -EPERM;
	if (ofs) {
		u8 gpara[2] = { 0xB0, ofs };

		ret = dpu_dsim_tx(d, MIPI_DSI_DCS_SHORT_WRITE_PARAM, gpara, 2);
		if (ret)
			return ret;
	}
	return dpu_dsim_dcs_read(d, reg, buf, len);
}

static const u8 pb_key2_on[] = { 0xF0, 0x5A, 0x5A };
static const u8 pb_key2_off[] = { 0xF0, 0xA5, 0xA5 };

/* crown_a3_s0_dimming_irc_table[73] scaled to 700 nit = rows 85 and 86 */
static const u8 pb_irc_value[] = { 0xB8, 0x25, 0x69, 0xD6, 0x62, 0x06, 0x57, 0x53, 0x89,
				   0x83, 0x83, 0x83, 0x2C, 0x2C, 0x2C, 0x2C, 0x2C, 0x2C };

static void pb_add(struct pb_pkt *p, int *n, u8 ofs, const u8 *data, size_t len,
		   const char *what)
{
	p[*n].ofs = ofs;
	p[*n].len = len;
	memcpy(p[*n].data, data, len);
	p[*n].what = what;
	(*n)++;
}

/*
 * The crown_a3_s0 PANEL_SET_BL_SEQ for brightness 44700 (row 86), with the
 * two per-panel values filled in.
 */
static int pb_build(struct pb_pkt *p, const u8 *gamma, u8 elvss_temp1)
{
	static const u8 aor[] = { 0xB1, 0x00, 0x0C };
	static const u8 tset_mps_elvss[] = { 0xB5, 0x19, 0xDC, 0x00 };
	static const u8 vgh_vint[] = { 0xF4, 0xEB, 0x26 };
	static const u8 acl_control[] = { 0xB4, 0x00, 0x55, 0x80, 0x65, 0x26, 0x26 };
	static const u8 acl_dim_frm[] = { 0xB4, 0x20 };
	static const u8 acl_onoff[] = { 0x55, 0x02 };
	static const u8 irc_on[] = { 0xB8, 0x15 };
	static const u8 poc_comp1[] = { 0xB1, 0x01, 0xAF, 0x54, 0x68, 0xCC, 0x78, 0x30, 0xCC,
					0x64, 0xFF };
	static const u8 poc_comp2[] = { 0xB8, 0x00, 0x00 };
	static const u8 gamma_update[] = { 0xF7, 0x03 };
	u8 g[1 + PB_HBM_GAMMA_LEN], et[2] = { 0xB5, elvss_temp1 };
	int n = 0;

	g[0] = 0xCA;
	memcpy(&g[1], gamma, PB_HBM_GAMMA_LEN);

	pb_add(p, &n, 0, pb_key2_on, sizeof(pb_key2_on), "level 2 key on");
	pb_add(p, &n, 0, g, sizeof(g), "gamma = HBM gamma (B3)");
	pb_add(p, &n, 0, aor, sizeof(aor), "AOR row 86");
	pb_add(p, &n, 0, tset_mps_elvss, sizeof(tset_mps_elvss), "TSET 25C / MPS / ELVSS row 86");
	pb_add(p, &n, PB_ELVSS_TEMP_0_OFS, et, sizeof(et), "ELVSS temp (OTP B5[23])");
	pb_add(p, &n, 0, vgh_vint, sizeof(vgh_vint), "VGH / VINT row 86");
	pb_add(p, &n, 0, acl_control, sizeof(acl_control), "ACL control");
	pb_add(p, &n, 0x0F, acl_dim_frm, sizeof(acl_dim_frm), "ACL dimming frames");
	pb_add(p, &n, 0, acl_onoff, sizeof(acl_onoff), "ACL on");
	pb_add(p, &n, PB_IRC_VALUE_OFS, pb_irc_value, sizeof(pb_irc_value),
	       "IRC value (table mode)");
	pb_add(p, &n, 0, irc_on, sizeof(irc_on), "IRC on");
	pb_add(p, &n, 0x06, poc_comp1, sizeof(poc_comp1), "POC comp 1");
	pb_add(p, &n, 0x53, poc_comp2, sizeof(poc_comp2), "POC comp 2 row 86");
	pb_add(p, &n, 0, gamma_update, sizeof(gamma_update), "gamma update");
	pb_add(p, &n, 0, pb_key2_off, sizeof(pb_key2_off), "level 2 key off");
	return n;
}

/*
 * 4.9 generate_hbm_gamma() for luminance > hbm_target_luminance (720 > 700):
 * gamma_ctoi_v0() of the B3 bytes, then copy_tpout_center_v0() into a row
 * whose upper bits of byte 0 are clear. The round trip keeps every byte
 * except bits 7..3 of byte 0 (only the V255 bit 8 of R/G/B lives there).
 */
static void pb_hbm_to_gamma(const u8 *hbm, u8 *gamma)
{
	memcpy(gamma, hbm, PB_HBM_GAMMA_LEN);
	gamma[0] = hbm[0] & 0x07;
}

struct pb_state {
	u8 ca[PB_HBM_GAMMA_LEN], b1[2], b5[3], f4[2], b8[17];
};

/* read-only snapshot of the registers the sequence writes */
static int pb_snapshot(struct dpu_dev *d, struct pb_state *s)
{
	int ret;

	ret = pb_read(d, 0xCA, 0, s->ca, sizeof(s->ca));
	if (!ret)
		ret = pb_read(d, 0xB1, 0, s->b1, sizeof(s->b1));
	if (!ret)
		ret = pb_read(d, 0xB5, 0, s->b5, sizeof(s->b5));
	if (!ret)
		ret = pb_read(d, 0xF4, 0, s->f4, sizeof(s->f4));
	if (!ret)
		ret = pb_read(d, 0xB8, PB_IRC_VALUE_OFS, s->b8, sizeof(s->b8));
	return ret;
}

/* make /sys/class/backlight/panel/brightness show what the panel now runs at */
static void pb_publish(u32 v)
{
	struct backlight_device *bd;

	WRITE_ONCE(pb_reported, v);
	if (!IS_BUILTIN(CONFIG_BACKLIGHT_CLASS_DEVICE))
		return;
	bd = backlight_device_get_by_name("panel");
	if (!bd)
		return;
	mutex_lock(&bd->ops_lock);
	if (v <= bd->props.max_brightness)
		bd->props.brightness = v;
	mutex_unlock(&bd->ops_lock);
	put_device(&bd->dev);
}

static int pb_sequence(struct dpu_dev *d)
{
	struct pb_pkt pkt[16];
	struct pb_state before, after;
	u8 rddpm, id[3], hbm[PB_HBM_GAMMA_LEN], gamma[PB_HBM_GAMMA_LEN], et1;
	int i, n, ret, ret2;
	bool key2 = false;

	/* 1. normal display state: sleep out, normal mode, display on, not idle (AOD) */
	ret = pb_read(d, MIPI_DCS_GET_POWER_MODE, 0, &rddpm, 1);
	if (ret)
		return ret;
	if ((rddpm & 0x7c) != 0x1c) {
		pr_err(DPU_PB_PFX "panel power mode %#04x is not normal display (want sleep-out, normal, display-on, no idle/partial)\n",
		       rddpm);
		return -ENODEV;
	}

	/* 2. the panel the tables belong to */
	ret = pb_read(d, 0x04, 0, id, sizeof(id));
	if (ret)
		return ret;
	if (d->pb_has_id && memcmp(id, d->pb_id, sizeof(id))) {
		pr_err(DPU_PB_PFX "panel id %*ph, DT expects %*ph\n", 3, id, 3, d->pb_id);
		return -ENODEV;
	}

	/*
	 * 3. calibration reads, under the level 2 key as 4.9's dim-flash
	 * resource init re-reads the HBM gamma.
	 */
	ret = pb_write(d, 0, pb_key2_on, sizeof(pb_key2_on));
	if (ret)
		return ret;
	key2 = true;
	ret = pb_read(d, 0xB3, PB_HBM_GAMMA_OFS, hbm, sizeof(hbm));
	if (!ret)
		ret = pb_read(d, 0xB5, PB_ELVSS_TEMP_1_OFS, &et1, 1);
	if (!ret)
		ret = pb_snapshot(d, &before);
	ret2 = pb_write(d, 0, pb_key2_off, sizeof(pb_key2_off));
	if (!ret2)
		key2 = false;
	if (ret || ret2)
		goto fail;

	pb_hbm_to_gamma(hbm, gamma);
	pr_info(DPU_PB_PFX "HBM gamma (B3[2..35]) %*ph -> CA payload %*ph\n",
		PB_HBM_GAMMA_LEN, hbm, PB_HBM_GAMMA_LEN, gamma);
	pr_info(DPU_PB_PFX "ELVSS temp 1 (B5[23]) %#04x\n", et1);
	if (d->pb_has_gamma && memcmp(gamma, d->pb_gamma, sizeof(gamma))) {
		pr_err(DPU_PB_PFX "CA payload differs from the 4.9 gamma[720] in DT %*ph: not writing\n",
		       PB_HBM_GAMMA_LEN, d->pb_gamma);
		return -EINVAL;
	}
	if (before.f4[0] != 0xEB) {
		pr_err(DPU_PB_PFX "F4 VGH byte %#04x, 4.9 always writes 0xEB: not writing\n",
		       before.f4[0]);
		return -EINVAL;
	}

	/* 4. the 4.9 set-bl sequence */
	n = pb_build(pkt, gamma, et1);
	pr_info(DPU_PB_PFX "sending the 4.9 PANEL_SET_BL_SEQ for %u (%d packets)\n",
		d->pb_target, n);
	for (i = 0; i < n; i++) {
		pr_info(DPU_PB_PFX "step %d/%d: %s\n", i + 1, n, pkt[i].what);
		ret = pb_write(d, pkt[i].ofs, pkt[i].data, pkt[i].len);
		if (ret)
			break;
		key2 = (i < n - 1);
	}
	if (ret)
		goto fail;

	/* 5. read back (diagnostic only) */
	ret = pb_write(d, 0, pb_key2_on, sizeof(pb_key2_on));
	if (!ret) {
		key2 = true;
		ret = pb_snapshot(d, &after);
		ret2 = pb_write(d, 0, pb_key2_off, sizeof(pb_key2_off));
		if (!ret2)
			key2 = false;
	}
	if (ret) {
		pr_warn(DPU_PB_PFX "read-back failed (%d); the sequence itself completed\n", ret);
	} else {
		bool ok = !memcmp(after.ca, gamma, sizeof(gamma)) &&
			  after.b1[0] == 0x00 && after.b1[1] == 0x0C &&
			  after.b5[0] == 0x19 && after.b5[1] == 0xDC && after.b5[2] == 0x00 &&
			  after.f4[0] == 0xEB && after.f4[1] == 0x26 &&
			  !memcmp(after.b8, pb_irc_value + 1, sizeof(after.b8));

		pr_info(DPU_PB_PFX "read-back %s: CA %s, B1 %*ph -> %*ph, B5 %*ph -> %*ph, F4 %*ph -> %*ph, B8[18..] %*ph -> %*ph\n",
			ok ? "MATCH" : "MISMATCH",
			memcmp(after.ca, gamma, sizeof(gamma)) ? "differs" : "matches",
			2, before.b1, 2, after.b1, 3, before.b5, 3, after.b5,
			2, before.f4, 2, after.f4, 17, before.b8, 17, after.b8);
		pr_info(DPU_PB_PFX "CA before %*ph\n", PB_HBM_GAMMA_LEN, before.ca);
		pr_info(DPU_PB_PFX "CA after  %*ph\n", PB_HBM_GAMMA_LEN, after.ca);
	}
	return 0;

fail:
	if (key2 && pb_write(d, 0, pb_key2_off, sizeof(pb_key2_off)))
		pr_err(DPU_PB_PFX "could not send level 2 key off\n");
	return ret ? ret : ret2;
}

/* decon_drv.c probe: parse the opt-in DT properties of the panel node */
int dpu_panel_bright_init(struct dpu_dev *d)
{
	struct device_node *np = of_parse_phandle(d->dev->of_node, "samsung,panel", 0);
	struct resource *r;
	int ret = -ENODEV;

	if (!np)
		return -ENODEV;
	if (of_property_read_u32(np, "samsung,fault-display-brightness", &d->pb_target))
		goto out;
	if (d->pb_target != PB_BRT_EXT_HBM) {
		dev_err(d->dev, "panel-bright: only %u (BRT(447), the 4.9 fault-display value) is implemented, DT asks %u\n",
			PB_BRT_EXT_HBM, d->pb_target);
		ret = -EINVAL;
		goto out;
	}
	d->pb_at_ms = 96000;
	of_property_read_u32(np, "samsung,fault-display-brightness-at-ms", &d->pb_at_ms);
	d->pb_has_id = !of_property_read_u8_array(np, "samsung,panel-id", d->pb_id,
						 sizeof(d->pb_id));
	d->pb_has_gamma = !of_property_read_u8_array(np, "samsung,hbm-gamma-expect",
						    d->pb_gamma, sizeof(d->pb_gamma));
	if (!d->pb_has_id || !d->pb_has_gamma)
		dev_warn(d->dev, "panel-bright: no %s cross-check in DT\n",
			 d->pb_has_id ? "gamma" : "panel id");

	r = platform_get_resource_byname(to_platform_device(d->dev), IORESOURCE_MEM, "dsim");
	if (!r) {
		dev_err(d->dev, "panel-bright: no \"dsim\" register window\n");
		goto out;
	}
	d->dsim = devm_ioremap_resource(d->dev, r);
	if (IS_ERR(d->dsim)) {
		ret = PTR_ERR(d->dsim);
		d->dsim = NULL;
		goto out;
	}
	ret = 0;
out:
	of_node_put(np);
	return ret;
}

void dpu_panel_bright_work(struct work_struct *work)
{
	struct dpu_dev *d = container_of(to_delayed_work(work), struct dpu_dev, pb_work);
	u64 ms = ktime_to_ms(ktime_get_boottime());
	u32 trig = 0, us;
	ktime_t t0;
	int ret;

	if (d->pb_done || READ_ONCE(d->stopping))
		return;
	d->pb_done = true;		/* once, whatever the outcome */
	pr_info(DPU_PB_PFX "start at %llu.%03llu s: target %u, DECON FRAME_COUNT=%u TE=%d\n",
		ms / 1000, ms % 1000, d->pb_target, dpu_hw_frame_count(d),
		atomic_read(&d->te_count));

	ret = dpu_cmd_window_begin(d, &trig);
	if (ret) {
		pr_err(DPU_PB_PFX "RESULT FAIL: no command window (%d), nothing sent\n", ret);
		return;
	}
	t0 = ktime_get();
	ret = dpu_dsim_check(d);
	if (!ret) {
		dpu_dsim_clear_status(d);
		ret = pb_sequence(d);
	}
	us = ktime_us_delta(ktime_get(), t0);
	dpu_cmd_window_end(d, trig);

	if (!ret)
		pb_publish(d->pb_target);
	pr_info(DPU_PB_PFX "RESULT %s: %d, %u packets sent, %u reads, %u DSIM errors, window %u us, FRAME_COUNT=%u\n",
		ret ? "FAIL" : "PASS", ret, d->dsim_ntx, d->dsim_nrx, d->dsim_nerr, us,
		dpu_hw_frame_count(d));
	if (!ret)
		pr_info(DPU_PB_PFX "the panel should now be at 4.9 brightness %u (720 nit extended HBM); a person must confirm the screen became readable\n",
			d->pb_target);
}
