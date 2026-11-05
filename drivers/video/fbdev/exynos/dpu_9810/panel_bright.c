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
 *
 * Panel status (read only, pb_status_read()): the 4.9 panel-dump registers
 * are read at the start and the end of that command window and again at
 * 5.5 s and 25 s in short windows of their own, to tell a panel fault (EE
 * ELVDD/VLOUT3/VLIN1 error) apart from a display path problem.
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

/*
 * The extra shapes of the 4.9 crown_a3_s0 alpm-enter / alpm-exit tables
 * (pb_alpm_cycle()), allowed only while that cycle runs. B1 (LPM AOR),
 * F0 and F7 are already in the brightness shapes above.
 */
static bool pb_alpm_write_allowed(u8 ofs, const u8 *b, size_t len)
{
	switch (b[0]) {
	case 0xFC:			/* level 3 key on / off */
		return !ofs && len == 3 && b[1] == b[2] && (b[1] == 0x5A || b[1] == 0xA5);
	case 0xBB:			/* LPM nit (ALPM layer), LPM voltage, LPM off nit */
		/* Sol review R1 S4: the full row-3 payload only */
		return (!ofs && len == 7 && b[1] == 0x07 && b[2] == 0x0C && b[3] == 0x0B &&
			b[4] == 0x10 && b[5] == 0x4F && b[6] == 0x04) ||
		       (ofs == 0x25 && len == 3 && b[1] == 0x88 && b[2] == 0x80) ||
		       (!ofs && len == 2 && b[1] == 0x03);
	case 0xFD:			/* AVS on */
		return ofs == 0x2B && len == 2 && b[1] == 0x03;
	case 0x53:			/* LPM mode on (02, row 3) / exit (00) */
		return !ofs && len == 2 && (b[1] == 0x00 || b[1] == 0x02);
	case 0xF4:			/* AVC2 off / on */
		return !ofs && len == 6 && b[1] == 0xEB && b[2] == 0x23 && b[3] == 0x0B &&
		       b[4] == 0x17 && (b[5] == 0x8A || b[5] == 0xCA);
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
	/* status, as 4.9's panel dump (see pb_status_read()) */
	case MIPI_DCS_GET_SIGNAL_MODE:		/* 0x0E */
	case MIPI_DCS_GET_ERROR_COUNT_ON_DSI:	/* 0x05 */
	case MIPI_DCS_GET_DIAGNOSTIC_RESULT:	/* 0x0F */
	case 0xEE: case 0xEA:
		return true;
	default:
		return false;
	}
}

/* 4.9 common_mipi_lcd.c mipi_write(), without the reset-and-retry loop */
static int pb_write(struct dpu_dev *d, u8 ofs, const u8 *buf, size_t len)
{
	int ret;

	if (!len || !(pb_write_allowed(ofs, buf, len) ||
		      (d->pb_alpm_active && pb_alpm_write_allowed(ofs, buf, len)))) {
		pr_err(DPU_PB_PFX "write %*ph (ofs %#x) not in the 4.9 brightness / alpm sequences, refused\n",
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

/*
 * Panel status, read only: the registers 4.9's panel dump reads for its
 * ESD/error checks (s6e3ha8.h "for panel dump", crown_a3_s0_dump_cmdtbl;
 * decoding as s6e3ha8.c show_rddpm() .. show_self_diag()):
 *   0A RDDPM 3 bytes, 0E RDDSM 3, 05 DSI error count 1, 0F self-diagnostic
 *   1, EE ERR_FG 1 (bit 2 VLOUT3, bit 3 ELVDD, bit 6 VLIN1 error), EA DSI
 *   error flags + CRC/ECC counts 5.
 * 4.9 reads them with its level 1-3 keys on. 0A/0E/05/0F are standard DCS
 * reads; EE and EA are manufacturer registers, so they are read both
 * without a key and, where this file already holds the level 2 key, with
 * it. The counters in 05 and EA may reset when read, as with 4.9's reads.
 */
struct pb_status {
	int ret, ret_k;		/* -ENODATA: not read */
	bool keyed;
	u8 pm[3], sm[3], dsie, diag, ee, ea[5];
	u8 ee_k, ea_k[5];
};

static void pb_status_init(struct pb_status *s)
{
	memset(s, 0, sizeof(*s));
	s->ret = -ENODATA;
}

static int pb_status_read(struct dpu_dev *d, struct pb_status *s)
{
	int ret;

	ret = pb_read(d, MIPI_DCS_GET_POWER_MODE, 0, s->pm, sizeof(s->pm));
	if (!ret)
		ret = pb_read(d, MIPI_DCS_GET_SIGNAL_MODE, 0, s->sm, sizeof(s->sm));
	if (!ret)
		ret = pb_read(d, MIPI_DCS_GET_ERROR_COUNT_ON_DSI, 0, &s->dsie, 1);
	if (!ret)
		ret = pb_read(d, MIPI_DCS_GET_DIAGNOSTIC_RESULT, 0, &s->diag, 1);
	if (!ret)
		ret = pb_read(d, 0xEE, 0, &s->ee, 1);
	if (!ret)
		ret = pb_read(d, 0xEA, 0, s->ea, sizeof(s->ea));
	s->ret = ret;
	return ret;
}

/* EE and EA again; the caller holds the level 2 key */
static int pb_status_read_keyed(struct dpu_dev *d, struct pb_status *s)
{
	int ret = pb_read(d, 0xEE, 0, &s->ee_k, 1);

	if (!ret)
		ret = pb_read(d, 0xEA, 0, s->ea_k, sizeof(s->ea_k));
	s->keyed = true;
	s->ret_k = ret;
	return ret;
}

static const char *pb_ee_str(u8 v, char *buf, size_t sz)
{
	if (!(v & 0x4c))
		return "GOOD";
	scnprintf(buf, sz, "NG:%s%s%s", v & 0x08 ? " ELVDD" : "",
		  v & 0x04 ? " VLOUT3" : "", v & 0x40 ? " VLIN1" : "");
	return buf;
}

/* printed after the command window closed */
static void pb_status_log(const char *when, const struct pb_status *s)
{
	char ee[32], eek[32];
	u8 pm = s->pm[0];

	/* Sol review S1: the keyed EE/EA below are logged whatever this says */
	if (s->ret)
		pr_info(DPU_PB_PFX "status %s: %s (%d)\n", when,
			s->ret == -ENODATA ? "not read" : "read failed", s->ret);
	else
		pr_info(DPU_PB_PFX "status %s: 0A %*ph (%s: booster %s, idle %s, partial %s, sleep %s, normal %s, display %s) 0E %*ph (TE %s) 05 %#04x (DSI errors) 0F %#04x (OTP load %s) EE %#04x (%s) EA %*ph (flags %#06x CRC %u ECC1 %u ECC2 %u)\n",
			when, 3, s->pm, pm == 0x9c ? "GOOD" : "not 9c", pm & 0x80 ? "on" : "OFF",
			pm & 0x40 ? "ON" : "off", pm & 0x20 ? "on" : "off",
			pm & 0x10 ? "out" : "IN", pm & 0x08 ? "on" : "OFF",
			pm & 0x04 ? "on" : "OFF", 3, s->sm, s->sm[0] & 0x80 ? "on" : "OFF",
			s->dsie, s->diag, s->diag & 0x80 ? "ok" : "ERROR", s->ee,
			pb_ee_str(s->ee, ee, sizeof(ee)), 5, s->ea, (s->ea[0] << 8) | s->ea[1],
			s->ea[2], s->ea[3], s->ea[4]);
	if (!s->keyed)
		return;
	if (s->ret_k)
		pr_info(DPU_PB_PFX "status %s, level 2 key: failed (%d)\n", when, s->ret_k);
	else
		pr_info(DPU_PB_PFX "status %s, level 2 key: EE %#04x (%s) EA %*ph (flags %#06x CRC %u ECC1 %u ECC2 %u)\n",
			when, s->ee_k, pb_ee_str(s->ee_k, eek, sizeof(eek)), 5, s->ea_k,
			(s->ea_k[0] << 8) | s->ea_k[1], s->ea_k[2], s->ea_k[3], s->ea_k[4]);
}

/* later status samples, each in its own short command window */
static const u32 pb_status_at_ms[] = { 5500, 25000 };

void dpu_panel_status_schedule(struct dpu_dev *d)
{
	u64 ms = ktime_to_ms(ktime_get_boottime());
	u32 at;

	if (READ_ONCE(d->stopping) || d->pb_nstatus >= ARRAY_SIZE(pb_status_at_ms))
		return;
	at = pb_status_at_ms[d->pb_nstatus];
	queue_delayed_work(system_long_wq, &d->pb_status_work,
			   msecs_to_jiffies(at > ms ? at - ms : 0));
}

/*
 * Without "samsung,panel-status-key2" in DT only the reads are sent. With
 * it, EE and EA are read again between F0 5A 5A and F0 A5 A5, the level 2
 * key packets the brightness sequence itself sends.
 */
void dpu_panel_status_work(struct work_struct *work)
{
	struct dpu_dev *d = container_of(to_delayed_work(work), struct dpu_dev, pb_status_work);
	u64 ms = ktime_to_ms(ktime_get_boottime());
	struct pb_status s;
	int ret, key_off = 0;
	char when[24];
	u32 trig = 0;

	if (READ_ONCE(d->stopping))
		return;
	pb_status_init(&s);
	snprintf(when, sizeof(when), "at %llu.%03llu s", ms / 1000, ms % 1000);
	ret = dpu_cmd_window_begin(d, &trig);
	if (ret) {
		pr_info(DPU_PB_PFX "status %s: no command window (%d), nothing sent\n", when, ret);
		goto next;
	}
	ret = dpu_dsim_check(d);
	if (ret) {
		s.ret = ret;
	} else {
		dpu_dsim_clear_status(d);
		pb_status_read(d, &s);
		if (d->pb_status_key2) {
			/*
			 * Sol review S2: a failed key on may still have reached
			 * the panel, so key off follows every key-on attempt.
			 */
			ret = pb_write(d, 0, pb_key2_on, sizeof(pb_key2_on));
			if (ret) {
				s.keyed = true;
				s.ret_k = ret;
			} else {
				pb_status_read_keyed(d, &s);
			}
			key_off = pb_write(d, 0, pb_key2_off, sizeof(pb_key2_off));
		}
	}
	dpu_cmd_window_end(d, trig);
	pb_status_log(when, &s);
	if (key_off)
		pr_err(DPU_PB_PFX "status %s: could not send level 2 key off (%d)\n",
		       when, key_off);
next:
	d->pb_nstatus++;
	dpu_panel_status_schedule(d);
}

static int pb_sequence(struct dpu_dev *d, struct pb_status *sb, struct pb_status *sa)
{
	struct pb_pkt pkt[16];
	struct pb_state before, after;
	u8 rddpm, id[3], hbm[PB_HBM_GAMMA_LEN], gamma[PB_HBM_GAMMA_LEN], et1;
	int i, n, ret, ret2 = 0;
	bool key2 = false;

	/* 0. status before anything else (a failed status read stops nothing) */
	pb_status_read(d, sb);

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
	 * resource init re-reads the HBM gamma. A failed key on may still
	 * have reached the panel, so key off follows every key-on attempt
	 * (as dpu_panel_status_work()).
	 */
	key2 = true;
	ret = pb_write(d, 0, pb_key2_on, sizeof(pb_key2_on));
	if (!ret)
		ret = pb_read(d, 0xB3, PB_HBM_GAMMA_OFS, hbm, sizeof(hbm));
	if (!ret)
		ret = pb_read(d, 0xB5, PB_ELVSS_TEMP_1_OFS, &et1, 1);
	if (!ret)
		ret = pb_snapshot(d, &before);
	if (!ret)
		pb_status_read_keyed(d, sb);	/* inside the existing key window */
	ret2 = pb_write(d, 0, pb_key2_off, sizeof(pb_key2_off));
	if (!ret2)
		key2 = false;
	if (ret || ret2)
		goto fail;

	pb_hbm_to_gamma(hbm, gamma);
	pb_dbg("HBM gamma (B3[2..35]) %*ph -> CA payload %*ph\n",
		PB_HBM_GAMMA_LEN, hbm, PB_HBM_GAMMA_LEN, gamma);
	pb_dbg("ELVSS temp 1 (B5[23]) %#04x\n", et1);
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
	pb_dbg("sending the 4.9 PANEL_SET_BL_SEQ for %u (%d packets)\n",
		d->pb_target, n);
	/* packet 0 is the key on: from its attempt on, key off is owed */
	key2 = true;
	for (i = 0; i < n; i++) {
		pb_dbg("step %d/%d: %s\n", i + 1, n, pkt[i].what);
		ret = pb_write(d, pkt[i].ofs, pkt[i].data, pkt[i].len);
		if (ret)
			break;
	}
	if (ret)
		goto fail;
	key2 = false;			/* the last packet was the key off */

	/* 5. read back (diagnostic only); key off after every key-on attempt */
	key2 = true;
	ret = pb_write(d, 0, pb_key2_on, sizeof(pb_key2_on));
	if (!ret) {
		ret = pb_snapshot(d, &after);
		if (!ret)
			pb_status_read_keyed(d, sa);	/* inside the existing key window */
	}
	if (!pb_write(d, 0, pb_key2_off, sizeof(pb_key2_off)) ||
	    !pb_write(d, 0, pb_key2_off, sizeof(pb_key2_off)))
		key2 = false;
	if (key2)
		pr_err(DPU_PB_PFX "could not send level 2 key off after the read-back\n");
	else
		pb_status_read(d, sa);		/* status after the sequence */
	if (ret) {
		d->pb_readback = "FAILED";
		pb_dbg("read-back failed (%d); the sequence itself completed\n", ret);
	} else {
		bool ok = !memcmp(after.ca, gamma, sizeof(gamma)) &&
			  after.b1[0] == 0x00 && after.b1[1] == 0x0C &&
			  after.b5[0] == 0x19 && after.b5[1] == 0xDC && after.b5[2] == 0x00 &&
			  after.f4[0] == 0xEB && after.f4[1] == 0x26 &&
			  !memcmp(after.b8, pb_irc_value + 1, sizeof(after.b8));

		d->pb_readback = ok ? "MATCH" : "MISMATCH";
		pb_dbg("read-back %s: CA %s, B1 %*ph -> %*ph, B5 %*ph -> %*ph, F4 %*ph -> %*ph, B8[18..] %*ph -> %*ph\n",
			ok ? "MATCH" : "MISMATCH",
			memcmp(after.ca, gamma, sizeof(gamma)) ? "differs" : "matches",
			2, before.b1, 2, after.b1, 3, before.b5, 3, after.b5,
			2, before.f4, 2, after.f4, 17, before.b8, 17, after.b8);
		pb_dbg("CA before %*ph\n", PB_HBM_GAMMA_LEN, before.ca);
		pb_dbg("CA after  %*ph\n", PB_HBM_GAMMA_LEN, after.ca);
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
	/* off by default: level 2 key around the EE/EA reads at 5.5 s / 25 s */
	d->pb_status_key2 = of_property_read_bool(np, "samsung,panel-status-key2");
	/* off by default: one 4.9 fix_green() ALPM cycle (dpu_panel_alpm_work) */
	d->pb_alpm_at_ms = 0;
	of_property_read_u32(np, "samsung,alpm-cycle-at-ms", &d->pb_alpm_at_ms);

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
	struct pb_status sb, sa;
	u32 trig = 0, us;
	ktime_t t0;
	int ret;

	if (d->pb_done || READ_ONCE(d->stopping))
		return;
	d->pb_done = true;		/* once, whatever the outcome */
	pb_status_init(&sb);
	pb_status_init(&sa);
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
	d->pb_readback = "not run";
	if (!ret) {
		dpu_dsim_clear_status(d);
		ret = pb_sequence(d, &sb, &sa);
	}
	us = ktime_us_delta(ktime_get(), t0);
	dpu_cmd_window_end(d, trig);
	pb_status_log("before brightness", &sb);
	pb_status_log("after brightness", &sa);

	if (!ret)
		pb_publish(d->pb_target);
	/* one console line: everything else is KERN_DEBUG (see pb_dbg) */
	pr_info(DPU_PB_PFX "RESULT %s: %d, brightness %u, read-back %s, %u tx / %u rx, %u DSIM errors, %u notes, window %u us, max tx %u us, max rx %u us, longest TE irq interval %u us\n",
		ret ? "FAIL" : "PASS", ret, ret ? 0 : d->pb_target, d->pb_readback,
		d->dsim_ntx, d->dsim_nrx, d->dsim_nerr, d->dsim_nnote, us,
		d->dsim_max_tx_us, d->dsim_max_rx_us, d->te_gap_max_us);
}

/*
 * One 4.9 fix_green() cycle, opt-in via DT "samsung,alpm-cycle-at-ms".
 *
 * The owner's 4.9 kernel carries a "Green Screen Workaround" for this
 * panel (panel_drv.c fix_green(), sysfs fix_green_screen, set to 1 on their
 * Lineage by init.greenfix.rc / persist.vendor.panel.greenfix=1). At every
 * display on (PANEL_IOC_EVT_FRAME_DONE with disp_on off) it runs:
 *   panel_doze(PANEL_IOC_DOZE)  -> __panel_seq_set_alpm() -> PANEL_ALPM_ENTER_SEQ
 *   usleep_range(126 ms)        ("ALPM spec")
 *   panel_sleep_out()           -> __panel_seq_exit_alpm() -> PANEL_ALPM_EXIT_SEQ,
 *                                  then backlight_update_status() -> PANEL_SET_BL_SEQ
 * Packets: s6e3ha8_crown_a3_s0_panel.h crown_a3_s0_alpm_enter_cmdtbl and
 * crown_a3_s0_alpm_exit_cmdtbl (CONFIG_ACTIVE_CLOCK off in the owner's
 * build, so no active-clock packets). The LPM rows are those of
 * getidx_lpm_table() as logged by the owner's Lineage 4.9 on screen on
 * ("alpm_mode 0, brightness 44700, layer 0 row 3" = ALPM_60NIT at enter, row 0
 * forced at exit): LPM nit BB 07 0C 0B 10 4F 04, LPM mode 53 02,
 * LPM off nit BB 03.
 * Left out (4.9 does them; all PMIC writes): the S2DOS05 ELVSS short-detect
 * current for doze (CONFIG_DISP_PMIC_SSD) and the panel 1.6 V rail set to
 * 1.5 V in ALPM and back (logged "regulator voltage : 1600000" / "1500000").
 * Also __delay_normal_alpm()'s 124 ms guard after panel on (the panel has
 * been on for seconds here).
 * The question this answers: does the cycle clear the DDI's EE ELVDD error
 * and light a panel that came up black?
 */
struct pb_step {
	u8 ofs, len;
	u8 data[8];
	u32 delay_us;		/* after the packet */
	const char *what;
};

static const struct pb_step pb_alpm_enter[] = {
	{ 0, 3, { 0xF0, 0x5A, 0x5A }, 0, "level 2 key on" },
	{ 0, 3, { 0xB1, 0x0B, 0x74 }, 0, "LPM AOR" },
	{ 0, 2, { 0xF7, 0x03 }, 0, "gamma update" },
	{ 0, 3, { 0xF0, 0xA5, 0xA5 }, 16700, "level 2 key off, 1 frame" },
	{ 0, 3, { 0xF0, 0x5A, 0x5A }, 0, "level 2 key on" },
	{ 0, 7, { 0xBB, 0x07, 0x0C, 0x0B, 0x10, 0x4F, 0x04 }, 0, "LPM nit (ALPM 60 nit)" },
	{ 0x25, 3, { 0xBB, 0x88, 0x80 }, 0, "LPM voltage" },
	{ 0, 3, { 0xFC, 0x5A, 0x5A }, 0, "level 3 key on" },
	{ 0x2B, 2, { 0xFD, 0x03 }, 0, "AVS on" },
	{ 0, 3, { 0xFC, 0xA5, 0xA5 }, 0, "level 3 key off" },
	{ 0, 2, { 0x53, 0x02 }, 0, "LPM mode (ALPM 60 nit)" },
	{ 0, 6, { 0xF4, 0xEB, 0x23, 0x0B, 0x17, 0x8A }, 0, "AVC2 off" },
	{ 0, 2, { 0xF7, 0x03 }, 0, "gamma update" },
	{ 0, 3, { 0xF0, 0xA5, 0xA5 }, 0, "level 2 key off" },
};

static const struct pb_step pb_alpm_exit[] = {
	{ 0, 3, { 0xF0, 0x5A, 0x5A }, 0, "level 2 key on" },
	{ 0, 2, { 0xBB, 0x03 }, 0, "LPM off nit" },
	{ 0, 6, { 0xF4, 0xEB, 0x23, 0x0B, 0x17, 0xCA }, 0, "AVC2 on" },
	{ 0, 2, { 0x53, 0x00 }, 0, "exit ALPM" },
	{ 0, 3, { 0xF0, 0xA5, 0xA5 }, 0, "level 2 key off" },
};

/*
 * Send a table. Entry (best_effort false) stops at the first error; exit
 * (best_effort true, Sol review R1 S1) still tries every packet so that
 * "53 00" goes out after an earlier error. On any error both key-off
 * packets follow (a failed key on may still have reached the panel).
 * *attempted counts packets handed to the DSIM, failed ones included (R1
 * S2: a timed-out packet may still have reached the panel). Returns the
 * first error.
 */
static int pb_steps(struct dpu_dev *d, const struct pb_step *st, int n, bool best_effort,
		    int *attempted)
{
	static const u8 key3_off[] = { 0xFC, 0xA5, 0xA5 };
	int i, r, ret = 0;

	for (i = 0; i < n; i++) {
		pb_dbg("alpm step %d/%d: %s\n", i + 1, n, st[i].what);
		*attempted = i + 1;
		r = pb_write(d, st[i].ofs, st[i].data, st[i].len);
		if (r) {
			pr_err(DPU_PB_PFX "alpm step %d/%d (%s): %d\n", i + 1, n, st[i].what, r);
			if (!ret)
				ret = r;
			if (!best_effort)
				break;
			continue;
		}
		if (st[i].delay_us)
			usleep_range(st[i].delay_us, st[i].delay_us + 100);
	}
	if (ret) {
		if (pb_write(d, 0, key3_off, sizeof(key3_off)))
			pr_err(DPU_PB_PFX "alpm: could not send level 3 key off\n");
		if (pb_write(d, 0, pb_key2_off, sizeof(pb_key2_off)))
			pr_err(DPU_PB_PFX "alpm: could not send level 2 key off\n");
	}
	return ret;
}

/* one window: optional status, a packet table, then RDDPM */
static int pb_alpm_window(struct dpu_dev *d, const struct pb_step *st, int n, bool best_effort,
			  struct pb_status *s, u8 *pm, int *attempted)
{
	u32 trig = 0;
	int ret, ret_pm;

	*attempted = 0;
	ret = dpu_cmd_window_begin(d, &trig);
	if (ret)
		return ret;
	ret = dpu_dsim_check(d);
	if (!ret) {
		dpu_dsim_clear_status(d);
		if (s)
			pb_status_read(d, s);
		d->pb_alpm_active = true;
		ret = pb_steps(d, st, n, best_effort, attempted);
		d->pb_alpm_active = false;
		ret_pm = pb_read(d, MIPI_DCS_GET_POWER_MODE, 0, pm, 1);
		if (ret_pm)
			*pm = 0;
	}
	dpu_cmd_window_end(d, trig);
	return ret;
}

void dpu_panel_alpm_work(struct work_struct *work)
{
	struct dpu_dev *d = container_of(to_delayed_work(work), struct dpu_dev, pb_alpm_work);
	u64 ms = ktime_to_ms(ktime_get_boottime());
	struct pb_status s0, sb, sa;
	u8 pm_in = 0, pm_out = 0;
	int ret, ret_exit, ret_bl = -ENODATA, n_in, n_out, tries;
	u32 trig = 0;

	if (READ_ONCE(d->stopping))
		return;
	if (READ_ONCE(pb_reported) != d->pb_target) {
		pr_info(DPU_PB_PFX "alpm cycle at %llu.%03llu s: skipped, brightness %u not applied\n",
			ms / 1000, ms % 1000, d->pb_target);
		return;
	}
	pb_status_init(&s0);
	pb_status_init(&sb);
	pb_status_init(&sa);
	pr_info(DPU_PB_PFX "alpm cycle (4.9 fix_green) start at %llu.%03llu s\n",
		ms / 1000, ms % 1000);

	/* 1. ALPM enter */
	ret = pb_alpm_window(d, pb_alpm_enter, ARRAY_SIZE(pb_alpm_enter), false, &s0, &pm_in,
			     &n_in);
	pb_status_log("before alpm", &s0);
	pr_info(DPU_PB_PFX "alpm enter: %d, %d/%zu packets attempted, RDDPM %#04x (idle %s)\n",
		ret, n_in, ARRAY_SIZE(pb_alpm_enter), pm_in, pm_in & 0x40 ? "ON" : "off");
	/* no enter packet was even attempted: nothing to undo */
	if (!n_in)
		return;

	/* 2. 4.9 fix_green(): 126 ms (ALPM spec), frames keep running */
	usleep_range(126 * 1000, 126 * 1000 + 10);

	/*
	 * 3. ALPM exit (every packet tried), then 4.9's backlight_update_status():
	 * the set-bl sequence, also when stopping (Sol review R1 S3: the enter
	 * changed AOR, which only the set-bl sequence restores, and dpu_stop()
	 * leaves the display running; cancel_delayed_work_sync() waits for it).
	 * pb_sequence() itself refuses unless RDDPM shows normal display.
	 */
	/*
	 * Sol review R2: the whole exit table again (bounded) until it went out
	 * without an error and RDDPM confirms normal display (sleep out,
	 * normal, display on, no idle/partial: the pb_sequence() check). The
	 * first error is kept for the report.
	 */
	ret_exit = 0;
	for (tries = 0; tries < 5; tries++) {
		int r;

		pm_out = 0;
		r = pb_alpm_window(d, pb_alpm_exit, ARRAY_SIZE(pb_alpm_exit), true, NULL,
				   &pm_out, &n_out);
		if (r && !ret_exit)
			ret_exit = r;
		if (!r && (pm_out & 0x7c) == 0x1c)
			break;
		pr_err(DPU_PB_PFX "alpm exit try %d: %d, %d/%zu packets attempted, RDDPM %#04x%s\n",
		       tries + 1, r, n_out, ARRAY_SIZE(pb_alpm_exit), pm_out,
		       tries < 4 ? ", again" : ", giving up");
		msleep(20);
	}
	pr_info(DPU_PB_PFX "alpm exit: first error %d, %d tries, RDDPM %#04x (idle %s)\n",
		ret_exit, min(tries + 1, 5), pm_out, pm_out & 0x40 ? "ON" : "off");
	if (!dpu_cmd_window_begin(d, &trig)) {
		ret_bl = dpu_dsim_check(d);
		if (!ret_bl) {
			dpu_dsim_clear_status(d);
			ret_bl = pb_sequence(d, &sb, &sa);
		}
		dpu_cmd_window_end(d, trig);
		pb_status_log("after alpm, before brightness", &sb);
		pb_status_log("after alpm and brightness", &sa);
	}
	pr_info(DPU_PB_PFX "alpm cycle RESULT %s: enter %d, exit %d, brightness %d, read-back %s\n",
		!ret && !ret_exit && !ret_bl ? "PASS" : "FAIL", ret, ret_exit, ret_bl,
		ret_bl ? "-" : d->pb_readback);
}
