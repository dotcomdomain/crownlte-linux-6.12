/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Galaxy Note9 (crownlte) S6E3HA8 "crown_a3_s0" panel commands: the
 * panel_bright.c helpers the panel recovery (panel_recovery.c) reuses.
 * Every helper must be called inside a DECON command window
 * (dpu_cmd_window_begin()).
 */
#ifndef __DPU9810_PANEL_BRIGHT_H__
#define __DPU9810_PANEL_BRIGHT_H__

#include "dpu9810.h"

#define PB_HBM_GAMMA_LEN	34	/* S6E3HA8_HBM_GAMMA_LEN */

/* read-only snapshot of the registers the set-bl sequence writes */
struct pb_state {
	u8 ca[PB_HBM_GAMMA_LEN], b1[2], b5[3], f4[2], b8[17];
};

/* the per-panel values of the set-bl sequence (pb_prepare()) */
struct pb_calib {
	u8 gamma[PB_HBM_GAMMA_LEN];	/* CA payload from the HBM gamma B3 */
	u8 et1;				/* ELVSS temp 1, B5[23] */
	struct pb_state before;
};

/*
 * Panel status, read only: the registers 4.9's panel dump reads for its
 * ESD/error checks (see pb_status_read()).
 */
struct pb_status {
	int ret, ret_k;		/* -ENODATA: not read */
	bool keyed;
	u8 pm[3], sm[3], dsie, diag, ee, ea[5];
	u8 ee_k, ea_k[5];
};

extern const u8 pb_key2_on[3], pb_key2_off[3];

int pb_write(struct dpu_dev *d, u8 ofs, const u8 *buf, size_t len);
int pb_read(struct dpu_dev *d, u8 reg, u8 ofs, u8 *buf, size_t len);
void pb_status_init(struct pb_status *s);
int pb_status_read(struct dpu_dev *d, struct pb_status *s);
int pb_status_read_keyed(struct dpu_dev *d, struct pb_status *s);
void pb_status_log(const char *when, const struct pb_status *s);
int pb_prepare(struct dpu_dev *d, struct pb_status *sb, struct pb_calib *c);
int pb_send_bl(struct dpu_dev *d, const struct pb_calib *c);
void pb_readback(struct dpu_dev *d, struct pb_status *sa, const struct pb_calib *c);
void pb_publish(u32 v);

#endif /* __DPU9810_PANEL_BRIGHT_H__ */
