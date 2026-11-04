// SPDX-License-Identifier: GPL-2.0
/*
 * Compile-time check of the Exynos9810 DECON fbdev ioctl ABI against the
 * values the LineageOS 23 vendor blobs were built with
 * (roadmap/vendor-abi/raw/display-ioctls.txt: ioctl constants recovered
 * from libexynosdisplay.so and gralloc.universal9810.so) and against the
 * 4.9 arm64 structure layout.
 */
#include <linux/build_bug.h>
#include <linux/fb.h>
#include <linux/stddef.h>
#include <uapi/video/exynos9810_decon.h>

/* struct sizes (4.9 arm64, CONFIG_SUPPORT_DSU=y) */
static_assert(sizeof(struct decon_win_rect) == 16);
static_assert(sizeof(struct decon_frame) == 24);
static_assert(sizeof(struct decon_dpp_params) == 48);
static_assert(sizeof(struct decon_win_config) == 200);
static_assert(offsetof(struct decon_win_config, color) == 8);
static_assert(offsetof(struct decon_win_config, fd_idma) == 8);
static_assert(offsetof(struct decon_win_config, format) == 40);
static_assert(offsetof(struct decon_win_config, dpp_parm) == 48);
static_assert(offsetof(struct decon_win_config, block_area) == 96);
static_assert(offsetof(struct decon_win_config, src) == 144);
static_assert(offsetof(struct decon_win_config, dst) == 168);
static_assert(offsetof(struct decon_win_config, protection) == 192);
static_assert(offsetof(struct decon_win_config, compression) == 193);
static_assert(sizeof(struct decon_win_config_data_old) == 1416);
static_assert(offsetof(struct decon_win_config_data_old, config) == 16);
static_assert(sizeof(struct decon_win_config_data) == 1648);
static_assert(sizeof(struct decon_disp_info) == 248);
static_assert(sizeof(struct decon_display_mode) == 28);
static_assert(sizeof(struct decon_hdr_capabilities) == 16);
static_assert(sizeof(struct decon_hdr_capabilities_info) == 16);
static_assert(sizeof(struct decon_user_window) == 8);
static_assert(sizeof(struct decon_color_mode_info) == 8);
static_assert(sizeof(struct decon_edid_data) == 516);
static_assert(sizeof(struct vsync_applied_time_data) == 32);
static_assert(sizeof(struct exynos_displayport_data) == 292);

/* ioctl numbers seen in the vendor blobs */
static_assert(S3CFB_WIN_CONFIG_OLD == 0x458846d1);
static_assert(S3CFB_SET_VSYNC_INT == 0x400446ce);
static_assert(S3CFB_DECON_SELF_REFRESH == 0x400446cf);
static_assert(S3CFB_POWER_MODE == 0x400446df);
static_assert(DECON_WIN_CURSOR_POS == 0x400846de);
static_assert(EXYNOS_DISP_INFO == 0x40f84704);
static_assert(EXYNOS_GET_DISPLAYPORT_CONFIG == 0x4124472c);
static_assert(S3CFB_GET_HDR_CAPABILITIES == 0x40104790);
static_assert(EXYNOS_GET_COLOR_MODE_NUM == 0x40044658);
static_assert(EXYNOS_GET_COLOR_MODE == 0x40084659);
static_assert(EXYNOS_SET_COLOR_MODE == 0x4004465a);
static_assert(EXYNOS_GET_DISPLAY_MODE_NUM == 0x400446bc);
static_assert(EXYNOS_GET_DISPLAY_MODE == 0x401c46bd);
static_assert(EXYNOS_SET_DISPLAY_MODE == 0x401c46be);
static_assert(EXYNOS_GET_DISPLAY_CURRENT_MODE == 0x400446c1);
static_assert(EXYNOS_GET_EDID == 0x42044720);
static_assert(EXYNOS_GET_VSYNC_CHANGE_TIMELINE == 0x40204752);
static_assert(FBIO_WAITFORVSYNC == 0x40044620);
/* the newer 1648-byte layout has a different number; both are accepted */
static_assert(S3CFB_WIN_CONFIG == 0x467046d1);
