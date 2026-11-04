/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Exynos9810 DECON fbdev ioctl ABI, as used by the LineageOS 23 vendor
 * HWC (hwc3-service.slsi / libexynosdisplay.so) and gralloc on the Galaxy
 * Note9 (crownlte).
 *
 * Byte-compatible with the 4.9 Samsung kernel
 * (drivers/video/fbdev/exynos/dpu_9810/decon.h, CONFIG_SUPPORT_DSU=y,
 * arm64). The 4.9 structures use kernel types (dma_addr_t, bool, enums);
 * they are spelled here with fixed-width types and explicit padding.
 * Only the LP64 (arm64) layout is defined; 32-bit clients are not
 * supported (4.9 had no compat handler either).
 *
 * The ioctl numbers above 255 overflow into the type byte, exactly as on
 * 4.9 (e.g. EXYNOS_DISP_INFO = _IOW('F', 260, ...) = 0x40f84704).
 */
#ifndef _UAPI_VIDEO_EXYNOS9810_DECON_H
#define _UAPI_VIDEO_EXYNOS9810_DECON_H

#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/videodev2.h>

#define DECON_MAX_WIN			6
#define DECON_MAX_PLANE_CNT		3
/* config[] entries: windows + window-update slot (+ DSU slot, new layout) */
#define DECON_WIN_CONFIG_CNT_OLD	(DECON_MAX_WIN + 1)
#define DECON_WIN_CONFIG_CNT		(DECON_MAX_WIN + 2)
#define DECON_WIN_UPDATE_IDX		DECON_MAX_WIN

/* decon_win_config.state */
#define DECON_WIN_STATE_DISABLED	0
#define DECON_WIN_STATE_COLOR		1
#define DECON_WIN_STATE_BUFFER		2
#define DECON_WIN_STATE_UPDATE		3
#define DECON_WIN_STATE_CURSOR		4
#define DECON_WIN_STATE_MRESOL		0x10000

/* enum decon_blending */
#define DECON_BLENDING_NONE		0
#define DECON_BLENDING_PREMULT		1
#define DECON_BLENDING_COVERAGE		2

/* enum decon_idma_type (= DPP alias index) */
#define DECON_IDMA_G0			0
#define DECON_IDMA_G1			1
#define DECON_IDMA_VG0			2
#define DECON_IDMA_VG1			3
#define DECON_IDMA_VGF0			4
#define DECON_IDMA_VGF1			5	/* VGRF on Exynos9810 */

/* enum decon_pixel_format (subset; values are positional as on 4.9) */
#define DECON_PIXEL_FORMAT_ARGB_8888	0
#define DECON_PIXEL_FORMAT_ABGR_8888	1
#define DECON_PIXEL_FORMAT_RGBA_8888	2
#define DECON_PIXEL_FORMAT_BGRA_8888	3
#define DECON_PIXEL_FORMAT_XRGB_8888	4
#define DECON_PIXEL_FORMAT_XBGR_8888	5
#define DECON_PIXEL_FORMAT_RGBX_8888	6
#define DECON_PIXEL_FORMAT_BGRX_8888	7
#define DECON_PIXEL_FORMAT_RGBA_5551	8
#define DECON_PIXEL_FORMAT_BGRA_5551	9
#define DECON_PIXEL_FORMAT_ABGR_4444	10
#define DECON_PIXEL_FORMAT_RGBA_4444	11
#define DECON_PIXEL_FORMAT_BGRA_4444	12
#define DECON_PIXEL_FORMAT_RGB_565	13
#define DECON_PIXEL_FORMAT_BGR_565	14
#define DECON_PIXEL_FORMAT_ARGB_2101010	15
#define DECON_PIXEL_FORMAT_ABGR_2101010	16
#define DECON_PIXEL_FORMAT_RGBA_1010102	17
#define DECON_PIXEL_FORMAT_BGRA_1010102	18
#define DECON_PIXEL_FORMAT_NV16		19
#define DECON_PIXEL_FORMAT_NV61		20
#define DECON_PIXEL_FORMAT_YVU422_3P	21
#define DECON_PIXEL_FORMAT_NV12		22
#define DECON_PIXEL_FORMAT_NV21		23
#define DECON_PIXEL_FORMAT_NV12M	24
#define DECON_PIXEL_FORMAT_NV21M	25
#define DECON_PIXEL_FORMAT_YUV420	26
#define DECON_PIXEL_FORMAT_YVU420	27
#define DECON_PIXEL_FORMAT_YUV420M	28
#define DECON_PIXEL_FORMAT_YVU420M	29
#define DECON_PIXEL_FORMAT_NV12N	30
#define DECON_PIXEL_FORMAT_NV12N_10B	31
#define DECON_PIXEL_FORMAT_NV12M_P010	32
#define DECON_PIXEL_FORMAT_NV21M_P010	33
#define DECON_PIXEL_FORMAT_NV12M_S10B	34
#define DECON_PIXEL_FORMAT_NV21M_S10B	35
#define DECON_PIXEL_FORMAT_MAX		36

struct decon_win_rect {
	__s32 x;
	__s32 y;
	__u32 w;
	__u32 h;
};

struct decon_rect {
	__u32 left;
	__u32 top;
	__u32 right;
	__u32 bottom;
};

struct decon_frame {
	__s32 x;
	__s32 y;
	__u32 w;
	__u32 h;
	__u32 f_w;
	__u32 f_h;
};

/* 4.9 struct dpp_params; addr[] was dma_addr_t (ignored on input) */
struct decon_dpp_params {
	__u64 addr[DECON_MAX_PLANE_CNT];
	__u32 rot;		/* enum dpp_rotate */
	__u32 eq_mode;		/* enum dpp_csc_eq */
	__u32 comp_src;		/* enum dpp_comp_src */
	__u32 hdr_std;		/* enum dpp_hdr_standard */
	__u32 min_luminance;
	__u32 max_luminance;
};

struct decon_win_config {
	__u32 state;
	__u32 __pad0;		/* union is 8-byte aligned (dma_addr_t) */
	union {
		__u32 color;
		struct {
			__s32 fd_idma[DECON_MAX_PLANE_CNT];
			__s32 acq_fence;
			__s32 rel_fence;
			__s32 plane_alpha;
			__u32 blending;
			__u32 idma_type;
			__u32 format;
			__u32 __pad1;
			struct decon_dpp_params dpp_parm;
			struct decon_win_rect block_area;
			struct decon_win_rect transparent_area;
			struct decon_win_rect opaque_area;
			struct decon_frame src;
		};
		struct {		/* DSU (state DECON_WIN_STATE_MRESOL) */
			__s32 left;
			__s32 top;
			__s32 right;
			__s32 bottom;
			__s32 mode;
		};
	};
	struct decon_frame dst;
	__u8 protection;
	__u8 compression;
	__u8 __pad2[6];
};

struct decon_win_config_extra {
	__s32 remained_frames;
	__u32 reserved[7];
};

/* S3CFB_WIN_CONFIG_OLD: the 1416-byte layout the vendor HWC issues */
struct decon_win_config_data_old {
	__s32 retire_fence;
	__s32 fd_odma;
	__u32 fps;
	__u32 __pad0;
	struct decon_win_config config[DECON_WIN_CONFIG_CNT_OLD];
};

struct decon_win_config_data {
	__s32 retire_fence;
	__s32 fd_odma;
	__u32 fps;
	__u32 __pad0;
	struct decon_win_config config[DECON_WIN_CONFIG_CNT];
	struct decon_win_config_extra extra;
};

#define DECON_MAX_RES_NUMBER		5

struct decon_lcd_res_info {
	__u32 width;
	__u32 height;
	__u32 dsc_en;
	__u32 dsc_width;
	__u32 dsc_height;
};

struct decon_lcd_mres_info {
	__u32 mres_en;
	__u32 mres_number;
	struct decon_lcd_res_info res_info[DECON_MAX_RES_NUMBER];
};

struct decon_disp_info {
	__u32 ver;		/* enum hwc_ver */
	__u32 psr_mode;		/* enum decon_psr_mode */
	struct decon_lcd_mres_info mres_info;
	__u32 chip_ver;		/* 9810 */
	__u8 reserved[128];
};

struct decon_display_mode {
	__u32 index;
	__u32 width;
	__u32 height;
	__u32 mm_width;
	__u32 mm_height;
	__u32 fps;
	__u32 group;
};

#define DECON_HDR_CAPA_NUM		4

struct decon_hdr_capabilities {
	__u32 out_types[DECON_HDR_CAPA_NUM];
};

struct decon_hdr_capabilities_info {
	__s32 out_num;
	__s32 max_luminance;
	__s32 max_average_luminance;
	__s32 min_luminance;
};

struct decon_user_window {
	__s32 x;
	__s32 y;
};

struct decon_color_mode_info {
	__s32 index;
	__u32 color_mode;
};

#define DECON_EDID_BLOCK_SIZE		128
#define DECON_MAX_EDID_BLOCK		4

struct decon_edid_data {
	__s32 size;
	__u8 edid_data[DECON_EDID_BLOCK_SIZE * DECON_MAX_EDID_BLOCK];
};

struct vsync_applied_time_data {
	__u32 config;
	__u32 __pad0;
	__u64 time;
	__u32 reserved[4];
};

struct exynos_displayport_data {
	__u32 state;
	struct v4l2_dv_timings timings;
	struct v4l2_enum_dv_timings etimings;
	__u32 audio_info;
	__s32 hdcp;
};

#define S3CFB_SET_VSYNC_INT		_IOW('F', 206, __u32)
#define S3CFB_DECON_SELF_REFRESH	_IOW('F', 207, __u32)
#define S3CFB_WIN_CONFIG_OLD		_IOW('F', 209, struct decon_win_config_data_old)
#define S3CFB_WIN_CONFIG		_IOW('F', 209, struct decon_win_config_data)
#define DECON_WIN_CURSOR_POS		_IOW('F', 222, struct decon_user_window)
#define S3CFB_POWER_MODE		_IOW('F', 223, __u32)
#define EXYNOS_DISP_INFO		_IOW('F', 260, struct decon_disp_info)
#define S3CFB_START_CRC			_IOW('F', 270, __u32)
#define S3CFB_SEL_CRC_BITS		_IOW('F', 271, __u32)
#define S3CFB_GET_CRC_DATA		_IOR('F', 272, __u32)
#define EXYNOS_GET_DISPLAYPORT_CONFIG	_IOW('F', 300, struct exynos_displayport_data)
#define EXYNOS_SET_DISPLAYPORT_CONFIG	_IOW('F', 301, struct exynos_displayport_data)
#define EXYNOS_DPU_DUMP			_IOW('F', 302, struct decon_win_config_data)
#define S3CFB_GET_HDR_CAPABILITIES	_IOW('F', 400, struct decon_hdr_capabilities)
#define S3CFB_GET_HDR_CAPABILITIES_NUM	_IOW('F', 401, struct decon_hdr_capabilities_info)
#define EXYNOS_DPU_GET_ACLK		_IOR('F', 500, __u32)
#define EXYNOS_GET_COLOR_MODE_NUM	_IOW('F', 600, __u32)
#define EXYNOS_GET_COLOR_MODE		_IOW('F', 601, struct decon_color_mode_info)
#define EXYNOS_SET_COLOR_MODE		_IOW('F', 602, __u32)
#define EXYNOS_GET_DISPLAY_MODE_NUM	_IOW('F', 700, __u32)
#define EXYNOS_GET_DISPLAY_MODE		_IOW('F', 701, struct decon_display_mode)
#define EXYNOS_SET_DISPLAY_MODE		_IOW('F', 702, struct decon_display_mode)
#define EXYNOS_GET_DISPLAY_CURRENT_MODE	_IOW('F', 705, __u32)
#define EXYNOS_GET_EDID			_IOW('F', 800, struct decon_edid_data)
#define EXYNOS_GET_VSYNC_CHANGE_TIMELINE _IOW('F', 850, struct vsync_applied_time_data)

/* S3CFB_POWER_MODE arguments (4.9 enum disp_pwr_mode, CONFIG_SUPPORT_DOZE=y) */
#define DISP_PWR_OFF			0
#define DISP_PWR_DOZE			1
#define DISP_PWR_NORMAL			2
#define DISP_PWR_DOZE_SUSPEND		3

#endif /* _UAPI_VIDEO_EXYNOS9810_DECON_H */
