/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Exynos9810 DSIM register subset used by the adopt-mode command path.
 *
 * Copied from the Samsung 4.9 kernel,
 * drivers/video/fbdev/exynos/dpu_9810/regs-dsim.h (offsets and bit
 * positions unchanged). Only the link-status, FIFO, packet and interrupt
 * status registers are listed: this driver never programs DSIM timing,
 * clocks, the PHY or the link configuration. The one exception is
 * CLK_CTRL.TX_REQUEST_HSCLK around the panel recovery's reset pulse
 * (dsim_cmd.c dpu_dsim_enter_lp11(), as 4.9 dsim_reg_set_hs_clock()).
 */
#ifndef _REGS_DSIM_9810_H
#define _REGS_DSIM_9810_H

#define DSIM_SWRST					(0x4)
#define DSIM_SWRST_FUNCRST				(1 << 8)
#define DSIM_SWRST_RESET				(1 << 0)

#define DSIM_LINK_STATUS0				(0x8)
#define DSIM_LINK_STATUS1				(0xc)
#define DSIM_LINK_STATUS3				(0x14)
#define DSIM_LINK_STATUS3_PLL_STABLE			(1 << 0)

#define DSIM_MIPI_STATUS				(0x18)

#define DSIM_DPHY_STATUS				(0x1c)
#define DSIM_DPHY_STATUS_TX_READY_HS_CLK		(1 << 10)
#define DSIM_DPHY_STATUS_ULPS_CLK			(1 << 9)
#define DSIM_DPHY_STATUS_STOP_STATE_CLK			(1 << 8)
#define DSIM_DPHY_STATUS_ULPS_DATA_LANE_GET(x)		(((x) >> 4) & 0xf)
#define DSIM_DPHY_STATUS_STOP_STATE_DAT(_x)		(((_x) & 0xf) << 0)

#define DSIM_CLK_CTRL					(0x20)
#define DSIM_CLK_CTRL_NONCONT_CLOCK_LANE		(1 << 25)
#define DSIM_CLK_CTRL_TX_REQUEST_HSCLK			(1 << 20)

#define DSIM_TIMEOUT					(0x28)

#define DSIM_ESCMODE					(0x2c)
#define DSIM_ESCMODE_FORCE_STOP_STATE			(1 << 20)
#define DSIM_ESCMODE_FORCE_BTA				(1 << 16)
#define DSIM_ESCMODE_CMD_LPDT				(1 << 7)
#define DSIM_ESCMODE_TX_ULPS_DATA			(1 << 3)
#define DSIM_ESCMODE_TX_ULPS_DATA_EXIT			(1 << 2)
#define DSIM_ESCMODE_TX_ULPS_CLK			(1 << 1)
#define DSIM_ESCMODE_TX_ULPS_CLK_EXIT			(1 << 0)

#define DSIM_CONFIG					(0x4c)
#define DSIM_CONFIG_VIDEO_MODE				(1 << 18)
#define DSIM_CONFIG_LANES_EN(_x)			(((_x) & 0x1f) << 0)

#define DSIM_INTSRC					(0x50)
#define DSIM_INTSRC_SFR_PL_FIFO_EMPTY			(1 << 29)
#define DSIM_INTSRC_SFR_PH_FIFO_EMPTY			(1 << 28)
#define DSIM_INTSRC_SFR_PH_FIFO_OVERFLOW		(1 << 27)
#define DSIM_INTSRC_BUS_TURN_OVER			(1 << 25)
#define DSIM_INTSRC_FRAME_DONE				(1 << 24)
#define DSIM_INTSRC_INVALID_SFR_VALUE			(1 << 23)
#define DSIM_INTSRC_ABNRMAL_CMD_ST			(1 << 22)
#define DSIM_INTSRC_LPRX_TOUT				(1 << 21)
#define DSIM_INTSRC_BTA_TOUT				(1 << 20)
#define DSIM_INTSRC_UNDER_RUN				(1 << 19)
#define DSIM_INTSRC_RX_DATA_DONE			(1 << 18)
#define DSIM_INTSRC_ERR_RX_ECC				(1 << 15)

#define DSIM_INTMSK					(0x54)

/* Packet Header FIFO register */
#define DSIM_PKTHDR					(0x58)
#define DSIM_PKTHDR_BTA_TYPE(_x)			((_x) << 24)
#define DSIM_PKTHDR_DATA1(_x)				((_x) << 16)
#define DSIM_PKTHDR_DATA0(_x)				((_x) << 8)
#define DSIM_PKTHDR_ID(_x)				((_x) << 0)
#define DSIM_PKTHDR_DATA				(0x1ffffff << 0)

/* Payload FIFO register */
#define DSIM_PAYLOAD					(0x5c)

/* Read FIFO register */
#define DSIM_RXFIFO					(0x60)

/* FIFO status and control register */
#define DSIM_FIFOCTRL					(0x68)
#define DSIM_FIFOCTRL_NUMBER_OF_PH_SFR_GET(x)		(((x) >> 16) & 0x3f)
#define DSIM_FIFOCTRL_EMPTY_RX				(1 << 12)
#define DSIM_FIFOCTRL_FULL_PH_SFR			(1 << 11)
#define DSIM_FIFOCTRL_EMPTY_PH_SFR			(1 << 10)
#define DSIM_FIFOCTRL_FULL_PL_SFR			(1 << 9)
#define DSIM_FIFOCTRL_EMPTY_PL_SFR			(1 << 8)

#define DSIM_CMD_CONFIG					(0x80)
#define DSIM_CMD_CONFIG_PKT_GO_RDY			(1 << 17)
#define DSIM_CMD_CONFIG_PKT_GO_EN			(1 << 16)
#define DSIM_CMD_CONFIG_MULTI_CMD_PKT_EN		(1 << 8)

#define DSIM_CMD_TE_CTRL0				(0x84)
#define DSIM_CMD_TE_CTRL1				(0x88)
#define DSIM_CMD_STATUS					(0x8c)

#endif /* _REGS_DSIM_9810_H */
