// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos9810 DSIM0 command path for the adopt-mode DECON driver.
 *
 * Port of the Samsung 4.9 dsim_write_data(), dsim_long_data_wr() and
 * dsim_read_data() (drivers/video/fbdev/exynos/dpu_9810/dsim_drv.c) and
 * the dsim_reg.c FIFO helpers they use. Differences, all on the
 * conservative side:
 *
 *  - polled: 6.12 has no DSIM interrupt handler, so completion is read
 *    from FIFOCTRL / INTSRC instead of the PH_FIFO_EMPTY / RX_DATA_DONE
 *    interrupts (same bits, same 50 ms / 100 ms limits as 4.9);
 *  - one packet in flight: every packet waits for the header and payload
 *    FIFOs to drain (4.9 only waits for reads, DCS power commands and
 *    full FIFOs);
 *  - no recovery: 4.9 retries with dsim_function_reset(); here any
 *    timeout or error report stops the caller, the link is never reset;
 *  - the caller must hold the DECON command window (decon_drv.c:
 *    dpu_cmd_window_begin), i.e. DECON adopted, hw_lock held, TE trigger
 *    masked and no frame in flight;
 *  - DCS power, sleep, display on/off, reset, tearing, address and memory
 *    write commands and the DSC/PPS/peripheral packet types are refused.
 *
 * Nothing here programs DSIM configuration, clocks, timing or the PHY.
 * The only registers written are PKTHDR/PAYLOAD (the packet FIFOs) and
 * INTSRC (write-1-to-clear of status bits nobody else in 6.12 uses).
 */
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <video/mipi_display.h>

#include "dpu9810.h"
#include "regs-dsim.h"

#define DSIM_WR_TIMEOUT_US	50000	/* 4.9 MIPI_WR_TIMEOUT */
#define DSIM_RD_TIMEOUT_US	100000	/* 4.9 MIPI_RD_TIMEOUT */
#define DSIM_RX_FIFO_MAX_DEPTH	64	/* 4.9 dsim.h */
#define DSIM_MAX_LONG		64
#define DSIM_MAX_READ		40	/* 4.9 panel.c MAX_READ_BYTES */

/*
 * Fatal: lost command or corrupted answer. 4.9 reports RX ECC errors
 * (dsim_irq_handler) and never queues past a full FIFO.
 */
#define DSIM_ERR_BITS	(DSIM_INTSRC_SFR_PH_FIFO_OVERFLOW | DSIM_INTSRC_ERR_RX_ECC)
/*
 * Logged only. 4.9 dsim_reg_set_int() leaves BTA_TOUT, LPRX_TOUT and
 * ABNRMAL_CMD_ST masked and its handlers never look at them or at
 * INVALID_SFR_VALUE; with the 4.9/SBoot BTA timeout of 0xff, BTA_TOUT is
 * raised by an otherwise good DCS read (trial q41: RX_DATA_DONE, BUS_TURN_OVER
 * and a well-formed response, plus BTA_TOUT). Read answers are validated by
 * their packet type and length, the panel ID / HBM gamma cross-checks and
 * the read-back instead.
 */
#define DSIM_NOTE_BITS	(DSIM_INTSRC_INVALID_SFR_VALUE | DSIM_INTSRC_ABNRMAL_CMD_ST | \
			 DSIM_INTSRC_LPRX_TOUT | DSIM_INTSRC_BTA_TOUT)
#define DSIM_OWN_BITS	(DSIM_ERR_BITS | DSIM_NOTE_BITS | DSIM_INTSRC_SFR_PH_FIFO_EMPTY | \
			 DSIM_INTSRC_SFR_PL_FIFO_EMPTY | DSIM_INTSRC_RX_DATA_DONE | \
			 DSIM_INTSRC_BUS_TURN_OVER)

/* 4.9 dsim.h, MIPI DSI error report bits [15:0] of an ack/error packet */
#define MIPI_DSI_ERR_BIT_MASK	0x3f3f

static u32 dsim_rd(struct dpu_dev *d, u32 off)
{
	return readl(d->dsim + off);
}

static void dsim_wr(struct dpu_dev *d, u32 off, u32 val)
{
	writel(val, d->dsim + off);
}

/* DCS commands this path never sends, whatever the caller asks */
static bool dsim_dcs_forbidden(u8 cmd)
{
	switch (cmd) {
	case MIPI_DCS_SOFT_RESET:
	case MIPI_DCS_ENTER_SLEEP_MODE:
	case MIPI_DCS_EXIT_SLEEP_MODE:
	case MIPI_DCS_ENTER_PARTIAL_MODE:
	case MIPI_DCS_ENTER_NORMAL_MODE:
	case MIPI_DCS_SET_DISPLAY_OFF:
	case MIPI_DCS_SET_DISPLAY_ON:
	case MIPI_DCS_SET_COLUMN_ADDRESS:
	case MIPI_DCS_SET_PAGE_ADDRESS:
	case MIPI_DCS_WRITE_MEMORY_START:
	case MIPI_DCS_WRITE_MEMORY_CONTINUE:
	case MIPI_DCS_SET_TEAR_OFF:
	case MIPI_DCS_SET_TEAR_ON:
	case MIPI_DCS_EXIT_IDLE_MODE:
	case MIPI_DCS_ENTER_IDLE_MODE:
		return true;
	default:
		return false;
	}
}

/*
 * Log the DSIM state SBoot left and decide whether commands may be sent:
 * command mode, link out of reset and out of ULPS, PLL stable, packet-go
 * off and both SFR FIFOs empty. Read only.
 */
int dpu_dsim_check(struct dpu_dev *d)
{
	u32 swrst, cfg, clk, esc, dphy, l3, fifo, cmdc, isrc, imsk;

	if (!d->dsim)
		return -ENODEV;
	swrst = dsim_rd(d, DSIM_SWRST);
	cfg = dsim_rd(d, DSIM_CONFIG);
	clk = dsim_rd(d, DSIM_CLK_CTRL);
	esc = dsim_rd(d, DSIM_ESCMODE);
	dphy = dsim_rd(d, DSIM_DPHY_STATUS);
	l3 = dsim_rd(d, DSIM_LINK_STATUS3);
	fifo = dsim_rd(d, DSIM_FIFOCTRL);
	cmdc = dsim_rd(d, DSIM_CMD_CONFIG);
	isrc = dsim_rd(d, DSIM_INTSRC);
	imsk = dsim_rd(d, DSIM_INTMSK);
	pb_dbg("DSIM0: SWRST=%#x CONFIG=%#x CLK_CTRL=%#x ESCMODE=%#x (cmd %s) TIMEOUT=%#x DPHY=%#x LINK1=%#x LINK3=%#x FIFOCTRL=%#x CMD_CONFIG=%#x INTSRC=%#x INTMSK=%#x\n",
		swrst, cfg, clk, esc, esc & DSIM_ESCMODE_CMD_LPDT ? "LP" : "HS",
		dsim_rd(d, DSIM_TIMEOUT), dphy, dsim_rd(d, DSIM_LINK_STATUS1), l3, fifo, cmdc,
		isrc, imsk);

	if (swrst & (DSIM_SWRST_RESET | DSIM_SWRST_FUNCRST)) {
		pr_err(DPU_PB_PFX "DSIM0 held in reset\n");
		return -ENODEV;
	}
	if (cfg & DSIM_CONFIG_VIDEO_MODE) {
		pr_err(DPU_PB_PFX "DSIM0 in video mode, expected command mode\n");
		return -ENODEV;
	}
	if (!(cfg & DSIM_CONFIG_LANES_EN(0x1f))) {
		pr_err(DPU_PB_PFX "DSIM0 has no lane enabled\n");
		return -ENODEV;
	}
	if (!(l3 & DSIM_LINK_STATUS3_PLL_STABLE)) {
		pr_err(DPU_PB_PFX "DSIM0 PLL not stable\n");
		return -ENODEV;
	}
	if ((dphy & DSIM_DPHY_STATUS_ULPS_CLK) || DSIM_DPHY_STATUS_ULPS_DATA_LANE_GET(dphy) ||
	    (esc & (DSIM_ESCMODE_TX_ULPS_CLK | DSIM_ESCMODE_TX_ULPS_DATA |
		    DSIM_ESCMODE_FORCE_STOP_STATE | DSIM_ESCMODE_FORCE_BTA))) {
		pr_err(DPU_PB_PFX "DSIM0 link in ULPS / forced stop state\n");
		return -ENODEV;
	}
	if (cmdc & DSIM_CMD_CONFIG_PKT_GO_EN) {
		pr_err(DPU_PB_PFX "DSIM0 packet-go enabled, not supported\n");
		return -ENODEV;
	}
	if (!(fifo & DSIM_FIFOCTRL_EMPTY_PH_SFR) || !(fifo & DSIM_FIFOCTRL_EMPTY_PL_SFR)) {
		pr_err(DPU_PB_PFX "DSIM0 SFR FIFOs not empty (someone else is sending)\n");
		return -EBUSY;
	}
	return 0;
}

/* Clear the INTSRC status bits this path polls (4.9 dsim_reg_clear_int) */
void dpu_dsim_clear_status(struct dpu_dev *d)
{
	dsim_wr(d, DSIM_INTSRC, DSIM_OWN_BITS);
}

/* 4.9 dsim_reg_wr_tx_header(): read-modify-write of PKTHDR[24:0] */
static void dsim_tx_header(struct dpu_dev *d, u8 id, u8 d0, u8 d1, bool bta)
{
	u32 val = DSIM_PKTHDR_BTA_TYPE(bta ? 1 : 0) | DSIM_PKTHDR_ID(id) |
		  DSIM_PKTHDR_DATA0(d0) | DSIM_PKTHDR_DATA1(d1);
	u32 old = dsim_rd(d, DSIM_PKTHDR);

	dsim_wr(d, DSIM_PKTHDR, (val & DSIM_PKTHDR_DATA) | (old & ~DSIM_PKTHDR_DATA));
}

/* 4.9 dsim_long_data_wr(): little-endian words, remainder in the last one */
static void dsim_tx_payload(struct dpu_dev *d, const u8 *buf, size_t len)
{
	size_t i;

	for (i = 0; i < len; i += 4) {
		u32 w = 0;
		size_t j;

		for (j = 0; j < 4 && i + j < len; j++)
			w |= (u32)buf[i + j] << (8 * j);
		dsim_wr(d, DSIM_PAYLOAD, w);
	}
}

static int dsim_wait_sfr_empty(struct dpu_dev *d, u32 timeout_us)
{
	u32 v;

	return readl_poll_timeout(d->dsim + DSIM_FIFOCTRL, v,
				  (v & DSIM_FIFOCTRL_EMPTY_PH_SFR) &&
				  (v & DSIM_FIFOCTRL_EMPTY_PL_SFR), 5, timeout_us);
}

static int dsim_check_errors(struct dpu_dev *d, const char *what, u32 nr)
{
	u32 isrc = dsim_rd(d, DSIM_INTSRC);

	if (isrc & DSIM_NOTE_BITS) {
		d->dsim_nnote++;
		pb_dbg("%s#%u: note: INTSRC bits %#x (masked and ignored by 4.9; INTSRC=%#x CMD_STATUS=%#x)\n",
			what, nr, isrc & DSIM_NOTE_BITS, isrc, dsim_rd(d, DSIM_CMD_STATUS));
		dsim_wr(d, DSIM_INTSRC, isrc & DSIM_NOTE_BITS);
	}
	if (!(isrc & DSIM_ERR_BITS))
		return 0;
	pr_err(DPU_PB_PFX "%s#%u: DSIM INTSRC error bits %#x (INTSRC=%#x CMD_STATUS=%#x)\n",
	       what, nr, isrc & DSIM_ERR_BITS, isrc, dsim_rd(d, DSIM_CMD_STATUS));
	dsim_wr(d, DSIM_INTSRC, isrc & DSIM_ERR_BITS);
	return -EIO;
}

/*
 * Send one packet: DCS/generic short or long write, or
 * SET_MAXIMUM_RETURN_PACKET_SIZE. @buf holds the command byte (DCS) or the
 * parameters (generic); short packets put buf[0]/buf[1] in DATA0/DATA1.
 */
int dpu_dsim_tx(struct dpu_dev *d, u8 type, const u8 *buf, size_t len)
{
	bool is_long = false, dcs = false;
	u8 d0 = 0, d1 = 0;
	u32 nr, us;
	ktime_t t0;
	int ret;

	if (!d->dsim || !d->cmd_window)
		return -EPERM;
	lockdep_assert_held(&d->hw_lock);

	switch (type) {
	case MIPI_DSI_DCS_SHORT_WRITE:
		dcs = true;
		fallthrough;
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
		if (len != 1)
			return -EINVAL;
		d0 = buf[0];
		break;
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
		dcs = true;
		fallthrough;
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
	case MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE:
		if (len != 2)
			return -EINVAL;
		d0 = buf[0];
		d1 = buf[1];
		break;
	case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
		if (len != 0)
			return -EINVAL;
		break;
	case MIPI_DSI_DCS_LONG_WRITE:
		dcs = true;
		fallthrough;
	case MIPI_DSI_GENERIC_LONG_WRITE:
		if (!len || len > DSIM_MAX_LONG)
			return -EINVAL;
		is_long = true;
		d0 = len & 0xff;
		d1 = (len >> 8) & 0xff;
		break;
	default:
		return -EINVAL;		/* DSC PRA/PPS, peripheral on/off, ... */
	}
	if (dcs && dsim_dcs_forbidden(buf[0])) {
		pr_err(DPU_PB_PFX "refusing DCS command %#04x\n", buf[0]);
		return -EPERM;
	}

	if (dsim_wait_sfr_empty(d, DSIM_WR_TIMEOUT_US)) {
		pr_err(DPU_PB_PFX "SFR FIFO busy before tx (FIFOCTRL=%#x)\n",
		       dsim_rd(d, DSIM_FIFOCTRL));
		return -EBUSY;
	}

	nr = ++d->dsim_ntx;
	t0 = ktime_get();

	dsim_wr(d, DSIM_INTSRC, DSIM_INTSRC_SFR_PH_FIFO_EMPTY | DSIM_INTSRC_SFR_PL_FIFO_EMPTY);
	if (is_long)
		dsim_tx_payload(d, buf, len);
	dsim_tx_header(d, type, d0, d1, false);

	ret = dsim_wait_sfr_empty(d, DSIM_WR_TIMEOUT_US);
	us = ktime_us_delta(ktime_get(), t0);
	d->dsim_max_tx_us = max(d->dsim_max_tx_us, us);
	if (ret) {
		d->dsim_nerr++;
		pr_err(DPU_PB_PFX "tx#%u dt=%#04x %*ph: FIFO did not drain in %u us (FIFOCTRL=%#x INTSRC=%#x)\n",
		       nr, type, (int)min_t(size_t, len, 8), buf, us,
		       dsim_rd(d, DSIM_FIFOCTRL), dsim_rd(d, DSIM_INTSRC));
		return ret;
	}
	ret = dsim_check_errors(d, "tx", nr);
	if (ret)
		d->dsim_nerr++;
	pb_dbg("tx#%u dt=%#04x len=%zu %u us: %*ph\n", nr, type, len, us, (int)len, buf);
	return ret;
}

/*
 * 4.9 waits for the RX_DATA_DONE interrupt. Poll the same INTSRC bit, and
 * also accept a non-empty RX FIFO in case INTSRC does not latch while the
 * interrupt is masked.
 */
static bool dsim_rx_ready(struct dpu_dev *d)
{
	return (dsim_rd(d, DSIM_INTSRC) & DSIM_INTSRC_RX_DATA_DONE) ||
	       !(dsim_rd(d, DSIM_FIFOCTRL) & DSIM_FIFOCTRL_EMPTY_RX);
}

/* 4.9 dsim_reg_rx_err_handler() */
static int dsim_rx_err(struct dpu_dev *d, u32 rx)
{
	u32 err = (rx >> 8) & 0xffff;

	if (!(err & MIPI_DSI_ERR_BIT_MASK))
		return 0;
	pr_err(DPU_PB_PFX "panel error report %#06x (rx_fifo=%#x)\n", err, rx);
	return -EIO;
}

/*
 * DCS read of @len bytes (4.9 dsim_read_data(MIPI_DSI_DCS_READ)): set the
 * maximum return packet size, send the read request with bus turnaround,
 * wait for the response and parse the RX FIFO the way 4.9 does.
 */
int dpu_dsim_dcs_read(struct dpu_dev *d, u8 cmd, u8 *buf, size_t len)
{
	u8 mrps[2] = { len, 0 };
	u32 rx, v, nr, depth = DSIM_RX_FIFO_MAX_DEPTH, drained = 0, hdr = 0;
	size_t got = 0, i;
	bool ready;
	ktime_t t0 = ktime_get();
	u32 us;
	int ret;

	if (!d->dsim || !d->cmd_window)
		return -EPERM;
	lockdep_assert_held(&d->hw_lock);
	if (!len || len > DSIM_MAX_READ)
		return -EINVAL;

	/* stale data in the RX FIFO would be parsed as our answer */
	while (!(dsim_rd(d, DSIM_FIFOCTRL) & DSIM_FIFOCTRL_EMPTY_RX) && drained < DSIM_RX_FIFO_MAX_DEPTH) {
		dsim_rd(d, DSIM_RXFIFO);
		drained++;
	}
	if (drained)
		pr_warn(DPU_PB_PFX "drained %u stale RX FIFO words\n", drained);
	dsim_wr(d, DSIM_INTSRC, DSIM_INTSRC_RX_DATA_DONE);

	ret = dpu_dsim_tx(d, MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, mrps, 2);
	if (ret)
		return ret;

	nr = ++d->dsim_nrx;
	dsim_wr(d, DSIM_INTSRC, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);
	dsim_tx_header(d, MIPI_DSI_DCS_READ, cmd, 0, true);
	ret = read_poll_timeout(dsim_rx_ready, ready, ready, 10, DSIM_RD_TIMEOUT_US, false, d);
	if (ret) {
		d->dsim_nerr++;
		pr_err(DPU_PB_PFX "rx#%u dcs %#04x: no response in %u us (INTSRC=%#x FIFOCTRL=%#x)\n",
		       nr, cmd, DSIM_RD_TIMEOUT_US, dsim_rd(d, DSIM_INTSRC),
		       dsim_rd(d, DSIM_FIFOCTRL));
		return ret;
	}
	/* data is arriving; let the packet complete before parsing */
	if (readl_poll_timeout(d->dsim + DSIM_INTSRC, v, v & DSIM_INTSRC_RX_DATA_DONE, 10, 2000))
		pr_warn(DPU_PB_PFX "rx#%u: RX_DATA_DONE not latched, parsing the RX FIFO anyway\n",
			nr);

	do {
		rx = dsim_rd(d, DSIM_RXFIFO);
		if (!hdr)
			hdr = rx;
		switch (rx & 0xff) {
		case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
			ret = dsim_rx_err(d, rx);
			if (ret)
				goto out;
			break;
		case MIPI_DSI_RX_END_OF_TRANSMISSION:
			break;
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
			for (i = 0; i < len && i < 2; i++)
				buf[i] = (rx >> (8 + i * 8)) & 0xff;
			got = i;
			break;
		case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
		case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
			got = (rx & 0x00ffff00) >> 8;
			if (got > len) {
				pr_err(DPU_PB_PFX "rx#%u: %zu bytes returned, %zu asked\n", nr, got, len);
				ret = -EIO;
				goto out;
			}
			for (i = 0; i < got; i += 4) {
				u32 w = dsim_rd(d, DSIM_RXFIFO);
				size_t j;

				for (j = 0; j < 4 && i + j < got; j++)
					buf[i + j] = (w >> (8 * j)) & 0xff;
			}
			break;
		default:
			pr_err(DPU_PB_PFX "rx#%u: invalid packet %#x\n", nr, rx);
			ret = -EIO;
			goto out;
		}
	} while (!(dsim_rd(d, DSIM_FIFOCTRL) & DSIM_FIFOCTRL_EMPTY_RX) && --depth);

	if (!depth) {
		pr_err(DPU_PB_PFX "rx#%u: RX FIFO did not empty\n", nr);
		ret = -EIO;
		goto out;
	}
	if (got != len) {
		pr_err(DPU_PB_PFX "rx#%u dcs %#04x: %zu of %zu bytes (rx hdr %#x)\n", nr, cmd, got,
		       len, hdr);
		ret = -EIO;
		goto out;
	}
	ret = dsim_check_errors(d, "rx", nr);
	if (!ret)
		pb_dbg("rx#%u dcs %#04x len=%zu %u us (rx hdr %#x): %*ph\n", nr, cmd, len,
		       (u32)ktime_us_delta(ktime_get(), t0), hdr, (int)len, buf);
	else if (got)
		pr_err(DPU_PB_PFX "rx#%u dcs %#04x discarded answer (rx hdr %#x): %*ph\n", nr, cmd, hdr,
		       (int)got, buf);
out:
	if (ret)
		d->dsim_nerr++;
	dsim_wr(d, DSIM_INTSRC, DSIM_INTSRC_RX_DATA_DONE);
	us = ktime_us_delta(ktime_get(), t0);
	d->dsim_max_rx_us = max(d->dsim_max_rx_us, us);
	return ret;
}
