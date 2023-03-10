/*
 *******************************************************************************
 *
 *  Component: XGold MIPI DSI driver
 *
 *  Copyright (C) 2014, Intel Mobile Communications GmbH.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *******************************************************************************
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/reset.h>
#include <video/mipi_display.h>

#include "dsi_display.h"
#include "dsi_hwregs.h"

/* #define USE_DSI_ACKNOWLEDGE */

static int dbg_thresd;
module_param(dbg_thresd, int, S_IRUGO | S_IWUSR);

#define DSI_DBG1(x...) do {	\
	if (unlikely(dbg_thresd >= 1))	\
		pr_info("[dsi] "x);		\
} while (0)

#define DSI_DBG2(x...) do {	\
	if (unlikely(dbg_thresd >= 2))	\
		pr_info("[dsi] "x);		\
} while (0)

#define DSI_DBG3(x...) do {	\
	if (unlikely(dbg_thresd >= 3))	\
		pr_info("[dsi] "x);		\
} while (0)


#define DSI_CFG_DATA_PIX	0
#define DSI_CFG_DATA_DAT	1
#define DSI_CFG_SOURCE_DPI 0
#define DSI_CFG_SOURCE_TXD 1

#define DSI_CFG_OFF(_mode_) (\
		BITFLDS(EXR_DSI_CFG_EN, 0) |\
		BITFLDS(EXR_DSI_CFG_LANES, 0) |\
		BITFLDS(EXR_DSI_CFG_LP, 0) |\
		BITFLDS(EXR_DSI_CFG_MODE, _mode_)|\
		BITFLDS(EXR_DSI_CFG_DATA, DSI_CFG_DATA_DAT)|\
		BITFLDS(EXR_DSI_CFG_SOURCE, DSI_CFG_SOURCE_TXD))

#define DSI_CFG_INIT(_nlanes_) (\
		BITFLDS(EXR_DSI_CFG_EN, 1) |\
		BITFLDS(EXR_DSI_CFG_LANES, (_nlanes_-1)) |\
		BITFLDS(EXR_DSI_CFG_LP, 0) |\
		BITFLDS(EXR_DSI_CFG_MODE, DSI_CMD)|\
		BITFLDS(EXR_DSI_CFG_DATA, DSI_CFG_DATA_DAT)|\
		BITFLDS(EXR_DSI_CFG_SOURCE, DSI_CFG_SOURCE_TXD))
/* need_to_modify
#define DSI_CFG_RX_LP_DATA(_nlanes_) (\
		BITFLDS(EXR_DSI_CFG_EN, 1) |\
		BITFLDS(EXR_DSI_CFG_LANES, (_nlanes_-1)) |\
		BITFLDS(EXR_DSI_CFG_TX, 1) |\
		BITFLDS(EXR_DSI_CFG_LP, 1) |\
		BITFLDS(EXR_DSI_CFG_TURN, 1) |\
		BITFLDS(EXR_DSI_CFG_MODE, DSI_CMD)|\
		BITFLDS(EXR_DSI_CFG_DATA, DSI_CFG_DATA_DAT)|\
		BITFLDS(EXR_DSI_CFG_SOURCE, DSI_CFG_SOURCE_TXD))
*/
#define DSI_CFG_RX_LP_STP(_stp_) (\
		BITFLDS(EXR_DSI_CFG_EN, 1) |\
		BITFLDS(EXR_DSI_CFG_LANES, 0) |\
		BITFLDS(EXR_DSI_CFG_TX, 0) |\
		BITFLDS(EXR_DSI_CFG_LP, 1) |\
		BITFLDS(EXR_DSI_CFG_STP, _stp_) |\
		BITFLDS(EXR_DSI_CFG_MODE, DSI_CMD)|\
		BITFLDS(EXR_DSI_CFG_DATA, DSI_CFG_DATA_DAT)|\
		BITFLDS(EXR_DSI_CFG_SOURCE, DSI_CFG_SOURCE_TXD))

#define DSI_CFG_TX_LP_DATA(_nlanes_) (\
		BITFLDS(EXR_DSI_CFG_EN, 1) |\
		BITFLDS(EXR_DSI_CFG_LANES, (0)) |\
		BITFLDS(EXR_DSI_CFG_LP, 1) |\
		BITFLDS(EXR_DSI_CFG_MODE, DSI_CMD)|\
		BITFLDS(EXR_DSI_CFG_DATA, DSI_CFG_DATA_DAT)|\
		BITFLDS(EXR_DSI_CFG_SOURCE, DSI_CFG_SOURCE_TXD))

/* need_to_modify
#define DSI_CFG_TX_HS_DATA_ACK(_nlanes_) (\
		BITFLDS(EXR_DSI_CFG_EN, 1) |\
		BITFLDS(EXR_DSI_CFG_LANES, (_nlanes_-1)) |\
		BITFLDS(EXR_DSI_CFG_TX, 1) |\
		BITFLDS(EXR_DSI_CFG_LP, 0) |\
		BITFLDS(EXR_DSI_CFG_TURN, 1) |\
		BITFLDS(EXR_DSI_CFG_MODE, DSI_CMD)|\
		BITFLDS(EXR_DSI_CFG_DATA, DSI_CFG_DATA_DAT))|\
		BITFLDS(EXR_DSI_CFG_SOURCE, DSI_CFG_SOURCE_TXD))
*/

#define DSI_CFG_TX_HS_DATA(_nlanes_) (\
		BITFLDS(EXR_DSI_CFG_EN, 1) |\
		BITFLDS(EXR_DSI_CFG_LANES, (_nlanes_-1)) |\
		BITFLDS(EXR_DSI_CFG_LP, 0) |\
		BITFLDS(EXR_DSI_CFG_MODE, DSI_CMD)|\
		BITFLDS(EXR_DSI_CFG_DATA, DSI_CFG_DATA_DAT)|\
		BITFLDS(EXR_DSI_CFG_SOURCE, DSI_CFG_SOURCE_TXD))

#define DSI_CFG_TX_HS_PIXEL(_nlanes_, _mode_) (\
		BITFLDS(EXR_DSI_CFG_EN, 1) |\
		BITFLDS(EXR_DSI_CFG_LANES, (_nlanes_-1)) |\
		BITFLDS(EXR_DSI_CFG_TX, 1) |\
		BITFLDS(EXR_DSI_CFG_LP, 0) |\
		BITFLDS(EXR_DSI_CFG_MODE, _mode_)|\
		BITFLDS(EXR_DSI_CFG_DATA, DSI_CFG_DATA_PIX)|\
		BITFLDS(EXR_DSI_CFG_SOURCE, DSI_CFG_SOURCE_DPI))

#define DSI_REF_CLK_KHZ         (26000)
#define DSI_RATE_N_MAX          (0xFF)
#define DSI_RATE_M_MAX          (0xF)
#define DSI_RATE_MAX            (1000000000)
#define DSI_RATE_MIN            (160000000)
#define DSI_RATE(n, m)          (DSI_REF_CLK_KHZ * (n + 1) / (m + 1) * 1000)
#define DSI_RATE_OVERHEAD(r)    (r / 100 * 101)

#define TLPX_NS 50

/**
 * Common TX functions
 */
static void dsi_mipidsi_send_short_packet(struct dsi_display *display,
					  struct display_msg *msg,
					  unsigned int dsicfg)
{
	unsigned char *data_msg = msg->datas;
	unsigned int dsihead =
		BITFLDS(EXR_DSI_HEAD_WCNT, data_msg[0]) |
		BITFLDS(EXR_DSI_HEAD_HEADER, msg->type);

	if (msg->length > 1)
		dsihead |= data_msg[1]<<16;

	DSI_DBG2("dsi short pkt: (head:0x%08x cfg:0x%08x)\n",
		 dsihead, dsicfg);

	dsi_write_field(display, EXR_DSI_VID3,
			BITFLDS(EXR_DSI_VID3_PIXEL_PACKETS, 1));

	dsi_write_field(display, EXR_DSI_HEAD, dsihead);
	dsi_write_field(display, EXR_DSI_CFG,
			dsicfg | BITFLDS(EXR_DSI_CFG_HEAD_LAT, 1));
	dsi_write_field(display, EXR_DSI_CFG, dsicfg);
	dsi_write_field(display, EXR_DSI_CFG,
			dsicfg | BITFLDS(EXR_DSI_CFG_TX, 1) |
			BITFLDS(EXR_DSI_CFG_CFG_LAT, 1));
	dsi_write_field(display, EXR_DSI_CFG,
			dsicfg | BITFLDS(EXR_DSI_CFG_TX, 1));
}

static void dsi_mipidsi_send_long_packet_dma(struct dsi_display *display,
					     struct display_msg *msg,
					     unsigned int dsicfg)
{
	unsigned char *data_msg = msg->datas;
	unsigned int length = msg->length;
	unsigned int dsihead =
		BITFLDS(EXR_DSI_HEAD_WCNT, msg->length) |
		BITFLDS(EXR_DSI_HEAD_HEADER, msg->type);

	/* the first data byte of DCS long packet must be put
	 * in CMDBYTE of DSI_HEAD */
	if (msg->type == MIPI_DSI_DCS_LONG_WRITE && length > 0) {
		dsihead |= BITFLDS(EXR_DSI_HEAD_CMD, *data_msg++);
		length--;
	}
	DSI_DBG2("dsi long dma pkt: wcnt:0x%04x (head:0x%08x cfg:0x%08x)\n",
		 msg->length, dsihead, dsicfg);

	dsi_write_field(display, EXR_DSI_VID3,
			BITFLDS(EXR_DSI_VID3_PIXEL_PACKETS, 1));
	dsi_write_field(display, EXR_DSI_VID6,
			BITFLDS(EXR_DSI_VID6_LAST_PIXEL, length));
	dsi_write_field(display, EXR_DSI_TPS_CTRL,
			BITFLDS(EXR_DSI_TPS_CTRL_TPS, length));

	while (length > 0) {
		int j = 0;
		unsigned int reg = 0;

		for (j = 0; j < 4 && length; j++) {
			length--;
			reg |= ((uint8_t) *data_msg++)<<(j*8);
		}

		dsi_write_field(display, EXR_DSI_TXD, reg);
		DSI_DBG2("payload 0x%08x\n", reg);
	}

	dsi_write_field(display, EXR_DSI_HEAD, dsihead);
	dsi_write_field(display, EXR_DSI_CFG,
			dsicfg | BITFLDS(EXR_DSI_CFG_HEAD_LAT, 1));
	dsi_write_field(display, EXR_DSI_CFG, dsicfg);
	dsi_write_field(display, EXR_DSI_CFG,
			dsicfg | BITFLDS(EXR_DSI_CFG_TX, 1) |
			BITFLDS(EXR_DSI_CFG_CFG_LAT, 1));
	dsi_write_field(display, EXR_DSI_CFG, dsicfg |
			BITFLDS(EXR_DSI_CFG_TX, 1));
}

static void dsi_send_cmd(struct dsi_display *display, struct display_msg *msg)
{
	int ret = 0;
	unsigned int dsicfg;

	if (msg->flags & LCD_MSG_LP)
#ifdef CONFIG_SONY_CADIZ
		dsicfg = DSI_CFG_TX_LP_DATA(1) | BITFLDS(EXR_DSI_CFG_GATE, 1);
#else
		dsicfg = DSI_CFG_TX_LP_DATA(1);
#endif
	else
		dsicfg = DSI_CFG_TX_HS_DATA(display->dif.dsi.nblanes);

	if (msg->length <= 2)
		dsi_mipidsi_send_short_packet(display, msg, dsicfg);
	else
		dsi_mipidsi_send_long_packet_dma(display, msg, dsicfg);

	DSI_DBG2("wait for eoc\n");
	ret = dsi_completion_timeout_ms(&display->sync.dsifin,
					display->sync.dsifin_to);
	if (!ret) {
		DSI_DBG2("dsifin interrupt timedout %dms\n",
			 display->sync.dsifin_to);
	} else {
		DSI_DBG2("eoc received\n");
#ifdef USE_DSI_ACKNOWLEDGE
		ret = dsi_mipidsi_ack_wait(display);
		dsi_mipidsi_force_ownership(display);
#endif
	}

	mdelay(msg->delay);
}

int dsi_read_cmd(struct dsi_display *display, u32 type, u8 *cmd,
		 unsigned int cmd_len, u8 *data, unsigned int data_len)
{
	struct display_msg msg;
	u32 nwords, nbytes;
	u32 dsicfg = DSI_CFG_TX_LP_DATA(1);
	int rcv_type;
	int i = 0;
	u8 rxdata[4];
	u8 mrps_data[2] = {0};

	if (type != MIPI_DSI_DCS_READ &&
		type != MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM &&
		type != MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM &&
		type != MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM)
		return -EINVAL;

	mrps_data[0] = (u8)data_len;
	msg.type = MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE;
	msg.datas = mrps_data;
	msg.length = 2;
	dsi_mipidsi_send_short_packet(display, &msg, dsicfg);
	if (!dsi_completion_timeout_ms(&display->sync.dsifin,
		display->sync.dsifin_to)) {
		DSI_ERR("dsifin interrupt timedout %dms\n",
			display->sync.dsifin_to);
		return -EBUSY;
	}

	msg.type = type;
	msg.datas = cmd;
	msg.length = cmd_len;
	if (msg.length <= 2)
		dsi_mipidsi_send_short_packet(display, &msg, dsicfg);
	else
		dsi_mipidsi_send_long_packet_dma(display, &msg, dsicfg);

	if (!dsi_completion_timeout_ms(&display->sync.dsifin,
		display->sync.dsifin_to)) {
		DSI_ERR("dsifin interrupt timedout %dms\n",
			display->sync.dsifin_to);
		return -EBUSY;
	}

	dsicfg |= BITFLDS(EXR_DSI_CFG_TURN, 1);
	dsi_write_field(display, EXR_DSI_CFG, dsicfg |
			BITFLDS(EXR_DSI_CFG_TX, 1) |
			BITFLDS(EXR_DSI_CFG_CFG_LAT, 1));
	dsi_write_field(display, EXR_DSI_CFG, dsicfg |
			BITFLDS(EXR_DSI_CFG_TX, 1));
	dsi_wait_status(display, EXR_DSI_STAT_DSI_DIR, DSI_DIR_RX, 1, 0, 1000);
	dsi_wait_status(display, EXR_DSI_STAT_DSI_DIR, DSI_DIR_TX, 1, 0, 1000);
	nwords = dsi_read_field(display, EXR_DSI_FIFO_STAT_RXFFS);
	nbytes = dsi_read_field(display, EXR_DSI_RPS_STAT);

	DSI_DBG3("EXR_DSI_FIFO_STAT_RXFFS = %#x\n", nwords);
	DSI_DBG3("EXR_DSI_RPS_STAT = %#x\n", nbytes);

	if (!nwords)
		return 0;

	*((u32 *)rxdata) = dsi_read_field(display, EXR_DSI_RXD);

	DSI_DBG3("RX DATA = %#x\n", *((u32 *)rxdata));

	rcv_type = rxdata[0];
	if (rcv_type == MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE ||
		rcv_type == MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE) {
		if (data_len < 1) {
			DSI_ERR("MIPI read data length is not enough\n");

			return -EINVAL;
		}

		data[0] = rxdata[1];
		i = 0;
	} else if (rcv_type == MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE ||
		rcv_type == MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE) {
		if (data_len < 2) {
			DSI_ERR("MIPI read data length is not enough\n");

			return -EINVAL;
		}

		data[0] = rxdata[1];
		data[1] = rxdata[2];
		i = 1;
	} else if (rcv_type == MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE ||
		rcv_type == MIPI_DSI_RX_DCS_LONG_READ_RESPONSE) {
		int rx_i;

		for (i = 0; i < data_len && nbytes; i++, nbytes--) {
			rx_i = i % 4;
			if (!rx_i && --nwords) {
				*((u32 *)rxdata) =
					dsi_read_field(display, EXR_DSI_RXD);

				DSI_DBG3("RX DATA = %#x\n", *((u32 *)rxdata));
			}

			data[i] = rxdata[rx_i];
		}
	}

	/* Empty RX FIFO */
	while (--nwords)
		dsi_read_field(display, EXR_DSI_RXD);

	return i + 1;
}

static int dsi_get_rate(struct dsi_display *display)
{
	return DSI_RATE(display->dif.dsi.n, display->dif.dsi.m);
}

static int dsi_get_bllp(struct dsi_display *display,
			int nlines, int bytes, int clk,
			int fps, int bitrate, int nlanes,
			int *bllp_time, int *line_time)
{
	/* bits per frames */
	unsigned int bitpframe = bytes * nlines * 8;
	/* maximum framerate */
	unsigned int maxfrate = bitrate * nlanes / bitpframe;
	/* shortest line time */
	unsigned int slt = NSEC_PER_SEC / maxfrate / nlines;
	/* target line time */
	unsigned int tlt = NSEC_PER_SEC / (fps) / nlines;
	/* clock cycle duration in ps */
	unsigned int clk_time = 1000000 / (clk/1000000);

	if (display->dif.dsi.video_mode == DSI_BURST)
		*bllp_time = ((tlt - slt) * 1000) / clk_time;
	else
		*bllp_time = 0;

	*line_time = tlt * 1000 / clk_time;

	DSI_DBG2("%d bytes / %d lines\n", bytes, nlines);
	DSI_DBG2("bits / frame = %d bits\n", bitpframe);
	DSI_DBG2("fps target %d fps (max=%d)\n", fps, maxfrate);
	DSI_DBG2("active time  = %d ns\n", slt);
	DSI_DBG2("target time  = %d ns\n", tlt);
	DSI_DBG2("clock cycle  = %d\n", clk_time);
	DSI_DBG2("bllp_time 0x%08x(%d)\n", *bllp_time, *bllp_time);
	DSI_DBG2("line_time 0x%08x(%d)\n", *line_time, *line_time);

	if (fps >= maxfrate) {
		DSI_ERR("target framerate(%d) cannot be reached, max %d\n",
				fps, maxfrate);
		*bllp_time = 0;
		return 0;
	}

	return 0;
}

static int dsi_configure_video_mode(struct dsi_display *display,
				    int stride, int nlines)
{
	unsigned int vid0, vid1, vid2, vid3, vid4, vid5, vid6;
	struct dsi_display_if_mipi_dsi *dif = &display->dif.dsi;

	if (display->dif.dsi.mode != DSI_VIDEO) {
		DSI_DBG2("%s: not video mode\n", __func__);
		return -EINVAL;
	}

	dsi_get_bllp(display,
		     nlines + dif->vfp + dif->vbp + dif->vsa,
		     stride + dif->hfp + dif->hbp + dif->hsa,
		     dif->dc_clk_rate,
		     display->fps,
		     dsi_get_rate(display),
		     dif->nblanes, &dif->bllp_time, &dif->line_time);

	vid0 =	BITFLDS(EXR_DSI_VID0_HFP, (!!display->dif.dsi.hfp))|
		BITFLDS(EXR_DSI_VID0_HBP, (!!display->dif.dsi.hbp))|
		BITFLDS(EXR_DSI_VID0_HSA, (!!display->dif.dsi.hsa))|
		BITFLDS(EXR_DSI_VID0_HFP_LP, dif->hfp_lp)|
		BITFLDS(EXR_DSI_VID0_HBP_LP, dif->hbp_lp)|
		BITFLDS(EXR_DSI_VID0_HSA_LP, dif->hsa_lp)|
		BITFLDS(EXR_DSI_VID0_HFP_BYTES, dif->hfp)|
		BITFLDS(EXR_DSI_VID0_HBP_BYTES, dif->hbp)|
		BITFLDS(EXR_DSI_VID0_HSA_BYTES, dif->hsa);

	vid1 =	BITFLDS(EXR_DSI_VID1_VACT_LINES, nlines)|
		BITFLDS(EXR_DSI_VID1_MODE, dif->video_mode)|
		BITFLDS(EXR_DSI_VID1_ID, dif->id)|
		BITFLDS(EXR_DSI_VID1_PIXEL, dif->video_pixel)|
		BITFLDS(EXR_DSI_VID1_FILL_BUFFER_TO, 0xFF);

	vid2 =	BITFLDS(EXR_DSI_VID2_VFP, dif->vfp)|
		BITFLDS(EXR_DSI_VID2_VBP, dif->vbp)|
		BITFLDS(EXR_DSI_VID2_VSA, dif->vsa);

	vid3 =	BITFLDS(EXR_DSI_VID3_PIXEL_BYTES, stride)|
		BITFLDS(EXR_DSI_VID3_PIXEL_PACKETS, 1);

	vid4 =	BITFLDS(EXR_DSI_VID4_BLANK_BYTES, 0)|
		BITFLDS(EXR_DSI_VID4_BLANK_PACKETS, 0);

	vid5 =	BITFLDS(EXR_DSI_VID5_LINE_TIME, dif->line_time)|
		BITFLDS(EXR_DSI_VID5_BLLP_TIME, dif->bllp_time);

	vid6 =	BITFLDS(EXR_DSI_VID6_LAST_BLANK, stride)|
		BITFLDS(EXR_DSI_VID6_LAST_PIXEL, stride);

	DSI_DBG2(
		"MIPI-DSI video:0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x)\n",
		vid0, vid1, vid2, vid3, vid4, vid5, vid6);

	dsi_write_field(display, EXR_DSI_VID0, vid0);
	dsi_write_field(display, EXR_DSI_VID1, vid1);
	dsi_write_field(display, EXR_DSI_VID2, vid2);
	dsi_write_field(display, EXR_DSI_VID3, vid3);
	dsi_write_field(display, EXR_DSI_VID4, vid4);
	dsi_write_field(display, EXR_DSI_VID5, vid5);
	dsi_write_field(display, EXR_DSI_VID6, vid6);

	return 0;
}

static void dsi_set_phy(struct dsi_display *display, int on)
{
	unsigned int phy0 = 0, phy1 = 0, phy2 = 0, phy3 = 0;

	if (!on) {
		phy0 = BITFLDS(EXR_DSI_PHY0_SHARE, 0x0) |
			BITFLDS(EXR_DSI_PHY0_M, 0) |
			BITFLDS(EXR_DSI_PHY0_N, 0xFF) |
			BITFLDS(EXR_DSI_PHY0_POWERUP, display->dif.dsi.pwup) |
			BITFLDS(EXR_DSI_PHY0_CALIB, display->dif.dsi.calib) |
			BITFLDS(EXR_DSI_PHY0_TO_LP_HS_REQ,
				display->dif.dsi.to_lp_hs_req);
	} else {
		phy0 = BITFLDS(EXR_DSI_PHY0_SHARE, 0x0) |
			BITFLDS(EXR_DSI_PHY0_M, display->dif.dsi.m) |
			BITFLDS(EXR_DSI_PHY0_N, display->dif.dsi.n) |
			BITFLDS(EXR_DSI_PHY0_POWERUP, display->dif.dsi.pwup) |
			BITFLDS(EXR_DSI_PHY0_CALIB, display->dif.dsi.calib) |
			BITFLDS(EXR_DSI_PHY0_TO_LP_HS_REQ,
				display->dif.dsi.to_lp_hs_req);
	}

	phy1 =	BITFLDS(EXR_DSI_PHY1_TO_LP_HS_DIS,
			display->dif.dsi.to_lp_hs_dis) |
		BITFLDS(EXR_DSI_PHY1_TO_LP_EOT,
			display->dif.dsi.to_lp_hs_eot) |
		BITFLDS(EXR_DSI_PHY1_TO_HS_ZERO,
			display->dif.dsi.to_hs_zero) |
		BITFLDS(EXR_DSI_PHY1_TO_HS_FLIP,
			display->dif.dsi.to_hs_flip) |
		BITFLDS(EXR_DSI_PHY1_LP_CLK_DIV,
			display->dif.dsi.lp_clk_div);

	phy2 =	BITFLDS(EXR_DSI_PHY2_HS_CLK_PRE,
			display->dif.dsi.to_hs_clk_pre) |
		BITFLDS(EXR_DSI_PHY2_HS_CLK_POST,
			display->dif.dsi.to_hs_clk_post) |
		BITFLDS(EXR_DSI_PHY2_DAT_DELAY,
			display->dif.dsi.data_delay) |
		BITFLDS(EXR_DSI_PHY2_CLK_DELAY,
			display->dif.dsi.clock_delay) |
		BITFLDS(EXR_DSI_PHY2_LPTX_TFALL,
			display->dif.dsi.lp_tx_tfall);

	phy3 =	BITFLDS(EXR_DSI_PHY3_EN, 0x1) |
		BITFLDS(EXR_DSI_PHY3_LPTX_TRISE,
			display->dif.dsi.lp_tx_trise) |
		BITFLDS(EXR_DSI_PHY3_LPTX_VREF,
			display->dif.dsi.lp_tx_vref);

	DSI_DBG2("MIPI-DSI @%d bps (%d,%d): 0x%08x 0x%08x 0x%08x 0x%08x)\n",
		 dsi_get_rate(display),
		 display->dif.dsi.n, display->dif.dsi.m,
		 phy0, phy1, phy2, phy3);

	dsi_write_field(display, EXR_DSI_PHY0, phy0);
	dsi_write_field(display, EXR_DSI_PHY1, phy1);
	dsi_write_field(display, EXR_DSI_PHY2, phy2);
	dsi_write_field(display, EXR_DSI_PHY3, phy3);

	if (on) {
		/* wait for PLL lock */
		dsi_wait_status(display, EXR_DSI_STAT_DSI_LOCK, 1, 1, 0, 1000);
	}
}

static void dsi_send_msglist(struct dsi_display *display,
			     struct display_msg *msgs)
{
	struct display_msg *msg;

	list_for_each_entry(msg, &msgs->list, list) {
		mdelay(1);
		DSI_DBG2("Sending command 0x%02x of length %d\n",
			 msg->type, msg->length);
		dsi_send_cmd(display, msg);
	}
}

static int dsi_panel_init(struct dsi_display *display)
{
	struct display_msg *msgs = display->msgs_init;

	if (msgs != NULL)
		dsi_send_msglist(display, msgs);

	return 0;
}

int dsi_stop(struct dsi_display *display)
{
	/* Reset and re-init for entering ULPS*/
	dsi_init(display);
	dsi_config(display, DIF_TX_DATA);

	/* Enter ULPS */
	dsi_write_field(display, EXR_DSI_CFG_ULPS, 1);

	/* Swicth off PLL */
	dsi_set_phy(display, 0);

	/* Switch off phy */
	dsi_write_field(display, EXR_DSI_PHY3, BITFLDS(EXR_DSI_PHY3_EN, 0x0));

	return 0;
}

int dsi_init(struct dsi_display *display)
{
	unsigned int clcstat;

	if (display->dsi_reset) {
		reset_control_assert(display->dsi_reset);
		udelay(10);
		reset_control_deassert(display->dsi_reset);
		usleep_range(8000, 8001);
	}

	dsi_write_field(display, EXR_DSI_CLC,
			BITFLDS(EXR_DSI_CLC_RUN, DSI_MODE_RUN));
	clcstat = BITFLDS(EXR_DSI_CLC_STAT_RUN, 1) |
		BITFLDS(EXR_DSI_CLC_STAT_MODEN, 1) |
		BITFLDS(EXR_DSI_CLC_STAT_KID, 1);
	dsi_wait_status(display, EXR_DSI_CLC_STAT, clcstat, clcstat, 0, 1000);
	dsi_write_field(display, EXR_DSI_CLK, 0x000F000F);
	dsi_write_field(display, EXR_DSI_TO0, 0);
	dsi_write_field(display, EXR_DSI_TO1, 0);
#ifdef CONFIG_SONY_CADIZ
	dsi_write_field(display, EXR_DSI_CFG, DSI_CFG_RX_LP_STP(1) | BITFLDS(EXR_DSI_CFG_GATE, 1));
#else
	dsi_write_field(display, EXR_DSI_CFG, DSI_CFG_RX_LP_STP(1));
#endif
	dsi_interrupt_setup(display);

	return 0;
}

static void
dsi_set_gpiolist(struct dsi_display *display, struct display_gpio *gpios)
{
	struct display_gpio *gpio;

	list_for_each_entry(gpio, &gpios->list, list) {
		switch (gpio->type) {
		case DSI_GPIO_VHIGH:
			if (!display->gpio_vhigh)
				break;

			gpio_direction_output(display->gpio_vhigh, gpio->value);
			break;

		case DSI_GPIO_VLOW:
			if (!display->gpio_vlow)
				break;

			gpio_direction_output(display->gpio_vlow, gpio->value);
			break;

		case DSI_GPIO_RESET:
			if (!display->gpio_reset)
				break;

			gpio_direction_output(display->gpio_reset, gpio->value);
			break;
		}

		if (gpio->delay)
			mdelay(gpio->delay);
	}
}

static void dsi_panel_power_on(struct dsi_display *display)
{
	if (display->gpios_power_on)
		dsi_set_gpiolist(display, display->gpios_power_on);
}

static void dsi_panel_lcd_reset(struct dsi_display *display)
{
	if (display->gpios_lcd_reset)
		dsi_set_gpiolist(display, display->gpios_lcd_reset);
}

static void dsi_panel_reset_low(struct dsi_display *display)
{
	if (display->gpios_reset_low)
		dsi_set_gpiolist(display, display->gpios_reset_low);
}

static void dsi_panel_power_off(struct dsi_display *display)
{
	if (display->gpios_power_off)
		dsi_set_gpiolist(display, display->gpios_power_off);
}

static int dsi_panel_sleep_in(struct dsi_display *display)
{
	struct display_msg *msgs = display->msgs_sleep_in;

	if (msgs != NULL)
		dsi_send_msglist(display, msgs);

	return 0;
}

static int dsi_panel_sleep_out(struct dsi_display *display)
{
	struct display_msg *msgs = display->msgs_sleep_out;

	if (msgs != NULL)
		dsi_send_msglist(display, msgs);

	return 0;
}

void dsi_config(struct dsi_display *display, int type)
{
	unsigned int dsicfg;

	if (type == DIF_TX_DATA) {
#ifdef CONFIG_SONY_CADIZ
		dsi_write_field(display, EXR_DSI_CFG, DSI_CFG_OFF(DSI_CMD) | BITFLDS(EXR_DSI_CFG_GATE, 1));
#else
		dsi_write_field(display, EXR_DSI_CFG, DSI_CFG_OFF(DSI_CMD));
#endif
		dsi_write_field(display, EXR_DSI_IMSC, DSI_IRQ_ERR_MASK);
		dsi_set_phy(display, 1);
#ifdef CONFIG_SONY_CADIZ
		dsi_write_field(display, EXR_DSI_CFG,
				DSI_CFG_INIT(display->dif.dsi.nblanes) | BITFLDS(EXR_DSI_CFG_GATE, 1));
#else
		dsi_write_field(display, EXR_DSI_CFG,
				DSI_CFG_INIT(display->dif.dsi.nblanes));
#endif
		return;
	}

	dsi_write_field(display, EXR_DSI_CFG, DSI_CFG_OFF(DSI_VIDEO));
	dsi_write_field(display, EXR_DSI_IMSC,
			DSI_IRQ_ERR_MASK & (~DSI_IRQ_ERR_DSIFIN));
	dsi_configure_video_mode(display,
			PIXELS_TO_BYTES(display->xres, display->bpp),
			display->yres);
	dsi_set_phy(display, 1);
	dsicfg = DSI_CFG_TX_HS_PIXEL(display->dif.dsi.nblanes,
					display->dif.dsi.mode);
	dsi_write_field(display, EXR_DSI_CFG, dsicfg);
}


#ifdef CONFIG_SONY_CADIZ
extern bool cadiz_support;
#endif
static void dsi_dphy_calculation(struct dsi_display *display)
{
	int ui_ps = 0, ths_prepare_ns, ths_trail_ns, ths_prepare_zero_ns,
		tclk_post_ns, tclk_prepare_ns;

	if (display->dif.dsi.bitrate)
		ui_ps = DIV_ROUND_CLOSEST(1000000000,
					  display->dif.dsi.bitrate / 1000);

	/*
	 * THS-PREPARE is between 40ns + 4UI and 85ns + 6UI,
	 * we set the THS-PREPARE to average of THS-PREPARE.
	 * THS-PREPARE = (40ns + 4UI + 85ns + 6UI) / 2 = 62.5ns + 5UI
	 */
	ths_prepare_ns = DIV_ROUND_CLOSEST(62500 + 5 * ui_ps, 1000);

	/*
	 * THS-TRAIL is 60ns + 4UI, we set THS-TRAIL to 63ns + 4UI for safety
	 * margin.
	 */
	ths_trail_ns = DIV_ROUND_UP(63000 + 4 * ui_ps, 1000);

	/*
	 * THS-PREPARE + THS-ZERO is 145ns + 10UI, we set THS-PREPARE + THS-ZERO
	 * to 152.25ns + 11UI for safety margin.
	 */
	ths_prepare_zero_ns = DIV_ROUND_UP(152250 + 11 * ui_ps, 1000);

	/*
	 * TCLK-POST is 60ns + 52UI, we set TCLK-POST to 63ns + 55UI for safety
	 * margin.
	 */
	tclk_post_ns = DIV_ROUND_UP(63000 + 55 * ui_ps, 1000);

	/*
	 * TCLK-PREPARE is between 38ns and 95ns, we set TCLK-PREPARE to average
	 * of TCLK-PREPARE 67.
	 * margin.
	 */
	tclk_prepare_ns = 67;

	display->dif.dsi.pwup = 6;
	display->dif.dsi.calib = 3;
	display->dif.dsi.data_delay = 7;
	display->dif.dsi.clock_delay = 7;
	display->dif.dsi.lp_tx_tfall = 2;
	display->dif.dsi.lp_tx_trise = 2;
	display->dif.dsi.lp_tx_vref = 31;
	display->dif.dsi.lp_clk_div =
		DIV_ROUND_UP(display->dif.dsi.dc_clk_rate / 1000 * TLPX_NS,
		1000000);
	display->dif.dsi.to_lp_hs_req = display->dif.dsi.lp_clk_div;
	display->dif.dsi.to_hs_flip = DIV_ROUND_UP(display->dif.dsi.bitrate /
		1000 * ths_trail_ns, 1000000 * 8);
	display->dif.dsi.to_hs_zero = DIV_ROUND_UP(display->dif.dsi.bitrate /
		1000 * ths_prepare_zero_ns, 1000000 * 8) - 5;
	display->dif.dsi.to_lp_hs_eot =
		DIV_ROUND_UP(display->dif.dsi.dc_clk_rate / 1000 *
		(ths_trail_ns + 18), 1000000) + DIV_ROUND_UP(3000000,
		display->dif.dsi.bitrate / 1000);
	display->dif.dsi.to_lp_hs_dis =
		DIV_ROUND_UP(display->dif.dsi.dc_clk_rate / 1000 *
		ths_prepare_ns, 1000000) - 1;
	display->dif.dsi.to_hs_clk_post =
		DIV_ROUND_UP(display->dif.dsi.dc_clk_rate / 1000 * tclk_post_ns,
		1000000);
	display->dif.dsi.to_hs_clk_pre =
		DIV_ROUND_UP(display->dif.dsi.dc_clk_rate / 1000 *
		tclk_prepare_ns, 1000000) + 5;

	if (!cadiz_support) {
		printk("[DISPLAY] ths_prepare_zero(%d, %d)\n",
				display->dif.dsi.to_lp_hs_dis, display->dif.dsi.to_hs_zero);
#if defined(CONFIG_Z380C)
		display->dif.dsi.to_lp_hs_dis = 35;
		display->dif.dsi.to_hs_zero = 9;
#elif defined(CONFIG_Z300C) || defined(CONFIG_Z300CG)
		display->dif.dsi.to_lp_hs_dis = 34;
		display->dif.dsi.to_hs_zero = 12;
#endif
		printk("[DISPLAY] adjust to ths_prepare_zero(%d, %d)\n",
				display->dif.dsi.to_lp_hs_dis, display->dif.dsi.to_hs_zero);
	}
}

static void dsi_rate_calculation(struct dsi_display *display)
{
	int diff, diff_min = DSI_RATE_MAX, n = 0, m = 0;
	int dsi_daterate = 0;

#if 1
	display->dif.dsi.bitrate = DSI_RATE_OVERHEAD((display->xres +
		BYTES_TO_PIXELS(display->dif.dsi.hfp, display->bpp) +
		BYTES_TO_PIXELS(display->dif.dsi.hbp, display->bpp) +
		BYTES_TO_PIXELS(display->dif.dsi.hsa, display->bpp)) *
		(display->yres + display->dif.dsi.vfp +
		display->dif.dsi.vbp + display->dif.dsi.vsa) *
		display->fps / display->dif.dsi.nblanes * display->bpp);
#else
	dsi_daterate = (display->xres +
		BYTES_TO_PIXELS(display->dif.dsi.hfp, display->bpp) +
		BYTES_TO_PIXELS(display->dif.dsi.hbp, display->bpp) +
		BYTES_TO_PIXELS(display->dif.dsi.hsa, display->bpp)) *
		(display->yres + display->dif.dsi.vfp +
		 display->dif.dsi.vbp + display->dif.dsi.vsa) *
		 display->fps / display->dif.dsi.nblanes * display->bpp;
#endif

	printk("[DISPLAY] hbp=%d hpw=%d hfp=%d, vbp=%d vpw=%d vfp=%d\n",
			(display->dif.dsi.hbp/3), (display->dif.dsi.hsa/3), (display->dif.dsi.hfp/3),
			display->dif.dsi.vbp, display->dif.dsi.vsa, display->dif.dsi.vfp);

	if (dsi_daterate > display->dif.dsi.bitrate)
		display->dif.dsi.bitrate = dsi_daterate;

	if (display->dif.dsi.bitrate > DSI_RATE_MAX)
		display->dif.dsi.bitrate = DSI_RATE_MAX;
	else if (display->dif.dsi.bitrate < DSI_RATE_MIN)
		display->dif.dsi.bitrate = DSI_RATE_MIN;
	printk("[DISPLAY] dif.dsi.bitrate: %d (%d)\n", display->dif.dsi.bitrate, dsi_daterate);

	for (m = 1; m <= DSI_RATE_M_MAX; m++) {
		for (n = 1; n <= DSI_RATE_N_MAX; n++) {
			diff = DSI_RATE(n, m) - display->dif.dsi.bitrate;

			if (diff < 0)
				continue;

			if (diff < diff_min) {
				diff_min = diff;
				display->dif.dsi.n = n;
				display->dif.dsi.m = m;
			}
		}
	}
}

int dsi_probe(struct dsi_display *display)
{
	display->panel_init = dsi_panel_init;
	display->sleep_in = dsi_panel_sleep_in;
	display->sleep_out = dsi_panel_sleep_out;
	display->power_on = dsi_panel_power_on;
	display->lcd_reset = dsi_panel_lcd_reset;
	display->reset_low = dsi_panel_reset_low;
	display->power_off = dsi_panel_power_off;
	init_completion(&display->sync.dsifin);
	display->sync.dsifin_to = 200;
	dsi_rate_calculation(display);
	dsi_dphy_calculation(display);

	return 0;
}
