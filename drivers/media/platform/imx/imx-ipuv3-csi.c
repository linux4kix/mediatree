/*
 * i.MX5/6 Camera Sensor Interface V4L2 Subdev
 *
 * Copyright (c) 2012-2014 Mentor Graphics Inc.
 * Copyright 2004-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-of.h>
#include <media/v4l2-ctrls.h>
#include <video/imx-ipu-v3.h>
#include <media/imx.h>
#include "imx-common.h"

#define DEVICE_NAME "imx-ipuv3-csi"

enum {
	IMXCSI_PAD_SINK = 0,
	IMXCSI_PAD_SOURCE,
	IMXCSI_NUM_PADS,
};

struct imxcsi_pixfmt {
	const char *name;
	u32 mbus_code;
	u32 fourcc;
	u32 bytes_per_pixel; /* memory */
	int bytes_per_sample; /* mbus */
	unsigned rgb:1;
	unsigned yuv:1;
	unsigned raw:1;
};

struct imxcsi_subdev {
	struct device *dev;
	struct v4l2_subdev sd;
	struct v4l2_subdev *sink_sd;    /* the downstream subdev */
	struct imx_media_link *link_up; /* the upstream link */
	struct media_pad pad[IMXCSI_NUM_PADS];

	struct ipu_soc *ipu;
	struct ipuv3_channel *ipuch;
	struct ipu_smfc *smfc;
	struct ipu_csi *csi;
	int id; /* the CSI id */

	struct v4l2_mbus_config mbus_cfg; /* the current media bus config */
	struct imxcsi_pixfmt pixfmt[IMXCSI_NUM_PADS];
	struct v4l2_mbus_framefmt format[IMXCSI_NUM_PADS]; /* the current
							      frame format */
	struct v4l2_of_endpoint endpoint;

	/* active (undergoing DMA) buffers, one for each IPU buffer */
	struct imx_buffer *active_frame[2];
	struct imx_dma_buf underrun_buf;

	/* current CSI window */
	struct v4l2_rect crop;

	int buf_num;

	int eof_irq;
	int nfb4eof_irq;

	spinlock_t irqlock;

	bool streaming;
	bool sink_is_mem; /* write to memory (not CSI->IC or CSI->VDI) */

	bool last_eof;  /* waiting for last EOF at stream off */
	struct completion last_eof_comp;
};

#define to_imxcsi(sd) container_of(sd, struct imxcsi_subdev, sd)

static struct imxcsi_pixfmt imxcsi_pixfmts[] = {
	{
		.name = "Monochrome 8 bit",
		.fourcc = V4L2_PIX_FMT_GREY,
		.mbus_code = V4L2_MBUS_FMT_Y8_1X8,
		.bytes_per_pixel = 1,
		.bytes_per_sample = 1,
		.raw = 1,
	}, {
		.name = "Monochrome 10 bit",
		.fourcc = V4L2_PIX_FMT_Y10,
		.mbus_code = V4L2_MBUS_FMT_Y10_1X10,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 2,
		.raw = 1,
	}, {
		.name = "Monochrome 12 bit",
		.fourcc = V4L2_PIX_FMT_Y16,
		.mbus_code = V4L2_MBUS_FMT_Y12_1X12,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 2,
		.raw = 1,
	}, {
		.name = "UYVY 2x8 bit",
		.fourcc = V4L2_PIX_FMT_UYVY,
		.mbus_code = V4L2_MBUS_FMT_UYVY8_2X8,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 1,
		.yuv = 1,
	}, {
		.name = "YUYV 2x8 bit",
		.fourcc = V4L2_PIX_FMT_YUYV,
		.mbus_code = V4L2_MBUS_FMT_YUYV8_2X8,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 1,
		.yuv = 1,
	}, {
		.name = "UYVY 1x16 bit",
		.fourcc = V4L2_PIX_FMT_UYVY,
		.mbus_code = V4L2_MBUS_FMT_UYVY8_1X16,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 2,
		.raw = 1,
	}, {
		.name = "YUYV 1x16 bit",
		.fourcc = V4L2_PIX_FMT_YUYV,
		.mbus_code = V4L2_MBUS_FMT_YUYV8_1X16,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 2,
		.raw = 1,
	},
};

static struct imxcsi_pixfmt *imxcsi_find_format(const u32 *fourcc,
						const u32 *mbus_code)
{
	struct imxcsi_pixfmt *fmt;
	int i;

	for (i = 0; i < ARRAY_SIZE(imxcsi_pixfmts); i++) {
		fmt = &imxcsi_pixfmts[i];
		if (fourcc && *fourcc == fmt->fourcc)
			return fmt;
		if (mbus_code && *mbus_code == fmt->mbus_code)
			return fmt;
	}

	return NULL;
}

static int imxcsi_link_setup(struct media_entity *entity,
			     const struct media_pad *local,
			     const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct v4l2_subdev *remote_sd =
		media_entity_to_v4l2_subdev(remote->entity);
	struct imxcsi_subdev *imxcsi = to_imxcsi(sd);
	int ret;

	dev_dbg(imxcsi->dev, "link setup %s -> %s\n",
		remote->entity->name, local->entity->name);

	if (imxcsi->streaming)
		return -EBUSY;

	switch (local->index) {
	case IMXCSI_PAD_SINK:
		/* remote is the imx-video-mux, ask it what the current
		   bus config is */
		ret = v4l2_subdev_call(remote_sd, video, g_mbus_config,
				       &imxcsi->mbus_cfg);
		if (ret)
			return ret;

		/* select either parallel or MIPI-CSI2 as CSI source */
		ipu_csi_set_src(imxcsi->csi,
				imxcsi->mbus_cfg.type == V4L2_MBUS_CSI2);
		break;
	case IMXCSI_PAD_SOURCE:
		imxcsi->sink_sd = remote_sd;
		if (!strcmp(remote_sd->name, IMX_MEDIA_SUBDEV_DEVNODE) ||
		    !strcmp(remote_sd->name, IMX_MEDIA_SUBDEV_VDIC)) {
			/* set CSI destination to direct to mem via SMFC */
			/* FIXME: direct CSI->VDIC path needs
			   IPU_CSI_DEST_VDIC */
			ipu_csi_set_dest(imxcsi->csi, IPU_CSI_DEST_IDMAC);
			imxcsi->sink_is_mem = true;
		} else {
			/* assume CSI destination is IC */
			ipu_csi_set_dest(imxcsi->csi, IPU_CSI_DEST_IC);
			imxcsi->sink_is_mem = false;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void imxcsi_put_ipu_resources(struct imxcsi_subdev *imxcsi)
{
	if (!IS_ERR_OR_NULL(imxcsi->ipuch))
		ipu_idmac_put(imxcsi->ipuch);
	imxcsi->ipuch = NULL;

	if (!IS_ERR_OR_NULL(imxcsi->smfc))
		ipu_smfc_put(imxcsi->smfc);
	imxcsi->smfc = NULL;

	if (!IS_ERR_OR_NULL(imxcsi->csi))
		ipu_csi_put(imxcsi->csi);
	imxcsi->csi = NULL;
}

static int imxcsi_get_ipu_resources(struct imxcsi_subdev *imxcsi)
{
	int csi_ch_num, ret;

	imxcsi->csi = ipu_csi_get(imxcsi->ipu, imxcsi->id);
	if (IS_ERR(imxcsi->csi)) {
		dev_err(imxcsi->dev, "failed to get CSI %d\n", imxcsi->id);
		return PTR_ERR(imxcsi->csi);
	}

	/* the rest is unneeded if writing directly to IC/VDIC */
	if (!imxcsi->sink_is_mem)
		return 0;

	/*
	 * Choose the CSI-->SMFC-->MEM channel corresponding
	 * to the IPU and CSI IDs.
	 */
	csi_ch_num = IPUV3_CHANNEL_CSI0 +
		(ipu_get_num(imxcsi->ipu) << 1) + imxcsi->id;

	imxcsi->smfc = ipu_smfc_get(imxcsi->ipu, csi_ch_num);
	if (IS_ERR(imxcsi->smfc)) {
		dev_err(imxcsi->dev, "failed to get SMFC\n");
		ret = PTR_ERR(imxcsi->smfc);
		goto out;
	}

	imxcsi->ipuch = ipu_idmac_get(imxcsi->ipu, csi_ch_num);
	if (IS_ERR(imxcsi->ipuch)) {
		dev_err(imxcsi->dev, "could not get IDMAC channel %u\n",
			 csi_ch_num);
		ret = PTR_ERR(imxcsi->ipuch);
		goto out;
	}

	return 0;
out:
	imxcsi_put_ipu_resources(imxcsi);
	return ret;
}

static irqreturn_t imxcsi_eof_interrupt(int irq, void *dev_id)
{
	struct imxcsi_subdev *imxcsi = dev_id;
	struct imx_buffer *frame, *next_frame;
	unsigned long flags;
	dma_addr_t phys;

	spin_lock_irqsave(&imxcsi->irqlock, flags);

	/* timestamp and return the completed frame */
	frame = imxcsi->active_frame[imxcsi->buf_num];
	if (frame) {
		/* pass frame downstream and get another */
		imx_video_buffer_done(imxcsi->sink_sd, frame,
				      VB2_BUF_STATE_DONE);
	}

	if (imxcsi->last_eof) {
		complete(&imxcsi->last_eof_comp);
		imxcsi->active_frame[imxcsi->buf_num] = NULL;
		imxcsi->last_eof = false;
		goto unlock;
	}

	next_frame = imx_video_buffer_next(imxcsi->sink_sd);

	if (next_frame) {
		phys = next_frame->phys;
		imxcsi->active_frame[imxcsi->buf_num] = next_frame;
	} else {
		phys = imxcsi->underrun_buf.phys;
		imxcsi->active_frame[imxcsi->buf_num] = NULL;
	}

	ipu_cpmem_set_buffer(imxcsi->ipuch, imxcsi->buf_num, phys);
	ipu_idmac_select_buffer(imxcsi->ipuch, imxcsi->buf_num);

	imxcsi->buf_num ^= 1;

unlock:
	spin_unlock_irqrestore(&imxcsi->irqlock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t imxcsi_nfb4eof_interrupt(int irq, void *dev_id)
{
	struct imxcsi_subdev *imxcsi = dev_id;

	dev_err(imxcsi->dev, "NFB4EOF\n");

	/*
	 * It has been discovered that with rotation, stream off
	 * creates a single NFB4EOF event which is 100% repeatable. So
	 * scheduling a restart causes an endless NFB4EOF-->restart
	 * cycle. The error itself seems innocuous, capture is not
	 * adversely affected.
	 *
	 * So don't schedule a restart on NFB4EOF error if we are sending
	 * to IC with possible rotation. If the cause of the NFB4EOF event
	 * on disable is ever found, it can be re-enabled, but is probably
	 * not necessary. Detecting the interrupt (and clearing the irq
	 * status in the IPU) seems to be enough.
	 */
	/* FIXME: event? */
	if (imxcsi->sink_is_mem)
		v4l2_subdev_notify(&imxcsi->sd, IMX_NFB4EOF_NOTIFY, NULL);

	return IRQ_HANDLED;
}

static void imxcsi_setup_channel(struct imxcsi_subdev *imxcsi)
{
	struct v4l2_mbus_framefmt *f = &imxcsi->format[IMXCSI_PAD_SOURCE];
	struct imxcsi_pixfmt *srcfmt = &imxcsi->pixfmt[IMXCSI_PAD_SOURCE];
	struct imxcsi_pixfmt *sinkfmt = &imxcsi->pixfmt[IMXCSI_PAD_SINK];
	unsigned int burst_size;
	struct ipu_image image;
	u32 stride;

	ipu_cpmem_zero(imxcsi->ipuch);

	stride = ipu_stride_to_bytes(f->width, srcfmt->fourcc);

	memset(&image, 0, sizeof(image));
	image.pix.width = image.rect.width = f->width;
	image.pix.height = image.rect.height = f->height;
	image.pix.bytesperline = stride;
	image.pix.pixelformat = srcfmt->fourcc;
	image.phys0 = imxcsi->active_frame[0]->phys;
	if (imxcsi->active_frame[1])
		image.phys1 = imxcsi->active_frame[1]->phys;
	else
		image.phys1 = imxcsi->underrun_buf.phys;

	ipu_cpmem_set_image(imxcsi->ipuch, &image);

	burst_size = (f->width % 16) ? 8 : 16;
	ipu_cpmem_set_burstsize(imxcsi->ipuch, burst_size);

	if (sinkfmt->raw)
		ipu_cpmem_set_format_passthrough(imxcsi->ipuch,
						 srcfmt->bytes_per_sample * 8);

	if (imxcsi->mbus_cfg.type == V4L2_MBUS_CSI2)
		ipu_smfc_map_channel(imxcsi->smfc, imxcsi->id,
				     1 /* FIXME: vc # */);
	else
		ipu_smfc_map_channel(imxcsi->smfc, imxcsi->id, 0);

	/*
	 * Set the channel to very high priority, by enabling the watermark
	 * signal in the SMFC, enabling WM in the channel, and setting
	 * the channel priority to high.
	 *
	 * Refer to the iMx6 rev. D TRM Table 36-8: Calculated priority
	 * value.
	 *
	 * The WM's are set very low by intention here to ensure that
	 * the SMFC FIFOs do not overflow.
	 */
	ipu_smfc_set_watermark(imxcsi->smfc, 0x02, 0x01);
	ipu_cpmem_set_high_priority(imxcsi->ipuch);
	ipu_idmac_enable_watermark(imxcsi->ipuch, true);
	ipu_cpmem_set_axi_id(imxcsi->ipuch, 0);
	ipu_idmac_lock_enable(imxcsi->ipuch, 8);

	/* FIXME */
	ipu_smfc_set_burstsize(imxcsi->smfc, burst_size);

	/* FIXME: Don't do this when sink is VDIC */
	if (ipu_csi_is_interlaced(imxcsi->csi))
		ipu_cpmem_interlaced_scan(imxcsi->ipuch, stride);

	ipu_idmac_set_double_buffer(imxcsi->ipuch, true);
}

static int imxcsi_setup(struct imxcsi_subdev *imxcsi)
{
	struct v4l2_mbus_config *cfg = &imxcsi->mbus_cfg;
	struct v4l2_mbus_framefmt *f = &imxcsi->format[IMXCSI_PAD_SOURCE];
	struct imxcsi_pixfmt *srcfmt = &imxcsi->pixfmt[IMXCSI_PAD_SOURCE];
	struct imx_buffer *frame;
	int size, ret;

	ipu_csi_set_window(imxcsi->csi, &imxcsi->crop);
	ipu_csi_init_interface(imxcsi->csi, cfg, f);
	if (cfg->type == V4L2_MBUS_CSI2)
		ipu_csi_set_mipi_datatype(imxcsi->csi, /* FIXME: vc # */ 1, f);

	/* the rest is unneeded if writing directly to IC/VDIC */
	if (!imxcsi->sink_is_mem)
		return 0;

	frame = imx_video_buffer_next(imxcsi->sink_sd);
	if (!frame) {
		dev_err(imxcsi->dev, "no downstream frame available\n");
		return -EINVAL;
	}
	imxcsi->active_frame[0] = frame;
	frame = imx_video_buffer_next(imxcsi->sink_sd);
	imxcsi->active_frame[1] = frame;

	imxcsi->buf_num = 0;

	size = f->width * f->height * srcfmt->bytes_per_pixel;
	ret = imx_alloc_dma_buf(imxcsi->dev, &imxcsi->underrun_buf, size);
	if (ret) {
		dev_err(imxcsi->dev, "failed to alloc underrun_buf, %d\n", ret);
		return ret;
	}

	imxcsi->nfb4eof_irq = ipu_idmac_channel_irq(imxcsi->ipu,
						    imxcsi->ipuch,
						    IPU_IRQ_NFB4EOF);
	ret = devm_request_irq(imxcsi->dev, imxcsi->nfb4eof_irq,
			       imxcsi_nfb4eof_interrupt, 0,
			       "imxcsi-nfb4eof", imxcsi);
	if (ret) {
		dev_err(imxcsi->dev,
			"failed to register NFB4EOF irq: %d\n", ret);
		goto out_free_underrun;
	}

	imxcsi->eof_irq = ipu_idmac_channel_irq(imxcsi->ipu,
						imxcsi->ipuch,
						IPU_IRQ_EOF);

	ret = devm_request_irq(imxcsi->dev, imxcsi->eof_irq,
			       imxcsi_eof_interrupt, 0,
			       "imxcsi-eof", imxcsi);
	if (ret) {
		dev_err(imxcsi->dev,
			"Error registering EOF irq: %d\n", ret);
		goto out_free_nfb4eof_irq;
	}

	imxcsi_setup_channel(imxcsi);

	ipu_smfc_enable(imxcsi->smfc);

	/* set buffers ready */
	ipu_idmac_select_buffer(imxcsi->ipuch, 0);
	ipu_idmac_select_buffer(imxcsi->ipuch, 1);

	/* enable the channel */
	ipu_idmac_enable_channel(imxcsi->ipuch);

	return 0;

out_free_underrun:
	imx_free_dma_buf(imxcsi->dev, &imxcsi->underrun_buf);
out_free_nfb4eof_irq:
	devm_free_irq(imxcsi->dev, imxcsi->nfb4eof_irq, imxcsi);
	return ret;
}

static int imxcsi_start(struct imxcsi_subdev *imxcsi)
{
	int ret;

	ret = imxcsi_get_ipu_resources(imxcsi);
	if (ret)
		return ret;

	ret = imxcsi_setup(imxcsi);
	if (ret)
		goto out_put_ipu;

	ipu_csi_enable(imxcsi->csi);

	return 0;

out_put_ipu:
	imxcsi_put_ipu_resources(imxcsi);
	return ret;
}

static int imxcsi_stop(struct imxcsi_subdev *imxcsi)
{
	struct imx_buffer *frame;
	int i, ret;

	if (imxcsi->sink_is_mem) {
		/*
		 * Mark next EOF interrupt as the last before stream off,
		 * and then wait for interrupt handler to mark completion.
		 */
		init_completion(&imxcsi->last_eof_comp);
		imxcsi->last_eof = true;
		ret = wait_for_completion_timeout(
			&imxcsi->last_eof_comp,
			msecs_to_jiffies(IMX_EOF_TIMEOUT));
		if (ret == 0)
			dev_warn(imxcsi->dev, "wait last EOF timeout\n");

		ipu_csi_disable(imxcsi->csi);

		devm_free_irq(imxcsi->dev, imxcsi->eof_irq, imxcsi);
		devm_free_irq(imxcsi->dev, imxcsi->nfb4eof_irq, imxcsi);

		ipu_idmac_disable_channel(imxcsi->ipuch);

		ipu_smfc_disable(imxcsi->smfc);

		imx_free_dma_buf(imxcsi->dev, &imxcsi->underrun_buf);

		/* return any remaining active frames with error */
		for (i = 0; i < 2; i++) {
			frame = imxcsi->active_frame[i];
			if (frame && frame->vb.state == VB2_BUF_STATE_ACTIVE)
				imx_video_buffer_done(imxcsi->sink_sd, frame,
						      VB2_BUF_STATE_ERROR);
		}
	} else {
		ipu_csi_disable(imxcsi->csi);
	}

	imxcsi_put_ipu_resources(imxcsi);

	return 0;
}

static int imxcsi_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct imxcsi_subdev *imxcsi = v4l2_get_subdevdata(sd);

	if (enable)
		return imxcsi_start(imxcsi);
	else
		return imxcsi_stop(imxcsi);
}

static struct v4l2_mbus_framefmt * imxcsi_get_pad_format(
	struct imxcsi_subdev *imxcsi, struct v4l2_subdev_fh *fh,
	unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imxcsi->format[pad];
	default:
		return NULL;
	}
}

static int imxcsi_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	switch (code->pad) {
	case IMXCSI_PAD_SINK:
		if (code->index >= ARRAY_SIZE(imxcsi_pixfmts))
			return -EINVAL;

                code->code = imxcsi_pixfmts[code->index].mbus_code;
                break;
	case IMXCSI_PAD_SOURCE:
		/* TODO */
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int imxcsi_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	/* TODO */
	return 0;
}

static int imxcsi_get_format(struct v4l2_subdev *sd,
			    struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_format *sdformat)
{
	struct imxcsi_subdev *imxcsi = v4l2_get_subdevdata(sd);

	if (sdformat->pad >= IMXCSI_NUM_PADS)
		return -EINVAL;

	sdformat->format = *imxcsi_get_pad_format(imxcsi, fh, sdformat->pad,
						  sdformat->which);
	return 0;
}

static int imxcsi_set_format(struct v4l2_subdev *sd,
			    struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_format *sdformat)
{
	struct imxcsi_subdev *imxcsi = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *mbusformat;

	if (sdformat->pad >= IMXCSI_NUM_PADS)
		return -EINVAL;

	mbusformat = imxcsi_get_pad_format(imxcsi, fh, sdformat->pad,
					   sdformat->which);
	if (!mbusformat)
		return -EINVAL;

	/* TODO, for now just set blindly  */
	*mbusformat = sdformat->format;

	return 0;
}

static int imxcsi_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_selection *sel)
{
	struct imxcsi_subdev *imxcsi = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	if (sel->pad != IMXCSI_PAD_SOURCE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		/* crop bounds is the raw sensor frame at our sink pad */
		format = imxcsi_get_pad_format(imxcsi, fh, IMXCSI_PAD_SINK,
					       sel->which);
		sel->r.left = sel->r.top = 0;
		sel->r.width = format->width;
		sel->r.height = format->height;
		break;
	case V4L2_SEL_TGT_CROP:
		sel->r = imxcsi->crop;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int imxcsi_set_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_selection *sel)
{
	struct imxcsi_subdev *imxcsi = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	if (sel->target != V4L2_SEL_TGT_CROP || sel->pad != IMXCSI_PAD_SOURCE)
		return -EINVAL;

	if (imxcsi->streaming)
		return -EBUSY;

	/*
	 * Modifying the crop rectangle always changes the format on the source
	 * pad. If the KEEP_CONFIG flag is set, just return the current crop
	 * rectangle.
	 */
	if (sel->flags & V4L2_SEL_FLAG_KEEP_CONFIG) {
		sel->r = imxcsi->crop;
		return 0;
	}

	/*
	 * FIXME: the IPU currently does not setup the CCIR code
	 * registers for interlaced BT.656 properly to handle arbitrary
	 * crop windows. So ignore this request if the sensor bus is BT.656.
	 */
	if (imxcsi->mbus_cfg.type == V4L2_MBUS_BT656)
		return 0;

	format = imxcsi_get_pad_format(imxcsi, fh,
				       IMXCSI_PAD_SINK, sel->which);
	/* make sure crop window is within bounds */
	if (sel->r.left + sel->r.width > format->width)
		sel->r.width = format->width - sel->r.left;
	if (sel->r.top + sel->r.height > format->height)
		sel->r.height = format->height - sel->r.top;

	/* adjust crop window to h/w alignment restrictions */
	sel->r.width &= ~0x7;
	sel->r.left &= ~0x3;

	/* Update the source format. */
	format = imxcsi_get_pad_format(imxcsi, fh,
				       IMXCSI_PAD_SOURCE, sel->which);
	format->width = sel->r.width;
	format->height = sel->r.height;

	return 0;
}

static struct v4l2_subdev_pad_ops imxcsi_pad_ops = {
	.enum_mbus_code = imxcsi_enum_mbus_code,
	.enum_frame_size = imxcsi_enum_frame_size,
	.get_fmt = imxcsi_get_format,
	.set_fmt = imxcsi_set_format,
	.get_selection = imxcsi_get_selection,
	.set_selection = imxcsi_set_selection,
};

static struct v4l2_subdev_video_ops imxcsi_video_ops = {
	.s_stream = imxcsi_s_stream,
};

static struct v4l2_subdev_ops imxcsi_subdev_ops = {
	.pad   = &imxcsi_pad_ops,
	.video = &imxcsi_video_ops,
};

struct media_entity_operations imxcsi_entity_ops = {
	.link_setup    = imxcsi_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

static int imxcsi_subdev_init(struct imxcsi_subdev *imxcsi,
			      struct device_node *node)
{
	struct device_node *endpoint;
	int ret;

	v4l2_subdev_init(&imxcsi->sd, &imxcsi_subdev_ops);

	v4l2_set_subdevdata(&imxcsi->sd, imxcsi);

	snprintf(imxcsi->sd.name, sizeof(imxcsi->sd.name), "%s%d",
		 dev_name(imxcsi->dev), imxcsi->id);

	endpoint = of_get_next_child(node, NULL);
	if (endpoint) {
		v4l2_of_parse_endpoint(endpoint, &imxcsi->endpoint);
		of_node_put(endpoint);
	}

	imxcsi->sd.entity.ops = &imxcsi_entity_ops;

	imxcsi->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	imxcsi->pad[IMXCSI_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	imxcsi->pad[IMXCSI_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_init(&imxcsi->sd.entity, IMXCSI_NUM_PADS,
				imxcsi->pad, 0);
	if (ret < 0)
		return ret;

	ret = v4l2_async_register_subdev(&imxcsi->sd);
	if (ret)
		return ret;

	return 0;
}

static int imxcsi_async_init(struct imxcsi_subdev *imxcsi,
			     struct device_node *node)
{
	struct device_node *endpoint, *rp, *rpp;
	struct v4l2_async_subdev asd = {0};
	uint32_t remote_portno;

	endpoint = of_get_next_child(node, NULL);
	if (!endpoint)
		return 0;

	rp = of_graph_get_remote_port(endpoint);
	rpp = of_graph_get_remote_port_parent(endpoint);
	of_node_put(endpoint);

	of_property_read_u32(rp, "reg", &remote_portno);
	of_node_put(rp);

	asd.match_type = V4L2_ASYNC_MATCH_OF;
	asd.match.of.node = rpp;

	imxcsi->link_up =
		imx_media_entity_create_link(
			&imxcsi->sd, IMXCSI_PAD_SINK, remote_portno, &asd,
			MEDIA_PAD_FL_SINK,
			MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
	if (IS_ERR(imxcsi->link_up)) {
		imxcsi->link_up = NULL;
		return PTR_ERR(imxcsi->link_up);
	}

	return 0;
}

static struct device_node *imxcsi_get_port(struct device_node *ipu_node,
					   int id)
{
	struct device_node *port;
	int reg;

	for_each_child_of_node(ipu_node, port) {
		if (!of_property_read_u32(port, "reg", &reg) && reg == id)
			return of_node_get(port);
	}

	return NULL;
}

static int imxcsi_probe(struct platform_device *pdev)
{
	struct ipu_client_platformdata *pdata = pdev->dev.platform_data;
	struct device *ipu_dev = pdev->dev.parent;
	struct ipu_soc *ipu = dev_get_drvdata(ipu_dev);
	struct imxcsi_subdev *imxcsi;
	struct device_node *port;
	int ret;

	imxcsi = devm_kzalloc(&pdev->dev, sizeof(*imxcsi), GFP_KERNEL);
	if (!imxcsi)
		return -ENOMEM;

	imxcsi->dev = imxcsi->sd.dev = &pdev->dev;
	imxcsi->id = pdata->csi; /* CSI0 or CSI1 */
	imxcsi->ipu = ipu;

	spin_lock_init(&imxcsi->irqlock);

	port = imxcsi_get_port(pdev->dev.parent->of_node, imxcsi->id);
	if (!port) {
		dev_err(&pdev->dev, "cannot find node port@%d\n", imxcsi->id);
		return -ENODEV;
	}

	ret = imxcsi_subdev_init(imxcsi, port);
	if (ret)
		goto putport;

	ret = imxcsi_async_init(imxcsi, port);
	if (ret)
		goto cleanup;

	of_node_put(port);

	return 0;

putport:
	of_node_put(port);
cleanup:
	media_entity_cleanup(&imxcsi->sd.entity);
	v4l2_async_unregister_subdev(&imxcsi->sd);
	return ret;
}

static int imxcsi_remove(struct platform_device *pdev)
{
	struct imxcsi_subdev *imxcsi = platform_get_drvdata(pdev);

	imx_media_entity_remove_link(imxcsi->link_up);
	media_entity_cleanup(&imxcsi->sd.entity);

	return 0;
}

static struct platform_driver imxcsi_pdrv = {
	.probe          = imxcsi_probe,
	.remove         = imxcsi_remove,
	.driver         = {
		.name   = DEVICE_NAME,
		.owner  = THIS_MODULE,
	},
};

module_platform_driver(imxcsi_pdrv);

MODULE_DESCRIPTION("i.MX5/6 CSI Subdev Driver");
MODULE_AUTHOR("Mentor Graphics Inc.");
MODULE_LICENSE("GPL");
