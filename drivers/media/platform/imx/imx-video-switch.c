/*
 * devicetree probed mediacontrol video multiplexer.
 *
 * Copyright (C) 2013 Sascha Hauer, Pengutronix
 * Copyright (c) 2014 Mentor Graphics Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/of_graph.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-of.h>

#include <media/imx.h> /* for imx_media_entity_create_link */

struct vidsw {
	struct device *dev;
	struct v4l2_subdev subdev;
	struct media_pad *pads;
	struct v4l2_mbus_framefmt *format_mbus;
	struct v4l2_of_endpoint *endpoint;
	struct regmap_field *field;
	unsigned int gpio;
	int output_pad;
	int numpads;
	int streaming;
	int active;
};

#define to_vidsw(sd) container_of(sd, struct vidsw, subdev)

static int vidsw_link_setup(struct media_entity *entity,
		const struct media_pad *local,
		const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct vidsw *vidsw = to_vidsw(sd);

	dev_dbg(vidsw->dev, "link setup %s -> %s", remote->entity->name,
		local->entity->name);

	if (vidsw->streaming)
		return -EBUSY;

	if (!(flags & MEDIA_LNK_FL_ENABLED)) {
		if (local->index == vidsw->active) {
			dev_dbg(vidsw->dev, "going inactive\n");
			vidsw->active = -1;
		}
		return 0;
	}

	if (vidsw->active >= 0) {
		if (vidsw->active == local->index)
			return 0;
		else
			return -EBUSY;
	}

	vidsw->active = local->index;

	dev_dbg(vidsw->dev, "setting %d active\n", vidsw->active);

	if (vidsw->field)
		regmap_field_write(vidsw->field, vidsw->active);
	else if (gpio_is_valid(vidsw->gpio))
		gpio_set_value(vidsw->gpio, vidsw->active);

	return 0;
}

static struct media_entity_operations vidsw_ops = {
	.link_setup = vidsw_link_setup,
};

static int vidsw_async_init(struct vidsw *vidsw, struct device_node *node)
{
	struct v4l2_of_endpoint endpoint;
	struct device_node *epnode, *rp, *rpp;
	struct v4l2_async_subdev asd;
	uint32_t remote_portno;
	int pad, numpads;
	int ret;

	numpads = of_get_child_count(node);
	if (numpads < 2) {
		dev_err(vidsw->dev, "Not enough ports %d\n", numpads);
		return -EINVAL;
	}

	vidsw->numpads = numpads;

	/*
	 * the last endpoint must define the mux output pad,
	 * the rest are the mux input pads.
	 */
	vidsw->output_pad = numpads - 1;

	vidsw->pads = devm_kzalloc(vidsw->dev,
				   numpads * sizeof(*vidsw->pads),
				   GFP_KERNEL);
	if (!vidsw->pads)
		return -ENOMEM;

	vidsw->endpoint = devm_kzalloc(vidsw->dev,
				       numpads * sizeof(*vidsw->endpoint),
				       GFP_KERNEL);
	if (!vidsw->endpoint)
		return -ENOMEM;

	vidsw->format_mbus = devm_kzalloc(vidsw->dev,
					  numpads * sizeof(*vidsw->format_mbus),
					  GFP_KERNEL);
	if (!vidsw->format_mbus)
		return -ENOMEM;

	vidsw->subdev.entity.ops = &vidsw_ops;

	/* init the pad directions */
	for (pad = 0; pad < vidsw->output_pad; pad++)
		vidsw->pads[pad].flags = MEDIA_PAD_FL_SINK;
	vidsw->pads[vidsw->output_pad].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_init(&vidsw->subdev.entity,
				vidsw->numpads, vidsw->pads, 0);
	if (ret < 0)
		return ret;

	epnode = NULL;
	for (pad = 0; pad < vidsw->numpads; pad++) {
		epnode = of_graph_get_next_endpoint(node, epnode);
		if (!epnode)
			return -EINVAL;

		v4l2_of_parse_endpoint(epnode, &endpoint);
		vidsw->endpoint[pad] = endpoint;

		rp = of_graph_get_remote_port(epnode);
		rpp = of_graph_get_remote_port_parent(epnode);
		of_node_put(epnode);

		of_property_read_u32(rp, "reg", &remote_portno);
		of_node_put(rp);

		memset(&asd, 0, sizeof(asd));
		asd.match_type = V4L2_ASYNC_MATCH_OF;
		asd.match.of.node = rpp;

		imx_media_entity_create_link(&vidsw->subdev,
					     pad, remote_portno, &asd,
					     vidsw->pads[pad].flags, 0);
	}

	return 0;
}

static int vidsw_registered(struct v4l2_subdev *sd)
{
	return 0;
}

int vidsw_g_mbus_config(struct v4l2_subdev *sd, struct v4l2_mbus_config *cfg)
{
	struct vidsw *vidsw = container_of(sd, struct vidsw, subdev);

	dev_dbg(vidsw->dev, "reporting configration %d\n", vidsw->active);

	/* Mirror the input side on the output side */
	cfg->type = vidsw->endpoint[vidsw->active].bus_type;
	if (cfg->type == V4L2_MBUS_PARALLEL || cfg->type == V4L2_MBUS_BT656)
		cfg->flags = vidsw->endpoint[vidsw->active].bus.parallel.flags;

	return 0;
}

static const struct v4l2_subdev_video_ops vidsw_subdev_video_ops = {
	.g_mbus_config = vidsw_g_mbus_config,
};

static struct v4l2_mbus_framefmt *
__vidsw_get_pad_format(struct vidsw *vidsw, struct v4l2_subdev_fh *fh,
		       unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &vidsw->format_mbus[pad];
	default:
		return NULL;
	}
}

static int vidsw_get_format(struct v4l2_subdev *sd,
			    struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_format *sdformat)
{
	struct vidsw *vidsw = container_of(sd, struct vidsw, subdev);

	sdformat->format = *__vidsw_get_pad_format(vidsw, fh, sdformat->pad,
						   sdformat->which);
	return 0;
}

static int vidsw_set_format(struct v4l2_subdev *sd,
			    struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_format *sdformat)
{
	struct vidsw *vidsw = container_of(sd, struct vidsw, subdev);
	struct v4l2_mbus_framefmt *mbusformat;

	if (sdformat->pad >= vidsw->numpads)
		return -EINVAL;

	mbusformat = __vidsw_get_pad_format(vidsw, fh, sdformat->pad,
					    sdformat->which);
	if (!mbusformat)
		return -EINVAL;

	/* Output pad mirrors active input pad, no limitations on input pads */
	if (sdformat->pad == vidsw->output_pad && vidsw->active >= 0)
		*mbusformat = vidsw->format_mbus[vidsw->active];
	else
		*mbusformat = sdformat->format;

	sdformat->format = *mbusformat;

	return 0;
}

static struct v4l2_subdev_pad_ops vidsw_pad_ops = {
	.get_fmt = vidsw_get_format,
	.set_fmt = vidsw_set_format,
};

static struct v4l2_subdev_ops vidsw_subdev_ops = {
	.pad = &vidsw_pad_ops,
	.video = &vidsw_subdev_video_ops,
};

static struct v4l2_subdev_internal_ops vidsw_internal_ops = {
	.registered = vidsw_registered,
};

static int of_get_reg_field(struct device_node *node, struct reg_field *field)
{
	u32 bit_mask;
	int ret;

	ret = of_property_read_u32(node, "reg", &field->reg);
	if (ret < 0)
		return ret;

	ret = of_property_read_u32(node, "bit-mask", &bit_mask);
	if (ret < 0)
		return ret;

	ret = of_property_read_u32(node, "bit-shift", &field->lsb);
	if (ret < 0)
		return ret;

	field->msb = field->lsb + ffs(bit_mask) - 1;

	return 0;
}

static int vidsw_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct reg_field field;
	struct vidsw *vidsw;
	struct regmap *map;
	int ret;

	vidsw = devm_kzalloc(&pdev->dev, sizeof(*vidsw), GFP_KERNEL);
	if (!vidsw)
		return -ENOMEM;

	platform_set_drvdata(pdev, vidsw);

	v4l2_subdev_init(&vidsw->subdev, &vidsw_subdev_ops);
	vidsw->subdev.internal_ops = &vidsw_internal_ops;
	snprintf(vidsw->subdev.name, sizeof(vidsw->subdev.name), "%s",
			np->name);
	vidsw->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	vidsw->subdev.dev = &pdev->dev;
	vidsw->dev = &pdev->dev;
	vidsw->active = -1;

	ret = of_get_reg_field(np, &field);
	if (ret == 0) {
		map = syscon_node_to_regmap(np->parent);
		if (!map) {
			dev_err(&pdev->dev,
				"Failed to get syscon register map\n");
			return PTR_ERR(map);
		}

		vidsw->field = devm_regmap_field_alloc(&pdev->dev, map, field);
		if (IS_ERR(vidsw->field)) {
			dev_err(&pdev->dev,
				"Failed to allocate regmap field\n");
			return PTR_ERR(vidsw->field);
		}
	} else {
		vidsw->gpio = of_get_named_gpio_flags(np, "gpios", 0, NULL);
		ret = gpio_request_one(vidsw->gpio,
				       GPIOF_OUT_INIT_LOW, np->name);
		if (ret < 0) {
			dev_warn(&pdev->dev,
				 "could not request control gpio %d: %d\n",
				 vidsw->gpio, ret);
			vidsw->gpio = -1;
		}
	}

	ret = vidsw_async_init(vidsw, np);
	if (ret)
		return ret;

	ret = v4l2_async_register_subdev(&vidsw->subdev);
	if (ret)
		return ret;

	return 0;
}

static int vidsw_remove(struct platform_device *pdev)
{
	/* FIXME */

	return -EBUSY;
}

static const struct of_device_id vidsw_dt_ids[] = {
	{ .compatible = "imx-video-mux", },
	{ /* sentinel */ }
};

static struct platform_driver vidsw_driver = {
	.probe		= vidsw_probe,
	.remove		= vidsw_remove,
	.driver		= {
		.of_match_table = vidsw_dt_ids,
		.name	= "imx-video-mux",
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(vidsw_driver);

MODULE_DESCRIPTION("i.MX video stream multiplexer");
MODULE_AUTHOR("Sascha Hauer, Pengutronix");
MODULE_LICENSE("GPL");
