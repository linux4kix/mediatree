#include <linux/module.h>
#include <linux/export.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <media/media-device.h>
#include <media/v4l2-device.h>

struct imx_media_controller {
	struct v4l2_device v4l2_dev;
	struct media_device mdev;
};

static struct imx_media_controller *imx_media;

struct media_device *imx_find_media_device(void)
{
	return &imx_media->mdev;
}
EXPORT_SYMBOL_GPL(imx_find_media_device);

struct imx_media_link {
	struct v4l2_async_notifier asn;
	struct v4l2_async_subdev   asd;
	struct v4l2_async_subdev   *asdp;
	struct v4l2_subdev	   *sd;
	int			   local_padno;
	int			   remote_padno;
	u32			   media_pad_flags;
	u32			   media_link_flags;
};

static int imx_media_bound(struct v4l2_async_notifier *notifier,
			   struct v4l2_subdev *sd,
			   struct v4l2_async_subdev *asd)
{
	struct imx_media_controller *im = imx_media;
	struct imx_media_link *link = container_of(notifier,
						   struct imx_media_link, asn);
	int ret;

	if ((sd->flags & V4L2_SUBDEV_FL_HAS_DEVNODE)) {
		ret = v4l2_device_register_subdev_node(&im->v4l2_dev, sd);
		if (ret)
			return ret;
	}

	if (link->media_pad_flags & MEDIA_PAD_FL_SINK)
		ret = media_entity_create_link(&sd->entity,
					       link->remote_padno,
					       &link->sd->entity,
					       link->local_padno,
					       link->media_link_flags);
	else
		ret = media_entity_create_link(&link->sd->entity,
					       link->local_padno,
					       &sd->entity,
					       link->remote_padno,
					       link->media_link_flags);
	if (ret)
		return ret;

	return 0;
}

static void imx_media_unbind(struct v4l2_async_notifier *notifier,
			     struct v4l2_subdev *sd,
			     struct v4l2_async_subdev *asd)
{
	if ((sd->flags & V4L2_SUBDEV_FL_HAS_DEVNODE)) {
		video_unregister_device(sd->devnode);
		kfree(sd->devnode);
	}
}

struct imx_media_link *
imx_media_entity_create_link(struct v4l2_subdev *sd,
			     int local_padno, int remote_padno,
			     struct v4l2_async_subdev *asd,
			     u32 media_pad_flags, u32 media_link_flags)
{
	struct imx_media_controller *im = imx_media;
	struct imx_media_link *link;
	int ret;

	if (!im)
		return ERR_PTR(-ENODEV);

	link = kzalloc(sizeof(*link), GFP_KERNEL);
	if (!link)
		return ERR_PTR(-ENOMEM);

	link->sd = sd;
	link->local_padno = local_padno;
	link->remote_padno = remote_padno;
	link->media_pad_flags = media_pad_flags;
	link->media_link_flags = media_link_flags;

	link->asd = *asd;
	link->asdp = &link->asd;

	link->asn.bound = imx_media_bound;
	link->asn.unbind = imx_media_unbind;
	link->asn.subdevs = &link->asdp;
	link->asn.num_subdevs = 1;
	link->asn.v4l2_dev = &im->v4l2_dev;

	ret = v4l2_async_notifier_register(&im->v4l2_dev, &link->asn);
	if (ret) {
		kfree(link);
		return ERR_PTR(ret);
	}

	return link;
}
EXPORT_SYMBOL_GPL(imx_media_entity_create_link);

void imx_media_entity_remove_link(struct imx_media_link *link)
{
	v4l2_async_notifier_unregister(&link->asn);

	kfree(link);
}
EXPORT_SYMBOL_GPL(imx_media_entity_remove_link);

struct v4l2_device *imx_media_get_v4l2_dev(void)
{
	if (!imx_media)
		return NULL;

	return &imx_media->v4l2_dev;
}
EXPORT_SYMBOL_GPL(imx_media_get_v4l2_dev);

int imx_media_device_register(struct device *dev)
{
	struct media_device *mdev;
	int ret;

	if (imx_media)
		return 0;

	imx_media = devm_kzalloc(dev, sizeof(*imx_media), GFP_KERNEL);
	if (!imx_media)
		return -ENOMEM;

	mdev = &imx_media->mdev;

	mdev->dev = dev;

	strlcpy(mdev->model, "i.MX IMXv3", sizeof(mdev->model));

	ret = media_device_register(mdev);
	if (ret)
		return ret;

	imx_media->v4l2_dev.mdev = mdev;

	ret = v4l2_device_register(mdev->dev, &imx_media->v4l2_dev);
	if (ret)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(imx_media_device_register);
