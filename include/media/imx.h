struct v4l2_subdev;
struct device_node;
struct imx_media_link;
struct v4l2_device;
struct media_device;
struct device;

struct imx_media_link *
imx_media_entity_create_link(struct v4l2_subdev *sd,
			     int local_padno, int remote_padno,
			     struct v4l2_async_subdev *asd,
			     u32 media_pad_flags, u32 media_link_flags);

void imx_media_entity_remove_link(struct imx_media_link *link);

struct v4l2_device *imx_media_get_v4l2_dev(void);

struct media_device *imx_find_media_device(void);

#ifdef CONFIG_MEDIA_IMX
int imx_media_device_register(struct device *dev);
#else
static inline int imx_media_device_register(struct device *dev)
{
	return 0;
}
#endif
