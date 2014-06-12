#ifndef __MEDIA_IMX_COMMON_H
#define __MEDIA_IMX_COMMON_H
#include <linux/videodev2.h>
#include <media/videobuf2-core.h>

struct imx_fmt {
	u32 fourcc;
	const char *name;
	int bytes_per_pixel;
};

struct imx_buffer {
	struct vb2_buffer vb; /* v4l buffer must be first */
	dma_addr_t        phys;
	struct list_head  list;
};

struct imx_dma_buf {
	void          *virt;
	dma_addr_t     phys;
	unsigned long  len;
};

#define IMX_EOF_TIMEOUT 1000 /* msec */

int imx_alloc_dma_buf(struct device *dev, struct imx_dma_buf *buf, int size);
void imx_free_dma_buf(struct device *dev, struct imx_dma_buf *buf);

void imx_video_buffer_done(struct v4l2_subdev *sd,
			   struct imx_buffer *frame,
			   enum vb2_buffer_state status);
struct imx_buffer *imx_video_buffer_next(struct v4l2_subdev *sd);

int imx_enum_fmt(struct file *file, void *fh,
		struct v4l2_fmtdesc *f);
int imx_enum_fmt_rgb(struct file *file, void *fh,
		struct v4l2_fmtdesc *f);
int imx_enum_fmt_yuv(struct file *file, void *fh,
		struct v4l2_fmtdesc *f);
struct imx_fmt *imx_find_fmt_rgb(unsigned int pixelformat);
struct imx_fmt *imx_find_fmt_yuv(unsigned int pixelformat);
int imx_try_fmt(struct file *file, void *fh,
		struct v4l2_format *f);
int imx_try_fmt_rgb(struct file *file, void *fh,
		struct v4l2_format *f);
int imx_try_fmt_yuv(struct file *file, void *fh,
		struct v4l2_format *f);
int imx_s_fmt(struct file *file, void *fh,
		struct v4l2_format *f, struct v4l2_pix_format *pix);
int imx_s_fmt_rgb(struct file *file, void *fh,
		struct v4l2_format *f, struct v4l2_pix_format *pix);
int imx_s_fmt_yuv(struct file *file, void *fh,
		struct v4l2_format *f, struct v4l2_pix_format *pix);
int imx_g_fmt(struct v4l2_format *f, struct v4l2_pix_format *pix);
int imx_enum_framesizes(struct file *file, void *fh,
			struct v4l2_frmsizeenum *fsize);

#endif /* __MEDIA_IMX_COMMON_H */
