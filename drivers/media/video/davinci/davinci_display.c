/*
 * Copyright (C) 2007 Texas Instruments Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
/* davinci_display.c */

/*#define DEBUG */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/videodev.h>
#include <linux/videodev2.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/device.h>
#include <asm/irq.h>
#include <asm/page.h>
#include <asm/arch/cpu.h>
#include <media/davinci/davinci_enc.h>
#include <media/davinci/davinci_display.h>

#define DAVINCI_DISPLAY_DRIVER "DavinciDisplay"
#define DM355_EVM_CARD  "DM355 EVM"
#define DM644X_EVM_CARD "DM644X EVM"

static u32 video2_numbuffers = 3;
static u32 video3_numbuffers = 3;

#define DAVINCI_DISPLAY_HD_BUF_SIZE (1920*1088*2)
#define DAVINCI_DISPLAY_SD_BUF_SIZE (720*576*2)

#ifdef FB_DAVINCI_THS8200_ENCODER
static u32 video2_bufsize = DAVINCI_DISPLAY_HD_BUF_SIZE;
#else
static u32 video2_bufsize = DAVINCI_DISPLAY_SD_BUF_SIZE;
#endif
static u32 video3_bufsize = DAVINCI_DISPLAY_SD_BUF_SIZE;

module_param(video2_numbuffers, uint, S_IRUGO);
module_param(video3_numbuffers, uint, S_IRUGO);

module_param(video2_bufsize, uint, S_IRUGO);
module_param(video3_bufsize, uint, S_IRUGO);

#define DAVINCI_DEFAULT_NUM_BUFS 3
static struct buf_config_params display_buf_config_params = {
	.min_numbuffers = DAVINCI_DEFAULT_NUM_BUFS,
	.numbuffers[0] = DAVINCI_DEFAULT_NUM_BUFS,
	.numbuffers[1] = DAVINCI_DEFAULT_NUM_BUFS,
	.min_bufsize[0] = DAVINCI_DISPLAY_SD_BUF_SIZE,
	.min_bufsize[1] = DAVINCI_DISPLAY_SD_BUF_SIZE,
#ifdef CONFIG_DAVINCI_THS8200_ENCODER
	.layer_bufsize[0] = DAVINCI_DISPLAY_HD_BUF_SIZE,
	.layer_bufsize[1] = DAVINCI_DISPLAY_SD_BUF_SIZE,
#else
	.layer_bufsize[0] = DAVINCI_DISPLAY_SD_BUF_SIZE,
	.layer_bufsize[1] = DAVINCI_DISPLAY_SD_BUF_SIZE,
#endif
};

static int davinci_display_nr[] = { 2, 3 };

/* global variables */
static struct davinci_display davinci_dm;

struct device *davinci_display_dev = NULL;

static struct v4l2_capability davinci_display_videocap = {
	.driver = DAVINCI_DISPLAY_DRIVER,
	.bus_info = "Platform",
	.version = DAVINCI_DISPLAY_VERSION_CODE,
	.capabilities = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING
};

static struct v4l2_fract ntsc_aspect = DAVINCI_DISPLAY_PIXELASPECT_NTSC;
static struct v4l2_fract pal_aspect = DAVINCI_DISPLAY_PIXELASPECT_PAL;
static struct v4l2_fract sp_aspect = DAVINCI_DISPLAY_PIXELASPECT_SP;

static struct v4l2_rect ntsc_bounds = DAVINCI_DISPLAY_WIN_NTSC;
static struct v4l2_rect pal_bounds = DAVINCI_DISPLAY_WIN_PAL;
static struct v4l2_rect vga_bounds = DAVINCI_DISPLAY_WIN_640_480;
static struct v4l2_rect hd_720p_bounds = DAVINCI_DISPLAY_WIN_720P;
static struct v4l2_rect hd_1080i_bounds = DAVINCI_DISPLAY_WIN_1080I;

/*
 *=====davinci_alloc_buffer=====*/
/* Allocate memory for buffers */
static inline unsigned long davinci_alloc_buffer(unsigned int buf_size)
{
	void *mem = 0;
	u32 size = PAGE_SIZE << (get_order(buf_size));

	mem = (void *)__get_free_pages(GFP_KERNEL | GFP_DMA,
				       get_order(buf_size));
	if (mem) {
		unsigned long adr = (unsigned long)mem;
		while (size > 0) {
			SetPageReserved(virt_to_page(adr));
			adr += PAGE_SIZE;
			size -= PAGE_SIZE;
		}
	}
	return (unsigned long)mem;
}

/*
 *=====davinci_free_buffer=====*/
/* Free memory for buffers */
static inline void davinci_free_buffer(unsigned long addr,
				       unsigned int buf_size)
{
	unsigned int size, adr;

	if (!addr)
		return;
	adr = addr;
	size = PAGE_SIZE << (get_order(buf_size));
	while (size > 0) {
		ClearPageReserved(virt_to_page(adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	free_pages(addr, get_order(buf_size));
}

/*
 * ===== davinci_uservirt_to_phys =====
 *
 * This inline function is used to convert user space virtual address
 * to physical address.
 */
static inline u32 davinci_uservirt_to_phys(u32 virt)
{
	pgd_t *pgd;
	pmd_t *pmd;
	pte_t *pte;

	struct mm_struct *mm = current->mm;
	pgd = pgd_offset(mm, virt);
	if (!(pgd_none(*pgd) || pgd_bad(*pgd))) {
		pmd = pmd_offset(pgd, virt);

		if (!(pmd_none(*pmd) || pmd_bad(*pmd))) {
			pte = pte_offset_kernel(pmd, virt);

			if (pte_present(*pte)) {
				return __pa(page_address(pte_page(*pte))
					    + (virt & ~PAGE_MASK));
			}
		}
	}

	return 0;
}

/*
 * =====buffer_prepare=====*/
/* This is the callback function called from videobuf_qbuf() function
 * the buffer is prepared and user space virtual address is converted into
 * physical address */
static int davinci_buffer_prepare(struct videobuf_queue *q,
				  struct videobuf_buffer *vb,
				  enum v4l2_field field)
{
	/* Get the file handle object and layer object */
	struct davinci_fh *fh = q->priv_data;
	struct display_obj *layer = fh->layer;
	dev_dbg(davinci_display_dev, "<davinci_buffer_prepare>\n");

	/* If buffer is not initialized, initialize it */
	if (STATE_NEEDS_INIT == vb->state) {
		vb->width = davinci_dm.mode_info.xres;
		vb->height = davinci_dm.mode_info.yres;
		vb->size = vb->width * vb->height;
		vb->field = field;
	}
	vb->state = STATE_PREPARED;
	/* if user pointer memory mechanism is used, get the physical
	 * address of the buffer
	 */
	if (V4L2_MEMORY_USERPTR == layer->memory) {
		vb->boff = davinci_uservirt_to_phys(vb->baddr);
		if (!ISALIGNED(vb->boff)) {
			dev_err(davinci_display_dev, "buffer_prepare:offset is \
				not aligned to 8 bytes\n");
			return -EINVAL;
		}
	}
	dev_dbg(davinci_display_dev, "</davinci_buffer_prepare>\n");
	return 0;
}

/*
 * =====davinci_buffer_config=====*/
/* This function is responsible to responsible for buffer's
 * physical address */
static void davinci_buffer_config(struct videobuf_queue *q, unsigned int count)
{
	/* Get the file handle object and layer object */
	struct davinci_fh *fh = q->priv_data;
	struct display_obj *layer = fh->layer;
	int i;
	dev_dbg(davinci_display_dev, "<davinci_buffer_config>\n");

	/* If memory type is not mmap, return */
	if (V4L2_MEMORY_MMAP != layer->memory) {
		dev_err(davinci_display_dev, "Not MMAP\n");
		return;
	}
	/* Convert kernel space virtual address to physical address */
	for (i = 0; i < count; i++) {
		q->bufs[i]->boff = virt_to_phys((void *)layer->fbuffers[i]);
		dev_dbg(davinci_display_dev, "buffer address: %x\n",
			q->bufs[i]->boff);
	}
	dev_dbg(davinci_display_dev, "</davinci_buffer_config>\n");
}

/*
 * =====davinci_buffer_setup=====*/
/* This function allocates memory for the buffers */
static int davinci_buffer_setup(struct videobuf_queue *q, unsigned int *count,
				unsigned int *size)
{
	/* Get the file handle object and layer object */
	struct davinci_fh *fh = q->priv_data;
	struct display_obj *layer = fh->layer;
	int i;
	dev_dbg(davinci_display_dev, "<davinci_buffer_setup>\n");

	/* If memory type is not mmap, return */
	if (V4L2_MEMORY_MMAP != layer->memory) {
		dev_err(davinci_display_dev, "Not MMAP\n");
		return 0;
	}

	/* Calculate the size of the buffer */
	*size = display_buf_config_params.layer_bufsize[layer->device_id];

	for (i = display_buf_config_params.numbuffers[layer->device_id];
	     i < *count; i++) {
		/* Allocate memory for the buffers */
		layer->fbuffers[i] = davinci_alloc_buffer(*size);
		if (!layer->fbuffers[i])
			break;
	}
	/* Store number of buffers allocated in numbuffer member */
	*count = layer->numbuffers = i;
	dev_dbg(davinci_display_dev, "</davinci_buffer_setup>\n");
	return 0;
}

/*
 * =====davinci_buffer_queue=====*/
/* This function adds the buffer to DMA queue  */
static void davinci_buffer_queue(struct videobuf_queue *q,
				 struct videobuf_buffer *vb)
{
	/* Get the file handle object and layer object */
	struct davinci_fh *fh = q->priv_data;
	struct display_obj *layer = fh->layer;
	dev_dbg(davinci_display_dev, "<davinci_buffer_queue>\n");

	/* add the buffer to the DMA queue */
	list_add_tail(&vb->queue, &layer->dma_queue);
	/* Change state of the buffer */
	vb->state = STATE_QUEUED;
	dev_dbg(davinci_display_dev, "</davinci_buffer_queue>\n");
}

/*
 * =====davinci_buffer_release=====*/
/* This function is called from the videobuf layer to free memory allocated to
 * the buffers */
static void davinci_buffer_release(struct videobuf_queue *q,
				   struct videobuf_buffer *vb)
{
	/* Get the file handle object and layer object */
	struct davinci_fh *fh = q->priv_data;
	struct display_obj *layer = fh->layer;
	unsigned int buf_size = 0;
	dev_dbg(davinci_display_dev, "<davinci_buffer_release>\n");

	/* If memory type is not mmap, return */
	if (V4L2_MEMORY_MMAP != layer->memory) {
		dev_err(davinci_display_dev, "Not MMAP\n");
		return;
	}
	/* Calculate the size of the buffer */
	buf_size = display_buf_config_params.layer_bufsize[layer->device_id];

	if (((vb->i < layer->numbuffers)
	     && (vb->i >=
		 display_buf_config_params.numbuffers[layer->device_id]))
	    && layer->fbuffers[vb->i]) {
		davinci_free_buffer(layer->fbuffers[vb->i], buf_size);
		layer->fbuffers[vb->i] = 0;
	}
	vb->state = STATE_NEEDS_INIT;
	dev_dbg(davinci_display_dev, "</davinci_buffer_release>\n");
}

static struct videobuf_queue_ops video_qops = {
	.buf_setup = davinci_buffer_setup,
	.buf_prepare = davinci_buffer_prepare,
	.buf_queue = davinci_buffer_queue,
	.buf_release = davinci_buffer_release,
	.buf_config = davinci_buffer_config,
};

static u8 layer_first_int = 1;

/* It changes status of the displayed buffer, takes next buffer from the queue
 * and sets its address in VPBE registers */
static void davinci_display_isr(unsigned int event, void *dispObj)
{
	unsigned long jiffies_time = get_jiffies_64();
	struct timeval timevalue;
	int i, fid;
	unsigned long addr = 0;
	struct display_obj *layer = NULL;
	struct davinci_display *dispDevice = (struct davinci_display *)dispObj;

	/* Convert time represention from jiffies to timeval */
	jiffies_to_timeval(jiffies_time, &timevalue);

	for (i = 0; i < DAVINCI_DISPLAY_MAX_DEVICES; i++) {
		layer = dispDevice->dev[i];
		/* If streaming is started in this layer */
		if (!layer->started)
			continue;
		/* Check the field format */
		if ((V4L2_FIELD_NONE == layer->pix_fmt.field) &&
		    (!list_empty(&layer->dma_queue)) &&
		    (event & DAVINCI_DISP_END_OF_FRAME)) {
			/* Progressive mode */
			/* Progressive mode */
			if (layer_first_int) {
				layer_first_int = 0;
				continue;
			} else {
				/* Mark status of the curFrm to
				 * done and unlock semaphore on it */
				layer->curFrm->ts = timevalue;
				layer->curFrm->state = STATE_DONE;
				wake_up_interruptible(&layer->curFrm->done);
				/* Make curFrm pointing to nextFrm */
				layer->curFrm = layer->nextFrm;
			}
			/* Get the next buffer from buffer queue */
			layer->nextFrm =
			    list_entry(layer->dma_queue.next,
				       struct videobuf_buffer, queue);
			/* Remove that buffer from the buffer queue */
			list_del(&layer->nextFrm->queue);
			/* Mark status of the buffer as active */
			layer->nextFrm->state = STATE_ACTIVE;
			/* Set top and bottom field addresses in
			   VPIF registers */
			addr = layer->curFrm->boff;
			davinci_disp_start_layer(layer->layer_info.id, addr);
		} else {
			/* Interlaced mode */
			/* If it is first interrupt, ignore it */
			if (layer_first_int) {
				layer_first_int = 0;
				dev_dbg(davinci_display_dev,
					"irq_first time\n");
				return;
			}

			layer->field_id ^= 1;
			if (event & DAVINCI_DISP_FIRST_FIELD)
				fid = 0;
			else if (event & DAVINCI_DISP_SECOND_FIELD)
				fid = 1;
			else
				return;

			/* If field id does not match with stored
			   field id */
			if (fid != layer->field_id) {
				/* Make them in sync */
				if (0 == fid) {
					layer->field_id = fid;
					dev_dbg(davinci_display_dev,
						"field synced\n");
				}
				return;
			}
			/* device field id and local field id are
			   in sync */
			/* If this is even field */
			if (0 == fid) {
				if (layer->curFrm == layer->nextFrm)
					continue;
				/* one frame is displayed If next frame is
				 *  available, release curFrm and move on*/

				/* Copy frame display time */
				layer->curFrm->ts = timevalue;
				/* Change status of the curFrm */
				dev_dbg(davinci_display_dev,
					"Done with this video buffer\n");
				layer->curFrm->state = STATE_DONE;
				/* unlock semaphore on curFrm */
				wake_up_interruptible(&layer->curFrm->done);
				/* Make curFrm pointing to
				   nextFrm */
				layer->curFrm = layer->nextFrm;
			} else if (1 == fid) {	/* odd field */
				if (list_empty(&layer->dma_queue)
				    || (layer->curFrm != layer->nextFrm))
					continue;

				/* one field is displayed configure
				   the next frame if it is available
				   otherwise hold on current frame
				 */
				/* Get next from the buffer
				   queue */
				layer->nextFrm = list_entry(layer->
							    dma_queue.
							    next, struct
							    videobuf_buffer,
							    queue);

				/* Remove that from the
				   buffer queue */
				list_del(&layer->nextFrm->queue);

				/* Mark state of the frame
				   to active */
				layer->nextFrm->state = STATE_ACTIVE;
				addr = layer->nextFrm->boff;
				davinci_disp_start_layer(layer->layer_info.id,
							 addr);
			}
		}
	}
}

static int davinci_check_format(struct display_obj *layer,
				struct v4l2_pix_format *pixfmt)
{
	enum v4l2_field field = pixfmt->field;
	struct vid_enc_mode_info *mode_info;
	int temp_width = pixfmt->width, temp_height = pixfmt->height;
	dev_dbg(davinci_display_dev, "<davinci_check_format, >\n");

	if (layer->memory == V4L2_MEMORY_USERPTR) {
		/* We use bytesperline to calculate the width of the image */
		temp_width = pixfmt->bytesperline / 2;
		temp_height = pixfmt->sizeimage / pixfmt->bytesperline;
	}

	dev_dbg(davinci_display_dev,
		"<pixelformat = %x, height = %d, width = %d, field = %d >\n",
		(unsigned int)pixfmt->pixelformat, (unsigned int)temp_height,
		(unsigned int)temp_width, (unsigned int)pixfmt->field);

	if (!(DAVINCI_DISPLAY_VALID_FIELD(field))) {
		dev_err(davinci_display_dev,
			"invalid frame format , field = %d\n", (int)field);
		return -EINVAL;
	}

	if (pixfmt->pixelformat != V4L2_PIX_FMT_UYVY) {
		dev_err(davinci_display_dev, "invalid frame format\n");
		return -EINVAL;
	}
	if (temp_width && ((temp_width % 16) != 0)) {
		/* must be a mutliple of 32 */
		dev_err(davinci_display_dev,
			"width should be a multiple of 16\n");
		return -EINVAL;
	}

	if ((temp_height & 0x1) && (field == V4L2_FIELD_INTERLACED)) {
		dev_err(davinci_display_dev,
			"height should be even for interlaced mode\n");
		return -EINVAL;
	}

	/* get the current video display mode from encoder manager */
	mode_info = &davinci_dm.mode_info;
	if (davinci_enc_get_mode(0, mode_info)) {
		dev_err(davinci_display_dev,
			"Error in getting current display mode from enc mngr\n");
		return -1;
	}

	if ((0 == temp_width) || (temp_width > mode_info->xres) ||
	    (0 == temp_height) || (temp_height > mode_info->yres)) {
		dev_err(davinci_display_dev, "Invalid width or height\n");
		return -EINVAL;
	}

	if (field == V4L2_FIELD_ANY) {
		field = (mode_info->interlaced) ? V4L2_FIELD_INTERLACED :
		    V4L2_FIELD_NONE;
	}
	if ((!mode_info->interlaced && (field == V4L2_FIELD_INTERLACED)) ||
	    (mode_info->interlaced && (field == V4L2_FIELD_NONE))) {
		dev_err(davinci_display_dev, "Invalid field\n");
		return -EINVAL;
	}

	dev_dbg(davinci_display_dev, "</davinci_check_format>\n");
	return 0;
}

static int davinci_set_video_display_params(struct display_obj *layer)
{
	unsigned long addr;

	addr = layer->curFrm->boff;
	/* Set address in the display registers */
	davinci_disp_start_layer(layer->layer_info.id, addr);
	davinci_disp_enable_layer(layer->layer_info.id);
	/* Enable the window */
	layer->layer_info.enable = 1;
	return 0;
}
static void davinci_disp_calculate_scale_factor(struct display_obj *layer,
						int expected_xsize,
						int expected_ysize)
{
	struct display_layer_info *layer_info = &layer->layer_info;
	struct v4l2_pix_format *pixfmt = &layer->pix_fmt;
	int h_scale = 0, v_scale = 0, h_exp = 0, v_exp = 0, temp;
	/* Application initially set the image format. Current display
	   size is obtained from the encoder manager. expected_xsize
	   and expected_ysize are set through S_CROP ioctl. Based on this,
	   driver will calculate the scale factors for vertical and
	   horizontal direction so that the image is displayed scaled
	   and expanded. Application uses expansion to display the image
	   in a square pixel. Otherwise it is displayed using displays
	   pixel aspect ratio.It is expected that application chooses
	   the crop coordinates for cropped or scaled display. if crop
	   size is less than the image size, it is displayed cropped or
	   it is displayed scaled and/or expanded.
	 */

	/* to begin with, set the crop window same as expected. Later we
	   will override with scaled window size
	 */
	layer->layer_info.config.xsize = pixfmt->width;
	layer->layer_info.config.ysize = pixfmt->height;
	layer_info->h_zoom = ZOOM_X1;	/* no horizontal zoom */
	layer_info->v_zoom = ZOOM_X1;	/* no horizontal zoom */
	layer_info->h_exp = H_EXP_OFF;	/* no horizontal zoom */
	layer_info->v_exp = V_EXP_OFF;	/* no horizontal zoom */

	if (pixfmt->width < expected_xsize) {
		h_scale = davinci_dm.mode_info.xres / pixfmt->width;
		if (h_scale < 2)
			h_scale = 1;
		else if (h_scale >= 4)
			h_scale = 4;
		else
			h_scale = 2;
		layer->layer_info.config.xsize *= h_scale;
		if (layer->layer_info.config.xsize < expected_xsize) {
			if (!strcmp(davinci_dm.mode_info.name, VID_ENC_STD_NTSC)
			    || !strcmp(davinci_dm.mode_info.name,
				       VID_ENC_STD_PAL)) {
				temp =
				    (layer->layer_info.config.xsize *
				     DAVINCI_DISPLAY_H_EXP_RATIO_N)
				    / DAVINCI_DISPLAY_H_EXP_RATIO_D;
				if (temp <= expected_xsize) {
					h_exp = 1;
					layer->layer_info.config.xsize = temp;
				}
			}
		}
		if (h_scale == 2)
			layer_info->h_zoom = ZOOM_X2;
		else if (h_scale == 4)
			layer_info->h_zoom = ZOOM_X4;
		if (h_exp)
			layer_info->h_exp = H_EXP_9_OVER_8;
	} else {
		/* no scaling, only cropping. Set display area to crop area */
		layer->layer_info.config.xsize = expected_xsize;
	}

	if (pixfmt->height < expected_ysize) {
		v_scale = expected_ysize / pixfmt->height;
		if (v_scale < 2)
			v_scale = 1;
		else if (v_scale >= 4)
			v_scale = 4;
		else
			v_scale = 2;
		layer->layer_info.config.ysize *= v_scale;
		if (layer->layer_info.config.ysize < expected_ysize) {
			if (!strcmp(davinci_dm.mode_info.name, "PAL")) {
				temp =
				    (layer->layer_info.config.ysize *
				     DAVINCI_DISPLAY_V_EXP_RATIO_N)
				    / DAVINCI_DISPLAY_V_EXP_RATIO_D;
				if (temp <= expected_ysize) {
					v_exp = 1;
					layer->layer_info.config.ysize = temp;
				}
			}
		}
		if (v_scale == 2)
			layer_info->v_zoom = ZOOM_X2;
		else if (v_scale == 4)
			layer_info->v_zoom = ZOOM_X4;
		if (v_exp)
			layer_info->h_exp = V_EXP_6_OVER_5;
	} else {
		/* no scaling, only cropping. Set display area to crop area */
		layer->layer_info.config.ysize = expected_ysize;
	}
	dev_dbg(davinci_display_dev,
		"crop display xsize = %d, ysize = %d\n",
		layer->layer_info.config.xsize, layer->layer_info.config.ysize);
}

static void davinci_disp_adj_position(struct display_obj *layer, int top,
				      int left)
{
	layer->layer_info.config.xpos = 0;
	layer->layer_info.config.ypos = 0;
	if (left + layer->layer_info.config.xsize <= davinci_dm.mode_info.xres)
		layer->layer_info.config.xpos = left;
	if (top + layer->layer_info.config.ysize <= davinci_dm.mode_info.yres)
		layer->layer_info.config.ypos = top;
	dev_dbg(davinci_display_dev,
		"new xpos = %d, ypos = %d\n",
		layer->layer_info.config.xpos, layer->layer_info.config.ypos);
}

static int davinci_disp_check_window_params(struct v4l2_rect *c)
{
	if ((c->width == 0)
	    || ((c->width + c->left) > davinci_dm.mode_info.xres)
	    || (c->height == 0)
	    || ((c->height + c->top) > davinci_dm.mode_info.yres)) {
		dev_err(davinci_display_dev, "Invalid crop values\n");
		return -1;
	}
	if ((c->height & 0x1) && (davinci_dm.mode_info.interlaced)) {
		dev_err(davinci_display_dev,
			"window height must be even for interlaced display\n");
		return -1;
	}
	return 0;
}

/*
 * ======== davinci_doioctl ========*
 * This function will provide different V4L2 commands.This function can be
 * used to configure driver or get status of driver as per command passed
 * by application */
static int davinci_doioctl(struct inode *inode, struct file *file,
			   unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct davinci_fh *fh = file->private_data;
	struct display_obj *layer = fh->layer;
	unsigned int index = 0;
	unsigned long addr, flags;
	dev_dbg(davinci_display_dev, "<davinci_doioctl>\n");

	/* Check for the priority */
	switch (cmd) {
	case VIDIOC_S_FMT:
		ret = v4l2_prio_check(&layer->prio, &fh->prio);
		if (0 != ret)
			return ret;
		break;
	}

	/* Check for null value of parameter */
	if (ISNULL((void *)arg)) {
		dev_err(davinci_display_dev, "Null pointer\n");
		return -EINVAL;
	}
	/* Switch on the command value */
	switch (cmd) {
		/* If the case is for querying capabilities */
	case VIDIOC_QUERYCAP:
		{
			struct v4l2_capability *cap =
			    (struct v4l2_capability *)arg;
			dev_dbg(davinci_display_dev,
				"VIDIOC_QUERYCAP, layer id = %d\n",
				layer->device_id);
			memset(cap, 0, sizeof(*cap));
			*cap = davinci_display_videocap;
			break;
		}

	case VIDIOC_CROPCAP:
		{
			struct v4l2_cropcap *cropcap =
			    (struct v4l2_cropcap *)arg;
			dev_dbg(davinci_display_dev,
				"\nStart of VIDIOC_CROPCAP ioctl");
			if (davinci_enc_get_mode(0, &davinci_dm.mode_info)) {
				dev_err(davinci_display_dev,
					"Error in getting current display mode from enc mngr\n");
				up(&davinci_dm.lock);
				return -EINVAL;
			}
			down_interruptible(&davinci_dm.lock);
			cropcap->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
			if (!strcmp
			    (davinci_dm.mode_info.name, VID_ENC_STD_NTSC)) {
				cropcap->bounds = cropcap->defrect =
				    ntsc_bounds;
				cropcap->pixelaspect = ntsc_aspect;
			} else
			    if (!strcmp
				(davinci_dm.mode_info.name, VID_ENC_STD_PAL)) {
				cropcap->bounds = cropcap->defrect = pal_bounds;
				cropcap->pixelaspect = pal_aspect;
			} else
			    if (!strcmp
				(davinci_dm.mode_info.name,
				 VID_ENC_STD_640x480)) {
				cropcap->bounds = cropcap->defrect = vga_bounds;
				cropcap->pixelaspect = sp_aspect;
			} else
			    if (!strcmp
				(davinci_dm.mode_info.name,
				 VID_ENC_STD_640x400)) {
				cropcap->bounds = cropcap->defrect = vga_bounds;
				cropcap->bounds.height =
				    cropcap->defrect.height = 400;
				cropcap->pixelaspect = sp_aspect;
			} else
			    if (!strcmp
				(davinci_dm.mode_info.name,
				 VID_ENC_STD_640x350)) {
				cropcap->bounds = cropcap->defrect = vga_bounds;
				cropcap->bounds.height =
				    cropcap->defrect.height = 350;
				cropcap->pixelaspect = sp_aspect;
			} else
			    if (!strcmp
				(davinci_dm.mode_info.name,
				 VID_ENC_STD_720P_60)) {
				cropcap->bounds = cropcap->defrect =
				    hd_720p_bounds;
				cropcap->pixelaspect = sp_aspect;
			} else
			    if (!strcmp
				(davinci_dm.mode_info.name,
				 VID_ENC_STD_1080I_30)) {
				cropcap->bounds = cropcap->defrect =
				    hd_1080i_bounds;
				cropcap->pixelaspect = sp_aspect;
			} else {
				dev_err(davinci_display_dev,
					"Unknown encoder display mode\n");
				up(&davinci_dm.lock);
				return -EINVAL;
			}
			up(&davinci_dm.lock);
			dev_dbg(davinci_display_dev,
				"\nEnd of VIDIOC_CROPCAP ioctl");
			break;
		}

	case VIDIOC_G_CROP:
		{
			/* TBD to get the x,y and height/width params */
			struct v4l2_crop *crop = (struct v4l2_crop *)arg;
			dev_dbg(davinci_display_dev,
				"VIDIOC_G_CROP, layer id = %d\n",
				layer->device_id);

			if (crop->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
				struct v4l2_rect *rect = &crop->c;
				down_interruptible(&davinci_dm.lock);
				davinci_disp_get_layer_config(layer->layer_info.
							      id,
							      &layer->
							      layer_info.
							      config);
				rect->top = layer->layer_info.config.ypos;
				rect->left = layer->layer_info.config.xpos;
				rect->width = layer->layer_info.config.xsize;
				rect->height = layer->layer_info.config.ysize;
				up(&davinci_dm.lock);
			} else {
				dev_err(davinci_display_dev,
					"Invalid buf type \n");
				return -EINVAL;
			}
			break;
		}
	case VIDIOC_S_CROP:
		{
			/* TBD to get the x,y and height/width params */
			struct v4l2_crop *crop = (struct v4l2_crop *)arg;
			dev_dbg(davinci_display_dev,
				"VIDIOC_S_CROP, layer id = %d\n",
				layer->device_id);

			if (crop->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
				struct v4l2_rect *rect = &crop->c;

				if (davinci_disp_check_window_params(rect)) {
					dev_err(davinci_display_dev,
						"Error in S_CROP params\n");
					return -EINVAL;
				}
				down_interruptible(&davinci_dm.lock);
				davinci_disp_get_layer_config(layer->layer_info.
							      id,
							      &layer->
							      layer_info.
							      config);

				davinci_disp_calculate_scale_factor(layer,
								    rect->width,
								    rect->
								    height);

				davinci_disp_adj_position(layer, rect->top,
							  rect->left);

				if (davinci_disp_set_layer_config
				    (layer->layer_info.id,
				     &layer->layer_info.config)) {
					dev_err(davinci_display_dev,
						"Error in S_CROP params\n");
					up(&davinci_dm.lock);
					return -EINVAL;
				}
				/* apply zooming and h or v expansion */
				davinci_disp_set_zoom
				    (layer->layer_info.id,
				     layer->layer_info.h_zoom,
				     layer->layer_info.v_zoom);

				davinci_disp_set_vid_expansion
				    (layer->layer_info.h_exp,
				     layer->layer_info.v_exp);

				if ((layer->layer_info.h_zoom != ZOOM_X1) ||
				    (layer->layer_info.v_zoom != ZOOM_X1) ||
				    (layer->layer_info.h_exp != H_EXP_OFF) ||
				    (layer->layer_info.v_exp != V_EXP_OFF))
					/* Enable expansion filter */
					davinci_disp_set_interpolation_filter
					    (1);
				else
					davinci_disp_set_interpolation_filter
					    (0);
				up(&davinci_dm.lock);
			} else {
				dev_err(davinci_display_dev,
					"Invalid buf type \n");
				return -EINVAL;
			}
			break;
		}
		/* If the case is for enumerating formats */
	case VIDIOC_ENUM_FMT:
		{
			struct v4l2_fmtdesc *fmt = (struct v4l2_fmtdesc *)arg;
			dev_dbg(davinci_display_dev,
				"VIDIOC_ENUM_FMT, layer id = %d\n",
				layer->device_id);
			if (fmt->index > 0) {
				dev_err(davinci_display_dev,
					"Invalid format index\n");
				return -EINVAL;
			}
			/* Fill in the information about format */

			index = fmt->index;
			memset(fmt, 0, sizeof(*fmt));
			fmt->index = index;
			fmt->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
			if (index == 0) {
				strcpy(fmt->description, "YUV 4:2:2 - UYVY");
				fmt->pixelformat = V4L2_PIX_FMT_UYVY;
			}
			break;
		}

		/* If the case is for getting formats */
	case VIDIOC_G_FMT:
		{
			struct v4l2_format *fmt = (struct v4l2_format *)arg;
			dev_dbg(davinci_display_dev,
				"VIDIOC_G_FMT, layer id = %d\n",
				layer->device_id);

			/* If buffer type is video output */
			if (V4L2_BUF_TYPE_VIDEO_OUTPUT == fmt->type) {
				struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
				/* Fill in the information about
				 * format */
				down_interruptible(&davinci_dm.lock);
				*pixfmt = layer->pix_fmt;
				up(&davinci_dm.lock);
			} else {
				dev_err(davinci_display_dev, "invalid type\n");
				ret = -EINVAL;
			}
			break;
		}

		/* If the case is for setting formats */
	case VIDIOC_S_FMT:
		{
			struct v4l2_format *fmt = (struct v4l2_format *)arg;
			dev_dbg(davinci_display_dev,
				"VIDIOC_S_FMT, layer id = %d\n",
				layer->device_id);

			/* If streaming is started, return error */
			if (layer->started) {
				dev_err(davinci_display_dev,
					"Streaming is started\n");
				return -EBUSY;
			}
			if (V4L2_BUF_TYPE_VIDEO_OUTPUT == fmt->type) {
				struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
				/* Check for valid field format */
				ret = davinci_check_format(layer, pixfmt);

				if (ret)
					return ret;

				down_interruptible(&davinci_dm.lock);
				/* store the pixel format in the layer 
				 * object */
				davinci_disp_get_layer_config(layer->layer_info.
							      id,
							      &layer->
							      layer_info.
							      config);

				if (davinci_enc_get_mode
				    (0, &davinci_dm.mode_info)) {
					dev_err(davinci_display_dev,
						"couldn't get current display mode from enc mngr\n");
					up(&davinci_dm.lock);
					return -EINVAL;
				}

				/* For user ptr io, we calculate image size from bytes
				   perline and sizeimage as per DaVinciHD
				 */
				if (layer->memory == V4L2_MEMORY_USERPTR) {
					layer->layer_info.config.xsize =
					    pixfmt->bytesperline / 2;
					layer->layer_info.config.line_length =
					    pixfmt->bytesperline;
					layer->layer_info.config.ysize =
					    pixfmt->sizeimage /
					    pixfmt->bytesperline;
				} else {
					layer->layer_info.config.xsize =
					    pixfmt->width;
					layer->layer_info.config.ysize =
					    pixfmt->height;
					layer->layer_info.config.line_length =
					    pixfmt->width * 2;
				}
				layer->layer_info.config.ypos = 0;
				layer->layer_info.config.xpos = 0;
				layer->layer_info.config.interlaced =
				    davinci_dm.mode_info.interlaced;

				if (davinci_disp_set_layer_config
				    (layer->layer_info.id,
				     &layer->layer_info.config)) {
					dev_err(davinci_display_dev,
						"Error in S_FMT params:- field\n");
					up(&davinci_dm.lock);
					return -EINVAL;
				}

				/* readback and fill the local copy of current pix format */
				davinci_disp_get_layer_config(layer->layer_info.
							      id,
							      &layer->
							      layer_info.
							      config);

				layer->pix_fmt.width =
				    layer->layer_info.config.xsize;
				layer->pix_fmt.height =
				    layer->layer_info.config.ysize;
				layer->pix_fmt.bytesperline =
				    layer->layer_info.config.line_length;
				layer->pix_fmt.sizeimage =
				    layer->pix_fmt.bytesperline *
				    layer->pix_fmt.height;
				if (layer->layer_info.config.interlaced)
					layer->pix_fmt.field =
					    V4L2_FIELD_INTERLACED;
				else
					layer->pix_fmt.field = V4L2_FIELD_NONE;
				layer->pix_fmt.pixelformat = V4L2_PIX_FMT_UYVY;
				/* Set the scale factors */
				up(&davinci_dm.lock);
			} else {
				dev_err(davinci_display_dev, "invalid type\n");
				ret = -EINVAL;
			}
			break;
		}
		/* If the case is for trying formats */
	case VIDIOC_TRY_FMT:
		{
			struct v4l2_format *fmt;
			dev_dbg(davinci_display_dev, "VIDIOC_TRY_FMT\n");
			fmt = (struct v4l2_format *)arg;

			if (V4L2_BUF_TYPE_VIDEO_OUTPUT == fmt->type) {
				struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
				/* Check for valid field format */
				ret = davinci_check_format(layer, pixfmt);
				if (ret) {
					*pixfmt = layer->pix_fmt;
				}
			} else {
				dev_err(davinci_display_dev, "invalid type\n");
				ret = -EINVAL;
			}
			break;
		}

		/* If the case is for requesting buffer allocation */
	case VIDIOC_REQBUFS:
		{
			struct v4l2_requestbuffers *reqbuf;
			reqbuf = (struct v4l2_requestbuffers *)arg;
			dev_dbg(davinci_display_dev,
				"VIDIOC_REQBUFS, count= %d, type = %d, memory = %d\n",
				reqbuf->count, reqbuf->type, reqbuf->memory);

			down_interruptible(&davinci_dm.lock);
			/* If io users of the layer is not zero,
			   return error */
			if (0 != layer->io_usrs) {
				dev_err(davinci_display_dev, "not IO user\n");
				ret = -EBUSY;
				break;
			}
			/* Initialize videobuf queue as per the
			   buffer type */
			videobuf_queue_init(&layer->buffer_queue,
					    &video_qops, NULL,
					    &layer->irqlock,
					    V4L2_BUF_TYPE_VIDEO_OUTPUT,
					    layer->pix_fmt.field,
					    sizeof(struct videobuf_buffer), fh);
			/* Set buffer to Linear buffer */
			videobuf_set_buftype(&layer->buffer_queue,
					     VIDEOBUF_BUF_LINEAR);
			/* Set io allowed member of file handle to
			 * TRUE */
			fh->io_allowed = 1;
			/* Increment io usrs member of layer object
			   to 1 */
			layer->io_usrs = 1;
			/* Store type of memory requested in layer 
			   object */
			layer->memory = reqbuf->memory;
			/* Initialize buffer queue */
			INIT_LIST_HEAD(&layer->dma_queue);
			/* Allocate buffers */
			ret = videobuf_reqbufs(&layer->buffer_queue, reqbuf);
			up(&davinci_dm.lock);
			break;
		}
		/* If the case is for en-queing buffer in the buffer
		 * queue */
	case VIDIOC_QBUF:
		{
			struct v4l2_buffer tbuf;
			struct videobuf_buffer *buf1;
			dev_dbg(davinci_display_dev,
				"VIDIOC_QBUF, layer id = %d\n",
				layer->device_id);

			/* If this file handle is not allowed to do IO,
			   return error */
			if (!fh->io_allowed) {
				dev_err(davinci_display_dev, "No io_allowed\n");
				ret = -EACCES;
				break;
			}
			if (!(list_empty(&layer->dma_queue)) ||
			    (layer->curFrm != layer->nextFrm) ||
			    !(layer->started) ||
			    (layer->started && (0 == layer->field_id))) {

				ret = videobuf_qbuf(&layer->buffer_queue,
						    (struct v4l2_buffer *)arg);
				break;
			}
			/* bufferqueue is empty store buffer address
			 *  in VPBE registers */
			down(&layer->buffer_queue.lock);
			tbuf = *(struct v4l2_buffer *)arg;
			buf1 = layer->buffer_queue.bufs[tbuf.index];
			if (buf1->memory != tbuf.memory) {
				dev_err(davinci_display_dev,
					"invalid buffer type\n");
				up(&layer->buffer_queue.lock);
				return -EINVAL;
			}
			if ((buf1->state == STATE_QUEUED) ||
			    (buf1->state == STATE_ACTIVE)) {
				up(&layer->buffer_queue.lock);
				dev_err(davinci_display_dev, "invalid state\n");
				return -EINVAL;
			}

			switch (buf1->memory) {
			case V4L2_MEMORY_MMAP:
				if (buf1->baddr == 0) {
					up(&layer->buffer_queue.lock);
					dev_err(davinci_display_dev,
						"No Buffer address\n");
					return -EINVAL;
				}
				break;
			case V4L2_MEMORY_USERPTR:
				if (tbuf.length < buf1->bsize) {
					up(&layer->buffer_queue.lock);
					dev_err(davinci_display_dev,
						"No Buffer address\n");
					return -EINVAL;
				}
				if ((STATE_NEEDS_INIT != buf1->state)
				    && (buf1->baddr != tbuf.m.userptr))
					davinci_buffer_release(&layer->
							       buffer_queue,
							       buf1);
				buf1->baddr = tbuf.m.userptr;
				break;
			default:
				up(&layer->buffer_queue.lock);
				dev_err(davinci_display_dev,
					"Unknow Buffer type \n");
				return -EINVAL;
			}
			local_irq_save(flags);
			ret =
			    davinci_buffer_prepare(&layer->buffer_queue,
						   buf1,
						   layer->buffer_queue.field);
			buf1->state = STATE_ACTIVE;
			addr = buf1->boff;
			layer->nextFrm = buf1;

			davinci_disp_start_layer(layer->layer_info.id, addr);
			local_irq_restore(flags);
			list_add_tail(&buf1->stream,
				      &(layer->buffer_queue.stream));
			up(&layer->buffer_queue.lock);
			break;
		}

		/* If the case is for de-queing buffer from the
		 * buffer queue */
	case VIDIOC_DQBUF:
		{
			dev_dbg(davinci_display_dev,
				"VIDIOC_DQBUF, layer id = %d\n",
				layer->device_id);

			/* If this file handle is not allowed to do IO,
			   return error */
			if (!fh->io_allowed) {
				dev_err(davinci_display_dev, "No io_allowed\n");
				ret = -EACCES;
				break;
			}
			if (file->f_flags & O_NONBLOCK)
				/* Call videobuf_dqbuf for non
				   blocking mode */
				ret =
				    videobuf_dqbuf(&layer->buffer_queue,
						   (struct v4l2_buffer *)
						   arg, 1);
			else
				/* Call videobuf_dqbuf for
				   blocking mode */
				ret =
				    videobuf_dqbuf(&layer->buffer_queue,
						   (struct v4l2_buffer *)
						   arg, 0);
			break;
		}

		/* If the case is for querying information about
		 *  buffer for memory mapping io */
	case VIDIOC_QUERYBUF:
		{
			dev_dbg(davinci_display_dev,
				"VIDIOC_QUERYBUF, layer id = %d\n",
				layer->device_id);
			/* Call videobuf_querybuf to get information */
			ret = videobuf_querybuf(&layer->buffer_queue,
						(struct v4l2_buffer *)
						arg);
			break;
		}

		/* If the case is starting streaming */
	case VIDIOC_STREAMON:
		{
			dev_dbg(davinci_display_dev,
				"VIDIOC_STREAMON, layer id = %d\n",
				layer->device_id);
			/* If file handle is not allowed IO,
			 * return error */
			if (!fh->io_allowed) {
				dev_err(davinci_display_dev, "No io_allowed\n");
				ret = -EACCES;
				break;
			}
			/* If Streaming is already started,
			 * return error */
			if (layer->started) {
				dev_err(davinci_display_dev,
					"layer is already streaming\n");
				ret = -EBUSY;
				break;
			}

			/* Call videobuf_streamon to start streaming
			   in videobuf */
			ret = videobuf_streamon(&layer->buffer_queue);
			if (ret) {
				dev_err(davinci_display_dev,
					"error in videobuf_streamon\n");
				break;
			}
			down_interruptible(&davinci_dm.lock);
			/* If buffer queue is empty, return error */
			if (list_empty(&layer->dma_queue)) {
				dev_err(davinci_display_dev,
					"buffer queue is empty\n");
				ret = -EIO;
				up(&davinci_dm.lock);
				break;
			}
			/* Get the next frame from the buffer queue */
			layer->nextFrm = layer->curFrm =
			    list_entry(layer->dma_queue.next,
				       struct videobuf_buffer, queue);
			/* Remove buffer from the buffer queue */
			list_del(&layer->curFrm->queue);
			/* Mark state of the current frame to active */
			layer->curFrm->state = STATE_ACTIVE;
			/* Initialize field_id and started member */

			layer->field_id = 0;

			/* Set parameters in OSD and VENC */
			ret = davinci_set_video_display_params(layer);
			if (ret < 0) {
				up(&davinci_dm.lock);
				return ret;
			}
			layer->started = 1;
			dev_dbg(davinci_display_dev,
				"Started streaming on layer id = %d, ret = %d\n",
				layer->device_id, ret);
			layer_first_int = 1;
			up(&davinci_dm.lock);
			break;
		}

		/* If the case is for stopping streaming */
	case VIDIOC_STREAMOFF:
		{
			dev_dbg(davinci_display_dev,
				"VIDIOC_STREAMOFF,layer id = %d\n",
				layer->device_id);
			/* If io is allowed for this file handle,
			   return error */
			if (!fh->io_allowed) {
				dev_err(davinci_display_dev, "No io_allowed\n");
				ret = -EACCES;
				break;
			}
			/* If streaming is not started, return error */
			if (!layer->started) {
				dev_err(davinci_display_dev,
					"streaming not started in layer id = %d\n",
					layer->device_id);
				ret = -EINVAL;
				break;
			}
			down_interruptible(&davinci_dm.lock);
			davinci_disp_disable_layer(layer->layer_info.id);
			layer->started = 0;
			up(&davinci_dm.lock);
			ret = videobuf_streamoff(&layer->buffer_queue);
			break;
		}

	default:
		return -EINVAL;
	}

	dev_dbg(davinci_display_dev, "<davinci_doioctl>\n");
	return ret;
}

/*
 * ======== davinci_ioctl ========*/
/* Calls davinci_doioctl function */
static int davinci_ioctl(struct inode *inode, struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int ret;
	dev_dbg(davinci_display_dev, "Start of davinci ioctl\n");
	ret = video_usercopy(inode, file, cmd, arg, (void *)davinci_doioctl);
	if ((ret >= 0) && (VIDIOC_S_FMT == cmd || VIDIOC_TRY_FMT == cmd)) {
		ret = video_usercopy(inode, file, VIDIOC_G_FMT,
				     arg, (void *)davinci_doioctl);
	}
	dev_dbg(davinci_display_dev, "</davinci_ioctl>\n");
	return ret;
}

/*
 * ======== davinci_mmap ========*/
/* It is used to map kernel space buffers into user spaces */
static int davinci_mmap(struct file *filep, struct vm_area_struct *vma)
{
	/* Get the layer object and file handle object */
	struct davinci_fh *fh = filep->private_data;
	struct display_obj *layer = fh->layer;
	int err = 0;
	dev_dbg(davinci_display_dev, "<davinci_mmap>\n");

	err = videobuf_mmap_mapper(&layer->buffer_queue, vma);
	dev_dbg(davinci_display_dev, "</davinci_mmap>\n");
	return err;
}

static int davinci_config_layer(enum davinci_display_device_id id)
{
	int err = 0;
	struct davinci_layer_config *layer_config;
	struct vid_enc_mode_info *mode_info;
	struct display_obj *layer = davinci_dm.dev[id];

	/* First claim the layer for this device */
	if (davinci_disp_request_layer(layer->layer_info.id)) {
		/* Couldn't get layer */
		dev_err(davinci_display_dev,
			"Display Manager failed to allocate layer\n");
		return -EBUSY;
	}

	/* get the current video display mode from encoder manager */
	mode_info = &davinci_dm.mode_info;
	if (davinci_enc_get_mode(0, mode_info)) {
		dev_err(davinci_display_dev,
			"Error in getting current display mode from enc mngr\n");
		return -1;
	}

	layer_config = &layer->layer_info.config;
	/* Set the default image and crop values */
	layer_config->pixfmt = PIXFMT_YCbCrI;
	layer->pix_fmt.pixelformat = V4L2_PIX_FMT_UYVY;
	layer->pix_fmt.bytesperline = layer_config->line_length =
	    mode_info->xres * 2;

	layer->pix_fmt.width = layer_config->xsize = mode_info->xres;
	layer->pix_fmt.height = layer_config->ysize = mode_info->yres;
	layer->pix_fmt.sizeimage =
	    layer->pix_fmt.bytesperline * layer->pix_fmt.height;
	layer_config->xpos = 0;
	layer_config->ypos = 0;
	layer_config->interlaced = mode_info->interlaced;

	/* turn off ping-pong buffer and field inversion to fix
	   the image shaking problem in 1080I mode */
	if (id == DAVINCI_DISPLAY_DEVICE_0 &&
	    strcmp(mode_info->name, VID_ENC_STD_1080I_30) == 0 &&
	    cpu_is_davinci_dm644x())
		davinci_disp_set_field_inversion(0);

	if (layer->layer_info.config.interlaced)
		layer->pix_fmt.field = V4L2_FIELD_INTERLACED;
	else
		layer->pix_fmt.field = V4L2_FIELD_NONE;
	davinci_disp_set_layer_config(layer->layer_info.id, layer_config);
	return err;
}

/*
 *=====davinci_open===== */
/* It creates object of file handle structure and stores it in private_data
 * member of filepointer */
static int davinci_open(struct inode *inode, struct file *filep)
{
	int minor = iminor(inode);
	int found = -1;
	int i = 0;
	struct display_obj *layer;
	struct davinci_fh *fh = NULL;

	/* Check for valid minor number */
	for (i = 0; i < DAVINCI_DISPLAY_MAX_DEVICES; i++) {
		/* Get the pointer to the layer object */
		layer = davinci_dm.dev[i];
		if (minor == layer->video_dev->minor) {
			found = i;
			break;
		}
	}

	/* If not found, return error no device */
	if (0 > found) {
		dev_err(davinci_display_dev, "device not found\n");
		return -ENODEV;
	}

	/* Allocate memory for the file handle object */
	fh = kmalloc(sizeof(struct davinci_fh), GFP_KERNEL);
	if (ISNULL(fh)) {
		dev_err(davinci_display_dev,
			"unable to allocate memory for file handle object\n");
		return -ENOMEM;
	}
	dev_dbg(davinci_display_dev, "<davinci open> plane = %d\n",
		layer->device_id);
	/* store pointer to fh in private_data member of filep */
	filep->private_data = fh;
	fh->layer = layer;

	if (!layer->usrs) {
		/* Configure the default values for the layer */
		if (davinci_config_layer(layer->device_id)) {
			dev_err(davinci_display_dev,
				"Unable to configure video layer for id = %d\n",
				layer->device_id);
			return -EINVAL;
		}
	}

	/* Increment layer usrs counter */
	layer->usrs++;
	/* Set io_allowed member to false */
	fh->io_allowed = 0;
	/* Initialize priority of this instance to default priority */
	fh->prio = V4L2_PRIORITY_UNSET;
	v4l2_prio_open(&layer->prio, &fh->prio);
	dev_dbg(davinci_display_dev, "</davinci_open>\n");
	return 0;
}

/*
 *=====davinci_release=====*/
/* This function deletes buffer queue, frees the buffers and the davinci 
   display file * handle */
static int davinci_release(struct inode *inode, struct file *filep)
{
	/* Get the layer object and file handle object */
	struct davinci_fh *fh = filep->private_data;
	struct display_obj *layer = fh->layer;

	dev_dbg(davinci_display_dev, "<davinci_release>\n");
	/* If this is doing IO and other layer are not closed */
	if ((layer->usrs != 1) && fh->io_allowed) {
		dev_err(davinci_display_dev, "Close other instances\n");
		return -EAGAIN;
	}
	/* Get the lock on layer object */
	down_interruptible(&davinci_dm.lock);
	/* if this instance is doing IO */
	if (fh->io_allowed) {
		/* Reset io_usrs member of layer object */
		layer->io_usrs = 0;
		davinci_disp_disable_layer(layer->layer_info.id);
		layer->started = 0;
		/* Free buffers allocated */
		videobuf_queue_cancel(&layer->buffer_queue);
		videobuf_mmap_free(&layer->buffer_queue);
	}

	/* Decrement layer usrs counter */
	layer->usrs--;
	/* If this file handle has initialize encoder device, reset it */
	if (!layer->usrs) {
		davinci_disp_disable_layer(layer->layer_info.id);
		davinci_disp_release_layer(layer->layer_info.id);
	}

	/* Close the priority */
	v4l2_prio_close(&layer->prio, &fh->prio);
	filep->private_data = NULL;

	/* Free memory allocated to file handle object */
	if (!ISNULL(fh))
		kfree(fh);
	/* unlock semaphore on layer object */
	up(&davinci_dm.lock);
	dev_dbg(davinci_display_dev, "</davinci_release>\n");
	return 0;
}

static void davinci_platform_release(struct device
				     *device)
{
	/* This is called when the reference count goes to zero */
}

static struct file_operations davinci_fops = {
	.owner = THIS_MODULE,
	.open = davinci_open,
	.release = davinci_release,
	.ioctl = davinci_ioctl,
	.mmap = davinci_mmap
};
static struct video_device davinci_video_template = {
	.name = "davinci",
	.type = VID_TYPE_CAPTURE,
	.hardware = 0,
	.fops = &davinci_fops,
	.minor = -1
};

/*
 *=====davinci_probe=====*/
/* This function creates device entries by register itself to the V4L2 driver
 * and initializes fields of each layer objects */
static __init int davinci_probe(struct device *device)
{
	int i, j = 0, k, err = 0;
	struct video_device *vbd = NULL;
	struct display_obj *layer = NULL;
	struct platform_device *pdev;

	davinci_display_dev = device;

	dev_dbg(davinci_display_dev, "<davinci_probe>\n");

	/* First request memory region for io */
	pdev = to_platform_device(device);
	if (pdev->num_resources != 0) {
		dev_err(davinci_display_dev, "probed for an unknown device\n");
		return -ENODEV;
	}
	for (i = 0; i < DAVINCI_DISPLAY_MAX_DEVICES; i++) {
		/* Get the pointer to the layer object */
		layer = davinci_dm.dev[i];
		/* Allocate memory for video device */
		vbd = video_device_alloc();
		if (ISNULL(vbd)) {
			for (j = 0; j < i; j++) {
				video_device_release
				    (davinci_dm.dev[j]->video_dev);
			}
			dev_err(davinci_display_dev, "ran out of memory\n");
			return -ENOMEM;
		}

		/* Initialize field of video device */
		*vbd = davinci_video_template;
		vbd->dev = device;
		vbd->release = video_device_release;
		snprintf(vbd->name, sizeof(vbd->name),
			 "DaVinci_VPBEDisplay_DRIVER_V%d.%d.%d",
			 (DAVINCI_DISPLAY_VERSION_CODE >> 16)
			 & 0xff,
			 (DAVINCI_DISPLAY_VERSION_CODE >> 8) &
			 0xff, (DAVINCI_DISPLAY_VERSION_CODE) & 0xff);
		/* Set video_dev to the video device */
		layer->video_dev = vbd;
		layer->device_id = i;
		layer->layer_info.id =
		    ((i == DAVINCI_DISPLAY_DEVICE_0) ? WIN_VID0 : WIN_VID1);
		if (display_buf_config_params.numbuffers[i] == 0)
			layer->memory = V4L2_MEMORY_USERPTR;
		else
			layer->memory = V4L2_MEMORY_MMAP;
		/* Initialize field of the layer objects */
		layer->usrs = layer->io_usrs = 0;
		layer->started = 0;
#if 0
		if (j < 2)
			layer->numbuffers
			    =
			    display_buf_config_params.numbuffers[layer->
								 layer_id];
		else
			layer->numbuffers = 0;
#endif

		/* Initialize prio member of layer object */
		v4l2_prio_init(&layer->prio);

		/* register video device */
		printk(KERN_NOTICE
		       "Trying to register davinci display video device.\n");
		printk(KERN_NOTICE "layer=%x,layer->video_dev=%x\n", (int)layer,
		       (int)&layer->video_dev);

		err = video_register_device(layer->
					    video_dev,
					    VFL_TYPE_GRABBER,
					    davinci_display_nr[i]);
		if (err)
			goto probe_out;
	}
	/* Initialize mutex */
	init_MUTEX(&davinci_dm.lock);
	return 0;

      probe_out:
	for (k = 0; k < j; k++) {
		/* Get the pointer to the layer object */
		layer = davinci_dm.dev[k];
		/* Unregister video device */
		video_unregister_device(layer->video_dev);
		/* Release video device */
		video_device_release(layer->video_dev);
		layer->video_dev = NULL;
	}
	return err;
}

/*
 * ===== davinci_remove =====*/
/* It un-register hardware planes from V4L2 driver */
static int davinci_remove(struct device *device)
{
	int i;
	struct display_obj *plane;
	dev_dbg(davinci_display_dev, "<davinci_remove>\n");
	/* un-register device */
	for (i = 0; i < DAVINCI_DISPLAY_MAX_DEVICES; i++) {
		/* Get the pointer to the layer object */
		plane = davinci_dm.dev[i];
		/* Unregister video device */
		video_unregister_device(plane->video_dev);

		plane->video_dev = NULL;
	}

	dev_dbg(davinci_display_dev, "</davinci_remove>\n");
	return 0;
}

static struct device_driver davinci_driver = {
	.name = DAVINCI_DISPLAY_DRIVER,
	.bus = &platform_bus_type,
	.probe = davinci_probe,
	.remove = davinci_remove,
};
static struct platform_device _davinci_display_device = {
	.name = DAVINCI_DISPLAY_DRIVER,
	.id = 1,
	.dev = {
		.release = davinci_platform_release,
		}
};

/*
 *=====davinci_display_init=====*/
/* This function registers device and driver to the kernel, requests irq
 * handler and allocates memory for layer objects */
static __init int davinci_display_init(void)
{
	int err = 0, i, j;
	int free_layer_objects_index;
	int free_buffer_layer_index;
	int free_buffer_index;
	u32 addr;
	int size;

	printk(KERN_DEBUG "<davinci_display_init>\n");

	/* Default number of buffers should be 3 */
	if ((video2_numbuffers > 0) &&
	    (video2_numbuffers < display_buf_config_params.min_numbuffers))
		video2_numbuffers = display_buf_config_params.min_numbuffers;
	if ((video3_numbuffers > 0) &&
	    (video3_numbuffers < display_buf_config_params.min_numbuffers))
		video3_numbuffers = display_buf_config_params.min_numbuffers;

	/* Set buffer size to min buffers size if invalid buffer size is
	 * given */
	if (video2_bufsize <
	    display_buf_config_params.min_bufsize[DAVINCI_DISPLAY_DEVICE_0])
		video2_bufsize =
		    display_buf_config_params.
		    min_bufsize[DAVINCI_DISPLAY_DEVICE_0];

	if (video3_bufsize <
	    display_buf_config_params.min_bufsize[DAVINCI_DISPLAY_DEVICE_1])
		video3_bufsize =
		    display_buf_config_params.
		    min_bufsize[DAVINCI_DISPLAY_DEVICE_1];

	if (video2_numbuffers) {
		display_buf_config_params.numbuffers[DAVINCI_DISPLAY_DEVICE_0] =
		    video2_numbuffers;
	}
	if (video3_numbuffers) {
		display_buf_config_params.numbuffers[DAVINCI_DISPLAY_DEVICE_1] =
		    video3_numbuffers;
	}
	if (cpu_is_davinci_dm355()) {
		strcpy(davinci_display_videocap.card, DM355_EVM_CARD);
	} else {
		strcpy(davinci_display_videocap.card, DM644X_EVM_CARD);
	}
	/* Allocate memory for four plane display objects */
	for (i = 0; i < DAVINCI_DISPLAY_MAX_DEVICES; i++) {
		davinci_dm.dev[i] =
		    kmalloc(sizeof(struct display_obj), GFP_KERNEL);
		/* If memory allocation fails, return error */
		if (!davinci_dm.dev[i]) {
			free_layer_objects_index = i;
			printk(KERN_ERR "ran out of memory\n");
			err = -ENOMEM;
			goto davinci_init_free_layer_objects;
		}
		davinci_dm.dev[i]->irqlock = SPIN_LOCK_UNLOCKED;
	}
	free_layer_objects_index = DAVINCI_DISPLAY_MAX_DEVICES;

	/* Allocate memory for buffers */
	for (i = 0; i < DAVINCI_DISPLAY_MAX_DEVICES; i++) {
		size = display_buf_config_params.layer_bufsize[i];
		for (j = 0; j < display_buf_config_params.numbuffers[i]; j++) {
			addr = davinci_alloc_buffer(size);
			if (!addr) {
				free_buffer_layer_index = i;
				free_buffer_index = j;
				printk(KERN_ERR "ran out of memory\n");
				err = -ENOMEM;
				goto davinci_init_free_buffers;
			}
			davinci_dm.dev[i]->fbuffers[j] = addr;
		}
	}
	free_buffer_layer_index = DAVINCI_DISPLAY_MAX_DEVICES;
	free_buffer_index = display_buf_config_params.numbuffers[i - 1];
	/* Register driver to the kernel */
	err = driver_register(&davinci_driver);
	if (0 != err) {
		goto davinci_init_free_buffers;
	}
	/* register device as a platform device to the kernel */
	err = platform_device_register(&_davinci_display_device);
	if (0 != err) {
		goto davinci_init_unregister_driver;
	}

	davinci_dm.event_callback.mask = (DAVINCI_DISP_END_OF_FRAME |
					  DAVINCI_DISP_FIRST_FIELD |
					  DAVINCI_DISP_SECOND_FIELD);

	davinci_dm.event_callback.arg = &davinci_dm;
	davinci_dm.event_callback.handler = davinci_display_isr;

	err = davinci_disp_register_callback(&davinci_dm.event_callback);

	if (0 != err) {
		goto davinci_init_unregister_driver;
	}
	printk(KERN_NOTICE
	       "davinci_init:DaVinci V4L2 Display Driver V1.0 loaded\n");
	printk(KERN_DEBUG "</davinci_init>\n");
	return 0;

      davinci_init_unregister_driver:
	driver_unregister(&davinci_driver);

      davinci_init_free_buffers:
	for (i = 0; i < free_buffer_layer_index; i++) {
		for (j = 0; j < display_buf_config_params.numbuffers[i]; j++) {
			addr = davinci_dm.dev[i]->fbuffers[j];
			if (addr) {
				davinci_free_buffer(addr,
						    display_buf_config_params.
						    layer_bufsize[i]
				    );
				davinci_dm.dev[i]->fbuffers[j] = 0;
			}
		}
	}
	for (j = 0; j < display_buf_config_params.numbuffers[free_buffer_index];
	     j++) {
		addr = davinci_dm.dev[free_buffer_layer_index]->fbuffers[j];
		if (addr) {
			davinci_free_buffer(addr,
					    display_buf_config_params.
					    layer_bufsize[i]);
			davinci_dm.dev[free_buffer_layer_index]->fbuffers[j]
			    = 0;
		}

	}

      davinci_init_free_layer_objects:
	for (j = 0; j < free_layer_objects_index; j++) {
		if (davinci_dm.dev[i]) {
			kfree(davinci_dm.dev[j]);
			davinci_dm.dev[i] = NULL;
		}
	}
	return err;
}

/* =====davinci_cleanup=====
 * This function un-registers device and driver to the kernel, frees requested
 * irq handler and de-allocates memory allocated for layer objects.
 * */
static void davinci_cleanup(void)
{
	int i = 0, j = 0;
	u32 addr;
	printk(KERN_INFO "<davinci_cleanup>\n");

	davinci_disp_unregister_callback(&davinci_dm.event_callback);
	platform_device_unregister(&_davinci_display_device);
	driver_unregister(&davinci_driver);
	for (i = 0; i < DAVINCI_DISPLAY_MAX_DEVICES; i++) {
		for (j = 0; j < display_buf_config_params.numbuffers[i]; j++) {
			addr = davinci_dm.dev[i]->fbuffers[j];
			if (addr) {
				davinci_free_buffer(addr,
						    display_buf_config_params.
						    layer_bufsize[i]);
				davinci_dm.dev[i]->fbuffers[j] = 0;
			}
		}
	}
	for (i = 0; i < DAVINCI_DISPLAY_MAX_DEVICES; i++) {
		if (davinci_dm.dev[i]) {
			kfree(davinci_dm.dev[i]);
			davinci_dm.dev[i] = NULL;
		}
	}
	printk(KERN_INFO "</davinci_cleanup>\n");
}

EXPORT_SYMBOL(davinci_display_dev);
MODULE_LICENSE("GPL");
/* Function for module initialization and cleanup */
module_init(davinci_display_init);
module_exit(davinci_cleanup);
