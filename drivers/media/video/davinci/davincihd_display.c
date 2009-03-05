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
/* davincihd_display.c */
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

#include <media/davinci/davincihd_display.h>
#include <media/davinci/vpif.h>
#include <asm/arch/video_hdevm.h>

#include <asm/arch/i2c-client.h>

#include <media/davinci/davinci_enc.h>

static u32 channel2_numbuffers = 3;
static u32 channel3_numbuffers = 3;
static u32 channel2_bufsize = 1920 * 1080 * 2;
static u32 channel3_bufsize = 720 * 576 * 2;

module_param(channel2_numbuffers, uint, S_IRUGO);
module_param(channel3_numbuffers, uint, S_IRUGO);
module_param(channel2_bufsize, uint, S_IRUGO);
module_param(channel3_bufsize, uint, S_IRUGO);

static struct vpif_config_params config_params = {
	.min_numbuffers = 3,
	.numbuffers[0] = 3,
	.numbuffers[1] = 3,
	.min_bufsize[0] = 720 * 480 * 2,
	.min_bufsize[1] = 720 * 480 * 2,
	.channel_bufsize[0] = 1920 * 1080 * 2,
	.channel_bufsize[1] = 720 * 576 * 2,
};

static int vpif_nr[] = { 2, 3, };

/* global variables */
static struct vpif_device vpif_obj = { {NULL} };
static struct workqueue_struct *vbi_workqueue;
static struct work_struct vbi_work[VPIF_DISPLAY_NUM_CHANNELS];

static struct device *vpif_dev = NULL;

static struct v4l2_capability vpif_videocap = {
	.driver = "vpif display",
	.card = "DaVinciHD EVM",
	.bus_info = "Platform",
	.version = VPIF_DISPLAY_VERSION_CODE,
	.capabilities = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING |
	    V4L2_CAP_VBI_OUTPUT | V4L2_CAP_HBI_OUTPUT |
	    V4L2_CAP_SLICED_VBI_OUTPUT
};

static struct v4l2_vbi_format vpif_raw_vbi_formats[2] = {
	{
	 .sampling_rate = 29.97,
	 .samples_per_line = VPIF_NTSC_VBI_SAMPLES_PER_LINE,
	 .sample_format = V4L2_PIX_FMT_GREY,
	 .start = {VPIF_NTSC_VBI_START_FIELD0,
		   VPIF_NTSC_VBI_START_FIELD1},
	 .count = {VPIF_NTSC_VBI_COUNT_FIELD0,
		   VPIF_NTSC_VBI_COUNT_FIELD1},
	 .flags = V4L2_VBI_INTERLACED},
	{
	 .sampling_rate = 25,
	 .samples_per_line = VPIF_PAL_VBI_SAMPLES_PER_LINE,
	 .sample_format = V4L2_PIX_FMT_GREY,
	 .start = {VPIF_PAL_VBI_START_FIELD0,
		   VPIF_PAL_VBI_START_FIELD1},
	 .count = {VPIF_PAL_VBI_COUNT_FIELD0,
		   VPIF_PAL_VBI_COUNT_FIELD1},
	 .flags = V4L2_VBI_INTERLACED}
};
static struct v4l2_vbi_format vpif_raw_hbi_formats[2] = {
	{
	 .sampling_rate = 29.97,
	 .samples_per_line = VPIF_NTSC_HBI_SAMPLES_PER_LINE,
	 .sample_format = V4L2_PIX_FMT_GREY,
	 .start = {VPIF_NTSC_HBI_START_FIELD0,
		   VPIF_NTSC_HBI_START_FIELD1},
	 .count = {VPIF_NTSC_HBI_COUNT_FIELD0,
		   VPIF_NTSC_HBI_COUNT_FIELD1},
	 .flags = V4L2_VBI_INTERLACED},
	{
	 .sampling_rate = 25,
	 .samples_per_line = VPIF_PAL_HBI_SAMPLES_PER_LINE,
	 .sample_format = V4L2_PIX_FMT_GREY,
	 .start = {VPIF_PAL_HBI_START_FIELD0,
		   VPIF_PAL_HBI_START_FIELD1},
	 .count = {VPIF_PAL_HBI_COUNT_FIELD0,
		   VPIF_PAL_HBI_COUNT_FIELD1},
	 .flags = V4L2_VBI_INTERLACED}
};

static struct vpif_service_line vbi_service_lines[] = {
	{V4L2_SLICED_CGMS_525, {20, 283}, VID_ENC_SLICED_VBI_CGMS_NTSC, 3},
	{V4L2_SLICED_CAPTION_525, {21, 284}, VID_ENC_SLICED_VBI_CC_NTSC,
	 2},
	{V4L2_SLICED_WSS_625, {23, 0}, VID_ENC_SLICED_VBI_WSS_PAL, 2},
};

static void vbi_work_handler(unsigned long data1)
{
	int i, j;
	struct channel_obj *channel = (struct channel_obj *)data1;
	struct common_obj *common = &(channel->common[VPIF_VBI_INDEX]);
	struct v4l2_sliced_vbi_data *addr = (struct v4l2_sliced_vbi_data *)
	    common->curFrm->boff;
	struct vid_enc_sliced_vbi_data data[2];
	addr = (struct v4l2_sliced_vbi_data *)
	    phys_to_virt((unsigned long)addr);
	down(&common->lock);
	if (channel->field_id == 1)
		addr += channel->vbi.num_services;
	for (i = 0; i < channel->vbi.num_services; i++) {
		data[i].service_id = 0;
		data[i].field = 0;
		for (j = 0; j < 3; j++) {
			if ((addr + i)->id & vbi_service_lines[j].service_id) {
				data[i].service_id =
				    vbi_service_lines[j].enc_service_id;
				data[i].field = addr->field;
				memcpy(data[i].data, (addr + i)->data,
				       vbi_service_lines[j].bytestowrite);
			}
		}
	}
	davinci_enc_write_sliced_vbi_data(channel->channel_id, data);
	up(&common->lock);
}

/*
 * vpif_alloc_buffer: Allocate memory for buffers */
static inline unsigned long vpif_alloc_buffer(unsigned int buf_size)
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
 *vpif_free_buffer : Free memory for buffers */
static inline void vpif_free_buffer(unsigned long addr, unsigned int buf_size)
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
 * vpif_uservirt_to_phys: This inline function is used to convert user
 * space virtual address to physical address.
 */
static inline u32 vpif_uservirt_to_phys(u32 virtp)
{
	unsigned long physp = 0;
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma;

	/* For kernel direct-mapped memory, take the easy way */
	if (virtp >= PAGE_OFFSET) {
		physp = virt_to_phys((void *)virtp);
	} else if ((vma = find_vma(mm, virtp)) && (vma->vm_flags & VM_IO)
		   && (vma->vm_pgoff)) {
		/* this will catch, kernel-allocated,
		   mmaped-to-usermode addresses */
		physp = (vma->vm_pgoff << PAGE_SHIFT) + (virtp - vma->vm_start);
	} else {
		/* otherwise, use get_user_pages() for general userland pages */
		int res, nr_pages = 1;
		struct page *pages;
		down_read(&current->mm->mmap_sem);

		res = get_user_pages(current, current->mm,
				     virtp, nr_pages, 1, 0, &pages, NULL);
		up_read(&current->mm->mmap_sem);

		if (res == nr_pages) {
			physp = __pa(page_address(&pages[0]) +
				     (virtp & ~PAGE_MASK));
		} else {
			dev_err(vpif_dev, "get_user_pages failed\n");
			return 0;
		}
	}

	return physp;
}

/*
 * buffer_prepare: This is the callback function called from videobuf_qbuf()
 * function the buffer is prepared and user space virtual address is converted
* into physical address */
static int vpif_buffer_prepare(struct videobuf_queue *q,
			       struct videobuf_buffer *vb,
			       enum v4l2_field field)
{
	/* Get the file handle object and channel object */
	struct vpif_fh *fh = q->priv_data;
	struct channel_obj *channel = fh->channel;
	unsigned long addr;
	int i, j, prevline = 0, num_services;
	struct v4l2_sliced_vbi_data *vbidata;
	struct common_obj *common = NULL;
	common = (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) ?
	    &(channel->common[VPIF_VIDEO_INDEX]) :
	    ((q->type == V4L2_BUF_TYPE_HBI_OUTPUT) ?
	     &(channel->common[VPIF_HBI_INDEX]) :
	     &(channel->common[VPIF_VBI_INDEX]));

	dev_dbg(vpif_dev, "<vpif_buffer_prepare>\n");

	/* If buffer is not initialized, initialize it */
	if (STATE_NEEDS_INIT == vb->state) {
		vb->width = common->width;
		vb->height = common->height;
		vb->size = vb->width * vb->height;
		vb->field = field;
	}
	vb->state = STATE_PREPARED;
	/* if user pointer memory mechanism is used, get the physical
	 * address of the buffer
	 */
	if (V4L2_MEMORY_USERPTR == common->memory) {
		if (0 == vb->baddr) {
			dev_err(vpif_dev, "buffer_address is 0\n");
			return -EINVAL;
		}
		vb->boff = vpif_uservirt_to_phys(vb->baddr);
		if (!ISALIGNED(vb->boff)) {
			dev_err(vpif_dev, "buffer_prepare:offset is \
					not aligned to 8 bytes\n");
			return -EINVAL;
		}
	}
	addr = vb->boff;
	if (q->streaming && (V4L2_BUF_TYPE_SLICED_VBI_OUTPUT != q->type)) {
		if (!ISALIGNED((addr + common->ytop_off)) ||
		    !ISALIGNED((addr + common->ybtm_off)) ||
		    !ISALIGNED((addr + common->ctop_off)) ||
		    !ISALIGNED((addr + common->cbtm_off))) {
			dev_err(vpif_dev, "buffer_prepare:offset is \
					not aligned to 8 bytes\n");
			return -EINVAL;
		}
	}
	if (V4L2_BUF_TYPE_SLICED_VBI_OUTPUT == q->type) {
		num_services = channel->vbi.num_services;
		vbidata = (struct v4l2_sliced_vbi_data *)phys_to_virt(vb->boff);
		prevline = 0;
		for (i = 0; i < num_services * 2; i++) {
			if (0 == vbidata->id)
				continue;
			if (!(vbidata->id & common->fmt.fmt.sliced.service_set)) {
				dev_err(vpif_dev, "Invalid service\n");
				return -EINVAL;
			}
			if (vbidata->field != 0 && vbidata->field != 1) {
				dev_err(vpif_dev, "Invalid field\n");
				return -EINVAL;
			}
			for (j = 0; j < VPIF_SLICED_MAX_SERVICES; j++) {
				if (!(vbidata->id &
				      vbi_service_lines[j].service_id))
					continue;
				if ((vbidata->field == 0)
				    && (vbidata->line !=
					vbi_service_lines[j].service_line[0])) {
					dev_err(vpif_dev,
						"Invalid field0line\n");
					return -EINVAL;
				}
				if ((vbidata->field == 1)
				    && (vbidata->line !=
					vbi_service_lines[j].service_line[1])) {
					dev_err(vpif_dev,
						"Invalid field0line\n");
					return -EINVAL;
				}
			}
			if (prevline > vbidata->line) {
				dev_err(vpif_dev,
					"Lines are not in increasing order\n");
				return -EINVAL;
			}
			prevline = vbidata->line;
			vbidata++;
		}
	}
	dev_dbg(vpif_dev, "</vpif_buffer_prepare>\n");
	return 0;
}

/*
 * vpif_buffer_config: This function is responsible to responsible for
 * buffer's physical address */
static void vpif_buffer_config(struct videobuf_queue *q, unsigned int count)
{
	/* Get the file handle object and channel object */
	struct vpif_fh *fh = q->priv_data;
	struct channel_obj *channel = fh->channel;
	struct common_obj *common = NULL;
	int i;
	dev_dbg(vpif_dev, "<vpif_buffer_config>\n");
	common = (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) ?
	    &(channel->common[VPIF_VIDEO_INDEX]) :
	    ((q->type == V4L2_BUF_TYPE_HBI_OUTPUT) ?
	     &(channel->common[VPIF_HBI_INDEX]) :
	     &(channel->common[VPIF_VBI_INDEX]));

	/* If memory type is not mmap, return */
	if (V4L2_MEMORY_MMAP != common->memory) {
		dev_dbg(vpif_dev, "End of buffer config\n");
		return;
	}
	/* Convert kernel space virtual address to physical address */
	for (i = 0; i < count; i++) {
		q->bufs[i]->boff = virt_to_phys((void *)common->fbuffers[i]);
		dev_dbg(vpif_dev, "buffer address: %x\n", q->bufs[i]->boff);
	}
	dev_dbg(vpif_dev, "</vpif_buffer_config>\n");
}

/*
 * vpif_buffer_setup: This function allocates memory for the buffers */
static int vpif_buffer_setup(struct videobuf_queue *q, unsigned int *count,
			     unsigned int *size)
{
	/* Get the file handle object and channel object */
	struct vpif_fh *fh = q->priv_data;
	struct channel_obj *channel = fh->channel;
	struct common_obj *common = NULL;
	int i;
	int startindex = 0;
	dev_dbg(vpif_dev, "<vpif_buffer_setup>\n");
	common = (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) ?
	    &(channel->common[VPIF_VIDEO_INDEX]) :
	    ((q->type == V4L2_BUF_TYPE_HBI_OUTPUT) ?
	     &(channel->common[VPIF_HBI_INDEX]) :
	     &(channel->common[VPIF_VBI_INDEX]));

	/* If memory type is not mmap, return */
	if (V4L2_MEMORY_MMAP != common->memory) {
		dev_dbg(vpif_dev, "End of buffer setup\n");
		return 0;
	}

	if (V4L2_BUF_TYPE_VIDEO_OUTPUT == q->type) {
		/* Calculate the size of the buffer */
		*size = config_params.channel_bufsize[channel->channel_id];
		startindex = config_params.numbuffers[channel->channel_id];
	} else if (V4L2_BUF_TYPE_SLICED_VBI_OUTPUT == q->type) {
		/* Get the size of the buffer */
		*size = VPIF_SLICED_BUF_SIZE;
	} else {
		/* Calculate the size of the buffer */
		*size = ((common->fmt.fmt.vbi.count[0]) +
			 (common->fmt.fmt.vbi.count[1])) *
		    (common->fmt.fmt.vbi.samples_per_line);
	}

	for (i = startindex; i < *count; i++) {
		/* Allocate memory for the buffers */
		common->fbuffers[i] = (u8 *) vpif_alloc_buffer(*size);
		if (!common->fbuffers[i])
			break;
	}
	/* Store number of buffers allocated in numbuffer member */
	*count = common->numbuffers = i;
	dev_dbg(vpif_dev, "</vpif_buffer_setup>\n");
	return 0;
}

/*
 * vpif_buffer_queue: This function adds the buffer to DMA queue  */
static void vpif_buffer_queue(struct videobuf_queue *q,
			      struct videobuf_buffer *vb)
{
	/* Get the file handle object and channel object */
	struct vpif_fh *fh = q->priv_data;
	struct channel_obj *channel = fh->channel;
	struct common_obj *common = NULL;
	dev_dbg(vpif_dev, "<vpif_buffer_queue>\n");
	common = (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) ?
	    &(channel->common[VPIF_VIDEO_INDEX]) :
	    ((q->type == V4L2_BUF_TYPE_HBI_OUTPUT) ?
	     &(channel->common[VPIF_HBI_INDEX]) :
	     &(channel->common[VPIF_VBI_INDEX]));

	/* add the buffer to the DMA queue */
	list_add_tail(&vb->queue, &common->dma_queue);
	/* Change state of the buffer */
	vb->state = STATE_QUEUED;
	dev_dbg(vpif_dev, "</vpif_buffer_queue>\n");
}

/*
 * vpif_buffer_release: This function is called from the videobuf layer to
 * free memory allocated to the buffers */
static void vpif_buffer_release(struct videobuf_queue *q,
				struct videobuf_buffer *vb)
{
	/* Get the file handle object and channel object */
	struct vpif_fh *fh = q->priv_data;
	struct channel_obj *channel = fh->channel;
	struct common_obj *common = NULL;
	unsigned int buf_size = 0;
	int endindex = 0;
	dev_dbg(vpif_dev, "<vpif_buffer_release>\n");
	common = (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) ?
	    &(channel->common[VPIF_VIDEO_INDEX]) :
	    ((q->type == V4L2_BUF_TYPE_HBI_OUTPUT) ?
	     &(channel->common[VPIF_HBI_INDEX]) :
	     &(channel->common[VPIF_VBI_INDEX]));

	vb->state = STATE_NEEDS_INIT;
	/* If memory type is not mmap, return */
	if (V4L2_MEMORY_MMAP != common->memory) {
		dev_dbg(vpif_dev, "End of buffer release\n");
		return;
	}
	if (V4L2_BUF_TYPE_VIDEO_OUTPUT == q->type) {
		/* Calculate the size of the buffer */
		buf_size = config_params.channel_bufsize[channel->channel_id];
		endindex = config_params.numbuffers[channel->channel_id];

	} else if (V4L2_BUF_TYPE_SLICED_VBI_OUTPUT == q->type) {
		/* Get the size of the buffer */
		buf_size = VPIF_SLICED_BUF_SIZE;
	} else {
		/* Calculate the size of the buffer */
		buf_size = ((common->fmt.fmt.vbi.count[0]) +
			    (common->fmt.fmt.vbi.count[1])) *
		    (common->fmt.fmt.vbi.samples_per_line);
	}

	dev_dbg(vpif_dev, "</vpif_buffer_release>\n");
}

static struct videobuf_queue_ops video_qops = {
	.buf_setup = vpif_buffer_setup,
	.buf_prepare = vpif_buffer_prepare,
	.buf_queue = vpif_buffer_queue,
	.buf_release = vpif_buffer_release,
	.buf_config = vpif_buffer_config,
};

static u8 channel_first_int[VPIF_NUMOBJECTS][2] = { {1, 1}, {1, 1}, {1, 1} };

/* vpif_channel_isr: It changes status of the displayed buffer, takes next
 * buffer from the queue and sets its address in VPIF registers */
static irqreturn_t vpif_channel_isr(int irq, void *dev_id, struct pt_regs *regs)
{
	struct timeval timevalue;
	int fid = -1, i;
	struct vpif_device *dev = &vpif_obj;
	int channel_id = 0;
	unsigned long addr = 0;
	struct channel_obj *channel = NULL;
	struct common_obj *common = NULL;
	struct video_obj *vid_ch = NULL;
	enum v4l2_field field;
	dev_dbg(vpif_dev, "<vpif_channel_isr>\n");

	channel_id = *(int *)(dev_id);
	channel = dev->dev[channel_id];
	vid_ch = &(channel->video);

	do_gettimeofday(&timevalue);
	field = channel->common[VPIF_VIDEO_INDEX].fmt.fmt.pix.field;
	for (i = 0; i < VPIF_NUMOBJECTS; i++) {
		common = &(channel->common[i]);
		/* If streaming is started in this channel */
		if (0 == common->started)
			continue;

		/* Check the field format */
		if (1 == vid_ch->std_info.frame_format) {
			if (list_empty(&common->dma_queue))
				continue;
			/* Progressive mode */
			if (!channel_first_int[i][channel_id]) {
				/* Mark status of the curFrm to
				 * done and unlock semaphore on it */
				common->curFrm->ts = timevalue;
				common->curFrm->state = STATE_DONE;
				wake_up_interruptible(&common->curFrm->done);
				/* Make curFrm pointing to nextFrm */
				common->curFrm = common->nextFrm;
			}
			channel_first_int[i][channel_id] = 0;

			/* Get the next buffer from buffer queue */
			common->nextFrm =
			    list_entry(common->dma_queue.next,
				       struct videobuf_buffer, queue);
			/* Remove that buffer from the buffer queue */
			list_del(&common->nextFrm->queue);
			/* Mark status of the buffer as active */
			common->nextFrm->state = STATE_ACTIVE;
			/* Set top and bottom field addresses in
			   VPIF registers */

			addr = common->nextFrm->boff;

			if (V4L2_BUF_TYPE_SLICED_VBI_OUTPUT == common->fmt.type) {
				queue_work(vbi_workqueue,
					   &vbi_work[channel->channel_id]);
			} else {
				common->set_addr(addr + common->ytop_off,
						 addr + common->ybtm_off,
						 addr + common->ctop_off,
						 addr + common->cbtm_off);
			}
		} else {
			/* Interlaced mode */
			/* If it is first interrupt, ignore it */
			if (channel_first_int[i][channel_id]) {
				channel_first_int[i][channel_id] = 0;
				continue;
			}
			if (0 == i) {
				channel->field_id ^= 1;
				/* Get field id from VPIF registers */
				fid =
				    vpif_channel_getfid(channel->
							channel_id + 2);
				/* If field id does not match with stored
				   field id */
				if (fid != channel->field_id) {
					/* Make them in sync */
					if (0 == fid) {
						channel->field_id = fid;
					}
					return IRQ_HANDLED;
				}
			}
			/* device field id and local field id are
			   in sync */
			/* If this is even field */
			if (0 == fid) {
				if (common->curFrm == common->nextFrm) {
					continue;
				}
				/* one frame is displayed If next frame is
				 *  available, release curFrm and move on*/

				/* Copy frame display time */
				common->curFrm->ts = timevalue;
				/* Change status of the curFrm */
				common->curFrm->state = STATE_DONE;
				/* unlock semaphore on curFrm */
				wake_up_interruptible(&common->curFrm->done);
				/* Make curFrm pointing to
				   nextFrm */
				common->curFrm = common->nextFrm;
				if (V4L2_BUF_TYPE_SLICED_VBI_OUTPUT
				    == common->fmt.type) {
					queue_work(vbi_workqueue,
						   &vbi_work[channel->
							     channel_id]);
				}
			} else if (1 == fid) {	/* odd field */
				if (list_empty(&common->dma_queue)
				    || (common->curFrm != common->nextFrm)) {
					continue;
				}

				/* one field is displayed configure
				   the next frame if it is available
				   otherwise hold on current frame
				 */
				/* Get next from the buffer
				   queue */
				common->nextFrm = list_entry(common->
							     dma_queue.
							     next, struct
							     videobuf_buffer,
							     queue);

				/* Remove that from the
				   buffer queue */
				list_del(&common->nextFrm->queue);

				/* Mark state of the frame
				   to active */
				common->nextFrm->state = STATE_ACTIVE;
				addr = common->nextFrm->boff;

				if (V4L2_BUF_TYPE_SLICED_VBI_OUTPUT
				    == common->fmt.type) {
					queue_work(vbi_workqueue,
						   &vbi_work[channel->
							     channel_id]);
				} else {
					common->set_addr(addr +
							 common->ytop_off,
							 addr +
							 common->ybtm_off,
							 addr +
							 common->ctop_off,
							 addr +
							 common->cbtm_off);
				}

			}
		}
	}
	dev_dbg(vpif_dev, "</vpif_channel_isr>\n");
	return IRQ_HANDLED;
}

static void vpif_get_std_info(struct channel_obj *ch)
{
	struct video_obj *vid_ch = &(ch->video);
	struct common_obj *common = &(ch->common[VPIF_VIDEO_INDEX]);
	int ret;

	vid_ch->std_info.channel_id = ch->channel_id + 2;

	/* Get standard name from the encoder by enumerating standards */
	strncpy(vid_ch->std_info.name, vid_ch->mode_info.name,
		sizeof(vid_ch->std_info.name));

	/* Get standard information from VPIF layer */
	ret = vpif_get_mode_info(&vid_ch->std_info);
	common->fmt.fmt.pix.width = vid_ch->std_info.activepixels;
	common->fmt.fmt.pix.height = vid_ch->std_info.activelines;

	/* Set height and width paramateres */
	ch->common[VPIF_VIDEO_INDEX].height = vid_ch->std_info.activelines;
	ch->common[VPIF_VIDEO_INDEX].width = vid_ch->std_info.activepixels;
}

/*
 * vpif_calculate_offsets: This function calculates buffers offset for Y and C
 * in the top and bottom field */
static void vpif_calculate_offsets(struct channel_obj *channel)
{
	unsigned int hpitch, vpitch, sizeimage;
	struct common_obj *common = &(channel->common[VPIF_VIDEO_INDEX]);
	struct video_obj *vid_ch = &(channel->video);
	struct vpif_params *vpifparams = &channel->vpifparams;
	enum v4l2_field field = common->fmt.fmt.pix.field;

	dev_dbg(vpif_dev, "<vpif_calculate_offsets>\n");

	if (V4L2_FIELD_ANY == common->fmt.fmt.pix.field) {
		if (vid_ch->std_info.frame_format)
			vid_ch->buf_field = V4L2_FIELD_NONE;
		else
			vid_ch->buf_field = V4L2_FIELD_INTERLACED;
	} else {
		vid_ch->buf_field = common->fmt.fmt.pix.field;
	}

	if (V4L2_MEMORY_USERPTR == common->memory) {
		sizeimage = common->fmt.fmt.pix.sizeimage;
	} else {
		sizeimage = config_params.channel_bufsize[channel->channel_id];
	}

	hpitch = common->fmt.fmt.pix.bytesperline;
	vpitch = sizeimage / (hpitch * 2);

	if ((V4L2_FIELD_NONE == vid_ch->buf_field) ||
	    (V4L2_FIELD_INTERLACED == vid_ch->buf_field)) {
		/* Calculate offsets for Y top, Y Bottom, C top and C Bottom */
		common->ytop_off = 0;
		common->ybtm_off = hpitch;
		common->ctop_off = sizeimage / 2;
		common->cbtm_off = sizeimage / 2 + hpitch;
	} else if (V4L2_FIELD_SEQ_TB == vid_ch->buf_field) {
		/* Calculate offsets for Y top, Y Bottom, C top and C Bottom */
		common->ytop_off = 0;
		common->ybtm_off = sizeimage / 4;
		common->ctop_off = sizeimage / 2;
		common->cbtm_off = common->ctop_off + sizeimage / 4;
	} else if (V4L2_FIELD_SEQ_BT == vid_ch->buf_field) {
		/* Calculate offsets for Y top, Y Bottom, C top and C Bottom */
		common->ybtm_off = 0;
		common->ytop_off = sizeimage / 4;
		common->cbtm_off = sizeimage / 2;
		common->ctop_off = common->cbtm_off + sizeimage / 4;
	}
	if ((V4L2_FIELD_NONE == vid_ch->buf_field) ||
	    (V4L2_FIELD_INTERLACED == vid_ch->buf_field)) {
		vpifparams->video_params.storage_mode = 1;
	} else {
		vpifparams->video_params.storage_mode = 0;
	}

	if (vid_ch->std_info.frame_format == 1) {
		vpifparams->video_params.hpitch =
		    common->fmt.fmt.pix.bytesperline;
	} else {
		if ((field == V4L2_FIELD_ANY)
		    || (field == V4L2_FIELD_INTERLACED))
			vpifparams->video_params.hpitch =
			    common->fmt.fmt.pix.bytesperline * 2;
		else
			vpifparams->video_params.hpitch =
			    common->fmt.fmt.pix.bytesperline;
	}
	strncpy(channel->vpifparams.video_params.name,
		vid_ch->std_info.name,
		sizeof(channel->vpifparams.video_params.name));
	dev_dbg(vpif_dev, "</vpif_calculate_offsets>\n");
}

static void vpif_calculate_offsets_vbi(struct channel_obj *channel, u8 index)
{
	struct common_obj *common = &(channel->common[index]);
	dev_dbg(vpif_dev, "<vpif_calculate_offsets_vbi>\n");

	common->ytop_off = 0;
	common->ctop_off = 0;
	common->ybtm_off = (common->fmt.fmt.vbi.count[0]) *
	    (common->fmt.fmt.vbi.samples_per_line);
	common->cbtm_off = 0;

	dev_dbg(vpif_dev, "</vpif_calculate_offsets_vbi>\n");
}
static void vpif_config_format(struct channel_obj *channel)
{
	struct common_obj *common = &(channel->common[VPIF_VIDEO_INDEX]);

	common->fmt.fmt.pix.field = V4L2_FIELD_ANY;

	if (config_params.numbuffers[channel->channel_id] == 0)
		common->memory = V4L2_MEMORY_USERPTR;
	else
		common->memory = V4L2_MEMORY_MMAP;

	common->fmt.fmt.pix.sizeimage
	    = config_params.channel_bufsize[channel->channel_id];

	common->fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV422UVP;
	common->fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	channel->common[VPIF_VBI_INDEX].fmt.type =
	    V4L2_BUF_TYPE_SLICED_VBI_OUTPUT;
	channel->common[VPIF_VBI_INDEX].fmt.fmt.sliced.service_set = 0;
	channel->common[VPIF_VBI_INDEX].memory = V4L2_MEMORY_MMAP;

	channel->common[VPIF_HBI_INDEX].fmt.type = V4L2_BUF_TYPE_HBI_OUTPUT;
	channel->common[VPIF_HBI_INDEX].memory = V4L2_MEMORY_MMAP;
}

static int vpif_check_format(struct channel_obj *channel,
			     struct v4l2_pix_format *pixfmt)
{
	u32 sizeimage, hpitch, vpitch;
	enum v4l2_field field = pixfmt->field;
	struct video_obj *vid_ch = &(channel->video);
	struct common_obj *common = &(channel->common[VPIF_VIDEO_INDEX]);
	dev_dbg(vpif_dev, "<vpif_check_format>\n");

	if (pixfmt->pixelformat != V4L2_PIX_FMT_YUV422UVP) {
		dev_err(vpif_dev, "invalid frame format\n");
		return -EINVAL;
	}
	if (!(VPIF_VALID_FIELD(field))) {
		dev_err(vpif_dev, "invalid field format\n");
		return -EINVAL;
	}
	if (pixfmt->bytesperline <= 0) {
		dev_err(vpif_dev, "invalid pitch\n");
		return -EINVAL;
	}

	if (V4L2_MEMORY_USERPTR == common->memory) {
		sizeimage = pixfmt->sizeimage;
	} else {
		sizeimage = config_params.channel_bufsize[channel->channel_id];
	}

	davinci_enc_get_mode(channel->channel_id, &channel->video.mode_info);
	vpif_get_std_info(channel);

	hpitch = pixfmt->bytesperline;
	vpitch = sizeimage / (hpitch * 2);

	/* Check for valid value of pitch */
	if ((hpitch < vid_ch->std_info.activepixels) ||
	    (vpitch < vid_ch->std_info.activelines)) {
		dev_err(vpif_dev, "Invalid pitch\n");
		return -EINVAL;
	}
	/* Check for 8 byte alignment */
	if (!(ISALIGNED(hpitch))) {
		dev_err(vpif_dev, "invalid pitch alignment\n");
		return -EINVAL;
	}
	pixfmt->width = common->fmt.fmt.pix.width;
	pixfmt->height = common->fmt.fmt.pix.height;

	dev_dbg(vpif_dev, "</vpif_check_format>\n");
	return 0;
}

static void vpif_config_addr(struct channel_obj *channel, int muxmode)
{
	struct common_obj *common = &(channel->common[VPIF_VIDEO_INDEX]);
	dev_dbg(vpif_dev, "<vpif_config_addr>");
	if (VPIF_CHANNEL3_VIDEO == channel->channel_id) {
		common->set_addr = ch3_set_videobuf_addr;
	} else {
		if (2 == muxmode) {
			common->set_addr = ch2_set_videobuf_addr_yc_nmux;
		} else {
			common->set_addr = ch2_set_videobuf_addr;
		}
	}
	dev_dbg(vpif_dev, "</vpif_config_addr>");
}

static void vpif_config_addr_vbi(struct channel_obj *channel,
				 int channel_id, u8 index)
{
	struct common_obj *common = &(channel->common[index]);
	dev_dbg(vpif_dev, "<vpif_config_addr_vbi>");
	if (VPIF_CHANNEL3_VIDEO == channel_id) {
		common->set_addr = ch3_set_vbi_addr;
	} else {
		common->set_addr = ch2_set_vbi_addr;
	}
	dev_dbg(vpif_dev, "</vpif_config_addr_vbi>");
}

static int vpif_try_raw_format(struct channel_obj *channel,
			       struct v4l2_vbi_format *fmt, u8 index)
{
	struct video_obj *vid_ch = &(channel->video);
	struct v4l2_vbi_format *raw_vbi;
	if (0 == strcmp(vid_ch->mode_info.name, "NTSC")) {
		raw_vbi = (1 == index) ? &(vpif_raw_vbi_formats[0]) :
		    &(vpif_raw_hbi_formats[0]);
	} else {
		raw_vbi = (1 == index) ? &(vpif_raw_vbi_formats[1]) :
		    &(vpif_raw_hbi_formats[1]);
	}

	if ((fmt->sampling_rate != raw_vbi->sampling_rate) ||
	    (fmt->samples_per_line != raw_vbi->samples_per_line) ||
	    (fmt->start[0] != raw_vbi->start[0]) ||
	    (fmt->count[0] != raw_vbi->count[0]) ||
	    (fmt->start[1] != raw_vbi->start[1]) ||
	    (fmt->count[1] != raw_vbi->count[1]) ||
	    (fmt->flags != raw_vbi->flags)) {
		dev_err(vpif_dev, "invalid parameters\n");
		*fmt = *raw_vbi;
		return -EINVAL;
	}
	return 0;
}
int vpif_try_sliced_vbi(struct v4l2_format *fmt,
			struct vid_enc_sliced_vbi_service *service)
{
	u16 serv = 0;
	if (service->service_set & VID_ENC_SLICED_VBI_CGMS_NTSC)
		serv |= V4L2_SLICED_CGMS_525;
	if (service->service_set & VID_ENC_SLICED_VBI_CC_NTSC)
		serv |= V4L2_SLICED_CAPTION_525;
	if (service->service_set & VID_ENC_SLICED_VBI_WSS_PAL)
		serv |= V4L2_SLICED_WSS_625;
	if (fmt->fmt.sliced.service_set & (~serv)) {
		fmt->fmt.sliced.service_set = serv;
		return -EINVAL;
	}
	return 0;
}
int vpif_convert_vbi_services(struct v4l2_format *fmt,
			      struct vid_enc_sliced_vbi_service *service)
{
	service->service_set = 0;
	if (fmt->fmt.sliced.service_set & V4L2_SLICED_CGMS_525)
		service->service_set |= VID_ENC_SLICED_VBI_CGMS_NTSC;
	if (fmt->fmt.sliced.service_set & V4L2_SLICED_CAPTION_525)
		service->service_set |= VID_ENC_SLICED_VBI_CC_NTSC;
	if (fmt->fmt.sliced.service_set & V4L2_SLICED_WSS_625)
		service->service_set |= VID_ENC_SLICED_VBI_WSS_PAL;
	if (0 == service->service_set && 0 != fmt->fmt.sliced.service_set)
		return -EINVAL;
	return 0;
}

/*
 * vpif_doioctl: This function will provide different V4L2 commands.This
 * function can be used to configure driver or get status of driver as per
 * command passed by application */
static int vpif_doioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct vpif_fh *fh = file->private_data;
	struct channel_obj *channel = fh->channel;
	struct video_obj *vid_ch = &(channel->video);
	unsigned int index = 0;
	unsigned long addr = 0, flags;
	dev_dbg(vpif_dev, "<vpif_doioctl>\n");

	/* This file handle has not initialized the channel, 
	   It is not allowed to do settings */
	if ((VPIF_CHANNEL2_VIDEO == channel->channel_id)
	    || (VPIF_CHANNEL3_VIDEO == channel->channel_id)) {
		switch (cmd) {
		case VIDIOC_S_FMT:
		case VIDIOC_REQBUFS:
		case VPIF_S_VPIF_PARAMS:
		case VPIF_CMD_S_ENCODER_PARAMS:
			if (!fh->initialized) {
				dev_err(vpif_dev, "Channel Busy\n");
				return -EBUSY;
			}
		}
	}
	/* Check for the priority */
	if ((VPIF_CHANNEL2_VIDEO == channel->channel_id)
	    || (VPIF_CHANNEL3_VIDEO == channel->channel_id)) {
		switch (cmd) {
		case VIDIOC_S_FMT:
			ret = v4l2_prio_check(&channel->prio, &fh->prio);
			if (0 != ret)
				return ret;
			fh->initialized = 1;
			break;
		}
	}
	/* Check for null value of parameter */
	if (ISNULL((void *)arg)) {
		dev_err(vpif_dev, "Null pointer\n");
		return -EINVAL;
	}
	/* Switch on the command value */
	switch (cmd) {
		/* If the case is for querying capabilities */
	case VIDIOC_QUERYCAP:
		{
			struct v4l2_capability *cap =
			    (struct v4l2_capability *)arg;
			dev_dbg(vpif_dev, "VIDIOC_QUERYCAP\n");
			memset(cap, 0, sizeof(*cap));
			if ((VPIF_CHANNEL2_VIDEO == channel->channel_id)
			    || (VPIF_CHANNEL3_VIDEO == channel->channel_id)) {
				*cap = vpif_videocap;
			} else {
				ret = -EINVAL;
			}
			break;
		}

		/* If the case is for enumerating formats */
	case VIDIOC_ENUM_FMT:
		{
			struct v4l2_fmtdesc *fmt = (struct v4l2_fmtdesc *)arg;
			dev_dbg(vpif_dev, "VIDIOC_ENUM_FMT\n");
			if (fmt->index != 0) {
				dev_err(vpif_dev, "Invalid format index\n");
				return -EINVAL;
			}
			/* Fill in the information about format */
			index = fmt->index;
			memset(fmt, 0, sizeof(*fmt));
			fmt->index = index;
			fmt->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
			strcpy(fmt->description, "YCbCr4:2:2 YC Planar");
			fmt->pixelformat = V4L2_PIX_FMT_YUV422UVP;
			break;
		}

		/* If the case is for getting formats */
	case VIDIOC_G_FMT:
		{
			struct v4l2_format *fmt = (struct v4l2_format *)arg;
			struct common_obj *common = NULL;
			common =
			    (fmt->type ==
			     V4L2_BUF_TYPE_VIDEO_OUTPUT) ? &(channel->
							     common
							     [VPIF_VIDEO_INDEX])
			    : ((fmt->type ==
				V4L2_BUF_TYPE_HBI_OUTPUT) ? &(channel->
							      common
							      [VPIF_HBI_INDEX])
			       : &(channel->common[VPIF_VBI_INDEX]));

			dev_dbg(vpif_dev, "VIDIOC_G_FMT\n");

			/* Check the validity of the buffer type */
			if (common->fmt.type != fmt->type) {
				ret = -EINVAL;
				break;
			}
			/* If buffer type is not video output */
			if (V4L2_BUF_TYPE_VIDEO_OUTPUT != fmt->type) {
				if (vid_ch->std_info.vbi_supported == 0)
					return -EINVAL;
			}
			/* Fill in the information about
			 * format */
			down_interruptible(&(common->lock));
			ret = davinci_enc_get_mode(channel->channel_id,
						   &channel->video.mode_info);
			if (ret < 0)
				return ret;

			vpif_get_std_info(channel);
			*fmt = common->fmt;
			up(&(common->lock));
			dev_dbg(vpif_dev, "SUCCESS.\n");
			break;
		}

		/* If the case is for setting formats */
	case VIDIOC_S_FMT:
		{
			struct v4l2_format *fmt = (struct v4l2_format *)arg;
			struct common_obj *common = NULL;
			common =
			    (fmt->type ==
			     V4L2_BUF_TYPE_VIDEO_OUTPUT) ? &(channel->
							     common
							     [VPIF_VIDEO_INDEX])
			    : ((fmt->type ==
				V4L2_BUF_TYPE_HBI_OUTPUT) ? &(channel->
							      common
							      [VPIF_HBI_INDEX])
			       : &(channel->common[VPIF_VBI_INDEX]));

			dev_dbg(vpif_dev, "VIDIOC_S_FMT\n");

			/* If streaming is started, return error */
			if (common->started) {
				dev_err(vpif_dev, "Streaming is started\n");
				return -EBUSY;
			}
			if (V4L2_BUF_TYPE_VIDEO_OUTPUT == fmt->type) {
				struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
				/* Check for valid field format */
				ret = vpif_check_format(channel, pixfmt);

				if (ret) {
					return ret;
				}

				/* store the pixel format in the channel
				 * object */
				common->fmt.fmt.pix = *pixfmt;
				dev_dbg(vpif_dev, "Success.\n");
			} else {
				if (vid_ch->std_info.vbi_supported == 0) {
					dev_err
					    (vpif_dev,
					     "standard doesn't support\n");
					return -EINVAL;
				}

				if (V4L2_BUF_TYPE_SLICED_VBI_OUTPUT
				    == fmt->type) {
					struct vid_enc_sliced_vbi_service
					    service;
					ret =
					    vpif_convert_vbi_services(fmt,
								      &service);
					if (ret < 0)
						return ret;
					ret =
					    davinci_enc_enable_sliced_vbi
					    (channel->channel_id, &service);
					if (ret >= 0) {
						common->height = 1;
						common->width =
						    VPIF_SLICED_BUF_SIZE;
						channel->vbi.num_services = ret;
					}
				} else if ((V4L2_BUF_TYPE_VBI_OUTPUT
					    == fmt->type) ||
					   (V4L2_BUF_TYPE_HBI_OUTPUT ==
					    fmt->type)) {
					index =
					    (V4L2_BUF_TYPE_VBI_OUTPUT ==
					     fmt->
					     type) ? VPIF_VBI_INDEX :
					    VPIF_HBI_INDEX;
					ret =
					    vpif_try_raw_format(channel,
								&(fmt->fmt.
								  vbi), index);
					if (ret < 0)
						return ret;
					if (V4L2_BUF_TYPE_VBI_OUTPUT
					    == fmt->type)
						ret =
						    davinci_enc_enable_vbi
						    (channel->channel_id, 1);
					else
						ret =
						    davinci_enc_enable_hbi
						    (channel->channel_id, 1);
					if (ret < 0)
						return ret;
					common->height =
					    common->fmt.fmt.vbi.count[0] +
					    common->fmt.fmt.vbi.count[1];
					common->width = common->fmt.
					    fmt.vbi.samples_per_line;
				} else {
					dev_err(vpif_dev, "invalid type\n");
					ret = -EINVAL;
				}
			}
			if (ret < 0)
				return ret;

			/* store the format in the channel
			 * object */
			down_interruptible(&common->lock);
			common->fmt = *fmt;
			up(&common->lock);
			break;
		}

		/* If the case is for trying formats */
	case VIDIOC_TRY_FMT:
		{
			struct v4l2_format *fmt = NULL;
			struct common_obj *common = NULL;
			fmt = (struct v4l2_format *)arg;
			dev_dbg(vpif_dev, "VIDIOC_TRY_FMT\n");
			common =
			    (fmt->type ==
			     V4L2_BUF_TYPE_VIDEO_OUTPUT) ? &(channel->
							     common
							     [VPIF_VIDEO_INDEX])
			    : ((fmt->type ==
				V4L2_BUF_TYPE_HBI_OUTPUT) ? &(channel->
							      common
							      [VPIF_HBI_INDEX])
			       : &(channel->common[VPIF_VBI_INDEX]));

			if (V4L2_BUF_TYPE_VIDEO_OUTPUT == fmt->type) {
				struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
				/* Check for valid field format */
				ret = vpif_check_format(channel, pixfmt);
				if (ret) {
					*pixfmt = common->fmt.fmt.pix;
					pixfmt->sizeimage =
					    pixfmt->width * pixfmt->height * 2;
				}
			} else {
				if (vid_ch->std_info.vbi_supported == 0) {
					dev_err(vpif_dev,
						"standard doesn't support\n");
					return -EINVAL;
				}
				if (V4L2_BUF_TYPE_SLICED_VBI_OUTPUT
				    == fmt->type) {
					struct vid_enc_sliced_vbi_service
					    service;
					ret =
					    davinci_enc_get_sliced_cap
					    (channel->channel_id, &service);
					if (ret < 0)
						return ret;
					ret = vpif_try_sliced_vbi(fmt,
								  &service);
				} else if ((V4L2_BUF_TYPE_VBI_OUTPUT
					    == fmt->type) ||
					   (V4L2_BUF_TYPE_HBI_OUTPUT ==
					    fmt->type)) {
					index =
					    (V4L2_BUF_TYPE_VBI_OUTPUT ==
					     fmt->
					     type) ? VPIF_VBI_INDEX :
					    VPIF_HBI_INDEX;
					ret |=
					    vpif_try_raw_format(channel,
								&(fmt->fmt.
								  vbi), index);
				} else {
					dev_err(vpif_dev, "invalid type\n");
					ret = -EINVAL;
				}
			}
			if (ret < 0)
				return ret;
			break;
		}

		/* If the case is for getting encoder parameters */
	case VPIF_CMD_G_ENCODER_PARAMS:
		{
			dev_dbg(vpif_dev, "VPIF_CMD_G_ENCODER_PARAMS\n");
			ret = davinci_enc_getparams(channel->channel_id,
						    (void *)arg);
			break;
		}

		/* If the case is for setting encoder parameters */
	case VPIF_CMD_S_ENCODER_PARAMS:
		{
			struct common_obj *common =
			    &(channel->common[VPIF_VIDEO_INDEX]);
			dev_dbg(vpif_dev, "VPIF_CMD_S_ENCODER_PARAMS\n");
			down_interruptible(&common->lock);
			/* If channel is already started, return
			 * error */
			if (common->started) {
				dev_err(vpif_dev, "streaming is started\n");
				up(&common->lock);
				return -EBUSY;
			}
			ret = davinci_enc_setparams(channel->channel_id,
						    (void *)arg);
			up(&common->lock);
			break;
		}

		/* If the case is for requesting buffer allocation */
	case VIDIOC_REQBUFS:
		{
			struct v4l2_requestbuffers *reqbuf =
			    (struct v4l2_requestbuffers *)arg;
			enum v4l2_field field;
			u8 index = 0;
			struct common_obj *common = NULL;
			reqbuf = (struct v4l2_requestbuffers *)arg;
			dev_dbg(vpif_dev, "VIDIOC_REQBUFS\n");

			if ((V4L2_BUF_TYPE_VIDEO_OUTPUT != reqbuf->type) &&
			    (V4L2_BUF_TYPE_VBI_OUTPUT != reqbuf->type) &&
			    (V4L2_BUF_TYPE_SLICED_VBI_OUTPUT != reqbuf->type)
			    && (V4L2_BUF_TYPE_HBI_OUTPUT != reqbuf->type))
				return -EINVAL;
			index = (V4L2_BUF_TYPE_VIDEO_OUTPUT
				 == reqbuf->type) ? VPIF_VIDEO_INDEX :
			    ((V4L2_BUF_TYPE_HBI_OUTPUT == reqbuf->type) ?
			     VPIF_HBI_INDEX : VPIF_VBI_INDEX);
			common = &(channel->common[index]);

			/* Check the validity of the buffer type */
			if (common->fmt.type != reqbuf->type) {
				ret = -EINVAL;
				break;
			}
			/* Only mmap is supported for sliced VBI */
			if ((V4L2_BUF_TYPE_SLICED_VBI_OUTPUT == reqbuf->type)
			    && (V4L2_MEMORY_MMAP != reqbuf->memory)) {
				ret = -EINVAL;
				break;
			}
			/* If io users of the channel is not zero,
			   return error */
			if (0 != common->io_usrs) {
				ret = -EBUSY;
				break;
			}
			down_interruptible(&common->lock);
			if (reqbuf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
				if (common->fmt.fmt.pix.field == V4L2_FIELD_ANY)
					field = V4L2_FIELD_INTERLACED;
				else
					field = common->fmt.fmt.pix.field;
			} else {
				field = V4L2_VBI_INTERLACED;
			}
			/* Initialize videobuf queue as per the
			   buffer type */
			videobuf_queue_init(&common->buffer_queue,
					    &video_qops, NULL,
					    &common->irqlock,
					    reqbuf->type,
					    field,
					    sizeof(struct videobuf_buffer), fh);
			/* Set buffer to Linear buffer */
			videobuf_set_buftype(&common->buffer_queue,
					     VIDEOBUF_BUF_LINEAR);
			/* Set io allowed member of file handle to
			 * TRUE */
			fh->io_allowed[index] = 1;
			/* Increment io usrs member of channel object
			   to 1 */
			common->io_usrs = 1;
			/* Store type of memory requested in channel
			   object */
			common->memory = reqbuf->memory;
			/* Initialize buffer queue */
			INIT_LIST_HEAD(&common->dma_queue);
			/* Allocate buffers */
			ret = videobuf_reqbufs(&common->buffer_queue, reqbuf);
			up(&common->lock);
			break;
		}

		/* If the case is for en-queing buffer in the buffer
		 * queue */
	case VIDIOC_QBUF:
		{
			struct v4l2_buffer tbuf = *(struct v4l2_buffer *)arg;
			u8 index = 0;
			struct common_obj *common = NULL;
			struct videobuf_buffer *buf1;
			dev_dbg(vpif_dev, "VIDIOC_QBUF\n");
			index = (V4L2_BUF_TYPE_VIDEO_OUTPUT
				 == tbuf.type) ? VPIF_VIDEO_INDEX :
			    ((V4L2_BUF_TYPE_HBI_OUTPUT == tbuf.type) ?
			     VPIF_HBI_INDEX : VPIF_VBI_INDEX);
			common = &(channel->common[index]);

			/* Check the validity of the buffer type */
			if (common->fmt.type != tbuf.type) {
				ret = -EINVAL;
				break;
			}
			/* If this file handle is not allowed to do IO,
			   return error */
			if (!fh->io_allowed[index]) {
				dev_err(vpif_dev, "fh->io_allowed\n");
				ret = -EACCES;
				break;
			}
			if (!(list_empty(&common->dma_queue)) ||
			    (common->curFrm != common->nextFrm) ||
			    !(common->started) ||
			    (common->started && (0 == channel->field_id))) {
				ret =
				    videobuf_qbuf(&common->buffer_queue,
						  (struct v4l2_buffer *)
						  arg);
				break;
			}
			if (V4L2_BUF_TYPE_VIDEO_OUTPUT != common->fmt.type) {
				ret = videobuf_qbuf(&common->buffer_queue,
						    (struct v4l2_buffer *)
						    arg);
				break;
			}
			/* bufferqueue is empty store buffer address
			 *  in VPIF registers */
			down(&common->buffer_queue.lock);
			tbuf = *(struct v4l2_buffer *)arg;
			buf1 = common->buffer_queue.bufs[tbuf.index];
			if (buf1->memory != tbuf.memory) {
				dev_err(vpif_dev, "invalid buffer" " type\n");
				up(&common->buffer_queue.lock);
				return -EINVAL;
			}
			if ((buf1->state == STATE_QUEUED) ||
			    (buf1->state == STATE_ACTIVE)) {
				dev_err(vpif_dev, "invalid state\n");
				up(&common->buffer_queue.lock);
				return -EINVAL;
			}

			switch (buf1->memory) {
			case V4L2_MEMORY_MMAP:
				if (buf1->baddr == 0) {
					up(&common->buffer_queue.lock);
					return -EINVAL;
				}
				break;
			case V4L2_MEMORY_USERPTR:
				if (tbuf.length < buf1->bsize) {
					up(&common->buffer_queue.lock);
					return -EINVAL;
				}
				if ((STATE_NEEDS_INIT != buf1->state)
				    && (buf1->baddr != tbuf.m.userptr))
					vpif_buffer_release(&common->
							    buffer_queue, buf1);
				buf1->baddr = tbuf.m.userptr;
				break;
			default:
				up(&common->buffer_queue.lock);
				return -EINVAL;
			}
			local_irq_save(flags);
			ret =
			    vpif_buffer_prepare(&common->buffer_queue,
						buf1,
						common->buffer_queue.field);
			if (ret < 0) {
				local_irq_restore(flags);
				up(&common->buffer_queue.lock);
				return -EINVAL;
			}
			buf1->state = STATE_ACTIVE;
			addr = buf1->boff;
			common->nextFrm = buf1;
			if (tbuf.type != V4L2_BUF_TYPE_SLICED_VBI_OUTPUT) {
				common->set_addr((addr + common->ytop_off),
						 (addr + common->ybtm_off),
						 (addr + common->ctop_off),
						 (addr + common->cbtm_off));
			}
			local_irq_restore(flags);
			list_add_tail(&buf1->stream,
				      &(common->buffer_queue.stream));
			up(&common->buffer_queue.lock);
			break;

		}

		/* If the case is for de-queing buffer from the
		 * buffer queue */
	case VIDIOC_DQBUF:
		{
			u8 index = 0;
			struct common_obj *common = NULL;
			dev_dbg(vpif_dev, "VIDIOC_DQBUF\n");
			index = (V4L2_BUF_TYPE_VIDEO_OUTPUT ==
				 ((struct v4l2_buffer *)arg)->type) ?
			    VPIF_VIDEO_INDEX :
			    ((V4L2_BUF_TYPE_HBI_OUTPUT ==
			      ((struct v4l2_buffer *)arg)->type) ?
			     VPIF_HBI_INDEX : VPIF_VBI_INDEX);
			common = &(channel->common[index]);

			if (file->f_flags & O_NONBLOCK)
				/* Call videobuf_dqbuf for non
				   blocking mode */
				ret =
				    videobuf_dqbuf(&common->buffer_queue,
						   (struct v4l2_buffer *)
						   arg, 1);
			else
				/* Call videobuf_dqbuf for
				   blocking mode */
				ret =
				    videobuf_dqbuf(&common->buffer_queue,
						   (struct v4l2_buffer *)
						   arg, 0);
			break;
		}

		/* If the case is for querying information about
		 *  buffer for memory mapping io */
	case VIDIOC_QUERYBUF:
		{
			struct v4l2_buffer tbuf = *(struct v4l2_buffer *)arg;
			u8 index = 0;
			struct common_obj *common = NULL;
			dev_dbg(vpif_dev, "VIDIOC_QUERYBUF\n");
			index = (V4L2_BUF_TYPE_VIDEO_OUTPUT
				 == tbuf.type) ? VPIF_VIDEO_INDEX :
			    ((V4L2_BUF_TYPE_HBI_OUTPUT == tbuf.type) ?
			     VPIF_HBI_INDEX : VPIF_VBI_INDEX);
			common = &(channel->common[index]);
			/* Check the validity of the buffer type */
			if (common->fmt.type != tbuf.type) {
				ret = -EINVAL;
				break;
			}

			if (tbuf.memory != V4L2_MEMORY_MMAP)
				return -EINVAL;
			/* Call videobuf_querybuf to get information */
			ret = videobuf_querybuf(&common->buffer_queue,
						(struct v4l2_buffer *)
						arg);
			break;
		}

		/* If the case is starting streaming */
	case VIDIOC_STREAMON:
		{
			enum v4l2_buf_type buftype = *(enum v4l2_buf_type *)arg;
			u8 index =
			    (V4L2_BUF_TYPE_VIDEO_OUTPUT ==
			     buftype) ? (VPIF_VIDEO_INDEX)
			    : ((V4L2_BUF_TYPE_HBI_OUTPUT ==
				buftype) ? VPIF_HBI_INDEX : VPIF_VBI_INDEX);
			struct common_obj *common = &(channel->common[index]);
			struct channel_obj *oth_ch =
			    vpif_obj.dev[!channel->channel_id];
			struct vpif_params *vpif = &channel->vpifparams;
			dev_dbg(vpif_dev, "VIDIOC_STREAMON\n");
			/* If file handle is not allowed IO,
			 * return error */

			if (!fh->io_allowed[index]) {
				dev_err(vpif_dev, "fh->io_allowed\n");
				ret = -EACCES;
				break;
			}
			/* If Streaming is already started,
			 * return error */
			if (common->started) {
				dev_err(vpif_dev, "channel->started\n");
				ret = -EBUSY;
				break;
			}
			if ((channel->channel_id == VPIF_CHANNEL2_VIDEO
			     && oth_ch->common[VPIF_VIDEO_INDEX].started &&
			     channel->video.std_info.ycmux_mode == 0)
			    || ((channel->channel_id == VPIF_CHANNEL3_VIDEO)
				&& (2 == oth_ch->common[VPIF_VIDEO_INDEX].
				    started))) {
				dev_err(vpif_dev, "other channel is using\n");
				ret = -EBUSY;
				break;
			}
			if (index == VPIF_VIDEO_INDEX) {
				ret = vpif_check_format
				    (channel, &(common->fmt.fmt.pix));
				if (ret < 0) {
					return ret;
				}
			} else {
				if (!channel->common[VPIF_VIDEO_INDEX].started)
					return -EINVAL;
			}

			/* Call videobuf_streamon to start streaming
			   in videobuf */
			ret = videobuf_streamon(&common->buffer_queue);
			if (ret < 0) {
				dev_err(vpif_dev, "videobuf_streamon\n");
				break;
			}
			down_interruptible(&common->lock);
			/* If buffer queue is empty, return error */
			if (list_empty(&common->dma_queue)) {
				dev_err(vpif_dev, "buffer queue is empty\n");
				ret = -EIO;
				up(&common->lock);
				break;
			}
			/* Get the next frame from the buffer queue */
			common->nextFrm = common->curFrm =
			    list_entry(common->dma_queue.next,
				       struct videobuf_buffer, queue);
			/* Remove buffer from the buffer queue */
			list_del(&common->curFrm->queue);
			/* Mark state of the current frame to active */
			common->curFrm->state = STATE_ACTIVE;
			/* Initialize field_id and started member */
			channel->field_id = 0;
			common->started = 1;

			if (buftype == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
				addr = common->curFrm->boff;
				/* Calculate the offset for Y and C data
				   in the buffer */
				vpif_calculate_offsets(channel);

				if ((vid_ch->std_info.frame_format &&
				     ((common->fmt.fmt.pix.field !=
				       V4L2_FIELD_NONE)
				      && (common->fmt.fmt.pix.field !=
					  V4L2_FIELD_ANY)))
				    || (!vid_ch->std_info.frame_format
					&& (common->fmt.fmt.pix.field ==
					    V4L2_FIELD_NONE))) {
					dev_err(vpif_dev,
						"conflict in field format \
							and std format\n");
					up(&common->lock);
					return -EINVAL;
				}
				/* if single channel is needed */
				if (vid_ch->std_info.ycmux_mode == 1) {
					ret = set_vid_out_mode_for_sd();
				} else {
					ret = set_vid_out_mode_for_hd();
				}
				if (ret < 0) {
					dev_err(vpif_dev,
						"cann't set vid out bit\n");
					up(&common->lock);
					return ret;
				}
				/* Set clock settings */
				ret = set_vid_clock(vid_ch->std_info.hd_sd);

				/* Call vpif_set_params function to set
				 * the parameters and addresses */
				ret = vpif_set_video_params(vpif,
							    channel->
							    channel_id + 2);
				if (ret < 0) {
					up(&common->lock);
					return ret;
				}
				common->started = ret;
				vpif_config_addr(channel, ret);

				common->set_addr((addr + common->ytop_off),
						 (addr + common->ybtm_off),
						 (addr + common->ctop_off),
						 (addr + common->cbtm_off));
				/* Set interrupt for both the fields in
				   VPIF Register enable channel in
				   VPIF register */
				if (VPIF_CHANNEL2_VIDEO == channel->channel_id) {
					channel2_intr_assert();
					channel2_intr_enable(1);
					enable_channel2(1);
				}
				if ((VPIF_CHANNEL3_VIDEO == channel->channel_id)
				    || (common->started == 2)) {
					channel3_intr_assert();
					channel3_intr_enable(1);
					enable_channel3(1);
				}
				channel_first_int[VPIF_VIDEO_INDEX]
				    [channel->channel_id] = 1;
			} else if ((V4L2_BUF_TYPE_VBI_OUTPUT == buftype)
				   || (V4L2_BUF_TYPE_HBI_OUTPUT == buftype)) {
				if (V4L2_BUF_TYPE_VBI_OUTPUT == buftype)
					index = VPIF_VBI_INDEX;
				else
					index = VPIF_HBI_INDEX;
				/* Calculate the offset for vbi data
				   in the buffer */
				vpif_calculate_offsets_vbi(channel, index);

				/* Call vpif_set_params function to set
				 * the parameters and addresses */
				ret = vpif_set_vbi_display_params
				    (&channel->vpifparams.params.
				     vbi_params, channel->channel_id);
				if (ret < 0) {
					up(&(common->lock));
					return ret;
				}
				vpif_config_addr_vbi(channel,
						     channel->channel_id,
						     index);

				common->set_addr((addr + common->ytop_off),
						 (addr + common->ybtm_off),
						 (addr + common->ctop_off),
						 (addr + common->cbtm_off));
				if (VPIF_CHANNEL2_VIDEO == channel->channel_id)
					channel2_raw_enable(1, index);

				if (VPIF_CHANNEL3_VIDEO == channel->channel_id)
					channel3_raw_enable(1, index);

				channel_first_int[index][channel->channel_id]
				    = 1;
			} else {
				queue_work(vbi_workqueue,
					   &vbi_work[channel->channel_id]);
				channel_first_int
				    [VPIF_VBI_INDEX][channel->channel_id]
				    = 1;
			}
			up(&(common->lock));
			break;
		}

		/* If the case is for stopping streaming */
	case VIDIOC_STREAMOFF:
		{
			enum v4l2_buf_type buftype = *(enum v4l2_buf_type *)arg;
			u8 index =
			    (V4L2_BUF_TYPE_VIDEO_OUTPUT ==
			     buftype) ? (VPIF_VIDEO_INDEX)
			    : ((V4L2_BUF_TYPE_HBI_OUTPUT ==
				buftype) ? VPIF_HBI_INDEX : VPIF_VBI_INDEX);
			struct common_obj *common = &(channel->common[index]);
			dev_dbg(vpif_dev, "VIDIOC_STREAMOFF\n");
			/* If io is allowed for this file handle,
			   return error */
			if (!fh->io_allowed[index]) {
				dev_err(vpif_dev, "fh->io_allowed\n");
				ret = -EACCES;
				break;
			}
			/* If streaming is not started, return error */
			if (!common->started) {
				dev_err(vpif_dev, "channel->started\n");
				ret = -EINVAL;
				break;
			}
			down_interruptible(&common->lock);
			if (buftype == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
				/* disable channel */
				if (VPIF_CHANNEL2_VIDEO == channel->channel_id) {
					enable_channel2(0);
					channel2_intr_enable(0);
				}
				if ((VPIF_CHANNEL3_VIDEO ==
				     channel->channel_id) ||
				    (2 == common->started)) {
					enable_channel3(0);
					channel3_intr_enable(0);
				}
			} else if (buftype == V4L2_BUF_TYPE_VBI_OUTPUT) {
				/* disable raw vbi channel */
				if (VPIF_CHANNEL2_VIDEO == channel->channel_id) {
					channel2_raw_enable(0, index);
				} else {
					channel3_raw_enable(0, index);
				}
			} else {
				/* disabel sliced vbi channel */
				struct vid_enc_sliced_vbi_service services;
				services.service_set = 0;
				ret =
				    davinci_enc_enable_sliced_vbi(channel->
								  channel_id,
								  &services);
			}
			common->started = 0;
			up(&common->lock);
			ret = videobuf_streamoff(&common->buffer_queue);
			break;
		}

		/* If the case is for setting VPIF parameters */
	case VPIF_S_VPIF_PARAMS:
		{
			struct common_obj *common =
			    &(channel->common[VPIF_VIDEO_INDEX]);
			struct vpif_params *params = (struct vpif_params *)arg;
			dev_dbg(vpif_dev, "VPIF_S_PARAMS\n");
			/* If streaming is not started, return error */
			if (common->started) {
				ret = -EBUSY;
				break;
			}
			down_interruptible(&common->lock);
			channel->vpifparams = *params;
			up(&common->lock);
			common = &(channel->common[VPIF_VBI_INDEX]);
			break;
		}

		/* If the case is for getting VPIF Parameters */
	case VPIF_G_VPIF_PARAMS:
		{
			struct vpif_params *params = (struct vpif_params *)arg;
			struct common_obj *common =
			    &(channel->common[VPIF_VIDEO_INDEX]);
			dev_dbg(vpif_dev, "VPIF_G_PARAMS\n");
			down_interruptible(&common->lock);
			*params = channel->vpifparams;
			up(&common->lock);
			common = &(channel->common[VPIF_VBI_INDEX]);
			break;
		}
	case VIDIOC_S_PRIORITY:
		{
			enum v4l2_priority *p = (enum v4l2_priority *)arg;
			ret = v4l2_prio_change(&channel->prio, &fh->prio, *p);
			break;
		}
	case VIDIOC_G_PRIORITY:
		{
			enum v4l2_priority *p = (enum v4l2_priority *)arg;
			*p = v4l2_prio_max(&channel->prio);
			break;
		}
		/* If the case is for getting sliced vbi capabilites */
	case VIDIOC_G_SLICED_VBI_CAP:
		{
			struct vid_enc_sliced_vbi_service service;
			struct v4l2_sliced_vbi_cap *cap =
			    (struct v4l2_sliced_vbi_cap *)arg;
			ret =
			    davinci_enc_get_sliced_cap(channel->channel_id,
						       &service);
			cap->service_set = 0;
			if (service.service_set & VID_ENC_SLICED_VBI_CGMS_NTSC)
				cap->service_set |= V4L2_SLICED_CGMS_525;
			if (service.service_set & VID_ENC_SLICED_VBI_CC_NTSC)
				cap->service_set |= V4L2_SLICED_CAPTION_525;
			if (service.service_set & VID_ENC_SLICED_VBI_WSS_PAL)
				cap->service_set |= V4L2_SLICED_WSS_625;
			break;
		}
		/* If the case if for getting cropping parameters */
	case VIDIOC_CROPCAP:
		{
			struct v4l2_cropcap *crop = (struct v4l2_cropcap *)arg;
			struct common_obj *common =
			    &(channel->common[VPIF_VIDEO_INDEX]);
			if (V4L2_BUF_TYPE_VIDEO_OUTPUT != crop->type)
				return -EINVAL;
			crop->bounds.left = crop->bounds.top = 0;
			crop->defrect.left = crop->defrect.top = 0;
			crop->defrect.height = crop->bounds.height =
			    common->height;
			crop->defrect.width = crop->bounds.width =
			    common->width;
			break;
		}
	default:
		dev_dbg(vpif_dev, "Invalid command\n");
		return -EINVAL;
	}

	dev_dbg(vpif_dev, "<vpif_doioctl>\n");
	return ret;
}

/*
 * vpif_ioctl: Calls vpif_doioctl function */
static int vpif_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	int ret;
	char sbuf[128];
	void *mbuf = NULL;
	void *parg = NULL;

	dev_dbg(vpif_dev, "Start of vpif ioctl\n");

	if (ISENCODERCMD(cmd)) {
		ret = vpif_doioctl(inode, file, cmd, (unsigned long)arg);
		if (ret == -ENOIOCTLCMD)
			ret = -EINVAL;
		goto out;
	}

	/*  Copy arguments into temp kernel buffer  */
	switch (_IOC_DIR(cmd)) {
	case _IOC_NONE:
		parg = NULL;
		break;
	case _IOC_READ:
	case _IOC_WRITE:
	case (_IOC_WRITE | _IOC_READ):
		if (_IOC_SIZE(cmd) <= sizeof(sbuf)) {
			parg = sbuf;
		} else {
			/* too big to allocate from stack */
			mbuf = kmalloc(_IOC_SIZE(cmd), GFP_KERNEL);
			if (NULL == mbuf)
				return -ENOMEM;
			parg = mbuf;
		}

		ret = -EFAULT;
		if (_IOC_DIR(cmd) & _IOC_WRITE)
			if (copy_from_user(parg, (void __user *)arg,
					   _IOC_SIZE(cmd)))
				goto out;
		break;
	}

	/* call driver */
	ret = vpif_doioctl(inode, file, cmd, (unsigned long)parg);
	if (ret == -ENOIOCTLCMD)
		ret = -EINVAL;

	/*  Copy results into user buffer  */
	switch (_IOC_DIR(cmd)) {
	case _IOC_READ:
	case (_IOC_WRITE | _IOC_READ):
		if (copy_to_user((void __user *)arg, parg, _IOC_SIZE(cmd)))
			ret = -EFAULT;
		break;
	}
out:
	if (mbuf)
		kfree(mbuf);

	dev_dbg(vpif_dev, "</vpif_ioctl>\n");
	return ret;
}

/*
 * vpif_mmap: It is used to map kernel space buffers into user spaces */
static int vpif_mmap(struct file *filep, struct vm_area_struct *vma)
{
	/* Get the channel object and file handle object */
	struct vpif_fh *fh = filep->private_data;
	struct channel_obj *channel = fh->channel;
	struct common_obj *common = &(channel->common[VPIF_VBI_INDEX]);
	int err = 0, i, found = 0;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	dev_dbg(vpif_dev, "<vpif_mmap>\n");
	for (i = 0; (i < common->numbuffers) && (0 == found); i++) {
		if (common->buffer_queue.bufs[i]->boff == offset)
			found = 1;
	}
	if (found == 0) {
		common = &(channel->common[VPIF_HBI_INDEX]);
		for (i = 0; (i < common->numbuffers) && (0 == found); i++) {
			if (common->buffer_queue.bufs[i]->boff == offset)
				found = 1;
		}
	}
	if (0 == found)
		common = &(channel->common[VPIF_VIDEO_INDEX]);

	err = videobuf_mmap_mapper(&common->buffer_queue, vma);
	dev_dbg(vpif_dev, "</vpif_mmap>\n");
	return err;
}

/* vpif_poll: It is used for select/poll system call
 */
static unsigned int vpif_poll(struct file *filep, poll_table * wait)
{
	int err = 0;
	struct vpif_fh *fh = filep->private_data;
	struct channel_obj *channel = fh->channel;
	struct common_obj *common = &(channel->common[VPIF_VIDEO_INDEX]);

	dev_dbg(vpif_dev, "<vpif_poll>");

	if (common->started)
		err = videobuf_poll_stream(filep, &common->buffer_queue, wait);

	common = &(channel->common[VPIF_VBI_INDEX]);
	if (common->started)
		err |= videobuf_poll_stream(filep, &common->buffer_queue, wait);

	if (err & POLLIN) {
		err &= (~POLLIN);
		err |= POLLOUT;
	}
	if (err & POLLRDNORM) {
		err &= (~POLLRDNORM);
		err |= POLLWRNORM;
	}

	dev_dbg(vpif_dev, "</vpif_poll>");
	return err;
}

/*
 * vpif_open: It creates object of file handle structure and stores it in
 * private_data member of filepointer */
static int vpif_open(struct inode *inode, struct file *filep)
{
	int minor = iminor(inode);
	int found = -1;
	int i = 0, err = 0;
	struct channel_obj *channel;
	struct vpif_fh *fh = NULL;

	dev_dbg(vpif_dev, "<vpif open>\n");

	/* Check for valid minor number */
	for (i = 0; i < VPIF_DISPLAY_MAX_DEVICES; i++) {
		/* Get the pointer to the channel object */
		channel = vpif_obj.dev[i];
		if (minor == channel->video_dev->minor) {
			found = i;
			break;
		}
	}
	/* If not found, return error no device */
	if (0 > found) {
		dev_err(vpif_dev, "device not found\n");
		return -ENODEV;
	}
	/* Allocate memory for the file handle object */
	fh = kmalloc(sizeof(struct vpif_fh), GFP_KERNEL);
	if (ISNULL(fh)) {
		dev_err(vpif_dev,
			"unable to allocate memory for file handle object\n");
		return -ENOMEM;
	}
	/* store pointer to fh in private_data member of filep */
	filep->private_data = fh;
	fh->channel = channel;
	fh->initialized = 0;
	/* If encoder is not initialized. initialize it */
	if (!channel->initialized) {
		fh->initialized = 1;
		channel->initialized = 1;
		/* increment module usage counter */
		/* Get the default standard and info about standard */
		err = davinci_enc_get_mode(channel->channel_id,
					   &channel->video.mode_info);
		if (err < 0)
			goto vpif_open_out;

		vpif_get_std_info(channel);
		channel->common[VPIF_VIDEO_INDEX].fmt.fmt.pix.
		    bytesperline =
		    channel->common[VPIF_VIDEO_INDEX].fmt.fmt.pix.width;
		/* Configure the default format information */
		vpif_config_format(channel);
		memset(&(channel->vpifparams), 0, sizeof(struct vpif_params));
	}
vpif_open_out:
	if (err < 0) {
		if (fh->initialized) {
			channel->initialized = 0;
		}
		filep->private_data = NULL;
		fh->initialized = 0;
		/* Free memory allocated to file handle object */
		if (!ISNULL(fh))
			kfree(fh);
		return err;
	}
	/* Increment channel usrs counter */
	channel->usrs++;
	/* Set io_allowed[VPIF_VIDEO_INDEX] member to false */
	fh->io_allowed[VPIF_VIDEO_INDEX] = 0;
	/* Set io_allowed[VPIF_VBI_INDEX] member to false */
	fh->io_allowed[VPIF_VBI_INDEX] = 0;
	fh->io_allowed[VPIF_HBI_INDEX] = 0;

	/* Initialize priority of this instance to default priority */
	fh->prio = V4L2_PRIORITY_UNSET;

	v4l2_prio_open(&channel->prio, &fh->prio);

	dev_dbg(vpif_dev, "</vpif_open>\n");
	return err;
}

static void vpif_free_vbibuffers(struct channel_obj *channel, u8 index)
{
	struct common_obj *common = NULL;
	u32 bufsize, i;
	common = &(channel->common[index]);
	if (V4L2_BUF_TYPE_SLICED_VBI_OUTPUT == common->buffer_queue.type) {
		bufsize = VPIF_SLICED_BUF_SIZE;
	} else {
		/* Calculate the size of the buffer */
		bufsize = ((common->fmt.fmt.vbi.count[0]) +
			   (common->fmt.fmt.vbi.count[1])) *
		    (common->fmt.fmt.vbi.samples_per_line);
	}
	for (i = 0; i < common->numbuffers; i++) {
		if (common->fbuffers[i]) {
			vpif_free_buffer((unsigned long)common->
					 fbuffers[i], bufsize);
		}
		common->fbuffers[i] = 0;
	}
	common->numbuffers = 0;
}

static void vpif_free_allbuffers(struct channel_obj *channel)
{
	int i;
	struct common_obj *common = &(channel->common[VPIF_VIDEO_INDEX]);
	u32 start = config_params.numbuffers[channel->channel_id];
	u32 end = common->numbuffers;
	u32 bufsize = config_params.channel_bufsize[channel->channel_id];
	for (i = start; i < end; i++) {
		if (common->fbuffers[i]) {
			vpif_free_buffer((unsigned long)common->
					 fbuffers[i], bufsize);
		}
		common->fbuffers[i] = 0;
	}
}

/*
 * vpif_release: This function deletes buffer queue, frees the buffers and
 * the vpif file handle */
static int vpif_release(struct inode *inode, struct file *filep)
{
	/* Get the channel object and file handle object */
	struct vpif_fh *fh = filep->private_data;
	struct channel_obj *channel = fh->channel;
	struct common_obj *common = &(channel->common[VPIF_VIDEO_INDEX]);
	int i;

	dev_dbg(vpif_dev, "<vpif_release>\n");
	/* If this is doing IO and other channels are not closed */
	if ((channel->usrs != 1) && fh->io_allowed[VPIF_VIDEO_INDEX]) {
		dev_err(vpif_dev, "Close other instances\n");
		return -EAGAIN;
	}
	/* Get the lock on channel object */
	down_interruptible(&common->lock);
	/* if this instance is doing IO */
	if (fh->io_allowed[VPIF_VIDEO_INDEX]) {
		/* Reset io_usrs member of channel object */
		common->io_usrs = 0;
		/* Disable channel/vbi as per its device type 
		   and channel id */
		if (VPIF_CHANNEL2_VIDEO == channel->channel_id) {
			enable_channel2(0);
			channel2_intr_enable(0);
		}
		if ((VPIF_CHANNEL3_VIDEO == channel->channel_id) ||
		    (2 == common->started)) {
			enable_channel3(0);
			channel3_intr_enable(0);
		}
		common->started = 0;
		/* Free buffers allocated */
		videobuf_queue_cancel(&common->buffer_queue);
		vpif_free_allbuffers(channel);
		common->numbuffers =
		    config_params.numbuffers[channel->channel_id];

	}
	/* unlock semaphore on channel object */
	up(&common->lock);

	for (i = VPIF_VBI_INDEX; i <= VPIF_HBI_INDEX; i++) {
		down_interruptible(&(channel->common[i].lock));
		if (fh->io_allowed[i]) {
			/* Reset io_usrs member of channel object */
			channel->common[i].io_usrs = 0;
			/* Disable channel/vbi as per its device type 
			   and channel id */
			if (VPIF_CHANNEL2_VIDEO == channel->channel_id) {
				channel2_raw_enable(0, i);
			} else {
				channel3_raw_enable(0, i);
			}
			channel->common[i].started = 0;
			/* Free buffers allocated */
			videobuf_queue_cancel
			    (&channel->common[VPIF_VBI_INDEX].buffer_queue);
			vpif_free_vbibuffers(channel, i);
			common->numbuffers = 0;
		}
		up(&(channel->common[i].lock));
	}
	/* Decrement channel usrs counter */
	channel->usrs--;
	/* If this file handle has initialize encoder device, reset it */
	if (fh->initialized) {
		channel->initialized = 0;
	}
	/* Close the priority */
	v4l2_prio_close(&channel->prio, &fh->prio);
	filep->private_data = NULL;
	fh->initialized = 0;
	/* Free memory allocated to file handle object */
	if (!ISNULL(fh))
		kfree(fh);
	dev_dbg(vpif_dev, "</vpif_release>\n");
	return 0;
}

static void vpif_platform_release(struct device
				  *device)
{
	/* This is called when the reference count goes to zero */
}

static struct file_operations vpif_fops = {
	.owner = THIS_MODULE,
	.open = vpif_open,
	.release = vpif_release,
	.ioctl = vpif_ioctl,
	.mmap = vpif_mmap,
	.poll = vpif_poll
};
static struct video_device vpif_video_template = {
	.name = "vpif",
	.type = VID_TYPE_CAPTURE,
	.hardware = 0,
	.fops = &vpif_fops,
	.minor = -1
};

/*
 * vpif_probe: This function creates device entries by register itself to the
 * V4L2 driver and initializes fields of each channel objects */
static __init int vpif_probe(struct device *device)
{
	int i, j = 0, k, err = 0;
	struct video_device *vfd = NULL;
	struct channel_obj *channel = NULL;
	struct common_obj *common = NULL;
	vpif_dev = device;
	dev_dbg(vpif_dev, "<vpif_probe>\n");

	for (i = 0; i < VPIF_DISPLAY_MAX_DEVICES; i++) {
		/* Get the pointer to the channel object */
		channel = vpif_obj.dev[i];
		/* Allocate memory for video device */
		vfd = video_device_alloc();
		if (ISNULL(vfd)) {
			for (j = 0; j < i; j++) {
				video_device_release
				    (vpif_obj.dev[j]->video_dev);
			}
			return -ENOMEM;
		}

		/* Initialize field of video device */
		*vfd = vpif_video_template;
		vfd->dev = device;
		vfd->release = video_device_release;
		snprintf(vfd->name, sizeof(vfd->name),
			 "DaVinciHD_VPIFDisplay_DRIVER_V%d.%d.%d",
			 (VPIF_DISPLAY_VERSION_CODE >> 16)
			 & 0xff,
			 (VPIF_DISPLAY_VERSION_CODE >> 8) &
			 0xff, (VPIF_DISPLAY_VERSION_CODE) & 0xff);
		/* Set video_dev to the video device */
		channel->video_dev = vfd;
	}

	for (j = 0; j < VPIF_DISPLAY_MAX_DEVICES; j++) {
		channel = vpif_obj.dev[j];
		/* Initialize field of the channel objects */
		channel->usrs = 0;
		for (k = 0; k < VPIF_NUMOBJECTS; k++) {
			channel->common[k].numbuffers = 0;
			common = &(channel->common[k]);
			common->io_usrs = 0;
			common->started = 0;
			common->irqlock = SPIN_LOCK_UNLOCKED;
			init_MUTEX(&common->lock);
			common->numbuffers = 0;
			common->set_addr = NULL;
			common->ytop_off = common->ybtm_off = 0;
			common->ctop_off = common->cbtm_off = 0;
			common->curFrm = common->nextFrm = NULL;
			memset(&common->fmt, 0, sizeof(struct v4l2_format));

		}
		channel->initialized = 0;
		channel->channel_id = j;
		if (j < 2)
			channel->common[VPIF_VIDEO_INDEX].numbuffers =
			    config_params.numbuffers[channel->channel_id];
		else
			channel->common[VPIF_VIDEO_INDEX].numbuffers = 0;

		memset(&(channel->vpifparams), 0, sizeof(struct vpif_params));

		/* Initialize prio member of channel object */
		v4l2_prio_init(&channel->prio);

		/* Initialize the work structure */
		INIT_WORK(&vbi_work[channel->channel_id],
			  (void (*)(void *))vbi_work_handler, (void *)channel);

		channel->common[VPIF_VIDEO_INDEX].fmt.type =
		    V4L2_BUF_TYPE_VIDEO_OUTPUT;
		channel->common[VPIF_VBI_INDEX].fmt.type =
		    V4L2_BUF_TYPE_SLICED_VBI_OUTPUT;
		channel->common[VPIF_HBI_INDEX].fmt.type =
		    V4L2_BUF_TYPE_HBI_OUTPUT;
		/* register video device */
		dev_dbg(vpif_dev, "trying to register vpif device.\n");
		dev_dbg(vpif_dev,
			"channel=%x,channel->video_dev=%x\n",
			(int)channel, (int)&channel->video_dev);

		err =
		    video_register_device(channel->
					  video_dev,
					  VFL_TYPE_GRABBER, vpif_nr[j]);
		if (err < 0)
			goto probe_out;
	}
	return 0;

probe_out:
	for (k = 0; k < j; k++) {
		/* Get the pointer to the channel object */
		channel = vpif_obj.dev[k];
		/* Unregister video device */
		video_unregister_device(channel->video_dev);
		/* Release video device */
		video_device_release(channel->video_dev);
		channel->video_dev = NULL;
	}
	dev_dbg(vpif_dev, "</vpif_probe>\n");
	return err;
}

/*
 * vpif_remove: It un-register channels from V4L2 driver */
static int vpif_remove(struct device *device)
{
	int i;
	struct channel_obj *channel;
	dev_dbg(vpif_dev, "<vpif_remove>\n");
	/* un-register device */
	for (i = 0; i < VPIF_DISPLAY_MAX_DEVICES; i++) {
		/* Get the pointer to the channel object */
		channel = vpif_obj.dev[i];
		/* Unregister video device */
		video_unregister_device(channel->video_dev);

		channel->video_dev = NULL;
	}
	dev_dbg(vpif_dev, "</vpif_remove>\n");
	return 0;
}

static struct device_driver vpif_driver = {
	.name = "vpif display",
	.bus = &platform_bus_type,
	.probe = vpif_probe,
	.remove = vpif_remove,
};
static struct platform_device _vpif_device = {
	.name = "vpif display",
	.id = 1,
	.dev = {
		.release = vpif_platform_release,
		}
};

/*
 * vpif_init: This function registers device and driver to the kernel,
* requests irq handler and allocates memory for channel objects */
static __init int vpif_init(void)
{
	int err = 0, i, j;
	int free_channel_objects_index;
	int free_irq_no_index;
	int free_buffer_channel_index;
	int free_buffer_index;
	u32 addr;
	int size;

	/* Default number of buffers should be 3 */
	if ((channel2_numbuffers > 0) &&
	    (channel2_numbuffers < config_params.min_numbuffers))
		channel2_numbuffers = config_params.min_numbuffers;
	if ((channel3_numbuffers > 0) &&
	    (channel3_numbuffers < config_params.min_numbuffers))
		channel3_numbuffers = config_params.min_numbuffers;

	/* Set buffer size to min buffers size if invalid buffer size is
	 * given */
	if (channel2_bufsize < config_params.min_bufsize[VPIF_CHANNEL2_VIDEO])
		channel2_bufsize =
		    config_params.min_bufsize[VPIF_CHANNEL2_VIDEO];
	if (channel3_bufsize < config_params.min_bufsize[VPIF_CHANNEL3_VIDEO])
		channel3_bufsize =
		    config_params.min_bufsize[VPIF_CHANNEL3_VIDEO];

	config_params.numbuffers[VPIF_CHANNEL2_VIDEO] = channel2_numbuffers;

	if (channel2_numbuffers) {
		config_params.channel_bufsize[VPIF_CHANNEL2_VIDEO]
		    = channel2_bufsize;
	}
	config_params.numbuffers[VPIF_CHANNEL3_VIDEO] = channel3_numbuffers;

	if (channel3_numbuffers) {
		config_params.channel_bufsize[VPIF_CHANNEL3_VIDEO]
		    = channel3_bufsize;
	}

	/* Allocate memory for six channel objects */
	for (i = 0; i < VPIF_DISPLAY_MAX_DEVICES; i++) {
		vpif_obj.dev[i] =
		    kmalloc(sizeof(struct channel_obj), GFP_KERNEL);
		/* If memory allocation fails, return error */
		if (!vpif_obj.dev[i]) {
			free_channel_objects_index = i;
			err = -ENOMEM;
			goto vpif_init_free_channel_objects;
		}
	}
	free_channel_objects_index = VPIF_DISPLAY_MAX_DEVICES;

	/* Allocate memory for buffers */
	for (i = 0; i < VPIF_DISPLAY_NUM_CHANNELS; i++) {
		size = config_params.channel_bufsize[i];
		for (j = 0; j < config_params.numbuffers[i]; j++) {
			addr = vpif_alloc_buffer(size);
			if (!addr) {
				free_buffer_channel_index = i;
				free_buffer_index = j;
				err = -ENOMEM;
				goto vpif_init_free_buffers;
			}
			vpif_obj.dev[i]->common[VPIF_VIDEO_INDEX].
			    fbuffers[j] = (u8 *) addr;
		}
	}
	free_buffer_channel_index = VPIF_DISPLAY_NUM_CHANNELS;
	free_buffer_index = config_params.numbuffers[i - 1];

	/* Create the workqueue */
	vbi_workqueue = create_singlethread_workqueue("vbi");
	if (!vbi_workqueue) {
		err = -ENOMEM;
		goto vpif_init_free_buffers;
	}

	/* Register driver to the kernel */
	err = driver_register(&vpif_driver);
	if (0 != err) {
		goto vpif_init_free_buffers;
	}
	/* register device as a platform device to the kernel */
	err = platform_device_register(&_vpif_device);
	if (0 != err) {
		goto vpif_init_unregister_vpif_driver;
	}
	for (j = 0; j < VPIF_DISPLAY_NUM_CHANNELS; j++) {
		err =
		    request_irq(vpif_get_irq_number(j + 2),
				vpif_channel_isr,
				SA_INTERRUPT, "DaVinciHD_Display",
				(void *)(&(vpif_obj.dev[j]->channel_id)));
		if (0 != err) {
			free_irq_no_index = j;
			goto vpif_init_free_irq;

		}
	}
	free_irq_no_index = VPIF_DISPLAY_NUM_CHANNELS;

	/* Set pinmux settings */
	set_vpif_pinmux();

	return 0;
vpif_init_free_irq:
	for (j = 0; j < free_irq_no_index; j++) {
		free_irq(vpif_get_irq_number(j + 2),
			 (void *)(&(vpif_obj.dev[j]->channel_id)));
	}
	platform_device_unregister(&_vpif_device);

vpif_init_unregister_vpif_driver:
	driver_unregister(&vpif_driver);

vpif_init_free_buffers:
	for (i = 0; i < free_buffer_channel_index; i++) {
		for (j = 0; j < config_params.numbuffers[i]; j++) {
			addr = (unsigned long)vpif_obj.dev[i]->
			    common[VPIF_VIDEO_INDEX].fbuffers[j];
			if (addr) {
				vpif_free_buffer(addr,
						 config_params.
						 channel_bufsize[i]);
				vpif_obj.dev[i]->common[VPIF_VIDEO_INDEX].
				    fbuffers[j] = 0;
			}
		}
	}
	for (j = 0; j < free_buffer_index; j++) {
		addr = (unsigned long)vpif_obj.
		    dev[free_buffer_channel_index]->
		    common[VPIF_VIDEO_INDEX].fbuffers[j];
		if (addr) {
			vpif_free_buffer(addr,
					 config_params.channel_bufsize[i]);
			vpif_obj.dev[free_buffer_channel_index]->
			    common[VPIF_VIDEO_INDEX].fbuffers[j] = 0;
		}

	}

vpif_init_free_channel_objects:
	for (j = 0; j < free_channel_objects_index; j++) {
		if (vpif_obj.dev[i]) {
			kfree(vpif_obj.dev[j]);
			vpif_obj.dev[i] = NULL;
		}
	}
	return err;
}

/* vpif_cleanup: This function un-registers device and driver to the kernel,
 * frees requested irq handler and de-allocates memory allocated for channel
 * objects.
 * */
static void vpif_cleanup(void)
{
	int i = 0, j = 0;
	u32 addr;

	for (i = 0; i < VPIF_DISPLAY_NUM_CHANNELS; i++) {
		free_irq(vpif_get_irq_number(i + 2),
			 (void *)(&(vpif_obj.dev[i]->channel_id)));
	}
	/* Flush and destroy the workqueue */
	flush_workqueue(vbi_workqueue);
	destroy_workqueue(vbi_workqueue);

	platform_device_unregister(&_vpif_device);
	driver_unregister(&vpif_driver);
	for (i = 0; i < VPIF_DISPLAY_NUM_CHANNELS; i++) {
		for (j = 0; j < config_params.numbuffers[i]; j++) {
			addr = (unsigned long)vpif_obj.dev[i]->
			    common[VPIF_VIDEO_INDEX].fbuffers[j];
			if (addr) {
				vpif_free_buffer(addr,
						 config_params.
						 channel_bufsize[i]);
				vpif_obj.dev[i]->common[VPIF_VIDEO_INDEX].
				    fbuffers[j] = 0;
			}
		}
	}
	for (i = 0; i < VPIF_DISPLAY_MAX_DEVICES; i++) {
		if (vpif_obj.dev[i]) {
			kfree(vpif_obj.dev[i]);
			vpif_obj.dev[i] = NULL;
		}
	}
}

MODULE_LICENSE("GPL");
/* Function for module initialization and cleanup */
module_init(vpif_init);
module_exit(vpif_cleanup);
