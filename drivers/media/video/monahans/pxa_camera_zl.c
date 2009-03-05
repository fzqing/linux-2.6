/*
 * pxa_camera_zl - main file for camera driver
 *
 * Copyright (C) 2005, Intel Corporation.
 * Copyright (C) 2006, Marvell International Ltd.
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/pagemap.h>
#include <linux/interrupt.h>
#include <linux/videodev.h>
#include <linux/pci.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/cpufreq.h>
#include <linux/list.h>
#include <linux/types.h>
#include <linux/kernel.h>

#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/hardware.h>
#include <asm/dma.h>
#include <asm/irq.h>
#include <asm/bug.h>

#include <asm/arch/irqs.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/cpu-freq-voltage-mhn.h>
#include <asm/arch/mhn_pmic.h>

#include <linux/mm.h>
#include <linux/videodev.h>
#include <linux/videodev2.h>
#include <linux/pxa_camera_zl.h>

#include "ov2620.h"
#include "ov2630.h"
#include "ov7660.h"
#include "camera.h"
#include "ci.h"

int ov7660_detected = 0;
int ov2620_detected = 0;
int ov2630_detected = 0;

/* #define HW_IP_OV7660 */

#define        PXA_CAMERA_VERSION    KERNEL_VERSION(0, 0, 1)

/*
 * main camera driver macros and data
 */

/* This mask just enable EOFX interrupt */
#define    INT_MASK    CAMERA_INTMASK_FIFO_OVERRUN |	\
	CAMERA_INTMASK_END_OF_FRAME |			\
	CAMERA_INTMASK_START_OF_FRAME |			\
	CAMERA_INTMASK_CI_DISABLE_DONE |		\
	CAMERA_INTMASK_CI_QUICK_DISABLE |		\
	CAMERA_INTMASK_PARITY_ERROR |			\
	CAMERA_INTMASK_END_OF_LINE |			\
	CAMERA_INTMASK_FIFO_EMPTY  |			\
	CAMERA_INTMASK_TIME_OUT  |			\
	CAMERA_INTMASK_FIFO3_UNDERRUN |			\
	CAMERA_INTMASK_BRANCH_STATUS |			\
	/*CAMERA_INTMASK_ENF_OF_FRAME_TRANSFER |*/	\
	CAMERA_INTMASK_DMA_CHANNEL0_STOP |		\
	CAMERA_INTMASK_DMA_CHANNEL1_STOP |		\
	CAMERA_INTMASK_DMA_CHANNEL2_STOP |		\
	CAMERA_INTMASK_DMA_CHANNEL3_STOP

#define DMA_DESCRIPTOR_SIZE (sizeof(struct ci_dma_descriptor))

#define SENSOR_NUMBER  4

#define SET_OV2620_SENSOR(cam_ctx)  do {			\
	cam_ctx->sensor_type = CAMERA_TYPE_OMNIVISION_2620;	\
	cam_ctx->camera_functions->init = ov2620_init;		\
	cam_ctx->camera_functions->deinit = ov2620_deinit;	\
	cam_ctx->camera_functions->set_capture_format =		\
			ov2620_set_capture_format;		\
	cam_ctx->camera_functions->start_capture =		\
			ov2620_start_capture;			\
	cam_ctx->camera_functions->stop_capture =		\
			ov2620_stop_capture;			\
	cam_ctx->camera_functions->sleep = ov2620_sleep;	\
	cam_ctx->camera_functions->wakeup = ov2620_wake;	\
	cam_ctx->camera_functions->read_8bit =			\
			ov2620_read_8bit;			\
	cam_ctx->camera_functions->write_8bit =			\
			ov2620_write_8bit;			\
	cam_ctx->camera_functions->read_16bit = NULL;		\
	cam_ctx->camera_functions->write_16bit = NULL;		\
	cam_ctx->camera_functions->read_32bit = NULL;		\
	cam_ctx->camera_functions->write_32bit = NULL;		\
	cam_ctx->camera_functions->set_power_mode =		\
			ov2620_set_power_mode;			\
	cam_ctx->camera_functions->set_contrast =		\
			ov2620_set_contrast;			\
	cam_ctx->camera_functions->set_whitebalance =		\
			ov2620_set_white_balance;		\
	cam_ctx->camera_functions->set_exposure =		\
			ov2620_set_exposure;			\
	cam_ctx->camera_functions->set_zoom = NULL;		\
} while (0)

#define SET_OV2630_SENSOR(cam_ctx)  do {			\
	cam_ctx->sensor_type = CAMERA_TYPE_OMNIVISION_2630;	\
	cam_ctx->camera_functions->init = ov2630_init;		\
	cam_ctx->camera_functions->deinit = ov2630_deinit;	\
	cam_ctx->camera_functions->set_capture_format =		\
			ov2630_set_capture_format;		\
	cam_ctx->camera_functions->start_capture =		\
			ov2630_start_capture;			\
	cam_ctx->camera_functions->stop_capture =		\
			ov2630_stop_capture;			\
	cam_ctx->camera_functions->sleep = ov2630_sleep;	\
	cam_ctx->camera_functions->wakeup = ov2630_wake;	\
	cam_ctx->camera_functions->read_8bit =			\
			ov2630_read_8bit;			\
	cam_ctx->camera_functions->write_8bit =			\
			ov2630_write_8bit;			\
	cam_ctx->camera_functions->read_16bit = NULL;		\
	cam_ctx->camera_functions->write_16bit = NULL;		\
	cam_ctx->camera_functions->read_32bit = NULL;		\
	cam_ctx->camera_functions->write_32bit = NULL;		\
	cam_ctx->camera_functions->set_power_mode =		\
			ov2630_set_power_mode;			\
	cam_ctx->camera_functions->set_contrast =		\
			ov2630_set_contrast;			\
	cam_ctx->camera_functions->set_whitebalance =		\
			ov2630_set_white_balance;		\
	cam_ctx->camera_functions->set_exposure =		\
			ov2630_set_exposure;			\
	cam_ctx->camera_functions->set_zoom = NULL;		\
} while (0)

#define SET_OV7660_SENSOR(cam_ctx)  do {			\
	cam_ctx->sensor_type = CAMERA_TYPE_OMNIVISION_7660;	\
	cam_ctx->camera_functions->init = ov7660_init;		\
	cam_ctx->camera_functions->deinit = ov7660_deinit;	\
	cam_ctx->camera_functions->set_capture_format =		\
			ov7660_set_capture_format;		\
	cam_ctx->camera_functions->start_capture =		\
			ov7660_start_capture;			\
	cam_ctx->camera_functions->stop_capture =		\
			ov7660_stop_capture;			\
	cam_ctx->camera_functions->sleep = ov7660_sleep;	\
	cam_ctx->camera_functions->wakeup = ov7660_wake;	\
	cam_ctx->camera_functions->read_8bit =			\
			ov7660_read_8bit;			\
	cam_ctx->camera_functions->write_8bit =			\
			ov7660_write_8bit;			\
	cam_ctx->camera_functions->read_16bit = NULL;		\
	cam_ctx->camera_functions->write_16bit = NULL;		\
	cam_ctx->camera_functions->read_32bit = NULL;		\
	cam_ctx->camera_functions->write_32bit = NULL;		\
	cam_ctx->camera_functions->set_power_mode =		\
			ov7660_set_power_mode;			\
	cam_ctx->camera_functions->set_contrast =		\
			ov7660_set_contrast;			\
	cam_ctx->camera_functions->set_whitebalance =		\
			ov7660_set_white_balance;		\
	cam_ctx->camera_functions->set_exposure =		\
			ov7660_set_exposure;			\
	cam_ctx->camera_functions->set_zoom = NULL;		\
} while (0)

#define SET_DEFAULT_SENSOR(cam_ctx)	SET_OV7660_SENSOR(cam_ctx)

/* default value */
#define WIDTH_DEFT		176
#define HEIGHT_DEFT		144
#define FRAMERATE_DEFT		0

/* sensor capability*/
#define OV_2620_MAX_WIDTH	1600
#define OV_2620_MAX_HEIGHT	1200
#define OV_2620_MIN_WIDTH	2
#define OV_2620_MIN_HEIGHT	2

#define OV_2630_MAX_WIDTH	1600
#define OV_2630_MAX_HEIGHT	1200
#define OV_2630_MIN_WIDTH	2
#define OV_2630_MIN_HEIGHT	2

#define OV_7660_MAX_WIDTH	640
#define OV_7660_MAX_HEIGHT	480
#define OV_7660_MIN_WIDTH	88
#define OV_7660_MIN_HEIGHT	72

extern int pxa_i2c_write(u8 slaveaddr, const u8 * bytesbuf, u32 bytescount);

static int pxa_camera_minor = 0;
static p_camera_context_t g_camera_context;

struct pxa_camera_driver_status_t {
	int		still_buf_ready;
	int		video_buf_ready;
	int		still_buf_submited;
	int		video_buf_submited;
	int		still_capture_started;
	int		video_capture_started;
	struct		list_head still_buf_head;
	struct		list_head video_buf_head;
	struct		list_head still_report_head;
	struct		list_head video_report_head;
	unsigned int	still_timeperframe_numerator;
	unsigned int	video_timeperframe_numerator;
	unsigned int	still_timeperframe_denominator;
	unsigned int	video_timeperframe_denominator;

	int		capture_mode;
	int		*p_buf_ready;
	int		*p_buf_submited;
	int		*p_capture_started;
	struct		list_head *p_buf_head;
	struct		list_head *p_report_head;
	unsigned int	*p_timeperframe_numerator;
	unsigned int	*p_timeperframe_denominator;
	wait_queue_head_t camera_wait_q;
	int		task_waiting;
	int		driver_opened;

	int		re_init_needed;
	int		re_bufprepare_needed;
	int		re_bufsubmit_needed;
	int		re_formatset_needed;

	int		i2c_inited;
	int		suspended;
};
static struct pxa_camera_driver_status_t g_camdrv_status;

static int pxa_camera_open(struct inode *inode, struct file *file);
static int pxa_camera_close(struct inode *inode, struct file *file);
static int pxa_camera_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long param);
static ssize_t pxa_camera_read(struct file *file, char __user * buf,
		size_t count, loff_t * ppos);
static int pxa_camera_mmap(struct file *file, struct vm_area_struct *vma);
static unsigned int pxa_camera_poll(struct file *file, poll_table * wait);
static void pxa_camera_release(struct video_device *dev);

/* Internal function */
static int pxa_camera_ioctl_streamon(struct pxa_camera_driver_status_t
		*p_camdrv_status);
static void pxa_camera_ioctl_streamoff(struct pxa_camera_driver_status_t
		*p_camdrv_status);
static int pxa_camera_reset(struct pxa_camera_driver_status_t *p_camdrv_status);
static int pxa_camera_get_framerate(struct pxa_camera_driver_status_t
		*p_camdrv_status);

static struct file_operations pxa_camera_fops = {
	.owner		= THIS_MODULE,
	.open		= pxa_camera_open,
	.release	= pxa_camera_close,
	.ioctl		= pxa_camera_ioctl,
	.read		= pxa_camera_read,
	.mmap		= pxa_camera_mmap,
	.poll		= pxa_camera_poll,
	.llseek		= no_llseek,
};

static struct video_device vd = {
	.name		= "PXA Camera",
	.type		= VID_TYPE_CAPTURE,
	.hardware	= 50,		/* FIXME */
	.fops		= &pxa_camera_fops,
	.release	= pxa_camera_release,
	.minor		= -1,
};

struct pxa_camera_format {
	char *name;		/* format description */
	int palette;		/* video4linux 1      */
	int fourcc;		/* video4linux 2      */
	int depth;		/* bit/pixel          */
};

static const struct pxa_camera_format pxa_camera_formats[] = {
	{
		.name		= "RGB565",
		.palette	= VIDEO_PALETTE_RGB565,
		.fourcc		= V4L2_PIX_FMT_RGB565X,
		.depth		= 16,
	}, {
#if defined(CONFIG_PXA310)
		.name 		= "YCbCr4:2:0",
		.palette 	= VIDEO_PALETTE_YUV420,
		.fourcc 	= V4L2_PIX_FMT_YUV420,
		.depth 	= 16,
	}, {
#endif
		.name 		= "YCbCr4:2:2(Planar)",
		.palette 	= VIDEO_PALETTE_YUV422P,
		.fourcc 	= V4L2_PIX_FMT_YUV422P,
		.depth 		= 16,
	}, {
		.name 		= "RawRGGB 8 bit",
		.palette 	= VIDEO_PALETTE_RGGB8,
		.fourcc 	= V4L2_PIX_FMT_SRGGB8,
		.depth 		= 8,
	}, {
		.name 		= "RawRGGB 10 bit",
		.palette 	= VIDEO_PALETTE_RGGB10,
		.fourcc 	= V4L2_PIX_FMT_SRGGB10,
		.depth 		= 16,
	}
};

const unsigned int PXA_CAMERA_FORMATS = ARRAY_SIZE(pxa_camera_formats);

/* look up table: sensor input format <--> qci output format*/
struct pxa_camera_format_t {
	unsigned int input_format;
	unsigned int output_format;
};

static struct pxa_camera_format_t ov7660_input_format_lut[] = {
	{
		.input_format	= CAMERA_IMAGE_FORMAT_RGB565,
		.output_format	= CAMERA_IMAGE_FORMAT_RGB565,
	},
#ifdef HW_IP_OV7660
	{
		.input_format	= CAMERA_IMAGE_FORMAT_RAW8,
		.output_format	= CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR,
	}, {
		.input_format	= CAMERA_IMAGE_FORMAT_RAW8,
		.output_format	= CAMERA_IMAGE_FORMAT_RGB888_PACKED,
	},
#else
	{
		.input_format	= CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR,
		.output_format	= CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR,
	},
#endif
	{
		.input_format	= CAMERA_IMAGE_FORMAT_RAW8,
		.output_format	= CAMERA_IMAGE_FORMAT_RAW8,
	}, {
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10,
		.output_format	= CAMERA_IMAGE_FORMAT_RAW10,
	}, {
#if defined(CONFIG_PXA310)
		.input_format	= CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR,
		.output_format	= CAMERA_IMAGE_FORMAT_YCBCR420_PLANAR,
	}, {
#endif
		.input_format	= CAMERA_IMAGE_FORMAT_MAX + 1,
		.output_format	= CAMERA_IMAGE_FORMAT_MAX + 1
	},
};

static struct pxa_camera_format_t ov2620_input_format_lut[] = {
	{
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10,
		.output_format	= CAMERA_IMAGE_FORMAT_RAW10,
	}, {
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10,
		.output_format	= CAMERA_IMAGE_FORMAT_RAW8,
	},
#if 0				/*will support in MH B0 */
	{CAMERA_IMAGE_FORMAT_RAW10, CAMERA_IMAGE_FORMAT_RGB565},
	{CAMERA_IMAGE_FORMAT_RAW10, CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR},
#endif
	{
		.input_format	= CAMERA_IMAGE_FORMAT_MAX + 1,
		.output_format	= CAMERA_IMAGE_FORMAT_MAX + 1
	},
};

static struct pxa_camera_format_t ov2630_input_format_lut[] = {
	{
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10,
		.output_format	= CAMERA_IMAGE_FORMAT_RAW10,
	}, {
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10,
		.output_format	= CAMERA_IMAGE_FORMAT_RAW8,
	}, {
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10,
		.output_format	= CAMERA_IMAGE_FORMAT_RGB888_PACKED,
	}, {
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10,
		.output_format	= CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR,
	}, {
#if defined(CONFIG_PXA310)
		.input_format	= CAMERA_IMAGE_FORMAT_RAW10,
		.output_format	= CAMERA_IMAGE_FORMAT_YCBCR420_PLANAR,
	}, {
#endif
		.input_format	= CAMERA_IMAGE_FORMAT_MAX + 1,
		.output_format	= CAMERA_IMAGE_FORMAT_MAX + 1,
	},
};

/*
 * buffer list
 */

struct buf_node {
	unsigned int io_type;           /* camera IO methods */
	struct list_head buf_head;	/* For buffer list */
	struct list_head report_head;	/* For report buffer list */
	void *vaddr;		/* vmap() return virtual address */
	struct page **pages;	/* physical pages */
	int page_num;		/* physical pages count */
	int buf_id;		/* buffer id to let driver access */
	int buf_index;		/* buffer index */
	int size;		/* buffer size */
	void *dma_desc_vaddr;	/* dma description virtual address */
	dma_addr_t dma_desc_paddr;	/* dma description physical address */
	int dma_desc_size;	/* dma description size */
	void *Y_vaddr;		/* Y virtual address */
	void *Cb_vaddr;		/* Cb virtual address */
	void *Cr_vaddr;		/* Cr virtual address */
	int fifo0_size;		/* fifo0 data transfer size */
	int fifo1_size;		/* fifo1 data transfer size */
	int fifo2_size;		/* fifo2 data transfer size */
};

static struct semaphore buf_list_sem;
static spinlock_t report_list_lock;	/* Spin lock for report_list */
static spinlock_t cam_queue_lock;	/* Spin lock for queue */

/* page cache */

#define        MAX_PAGE_CACHE         256
struct page_cache_head {
	struct list_head page_list;
	int page_count;
	spinlock_t lock;
};

static struct page_cache_head pc_head;

static unsigned int camera_format_from_v4l2(__u32 fourcc)
{
	return (fourcc == V4L2_PIX_FMT_YUV422P) ?
			CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR :
#if defined(CONFIG_PXA310)
		(fourcc == V4L2_PIX_FMT_YUV420) ?
			CAMERA_IMAGE_FORMAT_YCBCR420_PLANAR:
#endif
		(fourcc == V4L2_PIX_FMT_SRGGB8) ?
			CAMERA_IMAGE_FORMAT_RAW8 :
		(fourcc == V4L2_PIX_FMT_SRGGB10) ?
			CAMERA_IMAGE_FORMAT_RAW10 :
		(fourcc == V4L2_PIX_FMT_RGB565X) ?
			CAMERA_IMAGE_FORMAT_RGB565 :
		(fourcc == V4L2_PIX_FMT_RGB24) ?
			CAMERA_IMAGE_FORMAT_RGB888_PACKED :
			CAMERA_IMAGE_FORMAT_MAX + 1;
}

static __u32 camera_fromat_to_v4l2(unsigned int format)
{
	return (format == CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR) ?
			V4L2_PIX_FMT_YUV422P :
#if defined(CONFIG_PXA310)
		(format == CAMERA_IMAGE_FORMAT_YCBCR420_PLANAR) ?
			V4L2_PIX_FMT_YUV420 :
#endif
		(format == CAMERA_IMAGE_FORMAT_RAW8) ?
			V4L2_PIX_FMT_SRGGB8 :
		(format == CAMERA_IMAGE_FORMAT_RAW10) ?
			V4L2_PIX_FMT_SRGGB10 :
		(format == CAMERA_IMAGE_FORMAT_RGB565) ?
			V4L2_PIX_FMT_RGB565X : 0xffffffff;
}

static unsigned int camera_default_input(unsigned int output_format)
{
	struct pxa_camera_format_t *format_lut;
	p_camera_context_t cam_ctx = g_camera_context;

	if (CAMERA_TYPE_OMNIVISION_2620 == cam_ctx->sensor_type) {
		format_lut = ov2620_input_format_lut;
	} else if (CAMERA_TYPE_OMNIVISION_2630 == cam_ctx->sensor_type) {
		format_lut = ov2630_input_format_lut;
	} else {
		format_lut = ov7660_input_format_lut;
	}

	for (; format_lut->output_format != CAMERA_IMAGE_FORMAT_MAX + 1;
			format_lut++) {
		if (format_lut->output_format == output_format) {
			return format_lut->input_format;
		}
	}

	return CAMERA_IMAGE_FORMAT_MAX + 1;
}

static struct buf_node *camera_get_buffer_from_id(int buf_id)
{
	struct list_head *pos;
	struct buf_node *buf_node;

	list_for_each(pos, g_camdrv_status.p_buf_head) {
		buf_node = list_entry(pos, struct buf_node, buf_head);
		if (buf_node->buf_id == buf_id)
			goto found;
	}
	return NULL;

found:
	return buf_node;
}

static struct buf_node *camera_get_buffer_from_index(int buf_index)
{
	struct list_head *pos;
	struct buf_node *buf_node;

	list_for_each(pos, g_camdrv_status.p_buf_head) {
		buf_node = list_entry(pos, struct buf_node, buf_head);
		if (buf_node->buf_index == buf_index)
			goto found;
	}
	return NULL;

found:
	return buf_node;
}

static int camera_get_buffer_num(void)
{
	struct list_head *pos;
	int buf_num = 0;

	list_for_each(pos, g_camdrv_status.p_buf_head) {
		buf_num++;
	}
	return buf_num;
}

static ssize_t camera_get_buffer_size(void)
{
	struct buf_node *buf_node;
	ssize_t page_num;

	if (!list_empty(g_camdrv_status.p_buf_head)) {
		buf_node =
			list_entry(g_camdrv_status.p_buf_head->next,
					struct buf_node, buf_head);
		page_num = buf_node->page_num;
	} else {
		page_num = 0;
	}

	return page_num * PAGE_SIZE;
}

static struct page *camera_alloc_page(void)
{
	unsigned long flags;
	struct page *page = NULL;

	if (!list_empty(&pc_head.page_list)) {
		spin_lock_irqsave(&pc_head.lock, flags);
		page = list_entry(pc_head.page_list.next, struct page, lru);
		list_del(&page->lru);
		pc_head.page_count--;
		spin_unlock_irqrestore(&pc_head.lock, flags);
	} else {
		page = alloc_page(GFP_KERNEL);
	}

	return page;
}

/* camera_free_page()
 *
 * Free page.
 * Param:
 *    page:    the page will be freed
 *    limit:
 *        0: The page will be added to camera page cache list.
 *           This will be very useful when app change capture mode
 *           and capture resolution dynamically. We needn't free all
 *           of the old pages and alloc new pages for new catpure
 *           mode/resolution. Just need alloc/free the delta pages.
 *        1: If the number of camera page cache list is lager than
 *           MAX_PAGE_CACHE, the page will be free using __free_page.
 *           Else the page will be added to page cache list.
 *
 */
static void camera_free_page(struct page *page, int limit)
{
	unsigned long flags;

	if (0 == limit) {
		spin_lock_irqsave(&pc_head.lock, flags);
		list_add_tail(&page->lru, &pc_head.page_list);
		pc_head.page_count++;
		spin_unlock_irqrestore(&pc_head.lock, flags);
	} else {
		if (pc_head.page_count < MAX_PAGE_CACHE) {
			spin_lock_irqsave(&pc_head.lock, flags);
			list_add_tail(&page->lru, &pc_head.page_list);
			pc_head.page_count++;
			spin_unlock_irqrestore(&pc_head.lock, flags);
		} else {
			set_page_count(page, 1);
			ClearPageReserved(page);
			__free_page(page);
		}
	}
}

static void camera_free_buffer_node(struct buf_node *buf_node, int limit)
{
	int i;
	struct page *page = NULL;

	/*
	 * vunmap will do TLB flush for us.
	 * We map uncachable memory, so needn't cache invalid operation here.
	 */
	vunmap(buf_node->vaddr);
	if (buf_node->io_type == V4L2_MEMORY_USERPTR)
		goto done;


	for (i = 0; i < buf_node->page_num; i++) {
		page = buf_node->pages[i];
		camera_free_page(page, limit);
	}

done:
	kfree(buf_node->pages);
	kfree(buf_node);
}

static void camera_free_buffer_list(int capture_mode, int limit)
{
	struct buf_node *buf_node;
	unsigned long flags;
	struct list_head *p_buf_head;
	struct list_head *p_report_head;

	if (CAMERA_MODE_STILL == capture_mode) {
		p_buf_head = &(g_camdrv_status.still_buf_head);
		p_report_head = &(g_camdrv_status.still_report_head);
	} else {
		p_buf_head = &(g_camdrv_status.video_buf_head);
		p_report_head = &(g_camdrv_status.video_report_head);
	}

	down_interruptible(&buf_list_sem);
	while (!list_empty(p_buf_head)) {
		buf_node =
			list_entry(p_buf_head->next, struct buf_node, buf_head);
		list_del_init(p_buf_head->next);
		dma_free_coherent(NULL, buf_node->dma_desc_size,
				buf_node->dma_desc_vaddr,
				buf_node->dma_desc_paddr);
		camera_free_buffer_node(buf_node, limit);
	}
	up(&buf_list_sem);

	/* empty the report list */
	spin_lock_irqsave(&report_list_lock, flags);
	while (!list_empty(p_report_head)) {
		list_del_init(p_report_head->next);
	}
	spin_unlock_irqrestore(&report_list_lock, flags);

}

static unsigned long uva_to_pa(unsigned long addr, struct page **page)
{
	unsigned long ret = 0UL;
	pgd_t *pgd;
	pmd_t *pmd;
	pte_t *pte;

	pgd = pgd_offset(current->mm, addr);
	if (!pgd_none(*pgd)) {
		pmd = pmd_offset(pgd, addr);
		if (!pmd_none(*pmd)) {
			pte = pte_offset_map(pmd, addr);
			if (!pte_none(*pte) && pte_present(*pte)) {
				(*page) = pte_page(*pte);
				ret = page_to_phys(*page);
				ret |= (addr & (PAGE_SIZE-1));
			}
		}
	}
	return ret;
}


static int camera_alloc_buffer_node(struct buf_node **buf_node, unsigned long userptr, int size)
{
	int page_num;
	int i, j;
	unsigned int ret = 0;
	struct page *page;
	unsigned int vaddr = PAGE_ALIGN(userptr);
	struct buf_node *buf;

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf) {
		pr_debug("Not enough memory\n");
		return -ENOMEM;
	}

	page_num = PAGE_ALIGN(size) / PAGE_SIZE;
	buf->pages =
		(struct page **)kzalloc(page_num * sizeof(long), GFP_KERNEL);
	if (!buf->pages) {
		pr_debug("Not enough memory\n");
		ret = -ENOMEM;
		goto alloc_node_error;
	}

	for (i = 0; i < page_num; i++) {
		if (vaddr){
			uva_to_pa(vaddr, &page);
			vaddr += PAGE_SIZE;
		} else {
			page = camera_alloc_page();

			if (!page) {
				pr_debug("Not enough memory\n");
				ret = -ENOMEM;
				goto alloc_pages_error;
			}
			set_page_count(page, 1);
			SetPageReserved(page);
		}

		buf->pages[i] = page;
	}

	buf->page_num = page_num;
	buf->size = page_num * PAGE_SIZE;
	buf->buf_id = -1;
	buf->vaddr =
		vmap(buf->pages, buf->page_num, VM_MAP,
				pgprot_noncached(pgprot_kernel));

	memset(buf->vaddr, 0, buf->size);

	/* check if the memory map is OK. */
	if (!buf->vaddr) {
		pr_debug("vmap() failure\n");
		ret = -EFAULT;
		goto vmap_error;
	}

	*buf_node = buf;

	return ret;

vmap_error:
alloc_pages_error:
	for (j = 0; j < i; j++) {
		page = buf->pages[j];
		camera_free_page(page, 1);
	}
	kfree(buf->pages);
alloc_node_error:
	kfree(buf);
	return ret;
}

static void camera_get_fifo_size(p_camera_context_t cam_ctx,
		struct buf_node *buf_node, int buffer_type)
{
	/*
	 * caculate the fifo0-2 transfer size
	 */
	unsigned int capture_output_format;
	unsigned int capture_output_width;
	unsigned int capture_output_height;
	unsigned int frame_size;

	capture_output_format = (buffer_type == VIDEO_CAPTURE_BUFFER) ?
		cam_ctx->video_capture_output_format :
		cam_ctx->still_capture_output_format;

	capture_output_width = (buffer_type == VIDEO_CAPTURE_BUFFER) ?
		cam_ctx->video_capture_width : cam_ctx->still_capture_width;

	capture_output_height = (buffer_type == VIDEO_CAPTURE_BUFFER) ?
		cam_ctx->video_capture_height : cam_ctx->still_capture_height;

	switch (capture_output_format) {
		case CAMERA_IMAGE_FORMAT_RAW10:
			frame_size = capture_output_width *
				capture_output_height * 2;
			buf_node->fifo0_size = frame_size;
			buf_node->fifo1_size = 0;
			buf_node->fifo2_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_RAW9:
			frame_size = capture_output_width *
				capture_output_height * 2;
			buf_node->fifo0_size = frame_size;
			buf_node->fifo1_size = 0;
			buf_node->fifo2_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_RAW8:
			frame_size = capture_output_width *
				capture_output_height;
			buf_node->fifo0_size = frame_size;
			buf_node->fifo1_size = 0;
			buf_node->fifo2_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_RGB888_PACKED:
			frame_size = capture_output_width *
				capture_output_height * 3;
			buf_node->fifo0_size = frame_size;
			buf_node->fifo1_size = 0;
			buf_node->fifo2_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_RGB565:
			frame_size = capture_output_width *
				capture_output_height * 2;
			buf_node->fifo0_size = frame_size;
			buf_node->fifo1_size = 0;
			buf_node->fifo2_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_YCBCR422_PACKED:
			frame_size = capture_output_width *
				capture_output_height * 2;
			buf_node->fifo0_size = frame_size;
			buf_node->fifo1_size = 0;
			buf_node->fifo2_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR:
			frame_size = capture_output_width *
				capture_output_height * 2;
			buf_node->fifo0_size = frame_size / 2;
			buf_node->fifo1_size = frame_size / 4;
			buf_node->fifo2_size = frame_size / 4;
			break;
#if defined(CONFIG_PXA310)
		case CAMERA_IMAGE_FORMAT_YCBCR420_PACKED:
			frame_size = capture_output_width *
				capture_output_height * 3 / 2;
			buf_node->fifo0_size = frame_size;
			buf_node->fifo1_size = 0;
			buf_node->fifo2_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_YCBCR420_PLANAR:
			frame_size = capture_output_width *
				capture_output_height * 2;
			buf_node->fifo0_size = frame_size / 2;
			buf_node->fifo1_size = frame_size / 8;
			buf_node->fifo2_size = frame_size / 8;
			break;
#endif
		default:
			break;
	}
	return;
}

static int camera_prepare_buffer(p_camera_context_t cam_ctx, unsigned long userptr,
               unsigned int buf_len, unsigned int *buf_index)
{
	int ret = 0;
	int buf_type, buf_size, dma_desc_size;
	int j;
	void *dma_desc_virt;
	int *buf_phy_addr_array;
	int *dma_desc_phy_addr_array;

	dma_addr_t dma_desc_phy = 0;
	struct buf_node *buf_node = NULL;
	unsigned int dma_desc_phy_addr_array_size;
	unsigned long flags;

	buf_type = (cam_ctx->capture_mode == CAMERA_MODE_STILL) ?
		STILL_CAPTURE_BUFFER : VIDEO_CAPTURE_BUFFER;

	if (mcam_get_buffer_size(cam_ctx, buf_type, &buf_size, &dma_desc_size)) {
		pr_debug("Get buffer size failure!\n");
		return -EFAULT;
	}

	if (userptr && buf_len < buf_size)
		return -EFAULT;

	buf_phy_addr_array =
		kzalloc(PAGE_ALIGN(buf_size) / PAGE_SIZE * sizeof(long),
				GFP_KERNEL);

	if (!buf_phy_addr_array) {
		pr_debug("No memory for record pages's physical address\n");
		return -ENOMEM;
	}

	dma_desc_phy_addr_array_size =
		(dma_desc_size + DMA_DESCRIPTOR_SIZE - 1) / DMA_DESCRIPTOR_SIZE;

	dma_desc_phy_addr_array =
		kzalloc(dma_desc_phy_addr_array_size * sizeof(long), GFP_KERNEL);

	if (!dma_desc_phy_addr_array) {
		pr_debug("No memory for record dma descriptors's physical address\n");
		kfree(buf_phy_addr_array);
		return -ENOMEM;
	}

	dma_desc_virt = dma_alloc_coherent(NULL, dma_desc_size,
			&dma_desc_phy, GFP_KERNEL);

	if (!dma_desc_virt) {
		pr_debug("Just alloc %d number buffer node\n", camera_get_buffer_num() - 1);
		goto exit;
	}

	ret = camera_alloc_buffer_node(&buf_node, userptr, buf_size);
	if (ret) {
		pr_debug("Alloc %dth buffer node failure\n", camera_get_buffer_num());
		dma_free_coherent(NULL, dma_desc_size, dma_desc_virt,
				dma_desc_phy);
		goto exit;
	}

	buf_node->io_type = userptr?V4L2_MEMORY_USERPTR : V4L2_MEMORY_MMAP;
	buf_node->buf_index = camera_get_buffer_num();
	buf_node->dma_desc_vaddr = dma_desc_virt;
	buf_node->dma_desc_paddr = dma_desc_phy;
	buf_node->dma_desc_size = dma_desc_size;

	pr_debug("dma_desc_virt = 0x%p, dma_desc_phy = 0x%08x\n", dma_desc_virt, dma_desc_phy);

	for (j = 0; j < buf_node->page_num; j++) {
		buf_phy_addr_array[j] = __pa(page_address(buf_node->pages[j]));
	}

	dma_desc_phy_addr_array[0] = buf_node->dma_desc_paddr;
	for (j = 1; j < dma_desc_phy_addr_array_size; j++) {
		dma_desc_phy_addr_array[j] =
			dma_desc_phy_addr_array[j - 1] +
			DMA_DESCRIPTOR_SIZE;
	}

	spin_lock_irqsave(&cam_queue_lock, flags);
	ret = mcam_prepare_buffer(cam_ctx,
			buf_node->vaddr,
			(void *)buf_phy_addr_array,
			buf_node->page_num,
			buf_size,
			buf_type,
			buf_node->dma_desc_vaddr,
			dma_desc_phy_addr_array,
			&buf_node->buf_id,
			&buf_node->Y_vaddr,
			&buf_node->Cb_vaddr,
			&buf_node->Cr_vaddr);
	spin_unlock_irqrestore(&cam_queue_lock, flags);

	if (ret) {
		pr_debug("Prepare %dth buffer node failure\n", buf_node->buf_index + 1);
		camera_free_buffer_node(buf_node, 1);
		dma_free_coherent(NULL, dma_desc_size, dma_desc_virt, dma_desc_phy);
		goto exit;
	}

	camera_get_fifo_size(cam_ctx, buf_node, buf_type);

	list_add_tail(&buf_node->buf_head, g_camdrv_status.p_buf_head);

exit:
	kfree(buf_phy_addr_array);
	kfree(dma_desc_phy_addr_array);

	*(g_camdrv_status.p_buf_ready) = 1;

	pr_debug("test point in camera_prepare_buffers\n");

	if (buf_index)
		*buf_index = buf_node->buf_index;

	return ret;
}

static int camera_prepare_buffers(p_camera_context_t cam_ctx, int buf_num)
{
	int i;

	for (i = 0; i < buf_num; i++)
		camera_prepare_buffer(cam_ctx, 0, 0, NULL);

	return camera_get_buffer_num();
}

static int camera_submit_buffer(p_camera_context_t cam_ctx,
		unsigned int buf_indx)
{
	struct buf_node *buf_node;
	unsigned int buf_type;
	int ret;
	unsigned long flags;

	buf_node = camera_get_buffer_from_index(buf_indx);

	if (!buf_node)
		goto exit;

	buf_type = (cam_ctx->capture_mode == CAMERA_MODE_STILL) ?
		STILL_CAPTURE_BUFFER : VIDEO_CAPTURE_BUFFER;

	spin_lock_irqsave(&cam_queue_lock, flags);
	ret = mcam_submit_buffer(cam_ctx, buf_node->buf_id, buf_type);
	spin_unlock_irqrestore(&cam_queue_lock, flags);
	if (ret) {
		pr_debug("Submit %dth buffer node failure\n", buf_indx);
		goto exit;
	}

	*(g_camdrv_status.p_buf_submited) = 1;

	return 0;

exit:
	return -EFAULT;
}

static int camera_submit_buffers(p_camera_context_t cam_ctx)
{
	struct list_head *pos;
	struct buf_node *buf_node;
	unsigned int buf_type;
	int i = 0;
	unsigned long flags;

	if (!*(g_camdrv_status.p_buf_ready)) {
		pr_debug("buffer not ready!\n");
		goto exit;
	}
	buf_type = (cam_ctx->capture_mode == CAMERA_MODE_STILL) ?
		STILL_CAPTURE_BUFFER : VIDEO_CAPTURE_BUFFER;

	list_for_each(pos, g_camdrv_status.p_buf_head) {
		buf_node = list_entry(pos, struct buf_node, buf_head);

		spin_lock_irqsave(&cam_queue_lock, flags);
		mcam_submit_buffer(cam_ctx, buf_node->buf_id, buf_type);
		spin_unlock_irqrestore(&cam_queue_lock, flags);

		i++;
	}

	if (i == 0) {
		goto exit;
	}

	*(g_camdrv_status.p_buf_submited) = 1;

	return 0;

exit:
	return -EFAULT;
}

static int camera_start_capture(p_camera_context_t cam_ctx)
{

	if (!*(g_camdrv_status.p_buf_submited)) {
		pr_debug("pxa_camera: buffer not submited!\n");
		goto exit;
	}

	mcam_set_interrupt_mask(cam_ctx, INT_MASK);

	if (mcam_set_capture_format(cam_ctx)) {
		goto exit;
	}

	cam_ctx->frame_rate = pxa_camera_get_framerate(&g_camdrv_status);
	mcam_set_capture_frame_rate(cam_ctx);

	if (CAMERA_MODE_VIDEO == cam_ctx->capture_mode) {
		if (mcam_start_video_capture(cam_ctx)) {
			goto exit;
		}
	} else {
		if (mcam_capture_still_image(cam_ctx)) {
			goto exit;
		}
	}

	enable_irq(IRQ_CAMERA);

	return 0;

exit:

	return -EFAULT;
}

static void camera_stop_capture(p_camera_context_t cam_ctx)
{
	/*
	 * stop video capture
	 * stop still capture
	 * Note: a workaround of camera drv for stopping still capture
	 * which has no such stop still capture primitives
	 */
	mcam_stop_video_capture(cam_ctx);
	disable_irq(IRQ_CAMERA);

	return;

}

static void camera_desubmit_buffers(p_camera_context_t cam_ctx)
{
	/*
	 * stop capture: a workaround of camera drv
	 * which has no such desubmit buffer primitives
	 */

	if (CAMERA_MODE_STILL == cam_ctx->capture_mode) {
		cam_ctx->still_capture_buffer_queue.head = NULL;
		cam_ctx->still_capture_buffer_queue.tail = NULL;
	} else {
		cam_ctx->video_capture_buffer_queue.head = NULL;
		cam_ctx->video_capture_buffer_queue.tail = NULL;
	}

	*(g_camdrv_status.p_buf_submited) = 0;

}

static void camera_deprepare_buffers(p_camera_context_t cam_ctx)
{
	camera_free_buffer_list(cam_ctx->capture_mode, 1);
	*(g_camdrv_status.p_buf_ready) = 0;
}

/*
 * Init/Deinit APIs
 */
static int camera_init(p_camera_context_t camera_context)
{

	/* FIXME: init the Vcc for Camera analog. Should be done by system */
	if (CAMERA_TYPE_OMNIVISION_2620 == camera_context->sensor_type) {
		mhn_pmic_set_voltage(VCC_CAMERA_ANA, 3200);
	} else if (CAMERA_TYPE_OMNIVISION_2630 == camera_context->sensor_type) {
		mhn_pmic_set_voltage(VCC_CAMERA_ANA, 2800);
	} else {
		mhn_pmic_set_voltage(VCC_CAMERA_ANA, 2800);
	}

	/* enable QCI clock  */
	pxa_set_cken(CKEN_CAMERA, 1);

	if (!mcam_init(camera_context)) {
		return 0;
	} else {
		return -1;
	}
}

static int camera_deinit(p_camera_context_t camera_context)
{
	int status;

	if (!mcam_deinit(camera_context)) {
		status = 0;
	} else {
		status = -EFAULT;
	}

	/* disable QCI clock  */
	pxa_set_cken(CKEN_CAMERA, 0);

	return status;
}

static void pxa_camera_set_mode(struct pxa_camera_driver_status_t
		*p_camdrv_status, int capture_mode)
{
	p_camdrv_status->capture_mode = capture_mode;
	if (CAMERA_MODE_VIDEO == capture_mode) {
		p_camdrv_status->p_buf_ready =
			&(p_camdrv_status->video_buf_ready);
		p_camdrv_status->p_buf_submited =
			&(p_camdrv_status->video_buf_submited);
		p_camdrv_status->p_capture_started =
			&(p_camdrv_status->video_capture_started);
		p_camdrv_status->p_buf_head =
			&(p_camdrv_status->video_buf_head);
		p_camdrv_status->p_report_head =
			&(p_camdrv_status->video_report_head);
		p_camdrv_status->p_timeperframe_numerator =
			&(p_camdrv_status->video_timeperframe_numerator);
		p_camdrv_status->p_timeperframe_denominator =
			&(p_camdrv_status->video_timeperframe_denominator);

	} else {
		p_camdrv_status->p_buf_ready =
			&(p_camdrv_status->still_buf_ready);
		p_camdrv_status->p_buf_submited =
			&(p_camdrv_status->still_buf_submited);
		p_camdrv_status->p_capture_started =
			&(p_camdrv_status->still_capture_started);
		p_camdrv_status->p_buf_head =
			&(p_camdrv_status->still_buf_head);
		p_camdrv_status->p_report_head =
			&(p_camdrv_status->still_report_head);
		p_camdrv_status->p_timeperframe_numerator =
			&(p_camdrv_status->still_timeperframe_numerator);
		p_camdrv_status->p_timeperframe_denominator =
			&(p_camdrv_status->still_timeperframe_denominator);

	}

}

static int camera_do_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, void *arg)
{
	int retval = 0;
	p_camera_context_t cam_ctx = g_camera_context;

	if (!cam_ctx) {
		return -EINVAL;
	}

	switch (cmd) {

		case VIDIOC_QUERYCAP:
			{
				struct v4l2_capability *cap = arg;
				retval = 0;
				memset(cap, 0, sizeof(*cap));

				strcpy(cap->driver, "pxa camera");
				strcpy(cap->card, "");
				cap->version = PXA_CAMERA_VERSION;
				cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
					V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;

				retval = 0;
				break;
			}

		case VIDIOC_ENUMINPUT:
			{
				struct v4l2_input *i = arg;
				unsigned int n;

				n = i->index;
				if (n >= SENSOR_NUMBER) {
					retval = -EINVAL;
					break;
				}

				memset(i, 0, sizeof(*i));
				i->index = n;
				i->type = V4L2_INPUT_TYPE_CAMERA;

				switch (n) {
					case OV2620_SENSOR:
						strcpy(i->name, "Omnivision2620");
						break;

					case OV2630_SENSOR:
						strcpy(i->name, "Omnivision2630");
						break;

					case OV7660_SENSOR:
						strcpy(i->name, "Omnivision7660");
						break;

					default:
						break;
				}

				break;
			}

		case VIDIOC_ENUM_FMT:
			{
				struct v4l2_fmtdesc *f = arg;
				enum v4l2_buf_type type;
				int index;

				if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (f->index >= PXA_CAMERA_FORMATS) {
					retval = -EINVAL;
					break;
				}

				type = f->type;
				index = f->index;

				memset(f, 0, sizeof(*f));
				f->index = index;
				f->type = type;
				f->pixelformat = pxa_camera_formats[index].fourcc;
				strlcpy(f->description,
					pxa_camera_formats[index].name,
					sizeof(f->description));

				break;
			}

		case VIDIOC_G_FMT:
			{
				struct v4l2_format *f = arg;

				if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (CAMERA_MODE_VIDEO == cam_ctx->capture_mode) {
					f->fmt.pix.pixelformat =
						camera_fromat_to_v4l2(cam_ctx->
						video_capture_output_format);
					f->fmt.pix.width =
						cam_ctx->video_capture_width;
					f->fmt.pix.height =
						cam_ctx->video_capture_height;
				} else {
					f->fmt.pix.pixelformat =
						camera_fromat_to_v4l2(cam_ctx->
						still_capture_output_format);
					f->fmt.pix.width =
						cam_ctx->still_capture_width;
					f->fmt.pix.height =
						cam_ctx->still_capture_height;
				}

				break;
			}

		case VIDIOC_S_FMT:
			{
				struct v4l2_format *f = arg;
				unsigned int pixelformat;
				int capture_width;
				int capture_height;

				if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (CAMERA_TYPE_OMNIVISION_2620 ==
					cam_ctx->sensor_type) {
					if (V4L2_PIX_FMT_SRGGB10 !=
							f->fmt.pix.pixelformat) {
						retval = -EINVAL;
						break;
					}
					if ((f->fmt.pix.width > 1600) ||
						(f->fmt.pix.height > 1200)) {
						retval = -EINVAL;
						break;
					}
				} else if (CAMERA_TYPE_OMNIVISION_2630 ==
						cam_ctx->sensor_type) {
					if ((V4L2_PIX_FMT_RGB24 !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_SRGGB10 !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_YUV422P !=
							f->fmt.pix.pixelformat)
#if defined(CONFIG_PXA310)
						&& (V4L2_PIX_FMT_SRGGB8 !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_YUV420 !=
							f->fmt.pix.pixelformat)) {
#else
						&& (V4L2_PIX_FMT_SRGGB8 !=
							f->fmt.pix.pixelformat)) {
#endif

						retval = -EINVAL;
						break;
					}
					if ((f->fmt.pix.width > 1600)
						|| (f->fmt.pix.height > 1200)) {
						retval = -EINVAL;
						break;
					}
				} else {	/* OV7660 */
					if ((V4L2_PIX_FMT_YUV422P !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_RGB565X !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_SRGGB8 !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_SRGGB10 !=
							f->fmt.pix.pixelformat)
#if defined(CONFIG_PXA310)
						&& (V4L2_PIX_FMT_RGB24 !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_YUV420 !=
							f->fmt.pix.pixelformat)) {
#else
						&& (V4L2_PIX_FMT_RGB24 !=
							f->fmt.pix.pixelformat)) {
#endif
						retval = -EINVAL;
						break;
					}
					if ((f->fmt.pix.width > 640)
						|| (f->fmt.pix.height > 480)) {
						retval = -EINVAL;
						break;
					}
				}

				pixelformat = camera_format_from_v4l2(
						f->fmt.pix.pixelformat);
				capture_width = f->fmt.pix.width;
				capture_height = f->fmt.pix.height;

				if (CAMERA_MODE_VIDEO == cam_ctx->capture_mode) {

					if ((cam_ctx->video_capture_width !=
							capture_width) ||
						(cam_ctx->video_capture_height !=
							capture_height) ||
						(pixelformat !=
							cam_ctx->
							video_capture_output_format)) {
						pxa_camera_reset(&g_camdrv_status);
					}
					cam_ctx->video_capture_output_format =
						camera_format_from_v4l2(f->
							fmt.pix.pixelformat);
					cam_ctx->video_capture_input_format =
						camera_default_input(cam_ctx->
							video_capture_output_format);
					cam_ctx->video_capture_width =
						f->fmt.pix.width;
					cam_ctx->video_capture_height =
						f->fmt.pix.height;

				} else {

					if ((cam_ctx->still_capture_width !=
							capture_width) ||
						(cam_ctx->still_capture_height !=
							capture_height)	||
						(pixelformat != cam_ctx->
							still_capture_output_format)) {
						pxa_camera_reset(&g_camdrv_status);
					}
					cam_ctx->still_capture_output_format =
						camera_format_from_v4l2(f->
							fmt.pix.pixelformat);
					cam_ctx->still_capture_input_format =
						camera_default_input(cam_ctx->
							still_capture_output_format);
					cam_ctx->still_capture_width =
						f->fmt.pix.width;
					cam_ctx->still_capture_height =
						f->fmt.pix.height;

				}

				break;
			}

		case VIDIOC_TRY_FMT:
			{
				struct v4l2_format *f = arg;

				if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (CAMERA_TYPE_OMNIVISION_2620 ==
						cam_ctx->sensor_type) {
					if (V4L2_PIX_FMT_SRGGB10 !=
							f->fmt.pix.pixelformat) {
						retval = -EINVAL;
						break;
					}
					if ((f->fmt.pix.width > 1600) ||
						(f->fmt.pix.height > 1200)) {
						retval = -EINVAL;
						break;
					}
				} else if (CAMERA_TYPE_OMNIVISION_2630 ==
						cam_ctx->sensor_type) {
					if (V4L2_PIX_FMT_SRGGB10 !=
							f->fmt.pix.pixelformat) {
						retval = -EINVAL;
						break;
					}
					if ((f->fmt.pix.width > 1600) ||
						(f->fmt.pix.height > 1200)) {
						retval = -EINVAL;
						break;
					}
				} else {
					if ((V4L2_PIX_FMT_YUV422P !=
							f->fmt.pix.pixelformat)
						&& (V4L2_PIX_FMT_RGB565X !=
							f->fmt.pix.pixelformat)) {
						retval = -EINVAL;
						break;
					}
					if ((f->fmt.pix.width > 640) ||
						(f->fmt.pix.height > 480)) {
						retval = -EINVAL;
						break;
					}
				}
				break;
			}

		case VIDIOC_S_INPUT:
			{
				int *i = arg;
				int sensor_type;

				if (*(g_camdrv_status.p_capture_started)) {
					pxa_camera_ioctl_streamoff(
						&g_camdrv_status);
				}

				switch (*i) {
					case OV2620_SENSOR:
						if (ov2620_detected == 0) {
							retval = -EINVAL;
							return retval;
						}
						sensor_type =
							CAMERA_TYPE_OMNIVISION_2620;
						break;
					case OV2630_SENSOR:
						if (ov2630_detected == 0) {
							retval = -EINVAL;
							return retval;
						}
						sensor_type =
							CAMERA_TYPE_OMNIVISION_2630;
						break;
					case OV7660_SENSOR:
						if (ov7660_detected == 0) {
							retval = -EINVAL;
							return retval;
						}
						sensor_type =
							CAMERA_TYPE_OMNIVISION_7660;
						break;
					default:
						retval = -EINVAL;
						return retval;
				}

				if (sensor_type != cam_ctx->sensor_type) {
					mcam_deinit(cam_ctx);
					if (sensor_type ==
						CAMERA_TYPE_OMNIVISION_2620) {
						SET_OV2620_SENSOR(cam_ctx);
					} else if (sensor_type ==
						CAMERA_TYPE_OMNIVISION_2630) {
						SET_OV2630_SENSOR(cam_ctx);
					} else {
						SET_OV7660_SENSOR(cam_ctx);
					}
					mcam_init(cam_ctx);
				}

				break;
			}

		case VIDIOC_G_INPUT:
			{
				int *i = arg;

				if (CAMERA_TYPE_OMNIVISION_2620 ==
					cam_ctx->sensor_type) {
					*i = OV2620_SENSOR;
				} else if (CAMERA_TYPE_OMNIVISION_2630 ==
					cam_ctx->sensor_type) {
					*i = OV2630_SENSOR;
				} else if (CAMERA_TYPE_OMNIVISION_7660 ==
					cam_ctx->sensor_type) {
					*i = OV7660_SENSOR;
				} else {
					retval = -EINVAL;
				}
				break;
			}

		case VIDIOC_G_PARM:
			{
				struct v4l2_streamparm *parm = arg;

				if (parm->type !=
					V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (CAMERA_MODE_STILL ==
					cam_ctx->capture_mode) {
					parm->parm.capture.capturemode =
						V4L2_MODE_HIGHQUALITY;
				} else {
					parm->parm.capture.capturemode = 0;
				}
				parm->parm.capture.timeperframe.numerator =
					*(g_camdrv_status.p_timeperframe_numerator);
				parm->parm.capture.timeperframe.denominator =
					*(g_camdrv_status.p_timeperframe_denominator);

				break;
			}

		case VIDIOC_S_PARM:
			{

				struct v4l2_streamparm *parm = arg;
				int capture_mode;

				if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (V4L2_MODE_HIGHQUALITY ==
						parm->parm.capture.capturemode) {
					capture_mode = CAMERA_MODE_STILL;
				} else {
					capture_mode = CAMERA_MODE_VIDEO;
				}

				/* Spatial Scaling Unit(SSU) related functions */
				if (CI_SSU_SCALE_HALF ==
						parm->parm.capture.extendedmode) {
					if (CAMERA_MODE_STILL == capture_mode) {
						cam_ctx->still_capture_scale =
							CAMERA_CAPTURE_SCALE_HALF;
					} else {
						cam_ctx->video_capture_scale =
							CAMERA_CAPTURE_SCALE_HALF;
					}
				} else if (CI_SSU_SCALE_QUARTER ==
						parm->parm.capture.extendedmode) {
					if (CAMERA_MODE_STILL == capture_mode) {
						cam_ctx->still_capture_scale =
							CAMERA_CAPTURE_SCALE_QUATER;
					} else {
						cam_ctx->video_capture_scale =
							CAMERA_CAPTURE_SCALE_QUATER;
					}
				} else {
					if (CAMERA_MODE_STILL == capture_mode) {
						cam_ctx->still_capture_scale =
							CAMERA_CAPTURE_SCALE_DISABLE;
					} else {
						cam_ctx->video_capture_scale =
							CAMERA_CAPTURE_SCALE_DISABLE;
					}
				}

				if ((capture_mode != cam_ctx->capture_mode) ||
					(*(g_camdrv_status.p_timeperframe_numerator) !=
					parm->parm.capture.timeperframe.numerator) ||
					(*(g_camdrv_status.p_timeperframe_denominator) !=
					parm->parm.capture.timeperframe.denominator)) {

					if (*(g_camdrv_status.p_capture_started)) {
						pxa_camera_ioctl_streamoff
							(&g_camdrv_status);
					}

					if (capture_mode != cam_ctx->capture_mode) {
						cam_ctx->capture_mode = capture_mode;
						pxa_camera_set_mode(&g_camdrv_status,
								cam_ctx->
								capture_mode);
					}

					*(g_camdrv_status.p_timeperframe_numerator) =
						parm->parm.capture.timeperframe.numerator;
					*(g_camdrv_status.p_timeperframe_denominator) =
						parm->parm.capture.timeperframe.denominator;

				}

				break;

			}

		case VIDIOC_S_CTRL:
			{
				struct v4l2_control *ctrl = arg;

				switch (ctrl->id) {
					case V4L2_CID_CONTRAST:
						retval = mcam_set_contrast_value(
							cam_ctx,
							SENSOR_MANUAL_CONTRAST,
							ctrl->value);
						break;
					case V4L2_CID_DO_WHITE_BALANCE:
						retval =
							mcam_set_white_balance_value(
							cam_ctx,
							SENSOR_MANUAL_WHITEBALANCE,
							ctrl->value);
						break;
					case V4L2_CID_EXPOSURE:
						retval =
							mcam_set_exposure_value(cam_ctx,
							SENSOR_MANUAL_EXPOSURE,
							ctrl->value);
						break;
					default:
						retval = -EINVAL;
						break;
				}
				break;
			}

		case VIDIOC_G_CTRL:
			{
				/* do nothing */
				break;
			}

		case VIDIOC_QUERYBUF:
			{
				struct v4l2_buffer *buf = arg;
				int buf_size;

				if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (buf->memory != V4L2_MEMORY_MMAP) {
					retval = -EINVAL;
					break;
				}

				if (buf->index >= camera_get_buffer_num()) {
					retval = -EINVAL;
					break;
				}

				buf_size = camera_get_buffer_size();
				buf->length = buf_size;
				buf->m.offset = buf->index * buf_size;

				break;
			}

		case VIDIOC_QBUF:
			{
				struct v4l2_buffer *buf = arg;

				unsigned long flags;

				if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (buf->memory != V4L2_MEMORY_MMAP && (buf->memory != V4L2_MEMORY_USERPTR)) {
					retval = -EINVAL;
					break;
				}

				if ((buf->memory == V4L2_MEMORY_USERPTR) && (buf->length > 0)) {
					if (camera_prepare_buffer(cam_ctx, buf->m.userptr, buf->length, &buf->index)) {
						retval = -EINVAL;
						break;
					}
				}

				if (buf->index >= camera_get_buffer_num()) {
					retval = -EINVAL;
					break;
				}

				spin_lock_irqsave(&report_list_lock, flags);
				retval = camera_submit_buffer(cam_ctx, buf->index);
				spin_unlock_irqrestore(&report_list_lock, flags);

				break;
			}

		case VIDIOC_DQBUF:
			{
				struct v4l2_buffer *buf = arg;
				struct buf_node *buf_node;
				unsigned long flags;

				if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (buf->memory != V4L2_MEMORY_MMAP && (buf->memory != V4L2_MEMORY_USERPTR)) {
					retval = -EINVAL;
					break;
				}

				spin_lock_irqsave(&report_list_lock, flags);

				if (!list_empty(g_camdrv_status.p_report_head)) {
					buf_node =
						list_entry(
						g_camdrv_status.p_report_head->
						next,
						struct buf_node,
						report_head);
					BUG_ON(!buf_node);

					list_del(&buf_node->report_head);

					buf->index = buf_node->buf_index;

				} else
					retval = -EIO;
				spin_unlock_irqrestore(&report_list_lock, flags);

				break;
			}

		case VIDIOC_REQBUFS:
			{
				struct v4l2_requestbuffers *req = arg;
				int count;

				if (req->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (req->type != V4L2_MEMORY_MMAP && (req->memory != V4L2_MEMORY_USERPTR)) {
					retval = -EINVAL;
					break;
				}

				camera_deprepare_buffers(cam_ctx);

				if ((req->reserved[0] & 0x1) == YUV_NO_PADDING)
					cam_ctx->align_type = YUV_NO_PADDING;
				else
					cam_ctx->align_type = YUV_HAVE_PADDING;

				if (req->memory == V4L2_MEMORY_USERPTR)
					break;

				count = camera_prepare_buffers(cam_ctx, req->count);
				if (count < 0)
					retval = -EFAULT;
				else
					req->count = count;

				break;
			}

		case VIDIOC_STREAMON:
			{
				int *type = arg;

				if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				retval = pxa_camera_ioctl_streamon(
						&g_camdrv_status);

				break;
			}

		case VIDIOC_STREAMOFF:
			{
				int *type = arg;

				if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				pxa_camera_ioctl_streamoff(&g_camdrv_status);

				break;
			}

			/* Application extended IOCTL.  */
			/* Register access interface    */

		case WCAM_VIDIOCGCIREG:
			{
				struct reg_set_s *reg = arg;
				reg->val2 =  __raw_readl(
					((unsigned int)cam_ctx->ci_reg_base
					  + (reg->val1)));
				break;

			}

		case WCAM_VIDIOCSCIREG:
			{
				struct reg_set_s *reg = arg;
				unsigned int regVal;

				__raw_writel(reg->val2, (unsigned int)cam_ctx->
					ci_reg_base + reg->val1);
				regVal =
					__raw_readl((unsigned int)cam_ctx->
						ci_reg_base + reg->val1);
				if (regVal != reg->val2) {
					retval = -EFAULT;
					break;
				}
				break;
			}

		case WCAM_VIDIOCGCAMREG:
			{
				struct reg_set_s *reg = arg;
				unsigned char regVal;

				mcam_read_8bit(cam_ctx, (unsigned char)reg->val1, &regVal);
				reg->val2 = regVal;

				break;

			}

		case WCAM_VIDIOCSCAMREG:
			{
				struct reg_set_s *reg = arg;
				mcam_write_8bit(cam_ctx, (unsigned char)reg->val1,
					(unsigned char)reg->val2);
				break;
			}

		case WCAM_VIDIOCGHST:
			{
				struct hst_context_s *hst_ctx = arg;
				unsigned int color_type = CI_HISTO_GREEN1;
				unsigned short *ptemp;
				int i, count;
				unsigned int hst_sum = 0;

				if (HST_COLOR_RED == hst_ctx->color) {
					color_type = CI_HISTO_RED;
				} else if (HST_COLOR_BLUE == hst_ctx->color) {
					color_type = CI_HISTO_BLUE;
				} else if (HST_COLOR_GREEN1 == hst_ctx->color) {
					color_type = CI_HISTO_GREEN1;
				} else if (HST_COLOR_GREEN2 == hst_ctx->color) {
					color_type = CI_HISTO_GREEN2;
				}

				disable_irq(IRQ_CAMERA);
				mcam_get_histogram_info(cam_ctx, color_type,
					&(hst_ctx->size), &(hst_ctx->sum));
				enable_irq(IRQ_CAMERA);
				ptemp =	(unsigned short *)cam_ctx->
						histogram_lut_buffer_virtual;

				/* software work around for sum */
				count = hst_ctx->size / 2;
				for (i = 0; i < count; i++) {
					hst_sum += i * (ptemp[i]);
				}
				hst_ctx->sum = hst_sum;
				retval = copy_to_user((char __user *)hst_ctx->hst,
						(void *)(cam_ctx->
						 histogram_lut_buffer_virtual),
						hst_ctx->size);

				break;
			}

		default:
			{
				retval = -ENOIOCTLCMD;
				break;
			}
	}

	return retval;
}

static irqreturn_t pxa_camera_irq(int irq, void *dev_id, struct pt_regs * regs)
{
	unsigned int int_state;
	p_camera_context_t cam_ctx = g_camera_context;
	int buf_id;
	struct buf_node *buf_node;
	unsigned long flags1, flags2;

	int_state = mcam_get_interrupt_status(cam_ctx);

	spin_lock_irqsave(&report_list_lock, flags1);
	spin_lock_irqsave(&cam_queue_lock, flags2);

	if (!mcam_get_filled_buffer(cam_ctx, &buf_id)) {
		buf_node = camera_get_buffer_from_id(buf_id);
		if (&buf_node->report_head != g_camdrv_status.p_report_head->next) {
			list_add_tail(&buf_node->report_head, g_camdrv_status.p_report_head);
		}
		if (g_camdrv_status.task_waiting) {
			wake_up_interruptible(&(g_camdrv_status. camera_wait_q));
			g_camdrv_status.task_waiting = 0;
		}
	}

	spin_unlock_irqrestore(&cam_queue_lock, flags2);
	spin_unlock_irqrestore(&report_list_lock, flags1);

	mcam_clear_interrupt_status(cam_ctx, int_state);

	return IRQ_HANDLED;
}

/*
 * Application interface
 */

static int pxa_camera_get_framerate(struct pxa_camera_driver_status_t
		*p_camdrv_status)
{

	p_camera_context_t cam_ctx = g_camera_context;
	unsigned int sensor_timeperframe_numerator = 0;
	unsigned int sensor_timeperframe_denominator = 0;
	unsigned int framerate;

	if ((0 == *(p_camdrv_status->p_timeperframe_denominator))
			|| (0 == *(p_camdrv_status->p_timeperframe_numerator))) {
		framerate = 0;
		return framerate;

	}

	if (CAMERA_TYPE_OMNIVISION_2620 == cam_ctx->sensor_type) {
		switch (cam_ctx->capture_input_format) {
			case CAMERA_IMAGE_FORMAT_RAW10:
				if ((cam_ctx->capture_input_width <= 404)
						&& (cam_ctx->capture_input_height <= 302)) {
					/* CIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if ((cam_ctx->capture_input_width <= 808)
						&& (cam_ctx->capture_input_height <= 604)) {
					/* SVGA */
					sensor_timeperframe_numerator = 2;
					sensor_timeperframe_denominator = 15;
				} else if ((cam_ctx->capture_input_width <= 1616)
						&& (cam_ctx->capture_input_height <= 1208)) {
					/* UXGA */
					sensor_timeperframe_numerator = 2;
					sensor_timeperframe_denominator = 5;
				}
				break;
			default:
				break;
		}

	} else if (CAMERA_TYPE_OMNIVISION_2630 == cam_ctx->sensor_type) {
		switch (cam_ctx->capture_input_format) {
			case CAMERA_IMAGE_FORMAT_RAW10:
				if ((cam_ctx->capture_input_width <= 400)
						&& (cam_ctx->capture_input_height <= 292)) {
					/* CIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if ((cam_ctx->capture_input_width <= 800)
						&& (cam_ctx->capture_input_height <= 600)) {
					/* SVGA */
					sensor_timeperframe_numerator = 2;
					sensor_timeperframe_denominator = 15;
				} else if ((cam_ctx->capture_input_width <= 1600)
						&& (cam_ctx->capture_input_height <= 1200)) {
					/* UXGA */
					sensor_timeperframe_numerator = 2;
					sensor_timeperframe_denominator = 5;
				}
				break;
			default:
				break;
		}

	} else {

		switch (cam_ctx->capture_input_format) {
			case CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR:
			case CAMERA_IMAGE_FORMAT_YCBCR422_PACKED:
				if (cam_ctx->capture_input_width <= 88
						&& cam_ctx->capture_input_height <= 72) {
					/* QQCIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 176
						&& cam_ctx->capture_input_height <= 144) {
					/* QCIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 352
						&& cam_ctx->capture_input_height <= 288) {
					/* CIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 160
						&& cam_ctx->capture_input_height <= 120) {
					/* QQVGA */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 320
						&& cam_ctx->capture_input_height <= 240) {
					/* QVGA */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 640
						&& cam_ctx->capture_input_height <= 480) {
					/* VGA */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				}
				break;
			case CAMERA_IMAGE_FORMAT_RGB565:
				if (cam_ctx->capture_input_width <= 88
						&& cam_ctx->capture_input_height <= 72) {
					/* QQCIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 176
						&& cam_ctx->capture_input_height <= 144) {
					/* QCIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 352
						&& cam_ctx->capture_input_height <= 288) {
					/* CIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 160
						&& cam_ctx->capture_input_height <= 120) {
					/* QQVGA */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 320
						&& cam_ctx->capture_input_height <= 240) {
					/* QVGA */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 640
						&& cam_ctx->capture_input_height <= 480) {
					/* VGA */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				}
				break;
			case CAMERA_IMAGE_FORMAT_RAW8:
				if (cam_ctx->capture_input_width <= 88
						&& cam_ctx->capture_input_height <= 72) {
					/* QQCIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 176
						&& cam_ctx->capture_input_height <= 144) {
					/* QCIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 352
						&& cam_ctx->capture_input_height <= 288) {
					/* CIF */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 160
						&& cam_ctx->capture_input_height <= 120) {
					/* QQVGA */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 320
						&& cam_ctx->capture_input_height <= 240) {
					/* QVGA */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				} else if (cam_ctx->capture_input_width <= 640
						&& cam_ctx->capture_input_height <= 480) {
					/* VGA */
					sensor_timeperframe_numerator = 1;
					sensor_timeperframe_denominator = 15;
				}
				break;
			default:
				break;
		}

	}

	framerate =
		((*(p_camdrv_status->p_timeperframe_numerator)) *
		 sensor_timeperframe_denominator) /
		((*(p_camdrv_status->p_timeperframe_denominator)) *
		 sensor_timeperframe_numerator);

	if (framerate == 0) {
		framerate = 0;
	} else if (framerate > 8) {
		framerate = 7;
	} else {
		framerate--;
	}

	return framerate;

}

static int pxa_camera_reset(struct pxa_camera_driver_status_t *p_camdrv_status)
{

	p_camera_context_t cam_ctx = g_camera_context;

	if (*(p_camdrv_status->p_capture_started)) {
		pxa_camera_ioctl_streamoff(p_camdrv_status);
	}

	if (*(p_camdrv_status->p_buf_submited)) {
		camera_desubmit_buffers(cam_ctx);
	}

	if (*(p_camdrv_status->p_buf_ready)) {
		camera_deprepare_buffers(cam_ctx);
	}

	return 0;
}

static int pxa_camera_ioctl_streamon(struct pxa_camera_driver_status_t
		*p_camdrv_status)
{

	struct buf_node *buf_node;
	p_camera_context_t cam_ctx = g_camera_context;
	unsigned long flags;

	if (!*(g_camdrv_status.p_buf_ready)) {
		if (CAMERA_MODE_STILL == cam_ctx->capture_mode) {
			if (camera_prepare_buffers(cam_ctx, 2) < 0) {
				return -EIO;
			}
		} else {
			if (camera_prepare_buffers(cam_ctx, 3) < 0) {
				return -EIO;
			}
		}
	}

	if (!*(g_camdrv_status.p_buf_submited)) {
		if (camera_submit_buffers(cam_ctx)) {
			return -EIO;
		}
	}

	spin_lock_irqsave(&report_list_lock, flags);
	/* Empty report head and put its buf into head queue */
	while (!list_empty(p_camdrv_status->p_report_head)) {
		buf_node =
			list_entry(g_camdrv_status.p_report_head->next,
					struct buf_node, report_head);
		list_del_init(&buf_node->report_head);
		if (camera_submit_buffer(cam_ctx, buf_node->buf_index)) {
			spin_unlock_irqrestore(&report_list_lock, flags);
			return -EIO;
		}
	}
	spin_unlock_irqrestore(&report_list_lock, flags);

	if (!*(g_camdrv_status.p_capture_started)) {
		if (camera_start_capture(cam_ctx)) {
			return -EIO;
		}
	}

	*(g_camdrv_status.p_capture_started) = 1;
	return 0;
}

static void pxa_camera_ioctl_streamoff(struct pxa_camera_driver_status_t
		*p_camdrv_status)
{
	p_camera_context_t cam_ctx = g_camera_context;

	camera_stop_capture(cam_ctx);
	*(g_camdrv_status.p_capture_started) = 0;
}

static int pxa_camera_open(struct inode *inode, struct file *file)
{
	int status = -1;
	dma_addr_t handle = 0;

	p_camera_context_t cam_ctx = g_camera_context;

	if (g_camdrv_status.suspended) {
		return status;
	}
	g_camdrv_status.driver_opened = 1;

	g_camdrv_status.still_buf_ready = 0;
	g_camdrv_status.video_buf_ready = 0;
	g_camdrv_status.still_buf_submited = 0;
	g_camdrv_status.video_buf_submited = 0;
	g_camdrv_status.still_capture_started = 0;
	g_camdrv_status.video_capture_started = 0;

	g_camdrv_status.still_timeperframe_numerator = 0;
	g_camdrv_status.video_timeperframe_numerator = 0;
	g_camdrv_status.still_timeperframe_denominator = 0;
	g_camdrv_status.video_timeperframe_denominator = 0;

	INIT_LIST_HEAD(&(g_camdrv_status.still_buf_head));
	INIT_LIST_HEAD(&(g_camdrv_status.video_buf_head));
	INIT_LIST_HEAD(&(g_camdrv_status.still_report_head));
	INIT_LIST_HEAD(&(g_camdrv_status.video_report_head));

	init_waitqueue_head(&(g_camdrv_status.camera_wait_q));
	g_camdrv_status.task_waiting = 0;

	/* set default value */
	cam_ctx->capture_mode = CAMERA_MODE_VIDEO;
	pxa_camera_set_mode(&g_camdrv_status, cam_ctx->capture_mode);

	INIT_LIST_HEAD(&pc_head.page_list);
	pc_head.page_count = 0;
	spin_lock_init(&pc_head.lock);
	init_MUTEX(&buf_list_sem);
	spin_lock_init(&report_list_lock);
	spin_lock_init(&cam_queue_lock);

	cam_ctx->video_capture_width = WIDTH_DEFT;
	cam_ctx->video_capture_height = HEIGHT_DEFT;
	cam_ctx->video_capture_scale = CAMERA_CAPTURE_SCALE_DISABLE;
	cam_ctx->video_capture_input_format =
		CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR;
	cam_ctx->video_capture_output_format =
		CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR;

	cam_ctx->still_capture_width = WIDTH_DEFT;
	cam_ctx->still_capture_height = HEIGHT_DEFT;
	cam_ctx->still_capture_scale = CAMERA_CAPTURE_SCALE_DISABLE;
	cam_ctx->still_capture_input_format =
		CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR;
	cam_ctx->still_capture_output_format =
		CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR;

	cam_ctx->frame_rate = FRAMERATE_DEFT;
	cam_ctx->ci_reg_base = (unsigned int)(&(CICR0));

	SET_DEFAULT_SENSOR(cam_ctx);

	/*
	 * we alloc histogram/LUT buffer and its dma descriptor here
	 */
	cam_ctx->histogram_lut_buffer_virtual = dma_alloc_coherent(NULL,
			1024, &handle, GFP_KERNEL);

	if (!cam_ctx->histogram_lut_buffer_virtual) {
		pr_debug("Can't get memory for histogram buffer\n");
		goto alloc_histogram_buffer_error;
	} else
		cam_ctx->histogram_lut_buffer_physical = handle;

	cam_ctx->histogram_lut_dma_descriptors_virtual =
		dma_alloc_coherent(NULL, 16, &handle, GFP_KERNEL);

	if (!cam_ctx->histogram_lut_dma_descriptors_virtual) {
		pr_debug("Can't get memory for phantom buffer\n");
		goto alloc_histogram_dma_desc_error;
	} else
		cam_ctx->histogram_lut_dma_descriptors_physical = handle;

	/*
	 * We alloc phantom buffer here. The buffer node (list) will be
	 * alloc when application start capturing.
	 */
	cam_ctx->phantom_buffer_virtual = dma_alloc_coherent(NULL,
			PHANTOM_BUFFER_SIZE,
			&handle,
			GFP_KERNEL);
	if (!cam_ctx->phantom_buffer_virtual) {
		pr_debug("Can't get memory for phantom buffer\n");
		goto alloc_phantom_buffer_error;
	} else {
		cam_ctx->phantom_buffer_physical = handle;
	}

	status = camera_init(cam_ctx);
	if (status) {
		goto camera_init_error;
	}

	/* set interrupt mask */
	mcam_set_interrupt_mask(cam_ctx, INT_MASK);

	/* empty the report list */
	while (!list_empty(g_camdrv_status.p_report_head)) {
		list_del_init(g_camdrv_status.p_report_head->next);
	}

	/* empty the head list */
	while (!list_empty(g_camdrv_status.p_buf_head)) {
		list_del_init(g_camdrv_status.p_buf_head->next);
	}

	return status;

camera_init_error:
	dma_free_coherent(NULL, PHANTOM_BUFFER_SIZE,
			(void *)cam_ctx->phantom_buffer_virtual,
			cam_ctx->phantom_buffer_physical);

alloc_phantom_buffer_error:

	dma_free_coherent(NULL, 16,
			(void *)cam_ctx->
			histogram_lut_dma_descriptors_virtual,
			cam_ctx->histogram_lut_dma_descriptors_physical);

alloc_histogram_dma_desc_error:

	dma_free_coherent(NULL, 1024,
			(void *)cam_ctx->histogram_lut_buffer_virtual,
			cam_ctx->histogram_lut_buffer_physical);

alloc_histogram_buffer_error:
	return status;
}

static int pxa_camera_close(struct inode *inode, struct file *file)
{
	p_camera_context_t cam_ctx = g_camera_context;
	struct page *page;
	unsigned long flags;

	g_camdrv_status.driver_opened = 0;

	if (*(g_camdrv_status.p_capture_started)) {
		pxa_camera_ioctl_streamoff(&g_camdrv_status);
	}

	camera_deinit(cam_ctx);

	if (cam_ctx->phantom_buffer_virtual) {
		dma_free_coherent(NULL, PHANTOM_BUFFER_SIZE,
				(void *)cam_ctx->phantom_buffer_virtual,
				cam_ctx->phantom_buffer_physical);
	}
	if (cam_ctx->histogram_lut_dma_descriptors_virtual) {
		dma_free_coherent(NULL, 16,
				(void *)cam_ctx->
				histogram_lut_dma_descriptors_virtual,
				cam_ctx->
				histogram_lut_dma_descriptors_physical);
	}

	if (cam_ctx->histogram_lut_buffer_virtual) {
		dma_free_coherent(NULL, 1024,
				(void *)cam_ctx->histogram_lut_buffer_virtual,
				cam_ctx->histogram_lut_buffer_physical);
	}

	/* empty the report list */
	while (!list_empty(&(g_camdrv_status.still_report_head))) {
		list_del_init(g_camdrv_status.still_report_head.next);
	}
	while (!list_empty(&(g_camdrv_status.video_report_head))) {
		list_del_init(g_camdrv_status.video_report_head.next);
	}

	camera_free_buffer_list(CAMERA_MODE_STILL, 1);
	camera_free_buffer_list(CAMERA_MODE_VIDEO, 1);

	/* Free all the camera page cache */
	while (!list_empty(&pc_head.page_list)) {
		spin_lock_irqsave(&pc_head.lock, flags);
		page = list_entry(pc_head.page_list.next, struct page, lru);
		list_del(&page->lru);
		spin_unlock_irqrestore(&pc_head.lock, flags);

		set_page_count(page, 1);
		ClearPageReserved(page);
		__free_page(page);
	}
	pc_head.page_count = 0;

	return 0;
}

static ssize_t pxa_camera_read(struct file *file, char __user * buf,
		size_t count, loff_t * ppos)
{
	p_camera_context_t cam_ctx = g_camera_context;
	ssize_t ret = 0;
	struct buf_node *buf_node;
	char __user *tmp_buf = buf;
	unsigned long flags;

	if (!*(g_camdrv_status.p_capture_started)) {
		if (pxa_camera_ioctl_streamon(&g_camdrv_status)) {
			return -EIO;
		}
	}

	spin_lock_irqsave(&report_list_lock, flags);
	if (list_empty(g_camdrv_status.p_report_head)) {
		int ret;
		g_camdrv_status.task_waiting = 1;
		spin_unlock_irqrestore(&report_list_lock, flags);
		ret = wait_event_interruptible(g_camdrv_status.camera_wait_q,
				!list_empty(g_camdrv_status.
					p_report_head));
		if (ret)
			return ret;
	} else {
		spin_unlock_irqrestore(&report_list_lock, flags);
	}

	spin_lock_irqsave(&report_list_lock, flags);
	buf_node =
		list_entry(g_camdrv_status.p_report_head->next, struct buf_node,
				report_head);
	if (!buf_node) {
		ret = -EINVAL;
		spin_unlock_irqrestore(&report_list_lock, flags);
		goto out;
	}

	list_del(&buf_node->report_head);
	spin_unlock_irqrestore(&report_list_lock, flags);

	if (copy_to_user(tmp_buf, buf_node->Y_vaddr, buf_node->fifo0_size)) {
		ret = -EFAULT;
		goto out;
	}

	tmp_buf += buf_node->fifo0_size;

	if (buf_node->fifo1_size) {
		if (copy_to_user
				(tmp_buf, buf_node->Cb_vaddr, buf_node->fifo1_size)) {
			ret = -EFAULT;
			goto out;
		}
		tmp_buf += buf_node->fifo1_size;
	}

	if (buf_node->fifo2_size) {
		if (copy_to_user
				(tmp_buf, buf_node->Cr_vaddr, buf_node->fifo2_size)) {
			ret = -EFAULT;
			goto out;
		}
	}

	ret =
		buf_node->fifo0_size + buf_node->fifo1_size + buf_node->fifo2_size;

out:
	camera_submit_buffer(cam_ctx, buf_node->buf_index);

	return ret;
}

static int pxa_camera_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long iterators = (unsigned long)vma->vm_start;
	unsigned long size = 0;
	int i, ret = 0;
	struct buf_node *buf_node;
	unsigned int offset;
	unsigned int buf_index;

	offset = vma->vm_pgoff << PAGE_SHIFT;
	buf_index = offset / camera_get_buffer_size();

	down_interruptible(&buf_list_sem);
	buf_node = camera_get_buffer_from_index(buf_index);
	for (i = 0; i < buf_node->page_num; i++) {
		if (remap_pfn_range(vma,
				iterators,
				(__pa(page_address(buf_node->pages[i]))) >>
				PAGE_SHIFT, PAGE_SIZE,
				pgprot_noncached(PAGE_SHARED)
#ifdef	CACHEABLE_BUFFER
				| L_PTE_CACHEABLE | L_PTE_BUFFERABLE
#endif
				)) {
			ret = -EFAULT;
			goto remap_page_error;
		}
		size += PAGE_SIZE;
		iterators += PAGE_SIZE;
	}
	up(&buf_list_sem);

	return ret;

remap_page_error:
	do_munmap(vma->vm_mm, vma->vm_start, size);
	return ret;
}

static unsigned int pxa_camera_poll(struct file *file, poll_table * wait)
{
	unsigned long flags;

	if (!*(g_camdrv_status.p_capture_started)) {
		if (pxa_camera_ioctl_streamon(&g_camdrv_status)) {
			return -EIO;
		}
	}

	spin_lock_irqsave(&report_list_lock, flags);
	if (!list_empty(g_camdrv_status.p_report_head)) {
		spin_unlock_irqrestore(&report_list_lock, flags);
		return POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&report_list_lock, flags);

	g_camdrv_status.task_waiting = 1;
	poll_wait(file, &(g_camdrv_status.camera_wait_q), wait);

	spin_lock_irqsave(&report_list_lock, flags);
	if (!list_empty(g_camdrv_status.p_report_head)) {
		spin_unlock_irqrestore(&report_list_lock, flags);
		return POLLIN | POLLRDNORM;
	} else {
		spin_unlock_irqrestore(&report_list_lock, flags);
		return 0;
	}
}

static int pxa_camera_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long param)
{
	return video_usercopy(inode, file, cmd, param, camera_do_ioctl);
}

static void pxa_camera_release(struct video_device *dev)
{
}

#ifdef CONFIG_PM
/*
 * Suspend the Camera Module.
 */
static int pxa_camera_suspend(struct device *dev, u32 state, u32 level)
{
	p_camera_context_t cam_ctx = g_camera_context;

	g_camdrv_status.suspended = 1;

	if (!(g_camdrv_status.driver_opened)) {
		if (g_camdrv_status.i2c_inited) {
			g_camdrv_status.i2c_inited = 0;
		}
		return 0;
	}

	mcam_suspend(cam_ctx);
	disable_irq(IRQ_CAMERA);
	if (g_camdrv_status.i2c_inited) {
		g_camdrv_status.i2c_inited = 0;
	}

	return 0;
}

/*
 * Resume the Camera Module.
 */
static int pxa_camera_resume(struct device *dev, u32 level)
{
	p_camera_context_t cam_ctx = g_camera_context;
	struct buf_node *buf_node;
	unsigned long flags;
	struct pxa_camera_driver_status_t *p_camdrv_status;

	int buf_type;

	g_camdrv_status.suspended = 0;

	if (!(g_camdrv_status.driver_opened)) {
		if (!g_camdrv_status.i2c_inited) {
			g_camdrv_status.i2c_inited = 1;
		}
		return 0;
	}

	p_camdrv_status = &(g_camdrv_status);

	buf_type = (cam_ctx->capture_mode == CAMERA_MODE_STILL) ?
		STILL_CAPTURE_BUFFER : VIDEO_CAPTURE_BUFFER;
	switch (level) {
		case RESUME_POWER_ON:
			/* init to output 3.2v temporarily.
			 * should be done by system
			 */
			mhn_pmic_set_voltage(VCC_CAMERA_ANA, 3200);

			/* enable QCI clock  */
			pxa_set_cken(CKEN_CAMERA, 1);

			spin_lock_irqsave(&report_list_lock, flags);
			/* Empty report head and put its buf into head queue */
			while (!list_empty(p_camdrv_status->p_report_head)) {
				buf_node =
					list_entry(g_camdrv_status.p_report_head->next,
							struct buf_node, report_head);
				list_del_init(&buf_node->report_head);
				if (camera_submit_buffer(cam_ctx, buf_node->buf_index)) {
					spin_unlock_irqrestore(&report_list_lock,
							flags);
					return -EIO;
				}
			}
			spin_unlock_irqrestore(&report_list_lock, flags);

			mcam_resume(cam_ctx);

			mcam_set_interrupt_mask(cam_ctx, INT_MASK);
			enable_irq(IRQ_CAMERA);

			if (cam_ctx->dma_running) {
				spin_lock_irqsave(&report_list_lock, flags);
				if (list_empty(g_camdrv_status.p_report_head)) {
					g_camdrv_status.task_waiting = 1;
					spin_unlock_irqrestore(&report_list_lock,
							flags);
					wait_event_interruptible(g_camdrv_status.
							camera_wait_q,
							!list_empty
							(g_camdrv_status.
							 p_report_head));
				} else {
					spin_unlock_irqrestore(&report_list_lock,
							flags);
				}
			}

		default:
			break;
	}
	return 0;
}

#endif

static int __devinit pxa_camera_probe(struct device *dev)
{

	g_camdrv_status.i2c_inited = 1;
	g_camdrv_status.suspended = 0;

	/* allocte camera context */
	g_camera_context = kzalloc(sizeof(camera_context_t), GFP_KERNEL);
	if (!g_camera_context) {
		pr_debug("Can't allocate buffer for" "camera control structure \n");
		return -ENOMEM;
	}

	/* allocte camera functions context */
	g_camera_context->camera_functions = kzalloc(sizeof(camera_function_t),
			GFP_KERNEL);
	if (!g_camera_context->camera_functions) {
		pr_debug("Can't allocate buffer for"
				"camera functions structure \n");
		goto malloc_camera_functions_err;
		return -ENOMEM;
	}

	if (video_register_device(&vd, VFL_TYPE_GRABBER, pxa_camera_minor) < 0) {
		goto register_video_error;
	}

	pr_info("PXA_CAMERA: PXA Camera driver loaded for /dev/video%d\n",
			pxa_camera_minor);

	g_camdrv_status.driver_opened = 0;
	/* request irq */
	if (request_irq(IRQ_CAMERA, pxa_camera_irq, 0, "PXA Camera", &vd)) {
		pr_debug("PXA_CAMERA: Camera interrupt register failed \n");
		goto register_video_error;
	}
	disable_irq(IRQ_CAMERA);


	return 0;

register_video_error:
	if (g_camera_context->camera_functions) {
		kfree(g_camera_context->camera_functions);
	}

malloc_camera_functions_err:
	if (g_camera_context) {
		kfree(g_camera_context);
	}
	return -EIO;
}

static int __devexit pxa_camera_remove(struct device *dev)
{

	g_camdrv_status.i2c_inited = 0;

	free_irq(IRQ_CAMERA, &vd);
	video_unregister_device(&vd);
	kfree(g_camera_context->camera_functions);
	kfree(g_camera_context);
	return 0;
}

static struct device_driver pxa_camera_driver = {
	.name 		= "pxa2xx-camera",
	.bus 		= &platform_bus_type,
	.probe 		= pxa_camera_probe,
	.remove 	= __devexit_p(pxa_camera_remove),
#ifdef CONFIG_PM
	.suspend 	= pxa_camera_suspend,
	.resume 	= pxa_camera_resume,
#endif
};

static int __devinit pxa_camera_init(void)
{
	return driver_register(&pxa_camera_driver);
}

static void __exit pxa_camera_exit(void)
{
	return driver_unregister(&pxa_camera_driver);
}

module_init(pxa_camera_init);
module_exit(pxa_camera_exit);

MODULE_DESCRIPTION("Zylonite/Monahans Camera Interface driver");
MODULE_LICENSE("GPL");
