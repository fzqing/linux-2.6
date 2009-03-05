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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
/* davincihd_capture.h */

#ifndef DAVINCIHD_CAPTURE_H
#define DAVINCIHD_CAPTURE_H

#ifdef __KERNEL__

/* Header files */
#include <linux/videodev.h>
#include <linux/videodev2.h>
#include <media/video-buf.h>
#include <media/davinci/vid_decoder_if.h>
#include <media/davinci/vpif.h>

#define VPIF_CAPTURE_NUM_DECODERS        4

/* Macros */
#define VPIF_MAJOR_RELEASE              0
#define VPIF_MINOR_RELEASE              0
#define VPIF_BUILD                      1
#define VPIF_CAPTURE_VERSION_CODE       ((VPIF_MAJOR_RELEASE<<16) | \
	(VPIF_MINOR_RELEASE<<8)  | \
	VPIF_BUILD)

#define VPIF_VALID_FIELD(field)         (((V4L2_FIELD_ANY == field) || \
	(V4L2_FIELD_NONE == field)) || \
	(((V4L2_FIELD_INTERLACED == field) || \
	(V4L2_FIELD_SEQ_TB == field)) || \
	(V4L2_FIELD_SEQ_BT == field)))

#define VPIF_VALID_BUFFER_TYPE(buftype)	{ \
			(V4L2_BUF_TYPE_VIDEO_CAPTURE == buftype) || \
			(V4L2_BUF_TYPE_VBI_CAPTURE == buftype) || \
			(V4L2_BUF_TYPE_SLICED_VBI_CAPTURE == buftype) || \
			(V4L2_BUF_TYPE_HBI_CAPTURE == buftype)}

#define VPIF_CAPTURE_MAX_DEVICES	2
#define VPIF_MAX_DECODER_STD		50
#define VPIF_TIMER_COUNT		5
#define VPIF_SLICED_BUF_SIZE		256
#define VPIF_SLICED_MAX_SERVICES	3
#define VPIF_HBI_INDEX			2
#define VPIF_VBI_INDEX			1
#define VPIF_VIDEO_INDEX		0

#define VPIF_NUMBER_OF_OBJECTS		3

/* Macros */
#define ISNULL(p)       ((NULL) == (p))
#define ISALIGNED(a)    (0 == (a%8))
#define ISDECODERCMD(cmd)       ((VPIF_CMD_S_DECODER_PARAMS==cmd) || \
                                        (VPIF_CMD_G_DECODER_PARAMS==cmd))
/* enumerated data types */
/* Enumerated data type to give id to each device per channel */
enum vpif_channel_id {
	VPIF_CHANNEL0_VIDEO = 0,	/* Channel0 Video */
	VPIF_CHANNEL1_VIDEO,	/* Channel1 Video */
};

/* structures */
/* Table to keep track of the standards supported in all the decoders */
struct vpif_decoder_std_tbl {
	u8 dec_idx;
	u8 std_idx;
	v4l2_std_id std;
};

struct video_obj {
	v4l2_std_id std;	/* Currently selected or default
				   standard */
	enum v4l2_field buf_field;

	u32 latest_only;	/* indicate whether to return
				   most recent displayed frame
				   only */
	struct vpif_stdinfo std_info;	/*Keeps track of the information
					   about the standard */
	u32 input_idx;		/* This is to track the last
				   input that is passed to 
				   application */
	struct vpif_decoder_std_tbl std_tbl[VPIF_MAX_DECODER_STD];
	int count_std;
};

struct vbi_obj {
	u8 timer_counter;	/* Counter to synchronize 
				   access to the timer */
	struct tasklet_struct vbi_tasklet;	/* An object of tasklets 
						   structure which is used 
						   for read  sliced vbi data 
						   from decoders */
	u8 num_services;
};

struct common_obj {
	/* Buffer specific parameters */
	u8 *fbuffers[VIDEO_MAX_FRAME];	/* List of buffer pointers for
					   storing frames */
	u32 numbuffers;		/* number of buffers in fbuffers */
	struct videobuf_buffer *curFrm;	/* Pointer pointing to current
					   v4l2_buffer */
	struct videobuf_buffer *nextFrm;	/* Pointer pointing to current
						   v4l2_buffer */
	enum v4l2_memory memory;	/* This field keeps track of type
					   of buffer exchange mechanism 
					   user has selected */
	struct v4l2_format fmt;	/* Used to store pixel format */

	struct videobuf_queue buffer_queue;	/* Buffer queue used in
						   video-buf */
	struct list_head dma_queue;	/* Queue of filled frames */
	spinlock_t irqlock;	/* Used in video-buf */

	/* channel specifc parameters */
	struct semaphore lock;	/* lock used to access this
				   structure */
	u32 io_usrs;		/* number of users performing
				   IO */
	u8 started;		/* Indicates whether streaming
				   started */
	void (*set_addr) (unsigned long, unsigned long, unsigned long, unsigned long);	/* Function pointer to set 
											   the addresses */
	u32 ytop_off;		/* offset where Y top starts
				   from the starting of the
				   buffer */
	u32 ybtm_off;		/* offset where Y bottom starts
				   from the starting of the
				   buffer */
	u32 ctop_off;		/* offset where C top starts
				   from the starting of the
				   buffer */
	u32 cbtm_off;		/* offset where C bottom starts
				   from the starting of the
				   buffer */
	u32 width;		/* Indicates width of the image 
				   data or width of the vbi data */
	u32 height;		/* Indicates height of the image 
				   data or height of the vbi data */
};

struct channel_obj {
	/* V4l2 specific parameters */
	struct video_device *video_dev;	/* Identifies video device for
					   this channel */
	struct v4l2_prio_state prio;	/* Used to keep track of state of
					   the priority */
	u32 usrs;		/* number of open instances of
				   the channel */
	u32 field_id;		/* Indicates id of the field
				   which is being displayed */
	u8 initialized;		/* flag to indicate whether
				   decoder is initialized */

	enum vpif_channel_id channel_id;	/* Identifies channel */

	struct decoder_device *decoder[VPIF_CAPTURE_NUM_DECODERS];
	/* pointer to decoder device
	   structures for this channel */
	u8 numdecoders;		/* indicates number of decoders
				   registered to VPIF-V4L2 */
	u8 current_decoder;	/* Index of the currently
				   selected decoder */
	u8 default_decoder;	/* Index of the defalut decoder,
				   set as per boot args at 
				   channel creation time */
	struct vpif_params vpifparams;
	struct common_obj common[VPIF_NUMBER_OF_OBJECTS];
	struct video_obj video;
	struct vbi_obj vbi;
};

/* File handle structure */
struct vpif_fh {
	struct channel_obj *channel;	/* pointer to channel object for
					   opened device */
	u8 io_allowed[VPIF_NUMBER_OF_OBJECTS];	/* Indicates whether this 
						   file handle is doing IO */
	enum v4l2_priority prio;	/* Used to keep track priority of
					   this instance */
	u8 initialized;		/* Used to indicate channel is 
				   initialize or not */
};

/* vpif device structure */
struct vpif_device {
	struct channel_obj *dev[VPIF_CAPTURE_NUM_CHANNELS];
};

struct vpif_config_params {
	u8 min_numbuffers;
	u8 numbuffers[VPIF_CAPTURE_NUM_CHANNELS];
	s8 device_type;
	u32 min_bufsize[VPIF_CAPTURE_NUM_CHANNELS];
	u32 channel_bufsize[VPIF_CAPTURE_NUM_CHANNELS];
	u8 default_device[VPIF_CAPTURE_NUM_CHANNELS];
	u8 max_device_type;
};
/* Struct which keeps track of the line numbers for the sliced vbi service */
struct vpif_service_line {
	u16 service_id;
	u16 service_line[2];
};
#endif				/* End of __KERNEL__ */

/* IOCTLs */
#define VPIF_S_VPIF_PARAMS   _IOW('V', BASE_VIDIOC_PRIVATE+1, \
					struct vpif_params)
#define VPIF_G_VPIF_PARAMS   _IOR('V', BASE_VIDIOC_PRIVATE+2, \
					struct vpif_params)
#define VPFE_S_MT9T001_PARAMS _IOW('V', BASE_VIDIOC_PRIVATE+3, \
					void *)
#define VPFE_G_MT9T001_PARAMS _IOR('V', BASE_VIDIOC_PRIVATE+4, \
					void *)
#define VPFE_CMD_CONFIG_TVP5146 _IOW('V',BASE_VIDIOC_PRIVATE + 5, \
					void *)
#define VPIF_CMD_G_DECODER_PARAMS _IOR('V',BASE_VIDIOC_PRIVATE+8, \
					void *)
#define VPIF_CMD_S_DECODER_PARAMS _IOW('V',BASE_VIDIOC_PRIVATE+9, \
					void *)

#endif				/* DAVINCIHD_CAPTURE_H */
