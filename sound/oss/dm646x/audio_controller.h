#ifndef __AUDIO_CONTROLLER_H_
#define __AUDIO_CONTROLLER_H_
/*
 * audio_controller.h - Common audio handling. Replaces the old davinci-audio.h
 *                      to make things more generic, independent of ASP and
 * 			codec.
 *
 * Copyright (C) 2007  Texas Instruments, India
 * Author:Nirmal Pandey <n-pandey@ti.com>,
 *        Suresh Rajashekara <suresh.r@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */


/* Requires dma.h */
#include <asm/arch/dma.h>
#include <asm/atomic.h>
#include <asm/arch/davinci-audio-config.h>

/* Structure related to the audio buffer management */
typedef struct {
	/* current offset of data within the Fragment */
    s32 		offset;
	/* points to actual data buffer */
    s8 			*data;
	/* physical buffer address */
    dma_addr_t 	dma_addr;
	/* DMA reference count. Number of buffers DMA'd */
    s32 		dma_ref;
	/* owner for buffer allocation, contain size when true */
    s32 		master;
} audio_buf_t;

/* Structure describing the data stream related information */
typedef struct {
    s8 			*id;			/* identification string */
    audio_buf_t	*buffers; /* point32er to audio buffer structures */
    u32 		usr_head;		/* user fragment index */
    u32 		dma_head;		/* DMA fragment index to go */
    u32 		dma_tail; /* DMA fragment index to complete */
    u32 		fragsize;		/* fragment i.e. buffer size */
    u32 		nbfrags;	/* nbr of fragments i.e. buffers */
    u32 		pending_frags;	/* Fragments sent to DMA */
    s32 		dma_dev;		/* device identifier for DMA */
    u32 		prevbuf;	/* Prev pending frag size sent to DMA */
    s8 			started;/* to store if the chain was started or not */
    s32 		dma_q_head;		/* DMA Channel Q Head */
    s32 		dma_q_tail;		/* DMA Channel Q Tail */
    s8 			dma_q_count;	/* DMA Channel Q Count */
    s8 			in_use;			/*  Is this is use? */
    s32 		master_ch;
    s32 		*lch; /*  Chain of channels this stream is linked to */
    s32 		input_or_output;/* Direction of this data stream */
    s32 		bytecount;		/* nbr of processed bytes */
    s32 		fragcount;	/* nbr of fragment transitions */
    struct completion wfc;	/* wait for "nbfrags" fragment completion */
    wait_queue_head_t wq;		/* for poll */
    s32 		dma_spinref;	/* DMA is spinning */
    s32 		mapped:1;		/* mmap()'ed buffers */
    s32 		active:1;		/* actually in progress */
    s32 		stopped:1;	/* might be active but stopped */
    s32 		spin_idle:1;/* have DMA spin on zeros when idle */
    s32 		dma_started;	/* to store if DMA was started or not */
    s32		        asp_tx_started;
    s32		        asp_rx_started;
    u8                  device_instance; /* For storing the minor number of the
					    device to refer to in
					    audio_ctrl_open */
    atomic_t            playing_null;
    s32                 null_lch;
    atomic_t            in_write_path;
} audio_stream_t;

/* State structure for one opened instance */
typedef struct {
    struct module 	*owner;				/* Codec module ID */
    audio_stream_t 	*output_stream;
    audio_stream_t 	*input_stream;
    s32 		rd_ref:1;	/* open reference for recording */
    s32 		wr_ref:1;	/* open reference for playback */
    s32 		need_tx_for_rx:1;/* if data must be sent while receiving
					  */
    void		*data;
    void		(*hw_init) (void *);
    void		(*hw_shutdown) (void *);
    s32         	(*client_ioctl) (struct inode *, struct file *, u32,
					 ulong);
    s32 		(*hw_probe) (void);
    void 		(*hw_remove) (void);
    void 		(*hw_cleanup) (void);
    s32 		(*hw_suspend) (void);
    s32 		(*hw_resume) (void);
    struct pm_dev 	*pm_dev;
    struct compat_semaphore sem; /* to protect against races in attach() */
} audio_state_t;


#ifdef AUDIO_PM
void audio_ldm_suspend(void *data);

void audio_ldm_resume(void *data);

#endif

/* Register a Codec using this function */
extern int audio_register_codec(audio_state_t *codec_state);
/* Un-Register a Codec using this function */
extern int audio_unregister_codec(audio_state_t *codec_state);
/* Function to provide fops of davinci audio driver */
extern struct file_operations *audio_get_fops(void);
/* Function to initialize the device info for audio driver */
extern int audio_dev_init(void);
/* Function to un-initialize the device info for audio driver */
void audio_dev_uninit(void);

#endif /* __AUDIO_CONTROLLER_H_ */
