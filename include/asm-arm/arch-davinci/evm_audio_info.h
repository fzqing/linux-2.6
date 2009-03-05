#ifndef __EVM_AUDIO_INFO_H__
#define __EVM_AUDIO_INFO_H__

/*
 * evm_audio_info.h - Header file which defines the layout of ASP and codec on
 *                    an EVM.
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

#include <linux/module.h>
#include <linux/fs.h>

#define MAX_SERIALIZER_COUNT 16
#define MAX_ASP		2	/* Maximum number of ASP's */
#define MAX_CODEC	1	/* Highest number of codec's under any of the
				   #ASP */

#define ASP_SLAVE           0
#define ASP_MASTER          1

#define TRUE     1
#define FALSE    0

#define ERR_INVALID_CODEC_ID              100
#define ERR_INVALID_CODEC_FOPS_OR_INFO    101
#define ERR_INVALID_ASP_ID                102
#define ERR_INVALID_ASP_FOPS_OR_INFO      103
#define ERR_DUP_CODEC_ID                  104
#define ERR_INVAL_ASP_CNT                 105
#define ERR_DUP_ASP_ID                    106
#define ERR_NULL_AUDIO_CONFIG		  107
#define ERR_AUDIO_DEVICE_NOT_FOUND	  108
#define ERR_DIT_ENABLED_WITH_CODECS       109

#define TX_SUPPORT  0x01
#define RX_SUPPORT  0x02

typedef struct asp_info {
	/* Set -1 to indicate an unsupported or uninitialized feature */
    s8 id;
    u32 reg_base;
    u32 tx_data_port;
    u32 rx_data_port;
    s8 tx_dma_evt;
    s8 rx_dma_evt;
    s8 tx_irq;
    s8 rx_irq;
    s8 serializer_count;
    s8 left_dit_channel_count;
    s8 right_dit_channel_count;
    const char *lpsc;		/* Initialize this to NULL if there is no LPSC
				   domain to be initialized. Sometimes this init
				   of the power domain could be done by some
				   other portion of the BSP (eg: U-boot) or
				   sometime the module might not need any LPSC
				   to be initializd. */
    s8 flags;	                /* Useful to indicate things like feature
				   supported */
}asp_info_t;

typedef struct codec_info {
    const char *lpsc;		/* Initialize this to NULL if there is no LPSC
				   domain to be initialized. Sometimes this init
				   of the power domain could be done by some
				   other portion of the BSP (eg: U-boot) or
				   sometime the module might not need any LPSC
				   to be initializd. */
    s8 flags;			/* Useful to indicate things like feature
				   supported */
}codec_info_t;

/* Values for audio codec types */
enum codec_type {
    CODEC_UNKNOWN = 0,
    CODEC_AIC23,
    CODEC_AIC32,
    CODEC_AIC33
};

typedef struct audio_codec {
    u8 id;			/* Codec ID */
    enum codec_type type;	/* Type of Codec. Choose from list. Add any new
				   codec to the list */
    u8 is_configured:1;
    u8 is_initialized:1;
    u8 is_in_use:1;

    struct codec_file_operations *c_op;	/* File operations needed for the
					   central controller to access some of
					   the functionalities of the codec. */
    struct codec_info *c_info;	/* Hardware information */

    u8 device_instance;		/* This is the number that is returned by the
				 * register_sound_dsp function. This will be
				 * updated by the code. DON'T UPDATE THIS
				 * MANUALLY. Even if you update the value will
				 * be overwritten. */
    u8 pid;
	    /* It was planned to store the audio_state variable here. Just
	     * including one variable here was causing some glibc error (****
	     * glibc detected *** double free or corruption: 0x00012008 ***)
	     * . So its been moved to asp structure. Keeping here would be a
	     * neat way as it easily accomidates maintaing different
	     * audio_states for each codec else we might need to make the
	     * audio_state in the ASP structure an array to store the different
	     * audio_states*/
	/* void *audio_state; */
}audio_codec_t;

/* Values for ASP type */
enum asp_type {
    MCASP = 1,
    MCBSP
};

enum asp_mode {
    MASTER = 0,
    SLAVE
};

typedef struct audio_serial_port {
    u8 id;			/* ID of the ASP */
    enum asp_type type;	/* McASP/McBSP or any other new ASP. Add new asp
				   to the list above. */
    enum asp_mode master_mode;		/* 0-Slave, 1-Master */

    u8 is_configured:1;
    u8 is_initialized:1;
    u8 is_in_use:1;

    struct asp_file_operations *a_op; /* File operations which might need to be
					 called by the central controller. */
    struct asp_info *a_info;	/* Hardware Information */
    u8 dit_enabled;
    u8 need_tx_for_rx; 		/* Some cases, you might need to run Dummy TX
				   for RX. Enable this in such cases*/
    u8 codec_count;		/* Number of Audio codec's connected to the ASP
				 */
    audio_codec_t codec[MAX_CODEC];	/* Pointer to the codec/s */
    void *audio_state;		/* Make it an array if ever there are more then
				   once codec per ASP */
}audio_serial_port_t;

typedef struct evm_audio_info {
    u8 asp_count;		/* Number of serial ports (mcasp/mcbsp's) */
    struct audio_serial_port asp[MAX_ASP];      /* Pointer to the ASP/s */
} evm_audio_info_t;

s8 get_parent_id (u8 id);

typedef struct asp_file_operations {
    s8 (*init)(u8 asp_count, void *info);
    s8 (*de_init)(u8 id);
    s8 (*configure)(u8 id, void *cfg);
    s8 (*unconfigure)(u8 id);
    s8 (*configure_reg)(u8 mcasp_id, void *cfg);
    s8 (*start)(u8 id);
    s8 (*stop)(u8 id);
    s8 (*start_tx)(u8 id);
    s8 (*stop_tx)(u8 id);
    s8 (*start_rx)(u8 id);
    s8 (*stop_rx)(u8 id);
    void *(*get_info)(u8 id);
    void *(*get_config)(u8 id);
    struct clk *(*get_clock)(u8 id);
    s32 (*set_sample_rate)(u8 id, u32 sample_rate);
    s32 (*get_sample_rate)(u8 id);
    s8 (*get)(u8 id);
    s8 (*put)(u8 id);
}asp_fops_t;


typedef struct codec_file_operations {
    s8 (*init)(void);
    s8 (*de_init)(u8 id);
    s8 (*configure)(u8 id, void *ptr);
    s8 (*unconfigure)(u8 id);
    s8 (*start)(u8 id);
    s8 (*stop)(u8 id);
    s8 (*get_info)(u8 id);
    s8 (*get_config)(u8 id);
    int (*ioctl)(struct inode *inode, struct file *file, uint cmd, ulong arg);
}codec_fops_t;

s8 validate_evm_audio_config (evm_audio_info_t **info);
s8 find_audio_device ( u8 dev_instance, u8 *asp_id_ptr, u8 *codec_id_ptr );
unsigned long get_rx_dma_src_address (u8 device_instance);
unsigned long get_tx_dma_dest_address (u8 device_instance);
asp_fops_t *get_asp_fops(u8 device_instance);
codec_fops_t *get_codec_fops(u8 device_instance);
u8 is_dit_enabled (u8 device_instance);
#endif /* __EVM_AUDIO_INFO_H__ */
