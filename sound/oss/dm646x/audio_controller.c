/*
 * audio_controller.c
 *
 * Copyright (C) 2007  Texas Instruments, India
 * Author:Nirmal Pandey <n-pandey@ti.com>,
 *        Suresh Rajashekara <suresh.r@ti.com>
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/completion.h>
#include <linux/semaphore.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/hardware.h>

#include <asm/arch/evm_audio_info.h>
#include <asm/arch/davinci-audio-config.h>
#include "audio_controller.h"
#include "davinci-audio-dma-intfc.h"

static unsigned int audio_ctlr_poll(struct file *file, struct poll_table_struct
				    *wait);

static loff_t audio_ctlr_llseek(struct file *file, loff_t offset, int origin);

static int audio_ctlr_open(struct inode *inode, struct file *file);

static int audio_ctlr_write(struct file *file, const char *buffer,
			    size_t count, loff_t *ppos);

static int audio_ctlr_read(struct file *file, char *buffer, size_t count,
			   loff_t *ppos);

static int audio_ctlr_ioctl(struct inode *inode, struct file *file, uint cmd,
			    ulong arg);
static int audio_ctlr_release(struct inode *inode, struct file *file);


static struct file_operations audio_controller_fops = {
	.open = audio_ctlr_open,
	.write = audio_ctlr_write,
	.release = audio_ctlr_release,
	.read = audio_ctlr_read,
	.ioctl = audio_ctlr_ioctl,
	.poll = audio_ctlr_poll,
	.llseek = audio_ctlr_llseek,
	.owner = THIS_MODULE
};

extern struct file_operations davinci_mixer_fops;

#define CONFIGURED_AS_INACTIVE  0
#define CONFIGURED_AS_TX        1
#define CONFIGURED_AS_RX        2

static evm_audio_info_t *evm_aud_info;
/* Nirmal: Introducing new global. This may be required if we want to support
   different sampling frequencies for DIT. As of now let it be 4 */
int acount = 4;

/* This audio config is for each ASP */
static audio_config_t audio_cfg[] = {
	{
		.mcasp_id = 0,
		.mode = CODEC_IS_MASTER,
		.channel_size = 32,
		.serializer_count = 4,
		.serializer_mode[0] = CONFIGURED_AS_TX,
		.serializer_mode[1] = CONFIGURED_AS_RX,
		.serializer_mode[2] = CONFIGURED_AS_INACTIVE,
		.serializer_mode[3] = CONFIGURED_AS_INACTIVE,
		.loopback = 0,
		.amute = 0,
		.tdm_slots = 0,
		.sample_rate  = 48000,
	},
	{
		.mcasp_id = 1,
		.mode = CODEC_IS_SLAVE,
		.channel_size = 32,
		.serializer_count = 4,
		.serializer_mode[0] = CONFIGURED_AS_TX,
		.serializer_mode[1] = CONFIGURED_AS_INACTIVE,
		.serializer_mode[2] = CONFIGURED_AS_INACTIVE,
		.serializer_mode[3] = CONFIGURED_AS_INACTIVE,
		.loopback = 0,
		.amute = 0,
		.tdm_slots = 384, /* Configure this McASP for DIT */
		.sample_rate  = 48000,
	}
};

#define AUDIO_FRAGSIZE_DEFAULT 3072
#define AUDIO_NBFRAGS_DEFAULT 4

static s8 validate_file_mode ( u8 asp_id, struct file *file)
{
	/* If the file is opened in a mode which the ASP is not configured to
	 * operate in, return an error */
	u8 write = TRUE, read = TRUE;
	s8 ret_code;
	u8 cnt = 0;

	if ( (file->f_mode & FMODE_WRITE) ) {
		write = FALSE;
		for (cnt = 0; cnt < audio_cfg[asp_id].serializer_count; cnt++) {
			if (audio_cfg[asp_id].serializer_mode[cnt] ==
			    CONFIGURED_AS_TX) {
				write = TRUE;
				break;
			}
		}
	}

	if ( (file->f_mode & FMODE_READ) ) {
		read = FALSE;
		for (cnt = 0; cnt < audio_cfg[asp_id].serializer_count; cnt++) {
			if (audio_cfg[asp_id].serializer_mode[cnt] ==
			    CONFIGURED_AS_RX) {
				read = TRUE;
				break;
			}
		}
	}

	if ( (write == FALSE) || (read == FALSE) ) {
		ret_code = -1;
	} else {
		ret_code = 0;
	}

	return ret_code;
}

static audio_state_t *initialize_audio_state (u8 asp_id, u8 codec_id)
{
	audio_state_t *audio_state;
	audio_stream_t *output_stream;
	audio_stream_t *input_stream;
	audio_state_t *ret_code = NULL;

	audio_state = (audio_state_t *) kmalloc (sizeof(audio_state_t),
						 GFP_KERNEL);

	if ( audio_state == NULL ) {
		ret_code = NULL;
	} else {
		ret_code = audio_state;

		output_stream = (audio_stream_t *) kmalloc
			(sizeof(audio_stream_t), GFP_KERNEL);
		if ( output_stream == NULL ) {
			kfree(audio_state);
			ret_code = NULL;
		} else {
			input_stream = (audio_stream_t *) kmalloc
				(sizeof(audio_stream_t), GFP_KERNEL);
			if ( input_stream == NULL ) {
				kfree(audio_state);
				kfree(output_stream);
				ret_code = NULL;
			}
		}
	}

	if ( ret_code != NULL ) {
		memset (audio_state, 0, sizeof(audio_state_t));
		memset (output_stream, 0, sizeof(audio_stream_t));
		memset (input_stream, 0, sizeof(audio_stream_t));

		output_stream->id = "Output Stream";
		output_stream->dma_dev =
			evm_aud_info->asp[asp_id].a_info[asp_id].tx_dma_evt;
		output_stream->input_or_output = FMODE_WRITE;

		input_stream->id = "Input Stream";
		input_stream->dma_dev =
			evm_aud_info->asp[asp_id].a_info[asp_id].rx_dma_evt;
		input_stream->input_or_output = FMODE_READ;

		audio_state->output_stream = output_stream;
		audio_state->input_stream = input_stream;

		if (evm_aud_info->asp[asp_id].need_tx_for_rx == 1) {
			audio_state->need_tx_for_rx = 1;
		} else {
			audio_state->need_tx_for_rx = 0;
		}

		sema_init(&audio_state->sem, 1);
		audio_state->owner = THIS_MODULE;
		evm_aud_info->asp[asp_id].audio_state = (void *)audio_state;
	}

	return ret_code;
}

static loff_t audio_ctlr_llseek(struct file *file, loff_t offset, int origin)
{
	return -ESPIPE;
}

static unsigned int audio_ctlr_poll ( struct file *file, struct
				      poll_table_struct *wait )
{
	audio_state_t *state = file->private_data;
	audio_stream_t *is = state->input_stream;
	audio_stream_t *os = state->output_stream;
	unsigned int mask = 0;

	if (file->f_mode & FMODE_READ) {
		/* Start audio input if not already active */
		if (!is->active) {
			if (!is->buffers && audio_setup_buf(is)) {
				return -ENOMEM;
			}
			audio_prime_rx(state);
		}

		poll_wait(file, &is->wq, wait);
	}

	if (file->f_mode & FMODE_WRITE) {
		if (!os->buffers && audio_setup_buf(os)) {
			return -ENOMEM;
		}

		poll_wait(file, &os->wq, wait);
	}

	if (file->f_mode & FMODE_READ)
		if ((is->mapped && is->bytecount > 0) ||
		    (!is->mapped && is->wfc.done > 0))
			mask |= POLLIN | POLLRDNORM;

	if (file->f_mode & FMODE_WRITE)
		if ((os->mapped && os->bytecount > 0) ||
		    (!os->mapped && os->wfc.done > 0))
			mask |= POLLOUT | POLLWRNORM;

	return mask;
}

static int audio_ctlr_open ( struct inode *inode, struct file *file )
{
	u8 codec_id, asp_id;
	s8 status = 0;
	s8 ret_code = 0;
	int need_tx_dma;

	audio_state_t *state = NULL;
	audio_stream_t *os = NULL;
	audio_stream_t *is = NULL;

	u8 dev_instance = iminor (inode);

	/* With this given minor number, search the codec */
	status = find_audio_device ( dev_instance, &asp_id, &codec_id );

	if (status != 0) {
		printk(KERN_INFO " Device could not be opened, status %d",
		       status);
		ret_code = -ENODEV;
	}

	if ( validate_file_mode (asp_id, file) < 0 ) {
		printk (KERN_INFO "Device does not support the requested file \
		mode.\n");
		ret_code = -EINVAL;
	}

	if ( ret_code == 0 ) {
		if ( evm_aud_info->asp[asp_id].audio_state == NULL ) {
			state = initialize_audio_state (asp_id, codec_id);
			if (state == NULL) {
				ret_code = -ENOMEM;
			} else {
				os = state->output_stream;
				is = state->input_stream;
				os->device_instance = is->device_instance =
					dev_instance;
			}
		} else {
			state = (audio_state_t *)
				evm_aud_info->asp[asp_id].audio_state;
			os = state->output_stream;
			is = state->input_stream;
			os->device_instance = is->device_instance =
				dev_instance;
		}
	}

	/* Lock the module */
	if (!try_module_get(THIS_MODULE)) {
		printk (KERN_ERR "Failed to get module\n");
		return -ESTALE;
	}

	/* Lock the codec module */
	if (!try_module_get(state->owner)) {
		printk (KERN_ERR "Failed to get codec module\n");
		module_put(THIS_MODULE);
		return -ESTALE;
	}

	down(&state->sem);

	/* Reset the acount */
	acount = 4;

	if (ret_code != 0) {
		module_put(state->owner);
		module_put(THIS_MODULE);

		up(&state->sem);
		return ret_code;
	}

	if (((file->f_mode & FMODE_WRITE) && !os) ||
	    ((file->f_mode & FMODE_READ) && !is)) {
		up(&state->sem);
		return -ENODEV;
	}

	if (((file->f_mode & FMODE_WRITE) && state->wr_ref) ||
	    ((file->f_mode & FMODE_READ) && state->rd_ref)) {
		up(&state->sem);
		return -EBUSY;
	}

	if ((file->f_mode & FMODE_READ) && state->need_tx_for_rx && !os) {
		up(&state->sem);
		return -EINVAL;
	}

	/*
	 * Basic parameters are OK. Allocate the DMA channels
	 * for read and write
	 */

	/* Check the stream for which DMA is required */
	need_tx_dma = ((file->f_mode & FMODE_WRITE) ||
		      ((file->f_mode & FMODE_READ) &&  state->need_tx_for_rx));

	if (state->wr_ref || (state->rd_ref && state->need_tx_for_rx))
		need_tx_dma = 0;

	if (need_tx_dma) {
		DMA_REQUEST(ret_code, os, audio_dma_callback);
		if (ret_code < 0)
			printk(KERN_ERR
			       "Failed to request the DMA for playback\n");
	}

	if (file->f_mode & FMODE_READ) {
		DMA_REQUEST(ret_code, is, audio_dma_callback);
		if (ret_code < 0) {
			if (need_tx_dma)
				DMA_FREE(os);
			printk(KERN_ERR
			       "Failed to request the DMA for recording\n");
		}
	}

	if (ret_code == 0) {
		if ((file->f_mode & FMODE_WRITE)) {
			state->wr_ref = 1;
			audio_reset (os);
			os->fragsize = AUDIO_FRAGSIZE_DEFAULT;
			os->nbfrags = AUDIO_NBFRAGS_DEFAULT;
			os->mapped = 0;
			init_waitqueue_head (&os->wq);
		}

		if (file->f_mode & FMODE_READ) {
			state->rd_ref = 1;
			audio_reset(is);
			is->fragsize = AUDIO_FRAGSIZE_DEFAULT;
			is->nbfrags = AUDIO_NBFRAGS_DEFAULT;
			is->mapped = 0;
			init_waitqueue_head(&is->wq);
		}

		file->private_data = (void *) state;
	}

	up(&state->sem);

	return ret_code;
}

/* audio_ctlr_write function. Writes the amount the data supplied by the
 * user to the device */
static int audio_ctlr_write(struct file *file, const char *buffer, size_t count,
			    loff_t *ppos)
{
	const char *buffer0 = buffer;
	audio_state_t *state = file->private_data;
	audio_stream_t *s = state->output_stream;
	int chunksize, ret = 0;
	unsigned long flags;

	if (*ppos != file->f_pos) {
		printk(KERN_ERR "FPOS not ppos ppos=0x%x fpos =0x%x\n",
		       (u32) *ppos, (u32) file->f_pos);
		ret = -ESPIPE;
	} else if (s->mapped) {
		printk(KERN_ERR "s already mapped\n");
		ret = -ENXIO;
	} else if (!s->buffers && audio_setup_buf(s)) {
		printk(KERN_ERR "NO MEMORY\n");
		ret = -ENOMEM;
	} else {
		while (count > 0) {
			audio_buf_t *b = &s->buffers[s->usr_head];

			/* Wait for a buffer to become free */
			if (file->f_flags & O_NONBLOCK) {
				ret = -EAGAIN;
				if (!s->wfc.done) {
					break;
				} else {
					local_irq_save(flags);
					s->wfc.done--;
					local_irq_restore(flags);
				}
			} else {
				ret = -ERESTARTSYS;
				if ( wait_for_completion_interruptible(&s->wfc)
					) {
					break;
				}
			}

			/* Feed the current buffer */
			chunksize = s->fragsize - b->offset;
			if (chunksize > count)
				chunksize = count;

			if (copy_from_user(b->data + b->offset, buffer,
					   chunksize)) {
				printk(KERN_ERR "Audio: CopyFrom User failed \
						\n");
				complete(&s->wfc);
				ret = -EFAULT;
				break;
			}

			buffer += chunksize;
			count -= chunksize;
			b->offset += chunksize;

			if (b->offset < s->fragsize) {
				complete(&s->wfc);
				break;
			}

			b->offset = 0;

			if (++s->usr_head >= s->nbfrags)
				s->usr_head = 0;

			s->pending_frags++;
			s->active = 1;
			atomic_set(&s->in_write_path, 1);
			audio_process_dma(s);
			atomic_set(&s->in_write_path, 0);
		}

	}

	if ((buffer - buffer0))
		ret = buffer - buffer0;

	return ret;
}

/* 	audio_ctlr_read function. Reads the data and gives to the user. */

static int audio_ctlr_read(struct file *file, char *buffer, size_t count,
			   loff_t *ppos)
{
	char *buffer0 = buffer;
	audio_state_t *state = file->private_data;
	audio_stream_t *s = state->input_stream;
	int chunksize, ret = 0;
	unsigned long flags;

	if (*ppos != file->f_pos) {
		printk(KERN_ERR
		       "AudioRead - FPOS not ppos ppos=0x%x fpos = 0x%x\n",
		       (u32) * ppos, (u32) file->f_pos);
		return -ESPIPE;
	}

	if (s->mapped) {
		printk(KERN_ERR "AudioRead - s already mapped\n");
		return -ENXIO;
	}

	if (!s->active) {
		if (!s->buffers && audio_setup_buf(s)) {
			printk(KERN_ERR "AudioRead - No Memory\n");
			return -ENOMEM;
		}

		audio_prime_rx(state);
	}

	while (count > 0) {
		audio_buf_t *b = &s->buffers[s->usr_head];
		/* Wait for a buffer to become full */
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			if (!s->wfc.done) {
				break;
			} else {
				local_irq_save(flags);
				s->wfc.done--;
				local_irq_restore(flags);
			}
		} else {
			ret = -ERESTARTSYS;
			if (wait_for_completion_interruptible(&s->wfc)) {
				break;
			}
		}
		/* Grab data from the current buffer */
		chunksize = s->fragsize - b->offset;

		if (chunksize > count)
			chunksize = count;

		if (copy_to_user(buffer, b->data + b->offset, chunksize)) {
			complete(&s->wfc);
			return -EFAULT;
		}
		buffer += chunksize;
		count -= chunksize;
		b->offset += chunksize;
		if (b->offset < s->fragsize) {
			complete(&s->wfc);
			break;
		}
		/* Update pointers and return current fragment to DMA */
		local_irq_save(flags);

		b->offset = 0;

		if (++s->usr_head >= s->nbfrags)
			s->usr_head = 0;

		s->pending_frags++;

		local_irq_restore(flags);

		audio_process_dma(s);
	}

	if ((buffer - buffer0))
		ret = buffer - buffer0;

	return ret;
}


static int audio_ctlr_release(struct inode *inode, struct file *file)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *os = state->output_stream;
	audio_stream_t *is = state->input_stream;
	s8 status = 0;
	u8 codec_id = 0, asp_id = 0;
	u8 dev_instance = iminor (inode);

	status = find_audio_device ( dev_instance, &asp_id, &codec_id );

	if (status != 0) {
		printk(KERN_INFO " Audio device could not be found");
		return -ENODEV;
	}

	if (file->f_mode & FMODE_READ) {
		audio_discard_buf(is);
		DMA_FREE(is);
		is->dma_spinref = 0;
		if (state->need_tx_for_rx) {
			os->spin_idle = 0;
			if (!state->wr_ref) {
				DMA_FREE(os);
				os->dma_spinref = 0;
			}
		}
		state->rd_ref = 0;
		evm_aud_info->asp[asp_id].a_op->stop_rx(asp_id);
	}

	if (file->f_mode & FMODE_WRITE) {
		audio_sync(file);
		audio_discard_buf(os);
		if (!state->need_tx_for_rx || !state->rd_ref) {
			DMA_FREE(os);
			os->dma_spinref = 0;
		}
		state->wr_ref = 0;
		evm_aud_info->asp[asp_id].a_op->stop_tx(asp_id);
	}

	if ( !state->wr_ref && !state->rd_ref ) {
		module_put(state->owner);
		module_put(THIS_MODULE);
		atomic_set(&os->playing_null, 0);

		kfree(os);
		kfree(is);
		kfree(state);
		evm_aud_info->asp[asp_id].audio_state = NULL;
	}

	return 0;
}

static void configure_codec ( u8 dev_instance )
{
	u8 codec_id, asp_id;
	s8 status = 0;
	s8 ret_code = 0;

	/* With this given minor number, search the codec */
	status = find_audio_device ( dev_instance, &asp_id, &codec_id );

	if (unlikely(status != 0)) {
		printk(KERN_INFO " Device could not be opened");
		ret_code = -ENODEV;
	} else {
		ret_code =
			evm_aud_info->asp[asp_id].a_op->init(
				evm_aud_info->asp_count,
				evm_aud_info->asp[asp_id].a_info);
		if ( evm_aud_info->asp[asp_id].master_mode == ASP_MASTER ) {
			/* If ASP is master, configure ASP first and then the
			   codec */
			ret_code =
				evm_aud_info->asp[asp_id].a_op->configure(
					asp_id, &audio_cfg[asp_id]);
			if (ret_code != 0) {
				printk(KERN_ERR "Unable to initialise ASP %d",
				       asp_id);
			} else {
				if ( evm_aud_info->asp[asp_id].dit_enabled !=
				     1) {
					/* NK: Changes may be required according
					   to the FILE_MODES ??????*/
					ret_code =
						evm_aud_info->asp[asp_id].
						codec[codec_id].c_op->configure(
							codec_id,
							&audio_cfg[asp_id]);
					if ( ret_code != 0 ) {
						printk(KERN_ERR
						       "Unable to initialise\
							Codec %d", codec_id);
					}
				}
			}
		} else {		/* If ASP is Slave */
			/* Got the desired codec and ASP. Configure them */
			ret_code =
				evm_aud_info->asp[asp_id].codec[codec_id].
				c_op->configure(
					codec_id, (void *)&audio_cfg[asp_id]);
			if ( ret_code != 0 ) {
				printk(KERN_ERR "Unable to initialise Codec %d",
				       codec_id);
			} else {
				/* NK: Changes may be required according to the
				   FILE_MODES ??????*/
				ret_code = evm_aud_info->asp[asp_id].
					a_op->configure(
						asp_id, &audio_cfg[asp_id]);
				if ( ret_code != 0 ) {
					printk(KERN_ERR "Unable to initialise\
							Codec %d", codec_id);
				}
			}
		}
	}
}

s8 register_all_codecs ( void )
{
	s8 retcode = -1;
	u8 asp_cnt = 0, codec_cnt = 0;
	int minor = 0;
	int mixer_dev_id;

	printk(KERN_INFO
	       "Registering Audio Devices. Total communication	peripherals \
		(ASP) : %d \n", evm_aud_info->asp_count);

	if (evm_aud_info != NULL) {
		for ( asp_cnt = 0; asp_cnt < evm_aud_info->asp_count; asp_cnt++
			) {
			audio_serial_port_t *asp = NULL;

			asp = &evm_aud_info->asp[asp_cnt];

			if ( evm_aud_info->asp[asp_cnt].dit_enabled ==
			     1 ) {
				audio_codec_t *codec = NULL;
				minor = register_sound_dsp
					(&audio_controller_fops, -1);

				if (minor < 0) {
					printk (KERN_INFO \
						"SPDIF on ASP = %d is not" \
						"initialized. Skipping to" \
						"the next one\n", \
						asp->id);
					continue;
				} else {
					codec = &asp[asp_cnt].codec[codec_cnt];
					/* If atleast once codec is initialized,
					   then we return success */
					retcode = 0;
					codec->device_instance = minor;
					/* Initialise the minor number */
					evm_aud_info->asp[asp_cnt].codec[0].
						device_instance = minor;
					configure_codec(minor);
					printk (KERN_INFO \
						"SPDIF on ASP = %d is" \
						"initialized."\
						"Using minor number : %d\n", \
						asp->id, minor);
				}

				continue;
			}

			for ( codec_cnt = 0; codec_cnt < asp->codec_count;
			      codec_cnt++ ) {
				audio_codec_t *codec = NULL;

				minor = register_sound_dsp
					(&audio_controller_fops, -1);
				mixer_dev_id =
					register_sound_mixer(
						&davinci_mixer_fops, -1);

				if (minor < 0) {
					printk (KERN_INFO \
						"Codec with ID = %d on ASP =" \
						"%d is not" \
						"initialized. Skipping to the"\
						"next one\n", \
						codec->id, asp->id);
					continue;
				} else {
					codec = &asp[asp_cnt].codec[codec_cnt];
					/* If atleast once codec is initialized,
					   then we return success */
					retcode = 0;
					codec->device_instance = minor;
					configure_codec(minor);
					printk (KERN_INFO \
						"Codec with ID = %d on ASP ="\
						" %d is initialized."\
						"Using minor number :  %d\n", \
						codec->id, asp->id, minor);
				}
			}
		}
	}

	return retcode;
}


/*******************************************************************************
 * audio_ioctl(): Handles generic ioctls. If there is a request for something
 * this fn cannot handle, its then given to client specific ioctl routine, that
 * will take up platform specific requests
 ******************************************************************************/
static int audio_ctlr_ioctl(struct inode *inode, struct file *file, uint cmd,
			    ulong arg)
{
	audio_state_t *state = (audio_state_t *)file->private_data;
	audio_stream_t *os = state->output_stream;
	audio_stream_t *is = state->input_stream;
	long val;
	int ret = 0;
	static int default_rate = 48000;
	u8 codec_id, asp_id;
	s8 status = 0;
	u8 dev_instance = iminor (inode);

	status = find_audio_device ( dev_instance, &asp_id, &codec_id );

	if (status != 0) {
		printk(KERN_INFO " Audio device could not be found");
		return -ENODEV;
	}

	/* dispatch based on command */
	switch (cmd) {
	case OSS_GETVERSION:
		return put_user(SOUND_VERSION, (int *)arg);

	case SNDCTL_DSP_GETBLKSIZE:
		if (file->f_mode & FMODE_WRITE)
			return put_user(os->fragsize, (int *)arg);
		else
			return put_user(is->fragsize, (int *)arg);

	case SNDCTL_DSP_GETCAPS:
		val = DSP_CAP_REALTIME | DSP_CAP_TRIGGER | DSP_CAP_MMAP;
		if (is && os)
			val |= DSP_CAP_DUPLEX;
		return put_user(val, (int *)arg);

	case SNDCTL_DSP_SETFRAGMENT:
		if (get_user(val, (long *)arg)) {
			return -EFAULT;
		}
		if (file->f_mode & FMODE_READ) {
			int ret = audio_set_fragments(is, val);
			if (ret < 0) {
				return ret;
			}
			ret = put_user(ret, (int *)arg);
			if (ret) {
				return ret;
			}
		}
		if (file->f_mode & FMODE_WRITE) {
			int ret = audio_set_fragments(os, val);

			if (ret < 0) {
				return ret;
			}

			ret = put_user(ret, (int *)arg);
			if (ret) {
				return ret;
			}
		}
		return 0;

	case SNDCTL_DSP_SYNC:
		return audio_sync(file);

	case SNDCTL_DSP_SETDUPLEX:
		return 0;

	case SNDCTL_DSP_POST:
		return 0;

	case SNDCTL_DSP_GETTRIGGER:
		val = 0;
		if (file->f_mode & FMODE_READ && is->active && !is->stopped)
			val |= PCM_ENABLE_INPUT;
		if (file->f_mode & FMODE_WRITE && os->active && !os->stopped)
			val |= PCM_ENABLE_OUTPUT;
		return put_user(val, (int *)arg);

	case SNDCTL_DSP_SETTRIGGER:
		if (get_user(val, (int *)arg)) {
			return -EFAULT;
		}
		if (file->f_mode & FMODE_READ) {
			if (val & PCM_ENABLE_INPUT) {
				unsigned long flags;
				if (!is->active) {
					if ( !is->buffers && audio_setup_buf(is)
						) {
						return -ENOMEM;
					}
					audio_prime_rx(state);
				}
				local_irq_save(flags);
				is->stopped = 0;
				local_irq_restore(flags);
				audio_process_dma(is);
			} else {
				is->stopped = 1;
				audio_stop_dma(is);
			}
		}
		if (file->f_mode & FMODE_WRITE) {
			if (val & PCM_ENABLE_OUTPUT) {
				unsigned long flags;
				if (!os->buffers && audio_setup_buf(os)) {
					return -ENOMEM;
				}
				local_irq_save(flags);
				if (os->mapped && !os->pending_frags) {
					os->pending_frags = os->nbfrags;
					init_completion(&os->wfc);
					os->wfc.done = 0;
					os->active = 1;
				}
				os->stopped = 0;
				local_irq_restore(flags);
				audio_process_dma(os);
			} else {
				os->stopped = 1;
				audio_stop_dma(os);
			}
		}
		return 0;

	case SNDCTL_DSP_GETOPTR:
	case SNDCTL_DSP_GETIPTR:
	{
		count_info inf = { 0, };
		audio_stream_t *s =
			(cmd == SNDCTL_DSP_GETOPTR) ? os : is;
		int bytecount, offset;
		unsigned long flags;
		if ((s == is && !(file->f_mode & FMODE_READ)) ||
		    (s == os && !(file->f_mode & FMODE_WRITE))) {
			return -EINVAL;
		}
		if (s->active) {
			local_irq_save(flags);
			offset = audio_get_dma_pos(s);
			inf.ptr = s->dma_tail * s->fragsize + offset;
			bytecount = s->bytecount + offset;
			s->bytecount = -offset;
			inf.blocks = s->fragcount;
			s->fragcount = 0;
			local_irq_restore(flags);
			if (bytecount < 0)
				bytecount = 0;
			inf.bytes = bytecount;
		}
		return copy_to_user((void *)arg, &inf, sizeof(inf));
	}

	case SNDCTL_DSP_GETOSPACE:
	case SNDCTL_DSP_GETISPACE:
	{
		audio_buf_info inf = { 0, };
		audio_stream_t *s =
			(cmd == SNDCTL_DSP_GETOSPACE) ? os : is;
		if ((s == is && !(file->f_mode & FMODE_READ)) ||
		    (s == os && !(file->f_mode & FMODE_WRITE))) {
			return -EINVAL;
		}
		if (!s->buffers && audio_setup_buf(s)) {
			return -ENOMEM;
		}
		inf.bytes = s->wfc.done * s->fragsize;

		inf.fragments = inf.bytes / s->fragsize;
		inf.fragsize = s->fragsize;
		inf.fragstotal = s->nbfrags;
		return copy_to_user((void *)arg, &inf, sizeof(inf));
	}

	case SNDCTL_DSP_NONBLOCK:
		file->f_flags |= O_NONBLOCK;
		return 0;

	case SNDCTL_DSP_RESET:
		if (file->f_mode & FMODE_READ) {
			audio_reset(is);
			if (state->need_tx_for_rx) {
				unsigned long flags;
				local_irq_save(flags);
				os->spin_idle = 0;
				local_irq_restore(flags);
			}
		}
		if (file->f_mode & FMODE_WRITE) {
			audio_reset(os);
		}
		return 0;

	default:
		/* Nirmal: If McASP is configured for the DIT, do not call codec
		   ioctls */
		if (evm_aud_info->asp[asp_id].dit_enabled == 1) {
			/* Take some of the ioctls, which other wise are
			   applicable to codec */
			switch (cmd) {
			case SOUND_PCM_READ_RATE:
				return put_user(default_rate, (long *)arg);
				break;

			case SOUND_PCM_READ_BITS:
			case SNDCTL_DSP_GETFMTS:
				/* NK: Only 16 bit little endian format is
				   supported */
				return put_user(AFMT_S16_LE, (long *)arg);
				break;

			case SNDCTL_DSP_SPEED:
				ret = get_user(val, (long *)arg);
				if (ret) {
					printk(KERN_ERR "get_user failed\n");
					return -1;
				}
				if (val > 48000) {
					printk(KERN_ERR "\n Sample rate %d not"\
					       "supported", (int)val);
					return -1;
				} else {
					/* Set the desired sampling frequency */
					audio_cfg[asp_id].sample_rate = val;
					ret = evm_aud_info->asp[asp_id].
						a_op->configure(
							asp_id,
							&audio_cfg[asp_id]
							);
					return ret;
				}
				break;

			case SNDCTL_DSP_SETFMT:
				/* set the desired Format */
				ret = get_user(val, (long *)arg);
				if (ret) {
					printk("get_user failed\n");
					break;
				}
				if (val == AFMT_S16_LE) {
					printk (KERN_ERR "\n Setting" \
						" AFMT_S16_LE requested" \
						" Format ");
					acount = 4;
					return 0;
				}
				/*
				  else if (val == AFMT_U8 ) {
				  printk ("\n Setting" \
				  " AFMT_U8 requested Format");
				  acount = 2;
				  return 0;
				  }
				*/
				else{
					printk (KERN_ERR "\n Requested Format"\
						" not supported\n");
					return -EPERM;
				}
				break;
			default:
				printk(KERN_ERR "\n This ioctl might not be"\
				       " applicable for DIT\n");
				return 0;
			}
		} else {
			switch (cmd) {
			case SNDCTL_DSP_SPEED:
				if ( evm_aud_info->asp[asp_id].master_mode ==
				     ASP_MASTER &&
				     evm_aud_info->asp[asp_id].dit_enabled != 1
					) {
					asp_fops_t *asp_fops = NULL;
					u32 sample_rate = 0;

					asp_fops = get_asp_fops(dev_instance);

					ret = get_user(val, (long *)arg);
					if (ret) {
						printk("get_user failed\n");
						break;
					}

					sample_rate =
						asp_fops->set_sample_rate(
							asp_id, val);

					if (sample_rate != val) {
						printk (KERN_ERR
							"asp_fops->"\
							"set_sample_rate "\
							"failed\n");
					}

					return put_user(sample_rate,
							(long *)arg);
				} else {
					if ( evm_aud_info->asp[asp_id].
					     codec[codec_id].c_op->ioctl !=
					     NULL) {
						return (evm_aud_info->
							asp[asp_id].
							codec[codec_id].
							c_op->ioctl(inode,
								    file,
								    cmd,
								    arg)
							);
					} else {
						printk (KERN_INFO "Ioctl "\
							"command (%d) not "\
							" supported\n", cmd);
						return 0;
					}
				}
				break;

			case SOUND_PCM_READ_RATE:
				if ( evm_aud_info->asp[asp_id].master_mode
				     == ASP_MASTER &&
				     evm_aud_info->asp[asp_id].dit_enabled != 1
					) {
					asp_fops_t *asp_fops = NULL;
					u32 sample_rate = 0;

					asp_fops = get_asp_fops(dev_instance);
					sample_rate =
						asp_fops->get_sample_rate(
							asp_id);

					return put_user(sample_rate,
							(long *)arg);
				} else {
					if (evm_aud_info->asp[asp_id].
					    codec[codec_id].c_op->ioctl !=
					    NULL) {
						return (evm_aud_info->
							asp[asp_id].
							codec[codec_id].
							c_op->ioctl(inode,
								    file,
								    cmd,
								    arg));
					} else {
						printk (KERN_INFO "Ioctl "\
							"command (%d) not "\
							"supported\n", cmd);
						return 0;
					}
				}
				break;

			default:
				if (evm_aud_info->asp[asp_id].codec[codec_id].
				    c_op->ioctl !=
				    NULL) {
					return (evm_aud_info->
						asp[asp_id].
						codec[codec_id].
						c_op->ioctl(inode,
							    file,
							    cmd,
							    arg));
				} else {
					printk (KERN_INFO "Ioctl "\
						"command (%d) not "\
						"supported\n", cmd);
					return 0;
				}
			}
		}
	}

	return 0;
}


static int __init audio_controller_init ( void )
{
	s8 retcode = 0;

	if (validate_evm_audio_config (&evm_aud_info) < 0 ) {
		retcode = -EINVAL;
	} else {
		/* Register codecs described in the evm_aud_info
		   structure */
		register_all_codecs ();
	}

	return 0;
}

static void __exit audio_controller_exit ( void )
{
	return;
}

module_init(audio_controller_init);
module_exit(audio_controller_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Generic Audio Driver");
MODULE_LICENSE("GPL");
