/*
 * 
 *
 * Copyright (C) 2005 Texas Instruments Inc
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
/* dm355_ipipe.c file */

/* include Linux files */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/cdev.h>		/* Used for struct cdev */
#include <linux/dma-mapping.h>	/* For class_simple_create */
#include <linux/interrupt.h>	/* For IRQ_HANDLED and irqreturn_t */
#include <asm/uaccess.h>	/* for VERIFY_READ/VERIFY_WRITE/
				   copy_from_user */
#include <linux/device.h>
#include <asm/semaphore.h>
#include <asm-arm/arch-davinci/dm355_ipipe_hw.h>
#include <linux/major.h>
#include <asm-arm/arch-davinci/dm355_ipipe.h>

#include "ipipe_para.h"

/* Keeps track of how many times the device driver has been opened */
static atomic_t reference_count = ATOMIC_INIT(0);

static struct class_simple *ipipe_class = NULL;
struct device *ipipe_dev;
/* ipipe_isr: It is interrupt handler for PRVINT interrupt. 
 It will be called when ipipe completes processing of one 
 frame and writes data to the DDR. It unblocks the IPIPE 
 ioctl which is waiting for the processing to be completed */
irqreturn_t ipipe_isr(int irq, void *device_id, struct pt_regs *regs)
{

	struct ipipe_device *ipipedevice = (struct ipipe_device *)device_id;
	dev_dbg(ipipe_dev, "IPIPE isr returned.\n");
	/* indicate the completion ofr frame processing */
	if (ipipedevice)
		complete(&(ipipedevice->wfc));

	return IRQ_HANDLED;
}

#define DRIVERNAME  "DM355IPIPE"

/* global object of ipipe_device structure */
struct ipipe_device ipipedevice = { 0 };

/* inline function to free reserver pages  */
void inline ipipe_free_pages(unsigned long addr, unsigned long bufsize)
{
	unsigned long size, ad = addr;
	size = PAGE_SIZE << (get_order(bufsize));
	if (!addr)
		return;
	while (size > 0) {
		ClearPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	free_pages(ad, get_order(bufsize));
}

/* This function is used to free memory allocated to buffers */
int free_buffers(struct ipipe_device *device)
{
	int i;
	unsigned long adr;
	if (!device) {
		dev_err(ipipe_dev, "\nfree_buffers:error in argument");
		return -EINVAL;
	}
	/* free memory allocated to in buffers */
	for (i = 0; i < device->in_numbuffers; i++) {
		if (device->in_buff[i]) {
			adr = device->in_buff[i]->offset;
			if (adr)
				ipipe_free_pages((unsigned long)
						 phys_to_virt(adr),
						 device->in_buff[i]->size);

			kfree(device->in_buff[i]);

			device->in_buff[i] = NULL;
		}
	}
	device->in_numbuffers = 0;
	/* free memory allocated to out buffers */
	for (i = 0; i < device->out_numbuffers; i++) {
		if (device->out_buff[i]) {
			adr = device->out_buff[i]->offset;
			if (adr)
				ipipe_free_pages((unsigned long)
						 phys_to_virt(adr),
						 device->out_buff[i]->size);

			kfree(device->out_buff[i]);

			device->out_buff[i] = NULL;
		}
	}

	device->out_numbuffers = 0;
	return 0;
}

/* 
 * This function will query the buffer's physical address
 * whose index is passed in ipipe_buffer. 
 * It will store that address in ipipe_buffer. 
 */
int query_buffer(struct ipipe_device *device, struct ipipe_buffer *buffer)
{

	if (!buffer || !device) {
		dev_err(ipipe_dev, "query_buffer: error in argument\n");
		return -EINVAL;
	}

	if (buffer->index < 0) {
		dev_err(ipipe_dev, "query_buffer: invalid index %d\n",
			buffer->index);
		return -EINVAL;
	}

	if ((buffer->buf_type != IPIPE_BUF_IN)
	    && (buffer->buf_type != IPIPE_BUF_OUT)) {
		dev_err(ipipe_dev, "request_buffer: invalid buffer type\n");
		return -EINVAL;
	}
	/* if buf_type is input buffer then get offset of input buffer */
	if (buffer->buf_type == IPIPE_BUF_IN) {
		/* error checking for wrong index number */
		if (buffer->index >= device->in_numbuffers) {
			dev_err(ipipe_dev, "query_buffer: invalid index");

			return -EINVAL;
		}

		/* get the offset and size of the buffer and store
		   it in buffer */
		buffer->offset = device->in_buff[buffer->index]->offset;
		buffer->size = device->in_buff[buffer->index]->size;
	}
	/* if buf_type is output buffer then get offset of output buffer */
	else if (buffer->buf_type == IPIPE_BUF_OUT) {
		/* error checking for wrong index number */
		if (buffer->index >= device->out_numbuffers) {
			dev_err(ipipe_dev, "query_buffer: invalid index\n");

			return -EINVAL;
		}
		/* get the offset and size of the buffer and store
		   it in buffer */
		buffer->offset = device->out_buff[buffer->index]->offset;
		buffer->size = device->out_buff[buffer->index]->size;
	} else {
		dev_err(ipipe_dev, "query_buffer: invalid buffer type\n");

		return -EINVAL;
	}

	return 0;
}

int request_buffer(struct ipipe_device *device, struct ipipe_reqbufs *reqbufs)
{
	struct ipipe_buffer *buffer = NULL;
	int count = 0;
	unsigned long adr;
	u32 size;

	if (!reqbufs || !device) {
		dev_err(ipipe_dev, "request_buffer: error in argument\n");
		return -EINVAL;
	}

	/* if number of buffers requested is more then support return error */
	if (reqbufs->count > MAX_BUFFER) {
		dev_err(ipipe_dev, "request_buffer: invalid buffer count\n");
		return -EINVAL;
	}

	if ((reqbufs->buf_type != IPIPE_BUF_IN)
	    && (reqbufs->buf_type != IPIPE_BUF_OUT)) {
		dev_err(ipipe_dev, "request_buffer: invalid buffer type %d\n",
			reqbufs->buf_type);
		return -EINVAL;
	}
	if (reqbufs->count < 0) {
		dev_err(ipipe_dev, "request_buffer: invalid buffer count %d\n",
			reqbufs->count);
		return -EINVAL;
	}
	/* if buf_type is input then allocate buffers for input */
	if (reqbufs->buf_type == IPIPE_BUF_IN) {
		/*if buffer count is zero, free all the buffers */
		if (reqbufs->count == 0) {
			/* free all the buffers */
			for (count = 0; count < device->in_numbuffers; count++) {
				/* free memory allocate for the image */
				if (device->in_buff[count]) {
					adr =
					    (unsigned long)device->
					    in_buff[count]->offset;
					if (adr)
						ipipe_free_pages((unsigned long)
								 phys_to_virt
								 (adr),
								 device->in_buff
								 [count]->size);

					/* free the memory allocated
					   to ipipe_buffer */
					kfree(device->in_buff[count]);

					device->in_buff[count] = NULL;
				}
			}
			device->in_numbuffers = 0;
			return 0;
		}

		/* free the extra buffers */
		if (device->in_numbuffers > reqbufs->count &&
		    reqbufs->size == device->in_buff[0]->size) {
			for (count = reqbufs->count;
			     count < device->in_numbuffers; count++) {
				/* free memory allocate for the image */
				if (device->in_buff[count]) {
					adr = device->in_buff[count]->offset;
					if (adr)
						ipipe_free_pages((unsigned long)
								 phys_to_virt
								 (adr),
								 device->in_buff
								 [count]->size);

					/* free the memory allocated
					   to ipipe_buffer */
					kfree(device->in_buff[count]);

					device->in_buff[count] = NULL;
				}
			}
			device->in_numbuffers = reqbufs->count;
			return 0;
		}
		/* if size requested is different from already allocated,
		   free memory of all already allocated buffers */
		if (device->in_numbuffers) {
			if (reqbufs->size != device->in_buff[0]->size) {
				for (count = 0;
				     count < device->in_numbuffers; count++) {
					if (device->in_buff[count]) {
						adr =
						    device->
						    in_buff[count]->offset;
						if (adr)
							ipipe_free_pages((unsigned long)
									 phys_to_virt
									 (adr),
									 device->
									 in_buff
									 [count]->
									 size);

						kfree(device->in_buff[count]);

						device->in_buff[count] = NULL;
					}
				}
				device->in_numbuffers = 0;
			}
		}

		/* allocate the buffer */
		for (count = device->in_numbuffers; count < reqbufs->count;
		     count++) {
			/* Allocate memory for struct ipipe_buffer */
			buffer =
			    kmalloc(sizeof(struct ipipe_buffer), GFP_KERNEL);

			/* if memory allocation fails then return error */
			if (!buffer) {
				/* free all the buffers */
				while (--count >= device->in_numbuffers) {
					adr = device->in_buff[count]->offset;
					if (adr)
						ipipe_free_pages((unsigned long)
								 phys_to_virt
								 (adr),
								 device->in_buff
								 [count]->size);
					kfree(device->in_buff[count]);
					device->in_buff[count] = NULL;
				}
				dev_err(ipipe_dev, "1.request_buffer:not \
					enough memory\n");
				return -ENOMEM;
			}

			/* assign buffer's address in configuration */
			device->in_buff[count] = buffer;

			/* set buffers index and buf_type,size parameters */
			buffer->index = count;
			buffer->buf_type = IPIPE_BUF_IN;
			buffer->size = reqbufs->size;
			/* allocate memory for buffer of size passed
			   in reqbufs */
			buffer->offset =
			    (unsigned long)__get_free_pages(GFP_KERNEL |
							    GFP_DMA,
							    get_order
							    (reqbufs->size));

			/* if memory allocation fails, return error */
			if (!(buffer->offset)) {
				/* free all the buffer's space */
				kfree(buffer);
				device->in_buff[count] = NULL;
				while (--count >= device->in_numbuffers) {
					adr = device->in_buff[count]->offset;
					if (adr)
						ipipe_free_pages((unsigned long)
								 phys_to_virt
								 (adr),
								 device->in_buff
								 [count]->size);
					kfree(device->in_buff[count]);
					device->in_buff[count] = NULL;
				}
				dev_err(ipipe_dev, "2.request_buffer:not \
					enough memory\n");

				return -ENOMEM;
			}

			adr = (unsigned long)buffer->offset;
			size = PAGE_SIZE << (get_order(reqbufs->size));
			while (size > 0) {
				/* make sure the frame buffers
				   are never swapped out of memory */
				SetPageReserved(virt_to_page(adr));
				adr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
			/* convert vertual address to physical */
			buffer->offset = (unsigned long)
			    virt_to_phys((void *)(buffer->offset));
		}
		device->in_numbuffers = reqbufs->count;
	}
	/* if buf_type is output then allocate buffers for output */
	else if (reqbufs->buf_type == IPIPE_BUF_OUT) {
		if (reqbufs->count == 0) {
			/* free all the buffers */
			for (count = 0; count < device->out_numbuffers; count++) {
				/* free memory allocate for the image */
				if (device->out_buff[count]) {
					adr = device->out_buff[count]->offset;
					if (adr)
						ipipe_free_pages((unsigned long)
								 phys_to_virt
								 (adr),
								 device->
								 out_buff
								 [count]->size);

					/* free the memory allocated to
					   ipipe_buffer */
					kfree(device->out_buff[count]);

					device->out_buff[count] = NULL;
				}
			}
			device->out_numbuffers = 0;

			return 0;
		}
		/* free the buffers */
		if (device->out_numbuffers > reqbufs->count &&
		    reqbufs->size == device->out_buff[0]->size) {
			for (count = reqbufs->count;
			     count < device->out_numbuffers; count++) {
				/* free memory allocate for the image */
				if (device->out_buff[count]) {
					adr = device->out_buff[count]->offset;
					if (adr)
						ipipe_free_pages((unsigned long)
								 phys_to_virt
								 (adr),
								 device->
								 out_buff
								 [count]->size);

					/* free the memory allocated to
					   ipipe_buffer */
					kfree(device->out_buff[count]);

					device->out_buff[count] = NULL;
				}
			}
			device->out_numbuffers = reqbufs->count;

			return 0;
		}
		/* if size requested is different from already allocated,
		   free memory of all already allocated buffers */
		if (device->out_numbuffers) {
			if (reqbufs->size != device->out_buff[0]->size) {
				for (count = 0;
				     count < device->out_numbuffers; count++) {
					if (device->out_buff[count]) {
						adr =
						    device->
						    out_buff[count]->offset;

						if (adr)
							ipipe_free_pages((unsigned long)
									 phys_to_virt
									 (adr),
									 device->
									 out_buff
									 [count]->
									 size);

						kfree(device->out_buff[count]);

						device->out_buff[count] = NULL;
					}
				}
				device->out_numbuffers = 0;
			}
		}

		/* allocate the buffer */
		for (count = device->out_numbuffers;
		     count < reqbufs->count; count++) {
			/* Allocate memory for struct ipipe_buffer */
			buffer =
			    kmalloc(sizeof(struct ipipe_buffer), GFP_KERNEL);

			/* if memory allocation fails then return error */
			if (!buffer) {
				/* free all the buffers */
				while (--count >= device->out_numbuffers) {
					adr = device->out_buff[count]->offset;
					if (adr)
						ipipe_free_pages((unsigned long)
								 phys_to_virt
								 (adr),
								 device->
								 out_buff
								 [count]->size);
					kfree(device->out_buff[count]);
					device->out_buff[count] = NULL;
				}

				dev_err(ipipe_dev,
					"3.request_buffer:not enough \
					memory\n");

				return -ENOMEM;
			}

			/* assign buffer's address out configuration */
			device->out_buff[count] = buffer;

			/* set buffers outdex and buf_type,size parameters */
			buffer->index = count;
			buffer->buf_type = IPIPE_BUF_OUT;
			buffer->size = reqbufs->size;
			/* allocate memory for buffer of size passed
			   in reqbufs */
			buffer->offset =
			    (unsigned long)__get_free_pages(GFP_KERNEL |
							    GFP_DMA,
							    get_order
							    (reqbufs->size));

			/* if memory allocation fails, return error */
			if (!(buffer->offset)) {
				/* free all the buffer's space */
				kfree(buffer);
				device->out_buff[count] = NULL;
				while (--count >= device->out_numbuffers) {
					adr = device->out_buff[count]->offset;
					if (adr)
						ipipe_free_pages((unsigned long)
								 phys_to_virt
								 (adr),
								 device->
								 out_buff
								 [count]->size);
					kfree(device->out_buff[count]);
					device->out_buff[count] = NULL;
				}
				dev_err(ipipe_dev, "4.request_buffer:not \
					enough memory\n");

				return -ENOMEM;
			}

			adr = (unsigned long)buffer->offset;
			size = PAGE_SIZE << (get_order(reqbufs->size));
			while (size > 0) {
				/* make sure the frame buffers
				   are never swapped out of memory */
				SetPageReserved(virt_to_page(adr));
				adr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
			/* convert vertual address to physical */
			buffer->offset = (unsigned long)
			    virt_to_phys((void *)(buffer->offset));
		}
		device->out_numbuffers = reqbufs->count;
	} else {
		dev_err(ipipe_dev, "request_buffer: invalid buffer type\n");

		return -EINVAL;
	}

	return 0;
}

/* Functions */
int ipipe_open(struct inode *inode, struct file *filp)
{
	struct ipipe_params *config = NULL;

	if (atomic_inc_return(&reference_count) != 1) {
		dev_err(ipipe_dev, "ipipe_open: device is already openend\n");
		return -EBUSY;
	}

	/* allocate memory for a new configuration */
	if ((config = kmalloc(sizeof(struct ipipe_params), GFP_KERNEL)) == NULL)
		return -ENOMEM;

	/* store the pointer of ipipe_params in private_data member of file
	   and params member of ipipe_device */
	filp->private_data = config;

	/* initialize mutex to 0 */
	ipipedevice.params = config;
	ipipedevice.opened = 1;
	ipipedevice.in_numbuffers = 0;
	ipipedevice.out_numbuffers = 0;
	init_completion(&(ipipedevice.wfc));
	ipipedevice.wfc.done = 0;
	init_MUTEX(&(ipipedevice.sem));

	return 0;
}
int write_out_addr(int resize_no, unsigned int address)
{
	unsigned int utemp;
	unsigned int rsz_start_add;
	if ((resize_no == 0) || (resize_no == 1)) {
		if (resize_no)
			rsz_start_add = RSZ_EN_1;
		else
			rsz_start_add = RSZ_EN_0;
	} else
		return -EINVAL;

	utemp = address & SET_LOW_ADD;
	regw_ip(utemp, rsz_start_add + RSZ_SDR_BAD_L);
	regw_ip(utemp, rsz_start_add + RSZ_SDR_SAD_L);

	utemp = (address & SET_HIGH_ADD) >> 16;
	regw_ip(utemp, rsz_start_add + RSZ_SDR_BAD_H);
	regw_ip(utemp, rsz_start_add + RSZ_SDR_SAD_H);
	return 0;
}
int validate_params(struct ipipe_params *params)
{

	u32 data_format;

	if ((params->ipipeif_param.source == SDRAM_RAW) &&
	    (params->ipipe_dpaths_fmt == YUV2YUV)) {

		dev_err(ipipe_dev,
			"validate:input source and data format does not match\n");
		return -EINVAL;
	}

	if ((params->ipipeif_param.source == SDRAM_YUV) &&
	    (params->ipipe_dpaths_fmt == RAW2YUV)) {

		dev_err(ipipe_dev,
			"validate:input source and data format does not match\n");
		return -EINVAL;
	}

	if ((params->ipipeif_param.source < CCDC) ||
	    (params->ipipeif_param.source > SDRAM_YUV)) {

		dev_err(ipipe_dev,
			"validate:invalidate input source of ipipeif\n");
		return -EINVAL;
	}

	if (params->ipipeif_param.source == CCDC_DARKFM) {
		if (!(params->ipipeif_param.glob_ver_size)) {

			dev_err(ipipe_dev, "validate:glob_ver_size is 0\n");
			return -EINVAL;
		}
		/*Data should be read from SDRAM with CFG.ALAW set to .0 and
		   CFG.PACK8IN set to 1 */
		if ((params->ipipeif_param.ialaw)
		    || (!(params->ipipeif_param.pack_mode))) {

			dev_err(ipipe_dev,
				"validate: error in either ialaw or ipipeif pack mode\n");
			return -EINVAL;
		}
	}

	if ((params->ipipeif_param.source == CCDC_DARKFM)
	    || (params->ipipeif_param.source == CCDC)) {
		if (params->ipipeif_param.clock_select) {
			dev_err(ipipe_dev, "validate:error in clock select\n");
			return -EINVAL;
		}
	}

	if (params->ipipeif_param.decimation) {
		if ((params->ipipeif_param.rsz < ONE) ||
		    (params->ipipeif_param.rsz > ONE_SEVENTH)) {
			dev_err(ipipe_dev,
				"validate:error in resize factor for decimation \n");
			return -EINVAL;
		}
	}

	if (params->ipipeif_param.clock_select) {
		if ((params->ipipeif_param.clk_div < DIVIDE_HALF) ||
		    (params->ipipeif_param.clk_div > DIVIDE_THIRTY)) {
			dev_err(ipipe_dev,
				"validate:error in clock divisor factor value\n");
			return -EINVAL;
		}
	}

	if ((params->ipipeif_param.data_shift < BITS15_2)
	    || (params->ipipeif_param.data_shift > BITS9_0)) {

		dev_err(ipipe_dev, "validate:error in data shift value\n");
		return -EINVAL;
	}

	if ((params->ipipeif_param.clock_select != SDRAM_CLK)
	    && (params->ipipeif_param.clock_select != PIXCEL_CLK)) {

		dev_err(ipipe_dev, "validate:error in clock select value\n");
		return -EINVAL;
	}
	if ((params->ipipeif_param.ialaw != ALAW_OFF)
	    && (params->ipipeif_param.ialaw != ALAW_ON)) {

		dev_err(ipipe_dev, "validate:error in ialaw value\n");
		return -EINVAL;
	}
	if ((params->ipipeif_param.pack_mode != SIXTEEN_BIT)
	    && (params->ipipeif_param.pack_mode != EIGHT_BIT)) {

		dev_err(ipipe_dev, "validate:error in pack mode value\n");
		return -EINVAL;
	}

	if ((params->ipipeif_param.avg_filter != AVG_OFF)
	    && (params->ipipeif_param.avg_filter != AVG_ON)) {

		dev_err(ipipe_dev, "validate:error in average filter value\n");
		return -EINVAL;
	}

	if ((params->ipipeif_param.decimation != DECIMATION_OFF)
	    && (params->ipipeif_param.decimation != DECIMATION_ON)) {

		dev_err(ipipe_dev, "validate:error in decimation value\n");
		return -EINVAL;
	}

	if ((params->ipipeif_param.mode != CONTINUOUS)
	    && (params->ipipeif_param.mode != ONE_SHOT)) {

		dev_err(ipipe_dev, "validate:error in ipipeif mode value\n");
		return -EINVAL;
	}

	data_format =
	    (params->ipipe_dpaths_fmt | (params->ipipe_dpaths_bypass) << 2);
	/*in raw pass through mode   data can be < 4096 */
	if (data_format == RAW2RAW_BYPASS) {
		if (params->ipipe_hsz > MAX_SIZE_RAW_BY_PASS) {

			dev_err(ipipe_dev,
				"validate:invalid ipipe horizontal size\n");
			return -EINVAL;
		}
	}
	if (data_format != RAW2RAW_BYPASS) {
		if (params->ipipe_hsz > MAX_SIZE) {

			dev_err(ipipe_dev,
				"validate:invalid ipipe horizontal size\n");
			return -EINVAL;
		}
	}

	if ((params->ipipe_mode != 0) && (params->ipipe_mode != 1)) {

		dev_err(ipipe_dev, "validate:invalid ipipe mode\n");
		return -EINVAL;
	}
	if ((params->ipipe_dpaths_bypass != RAW_MODE_OFF) &&
	    (params->ipipe_dpaths_bypass != RAW_MODE_ON)) {

		dev_err(ipipe_dev, "validate:invalid ipipe datapath\n");
		return -EINVAL;

	}
	if ((params->ipipe_colpat_elep < RED)
	    || (params->ipipe_colpat_elep > BLUE)) {

		dev_err(ipipe_dev, "validate:invalid elep value\n");
		return -EINVAL;
	}
	if ((params->ipipe_colpat_elop < RED)
	    || (params->ipipe_colpat_elop > BLUE)) {

		dev_err(ipipe_dev, "validate:invalid elop value\n");
		return -EINVAL;
	}
	if ((params->ipipe_colpat_olep < RED)
	    || (params->ipipe_colpat_olep > BLUE)) {

		dev_err(ipipe_dev, "validate:invalid olep value\n");
		return -EINVAL;
	}
	if ((params->ipipe_colpat_olop < RED)
	    || (params->ipipe_colpat_olop > BLUE)) {

		dev_err(ipipe_dev, "validate:invalid olop value\n");
		return -EINVAL;
	}

	if (params->def_cor.dfc_en) {
		if ((params->def_cor.dfc_sel != FROMTOP)
		    && (params->def_cor.dfc_sel != FROMBOTTON)) {

			dev_err(ipipe_dev,
				"validate:invalid copy method in defect correction\n");
			return -EINVAL;
		}
		if (params->def_cor.dfc_siz > 1024) {

			dev_err(ipipe_dev,
				"validate:invalid defect correction table size\n");
			return -EINVAL;
		}
	}

	if (params->prog_nf.noise_fil_en) {
		if ((params->prog_nf.type != BOX)
		    && (params->prog_nf.type != DIAMOND)) {

			dev_err(ipipe_dev,
				"validate:invalid noise filter type\n");
			return -EINVAL;
		}

	}
	if (params->prefilter.pre_en) {

		if ((params->prefilter.sel_0 != AVG2MEDPIX)
		    && (params->prefilter.sel_0 != AVG4PIX)) {

			dev_err(ipipe_dev,
				"validate:invalid averaging method(sel0) in prefilter\n");
			return -EINVAL;
		}

		if ((params->prefilter.sel_1 != AVG2MEDPIX)
		    && (params->prefilter.sel_1 != AVG4PIX)) {

			dev_err(ipipe_dev,
				"validate:invalid averaging method(sel1) in prefilter\n");
			return -EINVAL;
		}

		if ((params->prefilter.typ_adaptive != DISABLE) &&
		    (params->prefilter.typ_adaptive != ENABLE)) {

			dev_err(ipipe_dev,
				"validate:invalid adaptive filter value(en0) in pre filter\n");
			return -EINVAL;
		}
		if ((params->prefilter.typ_adaptive_dotred != DISABLE) &&
		    (params->prefilter.typ_adaptive_dotred != ENABLE)) {

			dev_err(ipipe_dev,
				"validate:invalid adaptive dot reduction value(en1) in pre filter\n");
			return -EINVAL;
		}
	}

	if ((params->rgb2rgb.gmm_cfg_bypr != GC_ENABLE) &&
	    (params->rgb2rgb.gmm_cfg_bypr != GC_BYPASS)) {

		dev_err(ipipe_dev,
			"validate:invalid gamma correction enable/bypass value\n");
		return -EINVAL;
	}
	if ((params->rgb2rgb.gmm_cfg_bypr == GC_ENABLE)
	    && params->rgb2rgb.gmm_cfg_tbl == IPIPE_RAM
	    && (params->rgb2rgb.gmm_tbl_r == NULL)) {
		dev_err(ipipe_dev,
			"validate:gamma red enabled but table field is NULL\n");
		return -EINVAL;
	}

	if ((params->rgb2rgb.gmm_cfg_bypg != GC_ENABLE)
	    && params->rgb2rgb.gmm_cfg_tbl == IPIPE_RAM
	    && (params->rgb2rgb.gmm_cfg_bypg != GC_BYPASS)) {

		dev_err(ipipe_dev,
			"validate:invalid gamma green correction enable/bypass value\n");
		return -EINVAL;
	}

	if ((params->rgb2rgb.gmm_cfg_bypg == GC_ENABLE)
	    && params->rgb2rgb.gmm_cfg_tbl == IPIPE_RAM
	    && (params->rgb2rgb.gmm_tbl_g == NULL)) {
		dev_err(ipipe_dev,
			"validate:gamma green enabled but table field is NULL\n");
		return -EINVAL;
	}

	if ((params->rgb2rgb.gmm_cfg_bypb != GC_ENABLE)
	    && (params->rgb2rgb.gmm_cfg_bypb != GC_BYPASS)) {

		dev_err(ipipe_dev,
			"validate:invalid gamma blue correction enable/bypass value\n");
		return -EINVAL;
	}
	if ((params->rgb2rgb.gmm_cfg_bypb == GC_ENABLE)
	    && params->rgb2rgb.gmm_cfg_tbl == IPIPE_RAM
	    && (params->rgb2rgb.gmm_tbl_b == NULL)) {
		dev_err(ipipe_dev,
			"validate:gamma blue enabled but table field is NULL\n");
		return -EINVAL;
	}
	if ((params->rgb2rgb.gmm_cfg_tbl != IPIPE_RAM)
	    && (params->rgb2rgb.gmm_cfg_tbl != IPIPE_ROM)) {

		dev_err(ipipe_dev, "invlid gamma table source\n");
		return -EINVAL;
	}

	if ((params->rgb2rgb.gmm_cfg_siz < IPIPE_128)
	    || (params->rgb2rgb.gmm_cfg_siz > IPIPE_512)) {

		dev_err(ipipe_dev, "invalid gamma table size\n");
		return -EINVAL;
	}

	if ((params->rgb2yuv.yuv_phs_position != COSITING)
	    && (params->rgb2yuv.yuv_phs_position != CENTERING)) {

		dev_err(ipipe_dev, "invalid phase position\n");
		return -EINVAL;
	}
	if ((params->rgb2yuv.yuv_phs_lpf != ENABLE)
	    && (params->rgb2yuv.yuv_phs_lpf != DISABLE)) {

		dev_err(ipipe_dev, "invalid phase position\n");
		return -EINVAL;
	}

	if ((params->edge_enhancer.yee_en != ENABLE)
	    && (params->edge_enhancer.yee_en != DISABLE)) {

		dev_err(ipipe_dev,
			"invalid value in edge enhancement enable field\n");
		return -EINVAL;
	}

	if ((params->edge_enhancer.yee_emf != ENABLE)
	    && (params->edge_enhancer.yee_emf != DISABLE)) {

		dev_err(ipipe_dev, "invalid value in median NR enable field\n");
		return -EINVAL;
	}

	if (params->false_color_suppresion.fcs_en) {
		if ((params->false_color_suppresion.fcs_typ_typ < Y) ||
		    (params->false_color_suppresion.fcs_typ_typ > HPF_2D_YEE)) {

			dev_err(ipipe_dev,
				"invalid value in color suppresion enable field\n");
			return -EINVAL;
		}
	}

	if ((params->rsz_seq_seq != ENABLE) && (params->rsz_seq_seq != DISABLE)) {

		dev_err(ipipe_dev, "invalid value of operation mode field\n");
		return -EINVAL;
	}

	if ((params->rsz_seq_tmm != ENABLE) && (params->rsz_seq_tmm != DISABLE)) {

		dev_err(ipipe_dev,
			"invalid value of terminal condition enable field\n");
		return -EINVAL;
	}

	if ((params->rsz_seq_hrv != ENABLE) && (params->rsz_seq_hrv != DISABLE)) {

		dev_err(ipipe_dev,
			"invalid value of horizontal reversal field\n");
		return -EINVAL;
	}

	if ((params->rsz_seq_vrv != ENABLE) && (params->rsz_seq_vrv != DISABLE)) {

		dev_err(ipipe_dev,
			"invalid value of vertical reversal field\n");
		return -EINVAL;
	}

	if ((params->rsz_seq_crv != ENABLE) && (params->rsz_seq_crv != DISABLE)) {

		dev_err(ipipe_dev,
			"invalid value of chroma sampling point field\n");
		return -EINVAL;
	}

	if ((params->rsz_aal != ENABLE) && (params->rsz_aal != DISABLE)) {

		dev_err(ipipe_dev,
			"invalid value of anti aliasing filter field\n");
		return -EINVAL;
	}

	if (params->rsz_en[0]) {
		if ((params->rsz_rsc_param[0].rsz_h_typ != CUBIC)
		    && (params->rsz_rsc_param[0].rsz_h_typ != LINEAR)) {

			dev_err(ipipe_dev,
				"invalid low pass filter type in RZA\n");
			return -EINVAL;
		}

		if ((params->rsz_rsc_param[0].rsz_h_lse_sel != INTERNAL_VALUE)
		    && (params->rsz_rsc_param[0].rsz_h_lse_sel !=
			PROGRAMMED_VALUE)) {

			dev_err(ipipe_dev,
				"invalid intensity value of LPF in RZA\n");
			return -EINVAL;
		}

		if ((params->rsz2rgb[0].rsz_rgb_typ != OUTPUT_16BIT) &&
		    (params->rsz2rgb[0].rsz_rgb_typ != OUTPUT_32BIT)) {

			dev_err(ipipe_dev, "invalid RGB output type in RZA\n");
			return -EINVAL;
		}

		if ((params->rsz2rgb[0].rsz_rgb_msk0 != MASKLAST2)
		    && (params->rsz2rgb[0].rsz_rgb_msk0 != NOMASK)) {

			dev_err(ipipe_dev,
				"invalid value of mask0 field in RZA\n");
			return -EINVAL;
		}

		if ((params->rsz2rgb[0].rsz_rgb_msk1 != MASKLAST2)
		    && (params->rsz2rgb[0].rsz_rgb_msk1 != NOMASK)) {

			dev_err(ipipe_dev,
				"invalid value of mask1 field in RZA\n");
			return -EINVAL;
		}
		/*Check that mode selected by user in all modules should be same */
		if (!(params->ipipeif_param.mode == params->ipipe_mode) &&
		    (params->ipipeif_param.mode ==
		     params->rsz_rsc_param[0].rsz_mode)
		    && (params->ipipe_mode ==
			params->rsz_rsc_param[0].rsz_mode)) {

			dev_err(ipipe_dev, "mode does not match in RZA\n");
			return -EINVAL;
		}

		if (params->rsz_rsc_param[0].rsz_o_hsz > MAX_SIZE_RSZ0) {

			dev_err(ipipe_dev,
				"invalid output horizontal size in RZA\n");
			return -EINVAL;
		}

	}

	if (params->rsz_en[1]) {
		if ((params->rsz_rsc_param[1].rsz_h_typ != CUBIC)
		    && (params->rsz_rsc_param[1].rsz_h_typ != LINEAR)) {

			dev_err(ipipe_dev,
				"invalid low pass filter type in RZB\n");
			return -EINVAL;
		}

		if ((params->rsz_rsc_param[1].rsz_h_lse_sel != INTERNAL_VALUE)
		    && (params->rsz_rsc_param[1].rsz_h_lse_sel !=
			PROGRAMMED_VALUE)) {

			dev_err(ipipe_dev,
				"invalid intensity value of LPF in RZB\n");
			return -EINVAL;
		}

		if ((params->rsz2rgb[1].rsz_rgb_typ != OUTPUT_16BIT) &&
		    (params->rsz2rgb[1].rsz_rgb_typ != OUTPUT_32BIT)) {

			dev_err(ipipe_dev, "invalid RGB output type in RZB\n");
			return -EINVAL;
		}

		if ((params->rsz2rgb[1].rsz_rgb_msk0 != MASKLAST2)
		    && (params->rsz2rgb[1].rsz_rgb_msk0 != NOMASK)) {

			dev_err(ipipe_dev,
				"invalid value of mask0 field in RZB\n");
			return -EINVAL;
		}

		if ((params->rsz2rgb[1].rsz_rgb_msk1 != MASKLAST2)
		    && (params->rsz2rgb[1].rsz_rgb_msk1 != NOMASK)) {

			dev_err(ipipe_dev,
				"invalid value of mask1 field in RZB\n");
			return -EINVAL;
		}
		/*Check that mode selected by user in all modules should be same */
		if (!(params->ipipeif_param.mode == params->ipipe_mode) &&
		    (params->ipipeif_param.mode ==
		     params->rsz_rsc_param[1].rsz_mode)
		    && (params->ipipe_mode ==
			params->rsz_rsc_param[1].rsz_mode)) {

			dev_err(ipipe_dev, "mode does not match in RZB\n");
			return -EINVAL;
		}

		if (params->rsz_rsc_param[1].rsz_o_hsz > MAX_SIZE_RSZ1) {

			dev_err(ipipe_dev,
				"invalid output horizontal size in RZB\n");
			return -EINVAL;
		}
	}
	/*both resizers can not be enabled simultaniously in one shot mode */

	if ((params->ipipe_dpaths_fmt > YUV2YUV)
	    || (params->ipipe_dpaths_fmt < RAW2YUV)) {

		dev_err(ipipe_dev, "invalid data format\n");
		return -EINVAL;
	}

	if (params->def_cor.dfc_en)
		if (params->def_cor.dfc_siz > MAX_SIZE_DFC) {

			dev_err(ipipe_dev, "DFC table size exceeds limit\n");
			return -EINVAL;
		}
	return 0;
}

int ipipe(struct ipipe_device *device, struct ipipe_convert *arg)
{
	unsigned int utemp, utemp_h, utemp_l;
	int resizer_no_0, resizer_no_1;

	resizer_no_0 = device->params->rsz_en[0];
	resizer_no_1 = device->params->rsz_en[1];

	/*set ipipe clock */
	regw_vpss(0x79, VPSS_CLK);	/*!FIXME */
	if (device->params->ipipeif_param.source != 0) {
		regw_if(((device->params->ipipeif_param.adofs) >> 5),
			IPIPEIF_ADOFS);

		if (arg->in_buff.index < 0) {
			/*check if adress is 32 bit alligned */
			if (arg->in_buff.offset % 32) {
				dev_dbg(ipipe_dev,
					"address offset not alligned\n");
				return -EINVAL;
			}
			/*lower sixteen bit */
			utemp = arg->in_buff.offset;
			utemp_l = utemp >> 5;
			regw_if(utemp_l, IPIPEIF_ADDRL);
			/*upper next seven bit */
			utemp_h = utemp >> 21;
			regw_if(utemp_h, IPIPEIF_ADDRU);
		} else if (arg->in_buff.index > 7) {
			dev_dbg(ipipe_dev, "Invalid buffer index\n");
			return -EINVAL;
		} else {
			/*lower sixteen bit */
			utemp = device->in_buff[arg->in_buff.index]->offset;
			utemp_l = utemp >> 5;
			regw_if(utemp_l, IPIPEIF_ADDRL);
			utemp_h = utemp >> 21;

			regw_if(utemp_h, IPIPEIF_ADDRU);
		}
	}
	if (resizer_no_0)
		regw_ip(device->params->ext_mem_param[0].rsz_sdr_oft,
			RSZ_EN_0 + RSZ_SDR_OFT);
	regw_ip(device->params->rsz_en[0], RSZ_EN_0);
	if (resizer_no_1)
		regw_ip(device->params->ext_mem_param[1].rsz_sdr_oft,
			RSZ_EN_1 + RSZ_SDR_OFT);
	regw_ip(device->params->rsz_en[1], RSZ_EN_1);

	if (arg->out_buff.index < 0) {
		if (arg->out_buff.offset % 32)
			return -EINVAL;
		else {
			/* CHECK this statems */
			if (resizer_no_0 == 1 && resizer_no_1 == 1) {
				printk
				    ("If both the resizer enable then user can \
					 not provide output address\n");
				return -EINVAL;
			}
			if (resizer_no_0)
				write_out_addr(0, arg->out_buff.offset);
			if (resizer_no_1)
				write_out_addr(1, arg->out_buff.offset);
		}
	} else if (arg->in_buff.index > 7) {
		return -EINVAL;
	} else {
		if (resizer_no_0)
			write_out_addr(0,
				       device->out_buff[arg->out_buff.index]->
				       offset);
		if (resizer_no_1 && resizer_no_0)
			write_out_addr(1,
				       device->out_buff[arg->out_buff.index +
							1]->offset);
		if (resizer_no_1 && !resizer_no_0)
			write_out_addr(1,
				       device->out_buff[arg->out_buff.index]->
				       offset);

	}

	regw_ip(0xff, IRQ_EN);
	while (1) {
		utemp = regr_ip(IPIPE_EN);
		if (utemp == 0)
			break;
	}
	regw_ip(1, IPIPE_EN);
	while (1) {
		utemp = regr_if(IPIPEIF_ENABLE);
		if (utemp == 0)
			break;
	};

	regw_if(1, IPIPEIF_ENABLE);
	dev_dbg(ipipe_dev, "ipipe started\n");
	wait_for_completion_interruptible(&device->wfc);
	return 0;

}
int ipipe_release(struct inode *inode, struct file *filp)
{
	/* get the configuratin from private_date member of file */
	struct ipipe_params *config = filp->private_data;
	struct ipipe_device *device = &ipipedevice;
	/* call free_buffers to free memory allocated to buffers */
	if (atomic_dec_and_test(&reference_count)) {
		free_buffers(device);
		/* change the device status to available */
		device->opened = 0;
	}

	/* free the memory allocated to configuration */
	if (config)
		kfree(config);

	/* Assign null to private_data member of file and params
	   member of device */
	filp->private_data = device->params = NULL;

	return 0;
}

int ipipe_mmap(struct file *filp, struct vm_area_struct *vma)
{
	/* get the address of global object of ipipe_device structure */
	struct ipipe_device *device = &ipipedevice;
	int i, flag = 0, shift;
	/* get the page offset */
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	shift = PAGE_SHIFT;
	for (i = 0; i < device->in_numbuffers; i++) {
		if (device->in_buff[i]->offset == offset) {
			flag = 1;
			break;
		}
	}

	/* page offset passed in mmap should one from output buffers */
	if (flag == 0) {
		for (i = 0; i < device->out_numbuffers; i++) {
			if (device->out_buff[i]->offset == offset) {
				flag = 1;
				break;
			}
		}
	}
	/* map buffers address space from kernel space to user space */
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}
int ipipe_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	struct ipipe_params params_t;
	/* struct ipipe_reg_dump reg_addr; */
	int ret = 0;
	/* get the address of global object of ipipe_device structure */
	struct ipipe_device *device = &ipipedevice;
	struct ipipe_params *params = &params_t;
	struct ipipe_convert addr_struct;
	/* struct ipipe_cropsize *crop_struct = NULL; */
	/* Before decoding check for correctness of cmd */
	dev_dbg(ipipe_dev, "ipipe ioctl\n");
	if (_IOC_TYPE(cmd) != IPIPE_IOC_BASE) {
		dev_dbg(ipipe_dev, "Bad command Value \n");
		return -1;
	}
	if (_IOC_NR(cmd) > IPIPE_IOC_MAXNR) {
		dev_dbg(ipipe_dev, "Bad command Value\n");
		return -1;
	}

	/* Verify accesses       */
	if (_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		ret = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
	if (ret) {
		dev_err(ipipe_dev, "access denied\n");
		return -1;	/*error in access */
	}
	/* switch according value of cmd */
	switch (cmd) {

	case IPIPE_QUERYBUF:
		/* call query buffer which will return buffer address */
		down_interruptible(&(device->sem));
		if ((ret =
		     query_buffer(device, (struct ipipe_buffer *)arg)) != 0) {
			up(&(device->sem));
			return ret;
		}
		up(&(device->sem));
		break;
	case IPIPE_REQBUF:
		/* call request buffer to allocate buffers */
		down_interruptible(&(device->sem));
		if ((ret =
		     request_buffer(device,
				    (struct ipipe_reqbufs *)arg)) != 0) {
			up(&(device->sem));
			return ret;
		}

		up(&(device->sem));
		break;
	case IPIPE_SET_PARAM:
		/*set the parametres */

		down_interruptible(&(device->sem));
		/*default parameters */
		if ((void *)arg == NULL) {
			params = &param_def;
			dev_dbg(ipipe_dev, "NULL parameters.\n");
		} else {
			/* copy the parameters to the configuration */
			if (copy_from_user
			    (params, (struct ipipe_params *)arg,
			     sizeof(struct ipipe_params))) {
				/* if it fails return error */
				up(&(device->sem));
				return -EFAULT;
			}
		}

		/* test for register write OK */
		if ((ret = validate_params(params)) != 0) {
			dev_dbg(ipipe_dev, "Error in validate \n");
			up(&(device->sem));
			return ret;
		}
		dev_dbg(ipipe_dev, "Pass validation.\n");

		/* copy the values to device params */
		if (device->params)
			memcpy(device->params, params,
			       sizeof(struct ipipe_params));
		else {
			up(&(device->sem));
			dev_dbg(ipipe_dev,
				"invalid device parameter buffer.\n");
			return -EINVAL;
		}
		if ((ret = ipipe_hw_setup(device->params)) != 0) {
			dev_dbg(ipipe_dev, "Error in hardware set up\n");
			up(&(device->sem));
			return ret;
		}
		up(&(device->sem));
		break;

	case IPIPE_GET_PARAM:

		down_interruptible(&(device->sem));
		/* copy the parameters from the configuration */
		if (copy_to_user((struct ipipe_params *)arg, (device->params),
				 sizeof(struct ipipe_params))) {
			/* if copying fails return error */

			ret = -EFAULT;
		}
		up(&(device->sem));
		break;

	case IPIPE_START:

		down_interruptible(&(device->sem));
		dev_dbg(ipipe_dev, "ipipe startioctl\n");
		/* copy the parameters to the configuration */
		if (copy_from_user
		    (&addr_struct, (struct ipipe_convert *)arg,
		     sizeof(struct ipipe_convert)))
			/* if it fails return error */
		{
			up(&(device->sem));
			return -EFAULT;
		}
		/*ret = ipipe_hw_setup(device->params);*/
		if ((ret = ipipe(device, &addr_struct)) != 0) {
			up(&(device->sem));
			return ret;
		}
		up(&(device->sem));

		break;
	default:
		ret = -EINVAL;
	}

	return ret;

}
static void ipipe_platform_release(struct device *device)
{
	/* This is called when the reference count goes to zero */
}

static int ipipe_probe(struct device *device)
{
	ipipe_dev = device;
	return 0;
}

static int ipipe_remove(struct device *device)
{
	return 0;
}

/* global variable of type file_operations containing function 
pointers of file operations */
static struct file_operations ipipe_fops = {
	.owner = THIS_MODULE,
	.open = ipipe_open,
	.release = ipipe_release,
	.mmap = ipipe_mmap,
	.ioctl = ipipe_ioctl,
};

/* global variable of type cdev to register driver to the kernel */
static struct cdev cdev;

/* global variable which keeps major and minor number of the driver in it */
static dev_t dev;

static struct device_driver ipipe_driver = {
	.name = "dm355_ipipe",
	.bus = &platform_bus_type,
	.probe = ipipe_probe,
	.remove = ipipe_remove,
};
static struct platform_device ipipe_pt_device = {
	.name = "dm355_ipipe",
	.id = 2,
	.dev = {
		.release = ipipe_platform_release,
		}
};
int __init ipipe_init(void)
{
	int result;

	/* Register the driver in the kernel */
	/* dynmically get the major number for the driver using
	   alloc_chrdev_region function */
	result = alloc_chrdev_region(&dev, 0, 1, DRIVERNAME);

	/* if it fails return error */
	if (result < 0) {
		dev_dbg(ipipe_dev, "DM355IPIPE: Module intialization \
                failed. could not register character device\n");
		return -ENODEV;
	}
	printk(KERN_INFO "ipipe major#: %d, minor# %d\n", MAJOR(dev),
	       MINOR(dev));

	/* initialize cdev with file operations */
	cdev_init(&cdev, &ipipe_fops);

	cdev.owner = THIS_MODULE;
	cdev.ops = &ipipe_fops;

	/* add cdev to the kernel */
	result = cdev_add(&cdev, dev, 1);

	if (result) {
		unregister_chrdev_region(dev, 1);
		dev_dbg(ipipe_dev, "DM355 IPIPE: Error adding \
		DM355 IPIPE .. error no:%d\n", result);
		return -EINVAL;
	}

	/* register driver as a platform driver */
	if (driver_register(&ipipe_driver) != 0) {
		unregister_chrdev_region(dev, 1);
		cdev_del(&cdev);
		return -EINVAL;
	}

	/* Register the drive as a platform device */
	if (platform_device_register(&ipipe_pt_device) != 0) {
		driver_unregister(&ipipe_driver);
		unregister_chrdev_region(dev, 1);
		cdev_del(&cdev);
		return -EINVAL;
	}

	ipipe_class = class_simple_create(THIS_MODULE, "dm355_ipipe");
	if (!ipipe_class) {
		printk("ipipe_init: error in creating device class\n");
		driver_unregister(&ipipe_driver);
		platform_device_unregister(&ipipe_pt_device);
		unregister_chrdev_region(dev, 1);
		unregister_chrdev(MAJOR(dev), DRIVERNAME);
		cdev_del(&cdev);
		return -EINVAL;
	}
	/* register simple device class */
	class_simple_device_add(ipipe_class, dev, NULL, "dm355_ipipe");

	/* Set up the Interrupt handler for PRVINT interrupt */
	/*request irq 8(sdr) intt */
	result = request_irq(IRQ_DM355_IPIPE_SDR, ipipe_isr, SA_INTERRUPT,
			     "dm355_ipipe", (void *)&ipipedevice);

	if (result < 0) {
		dev_dbg(ipipe_dev, "ipipe_init:cannot get irq\n");

		unregister_chrdev_region(dev, 1);
		class_simple_device_remove(dev);
		class_simple_destroy(ipipe_class);
		driver_unregister(&ipipe_driver);
		platform_device_unregister(&ipipe_pt_device);
		cdev_del(&cdev);
		return -EINVAL;
	}

	ipipedevice.opened = 0;
	printk(KERN_INFO "ipipe driver registered\n");

	return 0;
}

void __exit ipipe_cleanup(void)
{
	/* remove major number allocated to this driver */
	unregister_chrdev_region(dev, 1);

	/* Remove platform driver */
	driver_unregister(&ipipe_driver);

	/* remove simple class device */
	class_simple_device_remove(dev);

	/* destroy simple class */
	class_simple_destroy(ipipe_class);

	/* remove platform device */
	platform_device_unregister(&ipipe_pt_device);

	/* disable interrupt */
	free_irq(IRQ_DM355_IPIPE_SDR, &ipipedevice);
	cdev_del(&cdev);

	/* unregistering the driver from the kernel */
	unregister_chrdev(MAJOR(dev), DRIVERNAME);
}

module_init(ipipe_init);
module_exit(ipipe_cleanup);
MODULE_LICENSE("GPL");
