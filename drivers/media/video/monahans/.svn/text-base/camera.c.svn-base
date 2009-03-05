/*
   Copyright (C) 2005, Intel Corporation.
   Copyright (C) 2006, Marvell International Ltd.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * Abstract:
 *	contains all primitive functions for camera driver
 * Notes:
 *	Only valid for processor code named Monahans.
 */
#include <linux/config.h>
#include <asm/errno.h>
#include <asm/arch/mfp.h>
#include <asm/arch/mhn_gpio.h>
#include <asm/arch/hardware.h>
#include <asm/arch/pxa-regs.h>

#include "camera.h"
#include "ci.h"

/*
 * Declarations
 */

#define SINGLE_DESCRIPTOR_TRANSFER_MAX  4096
#define SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX  ((1<<21) - 32)

/* there is a QCI bug that QCI can not branch to next descriptor upon the
 * current frame is done. instead, it will branch to next descriptor once
 * current descriptor is done.
 * workaround: assert branch only when last descriptor of current frame
 * is loaded */
#define  CI_DMAC_BRANCH_BUG

/*
 * Define macroses to shorten access to descriptors:
 * p -- pointer to camera_frame_buffer_info_t
 * chan -- 0 for ch0_dma_desc_XXX_addr, ...
 * vir_or_phy -- vir for chX_dma_desc_vir_addr, phy for chX_dma_desc_phy_addr
 * first_or_last -- FIRST for FIRST_DMA_DESC_FOR_REAL_XXX_INX, ...
 * real_or_phantom -- REAL XXX_DMA_DESC_FOR_REAL_BUFFER_INX, ...
 */

#define DESCR(p, chan, vir_or_phy, first_or_last, real_or_phantom) \
       ((p)->ch ## chan ## _dma_desc_ ## vir_or_phy ## _addr \
       [first_or_last ## _DMA_DESC_FOR_ ## real_or_phantom ## _BUFFER_INX])


/* map of camera image format (camera.h) ==> capture interface format (ci.h) */
static const int FORMAT_MAPPINGS[] = {
	/* RAW */
	CI_RAW8,
	CI_RAW9,
	CI_RAW10,

	/* RGB */
	CI_RGB444,
	CI_RGB555,
	CI_RGB565,
	CI_RGB666_PACKED,          /* RGB Packed */
	CI_RGB666,
	CI_RGB888_PACKED,
	CI_RGB888,
	CI_RGBT555_0,              /* RGB+Transparent bit 0 */
	CI_RGBT888_0,
	CI_RGBT555_1,              /* RGB+Transparent bit 1 */
	CI_RGBT888_1,

	CI_INVALID_FORMAT,
	CI_YCBCR422,               /* YCBCR */
	CI_YCBCR422_PLANAR,        /* YCBCR Planar */
	CI_INVALID_FORMAT,
#if defined(CONFIG_PXA310)
	CI_INVALID_FORMAT,
	CI_YCBCR420,
	CI_YCBCR420_PLANAR
#else
	CI_INVALID_FORMAT
#endif
};

static void cam_configure_dmac(p_camera_context_t camera_context);
static int cam_start_capture(p_camera_context_t camera_context,
		unsigned int frames);
static void cam_set_stop_state(p_camera_context_t camera_context);

/*
 * Private functions
 */
static void cam_configure_dmac(p_camera_context_t camera_context)
{
	unsigned int des_physical;
	camera_frame_buffer_queue_t *queue;

	queue = (camera_context->capture_mode == CAMERA_MODE_VIDEO)?
		&(camera_context->video_capture_buffer_queue):
			&(camera_context->still_capture_buffer_queue);

	if (queue->head == NULL)
		return;

	des_physical = DESCR(queue->head, 0, phy, FIRST, REAL);
	ci_dma_load_descriptor(des_physical, CI_DMA_CHANNEL_0);

	if (camera_context->fifo1_transfer_size) {
		des_physical = DESCR(queue->head, 1, phy, FIRST, REAL);
		ci_dma_load_descriptor(des_physical, CI_DMA_CHANNEL_1);
	}

	if (camera_context->fifo2_transfer_size) {
		des_physical = DESCR(queue->head, 2, phy, FIRST, REAL);
		ci_dma_load_descriptor(des_physical, CI_DMA_CHANNEL_2);
	}

	camera_context->dma_running = 1;
}

static int cam_start_capture(p_camera_context_t camera_context, unsigned int frames)
{
	int   status;

	/* Disable CI */
	cam_set_stop_state(camera_context);

	/* clear ci fifo */
	ci_reset_fifo();
	ci_clear_interrupt_status(0xFFFFFFFF);

	if (!camera_context->ci_disable_complete) {
		ci_disable(1, 1);
		ci_disable_complete();
	}

	cam_configure_dmac(camera_context);

	ci_enable();

	/* start capture */
	status = camera_context->camera_functions->
			start_capture(camera_context, frames);
	return status;
}

/*
 * Init/Deinit APIs
 */

extern void zylonite_enable_cif_pins(void);

int mcam_init(p_camera_context_t camera_context)
{
	int   status = 0;

#ifdef DEBUG_PARAM_CHECK

	/* parameter check */
	if (camera_context->sensor_type > CAMERA_TYPE_MAX)
		return -EINVAL;

	/* check the function dispatch table according to the sensor type */
	if (!camera_context->camera_functions)
		return -EINVAL;

	if (!camera_context->camera_functions->init ||
			!camera_context->camera_functions->deinit ||
			!camera_context->camera_functions->set_capture_format ||
			!camera_context->camera_functions->start_capture ||
			!camera_context->camera_functions->stop_capture)
		return -EINVAL;
#endif

	/* initialize some camera used parameters */
	camera_context->capture_input_width = 0;
	camera_context->capture_input_height = 0;
	camera_context->capture_output_width = 0;
	camera_context->capture_output_height = 0;
	camera_context->capture_input_format = CAMERA_IMAGE_FORMAT_MAX + 1;
	camera_context->capture_output_format = CAMERA_IMAGE_FORMAT_MAX + 1;
	camera_context->fifo0_transfer_size = 0;
	camera_context->fifo1_transfer_size = 0;
	camera_context->fifo2_transfer_size = 0;
	camera_context->fifo3_transfer_size = 0;
	camera_context->video_fifo0_transfer_size = 0;
	camera_context->video_fifo1_transfer_size = 0;
	camera_context->video_fifo2_transfer_size = 0;
	camera_context->still_fifo0_transfer_size = 0;
	camera_context->still_fifo1_transfer_size = 0;
	camera_context->still_fifo2_transfer_size = 0;
	camera_context->frame_buffer_number = 0;
	camera_context->video_capture_buffer_queue.head = NULL;
	camera_context->video_capture_buffer_queue.tail = NULL;
	camera_context->still_capture_buffer_queue.head = NULL;
	camera_context->still_capture_buffer_queue.tail = NULL;
	camera_context->psu_enable = 0;
	camera_context->cgu_enable = 0;
	camera_context->ssu_scale = CI_SSU_SCALE_DISABLE;
	camera_context->cmu_usage = CI_CMU_DISABLE;
	camera_context->dma_running = 0;

	/* MFP pins init */
	zylonite_enable_cif_pins();

	/* UTMI_SWITCH must be set to low and
	 * the UTMI_TEST_EN should be set to output (low)
	 */
	mhn_gpio_set_direction(MFP_UTMI_SWITCH,  GPIO_DIR_OUT);
	mhn_gpio_set_direction(MFP_UTMI_TEST_EN, GPIO_DIR_OUT);
	mhn_gpio_set_level(MFP_UTMI_SWITCH,  GPIO_LEVEL_LOW);
	mhn_gpio_set_level(MFP_UTMI_TEST_EN, GPIO_LEVEL_HIGH);

	/* set two gpio pin direction as output
	 * to control the power of two sensors.
	 */
	mhn_gpio_set_direction(MFP_CIF_HI_PWDN_GPI0, GPIO_DIR_OUT);
	mhn_gpio_set_direction(MFP_CIF_LO_PWDN_GPI0, GPIO_DIR_OUT);
	/* set two gpio pin output as HI to power off two sensors. */
	mhn_gpio_set_level(MFP_CIF_HI_PWDN_GPI0, GPIO_LEVEL_HIGH);
	mhn_gpio_set_level(MFP_CIF_LO_PWDN_GPI0, GPIO_LEVEL_HIGH);

	/* capture interface init */
	ci_init();

	/* sensor init */
	status = camera_context->camera_functions->init(camera_context);
	if (status) {
		goto camera_init_err;
	}

	/* set frame rate */
	mcam_set_capture_frame_rate(camera_context);

	return 0;

camera_init_err:
	mcam_deinit(camera_context);
	return -EIO;
}

int mcam_deinit(p_camera_context_t camera_context)
{
	int status;

	/* deinit sensor */
	status = camera_context->camera_functions->deinit(camera_context);

	/* capture interface deinit */
	ci_deinit();

	return status;
}

/*
 * Capture APIs
 */

/* Set the image format */
int mcam_set_capture_format(p_camera_context_t camera_context)
{
	int status;
	unsigned int capture_input_width, capture_input_height;
	int ci_input_format, ci_output_format;
	unsigned int scale;

	/* caculate some parameters according to the driver parameters */

	/* get basic capture format and resolution info */
	if (camera_context->capture_mode == CAMERA_MODE_VIDEO) {
		camera_context->capture_input_format =
			camera_context->video_capture_input_format;
		camera_context->capture_output_format =
			camera_context->video_capture_output_format;
		camera_context->capture_output_width =
			camera_context->video_capture_width;
		camera_context->capture_output_height =
			camera_context->video_capture_height;
		camera_context->fifo0_transfer_size =
			camera_context->video_fifo0_transfer_size;
		camera_context->fifo1_transfer_size =
			camera_context->video_fifo1_transfer_size;
		camera_context->fifo2_transfer_size =
			camera_context->video_fifo2_transfer_size;
	} else {
		camera_context->capture_input_format =
			camera_context->still_capture_input_format;
		camera_context->capture_output_format =
			camera_context->still_capture_output_format;
		camera_context->capture_output_width =
			camera_context->still_capture_width;
		camera_context->capture_output_height =
			camera_context->still_capture_height;
		camera_context->fifo0_transfer_size =
			camera_context->still_fifo0_transfer_size;
		camera_context->fifo1_transfer_size =
			camera_context->still_fifo1_transfer_size;
		camera_context->fifo2_transfer_size =
			camera_context->still_fifo2_transfer_size;
	}

#ifdef DEBUG_PARAM_CHECK
	if (camera_context->capture_input_format >  CAMERA_IMAGE_FORMAT_MAX ||
			camera_context->capture_output_format >
			CAMERA_IMAGE_FORMAT_MAX)
		return -EINVAL;
#endif

	/* determine whether to enable PSU */
	camera_context->psu_enable =
		(camera_context->capture_input_format <=
		CAMERA_IMAGE_FORMAT_RAW10) ? 1 : 0;

	/* determine whether to enable CGU */
	camera_context->cgu_enable =
		((camera_context->capture_input_format <=
		CAMERA_IMAGE_FORMAT_RAW10) &&
		(camera_context->capture_input_format !=
		camera_context->capture_output_format)) ?
		1 : 0;

	/* determine how to use SSU */
	camera_context->ssu_scale =
		(camera_context->capture_mode == CAMERA_MODE_VIDEO)?
		(CI_SSU_SCALE)camera_context->video_capture_scale:
		(CI_SSU_SCALE)camera_context->still_capture_scale;

	/* determine how to use CMU */
	if ((camera_context->capture_input_format <=
			CAMERA_IMAGE_FORMAT_RAW10) &&
			(camera_context->capture_output_format >=
			CAMERA_IMAGE_FORMAT_RGB444) &&
			(camera_context->capture_output_format <=
			CAMERA_IMAGE_FORMAT_RGBT888_1)) {
		camera_context->cmu_usage = CI_CMU_OUTPUT_RGB;
	} else if ((camera_context->capture_input_format <=
			CAMERA_IMAGE_FORMAT_RAW10) &&
			(camera_context->capture_output_format >=
			CAMERA_IMAGE_FORMAT_YCBCR400) &&
			(camera_context->capture_output_format <=
#if defined(CONFIG_PXA310)
			CAMERA_IMAGE_FORMAT_YCBCR420_PLANAR)) {
#else
			CAMERA_IMAGE_FORMAT_YCBCR444_PLANAR)) {
#endif
		camera_context->cmu_usage = CI_CMU_OUTPUT_YUV;
	} else {
		camera_context->cmu_usage = CI_CMU_DISABLE;
	}
#if defined(CONFIG_PXA310)
	if (camera_context->capture_output_format == CAMERA_IMAGE_FORMAT_YCBCR420_PACKED
		|| camera_context->capture_output_format == CAMERA_IMAGE_FORMAT_YCBCR420_PLANAR)
	{
		camera_context->ycbcr_ds = CI_YUV_420_DOWN_SAMPLE;
	} else {
		camera_context->ycbcr_ds = CI_NO_DOWN_SAMPLE;
	}
#endif

#ifdef DEBUG_PARAM_CHECK
	if ((camera_context->cmu_usage != CI_CMU_DISABLE) &&
			(camera_context->ssu_scale == CI_SSU_SCALE_DISABLE)) {
		return -EINVAL;
	}
#endif

	/* caculate the input resolution */
	capture_input_width = camera_context->capture_output_width;
	capture_input_height = camera_context->capture_output_height;

	if (camera_context->ssu_scale != CI_SSU_SCALE_DISABLE) {
		scale = (camera_context->ssu_scale == CAMERA_CAPTURE_SCALE_HALF) ?
				2:4;
		capture_input_width = (camera_context->capture_input_format <=
			CAMERA_IMAGE_FORMAT_RAW10) ?
			(capture_input_width*scale + 2) :
			(capture_input_width*scale + 2);

		capture_input_height = (camera_context->capture_input_format <=
			CAMERA_IMAGE_FORMAT_RAW10) ?
			(capture_input_height*scale + 2) :
			(capture_input_height*scale + 1);
	}

	if (camera_context->cmu_usage != CI_CMU_DISABLE) {
		if (CAMERA_CAPTURE_SCALE_HALF == camera_context->ssu_scale) {
			capture_input_height = capture_input_height + 2 + 1;
		} else if (CI_SSU_SCALE_QUARTER == camera_context->ssu_scale) {
			capture_input_height = capture_input_height + 4 + 3;
		} else if (CI_SSU_SCALE_DISABLE == camera_context->ssu_scale) {
			capture_input_height = capture_input_height + 1;
		}
	}

	camera_context->capture_input_width = capture_input_width;
	camera_context->capture_input_height = capture_input_height;

	/* set sensor setting */
	status = camera_context->camera_functions->
			set_capture_format(camera_context);
	if (status) {
		return status;
	}

	/* set CI setting */

	/* set scale to SSU */
	ci_ssu_set_scale(camera_context->ssu_scale);

	/* enable/disable CMU */
	ci_cmu_enable(camera_context->cmu_usage);

	/* set CI image format */
	ci_input_format =
		FORMAT_MAPPINGS[camera_context->capture_input_format];
	ci_output_format =
		FORMAT_MAPPINGS[camera_context->capture_output_format];

#ifdef DEBUG_PARAM_CHECK
	if (ci_input_format == CI_INVALID_FORMAT ||
			ci_output_format == CI_INVALID_FORMAT)
		return -EINVAL;
#endif

	ci_set_image_format(ci_input_format, ci_output_format);
#if defined(CONFIG_PXA310)
	ci_set_ycbcr_420_down_sample (camera_context->ycbcr_ds);
#endif

	return 0;
}

/* take a picture and copy it into the frame buffers */
int mcam_capture_still_image(p_camera_context_t camera_context)
{
	int status;
	status = cam_start_capture(camera_context, 1);
	return status;
}

/* capture motion video and copy them to the frame buffers */
int mcam_start_video_capture(p_camera_context_t camera_context)
{
	int status;
	status = cam_start_capture(camera_context, 0);
	return status;
}

static void cam_set_stop_state(p_camera_context_t camera_context)
{
	/* Clear the counts and the current status. */
	camera_context->ci_disable_complete = 0;
	ci_clear_interrupt_status(0xFFFFFFFF);


	/* Depending upon the timing of using the command to change to single
	 * frame mode, the OV9640 may issue a VSYNC and 1 HSYNC signal and then
	 * stop.  This would leave the CI in a non-IDLE state.  The camera
	 * interface is now disabled before leaving the stop state.
	 */
	ci_disable(1, 1);
}

/* disable motion video image capture */
void mcam_stop_video_capture(p_camera_context_t camera_context)
{
	int status;

	/* stop capture */
	status = camera_context->camera_functions->stop_capture(camera_context);

	/* Set up to receive interrupts and track the sof, eof,
	 * and eol after the shutdown.
	 */
	cam_set_stop_state(camera_context);

	/* clear ci fifo */
	ci_reset_fifo();

	/* empty the capture queue.
	 * WARNING: the driver must re-submit all of buffers into
	 * queue before it starts video/still capture next time!
	 */
	camera_context->dma_running = 0;
}

/*
 * Flow Control APIs
 */

/* Get the buffer size and the DMA descriptors memory size for this buffer */
int mcam_get_buffer_size(
		p_camera_context_t   camera_context,
		int   buffer_type,
		int  *buffer_size,
		int  *buffer_dma_desc_mem_size)
{
	unsigned int    capture_output_format;
	unsigned int    capture_output_width;
	unsigned int    capture_output_height;
	unsigned int    frame_size;
	unsigned int    fifo0_transfer_size;
	unsigned int    fifo1_transfer_size;
	unsigned int    fifo2_transfer_size;
	unsigned int    fifo0_num_descriptors = 0;
	unsigned int    fifo1_num_descriptors = 0;
	unsigned int    fifo2_num_descriptors = 0;
	unsigned int    fifo0_phantom_num_descriptors = 0;
	unsigned int    fifo1_phantom_num_descriptors = 0;
	unsigned int    fifo2_phantom_num_descriptors = 0;
	unsigned int    num_descriptors_for_buffer;


	/* caculate the fifo0~2 transfer size */
	capture_output_format = (buffer_type == VIDEO_CAPTURE_BUFFER) ?
		camera_context->video_capture_output_format:
		camera_context->still_capture_output_format;

	capture_output_width  = (buffer_type == VIDEO_CAPTURE_BUFFER) ?
		camera_context->video_capture_width:
		camera_context->still_capture_width;

	capture_output_height = (buffer_type == VIDEO_CAPTURE_BUFFER) ?
		camera_context->video_capture_height:
		camera_context->still_capture_height;

	switch (capture_output_format) {
		case CAMERA_IMAGE_FORMAT_RAW10:
			frame_size =
				capture_output_width * capture_output_height * 2;
			fifo0_transfer_size = frame_size;
			fifo1_transfer_size = 0;
			fifo2_transfer_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_RAW9:
			frame_size =
				capture_output_width * capture_output_height * 2;
			fifo0_transfer_size = frame_size;
			fifo1_transfer_size = 0;
			fifo2_transfer_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_RAW8:
			frame_size =
				capture_output_width * capture_output_height;
			fifo0_transfer_size = frame_size;
			fifo1_transfer_size = 0;
			fifo2_transfer_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_RGB565:
			frame_size =
				capture_output_width * capture_output_height * 2;
			fifo0_transfer_size = frame_size;
			fifo1_transfer_size = 0;
			fifo2_transfer_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_RGB888_PACKED:
			frame_size =
				capture_output_width * capture_output_height * 3;
			fifo0_transfer_size = frame_size;
			fifo1_transfer_size = 0;
			fifo2_transfer_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_YCBCR422_PACKED:
			frame_size =
				capture_output_width * capture_output_height * 2;
			fifo0_transfer_size = frame_size;
			fifo1_transfer_size = 0;
			fifo2_transfer_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR:
			frame_size =
				capture_output_width * capture_output_height * 2;
			fifo0_transfer_size = frame_size / 2;
			fifo1_transfer_size = frame_size / 4;
			fifo2_transfer_size = frame_size / 4;
			break;
#if defined(CONFIG_PXA310)
		case CAMERA_IMAGE_FORMAT_YCBCR420_PACKED:
			frame_size =
				capture_output_width * capture_output_height * 3 / 2;
			fifo0_transfer_size = frame_size;
			fifo1_transfer_size = 0;
			fifo2_transfer_size = 0;
			break;
		case CAMERA_IMAGE_FORMAT_YCBCR420_PLANAR:
			frame_size =
				capture_output_width * capture_output_height * 2;
			fifo0_transfer_size = frame_size / 2;
			fifo1_transfer_size = frame_size / 8;
			fifo2_transfer_size = frame_size / 8;
			break;
#endif
		default:
			return -EINVAL;
			break;
	}

	if (buffer_type == VIDEO_CAPTURE_BUFFER) {
		camera_context->video_fifo0_transfer_size = fifo0_transfer_size;
		camera_context->video_fifo1_transfer_size = fifo1_transfer_size;
		camera_context->video_fifo2_transfer_size = fifo2_transfer_size;
	} else {
		camera_context->still_fifo0_transfer_size = fifo0_transfer_size;
		camera_context->still_fifo1_transfer_size = fifo1_transfer_size;
		camera_context->still_fifo2_transfer_size = fifo2_transfer_size;
	}

	/* caculate the buffer size and descriptor memory size */
	fifo0_num_descriptors = (fifo0_transfer_size +
		SINGLE_DESCRIPTOR_TRANSFER_MAX - 1) /
		SINGLE_DESCRIPTOR_TRANSFER_MAX;

	fifo1_num_descriptors = (fifo1_transfer_size +
		SINGLE_DESCRIPTOR_TRANSFER_MAX - 1) /
		SINGLE_DESCRIPTOR_TRANSFER_MAX;

	fifo2_num_descriptors = (fifo2_transfer_size +
		SINGLE_DESCRIPTOR_TRANSFER_MAX - 1) /
		SINGLE_DESCRIPTOR_TRANSFER_MAX;

	fifo0_phantom_num_descriptors = (fifo0_transfer_size +
		SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX - 1) /
		SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX;

	fifo1_phantom_num_descriptors = (fifo1_transfer_size +
		SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX - 1) /
		SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX;

	fifo2_phantom_num_descriptors = (fifo2_transfer_size +
		SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX - 1) /
		SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX;

	if (camera_context->align_type == YUV_NO_PADDING)
		goto no_padding;

	num_descriptors_for_buffer = fifo0_num_descriptors +
			fifo1_num_descriptors + fifo2_num_descriptors;

	*buffer_size = num_descriptors_for_buffer *
			SINGLE_DESCRIPTOR_TRANSFER_MAX;

	/* descriptors memory for real frame buffer and phantom buffer */
	*buffer_dma_desc_mem_size = sizeof(struct ci_dma_descriptor)*(
			fifo0_num_descriptors + fifo0_phantom_num_descriptors +
			fifo1_num_descriptors + fifo1_phantom_num_descriptors +
			fifo2_num_descriptors + fifo2_phantom_num_descriptors);

	return 0;

no_padding:
	num_descriptors_for_buffer = (fifo0_transfer_size +
			fifo1_transfer_size + fifo2_transfer_size +
			SINGLE_DESCRIPTOR_TRANSFER_MAX - 1) / SINGLE_DESCRIPTOR_TRANSFER_MAX + 2;

	*buffer_size = fifo0_transfer_size + fifo1_transfer_size +
		fifo2_transfer_size;

	/* descriptors memory for real frame buffer and phantom buffer */
	*buffer_dma_desc_mem_size = sizeof(struct ci_dma_descriptor)*(
			num_descriptors_for_buffer + fifo0_phantom_num_descriptors
			+ fifo1_phantom_num_descriptors + fifo2_phantom_num_descriptors);

	return 0;
}

/* Add the buffer into the buffer pool
 * and generate the DMA chains for the buffer
 */
int mcam_prepare_buffer(
		p_camera_context_t camera_context,
		void *buf_virt_addr,
		int *buf_phy_addr_array,
		int buf_phy_addr_array_size,
		int buffer_size,
		int buffer_type,
		void *dma_desc_virt_addr,
		int *dma_desc_phy_addr_array,
		int *buffer_id,
		void **pY,
		void **pCb,
		void **pCr)
{
	camera_frame_buffer_info_t *buffer_info;
	int buffer_index, des_transfer_size, remain_size;
	volatile struct ci_dma_descriptor *cur_des_virtual;
	unsigned int buffer_page_seq, dma_descriptor_seq;
	unsigned int fifo0_transfer_size;
	unsigned int fifo1_transfer_size;
	unsigned int fifo2_transfer_size;
	unsigned int fifo0_num_descriptors = 0;
	unsigned int fifo1_num_descriptors = 0;
	unsigned int fifo2_num_descriptors = 0;
	unsigned int fifo0_phantom_num_descriptors = 0;
	unsigned int fifo1_phantom_num_descriptors = 0;
	unsigned int fifo2_phantom_num_descriptors = 0;

	unsigned int fifo0_last_buf_transfer_size = 0;
	unsigned int fifo1_last_buf_transfer_size = 0;
	unsigned int offset;

	/* compute descriptor number for fifo0~fifo2 */
	fifo0_transfer_size = (buffer_type == VIDEO_CAPTURE_BUFFER)?
		camera_context->video_fifo0_transfer_size:
		camera_context->still_fifo0_transfer_size;

	fifo1_transfer_size = (buffer_type == VIDEO_CAPTURE_BUFFER)?
		camera_context->video_fifo1_transfer_size:
		camera_context->still_fifo1_transfer_size;

	fifo2_transfer_size = (buffer_type == VIDEO_CAPTURE_BUFFER)?
		camera_context->video_fifo2_transfer_size:
		camera_context->still_fifo2_transfer_size;

	/* calculate how many descriptors are needed per frame */
	fifo0_num_descriptors = 0;
	fifo1_num_descriptors = 0;
	fifo2_num_descriptors = 0;
	fifo0_phantom_num_descriptors = 0;
	fifo1_phantom_num_descriptors = 0;
	fifo2_phantom_num_descriptors = 0;

	/* add the buffer into the buffer pool */
	buffer_index = (*buffer_id == -1) ?
				camera_context->frame_buffer_number:*buffer_id;
	buffer_info = &(camera_context->master_frame_buffer_list[buffer_index]);
	buffer_info->frame_id = buffer_index;
	buffer_info->buffer_size = buffer_size;
	buffer_info->dma_descriptors_virtual =
		(struct ci_dma_descriptor *)dma_desc_virt_addr;
	if (*buffer_id == -1)
		camera_context->frame_buffer_number++;

	/* generate the DMA chains */
	dma_descriptor_seq = 0;
	buffer_page_seq = 0;
	cur_des_virtual = (struct ci_dma_descriptor*)dma_desc_virt_addr;

	/* 1. generate fifo0 DMA chain */
	if (fifo0_transfer_size) {
		/* 1.1 Build the primary DMA chain */
		remain_size = fifo0_transfer_size;
		while(remain_size) {
			/* set descriptor */
			fifo0_num_descriptors++;
			if (remain_size > SINGLE_DESCRIPTOR_TRANSFER_MAX)
				des_transfer_size =
					SINGLE_DESCRIPTOR_TRANSFER_MAX;
			else
				des_transfer_size = remain_size;

			cur_des_virtual->ddadr =
				dma_desc_phy_addr_array[++dma_descriptor_seq];

			 /* FIFO0 physical address */
			cur_des_virtual->dsadr = __PREG_3(CIBR0);
			cur_des_virtual->dtadr =
				(unsigned)buf_phy_addr_array[buffer_page_seq++];
			cur_des_virtual->dcmd =
				des_transfer_size | CI_DMAC_DCMD_INC_TRG_ADDR;

			if (remain_size == fifo0_transfer_size) {
				DESCR(buffer_info, 0, vir, FIRST, REAL) = cur_des_virtual;
				DESCR(buffer_info, 0, phy, FIRST, REAL) =
					(u32)dma_desc_phy_addr_array[dma_descriptor_seq - 1];
				/* record address of the first real descriptor */
			}

			if (remain_size <= SINGLE_DESCRIPTOR_TRANSFER_MAX) {
				buffer_info->ch0_dma_desc_num = fifo0_num_descriptors;

				DESCR(buffer_info, 0, vir, LAST, REAL) =
					cur_des_virtual;
				DESCR(buffer_info, 0, phy, LAST, REAL) =
					(u32)dma_desc_phy_addr_array[dma_descriptor_seq - 1];
#ifdef CI_DMAC_BRANCH_BUG
				cur_des_virtual->dcmd |= CI_DMAC_DCMD_SOF_IRQ_EN;
#endif
				fifo0_last_buf_transfer_size = des_transfer_size;

				/* record address of the last real descriptor */
			}

			/* advance pointers */
			remain_size -= des_transfer_size;
			cur_des_virtual++;
		}

		/* 1.2 Build the phantom DMA chain */
		remain_size = fifo0_transfer_size;
		while(remain_size) {
			/* set descriptor */
			fifo0_phantom_num_descriptors++;
			if (remain_size > SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX)
				des_transfer_size =
					SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX;
			else
				des_transfer_size = remain_size;

			cur_des_virtual->ddadr =
				dma_desc_phy_addr_array[++dma_descriptor_seq];
			/* FIFO0 physical address */
			cur_des_virtual->dsadr = __PREG_3(CIBR0);
			cur_des_virtual->dtadr =
				camera_context->phantom_buffer_physical;
			cur_des_virtual->dcmd = des_transfer_size;

			if (remain_size == fifo0_transfer_size) {
				/* The address of the first phantom descriptor*/
				DESCR(buffer_info, 0, vir, FIRST, PHANTOM) =
					cur_des_virtual;
				DESCR(buffer_info, 0, phy, FIRST, PHANTOM) =
					(u32)dma_desc_phy_addr_array
					[dma_descriptor_seq - 1];
			}

			if (remain_size <= SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX){
				buffer_info->ch0_phantom_dma_desc_num =
					fifo0_phantom_num_descriptors;

				DESCR(buffer_info, 0, vir, LAST, PHANTOM) =
					cur_des_virtual;
				DESCR(buffer_info, 0, phy, LAST, PHANTOM) =
					(u32)dma_desc_phy_addr_array
					[dma_descriptor_seq - 1];
				/* The address of the last phantom descriptor */
				cur_des_virtual->ddadr =
					DESCR(buffer_info, 0, phy, FIRST, PHANTOM);
				/* loop the phantom buffer to itself */

#ifdef CI_DMAC_BRANCH_BUG
				cur_des_virtual->dcmd |= CI_DMAC_DCMD_SOF_IRQ_EN;
#endif

			}

			/* advance pointers */
			remain_size -= des_transfer_size;
			cur_des_virtual++;
		}
	}

	/* 2. generate fifo1 DMA chain */
	if (fifo1_transfer_size) {
		/* 2.1 Build the primary DMA chain */
		remain_size = fifo1_transfer_size;
		while(remain_size) {
			/* set descriptor */
			fifo1_num_descriptors++;
			if (remain_size > SINGLE_DESCRIPTOR_TRANSFER_MAX)
				des_transfer_size =
					SINGLE_DESCRIPTOR_TRANSFER_MAX;
			else
				des_transfer_size = remain_size;

			offset = 0;
			if((camera_context->align_type == YUV_NO_PADDING)
					&& (fifo0_last_buf_transfer_size != SINGLE_DESCRIPTOR_TRANSFER_MAX)
					&& (remain_size == fifo1_transfer_size)){
				buffer_page_seq -= 1;
				des_transfer_size = min(des_transfer_size,
						(int)(SINGLE_DESCRIPTOR_TRANSFER_MAX - fifo0_last_buf_transfer_size));
				offset = fifo0_last_buf_transfer_size;
			}

			cur_des_virtual->ddadr =
				dma_desc_phy_addr_array[++dma_descriptor_seq];

			/* FIFO1 physical address */
			cur_des_virtual->dsadr = __PREG_3(CIBR1);
			cur_des_virtual->dtadr =
				(unsigned)buf_phy_addr_array[buffer_page_seq++] + offset;
			cur_des_virtual->dcmd =
				des_transfer_size | CI_DMAC_DCMD_INC_TRG_ADDR;

			if (remain_size == fifo1_transfer_size) {
				/* The address of the first real descriptor */
				DESCR(buffer_info, 1, vir, FIRST, REAL) =
					cur_des_virtual;
				DESCR(buffer_info, 1, phy, FIRST, REAL) =
					(u32)dma_desc_phy_addr_array
					[dma_descriptor_seq - 1];
			}

			if (remain_size <= SINGLE_DESCRIPTOR_TRANSFER_MAX) {
				/* The address of the last real descriptor */
				buffer_info->ch1_dma_desc_num = fifo1_num_descriptors;
				DESCR(buffer_info, 1, vir, LAST, REAL) =
					cur_des_virtual;
				DESCR(buffer_info, 1, phy, LAST, REAL) =
					(u32)dma_desc_phy_addr_array[dma_descriptor_seq - 1];
				fifo1_last_buf_transfer_size = des_transfer_size;
			}

			/* advance pointers */
			remain_size -= des_transfer_size;
			cur_des_virtual++;
		}

		/* 2.2 Build the phantom DMA chain */
		remain_size = fifo1_transfer_size;
		while (remain_size) {
			/* set descriptor */
			fifo1_phantom_num_descriptors++;
			if (remain_size > SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX)
				des_transfer_size =
					SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX;
			else
				des_transfer_size = remain_size;

			cur_des_virtual->ddadr =
				dma_desc_phy_addr_array[++dma_descriptor_seq];

			/* FIFO1 physical address */
			cur_des_virtual->dsadr = __PREG_3(CIBR1);
			cur_des_virtual->dtadr =
				camera_context->phantom_buffer_physical;
			cur_des_virtual->dcmd = des_transfer_size;

			if (remain_size == fifo1_transfer_size) {
				/* The record address of the
				 * first phantom descriptor
				 */
				DESCR(buffer_info, 1, vir, FIRST, PHANTOM) =
					cur_des_virtual;
				DESCR(buffer_info, 1, phy, FIRST, PHANTOM) =
					(u32)dma_desc_phy_addr_array
					[dma_descriptor_seq - 1];
			}

			if (remain_size <= SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX) {
				/* The address of the last phantom descriptor */
				buffer_info->ch1_phantom_dma_desc_num =
					fifo1_phantom_num_descriptors;
				DESCR(buffer_info, 1, vir, LAST, PHANTOM) =
					cur_des_virtual;
				DESCR(buffer_info, 1, phy, LAST, PHANTOM) =
					(u32)dma_desc_phy_addr_array
					[dma_descriptor_seq - 1];

				/* loop the phantom buffer to itself */
				cur_des_virtual->ddadr =
					DESCR(buffer_info, 1, phy, FIRST, PHANTOM);
			}

			/* advance pointers */
			remain_size -= des_transfer_size;
			cur_des_virtual++;
		}
	}

	/* 3. generate fifo2 DMA chain */
	if (fifo2_transfer_size) {
		/* 3.1 Build the primary DMA chain */
		remain_size = fifo2_transfer_size;
		while(remain_size) {
			/* set descriptor */
			fifo2_num_descriptors++;
			if (remain_size > SINGLE_DESCRIPTOR_TRANSFER_MAX)
				des_transfer_size =
					SINGLE_DESCRIPTOR_TRANSFER_MAX;
			else
				des_transfer_size = remain_size;

			offset = 0;
			if((camera_context->align_type == YUV_NO_PADDING)
					&& (fifo1_last_buf_transfer_size != SINGLE_DESCRIPTOR_TRANSFER_MAX)
					&& (remain_size == fifo2_transfer_size)){
				buffer_page_seq -= 1;
				des_transfer_size = min(des_transfer_size,
						(int)(SINGLE_DESCRIPTOR_TRANSFER_MAX - fifo1_last_buf_transfer_size));
				offset = fifo1_last_buf_transfer_size;
			}

			cur_des_virtual->ddadr =
				dma_desc_phy_addr_array[++dma_descriptor_seq];

			/* FIFO2 physical address */
			cur_des_virtual->dsadr = __PREG_3(CIBR2);
			cur_des_virtual->dtadr =
				(unsigned)buf_phy_addr_array[buffer_page_seq++] + offset;
			cur_des_virtual->dcmd =
				des_transfer_size | CI_DMAC_DCMD_INC_TRG_ADDR;

			if (remain_size == fifo2_transfer_size) {
				/* The address of the first real descriptor */
				DESCR(buffer_info, 2, vir, FIRST, REAL) =
					cur_des_virtual;
				DESCR(buffer_info, 2, phy, FIRST, REAL) =
					(u32)dma_desc_phy_addr_array
					[dma_descriptor_seq - 1];
			}

			if (remain_size <= SINGLE_DESCRIPTOR_TRANSFER_MAX) {
				/* The address of the last real descriptor */
				buffer_info->ch2_dma_desc_num =
					fifo2_num_descriptors;
				DESCR(buffer_info, 2, vir, LAST, REAL) =
					cur_des_virtual;
				DESCR(buffer_info, 2, phy, LAST, REAL) =
					(u32)dma_desc_phy_addr_array
					[dma_descriptor_seq - 1];
			}

			/* advance pointers */
			remain_size -= des_transfer_size;
			cur_des_virtual++;
		}

		/* 3.2 Build the phantom DMA chain */
		remain_size = fifo2_transfer_size;
		while(remain_size) {
			/* set descriptor */
			fifo2_phantom_num_descriptors++;
			if (remain_size > SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX)
				des_transfer_size =
					SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX;
			else
				des_transfer_size = remain_size;

			cur_des_virtual->ddadr =
				dma_desc_phy_addr_array[++dma_descriptor_seq];

			/* FIFO2 physical address */
			cur_des_virtual->dsadr = __PREG_3(CIBR2);
			cur_des_virtual->dtadr =
				camera_context->phantom_buffer_physical;
			cur_des_virtual->dcmd = des_transfer_size;

			if (remain_size == fifo2_transfer_size){
				/* The address of the first phantom descriptor */
				DESCR(buffer_info, 2, vir, FIRST, PHANTOM) =
					cur_des_virtual;
				DESCR(buffer_info, 2, phy, FIRST, PHANTOM) =
					(u32)dma_desc_phy_addr_array
					[dma_descriptor_seq - 1];
			}

			if (remain_size <= SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX) {
				/* The address of the last phantom descriptor */
				buffer_info->ch2_phantom_dma_desc_num =
					fifo2_phantom_num_descriptors;
				DESCR(buffer_info, 2, vir, LAST, PHANTOM) =
					cur_des_virtual;
				DESCR(buffer_info, 2, phy, LAST, PHANTOM) =
					(u32)dma_desc_phy_addr_array
					[dma_descriptor_seq - 1];
				/* loop the phantom buffer to itself */
				cur_des_virtual->ddadr =
					DESCR(buffer_info, 2, phy, FIRST, PHANTOM);
			}

			/* advance pointers */
			remain_size -= des_transfer_size;
			cur_des_virtual++;
		}
	}

	if ((unsigned int)buf_phy_addr_array_size < buffer_page_seq)
		return -EINVAL;

	if(camera_context->align_type == YUV_NO_PADDING) {
		buffer_info->pY = (void*)buf_virt_addr;
		buffer_info->pCb = (fifo1_transfer_size) ?
			(void*)((unsigned int)buffer_info->pY + fifo0_transfer_size ) : 0;
		buffer_info->pCr = (fifo2_transfer_size) ?
			(void*)((unsigned int)buffer_info->pCb + fifo1_transfer_size) : 0;
	} else {
		buffer_info->pY = (void*)buf_virt_addr;
		buffer_info->pCb = (fifo1_transfer_size) ?
			(void*)((unsigned int)buffer_info->pY +
					fifo0_num_descriptors * SINGLE_DESCRIPTOR_TRANSFER_MAX) : 0;
		buffer_info->pCr = (fifo2_transfer_size) ?
			(void*)((unsigned int)buffer_info->pCb +
					fifo1_num_descriptors * SINGLE_DESCRIPTOR_TRANSFER_MAX) : 0;
	}

	/* return results */
	*buffer_id  = buffer_info->frame_id;
	*pY         = buffer_info->pY;
	*pCb        = buffer_info->pCb;
	*pCr        = buffer_info->pCr;

	return 0;
}

/* branch to queue head */
static int cam_branch_to_head(p_camera_context_t camera_context)
{
       camera_frame_buffer_queue_t *queue;
       unsigned int fifo0_transfer_size;
       unsigned int fifo1_transfer_size;
       unsigned int fifo2_transfer_size;
       volatile unsigned int cidadr0 = CIDADR0;

       /* get the specified capture queue and other info */
       queue = (camera_context->capture_mode == CAMERA_MODE_VIDEO)?
               &(camera_context->video_capture_buffer_queue):
                       &(camera_context->still_capture_buffer_queue);

#ifdef CI_DMAC_BRANCH_BUG
       if (DESCR(queue->tail, 0, phy, FIRST, PHANTOM) != cidadr0)
               return -1;
#endif

       fifo0_transfer_size =  camera_context->fifo0_transfer_size;
       fifo1_transfer_size =  camera_context->fifo1_transfer_size;
       fifo2_transfer_size =  camera_context->fifo2_transfer_size;

       if (fifo0_transfer_size)
               ci_dma_set_branch(DESCR(queue->head, 0, phy, FIRST, REAL), 1, 1, CI_DMA_CHANNEL_0);

       if (fifo1_transfer_size)
               ci_dma_set_branch(DESCR(queue->head, 1, phy, FIRST, REAL), 1, 1, CI_DMA_CHANNEL_1);

       if (fifo2_transfer_size)
               ci_dma_set_branch(DESCR(queue->head, 2, phy, FIRST, REAL), 1, 1, CI_DMA_CHANNEL_2);

       return 0;
}



/* Submit a buffer into the capture queue */
int mcam_submit_buffer(p_camera_context_t  camera_context,
		int  buffer_id, int buffer_type)
{
	camera_frame_buffer_info_t  *buffer_info;
	camera_frame_buffer_queue_t *queue;
	unsigned int fifo0_transfer_size;
	unsigned int fifo1_transfer_size;
	unsigned int fifo2_transfer_size;
	unsigned int need_branch = 0;

	/* get the specified capture queue and buffer and other info */
	queue = (buffer_type == VIDEO_CAPTURE_BUFFER)?
		&(camera_context->video_capture_buffer_queue):
			&(camera_context->still_capture_buffer_queue);

	buffer_info = &(camera_context->master_frame_buffer_list[buffer_id]);

	fifo0_transfer_size = (buffer_type == VIDEO_CAPTURE_BUFFER)?
		camera_context->video_fifo0_transfer_size:
		camera_context->still_fifo0_transfer_size;

	fifo1_transfer_size = (buffer_type == VIDEO_CAPTURE_BUFFER)?
		camera_context->video_fifo1_transfer_size:
		camera_context->still_fifo1_transfer_size;

	fifo2_transfer_size = (buffer_type == VIDEO_CAPTURE_BUFFER)?
		camera_context->video_fifo2_transfer_size:
		camera_context->still_fifo2_transfer_size;


	/* attach the submitted buffer's DMA chain to the queue */
	if ((queue->tail != NULL) && (queue->tail != buffer_info)) {
		if (fifo0_transfer_size) {
			DESCR(queue->tail, 0, vir, LAST, REAL)->ddadr =
				DESCR(buffer_info, 0, phy, FIRST, REAL);
			DESCR(queue->tail, 0, vir, LAST, PHANTOM)->ddadr =
				DESCR(buffer_info, 0, phy, FIRST, REAL);
		}

		if (fifo1_transfer_size) {
			DESCR(queue->tail, 1, vir, LAST, REAL)->ddadr =
				DESCR(buffer_info, 1, phy, FIRST, REAL);
			DESCR(queue->tail, 1, vir, LAST, PHANTOM)->ddadr =
				DESCR(buffer_info, 1, phy, FIRST, REAL);
		}

		if (fifo2_transfer_size) {
			DESCR(queue->tail, 2, vir, LAST, REAL)->ddadr =
				DESCR(buffer_info, 2, phy, FIRST, REAL);
			DESCR(queue->tail, 2, vir, LAST, PHANTOM)->ddadr =
				DESCR(buffer_info, 2, phy, FIRST, REAL);
		}

	} else if ((camera_context->dma_running && queue->tail == buffer_info) &&
			((buffer_type == VIDEO_CAPTURE_BUFFER &&
			camera_context->capture_mode == CAMERA_MODE_VIDEO) ||
			(buffer_type == STILL_CAPTURE_BUFFER &&
			camera_context->capture_mode == CAMERA_MODE_STILL))) {
		need_branch = 1;
	}

	/* update the head of the queue, if necessary */
	if (queue->head == NULL) {
		queue->head = buffer_info;
	}

	/* update the tail of the queue */
	if (queue->tail == NULL) {
		queue->tail = buffer_info;
	} else {
		queue->tail->next_buffer = buffer_info;
		queue->tail = buffer_info;
	}
	queue->tail->next_buffer = NULL;

	/* ensure the tail's phantom buffer loop to itself */

	if (fifo0_transfer_size) {
		DESCR(queue->tail, 0, vir, LAST, REAL)->ddadr =
			DESCR(buffer_info, 0, phy, FIRST, PHANTOM);
		DESCR(queue->tail, 0, vir, LAST, PHANTOM)->ddadr =
			DESCR(buffer_info, 0, phy, FIRST, PHANTOM);
	}

	if (fifo1_transfer_size) {
		DESCR(queue->tail, 1, vir, LAST, REAL)->ddadr =
			DESCR(buffer_info, 1, phy, FIRST, PHANTOM);
		DESCR(queue->tail, 1, vir, LAST, PHANTOM)->ddadr =
			DESCR(buffer_info, 1, phy, FIRST, PHANTOM);
	}

	if (fifo2_transfer_size) {
		DESCR(queue->tail, 2, vir, LAST, REAL)->ddadr =
			DESCR(buffer_info, 2, phy, FIRST, PHANTOM);
		DESCR(queue->tail, 2, vir, LAST, PHANTOM)->ddadr =
			DESCR(buffer_info, 2, phy, FIRST, PHANTOM);
	}


	/* tag the buffer as clean */
	*((unsigned int*)buffer_info->pY + fifo0_transfer_size/4 - 2) =
		CAMERA_CLEAN_BUFFER_IDENTIFIER;
	*((unsigned int*)buffer_info->pY + fifo0_transfer_size/4 - 1) =
		CAMERA_CLEAN_BUFFER_IDENTIFIER;

	/* reload the dma descriptor if needed */
	if (need_branch)
		cam_branch_to_head(camera_context);

	return 0;
}

#define DESCR(p, chan, vir_or_phy, first_or_last, real_or_phantom) \
       ((p)->ch ## chan ## _dma_desc_ ## vir_or_phy ## _addr \
       [first_or_last ## _DMA_DESC_FOR_ ## real_or_phantom ## _BUFFER_INX])

/* detect if the queue is in dead lock */
int queue_dead_locked(p_camera_context_t camera_context)
{
       camera_frame_buffer_queue_t *queue;
       unsigned int cidadr0 = CIDADR0;
       unsigned int fifo0_transfer_size;

       /* get the specified capture queue and other info */
       queue = (camera_context->capture_mode == CAMERA_MODE_VIDEO)?
               &(camera_context->video_capture_buffer_queue):
                       &(camera_context->still_capture_buffer_queue);

       fifo0_transfer_size =  camera_context->fifo0_transfer_size;

       if (queue->head == NULL)
               return 0;

       {
	       unsigned int * a = (unsigned int*)queue->head->pY + fifo0_transfer_size/4;
	       if ( (*(a - 2) != CAMERA_CLEAN_BUFFER_IDENTIFIER) && (*(a - 1) != CAMERA_CLEAN_BUFFER_IDENTIFIER))
		       return 0;
	       else
		       return ((CITADR0 == camera_context->phantom_buffer_physical) &&
			        DESCR(queue->tail, 0, phy, FIRST, PHANTOM) <= cidadr0) &&
			        (cidadr0 <= DESCR(queue->tail, 0, phy, LAST, PHANTOM));
       }
}


/* Get the buffer filled with valid frame data */
int mcam_get_filled_buffer(p_camera_context_t  camera_context,
		int *buffer_id)
{
	camera_frame_buffer_queue_t *queue;
	unsigned int fifo0_transfer_size;

	/* get the specified capture queue and other info */
	queue = (camera_context->capture_mode == CAMERA_MODE_VIDEO)?
		&(camera_context->video_capture_buffer_queue):
		&(camera_context->still_capture_buffer_queue);

	fifo0_transfer_size =  camera_context->fifo0_transfer_size;

	/* check if there are any buffers filled with valid image data */
	if (queue->head != NULL) {

		unsigned int * a = (unsigned int*)queue->head->pY + fifo0_transfer_size/4;

		if ((*(a - 2) != CAMERA_CLEAN_BUFFER_IDENTIFIER) &&
				(*(a - 1) != CAMERA_CLEAN_BUFFER_IDENTIFIER)) {

			*buffer_id  = queue->head->frame_id;
			queue->head = queue->head->next_buffer;

			return 0;
		}


		if (queue_dead_locked(camera_context))
			cam_branch_to_head(camera_context);

		return -EIO;
	}

	return -EIO;
}

/*
 * Frame rate APIs
 */
void mcam_set_capture_frame_rate(p_camera_context_t camera_context)
{
	ci_set_frame_rate((CI_FRAME_CAPTURE_RATE)camera_context->frame_rate);
}

/* return current setting */
void mcam_get_capture_frame_rate(p_camera_context_t camera_context)
{
	camera_context->frame_rate = ci_get_frame_rate();
}

/*
 * Interrupt APIs
 */
/* set interrupt mask */
void mcam_set_interrupt_mask(p_camera_context_t camera_context,
		unsigned int mask)
{
	/* set CI interrupt */
	ci_set_interrupt_mask(mask);
}

/* get interrupt mask */
unsigned int mcam_get_interrupt_mask(p_camera_context_t camera_context)
{
	/* get CI mask */
	return ci_get_interrupt_mask();
}

/* get interrupt status */
unsigned int mcam_get_interrupt_status(p_camera_context_t camera_context)
{
	return ci_get_interrupt_status();
}

/* clear interrupt status */
void mcam_clear_interrupt_status(p_camera_context_t camera_context,
		unsigned int status)
{
	ci_clear_interrupt_status(status);
}

/*
 * Sensor Control APIs
 */
int mcam_read_8bit(p_camera_context_t camera_context, u8 reg_addr, u8 *reg_val)
{
	int status;

	if (camera_context->camera_functions->read_8bit) {
		status = camera_context->camera_functions->
				read_8bit(camera_context, reg_addr, reg_val);
		return status;
	}
	return -EIO;
}

/* CMOS sensor 8 bit register write */
int mcam_write_8bit(p_camera_context_t camera_context, u8 reg_addr, u8 reg_val)
{
	int status;

	if (camera_context->camera_functions->write_8bit) {
		status = camera_context->camera_functions->
				write_8bit(camera_context, reg_addr, reg_val);
		return status;
	}
	return -EIO;
}

/* CMOS sensor 16-bit register read */
int mcam_read_16bit(p_camera_context_t camera_context, u16 reg_addr, u16 *reg_val)
{
	int status;

	if (camera_context->camera_functions->read_16bit) {
		status = camera_context->camera_functions->
				read_16bit(camera_context, reg_addr, reg_val);
		return status;
	}
	return -EIO;
}

/* CMOS sensor 16 bit register write */
int mcam_write_16bit(p_camera_context_t camera_context, u16 reg_addr,
		u16 reg_val)
{
	int status;

	if (camera_context->camera_functions->write_16bit) {
		status = camera_context->camera_functions->
				write_16bit(camera_context, reg_addr, reg_val);
		return status;
	}
	return -EIO;
}

/* CMOS sensor 32 bit register read */
int mcam_read_32bit(p_camera_context_t camera_context, u32 reg_addr,
		u32 *reg_val)
{
	int status;

	if (camera_context->camera_functions->read_32bit) {
		status = camera_context->camera_functions->
				read_32bit(camera_context, reg_addr, reg_val);
		return status;
	}
	return -EIO;
}

/* CMOS sensor 32 bit register write */
int mcam_write_32bit(p_camera_context_t camera_context, u32 reg_addr,
		u32 reg_val)
{
	int status;

	if (camera_context->camera_functions->write_32bit) {
		status = camera_context->camera_functions->
				write_32bit(camera_context, reg_addr, reg_val);
		return status;
	}
	return -EIO;
}

/* CMOS sensor Power Mode read */
void mcam_get_power_mode(p_camera_context_t camera_context, u8 *power_mode)
{
	*power_mode = camera_context->sensor_status.power_mode;
}

/* CMOS sensor Power Mode write */
int mcam_set_power_mode (p_camera_context_t camera_context, u8 power_mode)
{
	int status;

	status = 0;

	if (camera_context->sensor_status.power_mode != power_mode)
		status = camera_context->camera_functions->
				set_power_mode(camera_context, power_mode);

	return status;
}

/* CMOS sensor Capability read */
unsigned int mcam_get_caps (p_camera_context_t camera_context)
{
	return camera_context->sensor_status.caps;
}

/* CMOS sensor contrast value read */
void mcam_get_contrast_value (p_camera_context_t camera_context,
		unsigned char *mode, unsigned int *value)
{
	*mode = camera_context->sensor_status.contrast_mode;
	if (*mode == SENSOR_MANUAL_CONTRAST)
		*value = camera_context->sensor_status.contrast_value;
	else
		*value = 0;
}

/* CMOS sensor contrast value write */
int mcam_set_contrast_value (p_camera_context_t camera_context,
		unsigned char mode, unsigned int value)
{
	camera_context->sensor_status.contrast_mode = mode;
	camera_context->sensor_status.contrast_value = value;
	return camera_context->camera_functions->
		set_contrast(camera_context, mode, value);
}

/* CMOS sensor white balance value read */
void mcam_get_white_balance_value (p_camera_context_t camera_context,
		unsigned char *mode, unsigned int *value)
{
	*mode = camera_context->sensor_status.whitebalance_mode;
	if (*mode == SENSOR_MANUAL_WHITEBALANCE)
		*value = camera_context->sensor_status.whitebalance_value;
	else
		*value = 0;
}

/* CMOS sensor white balance value write */
int mcam_set_white_balance_value (p_camera_context_t camera_context,
		unsigned char mode, unsigned int value)
{
	camera_context->sensor_status.whitebalance_mode = mode;
	camera_context->sensor_status.whitebalance_value = value;
	return camera_context->camera_functions->
		set_whitebalance(camera_context, mode, value);
}

/* CMOS sensor exposure value read */
void mcam_get_exposure_value (p_camera_context_t camera_context,
		unsigned char *mode, unsigned int *value)
{
	*mode = camera_context->sensor_status.exposure_mode;
	if (*mode == SENSOR_MANUAL_EXPOSURE)
		*value = camera_context->sensor_status.exposure_value;
	else
		*value = 0;
}

/* CMOS sensor exposure value write */
int mcam_set_exposure_value (p_camera_context_t camera_context,
		unsigned char mode, unsigned int value)
{
	camera_context->sensor_status.exposure_mode = mode;
	camera_context->sensor_status.exposure_value = value;
	return camera_context->camera_functions->
		set_exposure(camera_context, mode, value);
}

/* CMOS sensor zoom value read */
void mcam_get_zoom_value (p_camera_context_t camera_context, unsigned int *value)
{
	*value = camera_context->sensor_status.zoom_value;
}

/* CMOS sensor zoom value write */
int mcam_set_zoom_value (p_camera_context_t camera_context, unsigned int value)
{
	camera_context->sensor_status.zoom_value = value;
	return camera_context->camera_functions->set_zoom(camera_context, value);
}

/* get Histogram Info */
int mcam_get_histogram_info(
		p_camera_context_t     camera_context,
		unsigned int                color_type,
		unsigned int               *histogram_size,
		unsigned int               *histogram_sum)
{
	unsigned int mux_select;

	mux_select = (camera_context->capture_input_format ==
		CAMERA_IMAGE_FORMAT_RAW10)?CI_HSU_MUX_1_TO_9:
		(camera_context->capture_input_format ==
		CAMERA_IMAGE_FORMAT_RAW9) ?CI_HSU_MUX_0_TO_8:
		CI_HSU_MUX_0_TO_7;
	camera_context->fifo3_transfer_size =
		(mux_select == CI_HSU_MUX_0_TO_7)?512:1024;

	*histogram_size = camera_context->fifo3_transfer_size;
	return ci_hsu_get_histgram(
			(CI_HSU_COLOR_TYPE) color_type,
			(CI_HSU_MUX_SEL_TYPE) mux_select,
			camera_context->histogram_lut_buffer_virtual,
			camera_context->histogram_lut_buffer_physical,
			(unsigned int*) camera_context->
				histogram_lut_dma_descriptors_virtual,
			camera_context->histogram_lut_dma_descriptors_physical,
			camera_context->fifo3_transfer_size,
			histogram_sum);
}

/*
 * sleep/wakeup APIs
 */
int mcam_suspend(p_camera_context_t    camera_context)
{
	/* stop capture at first, if necessary */
	if (camera_context->dma_running) {
		if (camera_context->capture_mode == CAMERA_MODE_VIDEO) {
			mcam_stop_video_capture(camera_context);
		} else if (camera_context->capture_mode == CAMERA_MODE_STILL) {
			mcam_stop_video_capture(camera_context);;
		} else {
			return 0;
		}
		camera_context->dma_running = 1;
	}

	/* sleep sensor */
	camera_context->camera_functions->sleep(camera_context);

	ci_deinit();

	/* set two gpio pin output as HI to power off two sensors */
	mhn_gpio_set_level(MFP_CIF_HI_PWDN_GPI0, GPIO_LEVEL_HIGH);
	mhn_gpio_set_level(MFP_CIF_LO_PWDN_GPI0, GPIO_LEVEL_HIGH);

	return 0;
}

int mcam_resume(p_camera_context_t    camera_context)
{
	int status;

	/* set two gpio pin direction as output to
	 * control the power of two sensors.
	 */
	mhn_gpio_set_direction(MFP_CIF_HI_PWDN_GPI0, GPIO_DIR_OUT);
	mhn_gpio_set_direction(MFP_CIF_LO_PWDN_GPI0, GPIO_DIR_OUT);
	/* set two gpio pin output as HI to power off two sensors */
	mhn_gpio_set_level(MFP_CIF_HI_PWDN_GPI0, GPIO_LEVEL_HIGH);
	mhn_gpio_set_level(MFP_CIF_LO_PWDN_GPI0, GPIO_LEVEL_HIGH);

	ci_init();

	mcam_set_capture_frame_rate(camera_context);

	/* wake up sensor */
	status = camera_context->camera_functions->wakeup(camera_context);
	if (status != 0) {
		goto camera_init_err;
	}

	/* recovered to the pre-sleep status */
	mcam_set_capture_format(camera_context);
	if (camera_context->dma_running) {
		if (camera_context->capture_mode == CAMERA_MODE_VIDEO) {
			mcam_start_video_capture(camera_context);
		} else if (camera_context->capture_mode == CAMERA_MODE_STILL) {
			mcam_capture_still_image(camera_context);
		}
	}

	return 0;
camera_init_err:
	mcam_deinit(camera_context);

	return -EIO;
}

