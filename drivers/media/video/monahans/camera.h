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
 *   contains all camera specific macros, typedefs, and prototypes.
 *   Declares no storage.
 *
 * Notes:
 *   Only valid for processor code named Monahans.
 */

#ifndef __MONAHANS_CAMERA_HEADER__
#define __MONAHANS_CAMERA_HEADER__


#include "ci.h"

/*
 * Macros
 */
/* Sensor type */
#define CAMERA_TYPE_ADCM_2650		1
#define CAMERA_TYPE_ADCM_2670		2
#define CAMERA_TYPE_ADCM_2700		3
#define CAMERA_TYPE_OMNIVISION_9640	4
#define CAMERA_TYPE_OMNIVISION_2620	5
#define CAMERA_TYPE_OMNIVISION_7660	6
#define CAMERA_TYPE_OMNIVISION_2630	7
#define CAMERA_TYPE_MAX			CAMERA_TYPE_OMNIVISION_2630

/* Image format definition */
#define CAMERA_IMAGE_FORMAT_RAW8		0
#define CAMERA_IMAGE_FORMAT_RAW9		1
#define CAMERA_IMAGE_FORMAT_RAW10		2
#define CAMERA_IMAGE_FORMAT_RGB444		3
#define CAMERA_IMAGE_FORMAT_RGB555		4
#define CAMERA_IMAGE_FORMAT_RGB565		5
#define CAMERA_IMAGE_FORMAT_RGB666_PACKED	6
#define CAMERA_IMAGE_FORMAT_RGB666_PLANAR	7
#define CAMERA_IMAGE_FORMAT_RGB888_PACKED	8
#define CAMERA_IMAGE_FORMAT_RGB888_PLANAR	9
#define CAMERA_IMAGE_FORMAT_RGBT555_0		10  /* RGB+Transparent bit 0 */
#define CAMERA_IMAGE_FORMAT_RGBT888_0		11
#define CAMERA_IMAGE_FORMAT_RGBT555_1		12  /* RGB+Transparent bit 1 */
#define CAMERA_IMAGE_FORMAT_RGBT888_1		13

#define CAMERA_IMAGE_FORMAT_YCBCR400		14
#define CAMERA_IMAGE_FORMAT_YCBCR422_PACKED	15
#define CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR	16
#define CAMERA_IMAGE_FORMAT_YCBCR444_PACKED	17
#define CAMERA_IMAGE_FORMAT_YCBCR444_PLANAR	18
#if defined(CONFIG_PXA310)
#define CAMERA_IMAGE_FORMAT_YCBCR420_PACKED	19
#define CAMERA_IMAGE_FORMAT_YCBCR420_PLANAR	20
#define CAMERA_IMAGE_FORMAT_MAX		\
		CAMERA_IMAGE_FORMAT_YCBCR420_PLANAR
#else
#define CAMERA_IMAGE_FORMAT_MAX		\
		CAMERA_IMAGE_FORMAT_YCBCR444_PLANAR
#endif

/* Interrupt mask */
#define CAMERA_INTMASK_FIFO_OVERRUN	CI_INT_IFO
#define CAMERA_INTMASK_END_OF_FRAME	CI_INT_EOF
#define CAMERA_INTMASK_START_OF_FRAME	CI_INT_SOF
#define CAMERA_INTMASK_CI_DISABLE_DONE	CI_INT_CDD
#define CAMERA_INTMASK_CI_QUICK_DISABLE	CI_INT_CQD
#define CAMERA_INTMASK_PARITY_ERROR	CI_INT_PAR_ERR
#define CAMERA_INTMASK_END_OF_LINE	CI_INT_EOL
#define CAMERA_INTMASK_FIFO_EMPTY	CI_INT_FEMPTY
#define CAMERA_INTMASK_TIME_OUT		CI_INT_FTO
#define CAMERA_INTMASK_FIFO3_UNDERRUN	CI_INT_FU
#define CAMERA_INTMASK_BRANCH_STATUS	CI_INT_BS
#define CAMERA_INTMASK_ENF_OF_FRAME_TRANSFER	CI_INT_EOFX
#define CAMERA_INTMASK_DMA_CHANNEL0_STOP	CI_INT_SC0
#define CAMERA_INTMASK_DMA_CHANNEL1_STOP	CI_INT_SC1
#define CAMERA_INTMASK_DMA_CHANNEL2_STOP	CI_INT_SC2
#define CAMERA_INTMASK_DMA_CHANNEL3_STOP	CI_INT_SC3


/* Capture mode */
#define CAMERA_MODE_VIDEO		0x0001
#define CAMERA_MODE_STILL		0x0002

/* buffer type */
#define VIDEO_CAPTURE_BUFFER		0
#define STILL_CAPTURE_BUFFER		1

/* clean buffer identifier */
#define CAMERA_CLEAN_BUFFER_IDENTIFIER	0x12345678

#define CAMERA_CAPTURE_SCALE_DISABLE     CI_SSU_SCALE_DISABLE
#define CAMERA_CAPTURE_SCALE_HALF        CI_SSU_SCALE_HALF
#define CAMERA_CAPTURE_SCALE_QUATER      CI_SSU_SCALE_QUARTER

/* sensor capabilities */
#define SENSOR_CAP_MANUAL_CONTRAST       0x0001
#define SENSOR_CAP_MANUAL_WHITEBALANCE   0x0002
#define SENSOR_CAP_MANUAL_EXPOSURE       0x0004
#define SENSOR_CAP_MANUAL_ZOOM           0x0008

/* sensor power mode */
#define CAMERA_POWER_OFF                 0
#define CAMERA_POWER_LOW                 1
#define CAMERA_POWER_MID                 2
#define CAMERA_POWER_FULL                3

/* sensor contrast mode and value */
#define SENSOR_MANUAL_CONTRAST           0
#define SENSOR_AUTO_CONTRAST             1

#define SENSOR_CONTRAST_LOWEST           0
#define SENSOR_CONTRAST_LOW              1
#define SENSOR_CONTRAST_MIDDLE           2
#define SENSOR_CONTRAST_HIGH             3
#define SENSOR_CONTRAST_HIGHEST          4

/* sensor white balance mode and value */
#define SENSOR_MANUAL_WHITEBALANCE       0
#define SENSOR_AUTO_WHITEBALANCE         1

#define SENSOR_WHITEBALANCE_AUTO         0
#define SENSOR_WHITEBALANCE_INCANDESCENT 1
#define SENSOR_WHITEBALANCE_SUNNY        2
#define SENSOR_WHITEBALANCE_FLUORESCENT  3

/* sensor exposure mode and value */
#define SENSOR_MANUAL_EXPOSURE           0
#define SENSOR_AUTO_EXPOSURE             1

#define SENSOR_EXPOSURE_LOWEST           0
#define SENSOR_EXPOSURE_LOW              1
#define SENSOR_EXPOSURE_MIDDLE           2
#define SENSOR_EXPOSURE_HIGH             3
#define SENSOR_EXPOSURE_HIGHEST          4

/* Buffer Pool Information */
/* Max Buffer number of Camera Buffer Pool has 64
 * First 32bytes: phantom buffer
 * Last 16 bytes: init dma descriptor for all channels
 * 32 bytes are enough: clear DCMD[IncTrgAddr] of
 * phantom buffer's DMA descriptor so as to save
 * memory, since phantom buffer is useless.
 */
#define MAX_CAMERA_FRAME_BUFFERS	32

#define PHANTOM_BUFFER_SIZE		112

/* Frame Buffer Information */
/* each channel of frame buffer has 3
 * special DMA descriptors, two for real
 * buffer, another for phantom buffer.
 */
#define SPECIAL_DMA_DESC_NUM_PER_CHANNEL_PER_FRAME      4

/* the first DMA descriptor of real buffer */
#define FIRST_DMA_DESC_FOR_REAL_BUFFER_INX              0

/* the last DMA descriptor of real buffer */
#define LAST_DMA_DESC_FOR_REAL_BUFFER_INX               1

/* the first DMA descriptor of phantom buffer */
#define FIRST_DMA_DESC_FOR_PHANTOM_BUFFER_INX           2

/* the last DMA descriptor of phantom buffer */
#define LAST_DMA_DESC_FOR_PHANTOM_BUFFER_INX            3

/*
 * Structures
 */
typedef struct camera_context_s camera_context_t, *p_camera_context_t;

typedef enum {
       YUV_HAVE_PADDING   = 0,
       YUV_NO_PADDING     = 1
} PADDING_TYPE;

typedef struct {
	int (*init)(p_camera_context_t camera_context);
	int (*deinit)(p_camera_context_t camera_context);
	int (*set_capture_format)(p_camera_context_t camera_context);
	int (*start_capture)(p_camera_context_t camera_context ,
			unsigned int frames);
	int (*stop_capture)(p_camera_context_t camera_context);
	int (*sleep)(p_camera_context_t camera_context);
	int (*wakeup)(p_camera_context_t camera_context);

	/*  functions for sensor control */
	int (*read_8bit)(p_camera_context_t camera_context,
			u8 reg_addr, u8 *reg_val);
	int (*write_8bit)(p_camera_context_t camera_context,
			u8 reg_addr, u8 reg_val);
	int (*read_16bit)(p_camera_context_t camera_context,
			u16 reg_addr, u16 *reg_val);
	int (*write_16bit)(p_camera_context_t camera_context,
			u16 reg_addr, u16 reg_val);
	int (*read_32bit)(p_camera_context_t camera_context,
			u32 reg_addr, u32 *reg_val);
	int (*write_32bit)(p_camera_context_t camera_context,
			u32 reg_addr, u32 reg_val);
	int (*set_power_mode)(p_camera_context_t camera_context, u8 power_mode);
	int (*set_contrast) (p_camera_context_t camera_context,
			u8 mode, u32 value);
	int (*set_whitebalance) (p_camera_context_t camera_context,
			u8 mode, u32 value);
	int (*set_exposure) (p_camera_context_t camera_context,
			u8 mode, u32 value);
	int (*set_zoom) (p_camera_context_t camera_context, u32 value);
} camera_function_t, *p_camera_function_t;


typedef struct {
	unsigned char power_mode;

	unsigned int caps;
	/*  1 << 0:    Manual ContrastMode supported by sensor;
	 *  1 << 1:    Manual whitebalance supported by sensor;
	 *  1 << 2:    Manual exposure supported by sensor;
	 *  1 << 3:    Manual Zoom supported by sensor;
	 */

	unsigned char contrast_mode;
	unsigned char whitebalance_mode;
	unsigned char exposure_mode;

	unsigned int contrast_value;
	unsigned int whitebalance_value;
	unsigned int exposure_value;
	unsigned int zoom_value;
} sensor_status_t, *p_sensor_status_t;

typedef struct camera_frame_buffer_info_s camera_frame_buffer_info_t ,\
		*p_camera_frame_buffer_info_t;
typedef struct camera_frame_buffer_queue_s camera_frame_buffer_queue_t, \
		*p_camera_frame_buffer_queue_t;

struct camera_frame_buffer_info_s {
	/* Information for frame buffer itself */

	/* Frame ID, indicates this buffer's
	 * position in the buffer pool.
	 */
	int			frame_id;

	/* the buffer size, can be obtained via
	 * mcam_get_buffer_size() before
	 * allocating the memory for the buffer.
	 */
	int			buffer_size;

	/* the virtual address of the memory for DMA descriptors
	 * for this buffer, should be 16-bytes aligned.
	 */
	volatile struct ci_dma_descriptor *dma_descriptors_virtual;

	/* the virtual address of the Y component */
	void			*pY;

	/* the virtual address of the Cb component */
	void			*pCb;

	/* the virtual address of the Cr component */
	void			*pCr;

	/* Information of the DMA descriptor Chains for this frame buffer */
	/* pointer to an array of physical address of
	 * channel 0's DMA chain for this buffer;
	 */
	u32		ch0_dma_desc_phy_addr	\
			[SPECIAL_DMA_DESC_NUM_PER_CHANNEL_PER_FRAME];

	/* pointer to an array of virtual address of
	 * channel 0's DMA chain for this buffer;
	 */
	volatile struct ci_dma_descriptor *ch0_dma_desc_vir_addr	\
			[SPECIAL_DMA_DESC_NUM_PER_CHANNEL_PER_FRAME];

	/* DMA descriptors number for channel 0 */
	int			ch0_dma_desc_num;

	/* DMA descriptors number of phantom buffer for channel 0 */
	int			ch0_phantom_dma_desc_num;

	/* pointer to an array of physical address of
	 * channel 1's DMA chain for this buffer;
	 */
	u32		ch1_dma_desc_phy_addr	\
			[SPECIAL_DMA_DESC_NUM_PER_CHANNEL_PER_FRAME];

	/* pointer to an array of virtual address of
	 * channel 1's DMA chain for this buffer;
	 */
	volatile struct ci_dma_descriptor *ch1_dma_desc_vir_addr	\
			[SPECIAL_DMA_DESC_NUM_PER_CHANNEL_PER_FRAME];

	/* DMA descriptors number for channel 1 */
	int			ch1_dma_desc_num ;

	/* DMA descriptors number of phantom buffer for channel 1 */
	int			ch1_phantom_dma_desc_num;

	/* pointer to an array of physical address of
	 * channel 2's DMA chain for this buffer;
	 */
	u32		ch2_dma_desc_phy_addr	\
			[SPECIAL_DMA_DESC_NUM_PER_CHANNEL_PER_FRAME];

	/* pointer to an array of virtual address of
	 * channel 2's DMA chain for this buffer;
	 */
	volatile struct ci_dma_descriptor *ch2_dma_desc_vir_addr	\
			[SPECIAL_DMA_DESC_NUM_PER_CHANNEL_PER_FRAME];

	/* DMA descriptors number for channel 2 */
	int			ch2_dma_desc_num;

	/* DMA descriptors number of phantom buffer for channel 2 */
	int			ch2_phantom_dma_desc_num;

	/* next buffer in the queue, only used in
	 * video capture buffer queue or still
	 * capture buffer queue.
	 */
	camera_frame_buffer_info_t	*next_buffer;
};

struct camera_frame_buffer_queue_s {
	/* first frame buffer owned by the queue. When
	 * this buffer will be reported the
	 * application when it is filled with frame
	 * data, then this pointer will point to the
	 * next buffer in the queue. If the queue runs
	 * out of its buffers, this pointer will be
	 * NULL until one new free buffer added into
	 * the queue when this pointer will point to
	 * this new buffer.
	 */
	camera_frame_buffer_info_t   *head;

	/* last frame buffer in the queue, always
	 * followed by the phantom buffer. When there
	 * is no frame available in the queue, the DMA
	 * controller will transfer the frame data to
	 * its phantom buffer until the Camera driver
	 * submits new frames into the queue. The
	 * frame complete interrupt for phantom buffer
	 * should be ignored.
         */
	camera_frame_buffer_info_t   *tail;
};

/* context */
struct camera_context_s {
	/* DRIVER FILLED PARAMTER */

	/* sensor information */
	/* defines the imagine sensor type connected to the camera
	 * interface. For example,
	 * TYPE_OMNIVISION_2620 selected the Agilent
	 * 2620 sensor available on the Zylonite platform
	 */
	unsigned int                       sensor_type;

	/* capture image information */
	/* video capture or still capture */
	unsigned int                       capture_mode;

	/* video_capture_width and video_capture_height define the
	 * desired width and height of the motion video capture
	 * stream.  Knowledge of the particular sensor is required
	 * in order to specify a valid width and height.  Some
	 * sensors support arbitrary width and height up to the
	 * maximum sensor dimensions.  Other sensors support only
	 * subset of possible sizes.
	 */
	unsigned int                       video_capture_width;
	unsigned int                       video_capture_height;

	/* defines the scaling rate of sensor output vs QCI output.
	 * possible choices: 1:1(default) or 2:1 or 4:1.However, if there
	 * is need to convert RAW data to YUV/RGB, then this field must be
	 * set as 2:1 or 4:1
	 */
	unsigned int                       video_capture_scale;

	/* video_capture_input_format describes the data format for
	 * motion video that the sensor provides to the camera
	 * interface.  For example,
	 * CAMERA_IMAGE_FORMAT_RAW10 is
	 * used to describe a 10-bits RAW data format, such as that
	 * used by the OV2620.
	 */
	unsigned int                       video_capture_input_format;

	/* video_capture_output_format describes the data format
	 * for motion video that the camera interface will use to
	 * source data to the DMA controller.  The output format
	 * need not be the same as the input format.  The Monahans
	 * camera interface provides some limited conversion
	 * mechanisms.  For example, input data of
	 * CAMERA_IMAGE_FORMAT_RAW10 may be converted to
	 * CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR by specifying
	 * that planar format.
	 */
	unsigned int                       video_capture_output_format;

	/* still_capture_width and still_capture_height define the
	 * desired width and height of the still image capture.
	 * Knowledge of the particular sensor is required
	 * in order to specify a valid width and height.  Some
	 * sensors support arbitrary width and height up to the
	 * maximum sensor dimensions.  Other sensors support only
	 * subset of possible sizes.
         */
	unsigned int                       still_capture_width;
	unsigned int                       still_capture_height;

	/* defines the scaling rate of sensor output vs QCI output.
	 * possible choices: 1:1(default) or 2:1 or 4:1.However, if there
	 * is need to convert RAW data to YUV/RGB, then this field must be
	 * set as 2:1 or 4:1
	 */
	unsigned int                       still_capture_scale;

	/* still_capture_input_format describes the data format for
	 * still image that the sensor provides to the camera
	 * interface.  For example,
	 * CAMERA_IMAGE_FORMAT_RAW10 is
	 * used to describe a 10-bits RAW data format, such as that
	 * used by the OV2620.
	 */
	unsigned int                       still_capture_input_format;

	/* still_capture_output_format describes the data format
	 * for still image that the camera interface will use to
	 * source data to the DMA controller.  The output format
	 * need not be the same as the input format.  The Monahans
	 * camera interface provides some limited conversion
	 * mechanisms.  For example, input data of
	 * CAMERA_IMAGE_FORMAT_RAW10 may be converted to
	 * CAMERA_IMAGE_FORMAT_YCBCR422_PLANAR by specifying
	 * that planar format.
	 */
	unsigned int                       still_capture_output_format;

	/* frame rate control */
	/* frame_rate specifies the desired frame rate control.
	 * Use a 0 to specify the maximum frame rate.  For n > 0,
	 * it specifies that the interface should capture 1 out of
	 * every n+1 frames generated.  A future implementation may
	 * change the interpretation of the frame rate in order to
	 * specify the frames per second.
	 */
	unsigned int                       frame_rate;

	/* memory for phantom buffer */
	/* buffer with PHANTOM_BUFFER_SIZE bytes,
	 * allocated by camera driver, used as phantom buffer by all
	 * buffers, should be 16-bytes aligned.
	 */
	u32                    *phantom_buffer_virtual;
	u32                     phantom_buffer_physical;

	/* memory for getting Histogram or loading CGU LUT */
	/* buffer with 1024 bytes, allocated by camera
	 * driver, used for getting Histogram(512 or 1024bytes) or
	 * loading CGU LUT(64*3bytes), should be 16-bytes aligned.
	 */
	u32                    *histogram_lut_buffer_virtual;
	u32                     histogram_lut_buffer_physical;

	/* memory for DMA descriptors to get Histogram or load CGU LUT */
	/* memory for DMA descriptors allocated by Camera driver,
	 * 16-bytes aligned, should be 16-byte sized. This
	 * descriptor is required to get histogram data or load
	 * Compander look-up-table (CGU LUT).
	 */
	struct ci_dma_descriptor *histogram_lut_dma_descriptors_virtual;
	u32                     histogram_lut_dma_descriptors_physical;

	/* os mapped register address */
	/* specifies the virtual address of the OST registers */
	/* unsigned int                       ost_reg_base; */

	/* specifies the virtual address of the GPIO registers */
	/* unsigned int                       gpio_reg_base; */

	/* specifies the virtual address of the CAMERA INTERFACE registers */
	unsigned int                       ci_reg_base;

	/* specifies the virtual address of the Clock registers registers */
	/* unsigned int                       clk_reg_base; */

	/* function dispatch table */
	/* specify pointers to the given camera sensor functions */
	p_camera_function_t           camera_functions;



	/* INTERNALLY USED: DON'T TOUCH! */

	/* Sensor status */
	sensor_status_t               sensor_status;

	/* sensor related parameter or status will be put here */

	/* capture parameters:
	 *    those parameters are being used by Camera currently
	 *    for video capture or still capture, generated from
	 *    driver-filled parameters according to the parameter
	 *    capture_mode.
	 */

	/* current image resolution of the sensor output or QCI
	 * input, may be the same as QCI output, may be not,
	 * depending on the capture scale.
	 */
	unsigned int                       capture_input_width;
	unsigned int                       capture_input_height;

	/* current image resolution of the QCI output */
	unsigned int                       capture_output_width;
	unsigned int                       capture_output_height;

	/* current image format of the sensor output or QCI input */
	unsigned int                       capture_input_format;

	/* current image format of the QCI output */
	unsigned int                       capture_output_format;

	/* this three parameters indicate the current transfer size
	 * of channel 0, channel 1 and channel 2, respectively.
	 * they would be calculated from the format and
	 * resolution of the current QCI output.
	 */
	unsigned int                       fifo0_transfer_size;
	unsigned int                       fifo1_transfer_size;
	unsigned int                       fifo2_transfer_size;

	/* transfer size of channel 3. This parameter is for
	 * reading Histogram only, may be 512 or 1024, depending on
	 * the CIHST.SCALE. For loading CGU LUT, the transfer size
	 * will be hard coded as 64*3 bytes, since the CGU LUT
	 *consists of three 64-element Look-Up-Tables.
	 */
	unsigned int                       fifo3_transfer_size;

	/* this three parameters indicate the transfer size
	 * of channel 0, channel 1 and channel 2 for video capture, respectively.
	 * they would be calculated from the format and
	 * resolution of the vide capture QCI output.
	 */
	unsigned int                       video_fifo0_transfer_size;
	unsigned int                       video_fifo1_transfer_size;
	unsigned int                       video_fifo2_transfer_size;

	/* this three parameters indicate the transfer size
	 * of channel 0, channel 1 and channel 2 for still capture, respectively.
	 * they would be calculated from the format and
	 * resolution of the still capture QCI output.
	 */
	unsigned int                       still_fifo0_transfer_size;
	unsigned int                       still_fifo1_transfer_size;
	unsigned int                       still_fifo2_transfer_size;

	/* frame buffer pool */
	/* frame buffer pool is maintained within the camera driver.
	 * When the application or camera driver is initialized, the
	 * frame buffer will be allocated, too. Those frame buffers
	 * will be added to this frame buffer pool. Afterward the
	 * application/camera driver will get buffers from this pool
	 * and submits them to the still capture buffer queue or the
	 * video capture buffer queue.
	 */
	camera_frame_buffer_info_t
		master_frame_buffer_list[MAX_CAMERA_FRAME_BUFFERS];

	/* indicates how many buffers the buffer pool currently has */
	int                                 frame_buffer_number;

	/* still capture buffer queue */
	/* frame buffer queue used by still capture, its frame
	 * buffers come from frame buffer pool. Generally, this
	 * queue only has one buffer.
	 */
	camera_frame_buffer_queue_t   still_capture_buffer_queue;

	/* video capture buffer queue */
	/* frame buffer queue used by video capture, its frame
	 * buffers come from frame buffer pool, too.
	 */
	camera_frame_buffer_queue_t   video_capture_buffer_queue;

	/* QCI status */
	/* indicates whether QCI disabling is completed */
	unsigned int                       ci_disable_complete;

	/* indicates whether PSU is enabled */
	int                        psu_enable;

	/* indicates whether the CGU is enabled */
	int                        cgu_enable;

	/* indicates the SSU scale ratio */
	CI_SSU_SCALE                  ssu_scale;

	/* indicates how to use CMU. This varible will tell the sensor
	 * that it should choose which CMU matrix to load
	 */
	CI_CMU_USAGE                  cmu_usage;

	int                        dma_running;

	/*
	 * align_tpye indicate whether YUV have padding
	 * 0: have padding
	 * 1: no padding;
         */
	int                       align_type;

#if defined(CONFIG_PXA310)
	CI_CICR4_YCBCR_DOWN_SAMPLE		ycbcr_ds;
#endif
};

/*
 * Prototypes
 */

/*
 * Init/Deinit APIs
 */

/* Setup the sensor type, configure image capture format (RGB, yuv 444,
 * yuv 422, yuv 420, packed | planar, MJPEG) regardless of current operating
 * mode (i.e. sets mode for both still capture and video capture)
 */
int mcam_init(p_camera_context_t camera_context);

/* Power off sensor */
int mcam_deinit(p_camera_context_t camera_context);

/*
 * Capture APIs
 */

/* Set the image format */
int mcam_set_capture_format(p_camera_context_t camera_context);

/* take a picture and copy it into the frame buffer */
int mcam_capture_still_image(p_camera_context_t camera_context);

/* capture motion video and copy it the frame buffer */
int mcam_start_video_capture(p_camera_context_t camera_context);

/* disable motion video image capture */
void mcam_stop_video_capture(p_camera_context_t camera_context);

/*
 * Flow Control APIs
 */
/* Get the buffer size and the DMA descriptors memory size for this buffer*/
int mcam_get_buffer_size(
		p_camera_context_t   camera_context,
		int   buffer_type,
		int  *buffer_size,
		int  *buffer_dma_desc_mem_size);

/* Add the buffer into the buffer pool */
int mcam_prepare_buffer(
		p_camera_context_t   camera_context,
		void   *buffer_virtual_address,
		int   *buffer_physical_address_array,
		int    buffer_physical_address_array_size,
		int    buffer_size,
		int    buffer_type,
		void   *dma_desc_mem_virtual_address,
		int   *dma_desc_mem_physical_address_array,
		int   *buffer_id,
		void   **pY,
		void   **pCb,
		void   **pCr);

/* Submit a buffer into the capture queue */
int mcam_submit_buffer(
		p_camera_context_t  camera_context,
		int  buffer_id,
		int  buffer_type);

/* Get the buffer filled with valid frame data */
int mcam_get_filled_buffer(
		p_camera_context_t  camera_context,
		int *buffer_id);

/*
 * Frame rate APIs
 */

/* Set desired frame rate */
void mcam_set_capture_frame_rate(p_camera_context_t camera_context);

/* return current setting */
void mcam_get_capture_frame_rate(p_camera_context_t camera_context);

/*
 * Interrupt APIs
 */

/* set interrupt mask */
void mcam_set_interrupt_mask(p_camera_context_t camera_context,
		unsigned int mask);

/* get interrupt mask */
unsigned int mcam_get_interrupt_mask(p_camera_context_t camera_context);

/* get interrupt status */
unsigned int mcam_get_interrupt_status(p_camera_context_t camera_context);

/* clear interrupt status */
void mcam_clear_interrupt_status(p_camera_context_t camera_context,
		unsigned int status);

/*
 * Sensor Control APIs
 */

/* CMOS sensor 8 bit register read */
int mcam_read_8bit(p_camera_context_t camera_context,
		u8 reg_addr, u8 *reg_val);

/* CMOS sensor 8 bit register write */
int mcam_write_8bit(p_camera_context_t camera_context,
		u8 reg_addr, u8 reg_val);

/* CMOS sensor 16-bit register read */
int mcam_read_16bit(p_camera_context_t camera_context,
		u16 reg_addr, u16 *reg_val);

/* CMOS sensor 16 bit register write */
int mcam_write_16bit(p_camera_context_t camera_context,
		u16 reg_addr, u16 reg_val);

/* CMOS sensor 32 bit register read */
int mcam_read_32bit(p_camera_context_t camera_context,
		u32 reg_addr, u32 *reg_val);

/* CMOS sensor 32 bit register write */
int mcam_write_32bit(p_camera_context_t camera_context,
		u32 reg_addr, u32 reg_val);

/* CMOS sensor Power Mode read */
void mcam_get_power_mode (p_camera_context_t camera_context,
		u8 *power_mode);

/* CMOS sensor Power Mode write */
int mcam_set_power_mode (p_camera_context_t camera_context, u8 power_mode);

/* CMOS sensor Capability read */
unsigned int mcam_get_caps (p_camera_context_t camera_context);

/* CMOS sensor contrast value read */
void mcam_get_contrast_value (p_camera_context_t camera_context,
		unsigned char *mode, unsigned int *value);

/* CMOS sensor contrast value write */
int mcam_set_contrast_value (p_camera_context_t camera_context,
		unsigned char mode, unsigned int value);

/* CMOS sensor white balance value read */
void mcam_get_white_balance_value (p_camera_context_t camera_context,
		unsigned char *mode, unsigned int *value);

/* CMOS sensor white balance value write */
int mcam_set_white_balance_value (p_camera_context_t camera_context,
		unsigned char mode, unsigned int value);

/* CMOS sensor exposure value read */
void mcam_get_exposure_value (p_camera_context_t camera_context,
		unsigned char *mode, unsigned int *value);

/* CMOS sensor exposure value write */
int mcam_set_exposure_value (p_camera_context_t camera_context,
		unsigned char mode, unsigned int value);

/* CMOS sensor zoom value read */
void mcam_get_zoom_value (p_camera_context_t camera_context,
		unsigned int *value);

/* CMOS sensor zoom value write */
int mcam_set_zoom_value (p_camera_context_t camera_context, unsigned int value);

/* get Histogram Info */
int mcam_get_histogram_info(
		p_camera_context_t     camera_context,
		unsigned int                color_type,
		unsigned int               *histogram_size,
		unsigned int               *histogram_sum);

/* sleep camera */
int mcam_suspend(p_camera_context_t    camera_context);

/* wakeup camera */
int mcam_resume(p_camera_context_t    camera_context);

#endif


