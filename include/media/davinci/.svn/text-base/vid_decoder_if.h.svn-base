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
/* vid_decoder_if.h */

/* This file contains data structure which will be used as the interface
 * between vpif and decoder */

#ifndef VID_DECODER_IF_H
#define VID_DECODER_IF_H

#ifdef __KERNEL__

#include <linux/videodev.h>
#include <linux/videodev2.h>

#define DECODER_MAX_NAME		(50)

#define FRAME_FORMAT_PROGRESSIVE        (1)
#define FRAME_FORMAT_INTERLACED         (0)

#define DECODER_I2C_BIND_FLAG		(0)
#define DECODER_FULL_INIT_FLAG		(1)

typedef enum {
	INTERFACE_TYPE_BT656 = 1,
	INTERFACE_TYPE_BT1120 = 2,
	INTERFACE_TYPE_RAW = 4
} vid_capture_interface_type;

/* structures */

/* Parameters ops structure */
struct param_ops {
	int (*setparams) (void *params, void *dec);	/* Pointer to function
							   to set parameters */
	int (*getparams) (void *params, void *dec);	/* Pointer to function
							   to get parameters */
};

/* standard ops structure */
struct standard_ops {
	int count;		/* Indicates number of standards
				   supported */
	int (*enumstd) (struct v4l2_standard *argp, void *dec);	/* Pointer to
									   function to
									   enumerate                                                                     standard */
	int (*querystd) (v4l2_std_id *argp, void *dec);	/* Pointer to function
								   to query standard */
	int (*setstd) (v4l2_std_id *argp, void *dec);	/* Pointer to
							   function to set
							   standard */
	int (*getstd) (v4l2_std_id *argp, void *dec);	/* Pointer to
							   function to get
							   standard */
};

/* format ops structure */
struct format_ops {
	int count;		/* Indicats number of formats
				   supported */
	int (*enumformat) (struct v4l2_fmtdesc *argp, void *dec);	/* Pointer
									   to
									   function
									   to
									   enumerate
									   formats */
	int (*tryformat) (struct v4l2_format *argp, void *dec);	/* Pointer to
									   function
									   to try
									   format */
	int (*setformat) (struct v4l2_format *argp, void *dec);	/* Pointer
									   to function
									   to set
									   formats */
	int (*getformat) (struct v4l2_format *argp, void *dec);	/* Pointer
									   to function
									   to get
									   formats */
};

/* control ops structure */
struct control_ops {
	int count;		/* Indicats number of controls
				   supported */
	int (*queryctrl) (struct v4l2_queryctrl *argp, void *dec);	/* Pointer
									   to
									   function
									   to
									   enumerate
									   controls */
	int (*setcontrol) (struct v4l2_control *argp, void *dec);	/* Pointer
									   to
									   function
									   to set
									   controls */
	int (*getcontrol) (struct v4l2_control *argp, void *dec);	/* Pointer
									   to
									   function
									   to get
									   controls */
};

/* input ops structure */
struct input_ops {
	int count;		/* Indicats number of
				   inputs supported */
	int (*enuminput) (struct v4l2_input *argp, void *dec);	/* Pointer to
								   function to
								   query
								   inputs */
	int (*setinput) (int *argp, void *dec);	/* Pointer to function
						   to set inputs */
	int (*getinput) (int *argp, void *dec);	/* Pointer to function
						   to get inputs */
};

/* decoder device structure */
struct decoder_device {
	u8 name[DECODER_MAX_NAME];	/* Name of the
					   decoder device */
	vid_capture_interface_type if_type;	/* Decoder interface
						   type i.e. BT656 */
	int channel_id;		/* Id of the channel
				   to which decoder
				   is connected */
	u32 capabilities;	/* decoder
				   capabilities */
	int (*initialize) (void *dec, int flag);	/* Pointer to
							   initialize
							   function to
							   initialize
							   decoder */
	struct standard_ops *std_ops;	/* Set of functions
					   pointers for
					   standard related
					   functions */
	struct control_ops *ctrl_ops;	/* Set of functions
					   pointers for
					   control related
					   functions */
	struct input_ops *input_ops;	/* Set of functions
					   pointers for input
					   related
					   functions */
	struct format_ops *fmt_ops;	/* Set of functions
					   pointers for
					   format related
					   functions */
	struct param_ops *params_ops;	/* Set of functions
					   pointers for
					   device specific
					   configs */
	int (*deinitialize) (void *dec);	/* Pointer to
						   deinitialize
						   function */
	int (*read_vbi_data) (struct v4l2_sliced_vbi_data *data, void *dec);
	/* This function
	   will be called
	   whenever the
	   sliced vbi data
	   has to be get
	   from the decoder */
	int (*get_sliced_vbi_cap) (struct v4l2_sliced_vbi_cap *cap, void *dec);
	/* This function
	   will be called
	   whenver all the
	   sliced vbi
	   services needs
	   to be get from
	   the decoder */

};

int vpif_register_decoder(struct decoder_device
			  *decoder);	/* Function to register decoder to the
					   VPIF-V4L2 layer */
int vpif_unregister_decoder(struct decoder_device
			    *decoder);	/* Function to un-register decoder 
					   to the
					   VPIF-V4L2 layer */

#endif				/* #ifdef __KERNEL__ */

#endif				/* #ifdef VID_DECODER_IF_H */
