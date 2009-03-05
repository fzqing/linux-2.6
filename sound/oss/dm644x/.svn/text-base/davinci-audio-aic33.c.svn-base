/*
 * linux/sound/oss/davinci-audio-aic33.c
 *
 * Glue driver for AIC33 for Davinci processors
 *
 * Copyright (C) 2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *  -------
 *  2005-10-18 Rishi Bhattacharya - Support for AIC33 codec and Davinci DM644x
 * 					Processor
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <sound/davincisound.h>

#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include <asm/arch/mcbsp.h>
#include <asm/hardware/clock.h>
#include "davinci-aic33.h"

#include "davinci-audio.h"
#include "davinci-audio-dma-intfc.h"

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#define PROC_START_FILE "driver/aic33-audio-start"
#define PROC_STOP_FILE  "driver/aic33-audio-stop"
#endif

/* #define DEBUG */

#ifdef DEBUG
#define DPRINTK(ARGS...)        do { \
				printk(KERN_DEBUG "<%s>: ", __FUNCTION__);\
				printk(KERN_DEBUG ARGS); \
				} while (0)
#else
#define DPRINTK( x... )
#endif

#define CODEC_NAME               "AIC33"
#define PLATFORM_NAME            "DAVINCI"

/* Define to set the AIC33 as the master w.r.t McBSP */
#define AIC33_MASTER

/* codec clock frequency */
static int aic33_mclk;
#define	MCLK_22  0
#define	MCLK_27  1
#define	MCLK_33  2


/*
 * AUDIO related MACROS
 */
#define DEFAULT_BITPERSAMPLE          16
#define AUDIO_RATE_DEFAULT            48000

/* Select the McBSP For Audio */
#define AUDIO_MCBSP		      ((cpu_is_davinci_dm355()) ? \
					   DAVINCI_MCBSP2 : DAVINCI_MCBSP1)

#define REC_MASK                      (SOUND_MASK_LINE | SOUND_MASK_MIC)
#define DEV_MASK                      (REC_MASK | SOUND_MASK_VOLUME)

#define MONO			      1
#define STEREO			      2

#define SET_VOLUME                    1
#define SET_LINE                      2
#define SET_MIC                       3
#define SET_RECSRC		      4
#define SET_IGAIN		      5
#define SET_OGAIN		      6
#define SET_BASS                      7
#define SET_TREBLE                    8
#define SET_MICBIAS		      9

#define DEFAULT_OUTPUT_VOLUME         70
#define DEFAULT_INPUT_VOLUME          20	/* 0 ==> mute line in */
#define DEFAULT_INPUT_IGAIN	      20
#define DEFAULT_INPUT_OGAIN	      100

#define OUTPUT_VOLUME_MIN             LHV_MIN
#define OUTPUT_VOLUME_MAX             LHV_MAX
#define OUTPUT_VOLUME_RANGE           (OUTPUT_VOLUME_MAX - OUTPUT_VOLUME_MIN)

#define INPUT_VOLUME_MIN              LIV_MIN
#define INPUT_VOLUME_MAX              LIV_MAX
#define INPUT_VOLUME_RANGE            (INPUT_VOLUME_MAX - INPUT_VOLUME_MIN)

#define INPUT_GAIN_MIN		      LIG_MIN
#define INPUT_GAIN_MAX		      LIG_MAX
#define INPUT_GAIN_RANGE	      (INPUT_GAIN_MAX - INPUT_GAIN_MIN)

#define OUTPUT_GAIN_MIN		      LOG_MIN
#define OUTPUT_GAIN_MAX		      LOG_MAX
#define OUTPUT_GAIN_RANGE	      (INPUT_GAIN_MAX - INPUT_GAIN_MIN)

#define NUMBER_SAMPLE_RATES_SUPPORTED 11

static audio_stream_t output_stream = {
	.id = "AIC33 out",
	.input_or_output = FMODE_WRITE
};

static audio_stream_t input_stream = {
	.id = "AIC33 in",
	.input_or_output = FMODE_READ
};

static int audio_dev_id, mixer_dev_id;

static struct aic33_local_info {
	u8 volume;
	u16 volume_reg;
	u8 line;
	u8 mic;
	int recsrc;
	u8 nochan;
	u16 igain;
	u16 ogain;
	u8 micbias;
	u8 bass;
	u8 treble;
	u16 input_volume_reg;
	int mod_cnt;
} aic33_local;

struct sample_rate_reg_info {
	u32 sample_rate;
	u32 Fsref;
	float divider;
	u8 data;
};

/* To Store the default sample rate */
static long audio_samplerate = AUDIO_RATE_DEFAULT;

extern struct clk *davinci_mcbsp_get_clock(void);

/* DAC USB-mode sampling rates*/
static const struct sample_rate_reg_info
 reg_info[NUMBER_SAMPLE_RATES_SUPPORTED] = {
/*  {sample_rate, Fsref, divider, data}*/
	{96000, 96000, 1, 0x00},
	{88200, 88200, 1, 0x00},
	{48000, 48000, 1, 0x00},
	{44100, 44100, 1, 0x00},
	{32000, 48000, 1.5, 0x11},
	{24000, 96000, 4, 0x66},
	{22050, 44100, 2, 0x22},
	{16000, 48000, 3, 0x44},
	{12000, 48000, 4, 0x66},
	{11025, 44100, 4, 0x66},
	{8000, 48000, 6, 0xAA},
};

static struct davinci_mcbsp_reg_cfg initial_config = {
	.spcr2 = FREE | XINTM(3),
	.spcr1 = RINTM(3),
	.rcr2 = RWDLEN2(DAVINCI_MCBSP_WORD_16) | RDATDLY(1),
	.rcr1 = RFRLEN1(1) | RWDLEN1(DAVINCI_MCBSP_WORD_16),
	.xcr2 = XWDLEN2(DAVINCI_MCBSP_WORD_16) | XDATDLY(1) | XFIG,
	.xcr1 = XFRLEN1(1) | XWDLEN1(DAVINCI_MCBSP_WORD_16),
	.srgr1 = FWID(DEFAULT_BITPERSAMPLE - 1),
	.srgr2 = FSGM | FPER(DEFAULT_BITPERSAMPLE * 2 - 1),
#ifndef AIC33_MASTER
	/* configure McBSP to be the I2S master */
	.pcr0 = FSXM | FSRM | CLKXM | CLKRM | CLKXP | CLKRP,
#else
	/* configure McBSP to be the I2S slave */
	.pcr0 = CLKXP | CLKRP,
#endif				/* AIC33_MASTER */
};

static void davinci_aic33_initialize(void *dummy);
static void davinci_aic33_shutdown(void *dummy);
static int davinci_aic33_ioctl(struct inode *inode, struct file *file,
			       uint cmd, ulong arg);
static int davinci_aic33_probe(void);

#ifdef MODULE
static void davinci_aic33_remove(void);
#endif

static int davinci_aic33_suspend(void);
static int davinci_aic33_resume(void);
static inline void aic33_configure(void);
static int mixer_open(struct inode *inode, struct file *file);
static int mixer_release(struct inode *inode, struct file *file);
static int mixer_ioctl(struct inode *inode, struct file *file, uint cmd,
		       ulong arg);

#ifdef CONFIG_PROC_FS
static int codec_start(char *buf, char **start, off_t offset, int count,
		       int *eof, void *data);
static int codec_stop(char *buf, char **start, off_t offset, int count,
		      int *eof, void *data);
#endif

/* File Op structure for mixer */
static struct file_operations davinci_mixer_fops = {
	.open = mixer_open,
	.release = mixer_release,
	.ioctl = mixer_ioctl,
	.owner = THIS_MODULE
};

/* To store characteristic info regarding the codec for the audio driver */
static audio_state_t aic33_state = {
	.owner = THIS_MODULE,
	.output_stream = &output_stream,
	.input_stream = &input_stream,
/*    .need_tx_for_rx = 1, //Once the Full Duplex works  */
	.need_tx_for_rx = 0,
	.hw_init = davinci_aic33_initialize,
	.hw_shutdown = davinci_aic33_shutdown,
	.client_ioctl = davinci_aic33_ioctl,
	.hw_probe = davinci_aic33_probe,
	.hw_remove = __exit_p(davinci_aic33_remove),
	.hw_suspend = davinci_aic33_suspend,
	.hw_resume = davinci_aic33_resume,
	.sem = __COMPAT_MUTEX_INITIALIZER(aic33_state.sem),
};

/* This will be defined in the audio.h */
static struct file_operations *davinci_audio_fops;

extern int tlv320aic33_write_value(u8 reg, u16 value);
extern int tlv320aic33_read_value(u8 address, u8 * regValue);

/* TLV320AIC33 read */
static __inline__ void audio_aic33_read(u8 address, u8 * regValue)
{
	tlv320aic33_read_value(address, regValue);
}

/* TLV320AIC33 write */
static __inline__ void audio_aic33_write(u8 address, u16 data)
{
	if (tlv320aic33_write_value(address, data) < 0)
		printk(KERN_INFO "aic33 write failed for reg = %d\n", address);
}

static int aic33_update(int flag, int val)
{
	u16 volume;
	s16 left_gain, left_val, right_gain, right_val;

	switch (flag) {
	case SET_VOLUME:
		/* Ignore separate left/right channel for now, even the codec
		   does support it. */

		val &= 0xff;

		if (val < 0 || val > 100) {
			DPRINTK("Trying a bad volume value(%d)!\n", val);
			return -EPERM;
		}
		/* Convert 0 -> 100 volume to 0x77 (LHV_MIN) -> 0x00 (LHV_MAX)
		 */

		volume =
		    ((val * OUTPUT_VOLUME_RANGE) / 100) + OUTPUT_VOLUME_MIN;

		aic33_local.volume_reg = OUTPUT_VOLUME_MAX - volume;

		if (aic33_local.nochan == STEREO) {
			audio_aic33_write(REGISTER_ADDR47, LOPM_ON |
					  aic33_local.volume_reg);
			audio_aic33_write(REGISTER_ADDR64, LOPM_ON |
					  aic33_local.volume_reg);
			audio_aic33_write(REGISTER_ADDR82, LOPM_ON |
					  aic33_local.volume_reg);
			audio_aic33_write(REGISTER_ADDR92, LOPM_ON |
					  aic33_local.volume_reg);
		} else if (aic33_local.nochan == MONO) {
#ifdef CONFIG_MONOSTEREO_DIFFJACK
			/* DACL1 to MONO_LOP/M routing and volume control */
			audio_aic33_write(REGISTER_ADDR75, LOPM_ON |
					  aic33_local.volume_reg);
			/* DACR1 to MONO_LOP/M routing and volume control */
			audio_aic33_write(REGISTER_ADDR78, LOPM_ON |
					  aic33_local.volume_reg);
#else
			audio_aic33_write(REGISTER_ADDR47, LOPM_ON |
					  aic33_local.volume_reg);
			audio_aic33_write(REGISTER_ADDR64, LOPM_ON |
					  aic33_local.volume_reg);
			audio_aic33_write(REGISTER_ADDR82, LOPM_ON |
					  aic33_local.volume_reg);
			audio_aic33_write(REGISTER_ADDR92, LOPM_ON |
					  aic33_local.volume_reg);
#endif
		}

		break;

	case SET_LINE:
	case SET_MIC:
		/* Ignore separate left/right channel for now, even the codec
		   does support it. */

		val &= 0xff;

		if (val < 0 || val > 100) {
			DPRINTK("Trying a bad volume value(%d)!\n", val);
			return -EPERM;
		}

		volume = ((val * INPUT_VOLUME_RANGE) / 100) + INPUT_VOLUME_MIN;

		aic33_local.input_volume_reg = volume;

		audio_aic33_write(REGISTER_ADDR15,
				  aic33_local.input_volume_reg);
		audio_aic33_write(REGISTER_ADDR16,
				  aic33_local.input_volume_reg);
		break;

	case SET_RECSRC:
		/* Ignore separate left/right channel for now, even the codec
		   does support it. */

		val &= 0xff;

		if (hweight32(val) > 1)
			val &= ~aic33_local.recsrc;

		if (val == SOUND_MASK_MIC) {
			/* enable the mic input*/
			DPRINTK("Enabling mic\n");
			audio_aic33_write(REGISTER_ADDR17, 0x0);
			audio_aic33_write(REGISTER_ADDR18, 0x0);

			/* enable ADC's and disable the line input*/
			audio_aic33_write(REGISTER_ADDR19, 0x7C);
			audio_aic33_write(REGISTER_ADDR22, 0x7C);

		} else if (val == SOUND_MASK_LINE) {
			/* enable ADC's, enable line iput */
			DPRINTK(" Enabling line in\n");
			audio_aic33_write(REGISTER_ADDR19, 0x4);
			audio_aic33_write(REGISTER_ADDR22, 0x4);

			/* disable the mic input */
			audio_aic33_write(REGISTER_ADDR17, 0xff);
			audio_aic33_write(REGISTER_ADDR18, 0xff);
		} else {
			/* do nothing */
		}
		aic33_local.recsrc = val;
		break;

	case SET_IGAIN:
		left_val = val & 0xFF;
		right_val = val >> 8;

		if (left_val < 0 || left_val > 100) {
			DPRINTK("Trying a bad igain value(%d)!\n", left_val);
			return -EPERM;
		}
		if (right_val < 0 || right_val > 100) {
			DPRINTK("Trying a bad igain value(%d)!\n", right_val);
			return -EPERM;
		}

		left_gain = ((left_val * INPUT_GAIN_RANGE) / 100) +
			INPUT_GAIN_MIN;
		right_gain = ((right_val * INPUT_GAIN_RANGE) / 100) +
			INPUT_GAIN_MIN;

		DPRINTK("left gain reg val = 0x%x", left_gain << 1);
		DPRINTK("right gain reg val = 0x%x", left_gain << 1);

		/* Left AGC control */
		audio_aic33_write(REGISTER_ADDR26, 0x80);
		audio_aic33_write(REGISTER_ADDR27, left_gain << 1);
		audio_aic33_write(REGISTER_ADDR28, 0x0);

		/* Right AGC control */
		audio_aic33_write(REGISTER_ADDR29, 0x80);
		audio_aic33_write(REGISTER_ADDR30, right_gain << 1);
		audio_aic33_write(REGISTER_ADDR31, 0x0);

		break;

	case SET_OGAIN:
		left_val = val & 0xFF;
		right_val = val >> 8;

		if (left_val < 0 || left_val > 100) {
			DPRINTK("Trying a bad igain value(%d)!\n", left_val);
			return -EPERM;
		}
		if (right_val < 0 || right_val > 100) {
			DPRINTK("Trying a bad igain value(%d)!\n", right_val);
			return -EPERM;
		}

		left_gain = ((left_val * OUTPUT_GAIN_RANGE) / 100) +
			OUTPUT_GAIN_MIN;
		left_gain = OUTPUT_GAIN_MAX - left_gain;
		right_gain = ((right_val * OUTPUT_GAIN_RANGE) / 100) +
			OUTPUT_GAIN_MIN;
		right_gain = OUTPUT_GAIN_MAX - right_gain;

		/* Left/Right DAC digital volume gain */
		audio_aic33_write(REGISTER_ADDR43, left_gain);
		audio_aic33_write(REGISTER_ADDR44, right_gain);
		break;

	case SET_MICBIAS:
		/* Ignore separate left/right channel for now, even the codec
		   does support it. */

		val &= 0xff;

		if (val < 0 || val > 3) {
			DPRINTK
			    ("Request for non supported mic bias level(%d)!\n",
			     val);
			return -EPERM;
		}

		if (val == 0)
			audio_aic33_write(REGISTER_ADDR25, 0x00);

		else if (val == 1)
			audio_aic33_write(REGISTER_ADDR25, MICBIAS_OUTPUT_2_0V);

		else if (val == 2)
			audio_aic33_write(REGISTER_ADDR25, MICBIAS_OUTPUT_2_5V);

		else if (val == 3)
			audio_aic33_write(REGISTER_ADDR25, MICBIAS_OUTPUT_AVDD);

		break;

	case SET_BASS:
		break;

	case SET_TREBLE:
		break;
	}
	return 0;
}

static int mixer_open(struct inode *inode, struct file *file)
{
	/* Any mixer specific initialization */
	return 0;
}

static int mixer_release(struct inode *inode, struct file *file)
{
	/* Any mixer specific Un-initialization */
	return 0;
}

static int
mixer_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
	int val;
	int ret = 0;
	int nr = _IOC_NR(cmd);

	/*
	 * We only accept mixer (type 'M') ioctls.
	 */
	if (_IOC_TYPE(cmd) != 'M')
		return -EINVAL;

	DPRINTK(" 0x%08x\n", cmd);

	if (cmd == SOUND_MIXER_INFO) {
		struct mixer_info mi;

		strncpy(mi.id, "AIC33", sizeof(mi.id));
		strncpy(mi.name, "TI AIC33", sizeof(mi.name));
		mi.modify_counter = aic33_local.mod_cnt;

		return copy_to_user((void *)arg, &mi, sizeof(mi));
	}

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		ret = get_user(val, (int *)arg);
		if (ret)
			goto out;

		switch (nr) {
		case SOUND_MIXER_VOLUME:
			aic33_local.mod_cnt++;
			ret = aic33_update(SET_VOLUME, val);
			if (!ret)
				aic33_local.volume = val;
			break;

		case SOUND_MIXER_LINE:
			aic33_local.mod_cnt++;
			ret = aic33_update(SET_LINE, val);
			if (!ret)
				aic33_local.line = val;
			break;

		case SOUND_MIXER_MIC:
			aic33_local.mod_cnt++;
			ret = aic33_update(SET_MIC, val);
			if (!ret)
				aic33_local.mic = val;
			break;

		case SOUND_MIXER_RECSRC:
			if ((val & SOUND_MASK_LINE) ||
			    (val & SOUND_MASK_MIC)) {
				if (aic33_local.recsrc != val) {
					aic33_local.mod_cnt++;
					aic33_update(SET_RECSRC, val);
				}
			} else {
				ret = -EINVAL;
			}
			break;

		case SOUND_MIXER_BASS:
			aic33_local.mod_cnt++;
			ret = aic33_update(SET_BASS, val);
			if (!ret)
				aic33_local.bass = val;
			break;

		case SOUND_MIXER_TREBLE:
			aic33_local.mod_cnt++;
			ret = aic33_update(SET_TREBLE, val);
			if (!ret)
				aic33_local.treble = val;
			break;

		case SOUND_MIXER_IGAIN:
			aic33_local.mod_cnt++;
			ret = aic33_update(SET_IGAIN, val);
			if (!ret)
				aic33_local.igain = val;
			break;

		case SOUND_MIXER_OGAIN:
			aic33_local.mod_cnt++;
			ret = aic33_update(SET_OGAIN, val);
			if (!ret)
				aic33_local.ogain = val;
			break;

		case SOUND_MIXER_MICBIAS:
			aic33_local.mod_cnt++;
			ret = aic33_update(SET_MICBIAS, val);
			if (!ret)
				aic33_local.micbias = val;
			break;

		default:
			ret = -EINVAL;
		}
	}

	if (ret == 0 && _IOC_DIR(cmd) & _IOC_READ) {
		ret = 0;

		switch (nr) {
		case SOUND_MIXER_VOLUME:
			val = aic33_local.volume;
			break;
		case SOUND_MIXER_LINE:
			val = aic33_local.line;
			break;
		case SOUND_MIXER_MIC:
			val = aic33_local.mic;
			break;
		case SOUND_MIXER_RECSRC:
			val = aic33_local.recsrc;
			break;
		case SOUND_MIXER_RECMASK:
			val = REC_MASK;
			break;
		case SOUND_MIXER_IGAIN:
			val = aic33_local.igain;
			break;
		case SOUND_MIXER_OGAIN:
			val = aic33_local.ogain;
			break;
		case SOUND_MIXER_DEVMASK:
			val = DEV_MASK;
			break;
		case SOUND_MIXER_BASS:
			val = aic33_local.bass;
			break;
		case SOUND_MIXER_TREBLE:
			val = aic33_local.treble;
			break;
		case SOUND_MIXER_CAPS:
			val = 0;
			break;
		case SOUND_MIXER_STEREODEVS:
			val = SOUND_MASK_VOLUME;
			break;
		case SOUND_MIXER_MICBIAS:
			val = aic33_local.micbias;
			break;
		default:
			val = 0;
			ret = -EINVAL;
			break;
		}

		if (ret == 0)
			ret = put_user(val, (int *)arg);
	}
out:
	return ret;
}

int davinci_set_samplerate(long sample_rate)
{
	u8 count = 0;
	u8 j_val = 0;
	u16 d_val = 0;

	/* wait for any frame to complete */
	udelay(125);

	/* Search for the right sample rate */
	while ((reg_info[count].sample_rate != sample_rate) &&
	       (count < NUMBER_SAMPLE_RATES_SUPPORTED)) {
		count++;
	}

	if (count == NUMBER_SAMPLE_RATES_SUPPORTED) {
		DPRINTK("Invalid Sample Rate %d requested\n", (int)sample_rate);
		return -EPERM;
	}

	/*   CODEC DATAPATH SETUP  */

	/* Fsref to 48kHz, dual rate mode upto 96kHz */
	if (reg_info[count].Fsref == 96000)
		audio_aic33_write(REGISTER_ADDR7,
				  FS_REF_DEFAULT_48 | ADC_DUAL_RATE_MODE |
				  DAC_DUAL_RATE_MODE | LDAC_LCHAN | RDAC_RCHAN);

	/* Fsref to 44.1kHz, dual rate mode upto 88.2kHz */
	else if (reg_info[count].Fsref == 88200)
		audio_aic33_write(REGISTER_ADDR7,
				  FS_REF_44_1 | ADC_DUAL_RATE_MODE |
				  DAC_DUAL_RATE_MODE | LDAC_LCHAN | RDAC_RCHAN);

	/* Fsref to 48kHz */
	else if (reg_info[count].Fsref == 48000)
		audio_aic33_write(REGISTER_ADDR7,
				  FS_REF_DEFAULT_48 | LDAC_LCHAN | RDAC_RCHAN);

	/* Fsref to 44.1kHz */
	else if (reg_info[count].Fsref == 44100)
		audio_aic33_write(REGISTER_ADDR7, FS_REF_44_1 | LDAC_LCHAN |
				  RDAC_RCHAN);

	/* Codec sample rate select */
	audio_aic33_write(REGISTER_ADDR2, reg_info[count].data);

	/* If PLL is to be used for generation of Fsref
	   Generate the Fsref using the PLL */

	/*Enable the PLL | Q-value | P-value */
	audio_aic33_write(REGISTER_ADDR3, PLL_ENABLE | 0x10 | 0x02);

	if ((reg_info[count].Fsref == 96000) ||
	    (reg_info[count].Fsref == 48000)) {
		/*
		 * For MCLK = 22.5792 MHz and to get Fsref = 48kHz
		 * Fsref = (MCLK * k * R)/(2048 * p);
		 * Select P = 2, R= 1, K = 8.7075, which results in J = 8,
		 * D = 7075
		 *
		 * For MCLK = 27 MHz and to get Fsref = 48kHz
		 * Fsref = (MCLK * k * R)/(2048 * p);
		 * Select P = 2, R= 1, K = 7.2818, which results in J = 7,
		 * D = 2818
		 *
		 * For MCLK = 33.8688 MHz and to get Fsref = 48kHz
		 * Fsref = (MCLK * k * R)/(2048 * p);
		 * Select P = 2, R= 1, K = 5.8049, which results in J = 5,
		 * D = 8049
		 */

		switch (aic33_mclk) {
		case MCLK_22:
			j_val = 8;
			d_val = 7075;
			break;
		case MCLK_27:
			j_val = 7;
			d_val = 2818;
			break;
		case MCLK_33:
			j_val = 5;
			d_val = 8049;
			break;
		default:
			printk(KERN_ERR "unknown audio codec frequency \n");
		}
	} else if ((reg_info[count].Fsref == 88200) ||
		   (reg_info[count].Fsref == 44100)) {
		/*
		 * MCLK = 22.5792 MHz and to get Fsref = 44.1kHz
		 * Fsref = (MCLK * k * R)/(2048 * p);
		 * Select P = 2, R =1, K = 8.0000, which results in J = 8,
		 * D = 0000
		 *
		 * MCLK = 27 MHz and to get Fsref = 44.1kHz
		 * Fsref = (MCLK * k * R)/(2048 * p);
		 * Select P = 2, R =1, K = 6.6901, which results in J = 6,
		 * D = 6901
		 *
		 * MCLK = 33.8688 MHz and to get Fsref = 44.1kHz
		 * Fsref = (MCLK * k * R)/(2048 * p);
		 * Select P = 2, R =1, K = 5.3333, which results in J = 5,
		 * D = 3333
		 */

		switch (aic33_mclk) {
		case MCLK_22:
			j_val = 8;
			d_val = 0;
			break;
		case MCLK_27:
			j_val = 6;
			d_val = 6901;
			break;
		case MCLK_33:
			j_val = 5;
			d_val = 3333;
			break;
		default:
			printk(KERN_ERR "unknown audio codec frequency \n");
		}
	}

	/* J-value */
	audio_aic33_write(REGISTER_ADDR4, j_val << 2);
	/* D-value 8-MSB's */
	audio_aic33_write(REGISTER_ADDR5, (unsigned char)(d_val >> 6));
	/* D-value 6-LSB's */
	audio_aic33_write(REGISTER_ADDR6, (unsigned char)(d_val << 2));

	audio_samplerate = sample_rate;

#ifndef AIC33_MASTER
	{
		int clkgdv = 0;
		unsigned long clkval = 0;
		struct clk *mbspclk;

		/*
		   Set Sample Rate at McBSP

		   Formula :
		   Codec System Clock = Input clock to McBSP;
		   clkgdv = ((Codec System Clock /
				(SampleRate * BitsPerSample * 2)) - 1);

		   FWID = BitsPerSample - 1;
		   FPER = (BitsPerSample * 2) - 1;
		 */

		mbspclk = davinci_mcbsp_get_clock();
		if (mbspclk == NULL) {
			DPRINTK(" Failed to get internal clock to MCBSP");
			return -EPERM;
		}

		clkval = clk_get_rate(mbspclk);
		DPRINTK("mcbsp_clk = %ld\n", clkval);

		if (clkval)
			clkgdv =
			    (clkval /
			     (sample_rate * DEFAULT_BITPERSAMPLE * 2)) - 1;
		else {
			DPRINTK(" Failed to get the MCBSP clock\n");
			return -EPERM;
		}

		DPRINTK("clkgdv = %d\n", clkgdv);

		if (clkgdv > 255 || clkgdv < 0) {

			/* For requested sampling rate, the input clock to MCBSP
			   cant be devided down to get the in range clock
			   divider value for 16 bits sample */

			DPRINTK("Invalid Sample Rate %d requested\n",
				(int)sample_rate);
			return -EPERM;
		}

		initial_config.srgr1 =
		    (FWID(DEFAULT_BITPERSAMPLE - 1) | CLKGDV(clkgdv));

		initial_config.srgr2 =
		    (CLKSM | FSGM | FPER(DEFAULT_BITPERSAMPLE * 2 - 1));

		davinci_mcbsp_stop_tx(AUDIO_MCBSP);
		davinci_mcbsp_stop_rx(AUDIO_MCBSP);
		davinci_mcbsp_config(AUDIO_MCBSP, &initial_config);
	}
#endif				/* AIC33_MASTER */

	return 0;
}

static void davinci_aic33_shutdown(void *dummy)
{
	/*
	   Turn off codec after it is done.
	   Can't do it immediately, since it may still have
	   buffered data.

	   Wait 20ms (arbitrary value) and then turn it off.
	 */

	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(2);

	davinci_mcbsp_stop_tx(AUDIO_MCBSP);
	davinci_mcbsp_stop_rx(AUDIO_MCBSP);
	davinci_mcbsp_free(AUDIO_MCBSP);

}

static void davinci_set_mono_stereo(int mode)
{
	if (mode == MONO) {

#ifdef CONFIG_MONOSTEREO_DIFFJACK
		/* MONO_LOP/M Output level control register */
		audio_aic33_write(REGISTER_ADDR79, 0x99);
#else
		/* Driver power ON pop control */
		audio_aic33_write(REGISTER_ADDR42, 0x6C);

		/* HPLOUT/HPROUT output level control */
		audio_aic33_write(REGISTER_ADDR51, 0x99);
		audio_aic33_write(REGISTER_ADDR65, 0x99);

		/* LEFT_LOP/M, RIGHT_LOP/M output level control */
		audio_aic33_write(REGISTER_ADDR86, 0x99);
		audio_aic33_write(REGISTER_ADDR93, 0x99);
#endif
		/* Left DAC power up, Right DAC power down */
		audio_aic33_write(REGISTER_ADDR37, 0xa0);
	} else if (mode == STEREO) {
		/* Driver power ON pop control */
		audio_aic33_write(REGISTER_ADDR42, 0x6C);

		/* HPLOUT/HPROUT output level control */
		audio_aic33_write(REGISTER_ADDR51, 0x99);
		audio_aic33_write(REGISTER_ADDR65, 0x99);

		/* LEFT_LOP/M, RIGHT_LOP/M output level control */
		audio_aic33_write(REGISTER_ADDR86, 0x99);
		audio_aic33_write(REGISTER_ADDR93, 0x99);

		/* Left/Right DAC power up */
		audio_aic33_write(REGISTER_ADDR37, 0xe0);
	} else
		DPRINTK(" REQUEST FOR INVALID MODE\n");
}

static inline void aic33_configure()
{
	DPRINTK(" CONFIGURING AIC33\n");

	/* Page select register */
	audio_aic33_write(REGISTER_ADDR0, 0x0);

	/* audio_aic33_write(REGISTER_ADDR38, 0x10); */
	davinci_set_mono_stereo(aic33_local.nochan);
#ifdef AIC33_MASTER
	/* Enable bit and word clock as Master mode, 3-d disabled */
	audio_aic33_write(REGISTER_ADDR8, 0xc0 /*0xc4 */ );
#endif

	aic33_update(SET_LINE, aic33_local.line);
	aic33_update(SET_VOLUME, aic33_local.volume);
	aic33_update(SET_RECSRC, aic33_local.recsrc);
	aic33_update(SET_IGAIN, aic33_local.igain);
	aic33_update(SET_OGAIN, aic33_local.ogain);
	aic33_update(SET_MICBIAS, aic33_local.micbias);
}

static void davinci_aic33_initialize(void *dummy)
{
	DPRINTK("entry\n");

	/* initialize with default sample rate */
	audio_samplerate = AUDIO_RATE_DEFAULT;

	if (davinci_mcbsp_request(AUDIO_MCBSP) < 0) {
		DPRINTK("MCBSP request failed\n");
		return;
	}

	/* if configured, then stop mcbsp */
	davinci_mcbsp_stop_tx(AUDIO_MCBSP);
	davinci_mcbsp_stop_rx(AUDIO_MCBSP);

	/* set initial (default) sample rate */
	davinci_set_samplerate(audio_samplerate);

	davinci_mcbsp_config(AUDIO_MCBSP, &initial_config);

	DPRINTK("exit\n");
}

static int
davinci_aic33_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
	long val;
	int ret = 0;

	DPRINTK(" 0x%08x\n", cmd);

	/*
	 * These are platform dependent ioctls which are not handled by the
	 * generic davinci-audio module.
	 */
	switch (cmd) {
	case SNDCTL_DSP_STEREO:
		ret = get_user(val, (int *)arg);
		if (ret)
			return ret;
		/* the Davinci supports AIC33 as stereo, mono on stereo jack */
		ret = (val == 0) ? -EINVAL : 1;
		return put_user(ret, (int *)arg);

	case SNDCTL_DSP_CHANNELS:

		ret = get_user(val, (long *)arg);
		if (ret) {
			DPRINTK("get_user failed\n");
			break;
		}
		if (val == STEREO) {
			DPRINTK("Driver support for AIC33 as stereo\n");
			aic33_local.nochan = STEREO;
			davinci_set_mono_stereo(aic33_local.nochan);
		} else if (val == MONO) {
			DPRINTK("Driver support for AIC33 as mono\n");
			aic33_local.nochan = MONO;
			davinci_set_mono_stereo(aic33_local.nochan);
		} else {
			DPRINTK
			    ("Driver support for AIC33 as stereo/mono mode\n");
			return -EPERM;
		}

	case SOUND_PCM_READ_CHANNELS:
		/* the Davinci supports AIC33 as stereo, mono on stereo jack */
		if (aic33_local.nochan == MONO)
			return put_user(MONO, (long *)arg);
		else
			return put_user(STEREO, (long *)arg);

	case SNDCTL_DSP_SPEED:
		ret = get_user(val, (long *)arg);
		if (ret) {
			DPRINTK("get_user failed\n");
			break;
		}
		ret = davinci_set_samplerate(val);
		if (ret) {
			DPRINTK("davinci_set_samplerate failed\n");
			break;
		}
		/* fall through */

	case SOUND_PCM_READ_RATE:
		return put_user(audio_samplerate, (long *)arg);

	case SNDCTL_DSP_SETFMT:	/* set Format */
		ret = get_user(val, (long *)arg);
		if (ret) {
			DPRINTK("get_user failed\n");
			break;
		}
		if (val != AFMT_S16_LE) {
			DPRINTK
			    ("Driver supports only AFMT_S16_LE audio format\n");
			return -EPERM;
		}

	case SOUND_PCM_READ_BITS:
	case SNDCTL_DSP_GETFMTS:
		/* we can do 16-bit only */
		return put_user(AFMT_S16_LE, (long *)arg);

	default:
		/* Maybe this is meant for the mixer (As per OSS Docs) */
		return mixer_ioctl(inode, file, cmd, arg);
	}

	return ret;
}

static int davinci_aic33_probe(void)
{
	davinci_audio_fops = audio_get_fops();

	/* Get the fops from audio oss driver */
	if (!davinci_audio_fops) {
		DPRINTK
		    ("Unable to get the file operations for AIC33 OSS\
			driver\n");

		audio_unregister_codec(&aic33_state);
		return -EPERM;
	}

	aic33_local.volume = DEFAULT_OUTPUT_VOLUME;
	aic33_local.line = DEFAULT_INPUT_VOLUME;
	/* either of SOUND_MASK_LINE/SOUND_MASK_MIC */
	aic33_local.recsrc = SOUND_MASK_LINE;
	aic33_local.igain = DEFAULT_INPUT_IGAIN | (DEFAULT_INPUT_IGAIN << 8);
	aic33_local.ogain = DEFAULT_INPUT_OGAIN | (DEFAULT_INPUT_OGAIN << 8);
	aic33_local.nochan = STEREO;
	aic33_local.micbias = 1;
	aic33_local.mod_cnt = 0;

	/* register devices */
	audio_dev_id = register_sound_dsp(davinci_audio_fops, -1);
	mixer_dev_id = register_sound_mixer(&davinci_mixer_fops, -1);

#ifdef CONFIG_PROC_FS
	create_proc_read_entry(PROC_START_FILE, 0 /* default mode */ ,
			       NULL /* parent dir */ ,
			       codec_start, NULL /* client data */ );

	create_proc_read_entry(PROC_STOP_FILE, 0 /* default mode */ ,
			       NULL /* parent dir */ ,
			       codec_stop, NULL /* client data */ );
#endif

	/* Announcement Time */
	DPRINTK(PLATFORM_NAME " " CODEC_NAME " audio support initialized\n");
	return 0;
}

#ifdef MODULE
static void __exit davinci_aic33_remove(void)
{
	/* Un-Register the codec with the audio driver */
	unregister_sound_dsp(audio_dev_id);
	unregister_sound_mixer(mixer_dev_id);

#ifdef CONFIG_PROC_FS
	remove_proc_entry(PROC_START_FILE, NULL);
	remove_proc_entry(PROC_STOP_FILE, NULL);
#endif
}
#endif				/* MODULE */

static int davinci_aic33_suspend(void)
{
	/* Empty for the moment */
	return 0;
}

static int davinci_aic33_resume(void)
{
	/* Empty for the moment */
	return 0;
}

static int __init audio_aic33_init(void)
{
	int err = 0;

	if (cpu_is_davinci_dm355()) {
		input_stream.dma_dev = DM355_DMA_MCBSP1_RX;
		output_stream.dma_dev = DM355_DMA_MCBSP1_TX;
		aic33_mclk = MCLK_27;
	}

	if (cpu_is_davinci_dm644x()) {
		input_stream.dma_dev = DAVINCI_DMA_MCBSP_RX;
		output_stream.dma_dev = DAVINCI_DMA_MCBSP_TX;
		aic33_mclk = MCLK_22;
	}

	err = audio_register_codec (&aic33_state);

	/* register the codec with the audio driver */
	if (err) {
		DPRINTK
		    ("Failed to register AIC33 driver with Audio OSS Driver\n");
	} else {
		DPRINTK("codec driver register success\n");
	}

	/* configure aic33 with default params */
	aic33_configure();

	return err;
}

static void __exit audio_aic33_exit(void)
{
	davinci_aic33_shutdown(NULL);
	(void)audio_unregister_codec(&aic33_state);
	return;
}

#ifdef CONFIG_PROC_FS
static int codec_start(char *buf, char **start, off_t offset, int count,
		       int *eof, void *data)
{
	davinci_aic33_initialize(NULL);

	DPRINTK("AIC33 codec initialization done.\n");
	return 0;
}

static int codec_stop(char *buf, char **start, off_t offset, int count,
		      int *eof, void *data)
{
	davinci_aic33_shutdown(NULL);

	DPRINTK("AIC33 codec shutdown.\n");
	return 0;
}
#endif				/* CONFIG_PROC_FS */

module_init(audio_aic33_init);
module_exit(audio_aic33_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Glue audio driver for the TI AIC33 codec.");
MODULE_LICENSE("GPL");
