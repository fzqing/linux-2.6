/*
 * linux/sound/oss/davinci-audio-aic32.c
 *
 * Glue driver for AIC32 for Davinci processors
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
 *                                  Processor
 *
 *  2007-07-06   Nirmal Pandey, Suresh Rajashekara - Included new sound driver
 *              arch (to support multiple ASP types and codec types)
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

#include <asm/hardware/clock.h>
#include "davinci-aic32.h"
#include <asm/arch/evm_audio_info.h>

#include "audio_controller.h"
#include "davinci-audio-dma-intfc.h"


#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#define PROC_START_FILE "driver/aic32-audio-start"
#define PROC_STOP_FILE  "driver/aic32-audio-stop"
#endif

/* #define DEBUG */

#ifdef DEBUG
#define DPRINTK(ARGS...)        do { \
					printk(KERN_DEBUG "<%s>: ", \
					       __FUNCTION__);\
					printk(KERN_DEBUG ARGS);\
				} while (0)
#else
#define DPRINTK( x... )
#endif

#define CODEC_NAME               "AIC32"
#define PLATFORM_NAME            "DAVINCI"

/* Define to set the AIC32 as the master w.r.t McBSP/McASP */
#define AIC32_MASTER

/* codec clock frequency */
#define MCLK  22

#ifdef DEBUG

#define FN_IN   printk(KERN_DEBUG "Entering Function %s\n", __FUNCTION__ );
#define FN_OUT  printk(KERN_DEBUG "Exiting Function %s\n", __FUNCTION__ );

#else  /* DEBUG */

#define FN_IN
#define FN_OUT

#endif /* DEBUG */
/*
 * AUDIO related MACROS
 */
#define DEFAULT_BITPERSAMPLE          16
#define AUDIO_RATE_DEFAULT            48000
#define DEFAULT_MCBSP_CLOCK           81000000

/* Select the McBSP For Audio */
#define AUDIO_MCBSP                   DAVINCI_MCBSP1

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

static struct aic32_local_info {
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
} aic32_local;

struct sample_rate_reg_info {
	u32 sample_rate;
	u32 Fsref;
	float divider;
	u8 data;
};

/* To Store the default sample rate */
static long audio_samplerate = AUDIO_RATE_DEFAULT;

#define NUMBER_SAMPLE_RATES_SUPPORTED 34

/* DAC USB-mode sampling rates*/
static const struct sample_rate_reg_info
 reg_info[NUMBER_SAMPLE_RATES_SUPPORTED] = {
/*  {sample_rate, Fsref, divider, data}*/
	{96000, 96000, 1, 0x00},
	{88200, 88200, 1, 0x00},
	{64000, 96000, 1.5, 0x11}, /* New */
	{58800, 88200, 1.5, 0x11}, /* New */
	{48000, 48000, 1, 0x00},
	{44100, 44100, 1, 0x00},
	{38400, 96000, 2.5, 0x33}, /* New */
	{35280, 88200, 2.5, 0x33}, /* New */
	{32000, 48000, 1.5, 0x11},
	{29400, 44100, 1.5, 0x11}, /* New */
	{27429, 96000, 3.5, 0x55}, /* New */
	{25200, 88200, 3.5, 0x55}, /* New */
	{24000, 96000, 4, 0x66},
	{22050, 44100, 2, 0x22},
	{21332, 96000, 4.5, 0x66}, /* New */
	{19600, 88200, 4.5, 0x66}, /* New */
	{19200, 48000, 2.5, 0x33}, /* New */
	{17640, 44100, 2.5, 0x33}, /* New */
	{17455, 96000, 5.5, 0x88}, /* New */
	{16036, 88200, 5.5, 0x88}, /* New */
	{16000, 48000, 3, 0x44},
	{14700, 44100, 3, 0x44}, /* New */
	{13714, 48000, 3.5, 0x55}, /* New */
	{12600, 44100, 3.5, 0x55}, /* New */
	{12000, 48000, 4, 0x66},
	{11025, 44100, 4, 0x66},
	{10667, 48000, 4.5, 0x77}, /* New */
	{9800, 44100, 4.5, 0x77}, /* New */
	{9600, 48000, 5, 0x88},	/* New */
	{8820, 44100, 5, 0x88},	/* New */
	{8727, 48000, 5.5, 0x99}, /* New */
	{8018, 44100, 5.5, 0x99}, /* New */
	{8000, 48000, 6, 0xAA},
	{7350, 44100, 6, 0xAA},	/* New */
 };

static void davinci_aic32_initialize(void *dummy);
/* static void davinci_aic32_shutdown(void *dummy); */
int davinci_aic32_ioctl(struct inode *inode, struct file *file,
			       uint cmd, ulong arg);

/* static int davinci_aic32_probe(void); */

/* #ifdef MODULE */
/* static void davinci_aic32_remove(void); */
/* #endif */

/* static int davinci_aic32_suspend(void); */
/* static int davinci_aic32_resume(void); */

static inline void aic32_configure(audio_config_t *audio_cfg);
static int mixer_open(struct inode *inode, struct file *file);
static int mixer_release(struct inode *inode, struct file *file);
static int mixer_ioctl(struct inode *inode, struct file *file, uint cmd,
		       ulong arg);

s8 aic32_config(u8 id, void *ptr)
{
    void *dummy;
    audio_config_t *audio_cfg = (audio_config_t *)ptr;

    aic32_local.volume = DEFAULT_OUTPUT_VOLUME;
    aic32_local.line = DEFAULT_INPUT_VOLUME;
    /* either of SOUND_MASK_LINE/SOUND_MASK_MIC */
    aic32_local.recsrc = SOUND_MASK_LINE;
    aic32_local.igain = DEFAULT_INPUT_IGAIN | (DEFAULT_INPUT_IGAIN << 8);
    aic32_local.ogain = DEFAULT_INPUT_OGAIN | (DEFAULT_INPUT_OGAIN << 8);
    aic32_local.nochan = STEREO;
    aic32_local.micbias = 1;
    aic32_local.mod_cnt = 0;
    aic32_configure(audio_cfg);

    davinci_aic32_initialize(dummy);
/*     davinci_set_samplerate(AUDIO_RATE_DEFAULT); */
    return 0;
}

codec_fops_t aic32_fops = {
    .configure = aic32_config,
    .ioctl = davinci_aic32_ioctl,
};

/* File Op structure for mixer */
struct file_operations davinci_mixer_fops = {
	.open = mixer_open,
	.release = mixer_release,
	.ioctl = mixer_ioctl,
	.owner = THIS_MODULE
};

extern int tlv320aic33_write_value(u8 reg, u16 value);
extern int tlv320aic33_read_value(u8 address, u8 * regValue);

/* TLV320AIC32 read */
static __inline__ void audio_aic32_read(u8 address, u8 * regValue)
{
    FN_IN;
    tlv320aic33_read_value(address, regValue);
    FN_OUT;
}

/* TLV320AIC32 write */
static __inline__ void audio_aic32_write(u8 address, u16 data)
{
    FN_IN;
    if (tlv320aic33_write_value(address, data) < 0)
	printk(KERN_INFO "aic32 write failed for reg = %d\n", address);
    FN_OUT;
}

static int aic32_update(int flag, int val)
{
	u16 volume;
	s16 left_gain, left_val, right_gain, right_val;

	FN_IN;

	switch (flag) {
	case SET_VOLUME:
		/* Ignore separate left/right channel for now, even the codec
		   does support it. */

		val &= 0xff;

		if (val < 0 || val > 100) {
			DPRINTK("Trying a bad volume value(%d)!\n", val);
			return -EPERM;
		}
		/* Convert 0 -> 100 volume to 0x77(LHV_MIN) -> 0x00(LHV_MAX) */
		volume =
		    ((val * OUTPUT_VOLUME_RANGE) / 100) + OUTPUT_VOLUME_MIN;

		aic32_local.volume_reg = OUTPUT_VOLUME_MAX - volume;

		if (aic32_local.nochan == STEREO) {
			audio_aic32_write(REGISTER_ADDR47, LOPM_ON |
					  aic32_local.volume_reg);
			audio_aic32_write(REGISTER_ADDR64, LOPM_ON |
					  aic32_local.volume_reg);
			audio_aic32_write(REGISTER_ADDR82, LOPM_ON |
					  aic32_local.volume_reg);
			audio_aic32_write(REGISTER_ADDR92, LOPM_ON |
					  aic32_local.volume_reg);
		} else if (aic32_local.nochan == MONO) {
#ifdef CONFIG_MONOSTEREO_DIFFJACK
			/* DACL1 to MONO_LOP/M routing and volume control */
			audio_aic32_write(REGISTER_ADDR75, LOPM_ON |
					  aic32_local.volume_reg);
			/* DACR1 to MONO_LOP/M routing and volume control */
			audio_aic32_write(REGISTER_ADDR78, LOPM_ON |
					  aic32_local.volume_reg);
#else
			audio_aic32_write(REGISTER_ADDR47, LOPM_ON |
					  aic32_local.volume_reg);
			audio_aic32_write(REGISTER_ADDR64, LOPM_ON |
					  aic32_local.volume_reg);
			audio_aic32_write(REGISTER_ADDR82, LOPM_ON |
					  aic32_local.volume_reg);
			audio_aic32_write(REGISTER_ADDR92, LOPM_ON |
					  aic32_local.volume_reg);
#endif
		}

		break;

	case SET_LINE:
	case SET_MIC:
		/* Ignore separate left/right channel for now even the codec
		   does support it. */

		val &= 0xff;

		if (val < 0 || val > 100) {
			DPRINTK("Trying a bad volume value(%d)!\n", val);
			return -EPERM;
		}

		volume = ((val * INPUT_VOLUME_RANGE) / 100) + INPUT_VOLUME_MIN;

		aic32_local.input_volume_reg = volume;

		audio_aic32_write(REGISTER_ADDR15,
				  aic32_local.input_volume_reg);
		audio_aic32_write(REGISTER_ADDR16,
				  aic32_local.input_volume_reg);
		break;

	case SET_RECSRC:
		/* Ignore separate left/right channel for now, even the codec
		   does support it. */

		val &= 0xff;

		if (hweight32(val) > 1)
			val &= ~aic32_local.recsrc;

		if (val == SOUND_MASK_MIC) {
			/* enable the mic input*/
			DPRINTK("Enabling mic\n");
			audio_aic32_write(REGISTER_ADDR17, 0x0);
			audio_aic32_write(REGISTER_ADDR18, 0x0);

			/* enable ADC's and disable the line input*/
			audio_aic32_write(REGISTER_ADDR19, 0x7C);
			audio_aic32_write(REGISTER_ADDR22, 0x7C);

		} else if (val == SOUND_MASK_LINE) {
			/* enable ADC's, enable line iput */
			DPRINTK(" Enabling line in\n");
			audio_aic32_write(REGISTER_ADDR19, 0x4);
			audio_aic32_write(REGISTER_ADDR22, 0x4);

			/* disable the mic input */
			audio_aic32_write(REGISTER_ADDR17, 0xff);
			audio_aic32_write(REGISTER_ADDR18, 0xff);
		} else {
			/* do nothing */
		}
		aic32_local.recsrc = val;
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
		audio_aic32_write(REGISTER_ADDR26, 0x80);
		audio_aic32_write(REGISTER_ADDR27, left_gain << 1);
		audio_aic32_write(REGISTER_ADDR28, 0x0);

		/* Right AGC control */
		audio_aic32_write(REGISTER_ADDR29, 0x80);
		audio_aic32_write(REGISTER_ADDR30, right_gain << 1);
		audio_aic32_write(REGISTER_ADDR31, 0x0);

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
		audio_aic32_write(REGISTER_ADDR43, left_gain);
		audio_aic32_write(REGISTER_ADDR44, right_gain);
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
			audio_aic32_write(REGISTER_ADDR25, 0x00);

		else if (val == 1)
			audio_aic32_write(REGISTER_ADDR25, MICBIAS_OUTPUT_2_0V);

		else if (val == 2)
			audio_aic32_write(REGISTER_ADDR25, MICBIAS_OUTPUT_2_5V);

		else if (val == 3)
			audio_aic32_write(REGISTER_ADDR25, MICBIAS_OUTPUT_AVDD);

		break;

	case SET_BASS:
		break;

	case SET_TREBLE:
		break;
	}
	FN_OUT;

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

static int mixer_ioctl(struct inode *inode, struct file *file, uint cmd, ulong
		       arg)
{
	int val;
	int ret = 0;
	int nr = _IOC_NR(cmd);

	/*
	 * We only accept mixer (type 'M') ioctls.
	 */
	FN_IN;

	if (_IOC_TYPE(cmd) != 'M')
		return -EINVAL;

	DPRINTK(" 0x%08x\n", cmd);

	if (cmd == SOUND_MIXER_INFO) {
		struct mixer_info mi;

		strncpy(mi.id, "AIC32", sizeof(mi.id));
		strncpy(mi.name, "TI AIC32", sizeof(mi.name));
		mi.modify_counter = aic32_local.mod_cnt;

		return copy_to_user((void *)arg, &mi, sizeof(mi));
	}

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		ret = get_user(val, (int *)arg);
		if (ret)
			goto out;

		switch (nr) {
		case SOUND_MIXER_VOLUME:
			aic32_local.mod_cnt++;
			ret = aic32_update(SET_VOLUME, val);
			if (!ret)
				aic32_local.volume = val;
			break;

		case SOUND_MIXER_LINE:
			aic32_local.mod_cnt++;
			ret = aic32_update(SET_LINE, val);
			if (!ret)
				aic32_local.line = val;
			break;

		case SOUND_MIXER_MIC:
			aic32_local.mod_cnt++;
			ret = aic32_update(SET_MIC, val);
			if (!ret)
				aic32_local.mic = val;
			break;

		case SOUND_MIXER_RECSRC:
			if ((val & SOUND_MASK_LINE) ||
			    (val & SOUND_MASK_MIC)) {
				if (aic32_local.recsrc != val) {
					aic32_local.mod_cnt++;
					aic32_update(SET_RECSRC, val);
				}
			} else {
				ret = -EINVAL;
			}
			break;

		case SOUND_MIXER_BASS:
			aic32_local.mod_cnt++;
			ret = aic32_update(SET_BASS, val);
			if (!ret)
				aic32_local.bass = val;
			break;

		case SOUND_MIXER_TREBLE:
			aic32_local.mod_cnt++;
			ret = aic32_update(SET_TREBLE, val);
			if (!ret)
				aic32_local.treble = val;
			break;

		case SOUND_MIXER_IGAIN:
			aic32_local.mod_cnt++;
			ret = aic32_update(SET_IGAIN, val);
			if (!ret)
				aic32_local.igain = val;
			break;

		case SOUND_MIXER_OGAIN:
			aic32_local.mod_cnt++;
			ret = aic32_update(SET_OGAIN, val);
			if (!ret)
				aic32_local.ogain = val;
			break;

		case SOUND_MIXER_MICBIAS:
			aic32_local.mod_cnt++;
			ret = aic32_update(SET_MICBIAS, val);
			if (!ret)
				aic32_local.micbias = val;
			break;

		default:
			ret = -EINVAL;
		}
	}

	if (ret == 0 && _IOC_DIR(cmd) & _IOC_READ) {
		ret = 0;

		switch (nr) {
		case SOUND_MIXER_VOLUME:
			val = aic32_local.volume;
			break;
		case SOUND_MIXER_LINE:
			val = aic32_local.line;
			break;
		case SOUND_MIXER_MIC:
			val = aic32_local.mic;
			break;
		case SOUND_MIXER_RECSRC:
			val = aic32_local.recsrc;
			break;
		case SOUND_MIXER_RECMASK:
			val = REC_MASK;
			break;
		case SOUND_MIXER_IGAIN:
			val = aic32_local.igain;
			break;
		case SOUND_MIXER_OGAIN:
			val = aic32_local.ogain;
			break;
		case SOUND_MIXER_DEVMASK:
			val = DEV_MASK;
			break;
		case SOUND_MIXER_BASS:
			val = aic32_local.bass;
			break;
		case SOUND_MIXER_TREBLE:
			val = aic32_local.treble;
			break;
		case SOUND_MIXER_CAPS:
			val = 0;
			break;
		case SOUND_MIXER_STEREODEVS:
			val = SOUND_MASK_VOLUME;
			break;
		case SOUND_MIXER_MICBIAS:
			val = aic32_local.micbias;
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
	FN_OUT;
	return ret;
}

int davinci_set_samplerate(long sample_rate)
{
	u8 count = 0;

	FN_IN;

	/* wait for any frame to complete */
	udelay(125);

	/* Search for the right sample rate */
	while ((reg_info[count].sample_rate != sample_rate) &&
	       (count < NUMBER_SAMPLE_RATES_SUPPORTED)) {

		    /* Issue an info to the user if the requested sample rate is
		       not exactly equal to whats supported, but quite close.*/
	    if (abs(sample_rate-reg_info[count].sample_rate) <= 5) {
		    printk(KERN_INFO "Nearest supported sampling rate is %d. \
					You might consider using this.\n",
		       reg_info[count].sample_rate);
	    }

	    count++;
	}

	if (count == NUMBER_SAMPLE_RATES_SUPPORTED) {
		DPRINTK("Invalid Sample Rate %d requested\n", (int)sample_rate);
		return -EPERM;
	}

	/*   CODEC DATAPATH SETUP  */

	/* Fsref to 48kHz, dual rate mode upto 96kHz */
	if (reg_info[count].Fsref == 96000)
		audio_aic32_write(REGISTER_ADDR7,
				  FS_REF_DEFAULT_48 | ADC_DUAL_RATE_MODE |
				  DAC_DUAL_RATE_MODE | LDAC_LCHAN | RDAC_RCHAN);

	/* Fsref to 44.1kHz, dual rate mode upto 88.2kHz */
	else if (reg_info[count].Fsref == 88200)
		audio_aic32_write(REGISTER_ADDR7,
				  FS_REF_44_1 | ADC_DUAL_RATE_MODE |
				  DAC_DUAL_RATE_MODE | LDAC_LCHAN | RDAC_RCHAN);

	/* Fsref to 48kHz */
	else if (reg_info[count].Fsref == 48000)
		audio_aic32_write(REGISTER_ADDR7,
				  FS_REF_DEFAULT_48 | LDAC_LCHAN | RDAC_RCHAN);

	/* Fsref to 44.1kHz */
	else if (reg_info[count].Fsref == 44100)
		audio_aic32_write(REGISTER_ADDR7, FS_REF_44_1 | LDAC_LCHAN |
				  RDAC_RCHAN);

	/* Codec sample rate select */
	audio_aic32_write(REGISTER_ADDR2, reg_info[count].data);

	/* If PLL is to be used for generation of Fsref
	   Generate the Fsref using the PLL */

	if ((reg_info[count].Fsref == 96000) ||
	    (reg_info[count].Fsref == 48000)) {
		    /* For MCLK = 27 MHz and to get Fsref = 48kHz
		       Fsref = (MCLK * k * R)/(2048 * p);
		       Select P = 2, R= 1, K = 7.2817, which results in J = 7, D
		       = 2817 */

		    /*Enable the PLL | Q-value | P-value */
	    audio_aic32_write(REGISTER_ADDR3, PLL_ENABLE | 0x10 | 0x02);
	    audio_aic32_write(REGISTER_ADDR4, 0x1C);	/* J-value */
	    audio_aic32_write(REGISTER_ADDR5, 0x2C);	/* D-value 8-MSB's */
	    audio_aic32_write(REGISTER_ADDR6, 0x01);	/* D-value 6-LSB's */
	} else if ((reg_info[count].Fsref == 88200)
		   || (reg_info[count].Fsref == 44100)) {

		    /* MCLK = 27 MHz and to get Fsref = 44.1kHz
		       Fsref = (MCLK * k * R)/(2048 * p);
		       Select P = 2, R =1, K = 6.6901, which results in J = 6, D
		       = 6901 */

		    /*Enable the PLL | Q-value | P-value */
	    audio_aic32_write(REGISTER_ADDR3, PLL_ENABLE | 0x10 | 0x02);
	    audio_aic32_write(REGISTER_ADDR4, 0x18);	/* J-value */
	    audio_aic32_write(REGISTER_ADDR5, 0x6B);	/* D-value 8-MSB's */
	    audio_aic32_write(REGISTER_ADDR6, 0x35);	/* D-value 6-LSB's */
	}

	audio_samplerate = sample_rate;

	FN_OUT;
	return 0;
}

static void davinci_set_mono_stereo(int mode)
{
    FN_IN;
	if (mode == MONO) {

#ifdef CONFIG_MONOSTEREO_DIFFJACK
		/* MONO_LOP/M Output level control register */
		audio_aic32_write(REGISTER_ADDR79, 0x99);
#else
		/* Driver power ON pop control */
		audio_aic32_write(REGISTER_ADDR42, 0x6C);

		/* HPLOUT/HPROUT output level control */
		audio_aic32_write(REGISTER_ADDR51, 0x99);
		audio_aic32_write(REGISTER_ADDR65, 0x99);

		/* LEFT_LOP/M, RIGHT_LOP/M output level control */
		audio_aic32_write(REGISTER_ADDR86, 0x99);
		audio_aic32_write(REGISTER_ADDR93, 0x99);
#endif
		/* Left DAC power up, Right DAC power down */
		audio_aic32_write(REGISTER_ADDR37, 0xa0);
	} else if (mode == STEREO) {
		/* Driver power ON pop control */
		audio_aic32_write(REGISTER_ADDR42, 0x6C);

		/* HPLOUT/HPROUT output level control */
		audio_aic32_write(REGISTER_ADDR51, 0x99);
		audio_aic32_write(REGISTER_ADDR65, 0x99);

		/* LEFT_LOP/M, RIGHT_LOP/M output level control */
		audio_aic32_write(REGISTER_ADDR86, 0x99);
		audio_aic32_write(REGISTER_ADDR93, 0x99);

		/* Left/Right DAC power up */
		audio_aic32_write(REGISTER_ADDR37, 0xe0);
	} else
		DPRINTK(" REQUEST FOR INVALID MODE\n");

	FN_OUT;
}

/* Select the desired mode. Define any one of the following.*/
#define AIC32_I2S_MODE              1
/* #define AIC32_RIGHT_JUSTIFIED_MODE  1 */
/* #define AIC32_LEFT_JUSTIFIED_MODE   1 */

static inline void aic32_configure(audio_config_t *audio_cfg)
{
    FN_IN;

    DPRINTK(" CONFIGURING AIC32\n");

	    /* Page select register */
    audio_aic32_write(REGISTER_ADDR0, 0x0);

    /* audio_aic32_write(REGISTER_ADDR38, 0x10); */

    davinci_set_mono_stereo(aic32_local.nochan);

    if (audio_cfg->mode == CODEC_IS_MASTER) {
		/* Enable bit and word clock as Master mode, 3-d disabled */
	audio_aic32_write(REGISTER_ADDR8, 0xc0 /*0xc4 */ );
		/* Enabling 3d generates shrill noise */
    } else {
		/* Enable bit and word clock as slave mode, 3-d disabled */
	audio_aic32_write(REGISTER_ADDR8, 0x00);
	audio_aic32_write(REGISTER_ADDR102, 0xA2);
    }

    switch (audio_cfg->tdm_slots) {
    default:
    case 0:
	    audio_aic32_write(REGISTER_ADDR9, 0x40);	/* DSP Mode */
	    break;

    case 2:
#ifdef AIC32_I2S_MODE
	    audio_aic32_write(REGISTER_ADDR9, 0x00); /* I2S Mode */
#endif /* AIC32_I2S_MODE */
#ifdef AIC32_RIGHT_JUSTIFIED_MODE
	    /* Right Justified Mode  */
	    audio_aic32_write(REGISTER_ADDR9, 0x80);
#endif /* AIC32_RIGHT_JUSTIFIED_MODE */
#ifdef AIC32_LEFT_JUSTIFIED_MODE
	    /* Left Justified Mode */
	    audio_aic32_write(REGISTER_ADDR9, 0xC0);
#endif /* AIC32_LEFT_JUSTIFIED_MODE */
	    break;
    }

    aic32_update(SET_LINE, aic32_local.line);
    aic32_update(SET_VOLUME, aic32_local.volume);
    aic32_update(SET_RECSRC, aic32_local.recsrc);
    aic32_update(SET_IGAIN, aic32_local.igain);
    aic32_update(SET_OGAIN, aic32_local.ogain);
    aic32_update(SET_MICBIAS, aic32_local.micbias);
    FN_OUT;
}

static void davinci_aic32_initialize(void *dummy)
{
	FN_IN;

	DPRINTK("entry\n");

	/* initialize with default sample rate */
	audio_samplerate = AUDIO_RATE_DEFAULT;

	/* set initial (default) sample rate */
	davinci_set_samplerate(audio_samplerate);

	FN_OUT;

	DPRINTK("exit\n");
}

int davinci_aic32_ioctl(struct inode *inode, struct file *file, uint cmd, ulong
			arg)
{
	long val;
	int ret = 0;

	FN_IN;

	/*
	 * These are platform dependent ioctls which are not handled by the
	 * generic davinci-audio module.
	 */
	switch (cmd) {
	case SNDCTL_DSP_STEREO:
		ret = get_user(val, (int *)arg);
		if (ret)
			return ret;
		/* the Davinci supports AIC32 as stereo, mono on stereo jack */
		ret = (val == 0) ? -EINVAL : 1;
		return put_user(ret, (int *)arg);

	case SNDCTL_DSP_CHANNELS:
		ret = get_user(val, (long *)arg);
		if (ret) {
			DPRINTK("get_user failed\n");
			break;
		}
		if (val == STEREO) {
			DPRINTK("Driver support for AIC32 as stereo\n");
			aic32_local.nochan = STEREO;
			davinci_set_mono_stereo(aic32_local.nochan);
		} else if (val == MONO) {
			DPRINTK("Driver support for AIC32 as mono\n");
			aic32_local.nochan = MONO;
			davinci_set_mono_stereo(aic32_local.nochan);
		} else {
			DPRINTK
			    ("Driver support for AIC32 as stereo/mono mode\n");
			return -EPERM;
		}

	case SOUND_PCM_READ_CHANNELS:
		/* the Davinci supports AIC32 as stereo, mono on stereo jack */
		if (aic32_local.nochan == MONO)
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
		break;

	case SOUND_PCM_READ_BITS:
	case SNDCTL_DSP_GETFMTS:
		/* we can do 16-bit only */
		return put_user(AFMT_S16_LE, (long *)arg);

	default:
		/* Maybe this is meant for the mixer (As per OSS Docs) */
		return mixer_ioctl(inode, file, cmd, arg);
	}
	FN_OUT;

	return ret;
}
EXPORT_SYMBOL(davinci_aic32_ioctl);
