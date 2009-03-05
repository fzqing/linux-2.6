/*
 * sound/pci/vr5701/nec_vr5701_sg2.h
 *
 * An ALSA sound driver for 
 * NEC Electronics Corporation VR5701 SolutionGearII.
 * It works with AC97 codec.
 *
 * Author: Sergey Podstavin <spodstavin@ru.mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <sound/driver.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <sound/pcm_params.h>

#define         vr5701_INT_CLR         	0x0
#define         vr5701_INT_STATUS	0x0
#define         vr5701_CODEC_WR        	0x4
#define         vr5701_CODEC_RD        	0x8
#define         vr5701_CTRL            	0x18
#define         vr5701_ACLINK_CTRL     	0x1c
#define         vr5701_INT_MASK        	0x24

#define		vr5701_DAC1_CTRL	0x30
#define		vr5701_DAC1L		0x34
#define		vr5701_DAC1_BADDR	0x38
#define		vr5701_DAC2_CTRL	0x3c
#define		vr5701_DAC2L		0x40
#define		vr5701_DAC2_BADDR	0x44
#define		vr5701_DAC3_CTRL	0x48
#define		vr5701_DAC3L		0x4c
#define		vr5701_DAC3_BADDR	0x50

#define		vr5701_ADC1_CTRL	0x54
#define		vr5701_ADC1L		0x58
#define		vr5701_ADC1_BADDR	0x5c
#define		vr5701_ADC2_CTRL	0x60
#define		vr5701_ADC2L		0x64
#define		vr5701_ADC2_BADDR	0x68
#define		vr5701_ADC3_CTRL	0x6c
#define		vr5701_ADC3L		0x70
#define		vr5701_ADC3_BADDR	0x74

#define		vr5701_CODEC_WR_RWC	(1 << 23)

#define		vr5701_CODEC_RD_RRDYA	(1 << 31)
#define		vr5701_CODEC_RD_RRDYD	(1 << 30)

#define		vr5701_ACLINK_CTRL_RST_ON	(1 << 15)
#define		vr5701_ACLINK_CTRL_RST_TIME	0x7f
#define		vr5701_ACLINK_CTRL_SYNC_ON	(1 << 30)
#define		vr5701_ACLINK_CTRL_CK_STOP_ON	(1 << 31)

#define		vr5701_CTRL_DAC2ENB		(1 << 15)
#define		vr5701_CTRL_ADC2ENB		(1 << 14)
#define		vr5701_CTRL_DAC1ENB		(1 << 13)
#define		vr5701_CTRL_ADC1ENB		(1 << 12)

#define		vr5701_INT_MASK_NMASK		(1 << 31)
#define		vr5701_INT_MASK_DAC1END		(1 << 5)
#define		vr5701_INT_MASK_DAC2END		(1 << 4)
#define		vr5701_INT_MASK_DAC3END		(1 << 3)
#define		vr5701_INT_MASK_ADC1END		(1 << 2)
#define		vr5701_INT_MASK_ADC2END		(1 << 1)
#define		vr5701_INT_MASK_ADC3END		(1 << 0)

#define		vr5701_DMA_ACTIVATION		(1 << 31)
#define		vr5701_DMA_WIP			(1 << 30)

#define vr5701_AC97_MODULE_NAME "NEC_vr5701_audio"
#define PLAYBACK 0
#define CAPTURE  1

/* module parameters */
static int index[SNDRV_CARDS]  = SNDRV_DEFAULT_IDX;
static char *id[SNDRV_CARDS]   = SNDRV_DEFAULT_STR;
static int enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;

/* definition of the chip-specific record */
typedef struct snd_vr5701 vr5701_t;

struct snd_vr5701 {
        snd_card_t *card;
        struct pci_dev *pci;
	ac97_t *ac97;      
        unsigned long port; 
        int irq;  
	snd_pcm_t	*pcm;
	snd_pcm_substream_t * substream[2]; 
	unsigned next_playback;
	unsigned next_capture;
};

static unsigned int rates[] = {
	8000,  11025, 16000, 22050,
	32000, 44100, 48000,
};

static snd_pcm_hw_constraint_list_t hw_constraints_rates = {
	.count	= ARRAY_SIZE(rates),
	.list	= rates,
	.mask	= 0,
};

/* hardware definition */
static snd_pcm_hardware_t snd_vr5701_playback_hw = {
	.info = (  SNDRV_PCM_INFO_MMAP |
                   SNDRV_PCM_INFO_NONINTERLEAVED |
                   SNDRV_PCM_INFO_BLOCK_TRANSFER |
                   SNDRV_PCM_INFO_MMAP_VALID),
	.formats =          SNDRV_PCM_FMTBIT_S16_LE,
	.rates =            SNDRV_PCM_RATE_8000_48000,
	.rate_min =         8000,
	.rate_max =         48000,
	.channels_min =     1,
	.channels_max =     2,
	.buffer_bytes_max = (128*1024),
	.period_bytes_min = 32,
	.period_bytes_max = (128*1024),
	.periods_min =      1,
	.periods_max =      1024,
};

/* hardware definition */
static snd_pcm_hardware_t snd_vr5701_capture_hw = {
	.info = (SNDRV_PCM_INFO_MMAP |
                   SNDRV_PCM_INFO_INTERLEAVED |
                   SNDRV_PCM_INFO_BLOCK_TRANSFER |
                   SNDRV_PCM_INFO_MMAP_VALID),
	.formats =          SNDRV_PCM_FMTBIT_S16_LE,
	.rates =            SNDRV_PCM_RATE_8000_48000,
	.rate_min =         8000,
	.rate_max =         48000,
	.channels_min =     1,
	.channels_max =     1,
	.buffer_bytes_max = (128*1024),
	.period_bytes_min = 32,
	.period_bytes_max = (128*1024),
	.periods_min =      1,
	.periods_max =      1024,
};
