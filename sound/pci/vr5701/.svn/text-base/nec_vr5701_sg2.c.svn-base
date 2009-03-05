/*
 * sound/pci/vr5701/nec_vr5701_sg2.c
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

#include "nec_vr5701_sg2.h"

static unsigned short snd_vr5701_ac97_read(ac97_t *ac97,
                                             unsigned short reg)
{
	vr5701_t *chip = ac97->private_data;
	u32 the_register_value;
	
	/* wait until we can access codec registers */
	while (inl(chip->port + vr5701_CODEC_WR) & 0x80000000) ;

	/* write the address and "read" command to codec */
	reg = reg & 0x7f;
	outl((reg << 16) | vr5701_CODEC_WR_RWC, chip->port + vr5701_CODEC_WR);

	/* get the return result */
	udelay(100);		/* workaround hardware bug */
	while ((the_register_value = inl(chip->port + vr5701_CODEC_RD)) &
		(vr5701_CODEC_RD_RRDYA | vr5701_CODEC_RD_RRDYD)) {
		/* we get either addr or data, or both */
		if (the_register_value & vr5701_CODEC_RD_RRDYD) {
			break;
		}
	}

	return the_register_value & 0xffff;
}

static void snd_vr5701_ac97_write(ac97_t *ac97,
                                   unsigned short reg, unsigned short val)
{
	vr5701_t *chip = ac97->private_data;

	/* wait until we can access codec registers */
	while (inl(chip->port + vr5701_CODEC_WR) & 0x80000000) ;

	/* write the address and value to codec */
	outl((reg << 16) | val, chip->port + vr5701_CODEC_WR);
}

static void snd_vr5701_ac97_reset(ac97_t *ac97)
{
	vr5701_t *chip = ac97->private_data;
	outl(vr5701_ACLINK_CTRL_RST_ON | vr5701_ACLINK_CTRL_RST_TIME,
		chip->port + vr5701_ACLINK_CTRL);
	while (inl(chip->port + vr5701_ACLINK_CTRL) & vr5701_ACLINK_CTRL_RST_ON) ;
	return;
}

static void snd_vr5701_ac97_wait(ac97_t *ac97)
{
	vr5701_t *chip = ac97->private_data;

	/* wait until we can access codec registers */
	while (inl(chip->port + vr5701_CODEC_WR) & 0x80000000) ;
}

static int snd_vr5701_ac97(vr5701_t *chip)
{
	ac97_bus_t *bus;
	ac97_template_t ac97;
	int err;
	static ac97_bus_ops_t ops = {
		.write = snd_vr5701_ac97_write,
		.read = snd_vr5701_ac97_read,
		.reset = snd_vr5701_ac97_reset,
		.wait = snd_vr5701_ac97_wait,
	};

	if ((err = snd_ac97_bus(chip->card, 0, &ops, NULL, &bus)) < 0)
		return err;
	memset(&ac97, 0, sizeof(ac97));
	ac97.private_data = chip;
	return snd_ac97_mixer(bus, &ac97, &chip->ac97);
}



/* open callback */
static int snd_vr5701_playback_open(snd_pcm_substream_t *substream)
{
	int err;
	vr5701_t *chip = snd_pcm_substream_chip(substream);
	chip->substream[PLAYBACK] = substream;
	substream->runtime->hw = snd_vr5701_playback_hw;
	chip->next_playback = 0;
	err = snd_pcm_hw_constraint_step(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 0x10);
	if( err < 0 )
		return err;
	err = snd_pcm_hw_constraint_step(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 0x40);
	if( err < 0 )
		return err;
	return (snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE, &hw_constraints_rates) < 0); 
}


/* open callback */
static int snd_vr5701_capture_open(snd_pcm_substream_t *substream)
{
	int err;
	vr5701_t *chip = snd_pcm_substream_chip(substream);
	chip->substream[CAPTURE] = substream;
	substream->runtime->hw = snd_vr5701_capture_hw;
	chip->next_capture = 0;
	err = snd_pcm_hw_constraint_step(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 0x10);
	if( err < 0 )
		return err;
	err = snd_pcm_hw_constraint_step(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 0x40);
	if( err < 0 )
		return err;
	return (snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE, &hw_constraints_rates) < 0);
}

/* hw_params callback */
static int snd_vr5701_pcm_hw_params(snd_pcm_substream_t *substream,
                               snd_pcm_hw_params_t * hw_params)
{
	return snd_pcm_lib_malloc_pages(substream,
		params_buffer_bytes(hw_params));
}

/* hw_free callback */
static int snd_vr5701_pcm_hw_free(snd_pcm_substream_t *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

/* playback prepare callback */
static int snd_vr5701_pcm_prepare_playback(snd_pcm_substream_t *substream)
{
	vr5701_t *chip = snd_pcm_substream_chip(substream);
	snd_pcm_runtime_t *runtime = substream->runtime;
	snd_ac97_set_rate(chip->ac97, AC97_PCM_FRONT_DAC_RATE, runtime->rate);
	return 0;
}

/* capture prepare callback */
static int snd_vr5701_pcm_prepare_capture(snd_pcm_substream_t *substream)
{
	vr5701_t *chip = snd_pcm_substream_chip(substream);
	snd_pcm_runtime_t *runtime = substream->runtime;
	snd_ac97_set_rate(chip->ac97, AC97_PCM_LR_ADC_RATE, runtime->rate);
	return 0;
}

static void vr5701_dma_stop_playback(snd_pcm_substream_t *substream)
{
	u32 temp;
	vr5701_t *chip = snd_pcm_substream_chip(substream);
	chip->next_playback = 0;

	/* deactivate the dma */
	outl(0, chip->port + vr5701_DAC1_CTRL);
	outl(0, chip->port + vr5701_DAC2_CTRL);

	/* wait for DMA completely stop */
	while (inl(chip->port + vr5701_DAC1_CTRL) & vr5701_DMA_WIP);
	while (inl(chip->port + vr5701_DAC2_CTRL) & vr5701_DMA_WIP);

	/* disable dac slots in aclink */
	temp = inl(chip->port + vr5701_CTRL);
	temp &= ~(vr5701_CTRL_DAC1ENB | vr5701_CTRL_DAC2ENB);
	outl(temp, chip->port + vr5701_CTRL);

	/* disable interrupts */
	temp = inl(chip->port + vr5701_INT_MASK);
	temp &= ~(vr5701_INT_MASK_DAC1END | vr5701_INT_MASK_DAC2END);
	outl(temp, chip->port + vr5701_INT_MASK);

	/* clear pending ones */
	outl(vr5701_INT_MASK_DAC1END | vr5701_INT_MASK_DAC2END,
		chip->port + vr5701_INT_CLR);

}

/* close callback */
static int snd_vr5701_playback_close(snd_pcm_substream_t *substream)
{
	vr5701_dma_stop_playback(substream);
	return 0;
}

static void vr5701_dma_stop_capture(snd_pcm_substream_t *substream)
{
	u32 temp;
	vr5701_t *chip = snd_pcm_substream_chip(substream);
	chip->next_capture = 0;

	/* deactivate the dma */
	outl(0, chip->port + vr5701_ADC1_CTRL);
	outl(0, chip->port + vr5701_ADC2_CTRL);

	/* disable dac slots in aclink */
	temp = inl(chip->port + vr5701_CTRL);
	temp &= ~(vr5701_CTRL_ADC1ENB | vr5701_CTRL_ADC2ENB);
	outl(temp, chip->port + vr5701_CTRL);

	/* disable interrupts */
	temp = inl(chip->port + vr5701_INT_MASK);
	temp &= ~(vr5701_INT_MASK_ADC1END | vr5701_INT_MASK_ADC2END);
	outl(temp, chip->port + vr5701_INT_MASK);

	/* clear pending ones */
	outl(vr5701_INT_MASK_ADC1END | vr5701_INT_MASK_ADC2END,
		chip->port + vr5701_INT_CLR);
}

/* close callback */
static int snd_vr5701_capture_close(snd_pcm_substream_t *substream)
{
	vr5701_dma_stop_capture(substream);
	return 0;
}


static void vr5701_dma_start_playback(snd_pcm_substream_t *substream)
{
	u32 temp;
	vr5701_t *chip = snd_pcm_substream_chip(substream);
	unsigned long period_size;

	/* clear pending fales interrupts */
	outl(vr5701_INT_MASK_DAC1END | vr5701_INT_MASK_DAC2END,
		chip->port + vr5701_INT_CLR);

	/* enable interrupts */
	temp = inl(chip->port + vr5701_INT_MASK);
	temp |= vr5701_INT_MASK_DAC1END | vr5701_INT_MASK_DAC2END;
	outl(temp, chip->port + vr5701_INT_MASK);

	/* setup dma base addr */
	if (substream->runtime->channels == 1) {
		outl(substream->runtime->dma_addr + chip->next_playback, 
						chip->port + vr5701_DAC1_BADDR);
		outl(substream->runtime->dma_addr + chip->next_playback, 
						chip->port + vr5701_DAC2_BADDR);
	} else {
		outl(substream->runtime->dma_addr + chip->next_playback, 
						chip->port + vr5701_DAC1_BADDR);
		outl(substream->runtime->dma_addr + substream->runtime->dma_bytes/2 
				+ chip->next_playback, chip->port + vr5701_DAC2_BADDR);
	}
	/* set dma length, in the unit of 0x10 bytes */
	period_size = (frames_to_bytes(substream->runtime, 
			substream->runtime->period_size)) >> 4;
	outl(period_size, chip->port + vr5701_DAC1L);
	outl(period_size, chip->port + vr5701_DAC2L);

	/* activate dma */
	outl(vr5701_DMA_ACTIVATION, chip->port + vr5701_DAC1_CTRL);
	outl(vr5701_DMA_ACTIVATION, chip->port + vr5701_DAC2_CTRL);

	/* enable dac slots - we should hear the music now! */
	temp = inl(chip->port + vr5701_CTRL);
	temp |= (vr5701_CTRL_DAC1ENB | vr5701_CTRL_DAC2ENB);
	outl(temp, chip->port + vr5701_CTRL);

	/* it is time to setup next dma transfer */
	temp = chip->next_playback + frames_to_bytes(substream->runtime, 
			substream->runtime->period_size);
 
	if (substream->runtime->channels == 1) {
		if (temp >= substream->runtime->dma_bytes) {
			temp = 0;
		}
		outl(substream->runtime->dma_addr + temp, 
				chip->port + vr5701_DAC1_BADDR);
		outl(substream->runtime->dma_addr + temp, 
				chip->port + vr5701_DAC2_BADDR);
	} else {
		if (temp >= substream->runtime->dma_bytes/2) {
			temp = 0;
		}
		outl(substream->runtime->dma_addr + temp, 
				chip->port + vr5701_DAC1_BADDR);
		outl(substream->runtime->dma_addr + substream->runtime->dma_bytes/2
						+ temp, chip->port + vr5701_DAC2_BADDR);
	}
	return ;
}

static void vr5701_dma_start_capture(snd_pcm_substream_t *substream)
{
	u32 temp;
	vr5701_t *chip = snd_pcm_substream_chip(substream);
	unsigned long period_size;

	/* clear pending fales interrupts */
	outl(vr5701_INT_MASK_ADC1END | vr5701_INT_MASK_ADC2END,
		chip->port + vr5701_INT_CLR);

	/* enable interrupts */
	temp = inl(chip->port + vr5701_INT_MASK);
	temp |= vr5701_INT_MASK_ADC1END | vr5701_INT_MASK_ADC2END;
	outl(temp, chip->port + vr5701_INT_MASK);

	/* setup dma base addr */
	if (substream->runtime->channels == 1) {
		outl(substream->runtime->dma_addr + chip->next_capture, 
					chip->port + vr5701_ADC1_BADDR);
	} else {
		outl(substream->runtime->dma_addr+ chip->next_capture, 
					chip->port + vr5701_ADC1_BADDR);
		outl(substream->runtime->dma_addr + substream->runtime->dma_bytes/2 
				+ chip->next_capture, chip->port + vr5701_ADC2_BADDR);
	}
	/* set dma length, in the unit of 0x10 bytes */
	period_size = (frames_to_bytes(substream->runtime, 
			substream->runtime->period_size))  >> 4;
	if (substream->runtime->channels == 1) {
		outl(period_size, chip->port + vr5701_ADC1L);
	} else {
		outl(period_size, chip->port + vr5701_ADC1L);
		outl(period_size, chip->port + vr5701_ADC2L);
	}

	/* activate dma */
	if (substream->runtime->channels == 1) {
		outl(vr5701_DMA_ACTIVATION, chip->port + vr5701_ADC1_CTRL);
	} else {
		outl(vr5701_DMA_ACTIVATION, chip->port + vr5701_ADC1_CTRL);
		outl(vr5701_DMA_ACTIVATION, chip->port + vr5701_ADC2_CTRL);
	}

	/* enable adc slots */
	temp = inl(chip->port + vr5701_CTRL);
	temp |= (vr5701_CTRL_ADC1ENB | vr5701_CTRL_ADC2ENB);
	outl(temp, chip->port + vr5701_CTRL);

	/* it is time to setup next dma transfer */
	temp = chip->next_capture + frames_to_bytes(substream->runtime, 
			substream->runtime->period_size);
	if (substream->runtime->channels == 1) {
		if (temp >= substream->runtime->dma_bytes) {
			temp = 0;
		}
		outl(substream->runtime->dma_addr + temp, 
				chip->port + vr5701_ADC1_BADDR);
	} else {
		if (temp >= substream->runtime->dma_bytes/2) {
			temp = 0;
		}
		outl(substream->runtime->dma_addr + temp, 
				chip->port + vr5701_ADC1_BADDR);
		outl(substream->runtime->dma_addr + substream->runtime->dma_bytes/2
						+ temp,	chip->port + vr5701_ADC2_BADDR);
	}
	return ;
}

/* trigger callback */
static int snd_vr5701_pcm_trigger(snd_pcm_substream_t *substream,
                                    int cmd)
{
	int err = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (substream->stream) {
			vr5701_dma_start_capture(substream);
		}
		else {
			vr5701_dma_start_playback(substream);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		if (substream->stream)
			vr5701_dma_stop_capture(substream);
		else
			vr5701_dma_stop_playback(substream);
		break;
	default:
		err = -EINVAL;
		printk(KERN_WARNING
			"vr5701_ac97 : wrong trigger callback!!!\n");
		break;
	}
	return err;
}

/* playback pointer callback */
static snd_pcm_uframes_t
snd_vr5701_pcm_pointer_playback(snd_pcm_substream_t *substream)
{
	vr5701_t *chip = snd_pcm_substream_chip(substream);
	return 	 bytes_to_frames(substream->runtime, 
					chip->next_playback);	
}

/* capture pointer callback */
static snd_pcm_uframes_t
snd_vr5701_pcm_pointer_capture(snd_pcm_substream_t *substream)
{
	vr5701_t *chip = snd_pcm_substream_chip(substream);
	return 	 bytes_to_frames(substream->runtime, 
					chip->next_capture);	
}
/* operators */
static snd_pcm_ops_t snd_vr5701_playback_ops = {
	.open =        snd_vr5701_playback_open,
	.close =       snd_vr5701_playback_close,
	.ioctl =       snd_pcm_lib_ioctl,
	.hw_params =   snd_vr5701_pcm_hw_params,
	.hw_free =     snd_vr5701_pcm_hw_free,
	.prepare =     snd_vr5701_pcm_prepare_playback,
	.trigger =     snd_vr5701_pcm_trigger,
	.pointer =     snd_vr5701_pcm_pointer_playback,
};

/* operators */
static snd_pcm_ops_t snd_vr5701_capture_ops = {
	.open =        snd_vr5701_capture_open,
	.close =       snd_vr5701_capture_close,
	.ioctl =       snd_pcm_lib_ioctl,
	.hw_params =   snd_vr5701_pcm_hw_params,
	.hw_free =     snd_vr5701_pcm_hw_free,
	.prepare =     snd_vr5701_pcm_prepare_capture,
	.trigger =     snd_vr5701_pcm_trigger,
	.pointer =     snd_vr5701_pcm_pointer_capture,
};

/* create a pcm device */
static int __devinit snd_vr5701_new_pcm(vr5701_t *chip)
{
	snd_pcm_t *pcm;
	int err;
	if ((err = snd_pcm_new(chip->card, vr5701_AC97_MODULE_NAME, 0, 1, 1,
		&pcm)) < 0) 
			return err;
	pcm->private_data = chip;
	strcpy(pcm->name, vr5701_AC97_MODULE_NAME);
	chip->pcm = pcm;

	/* set operators */
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
		&snd_vr5701_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
		&snd_vr5701_capture_ops);
	
	/* pre-allocation of buffers */
	/* NOTE: this may fail */
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
		snd_dma_pci_data(chip->pci), 64*1024, 64*1024);
	return 0;
}
          
/* chip-specific destructor
 */
static int snd_vr5701_free(vr5701_t *chip)
{
	/* release the irq */
	if (chip->irq >= 0)
		free_irq(chip->irq, (void *)chip);
	
	/* release the i/o ports & memory */
	pci_release_regions(chip->pci);
	
	/* disable the PCI entry */
	pci_disable_device(chip->pci);
	
	/* release the data */
	kfree(chip);
	return 0;
}

/* component-destructor
 */
static int snd_vr5701_dev_free(snd_device_t *device)
{
	vr5701_t *chip = device->device_data;
	return snd_vr5701_free(chip);
}
static void vr5701_ac97_adc_interrupt(struct snd_vr5701 *chip)
{
	unsigned temp;

	/* set the base addr for next DMA transfer */
	temp = chip->next_capture + 2*(frames_to_bytes(chip->substream[CAPTURE]->runtime, 
			chip->substream[CAPTURE]->runtime->period_size));
	if (chip->substream[CAPTURE]->runtime->channels == 1) {
		if (temp >= chip->substream[CAPTURE]->runtime->dma_bytes) {
			temp -= chip->substream[CAPTURE]->runtime->dma_bytes;
		}
	} else {
		if (temp >= chip->substream[CAPTURE]->runtime->dma_bytes/2) {
			temp -= chip->substream[CAPTURE]->runtime->dma_bytes/2;
		}
	}

	if (chip->substream[CAPTURE]->runtime->channels == 1) {
		outl(chip->substream[CAPTURE]->runtime->dma_addr 
				+ temp, chip->port + vr5701_ADC1_BADDR);
	} else {
		outl(chip->substream[CAPTURE]->runtime->dma_addr 
				+ temp, chip->port + vr5701_ADC1_BADDR);
		outl(chip->substream[CAPTURE]->runtime->dma_addr 
		+ chip->substream[CAPTURE]->runtime->dma_bytes/2+ temp, 
						chip->port + vr5701_ADC2_BADDR);
	}

	/* adjust next pointer */
	chip->next_capture += frames_to_bytes(chip->substream[CAPTURE]->runtime, 
					chip->substream[CAPTURE]->runtime->period_size);
	if (chip->substream[CAPTURE]->runtime->channels == 1) {
		if (chip->next_capture >= chip->substream[CAPTURE]->runtime->dma_bytes){
			chip->next_capture = 0;
		}
	} else {
		if (chip->next_capture >= chip->substream[CAPTURE]->runtime->dma_bytes/2){
			chip->next_capture = 0;
		}
	}
}

static void vr5701_ac97_dac_interrupt(struct snd_vr5701 *chip)
{
	unsigned temp;

	/* let us set for next next DMA transfer */
	temp = chip->next_playback + 2*(frames_to_bytes(chip->substream[PLAYBACK]->runtime, 
			chip->substream[PLAYBACK]->runtime->period_size));
	if (chip->substream[PLAYBACK]->runtime->channels == 1) {
		if (temp >= chip->substream[PLAYBACK]->runtime->dma_bytes) {
			temp -= chip->substream[PLAYBACK]->runtime->dma_bytes;
		}
	} else {
		if (temp >= chip->substream[PLAYBACK]->runtime->dma_bytes/2) {
			temp -= chip->substream[PLAYBACK]->runtime->dma_bytes/2;
		}
	}

	if (chip->substream[PLAYBACK]->runtime->channels == 1) {
		outl(chip->substream[PLAYBACK]->runtime->dma_addr 
				+ temp, chip->port + vr5701_DAC1_BADDR);
		outl(chip->substream[PLAYBACK]->runtime->dma_addr 
				+ temp, chip->port + vr5701_DAC2_BADDR);
	} else {
		outl(chip->substream[PLAYBACK]->runtime->dma_addr 
				+ temp, chip->port + vr5701_DAC1_BADDR);
		outl(chip->substream[PLAYBACK]->runtime->dma_addr 
			+ chip->substream[PLAYBACK]->runtime->dma_bytes/2
				+ temp,	chip->port + vr5701_DAC2_BADDR);
	}

	/* adjust next pointer */
	chip->next_playback += frames_to_bytes(chip->substream[PLAYBACK]->runtime, 
					chip->substream[PLAYBACK]->runtime->period_size);
	if (chip->substream[PLAYBACK]->runtime->channels == 1) {
		if (chip->next_playback >= chip->substream[PLAYBACK]->runtime->dma_bytes){
			chip->next_playback = 0;
		}
	} else {
		if (chip->next_playback >= chip->substream[PLAYBACK]->runtime->dma_bytes/2){
			chip->next_playback = 0;
		}
	}
}

static irqreturn_t snd_vr5701_interrupt(int irq, void *dev_id,
                                          struct pt_regs *regs)
{
	vr5701_t *chip = dev_id;
	u32 irqStatus;
	u32 adcInterrupts, dacInterrupts;

	/* get irqStatus and clear the detected ones */
	irqStatus = inl(chip->port + vr5701_INT_STATUS);
	outl(irqStatus, chip->port + vr5701_INT_CLR);

	/* let us see what we get */
	dacInterrupts = vr5701_INT_MASK_DAC1END | vr5701_INT_MASK_DAC2END;
	adcInterrupts = vr5701_INT_MASK_ADC1END | vr5701_INT_MASK_ADC2END;
	if (irqStatus & dacInterrupts) {
		/* we should get both interrupts */
		if (irqStatus & vr5701_INT_MASK_DAC1END) {
			vr5701_ac97_dac_interrupt(chip);
		}
		if ((irqStatus & dacInterrupts) != dacInterrupts) {
			printk(KERN_WARNING
			       "vr5701_ac97 : playback interrupts not in sync!!!\n");
			vr5701_dma_stop_playback(chip->substream[PLAYBACK]);
			vr5701_dma_start_playback(chip->substream[PLAYBACK]);
		}
		snd_pcm_period_elapsed(chip->substream[PLAYBACK]);
	} else if (irqStatus & adcInterrupts) {
		/* we should get both interrupts, but just in stereo case */
		if (irqStatus & vr5701_INT_MASK_ADC1END) {
			vr5701_ac97_adc_interrupt(chip);
		}
		if (((irqStatus & adcInterrupts) != adcInterrupts)
			&&(chip->substream[CAPTURE]->runtime->channels == 2)) {
			printk(KERN_WARNING
			       "vr5701_ac97 : capture interrupts not in sync!!!\n");
			vr5701_dma_stop_capture(chip->substream[CAPTURE]);
			vr5701_dma_start_capture(chip->substream[CAPTURE]);
		}
		snd_pcm_period_elapsed(chip->substream[CAPTURE]);
	}
	
	return IRQ_HANDLED;
}

/* chip-specific constructor
 */
static int __devinit snd_vr5701_create(snd_card_t *card,
	struct pci_dev *pci, vr5701_t **rchip)
{
	vr5701_t *chip;
	int err;
	static snd_device_ops_t ops = {
		.dev_free = snd_vr5701_dev_free,
	};

	*rchip = NULL;

	/* initialize the PCI entry */
	if ((err = pci_enable_device(pci)) < 0)
		return err;
	pci_set_master(pci);

	/* check PCI availability (28bit DMA) */
	if (pci_set_dma_mask(pci, 0x0fffffff) < 0 ||
		pci_set_consistent_dma_mask(pci, 0x0fffffff) < 0) {
			printk(KERN_ERR "error to set 28bit mask DMA\n");
			pci_disable_device(pci);
			return -ENXIO;
	}

	/* allocate a chip-specific data with zero filled */
	chip = kcalloc(1, sizeof(*chip), GFP_KERNEL);
	if (chip == NULL) {
		pci_disable_device(pci);
		return -ENOMEM;
	}

	/* initialize the stuff */
	chip->card = card;
	chip->pci = pci;
	chip->irq = -1;

	/* so that snd_chip_free will work as intended */
	chip->card->private_data = chip;

	/* PCI resource allocation */
	if ((err = pci_request_regions(pci, vr5701_AC97_MODULE_NAME)) < 0) {
		kfree(chip);
		pci_disable_device(pci);
		return err;
	}
	chip->port = pci_resource_start(pci, 0);
	if (request_irq(pci->irq, snd_vr5701_interrupt,
		SA_INTERRUPT|SA_SHIRQ, vr5701_AC97_MODULE_NAME,
			(void *)chip)) {
				printk(KERN_ERR "cannot grab irq %d\n", pci->irq);
				snd_vr5701_free(chip);
				return -EBUSY;
	}
	chip->irq = pci->irq;

	if ((err = snd_device_new(card, SNDRV_DEV_LOWLEVEL,
		chip, &ops)) < 0) {
			snd_vr5701_free(chip);
			return err;
	}

	snd_card_set_dev(card, &pci->dev);

	*rchip = chip;
	return 0;
}

/* constructor */
static int __devinit snd_vr5701_probe(struct pci_dev *pci,
                               const struct pci_device_id *pci_id)
{
	static int dev;
	snd_card_t *card;
	vr5701_t *chip;
	int err;

	if (dev >= SNDRV_CARDS)
		return -ENODEV;
	if (!enable[dev]) {
		dev++;
		return -ENOENT;
	}

	card = snd_card_new(index[dev], id[dev], THIS_MODULE, 0);
	if (card == NULL)
		return -ENOMEM;

	if ((err = snd_vr5701_create(card, pci, &chip)) < 0) {
		snd_card_free(card);
		return err;
	}

	strcpy(card->driver, vr5701_AC97_MODULE_NAME);
	strcpy(card->shortname, "NEC Electronics Corporation ");
	sprintf(card->longname, "%s at 0x%lx irq %i",
		card->shortname, chip->port, chip->irq);

	printk(KERN_INFO "IO at %#lx, IRQ %d\n", chip->port, chip->irq);
 
	/* codec init */
	if ((err = snd_vr5701_ac97(chip)) < 0)
		return err;

	snd_vr5701_ac97_reset(chip->ac97);
	
	/* Try to enable variable rate audio mode. */
	snd_vr5701_ac97_write(chip->ac97, AC97_EXTENDED_STATUS,
		snd_vr5701_ac97_read(chip->ac97, AC97_EXTENDED_STATUS) | AC97_EI_VRA);

	/* Did we enable it? */
	if (!(snd_vr5701_ac97_read(chip->ac97, AC97_EXTENDED_STATUS) & AC97_EI_VRA))
		printk(KERN_INFO "VRA mode not enabled; rate fixed at %d.",
		       48000);

	/* let us get the default volumne louder */
	snd_vr5701_ac97_write(chip->ac97, 0x2, 0x1010);	/* master volume, middle */
	snd_vr5701_ac97_write(chip->ac97, 0xc, 0x10);	/* phone volume, middle */
	snd_vr5701_ac97_write(chip->ac97, 0x10, 0x8000);/* line-in 2 line-out disable */
	snd_vr5701_ac97_write(chip->ac97, 0x18, 0x0707);/* PCM out (line out) middle */

	/* by default we select line in the input */
	snd_vr5701_ac97_write(chip->ac97, 0xe, 0x10);	/* misc volume, middle */
	snd_vr5701_ac97_write(chip->ac97, 0x1a, 0x0000);/* default line is Line_mic */
	snd_vr5701_ac97_write(chip->ac97, 0x1c, 0x0f0f);
	snd_vr5701_ac97_write(chip->ac97, 0x1e, 0x07);

	/* enable the master interrupt but disable all others */
	outl(vr5701_INT_MASK_NMASK, chip->port + vr5701_INT_MASK);

	if ((err = snd_vr5701_new_pcm(chip)) < 0) {
		snd_card_free(card);
		return err;
	}

	if ((err = snd_card_register(card)) < 0) {
		snd_card_free(card);
		return err;
	}

	pci_set_drvdata(pci, card);
	dev++;
	return 0;
}

/* destructor */
static void __devexit snd_vr5701_remove(struct pci_dev *pci)
{
	snd_card_free(pci_get_drvdata(pci));
	pci_set_drvdata(pci, NULL);
}

/* PCI IDs */
static struct pci_device_id snd_vr5701_ids[] = {
	{ PCI_VENDOR_ID_NEC, PCI_DEVICE_ID_NEC_VRC5477_AC97,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0, },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, snd_vr5701_ids);

/* pci_driver definition */
static struct pci_driver driver = {
	.name = vr5701_AC97_MODULE_NAME,
	.id_table = snd_vr5701_ids,
	.probe = snd_vr5701_probe,
	.remove = __devexit_p(snd_vr5701_remove),
	SND_PCI_PM_CALLBACKS
};

/* initialization of the module */
static int __init alsa_card_vr5701_init(void)
{
	printk(KERN_INFO "ALSA NEC Electronics Corporation VR5701 SolutionGearII AC97 driver: time "
					 __TIME__ " " __DATE__" by Sergey Podstavin\n");
	return pci_register_driver(&driver);
}

/* clean up the module */
static void __exit alsa_card_vr5701_exit(void)
{
	pci_unregister_driver(&driver);
}

module_init(alsa_card_vr5701_init)
module_exit(alsa_card_vr5701_exit)

MODULE_AUTHOR("Sergey Podstavin");
MODULE_DESCRIPTION("NEC Electronics Corporation VR5701 SolutionGearII audio (AC97) Driver");
MODULE_LICENSE("GPL");
