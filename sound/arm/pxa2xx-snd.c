/*
 * linux/sound/pxa2xx-snd.c -- AC97 support for the Intel PXA2xx chip.
 *
 * Author:	Nicolas Pitre
 * Created:	Dec 02, 2004
 * Copyright:	MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>

#include <asm/hardware.h>
#include <asm/arch/audio.h>

#include "pxa2xx-pcm.h"
#include "pxa2xx-ac97.h"

static unsigned short pxa2xx_snd_read(ac97_t *ac97, unsigned short reg)
{
	return pxa2xx_ac97_read(ac97->num, reg);
}

static void pxa2xx_snd_write(ac97_t *ac97, unsigned short reg, unsigned short val)
{
	pxa2xx_ac97_write(ac97->num, reg, val);
}

static void pxa2xx_snd_reset(ac97_t *ac97)
{
	pxa2xx_ac97_reset();
}

static ac97_bus_ops_t pxa2xx_ac97_ops = {
	.read	= pxa2xx_snd_read,
	.write	= pxa2xx_snd_write,
	.reset	= pxa2xx_snd_reset,
};

static pxa2xx_pcm_dma_params_t pxa2xx_ac97_pcm_out = {
	.name			= "AC97 PCM out",
	.dev_addr		= __PREG(PCDR),
	.drcmr			= &DRCMRTXPCDR,
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST32 | DCMD_WIDTH4,
};

static pxa2xx_pcm_dma_params_t pxa2xx_ac97_pcm_in = {
	.name			= "AC97 PCM in",
	.dev_addr		= __PREG(PCDR),
	.drcmr			= &DRCMRRXPCDR,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST32 | DCMD_WIDTH4,
};

static snd_pcm_t *pxa2xx_ac97_pcm;
static ac97_t *pxa2xx_ac97_ac97;

static int pxa2xx_ac97_pcm_startup(snd_pcm_substream_t *substream)
{
	snd_pcm_runtime_t *runtime = substream->runtime;
	pxa2xx_audio_ops_t *platform_ops;
	int r;

	runtime->hw.channels_min = 2;
	runtime->hw.channels_max = 2;

	r = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
	    AC97_RATES_FRONT_DAC : AC97_RATES_ADC;
	runtime->hw.rates = pxa2xx_ac97_ac97->rates[r];
	snd_pcm_limit_hw_rates(runtime);

       	platform_ops = substream->pcm->card->dev->platform_data;
	if (platform_ops && platform_ops->startup)
		return platform_ops->startup(substream, platform_ops->priv);
	else
		return 0;
}

static void pxa2xx_ac97_pcm_shutdown(snd_pcm_substream_t *substream)
{
	pxa2xx_audio_ops_t *platform_ops;

       	platform_ops = substream->pcm->card->dev->platform_data;
	if (platform_ops && platform_ops->shutdown)
		platform_ops->shutdown(substream, platform_ops->priv);
}

static int pxa2xx_ac97_pcm_prepare(snd_pcm_substream_t *substream)
{
	snd_pcm_runtime_t *runtime = substream->runtime;
	int reg = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
		  AC97_PCM_FRONT_DAC_RATE : AC97_PCM_LR_ADC_RATE;
	return snd_ac97_set_rate(pxa2xx_ac97_ac97, reg, runtime->rate);
}

static pxa2xx_pcm_client_t pxa2xx_ac97_pcm_client = {
	.playback_params	= &pxa2xx_ac97_pcm_out,
	.capture_params		= &pxa2xx_ac97_pcm_in,
	.startup		= pxa2xx_ac97_pcm_startup,
	.shutdown		= pxa2xx_ac97_pcm_shutdown,
	.prepare		= pxa2xx_ac97_pcm_prepare,
};

#ifdef CONFIG_PM

static int pxa2xx_snd_do_suspend(snd_card_t *card, unsigned int state)
{
	if (card->power_state != SNDRV_CTL_POWER_D3cold) {
		pxa2xx_audio_ops_t *platform_ops = card->dev->platform_data;
		snd_pcm_suspend_all(pxa2xx_ac97_pcm);
		snd_ac97_suspend(pxa2xx_ac97_ac97);
		snd_power_change_state(card, SNDRV_CTL_POWER_D3cold);
		if (platform_ops && platform_ops->suspend)
			platform_ops->suspend(platform_ops->priv);
		pxa2xx_ac97_suspend();
	}

	return 0;
}

static int pxa2xx_snd_do_resume(snd_card_t *card, unsigned int state)
{
	if (card->power_state != SNDRV_CTL_POWER_D0) {
		pxa2xx_audio_ops_t *platform_ops = card->dev->platform_data;
		pxa2xx_ac97_resume();
		if (platform_ops && platform_ops->resume)
			platform_ops->resume(platform_ops->priv);
		snd_ac97_resume(pxa2xx_ac97_ac97);
		snd_power_change_state(card, SNDRV_CTL_POWER_D0);
	}

	return 0;
}

static int pxa2xx_snd_suspend(struct device *_dev, u32 state, u32 level)
{
	snd_card_t *card = dev_get_drvdata(_dev);
	int ret = 0;

	if (card && level == SUSPEND_DISABLE)
		ret = pxa2xx_snd_do_suspend(card, SNDRV_CTL_POWER_D3cold);

	return ret;
}

static int pxa2xx_snd_resume(struct device *_dev, u32 level)
{
	snd_card_t *card = dev_get_drvdata(_dev);
	int ret = 0;

	if (card && level == RESUME_ENABLE)
		ret = pxa2xx_snd_do_resume(card, SNDRV_CTL_POWER_D0);

	return ret;
}

#else
#define pxa2xx_snd_suspend	NULL
#define pxa2xx_snd_resume	NULL
#endif

static int pxa2xx_snd_probe(struct device *dev)
{
	snd_card_t *card;
	ac97_bus_t *ac97_bus;
	ac97_template_t ac97_template;
	int ret;

	ret = pxa2xx_ac97_init();
	if (ret)
		return ret;

	ret = -ENOMEM;
	card = snd_card_new(SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
			    THIS_MODULE, 0);
	if (!card)
		goto err;

	card->dev = dev;
	strncpy(card->driver, dev->driver->name, sizeof(card->driver));

	ret = pxa2xx_pcm_new(card, &pxa2xx_ac97_pcm_client, &pxa2xx_ac97_pcm);
	if (ret)
		goto err;

	ret = snd_ac97_bus(card, 0, &pxa2xx_ac97_ops, NULL, &ac97_bus);
	if (ret)
		goto err;
	memset(&ac97_template, 0, sizeof(ac97_template));
	ret = snd_ac97_mixer(ac97_bus, &ac97_template, &pxa2xx_ac97_ac97);
	if (ret)
		goto err;

	snprintf(card->shortname, sizeof(card->shortname),
		 "%s", snd_ac97_get_short_name(pxa2xx_ac97_ac97));
	snprintf(card->longname, sizeof(card->longname),
		 "%s (%s)", dev->driver->name, card->mixername);

	snd_card_set_pm_callback(card, pxa2xx_snd_do_suspend,
				 pxa2xx_snd_do_resume, NULL);
	ret = snd_card_register(card);
	if (ret == 0) {
		dev_set_drvdata(dev, card);
		return 0;
	}

 err:
	if (card)
		snd_card_free(card);
	pxa2xx_ac97_exit();
	return ret;
}

static int pxa2xx_snd_remove(struct device *dev)
{
	snd_card_t *card = dev_get_drvdata(dev);

	if (card) {
		snd_card_free(card);
		dev_set_drvdata(dev, NULL);
	}
	pxa2xx_ac97_exit();

	return 0;
}

static struct device_driver pxa2xx_snd_driver = {
	.name		= "pxa2xx-ac97",
	.bus		= &platform_bus_type,
	.probe		= pxa2xx_snd_probe,
	.remove		= pxa2xx_snd_remove,
	.suspend	= pxa2xx_snd_suspend,
	.resume		= pxa2xx_snd_resume,
};

static int __init pxa2xx_snd_init(void)
{
	return driver_register(&pxa2xx_snd_driver);
}

static void __exit pxa2xx_snd_exit(void)
{
	driver_unregister(&pxa2xx_snd_driver);
}

module_init(pxa2xx_snd_init);
module_exit(pxa2xx_snd_exit);

MODULE_AUTHOR("Nicolas Pitre");
MODULE_DESCRIPTION("AC97 driver for the Intel PXA2xx chip");
MODULE_LICENSE("GPL");
