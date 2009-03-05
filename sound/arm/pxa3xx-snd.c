/*
 * linux/sound/pxa3xx-snd.c -- AC97 support for the Intel PXA2xx chip.
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
#include <sound/initval.h>

#include <asm/hardware.h>
#include <asm/arch/audio.h>
#include <asm/arch/pxa3xx-wm9713l.h>

#include "pxa2xx-pcm.h"

#define SOUND_RATES	SNDRV_PCM_RATE_8000  | SNDRV_PCM_RATE_11025 | \
			SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
			SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
			SNDRV_PCM_RATE_48000 ;

static pxa2xx_pcm_dma_params_t pxa3xx_wm9713l_pcm_out = {
	.name			= "AC97 PCM out",
	.dev_addr		= __PREG(PCDR),
	.drcmr			= &DRCMRTXPCDR,
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST32 | DCMD_WIDTH4,
};

static pxa2xx_pcm_dma_params_t pxa3xx_wm9713l_pcm_in = {
	.name			= "AC97 PCM in",
	.dev_addr		= __PREG(PCDR),
	.drcmr			= &DRCMRRXPCDR,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST32 | DCMD_WIDTH4,
};

static snd_pcm_t *pxa3xx_wm9713l_pcm;

static int pxa3xx_wm9713l_pcm_startup(snd_pcm_substream_t *substream)
{
	snd_pcm_runtime_t *runtime = substream->runtime;
	pxa2xx_audio_ops_t *platform_ops;

	runtime->hw.channels_min = 2;
	runtime->hw.channels_max = 2;

	runtime->hw.rates = SOUND_RATES;
	snd_pcm_limit_hw_rates(runtime);

	platform_ops = substream->pcm->card->dev->platform_data;
	if (platform_ops && platform_ops->startup)
		return platform_ops->startup(substream, platform_ops->priv);
	else
		return 0;
}

static void pxa3xx_wm9713l_pcm_shutdown(snd_pcm_substream_t *substream)
{
	pxa2xx_audio_ops_t *platform_ops;

	platform_ops = substream->pcm->card->dev->platform_data;
	if (platform_ops && platform_ops->shutdown)
		platform_ops->shutdown(substream, platform_ops->priv);
}

static int pxa3xx_wm9713l_pcm_prepare(snd_pcm_substream_t *substream)
{
	snd_pcm_runtime_t *runtime = substream->runtime;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return pxa3xx_wm9713l_snd_set_playback_rate(runtime->rate);
	else
		return pxa3xx_wm9713l_snd_set_capture_rate(runtime->rate);
}

static pxa2xx_pcm_client_t pxa3xx_wm9713l_pcm_client = {
	.playback_params	= &pxa3xx_wm9713l_pcm_out,
	.capture_params		= &pxa3xx_wm9713l_pcm_in,
	.startup		= pxa3xx_wm9713l_pcm_startup,
	.shutdown		= pxa3xx_wm9713l_pcm_shutdown,
	.prepare		= pxa3xx_wm9713l_pcm_prepare,
};

#ifdef CONFIG_PM

static int pxa3xx_snd_do_suspend(snd_card_t *card, unsigned int state)
{
	if (card->power_state != SNDRV_CTL_POWER_D3cold) {
		pxa2xx_audio_ops_t *platform_ops = card->dev->platform_data;
		snd_pcm_suspend_all(pxa3xx_wm9713l_pcm);
		pxa3xx_wm9713l_snd_suspend();
		snd_power_change_state(card, SNDRV_CTL_POWER_D3cold);
		if (platform_ops && platform_ops->suspend)
			platform_ops->suspend(platform_ops->priv);
	}

	return 0;
}

static int pxa3xx_snd_do_resume(snd_card_t *card, unsigned int state)
{
	if (card->power_state != SNDRV_CTL_POWER_D0) {
		pxa2xx_audio_ops_t *platform_ops = card->dev->platform_data;
		if (platform_ops && platform_ops->resume)
			platform_ops->resume(platform_ops->priv);
		pxa3xx_wm9713l_snd_resume();
		snd_power_change_state(card, SNDRV_CTL_POWER_D0);
	}

	return 0;
}

static int pxa3xx_snd_suspend(struct device *_dev, u32 state, u32 level)
{
	snd_card_t *card = dev_get_drvdata(_dev);
	int ret = 0;

	if (card && level == SUSPEND_DISABLE)
		ret = pxa3xx_snd_do_suspend(card, SNDRV_CTL_POWER_D3cold);

	return ret;
}

static int pxa3xx_snd_resume(struct device *_dev, u32 level)
{
	snd_card_t *card = dev_get_drvdata(_dev);
	int ret = 0;

	if (card && level == RESUME_ENABLE)
		ret = pxa3xx_snd_do_resume(card, SNDRV_CTL_POWER_D0);

	return ret;
}

#else
#define pxa3xx_snd_suspend	NULL
#define pxa3xx_snd_resume	NULL
#endif

static int pxa3xx_snd_probe(struct device *dev)
{
	snd_card_t *card;
	int ret;

	ret = pxa3xx_wm9713l_snd_init();
	if (ret)
		return ret;

	ret = -ENOMEM;
	card = snd_card_new(SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
			    THIS_MODULE, 0);
	if (!card)
		goto err;

	card->dev = dev;
	strncpy(card->driver, dev->driver->name, sizeof(card->driver));

	ret = pxa2xx_pcm_new(card, &pxa3xx_wm9713l_pcm_client, &pxa3xx_wm9713l_pcm);
	if (ret)
		goto err;

	ret = pxa3xx_wm9713l_snd_mixer(card);
	if (ret)
		goto err;

	snprintf(card->shortname, sizeof(card->shortname),
		 "%s", "WM9713L");
	snprintf(card->longname, sizeof(card->longname),
		 "%s (%s)", dev->driver->name, card->mixername);

	snd_card_set_pm_callback(card, pxa3xx_snd_do_suspend,
				 pxa3xx_snd_do_resume, NULL);
	ret = snd_card_register(card);
	if (ret == 0) {
		dev_set_drvdata(dev, card);
		return 0;
	}

err:
	if (card)
		snd_card_free(card);

	pxa3xx_wm9713l_snd_exit();

	return ret;
}

static int pxa3xx_snd_remove(struct device *dev)
{
	snd_card_t *card = dev_get_drvdata(dev);

	if (card) {
		snd_card_free(card);
		dev_set_drvdata(dev, NULL);
	}

	pxa3xx_wm9713l_snd_exit();

	return 0;
}

static struct device_driver pxa3xx_snd_driver = {
	.name		= "pxa2xx-ac97",
	.bus		= &platform_bus_type,
	.probe		= pxa3xx_snd_probe,
	.remove		= pxa3xx_snd_remove,
	.suspend	= pxa3xx_snd_suspend,
	.resume		= pxa3xx_snd_resume,
};

static int __init pxa3xx_snd_init(void)
{
	return driver_register(&pxa3xx_snd_driver);
}

static void __exit pxa3xx_snd_exit(void)
{
	driver_unregister(&pxa3xx_snd_driver);
}

module_init(pxa3xx_snd_init);
module_exit(pxa3xx_snd_exit);

MODULE_AUTHOR("Nicolas Pitre");
MODULE_DESCRIPTION("AC97 driver for the Intel PXA3xx chip");
MODULE_LICENSE("GPL");
