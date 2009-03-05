/*
 * linux/sound/arm/pxa3xx-wm9713l.c -- Driver for Wolfson wm9713l on
 *					Zylonite development platform
 *
 * Author:	Aleksey Makarov
 * Created:	Jul 04, 2007
 * Copyright:	MontaVista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>

#include <asm/arch/pxa3xx-wm9713l.h>

#include "pxa2xx-ac97.h"

/*
 * Registers that are specific to WM9713L
 */
#define WM9713L_DAC_PGA_VOL_ROUTE              0x0C
#define WM9713L_REC_PGA_VOL                    0x12
#define WM9713L_REC_ROUTE_MUX_SEL              0x14
#define WM9713L_OUTPUT_PGA_MUX                 0x1C
#define WM9713L_MIC_BIAS                       0x22
#define WM9713L_AUDIO_DAC_RATE                 0x2C
#define WM9713L_AUDIO_ADC_RATE                 0x32
#define WM9713L_POWER_DOWN_1                   0x3C
#define WM9713L_POWER_DOWN_2                   0x3E
#define WM9713L_GPIO_PIN_CFG		       0x4C
#define WM9713L_GPIO_PIN_STICKY		       0x50
#define WM9713L_GPIO_PIN_WAKEUP		       0x52
#define WM9713L_GPIO_PIN_STATUS		       0x54
#define WM9713L_GPIO_PIN_SHARING	       0x56
#define WM9713L_DIGITIZER_1_WM13               0x74
#define WM9713L_DIGITIZER_2_WM13               0x76
#define WM9713L_DIGITIZER_3_WM13               0x78
#define WM9713L_DIGITIZER_READ_BACK	       0x7a

/*
 * Vendor IDs (registers 0x7c, 0x7e)
 */
#define WM9713L_VENDOR_ID_1		0x574D
#define WM9713L_VENDOR_ID_2		0x4C13

/*
 * Conrtrols
 */
#define WM9713L_DOUBLE(xname, reg, shift_left, shift_right, mask, invert) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), .info = snd_wm9713l_info_double, \
  .get = snd_wm9713l_get_double, .put = snd_wm9713l_put_double, \
  .private_value = (reg) | ((shift_left) << 8) | ((shift_right) << 12) | ((mask) << 16) | ((invert) << 24) }

static int snd_wm9713l_info_double(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t * uinfo)
{
	int mask = (kcontrol->private_value >> 16) & 0xff;

	uinfo->type = mask == 1 ? SNDRV_CTL_ELEM_TYPE_BOOLEAN : SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = mask;
	return 0;
}

static int snd_wm9713l_get_double(snd_kcontrol_t * kcontrol, snd_ctl_elem_value_t * ucontrol)
{
	int reg = kcontrol->private_value & 0xff;
	int shift_left = (kcontrol->private_value >> 8) & 0x0f;
	int shift_right = (kcontrol->private_value >> 12) & 0x0f;
	int mask = (kcontrol->private_value >> 16) & 0xff;
	int invert = (kcontrol->private_value >> 24) & 0xff;
	unsigned short val;

	val = pxa2xx_ac97_read(0, reg);

	ucontrol->value.integer.value[0] = (val >> shift_left) & mask;
	ucontrol->value.integer.value[1] = (val >> shift_right) & mask;
	if (invert) {
		ucontrol->value.integer.value[0] = mask - ucontrol->value.integer.value[0];
		ucontrol->value.integer.value[1] = mask - ucontrol->value.integer.value[1];
	}
	return 0;
}

static int snd_wm9713l_put_double(snd_kcontrol_t * kcontrol, snd_ctl_elem_value_t * ucontrol)
{
	int reg = kcontrol->private_value & 0xff;
	int shift_left = (kcontrol->private_value >> 8) & 0x0f;
	int shift_right = (kcontrol->private_value >> 12) & 0x0f;
	int mask = (kcontrol->private_value >> 16) & 0xff;
	int invert = (kcontrol->private_value >> 24) & 0xff;
	unsigned short val1, val2;

	val1 = ucontrol->value.integer.value[0] & mask;
	val2 = ucontrol->value.integer.value[1] & mask;
	if (invert) {
		val1 = mask - val1;
		val2 = mask - val2;
	}

	pxa2xx_ac97_modify_register(0, reg,
		(mask << shift_left) | (mask << shift_right),
		(val1 << shift_left) | (val2 << shift_right));

	return 0;
}

#define WM9713L_SINGLE(xname, reg, shift, mask, invert) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, .info = snd_wm9713l_info_single, \
  .get = snd_wm9713l_get_single, .put = snd_wm9713l_put_single, \
  .private_value = ((reg) | ((shift) << 8) | ((mask) << 16) | ((invert) << 24)) }

int snd_wm9713l_info_single(snd_kcontrol_t *kcontrol, snd_ctl_elem_info_t * uinfo)
{
	int mask = (kcontrol->private_value >> 16) & 0xff;

	uinfo->type = mask == 1 ? SNDRV_CTL_ELEM_TYPE_BOOLEAN : SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = mask;
	return 0;
}

int snd_wm9713l_get_single(snd_kcontrol_t * kcontrol, snd_ctl_elem_value_t * ucontrol)
{
	int reg = kcontrol->private_value & 0xff;
	int shift = (kcontrol->private_value >> 8) & 0xff;
	int mask = (kcontrol->private_value >> 16) & 0xff;
	int invert = (kcontrol->private_value >> 24) & 0x01;
	unsigned short val;

	val = pxa2xx_ac97_read(0, reg);

	ucontrol->value.integer.value[0] = (val >> shift) & mask;
	if (invert)
		ucontrol->value.integer.value[0] = mask - ucontrol->value.integer.value[0];

	return 0;
}

int snd_wm9713l_put_single(snd_kcontrol_t * kcontrol, snd_ctl_elem_value_t * ucontrol)
{
	int reg = kcontrol->private_value & 0xff;
	int shift = (kcontrol->private_value >> 8) & 0xff;
	int mask = (kcontrol->private_value >> 16) & 0xff;
	int invert = (kcontrol->private_value >> 24) & 0x01;
	unsigned short val;

	val = (ucontrol->value.integer.value[0] & mask);
	if (invert)
		val = mask - val;

	pxa2xx_ac97_modify_register(0, reg, (mask << shift), (val << shift));

	return 0;
}

static snd_kcontrol_new_t wm9713_controls[] = {
WM9713L_DOUBLE("Headphone Playback Volume", AC97_HEADPHONE,       8, 0, 0x3f, 1),
WM9713L_DOUBLE("Headphone Playback Switch", AC97_HEADPHONE,      15, 7, 0x01, 1),
WM9713L_DOUBLE("Mic Capture Volume",        WM9713L_REC_PGA_VOL,  8, 0, 0x3f, 1),
WM9713L_SINGLE("Mic Capture Switch",        WM9713L_REC_PGA_VOL,    15, 0x01, 1),
};

int pxa3xx_wm9713l_snd_mixer(snd_card_t * card)
{
	snd_kcontrol_t *kctl;
	int i;
	int err;

	/*
	 * Add controls
	 */
	for (i = 0; i < ARRAY_SIZE(wm9713_controls); i++) {

		kctl = snd_ctl_new1(&wm9713_controls[i], NULL);
		if (!kctl)
			return -ENOMEM;

		err = snd_ctl_add(card, kctl);
		if (err < 0) {
			snd_ctl_free_one(kctl);
			return err;
		}
	}

	return 0;
}

int pxa3xx_wm9713l_snd_set_playback_rate(unsigned int rate)
{
	pxa2xx_ac97_write(0, WM9713L_AUDIO_DAC_RATE, rate);
	return 0;
}

int pxa3xx_wm9713l_snd_set_capture_rate(unsigned int rate)
{
	pxa2xx_ac97_write(0, WM9713L_AUDIO_ADC_RATE, rate);
	return 0;
}

static int init_chip(void)
{
	unsigned short val;
	static int initialized;

	if (initialized)
		return 0;

	pxa2xx_ac97_init();

	/*
	 * Check that the is codec connected to ac97 bus is WM9713L
	 */
	val = pxa2xx_ac97_read(0, AC97_VENDOR_ID1);
	if (val != WM9713L_VENDOR_ID_1)
		return -ENODEV;

	val = pxa2xx_ac97_read(0, AC97_VENDOR_ID2);
	if (val != WM9713L_VENDOR_ID_2)
		return -ENODEV;

	pxa2xx_ac97_write(0, AC97_POWERDOWN, 0x000);

	initialized = 1;

	return 0;
}

static void snd_pw_on(void)
{
	pxa2xx_ac97_modify_register(0, WM9713L_POWER_DOWN_1, 0x25fc, 0);
	pxa2xx_ac97_modify_register(0, WM9713L_POWER_DOWN_2, 0x460a, 0);
}

static void snd_pw_off(void)
{
	pxa2xx_ac97_modify_register(0, WM9713L_POWER_DOWN_1, 0, 0x25fc);
	pxa2xx_ac97_modify_register(0, WM9713L_POWER_DOWN_2, 0, 0x460a);
}

int pxa3xx_wm9713l_snd_init(void)
{
	int err;

	err = init_chip();
	if (err)
		return err;

	snd_pw_on();

	/*
	 * PLL
	 */
	/* initialise PLL when appropriate - do not forget to power on PLL */

	/*
	 * Capture path
	 */
	/* Turn on MIC1 only */
	pxa2xx_ac97_write(0, WM9713L_MIC_BIAS, 0xc440);
	/* Select MUX source: MICA for both channels; no boost; no path to phone mixer */
	pxa2xx_ac97_write(0, WM9713L_REC_ROUTE_MUX_SEL, 0xd600);

	/*
	 * Playback path
	 */
	/* Unmute to phones mixer only; 0dB gain */
	pxa2xx_ac97_write(0, WM9713L_DAC_PGA_VOL_ROUTE, 0x6808);
	/* mono, spk, out <- disable; hp <- hpmix */
	pxa2xx_ac97_write(0, WM9713L_OUTPUT_PGA_MUX, 0xffaf);

	/*
	 * Playbacl/Capture rates
	 */
	/* Enable variable rate audio */
	pxa2xx_ac97_write(0, AC97_EXTENDED_STATUS, 0x1);
	/* Set rates */
	pxa2xx_ac97_write(0, WM9713L_AUDIO_DAC_RATE, 44100); /* 0x2c */
	pxa2xx_ac97_write(0, WM9713L_AUDIO_ADC_RATE, 16000); /* 0x32 */

	return 0;
}

void pxa3xx_wm9713l_snd_exit(void)
{
	snd_pw_off();
}

#ifdef CONFIG_PM

void pxa3xx_wm9713l_snd_suspend(void)
{
	snd_pw_on();
}

void pxa3xx_wm9713l_snd_resume(void)
{
	snd_pw_off();
}

#endif

/*
 * Touchscreen
 */

static unsigned short ts_get_sample_1(unsigned short reg)
{
        unsigned long wait;
	unsigned short v;

        pxa2xx_ac97_write(0, WM9713L_DIGITIZER_1_WM13, reg);

        wait = 0;
        do {
                v = pxa2xx_ac97_read(0, WM9713L_DIGITIZER_1_WM13);
                if ( !(v & 0x200 ) )
                        break;
        } while ( 100 > wait++ );

        return pxa2xx_ac97_read(0, WM9713L_DIGITIZER_READ_BACK);
}

/*
 * returns 0 if ok
 *         1 if pen up
 */
int pxa3xx_wm9713l_ts_get_sample(unsigned short * x, unsigned short * y)
{
	*x = ts_get_sample_1(0x202);
	if (!(*x & 0x8000))
		return 1;
	*x &= 0xfff;

	*y = ts_get_sample_1(0x204);
	if (!(*y & 0x8000))
		return 1;
	*y &= 0xfff;

	return 0;
}

void pxa3xx_wm9713l_ts_irq_reset(void)
{
	unsigned short v;
	v = pxa2xx_ac97_read(0, WM9713L_GPIO_PIN_STATUS);
	pxa2xx_ac97_write(0, WM9713L_GPIO_PIN_STATUS, v & ~((1<<13)|(1<<2)));
}

static void ts_pw_on(void)
{
	pxa2xx_ac97_modify_register(0, WM9713L_POWER_DOWN_1, 0x8000, 0);
}

static void ts_pw_off(void)
{
	pxa2xx_ac97_modify_register(0, WM9713L_POWER_DOWN_1, 0, 0x8000);
}

int pxa3xx_wm9713l_ts_init(void)
{
	int err;

	err = init_chip();
	if (err)
		return err;

	ts_pw_on();

	pxa2xx_ac97_write(0, WM9713L_DIGITIZER_3_WM13, 0xc008);

	pxa2xx_ac97_modify_register(0, WM9713L_GPIO_PIN_CFG, 0x4, 0);
	pxa2xx_ac97_modify_register(0, WM9713L_GPIO_PIN_SHARING, 0x4, 0);

	pxa2xx_ac97_modify_register(0, WM9713L_GPIO_PIN_WAKEUP, 0, 0x2000);
	pxa2xx_ac97_modify_register(0, WM9713L_GPIO_PIN_STICKY, 0, 0x2000);

	return 0;
}

void pxa3xx_wm9713l_ts_exit(void)
{
	ts_pw_off();
}

#ifdef CONFIG_PM

void pxa3xx_wm9713l_ts_suspend(void)
{
	ts_pw_on();
}

void pxa3xx_wm9713l_ts_resume(void)
{
	ts_pw_off();
}

EXPORT_SYMBOL(pxa3xx_wm9713l_ts_resume);
EXPORT_SYMBOL(pxa3xx_wm9713l_ts_suspend);
#endif

EXPORT_SYMBOL(pxa3xx_wm9713l_ts_init);
EXPORT_SYMBOL(pxa3xx_wm9713l_ts_exit);
EXPORT_SYMBOL(pxa3xx_wm9713l_ts_irq_reset);
EXPORT_SYMBOL(pxa3xx_wm9713l_ts_get_sample);




