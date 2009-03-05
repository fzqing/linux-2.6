/*
 * linux/sound/arm/pxa3xx-wm9713l.h -- Driver for Wolfson wm9713l on
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

/*
 * Sound
 */

         int pxa3xx_wm9713l_snd_init(void);
        void pxa3xx_wm9713l_snd_exit(void);

	 int pxa3xx_wm9713l_snd_mixer(snd_card_t * card);

	 int pxa3xx_wm9713l_snd_set_playback_rate(unsigned int rate);
	 int pxa3xx_wm9713l_snd_set_capture_rate(unsigned int rate);

#ifdef CONFIG_PM
        void pxa3xx_wm9713l_snd_suspend(void);
        void pxa3xx_wm9713l_snd_resume(void);
#endif

/*
 * Touchscreen
 */

         int pxa3xx_wm9713l_ts_init(void);
        void pxa3xx_wm9713l_ts_exit(void);

	/*
	 * returns 0 if ok
	 *         1 if pen up
	 */
         int pxa3xx_wm9713l_ts_get_sample(unsigned short * x, unsigned short * y);
        void pxa3xx_wm9713l_ts_irq_reset(void);

#ifdef CONFIG_PM
        void pxa3xx_wm9713l_ts_suspend(void);
        void pxa3xx_wm9713l_ts_resume(void);
#endif


