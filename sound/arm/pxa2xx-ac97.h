/*
 * sound/arm/pxa2xx-ac97.h -- ac97 support for the Intel PXA2xx chip
 *
 * Author:      Nicolas Pitre
 * Created:     Nov 30, 2004
 * Copyright:   MontaVista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

unsigned short pxa2xx_ac97_read(int num, unsigned short reg);
void pxa2xx_ac97_write(int num, unsigned short reg, unsigned short val);
void pxa2xx_ac97_reset(void);

static inline void pxa2xx_ac97_modify_register(int num, unsigned short reg, unsigned short clear, unsigned short set)
{
       unsigned short val;

       val = pxa2xx_ac97_read(num, reg);
       val &= ~clear;
       val |= set;
       pxa2xx_ac97_write(num, reg, val);
}

int pxa2xx_ac97_init(void);
void pxa2xx_ac97_exit(void);

#if CONFIG_PM
void pxa2xx_ac97_suspend(void);
void pxa2xx_ac97_resume(void);
#endif
