/*
 * Copyright (C) 2004  Raza Microelectronics Inc 
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef __ASM_PHOENIX_H
#define __ASM_PHOENIX_H

#include <linux/config.h>

#define  CONFIG_PHOENIX 1

#ifdef CONFIG_PHOENIX
#define PHOENIX_BOARD_NAME "PHOENIX -ATX1"
#define PHOENIX_HAVE_PCMCIA 0
#define PHOENIX_HAVE_IDE    1
#endif


#ifdef PHOENIX_HAVE_IDE
#define IDE_CS          6
#define IDE_PHYS        0x1D000000
#define K_GPIO_GB_IDE   4
#define K_GPIO_PC_READY 11 
#define K_INT_GPIO_0    32 
#define K_INT_GB_IDE    (K_INT_GPIO_0 + K_GPIO_GB_IDE)
#endif

#ifdef PHOENIX_HAVE_PCMCIA
#define PCMCIA_CS       4
#define PCMCIA_PHYS     0x11000000
#define K_INT_PC_READY  (K_INT_GPIO_0 + K_GPIO_PC_READY)
#endif


#define IOADDR(a) (UNCAC_BASE + (a))

#endif /* __ASM_PHOENIX_H */
