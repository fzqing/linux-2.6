/*
 * include/asm-xtensa/platform-xt2000/serial.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

#ifndef __ASM_XTENSA_XT2000_SERIAL_H
#define __ASM_XTENSA_XT2000_SERIAL_H

#include <linux/config.h>
#include <asm/platform/xt2000.h>

#define BASE_BAUD ( DUART16552_XTAL_FREQ / 16 )

#ifdef __XTENSA_EL__
#define IO_BASE_1 (DUART16552_1_VADDR)
#define IO_BASE_2 (DUART16552_2_VADDR)
#elif defined(__XTENSA_EB__)
#define IO_BASE_1 (DUART16552_1_VADDR + 3)
#define IO_BASE_2 (DUART16552_2_VADDR + 3)
#else
#error endianess not defined
#endif

#define RS_TABLE_SIZE 2

#define STD_COM_FLAGS (ASYNC_BOOT_AUTOCONF|ASYNC_SKIP_TEST)

#define _SERIAL_PORT(intnr, base)				\
	{ .baud_base = BASE_BAUD, 				\
	  .irq = intnr,						\
	  .flags = STD_COM_FLAGS,				\
	  .iomem_base = (u8*) base,				\
          .iomem_reg_shift = 2,					\
	  .io_type = SERIAL_IO_MEM, }				\

#define STD_SERIAL_PORT_DFNS 					\
	_SERIAL_PORT (DUART16552_1_INTNUM, IO_BASE_1), 		\
	_SERIAL_PORT (DUART16552_2_INTNUM, IO_BASE_2),

#define SERIAL_PORT_DFNS STD_SERIAL_PORT_DFNS

#endif /* __ASM_XTENSA_XT2000_H */
