/*
 * include/asm-sh/se7780/io.h
 *
 * Copyright (C)  2005  Takashi Kusuda
 *
 * May be copied or modified under the terms of the GNU General Public
 * License.  See linux/COPYING for more information.
 *
 * IO functions for an Hitachi SH7780 SolutionEngine
 */

#ifndef _ASM_SH_SE7780_IO_H
#define _ASM_SH_SE7780_IO_H

extern unsigned char sh7780se_inb(unsigned long port);
extern unsigned short sh7780se_inw(unsigned long port);
extern unsigned int sh7780se_inl(unsigned long port);

extern void sh7780se_outb(unsigned char value, unsigned long port);
extern void sh7780se_outw(unsigned short value, unsigned long port);
extern void sh7780se_outl(unsigned int value, unsigned long port);

extern unsigned char sh7780se_inb_p(unsigned long port);
extern void sh7780se_outb_p(unsigned char value, unsigned long port);

extern void sh7780se_insb(unsigned long port, void *addr, unsigned long count);
extern void sh7780se_insw(unsigned long port, void *addr, unsigned long count);
extern void sh7780se_insl(unsigned long port, void *addr, unsigned long count);
extern void sh7780se_outsb(unsigned long port, const void *addr, unsigned long count);
extern void sh7780se_outsw(unsigned long port, const void *addr, unsigned long count);
extern void sh7780se_outsl(unsigned long port, const void *addr, unsigned long count);

extern unsigned char sh7780se_readb(unsigned long addr);
extern unsigned short sh7780se_readw(unsigned long addr);
extern unsigned int sh7780se_readl(unsigned long addr);
extern void sh7780se_writeb(unsigned char b, unsigned long addr);
extern void sh7780se_writew(unsigned short b, unsigned long addr);
extern void sh7780se_writel(unsigned int b, unsigned long addr);

extern unsigned long sh7780se_isa_port2addr(unsigned long offset);
extern void * sh7780se_ioremap(unsigned long offset, unsigned long size);
extern void sh7780se_iounmap(void *addr);

#endif /* _ASM_SH_SE7780_IO_H */
