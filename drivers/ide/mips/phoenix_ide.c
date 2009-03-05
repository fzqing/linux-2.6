/*
 * Copyright (C) 2004 Raza Microelelctronics Inc 
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

/*  Derived loosely from ide-pmac.c, so:
 *  
 *  Copyright (C) 1998 Paul Mackerras.
 *  Copyright (C) 1995-1998 Mark Lord
 */

/*
 * Boards with phoenix processors so far have supported IDE devices via
 * the Generic Bus, PCI bus, and built-in PCMCIA interface.  In all
 * cases, byte-swapping must be avoided for these devices (whereas
 * other PCI devices, for example, will require swapping).  Any
 * phoenix-targetted kernel including IDE support will include this
 * file.  Probing of a Generic Bus for an IDE device is controlled by
 * the definitions of "PHOENIX_HAVE_IDE" and "IDE_PHYS", which are
 * provided by <asm/rmi/phoenix_ide.h> for ATX1 boards.
 *
 */

#include <linux/kernel.h>
#include <linux/ide.h>
#include <asm/rmi/phoenix_ide.h>
#include <asm/rmi/64bit.h>
#include <asm/rmi/pic.h>

#undef DEBUG_PORT 

#define GPIO_INTR_CLR_REG    0x1EF19180
#define PCMCIA_CONFIG_REG    0x1EF19140

/*
 * Our non-swapping I/O operations.  
 */
static inline void phoenix_outb(u8 val, unsigned long port) {
#ifdef DEBUG_PORT
	printk(" %s port = %x %x \n", __FUNCTION__, (mips_io_port_base + port),val);
#endif
	*(volatile u8 *)(mips_io_port_base + (port)) = val;
}

static inline void phoenix_outw(u16 val, unsigned long port) {
#ifdef DEBUG_PORT
	printk("%s  port = %x  %x\n",__FUNCTION__,  (mips_io_port_base + port),val);
#endif
	*(volatile u16 *)(mips_io_port_base + (port)) = val;
}

static inline void phoenix_outl(u32 val, unsigned long port) {
#ifdef DEBUG_PORT
	printk("%s  port = %x %x\n",__FUNCTION__,  (mips_io_port_base + port),val);
#endif
	*(volatile u32 *)(mips_io_port_base + (port)) = val;
}

static inline unsigned char phoenix_inb(unsigned long port)
{
	unsigned char val ;
	val =  (*(volatile u8 *)(mips_io_port_base + (port)));
#ifdef DEBUG_PORT
	printk("%s  port = %x %x \n",__FUNCTION__,  (mips_io_port_base + port),val );
#endif
	return val;
}

static inline unsigned short phoenix_inw(unsigned long port)
{
	unsigned short val ;
	val = (*(volatile u16 *)(mips_io_port_base + (port)));
#ifdef DEBUG_PORT
	printk("%s  port = %x %x \n",__FUNCTION__,  (mips_io_port_base + port),val );
#endif
	return val;
}

static inline unsigned int phoenix_inl(unsigned long port)
{
	unsigned int val ;
	val =  (*(volatile u32 *)(mips_io_port_base + (port)));
#ifdef DEBUG_PORT
	printk("%s  port = %x %x \n",__FUNCTION__,  (mips_io_port_base + port),val );
#endif
	return val;
}

static inline void phoenix_outsb(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		phoenix_outb(*(u8 *)addr, port);
		addr++;
	}
}

static inline void phoenix_insb(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		*(u8 *)addr = phoenix_inb(port);
		addr++;
	}
}

static inline void phoenix_outsw(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		phoenix_outw(*(u16 *)addr, port);
		addr += 2;
	}
}

static inline void phoenix_insw(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		*(u16 *)addr = phoenix_inw(port);
		addr += 2;
	}
}

static inline void phoenix_outsl(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		phoenix_outl(*(u32 *)addr, port);
		addr += 4;
	}
}

static inline void phoenix_insl(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		*(u32 *)addr = phoenix_inl(port);
		addr += 4;
	}
}

static void phoenix_set_ideops(ide_hwif_t *hwif)
{
	hwif->INB 	= phoenix_inb;
	hwif->INW 	= phoenix_inw;
	hwif->INL 	= phoenix_inl;
	hwif->OUTB 	= phoenix_outb;
	hwif->OUTW 	= phoenix_outw;
	hwif->OUTL 	= phoenix_outl;
	hwif->INSW 	= phoenix_insw;
	hwif->INSL 	= phoenix_insl;
	hwif->OUTSW 	= phoenix_outsw;
	hwif->OUTSL 	= phoenix_outsl;
}


int phoenix_pcmcia_ide_ack_intr(struct hwif_s *hwif) {

	unsigned int  reg;

	/* In Phoenix the PC_READY is connected to GPIO 5 So the corressponing bit needs to 
	 * be cleared for the Phoenix/ATX1 - gmuruga 11/05/04 */

	/* Clear out the GPIO edge detector */
	reg = *(unsigned long *)(CKSEG1ADDR(GPIO_INTR_CLR_REG));
	printk("reg %x \n", reg);
	while ( !(reg & 0x80)) {
		printk("wfrs\n");
		reg = *(unsigned long *)(CKSEG1ADDR(GPIO_INTR_CLR_REG));
	}
	*(unsigned long *) CKSEG1ADDR(GPIO_INTR_CLR_REG) = 0xFFFFFFFF ;
	printk("reg %x \n", reg);
	reg = *(unsigned long *)(CKSEG1ADDR(GPIO_INTR_CLR_REG));
	printk("reg %x \n", reg);
	return 1;
}

/*
 * phoenix_ide_probe - if the board header indicates the existence of
 * Generic Bus IDE, allocate a HWIF for it.
 */
void __init phoenix_ide_probe(void) {
	
#if defined(PHOENIX_HAVE_IDE) && defined(IDE_PHYS)
	unsigned int i = 0;
	ide_hwif_t *phoenix_ide_hwif;

	for (i = 0; i < MAX_HWIFS; i++) {
		if (ide_hwifs[i].io_ports[IDE_DATA_OFFSET]) {
			/* Find an empty slot */
			/* printk("Find a next empty slot %d \n", i); */
			break;
		}
	}

	/* Preadjust for mips_io_port_base since the I/O ops expect
	 * relative addresses
	 */
	
#define PHOENIX_IDE_REG(pcaddr) (IOADDR(IDE_PHYS) + ((pcaddr)) - mips_io_port_base)

	phoenix_ide_hwif = &ide_hwifs[i];
	phoenix_ide_hwif->hw.io_ports[IDE_DATA_OFFSET]    = PHOENIX_IDE_REG(0x1f0);
	phoenix_ide_hwif->hw.io_ports[IDE_ERROR_OFFSET]   = PHOENIX_IDE_REG(0x1f1);
	phoenix_ide_hwif->hw.io_ports[IDE_NSECTOR_OFFSET] = PHOENIX_IDE_REG(0x1f2);
	phoenix_ide_hwif->hw.io_ports[IDE_SECTOR_OFFSET]  = PHOENIX_IDE_REG(0x1f3);
	phoenix_ide_hwif->hw.io_ports[IDE_LCYL_OFFSET]    = PHOENIX_IDE_REG(0x1f4);
	phoenix_ide_hwif->hw.io_ports[IDE_HCYL_OFFSET]    = PHOENIX_IDE_REG(0x1f5);
	phoenix_ide_hwif->hw.io_ports[IDE_SELECT_OFFSET]  = PHOENIX_IDE_REG(0x1f6);
	phoenix_ide_hwif->hw.io_ports[IDE_STATUS_OFFSET]  = PHOENIX_IDE_REG(0x1f7);
	phoenix_ide_hwif->hw.io_ports[IDE_CONTROL_OFFSET] = PHOENIX_IDE_REG(0x3f6);
	phoenix_ide_hwif->hw.io_ports[IDE_IRQ_OFFSET]     = PHOENIX_IDE_REG(0x3f7);

	phoenix_ide_hwif->hw.irq                          = PIC_PCMCIA_IRQ;
	phoenix_ide_hwif->irq                             = PIC_PCMCIA_IRQ;
	phoenix_ide_hwif->hw.dma                          = NODMA;
	phoenix_ide_hwif->dma                             = NODMA;
	phoenix_ide_hwif->hw.ack_intr                     = NULL;
	phoenix_ide_hwif->noprobe                         = 0;

	memcpy(phoenix_ide_hwif->io_ports, phoenix_ide_hwif->hw.io_ports, sizeof(phoenix_ide_hwif->io_ports));
	printk("Phoenix onboard IDE configured as device %d\n", i);

	/* Prevent resource map manipulation */
	phoenix_ide_hwif->mmio = 2;

	/* Reset the ideops */
	phoenix_set_ideops(phoenix_ide_hwif);
#endif
}
