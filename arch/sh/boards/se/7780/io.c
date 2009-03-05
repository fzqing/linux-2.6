/*
 * linux/arch/sh/boards/se/7780/io.c
 *
 * Copyright (C) 2004  Takashi Kusuda
 *
 * Based largely on linux/arch/sh/boards/se/77xx/io.c
 *
 * I/O routine for Hitachi 7780 SolutionEngine.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <asm/io.h>
#include <asm/mach/map.h>
#include <asm/addrspace.h>

#include <linux/pci.h>
#include <asm/cpu/pci-sh7780.h>

#define PCI_IO_AREA	SH7780_PCI_IO_BASE
#define PCI_MEM_AREA	SH7780_PCI_MEMORY_BASE

#define SH7780_PCIIOBR_MASK	0xFFE00000
#define PCI_IOMAP(adr)	(PCI_IO_AREA + (adr & ~SH7780_PCIIOBR_MASK))


/* In case someone configures the kernel w/o PCI support: in that */
/* scenario, don't ever bother to check for PCI-window addresses */

/* NOTE: WINDOW CHECK MAY BE A BIT OFF, HIGH PCIBIOS_MIN_IO WRAPS? */
#if defined(CONFIG_PCI)
#define CHECK_SH7780_PCIIO(port) \
  ((port >= PCIBIOS_MIN_IO) && (port < (PCIBIOS_MIN_IO + SH7780_PCI_IO_SIZE)))
#define CHECK_SH7780_PCIMEM(addr) \
  ((addr >= PCIBIOS_MIN_MEM) && (addr < (PCIBIOS_MIN_MEM + SH7780_PCI_MEM_SIZE)))
#else
#define CHECK_SH7780_PCIIO(port) (0)
#define CHECK_SH7780_PCIMEM(addr) (0)
#endif


static inline void delay(void)
{
	ctrl_inw(0xa0000000);
}

static inline unsigned long sh7780se_port2adr(unsigned long port, unsigned char access_size)
{
	if (CHECK_SH7780_PCIIO(port)) {
		/* SH7780 PCI I/O */
		return PCI_IOMAP(port);
#if defined(CONFIG_SMC91X)
	} else if (0x300 <= port && port <= 0x30f) {
		/* Ethernet SMC LAN91C111 */
		return (PA_LAN + port);
#endif
	} else if (PXSEG(port)) {
		return port;
	}

	return (PA_EXT5 + port);
}

unsigned char sh7780se_inb(unsigned long port)
{
	return *(volatile unsigned char *)sh7780se_port2adr(port, 1);
}

unsigned char sh7780se_inb_p(unsigned long port)
{
	unsigned char v = sh7780se_inb(port);
	delay();
	return v;
}

unsigned short sh7780se_inw(unsigned long port)
{
	return *(volatile unsigned short *)sh7780se_port2adr(port, 2);
}

unsigned int sh7780se_inl(unsigned long port)
{
	return *(volatile unsigned int *)sh7780se_port2adr(port, 4);
}

void sh7780se_outb(unsigned char value, unsigned long port)
{
	*(volatile unsigned char *)sh7780se_port2adr(port, 1) = value;
}

void sh7780se_outb_p(unsigned char value, unsigned long port)
{
	sh7780se_outb(value, port);
	delay();
}

void sh7780se_outw(unsigned short value, unsigned long port)
{
	*(volatile unsigned short *)sh7780se_port2adr(port, 2) = value;
}

void sh7780se_outl(unsigned int value, unsigned long port)
{
	*(volatile unsigned int *)sh7780se_port2adr(port, 4) = value;
}

void sh7780se_insb(unsigned long port, void *addr, unsigned long count)
{
	unsigned char *p = addr;
	while (count--) *p++ = sh7780se_inb(port);
}

void sh7780se_insw(unsigned long port, void *addr, unsigned long count)
{
	unsigned short *p = addr;
	while (count--) *p++ = sh7780se_inw(port);
}

void sh7780se_insl(unsigned long port, void *addr, unsigned long count)
{
	unsigned int *p = addr;
	while (count--) *p++ = sh7780se_inl(port);
}

void sh7780se_outsb(unsigned long port, const void *addr, unsigned long count)
{
	unsigned char *p = (unsigned char*)addr;
	while (count--) sh7780se_outb(*p++, port);
}

void sh7780se_outsw(unsigned long port, const void *addr, unsigned long count)
{
	unsigned short *p = (unsigned short*)addr;
	while (count--) sh7780se_outw(*p++, port);
}

void sh7780se_outsl(unsigned long port, const void *addr, unsigned long count)
{
	unsigned int *p = (unsigned int*)addr;
	while (count--) sh7780se_outl(*p++, port);
}

void * sh7780se_ioremap(unsigned long offset, unsigned long size)
{
	if(offset >= 0xfd000000)
		return (void *)offset;
	else
		return (void *)P2SEGADDR(offset);
}

void sh7780se_iounmap(void *addr)
{
}

EXPORT_SYMBOL(sh7780se_iounmap);
EXPORT_SYMBOL(sh7780se_ioremap);

