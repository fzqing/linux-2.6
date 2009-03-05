/*
 * Copyright (C) 1996 David S. Miller (dm@engr.sgi.com)
 * Copyright (C) 1997, 2001 Ralf Baechle (ralf@gnu.org)
 * Copyright © 2005 Raza Microelectronics, Inc. (.RMI.)
 *
 * This program is free software.  You may use it, redistribute it 
 * and/or modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation; either version two of the 
 * License or (at your option) any later version.
 *
 * This program is distributed in the hope that you will find it useful.  
 * Notwithstanding the foregoing, you understand and agree that this program 
 * is provided by RMI .as is,. and without any warranties, whether express, 
 * implied or statutory, including without limitation any implied warranty of 
 * non-infringement, merchantability or fitness for a particular purpose.  
 * In no event will RMI be liable for any loss of data, lost profits, cost 
 * of procurement of substitute technology or services or for any direct, 
 * indirect, incidental, consequential or special damages arising from the 
 * use of this program, however caused.  Your unconditional agreement to 
 * these terms and conditions is an express condition to, and shall be deemed 
 * to occur upon, your use, redistribution and/or modification of this program.
 *
 * See the GNU General Public License for more details.  
 */
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/ide.h>

#include <asm/io.h>

#include <asm/rmi/interrupt.h>
#include <asm/rmi/pci.h>
#include <asm/rmi/iomap.h>
#include <asm/rmi/mips-exts.h>
#include <asm/rmi/sim.h>

#define  PCI_HT_LCTR_INIT   0x0020  /* Initialization Complete */

#define LSU_CFG0_REGID       0
#define LSU_CERRLOG_REGID    9
#define LSU_CERROVF_REGID    10
#define LSU_CERRINT_REGID    11


#define pci_cfg_offset(bus,devfn,where) (((bus)<<16)+((devfn)<<8)+(where))
#define pci_cfg_addr(bus,devfn,where) pci_cfg_offset((bus)->number,(devfn),where)
static int  pci_bus_status;
static int  pci_start_busno;
static int  pci_start_bus_fixed;
/*
  Maximum bus number on PCI is 0xff,
  hence, start the ht_busno with 
  0xff + 1. This variable will get
  reset to the actual value when
  the enumeration of HT begins.
*/
static int  ht_start_busno = 0;
static int  ht_start_bus_fixed;
static void *pci_config_base;
void *pci_io_base;

static void *ht_io_base;

void *ht_config_base;

#define CFGTYPE(x) ((x)<(1)?(0):(1))
#define MB16 0x01000000

#define SWAP32(x)\
        (((x) & 0xff000000) >> 24) | \
        (((x) & 0x000000ff) << 24) | \
        (((x) & 0x0000ff00) << 8)  | \
        (((x) & 0x00ff0000) >> 8)

static __inline__ void disable_and_clear_cache_error(void)
{ 
        uint64_t lsu_cfg0 = read_64bit_phnx_ctrl_reg(CPU_BLOCKID_LSU, LSU_CFG0_REGID);
        lsu_cfg0 = lsu_cfg0 & ~0x2e;
        write_64bit_phnx_ctrl_reg(CPU_BLOCKID_LSU, LSU_CFG0_REGID, lsu_cfg0);
        /* Clear cache error log */
        write_64bit_phnx_ctrl_reg(CPU_BLOCKID_LSU, LSU_CERRLOG_REGID, 0);
}

static __inline__ void clear_and_enable_cache_error(void)
{ 
        uint64_t lsu_cfg0 = 0;

        /* first clear the cache error logging register */
        write_64bit_phnx_ctrl_reg(CPU_BLOCKID_LSU, LSU_CERRLOG_REGID, 0);
        write_64bit_phnx_ctrl_reg(CPU_BLOCKID_LSU, LSU_CERROVF_REGID, 0);
        write_64bit_phnx_ctrl_reg(CPU_BLOCKID_LSU, LSU_CERRINT_REGID, 0);

        lsu_cfg0 = read_64bit_phnx_ctrl_reg(CPU_BLOCKID_LSU, LSU_CFG0_REGID);
        lsu_cfg0 = lsu_cfg0 | 0x2e;
        write_64bit_phnx_ctrl_reg(CPU_BLOCKID_LSU, LSU_CFG0_REGID, lsu_cfg0);
}


static inline int ht_controller_init_done(void) 
{
    int init_done=0;
    phoenix_reg_t *ht_mmio = phoenix_io_mmio(PHOENIX_IO_HT_OFFSET);
    phoenix_reg_t reg = cpu_to_le32(phoenix_read_reg(ht_mmio, (0xA4 >> 2)));
    if ((uint16_t)reg & PCI_HT_LCTR_INIT)
        init_done = 1;
    else
        printk("Skipping XLR HT-Controller Registration...\n");
    return init_done;
}
#if 1
static inline __u32 pci_cfg_read_32bit(unsigned long addr)
{
        __u32 temp = 0;
        __u32 *p = (__u32 *) (pci_config_base + (addr & ~3));
        __u64 cerr_cpu_log = 0;

	disable_and_clear_cache_error();

        temp = SWAP32(*p);

        /* Read cache err log */
        cerr_cpu_log = read_64bit_phnx_ctrl_reg(CPU_BLOCKID_LSU, LSU_CERRLOG_REGID);

        if(cerr_cpu_log)
        {
                /* Device don't exist. */
                temp = ~0x0;
        }
	clear_and_enable_cache_error();
        return temp;

}
static inline void pci_cfg_write_32bit(unsigned long addr, __u32 data)
{
        unsigned int *p = (unsigned int *)(pci_config_base + (addr & ~3));

        *p = SWAP32(data);
}

#else
/*
 * Read/write 32-bit values in config space.
 * pci config space is little endian
 */
static inline __u32 pci_cfg_read_32bit(unsigned long addr)
{
  __u8 *p = (__u8 *)(pci_config_base + (addr & ~3));

  //printk("[%s]: addr = %p, data = %x\n", __FUNCTION__, p, *(__u32*)p);
  
  return ( (*(p+3) << 24) | (*(p+2) << 16) | (*(p+1) << 8) | *p);
}

static inline void pci_cfg_write_32bit(unsigned long addr, __u32 data)
{
  __u8 *p = (__u8 *)(pci_config_base + (addr & ~3));
  int i=0;

  for(i=0;i<4;i++)
    p[i] = (data >> (i<<3)) & 0xff;
}
#endif
/* 
 * Low-level HT Configuration READ and Write Routines 
 */
static inline __u32 ht_cfg_read_32bit(unsigned long addr) {


    __u8 *p;
    __u32 temp = 0;
    __u64 cerr_cpu_log = 0;

    disable_and_clear_cache_error();
    p = (__u8 *)((addr & ~3));
   //printk("[%s]: addr = %p, data = %x\n", __FUNCTION__, p, *(__u32*)p);
    temp =  ( (*(p+3) << 24) | (*(p+2) << 16) | (*(p+1) << 8) | *p);

    cerr_cpu_log = read_64bit_phnx_ctrl_reg(CPU_BLOCKID_LSU, LSU_CERRLOG_REGID);

    if(cerr_cpu_log)
    {
	    /* Device don't exist. */
	    temp = ~0x0;
    }
    clear_and_enable_cache_error();

    return temp;
}

static inline void ht_cfg_write_32bit(unsigned long addr, __u32 data) {

    __u8 *p;
    int i=0;

    p = (__u8 *)((addr & ~3));

    for(i=0;i<4;i++)
        p[i] = (data >> (i<<3)) & 0xff;
}
/* 
 * HT Wrapper Routine: READ 
 */
static int phoenix_htbios_read(struct pci_bus *bus, unsigned int devfn,
                               int where, int size, u32 * val)
{
    __u32 data = 0;
    unsigned long long int cfgaddr;

    /* Keep track of where the PCIX
     * bus numbering starts from..
     */
    if (!ht_start_bus_fixed) {
	    ht_start_busno     = (int)(bus->number);
	    ht_start_bus_fixed = 1;
    }

    if ((size == 2) && (where & 1))
        return PCIBIOS_BAD_REGISTER_NUMBER;
    else if ((size == 4) && (where & 3))
        return PCIBIOS_BAD_REGISTER_NUMBER;

    cfgaddr = (long) ht_config_base +
	    CFGTYPE(bus->number - ht_start_busno) * MB16 +
	    pci_cfg_offset((int)(bus->number-ht_start_busno),devfn,where);
	    
    if (pci_bus_status)
	    data = ht_cfg_read_32bit(cfgaddr);
    else
        data = 0xFFFFFFFF;

    if (size == 1)
        *val = (data >> ((where & 3) << 3)) & 0xff;
    else if (size == 2)
        *val = (data >> ((where & 3) << 3)) & 0xffff;
    else
        *val = data;

    return PCIBIOS_SUCCESSFUL;
}
/* 
 * HT Wrapper Routine: WRITE
 */
static int phoenix_htbios_write(struct pci_bus *bus, unsigned int devfn,
                                int where, int size, u32 val)
{  
    unsigned long long int cfgaddr;
//    __u32 cfgaddr = pci_cfg_offset(bus->number , devfn, where);
    __u32 data = 0;

    if ((size == 2) && (where & 1))
        return PCIBIOS_BAD_REGISTER_NUMBER;
    else if ((size == 4) && (where & 3))
        return PCIBIOS_BAD_REGISTER_NUMBER;

    if (!pci_bus_status)
        return PCIBIOS_BAD_REGISTER_NUMBER;

    cfgaddr = (long) ht_config_base +
	    CFGTYPE(bus->number - ht_start_busno) * MB16 +
	    pci_cfg_offset((int)(bus->number-ht_start_busno),devfn,where);
	    

    data = ht_cfg_read_32bit(cfgaddr);

    if (size == 1)
        data = (data & ~(0xff << ((where & 3) << 3))) |
            (val << ((where & 3) << 3));
    else if (size == 2)
        data = (data & ~(0xffff << ((where & 3) << 3))) |
            (val << ((where & 3) << 3));
    else
        data = val;

    ht_cfg_write_32bit(cfgaddr, data);

    return PCIBIOS_SUCCESSFUL;
}

static int phoenix_pcibios_read(struct pci_bus *bus, unsigned int devfn,
			       int where, int size, u32 * val)
{
    __u32 data = 0;

    /* Keep track of where the PCIX
     * bus numbering starts from..
     */
    if (!pci_start_bus_fixed) {
        pci_start_busno     = (int)(bus->number);
        pci_start_bus_fixed = 1;
    }

	if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	if (pci_bus_status)
		data = pci_cfg_read_32bit(pci_cfg_offset((bus->number-pci_start_busno), devfn, where));
	else
		data = 0xFFFFFFFF;

	if (size == 1)
		*val = (data >> ((where & 3) << 3)) & 0xff;
	else if (size == 2)
		*val = (data >> ((where & 3) << 3)) & 0xffff;
	else
		*val = data;

	return PCIBIOS_SUCCESSFUL;
}

static int phoenix_pcibios_write(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 val)
{  
	__u32 cfgaddr = pci_cfg_offset((bus->number-pci_start_busno), devfn, where);
	__u32 data = 0;

	if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	if (!pci_bus_status)
		return PCIBIOS_BAD_REGISTER_NUMBER;

	data = pci_cfg_read_32bit(cfgaddr);

	if (size == 1)
		data = (data & ~(0xff << ((where & 3) << 3))) |
		    (val << ((where & 3) << 3));
	else if (size == 2)
		data = (data & ~(0xffff << ((where & 3) << 3))) |
		    (val << ((where & 3) << 3));
	else
	  data = val;

	pci_cfg_write_32bit(cfgaddr, data);

	return PCIBIOS_SUCCESSFUL;
}

struct pci_ops phoenix_pci_ops = {
    .read  = phoenix_pcibios_read,
    .write = phoenix_pcibios_write
};

struct pci_ops phoenix_ht_ops = {
    .read  = phoenix_htbios_read,
    .write = phoenix_htbios_write
};

/* 
 * XLR PCIX Controller 
 */
static struct resource phoenix_mem_resource = {
    .name           = "PHOENIX PCI MEM",
    .start          = 0xd0000000UL,          /* 256MB PCI mem @ 0xd000_0000 */
    .end            = 0xdfffffffUL,                 
    .flags          = IORESOURCE_MEM,
};
static struct resource phoenix_io_resource = {
    .name           = "PHOENIX IO MEM",
    .start          = 0x10000000UL,         /* 16MB PCI IO @ 0x1000_0000 */
    .end            = 0x100fffffUL,
    .flags          = IORESOURCE_IO,
};

struct pci_controller phoenix_controller = {
    .pci_ops        = &phoenix_pci_ops,
    .mem_resource   = &phoenix_mem_resource,
    .io_resource    = &phoenix_io_resource,
    .io_offset      = 0x00000000UL,
    .mem_offset     = 0x00000000UL
};

/* 
 * XLR HT Controller 
 */
static struct resource phoenix_htmem_resource = {
    .name           = "PHOENIX HT MEM",
    .start          = 0xc0000000UL,                 /* 256MB HT mem @ 0xC0000000 */
    .end            = 0xcfffffffUL,
    .flags          = IORESOURCE_MEM,
};
static struct resource phoenix_htio_resource = {
    .name           = "PHOENIX HT IO",
    .start          = 0x14000000UL,                 /* 16MB HT IO @ 0x1400_0000 */
    .end            = 0x140fffffUL,
    .flags          = IORESOURCE_IO,
};
struct pci_controller phoenix_ht_controller = {
    .pci_ops        = &phoenix_ht_ops,
    .mem_resource   = &phoenix_htmem_resource,
    .io_resource    = &phoenix_htio_resource,
    .io_offset      = 0x00000000UL,
    .mem_offset     = 0x00000000UL
};
/* I/O routines for IDE on PCI */
#define pci_ide_phys_to_virt(x) ((x) - (phoenix_io_resource.start) + (unsigned long)pci_io_base )

static inline void rmi_ide_mm_insw(unsigned long port, void *addr, u32 count) 
{
  unsigned long v_port = pci_ide_phys_to_virt(port);
	while (count--) {
		*(u16 *)addr = (__raw_readw(v_port));
		addr += 2;
	}
}

static inline void rmi_ide_mm_insl(unsigned long port, void *addr, unsigned int count) 
{
  unsigned long v_port = pci_ide_phys_to_virt(port);
	while (count--) {
		*(u32 *)addr = (__raw_readl(v_port));
		addr += 4;
	}
}

static inline void rmi_ide_mm_outsw(unsigned long port, void *addr, unsigned int count) 
{
  unsigned long v_port = pci_ide_phys_to_virt(port);
	while (count--) {
		__raw_writew(*(u16 *)addr, v_port);
		addr += 2;
	}
}

static inline void rmi_ide_mm_outsl(unsigned long port, void *addr, unsigned int count) 
{
  unsigned long v_port = pci_ide_phys_to_virt(port);
	while (count--) {
		__raw_writel(*(u32 *)addr, v_port);
		addr += 4;
	}
}

static u8 rmi_ide_mm_inb (unsigned long port)
{
	return((u8)__raw_readb(pci_ide_phys_to_virt(port)));
}

static u16 rmi_ide_mm_inw (unsigned long port)
{
	return ((u16) swab16(__raw_readw(pci_ide_phys_to_virt(port))));
}

static u32 rmi_ide_mm_inl (unsigned long port)
{
	return ((u32)swab32(__raw_readl(pci_ide_phys_to_virt(port))));
}

static void rmi_ide_mm_outb (u8 value, unsigned long port)
{
	__raw_writeb(value, pci_ide_phys_to_virt(port));
}

static void rmi_ide_mm_outbsync (ide_drive_t *drive, u8 value, unsigned long port)
{
	__raw_writeb(value, pci_ide_phys_to_virt(port));
}

static void rmi_ide_mm_outw (u16 value, unsigned long port)
{
	__raw_writew(swab16(value), pci_ide_phys_to_virt(port));
}

static void rmi_ide_mm_outl (u32 value, unsigned long port)
{
	__raw_writel(swab32(value), pci_ide_phys_to_virt(port));
}

void xlr_hwif_mmiops (ide_hwif_t *hwif)
{
	hwif->OUTB      = rmi_ide_mm_outb;
	hwif->OUTBSYNC  = rmi_ide_mm_outbsync;
	hwif->OUTW      = rmi_ide_mm_outw;
	hwif->OUTL      = rmi_ide_mm_outl;
	hwif->OUTSW     = rmi_ide_mm_outsw;
	hwif->OUTSL     = rmi_ide_mm_outsl;
	hwif->INB       = rmi_ide_mm_inb;
	hwif->INW       = rmi_ide_mm_inw;
	hwif->INL       = rmi_ide_mm_inl;
	hwif->INSW      = rmi_ide_mm_insw;
	hwif->INSL      = rmi_ide_mm_insl;
}

EXPORT_SYMBOL(xlr_hwif_mmiops);

int pcibios_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
    if ((dev->bus->number) && (dev->bus->number >= ht_start_busno))
        return PIC_HYPER_IRQ;  /* IRQ Vector 23 for HT Devices */
    else 
        return PIC_PCIX_IRQ;   /* IRQ Vector 24 for PCI/X Devices */
}

extern int pci_probe_only;

static int __initdata xlr_nopci;

static int __init xlr_nopci_setup(char *str) 
{
    xlr_nopci = 1;
    return 1;
}
__setup("xlr_nopci", xlr_nopci_setup);

/* Do platform specific device initialization at pci_enable_device() time */
int pcibios_plat_dev_init(struct pci_dev *dev)
{
	return 0;
}

static int __init pcibios_init(void)
{
  if (xlr_nopci || xlr_board_atx_iii()) return 0;
  
	/* PSB assigns PCI resources */
	pci_probe_only = 1;
	
	/* Map the PCIX CFG space */
	pci_config_base = ioremap(DEFAULT_PCI_CONFIG_BASE, (32<<20));
	if (!pci_config_base) {
		printk("Unable to map PCI config space!\n");
		return 1;
	}
	                                    
	/* Map the HT CFG spaces... */
	ht_config_base = ioremap(DEFAULT_HT_TYPE0_CFG_BASE, (32<<20));
	if (!ht_config_base) {
		printk("Unable to map HT config space!\n");
		return 1;
	}

	{
	  unsigned long phys = phoenix_io_resource.start;
	  unsigned long size = phoenix_io_resource.end - phoenix_io_resource.start + 1;

	  pci_io_base = ioremap(phys, size);
	  if (!pci_io_base) {
	    printk("[%s]: Unable to IO-Remap phys=%lx, size=%lx\n",
		   __FUNCTION__, phys, size);
	  }
	  else {
	    printk("[%s]: IO-Remapped phys=%lx, size=%lx to vaddr=%p\n",
		   __FUNCTION__, phys, size, pci_io_base);
	  }
	}

	{
	  unsigned long phys = phoenix_htio_resource.start;
	  unsigned long size = phoenix_htio_resource.end - phoenix_htio_resource.start + 1;

	  ht_io_base = ioremap(phys, size);
	  if (!ht_io_base) {
	    printk("[%s]: Unable to IO-Remap phys=%lx, size=%lx\n",
		   __FUNCTION__, phys, size);
	  }
	  else {
	    printk("[%s]: IO-Remapped phys=%lx, size=%lx to vaddr=%p\n",
		   __FUNCTION__, phys, size, ht_io_base);
	  }
	}

	pci_bus_status = 1;
	pci_start_bus_fixed = 0;
	ht_start_bus_fixed = 0;

	/* IO Range for 16MB from where the MEM Range Ends */
	ioport_resource.start = 0;
	ioport_resource.end =   ~0;

    printk("Registering XLR PCI Controller \n");
	register_pci_controller(&phoenix_controller);
    
    if ((xlr_board_atx_i()) && ht_controller_init_done()) {
        printk("Registering XLR HT Controller \n");
        register_pci_controller(&phoenix_ht_controller);
    }
	
	return 0;
}

arch_initcall(pcibios_init);

struct pci_fixup pcibios_fixups[] = {
  {0}
};

