/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2005, 2006 Cavium Networks
*/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/time.h>
#include "pci-cvmx.h"

/* Octeon's PCI controller uses did=3, subdid=2 for PCI IO addresses.
   Use PCI endian swapping 1 so no address swapping is necessary. The
   Linux io routines will endian swap the data */
#define OCTEON_PCI_IOSPACE_BASE     0x80011a0400000000ull
#define OCTEON_PCI_IOSPACE_SIZE     (1ull<<32)
/* Octeon't PCI controller uses did=3, subdid=3 for PCI memory. Start PCI
       memory from 0x8000000, right after the 0-128MB BAR1 mapping */
#define OCTEON_PCI_MEMSPACE_OFFSET  (0x00011b0000000000ull)
#define OCTEON_PCI_MEMSPACE_BASE    (0x0000000008000000ull)
#define OCTEON_PCI_MEMSPACE_SIZE    (1ull<<35)

static inline int cavium_irq_map_asus_na038(struct pci_dev *dev, u8 slot,
					    u8 pin)
{
	int irq = ((PCI_SLOT(dev->devfn) + pin - 2) & 3) + 44;
	int irqmap_by_func[] =
	    { 80 + 10, 80 + 14, 80 + 13, 80 + 13, 80, 80, 80, 80 };

	/* The NEC USB */
	if (PCI_SLOT(dev->devfn) == 5)
		irq = (pin - 2) + 44;

	/* The Via southbridge (device 2) is on the GPIO Pin 15 */
	if (PCI_SLOT(dev->devfn) == 2) {
		octeon_i8259_setup(39);
		irq = irqmap_by_func[PCI_FUNC(dev->devfn)];
	}
	return irq;
}

static inline int cavium_irq_map_sonicwall_gadwin(struct pci_dev *dev, u8 slot,
						  u8 pin)
{
	int irqmap_by_slot[] = {
		[0] = -1,
		[1] = 44 + pin - 1,
		[2] = 44,
		[3] = 46,
		[4] = 46,
		[5] = 47,
		[6] = 46,
		[7] = -1,
		[8] = 44,
		[9] = 44,
		[0xA] = 44,[0xB] = 44,[0xC] = 44,[0xE] = 44,[0xF] = 44,
		[0x10] = 44,[0x11] = 44,[0x12] = 44,[0x13] = 44,
		[0x14] = 44,[0x15] = 44,[0x16] = 44,[0x17] = 44,
		[0x18] = 44,[0x19] = 44,[0x1A] = 44,[0x1B] = 44,
		[0x1C] = 44,[0x1D] = 44,[0x1E] = 44,[0x1F] = 44,
	};
	int irqmap_by_func[] = { 80 + 10, 80 + 14, 80 + 13, 80 + 13,
		80, 80, 80,
	};

	int irq;

	if (PCI_SLOT(dev->devfn) == 7) {	/* INTB VIA Southbridge */
		octeon_i8259_setup(39);
		irq = irqmap_by_func[PCI_FUNC(dev->devfn)];
	} else
		irq = irqmap_by_slot[PCI_SLOT(dev->devfn)];
	return irq;

}

static inline int cavium_irq_map_ebh3100(struct pci_dev *dev, u8 slot, int pin)
{
	return ((PCI_SLOT(dev->devfn) - 2) & 3) + 44;
}

/**
 * Map a PCI device to the appropriate interrupt line
 *
 * @param dev
 * @param slot
 * @param pin
 * @return
**/
int __init pcibios_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	int irq_num;

	/* Force the Cache line setting. The standard Linux bus scan doesn't seem
	   to set it */
	pci_write_config_byte(dev, PCI_CACHE_LINE_SIZE, L1_CACHE_BYTES/4);
	irq_num = cavium_irq_map(dev, slot, pin);
	printk("Device at %d:%d.%d slot %d pin %d irqmapped to %d\n",
	       dev->bus->number,
	       PCI_SLOT(dev->devfn), PCI_FUNC(dev->devfn),
	       slot, pin, irq_num);
	return irq_num;
}

static void chip_bridge_setup(struct pci_dev *dev)
{
	uint16_t val;
       
	pci_read_config_word(dev, 0x3e, &val);
	val &= ~(1 << 2);	/* Disable the ISA port filtering */
	val |= 1 << 3;		/* Enable VGA */
	val |= 1 << 4;		/* Enable VGA Alias Filter */
	pci_write_config_word(dev, 0x3E, val);
	pci_write_config_byte(dev, 1, 0x00);
	pci_write_config_byte(dev, 1, 0x00);
}

static void chip_vt82c686b_setup(struct pci_dev *dev)
{
	u8 kbc_rtc;
#ifdef CONFIG_PCI_NAMES
	printk("%s(bus=%d, dev=%d, func=%d, name=%s)\n", __FUNCTION__,
	       dev->bus->number, dev->devfn >> 3, dev->devfn & 7,
	       dev->pretty_name);
#endif
/*
Offset 48 - Miscellaneous Control 3 ................................ RW
  7-4 Reserved ........................................ always reads 0
   3    Extra RTC Port 74/75 Enable
            0 Disable ...................................................default
            1 Enable
   2    Integrated USB Controller Disable
            0 Enable ....................................................default
            1 Disable
   1    Integrated IDE Controller Disable
            0 Enable ....................................................default
            1 Disable
   0    512K PCI Memory Decode
            0 Use Rx4E[15-12] to select top of PCI memory
            1 Use contents of Rx4E[15-12] plus 512K as top
               of PCI memory.......................................default
*/
	pci_write_config_byte( dev, 0x48, 0x09);

/*
Offset 4A - IDE Interrupt Routing..................................RW
   7    Wait for PGNT Before Grant to ISA Master /
        DMA
           0 Disable ...................................................default
           1 Enable
   6    Bus Select for Access to I/O Devices Below 100h
           0 Access ports 00-FFh via XD bus ...........default
           1 Access ports 00-FFh via SD bus (applies to
              external devices only; internal devices such as
              the mouse controller are not effected)
  5-4 Reserved (do not program)..................... default = 0
  3-2 IDE Second Channel IRQ Routing
          00 IRQ14
          01 IRQ15.....................................................default
          10 IRQ10
          11 IRQ11
  1-0 IDE Primary Channel IRQ Routing
          00 IRQ14.....................................................default
          01 IRQ15
          10 IRQ10
          11 IRQ11
*/
	pci_write_config_byte(dev, 0x4a, 0x04);

/*
Offset 4F-4E - ISA DMA/Master Mem Access Ctrl 3 ... RW
 15-12 Top of PCI Memory for ISA DMA/Master accesses
         0000 1M .................................................... default
         0001 2M
          ... ...
         1111 16M
Note: All ISA DMA / Masters that access addresses higher
        than the top of PCI memory will not be directed to
        the PCI bus.
   11 Forward E0000-EFFFF Accesses to PCI ....... def=0
   10 Forward A0000-BFFFF Accesses to PCI ....... def=0
    9   Forward 80000-9FFFF Accesses to PCI ........def=1
    8   Forward 00000-7FFFF Accesses to PCI ........def=1
    7   Forward DC000-DFFFF Accesses to PCI ...... def=0
    6   Forward D8000-DBFFF Accesses to PCI ......def=0
    5   Forward D4000-D7FFF Accesses to PCI ......def=0
    4   Forward D0000-D3FFF Accesses to PCI ......def=0
    3   Forward CC000-CFFFF Accesses to PCI .....def=0
    2   Forward C8000-CBFFF Accesses to PCI ......def=0
    1   Forward C4000-C7FFF Accesses to PCI ......def=0
    0   Forward C0000-C3FFF Accesses to PCI ......def=0
*/
	pci_write_config_word(dev, 0x4e, 0xf000);

/*
Offset 51 - PNP IRQ Routing 1........................................RW
  7-4 PnP Routing for Parallel Port IRQ (see PnP IRQ
        routing table)
  3-0 PnP Routing for Floppy IRQ (see PnP IRQ routing
        table)
*/
	pci_write_config_byte(dev, 0x51, 0x76);

/*
Offset 52 - PNP IRQ Routing 2........................................RW
  7-4 PnP Routing for Serial Port 2 IRQ (see PnP IRQ
        routing table)
  3-0 PnP Routing for Serial Port 1 IRQ (see PnP IRQ
        routing table)
*/
	pci_write_config_byte(dev, 0x52, 0x34);

/*
Offset 5A  KBC / RTC Control......................................RW
Bits 7-4 of this register are latched from pins SD7-4 at power-
up but are read/write accessible so may be changed after
power-up to change the default strap setting:
    7    Keyboard RP16........................... latched from SD7
    6    Keyboard RP15 .......................... latched from SD6
    5    Keyboard RP14 .......................... latched from SD5
    4    Keyboard RP13 .......................... latched from SD4
    3    Reserved ........................................ always reads 0
    2    Internal RTC Enable
            0 Disable
            1 Enable ...................................................default
    1    Internal PS2 Mouse Enable
            0 Disable ..................................................default
            1 Enable
    0    Internal KBC Enable
            0 Disable ..................................................default
            1 Enable
*/
	pci_read_config_byte(dev, 0x5a, &kbc_rtc);
	pci_write_config_byte(dev, 0x5a, kbc_rtc | 0x07);

/*
Offset 61-60 - Distributed DMA Ch 0 Base / Enable ..... RW
 15-4 Channel 0 Base Address Bits 15-4.......... default = 0
   3    Channel 0 Enable
           0 Disable ...................................................default
           1 Enable
  2-0 Reserved ........................................ always reads 0
*/
	pci_write_config_word(dev, 0x60, 0x04);

/*
Offset 63-62 - Distributed DMA Ch 1 Base / Enable ..... RW
 15-4 Channel 1 Base Address Bits 15-4.......... default = 0
   3    Channel 1 Enable
           0 Disable ...................................................default
           1 Enable
  2-0 Reserved ........................................ always reads 0
*/
	pci_write_config_word(dev, 0x62, 0x4);

/*
Offset 65-64 - Distributed DMA Ch 2 Base / Enable ..... RW
 15-4 Channel 2 Base Address Bits 15-4.......... default = 0
   3    Channel 2 Enable
           0 Disable ...................................................default
           1 Enable
  2-0 Reserved ........................................ always reads 0
*/
	pci_write_config_word(dev, 0x64, 0x4);

/*
Offset 67-66 - Distributed DMA Ch 3 Base / Enable ..... RW
 15-4 Channel 3 Base Address Bits 15-4.......... default = 0
   3    Channel 3 Enable
           0 Disable ...................................................default
           1 Enable
  2-0 Reserved ........................................ always reads 0
*/
	pci_write_config_word(dev, 0x66, 0x4);

/*
Offset 77  GPIO Control 4 Control (10h)..................... RW
   7    DRQ / DACK# Pins are GPI / GPO
           0 Disable................................................... default
           1 Enable
   6    Game Port XY Pins are GPI / GPO
           0 Disable................................................... default
           1 Enable
   5    Reserved    ........................................always reads 0
   4    Internal APIC Enable
           0 Disable
           1 Enable (U10 = WSC#, V9 = APICD0, T10 =
              APICD1)................................................ default
   3    IRQ0 Output
           0 Disable................................................... default
           1 Enable IRQ0 output to GPIOC
   2    RTC Rx32 Write Protect
           0 Disable................................................... default
           1 Enable
   1    RTC Rx0D Write Protect
           0 Disable................................................... default
           1 Enable
   0    GPO13 Enable (Pin U5)
           0 Pin defined as SOE#.............................. default
           1 Pin defined as GPO13
*/
	pci_write_config_byte(dev, 0x77, 0);

/*
Offset 81  ISA Positive Decoding Control 1..................RW
 7  On-Board I/O Port Positive Decoding
      0 Disable ...................................................default
      1 Enable
 6  Microsoft-Sound System I/O Port Positive
    Decoding
      0 Disable ...................................................default
      1 Enable
5-4 Microsoft-Sound System I/O Decode Range
     00 0530h-0537h ..........................................default
     01 0604h-060Bh
     10 0E80-0E87h
     11 0F40h-0F47h
 3  APIC Positive Decoding
      0 Disable ...................................................default
      1 Enable
 2  BIOS ROM Positive Decoding
      0 Disable ...................................................default
      1 Enable
 1  Reserved ........................................ always reads 0
 0  PCS0 Positive Decoding
      0 Disable ...................................................default
      1 Enable
*/
	pci_write_config_byte(dev, 0x81, 0xc0);

/*
Offset 82  ISA Positive Decoding Control 2..................RW
   7    FDC Positive Decoding
            0 Disable ...................................................default
            1 Enable
   6    LPT Positive Decoding
            0 Disable ...................................................default
            1 Enable
  5-4 LPT Decode Range
           00 3BCh-3BFh, 7BCh-7BEh ......................default
           01 378h-37Fh, 778h-77Ah
           10 278h-27Fh, 678h-67Ah
           11 -reserved-
   3    Game Port Positive Decoding
            0 Disable ...................................................default
            1 Enable
   2    MIDI Positive Decoding
            0 Disable ...................................................default
            1 Enable
  1-0 MIDI Decode Range
           00 300h-303h ..............................................default
           01 310h-313h
           10 320h-323h
           11 330h-333h
*/
	pci_write_config_byte(dev, 0x82, 0xdc);

/*
Offset 83  ISA Positive Decoding Control 3 ................. RW
   7    COM Port B Positive Decoding
           0 Disable................................................... default
           1 Enable
  6-4 COM-Port B Decode Range
          000 3F8h-3FFh (COM1)............................ default
          001 2F8h-2FFh (COM2)
          010 220h-227h
          011 228h-22Fh
          100 238h-23Fh
          101 2E8h-2EFh (COM4)
          110 338h-33Fh
          111 3E8h-3EFh (COM3)
   3    COM Port A Positive Decoding
           0 Disable................................................... default
           1 Enable
  2-0 COM-Port A Decode Range
          000 3F8h-3FFh (COM1)............................ default
          001 2F8h-2FFh (COM2)
          010 220h-227h
          011 228h-22Fh
          100 238h-23Fh
          101 2E8h-2EFh (COM4)
          110 338h-33Fh
          111 3E8h-3EFh (COM3)
*/
	pci_write_config_byte(dev, 0x83, 0x98);

/*
Offset 84  ISA Positive Decoding Control 4 ................. RW
  7-5 Reserved        ........................................always reads 0
   4    CD: Reserved.....................................always reads 0
        CE: Port CF9 Positive Decoding
            0 Disable
            1 Enable................................................... default
   3    FDC Decoding Range
            0 Primary .................................................. default
            1 Secondary
   2    Sound Blaster Positive Decoding
            0 Disable................................................... default
            1 Enable
  1-0 Sound Blaster Decode Range
           00 220h-22Fh, 230h-233h .......................... default
           01 240h-24Fh, 250h-253h
           10 260h-26Fh, 270h-273h
           11 280h-28Fh, 290h-293h
*/
	pci_write_config_byte(dev, 0x84, 0x04);

/*
Offset 85  Extended Function Enable............................RW
  7-6 PCI Master Grant Timeout Select
           00 Disable ...................................................default
           01 32 PCI Clocks
           10 64 PCI Clocks
           11 96 PCI Clocks
   5    Keyboard Controller Configuration
            0 Disable ...................................................default
            1 Enable
   4    Function 3 USB Ports 2-3
            0 Enable ....................................................default
            1 Disable
   3    Function 6 Modem / Audio
            0 Enable ....................................................default
            1 Disable
   2    Function 5 Audio
            0 Enable ....................................................default
            1 Disable
   1    Super-I/O Configuration
            0 Disable ...................................................default
            1 Enable
   0    Super-I/O
            0 Disable ...................................................default
            1 Enable
*/
	pci_write_config_byte(dev, 0x85, 0x23);

/*
Index E0  Super-I/O Device ID (3Ch) ............................ RO
  7-0 Super-I/O ID ........................................ default = 3Ch
*/
	outb(0xe0, 0x3f0);
	if (inb(0x3f1) != 0x3c)
		printk
		    (KERN_ERR"Super-I/O Device ID not found (read 0x%x, expected 0x3c)\n",
		     inb(0x3f1));

/*
Index E2  Super-I/O Function Select (03h)...................RW
  7-5 Reserved ........................................ always reads 0
   4   Floppy Controller Enable
           0 Disable ...................................................default
           1 Enable
   3   Serial Port 2 Enable
           0 Disable ...................................................default
           1 Enable
   2   Serial Port 1 Enable
           0 Disable ...................................................default
           1 Enable
  1-0 Parallel Port Mode / Enable
          00 Unidirectional mode
          01 ECP
          10 EPP
          11 Parallel Port Disable ..............................default
*/
	outb(0xe2, 0x3f0);
	outb(0x1d, 0x3f1);

	/* Set the floppy controller address */
	outb(0xe3, 0x3f0);
	outb(0x3f0 >> 2, 0x3f1);	/* PC Legacy default is 0x3f0 */

	/* Set the LTP port address */
	outb(0xe6, 0x3f0);
	outb(0x378 >> 2, 0x3f1);	/* PC Legacy default is 0x378 */

	/* Set the Serail Port 1 port address */
	outb(0xe7, 0x3f0);
	outb(0x3f8 >> 2, 0x3f1);	/* PC Legacy default is 0x3f8 */

	/* Set the Serail Port 2 port address */
	outb(0xe8, 0x3f0);
	outb(0x2f8 >> 2, 0x3f1);	/* PC Legacy default is 0x2f8 */

/*
Index F6  Floppy Controller Configuration................. RW
  7-6 Reserved         ........................................always reads 0
   5    Floppy Drive On Parallel Port
           0 Parallel Port (SPP) Mode ...................... default
           1 FDC Mode
   4    3-Mode FDD
           0 Disable................................................... default
           1 Enable
   3    Reserved       ........................................always reads 0
   2    Four Floppy Drive Option
           0 Internal 2-Drive Decoder....................... default
           1 External 4-Drive Decoder
   1    FDC DMA Non-Burst
           0 Burst .................................................... default
           1 Non-Burst
   0    FDC Swap
           0 Disable................................................... default
           1 Enable
*/
	outb(0xf6, 0x3f0);
	outb(0x20, 0x3f1);

	printk("    Disabling   [Super-I/O Configuration]\n");
	pci_write_config_byte(dev, 0x85, 0x21);
}

static void chip_vt82c686b_usb_setup(struct pci_dev *dev)
{
	printk("Routing VT82C686B USB interrupt to legacy irq 13\n");
	pci_write_config_byte(dev, 0x3C, 80 + 13);
}

static void chip_vt82c686b_ide_setup(struct pci_dev *dev)
{
	u8 b;

	printk("Enabling VT82C686B IDE channels\n");
	pci_read_config_byte(dev, 0x40, &b);
	pci_write_config_byte(dev, 0x40, b | 3);
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x537c, chip_bridge_setup);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_PLX, 0x6520, chip_bridge_setup);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_82C686,
			chip_vt82c686b_setup);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_82C586_2,
			chip_vt82c686b_usb_setup);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_82C586_1,
			chip_vt82c686b_ide_setup);
