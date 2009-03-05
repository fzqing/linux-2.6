/*********************************************************************
 **
 * Module: V3USCDRV
 **
 * File Name: v3uscreg.h
 **
 * Authors: Phil Sikora, Jan Skibinski, Vadim Monshinsky, Dan Aizenstros
 **
 * Copyright (c) 1997-1999 V3 Semiconductor. All rights reserved.
 *
 * V3 Semiconductor makes no warranties for the use of its products.	V3 does
 * not assume any liability for errors which may appear in these files or
 * documents, however, we will attempt to notify customers of such errors.
 *
 * V3 Semiconductor retains the right to make changes to components,
 * documentation or specifications without notice.
 *
 * Please verify with V3 Semiconductor to be sure you have the latest
 * specifications before finalizing a design.
 **
 * $Revision: 1.1.1.1 $	$Date: 2002/08/28 16:11:31 $
 * $NoKeywords: $
 **
 * Description:
 **
 * This is the header file for the registers in V3 USC family of devices.
 * There are two basic approaches to accessing registers, one approach
 * has a known base address of the registers to which an offset is added,
 * for each access.
 * The second approach is to map a data structure matching the registers
 * to the physical device.  Writing and reading values from this data
 * structure has the effect of accessing the physical device's registers.
 * For convenience both methods are supported from this include file.
 *
 * Equates in this file are for the most recent stepping of the PCI
 * bridge devices, check the data sheets for older steppings.
 **
 * This include file CAN support three basic modes each of which may
 * have a little and big endian offset and/or data structures.
 * Check the _V3USCREG_H_MODE_XX_ defines determine exactly what is
 * currently supported.
 **
 * Added support for MIPS assembler, which can not handle enum statements
 * or structure definitions. 
 * Support for PMON defines that use pointers, se below table for details
 * of each mode supported by this include file.
 * A separate define enables the little endian data structures when
 * needed.
 **
 ********************************************************************/

#ifndef _V3USCREG_H_
#define _V3USCREG_H_

#ifdef __cplusplus
extern "C" {
#endif

#define V3USCREG_VER 200
#define V3USCREG_VER_STR "V3 Semiconductor (c) 1997-1999 V3USCREG Version 2.00"

/*********************************************************************
 *
 * This section is the base address and offset definition approach
 *
 ********************************************************************/

/*
 * PCI Configuration registers bit definitions
 *
 * For those bit fields larger than one bit a _MASK definition is provide
 * to isolate the value. As well a _SHIFT definition is provide to
 * convert the value to and from its register position and a number
 *
 * The table describes how Register names and their width will be encoded.
 * The DMA_LENGTHX register is a special case since it is 24 bits wide and
 * must be accessed as a 32 bit or 16 and 8 bit quantity or 16 bits with CSR.
 *
 * |--------------------------------------|
 * | Register Name | Suffix       | Width |
 * |--------------------------------------|
 * | LB_IMASK      | LB_IMASK_B   | 8     |
 * |--------------------------------------|
 * | PCI_VENDOR    | PCI_VENDOR_W | 16    |
 * |--------------------------------------|
 * | PCI_CC_REV    | PCI_CC_REV   | 32    |
 * |--------------------------------------|
 * | DMA_LENGHTX   | DMA_LENGHTX  | 24    |
 * |--------------------------------------|
 */

/*
 * Selection of which #defines to select can be determined from
 * the following table:
 **
 * #define _V3USCREG_H_CL_
 * - little endian register offsets
 * - USC base address plus register offset mode
 *   Note: The register equate is both base and offset, creating a
 *   pointer to the register in memory. (No function call)
 * Must call the v3uscreg_init_base() function to initialize the base
 * address of USC variable defined in this include file. 
 ** 
 * #define _V3USCREG_H_CB_ 
 * - big endian register offsets
 * - USC base address plus register offset mode
 *   Note: The register equate is both base and offset, creating a
 *   pointer to the register in memory. (No function call)
 * Must call the v3uscreg_init_base() function to initialize the base
 * address of USC variable defined in this include file. 
 */  

/* Mode A - Little Endian */
#ifdef _V3USCREG_H_AL_

#define V3REGW(x)		(x)
#define V3REGH(x)		(x)
#define V3REGB(x)		(x)
#endif

/* Mode A - Big Endian */
#ifdef _V3USCREG_H_AB_

#define V3REGW(x)		(x)
#define V3REGH(x)		((x)^2)
#define V3REGB(x)		((x)^3)
#endif

/* Mode B - Little Endian */
#ifdef _V3USCREG_H_BL_

#define V3REGW(x)		(x)
#define V3REGH(x)		(x)
#define V3REGB(x)		(x)
#endif

/* Mode B - Big Endian */
#ifdef _V3USCREG_H_BB_

#define V3REGW(x)		(x)
#define V3REGH(x)		((x)^2)
#define V3REGB(x)		((x)^3)
#endif


/* Mode C - Little Endian */
#ifdef _V3USCREG_H_CL_

/* To use this mode the v3uscreg_init_base() function must be */
/* called with the non-cached address of the V320USC internal registers */

extern char *_v3uscp;

#define V3REGW(x)		*(volatile unsigned long *)(_v3uscp + (x))
#define V3REGH(x)		*(volatile unsigned short *)(_v3uscp + (x))
#define V3REGB(x)		*(volatile unsigned char *)(_v3uscp + (x)) 	
#endif

/* Mode C - Big Endian */
#ifdef _V3USCREG_H_CB_

/* To use this mode the v3uscreg_init_base() function must be */
/* called with the non-cached address of the V320USC internal registers */

extern char *_v3uscp;

#define V3REGW(x)		*(volatile unsigned long *)(_v3uscp + (x))
#define V3REGH(x)		*(volatile unsigned short *)(_v3uscp + ((x)^2))
#define V3REGB(x)		*(volatile unsigned char *)(_v3uscp + ((x)^3))
#endif
 
/*
 * PCI Vendor ID
 * - Offset 00h, Size 16 bits
 */

/*
 * PCI Device ID
 * - Offset 02h, Size 16 bits
 */

/*
 * PCI Command Register
 * - Offset 04h, Size 16 bits
 */
#define PCI_CMD_W_IO_EN				0x0001		/* I/O access */
#define PCI_CMD_W_MEM_EN			0x0002		/* Memory access */
#define PCI_CMD_W_MASTER_EN			0x0004		/* PCI Master */
#define PCI_CMD_W_MWI_EN			0x0010		/* Memory Write and */
									/* Invalidate enable            */
#define PCI_CMD_W_PAR_EN			0x0040		/* Parity error */
#define PCI_CMD_W_SERR_EN			0x0100		/* System error */
									/* If PAR_EN is enabled then SERR is  */
									/* driven in response to parity error */
#define PCI_CMD_W_FBB_EN			0x0200		/* Fast back to back */
									/* transfers when Bus Master     */

/*
 * PCI Status Register
 * - Offset 06h, Size 16 bits
 */
#define PCI_STAT_W_NEW_CAP			0x0010		/* New Capabilites          */
#define PCI_STAT_W_UDF				0x0040		/* User Defined Feature     */
#define PCI_STAT_W_FAST_BACK		0x0008		/* Fast Back to Back Target */
									/* - Used to indicate ability of this   */
									/* device to other Bus Masters          */
#define PCI_STAT_W_PAR_REP			0x0010		/* Data Parity Report when    */
									/* USC is a Bus Master and PERR is driven */
#define PCI_STAT_W_DEVSEL_MASK		0x03		/* 10-9 Bits Device Select */
									/* Timing                              */
#define PCI_STAT_W_DEVSEL_SHIFT	9

#define PCI_STAT_W_T_ABORT			0x1000		/* Target Abort - set in */
									/* response to a target abort detected */
									/* while USC was a Bus Master          */
#define PCI_STAT_W_M_ABORT			0x2000		/* Master Abort - set in   */
									/* response to a master abort detected */
									/* while USC was a Bus Master          */
#define PCI_STAT_W_SYS_ERR			0x4000		/* System Error - set in */
									/* response to a system error on the */
									/* SERR pin                          */
#define PCI_STAT_W_PAR_ERR			0x8000		/* Parity Error - set in */
									/* response to a parity error on the */
									/* PCI bus                           */

/*
 * PCI Class and Revision Register
 * - Offset 08h, Size 32 bits
 */
#define PCI_CC_REV_VREV_MASK		0x0f		/* 3-0 Bits Stepping ID  */
									/* Rev A = 0,Rev B0 = 1, Rev B1 = 2, */
									/* Rev B2 = 3 */
#define PCI_CC_REV_VREV_SHIFT		0

#define PCI_CC_REV_UREV_MASK		0x0f		/* 7-4 Bits User Revision ID */
									/* user definable for system revisions   */
#define PCI_CC_REV_UREV_SHIFT		4

#define PCI_CC_REV_PROG_IF_MASK		0x0ff	/* 15-8 Bits PCI Programming */
									/* Interface code                        */
#define PCI_CC_REV_PROG_IF_SHIFT	8

#define PCI_CC_REV_SUB_CLASS_MASK	0x0ff	/* 23-16 Bits PCI Sub Class */
#define PCI_CC_REV_SUB_CLASS_SHIFT	16

#define PCI_CC_REV_BASE_CLASS_MASK	0x0ff	/* 32-24 Bits PCI Base Class */
#define PCI_CC_REV_BASE_CLASS_SHIFT 24

/*
 * PCI Header and Configuration Register
 * - Offset 0ch, Size 32 bits
 */

/* see pcivar.h */

/*
 * PCI Access to local memory map access
 * - Offset 10h, Size 32 bits (I2O mode)
 */
#define PCI_I2O_BASE_IO				0x00000001	/* I/O 1 - I/O space */
									/* 0 - Memory Space              */
#define PCI_I2O_BASE_TYPE_MASK		0x03		/* 2-1 Bits Address range */
									/* type                               */
#define PCI_I2O_BASE_TYPE_SHIFT		1			/* 0 - device can be mapped */
									/* any where in a 32 bit address space  */
#define PCI_I2O_BASE_PREFETCH		0x00000008	/* Prefetchable - no effect */
#define PCI_I2O_BASE_ADR_BASE_MASK	0x0fff	/* 31-20 Bits Base address */
									/* of ATU                              */
#define PCI_I2O_BASE_ADR_BASE_SHIFT	20

/*
 * PCI Access to local memory map access
 * - Offset 14h, Size 32 bits
 */
#define PCI_MEM_BASE_IO				0x00000001	/* I/O 1 - I/O space */
									/* 0 - Memory Space              */
#define PCI_MEM_BASE_TYPE_MASK		0x03	/* 2-1 Bits Address range */
									/* type                               */
#define PCI_MEM_BASE_TYPE_SHIFT		1			/* 0 - device can be mapped */
									/* any where in a 32 bit address space  */
#define PCI_MEM_BASE_PREFETCH		0x00000008	/* Prefetchable - no effect */
#define PCI_MEM_BASE_ADR_BASE_MASK	0x0fff	/* 31-20 Bits Base address */
									/* of ATU                              */
#define PCI_MEM_BASE_ADR_BASE_SHIFT	20

/*
 * PCI Access to Internal USC Register
 * - Offset 18h, Size 32 bits
 */
#define PCI_REG_BASE_IO				0x00000001	/* I/O 1 - I/O space */
									/* 0 - Memory Space              */
#define PCI_REG_BASE_TYPE_MASK		0x03	/* 2-1 Bits Address range */
									/* type                               */
#define PCI_REG_BASE_TYPE_SHIFT		1			/* 0 - device can be mapped */
									/* any where in a 32 bit address space  */
#define PCI_REG_BASE_PREFETCH		0x00000008	/* Prefetchable - no effect */
#define PCI_REG_BASE_ADR_BASE_MASK	0x07fffff	/* 31-20 Bits Base address */
									/* of USC registers                    */
#define PCI_REG_BASE_ADR_BASE_SHIFT	9

/*
 * PCI Base Address for Peripheral Access
 * - Offset 1ch, Size 32 bits
 */
#define PCI_PCU_BASE_IO				0x00000001	/* I/O 1 - I/O space */
									/* 0 - Memory Space              */
#define PCI_PCU_BASE_TYPE_MASK		0x03		/* 2-1 Bits Address range */
									/* type                               */
#define PCI_PCU_BASE_TYPE_SHIFT		1			/* 0 - device can be mapped */
									/* any where in a 32 bit address space  */
#define PCI_PCU_BASE_PREFETCH		0x00000008	/* Prefetchable - no effect */
#define PCI_PCU_BASE_SIZE_MASK		0x0f	/* 7-4 Bits size of aperture */
#define PCI_PCU_BASE_SIZE_SHIFT		4
#define PCI_PCU_BASE_ADR_BASE_MASK	0x0ffffff	/* 31-20 Bits Base address */
									/* of peripheral access                */
#define PCI_PCU_BASE_ADR_BASE_SHIFT	8

/*
 * PCI CARDBUS CIS Pointer
 * - Offset 28h, Size 32 bits
 */

/*
 * PCI Sub Vendor ID
 * - Offset 2ch, Size 16 bits
 * - This value must be assigned by PCI SIG
 */

/*
 * PCI Sub System ID
 * - Offset 2eh, Size 16 bits
 * - This value is managed by the vendor
 */

/*
 * PCI Read Only Memory Register
 * - Offset 30h, Size 32 bits
 */
#define PCI_ROM_BASE_ENABLE			0x00000001	/* Expansion ROM enable */
#define PCI_ROM_BASE_PREFETCH		0x00000008	/* Prefetchable enable  */
#define PCI_ROM_BASE_SIZE_MASK		0x03	/* 5-4 Bits Aperture size */
#define PCI_ROM_BASE_SIZE_SHIFT		4
#define PCI_ROM_BASE_MAP_MASK		0x0ff	/* 15-8 Bits Map address */
#define PCI_ROM_BASE_MAP_SHIFT		8
#define PCI_ROM_BASE_ADR_BASE_MASK	0x0ffff	/* 31-16 Bits Base address */
#define PCI_ROM_BASE_ADR_BASE_SHIFT	16

/*
 * PCI Capablities
 * - Offset 34h, Size 8 bits
 */

/*
 * PCI Bus Parameters Register
 * - Offset 3ch, Size 32 bits
 */

/* see pcireg.h */

/*
 * PCI I2O Map Register
 * - Offset 50h, Size 32 bits
 */
#define PCI_I2O_MAP_ENABLE			0x00000001	/* Enable aperture */
#define PCI_I2O_MAP_REG_EN			0x00000002	/* Register enable */
#define PCI_I2O_MAP_I2O_MODE		0x00000004	/* I2O Mode enable */
#define PCI_I2O_MAP_RD_POST_INH		0x00000008
#define PCI_I2O_MAP_SIZE_1MB		0x00000000
#define PCI_I2O_MAP_SIZE_2MB		0x00000010
#define PCI_I2O_MAP_SIZE_4MB		0x00000020
#define PCI_I2O_MAP_SIZE_8MB		0x00000030
#define PCI_I2O_MAP_SIZE_16MB		0x00000040
#define PCI_I2O_MAP_SIZE_32MB		0x00000050
#define PCI_I2O_MAP_SIZE_64MB		0x00000060
#define PCI_I2O_MAP_SIZE_128MB		0x00000070
#define PCI_I2O_MAP_SIZE_256MB		0x00000080
#define PCI_I2O_MAP_SIZE_512MB		0x00000090
#define PCI_I2O_MAP_SIZE_1GB		0x000000a0
#define PCI_I2O_BYTE_SWAP_NO		0x00000000	/* No swap 32 bits */
#define PCI_I2O_BYTE_SWAP_16		0x00000100	/* 16 bits */
#define PCI_I2O_BYTE_SWAP_8			0x00000200	/* bits */
#define PCI_I2O_BYTE_SWAP_AUTO		0x00000300	/* Auto swap use BE[3:0]   */
#define PCI_I2O_PCI_RD_MB_00		0x00000000
#define PCI_I2O_PCI_RD_MB_01		0x00001000
#define PCI_I2O_PCI_RD_MB_10		0x00002000
#define PCI_I2O_PCI_WR_MB_00		0x00000000
#define PCI_I2O_PCI_WR_MB_01		0x00004000
#define PCI_I2O_PCI_WR_MB_10		0x00008000
#define PCI_I2O_W_FLUSH				0x00010000	/* write prefetch reads */
#define PCI_I2O_MAP_ADR_MASK		0x03ff
#define PCI_I2O_MAP_ADR_SHIFT		20

/*
 * PCI MEM Map Register
 * - Offset 54h, Size 32 bits
 */
#define PCI_MEM_MAP_ENABLE			0x00000001	/* Enable aperture */
#define PCI_MEM_MAP_REG_EN			0x00000002	/* Register enable */
#define PCI_MEM_MAP_I2O_MODE		0x00000004	/* I2O Mode enable */
#define PCI_MEM_MAP_RD_POST_INH		0x00000008
#define PCI_MEM_MAP_SIZE_1MB		0x00000000
#define PCI_MEM_MAP_SIZE_2MB		0x00000010
#define PCI_MEM_MAP_SIZE_4MB		0x00000020
#define PCI_MEM_MAP_SIZE_8MB		0x00000030
#define PCI_MEM_MAP_SIZE_16MB		0x00000040
#define PCI_MEM_MAP_SIZE_32MB		0x00000050
#define PCI_MEM_MAP_SIZE_64MB		0x00000060
#define PCI_MEM_MAP_SIZE_128MB		0x00000070
#define PCI_MEM_MAP_SIZE_256MB		0x00000080
#define PCI_MEM_MAP_SIZE_512MB		0x00000090
#define PCI_MEM_MAP_SIZE_1GB		0x000000a0
#define PCI_MEM_BYTE_SWAP_NO		0x00000000	/* No swap 32 bits */
#define PCI_MEM_BYTE_SWAP_16		0x00000100	/* 16 bits */
#define PCI_MEM_BYTE_SWAP_8			0x00000200	/* bits */
#define PCI_MEM_BYTE_SWAP_AUTO		0x00000300	/* Auto swap use BE[3:0]   */
#define PCI_MEM_PCI_RD_MB_00		0x00000000
#define PCI_MEM_PCI_RD_MB_01		0x00001000
#define PCI_MEM_PCI_RD_MB_10		0x00002000
#define PCI_MEM_PCI_WR_MB_00		0x00000000
#define PCI_MEM_PCI_WR_MB_01		0x00004000
#define PCI_MEM_PCI_WR_MB_10		0x00008000
#define PCI_MEM_W_FLUSH				0x00010000	/* write prefetch reads */
#define PCI_MEM_MAP_ADR_MASK		0x03ff
#define PCI_MEM_MAP_ADR_SHIFT		20

/*
 * PCI BUS CFG Register
 * -5ch Offset 5ch, Size 32 bits
 */
#define PCI_BUS_CFG_CFG_RETRY		0x00000100
#define PCI_BUS_CFG_PCI_INH			0x00000200
#define PCI_BUS_CFG_I2O_ONLINE		0x00003000
#define PCI_BUS_CFG_I2O_EN			0x00004000
#define PCI_BUS_CFG_I2O_EN_EN		0x00008000
#define PCI_BUS_CFG_PBRST_MAX_4		0x00000000
#define PCI_BUS_CFG_PBRST_MAX_8		0x00010000
#define PCI_BUS_CFG_PBRST_MAX_16	0x00020000
#define PCI_BUS_CFG_PBRST_MAX_256	0x00030000
#define PCI_BUS_CFG_TRDY_STOP		0x00100000
#define PCI_BUS_CFG_PCU_SWAP_00		0x00000000
#define PCI_BUS_CFG_PCU_SWAP_01		0x01000000
#define PCI_BUS_CFG_PCU_SWAP_10		0x02000000
#define PCI_BUS_CFG_PCU_SWAP_11		0x03000000
#define PCI_BUS_CFG_FAST_SCL		0x80000000

/*
 * LB_PCI_BASEx Registers
 * - Offset 60h, Size 32 bits
 * - Offset 64h, Size 32 bits
 */
#define LB_PCI_BASEX_ALOW_MASK		0x03	/* select value AD1:0 */
#define LB_PCI_BASEX_ALOW_SHIFT		0x00000000
#define LB_PCI_BASEX_ERR_EN			0x00000004	
#define LB_PCI_BASEX_PREFETCH		0x00000008	/* prefetch */
#define LB_PCI_BASEX_SIZE_DISABLE	0x00000000
#define LB_PCI_BASEX_SIZE_16MB		0x00000010
#define LB_PCI_BASEX_SIZE_32MB		0x00000020
#define LB_PCI_BASEX_SIZE_64MB		0x00000030
#define LB_PCI_BASEX_SIZE_128MB		0x00000040
#define LB_PCI_BASEX_SIZE_256MB		0x00000050
#define LB_PCI_BASEX_SIZE_512MB		0x00000060
#define LB_PCI_BASEX_SIZE_1GB		0x00000070
#define LB_PCI_BASEX_BYTE_SWAP_NO	0x00000000	/* No swap 32 bits */
#define LB_PCI_BASEX_BYTE_SWAP_16	0x00000100	/* 16 bits */
#define LB_PCI_BASEX_BYTE_SWAP_8	0x00000200	/* bits */
#define LB_PCI_BASEX_BYTE_SWAP_AUTO	0x00000300	/* Auto swap use BE[3:0]   */
#define LB_PCI_BASEX_COMBINE		0x00000800	/* Burst Write Combine */

#define LB_PCI_BASEX_PCI_CMD_MASK	0x07
#define LB_PCI_BASEX_PCI_CMD_SHIFT	13
#define LB_PCI_BASEX_INT_ACK		0x00000000	/* Interrupt Ack */
#define LB_PCI_BASEX_IO				0x00002000	/* I/O Read/Write */
#define LB_PCI_BASEX_MEMORY			0x00006000	/* Memory Read/Write */
#define LB_PCI_BASEX_CONFIG			0x0000a000	/* Configuration Read/Write */
#define LB_PCI_BASEX_MULTI_MEMORY	0x0000c000	/* Multiple Memory Read/Write */
#define LB_PCI_BASEX_MEMORY_INVALIDATE	0x0000e000	/* Multiple Memory Read/e */
												/* Write Invalidate           */
#define LB_PCI_BASEX_MAP_ADR_MASK	0x0ff	/* PCI Address map */
#define LB_PCI_BASEX_MAP_ADR_SHIFT	16
#define LB_PCI_BASEX_BASE			0xff000000	/* Local Address base */
#define LB_PCI_BASEX_BASE_ADR_SHIFT	24


/*
 * System Register
 * - Offset 73h, Size 8 bits
 */
#define SYSTEM_B_SPROM_EN			0x01		/* 1 - Software control     */
									/* 0 - Hardware control                 */
#define SYSTEM_B_SDA_IN				0x02		/* Serial EEPROM data input */
#define SYSTEM_B_SDA_IN_SHIFT		1
#define SYSTEM_B_SDA_OUT			0x04		/* Serial EEPROM data output */
									/* SPROM_EN must be enabled              */
#define SYSTEM_B_SCL				0x08		/* Serial EEPROM clock output */
#define SYSTEM_B_LOO_EN				0x10		/* Hot swap LED control       */
#define SYSTEM_B_CFG_LOCK			0x20		/* Configuration Lock         */
#define SYSTEM_B_LOCK				0x40		/* Lock Register Contents set */										/* to 1 the contents become unwritable    */
									/* Clear lock by writing 0xa5 Writing     */									/* 0xa5 will not overwrite the current    */									/* system status values                   */
#define SYSTEM_B_UNLOCK_TOKEN		0xa5
#define SYSTEM_B_RST_OUT			0x80		/* Reset output control */

/*
 * SDRAM Local Base Address Register 
 * - Offset 78h, Size 32 bits
 */
#define LB_SDRAM_BASE_ENABLE		0x1			/* must be enabled to access */
#define LB_SDRAM_BASE_SIZE_64M		0x0
#define LB_SDRAM_BASE_SIZE_128M		0x10
#define LB_SDRAM_BASE_SIZE_256M		0x20
#define LB_SDRAM_BASE_SIZE_512M		0x30
#define LB_SDRAM_BASE_SIZE_1G		0x40

#define LB_SDRAM_BASE_MASK			0x03f
#define LB_SDRAM_BASE_SHIFT			26

/*
 * Local Bus aperture control 
 * - Offset 84h, Size 16 bits
 */

#define PCI_CNT_WR_PCI_SHIFT				14
#define PCI_CNT_WR_PCI_FIFO_00			(0x0 << PCI_CNT_WR_PCI_SHIFT)
#define PCI_CNT_WR_PCI_FIFO_01			(0x1 << PCI_CNT_WR_PCI_SHIFT)
#define PCI_CNT_WR_PCI_FIFO_10			(0x2 << PCI_CNT_WR_PCI_SHIFT)
#define PCI_CNT_WR_PCI_FIFO_11			(0x3 << PCI_CNT_WR_PCI_SHIFT)

#define PCI_CNT_RD_PCI_SHIFT				8
#define PCI_CNT_RD_PCI_FIFO_00			(0x0 << PCI_CNT_WR_PCI_SHIFT)
#define PCI_CNT_RD_PCI_FIFO_01			(0x1 << PCI_CNT_WR_PCI_SHIFT)
#define PCI_CNT_RD_PCI_FIFO_10			(0x2 << PCI_CNT_WR_PCI_SHIFT)
#define PCI_CNT_RD_PCI_FIFO_11			(0x3 << PCI_CNT_WR_PCI_SHIFT)

#define PCI_CNT_ONE_RD_BUF_SHIFT			7

#define PCI_CNT_WFLUSH1_SHIFT				2
#define PCI_CNT_WFLUSH1_00				(0x0 << PCI_CNT_WFLUSH1_SHIFT)
#define PCI_CNT_WFLUSH1_01				(0x1 << PCI_CNT_WFLUSH1_SHIFT)
#define PCI_CNT_WFLUSH1_10				(0x2 << PCI_CNT_WFLUSH1_SHIFT)
#define PCI_CNT_WFLUSH1_11				(0x3 << PCI_CNT_WFLUSH1_SHIFT)

#define PCI_CNT_WFLUSH0_SHIFT				0
#define PCI_CNT_WFLUSH0_00				(0x0 << PCI_CNT_WFLUSH0_SHIFT)
#define PCI_CNT_WFLUSH0_01				(0x1 << PCI_CNT_WFLUSH0_SHIFT)
#define PCI_CNT_WFLUSH0_10				(0x2 << PCI_CNT_WFLUSH0_SHIFT)
#define PCI_CNT_WFLUSH0_11				(0x3 << PCI_CNT_WFLUSH0_SHIFT)




/*
 * SDRAM Timing Parameters
 * - Offset 8ch, Size 32 bits
 */
#define SDRAM_CFG_TCAS_RD_1			0x0			/* CAS latency */
#define SDRAM_CFG_TCAS_RD_2			0x1			/* comment is cycles */
#define SDRAM_CFG_TCAS_RD_3			0x2

#define SDRAM_CFG_RCD_1				0x0			/* RAS to CAS delay */
#define SDRAM_CFG_RCD_2				0x40		/* comment is cycles */
#define SDRAM_CFG_RCD_3				0x80

#define SDRAM_CFG_RP_1				0x0			/* RAS precharge */
#define SDRAM_CFG_RP_2				0x100		/* comment is cycles */
#define SDRAM_CFG_RP_3				0x200
#define SDRAM_CFG_RP_4				0x300

#define SDRAM_CFG_RAS_2				0x0			/* RAS pulse width */
#define SDRAM_CFG_RAS_3				0x400		/* comment is cycles */
#define SDRAM_CFG_RAS_4				0x800
#define SDRAM_CFG_RAS_5				0xc00

#define SDRAM_CFG_DPL_1				0x0			/* Last data write precharge */
#define SDRAM_CFG_DPL_2				0x1000		/* comment is cycles */

#define SDRAM_CFG_REF_SCALE_MASK	0x01
#define SDRAM_CFG_REF_SCALE_SHIFT	21
#define SDRAM_CFG_REF_NDIV_MASK		0x03f
#define SDRAM_CFG_REF_NDIV_SHIFT	22

/*
 * SDRAM Command Register
 * - Offset 8ah, Size 16 bits
 */
#define SDRAM_CMD_W_CMD_IPR			0x1			/* initiate command */

#define SDRAM_CMD_W_CS_SEL_0		0x2			/* Chip select bank 0 */
#define SDRAM_CMD_W_CS_SEL_1		0x4			/* Chip select bank 1 */
#define SDRAM_CMD_W_CS_SEL_2		0x8			/* Chip select bank 2 */
#define SDRAM_CMD_W_CS_SEL_3		0x10		/* Chip select bank 3 */

#define SDRAM_CMD_W_NOOP			0x0			/* SDRAM Commands */
#define SDRAM_CMD_W_BURST_TERM		0x20
#define SDRAM_CMD_W_READ			0x40
#define SDRAM_CMD_W_WRITE			0x60
#define SDRAM_CMD_W_BANK_ACTIVATE	0x80
#define SDRAM_CMD_W_PRECHARGE		0xa0
#define SDRAM_CMD_W_AUTO_REFRESH	0xc0
#define SDRAM_CMD_W_MODE_REGISTER	0xe0

/*
 * SDRAM Block Control Register
 * - Offset 90h, Size 32 bits
 * - Offset 94h, Size 32 bits
 * - Offset 98h, Size 32 bits
 * - Offset 9ch, Size 32 bits
 */
#define SDRAM_BANKX_ENABLE			0x1			/* Enable Bank */
#define SDRAM_BANKX_READ_ONLY		0x2			/* Read only */
#define SDRAM_BANKX_SIZE_512K		0x0
#define SDRAM_BANKX_SIZE_1M			0x10
#define SDRAM_BANKX_SIZE_2M			0x20
#define SDRAM_BANKX_SIZE_4M			0x30
#define SDRAM_BANKX_SIZE_8M			0x40
#define SDRAM_BANKX_SIZE_16M		0x50
#define SDRAM_BANKX_SIZE_32M		0x60
#define SDRAM_BANKX_SIZE_64M		0x70
#define SDRAM_BANKX_SIZE_128M		0x80
#define SDRAM_BANKX_SIZE_256M		0x90

#define SDRAM_BANKX_ROW_MUX_MODE_0	0x0
#define SDRAM_BANKX_ROW_MUX_MODE_1	0x100
#define SDRAM_BANKX_ROW_MUX_MODE_2	0x200
#define SDRAM_BANKX_ROW_MUX_MODE_3	0x300
#define SDRAM_BANKX_ROW_MUX_MODE_4	0x400
#define SDRAM_BANKX_ROW_MUX_MODE_5	0x500
#define SDRAM_BANKX_ROW_MUX_MODE_6	0x600
#define SDRAM_BANKX_ROW_MUX_MODE_7	0x700
#define SDRAM_BANKX_ROW_MUX_MODE_8	0x800
#define SDRAM_BANKX_ROW_MUX_MODE_9	0x900
#define SDRAM_BANKX_ROW_MUX_MODE_A	0xa00
#define SDRAM_BANKX_ROW_MUX_MODE_B	0xb00
#define SDRAM_BANKX_ROW_MUX_MODE_C	0xc00

#define SDRAM_BANKX_COL_MUX_MODE_0	0x0
#define SDRAM_BANKX_COL_MUX_MODE_1	0x1000
#define SDRAM_BANKX_COL_MUX_MODE_2	0x2000
#define SDRAM_BANKX_COL_MUX_MODE_3	0x3000
#define SDRAM_BANKX_COL_MUX_MODE_4	0x4000
#define SDRAM_BANKX_COL_MUX_MODE_5	0x5000
#define SDRAM_BANKX_COL_MUX_MODE_6	0x6000
#define SDRAM_BANKX_COL_MUX_MODE_7	0x7000

#define SDRAM_BANKX_OFFSET_MASK		0x07ff
#define SDRAM_BANKX_OFFSET_SHIFT	19


/*
 * Interrupt Configuration Register
 * - Offset e0h, Size 32 bits
 * - Offset e4h, Size 32 bits
 * - Offset e8h, Size 32 bits
 * - Offset 158h, Size 32 bits
 */
#define INT_CFGX_LB_MBI				0x00000001
#define INT_CFGX_PCI_MBI			0x00000002
#define INT_CFGX_I2O_OP_NE			0x00000008
#define INT_CFGX_I2O_IF_NF			0x00000010
#define INT_CFGX_I2O_IP_NE			0x00000020
#define INT_CFGX_I2O_OP_NF			0x00000040
#define INT_CFGX_I2O_OF_NE			0x00000080
#define INT_CFGX_INT0				0x00000100
#define INT_CFGX_INT1				0x00000200
#define INT_CFGX_INT2				0x00000400
#define INT_CFGX_INT3				0x00000800
#define INT_CFGX_TIMER0				0x00001000
#define INT_CFGX_TIMER1				0x00002000
#define INT_CFGX_ENUM				0x00004000
#define INT_CFGX_DMA0				0x00010000
#define INT_CFGX_DMA1				0x00020000
#define INT_CFGX_PWR_STATE			0x00100000
#define INT_CFGX_HBI				0x00200000
#define INT_CFGX_WDI				0x00400000
#define INT_CFGX_BWI				0x00800000
#define INT_CFGX_PSLAVE_PI			0x01000000
#define INT_CFGX_PMASTER_PI			0x02000000
#define INT_CFGX_PCI_T_ABORT		0x04000000
#define INT_CFGX_PCI_M_ABORT		0x08000000
#define INT_CFGX_DRA_PI				0x10000000
#define INT_CFGX_MODE				0x20000000
#define INT_CFGX_DI0				0x40000000
#define INT_CFGX_DI1				0x80000000


/*
 * General Purpose Timer Control Register
 * - Offset 150h, Size 16 bits
 * - Offset 152h, Size 16 bits
 */
#define TIMER_CTLX_W_TI_MODE_0		0x0000		/* Timer input event */
#define TIMER_CTLX_W_TI_MODE_1		0x0001
#define TIMER_CTLX_W_TI_MODE_2		0x0002
#define TIMER_CTLX_W_TI_MODE_3		0x0003

#define TIMER_CTLX_W_CNT_EN_0		0x0000		/* Count enable */
#define TIMER_CTLX_W_CNT_EN_1		0x0004
#define TIMER_CTLX_W_CNT_EN_2		0x0008
#define TIMER_CTLX_W_CNT_EN_3		0x000C

#define TIMER_CTLX_W_TRG_MODE_0		0x0000		/* Trigger mode */
#define TIMER_CTLX_W_TRG_MODE_1		0x0010
#define TIMER_CTLX_W_TRG_MODE_2		0x0020
#define TIMER_CTLX_W_TRG_MODE_3		0x0030

#define TIMER_CTLX_W_TO_MODE_0		0x0000		/* Timer output mode */
#define TIMER_CTLX_W_TO_MODE_1		0x0100
#define TIMER_CTLX_W_TO_MODE_2		0x0200
#define TIMER_CTLX_W_TO_MODE_3		0x0300
#define TIMER_CTLX_W_TO_MODE_4		0x0400
#define TIMER_CTLX_W_TO_MODE_5		0x0500

#define TIMER_CTLX_W_DLTCH_0		0x0000		/* Data latch mode */
#define TIMER_CTLX_W_DLTCH_1		0x0800
#define TIMER_CTLX_W_DLTCH_2		0x1000

#define TIMER_CTLX_W_ENABLE			0x8000		/* Timer enable */


/*
 * DMA Delay Register
 * - Offset 16Ch, Size 8 bits
 */
#define DMA_DELAY_MASK		0x0ff	
#define DMA_DELAY_SHIFT		0

/*
 * DMA Command / Status Register
 * - Offset 170h, Size 32 bits
 * - Offset 174h, Size 32 bits
 */
#define DMA_CSR_IPR				0x00000001	/* initiate DMA transfer */
#define DMA_CSR_HALT			0x00000002	/* pause DMA transfer */
#define DMA_CSR_DONE			0x00000004	/* DMA transfer complete */
#define DMA_CSR_DCI				0x00000008	/* DMA control interrupt status	*/
#define DMA_CSR_DPE				0x00000010	/* DMA PCI BUS error status */
#define DMA_CSR_DONE_EN			0x00000400	/* DONE interrupt enable */
#define DMA_CSR_DCI_EN			0x00000800	/* DCI interrupt enable */
#define DMA_CSR_DPE_EN			0x00001000	/* DPE interrupt enable */
#define DMA_CSR_PRIORITY		0x00008000	/* DMA channel priority */
#define DMA_CSR_PCI_CMD0_MASK	0x07	/* PCI Command Type 0 */
#define DMA_CSR_PCI_CMD0_SHIFT	17
#define DMA_CSR_PCI_CMD1_MASK	0x07	/* PCI Command Type 1 */
#define DMA_CSR_PCI_CMD1_SHIFT	21
 
/*
 * DMA Transfer Control Register
 * - Offset 180h, Size 32 bits
 * - Offset 190h, Size 32 bits
 */
#define DMA_XFER_DMA_CNT_MASK	0x0FFFFF	/* DMA transfer count */
#define DMA_XFER_DMA_CNT_SHIFT	0
#define DMA_XFER_DTERM_EN		0x00400000	/* External terminate count enable */
#define DMA_XFER_BLOCK_FILL		0x00800000	/* Block fill feature enable */
#define DMA_XFER_DST_BUS		0x01000000	/* DMA destination BUS */
#define DMA_XFER_SRC_BUS		0x02000000	/* DMA source BUS */
#define DMA_XFER_PDST_TYPE		0x04000000	/* PCI destination command type */
#define DMA_XFER_PSRC_TYPE		0x08000000	/* PCI source command type */
#define DMA_XFER_SWAP_MASK		0x03	/* Byte swap control */
#define DMA_XFER_SWAP_SHIFT		28
#define DMA_XFER_UPDT_CNT		0x40000000	/* Update count */
#define DMA_XFER_DREQ_EN		0x80000000	/* External DRQ enable */


/*
 * DMA Control Block Register
 * - Offset 180h, Size 32 bits
 * - Offset 190h, Size 32 bits
 */
#define DMA_CTLB_BUS			0x00000001	/* DMA Control block address space */
#define DMA_CTLB_SA_INC_DIS		0x00000004	/* Source BUS address increment disable */
#define DMA_CTLB_DA_INC_DIS		0x00000008	/* Dest BUS address increment disable */
#define DMA_CTLB_ADDR_MASK		0x0FFFFFFF	/* DMA Control block address mask */
#define DMA_CTLB_ADDR_SHIFT		4


/*
 * PCI Configuration registers offsets
 */
#define	V3USC_PCI_VENDOR_W			V3REGH(0x00)
#define	V3USC_PCI_DEVICE_W			V3REGH(0x02)
#define	V3USC_PCI_CMD_W		 		V3REGH(0x04)
#define	V3USC_PCI_STAT_W			V3REGH(0x06)
#define	V3USC_PCI_CC_REV			V3REGW(0x08)
#define	V3USC_PCI_HDR_CFG	 		V3REGW(0x0c)
#define	V3USC_PCI_I2O_BASE	 		V3REGW(0x10)
#define	V3USC_PCI_MEM_BASE	 		V3REGW(0x14)
#define	V3USC_PCI_REG_BASE			V3REGW(0x18)
#define	V3USC_PCI_PCU_BASE			V3REGW(0x1c)
#define	V3USC_PCI_CIS				V3REGW(0x28)
#define	V3USC_PCI_SUB_VENDOR_W		V3REGH(0x2c)
#define	V3USC_PCI_SUB_ID_W			V3REGH(0x2e)
#define	V3USC_PCI_ROM_BASE	 		V3REGW(0x30)
#define	V3USC_PCI_CAPABLITY_B 		V3REGB(0x34)
#define	V3USC_PCI_BPARM				V3REGW(0x3c)
#define	V3USC_PM_CAP_ID_B			V3REGB(0x40)
#define	V3USC_PM_NEXT_ID_B			V3REGB(0x41)
#define	V3USC_PM_CAP_W				V3REGH(0x42)
#define	V3USC_PM_CRS_W				V3REGH(0x44)
#define	V3USC_PM_DATA_B				V3REGB(0x47)
#define	V3USC_PM_PWR_CON			V3REGW(0x48)
#define	V3USC_PM_PWR_DIS			V3REGW(0x4c)
#define	V3USC_PCI_I2O_MAP			V3REGW(0x50)
#define	V3USC_PCI_MEM_MAP			V3REGW(0x54)
#define	V3USC_PCI_BUS_CFG			V3REGW(0x5c)
#define	V3USC_LB_PCI_BASE0			V3REGW(0x60)
#define	V3USC_LB_PCI_BASE1			V3REGW(0x64)
#define	V3USC_LB_PCU_BASE			V3REGW(0x6c)
#define	V3USC_LB_REG_BASE_W			V3REGH(0x70)
#define	V3USC_SYSTEM_B				V3REGB(0x73)
#define	V3USC_LB_ROM_BASE			V3REGW(0x74)
#define	V3USC_LB_SDRAM_BASE			V3REGW(0x78)
#define	V3USC_LB_BUS_CFG			V3REGW(0x7c)
#define	V3USC_HS_CAP_ID_B			V3REGB(0x80)
#define	V3USC_HS_NEXT_ID_B			V3REGB(0x81)
#define	V3USC_HS_CSR_B				V3REGB(0x82)
#define	V3USC_LB_PCI_CTL_W			V3REGH(0x84)
#define	V3USC_T_CY_B				V3REGB(0x87)
#define	V3USC_SDRAM_MA_W			V3REGH(0x88)
#define	V3USC_SDRAM_CMD_W			V3REGH(0x8a)
#define	V3USC_SDRAM_CFG				V3REGW(0x8c)
#define	V3USC_SDRAM_BANK0			V3REGW(0x90)
#define	V3USC_SDRAM_BANK1			V3REGW(0x94)
#define	V3USC_SDRAM_BANK2			V3REGW(0x98)
#define	V3USC_SDRAM_BANK3			V3REGW(0x9c)
#define	V3USC_PCU_SUB0				V3REGW(0xa0)
#define	V3USC_PCU_SUB1				V3REGW(0xa4)
#define	V3USC_PCU_SUB2				V3REGW(0xa8)
#define	V3USC_PCU_TC_WR0			V3REGW(0xb0)
#define	V3USC_PCU_TC_WR1			V3REGW(0xb4)
#define	V3USC_PCU_TC_WR2			V3REGW(0xb8)
#define	V3USC_PCU_TC_WR3			V3REGW(0xbc)
#define	V3USC_PCU_TC_WR4			V3REGW(0xc0)
#define	V3USC_PCU_TC_RD0			V3REGW(0xc8)
#define	V3USC_PCU_TC_RD1			V3REGW(0xcc)
#define	V3USC_PCU_TC_RD2			V3REGW(0xd0)
#define	V3USC_PCU_TC_RD3			V3REGW(0xd4)
#define	V3USC_PCU_TC_RD4			V3REGW(0xd8)
#define	V3USC_INT_CFG0				V3REGW(0xe0)
#define	V3USC_INT_CFG1				V3REGW(0xe4)
#define	V3USC_INT_CFG2				V3REGW(0xe8)
#define	V3USC_INT_STAT				V3REGW(0xec)
#define	V3USC_IOS					V3REGW(0xf0)
#define	V3USC_WD_HBI_W				V3REGH(0xf4)
#define	V3USC_MAIL_WR_STAT_B		V3REGB(0xf8)
#define	V3USC_MAIL_RD_STAT_B		V3REGB(0xf9)
#define	V3USC_PCI_MAIL_IEWR_B		V3REGB(0xfc)
#define	V3USC_PCI_MAIL_IERD_B		V3REGB(0xfd)
#define	V3USC_LB_MAIL_IEWR_B		V3REGB(0xfe)
#define	V3USC_LB_MAIL_IERD_B		V3REGB(0xff)
#define	V3USC_MAIL_DATA0_B			V3REGB(0x100)
#define	V3USC_MAIL_DATA1_B			V3REGB(0x104)
#define	V3USC_MAIL_DATA2_B			V3REGB(0x108)
#define	V3USC_MAIL_DATA3_B			V3REGB(0x10c)
#define	V3USC_MAIL_DATA4_B			V3REGB(0x110)
#define	V3USC_MAIL_DATA5_B			V3REGB(0x114)
#define	V3USC_MAIL_DATA6_B			V3REGB(0x118)
#define	V3USC_MAIL_DATA7_B			V3REGB(0x11c)
#define	V3USC_TIMER_DATA0			V3REGW(0x140)
#define	V3USC_TIMER_DATA1			V3REGW(0x144)
#define	V3USC_TIMER_CTL0_W			V3REGH(0x150)
#define	V3USC_TIMER_CTL1_W			V3REGH(0x152)
#define	V3USC_INT_CFG3				V3REGW(0x158)
#define	V3USC_I2O_ISTAT				V3REGW(0x160)
#define	V3USC_I2O_IMASK				V3REGW(0x164)
#define	V3USC_DMA_DELAY				V3REGW(0x16C)
#define	V3USC_DMA_CSR0				V3REGW(0x170)
#define	V3USC_DMA_CSR1				V3REGW(0x174)
#define	V3USC_DMA_XFER_CTL0			V3REGW(0x180)
#define	V3USC_DMA_SRC_ADR0			V3REGW(0x184)
#define	V3USC_DMA_DST_ADR0			V3REGW(0x188)
#define	V3USC_DMA_CTLB_ADR0			V3REGW(0x18C)
#define	V3USC_DMA_XFER_CTL1			V3REGW(0x190)
#define	V3USC_DMA_SRC_ADR1			V3REGW(0x194)
#define	V3USC_DMA_DST_ADR1			V3REGW(0x198)
#define	V3USC_DMA_CTLB_ADR1			V3REGW(0x19C)

/*
 * I2O Mode offsets 
 */
#define	V3USC_IFL					V3USC_MAIL_DATA0_B
#define	V3USC_IPL					V3USC_MAIL_DATA2_B
#define	V3USC_OPL					V3USC_MAIL_DATA4_B
#define	V3USC_OFL					V3USC_MAIL_DATA6_B

/*********************************************************************
 *
 * This section defines some common tables by the registers
 *
 ********************************************************************/

/*
 * PCI Vendor ID
 * - Offset 00h, Size 16 bits
 */
#define	V3_VENDOR_ID				0x11b0		/* V3 PCI Vendor ID */

/*
 * PCI Device ID
 * - Offset 02h, Size 16 bits
 */
#define V960PBC						0x1			/* V3 Device ID for V960PBC */
#define V961PBC						0x2			/* V3 Device ID for V961PBC */
#define V962PBC						0x4			/* V3 Device ID for V962PBC */
#define V292PBC						0x10		/* V3 Device ID for V292PBC */
												/* MIPS MODE */ 
#define V320USC						0x100		/* V3 Device ID for V320USC */
												/* MIPS mode 9 bit SYSCMD   */ 
#define V320USC_5					0x101		/* V3 Device ID for V320USC */
												/* MIPS mode 5 bit SYSCMD   */ 
#define V320USC_SH3					0x102		/* V3 Device ID for V320USC */
												/* SH3 mode */ 
#define V320USC_SH4					0x103		/* V3 Device ID for V320USC */
												/* SH4 mode */ 
#define V370PDC						0x200		/* V3 Device ID for V370PDC */

/*
 * Stepping of V3USC320 as read back from V3USC_PCI_CC_REV register
 */
#define V3USC_REV_A0 0
#define V3USC_REV_B0 1
#define V3USC_REV_B1 2

/*
 * PCI Bus Parameters Register
 * - Offset 3ch, Size 32 bits
 */
#define INTERRUPT_PIN_DISABLE		0x0			/* Disabled */
#define INTERRUPT_PIN_INTA			0x1			/* Use INTA */
#define INTERRUPT_PIN_INTB			0x2			/* Use INTA */
#define INTERRUPT_PIN_INTC			0x3			/* Use INTA */
#define INTERRUPT_PIN_INTD			0x4			/* Use INTA */

/*
 * PCI Base Address for Peripheral Access 
 * - Offset 1ch, Size 32 bits
 * PCI Intelligent I/O Address Translation Unit Local Bus Address Map Register
 * - Offset ??h, Size 32 bits
 */
#define BYTE_SWAP_NO				0x0			/* No swap 32 bits */
#define BYTE_SWAP_16				0x1			/* 16 bits */
#define BYTE_SWAP_8					0x2			/* 8 bits */
#define BYTE_SWAP_AUTO				0x3			/* Auto swap use BE[3:0]   */
#define APERTURE_SIZE_1M			0x0			/* Aperture size of 1 MB   */
#define APERTURE_SIZE_2M			0x1			/* Aperture size of 2 MB   */
#define APERTURE_SIZE_4M			0x2			/* Aperture size of 4 MB   */
#define APERTURE_SIZE_8M			0x3			/* Aperture size of 8 MB   */
#define APERTURE_SIZE_16M			0x4			/* Aperture size of 16 MB  */
#define APERTURE_SIZE_32M			0x5			/* Aperture size of 32 MB  */
#define APERTURE_SIZE_64M			0x6			/* Aperture size of 64 MB  */
#define APERTURE_SIZE_128M			0x7			/* Aperture size of 128 MB */
#define APERTURE_SIZE_256M			0x8			/* Aperture size of 256 MB */
#define APERTURE_SIZE_512M			0x9			/* Aperture size of 512 MB */
#define APERTURE_SIZE_1G			0xa			/* Aperture size of 1 GB   */

/*
 * Mail box registers short equates
 */
#define EN0							0x0001		/* mailbox bit 0 */
#define EN1							0x0002		/* mailbox bit 1 */
#define EN2							0x0004		/* mailbox bit 2 */
#define EN3							0x0008		/* mailbox bit 3 */
#define EN4							0x0010		/* mailbox bit 4 */
#define EN5							0x0020		/* mailbox bit 5 */
#define EN6							0x0040		/* mailbox bit 6 */
#define EN7							0x0080		/* mailbox bit 7 */

/*
 * Enumerate bits 0-31 for KRF-Tech layer
 */

#define	BIT0 = 0x00000001,
#define	BIT1 = 0x00000002,
#define	BIT2 = 0x00000004,
#define	BIT3 = 0x00000008,
#define	BIT4 = 0x00000010,
#define	BIT5 = 0x00000020,
#define	BIT6 = 0x00000040,
#define	BIT7 = 0x00000080,
#define	BIT8 = 0x00000100,
#define	BIT9 = 0x00000200,
#define	BIT10 = 0x00000400,
#define	BIT11 = 0x00000800,
#define	BIT12 = 0x00001000,
#define	BIT13 = 0x00002000,
#define	BIT14 = 0x00004000,
#define	BIT15 = 0x00008000,
#define	BIT16 = 0x00010000,
#define	BIT17 = 0x00020000,
#define	BIT18 = 0x00040000,
#define	BIT19 = 0x00080000,
#define	BIT20 = 0x00100000,
#define	BIT21 = 0x00200000,
#define	BIT22 = 0x00400000,
#define	BIT23 = 0x00800000,
#define	BIT24 = 0x01000000,
#define	BIT25 = 0x02000000,
#define	BIT26 = 0x04000000,
#define	BIT27 = 0x08000000,
#define	BIT28 = 0x10000000,
#define	BIT29 = 0x20000000,
#define	BIT30 = 0x40000000,
#define	BIT31 = 0x80000000


#ifdef __cplusplus
}
#endif

#endif /* _V3USCREG_H_ */
