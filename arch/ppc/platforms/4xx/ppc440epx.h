/*
 * arch/ppc/platforms/4xx/amcc440epx.h
 *
 * PPC440EPX definitions
 *
 *
 * Copyright 2002 Roland Dreier
 * Copyright 2004 MontaVista Software, Inc.
 * Copyright 2006 AMCC.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifdef __KERNEL__
#ifndef __PPC_PLATFORMS_AMCC440EPX_H
#define __PPC_PLATFORMS_AMCC440EPX_H

#include <linux/config.h>
#include <asm/ibm44x.h>
/*Interrupt Assignements - used in OCP table definition */
#define IIC0_INT     2
#define IIC1_INT     7
#define EMAC0_INT   24
#define EMAC1_INT   25

/* UART */
#define PPC440EPX_UART0_ADDR		0x00000001EF600300ULL
#define PPC440EPX_UART1_ADDR		0x00000001EF600400ULL
#define PPC440EPX_UART2_ADDR		0x00000001EF600500ULL
#define PPC440EPX_UART3_ADDR		0x00000001EF600600ULL
#define UART0_INT			0
#define UART1_INT			1
#define UART2_INT			35
#define UART3_INT			36

/* IIC Bootstrap Registers */
#define SDR0_CFGADDR 0x00E		/* System DCR Address Register */
#define SDR0_CFGDATA 0x00F		/* System DCR Data Register */
#define SDR0_SDSTP1  0x021		/* Serial Device Strap Register 1 */

/* Clock and Power Management CPM0 */
#define IBM_CPM_IIC0		0x80000000	/* IIC interface */
#define IBM_CPM_IIC1		0x40000000	/* IIC interface */
#define IBM_CPM_PCI		  0x20000000	/* PCI bridge */
#define IBM_CPM_USB1H		0x08000000	/* USB 1.1 Host */
#define IBM_CPM_FPU		  0x04000000	/* floating point unit */
#define IBM_CPM_CPU		  0x02000000	/* processor core */
#define IBM_CPM_DMA		  0x01000000	/* DMA controller */
#define IBM_CPM_BGO		  0x00800000	/* PLB3 to OPB bus arbiter */
/*#define IBM_CPM_BGI		  0x00400000	 OPB to PLB bridge */
#define IBM_CPM_EBC		  0x00200000	/* External Bus Controller */
#define IBM_CPM_RGMII 	0x00100000	/* Reduced Gigabit MII Bridge */
#define IBM_CPM_DMC		  0x00080000	/* SDRAM peripheral controller */
#define IBM_CPM_PLB4		0x00040000	/* PLB4 bus arbiter */
#define IBM_CPM_PLB4x3	0x00020000	/* PLB4 to PLB3 bridge controller */
#define IBM_CPM_PLB3x4	0x00010000	/* PLB3 to PLB4 bridge controller */
#define IBM_CPM_PLB3		0x00008000	/* PLB3 bus arbiter */
#define IBM_CPM_NDFC 		0x00004000      /* NAND Flash Controller */
/*#define IBM_CPM_PPM		  0x00002000	 PLB Performance Monitor */
#define IBM_CPM_UIC1		0x00001000	/* Universal Interrupt Controller */
#define IBM_CPM_GPIO0		0x00000800	/* General Purpose IO (??) */
#define IBM_CPM_GPT		  0x00000400	/* General Purpose Timers  */
#define IBM_CPM_UART0		0x00000200	/* serial port 0 */
#define IBM_CPM_UART1		0x00000100	/* serial port 1 */
#define IBM_CPM_UIC0		0x00000080	/* Universal Interrupt Controller */
#define IBM_CPM_TMRCLK	0x00000040	/* CPU timers */
#define IBM_CPM_EMAC0		0x00000020	/* ethernet port 0 */
#define IBM_CPM_UART2		0x00000010	/* serial port 2 */
#define IBM_CPM_UART3		0x00000008	/* serial port 3 */
#define IBM_CPM_EMAC1		0x00000004	/* ethernet port 1 */
#define IBM_CPM_P42OPB1	0x00000002	/* USB 2.0 Host*/
#define IBM_CPM_OPB2P4	0x00000001	/* USB 2.0 Host */

/* Clock and Power Management CPM1*/
#define IBM_CPM_UIC2		0x80000000	/* Universal Interrupt Controller 2*/
#define IBM_CPM_SRAM0		0x40000000	/* Internal SRAM Controller  */
#define IBM_CPM_MAL0		0x20000000	/* Memory Access Layer  */
#define IBM_CPM_USB2D0	0x10000000	/* USB2.0 Device  */
#define IBM_CPM_USB2H		0x08000000	/* USB 2.0 HOST  */
#define IBM_CPM_CRYP0  	0x0400000	/* Security Engine */
#define IBM_CPM_KASU0		0x02000000	/* Kasumi Engine */


/* not used at all */
/*#define DFLT_IBM4xx_PM		*/

/*412- SDRAM Controller - ECC Function   */
#define UIC_IRQ_ECC  69
/*  ECC Registers */


#define DDR_DCR_BASE 0x10
#define ddrcfga  (DDR_DCR_BASE+0x0)   /* DDR configuration address reg */
#define ddrcfgd  (DDR_DCR_BASE+0x1)   /* DDR configuration data reg    */

#define DDR0_00                            0x00
#define DDR0_00_INT_ACK_MASK               0x7F000000 /* Write only */
#define DDR0_00_INT_STATUS_MASK            0x00FF0000 /* Read only */
#define DDR0_00_SINGLE_ACCESS_ERR_MASK     0x00800000
#define DDR0_00_MULTI_ACCESS_ERR_MASK      0x00400000
#define DDR0_00_SINGLE_C_EVENT_MASK        0x00200000
#define DDR0_00_MULTI_C_EVENT_MASK         0x00100000
#define DDR0_00_SINGLE_U_EVENT_MASK        0x00080000
#define DDR0_00_MULTI_U_EVENT_MASK         0x00040000

#define DDR0_01                           0x01
#define DDR0_01_INT_MASK                  0x0000000F
#define DDR0_01_ALL_INT_ENABLE            0x00000000

#define DDR0_22                           0x16

#define DDR0_22_CTRL_RAW_MASK             0x03000000
#define DDR0_22_CTRL_RAW_ECC_DISABLE      0x00000000
#define DDR0_22_CTRL_RAW_ECC_CHECK_ONLY   0x01000000
#define DDR0_22_CTRL_RAW_NO_ECC_RAM       0x02000000
#define DDR0_22_CTRL_RAW_ECC_ENABLE       0x03000000

#define DDR0_23                         0x17
#define DDR0_31                         0x1F
#define DDR0_31_XOR_CHECK_BITS_MASK       0x0000FFFF
#define DDR0_32                         0x20
#define DDR0_32_OUT_OF_RANGE_ADDR_MASK    0xFFFFFFFF /* Read only */
#define DDR0_33                         0x21
#define DDR0_33_OUT_OF_RANGE_ADDR_MASK    0x00000001 /* Read only */
#define DDR0_34                         0x22
#define DDR0_34_ECC_U_ADDR_MASK           0xFFFFFFFF /* Read only */
#define DDR0_35                         0x23
#define DDR0_35_ECC_U_ADDR_MASK           0x00000001 /* Read only */
#define DDR0_36                         0x24
#define DDR0_36_ECC_U_DATA_MASK           0xFFFFFFFF /* Read only */
#define DDR0_37                         0x25
#define DDR0_37_ECC_U_DATA_MASK           0xFFFFFFFF /* Read only */
#define DDR0_38                         0x26
#define DDR0_38_ECC_C_ADDR_MASK           0xFFFFFFFF /* Read only */
#define DDR0_39                         0x27
#define DDR0_39_ECC_C_ADDR_MASK           0x00000001 /* Read only */
#define DDR0_40                         0x28
#define DDR0_40_ECC_C_DATA_MASK           0xFFFFFFFF /* Read only */
#define DDR0_41                         0x29
#define DDR0_41_ECC_C_DATA_MASK           0xFFFFFFFF /* Read only */

/* Configuration Selection provided via Board Info structure by UBoot      */
/* The table bi-config[30] corresponds to the 30 selectable functionality  */
/* The function is selected if bi-config[function-number] == 1             */
/* The table order is defined by following define - don't change the value */

#define ENET_PACKET_REJECT	0     /* Enet Packet Reject                               */
#define    IIC_CORE		1     /* I2C1                                             */
#define    SCP_CORE		2     /* SCP                                              */
#define    UIC_0_3		3     /* UIC 0:3                                          */
#define    UIC_4_6		4     /* UIC 4:6                                          */
#define    UIC_7_9		5     /* UIC 7:9                                          */
#define    EXT_DMA_TO_PLB3_0 	6     /* External DMA to PLB3 Channel 0                   */
#define    EXT_DMA_TO_PLB3_1 	7     /* External DMA to PLB3 Channel 1                   */
#define    EXT_DMA_TO_PLB3_2 	8     /* External DMA to PLB3 Channel 2                   */
#define    EXT_DMA_TO_PLB3_3 	9     /* External DMA to PLB3 Channel 3                   */
#define    EBC_MASTER 		10    /* EBC Master                                       */
#define    EBC_16BITS_ADDR_8_31	11    /* EBC 16bits, address[8:31], with parity selection */
#define    EBC_ADDR_5_7		12    /* EBC address[5:7] selection                       */
#define    EBC_ADDR_2_4		13    /* EBC address[2:4] selection                       */
#define    EBC_32BITS		14    /* EBC 32bits with parity selection                 */
#define    NAND_FLASH 		15    /* NAND Flash                                       */
#define    USB2_DEVICE_EXT	16    /* USB2 Device with external PHY selection          */
#define    USB2_DEVICE_INT	17    /* USB2 Device with internal PHY selection          */
#define    USB2_HOST 		18    /* USB2 Host with internal PHY selection            */
#define    UART_CORE0		19    /* UART0                                            */
#define    UART_CORE1		20    /* UART1                                            */
#define    UART_CORE2		21    /* UART2                                            */
#define    UART_CORE3		22    /* UART3                                            */
#define    ENET_CONFIG_1_1	23    /* 1xMII   using RGMII core                         */
#define    ENET_CONFIG_1_2	24    /* 1xMII   using  ZMII core                         */
#define    ENET_CONFIG_2	25    /* 1xGMII  using RGMII core                         */
#define    ENET_CONFIG_3	26    /* 1xTBI   using RGMII core                         */
#define    ENET_CONFIG_4	27    /* 2xRGMII using RGMII core                         */
#define    ENET_CONFIG_5	28    /* 2xRTBI  using RGMII core                         */
#define    ENET_CONFIG_6	29    /* 2xSMII  using  ZMII core                         */


#endif /* __PPC_PLATFORMS_AMCC440EPX_H */
#endif /* __KERNEL__ */
