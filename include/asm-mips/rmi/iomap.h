/*
 *
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
#ifndef _ASM_RFI_IO_H
#define _ASM_RFI_IO_H

#define DEFAULT_PHOENIX_IO_BASE 0xffffffffbef00000
#define PHOENIX_IO_SIZE                   0x1000

#define PHOENIX_IO_BRIDGE_OFFSET          0x00000

#define PHOENIX_IO_DDR2_CHN0_OFFSET       0x01000
#define PHOENIX_IO_DDR2_CHN1_OFFSET       0x02000
#define PHOENIX_IO_DDR2_CHN2_OFFSET       0x03000
#define PHOENIX_IO_DDR2_CHN3_OFFSET       0x04000

#define PHOENIX_IO_RLD2_CHN0_OFFSET       0x05000
#define PHOENIX_IO_RLD2_CHN1_OFFSET       0x06000

#define PHOENIX_IO_SRAM_OFFSET            0x07000

#define PHOENIX_IO_PIC_OFFSET             0x08000
#define PHOENIX_IO_PCIX_OFFSET            0x09000
#define PHOENIX_IO_HT_OFFSET              0x0A000

#define PHOENIX_IO_SECURITY_OFFSET        0x0B000

#define PHOENIX_IO_GMAC_0_OFFSET          0x0C000
#define PHOENIX_IO_GMAC_1_OFFSET          0x0D000
#define PHOENIX_IO_GMAC_2_OFFSET          0x0E000
#define PHOENIX_IO_GMAC_3_OFFSET          0x0F000

#define PHOENIX_IO_SPI4_0_OFFSET          0x10000
#define PHOENIX_IO_XGMAC_0_OFFSET         0x11000
#define PHOENIX_IO_SPI4_1_OFFSET          0x12000
#define PHOENIX_IO_XGMAC_1_OFFSET         0x13000

#define PHOENIX_IO_UART_0_OFFSET          0x14000
#define PHOENIX_IO_UART_1_OFFSET          0x15000

#define PHOENIX_IO_I2C_0_OFFSET           0x16000
#define PHOENIX_IO_I2C_1_OFFSET           0x17000

#define PHOENIX_IO_GPIO_OFFSET            0x18000

#define PHOENIX_IO_FLASH_OFFSET           0x19000
#define PHOENIX_IO_DMA_OFFSET             0x1A000

#define PHOENIX_IO_TB_OFFSET           	  0x1C000
#define PHOENIX_CPLD_OFFSET               0xffffffffbd840000

/* Base Address (Virtual) of the PCI Config address space
 * For now, choose 256M phys in kseg1 = 0xA0000000 + (1<<28)
 * Config space spans 256 (num of buses) * 256 (num functions) * 256 bytes
 * ie 1<<24 = 16M
 */ 
#define DEFAULT_PCI_CONFIG_BASE 	0x18000000
#define DEFAULT_HT_TYPE0_CFG_BASE       0x16000000
#define DEFAULT_HT_TYPE1_CFG_BASE       0x17000000


#ifndef __ASSEMBLY__

#include <linux/types.h>
#include <asm/byteorder.h>

typedef volatile __u32 phoenix_reg_t;
extern unsigned long phoenix_io_base;

#define phoenix_io_mmio(offset) ((phoenix_reg_t *)((unsigned long)(DEFAULT_PHOENIX_IO_BASE) + (offset)))

#define phoenix_read_reg(base, offset) (be32_to_cpu((base)[(offset)]))
#define phoenix_write_reg(base, offset, value) ((base)[(offset)] = cpu_to_be32((value)))

#define phoenix_read_reg_le32(base, offset) (le32_to_cpu((base)[(offset)]))
#define phoenix_write_reg_le32(base, offset, value) \
	((base)[(offset)] = cpu_to_le32((value)))

extern void on_chip_init(void);

#endif

#endif
