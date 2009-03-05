#ifndef __SOC_H__
#define __SOC_H__

#define REG8_ADDR(addr)             (volatile UINT8 *)(KSEG1ADDR(addr))
#define REG8_DATA(addr)             (*(volatile UINT8 *)(KSEG1ADDR(addr)))
#define REG8_WRITE(addr, data)      REG8_DATA(addr) = data;
#define REG8_READ(addr, data)       data = (UINT8) REG8_DATA(addr);

#define REG16_ADDR(addr)            (volatile UINT16 *)(KSEG1ADDR(addr))
#define REG16_DATA(addr)            (*(volatile UINT16 *)(KSEG1ADDR(addr)))
#define REG16_WRITE(addr, data)     REG16_DATA(addr) = data;
#define REG16_READ(addr, data)      data = (UINT16) REG16_DATA(addr);

#define REG32_ADDR(addr)            (volatile unsigned int *)(KSEG1ADDR(addr))
#define REG32_DATA(addr)            (*(volatile unsigned int *)(KSEG1ADDR(addr)))
#define REG32_WRITE(addr, data)     REG32_DATA(addr) = data;
#define REG32_READ(addr, data)      data = (unsigned int) REG32_DATA(addr);

#ifdef  _LINK_KSEG0_                /* Application is linked into KSEG0 space */
#define VIRT_ADDR(addr)             PHYS_TO_K0(PHYS_ADDR(addr))
#endif

#ifdef  _LINK_KSEG1_                /* Application is linked into KSEG1 space */
#define VIRT_ADDR(addr)             KSEG1ADDR(PHYS_ADDR(addr))
#endif

/* These ugly macros are to access the -1 registers, like config1 */
#define MFC0_SEL1_OPCODE(dst, src)\
        .word (0x40000000 | ((dst)<<16) | ((src)<<11) | 1);\
        nop; \
        nop; \
        nop

#define MTC0_SEL1_OPCODE(dst, src)\
        .word (0x40800000 | ((dst)<<16) | ((src)<<11) | 1);\
        nop; \
        nop; \
        nop

#define CFG0_4K_IL_MASK         0x00380000
#define CFG0_4K_IL_SHIFT        19
#define CFG0_4K_IA_MASK         0x00070000
#define CFG0_4K_IA_SHIFT        16
#define CFG0_4K_IS_MASK         0x01c00000
#define CFG0_4K_IS_SHIFT        22
#define CFG0_4K_DL_MASK         0x00001c00
#define CFG0_4K_DL_SHIFT        10
#define CFG0_4K_DA_MASK         0x00000380
#define CFG0_4K_DA_SHIFT        7
#define CFG0_4K_DS_MASK         0x0000E000
#define CFG0_4K_DS_SHIFT        13

#define FREQ_1MHZ               1000000

#if !defined(_ASMLANGUAGE)
        
typedef enum AVALANCHE_CPU_TYPE_tag
{
    CPU_AVALANCHE_I         = 0,    
    CPU_AVALANCHE_D         = 2,
    CPU_PUMA                = 4,
    CPU_PUMAS               = 0x0104,
    CPU_SANGAM              = 0x5,
    CPU_TITAN               = 0x7,
    CPU_APEX                = 0xb,
    CPU_PUMA3               = 0x0204,
    CPU_UNIDENT             = 0xFF
    
}AVALANCHE_CPU_TYPE_T;
        
AVALANCHE_CPU_TYPE_T avalanche_get_cpu_type(void);
const char * avalanche_get_cpu_name(AVALANCHE_CPU_TYPE_T cpu_type);

typedef int (*SET_MDIX_ON_CHIP_FN_T)(unsigned int base_addr, unsigned int operation);
int avalanche_set_mdix_on_chip(unsigned int base_addr, unsigned int operation);
unsigned int avalanche_is_mdix_on_chip(void);

unsigned int avalanche_get_chip_version_info(void);

unsigned int avalanche_get_vbus_freq(void);
void         avalanche_set_vbus_freq(unsigned int);


extern unsigned int avalanche_mips_freq;
extern unsigned int avalanche_vbus_freq;

#define AVALANCHE_MIPS_FREQ     (avalanche_mips_freq)
#define AVALANCHE_VBUS_FREQ     (avalanche_vbus_freq)

#ifdef SEAD2_EMLN
#define AVALANCHE_UART_FREQ     3686400
#else
#define AVALANCHE_UART_FREQ     AVALANCHE_VBUS_FREQ
#endif

#endif /* !defined(_ASMLANGUAGE) */

#endif /* __SOC_H__ */

