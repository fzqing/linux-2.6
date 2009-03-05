#ifndef __PAL_SYS_H__
#define __PAL_SYS_H__

#include <linux/config.h>
#include <linux/module.h>
#include <asm/mach-avalanche/soc.h>
#include <asm/mach-avalanche/titan.h>
#include <asm/mach-avalanche/avalanche_intc.h>

#include "pal_defs.h" 
                                                                                                 
#define MIPS_4KEC
#define BOARD_TYPE_UNKNOWN 0xFF
typedef enum PAL_SYS_RESET_CTRL_tag
{
    IN_RESET        = 0,
    OUT_OF_RESET
} PAL_SYS_RESET_CTRL_T;

typedef enum PAL_SYS_SYSTEM_RST_MODE_tag
{
    RESET_SOC_WITH_MEMCTRL      = 1,    /* SW0 bit in SWRCR register */
    RESET_SOC_WITHOUT_MEMCTRL   = 2     /* SW1 bit in SWRCR register */
} PAL_SYS_SYSTEM_RST_MODE_T;

typedef enum PAL_SYS_SYSTEM_RESET_STATUS_tag
{
    HARDWARE_RESET = 0,
    SOFTWARE_RESET0,            /* Caused by writing 1 to SW0 bit in SWRCR register */
    WATCHDOG_RESET,
    SOFTWARE_RESET1             /* Caused by writing 1 to SW1 bit in SWRCR register */
} PAL_SYS_SYSTEM_RESET_STATUS_T;

typedef void (*REMOTE_VLYNQ_DEV_RESET_CTRL_FN)(unsigned int reset_module, 
                                               PAL_SYS_RESET_CTRL_T reset_ctrl);

typedef enum PAL_SYS_POWER_CTRL_tag
{
    POWER_CTRL_POWER_UP = 0,
    POWER_CTRL_POWER_DOWN
} PAL_SYS_POWER_CTRL_T;

typedef enum PAL_SYS_SYSTEM_POWER_MODE_tag
{
    GLOBAL_POWER_MODE_RUN       = 0,    /* All system is up */
    GLOBAL_POWER_MODE_IDLE,             /* MIPS is power down, all peripherals working */
    GLOBAL_POWER_MODE_STANDBY,          /* Chip in power down, but clock to ADSKL subsystem is running */
    GLOBAL_POWER_MODE_POWER_DOWN        /* Total chip is powered down */
} PAL_SYS_SYSTEM_POWER_MODE_T;


typedef enum PAL_SYS_WAKEUP_INTERRUPT_tag
{
    WAKEUP_INT0 = 1,
    WAKEUP_INT1 = 2,
    WAKEUP_INT2 = 4,
    WAKEUP_INT3 = 8
} PAL_SYS_WAKEUP_INTERRUPT_T;

typedef enum PAL_SYS_WAKEUP_CTRL_tag
{
    WAKEUP_DISABLED = 0,
    WAKEUP_ENABLED
} PAL_SYS_WAKEUP_CTRL_T;

typedef enum PAL_SYS_WAKEUP_POLARITY_tag
{
    WAKEUP_ACTIVE_HIGH = 0,
    WAKEUP_ACTIVE_LOW
} PAL_SYS_WAKEUP_POLARITY_T;

typedef enum PAL_SYS_GPIO_PIN_MODE_tag
{
    FUNCTIONAL_PIN = 0,
    GPIO_PIN = 1
} PAL_SYS_GPIO_PIN_MODE_T;

typedef enum PAL_SYS_GPIO_PIN_DIRECTION_tag
{
    GPIO_OUTPUT_PIN = 0,
    GPIO_INPUT_PIN = 1
} PAL_SYS_GPIO_PIN_DIRECTION_T;

typedef enum { GPIO_FALSE, GPIO_TRUE } PAL_SYS_GPIO_BOOL_T;

typedef struct module_info {
        unsigned int version;
        unsigned int base_addr;
} MOD_INFO_T;

typedef struct board_info {
        MOD_INFO_T modules[MAX_MODULES];
}BOARD_INFO_T;

extern BOARD_INFO_T soc[];

int avalanche_device_prepare(unsigned int module_id, unsigned int base_addr, BOARD_ID board_variant, void *param);

static inline int avalanche_memalloc (unsigned int numBytes,void** memAddr)
{
    *memAddr = kmalloc(numBytes, GFP_KERNEL);
     if(*memAddr == NULL)
     {
        return PAL_OS_ERROR_NO_RESOURCES;
     }
                                                                                              
     return 0;
}

static inline int avalanche_memfree (void* memAddr)
{
    kfree(memAddr);
    return 0;
}

#endif /* _PAL_H_ */
