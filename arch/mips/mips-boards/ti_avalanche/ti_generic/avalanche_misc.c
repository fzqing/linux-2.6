#include <asm/mach-avalanche/pal.h>
#include <linux/module.h>
#include <linux/spinlock.h>

static unsigned int avalanche_vbus_freq;

void avalanche_set_vbus_freq(unsigned int new_vbus_freq)
{
    avalanche_vbus_freq = new_vbus_freq;
}

unsigned int avalanche_get_vbus_freq(void)
{
    return(avalanche_vbus_freq);
}

unsigned int avalanche_get_chip_version_info(void)
{
    return(*(volatile unsigned int*)AVALANCHE_CVR);
}

int avalanche_is_cpmac_on_vbus(void)
{
    if(0x00b != (*((volatile unsigned int*)AVALANCHE_CVR) & 0xffff))
        return (1); /* The SoC which does not have cpmac on Vbus is Apex */
    else 
        return (0);
}

unsigned int avalanche_chip_version_info(void)
{
    return (*(volatile unsigned int*)AVALANCHE_CVR);
}

AVALANCHE_CPU_TYPE_T avalanche_get_cpu_type(void)
{
    volatile unsigned int cvr;
    volatile unsigned int cvr2;
    AVALANCHE_CPU_TYPE_T cpu_type;
   
    cpu_type =  CPU_UNIDENT;
   
    cvr =  avalanche_chip_version_info() & 0xffff;
   
    REG32_WRITE(AVALANCHE_CVR, 0xffff);
   
    cvr2 = (REG32_DATA(AVALANCHE_CVR) & 0xffff);
   
    if(cvr == cvr2)
    {
        switch(cvr)
        {
            case CPU_AVALANCHE_I:
            case 0xffff:   /* This is because of the Avalanche I errata */
                cpu_type = CPU_AVALANCHE_I;
            break;
            
            case CPU_AVALANCHE_D:
            case CPU_PUMA:
            case CPU_PUMAS:
            case CPU_SANGAM:
            case CPU_TITAN:
            case CPU_APEX:
                cpu_type = cvr;
            break;
         
            default:
                cpu_type = CPU_UNIDENT;
        }
    }
    else
    {
        cpu_type = CPU_AVALANCHE_I;
    }
   
    return cpu_type;    
}

const char* avalanche_get_cpu_name(AVALANCHE_CPU_TYPE_T cpu_type)
{
    char* name = NULL;
    static char* cpu_name[]=
    {
       "Unknown"    ,   /* 0 */ 
       "Avalanche I",   /* 1 */ 
       "Avalanche D",   /* 2 */ 
       "Puma"       ,   /* 3 */ 
       "Puma S"     ,   /* 4 */
       "Sangam"     ,   /* 5 */
       "Titan"      ,   /* 6 */
       "Apex"       ,   /* 7 */
       
       /* TODO: Add more CPUs here */
       0
    };
    
    switch(cpu_type)
    {
        case CPU_AVALANCHE_I: 
            name = cpu_name[1];  
        break;

        case CPU_AVALANCHE_D: 
            name = cpu_name[2];  
        break;

        case CPU_PUMA : 
            name = cpu_name[3];
        break;

        case CPU_PUMAS: 
            name = cpu_name[4];  
        break;

        case CPU_SANGAM: 
            name = cpu_name[5];  
        break;        
        
        case CPU_TITAN: 
            name = cpu_name[6];  
        break;               
        
        case CPU_APEX: 
            name = cpu_name[7];  
        break;                       

        case CPU_UNIDENT:
        default:        
            name = cpu_name[0];
    }
    
    return name;
}

SET_MDIX_ON_CHIP_FN_T p_set_mdix_on_chip_fn = NULL;
int avalanche_set_mdix_on_chip(unsigned int base_addr, unsigned int operation)
{
    if(p_set_mdix_on_chip_fn)
        return (p_set_mdix_on_chip_fn(base_addr, operation));
    else
        return (-1);
}

unsigned int avalanche_is_mdix_on_chip(void)
{
    return (p_set_mdix_on_chip_fn ? 1:0);
}

int avalanche_sysprobe_and_prep (unsigned int version, unsigned int base_addr, void *param)
{
        int i = 0;
        BOARD_ID board_variant = avalanche_get_board_variant();
        MOD_INFO_T *p = &(soc[board_variant].modules[0]);

        for(i = 0; i < MAX_MODULES; i++, p++)
        {
            if ((version == p->version) && (base_addr == p->base_addr))
            {
                version = p->version;
                avalanche_device_prepare(version, base_addr, board_variant, param);
                return 0;
            }
        }

        return (-1);
}

#define AVALANCHE_GLOBAL_POWER_DOWN_MASK	0x3FFFFFFF /* bit 31, 30 masked */
#define AVALANCHE_GLOBAL_POWER_DOWN_BIT		30         /* shift to bit 30, 31 */

REMOTE_VLYNQ_DEV_RESET_CTRL_FN p_remote_vlynq_dev_reset_ctrl = NULL;

void avalanche_set_global_powermode(PAL_SYS_SYSTEM_POWER_MODE_T power_mode)
{
	volatile unsigned int *power_status_reg = (unsigned int*)AVALANCHE_POWER_CTRL_PDCR;
                                                                                                
	*power_status_reg &= AVALANCHE_GLOBAL_POWER_DOWN_MASK;
	*power_status_reg |= ( power_mode << AVALANCHE_GLOBAL_POWER_DOWN_BIT);
}

void avalanche_reset_ctrl(unsigned int module_reset_bit, PAL_SYS_RESET_CTRL_T reset_ctrl)
{
    volatile unsigned int *reset_reg = (unsigned int*) AVALANCHE_RST_CTRL_PRCR;

    if(module_reset_bit >= 32 && module_reset_bit < 64)
	{
        return;
	}

    if(module_reset_bit >= 64)
    {
        if(p_remote_vlynq_dev_reset_ctrl)
		{
            return(p_remote_vlynq_dev_reset_ctrl(module_reset_bit - 64, reset_ctrl));
		}
		else
		{
            return;
		}
    }
    
    if(reset_ctrl == OUT_OF_RESET)
	{
        *reset_reg |= 1 << module_reset_bit;
	}
    else
	{
        *reset_reg &= ~(1 << module_reset_bit);
	}
}

PAL_SYS_RESET_CTRL_T avalanche_get_reset_status(unsigned int module_reset_bit)
{
    volatile unsigned int *reset_reg = (unsigned int*) AVALANCHE_RST_CTRL_PRCR;

    return (((*reset_reg) & (1 << module_reset_bit)) ? OUT_OF_RESET : IN_RESET );
}


void avalanche_system_reset(PAL_SYS_SYSTEM_RST_MODE_T mode)
{
    volatile unsigned int *sw_reset_reg = (unsigned int*) AVALANCHE_RST_CTRL_SWRCR;
    *sw_reset_reg =  mode;
}

#define AVALANCHE_RST_CTRL_RSR_MASK 0x3

PAL_SYS_SYSTEM_RESET_STATUS_T avalanche_get_last_reset_status()
{
    volatile unsigned int *sys_reset_status = (unsigned int*) AVALANCHE_RST_CTRL_RSR;

    return ( (PAL_SYS_SYSTEM_RESET_STATUS_T) (*sys_reset_status & AVALANCHE_RST_CTRL_RSR_MASK) );
}

