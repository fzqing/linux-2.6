/*
 * Jeff Harrell, jharrell@ti.com
 * Copyright (C) 2001 Texas Instruments, Inc.  All rights reserved.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 * Texas Instruments generic avalanche setup.
 */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/mc146818rtc.h>
#include <linux/ioport.h>
#include <asm/time.h>

#include <asm/cpu.h>
#include <asm/mipsregs.h>
#include <asm/bootinfo.h>
#include <asm/irq.h>
#include <asm/serial.h>
#include <asm/mips-boards/generic.h>
#include <asm/mips-boards/prom.h>

#include <linux/proc_fs.h>

#include <asm/uaccess.h>
#include <asm/mach-avalanche/pal.h>

#ifdef CONFIG_SERIAL_8250
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
extern int early_serial_setup(struct uart_port *port);
#endif /* CONFIG_SERIAL_8250 */

extern void mips_time_init(void);
extern void mips_timer_setup(struct irqaction *irq);
extern void avalanche_soc_platform_init(void);
extern void mips_reboot_setup(void);
extern unsigned int cpu_freq;
extern void sangam_reset_init(void);
irqreturn_t avalanche_unified_secondary_irq_handler( int unified_sec_irq,
					void * dev_id, struct pt_regs *regs);


#define PSP_NAME    "BasePSP"
#define PSP_VERSION "8.0.0.3"

static int avalanche_p_read_base_psp_version(char* buf, char **start, off_t offset, 
                                             int count, int *eof, void *data)
{
    int len   = 0;
    int limit = count - 80;
    char *cache_mode[4] = {"cached, write through, no write allocate", \
                           "cached, write through, write allocate", \
                           "uncached", \
                           "cached, write back, write allocate"};


    int cache_index = read_c0_config() & 0x7;
 
    if(cache_index == 7) cache_index = 2;
    if((cache_index > 3) && (cache_index <= 6)) cache_index = 3;

    if(len<=limit)
        len+= sprintf(buf+len, "\nLinux OS %s version %s\n"\
			       "Avalanche SOC Version: 0x%x operating in %s mode\n"\
			       "Cpu Frequency: %u MHZ\nSystem Bus frequency: %u MHZ\n\n", 
					PSP_NAME, PSP_VERSION, 
					avalanche_get_chip_version_info(), cache_mode[cache_index],
					cpu_freq/1000000, 2*avalanche_get_vbus_freq()/1000000);
    
    return (len);
}


int avalanche_proc_init(void)
{
    struct proc_dir_entry *avalanche_proc_root;

    avalanche_proc_root = proc_mkdir("avalanche",NULL);
    if(!avalanche_proc_root)
        return -ENOMEM;

    create_proc_read_entry("avalanche/base_psp_version", 0, NULL, avalanche_p_read_base_psp_version, NULL);

   /* Create other proc entries */
    return (0);
}
fs_initcall(avalanche_proc_init);

int setup_unified_secondary_interrupt(void)
{
    if (request_irq(LNXINTNUM(AVALANCHE_UNIFIED_SECONDARY_INT), 
                            avalanche_unified_secondary_irq_handler, 
                            SA_INTERRUPT, "Avalanche Unified Secondary",
                            NULL) != 0)
    {
        printk(KERN_ERR "Failed to setup handler for unified secondary interrupt\n");
        return -1; 
    }
    return 0;
}
late_initcall(setup_unified_secondary_interrupt);

const char *get_system_type(void)
{
	return "Texas Instruments BroadBand SoC";
}

struct serial_port_dfns 
{
    unsigned int irq;
    unsigned int iomem_base;
#ifdef CONFIG_KGDB
    int io_type;
    int iomem_reg_shift;
    unsigned int flags;
    unsigned int port;
    unsigned int baud_base;
#endif
};

static struct serial_port_dfns serial_port_dfns[] = {
	    SERIAL_PORT_DFNS /* defined in asm/serial.h */
};

int __init ti_avalanche_setup(void)
{
	struct uart_port av_serial[CONFIG_AVALANCHE_NUM_SER_PORTS];
    struct uart_port *console_port = NULL;
    int i;


	board_time_init = mips_time_init;		 
	board_timer_setup = mips_timer_setup;
	
	/* Initialize the platform first up */
    avalanche_soc_platform_init();

#ifdef CONFIG_SERIAL_8250

	memset(&av_serial, 0, sizeof(av_serial));

    for ( i = 0; i < CONFIG_AVALANCHE_NUM_SER_PORTS; i++)
    {
	    av_serial[i].line		= i;
	    av_serial[i].irq		= LNXINTNUM(serial_port_dfns[i].irq);
	    av_serial[i].flags		= STD_COM_FLAGS;
	    av_serial[i].uartclk	= avalanche_get_vbus_freq();
#ifdef CONFIG_KGDB
	    av_serial[i].iotype	    = 0; /* default to SERIAL_IO_PORT */
#else
	    av_serial[i].iotype	    = 4; /* default to "unknown". This works best */
#endif
	    av_serial[i].iobase	    = serial_port_dfns[i].iomem_base;
	    av_serial[i].regshift	= 2;
        av_serial[i].type       = PORT_16550A;
   
        /* skip if this is the console port */ 
        if(CONFIG_AVALANCHE_CONSOLE_PORT == i)
        {
            console_port = &av_serial[i];            
            continue;
        }
        		
	    if (early_serial_setup(&av_serial[i]) != 0) 
	    {
	        printk(KERN_ERR"call to early_serial_setup on port %d failed.\n", i);		
	    }
    }
	if (early_serial_setup(console_port) != 0) 
	{
	    printk(KERN_ERR"call to early_serial_setup on port %d failed.\n", i);
    }

#endif
    mips_reboot_setup();
	return 0;
}

early_initcall(ti_avalanche_setup);

