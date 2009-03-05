/*
 * Copyright (C) 2006 Texas Instruments Inc.
 *
 * This program is free software; you can distribute it and/or modify it
 * under the terms of the GNU General Public License (Version 2) as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/kernel_stat.h>
#include <linux/proc_fs.h>
#include <asm/irq.h>
#include <asm/atomic.h>
#include <asm/mips-boards/prom.h>
#include <asm/mach-avalanche/avalanche_intc.h>
#include <asm/mach-avalanche/pal.h>
#include <asm/gdb-stub.h>

extern irq_desc_t irq_desc [NR_IRQS];

struct avalanche_ictrl_regs         *avalanche_hw0_icregs;  
struct avalanche_exctrl_regs        *avalanche_hw0_ecregs;  
struct avalanche_ipace_regs         *avalanche_hw0_ipaceregs;
struct avalanche_channel_int_number *avalanche_hw0_chregs;  

extern asmlinkage void mipsIRQ(void);

#define INTERRUPT_PANIC(irq) \
        printk("whee, invalid irq_nr %d at %s line %d\n", \
                                            (irq), __FILE__, __LINE__);\
        panic("IRQ, you lose...");

typedef void (*AV_VLYNQ_DISABLE_IRQ_FN)(unsigned int);
typedef void (*AV_VLYNQ_EN_IRQ_FN)(unsigned int);
typedef int  (*AV_VLYNQ_SET_IRQ_POL_FN)(unsigned int, unsigned int);
typedef int  (*AV_VLYNQ_GET_IRQ_POL_FN)(unsigned int);
typedef int  (*AV_VLYNQ_SET_IRQ_TYPE_FN)(unsigned int, unsigned int);
typedef int  (*AV_VLYNQ_GET_IRQ_TYPE_FN)(unsigned int);
typedef int  (*AV_VLYNQ_REQ_IRQ_FN)(unsigned int irq, irqreturn_t (*handler)(int, void *, struct pt_regs *),
		                   unsigned long irqflags, const char*, void*);
typedef void (*AV_VLYNQ_FREE_IRQ_FN)(unsigned int, void*);
typedef int  (*AV_VLYNQ_IRQ_LIST_FN)(char*);

static AV_VLYNQ_SET_IRQ_POL_FN  p_av_vlynq_set_irq_pol_fn  = NULL;
static AV_VLYNQ_GET_IRQ_POL_FN  p_av_vlynq_get_irq_pol_fn  = NULL;
static AV_VLYNQ_REQ_IRQ_FN      p_av_vlynq_req_irq_fn      = NULL;
static AV_VLYNQ_FREE_IRQ_FN     p_av_vlynq_free_irq_fn     = NULL;
static AV_VLYNQ_SET_IRQ_TYPE_FN p_av_vlynq_set_irq_type_fn = NULL;
static AV_VLYNQ_GET_IRQ_TYPE_FN p_av_vlynq_get_irq_type_fn = NULL;

static unsigned long line_to_channel[AVINTNUM(AVALANCHE_INT_END_PRIMARY)];

static void disable_mips_irq(unsigned int irq_nr)
{
    unsigned long flags;
    unsigned long int_bit=0;

    if(unlikely((irq_nr < 0) || (irq_nr >= MIPS_EXCEPTION_OFFSET)))
    {
        INTERRUPT_PANIC(irq_nr);
    }

    save_and_cli(flags);

    int_bit = read_c0_status() & ~(1 << (8+irq_nr)); 
    change_c0_status(ST0_IM, int_bit);
    
    restore_flags(flags);
}

static inline void disable_avalanche_intc_irq(unsigned int irq_nr)
{
    unsigned long flags;
    unsigned long chan_nr=0;

    save_and_cli(flags);

    irq_nr = AVINTNUM(irq_nr);

    if(irq_nr >= AVALANCHE_INT_END_PRIMARY_REG2) {
        chan_nr = irq_nr;
    } else {
        chan_nr = line_to_channel[irq_nr];
    }

    if(chan_nr < AVALANCHE_INT_END_PRIMARY_REG1) {
        avalanche_hw0_icregs->intecr1 = (1 << chan_nr);
    } /* primary interrupt #'s 32-39 */
    else if ((chan_nr <  AVALANCHE_INT_END_PRIMARY_REG2) &&
         (chan_nr >= AVALANCHE_INT_END_PRIMARY_REG1))
    {
        avalanche_hw0_icregs->intecr2 = 
                (1 << (chan_nr - AVALANCHE_INT_END_PRIMARY_REG1));
    } else  {/* secondary interrupt #'s 0-31 */
        avalanche_hw0_ecregs->exiecr = 
                (1 << (chan_nr - AVALANCHE_INT_END_PRIMARY_REG2));
    }

    restore_flags(flags);
}

static void enable_mips_irq(unsigned int irq_nr)
{
    unsigned long flags;
    unsigned long int_bit=0;

    if((irq_nr < 0) || irq_nr >= MIPS_EXCEPTION_OFFSET) {
       INTERRUPT_PANIC(irq_nr); 
    }

    save_and_cli(flags);

    int_bit = read_c0_status();
    change_c0_status(ST0_IM, int_bit | (1<<(8+irq_nr))); 

    restore_flags(flags);
}

static inline void enable_avalanche_intc_irq(unsigned int irq_nr)
{
    unsigned int flags;
    unsigned long chan_nr=0;

    save_and_cli(flags);

    irq_nr = AVINTNUM(irq_nr);

    if(irq_nr >= AVALANCHE_INT_END_PRIMARY_REG2)
        chan_nr = irq_nr;
    else
        chan_nr = line_to_channel[irq_nr];

    if(chan_nr < AVALANCHE_INT_END_PRIMARY_REG1)    
    {
        avalanche_hw0_icregs->intesr1 = 1 << chan_nr;
    } /* primary interrupt #'s 32 throuth 39 */
    else if ((chan_nr < AVALANCHE_INT_END_PRIMARY_REG2) &&
        (chan_nr >= AVALANCHE_INT_END_PRIMARY_REG1))
    {
        avalanche_hw0_icregs->intesr2 = 
            (1 << (chan_nr - AVALANCHE_INT_END_PRIMARY_REG1));
    } 
    else /* secondary interrupt #'s 0-31 */ 
    {
        avalanche_hw0_ecregs->exiesr = 
            (1 << (chan_nr - AVALANCHE_INT_END_PRIMARY_REG2));
    }

    restore_flags(flags);
}

static void end_avalanche_intc_irq(unsigned int irq) \
{   
    if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS)))
        enable_avalanche_intc_irq(irq);
}

static void end_mips_irq(unsigned int irq) \
{   
    if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS)))
        enable_mips_irq(irq);
}

static unsigned int startup_avalanche_intc_irq(unsigned int irq)
{
    enable_avalanche_intc_irq(irq);
    return 0; 
}

static unsigned int startup_mips_irq(unsigned int irq)
{
    enable_mips_irq(irq);
    return 0; 
}

#define startup_avalanche_primary_irq       startup_avalanche_intc_irq
#define shutdown_avalanche_primary_irq      disable_avalanche_intc_irq
#define enable_avalanche_primary_irq        enable_avalanche_intc_irq
#define disable_avalanche_primary_irq       disable_avalanche_intc_irq
#define mask_and_ack_avalanche_primary_irq  disable_avalanche_intc_irq
#define end_avalanche_primary_irq           end_avalanche_intc_irq

static struct hw_interrupt_type avalanche_primary_irq_type = {
    "[INTC Primary]",
    startup_avalanche_primary_irq,
    shutdown_avalanche_primary_irq,
    enable_avalanche_primary_irq,
    disable_avalanche_primary_irq,
    mask_and_ack_avalanche_primary_irq,
    end_avalanche_primary_irq,
    NULL
};

#define shutdown_mips_irq      disable_mips_irq
#define mask_and_ack_mips_irq  disable_mips_irq

static struct hw_interrupt_type mips_irq_type = {
    "[MIPS]",
    startup_mips_irq,
    shutdown_mips_irq,
    enable_mips_irq,
    disable_mips_irq,
    mask_and_ack_mips_irq,
    end_mips_irq,
    NULL
};

#define startup_avalanche_secondary_irq       startup_avalanche_intc_irq
#define shutdown_avalanche_secondary_irq      disable_avalanche_intc_irq
#define enable_avalanche_secondary_irq        enable_avalanche_intc_irq
#define disable_avalanche_secondary_irq       disable_avalanche_intc_irq
#define mask_and_ack_avalanche_secondary_irq  disable_avalanche_intc_irq
#define end_avalanche_secondary_irq           end_avalanche_intc_irq

static struct hw_interrupt_type avalanche_secondary_irq_type = {
    "[INTC Secondary]",
    startup_avalanche_secondary_irq,
    shutdown_avalanche_secondary_irq,
    enable_avalanche_secondary_irq,
    disable_avalanche_secondary_irq,
    mask_and_ack_avalanche_secondary_irq,
    end_avalanche_secondary_irq,
    NULL
};

void arch_free_irq_hook(unsigned int irq, void *dev_id)
{
    if(irq >= AVALANCHE_INTC_END) {
        if(p_av_vlynq_free_irq_fn) {
            p_av_vlynq_free_irq_fn(irq, dev_id);
        }    
    }
}

int arch_req_irq_hook(unsigned int irq, 
        irqreturn_t (*handler)(int, void *, struct pt_regs *),
		unsigned long irqflags, const char * devname, void *dev_id)
{
    if(irq >= AVALANCHE_INTC_END) {
        if(p_av_vlynq_req_irq_fn) {
           return p_av_vlynq_req_irq_fn(irq, handler, irqflags, devname, dev_id);
        }    
    }
    return 0;
}


irqreturn_t avalanche_unified_secondary_irq_handler
(
    int unified_sec_irq,
    void * dev_id,
    struct pt_regs *regs
)
{
    int irq = AVALANCHE_INT_END_PRIMARY;
    unsigned int status = avalanche_hw0_ecregs->exsr;

    /* clear secondary interrupt */
    avalanche_hw0_ecregs->excr = status;

    /* service all the secondary interrupts one-by-one */
    while(status)
    {
        if(status & 0x1)
        {
            do_IRQ(irq, regs);
        }
        status >>= 1;
        irq++;
    }

    return IRQ_RETVAL(1);
}

static inline void __init avalanche_init_intc_hw(void)
{
    avalanche_hw0_icregs = 
        (struct avalanche_ictrl_regs *)AVALANCHE_ICTRL_REGS_BASE;
    avalanche_hw0_ecregs = 
        (struct avalanche_exctrl_regs *)AVALANCHE_ECTRL_REGS_BASE;
    avalanche_hw0_chregs = 
        (struct avalanche_channel_int_number *)AVALANCHE_CHCTRL_REGS_BASE;

    avalanche_hw0_icregs->intecr1 = 0xffffffff;    
    avalanche_hw0_icregs->intcr1  = 0xffffffff;
    avalanche_hw0_icregs->intecr2 = 0xff;
    avalanche_hw0_icregs->intcr2  = 0xff;          
    avalanche_hw0_ecregs->exiecr  = 0xffffffff;    
    avalanche_hw0_ecregs->excr    = 0xffffffff;    
}

void __init arch_init_irq(void)
{
    int i,j;

    avalanche_init_intc_hw();
    for(i = MIPS_EXCEPTION_OFFSET, j = 0; 
        i < AVALANCHE_INT_END_PRIMARY; i++, j++)
    {
        avalanche_int_set(j,i);
    }

    set_except_vector(0, mipsIRQ);

    for (i = 0; i < AVALANCHE_INT_END; i++)
    {
        irq_desc[i].status    = IRQ_DISABLED | IRQ_PER_CPU;
        irq_desc[i].action    = 0;
        irq_desc[i].depth    = 1;
    
        if(i < MIPS_EXCEPTION_OFFSET) {
            irq_desc[i].handler    = &mips_irq_type;
        } else if (i < AVALANCHE_INT_END_PRIMARY) {
            irq_desc[i].handler    = &avalanche_primary_irq_type;
        } else if (i < AVALANCHE_INT_END_SECONDARY) {
            irq_desc[i].handler    = &avalanche_secondary_irq_type;
        }
    }
}

void avalanche_hw0_irqdispatch(struct pt_regs *regs)
{
    int chan_nr=0, irq=0;
    unsigned int priority_irq_info = avalanche_hw0_icregs->pintir;

    irq     = (priority_irq_info >> 16) & 0x3F;
    chan_nr = priority_irq_info & 0x3F;

    if(chan_nr < AVALANCHE_INT_END_PRIMARY_REG1) {
        avalanche_hw0_icregs->intcr1 = (1 << chan_nr);
    } else if (chan_nr < AVALANCHE_INT_END_PRIMARY_REG2 ) {
            avalanche_hw0_icregs->intcr2 = 1 << 
                            (chan_nr - AVALANCHE_INT_END_PRIMARY_REG1);
    } else {
        return;
    }
    do_IRQ(LNXINTNUM(irq), regs);

    return;
}

void avalanche_int_set(int channel, int line)
{
    if(line <  MIPS_EXCEPTION_OFFSET ||
       line >= AVALANCHE_INT_END_PRIMARY ||
       line == LNXINTNUM(AVALANCHE_UNIFIED_SECONDARY_INT)||
       channel == 0)        
        return;

  line = AVINTNUM(line);


  avalanche_hw0_chregs->cintnr[channel] = line;  
  line_to_channel[line] = channel; 
}

int avalanche_intr_polarity_set(unsigned int irq_nr, unsigned long polarity_val)
{
    unsigned long flags;
    unsigned long chan_nr=0;

    if(irq_nr  <  MIPS_EXCEPTION_OFFSET ||
        irq_nr  >= AVALANCHE_INT_END ||
        (irq_nr >= AVALANCHE_INT_END_PRIMARY && 
        irq_nr <  AVALANCHE_INT_END_SECONDARY))
    {
        printk("whee, invalid irq_nr %d\n", irq_nr);
        printk("Not one of the primary avalanche interrupts\n");
        panic("IRQ, you lose...");
        return(-1);
    }

    if(polarity_val > 1)
    {
        printk("Not a valid polarity value.\n");
        return(-1);
    }


    if(irq_nr >= AVALANCHE_INTC_END)
    {
        if(p_av_vlynq_set_irq_pol_fn)
            p_av_vlynq_set_irq_pol_fn(irq_nr, polarity_val);
	    goto ret_from_set_polarity;
    }

    irq_nr = AVINTNUM(irq_nr);

    chan_nr = line_to_channel[irq_nr];

    save_and_cli(flags);

    if(chan_nr < AVALANCHE_INT_END_PRIMARY_REG1)
    {
        if(polarity_val)
            avalanche_hw0_icregs->intpolr1 |=  (1 << chan_nr);
        else
        avalanche_hw0_icregs->intpolr1 &= ~(1 << chan_nr);
    }
    else
    {
        if(polarity_val)
            avalanche_hw0_icregs->intpolr2 |=
            (1 << (chan_nr - AVALANCHE_INT_END_PRIMARY_REG1));
        else
            avalanche_hw0_icregs->intpolr2 &=
            ~(1 << (chan_nr - AVALANCHE_INT_END_PRIMARY_REG1));
    }

    restore_flags(flags);

ret_from_set_polarity:
    return(0);
}


int avalanche_intr_polarity_get(unsigned int irq_nr)
{
    unsigned long flags;
    unsigned long chan_nr=0;
    int           value = 0; 

    if(irq_nr  <  MIPS_EXCEPTION_OFFSET ||
       irq_nr  >= AVALANCHE_INT_END ||
       (irq_nr >= AVALANCHE_INT_END_PRIMARY && 
    irq_nr <  AVALANCHE_INT_END_SECONDARY))
    {
        printk("whee, invalid irq_nr %d\n", irq_nr);
        printk("Not one of the primary avalanche interrupts\n");
        panic("IRQ, you lose...");
        return(-1);
    }

    if(irq_nr >= AVALANCHE_INTC_END)
    {
        if(p_av_vlynq_get_irq_pol_fn)
            value = p_av_vlynq_get_irq_pol_fn(irq_nr);

    	goto ret_from_get_polarity;
    }

    irq_nr = AVINTNUM(irq_nr);

    chan_nr = line_to_channel[irq_nr];

    save_and_cli(flags);

    if(chan_nr < AVALANCHE_INT_END_PRIMARY_REG1)
    {
        value = avalanche_hw0_icregs->intpolr1;
        value = (value >> chan_nr) & 0x1;
    }
    else
    {
        value = avalanche_hw0_icregs->intpolr2;
        value = (value >> (chan_nr - AVALANCHE_INT_END_PRIMARY_REG1)) & 0x1;
    }

    restore_flags(flags);

ret_from_get_polarity:
    return(value);
}

int avalanche_intr_type_set(unsigned int irq_nr, unsigned long type_val)
{
    unsigned long flags;
    unsigned long chan_nr=0;
                                                                                               
    if(irq_nr  <  MIPS_EXCEPTION_OFFSET ||
       irq_nr  >= AVALANCHE_INT_END ||
       (irq_nr >= AVALANCHE_INT_END_PRIMARY &&
    irq_nr <  AVALANCHE_INT_END_SECONDARY))
    {
        printk("whee, invalid irq_nr %d\n", irq_nr);
    panic("IRQ, you lose...");
    return(-1);
    }
                                                                                               
    if(type_val > 1)
    {
        printk("Not a valid polarity value.\n");
    return(-1);
    }
    if(irq_nr >= AVALANCHE_INTC_END)
    {
        if(p_av_vlynq_set_irq_type_fn)
            p_av_vlynq_set_irq_type_fn(irq_nr, type_val);
                                                                                               
        goto ret_from_set_type;
    }
                                                                                               
    irq_nr = AVINTNUM(irq_nr);
                                                                                               
    chan_nr = line_to_channel[AVINTNUM(irq_nr)];
                                                                                               
    save_and_cli(flags);
                                                                                               
    if(chan_nr < AVALANCHE_INT_END_PRIMARY_REG1)
    {
        if(type_val)
            avalanche_hw0_icregs->inttypr1 |=  (1 << chan_nr);
        else
        avalanche_hw0_icregs->inttypr1 &= ~(1 << chan_nr);
    }
    else
    {
    if(type_val)
        avalanche_hw0_icregs->inttypr2 |=
        (1 << (chan_nr - AVALANCHE_INT_END_PRIMARY_REG1));
    else
        avalanche_hw0_icregs->inttypr2 &=
        ~(1 << (chan_nr - AVALANCHE_INT_END_PRIMARY_REG1));
    }
                                                                                               
    restore_flags(flags);
                                                                                               
ret_from_set_type:
    return(0);
}

int avalanche_intr_type_get(unsigned int irq_nr)
{
	unsigned long flags;
	unsigned long chan_nr=0;
	int value = 0;
                                                                                               
    if(irq_nr  <  MIPS_EXCEPTION_OFFSET ||
       irq_nr  >= AVALANCHE_INT_END ||
       (irq_nr >= AVALANCHE_INT_END_PRIMARY &&
    irq_nr <  AVALANCHE_INT_END_SECONDARY))
    {
        printk("whee, invalid irq_nr %d\n", irq_nr);
    panic("IRQ, you lose...");
    return(-1);
    }
                                                                                               
    if(irq_nr >= AVALANCHE_INTC_END)
    {
        if(p_av_vlynq_get_irq_type_fn)
            value = p_av_vlynq_get_irq_type_fn(irq_nr);
            goto ret_from_get_type;
    }
                                                                                               
    irq_nr = AVINTNUM(irq_nr);
                                                                                               
    chan_nr = line_to_channel[AVINTNUM(irq_nr)];
                                                                                               
    save_and_cli(flags);
                                                                                               
    if(chan_nr < AVALANCHE_INT_END_PRIMARY_REG1)
    {
        value = avalanche_hw0_icregs->inttypr1;
        value = (value >> chan_nr) & 0x1;
    }
    else
    {
        value = avalanche_hw0_icregs->inttypr2;
        value = (value >> (chan_nr - AVALANCHE_INT_END_PRIMARY_REG1)) & 0x1;
    }
                                                                                               
    restore_flags(flags);
                                                                                               
ret_from_get_type:
return value;
}

EXPORT_SYMBOL(avalanche_intr_polarity_set);
EXPORT_SYMBOL(avalanche_intr_polarity_get);
