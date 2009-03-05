/*
 * linux/arch/arm/mach-davinci/dma.c
 *
 * TI DaVinci DMA file
 *
 * Copyright (C) 2006 Texas Instruments
 *
 * ----------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 *
 */

#include <linux/sched.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/kernel.h>

#include <asm/io.h>

#include <asm/arch/memory.h>
#include <asm/arch/hardware.h>
#include <asm/arch/irqs.h>
#include <asm/arch/edma.h>
#include <asm/arch/cpu.h>

#ifdef DEBUG
#define DMA_PRINTK(ARGS...)  printk(KERN_INFO "<%s><%d>: ",__FUNCTION__,__LINE__);printk(ARGS)
#define DMA_FN_IN printk(KERN_INFO "[%s]: start\n", __FUNCTION__)
#define DMA_FN_OUT printk(KERN_INFO "[%s]: end\n",__FUNCTION__)
#else
#define DMA_PRINTK( x... )
#define DMA_FN_IN
#define DMA_FN_OUT
#endif


struct edma_map {
	int param1;
	int param2;
};

static unsigned int *edma_channels_arm;
static unsigned char *qdma_channels_arm;
static unsigned int *param_entry_arm;
static unsigned int *tcc_arm;
static unsigned int *param_entry_reserved;

const unsigned int davinci_qdma_ch_map[] = {
	EDMA_DM644X_NUM_PARAMENTRY,
	EDMA_DM646X_NUM_PARAMENTRY,
	EDMA_DM355_NUM_PARAMENTRY,
};

/* SoC specific EDMA3 hardware information, should be provided for a new SoC */
/* DaVinci specific EDMA3 information */
/*
  Each bit field of the elements below indicate the corresponding DMA channel
  availability on EDMA_MASTER_SHADOW_REGION side events
*/
static unsigned int dm644x_edma_channels_arm[EDMA_NUM_DMA_CHAN_DWRDS] = {
	0xFFFFFFFFu,  0xFFFFFFFFu
};

/*
  Each bit field of the elements below indicate the corresponding QDMA channel
  availability on EDMA_MASTER_SHADOW_REGION side events
*/
static unsigned char dm644x_qdma_channels_arm[EDMA_NUM_QDMA_CHAN_DWRDS] = {
	0x00000010u
};

/*
   Each bit field of the elements below indicate corresponding PARAM entry
   availability on EDMA_MASTER_SHADOW_REGION side events
*/
static unsigned int dm644x_param_entry_arm[] = {
	0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu
};

/*
   Each bit field of the elements below indicate corresponding TCC
   availability on EDMA_MASTER_SHADOW_REGION side events
*/
static unsigned int dm644x_tcc_arm[EDMA_NUM_DMA_CHAN_DWRDS] = {
	0xFFFFFFFFu, 0xFFFFFFFFu
};

/*
   Each bit field of the elements below indicate whether the corresponding
    PARAM entry is available for ANY DMA channel or not.
    1- reserved, 0 - not
    (First 64 PaRAM Sets are reserved for 64 DMA Channels)
*/
static unsigned int dm644x_param_entry_reserved[] = {
	0xFFFFFFFFu, 0xFFFFFFFFu, 0x0u, 0x0u
};

static struct edma_map dm644x_queue_priority_mapping[EDMA_DM644X_NUM_EVQUE] = {
	/* {Event Queue No, Priority} */
	{0, 0},
	{1, 1}
};

static struct edma_map dm644x_queue_watermark_level[EDMA_DM644X_NUM_EVQUE] = {
	/* {Event Queue No, Watermark Level} */
	{0, 16},
	{1, 16}
};

static struct edma_map dm644x_queue_tc_mapping[EDMA_DM644X_NUM_EVQUE] = {
	/* {Event Queue No, TC no} */
	{0, 0},
	{1, 1}
};

/* DaVinci-HD specific EDMA3 information */
/*
  Each bit field of the elements below indicate the corresponding DMA channel
  availability on EDMA_MASTER_SHADOW_REGION side events
*/
static unsigned int dm646x_edma_channels_arm[EDMA_NUM_DMA_CHAN_DWRDS] = {
	0x30FF1FF0u,  0x00C007FFu
};

/*
  Each bit field of the elements below indicate the corresponding QDMA channel
  availability on EDMA_MASTER_SHADOW_REGION side events
*/
static unsigned char dm646x_qdma_channels_arm[EDMA_NUM_QDMA_CHAN_DWRDS] = {
	0x00000080
};

/*
   Each bit field of the elements below indicate corresponding PARAM entry
   availability on EDMA_MASTER_SHADOW_REGION side events
*/
static unsigned int dm646x_param_entry_arm[] = {
	0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
	0x0u, 0x0u, 0x0u, 0x0u,
	0x0u, 0x0u, 0x0u, 0x0u,
	0x0u, 0x0u, 0x0u, 0x0u
};

/*
   Each bit field of the elements below indicate corresponding TCC
   availability on EDMA_MASTER_SHADOW_REGION side events
*/
static unsigned int dm646x_tcc_arm[EDMA_NUM_DMA_CHAN_DWRDS] = {
	0x30FF1FF0u, 0x00C007FFu
};

/*
   Each bit field of the elements below indicate whether the corresponding
    PARAM entry is available for ANY DMA channel or not.
    1- reserved, 0 - not
    (First 64 PaRAM Sets are reserved for 64 DMA Channels)
*/
static unsigned int dm646x_param_entry_reserved[] = {
	0xFFFFFFFFu, 0xFFFFFFFFu, 0x0u, 0x0u,
	0x0u, 0x0u, 0x0u, 0x0u,
	0x0u, 0x0u, 0x0u, 0x0u,
	0x0u, 0x0u, 0x0u, 0x0u
};

static struct edma_map dm646x_queue_priority_mapping[EDMA_DM646X_NUM_EVQUE] = {
	/* {Event Queue No, Priority} */
	{0, 0},
	{1, 1},
	{2, 2},
	{3, 3}
};

static struct edma_map dm646x_queue_watermark_level[EDMA_DM646X_NUM_EVQUE] = {
	/* {Event Queue No, Watermark Level} */
	{0, 16},
	{1, 16},
	{2, 16},
	{3, 16}
};

static struct edma_map dm646x_queue_tc_mapping[EDMA_DM646X_NUM_EVQUE] = {
	/* {Event Queue No, TC no} */
	{0, 0},
	{1, 1},
	{2, 2},
	{3, 3},
};


/* DM355 specific EDMA3 information */
/*
  Each bit field of the elements below indicate the corresponding DMA channel
  availability on EDMA_MASTER_SHADOW_REGION side events
*/
static unsigned int dm355_edma_channels_arm[] = {
	0xFFFFFFFFu,
	0x00000000u,
};

/*
  Each bit field of the elements below indicate the corresponding QDMA channel
  availability on EDMA_MASTER_SHADOW_REGION side events
*/
static unsigned char dm355_qdma_channels_arm[EDMA_NUM_QDMA_CHAN_DWRDS] = {
	0x000000FFu
};

/*
   Each bit field of the elements below indicate corresponding PARAM entry
   availability on EDMA_MASTER_SHADOW_REGION side events
*/
static unsigned int dm355_param_entry_arm[] = {
	0xFFFFFFFFu, 0x00000000u, 0x00000000u, 0xFFFFFFC0u,
	0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
	0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
	0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
};

/*
   Each bit field of the elements below indicate corresponding TCC
   availability on EDMA_MASTER_SHADOW_REGION side events
*/
static unsigned int dm355_tcc_arm[EDMA_NUM_DMA_CHAN_DWRDS] = {
	0xFFFFFFFFu, 0xFFFFFFFFu
};

/*
   Each bit field of the elements below indicate whether the corresponding
    PARAM entry is available for ANY DMA channel or not.
    1- reserved, 0 - not
    (First 64 PaRAM Sets are reserved for 64 DMA Channels)
*/
static unsigned int dm355_param_entry_reserved[] = {
	0xFFFFFFFFu, 0xFFFFFFFFu, 0x0u, 0x0u
};

static struct edma_map dm355_queue_priority_mapping[] = {
	/* {Event Queue No, Priority} */
	{0, 0},
	{1, 1},
	{2, 1},
	{3, 1},
	{4, 1},
	{5, 1},
	{6, 1},
	{7, 1},
};

static struct edma_map dm355_queue_watermark_level[] = {
	/* {Event Queue No, Watermark Level} */
	{0, 16},
	{1, 16},
	{2, 16},
	{3, 16},
	{4, 16},
	{5, 16},
	{6, 16},
	{7, 16},
};

static struct edma_map dm355_queue_tc_mapping[] = {
	/* {Event Queue No, TC no} */
	{0, 0},
	{1, 1},
	{2, 2},
	{3, 3},
	{4, 4},
	{5, 5},
	{6, 6},
	{7, 7},
};

static spinlock_t dma_chan_lock;

/**************************************************************************\
* Edma Driver Internal Data Structures
\**************************************************************************/

/*
 * Array to maintain the Callback details registered
 * against a particular TCC. Used to call the callback
 * functions linked to the particular channel.
 */
static struct dma_interrupt_data {
	void (*callback) (int lch, unsigned short ch_status, void *data);
	void *data;
} dma_interrupt_param[EDMA_NUM_TCC];


/*
 * Resources bound to a Logical Channel (DMA/QDMA/LINK)
 *
 * When a request for a channel is made, the resources PaRAM Set and TCC
 * get bound to that channel. This information is needed internally by the
 * driver when a request is made to free the channel (Since it is the
 * responsibility of the driver to free up the channel-associated resources
 * from the Resource Manager layer).
 */
struct edma3_ch_bound_res {
	/** PaRAM Set number associated with the particular channel */
	unsigned int param_id;
	/** TCC associated with the particular channel */
	unsigned int tcc;
};

static struct edma3_ch_bound_res *dma_ch_bound_res;
static int edma_max_logical_ch;
static unsigned int davinci_edma_num_evtq;
static unsigned int davinci_edma_chmap_exist;
static unsigned int davinci_edma_num_tc;
static unsigned int davinci_edma_num_param;
static unsigned int *davinci_edmatc_base_addrs;
static unsigned int *davinci_dma_ch_hw_event_map;

/*
* Mapping of DMA channels to Hardware Events from
* various peripherals, which use EDMA for data transfer.
* All channels need not be mapped, some can be free also.
*/
static unsigned int dm644x_dma_ch_hw_event_map[EDMA_NUM_DMA_CHAN_DWRDS] = {
	EDMA_DM644X_CHANNEL_TO_EVENT_MAPPING_0,
	EDMA_DM644X_CHANNEL_TO_EVENT_MAPPING_1
};

static unsigned int dm355_dma_ch_hw_event_map[EDMA_NUM_DMA_CHAN_DWRDS] = {
	EDMA_DM355_CHANNEL_TO_EVENT_MAPPING_0,
	EDMA_DM355_CHANNEL_TO_EVENT_MAPPING_1
};

static unsigned int dm646x_dma_ch_hw_event_map[EDMA_NUM_DMA_CHAN_DWRDS] = {
	EDMA_DM646X_CHANNEL_TO_EVENT_MAPPING_0,
	EDMA_DM646X_CHANNEL_TO_EVENT_MAPPING_1
};

/*
   Each bit field of the elements below indicate whether a DMA Channel
   is free or in use
   1 - free
   0 - in use
*/
static unsigned int dma_ch_use_status[EDMA_NUM_DMA_CHAN_DWRDS] = {
	0xFFFFFFFFu,
	0xFFFFFFFFu
};

/*
   Each bit field of the elements below indicate whether a intrerrupt
   is free or in use
   1 - free
   0 - in use
*/
static unsigned char qdma_ch_use_status[EDMA_NUM_QDMA_CHAN_DWRDS] = {
	0xFFu
};

/*
   Each bit field of the elements below indicate whether a PARAM entry
   is free or in use
   1 - free
   0 - in use
*/
static unsigned int param_entry_use_status[EDMA_MAX_PARAM_SET/32u] = {
	0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
	0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
	0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
	0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu
};

/*
   Each bit field of the elements below indicate whether a intrerrupt
   is free or in use
   1 - free
   0 - in use
*/
static unsigned long tcc_use_status[EDMA_NUM_DMA_CHAN_DWRDS] = {
	0xFFFFFFFFu,
	0xFFFFFFFFu
};



/*
   Global Array to store the mapping between DMA channels and Interrupt
   channels i.e. TCCs.
   DMA channel X can use any TCC Y. Transfer completion
   interrupt will occur on the TCC Y (IPR/IPRH Register, bit Y), but error
   interrupt will occur on DMA channel X (EMR/EMRH register, bit X). In that
   scenario, this DMA channel <-> TCC mapping will be used to point to
   the correct callback function.
*/
static unsigned int edma_dma_ch_tcc_mapping [EDMA_NUM_DMACH];


/**
    Global Array to store the mapping between QDMA channels and Interrupt
    channels i.e. TCCs.
    QDMA channel X can use any TCC Y. Transfer completion
    interrupt will occur on the TCC Y (IPR/IPRH Register, bit Y), but error
    interrupt will occur on QDMA channel X (QEMR register, bit X). In that
    scenario, this QDMA channel <-> TCC mapping will be used to point to
    the correct callback function.
*/
static unsigned int edma_qdma_ch_tcc_mapping [EDMA_NUM_QDMACH];


/**
 * The list of Interrupt Channels which get allocated while requesting the
 * TCC. It will be used while checking the IPR/IPRH bits in the RM ISR.
 */
static unsigned int allocated_tccs[2u] = {0u, 0u};


/* Pointer to CC Registers */
volatile edmacc_regs *ptr_edmacc_regs = NULL;

/* Array containing physical addresses of all the TCs present */
u32 dm644x_edmatc_base_addrs[EDMA_MAX_TC] = {
	(u32)DAVINCI_DMA_3PTC0_BASE,
	(u32)DAVINCI_DMA_3PTC1_BASE,
	(u32)NULL,
	(u32)NULL,
	(u32)NULL,
	(u32)NULL,
	(u32)NULL,
	(u32)NULL,
};
u32 dm646x_edmatc_base_addrs[EDMA_MAX_TC] = {
	(u32)DAVINCI_DMA_3PTC0_BASE,
	(u32)DAVINCI_DMA_3PTC1_BASE,
	(u32)DAVINCI_DM646X_DMA_3PTC2_BASE,
	(u32)DAVINCI_DM646X_DMA_3PTC3_BASE,
	(u32)NULL,
	(u32)NULL,
	(u32)NULL,
	(u32)NULL,
};
u32 dm355_edmatc_base_addrs[EDMA_MAX_TC] = {
	(u32)DAVINCI_DMA_3PTC0_BASE,
	(u32)DAVINCI_DMA_3PTC1_BASE,
	(u32)NULL,
	(u32)NULL,
	(u32)NULL,
	(u32)NULL,
	(u32)NULL,
	(u32)NULL,
};

/* Array containing the virtual addresses of all the TCs present */
volatile edmatc_regs *ptr_edmatc_regs[EDMA_MAX_TC] = {NULL};

/* Pointer to CC Shadow Region Specific Registers */
volatile edmacc_shadow_regs *ptr_edmacc_shadow_regs = NULL;

/**
 * Variable which will be used internally for referring transfer controllers'
 * error interrupts.
 */
unsigned int dm644x_tc_error_int[EDMA_MAX_TC] = {
	IRQ_TCERRINT0, IRQ_TCERRINT,
	0, 0, 0, 0, 0, 0,
};
unsigned int dm646x_tc_error_int[EDMA_MAX_TC] = {
	IRQ_TCERRINT0, IRQ_TCERRINT,
	IRQ_DM646X_TCERRINT2, IRQ_DM646X_TCERRINT3,
	0, 0, 0, 0,
};
unsigned int dm355_tc_error_int[EDMA_MAX_TC] = {
	IRQ_TCERRINT0, IRQ_TCERRINT,
	0, 0, 0, 0, 0, 0,
};

/**************************************************************************\
* Edma Driver Internal Functions
\**************************************************************************/

/**  EDMA3 TC0 Error Interrupt Handler ISR Routine */

static irqreturn_t dma_tc0_err_handler(int irq, void *dev_id,
					struct pt_regs *data);
/**  EDMA3 TC1 Error Interrupt Handler ISR Routine */
static irqreturn_t dma_tc1_err_handler(int irq, void *dev_id,
					struct pt_regs *data);
/**  EDMA3 TC2 Error Interrupt Handler ISR Routine */
static irqreturn_t dma_tc2_err_handler(int irq, void *dev_id,
					struct pt_regs *data);
/**  EDMA3 TC3 Error Interrupt Handler ISR Routine */
static irqreturn_t dma_tc3_err_handler(int irq, void *dev_id,
					struct pt_regs *data);
/**  EDMA3 TC4 Error Interrupt Handler ISR Routine */
static irqreturn_t dma_tc4_err_handler(int irq, void *dev_id,
					struct pt_regs *data);
/**  EDMA3 TC5 Error Interrupt Handler ISR Routine */
static irqreturn_t dma_tc5_err_handler(int irq, void *dev_id,
					struct pt_regs *data);
/**  EDMA3 TC6 Error Interrupt Handler ISR Routine */
static irqreturn_t dma_tc6_err_handler(int irq, void *dev_id,
					struct pt_regs *data);
/**  EDMA3 TC7 Error Interrupt Handler ISR Routine */
static irqreturn_t dma_tc7_err_handler(int irq, void *dev_id,
					struct pt_regs *data);


/**
  * EDMA3 TC ISRs which need to be registered with the underlying OS by the user
  * (Not all TC error ISRs need to be registered, register only for the
  * available Transfer Controllers).
  */
irqreturn_t (*ptr_edmatc_isrs[EDMA_MAX_TC])(int irq, void *dev_id, 
		struct pt_regs *data) = {
	&dma_tc0_err_handler,
	&dma_tc1_err_handler,
	&dma_tc2_err_handler,
	&dma_tc3_err_handler,
	&dma_tc4_err_handler,
	&dma_tc5_err_handler,
	&dma_tc6_err_handler,
	&dma_tc7_err_handler,
};

/* Function registering different ISRs with the OS */
static int register_dma_interrupts(void);



/*******************************************************************************/
static void map_dma_ch_evt_queue (unsigned int dma_ch, unsigned int evt_queue)
{
    ptr_edmacc_regs->dmaqnum[dma_ch >> 3] &= DMAQNUM_CLR_MASK(dma_ch);
    ptr_edmacc_regs->dmaqnum[dma_ch >> 3] |= DMAQNUM_SET_MASK(dma_ch, evt_queue);
}



static void map_qdma_ch_evt_queue (unsigned int qdma_ch, unsigned int evt_queue)
{
    /* Map QDMA channel to event queue*/
    ptr_edmacc_regs->qdmaqnum &= QDMAQNUM_CLR_MASK(qdma_ch);
    ptr_edmacc_regs->qdmaqnum |= QDMAQNUM_SET_MASK(qdma_ch, evt_queue);
}


static void map_dma_ch_param_set (unsigned int dma_ch, unsigned int param_set)
{

    if (davinci_edma_chmap_exist == 1)  {
    /* Map PaRAM Set Number for specified dma_ch */
        ptr_edmacc_regs->dchmap[dma_ch] &= DMACH_PARAM_CLR_MASK;
        ptr_edmacc_regs->dchmap[dma_ch] |=
                    DMACH_PARAM_SET_MASK(param_set);
    }
}


static void map_qdma_ch_param_set (unsigned int qdma_ch, unsigned int param_set)
{
    /* Map PaRAM Set Number for specified qdma_ch */
    ptr_edmacc_regs->qchmap[qdma_ch] &= QDMACH_PARAM_CLR_MASK;
    ptr_edmacc_regs->qchmap[qdma_ch] |= QDMACH_PARAM_SET_MASK(param_set);

    /* Set CCNT as default Trigger Word */
    ptr_edmacc_regs->qchmap[qdma_ch] &= QDMACH_TRWORD_CLR_MASK;
    ptr_edmacc_regs->qchmap[qdma_ch] |= QDMACH_TRWORD_SET_MASK(QDMA_DEF_TRIG_WORD);
}



static void register_callback(unsigned int tcc,
                        void (*callback) (int lch, unsigned short ch_status,
                                            void *data),
                        void *data)
{
    /* If callback function is not NULL */
    if (callback) {
        if (tcc < 32) {
            ptr_edmacc_shadow_regs->iesr |= (1UL << tcc);

             DMA_PRINTK ("ier = %x \r\n",
                                ptr_edmacc_shadow_regs->ier);

        } else if (tcc < EDMA_NUM_TCC) {
            ptr_edmacc_shadow_regs->iesrh |=
                                    (1UL << (tcc - 32));

             DMA_PRINTK ("ierh = %x \r\n",
                                ptr_edmacc_shadow_regs->ierh);
        } else {
		printk ("WARNING: dma register callback failed - "
			"invalid tcc %d\n", tcc);
		return;
	}

        /* Save the callback function also */
        dma_interrupt_param[tcc].callback = callback;
        dma_interrupt_param[tcc].data = data;
    }
}



static void unregister_callback(unsigned int lch, enum resource_type ch_type)
{
    unsigned int tcc;

    DMA_FN_IN;
    DMA_PRINTK("lch = %d\n", lch);

    switch (ch_type)   {
        case RES_DMA_CHANNEL:
            tcc = edma_dma_ch_tcc_mapping[lch];
            DMA_PRINTK("mapped tcc for DMA channel = %d\n", tcc);
            /* reset */
            edma_dma_ch_tcc_mapping[lch] = EDMA_NUM_TCC;
            break;

        case RES_QDMA_CHANNEL:
            tcc = edma_qdma_ch_tcc_mapping[lch-EDMA_QDMA_CHANNEL_0];
            DMA_PRINTK("mapped tcc for QDMA channel = %d\n", tcc);
            /* reset */
            edma_qdma_ch_tcc_mapping[lch-EDMA_QDMA_CHANNEL_0]=EDMA_NUM_TCC;
            break;

        default:
            return;
    }


    /* Remove the callback function and disable the interrupts */
    if (tcc < 32) {
	ptr_edmacc_shadow_regs->iecr |= (1UL << tcc);
    } else if (tcc < EDMA_NUM_TCC) {
	ptr_edmacc_shadow_regs->iecrh |= (1UL << (tcc - 32));
    } else {
	printk ("WARNING: dma unregister callback failed - invalid "
		"tcc %d on lch %d\n", tcc, lch);
	return;
    }

    if (tcc < EDMA_NUM_TCC)   {
	dma_interrupt_param[tcc].callback = 0;
	dma_interrupt_param[tcc].data = 0;
    }

    DMA_FN_OUT;
}



static int alloc_resource(unsigned int res_id,
                            enum resource_type res_type)
{
    unsigned int avlbl_id = 0;
    int result = -1;
    unsigned int res_id_clear = 0x0;
    unsigned int res_id_set = 0x0;

    res_id_clear = (unsigned int)(~(1u << (res_id % 32u)));
    res_id_set = (1u << (res_id % 32u));

    spin_lock(&dma_chan_lock);

    switch (res_type)   {
        case RES_DMA_CHANNEL :
        {
            if (res_id == EDMA_DMA_CHANNEL_ANY) {
                for (avlbl_id = 0; avlbl_id < EDMA_NUM_DMACH; ++avlbl_id)   {
                    if (((edma_channels_arm[avlbl_id/32]) &
                        (dma_ch_use_status[avlbl_id/32u]) &
                        ~(davinci_dma_ch_hw_event_map[avlbl_id/32u]) &
                        (1u << (avlbl_id % 32u))) != 0)   {
                        /* DMA Channel Available, mark it as unavailable */
                        DMA_PRINTK ("avlbl dma = %x\n", avlbl_id);
                        result = avlbl_id;
                        dma_ch_use_status[avlbl_id/32u] &= (~(1u << (avlbl_id%32u)));

                        /* Enable the DMA channel in the DRAE/DRAEH registers */
                        if (avlbl_id < 32u) {
                            ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].drae
                                |= (0x1u << avlbl_id);
                            DMA_PRINTK ("drae = %x\n",
                                ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].drae);
                        }   else    {
                            ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].draeh
                                |= (0x1u << (avlbl_id - 32u));
                            DMA_PRINTK ("draeh = %x\n",
                                ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].draeh);
                        }

                        break;
                    }
                }
            }
            else if (res_id < EDMA_NUM_DMACH)   {
                if (((edma_channels_arm[res_id/32]) & (res_id_set)) != 0)  {
                    if (((dma_ch_use_status[res_id/32u]) & (res_id_set)) != 0)  {
                        /* Mark it as non-available now */
                        dma_ch_use_status[res_id/32u] &= res_id_clear;

                        if (res_id < 32u)   {
                            /* Enable the DMA channel in the DRAE register */
                            ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].drae
                                |= (0x1u << res_id);
                            DMA_PRINTK ("drae = %x\n",
                                ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].drae);

                            ptr_edmacc_shadow_regs->eecr |= (1 << res_id);
                        } else {
                            /* Enable the DMA channel in the DRAEH register */
                            ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].draeh
                                |= (0x1u << (res_id - 32u));
                            DMA_PRINTK ("draeh = %x\n",
                                ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].draeh);

                            ptr_edmacc_shadow_regs->eecrh
                                                    |= (1 << (res_id - 32u));
                        }
                        result = res_id;
                    }
                }
            }
        }
        break;

    case RES_QDMA_CHANNEL:
        {
            if (res_id == EDMA_QDMA_CHANNEL_ANY) {
                for (avlbl_id = 0; avlbl_id < EDMA_NUM_QDMACH; ++avlbl_id)  {
                    if (((qdma_channels_arm[0]) &
                        (qdma_ch_use_status[0]) &
                        (1u << (avlbl_id % 32u))) != 0)   {
                        /* QDMA Channel Available, mark it as unavailable */
                        DMA_PRINTK ("avlbl qdma = %x\n", avlbl_id);
                        result = avlbl_id;
                        qdma_ch_use_status[0] &= (~(1u << (avlbl_id%32u)));

                        /* Enable the QDMA channel in the QRAE registers */
                        ptr_edmacc_regs->qrae[EDMA_MASTER_SHADOW_REGION]
                            |= (0x1u << avlbl_id);
                        DMA_PRINTK ("qrae = %x\n",
                            ptr_edmacc_regs->qrae[EDMA_MASTER_SHADOW_REGION]);

                        break;
                    }
                }
            }
            else if (res_id < EDMA_NUM_QDMACH)  {
                if (((qdma_channels_arm[0]) & (res_id_set)) != 0)  {
                    if (((qdma_ch_use_status[0]) & (res_id_set)) != 0)  {
                        /* QDMA Channel Available, mark it as unavailable */
                        qdma_ch_use_status[0] &= res_id_clear;

                        /* Enable the QDMA channel in the QRAE registers */
                        ptr_edmacc_regs->qrae[EDMA_MASTER_SHADOW_REGION]
                            |= (0x1u << res_id);
                        DMA_PRINTK ("qrae = %x\n",
                            ptr_edmacc_regs->qrae[EDMA_MASTER_SHADOW_REGION]);

                        result = res_id;
                    }
                }
            }
        }
        break;

    case RES_TCC:
        {
            if (res_id == EDMA_TCC_ANY) {
                for (avlbl_id = 0; avlbl_id < EDMA_NUM_TCC; ++avlbl_id) {
                    if (((tcc_arm[avlbl_id/32]) &
                        (tcc_use_status[avlbl_id/32u]) &
                        ~(davinci_dma_ch_hw_event_map[avlbl_id/32u]) &
                        (1u << (avlbl_id % 32u))) != 0)   {
                        /* TCC Available, mark it as unavailable */
                        DMA_PRINTK ("avlbl tcc = %x\n", avlbl_id);
                        result = avlbl_id;
                        tcc_use_status[avlbl_id/32u] &= (~(1u << (avlbl_id%32u)));

                        /* Enable the TCC in the DRAE/DRAEH registers */
                        if (avlbl_id < 32u) {
                            ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].drae
                                |= (0x1u << avlbl_id);
                            DMA_PRINTK ("drae = %x\n",
                                ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].drae);

                            /* Add it to the Allocated TCCs list */
                            allocated_tccs[0u] |= (0x1u << avlbl_id);
                        }   else    {
                            ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].draeh
                                |= (0x1u << (avlbl_id - 32u));
                            DMA_PRINTK ("draeh = %x\n",
                                ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].draeh);

                            /* Add it to the Allocated TCCs list */
                            allocated_tccs[1u] |= (0x1u << (avlbl_id - 32u));
                        }

                        break;
                    }
                }
            }
            else if (res_id < EDMA_NUM_TCC) {
                if (((tcc_arm[res_id/32]) & (1u << (res_id%32u))) != 0)   {
                    if (((tcc_use_status[res_id/32u]) & (res_id_set)) != 0) {
                        /* Mark it as non-available now */
                        tcc_use_status[res_id/32u] &= res_id_clear;

                        /* Enable the TCC in the DRAE/DRAEH registers */
                        if (res_id < 32u) {
                            ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].drae
                                |= (0x1u << res_id);

                            allocated_tccs[0u] |= (0x1u << res_id);
                        }   else    {
                            ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].draeh
                                |= (0x1u << (res_id - 32u));

                            allocated_tccs[1u] |= (0x1u << (res_id - 32u));
                        }

                        result = res_id;
                    }
                }
            }
        }
        break;

    case RES_PARAM_SET:
        {
            if (res_id == DAVINCI_EDMA_PARAM_ANY)   {
                for (avlbl_id = 0; avlbl_id < davinci_edma_num_param;
			++avlbl_id) {
                    if (((param_entry_arm[avlbl_id/32]) &
                        (param_entry_use_status[avlbl_id/32u]) &
                        ~(param_entry_reserved[avlbl_id/32u]) &
                        (1u << (avlbl_id%32u))) != 0)   {
                        /* PARAM Set Available, mark it as unavailable */
                        DMA_PRINTK ("avlbl param = %x\n", avlbl_id);
                        result = avlbl_id;
                        param_entry_use_status[avlbl_id/32u]
                                                &= (~(1u << (avlbl_id%32u)));

                        /* Also, make the actual PARAM Set NULL */
                        memset((void *)&(ptr_edmacc_regs->paramentry[avlbl_id]),
                                    0x00u,
                                    sizeof(ptr_edmacc_regs->paramentry[avlbl_id]));

                        break;
                    }
                }
            }
            else if (res_id < davinci_edma_num_param)  {
                if (((param_entry_arm[res_id/32])&(1u << (res_id%32u))) != 0) {
                    if (((param_entry_use_status[res_id/32u]) & (res_id_set)) != 0) {
                        /* Mark it as non-available now */
                        param_entry_use_status[res_id/32u] &= res_id_clear;
                        result = res_id;

                        /* Also, make the actual PARAM Set NULL */
                        memset((void *)&(ptr_edmacc_regs->paramentry[res_id]),
                                    0x00u,
                                    sizeof(ptr_edmacc_regs->paramentry[res_id]));
                    }
                }
            }
        }
        break;
    }

    spin_unlock(&dma_chan_lock);

    return result;
}

static void free_resource(unsigned int res_id,
                            enum resource_type res_type)
{
    unsigned int res_id_set = 0x0;

    res_id_set = (1u << (res_id%32u));

    spin_lock(&dma_chan_lock);

    switch (res_type)   {
        case RES_DMA_CHANNEL :
        {
            if (res_id < EDMA_NUM_DMACH)    {
                if (((edma_channels_arm[res_id/32]) & (res_id_set)) != 0)  {
                    if ((~(dma_ch_use_status[res_id/32u]) & (res_id_set)) != 0) {
                        /* Make it as available */
                        dma_ch_use_status[res_id/32u] |= res_id_set;

                        /* Reset the DRAE/DRAEH bit also */
                        if (res_id < 32u) {
                            ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].drae
                                &= (~(0x1u << res_id));
                        }   else    {
                            ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].draeh
                                &= (~(0x1u << (res_id - 32u)));
                        }
                    }
                }
            }
        }
        break;

    case RES_QDMA_CHANNEL:
        {
            if (res_id < EDMA_NUM_QDMACH)   {
                if (((qdma_channels_arm[0]) & (res_id_set)) != 0)  {
                    if ((~(qdma_ch_use_status[0]) & (res_id_set)) != 0) {
                        /* Make it as available */
                        qdma_ch_use_status[0] |= res_id_set;

                        /* Reset the DRAE/DRAEH bit also */
                        ptr_edmacc_regs->qrae[EDMA_MASTER_SHADOW_REGION]
                            &= (~(0x1u << res_id));
                    }
                }
            }
        }
        break;

    case RES_TCC:
        {
            if (res_id < EDMA_NUM_TCC)  {
                if (((tcc_arm[res_id/32]) & (res_id_set)) != 0)  {
                    if ((~(tcc_use_status[res_id/32u]) & (res_id_set)) != 0)   {
                        /* Make it as available */
                        tcc_use_status[res_id/32u] |= res_id_set;

                        /* Reset the DRAE/DRAEH bit also */
                        if (res_id < 32u) {
                            ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].drae
                                &= (~(0x1u << res_id));

                            /* Remove it from the Allocated TCCs list */
                            allocated_tccs[0u] &= (~(0x1u << res_id));
                        }   else    {
                            ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].draeh
                                &= (~(0x1u << (res_id - 32u)));

                            /* Remove it from the Allocated TCCs list */
                            allocated_tccs[1u] &= (~(0x1u << (res_id - 32u)));
                        }
                    }
                }
            }
        }
        break;

    case RES_PARAM_SET:
        {
            if (res_id < davinci_edma_num_param)   {
                if (((param_entry_arm[res_id/32])&(1u << (res_id%32u))) != 0) {
                    if ((~(param_entry_use_status[res_id/32u]) & (res_id_set)) 
			!= 0) {
                        /* Make it as available */
                        param_entry_use_status[res_id/32u] |= res_id_set;
                    }
                }
            }
        }
        break;

    }

    spin_unlock(&dma_chan_lock);
}


/******************************************************************************
 *
 * EDMA3 CC Transfer Completion Interrupt Handler
 *
 *****************************************************************************/
static irqreturn_t dma_irq_handler (int irq, void *dev_id, struct pt_regs *data)
{
    unsigned int cnt = 0;
    volatile unsigned int pendingIrqs = 0;
    unsigned int indexl = 1;
    unsigned int indexh = 1;

    if((ptr_edmacc_shadow_regs->ipr !=0 ) ||
        (ptr_edmacc_shadow_regs->iprh !=0 )) {
        /*Loop while cnt < 10, breaks when no pending interrupt is found*/
        while ((cnt < 10u) && ((indexl != 0u) || (indexh != 0u)))   {
            indexl = 0;
            pendingIrqs = ptr_edmacc_shadow_regs->ipr;

            /**
             * Choose interrupts coming from our allocated TCCs
             * and MASK remaining ones.
             */
            pendingIrqs = (pendingIrqs & allocated_tccs[0u]);

            while (pendingIrqs) {
                /*Process all the pending interrupts*/
                if((pendingIrqs & 1u) == 1)   {
                    /**
                     * If the user has not given any callback function
                     * while requesting the TCC, its TCC specific bit
                     * in the IPR register will NOT be cleared.
                     */
                    if (dma_interrupt_param[indexl].callback) {
                    /* here write to ICR to clear the corresponding IPR bits*/
                        ptr_edmacc_shadow_regs->icr |= (1u << indexl);

                        /* Call the callback function now */
                        dma_interrupt_param[indexl].callback(indexl,
                                            DMA_COMPLETE,
                                            dma_interrupt_param[indexl].data);
                    }
                }
                ++indexl;
                pendingIrqs >>= 1u;
            }

            indexh = 0;
            pendingIrqs = ptr_edmacc_shadow_regs->iprh;

            /**
             * Choose interrupts coming from our allocated TCCs
             * and MASK remaining ones.
             */
            pendingIrqs = (pendingIrqs & allocated_tccs[1u]);

            while (pendingIrqs) {
                /*Process all the pending interrupts*/
                if((pendingIrqs & 1u) == 1)   {
                    /**
                     * If the user has not given any callback function
                     * while requesting the TCC, its TCC specific bit
                     * in the IPRH register will NOT be cleared.
                     */
                    if (dma_interrupt_param[32+indexh].callback) {
                        /* Write to ICRH to clear the corresponding IPRH bits*/
                        ptr_edmacc_shadow_regs->icrh |= (1u << indexh);

                        /* Call the callback function now */
                        dma_interrupt_param[32+indexh].callback(32+indexh,
                                        DMA_COMPLETE,
                                        dma_interrupt_param[32+indexh].data);
                    }
                }
                ++indexh;
                pendingIrqs >>= 1u;
            }

            cnt++;
        }

        ptr_edmacc_shadow_regs->ieval = 0x1;
    }

    return IRQ_HANDLED;
}



/******************************************************************************
 *
 * EDMA3 CC Error Interrupt Handler
 *
 *****************************************************************************/
static irqreturn_t dma_ccerr_handler (int irq, void *dev_id, struct pt_regs *data)
{
    unsigned int cnt = 0;
    volatile unsigned int pendingIrqs = 0;
    unsigned int index = 1;
    unsigned int mapped_tcc = 0;
    unsigned int evtque_num = 0;

    if(((ptr_edmacc_regs->emr != 0 )
        || (ptr_edmacc_regs->emrh != 0 ))
        || ((ptr_edmacc_regs->qemr != 0)
        || (ptr_edmacc_regs->ccerr != 0))) {
        /*Loop while Cnt < 10, breaks when no pending interrupt is found*/
        while ((cnt < 10u) && (index != 0u))    {
            index = 0;
            pendingIrqs = ptr_edmacc_regs->emr;

            while (pendingIrqs) {
                /*Process all the pending interrupts*/
                if((pendingIrqs & 1u) == 1)   {
                    /* Write to EMCR to clear the corresponding EMR bit */
                    ptr_edmacc_regs->emcr |= (1u << index);
                    /*Clear any SER*/
                    ptr_edmacc_shadow_regs->secr |= (1u << index);

                    /**
                     * Using the 'index' value (basically the DMA
                     * channel), fetch the corresponding TCC
                     * value, mapped to this DMA channel.
                     */
                    mapped_tcc = edma_dma_ch_tcc_mapping[index];

                    if (dma_interrupt_param[mapped_tcc].callback) {
                        dma_interrupt_param[mapped_tcc].callback(mapped_tcc,
                                    DMA_EVT_MISS_ERROR,
                                    dma_interrupt_param[mapped_tcc].data);
                    }
                }
                ++index;
                pendingIrqs >>= 1u;
            }

            index = 0;
            pendingIrqs = ptr_edmacc_regs->emrh;

            while (pendingIrqs) {
                /*Process all the pending interrupts*/
                if((pendingIrqs & 1u) == 1)   {
                    /* Write to EMCRH to clear the corresponding EMRH bit */
                    ptr_edmacc_regs->emcrh |= (1u << index);
                    /*Clear any SERH*/
                    ptr_edmacc_shadow_regs->secrh |= (1u << index);

                    /**
                     * Using the 'index' value (basically the DMA
                     * channel), fetch the corresponding TCC
                     * value, mapped to this DMA channel.
                     */
                    mapped_tcc = edma_dma_ch_tcc_mapping[32 + index];

                    if (dma_interrupt_param[mapped_tcc].callback) {
                        dma_interrupt_param[mapped_tcc].callback(mapped_tcc,
                                    DMA_EVT_MISS_ERROR,
                                    dma_interrupt_param[mapped_tcc].data);
                    }
                }
                ++index;
                pendingIrqs >>= 1u;
            }

            index = 0;
            pendingIrqs = ptr_edmacc_regs->qemr;

            while (pendingIrqs) {
                DMA_PRINTK ("pendingIrqs = %x\n", pendingIrqs);

                /*Process all the pending interrupts*/
                if((pendingIrqs & 1u) == 1)   {
                    /* Write to QEMCR to clear the corresponding QEMR bit */
                    ptr_edmacc_regs->qemcr |= (1u << index);
                    /*Clear any QSER*/
                    ptr_edmacc_shadow_regs->qsecr |= (1u << index);

                    /**
                     * Using the 'index' value (basically the QDMA
                     * channel), fetch the corresponding TCC
                     * value, mapped to this QDMA channel.
                     */
                    mapped_tcc = edma_qdma_ch_tcc_mapping[index];
                    DMA_PRINTK ("index = %d, mapped_tcc = %d\n", index, mapped_tcc);

                    if (dma_interrupt_param[mapped_tcc].callback) {
                        dma_interrupt_param[mapped_tcc].callback(mapped_tcc,
                                    QDMA_EVT_MISS_ERROR,
                                    dma_interrupt_param[mapped_tcc].data);
                    }
                }
                ++index;
                pendingIrqs >>= 1u;
            }

            pendingIrqs = ptr_edmacc_regs->ccerr;

            for (evtque_num = 0; evtque_num < davinci_edma_num_evtq;
		evtque_num++) {
                if((pendingIrqs & (1u << evtque_num)) != 0)   {
                    /* Clear the event queue specific error interrupt */
                    ptr_edmacc_regs->ccerrclr |= (1u << evtque_num);
                }
            }

            if (pendingIrqs & (1 << CCRL_CCERR_TCCERR_SHIFT))  {
                ptr_edmacc_regs->ccerrclr |= (1 << CCRL_CCERR_TCCERR_SHIFT);
            }

        cnt++;
        }

        ptr_edmacc_regs->eeval=0x1u;
    }

    return IRQ_HANDLED;
}



/******************************************************************************
 *
 * EDMA3 Transfer Controller Error Interrupt Handler
 *
 *****************************************************************************/
static int dma_tc_err_handler(unsigned int tc_num)
{
	volatile edmatc_regs *tcregs = NULL;

	if (tc_num >= davinci_edma_num_tc) {
		return -EINVAL;
	}

	tcregs = (volatile edmatc_regs *)(ptr_edmatc_regs[tc_num]);

	if (tcregs != NULL) {
	    if(tcregs->errstat) {
		if((tcregs->errstat & (1u << EDMA_TC_ERRSTAT_BUSERR_SHIFT)) != 0) {
			tcregs->errclr = (1u << EDMA_TC_ERRSTAT_BUSERR_SHIFT);
		}

		if((tcregs->errstat & (1 << EDMA_TC_ERRSTAT_TRERR_SHIFT)) != 0) {
			tcregs->errclr = (1 << EDMA_TC_ERRSTAT_TRERR_SHIFT);
		}

		if((tcregs->errstat & (1 << EDMA_TC_ERRSTAT_MMRAERR_SHIFT)) != 0) {
			tcregs->errclr = (1 << EDMA_TC_ERRSTAT_MMRAERR_SHIFT);
		}

	    }
	}

	return 0;
}



/******************************************************************************
 *
 * EDMA3 TC0 Error Interrupt Handler
 *
 *****************************************************************************/
static irqreturn_t dma_tc0_err_handler(int irq, void *dev_id, struct pt_regs *data)
{
	/* Invoke Error Handler ISR for TC0*/
	dma_tc_err_handler (0u);

	return IRQ_HANDLED;
}

/******************************************************************************
 *
 * EDMA3 TC1 Error Interrupt Handler
 *
 *****************************************************************************/
static irqreturn_t dma_tc1_err_handler(int irq, void *dev_id, struct pt_regs *data)
{
	/* Invoke Error Handler ISR for TC1*/
	dma_tc_err_handler (1u);

	return IRQ_HANDLED;
}

/******************************************************************************
 *
 * EDMA3 TC2 Error Interrupt Handler
 *
 *****************************************************************************/
static irqreturn_t dma_tc2_err_handler(int irq, void *dev_id, struct pt_regs *data)
{
	/* Invoke Error Handler ISR for TC2*/
	dma_tc_err_handler (2u);

	return IRQ_HANDLED;
}

/******************************************************************************
 *
 * EDMA3 TC3 Error Interrupt Handler
 *
 *****************************************************************************/
static irqreturn_t dma_tc3_err_handler(int irq, void *dev_id, struct pt_regs *data)
{
	/* Invoke Error Handler ISR for TC3*/
	dma_tc_err_handler (3u);

	return IRQ_HANDLED;
}

/******************************************************************************
 *
 * EDMA3 TC4 Error Interrupt Handler
 *
 *****************************************************************************/
static irqreturn_t dma_tc4_err_handler(int irq, void *dev_id, struct pt_regs *data)
{
	/* Invoke Error Handler ISR for TC4*/
	dma_tc_err_handler (4u);

	return IRQ_HANDLED;
}

/******************************************************************************
 *
 * EDMA3 TC5 Error Interrupt Handler
 *
 *****************************************************************************/
static irqreturn_t dma_tc5_err_handler(int irq, void *dev_id, struct pt_regs *data)
{
	/* Invoke Error Handler ISR for TC5*/
	dma_tc_err_handler (5u);

	return IRQ_HANDLED;
}

/******************************************************************************
 *
 * EDMA3 TC6 Error Interrupt Handler
 *
 *****************************************************************************/
static irqreturn_t dma_tc6_err_handler(int irq, void *dev_id, struct pt_regs *data)
{
	/* Invoke Error Handler ISR for TC6*/
	dma_tc_err_handler (6u);

	return IRQ_HANDLED;
}

/******************************************************************************
 *
 * EDMA3 TC7 Error Interrupt Handler
 *
 *****************************************************************************/
static irqreturn_t dma_tc7_err_handler(int irq, void *dev_id, struct pt_regs *data)
{
	/* Invoke Error Handler ISR for TC7*/
	dma_tc_err_handler (7u);

	return IRQ_HANDLED;
}


/******************************************************************************
 *
 * davinci_get_qdma_channel: Convert qdma channel to logical channel
 * Arguments:
 *      ch     - input qdma channel.
 *
 * Return: logical channel associated with qdma channel or logical channel
 *     associated with qdma channel 0 for out of range channel input.
 *
 *****************************************************************************/
int davinci_get_qdma_channel(int ch)
{
	if ((ch>=0) || (ch <= EDMA_MAX_CHANNEL))
		return (davinci_qdma_ch_map[davinci_cpu_index] + ch);
	else    /* return channel 0 for out of range values */
		return davinci_qdma_ch_map[davinci_cpu_index];
}
EXPORT_SYMBOL(davinci_get_qdma_channel);



/******************************************************************************
 *
 * DMA channel requests: Requests for the dma device passed if it is free
 *
 * Arguments:
 *      dev_id     - request for the param entry device id
 *      dev_name   - device name
 *      callback   - pointer to the channel callback.
 *      Arguments:
 *          lch  - channel no, which is the IPR bit position,
 *         indicating from which channel the interrupt arised.
 *          data - channel private data, which is received as one of the
 *         arguments in davinci_request_dma.
 *      data - private data for the channel to be requested, which is used to
 *                   pass as a parameter in the callback function
 *           in irq handler.
 *      lch - contains the device id allocated
 *  tcc        - Transfer Completion Code, used to set the IPR register bit
 *                   after transfer completion on that channel.
 *  eventq_no  - Event Queue no to which the channel will be associated with
 *               (valied only if you are requesting for a DMA MasterChannel)
 *               Values : 0 to 7
 * INPUT:   dev_id
 * OUTPUT:  *dma_ch_out
 *
 * Return: zero on success, or corresponding error no on failure
 *
 *****************************************************************************/
int davinci_request_dma(int dev_id, const char *dev_name,
            void (*callback) (int lch, unsigned short ch_status,
                      void *data),
            void *data, int *lch,
            int *tcc, enum dma_event_q eventq_no)
{

    int ret_val = 0;
    int param_id = 0;
    int tcc_val = 0;

    DMA_FN_IN;

    /* Validating the arguments passed first */
    if ((!lch) || (!tcc) || (eventq_no >= davinci_edma_num_evtq))   {
        return -EINVAL;
    }

    if (dev_id >= 0u && dev_id < EDMA_NUM_DMACH) {

        if (alloc_resource(dev_id, RES_DMA_CHANNEL) == dev_id)  {
            *lch = dev_id;
            DMA_PRINTK ("DMA channel %d allocated\r\n", *lch);

            /*
                Allocate PaRAM Set.
                The 64 DMA Channels are mapped to the first 64 PARAM entries.
            */
            if (alloc_resource(dev_id, RES_PARAM_SET) == dev_id)    {
                param_id = dev_id;
                DMA_PRINTK ("PaRAM Set %d allocated\r\n", param_id);

                spin_lock(&dma_chan_lock);
                dma_ch_bound_res[dev_id].param_id = param_id;
                spin_unlock(&dma_chan_lock);

                /* Allocate TCC (1-to-1 mapped with the DMA channel) */
                if (alloc_resource(dev_id, RES_TCC) == dev_id)  {
                    *tcc = dev_id;
                    DMA_PRINTK ("TCC %d allocated\r\n", *tcc);

                    spin_lock(&dma_chan_lock);
                    dma_ch_bound_res[dev_id].tcc = *tcc;
                    spin_unlock(&dma_chan_lock);

                    /* all resources allocated */
                    /* Store the mapping b/w DMA channel and TCC first. */
                    edma_dma_ch_tcc_mapping[*lch] = *tcc;

                    /* Register callback function */
                    register_callback((*tcc), callback, data);

                    /* Map DMA channel to event queue */
                    map_dma_ch_evt_queue(*lch, eventq_no);

                    /* Map DMA channel to PaRAM Set */
                    map_dma_ch_param_set(*lch, param_id);

                    ret_val = 0;
                } else {
                    /* TCC allocation failed */
                    /*free previously allocated resources*/
                    free_resource(dev_id, RES_PARAM_SET);
                    spin_lock(&dma_chan_lock);
                    dma_ch_bound_res[dev_id].param_id = 0;
                    spin_unlock(&dma_chan_lock);

                    free_resource(dev_id, RES_DMA_CHANNEL);

            DMA_PRINTK ("TCC allocation failed \r\n");
                    ret_val = -EINVAL;
                }
            } else {
                /* PaRAM Set allocation failed */
                /*free previously allocated resources*/
                free_resource(dev_id, RES_DMA_CHANNEL);

                DMA_PRINTK ("PaRAM Set allocation  failed \r\n");
                ret_val = -EINVAL;
            }
        } else {
            /* Dma channel allocation failed */
            DMA_PRINTK ("DMA channel allocation  failed \r\n");
            ret_val = -EINVAL;
        }
    }   else if (dev_id >= EDMA_QDMA_CHANNEL_0
                && dev_id <= EDMA_QDMA_CHANNEL_7) {
            /**
             * Allocate QDMA channel first.
             * Modify the *lch to point it to the correct QDMA
             * channel and then check whether the same channel
             * has been allocated or not.
             */
            *lch = dev_id - EDMA_QDMA_CHANNEL_0;

            if (alloc_resource((*lch), RES_QDMA_CHANNEL) == (*lch))    {
                /* Requested Channel allocated successfully */
                *lch = dev_id;
                DMA_PRINTK ("QDMA channel %d allocated\r\n", (*lch));

                /* Allocate param set */
                param_id = alloc_resource (DAVINCI_EDMA_PARAM_ANY, RES_PARAM_SET);

                if (param_id != -1) {
                    DMA_PRINTK ("PaRAM Set %d allocated\r\n", param_id);

                    spin_lock(&dma_chan_lock);
                    dma_ch_bound_res[*lch].param_id = param_id;
                    spin_unlock(&dma_chan_lock);

                    /* allocate tcc */
                    tcc_val = alloc_resource(*tcc, RES_TCC);

                    if (tcc_val != -1) {
                        DMA_PRINTK("TCC %d allocated\n", tcc_val);
                        *tcc = tcc_val;

                        spin_lock(&dma_chan_lock);
                        dma_ch_bound_res[*lch].tcc = *tcc;
                        spin_unlock(&dma_chan_lock);

                        /* all resources allocated */
                        /* Store the mapping b/w QDMA channel and TCC first. */
                        edma_qdma_ch_tcc_mapping[(*lch) - EDMA_QDMA_CHANNEL_0]
                                                        = *tcc;

                        /* Register callback function */
                        register_callback((*tcc), callback, data);

                        /* Map QDMA channel to event queue */
                        map_qdma_ch_evt_queue((*lch) - EDMA_QDMA_CHANNEL_0,
                                                eventq_no);

                        /* Map QDMA channel to PaRAM Set */
                        map_qdma_ch_param_set((*lch) - EDMA_QDMA_CHANNEL_0,
                                                param_id);

                        ret_val = 0;
                    } else {
                        /* TCC allocation failed */
                        /*free previously allocated resources*/
                        free_resource(param_id, RES_PARAM_SET);
                        spin_lock(&dma_chan_lock);
                        dma_ch_bound_res[dev_id].param_id = 0;
                        spin_unlock(&dma_chan_lock);

                        free_resource((dev_id-EDMA_QDMA_CHANNEL_0),
                                                        RES_QDMA_CHANNEL);

                        DMA_PRINTK ("TCC channel allocation  failed \r\n");
                        ret_val = -EINVAL;
                    }
                } else {
                    /* PaRAM Set allocation failed. */
                    /*free previously allocated resources*/
                    free_resource((dev_id - EDMA_QDMA_CHANNEL_0),
                                            RES_QDMA_CHANNEL);

                    DMA_PRINTK ("PaRAM channel allocation  failed \r\n");
                    ret_val = -EINVAL;
                }
            } else {
                /* QDMA Channel allocation failed */
                DMA_PRINTK ("QDMA channel allocation  failed \r\n");
                ret_val = -EINVAL;
            }
    } else if (dev_id == EDMA_DMA_CHANNEL_ANY) {
        *lch = alloc_resource(EDMA_DMA_CHANNEL_ANY, RES_DMA_CHANNEL);

        if ((*lch) != -1)   {
            DMA_PRINTK ("EDMA_DMA_CHANNEL_ANY::channel %d allocated\n", (*lch));

            /* Allocate param set tied to the DMA channel (1-to-1 mapping) */
            param_id = alloc_resource ((*lch), RES_PARAM_SET);

            if (param_id != -1) {
                DMA_PRINTK ("EDMA_DMA_CHANNEL_ANY::param %d allocated\n", param_id);

                spin_lock(&dma_chan_lock);
                dma_ch_bound_res[*lch].param_id = param_id;
                spin_unlock(&dma_chan_lock);

                /* allocate tcc */
                *tcc = alloc_resource(*tcc, RES_TCC);

                if (*tcc != -1) {
                    DMA_PRINTK ("EDMA_DMA_CHANNEL_ANY:: tcc %d allocated\n", (*tcc));

                    spin_lock(&dma_chan_lock);
                    dma_ch_bound_res[*lch].tcc = *tcc;
                    spin_unlock(&dma_chan_lock);

                    /* all resources allocated */
                    /* Store the mapping b/w DMA channel and TCC first. */
                    edma_dma_ch_tcc_mapping[*lch] = *tcc;

                    /* Register callback function */
                    register_callback((*tcc), callback, data);

                    /* Map DMA channel to event queue */
                    map_dma_ch_evt_queue(*lch, eventq_no);

                    /* Map DMA channel to PaRAM Set */
                    map_dma_ch_param_set(*lch, param_id);

                    ret_val = 0;
                } else {
                    /* free previously allocated resources */
                    free_resource(param_id, RES_PARAM_SET);
                    spin_lock(&dma_chan_lock);
                    dma_ch_bound_res[*lch].param_id = 0;
                    spin_unlock(&dma_chan_lock);

                    free_resource((*lch), RES_DMA_CHANNEL);

                    DMA_PRINTK ("free resource \r\n");
                    ret_val = -EINVAL;
                }
            } else {
                /*
                    PaRAM Set allocation failed, free previously allocated
                    resources.
                */
                DMA_PRINTK ("PaRAM Set allocation failed \r\n");
                free_resource((*lch), RES_DMA_CHANNEL);
                ret_val = -EINVAL;
            }
        } else {
            DMA_PRINTK ("EINVAL \r\n");
            ret_val = -EINVAL;
        }
    } else if (dev_id == EDMA_QDMA_CHANNEL_ANY) {
        *lch = alloc_resource(dev_id, RES_QDMA_CHANNEL);

        if ((*lch) != -1)   {
            /* Channel allocated successfully */
            *lch = ((*lch) + EDMA_QDMA_CHANNEL_0);

            DMA_PRINTK ("EDMA_QDMA_CHANNEL_ANY::channel %d allocated\n", (*lch));

            /* Allocate param set */
            param_id = alloc_resource (DAVINCI_EDMA_PARAM_ANY, RES_PARAM_SET);

            if (param_id != -1) {
                DMA_PRINTK ("EDMA_QDMA_CHANNEL_ANY::param %d allocated \r\n",
                                                    param_id);

                spin_lock(&dma_chan_lock);
                dma_ch_bound_res[*lch].param_id = param_id;
                spin_unlock(&dma_chan_lock);

                /* allocate tcc */
                tcc_val = alloc_resource(*tcc, RES_TCC);

                if (tcc_val != -1) {
                    DMA_PRINTK ("EDMA_QDMA_CHANNEL_ANY:: tcc %d allocated\n",
                                                                    tcc_val);
                    *tcc = tcc_val;

                    spin_lock(&dma_chan_lock);
                    dma_ch_bound_res[*lch].tcc = *tcc;
                    spin_unlock(&dma_chan_lock);

                    /* all resources allocated */
                    /* Store the mapping b/w QDMA channel and TCC first. */
                    edma_qdma_ch_tcc_mapping[(*lch) - EDMA_QDMA_CHANNEL_0]
                                                             = *tcc;

                    /* Register callback function */
                    register_callback((*tcc), callback, data);

                    /* Map QDMA channel to event queue */
                    map_qdma_ch_evt_queue((*lch) - EDMA_QDMA_CHANNEL_0,
                                            eventq_no);

                    /* Map QDMA channel to PaRAM Set */
                    map_qdma_ch_param_set((*lch) - EDMA_QDMA_CHANNEL_0,
                                            param_id);

                    ret_val = 0;
                } else {
                    /* free previously allocated resources */
                    free_resource(param_id, RES_PARAM_SET);
                    spin_lock(&dma_chan_lock);
                    dma_ch_bound_res[*lch].param_id = 0;
                    spin_unlock(&dma_chan_lock);

                    free_resource((dev_id-EDMA_QDMA_CHANNEL_0),
                                            RES_QDMA_CHANNEL);

                    ret_val = -EINVAL;
                }
            } else {
                /*
                PaRAM Set allocation failed, free previously allocated
                resources.
                */
                free_resource((dev_id - EDMA_QDMA_CHANNEL_0), RES_QDMA_CHANNEL);
                ret_val = -EINVAL;
            }
        } else {
            /* QDMA Channel allocation failed */
            ret_val = -EINVAL;
        }
    } else if (dev_id == DAVINCI_EDMA_PARAM_ANY) {
        /* Allocate a PaRAM Set */
        *lch = alloc_resource(dev_id, RES_PARAM_SET);
        if ((*lch) != -1) {
            DMA_PRINTK ("DAVINCI_EDMA_PARAM_ANY:: link channel %d allocated\n", (*lch));

            /* link channel allocated */
            spin_lock(&dma_chan_lock);
            dma_ch_bound_res[*lch].param_id = *lch;
            spin_unlock(&dma_chan_lock);

            /* assign the link field to NO link. i.e 0xFFFF */
            ptr_edmacc_regs->paramentry[*lch].link_bcntrld |= 0xFFFFu;

            /*
                Check whether user has passed a NULL TCC or not.
                If it is not NULL, use that value to set the OPT.TCC field
                of the link channel and enable the interrupts also.
                Otherwise, disable the interrupts.
            */
            if (*tcc != -1) {
                /* Check for the valid TCC */
                if ((*tcc) < EDMA_NUM_TCC)   {
                    /* Set the OPT.TCC field */
                    ptr_edmacc_regs->paramentry[*lch].opt &= (~TCC);
                    ptr_edmacc_regs->paramentry[*lch].opt |=
                                            ((0x3F & (*tcc)) << 12);

                    /* set TCINTEN bit in PARAM entry */
                    ptr_edmacc_regs->paramentry[*lch].opt |= TCINTEN;

                    /* Store the TCC also */
                    spin_lock(&dma_chan_lock);
                    dma_ch_bound_res[*lch].tcc = *tcc;
                    spin_unlock(&dma_chan_lock);
                } else {
                    /* Invalid TCC passed. */
                    ret_val = -EINVAL;
                    return ret_val;
                }
            } else {
                ptr_edmacc_regs->paramentry[*lch].opt &= ~TCINTEN;
            }

            ret_val = 0;
            return ret_val;
        } else {
            ret_val = -EINVAL;
        }
    } else {
	ret_val = -EINVAL;
    }

    if (!ret_val) {
        if (callback) {
            ptr_edmacc_regs->paramentry[param_id].opt &= (~TCC);
            ptr_edmacc_regs->paramentry[param_id].opt |=
                                    ((0x3F & (*tcc)) << 12);

            /* set TCINTEN bit in PARAM entry */
            ptr_edmacc_regs->paramentry[param_id].opt |= TCINTEN;
        } else {
            ptr_edmacc_regs->paramentry[param_id].opt &= ~TCINTEN;
        }

        /* assign the link field to NO link. i.e 0xFFFF */
        ptr_edmacc_regs->paramentry[param_id].link_bcntrld |= 0xFFFF;
    }

    DMA_FN_OUT;

    return ret_val;
}
EXPORT_SYMBOL(davinci_request_dma);


/******************************************************************************
 *
 * DMA channel free: Free dma channel
 * Arguments:
 *      dev_id     - request for the param entry device id
 *
 * Return: zero on success, or corresponding error no on failure
 *
 *****************************************************************************/
void davinci_free_dma(int lch)
{
    int param_id=0;
    int tcc=0;

    DMA_FN_IN;
    DMA_PRINTK("lch = %d\n", lch);

    if (lch < EDMA_NUM_DMACH)   {
        /* Disable any ongoing transfer first */
        davinci_stop_dma(lch);

        /* Un-register the callback function */
        unregister_callback(lch, RES_DMA_CHANNEL);

        /* Remove DMA channel to PaRAM Set mapping */
        if (davinci_edma_chmap_exist == 1)  {
            ptr_edmacc_regs->dchmap[lch] &= DMACH_PARAM_CLR_MASK;
        }

        param_id = dma_ch_bound_res[lch].param_id;
        tcc = dma_ch_bound_res[lch].tcc;

        DMA_PRINTK("Free ParamSet %d\n",param_id);
        free_resource(param_id, RES_PARAM_SET);
        spin_lock(&dma_chan_lock);
        dma_ch_bound_res[lch].param_id=0;
        spin_unlock(&dma_chan_lock);

        DMA_PRINTK("Free TCC %d\n",tcc);
        free_resource(tcc, RES_TCC);
        spin_lock(&dma_chan_lock);
        dma_ch_bound_res[lch].tcc=0;
        spin_unlock(&dma_chan_lock);

        DMA_PRINTK("Free DMA channel %d\n",lch);
        free_resource(lch, RES_DMA_CHANNEL);
    } else  if (lch >= EDMA_NUM_DMACH && lch < davinci_edma_num_param) {
        param_id = dma_ch_bound_res[lch].param_id;

        DMA_PRINTK("Free LINK channel %d\n", param_id);
        free_resource(param_id, RES_PARAM_SET);
        spin_lock(&dma_chan_lock);
        dma_ch_bound_res[lch].param_id=0;
        spin_unlock(&dma_chan_lock);
    } else if (lch >= EDMA_QDMA_CHANNEL_0 && lch <= EDMA_QDMA_CHANNEL_7) {
        /* Disable any ongoing transfer first */
        davinci_stop_dma(lch);

        /* Un-register the callback function */
        unregister_callback(lch, RES_QDMA_CHANNEL);

        /* Remove QDMA channel to PaRAM Set mapping */
        ptr_edmacc_regs->qchmap[lch-EDMA_QDMA_CHANNEL_0] &= QDMACH_PARAM_CLR_MASK;
        /* Reset trigger word */
        ptr_edmacc_regs->qchmap[lch-EDMA_QDMA_CHANNEL_0] &= QDMACH_TRWORD_CLR_MASK;

        param_id = dma_ch_bound_res[lch].param_id;
        tcc = dma_ch_bound_res[lch].tcc;

        DMA_PRINTK("Free ParamSet %d\n",param_id);
        free_resource(param_id, RES_PARAM_SET);
        spin_lock(&dma_chan_lock);
        dma_ch_bound_res[lch].param_id=0;
        spin_unlock(&dma_chan_lock);

        DMA_PRINTK("Free TCC %d\n",tcc);
        free_resource(tcc, RES_TCC);
        spin_lock(&dma_chan_lock);
        dma_ch_bound_res[lch].tcc=0;
        spin_unlock(&dma_chan_lock);

        DMA_PRINTK("Free QDMA channel %d\n",lch);
        free_resource(lch - EDMA_QDMA_CHANNEL_0, RES_QDMA_CHANNEL);
    }
    DMA_FN_OUT;
}
EXPORT_SYMBOL(davinci_free_dma);


/******************************************************************************
 *
 * DMA source parameters setup
 * ARGUMENTS:
 *      lch         - channel for which the source parameters to be configured
 *      src_port    - Source port address
 *      addressMode - indicates wether addressing mode is fifo.
 *                      (In FIFO mode, address should be 32bytes aligned)
 *
 *****************************************************************************/
void davinci_set_dma_src_params(int lch, unsigned long src_port,
                enum address_mode mode, enum fifo_width width)
{
    DMA_FN_IN;

    if (lch < edma_max_logical_ch)   {
        int param_id = 0;
        param_id = dma_ch_bound_res[lch].param_id;

        if ((mode) && ((src_port & 0x1Fu) != 0))   {
            /* Address in FIFO mode not 32 bytes aligned */
            return;
        }

        /* set the source port address in source register of param structure */
        ptr_edmacc_regs->paramentry[param_id].src = src_port;

        /* set the fifo addressing mode */
        if (mode) {
            /* reset SAM and FWID */
            ptr_edmacc_regs->paramentry[param_id].opt &= (~(SAM | EDMA_FWID));
            /* set SAM and program FWID */
            ptr_edmacc_regs->paramentry[param_id].opt
                                    |= (SAM | ((width & 0x7) << 8));
        }
    }

    DMA_FN_OUT;
}
EXPORT_SYMBOL(davinci_set_dma_src_params);



/******************************************************************************
 *
 * DMA destination parameters setup
 * ARGUMENTS:
 *    lch - channel or param device for destination parameters to be configured
 *    dest_port    - Destination port address
 *    addressMode  - indicates wether addressing mode is fifo.
 *                      (In FIFO mode, address should be 32bytes aligned)
 *
 *****************************************************************************/
void davinci_set_dma_dest_params(int lch, unsigned long dest_port,
                 enum address_mode mode, enum fifo_width width)
{
    DMA_FN_IN;

    if (lch < edma_max_logical_ch)   {
        int param_id = 0;
        param_id=dma_ch_bound_res[lch].param_id;

        if ((mode) && ((dest_port & 0x1Fu) != 0))   {
            /* Address in FIFO mode not 32 bytes aligned */
            return;
        }

        /* set the dest port address in dest register of param structure */
        ptr_edmacc_regs->paramentry[param_id].dst = dest_port;

        /* set the fifo addressing mode */
        if (mode) {
            /* reset DAM and FWID */
            ptr_edmacc_regs->paramentry[param_id].opt &= (~(DAM | EDMA_FWID));
            /* set DAM and program FWID */
            ptr_edmacc_regs->paramentry[param_id].opt
                                    |= (DAM | ((width & 0x7) << 8));
        }
    }

    DMA_FN_OUT;
}
EXPORT_SYMBOL(davinci_set_dma_dest_params);



/******************************************************************************
 *
 * DMA source index setup
 * ARGUMENTS:
 *      lch     - channel or param device for configuration of source index
 *      srcbidx - source B-register index
 *      srccidx - source C-register index
 *
 *****************************************************************************/
void davinci_set_dma_src_index(int lch, short src_bidx, short src_cidx)
{
    DMA_FN_IN;

    if (lch < edma_max_logical_ch) {
        int param_id = 0;

        param_id = dma_ch_bound_res[lch].param_id;

        DMA_PRINTK("lch = %d, param_id = %d\n", lch, param_id);

        ptr_edmacc_regs->paramentry[param_id].src_dst_bidx
                                        &= 0xffff0000;
        ptr_edmacc_regs->paramentry[param_id].src_dst_bidx
                                        |= (src_bidx & 0xFFFFu);

        ptr_edmacc_regs->paramentry[param_id].src_dst_cidx
                                        &= 0xffff0000;
        ptr_edmacc_regs->paramentry[param_id].src_dst_cidx
                                        |= (src_cidx & 0xFFFFu);
    }

    DMA_FN_OUT;
}
EXPORT_SYMBOL(davinci_set_dma_src_index);



/******************************************************************************
 *
 * DMA destination index setup
 * ARGUMENTS:
 *      lch    - channel or param device for configuration of destination index
 *      srcbidx - dest B-register index
 *      srccidx - dest C-register index
 *
 *****************************************************************************/
void davinci_set_dma_dest_index(int lch, short dest_bidx, short dest_cidx)
{
    DMA_FN_IN;

    if (lch < edma_max_logical_ch) {
        int param_id = 0;

        param_id = dma_ch_bound_res[lch].param_id;

        DMA_PRINTK("lch = %d, param_id = %d\n", lch, param_id);

        ptr_edmacc_regs->paramentry[param_id].src_dst_bidx
                                        &= 0x0000ffff;
        ptr_edmacc_regs->paramentry[param_id].src_dst_bidx
                                        |= ((unsigned long)dest_bidx << 16);

        ptr_edmacc_regs->paramentry[param_id].src_dst_cidx
                                        &= 0x0000ffff;
        ptr_edmacc_regs->paramentry[param_id].src_dst_cidx
                                        |= ((unsigned long)dest_cidx << 16);
    }

    DMA_FN_OUT;
}
EXPORT_SYMBOL(davinci_set_dma_dest_index);



/******************************************************************************
 *
 * DMA transfer parameters setup
 * ARGUMENTS:
 *      lch  - channel or param device for configuration of aCount, bCount and
 *         cCount regs.
 *      acnt - acnt register value to be configured
 *      bcnt - bcnt register value to be configured
 *      ccnt - ccnt register value to be configured
 *
 *****************************************************************************/
void davinci_set_dma_transfer_params(int lch, unsigned short acnt,
                     unsigned short bcnt, unsigned short ccnt,
                     unsigned short bcntrld,
                     enum sync_dimension sync_mode)
{
        int param_id = 0;
    DMA_FN_IN;

    if (lch < edma_max_logical_ch) {

        param_id = dma_ch_bound_res[lch].param_id;

        DMA_PRINTK("lch = %d, param_id = %d\n", lch, param_id);

        ptr_edmacc_regs->paramentry[param_id].link_bcntrld
                                        &= 0x0000ffff;
        ptr_edmacc_regs->paramentry[param_id].link_bcntrld
                                        |= ((bcntrld & 0xFFFFu) << 16);

        if (sync_mode == ASYNC)     {
            ptr_edmacc_regs->paramentry[param_id].opt
                &= (~SYNCDIM);
        } else {
            ptr_edmacc_regs->paramentry[param_id].opt
                |= SYNCDIM;
        }

        /* Set the acount, bcount, ccount registers */
        ptr_edmacc_regs->paramentry[param_id].a_b_cnt =
            (((bcnt & 0xFFFFu) << 16) | acnt);
        ptr_edmacc_regs->paramentry[param_id].ccnt = ccnt;
    }

    DMA_FN_OUT;
}
EXPORT_SYMBOL(davinci_set_dma_transfer_params);



/******************************************************************************
 *
 * davinci_set_dma_params -
 * ARGUMENTS:
 *      lch - logical channel number
 *
 *****************************************************************************/
void davinci_set_dma_params(int lch, edmacc_paramentry_regs *new_param)
{
            int param_id = 0;
    DMA_FN_IN;

    if (new_param)   {
        if (lch < edma_max_logical_ch) {

            param_id = dma_ch_bound_res[lch].param_id;

            memcpy((void *)(&(ptr_edmacc_regs->paramentry[param_id].opt)),
                   (void *)new_param, sizeof(edmacc_paramentry_regs));
        }
    }

    DMA_FN_OUT;
}
EXPORT_SYMBOL(davinci_set_dma_params);



/******************************************************************************
 *
 * davinci_get_dma_params -
 * ARGUMENTS:
 *      lch - logical channel number
 *
 *****************************************************************************/
void davinci_get_dma_params(int lch, edmacc_paramentry_regs *curr_param)
{
    DMA_FN_IN;

    if (curr_param)  {
        if (lch < edma_max_logical_ch) {
            int param_id = 0;

            param_id = dma_ch_bound_res[lch].param_id;


            memcpy((void *)curr_param,
                    (void *)(&(ptr_edmacc_regs->paramentry[param_id].opt)),
                        sizeof(edmacc_paramentry_regs));
        }
    }

    DMA_FN_OUT;
}
EXPORT_SYMBOL(davinci_get_dma_params);



/******************************************************************************
 *
 * DMA Start - Starts the dma on the channel passed
 * ARGUMENTS:
 *      lch - logical channel number
 *
 *****************************************************************************/
int davinci_start_dma(int lch)
{
    int ret_val = 0;
    DMA_FN_IN;

    DMA_PRINTK ("lch %d edma_max_logical_ch %d\n", lch,edma_max_logical_ch);
    if (lch < edma_max_logical_ch)   {
        if (lch >= 0 && lch < EDMA_NUM_DMACH)   {
            /* DMA Channel */
                DMA_PRINTK ("davinci_dma_ch_hw_event_map 0x%8x \n",davinci_dma_ch_hw_event_map[lch/32u]);
                DMA_PRINTK ("lch/32u  %i  lch-32u %i\n",lch/32u,(lch % 32u));
            if (((davinci_dma_ch_hw_event_map[lch/32u]) &
                    (1u << (lch % 32u))) != 0)    {
               DMA_PRINTK ("Event mapped\n");
                /* Event Mapped */
                if (lch < 32)   {
                    DMA_PRINTK ("ER = %x\r\n", ptr_edmacc_shadow_regs->er);

                    /* Clear any SER */
                    ptr_edmacc_shadow_regs->secr = (1UL << lch);

                    /* Clear any pending error */
                    ptr_edmacc_regs->emcr = (1UL << lch);

                    /* Set EER */
                    ptr_edmacc_shadow_regs->eesr = (1UL << lch);

                    ptr_edmacc_shadow_regs->ecr = (1UL << lch);
                } else {
                    DMA_PRINTK ("ERH = %x\r\n", ptr_edmacc_shadow_regs->erh);

                    /* Clear any pending error */
                    ptr_edmacc_regs->emcrh = (1UL << (lch-32));

                    /* Clear any SERH */
                    ptr_edmacc_shadow_regs->secrh = (1UL << (lch-32));

                    /* Set EERH */
                    ptr_edmacc_shadow_regs->eesrh = (1UL << (lch-32));

                    ptr_edmacc_shadow_regs->ecrh = (1UL << (lch-32));
                }
            } else {
                /* Manual Triggered */
                if (lch < 32)   {
                    ptr_edmacc_shadow_regs->esr = (1UL << lch);
                } else {
                    ptr_edmacc_shadow_regs->esrh = (1UL << (lch-32));
                }
            }
        } else {
            /* QDMA Channel */
            ptr_edmacc_shadow_regs->qeesr =
                                    (1u << (lch - EDMA_QDMA_CHANNEL_0));
        }
    } else {
        DMA_PRINTK ("EINVAL lch %d \n", lch);
        ret_val = EINVAL;
    }

    DMA_FN_OUT;

    return ret_val;
}
EXPORT_SYMBOL(davinci_start_dma);


/******************************************************************************
 *
 * DMA Stop - Stops the dma on the channel passed
 * ARGUMENTS:
 *      lch - logical channel number
 *
 *****************************************************************************/
void davinci_stop_dma(int lch)
{
    DMA_FN_IN;

    if (lch < edma_max_logical_ch)   {
        if (lch >= 0 && lch < EDMA_NUM_DMACH)   {
            /* DMA Channel */
            if (((davinci_dma_ch_hw_event_map[lch/32u]) &
                    (1u << (lch%32u))) != 0)    {
                /* Event Mapped */
                if (lch < 32)   {
                    ptr_edmacc_shadow_regs->eecr = (1UL << lch);
                    if (ptr_edmacc_shadow_regs->er & (1 << lch)) {
                        DMA_PRINTK ("ER=%x\n", ptr_edmacc_shadow_regs->er);

                        ptr_edmacc_shadow_regs->ecr = (1 << lch);
                    }

                    if (ptr_edmacc_shadow_regs->ser & (1 << lch)) {
                        DMA_PRINTK ("SER=%x\n", ptr_edmacc_shadow_regs->ser);

                        ptr_edmacc_shadow_regs->secr = (1 << lch);
                    }

                    if (ptr_edmacc_regs->emr & (1 << lch)) {
                        DMA_PRINTK ("EMR=%x\n", ptr_edmacc_regs->emr);

                        ptr_edmacc_regs->emcr = (1 << lch);
                    }
                } else {
                    ptr_edmacc_shadow_regs->eecrh = (1UL << (lch-32));
                    if (ptr_edmacc_shadow_regs->erh & (1 << (lch-32))) {
                        DMA_PRINTK ("ERH=%x\n", ptr_edmacc_shadow_regs->erh);

                        ptr_edmacc_shadow_regs->ecrh = (1 << (lch-32));
                    }

                    if (ptr_edmacc_shadow_regs->serh & (1 << (lch-32))) {
                        DMA_PRINTK ("SERH=%x\n", ptr_edmacc_shadow_regs->serh);

                        ptr_edmacc_shadow_regs->secrh = (1 << (lch-32));
                    }

                    if (ptr_edmacc_regs->emrh & (1 << (lch-32))) {
                        DMA_PRINTK ("EMRH=%x\n", ptr_edmacc_regs->emrh);

                        ptr_edmacc_regs->emcrh = (1 << (lch-32));
                    }
                }
            } else {    /* Manual Triggered */
                if (lch < 32)   {
                    DMA_PRINTK ("ESR=%x\r\n", ptr_edmacc_shadow_regs->esr);

                    if (ptr_edmacc_shadow_regs->ser & (1 << lch)) {
                        DMA_PRINTK ("SER=%x\n", ptr_edmacc_shadow_regs->ser);

                        ptr_edmacc_shadow_regs->secr = (1 << lch);
                    }

                    if (ptr_edmacc_regs->emr & (1 << lch)) {
                        DMA_PRINTK ("EMR=%x\n", ptr_edmacc_regs->emr);

                        ptr_edmacc_regs->emcr = (1 << lch);
                    }
                } else {
                    DMA_PRINTK ("ESRH=%x\r\n", ptr_edmacc_shadow_regs->esrh);

                    if (ptr_edmacc_shadow_regs->serh & (1 << (lch-32))) {
                        DMA_PRINTK ("SERH=%x\n", ptr_edmacc_shadow_regs->serh);

                        ptr_edmacc_shadow_regs->secrh = (1 << (lch-32));
                    }

                    if (ptr_edmacc_regs->emrh & (1 << (lch-32))) {
                        DMA_PRINTK ("EMRH=%x\n", ptr_edmacc_regs->emrh);

                        ptr_edmacc_regs->emcrh = (1 << (lch-32));
                    }
                }
            }
        } else if ((lch <= EDMA_QDMA_CHANNEL_7)&&(lch >= EDMA_QDMA_CHANNEL_0) ) {
            /* QDMA Channel */
            ptr_edmacc_shadow_regs->qeecr = (1 << (lch - EDMA_QDMA_CHANNEL_0));
        }
    }

    DMA_FN_OUT;
}
EXPORT_SYMBOL(davinci_stop_dma);



/******************************************************************************
 *
 * DMA channel link - link the two logical channels passed through by linking
 *                    the link field of head to the param pointed by the lch_queue.
 * ARGUMENTS:
 *      lch_head  - logical channel number, in which the link field is linked
 *                  to the param pointed to by lch_queue
 * lch_queue - logical channel number or the param entry number, which is to be
 *                  linked to the lch_head
 *
 *****************************************************************************/
void davinci_dma_link_lch(int lch_head, int lch_queue)
{
    DMA_FN_IN;

    if ((lch_head < edma_max_logical_ch)
        || (lch_queue < edma_max_logical_ch))    {
        unsigned int param1_id = 0;
        unsigned int param2_id = 0;
        unsigned long link;

        param1_id = dma_ch_bound_res[lch_head].param_id;
        param2_id = dma_ch_bound_res[lch_queue].param_id;

        link = (unsigned long)(&(ptr_edmacc_regs->paramentry[param2_id].opt));

        ptr_edmacc_regs->paramentry[param1_id].link_bcntrld &= 0xffff0000;
        ptr_edmacc_regs->paramentry[param1_id].link_bcntrld |= ((unsigned short)link);
    }

    DMA_FN_OUT;
}
EXPORT_SYMBOL(davinci_dma_link_lch);



/******************************************************************************
 *
 * DMA channel unlink - unlink the two logical channels passed through by
 *                   setting the link field of head to 0xffff.
 * ARGUMENTS:
 * lch_head - logical channel number, from which the link field is to be removed
 * lch_queue - logical channel number or the param entry number, which is to be
 *             unlinked from lch_head
 *
 *****************************************************************************/
void davinci_dma_unlink_lch(int lch_head, int lch_queue)
{
    DMA_FN_IN;

    if ((lch_head < edma_max_logical_ch)
        || (lch_queue < edma_max_logical_ch))    {
        unsigned int param_id = 0;

        param_id = dma_ch_bound_res[lch_head].param_id;

        ptr_edmacc_regs->paramentry[param_id].link_bcntrld |= 0xFFFF;
    }

    DMA_FN_OUT;
}
EXPORT_SYMBOL(davinci_dma_unlink_lch);



/******************************************************************************
 *
 * DMA channel chain - chains the two logical channels passed through by
 * ARGUMENTS:
 * lch_head - logical channel number, which will trigger the chained channel
 *              'lch_queue'
 * lch_queue - logical channel number which will be triggered by 'lch_head'
 *
 *****************************************************************************/
void davinci_dma_chain_lch(int lch_head, int lch_queue)
{
    DMA_FN_IN;

    if ((lch_head < edma_max_logical_ch)
        || (lch_queue < edma_max_logical_ch))    {
        unsigned int param_id = 0;

        param_id = dma_ch_bound_res[lch_head].param_id;

        /* set TCCHEN */
        ptr_edmacc_regs->paramentry[param_id].opt |= TCCHEN;

        /* program tcc */
        ptr_edmacc_regs->paramentry[param_id].opt &= (~TCC);
        ptr_edmacc_regs->paramentry[param_id].opt |= (lch_queue & 0x3f) << 12;
    }

    DMA_FN_OUT;
}
EXPORT_SYMBOL(davinci_dma_chain_lch);



/******************************************************************************
 *
 * DMA channel unchain - unchain the two logical channels passed through by
 * ARGUMENTS:
 * lch_head - logical channel number, from which the link field is to be removed
 * lch_queue - logical channel number or the param entry number, which is to be
 *             unlinked from lch_head
 *
 *****************************************************************************/
void davinci_dma_unchain_lch(int lch_head, int lch_queue)
{
    DMA_FN_IN;

    if ((lch_head < edma_max_logical_ch)
        || (lch_queue < edma_max_logical_ch))    {
        unsigned int param_id = 0;

        param_id = dma_ch_bound_res[lch_head].param_id;

        /* reset TCCHEN */
        ptr_edmacc_regs->paramentry[param_id].opt &= ~TCCHEN;
        /* reset ITCCHEN */
        ptr_edmacc_regs->paramentry[param_id].opt &= ~ITCCHEN;
    }

    DMA_FN_OUT;
}
EXPORT_SYMBOL(davinci_dma_unchain_lch);



/******************************************************************************
 *
 * It cleans ParamEntry qand bring back EDMA to initial state if media has
 * been removed before EDMA has finished.It is usedful for removable media.
 * Arguments:
 *      ch_no     - channel no
 *
 * Return: zero on success, or corresponding error no on failure
 *
 *****************************************************************************/

void davinci_clean_channel(int ch_no)
{
    unsigned int count;
    unsigned int value = 0;

    DMA_FN_IN;

    if (ch_no < 32) {
        DMA_PRINTK ("EMR = %x\r\n", ptr_edmacc_regs->emr);

        ptr_edmacc_shadow_regs->ecr = (1 << ch_no);

        /* Clear the corresponding EMR bits */
        ptr_edmacc_regs->emcr = (1 << ch_no);

        /* Clear any SER */
        ptr_edmacc_shadow_regs->secr = (1 << ch_no);

        /* Clear any EER */
        ptr_edmacc_shadow_regs->eecr = (1 << ch_no);
    }

    if (ch_no > 32) {
        DMA_PRINTK ("EMRH = %x\r\n", ptr_edmacc_regs->emrh);

        ptr_edmacc_shadow_regs->ecrh = (1 << (ch_no-32));

        /* Clear the corresponding EMRH bits */
        ptr_edmacc_regs->emcrh = (1 << (ch_no-32));

        /* Clear any SER */
        ptr_edmacc_shadow_regs->secrh = (1 << (ch_no-32));

        /* Clear any EERH */
        ptr_edmacc_shadow_regs->eecrh = (1 << (ch_no-32));
    }

    for (count = 0; count < davinci_edma_num_evtq; count++) {
	value |= (1u << count);
    }

    ptr_edmacc_regs->ccerrclr = ((1 << 16) | value);

    DMA_FN_OUT;
}
EXPORT_SYMBOL(davinci_clean_channel);



/******************************************************************************
 *
 * EDMA3 Initialisation on DaVinci
 *
 *****************************************************************************/
int __init arch_dma_init(void)
{
    struct edma_map *q_pri, *q_wm, *q_tc;
    unsigned int i = 0u;

    ptr_edmacc_regs = (edmacc_regs *) IO_ADDRESS(EDMA_CC_BASE_ADDRESS);
    DMA_PRINTK ("DMA CC REG BASE ADDR=%x\n", (unsigned int)ptr_edmacc_regs);

    if(cpu_is_davinci_dm6467()) {
	davinci_edma_num_evtq = EDMA_DM646X_NUM_EVQUE;
	davinci_edma_chmap_exist = EDMA_DM646X_CHMAP_EXIST;
	davinci_edma_num_tc = EDMA_DM646X_NUM_TC;
	davinci_edmatc_base_addrs = dm646x_edmatc_base_addrs;
	edma_max_logical_ch = EDMA_NUM_QDMACH + EDMA_DM646X_NUM_PARAMENTRY;
	davinci_edma_num_param = EDMA_DM646X_NUM_PARAMENTRY;
	davinci_dma_ch_hw_event_map = dm646x_dma_ch_hw_event_map;

	edma_channels_arm = dm646x_edma_channels_arm;
	qdma_channels_arm = dm646x_qdma_channels_arm;
	param_entry_arm = dm646x_param_entry_arm;
	tcc_arm = dm646x_tcc_arm;
	param_entry_reserved = dm646x_param_entry_reserved;

	q_pri = dm646x_queue_priority_mapping;
	q_tc = dm646x_queue_tc_mapping;
	q_wm = dm646x_queue_watermark_level;

    } else if (cpu_is_davinci_dm355()) {
	davinci_edma_num_evtq = EDMA_DM355_NUM_EVQUE;
	davinci_edma_chmap_exist = EDMA_DM355_CHMAP_EXIST;
	davinci_edma_num_tc = EDMA_DM355_NUM_TC;
	davinci_edmatc_base_addrs = dm355_edmatc_base_addrs;
	edma_max_logical_ch = EDMA_NUM_QDMACH + EDMA_DM355_NUM_PARAMENTRY;
	davinci_edma_num_param = EDMA_DM355_NUM_PARAMENTRY;
	davinci_dma_ch_hw_event_map = dm355_dma_ch_hw_event_map;

	edma_channels_arm = dm355_edma_channels_arm;
	qdma_channels_arm = dm355_qdma_channels_arm;
	param_entry_arm = dm355_param_entry_arm;
	tcc_arm = dm355_tcc_arm;
	param_entry_reserved = dm355_param_entry_reserved;

	q_pri = dm355_queue_priority_mapping;
	q_tc = dm355_queue_tc_mapping;
	q_wm = dm355_queue_watermark_level;
    } else {
	davinci_edma_num_evtq = EDMA_DM644X_NUM_EVQUE;
	davinci_edma_chmap_exist = EDMA_DM644X_CHMAP_EXIST;
	davinci_edma_num_tc = EDMA_DM644X_NUM_TC;
	davinci_edmatc_base_addrs = dm644x_edmatc_base_addrs;
	edma_max_logical_ch = EDMA_NUM_QDMACH + EDMA_DM644X_NUM_PARAMENTRY;
	davinci_edma_num_param = EDMA_DM644X_NUM_PARAMENTRY;
	davinci_dma_ch_hw_event_map = dm644x_dma_ch_hw_event_map;

	edma_channels_arm = dm644x_edma_channels_arm;
	qdma_channels_arm = dm644x_qdma_channels_arm;
	param_entry_arm = dm644x_param_entry_arm;
	tcc_arm = dm644x_tcc_arm;
	param_entry_reserved = dm644x_param_entry_reserved;

	q_pri = dm644x_queue_priority_mapping;
	q_tc = dm644x_queue_tc_mapping;
	q_wm = dm644x_queue_watermark_level;
    }
    dma_ch_bound_res =
        kmalloc(sizeof(struct edma3_ch_bound_res) * edma_max_logical_ch,
             GFP_KERNEL);

    for (i = 0; i < davinci_edma_num_tc; i++) {
        ptr_edmatc_regs[i] = (volatile edmatc_regs *)
                IO_ADDRESS(davinci_edmatc_base_addrs[i]);
        DMA_PRINTK ("DMA TC[%d] REG BASE ADDR=%x\n", i,
                (unsigned int)ptr_edmatc_regs[i]);
    }

    /* Reset global data */
    /* Reset the DCHMAP registers if they exist */
    if (davinci_edma_chmap_exist == 1) {
        memset((void *)&(ptr_edmacc_regs->dchmap[0u]), 0x00u,
           sizeof(ptr_edmacc_regs->dchmap));
    }

    /* Reset book-keeping info */
    memset(dma_ch_bound_res, 0x00u,  (sizeof(struct edma3_ch_bound_res) *
		edma_max_logical_ch));
    memset(dma_interrupt_param, 0x00u,  sizeof(dma_interrupt_param));
    memset(edma_dma_ch_tcc_mapping, 0x00u,  sizeof(edma_dma_ch_tcc_mapping));
    memset(edma_qdma_ch_tcc_mapping, 0x00u,  sizeof(edma_qdma_ch_tcc_mapping));

    memset((void *)&(ptr_edmacc_regs->paramentry[0u]), 0x00u,
           sizeof(ptr_edmacc_regs->paramentry));

    /* Clear Error Registers */
    ptr_edmacc_regs->emcr = 0xFFFFFFFFu;
    ptr_edmacc_regs->emcrh = 0xFFFFFFFFu;
    ptr_edmacc_regs->qemcr = 0xFFFFFFFFu;
    ptr_edmacc_regs->ccerrclr = 0xFFFFFFFFu;

    i = 0u;
    while (i < davinci_edma_num_evtq) {
        /* Event Queue to TC mapping, if it exists */
        if (EDMA_EVENT_QUEUE_TC_MAPPING == 1u) {
            ptr_edmacc_regs->quetcmap &= QUETCMAP_CLR_MASK(q_tc[i].param1);
            ptr_edmacc_regs->quetcmap |= QUETCMAP_SET_MASK(q_tc[i].param1,
                                                           q_tc[i].param2);
        }

        /* Event Queue Priority */
        ptr_edmacc_regs->quepri &= QUEPRI_CLR_MASK(q_pri[i].param1);
        ptr_edmacc_regs->quepri |= QUEPRI_SET_MASK(q_pri[i].param1,
                                                   q_pri[i].param2);

        /* Event Queue Watermark Level */
        ptr_edmacc_regs->qwmthra &= QUEWMTHR_CLR_MASK(q_wm[i].param1);
        ptr_edmacc_regs->qwmthra |= QUEWMTHR_SET_MASK(q_wm[i].param1,
                                                      q_wm[i].param2);

        i++;
    }

    /* Reset the Allocated TCCs Array first. */
    allocated_tccs[0u] = 0x0u;
    allocated_tccs[1u] = 0x0u;

    /* Point to the Master Shadow Region for later use */
    ptr_edmacc_shadow_regs = (edmacc_shadow_regs *)&(ptr_edmacc_regs->shadow
                                        [EDMA_MASTER_SHADOW_REGION]);
    DMA_PRINTK ("\n\nShadow REG BASE ADDR = %x\n\n",
		(unsigned int)ptr_edmacc_shadow_regs);
    if (!ptr_edmacc_shadow_regs) {
        /* Bad address */
        return -EFAULT;
    }

    /* Clear region specific Shadow Registers */
    ptr_edmacc_shadow_regs->ecr = (edma_channels_arm[0] | tcc_arm[0]);
    ptr_edmacc_shadow_regs->ecrh = (edma_channels_arm[1] | tcc_arm[1]);
    ptr_edmacc_shadow_regs->eecr = (edma_channels_arm[0] | tcc_arm[0]);
    ptr_edmacc_shadow_regs->eecrh = (edma_channels_arm[1] | tcc_arm[1]);
    ptr_edmacc_shadow_regs->secr = (edma_channels_arm[0] | tcc_arm[0]);
    ptr_edmacc_shadow_regs->secrh = (edma_channels_arm[1] | tcc_arm[1]);
    ptr_edmacc_shadow_regs->iecr = (edma_channels_arm[0] | tcc_arm[0]);
    ptr_edmacc_shadow_regs->iecrh = (edma_channels_arm[1] | tcc_arm[1]);
    ptr_edmacc_shadow_regs->icr = (edma_channels_arm[0] | tcc_arm[0]);
    ptr_edmacc_shadow_regs->icrh = (edma_channels_arm[1] | tcc_arm[1]);
    ptr_edmacc_shadow_regs->qeecr = (qdma_channels_arm[0]);
    ptr_edmacc_shadow_regs->qsecr = (qdma_channels_arm[0]);


    /* Reset Region Access Enable Registers for the Master Shadow Region */
    ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].drae = 0x0u;
    ptr_edmacc_regs->dra[EDMA_MASTER_SHADOW_REGION].draeh = 0x0u;
    ptr_edmacc_regs->qrae[EDMA_MASTER_SHADOW_REGION] = 0x0u;

    if (register_dma_interrupts())
        return -EINVAL;


    spin_lock_init(&dma_chan_lock);

    return 0;
}


/* Register different ISRs with the underlying OS */
int register_dma_interrupts(void)
{
    int result = 0;
    int i;
    unsigned int *tc_error_int;

    if (cpu_is_davinci_dm6467())
        tc_error_int = dm646x_tc_error_int;
    else if (cpu_is_davinci_dm355())
        tc_error_int = dm355_tc_error_int;
    else
        tc_error_int = dm644x_tc_error_int;

    result = request_irq (EDMA_XFER_COMPLETION_INT, dma_irq_handler, 0,
                    "EDMA Completion", NULL);
    if (result < 0) {
        DMA_PRINTK ("request_irq failed for dma_irq_handler, error=%d\n",
                                                        result);
        return result;
    }

    result = request_irq (EDMA_CC_ERROR_INT, dma_ccerr_handler, 0,
                    "EDMA CC Err", NULL);
    if (result < 0) {
        DMA_PRINTK ("request_irq failed for dma_ccerr_handler, error=%d\n",
                                                        result);
        return result;
    }

    for (i = 0; i < davinci_edma_num_tc; i++)      {
            result = request_irq (tc_error_int[i], ptr_edmatc_isrs[i], 0,
                            "EDMA TC Error", NULL);
            if (result < 0) {
                DMA_PRINTK ("request_irq failed for dma_tc%d err_handler\n", i);
                DMA_PRINTK ("error = %d \n", result);
                return result;
            }
    }

    return result;
}



arch_initcall(arch_dma_init);


MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");

