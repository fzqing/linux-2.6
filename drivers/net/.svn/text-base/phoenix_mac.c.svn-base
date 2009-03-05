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
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/socket.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/in.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/cache.h>

#include <asm/rmi/debug.h>
#include <asm/rmi/pic.h>
#include <asm/rmi/phoenix_mac.h>
#include <asm/rmi/mips-exts.h>
#include <asm/rmi/msgring.h>

#include <asm/rmi/sim.h>
#include <asm/rmi/rmios_user_mac.h>
#include <asm/rmi/phnx_user_mac.h>

#include <linux/inet.h>
#include <linux/netdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include <linux/if_ether.h>	/* For the statistics structure. */
#include <linux/if_arp.h>	/* For ARPHRD_ETHER */


#define DRV_NAME	"rmi_phnx_mac"
#define DRV_VERSION	"0.1"
#define DRV_RELDATE	"10Feb2004"


#ifdef DEBUG
int mac_debug = 1;
#undef dbg_msg
#define phnx_dbg_msg(fmt, args...) \
	    do {\
	        if (mac_debug) {\
	            printk("[%s@%d|%s]: cpu_%d: " fmt, \
	            __FILE__, __LINE__, __FUNCTION__,  smp_processor_id(), \
			##args);\
	        }\
	    } while(0);
#define dbg_msg(fmt, args...) \
	do {\
		if (mac_debug) {\
			printk("[%s@%d|%s]: cpu_%d: " fmt, \
					__FILE__, __LINE__, __FUNCTION__,  smp_processor_id(), ##args);\
		}\
	} while(0);

#define DUMP_PACKETS
#else
#undef dbg_msg
#define phnx_dbg_msg(fmt, args...)
#define dbg_msg(fmt, args...)
int mac_debug = 0;
#endif

#define MAC_B2B_IPG             88
/* frame sizes need to be cacheline aligned */
#define MAX_FRAME_SIZE          1536
#define MAX_FRAME_SIZE_JUMBO    9216

#define MAC_SKB_BACK_PTR_SIZE   SMP_CACHE_BYTES
#define MAC_PREPAD              0
#define BYTE_OFFSET             2
#define PHNX_RX_BUF_SIZE 	(MAX_FRAME_SIZE+BYTE_OFFSET+MAC_PREPAD+ \
				MAC_SKB_BACK_PTR_SIZE+SMP_CACHE_BYTES)
#define MAC_CRC_LEN             4
#define MAX_NUM_MSGRNG_STN_CC   128

#define MAX_NUM_DESC            1024
#define MAX_SPILL_SIZE          (MAX_NUM_DESC + 128)

#define MAC_FRIN_TO_BE_SENT_THRESHOLD 16

#define NEXT(index) ( ((index) + 1) == MAX_NUM_DESC ? 0 : (index)+1)

#ifdef MAC_SPLIT_MODE
#define MAX_FRIN_SPILL          (MAX_SPILL_SIZE << 1)
#define MAX_FROUT_SPILL         (MAX_SPILL_SIZE << 1)
#define MAX_CLASS_0_SPILL       (MAX_SPILL_SIZE << 1)
#define MAX_CLASS_1_SPILL       (MAX_SPILL_SIZE << 1)
#define MAX_CLASS_2_SPILL       (MAX_SPILL_SIZE << 1)
#define MAX_CLASS_3_SPILL       (MAX_SPILL_SIZE << 1)
#else
#define MAX_FRIN_SPILL          (MAX_SPILL_SIZE << 2)
#define MAX_FROUT_SPILL         (MAX_SPILL_SIZE << 2)
#define MAX_CLASS_0_SPILL       (MAX_SPILL_SIZE << 2)
#define MAX_CLASS_1_SPILL       (MAX_SPILL_SIZE << 2)
#define MAX_CLASS_2_SPILL       (MAX_SPILL_SIZE << 2)
#define MAX_CLASS_3_SPILL       (MAX_SPILL_SIZE << 2)
#endif
#define PHNX_NUM_REG_DUMP 9 /* Register 0xa0 to 0xa8 */
#define PHNX_ETHTOOL_REG_LEN (PHNX_NUM_REG_DUMP * 4) 

/*****************************************************************
 * Phoenix Generic Mac driver
 *****************************************************************/
void phnx_show_gmac_stats(void);
void phnx_monitor_speed(void);
void phnx_collect_stats(unsigned int gmac_id);
typedef enum { phnx_mac_speed_10, phnx_mac_speed_100, 
	     phnx_mac_speed_1000, phnx_mac_speed_rsvd } phnx_mac_speed_t;

typedef enum { phnx_mac_duplex_auto, phnx_mac_duplex_half,
	     phnx_mac_duplex_full } phnx_mac_duplex_t;

typedef enum { phnx_mac_fc_auto, phnx_mac_fc_disabled, phnx_mac_fc_frame,
	     phnx_mac_fc_collision, phnx_mac_fc_carrier } phnx_mac_fc_t;

#define MAC_TX_PENDING_FIFO_SIZE   512
#define MAC_TX_COMPLETE_FIFO_SIZE  16
struct fifo {
	struct sk_buff **skbs;
	int size;
	int head;
	int tail;
	int count;
};

static __inline__ void fifo_init(struct fifo *fifo, int max_size)
{
	void *ptr = kmalloc(max_size*sizeof(struct sk_buff *), GFP_KERNEL);

	if (!ptr) panic("[%s]: Unable to allocate memory for Tx Fifos\n", 
			__FUNCTION__);

	fifo->skbs = ptr;
	fifo->head = fifo->tail = fifo->count = 0;
	fifo->size = max_size;
}

static __inline__ int fifo_next_index(struct fifo *fifo, int index)
{
	return (index+1) % fifo->size;
}
static __inline__ int  fifo_next_head(struct fifo *fifo) 
{ return (fifo->head+1) % fifo->size ; }

static __inline__ int  fifo_next_tail(struct fifo *fifo) 
{ return (fifo->tail+1) % fifo->size ; }

static __inline__ int  fifo_empty(struct fifo *fifo) 
{ return fifo->head == fifo->tail ? 1 : 0; }

static __inline__ int  fifo_full(struct fifo *fifo) 
{ return fifo_next_tail(fifo) == fifo->head ? 1 : 0 ; }

static __inline__ int  fifo_count(struct fifo *fifo) 
{ 
	if (fifo->head <= fifo->tail)
		return fifo->tail - fifo->head;
	else
		return (fifo->size - fifo->head) + (fifo->tail - 1);
}
static __inline__ struct sk_buff *fifo_dequeue(struct fifo *fifo) 
{
	struct sk_buff *skb = 0;

	if (fifo_empty(fifo))
		return skb;
	skb = fifo->skbs[fifo->head];
	fifo->head = fifo_next_head(fifo);
	return skb;
}
static __inline__ int fifo_enqueue(struct fifo *fifo, struct sk_buff *data) 
{
	if (fifo_full(fifo))
		return 0;
	fifo->skbs[fifo->tail] = data;
	fifo->tail = fifo_next_tail(fifo);
	return 1;
}
static __inline__ struct sk_buff *fifo_head(struct fifo *fifo) 
{
	if (fifo_empty(fifo)) return 0;
	return fifo->skbs[fifo->head];
}

/* one tx queue per cpu (cpu_thread) */
#define MAC_TX_QUEUES 32
/* TODO: allocate these structures on different cachelines 
 * The spinlock is only for synch between tx and tx complete of that thread
 * The tx completes of a thread go that thread's bucket
 */
static spinlock_t                     tx_lock[MAC_TX_QUEUES];
static struct fifo                    tx_pending_q[MAC_TX_QUEUES];
static struct fifo                    tx_complete_q[MAC_TX_QUEUES];

#define mac_tx_complete_fifo(txq) (&tx_complete_q[txq])
#define mac_tx_pending_fifo(txq) (&tx_pending_q[txq])
#define mac_tx_lock(txq) (&tx_lock[txq])

static struct work_struct              mac_frin_replenish_work[8];


struct driver_data {
	
	/* Let these be the first fields in this structure 
	 * the structure is cacheline aligned when allocated in 
	 * init_etherdev
	 */
	struct fr_desc                 *frin_spill;
	struct fr_desc                 *frout_spill;
	union rx_tx_desc               *class_0_spill;
	union rx_tx_desc               *class_1_spill;
	union rx_tx_desc               *class_2_spill;
	union rx_tx_desc               *class_3_spill;

	struct net_device *dev;		  /* pointer to linux device */
	struct timer_list link_timer;   /* for monitoring MII */
	struct net_device_stats        stats;
	spinlock_t                     lock;

	phoenix_reg_t                  *mmio;

	__u8                           hwaddr[6];
	int	   phy_oldbmsr;
	int	   phy_oldanlpar;
	int	   phy_oldk1stsr;
	int	   phy_oldlinkstat;
	unsigned char                  phys_addr[2];

	phnx_mac_speed_t            speed;		/* current speed */
	phnx_mac_duplex_t           duplex;	  /* current duplex */
	int				advertising;
	phnx_mac_fc_t               flow_ctrl;/* current flow control setting */
	
	int                            id;
	int                            type;
	int                            instance;
	atomic_t                        frin_to_be_sent[8];
	int				open;
};

static int mac_frin_to_be_sent_thr[8];

/* TODO: use fetchadd for this */
#define mac_stats_add(x, val) ({(x) += (val);})

struct phnx_mac {
	int instance;
	int type;
	int irq;
	unsigned long phnx_io_offset;  
	int init_done;
};

static struct phnx_mac phnx_mac_devices[] = {
	{ .instance = 0, .type = TYPE_GMAC, 
	.phnx_io_offset = PHOENIX_IO_GMAC_0_OFFSET, .irq = PIC_GMAC_0_IRQ 
	},
	{ .instance = 1, .type = TYPE_GMAC, 
	.phnx_io_offset = PHOENIX_IO_GMAC_1_OFFSET, .irq = PIC_GMAC_1_IRQ
	},
	{ .instance = 2, .type = TYPE_GMAC, 
	.phnx_io_offset = PHOENIX_IO_GMAC_2_OFFSET, .irq = PIC_GMAC_2_IRQ
	},
	{ .instance = 3, .type = TYPE_GMAC, 
	.phnx_io_offset = PHOENIX_IO_GMAC_3_OFFSET, .irq = PIC_GMAC_3_IRQ
	},
#ifdef CONFIGURE_XGMAC
#error "xgmac is currently not supported"
	{ .instance = 0, .type = TYPE_XGMAC, 
	.phnx_io_offset = PHOENIX_IO_XGMAC_0_OFFSET, .irq = PIC_XGS_0_IRQ
	},
	{ .instance = 1, .type = TYPE_XGMAC, 
	.phnx_io_offset = PHOENIX_IO_XGMAC_1_OFFSET, .irq = PIC_XGS_1_IRQ
	}
#endif
};

#define PHOENIX_MAX_MACS        \
	(int)(sizeof(phnx_mac_devices)/sizeof(struct phnx_mac))

phnx_stats  phnx_gmac_stats[PHOENIX_MAX_MACS];

#define phnx_mac_id(priv) ( ((priv)->type << 4) | (priv)->instance )

static struct net_device *dev_mac[PHOENIX_MAX_MACS];

static void rmi_phnx_mac_set_enable(struct driver_data *priv, int flag);
static void rmi_phnx_xgmac_init(struct driver_data *priv);
static void rmi_phnx_gmac_init(struct driver_data *priv);
static void phnx_mac_set_rx_mode(struct net_device *dev);
static void rmi_phnx_mac_msgring_handler(int bucket, int size, int code, 
				int stid, struct msgrng_msg *msg, void *data);
static irqreturn_t rmi_phnx_mac_int_handler(int irq, 
	            void *dev_id, struct pt_regs *regs);
static int rmi_phnx_mac_open(struct net_device *dev);
static int rmi_phnx_mac_xmit(struct sk_buff *skb, struct net_device *dev);
static int rmi_phnx_mac_close(struct net_device *dev);
static void rmi_phnx_mac_timer(unsigned long data);
static struct net_device_stats *rmi_phnx_mac_get_stats(struct net_device *dev);
static void rmi_phnx_mac_set_multicast_list(struct net_device *dev);
static int rmi_phnx_mac_do_ioctl (struct net_device *dev, 
	            struct ifreq *rq, int cmd);
static void rmi_phnx_mac_tx_timeout(struct net_device *dev);
static int rmi_phnx_mac_change_mtu(struct net_device *dev, int new_mtu);
static int rmi_phnx_mac_fill_rxfr(struct net_device *dev);
static void rmi_phnx_config_spill_area(struct driver_data *priv);

/*****************************************************************
 * Driver Helper Functions
 *****************************************************************/
static __inline__ struct sk_buff *mac_get_skb_back_ptr(unsigned long addr)
{
	unsigned long *back_ptr = 
		(unsigned long *)(addr - MAC_SKB_BACK_PTR_SIZE);
	phnx_dbg_msg("addr = %lx,  skb = %lx\n", addr, *back_ptr);

	/* this function should be used only for newly allocated packets. 
	 * It assumes the first cacheline is for the back pointer related 
	 * book keeping info
	 */
	return (struct sk_buff *)(*back_ptr);
}

static __inline__ void mac_put_skb_back_ptr(struct sk_buff *skb)
{
	unsigned long *back_ptr = (unsigned long *)skb->data;

	/* this function should be used only for newly allocated packets. 
	 * It assumes the first cacheline is for the back pointer related 
	 * book keeping info
	 */
	skb_reserve(skb, MAC_SKB_BACK_PTR_SIZE);

	*back_ptr = (unsigned long)skb;
	phnx_dbg_msg("p=%p, skb=%p\n", back_ptr, skb);
}

#define CACHELINE_ALIGNED_ADDR(addr) \
			(((unsigned long)(addr)) & ~(SMP_CACHE_BYTES-1))

static __inline__ void *cacheline_aligned_kmalloc(int size, int gfp_mask)
{
	void *buf = kmalloc(size+SMP_CACHE_BYTES, gfp_mask);
	if (buf) buf = (void *)(CACHELINE_ALIGNED_ADDR((unsigned long)buf + 
				                   SMP_CACHE_BYTES));
	return buf;
}

static __inline__ struct sk_buff *rmi_phnx_alloc_skb(void)
{
	int offset=0;
	struct sk_buff *skb = __dev_alloc_skb(PHNX_RX_BUF_SIZE, GFP_KERNEL);

	if (!skb) {
		return NULL;
	}

	/* align the data to the next cache line */
	offset = ((unsigned long)skb->data + SMP_CACHE_BYTES) & 
	                                        ~(SMP_CACHE_BYTES - 1);
	skb_reserve(skb, (offset - (unsigned long)skb->data));

	return skb;
}

/**********************************************************************
 **********************************************************************/
static void rmi_phnx_mac_set_enable(struct driver_data *priv, int flag)
{
	uint32_t regval;

	if (flag) {
		regval = phoenix_read_reg(priv->mmio, R_TX_CONTROL);
		regval |= (1<<O_TX_CONTROL__TxEnable) | 
	                            (512<<O_TX_CONTROL__TxThreshold);
	
		phoenix_write_reg(priv->mmio, R_TX_CONTROL, regval);

		regval = phoenix_read_reg(priv->mmio, R_RX_CONTROL);
		regval |= 1<<O_RX_CONTROL__RxEnable;
		phoenix_write_reg(priv->mmio, R_RX_CONTROL, regval);

		regval = phoenix_read_reg(priv->mmio, R_MAC_CONFIG_1);
		regval |= (O_MAC_CONFIG_1__txen | O_MAC_CONFIG_1__rxen);
		phoenix_write_reg(priv->mmio, R_MAC_CONFIG_1, regval);
	}
	else {
		regval = phoenix_read_reg(priv->mmio, R_TX_CONTROL);
		regval &= ~((1<<O_TX_CONTROL__TxEnable) | 
				(512<<O_TX_CONTROL__TxThreshold));
	
		phoenix_write_reg(priv->mmio, R_TX_CONTROL, regval);

		regval = phoenix_read_reg(priv->mmio, R_RX_CONTROL);
		regval &= ~(1<<O_RX_CONTROL__RxEnable);
		phoenix_write_reg(priv->mmio, R_RX_CONTROL, regval);
		regval = phoenix_read_reg(priv->mmio, R_MAC_CONFIG_1);
		regval &= ~(O_MAC_CONFIG_1__txen | O_MAC_CONFIG_1__rxen);
		phoenix_write_reg(priv->mmio, R_MAC_CONFIG_1, regval);
	}
}

/**********************************************************************
 **********************************************************************/
static __inline__ int phnx_mac_send_fr(struct driver_data *priv, 
				     unsigned long addr, int len)
{
	int stid = 0;
	struct msgrng_msg msg;
	int i, retry = 10;

	stid = mac_make_desc_rfr(&msg, priv->instance, priv->type, addr);

	/* Send the packet to MAC */
	phnx_dbg_msg("mac_%d: Sending free packet to stid %d\n", 
			priv->instance, stid);
	for(i=0; i < retry; i++) {
		if(message_send_retry(1, MSGRNG_CODE_MAC, stid, &msg) == 0)
			break;
	}
	
	return 0;  
}

/**********************************************************************
 *  Init MII interface
 *  
 *  Input parameters: 
 *  	 s - priv structure
 ********************************************************************* */
#define PHY_STATUS_RETRIES 25000

static void rmi_phnx_mac_mii_init(void)
{
	phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_GMAC_0_OFFSET);

	/* use the lowest clock divisor - divisor 28 */
	phoenix_write_reg(mmio, R_MII_MGMT_CONFIG, 0x07);
}


/**********************************************************************
 *  Read a PHY register.
 *  
 *  Input parameters: 
 *  	 s - priv structure
 *  	 phyaddr - PHY's address
 *  	 regidx = index of register to read
 *  	 
 *  Return value:
 *  	 value read, or 0 if an error occurred.
 ********************************************************************* */
static unsigned int rmi_phnx_mac_mii_read(int phyaddr, int regidx)
{
	phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_GMAC_0_OFFSET);
	int i=0;
	
	/* setup the phy reg to be used */
	phoenix_write_reg(mmio, R_MII_MGMT_ADDRESS, 
		  (phyaddr<<8) | (regidx<<0));
	
	/* Issue the read command */
	phoenix_write_reg(mmio, R_MII_MGMT_COMMAND, 
		  (1<<O_MII_MGMT_COMMAND__rstat));

	/* poll for the read cycle to complete */
	for(i=0;i<PHY_STATUS_RETRIES;i++) {
		if (phoenix_read_reg(mmio, R_MII_MGMT_INDICATORS)==0) break;
	}
	
	/* clear the read cycle */
	phoenix_write_reg(mmio, R_MII_MGMT_COMMAND, 0);

	if (i == PHY_STATUS_RETRIES) { return 0xffffffff; }

	/* Read the data back */
	return phoenix_read_reg(mmio, R_MII_MGMT_STATUS);
}

/**********************************************************************
 *  Write a value to a PHY register.
 *  
 *  Input parameters: 
 *  	 s - priv structure
 *  	 phyaddr - PHY to use
 *  	 regidx - register within the PHY
 *  	 regval - data to write to register
 *  	 
 *  Return value:
 *  	 nothing
 ********************************************************************* */
void rmi_phnx_mac_mii_write(int phyaddr, int regidx, unsigned int regval)  
{
	phoenix_reg_t *mmio = phoenix_io_mmio(PHOENIX_IO_GMAC_0_OFFSET);
	int i=0;

	phoenix_write_reg(mmio, R_MII_MGMT_ADDRESS, 
		  (phyaddr<<8) | (regidx<<0) );
	
	/* Write the data which starts the write cycle */
	phoenix_write_reg(mmio, R_MII_MGMT_WRITE_DATA, regval);

	/* poll for the write cycle to complete */
	for(i=0;i<PHY_STATUS_RETRIES;i++) {
		if (phoenix_read_reg(mmio, R_MII_MGMT_INDICATORS)==0) break;
	}  

	return;
}

/*****************************************************************
 * Initialize XGMAC
 *****************************************************************/
static void rmi_phnx_xgmac_init(struct driver_data *priv)
{
	int i=0;
	phoenix_reg_t *mmio = priv->mmio;
	int id = priv->instance;

	phnx_dbg_msg("Initializing phnx_mac type=%d, instance=%d:\n", 
	                                      priv->type, priv->instance);

	rmi_phnx_config_spill_area(priv);

	phoenix_write_reg(priv->mmio, R_DESC_PACK_CTRL, 
	              (BYTE_OFFSET<<O_DESC_PACK_CTRL__ByteOffset) |
	              (1<<O_DESC_PACK_CTRL__MaxEntry) |
	              (MAX_FRAME_SIZE_JUMBO<<O_DESC_PACK_CTRL__RegularSize) );

	/* To all CPUs */
	phoenix_write_reg(priv->mmio, R_PDE_CLASS_0, 0xff);

	/* configure the XGMAC Registers */
	phoenix_write_reg(mmio, R_XGMAC_CONFIG_1, 0x50000020);
	phoenix_write_reg(mmio, R_XGMAC_MAX_FRAME_LEN, 0x0A000A00);
	
	/* configure the XGMAC_GLUE Registers */
	phoenix_write_reg(mmio, R_DMACR0, 
		(7<<O_DMACR0__Data0WrMaxCr)|(7<<O_DMACR0__Data0RdMaxCr));
	phoenix_write_reg(mmio, R_DMACR3, 
	                (1<<O_DMACR3__SpClassWrMaxCr)|
	                (1<<O_DMACR3__SpClassRdMaxCr)|
	                (1<<O_DMACR3__RegFrInWrMaxCr)|
	                (1<<O_DMACR3__RegFrInRdMaxCr)|
	                (1<<O_DMACR3__FrOutWrMaxCr)|
	                (1<<O_DMACR3__FrOutRdMaxCr));

	if (id == 0) {
		for(i=0; i<16; i++) {
			phoenix_write_reg(mmio, R_XGS_TX0_BUCKET_SIZE+i, 
	                        bucket_sizes.bucket[MSGRNG_STNID_XGS0_TX+i]);
		}
		phoenix_write_reg(mmio, R_XGS_JFR_BUCKET_SIZE, 
	                        bucket_sizes.bucket[MSGRNG_STNID_XMAC0JFR]);
		phoenix_write_reg(mmio, R_XGS_RFR_BUCKET_SIZE, 
	                        bucket_sizes.bucket[MSGRNG_STNID_XMAC0RFR]);

		for(i=0; i<MAX_NUM_MSGRNG_STN_CC; i++) {
			phoenix_write_reg(mmio, R_CC_CPU0_0 + i, 
	                        cc_table_xgs_0.counters[i>>3][i&0x07]);
		}
	}
	else if (id == 1) {
		for(i=0; i<16; i++) {
			phoenix_write_reg(mmio, R_XGS_TX0_BUCKET_SIZE+i, 
	                        bucket_sizes.bucket[MSGRNG_STNID_XGS1_TX+i]);
		}

		phoenix_write_reg(mmio, R_XGS_JFR_BUCKET_SIZE, 
	                    bucket_sizes.bucket[MSGRNG_STNID_XMAC1JFR]);
		phoenix_write_reg(mmio, R_XGS_RFR_BUCKET_SIZE, 
	                    bucket_sizes.bucket[MSGRNG_STNID_XMAC1RFR]);
	
		for(i=0; i<MAX_NUM_MSGRNG_STN_CC; i++) {
			phoenix_write_reg(mmio, R_CC_CPU0_0 + i, 
	                        cc_table_xgs_1.counters[i>>3][i&0x07]);
		}
	}
}

/*****************************************************************
 * Initialize GMAC
 *****************************************************************/

static void rmi_phnx_populate_frin_to_be_sent(void)
{
	unsigned int phys_cpu_map=0;
	int cpu=0;
	unsigned int i;
	for(i=0;i<32;i++) {
		if (cpu_isset(i, cpu_online_map)) {
			cpu = cpu_logical_map(i);
			phys_cpu_map |= (1<<cpu);
		}
	}
	for(i=0;i<8;i++) {
		if (phys_cpu_map & (1<<(i<<2))) {
			for (cpu=3;cpu>=0;cpu--) {
				if (phys_cpu_map & (1<<((i<<2)+cpu)))
					break;
			}
			if (cpu>=0) mac_frin_to_be_sent_thr[i] = cpu;
			else BUG();
		}
	}
}


static void rmi_phnx_config_pde(struct driver_data *priv)
{
	 int i = 0, cpu = 0, bucket = 0;
  __u64 bucket_map = 0;
  __u32 desc_pack_ctrl = 0;

	if (xlr_hybrid_user_mac()) {
		extern __u32 rmios_user_mac_thr_mask;

		bucket_map = 0;
		for (i = 0; i < 32; i++) {
			if (rmios_user_mac_thr_mask & (1 << i)) {
				cpu = i;
				bucket =
					((cpu >> 2) << 3) + RMIOS_USER_MAC_BUCKET;
				bucket_map |= (1ULL << bucket);
				dbg_msg
					("i=%d, cpu=%d, bucket = %d, bucket_map=%llx\n",
					 i, cpu, bucket, bucket_map);
			}
		}
		printk("rmios pde bucket_map = %llx\n", bucket_map);

		phoenix_write_reg(priv->mmio, R_PDE_CLASS_0, (bucket_map & 0xffffffff));
		phoenix_write_reg(priv->mmio, R_PDE_CLASS_0 + 1,
				((bucket_map >> 32) & 0xffffffff));
		phoenix_write_reg(priv->mmio, R_PDE_CLASS_1, (bucket_map & 0xffffffff));
		phoenix_write_reg(priv->mmio, R_PDE_CLASS_1 + 1,
				((bucket_map >> 32) & 0xffffffff));

		phoenix_write_reg(priv->mmio, R_PDE_CLASS_2, (bucket_map & 0xffffffff));
		phoenix_write_reg(priv->mmio, R_PDE_CLASS_2 + 1,
				((bucket_map >> 32) & 0xffffffff));

		phoenix_write_reg(priv->mmio, R_PDE_CLASS_3, (bucket_map & 0xffffffff));
		phoenix_write_reg(priv->mmio, R_PDE_CLASS_3 + 1,
				((bucket_map >> 32) & 0xffffffff));

		desc_pack_ctrl = phoenix_read_reg(priv->mmio, R_DESC_PACK_CTRL);
		printk("[%s]: gmac_%d: configuring prepad \n", __FUNCTION__,
				priv->instance);
		phoenix_write_reg(priv->mmio, R_DESC_PACK_CTRL,
				desc_pack_ctrl | (1 << 16));
	}
	else{
	int i=0, cpu=0, bucket=0;
	__u64 bucket_map = 0;

	for (i=0;i<32;i++) {
		if (cpu_isset(i, cpu_online_map)) {
			cpu = cpu_logical_map(i);
			bucket = ((cpu >> 2)<<3)|(cpu & 0x03);
			bucket_map |= (1ULL << bucket);
		}
	}      
	printk("pde bucket_map = %llx\n", bucket_map);
	phoenix_write_reg(priv->mmio, R_PDE_CLASS_0, (bucket_map & 0xffffffff));
	phoenix_write_reg(priv->mmio, R_PDE_CLASS_0+1, 
					((bucket_map>>32) & 0xffffffff));
	phoenix_write_reg(priv->mmio, R_PDE_CLASS_1, (bucket_map & 0xffffffff));
	phoenix_write_reg(priv->mmio, R_PDE_CLASS_1+1, 
			((bucket_map>>32) & 0xffffffff));
#if !defined(MAC_SPLIT_MODE)

	phoenix_write_reg(priv->mmio, R_PDE_CLASS_2, (bucket_map & 0xffffffff));
	phoenix_write_reg(priv->mmio, R_PDE_CLASS_2+1, 
			((bucket_map>>32) & 0xffffffff));
	phoenix_write_reg(priv->mmio, R_PDE_CLASS_3, (bucket_map & 0xffffffff));
	phoenix_write_reg(priv->mmio, R_PDE_CLASS_3+1, 
			((bucket_map>>32) & 0xffffffff));
#endif
	}

}

static void rmi_phnx_config_parser(struct driver_data *priv)
{
				if (!xlr_hybrid_user_mac()) return;

				/* configure the parser : L2 Type is configured in the bootloader */
				/* extract IP: src, dest protocol */
				phoenix_write_reg(priv->mmio, R_L3CTABLE,
												(9 << 20) | (1 << 19) | (1 << 18) | (0x01 << 16) |
												(0x0800 << 0));
				phoenix_write_reg(priv->mmio, R_L3CTABLE + 1,
												(12 << 25) | (4 << 21) | (16 << 14) | (4 << 10));

				if (xlr_user_mac_l4_extract()) {
								/* extract TCP: src port, dest port */
								phoenix_write_reg(priv->mmio, R_L4CTABLE, (6 << 0));
								phoenix_write_reg(priv->mmio, R_L4CTABLE + 1,
																(0 << 21) | (2 << 17) | (2 << 11) | (2 << 7));
								/* extract UDP: src port, dest port */
								phoenix_write_reg(priv->mmio, R_L4CTABLE + 2, (17 << 0));
								phoenix_write_reg(priv->mmio, R_L4CTABLE + 3,
																(0 << 21) | (2 << 17) | (2 << 11) | (2 << 7));
				}
}


static void rmi_phnx_config_classifier(struct driver_data *priv)
{
				int tt_index = 0;
				int tt_table[128];
				extern __u32 rmios_user_mac_thr_mask;
				int i = 0, num_cores = 0;

				if (!xlr_hybrid_user_mac()) return;

				for (i = 0; i < 32; i++)
								if (rmios_user_mac_thr_mask & (1 << i))
												num_cores++;

				if (!num_cores)
								num_cores = 1;

				for (i = 0; i < 128; i++) {
								tt_table[i] =
												(((i % num_cores) << 3) + RMIOS_USER_MAC_BUCKET) & 0x3f;
				}

				for (tt_index = 0; tt_index < 64; tt_index++) {
								int b0 = 0;
								int b1 = 0;
								int class = tt_index & 0x03;
								b0 = tt_table[tt_index];
								b1 = tt_table[64 + tt_index];
								dbg_msg("[%s]: translation_table b0=%d, b1=%d, class=%d \n",
																__FUNCTION__, b0, b1, class);
								phoenix_write_reg(priv->mmio, R_TRANSLATETABLE + tt_index,
																((class << 23) | ((b1) << 17) | (1 << 16))
																/*entry1 */ |((class << 7) | (b0 << 1) |
																				(1 << 0)) /*entry0 */ );
				}

				/* Use 7-bit CRC Hash to lookup the translation table */
				phoenix_write_reg(priv->mmio, R_PARSERCONFIGREG,
												(0x3ff << 15) | (0x7f << 8) | (0x01 << 1));
}





static void rmi_phnx_gmac_init(struct driver_data *priv)
{
	int i=0;
	phoenix_reg_t *mmio = priv->mmio;
	int id = priv->instance;
	__u32 value=0;

	printk("Initializing gmac %d:\n", id);

	rmi_phnx_config_spill_area(priv);

	phoenix_write_reg(priv->mmio, R_DESC_PACK_CTRL, 
			(BYTE_OFFSET<<O_DESC_PACK_CTRL__ByteOffset) |
			(1<<O_DESC_PACK_CTRL__MaxEntry) |
			(MAX_FRAME_SIZE<<O_DESC_PACK_CTRL__RegularSize) );

	rmi_phnx_config_pde(priv);	
	rmi_phnx_config_parser(priv);
	rmi_phnx_config_classifier(priv);	
	rmi_phnx_populate_frin_to_be_sent();

	phoenix_write_reg(priv->mmio, R_MSG_TX_THRESHOLD, 3);

#ifdef MAC_DISABLE_GMAC0
	/* TEMP Convenience HACK for gmac0 */
	if (priv->instance == 0) {
		printk("[%s]: Disabling gmac0\n", __FUNCTION__);
		phoenix_write_reg(mmio, R_MAC_CONFIG_1, 0x0);
	}
	else
		phoenix_write_reg(mmio, R_MAC_CONFIG_1, 0x35);
#else
	phoenix_write_reg(mmio, R_MAC_CONFIG_1, 0x35);
#endif

	rmi_phnx_mac_mii_init();
	
	priv->advertising = ADVERTISED_10baseT_Full | ADVERTISED_10baseT_Half | 
			ADVERTISED_100baseT_Full | ADVERTISED_100baseT_Half |
			ADVERTISED_1000baseT_Full | ADVERTISED_Autoneg |
			ADVERTISED_MII;

	priv->speed = rmi_phnx_mac_mii_read(id, 28);
	printk("gmac_%d, aux control/status reg = %x\n", id, priv->speed);
	priv->speed = (priv->speed >> 3) & 0x03;
	priv->phy_oldlinkstat = 
		rmi_phnx_mac_mii_read(id, MII_BMSR) & BMSR_LSTATUS;

	if (priv->speed == phnx_mac_speed_10) {
		phoenix_write_reg(mmio, R_MAC_CONFIG_2, 0x7137);
		phoenix_write_reg(mmio, R_CORECONTROL, 0x02);
		printk(KERN_INFO "configuring gmac_%d in 10Mbps mode\n", id);
	}
	else if (priv->speed == phnx_mac_speed_100) {
		phoenix_write_reg(mmio, R_MAC_CONFIG_2, 0x7137);
		phoenix_write_reg(mmio, R_CORECONTROL, 0x01);      
		printk(KERN_INFO "configuring gmac_%d in 100Mbps mode\n", id);
	}
	else {
		if (priv->speed != phnx_mac_speed_1000) {
			printk("phy reported unknown mac speed, defaulting "
					"to 100Mbps\n");
			phoenix_write_reg(mmio, R_MAC_CONFIG_2, 0x7137);
			phoenix_write_reg(mmio, R_CORECONTROL, 0x01);      
		}
		phoenix_write_reg(mmio, R_MAC_CONFIG_2, 0x7237);
		phoenix_write_reg(mmio, R_CORECONTROL, 0x00);      
		printk(KERN_INFO "configuring gmac_%d in 1000Mbps mode\n", id);
	}
	value = phoenix_read_reg(mmio, R_IPG_IFG);
	phoenix_write_reg(mmio, R_IPG_IFG, ((value & ~0x7f)|MAC_B2B_IPG));
	phoenix_write_reg(mmio, R_DMACR0, 0xffffffff);
	phoenix_write_reg(mmio, R_DMACR1, 0xffffffff);
	phoenix_write_reg(mmio, R_DMACR2, 0xffffffff);
	phoenix_write_reg(mmio, R_DMACR3, 0xffffffff);
	phoenix_write_reg(mmio, R_STATCTRL, 0x04);
	phoenix_write_reg(mmio, R_L2ALLOCCTRL, 2);
#ifdef MAC_SPLIT_MODE
	phoenix_write_reg(mmio, R_FREEQCARVE, 0x60);
#else
	phoenix_write_reg(mmio, R_FREEQCARVE, 0);
#endif


	phoenix_write_reg(mmio, R_GMAC_TX0_BUCKET_SIZE+id, 
	                bucket_sizes.bucket[MSGRNG_STNID_GMACTX0+id]);
	phoenix_write_reg(mmio, R_GMAC_JFR0_BUCKET_SIZE, 
			bucket_sizes.bucket[MSGRNG_STNID_GMACJFR_0]);
	phoenix_write_reg(mmio, R_GMAC_RFR0_BUCKET_SIZE, 
			bucket_sizes.bucket[MSGRNG_STNID_GMACRFR_0]);
	phoenix_write_reg(mmio, R_GMAC_JFR1_BUCKET_SIZE, 
	                bucket_sizes.bucket[MSGRNG_STNID_GMACJFR_1]);
	phoenix_write_reg(mmio, R_GMAC_RFR1_BUCKET_SIZE, 
				bucket_sizes.bucket[MSGRNG_STNID_GMACRFR_1]);
	
	for(i=0; i<MAX_NUM_MSGRNG_STN_CC; i++) {
		phoenix_write_reg(mmio, R_CC_CPU0_0 + i, 
	                    cc_table_gmac.counters[i>>3][i&0x07]);
	}
}

/**********************************************************************
 * Set promiscuous mode
 **********************************************************************/
static void phnx_mac_set_rx_mode(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);
	uint32_t regval;

	
	regval = phoenix_read_reg(priv->mmio, R_MAC_FILTER_CONFIG);

	if (dev->flags & IFF_PROMISC) {
		regval |= (1<<O_MAC_FILTER_CONFIG__BROADCAST_EN)|
	              (1<<O_MAC_FILTER_CONFIG__PAUSE_FRAME_EN)|
	              (1<<O_MAC_FILTER_CONFIG__ALL_MCAST_EN)|
	              (1<<O_MAC_FILTER_CONFIG__ALL_UCAST_EN);
	}
	else {
		regval &= ~((1<<O_MAC_FILTER_CONFIG__PAUSE_FRAME_EN)|
			(1<<O_MAC_FILTER_CONFIG__ALL_UCAST_EN));
	}

	phoenix_write_reg(priv->mmio, R_MAC_FILTER_CONFIG, regval);
}


/*****************************************************************
 * Kernel Net Stack <-> MAC Driver Interface
 *****************************************************************/
/**********************************************************************
 **********************************************************************/
#define MAC_TX_FAIL 1
#define MAC_TX_PASS 0
static int mac_xmit(struct sk_buff *skb, struct net_device *dev, 
			     struct driver_data *priv, int txq)
{
	struct msgrng_msg msg;
	int stid=0;
	__u32 tx_cycles = 0;
	struct fifo *txc = mac_tx_complete_fifo(txq);
	int frid;

	phnx_dbg_msg("tx_complete_fifo count=%d, status=%x\n", 
			fifo_count(txc), read_c0_status());
	/* check if there is room in the complete q */
	if (fifo_full(txc)) return MAC_TX_FAIL;

	tx_cycles = read_c0_count();
	frid = ((txq>>2)<<3)|(txq&0x03);
	stid = mac_make_desc_tx(&msg, priv->instance, priv->type,
			virt_to_phys(skb->data), skb->len, frid);  

	if(xlr_revision_a0()) {
		if (message_send_retry(2, MSGRNG_CODE_MAC, stid, &msg)) 
			return MAC_TX_FAIL;
	} else {
		if (message_send_retry(1, MSGRNG_CODE_MAC, stid, &msg)) 
			return MAC_TX_FAIL;
	}

	/* Send the packet to MAC */
	phnx_dbg_msg("Sent tx packet to stid %d\n", stid);
#ifdef DUMP_PACKETS
	{
	phnx_dbg_msg("Tx Packet: length=%d\n", skb->len);
	int i=0;
	for(i=0;i<64;i++) {
		printk("%02x ", skb->data[i]);
		if (i && (i % 16) == 0) printk("\n");
	}
	printk("\n");
	}
#endif
	
	if (!fifo_enqueue(txc, skb)) {
		panic("BUG?: [%s]: failed to enqueue in tx_complete_fifo?\n", 
				__FUNCTION__);
	}
	phnx_dbg_msg("enqueued %p in tx_complete_fifo\n", skb);
	phnx_inc_counter(NETIF_TX);
	
	dev->trans_start = jiffies;
	
	return MAC_TX_PASS;
}

static int rmi_phnx_mac_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);
	int ret = -ENOSPC;
	unsigned long flags=0, mflags=0;
	int txq = hard_smp_processor_id();
	spinlock_t *txl = mac_tx_lock(txq);

	phnx_dbg_msg("IN\n"); 
	phnx_inc_counter(NETIF_STACK_TX);
	msgrng_access_save(txl, flags, mflags);

#ifdef MAC_TX_PENDING_QUEUES
	{
	struct fifo *txp = mac_tx_pending_fifo(txq);

	/* check if there are any pending transmits */
	if (fifo_empty(txp)) {
		/* no pending transmits, try sending the packet */
		ret = mac_xmit(skb, dev, priv, txq);
		if (ret == MAC_TX_PASS) goto out;	
	}

	/* either there are pending transmits or current trasmit failed,
	* Q the packet and let transmit complete send the packet 
	*/
	if (!fifo_enqueue(txp, skb)) {
		ret = MAC_TX_FAIL;
	}
	else {
		ret = MAC_TX_PASS;
	}
	}
 out:
#else 
	ret = mac_xmit(skb, dev, priv, txq);
#endif
	msgrng_access_restore(txl, flags, mflags);

	phnx_dbg_msg("OUT, ret = %d, flags=%lx, mflags=%lx\n", ret, flags, 
					mflags); 
	if (ret == MAC_TX_FAIL) {
#ifdef MAC_NETIF_BYPASS
		/* drop the packet */
		dev_kfree_skb_irq(skb);      
		phnx_inc_counter(NETIF_TX_DROP);
#else
		/* FULL */
		phnx_dbg_msg("Msg Ring Full. Stopping upper layer Q\n");
		spin_lock_irq(&priv->lock);
		msgrng_access_save(txl, flags, mflags);
		if((ret = mac_xmit(skb, dev, priv, txq)) != MAC_TX_PASS) {
			netif_stop_queue(dev);
			phnx_inc_counter(NETIF_STOP_Q);
		}
		msgrng_access_restore(txl, flags, mflags);
		spin_unlock_irq(&priv->lock);

#endif
	}

	return ret;
}

static void mac_frin_replenish(void *args/* ignored */)
{
	int cpu = phoenix_cpu_id();
	int done = 0;
	int i=0;

	phnx_inc_counter(REPLENISH_ENTER);
	phnx_set_counter(REPLENISH_CPU, hard_smp_processor_id());

	for(;;) {

		done = 0;

		for(i=0;i<4;i++) {
		int offset=0;
		unsigned long msgrng_flags;
		struct sk_buff *skb = 0;
		__u32 cycles;
		struct net_device *dev;
		struct driver_data *priv;
		atomic_t *frin_to_be_sent;

		dev = dev_mac[i];
		if (!dev) goto skip;

		priv = netdev_priv(dev);
		frin_to_be_sent = &priv->frin_to_be_sent[cpu];

		if (atomic_read(frin_to_be_sent) < 0) {
			panic("BUG?: [%s]: gmac_%d illegal value for "
			"frin_to_be_sent=%d\n", __FUNCTION__, i, 
			atomic_read(frin_to_be_sent));
		}

		if (!atomic_read(frin_to_be_sent)) goto skip;
	  
		cycles = read_c0_count();
		skb = __dev_alloc_skb(PHNX_RX_BUF_SIZE, 
				GFP_ATOMIC|__GFP_REPEAT|__GFP_NOWARN);
		if (!skb) {
			skb = __dev_alloc_skb(PHNX_RX_BUF_SIZE, GFP_KERNEL);
			if (!skb) 
				panic("[%s]: Unable to allocate skb!\n", 
						__FUNCTION__);
		}
		phnx_inc_counter(REPLENISH_FRIN);

		/* align the data to the next cache line */
		offset = (((unsigned long)skb->data + SMP_CACHE_BYTES) & 
				~(SMP_CACHE_BYTES - 1));
		skb_reserve(skb, (offset - (unsigned long)skb->data));


		msgrng_access_enable(msgrng_flags);
		mac_put_skb_back_ptr(skb);
		if (phnx_mac_send_fr(priv, virt_to_bus(skb->data), skb->len)) {
			dev_kfree_skb(skb);
			phnx_dbg_msg("[%s]: rx free message_send failed!\n", 
					__FUNCTION__);
			msgrng_access_disable(msgrng_flags);
			break;
		}
		msgrng_access_disable(msgrng_flags);
	
		phnx_set_counter(REPLENISH_CYCLES, (read_c0_count()-cycles));
	
		atomic_dec(frin_to_be_sent);
	
		continue;
		skip:
		done++;
		}
	if (done == 4) break;
	}
}

#ifdef MAC_TX_PENDING_QUEUES
static int mac_tx_complete_tx(int txq)
{
	struct sk_buff *skb = 0;
	int ret = 0;
	int i=0;
	struct fifo *txp = mac_tx_pending_fifo(txq);

	phnx_dbg_msg("IN, tx_pending_fifo count=%d\n", fifo_count(txp));
	for(i=0;i<1;i++) {
		/* check to see if there are any more pending packets to 
		 * be sent */
		if (fifo_empty(txp)) break;
	
		phnx_inc_counter(NETIF_TX_COMPLETE_TX);
		skb = fifo_head(txp);
		ret = mac_xmit(skb, skb->dev, netdev_priv(skb->dev), txq);
		if (ret == MAC_TX_PASS) {
			if (!fifo_dequeue(txp)) {
				panic("BUG?: [%s]: failed to dequeue from "
				"tx_pending_fifo?\n", __FUNCTION__);
			}
		phnx_dbg_msg("dequeued %p from tx_pending_fifo\n", skb);
		}
		else {
			/* stop processing */
			break;
		}
	}
	phnx_dbg_msg("OUT\n");

	return fifo_count(txp);
}
#endif

static phnx_atomic_t mac_stop;

#ifdef MAC_TX_PENDING_QUEUES
#define mac_netif_tx_resume(pending) ((pending)<(MAC_TX_PENDING_FIFO_SIZE>>2))
#else
#define mac_netif_tx_resume(pending) (1)
#endif

static void mac_tx_complete(int txq, unsigned long addr)
{
	struct driver_data *priv = 0;
	struct sk_buff *skb = 0;
	spinlock_t *txl = mac_tx_lock(txq);
	struct fifo *txc = mac_tx_complete_fifo(txq);
	int found = 0, head=0;
	// below 2 variables are only for debugging
	int tx_complete_reorder_count=0;
	int txc_count = 0;
	int pending = 0;
	struct net_device *dev;

	spin_lock(txl);
	
	txc_count = fifo_count(txc);
	for(head=txc->head; head!=txc->tail; head=fifo_next_index(txc, head)) {
		skb = txc->skbs[head];
		/* holes are possible if some other thread beats this 
		 * thread in processing tx completes for this thread 
		*/
		if (!skb) {
			tx_complete_reorder_count++;
			/* paranoia check */
			if (head == txc->head) {
				panic("BUG?: [%s]: fifo head with null skb?\n",
					     	__FUNCTION__);
			}
			continue;
		}
		if (CACHELINE_ALIGNED_ADDR(skb->data) == 
				CACHELINE_ALIGNED_ADDR(addr)) {
			found = 1;
			/* dequeue this entry */
			txc->skbs[head] = 0;
			/* advance fifo_head over all null skbs 
			 * (null -> dequeued) if needed */
			if (head == txc->head) {
				while (txc->head != txc->tail && 
						txc->skbs[txc->head]==NULL)
					txc->head = fifo_next_head(txc);
			}
			phnx_dbg_msg("tx complete for addr=%lx\n", addr);
	
			/* done */
			break;

		}
	}
	if (!found) {
		int i=0, j=0;
		int cpu=(phoenix_cpu_id() << 2);
		unsigned long flags;

		if (!phnx_test_and_set(&mac_stop)) {
			for(;;) {
				__asm__ __volatile__(".set mips64\nwait\n");
			}
		}
	
		printk("[%s]: unexpected tx complete for addr=%lx, txq=%d"
		 "fifo_count=%d, tx_complete_reorder_count=%d\n",
		__FUNCTION__, addr, txq, txc_count, tx_complete_reorder_count);
		for(j=cpu;j<(cpu+4);j++) {
			if (j != txq) {
				spin_lock_irqsave(mac_tx_lock(j), flags);
		}
		txc = mac_tx_complete_fifo(j);
		printk("txc=%p, txq = %d\n", txc, j);
		for(i=0;i<txc->size;i++) 
			printk("[%s]: txc->skbs[%d]: skb=%p, skb->data=%p\n", 
				__FUNCTION__, i, txc->skbs[i], 
				(txc->skbs[i]?txc->skbs[i]->data:0) );    
		printk("[%s]: head=%d, tail=%d\n", __FUNCTION__, 
				txc->head, txc->tail);
	}
	panic("BUG?\n");
	}

	/* process any pending transmits */
#ifdef MAC_TX_PENDING_QUEUES
	pending = mac_tx_complete_tx(txq);
#else
	pending = 1;
#endif

	spin_unlock(txl);

	/* release the skb and update statistics outside the spinlock */
	priv = netdev_priv(skb->dev);
	mac_stats_add(priv->stats.tx_packets, 1);
	mac_stats_add(priv->stats.tx_bytes, skb->len);
	dev = skb->dev;
	dev_kfree_skb_irq(skb);
	
	spin_lock(&priv->lock);
	if (mac_netif_tx_resume(pending) && netif_queue_stopped(dev)) {
		netif_wake_queue(dev);
		phnx_inc_counter(NETIF_START_Q);
	}
	spin_unlock(&priv->lock);
}

/* This function is called from an interrupt handler */
static void rmi_phnx_mac_msgring_handler(int bucket, int size, int code, 
		int stid, struct msgrng_msg *msg, void *data/* ignored */)
{
	unsigned long addr = 0;
	__u32 length = 0;
	int ctrl = 0, port=0;

	if (mac_stop.value) {
		for(;;) {
			__asm__ __volatile__ (".set mips64\nwait\n");
		}
	}
	
	phnx_dbg_msg("mac: bucket=%d, size=%d, code=%d, stid=%d, msg0=%llx" 
			"msg1=%llx\n", bucket, size, code, stid, 
			msg->msg0, msg->msg1);

	addr = (unsigned long)bus_to_virt(msg->msg0 & 0xffffffffe0ULL);

	if(xlr_revision_a0()){
		ctrl = (msg->msg0 >> 61) & 0x07;
		port = (msg->msg1 & 0x0f);
		length = ((msg->msg1 >> 40) & 0x3fff) ;
	}else { /* No control field in B0 */
		length = ((msg->msg0 >> 40) & 0x3fff) ;
		port = (msg->msg0 & 0x0f);
		if(length == 0)
			ctrl = CTRL_REG_FREE;
		else
			ctrl = CTRL_SNGL;
	}
	
	if (ctrl == CTRL_REG_FREE || ctrl == CTRL_JUMBO_FREE) {
		int txq = 0;

		/* Tx Complete */
		txq = (phoenix_cpu_id() << 2) + bucket;
		phnx_inc_counter(NETIF_TX_COMPLETE);

		mac_tx_complete(txq, addr);

		phnx_set_counter(NETIF_TX_COMPLETE_CYCLES, 
				(read_c0_count()-msgrng_msg_cycles));
	} else if (ctrl == CTRL_SNGL || ctrl == CTRL_START) {
		/* Rx Packet */
		int cpu = phoenix_cpu_id();
		struct driver_data *priv = 0;
		struct sk_buff *skb = 0;  

		length -= (BYTE_OFFSET + MAC_CRC_LEN);
		phnx_dbg_msg("Received packet, port = %d\n", port);

		if (xlr_revision_a0() && size != 2) {
			printk("[%s]: multi entry rx packet? size = %d\n", 
					__FUNCTION__, size);
			return;
		}
		skb = mac_get_skb_back_ptr(addr);
		if (!skb) {
			printk("[%s]: rx desc (0x%lx) for unknown skb? "
			"dropping packet\n", __FUNCTION__, addr);
			return;
		}

		skb->dev = dev_mac[port];
		priv = netdev_priv(skb->dev);

		/* if num frins to be sent exceeds threshold, 
		 * wake up the helper thread */
		if (atomic_inc_return(&priv->frin_to_be_sent[cpu]) 
				> MAC_FRIN_TO_BE_SENT_THRESHOLD) 
			schedule_work(&mac_frin_replenish_work[cpu]);

		/* compensate for the prepend data, byte offset */
		skb_reserve(skb, MAC_PREPAD+BYTE_OFFSET);
	  
		skb_put(skb, length);
		skb->protocol = eth_type_trans(skb, skb->dev);
		phnx_dbg_msg("gmac_%d: rx packet: addr = %lx, length = %x,"
			       "protocol=%d\n", priv->instance, addr, 
			       length, skb->protocol);

		mac_stats_add(priv->stats.rx_packets, 1);
		mac_stats_add(priv->stats.rx_bytes, skb->len);
		phnx_inc_counter(NETIF_RX);
		phnx_set_counter(NETIF_RX_CYCLES, 
				(read_c0_count()-msgrng_msg_cycles));
#ifdef MAC_NETIF_BYPASS
		{
		__u32 tx_cycles = read_c0_count();
		skb_push(skb, ETH_HLEN);
		rmi_phnx_mac_xmit(skb, skb->dev);
		phnx_set_counter(NETIF_TX_CYCLES, (read_c0_count()-tx_cycles));
		}
#else
		netif_rx(skb);
#endif
	}
	else {
		printk("[%s]: unrecognized ctrl=%d!\n", __FUNCTION__, ctrl);
	}

}

void mac_netif_start(void)
{
	int i=0;

	for(i=0;i<1;i++) {
		if (dev_mac[i] && netif_queue_stopped(dev_mac[i])) {
			netif_wake_queue(dev_mac[i]);
			phnx_inc_counter(NETIF_TIMER_START_Q);
		}
	}
}

void  phnx_mac_update_speed(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);
	u32 mii_status;

	mii_status = rmi_phnx_mac_mii_read(priv->instance, MII_NCONFIG);
	priv->speed = (mii_status >> 3) & 0x3;

	if (priv->speed == phnx_mac_speed_10) {
		phoenix_write_reg(priv->mmio, R_MAC_CONFIG_2, 0x7137);
		phoenix_write_reg(priv->mmio, R_CORECONTROL, 0x02);
		printk(KERN_INFO "configuring gmac_%d in 10Mbps mode\n", 
							priv->instance);
	}
	else if (priv->speed == phnx_mac_speed_100) {
		phoenix_write_reg(priv->mmio, R_MAC_CONFIG_2, 0x7137);
		phoenix_write_reg(priv->mmio, R_CORECONTROL, 0x01);      
		printk(KERN_INFO "configuring gmac_%d in 100Mbps mode\n", 
							priv->instance);
	}
	else {
		phoenix_write_reg(priv->mmio, R_MAC_CONFIG_2, 0x7237);
		phoenix_write_reg(priv->mmio, R_CORECONTROL, 0x00);      
		printk(KERN_INFO "configuring gmac_%d in 1000Mbps mode\n", 
							priv->instance);
	}
}

static void rmi_phnx_update_speed(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);
	int mii_status;


	mii_status = rmi_phnx_mac_mii_read(priv->instance, MII_BMSR);
	
	if ( (mii_status & BMSR_LSTATUS) != (priv->phy_oldlinkstat) ) {
		      priv->phy_oldlinkstat = mii_status & BMSR_LSTATUS;
		if (mii_status & BMSR_LSTATUS) {
			phnx_mac_update_speed(dev);
			printk(KERN_INFO "phnx mac %s link up\n", dev->name);
			netif_carrier_on(dev);
		}
		else {
			printk(KERN_INFO "phnx mac %s link down\n", dev->name);
			netif_carrier_off(dev);	
		}
	}
	
}

/**********************************************************************
 **********************************************************************/
static irqreturn_t rmi_phnx_mac_int_handler(int irq, 
	            void *dev_id, struct pt_regs *regs)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct driver_data *priv = netdev_priv(dev);
	phoenix_reg_t *mmio = priv->mmio;
	__u32 intreg = 0;
	
	phnx_dbg_msg("[%s]: mmio=%p\n", __FUNCTION__, mmio);

	intreg = phoenix_read_reg(mmio, R_INTREG);
	if (intreg & (1 << O_INTREG__MDInt)) {
		__u32 phy_int_status = 0;
		int i=0;

		for(i=0; i<PHOENIX_MAX_MACS; i++) {
			struct net_device *phy_dev = 0;
			struct driver_data *phy_priv = 0;
			
			phy_dev = dev_mac[i];
			phy_priv = netdev_priv(phy_dev);

			phy_int_status = 
				rmi_phnx_mac_mii_read(phy_priv->instance, 26);
			spin_lock(&phy_priv->lock);
			if(phy_priv->open == 0) {
				spin_unlock(&phy_priv->lock);
				continue;
			}

			rmi_phnx_update_speed(phy_dev);
			spin_unlock(&phy_priv->lock);
		}
	} else {
		if (!xlr_revision_a0()) {
			printk("[%s]: gmac_%d error interrupt: INTREG = 0x%08x\n", 
			       __FUNCTION__, priv->instance, intreg);
		}
	}
	/* clear all interrupts */
	phoenix_write_reg(mmio, R_INTREG, 0xffffffff);
	
	return IRQ_NONE;
}

/**********************************************************************
 **********************************************************************/
static int rmi_phnx_mac_open(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);
	int i=0;
	
	phnx_dbg_msg("IN\n");

#ifdef MAC_DISABLE_GMAC0
	/* TEMP Convenience HACK for gmac0 */
	if (priv->instance == 0) {
		printk("[%s]: gmac0 is currently disabled\n", __FUNCTION__);
		return -EBUSY;
	}
#endif

	spin_lock_irq(&priv->lock);

	if (rmi_phnx_mac_fill_rxfr(dev)) {
		spin_unlock_irq(&priv->lock);
		return -1;
	}

	phnx_mac_set_rx_mode(dev);
	
	if(xlr_revision_a0())
		phoenix_write_reg(priv->mmio, R_INTMASK, 0xfeffdffe);
	else {
		phoenix_write_reg(priv->mmio, R_INTMASK,
				(1<<O_INTMASK__TxIllegal)       |
				((priv->instance==0)<<O_INTMASK__MDInt) |
				(1<<O_INTMASK__TxFetchError)    |
				(1<<O_INTMASK__TagFull)         |
				(1<<O_INTMASK__AsyncFifoFull)   |
				(1<<O_INTMASK__Underrun)        |
				(1<<O_INTMASK__Abort)
				);

	}

	/* Set the timer to check for link beat. */
	if(xlr_revision_a0()) {
		init_timer(&priv->link_timer);
		priv->link_timer.expires = jiffies + 2 * HZ/100;
		priv->link_timer.data = (unsigned long)dev;
		priv->link_timer.function = &rmi_phnx_mac_timer;
		add_timer(&priv->link_timer);
	}
	
	rmi_phnx_update_speed(dev);
	rmi_phnx_mac_set_enable(priv, 1);
	
	netif_start_queue(dev);
	phnx_inc_counter(NETIF_START_Q);

	priv->open = 1;
	spin_unlock_irq(&priv->lock);

	for(i=0;i<8;i++)
		atomic_set(&priv->frin_to_be_sent[i], 0);
	
	return 0;
}

/**********************************************************************
 **********************************************************************/
static int rmi_phnx_mac_close(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);
	
	spin_lock_irq(&priv->lock);

	/* There may have left over skbs in the ring as well as in free in 
	 * they will be reused next time open is called 
	 */
	
	rmi_phnx_mac_set_enable(priv, 0);

	if(xlr_revision_a0())
		del_timer_sync(&priv->link_timer);

	netif_stop_queue(dev);
	phnx_inc_counter(NETIF_STOP_Q);

	priv->open = 0;
	spin_unlock_irq(&priv->lock);

	return 0;
}

/**********************************************************************
 **********************************************************************/
static void rmi_phnx_mac_timer(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
	struct driver_data *priv = netdev_priv(dev);
	int next_tick = HZ;

	spin_lock_irq (&priv->lock);
	rmi_phnx_update_speed(dev);
	spin_unlock_irq (&priv->lock);

	priv->link_timer.expires = jiffies + next_tick;
	add_timer(&priv->link_timer);
}

static void _phnx_get_mac_stats(struct net_device *dev, 
					struct net_device_stats *stats)
{
	struct driver_data *priv = netdev_priv(dev);

	stats->tx_errors = phoenix_read_reg(priv->mmio, TX_FCS_ERROR_COUNTER);
	stats->rx_dropped = phoenix_read_reg(priv->mmio, 
						RX_DROP_PACKET_COUNTER);
	stats->tx_dropped = phoenix_read_reg(priv->mmio, TX_DROP_FRAME_COUNTER);

	stats->multicast = phoenix_read_reg(priv->mmio, 
						RX_MULTICAST_PACKET_COUNTER);
	stats->collisions = phoenix_read_reg(priv->mmio, 
						TX_TOTAL_COLLISION_COUNTER);

	stats->rx_length_errors = phoenix_read_reg(priv->mmio, 
						RX_FRAME_LENGTH_ERROR_COUNTER);
	stats->rx_over_errors = phoenix_read_reg(priv->mmio, 
						RX_DROP_PACKET_COUNTER);
	stats->rx_crc_errors = phoenix_read_reg(priv->mmio, 
						RX_FCS_ERROR_COUNTER);
	stats->rx_frame_errors = phoenix_read_reg(priv->mmio, 
						RX_ALIGNMENT_ERROR_COUNTER);

	stats->rx_fifo_errors = phoenix_read_reg(priv->mmio,
					    	RX_DROP_PACKET_COUNTER);
	stats->rx_missed_errors = phoenix_read_reg(priv->mmio,
					    	RX_CARRIER_SENSE_ERROR_COUNTER);

	stats->rx_errors = (stats->rx_over_errors + stats->rx_crc_errors +
			     stats->rx_frame_errors + stats->rx_fifo_errors +
			     stats->rx_missed_errors);

	stats->tx_aborted_errors = phoenix_read_reg(priv->mmio, 
			TX_EXCESSIVE_COLLISION_PACKET_COUNTER);
	stats->tx_carrier_errors = phoenix_read_reg(priv->mmio, 
					TX_DROP_FRAME_COUNTER);
	stats->tx_fifo_errors = phoenix_read_reg(priv->mmio, 
					TX_DROP_FRAME_COUNTER);

}
/**********************************************************************
 **********************************************************************/
static struct net_device_stats *rmi_phnx_mac_get_stats(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);
	unsigned long flags;
	
	spin_lock_irqsave(&priv->lock, flags);
	
	/* update other stats here */
	_phnx_get_mac_stats(dev, &priv->stats);
	
	spin_unlock_irqrestore(&priv->lock, flags);
	
	return &priv->stats;
}

/**********************************************************************
 **********************************************************************/
static void rmi_phnx_mac_set_multicast_list(struct net_device *dev)
{
	uint32_t regval;
	struct driver_data *priv = netdev_priv(dev);

	
	regval = phoenix_read_reg(priv->mmio, R_MAC_FILTER_CONFIG);

	if (dev->flags & IFF_ALLMULTI) {
		regval |= (1<<O_MAC_FILTER_CONFIG__ALL_MCAST_EN);
		phoenix_write_reg(priv->mmio, R_MAC_FILTER_CONFIG, regval);
	}else if(dev->mc_count < 1) {
		regval &= ~(1<<O_MAC_FILTER_CONFIG__ALL_MCAST_EN);
		phoenix_write_reg(priv->mmio, R_MAC_FILTER_CONFIG, regval);
	}else {
		regval |= (1<<O_MAC_FILTER_CONFIG__ALL_MCAST_EN);
		phoenix_write_reg(priv->mmio, R_MAC_FILTER_CONFIG, regval);
	}

}


/**********************************************************************
 **********************************************************************/

static int phnx_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct driver_data *priv = netdev_priv(dev);
	int mii_status;

	cmd->supported = SUPPORTED_10baseT_Full | SUPPORTED_10baseT_Half | 
			SUPPORTED_100baseT_Full | SUPPORTED_100baseT_Half |
			SUPPORTED_1000baseT_Full | SUPPORTED_MII |
			SUPPORTED_Autoneg | SUPPORTED_TP;

	cmd->advertising = priv->advertising;

	mii_status = rmi_phnx_mac_mii_read(priv->instance, MII_NCONFIG);
	priv->speed = (mii_status >> 3) & 0x03;

	cmd->speed = (priv->speed == phnx_mac_speed_1000) ? SPEED_1000 :
		(priv->speed == phnx_mac_speed_100) ? SPEED_100: SPEED_10;

	cmd->duplex = (mii_status >> 5) & 0x1;
	cmd->port = PORT_TP;
	cmd->phy_address = priv->instance;
	cmd->transceiver = XCVR_INTERNAL;
	cmd->autoneg = (~(mii_status >> 14)) & 0x1;
	cmd->maxtxpkt = 0;
	cmd->maxrxpkt = 0;

	return 0;
}

static void phnx_get_drvinfo(struct net_device *dev, 
				struct ethtool_drvinfo *info)
{
	strcpy(info->driver, DRV_NAME);
	strcpy(info->version, DRV_VERSION);
}

static int phnx_get_regs_len(struct net_device *dev) 
{
	return PHNX_ETHTOOL_REG_LEN;
}
static void phnx_get_regs(struct net_device *dev,
				struct ethtool_regs *regs, void *p)
{
	u32 *data = (u32 *)p;
	int i;
	struct driver_data *priv = netdev_priv(dev);
	u32 flags;

	memset((void *)data, 0, PHNX_ETHTOOL_REG_LEN);

	spin_lock_irqsave(&priv->lock, flags);
	for(i=0; i <= PHNX_NUM_REG_DUMP; i++)
		*(data + i) = phoenix_read_reg(priv->mmio,  R_TX_CONTROL + i);
	spin_unlock_irqrestore(&priv->lock, flags);
}
static u32 phnx_get_msglevel(struct net_device *dev)
{
	return mac_debug;
}
static void phnx_set_msglevel(struct net_device *dev, u32 value)
{
	mac_debug = value;
}

static int phnx_nway_reset(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);
	int mii_status;
	u32 flags;
	int ret = -EINVAL;

	spin_lock_irqsave(&priv->lock, flags);
	mii_status = rmi_phnx_mac_mii_read(priv->instance, MII_BMCR);
	if(mii_status & BMCR_ANENABLE)
	{
		rmi_phnx_mac_mii_write(priv->instance, 
				MII_BMCR, BMCR_ANRESTART | mii_status);
		ret = 0;
	}
	spin_unlock_irqrestore(&priv->lock, flags);
	return ret;
}
static u32 phnx_get_link(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);
	int mii_status;
	u32 flags;

	spin_lock_irqsave(&priv->lock, flags);
	mii_status = rmi_phnx_mac_mii_read(priv->instance, MII_BMSR);

	spin_unlock_irqrestore(&priv->lock, flags);

	if(mii_status & BMSR_LSTATUS)
		return 1;
	return 0;
}
#define PHNX_STATS_KEY_LEN  \
		(sizeof(struct net_device_stats) / sizeof(unsigned long))
static struct {
	        const char string[ETH_GSTRING_LEN];
} phnx_ethtool_stats_keys[PHNX_STATS_KEY_LEN] = {
	{ "rx_packets" },
	{ "tx_packets" },
	{ "rx_bytes" },
	{ "tx_bytes" },
	{ "rx_errors" },
	{ "tx_errors" },
	{ "rx_dropped" },
	{ "tx_dropped" },
	{ "multicast" },
	{ "collisions" },
	{ "rx_length_errors" },
	{ "rx_over_errors" },
	{ "rx_crc_errors" },
	{ "rx_frame_errors" },
	{ "rx_fifo_errors" },
	{ "rx_missed_errors" },
	{ "tx_aborted_errors" },
	{ "tx_carrier_errors" },
	{ "tx_fifo_errors" },
	{ "tx_heartbeat_errors" },
	{ "tx_window_errors" },
	{ "rx_compressed" },
	{ "tx_compressed" }
};
static int phnx_get_stats_count (struct net_device *dev)
{
	return PHNX_STATS_KEY_LEN;
}

static void phnx_get_strings (struct net_device *dev, u32 stringset, u8 *buf)
{
	switch (stringset) {
	case ETH_SS_STATS:
		memcpy(buf, &phnx_ethtool_stats_keys, 
				sizeof(phnx_ethtool_stats_keys));
		break;
	default:
		printk(KERN_WARNING "%s: Invalid stringset %d\n", 
				__FUNCTION__, stringset);
		break;
	}
}

static void phnx_get_ethtool_stats (struct net_device *dev,
			struct ethtool_stats *estats, u64 *stats)
{
	int i;
	struct driver_data *priv = netdev_priv(dev);
	unsigned long flags;
	u32 *tmp_stats;
	
	spin_lock_irqsave(&priv->lock, flags);
	
	_phnx_get_mac_stats(dev, &priv->stats);
	
	spin_unlock_irqrestore(&priv->lock, flags);

	tmp_stats = (u32 *)&priv->stats;
	for(i=0; i < PHNX_STATS_KEY_LEN; i++) {
		*stats = (u64)*tmp_stats;
		stats++;
		tmp_stats++;
	}
}

static int phnx_enable_autoneg(struct net_device *dev, u32 adv)
{
	struct driver_data *priv = netdev_priv(dev);
	int mii_status;
	u32 flags, adv1, adv2;

	spin_lock_irqsave(&priv->lock, flags);
	rmi_phnx_mac_set_enable(priv, 0);
	/* advertising for 10/100 Mbps */
	adv1 = rmi_phnx_mac_mii_read(priv->instance, MII_ADVERTISE);
	adv1 &= ~(ADVERTISE_ALL | ADVERTISE_100BASE4);
	/* advertising for 1000 Mbps */
	adv2 = rmi_phnx_mac_mii_read(priv->instance, 0x9);
	adv2 &= ~(0x300);

	if(adv & ADVERTISED_10baseT_Half)
		adv1 |= ADVERTISE_10HALF;
	if(adv & ADVERTISED_10baseT_Full)
		adv1 |= ADVERTISE_10FULL;
	if(adv & ADVERTISED_100baseT_Full)
		adv1 |= ADVERTISE_100FULL;
	if(adv & ADVERTISED_100baseT_Half)
		adv1 |= ADVERTISE_100HALF;

	if(adv & ADVERTISED_1000baseT_Full)
		adv2 |= 0x200;
	if(adv & ADVERTISED_1000baseT_Half)
		adv2 |= 0x100;

	/* Set the advertising parameters */
	rmi_phnx_mac_mii_write(priv->instance, MII_ADVERTISE, adv1);
	rmi_phnx_mac_mii_write(priv->instance, 0x9, adv2);

	priv->advertising = adv1 | adv2;

	mii_status = rmi_phnx_mac_mii_read(priv->instance, MII_BMCR);
	/* enable autoneg and force restart autoneg */
	mii_status |= (BMCR_ANENABLE | BMCR_ANRESTART);
	rmi_phnx_mac_mii_write(priv->instance, MII_BMCR, mii_status);

	rmi_phnx_mac_set_enable(priv, 1);
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int phnx_set_link_speed(struct net_device *dev, int speed, int duplex)
{
	struct driver_data *priv = netdev_priv(dev);
	int mii_status = 0, tmp_status;
	u32 flags;
	int ret = 0;

	switch(speed) {
		case SPEED_10:
			mii_status = 0;
			priv->speed = phnx_mac_speed_10;
			break;
		case SPEED_100:
			mii_status = 0x2000;
			priv->speed = phnx_mac_speed_100;
			break;
		case SPEED_1000:
			mii_status = 0x0040;
			priv->speed = phnx_mac_speed_1000;
			break;
		default:
			ret = -EINVAL;
			return ret;
	}
	if(duplex == DUPLEX_FULL)
		mii_status |= BMCR_FULLDPLX;

	spin_lock_irqsave(&priv->lock, flags);
	rmi_phnx_mac_set_enable(priv, 0);

	tmp_status = rmi_phnx_mac_mii_read(priv->instance, MII_BMCR);
	/* Turn off autoneg, speed and duplexity setting */
	tmp_status &= ~(BMCR_ANENABLE | 0x0040 | 0x2000 | BMCR_FULLDPLX);

	rmi_phnx_mac_mii_write(priv->instance, 
				MII_BMCR, tmp_status | mii_status);

	if (priv->speed == phnx_mac_speed_10) {
		phoenix_write_reg(priv->mmio, R_MAC_CONFIG_2, 0x7137);
		phoenix_write_reg(priv->mmio, R_CORECONTROL, 0x02);
		printk(KERN_INFO "configuring gmac_%d in 10Mbps mode\n", 
						priv->instance);
	}
	else if (priv->speed == phnx_mac_speed_100) {
		phoenix_write_reg(priv->mmio, R_MAC_CONFIG_2, 0x7137);
		phoenix_write_reg(priv->mmio, R_CORECONTROL, 0x01);      
		printk(KERN_INFO "configuring gmac_%d in 100Mbps mode\n", 
						priv->instance);
	}
	else {
		phoenix_write_reg(priv->mmio, R_MAC_CONFIG_2, 0x7237);
		phoenix_write_reg(priv->mmio, R_CORECONTROL, 0x00);      
		printk(KERN_INFO "configuring gmac_%d in 1000Mbps mode\n", 
						priv->instance);
	}
	rmi_phnx_mac_set_enable(priv, 1);
	spin_unlock_irqrestore(&priv->lock, flags);
	return 0;

}

static int phnx_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	int ret;
	if (cmd->autoneg == AUTONEG_ENABLE) {
		ret = phnx_enable_autoneg(dev, cmd->advertising);
	}else {
		ret = phnx_set_link_speed(dev, cmd->speed, cmd->duplex);
	}

	return ret;
}

/**********************************************************************
 **********************************************************************/
static int rmi_phnx_mac_do_ioctl(struct net_device *dev, struct ifreq *ifr, 
					int cmd)
{
	struct mii_ioctl_data *data = if_mii(ifr);
	struct driver_data *priv = netdev_priv(dev);

	switch(cmd) {
	case SIOCGMIIPHY:
		data->phy_id = priv->instance;

		/* fallthru */
	case SIOCGMIIREG: 
		spin_lock_irq(&priv->lock);
		data->val_out = rmi_phnx_mac_mii_read(priv->instance,
						data->reg_num & 0x1f);
		spin_unlock_irq(&priv->lock);

		return 0;

	case SIOCSMIIREG:
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;

		spin_lock_irq(&priv->lock);
		rmi_phnx_mac_mii_write(priv->instance, data->reg_num & 0x1f,
							data->val_in);
		spin_unlock_irq(&priv->lock);
		return 0;

	default:
		/* do nothing */
		break;
	}
	return -EOPNOTSUPP;
}


/**********************************************************************
 **********************************************************************/
static void rmi_phnx_mac_tx_timeout(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);

	spin_lock_irq (&priv->lock);
	
	dev->trans_start = jiffies;
	mac_stats_add(priv->stats.tx_errors, 1);
	

	netif_wake_queue(dev);
	spin_unlock_irq (&priv->lock);
	phnx_inc_counter(NETIF_START_Q);

	printk (KERN_WARNING "%s: Transmit timed out\n",dev->name);
}

/**********************************************************************
 **********************************************************************/
static int rmi_phnx_mac_change_mtu(struct net_device *dev, int new_mtu)
{
	struct driver_data *priv = netdev_priv(dev);
	unsigned long flags;

	if ((new_mtu > 1500) || (new_mtu < 64)) {
		return -EINVAL;
	}

	spin_lock_irqsave(&priv->lock, flags);

	dev->mtu = new_mtu;

	if (netif_running(dev)) {
		/* Disable MAC TX/RX */
		netif_stop_queue(dev);
		mdelay(100);
		rmi_phnx_mac_set_enable(priv, 0);

		rmi_phnx_mac_set_enable(priv, 1);
		netif_wake_queue(dev);
	}

	spin_unlock_irqrestore(&priv->lock, flags);
	return 0;            
}

/**********************************************************************
 **********************************************************************/
static int rmi_phnx_mac_fill_rxfr(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);
	struct sk_buff *skb = 0;
	unsigned long msgrng_flags;
	int i;
	int ret = 0;

	for(i=0; i<MAX_NUM_DESC; i++) {
		skb = rmi_phnx_alloc_skb();
		if (!skb) {
			ret = -ENOMEM;
			break;
		}
		skb->dev = dev;
		/* Send the free Rx desc to the MAC */
		msgrng_access_enable(msgrng_flags);
		mac_put_skb_back_ptr(skb);
		if (phnx_mac_send_fr(priv, virt_to_bus(skb->data), skb->len)) {
			dev_kfree_skb(skb);
			printk("message_send failed!, unable to send free "
				"desc to mac\n");
			ret = -EIO;
			msgrng_access_disable(msgrng_flags);
			break;
		}
		msgrng_access_disable(msgrng_flags);
	}

	return ret;
}

/**********************************************************************
 **********************************************************************/
static __inline__ void *rmi_phnx_config_spill(phoenix_reg_t *mmio, 
		int reg_start_0, int reg_start_1, int reg_size, int size)
{
	__u32 spill_size = CACHELINE_ALIGNED_ADDR(size);
	void *spill = cacheline_aligned_kmalloc(spill_size, GFP_KERNEL);
	__u64 phys_addr = 0;

	if (!spill) {
		panic("Unable to allocate memory for spill area!\n");
	}
	phys_addr = virt_to_phys(spill);
	phoenix_write_reg(mmio, reg_start_0, (phys_addr >> 5) & 0xffffffff);
	phoenix_write_reg(mmio, reg_start_1, (phys_addr >> 37) & 0x07);
	phoenix_write_reg(mmio, reg_size, spill_size);

	return spill;
}

static void 
rmi_phnx_config_spill_area(struct driver_data *priv)
{
	/* if driver initialization is done parallely on multiple cpus
	 * spill_configured needs synchronization 
	 */
	static int spill_configured;

	if (!spill_configured) {
		priv->frin_spill = rmi_phnx_config_spill(priv->mmio, 
				R_REG_FRIN_SPILL_MEM_START_0, 
				R_REG_FRIN_SPILL_MEM_START_1, 
				R_REG_FRIN_SPILL_MEM_SIZE, 
				MAX_FRIN_SPILL * sizeof(struct fr_desc));

		priv->class_0_spill = rmi_phnx_config_spill(priv->mmio, 
				R_CLASS0_SPILL_MEM_START_0, 
				R_CLASS0_SPILL_MEM_START_1, 
				R_CLASS0_SPILL_MEM_SIZE, 
				MAX_CLASS_0_SPILL * sizeof(union rx_tx_desc));
		priv->class_1_spill = rmi_phnx_config_spill(priv->mmio, 
				R_CLASS1_SPILL_MEM_START_0, 
				R_CLASS1_SPILL_MEM_START_1, 
				R_CLASS1_SPILL_MEM_SIZE, 
				MAX_CLASS_1_SPILL * sizeof(union rx_tx_desc));

#if !defined(MAC_SPLIT_MODE)

		priv->frout_spill = rmi_phnx_config_spill(priv->mmio, 
				R_FROUT_SPILL_MEM_START_0, 
				R_FROUT_SPILL_MEM_START_1, 
				R_FROUT_SPILL_MEM_SIZE, 
				MAX_FROUT_SPILL * sizeof(struct fr_desc));

		priv->class_2_spill = rmi_phnx_config_spill(priv->mmio, 
				R_CLASS2_SPILL_MEM_START_0, 
				R_CLASS2_SPILL_MEM_START_1, 
				R_CLASS2_SPILL_MEM_SIZE, 
				MAX_CLASS_2_SPILL * sizeof(union rx_tx_desc));

		priv->class_3_spill = rmi_phnx_config_spill(priv->mmio, 
				R_CLASS3_SPILL_MEM_START_0, 
				R_CLASS3_SPILL_MEM_START_1, 
				R_CLASS3_SPILL_MEM_SIZE, 
				MAX_CLASS_3_SPILL * sizeof(union rx_tx_desc));
#endif
		spill_configured = 1;
	}
}

/*****************************************************************
 * Write the MAC address to the PHNX registers
 * All 4 addresses are the same for now
 *****************************************************************/
static void
phnx_mac_setup_hwaddr(struct driver_data *priv)
{
	struct net_device *dev = priv->dev;

	phoenix_write_reg(priv->mmio, R_MAC_ADDR0, 
		  ((dev->dev_addr[5]<<24)|(dev->dev_addr[4]<<16)
		   |(dev->dev_addr[3]<<8)|(dev->dev_addr[2]))
		  );

	phoenix_write_reg(priv->mmio, R_MAC_ADDR0+1, 
		  ((dev->dev_addr[1]<<24)|(dev->dev_addr[0]<<16)));

	phoenix_write_reg(priv->mmio, R_MAC_ADDR_MASK2,   0xffffffff);

	phoenix_write_reg(priv->mmio, R_MAC_ADDR_MASK2+1, 0xffffffff);

	phoenix_write_reg(priv->mmio, R_MAC_ADDR_MASK3,   0xffffffff);

	phoenix_write_reg(priv->mmio, R_MAC_ADDR_MASK3+1, 0xffffffff);

	if (xlr_hybrid_user_mac()) {
					phoenix_write_reg(priv->mmio, R_MAC_FILTER_CONFIG,
													(1 << O_MAC_FILTER_CONFIG__BROADCAST_EN) |
													(1 << O_MAC_FILTER_CONFIG__ALL_MCAST_EN) |
													(1 << O_MAC_FILTER_CONFIG__ALL_UCAST_EN) |
													(1 << O_MAC_FILTER_CONFIG__MAC_ADDR0_VALID)
													);
	} else {

	phoenix_write_reg(priv->mmio, R_MAC_FILTER_CONFIG, 
		  (1<<O_MAC_FILTER_CONFIG__BROADCAST_EN) |
		  (1<<O_MAC_FILTER_CONFIG__ALL_MCAST_EN) |
		  (1<<O_MAC_FILTER_CONFIG__MAC_ADDR0_VALID)
		  );
	}
}

/*****************************************************************
 * Read the MAC address from the PHNX registers
 * All 4 addresses are the same for now
 *****************************************************************/
static void
phnx_mac_get_hwaddr(struct net_device *dev)
{
	struct driver_data *priv = netdev_priv(dev);

	dev->dev_addr[0] = phoenix_base_mac_addr[0];
	dev->dev_addr[1] = phoenix_base_mac_addr[1];
	dev->dev_addr[2] = phoenix_base_mac_addr[2];
	dev->dev_addr[3] = phoenix_base_mac_addr[3];
	dev->dev_addr[4] = phoenix_base_mac_addr[4];
	dev->dev_addr[5] = phoenix_base_mac_addr[5] + priv->instance;
}

/*****************************************************************
 * Mac Module Initialization
 *****************************************************************/
static void mac_common_init(void)
{
	int i=0;

	for(i=0;i<MAC_TX_QUEUES;i++) {
		spin_lock_init(mac_tx_lock(i));
		fifo_init(mac_tx_pending_fifo(i), MAC_TX_PENDING_FIFO_SIZE);
		fifo_init(mac_tx_complete_fifo(i), MAC_TX_COMPLETE_FIFO_SIZE);
	}

	for(i=0;i<8;i++)
		INIT_WORK(&mac_frin_replenish_work[i],mac_frin_replenish, NULL);

	if(xlr_hybrid_user_mac())	return;
	/* Register a bucket handler with the phoenix messaging subsystem */   
	if (register_msgring_handler(TX_STN_GMAC, 
				rmi_phnx_mac_msgring_handler, NULL)) {
		panic("Couldn't register msgring handler\n");
	}
}

static struct ethtool_ops rmi_phnx_ethtool_ops = {
	.get_settings		= phnx_get_settings,
	.set_settings		= phnx_set_settings,
	.get_drvinfo		= phnx_get_drvinfo,
	.get_regs_len		= phnx_get_regs_len,
	.get_regs		= phnx_get_regs,
	.get_msglevel		= phnx_get_msglevel,
	.set_msglevel		= phnx_set_msglevel,
	.nway_reset		= phnx_nway_reset,
	.get_link		= phnx_get_link,
	.get_strings		= phnx_get_strings,
	.get_stats_count	= phnx_get_stats_count,
	.get_ethtool_stats	= phnx_get_ethtool_stats,
};

int rmi_phnx_mac_init_module(void)
{
	struct net_device  *dev  = 0;
	struct driver_data *priv = 0;
	unsigned long mmio_start=0;
	int i = 0;
	int ret = 0;

	for (i=0; i<PHOENIX_MAX_MACS; i++) {
		phnx_dbg_msg("Registering phnx_mac[%d]\n", i);

		dev = alloc_etherdev(sizeof(struct driver_data));
		if (!dev) {
			ret = -ENOMEM;
			goto out;
		}

		priv = netdev_priv(dev);
		priv->dev= dev;
		priv->open = 0;

		mmio_start = phoenix_io_base + 
				phnx_mac_devices[i].phnx_io_offset;
		priv->mmio = (phoenix_reg_t *)mmio_start;
		if (!priv->mmio) {
			rmi_dbg_panic("Unable to ioremap MMIO region of "
				"size %x @ %lx\n", PHOENIX_IO_SIZE, mmio_start);
		}

		phnx_dbg_msg("mmio_start=%lx, priv->mmio=%p\n", 
				mmio_start, priv->mmio);

		/* Initialize the net_device */
		dev->irq = phnx_mac_devices[i].irq;
		if (request_irq(dev->irq, rmi_phnx_mac_int_handler, 
	                            SA_INTERRUPT, dev->name, dev)) {
			ret = -EBUSY;
			panic("Couldn't get mac interrupt line (%d)", dev->irq);
		}

		ether_setup(dev);

		dev->base_addr          = mmio_start;
		dev->mem_end            = mmio_start + PHOENIX_IO_SIZE - 1;

		dev->open               = rmi_phnx_mac_open;
		dev->hard_start_xmit    = rmi_phnx_mac_xmit;
		dev->stop               = rmi_phnx_mac_close;
		dev->get_stats          = rmi_phnx_mac_get_stats;
		dev->set_multicast_list = rmi_phnx_mac_set_multicast_list;
		dev->do_ioctl           = rmi_phnx_mac_do_ioctl;
		dev->ethtool_ops 	= &rmi_phnx_ethtool_ops;
		dev->tx_timeout         = rmi_phnx_mac_tx_timeout;
		dev->watchdog_timeo     = (1000 * HZ);
		dev->change_mtu         = rmi_phnx_mac_change_mtu;
		dev->tx_queue_len       = 10000;

		/* Initialize the device specific driver data */    
		spin_lock_init(&priv->lock);

		priv->id = i;
		priv->instance = phnx_mac_devices[i].instance;
		priv->type = phnx_mac_devices[i].type;

		if (priv->type == TYPE_GMAC) {
			rmi_phnx_gmac_init(priv);
		} else if (priv->type == TYPE_XGMAC) {
			rmi_phnx_xgmac_init(priv);
		}
 
		phnx_mac_get_hwaddr(dev);
	
		phnx_mac_setup_hwaddr(priv);    

		if(!xlr_hybrid_user_mac()){
#ifdef MAC_SPLIT_MODE
		if (priv->instance < 2) {
			ret = register_netdev(dev);
			if (ret) {
				rmi_dbg_panic("Unable to register net device");
			}
		}
#else
		ret = register_netdev(dev);
		if (ret) {
			rmi_dbg_panic("Unable to register net device\n");
		}
#endif
 
		phnx_dbg_msg("%s: Phoenix Mac at 0x%p (mtu=%d)\n", 
	                              dev->name, priv->mmio, dev->mtu);
		}
		dev_mac[i] = dev;
	
	}

	mac_common_init();
	
out:
	if (ret < 0) {
		rmi_dbg_panic("Error, ret = %d\n", ret);
	}
	return ret;
}

/**********************************************************************
 **********************************************************************/
void rmi_phnx_mac_exit_module(void)
{
	struct net_device *dev;
	int idx;

	for (idx = 0; idx < PHOENIX_MAX_MACS; idx++) {
		dev = dev_mac[idx];
		if (!dev)
			continue;

		unregister_netdev(dev);
		free_netdev(dev);
	}
}

module_init(rmi_phnx_mac_init_module);
module_exit(rmi_phnx_mac_exit_module);




int phnx_get_reg_bit(phoenix_reg_t *mmio, int reg, int bit_no)
{
	unsigned long   int mask = 0x1;

	mask <<= bit_no;
	if(phoenix_read_reg(mmio, reg)& mask)
	{
		return 1;
	}
	return 0;
}


void phnx_clear_mac_stats(int gmac_id)
{
	unsigned int  		value;
	struct driver_data       *priv;
	phoenix_reg_t 		*mmio;
	struct net_device *dev  = 0;

	dev = dev_mac[gmac_id];
	if(dev==NULL) return;
	priv  = netdev_priv(dev);
	if(priv==NULL) return;
	mmio = priv->mmio;
	if(mmio==NULL) return;

	/* first clear the stat then again unset it*/
	value = phoenix_read_reg(mmio, R_STATCTRL);
	value |= 0x2;
	phoenix_write_reg(mmio, R_STATCTRL,value);
	value &= ~0x2;
	phoenix_write_reg(mmio, R_STATCTRL,value);


	return;
}


void phnx_get_mac_tx_stats(struct driver_data* priv,  mac_tx_statistics *tx_stats)
{

	tx_stats->tx_bytes = phoenix_read_reg(priv->mmio, TX_BYTE_COUNTER);
	tx_stats->tx_bytes_carry = phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 13);

	tx_stats->tx_pkts = phoenix_read_reg(priv->mmio,TX_PACKET_COUNTER);
	tx_stats->tx_pkts_carry = phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 12);

	tx_stats->tx_multicast_pkts = 
		phoenix_read_reg(priv->mmio,TX_MULTICAST_PACKET_COUNTER);
	tx_stats->tx_multicast_pkts_carry = 
		phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 11);

	tx_stats->tx_broadcast_pkts = 
		phoenix_read_reg(priv->mmio,TX_BROADCAST_PACKET_COUNTER);
	tx_stats->tx_broadcast_pkts_carry = 
		phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 10);

	tx_stats->tx_pause_control_frame_counter = 
		phoenix_read_reg(priv->mmio,TX_PAUSE_CONTROL_FRAME_COUNTER);
	tx_stats->tx_pause_control_frame_counter_carry = 
		phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 9);

	tx_stats->tx_deferral_pkt_counter = 
		phoenix_read_reg(priv->mmio,TX_DEFERRAL_PACKET_COUNTER);
	tx_stats->tx_deferral_pkt_counter_carry = 
		phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 8);

	tx_stats->tx_excessive_deferral_pkt_counter = 
		phoenix_read_reg(priv->mmio,
				TX_EXCESSIVE_DEFERRAL_PACKET_COUNTER);
	tx_stats->tx_excessive_deferral_pkt_counter_carry = 
		phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 7);

	tx_stats->tx_single_collision_pkt_counter = 
		phoenix_read_reg(priv->mmio,TX_SINGLE_COLLISION_PACKET_COUNTER);
	tx_stats->tx_single_collision_pkt_counter_carry = 
		phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 6);

	tx_stats->tx_multi_collision_pkt_counter = 
		phoenix_read_reg(priv->mmio,TX_MULTI_COLLISION_PACKET_COUNTER);
	tx_stats->tx_multi_collision_pkt_counter_carry = 
		phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 5);

	tx_stats->tx_late_collision_pkt_counter = 
		phoenix_read_reg(priv->mmio,TX_LATE_COLLISION_PACKET_COUNTER);
	tx_stats->tx_late_collision_pkt_counter_carry =  phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 4);

	tx_stats->tx_excessive_collision_pkt_counter = 
	phoenix_read_reg(priv->mmio,TX_EXCESSIVE_COLLISION_PACKET_COUNTER);
	tx_stats->tx_excessive_collision_pkt_counter_carry =  phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 3);

	tx_stats->tx_total_collision_counter = 
		phoenix_read_reg(priv->mmio,TX_TOTAL_COLLISION_COUNTER);
	tx_stats->tx_total_collision_counter_carry = 
		phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 2);

	tx_stats->tx_pause_frame_honered_counter = 
		phoenix_read_reg(priv->mmio,TX_PAUSE_FRAME_HONERED_COUNTER);
	tx_stats->tx_pause_frame_honered_counter_carry = 
		phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 1);

	tx_stats->tx_drop_frame_counter = 
		phoenix_read_reg(priv->mmio,TX_DROP_FRAME_COUNTER);
	tx_stats->tx_drop_frame_counter_carry = 
		phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 0);

	tx_stats->tx_jabber_frame_counter = 
		phoenix_read_reg(priv->mmio,TX_JABBER_FRAME_COUNTER);
	tx_stats->tx_jabber_frame_counter_carry = 
		phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 19);

	tx_stats->tx_fcs_error_counter =  
		phoenix_read_reg(priv->mmio,TX_FCS_ERROR_COUNTER);
	tx_stats->tx_fcs_error_counter_carry = 
		phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 18);

	tx_stats->tx_control_frame_counter = 
		phoenix_read_reg(priv->mmio,TX_CONTROL_FRAME_COUNTER);
	tx_stats->tx_control_frame_counter_carry = 
		phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 17);

	tx_stats->tx_oversize_frame_counter = 
		phoenix_read_reg(priv->mmio,TX_OVERSIZE_FRAME_COUNTER);
	tx_stats->tx_oversize_frame_counter_carry =  phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 16);

	tx_stats->tx_undersize_frame_counter = 
		phoenix_read_reg(priv->mmio,TX_UNDERSIZE_FRAME_COUNTER);
	tx_stats->tx_undersize_frame_counter_carry = 
		phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 15);

	tx_stats->tx_fragment_frame_counter =  
		phoenix_read_reg(priv->mmio,TX_FRAGMENT_FRAME_COUNTER);
	tx_stats->tx_fragment_frame_counter_carry = 
		phnx_get_reg_bit(priv->mmio,CARRY_REG_2, 14);
	return;
}


void phnx_get_mac_rx_stats(struct driver_data *priv, mac_rx_statistics *rx_stats)
{
	rx_stats->rx_bytes = phoenix_read_reg(priv->mmio, RX_BYTE_COUNTER);
	rx_stats->rx_bytes_carry = phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 16);

	rx_stats->rx_pkts = phoenix_read_reg(priv->mmio,RX_PACKET_COUNTER);
	rx_stats->rx_pkts_carry =  
		phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 15);

	rx_stats->rx_fcs_error_counter = 
		phoenix_read_reg(priv->mmio,RX_FCS_ERROR_COUNTER);
	rx_stats->rx_fcs_error_counter_carry = 
		phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 14);

	rx_stats->rx_multicast_pkt_counter =  
		phoenix_read_reg(priv->mmio,RX_MULTICAST_PACKET_COUNTER);
	rx_stats->rx_multicast_pkt_counter_carry = 
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 13);

	rx_stats->rx_broadcast_pkt_counter = 
	       	phoenix_read_reg(priv->mmio,RX_BROADCAST_PACKET_COUNTER);
	rx_stats->rx_broadcast_pkt_counter_carry =
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 12);

	rx_stats->rx_control_frame_pkt_counter =
	       	phoenix_read_reg(priv->mmio,RX_CONTROL_FRAME_PACKET_COUNTER);
	rx_stats->rx_control_frame_pkt_counter_carry = 
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 11);

	rx_stats->rx_pause_frame_pkt_counter =
	       	phoenix_read_reg(priv->mmio,RX_PAUSE_FRAME_PACKET_COUNTER);
	rx_stats->rx_pause_frame_pkt_counter_carry =
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 10);

	rx_stats->rx_unknown_opcode_pkt_counter =
	       	phoenix_read_reg(priv->mmio,RX_UNKNOWN_OP_CODE_COUNTER);
	rx_stats->rx_unknown_opcode_pkt_counter_carry =
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 9);

	rx_stats->rx_alignment_error_counter =
	       	phoenix_read_reg(priv->mmio,RX_ALIGNMENT_ERROR_COUNTER);
	rx_stats->rx_alignment_error_counter_carry =
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 8);

	rx_stats->rx_frame_length_error_counter =
	       	phoenix_read_reg(priv->mmio,RX_FRAME_LENGTH_ERROR_COUNTER);
	rx_stats->rx_frame_length_error_counter_carry = 
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 7);

	rx_stats->rx_code_error_counter = 
	       	phoenix_read_reg(priv->mmio,RX_CODE_ERROR_COUNTER);
	rx_stats->rx_code_error_counter_carry =
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 6);

	rx_stats->rx_carrier_sense_error_counter = 
	       	phoenix_read_reg(priv->mmio,RX_CARRIER_SENSE_ERROR_COUNTER);
	rx_stats->rx_carrier_sense_error_counter_carry =
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 5);

	rx_stats->rx_undersize_pkt_counter =
	       	phoenix_read_reg(priv->mmio,RX_UNDERSIZE_PACKET_COUNTER);
	rx_stats->rx_undersize_pkt_counter_carry =
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 4);

	rx_stats->rx_oversize_pkt_counter =
	       	phoenix_read_reg(priv->mmio,RX_OVERSIZE_PACKET_COUNTER);
	rx_stats->rx_oversize_pkt_counter_carry =
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 3);

	rx_stats->rx_fragments_counter =
	       	phoenix_read_reg(priv->mmio,RX_FRAGMENTS_COUNTER);
	rx_stats->rx_fragments_counter_carry = 
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 2);

	rx_stats->rx_jabber_counter =
	       	phoenix_read_reg(priv->mmio,RX_JABBER_COUNTER);
	rx_stats->rx_jabber_counter_carry =
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 1);

	rx_stats->rx_drop_pkt_counter =
	       	phoenix_read_reg(priv->mmio,RX_DROP_PACKET_COUNTER);
	rx_stats->rx_drop_pkt_counter_carry =
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 0);

	return;
}

void phnx_get_mac_tx_rx_stats(struct driver_data *priv, mac_tx_rx_statistics  *tx_rx_stats)
{
	tx_rx_stats->tx_rx_64_byte_frame =
	       	phoenix_read_reg(priv->mmio,TX_RX_64_BYTE_FRAME);
	tx_rx_stats->tx_rx_64_byte_frame_carry =
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 31);

	tx_rx_stats->tx_rx_65_127_byte_frame =
	       	phoenix_read_reg(priv->mmio,TX_RX_64_127_BYTE_FRAME);
	tx_rx_stats->tx_rx_65_127_byte_frame_carry = 
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 30);

	tx_rx_stats->tx_rx_128_255_byte_frame =
	       	phoenix_read_reg(priv->mmio,TX_RX_128_255_BYTE_FRAME);
	tx_rx_stats->tx_rx_128_255_byte_frame_carry =
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 29);

	tx_rx_stats->tx_rx_256_511_byte_frame =
	       	phoenix_read_reg(priv->mmio,TX_RX_256_511_BYTE_FRAME);
	tx_rx_stats->tx_rx_256_511_byte_frame_carry =
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 28);

	tx_rx_stats->tx_rx_512_1023_byte_frame =
	       	phoenix_read_reg(priv->mmio,TX_RX_512_1023_BYTE_FRAME);
	tx_rx_stats->tx_rx_512_1023_byte_frame_carry = 
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 27);

	tx_rx_stats->tx_rx_1024_1518_byte_frame = 
	       	phoenix_read_reg(priv->mmio,TX_RX_1024_1518_BYTE_FRAME);
	tx_rx_stats->tx_rx_1024_1518_byte_frame_carry = 
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 26);

	tx_rx_stats->tx_rx_1519_1522_byte_frame =
	       	phoenix_read_reg(priv->mmio,TX_RX_1519_1522_VLAN_BYTE_FRAME);
	tx_rx_stats->tx_rx_1519_1522_byte_frame_carry =
	       	phnx_get_reg_bit(priv->mmio,CARRY_REG_1, 25);


	return;
}



void  phnx_mac_update_tx_stats(phnx_stats *portStats, mac_tx_statistics   *tx_stats)
{
	portStats->tx_stats.tx_bytes += tx_stats->tx_bytes;
	portStats->tx_stats.tx_pkts += tx_stats->tx_pkts;
	portStats->tx_stats.tx_multicast_pkts += tx_stats->tx_multicast_pkts ;
	portStats->tx_stats.tx_broadcast_pkts += tx_stats->tx_broadcast_pkts;
	portStats->tx_stats.tx_pause_control_frame_counter +=
	       	tx_stats->tx_pause_control_frame_counter;
	portStats->tx_stats.tx_deferral_pkt_counter +=
	       	tx_stats->tx_deferral_pkt_counter;
	portStats->tx_stats.tx_excessive_deferral_pkt_counter +=
	       	tx_stats->tx_excessive_deferral_pkt_counter;
	portStats->tx_stats.tx_single_collision_pkt_counter +=
	       	tx_stats->tx_single_collision_pkt_counter;
	portStats->tx_stats.tx_multi_collision_pkt_counter +=
	       	tx_stats->tx_multi_collision_pkt_counter;
	portStats->tx_stats.tx_late_collision_pkt_counter +=
	       	tx_stats->tx_late_collision_pkt_counter;
	portStats->tx_stats.tx_excessive_collision_pkt_counter +=
	       	tx_stats->tx_excessive_collision_pkt_counter;
	portStats->tx_stats.tx_total_collision_counter +=
	       	tx_stats->tx_total_collision_counter;
	portStats->tx_stats.tx_pause_frame_honered_counter +=
	       	tx_stats->tx_pause_frame_honered_counter;
	portStats->tx_stats.tx_drop_frame_counter +=
	       	tx_stats->tx_drop_frame_counter;
	portStats->tx_stats.tx_jabber_frame_counter +=
	       	tx_stats->tx_jabber_frame_counter;
	portStats->tx_stats.tx_fcs_error_counter +=
	       	tx_stats->tx_fcs_error_counter;
	portStats->tx_stats.tx_control_frame_counter +=
	       	tx_stats->tx_control_frame_counter;
	portStats->tx_stats.tx_oversize_frame_counter +=
	       	tx_stats->tx_oversize_frame_counter;
	portStats->tx_stats.tx_undersize_frame_counter +=
	       	tx_stats->tx_undersize_frame_counter;
	portStats->tx_stats.tx_fragment_frame_counter +=
	       	tx_stats->tx_fragment_frame_counter;

	return;
}

void  phnx_mac_update_rx_stats(phnx_stats *portStats,  mac_rx_statistics   *rx_stats)
{
	portStats->rx_stats.rx_bytes += rx_stats->rx_bytes;
	portStats->rx_stats.rx_pkts += rx_stats->rx_pkts;
	portStats->rx_stats.rx_fcs_error_counter += rx_stats->rx_fcs_error_counter;
	portStats->rx_stats.rx_multicast_pkt_counter += rx_stats->rx_multicast_pkt_counter;
	portStats->rx_stats.rx_broadcast_pkt_counter += rx_stats->rx_broadcast_pkt_counter;
	portStats->rx_stats.rx_control_frame_pkt_counter += rx_stats->rx_control_frame_pkt_counter;
	portStats->rx_stats.rx_pause_frame_pkt_counter += rx_stats->rx_pause_frame_pkt_counter;
	portStats->rx_stats.rx_unknown_opcode_pkt_counter += rx_stats->rx_unknown_opcode_pkt_counter;
	portStats->rx_stats.rx_alignment_error_counter += rx_stats->rx_alignment_error_counter;
	portStats->rx_stats.rx_frame_length_error_counter += rx_stats->rx_frame_length_error_counter;
	portStats->rx_stats.rx_code_error_counter += rx_stats->rx_code_error_counter;
	portStats->rx_stats.rx_carrier_sense_error_counter += rx_stats->rx_carrier_sense_error_counter;
	portStats->rx_stats.rx_undersize_pkt_counter += rx_stats->rx_undersize_pkt_counter;
	portStats->rx_stats.rx_oversize_pkt_counter += rx_stats->rx_oversize_pkt_counter;
	portStats->rx_stats.rx_fragments_counter += rx_stats->rx_fragments_counter;
	portStats->rx_stats.rx_jabber_counter += rx_stats->rx_jabber_counter;
	portStats->rx_stats.rx_drop_pkt_counter += rx_stats->rx_drop_pkt_counter;
	return;
}

void  phnx_mac_update_tx_rx_stats(phnx_stats *portStats,  mac_tx_rx_statistics   *tx_rx_stats)
{
	portStats->tx_rx_stats.tx_rx_64_byte_frame +=
	       	tx_rx_stats->tx_rx_64_byte_frame;
	portStats->tx_rx_stats.tx_rx_65_127_byte_frame +=
	       	tx_rx_stats->tx_rx_65_127_byte_frame;
	portStats->tx_rx_stats.tx_rx_128_255_byte_frame +=
	       	tx_rx_stats->tx_rx_128_255_byte_frame;
	portStats->tx_rx_stats.tx_rx_256_511_byte_frame +=
	       	tx_rx_stats->tx_rx_256_511_byte_frame;
	portStats->tx_rx_stats.tx_rx_512_1023_byte_frame +=
	       	tx_rx_stats->tx_rx_512_1023_byte_frame;
	portStats->tx_rx_stats.tx_rx_1024_1518_byte_frame +=
	       	tx_rx_stats->tx_rx_1024_1518_byte_frame;
	portStats->tx_rx_stats.tx_rx_1519_1522_byte_frame +=
	       	tx_rx_stats->tx_rx_1519_1522_byte_frame;
	return;
}

void phnx_collect_stats(unsigned int gmac_id)
{
	struct driver_data*        priv  ;
	mac_tx_statistics   tx_stats;
	mac_rx_statistics   rx_stats;
	mac_tx_rx_statistics  tx_rx_stats;
	struct net_device *dev  = 0;

	if(!phnx_mac_devices[gmac_id].init_done) return;
	dev = dev_mac[gmac_id];
	if(dev==NULL) return;

	priv  = netdev_priv(dev);
	if(priv==NULL) return;


	phnx_get_mac_tx_stats(priv, &tx_stats);
	phnx_mac_update_tx_stats(&phnx_gmac_stats[gmac_id],&tx_stats);

	phnx_get_mac_rx_stats(priv, &rx_stats);
	phnx_mac_update_rx_stats(&phnx_gmac_stats[gmac_id],&rx_stats);


	phnx_get_mac_tx_rx_stats(priv, &tx_rx_stats);
	phnx_mac_update_tx_rx_stats(&phnx_gmac_stats[gmac_id],&tx_rx_stats);

	return;
}

void phnx_reset_stats(unsigned int gmac_id )
{
	phnx_stats   *portStats;

	if(!phnx_mac_devices[gmac_id].init_done) return;
	portStats = &phnx_gmac_stats[gmac_id];

	portStats->tx_stats.tx_bytes = 0;
	portStats->tx_stats.tx_pkts = 0;
	portStats->tx_stats.tx_multicast_pkts = 0 ;
	portStats->tx_stats.tx_broadcast_pkts = 0;
	portStats->tx_stats.tx_pause_control_frame_counter = 0;
	portStats->tx_stats.tx_deferral_pkt_counter = 0;
	portStats->tx_stats.tx_excessive_deferral_pkt_counter = 0;
	portStats->tx_stats.tx_single_collision_pkt_counter = 0;
	portStats->tx_stats.tx_multi_collision_pkt_counter = 0;
	portStats->tx_stats.tx_late_collision_pkt_counter = 0;
	portStats->tx_stats.tx_excessive_collision_pkt_counter = 0;
	portStats->tx_stats.tx_total_collision_counter = 0;
	portStats->tx_stats.tx_pause_frame_honered_counter = 0;
	portStats->tx_stats.tx_drop_frame_counter = 0;
	portStats->tx_stats.tx_jabber_frame_counter = 0;
	portStats->tx_stats.tx_fcs_error_counter = 0;
	portStats->tx_stats.tx_control_frame_counter = 0;
	portStats->tx_stats.tx_oversize_frame_counter = 0;
	portStats->tx_stats.tx_undersize_frame_counter = 0;
	portStats->tx_stats.tx_fragment_frame_counter = 0;

	portStats->rx_stats.rx_bytes = 0;
	portStats->rx_stats.rx_pkts = 0;
	portStats->rx_stats.rx_fcs_error_counter = 0;
	portStats->rx_stats.rx_multicast_pkt_counter = 0;
	portStats->rx_stats.rx_broadcast_pkt_counter = 0;
	portStats->rx_stats.rx_control_frame_pkt_counter = 0;
	portStats->rx_stats.rx_pause_frame_pkt_counter = 0;
	portStats->rx_stats.rx_unknown_opcode_pkt_counter = 0;
	portStats->rx_stats.rx_alignment_error_counter = 0;
	portStats->rx_stats.rx_frame_length_error_counter = 0;
	portStats->rx_stats.rx_code_error_counter = 0;
	portStats->rx_stats.rx_carrier_sense_error_counter = 0;
	portStats->rx_stats.rx_undersize_pkt_counter = 0;
	portStats->rx_stats.rx_oversize_pkt_counter = 0;
	portStats->rx_stats.rx_fragments_counter = 0;
	portStats->rx_stats.rx_jabber_counter = 0;
	portStats->rx_stats.rx_drop_pkt_counter = 0;

	portStats->tx_rx_stats.tx_rx_64_byte_frame = 0;
	portStats->tx_rx_stats.tx_rx_65_127_byte_frame = 0;
	portStats->tx_rx_stats.tx_rx_128_255_byte_frame = 0;
	portStats->tx_rx_stats.tx_rx_256_511_byte_frame = 0;
	portStats->tx_rx_stats.tx_rx_512_1023_byte_frame = 0;
	portStats->tx_rx_stats.tx_rx_1024_1518_byte_frame = 0;
	portStats->tx_rx_stats.tx_rx_1519_1522_byte_frame = 0;

	phnx_clear_mac_stats(gmac_id);
	return;
}

void phnx_show_tsv(void)
{
	int id;
	phnx_stats 			portStats[4];
	phnx_tx_stats   tx_stats[4];

	for(id=0; id<4; id++)
	{
		portStats[id] = phnx_gmac_stats[id];
		tx_stats[id] = portStats[id].tx_stats;
	}

	printk("\n                  gmac0          gmac1          gmac2          gmac3  ");

	printk("\nTx stats:");
	printk("\nBytes Tx          %-15llu %-15llu %-15llu %-15llu",
		tx_stats[0].tx_bytes,tx_stats[1].tx_bytes,
		tx_stats[2].tx_bytes,tx_stats[3].tx_bytes);

	printk("\nPackets Tx        %-15llu %-15llu %-15llu %-15llu",
		tx_stats[0].tx_pkts,tx_stats[1].tx_pkts,
		tx_stats[2].tx_pkts,tx_stats[3].tx_pkts);

	printk("\nMulticast Tx      %-15llu %-15llu %-15llu %-15llu",
		tx_stats[0].tx_multicast_pkts,tx_stats[1].tx_multicast_pkts,
		tx_stats[2].tx_multicast_pkts,tx_stats[3].tx_multicast_pkts);

	printk("\nBroadcast Tx      %-15llu %-15llu %-15llu %-15llu",
		tx_stats[0].tx_broadcast_pkts,tx_stats[1].tx_broadcast_pkts,
		tx_stats[2].tx_broadcast_pkts,tx_stats[3].tx_broadcast_pkts);

	printk("\nPause Tx          %-15llu %-15llu %-15llu %-15llu",
		tx_stats[0].tx_pause_control_frame_counter,
		tx_stats[1].tx_pause_control_frame_counter,
		tx_stats[2].tx_pause_control_frame_counter,
		tx_stats[3].tx_pause_control_frame_counter);

	printk("\nDeferral Tx       %-15llu %-15llu %-15llu %-15llu",
		tx_stats[0].tx_deferral_pkt_counter,
		tx_stats[1].tx_deferral_pkt_counter,
		tx_stats[2].tx_deferral_pkt_counter,
		tx_stats[3].tx_deferral_pkt_counter);

	printk("\nExces Def Tx      %-15llu %-15llu %-15llu %-15llu",
		tx_stats[0].tx_excessive_deferral_pkt_counter,
		tx_stats[1].tx_excessive_deferral_pkt_counter,
		tx_stats[2].tx_excessive_deferral_pkt_counter,
		tx_stats[3].tx_excessive_deferral_pkt_counter);


	printk("\nSingle Coll Tx    %-15llu %-15llu %-15llu %-15llu",
		tx_stats[0].tx_single_collision_pkt_counter,
		tx_stats[1].tx_single_collision_pkt_counter,
		tx_stats[2].tx_single_collision_pkt_counter,
		tx_stats[3].tx_single_collision_pkt_counter);

	printk("\nMultiple Coll Tx  %-15llu %-15llu %-15llu %-15llu",
		tx_stats[0].tx_multi_collision_pkt_counter,
		tx_stats[1].tx_multi_collision_pkt_counter,
		tx_stats[2].tx_multi_collision_pkt_counter,
		tx_stats[3].tx_multi_collision_pkt_counter);

	printk("\nLate Coll Tx      %-15llu %-15llu %-15llu %-15llu",
		tx_stats[0].tx_late_collision_pkt_counter,
		tx_stats[1].tx_late_collision_pkt_counter,
		tx_stats[2].tx_late_collision_pkt_counter,
		tx_stats[3].tx_late_collision_pkt_counter);

	printk("\nExcessive Coll Tx %-15llu %-15llu %-15llu %-15llu",
		tx_stats[0].tx_excessive_collision_pkt_counter,
		tx_stats[1].tx_excessive_collision_pkt_counter,
		tx_stats[2].tx_excessive_collision_pkt_counter,
		tx_stats[3].tx_excessive_collision_pkt_counter);

	printk("\nTotal Coll Tx     %-15llu %-15llu %-15llu %-15llu",
		tx_stats[0].tx_total_collision_counter,
		tx_stats[1].tx_total_collision_counter,
		tx_stats[2].tx_total_collision_counter,
		tx_stats[3].tx_total_collision_counter);

	printk("\nPause Honored Tx  %-15llu %-15llu %-15llu %-15llu",
		tx_stats[0].tx_pause_frame_honered_counter,
		tx_stats[1].tx_pause_frame_honered_counter,
		tx_stats[2].tx_pause_frame_honered_counter,
		tx_stats[3].tx_pause_frame_honered_counter);

	printk("\nDrop frames Tx    %-15llu %-15llu %-15llu %-15llu",
		tx_stats[0].tx_drop_frame_counter,
		tx_stats[1].tx_drop_frame_counter,
		tx_stats[2].tx_drop_frame_counter,
		tx_stats[3].tx_drop_frame_counter);

	printk("\nJabber Frames Tx  %-15llu %-15llu %-15llu %-15llu",
		tx_stats[0].tx_jabber_frame_counter,
		tx_stats[1].tx_jabber_frame_counter,
		tx_stats[2].tx_jabber_frame_counter,
		tx_stats[3].tx_jabber_frame_counter);

	printk("\nFcs error Tx      %-15llu %-15llu %-15llu %-15llu",
	tx_stats[0].tx_fcs_error_counter,
	tx_stats[1].tx_fcs_error_counter,
	tx_stats[2].tx_fcs_error_counter,
	tx_stats[3].tx_fcs_error_counter);

	printk("\nControl frames Tx %-15llu %-15llu %-15llu %-15llu",
			tx_stats[0].tx_control_frame_counter,
			tx_stats[1].tx_control_frame_counter,
			tx_stats[2].tx_control_frame_counter,
			tx_stats[3].tx_control_frame_counter);

	printk("\nOver size Tx      %-15llu %-15llu %-15llu %-15llu",
			tx_stats[0].tx_oversize_frame_counter,
			tx_stats[1].tx_oversize_frame_counter,
			tx_stats[2].tx_oversize_frame_counter,
			tx_stats[3].tx_oversize_frame_counter);

	printk("\nUnder size Tx     %-15llu %-15llu %-15llu %-15llu",
			tx_stats[0].tx_undersize_frame_counter,
			tx_stats[1].tx_undersize_frame_counter,
			tx_stats[2].tx_undersize_frame_counter,
			tx_stats[3].tx_undersize_frame_counter);

	printk("\nFragment Tx       %-15llu %-15llu %-15llu %-15llu",
			tx_stats[0].tx_fragment_frame_counter,
			tx_stats[1].tx_fragment_frame_counter,
			tx_stats[2].tx_fragment_frame_counter,
			tx_stats[3].tx_fragment_frame_counter);

	printk("\n");
	return;
}


void phnx_show_rsv(void)
{
	int id;
	phnx_stats portStats[4];
	phnx_rx_stats   rx_stats[4];
	for(id=0; id<4; id++)
	{
		portStats[id] = phnx_gmac_stats[id];
		rx_stats[id] = portStats[id].rx_stats;
	}


	printk("\nRx stats:");
	printk("\nBytes Rx          %-15llu %-15llu %-15llu %-15llu",
		rx_stats[0].rx_bytes,rx_stats[1].rx_bytes,
		rx_stats[2].rx_bytes,rx_stats[3].rx_bytes);

	printk("\nPackets Rx        %-15llu %-15llu %-15llu %-15llu",
		rx_stats[0].rx_pkts,rx_stats[1].rx_pkts,
		rx_stats[2].rx_pkts,rx_stats[3].rx_pkts);

	printk("\nFcs error Rx      %-15llu %-15llu %-15llu %-15llu",
		rx_stats[0].rx_fcs_error_counter,
		rx_stats[1].rx_fcs_error_counter,
		rx_stats[2].rx_fcs_error_counter,
		rx_stats[3].rx_fcs_error_counter);

	printk("\nMulticast Rx      %-15llu %-15llu %-15llu %-15llu",
		rx_stats[0].rx_multicast_pkt_counter,
		rx_stats[1].rx_multicast_pkt_counter,
		rx_stats[2].rx_multicast_pkt_counter,
		rx_stats[3].rx_multicast_pkt_counter);

	printk("\nBroadcast Rx      %-15llu %-15llu %-15llu %-15llu",
		rx_stats[0].rx_broadcast_pkt_counter,
		rx_stats[1].rx_broadcast_pkt_counter,
		rx_stats[2].rx_broadcast_pkt_counter,
		rx_stats[3].rx_broadcast_pkt_counter);

	printk("\nControl  Rx       %-15llu %-15llu %-15llu %-15llu",
		rx_stats[0].rx_control_frame_pkt_counter,
		rx_stats[1].rx_control_frame_pkt_counter,
		rx_stats[2].rx_control_frame_pkt_counter,
		rx_stats[3].rx_control_frame_pkt_counter);

	printk("\nPause Rx          %-15llu %-15llu %-15llu %-15llu",
		rx_stats[0].rx_pause_frame_pkt_counter,
		rx_stats[1].rx_pause_frame_pkt_counter,
		rx_stats[2].rx_pause_frame_pkt_counter,
		rx_stats[3].rx_pause_frame_pkt_counter);

	printk("\nUnknown opcode Rx %-15llu %-15llu %-15llu %-15llu",
		rx_stats[0].rx_unknown_opcode_pkt_counter,
		rx_stats[1].rx_unknown_opcode_pkt_counter,
		rx_stats[2].rx_unknown_opcode_pkt_counter,
		rx_stats[3].rx_unknown_opcode_pkt_counter);

	printk("\nAlignment error Rx%-15llu %-15llu %-15llu %-15llu",
		rx_stats[0].rx_alignment_error_counter,
		rx_stats[1].rx_alignment_error_counter,
		rx_stats[2].rx_alignment_error_counter,
		rx_stats[3].rx_alignment_error_counter);

	printk("\nLength error Rx   %-15llu %-15llu %-15llu %-15llu",
		rx_stats[0].rx_frame_length_error_counter,
		rx_stats[1].rx_frame_length_error_counter,
		rx_stats[2].rx_frame_length_error_counter,
		rx_stats[3].rx_frame_length_error_counter);

	printk("\nCode error Rx     %-15llu %-15llu %-15llu %-15llu",
		rx_stats[0].rx_code_error_counter,
		rx_stats[1].rx_code_error_counter,
		rx_stats[2].rx_code_error_counter,
		rx_stats[3].rx_code_error_counter);

	printk("\nCarrierSenseError %-15llu %-15llu %-15llu %-15llu",
		rx_stats[0].rx_carrier_sense_error_counter,
		rx_stats[1].rx_carrier_sense_error_counter,
		rx_stats[2].rx_carrier_sense_error_counter,
		rx_stats[3].rx_carrier_sense_error_counter);

	printk("\nUnder size Rx     %-15llu %-15llu %-15llu %-15llu",
		rx_stats[0].rx_undersize_pkt_counter,
		rx_stats[1].rx_undersize_pkt_counter,
		rx_stats[2].rx_undersize_pkt_counter,
		rx_stats[3].rx_undersize_pkt_counter);

	printk("\nOver size Rx      %-15llu %-15llu %-15llu %-15llu",
		rx_stats[0].rx_oversize_pkt_counter,
		rx_stats[1].rx_oversize_pkt_counter,
		rx_stats[2].rx_oversize_pkt_counter,
		rx_stats[3].rx_oversize_pkt_counter);

	printk("\nFragments Rx      %-15llu %-15llu %-15llu %-15llu",
		rx_stats[0].rx_fragments_counter,
		rx_stats[1].rx_fragments_counter,
		rx_stats[2].rx_fragments_counter,
		rx_stats[3].rx_fragments_counter);

	printk("\nJabber frames     %-15llu %-15llu %-15llu %-15llu",
		rx_stats[0].rx_jabber_counter,
		rx_stats[1].rx_jabber_counter,
		rx_stats[2].rx_jabber_counter,
		rx_stats[3].rx_jabber_counter);

	printk("\nDropped Pkts Rx   %-15llu %-15llu %-15llu %-15llu",
		rx_stats[0].rx_drop_pkt_counter,
		rx_stats[1].rx_drop_pkt_counter,
		rx_stats[2].rx_drop_pkt_counter,
		rx_stats[3].rx_drop_pkt_counter);

	printk("\n");
	return;
}

void phnx_show_tx_rx_counter(void)
{
	int id;
	phnx_tx_rx_stats tx_rx_stats[4];

	for(id=0; id<4; id++)
	{
		tx_rx_stats[id] = phnx_gmac_stats[id].tx_rx_stats;
	}

	printk("\nTotal TX and Rx packets of bytes in range:");
	printk("\n64 bytes          %-15llu %-15llu %-15llu %-15llu",
		tx_rx_stats[0].tx_rx_64_byte_frame,
		tx_rx_stats[1].tx_rx_64_byte_frame,
		tx_rx_stats[2].tx_rx_64_byte_frame,
		tx_rx_stats[3].tx_rx_64_byte_frame);

	printk("\n65-127 bytes      %-15llu %-15llu %-15llu %-15llu",
		tx_rx_stats[0].tx_rx_65_127_byte_frame,
		tx_rx_stats[1].tx_rx_65_127_byte_frame,
		tx_rx_stats[2].tx_rx_65_127_byte_frame,
		tx_rx_stats[3].tx_rx_65_127_byte_frame);

	printk("\n128-255 bytes     %-15llu %-15llu %-15llu %-15llu",
		tx_rx_stats[0].tx_rx_128_255_byte_frame,
		tx_rx_stats[1].tx_rx_128_255_byte_frame,
		tx_rx_stats[2].tx_rx_128_255_byte_frame,
		tx_rx_stats[3].tx_rx_128_255_byte_frame);

	printk("\n256-511 bytes     %-15llu %-15llu %-15llu %-15llu",
		tx_rx_stats[0].tx_rx_256_511_byte_frame,
		tx_rx_stats[1].tx_rx_256_511_byte_frame,
		tx_rx_stats[2].tx_rx_256_511_byte_frame,
		tx_rx_stats[3].tx_rx_256_511_byte_frame);

	printk("\n512-1023 bytes    %-15llu %-15llu %-15llu %-15llu",
		tx_rx_stats[0].tx_rx_512_1023_byte_frame,
		tx_rx_stats[1].tx_rx_512_1023_byte_frame,
		tx_rx_stats[2].tx_rx_512_1023_byte_frame,
		tx_rx_stats[3].tx_rx_512_1023_byte_frame);

	printk("\n1024-1518 bytes   %-15llu %-15llu %-15llu %-15llu",
		tx_rx_stats[0].tx_rx_1024_1518_byte_frame,
		tx_rx_stats[1].tx_rx_1024_1518_byte_frame,
		tx_rx_stats[2].tx_rx_1024_1518_byte_frame,
		tx_rx_stats[3].tx_rx_1024_1518_byte_frame);

	printk("\n1519-1522 VLAN    %-15llu %-15llu %-15llu %-15llu",
		tx_rx_stats[0].tx_rx_1519_1522_byte_frame,
		tx_rx_stats[1].tx_rx_1519_1522_byte_frame,
		tx_rx_stats[2].tx_rx_1519_1522_byte_frame,
		tx_rx_stats[3].tx_rx_1519_1522_byte_frame);

	printk("\n");
	return;
}





/*******************************************************************************
*
* xlrShowGmacStats - Display gmac stats for all four gmacs
*
* RETURNS: void
*/
void phnx_show_gmac_stats(void)
{
	phnx_show_tsv();
	phnx_show_rsv();
	phnx_show_tx_rx_counter();

 	return;
}

/*******************************************************************************
*
* xlrResetGmacStats - Reset the s/w and h/w gmac statistics counters for all
*                     four gmacs
*
* RETURNS: void
*/
void phnx_reset_gmac_stats(void)
{
	unsigned int gmac_id;

	for(gmac_id =0; gmac_id<4; gmac_id++)
		phnx_reset_stats(gmac_id);
  	return;
}

