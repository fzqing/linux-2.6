#ifndef __ASM_ARM_MCASP_H__
#define __ASM_ARM_MCASP_H__

/*
 * mcasp.h - Definitions for the interface to MultiChannel Audio Serial Port
 * 		(McASP).
 *
 * Copyright (C) 2007  Texas Instruments, India
 * Author:Nirmal Pandey <n-pandey@ti.com>,
 *        Suresh Rajashekara <suresh.r@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

/* Counts */
#define MAX_SERIALIZER_COUNT           16	/* Serializer Count */
#define MAX_DIT_LEFT_CHAN_STAT_COUNT   6	/* Left (even TDM slot) Channel
						 * status register count */
#define MAX_DIT_RIGHT_CHAN_STAT_COUNT  6	/* Right (odd TDM slot) Channel
						 * status register count */
#define MAX_LEFT_USR_DATA_REG_FILE     6	/* Left (even TDM slot) user
						 * data register count */
#define MAX_RIGHT_USR_DATA_REG_FILE    6	/* Right (odd TDM slot) user
						 * data register count */
#define TX_SUPPORT 0x01
#define RX_SUPPORT 0x02

/* McASP Register Overlay Structure */
typedef struct mcasp_registers {
	volatile u32 pid;	/* VBUS Std. Peripheral ID Register */
	volatile u32 pwremumgt;	/* Power Down and Emulation Management
				 * Register */
	volatile u32 rsvd_08_0c[2];	/* Reserved */
	volatile u32 pfunc;	/* Pin Function Register */
	volatile u32 pdir;	/* Pin Direction Register */
	volatile u32 pdout;	/* Pin Data Out Register */
	volatile u32 pdset;	/* Writes Affect: Pin Data Set Register
				 * (Alternate Write Address PDOUT)*/
	/* PDIN is also an alias for PDSET */
	volatile u32 pdclr;	/* Pin Data Clear Register
				 * (Alternate Write Address PDOUT */
	volatile u32 rsvd_24_2c[3];	/* Reserved */
	volatile u32 tlgc;	/* IODFT Test Logic Global Control Register */
	volatile u32 tlmr;	/* IODFT Test Logic MISR Result Register */
	volatile u32 rsvd_38_40[3];	/* Reserved */
	volatile u32 gblctl;	/* Global Control Register */
	volatile u32 amute;	/* Mute Control Register */
	volatile u32 lbctl;	/* Loop-Back Test Control Register */
	volatile u32 txditctl;	/* Transmit DIT Mode Control Register */
	volatile u32 rsvd_54_5c[3];	/* Reserved */
	volatile u32 gblctlr;	/* Alias of GBLCTL containing only Receiver
				 * Reset bits - allows transmit to be reset
				 * independently from Receive */
	volatile u32 rxmask;	/* Receiver Format Unit Bit Mask Register */
	volatile u32 rxfmt;	/* Receive Bitstream Format Register */
	volatile u32 rxfmctl;	/* Receive Frame Sync Control Register */
	volatile u32 aclkrctl;	/* Receive Clock Control Register */
	volatile u32 ahclkrctl;	/* High Frequency Receive Clock Control Register
				 * */
	volatile u32 rxtdm;	/* Receive TDM Slot 0-31 Register */
	volatile u32 evtctlr;	/* Receiver Interrupt Control */
	volatile u32 rxstat;	/* Status Register - Receiver */
	volatile u32 rxtdmslot;	/* Current Receive TDM Slot */
	volatile u32 rxclkchk;	/* Receiver Clock Check Control Register */
	volatile u32 revtctl;	/* Receiver DMA Event Control */
	volatile u32 rsvd_90_9c[4];	/* Reserved */
	volatile u32 gblctlx;	/* Alias of GBLCTL containing only Transmit
				 * Reset bits - allows transmit to be reset
				 * independently from Receive*/
	volatile u32 txmask;	/* Transmit Format Unit Bit Mask Register */
	volatile u32 txfmt;	/* Transmit Bitstream Format Register */
	volatile u32 txfmctl;	/* Transmit Frame Sync Control Register */
	volatile u32 aclkxctl;	/* Transmit Clock Control Register */
	volatile u32 ahclkxctl;	/* High Frequency Transmit Clock Control
				 * Register*/
	volatile u32 txtdm;	/* Transmit TDM Slot 0-31 Register */
	volatile u32 evtctlx;	/* Transmitter Interrupt Control */
	volatile u32 txstat;	/* Status Register - Transmitter */
	volatile u32 txtdmslot;	/* Current Transmit TDM Slot */
	volatile u32 txclkchk;	/* Transmit Clock Check Control Register */
	volatile u32 xevtctl;	/* Transmitter DMA Control */
	volatile u32 rsvd_d0_fc[12];	/* Reserved */
	/* Left (even TDM Slot) Channel Status Register File*/
	volatile u32 ditcsra[MAX_DIT_LEFT_CHAN_STAT_COUNT];
	/* Right (odd TDM slot) Channel Status RegisterFile*/
	volatile u32 ditcsrb[MAX_DIT_RIGHT_CHAN_STAT_COUNT];
	/* Left (even TDM slot) User Data Register File */
	volatile u32 ditudra[MAX_LEFT_USR_DATA_REG_FILE];
	/* Right (odd TDM Slot) User Data Register File */
	volatile u32 ditudrb[MAX_RIGHT_USR_DATA_REG_FILE];
	volatile u32 rsvd_160_17c[8];	/* Reserved */
	volatile u32 xrsrctl[MAX_SERIALIZER_COUNT];	/* Serializer n Control
							 * Register */
	volatile u32 rsvd_1c0_1fc[16];	/* Reserved */
	volatile u32 txbuf[MAX_SERIALIZER_COUNT];	/* Transmit Buffer for
							 * Serializer n */
	volatile u32 rsvd_240_27c[16];	/* Reserved */
	volatile u32 rxbuf[MAX_SERIALIZER_COUNT];	/* Receive Buffer for
							 * Serializer n */
} mcasp_registers_t;

typedef struct __mcasp_info {
	    /* Set -1 to indicate an unsupported or uninitialized feature */
    s8 id;
    u32 reg_base;
    u32 tx_data_port;
    u32 rx_data_port;
    s8 tx_dma_evt;
    s8 rx_dma_evt;
    s8 tx_irq;
    s8 rx_irq;
    s8 serializer_count;
    s8 left_dit_channel_count;
    s8 right_dit_channel_count;
    const char *lpsc;		/* Initialize this to NULL if there is no LPSC
				   domain to be initialized. Sometimes this init
				   of the power domain could be done by some
				   other portion of the BSP (eg: U-boot) or
				   sometime the module might not need any LPSC
				   to be initializd. */
    s8 flags;
} mcasp_info_t;

typedef struct __mcasp_unit {
    struct list_head list;
    struct __mcasp_info info;
    struct mcasp_registers config;
    struct clk *clock;
    u32 sample_rate;
    atomic_t init;
    atomic_t rx_in_use;
    atomic_t tx_in_use;
} mcasp_unit_t;

typedef struct __mcasp_device {
	u8 count;
	struct list_head *mcasp_list;	/* Points to the list of mcasp's */
} mcasp_device_t;

/* Register Bits */
#define BIT(n) (1<<n)

/* PWREMUMGT Power Down and Emulation Management Register Bits */
#define FREE         BIT(0)
#define SOFT         BIT(1)

/* Pin Function / GPIO Enable Register Bits */
#define AXR(n)       (1<<n)
#define PFUNC_AMUTE  BIT(25)
#define ACLKX        BIT(26)
#define AHCLKX       BIT(27)
#define AFSX         BIT(28)
#define ACLKR        BIT(29)
#define AHCLKR       BIT(30)
#define AFSR         BIT(31)

/* Pin Direction Register Bits */
#define AXR(n)       (1<<n)
#define PDIR_AMUTE   BIT(25)
#define ACLKX        BIT(26)
#define AHCLKX       BIT(27)
#define AFSX         BIT(28)
#define ACLKR        BIT(29)
#define AHCLKR       BIT(30)
#define AFSR         BIT(31)

/* Transmit DIT Control Register Bits */
#define DITEN        BIT(0)	/* Transmit DIT mode enable/disable */
#define VA           BIT(2)
#define VB           BIT(3)

/* Transmit Bitstream Format Register Bits */
#define TXROT(val)   (val)
#define TXSEL        BIT(3)
#define TXSSZ(val)   (val<<4)
#define TXPBIT(val)  (val<<8)
#define TXPAD(val)   (val<<13)
#define TXORD        BIT(15)
#define FSXDLY(val)  (val<<16)

/* Receive Bitstream Format Register Bits */
#define RXROT(val)   (val)
#define RXSEL        BIT(3)
#define RXSSZ(val)   (val<<4)
#define RXPBIT(val)  (val<<8)
#define RXPAD(val)   (val<<13)
#define RXORD        BIT(15)
#define FSRDLY(val)  (val<<16)

/* Transmit Frame Control Register Bits */
#define FSXPOL       BIT(0)
#define AFSXE        BIT(1)
#define FSXDUR       BIT(4)
#define FSXMOD(val)  (val<<7)

/* Receive Frame Control Register Bits */
#define FSRPOL       BIT(0)
#define AFSRE        BIT(1)
#define FSRDUR       BIT(4)
#define FSRMOD(val)  (val<<7)

/* Transmit Clock Control Register Bits */
#define ACLKXDIV(val) (val)
#define ACLKXE       BIT(5)
#define TX_ASYNC     BIT(6)
#define ACLKXPOL     BIT(7)

/* Receive Clock Control Register Bits */
#define ACLKRDIV(val) (val)
#define ACLKRE       BIT(5)
#define ACLKRPOL     BIT(7)

/* High Frequency Transmit Clock Control Register Bits */
#define AHCLKXDIV(val) (val)
#define AHCLKXPOL    BIT(14)
#define AHCLKXE      BIT(15)

/* High Frequency Receive Clock Control Register Bits */
#define AHCLKRDIV(val) (val)
#define AHCLKRPOL    BIT(14)
#define AHCLKRE      BIT(15)

/* Serializer Control Register Bits */
#define MODE(val)    (val)
#define DISMOD(val)  (val<<2)
#define TXSTATE      BIT(4)
#define RXSTATE      BIT(5)

/* Loop Back Control Register Bits */
#define LBEN         BIT(0)
#define LBORD        BIT(1)
#define LBGENMODE(val) (val<<2)

/* Transmit TDM Slot Register configuration */
#define TXTDMS(n)    (1<<n)

/* Receive TDM Slot Register configuration */
#define RXTDMS(n)    (1<<n)

/* Global Control Register Bits */
#define RXCLKRST     BIT(0)	/* Receiver Clock Divider Reset */
#define RXHCLKRST    BIT(1)	/* Receiver High Frequency Clock Divider */
#define RXSERCLR     BIT(2)	/* Receiver Serializer Clear */
#define RXSMRST      BIT(3)	/* Receiver State Machine Reset */
#define RXFSRST      BIT(4)	/* Frame Sync Generator Reset */
#define TXCLKRST     BIT(8)	/* Transmitter Clock Divider Reset */
#define TXHCLKRST    BIT(9)	/* Transmitter High Frequency Clock Divider
				 * and Transmit Bad Clock Detect /32 Counter
				 * Reset */
#define TXSERCLR     BIT(10)	/* Transmit Serializer Clear */
#define TXSMRST      BIT(11)	/* Transmitter State Machine Reset */
#define TXFSRST      BIT(12)	/* Frame Sync Generator Reset */

/* Mute Control Register Bits */
#define MUTENA(val)  (val)
#define MUTEINPOL    BIT(2)
#define MUTEINENA    BIT(3)
#define MUTEIN       BIT(4)
#define MUTER        BIT(5)
#define MUTEX        BIT(6)
#define MUTEFSR      BIT(7)
#define MUTEFSX      BIT(8)
#define MUTEBADCLKR  BIT(9)
#define MUTEBADCLKX  BIT(10)
#define MUTERXDMAERR BIT(11)
#define MUTETXDMAERR BIT(12)

/* Transmitter Event Control Register Bits */
#define TXUNDRNINTENA BIT(0)
#define TXUNFSRINTENA BIT(1)
#define TXBADCLKINTENA BIT(2)
#define TXDMAERRINTENA BIT(3)
#define TXLASTINTENA  BIT(4)
#define TXDATAINTENA  BIT(5)
#define TXSOFINTENA   BIT(7)

/* Receiver Event Control Register Bits */
#define RXOVRNINTENA BIT(0)
#define RXUNFSRINTENA BIT(1)
#define RXBADCLKINTENA BIT(2)
#define RXDMAERRINTENA BIT(3)
#define RXLASTINTENA BIT(4)
#define RXDATAINTENA BIT(5)
#define RXSOFINTENA  BIT(7)

/* Transmitter Status Register bits. All are not defined */
#define TXBADCLK     BIT(2)

/* Receiver Status Register bits. All are not defined */
#define RXBADCLK     BIT(2)

/* Transmit Clock Check Control Register Bits */
#define TXPS(val)    (val)
#define TXBADSW      BIT(7)
#define TXMIN(val)   (val<<8)
#define TXMAX(val)   (val<<16)
#define TXCOUNT(val) (val<<24)

/* Receiver Clock Check Control Register Bits */
#define RXPS(val)    (val)
#define RXMIN(val)   (val<<8)
#define RXMAX(val)   (val<<16)
#define RXCOUNT(val) (val<<24)

/* Receiver DMA Event Control Register bits */
#define RXDATADMADIS BIT(0)

/* Transmitter DMA Event Control Register bits */
#define TXDATADMADIS BIT(0)

struct clk *mcasp_get_clock(u8 mcasp_id);
s8 mcasp_get_free (void);
s8 mcasp_get(u8 mcasp_id);
s8 mcasp_put(u8 mcasp_id);
s8 mcasp_stop_rx (u8 mcasp_id);
s8 mcasp_stop_tx (u8 mcasp_id);
s8 mcasp_stop(u8 mcasp_id);
s8 mcasp_start(u8 mcasp_id);
s8 mcasp_start_tx (u8 mcasp_id);
s8 mcasp_start_rx (u8 mcasp_id);
s8 mcasp_configure(u8 mcasp_id, void *mcasp_config);
s8 mcasp_stop_tx (u8 mcasp_id);
s8 mcasp_reg_configure (u8 mcasp_id, void *cfg);
void *mcasp_get_info (u8 mcasp_id);
void *mcasp_get_config (u8 mcasp_id);
s8 mcasp_unconfigure(u8 mcasp_id);
s8 mcasp_dev_init(u8 mcasp_count, void *mcasp_info);
s8 mcasp_dev_deinit(u8 mcasp_id);
s32 mcasp_set_sample_rate(u8 id, u32 sample_rate);
s32 mcasp_get_sample_rate(u8 id);
#endif				/* __ASM_ARM_MCASP_H__ */
