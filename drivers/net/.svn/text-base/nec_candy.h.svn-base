/*
 * drivers/net/nec_candy.h
 *
 * NEC Candy Ethernet driver.
 *
 * Author: Wade Farnsworth <wfarnsworth@mvista.com>
 *
 * 2001-2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef NEC_CANDY_H
#define NEC_CANDY_H

/***********************************************************************
 * Configure
 ***********************************************************************/

#define TX_RING_SIZE	32
#define RX_RING_SIZE	512

#define RX_BUF_SIZE	1536
#define ETH_FRAME_SIZE	1536

#define TX_TIMEOUT	4*HZ

#define MAX_MARVELL_PORTS 10

#ifndef CONFIG_CANDY_NAPI
/* rx_copybreak:  for smaller packet we copy them to avoid emulated
 * unaligned access overhead.
 *
 * Set it to 1518 to always copy ( you should do that on fast machines)
 *
 * Set it to 0 to avoid any copy.
 *
 * On Korva, some value in the middle might be appropriate.
 */
static int rx_copybreak = 1518;
#endif

/***********************************************************************
 * Hardware bug workarounds table
 **********************************************************************/
/*	                CMBVR4133  |    VRBLADE V 2.0  |   VR7701     */
/**********************************************************************/
/*   E7_AFCE		  -        |        -          |     -	      */
/*   E10_PRM_AMC	  -        |        -          |     -	      */
/*   E13_TXFC	  	  -        |        -          |     -	      */
/*   E8_TX_STALL  	  -        |        -          |     -	      */
/*   E21_PAD  	  	  +        |        +          |     -	      */
/*   E10_VR4133  	  +        |        +          |     -	      */
/*   E19_VR4133A  	  -        |        +          |     -	      */
/*   E20_VR4133A  	  -        |        +          |     -	      */

/***********************************************************************
 * Candy.h macros
 ***********************************************************************
 */
/*---------------------------------------------------------------------------*/
/* PHY link status                                                           */
/*---------------------------------------------------------------------------*/
#define LINK_UP     1
#define LINK_DOWN   0

/*---------------------------------------------------------------------------*/
/* receive mode & related definitions                                        */
/*---------------------------------------------------------------------------*/
#define ACCEPT_ALL_PHYS         0x0001
#define ACCEPT_ALL_MCASTS       0x0002
#define ACCEPT_ALL_BCASTS       0x0004
#define ACCEPT_QUALIFIED_MCAST  0x0008
#define ACCEPT_STATION          0x0010
#define MAC_LOOPBACK            0x0020

/*---------------------------------------------------------------------------*/
/* MACC1 - MAC configuration register 1 (00H R/W)                            */
/*---------------------------------------------------------------------------*/
#define MACLB		0x00004000	/* MAC loopback */
#define TXFC		0x00000800	/* Transmit flow control enable */
#define RXFC		0x00000400	/* Receive flow control enable */
#define SRXEN		0x00000200	/* Receive enable */
#define PARF		0x00000100	/* Control packet pass */
#define PUREP		0x00000080	/* Pure preamble */
#define FLCHT		0x00000040	/* Length field check */
#define NOBO		0x00000020	/* No Back Off */
#define CRCEN		0x00000008	/* CRC append enable */
#define PADEN		0x00000004	/* PAD append enable */
#define FULLD		0x00000002	/* Full duplex enable */
#define HUGEN		0x00000001	/* Large packet enable */
#define MACC1_RESERVED	0x00004fef	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* MACC2 - MAC configuration register 2 (04H R/W)                            */
/*---------------------------------------------------------------------------*/
#define MCRST		0x00000400	/* MAC Control Block software reset */
#define RFRST		0x00000200	/* Rx Function Block software reset */
#define TFRST		0x00000100	/* Tx Function Block software reset */
#define BPNB		0x00000040	/* Back Pressure No Back Off */
#define APD		0x00000020	/* Auto VLAN PAD */
#define VPD		0x00000010	/* VLAN PAD mode */
#define MACC2_RESERVED	0x00000770	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* IPGT - Back-to-Back IPG register (08H R/W)                                */
/*---------------------------------------------------------------------------*/
#define IPGT		0x00000013	/* Back-To-Back IPG default value */
#define IPGT_RESERVED	0x0000007f	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* IPGR - Non Back-to-Back IPG register (0CH R/W)                            */
/*---------------------------------------------------------------------------*/
#define IPGR1		0x00000e00	/* Back-To-Back IPG default value */
#define IPGR2		0x00000013	/* Back-To-Back IPG default value */
#define IPGR_RESERVED	0x00007f7f	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* CLRT - Collision register (10H R/W)                                       */
/*---------------------------------------------------------------------------*/
#define LCOLW		0x00003800	/* Late collision window dflt value */
#define RETRY		0x0000000f	/* Max number of retry default value */
#define CLRT_RESERVED	0x00003f0f	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* LMAX - Maximum Packet Length register (14H R/W)                           */
/*---------------------------------------------------------------------------*/
#define MAXF		0x000005f2	/* Max pkt length value (1522 bytes) */
#define LMAX_RESERVED	0x0000ffff	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* RETX - Retry count register (20H R/W)                                     */
/*---------------------------------------------------------------------------*/
#define RETX_MASK	0x0000000f	/* Retry counter */
#define RETX_RESERVED	0x0000000f	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* LSA2 - Station address register 2 (54H R/W)                               */
/*---------------------------------------------------------------------------*/
#define LSA2_MASK	0x0000ffff  	/* Station address SA (47:32) */
#define LSA2_RESERVED	0x0000ffff	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* LSA1 - Station address register 1 (58H R/W)                               */
/*---------------------------------------------------------------------------*/
#define LSA1_MASK	0xffffffff	/* Station address SA(31:0) */

/*---------------------------------------------------------------------------*/ /* PTVR - Pause timer read register (5CH Read)                               */
/*---------------------------------------------------------------------------*/
#define PTCT_MASK	0x0000ffff	/* Pause timer counter */

/*---------------------------------------------------------------------------*/
/* VLTP - VLAN type register (64H R/W)                                       */
/*---------------------------------------------------------------------------*/
#define VLTP		0x00008100	/* VLAN type ( etpid:0x81 tci:0x00 ) */
#define VLTP_RESERVED	0x0000ffff	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* MIIC - MII Configuration register (80H R/W)                               */
/*---------------------------------------------------------------------------*/
#define MISRT		0x00008000	/* MII Mgmt Interface Block s/w reset */
#define CLKS25		0x00000000	/* HCLK <= 25 MHz */
#define CLKS33		0x00000004	/* HCLK <= 33 MHz */
#define CLKS50		0x00000008	/* HCLK <= 50 MHz */
#define CLKS66		0x0000000c	/* HCLK <= 66 MHz */
#define MIIC_RESERVED	0x0000800c	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* MCMD - MII command register (94H Write)                                   */
/*---------------------------------------------------------------------------*/
#define SCANC		0x00000002	/* SCAN command */
#define RSTAT		0x00000001	/* MII management read */
#define MCMD_RESERVED	0x00000003	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* MADR - MII address register (98H R/W)                                     */
/*---------------------------------------------------------------------------*/
#define FIAD_MASK	0x00001f00	/* MII PHY address */
#define FIAD_SHIFT  	8
#define RGAD_MASK   	0x0000001f	/* MII register address */
#define MADR_RESERVED	0x00001f1f	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* MIND - MII indicator register (A4H Read)                                  */
/*---------------------------------------------------------------------------*/
#define NVALID		0x00000004	/* SCAN command start status */
#define SCANA		0x00000002	/* SCAN command active */
#define BUSY		0x00000001	/* BUSY */
#define MIND_RESERVED	0x00000007	/* reserved bit 0 */


/*---------------------------------------------------------------------------*/
/* STLC - STL configuration register (C0H R/W)                               */
/*---------------------------------------------------------------------------*/
#define ATZ		0x00000004	/* Statistics counter read reset */
#define STLC_RESERVED	0x00000004	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* AFR - Address Filter register (C8H R/W)                                   */
/*---------------------------------------------------------------------------*/
#define PRO		0x00000008	/* Promiscuous mode */
#define PRM		0x00000004	/* Accept Multicast */
#define AMC		0x00000002	/* Accept Multicast ( qualified ) */
#define ABC		0x00000001	/* Accept Broadcast */
#define AFR_RESERVED	0x0000000f	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* CAR1 - CARRY register1 (DCH R/W)                                          */
/*---------------------------------------------------------------------------*/
#define C1VT		0x00008000	/* RVBT counter carry bit */
#define C1UT		0x00004000	/* TUCA counter carry bit */
#define C1BT		0x00002000	/* TBCA counter carry bit */
#define C1MT		0x00001000	/* TMCA counter carry bit */
#define C1PT		0x00000800	/* TPCT counter carry bit */
#define C1TB		0x00000400	/* TBYT counter carry bit */
#define C1MX		0x00000200	/* RMAX counter carry bit */
#define C11K		0x00000100	/* R1K counter carry bit */
#define C1FE		0x00000080	/* R511 counter carry bit */
#define C1TF		0x00000040	/* R255 counter carry bit */
#define C1OT		0x00000020	/* R127 counter carry bit */
#define C1SF		0x00000010	/* R64 counter carry bit */
#define C1BR		0x00000008	/* RBCA counter carry bit */
#define C1MR		0x00000004	/* RBCA counter carry bit */
#define C1PR		0x00000002	/* RPKT counter carry bit */
#define C1RB		0x00000001	/* RBYT counter carry bit */
#define CAR1_RESERVED	0x0000ffff	/* reserved bit 0  */

/*---------------------------------------------------------------------------*/
/* CAR2 - CARRY register2 (E0H R/W)                                          */ /*---------------------------------------------------------------------------*/
#define C2SV		0x80000000	/* Status vector overrun bit */
#define C2IM		0x00400000	/* TIME counter carry bit */
#define C2CS		0x00200000	/* TCSE counter carry bit */
#define C2BC		0x00100000	/* TNCL counter carry bit */
#define C2XC		0x00080000	/* TXCL counter carry bit */
#define C2LC		0x00040000	/* TLCL counter carry bit */
#define C2MC		0x00020000	/* TMCL counter carry bit */
#define C2SC		0x00010000	/* TSCL counter carry bit */
#define C2XD		0x00008000	/* TXDF counter carry bit */
#define C2DF		0x00004000	/* TDFR counter carry bit */
#define C2XF		0x00002000	/* TXPF counter carry bit */
#define C2TE		0x00001000	/* TFCS counter carry bit */
#define C2JB		0x00000800	/* RBJR counter carry bit */
#define C2FG		0x00000400	/* RFRG counter carry bit */
#define C2OV		0x00000200	/* ROVR counter carry bit */
#define C2UN		0x00000100	/* RUND counter carry bit */
#define C2FC		0x00000080	/* RFCR counter carry bit */
#define C2CD		0x00000040	/* RCDE counter carry bit */
#define C2FO		0x00000020	/* RFLR counter carry bit */
#define C2AL		0x00000010	/* RALN counter carry bit */
#define C2UO		0x00000008	/* RXUO counter carry bit */
#define C2PF		0x00000004	/* RXPF counter carry bit */
#define C2CF		0x00000002	/* RXCF counter carry bit */
#define C2RE		0x00000001	/* RFCS counter carry bit */
#define CAR2_RESERVED   0x807fffff	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* TXCFG - Transmit Configuration (200H R/W)                                 */
/*---------------------------------------------------------------------------*/
#define AFCE		0x00000001	/* Automatic Flow Control Enable */
#define DTBS1		0x00000000	/* DMA Transmit Burst Size 1 word */
#define DTBS2		0x00010000	/* DMA Transmit Burst Size 2 word */
#define DTBS4		0x00020000	/* DMA Transmit Burst Size 4 word */
#define DTBS8		0x00030000	/* DMA Transmit Burst Size 8 word */
#define DTBS16		0x00040000	/* DMA Transmit Burst Size 16 word */
#define DTBS32		0x00050000	/* DMA Transmit Burst Size 32 word */
#define DTBS64		0x00060000	/* DMA Transmit Burst Size 64 word */
#define TXE		0x80000000	/* Transmit Enable */
#define TXCFG_RESERVED	0x80070001	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* TXFC - Transmit FIFO Control (204H R/W)                                   */
/*---------------------------------------------------------------------------*/
#define TPTV_MASK	0xffff0000	/* Transmit Pause Timer Value mask       */
#define TPTV	0x10000000	/* default 0x1000 slot time (1slot:512bit) */
#define TX_DRTH_MASK	0x0000fc00	/* Transmit Fill Threshold Level mask    */
#define TX_DRTH	0x00004000	/* default 010000b (16word, 64byte)      */
#define TX_FLTH_MASK	0x000000fc	/* Transmit Drain Threshold Level mask   */

/* The hardware restriction: the sum of TX_FLTH and DTBS should be <= 192 bytes. */
/* DTBS is set to 32 bytes for VR4133. So we set this to 160 byte */
#define TX_FLTH_VR4133		0x000000a0	/* 101000b (160byte)     */
/* DTBS is set to 64 bytes for VR4133A. So we set this to 128 byte */
#define TX_FLTH_VR4133A		0x00000080	/* 100000b (128byte)     */
#define TX_FLTH		0x000000c0	/* default 110000b (48word, 192byte) */
#define TXFC_RESERVED	0xfffffcfc	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* TXST - Transmit Status  (20CH R)                                          */
/*---------------------------------------------------------------------------*/
#define CSE_AB		0x80000000	/* carrier lost (abort) */
#define TBP		0x40000000	/* back pressure occurred */
#define TPP		0x20000000	/* packet requested during PAUSE */
#define TPCF		0x10000000	/* transmit PAUSE control frame */
#define TCFR		0x08000000	/* transmit control frame */
#define TUDR_AB		0x04000000	/* underrun (abort) */
#define TGNT_AB		0x02000000	/* greater than LMAX (abort) */
#define LCOL_AB		0x01000000	/* late collision  (abort) */
#define ECOL_AB		0x00800000	/* excessive collisions (abort) */
#define TEDFR_AB	0x00400000	/* excessive defer (abort) */
#define TDFR		0x00200000	/* single defer */
#define TBRO		0x00100000	/* broadcast packet */
#define TMUL		0x00080000	/* multicast packet */
#define TDONE		0x00040000	/* transmit complete */
#define TFLOR		0x00020000	/* length field  was over 1518 bytes */
#define TFLER		0x00010000	/* length field != actual length */
#define TCRCE		0x00008000	/* CRC error */
#define TCBC_MASK	0x00007800	/* number of collisions */
#define TBYT_MASK	0x000007ff	/* number of the transmitted bytes */

/*---------------------------------------------------------------------------*/
/* RXCFG - Receive Configuration (218H R/W)                                  */
/*---------------------------------------------------------------------------*/
#define DRBS1		0x00000000	/* DMA Receive Burst Size 1 word */
#define DRBS2		0x00010000	/* DMA Receive Burst Size 2 word */
#define DRBS4		0x00020000	/* DMA Receive Burst Size 4 word */
#define DRBS8		0x00030000	/* DMA Receive Burst Size 8 word */
#define DRBS16		0x00040000	/* DMA Receive Burst Size 16 word */
#define DRBS32		0x00050000	/* DMA Receive Burst Size 32 word */
#define DRBS64		0x00060000	/* DMA Receive Burst Size 64 word */
#define RXE		0x80000000	/* Receive Enable */
#define RXCFG_RESERVED	0x80070000	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* RXFC - Receive FIFO Control (21CH R/W)                                    */
/*---------------------------------------------------------------------------*/
#define UWM_MASK	0xfc000000	/* Upper Water Mark mask */
#define UWM		0xc0000000	/* default 110000b (48word, 192byte) */
#define LWM_MASK	0x00fc0000	/* Lower Water Mark mask */
#define LWM		0x00400000	/* default 010000b (16word, 64byte) */
#define RX_DRTH_MASK	0x000000fc	/* Receive Drain Threshold Level */
#define RX_DRTH16W	0x00000040	/* default 010000b (16word, 64byte) */
#define RX_DRTH28W	0x00000070	/* default 011100b (28word, 112byte) */
#define RXFC_RESERVED	0xfcfc00fc	/* reserved bit 0 */

/*---------------------------------------------------------------------------*/
/* RXPD - Receive Pool Descriptor (230H R/W)                                 */
/*---------------------------------------------------------------------------*/
#define AL		0x70000000	/* Alert Level default value */
#define RNOD_MASK	0x0000ffff	/* Remaining Number of Descriptor */

/*---------------------------------------------------------------------------*/
/* RXST - Receive Status (224H R)                                            */
/*---------------------------------------------------------------------------*/
#define RLENE		0x80000000	/* less than 64 or larger than 1518 */
#define VLAN		0x40000000  	/* match VLTP */
#define USOP 		0x20000000	/* unknown OP code control frame */
#define RPCF		0x10000000	/* receive PAUSE control frame */
#define RCFR		0x08000000	/* receive control frame */
#define DBNB		0x04000000	/* alignment error */
#define RBRO		0x02000000	/* broadcast packet */
#define RMUL		0x01000000	/* multicast packet */
#define RX_OK		0x00800000	/* receive OK */
#define RLOR		0x00400000	/* length field was over 1518 bytes */
#define RLER		0x00200000	/* length field != actual length */
#define RCRCE		0x00100000	/* CRC error */
#define RCVE		0x00080000	/* RXER was detected (PHY error) */
#define CEPS		0x00040000	/* false Carrier */
#define REPS		0x00020000	/* preamble+SFD or +one data nibble */
#define PAIG		0x00010000	/* carrier length 3036 octets,
					   Short IPG, invalid preamble
					   or invalid SFD */
#define RBYT_MASK	0x0000ffff	/* received byte count */

/*---------------------------------------------------------------------------*/
/* RXPD - Receive Pool Descriptor (230H R/W)                                 */
/*---------------------------------------------------------------------------*/
#define AL		0x70000000	/* Alert Level default value */
#define AL_MASK		0x70000000
#define RNOD_MASK	0x0000ffff	/* Remaining Number of Descriptor */
#define RXPD_RESERVED	0x7000ffff	/* reserved bit */

/*---------------------------------------------------------------------------*/
/* CCR - Candy Configuration Register (234H R / 240H W)                      */
/*---------------------------------------------------------------------------*/
#define SRT		0x00000001	/* Candy Software Reset */
#define MII_PIN_SELECT	0x20000000	/* MII pin selection */
#define RMII_MODE	0x40000000	/* RMII mode */
#define SPD100		0x80000000	/* Operation speed in RMII mode */

/*---------------------------------------------------------------------------*/
/* ISR - Interrupt Status Register (238H R with clear)                       */
/* MSR - Mask Interrupt Register   (23cH R/W)                                */
/*---------------------------------------------------------------------------*/
#define BUSERR		0x80000000	/* IBUS Error */
#define XMTDN		0x00008000	/* Transmit Done */
#define TBDR		0x00004000	/* Transmit BD Request at Null */
#define TFLE		0x00002000	/* Transmit Frame Length Exceed */
#define UR		0x00001000	/* Underrun */
#define TABR		0x00000800	/* Transmit Aborted */
#define TCF		0x00000400	/* Transmit Control Frame */
#define RCVDN		0x00000080	/* Receive Done */
#define RBDRS		0x00000040	/* Receive BD Request at alert level */
#define RBDRU		0x00000020	/* Rx Buffer Desc Request at zero */
#define OF		0x00000010	/* Overflow */
#define LFAL		0x00000008	/* Link Failed */
#define CARRY		0x00000001	/* statistics counters carry flag */
#define ISR_RESERVED	0x8000fcf9	/* reserved bit */

#define INT_ISR_TX_MASK	0x0000FC00	/* ISR TX bits mask */
#define INT_ISR_RX_MASK	0x000000F0	/* ISR RX bits mask */

/*---------------------------------------------------------------------------*/
/* Transmit/Receive Status bit definition in Transmit/Receive Descriptor     */
/*---------------------------------------------------------------------------*/
#define LAST		0x8000		/* Last Descriptor */
#define IEN		0x0008		/* Interrupt Enable*/
#define DB_LP		0x4000		/* Data Buffer / Link Pointer */
#define OWN		0x2000		/* Owner 1:used by candy, 0:host set */

/*---------------------------------------------------------------------------*/
/* Transmit Status bit definition in Transmit Descriptor                     */
/*---------------------------------------------------------------------------*/
#define DBRE		0x1000		/* Data Buffer Read Error */
#define TUDR		0x0800		/* Transmit Underrun Error */
#define CSE		0x0400		/* Carrier Sense Lost Error */
#define LCOL		0x0200		/* Late Collision */
#define ECOL		0x0100		/* Excessive Collision */
#define EDFR		0x0080		/* Excessive Deferral */
#define TGNT		0x0004		/* Transmit Giant Frame */
#define HBF		0x0002		/* Heart Beat Fail for ENDEC mode */
#define TOK		0x0001		/* Transmit OK */

/*---------------------------------------------------------------------------*/
/* Receive Status bit definition in Receive Descriptor                       */
/*---------------------------------------------------------------------------*/
#define DBWE		0x1000		/* Data Buffer Write Error */
#define FTYP_MASK	0x0e00		/* Frame Type */
#define BCASTF		0x0000		/* Broadcast Frame */
#define MCASTF		0x0200		/* Multicast Frame */
#define UCASTF		0x0400		/* Unicast Frame */
#define VLANF		0x0600		/* VLAN Frame */
#define PAUSEF		0x0800		/* PAUSE control Frame */
#define CTLF		0x0a00		/* Control Frame */
#define OVRN		0x0100		/* Overrun Error */
#define RUNT		0x0080		/* Runt packet */
#define FRGE		0x0040		/* Fragment Error */
#define RCV		0x0020		/* Detects RXER */
#define FC		0x0010		/* False Carrier */
#define CRCE		0x0008		/* CRC Error */
#define FAE		0x0004		/* Frame Alignment Error */
#define RFLE		0x0002		/* Receive Frame Length Error */
#define RXOK		0x0001		/* Receive OK */

#define CANDY_REGS_VER1	1		/* version 1 */
#define CANDY_REGS_VER2	2		/* version 2 */

#define CANDY_REGS_SIZE	(0x3ff + 1)

/***********************************************************************
 * data structure
 ***********************************************************************/
typedef volatile struct {
	ulong macc1;		/* 0x00  MAC configuration register 1 */
	ulong macc2;		/* 0x04  MAC configuration register 2 */
	ulong ipgt;		/* 0x08  Back-to-Back IPG register */
	ulong ipgr;		/* 0x0c  Non Back-to-Back IPG register */
	ulong clrt;		/* 0x10  Collision register */
	ulong lmax;		/* 0x14  Max packet length register */
	ulong reserved0[2];
	ulong retx;		/* 0x20  Retry count register */
	ulong reserved1[12];
	ulong lsa2;		/* 0x54  Station Address register 2 */
	ulong lsa1;		/* 0x58  Station Address register 1 */
	ulong ptvr;		/* 0x5c  Pause timer value read register */
	ulong reserved2[1];
	ulong vltp;		/* 0x64  VLAN type register */
	ulong reserved3[6];
	ulong miic;		/* 0x80  MII configuration register */
	ulong reserved4[4];
	ulong mcmd;		/* 0x94  MII command register */
	ulong madr;		/* 0x98  MII address register */
	ulong mwtd;		/* 0x9c  MII write data register */
	ulong mrdd;		/* 0xa0  MII read data register */
	ulong mind;		/* 0xa4  MII indicator register */
	ulong reserved5[6];
	ulong stlc;		/* 0xc0  Statistics counter config register */
	ulong reserved6[1];
	ulong afr;		/* 0xc8  Address filter register */
	ulong ht1;		/* 0xcc  Hash table register 1 */
	ulong ht2;		/* 0xd0  Hash table register 2 */
	ulong reserved7[2];
	ulong car1;		/* 0xdc  Carry register 1 */
	ulong car2;		/* 0xe0  Carry register 2 */
	ulong reserved8[19];
	ulong cam1;		/* 0x130 Carry mask register 1 */
	ulong cam2;		/* 0x134 Carry mask register 2 */
	ulong reserved9[2];

	/* RX Statistics Counters */
	ulong rbyt;		/* 0x140 Rx Byte Counter */
	ulong rpkt;		/* 0x144 Rx Pkt Counter */
	ulong rfcs;		/* 0x148 Rx FCS Error Counter */
	ulong rmca;		/* 0x14c Rx Multicast Pkt Counter */
	ulong rbca;		/* 0x150 Rx Broadcast Pkt Counter */
	ulong rxcf;		/* 0x154 Rx Control Frame Pkt Counter */
	ulong rxpf;		/* 0x158 Rx PAUSE Frame Pkt Counter */
	ulong rxuo;		/* 0x15c Rx Unknown OP code Counter */
	ulong raln;		/* 0x160 Rx Alignment Error Counter */
	ulong rflr;		/* 0x164 Rx Frame Length Out of Range Counter */
	ulong rcde;		/* 0x168 Rx Code Error Counter */
	ulong rfcr;		/* 0x16c Rx False Carrier Counter */
	ulong rund;		/* 0x170 Rx Undersize Pkt Counter */
	ulong rovr;		/* 0x174 Rx Oversize Pkt Counter */
	ulong rfrg;		/* 0x178 Rx Error Undersize Pkt Counter */
	ulong rjbr;		/* 0x17c Rx Error Oversize Pkt Counter */
	ulong r64;		/* 0x180 Rx 64 Byte Frame Counter */
	ulong r127;		/* 0x184 Rx 65 to 127 Byte Frame Counter */
	ulong r255;		/* 0x188 Rx 128 to 255 Byte Frame Counter */
	ulong r511;		/* 0x18c Rx 256 to 511 Byte Frame Counter */
	ulong r1k;		/* 0x190 Rx 512 to 1023 Byte Frame Counter */
	ulong rmax;		/* 0x194 Rx Over 1023 Byte Frame Counter */
	ulong rvbt;		/* 0x198 Rx Valid Byte Counter */
	ulong reserved10[9];

	/* Tx Statistics Counter */
	ulong tbyt;		/* 0x1c0 Tx Byte Counter */
	ulong tpct;		/* 0x1c4 Tx Pkt Counter */
	ulong tfcs;		/* 0x1c8 Tx CRC Error Pkt Counter */
	ulong tmca;		/* 0x1cc Tx Multicast Pkt Counter */
	ulong tbca;		/* 0x1d0 Tx Broadcast Pkt Counter */
	ulong tuca;		/* 0x1d4 Tx Unicast Pkt Counter */
	ulong txpf;		/* 0x1d8 Tx PAUSE control Frame Counter */
	ulong tdfr;		/* 0x1dc Tx Single Deferral Pkt Counter */
	ulong txdf;		/* 0x1e0 Tx Excessive Deferral Pkt Counter */
	ulong tscl;		/* 0x1e4 Tx Single Collision Pkt Counter */
	ulong tmcl;		/* 0x1e8 Tx Multiple Collision Pkt Counter */
	ulong tlcl;		/* 0x1ec Tx Late Collision Pkt Counter */
	ulong txcl;		/* 0x1f0 Tx Excessive Collision Pkt Counter */
	ulong tncl;		/* 0x1f4 Tx Total Collision Counter */
	ulong tcse;		/* 0x1f8 Tx Carrier Sense Error Counter */
	ulong time;		/* 0x1fc Tx Internal MAC Error Counter */

	/*-------------------------------------------------------------------*/
	/* Candy DMA and FIFO Management registers                           */
	/*-------------------------------------------------------------------*/
	ulong txcfg;		/* 0x200 Transmit Configuration */
	ulong txfc;		/* 0x204 Transmit FIFO Control  */
	ulong txd;		/* 0x208 Transmit Data */
	ulong txst;		/* 0x20c Transmit Status */
	ulong txfap;		/* 0x210 Tx FIFO access pointer */
	ulong txdp;		/* 0x214 Transmit Descriptor Pointer */
	ulong rxcfg;		/* 0x218 Receive Configuration */
	ulong rxfc;		/* 0x21c Receive FIFO Control  */
	ulong rxd;		/* 0x220 Receive Data */
	ulong rxst;		/* 0x224 Receive Status */
	ulong rxfap;		/* 0x228 Rx FIFO access pointer */
	ulong rxdp;		/* 0x22c Receive Descriptor Pointer */
	ulong rxpd;		/* 0x230 Receive Pool Descriptor */

	/*-------------------------------------------------------------------*/
	/* Candy Interrupt and Configuration registers                       */
	/*-------------------------------------------------------------------*/
	ulong ccr;		/* 0x234 CANDY Config Read/Write register */
	ulong isr;		/* 0x238 Interrupt Status register */
	ulong msr;		/* 0x23c Mask Interuupt register */
	ulong reserved11[8];
	ulong mode;		/* 0x260    Mode register */
	ulong reserved12[14];
	ulong erev;
} candy_regs;

/*
 * descriptor structure
 */
struct candy_desc {
#if defined(__LITTLE_ENDIAN)
	ushort size;
	ushort status;
#elif defined(__BIG_ENDIAN)
	ushort status;
	ushort size;
#else
#error "No endian format defined!"
#endif
	ulong pointer;
};

/*
 * private data structure for candy driver.
 */
struct candy_private {
	struct candy_desc *tx_ring;
	struct sk_buff *tx_skb[TX_RING_SIZE];

	struct candy_desc *rx_ring;
	struct sk_buff *rx_skb[RX_RING_SIZE];
	int rx_disable;

	uint rx_head;
	uint tx_head, tx_stop, tx_tail;
	int tx_count;

	struct net_device_stats stats;

	u32 msg_enable;

	spinlock_t lock;
	spinlock_t rxlock;

	struct mii_if_info mii_if;

	uint oui;
	/* Marvel MIIs */
	uint carrierPort[MAX_MARVELL_PORTS];
	uint speedPort[MAX_MARVELL_PORTS];

	/* MII status */
	struct link_status {
		uint fullduplex:1;
		uint speed100:1;
		uint linkOK:1;
	} link_status;

	struct timer_list phy_timer;

	/* hardware related */
	candy_regs *regs;

	/* house keeping */
	struct net_device *ndev;
	struct device *dev;
	struct candy_private *next;
#ifdef CONFIG_CANDY_NAPI
	ulong prev_rpkt;
#endif
};

#endif
