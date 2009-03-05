/*
 *  linux/include/asm-arm/arch-davinci/edma.h
 *
 *  BRIEF MODULE DESCRIPTION
 *      TI EDMA3 definitions
 *
 *  Copyright (C) 2006 Texas Instruments.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
/******************************************************************************
 * EDMA3 Driver
 * EDMA3 Driver abstracts each ParamEntry as a Logical DMA channel
 * for the user. So for eg on DM644x, the user can request 128 DMA channels
 *
 * Actual Physical DMA channels (on DM644x) = 64 EDMA channels + 8 QDMA channels
 *
 * User can request for two kinds of Logical DMA channels
 * DMA MasterChannel -> ParamEntry which is associated with a DMA channel.
 *                      On DM644x, there are (64 + 8) MasterChanneles
 *                      MasterChannel can be triggered by an event or manually
 *
 * DMA SlaveChannel  -> ParamEntry which is not associated with DMA cahnnel but
 *                      which can be used to associate with MasterChannel.
 *                      On DM644x, there are (128-(64 + 8)) SlaveChannels
 *                      SlaveChannel can only be triggered by a MasterChannel
 *
 */

#ifndef EDMA_H_
#define EDMA_H_


/* Generic defines for all the platforms */
#define EDMA_NUM_DMACH				64
#define EDMA_NUM_QDMACH				8
#define EDMA_NUM_TCC				64
#define EDMA_CC_BASE_ADDRESS			DAVINCI_DMA_3PCC_BASE
#define EDMA_XFER_COMPLETION_INT		IRQ_CCINT0
#define EDMA_CC_ERROR_INT			IRQ_CCERRINT
#define EDMA_EVENT_QUEUE_TC_MAPPING		1
#define EDMA_MASTER_SHADOW_REGION		0
#define EDMA_NUM_DMA_CHAN_DWRDS			(EDMA_NUM_DMACH / 32)
#define EDMA_NUM_QDMA_CHAN_DWRDS		1


/* SoC specific EDMA3 hardware information, should be provided for a new SoC */
/* DM644x specific EDMA3 information */
#define EDMA_DM644X_NUM_PARAMENTRY		128
#define EDMA_DM644X_NUM_EVQUE			2
#define EDMA_DM644X_NUM_TC			2
#define EDMA_DM644X_CHMAP_EXIST			0
#define EDMA_DM644X_NUM_REGIONS			4
#define EDMA_DM644X_CHANNEL_TO_EVENT_MAPPING_0	0x3DFF0FFCu
#define EDMA_DM644X_CHANNEL_TO_EVENT_MAPPING_1	0x007F1FFFu
/* end DM644x specific */

/* DM646x specific EDMA3 information */
#define EDMA_DM646X_NUM_PARAMENTRY		512
#define EDMA_DM646X_NUM_EVQUE			4
#define EDMA_DM646X_NUM_TC			4
#define EDMA_DM646X_CHMAP_EXIST			1
#define EDMA_DM646X_NUM_REGIONS			8
#define EDMA_DM646X_CHANNEL_TO_EVENT_MAPPING_0	0x30FF1FF0u
#define EDMA_DM646X_CHANNEL_TO_EVENT_MAPPING_1	0x003F07FFu
/* end DM646X specific info */

/* DM355 specific info */
#define EDMA_DM355_NUM_PARAMENTRY		512
#define EDMA_DM355_NUM_EVQUE			8
#define EDMA_DM355_NUM_TC			2
#define EDMA_DM355_CHMAP_EXIST			0
#define EDMA_DM355_NUM_REGIONS			4
#define EDMA_DM355_CHANNEL_TO_EVENT_MAPPING_0	0xFDFF0FFCu
#define EDMA_DM355_CHANNEL_TO_EVENT_MAPPING_1	0x007F1FFFu

/* end DM355 specific info */

/**************************************************************************\
* Register Overlay Structure for Channel Controller
\**************************************************************************/
/**************************************************************************\
* Register Overlay Structure for DRA
\**************************************************************************/
typedef struct {
	unsigned int drae;
	unsigned int draeh;
} edmacc_dra_regs;

/**************************************************************************\
* Register Overlay Structure for QUEEVTENTRY
\**************************************************************************/
typedef struct {
	unsigned int evt_entry;
} edmacc_que_evtentry_regs;

/**************************************************************************\
* Register Overlay Structure for SHADOW
\**************************************************************************/
typedef struct {
	unsigned int er;
	unsigned int erh;
	unsigned int ecr;
	unsigned int ecrh;
	unsigned int esr;
	unsigned int esrh;
	unsigned int cer;
	unsigned int cerh;
	unsigned int eer;
	unsigned int eerh;
	unsigned int eecr;
	unsigned int eecrh;
	unsigned int eesr;
	unsigned int eesrh;
	unsigned int ser;
	unsigned int serh;
	unsigned int secr;
	unsigned int secrh;
	unsigned char rsvd0[8];
	unsigned int ier;
	unsigned int ierh;
	unsigned int iecr;
	unsigned int iecrh;
	unsigned int iesr;
	unsigned int iesrh;
	unsigned int ipr;
	unsigned int iprh;
	unsigned int icr;
	unsigned int icrh;
	unsigned int ieval;
	unsigned char rsvd1[4];
	unsigned int qer;
	unsigned int qeer;
	unsigned int qeecr;
	unsigned int qeesr;
	unsigned int qser;
	unsigned int qsecr;
	unsigned char rsvd2[360];
} edmacc_shadow_regs;

/**************************************************************************\
* Register Overlay Structure for PARAMENTRY
\**************************************************************************/
typedef struct {
	unsigned int opt;
	unsigned int src;
	unsigned int a_b_cnt;
	unsigned int dst;
	unsigned int src_dst_bidx;
	unsigned int link_bcntrld;
	unsigned int src_dst_cidx;
	unsigned int ccnt;
} edmacc_paramentry_regs;

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct {
	unsigned int rev;
	unsigned int cccfg;
	unsigned char rsvd0[244];
	unsigned int clkgdis;
	unsigned int dchmap[64];
	unsigned int qchmap[8];
	unsigned char rsvd1[32];
	unsigned int dmaqnum[8];
	unsigned int qdmaqnum;
	unsigned char rsvd2[28];
	unsigned int quetcmap;
	unsigned int quepri;
	unsigned char rsvd3[120];
	unsigned int emr;
	unsigned int emrh;
	unsigned int emcr;
	unsigned int emcrh;
	unsigned int qemr;
	unsigned int qemcr;
	unsigned int ccerr;
	unsigned int ccerrclr;
	unsigned int eeval;
	unsigned char rsvd4[28];
	edmacc_dra_regs dra[8];
	unsigned int qrae[8];
	unsigned char rsvd5[96];
	edmacc_que_evtentry_regs queevtentry[8][16];
	unsigned int qstat[8];
	unsigned int qwmthra;
	unsigned int qwmthrb;
	unsigned char rsvd6[24];
	unsigned int ccstat;
	unsigned char rsvd7[188];
	unsigned int aetctl;
	unsigned int aetstat;
	unsigned int aetcmd;
	unsigned char rsvd8[244];
	unsigned int mpfar;
	unsigned int mpfsr;
	unsigned int mpfcr;
	unsigned int mppag;
	unsigned int mppa[8];
	unsigned char rsvd9[2000];
	unsigned int er;
	unsigned int erh;
	unsigned int ecr;
	unsigned int ecrh;
	unsigned int esr;
	unsigned int esrh;
	unsigned int cer;
	unsigned int cerh;
	unsigned int eer;
	unsigned int eerh;
	unsigned int eecr;
	unsigned int eecrh;
	unsigned int eesr;
	unsigned int eesrh;
	unsigned int ser;
	unsigned int serh;
	unsigned int secr;
	unsigned int secrh;
	unsigned char rsvd10[8];
	unsigned int ier;
	unsigned int ierh;
	unsigned int iecr;
	unsigned int iecrh;
	unsigned int iesr;
	unsigned int iesrh;
	unsigned int ipr;
	unsigned int iprh;
	unsigned int icr;
	unsigned int icrh;
	unsigned int ieval;
	unsigned char rsvd11[4];
	unsigned int qer;
	unsigned int qeer;
	unsigned int qeecr;
	unsigned int qeesr;
	unsigned int qser;
	unsigned int qsecr;
	unsigned char rsvd12[3944];
	edmacc_shadow_regs shadow[8];
	unsigned char rsvd13[4096];
	edmacc_paramentry_regs paramentry[512];
} edmacc_regs;



/**************************************************************************\
* Register Overlay Structure for Transfer Controller
\**************************************************************************/
/**************************************************************************\
* Register Overlay Structure for DFIREG
\**************************************************************************/
typedef struct  {
	unsigned int dfopt;
	unsigned int dfsrc;
	unsigned int dfcnt;
	unsigned int dfdst;
	unsigned int dfbidx;
	unsigned int dfmpprxy;
	unsigned char rsvd0[40];
} edmtc_dfiregregs;


/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
	unsigned int rev;
	unsigned int tccfg;
	unsigned char rsvd0[248];
	unsigned int tcstat;
	unsigned int intstat;
	unsigned int inten;
	unsigned int intclr;
	unsigned int intcmd;
	unsigned char rsvd1[12];
	unsigned int errstat;
	unsigned int erren;
	unsigned int errclr;
	unsigned int errdet;
	unsigned int errcmd;
	unsigned char rsvd2[12];
	unsigned int rdrate;
	unsigned char rsvd3[188];
	unsigned int popt;
	unsigned int psrc;
	unsigned int pcnt;
	unsigned int pdst;
	unsigned int pbidx;
	unsigned int pmpprxy;
	unsigned char rsvd4[40];
	unsigned int saopt;
	unsigned int sasrc;
	unsigned int sacnt;
	unsigned int sadst;
	unsigned int sabidx;
	unsigned int sampprxy;
	unsigned int sacntrld;
	unsigned int sasrcbref;
	unsigned int sadstbref;
	unsigned char rsvd5[28];
	unsigned int dfcntrld;
	unsigned int dfsrcbref;
	unsigned int dfdstbref;
	unsigned char rsvd6[116];
	edmtc_dfiregregs dfireg[4];
} edmatc_regs;


#define SAM					(1)
#define DAM					(1<<1)
#define SYNCDIM					(1<<2)
#define STATIC					(1<<3)
#define EDMA_FWID				(0x7<<8)
#define TCCMODE					(0x1<<11)
#define TCC					(0x3f<<12)
#define WIMODE					(0x1<<19)
#define TCINTEN					(0x1<<20)
#define ITCINTEN				(0x1<<21)
#define TCCHEN					(0x1<<22)
#define ITCCHEN					(0x1<<23)
#define SECURE					(0x1<<30)
#define PRIV					(0x1<<31)
#define CCRL_CCERR_TCCERR_SHIFT			(0x10u)

/** DMAQNUM bits Clear */
#define DMAQNUM_CLR_MASK(ch_num)		(~(0x7u<<(((ch_num)%8u)*4u)))
/** DMAQNUM bits Set */
#define DMAQNUM_SET_MASK(ch_num, que_num)	((0x7u & (que_num)) << \
							(((ch_num)%8u)*4u))
/** QDMAQNUM bits Clear */
#define QDMAQNUM_CLR_MASK(ch_num)		(~(0x7u<<((ch_num)*4u)))
/** QDMAQNUM bits Set */
#define QDMAQNUM_SET_MASK(ch_num, que_num)	((0x7u & (que_num)) << \
							((ch_num)*4u))

#define TRWORD					(0x7<<2)
#define PAENTRY					(0x1ff<<5)

/*if changing the QDMA_TRWORD do appropriate change in davinci_start_dma */
#define QDMA_DEF_TRIG_WORD			0x7u

/* QUETCMAP bits Clear */
#define QUETCMAP_CLR_MASK(que_num)		(~(0x7u << ((que_num) * 0x4u)))
/* QUETCMAP bits Set */
#define QUETCMAP_SET_MASK(que_num, tc_num)	((0x7u & (tc_num)) << \
							((que_num) * 0x4u))
/* QUEPRI bits Clear */
#define QUEPRI_CLR_MASK(que_num)		(~(0x7u << ((que_num) * 0x4u)))
/* QUEPRI bits Set */
#define QUEPRI_SET_MASK(que_num, que_pri)	((0x7u & (que_pri)) << \
							((que_num) * 0x4u))
/* QUEWMTHR bits Clear */
#define QUEWMTHR_CLR_MASK(que_num)		(~(0x1Fu << ((que_num) * 0x8u)))
/* QUEWMTHR bits Set */
#define QUEWMTHR_SET_MASK(que_num, que_thr)	((0x1Fu & (que_thr)) << \
							((que_num) * 0x8u))


/** DCHMAP-PaRAMEntry bitfield Clear */
#define DMACH_PARAM_CLR_MASK			(~0x3FE0u)
/** DCHMAP-PaRAMEntry bitfield Set */
#define DMACH_PARAM_SET_MASK(param_id)		(((0x3FE0u >> 0x5u) & \
							(param_id)) << 0x5u)

/** QCHMAP-PaRAMEntry bitfield Clear */
#define QDMACH_PARAM_CLR_MASK			(~0x3FE0u)
/** QCHMAP-PaRAMEntry bitfield Set */
#define QDMACH_PARAM_SET_MASK(param_id)		(((0x3FE0u >> 0x5u) & \
							(param_id)) << 0x5u)
/** QCHMAP-TrigWord bitfield Clear */
#define QDMACH_TRWORD_CLR_MASK			(~0x1Cu)
/** QCHMAP-TrigWord bitfield Set */
#define QDMACH_TRWORD_SET_MASK(param_id)	(((0x1Cu >> 0x2u) & \
							(param_id)) << 0x2u)


/* Defines needed for TC error checking */
#define EDMA_TC_ERRSTAT_BUSERR_SHIFT		(0x00000000u)
#define EDMA_TC_ERRSTAT_TRERR_SHIFT		(0x00000002u)
#define EDMA_TC_ERRSTAT_MMRAERR_SHIFT		(0x00000003u)

/* Maximum number of TCs possible */
#define EDMA_MAX_TC				(8u)
/* Maximum number of PARAMs possible */
#define EDMA_MAX_PARAM_SET			(512u)

/* Defines for QDMA Channels */
#define EDMA_MAX_CHANNEL			(7u)
#define EDMA_QDMA_CHANNEL_0			davinci_get_qdma_channel(0)
#define EDMA_QDMA_CHANNEL_1			davinci_get_qdma_channel(1)
#define EDMA_QDMA_CHANNEL_2			davinci_get_qdma_channel(2)
#define EDMA_QDMA_CHANNEL_3			davinci_get_qdma_channel(3)
#define EDMA_QDMA_CHANNEL_4			davinci_get_qdma_channel(4)
#define EDMA_QDMA_CHANNEL_5			davinci_get_qdma_channel(5)
#define EDMA_QDMA_CHANNEL_6			davinci_get_qdma_channel(6)
#define EDMA_QDMA_CHANNEL_7			davinci_get_qdma_channel(7)

/* Used for any TCC (Interrupt Channel) */
#define EDMA_TCC_ANY				1001
/* Used for LINK Channels */
#define DAVINCI_EDMA_PARAM_ANY			1002
/* Used for any DMA Channel */
#define EDMA_DMA_CHANNEL_ANY			1003
/* Used for any QDMA Channel */
#define EDMA_QDMA_CHANNEL_ANY			1004



/* DaVinci specific EDMA3 Events Information */
#define DAVINCI_DMA_MCBSP_TX			2
#define DAVINCI_DMA_MCBSP_RX			3
#define DAVINCI_DMA_VPSS_HIST			4
#define DAVINCI_DMA_VPSS_H3A			5
#define DAVINCI_DMA_VPSS_PRVU			6
#define DAVINCI_DMA_VPSS_RSZ			7
#define DAVINCI_DMA_IMCOP_IMXINT		8
#define DAVINCI_DMA_IMCOP_VLCDINT		9
#define DAVINCI_DMA_IMCO_PASQINT		10
#define DAVINCI_DMA_IMCOP_DSQINT		11
#define DAVINCI_DMA_SPI_SPIX			16
#define DAVINCI_DMA_SPI_SPIR			17
#define DAVINCI_DMA_UART0_URXEVT0		18
#define DAVINCI_DMA_UART0_UTXEVT0		19
#define DAVINCI_DMA_UART1_URXEVT1		20
#define DAVINCI_DMA_UART1_UTXEVT1		21
#define DAVINCI_DMA_UART2_URXEVT2		22
#define DAVINCI_DMA_UART2_UTXEVT2		23
#define DAVINCI_DMA_MEMSTK_MSEVT		24
#define DAVINCI_DMA_MMCRXEVT			26
#define DAVINCI_DMA_MMCTXEVT			27
#define DAVINCI_DMA_I2C_ICREVT			28
#define DAVINCI_DMA_I2C_ICXEVT			29
#define DAVINCI_DMA_GPIO_GPINT0			32
#define DAVINCI_DMA_GPIO_GPINT1			33
#define DAVINCI_DMA_GPIO_GPINT2			34
#define DAVINCI_DMA_GPIO_GPINT3			35
#define DAVINCI_DMA_GPIO_GPINT4			36
#define DAVINCI_DMA_GPIO_GPINT5			37
#define DAVINCI_DMA_GPIO_GPINT6			38
#define DAVINCI_DMA_GPIO_GPINT7			39
#define DAVINCI_DMA_GPIO_GPBNKINT0		40
#define DAVINCI_DMA_GPIO_GPBNKINT1		41
#define DAVINCI_DMA_GPIO_GPBNKINT2		42
#define DAVINCI_DMA_GPIO_GPBNKINT3		43
#define DAVINCI_DMA_GPIO_GPBNKINT4		44
#define DAVINCI_DMA_TIMER0_TINT0		48
#define DAVINCI_DMA_TIMER1_TINT1		49
#define DAVINCI_DMA_TIMER2_TINT2		50
#define DAVINCI_DMA_TIMER3_TINT3		51
#define DAVINCI_DMA_PWM0			52
#define DAVINCI_DMA_PWM1			53
#define DAVINCI_DMA_PWM2			54


/* DaVinci-HD specific EDMA3 Events Information */
#define DAVINCI_DM646X_DMA_MCASP0_AXEVTE0	4
#define DAVINCI_DM646X_DMA_MCASP0_AXEVTO0	5
#define DAVINCI_DM646X_DMA_MCASP0_AXEVT0	6
#define DAVINCI_DM646X_DMA_MCASP0_AREVTE0	7
#define DAVINCI_DM646X_DMA_MCASP0_AREVTO0	8
#define DAVINCI_DM646X_DMA_MCASP0_AREVT0	9
#define DAVINCI_DM646X_DMA_MCASP1_AXEVTE1	10
#define DAVINCI_DM646X_DMA_MCASP1_AXEVTO1	11
#define DAVINCI_DM646X_DMA_MCASP1_AXEVT1	12
#define DAVINCI_DM646X_DMA_IMCOP1_CP_ECDCMP1	43
#define DAVINCI_DM646X_DMA_IMCOP1_CP_MC1	44
#define DAVINCI_DM646X_DMA_IMCOP1_CP_BS1	45
#define DAVINCI_DM646X_DMA_IMCOP1_CP_CALC1	46
#define DAVINCI_DM646X_DMA_IMCOP1_CP_LPF1	47
#define DAVINCI_DM646X_DMA_IMCOP0_CP_ME0	57
#define DAVINCI_DM646X_DMA_IMCOP0_CP_IPE0	58
#define DAVINCI_DM646X_DMA_IMCOP0_CP_ECDCMP0	59
#define DAVINCI_DM646X_DMA_IMCOP0_CP_MC0	60
#define DAVINCI_DM646X_DMA_IMCOP0_CP_BS0	61
#define DAVINCI_DM646X_DMA_IMCOP0_CP_CALC0	62
#define DAVINCI_DM646X_DMA_IMCOP0_CP_LPF0	63

/* DM355 specific EDMA3 Events Information */
#define DM355_DMA_TIMER3_TINT6			0
#define DM355_DMA_TIMER3_TINT7			1
#define DM355_DMA_MCBSP0_TX			2
#define DM355_DMA_MCBSP0_RX			3
#define DM355_DMA_MCBSP1_TX			8
#define DM355_DMA_TIMER2_TINT4			8
#define DM355_DMA_MCBSP1_RX			9
#define DM355_DMA_TIMER2_TINT5			9
#define DM355_DMA_SPI2_SPI2XEVT			10
#define DM355_DMA_SPI2_SPI2REVT			11
#define DM355_DMA_IMCOP_IMXINT			12
#define DM355_DMA_IMCOP_SEQINT			13
#define DM355_DMA_SPI1_SPI1XEVT			14
#define DM355_DMA_SPI1_SPI1REVT			15
#define DM355_DMA_SPI0_SPIX0			16
#define DM355_DMA_SPI0_SPIR0			17
#define DM355_DMA_RTOINT			24
#define DM355_DMA_GPIO_GPINT9			25
#define DM355_DMA_MMC0RXEVT			26
#define DM355_DMA_MEMSTK_MSEVT			26
#define DM355_DMA_MMC0TXEVT			27
#define DM355_DMA_MMC1RXEVT			30
#define DM355_DMA_MMC1TXEVT			31
#define DM355_DMA_GPIO_GPBNKINT5		45
#define DM355_DMA_GPIO_GPBNKINT6		46
#define DM355_DMA_GPIO_GPINT8			47
#define DM355_DMA_TIMER0_TINT1			49
#define DM355_DMA_TIMER1_TINT2			50
#define DM355_DMA_TIMER1_TINT3			51
#define DM355_DMA_PWM3				55
#define DM355_DMA_IMCOP_VLCDINT			56
#define DM355_DMA_IMCOP_BIMINT			57
#define DM355_DMA_IMCOP_DCTINT			58
#define DM355_DMA_IMCOP_QIQINT			59
#define DM355_DMA_IMCOP_BPSINT			60
#define DM355_DMA_IMCOP_VLCDERRINT		61
#define DM355_DMA_IMCOP_RCNTINT			62
#define DM355_DMA_IMCOP_COPCINT			63

/*ch_status paramater of callback function possible values*/
#define DMA_COMPLETE				1
#define DMA_EVT_MISS_ERROR			2
#define QDMA_EVT_MISS_ERROR			3
#define DMA_CC_ERROR				4
#define DMA_TC0_ERROR				5
#define DMA_TC1_ERROR				6
#define DMA_TC2_ERROR				7
#define DMA_TC3_ERROR				8

enum address_mode {
	INCR = 0,
	FIFO = 1
};

enum fifo_width {
	W8BIT = 0,
	W16BIT = 1,
	W32BIT = 2,
	W64BIT = 3,
	W128BIT = 4,
	W256BIT = 5
};

enum dma_event_q {
	EVENTQ_0 = 0,
	EVENTQ_1 = 1,
	EVENTQ_2 = 2,
	EVENTQ_3 = 3,
};

enum sync_dimension {
	ASYNC = 0,
	ABSYNC = 1
};

enum resource_type {
	RES_DMA_CHANNEL = 0,
	RES_QDMA_CHANNEL = 1,
	RES_TCC = 2,
	RES_PARAM_SET = 3
};

/******************************************************************************
 *
 * davinci_get_qdma_channel: Convert qdma channel to logical channel
 * Arguments:
 *      ch      - input qdma channel.
 *
 * Return: logical channel associated with qdma channel or logical channel
 *      associated with qdam channel 0 for out of range channel input.
 *
 *****************************************************************************/
int davinci_get_qdma_channel(int ch);

/******************************************************************************
 * davinci_request_dma - request for the Davinci DMA channel
 *
 * dev_id - DMA channel number
 *
 * EX: DAVINCI_DMA_MCBSP_TX - For requesting a DMA MasterChannel with MCBSP_TX
 *     event association
 *
 *     EDMA_DMA_CHANNEL_ANY - For requesting a DMA Master channel which does
 *                              not has event association
 *
 *     DAVINCI_EDMA_PARAM_ANY - for requesting a DMA Slave Channel
 *
 * dev_name   - name of the dma channel in human readable format
 * callback   - channel callback function (valied only if you are requesting
 *              for a DMA MasterChannel)
 * data       - private data for the channel to be requested
 * lch        - contains the device id allocated
 * tcc        - specifies the channel number on which the interrupt is
 *              generated
 *              Valied for QDMA and PARAM channes
 * eventq_no  - Event Queue no to which the channel will be associated with
 *              (valied only if you are requesting for a DMA MasterChannel)
 *              Values : EVENTQ_0/EVENTQ_1 for event queue 0/1.
 *
 * Return: zero on success,
 *         -EINVAL - if the requested channel is not supported on the ARM side events
 *
 *****************************************************************************/
int davinci_request_dma(int dev_id,
			const char *dev_name,
			void (*callback) (int lch, unsigned short ch_status,
				void *data), void *data, int *lch,
			int *tcc, enum dma_event_q);

/******************************************************************************
 *
 * Free DMA channel - Free the dma channel number passed
 *
 * ARGUMENTS:
 * lch - dma channel number to get free
 *
 *****************************************************************************/
void davinci_free_dma(int lch);

/******************************************************************************
 * davinci_set_dma_src_params - DMA source parameters setup
 *
 * lch         - channel for which the source parameters to be configured
 * src_port    - Source port address
 * addressMode - indicates whether the address mode is FIFO or not
 * fifoWidth   - valied only if addressMode is FIFO, indicates the vidth of
 *                FIFO
 *             0 - 8 bit
 *             1 - 16 bit
 *             2 - 32 bit
 *             3 - 64 bit
 *             4 - 128 bit
 *             5 - 256 bit
 *****************************************************************************/
void davinci_set_dma_src_params(int lch, unsigned long src_port,
				enum address_mode mode, enum fifo_width);

/******************************************************************************
 * davinci_set_dma_dest_params - DMA destination parameters setup
 *
 * lch         - channel or param device for destination parameters to be
 *               configured
 * dest_port   - Destination port address
 * addressMode - indicates whether the address mode is FIFO or not
 * fifoWidth   - valied only if addressMode is FIFO,indicates the vidth of FIFO
 *             0 - 8 bit
 *             1 - 16 bit
 *             2 - 32 bit
 *             3 - 64 bit
 *             4 - 128 bit
 *             5 - 256 bit
 *
 *****************************************************************************/
void davinci_set_dma_dest_params(int lch, unsigned long dest_port,
				 enum address_mode mode, enum fifo_width);

/******************************************************************************
 * davinci_set_dma_src_index - DMA source index setup
 *
 * lch     - channel or param device for configuration of source index
 * srcbidx - source B-register index
 * srccidx - source C-register index
 *
 *****************************************************************************/
void davinci_set_dma_src_index(int lch, short srcbidx, short srccidx);

/******************************************************************************
 * davinci_set_dma_dest_index - DMA destination index setup
 *
 * lch      - channel or param device for configuration of destination index
 * destbidx - dest B-register index
 * destcidx - dest C-register index
 *
 *****************************************************************************/
void davinci_set_dma_dest_index(int lch, short destbidx, short destcidx);

/******************************************************************************
 * davinci_set_dma_transfer_params -  DMA transfer parameters setup
 *
 * lch  - channel or param device for configuration of aCount, bCount and
 *        cCount regs.
 * aCnt - aCnt register value to be configured
 * bCnt - bCnt register value to be configured
 * cCnt - cCnt register value to be configured
 *
 *****************************************************************************/
void davinci_set_dma_transfer_params(int lch, unsigned short acnt,
				     unsigned short bcnt, unsigned short ccnt,
				     unsigned short bcntrld,
				     enum sync_dimension sync_mode);

/******************************************************************************
 *
 * davinci_set_dma_params -
 * ARGUMENTS:
 *      lch - logical channel number
 *
 *****************************************************************************/
void davinci_set_dma_params(int lch, edmacc_paramentry_regs * temp);

/******************************************************************************
 *
 * davinci_get_dma_params -
 * ARGUMENTS:
 *      lch - logical channel number
 *
 *****************************************************************************/
void davinci_get_dma_params(int lch, edmacc_paramentry_regs * temp);

/******************************************************************************
 * davinci_start_dma -  Starts the dma on the channel passed
 *
 * lch - logical channel number
 *
 * Note:    This API can be used only on DMA MasterChannel
 *
 * Return: zero on success
 *        -EINVAL on failure, i.e if requested for the slave channels
 *
 *****************************************************************************/
int davinci_start_dma(int lch);

/******************************************************************************
 * davinci_stop_dma -  Stops the dma on the channel passed
 *
 * lch - logical channel number
 *
 * Note:    This API can be used on MasterChannel and SlaveChannel
 *****************************************************************************/
void davinci_stop_dma(int lch);

/******************************************************************************
 * davinci_dma_link_lch - Link two Logical channels
 *
 * lch_head  - logical channel number, in which the link field is linked to the
 *             the param pointed to by lch_queue
 *             Can be a MasterChannel or SlaveChannel
 * lch_queue - logical channel number or the param entry number, which is to be
 *             linked to the lch_head
 *             Must be a SlaveChannel
 *
 *                     |---------------|
 *                     v               |
 *      Ex:    ch1--> ch2-->ch3-->ch4--|
 *
 *             ch1 must be a MasterChannel
 *
 *             ch2, ch3, ch4 must be SlaveChannels
 *
 * Note:       After channel linking,the user should not update any PaRam entry
 *             of MasterChannel ( In the above example ch1 )
 *
 *****************************************************************************/
void davinci_dma_link_lch(int lch_head, int lch_queue);

/******************************************************************************
 * davinci_dma_unlink_lch - unlink the two logical channels passed through by
 *                          setting the link field of head to 0xffff.
 *
 * lch_head  - logical channel number, from which the link field is to be
 *             removed
 * lch_queue - logical channel number or the param entry number,which is to be
 *             unlinked from lch_head
 *
 *****************************************************************************/
void davinci_dma_unlink_lch(int lch_head, int lch_queue);

/******************************************************************************
 *
 * DMA channel chain - chains the two logical channels passed through by
 * ARGUMENTS:
 * lch_head - logical channel number, from which the link field is to be removed
 * lch_queue - logical channel number or the param entry number, which is to be
 *             unlinked from lch_head
 *
 *****************************************************************************/
void davinci_dma_chain_lch(int lch_head, int lch_queue);

/******************************************************************************
 *
 * DMA channel unchain - unchain the two logical channels passed through by
 * ARGUMENTS:
 * lch_head - logical channel number, from which the link field is to be removed
 * lch_queue - logical channel number or the param entry number, which is to be
 *             unlinked from lch_head
 *
 *****************************************************************************/
void davinci_dma_unchain_lch(int lch_head, int lch_queue);


#endif
