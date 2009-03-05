/*
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * This file implements a DMA  interface using TI's CPPI DMA.
 * For now it's DaVinci-only, but CPPI isn't specific to DaVinci or USB.
 */

#include <linux/config.h>
#include <linux/usb.h>

#include "cppi_dma.h"
#include "musbdefs.h"
#include "musb_host.h"

/* CPPI DMA status 7-mar:
 *
 * - See musb_{host,gadget}.c for more info
 *
 * - Correct RX DMA generally forces the engine into irq-per-packet mode,
 *   which can easily saturate the CPU under non-mass-storage loads.
 */

/* REVISIT now we can avoid preallocating these descriptors; or
 * more simply, switch to a global freelist not per-channel ones.
 * Note: at full speed, 64 descriptors == 4K bulk data.
 */
#define NUM_TXCHAN_BD       64
#define NUM_RXCHAN_BD       64

static struct dma_controller *cppi_controller_new(MGC_pfDmaChannelStatusChanged
						  pfDmaChannelStatusChanged,
						  void *musb, u8 * pCoreBase);
static void cppi_controller_destroy(struct dma_controller *pController);

struct dma_controller_factory dma_controller_factory = {
	.pfNewDmaController = cppi_controller_new,
	.pfDestroyDmaController = cppi_controller_destroy,
};

static inline void cpu_drain_writebuffer(void)
{
	wmb();
#ifdef	CONFIG_CPU_ARM926T
	/* REVISIT this "should not be needed",
	 * but lack of it sure seemed to hurt ...
	 */
	asm("mcr p15, 0, r0, c7, c10, 4 @ drain write buffer\n");
#endif
}

static inline struct cppi_descriptor *cppi_bd_alloc(struct cppi_channel *c)
{
	struct cppi_descriptor *bd = c->bdPoolHead;

	if (bd)
		c->bdPoolHead = bd->next;
	return bd;
}

static inline void
cppi_bd_free(struct cppi_channel *c, struct cppi_descriptor *bd)
{
	if (!bd)
		return;
	bd->next = c->bdPoolHead;
	c->bdPoolHead = bd;
}

/*
 *  Start Dma controller
 *
 *  Initialize the Dma Controller as necessary.
 */

#define	CAST (void *__force __iomem)

/* zero out entire rx state RAM entry for the channel */
static void cppi_reset_rx(struct cppi_rx_stateram *__iomem rx)
{
	musb_writel(CAST & rx->buffOffset, 0, 0);
	musb_writel(CAST & rx->headPtr, 0, 0);
	musb_writel(CAST & rx->sopDescPtr, 0, 0);
	musb_writel(CAST & rx->currDescPtr, 0, 0);
	musb_writel(CAST & rx->currBuffPtr, 0, 0);
	musb_writel(CAST & rx->pktLength, 0, 0);
	musb_writel(CAST & rx->byteCount, 0, 0);
}

static void __init cppi_pool_init(struct cppi *cppi, struct cppi_channel *c)
{
	int j;

	/* initialize channel fields */
	c->activeQueueHead = NULL;
	c->activeQueueTail = NULL;
	c->lastHwBDProcessed = NULL;
	c->Channel.bStatus = MGC_DMA_STATUS_UNKNOWN;
	c->pController = cppi;
	c->bLastModeRndis = 0;
	c->Channel.pPrivateData = c;
	c->bdPoolHead = NULL;

	/* build the BD Free list for the channel */
	for (j = 0; j < NUM_TXCHAN_BD + 1; j++) {
		struct cppi_descriptor *bd;
		dma_addr_t dma;

		bd = dma_pool_alloc(cppi->pool, SLAB_KERNEL, &dma);
		bd->dma = dma;
		cppi_bd_free(c, bd);
	}
}

static int cppi_channel_abort(struct dma_channel *);

static void cppi_pool_free(struct cppi_channel *c)
{
	struct cppi *cppi = c->pController;
	struct cppi_descriptor *bd;

	(void)cppi_channel_abort(&c->Channel);
	c->Channel.bStatus = MGC_DMA_STATUS_UNKNOWN;
	c->pController = NULL;

	/* free all its bds */
	bd = c->lastHwBDProcessed;
	do {
		if (bd)
			dma_pool_free(cppi->pool, bd, bd->dma);
		bd = cppi_bd_alloc(c);
	} while (bd);
	c->lastHwBDProcessed = NULL;
}

static u8 __init cppi_controller_start(void *pPrivateData)
{
	struct cppi *pController = pPrivateData;
	void *__iomem regBase;
	int i;

	/* do whatever is necessary to start controller */
	for (i = 0; i < ARRAY_SIZE(pController->txCppi); i++) {
		pController->txCppi[i].bTransmit = TRUE;
		pController->txCppi[i].chNo = i;
	}
	for (i = 0; i < ARRAY_SIZE(pController->rxCppi); i++) {
		pController->rxCppi[i].bTransmit = FALSE;
		pController->rxCppi[i].chNo = i;
	}

	/* setup BD list on a per channel basis */
	for (i = 0; i < ARRAY_SIZE(pController->txCppi); i++)
		cppi_pool_init(pController, pController->txCppi + i);
	for (i = 0; i < ARRAY_SIZE(pController->rxCppi); i++)
		cppi_pool_init(pController, pController->rxCppi + i);

	/* Do Necessary configuartion in H/w to get started */
	regBase = pController->pCoreBase - DAVINCI_BASE_OFFSET;

	INIT_LIST_HEAD(&pController->tx_complete);

	/* initialise tx/rx channel head pointers to zero */
	for (i = 0; i < ARRAY_SIZE(pController->txCppi); i++) {
		struct cppi_channel *txChannel = pController->txCppi + i;
		struct cppi_tx_stateram *__iomem txState;

		INIT_LIST_HEAD(&txChannel->tx_complete);

		txState = regBase + DAVINCI_TXCPPI_STATERAM_OFFSET(i);
		txChannel->stateRam = txState;
		/* zero out entire state RAM entry for the channel */
		txState->headPtr = 0;
		txState->sopDescPtr = 0;
		txState->currDescPtr = 0;
		txState->currBuffPtr = 0;
		txState->flags = 0;
		txState->remLength = 0;
		/*txState->dummy = 0; */
		txState->completionPtr = 0;

	}
	for (i = 0; i < ARRAY_SIZE(pController->rxCppi); i++) {
		struct cppi_channel *rxChannel = pController->rxCppi + i;
		struct cppi_rx_stateram *__iomem rxState;

		INIT_LIST_HEAD(&rxChannel->tx_complete);

		rxState = regBase + DAVINCI_RXCPPI_STATERAM_OFFSET(i);
		rxChannel->stateRam = rxState;
		cppi_reset_rx(rxChannel->stateRam);
	}

	/* enable individual cppi channels */
	musb_writel(regBase, DAVINCI_TXCPPI_INTENAB_REG,
		    DAVINCI_DMA_ALL_CHANNELS_ENABLE);
	musb_writel(regBase, DAVINCI_RXCPPI_INTENAB_REG,
		    DAVINCI_DMA_ALL_CHANNELS_ENABLE);

	/* enable tx/rx CPPI control */
	musb_writel(regBase, DAVINCI_TXCPPI_CTRL_REG, DAVINCI_DMA_CTRL_ENABLE);
	musb_writel(regBase, DAVINCI_RXCPPI_CTRL_REG, DAVINCI_DMA_CTRL_ENABLE);

	/* disable RNDIS mode */
	musb_writel(regBase, DAVINCI_AUTOREQ_REG, 0);

	return TRUE;
}

/*
 *  Stop Dma controller
 *
 *  De-Init the Dma Controller as necessary.
 */

static u8 cppi_controller_stop(void *pPrivateData)
{
	struct cppi *pController = pPrivateData;
	void __iomem *regBase;
	int i;

	regBase = pController->pCoreBase - DAVINCI_BASE_OFFSET;
	/* DISABLE INDIVIDUAL CHANNEL Interrupts */
	musb_writel(regBase, DAVINCI_TXCPPI_INTCLR_REG,
		    DAVINCI_DMA_ALL_CHANNELS_ENABLE);
	musb_writel(regBase, DAVINCI_RXCPPI_INTCLR_REG,
		    DAVINCI_DMA_ALL_CHANNELS_ENABLE);

	DBG(1, "Tearing down RX and TX Channels\n");
	for (i = 0; i < ARRAY_SIZE(pController->txCppi); i++) {
		/* FIXME restructure of txdma to use bds like rxdma */
		pController->txCppi[i].lastHwBDProcessed = NULL;
		cppi_pool_free(pController->txCppi + i);
	}
	for (i = 0; i < ARRAY_SIZE(pController->rxCppi); i++)
		cppi_pool_free(pController->rxCppi + i);

	/* in Tx Case proper teardown is supported. We resort to disabling
	 * Tx/Rx CPPI after cleanup of Tx channels. Before TX teardown is
	 * complete TX CPPI cannot be disabled.
	 */
	/*disable tx/rx cppi */
	musb_writel(regBase, DAVINCI_TXCPPI_CTRL_REG, DAVINCI_DMA_CTRL_DISABLE);
	musb_writel(regBase, DAVINCI_RXCPPI_CTRL_REG, DAVINCI_DMA_CTRL_DISABLE);

	return TRUE;
}

/*
 * Allocate a CPPI Channel for DMA.  With CPPI, channels are bound to
 * each transfer direction of a non-control endpoint, so allocating
 * (and deallocating) is mostly a way to notice bad housekeeping on
 * the software side.  We assume the irqs are always active.
 */
static struct dma_channel *cppi_channel_allocate(void *pPrivateData,
						 u8 bLocalEnd, u8 bTransmit,
						 u8 bProtocol,
						 u16 wMaxPacketSize)
{
	struct cppi *pController = pPrivateData;
	u8 chNum;
	struct cppi_channel *otgCh;
	struct musb *pThis = pController->musb;
	struct musb_hw_ep *ep = &pThis->aLocalEnd[(bTransmit) ? bLocalEnd *
					2 - 1 : bLocalEnd * 2];

	/* remember bLocalEnd: 1..Max_EndPt, and cppi ChNum:0..Max_EndPt-1 */
	chNum = bLocalEnd - 1;

	/* as of now, just return the corresponding CPPI Channel Handle */
	if (bTransmit) {
		if (bLocalEnd > ARRAY_SIZE(pController->txCppi)) {
			DBG(1, "no %cX DMA channel for ep%d\n", 'T', bLocalEnd);
			return NULL;
		}
		otgCh = pController->txCppi + chNum;
	} else {
		if (bLocalEnd > ARRAY_SIZE(pController->rxCppi)) {
			DBG(1, "no %cX DMA channel for ep%d\n", 'R', bLocalEnd);
			return NULL;
		}
		otgCh = pController->rxCppi + chNum;
	}

	/* REVISIT make this an error later once the same driver code works
	 * with the Mentor DMA engine too
	 */
	if (otgCh->pEndPt)
		DBG(6, "re-allocating DMA%d %cX channel %p\n",
		    chNum, bTransmit ? 'T' : 'R', otgCh);
	otgCh->pEndPt = ep;
	otgCh->Channel.bStatus = MGC_DMA_STATUS_FREE;

	DBG(4, "Allocate CPPI%d %cX\n", chNum, bTransmit ? 'T' : 'R');
	otgCh->Channel.pPrivateData = otgCh;
	return &otgCh->Channel;
}

/* Release a CPPI Channel.  */
static void cppi_channel_release(struct dma_channel *channel)
{
	struct cppi_channel *c;

	/* REVISIT:  for paranoia, check state and abort if needed... */

	c = container_of(channel, struct cppi_channel, Channel);
	if (!c->pEndPt)
		DBG(1, "releasing idle DMA channel %p\n", c);

	/* but for now, not its IRQ */
	c->pEndPt = NULL;
	channel->bStatus = MGC_DMA_STATUS_UNKNOWN;
}

/* Context: controller irqlocked */
static void cppi_dump_rx(int level, struct cppi_channel *c, const char *tag)
{
	void *__iomem base = c->pController->pCoreBase;

	MGC_SelectEnd(base, c->chNo + 1);

	DBG(level, "RX DMA%d%s: %d left, csr %04x, " "%08x H%08x S%08x C%08x, " "B%08x L%08x %08x .. %08x" "\n", c->chNo, tag, musb_readl(base - DAVINCI_BASE_OFFSET, DAVINCI_RXCPPI_BUFCNT0_REG + 4 * c->chNo), MGC_ReadCsr16(base, MGC_O_HDRC_RXCSR, c->chNo + 1), musb_readl(c->stateRam, 0 * 4),	/* buf offset */
	    musb_readl(c->stateRam, 1 * 4),	/* head ptr */
	    musb_readl(c->stateRam, 2 * 4),	/* sop bd */
	    musb_readl(c->stateRam, 3 * 4),	/* current bd */
	    musb_readl(c->stateRam, 4 * 4),	/* current buf */
	    musb_readl(c->stateRam, 5 * 4),	/* pkt len */
	    musb_readl(c->stateRam, 6 * 4),	/* byte cnt */
	    musb_readl(c->stateRam, 7 * 4)	/* completion */
	    );
}

/* Context: controller irqlocked */
static void cppi_dump_tx(int level, struct cppi_channel *c, const char *tag)
{
	void *__iomem base = c->pController->pCoreBase;

	MGC_SelectEnd(base, c->chNo + 1);

	DBG(level, "TX DMA%d%s: csr %04x, " "H%08x S%08x C%08x %08x, " "F%08x L%08x .. %08x" "\n", c->chNo, tag, MGC_ReadCsr16(base, MGC_O_HDRC_TXCSR, c->chNo + 1), musb_readl(c->stateRam, 0 * 4),	/* head ptr */
	    musb_readl(c->stateRam, 1 * 4),	/* sop bd */
	    musb_readl(c->stateRam, 2 * 4),	/* current bd */
	    musb_readl(c->stateRam, 3 * 4),	/* buf offset */
	    musb_readl(c->stateRam, 4 * 4),	/* flags */
	    musb_readl(c->stateRam, 5 * 4),	/* len */
	    // dummy/unused word 6
	    musb_readl(c->stateRam, 7 * 4)	/* completion */
	    );
}

/* Context: controller irqlocked */
static inline void
cppi_rndis_update(struct cppi_channel *c, int is_rx,
		  void *__iomem tibase, int is_rndis)
{
	/* we may need to change the rndis flag for this cppi channel */
	if (c->bLastModeRndis != is_rndis) {
		u32 regVal = musb_readl(tibase, DAVINCI_RNDIS_REG);
		u32 temp = 1 << (c->chNo);

		if (is_rx)
			temp <<= 16;
		if (is_rndis)
			regVal |= temp;
		else
			regVal &= ~temp;
		musb_writel(tibase, DAVINCI_RNDIS_REG, regVal);
		c->bLastModeRndis = is_rndis;
	}
}

static void cppi_dump_rxbd(const char *tag, struct cppi_descriptor *bd)
{
	pr_debug("RXBD/%s %08x: "
		 "nxt %08x buf %08x off.blen %08x opt.plen %08x\n",
		 tag, bd->dma,
		 bd->hNext, bd->buffPtr, bd->bOffBLen, bd->hOptions);
}

static void cppi_dump_rxq(int level, const char *tag, struct cppi_channel *rx)
{
#if MUSB_DEBUG > 0
	struct cppi_descriptor *bd;

	if (!_dbg_level(level))
		return;
	cppi_dump_rx(level, rx, tag);
	if (rx->lastHwBDProcessed)
		cppi_dump_rxbd("last", rx->lastHwBDProcessed);
	for (bd = rx->activeQueueHead; bd; bd = bd->next)
		cppi_dump_rxbd("active", bd);
#endif
}

static inline int cppi_autoreq_update(struct cppi_channel *rx,
				      void *__iomem tibase, int shortpkt,
				      u8 rndis, signed n_bds, u8 startreq,
				      u8 endreq)
{
	u32 tmp, val;

	/* assert(is_host_active(musb)) */

	/* start from "AutoReq never" */
	tmp = musb_readl(tibase, DAVINCI_AUTOREQ_REG);
	val = tmp & ~((0x3) << (rx->chNo * 2));
	/* HCD arranged reqpkt for packet #1.  we arrange int
	 * for all but the last one, maybe in two segments.
	 */

	if (shortpkt && rndis) {
		val = (val | ((0x01) << (rx->chNo * 2)));
		rx->autoReq = 0x01;
	} else if (shortpkt && !rndis) {
		rx->autoReq = 0x00;
	} else if ((!shortpkt) && (n_bds > 2)) {	// there might be shortpacket not request we might convert in to RNDIS mode
		val = (val | ((0x03) << (rx->chNo * 2)));
		rx->autoReq = 0x03;
		if (endreq)
			n_bds -= 2;
	} else {
		rx->autoReq = 0;
	}

	if (val != tmp) {
		musb_writel(tibase, DAVINCI_AUTOREQ_REG, val);
	}

	return n_bds;
}

/* Buffer enqueuing Logic:
 *
 *  - RX builds new queues each time, to help handle routine "early
 *    termination" cases (faults, including errors and short reads)
 *    more correctly.
 *
 *  - for now, TX reuses the same queue of BDs every time
 *
 * REVISIT long term, we want a normal dynamic model.
 * ... the goal will be to append to the
 * existing queue, processing completed "dma buffers" (segments) on the fly.
 *
 * Otherwise we force an IRQ latency between requests, which slows us a lot
 * (especially in "transparent" dma).  Unfortunately that model seems to be
 * inherent in the DMA model from the Mentor code, except in the rare case
 * of transfers big enough (~128+ KB) that we could append "middle" segments
 * in the TX paths.  (RX can't do this, see below.)
 *
 * That's true even in the CPPI- friendly iso case, where most urbs have
 * several small segments provided in a group and where the "packet at a time"
 * "transparent" DMA model is always correct, even on the RX side.
 */

/*
 * CPPI TX:
 * ========
 * TX is a lot more reasonable than RX; it doesn't need to run in
 * irq-per-packet mode very often.  RNDIS mode seems to behave too
 * (other how it handles the exactly-N-packets case).  Building a
 * txdma queue with multiple requests (urb or usb_request) looks
 * like it would work ... but fault handling still needs testing.
 */
static void
cppi_next_tx_segment(struct musb *musb, struct cppi_channel *tx, int rndis)
{
	unsigned maxpacket = tx->pktSize;
	size_t length = tx->transferSize - tx->currOffset;
	struct cppi_descriptor *bd;
	unsigned n_bds;
	unsigned i, iso = (tx->rxMode == 2)? 1 : 0;
	int	iso_desc = tx->iso_desc;
	struct cppi_tx_stateram *txState = tx->stateRam;

	/* TX can use the CPPI "rndis" mode, where we can probably fit this
	 * transfer in one BD and one IRQ; though some common cases (like
	 * packet length not being n*64 bytes) can't work that way.
	 *
	 * To cppi hardware (but not the RNDIS protocol!) RNDIS is mostly a
	 * "short packet termination" mode.  So the only time we would NOT
	 * want to use it is to avoid sending spurious zero length packets,
	 * or when hardware constraints prevent it.
	 */

#if 0				// Disable RNDIS on both Tx & Rx
	if (!rndis && (length % maxpacket) != 0)
		rndis = 1;
	if (rndis && (length > 0xffff || (maxpacket & 0x3f) != 0
		      /* "undocumented" rndis mode constraint on txlen */
		      || (length & 0x3f) != 0))
#endif
		rndis = 0;

	if (iso) {
		n_bds = (tx->pEndPt->num_iso_desc - iso_desc)
				>= NUM_TXCHAN_BD ?  NUM_TXCHAN_BD :
				tx->pEndPt->num_iso_desc - iso_desc;
		tx->currOffset = tx->pEndPt->iso_desc[iso_desc].offset;
	} else if (!length) {
		rndis = 0;
		n_bds = 1;
	} else if (rndis) {
		maxpacket = length;
		n_bds = 1;
	} else {
		n_bds = length / maxpacket;
		if ((length % maxpacket) || (tx->rxMode == 1))
			n_bds++;

		n_bds = min(n_bds, (unsigned)NUM_TXCHAN_BD);
		length = min(n_bds * maxpacket, length);
	}

	DBG(4, "TX DMA%d, pktSz %d %s bds %d dma 0x%x len %u\n",
	    tx->chNo,
	    maxpacket,
	    rndis ? "rndis" : "transparent",
	    n_bds, tx->startAddr + tx->currOffset, length);

	cppi_rndis_update(tx, 0, musb->ctrl_base, rndis);

	/* assuming here that DmaProgramChannel is called during
	 * transfer initiation ... current code maintains state
	 * for one outstanding request only (no queues, not even
	 * the implicit ones of an iso urb).
	 */

	bd = tx->bdPoolHead;
	tx->activeQueueHead = tx->bdPoolHead;
	tx->lastHwBDProcessed = NULL;

	/* Prepare queue of BDs first, then hand it to hardware.
	 * All BDs except maybe the last should be of full packet
	 * size; for RNDIS there _is_ only that last packet.
	 */
	for (i = 0; i < n_bds;) {
		if (++i < n_bds && bd->next)
			bd->hNext = bd->next->dma;
		else
			bd->hNext = 0;

		bd->buffPtr = tx->startAddr + tx->currOffset;

		if (iso) {
			bd->bOffBLen = tx->pEndPt->iso_desc[iso_desc].length;
			bd->hOptions = CPPI_SOP_SET|CPPI_EOP_SET|CPPI_OWN_SET;
			if (tx->pEndPt->iso_desc[iso_desc].length == 0)
				bd->hOptions |= CPPI_ZERO_SET|1;
			else
				bd->hOptions |= tx->pEndPt->
						iso_desc[iso_desc].length;

			tx->currOffset =  tx->pEndPt->
						iso_desc[++iso_desc].offset;
		} else if ((tx->currOffset + maxpacket)
		    <= tx->transferSize) {
			tx->currOffset += maxpacket;
			bd->bOffBLen = maxpacket;
			bd->hOptions = CPPI_SOP_SET | CPPI_EOP_SET
			    | CPPI_OWN_SET | maxpacket;
		} else {
			/* only this one may be a partial USB Packet */
			u32 buffSz;

			buffSz = tx->transferSize - tx->currOffset;
			tx->currOffset = tx->transferSize;
			bd->bOffBLen = (buffSz) ? buffSz : 1;

			bd->hOptions = CPPI_SOP_SET | CPPI_EOP_SET
			    | CPPI_OWN_SET | (buffSz ? buffSz : 1);
			if (buffSz == 0)
				bd->hOptions |= CPPI_ZERO_SET;
		}

		DBG(5, "TXBD %p: nxt %08x buf %08x len %04x opt %08x\n",
		    bd, bd->hNext, bd->buffPtr, bd->bOffBLen, bd->hOptions);

		/* update the last BD enqueued to the list */
		tx->activeQueueTail = bd;
		bd = bd->next;
	}

	/* BDs live in DMA-coherent memory, but writes might be pending */

	/* Write to the HeadPtr in StateRam to trigger */
	txState->headPtr = (u32) tx->bdPoolHead->dma;

	cppi_dump_tx(5, tx, "/S");
}

/*
 * CPPI RX:
 * ========
 * Consider a 1KB bulk RX buffer in two scenarios:  (a) it's fed two 300 byte
 * packets back-to-back, and (b) it's fed two 512 byte packets back-to-back.
 * (Full speed transfers have similar scenarios.)
 *
 * The correct behavior for Linux is that (a) fills the buffer with 300 bytes,
 * and the next packet goes into a buffer that's queued later; while (b) fills
 * the buffer with 1024 bytes.  How to do that with CPPI?
 *
 * - CPPI RX queues in "rndis" mode -- one single BD -- handle (a) correctly,
 *   but (b) loses _badly_ because nothing (!) happens when that second packet
 *   fills the buffer, much less when a third one arrives.  (Which makes this
 *   not a "true" RNDIS mode.  In the RNDIS protocol short-packet termination
 *   is optional, and it's fine if senders pad messages out to end-of-buffer.)
 *
 * - CPPI RX queues in "transparent" mode -- two BDs with 512 bytes each -- have
 *   converse problems:  (b) is handled correctly, but (a) loses badly.  CPPI RX
 *   ignores SOP/EOP markings and processes both of those BDs; so both packets
 *   are loaded into the buffer (with a 212 byte gap between them), and the next
 *   buffer queued will NOT get its 300 bytes of data. (It seems like SOP/EOP
 *   are intended as outputs for RX queues, not inputs...)
 *
 * - A variant of "transparent" mode -- one BD at a time -- is the only way to
 *   reliably make both cases work, with software handling both cases correctly
 *   and at the significant penalty of needing an IRQ per packet.  (The lack of
 *   I/O overlap can be slightly ameliorated by enabling double buffering.)
 *
 * So how to get rid of IRQ-per-packet?  The transparent multi-BD case could
 * be used in special cases like mass storage, which sets URB_SHORT_NOT_OK
 * (or maybe its peripheral side counterpart) to flag (a) scenarios as errors
 * with guaranteed driver level fault recovery and scrubbing out what's left
 * of that garbaged datastream.
 *
 * But there seems to be no way to identify the cases where CPPI RNDIS mode
 * is appropriate -- which do NOT include the RNDIS driver, but do include
 * the CDC Ethernet driver! -- and the documentation is incomplete/wrong.
 * So we can't _ever_ use RX RNDIS mode.
 *
 * Leaving only "transparent" mode; we avoid multi-bd modes in almost all
 * cases other than mass storage class.  Otherwise e're correct but slow,
 * since CPPI penalizes our need for a "true RNDIS" default mode.
 */

/**
 * cppi_next_rx_segment - dma read for the next chunk of a buffer
 * @musb: the controller
 * @rx: dma channel
 * @onepacket: true unless caller treats short reads as errors, and
 * 	performs fault recovery above usbcore.
 * Context: controller irqlocked
 *
 * See above notes about why we can't use multi-BD RX queues except in
 * rare cases (mass storage class), and can never use the hardware "rndis"
 * mode (since it's not a "true" RNDIS mode).
 *
 * It's ESSENTIAL that callers specify "onepacket" mode unless they kick in
 * code to recover from corrupted datastreams after each short transfer.
 */
static void
cppi_next_rx_segment(struct musb *musb, struct cppi_channel *rx, int shortpkt)
{
	unsigned maxpacket = rx->pktSize;
	dma_addr_t addr = rx->startAddr + rx->currOffset;
	size_t length = rx->transferSize - rx->currOffset;
	struct cppi_descriptor *bd, *tail;
	signed n_bds;
	signed i;
	void *__iomem tibase = musb->ctrl_base;
	u8 rndis = 0;
	int max_bd = 0;
	unsigned iso = (rx->rxMode == 2)? 1 : 0;
	int     iso_desc = rx->iso_desc;

	if (((rx->rxMode == 1) && ((maxpacket & 0x3f) == 0)
		/* REVISIT MAXPACKET CHECK!!!! */
	     && ((length & 0x3f) == 0))) {
		rndis = 1;
		max_bd = 65536;	// MAX buffer aize per RNDIS BD is 64 K
	} else {
		rndis = 0;
		max_bd = maxpacket;
	}
#ifdef CONFIG_ARCH_DAVINCI
	/* Do not use RNDIS dma for DaVinci */
	rndis = 0;
	max_bd = maxpacket;
#endif
	if (iso)
		max_bd = rx->pEndPt->iso_desc[iso_desc].length;

	n_bds = length / max_bd;
	if (length % max_bd) {
		n_bds++;
	}

	n_bds = min(n_bds, (signed)NUM_RXCHAN_BD);
	if (n_bds == NUM_RXCHAN_BD)
		length = min(length, (size_t) (n_bds * max_bd));

	cppi_rndis_update(rx, 1, musb->ctrl_base, rndis);

	/* In host mode, autorequest logic can generate some IN tokens; it's
	 * tricky since we can't leave REQPKT set in RXCSR after the transfer
	 * finishes. So:  multipacket transfers involve two or more segments.
	 * And always at least two IRQs.
	 */
	if (is_host_active(musb))
		n_bds = cppi_autoreq_update(rx, tibase, shortpkt, rndis, n_bds,
					    (u8) ! rx->currOffset,
					    (u8) (rx->transferSize <=
						  (rx->currOffset + length)
					    ));

	DBG(4, "RX DMA%d seg, maxp %d %spacket bds %d (cnt %d) "
	    "dma 0x%x len %u/%u/%u\n",
	    rx->chNo, max_bd,
	    shortpkt ? "one" : "multi",
	    n_bds,
	    musb_readl(tibase, DAVINCI_RXCPPI_BUFCNT0_REG + (rx->chNo * 4))
	    & 0xffff, addr, length, rx->actualLen, rx->transferSize);

	/* only queue one segment at a time, since the hardware prevents
	 * correct queue shutdown after unexpected short packets
	 */
	bd = cppi_bd_alloc(rx);
	rx->activeQueueHead = bd;

	/* Build BDs for all packets in this segment */
	for (i = 0, tail = NULL; bd && i < n_bds; i++, tail = bd) {
		u32 buffSz;

		if (i) {
			bd = cppi_bd_alloc(rx);
			if (!bd)
				break;
			tail->next = bd;
			tail->hNext = bd->dma;
		}
		bd->hNext = 0;

		/* all but the last packet will be maxpacket size */
		if (max_bd < length)
			buffSz = max_bd;
		else
			buffSz = length;

		bd->buffPtr = addr;
		addr += buffSz;
		rx->currOffset += buffSz;

		bd->bOffBLen = (0 /*offset */  << 16) + buffSz;
		bd->enqBuffLen = buffSz;

		bd->hOptions = CPPI_OWN_SET | (i == 0 ? length : 0);
		length -= buffSz;
	}

	/* we always expect at least one reusable BD! */
	if (!tail) {
		WARN("rx dma%d -- no BDs? need %d\n", rx->chNo, n_bds);
		return;
	} else if (i < n_bds)
		WARN("rx dma%d -- only %d of %d BDs\n", rx->chNo, i, n_bds);

	tail->next = NULL;
	tail->hNext = 0;

	bd = rx->activeQueueHead;
	rx->activeQueueTail = tail;

	/* short reads and other faults should terminate this entire
	 * dma segment.  we want one "dma packet" per dma segment, not
	 * one per USB packet, terminating the whole queue at once...
	 * NOTE that current hardware seems to ignore SOP and EOP.
	 */
	if (MGC_DebugLevel >= 5) {
		struct cppi_descriptor *d;

		for (d = rx->activeQueueHead; d; d = d->next)
			cppi_dump_rxbd("S", d);
	}
	rx->lastHwBDProcessed = NULL;
	/* BDs live in DMA-coherent memory, but writes might be pending */

	/* REVISIT specs say to write this AFTER the BUFCNT register
	 * below ... but that loses badly.
	 */
	musb_writel(rx->stateRam, 4, bd->dma);

	/* bufferCount must be at least 3, and zeroes on completion
	 * unless it underflows below zero, or stops at two, or keeps
	 * growing ... grr.
	 */
	i = musb_readl(tibase, DAVINCI_RXCPPI_BUFCNT0_REG + (rx->chNo * 4))
	    & 0xffff;

	if (n_bds > (i - 2)) {
		musb_writel(tibase,
			    DAVINCI_RXCPPI_BUFCNT0_REG + (rx->chNo * 4),
			    n_bds - i + 2);
	}

	cppi_dump_rx(4, rx, "/S");
}

/**
 * cppi_channel_program - program channel for data transfer
 * @pChannel: the channel
 * @wPacketSz: max packet size
 * @mode: For RX, 1 unless the usb protocol driver promised to treat
 * 	all short reads as errors and kick in high level fault recovery.
 * 	For TX, 0 unless the protocol driver _requires_ short-packet
 * 	termination mode.
 * @dma_addr: dma address of buffer
 * @dwLength: length of buffer
 * Context: controller irqlocked
 */
static u8 cppi_channel_program(struct dma_channel *pChannel,
			       u16 wPacketSz, u8 mode,
			       dma_addr_t dma_addr, u32 dwLength)
{
	struct cppi_channel *otgChannel = pChannel->pPrivateData;
	struct cppi *pController = otgChannel->pController;
	struct musb *musb = pController->musb;

	switch (pChannel->bStatus) {
	case MGC_DMA_STATUS_BUS_ABORT:
	case MGC_DMA_STATUS_CORE_ABORT:
		/* fault irq handler should have handled cleanup */
		WARN("%cX DMA%d not cleaned up after abort!\n",
		     otgChannel->bTransmit ? 'T' : 'R', otgChannel->chNo);
		//WARN_ON(1);
		break;
	case MGC_DMA_STATUS_BUSY:
		WARN("program active channel?  %cX DMA%d\n",
		     otgChannel->bTransmit ? 'T' : 'R', otgChannel->chNo);
		//WARN_ON(1);
		break;
	case MGC_DMA_STATUS_UNKNOWN:
		DBG(1, "%cX DMA%d not allocated!\n",
		    otgChannel->bTransmit ? 'T' : 'R', otgChannel->chNo);
		/* FALLTHROUGH */
	case MGC_DMA_STATUS_FREE:
		break;
	}

	pChannel->bStatus = MGC_DMA_STATUS_BUSY;

	/* set transfer parameters, then queue up its first segment */
	otgChannel->startAddr = dma_addr;
	otgChannel->currOffset = 0;
	otgChannel->pktSize = wPacketSz;
	otgChannel->actualLen = 0;
	otgChannel->transferSize = dwLength;
	otgChannel->rxMode = mode;
	otgChannel->reqcomplete = 0;
	otgChannel->autoReq = 0;
	otgChannel->iso_desc = 0;

	/* TX channel? or RX? */
	if (otgChannel->bTransmit)
		cppi_next_tx_segment(musb, otgChannel, mode);
	else
		cppi_next_rx_segment(musb, otgChannel, mode);

	return TRUE;
}

static int cppi_rx_scan(struct cppi *cppi, unsigned ch, u8 abort)
{
	struct cppi_channel *rx = &cppi->rxCppi[ch];
	struct cppi_rx_stateram *state = rx->stateRam;
	struct cppi_descriptor *bd;
	struct cppi_descriptor *last = rx->lastHwBDProcessed;
	int completed = 0;
	dma_addr_t safe2ack;
	u32 csr;

	cppi_dump_rx(6, rx, "/K");

	if (abort) {
		safe2ack = musb_readl(CAST & state->completionPtr, 0);

		if ((last == NULL) || (safe2ack == last->dma)) {
			if (last == NULL)
				last = rx->activeQueueHead;

			goto free;
		}
	}

	bd = last ? last->next : rx->activeQueueHead;

	do {
		safe2ack = musb_readl(CAST & state->completionPtr, 0);
		if (completed)
			break;
		do {
			u16 len;

			DBG(5, "C/RXBD %08x: nxt %08x buf %08x "
			    "off.len %08x opt.len %08x (%d)\n",
			    bd->dma, bd->hNext, bd->buffPtr,
			    bd->bOffBLen, bd->hOptions, rx->actualLen);

			/* actual packet received length */
			len = bd->bOffBLen & CPPI_RECV_PKTLEN_MASK;
			if (bd->hOptions & CPPI_ZERO_SET)
				len = 0;

			if (bd->hNext == 0)
				completed = 1;

			if ((len < bd->enqBuffLen) && (rx->rxMode != 2)) {
				rx->reqcomplete = 1;
				completed = 1;
				DBG(3, "rx short %d/%d (%d)\n",
				    len, bd->enqBuffLen, rx->actualLen);
			}

			if (rx->rxMode == 2) {
				rx->pEndPt->iso_desc[rx->iso_desc].length = len;
				if (completed) {
					rx->reqcomplete = 1;
					rx->iso_desc = 0;
				} else
					rx->iso_desc++;
			}

			rx->actualLen += len;
			cppi_bd_free(rx, last);
			last = bd;
			if (safe2ack == bd->dma) {
				bd = bd->next;
				break;
			}
			bd = bd->next;
		} while (!completed);
	} while (musb_readl(CAST & state->completionPtr, 0) != safe2ack);

	if (is_host_active(rx->pController->musb) && (!abort) &&
	    (rx->autoReq == 0) &&
	    ((!completed) || (completed && (rx->reqcomplete == 0) &&
		(rx->actualLen != rx->transferSize)))) {

		csr = MGC_ReadCsr16(rx->pController->pCoreBase,
				    MGC_O_HDRC_RXCSR, rx->chNo + 1);
			csr |= MGC_M_RXCSR_H_REQPKT;
			MGC_WriteCsr16(rx->pController->pCoreBase,
				       MGC_O_HDRC_RXCSR, rx->chNo + 1, csr);
		}

	rx->lastHwBDProcessed = last;
	musb_writel(CAST & state->completionPtr, 0, safe2ack);
free:
	if (completed || abort) {

		/* Flush BD's not consumed */
		while (last != NULL) {
			bd = last;
			last = last->next;
			cppi_bd_free(rx, bd);
		}
		if (abort) {
			safe2ack = musb_readl(CAST & state->completionPtr, 0);
			musb_writel(CAST & state->completionPtr, 0, safe2ack);
		}
		rx->lastHwBDProcessed = NULL;
		rx->activeQueueHead = NULL;
		rx->activeQueueTail = NULL;
	}

	cppi_dump_rx(6, rx, completed ? "/completed" : "/cleaned");
	return completed;
}

void cppi_completion(struct musb *pThis, u32 rx, u32 tx)
{
	void *__iomem regBase;
	int i, chanNum;
	u8 bReqComplete;
	struct cppi *cppi;
	struct cppi_descriptor *bdPtr;

	cppi = container_of(pThis->pDmaController, struct cppi, Controller);
	regBase = cppi->pCoreBase - DAVINCI_BASE_OFFSET;

	/* process TX channels */
	for (chanNum = 0; tx; tx = tx >> 1, chanNum++) {
		if (tx & 1) {
			struct cppi_channel *txChannel;
			struct cppi_tx_stateram *txState;
			u32 safe2ack = 0;

			txChannel = cppi->txCppi + chanNum;
			txState = txChannel->stateRam;

			/* FIXME  need a cppi_tx_scan() routine, which
			 * can also be called from abort code
			 */
			cppi_dump_tx(5, txChannel, "/E");

			bdPtr = txChannel->activeQueueHead;
			i = 0;
			bReqComplete = 0;

			do {
				safe2ack = txState->completionPtr;
				do {
					DBG(5,
					    "C/TXBD %p n %x b %x off %x opt %x\n",
					    bdPtr, bdPtr->hNext, bdPtr->buffPtr,
					    bdPtr->bOffBLen, bdPtr->hOptions);

					txChannel->actualLen += (u16) (bdPtr->
						   bOffBLen &
						   CPPI_BUFFER_LEN_MASK);
					if (txChannel->rxMode == 2) {
						txChannel->pEndPt->
						iso_desc[txChannel->iso_desc].
						status = 0;
						txChannel->iso_desc++;
					}
					if (bdPtr->dma == safe2ack) {
						if (bdPtr->hOptions &
							CPPI_ZERO_SET)
							txChannel->actualLen -=
							1;
						if (bdPtr->hNext == 0)
							bReqComplete = 1;
						txChannel->lastHwBDProcessed =
						    bdPtr;
						bdPtr = bdPtr->next;
						break;
					}
					bdPtr = bdPtr->next;
				} while (1);
			} while (txState->completionPtr != safe2ack);

			txState->completionPtr = txChannel->lastHwBDProcessed->
			    dma;

			/* on end of segment, maybe go to next one */
			if (bReqComplete) {
				cppi_dump_tx(4, txChannel, "/complete");

				/* transfer more, or report completion */
				if (txChannel->actualLen
				    >= txChannel->transferSize) {
					txChannel->activeQueueHead = NULL;
					txChannel->activeQueueTail = NULL;
					txChannel->Channel.bStatus =
					    MGC_DMA_STATUS_FREE;
					txChannel->Channel.dwActualLength =
					    txChannel->actualLen;
					cppi->dma_completed(pThis, chanNum + 1,
								    1);

				} else {
					/* Bigger transfer than we could fit in
					 * that first batch of descriptors...
					 */
					cppi_next_tx_segment(pThis,
							txChannel,
							txChannel->rxMode);
				}
			} else
				txChannel->activeQueueHead = bdPtr;
		}
	}

	/* Start processing the RX block */
	for (chanNum = 0; rx; rx = rx >> 1, chanNum++) {

		if (rx & 1) {
			struct cppi_channel *rxChannel;

			rxChannel = cppi->rxCppi + chanNum;
			/* There is a race condition between abort channel and
			 * on going traffic.  Ensure that pending interrupts
			 * (that were raised during clean up but was held due
			 * to interrupt block) are acknowledged correctly by
			 * adressing those interrupts as abort (from interrupt
			 * context, as we already handled the user initiated
			 * abort which actually lead to this race). This occurs
			 * possible due to the fact that we do not support
			 * clean RX teardown at this point (May not require this
			 * code once RX teardown is correctly supported in
			 * hardware).
			 */
			if (rxChannel->activeQueueHead)
				bReqComplete = cppi_rx_scan(cppi, chanNum, 0);
			else {
				cppi_rx_scan(cppi, chanNum, 1);
				continue;
			}

			/* let incomplete dma segments finish */
			if (!bReqComplete)
				continue;

			/* start another dma segment if needed */
			if ((rxChannel->actualLen != rxChannel->transferSize)
			    && !(rxChannel->reqcomplete)) {
				cppi_next_rx_segment(pThis, rxChannel,
						     rxChannel->rxMode);
				continue;
			}

			/* all segments completed! */
			rxChannel->Channel.bStatus = MGC_DMA_STATUS_FREE;
			rxChannel->Channel.dwActualLength =
			    rxChannel->actualLen;
			(void)cppi->dma_completed(pThis, chanNum + 1, 0);
		}
	}

	/* write to CPPI EOI register to re-enable interrupts */
	musb_writel(regBase, DAVINCI_CPPI_EOI_REG, 0);
}

/* Instantiate a software object representing a DMA controller. */
static struct dma_controller *cppi_controller_new(MGC_pfDmaChannelStatusChanged
						  dma_completed, void *musb,
						  u8 * pCoreBase)
{
	struct dma_controller *pResult = NULL;
	struct cppi *pController;

	pController = kzalloc(sizeof *pController, GFP_KERNEL);
	if (!pController)
		return NULL;

	/* Initialize the Cppi DmaController  structure */
	pController->dma_completed = dma_completed;

	pController->pCoreBase = pCoreBase;
	pController->musb = musb;
	pController->Controller.pPrivateData = pController;
	pController->Controller.pfDmaStartController = cppi_controller_start;
	pController->Controller.pfDmaStopController = cppi_controller_stop;
	pController->Controller.pfDmaAllocateChannel = cppi_channel_allocate;
	pController->Controller.pfDmaReleaseChannel = cppi_channel_release;
	pController->Controller.pfDmaProgramChannel = cppi_channel_program;
	pController->Controller.pfDmaAbortChannel = cppi_channel_abort;
	pResult = &(pController->Controller);

	/* NOTE: allocating from on-chip SRAM would give the least
	 * contention for memory access, if that ever matters here.
	 */

	/* setup BufferPool */
	pController->pool = dma_pool_create("cppi",
					    pController->musb->controller,
					    sizeof(struct cppi_descriptor),
					    CPPI_DESCRIPTOR_ALIGN, 0);
	if (!pController->pool) {
		kfree(pController);
		pResult = NULL;
	}

	return pResult;
}

/*
 *  Destroy a previously-instantiated DMA controller.
 */
static void cppi_controller_destroy(struct dma_controller *pController)
{
	struct cppi *cpController = pController->pPrivateData;

	/* assert:  caller stopped the controller first */
	dma_pool_destroy(cpController->pool);

	if (cpController) {
		cpController->Controller.pPrivateData = NULL;
		kfree(cpController);
	}

}

/*
 * Context: controller irqlocked, endpoint selected
 */
static int cppi_channel_abort(struct dma_channel *pChannel)
{
	struct cppi_channel *otgCh = pChannel->pPrivateData;
	struct cppi *pController = otgCh->pController;
	int chNum = otgCh->chNo;
	void *__iomem mbase;
	void *__iomem regBase;
	u32 regVal;

	switch (pChannel->bStatus) {
	case MGC_DMA_STATUS_BUS_ABORT:
	case MGC_DMA_STATUS_CORE_ABORT:
		/* from RX or TX fault irq handler */
	case MGC_DMA_STATUS_BUSY:
		/* the hardware needs shutting down */
		break;
	case MGC_DMA_STATUS_UNKNOWN:
		DBG(8, "%cX DMA%d not allocated\n",
		    otgCh->bTransmit ? 'T' : 'R', otgCh->chNo);
		/* FALLTHROUGH */
	case MGC_DMA_STATUS_FREE:
		return 0;
	}

	if (chNum & ~CPPI_CHNUM_BITS_MASK)
		return -EINVAL;

	if (!otgCh->bTransmit && otgCh->activeQueueHead)
		cppi_dump_rxq(4, "/abort", otgCh);

	mbase = pController->pCoreBase;
	regBase = mbase - DAVINCI_BASE_OFFSET;

	/* REVISIT should rely on caller having done this,
	 * and caller should rely on us not changing it.
	 * peripheral code is safe ... check host too.
	 */
	MGC_SelectEnd(mbase, chNum + 1);

	if (otgCh->bTransmit) {
		struct cppi_tx_stateram *__iomem txState;
		int enabled;

		/* mask interrupts raised to signal teardown complete.  */
		enabled = musb_readl(regBase, DAVINCI_TXCPPI_INTENAB_REG)
		    & (1 << otgCh->chNo);
		if (enabled)
			musb_writel(regBase, DAVINCI_TXCPPI_INTCLR_REG,
				    (1 << otgCh->chNo));

		// REVISIT put timeouts on these controller handshakes

		cppi_dump_tx(6, otgCh, " (teardown)");

		/* teardown DMA engine then usb core */
		do {
			regVal = musb_readl(regBase, DAVINCI_TXCPPI_TEAR_REG);
		} while (!(regVal & CPPI_TEAR_READY));
		musb_writel(regBase, DAVINCI_TXCPPI_TEAR_REG, chNum);

		txState = otgCh->stateRam;
		do {
			regVal = txState->completionPtr;
		} while (0xFFFFFFFC != regVal);
		txState->completionPtr = 0xFFFFFFFC;

		musb_writel(regBase, DAVINCI_CPPI_EOI_REG, 0);

		/* FIXME clean up the transfer state ... here?
		 * the completion routine should get called with
		 * an appropriate status code.
		 */

		regVal = MGC_ReadCsr16(mbase, MGC_O_HDRC_TXCSR, chNum + 1);
		regVal |= MGC_M_TXCSR_FLUSHFIFO;
		MGC_WriteCsr16(mbase, MGC_O_HDRC_TXCSR, chNum + 1, regVal);
		MGC_WriteCsr16(mbase, MGC_O_HDRC_TXCSR, chNum + 1, regVal);

		txState->headPtr = 0;
		txState->sopDescPtr = 0;
		txState->currBuffPtr = 0;
		txState->currDescPtr = 0;
		txState->flags = 0;
		txState->remLength = 0;

		/* Ensure that we clean up any Interrupt asserted
		 * 1. Write to completion Ptr value 0x1(bit 0 set)
		 *    (write back mode)
		 * 2. Write to completion Ptr value 0x0(bit 0 cleared)
		 *    (compare mode)
		 * Value written is compared(for bits 31:2) and being
		 * equal interrupt deasserted?
		 */

		/* write back mode, bit 0 set, hence completion Ptr
		 * must be updated
		 */
		txState->completionPtr = 0x1;
		/* compare mode, write back zero now */
		txState->completionPtr = 0;

		/* re-enable interrupt */
		if (enabled)
			musb_writel(regBase, DAVINCI_TXCPPI_INTENAB_REG,
				    (1 << otgCh->chNo));

		cppi_dump_tx(5, otgCh, " (done teardown)");

		/* REVISIT tx side _should_ clean up the same way
		 * as the RX side ... this does no cleanup at all!
		 */

	} else {		/* RX */

		u16 csr;

		/* NOTE: docs don't guarantee any of this works ...  we
		 * expect that if the usb core stops telling the cppi core
		 * to pull more data from it, then it'll be safe to flush
		 * current RX DMA state iff any pending fifo transfer is done.
		 */

		/* for host, ensure ReqPkt is never set again */
		if (is_host_active(otgCh->pController->musb)) {
			regVal = musb_readl(regBase, DAVINCI_AUTOREQ_REG);
			regVal &= ~((0x3) << (otgCh->chNo * 2));
			musb_writel(regBase, DAVINCI_AUTOREQ_REG, regVal);
		}

		csr = MGC_ReadCsr16(mbase, MGC_O_HDRC_RXCSR, otgCh->chNo + 1);

		/* for host, clear (just) ReqPkt at end of current packet(s) */
		if (is_host_active(otgCh->pController->musb)) {
			csr |= MGC_M_RXCSR_H_WZC_BITS;
			csr &= ~MGC_M_RXCSR_H_REQPKT;
		} else
			csr |= MGC_M_RXCSR_P_WZC_BITS;

		/* clear dma enable */
		csr &= ~(MGC_M_RXCSR_DMAENAB);
		MGC_WriteCsr16(mbase, MGC_O_HDRC_RXCSR, otgCh->chNo + 1, csr);

		/* quiesce: wait for current dma to finish (if not cleanup)
		 * we can't use bit zero of stateram->sopDescPtr since that
		 * refers to an entire "DMA packet" not just emptying the
		 * current fifo, and most segements need multiple fifos.
		 */
		if (pChannel->bStatus == MGC_DMA_STATUS_BUSY)
			udelay(50);

		/* scan the current list, reporting any data that was
		 * transferred and acking any IRQ
		 */
		cppi_rx_scan(pController, chNum, 1);
		pChannel->dwActualLength += otgCh->actualLen;

		/* clobber the existing state once it's idle
		 *
		 * NOTE:  arguably, we should also wait for all the other
		 * RX channels to quiesce (how??) and then temporarily
		 * disable RXCPPI_CTRL_REG ... but it seems that we can
		 * rely on the controller restarting from state ram, with
		 * only RXCPPI_BUFCNT state being bogus.  BUFCNT will
		 * correct itself after the next DMA transfer though.
		 *
		 * REVISIT does using rndis mode change that?
		 */
		cppi_reset_rx(otgCh->stateRam);

		/* next DMA request _should_ load cppi head ptr */

		/* ... we don't "free" that list, only mutate it in place.  */
		cppi_dump_rx(5, otgCh, " (done abort)");
	}

	pChannel->bStatus = MGC_DMA_STATUS_FREE;
	otgCh->startAddr = 0;
	otgCh->currOffset = 0;
	otgCh->transferSize = 0;
	otgCh->pktSize = 0;

	return 0;
}

/* TBD Queries:
 *
 * Power Management ... probably turn off cppi during suspend, restart;
 * check state ram?  Clocking is presumably shared with usb core.
 */
