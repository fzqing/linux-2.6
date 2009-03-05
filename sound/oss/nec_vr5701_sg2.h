/*
 * sound/oss/nec_vr5701_sg2.h
 *
 * A sound driver for NEC Electronics Corporation VR5701 SolutionGearII.
 * It works with AC97 codec.
 *
 * Author: Sergey Podstavin <spodstavin@ru.mvista.com>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#include <linux/module.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/sound.h>
#include <linux/slab.h>
#include <linux/soundcard.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/bitops.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/smp_lock.h>
#include <linux/ac97_codec.h>
#include <linux/interrupt.h>
#include <asm/hardirq.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <asm/uaccess.h>

#define         vr5701_INT_CLR         0x0
#define         vr5701_INT_STATUS	0x0
#define         vr5701_CODEC_WR        0x4
#define         vr5701_CODEC_RD        0x8
#define         vr5701_CTRL            0x18
#define         vr5701_ACLINK_CTRL     0x1c
#define         vr5701_INT_MASK        0x24

#define		vr5701_DAC1_CTRL	0x30
#define		vr5701_DAC1L		0x34
#define		vr5701_DAC1_BADDR	0x38
#define		vr5701_DAC2_CTRL	0x3c
#define		vr5701_DAC2L		0x40
#define		vr5701_DAC2_BADDR	0x44
#define		vr5701_DAC3_CTRL	0x48
#define		vr5701_DAC3L		0x4c
#define		vr5701_DAC3_BADDR	0x50

#define		vr5701_ADC1_CTRL	0x54
#define		vr5701_ADC1L		0x58
#define		vr5701_ADC1_BADDR	0x5c
#define		vr5701_ADC2_CTRL	0x60
#define		vr5701_ADC2L		0x64
#define		vr5701_ADC2_BADDR	0x68
#define		vr5701_ADC3_CTRL	0x6c
#define		vr5701_ADC3L		0x70
#define		vr5701_ADC3_BADDR	0x74

#define		vr5701_CODEC_WR_RWC	(1 << 23)

#define		vr5701_CODEC_RD_RRDYA	(1 << 31)
#define		vr5701_CODEC_RD_RRDYD	(1 << 30)

#define		vr5701_ACLINK_CTRL_RST_ON	(1 << 15)
#define		vr5701_ACLINK_CTRL_RST_TIME	0x7f
#define		vr5701_ACLINK_CTRL_SYNC_ON	(1 << 30)
#define		vr5701_ACLINK_CTRL_CK_STOP_ON	(1 << 31)

#define		vr5701_CTRL_DAC2ENB		(1 << 15)
#define		vr5701_CTRL_ADC2ENB		(1 << 14)
#define		vr5701_CTRL_DAC1ENB		(1 << 13)
#define		vr5701_CTRL_ADC1ENB		(1 << 12)

#define		vr5701_INT_MASK_NMASK		(1 << 31)
#define		vr5701_INT_MASK_DAC1END	(1 << 5)
#define		vr5701_INT_MASK_DAC2END	(1 << 4)
#define		vr5701_INT_MASK_DAC3END	(1 << 3)
#define		vr5701_INT_MASK_ADC1END	(1 << 2)
#define		vr5701_INT_MASK_ADC2END	(1 << 1)
#define		vr5701_INT_MASK_ADC3END	(1 << 0)

#define		vr5701_DMA_ACTIVATION		(1 << 31)
#define		vr5701_DMA_WIP			(1 << 30)

#define vr5701_AC97_MODULE_NAME "NEC_vr5701_audio"
#define PFX vr5701_AC97_MODULE_NAME ": "
#define	WORK_BUF_SIZE	2048

#define DMABUF_DEFAULTORDER (16-PAGE_SHIFT)
#define DMABUF_MINORDER 1

/* maximum number of devices; only used for command line params */
#define NR_DEVICE 5
/* -------------------debug macros -------------------------------------- */
#undef vr5701_AC97_DEBUG

#undef vr5701_AC97_VERBOSE_DEBUG

#if defined(vr5701_AC97_VERBOSE_DEBUG)
#define vr5701_AC97_DEBUG
#endif

#if defined(vr5701_AC97_DEBUG)
#define ASSERT(x)  if (!(x)) { \
	panic("assertion failed at %s:%d: %s\n", __FILE__, __LINE__, #x); }
#else
#define	ASSERT(x)
#endif				/* vr5701_AC97_DEBUG */

static inline unsigned ld2(unsigned int x)
{
	unsigned r = 0;

	if (x >= 0x10000) {
		x >>= 16;
		r += 16;
	}
	if (x >= 0x100) {
		x >>= 8;
		r += 8;
	}
	if (x >= 0x10) {
		x >>= 4;
		r += 4;
	}
	if (x >= 4) {
		x >>= 2;
		r += 2;
	}
	if (x >= 2)
		r++;
	return r;
}

#if defined(vr5701_AC97_VERBOSE_DEBUG)
static u16 inTicket;		/* check sync between intr & write */
static u16 outTicket;
#endif

#undef OSS_DOCUMENTED_MIXER_SEMANTICS

static const unsigned sample_shift[] = { 0, 1, 1, 2 };

struct vr5701_ac97_state {
	/* list of vr5701_ac97 devices */
	struct list_head devs;

	/* the corresponding pci_dev structure */
	struct pci_dev *dev;

	/* soundcore stuff */
	int dev_audio;

	/* hardware resources */
	unsigned long io;
	unsigned int irq;

#ifdef vr5701_AC97_DEBUG
	/* debug /proc entry */
	struct proc_dir_entry *ps;
	struct proc_dir_entry *ac97_ps;
#endif				/* vr5701_AC97_DEBUG */

	struct ac97_codec *codec;

	unsigned dacChannels, adcChannels;
	unsigned short dacRate, adcRate;
	unsigned short extended_status;

	spinlock_t lock;
	struct semaphore open_sem;
	mode_t open_mode;
	wait_queue_head_t open_wait;

	struct dmabuf {
		void *lbuf, *rbuf;
		dma_addr_t lbufDma, rbufDma;
		unsigned bufOrder;
		unsigned numFrag;
		unsigned fragShift;
		unsigned fragSize;	/* redundant */
		unsigned fragTotalSize;	/* = numFrag * fragSize(real)  */
		unsigned nextIn;
		unsigned nextOut;
		int count;
		unsigned error;	/* over/underrun */
		wait_queue_head_t wait;
		/* OSS stuff */
		unsigned stopped:1;
		unsigned ready:1;
	} dma_dac, dma_adc;

	struct {
		u16 lchannel;
		u16 rchannel;
	} workBuf[WORK_BUF_SIZE / 4];
};
static inline void stop_dac(struct vr5701_ac97_state *s)
{
	struct dmabuf *db = &s->dma_dac;
	unsigned long flags;
	u32 temp;

	spin_lock_irqsave(&s->lock, flags);

	if (db->stopped) {
		spin_unlock_irqrestore(&s->lock, flags);
		return;
	}

	/* deactivate the dma */
	outl(0, s->io + vr5701_DAC1_CTRL);
	outl(0, s->io + vr5701_DAC2_CTRL);

	/* wait for DAM completely stop */
	while (inl(s->io + vr5701_DAC1_CTRL) & vr5701_DMA_WIP) ;
	while (inl(s->io + vr5701_DAC2_CTRL) & vr5701_DMA_WIP) ;

	/* disable dac slots in aclink */
	temp = inl(s->io + vr5701_CTRL);
	temp &= ~(vr5701_CTRL_DAC1ENB | vr5701_CTRL_DAC2ENB);
	outl(temp, s->io + vr5701_CTRL);

	/* disable interrupts */
	temp = inl(s->io + vr5701_INT_MASK);
	temp &= ~(vr5701_INT_MASK_DAC1END | vr5701_INT_MASK_DAC2END);
	outl(temp, s->io + vr5701_INT_MASK);

	/* clear pending ones */
	outl(vr5701_INT_MASK_DAC1END | vr5701_INT_MASK_DAC2END,
	     s->io + vr5701_INT_CLR);

	db->stopped = 1;

	spin_unlock_irqrestore(&s->lock, flags);
}

static inline void stop_adc(struct vr5701_ac97_state *s)
{
	struct dmabuf *db = &s->dma_adc;
	unsigned long flags;
	u32 temp;

	spin_lock_irqsave(&s->lock, flags);

	if (db->stopped) {
		spin_unlock_irqrestore(&s->lock, flags);
		return;
	}

	/* deactivate the dma */
	outl(0, s->io + vr5701_ADC1_CTRL);
	outl(0, s->io + vr5701_ADC2_CTRL);

	/* disable adc slots in aclink */
	temp = inl(s->io + vr5701_CTRL);
	temp &= ~(vr5701_CTRL_ADC1ENB | vr5701_CTRL_ADC2ENB);
	outl(temp, s->io + vr5701_CTRL);

	/* disable interrupts */
	temp = inl(s->io + vr5701_INT_MASK);
	temp &= ~(vr5701_INT_MASK_ADC1END | vr5701_INT_MASK_ADC2END);
	outl(temp, s->io + vr5701_INT_MASK);

	/* clear pending ones */
	outl(vr5701_INT_MASK_ADC1END | vr5701_INT_MASK_ADC2END,
	     s->io + vr5701_INT_CLR);

	db->stopped = 1;

	spin_unlock_irqrestore(&s->lock, flags);
}

static inline void dealloc_dmabuf(struct vr5701_ac97_state *s,
				  struct dmabuf *db)
{
	if (db->lbuf) {
		ASSERT(db->rbuf);
		pci_free_consistent(s->dev, PAGE_SIZE << db->bufOrder,
				    db->lbuf, db->lbufDma);
		pci_free_consistent(s->dev, PAGE_SIZE << db->bufOrder,
				    db->rbuf, db->rbufDma);
		db->lbuf = db->rbuf = NULL;
	}
	db->nextIn = db->nextOut = 0;
	db->ready = 0;
}

static inline int prog_dmabuf_adc(struct vr5701_ac97_state *s)
{
	stop_adc(s);
	return prog_dmabuf(s, &s->dma_adc, s->adcRate);
}

static inline int prog_dmabuf_dac(struct vr5701_ac97_state *s)
{
	stop_dac(s);
	return prog_dmabuf(s, &s->dma_dac, s->dacRate);
}

static inline void vr5701_ac97_adc_interrupt(struct vr5701_ac97_state *s)
{
	struct dmabuf *adc = &s->dma_adc;
	unsigned temp;

	/* we need two frags avaiable because one is already being used
	 * and the other will be used when next interrupt happens.
	 */
	if (adc->count >= adc->fragTotalSize - adc->fragSize) {
		stop_adc(s);
		adc->error++;
		printk(KERN_INFO PFX "adc overrun\n");
		return;
	}

	/* set the base addr for next DMA transfer */
	temp = adc->nextIn + 2 * adc->fragSize;
	if (temp >= adc->fragTotalSize) {
		ASSERT((temp == adc->fragTotalSize) ||
		       (temp == adc->fragTotalSize + adc->fragSize));
		temp -= adc->fragTotalSize;
	}
	outl(adc->lbufDma + temp, s->io + vr5701_ADC1_BADDR);
	outl(adc->rbufDma + temp, s->io + vr5701_ADC2_BADDR);

	/* adjust nextIn */
	adc->nextIn += adc->fragSize;
	if (adc->nextIn >= adc->fragTotalSize) {
		ASSERT(adc->nextIn == adc->fragTotalSize);
		adc->nextIn = 0;
	}

	/* adjust count */
	adc->count += adc->fragSize;

	/* wake up anybody listening */
	if (waitqueue_active(&adc->wait)) {
		wake_up_interruptible(&adc->wait);
	}
}

static inline void vr5701_ac97_dac_interrupt(struct vr5701_ac97_state *s)
{
	struct dmabuf *dac = &s->dma_dac;
	unsigned temp;

	/* let us set for next next DMA transfer */
	temp = dac->nextOut + dac->fragSize * 2;
	if (temp >= dac->fragTotalSize) {
		ASSERT((temp == dac->fragTotalSize) ||
		       (temp == dac->fragTotalSize + dac->fragSize));
		temp -= dac->fragTotalSize;
	}
	outl(dac->lbufDma + temp, s->io + vr5701_DAC1_BADDR);
	if (s->dacChannels == 1) {
		outl(dac->lbufDma + temp, s->io + vr5701_DAC2_BADDR);
	} else {
		outl(dac->rbufDma + temp, s->io + vr5701_DAC2_BADDR);
	}

#if defined(vr5701_AC97_VERBOSE_DEBUG)
	if (*(u16 *) (dac->lbuf + dac->nextOut) != outTicket) {
		printk(KERN_ERR "assert fail: - %d vs %d\n",
		       *(u16 *) (dac->lbuf + dac->nextOut), outTicket);
		ASSERT(1 == 0);
	}
#endif

	/* adjust nextOut pointer */
	dac->nextOut += dac->fragSize;
	if (dac->nextOut >= dac->fragTotalSize) {
		ASSERT(dac->nextOut == dac->fragTotalSize);
		dac->nextOut = 0;
	}

	/* adjust count */
	dac->count -= dac->fragSize;
	if (dac->count <= 0) {
		/* buffer under run */
		dac->count = 0;
		dac->nextIn = dac->nextOut;
		stop_dac(s);
	}
#if defined(vr5701_AC97_VERBOSE_DEBUG)
	if (dac->count) {
		outTicket++;
		ASSERT(*(u16 *) (dac->lbuf + dac->nextOut) == outTicket);
	}
#endif

	/* we cannot have both under run and someone is waiting on us */
	ASSERT(!(waitqueue_active(&dac->wait) && (dac->count <= 0)));

	/* wake up anybody listening */
	if (waitqueue_active(&dac->wait))
		wake_up_interruptible(&dac->wait);
}

static inline int
copy_two_channel_adc_to_user(struct vr5701_ac97_state *s,
			     char *buffer, int copyCount)
{
	struct dmabuf *db = &s->dma_adc;
	int bufStart = db->nextOut;
	for (; copyCount > 0;) {
		int i;
		int count = copyCount;
		if (count > WORK_BUF_SIZE / 2)
			count = WORK_BUF_SIZE / 2;
		for (i = 0; i < count / 2; i++) {
			s->workBuf[i].lchannel =
			    *(u16 *) (db->lbuf + bufStart + i * 2);
			s->workBuf[i].rchannel =
			    *(u16 *) (db->rbuf + bufStart + i * 2);
		}
		if (copy_to_user(buffer, s->workBuf, count * 2)) {
			return -1;
		}

		copyCount -= count;
		bufStart += count;
		ASSERT(bufStart <= db->fragTotalSize);
		buffer += count * 2;
	}
	return 0;
}

/* return the total bytes that is copied */
static inline int
copy_adc_to_user(struct vr5701_ac97_state *s,
		 char *buffer, size_t count, int avail)
{
	struct dmabuf *db = &s->dma_adc;
	int copyCount = 0;
	int copyFragCount = 0;
	int totalCopyCount = 0;
	int totalCopyFragCount = 0;
	unsigned long flags;

	/* adjust count to signel channel byte count */
	count >>= s->adcChannels - 1;

	/* we may have to "copy" twice as ring buffer wraps around */
	for (; (avail > 0) && (count > 0);) {
		/* determine max possible copy count for single channel */
		copyCount = count;
		if (copyCount > avail) {
			copyCount = avail;
		}
		if (copyCount + db->nextOut > db->fragTotalSize) {
			copyCount = db->fragTotalSize - db->nextOut;
			ASSERT((copyCount % db->fragSize) == 0);
		}

		copyFragCount = (copyCount - 1) >> db->fragShift;
		copyFragCount = (copyFragCount + 1) << db->fragShift;
		ASSERT(copyFragCount >= copyCount);

		/* we copy differently based on adc channels */
		if (s->adcChannels == 1) {
			if (copy_to_user(buffer,
					 db->lbuf + db->nextOut, copyCount))
				return -1;
		} else {
			/* *sigh* we have to mix two streams into one  */
			if (copy_two_channel_adc_to_user(s, buffer, copyCount))
				return -1;
		}

		count -= copyCount;
		totalCopyCount += copyCount;
		avail -= copyFragCount;
		totalCopyFragCount += copyFragCount;

		buffer += copyCount << (s->adcChannels - 1);

		db->nextOut += copyFragCount;
		if (db->nextOut >= db->fragTotalSize) {
			ASSERT(db->nextOut == db->fragTotalSize);
			db->nextOut = 0;
		}

		ASSERT((copyFragCount % db->fragSize) == 0);
		ASSERT((count == 0) || (copyCount == copyFragCount));
	}

	spin_lock_irqsave(&s->lock, flags);
	db->count -= totalCopyFragCount;
	spin_unlock_irqrestore(&s->lock, flags);

	return totalCopyCount << (s->adcChannels - 1);
}

static inline int
copy_two_channel_dac_from_user(struct vr5701_ac97_state *s,
			       const char *buffer, int copyCount)
{
	struct dmabuf *db = &s->dma_dac;
	int bufStart = db->nextIn;

	ASSERT(db->ready);

	for (; copyCount > 0;) {
		int i;
		int count = copyCount;
		if (count > WORK_BUF_SIZE / 2)
			count = WORK_BUF_SIZE / 2;
		if (copy_from_user(s->workBuf, buffer, count * 2)) {
			return -1;
		}
		for (i = 0; i < count / 2; i++) {
			*(u16 *) (db->lbuf + bufStart + i * 2) =
			    s->workBuf[i].lchannel;
			*(u16 *) (db->rbuf + bufStart + i * 2) =
			    s->workBuf[i].rchannel;
		}

		copyCount -= count;
		bufStart += count;
		ASSERT(bufStart <= db->fragTotalSize);
		buffer += count * 2;
	}
	return 0;

}
