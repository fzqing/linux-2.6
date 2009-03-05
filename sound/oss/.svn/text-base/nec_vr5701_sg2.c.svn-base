/*
 * sound/oss/nec_vr5701_sg2.c
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
#include "nec_vr5701_sg2.h"

static LIST_HEAD(devs);

static u16 rdcodec(struct ac97_codec *codec, u8 addr)
{
	struct vr5701_ac97_state *s =
	    (struct vr5701_ac97_state *)codec->private_data;
	unsigned long flags;
	u32 result;

	spin_lock_irqsave(&s->lock, flags);

	/* wait until we can access codec registers */
	while (inl(s->io + vr5701_CODEC_WR) & 0x80000000) ;

	/* write the address and "read" command to codec */
	addr = addr & 0x7f;
	outl((addr << 16) | vr5701_CODEC_WR_RWC, s->io + vr5701_CODEC_WR);

	/* get the return result */
	udelay(100);		/* workaround hardware bug */
	while ((result = inl(s->io + vr5701_CODEC_RD)) &
	       (vr5701_CODEC_RD_RRDYA | vr5701_CODEC_RD_RRDYD)) {
		/* we get either addr or data, or both */
		if (result & vr5701_CODEC_RD_RRDYA) {
			ASSERT(addr == ((result >> 16) & 0x7f));
		}
		if (result & vr5701_CODEC_RD_RRDYD) {
			break;
		}
	}

	spin_unlock_irqrestore(&s->lock, flags);

	return result & 0xffff;
}

static void wrcodec(struct ac97_codec *codec, u8 addr, u16 data)
{
	struct vr5701_ac97_state *s =
	    (struct vr5701_ac97_state *)codec->private_data;
	unsigned long flags;

	spin_lock_irqsave(&s->lock, flags);

	/* wait until we can access codec registers */
	while (inl(s->io + vr5701_CODEC_WR) & 0x80000000) ;

	/* write the address and value to codec */
	outl((addr << 16) | data, s->io + vr5701_CODEC_WR);

	spin_unlock_irqrestore(&s->lock, flags);
}

static void waitcodec(struct ac97_codec *codec)
{
	struct vr5701_ac97_state *s =
	    (struct vr5701_ac97_state *)codec->private_data;

	/* wait until we can access codec registers */
	while (inl(s->io + vr5701_CODEC_WR) & 0x80000000) ;
}

static void vr5701_ac97_delay(int msec)
{
	unsigned long tmo;
	signed long tmo2;

	if (in_interrupt())
		return;

	tmo = jiffies + (msec * HZ) / 1000;
	for (;;) {
		tmo2 = tmo - jiffies;
		if (tmo2 <= 0)
			break;
		schedule_timeout(tmo2);
	}
}

static void set_adc_rate(struct vr5701_ac97_state *s, unsigned rate)
{
	wrcodec(s->codec, AC97_PCM_LR_ADC_RATE, rate);
	s->adcRate = rate;
}

static void set_dac_rate(struct vr5701_ac97_state *s, unsigned rate)
{
	if (s->extended_status & AC97_EXTSTAT_VRA) {
		wrcodec(s->codec, AC97_PCM_FRONT_DAC_RATE, rate);
		s->dacRate = rdcodec(s->codec, AC97_PCM_FRONT_DAC_RATE);
	}
}

static int start_dac(struct vr5701_ac97_state *s)
{
	struct dmabuf *db = &s->dma_dac;
	unsigned long flags;
	u32 dmaLength;
	u32 temp;

	spin_lock_irqsave(&s->lock, flags);

	if (!db->stopped) {
		spin_unlock_irqrestore(&s->lock, flags);
		return -1;
	}

	/* we should have some data to do the DMA trasnfer */
	if(db->count < db->fragSize){
		spin_unlock_irqrestore(&s->lock, flags);
		return -1;
	}

	/* clear pending fales interrupts */
	outl(vr5701_INT_MASK_DAC1END | vr5701_INT_MASK_DAC2END,
	     s->io + vr5701_INT_CLR);

	/* enable interrupts */
	temp = inl(s->io + vr5701_INT_MASK);
	temp |= vr5701_INT_MASK_DAC1END | vr5701_INT_MASK_DAC2END;
	outl(temp, s->io + vr5701_INT_MASK);

	/* setup dma base addr */
	outl(db->lbufDma + db->nextOut, s->io + vr5701_DAC1_BADDR);
	if (s->dacChannels == 1) {
		outl(db->lbufDma + db->nextOut, s->io + vr5701_DAC2_BADDR);
	} else {
		outl(db->rbufDma + db->nextOut, s->io + vr5701_DAC2_BADDR);
	}

	/* set dma length, in the unit of 0x10 bytes */
	dmaLength = db->fragSize >> 4;
	outl(dmaLength, s->io + vr5701_DAC1L);
	outl(dmaLength, s->io + vr5701_DAC2L);

	/* activate dma */
	outl(vr5701_DMA_ACTIVATION, s->io + vr5701_DAC1_CTRL);
	outl(vr5701_DMA_ACTIVATION, s->io + vr5701_DAC2_CTRL);

	/* enable dac slots - we should hear the music now! */
	temp = inl(s->io + vr5701_CTRL);
	temp |= (vr5701_CTRL_DAC1ENB | vr5701_CTRL_DAC2ENB);
	outl(temp, s->io + vr5701_CTRL);

	/* it is time to setup next dma transfer */
	ASSERT(inl(s->io + vr5701_DAC1_CTRL) & vr5701_DMA_WIP);
	ASSERT(inl(s->io + vr5701_DAC2_CTRL) & vr5701_DMA_WIP);

	temp = db->nextOut + db->fragSize;
	if (temp >= db->fragTotalSize) {
		ASSERT(temp == db->fragTotalSize);
		temp = 0;
	}

	outl(db->lbufDma + temp, s->io + vr5701_DAC1_BADDR);
	if (s->dacChannels == 1) {
		outl(db->lbufDma + temp, s->io + vr5701_DAC2_BADDR);
	} else {
		outl(db->rbufDma + temp, s->io + vr5701_DAC2_BADDR);
	}

	db->stopped = 0;

#if defined(vr5701_AC97_VERBOSE_DEBUG)
	outTicket = *(u16 *) (db->lbuf + db->nextOut);
	if (db->count > db->fragSize) {
		ASSERT((u16) (outTicket + 1) == *(u16 *) (db->lbuf + temp));
	}
#endif
	spin_unlock_irqrestore(&s->lock, flags);
	return 0;
}

static void start_adc(struct vr5701_ac97_state *s)
{
	struct dmabuf *db = &s->dma_adc;
	unsigned long flags;
	u32 dmaLength;
	u32 temp;

	spin_lock_irqsave(&s->lock, flags);

	if (!db->stopped) {
		spin_unlock_irqrestore(&s->lock, flags);
		return;
	}

	/* we should at least have some free space in the buffer */
	ASSERT(db->count < db->fragTotalSize - db->fragSize * 2);

	/* clear pending ones */
	outl(vr5701_INT_MASK_ADC1END | vr5701_INT_MASK_ADC2END,
	     s->io + vr5701_INT_CLR);

	/* enable interrupts */
	temp = inl(s->io + vr5701_INT_MASK);
	temp |= vr5701_INT_MASK_ADC1END | vr5701_INT_MASK_ADC2END;
	outl(temp, s->io + vr5701_INT_MASK);

	/* setup dma base addr */
	outl(db->lbufDma + db->nextIn, s->io + vr5701_ADC1_BADDR);
	outl(db->rbufDma + db->nextIn, s->io + vr5701_ADC2_BADDR);

	/* setup dma length */
	dmaLength = db->fragSize >> 4;
	outl(dmaLength, s->io + vr5701_ADC1L);
	outl(dmaLength, s->io + vr5701_ADC2L);

	/* activate dma */
	outl(vr5701_DMA_ACTIVATION, s->io + vr5701_ADC1_CTRL);
	outl(vr5701_DMA_ACTIVATION, s->io + vr5701_ADC2_CTRL);

	/* enable adc slots */
	temp = inl(s->io + vr5701_CTRL);
	temp |= (vr5701_CTRL_ADC1ENB | vr5701_CTRL_ADC2ENB);
	outl(temp, s->io + vr5701_CTRL);

	/* it is time to setup next dma transfer */
	temp = db->nextIn + db->fragSize;
	if (temp >= db->fragTotalSize) {
		ASSERT(temp == db->fragTotalSize);
		temp = 0;
	}
	outl(db->lbufDma + temp, s->io + vr5701_ADC1_BADDR);
	outl(db->rbufDma + temp, s->io + vr5701_ADC2_BADDR);

	db->stopped = 0;

	spin_unlock_irqrestore(&s->lock, flags);
}

/* return the total bytes that is copied */
static inline int
copy_dac_from_user(struct vr5701_ac97_state *s,
		   const char *buffer, size_t count, int avail)
{
	struct dmabuf *db = &s->dma_dac;
	int copyCount = 0;
	int copyFragCount = 0;
	int totalCopyCount = 0;
	int totalCopyFragCount = 0;
	unsigned long flags;
#if defined(vr5701_AC97_VERBOSE_DEBUG)
	int i;
#endif

	/* adjust count to signel channel byte count */
	count >>= s->dacChannels - 1;

	/* we may have to "copy" twice as ring buffer wraps around */
	for (; (avail > 0) && (count > 0);) {
		/* determine max possible copy count for single channel */
		copyCount = count;
		if (copyCount > avail) {
			copyCount = avail;
		}
		if (copyCount + db->nextIn > db->fragTotalSize) {
			copyCount = db->fragTotalSize - db->nextIn;
			ASSERT(copyCount > 0);
		}

		copyFragCount = copyCount;
		ASSERT(copyFragCount >= copyCount);

		/* we copy differently based on the number channels */
		if (s->dacChannels == 1) {
			if (copy_from_user(db->lbuf + db->nextIn,
					   buffer, copyCount))
				return -1;
			/* fill gaps with 0 */
			memset(db->lbuf + db->nextIn + copyCount,
			       0, copyFragCount - copyCount);
		} else {
			/* we have demux the stream into two separate ones */
			if (copy_two_channel_dac_from_user
			    (s, buffer, copyCount))
				return -1;
			/* fill gaps with 0 */
			memset(db->lbuf + db->nextIn + copyCount,
			       0, copyFragCount - copyCount);
			memset(db->rbuf + db->nextIn + copyCount,
			       0, copyFragCount - copyCount);
		}

#if defined(vr5701_AC97_VERBOSE_DEBUG)
		for (i = 0; i < copyFragCount; i += db->fragSize) {
			*(u16 *) (db->lbuf + db->nextIn + i) = inTicket++;
		}
#endif

		count -= copyCount;
		totalCopyCount += copyCount;
		avail -= copyFragCount;
		totalCopyFragCount += copyFragCount;

		buffer += copyCount << (s->dacChannels - 1);

		db->nextIn += copyFragCount;
		if (db->nextIn >= db->fragTotalSize) {
			ASSERT(db->nextIn == db->fragTotalSize);
			db->nextIn = 0;
		}

		ASSERT((count == 0) || (copyCount == copyFragCount));
	}

	spin_lock_irqsave(&s->lock, flags);
	db->count += totalCopyFragCount;
	if (db->stopped) 
		if(start_dac(s)){
			spin_unlock_irqrestore(&s->lock, flags);
			return -1;
	}

	/* nextIn should not be equal to nextOut unless we are full */
	if(!(((db->count == db->fragTotalSize) &&
		(db->nextIn == db->nextOut)) ||
	       ((db->count < db->fragTotalSize) &&
		(db->nextIn != db->nextOut)))){
			spin_unlock_irqrestore(&s->lock, flags);
			return -1;
			}
	spin_unlock_irqrestore(&s->lock, flags);

	return totalCopyCount << (s->dacChannels - 1);

}

static int prog_dmabuf(struct vr5701_ac97_state *s,
		       struct dmabuf *db, unsigned rate)
{
	int order;
	unsigned bufsize;

	if (!db->lbuf) {
		ASSERT(!db->rbuf);

		db->ready = 0;
		for (order = DMABUF_DEFAULTORDER;
		     order >= DMABUF_MINORDER; order--) {
			db->lbuf = pci_alloc_consistent(s->dev,
							PAGE_SIZE << order,
							&db->lbufDma);
			db->rbuf = pci_alloc_consistent(s->dev,
							PAGE_SIZE << order,
							&db->rbufDma);
			if (db->lbuf && db->rbuf)
				break;
			if (db->lbuf) {
				ASSERT(!db->rbuf);
				pci_free_consistent(s->dev,
						    PAGE_SIZE << order,
						    db->lbuf, db->lbufDma);
			}
		}
		if (!db->lbuf) {
			ASSERT(!db->rbuf);
			return -ENOMEM;
		}

		db->bufOrder = order;
	}

	db->count = 0;
	db->nextIn = db->nextOut = 0;

	bufsize = PAGE_SIZE << db->bufOrder;
	db->fragShift = ld2(rate * 2 / 100);
	if (db->fragShift < 4)
		db->fragShift = 4;

	db->numFrag = bufsize >> db->fragShift;
	while (db->numFrag < 4 && db->fragShift > 4) {
		db->fragShift--;
		db->numFrag = bufsize >> db->fragShift;
	}
	db->fragSize = 1 << db->fragShift;
	db->fragTotalSize = db->numFrag << db->fragShift;
	memset(db->lbuf, 0, db->fragTotalSize);
	memset(db->rbuf, 0, db->fragTotalSize);

	db->ready = 1;

	return 0;
}

static irqreturn_t vr5701_ac97_interrupt(int irq, void *dev_id,
					 struct pt_regs *regs)
{
	struct vr5701_ac97_state *s = (struct vr5701_ac97_state *)dev_id;
	u32 irqStatus;
	u32 adcInterrupts, dacInterrupts;

	spin_lock(&s->lock);

	/* get irqStatus and clear the detected ones */
	irqStatus = inl(s->io + vr5701_INT_STATUS);
	outl(irqStatus, s->io + vr5701_INT_CLR);

	/* let us see what we get */
	dacInterrupts = vr5701_INT_MASK_DAC1END | vr5701_INT_MASK_DAC2END;
	adcInterrupts = vr5701_INT_MASK_ADC1END | vr5701_INT_MASK_ADC2END;
	if (irqStatus & dacInterrupts) {
		/* we should get both interrupts, but just in case ...  */
		if (irqStatus & vr5701_INT_MASK_DAC1END) {
			vr5701_ac97_dac_interrupt(s);
		}
		if ((irqStatus & dacInterrupts) != dacInterrupts) {
			printk(KERN_WARNING
			       "vr5701_ac97 : dac interrupts not in sync!!!\n");
			stop_dac(s);
			start_dac(s);
		}
	} else if (irqStatus & adcInterrupts) {
		/* we should get both interrupts, but just in case ...  */
		if (irqStatus & vr5701_INT_MASK_ADC1END) {
			vr5701_ac97_adc_interrupt(s);
		}
		if ((irqStatus & adcInterrupts) != adcInterrupts) {
			printk(KERN_WARNING
			       "vr5701_ac97 : adc interrupts not in sync!!!\n");
			stop_adc(s);
			start_adc(s);
		}
	}

	spin_unlock(&s->lock);
	return IRQ_HANDLED;
}

static int vr5701_ac97_open_mixdev(struct inode *inode, struct file *file)
{
	int minor = iminor(inode);
	struct list_head *list;
	struct vr5701_ac97_state *s;

	for (list = devs.next;; list = list->next) {
		if (list == &devs)
			return -ENODEV;
		s = list_entry(list, struct vr5701_ac97_state, devs);
		if (s->codec->dev_mixer == minor)
			break;
	}
	file->private_data = s;
	return nonseekable_open(inode, file);
}

static int vr5701_ac97_release_mixdev(struct inode *inode, struct file *file)
{
	return 0;
}

static int mixdev_ioctl(struct ac97_codec *codec, unsigned int cmd,
			unsigned long arg)
{
	return codec->mixer_ioctl(codec, cmd, arg);
}

static int vr5701_ac97_ioctl_mixdev(struct inode *inode, struct file *file,
				    unsigned int cmd, unsigned long arg)
{
	struct vr5701_ac97_state *s =
	    (struct vr5701_ac97_state *)file->private_data;
	struct ac97_codec *codec = s->codec;

	return mixdev_ioctl(codec, cmd, arg);
}

static struct file_operations vr5701_ac97_mixer_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.ioctl = vr5701_ac97_ioctl_mixdev,
	.open = vr5701_ac97_open_mixdev,
	.release = vr5701_ac97_release_mixdev,
};

static int drain_dac(struct vr5701_ac97_state *s, int nonblock)
{
	unsigned long flags;
	int count, tmo;

	if (!s->dma_dac.ready)
		return 0;

	for (;;) {
		spin_lock_irqsave(&s->lock, flags);
		count = s->dma_dac.count;
		spin_unlock_irqrestore(&s->lock, flags);
		if (count <= 0)
			break;
		if (signal_pending(current))
			break;
		if (nonblock)
			return -EBUSY;
		tmo = 1000 * count / s->dacRate / 2;
		vr5701_ac97_delay(tmo);
	}
	if (signal_pending(current))
		return -ERESTARTSYS;
	return 0;
}

static ssize_t
vr5701_ac97_read(struct file *file, char *buffer, size_t count, loff_t * ppos)
{
	struct vr5701_ac97_state *s =
	    (struct vr5701_ac97_state *)file->private_data;
	struct dmabuf *db = &s->dma_adc;
	ssize_t ret = 0;
	unsigned long flags;
	int copyCount;
	size_t avail;

	if (!access_ok(VERIFY_WRITE, buffer, count))
		return -EFAULT;

	ASSERT(db->ready);

	while (count > 0) {
		do {
			spin_lock_irqsave(&s->lock, flags);
			if (db->stopped)
				start_adc(s);
			avail = db->count;
			spin_unlock_irqrestore(&s->lock, flags);
			if (avail <= 0) {
				if (file->f_flags & O_NONBLOCK) {
					if (!ret)
						ret = -EAGAIN;
					return ret;
				}
				interruptible_sleep_on(&db->wait);
				if (signal_pending(current)) {
					if (!ret)
						ret = -ERESTARTSYS;
					return ret;
				}
			}
		} while (avail <= 0);

		ASSERT((avail % db->fragSize) == 0);
		copyCount = copy_adc_to_user(s, buffer, count, avail);
		if (copyCount <= 0) {
			if (!ret)
				ret = -EFAULT;
			return ret;
		}

		count -= copyCount;
		buffer += copyCount;
		ret += copyCount;
	}

	return ret;
}

static ssize_t vr5701_ac97_write(struct file *file, const char *buffer,
				 size_t count, loff_t * ppos)
{
	struct vr5701_ac97_state *s =
	    (struct vr5701_ac97_state *)file->private_data;
	struct dmabuf *db = &s->dma_dac;
	ssize_t ret;
	unsigned long flags;
	int copyCount, avail;

	if (!access_ok(VERIFY_READ, buffer, count))
		return -EFAULT;
	ret = 0;

	while (count > 0) {
		do {
			spin_lock_irqsave(&s->lock, flags);
			avail = db->fragTotalSize - db->count;
			spin_unlock_irqrestore(&s->lock, flags);
			if (avail <= 0) {
				if (file->f_flags & O_NONBLOCK) {
					if (!ret)
						ret = -EAGAIN;
					return ret;
				}
				interruptible_sleep_on(&db->wait);
				if (signal_pending(current)) {
					if (!ret)
						ret = -ERESTARTSYS;
					return ret;
				}
			}
		} while (avail <= 0);

		copyCount = copy_dac_from_user(s, buffer, count, avail);
		if (copyCount < 0) {
			if (!ret)
				ret = -EFAULT;
			stop_dac(s);
			synchronize_irq(s->irq);
			s->dma_dac.count = 0;
			s->dma_dac.nextIn = s->dma_dac.nextOut = 0;
			return ret;
		}

		count -= copyCount;
		buffer += copyCount;
		ret += copyCount;
	}

	return ret;
}

/* No kernel lock - we have our own spinlock */
static unsigned int vr5701_ac97_poll(struct file *file,
				     struct poll_table_struct *wait)
{
	struct vr5701_ac97_state *s =
	    (struct vr5701_ac97_state *)file->private_data;
	unsigned long flags;
	unsigned int mask = 0;

	if (file->f_mode & FMODE_WRITE)
		poll_wait(file, &s->dma_dac.wait, wait);
	if (file->f_mode & FMODE_READ)
		poll_wait(file, &s->dma_adc.wait, wait);
	spin_lock_irqsave(&s->lock, flags);
	if (file->f_mode & FMODE_READ) {
		if (s->dma_adc.count >= (signed)s->dma_adc.fragSize)
			mask |= POLLIN | POLLRDNORM;
	}
	if (file->f_mode & FMODE_WRITE) {
		if ((signed)s->dma_dac.fragTotalSize >=
		    s->dma_dac.count + (signed)s->dma_dac.fragSize)
			mask |= POLLOUT | POLLWRNORM;
	}
	spin_unlock_irqrestore(&s->lock, flags);
	return mask;
}

#ifdef vr5701_AC97_DEBUG
static struct ioctl_str_t {
	unsigned int cmd;
	const char *str;
} ioctl_str[] = {
	{
	SNDCTL_DSP_RESET, "SNDCTL_DSP_RESET"}, {
	SNDCTL_DSP_SYNC, "SNDCTL_DSP_SYNC"}, {
	SNDCTL_DSP_SPEED, "SNDCTL_DSP_SPEED"}, {
	SNDCTL_DSP_STEREO, "SNDCTL_DSP_STEREO"}, {
	SNDCTL_DSP_GETBLKSIZE, "SNDCTL_DSP_GETBLKSIZE"}, {
	SNDCTL_DSP_SETFMT, "SNDCTL_DSP_SETFMT"}, {
	SNDCTL_DSP_SAMPLESIZE, "SNDCTL_DSP_SAMPLESIZE"}, {
	SNDCTL_DSP_CHANNELS, "SNDCTL_DSP_CHANNELS"}, {
	SOUND_PCM_WRITE_CHANNELS, "SOUND_PCM_WRITE_CHANNELS"}, {
	SOUND_PCM_WRITE_FILTER, "SOUND_PCM_WRITE_FILTER"}, {
	SNDCTL_DSP_POST, "SNDCTL_DSP_POST"}, {
	SNDCTL_DSP_SUBDIVIDE, "SNDCTL_DSP_SUBDIVIDE"}, {
	SNDCTL_DSP_SETFRAGMENT, "SNDCTL_DSP_SETFRAGMENT"}, {
	SNDCTL_DSP_GETFMTS, "SNDCTL_DSP_GETFMTS"}, {
	SNDCTL_DSP_GETOSPACE, "SNDCTL_DSP_GETOSPACE"}, {
	SNDCTL_DSP_GETISPACE, "SNDCTL_DSP_GETISPACE"}, {
	SNDCTL_DSP_NONBLOCK, "SNDCTL_DSP_NONBLOCK"}, {
	SNDCTL_DSP_GETCAPS, "SNDCTL_DSP_GETCAPS"}, {
	SNDCTL_DSP_GETTRIGGER, "SNDCTL_DSP_GETTRIGGER"}, {
	SNDCTL_DSP_SETTRIGGER, "SNDCTL_DSP_SETTRIGGER"}, {
	SNDCTL_DSP_GETIPTR, "SNDCTL_DSP_GETIPTR"}, {
	SNDCTL_DSP_GETOPTR, "SNDCTL_DSP_GETOPTR"}, {
	SNDCTL_DSP_MAPINBUF, "SNDCTL_DSP_MAPINBUF"}, {
	SNDCTL_DSP_MAPOUTBUF, "SNDCTL_DSP_MAPOUTBUF"}, {
	SNDCTL_DSP_SETSYNCRO, "SNDCTL_DSP_SETSYNCRO"}, {
	SNDCTL_DSP_SETDUPLEX, "SNDCTL_DSP_SETDUPLEX"}, {
	SNDCTL_DSP_GETODELAY, "SNDCTL_DSP_GETODELAY"}, {
	SNDCTL_DSP_GETCHANNELMASK, "SNDCTL_DSP_GETCHANNELMASK"}, {
	SNDCTL_DSP_BIND_CHANNEL, "SNDCTL_DSP_BIND_CHANNEL"}, {
	OSS_GETVERSION, "OSS_GETVERSION"}, {
	SOUND_PCM_READ_RATE, "SOUND_PCM_READ_RATE"}, {
	SOUND_PCM_READ_CHANNELS, "SOUND_PCM_READ_CHANNELS"}, {
	SOUND_PCM_READ_BITS, "SOUND_PCM_READ_BITS"}, {
	SOUND_PCM_READ_FILTER, "SOUND_PCM_READ_FILTER"}
};
#endif

static int vr5701_ac97_ioctl(struct inode *inode, struct file *file,
			     unsigned int cmd, unsigned long arg)
{
	struct vr5701_ac97_state *s =
	    (struct vr5701_ac97_state *)file->private_data;
	unsigned long flags;
	audio_buf_info abinfo;
	int count;
	int val, ret;

#ifdef vr5701_AC97_DEBUG
	for (count = 0; count < sizeof(ioctl_str) / sizeof(ioctl_str[0]);
	     count++) {
		if (ioctl_str[count].cmd == cmd)
			break;
	}
	if (count < sizeof(ioctl_str) / sizeof(ioctl_str[0]))
		printk(KERN_INFO PFX "ioctl %s\n", ioctl_str[count].str);
	else
		printk(KERN_INFO PFX "ioctl unknown, 0x%x\n", cmd);
#endif

	switch (cmd) {
	case OSS_GETVERSION:
		return put_user(SOUND_VERSION, (int *)arg);

	case SNDCTL_DSP_SYNC:
		if (file->f_mode & FMODE_WRITE)
			return drain_dac(s, file->f_flags & O_NONBLOCK);
		return 0;

	case SNDCTL_DSP_SETDUPLEX:
		return 0;

	case SNDCTL_DSP_GETCAPS:
		return put_user(DSP_CAP_DUPLEX, (int *)arg);

	case SNDCTL_DSP_RESET:
		if (file->f_mode & FMODE_WRITE) {
			stop_dac(s);
			synchronize_irq(s->irq);
			s->dma_dac.count = 0;
			s->dma_dac.nextIn = s->dma_dac.nextOut = 0;
		}
		if (file->f_mode & FMODE_READ) {
			stop_adc(s);
			synchronize_irq(s->irq);
			s->dma_adc.count = 0;
			s->dma_adc.nextIn = s->dma_adc.nextOut = 0;
		}
		return 0;

	case SNDCTL_DSP_SPEED:
		if (get_user(val, (int *)arg))
			return -EFAULT;
		if (val >= 0) {
			if (file->f_mode & FMODE_READ) {
				stop_adc(s);
				set_adc_rate(s, val);
				if ((ret = prog_dmabuf_adc(s)))
					return ret;
			}
			if (file->f_mode & FMODE_WRITE) {
				stop_dac(s);
				set_dac_rate(s, val);
				if ((ret = prog_dmabuf_dac(s)))
					return ret;
			}
		}
		return put_user((file->f_mode & FMODE_READ) ?
				s->adcRate : s->dacRate, (int *)arg);

	case SNDCTL_DSP_STEREO:
		if (get_user(val, (int *)arg))
			return -EFAULT;
		if (file->f_mode & FMODE_READ) {
			stop_adc(s);
			if (val)
				s->adcChannels = 2;
			else
				s->adcChannels = 1;
			if ((ret = prog_dmabuf_adc(s)))
				return ret;
		}
		if (file->f_mode & FMODE_WRITE) {
			stop_dac(s);
			if (val)
				s->dacChannels = 2;
			else
				s->dacChannels = 1;
			if ((ret = prog_dmabuf_dac(s)))
				return ret;
		}
		return 0;

	case SNDCTL_DSP_CHANNELS:
		if (get_user(val, (int *)arg))
			return -EFAULT;
		if (val != 0) {
			if ((val != 1) && (val != 2))
				val = 2;

			if (file->f_mode & FMODE_READ) {
				stop_adc(s);
				s->dacChannels = val;
				if ((ret = prog_dmabuf_adc(s)))
					return ret;
			}
			if (file->f_mode & FMODE_WRITE) {
				stop_dac(s);
				s->dacChannels = val;
				if ((ret = prog_dmabuf_dac(s)))
					return ret;
			}
		}
		return put_user(val, (int *)arg);

	case SNDCTL_DSP_GETFMTS:	/* Returns a mask */
		return put_user(AFMT_S16_LE, (int *)arg);

	case SNDCTL_DSP_SETFMT:	/* Selects ONE fmt */
		if (get_user(val, (int *)arg))
			return -EFAULT;
		if (val != AFMT_QUERY) {
			if (val != AFMT_S16_LE)
				return -EINVAL;
			if (file->f_mode & FMODE_READ) {
				stop_adc(s);
				if ((ret = prog_dmabuf_adc(s)))
					return ret;
			}
			if (file->f_mode & FMODE_WRITE) {
				stop_dac(s);
				if ((ret = prog_dmabuf_dac(s)))
					return ret;
			}
		} else {
			val = AFMT_S16_LE;
		}
		return put_user(val, (int *)arg);

	case SNDCTL_DSP_POST:
		return 0;

	case SNDCTL_DSP_GETTRIGGER:
	case SNDCTL_DSP_SETTRIGGER:
		/* NO trigger */
		return -EINVAL;

	case SNDCTL_DSP_GETOSPACE:
		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;
		abinfo.fragsize = s->dma_dac.fragSize << (s->dacChannels - 1);
		spin_lock_irqsave(&s->lock, flags);
		count = s->dma_dac.count;
		spin_unlock_irqrestore(&s->lock, flags);
		abinfo.bytes = (s->dma_dac.fragTotalSize - count) <<
		    (s->dacChannels - 1);
		abinfo.fragstotal = s->dma_dac.numFrag;
		abinfo.fragments = abinfo.bytes >> s->dma_dac.fragShift >>
		    (s->dacChannels - 1);
		return copy_to_user((void *)arg, &abinfo,
				    sizeof(abinfo)) ? -EFAULT : 0;

	case SNDCTL_DSP_GETISPACE:
		if (!(file->f_mode & FMODE_READ))
			return -EINVAL;
		abinfo.fragsize = s->dma_adc.fragSize << (s->adcChannels - 1);
		spin_lock_irqsave(&s->lock, flags);
		count = s->dma_adc.count;
		spin_unlock_irqrestore(&s->lock, flags);
		if (count < 0)
			count = 0;
		abinfo.bytes = count << (s->adcChannels - 1);
		abinfo.fragstotal = s->dma_adc.numFrag;
		abinfo.fragments = (abinfo.bytes >> s->dma_adc.fragShift) >>
		    (s->adcChannels - 1);
		return copy_to_user((void *)arg, &abinfo,
				    sizeof(abinfo)) ? -EFAULT : 0;

	case SNDCTL_DSP_NONBLOCK:
		file->f_flags |= O_NONBLOCK;
		return 0;

	case SNDCTL_DSP_GETODELAY:
		if (!(file->f_mode & FMODE_WRITE))
			return -EINVAL;
		spin_lock_irqsave(&s->lock, flags);
		count = s->dma_dac.count;
		spin_unlock_irqrestore(&s->lock, flags);
		return put_user(count, (int *)arg);

	case SNDCTL_DSP_GETIPTR:
	case SNDCTL_DSP_GETOPTR:
		/* we cannot get DMA ptr */
		return -EINVAL;

	case SNDCTL_DSP_GETBLKSIZE:
		if (file->f_mode & FMODE_WRITE)
			return put_user(s->dma_dac.
					fragSize << (s->dacChannels - 1),
					(int *)arg);
		else
			return put_user(s->dma_adc.
					fragSize << (s->adcChannels - 1),
					(int *)arg);

	case SNDCTL_DSP_SETFRAGMENT:
		/* we ignore fragment size request */
		return 0;

	case SNDCTL_DSP_SUBDIVIDE:
		/* what is this for? [jsun] */
		return 0;

	case SOUND_PCM_READ_RATE:
		return put_user((file->f_mode & FMODE_READ) ?
				s->adcRate : s->dacRate, (int *)arg);

	case SOUND_PCM_READ_CHANNELS:
		if (file->f_mode & FMODE_READ)
			return put_user(s->adcChannels, (int *)arg);
		else
			return put_user(s->dacChannels ? 2 : 1, (int *)arg);

	case SOUND_PCM_READ_BITS:
		return put_user(16, (int *)arg);

	case SOUND_PCM_WRITE_FILTER:
	case SNDCTL_DSP_SETSYNCRO:
	case SOUND_PCM_READ_FILTER:
		return -EINVAL;
	}

	return mixdev_ioctl(s->codec, cmd, arg);
}

static int vr5701_ac97_open(struct inode *inode, struct file *file)
{
	int minor = iminor(inode);
	DECLARE_WAITQUEUE(wait, current);
	unsigned long flags;
	struct list_head *list;
	struct vr5701_ac97_state *s;
	int ret = 0;

	nonseekable_open(inode, file);
	for (list = devs.next;; list = list->next) {
		if (list == &devs)
			return -ENODEV;
		s = list_entry(list, struct vr5701_ac97_state, devs);
		if (!((s->dev_audio ^ minor) & ~0xf))
			break;
	}
	file->private_data = s;

	/* wait for device to become free */
	down(&s->open_sem);
	while (s->open_mode & file->f_mode) {

		if (file->f_flags & O_NONBLOCK) {
			up(&s->open_sem);
			return -EBUSY;
		}
		add_wait_queue(&s->open_wait, &wait);
		__set_current_state(TASK_INTERRUPTIBLE);
		up(&s->open_sem);
		schedule();
		remove_wait_queue(&s->open_wait, &wait);
		set_current_state(TASK_RUNNING);
		if (signal_pending(current))
			return -ERESTARTSYS;
		down(&s->open_sem);
	}

	spin_lock_irqsave(&s->lock, flags);

	if (file->f_mode & FMODE_READ) {
		/* set default settings */
		set_adc_rate(s, 48000);
		s->adcChannels = 2;

		ret = prog_dmabuf_adc(s);
		if (ret)
			goto bailout;
	}
	if (file->f_mode & FMODE_WRITE) {
		/* set default settings */
		set_dac_rate(s, 48000);
		s->dacChannels = 2;

		ret = prog_dmabuf_dac(s);
		if (ret)
			goto bailout;
	}

	s->open_mode |= file->f_mode & (FMODE_READ | FMODE_WRITE);

      bailout:
	spin_unlock_irqrestore(&s->lock, flags);

	up(&s->open_sem);
	return ret;
}

static int vr5701_ac97_release(struct inode *inode, struct file *file)
{
	struct vr5701_ac97_state *s =
	    (struct vr5701_ac97_state *)file->private_data;

	lock_kernel();
	if (file->f_mode & FMODE_WRITE)
		drain_dac(s, file->f_flags & O_NONBLOCK);
	down(&s->open_sem);
	if (file->f_mode & FMODE_WRITE) {
		stop_dac(s);
		dealloc_dmabuf(s, &s->dma_dac);
	}
	if (file->f_mode & FMODE_READ) {
		stop_adc(s);
		dealloc_dmabuf(s, &s->dma_adc);
	}
	s->open_mode &= (~file->f_mode) & (FMODE_READ | FMODE_WRITE);
	up(&s->open_sem);
	wake_up(&s->open_wait);
	unlock_kernel();
	return 0;
}

static struct file_operations vr5701_ac97_audio_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = vr5701_ac97_read,
	.write = vr5701_ac97_write,
	.poll = vr5701_ac97_poll,
	.ioctl = vr5701_ac97_ioctl,
	.open = vr5701_ac97_open,
	.release = vr5701_ac97_release,
};

/*
 * for debugging purposes, we'll create a proc device that dumps the
 * CODEC chipstate
 */

#ifdef vr5701_AC97_DEBUG

struct {
	const char *regname;
	unsigned regaddr;
} vr5701_ac97_regs[] = {
	{
	"vr5701_INT_STATUS", vr5701_INT_STATUS}, {
	"vr5701_CODEC_WR", vr5701_CODEC_WR}, {
	"vr5701_CODEC_RD", vr5701_CODEC_RD}, {
	"vr5701_CTRL", vr5701_CTRL}, {
	"vr5701_ACLINK_CTRL", vr5701_ACLINK_CTRL}, {
	"vr5701_INT_MASK", vr5701_INT_MASK}, {
	"vr5701_DAC1_CTRL", vr5701_DAC1_CTRL}, {
	"vr5701_DAC1L", vr5701_DAC1L}, {
	"vr5701_DAC1_BADDR", vr5701_DAC1_BADDR}, {
	"vr5701_DAC2_CTRL", vr5701_DAC2_CTRL}, {
	"vr5701_DAC2L", vr5701_DAC2L}, {
	"vr5701_DAC2_BADDR", vr5701_DAC2_BADDR}, {
	"vr5701_DAC3_CTRL", vr5701_DAC3_CTRL}, {
	"vr5701_DAC3L", vr5701_DAC3L}, {
	"vr5701_DAC3_BADDR", vr5701_DAC3_BADDR}, {
	"vr5701_ADC1_CTRL", vr5701_ADC1_CTRL}, {
	"vr5701_ADC1L", vr5701_ADC1L}, {
	"vr5701_ADC1_BADDR", vr5701_ADC1_BADDR}, {
	"vr5701_ADC2_CTRL", vr5701_ADC2_CTRL}, {
	"vr5701_ADC2L", vr5701_ADC2L}, {
	"vr5701_ADC2_BADDR", vr5701_ADC2_BADDR}, {
	"vr5701_ADC3_CTRL", vr5701_ADC3_CTRL}, {
	"vr5701_ADC3L", vr5701_ADC3L}, {
	"vr5701_ADC3_BADDR", vr5701_ADC3_BADDR}, {
	NULL, 0x0}
};

static int proc_vr5701_ac97_dump(char *buf, char **start, off_t fpos,
				 int length, int *eof, void *data)
{
	struct vr5701_ac97_state *s;
	int cnt, len = 0;

	if (list_empty(&devs))
		return 0;
	s = list_entry(devs.next, struct vr5701_ac97_state, devs);

	/* print out header */
	len += sprintf(buf + len, "\n\t\tvr5701 Audio Debug\n\n");

	len += sprintf(buf + len, "NEC Electronics Corporation VR5701 SolutionGearII Audio Controller registers\n");
	len += sprintf(buf + len, "---------------------------------\n");
	for (cnt = 0; vr5701_ac97_regs[cnt].regname != NULL; cnt++) {
		len += sprintf(buf + len, "%-20s = %08x\n",
			       vr5701_ac97_regs[cnt].regname,
			       inl(s->io + vr5701_ac97_regs[cnt].regaddr));
	}

	/* print out driver state */
	len += sprintf(buf + len, "NEC Electronics Corporation VR5701 SolutionGearII Audio driver states\n");
	len += sprintf(buf + len, "---------------------------------\n");
	len += sprintf(buf + len, "dacChannels  = %d\n", s->dacChannels);
	len += sprintf(buf + len, "adcChannels  = %d\n", s->adcChannels);
	len += sprintf(buf + len, "dacRate  = %d\n", s->dacRate);
	len += sprintf(buf + len, "adcRate  = %d\n", s->adcRate);

	len += sprintf(buf + len, "dma_dac is %s ready\n",
		       s->dma_dac.ready ? "" : "not");
	if (s->dma_dac.ready) {
		len += sprintf(buf + len, "dma_dac is %s stopped.\n",
			       s->dma_dac.stopped ? "" : "not");
		len += sprintf(buf + len, "dma_dac.fragSize = %x\n",
			       s->dma_dac.fragSize);
		len += sprintf(buf + len, "dma_dac.fragShift = %x\n",
			       s->dma_dac.fragShift);
		len += sprintf(buf + len, "dma_dac.numFrag = %x\n",
			       s->dma_dac.numFrag);
		len += sprintf(buf + len, "dma_dac.fragTotalSize = %x\n",
			       s->dma_dac.fragTotalSize);
		len += sprintf(buf + len, "dma_dac.nextIn = %x\n",
			       s->dma_dac.nextIn);
		len += sprintf(buf + len, "dma_dac.nextOut = %x\n",
			       s->dma_dac.nextOut);
		len += sprintf(buf + len, "dma_dac.count = %x\n",
			       s->dma_dac.count);
	}

	len += sprintf(buf + len, "dma_adc is %s ready\n",
		       s->dma_adc.ready ? "" : "not");
	if (s->dma_adc.ready) {
		len += sprintf(buf + len, "dma_adc is %s stopped.\n",
			       s->dma_adc.stopped ? "" : "not");
		len += sprintf(buf + len, "dma_adc.fragSize = %x\n",
			       s->dma_adc.fragSize);
		len += sprintf(buf + len, "dma_adc.fragShift = %x\n",
			       s->dma_adc.fragShift);
		len += sprintf(buf + len, "dma_adc.numFrag = %x\n",
			       s->dma_adc.numFrag);
		len += sprintf(buf + len, "dma_adc.fragTotalSize = %x\n",
			       s->dma_adc.fragTotalSize);
		len += sprintf(buf + len, "dma_adc.nextIn = %x\n",
			       s->dma_adc.nextIn);
		len += sprintf(buf + len, "dma_adc.nextOut = %x\n",
			       s->dma_adc.nextOut);
		len += sprintf(buf + len, "dma_adc.count = %x\n",
			       s->dma_adc.count);
	}

	/* print out CODEC state */
	len += sprintf(buf + len, "\nAC97 CODEC registers\n");
	len += sprintf(buf + len, "----------------------\n");
	for (cnt = 0; cnt <= 0x7e; cnt = cnt + 2)
		len += sprintf(buf + len, "reg %02x = %04x\n",
			       cnt, rdcodec(s->codec, cnt));

	if (fpos >= len) {
		*start = buf;
		*eof = 1;
		return 0;
	}
	*start = buf + fpos;
	if ((len -= fpos) > length)
		return length;
	*eof = 1;
	return len;

}
#endif				/* vr5701_AC97_DEBUG */

static unsigned int devindex;

MODULE_AUTHOR("Sergey Podstavin");
MODULE_DESCRIPTION("NEC Electronics Corporation VR5701 SolutionGearII audio (AC97) Driver");
MODULE_LICENSE("GPL");

static int __devinit vr5701_ac97_probe(struct pci_dev *pcidev,
				       const struct pci_device_id *pciid)
{
	struct vr5701_ac97_state *s;
#ifdef vr5701_AC97_DEBUG
	char proc_str[80];
#endif

	if (pcidev->irq == 0)
		return -1;

	if (!(s = kmalloc(sizeof(struct vr5701_ac97_state), GFP_KERNEL))) {
		printk(KERN_ERR PFX "alloc of device struct failed\n");
		return -1;
	}
	memset(s, 0, sizeof(struct vr5701_ac97_state));

	init_waitqueue_head(&s->dma_adc.wait);
	init_waitqueue_head(&s->dma_dac.wait);
	init_waitqueue_head(&s->open_wait);
	init_MUTEX(&s->open_sem);
	spin_lock_init(&s->lock);

	s->dev = pcidev;
	s->io = pci_resource_start(pcidev, 0);
	s->irq = pcidev->irq;

	s->codec = ac97_alloc_codec();

	s->codec->private_data = s;
	s->codec->id = 0;
	s->codec->codec_read = rdcodec;
	s->codec->codec_write = wrcodec;
	s->codec->codec_wait = waitcodec;

	/* setting some other default values such as
	 * adcChannels, adcRate is done in open() so that
	 * no persistent state across file opens.
	 */

	if (!request_region(s->io, pci_resource_len(pcidev, 0),
			    vr5701_AC97_MODULE_NAME)) {
		printk(KERN_ERR PFX "io ports %#lx->%#lx in use\n",
		       s->io, s->io + pci_resource_len(pcidev, 0) - 1);
		goto err_region;
	}
	if (request_irq(s->irq, vr5701_ac97_interrupt, SA_INTERRUPT,
			vr5701_AC97_MODULE_NAME, s)) {
		printk(KERN_ERR PFX "irq %u in use\n", s->irq);
		goto err_irq;
	}

	printk(KERN_INFO PFX "IO at %#lx, IRQ %d\n", s->io, s->irq);

	/* register devices */
	if ((s->dev_audio =
	     register_sound_dsp(&vr5701_ac97_audio_fops, -1)) < 0)
		goto err_dev1;
	if ((s->codec->dev_mixer =
	     register_sound_mixer(&vr5701_ac97_mixer_fops, -1)) < 0)
		goto err_dev2;

#ifdef vr5701_AC97_DEBUG
	/* initialize the debug proc device */
	s->ps = create_proc_read_entry(vr5701_AC97_MODULE_NAME, 0, NULL,
				       proc_vr5701_ac97_dump, NULL);
#endif				/* vr5701_AC97_DEBUG */

	/* enable pci io and bus mastering */
	if (pci_enable_device(pcidev))
		goto err_dev3;
	pci_set_master(pcidev);

	/* cold reset the AC97 */
	outl(vr5701_ACLINK_CTRL_RST_ON | vr5701_ACLINK_CTRL_RST_TIME,
	     s->io + vr5701_ACLINK_CTRL);
	while (inl(s->io + vr5701_ACLINK_CTRL) & vr5701_ACLINK_CTRL_RST_ON) ;

	/* codec init */
	if (!ac97_probe_codec(s->codec))
		goto err_dev3;

#ifdef vr5701_AC97_DEBUG
	sprintf(proc_str, "driver/%s/%d/ac97",
		vr5701_AC97_MODULE_NAME, s->codec->id);
	s->ac97_ps = create_proc_read_entry(proc_str, 0, NULL,
					    ac97_read_proc, s->codec);
	/* TODO : why this proc file does not show up? */
#endif

	/* Try to enable variable rate audio mode. */
	wrcodec(s->codec, AC97_EXTENDED_STATUS,
		rdcodec(s->codec, AC97_EXTENDED_STATUS) | AC97_EXTSTAT_VRA);
	/* Did we enable it? */
	if (rdcodec(s->codec, AC97_EXTENDED_STATUS) & AC97_EXTSTAT_VRA)
		s->extended_status |= AC97_EXTSTAT_VRA;
	else {
		s->dacRate = 48000;
		printk(KERN_INFO PFX "VRA mode not enabled; rate fixed at %d.",
		       s->dacRate);
	}

	/* let us get the default volumne louder */
	wrcodec(s->codec, 0x2, 0x1010);	/* master volume, middle */
	wrcodec(s->codec, 0xc, 0x10);	/* phone volume, middle */
	wrcodec(s->codec, 0x10, 0x8000);	/* line-in 2 line-out disable */
	wrcodec(s->codec, 0x18, 0x0707);	/* PCM out (line out) middle */

	/* by default we select line in the input */
	wrcodec(s->codec, 0xe, 0x10);	/* misc volume, middle */
	wrcodec(s->codec, 0x1a, 0x0000);	/* default line is Line_mic */
	/*	wrcodec(s->codec, 0x1a, 0x0404); *//* default line is Line_in */
	wrcodec(s->codec, 0x1c, 0x0f0f);
	wrcodec(s->codec, 0x1e, 0x07);

	/* enable the master interrupt but disable all others */
	outl(vr5701_INT_MASK_NMASK, s->io + vr5701_INT_MASK);

	/* store it in the driver field */
	pci_set_drvdata(pcidev, s);
	pcidev->dma_mask = 0xffffffff;
	/* put it into driver list */
	list_add_tail(&s->devs, &devs);
	/* increment devindex */
	if (devindex < NR_DEVICE - 1)
		devindex++;
	return 0;

      err_dev3:
	unregister_sound_mixer(s->codec->dev_mixer);
      err_dev2:
	unregister_sound_dsp(s->dev_audio);
      err_dev1:
	printk(KERN_ERR PFX "cannot register misc device\n");
	free_irq(s->irq, s);

      err_irq:
	release_region(s->io, pci_resource_len(pcidev, 0));
      err_region:
	ac97_release_codec(s->codec);
	kfree(s);
	return -1;
}

static void __devexit vr5701_ac97_remove(struct pci_dev *dev)
{
	struct vr5701_ac97_state *s = pci_get_drvdata(dev);

	if (!s)
		return;
	list_del(&s->devs);

#ifdef vr5701_AC97_DEBUG
	if (s->ps)
		remove_proc_entry(vr5701_AC97_MODULE_NAME, NULL);
#endif				/* vr5701_AC97_DEBUG */

	synchronize_irq();
	free_irq(s->irq, s);
	release_region(s->io, pci_resource_len(dev, 0));
	unregister_sound_dsp(s->dev_audio);
	unregister_sound_mixer(s->codec->dev_mixer);
	ac97_release_codec(s->codec);
	kfree(s);
	pci_set_drvdata(dev, NULL);
}

static struct pci_device_id id_table[] = {
	{PCI_VENDOR_ID_NEC, PCI_DEVICE_ID_NEC_VRC5477_AC97,
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0},
	{0,}
};

MODULE_DEVICE_TABLE(pci, id_table);

static struct pci_driver vr5701_ac97_driver = {
	.name = vr5701_AC97_MODULE_NAME,
	.id_table = id_table,
	.probe = vr5701_ac97_probe,
	.remove = __devexit_p(vr5701_ac97_remove)
};

static int __init init_vr5701_ac97(void)
{
	printk(KERN_INFO "NEC Electronics Corporation VR5701 SolutionGearII AC97 driver: time " __TIME__ " " __DATE__
	       " by Sergey Podstavin\n");
	return pci_module_init(&vr5701_ac97_driver);
}

static void __exit cleanup_vr5701_ac97(void)
{
	printk(KERN_INFO PFX "unloading\n");
	pci_unregister_driver(&vr5701_ac97_driver);
}

module_init(init_vr5701_ac97);
module_exit(cleanup_vr5701_ac97);
