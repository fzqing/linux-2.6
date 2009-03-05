/*
 * linux/arch/arm/mach-omap/dsp/fifo.h
 *
 * FIFO buffer operators
 *
 * Copyright (C) 2002-2004 Nokia Corporation
 *
 * Written by Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * 2004/06/29:  DSP Gateway version 3.1
 */

struct fifo_struct {
	char *buf;
	size_t sz;
	size_t cnt;
	unsigned int wp;
};

static inline int alloc_fifo(struct fifo_struct *fifo, size_t sz)
{
	if ((fifo->buf = kmalloc(sz, GFP_KERNEL)) == NULL)
		return -ENOMEM;
	fifo->sz = sz;
	fifo->cnt = 0;
	fifo->wp = 0;
	return 0;
}

static inline void free_fifo(struct fifo_struct *fifo)
{
	if (fifo->buf == NULL)
		return;

	kfree(fifo->buf);
	fifo->buf = NULL;
	fifo->sz = 0;
}

static inline void flush_fifo(struct fifo_struct *fifo)
{
	fifo->cnt = 0;
	fifo->wp = 0;
}

#define fifo_empty(fifo)	((fifo)->cnt == 0)

static inline void write_word_to_fifo(struct fifo_struct *fifo,
				      unsigned short word)
{
	*(unsigned short *)&fifo->buf[fifo->wp] = word;
	if ((fifo->wp += 2) == fifo->sz)
		fifo->wp = 0;
	if ((fifo->cnt += 2) > fifo->sz)
		fifo->cnt = fifo->sz;
}

/*
 * (before)
 *
 * [*******----------*************]
 *         ^wp
 *  <---------------------------->  sz = 30
 *  <----->          <----------->  cnt = 20
 *
 * (read: count=16)
 *  <->              <----------->  count = 16
 *                   <----------->  cnt1 = 13
 *                   ^rp
 *
 * (after)
 * [---****-----------------------]
 *         ^wp
 */
static inline size_t copy_to_user_fm_fifo(char *dst, struct fifo_struct *fifo,
					  size_t count)
{
	int rp;

	/* fifo size can be zero */
	if (fifo->sz == 0)
		return 0;

	if (count > fifo->cnt)
		count = fifo->cnt;

	if ((rp = fifo->wp - fifo->cnt) >= 0) {
		/* valid area is straight */
		if (copy_to_user(dst, &fifo->buf[rp], count))
			return -EFAULT;
	} else {
		int cnt1 = -rp;
		rp += fifo->sz;
		if (cnt1 >= count) {
			/* requested area is straight */
			if (copy_to_user(dst, &fifo->buf[rp], count))
				return -EFAULT;
		} else {
			if (copy_to_user(dst, &fifo->buf[rp], cnt1))
				return -EFAULT;
			if (copy_to_user(dst+cnt1, fifo->buf, count-cnt1))
				return -EFAULT;
		}
	}
	fifo->cnt -= count;

	return count;
}
