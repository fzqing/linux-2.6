#ifndef _LINUX_TTY_FLIP_H
#define _LINUX_TTY_FLIP_H

#ifdef INCLUDE_INLINE_FUNCS
#define _INLINE_ extern
#else
#define _INLINE_ static __inline__
#endif
#include <linux/jiffies.h>

_INLINE_ void tty_insert_flip_char(struct tty_struct *tty,
				   unsigned char ch, char flag)
{
	unsigned long   flags = 0;
	spin_lock_irqsave(&tty->flip_buf_lock, flags);
	if (tty->flip.count < TTY_FLIPBUF_SIZE) {
		tty->flip.count++;
		*tty->flip.flag_buf_ptr++ = flag;
		*tty->flip.char_buf_ptr++ = ch;
	}
	spin_unlock_irqrestore(&tty->flip_buf_lock, flags);
}

_INLINE_ void tty_schedule_flip(struct tty_struct *tty)
{
	schedule_delayed_work(&tty->flip.work, 1);
}

#undef _INLINE_


#endif /* _LINUX_TTY_FLIP_H */







