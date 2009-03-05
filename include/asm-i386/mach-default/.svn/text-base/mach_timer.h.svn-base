/*
 *  include/asm-i386/mach-default/mach_timer.h
 *
 *  Machine specific calibrate_tsc() for generic.
 *  Split out from timer_tsc.c by Osamu Tomita <tomita@cinet.co.jp>
 */
/* ------ Calibrate the TSC ------- 
 * Return 2^32 * (1 / (TSC clocks per usec)) for do_fast_gettimeoffset().
 * Too much 64-bit arithmetic here to do this cleanly in C, and for
 * accuracy's sake we want to keep the overhead on the CTC speaker (channel 2)
 * output busy loop as low as possible. We avoid reading the CTC registers
 * directly because of the awkward 8-bit access mechanism of the 82C54
 * device.
 */
#ifndef _MACH_TIMER_H
#define _MACH_TIMER_H

/*
 * Always use a 50ms calibrate time (PIT is only good for ~54ms)
 */
# define CAL_HZ             100
# define CALIBRATE_LATCH    (((CLOCK_TICK_RATE * 5) + 50) / CAL_HZ)
# define CALIBRATE_TIME	  (u32)(((u64)CALIBRATE_LATCH * USEC_PER_SEC)/ CLOCK_TICK_RATE)

#define PIT2_CHAN 0x42
#define PIT_COMMAND_CHAN 0x43

#define PIT_BINARY 0
#define PIT_SELECT2 0x80
#define PIT_RW_2BYTES 0x30
#define PIT2_BINARY PIT_SELECT2 + PIT_RW_2BYTES + PIT_BINARY

#define PIT_RB2     0x04
#define PIT_READBACK 0xc0
#define PIT_LATCH_STATUS 0x20 /* actually means don't latch count */
#define PIT2_CMD_LATCH_STATUS PIT_READBACK + PIT_LATCH_STATUS + PIT_RB2

#define PIT_NULL_COUNT 0x40

extern unsigned long tsc_cycles_per_50_ms;

static inline void mach_prepare_counter(void)
{
	unsigned char pit_status;
       /* Set the Gate high, disable speaker */
	outb((inb(0x61) & ~0x02) | 0x01, 0x61);

	/*
	 * Now let's take care of CTC channel 2
	 *
	 * Set the Gate high, program CTC channel 2 for mode 0,
	 * (interrupt on terminal count mode), binary count,
	 * load 5 * LATCH count, (LSB and MSB) to begin countdown.
	 *
	 * Some devices need a delay here.
	 */
	outb(PIT2_BINARY, PIT_COMMAND_CHAN);/* binary, mode 0, LSB/MSB, Ch 2 */
	outb_p(CALIBRATE_LATCH & 0xff, PIT2_CHAN);	/* LSB of count */
	outb_p(CALIBRATE_LATCH >> 8, PIT2_CHAN);       /* MSB of count */
	do {
		/*
		 * Here we wait for the PIT to actually load the count
		 * Yes, it does take a while.  Remember his clock is only
		 * about 1 MHZ.
		 */
		outb(PIT2_CMD_LATCH_STATUS, PIT_COMMAND_CHAN);
		pit_status = inb(PIT2_CHAN);
	} while (pit_status & PIT_NULL_COUNT);
}

static inline void mach_countup(unsigned long *count_p)
{
	unsigned long count = 0;
	do {
		count++;
	} while ((inb(0x61) & 0x20) == 0);
	*count_p = count;
}

#endif /* !_MACH_TIMER_H */
