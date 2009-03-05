/*
 *  linux/include/asm-arm/arch-davinci/preempt.h
 *
 *  BRIEF MODULE DESCRIPTION
 *      DAVINCI Virtual preempt definitions
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

#ifndef __ASM_ARCH_PREEMPT_H
#define __ASM_ARCH_PREEMPT_H

#include <asm/hrtime.h> /* for cycles-to-nsec macros */

static inline unsigned long clock_diff(unsigned long start, unsigned long stop)
{
        return (stop - start);
}

/*
 * DaVinci timers run at clk_ref = 27MHz
 */
#define TICKS_PER_USEC		27
#define ARCH_PREDEFINES_TICKS_PER_USEC
#define readclock()		davinci_get_cycles()
#define INTERRUPTS_ENABLED(x)   (!(x & PSR_I_BIT))

static inline u32 clock_to_usecs(u32 cycles) {
	u32 nsec = arch_cycle_to_nsec(cycles);
	return nsec / 1000;
}
#endif				/* __ASM_ARCH_PREEMPT_H */
