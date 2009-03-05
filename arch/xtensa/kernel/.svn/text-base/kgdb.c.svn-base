/*
 * arch/xtensa/gdb-stub.c
 *
 * Derived from MIPS.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 - 2002 Tensilica Inc.
 *	Authors:	Scott Foehner <sfoehner@yahoo.com>
 *
 * Copyright (C) 2004-2005 MontaVista Software Inc.
 *  Author: Manish Lachwani, mlachwani@mvista.com
 */

/*
 *  To enable debugger support, two things need to happen.  One, a
 *  call to set_debug_traps() is necessary in order to allow any breakpoints
 *  or error conditions to be properly intercepted and reported to gdb.
 *  Two, a breakpoint needs to be generated to begin communication.  This
 *  is most easily accomplished by a call to breakpoint().  Breakpoint()
 *  simulates a breakpoint by executing a BREAK instruction.
 *
 *
 *    The following gdb commands are supported:
 *
 * command          function                               Return value
 *
 *    pNN..NN       return the value of register           hex data or ENN
 *                    number NN..NN 
 *    PNN..NN:V..V  set the value of register number       OK or ENN
 *                    NN..NN to value V..V
 *    g             return the value of the CPU registers  hex data or ENN
 *    G             set the value of the CPU registers     OK or ENN
 *
 *    mAA..AA,LLLL  Read LLLL bytes at address AA..AA      hex data or ENN
 *    MAA..AA,LLLL: Write LLLL bytes at address AA.AA      OK or ENN
 *
 *    c             Resume at current address              SNN   ( signal NN)
 *    cAA..AA       Continue at address AA..AA             SNN
 *
 *    s             Step one instruction                   SNN
 *    sAA..AA       Step one instruction from AA..AA       SNN
 *
 *    k             kill -- ignored
 *
 *    D             detach -- resume execution
 *
 *    ?             What was the last sigval ?             SNN   (signal NN)
 *
 *
 * All commands and responses are sent with a packet which includes a
 * checksum.  A packet consists of
 *
 * $<packet info>#<checksum>.
 *
 * where
 * <packet info> :: <characters representing the command or response>
 * <checksum>    :: < two hex digits computed as modulo 256 sum of <packetinfo>>
 *
 * When a packet is received, it is first acknowledged with either '+' or '-'.
 * '+' indicates a successful transfer.  '-' indicates a failed transfer.
 *
 * Example:
 *
 * Host:                  Reply:
 * $m0,10#2a               +$00010203040506070809101112131415#42
 *
 */

#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/kgdb.h>
#include <linux/interrupt.h>

#include <asm/byteorder.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/ptrace.h>
#include <asm/cacheflush.h>
#include <asm/kgdb.h>

/*
 * breakpoint and test functions
 */
extern void trap_low(void);
extern void breakinst(void);
extern void init_IRQ(void);

/*
 * local prototypes
 */
void handle_exception(struct pt_regs *regs);

/*
 * local functions
 */

/* Use EPS[DEBUGLEVEL]. */

static inline unsigned long get_ps(void)
{
	unsigned long ps;
	__asm__ __volatile__ (" rsr %0, "__stringify(EPS)
			      "+"__stringify(XCHAL_DEBUGLEVEL)
			      : "=a" (ps));
	return ps;
}

static inline void set_ps(unsigned long ps)
{
	__asm__ __volatile__ (" wsr %0, "__stringify(EPS)
			      "+"__stringify(XCHAL_DEBUGLEVEL)
			      : : "a" (ps));
}

static inline unsigned long get_excsave(void)
{
	unsigned long excsave;
	__asm__ __volatile__ (" rsr %0, "__stringify(EXCSAVE)
			      "+"__stringify(XCHAL_DEBUGLEVEL)
			      : "=a" (excsave));
	return excsave;
}
static inline void set_excsave(unsigned long excsave)
{
	__asm__ __volatile__ (" wsr %0, "__stringify(EXCSAVE)
			      "+"__stringify(XCHAL_DEBUGLEVEL)
			      : : "a" (excsave));
}

void regs_to_gdb_regs(unsigned long *gdb_regs, struct pt_regs *regs)
{
	struct xtensa_gdb_registers *gregs;
	int reg;

	gregs = (struct xtensa_gdb_registers*)gdb_regs;

	/* Clear gdb register structure. */

	memset(gregs, sizeof (gregs), 0);

	/* Copy address register values. */

	if (user_mode(regs)) {

		unsigned long *areg = &gregs->ar0;
		unsigned long wb = regs->windowbase;
		const int frames = regs->wmask >> 4;

		/* Copy the current 16 registers */
		
		for (reg = 0; reg < 16; reg++) {
			int idx = (reg + wb * 4) & (XCHAL_NUM_AREGS - 1);
			areg[idx] = regs->areg[reg];
		}

		/* Copy the remaining registers. */

		for (reg = 1; reg <= frames * 4; reg++) {
			int idx = (wb * 4 - reg) & (XCHAL_NUM_AREGS - 1);
			areg[idx] = regs->areg[XCHAL_NUM_AREGS - reg];
		}

		gregs->windowbase = regs->windowbase;
		gregs->windowstart = regs->windowstart;

	} else {

		/*
		 * All register windows have been flushd, 
		 * so we only have to copy 16 regs.
		 * Windowsbase and windowstart weren't saved.
		 */

	 	unsigned long *areg = &gregs->ar0;

		for (reg = 0; reg < 16; reg++)
			areg[reg] = regs->areg[reg];
		gregs->windowbase = 0;
		gregs->windowstart = 1;
	}

	gregs->lbeg = regs->lbeg;
	gregs->lend = regs->lend;
	gregs->lcount = regs->lcount;

	gregs->exccause = get_excsave();
	gregs->depc = regs->depc;
	gregs->excvaddr = regs->excvaddr;
	gregs->sar = regs->sar;

	gregs->pc = regs->pc;
	gregs->ps = get_ps();

	return;
}

void gdb_regs_to_regs(unsigned long *gdb_regs, struct pt_regs *regs)
{
	struct xtensa_gdb_registers *gregs;
	int reg;

	gregs = (struct xtensa_gdb_registers*)gdb_regs;

	/* Copy address register values. */
	
	if (user_mode(regs)) {

		unsigned long *areg = &gregs->ar0;
		unsigned long wb = gregs->windowbase;

		/* Copy all registers */
		
		for (reg = 0; reg < XCHAL_NUM_AREGS; reg++) {
			int idx = (reg + wb *4) & (XCHAL_NUM_AREGS - 1);
			regs->areg[reg] = areg[idx];
		}

		regs->windowbase = gregs->windowbase;
		regs->windowstart = gregs->windowstart;

	} else {

		/*
		 * All register windows have been flushd, 
		 * so we only have to copy 16 regs.
		 * Windowsbase and windowstart weren't saved.
		 */

	 	unsigned long *areg = &gregs->ar0;

		for (reg = 0; reg < 16; reg++)
			regs->areg[reg] = areg[reg];
	}

	regs->lbeg = gregs->lbeg;
	regs->lend = gregs->lend;
	regs->lcount = gregs->lcount;

	regs->exccause = gregs->exccause;
	regs->depc = gregs->depc;
	regs->excvaddr = gregs->excvaddr;
	regs->sar = gregs->sar;

	regs->pc = gregs->pc;
	set_ps(gregs->ps);

	return;


#if 0
	unsigned long *ptr = gdb_regs;
	int reg = 0;

	for (reg = 0; reg <= 15 ; reg++)
                regs->areg[reg] = *(ptr++);

	for (reg = 16; reg <= 63; reg++)
		ptr++;

	regs->lbeg = *(ptr++);
	regs->lend = *(ptr++);
	regs->lcount = *(ptr++);

	for (reg = 0; reg <= 21; reg++)
		ptr++;

	regs->exccause = *(ptr++);
	regs->depc = *(ptr++);
	regs->excvaddr = *(ptr++);
	regs->windowbase = *(ptr++);
	regs->windowstart = *(ptr++);
	regs->sar = *(ptr++);

	ptr++;

	regs->ps = *(ptr++);

	for (reg = 0; reg <= 9; reg++)
		ptr++;

	regs->icountlevel = *(ptr++);
	regs->debugcause = *(ptr++);

	ptr++;
	ptr++;
	ptr++;

	regs->pc = *(ptr++);

	for (reg = 0; reg <= 31; reg++)
		ptr++;

	return;
#endif
}

int kgdb_fault_setjmp(unsigned long *curr_context)
{
	int ret = 0;

	ret = kgdb_fault_setjmp_aux(curr_context);
	return ret;
}

int kgdb_read_reg(unsigned long regno, char *output_buffer,struct pt_regs *regs)
{
	int ar_reg;
	extern int find_first_pane(int, int);

	switch(regno) {
		case REG_GDB_AR_BASE ... REG_GDB_AR_BASE + XCHAL_NUM_AREGS - 1:
                     ar_reg = ((regno - regs->windowbase * 4 - REG_GDB_AR_BASE) & (XCHAL_NUM_AREGS - 1));

                                kgdb_mem2hex((char *)&(regs->areg[ar_reg]),
                                        output_buffer, 4);
				break;

		case REG_GDB_PC:
                            kgdb_mem2hex((char *)&(regs->pc), output_buffer, 4);
                        break;
                                                                                      
                case REG_GDB_PS:
			/* Re-create PS, set WOE and keep PS.CALLING */
			{
                            unsigned long ps = get_ps();
			    kgdb_mem2hex((char *)&ps, output_buffer, 4);
			}
                        break;
                                                                                      
                case REG_GDB_WB:
                            kgdb_mem2hex((char *)&(regs->windowbase), output_buffer, 4);
                        break;
                                                                                      
                case REG_GDB_WS:
                            kgdb_mem2hex((char *)&(regs->windowstart), output_buffer, 4);
                        break;
                                                                                      
                case REG_GDB_LBEG:
                            kgdb_mem2hex((char *)&(regs->lbeg), output_buffer, 4);
                        break;
                                                                                      
                case REG_GDB_LEND:
                            kgdb_mem2hex((char *)&(regs->lend), output_buffer, 4);
                        break;
                                                                                      
                case REG_GDB_LCOUNT:
                            kgdb_mem2hex((char *)&(regs->lcount), output_buffer, 4);
                        break;
                                                                                      
                case REG_GDB_SAR:
                            kgdb_mem2hex((char *)&(regs->sar), output_buffer, 4);
                        break;
		case REG_GDB_DEPC:
                            kgdb_mem2hex((char *)&(regs->depc), output_buffer, 4);
                        break;
                                                                                      
                case REG_GDB_EXCCAUSE:
                            kgdb_mem2hex((char *)&(regs->exccause), output_buffer, 4);
                        break;
                                                                                      
                case REG_GDB_EXCVADDR:
                            kgdb_mem2hex((char *)&(regs->excvaddr), output_buffer, 4);
                        break;
                                                                                      
                default:
                            strcpy(output_buffer,"ffffffff");
                        break;
              }

	output_buffer[0] = 0;
	return -1;
}
		

void sleeping_thread_to_gdb_regs(unsigned long *gdb_regs, struct task_struct *p)
{
	unsigned long tos;
	struct pt_regs *regs;

	tos = (unsigned long)p->thread_info + THREAD_SIZE;
	regs = (struct pt_regs*)tos - 1;

	regs_to_gdb_regs(gdb_regs, regs);
}


/*
 * Set up exception handlers for tracing and breakpoints
 *
 * Linux/Xtensa has a dedicated debug handler that is always there, so we don't
 * have to set up a handler here.
 */
void set_debug_traps(void)
{
	unsigned long flags;

	save_and_cli(flags);

	restore_flags(flags);
}

/*
 * Set up exception handlers for tracing and breakpoints
 */

void handle_exception(struct pt_regs *regs)
{
	int sigval = 5;

	if (regs->pc == (unsigned long)breakinst) {
#if XCHAL_HAVE_DENSITY
		regs->pc += 2;  /* breakpoint() uses a 3-byte break insn */
#else 
		regs->pc += 3;
#endif
	}
	/* In kernel space, We need to spill the register windows. */

	if (!user_mode(regs)) {
		spill_registers();
		regs->windowbase = 0;
		regs->windowstart = 1;
	}


	kgdb_handle_exception(0, sigval, 0, regs);
}

/*
 * This function does all command processing for interfacing to gdb.  It
 * returns 1 if you should skip the instruction at the trap address, 0
 * otherwise.
 */
int kgdb_arch_handle_exception(int vector, int sigval, int err_code,
				char *input_buffer, char *output_buffer,
				struct pt_regs *regs)
{
	char *ptr;
	unsigned long address;
	int cpu = smp_processor_id ();
	int ret = 0;


	switch (input_buffer[0])
	{
		/*
		 * cAA..AA    Continue at address AA..AA(optional)
		 */
		case 'c':    
			ptr = &input_buffer[1];

			if (kgdb_hex2long (&ptr, &address))
				regs->pc = address;

			atomic_set(&cpu_doing_single_step,-1);

			regs->icountlevel = 0;

			flush_cache_all();

			return 0;

		/*
		 * Step to next instruction
		 */
		case 's':
                        atomic_set(&cpu_doing_single_step,-1);

                        if (kgdb_contthread)
                             atomic_set(&cpu_doing_single_step, cpu);

			regs->icountlevel = (regs->ps & 0xf) + 1;

			return 0;

		case 'p':
                         ptr = &input_buffer[1];
                         kgdb_hex2long(&ptr, &address);
                         ret = kgdb_read_reg(address,output_buffer,regs);

			return ret;
	}
	output_buffer[0] = 0;
	return -1;
}

#ifdef __XTENSA_EB__

/*
 * Big Endian
 */
struct kgdb_arch arch_kgdb_ops = {

#if XCHAL_HAVE_DENSITY
        .gdb_bpt_instr = {0xd2,0x0f},
#else
        .gdb_bpt_instr = {0x00, 0x04, 0x00},
#endif

};

#else

/*
 * Little Endian
 */
struct kgdb_arch arch_kgdb_ops = {


#if XCHAL_HAVE_DENSITY
        .gdb_bpt_instr = {0x2d,0xf0}
#else
        .gdb_bpt_instr = {0x00, 0x40, 0x00},
#endif


};

#endif

/*
 * We use kgdb_early_setup so that functions we need to call now don't
 * cause trouble when called again later.
 */
int kgdb_arch_init(void)
{
	if (kgdb_early_setup == 0) {
		trap_init();
		init_IRQ();
		kgdb_early_setup = 1;
	}

	set_debug_traps();

	return 0;
}

