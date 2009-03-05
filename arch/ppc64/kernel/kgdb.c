/*
 * arch/ppc64/kernel/kgdb.c
 *
 * PowerPC64 backend to the KGDB stub.
 *
 * Maintainer: Tom Rini <trini@kernel.crashing.org>
 *
 * Copied from arch/ppc/kernel/kgdb.c, updated for ppc64 
 *
 * Copyright (C) 1996 Paul Mackerras (setjmp/longjmp)
 * 1998 (c) Michael AK Tesch (tesch@cs.wisc.edu)
 * Copyright (C) 2003 Timesys Corporation.
 * 2004 (c) MontaVista Software, Inc.
 * 2005 (c) MontaVista Software, Inc.
 * PPC64 Mods (C) 2005 Frank Rowand (frowand@mvista.com)
 * 
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program as licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kgdb.h>
#include <linux/smp.h>
#include <linux/signal.h>
#include <linux/ptrace.h>
#include <asm/current.h>
#include <asm/ptrace.h>
#include <asm/processor.h>
#include <asm/machdep.h>

/*
 * This table contains the mapping between PowerPC64 hardware trap types, and
 * signals, which are primarily what GDB understands.  GDB and the kernel
 * don't always agree on values, so we use constants taken from gdb-6.2.
 */
static struct hard_trap_info
{
	unsigned int tt;		/* Trap type code for powerpc */
	unsigned char signo;		/* Signal that we map this trap into */
} hard_trap_info[] = {
	{ 0x0100, 0x02 /* SIGINT */  },		/* system reset */
	{ 0x0200, 0x0b /* SIGSEGV */ },		/* machine check */
	{ 0x0300, 0x0b /* SIGSEGV */ },		/* data access */
	{ 0x0400, 0x0a /* SIGBUS */  },		/* instruction access */
	{ 0x0480, 0x0a /* SIGBUS */  },		/* instruction segment */
	{ 0x0500, 0x02 /* SIGINT */  },		/* interrupt */
	{ 0x0600, 0x0a /* SIGBUS */  },		/* alignment */
	{ 0x0700, 0x04 /* SIGILL */  },		/* program */
	{ 0x0800, 0x08 /* SIGFPE */  },		/* fpu unavailable */
	{ 0x0900, 0x0e /* SIGALRM */  },	/* decrementer */
	{ 0x0a00, 0x04 /* SIGILL */  },		/* reserved */
	{ 0x0b00, 0x04 /* SIGILL */  },		/* reserved */
	{ 0x0c00, 0x14 /* SIGCHLD */ },		/* syscall */
	{ 0x0d00, 0x05 /* SIGTRAP */  },	/* single step */
	{ 0x0e00, 0x04 /* SIGILL */  },		/* reserved */
	{ 0x0f00, 0x04 /* SIGILL */  },		/* performance monitor */
	{ 0x0f20, 0x08 /* SIGFPE */  },		/* altivec unavailable */
	{ 0x1300, 0x05 /* SIGTRAP */  },	/* instruction address break */
	{ 0x1500, 0x04 /* SIGILL */  },		/* soft patch */
	{ 0x1600, 0x04 /* SIGILL */  },		/* maintenance */
	{ 0x1700, 0x04 /* SIGILL */  },		/* altivec assist */
	{ 0x1800, 0x04 /* SIGILL */  },		/* thermal */
	{ 0x2002, 0x05 /* SIGTRAP */},		/* debug */
	{ 0x0000, 0x000 }			/* Must be last */
};

extern atomic_t cpu_doing_single_step;

static int computeSignal(unsigned int tt)
{
	struct hard_trap_info *ht;

	for (ht = hard_trap_info; ht->tt && ht->signo; ht++)
		if (ht->tt == tt)
			return ht->signo;

	return SIGHUP; /* default for things we don't know about */
}

static int kgdb_call_nmi_hook(struct pt_regs *regs)
{
	kgdb_nmihook(smp_processor_id(), regs);
	return 0;
}

#ifdef CONFIG_SMP
void kgdb_roundup_cpus(unsigned long flags)
{
	smp_send_debugger_break(MSG_ALL_BUT_SELF);
}
#endif

/* KGDB functions to use existing PowerPC64 hooks. */
static int kgdb_debugger (struct pt_regs *regs)
{
	return kgdb_handle_exception(0, computeSignal(regs->trap), 0, regs);
}

static int kgdb_breakpoint (struct pt_regs *regs)
{
	if (user_mode(regs))
		return 0;

	kgdb_handle_exception(0, SIGTRAP, 0, regs);

	if (*(u32 *)(regs->nip) == *(u32 *)(&arch_kgdb_ops.gdb_bpt_instr))
		regs->nip += 4;

	return 1;
}

static int kgdb_singlestep (struct pt_regs *regs)
{
	if (user_mode(regs))
		return 0;

	kgdb_handle_exception(0, SIGTRAP, 0, regs);
	return 1;
}

int kgdb_iabr_match(struct pt_regs *regs)
{
	if (user_mode(regs))
		return 0;

	kgdb_handle_exception(0, computeSignal(regs->trap), 0, regs);
	return 1;
}

int kgdb_dabr_match(struct pt_regs *regs)
{
	if (user_mode(regs))
		return 0;

	kgdb_handle_exception(0, computeSignal(regs->trap), 0, regs);
	return 1;
}

#define PACK64(ptr,src) do { *(ptr++) = (src); } while(0)

#define PACK32(ptr,src) do {          \
	u32 *ptr32;                   \
	ptr32 = (u32 *)ptr;           \
	*(ptr32++) = (src);           \
	ptr = (unsigned long *)ptr32; \
	} while(0)

void regs_to_gdb_regs(unsigned long *gdb_regs, struct pt_regs *regs)
{
	int reg;
	unsigned long *ptr = gdb_regs;

	memset(gdb_regs, 0, NUMREGBYTES);

	for (reg = 0; reg < 32; reg++)
		PACK64(ptr, regs->gpr[reg]);

	/* fp registers not used by kernel, leave zero */
	ptr += 32;

	PACK64(ptr, regs->nip);
	PACK64(ptr, regs->msr);
	PACK32(ptr, regs->ccr);
	PACK64(ptr, regs->link);
	PACK64(ptr, regs->ctr);
	PACK32(ptr, regs->xer);

#if 0
	Following are in struct thread_struct, not struct pt_regs,
	ignoring for now since kernel does not use them.  Would it
	make sense to get them from the thread that kgdb is set to?

	PACK32(ptr, p->thread->fpscr);
	PACK32(ptr, p->thread->vscr);
	PACK32(ptr, p->thread->vrsave);
#else
	/* fpsrc, vscr, vrsave not used by kernel, leave zero */
	PACK32(ptr, 0);
	PACK32(ptr, 0);
	PACK32(ptr, 0);
#endif
	
	BUG_ON((unsigned long)ptr >
	       (unsigned long)(((void *)gdb_regs) + NUMREGBYTES));
}

void sleeping_thread_to_gdb_regs(unsigned long *gdb_regs, struct task_struct *p)
{
	struct pt_regs *regs = (struct pt_regs *) (p->thread.ksp +
	                                           STACK_FRAME_OVERHEAD);
	int reg;
	unsigned long *ptr = gdb_regs;

	memset(gdb_regs, 0, NUMREGBYTES);

	/* Regs GPR0-2 */
	for (reg = 0; reg < 3; reg++)
		PACK64(ptr, regs->gpr[reg]);

	/* Regs GPR3-13 are not saved */
	for (reg = 3; reg < 14; reg++)
		PACK64(ptr, 0);

	/* Regs GPR14-31 */
	for (reg = 14; reg < 32; reg++)
		PACK64(ptr, regs->gpr[reg]);

	/* fp registers not used by kernel, leave zero */
	ptr += 32;

	PACK64(ptr, regs->nip);
	PACK64(ptr, regs->msr);
	PACK32(ptr, regs->ccr);
	PACK64(ptr, regs->link);
	PACK64(ptr, regs->ctr);
	PACK32(ptr, regs->xer);

#if 0
	Following are in struct thread_struct, not struct pt_regs,
	ignoring for now since kernel does not use them.  Would it
	make sense to get them from the thread that kgdb is set to?

	PACK32(ptr, p->thread->fpscr);
	PACK32(ptr, p->thread->vscr);
	PACK32(ptr, p->thread->vrsave);
#else
	/* fpsrc, vscr, vrsave not used by kernel, leave zero */
	PACK32(ptr, 0);
	PACK32(ptr, 0);
	PACK32(ptr, 0);
#endif

	BUG_ON((unsigned long)ptr >
	       (unsigned long)(((void *)gdb_regs) + NUMREGBYTES));
}

#define UNPACK64(dest,ptr) do { dest = *(ptr++); } while(0)

#define UNPACK32(dest,ptr) do {       \
	u32 *ptr32;                   \
	ptr32 = (u32 *)ptr;           \
	dest = *(ptr32++);            \
	ptr = (unsigned long *)ptr32; \
	} while(0)

void gdb_regs_to_regs(unsigned long *gdb_regs, struct pt_regs *regs)
{
	int reg;
	unsigned long *ptr = gdb_regs;

	for (reg = 0; reg < 32; reg++)
		UNPACK64(regs->gpr[reg], ptr);

	/* fp registers not used by kernel, leave zero */
	ptr += 32;

	UNPACK64(regs->nip, ptr);
	UNPACK64(regs->msr, ptr);
	UNPACK32(regs->ccr, ptr);
	UNPACK64(regs->link, ptr);
	UNPACK64(regs->ctr, ptr);
	UNPACK32(regs->xer, ptr);

#if 0
	Following are in struct thread_struct, not struct pt_regs,
	ignoring for now since kernel does not use them.  Would it
	make sense to get them from the thread that kgdb is set to?

	/* fpsrc, vscr, vrsave not used by kernel, leave unchanged */

	UNPACK32(p->thread->fpscr, ptr);
	UNPACK32(p->thread->vscr, ptr);
	UNPACK32(p->thread->vrsave, ptr);
#endif

	BUG_ON((unsigned long)ptr >
	       (unsigned long)(((void *)gdb_regs) + NUMREGBYTES));
}

/*
 * This function does PowerPC64 specific procesing for interfacing to gdb.
 */
int kgdb_arch_handle_exception(int vector, int signo, int err_code,
		char *remcom_in_buffer, char *remcom_out_buffer,
		struct pt_regs *linux_regs)
{
	char *ptr = &remcom_in_buffer[1];
	unsigned long addr;

	switch (remcom_in_buffer[0])
		{
		/*
		 * sAA..AA   Step one instruction from AA..AA
		 * This will return an error to gdb ..
		 */
		case 's':
		case 'c':
			/* handle the optional parameter */
			if (kgdb_hex2long (&ptr, &addr))
				linux_regs->nip = addr;

			atomic_set(&cpu_doing_single_step, -1);
			/* set the trace bit if we're stepping */
			if (remcom_in_buffer[0] == 's') {
				linux_regs->msr |= MSR_SE;
				debugger_step = 1;
				if (kgdb_contthread)
					atomic_set(&cpu_doing_single_step,
							smp_processor_id());
			}
			return 0;
	}

	return -1;
}

int
kgdb_fault_setjmp(unsigned long *curr_context)
{
	__asm__ __volatile__("mflr 0; std 0,0(%0)\n\
			      std	1,8(%0)\n\
			      std	2,16(%0)\n\
			      mfcr 0; std 0,24(%0)\n\
			      std	13,32(%0)\n\
			      std	14,40(%0)\n\
			      std	15,48(%0)\n\
			      std	16,56(%0)\n\
			      std	17,64(%0)\n\
			      std	18,72(%0)\n\
			      std	19,80(%0)\n\
			      std	20,88(%0)\n\
			      std	21,96(%0)\n\
			      std	22,104(%0)\n\
			      std	23,112(%0)\n\
			      std	24,120(%0)\n\
			      std	25,128(%0)\n\
			      std	26,136(%0)\n\
			      std	27,144(%0)\n\
			      std	28,152(%0)\n\
			      std	29,160(%0)\n\
			      std	30,168(%0)\n\
			      std	31,176(%0)\n" : : "r" (curr_context));
	return 0;
}

void
kgdb_fault_longjmp(unsigned long *curr_context)
{
	__asm__ __volatile__("ld	13,32(%0)\n\
	 		      ld	14,40(%0)\n\
			      ld	15,48(%0)\n\
			      ld	16,56(%0)\n\
			      ld	17,64(%0)\n\
			      ld	18,72(%0)\n\
			      ld	19,80(%0)\n\
			      ld	20,88(%0)\n\
			      ld	21,96(%0)\n\
			      ld	22,104(%0)\n\
			      ld	23,112(%0)\n\
			      ld	24,120(%0)\n\
			      ld	25,128(%0)\n\
			      ld	26,136(%0)\n\
			      ld	27,144(%0)\n\
			      ld	28,152(%0)\n\
			      ld	29,160(%0)\n\
			      ld	30,168(%0)\n\
			      ld	31,176(%0)\n\
			      ld	0,24(%0)\n\
			      mtcrf	0x38,0\n\
			      ld	0,0(%0)\n\
			      ld	1,8(%0)\n\
			      ld	2,16(%0)\n\
			      mtlr	0\n\
			      mr	3,1\n" : : "r" (curr_context));
}

/*
 * Global data
 */
struct kgdb_arch arch_kgdb_ops =
{
	.gdb_bpt_instr = { 0x7d, 0x82, 0x10, 0x08 },
};

int kgdb_not_implemented(struct pt_regs *regs)
{
	return 0;
}

int kgdb_arch_init (void)
{
#ifdef CONFIG_XMON
#error Both XMON and KGDB selected in .config.  Unselect one of them.
#endif

	__debugger_ipi = kgdb_call_nmi_hook;
	__debugger = kgdb_debugger;
	__debugger_bpt = kgdb_breakpoint;
	__debugger_sstep = kgdb_singlestep;
	__debugger_iabr_match = kgdb_iabr_match;
	__debugger_dabr_match = kgdb_dabr_match;
	__debugger_fault_handler = kgdb_not_implemented;

	return 0;
}
arch_initcall(kgdb_arch_init);
