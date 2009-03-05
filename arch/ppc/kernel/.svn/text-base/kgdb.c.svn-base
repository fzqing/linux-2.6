/*
 * arch/ppc/kernel/kgdb.c
 *
 * PowerPC backend to the KGDB stub.
 *
 * Maintainer: Tom Rini <trini@kernel.crashing.org>
 *
 * 1998 (c) Michael AK Tesch (tesch@cs.wisc.edu)
 * Copyright (C) 2003 Timesys Corporation.
 * 2004 (c) MontaVista Software, Inc.
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
#include <asm/thread_info.h>

/*
 * This table contains the mapping between PowerPC hardware trap types, and
 * signals, which are primarily what GDB understands.  GDB and the kernel
 * don't always agree on values, so we use constants taken from gdb-6.2.
 */
static struct hard_trap_info
{
	unsigned int tt;		/* Trap type code for powerpc */
	unsigned char signo;		/* Signal that we map this trap into */
} hard_trap_info[] = {
#if defined(CONFIG_40x) || defined(CONFIG_BOOKE)
	{ 0x0100, 0x02 /* SIGINT */  },		/* critical input interrupt */
	{ 0x0200, 0x0b /* SIGSEGV */ },		/* machine check */
	{ 0x0300, 0x0b /* SIGSEGV */ },		/* data storage */
	{ 0x0400, 0x0a /* SIGBUS */  },		/* instruction storage */
	{ 0x0500, 0x02 /* SIGINT */  },		/* interrupt */
	{ 0x0600, 0x0a /* SIGBUS */  },		/* alignment */
	{ 0x0700, 0x04 /* SIGILL */  },		/* program */
	{ 0x0800, 0x04 /* SIGILL */  },		/* reserved */
	{ 0x0900, 0x04 /* SIGILL */  },		/* reserved */
	{ 0x0a00, 0x04 /* SIGILL */  },		/* reserved */
	{ 0x0b00, 0x04 /* SIGILL */  },		/* reserved */
	{ 0x0c00, 0x14 /* SIGCHLD */ },		/* syscall */
	{ 0x0d00, 0x04 /* SIGILL */  },		/* reserved */
	{ 0x0e00, 0x04 /* SIGILL */  },		/* reserved */
	{ 0x0f00, 0x04 /* SIGILL */  },		/* reserved */
	{ 0x2002, 0x05 /* SIGTRAP */},		/* debug */
#else
	{ 0x0200, 0x0b /* SIGSEGV */ },		/* machine check */
	{ 0x0300, 0x0b /* SIGSEGV */ },		/* address error (store) */
	{ 0x0400, 0x0a /* SIGBUS */ },		/* instruction bus error */
	{ 0x0500, 0x02 /* SIGINT */ },		/* interrupt */
	{ 0x0600, 0x0a /* SIGBUS */ },		/* alingment */
	{ 0x0700, 0x05 /* SIGTRAP */ },		/* breakpoint trap */
	{ 0x0800, 0x08 /* SIGFPE */},		/* fpu unavail */
	{ 0x0900, 0x0e /* SIGALRM */ },		/* decrementer */
	{ 0x0a00, 0x04 /* SIGILL */ },		/* reserved */
	{ 0x0b00, 0x04 /* SIGILL */ },		/* reserved */
	{ 0x0c00, 0x14 /* SIGCHLD */ },		/* syscall */
	{ 0x0d00, 0x05 /* SIGTRAP */ },		/* single-step/watch */
	{ 0x0e00, 0x08 /* SIGFPE */ },		/* fp assist */
#endif
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

/* KGDB functions to use existing PowerPC hooks. */
static void kgdb_debugger (struct pt_regs *regs)
{
	kgdb_handle_exception(0, computeSignal(regs->trap), 0, regs);
}

static int kgdb_breakpoint (struct pt_regs *regs)
{
	if (user_mode(regs))
		return 0;

	kgdb_handle_exception(0, SIGTRAP, 0, regs);

	if (atomic_read (&kgdb_setting_breakpoint))
		regs->nip += 4;

	return 1;
}

static int kgdb_singlestep (struct pt_regs *regs)
{
	struct thread_info *thread_info, *exception_thread_info;

	if (user_mode(regs))
		return 0;
	/*
	 * On Book E and perhaps other processsors, singlestep is handled on
	 * the critical exception stack.  This causes current_thread_info()
	 * to fail, since it it locates the thread_info by masking off
	 * the low bits of the current stack pointer.  We work around
	 * this issue by copying the thread_info from the kernel stack
	 * before calling kgdb_handle_exception, and copying it back
	 * afterwards.  On most processors the copy is avoided since
	 * exception_thread_info == thread_info.
	 */
	thread_info = (struct thread_info *)(regs->gpr[1] & ~(THREAD_SIZE-1));
	exception_thread_info = current_thread_info();

	if (thread_info != exception_thread_info)
		memcpy(exception_thread_info, thread_info, sizeof *thread_info);

	kgdb_handle_exception(0, SIGTRAP, 0, regs);

	if (thread_info != exception_thread_info)
		memcpy(thread_info, exception_thread_info, sizeof *thread_info);

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

void regs_to_gdb_regs(unsigned long *gdb_regs, struct pt_regs *regs)
{
	int reg;
	unsigned long *ptr = gdb_regs;

	memset(gdb_regs, 0, MAXREG*4);

	for (reg = 0; reg < 32; reg++)
		*(ptr++) = regs->gpr[reg];

#ifndef CONFIG_E500
	for (reg = 0; reg < 64; reg++)
		*(ptr++) = 0;
#else
	for (reg = 0; reg < 32; reg++)
		*(ptr++) = current->thread.evr[reg];
#endif

	*(ptr++) = regs->nip;
	*(ptr++) = regs->msr;
	*(ptr++) = regs->ccr;
	*(ptr++) = regs->link;
	*(ptr++) = regs->ctr;
	*(ptr++) = regs->xer;

#ifdef CONFIG_SPE
	/* u64 acc */
	*(ptr++) = (current->thread.acc >> 32);
	*(ptr++) = (current->thread.acc & 0xffffffff);
	*(ptr++) = current->thread.spefscr;
#endif
}

void sleeping_thread_to_gdb_regs(unsigned long *gdb_regs, struct task_struct *p)
{
	struct pt_regs *regs = (struct pt_regs *) (p->thread.ksp +
	                                           STACK_FRAME_OVERHEAD);
	int reg;
	unsigned long *ptr = gdb_regs;

	memset(gdb_regs, 0, MAXREG*4);

	/* Regs GPR0-2 */
	for (reg = 0; reg < 3; reg++)
		*(ptr++) = regs->gpr[reg];

	/* Regs GPR3-13 are not saved */
	for (reg = 3; reg < 14; reg++)
		*(ptr++) = 0;

	/* Regs GPR14-31 */
	for (reg = 14; reg < 32; reg++)
		*(ptr++) = regs->gpr[reg];

#ifndef CONFIG_E500
	for (reg = 0; reg < 64; reg++)
		*(ptr++) = 0;
#else
	for (reg = 0; reg < 32; reg++)
		*(ptr++) = current->thread.evr[reg];
#endif

	*(ptr++) = regs->nip;
	*(ptr++) = regs->msr;
	*(ptr++) = regs->ccr;
	*(ptr++) = regs->link;
	*(ptr++) = regs->ctr;
	*(ptr++) = regs->xer;

#ifdef CONFIG_SPE
	/* u64 acc */
	*(ptr++) = (current->thread.acc >> 32);
	*(ptr++) = (current->thread.acc & 0xffffffff);
	*(ptr++) = current->thread.spefscr;
#endif
}

void gdb_regs_to_regs(unsigned long *gdb_regs, struct pt_regs *regs)
{
	int reg;
	unsigned long *ptr = gdb_regs;
#ifdef CONFIG_SPE
	union {
		u32 v32[2];
		u64 v64;
	} u;
#endif

	for (reg = 0; reg < 32; reg++)
		regs->gpr[reg] = *(ptr++);

#ifndef CONFIG_E500
	for (reg = 0; reg < 64; reg++)
		ptr++;
#else
	for (reg = 0; reg < 32; reg++)
		current->thread.evr[reg] = *(ptr++);
#endif

	regs->nip = *(ptr++);
	regs->msr = *(ptr++);
	regs->ccr = *(ptr++);
	regs->link = *(ptr++);
	regs->ctr = *(ptr++);
	regs->xer = *(ptr++);

#ifdef CONFIG_SPE
	/* u64 acc */
	u.v32[0] = *(ptr++);
	u.v32[1] = *(ptr++);
	current->thread.acc = u.v64;
	current->thread.spefscr = *(ptr++);
#endif
}

/*
 * This function does PoerPC specific procesing for interfacing to gdb.
 */
int kgdb_arch_handle_exception (int vector, int signo, int err_code,
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
#if defined (CONFIG_40x) || defined(CONFIG_BOOKE)
				mtspr(SPRN_DBCR0, mfspr(SPRN_DBCR0) | DBCR0_IC);
				linux_regs->msr |= MSR_DE;
#else
				linux_regs->msr |= MSR_SE;
#endif
				debugger_step = 1;
				if (kgdb_contthread)
					atomic_set(&cpu_doing_single_step,
							smp_processor_id());
			}
			return 0;
	}

	return -1;
}

/*
 * Global data
 */
struct kgdb_arch arch_kgdb_ops =
{
	.gdb_bpt_instr = { 0x7d, 0x82, 0x10, 0x08 },
};

int kgdb_arch_init (void)
{
	debugger_ipi = kgdb_call_nmi_hook;
	debugger = kgdb_debugger;
	debugger_bpt = kgdb_breakpoint;
	debugger_sstep = kgdb_singlestep;
	debugger_iabr_match = kgdb_iabr_match;
	debugger_dabr_match = kgdb_dabr_match;

	return 0;
}
arch_initcall(kgdb_arch_init);
