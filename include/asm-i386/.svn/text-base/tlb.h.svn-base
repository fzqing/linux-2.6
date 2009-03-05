#ifndef _I386_TLB_H
#define _I386_TLB_H

/*
 * x86 doesn't need any special per-pte or
 * per-vma handling..
 */
#define tlb_start_vma(tlb, vma) do { } while (0)
#define tlb_end_vma(tlb, vma) do { } while (0)
#define __tlb_remove_tlb_entry(tlb, ptep, address) do { } while (0)

/*
 * .. because we flush the whole mm when it
 * fills up.
 */
#define tlb_flush(tlb) flush_tlb_mm(tlb_mm(tlb))

/*
 * The mutex based kernel can preempt anytime so the per-CPU
 * gather structures dont really fit. Fortunately TLB flushing
 * is really simple on x86 ...
 */
#ifndef CONFIG_PREEMPT_RT
# include <asm-generic/tlb.h>
#else
# include <asm-generic/tlb-simple.h>
#endif

#endif
