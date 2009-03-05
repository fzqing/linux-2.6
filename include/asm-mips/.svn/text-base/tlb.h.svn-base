#ifndef __ASM_TLB_H
#define __ASM_TLB_H

/*
 * MIPS doesn't need any special per-pte or per-vma handling, except
 * we need to flush cache for area to be unmapped.
 */
#ifdef CONFIG_PREEMPT_RT
#if defined(CONFIG_CPU_MIPS32) || defined(CONFIG_CPU_R5500) || defined(CONFIG_CPU_TX49XX) || defined(CONFIG_CPU_VR41XX)
/*
 * We need the cache flush in case of such processors, eg. MIPS Malta
 */
#define tlb_start_vma(tlb, vma)					\
		do {						\
			flush_cache_range(vma, vma->vm_start, vma->vm_end); \
		} while (0)
#else /* CONFIG_CPU_MIPS32 */
#define tlb_start_vma(tlb, vma)		do { } while (0)
#endif /* CONFIG_CPU_MIPS32 */
#else
#define tlb_start_vma(tlb, vma) 				\
	do {							\
		if (!tlb->fullmm)				\
			flush_cache_range(vma, vma->vm_start, vma->vm_end); \
	}  while (0)
#endif

#define tlb_end_vma(tlb, vma) do { } while (0)
#define __tlb_remove_tlb_entry(tlb, ptep, address) do { } while (0)

/*
 * .. because we flush the whole mm when it fills up.
 */
#define tlb_flush(tlb)	flush_tlb_mm(tlb_mm(tlb))

#ifdef CONFIG_PREEMPT_RT
#include <asm-generic/tlb-simple.h>
#else
#include <asm-generic/tlb.h>
#endif

#endif /* __ASM_TLB_H */
