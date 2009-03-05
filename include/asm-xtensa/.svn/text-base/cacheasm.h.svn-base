#include <cache.h>
#include <asmmacro.h>

/* 
 * Cache macros
 *
 * Locking
 *
 *   __unlock_dcache_all
 *   __unlock_icache_all
 *
 * Flush and invaldating
 *
 *   __flush_invalidate_dcache_{all|range|page} 
 *   __flush_dcache_{all|range|page}
 *   __invalidate_dcache_{all|range|page}
 *   __invalidate_icache_{all|range|page}
 *
 */

	.macro	__loop_cache_all ar at insn size line_width

	movi	\ar, 0

	__loopi	\ar, \at, \size, 4 << (\line_width)
	\insn	\ar, 0 << (\line_width)
	\insn	\ar, 1 << (\line_width)
	\insn	\ar, 2 << (\line_width)
	\insn	\ar, 3 << (\line_width)
	__endla	\ar, \at, 4 << (\line_width)

	.endm


	.macro	__loop_cache_range ar as at insn line_width

	extui	\at, \ar, 0, \line_width
	add	\as, \as, \at

	__loops	\ar, \as, \at, \line_width
	\insn	\ar, 0
	__endla	\ar, \at, (1 << (\line_width))

	.endm


	.macro	__loop_cache_page ar at insn line_width

	__loopi	\ar, \at, PAGE_SIZE, 4 << (\line_width)
	\insn	\ar, 0 << (\line_width)
	\insn	\ar, 1 << (\line_width)
	\insn	\ar, 2 << (\line_width)
	\insn	\ar, 3 << (\line_width)
	__endla	\ar, \at, 4 << (\line_width)

	.endm


#if XCHAL_DCACHE_LINE_LOCKABLE

	.macro	__unlock_dcache_all ar at

	__loop_cache_all \ar \at diu XCHAL_DCACHE_SIZE XCHAL_DCACHE_LINEWIDTH

	.endm


	.macro	__unlock_icache_all ar at

	__loop_cache_all \ar \at iiu XCHAL_ICACHE_SIZE XCHAL_ICACHE_LINEWIDTH

	.endm
#endif

	.macro	__flush_invalidate_dcache_all ar at

	__loop_cache_all \ar \at diwbi XCHAL_DCACHE_SIZE XCHAL_DCACHE_LINEWIDTH

	.endm


	.macro	__flush_dcache_all ar at

	__loop_cache_all \ar \at diwb XCHAL_DCACHE_SIZE XCHAL_DCACHE_LINEWIDTH

	.endm


	.macro	__invalidate_dcache_all ar at

	__loop_cache_all \ar \at dii DCACHE_WAY_SIZE XCHAL_DCACHE_LINEWIDTH

	.endm


	.macro	__invalidate_icache_all ar at

	__loop_cache_all \ar \at iii ICACHE_WAY_SIZE XCHAL_ICACHE_LINEWIDTH

	.endm



	.macro	__flush_invalidate_dcache_range ar as at

	__loop_cache_range \ar \as \at dhwbi XCHAL_DCACHE_LINEWIDTH

	.endm


	.macro	__flush_dcache_range ar as at

	__loop_cache_range \ar \as \at dhwb XCHAL_DCACHE_LINEWIDTH

	.endm


	.macro	__invalidate_dcache_range ar as at

	__loop_cache_range \ar \as \at dhi XCHAL_DCACHE_LINEWIDTH

	.endm


	.macro	__invalidate_icache_range ar as at

	__loop_cache_range \ar \as \at ihi XCHAL_ICACHE_LINEWIDTH

	.endm



	.macro	__flush_invalidate_dcache_page ar as

	__loop_cache_page \ar \as dhwbi XCHAL_DCACHE_LINEWIDTH

	.endm


	.macro __flush_dcache_page ar as

	__loop_cache_page \ar \as dhwb XCHAL_DCACHE_LINEWIDTH

	.endm


	.macro	__invalidate_dcache_page ar as

	__loop_cache_page \ar \as dhi XCHAL_DCACHE_LINEWIDTH

	.endm


	.macro	__invalidate_icache_page ar as

	__loop_cache_page \ar \as ihi XCHAL_ICACHE_LINEWIDTH

	.endm

