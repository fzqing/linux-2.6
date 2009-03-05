/*
 * arch/xtensa/mm/misc.S
 *
 * Miscellaneous assembly functions.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 - 2005 Tensilica Inc.
 *
 * Chris Zankel	<chris@zankel.net>
 */


#include <linux/linkage.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/asmmacro.h>
#include <asm/cacheasm.h>
#include <asm/tlbflush.h>

#define __DCACHE_WAYSIZE	(XCHAL_DCACHE_SIZE / XCHAL_DCACHE_WAYS)
#define __ICACHE_WAYSIZE	(XCHAL_ICACHE_SIZE / XCHAL_ICACHE_WAYS)

/*
 * clear_page (unsigned long page)
 *                    a2
 */

ENTRY(clear_page)
	entry	a1, 16
	movi	a3, 0

	__loopi	a2, a4, PAGE_SIZE, 32
	s32i	a3, a2, 0
	s32i	a3, a2, 4
	s32i	a3, a2, 8
	s32i	a3, a2, 12
	s32i	a3, a2, 16
	s32i	a3, a2, 20
	s32i	a3, a2, 24
	s32i	a3, a2, 28
	__endla	a2, a4, 32

	retw

/*
 * copy_page (void *to, void *from)
 *                  a2        a3
 */

ENTRY(copy_page)
	entry	a1, 16

	__loopi	a2, a4, PAGE_SIZE, 32
	l32i	a5, a3, 0
	l32i	a6, a3, 4
	l32i	a7, a3, 8
	s32i	a5, a2, 0
	s32i	a6, a2, 4
	s32i	a7, a2, 8
	l32i	a5, a3, 12
	l32i	a6, a3, 16
	l32i	a7, a3, 20
	s32i	a5, a2, 12
	s32i	a6, a2, 16
	s32i	a7, a2, 20
	l32i	a5, a3, 24
	l32i	a6, a3, 28
	s32i	a5, a2, 24
	s32i	a6, a2, 28
	addi	a2, a2, 32
	addi	a3, a3, 32
	__endl	a2, a4

	retw

/*
 * void __flush_invalidate_cache_all(void)
 */

ENTRY(__flush_invalidate_cache_all)
	entry	sp, 16

	movi	a2, 0
	__loopi	a2, a3, __DCACHE_WAYSIZE, XCHAL_DCACHE_LINESIZE * 4
	diwb	a2, 0 * XCHAL_DCACHE_LINESIZE
	diwb	a2, 1 * XCHAL_DCACHE_LINESIZE
	diwb	a2, 2 * XCHAL_DCACHE_LINESIZE
	diwb	a2, 3 * XCHAL_DCACHE_LINESIZE
	__endla	a2, a3, XCHAL_DCACHE_LINESIZE * 4

	movi	a2, 0
	__loopi	a2, a3, __ICACHE_WAYSIZE, XCHAL_ICACHE_LINESIZE * 4
	iii	a2, 0 * XCHAL_ICACHE_LINESIZE
	iii	a2, 1 * XCHAL_ICACHE_LINESIZE
	iii	a2, 2 * XCHAL_ICACHE_LINESIZE
	iii	a2, 3 * XCHAL_ICACHE_LINESIZE
	__endla	a2, a3, XCHAL_ICACHE_LINESIZE * 4

	isync
	retw

/*
 * void __invalidate_icache_all(void)
 */

ENTRY(__invalidate_icache_all)
	entry	sp, 16

	__invalidate_icache_all a2 a3
	isync

	retw

/*
 * void __flush_invalidate_dcache_all(void)
 */

ENTRY(__flush_invalidate_dcache_all)
	entry	sp, 16

	__flush_invalidate_dcache_all a2 a3	
	dsync
	
	dsync
	retw

/*
 * void __invalidate_dcache_all(void)
 */

ENTRY(__invalidate_dcache_all)
	entry	sp, 16

	__invalidate_dcache_all a2 a3
	dsync

	retw

/*
 * void __invalidate_icache_range(ulong start, ulong size)
 */

ENTRY(__invalidate_icache_range)
	entry	sp, 16

	__invalidate_icache_range a2 a3 a4
	isync

	retw

/*
 * void __flush_invalidate_dcache_range(ulong start, ulong size)
 */

ENTRY(__flush_invalidate_dcache_range)
	entry	sp, 16

	__flush_invalidate_dcache_range a2 a3 a4
	dsync

	retw

/*
 * void __invalidate_dcache_range(ulong start, ulong size)
 */

ENTRY(__invalidate_dcache_range)
	entry	sp, 16

	__invalidate_dcache_range a2 a3 a4
	dsync

	retw

/*
 * void __invalidate_icache_page(ulong start)
 */

ENTRY(__invalidate_icache_page)
	entry	sp, 16

	__invalidate_icache_page a2 a3
	isync

	retw

/*
 * void __invalidate_dcache_page(ulong start)
 */

ENTRY(__invalidate_dcache_page)
	entry	sp, 16

	__invalidate_dcache_page a2 a3
	dsync

	retw

/*
 * void __flush_invalidate_dcache_page(ulong start)
 */

ENTRY(__flush_invalidate_dcache_page)
	entry	sp, 16

	__flush_invalidate_dcache_page a2 a3

	dsync
	retw

/*
 * void __flush_dcache_page(ulong start)
 */

ENTRY(__flush_dcache_page)
	entry	sp, 16

	__flush_dcache_page a2 a3

	dsync
	retw

/*
 * void __flush_invalidate_dcache_page_phys(ulong start)
 */

ENTRY(__flush_invalidate_dcache_page_phys)
	entry	sp, 16

	movi	a3, XCHAL_DCACHE_SIZE
	movi	a4, PAGE_MASK | 1
	addi	a2, a2, 1

1:	addi	a3, a3, -XCHAL_DCACHE_LINESIZE

	ldct	a6, a3
	dsync
	and	a6, a6, a4
	beq	a6, a2, 2f
	bgeui	a3, 2, 1b
	retw

2:	diwbi	a3, 0
	bgeui	a3, 2, 1b
	retw

ENTRY(check_dcache_low0)
	entry	sp, 16

	movi	a3, XCHAL_DCACHE_SIZE / 4
	movi	a4, PAGE_MASK | 1
	addi	a2, a2, 1

1:	addi	a3, a3, -XCHAL_DCACHE_LINESIZE

	ldct	a6, a3
	dsync
	and	a6, a6, a4
	beq	a6, a2, 2f
	bgeui	a3, 2, 1b
	retw

2:	j 2b

ENTRY(check_dcache_high0)
	entry	sp, 16

	movi	a5, XCHAL_DCACHE_SIZE / 4
	movi	a3, XCHAL_DCACHE_SIZE / 2
	movi	a4, PAGE_MASK | 1
	addi	a2, a2, 1

1:	addi	a3, a3, -XCHAL_DCACHE_LINESIZE
	addi	a5, a5, -XCHAL_DCACHE_LINESIZE

	ldct	a6, a3
	dsync
	and	a6, a6, a4
	beq	a6, a2, 2f
	bgeui	a5, 2, 1b
	retw

2:	j 2b

ENTRY(check_dcache_low1)
	entry	sp, 16

	movi	a5, XCHAL_DCACHE_SIZE / 4
	movi	a3, XCHAL_DCACHE_SIZE * 3 / 4
	movi	a4, PAGE_MASK | 1
	addi	a2, a2, 1

1:	addi	a3, a3, -XCHAL_DCACHE_LINESIZE
	addi	a5, a5, -XCHAL_DCACHE_LINESIZE

	ldct	a6, a3
	dsync
	and	a6, a6, a4
	beq	a6, a2, 2f
	bgeui	a5, 2, 1b
	retw

2:	j 2b

ENTRY(check_dcache_high1)
	entry	sp, 16

	movi	a5, XCHAL_DCACHE_SIZE / 4
	movi	a3, XCHAL_DCACHE_SIZE
	movi	a4, PAGE_MASK | 1
	addi	a2, a2, 1

1:	addi	a3, a3, -XCHAL_DCACHE_LINESIZE
	addi	a5, a5, -XCHAL_DCACHE_LINESIZE

	ldct	a6, a3
	dsync
	and	a6, a6, a4
	beq	a6, a2, 2f
	bgeui	a5, 2, 1b
	retw

2:	j 2b


/*
 * void __invalidate_icache_page_phys(ulong start)
 */

ENTRY(__invalidate_icache_page_phys)
	entry	sp, 16

	movi	a3, XCHAL_ICACHE_SIZE
	movi	a4, PAGE_MASK | 1
	addi	a2, a2, 1

1:	addi	a3, a3, -XCHAL_ICACHE_LINESIZE

	lict	a6, a3
	isync
	and	a6, a6, a4
	beq	a6, a2, 2f
	bgeui	a3, 2, 1b
	retw

2:	iii	a3, 0
	bgeui	a3, 2, 1b
	retw


#if 0

	movi	a3, XCHAL_DCACHE_WAYS - 1
	movi	a4, PAGE_SIZE

1:	mov	a5, a2
	add	a6, a2, a4

2:	diwbi	a5, 0
	diwbi	a5, XCHAL_DCACHE_LINESIZE
	diwbi	a5, XCHAL_DCACHE_LINESIZE * 2
	diwbi	a5, XCHAL_DCACHE_LINESIZE * 3

	addi	a5, a5, XCHAL_DCACHE_LINESIZE * 4
	blt	a5, a6, 2b

	addi	a3, a3, -1
	addi	a2, a2, XCHAL_DCACHE_SIZE / XCHAL_DCACHE_WAYS
	bgez	a3, 1b

	retw

ENTRY(__invalidate_icache_page_index)
	entry	sp, 16

	movi	a3, XCHAL_ICACHE_WAYS - 1
	movi	a4, PAGE_SIZE

1:	mov	a5, a2
	add	a6, a2, a4

2:	iii	a5, 0
	iii	a5, XCHAL_ICACHE_LINESIZE
	iii	a5, XCHAL_ICACHE_LINESIZE * 2
	iii	a5, XCHAL_ICACHE_LINESIZE * 3

	addi	a5, a5, XCHAL_ICACHE_LINESIZE * 4
	blt	a5, a6, 2b

	addi	a3, a3, -1
	addi	a2, a2, XCHAL_ICACHE_SIZE / XCHAL_ICACHE_WAYS
	bgez	a3, 2b

	retw

#endif

