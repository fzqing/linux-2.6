/*************************************************************************
* Author: Cavium Networks info@caviumnetworks.com
*
* 2006 (c) Cavium Networks. This file is licensed under
* the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation. This program
* is licensed "as is" without any warranty of any kind, whether
* express or implied.
* This file may also be available under a different license from
* Cavium Networks.  Contact Cavium Networks for more details.
*************************************************************************/
/**
 * Main Octeon executive header file (This should be the second header
 * file included by an application).
*/
#ifndef __CVMX_H__
#define __CVMX_H__

/* Get NULL, integer types from standard header files... */
#include <stddef.h>
#include <linux/types.h>
#include "cvmx-abi.h"
#include "cvmx-asm.h"
#include "cvmx-packet.h"

/* To have a global variable be shared among all cores,
 * declare with the CVMX_SHARED attribute.  Ex:
 * CVMX_SHARED int myglobal;
 * This will cause the variable to be placed in a special
 * section that the loader will map as shared for all cores
 * This is for data structures use by software ONLY,
 * as it is not 1-1 VA-PA mapped.
 */
#define CVMX_SHARED      __attribute__ ((section (".cvmx_shared")))

#ifdef __cplusplus
#define EXTERN_ASM extern "C"
#else
#define EXTERN_ASM extern
#endif


#define CVMX_MAX_CORES          (16)
#define CVMX_CACHE_LINE_SIZE    (128)   /* In bytes */
#define CVMX_CACHE_LINE_MASK    (CVMX_CACHE_LINE_SIZE - 1)   /* In bytes */
#define CVMX_CORE_CLOCK_MHZ     (600)
#define CVMX_CORE_CLOCK_RATE    (CVMX_CORE_CLOCK_MHZ * 1000 * 1000)
#define CAST64(v) ((unsigned long long)(unsigned long)(v))
#define CASTPTR(type, v) ((type *)(unsigned long)(v))


/* simprintf uses simulator tricks to speed up printouts.  The format
** and args are passed to the simulator and processed natively on the host.
** Simprintf is limited to 7 arguments, and they all must use %ll (long long)
** format specifiers to be displayed correctly.
*/
EXTERN_ASM void simprintf(const char *format, ...);


/**
 * This function performs some default initialization of the Octeon executive.  It initializes
 * the cvmx_bootmem memory allocator with the list of physical memory provided by the bootloader,
 * and creates 1-1 TLB mappings for this memory.
 * This function should be called on all cores that will use either the bootmem allocator or the 1-1 TLB
 * mappings.
 * Applications which require a different configuration can replace this function with a suitable application
 * specific one.
 *
 * @return 0 on success
 *         -1 on failure
 */
int cvmx_user_app_init(void);

/* turn the variable name into a string */
#define CVMX_TMP_STR(x) CVMX_TMP_STR2(x)
#define CVMX_TMP_STR2(x) #x

/*
 * The macros cvmx_likely and cvmx_unlikely use the
 * __builtin_expect GCC operation to control branch
 * probabilities for a conditional. For example, an "if"
 * statement in the code that will almost always be
 * executed should be written as "if (cvmx_likely(...))".
 * If the "else" section of an if statement is more
 * probable, use "if (cvmx_unlikey(...))".
 */
#define cvmx_likely(x)      __builtin_expect(!!(x), 1)
#define cvmx_unlikely(x)    __builtin_expect(!!(x), 0)


/**
 * Builds a bit mask given the required size in bits.
 *
 * @param bits   Number of bits in the mask
 * @return The mask
 */
static inline uint64_t cvmx_build_mask(uint64_t bits)
{
    return ~((~0x0ull) << bits);
}


/**
 * Builds a memory address for I/O based on the Major and Sub DID.
 *
 * @param major_did 5 bit major did
 * @param sub_did   3 bit sub did
 * @return I/O base address
 */
static inline uint64_t cvmx_build_io_address(uint64_t major_did, uint64_t sub_did)
{
    return ((0x1ull << 48) | (major_did << 43) | (sub_did << 40));
}


/**
 * Perform mask and shift to place the supplied value into
 * the supplied bit rage.
 *
 * Example: cvmx_build_bits(39,24,value)
 * <pre>
 * 6       5       4       3       3       2       1
 * 3       5       7       9       1       3       5       7      0
 * +-------+-------+-------+-------+-------+-------+-------+------+
 * 000000000000000000000000___________value000000000000000000000000
 * </pre>
 *
 * @param high_bit Highest bit value can occupy (inclusive) 0-63
 * @param low_bit  Lowest bit value can occupy inclusive 0-high_bit
 * @param value    Value to use
 * @return Value masked and shifted
 */
static inline uint64_t cvmx_build_bits(uint64_t high_bit, uint64_t low_bit, uint64_t value)
{
    return ((value & cvmx_build_mask(high_bit - low_bit + 1)) << low_bit);
}


#ifndef TRUE
#define FALSE   0
#define TRUE    (!(FALSE))
#endif

typedef enum {
   CVMX_MIPS_SPACE_XKSEG = 3LL,
   CVMX_MIPS_SPACE_XKPHYS = 2LL,
   CVMX_MIPS_SPACE_XSSEG = 1LL,
   CVMX_MIPS_SPACE_XUSEG = 0LL
} cvmx_mips_space_t;

typedef enum {
   CVMX_MIPS_XKSEG_SPACE_KSEG0 = 0LL,
   CVMX_MIPS_XKSEG_SPACE_KSEG1 = 1LL,
   CVMX_MIPS_XKSEG_SPACE_SSEG = 2LL,
   CVMX_MIPS_XKSEG_SPACE_KSEG3 = 3LL
} cvmx_mips_xkseg_space_t;

/* decodes <14:13> of a kseg3 window address */
typedef enum {
   CVMX_ADD_WIN_SCR = 0L,
   CVMX_ADD_WIN_DMA = 1L,   /* see cvmx_add_win_dma_dec_t for further decode */
   CVMX_ADD_WIN_UNUSED = 2L,
   CVMX_ADD_WIN_UNUSED2 = 3L
} cvmx_add_win_dec_t;

/* decode within DMA space */
typedef enum {
   CVMX_ADD_WIN_DMA_ADD = 0L,     /* add store data to the write buffer entry, allocating it if necessary */
   CVMX_ADD_WIN_DMA_SENDMEM = 1L, /* send out the write buffer entry to DRAM */
                                     /* store data must be normal DRAM memory space address in this case */
   CVMX_ADD_WIN_DMA_SENDDMA = 2L, /* send out the write buffer entry as an IOBDMA command */
                                     /* see CVMX_ADD_WIN_DMA_SEND_DEC for data contents */
   CVMX_ADD_WIN_DMA_SENDIO = 3L,  /* send out the write buffer entry as an IO write */
                                     /* store data must be normal IO space address in this case */
   CVMX_ADD_WIN_DMA_SENDSINGLE = 4L, /* send out a single-tick command on the NCB bus */
                                        /* no write buffer data needed/used */
} cvmx_add_win_dma_dec_t;



/**
 *   Physical Address Decode
 *
 * Octeon-I HW never interprets this X (<39:36> reserved
 * for future expansion), software should set to 0.
 *
 *  - 0x0 XXX0 0000 0000 to      DRAM         Cached
 *  - 0x0 XXX0 0FFF FFFF
 *
 *  - 0x0 XXX0 1000 0000 to      Boot Bus     Uncached  (Converted to 0x1 00X0 1000 0000
 *  - 0x0 XXX0 1FFF FFFF         + EJTAG                           to 0x1 00X0 1FFF FFFF)
 *
 *  - 0x0 XXX0 2000 0000 to      DRAM         Cached
 *  - 0x0 XXXF FFFF FFFF
 *
 *  - 0x1 00X0 0000 0000 to      Boot Bus     Uncached
 *  - 0x1 00XF FFFF FFFF
 *
 *  - 0x1 01X0 0000 0000 to      Other NCB    Uncached
 *  - 0x1 FFXF FFFF FFFF         devices
 *
 * Decode of all Octeon addresses
 */
typedef union {

   uint64_t         u64;

   struct {
      cvmx_mips_space_t          R   : 2;
      uint64_t               offset :62;
   } sva; /* mapped or unmapped virtual address */

   struct {
      uint64_t               zeroes :33;
      uint64_t               offset :31;
   } suseg; /* mapped USEG virtual addresses (typically) */

   struct {
      uint64_t                ones  :33;
      cvmx_mips_xkseg_space_t   sp   : 2;
      uint64_t               offset :29;
   } sxkseg; /* mapped or unmapped virtual address */

   struct {
      cvmx_mips_space_t          R   : 2; /* CVMX_MIPS_SPACE_XKPHYS in this case */
      uint64_t                 cca  : 3; /* ignored by octeon */
      uint64_t                 mbz  :10;
      uint64_t                  pa  :49; /* physical address */
   } sxkphys; /* physical address accessed through xkphys unmapped virtual address */

   struct {
      uint64_t                 mbz  :15;
      uint64_t                is_io : 1; /* if set, the address is uncached and resides on MCB bus */
      uint64_t                 did  : 8; /* the hardware ignores this field when is_io==0, else device ID */
      uint64_t                unaddr: 4; /* the hardware ignores <39:36> in Octeon I */
      uint64_t               offset :36;
   } sphys; /* physical address  */

   struct {
      uint64_t               zeroes :24; /* techically, <47:40> are dont-cares */
      uint64_t                unaddr: 4; /* the hardware ignores <39:36> in Octeon I */
      uint64_t               offset :36;
   } smem; /* physical mem address  */

   struct {
      uint64_t                 mem_region  :2;
      uint64_t                 mbz  :13;
      uint64_t                is_io : 1; /* 1 in this case */
      uint64_t                 did  : 8; /* the hardware ignores this field when is_io==0, else device ID */
      uint64_t                unaddr: 4; /* the hardware ignores <39:36> in Octeon I */
      uint64_t               offset :36;
   } sio; /* physical IO address */

   struct {
      uint64_t                ones   : 49;
      cvmx_add_win_dec_t   csrdec : 2;    /* CVMX_ADD_WIN_SCR (0) in this case */
      uint64_t                addr   : 13;
   } sscr; /* scratchpad virtual address - accessed through a window at the end of kseg3 */

   /* there should only be stores to IOBDMA space, no loads */
   struct {
      uint64_t                ones   : 49;
      cvmx_add_win_dec_t   csrdec : 2;    /* CVMX_ADD_WIN_DMA (1) in this case */
      uint64_t                unused2: 3;
      cvmx_add_win_dma_dec_t type : 3;
      uint64_t                addr   : 7;
   } sdma; /* IOBDMA virtual address - accessed through a window at the end of kseg3 */

   struct {
      uint64_t                didspace : 24;
      uint64_t                unused   : 40;
   } sfilldidspace;

} cvmx_addr_t;

/* These macros for used by 32 bit applications */

#define CVMX_MIPS32_SPACE_KSEG0 1
#define CVMX_ADD_SEG32(segment, add)          (((int64_t)(segment << 31)) | (add))

/* Currently all IOs are performed using XKPHYS addressing. Linux uses the
    CvmMemCtl register to enable XKPHYS addressing to IO space from user mode.
    Future OSes may need to change the upper bits of IO addresses. The
    following define controls the upper two bits for all IO addresses generated
    by the simple executive library */
#define CVMX_IO_SEG CVMX_MIPS_SPACE_XKPHYS

/* These macros simplify the process of creating common IO addresses */
#define CVMX_ADD_SEG(segment, add)          ((((uint64_t)segment) << 62) | (add))
#define CVMX_ADD_IO_SEG(add)                CVMX_ADD_SEG(CVMX_IO_SEG, (add))
#define CVMX_ADDR_DIDSPACE(did)             (((CVMX_IO_SEG) << 22) | ((1ULL) << 8) | (did))
#define CVMX_ADDR_DID(did)                  (CVMX_ADDR_DIDSPACE(did) << 40)
#define CVMX_FULL_DID(did,subdid)           (((did) << 3) | (subdid))


/* from include/ncb_rsl_id.v */
#define CVMX_OCT_DID_MIS 0ULL   /* misc stuff */
#define CVMX_OCT_DID_GMX0 1ULL
#define CVMX_OCT_DID_GMX1 2ULL
#define CVMX_OCT_DID_PCI 3ULL
#define CVMX_OCT_DID_KEY 4ULL
#define CVMX_OCT_DID_FPA 5ULL
#define CVMX_OCT_DID_DFA 6ULL
#define CVMX_OCT_DID_ZIP 7ULL
#define CVMX_OCT_DID_RNG 8ULL
#define CVMX_OCT_DID_IPD 9ULL
#define CVMX_OCT_DID_PKT 10ULL
#define CVMX_OCT_DID_TIM 11ULL
#define CVMX_OCT_DID_TAG 12ULL
/* the rest are not on the IO bus */
#define CVMX_OCT_DID_L2C 16ULL
#define CVMX_OCT_DID_LMC 17ULL
#define CVMX_OCT_DID_SPX0 18ULL
#define CVMX_OCT_DID_SPX1 19ULL
#define CVMX_OCT_DID_PIP 20ULL
#define CVMX_OCT_DID_ASX0 22ULL
#define CVMX_OCT_DID_ASX1 23ULL
#define CVMX_OCT_DID_IOB 30ULL

#define CVMX_OCT_DID_PKT_SEND       CVMX_FULL_DID(CVMX_OCT_DID_PKT,2ULL)
#define CVMX_OCT_DID_TAG_SWTAG      CVMX_FULL_DID(CVMX_OCT_DID_TAG,0ULL)
#define CVMX_OCT_DID_TAG_TAG1       CVMX_FULL_DID(CVMX_OCT_DID_TAG,1ULL)
#define CVMX_OCT_DID_TAG_TAG3       CVMX_FULL_DID(CVMX_OCT_DID_TAG,3ULL)
#define CVMX_OCT_DID_TAG_NULL_RD    CVMX_FULL_DID(CVMX_OCT_DID_TAG,4ULL)
#define CVMX_OCT_DID_TAG_CSR        CVMX_FULL_DID(CVMX_OCT_DID_TAG,7ULL)
#define CVMX_OCT_DID_FAU_FAI        CVMX_FULL_DID(CVMX_OCT_DID_IOB,0ULL)
#define CVMX_OCT_DID_TIM_CSR        CVMX_FULL_DID(CVMX_OCT_DID_TIM,0ULL)
#define CVMX_OCT_DID_KEY_RW         CVMX_FULL_DID(CVMX_OCT_DID_KEY,0ULL)
#define CVMX_OCT_DID_PCI_6          CVMX_FULL_DID(CVMX_OCT_DID_PCI,6ULL)
#define CVMX_OCT_DID_MIS_BOO        CVMX_FULL_DID(CVMX_OCT_DID_MIS,0ULL)
#define CVMX_OCT_DID_PCI_RML        CVMX_FULL_DID(CVMX_OCT_DID_PCI,0ULL)
#define CVMX_OCT_DID_IPD_CSR        CVMX_FULL_DID(CVMX_OCT_DID_IPD,7ULL)
#define CVMX_OCT_DID_DFA_CSR        CVMX_FULL_DID(CVMX_OCT_DID_DFA,7ULL)
#define CVMX_OCT_DID_MIS_CSR        CVMX_FULL_DID(CVMX_OCT_DID_MIS,7ULL)
#define CVMX_OCT_DID_ZIP_CSR        CVMX_FULL_DID(CVMX_OCT_DID_ZIP,0ULL)

#define CVMX_HW_BASE        ((volatile uint64_t *) 0L)
/* these counts are all in words relative to zero */
#define CVMX_REG_OFFSET     (-4*1024ll)            /* local scratchpad register base */
#define CVMX_IOBDMA_OFFSET  (-3*1024ll)
#define CVMX_IOBDMA_SEP     16 /* 128 bytes */
#define CVMX_CSR_OFFSET     (-2*1024ll)
#define CVMX_CSR_SEP        1 /* 8 bytes */


/* IOBDMA addresses */
#define CVMX_IOBDMA_SENDSINGLE (CVMX_IOBDMA_OFFSET+(CVMX_ADD_WIN_DMA_SENDSINGLE*CVMX_IOBDMA_SEP))



/* CSR's */
#define CVMX_CSR_0          (CVMX_CSR_OFFSET+(0*CVMX_CSR_SEP))



/**
 * Convert a memory pointer (void*) into a hardware compatable
 * memory address (uint64_t). Octeon hardware widgets don't
 * understand logical addresses.
 *
 * @param ptr    C style memory pointer
 * @return Hardware physical address
 */
static inline uint64_t cvmx_ptr_to_phys(void *ptr)
{
#ifdef __linux__
    if (sizeof(void*) == 8)
    {
        /* We're running in 64 bit mode. Normally this means that we can use
            40 bits of address space (the hardware limit). Unfortunately there
            is one case were we need to limit this to 30 bits, sign extended
            32 bit. Although these are 64 bits wide, only 30 bits can be used */
        if ((CAST64(ptr) >> 62) == 3)
            return CAST64(ptr) & cvmx_build_mask(30);
        else
            return CAST64(ptr) & cvmx_build_mask(40);
    }
    else
    {
        return CAST64(ptr) & cvmx_build_mask(31);
    }
#else
    /* We are assumung we're running the Simple Executive standalone. In this
        mode the TLB is setup to perform 1:1 mapping and 32 bit sign extended
        addresses are never used. Since we know all this, save the masking
        cycles and do nothing */
    return CAST64(ptr);
#endif
}


/**
 * Convert a hardware physical address (uint64_t) into a
 * memory pointer (void *).
 *
 * @param physical_address
 *               Hardware physical address to memory
 * @return Pointer to memory
 */
static inline void *cvmx_phys_to_ptr(uint64_t physical_address)
{
#ifdef __linux__
    if (sizeof(void*) == 8)
    {
        /* Just set the top bit, avoiding any TLB uglyness */
        return (void*)(unsigned long)CVMX_ADD_SEG(CVMX_MIPS_SPACE_XKPHYS, physical_address);
    }
    else
    {
        return (void*)(unsigned long)physical_address;
    }
#else
    /* We are assumung we're running the Simple Executive standalone. In this
        mode the TLB is setup to perform 1:1 mapping and 32 bit sign extended
        addresses are never used. Since we know all this, save bit insert
        cycles and do nothing */
    return (void*)(unsigned long)physical_address;
#endif
}


#include "cvmx-csr.h"

/* The following #if controls the definition of the macro
    CVMX_BUILD_WRITE64. This macro is used to build a store operation to
    a full 64bit address. With a 64bit ABI, this can be done with a simple
    pointer access. 32bit ABIs require more complicated assembly */
#if defined(CVMX_ABI_N64) || defined(CVMX_ABI_EABI)

/* We have a full 64bit ABI. Writing to a 64bit address can be done with
    a simple volatile pointer */
#define CVMX_BUILD_WRITE64(TYPE, ST)                                    \
static inline void cvmx_write64_##TYPE(uint64_t addr, TYPE##_t val)     \
{                                                                       \
    *(volatile TYPE##_t *)(unsigned long)addr = val;                    \
}

#elif defined(CVMX_ABI_N32)

/* The N32 ABI passes all 64bit quantities in a single register, so it is
    possible to use the arguments directly. We have to use inline assembly
    for the actual store since a pointer would truncate the address */
#define CVMX_BUILD_WRITE64(TYPE, ST)                                    \
static inline void cvmx_write64_##TYPE(uint64_t addr, TYPE##_t val)     \
{                                                                       \
    asm volatile (ST " %[v], 0(%[c])" ::[v] "r" (val), [c] "r" (addr)); \
}

#elif defined(CVMX_ABI_O32)

/* Ok, now the ugly stuff starts. O32 splits 64bit quantities into two
    separate registers. Assembly must be used to put them back together
    before they're used. What should be a simple store becomes a
    convoluted mess of shifts and ors */
#define CVMX_BUILD_WRITE64(TYPE, ST)                                    \
static inline void cvmx_write64_##TYPE(uint64_t csr_addr, TYPE##_t val) \
{                                                                       \
    if (sizeof(TYPE##_t) == 8)                                          \
    {                                                                   \
        uint32_t csr_addrh = csr_addr>>32;                              \
        uint32_t csr_addrl = csr_addr;                                  \
        uint32_t valh = (uint64_t)val>>32;                              \
        uint32_t vall = val;                                            \
        uint32_t tmp1;                                                  \
        uint32_t tmp2;                                                  \
        uint32_t tmp3;                                                  \
                                                                        \
        asm volatile (                                                  \
            ".set mips64\n"                                             \
            "dsll   %[tmp1], %[valh], 32\n"                             \
            "dsll   %[tmp2], %[csrh], 32\n"                             \
            "dsll   %[tmp3], %[vall], 32\n"                             \
            "dsrl   %[tmp3], %[tmp3], 32\n"                             \
            "or     %[tmp1], %[tmp1], %[tmp3]\n"                        \
            "dsll   %[tmp3], %[csrl], 32\n"                             \
            "dsrl   %[tmp3], %[tmp3], 32\n"                             \
            "or     %[tmp2], %[tmp2], %[tmp3]\n"                        \
            ST "    %[tmp1], 0(%[tmp2])\n"                              \
            : [tmp1] "=&r" (tmp1), [tmp2] "=&r" (tmp2), [tmp3] "=&r" (tmp3)\
            : [valh] "r" (valh), [vall] "r" (vall),                     \
              [csrh] "r" (csr_addrh), [csrl] "r" (csr_addrl)            \
        );                                                              \
    }                                                                   \
    else                                                                \
    {                                                                   \
        uint32_t csr_addrh = csr_addr>>32;                              \
        uint32_t csr_addrl = csr_addr;                                  \
        uint32_t tmp1;                                                  \
        uint32_t tmp2;                                                  \
                                                                        \
        asm volatile (                                                  \
            ".set mips64\n"                                             \
            "dsll   %[tmp1], %[csrh], 32\n"                             \
            "dsll   %[tmp2], %[csrl], 32\n"                             \
            "dsrl   %[tmp2], %[tmp2], 32\n"                             \
            "or     %[tmp1], %[tmp1], %[tmp2]\n"                        \
            ST "    %[val], 0(%[tmp1])\n"                               \
            : [tmp1] "=&r" (tmp1), [tmp2] "=&r" (tmp2)                  \
            : [val] "r" (val), [csrh] "r" (csr_addrh),                  \
              [csrl] "r" (csr_addrl)                                    \
        );                                                              \
    }                                                                   \
}

#else

/* cvmx-abi.h didn't recognize the ABI. Force the compile to fail. */
#error: Unsupported ABI

#endif

/* The following #if controls the definition of the macro
    CVMX_BUILD_READ64. This macro is used to build a load operation from
    a full 64bit address. With a 64bit ABI, this can be done with a simple
    pointer access. 32bit ABIs require more complicated assembly */
#if defined(CVMX_ABI_N64) || defined(CVMX_ABI_EABI)

/* We have a full 64bit ABI. Writing to a 64bit address can be done with
    a simple volatile pointer */
#define CVMX_BUILD_READ64(TYPE, LT)                                     \
static inline TYPE##_t cvmx_read64_##TYPE(uint64_t addr)                \
{                                                                       \
    return *(volatile TYPE##_t *)(unsigned long)addr;                   \
}

#elif defined(CVMX_ABI_N32)

/* The N32 ABI passes all 64bit quantities in a single register, so it is
    possible to use the arguments directly. We have to use inline assembly
    for the actual store since a pointer would truncate the address */
#define CVMX_BUILD_READ64(TYPE, LT)                                     \
static inline TYPE##_t cvmx_read64_##TYPE(uint64_t addr)                \
{                                                                       \
    TYPE##_t val;                                                       \
    asm volatile (LT " %[v], 0(%[c])": [v] "=r" (val) : [c] "r" (addr));\
    return val;                                                         \
}

#elif defined(CVMX_ABI_O32)

/* Ok, now the ugly stuff starts. O32 splits 64bit quantities into two
    separate registers. Assembly must be used to put them back together
    before they're used. What should be a simple load becomes a
    convoluted mess of shifts and ors */
#define CVMX_BUILD_READ64(TYPE, LT)                                     \
static inline TYPE##_t cvmx_read64_##TYPE(uint64_t csr_addr)            \
{                                                                       \
    if (sizeof(TYPE##_t) == 8)                                          \
    {                                                                   \
        uint32_t csr_addrh = csr_addr>>32;                              \
        uint32_t csr_addrl = csr_addr;                                  \
        uint32_t valh;                                                  \
        uint32_t vall;                                                  \
                                                                        \
        asm volatile (                                                  \
            ".set mips64\n"                                             \
            "dsll   %[valh], %[csrh], 32\n"                             \
            "dsll   %[vall], %[csrl], 32\n"                             \
            "dsrl   %[vall], %[vall], 32\n"                             \
            "or     %[valh], %[valh], %[vall]\n"                        \
            LT "    %[vall], 0(%[valh])\n"                              \
            "dsrl   %[valh], %[vall], 32\n"                             \
            "dsll   %[vall], %[vall], 32\n"                             \
            "dsrl   %[vall], %[vall], 32\n"                             \
            : [valh] "=&r" (valh), [vall] "=&r" (vall)                  \
            : [csrh] "r" (csr_addrh), [csrl] "r" (csr_addrl)            \
        );                                                              \
        return ((uint64_t)valh<<32) | vall;                             \
    }                                                                   \
    else                                                                \
    {                                                                   \
        uint32_t csr_addrh = csr_addr>>32;                              \
        uint32_t csr_addrl = csr_addr;                                  \
        TYPE##_t val;                                                   \
        uint32_t tmp;                                                   \
                                                                        \
        asm volatile (                                                  \
            ".set mips64\n"                                             \
            "dsll   %[val], %[csrh], 32\n"                              \
            "dsll   %[tmp], %[csrl], 32\n"                              \
            "dsrl   %[tmp], %[tmp], 32\n"                               \
            "or     %[val], %[val], %[tmp]\n"                           \
            LT "    %[val], 0(%[val])\n"                                \
            : [val] "=&r" (val), [tmp] "=&r" (tmp)                      \
            : [csrh] "r" (csr_addrh), [csrl] "r" (csr_addrl)            \
        );                                                              \
        return val;                                                     \
    }                                                                   \
}
#else

/* cvmx-abi.h didn't recognize the ABI. Force the compile to fail. */
#error: Unsupported ABI

#endif

/* The following defines 8 functions for writing to a 64bit address. Each
    takes two arguments, the address and the value to write.
    cvmx_write64_int64      cvmx_write64_uint64
    cvmx_write64_int32      cvmx_write64_uint32
    cvmx_write64_int16      cvmx_write64_uint16
    cvmx_write64_int8       cvmx_write64_uint8 */
CVMX_BUILD_WRITE64(int64, "sd");
CVMX_BUILD_WRITE64(int32, "sw");
CVMX_BUILD_WRITE64(int16, "sh");
CVMX_BUILD_WRITE64(int8, "sb");
CVMX_BUILD_WRITE64(uint64, "sd");
CVMX_BUILD_WRITE64(uint32, "sw");
CVMX_BUILD_WRITE64(uint16, "sh");
CVMX_BUILD_WRITE64(uint8, "sb");
#define cvmx_write64 cvmx_write64_uint64

/* The following defines 8 functions for reading from a 64bit address. Each
    takes the address as the only argument
    cvmx_read64_int64       cvmx_read64_uint64
    cvmx_read64_int32       cvmx_read64_uint32
    cvmx_read64_int16       cvmx_read64_uint16
    cvmx_read64_int8        cvmx_read64_uint8 */
CVMX_BUILD_READ64(int64, "ld");
CVMX_BUILD_READ64(int32, "lw");
CVMX_BUILD_READ64(int16, "lh");
CVMX_BUILD_READ64(int8, "lb");
CVMX_BUILD_READ64(uint64, "ld");
CVMX_BUILD_READ64(uint32, "lwu");
CVMX_BUILD_READ64(uint16, "lhu");
CVMX_BUILD_READ64(uint8, "lbu");
#define cvmx_read64 cvmx_read64_uint64

static inline void cvmx_write_csr(uint64_t csr_addr, uint64_t val)
{
#if 0
    simprintf("CSR WRITE: 0x%llx <- 0x%llx\n", csr_addr & ~(1ULL<<63), val);
#endif

    cvmx_write64(csr_addr, val);

    /* Perform an immediate read after every write to an RSL register to force
        the write to complete. It doesn't matter what RSL read we do, so we
        choose CVMX_MIO_BOOT_BIST_STAT because it is fast and harmless */
    if ((csr_addr >> 40) == (0x800118))
        cvmx_read64(CVMX_MIO_BOOT_BIST_STAT);
}

static inline void cvmx_write_io(uint64_t io_addr, uint64_t val)
{
#if 0
    simprintf("CSR WRITE: 0x%llx <- 0x%llx\n", io_addr & ~(1ULL<<63), val);
#endif
    cvmx_write64(io_addr, val);

}

static inline uint64_t cvmx_read_csr(uint64_t csr_addr)
{
    uint64_t val = cvmx_read64(csr_addr);
#if 0
    simprintf("CSR READ: 0x%llx -> 0x%llx\n", csr_addr & ~(1ULL<<63), val);
#endif
    return(val);
}


static inline void cvmx_send_single(uint64_t data)
{
    cvmx_write64((uint64_t)(CVMX_IOBDMA_SENDSINGLE * (long long)8), data);
}


static inline uint64_t cvmx_get_core_num(void)
{
    uint64_t core_num;
    CVMX_RDHWRNV(core_num, 0);
    return core_num;
}

/**
 * Returns the number of bits set in the provided value.
 * Simple wrapper for POP instruction.
 *
 * @param val    32 bit value to count set bits in
 *
 * @return Number of bits set
 */
static inline uint32_t cvmx_pop(uint32_t val)
{
    uint32_t pop;
    CVMX_POP(pop, val);
    return pop;
}
/**
 * Returns the number of bits set in the provided value.
 * Simple wrapper for DPOP instruction.
 *
 * @param val    64 bit value to count set bits in
 *
 * @return Number of bits set
 */
static inline uint64_t cvmx_dpop(uint64_t val)
{
    uint64_t pop;
    CVMX_DPOP(pop, val);
    return pop;
}

/**
 * Provide current cycle counter as a return value
 *
 * @return current cycle counter
 */

#if defined(CVMX_ABI_O32)
static inline uint64_t cvmx_get_cycle(void)
{
    uint32_t tmp_low, tmp_hi;

    asm volatile (
               "   .set mips64                  \n"
               "   .set noreorder               \n"
               "   rdhwr %[tmpl], $31           \n"
               "   dadd  %[tmph], %[tmpl], $0   \n"
               "   dsrl  %[tmph], 32            \n"
               "   dsll  %[tmpl], 32            \n"
               "   dsrl  %[tmpl], 32            \n"
               "   .set reorder                 \n"
                  : [tmpl] "=&r" (tmp_low), [tmph] "=&r" (tmp_hi) : );

    return(((uint64_t)tmp_hi << 32) + tmp_low);
}
#else
static inline uint64_t cvmx_get_cycle(void)
{
    uint64_t cycle;
    CVMX_RDHWR(cycle, 31);
    return(cycle);
}
#endif
/**
 * Wait for the specified number of cycle
 *
 * @param cycles
 */
static inline void cvmx_wait(uint64_t cycles)
{
    uint64_t done = cvmx_get_cycle() + cycles;

    while (cvmx_get_cycle() < done)
    {
        /* Spin */
    }
}

/***************************************************************************/

/* Watchdog defines, to be moved.... */
typedef enum {
   CVMX_CIU_WDOG_MODE_OFF = 0,
   CVMX_CIU_WDOG_MODE_INT = 1,
   CVMX_CIU_WDOG_MODE_INT_NMI = 2,
   CVMX_CIU_WDOG_MODE_INT_NMI_SR = 3
} cvmx_ciu_wdog_mode_t;


static inline void cvmx_reset_octeon(void)
{
    cvmx_write_csr(CVMX_CIU_SOFT_RST, 1);
}

static inline int cvmx_octeon_is_pass1(void)
{
    uint32_t id;
    asm volatile ("mfc0 %0, $15,0" : "=r" (id));
    return (id == 0x000d0000);
}

#endif  /*  __CVMX_H__  */
