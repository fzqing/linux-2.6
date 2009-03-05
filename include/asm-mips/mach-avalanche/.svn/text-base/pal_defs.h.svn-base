#ifndef __PAL_DEFS_H__
#define __PAL_DEFS_H__

typedef enum
{
	False = 0,
	True = 1
} Bool;

#define PAL_DIM(array)  (sizeof(array)/sizeof(array[0]))

#define PAL_MK_UINT16(high8,low8)                               \
    ((unsigned short)( ((unsigned short)(high8) << 8) | (unsigned short)(low8) ))

#define PAL_UINT16_LOW8(a)                                      \
    ((unsigned char)((a) & 0x00FF))

#define PAL_UINT16_HIGH8(a)                                     \
    ((unsigned char)(((a) >> 8) & 0x00FF))

#define PAL_MK_UINT32(high16,low16)                             \
    ((unsigned int)( ((unsigned int)(high16) << 16) | (unsigned int)(low16) ))

#define PAL_MK_UINT32_FROM8S(high8,med_high8,med_low8,low8)     \
    PAL_MK_UINT32(PAL_MK_UINT16(high8,med_high8), PAL_MK_UINT16(med_low8, low8))

#define PAL_UINT32_LOW16(u32)                                   \
    ((unsigned short)((u32) & 0xFFFF))

#define PAL_UINT32_HIGH16(u32)                                  \
    ((unsigned short)(((u32) >> 16) & 0xFFFF))

#define PAL_UINT32_LOW8(u32)                                    \
    ((unsigned char)((u32) & 0x00FF))

#define PAL_UINT32_MED_LOW8(u32)                                \
    ((unsigned char)(((u32) >> 8) & 0xFF))

#define PAL_UINT32_MED_HIGH8(u32)                               \
    ((unsigned char)(((u32) >> 16) & 0xFF))

#define PAL_UINT32_HIGH8(u32)                                   \
    ((unsigned char)(((u32) >> 24) & 0xFF))

#define PAL_SWAP_UINT16(w)      \
    (PAL_MK_UINT16(unsigned short_LOW8(w), PAL_UINT16_HIGH8(w)))

#define PAL_SWAP_UINT32(u32)                \
    (PAL_MK_UINT32_FROM8S(                  \
        PAL_UINT32_LOW8(u32),               \
        PAL_UINT32_MED_LOW8(u32),           \
        PAL_UINT32_MED_HIGH8(u32),          \
        PAL_UINT32_HIGH8(u32)))

#ifdef PAL_NATIVE_ENDIAN_BIG
#define PAL_UINT16_LE(w)    PAL_SWAP_UINT16(w)
#define PAL_UINT16_BE(w)    (w)
#define PAL_UINT32_LE(d)    PAL_SWAP_UINT32(d)
#define PAL_UINT32_BE(d)    (d)

#else
#define PAL_UINT16_LE(w)    (w)
#define PAL_UINT16_BE(w)    PAL_SWAP_UINT16(w)
#define PAL_UINT32_LE(d)    (d)
#define PAL_UINT32_BE(d)    PAL_SWAP_UINT32(d)

#endif /* Endian switch */

#define PAL_INFO                (0)
#define PAL_WARNING             (1)
#define PAL_MINOR_ERROR         (2)
#define PAL_MAJOR_ERROR         (3)
#define PAL_CRITICAL_ERROR      (4)
#define PAL_ERROR_SRC_CSL       (0)
#define PAL_ERROR_SRC_DRV       (1)
#define PAL_ERROR_SRC_PAL       (2)
#define PAL_ERROR_SRC_SRV       (3)
#define PAL_ERROR_FLAG          (0x80000000)    /**< PAL Error occured sentinel flag */
#define PAL_SOK                 (0x0)
#define PAL_ERROR_SEVERITY_SHIFT    (28)
#define PAL_ERROR_SEVERITY_MASK     (0x70000000)
#define PAL_ERROR_SRC_SHIFT         (24)
#define PAL_ERROR_SRC_MASK          (0x0F000000)
#define PAL_ERROR_QUAL_SHIFT        16
#define PAL_ERROR_QUAL_MASK         (0x00FF0000)
#define PAL_ERROR_NUM_SHIFT         (0)
#define PAL_ERROR_NUM_MASK          (0x0000FFFF)

#define PAL_ERROR(severity, src, qual, num) \
    ( PAL_ERROR_FLAG | \
      (PAL_ERROR_SEVERITY_MASK & (severity << PAL_ERROR_SEVERITY_SHIFT)) | \
    (PAL_ERROR_SRC_MASK & (src << PAL_ERROR_SRC_SHIFT)) | \
    (PAL_ERROR_QUAL_MASK & (qual << PAL_ERROR_QUAL_SHIFT)) | \
    (PAL_ERROR_NUM_MASK & (num << PAL_ERROR_NUM_SHIFT)))

#define PAL_ERROR_NUM(code)         ((code & PAL_ERROR_NUM_MASK) >> PAL_ERROR_NUM_SHIFT)
#define PAL_ERROR_QUAL(code)        ((code & PAL_ERROR_QUAL_MASK) >> PAL_ERROR_QUAL_SHIFT)
#define PAL_ERROR_SRC(code)         ((code & PAL_ERROR_SRC_MASK) >> PAL_ERROR_SRC_SHIFT)
#define PAL_ERROR_SEVERITY(code)    ((code & PAL_ERROR_SEVERITY_MASK) >> PAL_ERROR_SEVERITY_SHIFT)

#define PAL_ERROR_CSLSTATUS(cslerr) \
    PAL_ERROR(PAL_MAJOR_ERROR, PAL_ERROR_SRC_CSL, 0, (cslerr & 0x7F))

#define CSL_IOCTL_SHIFT     (0)
#define CSL_IOCTL_MASK      (0xFF)
#define CSL_IOCTL(ioctl) ((ioctl & CSL_IOCTL_MASK) >> CSL_IOCTL_SHIFT)

typedef void *      CSL_ResHandle;  /**< arbitrary resource handle */
typedef short       CSL_ModuleId;   /**< numeric identifier of CSL module */
typedef unsigned short      CSL_BitMask16;  /**< 16bit binary mask */
typedef unsigned int      CSL_BitMask32;  /**< 32bit binary mask */
typedef volatile unsigned short CSL_Reg16;      /* 16bit register */
typedef volatile unsigned int CSL_Reg32;      /* 32bit register */
typedef short       CSL_Status;     /* CSL API execution status or result */
typedef short       CSL_InstNum;    /* numeric instance number of the device */
typedef short       CSL_ChaNum;     /* numeric channel number, local to a device instance */

typedef enum 
{
  CSL_FAIL = 0,
  CSL_PASS = 1
} CSL_Test;

typedef short  CSL_Uid; /* CSL unique identifier for peripheral resources */
typedef unsigned int CSL_Xio; /* SoC pin multiplex mask */

typedef struct 
{
    CSL_Uid uid;
    CSL_Xio xio;
} CSL_ResAttrs;

#define CSL_FMK(PER_REG_FIELD, val) \
    (((val) << CSL_##PER_REG_FIELD##_SHIFT) & CSL_##PER_REG_FIELD##_MASK)

#define CSL_FMKT(PER_REG_FIELD, TOKEN) \
    CSL_FMK(PER_REG_FIELD, CSL_##PER_REG_FIELD##_##TOKEN)

#define CSL_FMKR(msb, lsb, val) \
    (((val) & ((1 << ((msb) - (lsb) + 1)) - 1)) << (lsb))

#define CSL_FEXT(reg, PER_REG_FIELD) \
    (((reg) & CSL_##PER_REG_FIELD##_MASK) >> CSL_##PER_REG_FIELD##_SHIFT)

#define CSL_FEXTR(reg, msb, lsb) \
    (((reg) >> (lsb)) & ((1 << ((msb) - (lsb) + 1)) - 1))

#define CSL_FINS(reg, PER_REG_FIELD, val) \
    ((reg) = ((reg) & ~CSL_##PER_REG_FIELD##_MASK)\
    | CSL_FMK(PER_REG_FIELD, val))

#define CSL_FINST(reg, PER_REG_FIELD, TOKEN)\
    CSL_FINS((reg), PER_REG_FIELD, CSL_##PER_REG_FIELD##_##TOKEN)

#define CSL_FINSR(reg, msb, lsb, val)\
    ((reg) = ((reg) &~ (((1 << ((msb) - (lsb) + 1)) - 1) << (lsb)))\
    | CSL_FMKR(msb, lsb, val))


#define CSL_SOK                 (1)     /* Success */
#define CSL_ESYS_FAIL           (-1)    /* Generic failure */
#define CSL_ESYS_INUSE          (-2)    /* Peripheral resource is already in use */
#define CSL_ESYS_XIO            (-3)    /* Encountered a shared I/O(XIO) pin conflict */
#define CSL_ESYS_OVFL           (-4)    /* Encoutered CSL system resource overflow */
#define CSL_ESYS_BADHANDLE      (-5)    /* Handle passed to CSL was invalid */
#define CSL_ESYS_INVPARAMS      (-6)    /* invalid parameters */
#define CSL_ESYS_INVCMD         (-7)    /* invalid command */
#define CSL_ESYS_INVQUERY       (-8)    /* invalid query */
#define CSL_ESYS_NOTSUPPORTED   (-9)    /* action not supported */
#define CSL_ESYS_LAST           (-32)   /* Sentinel error code, end of sys range */

#define PAL_OS_COMMON_ERR   (0)
#define PAL_OSMEM_ERR       (1)
#define PAL_OSBUF_ERR       (2)
#define PAL_OSSEM_ERR       (3)
#define PAL_OSMUTEX_ERR     (4)
#define PAL_OSWAIT_ERR      (5)
#define PAL_OSLIST_ERR      (6)
#define PAL_OSPROTECT_ERR   (7)
#define PAL_OS_COMMON_ERROR_CREATE(x)   (PAL_ERROR(PAL_CRITICAL_ERROR, PAL_OS_COMMON_ERR, 0, (x)))
#define PAL_OS_ERROR_INVALID_PARAM      (PAL_OS_COMMON_ERROR_CREATE(1))
#define PAL_OS_ERROR_NOT_SUPPORTED      (PAL_OS_COMMON_ERROR_CREATE(2))
#define PAL_OS_ERROR_NO_RESOURCES       (PAL_OS_COMMON_ERROR_CREATE(3))
#define PAL_OS_ERROR_OS_SPECIFIC        (PAL_OS_COMMON_ERROR_CREATE(4))
#define PAL_OSMEM_DEFAULT_SEGID         0

#endif /* _CSL_DEFS_H_ */
