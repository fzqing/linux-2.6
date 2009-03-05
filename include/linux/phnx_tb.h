#ifndef __USER_MIPS_RMI_PHNXTB_H
#define __USER_MIPS_RMI_PHNXTB_H

#include <asm/ioctl.h>

/* Trace Buffer registers */

#define    TB_REQMATCH_REGS         0x00
#define    TB_REQMATCH_REG_1        0x00
#define    TB_REQMATCH_REG_2        0x01
#define    TB_RADDR_REGS            0x02
#define    TB_RADDR_REG_1           0x02
#define    TB_RADDR_REG_2           0x03
#define    TB_CTRL_REG              0x04
#define    TB_INIT_REG              0x05
#define    TB_ACCESS_REG            0x06
#define    TB_RDDATA_REGS           0x07
#define    TB_WRDATA_REGS           0x0b
#define    TB_STATUS_REG            0x0f

// values
#define    TB_REQMATCH_RCMD_WR             0x01
#define    TB_REQMATCH_RCMD_RD             0x02
#define    TB_REQMATCH_RCMD_RDEX           0x03
#define    TB_REQMATCH_RCMD_UPGRD          0x04
#define    TB_REQMATCH_RCMD_INV            0x05
#define    TB_REQMATCH_RCMD_MASK           0x00000007
#define    TB_SET_RCMD(reg, val)    \
               reg |= (val & TB_REQMATCH_RCMD_MASK)
#define    TB_GET_RCMD(reg) \
               (reg & TB_REQMATCH_RCMD_MASK)

// Node ID
#define    TB_REQMATCH_RCONNID_SHIFT       4 
#define    TB_REQMATCH_RCONNID_MASK        0x0000001f
#define    TB_SET_RCONNID(reg, nodeId)     \
                reg |= (nodeId & TB_REQMATCH_RCONNID_MASK) << TB_REQMATCH_RCONNID_SHIFT
#define    TB_GET_RCONNID(reg) \
                ((reg >> TB_REQMATCH_RCONNID_SHIFT) & TB_REQMATCH_RCONNID_MASK)

// Transaction ID
#define    TB_REQMATCH_RTHREADID_SHIFT     12
#define    TB_REQMATCH_RTHREADID_MASK      0x0000008f
#define    TB_SET_REQMATCH_RTHREADID(reg, transId)    \
               reg |= (transId & TB_REQMATCH_RTHREADID_MASK) << TB_REQMATCH_RTHREADID_SHIFT
#define    TB_GET_REQMATCH_RTHREADID(reg) \
               ((reg >> TB_REQMATCH_RTHREADID_SHIFT) & TB_REQMATCH_RTHREADID_MASK)

// Snoop Status
#define    TB_REQMATCH_SSTAT
#define    TB_REQMATCH_SSTAT_BRIDGE            0x00100000
#define    TB_REQMATCH_SSTAT_L1TAG             0x00200000
#define    TB_REQMATCH_SSTAT_L2CTAG            0x00400000
#define    TB_REQMATCH_SSTAT_L2UCTAG           0x00800000

#define    TB_SET_REQMATCH_SSTAT(x, val)       x |= val
#define    TB_IS_SET_REQMATCH_SSTAT(x, val)    (x & val)

// Cacheable?
#define    TB_REQMATCH_RCACHE              0x01000000
#define    TB_SET_REQMATCH_CACHEABLE(x)    x |=  TB_REQMATCH_RCACHE
#define    TB_IS_REQMATCH_CACHEABLE(x)     (x & TB_REQMATCH_RCACHE)

// Node ID of Hit
#define    TB_REQMATCH_RSPHITID_MASK       0x0000001f
#define    TB_SET_REQMATCH_RSPHITID(reg, hitNodeId)    \
               reg |= (hitNodeId & TB_REQMATCH_RSPHITID_MASK)
#define    TB_GET_REQMATCH_RSPHITID(reg) (reg & TB_REQMATCH_RSPHITID_MASK)

// Snoop Result Status
#define    TB_REQMATCH_SRSLT_CPUSHR            0x00000100
#define    TB_REQMATCH_SRSLT_CPUMOD            0x00000200
#define    TB_REQMATCH_SRSLT_L2SHR             0x00000400
#define    TB_REQMATCH_SRSLT_L2MODE            0x00000800

#define    TB_SET_REQMATCH_SRSLT(x, val)       x |= val
#define    TB_IS_SET_REQMATCH_SRSLT(x,val)     (x & val)

#define    TB_CTRL_RCMD                    0x0001
#define    TB_CTRL_RCONNID                 0x0002
#define    TB_CTRL_RTHREADID               0x0004
#define    TB_CTRL_SSTAT                   0x0008
#define    TB_CTRL_RCACHE                  0x0010
#define    TB_CTRL_SRSPHITID               0x0020
#define    TB_CTRL_SRSLT                   0x0040
#define    TB_CTRL_RADDR                   0x0080
#define    TB_CTRL_COLLMODE_MATCHONLY      0x0100

#define    TB_SET_CTRL(x, flag)            x |= flag
#define    TB_IS_SET_CTRL(x, flag)         (x & flag)

#define    TB_CTRL_REQCNT_SHIFT            16
#define    TB_CTRL_REQCNT_MASK             0x000000ff
#define    TB_SET_CTRL_REQCNT(reg, cnt)    \
               reg |= ((cnt & TB_CTRL_REQCNT_MASK) << TB_CTRL_REQCNT_SHIFT)
#define    TB_GET_CTRL_REQCNT(reg) \
               ((reg >> TB_CTRL_REQCNT_SHIFT) & TB_CTRL_REQCNT_MASK)

#define    TB_CTRL_DISABLE                 0x01000000
#define    TB_SET_CTRL_DISABLE(x)          x |= TB_CTRL_DISABLE
#define    TB_IS_CTRL_DISABLED(x)          (x & TB_CTRL_DISABLE)

#define    TB_EMPTY              0x01
#define    TB_FULL               0x02
#define    TB_COLLECTS_BMATCH    0x10
#define    TB_COLLECTS_AMATCH    0x20
#define    TB_STATUS_DONE        0x40
#define    TB_DISABLED           0x80

#define    TB_IS_EMPTY(x)              (x & TB_EMPTY)
#define    TB_IS_FULL(x)               (x & TB_FULL)
#define    TB_IS_COLLECTS_BMATCH(x)    (x & TB_COLLECTS_BMATCH)
#define    TB_IS_COLLECTS_AMATCH(x)    (x & TB_COLLECTS_AMATCH)
#define    TB_IS_STATUS_DONE(x) 	   (x & TB_STATUS_DONE)
#define    TB_IS_DISABLED(x)           (x & TB_DISABLED)

#define    TB_WRPTR_SHIFT        8
#define    TB_WRPTR_MASK         0x000000ff
#define    TB_GET_WRPTR(reg)     \
               ((reg >> TB_WRPTR_SHIFT) & TB_WRPTR_MASK)

#define    TB_RDPTR_SHIFT        16
#define    TB_RDPTR_MASK         0x000000ff
#define    TB_GET_RDPTR(reg)     \
               ((reg >> TB_RDPTR_SHIFT) & TB_RDPTR_MASK)

#define    TB_FIRST_MATCH_PTR_SHIFT    24
#define    TB_FIRST_MATCH_PTR_MASK     0x000000ff
#define    TB_GET_FIRST_MATCH_PTR(reg)     \
               ((reg >> TB_FIRST_MATCH_PTR_SHIFT) & TB_FIRST_MATCH_PTR_MASK) 

typedef struct tb_register {
    int             type;
    unsigned int    val;
} tb_register_t;

#define TB_IOC_MAGIC 'T'

#define    TB_IOC_GTBREG    _IOR(TB_IOC_MAGIC, 0, struct tb_register*)
#define    TB_IOC_STBREG    _IOW(TB_IOC_MAGIC, 1, struct tb_register*)
#define    TB_IOC_REINIT    _IO(TB_IOC_MAGIC, 2)

#endif
