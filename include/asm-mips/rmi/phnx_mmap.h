#ifndef _ASM_RMI_PHNX_MMAP_H
#define _ASM_RMI_PHNX_MMAP_H

#define PHNX_MMAP_VIRT_START         0x40000000ULL

#define PHNX_MMAP_PHYS_START         0xe0000000ULL

/* the below macros go together */
#define PHNX_MMAP_PHYS_SIZE          (512<<20)
#define PHNX_MMAP_PMASK_SIZE         (PHNX_MMAP_PHYS_SIZE >> 1)
#define PHNX_MMAP_PMASK_SIZE_256MB   0xffff

#endif
