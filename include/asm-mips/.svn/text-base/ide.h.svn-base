/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * This file contains the MIPS architecture specific IDE code.
 */
#ifndef __ASM_IDE_H
#define __ASM_IDE_H

#include <ide.h>

#ifdef __KERNEL__
#ifndef CONFIG_CPU_LITTLE_ENDIAN
#ifndef CONFIG_NEC_CMBVR4133
/*
 * Only for the Big Endian systems, do not do the swapping.
 * We cannot turn off the CONFIG_SWAP_IO_SPACE since the
 * other subsystems need it. Hence we need this approach for
 * IDE only - Manish Lachwani (mlachwani@mvista.com)
 */
extern const unsigned long mips_io_port_base;

#ifdef insb
#undef insb
#endif
#ifdef outsb
#undef outsb
#endif
#ifdef insw
#undef insw
#endif
#ifdef outsw
#undef outsw
#endif
#ifdef insl
#undef insl
#endif
#ifdef outsl
#undef outsl
#endif

#define insb(port, addr, count)		___ide_insb(port, addr, count)
#define insw(port, addr, count)		___ide_insw(port, addr, count)
#define insl(port, addr, count)		___ide_insl(port, addr, count)
#define outsb(port, addr, count)	___ide_outsb(port, addr, count)
#define outsw(port, addr, count)	___ide_outsw(port, addr, count)
#define outsl(port, addr, count)	___ide_outsl(port, addr, count)

static inline void ___ide_insb(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		*(u16 *)addr = *(volatile u8 *)(mips_io_port_base + port);
		addr++;
	}
}

static inline void ___ide_outsb(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		*(volatile u8 *)(mips_io_port_base + port) = *(u8 *)addr;
		addr++;
	}
}

static inline void ___ide_insw(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		*(u16 *)addr = *(volatile u16 *)(mips_io_port_base + port);
		addr += 2;
	}
}

static inline void ___ide_outsw(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		*(volatile u16 *)(mips_io_port_base + port) = *(u16 *)addr;
		addr += 2;
	}
}

static inline void ___ide_insl(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		*(u32 *)addr = *(volatile u32 *)(mips_io_port_base + port);
		addr += 4;
	}
}

static inline void ___ide_outsl(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		*(volatile u32 *)(mips_io_port_base + port) = *(u32 *)addr;
		addr += 4;
	}
}

#else
/*
 * The NEC CMB-VR4133 needs the bytes swapped.
 */
extern const unsigned long mips_io_port_base;

#ifdef insw
#undef insw
#endif
#ifdef outsw
#undef outsw
#endif
#ifdef insl
#undef insl
#endif
#ifdef outsl
#undef outsl
#endif

#define insw(port, addr, count)		___ide_insw(port, addr, count)
#define insl(port, addr, count)		___ide_insl(port, addr, count)
#define outsw(port, addr, count)	___ide_outsw(port, addr, count)
#define outsl(port, addr, count)	___ide_outsl(port, addr, count)

static inline void ___ide_insw(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		*(u16 *)addr = swab16(__raw_inw(port));
		addr += 2;
	}
}

static inline void ___ide_outsw(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		__raw_outw(swab16(*(u16 *)addr), port);
		addr += 2;
		}
}

static inline void ___ide_insl(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		*(u32 *)addr = swab32(__raw_inl(port));
		addr += 4;
	}
}

static inline void ___ide_outsl(unsigned long port, void *addr, unsigned int count)
{
	while (count--) {
		__raw_outl(swab32(*(u32 *)addr), port);
		addr += 4;
	}
}
#endif /* CONFIG_NEC_CMBVR4133 */
#endif /* CONFIG_LITTLE_ENDIAN */
#endif /* __KERNEL__ */

#endif /* __ASM_IDE_H */
