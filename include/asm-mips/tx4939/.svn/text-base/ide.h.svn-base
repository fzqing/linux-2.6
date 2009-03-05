/*
 * include/asm-mips/tx4939/ide.h
 *
 * ide supplement routines
 *
 * (C) Copyright TOSHIBA CORPORATION SEMICONDUCTOR COMPANY 2005
 *
 * Author: source@mvista.com,
 *               Hiroshi DOYU <Hiroshi_DOYU@montavista.co.jp>
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 */

#ifndef __ASM_MACH_TX4939_IDE_H
#define __ASM_MACH_TX4939_IDE_H

#include <asm/mach-generic/ide.h>

#ifdef __KERNEL__

#define ide_ack_intr(hwif) (hwif->hw.ack_intr ? hwif->hw.ack_intr(hwif) : 1)
#define IDE_ARCH_ACK_INTR

#ifndef CONFIG_CPU_LITTLE_ENDIAN
/*
 * Only for the Big Endian systems, do not do the swapping.
 * We cannot turn off the CONFIG_SWAP_IO_SPACE since the
 * other subsystems need it. Hence we need this approach for
 * IDE only.
 * Furthermore, since the big endian mode of TX4939 is more specific,
 * so add more the following to include/asm-mips/ide.h.
 */
#ifdef inw
#undef inw
#endif
#ifdef outw
#undef outw
#endif
#ifdef inl
#undef inl
#endif
#ifdef outl
#undef outl
#endif

#define inw(port)		__raw_inw(port & ~0x1)
#define inl(port)		__raw_inl(port)
#define outw(val, port)		__raw_outw(val, port & ~0x01)
#define outl(val, port)		__raw_outl(val, port)

#endif /* CONFIG_LITTLE_ENDIAN */
#endif /* __KERNEL__ */

#define IS_IDE_TX4939 (HWIF(drive)->chipset == ide_tx4939)

#endif /* __ASM_MACH_TX4939_IDE_H */

