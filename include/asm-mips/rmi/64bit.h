/*
 * Copyright (C) 2003 Raza Foundries
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef _ASM_RMI_64BIT_H
#define _ASM_RMI_64BIT_H

#include <linux/types.h>
#include <asm/system.h>

/* Implement 64bit read and write operations */

static inline void out64(u64 val, unsigned long addr)
{
  u32 low, high, tmp;
  unsigned long flags=0;

  high = val >> 32;
  low = val & 0xffffffff;
  local_irq_save(flags);
  __asm__ __volatile__ (
			".set push\t\t\t# out64n"
			".set noreorder\n"
			".set noat\n"
			".set mips4\n"
			"   dsll32 %0, %2, 0   \n"
			"   dsll32 $1, %1, 0   \n"
			"   dsrl32 %0, %0, 0   \n"
			"   or     $1, $1, %0  \n"
			"   sd $1, (%3)\n"
			".set pop\n"
			: "=&r" (tmp)
			: "r" (high), "r" (low), "r" (addr));
  local_irq_restore(flags);
}

static inline u64 in64(unsigned long addr)
{
  unsigned long flags;
  u32 low, high;

  local_irq_save(flags);
  __asm__ __volatile__ (
			".set push\t\t\t# in64\n"
			".set noreorder\n"
			".set noat     \n"
			".set mips4    \n"
			"  ld     %1, (%2)\n"
			"  dsra32 %0, %1, 0\n"
			"  sll    %1, %1, 0\n"
			".set pop\n"
			: "=r" (high), "=r" (low)
			: "r" (addr));
  local_irq_restore(flags);

  return (((u64)high) << 32) | low;
}

#endif 
