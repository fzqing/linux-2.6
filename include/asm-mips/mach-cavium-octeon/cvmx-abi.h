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
/*
* This file defines macros for use in determining the current calling ABI.
*/

#ifndef __CVMX_ABI_H__
#define __CVMX_ABI_H__

/* Check for N32 ABI, defined for 32-bit Simple Exec applications
   and Linux N32 ABI.*/
#if (defined _ABIN32 && _MIPS_SIM == _ABIN32)
#define CVMX_ABI_N32
/* Check for N64 ABI, defined for 64-bit Linux toolchain. */
#elif (defined _ABI64 && _MIPS_SIM == _ABI64)
#define CVMX_ABI_N64
/* Check for O32 ABI, defined for Linux 032 ABI, not supported yet. */
#elif (defined _ABIO32 && _MIPS_SIM == _ABIO32)
#define CVMX_ABI_O32
/* Check for EABI ABI, defined for 64-bit Simple Exec applications. */
#else
#define CVMX_ABI_EABI
#endif

#ifndef __BYTE_ORDER
    #if defined(__BIG_ENDIAN) && !defined(__LITTLE_ENDIAN)
        #define __BYTE_ORDER __BIG_ENDIAN
    #elif !defined(__BIG_ENDIAN) && defined(__LITTLE_ENDIAN)
        #define __BYTE_ORDER __LITTLE_ENDIAN
    #elif !defined(__BIG_ENDIAN) && !defined(__LITTLE_ENDIAN)
        #define __BIG_ENDIAN 4321
        #define __BYTE_ORDER __BIG_ENDIAN
    #else
        #error Unable to determine Endian mode
    #endif
#endif

#endif /* __CVMX_ABI_H__ */
