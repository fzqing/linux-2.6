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
 * Configuration and status register (CSR) address and type definitions for
 * Octoen.
 */
#ifndef __CVMX_CSR_H__
#define __CVMX_CSR_H__

/* Defines to represent the different versions of Octeon. Some of them may not
    exist yet... */
#define OCTEON_PASS1        1   /* First Octeon Chip - March 2005 */
#define OCTEON_PASS2        2   /* Second Pass Octeon Chip - November 2005 */

#ifndef OCTEON_REVISION
#define OCTEON_REVISION OCTEON_PASS2
#endif

#if OCTEON_REVISION == OCTEON_PASS1
#warning  OCTEON_PASS1OCTEON_PASS1OCTEON_PASS1OCTEON_PASS1OCTEON_PASS1OCTEON_PASS1OCTEON_PASS1OCTEON_PASS1OCTEON_PASS1

#include "cvmx-csr-pass1.h"

#elif OCTEON_REVISION == OCTEON_PASS2
#warning  OCTEON_PASS2OCTEON_PASS2OCTEON_PASS2OCTEON_PASS2OCTEON_PASS2OCTEON_PASS2OCTEON_PASS2OCTEON_PASS2

#include "cvmx-csr-pass2.h"

#else

#error Invalid value for OCTEON_REVISION

#endif

#endif /* __CVMX_CSR_H__ */
