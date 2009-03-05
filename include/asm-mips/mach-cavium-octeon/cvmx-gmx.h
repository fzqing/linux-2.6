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
 * Interface to the GMX hardware.
 */
#include "cvmx-csr.h"

#ifndef __CVMX_GMX_H__
#define __CVMX_GMX_H__

/* CSR typedefs have been moved to cvmx-csr-*.h */

/**
 * Disables the sending of flow control (pause) frames on the specified
 * RGMII port(s).
 *
 * @param interface Which interface (0 or 1)
 * @param port_mask Mask (4bits) of which ports on the interface to disable
 *                  backpressure on.
 *                  1 => disable backpressure
 *                  0 => enable backpressure
 *
 * @return 0 on success
 *         -1 on error
 */

static inline int cvmx_gmx_set_backpressure_override(uint32_t interface, uint32_t port_mask)
{
    /* Check for valid arguments */
    if (port_mask & ~0xf || interface & ~0x1)
        return(-1);
    cvmx_write_csr(CVMX_GMXX_TX_OVR_BP(interface), port_mask << 8 | port_mask);
    return(0);

}

#endif

