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
 * This file contains defines for the SPI interface
 */
#ifndef __CVMX_SPI_H__
#define __CVMX_SPI_H__

/* CSR typedefs have been moved to cvmx-csr-*.h */

/**
 * Initialize the SPI4000 for use
 *
 * @param interface SPI interface the SPI4000 is connected to
 */
int cvmx_spi4000_initialize(int interface);

#endif  /* __CVMX_SPI_H__ */
