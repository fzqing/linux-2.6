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
 * Packet buffer defines.
 */

#ifndef __CVMX_PACKET_H__
#define __CVMX_PACKET_H__

/**
 * This structure defines a buffer pointer on Octeon
 */
typedef union
{
    void*           ptr;
    uint64_t        u64;
    struct
    {
        uint64_t    i    : 1; /**< if set, invert the "free" pick of the overall packet. HW always sets this bit to 0 on inbound packet */
        uint64_t    back : 4; /**< Indicates the amount to back up to get to the buffer start in cache lines. In most cases
                                this is less than one complete cache line, so the value is zero */
        uint64_t    pool : 3; /**< The pool that the buffer came from / goes to */
        uint64_t    size :16; /**< The size of the segment pointed to by addr (in bytes) */
        uint64_t    addr :40; /**< Pointer to the first byte of the data, NOT buffer */
    } s;
} cvmx_buf_ptr_t;

#endif /*  __CVMX_PACKET_H__ */

