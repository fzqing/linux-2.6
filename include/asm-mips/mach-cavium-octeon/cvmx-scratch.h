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
 * This file provides support for the processor local scratch memory.
 * Scratch memory is byte addressable - all addresses are byte addresses.
 */


#ifndef __CVMX_SCRATCH_H__
#define __CVMX_SCRATCH_H__

#define CVMX_SCRATCH_BASE       ((volatile uint8_t *)(CVMX_HW_BASE+CVMX_REG_OFFSET))

/**
 * Reads an 8 bit value from the processor local scratchpad memory.
 *
 * @param address byte address to read from
 *
 * @return value read
 */
static inline uint8_t cvmx_scratch_read8(uint64_t address)
{
    return(*((volatile uint8_t *)(CVMX_SCRATCH_BASE + address)));
}
/**
 * Reads a 16 bit value from the processor local scratchpad memory.
 *
 * @param address byte address to read from
 *
 * @return value read
 */
static inline uint16_t cvmx_scratch_read16(uint64_t address)
{
    return(*((volatile uint16_t *)(CVMX_SCRATCH_BASE + address)));
}
/**
 * Reads a 32 bit value from the processor local scratchpad memory.
 *
 * @param address byte address to read from
 *
 * @return value read
 */
static inline uint32_t cvmx_scratch_read32(uint64_t address)
{
    return(*((volatile uint32_t *)(CVMX_SCRATCH_BASE + address)));
}
/**
 * Reads a 64 bit value from the processor local scratchpad memory.
 *
 * @param address byte address to read from
 *
 * @return value read
 */
static inline uint64_t cvmx_scratch_read64(uint64_t address)
{
    return(*((volatile uint64_t *)(CVMX_SCRATCH_BASE + address)));
}



/**
 * Writes an 8 bit value to the processor local scratchpad memory.
 *
 * @param address byte address to write to
 * @param value   value to write
 */
static inline void cvmx_scratch_write8(uint64_t address, uint64_t value)
{
    *((volatile uint8_t *)(CVMX_SCRATCH_BASE + address)) = (uint8_t)value;
}
/**
 * Writes a 32 bit value to the processor local scratchpad memory.
 *
 * @param address byte address to write to
 * @param value   value to write
 */
static inline void cvmx_scratch_write16(uint64_t address, uint64_t value)
{
    *((volatile uint16_t *)(CVMX_SCRATCH_BASE + address)) = (uint16_t)value;
}
/**
 * Writes a 16 bit value to the processor local scratchpad memory.
 *
 * @param address byte address to write to
 * @param value   value to write
 */
static inline void cvmx_scratch_write32(uint64_t address, uint64_t value)
{
    *((volatile uint32_t *)(CVMX_SCRATCH_BASE + address)) = (uint32_t)value;
}
/**
 * Writes a 64 bit value to the processor local scratchpad memory.
 *
 * @param address byte address to write to
 * @param value   value to write
 */
static inline void cvmx_scratch_write64(uint64_t address, uint64_t value)
{
    *((volatile uint64_t *)(CVMX_SCRATCH_BASE + address)) = value;
}

#endif /* __CVMX_SCRATCH_H__ */
