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
 * This file provides atomic operations
 */


#ifndef __CVMX_ATOMIC_H__
#define __CVMX_ATOMIC_H__

/**
 * Atomically adds a signed value to a 32 bit (aligned) memory location,
 * returns new value.
 *
 * This version does not perform 'sync' operations to enforce memory
 * operations.  This should only be used when there are no memory operation
 * ordering constraints.  (This should NOT be used for reference counting -
 * use the standard version instead.)
 *
 * @param ptr    address in memory to add incr to
 * @param incr   amount to increment memory location by (signed)
 *
 * @return Value of memory location after increment
 */
static inline int32_t cvmx_atomic_add32_nosync(int32_t *ptr, int32_t incr)
{
    uint32_t tmp, ret;

    __asm__ __volatile__(
    ".set noreorder         \n"
    "1: ll   %[tmp], %[val] \n"
    "   addu %[tmp], %[inc] \n"
    "   move %[ret], %[tmp] \n"
    "   sc   %[tmp], %[val] \n"
    "   beqz %[tmp], 1b     \n"
    "   nop                 \n"
    ".set reorder           \n"
    : [val] "+m" (*ptr), [tmp] "=&r" (tmp), [ret] "=&r" (ret)
    : [inc] "r" (incr)
    : "memory");

    return(ret);
}
/**
 * Atomically adds a signed value to a 32 bit (aligned) memory location,
 * returns new value.
 *
 * Memory access ordering is enforced before/after the atomic operation,
 * so no additional 'sync' instructions are required.
 *
 *
 * @param ptr    address in memory to add incr to
 * @param incr   amount to increment memory location by (signed)
 *
 * @return Value of memory location after increment
 */
static inline int32_t cvmx_atomic_add32(int32_t *ptr, int32_t incr)
{
    uint32_t ret;

    CVMX_SYNCW;
    ret = cvmx_atomic_add32_nosync(ptr, incr);
    CVMX_SYNCW;

    return ret;
}


/**
 * Atomically sets a 32 bit (aligned) memory location to a value
 *
 * @param ptr    address of memory to set
 * @param value  value to set memory location to.
 */
static inline void cvmx_atomic_set32(int32_t *ptr, int32_t value)
{
    uint32_t tmp;

    __asm__ __volatile__(
    ".set noreorder         \n"
    "1: ll   %[tmp], %[val] \n"
    "   move %[tmp], %[new_val] \n"
    "   sc   %[tmp], %[val] \n"
    "   beqz %[tmp], 1b     \n"
    "   nop                 \n"
    "   syncw               \n"
    ".set reorder           \n"
    : [val] "+m" (*ptr), [tmp] "=&r" (tmp)
    : [new_val] "r" (value)
    : "memory");
}

/**
 * Returns the current value of a 32 bit (aligned) memory
 * location.
 *
 * @param ptr    Address of memory to get
 * @return Value of the memory
 */
static inline int32_t cvmx_atomic_get32(int32_t *ptr)
{
    return *(volatile int32_t *)ptr;
}



/**
 * Atomically adds a signed value to a 64 bit (aligned) memory location,
 * returns new value.
 *
 * This version does not perform 'sync' operations to enforce memory
 * operations.  This should only be used when there are no memory operation
 * ordering constraints.  (This should NOT be used for reference counting -
 * use the standard version instead.)
 *
 * @param ptr    address in memory to add incr to
 * @param incr   amount to increment memory location by (signed)
 *
 * @return Value of memory location after increment
 */
static inline int64_t cvmx_atomic_add64_nosync(int64_t *ptr, int64_t incr)
{
    uint64_t tmp, ret;

    __asm__ __volatile__(
    ".set noreorder         \n"
    "1: lld  %[tmp], %[val] \n"
    "   daddu %[tmp], %[inc] \n"
    "   move %[ret], %[tmp] \n"
    "   scd  %[tmp], %[val] \n"
    "   beqz %[tmp], 1b     \n"
    "   nop                 \n"
    ".set reorder           \n"
    : [val] "+m" (*ptr), [tmp] "=&r" (tmp), [ret] "=&r" (ret)
    : [inc] "r" (incr)
    : "memory");

    return(ret);
}

/**
 * Atomically adds a signed value to a 64 bit (aligned) memory location,
 * returns new value.
 *
 * Memory access ordering is enforced before/after the atomic operation,
 * so no additional 'sync' instructions are required.
 *
 *
 * @param ptr    address in memory to add incr to
 * @param incr   amount to increment memory location by (signed)
 *
 * @return Value of memory location after increment
 */
static inline int64_t cvmx_atomic_add64(int64_t *ptr, int64_t incr)
{
    uint64_t ret;

    CVMX_SYNCW;
    ret = cvmx_atomic_add64_nosync(ptr, incr);
    CVMX_SYNCW;

    return ret;
}


/**
 * Atomically sets a 64 bit (aligned) memory location to a value
 *
 * @param ptr    address of memory to set
 * @param value  value to set memory location to.
 */
static inline void cvmx_atomic_set64(int64_t *ptr, int64_t value)
{
    uint64_t tmp;

    __asm__ __volatile__(
    ".set noreorder         \n"
    "1: lld  %[tmp], %[val] \n"
    "   move %[tmp], %[new_val] \n"
    "   scd  %[tmp], %[val] \n"
    "   beqz %[tmp], 1b     \n"
    "   nop                 \n"
    "   syncw               \n"
    ".set reorder           \n"
    : [val] "+m" (*ptr), [tmp] "=&r" (tmp)
    : [new_val] "r" (value)
    : "memory");
}

/**
 * Returns the current value of a 64 bit (aligned) memory
 * location.
 *
 * @param ptr    Address of memory to get
 * @return Value of the memory
 */
static inline int64_t cvmx_atomic_get64(int64_t *ptr)
{
    return *(volatile int64_t *)ptr;
}


/**
 * Atomically compares the old value with the value at ptr, and if they match,
 * stores new_val to ptr.
 * If *ptr and old don't match, function returns failure immediately.
 * If *ptr and old match, function spins until *ptr updated to new atomically, or
 *  until *ptr and old no longer match
 *
 * Does no memory synchronization.
 *
 * @return 1 on success (match and store)
 *         0 on no match
 */
static inline uint32_t cvmx_atomic_compare_and_store_nosync32(uint32_t *ptr, uint32_t old_val, uint32_t new_val)
{
    uint32_t tmp, ret;

    __asm__ __volatile__(
    ".set noreorder         \n"
    "1: ll   %[tmp], %[val] \n"
    "   li   %[ret], 0     \n"
    "   bne  %[tmp], %[old], 2f \n"
    "   move %[tmp], %[new_val] \n"
    "   sc   %[tmp], %[val] \n"
    "   beqz %[tmp], 1b     \n"
    "   li   %[ret], 1      \n"
    "2: nop               \n"
    ".set reorder           \n"
    : [val] "+m" (*ptr), [tmp] "=&r" (tmp), [ret] "=&r" (ret)
    : [old] "r" (old_val), [new_val] "r" (new_val)
    : "memory");

    return(ret);

}

/**
 * Atomically compares the old value with the value at ptr, and if they match,
 * stores new_val to ptr.
 * If *ptr and old don't match, function returns failure immediately.
 * If *ptr and old match, function spins until *ptr updated to new atomically, or
 *  until *ptr and old no longer match
 *
 * Does memory synchronization that is required to use this as a locking primitive.
 *
 * @return 1 on success (match and store)
 *         0 on no match
 */
static inline uint32_t cvmx_atomic_compare_and_store32(uint32_t *ptr, uint32_t old_val, uint32_t new_val)
{
    uint32_t ret;
    CVMX_SYNCW;
    ret = cvmx_atomic_compare_and_store_nosync32(ptr, old_val, new_val);
    CVMX_SYNCW;
    return ret;
}


/**
 * Atomically compares the old value with the value at ptr, and if they match,
 * stores new_val to ptr.
 * If *ptr and old don't match, function returns failure immediately.
 * If *ptr and old match, function spins until *ptr updated to new atomically, or
 *  until *ptr and old no longer match
 *
 * Does no memory synchronization.
 *
 * @return 1 on success (match and store)
 *         0 on no match
 */
static inline uint64_t cvmx_atomic_compare_and_store_nosync64(uint64_t *ptr, uint64_t old_val, uint64_t new_val)
{
    uint64_t tmp, ret;

    __asm__ __volatile__(
    ".set noreorder         \n"
    "1: lld  %[tmp], %[val] \n"
    "   li   %[ret], 0     \n"
    "   bne  %[tmp], %[old], 2f \n"
    "   move %[tmp], %[new_val] \n"
    "   scd  %[tmp], %[val] \n"
    "   beqz %[tmp], 1b     \n"
    "   li   %[ret], 1      \n"
    "2: nop               \n"
    ".set reorder           \n"
    : [val] "+m" (*ptr), [tmp] "=&r" (tmp), [ret] "=&r" (ret)
    : [old] "r" (old_val), [new_val] "r" (new_val)
    : "memory");

    return(ret);

}

/**
 * Atomically compares the old value with the value at ptr, and if they match,
 * stores new_val to ptr.
 * If *ptr and old don't match, function returns failure immediately.
 * If *ptr and old match, function spins until *ptr updated to new atomically, or
 *  until *ptr and old no longer match
 *
 * Does memory synchronization that is required to use this as a locking primitive.
 *
 * @return 1 on success (match and store)
 *         0 on no match
 */
static inline uint64_t cvmx_atomic_compare_and_store64(uint64_t *ptr, uint64_t old_val, uint64_t new_val)
{
    uint64_t ret;
    CVMX_SYNCW;
    ret = cvmx_atomic_compare_and_store_nosync64(ptr, old_val, new_val);
    CVMX_SYNCW;
    return ret;
}

#ifdef	__cplusplus
}
#endif

#endif /* __CVMX_ATOMIC_H__ */
