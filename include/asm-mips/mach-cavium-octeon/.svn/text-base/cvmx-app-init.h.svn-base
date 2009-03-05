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
 * Header file for simple executive application initialization.  This defines
 * part of the ABI between the bootloader and the application.
 */

#ifndef __CVMX_APP_INIT_H__
#define __CVMX_APP_INIT_H__

#define CVMX_BOOTINFO_MAJ_VER 1
#define CVMX_BOOTINFO_MIN_VER 0


#define CVMX_BOOTINFO_OCTOEN_SERIAL_LEN 20
/* This structure is populated by the bootloader.  For binary
** compatibility the only changes that should be made are
** adding members to the end of the structure, and the minor
** version should be incremented at that time.
** If an incompatible change is made, the major version
** must be incremented, and the minor version should be reset
** to 0.
*/
typedef struct
{
    uint32_t major_version;
    uint32_t minor_version;

    uint64_t stack_top;
    uint64_t heap_base;
    uint64_t heap_end;
    uint64_t desc_vaddr;

    uint32_t exception_base_addr;
    uint32_t stack_size;
    uint32_t flags;
    uint32_t core_mask;
    uint32_t dram_size;  /**< DRAM size in megabyes */
    uint32_t phy_mem_desc_addr;  /**< physical address of free memory descriptor block*/
    uint32_t debugger_flags_base_addr;  /**< used to pass flags from app to debugger */
    uint32_t eclock_hz;  /**< CPU clock speed, in hz */
    uint32_t dclock_hz;  /**< DRAM clock speed, in hz */
    uint32_t spi_clock_hz;  /**< SPI4 clock in hz */
    uint16_t board_type;
    uint8_t board_rev_major;
    uint8_t board_rev_minor;
    uint16_t chip_type;
    uint8_t chip_rev_major;
    uint8_t chip_rev_minor;
    char board_serial_number[CVMX_BOOTINFO_OCTOEN_SERIAL_LEN];
    uint8_t mac_addr_base[6];
    uint8_t mac_addr_count;


} cvmx_bootinfo_t;


/* Type defines for board and chip types */
enum cvmx_board_types_enum {
    CVMX_BOARD_NULL_TYPE = 0,
    CVMX_BOARD_SIM_TYPE = 1,
    CVMX_BOARD_EBT3000_TYPE = 2,
    CVMX_BOARD_MAX_TYPE,
};
enum cvmx_chip_types_enum {
    CVMX_CHIP_NULL_TYPE = 0,
    CVMX_CHIP_SIM_TYPE_DEPRECATED = 1,
    CVMX_CHIP_OCTEON_SAMPLE_TYPE = 2,
    CVMX_CHIP_MAX_TYPE,
};

#endif /* __CVMX_APP_INIT_H__ */
