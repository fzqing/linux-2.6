/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2004, 2005 Cavium Networks
 */
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/module.h>
#include <octeon-app-init.h>

#include <hal.h>
#include <cvmx-bootmem-shared.h>

/* These are for GPL compatable modules to get memory outside of Linux's
    control */
EXPORT_SYMBOL(octeon_bootmem_alloc_range);
EXPORT_SYMBOL(octeon_bootmem_alloc);

/* This must not be static since inline functions access it */
spinlock_t octeon_led_lock;

 /* Set to non-zero, so it is not in .bss section and is not zeroed */
volatile octeon_boot_descriptor_t *octeon_boot_desc_ptr = (void *)0xEADBEEFULL;
cvmx_bootinfo_t *octeon_bootinfo;

#if CONFIG_CAVIUM_RESERVE32
uint64_t octeon_reserve32_memory = 0;
#endif

void octeon_write_lcd(const char *s)
{
    if (octeon_bootinfo->led_display_base_addr)
    {
	volatile char *lcd_address = (volatile char*)((1ull << 63) | octeon_bootinfo->led_display_base_addr);
		int i;
	for (i=0; i<8; i++)
	{
			if (*s)
				lcd_address[i] = *s++;
			else
				lcd_address[i] = ' ';
		}
	}

}

/**
  * Return true if Octeon is in PCI Host mode. This means
  * Linux can control the PCI bus.
  *
  * @return Non zero if Octeon in host mode
  */
 int octeon_is_pci_host(void)
 {
     return (octeon_bootinfo->config_flags & CVMX_BOOTINFO_CFG_FLAG_PCI_HOST);
 }


 /**
  * Get the clock rate of Octeon
  *
  * @return Clock rate in HZ
  */
 uint64_t octeon_get_clock_rate(void)
 {

 #ifdef CONFIG_CAVIUM_OCTEON_SIMULATOR
     octeon_bootinfo->eclock_hz = 6000000;
 #else
     if ((octeon_bootinfo->eclock_hz < 300000000) ||
	 (octeon_bootinfo->eclock_hz > 800000000))
     {
	 printk("Clock speed from bootloader (%dMhz) is out of range. Assuming 500Mhz\n",
		octeon_bootinfo->eclock_hz/1000000);
	 octeon_bootinfo->eclock_hz = 500000000;
     }
 #endif

     return octeon_bootinfo->eclock_hz;
 }


 /**
  * Return the board name as a constant string
  *
  * @return board name
  */
 const char *octeon_board_type_string(void)
 {
     return cvmx_board_type_to_string(octeon_bootinfo->board_type);
 }


 /**
  * Get the coremask Linux was booted on.
  *
  * @return Core mask
  */
 int octeon_get_boot_coremask(void)
 {
     return octeon_boot_desc_ptr->core_mask;
 }


 /**
  * Return the number of arguments we got from the bootloader
  *
  * @return argc
  */
 int octeon_get_boot_num_arguments(void)
 {
     return octeon_boot_desc_ptr->argc;
 }


 /**
  * Get an argument from the bootloader
  *
  * @param arg    argument to get
  * @return argument
  */
 const char *octeon_get_boot_argument(int arg)
 {
     return octeon_phys_to_ptr(octeon_boot_desc_ptr->argv[arg]);
 }


 void octeon_hal_init(void)
  {
     /* Make sure we got the boot descriptor block */
     if ((octeon_boot_desc_ptr == (void *)0xEADBEEFULL))
	 panic("Boot descriptor block wasn't passed properly\n");

     octeon_bootinfo = octeon_phys_to_ptr(octeon_boot_desc_ptr->cvmx_desc_vaddr);

     spin_lock_init(&octeon_led_lock);
     octeon_write_csr(OCTEON_LED_EN, 0);
     octeon_write_csr(OCTEON_LED_PRT, 0);
#if CONFIG_CAVIUM_RESERVE32
    cvmx_bootmem_desc_t *bootmem_desc = octeon_phys_to_ptr(octeon_bootinfo->phy_mem_desc_addr);
    octeon_reserve32_memory = octeon_phy_mem_named_block_alloc(bootmem_desc, CONFIG_CAVIUM_RESERVE32<<20, 0, 0, 2<<20, "CAVIUM_RESERVE32");
    if (octeon_reserve32_memory == 0)
	printk("Failed to allocate CAVIUM_RESERVE32 memory area\n");
#endif
}

/**
 * Called after Linux allocates all of its memory. This sets
 * up the 32bit shared region. Note that this region was
 * allocated as a named block during HAL init. This made sure
 * that nobody used the memory for something else during
 * init. Now we'll free it so userspace apps can use this
 * memory region with bootmem_alloc.
 */
void octeon_hal_setup_reserved32(void)
 {
 #if CONFIG_CAVIUM_RESERVE32
     if (octeon_reserve32_memory)
     {
	 cvmx_bootmem_desc_t *bootmem_desc = octeon_phys_to_ptr(octeon_bootinfo->phy_mem_desc_addr);
	 octeon_phy_mem_named_block_free(bootmem_desc, "CAVIUM_RESERVE32");
     }
 #endif
}

void octeon_check_cpu_bist(void)
{
	const int coreid = octeon_get_core_num();
	uint64_t mask;
	uint64_t bist_val;

	/* Check BIST results for COP0 registers */
	mask = 0x1f00000000ull;
	bist_val = __read_64bit_c0_register($27, 0);
	if (bist_val & mask)
		printk("Core%d BIST Failure: CacheErr(icache) = 0x%lx\n",
		       coreid, bist_val);

	bist_val = __read_64bit_c0_register($27, 1);
	if (bist_val & 1)
		printk
		    ("Core%d L1 Dcache parity error: CacheErr(dcache) = 0x%lx\n",
		     coreid, bist_val);

	mask = 0xfc00000000000000ull;
	bist_val = __read_64bit_c0_register($11, 7);
	if (bist_val & mask)
		printk("Core%d BIST Failure: COP0_CVM_MEM_CTL = 0x%lx\n",
		       coreid, bist_val);

	__write_64bit_c0_register($27, 1, 0);

	mask = 0x18ull;
	bist_val = octeon_read_csr(OCTEON_L2D_ERR);
	octeon_write_csr(OCTEON_L2D_ERR, mask);	/* Clear error bits */
	if (bist_val & mask)
		printk("Core%d L2 Parity error: L2D_ERR = 0x%lx\n", coreid,
		       bist_val);
}

void octeon_led_init(void)
{
	spin_lock_init(&octeon_led_lock);
	octeon_write_csr(OCTEON_LED_EN, 0);
	octeon_write_csr(OCTEON_LED_PRT, 0);
	octeon_write_csr(OCTEON_LED_DBG, 0);
	octeon_write_csr(OCTEON_LED_PRT_FMT, 0);
	octeon_write_csr(OCTEON_LED_UDD_CNTX(0), 32);
	octeon_write_csr(OCTEON_LED_UDD_CNTX(1), 32);
	octeon_write_csr(OCTEON_LED_UDD_DATX(0), 0);
	octeon_write_csr(OCTEON_LED_UDD_DATX(1), 0);
	octeon_write_csr(OCTEON_LED_EN, 1);
}

void *octeon_bootmem_alloc(uint64_t size, uint64_t alignment)
{
	return octeon_bootmem_alloc_range(size, alignment, 0, 0);
}

void *octeon_bootmem_alloc_range(uint64_t size, uint64_t alignment,
				 uint64_t min_addr, uint64_t max_addr)
{
	cvmx_bootmem_desc_t *bootmem_desc =
	    octeon_phys_to_ptr(octeon_bootinfo->phy_mem_desc_addr);

	uint64_t address;
	octeon_lock(CAST64(&(bootmem_desc->lock)));
	address =
	    octeon_phy_mem_block_alloc(bootmem_desc, size, min_addr, max_addr,
				       alignment);
	octeon_unlock(CAST64(&(bootmem_desc->lock)));

	if (address)
		return octeon_phys_to_ptr(address);
	else
		return NULL;
}

static int octeon_bootmem_free(void *pMemBase, uint64_t size)
{
	int bSuccess = 0;
	cvmx_bootmem_desc_t *bootmem_desc =
	    octeon_phys_to_ptr(octeon_bootinfo->phy_mem_desc_addr);
	uint64_t address = octeon_ptr_to_phys(pMemBase);

	octeon_lock(CAST64(&(bootmem_desc->lock)));
	bSuccess = octeon_phy_mem_block_free(bootmem_desc, address, size);
	octeon_unlock(CAST64(&(bootmem_desc->lock)));

	return bSuccess;
}

static inline void octeon_phy_mem_set_size(uint64_t addr, uint64_t size)
{
	*(uint64_t *) octeon_xkphys(addr + 8) = size;
}

static inline void octeon_phy_mem_set_next(uint64_t addr, uint64_t next)
{
	*(uint64_t *) octeon_xkphys(addr) = next;
}

static inline uint64_t octeon_phy_mem_get_size(uint64_t addr)
{
	return *(uint64_t *) octeon_xkphys(addr + 8);
}

static inline uint64_t octeon_phy_mem_get_next(uint64_t addr)
{
	return *(uint64_t *) octeon_xkphys(addr);
}

#define printf printk
#define cvmx_phys_to_ptr octeon_phys_to_ptr
#include "cvmx-bootmem-shared.c"

/* These are aliases for the above function for easy use by CVMX based
    modules. They shouldn't be called by any statically linked code */
static void *cvmx_bootmem_alloc(uint64_t size, uint64_t alignment)
{
	return octeon_bootmem_alloc(size, alignment);
}

static void *cvmx_bootmem_alloc_range(uint64_t size, uint64_t alignment,
				      uint64_t min_addr, uint64_t max_addr)
{
	return octeon_bootmem_alloc_range(size, alignment, min_addr, max_addr);
}

int cvmx_bootmem_free(void *pMemBase, uint64_t size)
{
	return octeon_bootmem_free(pMemBase, size);
}

EXPORT_SYMBOL(cvmx_bootmem_free);
EXPORT_SYMBOL(cvmx_bootmem_alloc_range);
EXPORT_SYMBOL(cvmx_bootmem_alloc);
EXPORT_SYMBOL(octeon_bootinfo);
#if CONFIG_CAVIUM_RESERVE32
EXPORT_SYMBOL(octeon_reserve32_memory);
#endif
