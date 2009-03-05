/******************************************************************************
 *
 * Module Name: tbxfroot - Find the root ACPI table (RSDT)
 *              $Revision: 1.1.4.19 $
 *
 *****************************************************************************/

/*
 *  Copyright (C) 2000, 2001 R. Byron Moore

 *  This code purloined and modified by George Anzinger
 *                          Copyright (C) 2002 by MontaVista Software.
 *  It is part of the high-res-timers ACPI option and its sole purpose is
 *  to find the darn timer.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* This is most annoying!  We want to find the address of the pm timer in the
 * ACPI hardware package.  We know there is one if ACPI is available at all 
 * as it is part of the basic ACPI hardware set. 
 * However, the powers that be have conspired to make it a real
 * pain to find the address.  We have written a minimal search routine
 * that we use only once on boot up.  We try to cover all the bases including
 * checksum, and version.  We will try to get some constants and structures
 * from the ACPI code in an attempt to follow it, but darn, what a mess.
 *
 * First problem, the include files are in the driver package....
 * and what a mess they are.  We pick up the kernel string and types first.

 * But then there is the COMPILER_DEPENDENT_UINT64 ...
 */
//#define ACPI_MACHINE_WIDTH	BITS_PER_LONG
#define COMPILER_DEPENDENT_UINT64   unsigned long long
#define COMPILER_DEPENDENT_INT64   long long
#include <linux/kernel.h>
#include <linux/string.h>
#include <acpi/platform/aclinux.h>
#include <acpi/acconfig.h>
#include <acpi/actypes.h>
#include <acpi/actbl.h>
#include <acpi/acconfig.h>
#include <linux/init.h>
#include <asm/page.h>

#define STRNCMP(d,s,n)  strncmp((d), (s), (NATIVE_INT)(n))
#define RSDP_CHECKSUM_LENGTH 20
#define NATIVE_INT INT32
#define NATIVE_CHAR char

#ifndef CONFIG_ACPI_enough /* I am tired of trying to use the acpi stuff */
                           /* this code works, lets just use it. */
/*******************************************************************************
 *
 * FUNCTION:    hrt_acpi_checksum
 *
 * PARAMETERS:  Buffer              - Buffer to checksum
 *              Length              - Size of the buffer
 *
 * RETURNS      8 bit checksum of buffer
 *
 * DESCRIPTION: Computes an 8 bit checksum of the buffer(length) and returns it.
 *
 ******************************************************************************/
static __init u8
hrt_acpi_checksum(void *buffer, u32 length)
{
	u8 *limit;
	u8 *rover;
	u8 sum = 0;

	if (buffer && length) {
		/*  Buffer and Length are valid   */

		limit = (u8 *) buffer + length;

		for (rover = buffer; rover < limit; rover++) {
			sum = (u8) (sum + *rover);
		}
	}

	return (sum);
}

/*******************************************************************************
 *
 * FUNCTION:    hrt_acpi_scan_memory_for_rsdp
 *
 * PARAMETERS:  Start_address       - Starting pointer for search
 *              Length              - Maximum length to search
 *
 * RETURN:      Pointer to the RSDP if found, otherwise NULL.
 *
 * DESCRIPTION: Search a block of memory for the RSDP signature
 *
 ******************************************************************************/
static __init u8 *
hrt_acpi_scan_memory_for_rsdp(u8 * start_address, u32 length)
{
	u32 offset;
	u8 *mem_rover;

	/* Search from given start addr for the requested length  */

	for (offset = 0, mem_rover = start_address;
	     offset < length;
	     offset += ACPI_RSDP_SCAN_STEP, mem_rover += ACPI_RSDP_SCAN_STEP) {

		/* The signature and checksum must both be correct */

		if (STRNCMP((NATIVE_CHAR *) mem_rover,
			    RSDP_SIG, sizeof (RSDP_SIG) - 1) == 0 &&
		    hrt_acpi_checksum(mem_rover, RSDP_CHECKSUM_LENGTH) == 0) {
			/* If so, we have found the RSDP */

			;
			return (mem_rover);
		}
	}

	/* Searched entire block, no RSDP was found */

	return (NULL);
}

/*******************************************************************************
 *
 * FUNCTION:    hrt_acpi_find_rsdp
 *
 * PARAMETERS: 
 *
 * RETURN:      Logical address of rsdp
 *
 * DESCRIPTION: Search lower 1_mbyte of memory for the root system descriptor
 *              pointer structure.  If it is found, return its address,
 *              else return 0.
 *
 *              NOTE: The RSDP must be either in the first 1_k of the Extended
 *              BIOS Data Area or between E0000 and FFFFF (ACPI 1.0 section
 *              5.2.2; assertion #421).
 *
 ******************************************************************************/
/* Constants used in searching for the RSDP in low memory */

#define LO_RSDP_WINDOW_BASE         0	/* Physical Address */
#define HI_RSDP_WINDOW_BASE         0xE0000	/* Physical Address */
#define LO_RSDP_WINDOW_SIZE         0x400
#define HI_RSDP_WINDOW_SIZE         0x20000
#define RSDP_DESCRIPTOR struct rsdp_descriptor
static __init RSDP_DESCRIPTOR *
hrt_find_acpi_rsdp(void)
{
	u8 *mem_rover;

	/*
	 * 1) Search EBDA (low memory) paragraphs
	 */
	mem_rover =
	    hrt_acpi_scan_memory_for_rsdp((u8 *) __va(LO_RSDP_WINDOW_BASE),
					  LO_RSDP_WINDOW_SIZE);

	if (!mem_rover) {
		/*
		 * 2) Search upper memory: 
		 *    16-byte boundaries in E0000h-F0000h
		 */
		mem_rover =
		    hrt_acpi_scan_memory_for_rsdp((u8 *)
						  __va(HI_RSDP_WINDOW_BASE),
						  HI_RSDP_WINDOW_SIZE);
	}

	if (mem_rover) {
		/* Found it, return the logical address */

		return (RSDP_DESCRIPTOR *) mem_rover;
	}
	return (RSDP_DESCRIPTOR *) 0;
}

__init u32 hrt_get_acpi_pm_ptr(void)
{
	struct fadt_descriptor_rev2 *fadt2;
	struct fadt_descriptor_rev1 *fadt1;
	struct rsdt_descriptor_rev1 *rsdt;
	struct xsdt_descriptor_rev2 *xsdt;
	RSDP_DESCRIPTOR *rsdp = hrt_find_acpi_rsdp();
	struct acpi_table_header *header;
	u32 rtn;

	if (!rsdp) {
		printk("ACPI: System description tables not found\n");
		return 0;
	}
	/*
	 * Now that we have that problem out of the way, lets set up this
	 * timer.  We need to figure the addresses based on the revision
	 * of ACPI, which is in this here table we just found.
	 * We will not check the RSDT checksum, but will the FADT.
	 */
	if (rsdp->revision == 2) {
		xsdt =
		    (struct xsdt_descriptor_rev2 *) 
			__va(rsdp->xsdt_physical_address);
		fadt2 =
		    (struct fadt_descriptor_rev2 *)
			__va(xsdt->table_offset_entry[0]);
		header = (struct acpi_table_header *) fadt2;
		rtn = (u32) fadt2->xpm_tmr_blk.address;
	} else {
		rsdt =
		    (struct rsdt_descriptor_rev1 *) 
			__va(rsdp->rsdt_physical_address);
		fadt1 =
		    (struct fadt_descriptor_rev1 *) 
			__va(rsdt->table_offset_entry[0]);
		header = (struct acpi_table_header *) fadt1;
		rtn = (u32) fadt1->pm_tmr_blk;
	}
	/*
	 * Verify the signature and the checksum, if good, return
	 * the address.
	 */
	if (STRNCMP((NATIVE_CHAR *) header->signature,
		    FADT_SIG, sizeof (FADT_SIG) - 1) == 0 &&
	    hrt_acpi_checksum((NATIVE_CHAR *) header, header->length) == 0) 
		return rtn;

	printk("ACPI: Signature or checksum failed on FADT\n");
	return 0;
}

#else
extern int acpi_get_firmware_table(acpi_string signature,
				   u32 instance,
				   u32 flags, 
				   struct acpi_table_header ** table_pointer);

extern struct fadt_descriptor_rev2 acpi_fadt;
__init u32 hrt_get_acpi_pm_ptr(void)
{
	struct fadt_descriptor_rev2 *fadt = &acpi_fadt;
	struct fadt_descriptor_rev2 local_fadt;

	if (!fadt || !fadt->header.signature[0]) {
		fadt = &local_fadt;
		fadt->header.signature[0] = '\0';
		acpi_get_firmware_table("FACP", 1, ACPI_PHYSICAL_POINTER,
					(struct acpi_table_header **) & fadt);
	}
	if (!fadt || !fadt->header.signature[0]) {
		printk("ACPI: Could not find the ACPI pm timer.");
	}

	if (fadt->header.revision == 2) {
		return (u32) fadt->xpm_tmr_blk.address;
	} else {
		return (u32) fadt->V1_pm_tmr_blk;
	}
}
#endif
