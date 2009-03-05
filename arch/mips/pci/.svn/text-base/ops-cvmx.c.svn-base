#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/time.h>
#include "pci-cvmx.h"

#define OCTEON_HOST_CPU_BUS     (0)
#define OCTEON_HOST_CPU_DEVICE  (0)

extern void npi_write32(uint64_t address, uint32_t val);
extern void npi_write16(uint64_t address, uint16_t val);
extern void npi_write8(uint64_t address, uint8_t val);
extern uint32_t npi_read32(uint64_t address);
extern uint16_t npi_read16(uint64_t address);
extern uint8_t npi_read8(uint64_t address);

/**
 * Read a value from configuration space
 *
 * @param bus
 * @param devfn
 * @param reg
 * @param size
 * @param val
 * @return
 */
static int octeon_read_config(struct pci_bus *bus, unsigned int devfn, int reg,
			      int size, u32 * val)
{
	static int do_once = 0;
	octeon_pci_address_t pci_addr;

	pci_addr.u64 = 0;
	pci_addr.s.upper = 2;
	pci_addr.s.io = 1;
	pci_addr.s.did = 3;
	pci_addr.s.subdid = 1;
	pci_addr.s.endian_swap = 1;
	pci_addr.s.bus = bus->number;
	pci_addr.s.dev = devfn >> 3;
	pci_addr.s.func = devfn & 0x7;
	pci_addr.s.reg = reg;

	if ((pci_addr.s.bus == OCTEON_HOST_CPU_BUS) &&
	    (pci_addr.s.dev == OCTEON_HOST_CPU_DEVICE)) {
		u64 octeon_pci_config_addr = OCTEON_NPI_PCI_CFG00 + reg;

		switch (size) {
		case 4:
			*val = npi_read32(octeon_pci_config_addr);
			return PCIBIOS_SUCCESSFUL;
		case 2:
			*val = npi_read16(octeon_pci_config_addr);
			return PCIBIOS_SUCCESSFUL;
		case 1:
			*val = npi_read8(octeon_pci_config_addr);
			return PCIBIOS_SUCCESSFUL;
		}
	} else {
		switch (size) {
		case 4:
			*val = le32_to_cpup(pci_addr.u32_ptr);
			if (unlikely
			    (!do_once && (bus->number == 1) && (reg == 0)
			     && (*val == 0x06861106))) {
				/* VT82C686B Super South South Bridge */
				pci_addr.s.reg = 0x48;
				if (*pci_addr.u8_ptr & 0x2) {
					printk
					    ("Force enabling the Via IDE (bus=%d, dev=%d)\n",
					     bus->number, devfn >> 3);
					*pci_addr.u8_ptr ^= 2;
				}
				do_once = 1;
			}
			return PCIBIOS_SUCCESSFUL;
		case 2:
			*val = le16_to_cpup(pci_addr.u16_ptr);
			return PCIBIOS_SUCCESSFUL;
		case 1:
			*val = *pci_addr.u8_ptr;
			return PCIBIOS_SUCCESSFUL;
		}
	}

	return PCIBIOS_FUNC_NOT_SUPPORTED;
}

/**
 * Write a value to PCI configuration space
 *
 * @param bus
 * @param devfn
 * @param reg
 * @param size
 * @param val
 * @return
 */
static int octeon_write_config(struct pci_bus *bus, unsigned int devfn, int reg,
			       int size, u32 val)
{
	octeon_pci_address_t pci_addr;

	pci_addr.u64 = 0;
	pci_addr.s.upper = 2;
	pci_addr.s.io = 1;
	pci_addr.s.did = 3;
	pci_addr.s.subdid = 1;
	pci_addr.s.endian_swap = 1;
	pci_addr.s.bus = bus->number;
	pci_addr.s.dev = devfn >> 3;
	pci_addr.s.func = devfn & 0x7;
	pci_addr.s.reg = reg;

	if ((pci_addr.s.bus == OCTEON_HOST_CPU_BUS) &&
	    (pci_addr.s.dev == OCTEON_HOST_CPU_DEVICE)) {
		u64 octeon_pci_config_addr = OCTEON_NPI_PCI_CFG00 + reg;

		switch (size) {
		case 4:
			npi_write32(octeon_pci_config_addr, val);
			return PCIBIOS_SUCCESSFUL;
		case 2:
			npi_write16(octeon_pci_config_addr, val);
			return PCIBIOS_SUCCESSFUL;
		case 1:
			npi_write8(octeon_pci_config_addr, val);
			return PCIBIOS_SUCCESSFUL;
		}
		return PCIBIOS_FUNC_NOT_SUPPORTED;
	} else {
		switch (size) {
		case 4:
			*pci_addr.u32_ptr = cpu_to_le32(val);
			return PCIBIOS_SUCCESSFUL;
		case 2:
			*pci_addr.u16_ptr = cpu_to_le16(val);
			return PCIBIOS_SUCCESSFUL;
		case 1:
			*pci_addr.u8_ptr = val;
			return PCIBIOS_SUCCESSFUL;
		}
		return PCIBIOS_FUNC_NOT_SUPPORTED;
	}
}

struct pci_ops octeon_pci_ops = {
	.read = octeon_read_config,
	.write = octeon_write_config,
};
