#include <hal.h>

#define USE_OCTEON_INTERNAL_ARBITER

/**
 * This is the bit decoding used for the Octeon PCI controller addresses
 */
typedef union {
	uint64_t u64;
	uint64_t *u64_ptr;
	uint32_t *u32_ptr;
	uint16_t *u16_ptr;
	uint8_t *u8_ptr;
	struct {
		uint64_t upper:2;
		uint64_t reserved:13;
		uint64_t io:1;
		uint64_t did:5;
		uint64_t subdid:3;
		uint64_t reserved2:4;
		uint64_t endian_swap:2;
		uint64_t reserved3:10;
		uint64_t bus:8;
		uint64_t dev:5;
		uint64_t func:3;
		uint64_t reg:8;
	} s;
} octeon_pci_address_t;

extern int octeon_i8259_setup(int);

#if defined(CONFIG_CAVIUM_OCTEON_ASUS_NA038)
#define cavium_irq_map cavium_irq_map_asus_na038
#elif defined(CONFIG_CAVIUM_OCTEON_SONICWALL_GADWIN)
#define cavium_irq_map cavium_irq_map_sonicwall_gadwin
#elif defined(CONFIG_CAVIUM_OCTEON_EBT3000) || defined(CONFIG_CAVIUM_OCTEON_EBH3100)
#define cavium_irq_map cavium_irq_map_ebh3100
#else
#error PCI IRQ routing needs to be setup for this board
#endif
