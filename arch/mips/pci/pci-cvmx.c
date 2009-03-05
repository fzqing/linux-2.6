#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/time.h>
#include "pci-cvmx.h"

/* Octeon's PCI controller uses did=3, subdid=2 for PCI IO addresses.
    Use PCI endian swapping 1 so no address swapping is necessary. The
    Linux io routines will endian swap the data */
#define OCTEON_PCI_IOSPACE_BASE     0x80011a0400000000ull
#define OCTEON_PCI_IOSPACE_SIZE     (1ull<<32)

/* Octeon't PCI controller uses did=3, subdid=3 for PCI memory. Start PCI
    memory from 0x8000000, right after the 0-128MB BAR1 mapping */
#define OCTEON_PCI_MEMSPACE_OFFSET  (0x00011b0000000000ull)
#define OCTEON_PCI_MEMSPACE_BASE    (0x0000000008000000ull)
#define OCTEON_PCI_MEMSPACE_SIZE    (1ull<<40)

static struct resource octeon_pci_mem_resource = {
	"Octeon PCI memory",
	OCTEON_PCI_MEMSPACE_OFFSET + OCTEON_PCI_MEMSPACE_BASE,
	OCTEON_PCI_MEMSPACE_OFFSET + OCTEON_PCI_MEMSPACE_BASE +
	    OCTEON_PCI_MEMSPACE_SIZE - 1,
	IORESOURCE_MEM,
};

/* PCI ports must be above 16KB so the ISA bus filtering in the PCI-X to PCI bridge */
static struct resource octeon_pci_io_resource = {
	"Octeon PCI IO", 0x4000, OCTEON_PCI_IOSPACE_SIZE - 1, IORESOURCE_IO,
};

extern struct pci_ops octeon_pci_ops;

static struct pci_controller octeon_pci_controller = {
	.pci_ops = &octeon_pci_ops,
	.mem_resource = &octeon_pci_mem_resource,
	.mem_offset = OCTEON_PCI_MEMSPACE_OFFSET,
	.io_resource = &octeon_pci_io_resource,
	.io_offset = 0,
};


/**
 * The normal Linux udelay doesn't work for delays greater than 1ms
 *
 * @param usec
 */
static void _mdelay(uint64_t msec)
{
    uint64_t start = get_cycles();
    uint64_t delay = (1000 * msec) * mips_hpt_frequency / 1000000;

    do
    {
    } while (get_cycles() < start + delay);
}


#define LE_TO_BE_32BIT_MASK ((uint64_t) 0x04)
#define LE_TO_BE_16BIT_MASK ((uint64_t) 0x06)
#define LE_TO_BE_8BIT_MASK ((uint64_t) 0x07)

/**
 * Write a 32bit value to the Octeon NPI register space
 *
 * @param address Address to write to
 * @param val     Value to write
 */
void npi_write32(uint64_t address, uint32_t val)
{
	volatile uint32_t *ptr =
	    (volatile uint32_t *)(address ^ LE_TO_BE_32BIT_MASK);
	*ptr = val;
}

/**
 * Write a 16bit value to the Octeon NPI register space
 *
 * @param address Address to write to
 * @param val     Value to write
 */
void npi_write16(uint64_t address, uint16_t val)
{
	volatile uint16_t *ptr =
	    (volatile uint16_t *)(address ^ LE_TO_BE_16BIT_MASK);
	*ptr = val;
}

/**
 * Write a 8bit value to the Octeon NPI register space
 *
 * @param address Address to write to
 * @param val     Value to write
 */
void npi_write8(uint64_t address, uint8_t val)
{
	volatile uint8_t *ptr = (volatile uint8_t *)(address ^ LE_TO_BE_8BIT_MASK);
	*ptr = val;
}

/**
 * Read a 32bit value from the Octeon NPI register space
 *
 * @param address Address to read
 * @return The result
 */
uint32_t npi_read32(uint64_t address)
{
	volatile uint32_t *ptr =
	    (volatile uint32_t *)(address ^ LE_TO_BE_32BIT_MASK);
	uint32_t d;
	d = *ptr;
	return d;
}

/**
 * Read a 16bit value from the Octeon NPI register space
 *
 * @param address Address to read
 * @return The result
 */
uint16_t npi_read16(uint64_t address)
{
	volatile uint16_t *ptr =
	    (volatile uint16_t *)(address ^ LE_TO_BE_16BIT_MASK);
	uint16_t d;
	d = *ptr;
	return d;
}

/**
 * Read a 8bit value from the Octeon NPI register space
 *
 * @param address Address to read
 * @return The result
 */
uint8_t npi_read8(uint64_t address)
{
	volatile uint8_t *ptr =
	    (volatile uint8_t *)(address ^ LE_TO_BE_8BIT_MASK);
	uint8_t d;
	d = *ptr;
	return d;
}

/**
 * Low level initialize the Octeon PCI controller
 *
 * @return
 */
static inline void octeon_pci_initialize(void)
{
	int64_t stat;
	octeon_pci_cfg01_t cfg01;
	octeon_npi_ctl_status_t ctl_status;
	octeon_pci_ctl_status_2_t ctl_status_2;
	octeon_pci_cfg19_t cfg19;
	octeon_pci_cfg16_t cfg16;
	octeon_pci_cfg22_t cfg22;
	octeon_pci_cfg56_t cfg56;

	/* Reset the PCI Bus */
	octeon_write_csr(OCTEON_CIU_SOFT_PRST, 0x1);
	stat = octeon_read_csr(OCTEON_CIU_SOFT_PRST);

	_mdelay(2);		/* Hold  PCI reset for 2 ms */

	ctl_status.u64 = 0;
	ctl_status.s.max_word = 1;
	ctl_status.s.timer = 1;
	octeon_write_csr(OCTEON_NPI_CTL_STATUS, ctl_status.u64);

	/* Deassert PCI reset and advertize PCX Host Mode Device Capability (64b) */
	octeon_write_csr(OCTEON_CIU_SOFT_PRST, 0x4);
	stat = octeon_read_csr(OCTEON_CIU_SOFT_PRST);

	_mdelay(2);		/* Wait 2 ms after deasserting PCI reset */

	ctl_status_2.u32 = 0;
	ctl_status_2.s.bar2pres = 1;   /* bar2 present */
	ctl_status_2.s.bar2_enb = 1;   /* bar2 enable  */
	ctl_status_2.s.tsr_hwm = 1;	/* Initializes to 0.  Must be set before any PCI reads. */
	npi_write32(OCTEON_NPI_PCI_CTL_STATUS_2, ctl_status_2.u32);
	_mdelay(4);		/* Wait 2 ms before doing PCI reads */

	ctl_status_2.u32 = npi_read32(OCTEON_NPI_PCI_CTL_STATUS_2);
	printk("PCI Status: %s %s-bit\n",
	       ctl_status_2.s.ap_pcix ? "PCI-X" : "PCI",
	       ctl_status_2.s.ap_64ad ? "64" : "32");

	/*
	 ** TDOMC must be set to one in PCI mode. TDOMC should be set to 4
	 ** in PCI-X mode to allow four oustanding splits. Otherwise,
	 ** should not change from its reset value. Don't write PCI_CFG19
	 ** in PCI mode (0x82000001 reset value), write it to 0x82000004
	 ** after PCI-X mode is known. MRBCI,MDWE,MDRE -> must be zero.
	 ** MRBCM -> must be one.
	 */
	if (ctl_status_2.s.ap_pcix) {
		cfg19.u32 = 0;
		cfg19.s.tdomc = 4;	/* Target Delayed/Split request
					   outstanding maximum count. [1..31]
					   and 0=32.  NOTE: If the user
					   programs these bits beyond the
					   Designed Maximum outstanding count,
					   then the designed maximum table
					   depth will be used instead.  No
					   additional Deferred/Split
					   transactions will be accepted if
					   this outstanding maximum count is
					   reached. Furthermore, no additional
					   deferred/split transactions will be
					   accepted if the I/O delay/ I/O
					   Split Request outstanding maximum
					   is reached. */
		cfg19.s.mdrrmc = 2;	/* Master Deferred Read Request Outstanding Max
					   Count (PCI only).
					   CR4C[26:24]  Max SAC cycles   MAX DAC cycles
					   000              8                4
					   001              1                0
					   010              2                1
					   011              3                1
					   100              4                2
					   101              5                2
					   110              6                3
					   111              7                3
					   For example, if these bits are programmed to
					   100, the core can support 2 DAC cycles, 4 SAC
					   cycles or a combination of 1 DAC and 2 SAC cycles.
					   NOTE: For the PCI-X maximum outstanding split
					   transactions, refer to CRE0[22:20]  */

		cfg19.s.mrbcm = 1;	/* Master Request (Memory Read) Byte Count/Byte
					   Enable select.
					   0 = Byte Enables valid. In PCI mode, a burst
					   transaction cannot be performed using
					   Memory Read command=4?h6.
					   1 = DWORD Byte Count valid (default). In PCI
					   Mode, the memory read byte enables are
					   automatically generated by the core.
					   Note: N3 Master Request transaction sizes are
					   always determined through the
					   am_attr[<35:32>|<7:0>] field.  */
		npi_write32(OCTEON_NPI_PCI_CFG19, cfg19.u32);
	}

	cfg01.u32 = 0;
	cfg01.s.msae = 1;	/* Memory Space Access Enable */
	cfg01.s.me = 1;		/* Master Enable */
	cfg01.s.pee = 1;	/* PERR# Enable */
	cfg01.s.see = 1;	/* System Error Enable */
	cfg01.s.fbbe = 1;	/* Fast Back to Back Transaction Enable */

	npi_write32(OCTEON_NPI_PCI_CFG01, cfg01.u32);
	npi_read32(OCTEON_NPI_PCI_CFG01);

#ifdef USE_OCTEON_INTERNAL_ARBITER
	/*
	 ** When OCTEON is a PCI host, most systems will use OCTEON's
	 ** internal arbiter, so must enable it before any PCI/PCI-X
	 ** traffic can occur.
	 */
	{
		octeon_npi_pci_int_arb_cfg_t pci_int_arb_cfg;

		pci_int_arb_cfg.u64 = 0;
		pci_int_arb_cfg.s.en = 1;	/* Internal arbiter enable */
		octeon_write_csr(OCTEON_NPI_PCI_INT_ARB_CFG,
				 pci_int_arb_cfg.u64);
	}
#endif				/* USE_OCTEON_INTERNAL_ARBITER */

	/*
	 ** Preferrably written to 1 to set MLTD. [RDSATI,TRTAE,
	 ** TWTAE,TMAE,DPPMR -> must be zero. TILT -> must not be set to
	 ** 1..7.
	 */
	cfg16.u32 = 0;
	cfg16.s.mltd = 1;	/* Master Latency Timer Disable */
	npi_write32(OCTEON_NPI_PCI_CFG16, cfg16.u32);

	/*
	 ** Should be written to 0x4ff00. MTTV -> must be zero.
	 ** FLUSH -> must be 1. MRV -> should be 0xFF.
	 */
	cfg22.u32 = 0;
	cfg22.s.mrv = 0xff;	/* Master Retry Value [1..255] and 0=infinite */
	cfg22.s.flush = 1;	/* AM_DO_FLUSH_I control NOTE: This
				   bit MUST BE ONE for proper N3K
				   operation */
	npi_write32(OCTEON_NPI_PCI_CFG22, cfg22.u32);

	/*
	 ** MOST Indicates the maximum number of outstanding splits (in -1
	 ** notation) when OCTEON is in PCI-X mode.  PCI-X performance is
	 ** affected by the MOST selection.  Should generally be written
	 ** with one of 0x3be807, 0x2be807, 0x1be807, or 0x0be807,
	 ** depending on the desired MOST of 3, 2, 1, or 0, respectively.
	 */
	cfg56.u32 = 0;
	cfg56.s.pxcid = 7;	/* RO - PCI-X Capability ID */
	cfg56.s.ncp = 0xe8;	/* RO - Next Capability Pointer */
	cfg56.s.dpere = 1;	/* Data Parity Error Recovery Enable */
	cfg56.s.roe = 1;	/* Relaxed Ordering Enable */
	cfg56.s.mmbc = 1;	/* Maximum Memory Byte Count [0=512B,1=1024B,2=2048B,3=4096B] */
	cfg56.s.most = 3;	/* Maximum outstanding Split transactions [0=1 .. 7=32] */

	npi_write32(OCTEON_NPI_PCI_CFG56, cfg56.u32);

	/*
	 ** Affects PCI performance when OCTEON services reads to its
	 ** BAR1/BAR2. Refer to Section 10.6.1.  The recommended values are
	 ** 0x22, 0x33, and 0x33 for PCI_READ_CMD_6, PCI_READ_CMD_C, and
	 ** PCI_READ_CMD_E, respectively. Note that these values differ
	 ** from their reset values.
	 */
	npi_write32(OCTEON_NPI_PCI_READ_CMD_6, 0x22);
	npi_write32(OCTEON_NPI_PCI_READ_CMD_C, 0x33);
	npi_write32(OCTEON_NPI_PCI_READ_CMD_E, 0x33);
}

/**
 * Initialize the Octeon PCI controller
 *
 * @return
 */
extern octeon_bootinfo_t *octeon_bootinfo;

static int __init octeon_pci_setup(void)
{
	int index;
	octeon_pci_bar1_indexx_t bar1_index;
	octeon_npi_mem_access_subid_t mem_access;
	/* PCI I/O and PCI MEM values */
	set_io_port_base(OCTEON_PCI_IOSPACE_BASE);
	ioport_resource.start = 0;
	ioport_resource.end   = OCTEON_PCI_IOSPACE_SIZE - 1;

	if (!(octeon_bootinfo->config_flags & CVMX_BOOTINFO_CFG_FLAG_PCI_HOST))
	{
		printk("Not in host mode, PCI Controller not initialized\n");
		return 0;
	}

	octeon_pci_initialize();
	mem_access.u64 = 0;
	mem_access.s.esr = 1;
			    /**< Endian-Swap on read. */
	mem_access.s.esw = 1;
			    /**< Endian-Swap on write. */
	mem_access.s.nsr = 0;
			    /**< No-Snoop on read. */
	mem_access.s.nsw = 0;
			    /**< No-Snoop on write. */
	mem_access.s.ror = 0;
			    /**< Relax Read on read. */
	mem_access.s.row = 0;
			    /**< Relax Order on write. */
	mem_access.s.ba = OCTEON_PCI_MEMSPACE_BASE >> 36;
						       /**< PCI Address bits [63:36]. */
	octeon_write_csr(OCTEON_NPI_MEM_ACCESS_SUBID3, mem_access.u64);

    /* place Octeon BAR 0 at zero, so pci scan remaps */
	npi_write32(OCTEON_NPI_PCI_CFG04, 0);
	npi_write32(OCTEON_NPI_PCI_CFG05, 0);

	/* Remap the Octeon BAR 1 to map 0-128MB */
	bar1_index.u32 = 0;
	bar1_index.s.ca = 1;	/* 1 = Put in L2 cache */
	bar1_index.s.end_swp = 1;	/* 1 = Byte swapping */
	bar1_index.s.addr_v = 1;	/* This entry is valid */
	for (index = 0; index < 32; index++) {
		bar1_index.s.addr_idx = index;
		npi_write32(OCTEON_NPI_PCI_BAR1_INDEXX(index), bar1_index.u32);
	}
	npi_write32(OCTEON_NPI_PCI_CFG06, 0);
	npi_write32(OCTEON_NPI_PCI_CFG07, 0);

    /* place Octeon BAR 2 at zero, so pci scan remaps */
	npi_write32(OCTEON_NPI_PCI_CFG08, 0);
	npi_write32(OCTEON_NPI_PCI_CFG09, 0);

	register_pci_controller(&octeon_pci_controller);
	return 0;
}

arch_initcall(octeon_pci_setup);

int pcibios_plat_dev_init(struct pci_dev *dev)
{
	octeon_write_lcd("pci-plat");
	return 0;
}
