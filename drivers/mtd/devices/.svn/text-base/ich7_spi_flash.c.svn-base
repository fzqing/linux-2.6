/*
 * drivers/mtd/devices/ich7_spi_flash.c
 *
 * An MTD driver for an SPI Serial Flash device attached to the SPI bus of an
 * Intel ICH7 chipset.
 *
 * Author: Andy Lowe <alowe@mvista.com>
 *
 * 2006 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>

static char drvname[] = "ich7_spi_flash";

/* ICH7 SPI controller register offsets */
#define SPIS	0x00		/* 16-bit status register */
#define SPIC	0x02		/* 16-bit control register */
#define SPIA	0x04		/* 32-bit address register */
#define SPID	0x08		/* 64-byte data register */
#define BBAR	0x50		/* 32-bit BIOS base address */
#define PREOP	0x54		/* 16-bit prefix opcode */
#define OPTYPE	0x56		/* 16-bit opcode type */
#define OPMENU	0x58		/* 8-byte opcode menu */
#define PBR0	0x60		/* 32-bit protected BIOS range 0 */
#define PBR1	0x64		/* 32-bit protected BIOS range 1 */
#define PBR2	0x68		/* 32-bit protected BIOS range 2 */

/* SPI status register bit fields */
#define SPIS_LOCK	0x8000	/* configuration lock-down */
#define SPIS_BLOCKED	0x0008	/* blocked access status */
#define SPIS_DONE	0x0004	/* cycle done status */
#define SPIS_GRANT	0x0002	/* access grant */
#define SPIS_SCIP	0x0001	/* cycle in progress */

/* SPI control register bit fields */
#define SPIC_SMI	0x8000	/* SMI enable */
#define SPIC_DATA	0x4000	/* data cycle enable */
#define SPIC_DBC_SHIFT	8	/* bit shift to data byte count */
#define SPIC_DBC_MASK	0x003F
#define SPIC_COP_SHIFT	4	/* bit shift to cycle opcode pointer */
#define SPIC_COP_MASK	0x0007
#define SPIC_POP	0x0008	/* prefix opcode pointer */
#define SPIC_ACS	0x0004	/* atomic cycle sequence */
#define SPIC_SCGO	0x0002	/* SPI cycle go */
#define SPIC_AR		0x0001	/* access request */

/* SPI opcode type register bit fields */
#define OPTYPE_READ_NO_ADDR	0	/* read cycle with no address */
#define OPTYPE_WRITE_NO_ADDR	1	/* write cycle with no address */
#define OPTYPE_READ		2	/* read cycle with address */
#define OPTYPE_WRITE		3	/* write cycle with address */

/* SST25 Status Register bit fields */
#define	SR_BUSY			0x01	/* write in progress */
#define	SR_WEL			0x02	/* write enable latch */
#define	SR_BP0			0x04	/* block protect 0 */
#define	SR_BP1			0x08	/* block protect 1 */
#define	SR_BP2			0x10	/* block protect 2 */
#define	SR_BP3			0x20	/* block protect 3 */
#define	SR_AAI			0x40	/* auto address increment */
#define	SR_BPL			0x80	/* BP bits are read-only */

/* SST 25-series commands */
enum {  sst25_read = 0,  sst25_hsread,    sst25_erase_sector,
	sst25_erase_32k, sst25_erase_64k, sst25_erase_chip,
	sst25_prog,      sst25_aai,       sst25_rdsr,
	sst25_ewsr,      sst25_wrsr,      sst25_wren,
	sst25_wrdi,      sst25_rdid,      sst25_jedec_id,
	sst25_ebsy,      sst25_dbsy,      sst25_ehld,
};

/* SST 25-series opcodes */
static const unsigned char sst25_opcode[] = {
	0x03, 0x0B, 0x20,	/* read      hsread    erase_sector */
	0x52, 0xD8, 0x60,	/* erase_32k erase_64k erase_chip   */
	0x02, 0xAD, 0x05,	/* prog      aai       rdsr         */
	0x50, 0x01, 0x06,	/* ewsr      wrsr      wren         */
	0x04, 0xAB, 0x9F,	/* wrdi      rdid      jedec_id     */
	0x70, 0x80, 0xAA	/* ebsy      dbsy      ehld         */
};

/* macro to convert an sst25_opcode index to an opcode flag */
#define OPF(x) (1 << (x))

/* offset to the RCBA register in LPC PCI configuration space */
#define LPC_RCBA_REG		0xF0
/* offset to the BIOS_CNTRL register in LPC PCI configuration space */
#define LPC_BIOS_CNTL_REG	0xDC
/* bit fields in the BIOS_CNTL register */
#define BIOS_CNTL_BIOSWE	0x0001	/* write enable */
#define BIOS_CNTL_BLE		0x0002	/* lock enable */
/* offset of the SPI registers relative to RCBA */
#define ICH7_SPI_OFFSET	0x3020
/* size of the SPI register memory region */
#define ICH7_SPI_SIZE	0x70
/* offset of the general control and status register relative to RCBA */
#define ICH7_GCS_OFFSET	0x3410	/* 32-bit */
/* size (in bytes) of the GCS register */
#define ICH7_GCS_SIZE	4
/* mask of the Boot BIOS Straps field in the GCS register */
#define GCS_BBS_MASK	0x00000C00
#define GCS_BBS_SPI	0x00000400	/* enable SPI */

#define NO_PREFIX	0
#define NO_ADDR		(~0)

struct ich7_spi_flash {
	int nr_parts;		/* number of partitions */
	void __iomem *spi;
	void __iomem *gcs;
	struct pci_dev *pci;
	struct semaphore lock;
	struct mtd_info mtd;
	u32 opcodes;		/* bit mask of supported flash commands */
	u32 old_gcs;		/* original value of gcs register */
	u8 old_bios_cntl;	/* original value of bios_cntl register */
};

static inline struct ich7_spi_flash *mtd_to_spi(struct mtd_info *mtd)
{
	return container_of(mtd, struct ich7_spi_flash, mtd);
}

/*
 * Issue an SPI write command.  If prefix != NO_PREFIX, then a two-command
 * atomic sequence { prefix, cmd } is issued.  If addr != NO_ADDR, then the
 * cmd opcode is followed by a three-byte address.  If data_bytes is non-zero,
 * then the command and optional address are followed by data_bytes of data,
 * where data_bytes must be between 0 and 64 inclusive.
 */
static void ich7_spi_cmd_write(void __iomem * spi, u8 cmd, unsigned addr,
			       u8 prefix, unsigned data_bytes, const u8 * data)
{
	u16 spic = 0;

	if (addr != NO_ADDR) {
		writel(addr, spi + SPIA);
		writew(OPTYPE_WRITE, spi + OPTYPE);
	} else {
		writew(OPTYPE_WRITE_NO_ADDR, spi + OPTYPE);
	}

	if (prefix != NO_PREFIX) {
		writew(prefix, spi + PREOP);
		spic |= SPIC_ACS;
	}

	if (data_bytes) {
		u8 n = ((data_bytes - 1) & SPIC_DBC_MASK);

		spic |= SPIC_DATA;
		spic |= (n << SPIC_DBC_SHIFT);
		memcpy_toio(spi + SPID, data, n + 1);
	}

	/* clear any latched status bits that happen to be set in SPIS */
	writew(SPIS_BLOCKED | SPIS_DONE, spi + SPIS);

	writeb(cmd, spi + OPMENU);
	spic |= SPIC_SCGO;

	writew(spic, spi + SPIC);	/* start the command */

	/* wait for command to complete */
	while (readw(spi + SPIS) & SPIS_SCIP)
		cpu_relax();
}

/*
 * Issue an SPI read command.  If addr != NO_ADDR, then the cmd opcode is
 * followed by a three-byte address.  If data_bytes is non-zero, then the
 * command and optional address are followed by data_bytes of data, where
 * data_bytes must be between 0 and 64 inclusive.
 */
static void ich7_spi_cmd_read(void __iomem * spi, u8 cmd, unsigned addr,
			      unsigned data_bytes, u8 * data)
{
	u16 spic = 0;
	u8 n = ((data_bytes - 1) & SPIC_DBC_MASK);

	if (addr != NO_ADDR) {
		writel(addr, spi + SPIA);
		writew(OPTYPE_READ, spi + OPTYPE);
	} else {
		writew(OPTYPE_READ_NO_ADDR, spi + OPTYPE);
	}

	if (data_bytes) {
		spic |= SPIC_DATA;
		spic |= (n << SPIC_DBC_SHIFT);
	}

	/* clear any latched status bits that happen to be set in SPIS */
	writew(SPIS_BLOCKED | SPIS_DONE, spi + SPIS);

	writeb(cmd, spi + OPMENU);
	spic |= SPIC_SCGO;
	writew(spic, spi + SPIC);	/* start the command */

	/* wait for command to complete */
	while (readw(spi + SPIS) & SPIS_SCIP)
		cpu_relax();

	if (data_bytes)
		memcpy_fromio(data, spi + SPID, n + 1);
}

/* read the flash status register */
static u8 ich7_spi_flash_rdsr(void __iomem * spi)
{
	u8 sr;

	ich7_spi_cmd_read(spi, sst25_opcode[sst25_rdsr], NO_ADDR, 1, &sr);

	return sr;
}

/*
 * Busy-wait for a flash programming operation to complete.
 * Returns 0 if the operation completes, or non-zero in the event of a
 * timeout error.
 */
static int ich7_spi_flash_prog_wait(void __iomem * spi)
{
	int i;
	const int max_rdsr = 40;

	for (i = 0; i < max_rdsr; i++) {
		if (!(ich7_spi_flash_rdsr(spi) & SR_BUSY))
			return 0;
	}
	return 1;
}

/*
 * Wait for a flash erase operation to complete.
 * Erasing flash can be a lengthy operation (10s or even 100s of milliseconds),
 * so we voluntarily yield the processor between checks.
 * Returns 0 if the operation completes, or non-zero in the event of a
 * timeout error.
 */
static int ich7_spi_flash_erase_wait(void __iomem * spi)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(200);

	while (time_before(jiffies, timeout)) {
		if (!(ich7_spi_flash_rdsr(spi) & SR_BUSY))
			return 0;
		cond_resched();
	}
	return ((ich7_spi_flash_rdsr(spi) & SR_BUSY) != 0);
}

/*
 * Write to the flash status register.
 * The 'enable write status register' command is issued first, followed by the
 * 'write status register' command.
 */
static void ich7_spi_flash_wrsr(void __iomem * spi, u8 sr)
{
	ich7_spi_cmd_write(spi, sst25_opcode[sst25_wrsr], NO_ADDR,
			   sst25_opcode[sst25_ewsr], 1, &sr);
}

/* Issue a 'write disable' command to exit AAI mode. */
static void ich7_spi_flash_wrdi(void __iomem * spi)
{
	ich7_spi_cmd_read(spi, sst25_opcode[sst25_wrdi], NO_ADDR, 0, NULL);
}

/* Issue a 'read id' command to the flash. */
static void ich7_spi_flash_rdid(void __iomem * spi, u8 * mfr, u8 * id)
{
	u8 data[2];

	ich7_spi_cmd_read(spi, sst25_opcode[sst25_rdid], 0, 2, data);
	*mfr = data[0];
	*id = data[1];
}

/* Issue a 'read JEDEC id' command to the flash. */
static void ich7_spi_flash_jedec_id(void __iomem * spi, u8 * mfr,
				    u16 * jedec_id)
{
	u8 data[3];

	ich7_spi_cmd_read(spi, sst25_opcode[sst25_jedec_id], NO_ADDR, 3, data);
	*mfr = data[0];
	*jedec_id = (data[1] << 8) | data[2];
}

/*
 * Issue a single 'read data' command to the flash.
 * Returns the number of bytes successfully read.
 * The ICH7 SPI controller has a limit of 64 data bytes per command, so the
 * return value will never be greater than 64.
 */
static int ich7_spi_flash_read(void __iomem * spi, unsigned addr, unsigned len,
			       void *buf)
{
	u8 this_len = (len > 64) ? 64 : len;

	if (this_len)
		ich7_spi_cmd_read(spi, sst25_opcode[sst25_read], addr, this_len,
				  buf);

	return this_len;
}

/*
 * Issue a single 'write enable' command followed by a single 'byte program'
 * command to the flash.
 * This routine does not return until the flash status register indicates that
 * the write operation has completed.
 * Returns the number of bytes successfully written.
 */
static int ich7_spi_flash_prog(void __iomem * spi, unsigned addr,
			       const u8 data[1])
{
	ich7_spi_cmd_write(spi, sst25_opcode[sst25_prog], addr,
			   sst25_opcode[sst25_wren], 1, data);

	/* Wait for the write operation to complete. */
	ich7_spi_flash_prog_wait(spi);

	return 1;
}

/*
 * Issue a single 'write enable' command followed by a single
 * 'auto address increment word program' command to the flash.
 * The AAI command accepts exactly two bytes of data.
 * This routine does not return until the flash status register indicates that
 * the write operation has completed.  The flash will be in AAI mode on exit.
 * Returns the number of bytes successfully written.
 */
static int ich7_spi_flash_start_aai(void __iomem * spi, unsigned addr,
				    const u8 data[2])
{
	ich7_spi_cmd_write(spi, sst25_opcode[sst25_aai], addr,
			   sst25_opcode[sst25_wren], 2, data);

	/* Wait for the write operation to complete. */
	ich7_spi_flash_prog_wait(spi);

	return 2;
}

/*
 * Issue a single 'auto address increment word program' command to the flash.
 * No address is included with the command, so the flash must already be in AAI
 * mode.  The AAI command accepts exactly two bytes of data.
 * This routine does not return until the flash status register indicates that
 * the write operation has completed.  The flash will be in AAI mode on exit.
 * Returns the number of bytes successfully written.
 */
static int ich7_spi_flash_cont_aai(void __iomem * spi, const u8 data[2])
{
	ich7_spi_cmd_write(spi, sst25_opcode[sst25_aai], NO_ADDR, NO_PREFIX,
			   2, data);

	/* Wait for the write operation to complete. */
	ich7_spi_flash_prog_wait(spi);

	return 2;
}

/*
 * Issue a single 'write enable' command followed by a single
 * '4-Kbyte sector erase' command to the flash.
 * This routine returns without waiting for the erase operation to complete.
 */
static void ich7_spi_flash_erase_sector(void __iomem * spi, unsigned addr)
{
	ich7_spi_cmd_write(spi, sst25_opcode[sst25_erase_sector], addr,
			   sst25_opcode[sst25_wren], 0, NULL);
}

/*
 * Issue a single 'write enable' command followed by a single
 * '32-KByte block erase' command to the flash.
 * This routine returns without waiting for the erase operation to complete.
 */
static void ich7_spi_flash_erase_32k(void __iomem * spi, unsigned addr)
{
	ich7_spi_cmd_write(spi, sst25_opcode[sst25_erase_32k], addr,
			   sst25_opcode[sst25_wren], 0, NULL);
}

/*
 * Issue a single 'write enable' command followed by a single
 * '64-KByte block erase' command to the flash.
 * This routine returns without waiting for the erase operation to complete.
 */
static void ich7_spi_flash_erase_64k(void __iomem * spi, unsigned addr)
{
	ich7_spi_cmd_write(spi, sst25_opcode[sst25_erase_64k], addr,
			   sst25_opcode[sst25_wren], 0, NULL);
}

/*
 * Issue a single 'write enable' command followed by a single
 * 'chip erase' command to the flash.
 * This routine returns without waiting for the erase operation to complete.
 */
static void ich7_spi_flash_erase_chip(void __iomem * spi)
{
	ich7_spi_cmd_write(spi, sst25_opcode[sst25_erase_chip], NO_ADDR,
			   sst25_opcode[sst25_wren], 0, NULL);
}

static void ich7_spi_shutdown(struct platform_device *p)
{
	struct ich7_spi_flash *s = dev_get_drvdata(&p->dev);

	/* restore the original value of the BIOS_CNTL register */
	pci_write_config_byte(s->pci, LPC_BIOS_CNTL_REG, s->old_bios_cntl);

	/* restore the original Boot BIOS Strap mode in GCS */
	writel(s->old_gcs, s->gcs);
}

static int ich7_spi_startup(struct platform_device *p)
{
	struct ich7_spi_flash *s = dev_get_drvdata(&p->dev);

	/* Initialize the SPI controller.
	 * If SPI mode is not currently selected in the GCS Boot BIOS Strap
	 * bits, then an SPI command will be executed as a side-effect of
	 * switching to SPI mode.  We must initialize the SPI registers such
	 * that an innocuous command will be executed.
	 */
	if (readw(s->spi + SPIS) & SPIS_LOCK) {
		dev_err(&p->dev, "SPI configuration is locked.\n");
		goto out;
	}
	/* clear any latched status bits that happen to be set in SPIS */
	writew(SPIS_BLOCKED | SPIS_DONE, s->spi + SPIS);
	writew(0, s->spi + SPIC);
	writel(0, s->spi + BBAR);
	writew(0, s->spi + OPTYPE);
	writeb(sst25_opcode[sst25_wrdi], s->spi + OPMENU);
	writel(0, s->spi + PBR0);
	writel(0, s->spi + PBR1);
	writel(0, s->spi + PBR2);

	/* enable the SPI controller in the GCS register */
	s->old_gcs = readl(s->gcs);
	writel((s->old_gcs & ~GCS_BBS_MASK) | GCS_BBS_SPI, s->gcs);
	if ((readl(s->gcs) & GCS_BBS_MASK) != GCS_BBS_SPI) {
		dev_err(&p->dev, "Can't enable the ICH7 SPI controller.\n");
		goto restore_gcs;
	}

	/* Enable writes to the BIOS.
	 * Since the SPI flash can serve as BIOS storage, the 'BIOS write
	 * enable' bit of the LPC BIOS_CNTL PCI configuration register must be
	 * set in order to allow writing to the flash.  This is true regardless
	 * of whether the SPI flash is actually used for BIOS.
	 */
	pci_read_config_byte(s->pci, LPC_BIOS_CNTL_REG, &s->old_bios_cntl);
	if (!(s->old_bios_cntl & BIOS_CNTL_BIOSWE) &&
	    (s->old_bios_cntl & BIOS_CNTL_BLE)) {
		dev_warn(&p->dev, "Writes to SPI flash are prohibited by"
			 " BIOS.\n");
	} else {
		pci_write_config_byte(s->pci, LPC_BIOS_CNTL_REG,
				      s->old_bios_cntl | BIOS_CNTL_BIOSWE);
	}

	return 0;

      restore_gcs:
	writel(s->old_gcs, s->gcs);
      out:
	return -ENODEV;
}

/* mtd erase method */
static int ich7_spi_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct ich7_spi_flash *s = mtd_to_spi(mtd);
	u32 addr = instr->addr;
	u32 len = instr->len;

	/* Erase request must be aligned with erase regions. */
	if ((addr & (mtd->erasesize - 1)) || (len & (mtd->erasesize - 1))) {
		DEBUG(MTD_DEBUG_LEVEL0, "%s: Unaligned erase request\n",
		      drvname);
		return -EINVAL;
	}

	/* Do not allow erase past end of device. */
	if ((len + addr) > mtd->size) {
		DEBUG(MTD_DEBUG_LEVEL0, "%s: Erase past end of device\n",
		      drvname);
		return -EINVAL;
	}

	instr->fail_addr = NO_ADDR;
	instr->state = MTD_ERASING;

	down(&s->lock);

	if ((s->opcodes & OPF(sst25_erase_chip)) && (len == mtd->size)) {
		/* erase the whole device with one command */
		ich7_spi_flash_erase_chip(s->spi);

		/* Wait for the erase operation to complete. */
		ich7_spi_flash_erase_wait(s->spi);

		goto out;
	}

	while (len > 0) {
		u32 esize;

		esize = 0x10000;	/* 64KB block */
		if ((s->opcodes & OPF(sst25_erase_64k)) && !(addr & (esize - 1))
		    && (len >= esize)) {
			/* erase a 64KB block */
			ich7_spi_flash_erase_64k(s->spi, addr);

			/* Wait for the erase operation to complete. */
			ich7_spi_flash_erase_wait(s->spi);

			len -= esize;
			addr += esize;
			continue;
		}

		esize = 0x8000;	/* 32KB block */
		if ((s->opcodes & OPF(sst25_erase_32k)) && !(addr & (esize - 1))
		    && (len >= esize)) {
			/* erase a 32KB block */
			ich7_spi_flash_erase_32k(s->spi, addr);

			/* Wait for the erase operation to complete. */
			ich7_spi_flash_erase_wait(s->spi);

			len -= esize;
			addr += esize;
			continue;
		}

		esize = 0x1000;	/* 4KB sector */
		if (!(addr & (esize - 1)) && (len >= esize)) {
			/* erase a 4KB sector */
			ich7_spi_flash_erase_sector(s->spi, addr);

			/* Wait for the erase operation to complete. */
			ich7_spi_flash_erase_wait(s->spi);

			len -= esize;
			addr += esize;
			continue;
		}
	}

      out:

	up(&s->lock);

	instr->state = MTD_ERASE_DONE;
	mtd_erase_callback(instr);

	return 0;
}

/* mtd read method */
static int ich7_spi_mtd_read(struct mtd_info *mtd, loff_t from, size_t len,
			     size_t * retlen, u_char * buf)
{
	struct ich7_spi_flash *s = mtd_to_spi(mtd);

	*retlen = 0;

	/* do not allow reads past the end of the device */
	if ((from + len) > mtd->size) {
		DEBUG(MTD_DEBUG_LEVEL0, "%s: Attempt to read beyond end of "
		      "device\n", drvname);
		return -EINVAL;
	}

	down(&s->lock);

	while (*retlen < len) {
		*retlen += ich7_spi_flash_read(s->spi, from + *retlen,
					       len - *retlen, buf + *retlen);
	}

	up(&s->lock);

	return 0;
}

/* mtd write method */
static int ich7_spi_mtd_write(struct mtd_info *mtd, loff_t to, size_t len,
			      size_t * retlen, const u_char * buf)
{
	struct ich7_spi_flash *s = mtd_to_spi(mtd);
	int aai_mode = 0;

	*retlen = 0;

	/* do not allow writes past the end of the device */
	if ((to + len) > mtd->size) {
		DEBUG(MTD_DEBUG_LEVEL0, "%s: Attempt to write beyond end of "
		      "device\n", drvname);
		return -EINVAL;
	}

	down(&s->lock);

	if (len && (to & 1)) {
		/*
		 * We're writing to an odd address, so take care of the
		 * unaligned byte first.
		 */
		*retlen += ich7_spi_flash_prog(s->spi, to, buf);
	}
	if ((len - *retlen) >= 2) {
		/* Switch to AAI mode and write the first word. */
		*retlen += ich7_spi_flash_start_aai(s->spi, to + *retlen,
						    buf + *retlen);
		aai_mode = 1;
	}
	while ((len - *retlen) >= 2) {
		*retlen += ich7_spi_flash_cont_aai(s->spi, buf + *retlen);
	}
	if (aai_mode) {
		/* Exit AAI mode. */
		ich7_spi_flash_wrdi(s->spi);
	}
	if (*retlen < len) {
		/* Write the final unaligned byte. */
		*retlen += ich7_spi_flash_prog(s->spi, to + *retlen,
					       buf + *retlen);
	}

	up(&s->lock);

	return 0;
}

/* mtd resume method */
static void ich7_spi_mtd_resume(struct mtd_info *mtd)
{
	struct platform_device *p = mtd->priv;
	struct ich7_spi_flash *s = mtd_to_spi(mtd);

	down(&s->lock);

	/* initialize the SPI controller */
	ich7_spi_startup(p);

	/* Wait for the flash to be ready. */
	ich7_spi_flash_prog_wait(s->spi);

	/* Make sure the flash is not in AAI mode. */
	ich7_spi_flash_wrdi(s->spi);

	/* Unlock the entire flash. */
	ich7_spi_flash_wrsr(s->spi, 0);

	up(&s->lock);
}

/* mtd suspend method */
static int ich7_spi_mtd_suspend(struct mtd_info *mtd)
{
	struct platform_device *p = mtd->priv;
	struct ich7_spi_flash *s = mtd_to_spi(mtd);

	down(&s->lock);

	ich7_spi_shutdown(p);

	up(&s->lock);

	return 0;
}

/* device driver resume method */
static int ich7_spi_flash_resume(struct device *dev, u32 level)
{
	struct ich7_spi_flash *s = dev_get_drvdata(dev);
	struct mtd_info *mtd = &s->mtd;

	if (level == RESUME_ENABLE)
		mtd->resume(mtd);

	return 0;
}

/* device driver suspend method */
static int ich7_spi_flash_suspend(struct device *dev, u32 state, u32 level)
{
	struct ich7_spi_flash *s = dev_get_drvdata(dev);
	struct mtd_info *mtd = &s->mtd;
	int err = 0;

	if (level == SUSPEND_DISABLE)
		err = mtd->suspend(mtd);

	return err;
}

static int __exit ich7_spi_flash_remove(struct device *dev)
{
	struct platform_device *p = to_platform_device(dev);
	struct ich7_spi_flash *s = dev_get_drvdata(dev);

	if (s->nr_parts > 0) {
#if defined(CONFIG_MTD_PARTITIONS) || defined(CONFIG_MTD_PARTITIONS_MODULE)
		del_mtd_partitions(&s->mtd);
#endif
	} else
		del_mtd_device(&s->mtd);

	ich7_spi_shutdown(p);
	iounmap(s->gcs);
	iounmap(s->spi);

	return 0;
}

struct flash_info {
	char *name;
	unsigned mfr;
	unsigned id;
	unsigned jedec_id;
	unsigned size;		/* device size in bytes */
	unsigned erasesize;
	unsigned opcodes;	/* bit mask of all supported opcodes */
};

/* bitmask of all sst25 command opcodes */
#define SST25_CMDS \
	( OPF(sst25_read)		| OPF(sst25_hsread)		\
	| OPF(sst25_erase_sector)	| OPF(sst25_erase_32k)		\
	| OPF(sst25_erase_64k)		| OPF(sst25_erase_chip)		\
	| OPF(sst25_prog)		| OPF(sst25_aai)		\
	| OPF(sst25_rdsr)		| OPF(sst25_ewsr)		\
	| OPF(sst25_wrsr)		| OPF(sst25_wren)		\
	| OPF(sst25_wrdi)		| OPF(sst25_rdid)		\
	| OPF(sst25_jedec_id)		| OPF(sst25_ebsy)		\
	| OPF(sst25_dbsy)		| OPF(sst25_ehld))

/* sst25 command sets for specific device families */
#define SST25WFXXX_CMDS (SST25_CMDS & ~(OPF(sst25_erase_64k)))
#define SST25VFXXX_CMDS (SST25_CMDS & \
	~( OPF(sst25_hsread)	| OPF(sst25_erase_64k)		\
	 | OPF(sst25_jedec_id)	| OPF(sst25_ebsy)		\
	 | OPF(sst25_dbsy)	| OPF(sst25_ehld)))
#define SST25VFXXXB_CMDS (SST25_CMDS & ~(OPF(sst25_ehld)))

/*
 * Table of supported SPI serial flash chips.
 * If you add a device to this table, keep in mind that the erase size must be
 * a multiple of one of the erase sizes supported by the chip and by the mtd
 * erase method.
 */
#define MFR_SST 0xBF
static const struct flash_info __initdata flash_data[] = {
	{"SST25WF512",  MFR_SST, 0x01, 0x2501,   64 * 1024, 4 * 1024,
	 SST25WFXXX_CMDS},
	{"SST25WF010",  MFR_SST, 0x02, 0x2502,  128 * 1024, 4 * 1024,
	 SST25WFXXX_CMDS},
	{"SST25WF020",  MFR_SST, 0x03, 0x2503,  256 * 1024, 4 * 1024,
	 SST25WFXXX_CMDS},
	{"SST25VF512",  MFR_SST, 0x48, 0x0000,   64 * 1024, 4 * 1024,
	 SST25VFXXX_CMDS},
	{"SST25VF010",  MFR_SST, 0x49, 0x0000,  128 * 1024, 4 * 1024,
	 SST25VFXXX_CMDS},
	{"SST25VF020",  MFR_SST, 0x43, 0x0000,  256 * 1024, 4 * 1024,
	 SST25VFXXX_CMDS},
	{"SST25VF040",  MFR_SST, 0x44, 0x0000,  512 * 1024, 4 * 1024,
	 SST25VFXXX_CMDS},
	{"SST25VF040B", MFR_SST, 0x8D, 0x258D,  512 * 1024, 4 * 1024,
	 SST25VFXXXB_CMDS},
	{"SST25VF080B", MFR_SST, 0x8E, 0x258E, 1024 * 1024, 4 * 1024,
	 SST25VFXXXB_CMDS},
	{"SST25VF016B", MFR_SST, 0x41, 0x2541, 2048 * 1024, 4 * 1024,
	 SST25VFXXXB_CMDS},
	{"SST25VF032B", MFR_SST, 0x4A, 0x254A, 4096 * 1024, 4 * 1024,
	 SST25VFXXXB_CMDS},
};

static int __init ich7_spi_flash_probe(struct device *dev)
{
	struct platform_device *p = to_platform_device(dev);
	struct ich7_spi_flash *s = dev_get_drvdata(dev);
	const struct flash_info *f;
	u8 mfr, id;
	int i, err;
#if defined(CONFIG_MTD_PARTITIONS) || defined(CONFIG_MTD_PARTITIONS_MODULE)
	struct mtd_partition *parts;
	static const char *part_probes[] = { "cmdlinepart", NULL };
#endif

	/* map the SPI registers */
	err = -ENOMEM;
	s->spi = ioremap_nocache(p->resource[0].start,
				 p->resource[0].end - p->resource[0].start + 1);
	if (!s->spi)
		goto out;
	/* map the GCS register */
	s->gcs = ioremap_nocache(p->resource[1].start,
				 p->resource[1].end - p->resource[1].start + 1);
	if (!s->gcs)
		goto unmap_spi;

	/* initialize the SPI controller so we can probe the flash */
	err = ich7_spi_startup(p);
	if (err)
		goto unmap_gcs;

	/* Wait for the flash to be ready. */
	ich7_spi_flash_prog_wait(s->spi);

	/* Make sure the flash is not in AAI mode. */
	ich7_spi_flash_wrdi(s->spi);

	/* Read the manufacturer and device id from the flash. */
	ich7_spi_flash_rdid(s->spi, &mfr, &id);

	/* search for a match for the manufacturer and device id */
	for (i = 0, f = flash_data; i < ARRAY_SIZE(flash_data); i++, f++) {
		if ((f->mfr == mfr) && (f->id == id)) {
			/*
			 * The manufacturer and device id match, so now verify
			 * that the JEDEC device id (if supported) also matches.
			 */
			if (f->opcodes & OPF(sst25_jedec_id)) {
				u16 jid;

				ich7_spi_flash_jedec_id(s->spi, &mfr, &jid);
				if ((f->mfr != mfr) || (f->jedec_id != jid)) {
					continue;
				}
			}
			/* we found a match */
			break;
		}
	}
	if (i == ARRAY_SIZE(flash_data)) {
		dev_warn(dev, "Flash component with mfr id 0x%02x "
			 "and device id 0x%02x is not supported.\n", mfr, id);
		err = -ENODEV;
		goto spi_shutdown;
	}

	dev_info(dev, "Found %s, size %dKB\n", f->name, f->size >> 10);

	/* Unlock the entire flash. */
	ich7_spi_flash_wrsr(s->spi, 0);

	init_MUTEX(&s->lock);
	s->opcodes = f->opcodes;

	s->mtd.type = MTD_NORFLASH;
	s->mtd.flags = MTD_CAP_NORFLASH;
	s->mtd.size = f->size;
	s->mtd.erasesize = f->erasesize;
	s->mtd.name = drvname;
	s->mtd.erase = ich7_spi_mtd_erase;
	s->mtd.read = ich7_spi_mtd_read;
	s->mtd.write = ich7_spi_mtd_write;
	s->mtd.sync = NULL;
	s->mtd.lock = NULL;
	s->mtd.unlock = NULL;
	s->mtd.suspend = ich7_spi_mtd_suspend;
	s->mtd.resume = ich7_spi_mtd_resume;
	s->mtd.priv = p;
	s->mtd.owner = THIS_MODULE;

	/* register the flash */
#if defined(CONFIG_MTD_PARTITIONS) || defined(CONFIG_MTD_PARTITIONS_MODULE)
	/* partition the flash */
	s->nr_parts = parse_mtd_partitions(&s->mtd, part_probes, &parts, 0);
	if (s->nr_parts > 0)
		err = add_mtd_partitions(&s->mtd, parts, s->nr_parts);
#endif
	if (s->nr_parts <= 0)
		err = add_mtd_device(&s->mtd);

	if (err)
		goto spi_shutdown;

	return 0;

      spi_shutdown:
	ich7_spi_shutdown(p);
      unmap_gcs:
	iounmap(s->gcs);
      unmap_spi:
	iounmap(s->spi);
      out:
	return err;
}

static struct device_driver ich7_spi_flash_driver = {
	.name = drvname,
	.bus = &platform_bus_type,
	.probe = ich7_spi_flash_probe,
	.remove = __exit_p(ich7_spi_flash_remove),
	.suspend = ich7_spi_flash_suspend,
	.resume = ich7_spi_flash_resume,
};

static void ich7_spi_flash_release(struct device *dev)
{
}

static struct ich7_spi_flash ich7_spi_flash_data;
static struct resource ich7_spi_flash_res[2];

static struct platform_device ich7_spi_flash_device = {
	.name = drvname,
	.id = 0,
	.dev = {
		.driver_data = &ich7_spi_flash_data,
		.release = ich7_spi_flash_release,
		},
	.num_resources = ARRAY_SIZE(ich7_spi_flash_res),
	.resource = ich7_spi_flash_res,
};

static void __exit ich7_spi_flash_exit(void)
{
	struct ich7_spi_flash *s = dev_get_drvdata(&ich7_spi_flash_device.dev);

	platform_device_unregister(&ich7_spi_flash_device);
	driver_unregister(&ich7_spi_flash_driver);
	pci_dev_put(s->pci);
}

static struct pci_device_id ich7_pci_tbl[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_ICH7_0)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_ICH7_1)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_ICH7_31)},
	{.vendor = 0,},
};

static int __init ich7_spi_flash_init(void)
{
	struct pci_device_id *id;
	struct ich7_spi_flash *s = dev_get_drvdata(&ich7_spi_flash_device.dev);
	int err = -ENODEV;

	for (id = ich7_pci_tbl; id->vendor; id++) {
		s->pci = pci_get_device(id->vendor, id->device, NULL);
		if (s->pci) {
			u32 rcba;

			err = pci_read_config_dword(s->pci, LPC_RCBA_REG,
						    &rcba);
			if (err)
				goto pci_put;

			rcba &= 0xFFFFC000;	/* 16KB alignment */

			/* initialize the resource for the SPI registers */
			ich7_spi_flash_res[0].name = "ich7_spi";
			ich7_spi_flash_res[0].start = rcba + ICH7_SPI_OFFSET;
			ich7_spi_flash_res[0].end = ich7_spi_flash_res[0].start
			    + ICH7_SPI_SIZE - 1;
			ich7_spi_flash_res[0].flags = IORESOURCE_MEM
			    | IORESOURCE_BUSY;

			/* initialize the resource for the GCS register */
			ich7_spi_flash_res[1].name = "ich7_gcs";
			ich7_spi_flash_res[1].start = rcba + ICH7_GCS_OFFSET;
			ich7_spi_flash_res[1].end = ich7_spi_flash_res[1].start
			    + ICH7_GCS_SIZE - 1;
			ich7_spi_flash_res[1].flags = IORESOURCE_MEM
			    | IORESOURCE_BUSY;
			break;
		}
	}

	if (err)
		goto out;

	err = driver_register(&ich7_spi_flash_driver);
	if (err)
		goto pci_put;

	err = platform_device_register(&ich7_spi_flash_device);
	if (err)
		goto driver_unreg;

	return 0;

      driver_unreg:
	driver_unregister(&ich7_spi_flash_driver);
      pci_put:
	pci_dev_put(s->pci);
      out:
	return err;
}

module_init(ich7_spi_flash_init);
module_exit(ich7_spi_flash_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andy Lowe");
MODULE_DESCRIPTION("MTD driver for ICH7 SPI Serial Flash");
