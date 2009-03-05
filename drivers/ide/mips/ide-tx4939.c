/*
 * drivers/ide/mips/ide-tx4939.c
 *
 * TX4939 internal IDE driver
 *
 * (C) Copyright TOSHIBA CORPORATION SEMICONDUCTOR COMPANY 2000-2001,2005
 *
 * 2001-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Support for TX4939 in 2.6 - Hiroshi DOYU <Hiroshi_DOYU@montavista.co.jp>
 */

#include <linux/types.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/hdreg.h>
#include <linux/ide.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <asm/bootinfo.h>
#include <asm/pci.h>
#include <asm/delay.h>
#include <asm/tx4939/tx4939.h>

/* from ide-cd.h */
#define CD_FRAMESIZE    2048
#define SECTOR_BITS                     9
#define SECTORS_PER_FRAME       (CD_FRAMESIZE >> SECTOR_BITS)

/* add command line TX4939 IDE DMA setteing */
byte tx4939_ide_udma_mode;

/* tx4939 ata transfer mode */
static u16 transfer_mode[4];

/* wait for transfer end */
static u16 wait_transfer_end[2];

#define IS_ATAPI(drive) ((drive)->media == ide_cdrom || (drive)->media == ide_scsi)
#define GET_CH(hwif) (hwif->irq == TX4939_IRQ_ATA(0) ? 0 : 1)

void tx4939_ide_softreset(ide_drive_t * drive)
{
	int ch = GET_CH(HWIF(drive));
	u16 lo_bcnt, hi_bcnt, s;

	/* save ata controller valiable */
	lo_bcnt = reg_rd16(&tx4939_ataptr(ch)->lo_bcnt);
	hi_bcnt = reg_rd16(&tx4939_ataptr(ch)->hi_bcnt);

	/* Soft Reset */
	s = reg_rd16(&tx4939_ataptr(ch)->sysctl);
	reg_wr16(&tx4939_ataptr(ch)->sysctl, s|TX4939_ATA_SC_SOFT_RESET);
	wbflush();
	udelay(1);

	/* load ata controller valiable */
	reg_wr16(&tx4939_ataptr(ch)->lo_bcnt, lo_bcnt);
	reg_wr16(&tx4939_ataptr(ch)->hi_bcnt, hi_bcnt);
}

/**
 * tx4939_ide_tune_drive - ide_tuneproc_t function for TX4939-IDE
 * @drive: This is the drive data.
 * @pio: This is used to select the PIO mode by number (0,1,2,3,4,5).
 *
 * An ide_tuneproc_t() is used to set the speed of an IDE interface
 * to a particular PIO mode.  The "byte" parameter is used
 * to select the PIO mode by number (0,1,2,3,4,5), and a value of 255
 * indicates that the interface driver should "auto-tune" the PIO mode
 * according to the drive capabilities in drive->id;
 *
 * Not all interface types support tuning, and not all of those
 * support all possible PIO settings.  They may silently ignore
 * or round values as they see fit.
 */

static void tx4939_ide_tune_drive(ide_drive_t * drive, byte pio)
{
	u16 mode = 0;
	byte speed = XFER_PIO_0;
	int is_slave = (&HWIF(drive)->drives[1] == drive ? 1 : 0);
	int ch = GET_CH(HWIF(drive));

	pio = ide_get_best_pio_mode(drive, pio, 4, NULL);
	switch (pio) {
	case 4:
		mode = TX4939_ATA_SC_MODE_XFER_PIO_4;
		speed = XFER_PIO_4;
		break;
	case 3:
		mode = TX4939_ATA_SC_MODE_XFER_PIO_3;
		speed = XFER_PIO_3;
		break;
	case 2:
		mode = TX4939_ATA_SC_MODE_XFER_PIO_2;
		speed = XFER_PIO_2;
		break;
	case 1:
		mode = TX4939_ATA_SC_MODE_XFER_PIO_1;
		speed = XFER_PIO_1;
		break;
	case 0:
		mode = TX4939_ATA_SC_MODE_XFER_PIO_0;
		speed = XFER_PIO_0;
		break;
	default:
		break;
	}

	reg_wr16(&tx4939_ataptr(ch)->sysctl, mode);
	transfer_mode[ch*2 + is_slave] = mode;

	drive->current_speed = speed;
	ide_config_drive_speed(drive, speed);
}

/**
 * tx4939_ide_tune_chipset - ide_speedproc_t function for TX4939-IDE
 * @drive: This is the drive data.
 * @speed: This parameter is used to select the transfer mode.
 *
 * This function sets the transfer mode.
 */

static int tx4939_ide_tune_chipset(ide_drive_t * drive, byte speed)
{
	u16 mode;
	int err;

	int is_slave = (&HWIF(drive)->drives[1] == drive ? 1 : 0);
	int ch = GET_CH(HWIF(drive));

	tx4939_ide_tune_drive(drive, 4);
	switch (speed) {
	case XFER_UDMA_5:
		mode = TX4939_ATA_SC_MODE_XFER_UDMA_5;
		break;
	case XFER_UDMA_4:
		mode = TX4939_ATA_SC_MODE_XFER_UDMA_4;
		break;
	case XFER_UDMA_3:
		mode = TX4939_ATA_SC_MODE_XFER_UDMA_3;
		break;
	case XFER_UDMA_2:
		mode = TX4939_ATA_SC_MODE_XFER_UDMA_2;
		break;
	case XFER_UDMA_1:
		mode = TX4939_ATA_SC_MODE_XFER_UDMA_1;
		break;
	case XFER_UDMA_0:
		mode = TX4939_ATA_SC_MODE_XFER_UDMA_0;
		break;
	case XFER_MW_DMA_2:
		mode = TX4939_ATA_SC_MODE_XFER_MDMA_2;
		break;
	case XFER_MW_DMA_1:
		mode = TX4939_ATA_SC_MODE_XFER_MDMA_1;
		break;
	case XFER_MW_DMA_0:
		mode = TX4939_ATA_SC_MODE_XFER_MDMA_0;
		break;
	default:
		return -1;
	}
	mode &= ~TX4939_ATA_SC_CMD_MODE_MASK;
	mode |= (transfer_mode[ch*2 + is_slave] & TX4939_ATA_SC_CMD_MODE_MASK);
	reg_wr16(&tx4939_ataptr(ch)->sysctl, mode);
	transfer_mode[ch*2 + is_slave] = mode;

	if (!drive->init_speed) {
		drive->init_speed = speed;
	}

	drive->current_speed = speed;
	err = ide_config_drive_speed(drive, speed);

	return err;
}

/* called from ide_cdrom_setup */
void tx4939_ide_cdrom_setup(ide_drive_t * drive)
{
	unsigned int ch = GET_CH(HWIF(drive));
	ide_hwif_t *hwif = HWIF(drive);
	int is_slave = (&hwif->drives[1] == drive);

	if (!is_slave)
		reg_wr16(&tx4939_ataptr(ch)->xfer_cnt1, CD_FRAMESIZE / 2);	/* word, not byte */
	else
		reg_wr16(&tx4939_ataptr(ch)->xfer_cnt2, CD_FRAMESIZE / 2);
}

/* called from cdrom_transfer_packet_bytes */
static void
tx4939_ide_atapi_output_bytes(ide_drive_t * drive, void *buffer,
				unsigned int bytecount)
{
	unsigned int ch = GET_CH(HWIF(drive));
	unsigned int i;
	u16 s;

	if (bytecount < 12) {
		printk(KERN_ERR "tx4939_ide_atapi_output_command: bad count %d\n",
		       bytecount);
		return;
	}
#if 0				/* This driver uses Packet Command Register */
	for (i = 0; i < bytecount; i += 2) {
		OUT_WORD(*(u16 *) (buffer + i), IDE_DATA_REG);
	}
	do {
	} while ((IN_BYTE(IDE_NSECTOR_REG) & 0x1) == 0x1);	/* wait C/D clear */
	return;
#endif

	/* set interrupt mask (Data Transfer End) */
	reg_wr16(&tx4939_ataptr(ch)->int_ctl, TX4939_ATA_IC_DATA_TRANSFER_END << 8);

	/* set the command packet in FIFO */
	for (i = 0; i < bytecount; i += 2) {
		reg_wr16(&tx4939_ataptr(ch)->pkt_cmd, cpu_to_le16(*(u16 *)(buffer + i)));
	}

	/* set the transfer count and the packet start bit */
	reg_wr16(&tx4939_ataptr(ch)->pkt_xfer_ct,
		((bytecount / 2 - 1) << 8) | TX4939_ATA_PTC_PACKET_START);

	/* wait for command out from FIFO (Data Transfer End) */
	do {
		s = reg_rd16(&tx4939_ataptr(ch)->int_ctl);
	} while (!(s & TX4939_ATA_IC_DATA_TRANSFER_END));
	reg_wr16(&tx4939_ataptr(ch)->int_ctl, s & TX4939_ATA_IC_DATA_TRANSFER_END);

	/* clear packet transfer count register and */
	reg_wr16(&tx4939_ataptr(ch)->pkt_xfer_ct, 0);
}

/**
 * tx4939_ide_intr - interrupt handler for TX4939-IDE
 * @hwif:
 *
 * This function calls from ide_intr function which is entry point for
 * all interrupts of IDE driver.
 *
 * If TX4939-IDE has HOSTINT interrupt (which indicates INTRQ),
 * continues ide_intr process.
 */

#define INT_ERROR_MASK  (TX4939_ATA_IC_BUS_ERROR |\
                         TX4939_ATA_IC_DEV_TIMING_ERROR |\
                         TX4939_ATA_IC_REACH_MALTIPLE_INT |\
                         TX4939_ATA_IC_ADDRESS_ERROR_INT)

static int tx4939_ide_intr(struct hwif_s *hwif)
{
	unsigned int ch = GET_CH(hwif);
	ide_drive_t *drive;
	u16 int_ctl;

	drive = hwif->drives;

	/* get and clear interrupt status */
	int_ctl = reg_rd16(&tx4939_ataptr(ch)->int_ctl);
	reg_wr16(&tx4939_ataptr(ch)->int_ctl, int_ctl);

	/* check error sattus */
	if (int_ctl & INT_ERROR_MASK) {
		if (int_ctl & TX4939_ATA_IC_ADDRESS_ERROR_INT)
			panic("%s: Address Error\n", drive->name);
		if (int_ctl & TX4939_ATA_IC_REACH_MALTIPLE_INT)
			panic("%s: PIO transfer in the break state\n",
                              drive->name);
#ifdef  DEBUG
		if (int_ctl & TX4939_ATA_IC_DEV_TIMING_ERROR)
			printk(KERN_INFO "%s: Device timing error (out of spec) - 0x%04x\n",
			       drive->name, reg_rd16(&tx4939_ataptr(ch)->dev_terr));
		if (int_ctl & TX4939_ATA_IC_DMA_DEV_TERMINATE) {
			printk(KERN_INFO
			       "%s: The device terminated DMA transfer\n",
			       drive->name);
		}
#endif
		if (int_ctl & TX4939_ATA_IC_BUS_ERROR)
			panic("%s: Bus error\n", drive->name);
	}

	/* wait for transfer end in DMA mode */
	if (drive->waiting_for_dma == 1) {
		wait_transfer_end[ch] |= int_ctl;
		if ((wait_transfer_end[ch] & 0x3) == 0x3)
			return 1;
		/* On error, XFEREND might not be asserted. */
		if ((int_ctl & TX4939_ATA_IC_HOSTINT) &&
		    (hwif->INB(IDE_ALTSTATUS_REG) &
		     (BUSY_STAT|DRQ_STAT|ERR_STAT)) == ERR_STAT)
			return 1;
		return 0;
	}
	return (int_ctl & TX4939_ATA_IC_HOSTINT);
	/* HOSTINT indicates that INTRQ */
}

/* returns 1 on error, 0 otherwise */
static int tx4939_ide_dma_end(ide_drive_t *drive)
{
	ide_hwif_t *hwif = HWIF(drive);
	unsigned int ch = GET_CH(HWIF(drive));

	u8 dma_stat = 0, dma_cmd = 0;

	drive->waiting_for_dma = 0;
	/* get dma_command mode */
	dma_cmd = hwif->INB(hwif->dma_command);
	/* stop DMA */
	hwif->OUTB(dma_cmd&~1, hwif->dma_command);
	/* get DMA status */
	dma_stat = hwif->INB(hwif->dma_status);
	/* clear the INTR & ERROR bits */
	hwif->OUTB(dma_stat|6, hwif->dma_status);
	/* purge DMA mappings */
	ide_destroy_dmatable(drive);

	return (reg_rd16(&tx4939_ataptr(ch)->sec_cnt) != 0);
}

static int tx4939_ide_dma_test_irq (ide_drive_t *drive)
{
	return 1;
}

/**
 * tx4939_ide_config_drive_for_dma
 * @drive
 *
 */

static int
tx4939_ide_config_drive_for_dma(ide_drive_t * drive)
{
	ide_hwif_t *hwif = HWIF(drive);
	struct hd_driveid *id = drive->id;
	int ultra = 1;
	byte speed = 0;
	byte udma_66 = eighty_ninty_three(drive);	/* hwif->udma_four 0:default 1:ATA66 */

	if ((id->dma_ultra & 0x0020) && (ultra)) {
		speed = (udma_66) ? XFER_UDMA_5 : XFER_UDMA_2;
	} else if ((id->dma_ultra & 0x0010) && (ultra)) {
		speed = (udma_66) ? XFER_UDMA_4 : XFER_UDMA_2;
	} else if ((id->dma_ultra & 0x0008) && (ultra)) {
		speed = (udma_66) ? XFER_UDMA_3 : XFER_UDMA_1;
	} else if ((id->dma_ultra & 0x0004) && (ultra)) {
		speed = XFER_UDMA_2;
	} else if ((id->dma_ultra & 0x0002) && (ultra)) {
		speed = XFER_UDMA_1;
	} else if ((id->dma_ultra & 0x0001) && (ultra)) {
		speed = XFER_UDMA_0;
	} else if (id->dma_mword & 0x0004) {
		speed = XFER_MW_DMA_2;
	} else if (id->dma_mword & 0x0002) {
		speed = XFER_MW_DMA_1;
	} else if (id->dma_1word & 0x0004) {
		speed = XFER_SW_DMA_2;
	} else {
		/* speed = XFER_PIO_0 + ide_get_best_pio_mode(drive, 255, 5, NULL); */
		return hwif->ide_dma_off_quietly(drive);
	}

	/* add command line TX4939IDE DMA setting */
	if (tx4939_ide_udma_mode && speed >= XFER_UDMA_0) {
		speed =
		    (tx4939_ide_udma_mode >
		     speed) ? speed : tx4939_ide_udma_mode;
	}

	tx4939_ide_tune_chipset(drive, speed);

	return ((int) ((id->dma_ultra >> 11) & 7) ? hwif->ide_dma_on(drive) :
		((id->dma_ultra >> 8) & 7) ? hwif->ide_dma_on(drive) :
		((id->dma_mword >> 8) & 7) ? hwif->ide_dma_on(drive) :
		((id->dma_1word >> 8) & 7) ? hwif->ide_dma_on(drive) : hwif->ide_dma_off_quietly(drive));
}


/**
 *	tx4939_ide_dma_check		-	check DMA setup
 *	@drive: drive to check
 *
 *	Don't use - due for extermination
 */

static int tx4939_ide_dma_check (ide_drive_t *drive)
{
	return tx4939_ide_config_drive_for_dma(drive);
}

/**
 *	tx4939_ide_dma_setup	-	begin a DMA phase
 *	@drive: target device
 *
 *	Build an IDE DMA PRD (IDE speak for scatter gather table)
 *	and then set up the DMA transfer registers for a device
 *	that follows generic IDE PCI DMA behaviour. Controllers can
 *	override this function if they need to
 *
 *	Returns 0 on success. If a PIO fallback is required then 1
 *	is returned.
 */

static int tx4939_ide_dma_setup(ide_drive_t *drive)
{
	unsigned int ch = GET_CH(HWIF(drive));
	ide_hwif_t *hwif = HWIF(drive);
	struct request *rq = HWGROUP(drive)->rq;
	int ret;

	/* set sector count (cpu:c10h) */
	if (!IS_ATAPI(drive)) {
		reg_wr16(&tx4939_ataptr(ch)->sec_cnt, rq->nr_sectors);
	} else {
		unsigned int nframes = 0;
		struct bio *b = rq->bio;
		int is_slave = (&hwif->drives[1] == drive);

		do {
			nframes += b->bi_size;
		} while ((b = b->bi_next) != NULL);
		nframes /= CD_FRAMESIZE;
		reg_wr16(&tx4939_ataptr(ch)->sec_cnt, nframes);

		if (!is_slave)
			reg_wr16(&tx4939_ataptr(ch)->xfer_cnt1, CD_FRAMESIZE / 2); /* word, not byte */
		else
			reg_wr16(&tx4939_ataptr(ch)->xfer_cnt2, CD_FRAMESIZE / 2);

	}

	wait_transfer_end[ch] = 0;

	ret = ide_dma_setup(drive);
#ifndef CONFIG_CPU_LITTLE_ENDIAN
	/*
	 * swap the IDE DMA table for Big Endian, address and length, too.
	 */
	if (ret == 0) {
		unsigned int *table = hwif->dmatable_cpu;
		while (1) {
			le64_to_cpus((u64 *)table);
			if (*table & 0x80000000)
				break;
			table += 2;
		}
	}
#endif	/* CONFIG_CPU_LITTLE_ENDIAN */

#ifndef DEBUG
	{
		/*
		 * mask the interrupt of Ultra DMA Dev Terminate for the performance.
		 */
		u16 int_ctl;
		int_ctl = reg_rd16(&tx4939_ataptr(ch)->int_ctl);
		int_ctl &= TX4939_ATA_IC_MASK_ALL;
		int_ctl |= TX4939_ATA_IC_DMA_DEV_TERMINATE << 8;
		reg_wr16(&tx4939_ataptr(ch)->int_ctl, int_ctl);
	}
#endif

	return ret;
}

static void tx4939_ide_outb (u8 val, unsigned long port)
{
	outb(val, port);
	/* if Device/Head register access */
	if ((port & 0xfff) == offsetof(struct tx4939_ata_reg, device)) {
		unsigned int ch;
		u8 dev;

		ch = ((port & 0xf000) == 0x4000 ? 1 : 0);
		dev = (val & 0x10 ? 1 : 0); /* DEV bit */

		/* rewrite transfer mode to System Control Register */
		reg_wr16(&tx4939_ataptr(ch)->sysctl, transfer_mode[ch*2 + dev]);
	}
}

#ifndef CONFIG_CPU_LITTLE_ENDIAN
static void tx4939_ide_insw (unsigned long port, void *addr, u32 count)
{
	u16 *__addr = addr;
	port &= ~0x1;
	while (count--) {
		*__addr++ = le16_to_cpu(__raw_inw((void *)port));
	}
}

static void tx4939_ide_outsw (unsigned long port, void *addr, u32 count)
{
	u16 *__addr = addr;
	port &= ~0x1;
	while (count--) {
		__raw_outw(cpu_to_le16(*__addr++), (void *)port);
	}
}
#endif	/* CONFIG_CPU_LITTLE_ENDIAN */

/**
 * tx4939_ide_init - initialize TX4939 IDE function
 * @ch: TX4939-ATA controller channel
 *
 */
void __init tx4939_ide_init(int ch)
{
	hw_regs_t hw;
	int idx;
	int offsets[IDE_NR_PORTS];
	void *base;
	ide_hwif_t *hwif;
	unsigned long dma_base;

	offsets[IDE_DATA_OFFSET]    = offsetof(struct tx4939_ata_reg, data);
	offsets[IDE_ERROR_OFFSET]   = offsetof(struct tx4939_ata_reg, error);
	offsets[IDE_NSECTOR_OFFSET] = offsetof(struct tx4939_ata_reg, sector);
	offsets[IDE_SECTOR_OFFSET]  = offsetof(struct tx4939_ata_reg, low);
	offsets[IDE_LCYL_OFFSET]    = offsetof(struct tx4939_ata_reg, mid);
	offsets[IDE_HCYL_OFFSET]    = offsetof(struct tx4939_ata_reg, high);
	offsets[IDE_SELECT_OFFSET]  = offsetof(struct tx4939_ata_reg, device);
	offsets[IDE_STATUS_OFFSET]  = offsetof(struct tx4939_ata_reg, status);
	offsets[IDE_CONTROL_OFFSET] = offsetof(struct tx4939_ata_reg, alt_devctl);
	base = (void *)(TX4939_ATA_REG(ch) - mips_io_port_base);

	memset(&hw, 0, sizeof(hw));

	ide_setup_ports(&hw, (unsigned long)base, offsets,
			0, 0, tx4939_ide_intr, TX4939_IRQ_ATA(ch));

	idx = ide_register_hw(&hw, NULL);
	if (idx == -1) {
		printk(KERN_ERR "%s: IDE I/F register failed\n", __FILE__);
		return;
	}

	hwif = &ide_hwifs[idx];
	printk(KERN_INFO "%s: TX4939 IDE interface\n", hwif->name);

	/* set hwif functions */
	hwif->chipset = ide_tx4939;
	hwif->tuneproc = tx4939_ide_tune_drive;
	hwif->speedproc = tx4939_ide_tune_chipset;

	hwif->ultra_mask = 0x7f;
	hwif->mwdma_mask = 0x07;
	hwif->swdma_mask = 0x07;

	hwif->mmio = 0;

	hwif->OUTB = tx4939_ide_outb;
#ifndef CONFIG_CPU_LITTLE_ENDIAN
	hwif->INSW 	= tx4939_ide_insw;
	hwif->OUTSW 	= tx4939_ide_outsw;
#endif	/* CONFIG_CPU_LITTLE_ENDIAN */

	/* cable(PDIAGN) check */
	if (!(hwif->udma_four)) {
		unsigned int pdiagn;
		/* bit13(PDIAGN) = 0:(80pin cable) 1:(40pin cable) */
		pdiagn = (reg_rd16(&tx4939_ataptr(ch)->sysctl) & TX4939_ATA_SC_PDIAGN);
		hwif->udma_four = pdiagn ? 0 : 1;
	}
#ifdef CONFIG_BLK_DEV_IDEDMA
	hwif->autodma = 1;
	hwif->atapi_dma = 1;
	hwif->ide_dma_check = &tx4939_ide_dma_check;
	hwif->dma_setup = &tx4939_ide_dma_setup;
	hwif->ide_dma_test_irq = &tx4939_ide_dma_test_irq;
	hwif->ide_dma_end = &tx4939_ide_dma_end;
	hwif->atapi_output_bytes = &tx4939_ide_atapi_output_bytes;
        dma_base = TX4939_ATA_REG(ch) + offsetof(struct tx4939_ata_reg, dma_cmd) - mips_io_port_base;
        hwif->dma_status   = TX4939_ATA_REG(ch) + offsetof(struct tx4939_ata_reg, dma_stat) - mips_io_port_base;
        hwif->dma_prdtable = TX4939_ATA_REG(ch) + offsetof(struct tx4939_ata_reg, prd_tbl) - mips_io_port_base;

	ide_setup_dma(hwif, dma_base, 8);
#endif	/*CONFIG_BLK_DEV_IDEDMA */
}

