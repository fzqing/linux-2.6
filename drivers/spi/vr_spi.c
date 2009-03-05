/*
 *  drivers/spi/vr_spi.c
 *
 *  SPI master controller driver for the Vermilion Range SPI controller.
 *
 *  Copyright (C) 2007 MontaVista Software, Inc.
 *
 *  Derived from drivers/spi/pxa2xx_spi.c
 *  Copyright (C) 2005 Stephen Street / StreetFire Sound Labs
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/pci.h>

#include "vr_spi.h"

/* for testing SSCR1 changes that require SSP restart, basically
 * everything except the interrupt enables */
#define SSCR1_CHANGE_MASK (SSCR1_STRF | SSCR1_EFWR |SSCR1_RFT \
				| SSCR1_TFT | SSCR1_SPH | SSCR1_SPO | SSCR1_LBM)

#define START_STATE ((void*)0)
#define RUNNING_STATE ((void*)1)
#define DONE_STATE ((void*)2)
#define ERROR_STATE ((void*)-1)

#define QUEUE_RUNNING 0
#define QUEUE_STOPPED 1

struct driver_data {
	/* Driver model hookup */
	struct pci_dev *dev;

	/* SPI framework hookup */
	struct spi_master *master;

	/* SSP register addresses */
	void __iomem *ioaddr;

	/* SSP masks */
	u32 int_cr1;
	u32 clear_sr;
	u32 mask_sr;

	/* Driver message queue */
	struct workqueue_struct *workqueue;
	struct work_struct pump_messages;
	spinlock_t lock;
	struct list_head queue;
	int busy;
	int run;

	/* Message Transfer pump */
	struct tasklet_struct pump_transfers;

	/* Current message transfer state info */
	struct spi_message *cur_msg;
	struct spi_transfer *cur_transfer;
	struct chip_data *cur_chip;
	size_t len;
	void *tx;
	void *tx_end;
	void *rx;
	void *rx_end;
	u8 n_bytes;
	int cs_change;
	int (*write) (struct driver_data * drv_data);
	int (*read) (struct driver_data * drv_data);
	irqreturn_t(*transfer_handler) (struct driver_data * drv_data);
};

struct chip_data {
	u32 cr0;
	u32 cr1;
	u32 psp;
	u32 timeout;
	u8 n_bytes;
	u32 threshold;
	u8 bits_per_word;
	u8 chip_select;
	u32 speed_hz;
	int (*write) (struct driver_data * drv_data);
	int (*read) (struct driver_data * drv_data);
};

static void vr_spi_pump_messages(void *data);

static int flush(struct driver_data *drv_data)
{
	unsigned long limit = loops_per_jiffy << 1;

	void __iomem *reg = drv_data->ioaddr;

	do {
		while (readl(reg + SSSR) & SSSR_RNE) {
			readl(reg + SSDR);
		}
	} while ((readl(reg + SSSR) & SSSR_BSY) && limit--);
	writel(readl(reg + SSSR) | SSSR_ROR, reg + SSSR);

	return limit;
}

static int null_writer(struct driver_data *drv_data)
{
	void __iomem *reg = drv_data->ioaddr;
	u8 n_bytes = drv_data->n_bytes;

	if (((readl(reg + SSSR) & 0x00000f00) == 0x00000f00)
	    || (drv_data->tx == drv_data->tx_end))
		return 0;

	writel(0, reg + SSDR);
	drv_data->tx += n_bytes;

	return 1;
}

static int null_reader(struct driver_data *drv_data)
{
	void __iomem *reg = drv_data->ioaddr;
	u8 n_bytes = drv_data->n_bytes;

	while ((readl(reg + SSSR) & SSSR_RNE)
	       && (drv_data->rx < drv_data->rx_end)) {
		readl(reg + SSDR);
		drv_data->rx += n_bytes;
	}

	return drv_data->rx == drv_data->rx_end;
}

static int u8_writer(struct driver_data *drv_data)
{
	void __iomem *reg = drv_data->ioaddr;

	if (((readl(reg + SSSR) & 0x00000f00) == 0x00000f00)
	    || (drv_data->tx == drv_data->tx_end))
		return 0;

	writel(*(u8 *) (drv_data->tx), reg + SSDR);
	++drv_data->tx;

	return 1;
}

static int u8_reader(struct driver_data *drv_data)
{
	void __iomem *reg = drv_data->ioaddr;

	while ((readl(reg + SSSR) & SSSR_RNE)
	       && (drv_data->rx < drv_data->rx_end)) {
		*(u8 *) (drv_data->rx) = readl(reg + SSDR);
		++drv_data->rx;
	}

	return drv_data->rx == drv_data->rx_end;
}

static int u16_writer(struct driver_data *drv_data)
{
	void __iomem *reg = drv_data->ioaddr;

	if (((readl(reg + SSSR) & 0x00000f00) == 0x00000f00)
	    || (drv_data->tx == drv_data->tx_end))
		return 0;

	writel(*(u16 *) (drv_data->tx), reg + SSDR);
	drv_data->tx += 2;

	return 1;
}

static int u16_reader(struct driver_data *drv_data)
{
	void __iomem *reg = drv_data->ioaddr;

	while ((readl(reg + SSSR) & SSSR_RNE)
	       && (drv_data->rx < drv_data->rx_end)) {
		*(u16 *) (drv_data->rx) = readl(reg + SSDR);
		drv_data->rx += 2;
	}

	return drv_data->rx == drv_data->rx_end;
}

static void *next_transfer(struct driver_data *drv_data)
{
	struct spi_message *msg = drv_data->cur_msg;
	struct spi_transfer *trans = drv_data->cur_transfer;

	/* Move to next transfer */
	if (trans->transfer_list.next != &msg->transfers) {
		drv_data->cur_transfer =
		    list_entry(trans->transfer_list.next,
			       struct spi_transfer, transfer_list);
		return RUNNING_STATE;
	} else
		return DONE_STATE;
}

/* caller already set message->status; dma and pio irqs are blocked */
static void giveback(struct driver_data *drv_data)
{
	struct spi_transfer *last_transfer;
	unsigned long flags;
	struct spi_message *msg;

	spin_lock_irqsave(&drv_data->lock, flags);
	msg = drv_data->cur_msg;
	drv_data->cur_msg = NULL;
	drv_data->cur_transfer = NULL;
	drv_data->cur_chip = NULL;
	queue_work(drv_data->workqueue, &drv_data->pump_messages);
	spin_unlock_irqrestore(&drv_data->lock, flags);

	last_transfer = list_entry(msg->transfers.prev,
				   struct spi_transfer, transfer_list);

	msg->state = NULL;
	if (msg->complete)
		msg->complete(msg->context);
}

static int wait_ssp_rx_stall(void __iomem * ioaddr)
{
	unsigned long limit = loops_per_jiffy << 1;

	while ((readl(ioaddr + SSSR) & SSSR_BSY) && limit--)
		cpu_relax();

	return limit;
}

static void int_error_stop(struct driver_data *drv_data, const char *msg)
{
	void __iomem *reg = drv_data->ioaddr;

	/* Stop and reset SSP */
	writel(readl(reg + SSSR) | drv_data->clear_sr, reg + SSSR);
	writel(readl(reg + SSCR1) & ~drv_data->int_cr1, reg + SSCR1);
	flush(drv_data);
	writel(readl(reg + SSCR0) & ~SSCR0_SSE, reg + SSCR0);

	dev_err(&drv_data->dev->dev, "%s\n", msg);

	drv_data->cur_msg->state = ERROR_STATE;
	tasklet_schedule(&drv_data->pump_transfers);
}

static void int_transfer_complete(struct driver_data *drv_data)
{
	void __iomem *reg = drv_data->ioaddr;

	/* Stop SSP */
	writel(readl(reg + SSSR) | drv_data->clear_sr, reg + SSSR);
	writel(readl(reg + SSCR1) & ~drv_data->int_cr1, reg + SSCR1);

	/* Update total byte transfered return count actual bytes read */
	drv_data->cur_msg->actual_length += drv_data->len -
	    (drv_data->rx_end - drv_data->rx);

	/* Move to next transfer */
	drv_data->cur_msg->state = next_transfer(drv_data);

	/* Schedule transfer tasklet */
	tasklet_schedule(&drv_data->pump_transfers);
}

static irqreturn_t interrupt_transfer(struct driver_data *drv_data)
{
	void __iomem *reg = drv_data->ioaddr;

	u32 irq_mask = (readl(reg + SSCR1) & SSCR1_TIE) ?
	    drv_data->mask_sr : drv_data->mask_sr & ~SSSR_TFS;

	u32 irq_status = readl(reg + SSSR) & irq_mask;

	if (irq_status & SSSR_ROR) {
		int_error_stop(drv_data, "interrupt_transfer: fifo overrun");
		return IRQ_HANDLED;
	}

	/* Drain rx fifo, Fill tx fifo and prevent overruns */
	do {
		if (drv_data->read(drv_data)) {
			int_transfer_complete(drv_data);
			return IRQ_HANDLED;
		}
	} while (drv_data->write(drv_data));

	if (drv_data->read(drv_data)) {
		int_transfer_complete(drv_data);
		return IRQ_HANDLED;
	}

	if (drv_data->tx == drv_data->tx_end) {
		writel(readl(reg + SSCR1) & ~SSCR1_TIE, reg + SSCR1);
		/* read trailing bytes */
		if (!wait_ssp_rx_stall(reg)) {
			int_error_stop(drv_data, "interrupt_transfer: "
				       "rx stall failed");
			return IRQ_HANDLED;
		}
		if (!drv_data->read(drv_data)) {
			int_error_stop(drv_data,
				       "interrupt_transfer: "
				       "trailing byte read failed");
			return IRQ_HANDLED;
		}
		int_transfer_complete(drv_data);
	}

	/* We did something */
	return IRQ_HANDLED;
}

static irqreturn_t vr_spi_int(int irq, void *dev_id, struct pt_regs *pt_regs)
{
	struct driver_data *drv_data = dev_id;
	void __iomem *reg = drv_data->ioaddr;

	if (!(readl(reg + SSSR) & drv_data->mask_sr)) {
		/*
		 * This isn't our interrupt.  It must be for another device
		 * sharing this IRQ.
		 */
		return IRQ_NONE;
	}

	if (!drv_data->cur_msg) {

		writel(readl(reg + SSCR0) & ~SSCR0_SSE, reg + SSCR0);
		writel(readl(reg + SSCR1) & ~drv_data->int_cr1, reg + SSCR1);
		writel(readl(reg + SSSR) | drv_data->clear_sr, reg + SSSR);

		dev_err(&drv_data->dev->dev, "bad message state "
			"in interrupt handler\n");

		/* Never fail */
		return IRQ_HANDLED;
	}

	return drv_data->transfer_handler(drv_data);
}

static void vr_spi_pump_transfers(unsigned long data)
{
	struct driver_data *drv_data = (struct driver_data *)data;
	struct spi_message *message = NULL;
	struct spi_transfer *transfer = NULL;
	struct spi_transfer *previous = NULL;
	struct chip_data *chip = NULL;
	void __iomem *reg = drv_data->ioaddr;
	u32 cr0;
	u32 cr1;

	/* Get current state information */
	message = drv_data->cur_msg;
	transfer = drv_data->cur_transfer;
	chip = drv_data->cur_chip;

	/* Handle for abort */
	if (message->state == ERROR_STATE) {
		message->status = -EIO;
		giveback(drv_data);
		return;
	}

	/* Handle end of message */
	if (message->state == DONE_STATE) {
		message->status = 0;
		giveback(drv_data);
		return;
	}

	/* Delay if requested at end of transfer */
	if (message->state == RUNNING_STATE) {
		previous = list_entry(transfer->transfer_list.prev,
				      struct spi_transfer, transfer_list);
		if (previous->delay_usecs)
			udelay(previous->delay_usecs);
	}

	/* Check transfer length */
	if (transfer->len > 8191) {
		dev_warn(&drv_data->dev->dev, "pump_transfers: transfer "
			 "length greater than 8191\n");
		message->status = -EINVAL;
		giveback(drv_data);
		return;
	}

	/* Setup the transfer state based on the type of transfer */
	if (flush(drv_data) == 0) {
		dev_err(&drv_data->dev->dev, "pump_transfers: flush failed\n");
		message->status = -EIO;
		giveback(drv_data);
		return;
	}
	drv_data->n_bytes = chip->n_bytes;
	drv_data->tx = (void *)transfer->tx_buf;
	drv_data->tx_end = drv_data->tx + transfer->len;
	drv_data->rx = transfer->rx_buf;
	drv_data->rx_end = drv_data->rx + transfer->len;
	drv_data->len = transfer->len;
	drv_data->write = drv_data->tx ? chip->write : null_writer;
	drv_data->read = drv_data->rx ? chip->read : null_reader;
	drv_data->cs_change = transfer->cs_change;

	cr0 = chip->cr0;

	message->state = RUNNING_STATE;

	/* Ensure we have the correct interrupt handler */
	drv_data->transfer_handler = interrupt_transfer;

	/* Clear status  */
	cr1 = chip->cr1 | chip->threshold | drv_data->int_cr1;
	writel(drv_data->clear_sr | (chip->chip_select ? SSSR_ALT_FRM : 0),
	       reg + SSSR);

	/* see if we need to reload the config registers */
	if ((readl(reg + SSCR0) != cr0)
	    || (readl(reg + SSCR1) & SSCR1_CHANGE_MASK) !=
	    (cr1 & SSCR1_CHANGE_MASK)) {

		writel(cr0 & ~SSCR0_SSE, reg + SSCR0);
		writel(cr1, reg + SSCR1);
		writel(cr0, reg + SSCR0);
	} else {
		writel(cr1, reg + SSCR1);
	}
}

static void vr_spi_pump_messages(void *data)
{
	struct driver_data *drv_data = data;
	unsigned long flags;

	/* Lock queue and check for queue work */
	spin_lock_irqsave(&drv_data->lock, flags);
	if (list_empty(&drv_data->queue) || drv_data->run == QUEUE_STOPPED) {
		drv_data->busy = 0;
		spin_unlock_irqrestore(&drv_data->lock, flags);
		return;
	}

	/* Make sure we are not already running a message */
	if (drv_data->cur_msg) {
		spin_unlock_irqrestore(&drv_data->lock, flags);
		return;
	}

	/* Extract head of queue */
	drv_data->cur_msg = list_entry(drv_data->queue.next,
				       struct spi_message, queue);
	list_del_init(&drv_data->cur_msg->queue);

	/* Initial message state */
	drv_data->cur_msg->state = START_STATE;
	drv_data->cur_transfer = list_entry(drv_data->cur_msg->transfers.next,
					    struct spi_transfer, transfer_list);

	/* prepare to setup the SSP, in pump_transfers, using the per
	 * chip configuration */
	drv_data->cur_chip = spi_get_ctldata(drv_data->cur_msg->spi);

	/* Mark as busy and launch transfers */
	tasklet_schedule(&drv_data->pump_transfers);

	drv_data->busy = 1;
	spin_unlock_irqrestore(&drv_data->lock, flags);
}

static int vr_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct driver_data *drv_data = spi_master_get_devdata(spi->master);
	unsigned long flags;

	spin_lock_irqsave(&drv_data->lock, flags);

	if (drv_data->run == QUEUE_STOPPED) {
		spin_unlock_irqrestore(&drv_data->lock, flags);
		return -ESHUTDOWN;
	}

	msg->actual_length = 0;
	msg->status = -EINPROGRESS;
	msg->state = START_STATE;

	list_add_tail(&msg->queue, &drv_data->queue);

	if (drv_data->run == QUEUE_RUNNING && !drv_data->busy)
		queue_work(drv_data->workqueue, &drv_data->pump_messages);

	spin_unlock_irqrestore(&drv_data->lock, flags);

	return 0;
}

static int vr_spi_setup(struct spi_device *spi)
{
	struct chip_data *chip;
	unsigned int clk_div;

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	if (spi->bits_per_word < 4 || spi->bits_per_word > 16) {
		dev_err(&spi->dev, "failed setup: bits/wrd=%d b/w not 4-16\n",
			spi->bits_per_word);
		return -EINVAL;
	}

	/* Only alloc on first setup */
	chip = spi_get_ctldata(spi);
	if (!chip) {
		chip = kzalloc(sizeof(struct chip_data), GFP_KERNEL);
		if (!chip) {
			dev_err(&spi->dev,
				"failed setup: can't allocate chip data\n");
			return -ENOMEM;
		}

		chip->timeout = 1000;
		chip->threshold = SSCR1_RxThresh(4) | SSCR1_TxThresh(4);
	}

	chip->cr1 = 0;		/* disable loopback */

	clk_div = SSP_SerClkDiv(spi->max_speed_hz);
	chip->speed_hz = spi->max_speed_hz;

	chip->chip_select = spi->chip_select;

	chip->cr0 = clk_div
	    | SSCR0_Motorola
	    | SSCR0_DataSize(spi->bits_per_word > 16 ?
			     spi->bits_per_word - 16 : spi->bits_per_word)
	    | SSCR0_SSE;
	chip->cr1 |= (((spi->mode & SPI_CPHA) != 0) << 4)
	    | (((spi->mode & SPI_CPOL) != 0) << 3);

	dev_dbg(&spi->dev, "%d bits/word, %d Hz, mode %d\n",
		spi->bits_per_word, (SPI_REF_CLK_HZ / 2)
		/ (1 + ((chip->cr0 & SSCR0_SCR) >> 8)), spi->mode & 0x3);

	if (spi->bits_per_word <= 8) {
		chip->n_bytes = 1;
		chip->read = u8_reader;
		chip->write = u8_writer;
	} else if (spi->bits_per_word <= 16) {
		chip->n_bytes = 2;
		chip->read = u16_reader;
		chip->write = u16_writer;
	} else {
		dev_err(&spi->dev, "invalid wordsize\n");
		return -ENODEV;
	}
	chip->bits_per_word = spi->bits_per_word;

	spi_set_ctldata(spi, chip);

	return 0;
}

static void vr_spi_cleanup(const struct spi_device *spi)
{
	struct chip_data *chip = spi_get_ctldata((struct spi_device *)spi);

	kfree(chip);
}

static int vr_spi_init_queue(struct driver_data *drv_data)
{
	INIT_LIST_HEAD(&drv_data->queue);
	spin_lock_init(&drv_data->lock);

	drv_data->run = QUEUE_STOPPED;
	drv_data->busy = 0;

	tasklet_init(&drv_data->pump_transfers,
		     vr_spi_pump_transfers, (unsigned long)drv_data);

	INIT_WORK(&drv_data->pump_messages, vr_spi_pump_messages, drv_data);
	drv_data->workqueue = create_singlethread_workqueue("vr_spi");
	if (drv_data->workqueue == NULL)
		return -EBUSY;

	return 0;
}

static int vr_spi_start_queue(struct driver_data *drv_data)
{
	unsigned long flags;

	spin_lock_irqsave(&drv_data->lock, flags);

	if (drv_data->run == QUEUE_RUNNING || drv_data->busy) {
		spin_unlock_irqrestore(&drv_data->lock, flags);
		return -EBUSY;
	}

	drv_data->run = QUEUE_RUNNING;
	drv_data->cur_msg = NULL;
	drv_data->cur_transfer = NULL;
	drv_data->cur_chip = NULL;
	spin_unlock_irqrestore(&drv_data->lock, flags);

	queue_work(drv_data->workqueue, &drv_data->pump_messages);

	return 0;
}

static int vr_spi_stop_queue(struct driver_data *drv_data)
{
	unsigned long flags;
	unsigned limit = 500;
	int status = 0;

	spin_lock_irqsave(&drv_data->lock, flags);

	/* This is a bit lame, but is optimized for the common execution path.
	 * A wait_queue on the drv_data->busy could be used, but then the common
	 * execution path (pump_messages) would be required to call wake_up or
	 * friends on every SPI message. Do this instead */
	drv_data->run = QUEUE_STOPPED;
	while (!list_empty(&drv_data->queue) && drv_data->busy && limit--) {
		spin_unlock_irqrestore(&drv_data->lock, flags);
		msleep(10);
		spin_lock_irqsave(&drv_data->lock, flags);
	}

	if (!list_empty(&drv_data->queue) || drv_data->busy)
		status = -EBUSY;

	spin_unlock_irqrestore(&drv_data->lock, flags);

	return status;
}

static int vr_spi_destroy_queue(struct driver_data *drv_data)
{
	int status;

	status = vr_spi_stop_queue(drv_data);
	/* we are unloading the module or failing to load (only two calls
	 * to this routine), and neither call can handle a return value.
	 * However, destroy_workqueue calls flush_workqueue, and that will
	 * block until all work is done.  If the reason that stop_queue
	 * timed out is that the work will never finish, then it does no
	 * good to call destroy_workqueue, so return anyway. */
	if (status != 0)
		return status;

	destroy_workqueue(drv_data->workqueue);

	return 0;
}

static int vr_spi_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	struct spi_master *master;
	struct driver_data *drv_data = 0;
	int retval;

	/* Allocate master with space for drv_data */
	master = spi_alloc_master(&dev->dev, sizeof(struct driver_data));
	if (!master) {
		dev_err(&dev->dev, "spi_alloc_master failed\n");
		retval = -ENOMEM;
		goto out;
	}
	drv_data = spi_master_get_devdata(master);
	drv_data->master = master;
	drv_data->dev = dev;

	master->bus_num = 0;	/* 0 ==> assign bus number dynamically */
	master->num_chipselect = 2;
	master->cleanup = vr_spi_cleanup;
	master->setup = vr_spi_setup;
	master->transfer = vr_spi_transfer;

	drv_data->int_cr1 = SSCR1_TIE | SSCR1_RIE;
	drv_data->clear_sr = SSSR_ROR;
	drv_data->mask_sr = SSSR_RFS | SSSR_TFS | SSSR_ROR;

	retval = pci_enable_device(dev);
	if (retval)
		goto out_free_master;

	retval = pci_request_regions(dev, "vr_spi");
	if (retval)
		goto out_disable_dev;

	drv_data->ioaddr = pci_iomap(dev, 0, 0);
	if (!drv_data->ioaddr) {
		retval = -EFAULT;
		goto out_release_bars;
	}

	/* enable Message Signaled Interrupts */
	pci_enable_msi(dev);

	/* Attach to IRQ */
	retval = request_irq(dev->irq, vr_spi_int, SA_SHIRQ, "vr_spi",
			     drv_data);
	if (retval < 0) {
		dev_err(&dev->dev, "request_irq %d failed with error %d\n",
			dev->irq, retval);
		goto out_unmap;
	}

	/* Load default SSP configuration */
	writel(0, drv_data->ioaddr + SSCR0);
	writel(SSCR1_RxThresh(4) | SSCR1_TxThresh(4), drv_data->ioaddr + SSCR1);
	writel(SSCR0_SerClkDiv(2) | SSCR0_Motorola | SSCR0_DataSize(8),
	       drv_data->ioaddr + SSCR0);

	/* Initialize and start queue */
	retval = vr_spi_init_queue(drv_data);
	if (retval != 0) {
		dev_err(&dev->dev, "problem initializing queue\n");
		goto out_free_irq;
	}
	retval = vr_spi_start_queue(drv_data);
	if (retval != 0) {
		dev_err(&dev->dev, "problem starting queue\n");
		goto out_destroy_queue;
	}

	/* Register with the SPI framework */
	pci_set_drvdata(dev, drv_data);
	retval = spi_register_master(master);
	if (retval != 0) {
		dev_err(&dev->dev, "problem registering spi master\n");
		goto out_destroy_queue;
	}

	return retval;

      out_destroy_queue:
	vr_spi_destroy_queue(drv_data);
      out_free_irq:
	free_irq(dev->irq, drv_data);
      out_unmap:
	pci_disable_msi(dev);
	pci_iounmap(dev, drv_data->ioaddr);
      out_release_bars:
	pci_release_regions(dev);
      out_disable_dev:
	pci_disable_device(dev);
      out_free_master:
	spi_master_put(master);
      out:
	return retval;
}

static void vr_spi_remove(struct pci_dev *dev)
{
	struct driver_data *drv_data = pci_get_drvdata(dev);
	int status = 0;

	if (!drv_data)
		return;

	/* Remove the queue */
	status = vr_spi_destroy_queue(drv_data);
	if (status != 0)
		/* the kernel does not check the return status of this
		 * routine (mod->exit, within the kernel).  Therefore
		 * nothing is gained by returning from here, the module is
		 * going away regardless, and we should not leave any more
		 * resources allocated than necessary.  We cannot free the
		 * message memory in drv_data->queue, but we can release the
		 * resources below.  I think the kernel should honor -EBUSY
		 * returns but... */
		dev_err(&dev->dev, "vr_spi_remove: workqueue will not "
			"complete, message memory not freed\n");

	/* Disable the SSP */
	writel(0, drv_data->ioaddr + SSCR0);

	/* Release IRQ */
	free_irq(dev->irq, drv_data);
	pci_disable_msi(dev);

	/* Disconnect from the SPI framework */
	spi_unregister_master(drv_data->master);

	/* Prevent double remove */
	pci_set_drvdata(dev, NULL);

	pci_iounmap(dev, drv_data->ioaddr);
	pci_release_regions(dev);
	pci_disable_device(dev);
}

#ifdef CONFIG_PM
static int vr_spi_suspend_devices(struct device *dev, void *pm_message)
{
	pm_message_t *state = pm_message;

	if (dev->power.power_state.event != state->event) {
		dev_warn(dev, "pm state does not match request\n");
		return -1;
	}

	return 0;
}

static int vr_spi_suspend(struct pci_dev *dev, u32 state)
{
	struct driver_data *drv_data = pci_get_drvdata(dev);
	int status = 0;

	/* Check all children for current power state */
	if (device_for_each_child(&dev->dev, &state, vr_spi_suspend_devices)
	    != 0) {
		dev_warn(&dev->dev, "suspend aborted\n");
		return -1;
	}

	status = vr_spi_stop_queue(drv_data);
	if (status != 0)
		return status;
	writel(0, drv_data->ioaddr + SSCR0);

	return 0;
}

static int vr_spi_resume(struct pci_dev *dev)
{
	struct driver_data *drv_data = pci_get_drvdata(dev);
	int status = 0;

	/* Start the queue running */
	status = vr_spi_start_queue(drv_data);
	if (status != 0) {
		dev_err(&dev->dev, "problem starting queue (%d)\n", status);
		return status;
	}

	return 0;
}
#else
#define vr_spi_suspend NULL
#define vr_spi_resume NULL
#endif				/* CONFIG_PM */

static struct pci_device_id vr_spi_ids[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x5012)},
	{.vendor = 0,}
};

static struct pci_driver vr_spi_driver = {
	.name = "vr_spi",
	.id_table = vr_spi_ids,
	.probe = vr_spi_probe,
	.remove = __devexit_p(vr_spi_remove),
#ifdef CONFIG_PM
	.suspend = vr_spi_suspend,
	.resume = vr_spi_resume,
#endif
};

static int __init vr_spi_init(void)
{
	return pci_register_driver(&vr_spi_driver);
}

module_init(vr_spi_init);

static void __exit vr_spi_exit(void)
{
	pci_unregister_driver(&vr_spi_driver);
}

module_exit(vr_spi_exit);

MODULE_DESCRIPTION("Intel Vermilion Range SPI Controller Driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(pci, vr_spi_ids);
