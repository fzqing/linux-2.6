/*
 * DaVinci SPI-EEPROM client driver header file
 *
 * Author: Steve Chen <schen@mvista.com>
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef DAVINCI_SPI_EEPROM_H
#define DAVINCI_SPI_EEPROM_H

#include <linux/cache.h>
/*
 *  EEPROM op-codes
 */
#define DAVINCI_EEPROM_READ	0x03	/* read */
#define DAVINCI_EEPROM_WRITE	0x02	/* write */
#define DAVINCI_EEPROM_WREN	0x06	/* write enable */
#define DAVINCI_EEPROM_WRDIS	0x04	/* write disable */
#define DAVINCI_EEPROM_RDSTAT	0x05	/* read status register */
#define DAVINCI_EEPROM_WRSTAT	0x01	/* write status register */

#define SPI_BUFFER_SIZE SMP_CACHE_BYTES
#define DAVINCI_SPI_TX_CMD_SIZE 3

#define DAVINCI_SPI_EEPROM_NAME "davinci_spi_eeprom"

struct mtd_partition;

struct davinci_eeprom_info {
	unsigned int eeprom_size;
	unsigned int page_size;
	unsigned int page_mask;
	unsigned long chip_sel;
	unsigned int commit_delay;
	struct spi_device *spi;
	struct davinci_spi *spi_data;

	struct mtd_partition *parts;
	unsigned int nr_parts;

	struct semaphore lock;
	char tx_buffer[SPI_BUFFER_SIZE + DAVINCI_SPI_TX_CMD_SIZE];
	char rx_buffer[SPI_BUFFER_SIZE];
};

#endif				/*DAVINCI_SPI_EEPROM_H */
