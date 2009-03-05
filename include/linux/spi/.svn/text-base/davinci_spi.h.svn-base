/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __DAVINCI_SPI_H_
#define __DAVINCI_SPI_H_

#define DAVINCI_SPI_SIZE				0xFF
#define DAVINCI_SPI_INIT_SPMODE			0

/* Chip selects available on the spi IP */
#define DAVINCI_SPI_CS0		0
#define DAVINCI_SPI_CS1		1
#define DAVINCI_SPI_CS2		2
#define DAVINCI_SPI_CS3		3

struct spi_cmd_t {
	unsigned int addr;
	unsigned int data;
};

struct ctlr_cs_sel_t {
	u8 cs;
	u8 pol;
};

struct davinci_spi_platform_data {
	/* initial SPMODE value */
	u32 initial_spmode;
	/* board specific information */
	u16 bus_num;		/* id for controller */
	u16 max_chipselect;
	int (*activate_cs) (u8 cs, u8 polarity,
			    struct ctlr_cs_sel_t *ctlr_cs_sel);
	int (*deactivate_cs) (u8 cs, u8 polarity,
			      struct ctlr_cs_sel_t *ctlr_cs_sel);
	u32 sysclk;
};

#endif				/* __DAVINCI_SPI_H_ */
