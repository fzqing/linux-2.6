/*
 *  drivers/spi/vr_spi.h
 *
 *  Header file for the Vermilion Range SPI driver.
 *
 *  2007 (c) MontaVista Software, Inc. This file is licensed under
 *  the terms of the GNU General Public License version 2. This program
 *  is licensed "as is" without any warranty of any kind, whether express
 *  or implied.
 */

#ifndef VR_SPI_H_
#define VR_SPI_H_

#define SPI_REF_CLK_HZ 3686400
#define SSP_SerClkDiv(x) ((((SPI_REF_CLK_HZ/2/(x)) - 1)<<8)&0x0000ff00)

/*
 * SPI Controller Registers
 */

/* register offsets */
#define SSCR0	0x00
#define SSCR1	0x04
#define SSSR	0x08
#define SSDR	0x10

#define SSCR0_DataSize(x) ((x) - 1)	/* Data Size Select [4..16] */
#define SSCR0_Motorola	(0x0 << 4)	/* Motorola's Serial Peripheral Interface (SPI) */
#define SSCR0_SSE	(1 << 7)	/* Synchronous Serial Port Enable */
#define SSCR0_SCR	(0x0000ff00)	/* Serial Clock Rate (mask) */
#define SSCR0_SerClkDiv(x) ((((x) - 2)/2) << 8)	/* Divisor [2..512] */

#define SSCR1_RIE	(1 << 0)	/* Receive FIFO Interrupt Enable */
#define SSCR1_TIE	(1 << 1)	/* Transmit FIFO Interrupt Enable */
#define SSCR1_LBM	(1 << 2)	/* Loop-Back Mode */
#define SSCR1_SPO	(1 << 3)	/* Motorola SPI SSPSCLK polarity setting */
#define SSCR1_SPH	(1 << 4)	/* Motorola SPI SSPSCLK phase setting */
#define SSCR1_TFT	(0x000000c0)	/* Transmit FIFO Threshold (mask) */
#define SSCR1_TxThresh(x) (((x) - 1) << 6)	/* level [1..4] */
#define SSCR1_RFT	(0x00000c00)	/* Receive FIFO Threshold (mask) */
#define SSCR1_RxThresh(x) (((x) - 1) << 10)	/* level [1..4] */
#define SSCR1_EFWR	(1 << 14)	/* Enable FIFO Write/Read */
#define SSCR1_STRF	(1 << 15)	/* Select FIFO or EFWR */

#define SSSR_ALT_FRM	(1 << 0)	/* Alternate Frame (second device) */
#define SSSR_TNF	(1 << 2)	/* Transmit FIFO Not Full */
#define SSSR_RNE	(1 << 3)	/* Receive FIFO Not Empty */
#define SSSR_BSY	(1 << 4)	/* SSP Busy */
#define SSSR_TFS	(1 << 5)	/* Transmit FIFO Service Request */
#define SSSR_RFS	(1 << 6)	/* Receive FIFO Service Request */
#define SSSR_ROR	(1 << 7)	/* Receive FIFO Overrun */

#endif				/*VR_SPI_H_ */
