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

#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <asm/io.h>
#include <linux/device.h>
#include <asm-arm/arch/hardware.h>
#include <linux/spi/spi.h>
#include <linux/spi/davinci_spi.h>
#include <linux/spi/davinci_spi_bitbang.h>

#define FALSE 0
#define TRUE  1
#define ERROR -1

/*Board specific declarations*/
#define SPI_BUS_FREQ  			(4000000)
#define CS_DEFAULT 0xFF
#define SCS0_SELECT	0x01
#define SCS1_SELECT	0x02
#define SCS2_SELECT	0x04
#define SCS3_SELECT	0x08
#define SCS4_SELECT	0x10
#define SCS5_SELECT	0x20
#define SCS6_SELECT	0x40
#define SCS7_SELECT	0x80

/* Standard values for DAVINCI */
#define DAVINCI_SPI_MAX_CHIPSELECT 7

/* #define SPI_INTERRUPT_MODE 1 */
#define SPI_SPIFMT_PHASE_MASK        (0x00010000u)
#define SPI_SPIFMT_PHASE_SHIFT       (0x00000010u)
#define SPI_SPIFMT_PHASE_RESETVAL    (0x00000000u)

#define SPI_SPIFMT_POLARITY_MASK     (0x00020000u)
#define SPI_SPIFMT_POLARITY_SHIFT    (0x00000011u)
#define SPI_SPIFMT_POLARITY_RESETVAL (0x00000000u)

#define SPI_SPIFMT_SHIFTDIR_MASK     (0x00100000u)
#define SPI_SPIFMT_SHIFTDIR_SHIFT    (0x00000014u)
#define SPI_SPIFMT_SHIFTDIR_RESETVAL (0x00000000u)

/* SPIGCR1 */

#define SPI_SPIGCR1_SPIENA_MASK      (0x01000000u)
#define SPI_SPIGCR1_SPIENA_SHIFT     (0x00000018u)
#define SPI_SPIGCR1_SPIENA_RESETVAL  (0x00000000u)

#define SPI_INTLVL_1				 (0x000001FFu)
#define SPI_INTLVL_0				 (0x00000000u)

/* SPIPC0 */

#define SPI_SPIPC0_DIFUN_MASK        (0x00000800u)
#define SPI_SPIPC0_DIFUN_SHIFT       (0x0000000Bu)
#define SPI_SPIPC0_DIFUN_RESETVAL    (0x00000000u)

/*----DIFUN Tokens----*/
#define SPI_SPIPC0_DIFUN_DI          (0x00000001u)

#define SPI_SPIPC0_DOFUN_MASK        (0x00000400u)
#define SPI_SPIPC0_DOFUN_SHIFT       (0x0000000Au)
#define SPI_SPIPC0_DOFUN_RESETVAL    (0x00000000u)

/*----DOFUN Tokens----*/
#define SPI_SPIPC0_DOFUN_DO          (0x00000001u)

#define SPI_SPIPC0_CLKFUN_MASK       (0x00000200u)
#define SPI_SPIPC0_CLKFUN_SHIFT      (0x00000009u)
#define SPI_SPIPC0_CLKFUN_RESETVAL   (0x00000000u)

/*----CLKFUN Tokens----*/
#define SPI_SPIPC0_CLKFUN_CLK        (0x00000001u)

#define SPI_SPIPC0_EN1FUN_MASK       (0x00000002u)
#define SPI_SPIPC0_EN1FUN_SHIFT      (0x00000001u)
#define SPI_SPIPC0_EN1FUN_RESETVAL   (0x00000000u)

/*----EN1FUN Tokens----*/
#define SPI_SPIPC0_EN1FUN_EN1        (0x00000001u)

#define SPI_SPIPC0_EN0FUN_MASK       (0x00000001u)
#define SPI_SPIPC0_EN0FUN_SHIFT      (0x00000000u)
#define SPI_SPIPC0_EN0FUN_RESETVAL   (0x00000000u)

/*----EN0FUN Tokens----*/
#define SPI_SPIPC0_EN0FUN_EN0        (0x00000001u)

#define SPI_SPIPC0_RESETVAL          (0x00000000u)
#define SPI_SPIPC0_SPIENA		 (0x00000001u)
#define SPI_SPIPC0_SPIENA_SHIFT	 (0x00000008u)

#define SPI_SPIINT_MASKALL	         (0x000001FF)

/* SPIDAT1 */

#define SPI_SPIDAT1_CSHOLD_MASK      (0x10000000u)
#define SPI_SPIDAT1_CSHOLD_SHIFT     (0x0000001Cu)
#define SPI_SPIDAT1_CSHOLD_RESETVAL  (0x00000000u)

#define SPI_SPIDAT1_CSNR_MASK        (0x00030000u)
#define SPI_SPIDAT1_CSNR_SHIFT       (0x00000010u)
#define SPI_SPIDAT1_CSNR_RESETVAL    (0x00000000u)

#define SPI_SPIDAT1_DFSEL_MASK       (0x03000000u)
#define SPI_SPIDAT1_DFSEL_SHIFT      (0x00000018u)
#define SPI_SPIDAT1_DFSEL_RESETVAL   (0x00000000u)

#define SPI_SPIFMT_CHARLEN_MASK      (0x0000001Fu)
#define SPI_SPIFMT_CHARLEN_SHIFT     (0x00000000u)
#define SPI_SPIFMT_CHARLEN_RESETVAL  (0x00000000u)

#define SPI_SPIGCR1_CLKMOD_MASK      (0x00000002u)
#define SPI_SPIGCR1_CLKMOD_SHIFT     (0x00000001u)
#define SPI_SPIGCR1_CLKMOD_RESETVAL  (0x00000000u)

#define SPI_SPIGCR1_MASTER_MASK      (0x00000001u)
#define SPI_SPIGCR1_MASTER_SHIFT     (0x00000000u)
#define SPI_SPIGCR1_MASTER_RESETVAL  (0x00000000u)

#define SPI_SPIGCR1_LOOPBACK_MASK    (0x00010000u)
#define SPI_SPIGCR1_LOOPBACK_SHIFT   (0x00000010u)
#define SPI_SPIGCR1_LOOPBACK_RESETVAL (0x00000000u)

#define SPI_SPIBUF_TXFULL_MASK       (0x20000000u)
#define SPI_SPIBUF_TXFULL_SHIFT      (0x0000001Du)
#define SPI_SPIBUF_TXFULL_RESETVAL   (0x00000000u)

/* SPIBUF */

#define SPI_SPIBUF_RXEMPTY_MASK      (0x80000000u)
#define SPI_SPIBUF_RXEMPTY_SHIFT     (0x0000001Fu)
#define SPI_SPIBUF_RXEMPTY_RESETVAL  (0x00000001u)

#define SPI_SPIFLG_BITERRFLG_MASK    (0x00000010u)
#define SPI_SPIFLG_BITERRFLG_SHIFT   (0x00000004u)
#define SPI_SPIFLG_BITERRFLG_RESETVAL (0x00000000u)

#define SPI_SPIFLG_OVRNINTFLG_MASK   (0x00000040u)
#define SPI_SPIFLG_OVRNINTFLG_SHIFT  (0x00000006u)
#define SPI_SPIFLG_OVRNINTFLG_RESETVAL (0x00000000u)

#define SPI_SPIFLG_RXINTFLAG_MASK    (0x00000100u)
#define SPI_SPIFLG_RXINTFLAG_SHIFT   (0x00000008u)
#define SPI_SPIFLG_RXINTFLAG_RESETVAL (0x00000000u)

#define SPI_SPIINT_DMA_REQ_EN	         (0x00010000u)
#define SPI_SPIINT_RX_INTR	         (0x00000100u)
#define SPI_SPIINT_TIMEOUT_INTR 	 (0x00000002u)
#define SPI_SPIINT_PARERR_INTR	 (0x00000004u)
#define SPI_SPIINT_DESYNC_INTR	 (0x00000008u)
#define SPI_SPIINT_BITERR_INTR	 (0x00000010u)
#define SPI_SPIINT_OVRRUN_INTR	 (0x00000040u)

/**< Error return coded */
#define SPI_ERROR_BASE 			(-30)
#define SPI_RECEIVE_OVERRUN_ERR 	(SPI_ERROR_BASE-1)
#define SPI_BIT_ERR		(SPI_ERROR_BASE-2)
#define SPI_DESYNC_ERR         	(SPI_ERROR_BASE-3)
#define SPI_PARITY_ERR		(SPI_ERROR_BASE-4)
#define SPI_TIMEOUT_ERR		(SPI_ERROR_BASE-5)
#define SPI_TRANSMIT_FULL_ERR	(SPI_ERROR_BASE-6)
#define SPI_POWERDOWN		(SPI_ERROR_BASE-7)

#define SPI_BYTELENGTH 8u

/******************************************************************/

enum spiPinOpMode {
	SPI_OPMODE_3PIN = 0,
	/**< SPI master 3 pin mode */
	SPI_OPMODE_SPISCS_4PIN = 1,
	/**< SPI master 4 pin mode uses SPISCS */
	SPI_OPMODE_SPIENA_4PIN = 2,
	/**< SPI master 4 pin mode uses SPIENA */
	SPI_OPMODE_5PIN = 3
    /**< SPI master 5 pin mode */
};

struct davinci_spi_config_t {
	u32 op_mode;
	u32 moduleInputClkFreq;
	u32 spiBusFreq;
	u32 clkInternal;
	u32 csHold;
	u32 delay;
	u32 intrLevel;
	enum spiPinOpMode pinOpModes;
	u32 clkHigh;
	u32 lsbFirst;
	u32 oddParity;
	u32 parityEnable;
	u32 phaseIn;
	u32 loopBack;
};

/* SPI Controller registers */

struct davinci_spi_reg {
	volatile __u32 __bitwise SPIGCR0;
	volatile __u32 __bitwise SPIGCR1;
	volatile __u32 __bitwise SPIINT;
	volatile __u32 __bitwise SPILVL;
	volatile __u32 __bitwise SPIFLG;
	volatile __u32 __bitwise SPIPC0;
	volatile __u32 __bitwise SPIPC1;
	volatile __u32 __bitwise SPIPC2;
	volatile __u32 __bitwise SPIPC3;
	volatile __u32 __bitwise SPIPC4;
	volatile __u32 __bitwise SPIPC5;
	volatile __u32 __bitwise SPIPC6;
	volatile __u32 __bitwise SPIPC7;
	volatile __u32 __bitwise SPIPC8;
	volatile __u32 __bitwise SPIDAT0;
	volatile __u32 __bitwise SPIDAT1;
	volatile __u32 __bitwise SPIBUF;
	volatile __u32 __bitwise SPIEMU;
	volatile __u32 __bitwise SPIDELAY;
	volatile __u32 __bitwise SPIDEF;
	volatile __u32 __bitwise SPIFMT[4];
	volatile __u32 __bitwise TGINTVEC[2];
	volatile __u8 __bitwise RSVD0[8];
	volatile __u32 __bitwise MIBSPIE;
};

struct davinci_spi_slave {
	u32 cmd_to_write;
	u32 clk_ctrl_to_write;
	u32 bytes_per_word;
	u8 active_cs;
};

#define SPI_BUFSIZ      (SMP_CACHE_BYTES + 1)

/* We have 2 DMA channels per CS, one for RX and one for TX */
struct davinci_spi_dma {
	int dma_tx_channel;
	int dma_rx_channel;

	int dma_tx_sync_dev;
	int dma_rx_sync_dev;

	struct completion dma_tx_completion;
	struct completion dma_rx_completion;
};

/* SPI Controller driver's private data. */
struct davinci_spi {
	/* bitbang has to be first */
	struct spi_bitbang bitbang;
	struct completion done;

	struct davinci_spi_reg __iomem *base;

	/* rx & tx bufs from the spi_transfer */
	const void *tx;
	void *rx;

	u8 *tmp_buf;

	/* functions to deal with different sized buffers */
	void (*get_rx) (u32 rx_data, struct davinci_spi *);
	 u32(*get_tx) (struct davinci_spi *);

	int count;
	u32 irq;

	u32 nsecs;		/* (clock cycle time)/2 */
	u32 sysclk;
	u32 vbus_freq;

	/* chip select activation deactivation */
	int (*activate_cs) (u8 cs, u8 polarity,
			    struct ctlr_cs_sel_t *ctlr_cs_sel);
	int (*deactivate_cs) (u8 cs, u8 polarity,
			      struct ctlr_cs_sel_t *ctlr_cs_sel);

	struct davinci_spi_slave slave[DAVINCI_SPI_MAX_CHIPSELECT];
	struct spi_device *spi;

	struct davinci_spi_dma *dma_channels;
};
