/*
 * controller driver with Interrupt.
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

#include <asm/io.h>
#include <linux/spi/davinci_spi_master.h>
#include <linux/dma-mapping.h>
#include <asm/arch/edma.h>
#include <asm/arch/cpu.h>
#include <asm/arch/mux.h>

#if CONFIG_SPI_DAVINCI_DMA
static unsigned use_dma = 1;
#else
static unsigned use_dma;
#endif
module_param(use_dma, uint, 0644);

/* #define CONFIG_SPI_DEBUG */

#if defined(CONFIG_SPI_DEBUG)
#define DEBUG_SPI(fmt, arg...)  printk(KERN_EMERG fmt , ##arg);
#else
#define DEBUG_SPI(fmt, arg...)
#endif

#define ENTER DEBUG_SPI("[Ei %s-%d] \n", __FUNCTION__, __LINE__);
#define EXIT  DEBUG_SPI("[Ex %s-%d] \n", __FUNCTION__, __LINE__);

/* operating momde selection from kconfig */
struct davinci_spi_config_t davinci_spi_config;

#define davinci_SPI_RX_BUF(type) 	static inline void		  \
davinci_spi_rx_buf_##type(u32 data, struct davinci_spi *davinci_spi)	  \
{									  \
	type * rx = davinci_spi->rx;					  \
	*rx++ = (type)data;						  \
	davinci_spi->rx = rx;						  \
}

#define davinci_SPI_TX_BUF(type)				\
static inline u32 davinci_spi_tx_buf_##type(struct davinci_spi *davinci_spi) \
{								\
	u32 data;						\
	const type * tx = davinci_spi->tx;			\
	data = *tx++;						\
	davinci_spi->tx = tx;					\
	return data;						\
}

davinci_SPI_RX_BUF(u8);
davinci_SPI_RX_BUF(u16);
davinci_SPI_TX_BUF(u8);
davinci_SPI_TX_BUF(u16);

#define SP0_MUX_SEL_LINE \
{ \
davinci_cfg_reg(DM355_SPI0_SDI); \
davinci_cfg_reg(DM355_SPI0_SDENA0); \
davinci_cfg_reg(DM355_SPI0_SDENA1); \
}

#define SP1_MUX_SEL_LINE \
{ \
davinci_cfg_reg(DM355_SPI1_SCLK); \
davinci_cfg_reg(DM355_SPI1_SDO); \
davinci_cfg_reg(DM355_SPI1_SDENA0); \
davinci_cfg_reg(DM355_SPI1_SDENA1); \
}

#define SP2_MUX_SEL_LINE \
{ \
davinci_cfg_reg(DM355_SPI2_SCLK); \
davinci_cfg_reg(DM355_SPI2_SDO); \
davinci_cfg_reg(DM355_SPI2_SDENA0); \
davinci_cfg_reg(DM355_SPI2_SDENA1); \
}

#define DM355_CLEAR_RX_BUFFER(base) do { 				\
		if (base->SPIBUF & SPI_SPIBUF_RXEMPTY_MASK)		\
			break;						\
		} while (1)

extern void davinci_clean_channel(int);

static void davinci_spi_set_dma_req(const struct spi_device *spi, int enable)
{
	struct davinci_spi *davinci_spi = spi_master_get_devdata(spi->master);
	u32 l;

	l = davinci_spi->base->SPIINT;

	if (enable)
		l |= SPI_SPIINT_DMA_REQ_EN;
	else
		l &= ~SPI_SPIINT_DMA_REQ_EN;

	davinci_spi->base->SPIINT = l;
}

/*
 * Interface to control the chip select signal
 */
static void davinci_spi_chipselect(struct spi_device *spi, int value)
{
	struct davinci_spi *davinci_spi;
	u32 data1_reg_val = 0;
	ENTER
	davinci_spi = spi_master_get_devdata(spi->master);
	    /* board specific chip select logic decides the polarity and cs */
	    /* line for the controller */
	    if (value == BITBANG_CS_INACTIVE) {
		davinci_spi->base->SPIDEF |= CS_DEFAULT;

		data1_reg_val |= CS_DEFAULT << SPI_SPIDAT1_CSNR_SHIFT;
		davinci_spi->base->SPIDAT1 = data1_reg_val;

		DM355_CLEAR_RX_BUFFER(davinci_spi->base);
	}
	EXIT
	return;
}

/**
 * davinci_spi_setup_transfer - This functions will determine transfer method
 * @spi: spi device on which data transfer to be done
 * @t: spi transfer in which transfer info is filled
 *
 * This function determines data transfer method (8/16/32 bit transfer).
 * It will also set the SPI Clock Control register according to
 * SPI slave device freq.
 */
static int davinci_spi_setup_transfer(struct spi_device *spi,
				      struct spi_transfer *t)
{

	struct davinci_spi *davinci_spi;
	u8 bits_per_word = 0;
	u32 hz = 0;
	int i = 0;

	ENTER
	davinci_spi = spi_master_get_devdata(spi->master);

	spi->controller_data = NULL;
	spi->controller_data = &davinci_spi_config;

	if (t) {
		bits_per_word = t->bits_per_word;
		hz = t->speed_hz;
	}

	/* if bits_per_word is not set then set it default */
	if (!bits_per_word)
		bits_per_word = spi->bits_per_word;

	/* Assign function pointer to appropriate transfer method */
	/* 8bit/16bit or 32bit transfer */
	if (bits_per_word <= 8 && bits_per_word >= 2) {
		DEBUG_SPI("BITS PER WORD = %d\n", bits_per_word);
		davinci_spi->get_rx = davinci_spi_rx_buf_u8;
		davinci_spi->get_tx = davinci_spi_tx_buf_u8;
		davinci_spi->slave[spi->chip_select].bytes_per_word = 1;
	} else if (bits_per_word <= 16 && bits_per_word >= 2) {
		DEBUG_SPI("BITS PER WORD = %d\n", bits_per_word);
		davinci_spi->get_rx = davinci_spi_rx_buf_u16;
		davinci_spi->get_tx = davinci_spi_tx_buf_u16;
		davinci_spi->slave[spi->chip_select].bytes_per_word = 2;
	} else
		return ERROR;

	if (!hz) {
		hz = spi->max_speed_hz;
		if (!hz) {
			hz = 2000000;	/* defaulting to 2Mhz */
			printk (KERN_INFO "[SPI] -> Slave device speed not set "
			    "correctly. Trying with %dHz\n", hz);
		}
	}

/**************************************************************************/

	for (i = 0; i < 4; i++) {
		davinci_spi->base->SPIFMT[i] &= ~(SPI_SPIFMT_CHARLEN_MASK);
		davinci_spi->base->SPIFMT[i] |= bits_per_word;
	}

	EXIT
	return 0;
}

static void davinci_spi_dma_rx_callback(int lch, u16 ch_status, void *data)
{
	struct spi_device *spi = (struct spi_device *)data;
	struct davinci_spi *davinci_spi;
	struct davinci_spi_dma *davinci_spi_dma;

	davinci_spi = spi_master_get_devdata(spi->master);
	davinci_spi_dma = &(davinci_spi->dma_channels[spi->chip_select]);

	if (ch_status == DMA_COMPLETE)
		davinci_stop_dma(davinci_spi_dma->dma_rx_channel);
	else
		davinci_clean_channel(davinci_spi_dma->dma_rx_channel);

	complete(&davinci_spi_dma->dma_rx_completion);
	/* We must disable the DMA RX request */
	davinci_spi_set_dma_req(spi, 0);

	DEBUG_SPI("RX DMA COMPLETE ch_status=0x%x\n", ch_status);
}

static void davinci_spi_dma_tx_callback(int lch, u16 ch_status, void *data)
{
	struct spi_device *spi = (struct spi_device *)data;
	struct davinci_spi *davinci_spi;
	struct davinci_spi_dma *davinci_spi_dma;

	davinci_spi = spi_master_get_devdata(spi->master);
	davinci_spi_dma = &(davinci_spi->dma_channels[spi->chip_select]);

	if (ch_status == DMA_COMPLETE)
		davinci_stop_dma(davinci_spi_dma->dma_tx_channel);
	else
		davinci_clean_channel(davinci_spi_dma->dma_tx_channel);

	complete(&davinci_spi_dma->dma_tx_completion);
	/* We must disable the DMA TX request */
	davinci_spi_set_dma_req(spi, 0);

	DEBUG_SPI("TX DMA COMPLETE ch_status=0x%x\n", ch_status);
}

static int davinci_spi_request_dma(struct spi_device *spi)
{
	struct davinci_spi *davinci_spi;
	struct davinci_spi_dma *davinci_spi_dma;
	int tcc;

	davinci_spi = spi_master_get_devdata(spi->master);
	davinci_spi_dma = davinci_spi->dma_channels + spi->chip_select;

	if (davinci_request_dma(davinci_spi_dma->dma_rx_sync_dev, "MibSPI RX",
				davinci_spi_dma_rx_callback, spi,
				&davinci_spi_dma->dma_rx_channel,
				&tcc, EVENTQ_3)) {
		printk(KERN_ERR
		       "Unable to request DMA channel for MibSPI RX\n");
		return -EAGAIN;
	}
	if (davinci_request_dma(davinci_spi_dma->dma_tx_sync_dev, "MibSPI TX",
				davinci_spi_dma_tx_callback, spi,
				&davinci_spi_dma->dma_tx_channel,
				&tcc, EVENTQ_3)) {
		davinci_free_dma(davinci_spi_dma->dma_rx_channel);
		davinci_spi_dma->dma_rx_channel = -1;
		printk(KERN_ERR
		       "Unable to request DMA channel for MibSPI TX\n");
		return -EAGAIN;
	}

	return 0;
}

/**
 * davinci_spi_setup - This functions will set default transfer method
 * @spi: spi device on which data transfer to be done
 *
 * This functions sets the default transfer method.
 */

static int davinci_spi_setup(struct spi_device *spi)
{
	int retval;
	struct davinci_spi *davinci_spi;
	struct davinci_spi_dma *davinci_spi_dma;

	ENTER
	davinci_spi = spi_master_get_devdata(spi->master);
	davinci_spi_dma = &davinci_spi->dma_channels[spi->chip_select];

	/*if bits per word length is zero then set it default 8 */
	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	davinci_spi->slave[spi->chip_select].cmd_to_write = 0;

	if (davinci_spi_dma->dma_rx_channel == -1 ||
	    davinci_spi_dma->dma_tx_channel == -1) {
		retval = davinci_spi_request_dma(spi);
		if (retval < 0)
			return retval;
	}

	retval = davinci_spi_setup_transfer(spi, NULL);

	EXIT
	return retval;
}

static void davinci_spi_cleanup(const struct spi_device *spi)
{
	struct davinci_spi *davinci_spi = spi_master_get_devdata(spi->master);
	struct davinci_spi_dma *davinci_spi_dma;

	davinci_spi_dma = &davinci_spi->dma_channels[spi->chip_select];

	if (davinci_spi_dma->dma_rx_channel != -1 &&
	    davinci_spi_dma->dma_tx_channel != -1) {
		davinci_free_dma(davinci_spi_dma->dma_tx_channel);
		davinci_free_dma(davinci_spi_dma->dma_rx_channel);
	}
}

/**
 * davinci_spi_bufs - functions which will handle transfer data
 * @spi: spi device on which data transfer to be done
 * @t: spi transfer in which transfer info is filled
 *
 * This function will put data to be transferred into data register
 * of SPI controller and then wait untill the completion will be marked
 * by the IRQ Handler.
 */
#define BIT_CLEAR_TIME_OUT	10000

/* #define SPI_PROFILE 1 */

#ifdef SPI_PROFILE

static unsigned int spi_prof_start;
static unsigned int spi_prof_end;
static unsigned int spi_avg_latency;

#define rdtscl(dest) \
	    __asm__ __volatile__("mfc0 %0,$9; nop" : "=r" (dest))

static void calculate_spi_latency(void)
{
	unsigned int diff = spi_prof_end - spi_prof_start;

	if (diff < 0) {
		printk(KERN_INFO "Count register wraparound not supported.\n");
	} else {
		if (spi_avg_latency == 0) {
			spi_avg_latency = diff;
		} else {
			spi_avg_latency += diff;
			spi_avg_latency /= 2;
		}
	}
}

void spi_start_profile(void)
{
	rdtscl(spi_prof_start);
}

void spi_end_profile(void)
{
	rdtscl(spi_prof_end);
	calculate_spi_latency();
	DEBUG_SPI("[SPI-PROFILE] Throughput = %d\n", spi_avg_latency);
}

unsigned int spi_get_avg_latency()
{
	return spi_avg_latency;
}
#endif				/* SPI_PROFILE */

static int davinci_spi_bufs_pio(struct spi_device *spi, struct spi_transfer *t)
{
	struct davinci_spi *davinci_spi;
	int i, intStatus = 0;
	int count;
	u8 conv = 1;
	u8 tmp;
	u32 tx_data = 0;
	u32 data1_reg_val = 0;

	struct davinci_spi_config_t *spi_cfg;
	u32 sPIPC0 = 0;
	u32 buf_val, flg_val;

	ENTER
	davinci_spi = spi_master_get_devdata(spi->master);

	davinci_spi->tx = t->tx_buf;
	davinci_spi->rx = t->rx_buf;

	/* DEBUG_SPI("tx_buf value = %x\n",*(u16 *)davinci_spi->tx); */

	/* convert len to words bbased on bits_per_word */
	conv = davinci_spi->slave[spi->chip_select].bytes_per_word;

	davinci_spi->count = t->len / conv;

	DEBUG_SPI("chipselect=%d, bytes_per_word=%d, t->len=%d, conv=%d\n",
		  spi->chip_select,
		  davinci_spi->slave[spi->chip_select].bytes_per_word, t->len,
		  conv);

	INIT_COMPLETION(davinci_spi->done);

	/*configuraton parameter for SPI */

	spi_cfg = (struct davinci_spi_config_t *) spi->controller_data;

	for (i = 0; i < 4; i++) {

		if (spi_cfg->phaseIn)
			davinci_spi->base->SPIFMT[i] |= SPI_SPIFMT_PHASE_MASK;
		else
			davinci_spi->base->SPIFMT[i] &=
			    ~(SPI_SPIFMT_PHASE_MASK);

		if (spi_cfg->clkHigh)
			davinci_spi->base->SPIFMT[i] |=
			    SPI_SPIFMT_POLARITY_MASK;
		else
			davinci_spi->base->SPIFMT[i] &=
			    ~(SPI_SPIFMT_POLARITY_MASK);

		if (spi_cfg->lsbFirst)
			davinci_spi->base->SPIFMT[i] |=
			    SPI_SPIFMT_SHIFTDIR_MASK;
		else
			davinci_spi->base->SPIFMT[i] &=
			    ~(SPI_SPIFMT_SHIFTDIR_MASK);

	}

	/*Enable SPI */
	davinci_spi->base->SPIGCR1 |= SPI_SPIGCR1_SPIENA_MASK;

	/*Clock internal */
	if (spi_cfg->clkInternal)
		davinci_spi->base->SPIGCR1 |= SPI_SPIGCR1_CLKMOD_MASK;
	else
		davinci_spi->base->SPIGCR1 &= ~(SPI_SPIGCR1_CLKMOD_MASK);

	/* master mode default */
	davinci_spi->base->SPIGCR1 |= SPI_SPIGCR1_MASTER_MASK;

	if (spi_cfg->intrLevel)
		davinci_spi->base->SPILVL = SPI_INTLVL_1;
	else
		davinci_spi->base->SPILVL = SPI_INTLVL_0;

	switch (spi_cfg->pinOpModes) {

	case SPI_OPMODE_3PIN:
		sPIPC0 |= (SPI_SPIPC0_DIFUN_DI << SPI_SPIPC0_DIFUN_SHIFT)
		    | (SPI_SPIPC0_DOFUN_DO << SPI_SPIPC0_DOFUN_SHIFT)
		    | (SPI_SPIPC0_CLKFUN_CLK << SPI_SPIPC0_CLKFUN_SHIFT);

		davinci_spi->base->SPIPC0 = sPIPC0;

		break;

	case SPI_OPMODE_SPISCS_4PIN:
		sPIPC0 |= (SPI_SPIPC0_DIFUN_DI << SPI_SPIPC0_DIFUN_SHIFT)
		    | (SPI_SPIPC0_DOFUN_DO << SPI_SPIPC0_DOFUN_SHIFT)
		    | (SPI_SPIPC0_CLKFUN_CLK << SPI_SPIPC0_CLKFUN_SHIFT)
		    | (SPI_SPIPC0_EN1FUN_EN1 << SPI_SPIPC0_EN1FUN_SHIFT)
		    | (SPI_SPIPC0_EN0FUN_EN0 << SPI_SPIPC0_EN0FUN_SHIFT);

		davinci_spi->base->SPIPC0 = sPIPC0;

		break;

	case SPI_OPMODE_SPIENA_4PIN:
		sPIPC0 |= (SPI_SPIPC0_DIFUN_DI << SPI_SPIPC0_DIFUN_SHIFT)
		    | (SPI_SPIPC0_DOFUN_DO << SPI_SPIPC0_DOFUN_SHIFT)
		    | (SPI_SPIPC0_CLKFUN_CLK << SPI_SPIPC0_CLKFUN_SHIFT)
		    | (SPI_SPIPC0_SPIENA << SPI_SPIPC0_SPIENA_SHIFT);

		davinci_spi->base->SPIPC0 = sPIPC0;
		break;

	default:
		return -1;
	}

	if (spi_cfg->loopBack)
		davinci_spi->base->SPIGCR1 |= SPI_SPIGCR1_LOOPBACK_MASK;
	else
		davinci_spi->base->SPIGCR1 &= ~(SPI_SPIGCR1_LOOPBACK_MASK);

	/*Put delay val if required */
	davinci_spi->base->SPIDELAY = 0 | (8 << 24) | (8 << 16);

	count = davinci_spi->count;
	data1_reg_val |= spi_cfg->csHold << SPI_SPIDAT1_CSHOLD_SHIFT;

	/*CD default = 0xFF */
	tmp = ~(0x1 << spi->chip_select);
	davinci_spi->base->SPIDEF &= tmp;
	data1_reg_val |= tmp << SPI_SPIDAT1_CSNR_SHIFT;

	DM355_CLEAR_RX_BUFFER(davinci_spi->base);
	/* Determine the command to execute READ or WRITE */
	if (t->tx_buf) {
		davinci_spi->base->SPIINT &= ~(SPI_SPIINT_MASKALL);

		i = 0;
		while (1) {
			tx_data = davinci_spi->get_tx(davinci_spi);

			data1_reg_val &= ~(0xFFFF);
			data1_reg_val |= (0xFFFF & tx_data);

			DEBUG_SPI("data to be TX = 0x%x\n", tx_data);
			DEBUG_SPI("\nwriting into DATA1 register val = 0x%x\n",
				  data1_reg_val);

			buf_val = davinci_spi->base->SPIBUF;
			if ((buf_val & SPI_SPIBUF_TXFULL_MASK) == FALSE) {
				davinci_spi->base->SPIDAT1 = data1_reg_val;
				i++;
				DEBUG_SPI("\n***I have transmited...on "
					  "iteration %d and buf_val = "
					  "0x%8x***\n", i, buf_val);

				count--;
				if (count <= 0)
					break;
			}
			while (davinci_spi->base->
			       SPIBUF & SPI_SPIBUF_RXEMPTY_MASK) ;
		}
	} else {
		DEBUG_SPI("In receive\n");

#ifdef CONFIG_SPI_INTERRUPT
		spi_cfg->op_mode = 1;
#else
		spi_cfg->op_mode = 0;
#endif

		if (!spi_cfg->op_mode) {	/*In Polling mode receive */

			i = 0;
			while (1) {
				i++;

				/* keeps the serial clock going */
				if ((davinci_spi->base->SPIBUF &
				     SPI_SPIBUF_TXFULL_MASK) == FALSE)
					davinci_spi->base->SPIDAT1 =
					    data1_reg_val;

				while (davinci_spi->base->SPIBUF &
				       SPI_SPIBUF_RXEMPTY_MASK) ;

				flg_val = davinci_spi->base->SPIFLG;
				buf_val = davinci_spi->base->SPIBUF;

				DEBUG_SPI("flg_val = 0x%8x & buf_val = "
					  "0x%8x\n", flg_val, buf_val);

				davinci_spi->get_rx(buf_val, davinci_spi);

				DEBUG_SPI("I have got it... after "
					  "%d iteration buf_val = "
					  "0x%8x\nflg_val =  0x%8x\n",
					  i, buf_val, flg_val);

				count--;
				if (count <= 0)
					break;
			}
		} else {	/*Receive in Interrupt mode */

			for (i = 0; i <= davinci_spi->count; i++) {
				if (i == davinci_spi->count)
					davinci_spi->base->SPIDAT1 =
					    (data1_reg_val & 0x0ffcffff);
				else {
					davinci_spi->base->SPIINT |=
					    SPI_SPIFLG_BITERRFLG_MASK |
					    SPI_SPIFLG_OVRNINTFLG_MASK |
					    SPI_SPIFLG_RXINTFLAG_MASK;

					davinci_spi->base->SPIDAT1 =
					    (data1_reg_val);

					while ((flg_val =
						davinci_spi->base->
						SPIINT) &
					       SPI_SPIFLG_RXINTFLAG_MASK) ;
				}
			}
		}
	}

	/* Check for bit error, desync error,parity error,timeout error and
	   receive overflow errors */
	intStatus = davinci_spi->base->SPIFLG;

	if ((intStatus & SPI_SPIINT_TIMEOUT_INTR) == SPI_SPIINT_TIMEOUT_INTR) {
		printk(KERN_INFO "SPI Time-out Error\n");
		return SPI_TIMEOUT_ERR;
	}

	/* De-Synchronization error, holds only in master mode */
	else if ((intStatus&SPI_SPIINT_DESYNC_INTR) == SPI_SPIINT_DESYNC_INTR) {
		printk(KERN_INFO "SPI Desynchronization Error\n");
		return SPI_DESYNC_ERR;
	}

	/* Bit error error */
	else if ((intStatus&SPI_SPIINT_BITERR_INTR) == SPI_SPIINT_BITERR_INTR) {
		printk(KERN_INFO "SPI Bit error\n");
		return SPI_BIT_ERR;
	}

	/* SPI Framework maintains the count only in bytes so convert back */
	davinci_spi->count *= conv;

	EXIT
	return t->len;
}

#define DAVINCI_DMA_DATA_TYPE_S8           0x01
#define DAVINCI_DMA_DATA_TYPE_S16          0x02
#define DAVINCI_DMA_DATA_TYPE_S32          0x04

static int davinci_spi_bufs_dma(struct spi_device *spi, struct spi_transfer *t)
{
	struct davinci_spi *davinci_spi;
	int i, intStatus = 0;
	int count;
	u8 conv = 1;
	u8 tmp;
	u32 data1_reg_val = 0;
	struct davinci_spi_dma *davinci_spi_dma;
	int word_len, data_type;
	unsigned long tx_reg, rx_reg;

	struct davinci_spi_config_t *spi_cfg;
	u32 sPIPC0 = 0;

	ENTER
	davinci_spi = spi_master_get_devdata(spi->master);

	davinci_spi_dma = &davinci_spi->dma_channels[spi->chip_select];
	tx_reg = DAVINCI_SPI_BASE + 0x3c;	/* davinci_spi->base->SPIDAT1 */
	rx_reg = DAVINCI_SPI_BASE + 0x40;	/* davinci_spi->base->SPIBUF */

	/* used for macro defs */
	davinci_spi->tx = t->tx_buf;
	davinci_spi->rx = t->rx_buf;

	/* convert len to words bbased on bits_per_word */
	conv = davinci_spi->slave[spi->chip_select].bytes_per_word;
	davinci_spi->count = t->len / conv;

	DEBUG_SPI("chipselect=%d, bytes_per_word=%d, t->len=%d, conv=%d\n",
		  spi->chip_select,
		  davinci_spi->slave[spi->chip_select].bytes_per_word, t->len,
		  conv);

	INIT_COMPLETION(davinci_spi->done);

	init_completion(&davinci_spi_dma->dma_rx_completion);
	init_completion(&davinci_spi_dma->dma_tx_completion);

	word_len = conv * 8;
	if (word_len <= 8)
		data_type = DAVINCI_DMA_DATA_TYPE_S8;
	else if (word_len <= 16)
		data_type = DAVINCI_DMA_DATA_TYPE_S16;
	else if (word_len <= 32)
		data_type = DAVINCI_DMA_DATA_TYPE_S32;
	else
		return -1;

	/*configuraton parameter for SPI */
	spi_cfg = (struct davinci_spi_config_t *) spi->controller_data;

	for (i = 0; i < 4; i++) {
		if (spi_cfg->phaseIn)
			davinci_spi->base->SPIFMT[i] |= SPI_SPIFMT_PHASE_MASK;
		else
			davinci_spi->base->SPIFMT[i] &=
			    ~(SPI_SPIFMT_PHASE_MASK);

		if (spi_cfg->clkHigh)
			davinci_spi->base->SPIFMT[i] |=
			    SPI_SPIFMT_POLARITY_MASK;
		else
			davinci_spi->base->SPIFMT[i] &=
			    ~(SPI_SPIFMT_POLARITY_MASK);

		if (spi_cfg->lsbFirst)
			davinci_spi->base->SPIFMT[i] |=
			    SPI_SPIFMT_SHIFTDIR_MASK;
		else
			davinci_spi->base->SPIFMT[i] &=
			    ~(SPI_SPIFMT_SHIFTDIR_MASK);
	}

	/*Clock internal */
	if (spi_cfg->clkInternal)
		davinci_spi->base->SPIGCR1 |= SPI_SPIGCR1_CLKMOD_MASK;
	else
		davinci_spi->base->SPIGCR1 &= ~(SPI_SPIGCR1_CLKMOD_MASK);

	/* master mode default */
	davinci_spi->base->SPIGCR1 |= SPI_SPIGCR1_MASTER_MASK;

	if (spi_cfg->intrLevel)
		davinci_spi->base->SPILVL = SPI_INTLVL_1;
	else
		davinci_spi->base->SPILVL = SPI_INTLVL_0;

	switch (spi_cfg->pinOpModes) {
	case SPI_OPMODE_3PIN:
		sPIPC0 |= (SPI_SPIPC0_DIFUN_DI << SPI_SPIPC0_DIFUN_SHIFT)
		    | (SPI_SPIPC0_DOFUN_DO << SPI_SPIPC0_DOFUN_SHIFT)
		    | (SPI_SPIPC0_CLKFUN_CLK << SPI_SPIPC0_CLKFUN_SHIFT);

		davinci_spi->base->SPIPC0 = sPIPC0;

		break;

	case SPI_OPMODE_SPISCS_4PIN:
		sPIPC0 |= (SPI_SPIPC0_DIFUN_DI << SPI_SPIPC0_DIFUN_SHIFT)
		    | (SPI_SPIPC0_DOFUN_DO << SPI_SPIPC0_DOFUN_SHIFT)
		    | (SPI_SPIPC0_CLKFUN_CLK << SPI_SPIPC0_CLKFUN_SHIFT)
		    | (SPI_SPIPC0_EN1FUN_EN1 << SPI_SPIPC0_EN1FUN_SHIFT)
		    | (SPI_SPIPC0_EN0FUN_EN0 << SPI_SPIPC0_EN0FUN_SHIFT);

		davinci_spi->base->SPIPC0 = sPIPC0;

		break;

	case SPI_OPMODE_SPIENA_4PIN:
		sPIPC0 |= (SPI_SPIPC0_DIFUN_DI << SPI_SPIPC0_DIFUN_SHIFT)
		    | (SPI_SPIPC0_DOFUN_DO << SPI_SPIPC0_DOFUN_SHIFT)
		    | (SPI_SPIPC0_CLKFUN_CLK << SPI_SPIPC0_CLKFUN_SHIFT)
		    | (SPI_SPIPC0_SPIENA << SPI_SPIPC0_SPIENA_SHIFT);

		davinci_spi->base->SPIPC0 = sPIPC0;
		break;

	default:
		return -1;
	}

	if (spi_cfg->loopBack)
		davinci_spi->base->SPIGCR1 |= SPI_SPIGCR1_LOOPBACK_MASK;
	else
		davinci_spi->base->SPIGCR1 &= ~(SPI_SPIGCR1_LOOPBACK_MASK);

	/*Put delay val if required */
/*        davinci_spi->base->SPIDELAY = 0 | (8 << 24) | (8 <<16); */
	davinci_spi->base->SPIDELAY = 0;

	count = davinci_spi->count;	/* the number of elements */
	data1_reg_val |= spi_cfg->csHold << SPI_SPIDAT1_CSHOLD_SHIFT;

	/*CD default = 0xFF */
	tmp = ~(0x1 << spi->chip_select);
	davinci_spi->base->SPIDEF &= tmp;
	data1_reg_val |= tmp << SPI_SPIDAT1_CSNR_SHIFT;

	/*disable all interrupts for dma transfers */
	davinci_spi->base->SPIINT &= ~(SPI_SPIINT_MASKALL);
	/* Disable SPI to write configuration bits in SPIDAT */
	davinci_spi->base->SPIGCR1 &= ~SPI_SPIGCR1_SPIENA_MASK;
	davinci_spi->base->SPIDAT1 = data1_reg_val;
	/*Enable SPI */
	davinci_spi->base->SPIGCR1 |= SPI_SPIGCR1_SPIENA_MASK;

	DM355_CLEAR_RX_BUFFER(davinci_spi->base);

	if (t->tx_buf != NULL) {
#ifdef CONFIG_SPI_DEBUG
		for (i = 0; i < count; i++)
			DEBUG_SPI("tx[%d]=0x%x\n", i,
				  *((u8 *) (t->tx_buf) + i));
#endif
		t->tx_dma = dma_map_single(&spi->dev, (void *)t->tx_buf, count,
					   DMA_TO_DEVICE);
		if (dma_mapping_error(t->tx_dma)) {
			printk(KERN_ERR
			       "%s(): Couldn't DMA map a %d bytes TX buffer\n",
			       __FUNCTION__, count);
			return -1;
		}
		davinci_set_dma_transfer_params(davinci_spi_dma->dma_tx_channel,
						data_type, count, 1, 0, ASYNC);
		davinci_set_dma_dest_params(davinci_spi_dma->dma_tx_channel,
					    tx_reg, INCR, W8BIT);
		davinci_set_dma_src_params(davinci_spi_dma->dma_tx_channel,
					   t->tx_dma, INCR, W8BIT);
		davinci_set_dma_src_index(davinci_spi_dma->dma_tx_channel,
					  data_type, 0);
		davinci_set_dma_dest_index(davinci_spi_dma->dma_tx_channel, 0,
					   0);
	} else {
		/* We need TX clocking for RX transaction */
		t->tx_dma =
		    dma_map_single(&spi->dev, (void *)davinci_spi->tmp_buf,
				   count + 1, DMA_TO_DEVICE);
		if (dma_mapping_error(t->tx_dma)) {
			printk(KERN_ERR
			       "%s(): Couldn't DMA map a %d bytes TX "
			       "tmp buffer\n", __FUNCTION__, count);
			return -1;
		}
		davinci_set_dma_transfer_params(davinci_spi_dma->dma_tx_channel,
						data_type, count + 1, 1, 0,
						ASYNC);
		davinci_set_dma_dest_params(davinci_spi_dma->dma_tx_channel,
					    tx_reg, INCR, W8BIT);
		davinci_set_dma_src_params(davinci_spi_dma->dma_tx_channel,
					   t->tx_dma, INCR, W8BIT);
		davinci_set_dma_src_index(davinci_spi_dma->dma_tx_channel,
					  data_type, 0);
		davinci_set_dma_dest_index(davinci_spi_dma->dma_tx_channel, 0,
					   0);
	}

	if (t->rx_buf != NULL) {
		/* initiate transaction */
		davinci_spi->base->SPIDAT1 = data1_reg_val;

		t->rx_dma = dma_map_single(&spi->dev, (void *)t->rx_buf, count,
					   DMA_FROM_DEVICE);
		if (dma_mapping_error(t->rx_dma)) {
			printk(KERN_ERR
			       "%s(): Couldn't DMA map a %d bytes RX buffer\n",
			       __FUNCTION__, count);
			if (t->tx_buf != NULL)
				dma_unmap_single(NULL, t->tx_dma,
						 count, DMA_TO_DEVICE);
			return -1;
		}
		davinci_set_dma_transfer_params(davinci_spi_dma->dma_rx_channel,
						data_type, count, 1, 0, ASYNC);
		davinci_set_dma_src_params(davinci_spi_dma->dma_rx_channel,
					   rx_reg, INCR, W8BIT);
		davinci_set_dma_dest_params(davinci_spi_dma->dma_rx_channel,
					    t->rx_dma, INCR, W8BIT);
		davinci_set_dma_src_index(davinci_spi_dma->dma_rx_channel, 0,
					  0);
		davinci_set_dma_dest_index(davinci_spi_dma->dma_rx_channel,
					   data_type, 0);
	}

	if ((t->tx_buf != NULL) || (t->rx_buf != NULL))
		davinci_start_dma(davinci_spi_dma->dma_tx_channel);

	if (t->rx_buf != NULL)
		davinci_start_dma(davinci_spi_dma->dma_rx_channel);

	if ((t->rx_buf != NULL) || (t->tx_buf != NULL))
		davinci_spi_set_dma_req(spi, 1);

	if (t->tx_buf != NULL) {
		DEBUG_SPI("WAIT TX DMA\n");
		wait_for_completion_interruptible(&davinci_spi_dma->
						  dma_tx_completion);
	}

	if (t->rx_buf != NULL) {
		DEBUG_SPI("WAIT RX DMA\n");
		wait_for_completion_interruptible(&davinci_spi_dma->
						  dma_rx_completion);
#ifdef CONFIG_SPI_DEBUG
		for (i = 0; i < count; i++)
			DEBUG_SPI("rx[%d]=0x%x\n", i, *(u8 *) (t->rx_buf + i));
#endif
	}

	if (t->tx_buf != NULL)
		dma_unmap_single(NULL, t->tx_dma, count, DMA_TO_DEVICE);
	else
		dma_unmap_single(NULL, t->tx_dma, count + 1, DMA_TO_DEVICE);

	if (t->rx_buf != NULL)
		dma_unmap_single(NULL, t->rx_dma, count, DMA_FROM_DEVICE);

	/* Check for bit error, desync error,parity error,timeout error and
	   receive overflow errors */
	intStatus = davinci_spi->base->SPIFLG;

	if ((intStatus & SPI_SPIINT_TIMEOUT_INTR) == SPI_SPIINT_TIMEOUT_INTR) {
		printk(KERN_INFO "SPI Time-out Error\n");
		return SPI_TIMEOUT_ERR;
	}

	/* De-Synchronization error, holds only in master mode */
	else if ((intStatus&SPI_SPIINT_DESYNC_INTR) == SPI_SPIINT_DESYNC_INTR) {
		printk(KERN_INFO "SPI Desynchronization Error\n");
		return SPI_DESYNC_ERR;
	}

	/* Bit error error */
	else if ((intStatus&SPI_SPIINT_BITERR_INTR) == SPI_SPIINT_BITERR_INTR) {
		printk(KERN_INFO "SPI Bit error\n");
		return SPI_BIT_ERR;
	}

	/* SPI Framework maintains the count only in bytes so convert back */
	davinci_spi->count *= conv;

	EXIT
	return t->len;
}

/**
 * davinci_spi_irq - probe function for SPI Master Controller
 * @irq: IRQ number for this SPI Master
 * @context_data: structure for SPI Master controller davinci_spi
 * @ptregs:
 *
 * ISR will determine that interrupt arrives either for READ or WRITE command.
 * According to command it will do the appropriate action. It will check
 * transfer length and if it is not zero then dispatch transfer command again.
 * If transfer length is zero then it will indicate the COMPLETION so that
 * davinci_spi_bufs function can go ahead.
 */
irqreturn_t davinci_spi_irq(s32 irq, void *context_data,
			    struct pt_regs *ptregs)
{
	struct davinci_spi *davinci_spi = context_data;
	u32 rx_data = 0;

	DEBUG_SPI("IN_IRQ\n");
	DEBUG_SPI("received interrupt = 0x%8x\n", davinci_spi->base->SPIFLG);

	while (davinci_spi->base->SPIFLG != 0) {
		/*Time out Error */
		if ((davinci_spi->base->SPIFLG & SPI_SPIINT_TIMEOUT_INTR) ==
		    SPI_SPIINT_TIMEOUT_INTR) {
			printk(KERN_INFO "SPI Time-out Error\n");
			return SPI_TIMEOUT_ERR;
		}

		/* De-Synchronization interrupt, holds only in master mode */
		else if ((davinci_spi->base->SPIFLG & SPI_SPIINT_DESYNC_INTR) ==
			 SPI_SPIINT_DESYNC_INTR) {
			printk(KERN_INFO "SPI Desynchronization Error\n");
			return SPI_DESYNC_ERR;
		}

		/* Bit error Interrupt */
		else if ((davinci_spi->base->SPIFLG & SPI_SPIINT_BITERR_INTR) ==
			 SPI_SPIINT_BITERR_INTR) {
			printk(KERN_INFO "SPI Bit error\n");
			return SPI_BIT_ERR;
		 /**/}

		/*Receive Interrupt */
		else if (davinci_spi->base->SPIFLG & SPI_SPIINT_RX_INTR) {

			rx_data = davinci_spi->base->SPIBUF;
			davinci_spi->get_rx(rx_data, davinci_spi);
			DEBUG_SPI("IRQ rx_data = 0x16%x\n", rx_data);

			/*Disable Receive Interrupt */
			davinci_spi->base->SPIINT = ~SPI_SPIINT_RX_INTR;
			return IRQ_HANDLED;
		}

	}
	return -1;

}

/**
 * davinci_spi_probe - probe function for SPI Master Controller
 * @dev: platform_device structure which contains plateform specific data
 *
 * According to Linux Deviced Model this function will be invoked by Linux
 * with plateform_device struct which contains the device specific info
 * like bus_num, max_chipselect (how many slave devices can be connected),
 * clock freq. of SPI controller, SPI controller's memory range, IRQ number etc.
 *
 * According to Linux Deviced Model this function will be invoked by Linux
 * with plateform_device struct which contains the device specific info
 * like bus_num, max_chipselect (how many slave devices can be connected),
 * clock freq. of SPI controller, SPI controller's memory range, IRQ number etc.
 * This info will be provided by board specific code which will reside in
 * linux-2.6.10/arch/mips/mips-boards/davinci_davinci/davinci_yamuna code.
 * This function will map the SPI controller's memory, register IRQ,
 * Reset SPI controller and setting its registers to default value.
 * It will invoke spi_bitbang_start to create work queue so that client driver
 * can register transfer method to work queue.
 */
static u8 __initdata spi0_rxdma_id[] = {
	DAVINCI_DMA_SPI_SPIR,
	DAVINCI_DMA_SPI_SPIR,
};

static u8 __initdata spi0_txdma_id[] = {
	DAVINCI_DMA_SPI_SPIX,
	DAVINCI_DMA_SPI_SPIX,
};

static u8 __initdata spi1_rxdma_id[] = {
	DAVINCI_DMA_SPI_SPIR,
	DAVINCI_DMA_SPI_SPIR,
};

static u8 __initdata spi1_txdma_id[] = {
	DAVINCI_DMA_SPI_SPIX,
	DAVINCI_DMA_SPI_SPIX,
};

static int davinci_spi_probe(struct device *d)
{
	struct platform_device *dev =
	    container_of(d, struct platform_device, dev);
	struct spi_master *master;
	struct davinci_spi *davinci_spi;
	struct davinci_spi_platform_data *pdata;
	struct resource *r;
	int i = 0, ret = 0, prescale = 0, clk_freq = 0;
	const u8 *rxdma_id, *txdma_id;

	switch (dev->id) {
	case 0:
		rxdma_id = spi0_rxdma_id;
		txdma_id = spi0_txdma_id;
		break;
	case 1:
		rxdma_id = spi1_rxdma_id;
		txdma_id = spi1_txdma_id;
		break;
	default:
		return -EINVAL;
	}

	/* Get resources(memory, IRQ) associated with the device */
	master = spi_alloc_master(&dev->dev, sizeof(struct davinci_spi));

	if (master == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	dev_set_drvdata(&(dev)->dev, (master));

	pdata = dev->dev.platform_data;

	if (pdata == NULL) {
		ret = -ENODEV;
		goto free_master;
	}

	r = platform_get_resource(dev, IORESOURCE_MEM, 0);

	if (r == NULL) {
		ret = -ENODEV;
		goto free_master;
	}

	davinci_spi = spi_master_get_devdata(master);
	davinci_spi->bitbang.master = spi_master_get(master);
	davinci_spi->bitbang.chipselect = davinci_spi_chipselect;
	davinci_spi->bitbang.setup_transfer = davinci_spi_setup_transfer;

	/* DM355 does not have DMA capabilities */
	if (cpu_is_davinci_dm355())
		use_dma = 0;

	if (use_dma) {
		davinci_spi->bitbang.txrx_bufs = davinci_spi_bufs_dma;
	} else {
		davinci_spi->bitbang.txrx_bufs = davinci_spi_bufs_pio;
	}
	davinci_spi->sysclk = pdata->sysclk;
	davinci_spi->activate_cs = pdata->activate_cs;
	davinci_spi->deactivate_cs = pdata->deactivate_cs;
	davinci_spi->get_rx = davinci_spi_rx_buf_u8;
	davinci_spi->get_tx = davinci_spi_tx_buf_u8;

	davinci_spi->bitbang.master->setup = davinci_spi_setup;
	davinci_spi->bitbang.master->cleanup = davinci_spi_cleanup;
	init_completion(&davinci_spi->done);

	ENTER
	    davinci_spi->base =
	    (struct davinci_spi_reg __iomem *)ioremap(r->start,
						      (r->end - r->start));

	if (davinci_spi->base == NULL) {
		ret = -ENOMEM;
		goto put_master;
	}

	davinci_spi->irq = platform_get_irq(dev, 0);

	if (davinci_spi->irq < 0) {
		ret = -ENXIO;
		goto unmap_io;
	}

	/* Register for SPI Interrupt */
	ret = request_irq(davinci_spi->irq, davinci_spi_irq,
			  SA_INTERRUPT, "dm_spi", davinci_spi);

	if (ret != 0) {
		DEBUG_SPI("request_irq fails\n");
		goto unmap_io;
	}

	master->bus_num = pdata->bus_num;
	master->num_chipselect = pdata->max_chipselect;

	/* SPI controller initializations */
	if (cpu_is_davinci_dm6467()) {
		clk_freq = 240 * 1000000 / 2;
	} else if (cpu_is_davinci_dm355()) {
		clk_freq = 216000000 / 2;

	} else {
		clk_freq = 270 * 1000000 / 4;
	}

	prescale = (clk_freq / SPI_BUS_FREQ) + 1;

	/*Reset In/OUT SPI modle */
	(davinci_spi->base->SPIGCR0) = 0x0;
	udelay(100);
	(davinci_spi->base->SPIGCR0) = 0x1;

	for (i = 0; i < 4; i++) {
		davinci_spi->base->SPIFMT[i] &= 0xFFFF00FF;

		davinci_spi->base->SPIFMT[i] |= (prescale << 8);
	}

	davinci_spi->dma_channels =
	    (struct davinci_spi_dma *)kzalloc(master->num_chipselect *
					      sizeof(struct davinci_spi_dma),
					      GFP_KERNEL);
	if (davinci_spi->dma_channels == NULL)
		goto free_irq;

	for (i = 0; i < master->num_chipselect; i++) {
		davinci_spi->dma_channels[i].dma_rx_channel = -1;
		davinci_spi->dma_channels[i].dma_rx_sync_dev = rxdma_id[i];
		davinci_spi->dma_channels[i].dma_tx_channel = -1;
		davinci_spi->dma_channels[i].dma_tx_sync_dev = txdma_id[i];
	}

	/* Allocate tmp_buf for tx_buf */
	davinci_spi->tmp_buf = kmalloc(SPI_BUFSIZ, SLAB_KERNEL);
	if (!davinci_spi->tmp_buf)
		goto free_tmp_buf;

	ret = davinci_spi_bitbang_start(&davinci_spi->bitbang);

	if (ret != 0)
		goto free_dma;

	printk(KERN_INFO "%s: davinci SPI Controller driver at "
	       "0x%p (irq = %d) use_dma=%d\n",
	       dev->dev.bus_id, davinci_spi->base, davinci_spi->irq, use_dma);

	EXIT
	return ret;

free_dma:
	kfree(davinci_spi->dma_channels);
free_tmp_buf:
	kfree(davinci_spi->tmp_buf);
free_irq:
	free_irq(davinci_spi->irq, davinci_spi);
unmap_io:

	iounmap(davinci_spi->base);
put_master:
	spi_master_put(master);
free_master:
	kfree(master);
err:
	return ret;
}

/**
 * davinci_spi_remove - remove function for SPI Master Controller
 * @dev: platform_device structure which contains plateform specific data
 *
 * This function will do the reverse action of davinci_spi_probe function
 * It will free the IRQ and SPI controller's memory region.
 * It will also call spi_bitbang_stop to destroy the work queue which was
 * created by spi_bitbang_start.
 */
static int __devexit davinci_spi_remove(struct device *d)
{
	struct platform_device *dev =
	    container_of(d, struct platform_device, dev);
	struct davinci_spi *davinci_spi;
	struct spi_master *master;

	master = dev_get_drvdata(&(dev)->dev);

	davinci_spi = spi_master_get_devdata(master);

	davinci_spi_bitbang_stop(&davinci_spi->bitbang);
	free_irq(davinci_spi->irq, davinci_spi);
	iounmap(davinci_spi->base);

	kfree(davinci_spi->tmp_buf);
	kfree(davinci_spi->dma_channels);

	spi_master_put(davinci_spi->bitbang.master);

	return 0;
}

static struct device_driver davinci_spi_driver = {
	.name = "dm_spi",
	.bus = &platform_bus_type,
	.probe = davinci_spi_probe,
	.remove = __devexit_p(davinci_spi_remove),
};

static int __init davinci_spi_init(void)
{
	return driver_register(&davinci_spi_driver);
}

static void __exit davinci_spi_exit(void)
{
	driver_unregister(&davinci_spi_driver);
}

module_init(davinci_spi_init);
module_exit(davinci_spi_exit);

MODULE_AUTHOR("Dhruval Shah & Varun Shah");
MODULE_DESCRIPTION("DM355 SPI Master Controller Driver");
MODULE_LICENSE("GPL");
