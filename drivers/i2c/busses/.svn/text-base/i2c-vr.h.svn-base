/*
 * drivers/i2c/busses/i2c-vr.h
 *
 * Header file for the Vermilion Range I2C driver.
 *
 * 2006 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#ifndef I2C_VR_H
#define I2C_VR_H

#define	I2C_VR_ADAP_NR	2

#define I2C_VR_DEF_TIMEOUT             3

#define I2C_VR_BUS_ERROR               (-EREMOTEIO)
#define I2C_VR_XFER_NAKED              (-ECONNREFUSED)

struct i2c_vr_regs {
	uint32_t icr;
	uint32_t isr;
	uint32_t reserved;
	uint32_t idbr;
	uint32_t reserved1;
	uint32_t ibmr;
};

/*
 * New stuff...
 */
struct vr_i2c {
	spinlock_t lock;
	wait_queue_head_t wait;
	struct i2c_msg *msg;
	unsigned int msg_num;
	unsigned int msg_idx;
	unsigned int msg_ptr;
	struct i2c_vr_regs __iomem *regs;

	struct i2c_adapter adap;

	int irqlogidx;
	u32 isrlog[32];
	u32 icrlog[32];
};

/*
 * I2C registers
 */

#define ICR_START	(1 << 0)	/* start bit */
#define ICR_STOP	(1 << 1)	/* stop bit */
#define ICR_ACKNAK	(1 << 2)	/* send ACK(0) or NAK(1) */
#define ICR_TB		(1 << 3)	/* transfer byte bit */
#define ICR_MA		(1 << 4)	/* master abort */
#define ICR_SCLE	(1 << 5)	/* master clock enable */
#define ICR_IUE		(1 << 6)	/* unit enable */
#define ICR_GCD		(1 << 7)	/* general call disable */
#define ICR_ITEIE	(1 << 8)	/* enable tx interrupts */
#define ICR_IRFIE	(1 << 9)	/* enable rx interrupts */
#define ICR_BEIE	(1 << 10)	/* enable bus error ints */
#define ICR_SSDIE	(1 << 11)	/* slave STOP detected int enable */
#define ICR_ALDIE	(1 << 12)	/* enable arbitration interrupt */
#define ICR_SADIE	(1 << 13)	/* slave address detected int enable */
#define ICR_UR		(1 << 14)	/* unit reset */
#define ICR_FM		(1 << 15)	/* fast mode */

#define ISR_RWM		(1 << 0)	/* read/write mode */
#define ISR_ACKNAK	(1 << 1)	/* ack/nak status */
#define ISR_UB		(1 << 2)	/* unit busy */
#define ISR_IBB		(1 << 3)	/* bus busy */
#define ISR_SSD		(1 << 4)	/* slave stop detected */
#define ISR_ALD		(1 << 5)	/* arbitration loss detected */
#define ISR_ITE		(1 << 6)	/* tx buffer empty */
#define ISR_IRF		(1 << 7)	/* rx buffer full */
#define ISR_GCAD	(1 << 8)	/* general call address detected */
#define ISR_SAD		(1 << 9)	/* slave address detected */
#define ISR_BED		(1 << 10)	/* bus error no ACK/NAK */

#define I2C_RETRY               (-2000)	/* an error has occurred retry transmit */
#define I2C_TRANSMIT		1
#define I2C_RECEIVE		0

#define I2C_ICR_INIT	(ICR_BEIE | ICR_IRFIE | ICR_ITEIE | ICR_GCD | ICR_SCLE)
/* ICR initialize bit values
*
*  15. FM       0 (100 Khz operation)
*  14. UR       0 (No unit reset)
*  13. SADIE    0 (Disables the unit from interrupting on slave addresses
*                                       matching its slave address)
*  12. ALDIE    0 (Disables the unit from interrupt when it loses arbitration
*                                       in master mode)
*  11. SSDIE    0 (Disables interrupts from a slave stop detected, in slave mode)
*  10. BEIE     1 (Enable interrupts from detected bus errors, no ACK sent)
*  9.  IRFIE    1 (Enable interrupts from full buffer received)
*  8.  ITEIE    1 (Enables the I2C unit to interrupt when transmit buffer empty)
*  7.  GCD      1 (Disables i2c unit response to general call messages as a slave)
*  6.  IUE      0 (Disable unit until we change settings)
*  5.  SCLE     1 (Enables the i2c clock output for master mode (drives SCL)
*  4.  MA       0 (Only send stop with the ICR stop bit)
*  3.  TB       0 (We are not transmitting a byte initially)
*  2.  ACKNAK   0 (Send an ACK after the unit receives a byte)
*  1.  STOP     0 (Do not send a STOP)
*  0.  START    0 (Do not send a START)
*
*/

#define I2C_ISR_INIT	0x7FF
/* I2C status register init values
 *
 * 10. BED      1 (Clear bus error detected)
 * 9.  SAD      1 (Clear slave address detected)
 * 7.  IRF      1 (Clear IDBR Receive Full)
 * 6.  ITE      1 (Clear IDBR Transmit Empty)
 * 5.  ALD      1 (Clear Arbitration Loss Detected)
 * 4.  SSD      1 (Clear Slave Stop Detected)
 */

#define I2C_ISR_IRQ	(ISR_SSD | ISR_ALD | ISR_ITE | ISR_IRF | ISR_SAD | ISR_BED)

#endif
