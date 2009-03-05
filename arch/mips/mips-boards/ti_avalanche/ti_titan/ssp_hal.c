/*
 * Copyright (C) 2006 Texas Instruments Inc.
 *
 * Author: Manish Lachwani (mlachwani@mvista.com)
 * Copyright (C) 2006 Montavista Software Inc.
 *
 * This program is free software; you can distribute it and/or modify it
 * under the terms of the GNU General Public License (Version 2) as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 */
	
#include <ssp_hal.h>
#include <pal.h>
	
struct semaphore ssp_drvp_module_in_use;
	
ssp_hal_i2c_info_t* ssp_i2c_open( void );
	
ssp_hal_i2c_info_t ssp_i2c_static_config = {
	1, 0, 0x00, SSP_I2C_OUTPUT_CLK_FREQ, SSP_HAL_MODE_INTR
};
	
ssp_hal_spi_info_t ssp_spi_static_config = {
	0, 0, 1, 2, SSP_SPI_OUTPUT_CLK_FREQ, SSP_HAL_MODE_INTR
};
	
int __init ssp_dev_init(void)
{
	int error_num; 
	
	if((error_num = avalanche_sysprobe_and_prep(AVALANCHE_SSP_HW_MODULE_REV, AVALANCHE_SSP_BASE, NULL)) < 0) {
		printk("SSP: avalanche_sysprobe_and_prep failed\n");
		return -EINVAL;
	}
	
	avalanche_reset_ctrl(AVALANCHE_SSP_RESET_BIT, OUT_OF_RESET);
	sema_init( &ssp_drvp_module_in_use,1 );
	if( ssp_hal_init( AVALANCHE_SSP_BASE, avalanche_clkc_getfreq( CLKC_VBUS ) ) != SSP_DRV_OK )
		return SSP_DRV_ERROR;   
	
	return SSP_DRV_OK;
}
	
ssp_hal_i2c_info_t* ssp_i2c_open( void )
{
	ssp_hal_i2c_info_t  info;
	
	memcpy(&info, &ssp_i2c_static_config, sizeof(ssp_hal_i2c_info_t));
	
	if (down_interruptible(&ssp_drvp_module_in_use))
		return NULL; 
	
	return ssp_hal_i2c_open(info);
}
	
int ssp_i2c_close(ssp_hal_i2c_info_t *     id)
{
	ssp_hal_i2c_close(id);
	
	up(&ssp_drvp_module_in_use);
	
	return 0;
}
	
int ssp_i2c_read( ssp_hal_i2c_info_t  *info, unsigned char    *buffer, unsigned int   len)
{
	return ssp_hal_i2c_read(info, buffer, len);
}
	
int ssp_i2c_write( ssp_hal_i2c_info_t  *info, unsigned char    *buffer, unsigned int   len)
{
	return ssp_hal_i2c_write(info, buffer, len);
}
	
void __exit ssp_dev_exit(void)
{
	return;
}
	
module_init(ssp_dev_init);
module_exit(ssp_dev_exit);
	
#define SSP_HAL_PT0_SEQ_RAM_LOW                         0x100
#define SSP_HAL_PT1_SEQ_RAM_LOW                         0x180
#define SSP_HAL_IOSEL_PT0_USE_EXTENDED_RAM              0x00008000
#define SSP_HAL_PORT0_INT_ENABLE                        0x00000001
#define SSP_HAL_PORT1_INT_ENABLE                        0x00000002
#define SSP_HAL_PT0_CLK_DIV_MASK                        0x0000000F
#define SSP_HAL_PT0_RAM_WRITE_ENABLE                    0x00000010
#define SSP_HAL_PT0_RAM_READ_ENABLE                     0x00000020
#define SSP_HAL_PT0_16BIT_DATA_REG                      0x00000040
#define SSP_HAL_PT0_AUTO_XFR_START                      0x00000080
#define SSP_HAL_PT0_SOFT_RESET                          0x00000100
#define SSP_HAL_DIRECT_INPUT                             0x00
#define SSP_HAL_DIRECT_OUTPUT                            0x04
#define SSP_HAL_PT0_DATA_OUTPUT                          0x01
#define SSP_HAL_PT0_CLK                                  0x02
#define SSP_HAL_PT0_CS                                   0x03
#define SSP_HAL_PIN0_DIRECT_OUT_HIGH                     0x00000040
#define SSP_HAL_PIN1_DIRECT_OUT_HIGH                     0x00000080
#define SSP_HAL_PIN2_DIRECT_OUT_HIGH                     0x00000100
#define SSP_HAL_PIN3_DIRECT_OUT_HIGH                     0x00000200
#define SSP_HAL_PIN4_DIRECT_OUT_HIGH                     0x00000400
#define SSP_HAL_IOSEL_PIN0_MASK                          0x00000007
#define SSP_HAL_IOSEL_PIN1_MASK                          0x00000038
#define SSP_HAL_IOSEL_PIN2_MASK                          0x000001C0
#define SSP_HAL_IOSEL_PIN3_MASK                          0x00000E00
#define SSP_HAL_IOSEL_PIN4_MASK                          0x00007000
#define SSP_HAL_IOSEL_PT0_USE_EXTENDED_RAM               0x00008000
#define SSP_HAL_IOSEL_PIN0_BIT_POS                       0
#define SSP_HAL_IOSEL_PIN1_BIT_POS                       3
#define SSP_HAL_IOSEL_PIN2_BIT_POS                       6
#define SSP_HAL_IOSEL_PIN3_BIT_POS                       9
#define SSP_HAL_IOSEL_PIN4_BIT_POS                       12
#define SSP_HAL_PT0_STATE_STOP_ADDR_MASK                0x0000003F         
#define SSP_HAL_CLK_PRE_DIV_MASK                         0x000000FF
#define SSP_HAL_PT0_START_ADDR_MASK                      0x0000003F
#define SSP_HAL_PT0_START_ADDR_VALID_MASK                0x00000080
#define SSP_HAL_PT0_EARLY_DATA_IN_MASK                   0x00000100
#define SSP_HAL_PT0_DELAY_DATA_OUT_MASK                  0x00000200
#define SSP_HAL_PT0_STATE_BUSY_MASK                      0x00000400
#define SSP_HAL_PT0_CLEAR_DATA_REG_MASK                  0x00000800
#define SSP_HAL_PT0_SHIFT_ADDR_LSB_FIRST                 0x00001000
#define SSP_HAL_PT0_SHIFT_DATA_LSB_FIRST                 0x00002000
#define SSP_HAL_PT0_START_SERIAL_TRANSFER                0x00008000
	
#define SSP_HAL_SPI_SERIAL_DATA_MASK                    0x000000FF
#define SSP_HAL_I2C_SERIAL_DATA_MASK                    0x000000FF
#define SSP_HAL_I2C_READ_ENABLE_BIT                     0x00000100
#define SSP_HAL_I2C_ADDR_MASK                           0x0000FE00
#define SSP_HAL_I2C_ADDR_SHIFT_AMT                      9
#define SSP_HAL_SEQMAP_I2C_STOP_ADDR                    0x15
#define SSP_HAL_SEQMAP_I2C_READ_BYTE_ADDR               0x09
#define SSP_HAL_SEQMAP_I2C_READ_START_ADDR              0x04
#define SSP_HAL_SEQMAP_I2C_READ_ADDR_PAUSE              0x08
#define SSP_HAL_SEQMAP_I2C_READ_BYTE_PAUSE              0x0B
#define SSP_HAL_SEQMAP_I2C_WRITE_BYTE_ADDR              0x11
#define SSP_HAL_SEQMAP_I2C_WRITE_START_ADDR             0x0C
#define SSP_HAL_SEQMAP_I2C_WRITE_ADDR_PAUSE             0x10
#define SSP_HAL_SEQMAP_I2C_WRITE_BYTE_PAUSE             0x14
#define SSP_HAL_CLK_HIGH                                (1 << 0)
#define SSP_HAL_CLK_LOW                                 (0 << 0)
#define SSP_HAL_DATA_HIGH                               (1 << 1)
#define SSP_HAL_DATA_LOW                                (0 << 1)
#define SSP_HAL_CS_HIGH                                 (1 << 2)
#define SSP_HAL_CS_LOW                                  (0 << 2)
#define SSP_HAL_INPUT_MODE                              (0 << 3)                        
#define SSP_HAL_OUTPUT_MODE                             (1 << 3)                        
#define SSP_HAL_DATA_REG                                (1 << 4)
#define SSP_HAL_ADDR_REG                                (0 << 4)
#define SSP_HAL_OPCODE_DIRECT                           ((0x0) << 5)
#define SSP_HAL_OPCODE_TOGGLE                           ((0x1) << 5)
#define SSP_HAL_OPCODE_SHIFT                            ((0x2) << 5)
#define SSP_HAL_OPCODE_BRANCH0                          ((0x4) << 5)
#define SSP_HAL_OPCODE_BRANCH1                          ((0x5) << 5)
#define SSP_HAL_OPCODE_BRANCH                           ((0x6) << 5)
#define SSP_HAL_OPCODE_STOP                             ((0x7) << 5)
#define SSP_HAL_NOT_APPLICABLE                          0
#define SSP_HAL_BRANCH_ADDR_BIT_POS                     8
#define SSP_HAL_CNT_VALUE_BIT_POS                       8
#define SSP_HAL_BRANCH(addr)             ((addr)<<SSP_HAL_BRANCH_ADDR_BIT_POS)
#define SSP_HAL_COUNT(cycles)            ((cycles)<<SSP_HAL_CNT_VALUE_BIT_POS)
#define SSP_HAL_SEQ_SIZE                                32
#define SSP_HAL_I2C_READ_ENABLE                       0x02
#define SSP_HAL_WRITE_ADDR_WRITE                         1 
#define SSP_HAL_WRITE_DATA                               0 
#define SSP_HAL_WRITE_DATA_LAST                          2
#define SSP_HAL_READ_ADDR_WRITE                          4
#define SSP_HAL_READ_DATA_LAST                           5
#define SSP_HAL_OUTPUT_SAMPLE_RATE                      50
	
typedef struct SSP_HAL_REG_tag
{
	unsigned int revision_control;
	unsigned int iosel_control_1;
	unsigned int iosel_control_2;
	unsigned int clk_pre_divider;
	unsigned int interrupt_status;
	unsigned int interrupt_enable;
	unsigned int test_control;
	unsigned int pad1[9];
	unsigned int port0_config_2;
	unsigned int port0_address;
	unsigned int port0_data;    
	unsigned int port0_config_1;
	unsigned int port0_state;    
	unsigned int pad2[11];
	unsigned int port1_config_2;
	unsigned int port1_address;
	unsigned int port1_data;    
	unsigned int port1_config_1;
	unsigned int port1_state;    
}SSP_HAL_REG_T;
	
	
typedef struct 
{
	void* reentrantLock;
	void* completionSem;
	signed char completionSemAllocated;
	signed char reentrantLockAllocated;
}SSP_HAL_DEV;

SSP_HAL_DEV        ssp_dev;
typedef void *     SspHandle;
	
struct semaphore ssp_drvp_xfr_complete;
	
typedef struct  SSP_HAL_OBJ_tag
{    
	unsigned int base_addr;
	unsigned int input_freq;
	
}SSP_HAL_OBJ_T;
	
static volatile SSP_HAL_REG_T *ssp_halp_p_regs;
	
static SSP_HAL_OBJ_T ssp_halp_obj;
	
static volatile unsigned int *ssp_halp_p_pt0_ram;
static volatile unsigned int *ssp_halp_p_pt1_ram;
extern unsigned int ssp_halp_spi_read_seq_map[];
extern unsigned int ssp_halp_spi_write_seq_map[];
extern unsigned int ssp_halp_spi_stop_seq_map[];
extern unsigned int ssp_halp_i2c_engine0_write_seq_map[];
extern unsigned int ssp_halp_i2c_engine0_read_seq_map[];
extern unsigned int ssp_halp_i2c_engine1_seq_map[];
static inline unsigned int ssp_hal_i2c_start(void);
static unsigned int ssp_halp_load_seq_ram (unsigned int* p_seq_map, 
unsigned int map_size, 
unsigned int start_address);
	
	
static unsigned int ssp_halp_load_seq_ram1 (unsigned int* p_seq_map,
unsigned int map_size,
unsigned int start_address);
void ssp_isr(void)
{
	up( &ssp_drvp_xfr_complete );
}
	
static inline void ssp_wait_for_xfr_done(void)
{
	down( &ssp_drvp_xfr_complete );
}
	
unsigned int ssp_hal_init
(
	unsigned int base_address,
	unsigned int module_input_freq
)
{
	ssp_halp_obj.base_addr = base_address;
	ssp_halp_obj.input_freq = module_input_freq;
	ssp_halp_p_regs    = (volatile SSP_HAL_REG_T*)base_address;
	ssp_halp_p_pt0_ram = (volatile unsigned int*)(base_address + \
	SSP_HAL_PT0_SEQ_RAM_LOW);
	
	ssp_halp_p_pt1_ram = (volatile unsigned int*)(base_address + \
	SSP_HAL_PT1_SEQ_RAM_LOW);
	
	ssp_halp_p_regs->port0_config_2 &= ~SSP_HAL_PT0_SOFT_RESET;
	ssp_halp_p_regs->port1_config_2 &= ~SSP_HAL_PT0_SOFT_RESET;
	ssp_halp_p_regs->port0_config_2 |= SSP_HAL_PT0_16BIT_DATA_REG;
	ssp_halp_p_regs->port1_config_2 |= SSP_HAL_PT0_16BIT_DATA_REG;
	ssp_halp_p_regs->port0_config_2 &= ~SSP_HAL_PT0_AUTO_XFR_START;
	ssp_halp_p_regs->port1_config_2 &= ~SSP_HAL_PT0_AUTO_XFR_START;
	ssp_halp_p_regs->port0_config_1 &=~SSP_HAL_PT0_SHIFT_ADDR_LSB_FIRST;
	ssp_halp_p_regs->port0_config_1 &=~SSP_HAL_PT0_SHIFT_DATA_LSB_FIRST;
	ssp_halp_p_regs->port1_config_1 &=~SSP_HAL_PT0_SHIFT_ADDR_LSB_FIRST;
	ssp_halp_p_regs->port1_config_1 &=~SSP_HAL_PT0_SHIFT_DATA_LSB_FIRST;
	ssp_halp_p_regs->iosel_control_1 &= ~SSP_HAL_IOSEL_PT0_USE_EXTENDED_RAM;
	ssp_halp_p_regs->port0_config_1 |= SSP_HAL_PT0_START_ADDR_VALID_MASK;
	ssp_halp_p_regs->port1_config_1 |= SSP_HAL_PT0_START_ADDR_VALID_MASK;
	ssp_halp_p_regs->port0_config_2 &= ~SSP_HAL_PT0_RAM_WRITE_ENABLE;
	ssp_halp_p_regs->port0_config_2 &= ~SSP_HAL_PT0_RAM_READ_ENABLE;
	
	ssp_halp_p_regs->port1_config_2 &= ~SSP_HAL_PT0_RAM_WRITE_ENABLE;
	ssp_halp_p_regs->port1_config_2 &= ~SSP_HAL_PT0_RAM_READ_ENABLE;
	
	sema_init( &ssp_drvp_xfr_complete, 0);    
	
	return SSP_HAL_OK;
}
	
ssp_hal_i2c_info_t* ssp_hal_i2c_open
(
	ssp_hal_i2c_info_t info 
)
{
	unsigned int divisor;
	unsigned int sample_freq;
	ssp_hal_i2c_info_t* ret;
	
	ssp_halp_load_seq_ram(ssp_halp_i2c_engine1_seq_map, SSP_HAL_SEQ_SIZE,  \
	(unsigned int)ssp_halp_p_pt0_ram);
	
	ssp_halp_load_seq_ram1(ssp_halp_i2c_engine0_write_seq_map, SSP_HAL_SEQ_SIZE,
	(unsigned int)ssp_halp_p_pt1_ram);
	
	ssp_halp_p_regs->port0_config_2 |= SSP_HAL_PT0_SOFT_RESET;
	ssp_halp_p_regs->port0_config_2 &= ~SSP_HAL_PT0_SOFT_RESET;
	
	
	ssp_halp_p_regs->iosel_control_2 &= ~0x3F;
	ssp_halp_p_regs->iosel_control_1 &= ~0x7FFF;    
	ssp_halp_p_regs->iosel_control_1 &= 0;  
	
	ssp_halp_p_regs->iosel_control_1 |= (0x1 << (3 * info.clock_pin));
	ssp_halp_p_regs->iosel_control_2 |= 0x0 << (3 * info.clock_pin);
	ssp_halp_p_regs->iosel_control_1 |= (0x5 << (3 * info.data_pin)) ;
	ssp_halp_p_regs->iosel_control_2 |= (1 << (3 * info.data_pin));
	sample_freq = (info.bus_speed) * SSP_HAL_OUTPUT_SAMPLE_RATE;
	divisor = (unsigned int)((ssp_halp_obj.input_freq/sample_freq) - 1);
	ssp_halp_p_regs->clk_pre_divider = divisor;
	ssp_halp_p_regs->port0_config_2 &= ~0xF;
	ssp_halp_p_regs->port1_config_2 &= ~0xF;   
	
	if(info.mode == SSP_HAL_MODE_INTR) {
		ssp_halp_p_regs->interrupt_enable |= SSP_HAL_PORT1_INT_ENABLE;        
	} else {
		ssp_halp_p_regs->interrupt_enable &= ~SSP_HAL_PORT0_INT_ENABLE;
		ssp_halp_p_regs->interrupt_enable &= ~SSP_HAL_PORT1_INT_ENABLE;
	}
	
	if(avalanche_memalloc(sizeof(ssp_hal_i2c_info_t),( void *)&ret) != 0) {
		return NULL;
	} 
	
	ret->data_pin  = info.data_pin;
	ret->clock_pin = info.clock_pin;
	ret->addr      = info.addr;
	
	ret->bus_speed = info.bus_speed;
	ret->mode      = info.mode;
	
	ssp_halp_p_regs->port0_config_1 |= SSP_HAL_PT0_DELAY_DATA_OUT_MASK; 
	
	return ret;
}
	
unsigned int ssp_hal_i2c_close
(
	ssp_hal_i2c_info_t* info
) 
{  
	avalanche_memfree(( void *)info);
	
	return SSP_HAL_OK;
}
	
static unsigned int ssp_hal_update_seq_ram
(
	unsigned int      start_address,
	unsigned int      data,
	unsigned int      seq_id
)
{
	volatile unsigned int *p_ssp_ram = (volatile unsigned int*)start_address; 
	unsigned int           count;
	unsigned int           tempVal = data;
	unsigned char            startOffset = 0;
	
	ssp_halp_p_regs->port1_config_2 |= SSP_HAL_PT0_RAM_WRITE_ENABLE;
	
	if (seq_id == SSP_HAL_WRITE_DATA_LAST)
		startOffset = 4;
	if (seq_id == SSP_HAL_WRITE_ADDR_WRITE)
		startOffset = 3;
	if (seq_id == SSP_HAL_WRITE_DATA)
		startOffset = 4;
	if (seq_id == SSP_HAL_READ_ADDR_WRITE)
		startOffset = 3;
	
	if (seq_id == SSP_HAL_READ_DATA_LAST) {
		p_ssp_ram[18] = ((0x25 <<8) |SSP_HAL_OPCODE_TOGGLE |SSP_HAL_ADDR_REG 
				|SSP_HAL_OUTPUT_MODE |SSP_HAL_CS_LOW 
				|SSP_HAL_DATA_LOW |SSP_HAL_CLK_LOW);
	
	
		p_ssp_ram[19] = ((0x01 <<8) |SSP_HAL_OPCODE_TOGGLE |SSP_HAL_DATA_REG 
				|SSP_HAL_OUTPUT_MODE |SSP_HAL_CS_LOW 
				|SSP_HAL_DATA_LOW |SSP_HAL_CLK_LOW);
	
		p_ssp_ram[20] = ((0x00 <<8) |SSP_HAL_OPCODE_STOP |SSP_HAL_NOT_APPLICABLE     
				|SSP_HAL_OUTPUT_MODE |SSP_HAL_CS_LOW 
				|SSP_HAL_DATA_LOW |SSP_HAL_CLK_LOW);
	
		ssp_halp_p_regs->port1_config_2 &= ~SSP_HAL_PT0_RAM_WRITE_ENABLE;
	
		return SSP_HAL_OK;                  
	}    
	
	tempVal &= 0x1;
	
	for (count = 11; count > startOffset ; count--){
		data = data >> 1;
	
		if(data & 0x1)
			p_ssp_ram[count] = ((0x31 <<8)|SSP_HAL_OPCODE_TOGGLE |SSP_HAL_ADDR_REG 
						|SSP_HAL_OUTPUT_MODE | SSP_HAL_CS_LOW 
						|SSP_HAL_DATA_HIGH |SSP_HAL_CLK_LOW);
	
		else
			p_ssp_ram[count] = ((0x31 <<8)|SSP_HAL_OPCODE_TOGGLE |SSP_HAL_ADDR_REG 
						|SSP_HAL_OUTPUT_MODE |SSP_HAL_CS_LOW 
						|SSP_HAL_DATA_LOW |SSP_HAL_CLK_LOW);        
	}
	
	if (seq_id == SSP_HAL_READ_ADDR_WRITE){
		p_ssp_ram[0] =  ((0x00 <<8)|SSP_HAL_OPCODE_STOP   |SSP_HAL_NOT_APPLICABLE     
				|SSP_HAL_INPUT_MODE    |SSP_HAL_CS_LOW 
				|SSP_HAL_DATA_LOW      |SSP_HAL_CLK_LOW);
	
		p_ssp_ram[15] = ((0x16 <<8)|SSP_HAL_OPCODE_TOGGLE |SSP_HAL_DATA_REG 
				|SSP_HAL_INPUT_MODE    |SSP_HAL_CS_LOW 
				|SSP_HAL_DATA_HIGH     |SSP_HAL_CLK_LOW);
	
		p_ssp_ram[16] = ((0x01 <<8)|SSP_HAL_OPCODE_SHIFT  |SSP_HAL_DATA_REG 
				|SSP_HAL_INPUT_MODE    |SSP_HAL_CS_LOW 
				|SSP_HAL_NOT_APPLICABLE|SSP_HAL_CLK_LOW);
	
		p_ssp_ram[17] = ((0x00 <<8)|SSP_HAL_OPCODE_BRANCH |SSP_HAL_NOT_APPLICABLE     
				|SSP_HAL_INPUT_MODE    |SSP_HAL_CS_LOW 
				|SSP_HAL_DATA_LOW      |SSP_HAL_CLK_LOW); 
	}
	
	if (seq_id == SSP_HAL_WRITE_DATA_LAST){
		p_ssp_ram[15] = ((0x31 <<8)|SSP_HAL_OPCODE_TOGGLE |SSP_HAL_ADDR_REG 
				|SSP_HAL_OUTPUT_MODE   |SSP_HAL_CS_LOW 
				|SSP_HAL_DATA_LOW      |SSP_HAL_CLK_LOW);  
	
		p_ssp_ram[16] = ((0x11 <<8)|SSP_HAL_OPCODE_BRANCH |SSP_HAL_NOT_APPLICABLE     
				|SSP_HAL_OUTPUT_MODE   |SSP_HAL_CS_LOW 
				|SSP_HAL_DATA_LOW      |SSP_HAL_CLK_LOW);
	
		p_ssp_ram[18] = ((0x31 <<8)|SSP_HAL_OPCODE_TOGGLE |SSP_HAL_NOT_APPLICABLE     
				|SSP_HAL_OUTPUT_MODE   |SSP_HAL_CS_LOW 
				|SSP_HAL_DATA_LOW      |SSP_HAL_CLK_LOW);
	
		p_ssp_ram[19] = ((0x00 <<8)|SSP_HAL_OPCODE_STOP   |SSP_HAL_NOT_APPLICABLE     
				|SSP_HAL_INPUT_MODE   |SSP_HAL_CS_LOW 
				|SSP_HAL_DATA_HIGH     |SSP_HAL_CLK_LOW);   
	}
	
	if ((seq_id == SSP_HAL_WRITE_ADDR_WRITE) || (seq_id == SSP_HAL_WRITE_DATA)) {
		if(tempVal & 0x1){
			p_ssp_ram[15] = ((0x30 <<8)|SSP_HAL_OPCODE_TOGGLE |SSP_HAL_NOT_APPLICABLE     
					|SSP_HAL_OUTPUT_MODE   |SSP_HAL_CS_LOW 
					|SSP_HAL_DATA_HIGH      |SSP_HAL_CLK_LOW);
	
			p_ssp_ram[16] = ((0x00 <<8)|SSP_HAL_OPCODE_BRANCH |SSP_HAL_NOT_APPLICABLE     
					|SSP_HAL_OUTPUT_MODE   |SSP_HAL_CS_LOW 
					|SSP_HAL_DATA_HIGH     |SSP_HAL_CLK_LOW);   
		} else {
			p_ssp_ram[15] = ((0x19 <<8)|SSP_HAL_OPCODE_TOGGLE |SSP_HAL_NOT_APPLICABLE     
					|SSP_HAL_OUTPUT_MODE   |SSP_HAL_CS_LOW 
					|SSP_HAL_DATA_LOW      |SSP_HAL_CLK_LOW);
	
			p_ssp_ram[16] = ((0x19 <<8)|SSP_HAL_OPCODE_TOGGLE |SSP_HAL_NOT_APPLICABLE     
					|SSP_HAL_OUTPUT_MODE   |SSP_HAL_CS_LOW 
					|SSP_HAL_DATA_LOW      |SSP_HAL_CLK_LOW);
	
			p_ssp_ram[17] = ((0x00 <<8)|SSP_HAL_OPCODE_STOP   |SSP_HAL_NOT_APPLICABLE     
					|SSP_HAL_OUTPUT_MODE   |SSP_HAL_CS_LOW 
					|SSP_HAL_DATA_LOW     |SSP_HAL_CLK_HIGH);   
		}        
	}
	
	/* disable RAM reads/writes again */
	ssp_halp_p_regs->port1_config_2 &= ~SSP_HAL_PT0_RAM_WRITE_ENABLE;
	
	return SSP_HAL_OK;
}
	
static inline unsigned int ssp_hal_i2c_start()
{
	for(;ssp_halp_p_regs->port0_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;);

	for(;ssp_halp_p_regs->port1_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;);
	
	ssp_halp_p_regs->port0_config_1 |= SSP_HAL_PT0_START_SERIAL_TRANSFER;
	ssp_halp_p_regs->port1_config_1 |= SSP_HAL_PT0_START_SERIAL_TRANSFER;
	
	return 0;
}
	
unsigned int ssp_hal_i2c_wait_for_free(ssp_hal_i2c_info_t  *info)
{
	ssp_halp_p_regs->port0_config_1 &= (~SSP_HAL_PT0_START_ADDR_MASK);
	ssp_halp_p_regs->port0_config_1 |= (0x19 & 
	SSP_HAL_PT0_START_ADDR_MASK);
	
	/* Trigger device address write by writing to ST bit */
	ssp_halp_p_regs->port0_config_1 |= SSP_HAL_PT0_START_SERIAL_TRANSFER;
	
	for(;ssp_halp_p_regs->port0_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;);
	
	if((ssp_halp_p_regs->port0_state & SSP_HAL_PT0_STATE_STOP_ADDR_MASK) !=
		0x1b) {
		return 1;
	}
	return 0;    
}
	
unsigned int ssp_hal_i2c_write(ssp_hal_i2c_info_t  *info,unsigned char *buffer, unsigned int len)
{
	unsigned int count = 0;
	unsigned int  data;
	unsigned char bitState;
	
	if(!buffer) {
		return SSP_HAL_ERROR;            
	}
	
	ssp_halp_load_seq_ram1(ssp_halp_i2c_engine0_write_seq_map, SSP_HAL_SEQ_SIZE,
				(unsigned int)ssp_halp_p_pt1_ram);
	
	ssp_halp_load_seq_ram(ssp_halp_i2c_engine1_seq_map, SSP_HAL_SEQ_SIZE,
				(unsigned int)ssp_halp_p_pt0_ram);        
	
	bitState = ((buffer[0] & 0x80) >> 7);
	
	data = (info->addr << 2)| bitState;
	data &= ~SSP_HAL_I2C_READ_ENABLE;
	
	ssp_hal_update_seq_ram((unsigned int)ssp_halp_p_pt1_ram, data, 
				SSP_HAL_WRITE_ADDR_WRITE);
	
	ssp_halp_p_regs->port0_config_1 &= (~SSP_HAL_PT0_START_ADDR_MASK);
	ssp_halp_p_regs->port0_config_1 |= (0x01 & 
				SSP_HAL_PT0_START_ADDR_MASK);
	
	ssp_halp_p_regs->port1_config_1 &= (~SSP_HAL_PT0_START_ADDR_MASK);
	ssp_halp_p_regs->port1_config_1 |= (0x03 & SSP_HAL_PT0_START_ADDR_MASK);
	
	ssp_hal_i2c_start();
	
	if(info->mode == SSP_HAL_MODE_POLL) {        
		for(;ssp_halp_p_regs->port1_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;);                  
		for(;ssp_halp_p_regs->port0_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;);
	} else {
		ssp_wait_for_xfr_done();
	}
	
	if((ssp_halp_p_regs->port1_state & SSP_HAL_PT0_STATE_STOP_ADDR_MASK) == 0x14) {
		goto writeerr;
	}
	
	if((ssp_halp_p_regs->port0_state & SSP_HAL_PT0_STATE_STOP_ADDR_MASK) != 0x15) {
		goto writeerr;
	}
	ssp_hal_i2c_wait_for_free(info);
	ssp_isr();
	
	for(count = 0; count < len - 1; count++) {
		if(info->mode == SSP_HAL_MODE_POLL) {
			for(;ssp_halp_p_regs->port0_config_1&SSP_HAL_PT0_STATE_BUSY_MASK;);
			for(;ssp_halp_p_regs->port1_config_1&SSP_HAL_PT0_STATE_BUSY_MASK;); 
		}	 
	
		data = (buffer[count] & 0x7F) <<  1;
		bitState = ((buffer[count + 1] & 0x80) >> 7);
		data = data | bitState;
	
		ssp_hal_update_seq_ram((unsigned int)ssp_halp_p_pt1_ram, data, SSP_HAL_WRITE_DATA);
		ssp_halp_p_regs->port1_config_1 &= (~SSP_HAL_PT0_START_ADDR_MASK);
		ssp_halp_p_regs->port1_config_1 |= ( 0x05 
						& SSP_HAL_PT0_START_ADDR_MASK); 

		ssp_halp_p_regs->port0_config_1 &= (~SSP_HAL_PT0_START_ADDR_MASK);
		ssp_halp_p_regs->port0_config_1 |= ( 0x04 
						& SSP_HAL_PT0_START_ADDR_MASK);
	
		ssp_hal_i2c_start();
	
		if(info->mode == SSP_HAL_MODE_POLL) {        
			for(;ssp_halp_p_regs->port1_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;);                  
			for(;ssp_halp_p_regs->port0_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;);
		} else {
			ssp_wait_for_xfr_done();
		}
	
		if((ssp_halp_p_regs->port1_state & SSP_HAL_PT0_STATE_STOP_ADDR_MASK) == 0x14) {
			goto writeerr;
		}
	
		if((ssp_halp_p_regs->port0_state & SSP_HAL_PT0_STATE_STOP_ADDR_MASK) != 0x15) {
			goto writeerr;
		}
		ssp_hal_i2c_wait_for_free(info);
	}
	ssp_isr();	
	if(info->mode == SSP_HAL_MODE_POLL) {
		for(;ssp_halp_p_regs->port0_config_1&SSP_HAL_PT0_STATE_BUSY_MASK;);
		for(;ssp_halp_p_regs->port1_config_1&SSP_HAL_PT0_STATE_BUSY_MASK;);        
	} 
	
	data = (buffer[count] & 0x7F) <<  1;
	
	ssp_hal_update_seq_ram((unsigned int)ssp_halp_p_pt1_ram, data, SSP_HAL_WRITE_DATA_LAST);
	
	ssp_halp_p_regs->port1_config_1 &= (~SSP_HAL_PT0_START_ADDR_MASK);
	ssp_halp_p_regs->port1_config_1 |= (0x05 & SSP_HAL_PT0_START_ADDR_MASK);                   
	ssp_halp_p_regs->port0_config_1 &= (~SSP_HAL_PT0_START_ADDR_MASK);
	ssp_halp_p_regs->port0_config_1 |= (0x06 & SSP_HAL_PT0_START_ADDR_MASK);
	
	ssp_hal_i2c_start();
	
	if(info->mode == SSP_HAL_MODE_POLL) {        
		for(;ssp_halp_p_regs->port1_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;); 
		for(;ssp_halp_p_regs->port0_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;);
	} else {
		ssp_wait_for_xfr_done();
	}
	
	if((ssp_halp_p_regs->port1_state & SSP_HAL_PT0_STATE_STOP_ADDR_MASK) == 0x14) {
		goto writeerr;
	}
	
	if((ssp_halp_p_regs->port0_state & SSP_HAL_PT0_STATE_STOP_ADDR_MASK) != 0x15) {
		goto writeerr;
	}
	ssp_hal_i2c_wait_for_free(info);    
	
	count++;
	
	writeerr:
	ssp_isr();
	
	ssp_halp_load_seq_ram1(ssp_halp_i2c_engine0_write_seq_map, SSP_HAL_SEQ_SIZE,
				(unsigned int)ssp_halp_p_pt1_ram);
	
	ssp_halp_load_seq_ram(ssp_halp_i2c_engine1_seq_map, SSP_HAL_SEQ_SIZE,
				(unsigned int)ssp_halp_p_pt0_ram);        
	
	ssp_halp_p_regs->port1_config_1 &= (~SSP_HAL_PT0_START_ADDR_MASK);
	ssp_halp_p_regs->port1_config_1 |= (0x11 & SSP_HAL_PT0_START_ADDR_MASK);                   
	
	ssp_halp_p_regs->port0_config_1 &= (~SSP_HAL_PT0_START_ADDR_MASK);
	ssp_halp_p_regs->port0_config_1 |= (0x16 & SSP_HAL_PT0_START_ADDR_MASK);
	
	ssp_hal_i2c_start();
	
	if(info->mode == SSP_HAL_MODE_POLL) {        
		for(;ssp_halp_p_regs->port1_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;); 
		for(;ssp_halp_p_regs->port0_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;);
	} else {
		ssp_wait_for_xfr_done();
	}
	return (count);
	
}
	
unsigned int ssp_hal_i2c_read(ssp_hal_i2c_info_t  *info,unsigned char *buffer,unsigned int len)
{
	unsigned int count = 0;
	unsigned int data;
	unsigned int tempVal  = 0;
	unsigned int tempVal1 = 0;
	
	
	if(!buffer) {
		return SSP_HAL_ERROR;            
	}
	
	ssp_halp_load_seq_ram1(ssp_halp_i2c_engine0_write_seq_map, SSP_HAL_SEQ_SIZE,
				(unsigned int)ssp_halp_p_pt1_ram);
	
	ssp_halp_load_seq_ram(ssp_halp_i2c_engine1_seq_map, SSP_HAL_SEQ_SIZE,
				(unsigned int)ssp_halp_p_pt0_ram);        
	
	data = (info->addr << 2);
	
	data |= SSP_HAL_I2C_READ_ENABLE;
	
	ssp_hal_update_seq_ram((unsigned int)ssp_halp_p_pt1_ram, data, SSP_HAL_READ_ADDR_WRITE);  
	
	ssp_halp_p_regs->port0_config_1 &= (~SSP_HAL_PT0_START_ADDR_MASK);
	ssp_halp_p_regs->port0_config_1 |= (0x01 & SSP_HAL_PT0_START_ADDR_MASK);
	
	ssp_halp_p_regs->port1_config_1 &= (~SSP_HAL_PT0_START_ADDR_MASK);
	ssp_halp_p_regs->port1_config_1 |= (0x03 & SSP_HAL_PT0_START_ADDR_MASK);
	
	ssp_halp_p_regs->port0_config_1 |= SSP_HAL_PT0_CLEAR_DATA_REG_MASK;
	
	ssp_hal_i2c_start();
	
	if(info->mode == SSP_HAL_MODE_POLL) {        
		for(;ssp_halp_p_regs->port1_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;);
		for(;ssp_halp_p_regs->port0_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;);
	} else {
		ssp_wait_for_xfr_done();
	}
	
	if((ssp_halp_p_regs->port0_state & SSP_HAL_PT0_STATE_STOP_ADDR_MASK) != 0x15) {
		goto readerr;
	}
	ssp_isr();
	tempVal = (ssp_halp_p_regs->port1_data & 0x01);
	
	ssp_hal_i2c_wait_for_free(info);
	ssp_halp_load_seq_ram1(ssp_halp_i2c_engine0_read_seq_map, SSP_HAL_SEQ_SIZE,
				(unsigned int)ssp_halp_p_pt1_ram);
	
	ssp_halp_load_seq_ram(ssp_halp_i2c_engine1_seq_map, SSP_HAL_SEQ_SIZE,
				(unsigned int)ssp_halp_p_pt0_ram);        
	ssp_halp_p_regs->port0_config_1 |= SSP_HAL_PT0_CLEAR_DATA_REG_MASK;
	
	for(count = 0; count < (len - 1); count++) {
		/* Make the sequence map start at the right address */
		ssp_halp_p_regs->port0_config_1 &= (~SSP_HAL_PT0_START_ADDR_MASK);
		ssp_halp_p_regs->port0_config_1 |= (0x04 & SSP_HAL_PT0_START_ADDR_MASK);
	
		ssp_halp_p_regs->port1_config_1 &= (~SSP_HAL_PT0_START_ADDR_MASK);
		ssp_halp_p_regs->port1_config_1 |= (0x02 & SSP_HAL_PT0_START_ADDR_MASK);
	
		ssp_hal_i2c_start();
		
		if(info->mode == SSP_HAL_MODE_POLL) {
			for(;ssp_halp_p_regs->port0_config_1&SSP_HAL_PT0_STATE_BUSY_MASK;);
			for(;ssp_halp_p_regs->port1_config_1&SSP_HAL_PT0_STATE_BUSY_MASK;);
		} else {
			ssp_wait_for_xfr_done();
		}
	
		if((ssp_halp_p_regs->port0_state & SSP_HAL_PT0_STATE_STOP_ADDR_MASK) != 0x15) {
			goto readerr;              
		}
		ssp_hal_i2c_wait_for_free(info);
	
		tempVal1 = (ssp_halp_p_regs->port1_data);
	
		buffer[count] = ((tempVal1 >> 1) & SSP_HAL_I2C_SERIAL_DATA_MASK) | (tempVal << 7); 
	
		tempVal = tempVal1 & 0x01;
	
		ssp_halp_p_regs->port0_config_1 |= SSP_HAL_PT0_CLEAR_DATA_REG_MASK;
	}    
	
	ssp_hal_update_seq_ram((unsigned int)ssp_halp_p_pt1_ram, data, SSP_HAL_READ_DATA_LAST);  
	ssp_halp_p_regs->port0_config_1 &= (~SSP_HAL_PT0_START_ADDR_MASK);
	ssp_halp_p_regs->port0_config_1 |= (0x06 & SSP_HAL_PT0_START_ADDR_MASK);
	
	ssp_halp_p_regs->port1_config_1 &= (~SSP_HAL_PT0_START_ADDR_MASK);
	ssp_halp_p_regs->port1_config_1 |= (0x02 & SSP_HAL_PT0_START_ADDR_MASK);
	
	ssp_hal_i2c_start();
	ssp_isr();
		
	if(info->mode == SSP_HAL_MODE_POLL) {
		for(;ssp_halp_p_regs->port0_config_1&SSP_HAL_PT0_STATE_BUSY_MASK;);
		for(;ssp_halp_p_regs->port1_config_1&SSP_HAL_PT0_STATE_BUSY_MASK;);
	} else {
		ssp_wait_for_xfr_done();
	}
	
	if((ssp_halp_p_regs->port0_state & SSP_HAL_PT0_STATE_STOP_ADDR_MASK) != 0x15) {
		goto readerr;              
	}
	
	ssp_hal_i2c_wait_for_free(info);
	
	buffer[count] = (ssp_halp_p_regs->port1_data & SSP_HAL_I2C_SERIAL_DATA_MASK) | (tempVal << 7); 
	
	count++;
	
	ssp_halp_p_regs->port0_config_1 |= SSP_HAL_PT0_CLEAR_DATA_REG_MASK;
	
	readerr:   
	ssp_isr();
	ssp_halp_load_seq_ram1(ssp_halp_i2c_engine0_write_seq_map,SSP_HAL_SEQ_SIZE,
				(unsigned int)ssp_halp_p_pt1_ram);
	
	ssp_halp_load_seq_ram(ssp_halp_i2c_engine1_seq_map,SSP_HAL_SEQ_SIZE,
				(unsigned int)ssp_halp_p_pt0_ram);        
	
	ssp_halp_p_regs->port1_config_1 &= (~SSP_HAL_PT0_START_ADDR_MASK);
	ssp_halp_p_regs->port1_config_1 |= (0x11 & SSP_HAL_PT0_START_ADDR_MASK);                   
	ssp_halp_p_regs->port0_config_1 &= (~SSP_HAL_PT0_START_ADDR_MASK);
	ssp_halp_p_regs->port0_config_1 |= (0x16 & SSP_HAL_PT0_START_ADDR_MASK);
	
	ssp_hal_i2c_start();
	
	if(info->mode == SSP_HAL_MODE_POLL) {        
		for(;ssp_halp_p_regs->port1_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;);
		for(;ssp_halp_p_regs->port0_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;);
	} else {
		ssp_wait_for_xfr_done();
	}
	
	return count;
}
	
ssp_hal_spi_info_t* ssp_hal_spi_open(ssp_hal_spi_info_t info)
{
	unsigned int pre_div;
	
	ssp_hal_spi_info_t* ret;
	
	ssp_halp_p_regs->iosel_control_1 = SSP_HAL_IOSEL_PT0_USE_EXTENDED_RAM;
	ssp_halp_p_regs->iosel_control_2 = 0;
	
	ssp_halp_p_regs->iosel_control_1 |= (0x1 << (3*info.data_out_pin)) ;
	ssp_halp_p_regs->iosel_control_2 |= info.data_in_pin;
	
	ssp_halp_p_regs->iosel_control_1 |= (0x2 << (3*info.clock_pin)) ;
	
	ssp_halp_p_regs->iosel_control_1 |= (0x3 << (3*info.cs_pin)) ;
	
	ssp_halp_p_regs->port0_config_2 &= ~0xF;
	
	pre_div = ssp_halp_obj.input_freq/(info.clock_speed * 2) + 
			((ssp_halp_obj.input_freq % (info.clock_speed * 2)) != 0);
	
	ssp_halp_p_regs->clk_pre_divider = (pre_div == 0)? 0: (pre_div - 1);  
	
	ssp_halp_p_regs->port0_config_1 &= ~SSP_HAL_PT0_DELAY_DATA_OUT_MASK; 
	
	ssp_halp_p_regs->port0_config_1 |= SSP_HAL_PT0_EARLY_DATA_IN_MASK;
	
	if(info.mode == SSP_HAL_MODE_INTR) {
		ssp_halp_p_regs->interrupt_enable |= SSP_HAL_PORT0_INT_ENABLE;
	} else {
		ssp_halp_p_regs->interrupt_enable &= ~SSP_HAL_PORT0_INT_ENABLE;
	}
	
	if(avalanche_memalloc(sizeof(ssp_hal_spi_info_t), ( void *)&ret) != 0) {
		return NULL;    
	}
	
	ret->data_in_pin  = info.data_in_pin;
	ret->data_out_pin = info.data_out_pin;
	ret->clock_pin    = info.clock_pin;
	ret->cs_pin       = info.cs_pin;
	ret->clock_speed  = ssp_halp_obj.input_freq/
				(((ssp_halp_p_regs->port0_config_2 & 0xF) + 1)*2);
	ret->clock_speed  = ret->clock_speed/(ssp_halp_p_regs->clk_pre_divider + 1);
	ret->mode         = info.mode;
	
	return ret;
}
	
unsigned int ssp_hal_spi_close(ssp_hal_spi_info_t* info)
{
	avalanche_memfree(( void *)info);
	return SSP_HAL_OK;
}
	
unsigned int ssp_hal_spi_write_read (ssp_hal_spi_info_t *info,unsigned char *write_buf,unsigned int write_len,unsigned char *read_buf,unsigned int read_len)
{
	unsigned int count = 0, ret;
	
	if((!read_buf) && (!write_buf)) {
		return SSP_HAL_ERROR;            
	}
	
	ssp_halp_p_regs->port0_config_1 &= (~SSP_HAL_PT0_START_ADDR_MASK);
	
	if((write_buf) && (write_len > 0)) {
		ssp_halp_load_seq_ram(ssp_halp_spi_write_seq_map, 2, 
			(unsigned int)ssp_halp_p_pt0_ram);
	
		for(count = 0; count < write_len; count++) {
			ssp_halp_p_regs->port0_address = (write_buf[count] & 0xFF) <<  8;
	
			for(;ssp_halp_p_regs->port0_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;);
	
			ssp_halp_p_regs->port0_config_1|=SSP_HAL_PT0_START_SERIAL_TRANSFER;
	
			if(info->mode == SSP_HAL_MODE_POLL) {
				for(;ssp_halp_p_regs->port0_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;);
			} else {	
				ssp_wait_for_xfr_done();
			}        
		}                    
	}	 
	
	ret = count;
	ssp_isr();
	
	if((read_buf) && (read_len > 0)) {
		ssp_halp_load_seq_ram(ssp_halp_spi_read_seq_map, 2, 
				(unsigned int)ssp_halp_p_pt0_ram);
		for(count = 0; count < read_len; count++) {
			ssp_halp_p_regs->port0_address = 0;
	
			for(;ssp_halp_p_regs->port0_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;);
			ssp_halp_p_regs->port0_config_1|=SSP_HAL_PT0_START_SERIAL_TRANSFER;
	
			if(info->mode == SSP_HAL_MODE_POLL) {
				for(;ssp_halp_p_regs->port0_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;);
			} else {
				ssp_wait_for_xfr_done();
			}        
	
			read_buf[count] = ssp_halp_p_regs->port0_address &= 
					SSP_HAL_SPI_SERIAL_DATA_MASK;
		}
	}        
	
	ret += count;
	ssp_isr();
	
	ssp_halp_load_seq_ram(ssp_halp_spi_stop_seq_map, 1, (unsigned int)ssp_halp_p_pt0_ram);
	
	for(;ssp_halp_p_regs->port0_config_1 & SSP_HAL_PT0_STATE_BUSY_MASK;);    
	ssp_halp_p_regs->port0_config_1 |= SSP_HAL_PT0_START_SERIAL_TRANSFER;
	
	if(info->mode == SSP_HAL_MODE_POLL) {
		for(;ssp_halp_p_regs->port0_config_1&SSP_HAL_PT0_STATE_BUSY_MASK;);
	} else {
		ssp_wait_for_xfr_done();
	}
	
	return ret;
}
	
static unsigned int ssp_halp_load_seq_ram(unsigned int *p_seq_map,unsigned int map_size,unsigned int start_address)
{
	volatile unsigned int *p_ssp_ram = (volatile unsigned int*)start_address; 
	unsigned int  i ; 
	
	ssp_halp_p_regs->port0_config_2 |= SSP_HAL_PT0_RAM_WRITE_ENABLE;
	for(i = 0 ; i < map_size; i++)
	{
		p_ssp_ram[i]  = p_seq_map[i];
	}
	
	ssp_halp_p_regs->port0_config_2 &= ~SSP_HAL_PT0_RAM_WRITE_ENABLE;
	
	return SSP_HAL_OK;
}
	
static unsigned int ssp_halp_load_seq_ram1(unsigned int *p_seq_map,unsigned int      map_size,unsigned int start_address)
{
	volatile unsigned int *p_ssp_ram = (volatile unsigned int*)start_address; 
	unsigned int  i ; 
	
	ssp_halp_p_regs->port1_config_2 |= SSP_HAL_PT0_RAM_WRITE_ENABLE;
	
	for(i = 0 ; i < map_size; i++)
	{
		p_ssp_ram[i]  = p_seq_map[i];
	}
	
	ssp_halp_p_regs->port1_config_2 &= ~SSP_HAL_PT0_RAM_WRITE_ENABLE;
	
	return SSP_HAL_OK;
}
	
unsigned int ssp_hal_intr_ctrl(unsigned int action)
{
	switch (action) {
	case SSP_HAL_INTR_ACK:
		ssp_halp_p_regs->interrupt_status |= 0x3;
		break;
	case SSP_HAL_INTR_DISABLE:
		ssp_halp_p_regs->interrupt_enable &= ~SSP_HAL_PORT1_INT_ENABLE;
		break;
	case SSP_HAL_INTR_ENABLE:
		ssp_halp_p_regs->interrupt_enable |= SSP_HAL_PORT1_INT_ENABLE;
		break;
	default:
		return SSP_HAL_ERROR;
	}

	return SSP_HAL_OK;
}
#define SSP_HAL_CLK_HIGH                                 (1 << 0)
#define SSP_HAL_CLK_LOW                                  (0 << 0)
#define SSP_HAL_DATA_HIGH                                (1 << 1)
#define SSP_HAL_DATA_LOW                                 (0 << 1)
#define SSP_HAL_CS_HIGH                                  (1 << 2)
#define SSP_HAL_CS_LOW                                   (0 << 2)
#define SSP_HAL_INPUT_MODE                               (0 << 3)                        
#define SSP_HAL_OUTPUT_MODE                              (1 << 3)                        
#define SSP_HAL_DATA_REG                                 (1 << 4)
#define SSP_HAL_ADDR_REG                                 (0 << 4)
#define SSP_HAL_OPCODE_DIRECT                            ((0x0) << 5)
#define SSP_HAL_OPCODE_TOGGLE                            ((0x1) << 5)
#define SSP_HAL_OPCODE_SHIFT                             ((0x2) << 5)
#define SSP_HAL_OPCODE_BRANCH0                           ((0x4) << 5)
#define SSP_HAL_OPCODE_BRANCH1                           ((0x5) << 5)
#define SSP_HAL_OPCODE_BRANCH                            ((0x6) << 5)
#define SSP_HAL_OPCODE_STOP                              ((0x7) << 5)
#define SSP_HAL_NOT_APPLICABLE                           0
	
#define SSP_HAL_BRANCH_ADDR_BIT_POS                      8
#define SSP_HAL_CNT_VALUE_BIT_POS                        8
#define SSP_HAL_BRANCH(addr)             ((addr)<<SSP_HAL_BRANCH_ADDR_BIT_POS)
#define SSP_HAL_COUNT(cycles)            ((cycles)<<SSP_HAL_CNT_VALUE_BIT_POS)
	
unsigned int ssp_halp_spi_write_seq_map[] = 
{
	(SSP_HAL_COUNT(15)      |SSP_HAL_OPCODE_SHIFT        |SSP_HAL_ADDR_REG
		|SSP_HAL_OUTPUT_MODE         |SSP_HAL_CS_LOW
		|SSP_HAL_NOT_APPLICABLE      |SSP_HAL_NOT_APPLICABLE
	), 
	
	(SSP_HAL_NOT_APPLICABLE |SSP_HAL_OPCODE_STOP         |SSP_HAL_NOT_APPLICABLE
		|SSP_HAL_OUTPUT_MODE         |SSP_HAL_CS_LOW
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW
	),
		0,0,0,0,0,0,0
};
	
unsigned int ssp_halp_spi_read_seq_map[] = 
	{
	(SSP_HAL_COUNT(15)      |SSP_HAL_OPCODE_SHIFT        |SSP_HAL_ADDR_REG
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW
		|SSP_HAL_NOT_APPLICABLE      |SSP_HAL_NOT_APPLICABLE
	),  
	
	(SSP_HAL_NOT_APPLICABLE |SSP_HAL_OPCODE_STOP         |SSP_HAL_NOT_APPLICABLE
		|SSP_HAL_OUTPUT_MODE         |SSP_HAL_CS_LOW
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW
	),
	0,0,0,0,0,0,0
};
	
unsigned int ssp_halp_spi_stop_seq_map[] = 
{
	(SSP_HAL_NOT_APPLICABLE |SSP_HAL_OPCODE_STOP         |SSP_HAL_NOT_APPLICABLE
		|SSP_HAL_OUTPUT_MODE         |SSP_HAL_CS_HIGH
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW
	),
	0,0,0,0,0,0
}; 
unsigned int ssp_halp_i2c_engine0_write_seq_map[] ={
	
/* 0x00 I2C pause */
	( (0x00 << 8)           |SSP_HAL_OPCODE_STOP      |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_OUTPUT_MODE      |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH        |SSP_HAL_CLK_LOW       
	),   
	
	/* 0x01 I2C pause */
	( (0x00 << 8)           |SSP_HAL_OPCODE_STOP      |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_OUTPUT_MODE      |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH        |SSP_HAL_CLK_LOW       
	),   
	
	/* 0x02 Initial state */
	( (0x18 << 8)           |SSP_HAL_OPCODE_TOGGLE    |SSP_HAL_ADDR_REG   
		|SSP_HAL_OUTPUT_MODE      |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH        |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x03 I2C start */
	( (0x31 << 8)           |SSP_HAL_OPCODE_TOGGLE    |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE      |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW         |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x04 I2C data */
	( (0x31 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x05 I2C data */
	( (0x31 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x06 I2C data */
	( (0x31 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x07 I2C data */
	( (0x31 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x08 I2C data */
	( (0x31 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x09 I2C data */
	( (0x31 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x0a I2C data */
	( (0x31 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x0b I2C data */
	( (0x31 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x0c I2C data */
	( (0x17 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x0d Branch on no Ack */
	( (0x14 <<8)            |SSP_HAL_OPCODE_BRANCH1       |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_INPUT_MODE           |SSP_HAL_CS_LOW 
		|SSP_HAL_NOT_APPLICABLE       |SSP_HAL_CLK_HIGH     
	),
	
	/* 0x0e I2C data */
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x0f I2C data */
	( (0x31 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x10 Pause with Ack */
	( (0x00 <<8)            |SSP_HAL_OPCODE_STOP          |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW             |SSP_HAL_CLK_LOW     
	),
	
	/* 0x11 I2C data */
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x12 I2C data */
	( (0x31 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x13 Pause with Ack */
	( (0x00 <<8)            |SSP_HAL_OPCODE_STOP          |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW     
	),
	
	/* 0x14 I2C pause */
	( (0x00 << 8)           |SSP_HAL_OPCODE_STOP      |SSP_HAL_NOT_APPLICABLE
		|SSP_HAL_OUTPUT_MODE      |SSP_HAL_CS_LOW
		|SSP_HAL_DATA_HIGH        |SSP_HAL_CLK_LOW
	)
	
	
	};
	
	unsigned int ssp_halp_i2c_engine0_read_seq_map[] ={
	
	/* 0x00 I2C pause */
	( (0x00 <<8)            |SSP_HAL_OPCODE_STOP         |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),   
	
	/* 0x01 I2C pause */
	( (0x00 <<8)            |SSP_HAL_OPCODE_STOP         |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),   
	
	/* 0x02 i2c start */
	( (0x1a <<8)            |SSP_HAL_OPCODE_TOGGLE       |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW      
	),  
	
	/* 0x03 write device address byte */
	( (0x01 <<8)            |SSP_HAL_OPCODE_SHIFT        |SSP_HAL_DATA_REG 
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_NOT_APPLICABLE      |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x04 i2c start */
	( (0x30 <<8)            |SSP_HAL_OPCODE_TOGGLE       |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW      
	),  
	
	/* 0x05 write device address byte */
	( (0x01 <<8)            |SSP_HAL_OPCODE_SHIFT        |SSP_HAL_DATA_REG 
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_NOT_APPLICABLE      |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x06 i2c start */
	( (0x30 <<8)            |SSP_HAL_OPCODE_TOGGLE       |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW      
	),  
	
	/* 0x07 write device address byte */
	( (0x01 <<8)            |SSP_HAL_OPCODE_SHIFT        |SSP_HAL_DATA_REG 
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_NOT_APPLICABLE      |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x08 i2c start */
	( (0x30 <<8)            |SSP_HAL_OPCODE_TOGGLE       |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW      
	),  
	
	/* 0x09 write device address byte */
	( (0x01 <<8)            |SSP_HAL_OPCODE_SHIFT        |SSP_HAL_DATA_REG 
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_NOT_APPLICABLE      |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x0a i2c start */
	( (0x30 <<8)            |SSP_HAL_OPCODE_TOGGLE       |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW      
	),  
	
	/* 0x0b write device address byte */
	( (0x01 <<8)            |SSP_HAL_OPCODE_SHIFT        |SSP_HAL_DATA_REG 
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_NOT_APPLICABLE      |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x0c i2c start */
	( (0x30 <<8)            |SSP_HAL_OPCODE_TOGGLE       |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW      
	),  
	
	/* 0x0d write device address byte */
	( (0x01 <<8)            |SSP_HAL_OPCODE_SHIFT        |SSP_HAL_DATA_REG 
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_NOT_APPLICABLE      |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x0e i2c start */
	( (0x30 <<8)            |SSP_HAL_OPCODE_TOGGLE       |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW      
	),  
	
	/* 0x0f write device address byte */
	( (0x01 <<8)            |SSP_HAL_OPCODE_SHIFT        |SSP_HAL_DATA_REG 
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_NOT_APPLICABLE      |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x10 i2c start */
	( (0x14 <<8)            |SSP_HAL_OPCODE_TOGGLE       |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW      
	),  
	
	/* 0x11 i2c start */
	( (0x31 <<8)            |SSP_HAL_OPCODE_TOGGLE       |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW      
	),
	
	/* 0x12 i2c start */
	( (0x14 <<8)            |SSP_HAL_OPCODE_TOGGLE       |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW      
	),  
	
	/* 0x13 write device address byte */
	( (0x01 <<8)            |SSP_HAL_OPCODE_SHIFT        |SSP_HAL_DATA_REG 
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_NOT_APPLICABLE      |SSP_HAL_CLK_LOW       
	),      
	
	/* 0x14 i2c start */
	( (0x25 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_INPUT_MODE           |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW
	),
	
	/* 0x15 i2c start */                                    
	( (0x00 <<8)            |SSP_HAL_OPCODE_STOP          |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_INPUT_MODE           |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH             |SSP_HAL_CLK_LOW
	),                                                
	
	/* 0x16 i2c BR to stop */
	( (0x00 <<8)            |SSP_HAL_OPCODE_DIRECT       |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_OUTPUT_MODE         |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	)
	};
	
	unsigned int ssp_halp_i2c_engine1_seq_map[] ={
	
	/* 0x00 I2C pause */
	( (0x00 <<8)            |SSP_HAL_OPCODE_DIRECT        |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),   
	
	/* 0x01 I2C start */
	( (0x31 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG  
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x02 toggle */
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW             |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x03 toggle */    
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x04 toggle */
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x05 toggle */    
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x06 toggle */
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x07 toggle */    
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x08 toggle */
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x09 toggle */    
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x0a toggle */
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x0b toggle */    
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x0c toggle */
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x0d toggle */    
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x0e toggle */
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x0f toggle */    
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x10 toggle */
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x11 toggle */    
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x12 toggle */
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x13 toggle */    
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x14 toggle */
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x15 normal stop  */
	( (0x00 <<8)            |SSP_HAL_OPCODE_STOP         |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_INPUT_MODE         |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH           |SSP_HAL_CLK_HIGH      
	),
	
	/* 0x16 I2C data */
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x17 I2C data */
	( (0x31 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x18 Pause with Ack */
	( (0x00 <<8)            |SSP_HAL_OPCODE_STOP          |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_OUTPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW     
	),
	
	
	/* 0x19 I2C data */
	( (0x18 <<8)            |SSP_HAL_OPCODE_TOGGLE        |SSP_HAL_ADDR_REG 
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_LOW            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x1a branch */
	( (0x19 <<8)            |SSP_HAL_OPCODE_BRANCH0        |SSP_HAL_ADDR_REG 
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW       
	),  
	
	/* 0x1b Pause */
	( (0x00 <<8)            |SSP_HAL_OPCODE_STOP          |SSP_HAL_NOT_APPLICABLE     
		|SSP_HAL_INPUT_MODE          |SSP_HAL_CS_LOW 
		|SSP_HAL_DATA_HIGH            |SSP_HAL_CLK_LOW     
	)
	
	};
	
