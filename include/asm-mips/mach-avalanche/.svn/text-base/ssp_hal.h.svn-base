#ifndef __SSP_HAL__
#define __SSP_HAL__

#define SSP_HAL_OK                      0
#define SSP_HAL_ERROR                  -1
#define SSP_HAL_MODE_POLL               0
#define SSP_HAL_MODE_INTR               1
#define SSP_HAL_INTR_ACK                0
#define SSP_HAL_INTR_DISABLE            1
#define SSP_HAL_INTR_ENABLE             2

typedef struct ssp_hal_i2c_info_tag
{
    unsigned int   data_pin;  
    unsigned int   clock_pin; 
    unsigned char	 addr;      
    unsigned int	 bus_speed; 
    unsigned int   mode;      
    
}ssp_hal_i2c_info_t;

typedef struct ssp_hal_spi_info_tag
{
    unsigned int  data_in_pin;  
    unsigned int  data_out_pin; 
    unsigned int  clock_pin;   
    unsigned int  cs_pin;     
    unsigned int	clock_speed; 
    unsigned int  mode; 
    
}ssp_hal_spi_info_t;

void sspIsr (void);

unsigned int ssp_hal_init
(
    unsigned int base_address,
    unsigned int module_input_freq
);

ssp_hal_i2c_info_t* ssp_hal_i2c_open
(
    ssp_hal_i2c_info_t info
);

unsigned int ssp_hal_i2c_close
(
    ssp_hal_i2c_info_t* info
);

unsigned int ssp_hal_i2c_write 
(
    ssp_hal_i2c_info_t  *info,
    unsigned char               *buffer, 
    unsigned int              len
);

unsigned int ssp_hal_i2c_read
(
    ssp_hal_i2c_info_t  *info,
    unsigned char               *buffer,
    unsigned int              len
);

ssp_hal_spi_info_t* ssp_hal_spi_open
(
    ssp_hal_spi_info_t info
);

unsigned int ssp_hal_spi_close
(
    ssp_hal_spi_info_t* info
);

unsigned int ssp_hal_spi_write_read 
(
    ssp_hal_spi_info_t  *info,
    unsigned char               *write_buf, 
    unsigned int              write_len,
    unsigned char               *read_buf, 
    unsigned int              read_len
);

unsigned int ssp_hal_intr_ctrl
(
    unsigned int action
);

void ssp_drv_cbk_wait_for_xfr_done( void );

#define SSP_DRV_OK          SSP_HAL_OK
#define SSP_DRV_ERROR       SSP_HAL_ERROR
typedef void* ssp_drv_desc_t;
#define SSP_DRIVER_MAGIC           0xD1
#define I2C_MAX_BUF_SIZE       100
#define I2C_SET_ADDR            _IOW( SSP_DRIVER_MAGIC, 1, unsigned char )
ssp_hal_i2c_info_t* ssp_i2c_open( void );
int ssp_i2c_close(ssp_hal_i2c_info_t *     id);
int ssp_i2c_read( ssp_hal_i2c_info_t  *info, unsigned char    *buffer, unsigned int   len);
int ssp_i2c_write( ssp_hal_i2c_info_t  *info, unsigned char    *buffer, unsigned int   len);
#define SSP_I2C_OUTPUT_CLK_FREQ                 400000    /* 400 KHz */
#define SSP_SPI_OUTPUT_CLK_FREQ                 2000000   /* 2 MHz */
#define SSP_UW_OUTPUT_CLK_FREQ                  1000000   /* 1 MHz */

#endif /* ifndef  __SSP_HAL__*/
