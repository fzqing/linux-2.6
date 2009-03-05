/*
 * drivers/media/video/adcm2650.c
 *
 * ADCM 2650 Camera Module driver.
 *
 * Author: Intel Corporation
 *
 * 2003 (c) Intel Corporation
 * 2003-2005 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/types.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/dma.h>
#include <asm/irq.h>

#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include "adcm2650.h"

#define LINE_STRING __stringify(KBUILD_BASENAME) ":" __stringify(__LINE__) " : "

/* *INDENT-OFF* */
static const struct {
	u16 addr;
	u16 value;
} firm_update[] = {
{0x00fc, 0x0008},  /* 1 */
{0x00fe, 0x000c},  /* 2 */
{0x42cc, 0x0002},  /* 3 */
{0x42ce, 0xfd13},  /* 4 */
{0x4308, 0x0002},  /* 5 */
{0x430a, 0xfd2d},  /* 6 */
{0x430c, 0x0002},  /* 7 */
{0x430e, 0xfd00},  /* 8 */
{0x4400, 0x0000},  /* 9 */
{0x4402, 0x060e},  /* 10 */
{0x4404, 0x0003},  /* 11 */
{0x4406, 0x03b9},  /* 12 */
{0x4408, 0x000d},  /* 13 */
{0x440a, 0x3e80},  /* 14 */
{0x440c, 0x0000},  /* 15 */
{0x440e, 0x02ac},  /* 16 */
{0x4410, 0x0000},  /* 17 */
{0x4412, 0x5c04},  /* 18 */
{0x4414, 0x0000},  /* 19 */
{0x4416, 0x02a7},  /* 20 */
{0x4418, 0x0000},  /* 21 */
{0x441a, 0x093e},  /* 22 */
{0x441c, 0x0000},  /* 23 */
{0x441e, 0x0436},  /* 24 */
{0x4420, 0x0000},  /* 25 */
{0x4422, 0x5445},  /* 26 */
{0x4424, 0x0000},  /* 27 */
{0x4426, 0x042d},  /* 28 */
{0x4428, 0x0000},  /* 29 */
{0x442a, 0x041d},  /* 30 */
{0x442c, 0x0000},  /* 31 */
{0x442e, 0x026e},  /* 32 */
{0x4430, 0x0000},  /* 33 */
{0x4432, 0x01a6},  /* 34 */
{0x4434, 0x000d},  /* 35 */
{0x4436, 0x3300},  /* 36 */
{0x4438, 0x0000},  /* 37 */
{0x443a, 0x0175},  /* 38 */
{0x443c, 0x000e},  /* 39 */
{0x443e, 0xc350},  /* 40 */
{0x4440, 0x0007},  /* 41 */
{0x4442, 0x0681},  /* 42 */
{0x4444, 0x000e},  /* 43 */
{0x4446, 0x61a8},  /* 44 */
{0x4448, 0x0002},  /* 45 */
{0x444a, 0x0681},  /* 46 */
{0x444c, 0x0003},  /* 47 */
{0x444e, 0x01ba},  /* 48 */
{0x4450, 0x0003},  /* 49 */
{0x4452, 0x03b9},  /* 50 */
{0x4454, 0x0000},  /* 51 */
{0x4456, 0x5c04},  /* 52 */
{0x4458, 0x0000},  /* 53 */
{0x445a, 0x02bc},  /* 54 */
{0x445c, 0x000f},  /* 55 */
{0x445e, 0x010b},  /* 56 */
{0x4460, 0x0000},  /* 57 */
{0x4462, 0x0267},  /* 58 */
{0x4464, 0x0000},  /* 59 */
{0x4466, 0x01a7},  /* 60 */
{0x4468, 0x0000},  /* 61 */
{0x446a, 0x043f},  /* 62 */
{0x446c, 0x0000},  /* 63 */
{0x446e, 0x01fb},  /* 64 */
{0x4470, 0x0004},  /* 65 */
{0x4472, 0xfd1e},  /* 66 */
{0x4474, 0x0000},  /* 67 */
{0x4476, 0x0197},  /* 68 */
{0x4478, 0x0000},  /* 69 */
{0x447a, 0x5845},  /* 70 */
{0x447c, 0x000d},  /* 71 */
{0x447e, 0x3000},  /* 72 */
{0x4480, 0x0000},  /* 73 */
{0x4482, 0x02ae},  /* 74 */
{0x4484, 0x0000},  /* 75 */
{0x4486, 0x0196},  /* 76 */
{0x4488, 0x0000},  /* 77 */
{0x448a, 0x0177},  /* 78 */
{0x448c, 0x0007},  /* 79 */
{0x448e, 0xfd25},  /* 80 */
{0x4490, 0x0000},  /* 81 */
{0x4492, 0x0937},  /* 82 */
{0x4494, 0x0000},  /* 83 */
{0x4496, 0x5b30},  /* 84 */
{0x4498, 0x0000},  /* 85 */
{0x449a, 0x0416},  /* 86 */
{0x449c, 0x0000},  /* 87 */
{0x449e, 0x0436},  /* 88 */
{0x44a0, 0x0000},  /* 89 */
{0x44a2, 0x041f},  /* 90 */
{0x44a4, 0x0000},  /* 91 */
{0x44a6, 0x0417},  /* 92 */
{0x44a8, 0x0000},  /* 93 */
{0x44aa, 0x037e},  /* 94 */
{0x44ac, 0x0000},  /* 95 */
{0x44ae, 0x7b30},  /* 96 */
{0x44b0, 0x0000},  /* 97 */
{0x44b2, 0x01ff},  /* 98 */
{0x44b4, 0x0000},  /* 99 */
{0x44b6, 0x5c44},  /* 100 */
{0x44b8, 0x0000},  /* 101 */
{0x44ba, 0x5845},  /* 102 */
{0x44bc, 0x0000},  /* 103 */
{0x44be, 0x0416},  /* 104 */
{0x44c0, 0x0000},  /* 105 */
{0x44c2, 0x0277},  /* 106 */
{0x44c4, 0x0000},  /* 107 */
{0x44c6, 0x01a7},  /* 108 */
{0x44c8, 0x0000},  /* 109 */
{0x44ca, 0x043f},  /* 110 */
{0x44cc, 0x0000},  /* 111 */
{0x44ce, 0x0647},  /* 112 */
{0x44d0, 0x0004},  /* 113 */
{0x44d2, 0xfd36},  /* 114 */
{0x44d4, 0x0000},  /* 115 */
{0x44d6, 0x0427},  /* 116 */
{0x44d8, 0x0000},  /* 117 */
{0x44da, 0x5863},  /* 118 */
{0x44dc, 0x0000},  /* 119 */
{0x44de, 0x017e},  /* 120 */
{0x44e0, 0x0007},  /* 121 */
{0x44e2, 0xfd41},  /* 122 */
{0x44e4, 0x0000},  /* 123 */
{0x44e6, 0x5864},  /* 124 */
{0x44e8, 0x0000},  /* 125 */
{0x44ea, 0x017e},  /* 126 */
{0x44ec, 0x0007},  /* 127 */
{0x44ee, 0xfd41},  /* 128 */
{0x44f0, 0x0000},  /* 129 */
{0x44f2, 0x5865},  /* 130 */
{0x44f4, 0x0000},  /* 131 */
{0x44f6, 0x017e},  /* 132 */
{0x44f8, 0x0007},  /* 133 */
{0x44fa, 0xfd41},  /* 134 */
{0x44fc, 0x0000},  /* 135 */
{0x44fe, 0x0607},  /* 136 */
{0x4500, 0x0002},  /* 137 */
{0x4502, 0x05e7},  /* 138 */
{0x4504, 0x0000},  /* 139 */
{0x4506, 0x0627},  /* 140 */
{0x4508, 0x0000},  /* 141 */
{0x450a, 0x5c29},  /* 142 */
{0x450c, 0x000e},  /* 143 */
{0x450e, 0x0100},  /* 144 */
{0x4510, 0x0000},  /* 145 */
{0x4512, 0x00f7},  /* 146 */
{0x4514, 0x0000},  /* 147 */
{0x4516, 0x0407},  /* 148 */
{0x4518, 0x0000},  /* 149 */
{0x451a, 0x0077},  /* 150 */
{0x451c, 0x0002},  /* 151 */
{0x451e, 0x05ed},  /* 152 */
};
/* *INDENT-ON* */

static struct i2c_driver adcm2650_driver;
static DECLARE_MUTEX(i2c_lock);
static int block_addr = -1;

/*
 * support only one client (= exemplar of the sensor)
 * per whole system so far
 */
struct i2c_client adcm2650_client = {
	.id = -1,		/* -1 : not initialized; 0 : initialized */
	.flags = I2C_CLIENT_ALLOW_USE,
	.driver = &adcm2650_driver,
	.name = "adcm2650-0",
};

static int i2c_adcm2650_detect_client(struct i2c_adapter *adapter, int address,
				      int kind)
{
	int err;

	pr_debug(LINE_STRING "\n");

	pr_info("ADCM2650: i2c-bus:%s; address:0x%x... ",
		adapter->name, address);

	if (adcm2650_client.id != -1) {
		pr_info("the sensor is already initialized.\n");
		/*
		 * anyway, return 0 or else
		 * the detection cycle will be broken
		 */
		return 0;
	}

	/* returns true if the functionality is supported */
	err = i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA);
	if (!err) {
		pr_info("word operations is not permited.\n");
		return 0;
	}

	adcm2650_client.addr = address;
	adcm2650_client.adapter = adapter;

	/*
	 * don't pay attention to kind argument --
	 * we make the lists of addresses ourselves
	 */
	err = i2c_smbus_read_word_data(&adcm2650_client, ADCM2650_REV);
	if (err != 0x0600) {
		pr_info("failed.\n");
		return 0;
	} else {
		pr_info("detected.\n");
	}

	adcm2650_client.id = 0;

	err = i2c_attach_client(&adcm2650_client);
	if (err) {
		pr_debug(LINE_STRING "i2c_attach_client()\n");
		adcm2650_client.id = -1;
	}

	return 0;
}

static unsigned short normal_i2c[] = { 0x52, I2C_CLIENT_END };
static unsigned short normal_i2c_range[] = { I2C_CLIENT_END };
static unsigned short probe[2] = { I2C_CLIENT_END, I2C_CLIENT_END };
static unsigned short probe_range[2] = { I2C_CLIENT_END, I2C_CLIENT_END };
static unsigned short ignore[2] = { I2C_CLIENT_END, I2C_CLIENT_END };
static unsigned short ignore_range[2] = { I2C_CLIENT_END, I2C_CLIENT_END };
static unsigned short force[2] = { I2C_CLIENT_END, I2C_CLIENT_END };

static struct i2c_client_address_data addr_data = {
	.normal_i2c = normal_i2c,
	.normal_i2c_range = normal_i2c_range,
	.probe = probe,
	.probe_range = probe_range,
	.ignore = ignore,
	.ignore_range = ignore_range,
	.force = force,
};

static int i2c_adcm2650_probe(struct i2c_adapter *adap)
{
	pr_debug(LINE_STRING "\n");
	return i2c_probe(adap, &addr_data, i2c_adcm2650_detect_client);
}

static int i2c_adcm2650_detach(struct i2c_client *client)
{
	int err;

	pr_debug(LINE_STRING "\n");

	err = i2c_detach_client(client);
	if (err) {
		pr_debug(LINE_STRING "i2c_detach_client()\n");
		return err;
	}

	return 0;
}

#define I2C_DRIVERID_ADCM2650   I2C_DRIVERID_EXP1

static struct i2c_driver adcm2650_driver = {
	.owner = THIS_MODULE,
	.name = "adcm2650",
	.id = I2C_DRIVERID_ADCM2650,
	.flags = I2C_DF_NOTIFY,
	.attach_adapter = &i2c_adcm2650_probe,
	.detach_client = &i2c_adcm2650_detach,
};

int adcm2650_init(void)
{
	pr_debug(LINE_STRING "\n");

	return i2c_add_driver(&adcm2650_driver);
}

void adcm2650_exit(void)
{
	pr_debug(LINE_STRING "\n");
	i2c_del_driver(&adcm2650_driver);
}

/* i2c pipeline access */

static int change_block_address(u8 new_block_addr)
{
	int err;

	if (adcm2650_client.id != 0) {
		pr_debug(LINE_STRING "not initialized\n");
		return -ENODEV;
	}

	if (block_addr == (int)new_block_addr) {
		return 0;
	}

	err =
	    i2c_smbus_write_byte_data(&adcm2650_client,
				      ADCM2650_BLOCK_SWITCH_CMD,
				      new_block_addr);
	if (err) {
		pr_debug(LINE_STRING "couldn't change block address\n");
		return -EIO;
	}

	block_addr = (int)new_block_addr;
	return 0;
}

int adcm2650_pipeline_read(u16 addr, u16 * pvalue)
{
	int err = 0;
	u8 block = ADCM2650_BLOCK(addr);
	u8 offset = ADCM2650_OFFSET(addr);

	if (adcm2650_client.id != 0)
		return -ENODEV;

	down(&i2c_lock);

	err = change_block_address(block);
	if (err) {
		up(&i2c_lock);
		pr_debug(LINE_STRING "change_block_address(%d)\n", (int)block);
		return err;
	}

	err = i2c_smbus_read_word_data(&adcm2650_client, offset);
	if (err < 0) {
		up(&i2c_lock);
		pr_debug(LINE_STRING "i2c_smbus_read_word_data(%d:%d)\n",
			 (int)offset, (int)block);
		return -EIO;
	}
	*pvalue = (u16) err;

	up(&i2c_lock);

	return 0;
}

int adcm2650_pipeline_write(u16 addr, u16 value)
{
	int err;
	u8 block = ADCM2650_BLOCK(addr);
	u8 offset = ADCM2650_OFFSET(addr);

	if (adcm2650_client.id != 0)
		return -ENODEV;

	down(&i2c_lock);

	err = change_block_address(block);
	if (err) {
		up(&i2c_lock);
		pr_debug(LINE_STRING "change_block_address(%d)\n", (int)block);
		return err;
	}

	err = i2c_smbus_write_word_data(&adcm2650_client, offset, value);
	if (err) {
		up(&i2c_lock);
		pr_debug(LINE_STRING "i2c_smbus_write_word_data(%d:%d) : %d\n",
			 (int)offset, (int)block, err);
		return -EIO;
	}

	up(&i2c_lock);

	return err;
}

int adcm2650_pipeline_read_rl(u16 reg_addr, u16 mask, u16 field)
{
	u16 reg_value;
	int retry = ADCM2650_TIMEOUT;
	int err;

	while (retry--) {

		err = adcm2650_pipeline_read(reg_addr, &reg_value);
		if (err) {
			pr_debug(LINE_STRING "adcm2650_pipeline_read()\n");
			return err;
		}
		if ((reg_value & mask) == field) {
			return 0;
		}

		msleep(1);
	}

	return -ETIME;

}

int adcm2650_pipeline_write_wa(u16 reg_addr, u16 mask)
{
	u16 reg_value;
	int err;

	err = adcm2650_pipeline_read(reg_addr, &reg_value);
	if (err) {
		pr_debug(LINE_STRING "adcm2650_pipeline_read()\n");
		return err;
	}
	reg_value &= mask;
	err = adcm2650_pipeline_write(reg_addr, reg_value);
	if (err) {
		pr_debug(LINE_STRING "adcm2650_pipeline_write()\n");
		return err;
	}

	return 0;
}

int adcm2650_pipeline_write_wo(u16 reg_addr, u16 field)
{
	u16 reg_value;
	int err;

	err = adcm2650_pipeline_read(reg_addr, &reg_value);
	if (err) {
		pr_debug(LINE_STRING "adcm2650_pipeline_read()\n");
		return err;
	}
	reg_value |= field;
	err = adcm2650_pipeline_write(reg_addr, reg_value);
	if (err) {
		pr_debug(LINE_STRING "adcm2650_pipeline_write()\n");
		return err;
	}

	return 0;
}

/* sensor access */
int adcm2650_sensor_read_rs(u8 reg_addr, u8 * reg_value)
{
	u16 value;
	int retry = ADCM2650_TIMEOUT;
	int err;

	/* Loads SENSOR_ADDRESS (0x004a) with address 0x55 and register */
	/* address for a read operation. */
	value = ADCM2650_SENSOR_SLAVE_ADDR | (reg_addr << 8);
	err = adcm2650_pipeline_write(ADCM2650_SENSOR_ADDRESS, value);
	if (err) {
		pr_debug(LINE_STRING "adcm2650_pipeline_write()\n");
		return err;
	}

	/* Loads SENSOR_CTRL (0x0050) with a read request */
	value = ADCM2650_SENSOR_CTRL_RW | ADCM2650_SENSOR_CTRL_GO;
	err = adcm2650_pipeline_write(ADCM2650_SENSOR_CTRL, value);
	if (err) {
		pr_debug(LINE_STRING "adcm2650_pipeline_write()\n");
		return err;
	}

	/* Checks if the request is complete and if data is available to read */
	while (--retry) {
		/* Reads the SENSOR_CTRL (0x0050) address for status */
		value = 0xFF;
		err = adcm2650_pipeline_read(ADCM2650_SENSOR_CTRL, &value);
		if (err) {
			pr_debug(LINE_STRING "adcm2650_pipeline_read()\n");
			return err;
		}

		if (!(value & ADCM2650_SENSOR_CTRL_GO)) {
			err =
			    adcm2650_pipeline_read(ADCM2650_SENSOR_DATA_1,
						   &value);
			if (err) {
				pr_debug(LINE_STRING
					 "adcm2650_pipeline_read()\n");
				return err;
			}
			*reg_value = value & 0xFF;
			return 0;
		}
		msleep(1);
	}

	pr_debug(LINE_STRING "timeout\n");
	return -ETIME;
}

int adcm2650_sensor_write_ws(u8 reg_addr, u8 reg_value)
{
	u16 value;
	int retry = ADCM2650_TIMEOUT;
	int err;

	/* Loads SENSOR_ADDRESS (0x004a) with address 0x55 and register */
	/* a write operation */
	value = ADCM2650_SENSOR_SLAVE_ADDR | (reg_addr << 8);
	err = adcm2650_pipeline_write(ADCM2650_SENSOR_ADDRESS, reg_value);
	if (err) {
		pr_debug(LINE_STRING "adcm2650_pipeline_write()\n");
		return err;
	}

	/* Loads SENSOR_DATA0 (0x004c) */
	value = reg_value;
	err = adcm2650_pipeline_write(ADCM2650_SENSOR_DATA_1, reg_value);
	if (err) {
		pr_debug(LINE_STRING "adcm2650_pipeline_write()\n");
		return err;
	}

	/* Loads SENSOR_CTRL (0x0050) with a write request */
	value = ADCM2650_SENSOR_CTRL_GO;
	/* write one byte */
	err = adcm2650_pipeline_write(ADCM2650_SENSOR_CTRL, value | 0x1);
	if (err) {
		pr_debug(LINE_STRING "adcm2650_pipeline_write()\n");
		return err;
	}

	while (--retry) {
		value = 0xFF;
		err = adcm2650_pipeline_read(ADCM2650_SENSOR_CTRL, &value);
		if (err) {
			pr_debug(LINE_STRING "adcm2650_pipeline_read()\n");
			return err;
		}

		if (!(value & ADCM2650_SENSOR_CTRL_GO)) {
			return 0;
		}
		msleep(1);
	}

	pr_debug(LINE_STRING "timeout\n");
	return -ETIME;
}

/* helper functions */

int adcm2650_set_output_format(unsigned int output_format)
{
	int err;

	/* -- adcm2650_viewfinder_cfg_output -- */
	err =
	    adcm2650_pipeline_write_wa(ADCM2650_VIDEO_CONFIG,
				       ~ADCM2650_VIDEO_CONFIG_O_FORMAT_MASK);
	if (err)
		return err;
	err |=
	    adcm2650_pipeline_write_wo(ADCM2650_VIDEO_CONFIG,
				       output_format <<
				       ADCM2650_VIDEO_CONFIG_O_FORMAT_SHIFT);
	if (err)
		return err;

	/* -- adcm2650_stillframe_cfg_output -- */
	err =
	    adcm2650_pipeline_write_wa(ADCM2650_STILL_CONFIG,
				       ~ADCM2650_STILL_CONFIG_O_FORMAT_MASK);
	if (err)
		return err;
	err =
	    adcm2650_pipeline_write_wo(ADCM2650_STILL_CONFIG,
				       output_format <<
				       ADCM2650_STILL_CONFIG_O_FORMAT_SHIFT);
	if (err)
		return err;

	return 0;
}

int adcm2650_firmware_upgrade(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(firm_update); i++) {
		adcm2650_pipeline_write(firm_update[i].addr,
					firm_update[i].value);
	}
	return 0;
}

int adcm2650_switch_to_normal(void)
{
	int err;

	/* Switches to Normal output mode */
	err =
	    adcm2650_pipeline_write_wa(ADCM2650_STILL_CONFIG,
				       ~ADCM2650_STILL_CONFIG_JPEG_MASK);
	if (err)
		return err;

	/* Changes still frame output mode to NORMAL */
	err =
	    adcm2650_pipeline_write_wa(ADCM2650_VIDEO_CONFIG,
				       ~ADCM2650_VIDEO_CONFIG_JPEG_MASK);
	if (err)
		return err;

	return 0;
}
