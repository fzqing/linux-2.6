/*
 *  linux/drivers/input/keyboard/zylonite_kbd.c
 *
 *  Keypad driver for zylonite Platform
 *
 *  Copyright (C) 2006, Marvell International Ltd (jingqing.xu@marvell.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <linux/device.h>

#include "zylonite_kbd.h"

#define USE_DEFAULT_DEBOUNCE 1

/* Bit Definitions for KeyPad Interface Control Register */
#define     DIRECT_KP_INTR_ENABLE		(0x1    <<  0)
#define     DIRECT_KP_ENABLE			(0x1    <<  1)
#define     ROTARY_ENCODER_0_ENABLE		(0x1    <<  2)
#define     ROTARY_ENCODER_1_ENABLE		(0x1    <<  3)
#define     ROTARY_ENCODER_ZERO_DEB		(0x1    <<  4)
#define     DIRECT_INTR_BIT			(0x1    <<  5)
#define     DIRECT_KEY_NUMS			(0x1	<<  6)
#define     DIRECT_DEBOUNCE_BIT			(0x1    <<  9)
#define     MATRIX_INTR_ENABLE			(0x1    <<  11)
#define     MATRIX_KP_ENABLE			(0x1    <<  12)
#define     IGNORE_MULTIPLE_KEY_PRESS		(0x1    <<  21)
#define     MATRIX_INTR_BIT			(0x1    <<  22)
#define     AUTO_SCAN_ON_ACTIVITY		(0x1    <<  29)
#define     AUTO_SCAN_BIT			(0x1    <<  30)
#define     MAT_SCAN_LINE0			(0x1    <<  13)
#define     MAT_SCAN_LINE1			(0x1    <<  14)
#define     MAT_SCAN_LINE2			(0x1    <<  15)
#define     MAT_SCAN_LINE3			(0x1    <<  16)
#define     MAT_SCAN_LINE4			(0x1    <<  17)
#define     MAT_SCAN_LINE5			(0x1    <<  18)
#define     MAT_SCAN_LINE6			(0x1    <<  19)
#define     MAT_SCAN_LINE7			(0x1    <<  20)

/* Bit Definitions for KeyPad Interface Direct Key Register */

#define     DIRECT_KEY_PRESSED			(0x1    <<  31)
#define     DIRECT_KEY_IN_7			(0x1    <<  7)
#define     DIRECT_KEY_IN_6			(0x1    <<  6)
#define     DIRECT_KEY_IN_5			(0x1    <<  5)
#define     DIRECT_KEY_IN_4			(0x1    <<  4)
#define     DIRECT_KEY_IN_3			(0x1    <<  3)
#define     DIRECT_KEY_IN_2			(0x1    <<  2)
#define     ROTARY_ENC_0_SENSOR_B		(0x1    <<  1)
#define     ROTARY_ENC_0_SENSOR_A		(0x1    <<  0)
#define     DIRECT_KEY_NUMS_MIN_1		(0x1    <<  6)

/* Bit Definitions for KeyPad Encoder Count Register */

#define     UNDERFLOW_ROTARY_ENC_0		(0x1    <<  14)
#define     OVERFLOW_ROTARY_ENC_0		(0x1    <<  15)

/* Bit Definitions for KeyPad Interface Matrix Key Register */

#define     MATRIX_KEY_PRESSED			(0x1    <<  31)
#define     MATRIX_ROW_MASK			0xFF

/* Bit Definitions for KeyPad Interface Automatic Scan Register */

#define     SCAN_ON_BIT				(0x1    <<  31)
#define     ROW_SELECTED_MASK			0xF0
#define     COL_SELECTED_MASK			0x0F
#define     MULTI_KEYS_PRESS			0x7C000000
#define     SINGLE_KEY_PRESS			(0x1    <<  26)

#define     KPASMKP(n)        			__REG(0x41500028 + ((n)<<3))
#define     KPASMKP_MASK       			0xFF
/*
Bit Definitions for KeyPad Interface Automatic Scan Multiple
Key Press Register 0,1,2,3
*/

#define     MATRIX_KP_COL_EVEN_MASK		0x0000FF
#define     MATRIX_KP_COL_ODD_MASK		0xFF0000
#define     KP_INVALID_ROW			0xFF

/*
KeyPad controller supports 3 modes of operation.
All three will be tested but the default
mode shall be AUTO_SCAN_ON_ACT
*/
#define     MANUAL_MATRIX_SCAN		0
#define     AUTO_SCAN_ON_ACT		1
#define     AUTO_SCAN_BY_AS_BIT		2

#define     NOT_AVAILABLE		0
#define     AVAILABLE			1
#define     COUNT_MASK			0xFF

#define     START_VALUE			0x7F

/*maximum number of keys that can be detected simultaneously*/
#define  DEFAULT_DIRECT_DEBOUNCE_MS  30
#define  DEFAULT_MATRIX_DEBOUNCE_MS  30

union scode {
	u8 scan_code[8];
	u32 word[2];
};

#ifdef CONFIG_KEYPAD_ZYLONITE_COMBOKEY
static int combo_key = 1;
static int pre_combo = 0;
static int pre_combo_rel = 1;
static int keys_pressed[] = { 0, 0 };
static union scode u_pre;
#endif


MODULE_AUTHOR("Xu Jingqing <Jingqing.Xu@marvell.com>");
MODULE_DESCRIPTION("Zylonite board keypad driver");
MODULE_LICENSE("GPL");

static int read_scan_code_automatically(int *key_index)
{
	int retval = -1;	/* Default to failure indication */
	u32 num_of_keys_pressed = 0;
	int scan_code_index = 0;
	/* minimize slow hardware accesses */
	u32 kp_asr_tmp;

	*key_index = NO_KEY;	/* Default to failure indication */

	num_of_keys_pressed = ((KPAS & MULTI_KEYS_PRESS) >> 26);

	if (num_of_keys_pressed == 1) {
		kp_asr_tmp = KPAS;
		/* test for valid data */
		if ((ROW_SELECTED_MASK == (kp_asr_tmp & ROW_SELECTED_MASK)) ||
		    (COL_SELECTED_MASK == (kp_asr_tmp & COL_SELECTED_MASK))) {
			/* If either row or column field == 0xFF,
			   data are invalid */
			*key_index = NO_KEY;
		} else {
			/* if not invalid, 3 bits of data for each field.
			   Compress to 6 bits for index into
			   the 64-entry LUT. */
			scan_code_index =
			    ((kp_asr_tmp & ROW_SELECTED_MASK) >> 1) |
			    (kp_asr_tmp & COL_SELECTED_MASK);

			*key_index = scan_code_index;

#ifdef CONFIG_KEYPAD_ZYLONITE_COMBOKEY
			if (combo_key) {
				u_pre.word[0] = 0;
				u_pre.word[1] = 0;
				u_pre.
				    scan_code[kp_asr_tmp & COL_SELECTED_MASK] =
				    (1 <<
				     ((kp_asr_tmp & ROW_SELECTED_MASK) >> 4));
				if (keys_pressed[1] !=
				    pxakbd_keycode[scan_code_index])
					keys_pressed[0] =
					    pxakbd_keycode[scan_code_index];
				if (pre_combo)
					pre_combo_rel = 1;
			}
#endif

			retval = 0;
		}
	} else {
#ifdef CONFIG_KEYPAD_ZYLONITE_COMBOKEY
		if (combo_key && (num_of_keys_pressed > 1)) {
			int i, row, col;
			u32 kpasmkp;
			union scode u;

			u.word[0] = 0;
			u.word[1] = 0;

			for (i = 0; i < 4; i++) {
				kpasmkp = KPASMKP(i);
				if (kpasmkp & KPASMKP_MASK) {
					col = i * 2;
					u.scan_code[col] =
					    (kpasmkp & KPASMKP_MASK);
					if (u_pre.scan_code[col] !=
					    u.scan_code[col]) {
						row = fls(u.scan_code[col]) - 1;
						scan_code_index =
						    (row << 3) | col;
						*key_index = scan_code_index;
					}
				}
				if (kpasmkp & (KPASMKP_MASK << 16)) {
					col = i * 2 + 1;
					u.scan_code[col] =
					    (kpasmkp >> 16) & KPASMKP_MASK;
					if (u_pre.scan_code[col] !=
					    u.scan_code[col]) {
						row = fls(u.scan_code[col]) - 1;
						scan_code_index =
						    (row << 3) | col;
						*key_index = scan_code_index;
					}
				}
			}

			pre_combo = 1;
			pre_combo_rel = 0;
			keys_pressed[1] = pxakbd_keycode[scan_code_index];
			u_pre.word[0] = u.word[0];
			u_pre.word[1] = u.word[1];

		} else
#endif
			/* do not support multi key press */
			*key_index = NO_KEY;
	}
	return retval;
}

/*-------------------------------------------------------------------------
Function: read_direct_keys
Purpose:  This function looks for any Thumbwheel movement or
	button press and returns a scan code.
Returns:  success/failure.
type no:  I/F proc 2
---------------------------------------------------------------------------*/
static int read_direct_keys(int *key)
{
	u32 curr_count, save_kp_rotary_encoder_count_reg;
	static u32 prev_count = START_VALUE;

	save_kp_rotary_encoder_count_reg = KPREC;
	curr_count = save_kp_rotary_encoder_count_reg & COUNT_MASK;

	if (save_kp_rotary_encoder_count_reg & OVERFLOW_ROTARY_ENC_0) {
		KPREC = START_VALUE;
		prev_count = START_VALUE;
		*key = KP_SC_SCROLL_UP;	/* Scroll Up */
	} else if (save_kp_rotary_encoder_count_reg & UNDERFLOW_ROTARY_ENC_0) {
		KPREC = START_VALUE;
		prev_count = START_VALUE;
		*key = KP_SC_SCROLL_DOWN;	/* Scroll Down */
	} else if (curr_count > prev_count) {
		*key = KP_SC_SCROLL_UP;
		prev_count = curr_count;	/* Scroll Up */
	} else if (curr_count < prev_count) {
		*key = KP_SC_SCROLL_DOWN;
		prev_count = curr_count;	/* Scroll Down */
	}

	return 0;
}

static void pxakbd_config_controller(int use_default_debounces,
			      u8 matix_debounce_ms,
			      u8 direct_debounce_ms, u8 max_row, u8 max_col)
{
	/* Init the control regs for direct keys */
	KPC = (((max_row - 1) << 26) |
	       ((max_col - 1) << 23) |
	       MAT_SCAN_LINE0 |
	       MAT_SCAN_LINE1 |
	       MAT_SCAN_LINE2 |
	       MAT_SCAN_LINE3 |
	       MAT_SCAN_LINE4 |
	       MAT_SCAN_LINE5 |
	       MAT_SCAN_LINE6 |
	       MAT_SCAN_LINE7 |
	       AUTO_SCAN_ON_ACTIVITY |
	       MATRIX_INTR_ENABLE |
	       MATRIX_KP_ENABLE |
	       ROTARY_ENCODER_ZERO_DEB |
	       DIRECT_KP_INTR_ENABLE |
	       DIRECT_KEY_NUMS | DIRECT_KP_ENABLE | ROTARY_ENCODER_0_ENABLE);

	KPREC = START_VALUE;

	if (use_default_debounces)
		KPKDI = (DEFAULT_DIRECT_DEBOUNCE_MS << 8 |
			 DEFAULT_MATRIX_DEBOUNCE_MS);
	else
		KPKDI = ((direct_debounce_ms << 8)
			 | matix_debounce_ms);
}

static int pxakbd_read_scancode(int *pui8_data)
{
	u32 kp_status;
	int key = NO_KEY;

	kp_status = KPC;
	/*
	   Process matrix first to capture scroll
	   wheel press rather than interpreting it
	   as a rotate, which can happen unintentionally.
	 */
	if (kp_status & MATRIX_INTR_BIT) {
		read_scan_code_automatically(&key);
		if (key == NO_KEY) {
			*pui8_data = NO_KEY;
			return -1;
		}
		key = pxakbd_keycode[key];
	} else if (kp_status & DIRECT_INTR_BIT)
		read_direct_keys(&key);

	*pui8_data = key;

	return key == NO_KEY;
}

static struct input_dev zylonite_kp_dev;

static irqreturn_t pxakbd_interrrupt(int irq, void *ptr, struct pt_regs *regs)
{
	int key;
	static int pre_key;
	int handled = 0;

	pxakbd_read_scancode(&key);

#ifdef CONFIG_KEYPAD_ZYLONITE_COMBOKEY
	pr_debug(
	    "%s, key %x, [0] %x [1] %x, pre_combo %d pre_combo_rel %d, pre_key %x\n",
	    __func__, key, keys_pressed[0], keys_pressed[1], pre_combo,
	    pre_combo_rel, pre_key);
	if (keys_pressed[0] && keys_pressed[1] && pre_combo_rel && pre_combo) {
		if (keys_pressed[0] == key)
			input_report_key(&zylonite_kp_dev, keys_pressed[1], 0);
		if (keys_pressed[1] == key) {
			input_report_key(&zylonite_kp_dev, keys_pressed[0], 0);
			keys_pressed[0] = key;
		}
		keys_pressed[1] = 0;
		pre_combo = 0;
		pre_key = key;
		handled = !0;
	}
#endif

	if (!handled && (key == KP_SC_SCROLL_UP || key == KP_SC_SCROLL_DOWN)) {
		input_report_key(&zylonite_kp_dev, key, 1);
		input_report_key(&zylonite_kp_dev, key, 0);
		handled = !0;
	}
	if (!handled) {
		if (key != NO_KEY) {
			input_report_key(&zylonite_kp_dev, key, 1);
			pre_key = key;
		} else
			input_report_key(&zylonite_kp_dev, pre_key, 0);
	}
	return IRQ_HANDLED;
}

static int zb_kp_resume(struct device *dev, u32 level)
{
	switch (level) {
	case RESUME_POWER_ON:
		pxa_set_cken(CKEN_KEYPAD, 1);
		pxakbd_config_controller(USE_DEFAULT_DEBOUNCE, 0, 0,
				 KEYPAD_ROWS_NUM, KEYPAD_COLS_NUM);

		break;
	}

	return 0;

}

static int zb_kp_suspend(struct device *dev, u32 state, u32 level)
{
	switch (level) {
	case SUSPEND_POWER_DOWN:
		pxa_set_cken(CKEN_KEYPAD, 0);
		break;
	}

	return 0;

}

static struct device_driver zb_kp_drv = {
	.bus = &platform_bus_type,
	.name = "zb_kp",
	.resume = zb_kp_resume,
	.suspend = zb_kp_suspend,
};

static int __init pxakbd_init(void)
{
	int i, err;

	pxa_set_cken(CKEN_KEYPAD, 1);
	pxakbd_config_controller(USE_DEFAULT_DEBOUNCE, 0, 0,
				 KEYPAD_ROWS_NUM, KEYPAD_COLS_NUM);
	init_input_dev(&zylonite_kp_dev);
#ifdef CONFIG_KEYPAD_ZYLONITE_REPEATKEY
	zylonite_kp_dev.evbit[0] = BIT(EV_KEY) | BIT(EV_REP);
#else
	zylonite_kp_dev.evbit[0] = BIT(EV_KEY);
#endif

	for (i = 0; i < NBITS(MAX_KEYS); i++)
		zylonite_kp_dev.keybit[i] = 0xffffffff;

	err = request_irq(IRQ_KEYPAD, pxakbd_interrrupt, 0, "Keypad", NULL);

	if (err) {
		printk(KERN_CRIT "Wow!  Can't register IRQ[%d] for Keypad\n",
		       IRQ_KEYPAD);
		return err;
	}

	zylonite_kp_dev.name = "pxa-keypad";
	input_register_device(&zylonite_kp_dev);

	return driver_register(&zb_kp_drv);
}

static void __exit pxakbd_exit(void)
{
	driver_unregister(&zb_kp_drv);
	input_unregister_device(&zylonite_kp_dev);
	free_irq(IRQ_KEYPAD, NULL);
}

module_init(pxakbd_init);
module_exit(pxakbd_exit);
