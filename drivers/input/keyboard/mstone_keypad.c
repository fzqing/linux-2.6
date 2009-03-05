/*
 * linux/driver/char/mstone_keypad.c
 * Keypad driver for Intel Mainstone development board
 *
 * Author:	Nicolas Pitre
 * Created:	may 5, 2004
 * Copyright:	(C) 2004 MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/device.h>

#include <asm/irq.h>
#include <asm/hardware.h>

#include <asm/arch/pxa-regs.h>

static int pxakp_suspend(struct device *dev, u32 state, u32 level);
static int pxakp_resume(struct device *dev, u32 level);



static struct device_driver pxakp_driver = {
	.name		= "pxa2xx-keypad",
	.bus		= &platform_bus_type,
	.suspend	= pxakp_suspend,
	.resume		= pxakp_resume,
};

static struct platform_device pxakp_device_ldm = {
	.name           = "keypad",
	.id             = 0,
	.dev = {
		.driver = &pxakp_driver,
		.platform_data  = NULL,
	},
};

/*
 * The Mainstone II manual says:
 *
 * · The 75-button, 7x7 matrix keypad maps to the Bulverde matrix signals
 *   MKIN<7:0> and MKOUT<7:0>.
 * · The rotary scroll wheel maps to the Bulverde signals DKIN<1:0>. The
 *   push-button maps to a matrix key.
 *
 * Since a 7x7=49 matrix can't hold all those keys, many keys were
 * made to activate 2 matrix positions at the same time.
 * This is not the most reliable thing to do, but we're stuck with it.
 *
 * The problem is that those combined key presses don't activate
 * simultaneously, so for each matrix event we need to wait a
 * certain delay to account for this before reporting a real key event.
 *
 * But yet some of those combined keys are rather unstable and don't produce
 * a consistent state for the whole duration of the key down time.
 *
 * So it seems that the only way to have some sort of sane behavior is to
 * actually report any event only when all keys have been released. Because
 * of this we can't rely on and report any maintained state.
 *
 * "Please don't use such scheme for your own product."
 */

#define MULTISWITCH

/* First, the array of single switch keys */
static const unsigned short mstkp_keycodes[64] = {
  KEY_A,     KEY_G,    KEY_M,    KEY_S,     KEY_DOT,       KEY_HOME,      KEY_UP,    0,
  KEY_B,     KEY_H,    KEY_N,    KEY_T,     KEY_EQUAL,     KEY_LEFTSHIFT, KEY_DOWN,  0,
  KEY_C,     KEY_I,    KEY_O,    KEY_U,     KEY_Y,         KEY_SPACE,     KEY_LEFT,  0,
  KEY_D,     KEY_J,    KEY_P,    KEY_V,     KEY_Z,         KEY_SPACE,     KEY_RIGHT, 0,
  KEY_E,     KEY_K,    KEY_Q,    KEY_W,     KEY_SLASH,     KEY_POWER,     KEY_ENTER, 0,
  KEY_F,     KEY_L,    KEY_R,    KEY_X,     KEY_BACKSLASH, KEY_DELETE,    0,         0,
  KEY_PHONE, KEY_STOP, BTN_LEFT, BTN_RIGHT, BTN_MIDDLE,    0,	          0,         0,
  0,         0,        0,        0,         0,             0,             0,         0,
};

static void hw_init(void){
  /* Setup GPIOs, set default debounce time  and so on*/
	pxa_gpio_mode( 93 | GPIO_ALT_FN_1_IN);	/* KP_DKIN0 */
	pxa_gpio_mode( 94 | GPIO_ALT_FN_1_IN);	/* KP_DKIN1 */
	pxa_gpio_mode( 95 | GPIO_ALT_FN_3_IN);	/* KP_MKIN6 */
	pxa_gpio_mode( 96 | GPIO_ALT_FN_3_OUT);	/* KP_MKOUT6 */
	pxa_gpio_mode( 97 | GPIO_ALT_FN_3_IN);	/* KP_MKIN3 */
	pxa_gpio_mode( 98 | GPIO_ALT_FN_3_IN);	/* KP_MKIN4 (?) */
	pxa_gpio_mode( 99 | GPIO_ALT_FN_3_IN);	/* KP_MKIN5 */
	pxa_gpio_mode(100 | GPIO_ALT_FN_1_IN);	/* KP_MKIN0 */
	pxa_gpio_mode(101 | GPIO_ALT_FN_1_IN);	/* KP_MKIN1 */
	pxa_gpio_mode(102 | GPIO_ALT_FN_1_IN);	/* KP_MKIN2 */
	pxa_gpio_mode(103 | GPIO_ALT_FN_2_OUT);	/* KP_MKOUT0 */
	pxa_gpio_mode(104 | GPIO_ALT_FN_2_OUT);	/* KP_MKOUT1 */
	pxa_gpio_mode(105 | GPIO_ALT_FN_2_OUT);	/* KP_MKOUT2 */
	pxa_gpio_mode(106 | GPIO_ALT_FN_2_OUT);	/* KP_MKOUT3 */
	pxa_gpio_mode(107 | GPIO_ALT_FN_2_OUT);	/* KP_MKOUT4 */
	pxa_gpio_mode(108 | GPIO_ALT_FN_2_OUT);	/* KP_MKOUT5 */

	/* Set default debounce time */
	KPKDI = ((0 << 8) | (100 << 0));
}

static void kp_set_ctrl_reg(void ){
  KPC = (KPC_ASACT | (7<<26) | (7<<23) | KPC_MS_ALL |
  	 (2<<6) | KPC_REE0 | KPC_DK_DEB_SEL |
  	 KPC_ME | KPC_MIE | KPC_DE | KPC_DIE);
}

#ifdef MULTISWITCH

/* Next, multiswitch keys, indexed by their lowest coordinate */
static const unsigned short mstkp_multicodes[40] = {
  0, KEY_1,  KEY_4,  KEY_7,         KEY_KPASTERISK, KEY_TAB,        0, 0,
  0, KEY_F1, KEY_F3, KEY_MINUS,     KEY_APOSTROPHE, KEY_LEFTBRACE,  0, 0,   
  0, KEY_2,  KEY_5,  KEY_8,         KEY_0,          KEY_BACKSPACE,  0, 0,
  0, KEY_F2, KEY_F4, KEY_SEMICOLON, KEY_COMMA,      KEY_RIGHTBRACE, 0, 0,   
  0, KEY_3,  KEY_6,  KEY_9,         KEY_GRAVE,      KEY_KPENTER,    0, 0,
};

static unsigned long mst_keys_down[2];
static unsigned long mst_keys_cumul[2];

static void mst_handle_key(struct input_dev *dev, unsigned int key, int down)
{
	if (key >= 48 || key % 8 >= 6) {
		/* outside of the alphanum keypad -- send event right away */
		input_report_key( dev, mstkp_keycodes[key], down);
	} else 	if (down) {
		__set_bit(key, mst_keys_down);
		__set_bit(key, mst_keys_cumul);
	} else {
		__clear_bit(key, mst_keys_down);
		if ((mst_keys_down[0] | mst_keys_down[1]) == 0) {
			/* everything released at this point */
			int nbits = hweight32(mst_keys_cumul[0])
				  + hweight32(mst_keys_cumul[1]);
			if (nbits == 1) {
				/* this was a single switch key */
				input_report_key(dev, mstkp_keycodes[key], 1);
				input_report_key(dev, mstkp_keycodes[key], 0);
			} else if (nbits == 2) {
				int x, y;
				mst_keys_cumul[0] ^= ~0;
				mst_keys_cumul[1] ^= ~0;
				x = find_first_zero_bit(mst_keys_cumul, 64);
				y = find_next_zero_bit(mst_keys_cumul, 64, x+1);
				if (x < 38 && y - x == 7) {
					/* we have a valid multiswitch key */
					input_report_key(dev, mstkp_multicodes[x], 1);
					input_report_key(dev, mstkp_multicodes[x], 0);
				}
			}
			mst_keys_cumul[0] = 0;
			mst_keys_cumul[1] = 0;
		}
	}
}

#endif  /* MULTISWITCH */

static irqreturn_t pxakp_interrupt(int irq, void *ptr, struct pt_regs *regs) 
{
	struct input_dev *dev = ptr;
	unsigned long kpc = KPC;

	if (kpc & KPC_DI) {
		unsigned long kpdk = KPDK;
		if (!(kpdk & KPDK_DKP)) {
			/* better luck next time */
		} else if (kpc & KPC_REE0) {
			int rel;
			unsigned long kprec = KPREC;
			KPREC = 0x7f;
			if (kprec & KPREC_OF0) {
				rel = (kprec & 0xff) + 0x7f;
			} else if (kprec & KPREC_UF0) {
				rel = (kprec & 0xff) - 0x7f - 0xff;
			} else {
				rel = (kprec & 0xff) - 0x7f;
			}
			if (rel) {
				input_report_rel(dev, REL_WHEEL, rel);
			}
		}
	}

	if (kpc & KPC_MI) {
		static unsigned long prev_mkp[4] = {0, 0, 0, 0};
		unsigned long mkp[4] = {KPASMKP0, KPASMKP1, KPASMKP2, KPASMKP3};
		unsigned long stat, flip;
		unsigned int col, row;
		for (col = 0; col < 8; col += 2) {
			stat = mkp[col/2];
			if (stat & KPASMKPx_SO)
				continue;
			flip = stat ^ prev_mkp[col/2];
			prev_mkp[col/2] = stat;
			while (flip) {
				row = __ffs(flip);
				if (row >= 8) {
					stat >>= 16;
					flip >>= 16;
					col |= 1;
					continue;
				}
#ifndef MULTISWITCH
				input_report_key( dev, 
						  mstkp_keycodes[row*8 + col],
						  stat & (1 << row) );
#else
				mst_handle_key( dev, row*8 + col, 
						stat & (1 << row) );
#endif
				flip &= ~(1 << row);
			}
			col &= ~1;
		}
	}
	return IRQ_HANDLED;
}

static int pxakp_open(struct input_dev *dev)
{
	if (dev->private++ == 0) {
		/* Set keypad control register */
		kp_set_ctrl_reg();

		/* Set rotary count to mid-point value */
		KPREC = 0x7F;

		/* Enable unit clock */
		pxa_set_cken(CKEN19_KEYPAD, 1);
	}

	return 0;
}

static void pxakp_close(struct input_dev *dev)
{
	if (--dev->private == 0) {
		/* Disable clock unit */
		pxa_set_cken(CKEN19_KEYPAD, 0);

		/* Disable keypad */
		KPC = 0;
	}
}

static int pxakp_suspend(struct device *dev, u32 state, u32 level)
{
	/* Section 28000101.pdf 18.4.3, 28000001.pdf Section 18.4.5.
	*Keypad may be suspended if it's not used as wakeup source (see PKWR register)
	*/

	/* Disable clock unit */
	pxa_set_cken(CKEN19_KEYPAD, 0);

	/* Disable keypad */
	KPC = 0;
	return 0;
}

static int pxakp_resume(struct device *dev, u32 level)
{
	/*28000101.pdf Section 3.8.2.2 - enable/disable clock CKEN*/
	hw_init();
	/*read KPSR  int pksr=PKSR;*/

	/* Set keypad control register */
	kp_set_ctrl_reg();

	/* Set rotary count to mid-point value */
	KPREC = 0x7F;

	/* Enable unit clock */
	pxa_set_cken(CKEN19_KEYPAD, 1);

	return 0;
}

static struct input_dev *pxakp_inputdev;

static int __init pxakp_init(void)
{
	int ret, i;

	pxakp_inputdev = kmalloc(sizeof(*pxakp_inputdev), GFP_KERNEL);
	if (!pxakp_inputdev)
		return -ENOMEM;
	memset(pxakp_inputdev, 0, sizeof(*pxakp_inputdev));

	ret = request_irq(IRQ_KEYPAD, pxakp_interrupt, 0, "keypad", pxakp_inputdev);
	if (ret < 0) {
		kfree(pxakp_inputdev);
		return ret;
	}

	hw_init();

	pxakp_inputdev->name = "keypad";
	pxakp_inputdev->id.bustype = BUS_HOST;
	pxakp_inputdev->open = pxakp_open;
	pxakp_inputdev->close = pxakp_close;

	__set_bit(EV_REL, pxakp_inputdev->evbit);
	__set_bit(EV_KEY, pxakp_inputdev->evbit);
	__set_bit(REL_WHEEL, pxakp_inputdev->relbit);
	for (i = 0; i < ARRAY_SIZE(mstkp_keycodes); i++)
		__set_bit(mstkp_keycodes[i], pxakp_inputdev->keybit);
#ifdef MULTISWITCH
	for (i = 0; i < ARRAY_SIZE(mstkp_multicodes); i++)
		__set_bit(mstkp_multicodes[i], pxakp_inputdev->keybit);
#endif

	input_register_device(pxakp_inputdev);

	driver_register(&pxakp_driver);
	platform_device_register(&pxakp_device_ldm);

	return 0;
}

static void __exit pxakp_exit(void)
{
	free_irq(IRQ_KEYPAD, pxakp_inputdev);
	input_unregister_device(pxakp_inputdev);

	platform_device_unregister(&pxakp_device_ldm);
	driver_unregister(&pxakp_driver);

	kfree(pxakp_inputdev);
}

module_init(pxakp_init);
module_exit(pxakp_exit);

MODULE_DESCRIPTION("Intel Mainstone KEYPAD driver");
MODULE_LICENSE("GPL");
