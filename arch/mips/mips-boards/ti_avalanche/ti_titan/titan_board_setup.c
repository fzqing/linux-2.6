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
	
#include <linux/config.h>
#include <pal.h>
	
void avalanche_set_vbus_freq(unsigned int new_vbus_freq);
void vlynq_dev_init(void);
	
static void avalanche_display_titan_variant( int titan_variant )
{
	char    *type_str;    
	
	/* Display Titan variant */
	switch(titan_variant)
	{
	case TITAN_1050:
		type_str = "TNETV1050";
		break;
	
	case TITAN_1055:
		type_str = "TNETV1055";
		break;
	
	case TITAN_1056:
		type_str = "TNETV1056";
		break;
	
	case TITAN_1060:
		type_str = "TNETV1060";
		break;
	
	default:
		type_str = "Unknown";
		printk("titan_type val is 0x%x\n",titan_variant);
	}
	
	printk("%s Communication Processor detected\n", type_str );
}
	
void avalanche_soc_platform_init(void)
{
	volatile PIN_SEL_REG_ARRAY_T *pin_sel_array = 
				(PIN_SEL_REG_ARRAY_T*) AVALANCHE_PIN_SEL_BASE;
	volatile unsigned int *eswitch_control_reg  = 
				(unsigned int*) AVALANCHE_ESWITCH_CONTROL;
	PAL_SYS_Tnetv1050Init clkc;
	
	/* Get entities on the board out of reset. */
	avalanche_reset_ctrl( AVALANCHE_ESWITCH_RESET_BIT, OUT_OF_RESET ); 
	avalanche_reset_ctrl( AVALANCHE_LOW_EPHY_RESET_BIT, OUT_OF_RESET );
	avalanche_reset_ctrl( AVALANCHE_HIGH_EPHY_RESET_BIT, OUT_OF_RESET );
	
	avalanche_gpio_init();
	
	avalanche_display_titan_variant( avalanche_get_board_variant() );    
	
	pin_sel_array->reg[3] &= ~0xff000000;  /* Gpio bits:    b 1111 1111 xxxx xxxx xxxx xxxx xxxx xxxx */
	pin_sel_array->reg[3] |=  0x55000000;  /* Bit setting:  b 0101 0101 xxxx xxxx xxxx xxxx xxxx xxxx */  
	
	pin_sel_array->reg[4] &= ~0x00003fc0;  /* Gpio bits:    b xxxx xxxx xxxx xxxx xx11 1111 11xx xxxx */
	pin_sel_array->reg[4] |=  0x00001540;  /* Bit setting:  b xxxx xxxx xxxx xxxx xx01 0101 01xx xxxx */
	
	pin_sel_array->reg[9] &= ~0xf03f0000;  /* Eth LED bits: b 1111 xxxx xx11 1111 xxxx xxxx xxxx xxxx */
	pin_sel_array->reg[9] |=  0x50150000;  /* Bit setting:  b 0101 xxxx xx01 0101 xxxx xxxx xxxx xxxx */
	
	pin_sel_array->reg[10] &= ~0x0000003f;  /* Eth LED bits: b xxxx xxxx xxxx xxxx xxxx xxxx xx11 1111 */
	pin_sel_array->reg[10] |=  0x00000015;  /* Bit setting:  b xxxx xxxx xxxx xxxx xxxx xxxx xx01 0101 */ 
	
	clkc.audclk = AUDCLK_FREQ;
	clkc.refclk = REFCLK_FREQ;
	clkc.altclk = ALTCLK_FREQ;
	avalanche_clkc_init(&clkc);
	
	avalanche_set_vbus_freq(avalanche_clkc_getfreq(CLKC_VBUS));
	
	*eswitch_control_reg = 0x010000ce;    
}
