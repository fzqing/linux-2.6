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
	
#include <asm/mach-avalanche/pal.h>
	
#define MIN(x,y)               ( ((x) <  (y)) ? (x) : (y) )
#define MAX(x,y)               ( ((x) >  (y)) ? (x) : (y) )
#define ABS(x)                 ( ((signed)(x) > 0) ? (x) : (-(x)) )
#define QUOTIENT(x,y)              ( ((x) + (y) / 2) / (y) )
	
#define CLKC_NUM               6
	
#define CLKC_PRE_DIVIDER        0x001F0000
#define CLKC_POST_DIVIDER       0x0000001F
	
#define CLKC_PLL_STATUS         0x1
#define CLKC_PLL_FACTOR         0x0000F000
	
#define BOOTCR_PLL_BYPASS       (1 << 5)
#define BOOTCR_MIPS_ASYNC_MODE  (1 << 25)
#define BOOTCR_VBUS_MUXSEL      (1 << 27)
	
#define MIPS_PLL_SELECT         0x01800000
#define VBUS_PLL_SELECT         0x0000C000
#define USB_PLL_SELECT          0x00030000
#define ETH_INPUT_SELECT        0x000C0000
#define DSP_INPUT_SELECT        0x00300000
	
#define USB_MODE_SELECT         0x00400000
#define VBUS_MUX_SELECT         0x08000000
	
#define MIPS_AUDCLKI_SELECT     0x00000000
#define MIPS_REFCLKI_SELECT     0x00800000
#define MIPS_ALTIN_SELECT       0x01000000
	
#define VBUS_AUDCLKI_SELECT   0x00000000
#define VBUS_REFCLKI_SELECT   0x00004000
#define VBUS_ALTIN_SELECT     0x00008000
#define VBUS_MIPSPLL_SELECT   0x0000C000
	
#define USB_AUDCLKI_SELECT      0x00000000
#define USB_REFCLKI_SELECT      0x00010000
#define USB_ALTIN_SELECT        0x00020000
#define USB_MIPSPLL_SELECT      0x00030000
	
#define  SYS_MAX                CLK_MHZ(150)
#define  SYS_MIN                CLK_MHZ(1)
	
#define VBUS_MAX                SYS_MAX/2
#define VBUS_MIN                SYS_MIN
	
#define  MIPS_SYNC_MAX          SYS_MAX
#define  MIPS_ASYNC_MAX         CLK_MHZ(165)
#define  MIPS_MIN               CLK_MHZ(1)
	
#define  USB_MAX                CLK_MHZ(100)
#define  USB_MIN                CLK_MHZ(1)
	
#define  LCD_MAX                CLK_MHZ(100)
#define  LCD_MIN                CLK_MHZ(1)
	
#define  PLL_MUL_MAXFACTOR      15
#define  MAX_DIV_VALUE          32
#define  MIN_DIV_VALUE          1
	
#define  MIN_PLL_INP_FREQ       CLK_MHZ(8)
#define  MAX_PLL_INP_FREQ       CLK_MHZ(100)
	
#define  DIVIDER_LOCK_TIME      (10100)
#define  PLL_LOCK_TIME          (10100 * 75)
	
typedef struct CLKC_STRUCT_tag
{
	unsigned int clkcr_reg;
	unsigned int reserved1[3];
	unsigned int clkpllcr_reg;
	unsigned int reserved2[3];
} CLKC_STRUCT_T;
	
	
typedef struct PAL_SYS_CLKC_STRUCT_tag
{
	CLKC_STRUCT_T clk[CLKC_NUM];
} PAL_SYS_CLKC_STRUCT_T;
	
static volatile PAL_SYS_CLKC_STRUCT_T *pclk_regs;
static volatile int  *bootcr_reg;
static unsigned int *clk_pll_src[CLKC_NUM];
static unsigned int clk_to_pll[CLKC_NUM];
static unsigned int mips_pll_out;
static unsigned int audclk_inp;
static unsigned int refclk_inp;
static unsigned int alt_inp;
static unsigned int present_min;
static unsigned int present_max;
	
static unsigned int find_gcd(unsigned int min, unsigned int max);
static unsigned int compute_prediv( unsigned int divider, unsigned int min, unsigned int max);
static void get_val(unsigned int base_freq, unsigned int output_freq,unsigned int *multiplier, unsigned int *divider);
static unsigned int get_base_frequency(PAL_SYS_CLKC_ID_T clk_id);
static void find_approx(unsigned int *,unsigned int *,unsigned int);
static int validate_arg(PAL_SYS_CLKC_ID_T clk_id,int output_freq);
static void usb_clk_check(void);
static int set_pll_div(PAL_SYS_CLKC_ID_T clk_id, unsigned int output_freq);
static unsigned int get_pll_div(PAL_SYS_CLKC_ID_T clk_id);
	
void avalanche_clkc_init(void* param)
{
	unsigned int choice;               
	
	PAL_SYS_Tnetv1050Init* ptr = (PAL_SYS_Tnetv1050Init *)param;                                   
	audclk_inp = ptr->audclk;                                          
	refclk_inp = ptr->refclk;                                        
	alt_inp 	= ptr->altclk;                                          
	
	pclk_regs = (volatile PAL_SYS_CLKC_STRUCT_T *)(AVALANCHE_CLOCK_CONTROL_BASE + 0x20); 
	bootcr_reg = (volatile int *)AVALANCHE_DCL_BOOTCR;
	
	choice = (*bootcr_reg) & MIPS_PLL_SELECT;
	switch(choice)
	{
	case MIPS_AUDCLKI_SELECT:
		clk_pll_src[CLKC_MIPS] = &audclk_inp;
		break;
	
	case MIPS_REFCLKI_SELECT:
		clk_pll_src[CLKC_MIPS] = &refclk_inp;
		break;
	
	case MIPS_ALTIN_SELECT:
		clk_pll_src[CLKC_MIPS] = &alt_inp;
		break;
	
	default :
		clk_pll_src[CLKC_MIPS] = 0;
	
	}
	choice = (*bootcr_reg) & VBUS_PLL_SELECT;
	switch(choice)
	{
	case VBUS_AUDCLKI_SELECT:
		clk_pll_src[CLKC_VBUS] = &audclk_inp;
		break;
	
	case VBUS_REFCLKI_SELECT:
		clk_pll_src[CLKC_VBUS] = &refclk_inp;
		break;
	
	case VBUS_ALTIN_SELECT:
		clk_pll_src[CLKC_VBUS] = &alt_inp;
		break;
	
	case VBUS_MIPSPLL_SELECT:
		clk_pll_src[CLKC_VBUS] = &mips_pll_out;
		break;
	
	default :
		clk_pll_src[CLKC_VBUS] = 0;
	
	}
	
	choice = (*bootcr_reg) & USB_PLL_SELECT;
	switch(choice)
	{
	case USB_AUDCLKI_SELECT:
		clk_pll_src[CLKC_USB] = &audclk_inp;
		break;
	
	case USB_REFCLKI_SELECT:
		clk_pll_src[CLKC_USB] = &refclk_inp;
		break;
	
	case USB_ALTIN_SELECT:
		clk_pll_src[CLKC_USB] = &alt_inp;
		break;
	
	case USB_MIPSPLL_SELECT:
		clk_pll_src[CLKC_USB] = &mips_pll_out;
		break;
	
	default :
		clk_pll_src[CLKC_USB] = 0;
	
	}
	
	clk_to_pll[CLKC_VBUS] =   \
	(BOOTCR_VBUS_MUXSEL & (*bootcr_reg)) ? CLKC_MIPS: CLKC_VBUS; 
	
	clk_to_pll[CLKC_MIPS] =   \
	(BOOTCR_MIPS_ASYNC_MODE & (*bootcr_reg)) ? CLKC_MIPS: CLKC_VBUS; 
	
	clk_to_pll[CLKC_USB] = CLKC_USB;
	clk_to_pll[CLKC_LCD] = CLKC_VBUS;
	clk_to_pll[CLKC_VBUSP] = CLKC_VBUS;
}
	
int avalanche_clk_setfreq(PAL_SYS_CLKC_ID_T clk_id, unsigned int output_freq)
{
	int pll_id;
	
	if(CLKC_SYS == clk_id)
		clk_id = CLKC_VBUSP;
	
	if(clk_id >= CLKC_NUM)
		return -1;

	pll_id = clk_to_pll[clk_id];
	
	if(pll_id == CLKC_VBUS)
	return -1;
	
	if( (clk_id == CLKC_USB) && (CLK_MHZ(48) == output_freq) )
		usb_clk_check();  
	
	return set_pll_div(pll_id,output_freq);
}
	
int avalanche_clkc_getfreq(PAL_SYS_CLKC_ID_T clk_id)
{
	int pll_id;
	int output_freq;
	
	if(clk_id >= CLKC_NUM)
		return -1;
	
	if(CLKC_SYS == clk_id)
		clk_id = CLKC_VBUSP;
	
	pll_id = clk_to_pll[clk_id];
	
	output_freq = get_pll_div(pll_id);
	
	if(clk_id == CLKC_VBUS)
		output_freq >>= 1;
	
	return output_freq;
	
}
	
static int set_pll_div(PAL_SYS_CLKC_ID_T clk_id,unsigned int output_freq)
{
	unsigned int base_freq;
	unsigned int multiplier;
	unsigned int divider;
	unsigned int min_prediv;
	unsigned int max_prediv;
	unsigned int prediv;
	unsigned int postdiv;
	int ret =0;
	volatile unsigned int temp;
	
	if( validate_arg(clk_id,output_freq) == -1)
		return -1;
	
	
	if( (clk_id == CLKC_USB) && (CLK_MHZ(48) == output_freq) )
		usb_clk_check();  
	
	base_freq = get_base_frequency(clk_id);
	
	if( base_freq < MIN_PLL_INP_FREQ)
	{
		return -1;
	}
	
	get_val(output_freq, base_freq, &multiplier, &divider);
	
	if( (multiplier  > PLL_MUL_MAXFACTOR) || (multiplier <= 0) )
	{
		return -1;
	}
	
	if( divider == 0 )
	{
		return -1;
	}
	
	min_prediv = MAX(base_freq / MAX_PLL_INP_FREQ + 1, divider / MAX_DIV_VALUE + 1);
	max_prediv = MIN(base_freq / MIN_PLL_INP_FREQ, MAX_DIV_VALUE);
	
	if (divider < min_prediv)
	{
		temp = QUOTIENT(min_prediv, divider);
		if ((temp * multiplier) > PLL_MUL_MAXFACTOR)
		{
			return -1  ;
		}
		else
		{
			multiplier = temp * multiplier;
			divider = min_prediv;
		}
	}
	
	prediv = compute_prediv (divider, min_prediv, max_prediv);
	postdiv = QUOTIENT(divider,prediv);
	
	if(postdiv > MAX_DIV_VALUE)
	{
		return -1;
	}
	
	if( (base_freq * multiplier) != (output_freq * prediv * postdiv))
		ret = 1; /* Approximate frequency value  is set */
	
	pclk_regs->clk[clk_id].clkcr_reg = ( ((prediv -1) & 0x1F) << 16) |
	((postdiv -1) & 0x1F);
	
	
	for(temp =0; temp < DIVIDER_LOCK_TIME; temp++);
	
	pclk_regs->clk[clk_id].clkpllcr_reg = 0x4;
	multiplier = (((multiplier - 1) & 0xf) << 12)| ((255 <<3) | 0x0e);
	while((pclk_regs->clk[clk_id].clkpllcr_reg) & CLKC_PLL_STATUS)
		/*nothing*/;
	
	pclk_regs->clk[clk_id].clkpllcr_reg = multiplier;
	
	while(!((pclk_regs->clk[clk_id].clkpllcr_reg) & CLKC_PLL_STATUS))
		/*nothing*/;
	
	for(temp =0; temp < PLL_LOCK_TIME; temp++);
	
	return ret;
}
	
static unsigned int get_pll_div(PAL_SYS_CLKC_ID_T clk_id)
{
	unsigned int  clk_ctrl_register;
	unsigned int  clk_pll_setting;
	unsigned int  clk_predivider;
	unsigned int  clk_postdivider;
	unsigned short  pll_factor;
	unsigned int  base_freq;
	unsigned int  divider;
	
	base_freq = get_base_frequency(clk_id);
	
	clk_ctrl_register = pclk_regs->clk[clk_id].clkcr_reg;
	clk_postdivider = (CLKC_POST_DIVIDER & clk_ctrl_register) + 1;
	clk_predivider = ((CLKC_PRE_DIVIDER & clk_ctrl_register) >> 16) + 1;
	
	divider =  clk_predivider * clk_postdivider;
	
	if( ((*bootcr_reg) & BOOTCR_PLL_BYPASS))
	{
		return (QUOTIENT(base_freq, divider));  /* PLLs bypassed.*/
	}
	else
	{
		clk_pll_setting = pclk_regs->clk[clk_id].clkpllcr_reg;
	
		pll_factor = ((clk_pll_setting & CLKC_PLL_FACTOR) >> 12) + 1;
	
		if((clk_pll_setting & 0x1)   == 0)
		{
			if(pll_factor <  0x10)
				return (QUOTIENT(base_freq >> 1, divider));
			else
				return (QUOTIENT(base_freq >> 2, divider));
		}
	
		else     /* We're in PLL mode */
		{
			if((clk_pll_setting & 0x0800) && (clk_pll_setting & 0x2))
			{
				if( pll_factor & 0x1 )
				{
					return(QUOTIENT((base_freq * pll_factor) >> 1, divider));
				}
				else
				{
					return(QUOTIENT((base_freq * (pll_factor - 1)) >>2, divider));
				}
			}
			else
			{
				if(pll_factor < 0x10)
				{
					return(QUOTIENT(base_freq * pll_factor, divider));
				}
				else
				{
					return(QUOTIENT(base_freq, divider));
				}
			}
		}
		
		return(0); /* Should never reach here */
	}
}
	
static unsigned int get_base_frequency(PAL_SYS_CLKC_ID_T clk_id)
{
	if ( clk_pll_src[clk_id] == &mips_pll_out)
	{
		*clk_pll_src[clk_id] = get_pll_div(CLKC_MIPS);
	}
	
	return (*clk_pll_src[clk_id]);
}
	
static unsigned int find_gcd(unsigned int min,unsigned int max)
{
	if (max % min == 0)
	{
		return min;
	}
	else
	{
		return find_gcd(max % min, min);
	}
}
	
static unsigned int compute_prediv(unsigned int divider, unsigned int min, unsigned int max)
{
	unsigned short prediv;
	if (min <= divider && divider <= max)
	{
		return divider;
	}
	
	for (prediv = max; prediv >= min ; prediv--)
	{
		if ( (divider % prediv) == 0 )
		{
			return prediv;
		}
	}
	
	return min;
}
	
static void get_val(unsigned int output_freq, unsigned int base_freq, unsigned int *multiplier, unsigned int *divider)
{
	unsigned int temp_mul;
	unsigned int temp_div;
	unsigned int gcd;
	unsigned int min_freq;
	unsigned int max_freq;
	
	min_freq = (base_freq < output_freq) ? base_freq : output_freq;
	max_freq = (base_freq > output_freq) ? base_freq : output_freq;
	gcd = find_gcd(min_freq , max_freq);
	
	if(gcd == 0)
		return;  /* ERROR */
	
	temp_mul = output_freq / gcd;
	temp_div = base_freq / gcd;
	
	if( temp_mul > PLL_MUL_MAXFACTOR )
	{
		if((temp_mul / temp_div) > PLL_MUL_MAXFACTOR)
			return;
	
		find_approx(&temp_mul,&temp_div,base_freq);
	}
	
	*multiplier = temp_mul;
	*divider    = temp_div;
}
	
static void find_approx(unsigned int *num, unsigned int *denom, unsigned int base_freq)
{
	unsigned int num1;
	unsigned int denom1;
	unsigned int num2;
	unsigned int denom2;
	int closest;
	int prev_closest;
	unsigned int temp_num;
	unsigned int temp_denom;
	unsigned int normalize;
	unsigned int gcd;
	unsigned int output_freq;
	
	num1 = *num;
	denom1 = *denom;
	
	prev_closest = 0x7fffffff; /* maximum possible value */
	num2 = num1;
	denom2 = denom1;
	
	for(temp_num = 15; temp_num >=1; temp_num--)
	{
		temp_denom = QUOTIENT(temp_num * denom1, num1);
		output_freq = (temp_num * base_freq) / temp_denom;
	
		if(temp_denom < 1)
		{
			break;
		}
		else
		{
			normalize = QUOTIENT(num1,temp_num);
			closest = (ABS((num1 * (temp_denom) ) - (temp_num * denom1)))  * normalize;
			if(closest < prev_closest && output_freq > present_min && output_freq <present_max)
			{
				prev_closest = closest;
				num2 = temp_num;
				denom2 = temp_denom;
			}
		}
	}
	
	gcd = find_gcd(num2,denom2);
	num2 = num2 / gcd;
	denom2 = denom2 /gcd;
	
	*num      = num2;
	*denom    = denom2;
}
	
static int validate_arg(PAL_SYS_CLKC_ID_T clk_id,int output_freq)
{
	if((*bootcr_reg)  & BOOTCR_PLL_BYPASS)
	{
		return -1;
	}
	
	switch( clk_id )
	{
	case CLKC_VBUS:
		if((output_freq < VBUS_MIN) ||
			(output_freq > ((*bootcr_reg) & BOOTCR_VBUS_MUXSEL) ? (MIPS_ASYNC_MAX/2): (MIPS_SYNC_MAX/2)))
		{
			return -1;
		}
		present_min = VBUS_MIN;
		present_max = ( (*bootcr_reg) & BOOTCR_VBUS_MUXSEL) ? (MIPS_ASYNC_MAX/2):(MIPS_SYNC_MAX/2);
		break;
	
	case CLKC_MIPS:
		if((output_freq < MIPS_MIN) ||
			(output_freq > ( (*bootcr_reg) & BOOTCR_MIPS_ASYNC_MODE) ? MIPS_ASYNC_MAX: MIPS_SYNC_MAX))
		{
			return -1;
		}
		present_min = MIPS_MIN;
		present_max = ((*bootcr_reg) & BOOTCR_MIPS_ASYNC_MODE) ? MIPS_ASYNC_MAX: MIPS_SYNC_MAX;
		break;
	
	case CLKC_USB:
		if( (output_freq < USB_MIN) || (output_freq > USB_MAX))
		{
			return -1;
		}
		present_min = USB_MIN;
		present_max = USB_MAX;
		break;
	
	case CLKC_LCD:
		if( (output_freq < LCD_MIN) || (output_freq > LCD_MAX))
		{
			return -1;
		}
		break;
	
	case CLKC_VBUSP:
		if( (output_freq < SYS_MIN) || (output_freq > SYS_MAX))
		{
			return -1;
		}
		present_min = SYS_MIN;
		present_max = SYS_MAX;
		break;
	
	default:
		return -1;
	}
	return 0;
}
	
static void usb_clk_check(void)
{
	if(clk_pll_src[CLKC_USB] == &mips_pll_out)
	{
		if(!( (*bootcr_reg) & BOOTCR_MIPS_ASYNC_MODE) )
		{
			if(get_base_frequency(CLKC_MIPS) == CLK_MHZ(25)) 
				set_pll_div(CLKC_MIPS,CLK_MHZ(150));
		}
	}
}
	
#define    IOMUX_GPIO      0    /* GPIO */
#define    IOMUX_LCD       1    /* LCD */
#define    IOMUX_KEYPAD    2    /* Keypad */
#define    IOMUX_PHY       3    /* PHY */
#define    IOMUX_SSP       4    /* SSP */
#define    IOMUX_MARVEL    5    /* MARVEL */
	
static void avalanche_iomux_enable(unsigned int module)
{
	volatile PIN_SEL_REG_ARRAY_T *pin_sel_array = (PIN_SEL_REG_ARRAY_T*) AVALANCHE_PIN_SEL_BASE;
	
	switch (module)
	{
	case IOMUX_GPIO:
		pin_sel_array->reg[3] &= ~0xff000000;  
		pin_sel_array->reg[3] |=  0x55000000;  
	
		pin_sel_array->reg[4] &= ~0x00003fc0;  
		pin_sel_array->reg[4] |=  0x00001540;  
		break;
	
	case IOMUX_LCD:
		pin_sel_array->reg[7] &= ~0xcc000000;  
		pin_sel_array->reg[7] |=  0x44000000;  
		
		pin_sel_array->reg[8] &= ~0xff3cff3f;  
		pin_sel_array->reg[8] |=  0x55145515;  
	
		pin_sel_array->reg[9] &= ~0x0000ff0f;  
		pin_sel_array->reg[9] |=  0x00005505;  
		break;
	
	case IOMUX_KEYPAD:
		pin_sel_array->reg[6] &= ~0xfcffc000;  
		pin_sel_array->reg[6] |=  0x54554000;  
	
		pin_sel_array->reg[7] &= ~0x003fcc3f;  
		pin_sel_array->reg[7] |=  0x00154415;  
		break;
	
	case IOMUX_PHY:
		pin_sel_array->reg[9]  &= ~0xf03f0000;  
		pin_sel_array->reg[9]  |=  0x50150000;  
	
		pin_sel_array->reg[10] &= ~0x0000003f;  
		pin_sel_array->reg[10] |=  0x00000015;  
		break;
	
	case IOMUX_SSP:
		pin_sel_array->reg[0] &= ~0x000ff000;  
		pin_sel_array->reg[0] |=  0x00055000;  
	
		pin_sel_array->reg[4] |=  0x00000300;
		break;
	
	case IOMUX_MARVEL:
		pin_sel_array->reg[4] &= ~0x00000c00;  
		pin_sel_array->reg[4] |=  0x00000400;
	
		pin_sel_array->reg[8] &= ~0xff3cff3f;
		pin_sel_array->reg[8] |=  0xaa28aa28;
	
		pin_sel_array->reg[9] &= ~0x0000ff0f;
		pin_sel_array->reg[9] |=  0x0000aa0a; 
		break;
	}
	
	return;
}
	
BOARD_INFO_T soc[] = 
{
	/* 1050 */
	{ 
	{
	{AVALANCHE_CPMAC_HW_MODULE_REV,           AVALANCHE_LOW_CPMAC_BASE},
	{AVALANCHE_CPMAC_HW_MODULE_REV,           AVALANCHE_HIGH_CPMAC_BASE},
	{AVALANCHE_USB_1_1_DEVICE_HW_MODULE_REV,  AVALANCHE_USB_SLAVE_CONTROL_BASE},
	{AVALANCHE_USB_1_1_HOST_HW_MODULE_REV,    AVALANCHE_USB_MASTER_CONTROL_BASE},
	{AVALANCHE_LCD_HW_MODULE_REV,             AVALANCHE_LCD_CONTROL_BASE},
	{AVALANCHE_GLCD_HW_MODULE_REV,            AVALANCHE_LCD_CONTROL_BASE},
	{AVALANCHE_KEYPAD_HW_MODULE_REV,          AVALANCHE_KEYPAD_CONTROL_BASE},
	{AVALANCHE_INT_ESWITCH_HW_MODULE_REV,     AVALANCHE_ESWITCH_BASE},
	{AVALANCHE_SSP_HW_MODULE_REV,             AVALANCHE_SSP_BASE},
	{0, 0}
	}
	},
	/* 1055 */
	{ 
	{
	{AVALANCHE_CPMAC_HW_MODULE_REV,           AVALANCHE_LOW_CPMAC_BASE},
	{AVALANCHE_CPMAC_HW_MODULE_REV,           AVALANCHE_HIGH_CPMAC_BASE},
	{AVALANCHE_LCD_HW_MODULE_REV,             AVALANCHE_LCD_CONTROL_BASE},
	{AVALANCHE_GLCD_HW_MODULE_REV,            AVALANCHE_LCD_CONTROL_BASE},
	{AVALANCHE_KEYPAD_HW_MODULE_REV,          AVALANCHE_KEYPAD_CONTROL_BASE},
	{AVALANCHE_INT_ESWITCH_HW_MODULE_REV,     AVALANCHE_ESWITCH_BASE},
	{AVALANCHE_SSP_HW_MODULE_REV,             AVALANCHE_SSP_BASE},
	{0, 0},
	{0, 0},
	{0, 0}
	}
	},
	
	/* 1056 */
	{ 
	{
	{AVALANCHE_CPMAC_HW_MODULE_REV,           AVALANCHE_LOW_CPMAC_BASE},
	{AVALANCHE_LCD_HW_MODULE_REV,             AVALANCHE_LCD_CONTROL_BASE},
	{AVALANCHE_GLCD_HW_MODULE_REV,            AVALANCHE_LCD_CONTROL_BASE},
	{AVALANCHE_KEYPAD_HW_MODULE_REV,          AVALANCHE_KEYPAD_CONTROL_BASE},
	{AVALANCHE_SSP_HW_MODULE_REV,             AVALANCHE_SSP_BASE},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0}
	}
	},
	
	/* 1060 */ /* NOTE: This list is not valid and will change */
	{ 
	{
	{AVALANCHE_CPMAC_HW_MODULE_REV,           AVALANCHE_LOW_CPMAC_BASE},
	{AVALANCHE_CPMAC_HW_MODULE_REV,           AVALANCHE_HIGH_CPMAC_BASE},
	{AVALANCHE_USB_1_1_DEVICE_HW_MODULE_REV,  AVALANCHE_USB_SLAVE_CONTROL_BASE},
	{AVALANCHE_USB_1_1_HOST_HW_MODULE_REV,    AVALANCHE_USB_MASTER_CONTROL_BASE},
	{AVALANCHE_LCD_HW_MODULE_REV,             AVALANCHE_LCD_CONTROL_BASE},
	{AVALANCHE_GLCD_HW_MODULE_REV,            AVALANCHE_LCD_CONTROL_BASE},
	{AVALANCHE_KEYPAD_HW_MODULE_REV,          AVALANCHE_KEYPAD_CONTROL_BASE},
	{AVALANCHE_SSP_HW_MODULE_REV,             AVALANCHE_SSP_BASE},
	{0, 0},
	{0, 0}
	}
	}
};
	
static int avalanche_titan_variant( void )
{
	volatile PIN_SEL_REG_ARRAY_T *pin_sel_array = (PIN_SEL_REG_ARRAY_T*) AVALANCHE_PIN_SEL_BASE;
	unsigned int new_val;
	
	pin_sel_array->reg[12] &= ~0xff000000;    /*Board settings : b 1111 1111 xxxx xxxx xxxx xxxx xxxx xxxx */
	pin_sel_array->reg[12] |=  0xff000000;    /*bit setting    : b 1111 1111 xxxx xxxx xxxx xxxx xxxx xxxx */
	
	/* set GPIO 44 - 47 as input */
	avalanche_gpio_ctrl(44, GPIO_PIN, GPIO_INPUT_PIN);
	avalanche_gpio_ctrl(45, GPIO_PIN, GPIO_INPUT_PIN);
	avalanche_gpio_ctrl(46, GPIO_PIN, GPIO_INPUT_PIN);
	avalanche_gpio_ctrl(47, GPIO_PIN, GPIO_INPUT_PIN);
	
	/* read GPIO to get Titan variant type */
	avalanche_gpio_invalue( &new_val, 1 );
	
	new_val >>= 12;
	new_val &= 0x0f;
	
	switch ( new_val )
	{
	case TITAN_TYPE_1050:
		return TITAN_1050;
	
	case TITAN_TYPE_1055:
		return TITAN_1055;
	
	case TITAN_TYPE_1056:
		return TITAN_1056;
	
	case TITAN_TYPE_1060:
		return TITAN_1060;
	
	default:
		break;
	}
	
	return TITAN_1050;
}
	
int avalanche_get_board_variant( void )
{
	static int init = 0;
	static int board_type;
	
	if ( !init )
	{
		board_type = avalanche_titan_variant();
		init = 1;
	}
	
	return board_type;
}
	
int avalanche_device_prepare(unsigned int module_id, unsigned int base_addr, BOARD_ID board_variant, void *param)
{
	switch (module_id)
	{
	case AVALANCHE_CPMAC_HW_MODULE_REV:
		break;
	
	case AVALANCHE_SSP_HW_MODULE_REV:
		avalanche_iomux_enable(IOMUX_SSP);
		break;
	
	default:
		return -1;
	};

	return 0;
}

typedef struct 
{
	int pinSelReg;
	int shift;
	int func;
} GPIO_CFG;
	
GPIO_CFG gptable[]= {
		      /* PIN_SEL_REG, START_BIT, GPIO_CFG_MUX_VALUE */
		              {4,24,1},
			      {4,26,1},
			      {4,28,1},
			      {4,30,1},
			      {5,6,1},
			      {5,8,1},
			      {5,10,1},
			      {5,12,1},
			      {7,14,3},
			      {7,16,3},
			      {7,18,3},
			      {7,20,3},
			      {7,22,3},
			      {7,26,3},
			      {7,28,3},
			      {7,30,3},
			      {8,0,3},
			      {8,2,3},
			      {8,4,3},
			      {8,10,3},
			      {8,14,3},
			      {8,16,3},
			      {8,18,3},
			      {8,20,3},
			      {9,8,3},
			      {9,10,3},
			      {9,12,3},
			      {9,14,3},
			      {9,18,3},
			      {9,20,3},
			      {9,24,3},
			      {9,26,3},
			      {9,28,3},
			      {9,30,3},
			      {10,0,3},
			      {10,2,3},
			      {10,8,3},
			      {10,10,3},
			      {10,12,3},
			      {10,14,3},
			      {13,12,3},
			      {13,14,3},
			      {13,16,3},
			      {13,18,3},
			      {13,24,3},
			      {13,26,3},
			      {13,28,3},
			      {13,30,3},
			      {14,2,3},
			      {14,6,3},
			      {14,8,3},
			      {14,12,3}
};
			      
void avalanche_gpio_init(void)
{
	unsigned long flags;

	local_irq_save(flags);
	avalanche_reset_ctrl( AVALANCHE_GPIO_RESET_BIT, OUT_OF_RESET );
	local_irq_restore(flags);
}
	
int avalanche_gpio_ctrl(unsigned int gpio_pin, PAL_SYS_GPIO_PIN_MODE_T pin_mode,
				PAL_SYS_GPIO_PIN_DIRECTION_T pin_direction)
{
	int reg_index = 0;
	int mux_status;
	GPIO_CFG  gpio_cfg;
	volatile PIN_SEL_REG_ARRAY_T *pin_sel_array = (PIN_SEL_REG_ARRAY_T*) AVALANCHE_PIN_SEL_BASE;
	volatile TITAN_GPIO_CONTROL_T   *gpio_cntl     = (TITAN_GPIO_CONTROL_T*) TITAN_GPIO_DATA_IN_0;
		
	if (gpio_pin > 51 )
		return(-1);
	
	gpio_cfg = gptable[gpio_pin];
	mux_status = (pin_sel_array->reg[gpio_cfg.pinSelReg - 1] >> gpio_cfg.shift) & 0x3;
	
	if(!((mux_status == 0 /* tri-stated */ ) || (mux_status == gpio_cfg.func /*GPIO functionality*/)))
	{
		return(-1); /* Pin have been configured for non GPIO funcs. */
	}
	
	/* Set the pin to be used as GPIO. */
	pin_sel_array->reg[gpio_cfg.pinSelReg - 1] |= ((gpio_cfg.func & 0x3) << gpio_cfg.shift);
	
	/* Check whether gpio refers to the first GPIO reg or second. */
	if(gpio_pin > 31)
	{
		reg_index = 1;
		gpio_pin -= 32;
	}
	
	if(pin_mode)
		gpio_cntl->enable[reg_index] |=  (1 << gpio_pin); /* Enable */
	else
		gpio_cntl->enable[reg_index] &= ~(1 << gpio_pin);
	
	if(pin_direction)
		gpio_cntl->dir[reg_index] |=  (1 << gpio_pin); /* Input */
	else
		gpio_cntl->dir[reg_index] &= ~(1 << gpio_pin);
	
	return(0);
	
}/* end of function gpio_ctrl */
	
int PAL_sysGpioOutBit(unsigned int gpio_pin, int value)
{
	volatile TITAN_GPIO_CONTROL_T   *gpio_cntl     = (TITAN_GPIO_CONTROL_T*) TITAN_GPIO_DATA_IN_0;
	unsigned int reg_index = 0;
	
	if(gpio_pin > 51)
		return (-1);
	
	if(gpio_pin > 31)
	{
		gpio_pin -= 32;
		reg_index = 1;
	}	
	
	if(value)
		gpio_cntl->data_out[reg_index] |=  (1 << gpio_pin);
	else
		gpio_cntl->data_out[reg_index] &=  ~(1 << gpio_pin);
	
	return (0);
}/* end of function gpio_out */
	
	
	//int PAL_sysGpioInBit(unsigned int gpio_pin)
int avalanche_gpio_inbit(unsigned int gpio_pin)
{
	volatile TITAN_GPIO_CONTROL_T *gpio_cntl = 
			(TITAN_GPIO_CONTROL_T*) TITAN_GPIO_DATA_IN_0;
	unsigned int reg_index = 0;
	
	if(gpio_pin > 51)
		return (-1);
	
	if(gpio_pin > 31)
	{
		gpio_pin -= 32;
		reg_index = 1;
	}	
	
	return ((gpio_cntl->data_in[reg_index] >> gpio_pin) & 0x1);
	
}/* end of function gpio_in */
	
	
int avalanche_gpio_outvalue(unsigned int out_val, unsigned int out_mask, unsigned int reg_index)
{
	volatile TITAN_GPIO_CONTROL_T *gpio_cntl = 
			(TITAN_GPIO_CONTROL_T*) TITAN_GPIO_DATA_IN_0;
	
	if(reg_index > 1)
		return (-1);
	
	gpio_cntl->data_out[reg_index] &= ~out_mask;
	gpio_cntl->data_out[reg_index] |= out_val & out_mask;
		
	return (-1);	    
}
	
int avalanche_gpio_invalue(unsigned int *in_val, unsigned int reg_index)
{
	volatile TITAN_GPIO_CONTROL_T *gpio_cntl = 
			(TITAN_GPIO_CONTROL_T*) TITAN_GPIO_DATA_IN_0;
	
	if(reg_index > 1)
		return (-1);
	
	*in_val = gpio_cntl->data_in[reg_index];
	
	return (0);
}
