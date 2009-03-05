/*
 * MDIO driver for the MIPS TI TITAN
 *
 * Copyright: (C) 2006 Texas Instruments Inc.
 *
 * Author: Manish Lachwani <mlachwani@mvista.com>
 * Copyright: (C) 2006 MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
	
#define COMMON_RANDOM_MAX        0xFFFF
static unsigned long random_seed = 1;
	
static int cpsw_hal_common_random(void)
{
	random_seed = random_seed * 1103515245 + 12345;
	return ((unsigned int) (random_seed/65536) % (COMMON_RANDOM_MAX + 1));
}
	
static int cpsw_hal_common_random_range(int min, int max)
{
	int tmp;

	tmp =  cpsw_hal_common_random();
	tmp %= ((max-min)+1);
	tmp += min;
	return(tmp);
}
	
typedef struct _phy_device
{
	unsigned int miibase;
	unsigned int inst;
	unsigned int phy_state;
	unsigned int mdix_mask;
	unsigned int phy_mask;
	unsigned int mlink_mask;
	unsigned int phy_mode;
} phy_device;

void _cpsw_halcommon_mii_mdio_initstate(phy_device *phy_dev);
void _cpsw_halcommon_mii_mdio_finding_state(phy_device *phy_dev);
void _cpsw_halcommon_mii_mdio_found_state(phy_device *phy_dev);
void _cpsw_halcommon_mii_mdio_nway_start_state(phy_device *phy_dev);
void _cpsw_halcommon_mii_mdio_nway_wait_state(phy_device *phy_dev);
void _cpsw_halcommon_mii_mdio_link_wait_state(phy_device *phy_dev);
void _cpsw_halcommon_mii_mdio_linked_state(phy_device *phy_dev);
void _cpsw_halcommon_mii_mdio_loopback_state(phy_device *phy_dev);
void _cpsw_halcommon_mii_mdio_default_state(phy_device *phy_dev);

void _cpsw_halcommon_mii_mdio_wait_for_access_complete(phy_device *phy_dev);
void _cpsw_halcommon_mii_mdio_useraccess(phy_device *phy_dev, unsigned int method, unsigned int regadr, unsigned int phyadr, unsigned int data);
void _cpsw_halcommon_mii_mdio_disable_phy(phy_device *phy_dev,unsigned int PhyNum);
void _cpsw_halcommon_mii_mdio_phy_timeout(phy_device *phy_dev);
void _cpsw_halcommon_mii_mdio_reset_phy(phy_device *phy_dev,unsigned int PhyNum);
void _cpsw_halcommon_mii_mdio_dump_phy(phy_device *phy_dev, unsigned int p);
void _cpsw_halcommon_mii_mdio_dump_state(phy_device *phy_dev);
void _cpsw_halcommon_mii_mdio_dump_phy_detailed(phy_device *phy_dev);
	
#include "ti_titan_mdio.h"

#define PHY_NOT_FOUND	0xFFFF    /*  Used in Phy Detection */
#define PHY_DEV_OFFSET	(0)
#define PHY_DEV_SIZE	(5)
#define PHY_DEV_MASK	(0x1f<<PHY_DEV_OFFSET)
	
#define PHY_STATE_OFFSET	(PHY_DEV_SIZE+PHY_DEV_OFFSET)
#define PHY_STATE_SIZE		(5)
#define PHY_STATE_MASK		(0x1f<<PHY_STATE_OFFSET)
#define INIT		(1<<PHY_STATE_OFFSET)
#define FINDING		(2<<PHY_STATE_OFFSET)
#define FOUND		(3<<PHY_STATE_OFFSET)
#define NWAY_START	(4<<PHY_STATE_OFFSET)
#define NWAY_WAIT	(5<<PHY_STATE_OFFSET)
#define LINK_WAIT	(6<<PHY_STATE_OFFSET)
#define LINKED		(7<<PHY_STATE_OFFSET)
#define LOOPBACK	(8<<PHY_STATE_OFFSET)
	
#define PHY_SPEED_OFFSET	(PHY_STATE_OFFSET+PHY_STATE_SIZE)
#define PHY_SPEED_SIZE		(1)
#define PHY_SPEED_MASK		(1<<PHY_SPEED_OFFSET)
#define PHY_DUPLEX_OFFSET	(PHY_SPEED_OFFSET+PHY_SPEED_SIZE)
#define PHY_DUPLEX_SIZE		(1)
#define PHY_DUPLEX_MASK		(1<<PHY_DUPLEX_OFFSET)
#define PHY_TIM_OFFSET		(PHY_DUPLEX_OFFSET+PHY_DUPLEX_SIZE)
#define PHY_TIM_SIZE		(10)
#define PHY_TIM_MASK		(0x3ff<<PHY_TIM_OFFSET)
	
#define PHY_FIND_TO 		(2<<PHY_TIM_OFFSET)
#define PHY_RECK_TO		(200<<PHY_TIM_OFFSET)
#define PHY_LINK_TO		(500<<PHY_TIM_OFFSET)
#define PHY_NWST_TO		(500<<PHY_TIM_OFFSET)
#define PHY_NWDN_TO		(800<<PHY_TIM_OFFSET)
#define PHY_MDIX_TO		(274<<PHY_TIM_OFFSET)
	
#define PHY_SMODE_OFFSET	(PHY_TIM_OFFSET+PHY_TIM_SIZE)
#define PHY_SMODE_SIZE		(6)
#define PHY_SMODE_MASK		(0x3f<<PHY_SMODE_OFFSET)
#define SMODE_LPBK		(0x20<<PHY_SMODE_OFFSET)
#define SMODE_AUTO		(0x10<<PHY_SMODE_OFFSET)
#define SMODE_FD100		(0x08<<PHY_SMODE_OFFSET)
#define SMODE_HD100		(0x04<<PHY_SMODE_OFFSET)
#define SMODE_FD10		(0x02<<PHY_SMODE_OFFSET)
#define SMODE_HD10		(0x01<<PHY_SMODE_OFFSET)
#define SMODE_ALL		(0x1f<<PHY_SMODE_OFFSET)
	
#define PHY_CHNG_OFFSET		(PHY_SMODE_OFFSET+PHY_SMODE_SIZE)
#define PHY_CHNG_SIZE		(1)
#define PHY_CHNG_MASK		(1<<PHY_CHNG_OFFSET)
#define PHY_CHANGE		(1<<PHY_CHNG_OFFSET)

#define PHY_TIMEDOUT_OFFSET	(PHY_CHNG_OFFSET+PHY_CHNG_SIZE)
#define PHY_TIMEDOUT_SIZE	(1)     /*  30 Bits used */
#define PHY_TIMEDOUT_MASK	(1<<PHY_TIMEDOUT_OFFSET)
#define PHY_MDIX_SWITCH		(1<<PHY_TIMEDOUT_OFFSET)
	
#define PHY_MDIX_OFFSET		(PHY_TIMEDOUT_OFFSET+PHY_TIMEDOUT_SIZE)
#define PHY_MDIX_SIZE		(1)     /*  31 Bits used */
#define PHY_MDIX_MASK		(1<<PHY_MDIX_OFFSET)
#define PHY_MDIX		(1<<PHY_MDIX_OFFSET)
	
static char *lstate[]={"NULL","INIT","FINDING","FOUND","NWAY_START","NWAY_WAIT","LINK_WAIT","LINKED","LOOPBACK"};
	
#define TI_MDIOALIVE		MDIO_ALIVE(phy_dev->miibase)
#define TI_MDIOCONTROL		MDIO_CONTROL(phy_dev->miibase)
#define TI_MDIOLINK		MDIO_LINK(phy_dev->miibase)
#define TI_MDIOLINKINT		MDIO_LINKINT(phy_dev->miibase)
#define TI_MDIOUSERACCESS	MDIO_USERACCESS(phy_dev->miibase, phy_dev->inst)
#define TI_MDIOUSERPHYSEL	MDIO_USERPHYSEL(phy_dev->miibase, phy_dev->inst)
#define TI_MDIOVER		MDIO_VER(phy_dev->miibase)
	
#ifndef VOLATILE32
#define VOLATILE32(addr) (*((volatile unsigned int *)(addr)))
#endif
	
void cpsw_halcommon_mii_mdioClose(phy_device *phy_dev, int Full)
{
	/* Do Nothing */
}
	
int cpsw_hal_common_mii_mdio_init(phy_device *phy_dev, unsigned int miibase, 
				unsigned int inst, unsigned int phy_mask, 
				unsigned int mlink_mask, unsigned int reset_reg, 
				unsigned int reset_bit, unsigned int mdio_bus_freq, 
				unsigned int mdio_clock_freq)
{
	unsigned int highest_channel;
	unsigned int control_state;
	unsigned int *phy_state = &phy_dev->phy_state;
	unsigned int clkdiv;
	
	phy_dev->miibase   = miibase;
	phy_dev->inst      = inst;
	phy_dev->phy_mask   = phy_mask;
	phy_dev->mlink_mask = mlink_mask;
	phy_dev->mdix_mask  = phy_mask; 
	
	*phy_state &= ~PHY_MDIX_MASK;   /* Set initial State to MDI */
	
	highest_channel = (TI_MDIOCONTROL & MDIO_CONTROL_HIGHEST_USER_CHANNEL) > 8;
	if(inst > highest_channel)
		return(highest_channel);
	
	if (mdio_clock_freq)                                       
		clkdiv = (mdio_bus_freq / mdio_clock_freq) - 1;         
	else                                                   
		clkdiv = 0xFF;                 
	
	control_state  = MDIO_CONTROL_ENABLE;
	control_state |= ((clkdiv) & MDIO_CONTROL_CLKDIV);
	
	TI_MDIOCONTROL = control_state;  /* Enable MDIO   */
	*phy_state=INIT;
	
	_cpsw_halcommon_mii_mdio_dump_state(phy_dev);
	return(0);
}
	
void cpsw_halcommon_mii_mdio_setphymode(phy_device *phy_dev,unsigned int phy_mode)
{
	unsigned int *phy_state = &phy_dev->phy_state;
	unsigned int current_state;
	
	phy_dev->phy_mode = phy_mode;   
	*phy_state&=~PHY_SMODE_MASK;
	
	if (phy_mode & NWAY_LPBK)  
		*phy_state |= SMODE_LPBK;

	if (phy_mode & NWAY_AUTO)  
		*phy_state |= SMODE_AUTO;

	if (phy_mode & NWAY_FD100) 
		*phy_state |= SMODE_FD100;

	if (phy_mode & NWAY_HD100) 
		*phy_state |= SMODE_HD100;

	if (phy_mode & NWAY_FD10)  
		*phy_state |= SMODE_FD10;

	if (phy_mode & NWAY_HD10)  
		*phy_state |= SMODE_HD10;
	
	current_state = *phy_state & PHY_STATE_MASK;
	if ((current_state == NWAY_START)||
		(current_state == NWAY_WAIT) ||
		(current_state == LINK_WAIT) ||
		(current_state == LINKED)    ||
		(current_state == LOOPBACK)    )
			*phy_state = (*phy_state & ~PHY_STATE_MASK) | FOUND | PHY_CHANGE;

	_cpsw_halcommon_mii_mdio_dump_state(phy_dev);
}
	
int cpsw_halcommon_mii_mdio_tic(phy_device *phy_dev)
{
	unsigned int *phy_state = &phy_dev->phy_state;
	unsigned int  current_state;
	
	current_state=*phy_state;
	switch(current_state & PHY_STATE_MASK) {
	case INIT:       
		_cpsw_halcommon_mii_mdio_initstate(phy_dev); 
		break;

	case FINDING:    
		_cpsw_halcommon_mii_mdio_finding_state(phy_dev);   
		break;

	case FOUND:     
		 _cpsw_halcommon_mii_mdio_found_state(phy_dev);
		break;

	case NWAY_START: 
		_cpsw_halcommon_mii_mdio_nway_start_state(phy_dev); 
		break;

	case NWAY_WAIT:  
		_cpsw_halcommon_mii_mdio_nway_wait_state(phy_dev);
		break;

	case LINK_WAIT:  
		_cpsw_halcommon_mii_mdio_link_wait_state(phy_dev);
		break;

	case LINKED:     
		_cpsw_halcommon_mii_mdio_linked_state(phy_dev);
		break;

	case LOOPBACK:   
		_cpsw_halcommon_mii_mdio_loopback_state(phy_dev);
		break;

	default:
		_cpsw_halcommon_mii_mdio_default_state(phy_dev);
		break;
	}
	
	if ((current_state & ~PHY_TIM_MASK) != (*phy_state&~PHY_TIM_MASK))
		_cpsw_halcommon_mii_mdio_dump_state(phy_dev);
	
	if(*phy_state & PHY_MDIX_SWITCH) {
		unsigned int mdix;
	
		*phy_state &= ~PHY_MDIX_SWITCH;  /* Clear Mdix Flip indicator */
	
		if(*phy_state & PHY_MDIX)
			mdix = 1;
		else
			mdix = 0;
	
		return(_MIIMDIO_MDIXFLIP|mdix);
	}
	
	if (*phy_state & PHY_CHNG_MASK) {
		*phy_state &= ~PHY_CHNG_MASK;
		return(1);
	}
	else
		return(0);
}
	
int cpsw_halcommon_mii_mdio_get_duplex(phy_device *phy_dev)
{
	unsigned int *phy_state = &phy_dev->phy_state;

	return((*phy_state & PHY_DUPLEX_MASK) ? 1:0);  /* return 0 or a 1  */
}
	
int cpsw_halcommon_mii_mdio_get_speed(phy_device *phy_dev)
{
	unsigned int *phy_state = &phy_dev->phy_state;

	return(*phy_state & PHY_SPEED_MASK);
}
	
int cpsw_halcommon_mii_mdio_getphy_num(phy_device *phy_dev)
{
	unsigned int *phy_state = &phy_dev->phy_state;

	return((*phy_state & PHY_DEV_MASK)>>PHY_DEV_OFFSET);
}
	
int cpsw_halcommon_mii_mdio_getloopback(phy_device *phy_dev)
{
	unsigned int *phy_state = &phy_dev->phy_state;

	return((*phy_state & PHY_STATE_MASK)==LOOPBACK);
}
	
void cpsw_halcommon_mii_mdio_link_change(phy_device *phy_dev)
{
	unsigned int *phy_state = &phy_dev->phy_state;
	unsigned int phy_num, phy_status;
	
	phy_num=(*phy_state & PHY_DEV_MASK) >> PHY_DEV_OFFSET;
	
	if (cpsw_halcommon_mii_mdio_getlinked(phy_dev)) {
		phy_status = _cpsw_halcommon_mii_mdio_user_accessread(phy_dev, PHY_STATUS_REG, phy_num);
	
		if ((phy_status&MII_PHY_LINKED)==0) {
			*phy_state&=~(PHY_TIM_MASK|PHY_STATE_MASK);
			if (*phy_state&SMODE_AUTO) {
				_cpsw_halcommon_mii_mdiouser_accesswrite(phy_dev, PHY_CONTROL_REG, phy_num, MII_AUTO_NEGOTIATE_EN|MII_RENEGOTIATE);
				*phy_state|=PHY_CHANGE|PHY_NWST_TO|NWAY_START;
			}
			else {
				*phy_state|=PHY_CHANGE|PHY_LINK_TO|LINK_WAIT;
			}
		}
	}
}
	
int cpsw_halcommon_mii_mdio_getlinked(phy_device *phy_dev)
{
	unsigned int *phy_state = &phy_dev->phy_state;

	return((*phy_state & PHY_STATE_MASK) == LINKED);
}
	
void cpsw_halcommon_mii_mdio_getver(unsigned int miibase, unsigned int *mod_id,  
					unsigned int *rev_maj,  unsigned int *rev_min)
{
	unsigned int ver;
	
	ver = MDIO_VER(miibase);
	
	*mod_id  = (ver & MDIO_VER_MODID) >> 16;
	*rev_maj = (ver & MDIO_VER_REVMAJ) >> 8;
	*rev_min = (ver & MDIO_VER_REVMIN);
}
	
int cpsw_halcommon_mii_mdio_getphydevsize(void)
{
	return(sizeof(phy_device));
}
	
int _cpsw_halcommon_mii_mdio_mdix_supported(phy_device *phy_dev)
{
	unsigned int *phy_state = &phy_dev->phy_state;
	unsigned int phy_num;
	
	if((phy_dev->phy_mode & NWAY_AUTOMDIX) == 0)
		return(0);  
	
	phy_num=(*phy_state & PHY_DEV_MASK) >> PHY_DEV_OFFSET;

	if( ((1<<phy_num) & phy_dev->mdix_mask) == 0)
		return(0);  
	
	return(1);
}
	
void _cpsw_halcommon_mii_mdio_mdix_delay(phy_device *phy_dev)
{
	int delay;
	unsigned int *phy_state = &phy_dev->phy_state;
	unsigned int phy_num;
	
	phy_num=(*phy_state&PHY_DEV_MASK)>>PHY_DEV_OFFSET;
	
	if(_cpsw_halcommon_mii_mdio_mdix_supported(phy_dev) == 0)
		return;  
	
	delay = cpsw_hal_common_random_range(_AUTOMDIX_DELAY_MIN, _AUTOMDIX_DELAY_MAX);
	delay /= 10;
	
	delay += (PHY_MDIX_TO>>PHY_TIM_OFFSET);
	
	*phy_state &= ~(PHY_TIM_MASK);  /* Clear current Time out value */
	*phy_state |=  (delay<<PHY_TIM_OFFSET);     /* Set new value */
}
	
void _cpsw_halcommon_mii_mdio_dump_state(phy_device *phy_dev)
{
	unsigned int state    = phy_dev->phy_state;
	
	printk("Phy: %d, ",(state&PHY_DEV_MASK)>>PHY_DEV_OFFSET);
	printk("State: %d/%s, ",(state&PHY_STATE_MASK)>>PHY_STATE_OFFSET,lstate[(state&PHY_STATE_MASK)>>PHY_STATE_OFFSET]);
	printk("Speed: %d, ",(state&PHY_SPEED_MASK)>>PHY_SPEED_OFFSET);
	printk("Dup: %d, ",(state&PHY_DUPLEX_MASK)>>PHY_DUPLEX_OFFSET);
	printk("Tim: %d, ",(state&PHY_TIM_MASK)>>PHY_TIM_OFFSET);
	printk("SMode: %d, ",(state&PHY_SMODE_MASK)>>PHY_SMODE_OFFSET);
	printk("Chng: %d",(state&PHY_CHNG_MASK)>>PHY_CHNG_OFFSET);
	printk("\n");
	
	if (((state&PHY_STATE_MASK)!=FINDING)&&((state&PHY_STATE_MASK)!=INIT))
		_cpsw_halcommon_mii_mdio_dump_phy(phy_dev, (state&PHY_DEV_MASK)>>PHY_DEV_OFFSET);
}
	
void _cpsw_halcommon_mii_mdio_disable_phy(phy_device *phy_dev,unsigned int phy_num)
{
	_cpsw_halcommon_mii_mdiouser_accesswrite(phy_dev, PHY_CONTROL_REG, phy_num, MII_PHY_ISOLATE|MII_PHY_PDOWN);
	
}
	
void _cpsw_halcommon_mii_mdio_initstate(phy_device *phy_dev)
{
	unsigned int *phy_state = &phy_dev->phy_state;
	unsigned int current_state;
	
	current_state=*phy_state;
	current_state=(current_state&~PHY_TIM_MASK)|(PHY_FIND_TO);
	current_state=(current_state&~PHY_STATE_MASK)|(FINDING);
	current_state=(current_state&~PHY_SPEED_MASK);
	current_state=(current_state&~PHY_DUPLEX_MASK);
	current_state|=PHY_CHANGE;
	
	*phy_state=current_state;
}
	
void _cpsw_halcommon_mii_mdio_finding_state(phy_device *phy_dev)
{
	unsigned int *phy_state = &phy_dev->phy_state;
	unsigned int phy_mask  = phy_dev->phy_mask;
	unsigned int phy_num,i,j,phy_acks;
	
	phy_num = PHY_NOT_FOUND;
	
	if (*phy_state&PHY_TIM_MASK) {
		*phy_state=(*phy_state&~PHY_TIM_MASK)|((*phy_state&PHY_TIM_MASK)-(1<<PHY_TIM_OFFSET));
	}
	else {
		phy_acks=TI_MDIOALIVE;
		phy_acks&=phy_mask;   /* Only interested in 'our' Phys */
	
		for(i=0,j=1;(i<32)&&((j&phy_acks)==0);i++,j<<=1);
	
		if ((phy_acks)&&(i<32)) 
			phy_num=i;

		if (phy_num!=PHY_NOT_FOUND) {
			*phy_state=(*phy_state&~PHY_DEV_MASK)|((phy_num&PHY_DEV_MASK)<<PHY_DEV_OFFSET);
			*phy_state=(*phy_state&~PHY_STATE_MASK)|(FOUND);
			*phy_state|=PHY_CHANGE;
		}
		else {
			*phy_state|=PHY_RECK_TO;  /* Set Timer */
		}
	}
}

void _cpsw_halcommon_mii_mdio_found_state(phy_device *phy_dev)
{
	unsigned int *phy_state  = &phy_dev->phy_state;
	unsigned int phy_mask   = phy_dev->phy_mask;
	unsigned int mlink_mask = phy_dev->mlink_mask;
	unsigned int phy_num,phy_status,nway_advertise,m,phynum,i,j,phy_acks;
	unsigned int phy_sel;

	if ((*phy_state&PHY_SMODE_MASK)==0)
		return;
	
	phy_num=(*phy_state&PHY_DEV_MASK)>>PHY_DEV_OFFSET;
	
	phy_acks=TI_MDIOALIVE;
	phy_acks&=phy_mask;   /* Only interested in 'our' Phys */
	
	for(phynum=0,j=1;phynum<32;phynum++,j<<=1) {
		if (phy_acks&j) {
			if (phynum!=phy_num)  /* Do not disabled Found Phy */
				_cpsw_halcommon_mii_mdio_disable_phy(phy_dev,phynum);
		}
	}
	
	_cpsw_halcommon_mii_mdio_reset_phy(phy_dev,phy_num);
	
	phy_sel=phy_num;  
	
	if ((1 << phy_num) & mlink_mask)
		phy_sel |= MDIO_USERPHYSEL_LINKSEL;
	
	TI_MDIOUSERPHYSEL = phy_sel; 
	
	phy_status = _cpsw_halcommon_mii_mdio_user_accessread(phy_dev, PHY_STATUS_REG, phy_num);
	
	if (*phy_state&SMODE_LPBK) {
		_cpsw_halcommon_mii_mdiouser_accesswrite(phy_dev, PHY_CONTROL_REG, phy_num, MII_PHY_LOOP|MII_PHY_FD);
		_cpsw_halcommon_mii_mdio_user_accessread(phy_dev, PHY_STATUS_REG, phy_num);
		*phy_state=(*phy_state&~PHY_STATE_MASK)|(LOOPBACK);
		*phy_state|=PHY_CHANGE;
		return;
	}
	
	
	nway_advertise=MII_NWAY_SEL;

	if (*phy_state&SMODE_FD100)
		nway_advertise|=MII_NWAY_FD100;
	if (*phy_state&SMODE_HD100) 
		nway_advertise|=MII_NWAY_HD100;
	if (*phy_state&SMODE_FD10)
		nway_advertise|=MII_NWAY_FD10;
	if (*phy_state&SMODE_HD10)
		nway_advertise|=MII_NWAY_HD10;
	
	*phy_state&=~(PHY_TIM_MASK|PHY_STATE_MASK);
	if ((phy_status&MII_NWAY_CAPABLE)&&(*phy_state&SMODE_AUTO))   {
		_cpsw_halcommon_mii_mdiouser_accesswrite(phy_dev, NWAY_ADVERTIZE_REG, phy_num, nway_advertise);
	
		_cpsw_halcommon_mii_mdiouser_accesswrite(phy_dev, PHY_CONTROL_REG, phy_num, MII_AUTO_NEGOTIATE_EN);
	
		_cpsw_halcommon_mii_mdiouser_accesswrite(phy_dev, PHY_CONTROL_REG, phy_num, MII_AUTO_NEGOTIATE_EN|MII_RENEGOTIATE);
	
		*phy_state|=PHY_CHANGE|PHY_NWST_TO|NWAY_START;
	}
	else {
		*phy_state &=~ SMODE_AUTO;   
		m = nway_advertise;
		for(j=0x8000,i=0;(i<16)&&((j&m)==0);i++,j>>=1);
		m = j;
		j = 0;

		if (m&(MII_NWAY_FD100|MII_NWAY_HD100)) {
			j=MII_PHY_100;
			m&=(MII_NWAY_FD100|MII_NWAY_HD100);
		}
		if (m&(MII_NWAY_FD100|MII_NWAY_FD10))
			j |= MII_PHY_FD;

		_cpsw_halcommon_mii_mdiouser_accesswrite(phy_dev, PHY_CONTROL_REG, phy_num, j);
		*phy_state&=~PHY_SPEED_MASK;
		if (j&MII_PHY_100)
			*phy_state|=(1<<PHY_SPEED_OFFSET);

		*phy_state&=~PHY_DUPLEX_MASK;
		if (j&MII_PHY_FD)
			*phy_state|=(1<<PHY_DUPLEX_OFFSET);
	
		*phy_state|=PHY_CHANGE|PHY_LINK_TO|LINK_WAIT;
	}
	
	_cpsw_halcommon_mii_mdio_mdix_delay(phy_dev);  /* If AutoMdix add delay */
}
	
void _cpsw_halcommon_mii_mdio_reset_phy(phy_device *phy_dev,unsigned int phy_num)
{
	unsigned short phy_control_reg;
	
	_cpsw_halcommon_mii_mdiouser_accesswrite(phy_dev, PHY_CONTROL_REG, phy_num, MII_PHY_RESET);
	do {
		phy_control_reg = _cpsw_halcommon_mii_mdio_user_accessread(phy_dev, PHY_CONTROL_REG, phy_num);
	}
	while (phy_control_reg & MII_PHY_RESET); /* Wait for Reset to clear */
}
	
void _cpsw_halcommon_mii_mdio_nway_start_state(phy_device *phy_dev)
{
	unsigned int *phy_state = &phy_dev->phy_state;
	unsigned int phy_num,phy_mode;
	
	phy_num=(*phy_state&PHY_DEV_MASK)>>PHY_DEV_OFFSET;
	phy_mode=_cpsw_halcommon_mii_mdio_user_accessread(phy_dev, PHY_CONTROL_REG, phy_num);
	
	if((phy_mode&MII_RENEGOTIATE)==0) {
		_cpsw_halcommon_mii_mdio_user_accessread(phy_dev, PHY_STATUS_REG, phy_num);
		*phy_state&=~(PHY_STATE_MASK|PHY_TIM_MASK);
		*phy_state|=PHY_CHANGE|NWAY_WAIT|PHY_NWDN_TO;
	}
	else {
		if (*phy_state&PHY_TIM_MASK)
			*phy_state=(*phy_state&~PHY_TIM_MASK)|((*phy_state&PHY_TIM_MASK)-(1<<PHY_TIM_OFFSET));
		else
			_cpsw_halcommon_mii_mdio_phy_timeout(phy_dev);
	}
}
	
void _cpsw_halcommon_mii_mdio_nway_wait_state(phy_device *phy_dev)
{
	unsigned int *phy_state = &phy_dev->phy_state;
	unsigned int  phy_num,phy_status,nway_advertise,nway_readvertise,neg_mode,i,j;
	
	phy_num=(*phy_state&PHY_DEV_MASK)>>PHY_DEV_OFFSET;
	
	phy_status=_cpsw_halcommon_mii_mdio_user_accessread(phy_dev, PHY_STATUS_REG, phy_num);
	
	if (phy_status&MII_NWAY_COMPLETE) {
		*phy_state|=PHY_CHANGE;
		*phy_state&=~PHY_SPEED_MASK;
		*phy_state&=~PHY_DUPLEX_MASK;
	
		nway_advertise = _cpsw_halcommon_mii_mdio_user_accessread(phy_dev, NWAY_ADVERTIZE_REG, phy_num);
		nway_readvertise = _cpsw_halcommon_mii_mdio_user_accessread(phy_dev, NWAY_REMADVERTISE_REG, phy_num);
	
		neg_mode = nway_advertise & nway_readvertise;
	
		neg_mode &= (MII_NWAY_FD100|MII_NWAY_HD100|MII_NWAY_FD10|MII_NWAY_HD10);
	
		if (neg_mode==0) {
			neg_mode=(MII_NWAY_HD100|MII_NWAY_HD10)&nway_advertise;
		}

		for(j=0x8000,i=0;(i<16)&&((j&neg_mode)==0);i++,j>>=1);

		neg_mode=j;
		if (neg_mode!=0) {
			if (phy_status&MII_PHY_LINKED)
				*phy_state=(*phy_state&~PHY_STATE_MASK)|LINKED;
			else
				*phy_state=(*phy_state&~PHY_STATE_MASK)|LINK_WAIT;

			if (neg_mode&(MII_NWAY_FD100|MII_NWAY_HD100))
				*phy_state=(*phy_state&~PHY_SPEED_MASK)|(1<<PHY_SPEED_OFFSET);

			if (neg_mode&(MII_NWAY_FD100|MII_NWAY_FD10))
				*phy_state=(*phy_state&~PHY_DUPLEX_MASK)|(1<<PHY_DUPLEX_OFFSET);
		}
	}
	else {
		if (*phy_state&PHY_TIM_MASK)
			*phy_state=(*phy_state&~PHY_TIM_MASK)|((*phy_state&PHY_TIM_MASK)-(1<<PHY_TIM_OFFSET));
		else
			_cpsw_halcommon_mii_mdio_phy_timeout(phy_dev);
	}
}
	
void _cpsw_halcommon_mii_mdio_link_wait_state(phy_device *phy_dev)
{
	unsigned int *phy_state = &phy_dev->phy_state;
	unsigned int  phy_status;
	unsigned int  phy_num;
	
	phy_num=(*phy_state&PHY_DEV_MASK)>>PHY_DEV_OFFSET;
	
	phy_status=_cpsw_halcommon_mii_mdio_user_accessread(phy_dev, PHY_STATUS_REG, phy_num);
	
	if (phy_status&MII_PHY_LINKED)
	{
		*phy_state=(*phy_state&~PHY_STATE_MASK)|LINKED;
		*phy_state|=PHY_CHANGE;
	}
	else
	{
		if (*phy_state&PHY_TIM_MASK)
			*phy_state=(*phy_state&~PHY_TIM_MASK)|((*phy_state&PHY_TIM_MASK)-(1<<PHY_TIM_OFFSET));
		else
			_cpsw_halcommon_mii_mdio_phy_timeout(phy_dev);
	}
}
	
void _cpsw_halcommon_mii_mdio_phy_timeout(phy_device *phy_dev)
{
	unsigned int *phy_state;
	
	if(_cpsw_halcommon_mii_mdio_mdix_supported(phy_dev) == 0)
		return;  /* AutoMdix not supported */
	
	phy_state = &phy_dev->phy_state;
	
	*phy_state|=PHY_MDIX_SWITCH;
	
	if(*phy_state & PHY_MDIX)
		*phy_state &= ~PHY_MDIX_MASK;       /* Current State is MDIX, set to MDI */
	else
		*phy_state |=  PHY_MDIX_MASK;      /* Current State is MDI, set to MDIX */
	
	*phy_state=(*phy_state&~PHY_STATE_MASK)|(FOUND);
}
	
void _cpsw_halcommon_mii_mdio_loopback_state(phy_device *phy_dev)
{
	return;
}
	
void _cpsw_halcommon_mii_mdio_linked_state(phy_device *phy_dev)
{
	unsigned int *phy_state = &phy_dev->phy_state;
	unsigned int  phy_num   = (*phy_state&PHY_DEV_MASK)>>PHY_DEV_OFFSET;
	
	if (TI_MDIOLINK&(1<<phy_num))
		return;  /* if still Linked, exit*/
	
	*phy_state&=~(PHY_STATE_MASK|PHY_TIM_MASK);

	if (*phy_state&SMODE_AUTO)
		*phy_state|=PHY_CHANGE|NWAY_WAIT|PHY_NWDN_TO;
	else
		*phy_state|=PHY_CHANGE|PHY_LINK_TO|LINK_WAIT;
	
	_cpsw_halcommon_mii_mdio_mdix_delay(phy_dev);  /* If AutoMdix add delay */
}
	
void _cpsw_halcommon_mii_mdio_default_state(phy_device *phy_dev)
{
	unsigned int *phy_state = &phy_dev->phy_state;
	*phy_state|=PHY_CHANGE;
}
	
void _cpsw_halcommon_mii_mdio_dump_phy(phy_device *phy_dev, unsigned int p)
{
	unsigned int j,n,phy_acks;
	unsigned int PhyRegAddr;
	unsigned int phy_num;
	unsigned int phy_mask  = phy_dev->phy_mask;
	
	phy_acks=TI_MDIOALIVE;
	phy_acks&=phy_mask;   
	
	for(phy_num=0,j=1;phy_num<32;phy_num++,j<<=1) {
		if (phy_acks&j) {
			printk("%2d%s:",phy_num,(phy_num==p)?">":" ");
			for(PhyRegAddr=0;PhyRegAddr<6;PhyRegAddr++) {
				n = _cpsw_halcommon_mii_mdio_user_accessread(phy_dev, PhyRegAddr, phy_num);
				printk(" %04x",n&0x0ffff);
			}
			printk("\n");
		}
	}
	_cpsw_halcommon_mii_mdio_dump_phy_detailed(phy_dev);
}
	
void _cpsw_halcommon_mii_mdio_dump_phy_detailed(phy_device *phy_dev)
{
	unsigned int *phy_state = &phy_dev->phy_state;
	unsigned int  phy_num;
	int reg_data;
	
	phy_num=(*phy_state&PHY_DEV_MASK)>>PHY_DEV_OFFSET;
	
	reg_data = _cpsw_halcommon_mii_mdio_user_accessread(phy_dev, 0, phy_num);
	printk("PhyControl: %04X, Lookback=%s, Speed=%s, Duplex=%s\n",
			reg_data,
			reg_data&MII_PHY_LOOP?"On":"Off",
			reg_data&MII_PHY_100?"100":"10",
			reg_data&MII_PHY_FD?"Full":"Half");
			reg_data = _cpsw_halcommon_mii_mdio_user_accessread(phy_dev, 1, phy_num);
	printk("phy_status: %04X, AutoNeg=%s, Link=%s\n",
			reg_data,
			reg_data&MII_NWAY_COMPLETE?"Complete":"NotComplete",
			reg_data&MII_PHY_LINKED?"Up":"Down");
			reg_data = _cpsw_halcommon_mii_mdio_user_accessread(phy_dev, 4, phy_num);

	printk("PhyMyCapability: %04X, 100FD=%s, 100HD=%s, 10FD=%s, 10HD=%s\n",
			reg_data,
			reg_data&MII_NWAY_FD100?"Yes":"No",
			reg_data&MII_NWAY_HD100?"Yes":"No",
			reg_data&MII_NWAY_FD10?"Yes":"No",
			reg_data&MII_NWAY_HD10?"Yes":"No");
	
	reg_data = _cpsw_halcommon_mii_mdio_user_accessread(phy_dev, 5, phy_num);

	printk("PhyPartnerCapability: %04X, 100FD=%s, 100HD=%s, 10FD=%s, 10HD=%s\n",
			reg_data,
			reg_data&MII_NWAY_FD100?"Yes":"No",
			reg_data&MII_NWAY_HD100?"Yes":"No",
			reg_data&MII_NWAY_FD10?"Yes":"No",
			reg_data&MII_NWAY_HD10?"Yes":"No");
}

unsigned int _cpsw_halcommon_mii_mdio_user_accessread(phy_device *phy_dev, 
					unsigned int regadr, unsigned int phyadr)
{
	_cpsw_halcommon_mii_mdio_wait_for_access_complete(phy_dev); 
	_cpsw_halcommon_mii_mdio_useraccess(phy_dev, MDIO_USERACCESS_READ, regadr, phyadr, 0);
	_cpsw_halcommon_mii_mdio_wait_for_access_complete(phy_dev); 
	
	return(TI_MDIOUSERACCESS & MDIO_USERACCESS_DATA);
}
	
void _cpsw_halcommon_mii_mdiouser_accesswrite(phy_device *phy_dev, unsigned int regadr, unsigned int phyadr, unsigned int data)
{
	_cpsw_halcommon_mii_mdio_wait_for_access_complete(phy_dev);
	_cpsw_halcommon_mii_mdio_useraccess(phy_dev, MDIO_USERACCESS_WRITE, regadr, phyadr, data);
}
	
void _cpsw_halcommon_mii_mdio_wait_for_access_complete(phy_device *phy_dev)
{
	while((TI_MDIOUSERACCESS & MDIO_USERACCESS_GO)!=0)
	{
		/* Do Nothing */
	}
}
	
void _cpsw_halcommon_mii_mdio_useraccess(phy_device *phy_dev, unsigned int method, 
				unsigned int regadr, unsigned int phyadr, unsigned int data)
{
	unsigned int control;
	
	control =  MDIO_USERACCESS_GO |
		(method) |
		(((regadr) << 21) & MDIO_USERACCESS_REGADR) |
		(((phyadr) << 16) & MDIO_USERACCESS_PHYADR) |
		((data) & MDIO_USERACCESS_DATA);
	
	TI_MDIOUSERACCESS = control;
}
