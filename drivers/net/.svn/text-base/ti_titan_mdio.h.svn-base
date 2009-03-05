#ifndef _CPSWHALCOMMON_MIIMDIO_H
#define _CPSWHALCOMMON_MIIMDIO_H

typedef const char HAL_CONTROL_KEY;

void cpsw_halcommon_mii_mdio_getver(unsigned int mii_base,unsigned int *mod_id,
					unsigned int *revmaj,  unsigned int *revmin);

int cpsw_halcommon_mii_mdio_getphydevsize(void); 

int cpsw_halcommon_mii_mdio_init(phy_device *phy_dev,unsigned int miibase,
					unsigned int inst,unsigned int phy_mask,
					unsigned int mlink_mask,unsigned int reset_base,
					unsigned int reset_bit,unsigned int mdio_bus_freq, 
					unsigned int mdio_clock_freq);

unsigned int _cpsw_halcommon_mii_mdio_user_accessread(phy_device *phy_dev, 
						unsigned int regadr, unsigned int phyadr);

void  _cpsw_halcommon_mii_mdiouser_accesswrite(phy_device *phy_dev, 
					unsigned int regadr, unsigned int phyadr, unsigned int data);

#define NWAY_AUTOMDIX       (1<<16)
#define NWAY_FD1000         (1<<13)
#define NWAY_HD1000         (1<<12)
#define NWAY_NOPHY          (1<<10)
#define NWAY_LPBK           (1<<9)
#define NWAY_FD100          (1<<8)
#define NWAY_HD100          (1<<7)
#define NWAY_FD10           (1<<6)
#define NWAY_HD10           (1<<5)
#define NWAY_AUTO           (1<<0)

#define _MIIMDIO_MDIXFLIP (1<<28)
#define _AUTOMDIX_DELAY_MIN  80  /* milli-seconds*/
#define _AUTOMDIX_DELAY_MAX 200  /* milli-seconds*/

#define PMDIO_VER(base)			((volatile unsigned int *)(base+0x00))
#define PMDIO_CONTROL(base)		((volatile unsigned int *)(base+0x04))
#define PMDIO_ALIVE(base)		((volatile unsigned int *)(base+0x08))
#define PMDIO_LINK(base)		((volatile unsigned int *)(base+0x0C))
#define PMDIO_LINKINTRAW(base)		((volatile unsigned int *)(base+0x10))
#define PMDIO_LINKINTMASKED(base)       ((volatile unsigned int *)(base+0x14))
#define PMDIO_USERINTRAW(base)		((volatile unsigned int *)(base+0x20))
#define PMDIO_USERINTMASKED(base)	((volatile unsigned int *)(base+0x24))
#define PMDIO_USERINTMASKED_SET(base)	((volatile unsigned int *)(base+0x28))
#define PMDIO_USERINTMASKED_CLR(base)	((volatile unsigned int *)(base+0x2C))
#define PMDIO_USERACCESS(base, channel)	((volatile unsigned int *)(base+(0x80+(channel*8))))
#define PMDIO_USERPHYSEL(base, channel)	((volatile unsigned int *)(base+(0x84+(channel*8))))

#define MDIO_ALIVE(base)			(*(PMDIO_ALIVE(base)))
#define MDIO_CONTROL(base)			(*(PMDIO_CONTROL(base)))
#define MDIO_CONTROL_IDLE			(1 << 31)
#define MDIO_CONTROL_ENABLE			(1 << 30)
#define MDIO_CONTROL_PREAMBLE			(1 << 20)  
#define MDIO_CONTROL_FAULT			(1 << 19)
#define MDIO_CONTROL_FAULT_DETECT_ENABLE	(1 << 18)
#define MDIO_CONTROL_INT_TEST_ENABLE		(1 << 17)
#define MDIO_CONTROL_HIGHEST_USER_CHANNEL	(0x1F << 8)
#define MDIO_CONTROL_CLKDIV			(0xFF)


#define MDIO_LINK(base)			(*(PMDIO_LINK(base)))
#define MDIO_LINKINTRAW(base)		(*(PMDIO_LINKINTRAW(base)))
#define MDIO_LINKINTMASKED(base)	(*(PMDIO_LINKINTMASKED(base)))
#define MDIO_USERINTRAW(base)		(*(PMDIO_USERINTRAW(base)))
#define MDIO_USERINTMASKED(base)	(*(PMDIO_USERINTMASKED(base)))
#define MDIO_USERINTMASKED_CLR(base)	(*(PMDIO_USERINTMASKED_CLR(base)))
#define MDIO_USERINTMASKED_SET(base)	(*(PMDIO_USERINTMASKED_SET(base)))
#define MDIO_USERINTRAW(base)		(*(PMDIO_USERINTRAW(base)))
#define MDIO_USERACCESS(base, channel)	(*(PMDIO_USERACCESS(base, channel)))
#define MDIO_USERACCESS_GO		(1 << 31)
#define MDIO_USERACCESS_WRITE		(1 << 30)
#define MDIO_USERACCESS_READ		(0 << 30)
#define MDIO_USERACCESS_ACK		(1 << 29)
#define MDIO_USERACCESS_REGADR		(0x1F << 21)
#define MDIO_USERACCESS_PHYADR		(0x1F << 16)
#define MDIO_USERACCESS_DATA		(0xFFFF)
#define MDIO_USERPHYSEL(base, channel)	(*(PMDIO_USERPHYSEL(base, channel)))
#define MDIO_USERPHYSEL_LINKSEL		(1 << 7)
#define MDIO_USERPHYSEL_LINKINT_ENABLE	(1 << 6)
#define MDIO_USERPHYSEL_PHYADR_MON 	(0x1F)

#define MDIO_VER(base)		(*(PMDIO_VER(base)))
#define MDIO_VER_MODID		(0xFFFF << 16)
#define MDIO_VER_REVMAJ		(0xFF   << 8)
#define MDIO_VER_REVMIN		(0xFF)
#define PHY_CONTROL_REG		0
#define MII_PHY_RESET		(1<<15)
#define MII_PHY_LOOP		(1<<14)
#define MII_PHY_100		(1<<13)
#define MII_AUTO_NEGOTIATE_EN	(1<<12)
#define MII_PHY_PDOWN		(1<<11)
#define MII_PHY_ISOLATE		(1<<10)
#define MII_RENEGOTIATE		(1<<9)
#define MII_PHY_FD		(1<<8)
#define PHY_STATUS_REG		1
#define MII_NWAY_COMPLETE	(1<<5)
#define MII_NWAY_CAPABLE	(1<<3)
#define MII_PHY_LINKED		(1<<2)
#define NWAY_ADVERTIZE_REG	4
#define NWAY_REMADVERTISE_REG	5
#define MII_NWAY_FD100		(1<<8)
#define MII_NWAY_HD100		(1<<7)
#define MII_NWAY_FD10		(1<<6)
#define MII_NWAY_HD10		(1<<5)
#define MII_NWAY_SEL		(1<<0)

#endif
