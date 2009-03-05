/* ------------------------------------------------------------------------- */
/* i2c-algo-emma2rh.h i2c driver algorithms for NEC EMMA2RH I2C adapters     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 1995-97 Simon G. Vogl
                   1998-99 Hans Berglund

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                */
/* ------------------------------------------------------------------------- */

/* With some changes from Kyösti Mälkki <kmalkki@cc.hut.fi> and even
   Frodo Looijaard <frodol@dds.nl> */

/* Modifications by NEC Electronics Corporation, 2003-2006
   Changes made to support the I2C peripheral on the NEC EMMA2RH */

#ifndef I2C_ALGO_EMMA2RH_H
#define I2C_ALGO_EMMA2RH_H

/* --- Defines for pcf-adapters ---------------------------------------	*/
#include <linux/i2c.h>

struct i2c_algo_emma_data {
	void *data;		/* private data for lolevel routines    */
	void (*setemma) (void *data, int ctl, int val);
	int (*getemma) (void *data, int ctl);
	int (*getown) (void *data);
	int (*getclock) (void *data);
	void (*waitforpin) (void *data);

	/* local settings */
	int udelay;
	int mdelay;
	int timeout;
};

int i2c_emma_add_bus(struct i2c_adapter *);
int i2c_emma_del_bus(struct i2c_adapter *);

#endif				/* I2C_ALGO_EMMA2RH_H */
