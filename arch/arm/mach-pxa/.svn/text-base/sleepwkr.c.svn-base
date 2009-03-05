/*
 * arch/arm/mach-pxa/sleepwkr.c
 *
 * Copyright (C) 2006, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */
#include <linux/string.h>
#include "sleepwkr.h"

static char start_str[] = "0000110000000000000000000000001";
static char end_str[] = "0000000000000000000000000000000";

static unsigned long mpu_reg;

void sleep_wbit(unsigned int tdi, unsigned int tms)
{
	volatile unsigned long *pmpu_reg = (unsigned long *)mpu_reg;

	*pmpu_reg = SIG_ENABLE | tdi | tms;
	*pmpu_reg = SIG_ENABLE | tdi | tms | SIG_TCK;
}
int sleep_wreg(unsigned int portal, char *bit_field)
{
	unsigned int tms;
	unsigned int i,j;

	sleep_wbit(0,0);

	sleep_wbit(0,1);

	sleep_wbit(0,1);

	sleep_wbit(0,0);

	sleep_wbit(0,0);

	for (i=0;i<11;i++)
	{
		if (i < 10)
			sleep_wbit((portal >> i) & 1, 0);
		else
			sleep_wbit((portal >> i) & 1, 1);
	}

	/*
	 * On the the last data shift, the loop sets TMS to 1
	 * causing the TAP controller to move to the "Exit1-IR" state
	 */

	/* move the TAP controller to the "Update-IR" state
	 */
	sleep_wbit(0,1);

	if (bit_field == NULL)
	{
   		/* move the TAP controller to the "Run-Test/Idle" state
		 */
	   	sleep_wbit(0,0);
		return(0);
	}

	/* move the TAP controller to the "Select-DR-Scan" state
	 */
	sleep_wbit(0,1);

	/* move the TAP controller to the "Capture-DR" state
	 */
	sleep_wbit(0,0);

	/* move the TAP controller to the "Shift-DR" state
	 */
	sleep_wbit(0,0);

	/* clock in the data
	 * remove possible bad character from end that would mess up tms check...
	 */
	i = strlen(bit_field);
	while ( i && (bit_field[i]=='_' || bit_field[i]==' ') )
	{
		bit_field[i] = 0;
		i--;
	}

	for (i=0;i<1000000;i++)
	{
		/* skip human readability elements... */
		if (bit_field[i] == '_' || bit_field[i] == ' ')
			continue;
		if (bit_field[i+1] == 0)
			tms = 1;
		else
			tms = 0;
		if (bit_field[i] == '1')
			sleep_wbit(1,tms);
		else if (bit_field[i] == '0')
			sleep_wbit(0,tms);
		else if (bit_field[i] == 'F')
		{
			for(j=0; j<4; j++)
				sleep_wbit(1,tms);
		}else if (bit_field[i] == 0)
			break;
		else
			return (-1);
	}

	if (i == 1000000)
		return (-1);

	/* On the the last data shift, the loop sets TMS to 1
	 * causing the TAP controller to move to the "Exit1-DR" state
	 */

	/* move the TAP controller to the "Update DR" state
	 */
	sleep_wbit(0,1);

	/* move the TAP controller to the "Run-Test/Idle" state
	 */
	sleep_wbit(0,0);
	return (0);
}

int sleep_wkr_start(unsigned long reg)
{
	mpu_reg = reg;
	sleep_wreg(0x0b, start_str);
	return 0;
}

int sleep_wkr_end(unsigned long reg)
{
	mpu_reg = reg;
	sleep_wreg(0x0b, end_str);
	return 0;
}

