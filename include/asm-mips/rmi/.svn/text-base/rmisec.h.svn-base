/*
 * Copyright (C) 2003 Raza Foundries
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */


#define RMI_SECENG_DEV_ID     4444
#define RMI_SECENG_IRQ        0

#define RMI_MAX_DESCRIPTORS   20


typedef struct device_info_s {
	void               *context;
	void               *completion_handler;
	struct semaphore   dev_sem;
	volatile uint32_t  dev_status;
	uint32_t           dev_features;
} dev_info_t, *dev_info_pt;




