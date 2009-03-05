/*************************************************************************** 
 * RT2400/RT2500 SourceForge Project - http://rt2x00.serialmonkey.com      * 
 *                                                                         * 
 *   This program is free software; you can redistribute it and/or modify  * 
 *   it under the terms of the GNU General Public License as published by  * 
 *   the Free Software Foundation; either version 2 of the License, or     * 
 *   (at your option) any later version.                                   * 
 *                                                                         * 
 *   This program is distributed in the hope that it will be useful,       * 
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        * 
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         * 
 *   GNU General Public License for more details.                          * 
 *                                                                         * 
 *   You should have received a copy of the GNU General Public License     * 
 *   along with this program; if not, write to the                         * 
 *   Free Software Foundation, Inc.,                                       * 
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             * 
 *                                                                         * 
 *   Licensed under the GNU GPL                                            * 
 *   Original code supplied under license from RaLink Inc, 2004.           * 
 ***************************************************************************/ 

 /*************************************************************************** 
 *      Module Name: rt_config.h
 *              
 *      Abstract: Central header file for all includes
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      RoryC           21st Dec 02     Initial code   
 *      MarkW           8th  Dec 04     Baseline code  
 ***************************************************************************/ 

#ifndef __RT_CONFIG_H__
#define __RT_CONFIG_H__

#define PROFILE_PATH                "/etc/Wireless/RT2500STA/RT2500STA.dat"
#define NIC_DEVICE_NAME             "RT2500STA"

#define	DRV_NAME	"rt2500"
#define DRV_VERSION	"1.1.0 BETA3"
#define DRV_RELDATE	"2005/07/31"
#define DRV_VERSION_MAJOR 1
#define DRV_VERSION_MINOR 1 
#define DRV_VERSION_SUB 0
#define DRV_BUILD_YEAR 2005
#define DRV_BUILD_MONTH 07
#define DRV_BUILD_DAY 31 

/* Operational parameters that are set at compile time. */
#if !defined(__OPTIMIZE__)  ||  !defined(__KERNEL__)
#warning  You must compile this file with the correct options!
#warning  See the last lines of the source file.
#error  You must compile this driver with "-O".
#endif

#include <linux/config.h>  //can delete
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/errno.h> //can delete
#include <linux/ioport.h> // can delete
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/init.h>  //can delete
#include <linux/delay.h> // can delete
#include <linux/ethtool.h>
#include <linux/wireless.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/if_arp.h>
#include <linux/ctype.h>

#if LINUX_VERSION_CODE >= 0x20407
#include <linux/mii.h>
#endif
#include <asm/processor.h>      /* Processor type for cache alignment. */
#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/div64.h>

// The type definition has to be placed before including rt2460.h
#ifndef ULONG
#define CHAR            char
#define INT             int
#define SHORT           int
#define UINT            u32
#define ULONG           u32
#define USHORT          u16
#define UCHAR           u8

#define BOOLEAN         u8
//#define LARGE_INTEGER s64
#define VOID            void
#define LONG            int
#define ULONGLONG       u64
typedef VOID            *PVOID;
typedef CHAR            *PCHAR;
typedef UCHAR           *PUCHAR;
typedef LONG            *PLONG;
typedef ULONG           *PULONG;

typedef union _LARGE_INTEGER {
    struct {
        ULONG LowPart;
        LONG HighPart;
    }vv;
    struct {
        ULONG LowPart;
        LONG HighPart;
    } u;
    s64 QuadPart;
} LARGE_INTEGER;

#endif

#define IN
#define OUT

#define TRUE        1
#define FALSE       0

#define NDIS_STATUS                             INT
#define NDIS_STATUS_SUCCESS                     0x00
#define NDIS_STATUS_FAILURE                     0x01
#define NDIS_STATUS_RESOURCES                   0x03
#define NDIS_STATUS_MEDIA_DISCONNECT            0x04
#define NDIS_STATUS_MEDIA_CONNECT               0x05

#ifdef __BIG_ENDIAN
#if 0
/* Removed this from MVL kernel ... need to verify big endian operation */
#warning Compiling for big endian machine.
#endif
#define BIG_ENDIAN TRUE
#endif /* __BIG_ENDIAN */

#include    "rtmp_type.h"
#include    "rtmp_def.h"
#include    "rt2560.h"
#include    "rtmp.h"
#include    "mlme.h"
#include    "oid.h"
#include    "wpa.h"
#include    "md5.h"

#define DEBUG_TASK_DELAY        2000

enum rt2560_chips {
    RT2560A = 0,
};

#ifdef RTMP_EMBEDDED
#undef GFP_KERNEL
#define GFP_KERNEL      (GFP_DMA | GFP_ATOMIC)
#endif

#endif  // __RT_CONFIG_H__
