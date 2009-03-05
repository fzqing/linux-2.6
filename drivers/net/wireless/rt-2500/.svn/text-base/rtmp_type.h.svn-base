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
 *      Module Name: rtmp_type.h
 *              
 *      Abstract: 
 *              
 *      Revision History: 
 *      Who             When            What 
 *      --------        -----------     ----------------------------- 
 *      PaulL           2md  Jan 03     Initial code     
 *      MarkW           8th  Dec 04     Baseline code  
 ***************************************************************************/ 

#ifndef	__RTMP_TYPE_H__
#define	__RTMP_TYPE_H__

typedef	unsigned short		UINT16;
typedef	unsigned long		UINT32;
typedef unsigned long long	UINT64;

#define PACKED  __attribute__ ((packed))

// Endian byte swapping codes
#ifdef	BIG_ENDIAN

#define SWAP16(x) \
	((UINT16)( \
	(((UINT16)(x) & (UINT16) 0x00ffU) << 8) | \
	(((UINT16)(x) & (UINT16) 0xff00U) >> 8) ))

#define SWAP32(x) \
	((UINT32)( \
	(((UINT32)(x) & (UINT32) 0x000000ffUL) << 24) | \
	(((UINT32)(x) & (UINT32) 0x0000ff00UL) <<  8) | \
	(((UINT32)(x) & (UINT32) 0x00ff0000UL) >>  8) | \
	(((UINT32)(x) & (UINT32) 0xff000000UL) >> 24) ))

#define SWAP64(x) \
	((UINT64)( \
	(UINT64)(((UINT64)(x) & (UINT64) 0x00000000000000ffULL) << 56) | \
	(UINT64)(((UINT64)(x) & (UINT64) 0x000000000000ff00ULL) << 40) | \
	(UINT64)(((UINT64)(x) & (UINT64) 0x0000000000ff0000ULL) << 24) | \
	(UINT64)(((UINT64)(x) & (UINT64) 0x00000000ff000000ULL) <<  8) | \
	(UINT64)(((UINT64)(x) & (UINT64) 0x000000ff00000000ULL) >>  8) | \
	(UINT64)(((UINT64)(x) & (UINT64) 0x0000ff0000000000ULL) >> 24) | \
	(UINT64)(((UINT64)(x) & (UINT64) 0x00ff000000000000ULL) >> 40) | \
	(UINT64)(((UINT64)(x) & (UINT64) 0xff00000000000000ULL) >> 56) ))
#else

#define SWAP16(x)
#define SWAP32(x)
#define	SWAP64(x)
#endif

#ifdef BIG_ENDIAN

#define cpu2le64(x) SWAP64((x))
#define le2cpu64(x) SWAP64((x))
#define cpu2le32(x) SWAP32((x))
#define le2cpu32(x) SWAP32((x))
#define cpu2le16(x) SWAP16((x))
#define le2cpu16(x) SWAP16((x))
#define cpu2be64(x) ((UINT64)(x))
#define be2cpu64(x) ((UINT64)(x))
#define cpu2be32(x) ((UINT32)(x))
#define be2cpu32(x) ((UINT32)(x))
#define cpu2be16(x) ((UINT16)(x))
#define be2cpu16(x) ((UINT16)(x))

#else	// Little_Endian

#define cpu2le64(x) ((UINT64)(x))
#define le2cpu64(x) ((UINT64)(x))
#define cpu2le32(x) ((UINT32)(x))
#define le2cpu32(x) ((UINT32)(x))
#define cpu2le16(x) ((UINT16)(x))
#define le2cpu16(x) ((UINT16)(x))
#define cpu2be64(x) SWAP64((x))
#define be2cpu64(x) SWAP64((x))
#define cpu2be32(x) SWAP32((x))
#define be2cpu32(x) SWAP32((x))
#define cpu2be16(x) SWAP16((x))
#define be2cpu16(x) SWAP16((x))

#endif	// BIG_ENDIAN

#endif	// __RTMP_TYPE_H__

