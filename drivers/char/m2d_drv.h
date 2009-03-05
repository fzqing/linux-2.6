/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _M2D_H
#define _M2D_H

#define MAX_DEVICE_GMEM_SIZE		(8*1024*1024)
#define MAX_CONTEXT_GMEM_SIZE		(4*1024*1024)

#define GCU_RINGBUF_SIZE		(16384)
#define GCU_SCRATCHREG_NR		(8)

#include <asm/ioctl.h>

#define M2DIO_SUBMIT		_IOW('2',  1, struct iovec *)
#define M2DIO_SYNC		_IOW('2',  2, void)

#define M2DIO_REQUEST_MEM	_IOW('2', 10, struct m2d_mem_request *)
#define M2DIO_RELEASE_MEM	_IOW('2', 11, unsigned long)
#define M2DIO_FLUSH_MEM		_IOW('2', 12, unsigned long)

#define M2DIO_GET_BUS_ADDR	_IOW('2', 20, unsigned long)

//#define M2DIO_QUERY_GCU               _IOR('2', 20, struct m2d_gcu_stat *)

#define M2D_GRAPHICS_MEM	0
#define M2D_FRAME_BUFFER	1
#define M2D_REGISTERS		2
#define M2D_RING_BUFFER		3

#define M2D_ATTR_COHERENT	0x00
#define M2D_ATTR_WRITECOMBINE	0x10
#define M2D_ATTR_CACHEABLE	0x20

#define M2D_MEM_REQ_TYPE(f)		(f & 0x0f)
#define M2D_MEM_REQ_ATTR(f)		(f & 0xf0)

struct m2d_mem_req {
	unsigned int req_type;
	unsigned int req_size;
	unsigned long phys_addr;
	unsigned long mmap_addr;
	unsigned long mmap_size;
};

#define M2D_SUBMIT_MODE_NDELAY	(1 << 0)
#define M2D_SUBMIT_MODE_SYNC	(1 << 1)

struct m2d_submit_req {
	unsigned int mode;
	void *base;
	size_t len;
};

#endif
