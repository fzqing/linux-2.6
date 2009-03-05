#ifndef _RELAY_RESIZE_H
#define _RELAY_RESIZE_H

/*
 * RelayFS buffer management and resizing code.
 *
 * Copyright (C) 2002, 2003 - Tom Zanussi (zanussi@us.ibm.com), IBM Corp
 * Copyright (C) 1999, 2000, 2001, 2002 - Karim Yaghmour (karim@opersys.com)
 * Copyright (C) 2004, 2005 - MontaVista Software, Inc. (source@mvista.com)
 *
 * This file is released  under the terms of the GNU GPL version 2.
 * This program  is licensed "as is" without any warranty of any kind,
 * whether express or implied.
 */

#include "rtasklet.h"

/* 
 * If the channel usage has been below the low water mark for more than
 * this amount of time, we can shrink the buffer if necessary.
 */
#define SHRINK_TIMER_SECS	60

/* This inspired by rtai/shmem */
#define FIX_SIZE(x) (((x) - 1) & PAGE_MASK) + PAGE_SIZE

/* Don't attempt resizing again after this many failures */
#define MAX_RESIZE_FAILURES	1

/* Trigger resizing if a resizable channel is this full */
#define RESIZE_THRESHOLD	3 / 4

/*
 * Used for deferring resized channel free
 */
struct free_rchan_buf
{
	char *unmap_buf;
	struct 
	{
		struct page **array;
		int count;
	} page_array[3];
	
	int cur;
	struct work_struct work; 	/* resize de-allocation work struct */
	struct tasklet_queue_head work_tasklet;	/* resize de-allocation work struct */
};

extern void *
alloc_rchan_buf(unsigned long size,
		struct page ***page_array,
		int *page_count);

extern int
free_rchan_buf(void *buf,
	       struct page **page_array,
	       int page_count);

extern void
expand_check(struct rchan *rchan);

extern void
init_shrink_timer(struct rchan *rchan);

#endif/* _RELAY_RESIZE_H */
