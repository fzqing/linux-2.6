/*
 *   Copyright (C) 2002 MontaVista Software  
 *                      George Anzinger (george@mvista.com)
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#ifdef __linux__
#include <posix_time.h>
#include "utils.h"
#endif

#define MAX_SAMPLES 25

int timespec_diff(struct timespec *start, struct timespec *end)
{
  return (end->tv_sec - start->tv_sec) * 1000000000 +
    (end->tv_nsec - start->tv_nsec);
}


int main(int argc, char * argv[])
{
	struct timespec start_rt, end_rt;
	struct timespec start_tsc, end_tsc;
	int i;
	int delta_rt, delta_tsc;

	init_test(argc, argv);
	fprintft(stderr, "Measured times to get times (in nano secs):\n");
	for (i = 0; i < MAX_SAMPLES; i++) {
		clock_gettime(CLOCK_REALTIME, &start_rt);
		clock_gettime(CLOCK_REALTIME, &end_rt);
		clock_gettime(CLOCK_MONOTONIC, &start_tsc);
		clock_gettime(CLOCK_MONOTONIC, &end_tsc);
		delta_rt = timespec_diff(&start_rt, &end_rt);
		delta_tsc = timespec_diff(&start_tsc, &end_tsc);

		fprintft(stderr, "CLOCK_REALTIME = %9d\tCLOCK_MONOTONIC = %9d\n",
		       delta_rt, delta_tsc);
	}
	by_now();
}
