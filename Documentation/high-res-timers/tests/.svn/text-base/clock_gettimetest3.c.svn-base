/*
 * Copyright (C) 1997 by the University of Kansas Center for Research,
 * Inc.  This software was developed by the Information and
 * Telecommunication Technology Center (ITTC) at the University of
 * Kansas.  Partial funding for this project was provided by Sprint. This
 * software may be used and distributed according to the terms of the GNU
 * Public License, incorporated herein by reference.  Neither ITTC nor
 * Sprint accept any liability whatsoever for this product.
 *
 * This project was developed under the direction of Dr. Douglas Niehaus.
 *
 * Authors: Shyam Pather, Balaji Srinivasan 
 *
 * Please send bug-reports/suggestions/comments to posix@ittc.ukans.edu
 *
 * Further details about this project can be obtained at
 *    http://hegel.ittc.ukans.edu/projects/posix/
 */

/* clock_gettimetest3.c
 *
 * This program simply calls gettimeofday(), followed by clock_gettime() 
 * in a loop. The sequence of time values printed should be monotonically
 * increasing. The time values returned by clock_gettimeofday() have greater
 * accuracy than those returned by gettimeofday().
 */

#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#ifdef __linux__
#include <posix_time.h>
#endif
#include "utils.h"

#define MAX_SAMPLES 25

int main(int argc, char * argv[]) 
{
	struct timespec ts, start, end, end2;
	struct timeval tv;
	int i = 0, retval;
	int delta;

	init_test(argc, argv);
	fprintft(stderr, "#sample\tdelta (usec)\n");
	for (i = 0; i < MAX_SAMPLES; i++) {
		Try( gettimeofday(&tv, NULL));
		
		clock_gettime(CLOCK_MONOTONIC, &start);
		Try(clock_gettime(CLOCK_REALTIME, &ts));
		clock_gettime(CLOCK_MONOTONIC, &end);
		clock_gettime(CLOCK_MONOTONIC, &end2);

		delta = (ts.tv_sec - tv.tv_sec) * 1000000 +
		  (ts.tv_nsec / 1000 - tv.tv_usec);

		fprintft(stderr, "%d\t%d ", i, delta);
		fprintft(stderr, "start %ld.%09ld, end %ld.%09ld, "
			 "end2 %ld.%09ld delta %ld.%09ld delta2 %ld.%09ld\n",
		       start.tv_sec, start.tv_nsec, end.tv_sec, end.tv_nsec,
		       end2.tv_sec, end2.tv_nsec,
		       end.tv_sec - start.tv_sec, end.tv_nsec - start.tv_nsec,
		       end2.tv_sec - end.tv_sec, end2.tv_nsec - end.tv_nsec);
	}
	by_now();
}
