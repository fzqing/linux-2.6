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

/* clock_gettimetest2.c
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

#define N 25

int main(int argc, char * argv[]) {
	struct timespec ts[N],tvs, tc;
	struct timeval tv[N],tvt;
	int i = 0, retval;

	init_test(argc, argv);
	for (;i < N;i++) {
		retval = gettimeofday(&tv[i], NULL);
		if (retval) {
			myperror("gettimeofday() failed");
			exit(1);
		}
		
		retval = clock_gettime(CLOCK_REALTIME, &ts[i]);
		if (retval) {
			myperror("clock_gettime() failed");
			exit(1);
		}
        }
	for (i=0;i < N;i++) {
		fprintft(stderr, 
			 "(%d) tv_sec = %ld, tv_usec*1000 = %ld000\n",
			 i, tv[i].tv_sec, tv[i].tv_usec);
		tc.tv_sec = tv[i].tv_sec;
		tc.tv_nsec = tv[i].tv_usec * 1000;
		fprintft(stderr, 
			 "(%d) tv_sec = %ld, tv_nsec      = %ld diff %.9f\n",i, 
			 ts[i].tv_sec, ts[i].tv_nsec, timerdiff(&ts[i], &tc));	
                timeval_to_timespec(&tv[i],&tvs);
                assert( ! timer_gt(&tvs,&ts[i]));
                if ( i < (N -1)){
                        timespec_to_timeval(&ts[i],&tvt);
                        assert( timevaldiff(&tv[i+1],&tvt) >= 0);
                }
                        
	}
	by_now();
}
