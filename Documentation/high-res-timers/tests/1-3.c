/*   
 * Copyright (c) 2002, Intel Corporation. All rights reserved.
 * Created by:  julie.n.fleischer REMOVE-THIS AT intel DOT com
 * This file is licensed under the GPL license.  For the full content
 * of this license, see the COPYING file at the top level of this 
 * source tree.

 * Test that timer_gettime() sets sets itimerspec.it_value to the
 * amount of time remaining in the middle of a timer.
 * - Create and arm a timer for TIMERNSEC nanoseconds.
 * - Sleep for SLEEPNSEC < TIMERNSEC.
 * - Call timer_gettime().
 * - Ensure the value returned is within ACCEPTABLEDELTA less than
 *   TIMERNSEC-SLEEPNSEC.
 *
 * Signal SIGCONT will be used so that it will not affect the test if
 * the timer expires.
 * Clock CLOCK_REALTIME will be used.
 */

#include <time.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include "utils.h"

#define TIMERNSEC 80000000
#define SLEEPNSEC 40000000
#define ACCEPTABLEDELTA 3000000

int main(int argc, char *argv[])
{
	struct sigevent ev;
	timer_t tid;
	struct itimerspec itsset, itsget_start, itsget;
	struct timespec ts, start_sleep, stop_sleep, zero = {0,0};
	int deltans;

	init_test(argc, argv);

	ev.sigev_notify = SIGEV_SIGNAL;
	ev.sigev_signo = SIGCONT;

	itsset.it_interval.tv_sec = 0;
	itsset.it_interval.tv_nsec = 0;
	itsset.it_value.tv_sec = 0;
	itsset.it_value.tv_nsec = TIMERNSEC;

	Try (timer_create(CLOCK_REALTIME, &ev, &tid)); 

	Try (timer_settime(tid, 0, &itsset, NULL));

	Try (timer_gettime(tid, &itsget_start));

	ts.tv_sec=0;
	ts.tv_nsec=SLEEPNSEC;

	Try (clock_gettime(CLOCK_REALTIME, &start_sleep));

#if 0
	Try (clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL));
#else
	Try (nanosleep(&ts, NULL));
#endif
	Try (clock_gettime(CLOCK_REALTIME, &stop_sleep));

	Try (timer_gettime(tid, &itsget));

	fprintft(stderr, "Request sleep time was %12.9f sec\n", 
		 timerdiff(&ts, &zero));

	fprintft(stderr, "Actual  sleep time was %12.9f sec\n", 
		 timerdiff(&stop_sleep, &start_sleep));

	fprintft(stderr, "Actual  timer time was %12.9f sec\n", 
		 timerdiff(&itsget_start.it_value, &zero));

	fprintft(stderr, "Left    timer time was %12.9f sec\n", 
		 timerdiff(&itsget.it_value, &zero));
        /*
	 * Algorithm for determining success:
	 * - itsget must be < itsset
	 * - itsset-itsget nsec must be <= ACCEPTABLEDELTA
	 */

	if (itsget.it_value.tv_sec > 0) {
		printf("FAIL:  timer_gettime() value > time expected left\n");
		printf("%d seconds > 0 seconds\n", 
				(int) itsget.it_value.tv_sec);
		return PTS_FAIL;
	}

	deltans=(itsset.it_value.tv_nsec - ts.tv_nsec)- itsget.it_value.tv_nsec;

	if (deltans < 0) {
		printf("FAIL:  timer_gettime() value > time expected left\n");
		printf("%d > %d\n", (int) itsget.it_value.tv_nsec, 
				(int) itsset.it_value.tv_nsec - 
					(int) ts.tv_nsec);
		return PTS_FAIL;
	}

	if (deltans <= ACCEPTABLEDELTA) {
		printf("Test PASSED\n");
		return PTS_PASS;
	} else {
		printf("FAIL:  timer_gettime() value !~= time expected left\n");
		printf("%d !~= %d\n", (int) itsget.it_value.tv_nsec, 
				(int) itsset.it_value.tv_nsec - 
					(int) ts.tv_nsec);
		return PTS_FAIL;
	}

	printf("This code should not be executed\n");
	return PTS_UNRESOLVED;
}
