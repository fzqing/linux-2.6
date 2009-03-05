/*   
 * Copyright (c) 2002, Intel Corporation. All rights reserved.
 * Created by:  julie.n.fleischer REMOVE-THIS AT intel DOT com
 * This file is licensed under the GPL license.  For the full content
 * of this license, see the COPYING file at the top level of this 
 * source tree.
 *
 * Test that timers are not allowed to expire before their scheduled
 * time.
 *
 * Test for a variety of timer values on relative timers.
 *
 * For this test, signal SIGTOTEST will be used, clock CLOCK_REALTIME
 * will be used.
 */
/*
 * Changed to fit/use the HRT test harness...
 */

#include <time.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include "utils.h"

#define PTS_PASS        0
#define PTS_FAIL        1
#define PTS_UNRESOLVED  2
#define PTS_UNSUPPORTED 4
#define PTS_UNTESTED    5

#define SIGTOTEST SIGALRM
#define TIMERVALUESEC 2
#define TIMERINTERVALSEC 5
#define INCREMENT 1
#define ACCEPTABLEDELTA 1

#define NUMTESTS 6

static int init_timeroffsets[NUMTESTS][2] = { {0, 30000000}, {1, 0}, 
					{1, 30000000}, {2, 0},
					{1, 5000}, {1, 5} };
static int timeroffsets[NUMTESTS][2];

void do_test(clock_t clock)
{
	struct sigevent ev;
	timer_t tid;
	struct itimerspec its;
	struct timespec tsbefore, tsafter, res, diff, zero = {0,0};
	sigset_t set;
	int sig;
	int i, ires;
 	double totalnsecs, testnsecs; 
	/*
	 * set up signal set containing SIGTOTEST that will be used
	 * in call to sigwait immediately after timer is set
	 */

	Try (sigemptyset(&set));

	Try (sigaddset(&set, SIGTOTEST));

        Try (sigprocmask (SIG_BLOCK, &set, NULL));

	/*
	 * set up timer to perform action SIGTOTEST on expiration
	 */
	ev.sigev_notify = SIGEV_SIGNAL;
	ev.sigev_signo = SIGTOTEST;

	/*
	 * Convert the initial offsets into multiples of resolution.
	 * (This gives the worse possible value to test with.)
	 */
	Try (clock_getres(clock, &res));
	ires = res.tv_nsec;

	for (i = 0; i < NUMTESTS; i++) {
		long long time = (init_timeroffsets[i][0] * NSEC_PER_SEC) +
			init_timeroffsets[i][1] + ires;
		time /= ires;
		time *= ires;

		timeroffsets[i][0] = time / NSEC_PER_SEC;
		timeroffsets[i][1] = time % NSEC_PER_SEC;
	}
	Try (timer_create(clock, &ev, &tid));

	for (i = 0; i < NUMTESTS; i++) {
		its.it_interval.tv_sec = 0; its.it_interval.tv_nsec = 0;
		its.it_value.tv_sec = timeroffsets[i][0];
		its.it_value.tv_nsec = timeroffsets[i][1];

		fprintft(stderr,"Test for value %d sec %d nsec\n", 
				(int) its.it_value.tv_sec,
				(int) its.it_value.tv_nsec);

		Try (clock_gettime(clock, &tsbefore));
		
		Try (timer_settime(tid, 0, &its, NULL));
	
		Try (sigwait(&set, &sig));
	
		Try (clock_gettime(clock, &tsafter));
	        if (timer_gt(&tsbefore, &tsafter)) 
			myerror("Time appears to be moving backward!!!\n");
		timersubtract(&diff, &tsafter, &tsbefore);
		totalnsecs = timerdiff(&diff, &zero);
		testnsecs = timerdiff(&its.it_value, &zero);
		if (totalnsecs < testnsecs) {
			myerror("FAIL:  Expired too soon\n");

			fprintf(stderr, SETCOLOR_FAILURE 
				"tsbefore: sec--%lu, nsec--%lu\n", 
				tsbefore.tv_sec,
				tsbefore.tv_nsec);
			fprintf(stderr, SETCOLOR_FAILURE 
				"tsafter:  sec--%lu, nsec--%lu\n", 
				tsafter.tv_sec,
				tsafter.tv_nsec);
			fprintf(stderr, SETCOLOR_FAILURE 
				"early by %11.9f secs\n"
				SETCOLOR_NORMAL, 
				testnsecs - totalnsecs);
		} else {
			fprintft(stderr, "Late by: %11.9f secs \n",
				 totalnsecs - testnsecs);
		}	
	}

	Try (timer_delete(tid));

}
int main(int argc, char *argv[])
{
	init_test(argc, argv);
	do_test(CLOCK_REALTIME);
	by_now();
}
