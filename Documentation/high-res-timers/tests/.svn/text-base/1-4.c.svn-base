/*   
 * Copyright (c) 2002, Intel Corporation. All rights reserved.
 * Created by:  julie.n.fleischer REMOVE-THIS AT intel DOT com
 * This file is licensed under the GPL license.  For the full content
 * of this license, see the COPYING file at the top level of this 
 * source tree.

 * Test that timer_gettime() sets itimerspec.it_value to the amount of
 * time remaining after it has expired once and is reloaded.  Also
 * test that timer_gettime() sets itimerspec.it_interval to the
 * interval remaining.
 * - Create and arm a timer for TIMERVALSEC seconds with interval
 *   TIMERINTERVALSEC seconds.
 * - Sleep for SLEEPSEC > TIMERVALSEC, but < TIMERINTERVAL seconds
 * - Call timer_gettime().
 * - Ensure the value returned is within ACCEPTABLEDELTA less than
 *   TIMERINTERVALSEC - (SLEEPSEC-TIMERVALSEC).
 * - Ensure that interval TIMERINTERVALSEC is returned.
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
#include <posix_time.h>
//#include "posixtest.h"

int main(int argc, char *argv[])
{
	struct sigevent ev;
	timer_t tid;
	struct itimerspec itsset, itsget;
	int delta;
	int expectedleft;

	ev.sigev_notify = SIGEV_SIGNAL;
	ev.sigev_signo = SIGCONT;

	itsset.it_interval.tv_sec = 5;
	itsset.it_interval.tv_nsec = 0;
	itsset.it_value.tv_sec = 2;
	itsset.it_value.tv_nsec = 0;

	if (timer_create(CLOCK_REALTIME_HR, &ev, &tid) != 0) {
		perror("timer_create() did not return success\n");
		return -1;
	}

	if (timer_settime(tid, 0, &itsset, NULL) != 0) {
		perror("timer_settime() did not return success\n");
		return -1;
	}

	if (sleep(3) != 0) {
		perror("sleep() did not return 0\n");
 		return -1;
	}

	if (timer_gettime(tid, &itsget) != 0) {
		perror("timer_gettime() did not return success\n");
		return -1;
	}

printf ("it value left is %d %d\n", itsget.it_value.tv_sec, itsget.it_value.tv_nsec);
	return -1;
}
