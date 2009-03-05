/*
 *   Copyright (C) 2002  MontaVista Software
 *                       George Anzinger (george@mvista.com)
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
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <signal.h>
// #include <assert.h>
#include <sys/utsname.h>
#ifdef __linux__
#include <posix_time.h>
#endif
#include "utils.h"


#define MAX_NOF_TIMERS 6000

int timer_id_in_sival_int = 0;

struct timespec ref;

static void reset_ref_time(void)
{
  clock_gettime(CLOCK_REALTIME, &ref);
}

static void print_rel_time(void)
{
  struct timespec now;
  clock_gettime(CLOCK_REALTIME, &now);
  now.tv_sec -= ref.tv_sec;
  now.tv_nsec -= ref.tv_nsec;
  if (now.tv_nsec < 0) {
    now.tv_sec--;
    now.tv_nsec += 1000000000;
  }
  fprintft(stderr, "%ld.%09ld", now.tv_sec, now.tv_nsec);
}

int sival_expected;

static void print_siginfo(int signo, siginfo_t *info)
{
  int overrun;

  print_rel_time();
  fprintft(stderr, ": siginfo dump: ");
  fprintft(stderr, "si_signo: %d [%d], ", info->si_signo, signo);
  fprintft(stderr, "si_errno: %d, ", info->si_errno);
  fprintft(stderr, "si_value: int=%d ptr=%p ",
	 info->si_value.sival_int, info->si_value.sival_ptr);
  if (sival_expected != -1) {
    assert(info->si_value.sival_int == sival_expected);
  }
  switch (info->si_code) {
  case SI_USER:
    fprintft(stderr, "si_code: SI_USER");
    fprintft(stderr, ", si_pid: %d", info->si_pid);
    fprintft(stderr, ", si_uid: %d\n", info->si_uid);
    break;
  case SI_QUEUE:
    fprintft(stderr, "si_code: SI_QUEUE");
    fprintft(stderr, ", si_pid: %d", info->si_pid);
    fprintft(stderr, ", si_uid: %d\n", info->si_uid);
    break;
  case SI_ASYNCIO:
    fprintft(stderr, "si_code: SI_ASYNCIO\n");
    break;
  case SI_TIMER:
#if 0
    t = (timer_id_in_sival_int) ? info->si_value.sival_int : *(timer_t*)info->si_value.sival_ptr;
#endif
    fprintft(stderr, "si_code: SI_TIMER");
#ifdef __linux__
    fprintft(stderr, ", timer id: %d", info->si_tid);
#endif
    overrun = timer_getoverrun(info->si_tid);
    switch (overrun) {
    case -1:
      fprintft(stderr, ", timer_getoverrun() failed: %s\n", strerror(errno));
      break;
    case 0:
      fprintft(stderr, "\n");
      break;
    default:
      fprintft(stderr, ", overrun: %d\n", overrun);
      break;
    }
    break;
  case SI_MESGQ:
    fprintft(stderr, "si_code: SI_MESGQ\n");
    break;
  default:
    fprintft(stderr, "si_code: %d\n", info->si_code);
    break;
  }
}

void alrm_handler(int signo, siginfo_t *info, void *context)
{
	print_siginfo(signo, info);
}

int main(int argc, char * argv[])
{
	int retval;
	int signo;
	timer_t t = 0;
	int i,j;
	timer_t tt[MAX_NOF_TIMERS];
	struct itimerspec ispec;
	struct itimerspec ospec;
	struct sigevent timer_event_spec;
	struct sigaction sa;
	union sigval val;
	sigset_t set;
	siginfo_t info;
	struct utsname utsname;

	init_test(argc, argv);
	sigemptyset(&set);
	sigprocmask(SIG_SETMASK, &set, NULL);

	sa.sa_sigaction = alrm_handler;
	sa.sa_flags = SA_SIGINFO;
	sigemptyset(&sa.sa_mask);

	Try(sigaction(SIGALRM, &sa, NULL));

	Try(sigaction(SIGRTMIN, &sa, NULL));

	Try(sigaction(SIGRTMIN + 1, &sa, NULL));

	try(EINVAL, timer_delete(0));

	try(0, timer_create(CLOCK_REALTIME, NULL, &t));

	try(0, timer_delete(t));

	try(EINVAL, timer_delete(t));

	try(EINVAL, timer_delete(-1));

	try(EINVAL, timer_delete(100));

	fprintfd(stderr,"\nattempt to create too many timers: "
		 "expect failure at timer %d\n", MAX_NOF_TIMERS);
	for (i = 0; i < MAX_NOF_TIMERS; i++) {
	  	retval = timer_create(CLOCK_REALTIME, NULL, &tt[i]);
		if (retval) {
			fprintfd(stderr, "timer_create(CLOCK_REALTIME) %d "
				"failed: %s\n", i, strerror(errno));
			break;
		}
	}
        if ( i < MAX_NOF_TIMERS) {
                assert((i > 30) && (retval == -1));
                j = i;
        }else {
                assert( (i == MAX_NOF_TIMERS) && (retval != -1));
                fprintfd(stderr, "timer_create(CLOCK_REALTIME) %d "
			"timers created\n",i);
                j = i ;
        }

	fprintft(stderr, "\ndelete these timers: "
		"expect failure at timer %d\n", j);
	    for (i = 0; i <= j; i++) {
	  	retval = timer_delete(tt[i]);
		if (retval) {
			fprintft(stderr, "timer_delete(CLOCK_REALTIME) "
				 "%d failed: %s\n", i, strerror(errno));
			break;
		}
	}
	assert((i == j) && (retval == -1));

	try(0,timer_create(CLOCK_REALTIME, NULL, &t));

	// set absolute time (no time specification): expect failure

	try(EINVAL, timer_settime(t, TIMER_ABSTIME, NULL, NULL));

	// set relative time (no time specification): expect failure\n");
	try(EINVAL, timer_settime(t, 0, NULL, NULL));

	//set absolute time (bogus time specification): expect failure\n");

	try(EFAULT, timer_settime(t, TIMER_ABSTIME, 
				  (struct itimerspec*)1, NULL));

	// set absolute time (bogus time specification): expect failure\n");

	try(EINVAL, timer_settime(t, TIMER_ABSTIME, NULL, 
				  (struct itimerspec*)1));

	// set absolute time (bogus time specification): expect failure\n");

	try(EFAULT, timer_settime(t, TIMER_ABSTIME, 
				  (struct itimerspec*)1, 
				  (struct itimerspec*)1));

	// set relative time (bogus time specification): expect failure\n");

	try(EFAULT, timer_settime(t, 0, (struct itimerspec*)1, NULL));

	// set relative time (bogus time specification): expect failure\n");

	try(EINVAL, timer_settime(t, 0, NULL, (struct itimerspec*)1));

	// set relative time (bogus time specification): expect failure\n");
	try(EFAULT, timer_settime(t, 0, (struct itimerspec*)1,
				  (struct itimerspec*)1));

	Try(clock_gettime(CLOCK_REALTIME, &ispec.it_value));

	ispec.it_value.tv_sec += 2;
	ispec.it_value.tv_nsec = 0;
	ispec.it_interval.tv_sec = 0;
	ispec.it_interval.tv_nsec = 0;
	reset_ref_time();

	// set timer (absolute time) 2 seconds in the future\n");
	try(0, timer_settime(t, TIMER_ABSTIME, &ispec, &ospec));

	fprintft(stderr, 
		"timer_settime: old setting value=%ld.%09ld, "
		"interval=%ld.%09ld\n",
		ospec.it_value.tv_sec, ospec.it_value.tv_nsec,
		ospec.it_interval.tv_sec, ospec.it_interval.tv_nsec);

	reset_ref_time();

	//test 19: set timer (absolute time) same time\n");
	try(0, timer_settime(t, TIMER_ABSTIME, &ispec, &ospec));

	fprintft(stderr,
		 "timer_settime: old setting value=%ld.%09ld, "
		 "interval=%ld.%09ld\n",
		 ospec.it_value.tv_sec, ospec.it_value.tv_nsec,
		 ospec.it_interval.tv_sec, ospec.it_interval.tv_nsec);

	// timer_gettime bogus timer id (-1): expect failure\n");
	try(EINVAL, timer_gettime(-1, NULL));

	// timer_gettime good timer id, NULL timespec pointer: expect failure
	try(EFAULT, timer_gettime(t, NULL));

	// timer_gettime good timer id, bogus timespec pointer: expect failure
	try(EFAULT,timer_gettime(t, (struct itimerspec*)1));

	// timer_gettime good timer id, good timespec pointer\n");
	try(0,timer_gettime(t, &ispec));

	fprintft(stderr, "timer_gettime: value=%ld.%09ld, interval=%ld.%09ld\n",
		 ispec.it_value.tv_sec, ispec.it_value.tv_nsec,
		 ispec.it_interval.tv_sec, ispec.it_interval.tv_nsec);

	// send ALRM signal to self with kill()\n");
	reset_ref_time();
	sival_expected = -1;
	try(0,kill(getpid(), SIGALRM));
	
	// send ALRM signal to self with sigqueue()\n");
	reset_ref_time();
	sival_expected = val.sival_int = 4;
	try(0,sigqueue(getpid(), SIGALRM, val));

	sigemptyset(&set);
	sigaddset(&set, SIGALRM);
	sigprocmask(SIG_BLOCK, &set, NULL);
	
	// send ALRM signal to self with kill() (signal blocked)\n");
	try(0,kill(getpid(), SIGALRM));

	// wait for ALRM signal with info\n");
	reset_ref_time();
	info.si_uid = 1;
	sival_expected = -1;
	signo = sigwaitinfo(&set, &info);
	print_siginfo(signo, &info);
	
	// send ALRM signal to self with sigqueue() (signal blocked)\n");
	sival_expected = val.sival_int = 4;
	try(0,sigqueue(getpid(), SIGALRM, val));

	uname(&utsname);
	if (strncmp(utsname.release, "2.3", 3) <= 0) {
	  fprintfd(stderr, 
		  "\nLinux <= 2.3 does not carry siginfo data for SIGALRM\n");
	  sival_expected = -1;
	}

	// wait for ALRM signal with info\n");
	reset_ref_time();
	info.si_uid = 1;
	signo = sigwaitinfo(&set, &info);
	print_siginfo(signo, &info);
	sigprocmask(SIG_UNBLOCK, &set, NULL);

	// timer_gettime()\n");
	sleep(1);
	try(0, timer_gettime(t, &ispec));

	fprintft(stderr, "timer_gettime: value=%ld.%09ld, interval=%ld.%09ld\n",
		 ispec.it_value.tv_sec, ispec.it_value.tv_nsec,
		 ispec.it_interval.tv_sec, ispec.it_interval.tv_nsec);

	sival_expected = -1;
	sleep(1);		/* wait for timer expiration of test 18 */
	
	// timer_delete()\n");
	try(0,timer_delete(t));

	// timer_gettime: deleted timer and NULL itimer_spec: expect failure
	try(EINVAL,timer_gettime(t, NULL));

	/*
	 * test to check timer cancellation by deletion
	 */

	// create default timer\n");
	try(0,timer_create(CLOCK_REALTIME, NULL, &t));

	ispec.it_value.tv_sec = 2;
	ispec.it_value.tv_nsec = 0;
	ispec.it_interval.tv_sec = 0;
	ispec.it_interval.tv_nsec = 0;
	reset_ref_time();

	try(0,timer_settime(t, 0, &ispec, &ospec));

	fprintft(stderr, "timer_settime: old setting value=%ld.%09ld, "
		 "interval=%ld.%09ld\n",
		 ospec.it_value.tv_sec, ospec.it_value.tv_nsec,
		 ospec.it_interval.tv_sec, ospec.it_interval.tv_nsec);

	// delete the timer\n");
	try(0,timer_delete(t));
	fprintft(stderr, "wait 3 seconds\n");
	sleep(3);
	fprintft(stderr, "no timer expiration expected\n");

	/*
	 * test to check relative timer expirations
	 */

	// Expiration in one second with relative timer\n");

	try(0,timer_create(CLOCK_REALTIME, NULL, &t));

	ispec.it_value.tv_sec = 1;
	ispec.it_value.tv_nsec = 0;
	ispec.it_interval.tv_sec = 0;
	ispec.it_interval.tv_nsec = 0;
	reset_ref_time();
	try(0,timer_settime(t, 0, &ispec, &ospec));

	fprintft(stderr, "timer_settime: old setting value=%ld.%09ld, "
		 "interval=%ld.%09ld\n",
		 ospec.it_value.tv_sec, ospec.it_value.tv_nsec,
		 ospec.it_interval.tv_sec, ospec.it_interval.tv_nsec);

	try(0,timer_gettime(t, &ispec));

	fprintft(stderr, "timer_gettime: value=%ld.%09ld, interval=%ld.%09ld\n",
	       ispec.it_value.tv_sec, ispec.it_value.tv_nsec,
	       ispec.it_interval.tv_sec, ispec.it_interval.tv_nsec);
	
	fprintft(stderr, "waiting for signal to arrive...\n");
	sleep(2);

	try(0,timer_gettime(t, &ispec));

	fprintft(stderr, "timer_gettime: value=%ld.%09ld, interval=%ld.%09ld\n",
	       ispec.it_value.tv_sec, ispec.it_value.tv_nsec,
	       ispec.it_interval.tv_sec, ispec.it_interval.tv_nsec);
	
	if(ispec.it_value.tv_sec != 0 || ispec.it_value.tv_nsec != 0) 
		myerror("timer_gettime failed: it_value should be zero.\n");

	reset_ref_time();
	try(0,timer_settime(t, 0, &ispec, &ospec));

	fprintft(stderr, "timer_settime: old setting value=%ld.%09ld, "
		 "interval=%ld.%09ld\n",
		 ospec.it_value.tv_sec, ospec.it_value.tv_nsec,
		 ospec.it_interval.tv_sec, ospec.it_interval.tv_nsec);

	try(0,timer_gettime(t, &ispec));

	fprintft(stderr, "timer_gettime: value=%ld.%09ld, interval=%ld.%09ld\n",
	       ispec.it_value.tv_sec, ispec.it_value.tv_nsec,
	       ispec.it_interval.tv_sec, ispec.it_interval.tv_nsec);
	try(0,timer_delete(t));

	/*
	 * Test to see if timer goes off immediately if not a future time is
	 * provided with TIMER_ABSTIME 
	 */
	// set up timer to go off immediately, followed by 10 ticks at 10 Hz
	timer_event_spec.sigev_notify = SIGEV_SIGNAL;
	timer_event_spec.sigev_signo = SIGRTMIN + 0;
	sival_expected = timer_event_spec.sigev_value.sival_int = 0x1234;
	try(0,timer_create(CLOCK_REALTIME, &timer_event_spec, &t));

	try(0,clock_gettime(CLOCK_REALTIME, &ispec.it_value));

	ispec.it_value.tv_sec -= 1;
	ispec.it_interval.tv_sec = 0;
	ispec.it_interval.tv_nsec = 100000000;
	reset_ref_time();
	try(0,timer_settime(t, TIMER_ABSTIME, &ispec, &ospec));

	fprintft(stderr, "timer should have expired now\n");
	fprintft(stderr, "timer_settime: old setting value=%ld.%09ld, "
		 "interval=%ld.%09ld\n",
		 ospec.it_value.tv_sec, ospec.it_value.tv_nsec,
		 ospec.it_interval.tv_sec, ospec.it_interval.tv_nsec);
	try(0,timer_gettime(t, &ispec));

	fprintft(stderr, "timer_gettime: value=%ld.%09ld, interval=%ld.%09ld\n",
		 ispec.it_value.tv_sec, ispec.it_value.tv_nsec,
		 ispec.it_interval.tv_sec, ispec.it_interval.tv_nsec);
	fprintft(stderr, "catch 10 signals\n");
	for (i = 0; i < 10; i++) sleep(1);

        by_now();
}
