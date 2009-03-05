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

/* 2timer_test.c
 * 
 * This program demonstrates the use of POSIX.4 interval timers without 
 * queued signals. It simply creates a POSIX.4 interval timer that sends
 * a normal signal when it expires. The handler for this signal prints
 * a message, and increments a count. When the count reaches MAX_EXPIRATIONS,
 * the timer is deleted, and the program exits. 
 */

#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <signal.h>
#ifdef __linux__
#include <posix_time.h>
#endif
#include "utils.h"

#define TIMER1_SIGNAL SIGRTMAX-1
#define TIMER2_SIGNAL SIGRTMIN
#define DATA_VAL 17
#define MAX_EXPIRATIONS 100
#define MAX_SIGNALS 100

pid_t pid;
int expirations;

struct datum {
        siginfo_t info;
        int       overrun;
}datum[MAX_SIGNALS];

void print_siginfo(struct datum *ptr)
{
        //int overrun = ptr->overrun;
  siginfo_t *info = &ptr->info;
        int overrun = info->si_overrun;
  timer_t t;

  fprintft(stderr,"siginfo dump: ");
  fprintft(stderr,"si_signo: %d, ", info->si_signo);
  fprintft(stderr,"si_errno: %d, ", info->si_errno);
  switch (info->si_code) {
  case SI_TIMER:
    t = *(timer_t*)info->si_value.sival_ptr;
    fprintft(stderr,"si_code: SI_TIMER, %d", t);
#ifdef __linux__
    fprintft(stderr,", timer id: %d", info->si_tid);
#endif
    if (overrun) {
      fprintft(stderr,", overrun: %d", overrun);
    }
    if ((overrun != ptr->overrun && ptr->overrun > 0) || overrun < 0){
            fprintf(stderr,SETCOLOR_FAILURE 
                   " info overrunn %d, getoverrun %d\n"
                   SETCOLOR_NORMAL,overrun,ptr->overrun);
            global_error++;
    } else {
            fprintft(stderr,"\n");
    }
    break;
  default:
    fprintft(stderr,"si_code: %d\n", info->si_code);
    break;
  }
}
void print_siginfo2(siginfo_t *info) {
        struct datum datum;

        datum.info = *info;
        datum.overrun = timer_getoverrun(info->si_tid);
        print_siginfo(&datum);
}

void timer1_handler(int signo)
{
	fprintft(stderr,"handler timer1 expired: signal %d expirations %d\n", 
		 signo, expirations);
	expirations++;
}

void timer1_action(int signo, siginfo_t *info, void *context)
{
	fprintft(stderr,"action timer1 expired: signal %d expirations %d\n", 
		 signo, expirations);
	print_siginfo2(info);
	expirations++;
}

void timer2_handler(int signo)
{
	fprintft(stderr,"handler timer2 expired: signal %d\n", signo);
}

void timer2_action(int signo, siginfo_t *info, void *context)
{
	fprintft(stderr,"action timer2 expired: signal %d\n", signo);
	print_siginfo2(info);
}
int do_main(clock_t clock,int of1,int in1,int of2,int in2)
{
	timer_t t1, t2;
	struct sigevent sig1, sig2;
	struct itimerspec new_setting1, new_setting2, current;
	struct sigaction sa;
	sigset_t set;
	//siginfo_t info;
        struct datum *pptr,*ptr = &datum[0];
	
	Try(pid = getpid()); 

	expirations = 0;
        

	/* 
	 * Set up the signal event that will occur when the timer 
	 * expires. 
	 */	
	sig1.sigev_notify = SIGEV_SIGNAL;
	sig1.sigev_signo = TIMER1_SIGNAL;
	sig1.sigev_value.sival_ptr = &t1;

	sig2.sigev_notify = SIGEV_SIGNAL;
	sig2.sigev_signo = TIMER2_SIGNAL;
	sig2.sigev_value.sival_ptr = &t2;

	/*
	 * Initialize the timer setting structure.
	 */
	new_setting1.it_value.tv_sec = 0L;
	new_setting1.it_value.tv_nsec = of1;
	new_setting1.it_interval.tv_sec = 0L;
	new_setting1.it_interval.tv_nsec = in1;

	new_setting2.it_value.tv_sec = 0L;
	new_setting2.it_value.tv_nsec = of2;
	new_setting2.it_interval.tv_sec = 0L;
	new_setting2.it_interval.tv_nsec = in2;

	/* 
	 * Set up and install a signal handler for the signal that 
	 * the timer will send. 
	 */
#if 0
	sa.sa_handler = timer1_handler;
	sa.sa_flags = 0;
#else
	sa.sa_flags = SA_SIGINFO;
	sa.sa_sigaction = timer1_action;
#endif
	sigemptyset(&sa.sa_mask);

	Try(sigaction(TIMER1_SIGNAL, &sa, NULL));
#if 0
	sa.sa_handler = timer2_handler;
	sa.sa_flags = 0;
#else
	sa.sa_flags = SA_SIGINFO;
	sa.sa_sigaction = timer2_action;
#endif
	sigemptyset(&sa.sa_mask);

	Try(sigaction(TIMER2_SIGNAL, &sa, NULL));	
	/*
	 * Create and set the timer.
	 */

	Try(timer_create(clock, &sig1, &t1));

	Try(timer_create(clock, &sig2, &t2));

	sigemptyset(&set);
	sigaddset(&set, TIMER1_SIGNAL);
	sigaddset(&set, TIMER2_SIGNAL);

	sigprocmask(SIG_BLOCK, &set, NULL);

	Try(timer_settime(t1, 0, &new_setting1, NULL)); 
	timer_gettime(t1, &current);
	fprintft(stderr,
		 "timer id %d: it_value=%ld.%09ld, it_interval=%ld.%09ld\n",
               t1,
	       current.it_value.tv_sec,
	       current.it_value.tv_nsec,
	       current.it_interval.tv_sec,
	       current.it_interval.tv_nsec);
	Try(timer_settime(t2, 0, &new_setting2, NULL)); 

	timer_gettime(t2, &current);
	fprintft(stderr,
		 "timer id %d: it_value=%ld.%09ld, it_interval=%ld.%09ld\n",
               t2,
	       current.it_value.tv_sec,
	       current.it_value.tv_nsec,
	       current.it_interval.tv_sec,
	       current.it_interval.tv_nsec);

	//sleep(1);
	/* 
	 * Busy wait until the timer expires MAX_EXPIRATIONS number 
	 * of times.
	 */
	while ((expirations < MAX_EXPIRATIONS) && 
	       (ptr != &datum[MAX_SIGNALS])) {
	  sigwaitinfo(&set, &ptr->info);
          //ptr->overrun = timer_getoverrun(ptr->info.si_tid);
	  if (ptr->info.si_signo == TIMER2_SIGNAL) expirations++;
          ptr++;
	}
	/* 
	 * Delete the timer.
	 */	
	Try(timer_delete(t1)); 
	Try(timer_delete(t2));
	sigprocmask(SIG_UNBLOCK, &set, NULL);
        pptr = &datum[0];
        while ( pptr != ptr) {
                print_siginfo(pptr);
                pptr++;
        }



	return 0;
}

int main(int argc, char * argv[])
{
	init_test(argc, argv);
#if 1
        fprintfd(stderr,"Using CLOCK_REALTIME \n");
        do_main(CLOCK_REALTIME,12345678L,20000000L,10000000L,10000000L);
        fprintfd(stderr,"Using CLOCK_MONOTONIC \n");
        do_main(CLOCK_MONOTONIC,12345678L,20000000L,10000000L,10000000L);
        IF_HIGH_RES {
                fprintfd(stderr,"Using CLOCK_REALTIME_HR \n");
                do_main(CLOCK_REALTIME_HR,12345678L,2000000L,
                        10000000L,1000000L);
                fprintfd(stderr,"Using CLOCK_MONOTONIC_HR \n");
                do_main(CLOCK_MONOTONIC_HR,10000000L,2000L,10000000L,1000L);
        }
#endif
        IF_HIGH_RES {
                fprintfd(stderr,"Using CLOCK_REALTIME_HR \n");
                do_main(CLOCK_REALTIME_HR,552345678L,20000L,10000000L,16000L);
        }else {
                fprintfd(stderr,"Using CLOCK_REALTIME \n");
                do_main(CLOCK_REALTIME,12345678L,20000000L,10000000L,10000000L);
        }
	by_now();
}
