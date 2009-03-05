/*
*   Copyright (C) 2002  MontaVista Software 
*                       Robert Love (rml@tech9.net)
*                       George Anzinger <george@mvista.com
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
/*
 * tests/abs_timer_test.c - This code exists to test the correct
 * functioning of absolsute timers WRT clock setting.  It must be run as
 * root to allow the clock to be set.  (Derived from the simular test
 * for clock_nanosleep.)
 *
 * 21 June 2004
 * 
 */
#define debug
#include <sys/types.h>
#include <sys/wait.h>

#include <string.h>
#include <linux/unistd.h>
#include "../lib/posix_time.h"
#include "utils.h"

#define SIGNAL1  SIGPROF //SIGUNUSED
#define SIGNAL2 SIGUSR1  //SIGHUP
WAIT_VARS();
int sival_expected;

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
  fprintft(stderr,"%ld.%09ld", now.tv_sec, now.tv_nsec);
}

static void print_siginfo(int signo, siginfo_t *info)
{
  int overrun;

  print_rel_time();
  fprintft(stderr,": siginfo dump: ");
  fprintft(stderr,"si_signo: %d [%d], ", info->si_signo, signo);
  fprintft(stderr,"si_errno: %d, ", info->si_errno);
  fprintft(stderr,"si_value: int=%d ptr=%p ",
	 info->si_value.sival_int, info->si_value.sival_ptr);
  if (sival_expected != -1) {
    assert(info->si_value.sival_int == sival_expected);
  }
  switch (info->si_code) {
  case SI_USER:
    fprintft(stderr,"si_code: SI_USER");
    fprintft(stderr,", si_pid: %d", info->si_pid);
    fprintft(stderr,", si_uid: %d\n", info->si_uid);
    break;
  case SI_QUEUE:
    fprintft(stderr,"si_code: SI_QUEUE");
    fprintft(stderr,", si_pid: %d", info->si_pid);
    fprintft(stderr,", si_uid: %d\n", info->si_uid);
    break;
  case SI_ASYNCIO:
    fprintft(stderr,"si_code: SI_ASYNCIO\n");
    break;
  case SI_TIMER:
#if 0
    t = (timer_id_in_sival_int) ? info->si_value.sival_int : *(timer_t*)info->si_value.sival_ptr;
#endif
    fprintft(stderr,"si_code: SI_TIMER");
#ifdef __linux__
    fprintft(stderr,", timer id: %d", info->si_tid);
#endif
    overrun = timer_getoverrun(info->si_tid);
    switch (overrun) {
    case -1:
      fprintft(stderr,", timer_getoverrun() failed: %s\n", strerror(errno));
      break;
    case 0:
      fprintft(stderr,"\n");
      break;
    default:
      fprintft(stderr,", overrun: %d\n", overrun);
      break;
    }
    break;
  case SI_MESGQ:
    fprintft(stderr,"si_code: SI_MESGQ\n");
    break;
  default:
    fprintft(stderr,"si_code: %d\n", info->si_code);
    break;
  }
}
/*
 * some funky signal handlers, don't do much, but allow us to notice them
 */
void handler1(int signo, siginfo_t *info, void *context)
{
	fprintft(stderr,"handler1 entered: signal %d\n", signo);
}
int waitf;
void handler2(int signo, siginfo_t *info, void *context)
{
	fprintfb(stderr,"handler2 entered: signal %d\n", signo);
        waitf = 1;
}

int test(clock_t clock,double EPS)
{
	struct timespec ns, ts, rs, bs, tm, reser, zero = {0,0};
	struct timespec asked, actual, start;
	int i,signo, monoclock = 0;
        double diff, err, Eps, er;
        pid_t child;
        sigset_t set;
	int res;
	timer_t t;
	long offset;
	struct sigevent timer_event_spec;
	struct itimerspec ispec, ispecr;
	siginfo_t info;
	char *color;
	int pr;
//        Try(NSEC_PER_SEC == 1000000000?0:-1);

	ts.tv_sec = 0;
	ts.tv_nsec = 100;

        sigemptyset(&set);
        sigaddset(&set, SIGNAL1);
        sigaddset(&set, SIGSTOP);
        sigaddset(&set, SIGCONT);
        sigprocmask(SIG_UNBLOCK, &set, NULL);
        wait_setup(SIGNAL2);
        sigemptyset(&set);
	sigaddset(&set, SIGRTMIN);
        sigprocmask(SIG_BLOCK, &set, NULL);

	sival_expected = timer_event_spec.sigev_value.sival_int = 0x1234;
	timer_event_spec.sigev_notify = SIGEV_SIGNAL;
	timer_event_spec.sigev_signo = SIGRTMIN + 0;
	Try(timer_create(clock, &timer_event_spec, &t));

	Try(clock_getres(clock, &ts));
	res = ts.tv_nsec;

	ts.tv_nsec = -1;

        /*
         * Now we get really dirty.  The spec says we should be able to
         * reset the clock while the timer(absolute) is at work and
         * still wake up at the correct time.  We need to be su to do 
         * this, but then that will just fall out as an error.
         * We will let the child do the clock setting...
         */
        if( clock == CLOCK_MONOTONIC || clock == CLOCK_MONOTONIC_HR){
		monoclock = 1;
        }
#define CHILD_DELAY 10000000;

        fprintfd(stderr, "\nTesting behavor with clock seting...\n");
        for (i = -2; i < 8; i+=8) {
                int exv;
                char * what = i > 0 ? "Advancing" :"Retarding";
                wait_flush();
                fprintft(stderr,"%s the clock %d seconds\n",what, i >0 ? i:-i); 
 		flushit();
		fsync(STDERR_FILENO);
		if(!(child = fork())){
                        /* The child... */
                
                        ts.tv_sec = 0;
                        ts.tv_nsec = CHILD_DELAY;
                        Try0(clock_nanosleep(clock, 0, &ts, NULL));
                        /* Advance/ retard the clock */
                        Try(clock_gettime(clock,&bs));
                        ts = bs;
                        ts.tv_sec += i;
                        if(clock_settime(CLOCK_REALTIME, &ts) <0){
                                if( errno == EPERM){
                                        fprintf(stderr,SETCOLOR_WARNING
                                                "Must be able to set time to do"
                                                " this test, come back then:)\n"
                                                SETCOLOR_NORMAL);
                                        // 
                                        //  Generate EINTR error to log it
                                        //
                                        Try(kill(getppid(),SIGNAL1));
                                        exit(0);
                                }
                                Try(("clock_settime error",-1));
                        }else{
                                Try(clock_gettime(clock,&ns));
                                diff = timerdiff(&ns,&ts);
                                if(!monoclock && ((diff < 0) || (diff > EPS))) {
                                       fprintf(stderr,SETCOLOR_WARNING
                                               "Clock did not seem to move"
                                               "\n was:\t\t%lds %ldns"
                                               "\n requested:\t%lds %ldns\n"
                                               " now:\t\t%lds %ldns\n"	
                                               " diff is %12.9fsec\n" 
                                               SETCOLOR_NORMAL,
                                               bs.tv_sec, bs.tv_nsec,
                                               ts.tv_sec, ts.tv_nsec, 
                                               ns.tv_sec, ns.tv_nsec, diff);
                                }

                        }
                        wait_sync();
                        Try(clock_gettime(CLOCK_REALTIME,&ts));
                        ts.tv_sec -= i;
                        Try(clock_settime(CLOCK_REALTIME,&ts));                   
                        exit(0);
                }
                /* The parent ... */
                Try(clock_gettime(clock, &ts));
                rs.tv_sec = 0;
                rs.tv_nsec = 500000000;
                timersum(&ts,&ts,&rs);
		ispec.it_value = ts;
		reset_ref_time();
		ispec.it_interval = zero;
		try(0, timer_settime(t, TIMER_ABSTIME, &ispec, NULL));
		signo = sigwaitinfo(&set, &info);
                Try(clock_gettime(clock, &ns));
		print_siginfo(signo, &info);
//                try0(0,clock_nanosleep(clock, TIMER_ABSTIME, &ts, NULL));
                wait_send(child);
                /* 
                 * Correct expected time.  If clock moved back, we will be
                 * late by (back - req.)  if forward, we should be on time.
                 */
                if ( !monoclock && (i > 0)){
                        ts.tv_sec += i;
                        rs.tv_nsec -= CHILD_DELAY;
                        timersubtract(&ts,&ts,&rs);
                }
 
                //roundtores(&ts,res);  // expected wake up time
                diff = timerdiff(&ns, &ts);
                if ( diff > EPS || diff < 0){
                        char *ls = long_short(diff);
                        myerror("slept too");
                        fprintf(stderr, SETCOLOR_FAILURE 
                                "%s!\n requested:\t%lds %ldns\n"
                                " now:\t\t%lds %ldns\n"	
                                " diff is %12.9fsec\n" SETCOLOR_NORMAL, ls,
                                ts.tv_sec, ts.tv_nsec, 
                                ns.tv_sec, ns.tv_nsec, diff);
                        fprintf(stderr, SETCOLOR_FAILURE 
                                "Note this may indicate that the timer \n"
                                "did NOT respond to the clock setting\n"
                                SETCOLOR_NORMAL);

                }
                waitpid(child, &exv, 0);
        }
/*
 * And now for something completly different...  We want to be test the
 * SIGEV_NONE option.  These timers never deliver a signal, however, we
 * should still be able to ask when they will expire.  And, again, they
 * should handle the abs time thing and adjust with the clock setting.
 */
        fprintfd(stderr, "\nTesting SIGEV_NONE behavor with clock seting...\n");
	Try(timer_delete(t));
	timer_event_spec.sigev_notify = SIGEV_NONE;
	Try(timer_create(clock, &timer_event_spec, &t));
	Try(clock_gettime(clock, &ts));
	rs.tv_sec = 0;
	rs.tv_nsec = 500000000;
	timersum(&ts,&ts,&rs);
	/*
	 * set it up to expire on the 250ms mark and every 250 ms after.
	 */
#define INTERV 250000000
	roundtores(&ts, 250000000);
	ispec.it_value = ts;
	reset_ref_time();
	ispec.it_interval.tv_nsec = INTERV;
	ispec.it_interval.tv_sec = 0;
	asked = ispec.it_interval;
	Try( timer_settime(t, TIMER_ABSTIME, &ispec, NULL));
	start = ispec.it_value;
	/*
	 * So now if we ask about time left (after the first interal) we
	 * should always get at time less than 250ms AND that plus now
	 * should be real close to 0, 250, 500, or 750ms past the last
	 * second.

	 * We also have an error accumulating here due to the clock res
	 * not being what we asked.  (Particularly true for low res.)
	 * To fix this we keep a running total of the error in trying to
	 * hit the .25 mark.  This will be the error in the interval
	 * used (as apposed to requested) times the number of such
	 * intervals to cover the clock set difference.  This error is
	 * only present when we advance the clock and it accumulates.
	 *
	 * The other error we have is that each correction is rolled
	 * into the expire time with a resolution correction.  This will
	 * increase the expire time by an average of 0.5 * resolution
	 * and a max of a full resolution.
	 */
	Try(clock_gettime(clock, &ns));
	//timersubtract(&rs, &ts, &ns);
	Try0(clock_nanosleep(clock, TIMER_ABSTIME, &ts, NULL));
	tm = zero;
	reser = zero;
	err = 0.0; 
	Eps = EPS;
	for (i = 0; i < 20; i++) {
		Try(timer_gettime(t, &ispecr));
		Try(clock_gettime(clock, &bs));
		fprintfb(stderr, 
			 "Raw timer time : %lds %ldns\n"
			 "Raw inter time : %lds %ldns\n",
			 ispecr.it_value.tv_sec, ispecr.it_value.tv_nsec,
			 ispecr.it_interval.tv_sec, ispecr.it_interval.tv_nsec);
		fprintfb(stderr,
			 "Clock time :  %lds %ldns\n",
			 bs.tv_sec, bs.tv_nsec);
		actual = ispecr.it_interval;
		diff = timerdiff(&bs, &start);
		start = bs;
		er = timerdiff(&actual, &asked) * 
			diff / timerdiff(&actual, &zero);
		if (i && i < 11) 
			err += er;
		dbltotimer(err, &reser);
		//timersum(&reser, &reser, &bs);
		
		rs = actual;
		ns = ispecr.it_value;
		diff = timerdiff(&rs, &ns);
		if (diff > timerdiff(&rs, &zero)) {
			myerror("time too long");
			color = SETCOLOR_FAILURE;
			pr = 1;
		} else {
			color = "";
			pr = debugf;
		}
		if (pr) {
			fprintf(stderr, "%s"  
                                "\n expected:\t%lds %ldns or less\n"
                                "    found:\t\t%lds %ldns\n"
				"res error:\t\t%12.9f sec\n"
                                " diff is %12.9fsec\n" SETCOLOR_NORMAL,
				color,
                                rs.tv_sec, rs.tv_nsec, 
                                ns.tv_sec, ns.tv_nsec,
				timerdiff(&reser, &zero),
				diff);
		}
		timersubtract(&ns, &ns, &reser);
		timersum(&ts, &ns, &bs);
		offset = ts.tv_nsec;
		diff = (double)offset / NSEC_PER_SEC;
		if ((( diff         >= 0) && ( diff         < Eps)) || 
		    (((diff - .250) >= 0) && ((diff - .250) < Eps)) || 
		    (((diff - .500) >= 0) && ((diff - .500) < Eps)) ||
		    (((diff - .750) >= 0) && ((diff - .750) < Eps ))) {
			pr = debugf;
			color = "";
		} else {
			myerror("time wrong");
			color = SETCOLOR_FAILURE;
			pr =1;
		}
		if (pr) {
			fprintf(stderr, "%s"  
				"\n expected one of: 0.0, 0.25, 0.5 or 0.75\n"
				"           found:%12.9f sec\n" 
				"     res   error:%12.9f sec\n"
				SETCOLOR_NORMAL,
				color,
				diff, timerdiff(&reser, &zero));
		}
		/*
		 * Here we jiggle the clock...
		 */
		Try(clock_gettime(CLOCK_REALTIME,&bs));
		ts.tv_sec = i % 10;
		ts.tv_nsec = 125000555 + (i % 10) * (long)(EPS * NSEC_PER_SEC);
		if (i >= 10) {
			timersubtract(&tm, &tm, &ts);
			fprintft(stderr,"Retarding the clock by %12.9fsec\n",
				timerdiff(&ts, &zero));
			timersubtract(&ts, &bs, &ts);
		} else {
			timersum(&tm, &tm, &ts);
			fprintft(stderr,"Advancing the clock by %12.9fsec\n",
				 timerdiff(&ts, &zero));
			timersum(&ts, &bs, &ts);
			/*
			 * Here we do the error calc.
			 */
		}
		if (!monoclock) {
			Eps += (double)res / NSEC_PER_SEC;
			fprintfb(stderr,
				 "Eps increased by %d nsec to"
				 " : %12.9fsec\n", res, Eps);
		}
		normalize(&ts);
 		Try(clock_settime(CLOCK_REALTIME, &ts));
		normalize(&tm);
		bs.tv_sec = 0;
		bs.tv_nsec = 255000000 * (i % 10);
		normalize(&bs);
		bs.tv_sec = 0;
		Try0(clock_nanosleep(clock, 0, &bs, NULL));
  	
	}
	/*
	 * set the clock back to the right time
	 */
	Try(clock_gettime(CLOCK_REALTIME,&bs));
	timersubtract(&ts, &bs, &tm);
	normalize(&ts);
	Try(clock_settime(CLOCK_REALTIME, &ts));
        return 0;
}
#define EPS		0.030000000	/* max error in s */
#define EPS_HR		0.003500000	/* max error in s */
int main(int argc, char *argv[])
{
	struct sigaction sa;
	int flag;
//	sigset_t set;
	init_test(argc, argv);
	/*
	 * flag 0, run all tests,
	 *      1, CLOCK_REALTIME 
	 *      2, CLOCK_REALTIME_HR
	 *      4, CLOCK_MONOTONIC
	 *      8, CLOCK_MONOTONIC_HR
	 *  Its a bit map folks.
	 */
	flag = (argc > 2) ? atoi(argv[2]) : 0xff;
	sa.sa_flags = SA_SIGINFO;
	sa.sa_sigaction = handler1;
	Try(sigaction(SIGNAL1, &sa, NULL));
	sa.sa_sigaction = handler2;
	Try(sigaction(SIGNAL2, &sa, NULL));

	if (flag & 1){
		fprintfd(stderr,"\nTesting absolute timers with"
			 " respect to clock setting(CLOCK_REALTIME...\n");
		test(CLOCK_REALTIME,EPS);
	}
	if (flag & 2){
		IF_HIGH_RES {
			fprintfd(stderr,"\nTesting absolute timers with respect"
				 " to clock setting(CLOCK_REALTIME_HR...\n");
			test(CLOCK_REALTIME_HR,EPS);
		}
	}
	if (flag & 4){
		fprintfd(stderr,"\nTesting absolute timers with"
			 " respect to clock setting(CLOCK_MONOTONIC...\n");
		test(CLOCK_MONOTONIC,EPS);
	}
	if (flag & 8){
		IF_HIGH_RES {
			fprintfd(stderr,"\nTesting absolute timers with respect"
				 " to clock setting(CLOCK_MONOTONIC_HR...\n");
			test(CLOCK_MONOTONIC_HR,EPS);
		}
	}
	by_now();
 }
