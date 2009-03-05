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
 * tests/clock_nanosleeptest.c - test program for clock_nanosleep(3)
 *
 * 30 March 2002
 * 
 */
#define debug
#include <sys/types.h>
#include <sys/wait.h>

#include <string.h>
#include <linux/unistd.h>
#include "../lib/posix_time.h"
#include "utils.h"

#define CLOCK_BAD	55		/* an invalid clock id */
#define TIMER_BAD	55		/* an invalid flag */
#define BAD_POINTER	(NULL+1)	/* NULL is too obvious */
#define SIGNAL1  SIGPROF //SIGUNUSED
#define SIGNAL2 SIGUSR1  //SIGHUP
WAIT_VARS();

int shortf = 1;                       /* if true shorten the tests */

/*
 * some funky signal handlers, don't do much, but allow us to notice them
 */
void handler1(int signo, siginfo_t *info, void *context)
{
	fprintfb(stderr, "handler1 entered: signal %d\n", signo);
}
int waitf;
void handler2(int signo, siginfo_t *info, void *context)
{
	fprintfb(stderr, "handler2 entered: signal %d\n", signo);
        waitf = 1;
}

int test(clock_t clock,double EPS)
{
	struct timespec ns, ts, rs, bs;
	int i;
        double diff;
        pid_t child;
        sigset_t new_mask;
	int res;
	int loops = shortf ? 2 : 5;
//        Try(NSEC_PER_SEC == 1000000000?0:-1);

	ts.tv_sec = 0;
	ts.tv_nsec = 100;

        sigemptyset(&new_mask);
        sigaddset(&new_mask, SIGNAL1);
        //sigaddset(&new_mask, SIGNAL2);
        sigaddset(&new_mask, SIGSTOP);
        sigaddset(&new_mask, SIGCONT);
        sigprocmask(SIG_UNBLOCK, &new_mask, NULL);
        wait_setup(SIGNAL2);

	/* bad clock id */
	try0(EINVAL, clock_nanosleep(CLOCK_BAD, 0, &ts, NULL));

	/* bad rqtp pointer */
	try0(EFAULT, clock_nanosleep(clock, 0, BAD_POINTER, NULL));

	/* bad rqtp pointer */
	try(EFAULT, nanosleep(BAD_POINTER, NULL));
	    
	/* bad rmtp pointer */
        // This one is hard to do as the return parameter is only
        // used if the call is interrupted and is not otherwise checked

	// try0(EFAULT, clock_nanosleep(clock, 0, &ts, BAD_POINTER));

	/* invalid sleep value (tv_nsec < 0) */
	Try(clock_getres(clock, &ts));
	res = ts.tv_nsec;

	ts.tv_nsec = -1;
	try0(EINVAL, clock_nanosleep(clock, 0, &ts, NULL));

	try(EINVAL, nanosleep(&ts, NULL));

	/* invalid sleep value (tv_nsec >= 1,000,000,000) */
	ts.tv_nsec = 1000000000;
	try0(EINVAL, clock_nanosleep(clock, 0, &ts, NULL));

	try(EINVAL, nanosleep(&ts, NULL));

	/* valid 200ns sleep */
	ts.tv_nsec = 200;
	try0(0, clock_nanosleep(clock, 0, &ts, &rs));

	/* valid 200ns sleep */
	ts.tv_nsec = 200;
	try(0,nanosleep(&ts, &rs));

	/* is rmtp sane (should be zero) ? */
        // again, rmtp is only set if the call is interrupted 
        // i.e. it returns EINTR
#if 0
	if (rs.tv_sec || rs.tv_nsec ){
                myerror("return rmtp is bad ");
		fprintf(stderr, SETCOLOR_FAILURE
			"(%lds, %ldns)\n" 
                        SETCOLOR_NORMAL,rs.tv_sec,rs.tv_nsec);
        }
#endif
	/* if TIMER_ABSTIME is specified, rmtp should not be touched */
	rs.tv_sec = rs.tv_nsec = 55;
        Try(clock_gettime(clock, &ts));
        ts.tv_nsec += 1000;
	try0(0,clock_nanosleep(clock, TIMER_ABSTIME, &ts, &rs));
	if (rs.tv_sec != 55 || rs.tv_nsec != 55){
                myerror("TIME_ABSTIME specified and rmtp touched");
		fprintf(stderr, SETCOLOR_FAILURE			
                        "(%lds, %ldns)!\n"
			SETCOLOR_NORMAL, rs.tv_sec, rs.tv_nsec);
		flushit();
        }

	/*
	 * make sure we awake reasonably close to the time requested and
	 * never awake earlier than requested
	 */


	for (i = 0; i < loops; i++) {
                double diff;
                int dot = 0;
		Try(clock_gettime(clock, &bs));
		ts.tv_sec = 3;
		ts.tv_nsec = 150000;
		Try0(clock_nanosleep(clock, 0, &ts, NULL));
                timersum(&bs,&bs,&ts);// expected wake up time
                //roundtores(&bs,res);
 
		Try(clock_gettime(clock, &ns));

		if ((diff = timerdiff(&ns, &bs)) > EPS || diff < 0){
                        char *ls = long_short(diff);
                        if( dot) fprintfd(stderr, "\n");
                        myerror("slept too");
			fprintf(stderr, SETCOLOR_FAILURE 
				"%s!\n requested:\t%lds %ldns\n"
				" now:\t\t%lds %ldns\n"	
                                " ititeration %d, diff is %12.9fsec\n" 
                                SETCOLOR_NORMAL, ls,
				bs.tv_sec, bs.tv_nsec, 
                                ns.tv_sec, ns.tv_nsec, i, diff);
                        dot = 0;

                }else{
                        fprintfd(stderr, ".");
                        dot = 1;
                }
		flushit();
       }
	/*
         * Now for abs time...
	 * make sure we awake reasonably close to the time requested and
	 * never awake earlier than requested
	 */
	for (i = 0; i < loops; i++) {
                double diff;
                int dot = 0;
		Try(clock_gettime(clock, &ts));
                ts.tv_sec+=3;
                ts.tv_nsec += 150000;
                roundtores(&ts,1);  // cheap normalization
		Try0(clock_nanosleep(clock, TIMER_ABSTIME, &ts, NULL));

		Try(clock_gettime(clock, &ns));
                //roundtores(&ts,res);

		if ((diff = timerdiff(&ns, &ts)) > EPS || diff < 0){
                        char *ls = long_short(diff);
                        if( dot) fprintfd(stderr, "\n");
                        myerror("slept too ");
                        
			fprintf(stderr, SETCOLOR_FAILURE
				"%s!\n requested:\t%lds %ldns\n"
				" now:\t\t%lds %ldns\n"	
                                " diff is %12.9fsec\n" SETCOLOR_NORMAL, ls,
				ts.tv_sec, ts.tv_nsec, 
                                ns.tv_sec, ns.tv_nsec, diff);
                        dot = 0;

                }else{
                        fprintfd(stderr, ".");
                        dot = 1;
                }
		flushit();
        }
        /*
         * Now for some more interesting tests.  First, what happens
         * if we interrupt nanosleep.  To do this we fork so we have a
         * child that can cry... uh interrupt us at importune times.
         */
        fprintfd(stderr, "\nTesting signal behavor...\n");
        wait_flush();
        if(!fork()){
                /* The child... */
                
                ts.tv_sec = 0;
                ts.tv_nsec = 10000000;
               
 		Try0(clock_nanosleep(clock, 0, &ts, NULL));
                Try(kill(getppid(),SIGNAL1));
                usleep(5000);
                wait_send(getppid());
                exit(0);
        }
        /* The parent ... */
        Try(clock_gettime(clock, &bs));
	test_normal(&bs);
        ts.tv_sec = 1;
        ts.tv_nsec = 0;
        try0(EINTR,clock_nanosleep(clock, 0, &ts, &rs));
        Try(clock_gettime(clock, &ns));
	test_normal(&rs);
	test_normal(&ns);
        fprintfb(stderr,"Time remaining is %lds %ldns\n",rs.tv_sec,rs.tv_nsec);
 
        timersum(&bs,&bs,&ts);// expected wake up time
        //roundtores(&bs,res);
        timersum(&ns,&ns,&rs);// wake up time + remaining time
        fprintfb(stderr,"Waiting for sig2...\n");
	flushit();
       
        if ((diff = timerdiff(&ns, &bs)) > EPS || diff < 0){
                char *ls = long_short(diff);
                myerror("slept too");
                fprintf(stderr, SETCOLOR_FAILURE 
                        "%s!\n requested:\t%lds %ldns\n"
                        " now:\t\t%lds %ldns\n"	
                        " diff is %12.9fsec\n" SETCOLOR_NORMAL, ls,
                        bs.tv_sec, bs.tv_nsec, 
                        ns.tv_sec, ns.tv_nsec, diff);
		flushit();
        }
        waitpid(child, &i, 0);
       /*
         * For this one we do a signal that is not delivered to the
         * task, and make sure that nanosleep keeps running but not 
	 * for too long.
         */
        fprintfd(stderr, "\nTesting undelivered signal behavor...\n");
        wait_flush();
        if(!(child = fork())){
                /* The child... */
                Try(clock_gettime(clock, &bs));
                ts.tv_sec = 1;
                ts.tv_nsec = 0;
                if(!try0(0,clock_nanosleep(clock, 0, &ts, &rs)))
                        rs.tv_sec = rs.tv_nsec = 0;
                Try(clock_gettime(clock, &ns));
		if( rs.tv_sec | rs.tv_nsec){
			myerror("Sleep time remains, should be zero. ");
				fprintf(stderr,"Time remaining is %lds %ldns\n",
					rs.tv_sec,rs.tv_nsec);
		}
 
                timersum(&bs,&bs,&ts);// expected wake up time
                //roundtores(&bs,res);
                timersum(&ns,&ns,&rs);// wake up time + remaining time


                if ((diff = timerdiff(&ns, &bs)) > EPS || diff < 0){
                        char *ls = long_short(diff);
                        myerror("slept too");
                        fprintf(stderr, SETCOLOR_FAILURE 
                                "%s!\n requested:\t%lds %ldns\n"
                                " now:\t\t%lds %ldns\n"	
                                " diff is %12.9fsec\n" SETCOLOR_NORMAL, ls,
                                bs.tv_sec, bs.tv_nsec, 
                                ns.tv_sec, ns.tv_nsec, diff);

                }
		fprintfb(stderr,"%12.9fsec\n" ,diff);
		flushit();
                wait_send(getppid());
                exit(0);
                
        }else {
        /* The parent ... */
                ts.tv_sec = 0;
                ts.tv_nsec = 400000000;
                Try0(clock_nanosleep(clock, 0, &ts, NULL));
                kill(child,SIGSTOP);
                Try0(clock_nanosleep(clock, 0, &ts, NULL));
                kill(child,SIGCONT);
        }
        wait_sync();
        waitpid(child, &i, 0);
        /*
         * Now we get really dirty.  The spec says we should be able to
         * reset the clock while clock_nanosleep(absolute) is at work and
         * still wake up at the correct time.  We need to be su to do 
         * this, but then that will just fall out as an error.
         * We will let the child do the clock setting...
         */
        if( clock == CLOCK_MONOTONIC || clock == CLOCK_MONOTONIC_HR){
                /* 
                 * can not do this test if we can not set the clock...
                 */
                return 0;
        }
#define CHILD_DELAY 10000000;

        fprintfd(stderr, "\nTesting behavor with clock seting...\n");
        for (i = -2; i < 8; i+=8){
                int exv;
                char * what = i > 0 ? "Advancing" :"Retarding";
                wait_flush();
                fprintft(stderr,"%s the clock %d seconds\n",what, i >0 ? i:-i); 
 		flushit();
		if(!(child = fork())){
                        /* The child... */
                
                        ts.tv_sec = 0;
                        ts.tv_nsec = CHILD_DELAY;
                        Try0(clock_nanosleep(clock, 0, &ts, NULL));
                        /* Advance/ retard the clock */
                        Try(clock_gettime(clock,&bs));
                        ts = bs;
                        ts.tv_sec += i;
                        if(clock_settime(clock,&ts) <0){
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
                                if((diff < 0) || (diff > EPS)){
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
                        Try(clock_gettime(clock,&ts));
                        ts.tv_sec -= i;
                        Try(clock_settime(clock,&ts));                   
                        exit(0);
                }
                /* The parent ... */
                Try(clock_gettime(clock, &ts));
                rs.tv_sec = 0;
                rs.tv_nsec = 500000000;
                timersum(&ts,&ts,&rs);
                try0(0,clock_nanosleep(clock, TIMER_ABSTIME, &ts, NULL));
                Try(clock_gettime(clock, &ns));
                wait_send(child);
                /* 
                 * Correct expected time.  If clock moved back, we will be
                 * late by (back - req.)  if forward, we should be on time.
                 */
                if ( i > 0){
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
                                "Note this may indicate that nano_sleep \n"
                                "did NOT respond to the clock setting\n"
                                SETCOLOR_NORMAL);

                }
                waitpid(child, &exv, 0);
        }

        return 0;
}


#define EPS		0.030000000	/* max error in s */
#define EPS_HR		0.003500000	/* max error in s */
int main(int argc, char *argv[])
{
	struct sigaction sa;
//	sigset_t set;

	init_test(argc, argv);
	shortf = (argc > 2) ? atoi(argv[2]) : shortf;
	sa.sa_flags = SA_SIGINFO;
	sa.sa_sigaction = handler1;
	Try(sigaction(SIGNAL1, &sa, NULL));
	sa.sa_sigaction = handler2;
	Try(sigaction(SIGNAL2, &sa, NULL));

        fprintfd(stderr,"\nTesting clock_nanosleep(CLOCK_REALTIME...\n");
        test(CLOCK_REALTIME,EPS);
	IF_HIGH_RES {
		fprintfd(stderr,
			"\nTesting clock_nanosleep(CLOCK_REALTIME_HR...\n");
		test(CLOCK_REALTIME_HR,EPS_HR);
	}
        fprintfd(stderr,"\nTesting clock_nanosleep(CLOCK_MONOTONIC...\n");
        test(CLOCK_MONOTONIC,EPS);
	IF_HIGH_RES {
		fprintfd(stderr,
			"\nTesting clock_nanosleep(CLOCK_MONOTONIC_HR...\n");
		test(CLOCK_MONOTONIC_HR,EPS_HR);
	}
	by_now();
 }
