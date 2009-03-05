/*
 *   Copyright (C) 2002  MontaVista Software
 *                       George Anzinger (george@mvista.com)
 *   Modified 2003-Jan+Feb to be used as a timer stress test by
 *   			Randy Dunlap (rddunlap@osdl.org)
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

#ifdef comments
/*
The performance measurements are:

1.) time to activate a timer with 0 to N timers active in the system
    (plot time expecting it to have some minor slope) where N is 3000
    timers.

1a) Do same timer measurement with the timer list set to 512, 1024,
    2048, and 4192

2.) Measure the preemption operations with timers expiring at random
    intervals.

3.) Measure the preemption operations with small numbers of timers to
    large numbers of timers expiring at the same time.

Lets see.  There are two ways of doing this. 

1.) We measure the time to activate timer N as we do each timer from 1
    to N.  We do this several times take the average, min, max for each
    N.

2.) For each N we loop for M times activating and deactivating a timer,
    again measuring the min, max, average.

3.) (I lied about the number of ways :) We do a combo of the above.

The data we gather here is min, max, average for each of N timers. 

So much for test 1.  Test 2 requires some method of measuring the actual
    preemption times.  Suppose we create a thread that does the
    measurement.  By using a thread, we can keep track of the number of
    timers that expire during each period.  The thread could, each time
    period, get the preemption info from /proc and put the two together
    in a data structure/file for subsequent display.

Test 3 is a version of Test 2 with out the random expire times.

An additional test might be to do tests 2 & 3 with both high res and low
    res clocks.  For that matter, test 1 should do this also.  Should we
    also consider MONOTONIC as well as REALTIME clocks?

Test 'stress' (test 9) is much like test 1 and is based on code from
do_test1().  This test creates the specified number of the specified
types of timers and runs them for the specified number of minutes.
This is meant to be useful for kernel profiling of the high-res-timers
code.

Ok, so, then what are the variables we want to work with and what is the
    output:

  Test 1,1a
    Number of timers
    Number of times to loop thru the N timers
    Number of times to activate each timer each time thru the loop
    CLOCK to use

    Produces a file of trupples (min, max, average) for each N.

  Test 2,3
    Bound of the expire time (i.e. how long will the test run)
    Number of times to run the above (again keeping min, max, average)
    Interval over which to measure (e.g. for each 10ms)
    CLOCK to use

    Produces a file of trupples (min, max, average) for each N expires 

  Test 9 (stress)
    Running time (minutes)
    Number of timers (low-res)   \
    Number of timers (high-res)   \ These are currently split over <no_timers>.
    Number of timers (realtime)   /
    Number of timers (monotonic) /
    Min and max timer expiration times
    Priority and scheduling type as in other tests here.

    Record timer expiration requested & actual times.
*/
#endif

#include <sys/mman.h>
#include <string.h>
#include "../lib/posix_time.h"
#include "utils.h"

#define FALSE 0
#define TRUE 1
static char *VERSION = "1.0.0 <20030320.0037.07>";
static char *program_name = NULL;
extern void print_usage(FILE*, int);
static int verbose = FALSE;
struct timeval start, stop;
char * ploting;
#ifdef debug
#undef debug
#define debug(a) do {a}while (0)
#else
#define debug(a) do {} while (0)
#endif
#define TEST_STRESS	9
#define TIMER_RELATIVE	0

/*
 * A function to caculate the new running average given the old (av),
 * the new (new) and the new item count (count).
 * av and new should be "double", while count will usually be int.
 */
#define run_av(av, new, count) (((av *(count - 1)) + new)/count)

/*
 * Ever notice that you can do #foo to get a string but there is 
 * no way to get a character constant (i.e. 'a') in a macro.  You either
 * have it or you don't :(
 *
 * Ok, here is where we define the run time options.  This one definition
 * is used three times, the usage print out, the short options init and
 * the long options init.  The parameters are:
 * the short options character constant, the same char as is, i.e.
 * constant, the long version, NIL if no value, PAR if a value should
 * follow, a user friendly info string for the usage print out.
 */
#define OPTIONS \
OPTION_H('h',h, help,        NIL, Display this usage information (try -vh).\n) \
OPTION_H('v',v, verbose,     NIL, print verbose data.\n) \
OPTION_H('n',n, timers,      PAR, Number of timers to work with (3000).\n) \
OPTION_H('l',l, loop,        PAR, Number of times to loop over all timers (5).\n) \
OPTION_H('i',i, iterate,     PAR, Number of times to do each timer (5).\n) \
OPTION_H('r',r, range,       PAR, Range of timer values in milliseconds (no. tmrs *20ms).\n) \
OPTION_H('m',m, tim-min,     PAR, Minimum timer value to use in milliseconds (10,000).\n) \
OPTION_H('a',a, abs,         NIL, Use absolute timers.\n) \
OPTION_H('c',c, clear,       NIL, Timers will be separately cleared prio to arming.\n) \
OPTION_H('P',P, priority,    PAR, Use priority X on the test (50).\n) \
OPTION_H('F',F, fifo,        NIL, Use SCHED_FIFO algorithm.\n) \
OPTION_H('R',R, rr,          NIL, Use SCHED_RR algorithm (default = SCHED_FIFO).\n) \
OPTION_H('L',L, low,         NIL, Use low resolution clock (high res).\n) \
OPTION_H('M',M, mono,        NIL, Use CLOCK_MONOTONIC (CLOCK_REALTIME).\n) \
OPTION_H('g',g, gnu_plot,    NIL, Produce gnu_plot file on stdout.\n) \
OPTION_H('s',s, stress,      NIL, Do test: stress (see -vh).\n)  \
OPTION_H('t',t, test2,       NIL, Do test 2 (see -vh).\n)  \
OPTION_H('T',T, runtime,     PAR, Runtime in seconds for stress test (60).\n)  \
OPTION_H('V',V, version,     NIL, Print version of this program.\n) \

char *verbose_help = 
   "\n"
   "Test 1 (use -t to get test 2, for which see comments below)\n\n"
   "This program collects data on timer arm time against number of active\n"
   "timers.  It also keeps track of timer completions while it is\n"
   "measuring the arming times.  It provides its output in a form\n"
   "compatible with gnuplot.  Several runs through the timers are made with\n"
   "each run doing several timer arms.  The minimum, maximum and average\n"
   "times are kept for each timer count.  The data file contains space\n"
   "delimited data in the following order: count, average, min, max,\n"
   "completions.  The completions column is the number of timer\n"
   "completions while measuring the arm times for that timer.  This\n"
   "column is absent if the count is zero.  All times are in microseconds.\n"
   "A suggest gnuplot directive is:\n"
   "'plot <file> w lines, plot <file> using 1:5'.\n"
   "This will put the timer completions on the plot as points only when\n" 
   "there are some.  You could also use:\n"
   "'plot <file> w error, plot <file> using 1:5'.\n"
   "to plot each value as an error bar.\n"
   "\n"
   "Comments on the parameters: Each of the <timers> is armed <iterate>\n"
   "times on each of <loop> loops, thus there are <iterate> * <loop>\n"
   "times.  \n"
   "\n"
   "The timer must be disarmed between each of the <loop> passes, but\n"
   "may be left armed for the <iterate> loop.  You can control this with\n"
   "the <clear> parameter.  \n"
   "\n"
   "The time to use for each timer is generated using a random number\n"
   "generator constrained to give times from <tim-min> to <tim-min> +\n"
   "<range>.  The default value of <tim-min> precludes timers from\n"
   "completing during the test, however, this need not be the case.  \n"
   "\n"
   "By constraining <range> you can defeat the timer hash, causing all\n"
   "timers to be inserted in the same hash bucket. (Hash buckets are\n"
   "1/HZ seconds in size.)  Since the test take well over 1/HZ seconds,\n"
   "you will need to use the <abs> option to actually force all timers\n"
   "to the same bucket.  Without the <abs> option you should notice a\n"
   "sawtooth graph with each 1/HZ (in test time, which is not kept)\n"
   "representing a tooth, due to the relative alarm times.\n"
   "\n"
   "It is recommended that this test be run as a real time task to avoid\n"
   "its being preempted by other system activity.  See <priority>,\n"
   "<fifo> and <rr>.  If none of these are given, the task will inherit\n"
   "its parents scheduling attributes.\n"
   "\n"
   "You can also choose the POSIX clock and resolution.  Default is\n"
   "CLOCK_REALTIME_HR.  <low> change removes the '_HR' and <mono>\n"
   "changes 'REALTIME' to 'MONOTONIC'.\n\n"
   "The -g option will cause gnuplot commands to be emitted in front of \n"
   "the data such that the output can be piped to gnuplot.  Without the \n"
   "-g these commands will be replaced with comments such that you can \n"
   "enter your own commands to gnuplot and \"plot\" the output.\n\n"
   "Test 2 is a generates a plot of preemption time against number of timers\n"
   "completing on the same tick.  The preemption time is gathered from\n"
   "the preemption \"stats\" patch.  This test is slow as it needs to wait\n"
   "for the timers to complete.  For this test N timers are armed in ABSOLUTE\n"
   "mode for N running from 0 to <-n> timers.  The run will be done <-l> \n"
   "times and the results averaged to give the same format as the test 1\n"
   "file except for the completion info.  For this test the \n"
   "-i, -r, -a, -c, and -m options are meaningless.\n"
   "\n"
   "Specifying option -s runs a stress test that can be used with\n"
   "kernel profiling to see which kernel clock/timer functions are\n"
   "using lots of processor time.\n"

;

#define OPTION_U USAGE
#include "utils.h"
void print_usage(FILE* stream, int exit_code)
{
        fprintf (stream,"%s version %s\n", program_name, VERSION);
        if(verbose){
                fprintf(stream,"%s",verbose_help);
        }
        fprintf (stream, "Usage:  %s options\n", program_name);
        fprintf (stream, OPTIONS);
        exit (exit_code);
}


int clock_index = 0;
#define MONOTONIC 2
#define LOW_RES 1
/* 
 * We know about the following clocks:
 */
clock_t clocks[] = {CLOCK_REALTIME_HR, 
                    CLOCK_REALTIME, 
                    CLOCK_MONOTONIC_HR,
                    CLOCK_MONOTONIC};
/*
 * Which have these names:
 */
char * clock_names[] = {"CLOCK_REALTIME_HR", 
                        "CLOCK_REALTIME", 
                        "CLOCK_MONOTONIC_HR",
                        "CLOCK_MONOTONIC"};
int no_timers = 3000;
int run_time = 60;
int main_loop = 5;
int iterate_loop = 5;
int clear_flag = 0;
int high_bound = 0;
int mypriority = 0;
int expirations = 0;
int line = 90;  // take down by 5 for each line in the graph
#define new_line() line -= 5
int test_which = 1;	// default to test 1
int expire = 0;
int gnu_plot = 0;
int sched_policy = SCHED_OTHER;
double timer_range = 0.0;
double min_timer = 10000.0;
int absf = 0;
struct timespec time_res;
#define SIGNAL1 SIGRTMIN //SIGUNUSED
struct timespec tm, ts, rs, tb ;
struct itimerspec tt ;

struct itimerspec * rtime(struct itimerspec * t)
{
        double tim = (((double)random() *  timer_range) / RAND_MAX) + min_timer;

        t->it_value.tv_sec =tim;
        t->it_value.tv_nsec =  ((tim - t->it_value.tv_sec) * NSEC_PER_SEC);
        if( absf ) timersum(&t->it_value,&t->it_value,&tb);
        return t;
}

struct itimerspec * timer_random(int tx, struct itimerspec * t)
{
	// TBD: add absolute timers as well; currently only uses relative timers.
        ///double tim = (((double)random() * timer_range) / RAND_MAX) + min_timer;
        double tim = (((double)random() / (double)RAND_MAX) * timer_range) + min_timer;
	static int timer_cut = 0;

	if (verbose)
		printf ("timer_range: %f, min_timer: %f, tim: %f\n",
			timer_range, min_timer, tim);
	if (timer_cut) {
		tim /= 2.0;	// reduce every other one in half.
		timer_cut = 0;
	}
	else
		timer_cut = 1;
        t->it_value.tv_sec = tim;
        t->it_value.tv_nsec = ((tim - t->it_value.tv_sec) * NSEC_PER_SEC);
        t->it_interval.tv_sec = tim;
        t->it_interval.tv_nsec = t->it_value.tv_nsec;
        ///if (absf)
		///timersum(&t->it_value, &t->it_value, &tb);
	if (verbose)
		printf ("timer %d start in %ld.%09ld, interval is %ld.%09ld\n",
			tx, t->it_value.tv_sec, t->it_value.tv_nsec,
			t->it_interval.tv_sec, t->it_interval.tv_nsec);
        return t;
}

#define IF_V(a) do {if( verbose ) { a };}while(0)
#define start() expirations = 0;Try(clock_gettime(clocke, &ts))
#define elapsed() ({Try(clock_gettime(clocke, &rs));\
                     expire = expirations; \
                     timerdiff(&rs,&ts);\
                   })
#define for_each(a) for (t = 0; t < no_timers; t++){a;}
#define for_main(a) for (o = 0; o < main_loop; o++){ a; }
#define for_iter(a) for (i = 0; i < iterate_loop; i++){ a; } 
#define for_all(a)  for_main( for_each (for_iter( a) ))

#define timerx() timer[t].result[o].in_loop[i]

int rotate_ix = 0;
char rotate_chars[4] = "|/-\\";

void timer1_handler(int signo, siginfo_t * info, void * ptr)
{
        IF_V(printf("In handler\n"););
	expirations++;
}

void rotator (void)
{
	putchar (rotate_chars [rotate_ix++]);
	fflush (stdout);
	if (rotate_ix >= sizeof rotate_chars)
		rotate_ix = 0;
}

struct timeval stress_begin;

void timer9_handler(int signo, siginfo_t * info, void * ptr)
{
	int over = timer_getoverrun (info->si_tid);
	long usecs;
	struct timeval time_now;

	if (info->si_code != SI_TIMER || signo != info->si_signo)
		printf ("err:  bad si_code (%d) or signo (%d vs. %d)\n",
				info->si_code, signo, info->si_signo);
	Try (gettimeofday(&time_now, 0));
	usecs = timevaldiff(&time_now, &stress_begin);
	if (verbose) {
		printf ("RTSIG: timer_id=0x%x, timer_tag=0x%x, over=%d, elapsed (secs) = %ld.%06ld\n",
				///signo, info->si_signo, info->si_code,
				///info->_sifields._timer._timer1,
				///info->_sifields._timer._timer2);
				info->si_tid, info->si_value.sival_int, over,
				usecs / USEC_PER_SEC, usecs % USEC_PER_SEC);
	}
	else
		rotator();
	expirations++;
}

void do_test1(void)
{
        struct {
                struct result {
                        double in_loop[iterate_loop];
                } result[main_loop];
        } timer[no_timers];
        timer_t  timer_id[no_timers];
        int expires[no_timers];
        int i,o,t;
        clock_t clockt = clocks[clock_index];
        clock_t clocke = CLOCK_MONOTONIC;
        struct sigevent  sigev;
        sigset_t new_mask,old_mask;
        struct sigaction oldact,sigact;
        struct itimerspec tc = {{0,0},{0,0}};
        
        /* 
         * For this test we don't want any signals, so
         * just ignore them.
         */
        sigact.sa_sigaction = timer1_handler;
        sigact.sa_flags = SA_SIGINFO;
        Try(sigemptyset(&sigact.sa_mask));

	sigaddset(&new_mask, SIGNAL1);
	Try(sigprocmask(SIG_UNBLOCK, &new_mask, &old_mask));
        /*
         * Set up a random number generator
         */
        start();
        srandom(ts.tv_sec);
        Try(clock_gettime(clockt, &tb));
 
        Try(sigaction(SIGNAL1,&sigact,&oldact));
        sigev.sigev_notify = SIGEV_SIGNAL;
        sigev.sigev_signo = SIGNAL1;
        start();
        for_all( timerx() = 0.0);
        IF_V(printf(" Result array (%d bytes) cleared in %12.9f sec \n",
                    main_loop*iterate_loop*no_timers*(sizeof(double)),
                                                      elapsed()););

        for_each (
                Try(timer_create(clockt, &sigev, &timer_id[t]));
                expires[t] = 0;
                );
        for_main( 
                for_each( 
                        for_iter(
                                if( clear_flag ){ 
                                        Try(timer_settime(timer_id[t], 
                                                          0, &tc, NULL));
                                }
                                start();
                                Try(timer_settime(timer_id[t], 
                                                  absf, rtime(&tt), NULL));
                                timerx() = elapsed();
                                expires[t] = expire;
                                );
                        );
                for_each(
                        Try(timer_settime(timer_id[t], 0, &tc, NULL));
                        );
                );
     
        /*
         * Ok, the test is done, clean up the mess and push out the data.
         */
        for_each (Try(timer_delete(timer_id[t])););
        /*
         * don't forget to restore the signal system
         */
        Try(sigaction(SIGNAL1,&oldact,NULL));
        /*
         * Here is where we write out the data.  All
         * the numbers are floating point.
         */
        printf("%splot \"-\" w lines \n", gnu_plot ? "" : "#");

        for_each(
                double high = 0.0;
                double low  = 1000;
                double sum  = 0.0;
                double scale = USEC_PER_SEC;  // values in microseconds
                int ppt = iterate_loop + main_loop;
                for_main(
                        for_iter(
                                double v = timerx();
                                
                                sum += v;
                                if( v > high) high = v;
                                if( v < low)  low  = v;
                                );
                        );
                if(expires[t]) {
                        printf( "%d %12.9f %12.9f %12.9f %d\n",
                                t,
                                sum * scale/ ppt,
                                low * scale,
                                high * scale,
                                expires[t]);
                }else{
                        printf( "%d %12.9f %12.9f %12.9f\n",
                                t,
                                sum * scale/ ppt,
                                low * scale,
                                high * scale);
                }
                );

        return;
}

/*
 * Test 2 is to generate data for a graph of preemption time against
 * timer completions.  We set up N timers to complete at the same
 * time and then wait for that time.  We then read the preemption
 * stats in /proc and see how big a hit we took.  For this test we 
 * will not allow much in the way of control as a timer competion 
 * is a timer completion.  We will try and fine tune the set up time
 * to allow us to get all the timers armed prior to the expire time.
 * This will take longer as N gets larger.  We will test what this
 * is for N=max and assume it is linear in N (if it is not we will be
 * erroring on the right side).  We will do the whole test M times
 * and produce a gnu_plot error file (i.e. N ave min max).
 */
#undef timerx
#undef for_all
//#define for_each(a) for (t = 0; t < no_timers; t++){a;}
//#define for_main(a) for (o = 0; o < main_loop; o++){ a; }
#define for_all(a)  for_main( for_each(  a ))
#define for_timers(a) for (l = 0; l <= t; l++){a;}

#define timerx() timer[t].result[o]
#define wait_t() timer[t].wait_time[o]

struct itimerspec * etime(struct itimerspec * t, double tim)
{
        t->it_value.tv_sec =tim;
        t->it_value.tv_nsec =  ((tim - t->it_value.tv_sec) * NSEC_PER_SEC);
        roundtores(&t->it_value,10000000);
        timersum(&t->it_value,&t->it_value,&ts);
        return t;
}
#define  wait_for_expire(new_mask) sigwaitinfo(&new_mask, &info);

#define FILEN "/proc/latencytimes"
#define BUF_SIZE 4048

long get_proc_latency(int count)
{
        char buff[BUF_SIZE];
        int rd;
        long n, rtn;
        char * loc, * end;
        int fd = Try(open(FILEN, O_RDONLY));

        rd = Try(read(fd, &buff[0], BUF_SIZE));
        if (rd >= BUF_SIZE){
                printf("%s buffer size too small (%d)\n", FILEN, BUF_SIZE);
        }
        n = Try(read(fd, &n, sizeof(n)));
        if ( n ) {
                printf("Failed to find expected EOF on %s\n", FILEN);
        }
        close(fd);
        buff[rd] = 0;
        /*
         * This is a dirty little bit of code to get the worst case 
         * latency from the above buffer.  It will be the number just
         * after the string "end line/file".  There will be a new line
         * here AND, if SMP there may be more than one of them.  Find
         * them all and return the biggest.  Units will be microseconds.
         */

        rtn = 0;
        end = buff + rd;
        loc = buff;
        while (loc < end){
                char mstring[] = "end line/file";
                loc = strstr(loc, mstring);
                if( ! loc )
                        break;
                loc += sizeof(mstring);
                n = strtol(loc, &loc, 10);
                if ( n > rtn) rtn = n;
        }
        if( count > 100 && count > rtn + rtn) {
                printf("Found %d with count %d from:\n%s", (int)rtn, count, buff);
        }
        
        return rtn;
}

void do_test2(void)
{
        struct result {
                double result[main_loop];
                double wait_time[main_loop];
        } timer[no_timers];
        timer_t  timer_id[no_timers];
//       int expires[no_timers];
        int l,o,t;
        clock_t clockt = clocks[clock_index];
        clock_t clocke = CLOCK_MONOTONIC;
        double max_setup_time, setup_per;
        struct sigevent  sigev;
        //sigset_t new_mask,old_mask;
        struct sigaction oldact,sigact;
        struct itimerspec tc = {{0,0},{0,0}};
        struct itimerspec rm;
        struct timespec tm = {0,20000000}, tz  = {0,0};
        WAIT_VARS();

        /* 
         * For this test we will do a sig wait so first block them.
         */
        sigact.sa_sigaction = timer1_handler;
        sigact.sa_flags = SA_SIGINFO;

        Try(clock_gettime(clockt, &tb));
 
        Try(sigaction(SIGNAL1,&sigact,&oldact));
        wait_setup( SIGNAL1);
        sigev.sigev_notify = SIGEV_SIGNAL;
        sigev.sigev_signo = SIGNAL1;
        start();
        for_all( timerx() = 0.0);
        IF_V(printf(" Result array (%d bytes) cleared in %12.9f sec \n",
                    main_loop*iterate_loop*no_timers*(sizeof(double)),
                                                      elapsed()););

        for_each (
                Try(timer_create(clocke, &sigev, &timer_id[t]));
//               expires[t] = 0;
                );
        start();
        for_each(
                
                Try(timer_settime(timer_id[t], TIMER_ABSTIME, etime(&tt,60.0), 
                                  NULL));
                );
        max_setup_time = elapsed();
        setup_per = max_setup_time / no_timers;
        IF_V(
                printf("# Set up time for %d timers %12.9f seconds or \n"
                       "%12.9f seconds per timer.\n", 
                       no_timers, max_setup_time, max_setup_time / no_timers););
        //setup_per *= 2.0; // use double to be safe

        // clear all the timers
        for_each(
                Try(timer_settime(timer_id[t], 0, &tc, NULL));
                ); 
        start();
        get_proc_latency(0); // Clear the latency counters
        IF_V(
                printf("Get latency takes %12.9f seconds.\n",elapsed());
                );

        for_main(
                for_each(
                        start();
                        get_proc_latency(0); // Clear the latency counters
                        etime(&tt, (setup_per * t) + 0.01); 
                        
                        for_timers(
                                Try(timer_settime(
                                            timer_id[l], TIMER_ABSTIME, 
                                            &tt, 
                                            NULL));
                                );
                        IF_V( {
                                struct itimerspec tl;
                                Try(timer_gettime(timer_id[0],&tl));
                                if(timerdiff(&tl.it_value,&tc.it_value) == 0.0){
                                        printf("Timers already expired!"
                                               "  Consider runing at "
                                               "higher priority.\n" );
                                }
                                printf(".");fflush(stdout);
                        });
                        //get_proc_latency(); // Clear the latency counters
                        wait_sync();  // wait for first signal
                        clock_nanosleep(clocke, 0, &tm, NULL);
                        for_timers(      // clear the timers
                                Try(timer_settime(timer_id[l], TIMER_ABSTIME, 
                                                  &tc, 
                                                  &rm));
                                if (rm.it_value.tv_sec + rm.it_value.tv_nsec){
                                        printf("Oops!  Time remains on "
                                               "%d %12.9f secs\n", 
                                               l, timerdiff(&rm.it_value, &tz));
                                }
                                );
                        timerx() = get_proc_latency(t);
                        wait_flush();  // flush any left over signals
                        );
                );
        /*
         * Ok, the test is done, clean up the mess and push out the data.
         */
        for_each (Try(timer_delete(timer_id[t])););
        /*
         * Here is where we write out the data.  All
         * the numbers are floating point.
         */
        printf("%splot \"-\" w lines \n", gnu_plot ? "" : "#");
        for_each(
                double high = 0.0;
                double low  = 1000;
                double sum  = 0.0;
                double scale = 1.0;
                double ppt =  main_loop;
                for_main(
                        double v = timerx();
                        sum += v;
                        if( v > high) high = v;
                        if( v < low)  low  = v;
                        /*  wsum += w;
                        if( w > whigh) whigh = w;
                        if( w < wlow)  wlow  = w; */
                        );/*%12.9f %12.9f %12.9f*/
                printf( "%d %12.9f %12.9f %12.9f  \n",
                        t,
                        sum * scale/ ppt,
                        low * scale,
                        high * scale
                        );
                );

        return;
}

void do_resolutions(void)
{
	int err;
	int ix;

	for (ix = 0; ix < ARRAY_SIZE(clocks); ix++) {
		err = clock_getres(clocks[ix], &time_res);
		if (err)
			perror("clock_getres");
		else
			printf("resolution for '%s' = %ld.%09ld\n",
				clock_names[ix],
				time_res.tv_sec, time_res.tv_nsec);
	}
}

void do_current_clocks(void)
{
	struct timespec ts;

	Try (clock_gettime(CLOCK_REALTIME, &ts));
	printf ("CLOCK_REALTIME = %ld.%09ld\n", ts.tv_sec, ts.tv_nsec);
	Try (clock_gettime(CLOCK_REALTIME_HR, &ts));
	printf ("CLOCK_REALTIME_HR = %ld.%09ld\n", ts.tv_sec, ts.tv_nsec);
	Try (clock_gettime(CLOCK_MONOTONIC, &ts));
	printf ("CLOCK_MONOTONIC = %ld.%09ld\n", ts.tv_sec, ts.tv_nsec);
	Try (clock_gettime(CLOCK_MONOTONIC_HR, &ts));
	printf ("CLOCK_MONOTONIC_HR = %ld.%09ld\n", ts.tv_sec, ts.tv_nsec);
#if 0
	Try (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts));
	printf ("CLOCK_PROCESS_CPUTIME_ID = %ld.%09ld\n", ts.tv_sec, ts.tv_nsec);
	Try (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts));
	printf ("CLOCK_THREAD_CPUTIME_ID = %ld.%09ld\n", ts.tv_sec, ts.tv_nsec);
#endif
}

// usage:  timerstress -s -T 60 -n 1000 [ -m M -r N ] [-v]
void do_stress(void)
{
	int results_size = no_timers * 10;
        struct result {		// collect non-0 timer overruns
		timer_t	tmrid;
                double	overrun;
        } results [results_size];	// was: timer[]
	int results_count = 0, err_count = 0;
        timer_t  timer_id[no_timers];
        int expires[no_timers];
        int t, ix;
        clock_t clockt = clocks[clock_index];
        clock_t clocke = CLOCK_MONOTONIC;
        struct sigevent  sigev;
        sigset_t new_mask,old_mask;
        struct sigaction oldact,sigact;
        struct itimerspec tc = {{0,0},{0,0}};
	struct timeval time_now;
	long usecs;

	min_timer /= 1000;		// now in seconds
	if (high_bound)			// seconds
		timer_range = high_bound;
	else
		timer_range = 60.0;
        /* 
         * For this test we use one real-time signal.
         */
        sigact.sa_sigaction = timer9_handler;
        sigact.sa_flags = SA_SIGINFO;
        Try (sigemptyset(&sigact.sa_mask));
	///Try (sigaddset(&sigact.sa_mask, SIGNAL1));

        /*
         * Set up a random number generator
         */
        start();
        srandom(ts.tv_sec);
        Try (clock_gettime(clockt, &tb));

	Try (sigprocmask(SIG_UNBLOCK, &new_mask, &old_mask));
        Try (sigaction(SIGNAL1,&sigact,&oldact));
        sigev.sigev_notify = SIGEV_SIGNAL;
        sigev.sigev_signo = SIGNAL1;
        start();
	memset (results, 0, sizeof(results));

	for (t = 0; t < no_timers; t++) {
		sigev.sigev_value.sival_int = t;
                Try (timer_create(clockt, &sigev, &timer_id[t]));
                expires[t] = 0;
		if (verbose)
			printf ("timer %d ID = 0x%x\n", t, timer_id[t]);
	}
	/* now wait a second before beginning... */
	printf ("sleep 1 second....\n");
	sleep(1);	// second
	Try (gettimeofday(&stress_begin, 0));
	expirations = 0;

	for (t = 0; t < no_timers; t++) {
		timer_random (t, &tc);
		Try (timer_settime (timer_id[t], TIMER_RELATIVE, &tc, NULL));
	}

	for (ix = 0; ; ix++) {
		if (ix >= 1000) {
			rotator();
			ix = 0;
		}
		Try (gettimeofday(&time_now, 0));
		usecs = timevaldiff(&time_now, &stress_begin);
#if 0
		if (verbose)
			printf ("stress test: elapsed time (sec) = %ld.%06ld\n",
				usecs / USEC_PER_SEC, usecs % USEC_PER_SEC);
#endif
		if (usecs >= run_time * USEC_PER_SEC)
			break;
		usleep(1000);	// 1 ms
	}

#if 0
	for_each( 
                results[t].tmrid = timer_id[t]; /////WAS: = elapsed();
                expires[t] = expire;
	);
#endif

        /*
         * Ok, the test is done, clean up the mess and push out the data.
         */
	for (t = 0; t < no_timers; t++)
                Try (timer_settime(timer_id[t], 0, &tc, NULL));

	for (t = 0; t < no_timers; t++){
        	Try (timer_delete(timer_id[t]));
	}

        /*
         * don't forget to restore the signal system
         */
        Try (sigaction(SIGNAL1,&oldact,NULL));
	printf ("\ntotal timer expirations = %d\n", expirations);
}

#define OPTION_U LONG
#include "utils.h"
int main(int argc, char *argv[])
{        
        int next_option;
        pid_t mypid = getpid();
        struct sched_param sched_pr;
        //const char* const short_options = "m:hfvrnpP:b:V";
        const struct option long_options[] = {
                OPTIONS
                { NULL,            0, NULL,  0 }  /* Required at end of array.  */
        };
#define OPTION_U SHORT
#include "utils.h"
        const char* const short_options = OPTIONS;
        program_name = argv[0];

        do {
        	next_option = getopt_long(argc, argv, short_options,
		     long_options, NULL);
                switch (next_option) {
	   	   case 'a':
			absf = TIMER_ABSTIME;
			break;
	   	   case 'c':
			clear_flag = TRUE;
			break;
                   case 'F': 
                        sched_policy = SCHED_FIFO;
                        break;
                   case 'g': 
                        gnu_plot = TRUE;
                        break;
                   case 'h':
                        print_usage (stdout, 0);
			break;
                   case 'i': 
                        iterate_loop = atol(optarg);
                        break;
                   case 'l': 
                        main_loop = atol(optarg);
                        break;
                   case 'L': 
                        clock_index |= LOW_RES;
                        break;
                    case 'm': 
                        min_timer = atol(optarg);
                        break;
                   case 'M': 
                        clock_index |= MONOTONIC;
                        break;
                   case 'n': 
                        no_timers = atol(optarg);
                        break;
                   case 'P': 
                        mypriority = atoi(optarg);
                        break;
                    case 'r': 
                        high_bound = atol(optarg);
                        break;
                   case 'R': 
                        sched_policy = SCHED_RR;
                        break;
                   case 's': 
                        test_which = 9;
                        break;
                   case 't': 
                        test_which = 2;
                        break;
                   case 'T': 
                        run_time = atoi(optarg);
                        break;
	   	   case 'v':
			verbose = TRUE;
			break;
                   case 'V': 
                        printf("%s version %s\n", program_name, VERSION);
                        exit(0);
			break;
                        
                   case -1:  /* Done with options.  */
                        break;

                   default: /* Something else: unexpected.  */
			print_usage(stderr, 1);
                        exit(-1);
                }
        }
        while (next_option != -1);

        /*
        **  Lock all memory pages associated with this process to prevent
        **  delays due to process memory being swapped out to disk and back.
        */
        mlockall( (MCL_CURRENT | MCL_FUTURE) );
	/*
         * Run as SCHED_FIFO (real time) priority 99, since I don't
	 * trust the user to remember.  Decidely unfriendly.  Changed
         * to default to inherit.  Reports what is in all cases.
         */
        if( sched_policy != SCHED_OTHER) {
                if (!mypriority) mypriority = 50;
        }else{
                if (mypriority)  sched_policy = SCHED_FIFO;
        }
	sched_pr.sched_priority = mypriority;
        if(mypriority){
                Try (sched_setscheduler(mypid, sched_policy, &sched_pr)); 
        }
        Try (sched_getparam(mypid, &sched_pr));
        sched_policy = Try(sched_getscheduler(0));

	Try (gettimeofday(&start, 0));
        ploting = gnu_plot ? "set label " : "#";

	if (test_which != TEST_STRESS)
	{
        printf("%s"
               "\"Calculations done at priority %d "
               "using %s scheduling policy.\" at graph 0.1,0.%d\n",
               ploting,
               sched_pr.sched_priority,
               sched_policy == SCHED_OTHER ? "SCHED_OTHER" : 
               sched_policy == SCHED_FIFO ? "SCHED_FIFO" : 
               sched_policy == SCHED_RR ? "SCHED_RR" : "UNKNOWN",
               line
                );
        new_line();
	IF_HIGH_RES {}else{
		if(!(clock_index & LOW_RES)){
			printf("High Res Clocks not available."
			       "  Using low.\n");
			clock_index |= LOW_RES;
		}
	}
        printf("%s\"Using %d timers on clock %s\"at graph 0.1,0.%d\n",
               ploting,
               no_timers,clock_names[clock_index],
               line
               );
        new_line();
        printf("%sset ylabel \"Microseconds\"\n",
               gnu_plot ? "" :"#");
        printf("%sset xlabel \"Number of timers\"\n",
               gnu_plot ? "" :"#");
	}
	else {	// stress test
		printf ("stress test: #timers = %d, run_time = %d seconds\n",
			no_timers, run_time);
        	printf ("Calculations done at priority %d "
			"using %s scheduling policy.\n",
			sched_pr.sched_priority,
			sched_policy == SCHED_OTHER ? "SCHED_OTHER" : 
			sched_policy == SCHED_FIFO ? "SCHED_FIFO" : 
			sched_policy == SCHED_RR ? "SCHED_RR" : "UNKNOWN"
			);

	}

        /*
         * Ok, we have all the info from the user and are now ready to
         * start the tests.  We put the tests in a function so that the 
         * data array can be dynamicaly allocated.  Since we have it
         * locked, it is possible to fail here with a SIGSEGV.
         */
	switch (test_which) {
	case 1:
                printf("%s\"Time to arm vs. number of armed "
                       "timers\" at graph 0.5, .975 center\n", ploting);
                printf("%s\"Outer loop is %d and inner loop %d.\"at "
                       "graph 0.1,0.%d\n",
                       ploting,
                       main_loop,iterate_loop,
                       line
                        );
                new_line();
                if ( high_bound ){
                        timer_range = (double)high_bound / 1000;
                }else{
                        timer_range = .02 * no_timers;
                }
                tm.tv_sec = timer_range;
                tm.tv_nsec =  (timer_range - tm.tv_sec) * NSEC_PER_SEC;
                min_timer /= 1000;
                printf("%s\"Maximum timer range is from %12.9f "
                       "to %12.9f seconds.\" at graph 0.1,0.%d\n",
                       ploting,
                       min_timer, 
                       min_timer + timer_range,
                       line
                      );
                new_line();
		do_test1();
		break;

	case 2:
               printf("%s\"Schedule latency vs. number of "
                      "completing timers\" at graph 0.5, .975 center\n", 
                      ploting);
                printf("%s\"Average of %d completions.\"at "
                       "graph 0.1,0.%d\n",
                       ploting,
                       main_loop,
                       line
                        );
                new_line();
		do_test2();
		break;

	case TEST_STRESS:
		do_resolutions();
		///do_current_clocks();
		do_stress();
		break;
	}

        exit(0);
        return 0;
}
