/*#* rt_sev_none.c 
*/
#include <signal.h>
#include <time.h>

#include <sys/types.h>	
#include <stdio.h>
#include <stdlib.h>	
#include <unistd.h>	
#include <errno.h>	

#ifndef NO_SYSCALL
#include <posix_time.h>
#endif

#define errMsg(msg) 	{ perror(msg); }

#define errExit(msg) 	{ perror(msg); exit(EXIT_FAILURE); }

#define fatalErr(msg) 	{ fprintf(stderr, "%s\n", msg); \
     			  exit(EXIT_FAILURE); }

#define usageErr(msg, progName)	\
			{ fprintf(stderr, "Usage: "); \
			  fprintf(stderr, msg, progName); \
     			  exit(EXIT_FAILURE); }

int
main(int argc, char *argv[])
{
    struct sigevent sev;
    struct itimerspec ts;
    long arg1, arg2, arg3, arg4;
    timer_t tid;
    int j;
    
    if (argc < 2) {
	fprintf(stderr, 
		"Usage: %s secs [nsecs [int-secs [int-nsecs]]]\n", 
		argv[0]);
	exit(EXIT_FAILURE);
    } /* if */

    sev.sigev_notify = SIGEV_NONE;
    if (timer_create(CLOCK_REALTIME, &sev, &tid) == -1) 
	errExit("timer_create");

    arg1 = ts.it_value.tv_sec = atoi(argv[1]);
    arg2 = ts.it_value.tv_nsec = (argc > 2) ? atoi(argv[2]) : 0;
    arg3 = ts.it_interval.tv_sec = (argc > 3) ? atoi(argv[3]) : 0;
    arg4 = ts.it_interval.tv_nsec = (argc > 4) ? atoi(argv[4]) : 0;

    if (timer_settime(tid, 0, &ts, NULL) == -1) errExit("timer_settime");
    
    for (j = 0; ; j++) {
	 usleep(500000);

	 if (timer_gettime(tid, &ts) == -1) errExit("timer_gettime");
	 printf("%d: value=%ld.%09ld(%ld.%09ld); interval=%ld.%09ld(%ld.%09ld)\n", j,
		 (long) ts.it_value.tv_sec, (long) ts.it_value.tv_nsec,
		arg1, arg2,
		 (long) ts.it_interval.tv_sec, 
		(long) ts.it_interval.tv_nsec,
		 arg3, arg4);
    } /* if */

    exit(EXIT_SUCCESS);
} /* main */
