/* should gives effective times of a nanosleep() */

#include <stdio.h>
#include <sys/time.h>
#include <sys/io.h>
#include <time.h>
#include <posix_time.h>
#include "utils.h"

#define USEC_PER_SEC	1000000
#define NSEC_PER_SEC	1000000000L
#define NSEC_PER_USEC	1000L

//#define timerdiff(a,b) (((double)((a)->tv_sec - (b)->tv_sec) * USEC_PER_SEC) + \
//                         (double)((a)->tv_usec - (b)->tv_usec))
#define better_timerdiff(a,b) ((double)((a)->tv_sec - (b)->tv_sec)*NSEC_PER_SEC + \
                         (double)((a)->tv_nsec - (b)->tv_nsec))

#define CLOCK CLOCK_REALTIME_HR
int main(void)
{
	struct timespec pre_time, post_time;
	struct timespec req, res;
	double diff;
	int k,i;
	static int bla=0;
	
	ioperm(0x378,3,1);
		
	req.tv_sec = 0;
	req.tv_nsec = 10;
	Try(clock_getres(CLOCK, &res));
	printf("Clock resolution %12.3f usec\n", 
	       (double)res.tv_nsec/NSEC_PER_USEC);
	printf("Requested time    difference(usec)\n");
	for (i=0; i<30; i++){
		req.tv_nsec *= 10;
		if (req.tv_nsec >= NSEC_PER_SEC) {
			req.tv_nsec = 100;
			req.tv_sec++;
		}
//		req.tv_sec = 0;
//		req.tv_nsec = 500 * 1000;
//		for (k = 0; k < 20; k++) {
			Try(clock_gettime(CLOCK, &pre_time));
			Try0(clock_nanosleep(CLOCK, 0, &req, NULL));
			Try(clock_gettime(CLOCK, &post_time));
			timersum(&post_time, &post_time, &req);
			if(bla) {
				outb(0x1,0x378);
				bla = 0;
			} else { 
				outb(0x0,0x378);
				bla = 1;
			}
		
			diff = better_timerdiff(&post_time, &pre_time)/NSEC_PER_USEC;
			////diff = timerdiff(&post_time, &pre_time);
			printf("%12.9f\t %12.9f\n", (double)req.tv_sec + ((double)req.tv_nsec/NSEC_PER_SEC), diff);
//		}
	}

	return 0;
}
