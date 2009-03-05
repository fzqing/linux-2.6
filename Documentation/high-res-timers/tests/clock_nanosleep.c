/* should gives effective times of a nanosleep() */

#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include <posix_time.h>
#include "utils.h"

#define USEC_PER_SEC	1000000
#define NSEC_PER_SEC	1000000000L

//#define timerdiff(a,b) (((double)((a)->tv_sec - (b)->tv_sec) * USEC_PER_SEC) + \
                         (double)((a)->tv_usec - (b)->tv_usec))
#define CLOCK CLOCK_REALTIME_HR
int main(void)
{
	struct timespec pre_time, post_time, zero = {0, 0};
	struct timespec req, res;
	double diff, request;
	int i;
	
	req.tv_sec = 0;
	req.tv_nsec = 10;
	Try(clock_getres(CLOCK, &res));
	printf("Clock resolution %12.3f usec\n", 
	       (double)res.tv_nsec/NSEC_PER_USEC);
	printf("Requested delay    actual delay(sec) error(sec)\n");
	for (i=0; i<30; i++){
		req.tv_nsec *= 10;
		if (req.tv_nsec >= NSEC_PER_SEC) {
			req.tv_nsec = 100;
			req.tv_sec++;
		}
		Try(clock_gettime(CLOCK, &pre_time));
		Try0(clock_nanosleep(CLOCK, 0, &req, NULL));
		Try(clock_gettime(CLOCK, &post_time));
		diff = timerdiff(&post_time, &pre_time);
		request = timerdiff(&req, &zero);
		
		printf("%12.9f\t %12.9f \t %12.9f\n", 
		request, diff, diff-request);
	}

	return 0;
}
