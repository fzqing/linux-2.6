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

/* clock_gettimetest3.c
 *
 * This program simply calls clock_gettime(CLOCK_REALTIME), 
 * followed by clock_gettime(CLOCK_MONOTONIC) and figures the difference
 * between them.  It then pauses for a few seconds and does it again.
 * It then prints the difference. 
 */

#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#ifdef __linux__
#include <posix_time.h>
#endif
#include "utils.h"

#define MAX_SAMPLES 25
#define few_seconds 1

double skew(void)
{
	struct timespec tr[MAX_SAMPLES], tm[MAX_SAMPLES];
        int i;
        double result;

        for (i = 0; i < MAX_SAMPLES; i++) {
                Try(clock_gettime(CLOCK_MONOTONIC,&tm[i]));
                Try(clock_gettime(CLOCK_REALTIME,&tr[i]));
                result += timerdiff(&tr[i],&tm[i]);
        }
        return result / MAX_SAMPLES;
}
                

int main() {
        double first;
        int i;

        printf("Clock skew test\n");
        for ( i = 0; i < 30; i++){
                first = skew();
                printf("skew %f \n",first);
                sleep(few_seconds);
        }
	by_now();
}
