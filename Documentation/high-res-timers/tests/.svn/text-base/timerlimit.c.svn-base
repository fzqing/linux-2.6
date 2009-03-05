/* max_posix_timers.c

    Compile on Linux with -lrt -lreadline -lncurses
*/
#include <sys/types.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <posix_time.h>

#if 0
#ifndef NO_SYSCALL
#include "pt_syscall.h"
#endif
#endif
int
main(int argc, char *argv[])
{
    int  j, s;
    timer_t timerId;
    struct sigevent sev;

    for (j = 1; ; j++) {
        sev.sigev_notify = SIGEV_SIGNAL;
        sev.sigev_signo = 10;
        sev.sigev_value.sival_int = 0;

        s = timer_create(CLOCK_REALTIME, &sev, &timerId);
        usleep(10000);
        if (s == -1) {
            perror("timer_create"); exit(1);
        } else
            printf("%d: timer id = %ld\n", j, (long) timerId);
    } /* for */
} /* main */

