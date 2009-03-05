#include <linux/unistd.h>
#include "posix_time.h"
#include <errno.h>
#include <sys/syscall.h>


/*
 * This is nolonger defined by errno.h in RedHat 8.0
 */
#ifndef __set_errno
#define __set_errno(val)       (errno = (val))
#endif

#define __NR___timer_create     __NR_timer_create
#define __NR___timer_gettime    __NR_timer_gettime
#define __NR___timer_settime    __NR_timer_settime
#define __NR___timer_getoverrun __NR_timer_getoverrun
#define __NR___timer_delete     __NR_timer_delete

#define __NR___clock_settime   __NR_clock_settime
#define __NR___clock_gettime   __NR_clock_gettime
#define __NR___clock_getres    __NR_clock_getres
#define __NR___clock_nanosleep __NR_clock_nanosleep

/* MV LOCAL: Use syscall() for portability.  */
extern int
     syscall(int number, ...);

#undef _syscall1
#define _syscall1(type,name,t1,a1)			\
type name (t1 a1)					\
{							\
	return syscall (__NR_##name, a1);		\
}

#undef _syscall2
#define _syscall2(type,name,t1,a1,t2,a2)		\
type name (t1 a1, t2 a2)				\
{							\
	return syscall (__NR_##name, a1, a2);		\
}

#undef _syscall3
#define _syscall3(type,name,t1,a1,t2,a2,t3,a3)		\
type name (t1 a1, t2 a2, t3 a3)				\
{							\
	return syscall (__NR_##name, a1, a2, a3);	\
}

#undef _syscall4
#define _syscall4(type,name,t1,a1,t2,a2,t3,a3,t4,a4)	\
type name (t1 a1, t2 a2, t3 a3, t4 a4)			\
{							\
	return syscall (__NR_##name, a1, a2, a3, a4);	\
}



_syscall3(int, timer_create, clockid_t, which_clock, struct sigevent *,
	  timer_event_spec, timer_t *, created_timer_id)
/* This will expand into the timer_gettime system call stub. */
_syscall2(int, timer_gettime, 
	  timer_t, timer_id, 
	  struct itimerspec *, setting)

/* This will expand into the timer_settime system call stub. */
_syscall4(int, timer_settime, 
	  timer_t, timer_id, 
	  int, flags, 
	  const struct itimerspec *, new_setting,
	  struct itimerspec *, old_setting)

/* This will expand into the timer_gettime system call stub. */
_syscall1(int, timer_getoverrun, 
	  timer_t, timer_id)

/* This will expand into the timer_delete system call stub. */
_syscall1(int, timer_delete, 
	  timer_t, timer_id)

/* This will expand into the clock_getres system call stub. */
_syscall2(int, clock_getres, 
	  clockid_t, which_clock,
          struct timespec *,resolution)

/* This will expand into the clock_gettime system call stub. */
_syscall2(int, clock_gettime, 
	  clockid_t, which_clock,
          struct timespec *,time)

/* This will expand into the clock_settime system call stub. */
_syscall2(int, clock_settime, 
	  clockid_t, which_clock,
          const struct timespec *,time)

/* This will expand into the clock_nanosleep system call stub. */
#define __NR_clock_nanosleep_stub __NR_clock_nanosleep
_syscall4(int, clock_nanosleep_stub, 
	  clockid_t, which_clock,
          int, flags,
          const struct timespec *,rqtp,
          struct timespec *,rmtp)

int clock_nanosleep(clockid_t which_clock,
	int flags,
	const struct timespec *new_setting,
	struct timespec *old_setting)
{
	int res,err = errno;

	res = clock_nanosleep_stub (which_clock, flags, new_setting, old_setting);

	if (res == -1) {
		res = errno;
		errno = err;
		return res;
	}
	return 0;
}


