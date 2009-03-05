SOURCES = syscall_timer.c 
OBJECTS = $(SOURCES:.c=.o)
SYS_INCLUDE := ../../../include/
SYS2_INCLUDE := ../../linux/include/
SYSASM := -I$(SYS_INCLUDE) -I$(SYS2_INCLUDE) -I../usr_incl/

#IS_I386 := $(shell $(CC) -dumpmachine | grep i[3456]86 )

CPPFLAGS = -D_POSIX_TIMERS=1 -D_POSIX_C_SOURCE=199309L -D_XOPEN_SOURCE $(SYSASM)
CFLAGS = -g -Wall
LIBPOSIXTIME = libposixtime.a

all: $(LIBPOSIXTIME)($(OBJECTS))

clean:
	rm -f *.o *.a *~ core .depend t--* tmp

.depend: $(SOURCES) test_itimerspec
	$(CC) $(CPPFLAGS)  -M $(SOURCES) | \
		sed -e '/:/s|^[^ :]*|$(LIBPOSIXTIME)(&)|' > .depend
	chmod +x test_itimerspec
	./test_itimerspec $(CC) $(CPPFLAGS) &>/dev/null
	make REMOTE_KERNEL=$(REMOTE_KERNEL)
# The above make insures that "all:" is done with the newly created .depend

include .depend
