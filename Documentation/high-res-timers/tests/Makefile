#
# Copyright (C) 1997 by the University of Kansas Center for Research,
# Inc.  This software was developed by the Information and
# Telecommunication Technology Center (ITTC) at the University of
# Kansas.  Partial funding for this project was provided by Sprint. This
# software may be used and distributed according to the terms of the GNU
# Public License, incorporated herein by reference.  Neither ITTC nor
# Sprint accept any liability whatsoever for this product.
#
# This project was developed under the direction of Dr. Douglas Niehaus.
#
# Authors: Shyam Pather, Balaji Srinivasan 
#
# Please send bug-reports/suggestions/comments to posix@ittc.ukans.edu
#
# Further details about this project can be obtained at
#    http://hegel.ittc.ukans.edu/projects/posix/
#

# Makefile for POSIX.4 test/example programs.
# 
# Targets: 
#         time_tests   --> POSIX.4 interval timers and time_query tests

OS := $(shell uname)

CROSS_COMPILE	=
PREFIX		= /opt/hrtimers-support
DESTDIR		=
ifneq (x$(DESTDIR),x)
 DESTDIR=$(DESTDIR)/
endif

AS		= $(CROSS_COMPILE)as
LD		= $(CROSS_COMPILE)ld
CC		= $(CROSS_COMPILE)gcc
AR		= $(CROSS_COMPILE)ar
RM		= rm -f
LIB		= /lib
POSIX_LIBDIR	= ../lib
POSIX2_LIBDIR	= lib
POSIX_INCDIR	= ../lib
# to pick up the modified files, with out putting them in /usr/include
USE_INCDIR = ../usr_incl
USE2_INCDIR = usr_incl
debug :=
CFLAGS := -g -Wall $(debug)
ifeq ($(OS),Linux)
CPPFLAGS = -D_POSIX_TIMERS=1 -D_GNU_SOURCE -I$(POSIX_INCDIR) -I$(USE_INCDIR) -I$(USE2_INCDIR)
LDFLAGS = -L. 
endif
SCRIPTS = do_test
SOURCES = 	2timer_test.c \
		timer_test.c \
		clock_getrestest.c \
		clock_gettimetest.c \
		clock_settimetest.c \
		clock_gettimetest2.c \
	        clock_gettimetest3.c \
		clock_gettimetest4.c \
		clock_gettimetest5.c \
		clock_nanosleeptest.c \
		clock_nanosleep.c     \
		nanosleep_jitter.c \
		1-4.c                 \
		1-3.c                 \
                rt_sev_none.c         \
		timerlimit.c          \
		4-1.c                 \
		9-1.c                 \
		abs_timer_test.c      \
	        performance.c timerstress.c

PROGS = $(SOURCES:.c=) 
#
# Figure if glibc has what we need.  CLOCK_MONOTONIC being defined is 
# what we use.
#
needlib := $(shell grep CLOCK_MONOTONIC /usr/include/bits/time.h)
# When we get glibc to do the "right" thing, eliminate the following line.
needlib :=
ifeq ("$(needlib)","")
LIB:= libposixtime.a
LDLIB:= lposixtime
else
LIB:=
LDLIB:= lrt
endif
LDLIBS := -$(LDLIB) -lpthread

.PHONY: clean install uninstall

all: $(PROGS)

$(PROGS): $(LIB)

libposixtime.a : 
	@here=`pwd` ; \
	if [ -d $(POSIX_LIBDIR) ] ; then \
		cd $(POSIX_LIBDIR) ;\
		make  CC=$(CC) AR=$(AR) ; \
		cd $$here ; pwd ;\
		if [ -f $(POSIX_LIBDIR)/libposixtime.a ] ; then \
			$(RM) libposixtime.a ; \
			ln -s $(POSIX_LIBDIR)/libposixtime.a libposixtime.a ; \
		else  exit 1 ;\
		fi ;\
	elif [ -e $(POSIX2_LIBDIR) ] ; then \
		cd $(POSIX2_LIBDIR) ;\
		make  CC=$(CC) AR=$(AR) ; \
		cd $$here; \
		if [ -f $(POSIX2_LIBDIR)/libposixtime.a ] ; then \
			$(RM) libposixtime.a ; \
			ln -s $(POSIX2_LIBDIR)/libposixtime.a libposixtime.a ;\
		else  exit 1 ;\
		fi ;\
	elif [ -f $(LIB)/libposixtime.a ] ; then \
		$(RM) -f libposixtime.a ; \
		ln -s $(LIB)/libposixtime.a libposixtime.a ;\
	else  echo "error making libposixtime.a" ; exit 1 ;\
	fi

%: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $< -o $@ $(LDFLAGS) $(LDLIBS)

clean:
	$(RM) $(PROGS) *~ *.o core .depend libposixtime.a
	if [ -d $(POSIX_LIBDIR) ] ; then \
		(cd $(POSIX_LIBDIR) && make CC=$(CC) AR=$(AR) clean) ; \
	elif [ -e $(POSIX2_LIBDIR) ] ; then \
		(cd $(POSIX2_LIBDIR) && make CC=$(CC) AR=$(AR) clean) ; \
	else  echo "error cleaning lib" ; exit 1 ;\
	fi

$(DESTDIR)$(PREFIX):
	@mkdir -p $(DESTDIR)$(PREFIX)

install: $(DESTDIR)$(PREFIX) $(PROGS) $(SCRIPTS)
	(cd $(DESTDIR)$(PREFIX) && rm -f $(PROGS) $(SCRIPTS))
	cp $(PROGS) $(SCRIPTS) $(DESTDIR)$(PREFIX)
	for i in $(SCRIPTS); do chmod +x $(DESTDIR)$(PREFIX)/$${i}; done

uninstall:
	if [ -d $(DESTDIR)$(PREFIX) ] ; then \
		(cd $(DESTDIR)$(PREFIX) && rm -f $(PROGS) $(SCRIPTS)) ; \
	fi;

.depend depend: $(SOURCES)
	$(CC) -M $(CPPFLAGS) $(SOURCES) | \
		sed -e '/:/s|\(^[^ :]*\)\.o|\1|' > .depend
	chmod +x do_test
	make

# This above make insures that we have the .depend when doing the build.

include .depend
