# Makefile: install high-res-timers man pages
#
# We attempt to find the "correct" directory for the man pages as follows:

# If the enviroment variable MANPATH is defined and not null, use the first
# entry in it.

# Otherwise, if /etc/man.config exists, use the first MANPATH entry in it.
# If both of these fail, print an error and quit.

# If you want to override this to put the pages in directory foo use:
# make MANPATH=foo

# If you want to force it to use the /etc/man.config and you have a MANPATH use:
# make MANPATH=

# You may need to be root to run this make.

all: man
MANCONF = /etc/man.config
MANPAGES = clock_getcpuclockid.3 clock_getres.3 clock_gettime.3 \
           clock_nanosleep.3 clock_settime.3 pthread_getcpuclockid.3 \
           timer_create.3 timer_delete.3 timer_getoverrun.3 \
           timer_gettime.3 timer_settime.3

man: $(MANPAGES)
	@ENVMANPATH=`echo $$MANPATH | sed -e "s/:.*//"` ;\
	if [ "$$ENVMANPATH" = "" ] ; then \
		if [ -e $(MANCONF) ] ; then \
			MANDIR=`grep -w '^ *MANPATH' $(MANCONF) |sed -e 2,50d -e "s/ *MANPATH[^/]*//"` ; \
		else \
			MANDIR="" ; \
		fi \
	else \
		MANDIR=$$ENVMANPATH ; \
	fi  ; \
	if [ -d $$MANDIR/man3 ] ; then \
		echo "Installing POSIX timers man pages to >$$MANDIR/< ..."; \
		cp $(MANPAGES) $$MANDIR/man3 && \
		cd $$MANDIR/man3 && \
		chmod 644 $(MANPAGES) && \
		gzip -9 $(MANPAGES); \
	else \
		echo "error: directory >$$MANDIR/man3< does not exist, create or use make MANPATH=<dir>!"; \
	fi
