#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>

#define myperror(s) MYperror(__BASE_FILE__,__LINE__,s)
int MYperror(char * where,int line,char *what)
{
        fprintf(stderr,"%s,%d:",where,line);
        perror(what);
	fflush(stderr);
	fflush(stdout);
        exit(0);
}
#define Try(f) ({int foobas = (f); \
                if( foobas == -1) \
                myperror( #f );   \
                foobas;})
#define SIGNAL1  SIGPROF //SIGUNUSED
#define SIGNAL2 SIGUSR1  //SIGHUP
/*
 * some funky signal handlers, don't do much, but allow us to notice them
 */
void handler1(int signo, siginfo_t *info, void *context)
{
	printf("handler1 entered: signal %d\n", signo);
}
void handler2(int signo, siginfo_t *info, void *context)
{
	printf("handler2 entered: signal %d\n", signo);
}
int main(int argc, char *argv[])
{
	struct sigaction sa;
        pid_t pid = getpid();
        pid_t child;

	sa.sa_flags = SA_SIGINFO;
	sa.sa_sigaction = handler1;
	Try(sigaction(SIGNAL1, &sa, NULL));
	sa.sa_sigaction = handler2;
	Try(sigaction(SIGNAL2, &sa, NULL));
        printf("my pid is %d, and paws is %d\n",getpid(),getppid());

        Try(child = fork());
        if(!child){
                
                printf("my pid is %d, and paws is %d\n",getpid(),getppid());
              
                Try(kill(getppid(),SIGNAL2));
                Try(kill(getppid(),SIGNAL1));
                exit(0);
        }
        sleep(2);
}
