#ifndef _ASMPPC_DIV64_H
#define _ASMPPC_DIV64_H
#include <asm-generic/div64.h>

#define div_long_long_rem(a,b,c) div_ll_X_l_rem(a,b,c)

static inline unsigned long div_ll_X_l_rem(unsigned long long divs, 
					   unsigned long div, 
					   unsigned long * rem)
{
	unsigned long long it = divs;
	*rem = do_div(it, div);
	return (unsigned long)it;
}
#endif /* _ASMPPC_DIV64_H */
