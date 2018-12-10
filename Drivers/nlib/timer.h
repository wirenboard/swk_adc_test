#ifndef TIMER_H
#define TIMER_H

#include "core.h"

extern volatile u32 global_ms;

struct Timer
{
	u32 timeout;
	u32 start;
};

#define timer_set_timeout(tim, val) 	(&tim)->timeout = val
#define timer_reset(tim) 				(&tim)->start = global_ms
#define timer_get_time(tim)				(global_ms - (&tim)->start)
#define timer_is_over(tim) 				(timer_get_time(tim) > (&tim)->timeout)

void init_systick(void);
void delay_ms(u32 time);
void delay_us(u32 time);

#endif
