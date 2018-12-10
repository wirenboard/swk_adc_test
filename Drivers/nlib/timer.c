#include "timer.h"

volatile u32 global_ms = 0;

// void SysTick_Handler(void)
// {
// 	global_ms++;
// }

void init_systick(void)
{
	SysTick->LOAD = F_CPU/1000 - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = (1 << 2) + (1 << 1) + (1 << 0);		// CLKSOURCE + TICKINT + ENABLE
}

void delay_ms(u32 time)
{
	int start_time = global_ms;
	while(global_ms - start_time < time);
}

void delay_us(u32 time)
{
	uint32_t ticks = time * (F_CPU/1000000);
	uint32_t start = SysTick->VAL;
	volatile uint32_t countflag = SysTick->CTRL & (1 << 16);
	countflag = 1;
	u32 systick_val;
	while (countflag)
	{
		systick_val = SysTick->VAL;
		if (systick_val > start)
		{
			start += F_CPU/1000;
		}
		if (start - systick_val > ticks) countflag = 0;
	}
}
