#include "debug.h"
#include "core.h"

void write_u1(char c)
{
	while(!(USART1->SR & USART_SR_TC));
	USART1->DR = c;
}

