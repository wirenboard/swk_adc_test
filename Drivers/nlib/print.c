#include "print.h"

void print(char *str, void (*write)(char c))
{
	while (*str != 0)
	{
		write(*str++);
	}
}

static const char hexvalues[] = "0123456789ABCDEF";

void print_hex(u8 *ptr, u8 len, void (*write)(char c))
{
	while (len--)
	{
		write(hexvalues[(*(ptr + len) >> 4) & 0x0F]);
		write(hexvalues[*(ptr + len) & 0x0F]);
	}
}

void print_hb(u8 *ptr, u8 grp, u8 len, u8 inl, void (*write)(char c))
{
	for (u8 i = 0; i < len; i++)
	{
		print_hex(ptr + grp * i, grp, write);
		if ((inl != 0) && ((i % inl) == (inl - 1)))
		{
			write('\r');
			write('\n');
		}
		else
		{
			write(' ');
		}
	}
}

void print_field(char* str, u32 width, void (*write)(char c))
{
	u32 dec = 1;
	if(width == 0){
		width = 1;
		dec = 0;
	}
	while((*str != 0) && (width != 0)){
		write(*str++);
		width -= dec;
	}
	if(dec != 0){
		while(width != 0){
			write(' ');
			width--;
		}
	}
}

void printDEC(u32 value, u8 width, void (*write)(char c))
{
	u32 devider = 1;
	u32 digit;
	u8 dn = 1;

	if(value > 1000000000){
		devider = 1000000000;
		dn = 10;
	}
	else{
		while ((devider * 10) <= value){
			devider = devider * 10;
			dn++;
		}
	}

	if(width == 0){
		width = dn;
	}

	if(width < dn){
		write('@');
		width--;
	}

	while(width){
		if(width > dn){
			write(' ');
		}
		else{
			digit = value / devider;
			value = value % devider;
			devider = devider / 10;
			write('0' + digit);
		}
		width--;
	}
}