#include "core.h"

void print(char *str, void (*write)(char c));
void print_hex(u8 *ptr, u8 len, void (*write)(char c));
void print_hb(u8 *ptr, u8 grp, u8 len, u8 inl, void (*write)(char c));
void print_field(char* str, u32 width, void (*write)(char c));
void printDEC(u32 value, u8 width, void (*write)(char c));

