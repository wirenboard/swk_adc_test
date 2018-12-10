#include "core.h"
#include "print.h"

void write_u1(char c);

//		dp_ 	value
//		dpn_	value + crlf
//		db_		str + value
//		dbn_	str + value + crlf

#define dbc write_u1

#define db(str) 					print(str, dbc)
#define db_l(str, l)				print_field(str, l, dbc)
#define db_crlf()					dbc('\r');	dbc('\n')
#define dbn(str)					db(str); db_crlf()

#define dp_hb8(buf, instr, len)		print_hb(buf, 1, len, instr, dbc)
#define dp_hb16(buf, instr, len)	print_hb(buf, 2, len, instr, dbc)
#define dp_hb32(buf, instr, len)	print_hb(buf, 4, len, instr, dbc)

#define dp_d(val)					printDEC(val, 0, dbc)
#define dp_dl(val, l)				printDEC(val, l, dbc)

#define dp_h8(a) 					print_hex(&a, 1, dbc)
#define dp_h16(a) 					print_hex(&a, 2, dbc)
#define dp_h32(a) 					print_hex(&a, 4, dbc)

#define db_h8(str, val)				db(str); dp_h8(val)
#define db_h16(str, val)			db(str); dp_h16(val)
#define db_h32(str, val)			db(str); dp_h32(val)

#define db_d(str, val)				db(str); dp_d(val)
#define db_dl(str, val, l)			db(str); dp_dl(val, l)

#define dbn_h8(str, val)			db(str); dp_h8(val); db_crlf()
#define dbn_h16(str, val)			db(str); dp_h16(val); db_crlf()
#define dbn_h32(str, val)			db(str); dp_h32(val); db_crlf()
#define dbn_d(str, val)				db_d(str, val); db_crlf()
#define dbn_dl(str, val, l)			db_dl(str, val, l); db_crlf()

