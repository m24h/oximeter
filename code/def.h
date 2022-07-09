#ifndef     __CONFIG_H__
#define     __CONFIG_H__

#ifdef SDCC

#include	"stc8g_sdcc.h"

#define interrupt __interrupt
#define code __code
#define data __data
#define idata __idata
#define xdata __xdata
#define pdata __pdata
#define bit   __bit

#define _sbit_(a,b) __sbit __at b a
#define _sfr_(a,b) __sfr __at b a

#define _nop_()   __asm NOP __endasm
#define _push_(x) __asm push _##x  __endasm
#define _pop_(x)  __asm pop  _##x  __endasm

// SMALL-ENDIAN
typedef struct {
	unsigned char ll;
	unsigned char lh;
	unsigned char hl;
	unsigned char hh;
} u32s;

typedef struct {
	unsigned char l;
	unsigned char h;
} u16s;

#else

#include	"stc8g.h"
#include  <INTRINS.H>

#define _sbit_(a,b) sbit a = b
#define _sfr_(a,b) sfr a = b

// BIG-ENDIAN
typedef struct {
	unsigned char hh;
	unsigned char hl;
	unsigned char lh;
	unsigned char ll;
} u32s;

typedef struct {
	unsigned char h;
	unsigned char l;
} u16s;

#endif

typedef 	unsigned char	u8;
typedef 	unsigned int	u16;
typedef 	unsigned long	u32;

typedef 	signed char	i8;
typedef 	signed int		i16;
typedef 	signed long	i32;

#define _ea_clr_() \
	do { \
		_push_(IE);\
		EA=0; \
		_nop_(); \
		_nop_();

#define _ea_set_() \
	do { \
		_push_(IE);\
		_nop_(); \
		EA=1;
	
#define _ea_pop_() \
		_nop_(); \
		_pop_(IE); \
		_nop_(); \
		_nop_(); \
  } while(0);

#endif

