#ifndef	LIN_DEFS_H
#define	LIN_DEFS_H
/*
  #define GET_BIT(x,y) (((x)&(1<<(y)))>0? 1:0) //returns value of bit y in byte x
  #define SET_BIT(x,y) ((x)|=(1<<(y))) //sets bit y in byte x
  #define CLR_BIT(x,y) ((x)&=(~(1<<(y)))) //clear bit y in byte x
  #define TGL_BIT(x,y) ((x)^=(1<<(y))) //toggle bit y in byte x
*/
#define LINFRAME_NOMINAL 124	// 124 = 34 +90 [serial_brk(14) + sync_brk_field(10) + PID(10) = 34, 10*8(max payload bytes) + checksum(10) = 90]
#define LINFRAME_TIMEOUT 2	// Wait this many frames before declaring a read timeout.
#define READTIMEOUT(x,y) (((x)*(y)*14)/10) //specced addtl 40% space above normal hence 1.4*nominal frame.
//x = LINFRAME_NOMINAL
//y = LINFRAME_TIMEOUT
#define BIT_TIMEOUT READTIMEOUT(LINFRAME_TIMEOUT,LINFRAME_NOMINAL)
#define BIT_VAL(x,y) (((x)&(1<<(y)))>0? 1:0) //returns value selected bit '0' or '1'

#define BREAK_BITS 13 //number of bits in breakfield. (one less to account to serial start bit)
//#define FRAMEERROR 0x0100
#define BREAKFIELD 0x0100
#define SYNC 0x0200
#define PID 0x0300
#define DATA 0x0400
#define CHECKSUM 0x0500
#define NEW_FRAME 0x0600

#endif
