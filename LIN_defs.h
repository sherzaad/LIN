/*
LIN.h- Library for Arduino LIN shield
ver 1.0 created by Sherzaad Dinah
Revision History
ver1.0 - Newly created
*/

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
#define BREAKFIELD 0
#define SYNC 1
#define PID 2
#define DATA 3
#define CHECKSUM 4
#define NEW_FRAME 5 

typedef struct
{
	uint8_t sync;
	uint8_t pid;
	uint8_t dlc;
	uint8_t data[8];
	uint8_t checksum;
} tLIN;

#endif
