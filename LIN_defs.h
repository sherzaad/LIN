/*
  LIN_defs.h- Library for Arduino LIN shield based on Hardware Serial Library for Arduino
  ver 1.0 created by Sherzaad Dinah
  Revision History
  ver1.0 - Newly created
  version 1.1 - enum Chksum_T added to support update to "GetChecksum" routine
*/

#ifndef	LIN_DEFS_H
#define	LIN_DEFS_H
#include <arduino.h>
/*
  #define GET_BIT(x,y) (((x)&(1<<(y)))>0? 1:0) //returns value of bit y in byte x
  #define SET_BIT(x,y) ((x)|=(1<<(y))) //sets bit y in byte x
  #define CLR_BIT(x,y) ((x)&=(~(1<<(y)))) //clear bit y in byte x
  #define TGL_BIT(x,y) ((x)^=(1<<(y))) //toggle bit y in byte x
*/
#define LINFRAME_NOMINAL 124	// 124 = 34 +90 [brk_field(14) + sync_brk_field(10) + PID(10) = 34, 10*8(max payload bytes) + checksum(10) = 90]
#define LINFRAME_TIMEOUT 2	// Wait this many frames before declaring a read timeout.
#define READTIMEOUT(x,y) (((x)*(y)*14)/10) //specced addtl 40% space above normal hence 1.4*nominal frame.
//x = LINFRAME_NOMINAL
//y = LINFRAME_TIMEOUT
#define BIT_TIMEOUT READTIMEOUT(LINFRAME_TIMEOUT,LINFRAME_NOMINAL)
#define BIT_VAL(x,y) (((x)&(1<<(y)))>0? 1:0) //returns value selected bit '0' or '1'

#define BREAK_BITS 13 //number of bits in brk_field. (one less to account to serial start bit)
//#define FRAMEERROR 0x0100
#define BREAKFIELD 0x0100
#define SYNC 0x0200
#define PID 0x0300
#define DATA 0x0400
#define CHECKSUM 0x0500
#define NEW_FRAME 0x0600

enum Chksum_T {CLASSIC, ENHANCED};

typedef struct
{
	uint8_t Breakfield; //number of bits forming the breakfield
	uint8_t Sync; //value of sync field
	uint8_t Pid;  //ID/PID value
	uint8_t dlc;  //length of data frame (excluding checksum)
	uint8_t Data[9];  //array to contain data frame information (8(max) + 1 to include checksum field)
	uint8_t Checksum; //value of checksum. Would normally be equal to the last element of Data array
} tLIN;

//extern tLIN linframe;

#endif
