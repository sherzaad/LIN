/*
  LIN.h- Library for Arduino LIN shield based on Hardware Serial Library for Arduino
  ver 1.0 created by Sherzaad Dinah
  Revision History
  ver1.0 - Newly created
  ver1.1 - Moved 'break' routine to file to fix sending serial break issue
*/

#ifndef	LIN_H
#define	LIN_H

#include <LIN_defs.h>
#include <HardwareSerialLIN.h>
#include <HardwareSerialLIN_private.h>

//return the LIN frame type received by UART buffer
inline uint16_t LIN_FieldType(uint16_t val) {
  return (val & 0xFF00);
}

inline LIN_Send_Break(HardwareSerialLIN &s, unsigned long baud, uint8_t brkbits = BREAK_BITS){
  unsigned long brk_baud = 9 * (baud/ brkbits);

  //breakfield mode
  s.end();
  s.begin(brk_baud);
  s.write(0x00);

  //normal mode
  s.end();
  s.begin(baud);	
}

//return PID for given LIN ID
inline uint8_t LIN_PID_Calc(uint8_t id)
{
  uint8_t pid, bit6, bit7, temp;

  temp = id & 0x03F; //get rid of P1 and P2 in case included

  //calculate parity bits for protected id(pid) from id
  //calcultate P1
  bit6 = BIT_VAL(id, 0);
  bit6 ^= BIT_VAL(id, 1);
  bit6 ^= BIT_VAL(id, 2);
  bit6 ^= BIT_VAL(id, 4);
  bit6 = (bit6 && 0x01) << 6;

  //calcultate P2
  bit7 = BIT_VAL(id, 1);
  bit7 ^= BIT_VAL(id, 3);
  bit7 ^= BIT_VAL(id, 4);
  bit7 ^= BIT_VAL(id, 5);
  bit7 = (bit7 && 0x01) << 7;

  pid = temp | bit6 | bit7;

  return pid;
}

//returns if given PID is valid or not
inline uint8_t Verify_LIN_PID(uint8_t pid)
{
  uint8_t _pid = LIN_PID_Calc(pid);

  if (_pid == pid) return 1;

  return 0; //invalid pid
}

//returns Checksum for given LIN frame
inline uint8_t LIN_Checksum_Calc(uint8_t pid, uint8_t *msg_data, uint8_t dlc, uint8_t LINver = 2)
{
  uint8_t length = dlc;
  uint16_t calc_checksum = 0;

  //----LINver=1 any version before 2.0. LINver=2 version 2.0 and above----//
  //LINver=1 Classic Checksum
  for (uint8_t i = 0; i < length; ++i) {
    calc_checksum += msg_data[i];
  }

  if (LINver == 2) { //Enhanced Checksum
    calc_checksum += pid;
  }

  if (calc_checksum >= 256) calc_checksum -= 255;

  return (uint8_t) ((~calc_checksum) & 0x00FF);
}

//send breakfield, sync field and LIN PID
inline void Send_LIN_Header_Frame(HardwareSerialLIN &s,unsigned long baud, uint8_t pid, uint8_t brkbits = BREAK_BITS, uint8_t sync = 0x55)
{
  LIN_Send_Break(s, baud, brkbits);
  
  s.write(sync);
  s.write(pid);
}

//Send LIN data frame + checksum
inline void Send_LIN_Data_Frame(HardwareSerialLIN &s, uint8_t *data, uint8_t dlc, uint8_t checksum)
{
  uint8_t n = dlc;

  for (uint8_t i = 0; i < n; ++i) {
    s.write(data[i]);
  }

  s.write(checksum);
}
#endif
