/*
  LIN.h- Library for Arduino LIN shield based on Hardware Serial Library for Arduino
  ver 1.0 created by Sherzaad Dinah
  Revision History
  ver1.0 - Newly created
  ver1.1 - fixed incorrect PID calculation in "GetPID" 
  ver1.2 - "GetChecksum" revised. no change to calculation method
*/

#include "LIN.h"

//return the LIN frame type received by UART buffer
uint16_t LINClass::FieldType(uint16_t val) {
  return (val & 0xFF00);
}

void LINClass::SendBreak(HardwareSerialLIN &s, unsigned long baud, uint8_t brkbits){
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
uint8_t LINClass::GetPID(uint8_t id)
{
  uint8_t pid, bit6, bit7, temp;

  temp = id & 0x03F; //get rid of P0 and P1 in case included
  
  //P0(bit6) = ID0^ID1^ID2^ID4
  //P1(bit7) = !(ID1^ID3^ID4^ID5)

  //calculate parity bits for protected id(pid) from id
  //calcultate P0
  bit6 = ((temp^(((temp>>1)^(temp>>2))^(temp>>4)))&0x01)<<6;

  //calcultate P1
  bit7 = (~(((temp>>1)^(((temp>>3)^(temp>>4))^(temp>>5)))&0x01))<<7;

  pid = temp | (bit6 | bit7);

  return pid;
}

//returns if given PID is valid or not
uint8_t LINClass::VerifyPID(uint8_t pid)
{
  uint8_t _pid = GetPID(pid);

  if (_pid == pid) return 1;

  return 0; //invalid pid
}

//returns Checksum for given LIN frame
uint8_t LINClass::GetChecksum(uint8_t pid, uint8_t *msg_data, uint8_t dlc, enum Chksum_T typ)
{
  uint8_t length = dlc;

  //Classic Checksum. May be used in all LIN protocol versions
  uint16_t calc_checksum = msg_data[0];
  for (uint8_t i = 1; i < length; ++i) {
    calc_checksum += msg_data[i];
    if(calc_checksum > 255) calc_checksum -= 255;
  }

  if (typ == ENHANCED) { //Enhanced Checksum. May be used only where LIN Protocol 2.x has been applied
    calc_checksum += pid;
  }

  if (calc_checksum > 255) calc_checksum -= 255;

  return (uint8_t) ((~calc_checksum) & 0x00FF);
}

//send breakfield, sync field and LIN PID
void LINClass::SendHeaderFrame(HardwareSerialLIN &s,unsigned long baud, uint8_t pid, uint8_t brkbits, uint8_t sync)
{
  SendBreak(s, baud, brkbits);
  
  s.write(sync);
  s.write(pid);
}

//Send LIN data frame + checksum
void LINClass::SendDataFrame(HardwareSerialLIN &s, uint8_t *data, uint8_t dlc, uint8_t checksum)
{
  uint8_t n = dlc;

  for (uint8_t i = 0; i < n; ++i) {
    s.write(data[i]);
  }

  s.write(checksum);
}
