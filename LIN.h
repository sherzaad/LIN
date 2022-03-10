/*
  LIN.h- Library for Arduino LIN shield based on Hardware Serial Library for Arduino
  ver 1.0 created by Sherzaad Dinah
  Revision History
  ver1.0 - Newly created
  ver1.1 - Moved 'SendBreak' routine to file to fix sending serial break issue
  ver1.2 - converted LIN.h functions into class.
*/

#ifndef	LIN_H
#define	LIN_H

#include "LIN_defs.h"
#include "HardwareSerialLIN.h"
#include "HardwareSerialLIN_private.h"

class LINClass{
	public:
		LINClass() {};
		uint16_t FieldType(uint16_t val); //return the LIN frame type of element read from UART buffer
		void SendBreak(HardwareSerialLIN &s, unsigned long baud, uint8_t brkbits = BREAK_BITS); //send break field
		uint8_t GetPID(uint8_t id); //return PID for given LIN ID
		uint8_t VerifyPID(uint8_t pid); //returns if given PID is valid or not
		uint8_t GetChecksum(uint8_t pid, uint8_t *msg_data, uint8_t dlc, uint8_t LINver = 2); //returns Checksum for given LIN frame
		void SendHeaderFrame(HardwareSerialLIN &s,unsigned long baud, uint8_t pid, uint8_t brkbits = BREAK_BITS, uint8_t sync = 0x55); //send breakfield, sync field and LIN PID
		void SendDataFrame(HardwareSerialLIN &s, uint8_t *data, uint8_t dlc, uint8_t checksum); //Send LIN data frame + checksum
	private:
};
extern LINClass LIN;
#endif
