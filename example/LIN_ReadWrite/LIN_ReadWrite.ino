/* This example was tested on a Mega2560. Serial1(sender Lin Node) was looped back onto Serial2(receiver lin Node) to emulate a LIN network.*/

#include "LIN.h"

#define LIN_BAUD1 9600
#define LIN_BAUD2 19200UL

const uint8_t msg_pid = 0x1D;
const uint8_t msg_dlc = 4;
uint8_t msg_Data[msg_dlc] = {0x00,0x79,0x00,0x00};
uint8_t Checksum; 

void setup() {
  /*IMPORTANT BEFORE START*/
  /*
   * if you are using ANY of the Hw Serial Ports for other that LIN, 
   * you MUST move the corresponding HardwareSerialLINx.cpp file to the 'OtherThanLIN' folder.
   * Else you may have 'multiple definition' compile error 
   */
  Serial.begin(115200); //HardwareSerialLIN0.cpp moved to 'OtherThanLIN' folder since using Serial0 port for Serial Monitor
  LIN1.begin(LIN_BAUD1);
  LIN2.begin(LIN_BAUD1);
  Serial.println("READY");
  
  Checksum = LIN.GetChecksum(msg_pid, msg_Data, msg_dlc, ENHANCED); //assuming LIN Version Protocol 2.x

}

void loop() {
  Serial.print("\nSending LIN FRAME...");
  LIN.SendHeaderFrame(LIN1, msg_pid);
  LIN.SendDataFrame(LIN1, msg_Data , msg_dlc, Checksum);
  ++msg_Data[3];
  Checksum = LIN.GetChecksum(msg_pid, msg_Data, msg_dlc, ENHANCED); //refesh checksum value
  
  delay(1000); //arbitrary delay
  
  while (LIN2.available()) {
    LIN.RxData = LIN2.read();

    if (LIN.RxDataType() == BREAKFIELD) {
      Serial.println("");
    }
    else if (LIN.RxDataType() == NEW_FRAME) {
      Serial.println("");
      Serial.print("New Frame: ");
      Serial.print(LIN.RxDataValue(), HEX);
      Serial.print(", ");
    }
    else if (LIN.RxDataType() == SYNC) {
      Serial.print("Sync: ");
      Serial.print(LIN.RxDataValue(), HEX);
      Serial.print(", ");
    }
    else if (LIN.RxDataType() == PID) {
      Serial.print("PID: ");
      Serial.print(LIN.RxDataValue(), HEX);
      Serial.print(", ID: ");
      Serial.print((LIN.RxDataValue()) & 0x3F, HEX);
      Serial.print(", Data: ");
    }
    else if (LIN.RxDataType() == DATA) {
      Serial.print(LIN.RxDataValue(), HEX); //data/checksum bytes. 
                                     //Last received DATA byte of a given LIN frame would normally be the Checksum Byte
      Serial.print(", ");
    }
  }

}
