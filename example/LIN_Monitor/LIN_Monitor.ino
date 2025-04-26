/* This example was tested on a Mega2560. Serial1 was connected via a suitable LIN transceiver such as a MCP2003 to sniff LIN bus*/

#include "LIN.h"

#define LIN_BAUD1 9600
#define LIN_BAUD2 19200UL

void setup() {
  /*IMPORTANT BEFORE START*/
  /*
   * if you are using ANY of the Hw Serial Ports for other that LIN, 
   * you MUST move the corresponding HardwareSerialLINx.cpp file to the 'OtherThanLIN' folder.
   * Else you may have 'multiple definition' compile error 
   */
  Serial.begin(115200); //HardwareSerialLIN0.cpp moved to 'OtherThanLIN' folder since using Serial0 port for Serial Monitor
  LIN1.begin(LIN_BAUD2);

  Serial.println("READY");

}

void loop() {
  if (LIN1.available()) {
    LIN.RxData = LIN1.read();

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
      Serial.print(LIN.RxDataValue() & 0x3F, HEX);
      Serial.print(", Data: ");
    }
    else if (LIN.RxDataType() == DATA) {
      Serial.print(LIN.RxDataValue(), HEX); //data/checksum bytes. 
                                     //Last received DATA byte of a given LIN frame would normally be the Checksum Byte
      Serial.print(", ");
    }
  }

}
