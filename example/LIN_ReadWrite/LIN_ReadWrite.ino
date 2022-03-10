#include "LIN.h"

#define LIN_BAUD1 9600
#define LIN_BAUD2 19200UL

const uint8_t msg_pid = 0x0D;
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
  LIN1.begin(LIN_BAUD2);
  LIN2.begin(LIN_BAUD2);
  Serial.println("READY");
  
  Checksum = LIN.GetChecksum(msg_pid, msg_Data, msg_dlc); //assuming LIN Version Protocol 2.x

}

void loop() {
  Serial.print("\nSending LIN FRAME...");
  LIN.SendHeaderFrame(LIN1, LIN_BAUD2, msg_pid);
  LIN.SendDataFrame(LIN1, msg_Data , msg_dlc, Checksum);
  
  delay(1000); //arbitrary delay
  
  while (LIN2.available()) {
    uint16_t val = LIN2.read();

    if (LIN.FieldType(val) == BREAKFIELD) {
      Serial.println("");
    }
    else if (LIN.FieldType(val) == NEW_FRAME) {
      Serial.println("");
      Serial.print("New Frame: ");
      Serial.print(val & 0xFF, HEX);
      Serial.print(", ");
    }
    else if (LIN.FieldType(val) == SYNC) {
      Serial.print("Sync: ");
      Serial.print(val & 0xFF, HEX);
      Serial.print(", ");
    }
    else if (LIN.FieldType(val) == PID) {
      Serial.print("PID: ");
      Serial.print(val & 0xFF, HEX);
      Serial.print(", ID: ");
      Serial.print((val & 0xFF) & 0x3F, HEX);
      Serial.print(", Data: ");
    }
    else if (LIN.FieldType(val) == DATA) {
      Serial.print(val & 0xFF, HEX); //data/checksum bytes. 
                                     //Last received DATA byte of a given LIN frame would normally be the Checksum Byte
      Serial.print(", ");
    }
  }

}
