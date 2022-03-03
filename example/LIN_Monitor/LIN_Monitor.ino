#include <LIN.h>

#define LIN_BAUD1 9600
#define LIN_BAUD2 19200UL

void setup() {
  /*IMPORTANT BEFORE START*/
  /*
   * if you are using ANY other the Hw Serial Ports for other that LIN, 
   * you MUST move the corresponding HardwareSerialLINx.cpp file to the 'OtherThanLIN' folder.
   * Else you may have 'multiple definition' compile error 
   */
  Serial.begin(115200); //HardwareSerialLIN0.cpp moved to 'OtherThanLIN' folder since using Serial0 port for Serial Monitor
  LIN1.begin(LIN_BAUD2);

  Serial.println("READY");

}

void loop() {
  if (LIN1.available()) {
    uint16_t val = LIN1.read();

    if (LIN_FieldType(val) == BREAKFIELD) {
      Serial.println("");
    }
    else if (LIN_FieldType(val) == NEW_FRAME) {
      Serial.println("");
      Serial.print("New Frame: ");
      Serial.print(val & 0xFF, HEX);
      Serial.print(", ");
    }
    else if (LIN_FieldType(val) == SYNC) {
      Serial.print("Sync: ");
      Serial.print(val & 0xFF, HEX);
      Serial.print(", ");
    }
    else if (LIN_FieldType(val) == PID) {
      Serial.print("PID: ");
      Serial.print(val & 0xFF, HEX);
      Serial.print(", ID: ");
      Serial.print((val & 0xFF) & 0x3F, HEX);
      Serial.print(", Data: ");
    }
    else if (LIN_FieldType(val) == DATA) {
      Serial.print(val & 0xFF, HEX); //data/checksum bytes. 
                                     //Last received DATA byte of a given LIN frame would normally be the Checksum Byte
      Serial.print(", ");
    }
  }

}
