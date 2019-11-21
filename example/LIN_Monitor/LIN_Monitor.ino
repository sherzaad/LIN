#include <LIN.h>

#define LIN_BAUD1 9600
#define LIN_BAUD2 19200UL

void setup() {
  Serial.begin(115200);
  LIN1.begin(LIN_BAUD2);

  Serial.println("READY");

}

void loop() {
  union {
    uint16_t val;
    uint8_t bytes[2];
  } in;

  if (LIN1.available()) {
    in.val = LIN1.read();

    if (in.bytes[1] == BREAKFIELD) {
      Serial.println("");
    }
    else if (in.bytes[1] == NEW_FRAME) {
      Serial.println("");
      Serial.print("New Frame: ");
      Serial.print(in.bytes[0], HEX);
      Serial.print(", ");
    }
    else if (in.bytes[1] == SYNC) {
      Serial.print("Sync: ");
      Serial.print(in.bytes[0], HEX);
      Serial.print(", ");
    }
    else if (in.bytes[1] == PID) {
      Serial.print("PID: ");
      Serial.print(in.bytes[0], HEX);
      Serial.print(", ID: ");
      Serial.print(in.bytes[0] & 0x3F, HEX);
      Serial.print(", Data: ");
    }
    else if (in.bytes[1] == DATA) {
      Serial.print(in.bytes[0], HEX); //data/checksum bytes
      Serial.print(", ");
    }
    else if (in.bytes[1] == CHECKSUM){
      Serial.print("Checksum: ");
      Serial.print(in.bytes[0], HEX); //checksum bytes
    }
  }

}
