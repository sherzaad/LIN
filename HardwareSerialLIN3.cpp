#include "Arduino.h"
#include "HardwareSerialLIN.h"
#include "HardwareSerialLIN_private.h"

// Each HardwareSerial is defined in its own file, sine the linker pulls
// in the entire file when any element inside is used. --gc-sections can
// additionally cause unused symbols to be dropped, but ISRs have the
// "used" attribute so are never dropped and they keep the
// HardwareSerial instance in as well. Putting each instance in its own
// file prevents the linker from pulling in any unused instances in the
// first place.

#if defined(HAVE_HWSERIAL3)

ISR(USART3_RX_vect)
{
  LIN3._rx_complete_irq();
}

ISR(USART3_UDRE_vect)
{
  LIN3._tx_udr_empty_irq();
}

HardwareSerialLIN LIN3(&UBRR3H, &UBRR3L, &UCSR3A, &UCSR3B, &UCSR3C, &UDR3);

// Function that can be weakly referenced by serialEventRun to prevent
// pulling in this file if it's not otherwise used.
bool LINSerial3_available() {
  return LIN3.available();
}

#endif // HAVE_HWSERIAL3
