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

#if defined(HAVE_HWSERIAL2)

ISR(USART2_RX_vect)
{
  LIN2._rx_complete_irq();
}

ISR(USART2_UDRE_vect)
{
  LIN2._tx_udr_empty_irq();
}

HardwareSerialLIN LIN2(&UBRR2H, &UBRR2L, &UCSR2A, &UCSR2B, &UCSR2C, &UDR2);

// Function that can be weakly referenced by serialEventRun to prevent
// pulling in this file if it's not otherwise used.
bool LINSerial2_available() {
  return LIN2.available();
}

#endif // HAVE_HWSERIAL2
