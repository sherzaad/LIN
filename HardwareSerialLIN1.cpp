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

#if defined(HAVE_HWSERIAL1)

#if defined(UART1_RX_vect)
ISR(UART1_RX_vect)
#elif defined(USART1_RX_vect)
ISR(USART1_RX_vect)
#else
#error "Don't know what the Data Register Empty vector is called for Serial1"
#endif
{
  LIN1._rx_complete_irq();
}

#if defined(UART1_UDRE_vect)
ISR(UART1_UDRE_vect)
#elif defined(USART1_UDRE_vect)
ISR(USART1_UDRE_vect)
#else
#error "Don't know what the Data Register Empty vector is called for Serial1"
#endif
{
  LIN1._tx_udr_empty_irq();
}

HardwareSerialLIN LIN1(&UBRR1H, &UBRR1L, &UCSR1A, &UCSR1B, &UCSR1C, &UDR1);

// Function that can be weakly referenced by serialEventRun to prevent
// pulling in this file if it's not otherwise used.
bool LINSerial1_available() {
  return LIN1.available();
}

#endif // HAVE_HWSERIAL1
