#include "wiring_private.h"

// this next line disables the entire HardwareSerial.cpp,
// this is so I can support Attiny series and any other chip without a uart
#if defined(HAVE_HWSERIAL0) || defined(HAVE_HWSERIAL1) || defined(HAVE_HWSERIAL2) || defined(HAVE_HWSERIAL3)

// Ensure that the various bit positions we use are available with a 0
// postfix, so we can always use the values for UART0 for all UARTs. The
// alternative, passing the various values for each UART to the
// HardwareSerial constructor also works, but makes the code bigger and
// slower.
#if !defined(TXC0)
#if defined(TXC)
// Some chips like ATmega8 don't have UPE, only PE. The other bits are
// named as expected.
#if !defined(UPE) && defined(PE)
#define UPE PE
#endif
// On ATmega8, the uart and its bits are not numbered, so there is no TXC0 etc.
#define TXC0 TXC
#define RXEN0 RXEN
#define TXEN0 TXEN
#define RXCIE0 RXCIE
#define UDRIE0 UDRIE
#define U2X0 U2X

#define UDRE0 UDRE
#define UCSZ02 UCSZ2
#define RXB80 RXB8
#define TXB80 TXB8
#define UPE0 UPE
#define FE0 FE
#define DOR0 DOR
#elif defined(TXC1)
// Some devices have uart1 but no uart0
#define TXC0 TXC1
#define RXEN0 RXEN1
#define TXEN0 TXEN1
#define RXCIE0 RXCIE1
#define UDRIE0 UDRIE1
#define U2X0 U2X1

#define UDRE0 UDRE1
#define UCSZ02 UCSZ12
#define RXB80 RXB81
#define TXB80 TXB81
#define UPE0 UPE1
#define FE0 FE1
#define DOR0 DOR1
#else
#error No UART found in HardwareSerialLIN.cpp
#endif
#endif // !defined TXC0

// Check at compiletime that it is really ok to use the bit positions of
// UART0 for the other UARTs as well, in case these values ever get
// changed for future hardware.
#if defined(TXC1) && (TXC1 != TXC0 || RXEN1 != RXEN0 || RXCIE1 != RXCIE0 || \
		      UDRIE1 != UDRIE0 || U2X1 != U2X0 || UPE1 != UPE0 || \
		      UDRE1 != UDRE0)
#error "Not all bit positions for UART1 are the same as for UART0"
#endif
#if defined(TXC2) && (TXC2 != TXC0 || RXEN2 != RXEN0 || RXCIE2 != RXCIE0 || \
		      UDRIE2 != UDRIE0 || U2X2 != U2X0 || UPE2 != UPE0 || \
		      UDRE2 != UDRE0)
#error "Not all bit positions for UART2 are the same as for UART0"
#endif
#if defined(TXC3) && (TXC3 != TXC0 || RXEN3 != RXEN0 || RXCIE3 != RXCIE0 || \
		      UDRIE3 != UDRIE0 || U3X3 != U3X0 || UPE3 != UPE0 || \
		      UDRE3 != UDRE0)
#error "Not all bit positions for UART3 are the same as for UART0"
#endif

// Constructors ////////////////////////////////////////////////////////////////

HardwareSerialLIN::HardwareSerialLIN(
  volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
  volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
  volatile uint8_t *ucsrc, volatile uint8_t *udr) :
  _ubrrh(ubrrh), _ubrrl(ubrrl),
  _ucsra(ucsra), _ucsrb(ucsrb), _ucsrc(ucsrc),
  _udr(udr),
  _rx_buffer_head(0), _rx_buffer_tail(0),
  _tx_buffer_head(0), _tx_buffer_tail(0)
{
}

// Actual interrupt handlers //////////////////////////////////////////////////////////////

void HardwareSerialLIN::_rx_complete_irq(void)
{
  //read byte and store it in the buffer if there is room
  uint16_t val;
  val = *_ucsra; //UCSRA for error flags status. Bit 4: FE – Frame Error, Bit 3: DOR – Data Overrun Error, Bit 2: PE – Parity Error.
  val = (val << 8) | *_udr ;

  rx_buffer_index_t i = (unsigned int)(_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if (i != _rx_buffer_tail) {
    _rx_buffer[_rx_buffer_head] = val;
    _timestamp_buffer[_rx_buffer_head] = micros();
    _rx_buffer_head = i;
  }
}

#endif // whole file
