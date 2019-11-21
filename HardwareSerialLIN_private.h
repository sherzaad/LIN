/*
  HardwareSerial_private.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus
  //--- Modified 21 December 2017 by Sherzaad
  list of changes:
  - added definitions for UCSZ02, RXB80, TXB80, FE0 and DOR0
  - update _rx_complete_irq(void) to receive 9-bit serial
  - Frame error(FE), Data Overrun Error(DOR) and Parity Error(UPE) status now appended to received data byte.
    > RX buffer LSByte -> rx data
	> RX buffer MSByte -> 9th data bit + error frags
  ---//
*/

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
		      UDRE1 != UDRE0 || UCSZ12 != UCSZ02)
#error "Not all bit positions for UART1 are the same as for UART0"
#endif
#if defined(TXC2) && (TXC2 != TXC0 || RXEN2 != RXEN0 || RXCIE2 != RXCIE0 || \
		      UDRIE2 != UDRIE0 || U2X2 != U2X0 || UPE2 != UPE0 || \
		      UDRE2 != UDRE0 || UCSZ22 != UCSZ02)
#error "Not all bit positions for UART2 are the same as for UART0"
#endif
#if defined(TXC3) && (TXC3 != TXC0 || RXEN3 != RXEN0 || RXCIE3 != RXCIE0 || \
		      UDRIE3 != UDRIE0 || U3X3 != U3X0 || UPE3 != UPE0 || \
		      UDRE3 != UDRE0 || UCSZ32 != UCSZ02)
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
    // No Parity error, read byte and store it in the buffer if there is
    // room
	
	_rx.bytes[1] = *_ucsra; //UCSRA for error flags status. Bit 4: FE – Frame Error, Bit 3: DOR – Data Overrun Error, Bit 2: PE – Parity Error.
    _rx.bytes[0] = *_udr ; 
	
    rx_buffer_index_t i = (unsigned int)(_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;
	
    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (i != _rx_buffer_tail) {
      _rx_buffer[_rx_buffer_head] = _rx.val;
	  _timestamp_buffer[_rx_buffer_head] = micros();
      _rx_buffer_head = i;
    }
}

#endif // whole file
