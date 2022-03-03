#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <util/atomic.h>
#include "Arduino.h"

#include "HardwareSerialLIN.h"
#include "HardwareSerialLIN_private.h"

// this next line disables the entire HardwareSerialLIN.cpp,
// this is so I can support Attiny series and any other chip without a uart
#if defined(HAVE_HWSERIAL0) || defined(HAVE_HWSERIAL1) || defined(HAVE_HWSERIAL2) || defined(HAVE_HWSERIAL3)

// SerialEvent functions are weak, so when the user doesn't define them,
// the linker just sets their address to 0 (which is checked below).
// The Serialx_available is just a wrapper around Serialx.available(),
// but we can refer to it weakly so we don't pull in the entire
// HardwareSerialLIN instance if the user doesn't also refer to it.
#if defined(HAVE_HWSERIAL0)
void LINserialEvent() __attribute__((weak));
bool LINSerial0_available() __attribute__((weak));
#endif

#if defined(HAVE_HWSERIAL1)
void LINserialEvent1() __attribute__((weak));
bool LINSerial1_available() __attribute__((weak));
#endif

#if defined(HAVE_HWSERIAL2)
void LINserialEvent2() __attribute__((weak));
bool LINSerial2_available() __attribute__((weak));
#endif

#if defined(HAVE_HWSERIAL3)
void LINserialEvent3() __attribute__((weak));
bool LINSerial3_available() __attribute__((weak));
#endif

void LINserialEventRun(void)
{
#if defined(HAVE_HWSERIAL0)
  if (LINSerial0_available && LINserialEvent && LINSerial0_available()) LINserialEvent();
#endif
#if defined(HAVE_HWSERIAL1)
  if (LINSerial1_available && LINserialEvent1 && LINSerial1_available()) LINserialEvent1();
#endif
#if defined(HAVE_HWSERIAL2)
  if (LINSerial2_available && LINserialEvent2 && LINSerial2_available()) LINserialEvent2();
#endif
#if defined(HAVE_HWSERIAL3)
  if (LINSerial3_available && LINserialEvent3 && LINSerial3_available()) LINserialEvent3();
#endif
}

// macro to guard critical sections when needed for large TX buffer sizes
#if (SERIAL_TX_BUFFER_SIZE>256)
#define TX_BUFFER_ATOMIC ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#else
#define TX_BUFFER_ATOMIC
#endif

// Actual interrupt handlers //////////////////////////////////////////////////////////////

void HardwareSerialLIN::_tx_udr_empty_irq(void)
{
  // If interrupts are enabled, there must be more data in the output
  // buffer. Send the next byte
  unsigned char c = _tx_buffer[_tx_buffer_tail];
  _tx_buffer_tail = (_tx_buffer_tail + 1) % SERIAL_TX_BUFFER_SIZE;

  *_udr = c;

  // clear the TXC bit -- "can be cleared by writing a one to its bit
  // location". This makes sure flush() won't return until the bytes
  // actually got written. Other r/w bits are preserved, and zeroes
  // written to the rest.

#ifdef MPCM0
  *_ucsra = ((*_ucsra) & ((1 << U2X0) | (1 << MPCM0))) | (1 << TXC0);
#else
  *_ucsra = ((*_ucsra) & ((1 << U2X0) | (1 << TXC0)));
#endif

  if (_tx_buffer_head == _tx_buffer_tail) {
    // Buffer empty, so disable interrupts
    cbi(*_ucsrb, UDRIE0);
  }
}

// Public Methods //////////////////////////////////////////////////////////////

void HardwareSerialLIN::begin(unsigned long baud, byte config)
{
  // Try u2x mode first
  uint16_t baud_setting = (F_CPU / 4 / baud - 1) / 2;

  Read_Timeout(baud);
  *_ucsra = 1 << U2X0;

  // hardcoded exception for 57600 for compatibility with the bootloader
  // shipped with the Duemilanove and previous boards and the firmware
  // on the 8U2 on the Uno and Mega 2560. Also, The baud_setting cannot
  // be > 4095, so switch back to non-u2x mode if the baud rate is too
  // low.
  if (((F_CPU == 16000000UL) && (baud == 57600)) || (baud_setting > 4095))
  {
    *_ucsra = 0;
    baud_setting = (F_CPU / 8 / baud - 1) / 2;
  }

  // assign the baud_setting, a.k.a. ubrr (USART Baud Rate Register)
  *_ubrrh = baud_setting >> 8;
  *_ubrrl = baud_setting;

  _written = false;

  //set the data bits, parity, and stop bits
#if defined(__AVR_ATmega8__)
  config |= 0x80; // select UCSRC register (shared with UBRRH)
#endif
  *_ucsrc = config;

  sbi(*_ucsrb, RXEN0);
  sbi(*_ucsrb, TXEN0);
  sbi(*_ucsrb, RXCIE0);
  cbi(*_ucsrb, UDRIE0);
}

void HardwareSerialLIN::end()
{
  // wait for transmission of outgoing data
  flush();

  cbi(*_ucsrb, RXEN0);
  cbi(*_ucsrb, TXEN0);
  cbi(*_ucsrb, RXCIE0);
  cbi(*_ucsrb, UDRIE0);

  // clear any received data
  _rx_buffer_head = _rx_buffer_tail;
}

unsigned long HardwareSerialLIN::Read_Timeout(unsigned long baud, unsigned long bittimeout)
{
  unsigned long t_bit;
  t_bit = 1000000 / (baud);

  timeout_bit = bittimeout * t_bit;

  return timeout_bit;
}
int HardwareSerialLIN::available(void)
{
  return ((unsigned int)(SERIAL_RX_BUFFER_SIZE + _rx_buffer_head - _rx_buffer_tail)) % SERIAL_RX_BUFFER_SIZE;
}

int HardwareSerialLIN::peek(void)
{
  if (_rx_buffer_head == _rx_buffer_tail) {
    return -1;
  } else {
    return _rx_buffer[_rx_buffer_tail];
  }
}

int HardwareSerialLIN::read(void)
{
  uint32_t oldtime, newtime;
  rx_buffer_index_t last_buf;
  uint16_t val;

  // if the head isn't ahead of the tail, we don't have any characters
  if (_rx_buffer_head == _rx_buffer_tail) {
    return -1;
  }
  else {
    last_buf = (rx_buffer_index_t)(_rx_buffer_tail + SERIAL_RX_BUFFER_SIZE - 1) % SERIAL_RX_BUFFER_SIZE;
    val = _rx_buffer[_rx_buffer_tail];
    oldtime = _timestamp_buffer[last_buf];
    newtime = _timestamp_buffer[_rx_buffer_tail];

    if ((val & 0x00FF) == 0 && (val & (1 << (8 + FE0))) != 0) {
      val = (val & 0x00FF) | BREAKFIELD;
      //previous byte = CHECKSUM; //no point since previous data was already read!
    }
    else if (newtime - oldtime > timeout_bit) {
      val = (val & 0x00FF) | NEW_FRAME;
    }
    else if ((val & 0x00FF) == 0x55 && ((_rx_buffer[last_buf] & 0xFF00) | BREAKFIELD) == BREAKFIELD) {
      val = (val & 0x00FF) | SYNC;
    }
    else if (((_rx_buffer[last_buf] & 0xFF00) | SYNC) == SYNC) {
      val = (val & 0x00FF) | PID;
    }
    else val = (val & 0x00FF) | DATA;

    _rx_buffer[_rx_buffer_tail] = val;
    _rx_buffer_tail = (rx_buffer_index_t)(_rx_buffer_tail + 1) % SERIAL_RX_BUFFER_SIZE;

    return val;
  }
}

uint32_t *HardwareSerialLIN::getTimestamp(void)
{
  static uint32_t val;
  //if the head isn't ahead of the tail, we don't have any characters
  if (_rx_buffer_head == _rx_buffer_tail)  {
    return nullptr;
  }
  else {
    val  = _timestamp_buffer[_rx_buffer_tail];
    return &val;
  }
}

int HardwareSerialLIN::availableForWrite(void)
{
  tx_buffer_index_t head;
  tx_buffer_index_t tail;

  TX_BUFFER_ATOMIC {
    head = _tx_buffer_head;
    tail = _tx_buffer_tail;
  }
  if (head >= tail) return SERIAL_TX_BUFFER_SIZE - 1 - head + tail;
  return tail - head - 1;
}

void HardwareSerialLIN::flush()
{
  // If we have never written a byte, no need to flush. This special
  // case is needed since there is no way to force the TXC (transmit
  // complete) bit to 1 during initialization
  if (!_written)
    return;

  while (bit_is_set(*_ucsrb, UDRIE0) || bit_is_clear(*_ucsra, TXC0)) {
    if (bit_is_clear(SREG, SREG_I) && bit_is_set(*_ucsrb, UDRIE0))
      // Interrupts are globally disabled, but the DR empty
      // interrupt should be enabled, so poll the DR empty flag to
      // prevent deadlock
      if (bit_is_set(*_ucsra, UDRE0))
        _tx_udr_empty_irq();
  }
  // If we get here, nothing is queued anymore (DRIE is disabled) and
  // the hardware finished tranmission (TXC is set).
}

size_t HardwareSerialLIN::write(uint8_t c)
{
  _written = true;
  // If the buffer and the data register is empty, just write the byte
  // to the data register and be done. This shortcut helps
  // significantly improve the effective datarate at high (>
  // 500kbit/s) bitrates, where interrupt overhead becomes a slowdown.
  if (_tx_buffer_head == _tx_buffer_tail && bit_is_set(*_ucsra, UDRE0)) {
    // If TXC is cleared before writing UDR and the previous byte
    // completes before writing to UDR, TXC will be set but a byte
    // is still being transmitted causing flush() to return too soon.
    // So writing UDR must happen first.
    // Writing UDR and clearing TC must be done atomically, otherwise
    // interrupts might delay the TXC clear so the byte written to UDR
    // is transmitted (setting TXC) before clearing TXC. Then TXC will
    // be cleared when no bytes are left, causing flush() to hang
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      *_udr = c;
#ifdef MPCM0
      *_ucsra = ((*_ucsra) & ((1 << U2X0) | (1 << MPCM0))) | (1 << TXC0);
#else
      *_ucsra = ((*_ucsra) & ((1 << U2X0) | (1 << TXC0)));
#endif
    }
    return 1;
  }
  tx_buffer_index_t i = (_tx_buffer_head + 1) % SERIAL_TX_BUFFER_SIZE;

  // If the output buffer is full, there's nothing for it other than to
  // wait for the interrupt handler to empty it a bit
  while (i == _tx_buffer_tail) {
    if (bit_is_clear(SREG, SREG_I)) {
      // Interrupts are disabled, so we'll have to poll the data
      // register empty flag ourselves. If it is set, pretend an
      // interrupt has happened and call the handler to free up
      // space for us.
      if (bit_is_set(*_ucsra, UDRE0))
        _tx_udr_empty_irq();
    } else {
      // nop, the interrupt handler will free up space for us
    }
  }

  _tx_buffer[_tx_buffer_head] = c;

  // make atomic to prevent execution of ISR between setting the
  // head pointer and setting the interrupt flag resulting in buffer
  // retransmission
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    _tx_buffer_head = i;
    sbi(*_ucsrb, UDRIE0);
  }

  return 1;
}

#endif // whole file
