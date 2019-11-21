/*
  HardwareSerial.cpp - Hardware serial library for Wiring
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
  Modified 3 December 2013 by Matthijs Kooijman
  //--- Modified 21 December 2017 by Sherzaad
  list of changes:
  - _tx_udr_empty_irq(void) function updated to make possible 9-bit serial TX
  - begin function updated to allow 9 bit serial configuration
  - read/write functions updated to support 9 bit serial
  ---//
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include <HardwareSerialLIN.h>
#include <HardwareSerialLIN_private.h>

// this next line disables the entire HardwareSerial.cpp, 
// this is so I can support Attiny series and any other chip without a uart
#if defined(HAVE_HWSERIAL0) || defined(HAVE_HWSERIAL1) || defined(HAVE_HWSERIAL2) || defined(HAVE_HWSERIAL3)

// SerialEvent functions are weak, so when the user doesn't define them,
// the linker just sets their address to 0 (which is checked below).
// The Serialx_available is just a wrapper around Serialx.available(),
// but we can refer to it weakly so we don't pull in the entire
// HardwareSerial instance if the user doesn't also refer to it.
#if defined(HAVE_HWSERIAL0)
  void serialEvent() __attribute__((weak));
  bool Serial0_available() __attribute__((weak));
#endif

#if defined(HAVE_HWSERIAL1)
  void serialEvent1() __attribute__((weak));
  bool Serial1_available() __attribute__((weak));
#endif

#if defined(HAVE_HWSERIAL2)
  void serialEvent2() __attribute__((weak));
  bool Serial2_available() __attribute__((weak));
#endif

#if defined(HAVE_HWSERIAL3)
  void serialEvent3() __attribute__((weak));
  bool Serial3_available() __attribute__((weak));
#endif

void serialEventRun(void)
{
#if defined(HAVE_HWSERIAL0)
  if (Serial0_available && serialEvent && Serial0_available()) serialEvent();
#endif
#if defined(HAVE_HWSERIAL1)
  if (Serial1_available && serialEvent1 && Serial1_available()) serialEvent1();
#endif
#if defined(HAVE_HWSERIAL2)
  if (Serial2_available && serialEvent2 && Serial2_available()) serialEvent2();
#endif
#if defined(HAVE_HWSERIAL3)
  if (Serial3_available && serialEvent3 && Serial3_available()) serialEvent3();
#endif
}

// Actual interrupt handlers //////////////////////////////////////////////////////////////

void HardwareSerialLIN::_tx_udr_empty_irq(void)
{
  // If interrupts are enabled, there must be more data in the output
  // buffer. Send the next byte

  _tx.val = _tx_buffer[_tx_buffer_tail];
  _tx_buffer_tail = (_tx_buffer_tail + 1) % SERIAL_TX_BUFFER_SIZE;

  if(bit_is_set(*_ucsrb, UCSZ02)){
	if(_tx.bytes[1]) sbi(*_ucsrb, TXB80);
	else cbi(*_ucsrb, TXB80);  
  } 
  *_udr = _tx.bytes[0];

  // clear the TXC bit -- "can be cleared by writing a one to its bit
  // location". This makes sure flush() won't return until the bytes
  // actually got written
  sbi(*_ucsra, TXC0);

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
  uint8_t config1 = config & 0x7F;
  
  baudrate = baud;
  
  Read_Timeout();
  
  *_ucsra = 1 << U2X0;

  // hardcoded exception for 57600 for compatibility with the bootloader
  // shipped with the Duemilanove and previous boards and the firmware
  // on the 8U2 on the Uno and Mega 2560. Also, The baud_setting cannot
  // be > 4095, so switch back to non-u2x mode if the baud rate is too
  // low.
  if (((F_CPU == 16000000UL) && (baud == 57600)) || (baud_setting >4095))
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
  config1 |= 0x80; // select UCSRC register (shared with UBRRH)
#endif
  *_ucsrc = config1;
  
  sbi(*_ucsrb, RXEN0);
  sbi(*_ucsrb, TXEN0);
  sbi(*_ucsrb, RXCIE0);
  cbi(*_ucsrb, UDRIE0);
  
  // set the 9 bit character size if required
  if(bit_is_set(config,7)) sbi(*_ucsrb, UCSZ02);
  else cbi(*_ucsrb, UCSZ02);

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

void HardwareSerialLIN::send_break(uint8_t brk_bits)
{
	unsigned long brk_baud = 9*(baudrate/brk_bits);
	
	//breakfield mode
	end();
	begin(brk_baud);
	write(0x00);
	
	//normal mode
	end();
	begin(baudrate);
	
}

unsigned long HardwareSerialLIN::Read_Timeout(unsigned long bittimeout)
{ 
	unsigned long t_bit;
	t_bit = 1000000/(baudrate);

	timeout_bit = bittimeout*t_bit;
	
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

union {
  uint16_t val;
  uint8_t bytes[2];
} in, pre;

unsigned long oldtime, newtime;

rx_buffer_index_t last_buf;

  // if the head isn't ahead of the tail, we don't have any characters
  if (_rx_buffer_head == _rx_buffer_tail) {
    return -1;
  } else {
		last_buf = (rx_buffer_index_t)(_rx_buffer_tail + SERIAL_RX_BUFFER_SIZE - 1) % SERIAL_RX_BUFFER_SIZE;
		in.val = _rx_buffer[_rx_buffer_tail];
		pre.val = _rx_buffer[last_buf];
		oldtime = _timestamp_buffer[last_buf];
		newtime = _timestamp_buffer[_rx_buffer_tail];
		
		if (newtime - oldtime > timeout_bit){
			in.bytes[1] = NEW_FRAME;
			//pre.bytes[1] = DATA;
			//_rx_buffer[last_buf] = pre.val; //no point since previous data was already read!
		}
		else if (in.bytes[0] == 0 && bit_is_set(in.bytes[1], FE0) != 0){
			in.bytes[1] = BREAKFIELD;
			//pre.bytes[1] = CHECKSUM;
			//_rx_buffer[last_buf] = pre.val; //no point since previous data was already read!
		}
		else if (pre.bytes[1] == BREAKFIELD) in.bytes[1] = SYNC;
		else if (pre.bytes[1] == SYNC) in.bytes[1] = PID;
		else in.bytes[1] = DATA;
		
		_rx_buffer[_rx_buffer_tail] = in.val;
		_rx_buffer_tail = (rx_buffer_index_t)(_rx_buffer_tail + 1) % SERIAL_RX_BUFFER_SIZE;
		
    return in.val;
  }
}

int64_t HardwareSerialLIN::getTimestamp(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (_rx_buffer_head == _rx_buffer_tail) {
    return -1;
  } else {
		int64_t c = (int64_t) _timestamp_buffer[_rx_buffer_tail];
    return c;
  }
}

int HardwareSerialLIN::availableForWrite(void)
{
#if (SERIAL_TX_BUFFER_SIZE>256)
  uint8_t oldSREG = SREG;
  cli();
#endif
  tx_buffer_index_t head = _tx_buffer_head;
  tx_buffer_index_t tail = _tx_buffer_tail;
#if (SERIAL_TX_BUFFER_SIZE>256)
  SREG = oldSREG;
#endif
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

size_t HardwareSerialLIN::write(uint16_t n)
{
	
  _written = true;
  
  _tx.val = n & 0x1FF;
  // If the buffer and the data register is empty, just write the byte
  // to the data register and be done. This shortcut helps
  // significantly improve the effective datarate at high (>
  // 500kbit/s) bitrates, where interrupt overhead becomes a slowdown.
  if (_tx_buffer_head == _tx_buffer_tail && bit_is_set(*_ucsra, UDRE0)) {
	if(bit_is_set(*_ucsrb, UCSZ02)){
		// set the 9th bit character size if required
		if(_tx.bytes[1]) sbi(*_ucsrb, TXB80);
		else cbi(*_ucsrb, TXB80);   
	} 
    *_udr = _tx.bytes[0];
    sbi(*_ucsra, TXC0);
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
      if(bit_is_set(*_ucsra, UDRE0))
	_tx_udr_empty_irq();
    } else {
      // nop, the interrupt handler will free up space for us
    }
  }

  _tx_buffer[_tx_buffer_head] = _tx.val;
  _tx_buffer_head = i;
	
  sbi(*_ucsrb, UDRIE0);
  
  return 1;
}

#endif // whole file
