/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"

#define BAUD_RATE 38400

uint8_t serial1_rx_buffer[RX_BUFFER_SIZE];
uint8_t serial1_rx_buffer_head = 0;
volatile uint8_t serial1_rx_buffer_tail = 0;

uint8_t serial1_tx_buffer[TX_BUFFER_SIZE];
uint8_t serial1_tx_buffer_head = 0;
volatile uint8_t serial1_tx_buffer_tail = 0;


#ifdef ENABLE_XONXOFF
  volatile uint8_t flow_ctrl = XON_SENT; // Flow control state variable
#endif
  

// Returns the number of bytes used in the RX serial buffer.
uint8_t serial1_get_rx_buffer_count()
{
  uint8_t rtail = serial1_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial1_rx_buffer_head >= rtail) { return(serial1_rx_buffer_head-rtail); }
  return (RX_BUFFER_SIZE - (rtail-serial1_rx_buffer_head));
}


// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial1_get_tx_buffer_count()
{
  uint8_t ttail = serial1_tx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial1_tx_buffer_head >= ttail) { return(serial1_tx_buffer_head-ttail); }
  return (TX_BUFFER_SIZE - (ttail-serial1_tx_buffer_head));
}
#if 0

void serial1_init()
{
  // Set baud rate
  #if BAUD_RATE < 57600
    uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
    UCSR1A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
  #else
    uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
    UCSR1A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200
  #endif
  UBRR1H = UBRR0_value >> 8;
  UBRR1L = UBRR0_value;
            
  // enable rx and tx
  UCSR1B |= 1<<RXEN0;
  UCSR1B |= 1<<TXEN0;
	
  // enable interrupt on complete reception of a byte
  UCSR1B |= 1<<RXCIE0;
	  
  // defaults to 8-bit, no parity, 1 stop bit
}


// Writes one byte to the TX serial buffer. Called by main program.
// TODO: Check if we can speed this up for writing strings, rather than single bytes.
void serial1_write(uint8_t data) {
  // Calculate next head
  uint8_t next_head = serial1_tx_buffer_head + 1;
  if (next_head == TX_BUFFER_SIZE) { next_head = 0; }

  // Wait until there is space in the buffer
  while (next_head == serial1_tx_buffer_tail) { 
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.    
    if (sys_rt_exec_state & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
  }

  // Store data and advance head
  serial1_tx_buffer[serial1_tx_buffer_head] = data;
  serial1_tx_buffer_head = next_head;
  
  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
  UCSR1B |=  (1 << UDRIE0); 
}


// Data Register Empty Interrupt handler
ISR(USART1_UDRE_vect)
{
  uint8_t tail = serial1_tx_buffer_tail; // Temporary serial1_tx_buffer_tail (to optimize for volatile)
  
  #ifdef ENABLE_XONXOFF
    if (flow_ctrl == SEND_XOFF) { 
      UDR1 = XOFF_CHAR; 
      flow_ctrl = XOFF_SENT; 
    } else if (flow_ctrl == SEND_XON) { 
      UDR1 = XON_CHAR; 
      flow_ctrl = XON_SENT; 
    } else
  #endif
  { 
    // Send a byte from the buffer	
    UDR1 = serial1_tx_buffer[tail];
  
    // Update tail position
    tail++;
    if (tail == TX_BUFFER_SIZE) { tail = 0; }
  
    serial1_tx_buffer_tail = tail;
  }
  
  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (tail == serial1_tx_buffer_head) { UCSR1B &= ~(1 << UDRIE0); }
}


// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial1_read()
{
  uint8_t tail = serial1_rx_buffer_tail; // Temporary serial1_rx_buffer_tail (to optimize for volatile)
  if (serial1_rx_buffer_head == tail) {
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = serial1_rx_buffer[tail];
    
    tail++;
    if (tail == RX_BUFFER_SIZE) { tail = 0; }
    serial1_rx_buffer_tail = tail;

    #ifdef ENABLE_XONXOFF
      if ((serial1_get_rx_buffer_count() < RX_BUFFER_LOW) && flow_ctrl == XOFF_SENT) { 
        flow_ctrl = SEND_XON;
        UCSR1B |=  (1 << UDRIE0); // Force TX
      }
    #endif
    
    return data;
  }
}


ISR(USART1_RX_vect)
{
  uint8_t data = UDR1;
  uint8_t next_head;
  
  // Pick off realtime command characters directly from the serial stream. These characters are
  // not passed into the buffer, but these set system state flag bits for realtime execution.
  switch (data) {
    case CMD_STATUS_REPORT: bit_true_atomic(sys_rt_exec_state, EXEC_STATUS_REPORT); break; // Set as true
    case CMD_CYCLE_START:   bit_true_atomic(sys_rt_exec_state, EXEC_CYCLE_START); break; // Set as true
    case CMD_FEED_HOLD:     bit_true_atomic(sys_rt_exec_state, EXEC_FEED_HOLD); break; // Set as true
    case CMD_SAFETY_DOOR:   bit_true_atomic(sys_rt_exec_state, EXEC_SAFETY_DOOR); break; // Set as true
    case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
    default: // Write character to buffer    
      next_head = serial1_rx_buffer_head + 1;
      if (next_head == RX_BUFFER_SIZE) { next_head = 0; }
    
      // Write data to buffer unless it is full.
      if (next_head != serial1_rx_buffer_tail) {
        serial1_rx_buffer[serial1_rx_buffer_head] = data;
        serial1_rx_buffer_head = next_head;    
        
        #ifdef ENABLE_XONXOFF
          if ((serial1_get_rx_buffer_count() >= RX_BUFFER_FULL) && flow_ctrl == XON_SENT) {
            flow_ctrl = SEND_XOFF;
            UCSR1B |=  (1 << UDRIE0); // Force TX
          } 
        #endif
        
      }
      //TODO: else alarm on overflow?
  }
}


void serial1_reset_read_buffer() 
{
  serial1_rx_buffer_tail = serial1_rx_buffer_head;

  #ifdef ENABLE_XONXOFF
    flow_ctrl = XON_SENT;
  #endif
}
#else

#define setBit(a,b) (a |= (1<<b))
#define clearBit(a,b) (a &= ~(1<<b))
#define DELAY 26

#define TX_LOW() clearBit(PORTD, 3)
#define TX_HIGH() setBit(PORTD, 3)

enum STATE {
	IDLE,
	START,
	DATA,
	STOP
};

volatile unsigned char curr_byte;
volatile unsigned char curr_bit;
volatile unsigned char state = IDLE;

static void schedule_transmit(unsigned char data) {

	cli();
	curr_byte = data;
	curr_bit = 0;


	TCNT2 = 0;
	TIFR |= (1<<OCF2);
	TIMSK |= (1<<OCIE2);
	TX_LOW();

	state = START;
	sei();


}

static uint8_t get_next_char() {
	uint8_t tail = serial1_tx_buffer_tail;
	
	uint8_t next_char = serial1_tx_buffer[tail];

	// Update tail position
	tail++;
	if (tail == TX_BUFFER_SIZE) { tail = 0; }

	serial1_tx_buffer_tail = tail;
	
	return next_char;
}

static void put_char(uint8_t data) {
  // Calculate next head
  uint8_t next_head = serial1_tx_buffer_head + 1;
  if (next_head == TX_BUFFER_SIZE) { next_head = 0; }

  // Wait until there is space in the buffer
  while (next_head == serial1_tx_buffer_tail) { 
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.    
    if (sys_rt_exec_state & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
  }

  // Store data and advance head
  serial1_tx_buffer[serial1_tx_buffer_head] = data;
  serial1_tx_buffer_head = next_head;

}

static bool next_char_avail() {
	return serial1_tx_buffer_tail != serial1_tx_buffer_head;
}

ISR(TIMER2_COMP_vect) {

	if(state == START) {
		state = DATA;
		curr_bit = 0;
	} 
	
	if(state == DATA && curr_bit == 8) {
		state = STOP;
		TX_HIGH();
		return;
	}  
	
	if(state == DATA) {
		if(curr_byte & 1) {
			TX_HIGH();
		} else {
			TX_LOW();
		}
		curr_byte >>= 1;
		curr_bit++;
		return;
	} 
	if(state == STOP) {
		if (next_char_avail()) { //more data to send
			schedule_transmit(get_next_char());
			//TIMSK &= ~(1<<OCIE2);
		} else {
			state = IDLE;
			TIMSK &= ~(1<<OCIE2); // disable interrupts
		}
	}

}


void serial1_write(unsigned char data) {
	put_char(data);

	if(state == IDLE) {
		schedule_transmit(get_next_char());
	} 
}

void serial1_init() {
	DDRD |= 1<<3;
	TX_HIGH();
	
	TCCR2 = (1<<WGM21) | (1<<CS21);
	OCR2 = DELAY*2;
}

void serial1_reset_read_buffer() {}

#endif

