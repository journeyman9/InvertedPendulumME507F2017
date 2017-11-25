//*************************************************************************************
/** \file rs232int.h
 *    This file contains a class which allows the use of a serial port on an AVR 
 *    microcontroller. This version of the class uses the serial port receiver 
 *    interrupt and a buffer to allow characters to be received in the background.
 *    The port is used in "text mode"; that is, the information which is sent and 
 *    received is expected to be plain ASCII text, and the set of overloaded left-shift 
 *    operators "<<" in emstream.* can be used to easily send all sorts of data 
 *    to the serial port in a manner similar to iostreams (like "cout") in regular C++. 
 *
 *  Revisions:
 *    \li 09-14-2017 CTR Adapted from JRR code for AVR to be compatible with xmega 
 *                       series
 *
 *  License:
 *    This file is released under the Lesser GNU Public License, version 2. This 
 *    program is intended for educational use only, but it is not limited thereto. 
 */

/// This define prevents this .h file from being included more than once in a .cpp file
#ifndef _RS232_H_
#define _RS232_H_

#include <avr/interrupt.h>					// Header for AVR interrupt programming
#include "base232.h"						// Grab the base RS232-style header file
#include "emstream.h"						// Pull in the base class header file



/** This is the size of the buffer which holds characters received by the serial port.
 *  It is usually set to something fairly large (~100 bytes) so that we don't miss
 *  incoming characters. However, when run on an AVR with very little RAM such as an
 *  ATmega8, ATmega32, ATmega324P or similar, it should usually be set smaller, for
 *  example 20 ~ 30 bytes or so. 
 */
#define RSINT_BUF_SIZE		100


//-------------------------------------------------------------------------------------
/** \brief This class controls a UART (Universal Asynchronous Receiver Transmitter), 
 *  a common asynchronous serial interface used in microcontrollers, and allows the
 *  use of an overloaded \c << operator to print things in \c cout style. 
 * 
 *  \details It talks to old-style RS232 serial ports (through a voltage converter chip 
 *  such as a MAX232) or through a USB to serial converter such as a FT232RL chip. The 
 *  UART (or USART on a newer AVR chip) is also sometimes used to communicate directly 
 *  with other microcontrollers, sensors, or wireless modems. 
 * 
 *  In the version in files \c rs232int.* this class installs an interrupt service
 *  routine (ISR) for receiving characters. When characters arrive in the UART, they
 *  are placed in a buffer whose size is configurable with the macro \c RSINT_BUF_SIZE.
 *  Calls to \c getchar() will check the buffer for received characters. This method,
 *  as opposed to polling the receiver without using interrupts, allows much higher
 *  data rates to be reliably supported in a multitasking program. Sending of 
 *  characters is currently not interrupt based. 
 * 
 *  \section Usage
 *  Creation and use of a serial port driver object requires only code such as the
 *  following:
 *  \code
 *  rs232 ser_port (9600, &USARTC0);
 *  ser_port << "Hello from test program number " << prog_num << endl;
 *  \endcode
 *  The first parameter to the \c rs232 constructor is the baud rate; 9600 is by far 
 *  the most commonly used. The second parameter is the USART struct; use USART C0,
 *  C1, D0, D1, E0, E1, and F0. Multiple \c rs232 objects may be made on many USART channels
 */

class rs232 : public emstream, public base232
{
	// This protected data can only be accessed from this class or its descendents
	protected:
		uint8_t**	p_rcv_buffer;
		uint16_t*	p_rcv_read_index;
		uint16_t*	p_rcv_write_index;

	// Public methods can be called from anywhere in the program where there is a 
	// pointer or reference to an object of this class
	public:
		// The constructor sets up the UART, saving its baud rate and a pointer to the usart struct
		rs232 (uint16_t = 9600, USART_t* = &USARTC0);

		// This method writes one character to the serial port.
		bool putchar (char);

		bool check_for_char (void);			// Check if a character is in the buffer
		int16_t getchar (void);				// Get a character; wait if none is ready
		void clear_screen (void);			// Send the 'clear display screen' code
};

#endif  // _RS232_H_
