//*************************************************************************************
/** \file base232.h
 *    This file contains a set of macros which are used by several classes and class
 *    templates that interact with the asynchronous (RS-232 style) serial port on an 
 *    AVR microcontroller. Classes which use this stuff include rs232 and packet232. 
 *
 *  Revisions:
 *    \li 09-14-2017 CTR Adapted from JRR code for AVR to be compatible with xmega 
 *                       series
 *
 *  License:
 *    This file is released under the Lesser GNU Public License, version 2. This 
 *    program is intended for educational use only, but it is not limited thereto. 
 */
//*************************************************************************************

/// This define prevents this .h file from being included more than once in a .cpp file
#ifndef _BASE232_H_
#define _BASE232_H_


// Check that the user has set the CPU frequency in the Makefile; if not, complain
#if !defined (F_CPU)
	#error The macro F_CPU must be set in the Makefile
#endif

//-------------------------------------------------------------------------------------
/** The number of tries to wait for the transmitter buffer to become empty */
#define UART_TX_TOUT		20000

//-------------------------------------------------------------------------------------
/** \brief This is a base class for classes that use an RS-232 port on an AVR
 *  microcontroller. It is not designed to be used by itself; descendents such as 
 *  class \c rs232 should be used. 
 * 
 *  This class provides a basic hardware interface for an RS-232 port (also called a
 *  UART or USART in the AVR datasheets). It sets up the port, checks for characters, 
 *  and sends characters one at a time. Its descendant classes are the ones that do 
 *  useful things such as sending text messages and packets. 
 */

class base232
{
	protected:
		/// This is a pointer to the usart struct that will be used for this serial port
		USART_t* p_usart;
		
		/// This is a pointer to the port which contains the TXD and RXD pins
		PORT_t* p_port;
		
		/// This is a number indicating the pins where TXD and RXD are located
		volatile unsigned char rxd_pin;
		volatile unsigned char txd_pin;
	
		/// This is a pointer to the data register used by the UART
		volatile unsigned char* p_UDR;

		/// This is a pointer to the status register used by the UART
		volatile unsigned char* p_USR;

		/// This is a pointer to the control register used by the UART
		volatile unsigned char* p_UCR;

		/// This bitmask identifies the bit for data register empty, UDRE
		unsigned char mask_UDRE;

		/// This bitmask identifies the bit for receive complete, RXC
		unsigned char mask_RXC;

		/// This bitmask identifies the bit for transmission complete, TXC
		unsigned char mask_TXC;

	public:
		/// The constructor sets up the port with the given baud rate and port number.
		base232 (unsigned int = 9600, USART_t* = &USARTC0);
	
		/// This method checks if the serial port is ready to transmit data.
		bool ready_to_send (void);

		/// This method returns true if the port is currently sending a character out.
		bool is_sending (void);
};

#endif  // _BASE232_H_
