//*************************************************************************************
/** \file base232.cpp
 *    This file contains a class which operates an asynchronous (RS-232 style) serial 
 *    port on an AVR microcontroller. Classes which use this stuff include rs232 and 
 *    packet232. 
 *
 *  Revisions:
 *    \li 09-14-2017 CTR Adapted from JRR code for AVR to be compatible with xmega series
 *
 *  License:
 *    This file is released under the Lesser GNU Public License, version 2. This 
 *    program is intended for educational use only, but it is not limited thereto. 
 */
//*************************************************************************************

#include <avr/io.h>						// Definitions of AVR's I/O registers
#include "base232.h"


//-------------------------------------------------------------------------------------
/** This method sets up the AVR UART for communications.  It enables the appropriate 
 *  inputs and outputs and sets the baud rate divisor, and it saves pointers to the
 *  registers which are used to operate the serial port. Since some AVR processors
 *  have dual serial ports, this method allows one to specify a port number. 
 *  @param baud_rate The desired baud rate for serial communications. Default is 9600
 *  @param port_number The number of the serial port, 0 or 1 (the second port numbered
 *                     1 only exists on some processors). The default is port 0 
 */

base232::base232 (unsigned int baud_rate, USART_t* p_usart)
{
	// Store pointers to usart struct and port struct
	p_usart = p_usart;
	p_port = p_port;
	
	if(p_usart == &USARTC0)
	{
		p_port = &PORTC;
		rxd_pin = 2;
		txd_pin = 3;
	}
#ifdef USARTC1 
	else if(p_usart == &USARTC1)
	{
		p_port = &PORTC;
		rxd_pin = 6;
		txd_pin = 7;
	}
#endif
#ifdef USARTD0
	else if(p_usart == &USARTD0)
	{
		p_port = &PORTD;
		rxd_pin = 2;
		txd_pin = 3;
	}
#endif
#ifdef USARTD1 
	else if(p_usart == &USARTD1)
	{
		p_port = &PORTD;
		rxd_pin = 6;
		txd_pin = 7;
	}
#endif
#ifdef USARTE0
	else if(p_usart == &USARTE0)
	{
		p_port = &PORTE;
		rxd_pin = 2;
		txd_pin = 3;
	}
#endif
#ifdef USARTE1 
	else if(p_usart == &USARTE1)
	{
		p_port = &PORTE;
		rxd_pin = 6;
		txd_pin = 7;
	}
#endif
#ifdef USARTF0 
	else if(p_usart == &USARTF0)
	{
		p_port = &PORTF;
		rxd_pin = 2;
		txd_pin = 3;
	}
#endif
	else
	{
	}
	
	// Store pointers to data, status, config registers
	p_UDR = &(p_usart->DATA);
	p_USR = &(p_usart->STATUS);
	p_UCR = &(p_usart->CTRLB);
	
	// Set up TXD as an output that is high
	p_port->OUT |= (1 << txd_pin);
	p_port->DIR |= (1 << txd_pin);
	
	// Set up RXD as an input
	p_port->DIR &= ~(1 << rxd_pin);
	
	// Enable interrupts for RX at high priority
	p_usart->CTRLA = (0 << USART_RXCINTLVL1_bp | 1 << USART_RXCINTLVL0_bp);
	
	// Select USART Asynchronous Mode and choose character size
	p_usart->CTRLC = (1 << USART_CHSIZE0_bp | 1 << USART_CHSIZE1_bp );

	// Select the baudrate according to Table 23-5 in the datasheet
	p_usart->BAUDCTRLB = (-1 << USART_BSCALE0_bp);
	p_usart->BAUDCTRLA = (33 << USART_BSEL0_bp);

	// Enable the RX and TX devices
	p_usart->CTRLB = (1 << USART_RXEN_bp | 1<< USART_TXEN_bp);

	// Save some masks
	mask_RXC = USART_RXCIF_bm;
	mask_TXC = USART_TXCIF_bm;
	mask_UDRE = USART_DREIF_bm;
	
	// Read the data register to ensure that it's empty
	p_UDR[0]; p_UDR[0];
}


//-------------------------------------------------------------------------------------
/** This method checks if the serial port transmitter is ready to send data.  It 
 *  tests whether transmitter buffer is empty. 
 *  @return True if the serial port is ready to send, and false if not
 */

bool base232::ready_to_send (void)
{
// If transmitter buffer is full, we're not ready to send
if (*p_USR & mask_UDRE)
	return (true);

return (false);
}


//-------------------------------------------------------------------------------------
/** This method checks if the serial port is currently sending a character. Even if the
 *  buffer is ready to accept a new character, the port might still be busy sending the
 *  last one; it would be a bad idea to put the processor to sleep before the character
 *  has been sent. 
 *  @return True if the port is currently sending a character, false if it's idle
 */

bool base232::is_sending (void)
{
if (*p_USR & mask_TXC)
	return (false);
else
	return (true);
}