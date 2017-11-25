//*************************************************************************************
/** \file rs232int.cpp
 *    This file contains a class which allows the use of a serial port on an AVR 
 *    microcontroller. This version of the class uses the serial port receiver 
 *    interrupt and a buffer to allow characters to be received in the background.
 *    The port is used in "text mode"; that is, the information which is sent and 
 *    received is expected to be plain ASCII text, and the set of overloaded left-shift 
 *    operators "<<" in emstream.* can be used to easily send all sorts of data 
 *    to the serial port in a manner similar to iostreams (like "cout") in regular C++. 
 *
 *  Revised:
 *    \li 09-14-2017 CTR Adapted from JRR code for AVR to be compatibile with xmega series
 *
 *  License:
 *		This file is copyright 2012 by JR Ridgely and released under the Lesser GNU 
 *		Public License, version 2. It intended for educational use only, but its use
 *		is not limited thereto. */
/*		THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *		AND	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * 		IMPLIED 	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * 		ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * 		LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUEN-
 * 		TIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
 * 		OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * 		CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * 		OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * 		OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//*************************************************************************************

#include <stdint.h>
#include <stdlib.h>
#include <avr/io.h>
#include "rs232int.h"

uint8_t* rcvC0_buffer = NULL;
uint8_t* rcvC1_buffer = NULL;
uint8_t* rcvD0_buffer = NULL;
uint8_t* rcvD1_buffer = NULL;
uint8_t* rcvE0_buffer = NULL;
uint8_t* rcvE1_buffer = NULL;
uint8_t* rcvF0_buffer = NULL;

/// This index is used to write into serial character receiver buffer 0.
uint16_t rcvC0_read_index;
uint16_t rcvC1_read_index;
uint16_t rcvD0_read_index;
uint16_t rcvD1_read_index;
uint16_t rcvE0_read_index;
uint16_t rcvE1_read_index;
uint16_t rcvF0_read_index;

/// This index is used to read from serial character receiver buffer 0.
uint16_t rcvC0_write_index;
uint16_t rcvC1_write_index;
uint16_t rcvD0_write_index;
uint16_t rcvD1_write_index;
uint16_t rcvE0_write_index;
uint16_t rcvE1_write_index;
uint16_t rcvF0_write_index;

//-------------------------------------------------------------------------------------
/** This method sets up the AVR UART for communications.  It calls the emstream
 *  constructor, which prepares to convert numbers to text strings, and the base232 
 *  constructor, which does the work of setting up the serial port. Note that the user
 *  program must call sei() somewhere to enable global interrupts so that this driver
 *  will work. It is not called in this constructor because it's common to construct 
 *  many drivers which use interrupts, including this one, and then enable interrupts
 *  globally using sei() after all the constructors have been called. 
 *  @param baud_rate The desired baud rate for serial communications. Default is 9600
 *  @param p_usart A pointer to the desired USART c-struct. The default is USARTC0. On an
 *                 XMGEGA choices are C0, C1, D0, D1, E0, E1, F0  
 */

rs232::rs232 (uint16_t baud_rate, USART_t* p_usart)
	: emstream (), base232 (baud_rate, p_usart)
{
	if(p_usart == &USARTC0)
	{
		p_rcv_buffer		= &rcvC0_buffer;
		p_rcv_read_index	= &rcvC0_read_index;
		p_rcv_write_index	= &rcvC0_write_index;
	}
#ifdef USARTC1
	else if(p_usart == &USARTC1)
	{
		p_rcv_buffer		= &rcvC1_buffer;
		p_rcv_read_index	= &rcvC1_read_index;
		p_rcv_write_index	= &rcvC1_write_index;
	}
#endif
#ifdef USARTD0
	else if(p_usart == &USARTD0)
	{
		p_rcv_buffer		= &rcvD0_buffer;
		p_rcv_read_index	= &rcvD0_read_index;
		p_rcv_write_index	= &rcvD0_write_index;
	}
#endif
#ifdef USARTD1
	else if(p_usart == &USARTD1)
	{
		p_rcv_buffer		= &rcvD1_buffer;
		p_rcv_read_index	= &rcvD1_read_index;
		p_rcv_write_index	= &rcvD1_write_index;
	}
#endif
#ifdef USARTE0
	else if(p_usart == &USARTE0)
	{
		p_rcv_buffer		= &rcvE0_buffer;
		p_rcv_read_index	= &rcvE0_read_index;
		p_rcv_write_index	= &rcvE0_write_index;
	}
#endif
#ifdef USARTE1
	else if(p_usart == &USARTE1)
	{
		p_rcv_buffer		= &rcvE1_buffer;
		p_rcv_read_index	= &rcvE1_read_index;
		p_rcv_write_index	= &rcvE1_write_index;
	}
#endif
#ifdef USARTF0
	else if(p_usart == &USARTF0)
	{
		p_rcv_buffer		= &rcvF0_buffer;
		p_rcv_read_index	= &rcvF0_read_index;
		p_rcv_write_index	= &rcvF0_write_index;
	}
#endif
	else
	{
	}
	
	*p_rcv_buffer = new uint8_t[RSINT_BUF_SIZE];
	*p_rcv_read_index = 0;
	*p_rcv_write_index = 0;
}


//-------------------------------------------------------------------------------------
/** This method sends one character to the serial port.  It waits until the port is
 *  ready, so it can hold up the system for a while.  It times out if it waits too 
 *  long to send the character; you can check the return value to see if the character
 *  was successfully sent, or just cross your fingers and ignore the return value.
 *  Note 1:  It's possible that at slower baud rates and/or higher processor speeds, 
 *  this routine might time out even when the port is working fine.  A solution would
 *  be to change the count variable to an integer and use a larger starting number. 
 *  Note 2:  Fixed!  The count is now an integer and it works at lower baud rates.
 *  @param chout The character to be sent out
 *  @return True if everything was OK and false if there was a timeout
 */

bool rs232::putchar (char chout)
{
	// Now wait for the serial port transmitter buffer to be empty	 
	for (uint16_t count = 0; ((*p_USR & mask_UDRE) == 0); count++)
	{
		if (count > UART_TX_TOUT)
			return (false);
	}

	// Clear the TXCn bit so it can be used to check if the serial port is busy.  This
	// check needs to be done prior to putting the processor into sleep mode.  Oddly,
	// the TXCn bit is cleared by writing a one to its bit location
	*p_USR |= mask_TXC;

	// The CTS line is 0 and the transmitter buffer is empty, so send the character
	*p_UDR = chout;
	return (true);
}


//-------------------------------------------------------------------------------------
/** This method gets one character from the serial port, if one is there.  If not, it
 *  waits until there is a character available.  This can sometimes take a long time
 *  (even forever), so use this function carefully.  One should almost always use
 *  check_for_char() to ensure that there's data available first. 
 *  @return The character which was found in the serial port receive buffer
 */

int16_t rs232::getchar (void)
{
	uint8_t recv_char;						// Character read from the queue

	// Wait until there's a character in the receiver queue
	while (*p_rcv_read_index == *p_rcv_write_index);
	recv_char = (*p_rcv_buffer)[*p_rcv_read_index];
	if (++(*p_rcv_read_index) >= RSINT_BUF_SIZE)
		*p_rcv_read_index = 0;

	return (recv_char);
}


//-------------------------------------------------------------------------------------
/** This method checks if there is a character in the serial port's receiver queue.
 *  The queue will have been filled if a character came in through the serial port and
 *  caused an interrupt. 
 *  @return True for character available, false for no character available
 */

bool rs232::check_for_char (void)
{
	return (*p_rcv_read_index != *p_rcv_write_index);
}


//-------------------------------------------------------------------------------------
/** This method sends the ASCII code to clear a display screen. It is called when the
 *  format modifier 'clrscr' is inserted in a line of "<<" stuff.
 */

void rs232::clear_screen (void)
{
	putchar (CLRSCR_STYLE);
}


//-------------------------------------------------------------------------------------
/** \cond NOT_ENABLED  (This ISR is not to be documented by Doxygen)
 *  This interrupt service routine runs whenever a character has been received by the
 *  first serial port (number 0).  It saves that character into the receiver buffer.
 */

#ifdef USARTC0_RXC_vect
ISR (USARTC0_RXC_vect)
{
	// When this ISR is triggered, there's a character waiting in the USART data reg-
	// ister, and the write index indexes the place where that character should go

	rcvC0_buffer[rcvC0_write_index] = USARTC0.DATA;

	// Increment the write pointer
	if (++rcvC0_write_index >= RSINT_BUF_SIZE)
		rcvC0_write_index = 0;

	// If the write pointer is now equal to the read pointer, that means we've just
	// overwritten the oldest data. Increment the read pointer so that it doesn't seem
	// as if the buffer is empty
	if (rcvC0_write_index == rcvC0_read_index)
		if (++rcvC0_read_index >= RSINT_BUF_SIZE)
			rcvC0_read_index = 0;
}
#endif

#ifdef USARTC1_RXC_vect
ISR (USARTC1_RXC_vect)
{
	// When this ISR is triggered, there's a character waiting in the USART data reg-
	// ister, and the write index indexes the place where that character should go

	rcvC1_buffer[rcvC1_write_index] = USARTC1.DATA;

	// Increment the write pointer
	if (++rcvC1_write_index >= RSINT_BUF_SIZE)
	rcvC1_write_index = 0;

	// If the write pointer is now equal to the read pointer, that means we've just
	// overwritten the oldest data. Increment the read pointer so that it doesn't seem
	// as if the buffer is empty
	if (rcvC1_write_index == rcvC1_read_index)
	if (++rcvC1_read_index >= RSINT_BUF_SIZE)
	rcvC1_read_index = 0;
}
#endif

#ifdef USARTD0_RXC_vect
ISR (USARTD0_RXC_vect)
{
	// When this ISR is triggered, there's a character waiting in the USART data reg-
	// ister, and the write index indexes the place where that character should go

	rcvD0_buffer[rcvC0_write_index] = USARTD0.DATA;

	// Increment the write pointer
	if (++rcvD0_write_index >= RSINT_BUF_SIZE)
	rcvD0_write_index = 0;

	// If the write pointer is now equal to the read pointer, that means we've just
	// overwritten the oldest data. Increment the read pointer so that it doesn't seem
	// as if the buffer is empty
	if (rcvD0_write_index == rcvD0_read_index)
	if (++rcvD0_read_index >= RSINT_BUF_SIZE)
	rcvD0_read_index = 0;
}
#endif

#ifdef USARTD1_RXC_vect
ISR (USARTD1_RXC_vect)
{
	// When this ISR is triggered, there's a character waiting in the USART data reg-
	// ister, and the write index indexes the place where that character should go

	rcvD1_buffer[rcvD1_write_index] = USARTD1.DATA;

	// Increment the write pointer
	if (++rcvD1_write_index >= RSINT_BUF_SIZE)
	rcvD1_write_index = 0;

	// If the write pointer is now equal to the read pointer, that means we've just
	// overwritten the oldest data. Increment the read pointer so that it doesn't seem
	// as if the buffer is empty
	if (rcvD1_write_index == rcvD1_read_index)
	if (++rcvD1_read_index >= RSINT_BUF_SIZE)
	rcvD1_read_index = 0;
}
#endif

#ifdef USARTE0_RXC_vect
ISR (USARTE0_RXC_vect)
{
	// When this ISR is triggered, there's a character waiting in the USART data reg-
	// ister, and the write index indexes the place where that character should go

	rcvE0_buffer[rcvE0_write_index] = USARTE0.DATA;

	// Increment the write pointer
	if (++rcvE0_write_index >= RSINT_BUF_SIZE)
	rcvE0_write_index = 0;

	// If the write pointer is now equal to the read pointer, that means we've just
	// overwritten the oldest data. Increment the read pointer so that it doesn't seem
	// as if the buffer is empty
	if (rcvE0_write_index == rcvE0_read_index)
	if (++rcvE0_read_index >= RSINT_BUF_SIZE)
	rcvE0_read_index = 0;
}
#endif

#ifdef USARTE1_RXC_vect
ISR (USARTE1_RXC_vect)
{
	// When this ISR is triggered, there's a character waiting in the USART data reg-
	// ister, and the write index indexes the place where that character should go

	rcvE1_buffer[rcvE1_write_index] = USARTE1.DATA;

	// Increment the write pointer
	if (++rcvE1_write_index >= RSINT_BUF_SIZE)
	rcvE1_write_index = 0;

	// If the write pointer is now equal to the read pointer, that means we've just
	// overwritten the oldest data. Increment the read pointer so that it doesn't seem
	// as if the buffer is empty
	if (rcvE1_write_index == rcvE1_read_index)
	if (++rcvE1_read_index >= RSINT_BUF_SIZE)
	rcvE1_read_index = 0;
}
#endif

#ifdef USARTF0_RXC_vect
ISR (USARTF0_RXC_vect)
{
	// When this ISR is triggered, there's a character waiting in the USART data reg-
	// ister, and the write index indexes the place where that character should go

	rcvF0_buffer[rcvF0_write_index] = USARTF0.DATA;

	// Increment the write pointer
	if (++rcvF0_write_index >= RSINT_BUF_SIZE)
	rcvF0_write_index = 0;

	// If the write pointer is now equal to the read pointer, that means we've just
	// overwritten the oldest data. Increment the read pointer so that it doesn't seem
	// as if the buffer is empty
	if (rcvF0_write_index == rcvF0_read_index)
	if (++rcvF0_read_index >= RSINT_BUF_SIZE)
	rcvF0_read_index = 0;
}
#endif