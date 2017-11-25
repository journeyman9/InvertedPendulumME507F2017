//**************************************************************************************
/** \file task_user.cpp
 *    This file contains source code for a user interface task for a ME405/FreeRTOS
 *    test suite. 
 *
 *  Revisions:
 *    \li 09-30-2012 JRR Original file was a one-file demonstration with two tasks
 *    \li 10-05-2012 JRR Split into multiple files, one for each task
 *    \li 10-25-2012 JRR Changed to a more fully C++ version with class task_user
 *    \li 11-04-2012 JRR Modified from the data acquisition example to the test suite
 *
 *  License:
 *    This file is copyright 2012 by JR Ridgely and released under the Lesser GNU 
 *    Public License, version 2. It intended for educational use only, but its use
 *    is not limited thereto. */
/*    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 *    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUEN-
 *    TIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
 *    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 *    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 *    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//**************************************************************************************

#include <avr/io.h>                         // Port I/O for SFR's
#include <avr/wdt.h>                        // Watchdog timer header

#include "shared_data_sender.h"
#include "shared_data_receiver.h"
#include "task_LED.h"                      // Header for this file


//-------------------------------------------------------------------------------------
/** This constructor creates a new data acquisition task. Its main job is to call the
 *  parent class's constructor which does most of the work.
 *  @param a_name A character string which will be the name of this task
 *  @param a_priority The priority at which this task will initially run (default: 0)
 *  @param a_stack_size The size of this task's stack in bytes 
 *                      (default: configMINIMAL_STACK_SIZE)
 *  @param p_ser_dev Pointer to a serial device (port, radio, SD card, etc.) which can
 *                   be used by this task to communicate (default: NULL)
 */

task_LED::task_LED (const char* a_name, 
					  unsigned portBASE_TYPE a_priority, 
					  size_t a_stack_size,
					  emstream* p_ser_dev
					 )
	: frt_task (a_name, a_priority, a_stack_size, p_ser_dev)
{
	// Nothing is done in the body of this constructor. All the work is done in the
	// call to the frt_task constructor on the line just above this one
}


//-------------------------------------------------------------------------------------
/** This task blinks an LED attached to PORTR Pin 1
 */

void task_LED::run (void)
{
	// Make a variable which will hold times to use for precise task scheduling
	portTickType previousTicks = xTaskGetTickCount ();
	
	
	// Configure PORT C to be used for PWM output on PC0-PC2 and for bit toggling on PC3
	PORTC.OUT &= ~(1 << 0 | 1 << 1 | 1 << 2 | 1 << 3); // Clear Bits 0-2 on port c
	PORTC.DIR |= (1 << 0 | 1 << 1 | 1 << 2 | 1 << 3); // Set Bits 0-2 on port c as outputs
	
	// Configure Timer Counter C0 for center-aligned 64KHz PWM on ChA ChB and ChC (PC0-PC2)
	// and enable interrupts on ovf so that a new duty cycle may be selected
	TCC0.CTRLA = (1 << TC0_CLKSEL0_bp); // Select a pre-scaler of 1
	TCC0.CTRLB = (1 << TC0_CCAEN_bp | 1 << TC0_CCBEN_bp | 1 << TC0_CCCEN_bp | 0b101 << TC0_WGMODE0_bp); // Enable channels A, B, and C and Configure for Dual Slope PWM
	TCC0.CTRLD = 0;
	TCC0.CTRLE = 0;
	TCC0.INTCTRLA |= (0b11 << TC0_OVFINTLVL0_bp); // Enable high level interrupts on ovf

	// Set the duty cycles all to 0 and set the period to 256 clock ticks; this will be the PWM resolution
	TCC0.CCABUF = 0x00;
	TCC0.CCBBUF = 0x00;
	TCC0.CCCBUF = 0x00;
	TCC0.PERBUF = 0xFF; // Set Period to 256 clock ticks (64KHz pwm)
	
	//TCC1.CTRLA = (0 << TC1_CLKSEL0_bp | 0 << TC1_CLKSEL1_bp | 1 << TC1_CLKSEL2_bp);

	// Wait a little while for user interface task to finish up
	delay_ms(10);
	
	// Print the header for the PWM output table
	*p_serial << endl << "t\tang\tA\tB\tC\t" << endl;
	

	while(1)
	{


		
		//// Print a table of duty cycles
		//if (runs<256)
		//{
			////*p_serial << runs << endl;
			//time_queue.put((uint16_t) runs);
			//ang_queue.put((int16_t) angle);
			//A_queue.put(A_duty);
			//B_queue.put(B_duty);
			//C_queue.put(C_duty);
		//}
		//
		//if(A_queue.not_empty() && runs >= 256)
		//{
			//*p_serial << time_queue.get() << "\t" << ang_queue.get() << "\t" << A_queue.get() << "\t"  << B_queue.get() << "\t"  << C_queue.get() << endl;	
		//}
		
		// Increment the run counter. This counter belongs to the parent class and can
		// be printed out for debugging purposes
		runs++;

//		*p_serial << (TCE1_CNT - duration) << endl;
		
		// This is a method we use to cause a task to make one run through its task
		// loop every N milliseconds and let other tasks run at other times
		delay_from_to (previousTicks, configMS_TO_TICKS (1));
		//delay_from_to (previousTicks, 200);
		//delay_ms(1);
	}
}