//*************************************************************************************
/** \file lab1_main.cpp
 *    This file contains the main() code for a program which runs a port of the FreeRTOS
 *    for AVR devices. This port is specific to the XMEGA family.
 *
 *  Revisions:
 *    \li 09-14-2017 CTR Adapted from JRR code for AVR to be compatible with xmega 
 *
 *  License:
 *    This file is released under the Lesser GNU Public License, version 2. This 
 *    program is intended for educational use only, but it is not limited thereto. 
 */
//*************************************************************************************


#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Port I/O for SFR's
#include <avr/wdt.h>                        // Watchdog timer header
#include <avr/interrupt.h>
#include <string.h>                         // Functions for C string handling

#include "FreeRTOS.h"                       // Primary header for FreeRTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // FreeRTOS inter-task communication queues
#include "croutine.h"                       // Header for co-routines and such

#include "rs232int.h"                       // ME405/507 library for serial comm.
#include "time_stamp.h"                     // Class to implement a microsecond timer
#include "frt_task.h"                       // Header of wrapper for FreeRTOS tasks
#include "frt_text_queue.h"                 // Wrapper for FreeRTOS character queues
#include "frt_queue.h"                      // Header of wrapper for FreeRTOS queues
#include "frt_shared_data.h"                // Header for thread-safe shared data
#include "shares.h"                         // Global ('extern') queue declarations

#include "task_user.h"                      // Header for user interface task

#include "util/delay.h"						// Header for delay

volatile int counter;
frt_text_queue print_ser_queue (32, NULL, 10);

/*! \brief CCP write helper function written in assembly.
 *
 *  This function is written in assembly because of the time critical
 *  operation of writing to the registers.
 *
 *  \param address A pointer to the address to write to.
 *  \param value   The value to put in to the register.
 */
void CCPWrite( volatile uint8_t * address, uint8_t value )
{
	#if defined __GNUC__
	uint8_t volatile saved_sreg = SREG;
	cli();
	volatile uint8_t * tmpAddr = address;
	#ifdef RAMPZ
	RAMPZ = 0;
	#endif
	asm volatile(
	"movw r30,  %0"	      "\n\t"
	"ldi  r16,  %2"	      "\n\t"
	"out   %3, r16"	      "\n\t"
	"st     Z,  %1"       "\n\t"
	:
	: "r" (tmpAddr), "r" (value), "M" (0xD8), "i" (&CCP)
	: "r16", "r30", "r31"
	);

	SREG = saved_sreg;
	#endif
}


//=====================================================================================
/** The main function sets up the RTOS.  Some test tasks are created. Then the 
 *  scheduler is started up; the scheduler runs until power is turned off or there's a 
 *  reset.
 *  @return This is a real-time microcontroller program which doesn't return. Ever.
 */

int main (void)
{
	//Clear any interrupts
	cli();
	// Configure the system clock
	{	
		// Enable the 32MHz internal RC oscillator and the external 32KHz oscillator  <-----------------------MAY NEED TO CHANGE FOR 16MHz 
		OSC.CTRL |= (1 << OSC_RC32MEN_bp);
		do {} while((OSC.STATUS & (1 << OSC_RC32MRDY_bp)) != (1 << OSC_RC32MRDY_bp));

		// Select the clock
		CCPWrite(&(CLK.CTRL),((CLK.CTRL & ~CLK_SCLKSEL_gm) | (1 << CLK_SCLKSEL0_bp)));
		
		// Disable the 2MHz internal RC oscillator
		OSC.CTRL &= ~(1 << OSC_RC2MEN_bp);
	}
	
	// Disable the watchdog timer unless it's needed later. This is important because
	// sometimes the watchdog timer may have been left on...and it tends to stay on	 
	wdt_disable ();

	// Configure a serial port which can be used by a task to print debugging infor-
	// mation, or to allow user interaction, or for whatever use is appropriate.  The
	// serial port will be used by the user interface task after setup is complete and
	// the task scheduler has been started by the function vTaskStartScheduler()
	rs232 ser_dev(0,&USARTC0); // Create a serial device on USART E0 with always baud = 115200
	ser_dev << clrscr << "FreeRTOS Xmega Testing Program" << endl << endl;
	
	// The user interface is at low priority; it could have been run in the idle task
	// but it is desired to exercise the RTOS more thoroughly in this test program
	//new task_user ("UserInt", task_priority (0), 260, &ser_dev); <-----------------------------------------commented out to run things in main
	// Enable high level interrupts and global interrupts
	PMIC_CTRL = (1 << PMIC_HILVLEN_bp | 1 << PMIC_MEDLVLEN_bp | 1 << PMIC_LOLVLEN_bp);
	sei();
	
	// Here's where the RTOS scheduler is started up. It should never exit as long as
	// power is on and the microcontroller isn't rebooted
	//vTaskStartScheduler (); <------------------------------------------------------------------------------commented out to run things in main
	
	PORTC.DIRCLR = PIN0_bm | PIN1_bm;										// Set both CHa and CHb for input
	PORTC.PIN0CTRL |= PORT_ISC_LEVEL_gc;									// Set low level sense for Cha
	PORTC.PIN1CTRL |= PORT_ISC_LEVEL_gc;									// Set low level sense for Chb
		
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTC_PIN0_gc;								// Configure CHa as a multiplexer input for event channel 0
	EVSYS.CH0CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;				// Enable the quadrature encoder
		
	TCC0.CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH0_gc;						// Set the quadrature decoding as the event action for the timer
	TCC0.PER = 0xFFFF;														// Set the timer counter period 1000 cpr, = 1000*4-1 F9F
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;											// Start the timer
	
	uint16_t encoder_count;
	uint16_t last_encoder_count;
	float AngularPositionCalc;
	int16_t AngularPosition;
	float dt = .001;
	float AngularVelocityCalc;
	int16_t AngularVelocity;
	float x_calc;
	int16_t x;
	

	while(1){
		encoder_count = TCC0.CNT;											// get count
		
		AngularPositionCalc = (encoder_count/(4.00000*1000.00000))*360;		// convert to position [deg], quadrature = 4, cpr = 1000. (encoder_count/(4*1000))*360
		AngularPosition = AngularPositionCalc;
		//ser_dev << "Angular Position: " << AngularPosition << " [deg]" << endl;
		
		x_calc = encoder_count*3/100;		// PPMM  = (4*1000)/(pi*38)
		x = x_calc;															// convert to linear position [mm]
		//ser_dev << "Linear Position: " << x << " [mm]" << endl;
		
		AngularVelocityCalc = ((encoder_count-last_encoder_count)*60/(4.00000*1000.00000))/dt;	// convert to velocity [RPM]
		AngularVelocity = AngularVelocityCalc;
		//ser_dev << "Angular Velocity: " << AngularVelocity << " [RPM]" << endl;
		
		last_encoder_count = encoder_count;									// make present encoder_count the previous for the next calculation
		
		// set dt
		_delay_ms(1);

	}
}