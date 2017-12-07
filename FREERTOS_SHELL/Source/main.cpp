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
#include <stdbool.h>

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

#include "EncoderMotor.h"					// Header for Encoder of Motor
#include "LimitSwitches.h"					// Header for Limit Switches
#include "Motor.h"							// Inverted Pendulum file
#include "EncoderPendulum.h"				// Inverted Pendulum file
#include "PWMdriver.h"						// Inverted Pendulum file
#include "pid.h"							// Inverted Pendulum file

volatile int counter;
frt_text_queue print_ser_queue (32, NULL, 10);

shared_data<int16_t> linear_position;		// Establish shared variable type
shared_data<int16_t> thMotor;
shared_data<int16_t> thdMotor;
shared_data<int16_t> thPendulum;
shared_data<bool> leftLimitSwitch;
shared_data<bool> rightLimitSwitch;
shared_data<int16_t> PWMvalue;

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
	// Set up pins for LED output
	PORTD.DIRSET = PIN6_bm;									// set pin D4 as output LED1: turns on when power is on.
	PORTD.OUTSET = PIN6_bm;									// set pin high, LED 1
	
	//PORTD.DIRSET = PIN4_bm;									// set pin D5 as output LED2
	//PORTD.DIRSET = PIN5_bm;									// set pin D6 as output LED3
	//PORTD.OUTSET = PIN4_bm;									// set pin high, LED 2
	//PORTD.OUTSET = PIN5_bm;									// set pin high, LED 3
	
	//Clear any interrupts
	cli();
	// Configure the system clock
	{	
		// Enable the 32MHz internal RC oscillator
		OSC.CTRL |= (1 << OSC_RC32MEN_bp);				// 32 MHz
		do {} while((OSC.STATUS & (1 << OSC_RC32MRDY_bp)) != (1 << OSC_RC32MRDY_bp));

		//Enable PLL

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
	rs232 ser_dev(0,&USARTC1); // Create a serial device on USART C0 with always baud = 115200
	ser_dev << clrscr << "FreeRTOS XMEGA Testing Program" << endl << endl;
	
	// The user interface is at low priority; it could have been run in the idle task
	// but it is desired to exercise the RTOS more thoroughly in this test program
	new task_user ("UserInt", task_priority (0), 260, &ser_dev);
	
	// The Encoder Motor task is a high priority and is used for controlling the cart
	// to ensure centering
	new EncoderMotor ("EncMtr", task_priority(2), 260, &ser_dev);

	// The EncoderPendulum task is a high priority and is used for controlling the balance of
	//the pendulum
	//new EncoderPendulum ("EncPen", task_priority(3), 260, &ser_dev);
	
	// The LimitSwitches task is an extremely high priority to kill the motor if either
	// are hit.
	new LimitSwitches ("LimSwtch", task_priority(4), 260, &ser_dev);

	// The Motor task sets velocity, position, then maybe current control
	new Motor ("Motor", task_priority(3), 260, &ser_dev);
	
	new PWMdriver ("PWM", task_priority(5), 260, &ser_dev);
	
	// Enable high level interrupts and gl;obal interrupts
	PMIC_CTRL = (1 << PMIC_HILVLEN_bp | 1 << PMIC_MEDLVLEN_bp | 1 << PMIC_LOLVLEN_bp);
	sei();
	
	// Here's where the RTOS scheduler is started up. It should never exit as long as
	// power is on and the microcontroller isn't rebooted
	vTaskStartScheduler ();
	
	
	return 0;
}