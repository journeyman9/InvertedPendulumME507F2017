#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Port I/O for SFR's
#include <avr/wdt.h>                        // Watchdog timer header
#include <avr/interrupt.h>					//
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
#include "Motor.h"							// Inverted Pendulum file
#include "EncoderMotor.h"					// Inverted Pendulum file
#include "EncoderPendulum.h"				// Inverted Pendulum file
#include "LimitSwitches.h"					// Inverted Pendulum file
#include "UserInterface.h"					// Inverted Pendulum file
#include "PWMdriver.h"						// Inverted Pendulum file


EncoderMotor::EncoderMotor(void)

	// Call the parent (task base) constructor
	: frt_task (a_name, a_priority, a_stack_size, p_ser_dev)
{
	// Nothing to do in this constructor other than call the parent constructor
}


void EncoderMotor::run (void)
{
	// INIT:
	// Setup quad encoder on pins C2 & C6
	PORTC.DIRCLR = (PIN2_bm | PIN6_bm);							// set C2 & C6 as inputs
	PORTC.PIN2CTRL = PORT_ISC_LEVEL_gc;							// set C2 for level sensing
	PORTC.PIN6CTRL = PORT_ISC_LEVEL_gc;							// set C6 for level sensing
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTC_PIN2_gc;					// set PC2 as Multiplexer for Event Chan 0
	EVSYS.CH0CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;	// enable quad encoder mode with 2-sample filtering
	TCC0.CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH0_gc;			// set TCC0 event action to quad decoding, and event source as Event Chan 0
	TCC0.PER = 0xFFFF;											// usually ticks/rev, but this doesn't matter since we're converting to linear anyway
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;								// start TCC0 with prescaler = 1
	
	uint16_t count;												// contains the current encoder value
	
	
	// LOOP (state transition loop typically goes in here):
	while(1)
	{
		// Read value from hardware counter
		count = TCC0.CNT;
		
		if(zero = true) // (just a placeholder parameter name) - checks if the "zero" flag is set by some other task (like when the limit switch is triggered)
		{
			// Reset ticks to 0 (there may be a better way to do this)
			TCC0.CNT = 0;
			
			// Reset the flag
			zero = false;
		}
		
		// Increment counter for debugging
		runs++;
		
		/*
		// This is a method we use to cause a task to make one run through its task
		// loop every N milliseconds and let other tasks run at other times
		delay_from_to (previousTicks, configMS_TO_TICKS (1));
		*/
	}
}