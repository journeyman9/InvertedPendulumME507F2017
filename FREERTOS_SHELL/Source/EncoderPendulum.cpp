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

#include "shared_data_sender.h"
#include "shared_data_receiver.h"

#include "EncoderMotor.h"					// Header for this file
#include "Motor.h"							// Inverted Pendulum file
#include "EncoderPendulum.h"				// Inverted Pendulum file
#include "LimitSwitches.h"					// Inverted Pendulum file
#include "PWMdriver.h"						// Inverted Pendulum file
#include "pid.h"							// Inverted Pendulum file


EncoderPendulum::EncoderPendulum(const char* a_name,
								unsigned portBASE_TYPE a_priority,
								size_t a_stack_size,
								emstream* p_ser_dev
								)
								
	// Call the parent (task base) constructor
	: frt_task (a_name, a_priority, a_stack_size, p_ser_dev)
	{
		// Nothing to do in this constructor other than call the parent constructor
	}

void EncoderPendulum::run(void){
	// Make a variable which will hold times to use for precise task scheduling
	portTickType previousTicks = xTaskGetTickCount ();
	
	// INIT:
	// Setup quad encoder on pins C4 & C5
	PORTC.DIRCLR = (PIN4_bm | PIN5_bm);							// set C4 & C5 as inputs
	PORTC.PIN4CTRL = PORT_ISC_LEVEL_gc;							// set C4 for level sensing
	PORTC.PIN5CTRL = PORT_ISC_LEVEL_gc;							// set C5 for level sensing
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTC_PIN4_gc;					// set PC4 as Multiplexer for Event Chan 0
	EVSYS.CH0CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;	// enable quad encoder mode with 2-sample filtering
	TCC1.CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH1_gc;			// set TCC1 event action to quad decoding, and event source as Event Chan 1
	TCC1.PER = 0xFFFF;											// usually ticks/rev, but this doesn't matter since we're converting to linear anyway
	TCC1.CTRLA = TC_CLKSEL_DIV1_gc;								// start TCC1 with prescaler = 1
	
	uint16_t count;												// contains the current encoder value
	
	while(1){
		// Read value from hardware counter
		count = TCC1.CNT;
		
		/*	
		if(pendulum_enc_zero = true) // (just a placeholder parameter name) - checks if the "zero" flag is set by some other task (like when the limit switch is triggered)
			{
			// Reset ticks to 0 (there may be a better way to do this)
			TCC1.CNT = 0;
				
			// Reset the flag
			pendulum_enc_zero = false;
			}
		*/
		
		// Increment counter for debugging
		runs++;
		
		//*p_serial << "Econder Pulses" << encoder_count << endl;
		
		// set dt
		// This is a method we use to cause a task to make one run through its task
		// loop every N milliseconds and let other tasks run at other times
		delay_from_to (previousTicks, configMS_TO_TICKS (1));
	}	
}