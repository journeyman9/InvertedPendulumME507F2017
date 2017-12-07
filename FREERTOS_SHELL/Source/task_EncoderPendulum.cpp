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

#include "task_user.h"                      // Header for user interface task
#include "task_EncoderMotor.h"				// Header for Encoder of Motor
#include "task_Motor.h"						// Inverted Pendulum file
#include "task_EncoderPendulum.h"			// Inverted Pendulum file
#include "task_Velocity.h"					// Position Loop
#include "task_Position.h"					// Position Loop
#include "task_Angle.h"						// Angle Loop
#include "task_LimitSwitches.h"				// Header for Limit Switches
#include "satmath.h"
#include "PWMdriver.h"						// Inverted Pendulum file
#include "pid.h"							// Inverted Pendulum file


task_EncoderPendulum::task_EncoderPendulum(const char* a_name,
								unsigned portBASE_TYPE a_priority,
								size_t a_stack_size,
								emstream* p_ser_dev
								)
								
	// Call the parent (task base) constructor
	: frt_task (a_name, a_priority, a_stack_size, p_ser_dev)
	{
		// Nothing to do in this constructor other than call the parent constructor
	}

void task_EncoderPendulum::run(void){
	// Make a variable which will hold times to use for precise task scheduling
	portTickType previousTicks = xTaskGetTickCount ();
	
	// INIT:
	// Setup quad encoder on pins C4 & C5
	PORTE.DIRCLR = (PIN0_bm | PIN1_bm);							// set E0 & E1 as inputs
	PORTE.PIN0CTRL = PORT_ISC_LEVEL_gc;							// set E0 for level sensing
	PORTE.PIN1CTRL = PORT_ISC_LEVEL_gc;							// set E1 for level sensing
	EVSYS.CH2MUX = EVSYS_CHMUX_PORTE_PIN0_gc;					// set PE0 as Multiplexer for Event Chan 2
	EVSYS.CH2CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;	// enable quad encoder mode with 2-sample filtering
	TCC1.CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH2_gc;			// set TCC1 event action to quad decoding, and event source as Event Chan 1
	TCC1.PER = 0x5A0;											// usually ticks/rev, but this doesn't matter since we're converting to linear anyway
	TCC1.CTRLA = TC_CLKSEL_DIV1_gc;								// start TCC1 with prescaler = 1
	
	int16_t count;												// contains the current encoder value
	int16_t theta_pendulum;
	
	while(1){
		// Read value from hardware counter
		count = TCC1.CNT; 
		//*p_serial << count << endl;
		theta_pendulum = ( (int32_t) count*100/4);				//count/(4*360)*360 degrees * 100
		//*p_serial << "thetaPendulum: " <<  theta_pendulum << endl;
		thPendulum.put(theta_pendulum);
		
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
		delay_from_to (previousTicks, configMS_TO_TICKS (3));
	}	
}