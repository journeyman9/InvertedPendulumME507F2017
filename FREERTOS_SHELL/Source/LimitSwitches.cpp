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

LimitSwitches::LimitSwitches(const char* a_name,
								unsigned portBASE_TYPE a_priority,
								size_t a_stack_size,
								emstream* p_ser_dev
								)
								
	// Call the parent (task base) constructor
	: frt_task (a_name, a_priority, a_stack_size, p_ser_dev)
	{
		// Nothing to do in this constructor other than call the parent constructor
	}

void LimitSwitches::run(void){
	// Make a variable which will hold times to use for precise task scheduling
	portTickType previousTicks = xTaskGetTickCount ();
	
	// Setup pins for Limit Switch (PD0 & PD1) and LED output
	PORTD.DIRCLR = PIN0_bm;									// set D0 as input left limit
	PORTD.DIRCLR = PIN2_bm;									// set D2 as input right limit
	PORTD.PIN0CTRL = PORT_OPC_PULLUP_gc;					// set D0 as pullup
	PORTD.PIN2CTRL = PORT_OPC_PULLUP_gc;					// set D2 as pullup
	
	bool rightLimit;
	bool leftLimit;
		
	while(1){
		
		if(!(PORTD_IN & PIN0_bm))							// check whether limit is pressed (pin D0 is high)
		{	
			leftLimit = 1;
			*p_serial << "leftLimit: " << leftLimit << endl;
			
		}
		else if (!(PORTD_IN & PIN2_bm))						// check whether limit is pressed (pin D1 is high)
		{
			rightLimit = 1;
			*p_serial << "rightLimit: " << rightLimit << endl;
		}
		else
		{
			rightLimit = 0;
			leftLimit = 0;
			//*p_serial << "limits: " << rightLimit << leftLimit << endl;
		}
		
		// Increment counter for debugging
		runs++;
		
		//*p_serial << "Econder Pulses" << encoder_count << endl;
		
		// set dt
		//_delay_ms(1);
		// This is a method we use to cause a task to make one run through its task
		// loop every N milliseconds and let other tasks run at other times
		delay_from_to (previousTicks, configMS_TO_TICKS (5));
	}	
}
