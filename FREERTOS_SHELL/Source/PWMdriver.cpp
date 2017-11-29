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


PWMdriver::PWMdriver(const char* a_name,
								unsigned portBASE_TYPE a_priority,
								size_t a_stack_size,
								emstream* p_ser_dev
								)
								
	// Call the parent (task base) constructor
	: frt_task (a_name, a_priority, a_stack_size, p_ser_dev)
	{
		// Nothing to do in this constructor other than call the parent constructor
	}



void PWMdriver::run(void){
	// Make a variable which will hold times to use for precise task scheduling
	portTickType previousTicks = xTaskGetTickCount ();

	//PC0 - MD0
	//PC1 - MD1
		
	PORTC.DIRSET = PIN0_bm | PIN1_bm;					// Configure PC0 and PC1 as outputs
	TCD0.CTRLA = TC0_CLKSEL0_bm;						// Configures Clock select bits for divide by 1
	TCD1.CTRLB = TC0_WGMODE0_bm | TC0_WGMODE1_bm;		// Configures waveform generation mode to single slope PWM
	TCD1.PER = 16;										// Configures period to be 16 counts for a pwm freq 2kHz
	TCD1.CCA = 0;										// Ensure channel A is off when enabled
	TCD1.CCB  = 0;										// Ensure channel B is off when enabled
	
	TCD1.CTRLB |= TC0_CCAEN_bm | TC0_CCBEN_bm;			// Enable output compare on channels A and B

	while(1){
		// Increment counter for debugging
		runs++;
		
		
		TCD1.CCA = 6;
		//TCD1.CCB = 6;
		// This is a method we use to cause a task to make one run through its task
		// loop every N milliseconds and let other tasks run at other times
		//delay_from_to (previousTicks, configMS_TO_TICKS (28));
		
		
	}	
}

