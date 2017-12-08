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


EncoderMotor::EncoderMotor(const char* a_name,
							unsigned portBASE_TYPE a_priority,
							size_t a_stack_size,
							emstream* p_ser_dev
							)
							
	// Call the parent (task base) constructor
	: frt_task (a_name, a_priority, a_stack_size, p_ser_dev)
{
	// Nothing to do in this constructor other than call the parent constructor
}


void EncoderMotor::run (void)
{ 
	// Make a variable which will hold times to use for precise task scheduling
	portTickType previousTicks = xTaskGetTickCount ();
	
	PORTE.DIRCLR = PIN2_bm | PIN3_bm;										// Set both CHa and CHb for input
	PORTE.PIN2CTRL |= PORT_ISC_LEVEL_gc;									// Set low level sense for Cha
	PORTE.PIN3CTRL |= PORT_ISC_LEVEL_gc;									// Set low level sense for Chb
	
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTE_PIN2_gc;								// Configure CHa as a multiplexer input for event channel 0
	EVSYS.CH0CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;				// Enable the quadrature encoder
	
	TCD0.CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH0_gc;						// Set the quadrature decoding as the event action for the timer
	TCD0.PER = 0xFFFF;														// Set the timer counter period 1000 cpr, = 1000*4-1 F9F
	TCD0.CTRLA = TC_CLKSEL_DIV1_gc;											// Start the timer
	
	int16_t encoder_count;
	int16_t last_encoder_count;
	int16_t angularPosition;
	uint8_t dt = 1;															// 1 ms
	int16_t angularVelocity;
	int16_t x;

	while(1){
		
		encoder_count = TCD0.CNT;											// get count
		
		//angularPosition = ( (int32_t) encoder_count*9);						// count/(4*1000)*360 deg * 100
		//thMotor.put(angularPosition);
		
		x = ( (int32_t) encoder_count*3)/100 - linear_offset.get();								// PPMM = (4*1000)/(pi*38)
		linear_position.put(x);
		
		
		int16_t ticks_per_ms = (encoder_count - last_encoder_count); // current angular velocity [ticks/ms]
		thdMotor.put(ticks_per_ms);
				
		angularVelocity = ((int32_t) (encoder_count-last_encoder_count)*15)/dt;	// d_ec*60/(4*1000)/dt where dt is in ms so * 1000
		
		/*
		if(runs%100==0)
		{
			*p_serial << "Encoder Pulses: " << encoder_count << endl;
			*p_serial << "angularPosition: " << angularPosition << endl;		// divide by a hundred to read degrees
			*p_serial << "linearPosition: " << x << " [mm]" << endl;			// x position in mm
			*p_serial << "Ticks_per_ms: " << ticks_per_ms << endl;
			*p_serial << ticks_per_ms << endl;
			*p_serial << "angularVelocity: " << angularVelocity << " [RPM]" << endl;
		}
		*/
		
		last_encoder_count = encoder_count;									// make present encoder_count the previous for the next calculation
		
		// Increment counter for debugging
		runs++;
		
		// set dt
		// This is a method we use to cause a task to make one run through its task
		// loop every N milliseconds and let other tasks run at other times
		delay_from_to (previousTicks, configMS_TO_TICKS (dt));
		
	}

}