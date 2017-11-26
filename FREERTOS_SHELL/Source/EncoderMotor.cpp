﻿#include <stdlib.h>                         // Prototype declarations for I/O functions
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
	
	PORTE.DIRCLR = PIN4_bm | PIN5_bm;										// Set both CHa and CHb for input
	PORTE.PIN4CTRL |= PORT_ISC_LEVEL_gc;									// Set low level sense for Cha
	PORTE.PIN5CTRL |= PORT_ISC_LEVEL_gc;									// Set low level sense for Chb
	
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTE_PIN4_gc;								// Configure CHa as a multiplexer input for event channel 1
	EVSYS.CH0CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;				// Enable the quadrature encoder
	
	TCD0.CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH0_gc;						// Set the quadrature decoding as the event action for the timer
	TCD0.PER = 0xFFFF;														// Set the timer counter period 1000 cpr, = 1000*4-1 F9F
	TCD0.CTRLA = TC_CLKSEL_DIV1_gc;											// Start the timer
	
	int16_t encoder_count;
	int16_t last_encoder_count;
	float angularPositionCalc;
	int16_t angularPosition;
	float dt = .001;
	float angularVelocityCalc;
	int16_t angularVelocity;
	float x_calc;
	int16_t x;

	while(1){
		encoder_count = TCD0.CNT;											// get count
		*p_serial << "Encoder Pulses: " << encoder_count << endl;
		
		angularPositionCalc = (encoder_count/(4.00000*1000.00000))*360;		// convert to position [deg], quadrature = 4, cpr = 1000. (encoder_count/(4*1000))*360
		angularPosition = angularPositionCalc;
		//*p_serial << "Angular Position: " << AngularPosition << " [deg]" << endl;
		thMotor.put(angularPosition);
		
		x_calc = ( encoder_count * 3)/100;		// PPMM  = (4*1000)/(pi*38)
		x = x_calc;															// convert to linear position [mm]
		//*p_serial << "Linear Position: " << x << " [mm]" << endl;
		linear_position.put(x);
		//*p_serial << "Linear Position: " << linear_position.get() << " [mm]" << endl;
		
		angularVelocityCalc = ((int16_t) (encoder_count-last_encoder_count))*60/(4.00000*1000.00000)/dt;	// convert to velocity [RPM]
		angularVelocity = angularVelocityCalc;
		//*p_serial << "Angular Velocity: " << AngularVelocity << " [RPM]" << endl;
		thdMotor.put(angularVelocity);
		
		last_encoder_count = encoder_count;									// make present encoder_count the previous for the next calculation
		
		/*
		if(motor_enc_zero = true) // (just a placeholder parameter name) - checks if the "zero" flag is set by some other task (like when the limit switch is triggered)
		{
			// Reset ticks to 0 (there may be a better way to do this)
			TCC0.CNT = 0;
			
			// Reset the flag
			motor_enc_zero = false;
		}
		*/
		
		// Increment counter for debugging
		runs++;
		
		// set dt
		// This is a method we use to cause a task to make one run through its task
		// loop every N milliseconds and let other tasks run at other times
		delay_from_to (previousTicks, configMS_TO_TICKS (5));
		
	}

}