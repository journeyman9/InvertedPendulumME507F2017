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


Motor::Motor(const char* a_name,
								unsigned portBASE_TYPE a_priority,
								size_t a_stack_size,
								emstream* p_ser_dev
								)
								
	// Call the parent (task base) constructor
	: frt_task (a_name, a_priority, a_stack_size, p_ser_dev)
	{
		// Nothing to do in this constructor other than call the parent constructor
	}

void Motor::run(void){
	// Make a variable which will hold times to use for precise task scheduling
	portTickType previousTicks = xTaskGetTickCount ();

	dt = .008;
	inc = 1;
	
	while(1){
		// Increment counter for debugging
		runs++;
		
		// Actual motor code
		// Right now just working with speed control for motor.
		// Previously commented code extends to be a position control.
		omegam_set = 2;

		// omegam_measured will be the derivative of theta_measured from the encoder
		omegam_measured = 0;
		
		// PID to get Tset from Omegam_set with a max torque value
		// PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
		PID pidTorque = PID(0.1, 1.1225, -1.1225, 0.1, 0.01, 0.5);
		double Tset = pidTorque.calculate(omegam_set, omegam_measured);
		//printf("val:% 7.3f inc:% 7.3f\n", omegam_measured, inc);
		omegam_measured += inc;

		K_T = 0.065; // Nm/A. Taken from Pittman 14203 series motor documentation page G 21
		Im_set = Tset/K_T;

		// Saturater for current
		
		if(Im_set > 17.4) {
			Im_set = 17.4;
		} else if(Im_set < -17.4) {
			Im_set = -17.4;
		}
		
		// This will be measured from the board
		//I_actuator = 0;

		//I_error = Im_set - I_actuator;

		// This sums the I_error. The [Kp/(Ti S)] block in diagram
		//i_error_sum = i_error_sum + I_error;

		K_v = 0.065; // V/rad/s. Taken from Pittman 14203 series motor documentation page G 21
		K_p = K_T*(1/1.21E-5)*K_v; // (Nm/A)([rad/s]/Nm)(V/[rad/s]) so (V/A)
		//V_R = i_error_sum - K_p*I_actuator;

		V_E = K_v*omegam_measured;

		V_m = V_R + V_E;

		// Saturater for voltage
		if (V_m > 24) {
			V_m = 24;
		} else if(V_m < -24) {
			V_m = -24;
		}
		
		// V_m then feeds into PWM function to command motor

		// set dt
		// This is a method we use to cause a task to make one run through its task
		// loop every N milliseconds and let other tasks run at other times
		delay_from_to (previousTicks, configMS_TO_TICKS (1));
		
	}
}
/*
Motor::Motor(int omegam_measured, int I_actuator)
{

//	int integrator(int time, int data)
//	{
//		
//		integral = 
//	}
	
	while(1)
	{
//	xm_set = 100; // xm_set is in millimeters and is the desired distance of the slider away from motor
//	xm_measured = 2; // xm_measured is read from the encoder and converted from angle of the motor to position of the using the radius
//	xm_error = xm_set - xm_measured;  // this is the position error from the set value and the measured value

//	radius = 26; // radius is the radius of the pulley that is connected directly to the motor

//  vm_set = derivative(xm_set);
//	vm_set = 3;
//	omegam_set = vm_set/radius; // the desired angular velocity of the motor
    
	// Right now just working with speed control for motor.
	// Previously commented code extends to be a position control.
	omegam_set = 2;	

	// omegam_measured will be the derivative of theta_measured from the encoder
	omegam_measured = 0;
	
	// integrating omegam_measured to get anglem_measured
	// Can we assume that the time we're integrating over is unit time steps???
//	anglem_measured = omegam_measured + anglem_measured;

	// PID to get Tset from Omegam_set
	PID pidTorque = PID(0.1, 1.1225, -1.1225, 0.1, 0.01, 0.5);
//	for (int i = 0; i < 100; i++)
//	{
		double Tset = pidTorque.calculate(omegam_set, omegam_measured);
		printf("val:% 7.3f inc:% 7.3f\n", omegam_measured, inc);
		omegam_measured += inc;
//	}

	K_T = 0.065; // Nm/A. Taken from Pittman 14203 series motor documentation page G 21
	Im_set = Tset/K_T;

	// Saturater for current
	if (Im_set > 17.4) {
        Im_set = 17.4;
	}
	else (Im_set < -17.4) {
        Im_set = -17.4;
	}

	// This will be measured from the board
	I_actuator = 0;

	I_error = Im_set - I_actuator;

	// This sums the I_error. The [Kp/(Ti S)] block in diagram
	i_error_sum = i_error_sum + I_error;

	K_v = 0.065; // V/rad/s. Taken from Pittman 14203 series motor documentation page G 21
	K_p = Kt*(1/1.21E-5)*K_v; // (Nm/A)([rad/s]/Nm)(V/[rad/s]) so (V/A)
	V_R = i_error_sum - K_p*I_actuator;

	V_E = K_v*omegam_measured;

	V_m = V_R + V_E;

	// Saturater for voltage
	if (V_m > 24) {
        V_m = 24;
	}
    else (V_m < -24) {
        V_m = -24;
    }

	// V_m then feeds into PWM function to command motor
	
	}
}
*/