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

task_Motor::task_Motor(const char* a_name,
								unsigned portBASE_TYPE a_priority,
								size_t a_stack_size,
								emstream* p_ser_dev
								)
								
	// Call the parent (task base) constructor
	: frt_task (a_name, a_priority, a_stack_size, p_ser_dev)
	{
		// Nothing to do in this constructor other than call the parent constructor
	}

void task_Motor::run(void){
	// Make a variable which will hold times to use for precise task scheduling
	portTickType previousTicks = xTaskGetTickCount ();

	dt = 5; // [ms]
	inc = 1;
	
	while(1){
		// Increment counter for debugging
		runs++;
		
		// Actual motor code
		// Right now just working with speed control for motor.
		// Previously commented code extends to be a position control.
		
		omegam_set = 100; // [ticks/ms]

		// omegam_measured will be the derivative of theta_measured from the encoder
		omegam_measured = thdMotor.get();
		//*p_serial << "Measured: " << omegam_measured << endl;
		
		// PID to get Tset from Omegam_set with a max torque value
		// PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
		// PID pidTorque = PID(1, 1600, -1600, 10, 0, 1); // PID output
		//uint16_t error_sum = pidTorque.get_Integral();
		
		_Kp = 7;
		_Ki = .7*256;
		_Kd = 0;
		antiwind_gain = .95*256;
		
		_max = 1600;
		_min = -1600;
		
		// Calculate error
		int32_t error = omegam_set - omegam_measured;
		
		
		// Proportional term
		//int16_t Pout = ((_Kp) * error);
		Pout = ssmul(_Kp,error);

		// Integral term
		error_int = error - antiwind_correct;
		error_int_gain = (_Ki * error_int);
		_integral += (error_int_gain * dt)/256;
		if(_integral < 1000000000)
		{
			_integral = _integral;
		}
		else if(_integral > 1000000000)
		{
			_integral = 1000000000;
		}
		//_integral = ssadd((error*dt), _integral);
		
		// Restrict to _integral term
		/*
		arbitraryNumber = 32767;
		if( _integral > arbitraryNumber )
		_integral = arbitraryNumber;
		else if( _integral < arbitraryNumber )
		_integral = arbitraryNumber;
		*/

		// Derivative term
		int16_t derivative = (error - _pre_error) / dt;
		int16_t Dout = _Kd * derivative;

		// Calculate total output	
		// int16_t output = Pout + Iout + Dout;
		output = ssadd(Pout, _integral);

		output_correct = output;
		// Restrict to max/min
		if( output_correct > _max )
		output_correct = _max;
		else if( output_correct < _min )
		output_correct = _min;

		// Save error to previous error
		_pre_error = error;
		
		// Anti-windup correction
		antiwind_error = output - output_correct;
		
		antiwind_correct = (antiwind_error*antiwind_gain)/256;
		
		
			if(runs%5==0){
				//*p_serial << "Ierror: " << Iout << endl;
				//*p_serial << "Pout: " << Pout << endl;
				//*p_serial << "error: " << error << endl;
				//*p_serial << "Integral: " << _integral << endl;
				//*p_serial << "Measured: " << omegam_measured << endl;
				//*p_serial << "PWM Signal: " << output_correct << endl;
				//*p_serial << omegam_measured << endl;
			}
		
		if (leftLimitSwitch.get() || rightLimitSwitch.get())
		{
			omegam_set = 0; // [ticks/ms]
			//Pout = 0;
			//Iout = 0;
			_integral = 0;
			output_correct = 0;
			//omegam_measured = 0; // [ticks/ms]
			//*p_serial << "Left" << leftLimitSwitch.get() << endl;
			//*p_serial << "Right" << rightLimitSwitch.get() << endl;
		}
		else
		{
		//omegam_set = omegam_set; // [ticks/ms]
		}
		
		// int16_t Tset = (pidTorque.calculate(omegam_set, omegam_measured));
		PWMvalue.put(output_correct);
		
		/*
			if(runs%100==0){
				*p_serial << "CONTROLLER OUTPUT: " << Tset << endl;
				*p_serial << omegam_measured << endl;
			}
		*/		
		

		
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
		
		delay_from_to (previousTicks, configMS_TO_TICKS (dt));
		
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
