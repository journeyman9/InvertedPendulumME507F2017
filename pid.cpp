#ifndef _PID_SOURCE_
#define _PID_SOURCE_

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
#include "util/delay.h"						// Header for delay

#include "EncoderMotor.h"					// Header for this file
#include "Motor.h"							// Inverted Pendulum file
#include "EncoderPendulum.h"				// Inverted Pendulum file
#include "LimitSwitches.h"					// Inverted Pendulum file
#include "PWMdriver.h"						// Inverted Pendulum file
#include "pid.h"							// Inverted Pendulum file

//#include <iostream>
//#include <cmath>

using namespace std;

class PIDImpl
{
    public:
        PIDImpl( int16_t dt, int16_t max, int16_t min, int16_t Kp, int16_t Kd, int16_t Ki );
        ~PIDImpl();
        int16_t calculate( int16_t setpoint, int16_t pv );

    private:
        int16_t _dt;
        int16_t _max;
        int16_t _min;
        int16_t _Kp;
        int16_t _Kd;
        int16_t _Ki;
        int16_t _pre_error;
        int16_t _integral;
		int16_t arbitraryNumber;
};


PID::PID( int16_t dt, int16_t max, int16_t min, int16_t Kp, int16_t Kd, int16_t Ki )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}
int16_t PID::calculate( int16_t setpoint, int16_t pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID() 
{
    delete pimpl;
}


/**
 * Implementation
 */

PIDImpl::PIDImpl( int16_t dt, int16_t max, int16_t min, int16_t Kp, int16_t Kd, int16_t Ki ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
}

int16_t PIDImpl::calculate( int16_t setpoint, int16_t pv )
{
    
    // Calculate error
    int16_t error = setpoint - pv;

    // Proportional term
    int16_t Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
	
	// Restrict to _integral term
	arbitraryNumber = 327676;
	if( _integral > arbitraryNumber )
	_integral = arbitraryNumber;
	else if( _integral < arbitraryNumber )
	_integral = arbitraryNumber;
	
    int16_t Iout = _Ki * _integral;

    // Derivative term
    int16_t derivative = (error - _pre_error) / _dt;
    int16_t Dout = _Kd * derivative;

    // Calculate total output
    int16_t output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif
