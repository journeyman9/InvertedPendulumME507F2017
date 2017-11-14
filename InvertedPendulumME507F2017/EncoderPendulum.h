// Prevent this .h file from being included multiple times in a .cpp file
#ifndef ENCODERPENDULUM_H
#define ENCODERPENDULUM_H

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


class EncoderPendulum : public frt_task
{
	protected:
	
	public:
	// Constructor creates a motor encoder task object
	EncoderPendulum(const char*, unsigned portBASE_TYPE, size_t, emstream*);
	
	// This gets called by the RTOS once to start this task's state loop
	void run(void);
};

#endif