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


task_Velocity::task_Velocity(const char* a_name,
							unsigned portBASE_TYPE a_priority,
							size_t a_stack_size,
							emstream* p_ser_dev
							)
							
	// Call the parent (task base) constructor
	: frt_task (a_name, a_priority, a_stack_size, p_ser_dev)
{
	// Nothing to do in this constructor other than call the parent constructor
}


void task_Velocity::run (void)
{ 
	// Make a variable which will hold times to use for precise task scheduling
	portTickType previousTicks = xTaskGetTickCount ();
	
	while(1){
		
	
		// This is a method we use to cause a task to make one run through its task
		// loop every N milliseconds and let other tasks run at other times
		delay_from_to (previousTicks, configMS_TO_TICKS (2));
		
	}

}