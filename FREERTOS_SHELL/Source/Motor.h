#ifndef MOTOR_H
#define MOTOR_H

#include <stdlib.h>                         // Prototype declarations for I/O functions

#include "FreeRTOS.h"                       // Primary header for FreeRTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // FreeRTOS inter-task communication queues

#include "rs232int.h"                       // ME405/507 library for serial comm.
#include "time_stamp.h"                     // Class to implement a microsecond timer
#include "frt_task.h"                       // Header for ME405/507 base task class
#include "frt_queue.h"                      // Header of wrapper for FreeRTOS queues
#include "frt_text_queue.h"                 // Header for a "<<" queue class
#include "frt_shared_data.h"                // Header for thread-safe shared data

#include "shares.h"                         // Global ('extern') queue declarations

#include "math.h"

class Motor : public frt_task{
	protected:
				volatile int xm_set;
				volatile int xm_error;
				volatile int xm_measured;
				volatile int radius;
				volatile int omegam_set;
				volatile int vm_set;
				volatile int omegam_measured;
				volatile int anglem_measured;
				volatile int Tset;
				volatile int Im_set;
				volatile int Voltm_set;
				volatile int K_T;
				volatile int I_motor;
				volatile int I_error;
				volatile int I_actuator;
				volatile int V_R;
				volatile int K_p;
				volatile int V_E;
				volatile int K_v;
				volatile int V_m;
				volatile int i_error_sum;
	public:
		// Constructor creates a motor encoder task object
		Motor(const char*, unsigned portBASE_TYPE, size_t, emstream*);
		//Motor(int omegam_measured, int I_actuator);
	
		// This gets called by the RTOS once to start this task's state loop
		void run(void);
	
};

#endif



