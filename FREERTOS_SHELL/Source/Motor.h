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
				int16_t xm_set;
				int16_t xm_error;
				int16_t xm_measured;
				int16_t radius;
				int16_t omegam_set;
				int16_t vm_set;
				int16_t omegam_measured;
				int16_t anglem_measured;
				int16_t Tset;
				int16_t Im_set;
				int16_t Voltm_set;
				int16_t K_T;
				int16_t I_motor;
				int16_t I_error;
				int16_t I_actuator;
				int16_t V_R;
				int16_t K_p;
				int16_t V_E;
				int16_t K_v;
				int16_t V_m;
				int16_t i_error_sum;
				int16_t inc;
				int32_t dt; // ms
				int16_t _dt;
				int16_t _max;
				int16_t _min;
				int16_t _Kp;
				int16_t _Kd;
				int16_t _Ki;
				int16_t _pre_error;
				int32_t _integral;
				int16_t arbitraryNumber;
				uint16_t count;
				int32_t Iout;
				int16_t Pout;
<<<<<<< HEAD
				int16_t output_correct;
				int16_t output;
				int16_t antiwind_error;
				int16_t antiwind_gain;
				int16_t antiwind_correct;
				int16_t error_int;
				int16_t error_int_gain;
=======
>>>>>>> 7c2d071828dcb5e5e53b50a3fee482a78737eed9
	public:
		// Constructor creates a motor encoder task object
		Motor(const char*, unsigned portBASE_TYPE, size_t, emstream*);
		//Motor(int omegam_measured, int I_actuator);
	
		// This gets called by the RTOS once to start this task's state loop
		void run(void);
	
};

#endif



