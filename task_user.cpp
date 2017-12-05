//**************************************************************************************
/** \file task_user.cpp
 *    This file contains source code for a user interface task for a ME405/FreeRTOS
 *    test suite. 
 *
 *  Revisions:
 *    \li 09-30-2012 JRR Original file was a one-file demonstration with two tasks
 *    \li 10-05-2012 JRR Split into multiple files, one for each task
 *    \li 10-25-2012 JRR Changed to a more fully C++ version with class task_user
 *    \li 11-04-2012 JRR Modified from the data acquisition example to the test suite
 *
 *  License:
 *    This file is copyright 2012 by JR Ridgely and released under the Lesser GNU 
 *    Public License, version 2. It intended for educational use only, but its use
 *    is not limited thereto. */
/*    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 *    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUEN-
 *    TIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
 *    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 *    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 *    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//**************************************************************************************

#include <avr/io.h>                         // Port I/O for SFR's
#include <avr/wdt.h>                        // Watchdog timer header

#include "shared_data_sender.h"
#include "shared_data_receiver.h"
#include "task_user.h"                      // Header for this file


/** This constant sets how many RTOS ticks the task delays if the user's not talking.
 *  The duration is calculated to be about 5 ms.
 */
const portTickType ticks_to_delay = ((configTICK_RATE_HZ / 1000) * 5);


//-------------------------------------------------------------------------------------
/** This constructor creates a new data acquisition task. Its main job is to call the
 *  parent class's constructor which does most of the work.
 *  @param a_name A character string which will be the name of this task
 *  @param a_priority The priority at which this task will initially run (default: 0)
 *  @param a_stack_size The size of this task's stack in bytes 
 *                      (default: configMINIMAL_STACK_SIZE)
 *  @param p_ser_dev Pointer to a serial device (port, radio, SD card, etc.) which can
 *                   be used by this task to communicate (default: NULL)
 */

task_user::task_user (const char* a_name, 
					  unsigned portBASE_TYPE a_priority, 
					  size_t a_stack_size,
					  emstream* p_ser_dev
					 )
	: frt_task (a_name, a_priority, a_stack_size, p_ser_dev)
{
	// Nothing is done in the body of this constructor. All the work is done in the
	// call to the frt_task constructor on the line just above this one
}


//-------------------------------------------------------------------------------------
/** This task interacts with the user for force him/her to do what he/she is told. It
 *  is just following the modern government model of "This is the land of the free...
 *  free to do exactly what you're told." 
 */

void task_user::run (void)
{
	char char_in;                           // Character read from serial device
	time_stamp a_time;                      // Holds the time so it can be displayed

	// Tell the user how to get into command mode (state 1), where the user interface
	// task does interesting things such as diagnostic printouts
	*p_serial << PMS ("Press Ctrl-A for command mode") << endl;

	// This is an infinite loop; it runs until the power is turned off. There is one 
	// such loop inside the code for each task
	for (;;)
	{
		// Run the finite state machine. The variable 'state' is kept by the parent class
		switch (state)
		{
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			// In state 0, we transparently relay characters from the radio to the USB 
			// serial port and vice versa but watch for certain control characters
			case (0):
				if (p_serial->check_for_char ())        // If the user typed a
				{                                       // character, read
					char_in = p_serial->getchar ();     // the character

					// In this switch statement, we respond to different characters
					switch (char_in)
					{
						// Control-C means reset the AVR computer
						case (3):
							*p_serial << PMS ("Resetting AVR") << endl;
							wdt_enable (WDTO_120MS);
							for (;;);
							break;

						// Control-A puts this task in command mode
						case (1):
							print_help_message ();
							transition_to (1);
							break;

						// Any other character will be ignored
						default:
							break;
					};
				}

				// Check the print queue to see if another task has sent this task 
				// something to be printed
				else if (print_ser_queue.check_for_char ())
				{
					p_serial->putchar (print_ser_queue.getchar ());
				}

				break; // End of state 0

			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			// In state 1, we're in command mode, so when the user types characters, the
			// characters are interpreted as commands to do something
			case (1):
				if (p_serial->check_for_char ())				// If the user typed a
				{											// character, read
					char_in = p_serial->getchar ();			// the character

					// In this switch statement, we respond to different characters as
					// commands typed in by the user
					switch (char_in)
					{
						// The 'n' command asks what time it is right now
						case ('n'):
							*p_serial << (a_time.set_to_now ()) << endl;
							break;

						// The 'v' command asks for version and status information
						case ('v'):
							show_status ();
							break;

						// The 's' command has all the tasks dump their stacks
						case ('s'):
							print_task_stacks (p_serial);
							break;

						// The 'h' command is a plea for help
						case ('h'):
							print_help_message ();
							break;

						// The escape key or 'e' key go back to non-command mode
						case (27):
						case ('e'):
							*p_serial << PMS ("Exit command mode") << endl;
							transition_to (0);
							break;

						// If the character isn't recognized, ask: What's That Function?
						default:
							p_serial->putchar (char_in);
							*p_serial << PMS (":WTF?") << endl;
							break;
					}; // End switch for characters
				} // End if a character was received

				// In this state, characters from the radio are ignored; hopefully we'll
				// get out of this state before the radio buffer overflows and characters
				// are missed
				
				break; // End of state 1

			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			// We should never get to the default state. If we do, complain and restart
			default:
				*p_serial << PMS ("Illegal state! Resetting AVR") << endl;
				wdt_enable (WDTO_120MS);
				for (;;);
				break;

		} // End switch state

		runs++;                             // Increment counter for debugging

		// No matter the state, wait for approximately a millisecond before we 
		// run the loop again. This gives lower priority tasks a chance to run
		vTaskDelay (configMS_TO_TICKS (1));
	}
}


//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
/** This method prints a simple help message.
 */

void task_user::print_help_message (void)
{
	*p_serial << ATERM_BKG_CYAN << ATERM_TXT_BLACK << clrscr;
	*p_serial << PROGRAM_VERSION << PMS (" help") << endl;
	*p_serial << PMS ("  Ctl-C: Reset the AVR") << endl;
	*p_serial << PMS ("  Ctl-A: Enter command mode") << endl;
	*p_serial << PMS ("  In command mode only:") << endl;
	*p_serial << PMS ("    n:   Show the time right now") << endl;
	*p_serial << PMS ("    v:   Version and setup information") << endl;
	*p_serial << PMS ("    s:   Stack dump for tasks") << endl;
	*p_serial << PMS ("    e:   Exit command mode") << endl;
	*p_serial << PMS ("    h:   HALP!") << endl;
}


//-------------------------------------------------------------------------------------
/** This method displays information about the status of the system, including the
 *  following: 
 *    \li The name and version of the program
 *    \li The name, status, priority, and free stack space of each task
 *    \li Processor cycles used by each task
 *    \li Amount of heap space free and setting of RTOS tick timer
 */

void task_user::show_status (void)
{
	time_stamp the_time;					// Holds current time for printing

	// Show program vesion, time, and free heap space
	*p_serial << endl << PROGRAM_VERSION << PMS (__DATE__) << endl 
			  << PMS ("Time: ") << the_time.set_to_now ()
			  << PMS (", Heap free: ") << heap_left() << PMS ("/") 
			  << configTOTAL_HEAP_SIZE;

	// Show how the timer/counter is set up to cause RTOS timer ticks
	*p_serial << PMS (", TCC0CCA=") << TCC0.CCA << endl << endl;

	// Have the tasks print their status
	print_task_list (p_serial);
}

