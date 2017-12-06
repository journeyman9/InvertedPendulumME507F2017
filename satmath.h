//======================================================================================
/** \file satmath.h
 *    This file contains a saturated fixed point math library
 *
 *  Revisions:
 *    \li 01-14-2015 CTR Original File
 *
 *  License:
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 *    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUEN-
 *    TIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
 *    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 *    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 *    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//======================================================================================

// This define prevents this .H file from being included multiple times in a .CPP file
#ifndef SATMATH_H
#define SATMATH_H

#include "stdint.h"
#include "stdlib.h"

//#define INT16_MIN (int16_t) 0x8000
//#define INT16_MAX (int16_t) 0x7FFF

// Some handy macros to shorten the function names
#define ssadd(x,y)	satmath::signed_saturated_add(x,y)		//!< This macro provides shorthand for saturated addition.
#define sssub(x,y)	satmath::signed_saturated_sub(x,y)		//!< This macro provides shorthand for saturated subtraction.
#define ssabs(x)	satmath::saturated_abs(x)				//!< This macro provides shorthand for saturated absolute value.
#define ssmul(x,y)	satmath::signed_saturated_mul(x,y)		//!< This macro provides shorthand for saturated multiplication.
#define ssdiv(x,y)	satmath::signed_saturated_div(x,y)		//!< This macro provides shorthand for saturated division.

//-------------------------------------------------------------------------------------
/** \brief This namespace includes several functions to do saturated 16-bit signed math
 */
namespace satmath 
{
	int16_t				signed_saturated_add(int16_t x, int16_t y);
	int16_t				signed_saturated_sub(int16_t x, int16_t y);
	int16_t				saturated_abs(int16_t x);
	int32_t				signed_saturated_mul(int16_t x, int16_t y);
	int16_t				signed_saturated_div(int32_t x, int16_t y);	
} // end namespace satmath

#endif // SATMATH_H