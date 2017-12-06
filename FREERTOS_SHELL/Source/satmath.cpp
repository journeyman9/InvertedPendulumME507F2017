//*************************************************************************************
/** \file satmath.cpp
 *    This file contains a saturated fixed point math library
 *
 *  \author Charlie Refvem
 *
 *  Revisions:
 *    \li 10-13-2015 CTR Original file (.c and .h files)
 *	  \li 01-19-2016 CTR Modified for C++
 *	  \li 01-27-2016 Added doxygen comments
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
//*************************************************************************************

#include "satmath.h"

//-------------------------------------------------------------------------------------
/** \brief This function performs saturated addition
 *  \details The function performs addition of two 16-bit signed integers, saturating
 *           at either the maximal or minimal 16-bit values
 *  @param x Augend
 *  @param y Addend
 *  @return Sum
 */
int16_t satmath::signed_saturated_add(int16_t x, int16_t y)
{
	// determine the lower or upper bound of the result
	int16_t ret =  (x < 0) ? INT16_MIN : INT16_MAX;
	// this is always well defined:
	// if x < 0 this adds a positive value to INT16_MIN
	// if x > 0 this subtracts a positive value from INT16_MAX
	int16_t comp = ret - x;
	// the condition is equivalent to
	// ((x < 0) && (y > comp)) || ((x >=0) && (y <= comp))
	if ((x < 0) == (y > comp)) ret = x + y;
	return ret;
}

//-------------------------------------------------------------------------------------
/** \brief This function performs saturated subtraction
 *  \details The function performs subtraction of two 16-bit signed integers, saturating
 *           at either the maximal or minimal 16-bit values
 *  @param x Minuend
 *  @param y Subtrahend 
 *  @return Difference
 */
int16_t satmath::signed_saturated_sub(int16_t x, int16_t y)
{
	// if they are equal return 0
	if(x == y) return 0;
	
	// negate y
	if (y == INT16_MIN && x < 0)  x++;
	y = (y == INT16_MIN) ? INT16_MAX:-y;
	return signed_saturated_add(x,y);
}
//-------------------------------------------------------------------------------------
/** \brief This function performs a saturated absolute value
 *  \details The function corrects for the undefined behavior when using abs() on a
 *           minimally saturated 16-bit number by returning a maximally saturated
 *           16-bit number
 *  @param x Signed value
 *  @return Absolute value
 */
int16_t satmath::saturated_abs(int16_t x)
{
	return (x == 0x8000L) ? 0x7fffL:abs(x);
}

//-------------------------------------------------------------------------------------
/** \brief This function performs saturated multiplication
 *  \details The function performs multiplication of two 16-bit signed integers
 *           ensuring a 32-bit result
 *  @param x Multiplicand
 *  @param y Multiplier
 *  @return Product
 */
int32_t satmath::signed_saturated_mul(int16_t x, int16_t y)
{
	if (x==INT16_MIN && y==INT16_MIN) return INT32_MAX;
	return (int32_t)(int16_t)x * (int32_t)(int16_t)y;
}

//-------------------------------------------------------------------------------------
/** \brief This function performs saturated division
 *  \details The function performs division of a 32-bit integer by a 16-bit signed
 *           integer. The result is saturated to the minimal or maximal 16-bit value
 *  @param x Dividend
 *  @param y Divisor 
 *  @return Quotient
 */
int16_t satmath::signed_saturated_div(int32_t x, int16_t y)
{
	int32_t ret = (x / (int32_t) y);
	ret = (ret > 32767) ? 32767:ret;
	ret = (ret < -32768) ? -32768:ret;
	return (int16_t) ret;
}