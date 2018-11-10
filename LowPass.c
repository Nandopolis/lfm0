/******************************* SOURCE LICENSE *********************************
Copyright (c) 2015 MicroModeler.

A non-exclusive, nontransferable, perpetual, royalty-free license is granted to the Licensee to 
use the following Information for academic, non-profit, or government-sponsored research purposes.
Use of the following Information under this License is restricted to NON-COMMERCIAL PURPOSES ONLY.
Commercial use of the following Information requires a separately executed written license agreement.

This Information is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

******************************* END OF LICENSE *********************************/

// A commercial license for MicroModeler DSP can be obtained at http://www.micromodeler.com/launch.jsp

#include "LowPass.h"

#include <stdlib.h> // For malloc/free
#include <string.h> // For memset

// 1kHz sampling
/*float32_t LowPass_coefficients[10] = 
{
// Scaled for floating point

    0.026662349082022373, 0.05332469816404475, 0.026662349082022373, 1.4796742169311934, -0.5558215432824889,// b0, b1, b2, a1, a2
    0.015625, 0.03125, 0.015625, 1.700964331943526, -0.7884997398152981// b0, b1, b2, a1, a2

};*/

// 500Hz sampling
float32_t LowPass_coefficients[10] = 
{
// Scaled for floating point

    0.07718949372345962, 0.15437898744691925, 0.07718949372345962, 1.0485995763626117, -0.2961403575616696,// b0, b1, b2, a1, a2
    0.0625, 0.125, 0.0625, 1.3209134308194261, -0.6327387928852763// b0, b1, b2, a1, a2

};

LowPassType *LowPass_create( void )
{
	LowPassType *result = (LowPassType *)malloc( sizeof( LowPassType ) );	// Allocate memory for the object
	LowPass_init( result );											// Initialize it
	return result;																// Return the result
}

void LowPass_destroy( LowPassType *pObject )
{
	free( pObject );
}

 void LowPass_init( LowPassType * pThis )
{
	arm_biquad_cascade_df1_init_f32(	&pThis->instance, LowPass_numStages, LowPass_coefficients, pThis->state );
	LowPass_reset( pThis );

}

 void LowPass_reset( LowPassType * pThis )
{
	memset( &pThis->state, 0, sizeof( pThis->state ) ); // Reset state to 0
	pThis->output = 0;									// Reset output

}

 int LowPass_filterBlock( LowPassType * pThis, float * pInput, float * pOutput, unsigned int count )
{
	arm_biquad_cascade_df1_f32( &pThis->instance, pInput, pOutput, count );
	return count;

}


