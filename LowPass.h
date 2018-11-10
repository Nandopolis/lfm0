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

// Begin header file, LowPass.h

#ifndef LOWPASS_H_ // Include guards
#define LOWPASS_H_

#define ARM_MATH_CM0	// Use ARM Cortex M0
#define __FPU_PRESENT 1		// Does this device have a floating point unit?
#include <arm_math.h>	// Include CMSIS header

// Link with library: libarm_cortexM0_mathL.a (or equivalent)
// Add CMSIS/Lib/GCC to the library search path
// Add CMSIS/Include to the include search path
extern float32_t LowPass_coefficients[10];
static const int LowPass_numStages = 2;

typedef struct
{
	arm_biquad_casd_df1_inst_f32 instance;
	float32_t state[8];
	float32_t output;
} LowPassType;


LowPassType *LowPass_create( void );
void LowPass_destroy( LowPassType *pObject );
 void LowPass_init( LowPassType * pThis );
 void LowPass_reset( LowPassType * pThis );
#define LowPass_writeInput( pThis, input )  \
	arm_biquad_cascade_df1_f32( &pThis->instance, &input, &pThis->output, 1 );

#define LowPass_readOutput( pThis )  \
	pThis->output


 int LowPass_filterBlock( LowPassType * pThis, float * pInput, float * pOutput, unsigned int count );
#define LowPass_outputToFloat( output )  \
	(output)

#define LowPass_inputFromFloat( input )  \
	(input)

#endif // LOWPASS_H_
	
