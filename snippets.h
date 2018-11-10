#ifndef __snippets
#define __snippets

#include "stdint.h"

#define NUMBER_OF_ADC_CHANNEL 6
#define M_RIGHT_BW TIM3->CCR1
#define M_RIGHT_FW TIM3->CCR2
#define M_LEFT_BW TIM1->CCR3
#define M_LEFT_FW TIM1->CCR2

extern uint16_t ADC_array[NUMBER_OF_ADC_CHANNEL];

void SetSysClock(void);
void ConfigureGPIO(void);
void ConfigureGPIOforADC(void);
void SetClockForADC(void);
void CalibrateADC(void);
void EnableADC(void);
void ConfigureADC(void);
void ConfigureDMA(void);
void ConfigureTIMsPWM(void);

#endif
