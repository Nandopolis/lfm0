#include "snippets.h"
#include "stm32f0xx.h"

uint16_t ADC_array[NUMBER_OF_ADC_CHANNEL];

void SetSysClock(void) {
	/* SYSCLK, HCLK, PCLK configuration ----------------------------------------*/

	/* At this stage the HSI is already enabled */
	
	/* Enable debug clock */
	RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;

	/* Enable Prefetch Buffer and set Flash Latency */
	FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

	/* HCLK = SYSCLK */
	RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
	  
	/* PCLK = HCLK */
	RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE_DIV1;

	/* PLL configuration = (HSI/2) * 12 = ~48 MHz */
	RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL));
	RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI_DIV2 | RCC_CFGR_PLLMUL12);
			
	/* Enable PLL */
	RCC->CR |= RCC_CR_PLLON;

	/* Wait till PLL is ready */
	while((RCC->CR & RCC_CR_PLLRDY) == 0);

	/* Select PLL as system clock source */
	RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
	RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    

	/* Wait till PLL is used as system clock source */
	while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL);
}

/**
  * @brief  This function enables the peripheral clocks on GPIO port A and B,
  *         configures GPIO PB1 in output mode for the LED pin,
  * @param  None
  * @retval None
  */
void ConfigureGPIO(void)
{  
	/* (1) Enable the peripheral clock of GPIO */
	/* (2) Select output mode (01) on GPIOB pin 1 */
	/* (3) Select input mode (00) on GPIOF pin 0-1 */
	/* (4) Select pullup mode (01) on GPIOF pin 0-1 */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOFEN; /* (1) */  
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER1) | GPIO_MODER_MODER1_0; /* (2) */
	GPIOF->MODER = GPIOF->MODER & ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1); /* (3) */
	GPIOF->PUPDR = (GPIOF->PUPDR & ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1)) | \
					GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0; /* (4) */
}

/**
  * @brief  This function enables the peripheral clocks on GPIO port A
  *         configures PA0 - PA5 in Analog mode.
  *         For portability, some GPIO are again enabled.
  * @param  None
  * @retval None
  */
void ConfigureGPIOforADC(void)
{
	/* (1) Enable the peripheral clock of GPIOA, GPIOB and GPIOC */
	/* (2) Select analog mode for PA0 - PA5 */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (1) */
	GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 \
					| GPIO_MODER_MODER2 | GPIO_MODER_MODER3 \
					| GPIO_MODER_MODER4 | GPIO_MODER_MODER5; /* (2) */ 
}

/**
  * @brief  This function enables the clock in the RCC for the ADC
  *         and start HSI 14MHz dedicated RC oscillator
  * @param  None
  * @retval None
  */
void SetClockForADC(void) {
	/* (1) Enable the peripheral clock of the ADC */
	/* (2) Start HSI14 RC oscillator */ 
	/* (3) Wait HSI14 is ready */
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; /* (1) */
	RCC->CR2 |= RCC_CR2_HSI14ON; /* (2) */
	while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0); /* (3) */
}

/**
  * @brief  This function performs a self-calibration of the ADC
  * @param  None
  * @retval None
  */
void CalibrateADC(void)
{
  /* (1) Ensure that ADEN = 0 */
  /* (2) Clear ADEN */ 
  /* (3) Launch the calibration by setting ADCAL */
  /* (4) Wait until ADCAL=0 */
  if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
  {
    ADC1->CR &= (uint32_t)(~ADC_CR_ADEN);  /* (2) */  
  }
  ADC1->CR |= ADC_CR_ADCAL; /* (3) */
  while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (4) */
  {
    /* For robust implementation, add here time-out management */
  }  
}

/**
  * @brief  This function enables the ADC
  * @param  None
  * @retval None
  */
void EnableADC(void)
{
  /* (1) Enable the ADC */
  /* (2) Wait until ADC ready */
  do 
  {
    /* For robust implementation, add here time-out management */
		ADC1->CR |= ADC_CR_ADEN; /* (1) */
  }while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (2) */;
}

/**
  * @brief  This function configures the ADC to convert sequentially 6 channels
            in single conversion mode.
  *         The conversion frequency is 14MHz 
  *         The interrupt on overrun is enabled and the NVIC is configured
  * @param  None
  * @retval None
  */
void ConfigureADC(void)
{
	/* (1) Select HSI14 by writing 00 in CKMODE (reset value) */  
	/* (2) Select the single conversion mode (reset value)*/
	/* (3) Select CHSEL0 - 5 */
	/* (4) Select a sampling mode of 010 i.e. 13.5 ADC clk to be greater than 0.96us */
	/* (5) Enable interrupts on over run */
	//ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; /* (1) */  
	//ADC1->CFGR1 &= ~ADC_CFGR1_CONT; /* (2)*/
	ADC1->CHSELR = ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1 \
					| ADC_CHSELR_CHSEL2 | ADC_CHSELR_CHSEL3 \
					| ADC_CHSELR_CHSEL4 | ADC_CHSELR_CHSEL5; /* (3)*/
	ADC1->SMPR |= ADC_SMPR_SMP_1; /* (4) */
	ADC1->IER = ADC_IER_OVRIE; /* (5) */

	/* Configure NVIC for ADC */
	/* (7) Enable Interrupt on ADC */
	/* (8) Set priority for ADC */
	NVIC_EnableIRQ(ADC1_COMP_IRQn); /* (7) */
	NVIC_SetPriority(ADC1_COMP_IRQn,2); /* (8) */
}

/**
  * @brief  This function configures the DMA to store the result of an ADC sequence.
  *         The conversion results are stored in N-items array.
  * @param  None
  * @retval None
  */
void ConfigureDMA(void)
{
	/* (1) Enable the peripheral clock on DMA */
	/* (2) Enable DMA transfer on ADC - DMACFG is kept at 0 for one shot mode */ 
	/* (3) Configure the peripheral data register address */ 
	/* (4) Configure the memory address */
	/* (5) Configure the number of DMA tranfer to be performs on DMA channel 1 */
	/* (6) Configure increment, size and interrupts */
	/* (7) Enable DMA Channel 1 */
	RCC->AHBENR |= RCC_AHBENR_DMA1EN; /* (1) */
	ADC1->CFGR1 |= ADC_CFGR1_DMAEN; /* (2) */
	DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR)); /* (3) */
	DMA1_Channel1->CMAR = (uint32_t)(ADC_array); /* (4) */
	DMA1_Channel1->CNDTR = NUMBER_OF_ADC_CHANNEL; /* (5) */
	DMA1_Channel1->CCR |= DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 \
						| DMA_CCR_TEIE | DMA_CCR_TCIE ; /* (6) */  
	DMA1_Channel1->CCR |= DMA_CCR_EN; /* (7) */

	/* Configure NVIC for DMA */
	/* (8) Enable Interrupt on DMA */
	/* (9) Set priority for DMA */
	NVIC_EnableIRQ(DMA1_Channel1_IRQn); /* (8) */
	NVIC_SetPriority(DMA1_Channel1_IRQn,2); /* (9) */
}

/**
  * @brief  This function configures the TIM1 and TIM3 as PWM mode 1 and center-aligned
  *         and enables the peripheral clock on TIMx and on GPIOA.
  *         It configures GPIO PAx as Alternate function for TIMx_CHx
  *         To use another timer, channel or GPIO, the RCC and GPIO configuration 
  *         must be adapted according to the datasheet.
  *         In case of other timer, the interrupt sub-routine must also be renamed
  *         with the right handler and the NVIC configured correctly.
  * @param  None
  * @retval None
  */
void ConfigureTIMsPWM(void)
{
	/* (1) Enable the peripheral clock of Timer x */
	/* (2) Enable the peripheral clock of GPIOA */
	/* (3) Select alternate function mode on GPIOA pin 8 */
	/* (4) Select AF1 on PA6,7 znd AF2 on PA9,10 in AFRL-H for TIMx_CHx */

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* (1) */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (2) */  
	GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7
									| GPIO_MODER_MODER9 | GPIO_MODER_MODER10)) \
					| (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1
						| GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1); /* (3) */
	GPIOA->AFR[0] |= 0x11000000; /* (4) */
	GPIOA->AFR[1] |= 0x00000220;

	/* (1) Set prescaler to 0, i.e 48MHz (reset value) */ 
	/* (2) Set ARR = 4095, as timer clock is 48MHz and center-aligned counting,
		 the period is 170.625 us */
	/* (3) Set CCRx = 0 (reset value) */
	/* (4) Select PWM mode 1 on OC2,3  (OC2,3M = 110),
		 enable preload register on OC2,3 (OC2,3PE = 1, reset value) */
	/* (5) Select active high polarity on OC2,3 (CC2,3P = 0, reset value),
		 enable the output on OC2,3 (CC2,3E = 1)*/
	/* (6) Enable output (MOE = 1)*/
	/* (7) Enable counter (CEN = 1)
		 select center-aligned mode 1 (CMS = 01) */  
	/* (8) Force update generation (UG = 1) */

	//TIM1->PSC = 0; /* (1) */
	TIM1->ARR = 0x0FFF; /* (2) */
	TIM1->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE; /* (4) */
	TIM1->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;
	TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3E; /* (5) */
	TIM1->BDTR |= TIM_BDTR_MOE; /* (6) */
	TIM1->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN; /* (7) */
	TIM1->EGR |= TIM_EGR_UG; /* (8) */
	
	//basically the same for TIM3
	//TIM3->PSC = 0; /* (1) */
	TIM3->ARR = 0x0FFF; /* (2) */
	TIM3->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE; /* (4) */
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
	TIM3->CCER |= TIM_CCER_CC2E | TIM_CCER_CC1E; /* (5) */
	TIM3->BDTR |= TIM_BDTR_MOE; /* (6) */
	TIM3->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN; /* (7) */
	TIM3->EGR |= TIM_EGR_UG; /* (8) */
}
