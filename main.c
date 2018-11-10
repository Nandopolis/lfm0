/*----------------------------------------------------------------------------
 *      main Template for CMSIS RTE C/C++ Project
 *----------------------------------------------------------------------------
 *      Name:    main.c
 *      Purpose: Generic main program body including main() function
 *      Rev.:    1.0.0
 *----------------------------------------------------------------------------*/
/*******************************************************************************
* Copyright (c) 2015 ARM Ltd. and others
* All rights reserved. This program and the accompanying materials
* are made available under the terms of the Eclipse Public License v1.0
* which accompanies this distribution, and is available at
* http://www.eclipse.org/legal/epl-v10.html
*
* Contributors:
* ARM Ltd and ARM Germany GmbH - file template
*******************************************************************************/

#ifdef _RTE_
  #include "RTE_Components.h"             // Component selection
#endif
#ifdef RTE_CMSIS_RTOS                     // when RTE component CMSIS RTOS is used
  #include "cmsis_os.h"                   // CMSIS RTOS header file
#endif

#include CMSIS_device_header

#include "snippets.h"
#include "LowPass.h"
#include "fsme.h"

enum CONTROL_CONSTANTS
{
	SAMPLING_PERIOD = 2,
	CONTROL_PERIOD = 4,
	ROBOT_SPEED = 768,
	FORWARD_WORKSPACE = 4095 - ROBOT_SPEED,
};

enum STATES {
	STANDBY = 0,
	ADC_SAMPLE,
	FILTER,
	CALIB,
	CONTROL,

	STATES_NO,
};

typedef struct
{
	float32_t filter_output[NUMBER_OF_ADC_CHANNEL];
} filter_array;

float32_t max_array[NUMBER_OF_ADC_CHANNEL], min_array[NUMBER_OF_ADC_CHANNEL];
LowPassType *low_pass[NUMBER_OF_ADC_CHANNEL];
float32_t line_pos = 0;
// float32_t kp = 1024.0, ki = 64.0, kd = 32.0;
float32_t const kp = (0.463564704062526 + 1.45) * 4095.0 / 5.0;
float32_t const ki = (0.032882755844441 - 0.02) * 4095.0 / 5.0;
float32_t const kd = (1.45161100320551 + 1.65) * 4095.0 / 5.0;
float32_t const sensor_angle = 0.15;
float32_t const T = (float32_t)CONTROL_PERIOD / 1000.0;
float32_t e, u, aux;
float32_t sum_e = 0.0, e_1 = 0.0;
uint32_t last_mode, start_ms_tick;
uint8_t sample_count = 0, adc_ready = 0, mode_change = 0;

uint32_t ms_ticks = 0;
void SysTick_Handler (void) {
	ms_ticks++;
}
void delay_ms (uint32_t ms) {
	uint32_t time;

	time = ms_ticks;
	while (ms_ticks - time < ms);
}

uint8_t isNotStdby(void);
uint8_t isAdcReady(void);
uint8_t isSamplingPeriod(void);
uint8_t isCalib(void);
uint8_t isControl(void);

void standby(void);
void filter(void);
void control(void);
void calib(void);
void adc(void);

FSME_TRANS stdby_trans[] = {
		{isNotStdby, ADC_SAMPLE}
};
FSME_TRANS adc_trans[] = {
		{isAdcReady, FILTER}
};
FSME_TRANS filter_trans[] = {
		{isSamplingPeriod, STANDBY},
		{isControl, CONTROL},
		{isCalib, CALIB}
};
FSME_TRANS calib_trans[] = {
		{isSamplingPeriod, STANDBY}
};
FSME_TRANS control_trans[] = {
		{isSamplingPeriod, STANDBY}
};

FSME_STATE fsm_states[STATES_NO] = {
		{standby, 1, stdby_trans},
		{adc, 1, adc_trans},
		{filter, 3, filter_trans},
		{calib, 1, calib_trans},
		{control, 1, control_trans}
};

FSME_FSM fsm = {1, STANDBY, STATES_NO, 1, fsm_states, 1, stdby_trans};

/* main function */
int main(void)
{
	uint8_t i;

#ifdef RTE_CMSIS_RTOS                   // when using CMSIS RTOS
  osKernelInitialize ();                // initialize CMSIS-RTOS
#endif

  /* Initialize device HAL here */
  SetSysClock();
  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock / 1000);
	ConfigureGPIO();
	SetClockForADC();
	CalibrateADC();
	ConfigureGPIOforADC();
	EnableADC();
	ConfigureADC();
	ConfigureDMA();
  ConfigureTIMsPWM();
  for (i = 0; i < NUMBER_OF_ADC_CHANNEL; i++) {
		low_pass[i] = LowPass_create();
	}
  for(i = 0; i < NUMBER_OF_ADC_CHANNEL; i++) {
		max_array[i] = 0.0;
		min_array[i] = 4095.0;
	}

#ifdef RTE_CMSIS_RTOS                   // when using CMSIS RTOS
  // create 'thread' functions that start executing,
  // example: tid_name = osThreadCreate (osThread(name), NULL);
  osKernelStart ();                     // start thread execution 
#endif

  /* Infinite loop */
  while (1)
  {
   /* Add application code here */
  	FSM_Run(&fsm);
  }

}

void stopMotors(void) {
	M_RIGHT_BW = 4095;
	M_RIGHT_FW = 4095;
	M_LEFT_BW = 4095;
	M_LEFT_FW = 4095;
}

void setMotorPWM(int32_t u) {
	uint32_t aux;
	__IO uint32_t *m1_fw, *m1_bw, *m2_fw, *m2_bw;

	if (u > 0) {
		m2_fw = &M_RIGHT_FW;
		m2_bw = &M_RIGHT_BW;
		m1_fw = &M_LEFT_FW;
		m1_bw = &M_LEFT_BW;
	}
	else {
		u *= -1;
		m1_fw = &M_RIGHT_FW;
		m1_bw = &M_RIGHT_BW;
		m2_fw = &M_LEFT_FW;
		m2_bw = &M_LEFT_BW;
	}

	if (u > FORWARD_WORKSPACE) {
		aux = 2 * u - FORWARD_WORKSPACE;
		*m1_bw = 0;
		*m1_fw = 4095;
	}
	else {
		aux = u;
		*m1_bw = 4095 - (ROBOT_SPEED + u);
		*m1_fw = 4095;
	}
	if (aux > ROBOT_SPEED) {
		aux -= ROBOT_SPEED;
		*m2_bw = 4095;
		*m2_fw = 4095 - aux;
	}
	else {
		*m2_bw = 4095 - (ROBOT_SPEED - aux);
		*m2_fw = 4095;
	}
}

uint8_t calibrate(LowPassType **data, float32_t *cal_data) {
	uint32_t i;
	uint8_t in_line = 0;

	for(i = 0; i < NUMBER_OF_ADC_CHANNEL; i++) {
		if (data[i]->output > min_array[i]) {
			if (data[i]->output < max_array[i]) {
				cal_data[i] = (data[i]->output - min_array[i]) \
							/ (max_array[i] - min_array[i]);
			}
			else {
				cal_data[i] = 1.0;
			}
		}
		else {
			cal_data[i] = 0.0;
		}
		if (cal_data[i] > 0.5) {
			in_line++;
		}
	}
	return in_line;
}

float32_t getLinePos(float32_t last_line_pos) {
	float32_t line_pos;
	float32_t cal_data[NUMBER_OF_ADC_CHANNEL];
	float32_t num = 0.0, div = 0.0;
	float32_t static const offset = ((float32_t)NUMBER_OF_ADC_CHANNEL - 1) / 2;
	uint32_t i;

	if (calibrate(low_pass, cal_data)) {
		for (i = 0; i < NUMBER_OF_ADC_CHANNEL; ++i) {
			num += cal_data[i] * i;
			div += cal_data[i];
		}
		line_pos = num /div - offset;
	}
	else {
		if (last_line_pos > 0.0) {
			line_pos = offset;
		}
		else {
			line_pos = -offset;
		}
	}
	return line_pos;
}

uint8_t isNotStdby(void) {
	uint32_t new_mode;

	new_mode = GPIOF->IDR & 3;
	if (new_mode != 0) {
		if (new_mode != last_mode) {
			delay_ms(10);
			if (new_mode == (GPIOF->IDR & 3)) {
				last_mode = new_mode;
				mode_change = 1;
				return 1;
			}
			else {
				return 0;
			}
		}
		else {
			mode_change = 0;
			return 1;
		}
	}
	else {
		stopMotors();
		return 0;
	}
}
uint8_t isAdcReady(void) {
	if (adc_ready) {
		adc_ready = 0;
		return 1;
	}
	return 0;
}
uint8_t isSamplingPeriod(void) {
	if (ms_ticks - start_ms_tick >= SAMPLING_PERIOD) {
		return 1;
	}
	return 0;
}
uint8_t isCalib(void) {
	if ((GPIOF->IDR & 3) == 1) {
		return 1;
	}
	return 0;
}
uint8_t isControl(void) {
	if (((GPIOF->IDR & 3) == 2) && (sample_count == 2)) {
		sample_count = 0;
		return 1;
	}
	return 0;
}

void standby(void) {
	__NOP();
}

void adc(void) {
	if (fsm.StateChanged) {
		GPIOB->BSRR = GPIO_BSRR_BS_1;
		__NOP();
		__NOP();
		ADC1->CR |= ADC_CR_ADSTART; /* start the ADC conversion */
		start_ms_tick = ms_ticks;
	}
}

void filter(void) {
	float32_t input;
	uint32_t i;

	if (fsm.StateChanged) {
		if (mode_change) {
			for(i = 0; i < NUMBER_OF_ADC_CHANNEL; i++) {
				LowPass_reset(low_pass[i]);
			}
		}
		for(i = 0; i < NUMBER_OF_ADC_CHANNEL; i++) {
			input = (float32_t)ADC_array[i];
			LowPass_writeInput(low_pass[i], input);
		}
		sample_count++;
	}
}

void calib(void)
{
	uint32_t i;
	if (fsm.StateChanged) {
		/*if (mode_change) {
			for(i = 0; i < NUMBER_OF_ADC_CHANNEL; i++) {
				max_array[i] = 0.0;
				min_array[i] = 4095.0;
			}
		}*/
		for(i = 0; i < NUMBER_OF_ADC_CHANNEL; i++) {
			if (low_pass[i]->output > max_array[i]) {
				max_array[i] = low_pass[i]->output;
			}
			else if (low_pass[i]->output < min_array[i]) {
				min_array[i] = low_pass[i]->output;
			}
		}
	}
}

void control (void)
{
	if (fsm.StateChanged) {
		/*if (mode_change) {
			line_pos = 0;
			sum_e = 0.0;
			e_1 = 0.0;
		}*/
		line_pos = getLinePos(line_pos);
		e = (0.0 - line_pos);
		u = kp * e;
		u += kd * (e - e_1) / T;
		aux = ki * (e + sum_e * 0.5) * T;
		if ((aux > -2 * ROBOT_SPEED) && (aux < 2 * ROBOT_SPEED)) {
			sum_e = e + sum_e * 0.5;
		}
		u += ki * T * sum_e;
		if (u > 2 * ROBOT_SPEED)
		{
			u = 2 * ROBOT_SPEED;
		}
		else if (u < -2 * ROBOT_SPEED)
		{
			u = -2 * ROBOT_SPEED;
		}
		setMotorPWM((int32_t)u);
		e_1 = e;
	}
}

void DMA1_Channel1_IRQHandler(void) {
	if ((DMA1->ISR & DMA_ISR_TCIF1) != 0) { /* Test if transfer completed on DMA channel 1 */
		DMA1_Channel1->CCR &= (uint32_t)(~DMA_CCR_EN); /* Disable DMA Channel 1 to write in CNDTR*/
		DMA1_Channel1->CNDTR = NUMBER_OF_ADC_CHANNEL; /* Reload the number of DMA tranfer to be performs on DMA channel 1 */
		DMA1_Channel1->CCR |= DMA_CCR_EN; /* Enable again DMA Channel 1 */
		DMA1->IFCR |= DMA_IFCR_CTCIF1; /* Clear the flag */
		GPIOB->BRR = GPIO_BRR_BR_1;
		adc_ready = 1;
	}
	else if ((DMA1->ISR & DMA_ISR_TEIF1) != 0) { /* Test if transfer error on DMA channel 1 */
		DMA1->IFCR |= DMA_IFCR_CTEIF1; /* Clear the flag */
		while(1);
	}
}

