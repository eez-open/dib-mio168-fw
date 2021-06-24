#include <math.h>

#include "main.h"

#include "firmware.h"
#include "utils.h"

extern "C" TIM_HandleTypeDef htim2; // for PWM1 output
extern "C" TIM_HandleTypeDef htim3; // for PWM2 output

extern "C" void TIM2_Init(void);
extern "C" void TIM3_Init(void);

void PWM_Setup() {
	SET_PIN(DOUT_EN_GPIO_Port, DOUT_EN_Pin);
}

void PWM_SetParams(int i, SetParams &newState) {
	float newFreq = newState.pwm[i].freq;
	float newDuty = newState.pwm[i].duty;
	if (newFreq != currentState.pwm[i].freq || newDuty != currentState.pwm[i].duty) {
		if (i == 0) {
			if (currentState.pwm[i].freq > 0) {
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			}
			HAL_TIM_PWM_DeInit(&htim2);
			TIM2_Init();
		} else {
			if (currentState.pwm[i].freq > 0) {
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			}
			HAL_TIM_PWM_DeInit(&htim3);
			TIM3_Init();
		}

		uint32_t prescaler = sqrt(90000000.0f / newFreq) - 1;
		uint32_t period = prescaler;
		uint32_t pulse = newDuty == 100.0f ? period + 1 : (uint32_t)roundf(period * newDuty / 100.0f);

		if (i == 0) {
			TIM2->PSC = prescaler;
			TIM2->ARR = period;
			TIM2->CCR1 = pulse;
		} else {
			TIM3->PSC = prescaler;
			TIM3->ARR = period;
			TIM3->CCR2 = pulse;
		}

	    if (i == 0) {
	    	if (newFreq > 0) {
	    		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	    	}
	    } else {
	    	if (newFreq > 0) {
	    		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	    	}
	    }
	}
}
