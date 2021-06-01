
#define _USE_MATH_DEFINES
#include <math.h>
#include <memory.h>

#include "main.h"
#include "firmware.h"
#include <funcgen.h>
#include "dac7760.h"
#include "dac7563.h"
#include "adc.h"
#include "dout.h"
#include "utils.h"

extern "C" TIM_HandleTypeDef htim7;

#define M_PI_F ((float)M_PI)

static const float TIMER_PERIOD = 1.0f / 60000.0f;

typedef float (*WaveformFunction)(float);

////////////////////////////////////////////////////////////////////////////////

WaveformFunction g_aoutWaveFormFunc[4];

float g_aoutMin[4];
float g_aoutMax[4];

float g_aoutPhi[4];
float g_aoutDphi[4];

float g_aoutDutyCycles[4];

WaveformParameters g_aoutWaveformParameters[4];

////////////////////////////////////////////////////////////////////////////////

WaveformFunction g_doutWaveFormFunc[8];

float g_doutPhi[8];
float g_doutDphi[8];

float g_doutDutyCycles[8];

WaveformParameters g_doutWaveformParameters[8];

////////////////////////////////////////////////////////////////////////////////

typedef void (*TimerTickFunc)(void);
TimerTickFunc g_timerTickFunc[5];
int g_numTimerTickFuncs;
volatile int g_nextTimerTickFunc;

////////////////////////////////////////////////////////////////////////////////

float dcf(float t) {
	return 0.0f;
}

float sineHalff(float t) {
	if (t < M_PI_F) {
		return sinf(t);
	}

	return 0.0f;
}

float sineRectifiedf(float t) {
	if (t < M_PI_F) {
		return sinf(t);
	}

	return sinf(t - M_PI_F);
}

float trianglef(float t) {
	float a, b, c;

	if (t < M_PI_F / 2.0f) {
		a = 0;
		b = 1;
		c = 0;
	} else if (t < 3.0f * M_PI_F / 2.0f) {
		a = 1;
		b = -1;
		c = M_PI_F / 2.0f;
	} else {
		a = -1;
		b = 1;
		c = 3.0f * M_PI_F / 2.0f;
	}

	return a + b * (t - c) / (M_PI_F / 2.0f);
}

float squaref(float t) {
	if (t < M_PI_F) {
		return 1.0f;
	}
	return -1.0f;
}

static float g_dutyCycle;

float pulsef(float t) {
	if (t < g_dutyCycle * 2.0f * M_PI_F / 100.0f) {
		return 1.0f;
	}
	return -1.0f;
}

float sawtoothf(float t) {
	return -1.0f + t / M_PI_F;
}

float arbitraryf(float t) {
	return 0.0f;
}

WaveformFunction getWaveformFunction(WaveformParameters &waveformParameters) {
	if (waveformParameters.waveform == WAVEFORM_DC) {
		return dcf;
	} else if (waveformParameters.waveform == WAVEFORM_SINE) {
		return sinf;
	} else if (waveformParameters.waveform == WAVEFORM_SINE_HALF) {
		return sineHalff;
	} else if (waveformParameters.waveform == WAVEFORM_SINE_RECTIFIED) {
		return sineRectifiedf;
	} else if (waveformParameters.waveform == WAVEFORM_TRIANGLE) {
		return trianglef;
	} else if (waveformParameters.waveform == WAVEFORM_SQUARE) {
		return squaref;
	} else if (waveformParameters.waveform == WAVEFORM_PULSE) {
		g_dutyCycle = waveformParameters.dutyCycle;
		return pulsef;
	} else if (waveformParameters.waveform == WAVEFORM_SAWTOOTH) {
		return sawtoothf;
	} else {
		return arbitraryf;
	}
}

////////////////////////////////////////////////////////////////////////////////

void FuncGen_AOUT(int i) {
	auto &waveformParameters = g_aoutWaveformParameters[i];

	g_dutyCycle = g_aoutDutyCycles[i];
	float value;
	if (waveformParameters.waveform == WAVEFORM_DC) {
		value = waveformParameters.amplitude;
	} else {
		value = waveformParameters.offset + waveformParameters.amplitude * g_aoutWaveFormFunc[i](g_aoutPhi[i]) / 2.0f;
	}

	g_aoutPhi[i] += g_aoutDphi[i];
	if (g_aoutPhi[i] >= 2.0f * M_PI_F) {
		g_aoutPhi[i] -= 2.0f * M_PI_F;
	}

	value = 65535.0f * (value - g_aoutMin[i]) / (g_aoutMax[i] - g_aoutMin[i]);

	if (i < 2) {
		DAC_SetValue(i, value);
	} else {
		DACDual_SetValue(i - 2, value);
	}
}

void FuncGen_AOUT1() {
	FuncGen_AOUT(0);
}

void FuncGen_AOUT2() {
	FuncGen_AOUT(1);
}

void FuncGen_AOUT3() {
	FuncGen_AOUT(2);
}

void FuncGen_AOUT4() {
	FuncGen_AOUT(3);
}

////////////////////////////////////////////////////////////////////////////////

void ADC_MeasureTickFromFuncGen30Ksps() {
	ADC_MeasureTickFromFuncGen(30);
}

void ADC_MeasureTickFromFuncGen20Ksps() {
	ADC_MeasureTickFromFuncGen(30);
}

void ADC_MeasureTickFromFuncGen15Ksps() {
	ADC_MeasureTickFromFuncGen(15);
}

void ADC_MeasureTickFromFuncGen12Ksps() {
	ADC_MeasureTickFromFuncGen(12);
}

////////////////////////////////////////////////////////////////////////////////

void TimerTickFuncIdle() {
}

////////////////////////////////////////////////////////////////////////////////

void FuncGen_DOUT(int i) {
	auto &waveformParameters = g_doutWaveformParameters[i];

	if (waveformParameters.waveform == WAVEFORM_NONE) {
		return;
	}

	g_dutyCycle = g_doutDutyCycles[i];
	float value = g_doutWaveFormFunc[i](g_doutPhi[i]);

	g_doutPhi[i] += g_doutDphi[i];
	if (g_doutPhi[i] >= 2.0f * M_PI_F) {
		g_doutPhi[i] = 0;
	}

	Dout_SetPinState(i, value > 0.5f ? 1 : 0);
}

////////////////////////////////////////////////////////////////////////////////

void FuncGen_Setup() {
	g_timerTickFunc[0] = ADC_MeasureTickFromFuncGen30Ksps;
	g_timerTickFunc[1] = TimerTickFuncIdle;
	g_numTimerTickFuncs = 2;

	TIM7->PSC = 99;
	TIM7->ARR = (uint16_t)(TIMER_PERIOD * 900000) - 1;
	HAL_TIM_Base_Start_IT(&htim7);
}

void FuncGen_SetParams(SetParams &newState) {
	__disable_irq();

	if (memcmp((void *)g_aoutWaveformParameters, newState.aoutWaveformParameters, sizeof(newState.aoutWaveformParameters)) != 0) {
		TimerTickFunc timerTickFunc[5];
		int numTimerTickFuncs = 0;

		if (newState.aoutWaveformParameters[0].waveform != WAVEFORM_NONE) {
			timerTickFunc[numTimerTickFuncs++] = FuncGen_AOUT1;
		}
		if (newState.aoutWaveformParameters[1].waveform != WAVEFORM_NONE) {
			timerTickFunc[numTimerTickFuncs++] = FuncGen_AOUT2;
		}
		if (newState.aoutWaveformParameters[2].waveform != WAVEFORM_NONE) {
			timerTickFunc[numTimerTickFuncs++] = FuncGen_AOUT3;
		}
		if (newState.aoutWaveformParameters[3].waveform != WAVEFORM_NONE) {
			timerTickFunc[numTimerTickFuncs++] = FuncGen_AOUT4;
		}

		if (numTimerTickFuncs == 0) {
			timerTickFunc[numTimerTickFuncs++] = ADC_MeasureTickFromFuncGen30Ksps;
		} else if (numTimerTickFuncs == 1) {
			timerTickFunc[numTimerTickFuncs++] = ADC_MeasureTickFromFuncGen30Ksps;
		} else if (numTimerTickFuncs == 2) {
			timerTickFunc[numTimerTickFuncs++] = ADC_MeasureTickFromFuncGen20Ksps;
		} else if (numTimerTickFuncs == 3) {
			timerTickFunc[numTimerTickFuncs++] = ADC_MeasureTickFromFuncGen15Ksps;
		} else {
			timerTickFunc[numTimerTickFuncs++] = ADC_MeasureTickFromFuncGen12Ksps;
		}

		if (numTimerTickFuncs == 1) {
			timerTickFunc[numTimerTickFuncs++] = TimerTickFuncIdle;
		}

		float period = numTimerTickFuncs * TIMER_PERIOD;

		for (int i = 0; i < 4; i++) {
			auto &waveformParameters = newState.aoutWaveformParameters[i];

			if (waveformParameters.waveform != WAVEFORM_NONE) {
				g_aoutWaveFormFunc[i] = getWaveformFunction(waveformParameters);
				g_aoutDutyCycles[i] = g_dutyCycle;

				g_aoutPhi[i] = waveformParameters.phaseShift / 360.0f;
				g_aoutDphi[i] = 2.0f * M_PI_F * waveformParameters.frequency * period;

				if (i < 2) {
					DAC_GetValueRange(newState.aout_dac7760[i].outputRange, g_aoutMin[i], g_aoutMax[i]);
				} else {
					DACDual_GetValueRange(g_aoutMin[i], g_aoutMax[i]);
				}
			}
		}

		memcpy(g_timerTickFunc, timerTickFunc, sizeof(timerTickFunc));
		g_numTimerTickFuncs = numTimerTickFuncs;

		memcpy(g_aoutWaveformParameters, newState.aoutWaveformParameters, sizeof(newState.aoutWaveformParameters));

		g_nextTimerTickFunc = 0;
	}

	if (memcmp(g_doutWaveformParameters, newState.doutWaveformParameters, sizeof(newState.doutWaveformParameters)) != 0) {
		for (int i = 0; i < 8; i++) {
			auto &waveformParameters = newState.doutWaveformParameters[i];

			if (waveformParameters.waveform != WAVEFORM_NONE) {
				g_doutWaveFormFunc[i] = getWaveformFunction(waveformParameters);
				g_doutDutyCycles[i] = g_dutyCycle;

				g_doutPhi[i] = waveformParameters.phaseShift / 360.0f;
				g_doutDphi[i] = 2.0f * M_PI_F * waveformParameters.frequency * TIMER_PERIOD;
			}
		}

		memcpy(g_doutWaveformParameters, newState.doutWaveformParameters, sizeof(newState.doutWaveformParameters));
	}

	__enable_irq();
}

void FuncGen_onTimerPeriodElapsed() {
	//RESET_PIN(DOUT0_GPIO_Port, DOUT0_Pin);

	g_timerTickFunc[g_nextTimerTickFunc]();

	if (++g_nextTimerTickFunc == g_numTimerTickFuncs) {
		g_nextTimerTickFunc = 0;
	}

	for (int i = 0; i < 8; i++) {
		FuncGen_DOUT(i);
	}

	//SET_PIN(DOUT0_GPIO_Port, DOUT0_Pin);
}
