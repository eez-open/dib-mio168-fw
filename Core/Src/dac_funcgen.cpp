#define _USE_MATH_DEFINES
#include <math.h>
#include <memory.h>

#include "main.h"
#include "firmware.h"
#include "dac_funcgen.h"
#include "dac7760.h"
#include "dac7563.h"
#include "adc.h"
#include "utils.h"

extern "C" TIM_HandleTypeDef htim7;

#define M_PI_F ((float)M_PI)

static const float TIMER_PERIOD = 1.0f / 60000.0f;

////////////////////////////////////////////////////////////////////////////////

typedef float (*WaveformFunction)(float);
WaveformFunction g_waveFormFunc[4];

float g_min[4];
float g_max[4];

float g_phi[4];
float g_dphi[4];

WaveformParameters g_dacWaveformParameters[4];

typedef void (*TimerTickFunc)(void);
TimerTickFunc g_timerTickFunc[5];
int g_numTimerTickFuncs;
int g_nextTimerTickFunc;

////////////////////////////////////////////////////////////////////////////////

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

static float g_pulseWidth;

float pulsef(float t) {
	if (t < g_pulseWidth * 2.0f * M_PI_F / 100.0f) {
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

float dcf(float t) {
	return 1.0f;
}

WaveformFunction getWaveformFunction(WaveformParameters &waveformParameters) {
	if (waveformParameters.waveform == WAVEFORM_SINE_WAVE) {
		return sinf;
	} else if (waveformParameters.waveform == WAVEFORM_TRIANGLE) {
		return trianglef;
	} else if (waveformParameters.waveform == WAVEFORM_SQUARE_WAVE) {
		return squaref;
	} else if (waveformParameters.waveform == WAVEFORM_PULSE) {
		g_pulseWidth = waveformParameters.pulseWidth;
		return pulsef;
	} else if (waveformParameters.waveform == WAVEFORM_SAWTOOTH) {
		return sawtoothf;
	} else if (waveformParameters.waveform == WAVEFORM_ARBITRARY)  {
		return arbitraryf;
	} else {
		return dcf;
	}
}

////////////////////////////////////////////////////////////////////////////////

void DAC_FuncGen_AOUT(int i) {
	auto &waveformParameters = g_dacWaveformParameters[i];

	g_phi[i] += g_dphi[i];
	if (g_phi[i] >= 2.0f * M_PI_F) {
		g_phi[i] = 0;
	}

	float value = waveformParameters.offset + waveformParameters.amplitude * g_waveFormFunc[i](g_phi[i]);

	value = 65535.0f * (value - g_min[i]) / (g_max[i] - g_min[i]);

	if (i < 2) {
		DAC_SetValue(i, value);
	} else {
		DACDual_SetValue(i - 2, value);
	}
}

void DAC_FuncGen_AOUT1() {
	DAC_FuncGen_AOUT(0);
}

void DAC_FuncGen_AOUT2() {
	DAC_FuncGen_AOUT(1);
}

void DAC_FuncGen_AOUT3() {
	DAC_FuncGen_AOUT(2);
}

void DAC_FuncGen_AOUT4() {
	DAC_FuncGen_AOUT(3);
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

void DAC_FuncGen_Setup() {
	g_timerTickFunc[0] = ADC_MeasureTickFromFuncGen30Ksps;
	g_timerTickFunc[1] = TimerTickFuncIdle;
	g_numTimerTickFuncs = 2;

	TIM7->ARR = (uint16_t)(TIMER_PERIOD * 10000000) - 1;
	HAL_TIM_Base_Start_IT(&htim7);
}

void DAC_FuncGen_SetParams(SetParams &newState) {
	if (memcmp(g_dacWaveformParameters, newState.dacWaveformParameters, sizeof(newState.dacWaveformParameters)) != 0) {
		TimerTickFunc timerTickFunc[5];
		int numTimerTickFuncs = 0;

		if (newState.dacWaveformParameters[0].waveform != WAVEFORM_NONE) {
			timerTickFunc[numTimerTickFuncs++] = DAC_FuncGen_AOUT1;
		}
		if (newState.dacWaveformParameters[1].waveform != WAVEFORM_NONE) {
			timerTickFunc[numTimerTickFuncs++] = DAC_FuncGen_AOUT2;
		}
		if (newState.dacWaveformParameters[2].waveform != WAVEFORM_NONE) {
			timerTickFunc[numTimerTickFuncs++] = DAC_FuncGen_AOUT3;
		}
		if (newState.dacWaveformParameters[3].waveform != WAVEFORM_NONE) {
			timerTickFunc[numTimerTickFuncs++] = DAC_FuncGen_AOUT4;
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
			auto &waveformParameters = newState.dacWaveformParameters[i];

			if (waveformParameters.waveform != WAVEFORM_NONE) {
				g_waveFormFunc[i] = getWaveformFunction(waveformParameters);

				g_phi[i] = waveformParameters.phaseShift / 360.0f;
				g_dphi[i] = 2.0f * M_PI_F * waveformParameters.frequency * period;

				if (i < 2) {
					DAC_GetValueRange(newState.aout_dac7760[i].outputRange, g_min[i], g_max[i]);
				} else {
					DACDual_GetValueRange(g_min[i], g_max[i]);
				}
			}
		}

		__disable_irq();

		memcpy(g_timerTickFunc, timerTickFunc, sizeof(timerTickFunc));
		g_numTimerTickFuncs = numTimerTickFuncs;

		memcpy(g_dacWaveformParameters, newState.dacWaveformParameters, sizeof(newState.dacWaveformParameters));

		g_nextTimerTickFunc = 0;

		__enable_irq();
	}
}

void DAC_FuncGen_onTimerPeriodElapsed() {
	RESET_PIN(DOUT0_GPIO_Port, DOUT0_Pin);

	g_timerTickFunc[g_nextTimerTickFunc]();

	if (++g_nextTimerTickFunc == g_numTimerTickFuncs) {
		g_nextTimerTickFunc = 0;
	}

	SET_PIN(DOUT0_GPIO_Port, DOUT0_Pin);
}
