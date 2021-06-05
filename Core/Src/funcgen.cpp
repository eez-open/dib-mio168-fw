
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
#include "dlog.h"
#include "utils.h"

extern "C" TIM_HandleTypeDef htim7;

#define M_PI_F ((float)M_PI)

#define TICKS_RESERVE 10.0f
#define TICKS_ADC 8.0f
#define TICKS_ADC_DLOG 10.0f
#define TICKS_AOUT 4.0f
#define TICKS_DOUT_FIXED 1.5f
#define TICKS_DOUT_PER_DOUT 0.35f

static uint16_t TIMER_PERIOD;

typedef float (*WaveformFunction)(float);

//#define DEBUG_TIMING

#ifdef DEBUG_TIMING
float m_debugAOUT1;
float m_debugAOUT2;
float m_debugAOUT3;
float m_debugAOUT4;
float m_debugADC;
float m_debugDOUT;
float m_debugTotal;
float m_debugTicks;
#define TIMING_BEGIN() uint16_t startCnt = TIM7->CNT;
#define TIMING_END(var) { uint16_t endCnt = TIM7->CNT; if (endCnt > startCnt) { var = (endCnt - startCnt) / 90.0f; } else { var = (TIMER_PERIOD - startCnt + endCnt) / 90.0f; } }
#else
#define TIMING_BEGIN() (void)0
#define TIMING_END(var) (void)0
#endif

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
TimerTickFunc g_timerTickFunc[6];
int g_numTimerTickFuncs;

////////////////////////////////////////////////////////////////////////////////

float dcf(float t) {
	return 0.0f;
}

float sineHalfRectifiedf(float t) {
	if (t < M_PI_F) {
		return 2.0f * sinf(t);
	}

	return 0.0f;
}

float sineFullRectifiedf(float t) {
	if (t < M_PI_F) {
		return 2.0f * sinf(t);
	}

	return 2.0f * sinf(t - M_PI_F);
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
	} else if (waveformParameters.waveform == WAVEFORM_HALF_RECTIFIED) {
		return sineHalfRectifiedf;
	} else if (waveformParameters.waveform == WAVEFORM_FULL_RECTIFIED) {
		return sineFullRectifiedf;
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
	TIMING_BEGIN();

	FuncGen_AOUT(0);

	TIMING_END(m_debugAOUT1);
}

void FuncGen_AOUT2() {
	TIMING_BEGIN();

	FuncGen_AOUT(1);

	TIMING_END(m_debugAOUT2);
}

void FuncGen_AOUT3() {
	TIMING_BEGIN();

	FuncGen_AOUT(2);

	TIMING_END(m_debugAOUT3);
}

void FuncGen_AOUT4() {
	TIMING_BEGIN();

	FuncGen_AOUT(3);

	TIMING_END(m_debugAOUT4);
}

////////////////////////////////////////////////////////////////////////////////

void ADC_MeasureTickFromFuncGen() {
	TIMING_BEGIN();
	ADC_MeasureTickFromFuncGen(TIMER_PERIOD);
	TIMING_END(m_debugADC);
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
		g_doutPhi[i] -= 2.0f * M_PI_F;
	} else {
		g_doutPhi[i] -= 0.0f;
	}

	Dout_SetPinState(i, value > 0.5f ? 1 : 0);
}

void FuncGen_DOUT() {
	TIMING_BEGIN();

	for (int i = 0; i < 8; i++) {
		FuncGen_DOUT(i);
	}

	TIMING_END(m_debugDOUT);
}

////////////////////////////////////////////////////////////////////////////////

void FuncGen_Setup() {
	TIM7->PSC = 0;
}

void FuncGen_SetParams(SetParams &newState) {
	float ticks = TICKS_RESERVE + TICKS_ADC + TICKS_DOUT_FIXED;

	for (int i = 0; i < 4; i++) {
		auto &waveformParameters = newState.aoutWaveformParameters[i];
		if (waveformParameters.waveform != WAVEFORM_NONE) {
			ticks += TICKS_AOUT;
		}
	}

	for (int i = 0; i < 8; i++) {
		auto &waveformParameters = newState.doutWaveformParameters[i];
		if (waveformParameters.waveform != WAVEFORM_NONE) {
			ticks += TICKS_DOUT_PER_DOUT;
		}
	}

	if (ADC_DLOG_started) {
		ticks += TICKS_ADC_DLOG;
	}

	ticks = ceilf(ticks);

	auto NEW_TIMER_PERIOD = (uint16_t)(ticks * 90.0);

	auto timerPeriodChanged = NEW_TIMER_PERIOD != TIMER_PERIOD;
	auto aoutParamsChanged = memcmp((void *)g_aoutWaveformParameters, newState.aoutWaveformParameters, sizeof(newState.aoutWaveformParameters)) != 0;
	auto doutParamsChanged = memcmp(g_doutWaveformParameters, newState.doutWaveformParameters, sizeof(newState.doutWaveformParameters)) != 0;
	if (!timerPeriodChanged && !aoutParamsChanged && !doutParamsChanged) {
		return;
	}

	TIMER_PERIOD = NEW_TIMER_PERIOD;

	HAL_TIM_Base_Stop_IT(&htim7);

	__disable_irq();

#ifdef DEBUG_TIMING
	m_debugTicks = ticks;
	m_debugAOUT1 = 0;
	m_debugAOUT2 = 0;
	m_debugAOUT3 = 0;
	m_debugAOUT4 = 0;
	m_debugADC = 0;
	m_debugDOUT = 0;
#endif

	TimerTickFunc timerTickFunc[6];
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

	timerTickFunc[numTimerTickFuncs++] = FuncGen_DOUT;
	timerTickFunc[numTimerTickFuncs++] = ADC_MeasureTickFromFuncGen;

	if (timerPeriodChanged || aoutParamsChanged) {
		for (int i = 0; i < 4; i++) {
			auto &waveformParameters = newState.aoutWaveformParameters[i];

			if (waveformParameters.waveform != WAVEFORM_NONE) {
				g_aoutWaveFormFunc[i] = getWaveformFunction(waveformParameters);
				g_aoutDutyCycles[i] = g_dutyCycle;

				g_aoutPhi[i] = waveformParameters.phaseShift / 360.0f;
				g_aoutDphi[i] = 2.0 * M_PI * waveformParameters.frequency * TIMER_PERIOD / 90000000.0;

				if (i < 2) {
					DAC_GetValueRange(newState.aout_dac7760[i].outputRange, g_aoutMin[i], g_aoutMax[i]);
				} else {
					DACDual_GetValueRange(g_aoutMin[i], g_aoutMax[i]);
				}
			}
		}

		memcpy(g_aoutWaveformParameters, newState.aoutWaveformParameters, sizeof(newState.aoutWaveformParameters));
	}

	if (timerPeriodChanged || doutParamsChanged) {
		for (int i = 0; i < 8; i++) {
			auto &waveformParameters = newState.doutWaveformParameters[i];

			if (waveformParameters.waveform != WAVEFORM_NONE) {
				g_doutWaveFormFunc[i] = getWaveformFunction(waveformParameters);
				g_doutDutyCycles[i] = g_dutyCycle;

				g_doutPhi[i] = waveformParameters.phaseShift / 360.0f;
				g_doutDphi[i] = 2.0 * M_PI * waveformParameters.frequency * TIMER_PERIOD / 90000000.0;
			}
		}

		memcpy(g_doutWaveformParameters, newState.doutWaveformParameters, sizeof(newState.doutWaveformParameters));
	}

	memcpy(g_timerTickFunc, timerTickFunc, sizeof(timerTickFunc));
	g_numTimerTickFuncs = numTimerTickFuncs;

	__enable_irq();

	TIM7->ARR = TIMER_PERIOD - 1;
	HAL_TIM_Base_Start_IT(&htim7);
}

void FuncGen_onTimerPeriodElapsed() {
	//RESET_PIN(DOUT0_GPIO_Port, DOUT0_Pin);

	TIMING_BEGIN();

	for (int i = 0; i < g_numTimerTickFuncs; i++) {
		g_timerTickFunc[i]();
	}

	TIMING_END(m_debugTotal);

	//SET_PIN(DOUT0_GPIO_Port, DOUT0_Pin);
}
