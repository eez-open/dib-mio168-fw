
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
#define TIMING_BEGIN() uint16_t startCnt = TIM7->CNT;
#define TIMING_END(var) { uint16_t endCnt = TIM7->CNT; if (endCnt > startCnt) { var = (endCnt - startCnt) / 88.0f; } else { var = (1375 - startCnt + endCnt) / 88.0f; } }
#else
#define TIMING_BEGIN() (void)0
#define TIMING_END(var) (void)0
#endif

////////////////////////////////////////////////////////////////////////////////

typedef void (*TimerTickFunc)(void);

struct State {
	////////////////////////////////////////////////////////////////////////////////

	WaveformFunction aoutWaveFormFunc[4];

	float aoutMin[4];
	float aoutScale[4];

	float aoutPhi[4];
	float aoutDphi[4];

	float aoutDutyCycles[4];

	WaveformParameters aoutWaveformParameters[4];

	////////////////////////////////////////////////////////////////////////////////

	WaveformFunction doutWaveFormFunc[8];

	float doutPhi[8];
	float doutDphi[8];

	float doutDutyCycles[8];

	WaveformParameters doutWaveformParameters[8];

	////////////////////////////////////////////////////////////////////////////////

	TimerTickFunc tickFunc[10][3];
	int tickFuncIndex;
	int tickFuncM;
	int tickFuncN;
};

State g_states[2];
State *g_pCurrentState;

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
	return 2.0f * sinf(t / 2.0f);
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
	auto &waveformParameters = g_pCurrentState->aoutWaveformParameters[i];

	g_dutyCycle = g_pCurrentState->aoutDutyCycles[i];
	float value;
	if (waveformParameters.waveform == WAVEFORM_DC) {
		value = waveformParameters.amplitude;
	} else {
		value = waveformParameters.offset + waveformParameters.amplitude * g_pCurrentState->aoutWaveFormFunc[i](g_pCurrentState->aoutPhi[i]) / 2.0f;
	}

	g_pCurrentState->aoutPhi[i] += g_pCurrentState->aoutDphi[i];
	if (g_pCurrentState->aoutPhi[i] >= 2.0f * M_PI_F) {
		g_pCurrentState->aoutPhi[i] -= 2.0f * M_PI_F;
	}

	value = 65535.0f * (value - g_pCurrentState->aoutMin[i] ) / g_pCurrentState->aoutScale[i];

	if (i < 2) {
		DAC_SetValue_FromFuncGen(i, value);
	} else {
		DACDual_SetValue_FromFuncGen(i - 2, value);
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

void FuncGen_ADC() {
	TIMING_BEGIN();
	ADC_Tick(1.0f / 32000.0f);
	TIMING_END(m_debugADC);
}

////////////////////////////////////////////////////////////////////////////////

void FuncGen_DOUT(int i) {
	auto &waveformParameters = g_pCurrentState->doutWaveformParameters[i];

	if (waveformParameters.waveform == WAVEFORM_NONE) {
		return;
	}

	g_dutyCycle = g_pCurrentState->doutDutyCycles[i];
	float value = g_pCurrentState->doutWaveFormFunc[i](g_pCurrentState->doutPhi[i]);

	g_pCurrentState->doutPhi[i] += g_pCurrentState->doutDphi[i];
	if (g_pCurrentState->doutPhi[i] >= 2.0f * M_PI_F) {
		g_pCurrentState->doutPhi[i] -= 2.0f * M_PI_F;
	}

	Dout_SetPinState(i, value > 0.5f ? 1 : 0);
}

void FuncGen_DOUT() {
	TIMING_BEGIN();

	FuncGen_DOUT(0);
	FuncGen_DOUT(1);
	FuncGen_DOUT(2);
	FuncGen_DOUT(3);
	FuncGen_DOUT(4);
	FuncGen_DOUT(5);
	FuncGen_DOUT(6);
	FuncGen_DOUT(7);

	TIMING_END(m_debugDOUT);
}

void FuncGen_Void() {
}

////////////////////////////////////////////////////////////////////////////////

static void initTickFuncs_dlog_ac_dout(State *pNewState, int dout, float &doutPeriod, int aout, TimerTickFunc *aoutFunc, float *aoutPeriod) {
	if (aout == 0) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = FuncGen_DOUT;

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 1;

		doutPeriod = 2.0f * 1375.0f / 88000000.0f;
	} else if (aout == 1) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = FuncGen_DOUT;
		pNewState->tickFunc[2][0] = FuncGen_ADC;
		pNewState->tickFunc[3][0] = aoutFunc[0];

		pNewState->tickFuncM = 4;
		pNewState->tickFuncN = 1;

		doutPeriod = 4.0f * 1375.0f / 88000000.0f;
		aoutPeriod[0] = 4.0f * 1375.0f / 88000000.0f;
	} else if (aout == 2) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = FuncGen_DOUT;
		pNewState->tickFunc[2][0] = FuncGen_ADC;
		pNewState->tickFunc[3][0] = aoutFunc[0];
		pNewState->tickFunc[4][0] = FuncGen_ADC;
		pNewState->tickFunc[5][0] = aoutFunc[1];

		pNewState->tickFuncM = 6;
		pNewState->tickFuncN = 1;

		doutPeriod = 6.0f * 1375.0f / 88000000.0f;
		aoutPeriod[0] = 6.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 6.0f * 1375.0f / 88000000.0f;
	} else if (aout == 3) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = FuncGen_DOUT;
		pNewState->tickFunc[2][0] = FuncGen_ADC;
		pNewState->tickFunc[3][0] = aoutFunc[0];
		pNewState->tickFunc[4][0] = FuncGen_ADC;
		pNewState->tickFunc[5][0] = aoutFunc[1];
		pNewState->tickFunc[6][0] = FuncGen_ADC;
		pNewState->tickFunc[7][0] = aoutFunc[2];

		pNewState->tickFuncM = 8;
		pNewState->tickFuncN = 1;

		doutPeriod = 8.0f * 1375.0f / 88000000.0f;
		aoutPeriod[0] = 8.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 8.0f * 1375.0f / 88000000.0f;
		aoutPeriod[2] = 8.0f * 1375.0f / 88000000.0f;
	} else {
		pNewState->tickFunc[0][0] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = FuncGen_DOUT;
		pNewState->tickFunc[2][0] = FuncGen_ADC;
		pNewState->tickFunc[3][0] = aoutFunc[0];
		pNewState->tickFunc[4][0] = FuncGen_ADC;
		pNewState->tickFunc[5][0] = aoutFunc[1];
		pNewState->tickFunc[6][0] = FuncGen_ADC;
		pNewState->tickFunc[7][0] = aoutFunc[2];
		pNewState->tickFunc[8][0] = FuncGen_ADC;
		pNewState->tickFunc[9][0] = aoutFunc[3];

		pNewState->tickFuncM = 10;
		pNewState->tickFuncN = 1;

		doutPeriod = 10.0f * 1375.0f / 88000000.0f;
		aoutPeriod[0] = 10.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 10.0f * 1375.0f / 88000000.0f;
		aoutPeriod[2] = 10.0f * 1375.0f / 88000000.0f;
		aoutPeriod[3] = 10.0f * 1375.0f / 88000000.0f;
	}
}

static void initTickFuncs_dlog_ac_aout(State *pNewState, int aout, TimerTickFunc *aoutFunc, float *aoutPeriod) {
	if (aout == 0) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = FuncGen_Void;

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 1;
	} else if (aout == 1) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = aoutFunc[0];

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 1;

		aoutPeriod[0] = 2.0f * 1375.0f / 88000000.0f;
	} else if (aout == 2) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = aoutFunc[0];
		pNewState->tickFunc[2][0] = FuncGen_ADC;
		pNewState->tickFunc[3][0] = aoutFunc[1];

		pNewState->tickFuncM = 4;
		pNewState->tickFuncN = 1;

		aoutPeriod[0] = 4.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 4.0f * 1375.0f / 88000000.0f;
	} else if (aout == 3) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = aoutFunc[0];
		pNewState->tickFunc[2][0] = FuncGen_ADC;
		pNewState->tickFunc[3][0] = aoutFunc[1];
		pNewState->tickFunc[4][0] = FuncGen_ADC;
		pNewState->tickFunc[5][0] = aoutFunc[2];

		pNewState->tickFuncM = 6;
		pNewState->tickFuncN = 1;

		aoutPeriod[0] = 6.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 6.0f * 1375.0f / 88000000.0f;
		aoutPeriod[2] = 6.0f * 1375.0f / 88000000.0f;
	} else {
		pNewState->tickFunc[0][0] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = aoutFunc[0];
		pNewState->tickFunc[2][0] = FuncGen_ADC;
		pNewState->tickFunc[3][0] = aoutFunc[1];
		pNewState->tickFunc[4][0] = FuncGen_ADC;
		pNewState->tickFunc[5][0] = aoutFunc[2];
		pNewState->tickFunc[6][0] = FuncGen_ADC;
		pNewState->tickFunc[7][0] = aoutFunc[3];

		pNewState->tickFuncM = 8;
		pNewState->tickFuncN = 1;

		aoutPeriod[0] = 8.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 8.0f * 1375.0f / 88000000.0f;
		aoutPeriod[2] = 8.0f * 1375.0f / 88000000.0f;
		aoutPeriod[3] = 8.0f * 1375.0f / 88000000.0f;
	}
}

static void initTickFuncs_dlog_dout(State *pNewState, int dout, float &doutPeriod, int aout, TimerTickFunc *aoutFunc, float *aoutPeriod) {
	if (aout == 0) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = FuncGen_DOUT;

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 1;

		doutPeriod = 2.0f * 1375.0f / 88000000.0f;
	} else if (aout == 1) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;  pNewState->tickFunc[0][1] = FuncGen_Void;
		pNewState->tickFunc[1][0] = FuncGen_DOUT; pNewState->tickFunc[1][1] = aoutFunc[0];

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 2;

		doutPeriod = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[0] = 2.0f * 1375.0f / 88000000.0f;
	} else if (aout == 2) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;  pNewState->tickFunc[0][1] = FuncGen_Void;
		pNewState->tickFunc[1][0] = FuncGen_DOUT; pNewState->tickFunc[1][1] = aoutFunc[0];
		pNewState->tickFunc[2][0] = FuncGen_ADC;  pNewState->tickFunc[2][1] = FuncGen_Void;
		pNewState->tickFunc[3][0] = FuncGen_DOUT; pNewState->tickFunc[3][1] = aoutFunc[1];

		pNewState->tickFuncM = 4;
		pNewState->tickFuncN = 2;

		doutPeriod = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[0] = 4.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 4.0f * 1375.0f / 88000000.0f;
	} else if (aout == 3) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;  pNewState->tickFunc[0][1] = FuncGen_Void;
		pNewState->tickFunc[1][0] = FuncGen_DOUT; pNewState->tickFunc[1][1] = aoutFunc[0];
		pNewState->tickFunc[2][0] = FuncGen_ADC;  pNewState->tickFunc[2][1] = FuncGen_Void;
		pNewState->tickFunc[3][0] = aoutFunc[1];  pNewState->tickFunc[3][1] = aoutFunc[2];

		pNewState->tickFuncM = 4;
		pNewState->tickFuncN = 2;

		doutPeriod = 4.0f * 1375.0f / 88000000.0f;
		aoutPeriod[0] = 4.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 4.0f * 1375.0f / 88000000.0f;
		aoutPeriod[2] = 4.0f * 1375.0f / 88000000.0f;
	} else {
		pNewState->tickFunc[0][0] = FuncGen_ADC;  pNewState->tickFunc[0][1] = FuncGen_Void;
		pNewState->tickFunc[1][0] = FuncGen_DOUT; pNewState->tickFunc[1][1] = aoutFunc[0];
		pNewState->tickFunc[2][0] = FuncGen_ADC;  pNewState->tickFunc[2][1] = FuncGen_Void;
		pNewState->tickFunc[3][0] = FuncGen_DOUT; pNewState->tickFunc[3][1] = aoutFunc[1];
		pNewState->tickFunc[4][0] = FuncGen_ADC;  pNewState->tickFunc[4][1] = FuncGen_Void;
		pNewState->tickFunc[5][0] = FuncGen_DOUT; pNewState->tickFunc[5][1] = aoutFunc[2];
		pNewState->tickFunc[6][0] = FuncGen_ADC;  pNewState->tickFunc[6][1] = FuncGen_Void;
		pNewState->tickFunc[7][0] = FuncGen_DOUT; pNewState->tickFunc[7][1] = aoutFunc[3];

		pNewState->tickFuncM = 8;
		pNewState->tickFuncN = 2;

		doutPeriod = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[0] = 8.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 8.0f * 1375.0f / 88000000.0f;
		aoutPeriod[2] = 8.0f * 1375.0f / 88000000.0f;
		aoutPeriod[3] = 8.0f * 1375.0f / 88000000.0f;
	}
}

static void initTickFuncs_dlog_aout(State *pNewState, int aout, TimerTickFunc *aoutFunc, float *aoutPeriod) {
	if (aout == 0) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = FuncGen_Void;

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 1;
	} else if (aout == 1) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = aoutFunc[0];

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 1;

		aoutPeriod[0] = 2.0f * 1375.0f / 88000000.0f;
	} else if (aout == 2) {
		pNewState->tickFunc[0][0] = FuncGen_ADC; pNewState->tickFunc[0][1] = FuncGen_Void;
		pNewState->tickFunc[1][0] = aoutFunc[0]; pNewState->tickFunc[1][1] = aoutFunc[1];

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 2;

		aoutPeriod[0] = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 2.0f * 1375.0f / 88000000.0f;
	} else if (aout == 3) {
		pNewState->tickFunc[0][0] = FuncGen_ADC; pNewState->tickFunc[0][1] = FuncGen_Void;
		pNewState->tickFunc[1][0] = aoutFunc[0]; pNewState->tickFunc[1][1] = aoutFunc[1];
		pNewState->tickFunc[2][0] = FuncGen_ADC; pNewState->tickFunc[2][1] = FuncGen_Void;
		pNewState->tickFunc[3][0] = aoutFunc[0]; pNewState->tickFunc[3][1] = aoutFunc[2];

		pNewState->tickFuncM = 4;
		pNewState->tickFuncN = 2;

		aoutPeriod[0] = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 4.0f * 1375.0f / 88000000.0f;
		aoutPeriod[2] = 4.0f * 1375.0f / 88000000.0f;
	} else {
		pNewState->tickFunc[0][0] = FuncGen_ADC; pNewState->tickFunc[0][1] = FuncGen_Void;
		pNewState->tickFunc[1][0] = aoutFunc[0]; pNewState->tickFunc[1][1] = aoutFunc[1];
		pNewState->tickFunc[2][0] = FuncGen_ADC; pNewState->tickFunc[2][1] = FuncGen_Void;
		pNewState->tickFunc[3][0] = aoutFunc[0]; pNewState->tickFunc[3][1] = aoutFunc[2];
		pNewState->tickFunc[4][0] = FuncGen_ADC; pNewState->tickFunc[4][1] = FuncGen_Void;
		pNewState->tickFunc[5][0] = aoutFunc[0]; pNewState->tickFunc[5][1] = aoutFunc[3];

		pNewState->tickFuncM = 6;
		pNewState->tickFuncN = 2;

		aoutPeriod[0] = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 6.0f * 1375.0f / 88000000.0f;
		aoutPeriod[2] = 6.0f * 1375.0f / 88000000.0f;
		aoutPeriod[3] = 6.0f * 1375.0f / 88000000.0f;
	}
}

static void initTickFuncs_ac_dout(State *pNewState, int dout, float &doutPeriod, int aout, TimerTickFunc *aoutFunc, float *aoutPeriod) {
	if (aout == 0) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = FuncGen_DOUT;

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 1;

		doutPeriod = 2.0f * 1375.0f / 88000000.0f;
	} else if (aout == 1) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;  pNewState->tickFunc[0][1] = FuncGen_Void;
		pNewState->tickFunc[1][0] = FuncGen_DOUT; pNewState->tickFunc[1][1] = aoutFunc[0];

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 2;

		doutPeriod = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[0] = 2.0f * 1375.0f / 88000000.0f;
	} else if (aout == 2) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;  pNewState->tickFunc[0][1] = FuncGen_Void;
		pNewState->tickFunc[1][0] = FuncGen_DOUT; pNewState->tickFunc[1][1] = aoutFunc[0];
		pNewState->tickFunc[2][0] = FuncGen_ADC;  pNewState->tickFunc[2][1] = FuncGen_Void;
		pNewState->tickFunc[3][0] = FuncGen_DOUT; pNewState->tickFunc[3][1] = aoutFunc[1];

		pNewState->tickFuncM = 4;
		pNewState->tickFuncN = 2;

		doutPeriod = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[0] = 4.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 4.0f * 1375.0f / 88000000.0f;
	} else if (aout == 3) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;  pNewState->tickFunc[0][1] = FuncGen_Void;
		pNewState->tickFunc[1][0] = FuncGen_DOUT; pNewState->tickFunc[1][1] = aoutFunc[0];
		pNewState->tickFunc[2][0] = FuncGen_ADC;  pNewState->tickFunc[2][1] = FuncGen_Void;
		pNewState->tickFunc[3][0] = aoutFunc[1];  pNewState->tickFunc[3][1] = aoutFunc[2];

		pNewState->tickFuncM = 4;
		pNewState->tickFuncN = 2;

		doutPeriod = 4.0f * 1375.0f / 88000000.0f;
		aoutPeriod[0] = 4.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 4.0f * 1375.0f / 88000000.0f;
		aoutPeriod[2] = 4.0f * 1375.0f / 88000000.0f;
	} else {
		pNewState->tickFunc[0][0] = FuncGen_ADC;  pNewState->tickFunc[0][1] = FuncGen_Void;
		pNewState->tickFunc[1][0] = FuncGen_DOUT; pNewState->tickFunc[1][1] = aoutFunc[0];
		pNewState->tickFunc[2][0] = FuncGen_ADC;  pNewState->tickFunc[2][1] = FuncGen_Void;
		pNewState->tickFunc[3][0] = FuncGen_DOUT; pNewState->tickFunc[3][1] = aoutFunc[1];
		pNewState->tickFunc[4][0] = FuncGen_ADC;  pNewState->tickFunc[4][1] = FuncGen_Void;
		pNewState->tickFunc[5][0] = FuncGen_DOUT; pNewState->tickFunc[5][1] = aoutFunc[2];
		pNewState->tickFunc[6][0] = FuncGen_ADC;  pNewState->tickFunc[6][1] = FuncGen_Void;
		pNewState->tickFunc[7][0] = FuncGen_DOUT; pNewState->tickFunc[7][1] = aoutFunc[3];

		pNewState->tickFuncM = 8;
		pNewState->tickFuncN = 2;

		doutPeriod = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[0] = 8.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 8.0f * 1375.0f / 88000000.0f;
		aoutPeriod[2] = 8.0f * 1375.0f / 88000000.0f;
		aoutPeriod[3] = 8.0f * 1375.0f / 88000000.0f;
	}
}

static void initTickFuncs_ac_aout(State *pNewState, int aout, TimerTickFunc *aoutFunc, float *aoutPeriod) {
	if (aout == 0) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = FuncGen_Void;

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 1;
	} else if (aout == 1) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = aoutFunc[0];

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 1;

		aoutPeriod[0] = 2.0f * 1375.0f / 88000000.0f;
	} else if (aout == 2) {
		pNewState->tickFunc[0][0] = FuncGen_ADC; pNewState->tickFunc[0][1] = FuncGen_Void;
		pNewState->tickFunc[1][0] = aoutFunc[0]; pNewState->tickFunc[1][1] = aoutFunc[1];

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 2;

		aoutPeriod[0] = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 2.0f * 1375.0f / 88000000.0f;
	} else if (aout == 3) {
		pNewState->tickFunc[0][0] = FuncGen_ADC; pNewState->tickFunc[0][1] = FuncGen_Void;
		pNewState->tickFunc[1][0] = aoutFunc[0]; pNewState->tickFunc[1][1] = aoutFunc[1];
		pNewState->tickFunc[2][0] = FuncGen_ADC; pNewState->tickFunc[2][1] = FuncGen_Void;
		pNewState->tickFunc[3][0] = aoutFunc[0]; pNewState->tickFunc[3][1] = aoutFunc[2];

		pNewState->tickFuncM = 4;
		pNewState->tickFuncN = 2;

		aoutPeriod[0] = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 4.0f * 1375.0f / 88000000.0f;
		aoutPeriod[2] = 4.0f * 1375.0f / 88000000.0f;
	} else {
		pNewState->tickFunc[0][0] = FuncGen_ADC; pNewState->tickFunc[0][1] = FuncGen_Void;
		pNewState->tickFunc[1][0] = aoutFunc[0]; pNewState->tickFunc[1][1] = aoutFunc[1];
		pNewState->tickFunc[2][0] = FuncGen_ADC; pNewState->tickFunc[2][1] = FuncGen_Void;
		pNewState->tickFunc[3][0] = aoutFunc[0]; pNewState->tickFunc[3][1] = aoutFunc[2];
		pNewState->tickFunc[4][0] = FuncGen_ADC; pNewState->tickFunc[4][1] = FuncGen_Void;
		pNewState->tickFunc[5][0] = aoutFunc[0]; pNewState->tickFunc[5][1] = aoutFunc[3];

		pNewState->tickFuncM = 6;
		pNewState->tickFuncN = 2;

		aoutPeriod[0] = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 6.0f * 1375.0f / 88000000.0f;
		aoutPeriod[2] = 6.0f * 1375.0f / 88000000.0f;
		aoutPeriod[3] = 6.0f * 1375.0f / 88000000.0f;
	}
}

static void initTickFuncs_dout(State *pNewState, int dout, float &doutPeriod, int aout, TimerTickFunc *aoutFunc, float *aoutPeriod) {
	if (aout == 0) {
		pNewState->tickFunc[0][0] = FuncGen_DOUT; pNewState->tickFunc[0][1] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = FuncGen_DOUT; pNewState->tickFunc[1][1] = FuncGen_Void;

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 2;

		doutPeriod = 1375.0f / 88000000.0f;
	} else if (aout == 1) {
		pNewState->tickFunc[0][0] = FuncGen_DOUT; pNewState->tickFunc[0][1] = aoutFunc[0]; pNewState->tickFunc[0][2] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = FuncGen_DOUT; pNewState->tickFunc[1][1] = aoutFunc[0]; pNewState->tickFunc[1][2] = FuncGen_Void;

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 3;

		doutPeriod = 1375.0f / 88000000.0f;
		aoutPeriod[0] = 1375.0f / 88000000.0f;
	} else if (aout == 2) {
		pNewState->tickFunc[0][0] = FuncGen_DOUT; pNewState->tickFunc[0][1] = aoutFunc[0]; pNewState->tickFunc[0][2] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = FuncGen_DOUT; pNewState->tickFunc[1][1] = aoutFunc[0]; pNewState->tickFunc[1][2] = aoutFunc[1];

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 3;

		doutPeriod = 1375.0f / 88000000.0f;
		aoutPeriod[0] = 1375.0f / 88000000.0f;
		aoutPeriod[1] = 2.0f * 1375.0f / 88000000.0f;
	} else if (aout == 3) {
		pNewState->tickFunc[0][0] = FuncGen_DOUT; pNewState->tickFunc[0][1] = aoutFunc[0]; pNewState->tickFunc[0][2] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = FuncGen_DOUT; pNewState->tickFunc[1][1] = aoutFunc[1]; pNewState->tickFunc[1][2] = aoutFunc[2];

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 3;

		doutPeriod = 1375.0f / 88000000.0f;
		aoutPeriod[0] = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[2] = 2.0f * 1375.0f / 88000000.0f;
	} else {
		pNewState->tickFunc[0][0] = FuncGen_DOUT; pNewState->tickFunc[0][1] = aoutFunc[0]; pNewState->tickFunc[0][2] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = aoutFunc[1];  pNewState->tickFunc[1][1] = aoutFunc[2]; pNewState->tickFunc[1][2] = aoutFunc[3];

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 3;

		doutPeriod = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[0] = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[2] = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[3] = 2.0f * 1375.0f / 88000000.0f;
	}

}

static void initTickFuncs_aout(State *pNewState, int aout, TimerTickFunc *aoutFunc, float *aoutPeriod) {
	//
	if (aout == 0) {
		pNewState->tickFunc[0][0] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = FuncGen_Void;

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 1;
	} else if (aout == 1) {
		pNewState->tickFunc[0][0] = aoutFunc[0]; pNewState->tickFunc[0][1] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = aoutFunc[0]; pNewState->tickFunc[1][1] = FuncGen_Void;

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 2;

		aoutPeriod[0] = 1375.0f / 88000000.0f;
	} else if (aout == 2) {
		pNewState->tickFunc[0][0] = aoutFunc[0]; pNewState->tickFunc[0][1] = aoutFunc[1]; pNewState->tickFunc[0][2] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = aoutFunc[0]; pNewState->tickFunc[1][1] = aoutFunc[1]; pNewState->tickFunc[1][2] = FuncGen_Void;

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 3;

		aoutPeriod[0] = 1375.0f / 88000000.0f;
		aoutPeriod[1] = 1375.0f / 88000000.0f;
	} else if (aout == 3) {
		pNewState->tickFunc[0][0] = aoutFunc[0]; pNewState->tickFunc[0][1] = aoutFunc[1]; pNewState->tickFunc[0][2] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = aoutFunc[0]; pNewState->tickFunc[1][1] = aoutFunc[1]; pNewState->tickFunc[1][2] = aoutFunc[2];

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 3;

		aoutPeriod[0] = 1375.0f / 88000000.0f;
		aoutPeriod[1] = 1375.0f / 88000000.0f;
		aoutPeriod[2] = 2.0f * 1375.0f / 88000000.0f;
	} else {
		pNewState->tickFunc[0][0] = aoutFunc[0]; pNewState->tickFunc[0][1] = aoutFunc[1]; pNewState->tickFunc[0][2] = FuncGen_ADC;
		pNewState->tickFunc[1][0] = aoutFunc[2]; pNewState->tickFunc[1][1] = aoutFunc[3]; pNewState->tickFunc[1][2] = FuncGen_Void;

		pNewState->tickFuncM = 2;
		pNewState->tickFuncN = 3;

		aoutPeriod[0] = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[1] = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[2] = 2.0f * 1375.0f / 88000000.0f;
		aoutPeriod[3] = 2.0f * 1375.0f / 88000000.0f;
	}
}

////////////////////////////////////////////////////////////////////////////////

void FuncGen_Setup() {
	TIM7->PSC = 0;
	TIM7->ARR = 1375 - 1; // 64 KSPS -> 88,000,000 / 64,000 = 1375
}

void FuncGen_SetParams(SetParams &newParams) {
	State *pNewState = g_pCurrentState == g_states ? g_states + 1 : g_states;

	TimerTickFunc aoutFunc[4];

	int aout = 0;
	if (newParams.aoutWaveformParameters[0].waveform != WAVEFORM_NONE) {
		aoutFunc[aout++] = FuncGen_AOUT1;
	}
	if (newParams.aoutWaveformParameters[1].waveform != WAVEFORM_NONE) {
		aoutFunc[aout++] = FuncGen_AOUT2;
	}
	if (newParams.aoutWaveformParameters[2].waveform != WAVEFORM_NONE) {
		aoutFunc[aout++] = FuncGen_AOUT3;
	}
	if (newParams.aoutWaveformParameters[3].waveform != WAVEFORM_NONE) {
		aoutFunc[aout++] = FuncGen_AOUT4;
	}

	bool dout = false;
	for (int i = 0; i < 8; i++) {
		auto &waveformParameters = newParams.doutWaveformParameters[i];
		if (waveformParameters.waveform != WAVEFORM_NONE) {
			dout = true;
			break;
		}
	}

	float doutPeriod = 0.0f;
	float aoutPeriod[4] = {0.0f, 0.0f, 0.0f, 0.0f};

	if (ADC_DLOG_started || DIN_DLOG_started) {
		if (newParams.acAnalysisEnabled) {
			if (dout) {
				initTickFuncs_dlog_ac_dout(pNewState, dout, doutPeriod, aout, aoutFunc, aoutPeriod);
			} else {
				initTickFuncs_dlog_ac_aout(pNewState, aout, aoutFunc, aoutPeriod);
			}
		} else {
			if (dout) {
				initTickFuncs_dlog_dout(pNewState, dout, doutPeriod, aout, aoutFunc, aoutPeriod);
			} else {
				initTickFuncs_dlog_aout(pNewState, aout, aoutFunc, aoutPeriod);
			}
		}
	} else {
		if (newParams.acAnalysisEnabled) {
			if (dout) {
				initTickFuncs_ac_dout(pNewState, dout, doutPeriod, aout, aoutFunc, aoutPeriod);
			} else {
				initTickFuncs_ac_aout(pNewState, aout, aoutFunc, aoutPeriod);
			}
		} else {
			if (dout) {
				initTickFuncs_dout(pNewState, dout, doutPeriod, aout, aoutFunc, aoutPeriod);
			} else {
				initTickFuncs_aout(pNewState, aout, aoutFunc, aoutPeriod);
			}
		}
	}

	int j = 0;
	for (int i = 0; i < 4; i++) {
		auto &waveformParameters = newParams.aoutWaveformParameters[i];

		if (waveformParameters.waveform != WAVEFORM_NONE) {
			DAC_SetRange(i, newParams);

			pNewState->aoutWaveFormFunc[i] = getWaveformFunction(waveformParameters);
			pNewState->aoutDutyCycles[i] = g_dutyCycle;

			pNewState->aoutPhi[i] = 2.0 * M_PI * waveformParameters.phaseShift / 360.0f;
			pNewState->aoutDphi[i] = 2.0 * M_PI * waveformParameters.frequency * aoutPeriod[j++];

			float min;
			float max;
			if (i < 2) {
				DAC_GetValueRange(newParams.aout_dac7760[i].outputRange, min, max);
			} else {
				DACDual_GetValueRange(min, max);
			}

			pNewState->aoutMin[i] = min;
			pNewState->aoutScale[i] = max - min;
		}
	}

	memcpy(pNewState->aoutWaveformParameters, newParams.aoutWaveformParameters, sizeof(newParams.aoutWaveformParameters));

	for (int i = 0; i < 8; i++) {
		auto &waveformParameters = newParams.doutWaveformParameters[i];

		if (waveformParameters.waveform != WAVEFORM_NONE) {
			pNewState->doutWaveFormFunc[i] = getWaveformFunction(waveformParameters);
			pNewState->doutDutyCycles[i] = g_dutyCycle;

			pNewState->doutPhi[i] = 2.0 * M_PI * waveformParameters.phaseShift / 360.0f;
			pNewState->doutDphi[i] = 2.0 * M_PI * waveformParameters.frequency * doutPeriod;
		}
	}

	memcpy(pNewState->doutWaveformParameters, newParams.doutWaveformParameters, sizeof(newParams.doutWaveformParameters));

	pNewState->tickFuncIndex = 0;

	HAL_TIM_Base_Stop_IT(&htim7);

	g_pCurrentState = pNewState;
	
#ifdef DEBUG_TIMING
	m_debugAOUT1 = 0;
	m_debugAOUT2 = 0;
	m_debugAOUT3 = 0;
	m_debugAOUT4 = 0;
	m_debugADC = 0;
	m_debugDOUT = 0;
#endif

	HAL_TIM_Base_Start_IT(&htim7);
}

void FuncGen_onTimerPeriodElapsed() {
	TIMING_BEGIN();

	for (int i = 0; i < g_pCurrentState->tickFuncN; i++) {
		g_pCurrentState->tickFunc[g_pCurrentState->tickFuncIndex][i]();
	}

	g_pCurrentState->tickFuncIndex = (g_pCurrentState->tickFuncIndex + 1) % g_pCurrentState->tickFuncM;

	TIMING_END(m_debugTotal);
}
