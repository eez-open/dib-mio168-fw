#include <math.h>

#include "main.h"

#include "firmware.h"
#include "utils.h"

////////////////////////////////////////////////////////////////////////////////

#define DEBUG_VARS 1

#if DEBUG_VARS
volatile uint32_t g_debugVar1;
volatile uint32_t g_debugVar2;
volatile uint32_t g_debugVar3;
volatile float g_debugVar4;
volatile uint16_t g_debugVar5;
#endif

////////////////////////////////////////////////////////////////////////////////

extern "C" SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef *hspiADC = &hspi3; // for ADC

float ADC_samples[4];
uint16_t ADC_faultStatus;

static const uint64_t ADC_MOVING_AVERAGE_MAX_NUM_SAMPLES = 25 * (1000 / 50);
static MovingAverage<float, double, ADC_MOVING_AVERAGE_MAX_NUM_SAMPLES> g_adcMovingAverage[4];

static uint8_t ADC_pga[4];

static double ADC_factor[4];

volatile static bool ADC_started = true;

////////////////////////////////////////////////////////////////////////////////

#define ADC_SPI_SELECT() RESET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin)
#define ADC_SPI_DESELECT() delayMicroseconds(1); SET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin)
//#define ADC_SPI_SELECT() (void)0
//#define ADC_SPI_DESELECT() (void)0

#define SPI_IS_BUSY(SPIx)  (((SPIx)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || ((SPIx)->SR & SPI_SR_BSY))
#define SPI_WAIT(SPIx)     while (SPI_IS_BUSY(SPIx))
#define SPI1_DR_8bit(SPIx) (*(__IO uint8_t *)((uint32_t)&(SPIx->DR)))

uint8_t ADC_SPI_TransferLL(uint8_t data){
	SPI_WAIT(SPI3);
	SPI1_DR_8bit(SPI3) = data;
	SPI_WAIT(SPI3);
	return SPI1_DR_8bit(SPI3);
}

////////////////////////////////////////////////////////////////////////////////

void ADC_SendSimpleCommand(uint8_t cmd) {
	ADC_SPI_SELECT();
	HAL_SPI_Transmit(hspiADC, &cmd, 1, 100);
	ADC_SPI_DESELECT();
}

////////////////////////////////////////////////////////////////////////////////

void ADC_WakeUp_Command() {
	ADC_SendSimpleCommand(0x02);
}

void ADC_SwReset_Command() {
	ADC_SendSimpleCommand(0x06);
	delayMicroseconds(10);
}

void ADC_Start_Command() {
	ADC_SendSimpleCommand(0x08);
}

void ADC_Stop_Command() {
	ADC_SendSimpleCommand(0x0A);
}

void ADC_StopReadContinuous_Command() {
	ADC_SendSimpleCommand(0x11);
}

void ADC_OffsetCalc_Command() {
	ADC_SendSimpleCommand(0x1A);
	HAL_Delay(77); // 77 = 19 * 4 + 1
}

////////////////////////////////////////////////////////////////////////////////

void ADC_WriteReg(uint8_t reg, uint8_t val) {
	uint8_t cmd[2];
	cmd[0] = ( reg & 0b0001'1111 ) | 0b0100'0000;
	cmd[1] = 1; // write 1 register

	ADC_SPI_SELECT();
	HAL_SPI_Transmit(hspiADC, cmd, 2, 100);
	delayMicroseconds(2);
	HAL_SPI_Transmit(hspiADC, &val, 1, 100);
	ADC_SPI_DESELECT();
}

uint8_t ADC_ReadReg(uint8_t reg) {
	uint8_t cmd[2];
	cmd[0] = ( reg & 0b0001'1111 ) | 0b0010'0000;
	cmd[1] = 1; // read 1 register

	uint8_t value;

	ADC_SPI_SELECT();
	HAL_SPI_Transmit(hspiADC, cmd, 2, 100);
	delayMicroseconds(2);
	HAL_SPI_Receive(hspiADC, &value, 1, 100);
	ADC_SPI_DESELECT();

	return value;
}

////////////////////////////////////////////////////////////////////////////////

void ADC_Pin_SetState(int pinIndex, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool pinState) {
	static int ADC_Pin_CurrentState[16] = {
		// undefined
		-1, -1, -1, -1,
		-1, -1, -1, -1,
		-1, -1, -1, -1,
		-1, -1, -1, -1
	};

	int state = pinState ? 1 : 0;

	if (ADC_Pin_CurrentState[pinIndex] != state) {
		if (state) {
			SET_PIN(GPIOx, GPIO_Pin);
		} else {
			RESET_PIN(GPIOx, GPIO_Pin);
		}
		ADC_Pin_CurrentState[pinIndex] = state;
	}
}

void ADC_Relay_SetState(
	int relayIndex,
	GPIO_TypeDef* GPIOx_Set, uint16_t GPIO_Pin_Set,
	GPIO_TypeDef* GPIOx_Reset, uint16_t GPIO_Pin_Reset,
	bool relayState
) {
	static int ADC_Relay_CurrentState[4] = {
		// undefined
		-1, -1, -1, -1
	};

	int state = relayState ? 1 : 0;

	if (ADC_Relay_CurrentState[relayIndex] != state) {
		if (state) {
			SET_PIN(GPIOx_Set, GPIO_Pin_Set);
			HAL_Delay(40);
			RESET_PIN(GPIOx_Set, GPIO_Pin_Set);
		} else {
			SET_PIN(GPIOx_Reset, GPIO_Pin_Reset);
			HAL_Delay(40);
			RESET_PIN(GPIOx_Reset, GPIO_Pin_Reset);
		}

		ADC_Relay_CurrentState[relayIndex] = state;
	}
}

////////////////////////////////////////////////////////////////////////////////

void ADC_UpdateChannel(uint8_t channelIndex, uint8_t mode, uint8_t range, uint16_t numSamples) {
	uint8_t pga = 0b0001'0000;

	if (channelIndex == 2 || channelIndex == 3) {
		if (mode == SOURCE_MODE_CURRENT) {
			if (range == 0) {
				pga = 0b0010'0000; // x2
			} else if (range == 1) {
				pga = 0b0100'0000; // x4
			} else {
				pga = 0b0110'0000; // x12
			}
		}
	}

	ADC_pga[channelIndex] = pga;

	ADC_WriteReg(0x05 + channelIndex, 0b0000'0000 | pga);

	if (channelIndex == 0) {
		ADC_Pin_SetState(0, USEL1_1_GPIO_Port, USEL1_1_Pin, mode == SOURCE_MODE_VOLTAGE && range == 0);
		ADC_Pin_SetState(1, USEL10_1_GPIO_Port, USEL10_1_Pin, mode == SOURCE_MODE_VOLTAGE && range == 1);
		ADC_Pin_SetState(2, USEL100_1_GPIO_Port, USEL100_1_Pin, mode == SOURCE_MODE_VOLTAGE && range == 2);
		ADC_Pin_SetState(3, ISEL_1_GPIO_Port, ISEL_1_Pin, mode == SOURCE_MODE_CURRENT);
	} else if (channelIndex == 1) {
		ADC_Pin_SetState(4, USEL1_2_GPIO_Port, USEL1_2_Pin, mode == SOURCE_MODE_VOLTAGE && range == 0);
		ADC_Pin_SetState(5, USEL10_2_GPIO_Port, USEL10_2_Pin, mode == SOURCE_MODE_VOLTAGE && range == 1);
		ADC_Pin_SetState(6, USEL100_2_GPIO_Port, USEL100_2_Pin, mode == SOURCE_MODE_VOLTAGE && range == 2);
		ADC_Pin_SetState(7, ISEL_2_GPIO_Port, ISEL_2_Pin, mode == SOURCE_MODE_CURRENT);
	} else if (channelIndex == 2) {
		// Current 24mA : ISEL_LOW, ISEL10A, ISEL
		// Current 1A   : ISEL_MID, ISEL10A, ISEL
		// Current 10A  : ISEL_MID

		ADC_Pin_SetState(8, USEL1_3_GPIO_Port, USEL1_3_Pin, mode == SOURCE_MODE_VOLTAGE && range == 0);
		ADC_Pin_SetState(9, USEL10_3_GPIO_Port, USEL10_3_Pin, mode == SOURCE_MODE_VOLTAGE && range == 1);
		ADC_Pin_SetState(10, ISEL_LOW_3_GPIO_Port, ISEL_LOW_3_Pin, mode == SOURCE_MODE_CURRENT && range == 0);
		ADC_Pin_SetState(11, ISEL_MID_3_GPIO_Port, ISEL_MID_3_Pin, mode == SOURCE_MODE_CURRENT && (range == 1 || range == 2));

		ADC_Relay_SetState(0, ISEL10_S_3_GPIO_Port, ISEL10_S_3_Pin, ISEL10_R_3_GPIO_Port, ISEL10_R_3_Pin,
			mode == SOURCE_MODE_VOLTAGE || (mode == SOURCE_MODE_CURRENT && (range == 0 || range == 1)));

		ADC_Relay_SetState(1, ISEL_S_3_GPIO_Port, ISEL_S_3_Pin, ISEL_R_3_GPIO_Port, ISEL_R_3_Pin,
			mode == SOURCE_MODE_CURRENT && (range == 0 || range == 1));
	} else {
		// Current 24mA : ISEL_LOW, ISEL10A, ISEL
		// Current 1A   : ISEL_MID, ISEL10A, ISEL
		// Current 10A  : ISEL_MID

		ADC_Pin_SetState(12, USEL1_4_GPIO_Port, USEL1_4_Pin, mode == SOURCE_MODE_VOLTAGE && range == 0);
		ADC_Pin_SetState(13, USEL10_4_GPIO_Port, USEL10_4_Pin, mode == SOURCE_MODE_VOLTAGE && range == 1);
		ADC_Pin_SetState(14, ISEL_LOW_4_GPIO_Port, ISEL_LOW_4_Pin, mode == SOURCE_MODE_CURRENT && range == 0);
		ADC_Pin_SetState(15, ISEL_MID_4_GPIO_Port, ISEL_MID_4_Pin, mode == SOURCE_MODE_CURRENT && (range == 1 || range == 2));

		ADC_Relay_SetState(2, ISEL10_S_4_GPIO_Port, ISEL10_S_4_Pin, ISEL10_R_4_GPIO_Port, ISEL10_R_4_Pin,
			mode == SOURCE_MODE_VOLTAGE || (mode == SOURCE_MODE_CURRENT && (range == 0 || range == 1)));

		ADC_Relay_SetState(3, ISEL_S_4_GPIO_Port, ISEL_S_4_Pin, ISEL_R_4_GPIO_Port, ISEL_R_4_Pin,
			mode == SOURCE_MODE_CURRENT && (range == 0 || range == 1));
	}

	//
	double f;

	if (channelIndex < 2) {
		if (mode == SOURCE_MODE_VOLTAGE) {
			if (range == 0) {
				f = 2.4; // +/- 2.4 V
			} else if (range == 1) {
				f = 48.0; // +/- 48 V
			} else {
				f = 240.0; // +/- 240 V
			}
		} else if (mode == SOURCE_MODE_CURRENT) {
			f = 0.048; // +/- 48 mV ( = 2.4 V / 50 Ohm)
		} else {
			f = 0;
		}
	} else {
		if (mode == SOURCE_MODE_VOLTAGE) {
			if (range == 0) {
				f = 2.4; // +/- 2.4 V
			} else {
				f = 12.0; // +/- 12 V
			}
		} else if (mode == SOURCE_MODE_CURRENT) {
			if (range == 0) {
				f = 0.024; // +/- 24 mA (rsense is 50 ohm, PGA is 2)
			} else if (range == 1) {
				f = 1.2; // +/- 1.2 A (rsense 0.5 ohm, PGA is 4)
			} else {
				f = 20.0; // +/- 10 A (rsense is 0.01 ohm, PGA is 12 => 2.4 / 0.01 / 16 = 20 A)
			}
		} else {
			f = 0;
		}
	}

	ADC_factor[channelIndex] = f;

	g_adcMovingAverage[channelIndex].reset(numSamples);

#if DEBUG_VARS
	g_debugVar1 = 0;
#endif

}

////////////////////////////////////////////////////////////////////////////////

void ADC_Setup() {
	ADC_SwReset_Command();
	ADC_WakeUp_Command();
	ADC_StopReadContinuous_Command();

	////////////////////////////////////////

	ADC_WriteReg(0x01, 0b1111'0110); // CONFIG1  24-bit 16 KSPS
	ADC_WriteReg(0x02, 0b1111'0101); // CONFIG2
	ADC_WriteReg(0x03, 0b1100'0000); // CONFIG3

	ADC_WriteReg(0x04, 0b0000'0000); // FAULT

	////////////////////////////////////////

	for (uint8_t channelIndex = 0; channelIndex < 4; channelIndex++) {
		ADC_UpdateChannel(
			channelIndex,
			currentState.ain[channelIndex].mode,
			currentState.ain[channelIndex].range,
			1
		);
	}

	ADC_OffsetCalc_Command();
}

////////////////////////////////////////////////////////////////////////////////

void ADC_SetParams(SetParams &newState) {
	uint8_t ADC_pga_before[4] = {
		ADC_pga[0],
		ADC_pga[1],
		ADC_pga[2],
		ADC_pga[3]
	};

	for (uint8_t channelIndex = 0; channelIndex < 4; channelIndex++) {
		if (
			newState.ain[channelIndex].mode != currentState.ain[channelIndex].mode ||
			newState.ain[channelIndex].range != currentState.ain[channelIndex].range ||
			newState.ain[channelIndex].numPowerLineCycles != currentState.ain[channelIndex].numPowerLineCycles ||
			newState.powerLineFrequency != currentState.powerLineFrequency
		) {
			// stop ADC read if started
			if (ADC_started) {
				ADC_started = false;
				ADC_Stop_Command();
				HAL_Delay(1);
			}
			
			uint16_t numSamples = (uint16_t)ceilf(newState.ain[channelIndex].numPowerLineCycles * (1000.0f / newState.powerLineFrequency));
			if (numSamples < 1) {
				numSamples = 1;
			}
			if (numSamples > ADC_MOVING_AVERAGE_MAX_NUM_SAMPLES) {
				numSamples = ADC_MOVING_AVERAGE_MAX_NUM_SAMPLES;
			}

			ADC_UpdateChannel(
				channelIndex,
				newState.ain[channelIndex].mode,
				newState.ain[channelIndex].range,
				numSamples
			);
		}
	}

	// recalc offset if PGA changed on any channel
	if (
		ADC_pga_before[0] != ADC_pga[0] ||
		ADC_pga_before[1] != ADC_pga[1] ||
		ADC_pga_before[2] != ADC_pga[2] ||
		ADC_pga_before[3] != ADC_pga[3]
	) {
		ADC_OffsetCalc_Command();
	}

	// start ADC read if not started
	if (!ADC_started) {
		HAL_Delay(1);
		ADC_Start_Command();
		ADC_started = true;
	}
}

////////////////////////////////////////////////////////////////////////////////
//
// ADC measure, no DMA version
//

inline void ADC_Measure() {
	ADC_SPI_SELECT();

	ADC_SPI_TransferLL(0x12);

	uint8_t rx[15];
	for (int i = 0; i < 15; i++) {
		rx[i] = ADC_SPI_TransferLL(0);
	}

	ADC_SPI_DESELECT();

	ADC_faultStatus = ((rx[0] << 16) | (rx[1] << 8) | rx[2]) >> 4;

	int32_t ch[4];

	ch[0] = ((int32_t)((rx[ 3] << 24) + (rx[ 4] << 16) +(rx[ 5] << 8))) >> 8;
	ch[1] = ((int32_t)((rx[ 6] << 24) + (rx[ 7] << 16) +(rx[ 8] << 8))) >> 8;
	ch[2] = ((int32_t)((rx[ 9] << 24) + (rx[10] << 16) +(rx[11] << 8))) >> 8;
	ch[3] = ((int32_t)((rx[12] << 24) + (rx[13] << 16) +(rx[14] << 8))) >> 8;

	for (uint8_t channelIndex = 0; channelIndex < 4; channelIndex++) {
		double f = ADC_factor[channelIndex];
		float value = (float)(f * ch[channelIndex] / (1 << 23));
		g_adcMovingAverage[channelIndex](value);
		ADC_samples[channelIndex] = g_adcMovingAverage[channelIndex];
	}

#if DEBUG_VARS
	g_debugVar5 = g_adcMovingAverage[0].getN();
	if (++g_debugVar1 == g_adcMovingAverage[0].getN()) {
		g_debugVar1 = 0;
		uint32_t tickCount = HAL_GetTick();
		g_debugVar2 = tickCount - g_debugVar3;
		g_debugVar3 = tickCount;
	}
	g_debugVar4 = ADC_samples[0] * 10000;
#endif
}

////////////////////////////////////////////////////////////////////////////////
//
// DMA version
//

uint8_t ADX_rx[16];

inline void ADC_Measure_Start() {
	uint8_t ADC_tx[16] = {
		0x12, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00
	};

	RESET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin);
	HAL_SPI_TransmitReceive_DMA(hspiADC, ADC_tx, ADX_rx, 16);
}

void ADC_Measure_Finish(bool ok) {
	SET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin);

	if (ok && ADC_started) {
		uint8_t *rx = ADX_rx + 1;

		ADC_faultStatus = ((rx[0] << 16) | (rx[1] << 8) | rx[2]) >> 4;

		int32_t ch[4];

		ch[0] = ((int32_t)((rx[ 3] << 24) + (rx[ 4] << 16) +(rx[ 5] << 8))) >> 8;
		ch[1] = ((int32_t)((rx[ 6] << 24) + (rx[ 7] << 16) +(rx[ 8] << 8))) >> 8;
		ch[2] = ((int32_t)((rx[ 9] << 24) + (rx[10] << 16) +(rx[11] << 8))) >> 8;
		ch[3] = ((int32_t)((rx[12] << 24) + (rx[13] << 16) +(rx[14] << 8))) >> 8;

		for (uint8_t channelIndex = 0; channelIndex < 4; channelIndex++) {
			double f = ADC_factor[channelIndex];
			float value = (float)(f * ch[channelIndex] / (1 << 23));
			g_adcMovingAverage[channelIndex](value);
			ADC_samples[channelIndex] = g_adcMovingAverage[channelIndex];
		}
	}

#if DEBUG_VARS
	if (++g_debugVar1 == g_adcMovingAverage[0].getN()) {
		g_debugVar1 = 0;
		uint32_t tickCount = HAL_GetTick();
		g_debugVar2 = tickCount - g_debugVar3;
		g_debugVar3 = tickCount;
	}
	g_debugVar4 = ADC_samples[0];
#endif
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == ADC_DRDY_Pin) {
		if (ADC_started) {
			ADC_Measure();
			// DMA version
			// ADC_Measure_Start();
		}
	}
}
