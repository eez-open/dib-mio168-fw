#include <math.h>
#include <memory.h>

#include "main.h"

#include "firmware.h"
#include "utils.h"
#include "dlog.h"

volatile uint32_t g_debugVarDiff_ADC3 = 0;

////////////////////////////////////////////////////////////////////////////////

static uint32_t ADC_DLOG_RECORD_SIZE_24_BIT = 14;
static uint32_t ADC_DLOG_RECORD_SIZE_16_BIT = 10;

extern "C" SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef *hspiADC = &hspi3; // for ADC

float ADC_samples[4];
uint16_t ADC_faultStatus;

static const uint64_t ADC_MOVING_AVERAGE_MAX_NUM_SAMPLES = 25 * (1000 / 50);
static MovingAverage<int32_t, int64_t, ADC_MOVING_AVERAGE_MAX_NUM_SAMPLES> g_adcMovingAverage[4];

static uint8_t ADC_pga[4];

static double ADC_factor[4];

volatile static bool ADC_measureStarted = false;

uint8_t ADC_tx[16] = { 0x12 };
uint8_t ADC_rx[16];

int ADC_ksps;
#define IS_24_BIT (ADC_ksps < 32)

////////////////////////////////////////////////////////////////////////////////

#define SPI_WAIT_TX(SPIx)    while (!(SPIx->SR & SPI_FLAG_TXE))
#define SPI_WAIT_RX(SPIx)    while (!(SPIx->SR & SPI_FLAG_RXNE))
#define SPI1_DR_8bit(SPIx) (*(__IO uint8_t *)((uint32_t)&(SPIx->DR)))

inline uint8_t ADC_SPI_TransferLL(uint8_t data){
	SPI_WAIT_TX(SPI3);
	SPI1_DR_8bit(SPI3) = data;
	SPI_WAIT_RX(SPI3);
	return SPI1_DR_8bit(SPI3);
}

////////////////////////////////////////////////////////////////////////////////

void ADC_SendSimpleCommand(uint8_t cmd) {
	RESET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin);
	delayMicroseconds(5);
	ADC_SPI_TransferLL(cmd);
	delayMicroseconds(5);
	SET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin);
}

////////////////////////////////////////////////////////////////////////////////

void ADC_WakeUp_Command() {
	ADC_SendSimpleCommand(0x02);
}

void ADC_SwReset_Command() {
	ADC_SendSimpleCommand(0x06);
	delayMicroseconds(5);
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

uint8_t ADC_ReadReg(uint8_t reg) {
	RESET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin);
	delayMicroseconds(5);
	ADC_SPI_TransferLL((reg & 0b0001'1111) | 0b0010'0000);
	delayMicroseconds(5);
	ADC_SPI_TransferLL(0); // read 1 register
	delayMicroseconds(5);
	uint8_t value = ADC_SPI_TransferLL(0);
	delayMicroseconds(5);
	SET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin);
	return value;
}

void ADC_WriteReg(uint8_t reg, uint8_t val) {
	static const int NUM_RETRIES = 10;
	for (int i = 0; i < NUM_RETRIES; i++) {
		RESET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin);
		delayMicroseconds(5);
		ADC_SPI_TransferLL((reg & 0b0001'1111) | 0b0100'0000);
		delayMicroseconds(5);
		ADC_SPI_TransferLL(0); // write 1 register
		delayMicroseconds(5);
		ADC_SPI_TransferLL(val);
		delayMicroseconds(5);
		SET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin);
		// verify
		if (reg == 0x03) {
			if ((ADC_ReadReg(reg) & 0xFE) == val) {
				break;
			}
		} else {
			if (ADC_ReadReg(reg) == val) {
				break;
			}
		}
	};
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
		if (mode == MEASURE_MODE_CURRENT) {
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
		ADC_Pin_SetState(0, USEL1_1_GPIO_Port, USEL1_1_Pin, mode == MEASURE_MODE_VOLTAGE && range == 0);
		ADC_Pin_SetState(1, USEL10_1_GPIO_Port, USEL10_1_Pin, mode == MEASURE_MODE_VOLTAGE && range == 1);
		ADC_Pin_SetState(2, USEL100_1_GPIO_Port, USEL100_1_Pin, mode == MEASURE_MODE_VOLTAGE && range == 2);
		ADC_Pin_SetState(3, ISEL_1_GPIO_Port, ISEL_1_Pin, mode == MEASURE_MODE_CURRENT);
	} else if (channelIndex == 1) {
		ADC_Pin_SetState(4, USEL1_2_GPIO_Port, USEL1_2_Pin, mode == MEASURE_MODE_VOLTAGE && range == 0);
		ADC_Pin_SetState(5, USEL10_2_GPIO_Port, USEL10_2_Pin, mode == MEASURE_MODE_VOLTAGE && range == 1);
		ADC_Pin_SetState(6, USEL100_2_GPIO_Port, USEL100_2_Pin, mode == MEASURE_MODE_VOLTAGE && range == 2);
		ADC_Pin_SetState(7, ISEL_2_GPIO_Port, ISEL_2_Pin, mode == MEASURE_MODE_CURRENT);
	} else if (channelIndex == 2) {
		// Current 24mA : ISEL_LOW, ISEL10A, ISEL
		// Current 1A   : ISEL_MID, ISEL10A, ISEL
		// Current 10A  : ISEL_MID

		ADC_Pin_SetState(8, USEL1_3_GPIO_Port, USEL1_3_Pin, mode == MEASURE_MODE_VOLTAGE && range == 0);
		ADC_Pin_SetState(9, USEL10_3_GPIO_Port, USEL10_3_Pin, mode == MEASURE_MODE_VOLTAGE && range == 1);
		ADC_Pin_SetState(10, ISEL_LOW_3_GPIO_Port, ISEL_LOW_3_Pin, mode == MEASURE_MODE_CURRENT && range == 0);
		ADC_Pin_SetState(11, ISEL_MID_3_GPIO_Port, ISEL_MID_3_Pin, mode == MEASURE_MODE_CURRENT && (range == 1 || range == 2));

		ADC_Relay_SetState(0, ISEL10_S_3_GPIO_Port, ISEL10_S_3_Pin, ISEL10_R_3_GPIO_Port, ISEL10_R_3_Pin,
			mode == MEASURE_MODE_VOLTAGE || (mode == MEASURE_MODE_CURRENT && (range == 0 || range == 1)));

		ADC_Relay_SetState(1, ISEL_S_3_GPIO_Port, ISEL_S_3_Pin, ISEL_R_3_GPIO_Port, ISEL_R_3_Pin,
			mode == MEASURE_MODE_CURRENT && (range == 0 || range == 1));
	} else {
		// Current 24mA : ISEL_LOW, ISEL10A, ISEL
		// Current 1A   : ISEL_MID, ISEL10A, ISEL
		// Current 10A  : ISEL_MID

		ADC_Pin_SetState(12, USEL1_4_GPIO_Port, USEL1_4_Pin, mode == MEASURE_MODE_VOLTAGE && range == 0);
		ADC_Pin_SetState(13, USEL10_4_GPIO_Port, USEL10_4_Pin, mode == MEASURE_MODE_VOLTAGE && range == 1);
		ADC_Pin_SetState(14, ISEL_LOW_4_GPIO_Port, ISEL_LOW_4_Pin, mode == MEASURE_MODE_CURRENT && range == 0);
		ADC_Pin_SetState(15, ISEL_MID_4_GPIO_Port, ISEL_MID_4_Pin, mode == MEASURE_MODE_CURRENT && (range == 1 || range == 2));

		ADC_Relay_SetState(2, ISEL10_S_4_GPIO_Port, ISEL10_S_4_Pin, ISEL10_R_4_GPIO_Port, ISEL10_R_4_Pin,
			mode == MEASURE_MODE_VOLTAGE || (mode == MEASURE_MODE_CURRENT && (range == 0 || range == 1)));

		ADC_Relay_SetState(3, ISEL_S_4_GPIO_Port, ISEL_S_4_Pin, ISEL_R_4_GPIO_Port, ISEL_R_4_Pin,
			mode == MEASURE_MODE_CURRENT && (range == 0 || range == 1));
	}

	//
	ADC_factor[channelIndex] = getAinConversionFactor(channelIndex, mode, range);

	g_adcMovingAverage[channelIndex].reset(numSamples);
}

////////////////////////////////////////////////////////////////////////////////

void ADC_SetSampleRate(int ksps) {
	if      (ksps == 2 ) ADC_WriteReg(0x01, 0b1111'0101); // CONFIG1  24-bit 2  KSPS
	else if (ksps == 4 ) ADC_WriteReg(0x01, 0b1111'0100); // CONFIG1  24-bit 4  KSPS
	else if (ksps == 8 ) ADC_WriteReg(0x01, 0b1111'0011); // CONFIG1  24-bit 8  KSPS
	else if (ksps == 16) ADC_WriteReg(0x01, 0b1111'0010); // CONFIG1  24-bit 16 KSPS
	else if (ksps == 32) ADC_WriteReg(0x01, 0b1111'0001); // CONFIG1  16-bit 32 KSPS
	else if (ksps == 64) ADC_WriteReg(0x01, 0b1111'0000); // CONFIG1  16-bit 64 KSPS
	else                 ADC_WriteReg(0x01, 0b1111'0110); // CONFIG1  24-bit 1  KSPS

	ADC_ksps = ksps;

	g_adcMovingAverage[0].reset();
	g_adcMovingAverage[1].reset();
	g_adcMovingAverage[2].reset();
	g_adcMovingAverage[3].reset();
}

////////////////////////////////////////////////////////////////////////////////

void ADC_Setup() {
	__HAL_SPI_ENABLE(hspiADC);

	ADC_SwReset_Command();
	ADC_WakeUp_Command();
	ADC_StopReadContinuous_Command();

	////////////////////////////////////////

	ADC_SetSampleRate(1);

	ADC_WriteReg(0x02, 0b1111'0010); // CONFIG2
	ADC_WriteReg(0x03, 0b1100'0000); // CONFIG3

	ADC_WriteReg(0x04, 0b0000'0000); // FAULT

	////////////////////////////////////////

	for (uint8_t channelIndex = 0; channelIndex < 4; channelIndex++) {
		uint8_t mode = MEASURE_MODE_VOLTAGE;
		uint8_t range = channelIndex == 0 || channelIndex == 1 ? 2 : 1;

		currentState.ain[channelIndex].mode = mode;
		currentState.ain[channelIndex].range = range;

		ADC_UpdateChannel(channelIndex, mode, range, 1);
	}

	////////////////////////////////////////

	ADC_OffsetCalc_Command();
}

////////////////////////////////////////////////////////////////////////////////

void ADC_StartMeasuring() {
	// start ADC read if not started
	if (!ADC_measureStarted) {
		HAL_Delay(1);
		ADC_Start_Command();
		ADC_measureStarted = true;
	}
}

void ADC_StopMeasuring() {
	// stop ADC read if started
	if (ADC_measureStarted) {
		ADC_measureStarted = false;
		ADC_Stop_Command();
		HAL_Delay(1);
	}
}

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
			newState.ain[channelIndex].nplc != currentState.ain[channelIndex].nplc ||
			newState.powerLineFrequency != currentState.powerLineFrequency
		) {
			ADC_StopMeasuring();
			
			uint16_t numSamples = (uint16_t)roundf(newState.ain[channelIndex].nplc * (1000.0f / newState.powerLineFrequency));
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

	ADC_StartMeasuring();
}

////////////////////////////////////////////////////////////////////////////////

void ADC_DLOG_Start(Request &request, Response &response) {
	ADC_StopMeasuring();

		 if (request.dlogRecordingStart.period < 1.0f / 32000) ADC_SetSampleRate(64);
	else if (request.dlogRecordingStart.period < 1.0f / 16000) ADC_SetSampleRate(32);
	else if (request.dlogRecordingStart.period < 1.0f /  8000) ADC_SetSampleRate(16);
	else if (request.dlogRecordingStart.period < 1.0f /  4000) ADC_SetSampleRate( 8);
	else if (request.dlogRecordingStart.period < 1.0f /  2000) ADC_SetSampleRate( 4);
	else if (request.dlogRecordingStart.period < 1.0f /  1000) ADC_SetSampleRate( 2);
	else                                                       ADC_SetSampleRate( 1);

	DLOG_bufferIndex = 0;
	DLOG_bufferLastTransferredIndex = 0;
	DLOG_recordIndex = 0;
	DLOG_recordSize = IS_24_BIT ? ADC_DLOG_RECORD_SIZE_24_BIT : ADC_DLOG_RECORD_SIZE_16_BIT;
	DLOG_bufferSize = (BUFFER_SIZE / DLOG_recordSize) * DLOG_recordSize;

	ADC_DLOG_started = true;

	ADC_StartMeasuring();
}

void ADC_DLOG_Stop(Request &request, Response &response) {
	ADC_StopMeasuring();

	ADC_DLOG_started = false;
	ADC_SetSampleRate(1);

	ADC_StartMeasuring();
}

////////////////////////////////////////////////////////////////////////////////
//
// ADC measure
//

void ADC_GetSamples(float *samples) {
	if (IS_24_BIT) {
		samples[0] = float(ADC_factor[0] * (int32_t)g_adcMovingAverage[0] / (1 << 23));
		samples[1] = float(ADC_factor[1] * (int32_t)g_adcMovingAverage[1] / (1 << 23));
		samples[2] = float(ADC_factor[2] * (int32_t)g_adcMovingAverage[2] / (1 << 23));
		samples[3] = float(ADC_factor[3] * (int32_t)g_adcMovingAverage[3] / (1 << 23));
	} else {
		samples[0] = float(ADC_factor[0] * (int32_t)g_adcMovingAverage[0] / (1 << 15));
		samples[1] = float(ADC_factor[1] * (int32_t)g_adcMovingAverage[1] / (1 << 15));
		samples[2] = float(ADC_factor[2] * (int32_t)g_adcMovingAverage[2] / (1 << 15));
		samples[3] = float(ADC_factor[3] * (int32_t)g_adcMovingAverage[3] / (1 << 15));
	}
}

void ADC_AfterMeasure() {
	uint8_t *rx = ADC_rx + 1;

	ADC_faultStatus = ((rx[0] << 16) | (rx[1] << 8) | rx[2]) >> 4;
	rx += 3;

	if (ADC_DLOG_started) {
		auto p = DLOG_buffer + DLOG_bufferIndex % DLOG_bufferSize;

		memcpy(p, rx, DLOG_recordSize - 2);

		p[DLOG_recordSize - 2] =
			(READ_PIN(DIN0_GPIO_Port, DIN0_Pin) << 0) |
			(READ_PIN(DIN1_GPIO_Port, DIN1_Pin) << 1) |
			(READ_PIN(DIN2_GPIO_Port, DIN2_Pin) << 2) |
			(READ_PIN(DIN3_GPIO_Port, DIN3_Pin) << 3) |
			(READ_PIN(DIN4_GPIO_Port, DIN4_Pin) << 4) |
			(READ_PIN(DIN5_GPIO_Port, DIN5_Pin) << 5) |
			(READ_PIN(DIN6_GPIO_Port, DIN6_Pin) << 6) |
			(READ_PIN(DIN7_GPIO_Port, DIN7_Pin) << 7);

		p[DLOG_recordSize - 1] = currentState.doutStates;

		DLOG_bufferIndex += DLOG_recordSize;
	}

	int32_t ADC_ch[4];

	if (IS_24_BIT) {
		ADC_ch[0] = ((int32_t)((rx[0] << 24) | (rx[ 1] << 16) | (rx[ 2] << 8))) >> 8;
		ADC_ch[1] = ((int32_t)((rx[3] << 24) | (rx[ 4] << 16) | (rx[ 5] << 8))) >> 8;
		ADC_ch[2] = ((int32_t)((rx[6] << 24) | (rx[ 7] << 16) | (rx[ 8] << 8))) >> 8;
		ADC_ch[3] = ((int32_t)((rx[9] << 24) | (rx[10] << 16) | (rx[11] << 8))) >> 8;
	} else {
		ADC_ch[0] = int16_t((rx[0] << 8) | rx[1]);
		ADC_ch[1] = int16_t((rx[2] << 8) | rx[3]);
		ADC_ch[2] = int16_t((rx[4] << 8) | rx[5]);
		ADC_ch[3] = int16_t((rx[6] << 8) | rx[7]);
	}

	g_adcMovingAverage[0](ADC_ch[0]);
	g_adcMovingAverage[1](ADC_ch[1]);
	g_adcMovingAverage[2](ADC_ch[2]);
	g_adcMovingAverage[3](ADC_ch[3]);
}

////////////////////////////////////////////////////////////////////////////////
//
// ADC measure, no DMA version
//

inline void ADC_Measure() {
	//RESET_PIN(DOUT0_GPIO_Port, DOUT0_Pin);

	RESET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin);

	uint8_t *rx = ADC_rx + 1;

	ADC_SPI_TransferLL(0x12);
	if (IS_24_BIT) {
		*rx++ = ADC_SPI_TransferLL(0); *rx++ = ADC_SPI_TransferLL(0); *rx++ = ADC_SPI_TransferLL(0);

		*rx++ = ADC_SPI_TransferLL(0); *rx++ = ADC_SPI_TransferLL(0); *rx++ = ADC_SPI_TransferLL(0);
		*rx++ = ADC_SPI_TransferLL(0); *rx++ = ADC_SPI_TransferLL(0); *rx++ = ADC_SPI_TransferLL(0);
		*rx++ = ADC_SPI_TransferLL(0); *rx++ = ADC_SPI_TransferLL(0); *rx++ = ADC_SPI_TransferLL(0);
		*rx++ = ADC_SPI_TransferLL(0); *rx++ = ADC_SPI_TransferLL(0); *rx++ = ADC_SPI_TransferLL(0);
	} else {
		*rx++ = ADC_SPI_TransferLL(0); *rx++ = ADC_SPI_TransferLL(0); *rx++ = ADC_SPI_TransferLL(0);

		*rx++ = ADC_SPI_TransferLL(0); *rx++ = ADC_SPI_TransferLL(0);
		*rx++ = ADC_SPI_TransferLL(0); *rx++ = ADC_SPI_TransferLL(0);
		*rx++ = ADC_SPI_TransferLL(0); *rx++ = ADC_SPI_TransferLL(0);
		*rx++ = ADC_SPI_TransferLL(0); *rx++ = ADC_SPI_TransferLL(0);
	}

	ADC_AfterMeasure();

	SET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin);

	//SET_PIN(DOUT0_GPIO_Port, DOUT0_Pin);
}

////////////////////////////////////////////////////////////////////////////////
//
// DMA version
//

int g_DMA;

inline void ADC_Measure_Start() {
	g_DMA = 1;
	RESET_PIN(DOUT0_GPIO_Port, DOUT0_Pin);
	RESET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin);
	if (IS_24_BIT) {
		HAL_SPI_TransmitReceive_DMA(hspiADC, ADC_tx, ADC_rx, 16);
	} else {
		HAL_SPI_TransmitReceive_DMA(hspiADC, ADC_tx, ADC_rx, 12);
	}
}

void ADC_DMA_TransferCompleted(bool ok) {
//	if (ok) {
		ADC_AfterMeasure();
//	}
	SET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin);
	SET_PIN(DOUT0_GPIO_Port, DOUT0_Pin);
	g_DMA = 0;
}

////////////////////////////////////////////////////////////////////////////////

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == ADC_DRDY_Pin) {
		if (ADC_measureStarted) {
            //ADC_Measure();
			if (g_DMA == 1) {
				g_debugVarDiff_ADC3++;
				//HAL_SPI_Abort(hspiADC);
			} else {
				ADC_Measure_Start(); // DMA version
			}
		}
	}
}
