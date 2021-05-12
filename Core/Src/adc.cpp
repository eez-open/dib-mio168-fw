#include <math.h>
#include <memory.h>

#include "main.h"

#include "firmware.h"
#include "utils.h"
#include "dlog.h"

volatile uint32_t g_debugVarDiff_ADC3 = 0;

static const int DEFAULT_SAMPLE_RATE_KSPS = 16;

////////////////////////////////////////////////////////////////////////////////

static uint32_t ADC_DLOG_RECORD_SIZE_24_BIT = 14;
static uint32_t ADC_DLOG_RECORD_SIZE_16_BIT = 10;

extern "C" SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef *hspiADC = &hspi3; // for ADC

float ADC_samples[4];
uint16_t ADC_faultStatus;
uint8_t ADC_diagStatus;

static const uint64_t ADC_MOVING_AVERAGE_MAX_NUM_SAMPLES = 25 * (1000 / 50);
static MovingAverage<int32_t, int64_t, ADC_MOVING_AVERAGE_MAX_NUM_SAMPLES> g_adcMovingAverage[4];

static uint8_t ADC_pga[4];

static double ADC_factor[4];

static struct {
	float p1CalX;
	float p1CalY;
	float p2CalX;
	float p2CalY;
} ADC_calPoints[4];

volatile static bool ADC_measureStarted = false;

uint8_t ADC_tx[16] = { 0x12 };
uint8_t ADC_rx[16];

int ADC_ksps;
#define IS_24_BIT (ADC_ksps < 32)

////////////////////////////////////////////////////////////////////////////////
//
// Power calc
//

static float g_voltBuffer[64 * 1000 / 50];
static float g_currBuffer[64 * 1000 / 50];

static int g_bufferIndex;

static int g_timeWindow = 10;
static int g_periodCounter = 0;

static double g_activePowerAcc;
static double g_reactivePowerAcc;
static double g_voltRMSAcc;
static double g_currRMSAcc;

float g_activePower;
float g_reactivePower;
float g_voltRMS;
float g_currRMS;

float remap(float x, float x1, float y1, float x2, float y2) {
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

void powerCalcReset() {
	for (unsigned i = 0; i < sizeof(g_voltBuffer) / sizeof(float); i++) {
		g_voltBuffer[i] = 0;
		g_currBuffer[i] = 0;
	}

	g_bufferIndex = 0;
	g_periodCounter = 0;

	g_activePowerAcc = 0;
	g_reactivePowerAcc = 0;
	g_voltRMSAcc = 0;
	g_currRMSAcc = 0;

	g_activePower = 0;
	g_reactivePower = 0;
	g_voltRMS = 0;
	g_currRMS = 0;
}

void powerCalc(int32_t ain1_ADC, int32_t ain2_ADC) {
	float volt;
	float curr;

	if (IS_24_BIT) {
		volt = float(ADC_factor[0] * ain1_ADC / (1 << 23));
		curr = float(ADC_factor[1] * ain2_ADC / (1 << 23));
	} else {
		volt = float(ADC_factor[0] * ain1_ADC / (1 << 15));
		curr = float(ADC_factor[1] * ain2_ADC / (1 << 15));
	}

	volt = remap(volt, ADC_calPoints[0].p1CalX, ADC_calPoints[0].p1CalY, ADC_calPoints[0].p2CalX, ADC_calPoints[0].p2CalY);
	curr = remap(curr, ADC_calPoints[1].p1CalX, ADC_calPoints[1].p1CalY, ADC_calPoints[1].p2CalX, ADC_calPoints[1].p2CalY);

	int n = ADC_ksps * 1000 / 50;

	g_voltBuffer[g_bufferIndex] = volt;
	g_currBuffer[g_bufferIndex] = curr;
	g_bufferIndex++;
	if (g_bufferIndex >= n) {
		g_bufferIndex = 0;

		g_periodCounter++;
		if (g_periodCounter >= g_timeWindow) {
			g_activePower = float(g_activePowerAcc / (g_timeWindow * n));
			g_reactivePower = float(g_reactivePowerAcc / (g_timeWindow * n));
			g_voltRMS = float(sqrt(g_voltRMSAcc / (g_timeWindow * n)));
			g_currRMS = float(sqrt(g_currRMSAcc / (g_timeWindow * n)));

			g_activePowerAcc = 0;
			g_reactivePowerAcc = 0;
			g_voltRMSAcc = 0;
			g_currRMSAcc = 0;

			g_periodCounter = 0;
		}
	}

	g_activePowerAcc += volt * curr;

	auto t = g_bufferIndex - n / 4;
	g_reactivePowerAcc = g_voltBuffer[t >= 0 ? t : t + n] * curr;

	g_voltRMSAcc += volt * volt;
	g_currRMSAcc += curr * curr;
}

////////////////////////////////////////////////////////////////////////////////

#define SPI_WAIT_TX(SPIx)    while (!(SPIx->SR & SPI_FLAG_TXE))
#define SPI_WAIT_RX(SPIx)    while (!(SPIx->SR & SPI_FLAG_RXNE))
#define SPI1_DR_8bit(SPIx) (*(__IO uint8_t *)((uint32_t)&(SPIx->DR)))

inline uint8_t ADC_SPI_TransferReceiveLL(uint8_t data){
	SPI_WAIT_TX(SPI3);
	SPI1_DR_8bit(SPI3) = data;
	SPI_WAIT_RX(SPI3);
	return SPI1_DR_8bit(SPI3);
}

inline uint8_t ADC_SPI_TransferLL(uint8_t data){
	SPI_WAIT_TX(SPI3);
	SPI1_DR_8bit(SPI3) = data;
	SPI_WAIT_RX(SPI3);
	return SPI1_DR_8bit(SPI3);
}

////////////////////////////////////////////////////////////////////////////////

void ADC_SendSimpleCommand(uint8_t cmd) {
	RESET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin);
	delayMicroseconds(10);
	ADC_SPI_TransferLL(cmd);
	delayMicroseconds(10);
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
	uint8_t value = ADC_SPI_TransferReceiveLL(0);
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

const uint32_t RELAY_DELAY = 10;

static int ADC_Pin_CurrentState[16] = {
	// undefined
	-1, -1, -1, -1,
	-1, -1, -1, -1,
	-1, -1, -1, -1,
	-1, -1, -1, -1
};

static int ADC_Relay_CurrentState[4] = {
	// undefined
	-1, -1, -1, -1
};

void ADC_Pin_SetState(int pinIndex, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool pinState) {

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
	int i, GPIO_TypeDef* iSetPort, uint16_t iSetPin, GPIO_TypeDef* iResetPort, uint16_t iResetPin, int iState
) {
	if (ADC_Relay_CurrentState[i] != iState) {
		if (ADC_Relay_CurrentState[i] != iState) {
			if (iState) {
				SET_PIN(iSetPort, iSetPin);
			} else {
				SET_PIN(iResetPort, iResetPin);
			}
		}

		HAL_Delay(RELAY_DELAY);

		if (ADC_Relay_CurrentState[i] != iState) {
			if (iState) {
				RESET_PIN(iSetPort, iSetPin);
			} else {
				RESET_PIN(iResetPort, iResetPin);
			}
			ADC_Relay_CurrentState[i] = iState;
		}
	}
}

void ADC_Relays_SetState(
	int i, GPIO_TypeDef* iSetPort, uint16_t iSetPin, GPIO_TypeDef* iResetPort, uint16_t iResetPin, int iState,
	int j, GPIO_TypeDef* jSetPort, uint16_t jSetPin, GPIO_TypeDef* jResetPort, uint16_t jResetPin, int jState
) {
	if (ADC_Relay_CurrentState[i] != iState || ADC_Relay_CurrentState[j] != jState) {
		if (ADC_Relay_CurrentState[i] != iState) {
			if (iState) {
				SET_PIN(iSetPort, iSetPin);
			} else {
				SET_PIN(iResetPort, iResetPin);
			}
		}

		if (ADC_Relay_CurrentState[j] != jState) {
			if (jState) {
				SET_PIN(jSetPort, jSetPin);
			} else {
				SET_PIN(jResetPort, jResetPin);
			}
		}

		HAL_Delay(RELAY_DELAY);

		if (ADC_Relay_CurrentState[i] != iState) {
			if (iState) {
				RESET_PIN(iSetPort, iSetPin);
			} else {
				RESET_PIN(iResetPort, iResetPin);
			}
			ADC_Relay_CurrentState[i] = iState;
		}

		if (ADC_Relay_CurrentState[j] != jState) {
			if (jState) {
				RESET_PIN(jSetPort, jSetPin);
			} else {
				RESET_PIN(jResetPort, jResetPin);
			}
			ADC_Relay_CurrentState[j] = jState;
		}
	}
}

void ADC_UpdateChannel(
	uint8_t channelIndex, uint8_t mode, uint8_t range, uint16_t numSamples,
	float p1CalX, float p1CalY, float p2CalX, float p2CalY
) {
	uint8_t pga = 0b0001'0000;

	if (g_afeVersion == 1) {
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
	} else if (g_afeVersion == 3) {
		if (channelIndex == 1) {
			if (range == 1) {
				pga = 0b0010'0000; // x2
			}
		} else if (channelIndex == 3) {
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
	}

	ADC_pga[channelIndex] = pga;

	ADC_WriteReg(0x05 + channelIndex, 0b0000'0000 | pga);

	if (g_afeVersion == 1) {
		if (channelIndex == 0) {
			ADC_Pin_SetState(0, USEL1_1_GPIO_Port, USEL1_1_Pin, mode == MEASURE_MODE_VOLTAGE && range == 0);
			ADC_Pin_SetState(1, USEL20_1_GPIO_Port, USEL20_1_Pin, mode == MEASURE_MODE_VOLTAGE && range == 1);
			ADC_Pin_SetState(2, USEL100_1_GPIO_Port, USEL100_1_Pin, mode == MEASURE_MODE_VOLTAGE && range == 2);
			ADC_Pin_SetState(3, ISEL_1_GPIO_Port, ISEL_1_Pin, mode == MEASURE_MODE_CURRENT);
		} else if (channelIndex == 1) {
			ADC_Pin_SetState(4, USEL1_2_GPIO_Port, USEL1_2_Pin, mode == MEASURE_MODE_VOLTAGE && range == 0);
			ADC_Pin_SetState(5, USEL20_2_GPIO_Port, USEL20_2_Pin, mode == MEASURE_MODE_VOLTAGE && range == 1);
			ADC_Pin_SetState(6, USEL100_2_GPIO_Port, USEL100_2_Pin, mode == MEASURE_MODE_VOLTAGE && range == 2);
			ADC_Pin_SetState(7, ISEL_2_GPIO_Port, ISEL_2_Pin, mode == MEASURE_MODE_CURRENT);
		} else if (channelIndex == 2) {
			if (mode == MEASURE_MODE_VOLTAGE) {
				if (range == 0) {
					ADC_Pin_SetState(8, USEL1_3_GPIO_Port, USEL1_3_Pin, true);
					ADC_Pin_SetState(9, USEL20_3_GPIO_Port, USEL20_3_Pin, false);
				} else {
					ADC_Pin_SetState(9, USEL20_3_GPIO_Port, USEL20_3_Pin, true);
					ADC_Pin_SetState(8, USEL1_3_GPIO_Port, USEL1_3_Pin, false);
				}

				ADC_Pin_SetState(10, ISEL_LOW_3_GPIO_Port, ISEL_LOW_3_Pin, false);
				ADC_Pin_SetState(11, ISEL_MID_3_GPIO_Port, ISEL_MID_3_Pin, false);
				
				ADC_Relays_SetState(
					0, ISEL10_S_3_GPIO_Port, ISEL10_S_3_Pin, ISEL10_R_3_GPIO_Port, ISEL10_R_3_Pin, 1,
					1, ISEL_S_3_GPIO_Port, ISEL_S_3_Pin, ISEL_R_3_GPIO_Port, ISEL_R_3_Pin, 0
				);
			} else {
				ADC_Pin_SetState(8, USEL1_3_GPIO_Port, USEL1_3_Pin, false);
				ADC_Pin_SetState(9, USEL20_3_GPIO_Port, USEL20_3_Pin, false);
				if (range == 0) {
					ADC_Pin_SetState(10, ISEL_LOW_3_GPIO_Port, ISEL_LOW_3_Pin, true);
					ADC_Pin_SetState(11, ISEL_MID_3_GPIO_Port, ISEL_MID_3_Pin, false);

					ADC_Relays_SetState(
						0, ISEL10_S_3_GPIO_Port, ISEL10_S_3_Pin, ISEL10_R_3_GPIO_Port, ISEL10_R_3_Pin, 1,
						1, ISEL_S_3_GPIO_Port, ISEL_S_3_Pin, ISEL_R_3_GPIO_Port, ISEL_R_3_Pin, 1
					);
				} else if (range == 1) {
					ADC_Pin_SetState(11, ISEL_MID_3_GPIO_Port, ISEL_MID_3_Pin, true);
					ADC_Pin_SetState(10, ISEL_LOW_3_GPIO_Port, ISEL_LOW_3_Pin, false);

					ADC_Relays_SetState(
						0, ISEL10_S_3_GPIO_Port, ISEL10_S_3_Pin, ISEL10_R_3_GPIO_Port, ISEL10_R_3_Pin, 1,
						1, ISEL_S_3_GPIO_Port, ISEL_S_3_Pin, ISEL_R_3_GPIO_Port, ISEL_R_3_Pin, 1
					);
				} else {
					ADC_Pin_SetState(11, ISEL_MID_3_GPIO_Port, ISEL_MID_3_Pin, true);
					ADC_Pin_SetState(10, ISEL_LOW_3_GPIO_Port, ISEL_LOW_3_Pin, false);

					ADC_Relays_SetState(
						0, ISEL10_S_3_GPIO_Port, ISEL10_S_3_Pin, ISEL10_R_3_GPIO_Port, ISEL10_R_3_Pin, 0,
						1, ISEL_S_3_GPIO_Port, ISEL_S_3_Pin, ISEL_R_3_GPIO_Port, ISEL_R_3_Pin, 0
					);
				}
			}
		} else {
			if (mode == MEASURE_MODE_VOLTAGE) {
				if (range == 0) {
					ADC_Pin_SetState(12, USEL1_4_GPIO_Port, USEL1_4_Pin, true);
					ADC_Pin_SetState(13, USEL20_4_GPIO_Port, USEL20_4_Pin, false);
				} else {
					ADC_Pin_SetState(13, USEL20_4_GPIO_Port, USEL20_4_Pin, true);
					ADC_Pin_SetState(12, USEL1_4_GPIO_Port, USEL1_4_Pin, false);
				}
				ADC_Pin_SetState(14, ISEL_LOW_4_GPIO_Port, ISEL_LOW_4_Pin, false);
				ADC_Pin_SetState(15, ISEL_MID_4_GPIO_Port, ISEL_MID_4_Pin, false);

				ADC_Relays_SetState(
					2, ISEL10_S_4_GPIO_Port, ISEL10_S_4_Pin, ISEL10_R_4_GPIO_Port, ISEL10_R_4_Pin, 1,
					3, ISEL_S_4_GPIO_Port, ISEL_S_4_Pin, ISEL_R_4_GPIO_Port, ISEL_R_4_Pin, 0
				);
			} else {
				ADC_Pin_SetState(12, USEL1_4_GPIO_Port, USEL1_4_Pin, false);
				ADC_Pin_SetState(13, USEL20_4_GPIO_Port, USEL20_4_Pin, false);
				if (range == 0) {
					ADC_Pin_SetState(14, ISEL_LOW_4_GPIO_Port, ISEL_LOW_4_Pin, true);
					ADC_Pin_SetState(15, ISEL_MID_4_GPIO_Port, ISEL_MID_4_Pin, false);

					ADC_Relays_SetState(
						2, ISEL10_S_4_GPIO_Port, ISEL10_S_4_Pin, ISEL10_R_4_GPIO_Port, ISEL10_R_4_Pin, 1,
						3, ISEL_S_4_GPIO_Port, ISEL_S_4_Pin, ISEL_R_4_GPIO_Port, ISEL_R_4_Pin, 1
					);
				} else if (range == 1) {
					ADC_Pin_SetState(15, ISEL_MID_4_GPIO_Port, ISEL_MID_4_Pin, true);
					ADC_Pin_SetState(14, ISEL_LOW_4_GPIO_Port, ISEL_LOW_4_Pin, false);

					ADC_Relays_SetState(
						2, ISEL10_S_4_GPIO_Port, ISEL10_S_4_Pin, ISEL10_R_4_GPIO_Port, ISEL10_R_4_Pin, 1,
						3, ISEL_S_4_GPIO_Port, ISEL_S_4_Pin, ISEL_R_4_GPIO_Port, ISEL_R_4_Pin, 1
					);
				} else {
					ADC_Pin_SetState(15, ISEL_MID_4_GPIO_Port, ISEL_MID_4_Pin, true);
					ADC_Pin_SetState(14, ISEL_LOW_4_GPIO_Port, ISEL_LOW_4_Pin, false);

					ADC_Relays_SetState(
						2, ISEL10_S_4_GPIO_Port, ISEL10_S_4_Pin, ISEL10_R_4_GPIO_Port, ISEL10_R_4_Pin, 0,
						3, ISEL_S_4_GPIO_Port, ISEL_S_4_Pin, ISEL_R_4_GPIO_Port, ISEL_R_4_Pin, 0
					);
				}
			}
		}
	} else if (g_afeVersion == 3) {
		if (channelIndex == 0) {
			ADC_Pin_SetState(0, USEL1_1_GPIO_Port, USEL1_1_Pin, range == 1);
		} else if (channelIndex == 1) {
			ADC_Relay_SetState(1, USEL1_2_GPIO_Port, USEL1_2_Pin, USEL20_2_GPIO_Port, USEL20_2_Pin, range ? 0 : 1);
		} else if (channelIndex == 2) {
			if (mode == MEASURE_MODE_VOLTAGE) {
				if (range == 0) {
					ADC_Pin_SetState(8, USEL1_3_GPIO_Port, USEL1_3_Pin, true);
					ADC_Pin_SetState(9, USEL20_3_GPIO_Port, USEL20_3_Pin, false);
					ADC_Pin_SetState(2, USEL100_1_GPIO_Port, USEL100_1_Pin, false);
				} else if (range == 1) {
					ADC_Pin_SetState(9, USEL20_3_GPIO_Port, USEL20_3_Pin, true);
					ADC_Pin_SetState(8, USEL1_3_GPIO_Port, USEL1_3_Pin, false);
					ADC_Pin_SetState(2, USEL100_1_GPIO_Port, USEL100_1_Pin, false);
				} else {
					ADC_Pin_SetState(2, USEL100_1_GPIO_Port, USEL100_1_Pin, true);
					ADC_Pin_SetState(8, USEL1_3_GPIO_Port, USEL1_3_Pin, false);
					ADC_Pin_SetState(9, USEL20_3_GPIO_Port, USEL20_3_Pin, false);
				}
				ADC_Pin_SetState(3, ISEL_S_3_GPIO_Port, ISEL_S_3_Pin, false);
			} else {
				ADC_Pin_SetState(3, ISEL_S_3_GPIO_Port, ISEL_S_3_Pin, true);
				ADC_Pin_SetState(8, USEL1_3_GPIO_Port, USEL1_3_Pin, false);
				ADC_Pin_SetState(9, USEL20_3_GPIO_Port, USEL20_3_Pin, false);
				ADC_Pin_SetState(2, USEL100_1_GPIO_Port, USEL100_1_Pin, false);
			}
		} else {
			if (mode == MEASURE_MODE_VOLTAGE) {
				if (range == 0) {
					ADC_Pin_SetState(12, USEL1_4_GPIO_Port, USEL1_4_Pin, true);
					ADC_Pin_SetState(13, USEL20_4_GPIO_Port, USEL20_4_Pin, false);
				} else {
					ADC_Pin_SetState(13, USEL20_4_GPIO_Port, USEL20_4_Pin, true);
					ADC_Pin_SetState(12, USEL1_4_GPIO_Port, USEL1_4_Pin, false);
				}
				ADC_Pin_SetState(14, ISEL_LOW_3_GPIO_Port, ISEL_LOW_3_Pin, false);
				ADC_Pin_SetState(15, ISEL_LOW_4_GPIO_Port, ISEL_LOW_4_Pin, false);

				ADC_Relays_SetState(
					2, ISEL10_S_4_GPIO_Port, ISEL10_S_4_Pin, ISEL10_R_4_GPIO_Port, ISEL10_R_4_Pin, 1,
					3, ISEL_S_4_GPIO_Port, ISEL_S_4_Pin, ISEL_R_4_GPIO_Port, ISEL_R_4_Pin, 0
				);
			} else {
				ADC_Pin_SetState(12, USEL1_4_GPIO_Port, USEL1_4_Pin, false);
				ADC_Pin_SetState(13, USEL20_4_GPIO_Port, USEL20_4_Pin, false);
				if (range == 0) {
					ADC_Pin_SetState(14, ISEL_LOW_3_GPIO_Port, ISEL_LOW_3_Pin, true);
					ADC_Pin_SetState(15, ISEL_LOW_4_GPIO_Port, ISEL_LOW_4_Pin, false);

					ADC_Relays_SetState(
						2, ISEL10_S_4_GPIO_Port, ISEL10_S_4_Pin, ISEL10_R_4_GPIO_Port, ISEL10_R_4_Pin, 1,
						3, ISEL_S_4_GPIO_Port, ISEL_S_4_Pin, ISEL_R_4_GPIO_Port, ISEL_R_4_Pin, 1
					);
				} else if (range == 1) {
					ADC_Pin_SetState(15, ISEL_LOW_4_GPIO_Port, ISEL_LOW_4_Pin, true);
					ADC_Pin_SetState(14, ISEL_LOW_3_GPIO_Port, ISEL_LOW_3_Pin, false);

					ADC_Relays_SetState(
						2, ISEL10_S_4_GPIO_Port, ISEL10_S_4_Pin, ISEL10_R_4_GPIO_Port, ISEL10_R_4_Pin, 1,
						3, ISEL_S_4_GPIO_Port, ISEL_S_4_Pin, ISEL_R_4_GPIO_Port, ISEL_R_4_Pin, 1
					);
				} else {
					ADC_Pin_SetState(15, ISEL_LOW_4_GPIO_Port, ISEL_LOW_4_Pin, true);
					ADC_Pin_SetState(14, ISEL_LOW_3_GPIO_Port, ISEL_LOW_3_Pin, false);

					ADC_Relays_SetState(
						2, ISEL10_S_4_GPIO_Port, ISEL10_S_4_Pin, ISEL10_R_4_GPIO_Port, ISEL10_R_4_Pin, 0,
						3, ISEL_S_4_GPIO_Port, ISEL_S_4_Pin, ISEL_R_4_GPIO_Port, ISEL_R_4_Pin, 0
					);
				}
			}
		}
	}

	//
	ADC_factor[channelIndex] = getAinConversionFactor(g_afeVersion, channelIndex, mode, range);

	ADC_calPoints[channelIndex].p1CalX = p1CalX;
	ADC_calPoints[channelIndex].p1CalY = p1CalY;
	ADC_calPoints[channelIndex].p2CalX = p2CalX;
	ADC_calPoints[channelIndex].p2CalY = p2CalY;

	g_adcMovingAverage[channelIndex].reset(numSamples);

	powerCalcReset();
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

	powerCalcReset();
}

////////////////////////////////////////////////////////////////////////////////

void ADC_Setup() {
	if (g_afeVersion == 4) {
		return;
	}

	__HAL_SPI_ENABLE(hspiADC);

	ADC_SwReset_Command();
	ADC_WakeUp_Command();
	ADC_StopReadContinuous_Command();

	////////////////////////////////////////

	ADC_SetSampleRate(DEFAULT_SAMPLE_RATE_KSPS);

	ADC_WriteReg(0x02, 0b1111'0010); // CONFIG2
	ADC_WriteReg(0x03, 0b1100'0000); // CONFIG3

	ADC_WriteReg(0x04, 0b0000'0000); // FAULT

	////////////////////////////////////////

	for (uint8_t channelIndex = 0; channelIndex < 4; channelIndex++) {
		uint8_t mode =
			g_afeVersion == 1 ? MEASURE_MODE_VOLTAGE :
			g_afeVersion == 3 ? (
					channelIndex == 1 ? MEASURE_MODE_CURRENT : MEASURE_MODE_VOLTAGE
			) :
			MEASURE_MODE_VOLTAGE;

		uint8_t range =
			g_afeVersion == 1 ? (
				channelIndex == 0 || channelIndex == 1 ? 2 : 1
			) :
			g_afeVersion == 3 ? (
				channelIndex == 0 ? 1 :
				channelIndex == 1 ? 1 :
				channelIndex == 2 ? 2 :
				1
			) :
			0;

		currentState.ain[channelIndex].mode = mode;
		currentState.ain[channelIndex].range = range;

		ADC_UpdateChannel(channelIndex, mode, range, 1, 0, 0, 1, 1);
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
	if (g_afeVersion == 4) {
		return;
	}

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
				numSamples,
				newState.ain[channelIndex].p1CalX,
				newState.ain[channelIndex].p1CalY,
				newState.ain[channelIndex].p2CalX,
				newState.ain[channelIndex].p2CalY
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
	ADC_SetSampleRate(DEFAULT_SAMPLE_RATE_KSPS);

	ADC_StartMeasuring();
}

////////////////////////////////////////////////////////////////////////////////
//
// ADC measure
//

void ADC_GetSamples(float *samples) {
	if (g_afeVersion != 4) {
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

	static int g_adcMovingAverageCounter;
	if (++g_adcMovingAverageCounter >= ADC_ksps) {
		g_adcMovingAverageCounter = 0;

		g_adcMovingAverage[0](ADC_ch[0]);
		g_adcMovingAverage[1](ADC_ch[1]);
		g_adcMovingAverage[2](ADC_ch[2]);
		g_adcMovingAverage[3](ADC_ch[3]);
	}

	if (g_afeVersion == 3) {
		ADC_diagStatus = READ_PIN(GPIOG, GPIO_PIN_14) | (READ_PIN(GPIOG, GPIO_PIN_15) << 1);
	}

	powerCalc(ADC_ch[0], ADC_ch[1]);
}

////////////////////////////////////////////////////////////////////////////////
//
// ADC measure, no DMA version
//

inline void ADC_Measure() {
	// RESET_PIN(DOUT0_GPIO_Port, DOUT0_Pin);

	RESET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin);

	uint8_t *rx = ADC_rx + 1;

	ADC_SPI_TransferLL(0x12);
	if (IS_24_BIT) {
		*rx++ = ADC_SPI_TransferReceiveLL(0); *rx++ = ADC_SPI_TransferReceiveLL(0); *rx++ = ADC_SPI_TransferReceiveLL(0);

		*rx++ = ADC_SPI_TransferReceiveLL(0); *rx++ = ADC_SPI_TransferReceiveLL(0); *rx++ = ADC_SPI_TransferReceiveLL(0);
		*rx++ = ADC_SPI_TransferReceiveLL(0); *rx++ = ADC_SPI_TransferReceiveLL(0); *rx++ = ADC_SPI_TransferReceiveLL(0);
		*rx++ = ADC_SPI_TransferReceiveLL(0); *rx++ = ADC_SPI_TransferReceiveLL(0); *rx++ = ADC_SPI_TransferReceiveLL(0);
		*rx++ = ADC_SPI_TransferReceiveLL(0); *rx++ = ADC_SPI_TransferReceiveLL(0); *rx++ = ADC_SPI_TransferReceiveLL(0);
	} else {
		*rx++ = ADC_SPI_TransferReceiveLL(0); *rx++ = ADC_SPI_TransferReceiveLL(0); *rx++ = ADC_SPI_TransferReceiveLL(0);

		*rx++ = ADC_SPI_TransferReceiveLL(0); *rx++ = ADC_SPI_TransferReceiveLL(0);
		*rx++ = ADC_SPI_TransferReceiveLL(0); *rx++ = ADC_SPI_TransferReceiveLL(0);
		*rx++ = ADC_SPI_TransferReceiveLL(0); *rx++ = ADC_SPI_TransferReceiveLL(0);
		*rx++ = ADC_SPI_TransferReceiveLL(0); *rx++ = ADC_SPI_TransferReceiveLL(0);
	}

	ADC_AfterMeasure();

	SET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin);

	// SET_PIN(DOUT0_GPIO_Port, DOUT0_Pin);
}

////////////////////////////////////////////////////////////////////////////////
//
// DMA version
//

int g_DMA;

inline void ADC_Measure_Start() {
	g_DMA = 1;
	// RESET_PIN(DOUT0_GPIO_Port, DOUT0_Pin);
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
	// SET_PIN(DOUT0_GPIO_Port, DOUT0_Pin);
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
