#include <math.h>
#include <memory.h>

#include "main.h"

#include "firmware.h"
#include "utils.h"
#include "dlog.h"
#include "funcgen.h"

static const int DEFAULT_SAMPLE_RATE_KSPS = 16;

////////////////////////////////////////////////////////////////////////////////

static uint32_t ADC_DLOG_RECORD_SIZE_24_BIT = 14;
static uint32_t ADC_DLOG_RECORD_SIZE_16_BIT = 10;

extern "C" SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef *hspiADC = &hspi3; // for ADC

float ADC_samples[4];
uint16_t ADC_faultStatus;
uint8_t ADC_autoRangeAFE2;

static const uint64_t ADC_AUTO_RANGE_MIN_SAMPLES = 1;
static const float ADC_AUTO_RANGE_CHANGE_VALUE_PERCENT_THRESHOLD = 0.95;
static uint8_t ADC_AutoRange_currentRange[4];

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

static float *g_samples;
static uint8_t *g_ranges;

volatile static bool ADC_measureStarted = false;
volatile static bool ADC_dataReady = false;

static uint8_t ADC_tx[22] = { 0x12 };

static uint8_t ADC_rx[22];
static uint8_t ADC_rxNext[22];

static uint32_t ADC_PacketSize_24bit = 16;
static uint32_t ADC_PacketSize_16bit = 12;

int ADC_ksps;
float ADC_period;
#define IS_24_BIT (ADC_ksps < 32)

int g_DMA;

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

void acPowerCalcReset() {
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

void acPowerCalc(int32_t *ADC_ch, float period) {
	float volt;
	float curr;

	if (g_afeVersion == 1) {
		if (IS_24_BIT) {
			volt = float(ADC_factor[0] * ADC_ch[0] / (1 << 23));
			curr = float(ADC_factor[2] * ADC_ch[2] / (1 << 23));
		} else {
			volt = float(ADC_factor[0] * ADC_ch[0] / (1 << 15));
			curr = float(ADC_factor[2] * ADC_ch[2] / (1 << 15));
		}

		volt = remap(volt, ADC_calPoints[0].p1CalX, ADC_calPoints[0].p1CalY, ADC_calPoints[0].p2CalX, ADC_calPoints[0].p2CalY);
		curr = remap(curr, ADC_calPoints[2].p1CalX, ADC_calPoints[2].p1CalY, ADC_calPoints[2].p2CalX, ADC_calPoints[2].p2CalY);
	} else if (g_afeVersion == 2) {
		if (IS_24_BIT) {
			volt = float(ADC_factor[0] * ADC_ch[0] / (1 << 23));
			curr = float(ADC_factor[2] * ADC_ch[2] / (1 << 23));
		} else {
			volt = float(ADC_factor[0] * ADC_ch[0] / (1 << 15));
			curr = float(ADC_factor[2] * ADC_ch[2] / (1 << 15));
		}

		volt = remap(volt, ADC_calPoints[0].p1CalX, ADC_calPoints[0].p1CalY, ADC_calPoints[0].p2CalX, ADC_calPoints[0].p2CalY);
		curr = remap(curr, ADC_calPoints[1].p1CalX, ADC_calPoints[1].p1CalY, ADC_calPoints[1].p2CalX, ADC_calPoints[1].p2CalY);
	} else if (g_afeVersion == 3) {
		if (IS_24_BIT) {
			volt = float(ADC_factor[0] * ADC_ch[0] / (1 << 23));
			curr = float(ADC_factor[1] * ADC_ch[1] / (1 << 23));
		} else {
			volt = float(ADC_factor[0] * ADC_ch[0] / (1 << 15));
			curr = float(ADC_factor[1] * ADC_ch[1] / (1 << 15));
		}

		volt = remap(volt, ADC_calPoints[0].p1CalX, ADC_calPoints[0].p1CalY, ADC_calPoints[0].p2CalX, ADC_calPoints[0].p2CalY);
		curr = remap(curr, ADC_calPoints[1].p1CalX, ADC_calPoints[1].p1CalY, ADC_calPoints[1].p2CalX, ADC_calPoints[1].p2CalY);
	} else {
		return;
	}

	int n = (int)floorf((1.0f / period) / currentState.powerLineFrequency);

	g_voltBuffer[g_bufferIndex] = volt;
	g_currBuffer[g_bufferIndex] = curr;

	g_activePowerAcc += volt * curr;

	auto t = g_bufferIndex - n / 4;
	g_reactivePowerAcc += g_voltBuffer[t >= 0 ? t : t + n] * curr;

	g_voltRMSAcc += volt * volt;
	g_currRMSAcc += curr * curr;

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
	float *p1CalX, float *p1CalY, float *p2CalX, float *p2CalY
) {
	uint8_t pga = 0b0001'0000; // x1

	if (g_afeVersion == 1) {
		if (channelIndex == 0 || channelIndex == 1) {
			if (mode == MEASURE_MODE_VOLTAGE) {
				if (range == 3) {
					range = 2;
				}
			} else {
			}
		} else {
			if (mode == MEASURE_MODE_VOLTAGE) {
				if (range == 2) {
					range = 1;
				}
			} else {
				if (range == 3) {
					range = 2;
				}

				if (range == 1) {
					pga = 0b0100'0000; // x4
				} else if (range == 2) {
					pga = 0b0110'0000; // x12
				}
			}
		}
	} else if (g_afeVersion == 3) {
		if (channelIndex == 0) {
			if (range == 2) {
				range = 1;
			}
		} else if (channelIndex == 1) {
			if (range == 2) {
				range = 1;
			}

			if (range == 1) {
				pga = 0b0010'0000; // x2
			}
		} else if (channelIndex == 2) {
			if (range == 3) {
				range = 2;
			}
		} else {
			if (range == 3) {
				range = 2;
			}

			if (mode == MEASURE_MODE_CURRENT) {
				if (range == 1) {
					pga = 0b0100'0000; // x4
				} else if (range == 2) {
					pga = 0b0110'0000; // x12
				}
			}
		}
	}

	ADC_AutoRange_currentRange[channelIndex] = range;

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
	} else if (g_afeVersion == 2) {
		if (channelIndex == 0) {
			ADC_Pin_SetState(0, ISEL_MID_3_GPIO_Port, ISEL_MID_3_Pin, range == 1);
		} else if (channelIndex == 1) {
			ADC_Pin_SetState(1, ISEL_MID_4_GPIO_Port, ISEL_MID_4_Pin, range == 1);
		} else if (channelIndex == 2) {
			ADC_Pin_SetState(2, ISEL_1_GPIO_Port, ISEL_1_Pin, range == 1);
		} else {
			ADC_Pin_SetState(3, ISEL_2_GPIO_Port, ISEL_2_Pin, range == 1);
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

	ADC_calPoints[channelIndex].p1CalX = p1CalX[range];
	ADC_calPoints[channelIndex].p1CalY = p1CalY[range];
	ADC_calPoints[channelIndex].p2CalX = p2CalX[range];
	ADC_calPoints[channelIndex].p2CalY = p2CalY[range];

	g_adcMovingAverage[channelIndex].reset(numSamples);

	acPowerCalcReset();
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
	ADC_period = 1.0f / (ksps * 1000.0f);

	g_adcMovingAverage[0].reset();
	g_adcMovingAverage[1].reset();
	g_adcMovingAverage[2].reset();
	g_adcMovingAverage[3].reset();

	acPowerCalcReset();
}

////////////////////////////////////////////////////////////////////////////////

void ADC_Setup() {
	if (g_afeVersion == 4) {
		return;
	}

	if (g_afeVersion == 2) {
		ADC_PacketSize_24bit = 16 + 6;
		ADC_PacketSize_16bit = 12 + 4;

		GPIO_InitTypeDef GPIO_InitStruct = {0};
	
  		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  		GPIO_InitStruct.Pull = GPIO_NOPULL;

  		GPIO_InitStruct.Pin = GPIO_PIN_5;
  		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  		GPIO_InitStruct.Pin = GPIO_PIN_2;
  		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  	}

	__HAL_SPI_ENABLE(hspiADC);

	ADC_SwReset_Command();
	HAL_Delay(1);
	ADC_WakeUp_Command();
	HAL_Delay(1);
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
			g_afeVersion == 2 ? (
					channelIndex == 0 || channelIndex == 1 ? MEASURE_MODE_CURRENT : MEASURE_MODE_VOLTAGE
			) :
			g_afeVersion == 3 ? (
					channelIndex == 1 ? MEASURE_MODE_CURRENT : MEASURE_MODE_VOLTAGE
			):
			MEASURE_MODE_VOLTAGE;

		uint8_t range =
			g_afeVersion == 1 ? (
				channelIndex == 0 || channelIndex == 1 ? 2 : 1
			) :
			g_afeVersion == 2 ? 1 :
			g_afeVersion == 3 ? (
				channelIndex == 0 ? 1 :
				channelIndex == 1 ? 1 :
				channelIndex == 2 ? 2 :
				1
			) :
			0;

		currentState.ain[channelIndex].mode = mode;
		currentState.ain[channelIndex].range = range;

        float p1CalX[3] = {0, 0, 0};
        float p1CalY[3] = {0, 0, 0};
        float p2CalX[3] = {1, 1, 1};
        float p2CalY[3] = {1, 1, 1};

		ADC_UpdateChannel(channelIndex, mode, range, 1, p1CalX, p1CalY, p2CalX, p2CalY);
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
		if (g_DMA) {
			HAL_Delay(1);
		}
		ADC_Stop_Command();
		HAL_Delay(1);
		ADC_dataReady = false;
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

	bool forceUpdate = newState.acAnalysisEnabled != currentState.acAnalysisEnabled;

	for (uint8_t channelIndex = 0; channelIndex < 4; channelIndex++) {
		if (
			forceUpdate ||
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

void ADC_AutoRange_setRange(int channelIndex, uint8_t range) {
	uint8_t ADC_pga_before = ADC_pga[channelIndex];

	ADC_StopMeasuring();

	uint16_t numSamples = (uint16_t)roundf(currentState.ain[channelIndex].nplc * (1000.0f / currentState.powerLineFrequency));
	if (numSamples < 1) {
		numSamples = 1;
	}
	if (numSamples > ADC_MOVING_AVERAGE_MAX_NUM_SAMPLES) {
		numSamples = ADC_MOVING_AVERAGE_MAX_NUM_SAMPLES;
	}

	ADC_UpdateChannel(
		channelIndex,
		currentState.ain[channelIndex].mode,
		range,
		numSamples,
		currentState.ain[channelIndex].p1CalX,
		currentState.ain[channelIndex].p1CalY,
		currentState.ain[channelIndex].p2CalX,
		currentState.ain[channelIndex].p2CalY
	);

	// recalc offset if PGA changed
	if (ADC_pga_before != ADC_pga[channelIndex]) {
		ADC_OffsetCalc_Command();
	}

	ADC_StartMeasuring();
}

bool ADC_AutoRange_TestLowest(float value, int channelIndex, uint8_t currentRange, float valueLowest) {
	if (ADC_AutoRange_currentRange[channelIndex] != currentRange) {
		return false;
	}

	if (value <= ADC_AUTO_RANGE_CHANGE_VALUE_PERCENT_THRESHOLD * valueLowest) {
		ADC_AutoRange_setRange(channelIndex, currentRange - 2);
		return true;
	}

	return false;
}

bool ADC_AutoRange_TestLower(float value, int channelIndex, uint8_t currentRange, float valueLower) {
	if (ADC_AutoRange_currentRange[channelIndex] != currentRange) {
		return false;
	}

	if (value <= ADC_AUTO_RANGE_CHANGE_VALUE_PERCENT_THRESHOLD * valueLower) {
		ADC_AutoRange_setRange(channelIndex, currentRange - 1);
	}

	return true;
}

bool ADC_AutoRange_TestBetween(float value, int channelIndex, uint8_t currentRange, float valueLower, float valueUpper) {
	if (ADC_AutoRange_currentRange[channelIndex] != currentRange) {
		return false;
	}

	if (value <= ADC_AUTO_RANGE_CHANGE_VALUE_PERCENT_THRESHOLD * valueLower) {
		ADC_AutoRange_setRange(channelIndex, currentRange - 1);
	} else if (value > ADC_AUTO_RANGE_CHANGE_VALUE_PERCENT_THRESHOLD * valueUpper) {
		ADC_AutoRange_setRange(channelIndex, currentRange + 1);
	}	
	
	return true;
}

bool ADC_AutoRange_TestUpper(float value, int channelIndex, uint8_t currentRange, float valueUpper) {
	if (ADC_AutoRange_currentRange[channelIndex] != currentRange) {
		return false;
	}

	if (value > ADC_AUTO_RANGE_CHANGE_VALUE_PERCENT_THRESHOLD * valueUpper) {
		ADC_AutoRange_setRange(channelIndex, currentRange + 1);
	}

	return true;
}

void ADC_AutoRange_AFE1_AIN12_Voltage(int channelIndex, float value) {
	if (ADC_AutoRange_TestLowest(value, channelIndex, 2, 2.4f)) {
		return;
	}

	if (ADC_AutoRange_TestLower(value, channelIndex, 2, 48.0f)) {
		return;
	} 
	
	if (ADC_AutoRange_TestBetween(value, channelIndex, 1, 2.4f, 48.0f)) {
		return;
	}
	
	ADC_AutoRange_TestUpper(value, channelIndex, 0, 2.4f);
}

void ADC_AutoRange_AFE1_AIN34_Voltage(int channelIndex, float value) {
	if (ADC_AutoRange_TestLower(value, channelIndex, 1, 2.4f)) {
		return;
	} 

	ADC_AutoRange_TestUpper(value, channelIndex, 0, 2.4f);
}

void ADC_AutoRange_AFE1_AIN34_Current(int channelIndex, float value) {
	if (ADC_AutoRange_TestLowest(value, channelIndex, 2, 0.048f)) {
		return;
	}

	if (ADC_AutoRange_TestLower(value, channelIndex, 2, 1.2f)) {
		return;
	} 
	
	if (ADC_AutoRange_TestBetween(value, channelIndex, 1, 0.048f, 1.2f)) {
		return;
	}
	
	ADC_AutoRange_TestUpper(value, channelIndex, 0, 0.048f);
}

void ADC_AutoRange_AFE3_AIN1(int channelIndex, float value) {
	if (ADC_AutoRange_TestLower(value, 0, 1, 50.0f)) {
		return;
	} 

	ADC_AutoRange_TestUpper(value, 0, 0, 50.0f);
}

void ADC_AutoRange_AFE3_AIN2(int channelIndex, float value) {
	if (ADC_AutoRange_TestLower(value, 1, 1, 1.0f)) {
		return;
	} 

	ADC_AutoRange_TestUpper(value, 1, 0, 1.0f);
}

void ADC_AutoRange_AFE3_AIN3(int channelIndex, float value) {
	if (ADC_AutoRange_TestLowest(value, 2, 2, 2.4f)) {
		return;
	}

	if (ADC_AutoRange_TestLower(value, 2, 2, 48.0f)) {
		return;
	} 
	
	if (ADC_AutoRange_TestBetween(value, 2, 1, 2.4f, 48.0f)) {
		return;
	}
	
	ADC_AutoRange_TestUpper(value, 2, 0, 2.4f);
}

void ADC_AutoRange_AFE3_AIN4(int channelIndex, float value) {
	if (ADC_AutoRange_TestLowest(value, 3, 2, 0.024f)) {
		return;
	}

	if (ADC_AutoRange_TestLower(value, 3, 2, 1.2f)) {
		return;
	} 
	
	if (ADC_AutoRange_TestBetween(value, 3, 1, 0.024f, 1.2f)) {
		return;
	}
	
	ADC_AutoRange_TestUpper(value, 3, 0, 0.024f);
}

void ADC_doAutoRange(int channelIndex, void (*func)(int, float)) {
	if (g_adcMovingAverage[channelIndex].getNumSamples() >= g_adcMovingAverage[channelIndex].getN()) {
		auto div = 1 << (IS_24_BIT ? 23 : 15);
		auto sample = float(ADC_factor[channelIndex] * (int32_t)g_adcMovingAverage[channelIndex] / div);
		auto range = ADC_AutoRange_currentRange[channelIndex];

		float value = fabs(remap(sample, ADC_calPoints[channelIndex].p1CalX, ADC_calPoints[channelIndex].p1CalY, ADC_calPoints[channelIndex].p2CalX, ADC_calPoints[channelIndex].p2CalY));

		func(channelIndex, value);

		if (range != ADC_AutoRange_currentRange[channelIndex]) {
			if (g_samples) {
				g_samples[channelIndex] = sample;
				g_ranges[channelIndex] = range;
			}
		}
	}
}

void ADC_autoRange() {
	if (g_afeVersion == 1) {
		if (currentState.ain[0].mode == MEASURE_MODE_VOLTAGE && currentState.ain[0].range == 3) {
			ADC_doAutoRange(0, ADC_AutoRange_AFE1_AIN12_Voltage);
		}

		if (currentState.ain[1].mode == MEASURE_MODE_VOLTAGE && currentState.ain[1].range == 3) {
			ADC_doAutoRange(1, ADC_AutoRange_AFE1_AIN12_Voltage);
		}

		if (currentState.ain[2].mode == MEASURE_MODE_VOLTAGE) {
			if (currentState.ain[2].range == 2) {
				ADC_doAutoRange(2, ADC_AutoRange_AFE1_AIN34_Voltage);
			}
		} else {
			if (currentState.ain[2].range == 3) {
				ADC_doAutoRange(2, ADC_AutoRange_AFE1_AIN34_Current);
			}
		}

		if (currentState.ain[3].mode == MEASURE_MODE_VOLTAGE) {
			if (currentState.ain[3].range == 2) {
				ADC_doAutoRange(3, ADC_AutoRange_AFE1_AIN34_Voltage);
			}
		} else {
			if (currentState.ain[3].range == 3) {
				ADC_doAutoRange(3, ADC_AutoRange_AFE1_AIN34_Current);
			}
		}
	} else if (g_afeVersion == 3) {
		if (currentState.ain[0].range == 2) {
			ADC_doAutoRange(0, ADC_AutoRange_AFE3_AIN1);
		}

		if (currentState.ain[1].range == 2) {
			ADC_doAutoRange(1, ADC_AutoRange_AFE3_AIN2);
		}

		if (currentState.ain[2].range == 3) {
			ADC_doAutoRange(2, ADC_AutoRange_AFE3_AIN3);
		}

		if (currentState.ain[3].range == 3) {
			ADC_doAutoRange(3, ADC_AutoRange_AFE3_AIN4);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

bool ADC_isAutoRange(int channelIndex) {
	uint8_t mode = currentState.ain[channelIndex].mode;
	uint8_t range = currentState.ain[channelIndex].range;

	if (g_afeVersion == 1) {
		if (channelIndex == 0 || channelIndex == 1) {
			if (mode == MEASURE_MODE_VOLTAGE) {
				if (range == 3) {
					return true;
				}
			}
		} else {
			if (mode == MEASURE_MODE_VOLTAGE) {
				if (range == 2) {
					return true;
				}
			} else {
				if (range == 3) {
					return true;
				}
			}
		}
	} else if (g_afeVersion == 3) {
		if (channelIndex == 0) {
			if (range == 2) {
				return true;
			}
		} else if (channelIndex == 1) {
			if (range == 2) {
				return true;
			}
		} else if (channelIndex == 2) {
			if (range == 3) {
				return true;
			}
		} else {
			if (range == 3) {
				return true;
			}
		}
	}

	return false;
}

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

	uint8_t ADC_pga_before[4] = {
		ADC_pga[0],
		ADC_pga[1],
		ADC_pga[2],
		ADC_pga[3]
	};

	for (uint8_t channelIndex = 0; channelIndex < 4; channelIndex++) {
		if (ADC_isAutoRange(channelIndex)) {
			uint16_t numSamples = (uint16_t)roundf(currentState.ain[channelIndex].nplc * (1000.0f / currentState.powerLineFrequency));
			if (numSamples < 1) {
				numSamples = 1;
			}
			if (numSamples > ADC_MOVING_AVERAGE_MAX_NUM_SAMPLES) {
				numSamples = ADC_MOVING_AVERAGE_MAX_NUM_SAMPLES;
			}

			ADC_UpdateChannel(
				channelIndex,
				currentState.ain[channelIndex].mode,
				currentState.ain[channelIndex].range,
				numSamples,
				currentState.ain[channelIndex].p1CalX,
				currentState.ain[channelIndex].p1CalY,
				currentState.ain[channelIndex].p2CalX,
				currentState.ain[channelIndex].p2CalY
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

	ADC_DLOG_started = true;

	FuncGen_SetParams(currentState);

	ADC_StartMeasuring();
}

void ADC_DLOG_Stop(Request &request, Response &response) {
	ADC_StopMeasuring();

	ADC_DLOG_started = false;
	ADC_SetSampleRate(DEFAULT_SAMPLE_RATE_KSPS);

	FuncGen_SetParams(currentState);

	ADC_StartMeasuring();
}

void ADC_GetSamples(float *samples, uint8_t *ranges) {
	if (g_afeVersion != 4) {
		auto div = 1 << (IS_24_BIT ? 23 : 15);
		for (int i = 0; i < 4; i++) {
			if (g_adcMovingAverage[i].getNumSamples() >= g_adcMovingAverage[i].getN()) {
				samples[i] = float(ADC_factor[i] * (int32_t)g_adcMovingAverage[i] / div);
				ranges[i] = ADC_AutoRange_currentRange[i];
			}
		}

		g_samples = samples;
		g_ranges = ranges;
	}
}

void ADC_Tick(float period) {
	if (!ADC_dataReady) {
		return;
	}

	uint8_t *rx = ADC_rx + 1;

	ADC_faultStatus = ((rx[0] << 16) | (rx[1] << 8) | rx[2]) >> 4;
	rx += 3;

	if (ADC_DLOG_started) {
		static float adcDlogAcc = 0;

		adcDlogAcc += period;

		if (adcDlogAcc >= ADC_period) {
			adcDlogAcc -= ADC_period;

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
	}

	int32_t ADC_ch[4];

	if (g_afeVersion == 2) {
		ADC_autoRangeAFE2 = READ_PIN(GPIOD, GPIO_PIN_5) | (READ_PIN(GPIOC, GPIO_PIN_2) << 1);
	}

	if (IS_24_BIT) {
		ADC_ch[0] = ((int32_t)((rx[0] << 24) | (rx[ 1] << 16) | (rx[ 2] << 8))) >> 8;
		ADC_ch[1] = ((int32_t)((rx[3] << 24) | (rx[ 4] << 16) | (rx[ 5] << 8))) >> 8;

		if (g_afeVersion == 2) {
			if (currentState.ain[2].range == 1) {
				ADC_ch[2] = ((int32_t)((rx[6] << 24) | (rx[ 7] << 16) | (rx[ 8] << 8))) >> 8;
			} else {
				ADC_ch[2] = ((int32_t)((rx[12] << 24) | (rx[13] << 16) | (rx[14] << 8))) >> 8;
			}

			if (currentState.ain[3].range == 1) {
				ADC_ch[3] = ((int32_t)((rx[9] << 24) | (rx[10] << 16) | (rx[11] << 8))) >> 8;
			} else {
				ADC_ch[3] = ((int32_t)((rx[15] << 24) | (rx[16] << 16) | (rx[17] << 8))) >> 8;
			}
		} else {
			ADC_ch[2] = ((int32_t)((rx[6] << 24) | (rx[ 7] << 16) | (rx[ 8] << 8))) >> 8;
			ADC_ch[3] = ((int32_t)((rx[9] << 24) | (rx[10] << 16) | (rx[11] << 8))) >> 8;
		}
	} else {
		ADC_ch[0] = int16_t((rx[0] << 8) | rx[1]);
		ADC_ch[1] = int16_t((rx[2] << 8) | rx[3]);

		if (g_afeVersion == 2) {
			if (currentState.ain[2].range == 1) {
				ADC_ch[2] = int16_t((rx[4] << 8) | rx[5]);
			} else {
				ADC_ch[2] = int16_t((rx[8] << 8) | rx[9]);
			}

			if (currentState.ain[3].range == 1) {
				ADC_ch[3] = int16_t((rx[6] << 8) | rx[7]);
			} else {
				ADC_ch[3] = int16_t((rx[10] << 8) | rx[11]);
			}
		} else {
			ADC_ch[2] = int16_t((rx[4] << 8) | rx[5]);
			ADC_ch[3] = int16_t((rx[6] << 8) | rx[7]);
		}
	}

	// 1ksps
	static float g_adcMovingAverageACC;
	g_adcMovingAverageACC += period;
	if (g_adcMovingAverageACC >= 0.001f) {
		g_adcMovingAverageACC -= 0.001f;

		g_adcMovingAverage[0](ADC_ch[0]);
		g_adcMovingAverage[1](ADC_ch[1]);
		g_adcMovingAverage[2](ADC_ch[2]);
		g_adcMovingAverage[3](ADC_ch[3]);
	}

	if (currentState.acAnalysisEnabled) {
		acPowerCalc(ADC_ch, period);
	}
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == ADC_DRDY_Pin) {
		if (ADC_measureStarted) {
			if (g_DMA == 1) {
				//HAL_SPI_Abort(hspiADC);
			} else {
				g_DMA = 1;
				RESET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin);

				if (IS_24_BIT) {
					HAL_SPI_TransmitReceive_DMA(hspiADC, ADC_tx, ADC_rxNext, ADC_PacketSize_24bit);
				} else {
					HAL_SPI_TransmitReceive_DMA(hspiADC, ADC_tx, ADC_rxNext, ADC_PacketSize_16bit);
				}
			}
		}
	}
}

void ADC_DMA_TransferCompleted(bool ok) {
	if (ok) {
		memcpy(ADC_rx, ADC_rxNext, sizeof(ADC_rxNext));
		ADC_dataReady = true;
	}

	SET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin);
	g_DMA = 0;
}
