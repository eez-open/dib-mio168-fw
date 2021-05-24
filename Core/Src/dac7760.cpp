#include <math.h>

#include "main.h"

#include "firmware.h"
#include "utils.h"

extern "C" SPI_HandleTypeDef hspi2;
static SPI_HandleTypeDef *hspiDAC = &hspi2;

#define DAC7760_CONTROL_REGISTER 0x55
#define DAC7760_CONFIGURATION_REGISTER 0x57
#define DAC7760_DATA_REGISTER 0x01

#define SPI_WAIT_TX(SPIx)    while (!(SPIx->SR & SPI_FLAG_TXE))
#define SPI_WAIT_RX(SPIx)    while (!(SPIx->SR & SPI_FLAG_RXNE))
#define SPI1_DR_8bit(SPIx) (*(__IO uint8_t *)((uint32_t)&(SPIx->DR)))

void DAC_SpiWrite(int i, uint8_t b0, uint8_t b1, uint8_t b2) {
    uint8_t buf[3] = { b0, b1, b2 };

    if (i == 0) {
    	RESET_PIN(DAC_CS_1_GPIO_Port, DAC_CS_1_Pin);
    	HAL_SPI_Transmit(hspiDAC, buf, 3, 100);
    	SET_PIN(DAC_CS_1_GPIO_Port, DAC_CS_1_Pin);
    } else {
        RESET_PIN(DAC_CS_2_GPIO_Port, DAC_CS_2_Pin);
        HAL_SPI_Transmit(hspiDAC, buf, 3, 100);
        SET_PIN(DAC_CS_2_GPIO_Port, DAC_CS_2_Pin);
    }
}

void DAC_Setup(int i) {
	DAC_SpiWrite(i, DAC7760_CONTROL_REGISTER, 0, 0);
	DAC_SpiWrite(i, DAC7760_CONFIGURATION_REGISTER, 0, 0);
	DAC_SpiWrite(i, DAC7760_DATA_REGISTER, 0, 0);
}

//auto DR = (__IO uint8_t *)&hspi2.Instance->DR;
//
//static void SPI_Transmit(uint8_t *pData) {
//	/* Wait until TXE flag is set to send data */
//	while (!(SPI2->SR & SPI_FLAG_TXE))
//	*DR = *pData++;
//
//	/* Wait until TXE flag is set to send data */
//	while (!(SPI2->SR & SPI_FLAG_TXE))
//	*DR = *pData++;
//
//	/* Wait until TXE flag is set to send data */
//	while (!(SPI2->SR & SPI_FLAG_TXE))
//	*DR = *pData++;
//
//	__HAL_SPI_CLEAR_OVRFLAG(&hspi2);
//}

void DAC_SpiWriteFast(int i, uint8_t b0, uint8_t b1, uint8_t b2) {
    uint8_t buf[3] = { b0, b1, b2 };

    if (i == 0) {
    	RESET_PIN(DAC_CS_1_GPIO_Port, DAC_CS_1_Pin);
    	//SPI_Transmit(buf);
    	HAL_SPI_Transmit(hspiDAC, buf, 3, 100);
    	SET_PIN(DAC_CS_1_GPIO_Port, DAC_CS_1_Pin);
    } else {
        RESET_PIN(DAC_CS_2_GPIO_Port, DAC_CS_2_Pin);
        //SPI_Transmit(buf);
        HAL_SPI_Transmit(hspiDAC, buf, 3, 100);
        SET_PIN(DAC_CS_2_GPIO_Port, DAC_CS_2_Pin);
    }
}

void DAC_GetValueRange(uint8_t outputRange, float &min, float &max) {
	if (outputRange == 0) {
		min = 0;
		max = 5.0f;
	} else if (outputRange == 1) {
		min = 0;
		max = 10.0f;
	} else if (outputRange == 2) {
		min = -5.0f;
		max = 5.0f;
	} else if (outputRange == 3) {
		min = -10.0f;
		max = 10.0f;
	} else if (outputRange == 5) {
		min = 4E-3f;
		max = 20E-3f;
	} else if (outputRange == 6) {
		min = 0;
		max = 20E-3f;
	} else if (outputRange == 7) {
		min = 0;
		max = 24E-3f;
	} else {
		min = 0;
		max = 0;
	}
}

float DAC_ValueToDacValue(uint8_t outputRange, float value) {
	float min;
	float max;
	DAC_GetValueRange(outputRange, min, max);
	return 65535.0f * (value - min) / (max - min);
}

void DAC_SetValue(int i, float value) {
	uint16_t dacValue;

	if (value < 0) {
		dacValue = 0;
	} else if (value > 65535.0f) {
		dacValue = 65535;
	} else {
		dacValue = roundf(value);
	}

	DAC_SpiWriteFast(i, DAC7760_DATA_REGISTER, dacValue >> 8, dacValue & 0xFF);
}

void DAC_SetValue(int i, uint8_t outputRange, float value) {
	DAC_SetValue(i, DAC_ValueToDacValue(outputRange, value));
}

void DAC_SetParams(int i, SetParams &newState) {
	uint8_t newOutputEnabled = newState.aout_dac7760[i].outputEnabled;
	uint8_t newOutputRange = newState.aout_dac7760[i].outputRange;
	if (
		newOutputEnabled != currentState.aout_dac7760[i].outputEnabled ||
		newOutputRange != currentState.aout_dac7760[i].outputRange
	) {
		uint16_t controlRegister = 0;

		if (newOutputEnabled) {
			controlRegister |= (1 << 12);
		}

		controlRegister |= newOutputRange;

		DAC_SpiWrite(i, DAC7760_CONTROL_REGISTER, controlRegister >> 8, controlRegister & 0xFF);
	}

	float newOutputValue = newState.aout_dac7760[i].outputValue;
	if (
		newOutputValue != currentState.aout_dac7760[i].outputValue ||
		newOutputRange != currentState.aout_dac7760[i].outputRange
	) {
		if (newState.dacWaveformParameters[i].waveform == WAVEFORM_NONE) {
			DAC_SetValue(i, newOutputRange, newOutputValue);
		}
	}
}
