#include <math.h>

#include "main.h"

#include "firmware.h"
#include "utils.h"

extern "C" SPI_HandleTypeDef hspi2;
static SPI_HandleTypeDef *hspiDAC = &hspi2;

void DACDual_SpiWrite(uint8_t b0, uint8_t b1, uint8_t b2) {
    uint8_t buf[3] = { b0, b1, b2 };
    RESET_PIN(DAC_CS_DUAL_GPIO_Port, DAC_CS_DUAL_Pin);
    HAL_SPI_Transmit(hspiDAC, buf, 3, 100);
    SET_PIN(DAC_CS_DUAL_GPIO_Port, DAC_CS_DUAL_Pin);
}

void DACDual_Setup() {
	SET_PIN(DAC_CS_DUAL_GPIO_Port, DAC_CS_DUAL_Pin);

    // Enable internal reference
	DACDual_SpiWrite(0b00111000, 0x00, 0x01);

	// Set gain x1
	DACDual_SpiWrite(0b00000010, 0x00, 0x03);
}

void DACDual_GetValueRange(float &min, float &max) {
	min = -10.0f;
	max = 10.0f;
}

float DACDual_ValueToDacValue(float value) {
	float min = -10.0f;
	float max = 10.0f;
	return 65535.0f * (value - min) / (max - min);
}

void DACDual_SetValue_FromFuncGen(int i, float value) {
	currentState.aout_dac7563[i].voltage = value;

	uint16_t dacValue;

	if (value < 0) {
		dacValue = 0;
	} else if (value > 65535.0f) {
		dacValue = 65535;
	} else {
		dacValue = roundf(value);
	}

	RESET_PIN(DAC_CS_DUAL_GPIO_Port, DAC_CS_DUAL_Pin);

	static auto DR = (__IO uint8_t *)&hspi2.Instance->DR;

	while ((hspi2.Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE);
	*DR = i == 0 ? 0b00011000 : 0b00011001;

	while ((hspi2.Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE);
	*DR = dacValue >> 8;

	while ((hspi2.Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE);
	*DR = dacValue & 0xFF;

	while ((hspi2.Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);

	SET_PIN(DAC_CS_DUAL_GPIO_Port, DAC_CS_DUAL_Pin);
}

void DACDual_SetValue(int i, float value) {
	uint16_t dacValue;

	if (value < 0) {
		dacValue = 0;
	} else if (value > 65535.0f) {
		dacValue = 65535;
	} else {
		dacValue = roundf(value);
	}

	if (i == 0) {
		DACDual_SpiWrite(0b00011000, dacValue >> 8, dacValue & 0xFF);
	} else {
		DACDual_SpiWrite(0b00011001, dacValue >> 8, dacValue & 0xFF);
	}
}

void DACDual_SetParams(int i, SetParams &newState) {
	if (newState.aoutWaveformParameters[2 + i].waveform == WAVEFORM_NONE) {
		float newVoltage = newState.aout_dac7563[i].voltage;
		if (newVoltage != currentState.aout_dac7563[i].voltage) {
			DACDual_SetValue(i, DACDual_ValueToDacValue(newVoltage));
		}
	}
}
