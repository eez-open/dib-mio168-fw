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
	float newVoltage = newState.aout_dac7563[i].voltage;
	if (newVoltage != currentState.aout_dac7563[i].voltage) {
		if (newState.dacWaveformParameters[2 + i].waveform == WAVEFORM_NONE) {
			DACDual_SetValue(i, DACDual_ValueToDacValue(newVoltage));
		}
	}
}
