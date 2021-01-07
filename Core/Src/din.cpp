#include "main.h"

#include "firmware.h"
#include "utils.h"

GPIO_TypeDef* dinRangePorts[8] = {
	IN_CTRL0_GPIO_Port, IN_CTRL1_GPIO_Port, IN_CTRL2_GPIO_Port, IN_CTRL3_GPIO_Port,
	IN_CTRL4_GPIO_Port, IN_CTRL5_GPIO_Port, IN_CTRL6_GPIO_Port, IN_CTRL7_GPIO_Port
};

uint16_t dinRangePins[8] = {
	IN_CTRL0_Pin, IN_CTRL1_Pin, IN_CTRL2_Pin, IN_CTRL3_Pin,
	IN_CTRL4_Pin,IN_CTRL5_Pin, IN_CTRL6_Pin, IN_CTRL7_Pin
};

GPIO_TypeDef* dinSpeedPorts[2] = {
	SLOW_DIN_0_GPIO_Port, SLOW_DIN_1_GPIO_Port
};

uint16_t dinSpeedPins[2] = {
	SLOW_DIN_0_Pin, SLOW_DIN_1_Pin
};

uint8_t Din_readDataInputs() {
	return
		(READ_PIN(DIN0_GPIO_Port, DIN0_Pin) << 0) |
		(READ_PIN(DIN1_GPIO_Port, DIN1_Pin) << 1) |
		(READ_PIN(DIN2_GPIO_Port, DIN2_Pin) << 2) |
		(READ_PIN(DIN3_GPIO_Port, DIN3_Pin) << 3) |
		(READ_PIN(DIN4_GPIO_Port, DIN4_Pin) << 4) |
		(READ_PIN(DIN5_GPIO_Port, DIN5_Pin) << 5) |
		(READ_PIN(DIN6_GPIO_Port, DIN6_Pin) << 6) |
		(READ_PIN(DIN7_GPIO_Port, DIN7_Pin) << 7);
}

void Din_Setup() {
	for (int i = 0; i < 8; i++) {
		RESET_PIN(dinRangePorts[i], dinRangePins[i]);
	}

	for (int i = 0; i < 2; i++) {
		RESET_PIN(dinSpeedPorts[i], dinSpeedPins[i]);
	}
}

void Din_SetParams(SetParams &newState) {
	for (int i = 0; i < 8; i++) {
		int newRange = newState.dinRanges & (1 << i);
		if (newRange != (currentState.dinRanges & (1 << i))) {
			if (newRange) {
				SET_PIN(dinRangePorts[i], dinRangePins[i]);
			} else {
				RESET_PIN(dinRangePorts[i], dinRangePins[i]);
			}
		}
	}

	for (int i = 0; i < 2; i++) {
		int newSpeed = newState.dinSpeeds & (1 << i);
		if (newSpeed != (currentState.dinSpeeds & (1 << i))) {
			if (newSpeed) {
				SET_PIN(dinSpeedPorts[i], dinSpeedPins[i]);
			} else {
				RESET_PIN(dinSpeedPorts[i], dinSpeedPins[i]);
			}
		}
	}
}
