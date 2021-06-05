#include "main.h"

#include "firmware.h"
#include "utils.h"

static GPIO_TypeDef *doutPort[8] = { DOUT0_GPIO_Port, DOUT1_GPIO_Port, DOUT2_GPIO_Port, DOUT3_GPIO_Port, DOUT4_GPIO_Port, DOUT5_GPIO_Port, DOUT6_GPIO_Port, DOUT7_GPIO_Port };
static uint16_t doutPin[8] = { DOUT0_Pin, DOUT1_Pin, DOUT2_Pin, DOUT3_Pin, DOUT4_Pin, DOUT5_Pin, DOUT6_Pin, DOUT7_Pin };

static uint8_t doutStates;

void Dout_Setup() {
    for (unsigned i = 0; i < 8; i++) {
		RESET_PIN(doutPort[i], doutPin[i]);
    }
    doutStates = 0;
}

void Dout_SetParams(SetParams &newState) {
	uint8_t newDoutStates = newState.doutStates;

	bool hasWaveform = false;

    for (unsigned i = 0; i < 8; i++) {
    	if (newState.doutWaveformParameters[i].waveform == WAVEFORM_NONE) {
			GPIO_PinState currentState = doutStates & (1 << i) ? GPIO_PIN_SET : GPIO_PIN_RESET;
			GPIO_PinState newState = newDoutStates & (1 << i) ? GPIO_PIN_SET : GPIO_PIN_RESET;
			if (currentState != newState) {
				if (newState) {
					SET_PIN(doutPort[i], doutPin[i]);
					doutStates |= (1 << i);
				} else {
					RESET_PIN(doutPort[i], doutPin[i]);
					doutStates &= ~(1 << i);
				}
			}
    	} else {
    		hasWaveform = true;
    	}
    }

    if (!doutStates && (newDoutStates || hasWaveform)) {
    	SET_PIN(DOUT_EN_GPIO_Port, DOUT_EN_Pin);
    } else if (doutStates && (!newDoutStates && !hasWaveform)) {
    	RESET_PIN(DOUT_EN_GPIO_Port, DOUT_EN_Pin);
    }
}

void Dout_SetPinState(int i, int newState) {
    GPIO_PinState currentState = doutStates & (1 << i) ? GPIO_PIN_SET : GPIO_PIN_RESET;
	if (currentState != newState) {
		if (newState) {
			SET_PIN(doutPort[i], doutPin[i]);
			doutStates |= (1 << i);
		} else {
			RESET_PIN(doutPort[i], doutPin[i]);
			doutStates &= ~(1 << i);
	 	}
	}
}
