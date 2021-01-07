#include "main.h"

#include "firmware.h"
#include "utils.h"

static GPIO_TypeDef *doutPort[8] = { DOUT0_GPIO_Port, DOUT1_GPIO_Port, DOUT2_GPIO_Port, DOUT3_GPIO_Port, DOUT4_GPIO_Port, DOUT5_GPIO_Port, DOUT6_GPIO_Port, DOUT7_GPIO_Port };
static uint16_t doutPin[8] = { DOUT0_Pin, DOUT1_Pin, DOUT2_Pin, DOUT3_Pin, DOUT4_Pin, DOUT5_Pin, DOUT6_Pin, DOUT7_Pin };

void updateDoutStates(uint8_t newDoutStates) {
	uint8_t currentDoutStates = currentState.doutStates;

    if (currentDoutStates == 0 && newDoutStates != 0) {
    	SET_PIN(DOUT_EN_GPIO_Port, DOUT_EN_Pin);
    } else if (currentDoutStates != 0 && newDoutStates == 0) {
    	RESET_PIN(DOUT_EN_GPIO_Port, DOUT_EN_Pin);
    }

    for (unsigned i = 0; i < 8; i++) {
    	GPIO_PinState oldState = currentDoutStates & (1 << i) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    	GPIO_PinState newState = newDoutStates & (1 << i) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    	if (oldState != newState) {
    		if (newState) {
    			SET_PIN(doutPort[i], doutPin[i]);
    		} else {
    			RESET_PIN(doutPort[i], doutPin[i]);
    		}
    	}
    }
}

void Dout_Setup() {
	currentState.doutStates = 255;
	updateDoutStates(0);
	currentState.doutStates = 0;
}

void Dout_SetParams(SetParams &newState) {
	updateDoutStates(newState.doutStates);
}
