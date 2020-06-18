#include <math.h>

#include "main.h"

////////////////////////////////////////////////////////////////////////////////

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim4;

////////////////////////////////////////////////////////////////////////////////

#define BUFFER_SIZE 16

uint8_t output[BUFFER_SIZE];
uint8_t input[BUFFER_SIZE];

volatile int transferCompleted;

uint8_t outputPinStates = 0;

volatile int adcTransferCompleted = 1;
uint16_t analogInputs[4] = { 0, 0, 0, 0 };

////////////////////////////////////////////////////////////////////////////////

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
}

////////////////////////////////////////////////////////////////////////////////

#define SPI_SLAVE_SYNBYTE         0x53
#define SPI_MASTER_SYNBYTE        0xAC

void slaveSynchro(void) {
    uint32_t idw0 = HAL_GetUIDw0();
    uint32_t idw1 = HAL_GetUIDw1();
    uint32_t idw2 = HAL_GetUIDw2();

    uint8_t txBuffer[15] = {
        SPI_SLAVE_SYNBYTE,
        FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR,
        idw0 >> 24, (idw0 >> 16) & 0xFF, (idw0 >> 8) & 0xFF, idw0 & 0xFF,
        idw1 >> 24, (idw1 >> 16) & 0xFF, (idw1 >> 8) & 0xFF, idw1 & 0xFF,
        idw2 >> 24, (idw2 >> 16) & 0xFF, (idw2 >> 8) & 0xFF, idw2 & 0xFF
    };

    uint8_t rxBuffer[15];

    while (1) {
		rxBuffer[0] = 0;

		transferCompleted = 0;
        if (HAL_SPI_TransmitReceive_DMA(&hspi4, (uint8_t *)&txBuffer, (uint8_t *)&rxBuffer, sizeof(rxBuffer)) != HAL_OK) {
        	continue;
        }
		while (!transferCompleted);

		if (rxBuffer[0] == SPI_MASTER_SYNBYTE) {
			break;
		}
	};
}

////////////////////////////////////////////////////////////////////////////////

uint8_t readDataInputs() {
	return
		(HAL_GPIO_ReadPin(DIN0_GPIO_Port, DIN0_Pin) << 0) |
		(HAL_GPIO_ReadPin(DIN1_GPIO_Port, DIN1_Pin) << 1) |
		(HAL_GPIO_ReadPin(DIN2_GPIO_Port, DIN2_Pin) << 2) |
		(HAL_GPIO_ReadPin(DIN3_GPIO_Port, DIN3_Pin) << 3) |
		(HAL_GPIO_ReadPin(DIN4_GPIO_Port, DIN4_Pin) << 4) |
		(HAL_GPIO_ReadPin(DIN5_GPIO_Port, DIN5_Pin) << 5) |
		(HAL_GPIO_ReadPin(DIN6_GPIO_Port, DIN6_Pin) << 6) |
		(HAL_GPIO_ReadPin(DIN7_GPIO_Port, DIN7_Pin) << 7);
}

////////////////////////////////////////////////////////////////////////////////

static GPIO_TypeDef *doutPort[8] = {
	DOUT0_GPIO_Port,
	DOUT1_GPIO_Port,
	DOUT2_GPIO_Port,
	DOUT3_GPIO_Port,
	DOUT4_GPIO_Port,
	DOUT5_GPIO_Port,
	DOUT6_GPIO_Port,
	DOUT7_GPIO_Port
};

static uint16_t doutPin[8] = {
	DOUT0_Pin,
	DOUT1_Pin,
	DOUT2_Pin,
	DOUT3_Pin,
	DOUT4_Pin,
	DOUT5_Pin,
	DOUT6_Pin,
	DOUT7_Pin
};

void updateOutputPinStates(uint8_t newOutputPinStates) {
    if (outputPinStates == 0 && newOutputPinStates != 0) {
    	HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_SET);
    } else if (outputPinStates != 0 && newOutputPinStates == 0) {
    	HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_RESET);
    }

    for (unsigned i = 0; i < 8; i++) {
    	int oldState = outputPinStates & (1 << i) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    	int newState = newOutputPinStates & (1 << i) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    	if (oldState != newState) {
    		HAL_GPIO_WritePin(doutPort[i], doutPin[i], newState);
    	}
    }

	outputPinStates = newOutputPinStates;
}

////////////////////////////////////////////////////////////////////////////////

uint16_t adcStep = 0;
uint8_t adcIn[4] = { 0x00, 0x00, 0x00, 0x00 };
uint8_t adcOut[4] = { 0x00, 0x00, 0x00, 0x00 };

void ADC_Setup() {
    HAL_GPIO_WritePin(VOLT_SW0_GPIO_Port, VOLT_SW0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(VOLT_SW1_GPIO_Port, VOLT_SW1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(VOLT_SW2_GPIO_Port, VOLT_SW2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(VOLT_SW3_GPIO_Port, VOLT_SW3_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);
}

void ADC_Loop() {
	if (adcTransferCompleted) {
		adcTransferCompleted = 0;
		HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

		analogInputs[adcStep % 4] = (adcOut[2] << 8) | adcOut[3];

		adcStep++;

		adcIn[0] = 0xC0 + (adcStep % 4) * 4;

	    HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive_DMA(&hspi2, adcIn, adcOut, 4);
	}
}

////////////////////////////////////////////////////////////////////////////////
// DAC7563

uint16_t dacStep = 0;

void DACDual_SpiWrite(uint8_t b0, uint8_t b1, uint8_t b2) {
    uint8_t buf[3] = { b0, b1, b2 };
    HAL_GPIO_WritePin(DAC_CS_DUAL_GPIO_Port, DAC_CS_DUAL_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, buf, 3, 100);
    HAL_GPIO_WritePin(DAC_CS_DUAL_GPIO_Port, DAC_CS_DUAL_Pin, GPIO_PIN_SET);
}

void DACDual_Setup() {
    // Enable internal reference
	DACDual_SpiWrite(0b00111000, 0x00, 0x01);
    // Set gain x1
	DACDual_SpiWrite(0b00000010, 0x00, 0x03);
}

void DACDual_Loop() {
    dacStep += 16;

    // DAC7563
    DACDual_SpiWrite(0b00011111, dacStep >> 8, dacStep & 0xFF);
}

////////////////////////////////////////////////////////////////////////////////

void DAC1_SpiWrite(uint8_t b0, uint8_t b1, uint8_t b2) {
    uint8_t buf[3] = { b0, b1, b2 };
    HAL_SPI_Transmit(&hspi1, buf, 3, 100);
    HAL_GPIO_WritePin(DAC_CS_1_GPIO_Port, DAC_CS_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DAC_CS_1_GPIO_Port, DAC_CS_1_Pin, GPIO_PIN_RESET);
  }

void DAC1_Setup() {
    // DAC7760

	// Control register: OUTEN=1, RANGE=001
	// RANGE=000 (0 - 5 V)
	// RANGE=001 (0 - 10 V)
	// RANGE=010 (-5 V - 5 V)
	// RANGE=011 (-10 V - 10 V)
	DAC1_SpiWrite(0x55, 0b00010000, 0b00000000);

	// Config register:
	DAC1_SpiWrite(0x57, 0b00000000, 0b00000000);

	// Write data
	uint16_t dacValue = 65535;
	DAC1_SpiWrite(0x01, dacValue >> 8, dacValue & 0xFF);
}

void DAC1_Loop() {
}

////////////////////////////////////////////////////////////////////////////////

void DAC2_SpiWrite(uint8_t b0, uint8_t b1, uint8_t b2) {
    uint8_t buf[3] = { b0, b1, b2 };
    HAL_SPI_Transmit(&hspi1, buf, 3, 100);
    HAL_GPIO_WritePin(DAC_CS_2_GPIO_Port, DAC_CS_2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DAC_CS_2_GPIO_Port, DAC_CS_2_Pin, GPIO_PIN_RESET);
}

void DAC2_Setup() {
    // DAC7760

	// Control register: OUTEN=1
	// RANGE=000 (0 - 5 V)
	// RANGE=001 (0 - 10 V)
	// RANGE=010 (-5 V - 5 V)
	// RANGE=011 (-10 V - 10 V)
	// RANGE=111 (0 mA to 24 mA)
	DAC2_SpiWrite(0x55, 0b00010000, 0b00000111);

	// Config register:
	DAC2_SpiWrite(0x57, 0b00000000, 0b00000000);

	// Write data
	uint16_t dacValue = 65535;
	DAC2_SpiWrite(0x01, dacValue >> 8, dacValue & 0xFF);
}

void DAC2_Loop() {
}

////////////////////////////////////////////////////////////////////////////////

void PWM_Setup() {
	HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

void PWM_Loop() {
}

////////////////////////////////////////////////////////////////////////////////

void beginTransfer() {
    output[0] = readDataInputs();
    output[1] = outputPinStates;

	uint16_t *outputU16 = (uint16_t *)(output + 2);
	outputU16[0] = analogInputs[0];
	outputU16[1] = analogInputs[1];
	outputU16[2] = analogInputs[2];
	outputU16[3] = analogInputs[3];

    HAL_SPI_TransmitReceive_DMA(&hspi4, output, input, BUFFER_SIZE);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &hspi4) {
		transferCompleted = 1;
	} else {
		adcTransferCompleted = 1;
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &hspi4) {
		transferCompleted = 1;
	} else {
		adcTransferCompleted = 1;
	}
}

////////////////////////////////////////////////////////////////////////////////

void setup() {
	//
	ADC_Setup();
    DACDual_Setup();
    DAC1_Setup();
    DAC2_Setup();
    PWM_Setup();
    //

    slaveSynchro();
}

void loop() {
    beginTransfer();
    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_SET);

    //

	ADC_Loop();
	DACDual_Loop();
	DAC1_Loop();
	DAC2_Loop();
	PWM_Loop();

	//

    while (!transferCompleted) {
    }
    transferCompleted = 0;
    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_RESET);

    //
	updateOutputPinStates(input[1]);
	//
}

