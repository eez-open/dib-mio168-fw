#include <math.h>

#include "main.h"

////////////////////////////////////////////////////////////////////////////////

#define FIRMWARE_VERSION_MAJOR 0x00
#define FIRMWARE_VERSION_MINOR 0x02

////////////////////////////////////////////////////////////////////////////////

extern SPI_HandleTypeDef hspi2;
extern CRC_HandleTypeDef hcrc;

////////////////////////////////////////////////////////////////////////////////

#define BUFFER_SIZE 20

uint8_t output[BUFFER_SIZE];
uint8_t input[BUFFER_SIZE];

uint32_t *output_CRC = (uint32_t *)(output + BUFFER_SIZE - 4);

int transferCompleted;

int loopOperationIndex;

uint8_t outputPinStates = 0;

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
        if (HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t *)&txBuffer, (uint8_t *)&rxBuffer, sizeof(rxBuffer)) != HAL_OK) {
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

void loopOperation_Dummy() {
}

typedef void (*LoopOperation)();

LoopOperation loopOperations[] = {
	loopOperation_Dummy
};

static const int NUM_LOOP_OPERATIONS = sizeof(loopOperations) / sizeof(LoopOperation);

////////////////////////////////////////////////////////////////////////////////

// This MCU doesn't support CRC_INPUTDATA_FORMAT_BYTES (!?).
// This works as CRC_INPUTDATA_FORMAT_BYTES except bufferLength must be multiply of 4,
// i.e. make sure (BUFFER_SIZE - 4) is multiply of 4
uint32_t CRC_Calculate(CRC_HandleTypeDef *hcrc, uint8_t *buffer, uint32_t bufferLength) {
	hcrc->State = HAL_CRC_STATE_BUSY;
	__HAL_CRC_DR_RESET(hcrc);

	for (uint32_t i = 0; i < bufferLength; i += 4) {
		hcrc->Instance->DR = (buffer[i] << 24) | (buffer[i + 1] << 16) | (buffer[i + 2] << 8) | (buffer[i + 3]);
	}
	uint32_t result = hcrc->Instance->DR;

	hcrc->State = HAL_CRC_STATE_READY;

	return result;
}

void beginTransfer() {
    output[0] = readDataInputs();
    output[1] = outputPinStates;

    *output_CRC = CRC_Calculate(&hcrc, output, BUFFER_SIZE - 4);

    HAL_SPI_TransmitReceive_DMA(&hspi2, output, input, BUFFER_SIZE);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	transferCompleted = 1;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    transferCompleted = 1;
}

void setup() {
    slaveSynchro();

    beginTransfer();
    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_SET);
}

void loop() {
    if (transferCompleted) {
        transferCompleted = 0;

        updateOutputPinStates(input[1]);

        beginTransfer();
    }

    LoopOperation loopOperation = loopOperations[loopOperationIndex];
    loopOperationIndex = (loopOperationIndex + 1) % NUM_LOOP_OPERATIONS;
    loopOperation();
}

