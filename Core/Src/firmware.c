#include <math.h>
#include <memory.h>

#include "main.h"
#include <bsp_driver_sd.h>
#include <ff_gen_drv.h>
#include <sd_diskio.h>

////////////////////////////////////////////////////////////////////////////////

extern SPI_HandleTypeDef hspi1; // for DAC7760 and DAC7563
extern SPI_HandleTypeDef hspi2; // for ADC8674
extern SPI_HandleTypeDef hspi4; // for MASTER-SLAVE communication
extern TIM_HandleTypeDef htim4; // for PWM outputs

extern TIM_HandleTypeDef htim6; // for DIN's data logging

////////////////////////////////////////////////////////////////////////////////

extern void TIM4_Init(void);

////////////////////////////////////////////////////////////////////////////////

enum SourceMode {
	SOURCE_MODE_CURRENT,
	SOURCE_MODE_VOLTAGE,
	SOURCE_MODE_OPEN
};

typedef struct {
    uint8_t operation; // DISK_DRIVER_OPERATION_NONE

    uint8_t dinRanges;
    uint8_t dinSpeeds;

    uint8_t doutStates;

    struct {
        uint8_t mode; // enum SourceMode
        uint8_t range;
        uint8_t tempSensorBias;
    } ain[4];

    struct {
        uint8_t outputEnabled;
        uint8_t outputRange;
        float outputValue;
    } aout_dac7760[2];

    struct {
        float voltage;
    } aout_dac7563[2];

    struct {
        float freq;
        float duty;
    } pwm[2];

    uint16_t dinReadPeriod;
} FromMasterToSlaveParamsChange;

typedef struct {
    uint8_t operation; // enum DiskDriverOperation
    uint32_t sector;
    uint8_t cmd;
    uint8_t buffer[512];
} FromMasterToSlaveDiskDriveOperation;

enum DiskDriverOperation {
    DISK_DRIVER_OPERATION_NONE,
    DISK_DRIVER_OPERATION_INITIALIZE,
    DISK_DRIVER_OPERATION_STATUS,
    DISK_DRIVER_OPERATION_READ,
    DISK_DRIVER_OPERATION_WRITE,
    DISK_DRIVER_OPERATION_IOCTL
};

#define FLAG_SD_CARD_PRESENT (1 << 0)
#define FLAG_DIN_READ_OVERFLOW (1 << 1)
#define FLAG_DISK_OPERATION_RESPONSE (1 << 2)

#define MAX_DIN_VALUES 100

#define DISK_DRIVER_IOCTL_BUFFER_MAX_SIZE 4

typedef struct {
    uint8_t dinStates;
    uint16_t ainValues[4];
    uint8_t flags; // see FLAG_...
	uint16_t numDinValues;
	uint32_t diskOperationResult;
	uint8_t buffer[512];
} FromSlaveToMaster;

FromMasterToSlaveParamsChange currentState;

////////////////////////////////////////////////////////////////////////////////

uint8_t output[1024];
uint8_t input[1024];

volatile int transferCompleted;

////////////////////////////////////////////////////////////////////////////////

void resetState() {
	currentState.dinRanges = 0;
	currentState.dinSpeeds = 0;

	currentState.doutStates = 0;

	for (int i = 0; i < 4; i++) {
		currentState.ain[i].mode = 1;
		currentState.ain[i].range = 0;
		currentState.ain[i].tempSensorBias = 0;
	}

	for (int i = 0; i < 2; i++) {
		currentState.aout_dac7760[i].outputEnabled = 0;
		currentState.aout_dac7760[i].outputRange = 0;
		currentState.aout_dac7760[i].outputValue = 0;
	}

	for (int i = 0; i < 2; i++) {
		currentState.aout_dac7563[i].voltage = 0;
	}

	for (int i = 0; i < 2; i++) {
		currentState.pwm[i].freq = 0;
		currentState.pwm[i].duty = 0;
	}
}

////////////////////////////////////////////////////////////////////////////////

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
}

////////////////////////////////////////////////////////////////////////////////

uint32_t g_dinReadTimerCounter = 0;
#define DIN_READ_BUFFER_SIZE 1000
uint8_t g_dinReadBuffer[DIN_READ_BUFFER_SIZE];
volatile uint32_t g_dinReadBufferIndex;
uint32_t g_dinReadBufferTransferredIndex;

uint8_t readDataInputs();

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim6) {
		g_dinReadBuffer[g_dinReadBufferIndex++ % DIN_READ_BUFFER_SIZE] = readDataInputs();
	}
}

void DinRead_Loop(FromMasterToSlaveParamsChange *newState) {
	if (newState->dinReadPeriod != currentState.dinReadPeriod) {
		HAL_TIM_Base_Stop_IT(&htim6);

		if (newState->dinReadPeriod > 0) {
			TIM6->ARR = newState->dinReadPeriod;

			g_dinReadTimerCounter = 0;
			g_dinReadBufferIndex = 0;
			g_dinReadBufferTransferredIndex = 0;

			HAL_TIM_Base_Start_IT(&htim6);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
// Digital Inputs

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

uint8_t dinStates;

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

void Din_Setup() {
	for (int i = 0; i < 8; i++) {
		HAL_GPIO_WritePin(dinRangePorts[i], dinRangePins[i], GPIO_PIN_RESET);
	}

	for (int i = 0; i < 2; i++) {
		HAL_GPIO_WritePin(dinSpeedPorts[i], dinSpeedPins[i], GPIO_PIN_RESET);
	}

	dinStates = readDataInputs();
}

void Din_Loop(FromMasterToSlaveParamsChange *newState) {
	for (int i = 0; i < 8; i++) {
		int newRange = newState->dinRanges & (1 << i);
		if (newRange != (currentState.dinRanges & (1 << i))) {
			HAL_GPIO_WritePin(dinRangePorts[i], dinRangePins[i], newRange ? GPIO_PIN_SET : GPIO_PIN_RESET);
		}
	}

	for (int i = 0; i < 2; i++) {
		int newSpeed = newState->dinSpeeds & (1 << i);
		if (newSpeed != (currentState.dinSpeeds & (1 << i))) {
			HAL_GPIO_WritePin(dinSpeedPorts[i], dinSpeedPins[i], newSpeed ? GPIO_PIN_SET : GPIO_PIN_RESET);
		}
	}

	dinStates = readDataInputs();
}

////////////////////////////////////////////////////////////////////////////////
// Digital Outputs

static GPIO_TypeDef *doutPort[8] = { DOUT0_GPIO_Port, DOUT1_GPIO_Port, DOUT2_GPIO_Port, DOUT3_GPIO_Port, DOUT4_GPIO_Port, DOUT5_GPIO_Port, DOUT6_GPIO_Port, DOUT7_GPIO_Port };
static uint16_t doutPin[8] = { DOUT0_Pin, DOUT1_Pin, DOUT2_Pin, DOUT3_Pin, DOUT4_Pin, DOUT5_Pin, DOUT6_Pin, DOUT7_Pin };

void updateDoutStates(uint8_t newDoutStates) {
	uint8_t currentDoutStates = currentState.doutStates;

    if (currentDoutStates == 0 && newDoutStates != 0) {
    	HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_SET);
    } else if (currentDoutStates != 0 && newDoutStates == 0) {
    	HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_RESET);
    }

    for (unsigned i = 0; i < 8; i++) {
    	int oldState = currentDoutStates & (1 << i) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    	int newState = newDoutStates & (1 << i) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    	if (oldState != newState) {
    		HAL_GPIO_WritePin(doutPort[i], doutPin[i], newState);
    	}
    }
}

////////////////////////////////////////////////////////////////////////////////
// ADC8674

uint16_t adcIn[] = { 0xC000, 0x0000, 0xC000, 0x0000 };
uint16_t adcOut[] = { 0x0000, 0x0000, 0x0000, 0x0000 };

SPI_HandleTypeDef *ADC_hspi = &hspi2;

GPIO_TypeDef* voltSwPorts[4] = { VOLT_SW0_GPIO_Port, VOLT_SW1_GPIO_Port, VOLT_SW2_GPIO_Port, VOLT_SW3_GPIO_Port };
uint16_t      voltSwPins [4] = { VOLT_SW0_Pin,       VOLT_SW1_Pin,       VOLT_SW2_Pin,       VOLT_SW3_Pin       };

GPIO_TypeDef* currSwPorts[4] = { CURR_SW0_GPIO_Port, CURR_SW1_GPIO_Port, CURR_SW2_GPIO_Port, CURR_SW3_GPIO_Port };
uint16_t      currSwPins [4] = { CURR_SW0_Pin,       CURR_SW1_Pin,       CURR_SW2_Pin,       CURR_SW3_Pin       };

GPIO_TypeDef* tempSwPorts[2] = { TEMP_SW_1_GPIO_Port, TEMP_SW_2_GPIO_Port };
uint16_t      tempSwPins [2] = { TEMP_SW_1_Pin,       TEMP_SW_2_Pin,      };

uint16_t ADC_samples[4];

void ADC_Setup() {
	for (int i = 0; i < 4; i++) {
		HAL_GPIO_WritePin(voltSwPorts[i], voltSwPins[i], GPIO_PIN_SET);
		HAL_GPIO_WritePin(currSwPorts[i], currSwPins[i], GPIO_PIN_RESET);
	}

	ADC_CS_GPIO_Port->BSRR = ADC_CS_Pin; // SET ADC CS
}

void ADC_Loop(FromMasterToSlaveParamsChange *newState) {
	const int TIMEOUT = 5;

	for (int i = 0; i < 4; i++) {
		if (newState->ain[i].mode != currentState.ain[i].mode) {
			HAL_GPIO_WritePin(voltSwPorts[i], voltSwPins[i], newState->ain[i].mode == SOURCE_MODE_VOLTAGE ? GPIO_PIN_SET : GPIO_PIN_RESET);
			HAL_GPIO_WritePin(currSwPorts[i], currSwPins[i], newState->ain[i].mode == SOURCE_MODE_CURRENT ? GPIO_PIN_SET : GPIO_PIN_RESET);
		}

		if (newState->ain[i].range != currentState.ain[i].range) {
			adcIn[0] = ((((0x05 + i) << 1) | 1) << 8) | newState->ain[i].range;
			ADC_CS_GPIO_Port->BSRR = (uint32_t)ADC_CS_Pin << 16U; // RESET ADC CS
			HAL_SPI_TransmitReceive(ADC_hspi, (uint8_t *)adcIn, (uint8_t *)adcOut, 4, TIMEOUT);
			ADC_CS_GPIO_Port->BSRR = ADC_CS_Pin; // SET ADC CS
		}

		if (newState->ain[i].tempSensorBias != currentState.ain[i].tempSensorBias) {
			HAL_GPIO_WritePin(tempSwPorts[i], tempSwPins[i], newState->ain[i].tempSensorBias ? GPIO_PIN_SET : GPIO_PIN_RESET);
		}
	}

	uint32_t  manualChannelSelect[4] = { 0xC400, 0xC800, 0xCC00, 0xC000};

	adcIn[0] = 0xC000;
	ADC_CS_GPIO_Port->BSRR = (uint32_t)ADC_CS_Pin << 16U; // RESET ADC CS
	HAL_SPI_TransmitReceive(ADC_hspi, (uint8_t *)adcIn, (uint8_t *)adcOut, 4, TIMEOUT);
	ADC_CS_GPIO_Port->BSRR = ADC_CS_Pin; // SET ADC CS

	for (int i = 0; i < 4; i++) {
		adcIn[0] = manualChannelSelect[i];
		ADC_CS_GPIO_Port->BSRR = (uint32_t)ADC_CS_Pin << 16U; // RESET ADC CS
		HAL_SPI_TransmitReceive(ADC_hspi, (uint8_t *)adcIn, (uint8_t *)adcOut, 4, TIMEOUT);
		ADC_CS_GPIO_Port->BSRR = ADC_CS_Pin; // SET ADC CS

		ADC_samples[i] = adcOut[1];
	}
}

////////////////////////////////////////////////////////////////////////////////
// DAC7760

#define DAC7760_CONTROL_REGISTER 0x55
#define DAC7760_CONFIGURATION_REGISTER 0x57
#define DAC7760_DATA_REGISTER 0x01

void DAC_SpiWrite(int i, uint8_t b0, uint8_t b1, uint8_t b2) {
    uint8_t buf[3] = { b0, b1, b2 };

    HAL_SPI_Transmit(&hspi1, buf, 3, 100);

    if (i == 0) {
    	HAL_GPIO_WritePin(DAC_CS_1_GPIO_Port, DAC_CS_1_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(DAC_CS_1_GPIO_Port, DAC_CS_1_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(DAC_CS_2_GPIO_Port, DAC_CS_2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DAC_CS_2_GPIO_Port, DAC_CS_2_Pin, GPIO_PIN_RESET);
    }
}

void DAC_Setup(int i) {
	DAC_SpiWrite(i, DAC7760_CONTROL_REGISTER, 0, 0);
	DAC_SpiWrite(i, DAC7760_CONFIGURATION_REGISTER, 0, 0);
	DAC_SpiWrite(i, DAC7760_DATA_REGISTER, 0, 0);
}

void DAC_Loop(int i, FromMasterToSlaveParamsChange *newState) {
	uint8_t newOutputEnabled = newState->aout_dac7760[i].outputEnabled;
	uint8_t newOutputRange = newState->aout_dac7760[i].outputRange;
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

	float newOutputValue = newState->aout_dac7760[i].outputValue;
	if (
		newOutputValue != currentState.aout_dac7760[i].outputValue ||
		newOutputRange != currentState.aout_dac7760[i].outputRange
	) {
		uint16_t dacValue;

		float min = 0;
		float max = 0;

		if (newOutputRange == 0) {
			min = 0;
			max = 5.0f;
		} else if (newOutputRange == 1) {
			min = 0;
			max = 10.0f;
		} else if (newOutputRange == 2) {
			min = -5.0f;
			max = 5.0f;
		} else if (newOutputRange == 3) {
			min = -10.0f;
			max = 10.0f;
		} else if (newOutputRange == 5) {
			min = 4E-3f;
			max = 20E-3f;
		} else if (newOutputRange == 6) {
			min = 0;
			max = 20E-3f;
		} else if (newOutputRange == 7) {
			min = 0;
			max = 24E-3f;
		}

		if (newOutputValue <= min) {
			dacValue = 0;
		} else if (newOutputValue >= max) {
			dacValue = 65535;
		} else {
			dacValue = (uint16_t)roundf(65535.0f * (newOutputValue - min) / (max - min));
		}

		DAC_SpiWrite(i, DAC7760_DATA_REGISTER, dacValue >> 8, dacValue & 0xFF);
	}
}

////////////////////////////////////////////////////////////////////////////////
// DAC7563

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

void DACDual_Loop(int i, FromMasterToSlaveParamsChange *newState) {
	float newVoltage = newState->aout_dac7563[i].voltage;
	if (newVoltage != currentState.aout_dac7563[i].voltage) {
		uint16_t dacValue;

		float min = -10.0f;
		float max = 10.0f;

		if (newVoltage <= min) {
			dacValue = 0;
		} else if (newVoltage >= max) {
			dacValue = 65535;
		} else {
			dacValue = (uint16_t)roundf(65535.0f * (newVoltage - min) / (max - min));
		}

		if (i == 0) {
			DACDual_SpiWrite(0b00011000, dacValue >> 8, dacValue & 0xFF);
		} else {
			DACDual_SpiWrite(0b00011001, dacValue >> 8, dacValue & 0xFF);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
// PWM outputs

void PWM_Setup() {
	HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_SET);
}

void PWM_Loop(int i, FromMasterToSlaveParamsChange *newState) {
	float newFreq = newState->pwm[i].freq;
	float newDuty = newState->pwm[i].duty;
	if (newFreq != currentState.pwm[i].freq || newDuty != currentState.pwm[i].duty) {
		if (i == 0) {
			if (currentState.pwm[i].freq > 0) {
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
			}
			HAL_TIM_PWM_DeInit(&htim4);
			TIM4_Init();
		} else {
			if (currentState.pwm[i].freq > 0) {
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
			}
			HAL_TIM_PWM_DeInit(&htim4);
			TIM4_Init();
		}

		uint32_t prescaler = sqrt(90000000.0f / newFreq) - 1;
		uint32_t period = prescaler;
		uint32_t pulse = newDuty == 100.0f ? period + 1 : (uint32_t)roundf(period * newDuty / 100.0f);

		if (i == 0) {
			TIM4->PSC = prescaler;
			TIM4->ARR = period;
			TIM4->CCR1 = pulse;
		} else {
			TIM4->PSC = prescaler;
			TIM4->ARR = period;
			TIM4->CCR2 = pulse;
		}

	    if (i == 0) {
	    	if (newFreq > 0) {
	    		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	    	}
	    } else {
	    	if (newFreq > 0) {
	    		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	    	}
	    }
	}
}

////////////////////////////////////////////////////////////////////////////////
// MASTER - SLAVE Communication

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
		transferCompleted = 0;
		HAL_StatusTypeDef result = HAL_SPI_TransmitReceive_DMA(&hspi4, (uint8_t *)&txBuffer, (uint8_t *)&rxBuffer, sizeof(rxBuffer));
		if (result == HAL_OK) {
			HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_RESET);
			while (!transferCompleted) {
			}
			if (transferCompleted == 1) {
				break;
			}
		}
		HAL_Delay(1);
    }
}

void beginTransfer() {
	FromSlaveToMaster *slaveToMaster = (FromSlaveToMaster *)output;

	slaveToMaster->dinStates = dinStates;

	slaveToMaster->ainValues[0] = ADC_samples[0];
	slaveToMaster->ainValues[1] = ADC_samples[1];
	slaveToMaster->ainValues[2] = ADC_samples[2];
	slaveToMaster->ainValues[3] = ADC_samples[3];

	if (BSP_PlatformIsDetected() == SD_PRESENT) {
		slaveToMaster->flags |= FLAG_SD_CARD_PRESENT;
	}

	if (currentState.dinReadPeriod > 0) {
		uint32_t diff = g_dinReadBufferIndex - g_dinReadBufferTransferredIndex;

		slaveToMaster->flags |= FLAG_DIN_READ_OVERFLOW;

		if (diff > MAX_DIN_VALUES) {
			diff = MAX_DIN_VALUES;
		}
		slaveToMaster->numDinValues = diff;

		if (diff > 0) {
			uint32_t from = g_dinReadBufferTransferredIndex % DIN_READ_BUFFER_SIZE;

			g_dinReadBufferTransferredIndex += diff;

			uint32_t to = g_dinReadBufferTransferredIndex % DIN_READ_BUFFER_SIZE;

			if (from < to) {
				memcpy(slaveToMaster->buffer, g_dinReadBuffer + from, diff);
			} else {
				memcpy(slaveToMaster->buffer, g_dinReadBuffer + from, diff - to);
				memcpy(slaveToMaster->buffer, g_dinReadBuffer, to);
			}
		}

	} else {
		slaveToMaster->numDinValues = 0;
	}

    transferCompleted = 0;
    HAL_SPI_TransmitReceive_DMA(&hspi4, output, input, sizeof(FromSlaveToMaster));
    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_RESET);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == ADC_hspi) {
		return;
	}

    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_SET);
	transferCompleted = 1;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == ADC_hspi) {
		return;
	}

    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_SET);
	transferCompleted = 2;
}

////////////////////////////////////////////////////////////////////////////////
// Setup & Loop

void setup() {
	resetState();

    slaveSynchro();

    beginTransfer();

    //
	Din_Setup();

	ADC_Setup();

	DAC_Setup(0);
    DAC_Setup(1);

    DACDual_Setup();

    PWM_Setup();
    //
}

void loop() {
	uint32_t startTick = HAL_GetTick();
	while (!transferCompleted) {
		if (HAL_GetTick() - startTick > 500) {
			HAL_SPI_Abort(&hspi4);
			HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_SET);
			transferCompleted = 2;
			break;
		}
	}

	FromSlaveToMaster *slaveToMaster = (FromSlaveToMaster *)output;

	slaveToMaster->flags = 0;

	if (transferCompleted == 1) {
		FromMasterToSlaveParamsChange *newState = (FromMasterToSlaveParamsChange *)input;

		if (newState->operation == DISK_DRIVER_OPERATION_NONE) {
			if (newState->doutStates != currentState.doutStates) {
				updateDoutStates(newState->doutStates);
			}

			DinRead_Loop(newState);

			Din_Loop(newState);

			ADC_Loop(newState);

			DAC_Loop(0, newState);
			DAC_Loop(1, newState);

			DACDual_Loop(0, newState);
			DACDual_Loop(1, newState);

			PWM_Loop(0, newState);
			// PWM_Loop(1, newState);

			memcpy(&currentState, newState, sizeof(FromMasterToSlaveParamsChange));
		} else {
			FromMasterToSlaveDiskDriveOperation *diskDriveOperation = (FromMasterToSlaveDiskDriveOperation *)input;

			if (diskDriveOperation->operation == DISK_DRIVER_OPERATION_INITIALIZE) {
				slaveToMaster->diskOperationResult = (uint32_t)SD_Driver.disk_initialize(0);
			} else if (diskDriveOperation->operation == DISK_DRIVER_OPERATION_STATUS) {
				slaveToMaster->diskOperationResult = (uint32_t)SD_Driver.disk_status(0);
			} else if (diskDriveOperation->operation == DISK_DRIVER_OPERATION_READ) {
				slaveToMaster->diskOperationResult = (uint32_t)SD_Driver.disk_read(0, slaveToMaster->buffer, diskDriveOperation->sector, 1);
			} else if (diskDriveOperation->operation == DISK_DRIVER_OPERATION_WRITE) {
				slaveToMaster->diskOperationResult = (uint32_t)SD_Driver.disk_write(0, diskDriveOperation->buffer, diskDriveOperation->sector, 1);
			} else if (diskDriveOperation->operation == DISK_DRIVER_OPERATION_IOCTL) {
				slaveToMaster->diskOperationResult = (uint32_t)SD_Driver.disk_ioctl(0, diskDriveOperation->cmd, diskDriveOperation->buffer);
				memcpy(slaveToMaster->buffer, diskDriveOperation->buffer, DISK_DRIVER_IOCTL_BUFFER_MAX_SIZE);
			}

			slaveToMaster->flags |= FLAG_DISK_OPERATION_RESPONSE;
		}
	}

	beginTransfer();
}

