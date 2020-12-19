#include <math.h>
#include <memory.h>

#include "main.h"
#include "fatfs.h"
#include <bsp_driver_sd.h>
#include <ff_gen_drv.h>
#include <sd_diskio.h>

#include "./dlog_file.h"

using namespace eez;

////////////////////////////////////////////////////////////////////////////////

extern "C" SPI_HandleTypeDef hspi1;
extern "C" SPI_HandleTypeDef hspi2;
extern "C" SPI_HandleTypeDef hspi4;

extern "C" TIM_HandleTypeDef htim4; // for PWM outputs

extern "C" TIM_HandleTypeDef htim6; // for DIN's data logging

////////////////////////////////////////////////////////////////////////////////

SPI_HandleTypeDef *hspiDAC = &hspi1; // for DAC7760 and DAC7563
SPI_HandleTypeDef *hspiADC = &hspi2; // for ADC8674
SPI_HandleTypeDef *hspiMaster = &hspi4; // for MASTER-SLAVE communication

////////////////////////////////////////////////////////////////////////////////

extern "C" void TIM4_Init(void);

////////////////////////////////////////////////////////////////////////////////

DWORD g_fatTime;

////////////////////////////////////////////////////////////////////////////////

enum SourceMode {
	SOURCE_MODE_CURRENT,
	SOURCE_MODE_VOLTAGE,
	SOURCE_MODE_OPEN
};

////////////////////////////////////////////////////////////////////////////////

#define MAX_PATH_LENGTH 255

struct DlogParams {
    float period;
    float duration;
    uint32_t resources;
    char filePath[MAX_PATH_LENGTH + 1];
};

static const size_t CHANNEL_LABEL_MAX_LENGTH = 5;

struct Labels {
    char din[8 * (CHANNEL_LABEL_MAX_LENGTH + 1)];
    char dout[8 * (CHANNEL_LABEL_MAX_LENGTH + 1)];
    char ain[4 * (CHANNEL_LABEL_MAX_LENGTH + 1)];
};

struct FromMasterToSlaveParamsChange {
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

    DlogParams dlog;

    Labels labels;

	uint32_t fatTime;
};

struct FromMasterToSlaveDiskDriveOperation {
    uint8_t operation; // enum DiskDriverOperation
    uint32_t sector;
    uint8_t cmd;
    uint8_t buffer[512];
};

enum DiskDriverOperation {
    DISK_DRIVER_OPERATION_NONE,
    DISK_DRIVER_OPERATION_INITIALIZE,
    DISK_DRIVER_OPERATION_STATUS,
    DISK_DRIVER_OPERATION_READ,
    DISK_DRIVER_OPERATION_WRITE,
    DISK_DRIVER_OPERATION_IOCTL
};

#define FLAG_SD_CARD_PRESENT (1 << 0)
#define FLAG_DLOG_RECORD_FINISHED (1 << 1)
#define FLAG_DLOG_RECORD_STATUS (1 << 2)
#define FLAG_DISK_OPERATION_RESPONSE (1 << 3)

#define DLOG_RECORD_RESULT_OK 0
#define DLOG_RECORD_RESULT_BUFFER_OVERFLOW 1
#define DLOG_RECORD_RESULT_MASS_STORAGE_ERROR 2

#define MAX_DIN_VALUES 100

#define DISK_DRIVER_IOCTL_BUFFER_MAX_SIZE 4

struct DlogStatus {
    uint32_t fileLength;
    uint32_t numSamples;
};

struct FromSlaveToMaster {
	uint8_t flags;
    uint16_t result;
    uint8_t dinStates;
    uint16_t ainValues[4];
    union {
    	uint8_t buffer[512];
    	DlogStatus dlogStatus;
    };
};


////////////////////////////////////////////////////////////////////////////////

FromMasterToSlaveParamsChange currentState;

////////////////////////////////////////////////////////////////////////////////

uint8_t output[sizeof(FromSlaveToMaster)];
uint8_t input[sizeof(FromSlaveToMaster)];

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
    	GPIO_PinState oldState = currentDoutStates & (1 << i) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    	GPIO_PinState newState = newDoutStates & (1 << i) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    	if (oldState != newState) {
    		HAL_GPIO_WritePin(doutPort[i], doutPin[i], newState);
    	}
    }
}

////////////////////////////////////////////////////////////////////////////////
// ADC8674

uint16_t adcIn[] = { 0xC000, 0x0000, 0xC000, 0x0000 };
uint16_t adcOut[] = { 0x0000, 0x0000, 0x0000, 0x0000 };

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
			HAL_SPI_TransmitReceive(hspiADC, (uint8_t *)adcIn, (uint8_t *)adcOut, 4, TIMEOUT);
			ADC_CS_GPIO_Port->BSRR = ADC_CS_Pin; // SET ADC CS
		}

		if (newState->ain[i].tempSensorBias != currentState.ain[i].tempSensorBias) {
			HAL_GPIO_WritePin(tempSwPorts[i], tempSwPins[i], newState->ain[i].tempSensorBias ? GPIO_PIN_SET : GPIO_PIN_RESET);
		}
	}

	uint32_t  manualChannelSelect[4] = { 0xC400, 0xC800, 0xCC00, 0xC000};

	adcIn[0] = 0xC000;
	ADC_CS_GPIO_Port->BSRR = (uint32_t)ADC_CS_Pin << 16U; // RESET ADC CS
	HAL_SPI_TransmitReceive(hspiADC, (uint8_t *)adcIn, (uint8_t *)adcOut, 4, TIMEOUT);
	ADC_CS_GPIO_Port->BSRR = ADC_CS_Pin; // SET ADC CS

	for (int i = 0; i < 4; i++) {
		adcIn[0] = manualChannelSelect[i];
		ADC_CS_GPIO_Port->BSRR = (uint32_t)ADC_CS_Pin << 16U; // RESET ADC CS
		HAL_SPI_TransmitReceive(hspiADC, (uint8_t *)adcIn, (uint8_t *)adcOut, 4, TIMEOUT);
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

    HAL_SPI_Transmit(hspiDAC, buf, 3, 100);

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
    HAL_SPI_Transmit(hspiDAC, buf, 3, 100);
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

uint8_t g_dinResources;
uint8_t g_doutResources;

uint8_t g_writerBuffer[64 * 1024];
dlog_file::Writer g_writer(g_writerBuffer, sizeof(g_writerBuffer));

uint8_t g_fileWriteBuffer[32 * 1024];
uint32_t g_fileWriteBufferIndex;

uint32_t g_numSamples;
uint32_t g_maxNumSamples;
uint32_t g_lastSavedBufferIndex;

uint8_t readDataInputs();

// TODO remove after debugging
volatile uint32_t g_diff;
volatile uint32_t g_writing;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim6) {
		if (g_numSamples < g_maxNumSamples) {
			// this is valid sample
			g_writer.writeBit(1);

			if (g_dinResources & 0b00000001) g_writer.writeBit(DIN0_GPIO_Port->IDR & DIN0_Pin ? 1 : 0);
			if (g_dinResources & 0b00000010) g_writer.writeBit(DIN1_GPIO_Port->IDR & DIN1_Pin ? 1 : 0);
			if (g_dinResources & 0b00000100) g_writer.writeBit(DIN2_GPIO_Port->IDR & DIN2_Pin ? 1 : 0);
			if (g_dinResources & 0b00001000) g_writer.writeBit(DIN3_GPIO_Port->IDR & DIN3_Pin ? 1 : 0);
			if (g_dinResources & 0b00010000) g_writer.writeBit(DIN4_GPIO_Port->IDR & DIN4_Pin ? 1 : 0);
			if (g_dinResources & 0b00100000) g_writer.writeBit(DIN5_GPIO_Port->IDR & DIN5_Pin ? 1 : 0);
			if (g_dinResources & 0b01000000) g_writer.writeBit(DIN6_GPIO_Port->IDR & DIN6_Pin ? 1 : 0);
			if (g_dinResources & 0b10000000) g_writer.writeBit(DIN7_GPIO_Port->IDR & DIN7_Pin ? 1 : 0);

			if (g_doutResources & 0b00000001) g_writer.writeBit(currentState.doutStates & 0b00000001 ? 1 : 0);
			if (g_doutResources & 0b00000010) g_writer.writeBit(currentState.doutStates & 0b00000010 ? 1 : 0);
			if (g_doutResources & 0b00000100) g_writer.writeBit(currentState.doutStates & 0b00000100 ? 1 : 0);
			if (g_doutResources & 0b00001000) g_writer.writeBit(currentState.doutStates & 0b00001000 ? 1 : 0);
			if (g_doutResources & 0b00010000) g_writer.writeBit(currentState.doutStates & 0b00010000 ? 1 : 0);
			if (g_doutResources & 0b00100000) g_writer.writeBit(currentState.doutStates & 0b00100000 ? 1 : 0);
			if (g_doutResources & 0b01000000) g_writer.writeBit(currentState.doutStates & 0b01000000 ? 1 : 0);
			if (g_doutResources & 0b10000000) g_writer.writeBit(currentState.doutStates & 0b10000000 ? 1 : 0);

			g_writer.flushBits();

			g_numSamples++;
		}
	}
}

void DLOG_FillParameters(FromMasterToSlaveParamsChange *newState, dlog_file::Parameters &parameters) {
	memset(&parameters, 0, sizeof(parameters));

    parameters.xAxis.unit = UNIT_SECOND;
    parameters.xAxis.step = newState->dlog.period;
    parameters.xAxis.range.min = 0;
    parameters.xAxis.range.max = newState->dlog.duration;

    parameters.yAxisScale = dlog_file::SCALE_LINEAR;

    g_dinResources = newState->dlog.resources;
    for (int i = 0; i < 8; i++) {
    	if (g_dinResources & (1 << i)) {
    		auto &yAxis = parameters.yAxes[parameters.numYAxes++];

            yAxis.unit = UNIT_BIT;
            yAxis.range.min = 0;
            yAxis.range.max = 1;
            yAxis.channelIndex = i;

            strcpy(yAxis.label, newState->labels.din + i * (CHANNEL_LABEL_MAX_LENGTH + 1));
    	}
    }

    g_doutResources = newState->dlog.resources >> 8;
    for (int i = 0; i < 8; i++) {
    	if (g_doutResources & (1 << i)) {
    		auto &yAxis = parameters.yAxes[parameters.numYAxes++];

            yAxis.unit = UNIT_BIT;
            yAxis.range.min = 0;
            yAxis.range.max = 1;
            yAxis.channelIndex = 8 + i;

            strcpy(yAxis.label, newState->labels.dout + i * (CHANNEL_LABEL_MAX_LENGTH + 1));
    	}
    }

    parameters.period = newState->dlog.period;
	parameters.duration = newState->dlog.duration;
}

bool g_mounted = false;

FRESULT DLOG_CreateRecordingsDir() {
	auto result = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
	if (result != FR_OK) {
		return result;
	}
	g_mounted = true;

    FILINFO fno;
    result = f_stat("/Recordings", &fno);
	if (result != FR_OK) {
		result = f_mkdir("/Recordings");
	}
	return result;
}

FIL g_file;

FRESULT DLOG_WriteHeader(FromMasterToSlaveParamsChange *newState) {
	auto result = DLOG_CreateRecordingsDir();
	if (result != FR_OK) {
		return result;
	}

	result = f_open(&g_file, newState->dlog.filePath, FA_WRITE | FA_CREATE_ALWAYS);
	if (result != FR_OK) {
		return result;
	}

	UINT bw;
	result = f_write(&g_file, g_writer.getBuffer(), g_writer.getDataOffset(), &bw);
	if (result != FR_OK) {
		goto Exit;
	}

	if (bw < g_writer.getDataOffset()) {
		// disk is full
		result = FR_DENIED;
		goto Exit;
	}

Exit:
	return result;
}

FRESULT DLOG_StartFile(FromMasterToSlaveParamsChange *newState) {
	dlog_file::Parameters parameters;
	DLOG_FillParameters(newState, parameters);

	g_writer.reset();
	g_writer.writeFileHeaderAndMetaFields(parameters);
	return DLOG_WriteHeader(newState);
}

uint32_t DLOG_WriteFile(bool flush = false) {
	__disable_irq();
	{
		uint32_t n = sizeof(g_fileWriteBuffer) - g_fileWriteBufferIndex;

		uint32_t writerBufferIndex = g_writer.getBufferIndex();
		uint32_t diff = writerBufferIndex - g_lastSavedBufferIndex;

		if (diff < n) {
			n = diff;
		}

		if (diff > sizeof(g_writerBuffer)) {
			// overflow detected

			uint32_t nInvalid = diff - sizeof(g_writerBuffer);
			if (nInvalid >= n) {
				nInvalid = n;
			}

			// TODO remove after debugging
			g_diff = nInvalid;

			// invalid samples
			memset(g_fileWriteBuffer + g_fileWriteBufferIndex, 0, nInvalid);
			g_fileWriteBufferIndex += nInvalid;
			g_lastSavedBufferIndex += nInvalid;

			n -= nInvalid;
		} else {
			// TODO remove after debugging
			g_diff = 0;
		}

		if (n > 0) {
			uint32_t from = g_lastSavedBufferIndex % sizeof(g_writerBuffer);
			memcpy(g_fileWriteBuffer + g_fileWriteBufferIndex, g_writerBuffer + from, n);
			g_fileWriteBufferIndex += n;
			g_lastSavedBufferIndex += n;
		}
	}
	__enable_irq();

	if (g_fileWriteBufferIndex == sizeof(g_fileWriteBuffer) || flush) {
		UINT btw = g_fileWriteBufferIndex;
		g_writing = btw;
		UINT bw;
		FRESULT result = f_write(&g_file, g_fileWriteBuffer, btw, &bw);
		g_writing = 0;

		if (result != FR_OK) {
			goto Exit;
		}
		if (bw < btw) {
			// disk is full
			result = FR_DENIED;
			goto Exit;
		}

		g_fileWriteBufferIndex = 0;

Exit:
		return result != FR_OK ? DLOG_RECORD_RESULT_MASS_STORAGE_ERROR : DLOG_RECORD_RESULT_OK;
	}

	return DLOG_RECORD_RESULT_OK;
}

void DLOG_CloseFile() {
	f_close(&g_file);
	// unmount
	f_mount(NULL, 0, 0);
	g_mounted = false;
}

void DLOG_LoopWrite() {
	auto slaveToMaster = (FromSlaveToMaster *)output;

	if (g_mounted) {
		auto result = DLOG_WriteFile(false);

		slaveToMaster->dlogStatus.fileLength = g_writer.getFileLength();
		slaveToMaster->dlogStatus.numSamples = g_numSamples;
		slaveToMaster->flags |= FLAG_DLOG_RECORD_STATUS;

		if (result != DLOG_RECORD_RESULT_OK) {
			slaveToMaster->flags |= FLAG_DLOG_RECORD_FINISHED;
			slaveToMaster->result = result;
		} else if (g_numSamples >= g_maxNumSamples) {
			slaveToMaster->flags |= FLAG_DLOG_RECORD_FINISHED;
			slaveToMaster->result = DLOG_RECORD_RESULT_OK;
		}
	}
}

void DLOG_Loop(FromMasterToSlaveParamsChange *newState) {
	auto slaveToMaster = (FromSlaveToMaster *)output;

	if (newState->dlog.period != currentState.dlog.period) {
		if (currentState.dlog.period > 0) {
			HAL_TIM_Base_Stop_IT(&htim6);
		}

		if (newState->dlog.period > 0) {
			auto result = DLOG_StartFile(newState);
			if (result != FR_OK) {
				DLOG_CloseFile();

				slaveToMaster->flags |= FLAG_DLOG_RECORD_FINISHED;
				slaveToMaster->result = DLOG_RECORD_RESULT_MASS_STORAGE_ERROR;
			} else {
				TIM6->ARR = (uint16_t)(newState->dlog.period * 10000000) - 1; // convert to microseconds

				g_numSamples = 0;
				g_maxNumSamples = (uint32_t)(newState->dlog.duration / newState->dlog.period) + 1;

				g_lastSavedBufferIndex = g_writer.getDataOffset();
				g_fileWriteBufferIndex = 0;

				slaveToMaster->dlogStatus.fileLength = g_writer.getFileLength();
				slaveToMaster->dlogStatus.numSamples = g_numSamples;
				slaveToMaster->flags |= FLAG_DLOG_RECORD_STATUS;

				HAL_TIM_Base_Start_IT(&htim6);
			}
		} else if (g_mounted) {
			// flush buffer
			uint32_t result = DLOG_RECORD_RESULT_OK;
			do {
				result = DLOG_WriteFile(true);
				if (result != DLOG_RECORD_RESULT_OK) {
					break;
				}
			} while (g_lastSavedBufferIndex < g_writer.getBufferIndex());

			slaveToMaster->dlogStatus.fileLength = g_writer.getFileLength();
			slaveToMaster->dlogStatus.numSamples = g_numSamples;
			slaveToMaster->flags |= FLAG_DLOG_RECORD_STATUS;

			DLOG_CloseFile();

			slaveToMaster->flags |= FLAG_DLOG_RECORD_FINISHED;
			slaveToMaster->result = result;
		}
	} else {
		DLOG_LoopWrite();
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
        (uint8_t)(idw0 >> 24), (uint8_t)((idw0 >> 16) & 0xFF), (uint8_t)((idw0 >> 8) & 0xFF), (uint8_t)(idw0 & 0xFF),
		(uint8_t)(idw1 >> 24), (uint8_t)((idw1 >> 16) & 0xFF), (uint8_t)((idw1 >> 8) & 0xFF), (uint8_t)(idw1 & 0xFF),
		(uint8_t)(idw2 >> 24), (uint8_t)((idw2 >> 16) & 0xFF), (uint8_t)((idw2 >> 8) & 0xFF), (uint8_t)(idw2 & 0xFF)
    };

    uint8_t rxBuffer[15];

	while (1) {
		transferCompleted = 0;
		HAL_StatusTypeDef result = HAL_SPI_TransmitReceive_DMA(hspiMaster, (uint8_t *)&txBuffer, (uint8_t *)&rxBuffer, sizeof(rxBuffer));
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

    transferCompleted = 0;
    HAL_SPI_TransmitReceive_DMA(hspiMaster, output, input, sizeof(FromSlaveToMaster));
    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_RESET);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == hspiADC) {
		return;
	}

    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_SET);
	transferCompleted = 1;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == hspiADC) {
		return;
	}

    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_SET);
	transferCompleted = 2;
}

////////////////////////////////////////////////////////////////////////////////
// Setup & Loop

extern "C" void setup() {
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

extern "C" void loop() {
	uint32_t startTick = HAL_GetTick();
	while (!transferCompleted) {
		if (HAL_GetTick() - startTick > 500) {
			HAL_SPI_Abort(hspiMaster);
			HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_SET);
			transferCompleted = 2;
			break;
		}

		DLOG_LoopWrite();
	}

	FromSlaveToMaster *slaveToMaster = (FromSlaveToMaster *)output;

	slaveToMaster->flags = 0;

	if (transferCompleted == 1) {
		FromMasterToSlaveParamsChange *newState = (FromMasterToSlaveParamsChange *)input;

		if (newState->operation == DISK_DRIVER_OPERATION_NONE) {
			if (newState->doutStates != currentState.doutStates) {
				updateDoutStates(newState->doutStates);
			}

			g_fatTime = newState->fatTime;

			DLOG_Loop(newState);

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
				if (g_mounted) {
					slaveToMaster->result = STA_NOINIT;
				} else {
					slaveToMaster->result = (uint32_t)SD_Driver.disk_initialize(0);
				}
			} else if (diskDriveOperation->operation == DISK_DRIVER_OPERATION_STATUS) {
				if (g_mounted) {
					slaveToMaster->result = STA_NOINIT;
				} else {
					slaveToMaster->result = (uint32_t)SD_Driver.disk_status(0);
				}
			} else if (diskDriveOperation->operation == DISK_DRIVER_OPERATION_READ) {
				if (g_mounted) {
					slaveToMaster->result = RES_ERROR;
				} else {
					slaveToMaster->result = (uint32_t)SD_Driver.disk_read(0, slaveToMaster->buffer, diskDriveOperation->sector, 1);
				}
			} else if (diskDriveOperation->operation == DISK_DRIVER_OPERATION_WRITE) {
				if (g_mounted) {
					slaveToMaster->result = RES_ERROR;
				} else {
					slaveToMaster->result = (uint32_t)SD_Driver.disk_write(0, diskDriveOperation->buffer, diskDriveOperation->sector, 1);
				}
			} else if (diskDriveOperation->operation == DISK_DRIVER_OPERATION_IOCTL) {
				if (g_mounted) {
					slaveToMaster->result = RES_ERROR;
				} else {
					slaveToMaster->result = (uint32_t)SD_Driver.disk_ioctl(0, diskDriveOperation->cmd, diskDriveOperation->buffer);
					memcpy(slaveToMaster->buffer, diskDriveOperation->buffer, DISK_DRIVER_IOCTL_BUFFER_MAX_SIZE);
				}
			}

			slaveToMaster->flags |= FLAG_DISK_OPERATION_RESPONSE;
		}
	}

	beginTransfer();
}

