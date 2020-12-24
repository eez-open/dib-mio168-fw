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

#define READ_PIN(PORT, PIN) ((PORT->IDR & PIN) ? 1 : 0)
#define SET_PIN(PORT, PIN) PORT->BSRR = PIN
#define RESET_PIN(PORT, PIN) PORT->BSRR = (uint32_t)PIN << 16

////////////////////////////////////////////////////////////////////////////////

// TODO remove after debugging
volatile uint32_t g_debugVarCrcError;
volatile uint32_t g_debugVarOtherError;
volatile uint32_t g_debugVarDiff;
volatile uint32_t g_debugVarWriting;
volatile uint32_t g_debugVarState;
volatile uint32_t g_debugVarRequestStructSize;

////////////////////////////////////////////////////////////////////////////////

extern "C" SPI_HandleTypeDef hspi2;
extern "C" SPI_HandleTypeDef hspi3;
extern "C" SPI_HandleTypeDef hspi4;

extern "C" TIM_HandleTypeDef htim2; // for PWM1 output
extern "C" TIM_HandleTypeDef htim3; // for PWM2 output

extern "C" TIM_HandleTypeDef htim6; // for DIN's data logging

////////////////////////////////////////////////////////////////////////////////

SPI_HandleTypeDef *hspiDAC = &hspi2; // for DAC7760 and DAC7563
SPI_HandleTypeDef *hspiADC = &hspi3; // for ADC
SPI_HandleTypeDef *hspiMaster = &hspi4; // for MASTER-SLAVE communication

////////////////////////////////////////////////////////////////////////////////

extern "C" void TIM2_Init(void);
extern "C" void TIM3_Init(void);

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
static const size_t CHANNEL_LABEL_MAX_LENGTH = 5;

////////////////////////////////////////////////////////////////////////////////

enum Command {
	COMMAND_NONE,

    COMMAND_GET_INFO,
    COMMAND_GET_STATE,
    COMMAND_SET_PARAMS,

    COMMAND_DLOG_RECORDING_START,
    COMMAND_DLOG_RECORDING_STOP,

    COMMAND_DISK_DRIVE_INITIALIZE,
    COMMAND_DISK_DRIVE_STATUS,
    COMMAND_DISK_DRIVE_READ,
    COMMAND_DISK_DRIVE_WRITE,
    COMMAND_DISK_DRIVE_IOCTL
};

#define GET_STATE_COMMAND_FLAG_SD_CARD_PRESENT (1 << 0)

#define DLOG_STATE_IDLE 0
#define DLOG_STATE_EXECUTING 1
#define DLOG_STATE_FINISH_RESULT_OK 2
#define DLOG_STATE_FINISH_RESULT_BUFFER_OVERFLOW 3
#define DLOG_STATE_FINISH_RESULT_MASS_STORAGE_ERROR 4

struct DlogState {
    uint8_t state; // DLOG_STATE_...
    uint32_t fileLength;
    uint32_t numSamples;
};

struct SetParams {
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
};

struct DlogRecordingStart {
    float period;
    float duration;
    uint32_t resources;
    char filePath[MAX_PATH_LENGTH + 1];
    char dinLabels[8 * (CHANNEL_LABEL_MAX_LENGTH + 1)];
    char doutLabels[8 * (CHANNEL_LABEL_MAX_LENGTH + 1)];
    char ainLabels[4 * (CHANNEL_LABEL_MAX_LENGTH + 1)];
};

#define DISK_DRIVER_IOCTL_BUFFER_MAX_SIZE 4

struct Request {
    uint8_t command;

    union {
        struct {
            uint32_t fatTime;
        } getState;

		SetParams setParams;

        DlogRecordingStart dlogRecordingStart;

        struct {
            uint32_t sector;
        } diskDriveRead;

        struct {
            uint32_t sector;
            uint8_t buffer[512];
        } diskDriveWrite;

        struct {
            uint8_t cmd;
            uint8_t buffer[DISK_DRIVER_IOCTL_BUFFER_MAX_SIZE];
        } diskDriveIoctl;
    };
};

struct Response {
	uint8_t command;

    union {
        struct {
            uint8_t firmwareMajorVersion;
            uint8_t firmwareMinorVersion;
            uint32_t idw0;
            uint32_t idw1;
            uint32_t idw2;
        } getInfo;

        struct {
            uint8_t flags; // GET_STATE_COMMAND_FLAG_...
            uint8_t dinStates;
            uint16_t ainValues[4];
            DlogState dlogState;
        } getState;

        struct {
            uint8_t result; // 1 - success, 0 - failure
        } setParams;

        struct {
            uint8_t result;
        } diskDriveInitialize;

        struct {
            uint8_t result;
        } diskDriveStatus;

        struct {
            uint8_t result;
            uint8_t buffer[512];
        } diskDriveRead;

        struct {
            uint8_t result;
        } diskDriveWrite;

        struct {
            uint8_t result;
            uint8_t buffer[DISK_DRIVER_IOCTL_BUFFER_MAX_SIZE];
        } diskDriveIoctl;
	};
};

////////////////////////////////////////////////////////////////////////////////

SetParams currentState;

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

uint8_t readDataInputs() {
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
		HAL_GPIO_WritePin(dinRangePorts[i], dinRangePins[i], GPIO_PIN_RESET);
	}

	for (int i = 0; i < 2; i++) {
		HAL_GPIO_WritePin(dinSpeedPorts[i], dinSpeedPins[i], GPIO_PIN_RESET);
	}
}

void Din_SetParams(SetParams &newState) {
	for (int i = 0; i < 8; i++) {
		int newRange = newState.dinRanges & (1 << i);
		if (newRange != (currentState.dinRanges & (1 << i))) {
			HAL_GPIO_WritePin(dinRangePorts[i], dinRangePins[i], newRange ? GPIO_PIN_SET : GPIO_PIN_RESET);
		}
	}

	for (int i = 0; i < 2; i++) {
		int newSpeed = newState.dinSpeeds & (1 << i);
		if (newSpeed != (currentState.dinSpeeds & (1 << i))) {
			HAL_GPIO_WritePin(dinSpeedPorts[i], dinSpeedPins[i], newSpeed ? GPIO_PIN_SET : GPIO_PIN_RESET);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
// Digital Outputs

static GPIO_TypeDef *doutPort[8] = { DOUT0_GPIO_Port, DOUT1_GPIO_Port, DOUT2_GPIO_Port, DOUT3_GPIO_Port, DOUT4_GPIO_Port, DOUT5_GPIO_Port, DOUT6_GPIO_Port, DOUT7_GPIO_Port };
static uint16_t doutPin[8] = { DOUT0_Pin, DOUT1_Pin, DOUT2_Pin, DOUT3_Pin, DOUT4_Pin, DOUT5_Pin, DOUT6_Pin, DOUT7_Pin };

void updateDoutStates(uint8_t newDoutStates) {
	uint8_t currentDoutStates = currentState.doutStates;

    if (currentDoutStates == 0 && newDoutStates != 0) {
    	HAL_GPIO_WritePin(DOUT_EN_GPIO_Port, DOUT_EN_Pin, GPIO_PIN_SET);
    } else if (currentDoutStates != 0 && newDoutStates == 0) {
    	HAL_GPIO_WritePin(DOUT_EN_GPIO_Port, DOUT_EN_Pin, GPIO_PIN_RESET);
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
// ADC (ADS131E04)

uint16_t ADC_samples[4];

void ADC_Setup() {
	// hspiADC
#if 0
	uint8_t buf[10];
	HAL_SPI_Transmit(hspiADC, buf, 10, 100);

	SET_PIN(ADC_START_GPIO_Port, ADC_START_Pin);
	for (int i = 0; i < 10; i++) {}
	RESET_PIN(ADC_START_GPIO_Port, ADC_START_Pin);

	while (READ_PIN(ADC_DRDY_GPIO_Port, ADC_DRDY_Pin)) {
	}

	// select ADC
	RESET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin)

	// deselect ADC
	SET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin)
#endif
}

void ADC_SetParams(SetParams &newState) {
}

void ADC_Measure() {
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

void DACDual_SetParams(int i, SetParams &newState) {
	float newVoltage = newState.aout_dac7563[i].voltage;
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
	HAL_GPIO_WritePin(DOUT_EN_GPIO_Port, DOUT_EN_Pin, GPIO_PIN_SET);
}

void PWM_SetParams(int i, SetParams &newState) {
	float newFreq = newState.pwm[i].freq;
	float newDuty = newState.pwm[i].duty;
	if (newFreq != currentState.pwm[i].freq || newDuty != currentState.pwm[i].duty) {
		if (i == 0) {
			if (currentState.pwm[i].freq > 0) {
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			}
			HAL_TIM_PWM_DeInit(&htim2);
			TIM2_Init();
		} else {
			if (currentState.pwm[i].freq > 0) {
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			}
			HAL_TIM_PWM_DeInit(&htim3);
			TIM3_Init();
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
	    		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	    	}
	    } else {
	    	if (newFreq > 0) {
	    		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	    	}
	    }
	}
}

////////////////////////////////////////////////////////////////////////////////

DlogState dlogState;

uint8_t g_dinResources;
uint8_t g_doutResources;

uint8_t g_writerBuffer[64 * 1024];
dlog_file::Writer g_writer(g_writerBuffer, sizeof(g_writerBuffer));

uint8_t g_fileWriteBuffer[32 * 1024];
uint32_t g_fileWriteBufferIndex;

uint32_t g_numSamples;
uint32_t g_maxNumSamples;
uint32_t g_lastSavedBufferIndex;

bool g_mounted = false;
FIL g_file;

uint8_t readDataInputs();

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim6) {
		if (g_numSamples < g_maxNumSamples) {
			// this is valid sample
			g_writer.writeBit(1);

			if (g_dinResources & 0b00000001) g_writer.writeBit(READ_PIN(DIN0_GPIO_Port, DIN0_Pin) ? 1 : 0);
			if (g_dinResources & 0b00000010) g_writer.writeBit(READ_PIN(DIN1_GPIO_Port, DIN1_Pin) ? 1 : 0);
			if (g_dinResources & 0b00000100) g_writer.writeBit(READ_PIN(DIN2_GPIO_Port, DIN2_Pin) ? 1 : 0);
			if (g_dinResources & 0b00001000) g_writer.writeBit(READ_PIN(DIN3_GPIO_Port, DIN3_Pin) ? 1 : 0);
			if (g_dinResources & 0b00010000) g_writer.writeBit(READ_PIN(DIN4_GPIO_Port, DIN4_Pin) ? 1 : 0);
			if (g_dinResources & 0b00100000) g_writer.writeBit(READ_PIN(DIN5_GPIO_Port, DIN5_Pin) ? 1 : 0);
			if (g_dinResources & 0b01000000) g_writer.writeBit(READ_PIN(DIN6_GPIO_Port, DIN6_Pin) ? 1 : 0);
			if (g_dinResources & 0b10000000) g_writer.writeBit(READ_PIN(DIN7_GPIO_Port, DIN7_Pin) ? 1 : 0);

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

void DLOG_FillParameters(DlogRecordingStart &dlogRecordingStart, dlog_file::Parameters &dlogFileParameters) {
	memset(&dlogFileParameters, 0, sizeof(dlogFileParameters));

    dlogFileParameters.xAxis.unit = UNIT_SECOND;
    dlogFileParameters.xAxis.step = dlogRecordingStart.period;
    dlogFileParameters.xAxis.range.min = 0;
    dlogFileParameters.xAxis.range.max = dlogRecordingStart.duration;

    dlogFileParameters.yAxisScale = dlog_file::SCALE_LINEAR;

    g_dinResources = dlogRecordingStart.resources;
    for (int i = 0; i < 8; i++) {
    	if (g_dinResources & (1 << i)) {
    		auto &yAxis = dlogFileParameters.yAxes[dlogFileParameters.numYAxes++];

            yAxis.unit = UNIT_BIT;
            yAxis.range.min = 0;
            yAxis.range.max = 1;
            yAxis.channelIndex = i;

            strcpy(yAxis.label, dlogRecordingStart.dinLabels + i * (CHANNEL_LABEL_MAX_LENGTH + 1));
    	}
    }

    g_doutResources = dlogRecordingStart.resources >> 8;
    for (int i = 0; i < 8; i++) {
    	if (g_doutResources & (1 << i)) {
    		auto &yAxis = dlogFileParameters.yAxes[dlogFileParameters.numYAxes++];

            yAxis.unit = UNIT_BIT;
            yAxis.range.min = 0;
            yAxis.range.max = 1;
            yAxis.channelIndex = 8 + i;

            strcpy(yAxis.label, dlogRecordingStart.doutLabels + i * (CHANNEL_LABEL_MAX_LENGTH + 1));
    	}
    }

    dlogFileParameters.period = dlogRecordingStart.period;
	dlogFileParameters.duration = dlogRecordingStart.duration;
}

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

FRESULT DLOG_WriteHeader(DlogRecordingStart &dlogRecordingStart) {
	auto result = DLOG_CreateRecordingsDir();
	if (result != FR_OK) {
		return result;
	}

	result = f_open(&g_file, dlogRecordingStart.filePath, FA_WRITE | FA_CREATE_ALWAYS);
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

FRESULT DLOG_StartFile(DlogRecordingStart &dlogRecordingStart) {
	dlog_file::Parameters dlogFileParameters;
	DLOG_FillParameters(dlogRecordingStart, dlogFileParameters);

	g_writer.reset();
	g_writer.writeFileHeaderAndMetaFields(dlogFileParameters);
	return DLOG_WriteHeader(dlogRecordingStart);
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
			g_debugVarDiff = nInvalid;

			// invalid samples
			memset(g_fileWriteBuffer + g_fileWriteBufferIndex, 0, nInvalid);
			g_fileWriteBufferIndex += nInvalid;
			g_lastSavedBufferIndex += nInvalid;

			n -= nInvalid;
		} else {
			// TODO remove after debugging
			g_debugVarDiff = 0;
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
		g_debugVarWriting = btw;
		UINT bw;
		FRESULT result = f_write(&g_file, g_fileWriteBuffer, btw, &bw);
		g_debugVarWriting = 0;

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
		return result != FR_OK ? DLOG_STATE_FINISH_RESULT_MASS_STORAGE_ERROR : DLOG_STATE_FINISH_RESULT_OK;
	}

	return DLOG_STATE_FINISH_RESULT_OK;
}

void DLOG_CloseFile() {
	f_close(&g_file);
	// unmount
	f_mount(NULL, 0, 0);
	g_mounted = false;
}

void DLOG_Start(DlogRecordingStart &dlogRecordingStart) {
	auto result = DLOG_StartFile(dlogRecordingStart);
	if (result != FR_OK) {
		DLOG_CloseFile();
		dlogState.state = DLOG_STATE_FINISH_RESULT_MASS_STORAGE_ERROR;
	} else {
		TIM6->ARR = (uint16_t)(dlogRecordingStart.period * 10000000) - 1; // convert to microseconds

		g_numSamples = 0;
		g_maxNumSamples = (uint32_t)(dlogRecordingStart.duration / dlogRecordingStart.period) + 1;

		g_lastSavedBufferIndex = g_writer.getDataOffset();
		g_fileWriteBufferIndex = 0;

		dlogState.fileLength = g_writer.getFileLength();
		dlogState.numSamples = g_numSamples;
		dlogState.state = DLOG_STATE_EXECUTING;

		HAL_TIM_Base_Start_IT(&htim6);
	}
}

void DLOG_LoopWrite() {
	if (g_mounted) {
		auto result = DLOG_WriteFile(false);

		dlogState.fileLength = g_writer.getFileLength();
		dlogState.numSamples = g_numSamples;

		if (result != DLOG_STATE_FINISH_RESULT_OK) {
			dlogState.state = result;
		} else if (g_numSamples >= g_maxNumSamples) {
			dlogState.state = DLOG_STATE_FINISH_RESULT_OK;
		}
	}
}

void DLOG_Stop() {
	HAL_TIM_Base_Stop_IT(&htim6);

	// flush buffer
	uint32_t result = DLOG_STATE_FINISH_RESULT_OK;
	do {
		result = DLOG_WriteFile(true);
		if (result != DLOG_STATE_FINISH_RESULT_OK) {
			break;
		}
	} while (g_lastSavedBufferIndex < g_writer.getBufferIndex());

	dlogState.fileLength = g_writer.getFileLength();
	dlogState.numSamples = g_numSamples;
	dlogState.state = result;

	DLOG_CloseFile();
}

////////////////////////////////////////////////////////////////////////////////
// MASTER - SLAVE Communication

uint32_t input[(sizeof(Request) + 3) / 4 + 1];
uint32_t output[(sizeof(Request) + 3) / 4];
volatile int transferCompleted;

void beginTransfer() {
    transferCompleted = 0;
    HAL_SPI_TransmitReceive_DMA(hspiMaster, (uint8_t *)output, (uint8_t *)input, sizeof(Request));
    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_RESET);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == hspiADC) {
		return;
	}

	g_debugVarCrcError = 0;
	g_debugVarOtherError = 0;

	HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_SET);
	transferCompleted = 1;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == hspiADC) {
		return;
	}

	// TODO remove after debugging
	if (hspi->ErrorCode == HAL_SPI_ERROR_CRC) {
		g_debugVarCrcError++;
	} else {
		g_debugVarOtherError++;
	}

    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_SET);
	transferCompleted = 2;
}

int waitTransferCompletion() {
	uint32_t startTick = HAL_GetTick();
	while (!transferCompleted) {
		if (HAL_GetTick() - startTick > 500) {
			g_debugVarState = 1;
			HAL_SPI_Abort(hspiMaster);
			HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_SET);
			return 2;
		}

		DLOG_LoopWrite();
	}
	g_debugVarState = 2;
	return transferCompleted;
}

////////////////////////////////////////////////////////////////////////////////

void Command_GetInfo(Request &request, Response &response) {
	response.getInfo.firmwareMajorVersion = FIRMWARE_VERSION_MAJOR;
	response.getInfo.firmwareMinorVersion = FIRMWARE_VERSION_MINOR;
	response.getInfo.idw0 = HAL_GetUIDw0();
	response.getInfo.idw1 = HAL_GetUIDw1();
	response.getInfo.idw2 = HAL_GetUIDw2();
}

void Command_GetState(Request &request, Response &response) {
	g_fatTime = request.getState.fatTime;

	response.getState.flags = 0;
	if (BSP_PlatformIsDetected() == SD_PRESENT) {
		response.getState.flags |= GET_STATE_COMMAND_FLAG_SD_CARD_PRESENT;
	}

	response.getState.dinStates = readDataInputs();

	ADC_Measure();

	response.getState.ainValues[0] = ADC_samples[0];
	response.getState.ainValues[1] = ADC_samples[1];
	response.getState.ainValues[2] = ADC_samples[2];
	response.getState.ainValues[3] = ADC_samples[3];

	memcpy(&response.getState.dlogState, &dlogState, sizeof(DlogState));
}

void Command_SetParams(Request &request, Response &response) {
	Din_SetParams(request.setParams);
	ADC_SetParams(request.setParams);
	DAC_SetParams(0, request.setParams);
	DAC_SetParams(1, request.setParams);
	DACDual_SetParams(0, request.setParams);
	DACDual_SetParams(1, request.setParams);
	PWM_SetParams(0, request.setParams);
	PWM_SetParams(1, request.setParams);

	memcpy(&currentState, &request.setParams, sizeof(SetParams));

	response.setParams.result = 1; // success
}

void Command_DlogRecordingStart(Request &request, Response &response) {
	DLOG_Start(request.dlogRecordingStart);
}

void Command_DlogRecordingStop(Request &request, Response &response) {
	DLOG_Stop();
}

void Command_DiskDriveInitialize(Request &request, Response &response) {
	if (g_mounted) {
		response.diskDriveInitialize.result = STA_NOINIT;
	} else {
		response.diskDriveInitialize.result = (uint32_t)SD_Driver.disk_initialize(0);
	}
}

void Command_DiskDriveStatus(Request &request, Response &response) {
	if (g_mounted) {
		response.diskDriveStatus.result = STA_NOINIT;
	} else {
		response.diskDriveStatus.result = (uint32_t)SD_Driver.disk_status(0);
	}
}

void Command_DiskDriveRead(Request &request, Response &response) {
	if (g_mounted) {
		response.diskDriveRead.result = RES_ERROR;
	} else {
		response.diskDriveRead.result = (uint32_t)SD_Driver.disk_read(0,
			response.diskDriveRead.buffer, request.diskDriveRead.sector, 1);
	}
}

void Command_DiskDriveWrite(Request &request, Response &response) {
	if (g_mounted) {
		response.diskDriveWrite.result = RES_ERROR;
	} else {
		response.diskDriveWrite.result = (uint32_t)SD_Driver.disk_write(0,
			request.diskDriveWrite.buffer, request.diskDriveWrite.sector, 1);
	}
}

void Command_DiskDriveIoctl(Request &request, Response &response) {
	if (g_mounted) {
		response.diskDriveInitialize.result = RES_ERROR;
	} else {
		response.diskDriveIoctl.result = (uint32_t)SD_Driver.disk_ioctl(0,
			request.diskDriveIoctl.cmd, request.diskDriveIoctl.buffer);

		memcpy(response.diskDriveRead.buffer, request.diskDriveIoctl.buffer,
			DISK_DRIVER_IOCTL_BUFFER_MAX_SIZE);
	}
}

////////////////////////////////////////////////////////////////////////////////
// Setup & Loop

extern "C" void setup() {
	g_debugVarRequestStructSize = sizeof(Request);

	resetState();

	Din_Setup();
	ADC_Setup();
	DAC_Setup(0);
    DAC_Setup(1);
    DACDual_Setup();
    PWM_Setup();
}

extern "C" void loop() {
	g_debugVarState = 0;
	beginTransfer();

    auto transferResult = waitTransferCompletion();

	g_debugVarState = 3;

    Request &request = *(Request *)input;
    Response &response = *(Response *)output;

    if (transferResult == 1) {
    	g_debugVarState = 4;
    	response.command = 0x80 | request.command;

    	if (request.command == COMMAND_GET_INFO) {
			Command_GetInfo(request, response);
		} else if (request.command == COMMAND_GET_STATE) {
			Command_GetState(request, response);
		} else if (request.command == COMMAND_SET_PARAMS) {
			Command_SetParams(request, response);
		} else if (request.command == COMMAND_DLOG_RECORDING_START) {
			Command_DlogRecordingStart(request, response);
		} else if (request.command == COMMAND_DLOG_RECORDING_STOP) {
			Command_DlogRecordingStop(request, response);
		} else if (request.command == COMMAND_DISK_DRIVE_INITIALIZE) {
			Command_DiskDriveInitialize(request, response);
		} else if (request.command == COMMAND_DISK_DRIVE_STATUS) {
			Command_DiskDriveStatus(request, response);
		} else if (request.command == COMMAND_DISK_DRIVE_READ) {
			Command_DiskDriveRead(request, response);
		} else if (request.command == COMMAND_DISK_DRIVE_WRITE) {
			Command_DiskDriveWrite(request, response);
		} else if (request.command == COMMAND_DISK_DRIVE_IOCTL) {
			Command_DiskDriveIoctl(request, response);
		} else {
    		g_debugVarState = 5;
	    	response.command = COMMAND_NONE;
		}
    } else {
    	g_debugVarState = 6;
    	response.command = COMMAND_NONE;
    	HAL_Delay(1);
    }
}
