#include <math.h>
#include <memory.h>

#include "main.h"
#include "fatfs.h"
#include <bsp_driver_sd.h>
#include <ff_gen_drv.h>
#include <sd_diskio.h>

#include "./dlog_file.h"

using namespace eez;

#define DEBUG_VARS 1

#if DEBUG_VARS
volatile uint32_t g_debugVar1;
volatile uint32_t g_debugVar2;
volatile uint32_t g_debugVar3;
#endif

////////////////////////////////////////////////////////////////////////////////

#define READ_PIN(PORT, PIN) ((PORT->IDR & PIN) ? 1 : 0)
#define SET_PIN(PORT, PIN) PORT->BSRR = PIN
#define RESET_PIN(PORT, PIN) PORT->BSRR = (uint32_t)PIN << 16

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
extern "C" void SPI4_Init(void);

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
            uint8_t afeVersion;
        } getInfo;

        struct {
            uint8_t flags; // GET_STATE_COMMAND_FLAG_...
            uint8_t dinStates;
            float ainValues[4];
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
////////////////////////////////////////////////////////////////////////////////
//
//        Digital Inputs
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//        Digital Outputs
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static GPIO_TypeDef *doutPort[8] = { DOUT0_GPIO_Port, DOUT1_GPIO_Port, DOUT2_GPIO_Port, DOUT3_GPIO_Port, DOUT4_GPIO_Port, DOUT5_GPIO_Port, DOUT6_GPIO_Port, DOUT7_GPIO_Port };
static uint16_t doutPin[8] = { DOUT0_Pin, DOUT1_Pin, DOUT2_Pin, DOUT3_Pin, DOUT4_Pin, DOUT5_Pin, DOUT6_Pin, DOUT7_Pin };

void delayMicroseconds(uint32_t microseconds);

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

//    		SET_PIN(DOUT7_GPIO_Port, DOUT7_Pin);
//    		delayMicroseconds(1);
//    		RESET_PIN(DOUT7_GPIO_Port, DOUT7_Pin);
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

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//        ADC (ADS131E04)
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

template <typename T, typename Total, uint64_t N>
class MovingAverage {
public:
    void operator()(T sample) {
        if (m_numSamples < N) {
            m_samples[m_numSamples++] = sample;
            m_total += sample;
        } else {
            T& oldest = m_samples[m_numSamples++ % N];
			m_total += sample - oldest;
            oldest = sample;
        }
    }

    operator T() const {
		if (m_numSamples < N) {
			return m_total / m_numSamples;
		} else {
			return m_total / N;
		}
    }

    void reset() {
        m_numSamples = 0;
        m_total = 0;
    }

private:
    T m_samples[N];
    uint64_t m_numSamples{0};
    Total m_total{0};
};

float ADC_samples[4];

static const uint64_t ADC_MOVING_AVERAGE_NUM_SAMPLES = 500;
MovingAverage<float, double, ADC_MOVING_AVERAGE_NUM_SAMPLES> g_adcMovingAverage[4];

uint8_t ADC_pga[4];

double ADC_factor[4];

volatile bool ADC_stopped;

void delayMicroseconds(uint32_t microseconds) {
	while (microseconds--) {
		// 180 NOP's

		// remove 6 NOP's to compensate looping costs

		// __ASM volatile ("NOP");
		// __ASM volatile ("NOP");
		// __ASM volatile ("NOP");
		// __ASM volatile ("NOP");
		// __ASM volatile ("NOP");
		// __ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
		__ASM volatile ("NOP");
	}
}

#define ADC_SPI_SELECT() RESET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin)
#define ADC_SPI_DESELECT() delayMicroseconds(1); SET_PIN(ADC_CS_GPIO_Port, ADC_CS_Pin)
//#define ADC_SPI_SELECT() (void)0
//#define ADC_SPI_DESELECT() (void)0

#define SPI_IS_BUSY(SPIx)  (((SPIx)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || ((SPIx)->SR & SPI_SR_BSY))
#define SPI_WAIT(SPIx)     while (SPI_IS_BUSY(SPIx))
#define SPI1_DR_8bit(SPIx) (*(__IO uint8_t *)((uint32_t)&(SPIx->DR)))

uint8_t ADC_SPI_TransferLL(uint8_t data){
	SPI_WAIT(SPI3);
	SPI1_DR_8bit(SPI3) = data;
	SPI_WAIT(SPI3);
	return SPI1_DR_8bit(SPI3);
}

void ADC_WriteReg(uint8_t reg, uint8_t val) {
	uint8_t cmd[2];
	cmd[0] = ( reg & 0b0001'1111 ) | 0b0100'0000;
	cmd[1] = 1; // write 1 register

	ADC_SPI_SELECT();
	HAL_SPI_Transmit(hspiADC, cmd, 2, 100);
	delayMicroseconds(2);
	HAL_SPI_Transmit(hspiADC, &val, 1, 100);
	ADC_SPI_DESELECT();
}

uint8_t ADC_ReadReg(uint8_t reg) {
	uint8_t cmd[2];
	cmd[0] = ( reg & 0b0001'1111 ) | 0b0010'0000;
	cmd[1] = 1; // read 1 register

	uint8_t value;

	ADC_SPI_SELECT();
	HAL_SPI_Transmit(hspiADC, cmd, 2, 100);
	delayMicroseconds(2);
	HAL_SPI_Receive(hspiADC, &value, 1, 100);
	ADC_SPI_DESELECT();

	return value;
}

void ADC_SendSimpleCmd(uint8_t cmd) {
	ADC_SPI_SELECT();
	HAL_SPI_Transmit(hspiADC, &cmd, 1, 100);
	ADC_SPI_DESELECT();
}

void ADC_SwReset() {
	ADC_SendSimpleCmd(0x06);
	delayMicroseconds(10);
}

void ADC_WakeUp() {
	ADC_SendSimpleCmd(0x02);
}

void ADC_StopReadContinuous() {
	ADC_SendSimpleCmd(0x11);
}

void ADC_OffsetCalc() {
	ADC_SendSimpleCmd(0x1A);
	HAL_Delay(153);
}

void ADC_Start() {
	ADC_SendSimpleCmd(0x08);
}

void ADC_Stop() {
	ADC_SendSimpleCmd(0x0A);
}

void ADC_Pin_SetState(int pinIndex, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool pinState) {
	static int ADC_Pin_CurrentState[16] = {
		// undefined
		-1, -1, -1, -1,
		-1, -1, -1, -1,
		-1, -1, -1, -1,
		-1, -1, -1, -1
	};

	int state = pinState ? 1 : 0;

	if (ADC_Pin_CurrentState[pinIndex] != state) {
		if (state) {
			SET_PIN(GPIOx, GPIO_Pin);
		} else {
			RESET_PIN(GPIOx, GPIO_Pin);
		}
		ADC_Pin_CurrentState[pinIndex] = state;
	}
}

void ADC_Relay_SetState(
	int relayIndex,
	GPIO_TypeDef* GPIOx_Set, uint16_t GPIO_Pin_Set,
	GPIO_TypeDef* GPIOx_Reset, uint16_t GPIO_Pin_Reset,
	bool relayState
) {
	static int ADC_Relay_CurrentState[4] = {
		// undefined
		-1, -1, -1, -1
	};

	int state = relayState ? 1 : 0;

	if (ADC_Relay_CurrentState[relayIndex] != state) {
		if (state) {
			SET_PIN(GPIOx_Set, GPIO_Pin_Set);
			HAL_Delay(40);
			RESET_PIN(GPIOx_Set, GPIO_Pin_Set);
		} else {
			SET_PIN(GPIOx_Reset, GPIO_Pin_Reset);
			HAL_Delay(40);
			RESET_PIN(GPIOx_Reset, GPIO_Pin_Reset);
		}

		ADC_Relay_CurrentState[relayIndex] = state;
	}
}

void ADC_UpdateChannel(uint8_t channelIndex, uint8_t mode, uint8_t range) {
	uint8_t pga = 0b0001'0000;

	if (channelIndex == 2 || channelIndex == 3) {
		if (mode == SOURCE_MODE_CURRENT) {
			if (range == 0) {
				pga = 0b0010'0000; // x2
			} else if (range == 1) {
				pga = 0b0100'0000; // x4
			} else {
				pga = 0b0110'0000; // x12
			}
		}
	}

	ADC_pga[channelIndex] = pga;

	if (mode == SOURCE_MODE_OPEN) {
		ADC_WriteReg(0x05 + channelIndex, 0b1000'0000);
	} else {
		ADC_WriteReg(0x05 + channelIndex, 0b0000'0000 | pga);
	}

	if (channelIndex == 0) {
		ADC_Pin_SetState(0, USEL1_1_GPIO_Port, USEL1_1_Pin, mode == SOURCE_MODE_VOLTAGE && range == 0);
		ADC_Pin_SetState(1, USEL10_1_GPIO_Port, USEL10_1_Pin, mode == SOURCE_MODE_VOLTAGE && range == 1);
		ADC_Pin_SetState(2, USEL100_1_GPIO_Port, USEL100_1_Pin, mode == SOURCE_MODE_VOLTAGE && range == 2);
		ADC_Pin_SetState(3, ISEL_1_GPIO_Port, ISEL_1_Pin, mode == SOURCE_MODE_CURRENT);
	} else if (channelIndex == 1) {
		ADC_Pin_SetState(4, USEL1_2_GPIO_Port, USEL1_2_Pin, mode == SOURCE_MODE_VOLTAGE && range == 0);
		ADC_Pin_SetState(5, USEL10_2_GPIO_Port, USEL10_2_Pin, mode == SOURCE_MODE_VOLTAGE && range == 1);
		ADC_Pin_SetState(6, USEL100_2_GPIO_Port, USEL100_2_Pin, mode == SOURCE_MODE_VOLTAGE && range == 2);
		ADC_Pin_SetState(7, ISEL_2_GPIO_Port, ISEL_2_Pin, mode == SOURCE_MODE_CURRENT);
	} else if (channelIndex == 2) {
		// Current 24mA : ISEL_LOW, ISEL10A, ISEL
		// Current 1A   : ISEL_MID, ISEL10A, ISEL
		// Current 10A  : ISEL_MID

		ADC_Pin_SetState(8, USEL1_3_GPIO_Port, USEL1_3_Pin, mode == SOURCE_MODE_VOLTAGE && range == 0);
		ADC_Pin_SetState(9, USEL10_3_GPIO_Port, USEL10_3_Pin, mode == SOURCE_MODE_VOLTAGE && range == 1);
		ADC_Pin_SetState(10, ISEL_LOW_3_GPIO_Port, ISEL_LOW_3_Pin, mode == SOURCE_MODE_CURRENT && range == 0);
		ADC_Pin_SetState(11, ISEL_MID_3_GPIO_Port, ISEL_MID_3_Pin, mode == SOURCE_MODE_CURRENT && (range == 1 || range == 2));

		ADC_Relay_SetState(0, ISEL10_S_3_GPIO_Port, ISEL10_S_3_Pin, ISEL10_R_3_GPIO_Port, ISEL10_R_3_Pin,
			mode == SOURCE_MODE_VOLTAGE || (mode == SOURCE_MODE_CURRENT && (range == 0 || range == 1)));

		ADC_Relay_SetState(1, ISEL_S_3_GPIO_Port, ISEL_S_3_Pin, ISEL_R_3_GPIO_Port, ISEL_R_3_Pin,
			mode == SOURCE_MODE_CURRENT && (range == 0 || range == 1));
	} else {
		// Current 24mA : ISEL_LOW, ISEL10A, ISEL
		// Current 1A   : ISEL_MID, ISEL10A, ISEL
		// Current 10A  : ISEL_MID

		ADC_Pin_SetState(12, USEL1_4_GPIO_Port, USEL1_4_Pin, mode == SOURCE_MODE_VOLTAGE && range == 0);
		ADC_Pin_SetState(13, USEL10_4_GPIO_Port, USEL10_4_Pin, mode == SOURCE_MODE_VOLTAGE && range == 1);
		ADC_Pin_SetState(14, ISEL_LOW_4_GPIO_Port, ISEL_LOW_4_Pin, mode == SOURCE_MODE_CURRENT && range == 0);
		ADC_Pin_SetState(15, ISEL_MID_4_GPIO_Port, ISEL_MID_4_Pin, mode == SOURCE_MODE_CURRENT && (range == 1 || range == 2));

		ADC_Relay_SetState(2, ISEL10_S_4_GPIO_Port, ISEL10_S_4_Pin, ISEL10_R_4_GPIO_Port, ISEL10_R_4_Pin,
			mode == SOURCE_MODE_VOLTAGE || (mode == SOURCE_MODE_CURRENT && (range == 0 || range == 1)));

		ADC_Relay_SetState(3, ISEL_S_4_GPIO_Port, ISEL_S_4_Pin, ISEL_R_4_GPIO_Port, ISEL_R_4_Pin,
			mode == SOURCE_MODE_CURRENT && (range == 0 || range == 1));
	}

	//
	double f;

	if (channelIndex < 2) {
		if (mode == SOURCE_MODE_VOLTAGE) {
			if (range == 0) {
				f = 2.4; // +/- 2.4 V
			} else if (range == 1) {
				f = 48.0; // +/- 48 V
			} else {
				f = 240.0; // +/- 240 V
			}
		} else if (mode == SOURCE_MODE_CURRENT) {
			f = 0.048; // +/- 48 mV ( = 2.4 V / 50 Ohm)
		} else {
			f = 0;
		}
	} else {
		if (mode == SOURCE_MODE_VOLTAGE) {
			if (range == 0) {
				f = 2.4; // +/- 2.4 V
			} else {
				f = 12.0; // +/- 12 V
			}
		} else if (mode == SOURCE_MODE_CURRENT) {
			if (range == 0) {
				f = 0.024; // +/- 24 mA (rsense is 50 ohm, PGA is 2)
			} else if (range == 1) {
				f = 1.2; // +/- 1 A (rsense 0.5 ohm, PGA is 4)
			} else {
				f = 20.0; // +/- 10 A (rsense is 0.01 ohm, PGA is 12 => 2.4 / 0.01 / 16 = 20 A)
			}
		} else {
			f = 0;
		}
	}

	ADC_factor[channelIndex] = f;

	g_adcMovingAverage[channelIndex].reset();
}

void ADC_Setup() {
	ADC_SwReset();
	ADC_WakeUp();
	ADC_StopReadContinuous();

	ADC_WriteReg(0x01, 0b1111'0110); // CONFIG1  24-bit 16 KSPS
	ADC_WriteReg(0x02, 0b1111'0101); // CONFIG2
	ADC_WriteReg(0x03, 0b1100'0000); // CONFIG3

	ADC_WriteReg(0x04, 0b0000'0000); // FAULT

	for (uint8_t channelIndex = 0; channelIndex < 4; channelIndex++) {
		ADC_UpdateChannel(
			channelIndex,
			currentState.ain[channelIndex].mode,
			currentState.ain[channelIndex].range
		);
	}

	ADC_OffsetCalc();

	ADC_Start();
}

void ADC_SetParams(SetParams &newState) {
	uint8_t ADC_pga_before[4] = {
		ADC_pga[0],
		ADC_pga[1],
		ADC_pga[2],
		ADC_pga[3]
	};

	for (uint8_t channelIndex = 0; channelIndex < 4; channelIndex++) {
		if (
			newState.ain[channelIndex].mode != currentState.ain[channelIndex].mode ||
			newState.ain[channelIndex].range != currentState.ain[channelIndex].range
		) {
			if (!ADC_stopped) {
				ADC_stopped = true;
				ADC_Stop();
				HAL_Delay(2);
			}

			ADC_UpdateChannel(
				channelIndex,
				newState.ain[channelIndex].mode,
				newState.ain[channelIndex].range
			);
		}
	}

	if (
		ADC_pga_before[0] != ADC_pga[0] ||
		ADC_pga_before[1] != ADC_pga[1] ||
		ADC_pga_before[2] != ADC_pga[2] ||
		ADC_pga_before[3] != ADC_pga[3]
	) {
		ADC_OffsetCalc();
	}

	if (ADC_stopped) {
		HAL_Delay(2);
		ADC_Start();
		ADC_stopped = false;
	}
}

inline void ADC_Measure() {
	ADC_SPI_SELECT();

	ADC_SPI_TransferLL(0x12);

	uint8_t rx[15];
	for (int i = 0; i < 15; i++) {
		rx[i] = ADC_SPI_TransferLL(0);
	}

	ADC_SPI_DESELECT();

	// status
	// uint32_t status = (rx[0] << 16) | (rx[1] << 8) | rx[2];

	int32_t ch[4];

	ch[0] = ((int32_t)((rx[ 3] << 24) + (rx[ 4] << 16) +(rx[ 5] << 8))) >> 8;
	ch[1] = ((int32_t)((rx[ 6] << 24) + (rx[ 7] << 16) +(rx[ 8] << 8))) >> 8;
	ch[2] = ((int32_t)((rx[ 9] << 24) + (rx[10] << 16) +(rx[11] << 8))) >> 8;
	ch[3] = ((int32_t)((rx[12] << 24) + (rx[13] << 16) +(rx[14] << 8))) >> 8;

	for (uint8_t channelIndex = 0; channelIndex < 4; channelIndex++) {
		double f = ADC_factor[channelIndex];
		float value = (float)(f * ch[channelIndex] / (1 << 23));
		g_adcMovingAverage[channelIndex](value);
		ADC_samples[channelIndex] = g_adcMovingAverage[channelIndex];
	}

#if DEBUG_VARS
	if (++g_debugVar1 == ADC_MOVING_AVERAGE_NUM_SAMPLES) {
		g_debugVar1 = 0;
		uint32_t tickCount = HAL_GetTick();
		g_debugVar2 = tickCount - g_debugVar3;
		g_debugVar3 = tickCount;
	}
#endif
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == ADC_DRDY_Pin) {
		if (!ADC_stopped) {
			ADC_Measure();
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
//        DAC7760
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define DAC7760_CONTROL_REGISTER 0x55
#define DAC7760_CONFIGURATION_REGISTER 0x57
#define DAC7760_DATA_REGISTER 0x01

void DAC_SpiWrite(int i, uint8_t b0, uint8_t b1, uint8_t b2) {
    uint8_t buf[3] = { b0, b1, b2 };

    HAL_SPI_Transmit(hspiDAC, buf, 3, 100);

    if (i == 0) {
    	SET_PIN(DAC_CS_1_GPIO_Port, DAC_CS_1_Pin);
    	RESET_PIN(DAC_CS_1_GPIO_Port, DAC_CS_1_Pin);
    } else {
        SET_PIN(DAC_CS_2_GPIO_Port, DAC_CS_2_Pin);
        RESET_PIN(DAC_CS_2_GPIO_Port, DAC_CS_2_Pin);
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
////////////////////////////////////////////////////////////////////////////////
//
//        DAC7563
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

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
////////////////////////////////////////////////////////////////////////////////
//
//        PWM outputs
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void PWM_Setup() {
	SET_PIN(DOUT_EN_GPIO_Port, DOUT_EN_Pin);
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

uint8_t g_writerBuffer[48 * 1024];
dlog_file::Writer g_writer(g_writerBuffer, sizeof(g_writerBuffer));

uint8_t g_fileWriteBuffer[24 * 1024];
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

			// invalid samples
			memset(g_fileWriteBuffer + g_fileWriteBufferIndex, 0, nInvalid);
			g_fileWriteBufferIndex += nInvalid;
			g_lastSavedBufferIndex += nInvalid;

			n -= nInvalid;
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
		UINT bw;
		FRESULT result = f_write(&g_file, g_fileWriteBuffer, btw, &bw);

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
    RESET_PIN(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == hspiADC) {
		return;
	}

	SET_PIN(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin);
	transferCompleted = 1;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == hspiADC) {
		return;
	}

    SET_PIN(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin);
	transferCompleted = 2;
}

int waitTransferCompletion() {
	uint32_t startTick = HAL_GetTick();
	while (!transferCompleted) {
		if (HAL_GetTick() - startTick > 500) {
			HAL_SPI_Abort(hspiMaster);
			SET_PIN(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin);
			return 2;
		}

		DLOG_LoopWrite();
	}
	return transferCompleted;
}

////////////////////////////////////////////////////////////////////////////////

void Command_GetInfo(Request &request, Response &response) {
	response.getInfo.firmwareMajorVersion = FIRMWARE_VERSION_MAJOR;
	response.getInfo.firmwareMinorVersion = FIRMWARE_VERSION_MINOR;
	response.getInfo.idw0 = HAL_GetUIDw0();
	response.getInfo.idw1 = HAL_GetUIDw1();
	response.getInfo.idw2 = HAL_GetUIDw2();
	response.getInfo.afeVersion = ((READ_PIN(AFE_ID1_GPIO_Port, AFE_ID1_Pin) << 1) | READ_PIN(AFE_ID0_GPIO_Port, AFE_ID0_Pin)) + 1;
}

void Command_GetState(Request &request, Response &response) {
	g_fatTime = request.getState.fatTime;

	response.getState.flags = 0;
	if (BSP_PlatformIsDetected() == SD_PRESENT) {
		response.getState.flags |= GET_STATE_COMMAND_FLAG_SD_CARD_PRESENT;
	}

	response.getState.dinStates = readDataInputs();

	response.getState.ainValues[0] = ADC_samples[0];
	response.getState.ainValues[1] = ADC_samples[1];
	response.getState.ainValues[2] = ADC_samples[2];
	response.getState.ainValues[3] = ADC_samples[3];

	memcpy(&response.getState.dlogState, &dlogState, sizeof(DlogState));
}

void Command_SetParams(Request &request, Response &response) {
	Din_SetParams(request.setParams);
	Dout_SetParams(request.setParams);
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
	resetState();

	Din_Setup();
	Dout_Setup();
	ADC_Setup();
	DAC_Setup(0);
    DAC_Setup(1);
    DACDual_Setup();
    PWM_Setup();
}

extern "C" void loop() {
	beginTransfer();

    auto transferResult = waitTransferCompletion();

    Request &request = *(Request *)input;
    Response &response = *(Response *)output;

    if (transferResult == 1) {
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
	    	response.command = COMMAND_NONE;
		}
    } else {
    	response.command = COMMAND_NONE;

    	while (!READ_PIN(DIB_NSS_GPIO_Port, DIB_NSS_Pin)) {
    	}

		HAL_SPI_DeInit(hspiMaster);
		SPI4_Init();
    }
}
