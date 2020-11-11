#include <math.h>
#include <memory.h>

#include "main.h"

////////////////////////////////////////////////////////////////////////////////

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim4;

void TIM4_Init(void);

////////////////////////////////////////////////////////////////////////////////

#define BUFFER_SIZE 1024

uint8_t outputBuffers[2][BUFFER_SIZE];
uint8_t inputBuffers[2][BUFFER_SIZE];

uint8_t *output;
uint8_t *input;

volatile int transferCompleted;

enum SourceMode {
	SOURCE_MODE_CURRENT,
	SOURCE_MODE_VOLTAGE,
	SOURCE_MODE_OPEN
};

////////////////////////////////////////////////////////////////////////////////

typedef struct {
    uint8_t dinRanges;
    uint8_t dinSpeeds;

    uint8_t doutStates;

    struct {
        uint8_t mode;
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
} FromMasterToSlave;

typedef struct {
    uint8_t dinStates;
    uint16_t ainValues[4];
} FromSlaveToMaster;

FromMasterToSlave currentState;

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

void Din_Setup() {
	for (int i = 0; i < 8; i++) {
		HAL_GPIO_WritePin(dinRangePorts[i], dinRangePins[i], GPIO_PIN_RESET);
	}

	for (int i = 0; i < 2; i++) {
		HAL_GPIO_WritePin(dinSpeedPorts[i], dinSpeedPins[i], GPIO_PIN_RESET);
	}
}

void Din_Loop(FromMasterToSlave *newState) {
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
}

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

SPI_TypeDef *ADC_SPI_Instance;
volatile uint32_t *ADC_SPI_CR1;
volatile uint32_t *ADC_SPI_CR2;

DMA_HandleTypeDef *ADC_DMA_RX;
DMA_HandleTypeDef *ADC_DMA_TX;

volatile uint32_t *ADC_DMA_RX_CR;
volatile uint32_t *ADC_DMA_TX_CR;

volatile uint32_t *ADC_DMA_RX_LISR;
volatile uint32_t *ADC_DMA_TX_LISR;

volatile uint32_t *ADC_DMA_RX_LIFCR;
volatile uint32_t *ADC_DMA_TX_LIFCR;

uint32_t ADC_DMA_RX_LIFCR_clearTransferComplete;
uint32_t ADC_DMA_TX_LIFCR_clearTransferComplete;

uint32_t ADC_DMA_RX_LIFCR_value;
uint32_t ADC_DMA_TX_LIFCR_value;

volatile int ADC_selectedBuffer;
uint16_t *ADC_samples;
uint16_t *ADC_numSamples;

volatile int ADC_switchBuffer;

void ADC_SpiRxCallback();
void ADC_SpiTxCallback();

void ADC_DMA_Config() {
    // RX DMA config
    {
	    ADC_DMA_RX = ADC_hspi->hdmarx;
	    ADC_DMA_RX_CR = &ADC_DMA_RX->Instance->CR;
	    ADC_DMA_RX_LISR = &((DMA_TypeDef *)ADC_DMA_RX->StreamBaseAddress)->LISR;
	    ADC_DMA_RX_LIFCR = &((DMA_TypeDef *)ADC_DMA_RX->StreamBaseAddress)->LIFCR;
	    ADC_DMA_RX_LIFCR_clearTransferComplete = DMA_FLAG_TCIF0_4 << ADC_DMA_RX->StreamIndex;
	    ADC_DMA_RX_LIFCR_value = 0x3FU << ADC_DMA_RX->StreamIndex;

	    ADC_DMA_RX->XferCpltCallback = NULL;

		// Clear DBM bit
		ADC_DMA_RX->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

		// Configure DMA Stream data length
		ADC_DMA_RX->Instance->NDTR = 2;

		// Configure DMA Stream source address
		ADC_DMA_RX->Instance->PAR = (uint32_t)&ADC_SPI_Instance->DR;

		// Configure DMA Stream destination address
		ADC_DMA_RX->Instance->M0AR = (uint32_t)adcOut;
    }

    // TX DMA config
    {
        ADC_DMA_TX = ADC_hspi->hdmatx;
        ADC_DMA_TX_CR = &ADC_DMA_TX->Instance->CR;
        ADC_DMA_TX_LISR = &((DMA_TypeDef *)ADC_DMA_TX->StreamBaseAddress)->LISR;
        ADC_DMA_TX_LIFCR = &((DMA_TypeDef *)ADC_DMA_TX->StreamBaseAddress)->LIFCR;
        ADC_DMA_TX_LIFCR_clearTransferComplete = DMA_FLAG_TCIF0_4 << ADC_DMA_TX->StreamIndex;
        ADC_DMA_TX_LIFCR_value = 0x3FU << ADC_DMA_TX->StreamIndex;

        ADC_DMA_TX->XferCpltCallback = ADC_SpiTxCallback;

    	// Clear DBM bit
    	ADC_DMA_TX->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

    	// Configure DMA Stream data length
    	ADC_DMA_TX->Instance->NDTR = 2;

    	// Configure DMA Stream destination address
    	ADC_DMA_TX->Instance->PAR = (uint32_t)&ADC_SPI_Instance->DR;

    	// Configure DMA Stream source address
    	ADC_DMA_TX->Instance->M0AR = (uint32_t)adcIn;
    }
}

__STATIC_FORCEINLINE void ADC_Transfer() {
    // DMA RX
    {
		// Clear all interrupt flags at correct offset within the register
		*ADC_DMA_RX_LIFCR = ADC_DMA_RX_LIFCR_value;

	    // Enable Common interrupts and Peripheral
		*ADC_DMA_RX_CR |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_SxCR_EN;
	}

    // DMA TX
	{
		// Clear all interrupt flags at correct offset within the register
		*ADC_DMA_TX_LIFCR = ADC_DMA_TX_LIFCR_value;

	    // Enable Common interrupts and Peripheral
		*ADC_DMA_TX_CR |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_SxCR_EN;
	}

	// RESET ADC CS
	ADC_CS_GPIO_Port->BSRR = (uint32_t)ADC_CS_Pin << 16U;

	// Enable the SPI Error Interrupt Bit
	// Enable Rx DMA Request
	// Enable Tx DMA Request
	*ADC_SPI_CR2 |= SPI_IT_ERR | SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
}

__STATIC_FORCEINLINE void switchBuffer() {
	ADC_numSamples = (uint16_t *)(output + 22);
	ADC_samples = ADC_numSamples + 1;
	*ADC_numSamples = 0;

	ADC_selectedBuffer ^= 1;
	output = &outputBuffers[ADC_selectedBuffer][0];
	input = &inputBuffers[ADC_selectedBuffer][0];
}


__STATIC_FORCEINLINE void ADC_onNewSample(uint16_t sample) {
	if (ADC_switchBuffer) {
		switchBuffer();
		ADC_switchBuffer = 0;
	}

	uint16_t n = *ADC_numSamples;
	if (n < 500) {
		ADC_samples[n] = sample;
	}
	*ADC_numSamples = n + 1;
}

void ADC_SpiRxCallback() {
#if defined(SIMPLE_METHOD)
#else
	// Clear the transfer complete flag
	*ADC_DMA_RX_LIFCR = ADC_DMA_RX_LIFCR_clearTransferComplete;

	if (ADC_switchBuffer) {
		switchBuffer();
		ADC_switchBuffer = 0;
	}

	// SET ADC CS
	ADC_CS_GPIO_Port->BSRR = ADC_CS_Pin;

	uint16_t n = *ADC_numSamples;
	if (n < 500) {
		ADC_samples[n] = adcOut[1];
	}
	*ADC_numSamples = n + 1;

	adcIn[0] = 0xC000 | ((adcIn[0] + 0x0400) & 0x0F00);

	ADC_Transfer();
#endif
}


void ADC_SpiTxCallback() {
#if defined(SIMPLE_METHOD)
#else
	// Clear the transfer complete flag */
	*ADC_DMA_TX_LIFCR = ADC_DMA_TX_LIFCR_clearTransferComplete;
#endif
}

GPIO_TypeDef* voltSwPorts[4] = { VOLT_SW0_GPIO_Port, VOLT_SW1_GPIO_Port, VOLT_SW2_GPIO_Port, VOLT_SW3_GPIO_Port };
uint16_t      voltSwPins [4] = { VOLT_SW0_Pin,       VOLT_SW1_Pin,       VOLT_SW2_Pin,       VOLT_SW3_Pin       };

GPIO_TypeDef* currSwPorts[4] = { CURR_SW0_GPIO_Port, CURR_SW1_GPIO_Port, CURR_SW2_GPIO_Port, CURR_SW3_GPIO_Port };
uint16_t      currSwPins [4] = { CURR_SW0_Pin,       CURR_SW1_Pin,       CURR_SW2_Pin,       CURR_SW3_Pin       };

GPIO_TypeDef* tempSwPorts[2] = { TEMP_SW_1_GPIO_Port, TEMP_SW_2_GPIO_Port };
uint16_t      tempSwPins [2] = { TEMP_SW_1_Pin,       TEMP_SW_2_Pin,      };

void ADC_Setup() {
	for (int i = 0; i < 4; i++) {
		HAL_GPIO_WritePin(voltSwPorts[i], voltSwPins[i], GPIO_PIN_SET);
		HAL_GPIO_WritePin(currSwPorts[i], currSwPins[i], GPIO_PIN_RESET);
	}

#if defined(SIMPLE_METHOD)
	ADC_CS_GPIO_Port->BSRR = ADC_CS_Pin; // SET ADC CS
#else
	#if defined(INT2_METHOD)
		switchBuffer();

		adcIn[0] = 0xC000;
		adcIn[2] = 0xC000;

		ADC_CS_GPIO_Port->BSRR = (uint32_t)ADC_CS_Pin << 16U; // RESET ADC CS

		HAL_SPI_TransmitReceive_DMA(ADC_hspi, (uint8_t *)adcIn, (uint8_t *)adcOut, 4);
	#else
		ADC_hspi->hdmarx->Init.Mode = DMA_NORMAL;
		HAL_DMA_Init(ADC_hspi->hdmarx);
		ADC_hspi->hdmatx->Init.Mode = DMA_NORMAL;
		HAL_DMA_Init(ADC_hspi->hdmatx);

		// ADC SPI config
		ADC_SPI_Instance = ADC_hspi->Instance;
		ADC_SPI_CR1 = &ADC_SPI_Instance->CR1;
		ADC_SPI_CR2 = &ADC_SPI_Instance->CR2;

		ADC_DMA_Config();

		// Enable SPI peripheral
		*ADC_SPI_CR1 |= SPI_CR1_SPE;

		switchBuffer();

		adcIn[0] = 0xC400;

		#if defined(INT_METHOD)
			ADC_Transfer();
		#endif
	#endif
#endif
}

void ADC_Loop(FromMasterToSlave *newState) {
#if defined(SIMPLE_METHOD)
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

	ADC_numSamples = (uint16_t *)(output + 22);
	ADC_samples = ADC_numSamples + 1;
	*ADC_numSamples = 0;

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
		(*ADC_numSamples)++;
	}

#else
	#if defined(INT_METHOD) || defined(INT2_METHOD)
		ADC_switchBuffer = 1;
		while (ADC_switchBuffer);
	#else
		switchBuffer();
	#endif
#endif
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

void DAC_Loop(int i, FromMasterToSlave *newState) {
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

void DACDual_Loop(int i, FromMasterToSlave *newState) {
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

void PWM_Setup() {
	HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_SET);
}

void PWM_Loop(int i, FromMasterToSlave *newState) {
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

void beginTransfer() {
	FromSlaveToMaster *slaveToMaster = (FromSlaveToMaster *)output;

	slaveToMaster->dinStates = readDataInputs();

	uint16_t *ADC_samples = (uint16_t *)(output + 24);

	slaveToMaster->ainValues[0] = ADC_samples[0];
	slaveToMaster->ainValues[1] = ADC_samples[1];
	slaveToMaster->ainValues[2] = ADC_samples[2];
	slaveToMaster->ainValues[3] = ADC_samples[3];

    transferCompleted = 0;
    HAL_SPI_TransmitReceive_DMA(&hspi4, output, input, sizeof(FromMasterToSlave));
    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_RESET);
}

void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi) {
#if defined(INT2_METHOD)
	if (hspi == ADC_hspi) {
		ADC_onNewSample(adcOut[1]);
	}
#endif
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
#if defined(SIMPLE_METHOD)
	if (hspi == ADC_hspi) {
		return;
	}
#endif

#if defined(INT2_METHOD)
	if (hspi == ADC_hspi) {
		ADC_onNewSample(adcOut[3]);
		return;
	}
#endif

    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_SET);
	transferCompleted = 1;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
#if defined(SIMPLE_METHOD)
	if (hspi == ADC_hspi) {
		return;
	}
#endif

#if defined(INT2_METHOD)
	if (hspi == ADC_hspi) {
		return;
	}
#endif

    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_SET);
	transferCompleted = 2;
}

////////////////////////////////////////////////////////////////////////////////

void setup() {
	resetState();

    slaveSynchro();

	ADC_selectedBuffer = 0;
	output = &outputBuffers[0][0];
	input = &inputBuffers[0][0];

    beginTransfer();

    ADC_selectedBuffer = 1;
	output = &outputBuffers[1][0];
	input = &inputBuffers[1][0];

    //
	Din_Setup();

	ADC_Setup();

	DAC_Setup(0);
    DAC_Setup(1);

    DACDual_Setup();

    PWM_Setup();
    //
}

void noDmaMethod() {
	// Speed: 292 KSPS on 180 Mhz
	volatile uint16_t *pDR = (volatile uint16_t *)&SPI2->DR;

	uint16_t *start = ADC_samples + 0;
	uint16_t *end = ADC_samples + 500;

	uint16_t *p = start;

	while (!transferCompleted) {
		ADC_CS_GPIO_Port->BSRR = (uint32_t)ADC_CS_Pin << 16U; // RESET ADC CS

		*pDR = 0xC000; while (!(SPI2->SR & SPI_SR_RXNE)); *pDR;
		*pDR = 0x0000; while (!(SPI2->SR & SPI_SR_RXNE)); *p++ = *pDR;

		ADC_CS_GPIO_Port->BSRR = ADC_CS_Pin; // SET ADC CS
		ADC_CS_GPIO_Port->BSRR = (uint32_t)ADC_CS_Pin << 16U; // RESET ADC CS

		*pDR = 0xC400; while (!(SPI2->SR & SPI_SR_RXNE)); *pDR;
		*pDR = 0x0000; while (!(SPI2->SR & SPI_SR_RXNE)); *p++ = *pDR;

		ADC_CS_GPIO_Port->BSRR = ADC_CS_Pin; // SET ADC CS
		ADC_CS_GPIO_Port->BSRR = (uint32_t)ADC_CS_Pin << 16U; // RESET ADC CS

		*pDR = 0xC800; while (!(SPI2->SR & SPI_SR_RXNE)); *pDR;
		*pDR = 0x0000; while (!(SPI2->SR & SPI_SR_RXNE)); *p++ = *pDR;

		ADC_CS_GPIO_Port->BSRR = ADC_CS_Pin; // SET ADC CS
		ADC_CS_GPIO_Port->BSRR = (uint32_t)ADC_CS_Pin << 16U; // RESET ADC CS

		*pDR = 0xCC00; while (!(SPI2->SR & SPI_SR_RXNE)); *pDR;
		*pDR = 0x0000; while (!(SPI2->SR & SPI_SR_RXNE)); *p++ = *pDR;

		ADC_CS_GPIO_Port->BSRR = ADC_CS_Pin; // SET ADC CS

		if (p == end) {
			p = start;
		}
	}

	*ADC_numSamples = p - ADC_samples;
}

void loop() {
#if defined(NO_DMA_METHOD)
	noDmaMethod();
#endif

#if defined(INT_METHOD) || defined(INT2_METHOD) || defined(SIMPLE_METHOD)
	while (!transferCompleted) {
	}
#endif

	if (transferCompleted == 1) {
		FromMasterToSlave *newState = (FromMasterToSlave *)input;

		if (newState->doutStates != currentState.doutStates) {
			updateDoutStates(newState->doutStates);
		}

		//

		Din_Loop(newState);

		ADC_Loop(newState);

		DAC_Loop(0, newState);
		DAC_Loop(1, newState);

		DACDual_Loop(0, newState);
		DACDual_Loop(1, newState);

		PWM_Loop(0, newState);
		// PWM_Loop(1, newState);

		memcpy(&currentState, newState, sizeof(FromMasterToSlave));
	}

	beginTransfer();
}

