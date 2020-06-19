#include <math.h>

#include "main.h"

////////////////////////////////////////////////////////////////////////////////

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim4;

////////////////////////////////////////////////////////////////////////////////

#define BUFFER_SIZE 1024

uint8_t outputBuffers[2][BUFFER_SIZE];
uint8_t inputBuffers[2][BUFFER_SIZE];

uint8_t *output;
uint8_t *input;

volatile int transferCompleted;

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

        if (HAL_SPI_TransmitReceive_DMA(&hspi4, (uint8_t *)&txBuffer, (uint8_t *)&rxBuffer, sizeof(rxBuffer)) != HAL_OK) {
        	transferCompleted = 0;
        	continue;
        }

    	while (!transferCompleted);
        transferCompleted = 0;

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

static GPIO_TypeDef *doutPort[8] = { DOUT0_GPIO_Port, DOUT1_GPIO_Port, DOUT2_GPIO_Port, DOUT3_GPIO_Port, DOUT4_GPIO_Port, DOUT5_GPIO_Port, DOUT6_GPIO_Port, DOUT7_GPIO_Port };
static uint16_t doutPin[8] = { DOUT0_Pin, DOUT1_Pin, DOUT2_Pin, DOUT3_Pin, DOUT4_Pin, DOUT5_Pin, DOUT6_Pin, DOUT7_Pin };

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
// ADC8674

uint8_t adcIn[4] = { 0x00, 0x00, 0x00, 0x00 };
uint8_t adcOut[4] = { 0x00, 0x00, 0x00, 0x00 };

SPI_HandleTypeDef *ADC_hspi = &hspi2;

SPI_TypeDef *ADC_SPI_Instance;
volatile uint32_t *ADC_SPI_CR1;
volatile uint32_t *ADC_SPI_CR2;

DMA_HandleTypeDef *ADC_DMA_RX;
DMA_HandleTypeDef *ADC_DMA_TX;

volatile uint32_t *ADC_DMA_RX_CR;
volatile uint32_t *ADC_DMA_TX_CR;

volatile uint32_t *ADC_DMA_RX_ISR;
volatile uint32_t *ADC_DMA_TX_ISR;

volatile uint32_t *ADC_DMA_RX_IFCR;
volatile uint32_t *ADC_DMA_TX_IFCR;

uint32_t ADC_DMA_RX_IFCR_clearTransferComplete;
uint32_t ADC_DMA_TX_IFCR_clearTransferComplete;

uint32_t ADC_DMA_RX_IFCR_value;
uint32_t ADC_DMA_TX_IFCR_value;

volatile int ADC_selectedBuffer;
volatile uint16_t *ADC_samples;
volatile uint16_t *ADC_numSamples;

volatile int ADC_switchBuffer;

void ADC_SpiRxCallback();
void ADC_SpiTxCallback();

void ADC_DMA_Config() {
	typedef struct
	{
	  __IO uint32_t ISR;   // DMA interrupt status register
	  __IO uint32_t Reserved0;
	  __IO uint32_t IFCR;  // DMA interrupt flag clear register
	} DMA_Base_Registers;

    // RX DMA config
    {
	    ADC_DMA_RX = ADC_hspi->hdmarx;
	    ADC_DMA_RX_CR = &ADC_DMA_RX->Instance->CR;
	    ADC_DMA_RX_ISR = &((DMA_Base_Registers *)ADC_DMA_RX->StreamBaseAddress)->ISR;
	    ADC_DMA_RX_IFCR = &((DMA_Base_Registers *)ADC_DMA_RX->StreamBaseAddress)->IFCR;
	    ADC_DMA_RX_IFCR_clearTransferComplete = DMA_FLAG_TCIF0_4 << ADC_DMA_RX->StreamIndex;
	    ADC_DMA_RX_IFCR_value = 0x3FU << ADC_DMA_RX->StreamIndex;

	    ADC_DMA_RX->XferCpltCallback = NULL;

		// Clear DBM bit
		ADC_DMA_RX->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);
		// Configure DMA Stream data length
		ADC_DMA_RX->Instance->NDTR = 4;
		// Configure DMA Stream source address
		ADC_DMA_RX->Instance->PAR = (uint32_t)&ADC_SPI_Instance->DR;
		// Configure DMA Stream destination address
		ADC_DMA_RX->Instance->M0AR = (uint32_t)adcOut;
    }

    // TX DMA config
    {
        ADC_DMA_TX = ADC_hspi->hdmatx;
        ADC_DMA_TX_CR = &ADC_DMA_TX->Instance->CR;
        ADC_DMA_TX_ISR = &((DMA_Base_Registers *)ADC_DMA_TX->StreamBaseAddress)->ISR;
        ADC_DMA_TX_IFCR = &((DMA_Base_Registers *)ADC_DMA_TX->StreamBaseAddress)->IFCR;
        ADC_DMA_TX_IFCR_clearTransferComplete = DMA_FLAG_TCIF0_4 << ADC_DMA_TX->StreamIndex;
        ADC_DMA_TX_IFCR_value = 0x3FU << ADC_DMA_TX->StreamIndex;

        ADC_DMA_TX->XferCpltCallback = ADC_SpiTxCallback;

    	// Clear DBM bit
    	ADC_DMA_TX->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

    	// Configure DMA Stream data length
    	ADC_DMA_TX->Instance->NDTR = 4;

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
		*ADC_DMA_RX_IFCR = ADC_DMA_RX_IFCR_value;

	    // Enable Common interrupts and Peripheral
		*ADC_DMA_RX_CR |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_SxCR_EN;
	}

    // DMA TX
	{
		// Clear all interrupt flags at correct offset within the register
		*ADC_DMA_TX_IFCR = ADC_DMA_TX_IFCR_value;

	    // Enable Common interrupts and Peripheral
		*ADC_DMA_TX_CR |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_SxCR_EN;
	}

	// Enable SPI peripheral
	*ADC_SPI_CR1 |= SPI_CR1_SPE;

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


void ADC_SpiRxCallback() {
//	/* Clear the transfer complete flag */
//	*ADC_DMA_RX_IFCR = ADC_DMA_RX_IFCR_clearTransferComplete;
//
//	/* Disable the transfer complete interrupt */
//	*ADC_DMA_RX_CR &= ~(DMA_IT_TC);
//
//	/* Disable ERR interrupt */
//	*ADC_SPI_CR2 &= ~SPI_IT_ERR;
//
//	while (ADC_SPI_Instance->SR & SPI_FLAG_BSY);
//
//	/* Disable Rx/Tx DMA Request */
//	*ADC_SPI_CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
}


void ADC_SpiTxCallback() {
//	/* Clear the transfer complete flag */
//	*ADC_DMA_TX_IFCR = ADC_DMA_TX_IFCR_clearTransferComplete;
//
//	/* Disable the transfer complete interrupt */
//	*ADC_DMA_TX_CR &= ~(DMA_IT_TC);

	if (ADC_switchBuffer) {
		switchBuffer();
		ADC_switchBuffer = 0;
	}

	uint16_t n = *ADC_numSamples;
	if (n < 500) {
		ADC_samples[n] = (adcOut[2] << 8) | adcOut[3];
	}
	*ADC_numSamples = n + 1;

	// SET ADC CS
	ADC_CS_GPIO_Port->BSRR = ADC_CS_Pin;

	adcIn[0] = 0xC0 + ((adcIn[0] + 4) & 0x0F);
    ADC_Transfer();
}

void ADC_Setup() {
    HAL_GPIO_WritePin(VOLT_SW0_GPIO_Port, VOLT_SW0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(VOLT_SW1_GPIO_Port, VOLT_SW1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(VOLT_SW2_GPIO_Port, VOLT_SW2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(VOLT_SW3_GPIO_Port, VOLT_SW3_Pin, GPIO_PIN_SET);

    // ADC SPI config
    ADC_SPI_Instance = ADC_hspi->Instance;
    ADC_SPI_CR1 = &ADC_SPI_Instance->CR1;
    ADC_SPI_CR2 = &ADC_SPI_Instance->CR2;

    ADC_DMA_Config();

    switchBuffer();

    adcIn[0] = 0xC4;
    ADC_Transfer();
}

void ADC_Loop() {
	ADC_switchBuffer = 1;
	while (ADC_switchBuffer);
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
// DAC7760

void DAC1_SpiWrite(uint8_t b0, uint8_t b1, uint8_t b2) {
    uint8_t buf[3] = { b0, b1, b2 };
    HAL_SPI_Transmit(&hspi1, buf, 3, 100);
    HAL_GPIO_WritePin(DAC_CS_1_GPIO_Port, DAC_CS_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DAC_CS_1_GPIO_Port, DAC_CS_1_Pin, GPIO_PIN_RESET);
  }

void DAC1_Setup() {
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
// DAC7760

void DAC2_SpiWrite(uint8_t b0, uint8_t b1, uint8_t b2) {
    uint8_t buf[3] = { b0, b1, b2 };
    HAL_SPI_Transmit(&hspi1, buf, 3, 100);
    HAL_GPIO_WritePin(DAC_CS_2_GPIO_Port, DAC_CS_2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DAC_CS_2_GPIO_Port, DAC_CS_2_Pin, GPIO_PIN_RESET);
}

void DAC2_Setup() {
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

    HAL_SPI_TransmitReceive_DMA(&hspi4, output, input, BUFFER_SIZE);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	transferCompleted = 1;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	transferCompleted = 1;
}

////////////////////////////////////////////////////////////////////////////////

void setup() {
    slaveSynchro();

	ADC_selectedBuffer = 0;
	output = &outputBuffers[0][0];
	input = &inputBuffers[0][0];

    beginTransfer();
    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_SET);

    ADC_selectedBuffer = 1;
	output = &outputBuffers[1][0];
	input = &inputBuffers[1][0];

    //
	ADC_Setup();
    DACDual_Setup();
    DAC1_Setup();
    DAC2_Setup();
    PWM_Setup();
    //
}

void loop() {
	while (!transferCompleted);
    transferCompleted = 0;
    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_RESET);

    //

	updateOutputPinStates(input[1]); // TODO input???

    //

	ADC_Loop();
	DACDual_Loop();
	DAC1_Loop();
	DAC2_Loop();
	PWM_Loop();

	//

	beginTransfer();
    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_SET);
}

