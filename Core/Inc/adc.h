#pragma once

void ADC_Setup();
void ADC_SetParams(SetParams &newState);

extern uint16_t ADC_faultStatus;
extern uint8_t ADC_autoRangeAFE2;
void ADC_GetSamples(float *samples, uint8_t *ranges);

extern SPI_HandleTypeDef *hspiADC;
void ADC_DMA_TransferCompleted(bool ok);

void ADC_DLOG_Start(Request &request, Response &response);
void ADC_DLOG_Stop(Request &request, Response &response);

void ADC_Tick(float period);

void ADC_autoRange();

extern float g_activePower;
extern float g_reactivePower;
extern float g_voltRMS;
extern float g_currRMS;
