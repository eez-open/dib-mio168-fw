#pragma once

void ADC_Setup();
void ADC_SetParams(SetParams &newState);

extern uint16_t ADC_faultStatus;
void ADC_GetSamples(float *samples);

extern SPI_HandleTypeDef *hspiADC;
void ADC_DMA_TransferCompleted(bool ok);

extern bool ADC_DLOG_started;
void ADC_DLOG_Start(Request &request, Response &response);
void ADC_DLOG_Stop(Request &request, Response &response);
void ADC_DLOG_Data(Response &response);