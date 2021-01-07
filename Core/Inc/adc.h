#pragma once

void ADC_Setup();
void ADC_SetParams(SetParams &newState);

extern float ADC_samples[4];
extern uint16_t ADC_faultStatus;

extern SPI_HandleTypeDef *hspiADC;
void ADC_Measure_Finish(bool ok);