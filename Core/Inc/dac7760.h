#pragma once

void DAC_Setup(int i);
void DAC_SetParams(int i, SetParams &newState);
void DAC_SetRange(int i, SetParams &newState);
void DAC_GetValueRange(uint8_t outputRange, float &min, float &max);
void DAC_SetValue_FromFuncGen(int i, float value);
