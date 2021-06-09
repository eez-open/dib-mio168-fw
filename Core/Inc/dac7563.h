#pragma once

void DACDual_Setup();
void DACDual_SetParams(int i, SetParams &newState);
void DACDual_GetValueRange(float &min, float &max);
void DACDual_SetValue_FromFuncGen(int i, float value);
