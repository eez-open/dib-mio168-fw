#pragma once

void DACDual_Setup();
void DACDual_SetParams(int i, SetParams &newState);
void DACDual_GetValueRange(float &min, float &max);
void DACDual_SetValue(int i, float value);
