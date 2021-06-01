#pragma once

#include "firmware.h"

void FuncGen_Setup();
void FuncGen_SetParams(SetParams &newState);
void FuncGen_onTimerPeriodElapsed();
