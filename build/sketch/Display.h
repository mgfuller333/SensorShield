#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/Display.h"
#pragma once

#include "sketch_types.h"

bool setupDisplay();
void showDisplay(const DeviceStatus &status);

bool isDisplayReady();

void stopDisplay();
