#pragma once

#include "sketch_types.h"

bool setupDisplay();
void showDisplay(const DeviceStatus &status);

bool isDisplayReady();

void stopDisplay();
