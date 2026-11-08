#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/PowerSensors.h"
#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

void setupPowerSensors();
void addPowerToPayload(JsonObject sample);
String getPowerStatus();

void printPowerReadings();

void stopPowerSensors();
