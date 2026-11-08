#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

void setupPowerSensors();
void addPowerToPayload(JsonObject sample);
String getPowerStatus();

void printPowerReadings();

void stopPowerSensors();
