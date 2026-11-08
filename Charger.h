#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

bool setupCharger();
void readCharger();
void addChargerToPayload(JsonObject sample);
String getChargerStatus();

void printChargerReadings();

bool batteryAllowsUpload(uint16_t minimumMv);
bool stopChargerTelemetry();
