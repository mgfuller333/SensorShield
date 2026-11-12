#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/Charger.h"
#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

// Assert /CE before startup delays; retain LOW through deep sleep and wake.
bool holdChargerEnableLow();
bool setupCharger();
void readCharger();
void addChargerToPayload(JsonObject sample);
String getChargerStatus();

void printChargerReadings();

bool batteryAllowsUpload(uint16_t minimumMv);
bool stopChargerTelemetry();
