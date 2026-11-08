#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/AirSensors.h"
#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <bsec2.h>

bool setupAirSensors();
void readAirSensors();
extern uint8_t currentBmeSensor;
uint8_t getActiveBmeCount();
String getAirSensorStatus();

// Implemented in SensorShield.ino; BSEC calls it when readings are ready.
void newBsecData(const bme68xData data, const bsecOutputs outputs, Bsec2 sensor);

bool isBmeSensorReady(uint8_t index);

void stopAirSensors();
void addAirSensorHealth(JsonObject sample);
