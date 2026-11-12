#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/Display.h"
#pragma once
#include <Arduino.h>

// Updated from the main Arduino task, never from modem/particle callbacks.
enum DisplayField { INIT_STATUS, GPS_STATUS, MODEM_STATUS, SENSOR_STATUS, SD_STATUS, PAYLOAD_STATUS };
bool setupDisplay();
void setDisplayStatus(DisplayField field, const char *value);
void setDisplaySensor(uint8_t index, const char *value); // A1-A8, PM, PW, BT
void showSensorPage();
void finishDisplaySensors();
void refreshDisplay();
bool isDisplayReady();
void stopDisplay();
