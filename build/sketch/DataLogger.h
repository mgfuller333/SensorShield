#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/DataLogger.h"
#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

bool beginDataLogger();
// Call only after all sensor SPI activity has stopped.
bool retryDataLoggerAfterMeasurements();
bool saveSample(JsonObject sample);
// Only the successfully saved sample from this wake is eligible, once.
bool prepareUpload(const String &device, String &body, uint64_t &first, uint64_t &last);
bool acknowledgeUpload(const String &device, uint64_t first, uint64_t last,
                       int httpStatus, const String &response);
bool closeDataLogger();
