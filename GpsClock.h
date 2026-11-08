#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

// Initialize retained clock state and restore the last GNSS position.
void setupGpsClock();

// epochSeconds is Unix UTC seconds, receivedUs is esp_timer_get_time() at receipt.
void setGpsTime(int64_t epochSeconds, int64_t receivedUs);

// Returns an ISO-8601 UTC timestamp for the BigQuery TIMESTAMP field, or false.
bool formatGpsTimestamp(char *buffer, size_t bufferSize);
bool locationRefreshDue();
void saveLocation(double latitude, double longitude, double altitude,
                  uint8_t satellites, int64_t epoch);
void addLocationMetadata(JsonObject sample);
