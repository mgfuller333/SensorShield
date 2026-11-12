#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Start the dedicated BMV080 task, using the same SPI lock as the BME sensors.
bool setupParticleSensor(SemaphoreHandle_t spiMutex);
bool isParticleSensorReady();
void addParticlesToPayload(JsonObject sample);

void printParticleReadings();

bool hasParticleReading();
bool particleSensorFailed();
bool stopParticleSensor();
