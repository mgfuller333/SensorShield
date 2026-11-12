#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/LoggerConfig.h"
#pragma once
#include <Arduino.h>

// Measurement and upload policy. Battery threshold is for this board's 2S pack.
const uint32_t SLEEP_SECONDS = 20; // Sleep 1 minute after each measurement/upload attempt.
const uint32_t MEASUREMENT_TIMEOUT_MS = 30000;
const uint16_t UPLOAD_MAX_RECORDS = 8;
const size_t UPLOAD_MAX_BYTES = 48000;
const uint16_t UPLOAD_MIN_BATTERY_MV = 7000;
// Retry only a card that did not respond; never retry damaged queue recovery.
const uint32_t SD_STARTUP_DELAY_MS = 500; // Wait before the first SD attempt.
const uint8_t SD_STARTUP_ATTEMPTS = 3;
const uint32_t SD_RETRY_DELAY_MS = 250;
const uint32_t SD_SPI_CLOCK_HZ = 10000000; // 10 MHz SD transfers for board testing.
const uint32_t SD_MIN_FREE_BYTES = 1024UL * 1024UL;
const bool GPS_FIX_ENABLED = true;      // Refresh GPS before measuring when the stored fix is due.
const uint32_t GPS_REFRESH_SECONDS = 0; // Mobile platform: request a fresh fix every measurement cycle.
const uint32_t GPS_TIMEOUT_MS = 180000;
const uint8_t TLS_CA_CERTIFICATE_SLOT = 12; // Application CA slot, as in Walter HTTPS example.
const bool PRINT_PAYLOAD_JSON = false;      // Enable for detailed troubleshooting.
const bool PRINT_UPLOAD_JSON = true;        // Exact HTTP request body for API troubleshooting.
