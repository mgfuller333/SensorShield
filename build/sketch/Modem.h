#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/Modem.h"
#pragma once
#include <Arduino.h>
#include <WalterModem.h>

// WalterModem 1.6.1: GNSS time/position and Soracom HTTPS transport.
bool setupModem();
String getModemStatus();
bool httpsEndpointConfigured();

// Before measurement, refresh GNSS time/position when configured and due.
// A valid modem UTC clock can set ts even if a position fix is unavailable.
bool refreshLocationIfDue();

// POST already-serialized queued sensor records without changing their ts.
// DataLogger validates the response before marking records uploaded.
bool sendStoredPayload(const String &body, int &httpStatus, String &response);

// Send MINIMUM once to disable the radio; no polling or application retries.
bool shutdownModem();
