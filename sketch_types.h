#pragma once

#include <Arduino.h>

// Small records for the charger readings and the OLED text.
struct BmsData {
  uint16_t chargeVoltageMv = 0;
  uint16_t chargeCurrentMa = 0;
  uint16_t batteryMv = 0;
  uint16_t systemMv = 0;
  int16_t batteryCurrentMa = 0;
  uint8_t chargeState = 0;
  uint8_t fault0 = 0;
  uint8_t fault1 = 0;
};

struct DeviceStatus {
  String setup = "---";
  String modem = "---";
  String gps = "---";
  String power = "---";
  String air = "---";
  String bms = "---";
};
