#include "Charger.h"
#include "sketch_types.h"

bool configureCharger();
void readChargerTelemetry();

// Follows test_bq25798, with this board's existing 2-cell charger settings.
// Raw ADC/status reads are retained because the installed library lacks these helpers.
#include <Adafruit_BQ25798.h>
#include <Wire.h>

const float CHARGE_VOLTAGE = 8.1f;
const float CHARGE_CURRENT = 1.0f;
const float MIN_SYSTEM_VOLTAGE = 7.0f;
const uint32_t POLL_INTERVAL_MS = 5000;

Adafruit_BQ25798 charger;
BmsData chargerData = {};
bool chargerReady = false;
bool chargerTelemetryValid = false;
uint32_t lastChargerReadMs = 0;

bool setupCharger() {
  chargerReady = charger.begin(BQ25798_DEFAULT_ADDR, &Wire) && configureCharger();
  readChargerTelemetry();

  Serial.print("Battery monitor: ");
  Serial.println(chargerReady ? "OK" : "setup failed");
  return chargerReady;
}

void readCharger() {
  if (!chargerReady || millis() - lastChargerReadMs < POLL_INTERVAL_MS) {
    return;
  }

  lastChargerReadMs = millis();
  readChargerTelemetry();
}

void addChargerToPayload(JsonObject sample) {
  // Legacy dashboard names: volts, milliamps and milliwatts.
  sample["batV"] = nullptr; sample["batC"] = nullptr; sample["batP"] = nullptr;
  sample["batSV"] = nullptr; // The charger does not provide battery shunt voltage.
  sample["chargerValid"] = chargerReady && chargerTelemetryValid;
  // A very low reading may mean no battery is connected, not a failed charger.
  sample["batteryReadingSuspect"] = !chargerReady || !chargerTelemetryValid ||
                                   chargerData.batteryMv < 4000 || chargerData.batteryMv > 9000;
  if (!chargerReady || !chargerTelemetryValid) {
    for (const char *key : {"bmsVreg", "bmsIchg", "batMv", "sysMv", "ibatMa", "chgState", "bmsF0", "bmsF1"}) sample[key] = nullptr;
    return;
  }
  sample["bmsVreg"] = chargerData.chargeVoltageMv;
  sample["bmsIchg"] = chargerData.chargeCurrentMa;
  sample["batV"] = chargerData.batteryMv / 1000.0f;
  sample["batC"] = chargerData.batteryCurrentMa;
  sample["batP"] = (chargerData.batteryMv / 1000.0f) * chargerData.batteryCurrentMa;
  sample["batMv"] = chargerData.batteryMv;
  sample["sysMv"] = chargerData.systemMv;
  sample["ibatMa"] = chargerData.batteryCurrentMa;
  sample["chgState"] = chargerData.chargeState;
  sample["bmsF0"] = chargerData.fault0;
  sample["bmsF1"] = chargerData.fault1;
}

String getChargerStatus() {
  static const char *const states[] = {"Idle",  "Trickle", "Pre",    "Fast",
                                       "Taper", "Healthy", "Topoff", "Done"};

  if (!chargerReady) {
    return "Bad";
  }

  if (chargerData.fault0 || chargerData.fault1) {
    return "Fault";
  }

  return states[chargerData.chargeState & 0x07];
}

bool writeChargerRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BQ25798_DEFAULT_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readChargerRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(BQ25798_DEFAULT_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    chargerTelemetryValid = false;
    return false;
  }

  if (Wire.requestFrom(BQ25798_DEFAULT_ADDR, length) != length) {
    chargerTelemetryValid = false;
    return false;
  }

  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

uint8_t readChargerRegister8(uint8_t reg) {
  uint8_t value = 0;
  readChargerRegisters(reg, &value, sizeof(value));
  return value;
}

uint16_t readChargerRegister16(uint8_t reg) {
  uint8_t bytes[2] = {};
  readChargerRegisters(reg, bytes, sizeof(bytes));
  return ((uint16_t)bytes[0] << 8) | bytes[1];
}

bool configureCharger() {
  // Keep the board's existing battery settings. Stop if any write fails.
  if (!charger.setCellCount(BQ25798_CELL_COUNT_2S))
    return false;
  if (!charger.setChargeLimitV(CHARGE_VOLTAGE))
    return false;
  if (!charger.setChargeLimitA(CHARGE_CURRENT))
    return false;
  if (!charger.setMinSystemV(MIN_SYSTEM_VOLTAGE))
    return false;
  if (!charger.setWDT(BQ25798_WDT_DISABLE))
    return false;
  if (!charger.setHIZMode(false))
    return false;
  if (!charger.setChargeEnable(true))
    return false;
  // Existing ADC control value: enable conversions for telemetry.
  return writeChargerRegister(BQ25798_REG_ADC_CONTROL, 0xB0);
}

void readChargerTelemetry() {
  chargerTelemetryValid = chargerReady;
  if (!chargerReady) return;
  chargerData.chargeVoltageMv = (uint16_t)(charger.getChargeLimitV() * 1000.0f + 0.5f);
  chargerData.chargeCurrentMa = (uint16_t)(charger.getChargeLimitA() * 1000.0f + 0.5f);
  chargerData.batteryMv = readChargerRegister16(BQ25798_REG_VBAT_ADC);
  chargerData.systemMv = readChargerRegister16(BQ25798_REG_VSYS_ADC);
  chargerData.batteryCurrentMa = (int16_t)readChargerRegister16(BQ25798_REG_IBAT_ADC);
  chargerData.chargeState = (readChargerRegister8(BQ25798_REG_CHARGER_STATUS_1) >> 5) & 0x07;
  chargerData.fault0 = readChargerRegister8(BQ25798_REG_FAULT_STATUS_0);
  chargerData.fault1 = readChargerRegister8(BQ25798_REG_FAULT_STATUS_1);
}

void printChargerReadings() {
  if (!chargerReady) {
    Serial.println("Battery monitor: unavailable; battery and charging readings cannot be verified.");
    return;
  }
  Serial.printf("Battery monitor (%s): battery %u mV, system %u mV, current %d mA. Diagnostic flags: 0x%02X/0x%02X\n",
                getChargerStatus().c_str(), chargerData.batteryMv, chargerData.systemMv,
                chargerData.batteryCurrentMa, chargerData.fault0, chargerData.fault1);
  if (!chargerTelemetryValid || chargerData.batteryMv < 4000 || chargerData.batteryMv > 9000)
    Serial.println("Battery reading needs checking: battery may be disconnected or outside the expected 2-cell range; verify with a meter.");
}

bool batteryAllowsUpload(uint16_t minimumMv) {
  readChargerTelemetry();
  return chargerReady && chargerTelemetryValid && chargerData.batteryMv >= minimumMv &&
         chargerData.batteryMv <= 9000 && !chargerData.fault0 && !chargerData.fault1;
}
bool stopChargerTelemetry() {
  // Charging stays enabled; only the telemetry ADC is stopped for sleep.
  return !chargerReady || writeChargerRegister(BQ25798_REG_ADC_CONTROL, 0x00);
}
