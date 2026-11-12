#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/Charger.cpp"
#include "Charger.h"
#include "sketch_types.h"
#include "board_pins.h"

bool configureCharger();
void readChargerTelemetry();

// Follows the supplied working Sriko 33140 2S / Adafruit example.
// Raw ADC/status reads are retained because the installed library lacks these helpers.
#include <Adafruit_BQ25798.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <cmath>

const float CHARGE_VOLTAGE = 7.90f;
const float CHARGE_CURRENT = 2.00f;
const float INPUT_CURRENT = 2.00f;
const float MIN_SYSTEM_VOLTAGE = 5.00f;
const uint32_t POLL_INTERVAL_MS = 5000;

Adafruit_BQ25798 charger;
BmsData chargerData = {};
bool chargerReady = false;
bool chargerTelemetryValid = false;
uint32_t lastChargerReadMs = 0;

bool holdChargerEnableLow() {
  // Configure the wake-up output BEFORE releasing the previous hold, so /CE
  // stays LOW. Open-drain only sinks current; it never drives this net HIGH.
  pinMode(BQ_CE_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(BQ_CE_PIN, LOW);
  if (gpio_hold_dis((gpio_num_t)BQ_CE_PIN) != ESP_OK) return false;
  if (gpio_hold_en((gpio_num_t)BQ_CE_PIN) != ESP_OK) return false;
  gpio_deep_sleep_hold_en();
  return true;
  // This hold does not survive power loss or fix the PCB's REGN pull-up voltage.
}

bool setupCharger() {
  // begin() resets the charger; call it once per wake, then apply all settings.
  chargerReady = holdChargerEnableLow() &&
                 charger.begin(BQ25798_DEFAULT_ADDR, &Wire) && configureCharger();
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
  // Match the working example: pause charging, configure, verify, then enable.
  if (!charger.setChargeEnable(false) || charger.getChargeEnable()) {
    Serial.println("Charger: could not pause charging for configuration.");
    return false;
  }
  bool ok = true;
  ok &= charger.setWDT(BQ25798_WDT_DISABLE); // No host servicing needed in sleep.
  ok &= charger.setCellCount(BQ25798_CELL_COUNT_2S);
  ok &= charger.setMinSystemV(MIN_SYSTEM_VOLTAGE);
  ok &= charger.setChargeLimitV(CHARGE_VOLTAGE);
  ok &= charger.setChargeLimitA(CHARGE_CURRENT);
  ok &= charger.setInputLimitA(INPUT_CURRENT);
  ok &= charger.setPrechargeLimitA(2.0f);
  ok &= charger.setTerminationA(0.48f);
  ok &= charger.setTerminationEnable(true);
  ok &= charger.setTrickleChargeTimerEnable(true);
  ok &= charger.setPrechargeTimerEnable(true);
  ok &= charger.setFastChargeTimerEnable(true);
  ok &= charger.setOTGenable(false);
  ok &= charger.setIINDPMenable(true);
  ok &= charger.setHIZMode(false);
  ok &= charger.setVINDPM_VOCpercent(BQ25798_VOC_PCT_87_5);
  ok &= charger.setVOCrate(BQ25798_VOC_RATE_2MIN);
  ok &= charger.setVOCdelay(BQ25798_VOC_DLY_300MS);
  // Preserve the working example's USB/low-battery mode. Solar input can charge
  // without MPPT; do not enable a different solar algorithm without testing it.
  ok &= charger.setMPPTenable(false);
  uint8_t adc = 0;
  if (readChargerRegisters(BQ25798_REG_ADC_CONTROL, &adc, 1))
    ok &= writeChargerRegister(BQ25798_REG_ADC_CONTROL, (adc | 0x80) & ~0x40);
  else
    ok = false;

  ok &= charger.getCellCount() == BQ25798_CELL_COUNT_2S;
  ok &= fabsf(charger.getChargeLimitV() - CHARGE_VOLTAGE) < 0.03f;
  ok &= fabsf(charger.getChargeLimitA() - CHARGE_CURRENT) < 0.03f;
  ok &= fabsf(charger.getInputLimitA() - INPUT_CURRENT) < 0.06f;
  ok &= fabsf(charger.getMinSystemV() - MIN_SYSTEM_VOLTAGE) < 0.03f;
  ok &= charger.getWDT() == BQ25798_WDT_DISABLE;
  if (!ok) {
    charger.setChargeEnable(false);
    Serial.println("Charger: configuration check failed; charging has not been enabled.");
    return false;
  }
  if (!charger.setChargeEnable(true) || !charger.getChargeEnable()) {
    charger.setChargeEnable(false);
    Serial.println("Charger: could not confirm charging enabled.");
    return false;
  }
  Serial.println("Charger: 2 cells, 7.90 V target, 2.00 A charge/input limits, 5.00 V minimum system voltage.");
  Serial.println("Charging enabled: GPIO10 /CE held LOW, including during deep sleep. Watchdog disabled; protection timers enabled.");
  Serial.println("Solar: MPPT remains off, matching the working USB/low-battery example.");
  return true;
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
  // Keep the charger running independently of the sleeping ESP32. Stop only ADC.
  if (!holdChargerEnableLow()) return false;
  if (!chargerReady) return true;
  uint8_t adc = 0;
  return readChargerRegisters(BQ25798_REG_ADC_CONTROL, &adc, 1) &&
         writeChargerRegister(BQ25798_REG_ADC_CONTROL, adc & ~0x80);
}
