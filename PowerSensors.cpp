#include "PowerSensors.h"

// Based on Adafruit's INA3221_basictest example, adapted for two chips.
// U7: solar, USB-C, 12 V. U8: 5 V, 3.3 V, external input.
#include <Adafruit_INA3221.h>
#include <Wire.h>
#include <math.h>

const uint8_t INA3221_ADDR_1 = 0x40;
const uint8_t INA3221_ADDR_2 = 0x41;

Adafruit_INA3221 inaTop;
Adafruit_INA3221 inaBottom;
bool inaTopReady = false;
bool inaBottomReady = false;

bool powerReadingsSuspect = false;

// Plausibility warnings, not a diagnosis of wiring or supply faults.
// With the existing 0.05-ohm shunts, about 3276 mA is the ADC limit.
// Zero volts is allowed: a monitored input may simply be disconnected.
bool powerReadingSuspect(float volts, float milliamps, float maximumVolts) {
  return !isfinite(volts) || !isfinite(milliamps) || volts < 0 ||
         volts > maximumVolts || fabsf(milliamps) >= 3275.0f;
}

void warnPowerReading(const char *name, float volts, float milliamps, float maximumVolts) {
  if (!powerReadingSuspect(volts, milliamps, maximumVolts)) return;
  powerReadingsSuspect = true;
  Serial.printf("Power reading needs checking (%s): outside the expected voltage range or near the current measurement limit; verify with a meter.\n", name);
}

// Scan once at startup, before initializing either INA3221.
void scanI2C() {
  int devicesFound = 0;
  Serial.println("\n----- I2C Scan -----");
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      devicesFound++;
    } else if (error == 4) {
      Serial.print("Unknown I2C error at address 0x");
      Serial.println(address, HEX);
    }
  }
  Serial.print("Total devices found: ");
  Serial.println(devicesFound);
}

// Same initialization sequence as the working dual-INA3221 test.
bool initializeINA3221(Adafruit_INA3221 &ina, uint8_t address) {
  if (!ina.begin(address, &Wire)) {
    Serial.print("Power monitor unavailable (address 0x");
    Serial.print(address, HEX); Serial.println(")");
    return false;
  }

  Serial.print("Power monitor ready (address 0x");
  Serial.print(address, HEX); Serial.println(")");
  ina.setAveragingMode(INA3221_AVG_16_SAMPLES);
  for (uint8_t channel = 0; channel < 3; channel++) {
    ina.setShuntResistance(channel, 0.05f);
  }
  ina.setPowerValidLimits(3.0f, 15.0f);
  return true;
}

void setupPowerSensors() {
  // Initialize only known devices; a full bus scan is unnecessary on each wake.
  inaTopReady = initializeINA3221(inaTop, INA3221_ADDR_1);
  inaBottomReady = initializeINA3221(inaBottom, INA3221_ADDR_2);
}

void addPowerToPayload(JsonObject sample) {
  // Read with getBusVoltage/getCurrentAmps, just like the working test.
  // Library channel numbers start at zero. Each name becomes three JSON keys:
  // sol -> solV (volts), solC (milliamps), solP (milliwatts).
  const char *topNames[] = {"sol", "usbc", "v12"};
  const char *bottomNames[] = {"v5", "v3_3", "ext"};

  Wire.beginTransmission(INA3221_ADDR_1);
  bool topAvailable = Wire.endTransmission() == 0 && inaTopReady;
  Wire.beginTransmission(INA3221_ADDR_2);
  bool bottomAvailable = Wire.endTransmission() == 0 && inaBottomReady;
  powerReadingsSuspect = false;
  sample["ina40Valid"] = topAvailable;
  sample["ina41Valid"] = bottomAvailable;
  if (topAvailable) sample["ina40Flags"] = inaTop.getFlags();
  else sample["ina40Flags"] = nullptr;
  if (bottomAvailable) sample["ina41Flags"] = inaBottom.getFlags();
  else sample["ina41Flags"] = nullptr;
  for (uint8_t channel = 0; channel < 3; channel++) {
    // Missing sensors are explicitly invalid, never valid zero measurements.
    float voltage = 0;
    float current = 0;
    if (topAvailable) {
      voltage = inaTop.getBusVoltage(channel);
      current = inaTop.getCurrentAmps(channel) * 1000.0f;
    }
    // Adafruit returns shunt voltage in volts; the dashboard uses millivolts.
    if (channel == 0) {
      sample["solSV"] = nullptr;
      if (topAvailable) sample["solSV"] = inaTop.getShuntVoltage(channel) * 1000.0f;
    }
    String name = topNames[channel];
    bool suspect = !topAvailable || powerReadingSuspect(voltage, current, 26.0f);
    sample[name + "ReadingSuspect"] = suspect;
    powerReadingsSuspect |= suspect;
    sample[name + "V"] = voltage;
    sample[name + "C"] = current;
    sample[name + "P"] = voltage * current;
    if (!topAvailable) { sample[name+"V"] = nullptr; sample[name+"C"] = nullptr; sample[name+"P"] = nullptr; }

    voltage = 0;
    current = 0;
    if (bottomAvailable) {
      voltage = inaBottom.getBusVoltage(channel);
      current = inaBottom.getCurrentAmps(channel) * 1000.0f;
    }
    name = bottomNames[channel];
    // The two regulated rails get a generous upper limit; external input uses 26 V.
    float maximumVolts = channel == 0 ? 5.5f : (channel == 1 ? 3.6f : 26.0f);
    suspect = !bottomAvailable || powerReadingSuspect(voltage, current, maximumVolts);
    sample[name + "ReadingSuspect"] = suspect;
    powerReadingsSuspect |= suspect;
    sample[name + "V"] = voltage;
    sample[name + "C"] = current;
    sample[name + "P"] = voltage * current;
    if (!bottomAvailable) { sample[name+"V"] = nullptr; sample[name+"C"] = nullptr; sample[name+"P"] = nullptr; }
  }
}

String getPowerStatus() {
  if (inaTopReady && inaBottomReady && !powerReadingsSuspect)
    return "Healthy";
  if (inaTopReady || inaBottomReady)
    return "Partial";
  return "Bad";
}

void printPowerReadings() {
  powerReadingsSuspect = false;
  const char *names[] = {"Solar", "USB-C", "12 V", "5 V", "3.3 V", "External"};
  Serial.printf("Power monitors: onboard=%s, external=%s\n", inaTopReady ? "OK" : "FAIL", inaBottomReady ? "OK" : "FAIL");
  for (uint8_t channel = 0; channel < 3; channel++) {
    if (inaTopReady) {
      float v = inaTop.getBusVoltage(channel);
      float ma = inaTop.getCurrentAmps(channel) * 1000.0f;
      Serial.printf("  Channel %u, %s: %.3f volts, %.3f milliamps, %.3f milliwatts\n", channel, names[channel], v, ma, v*ma);
      warnPowerReading(names[channel], v, ma, 26.0f);
    }
    if (inaBottomReady) {
      float v = inaBottom.getBusVoltage(channel);
      float ma = inaBottom.getCurrentAmps(channel) * 1000.0f;
      Serial.printf("  Channel %u, %s: %.3f volts, %.3f milliamps, %.3f milliwatts\n", channel, names[channel+3], v, ma, v*ma);
      warnPowerReading(names[channel+3], v, ma, channel == 0 ? 5.5f : (channel == 1 ? 3.6f : 26.0f));
    }
  }
}

void stopPowerSensors() {
  if (inaTopReady && !inaTop.setMode(INA3221_MODE_POWER_DOWN))
    Serial.println("Power monitor: standby command failed; the sensor supply will be switched off before sleep.");
  if (inaBottomReady && !inaBottom.setMode(INA3221_MODE_POWER_DOWN))
    Serial.println("External power monitor: standby command failed; the sensor supply will be switched off before sleep.");
}
