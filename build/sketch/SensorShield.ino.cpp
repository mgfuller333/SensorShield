#include <Arduino.h>
#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/SensorShield.ino"
// Odrade: wake, measure, save locally, upload when eligible, then sleep.
#include "AirSensors.h"
#include "Charger.h"
#include "DataLogger.h"
#include "Display.h"
#include "GpsClock.h"
#include "LoggerConfig.h"
#include "Modem.h"
#include "ParticleSensor.h"
#include "PowerSensors.h"
#include "board_pins.h"
#include "sketch_types.h"
#include <Wire.h>
#include <SPI.h>
#include <esp_mac.h>
#include <esp_system.h>
#include <esp_sleep.h>
#include <driver/gpio.h>

JsonDocument payloadDoc;
JsonObject currentSample;
bool bmeSampleReceived[BME_COUNT] = {};
uint8_t sensorStepCount = 0;
uint32_t measurementStartedMs = 0;
bool storageAvailable = false;
bool cycleFinished = false;
bool devicesStopped = false;
String deviceId;
DeviceStatus status;

// These messages distinguish a processor reset from a disconnected USB monitor.
#line 32 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/SensorShield.ino"
const char * resetDescription();
#line 59 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/SensorShield.ino"
void printStartupStep(const char *message);
#line 64 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/SensorShield.ino"
void printResetReason();
#line 70 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/SensorShield.ino"
void setup();
#line 174 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/SensorShield.ino"
void loop();
#line 257 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/SensorShield.ino"
void finishAndSleep();
#line 296 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/SensorShield.ino"
void newBsecData(const bme68xData data, const bsecOutputs outputs, Bsec2 sensor);
#line 32 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/SensorShield.ino"
const char *resetDescription()
{
  switch (esp_reset_reason())
  {
  case ESP_RST_POWERON:
    return "power-on";
  case ESP_RST_DEEPSLEEP:
    return "scheduled wake from sleep";
  case ESP_RST_BROWNOUT:
    return "brownout: supply voltage dropped";
  case ESP_RST_PWR_GLITCH:
    return "power glitch detected";
  case ESP_RST_SW:
    return "software restart";
  case ESP_RST_PANIC:
    return "software crash";
  case ESP_RST_TASK_WDT:
  case ESP_RST_INT_WDT:
  case ESP_RST_WDT:
    return "watchdog: software stopped responding";
  case ESP_RST_USB:
    return "USB reset";
  default:
    return "other reset; see diagnostic code";
  }
}

void printStartupStep(const char *message)
{
  Serial.printf("[Startup %lu ms] %s\n", (unsigned long)millis(), message);
}

void printResetReason()
{
  Serial.printf("[Startup %lu ms] Boot reason: %s (code %d).\n",
                (unsigned long)millis(), resetDescription(), (int)esp_reset_reason());
}

void setup()
{
  Serial.begin(115200);
  // Keep time to open Serial after a manual start; timer wakes measure immediately.
  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER)
    delay(2000);
  Serial.println("\nODRADE | Environmental monitoring");
  Serial.println("Starting: measure once, save to SD, attempt upload, then sleep until the next 15-minute cycle.");
  printResetReason();
  printStartupStep("Enabling external 3.3 V; allowing sensors to settle.");

  // pinMode(48, OUTPUT);
  // printStartupStep("Test 0");
  // digitalWrite(48, 1);
  // printStartupStep("Test 1");
  // delay(200);
  // printStartupStep("Test 2");
  // digitalWrite(48, 0);
  // printStartupStep("Test 3");
  // gpio_hold_dis((gpio_num_t)WALTER_3V3_EN);
  // #define WALTER_3V3_EN 0 // Active LOW, matching the working INA3221 test.

  // printStartupStep("Test 4");
  // pinMode(WALTER_3V3_EN, OUTPUT);
  // printStartupStep("Test 5");
  // digitalWrite(WALTER_3V3_EN, 0);
  // printStartupStep("Test 6");

  delay(200); // Retain the power settling time from the working hardware test.
  printStartupStep("Power settling wait finished; voltage has not been verified.");
  printResetReason(); // Repeat in case the monitor disconnected during power-up.
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char id[13];
  snprintf(id, sizeof(id), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  deviceId = id;
  Serial.printf("Device: %s\n", id);

  // All chip selects must be high before any device uses the shared bus.
  for (uint8_t cs : BME_CS)
  {
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
  }
  pinMode(BMV080_CS, OUTPUT);
  digitalWrite(BMV080_CS, HIGH);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  printStartupStep("Allowing the SD card to settle before its first initialization attempt.");
  Serial.flush();
  delay(SD_STARTUP_DELAY_MS);
  printStartupStep("Starting SD card initialization.");
  storageAvailable = beginDataLogger();
  printStartupStep(storageAvailable ? "SD storage ready." : "SD initialization/recovery failed; continuing sensor checks.");
  if (!storageAvailable)
    Serial.println("Storage needs attention: this cycle cannot be saved until the card is working.");

  printStartupStep("Test 4");
  pinMode(WALTER_3V3_EN, OUTPUT);
  printStartupStep("Test 5");
  digitalWrite(WALTER_3V3_EN, 0);
  printStartupStep("Test 6");

  setupGpsClock();
  // Get time/location before measuring, so this cycle saves the new fix too.
  // Mobile platform: the configured zero refresh interval requests a fix every wake.
  if (GPS_FIX_ENABLED && locationRefreshDue()) {
    Serial.println("Location: starting GPS before this measurement.");
    if (setupModem()) {
      if (!refreshLocationIfDue())
        Serial.println("Location: radio standby was not confirmed; sensor checks continue, and upload remains blocked while GPS is active.");
    } else {
      Serial.println("Location: modem unavailable; this mobile measurement will have no new GPS position.");
    }
  } else if (GPS_FIX_ENABLED) {
    Serial.println("Location: using saved position and GPS-synchronized clock; no new fix needed.");
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Serial.println("Checking power sensors and battery...");
  printStartupStep("Starting power monitor checks.");

  setupPowerSensors();
  printPowerReadings();
  setupCharger();
  printChargerReadings();
  setupDisplay();
  Serial.println("Checking air-quality and particle sensors...");
  currentSample = payloadDoc.to<JsonObject>();
  currentSample["bID"] = nullptr; // Legacy batch ID is not assigned; queue identity uses device + seq.
  // Missing readings stay null, including sensors that never call back.
  for (uint8_t i = 0; i < BME_COUNT; ++i) {
    String suffix = String(i);
    for (const char *key : {"iaq_", "iaqAcc_", "co2_", "bvoc_", "pres_", "gasR_", "temp_", "hum_", "com_gas"})
      currentSample[String(key) + suffix] = nullptr;
  }
  printStartupStep("Starting air and particle sensor checks.");
  setupAirSensors();
  measurementStartedMs = millis();
  Serial.println("Measuring: readings appear as each sensor becomes ready.");
}

void loop()
{
  if (!cycleFinished)
  {
    readAirSensors();
    readCharger();
    bool complete = sensorStepCount == BME_COUNT && hasParticleReading();
    bool timedOut = millis() - measurementStartedMs >= MEASUREMENT_TIMEOUT_MS;
    if (!complete && !timedOut)
    {
      delay(10);
      return;
    }
    // Stop the particle worker cooperatively before SD or modem operations.
    if (!stopParticleSensor())
    {
      Serial.println("Waiting: the particle sensor is finishing its current operation.");
      delay(1000);
      return;
    }
    stopAirSensors();
    currentSample["device"] = deviceId;
    currentSample["measurementComplete"] = complete;
    currentSample["measurementDurationMs"] = millis() - measurementStartedMs;
    addAirSensorHealth(currentSample);
    JsonArray health = currentSample["bmeHealth"];
    for (uint8_t i = 0; i < BME_COUNT; ++i)
      health[i]["readingValid"] = bmeSampleReceived[i];
    addPowerToPayload(currentSample);
    stopPowerSensors();
    addParticlesToPayload(currentSample);
    addChargerToPayload(currentSample);
    addLocationMetadata(currentSample);
    Serial.printf("Measurement: %u of %u air sensors supplied data. Particle reading: %s.\n",
                  sensorStepCount, BME_COUNT, hasParticleReading() ? "available" : "unavailable");
    Serial.printf("Timing: sensor collection took %lu ms.\n", (unsigned long)(millis() - measurementStartedMs));
    if (!complete)
      Serial.println("Some readings are unavailable; the saved record will identify them.");
    if (PRINT_PAYLOAD_JSON)
    {
      serializeJsonPretty(currentSample, Serial);
      Serial.println();
    }
    if (payloadDoc.overflowed())
      Serial.println("Measurement: not enough memory to assemble the full record; this reading will not be marked saved.");
    // A late/slow card gets one final chance, with BME and particle SPI activity stopped.
    if (!storageAvailable)
      storageAvailable = retryDataLoggerAfterMeasurements();
    uint32_t saveStartedMs = millis();
    bool saved = !payloadDoc.overflowed() && storageAvailable && saveSample(currentSample);
    Serial.printf("Timing: saving the reading took %lu ms.\n", (unsigned long)(millis() - saveStartedMs));
    if (!saved)
      Serial.println("Not saved: storage needs attention. Existing queued readings have not been discarded.");

    // Only start the modem after the current sample is durably stored.
    if (saved)
    {
      if (!httpsEndpointConfigured())
        Serial.println("Upload paused: the server address is not configured. Readings stay on SD.");
      else
      {
        // Faulty/missing battery readings are diagnostic data, not an upload veto.
        Serial.println("Upload: sending readings with their health flags, including any battery faults.");
        String body, response;
        uint64_t first = 0, last = 0;
        int httpStatus = 0;
        uint32_t uploadStartedMs = millis();
        bool uploaded = false;
        if (prepareUpload(deviceId, body, first, last) && setupModem())
        {
          if (sendStoredPayload(body, httpStatus, response))
            uploaded = acknowledgeUpload(deviceId, first, last, httpStatus, response);
        }
        Serial.printf("Timing: upload attempt took %lu ms.\n", (unsigned long)(millis() - uploadStartedMs));
        if (!uploaded)
          Serial.println("Upload incomplete: readings stay on SD. Trying again next measurement cycle.");
      }
    }
    cycleFinished = true;
  }
  finishAndSleep();
}

void finishAndSleep()
{
  if (!devicesStopped)
  {
    if (!stopChargerTelemetry())
    {
      Serial.println("Waiting: battery monitor has not confirmed standby. Sleep postponed.");
      delay(5000);
      return;
    }
    stopDisplay();
    closeDataLogger();
    SPI.end();
    Wire.end();
    // Prevent signal pins from feeding the switched-off sensor supply.
    for (uint8_t cs : BME_CS)
      pinMode(cs, INPUT);
    for (uint8_t pin : {BMV080_CS, SD_CS, SPI_SCK, SPI_MISO, SPI_MOSI, I2C_SDA, I2C_SCL})
      pinMode(pin, INPUT);
    digitalWrite(WALTER_3V3_EN, HIGH);
    gpio_hold_en((gpio_num_t)WALTER_3V3_EN);
    gpio_deep_sleep_hold_en();
    devicesStopped = true;
  }
  if (!shutdownModem())
  {
    Serial.println("Sleep postponed: the cellular radio has not confirmed standby. Checking again in five seconds.");
    delay(5000);
    return;
  }
  uint64_t intervalMs = uint64_t(WAKE_INTERVAL_SECONDS) * 1000;
  uint64_t sleepMs = millis() < intervalMs ? intervalMs - millis() : 1000;
  Serial.printf("Timing: total awake time %lu ms.\n", (unsigned long)millis());
  Serial.printf("Sleeping: SD card closed, sensors off, and cellular radio in standby. Next measurement in %llu seconds.\n", sleepMs / 1000);
  Serial.flush();
  esp_sleep_enable_timer_wakeup(sleepMs * 1000);
  esp_deep_sleep_start();
}

void newBsecData(const bme68xData data, const bsecOutputs outputs, Bsec2 sensor)
{
  (void)sensor;
  if (currentBmeSensor >= BME_COUNT || !outputs.nOutputs || bmeSampleReceived[currentBmeSensor])
    return;
  String suffix = String(currentBmeSensor);
  bool temperature = false, humidity = false, iaq = false, co2 = false, voc = false;
  for (uint8_t i = 0; i < outputs.nOutputs; ++i)
  {
    const bsecData &o = outputs.output[i];
    switch (o.sensor_id)
    {
    case BSEC_OUTPUT_IAQ:
      currentSample["iaq_" + suffix] = o.signal;
      currentSample["iaqAcc_" + suffix] = o.accuracy;
      iaq = true;
      break;
    case BSEC_OUTPUT_RAW_TEMPERATURE:
      currentSample["temp_" + suffix] = o.signal;
      temperature = true;
      break;
    case BSEC_OUTPUT_RAW_HUMIDITY:
      currentSample["hum_" + suffix] = o.signal;
      humidity = true;
      break;
    case BSEC_OUTPUT_CO2_EQUIVALENT:
      currentSample["co2_" + suffix] = o.signal;
      co2 = true;
      break;
    case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
      currentSample["bvoc_" + suffix] = o.signal;
      voc = true;
      break;
    default:
      break;
    }
  }
  if (!(temperature && humidity && iaq && co2 && voc))
    return;
  // Raw measurements already supplied by Bosch; no BSEC subscription changes.
  if (data.status & BME68X_NEW_DATA_MSK) {
    currentSample["pres_" + suffix] = data.pressure / 100.0f; // Pa to hPa
    if ((data.status & (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK)) ==
        (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK))
      currentSample["gasR_" + suffix] = data.gas_resistance; // ohms
  }
  // com_gas remains null: its legacy meaning/calculation is not known.
  bmeSampleReceived[currentBmeSensor] = true;
  sensorStepCount++;
  Serial.printf("Air sensor %u: %.1f C, %.1f %% humidity, air-quality index %.1f (accuracy %u/3).\n",
                currentBmeSensor + 1, currentSample["temp_" + suffix].as<float>(), currentSample["hum_" + suffix].as<float>(),
                currentSample["iaq_" + suffix].as<float>(), currentSample["iaqAcc_" + suffix].as<unsigned>());
}

