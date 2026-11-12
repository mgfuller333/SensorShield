#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/AirSensors.cpp"
#include "Display.h"
#include "AirSensors.h"
#include "board_pins.h"
#include "ParticleSensor.h"

void checkBsecStatus(Bsec2 sensor);

// Follows Bosch BSEC basic/x8 examples and 7Semi Basic_SPI.
// The board has eight individual chip-select pins, so no SPI multiplexer is needed.
#include <SPI.h>

#if defined(ESP32)
// BMV080 has its own 64 KB task; the main sensor/storage loop uses 16 KB.
SET_LOOP_TASK_STACK_SIZE(16 * 1024);
#endif

const uint8_t MAX_BME_INIT_ATTEMPTS = 3;
const float SAMPLE_RATE = BSEC_SAMPLE_RATE_LP;

static bsecSensor BSEC_OUTPUTS[] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
};
const uint8_t BSEC_OUTPUT_COUNT = sizeof(BSEC_OUTPUTS) / sizeof(BSEC_OUTPUTS[0]);

Bsec2 bme[BME_COUNT];
uint8_t bsecMemory[BME_COUNT][BSEC_INSTANCE_SIZE];
bool bmeReady[BME_COUNT] = {false};
int bmeSubscriptionStatus[BME_COUNT] = {};
uint8_t activeBmeCount = 0;
uint32_t lastBmeErrorMs[BME_COUNT] = {};

// Set before run(), just like currentSensor in the working BSEC example.
uint8_t currentBmeSensor = 0;
SemaphoreHandle_t sensorSpiMutex = nullptr;

bool setupAirSensors()
{
  activeBmeCount = 0;

  sensorSpiMutex = xSemaphoreCreateMutex();
  if (!sensorSpiMutex) {
    Serial.println("Air sensors unavailable: not enough memory to coordinate the shared connection.");
    return false;
  }

  // Deselect every SPI device before initializing any one of them.
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  pinMode(BMV080_CS, OUTPUT);
  digitalWrite(BMV080_CS, HIGH);
  for (uint8_t i = 0; i < BME_COUNT; i++) {
    pinMode(BME_CS[i], OUTPUT);
    digitalWrite(BME_CS[i], HIGH);
  }
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  // Each BSEC instance needs its own memory block and SPI chip select.
  for (uint8_t i = 0; i < BME_COUNT; i++)
  {
    pinMode(BME_CS[i], OUTPUT);
    digitalWrite(BME_CS[i], HIGH);
    bmeReady[i] = false;
    bmeSubscriptionStatus[i] = 0;

    setDisplaySensor(i, "gathering");
    refreshDisplay();
    Serial.printf("Air sensor %u: starting (connection pin %u).\n", i+1, BME_CS[i]);

    bme[i].allocateMemory(bsecMemory[i]);
    for (uint8_t attempt = 0; attempt < MAX_BME_INIT_ATTEMPTS; attempt++)
    {
      digitalWrite(BME_CS[i], HIGH);
      delay(50);
      bmeReady[i] = bme[i].begin(BME_CS[i], SPI);
      if (bmeReady[i])
        break;
      checkBsecStatus(bme[i]);
      delay(200);
    }

    if (!bmeReady[i])
    {
      Serial.printf("Air sensor %u (pin %u): could not start. Diagnostic codes: %d / %d\n",
                    i+1, BME_CS[i], (int)bme[i].status, (int)bme[i].sensor.status);
      setDisplaySensor(i, "failed");
      refreshDisplay();
      continue;
    }

    if (i == 0) {
      Serial.printf("Air algorithm: BSEC %u.%u.%u.%u; requested interval %.1f seconds.\n",
                    bme[i].version.major, bme[i].version.minor,
                    bme[i].version.major_bugfix, bme[i].version.minor_bugfix, 1.0f / SAMPLE_RATE);
      if (bme[i].version.major < 3 || (bme[i].version.major == 3 && bme[i].version.minor < 2))
        Serial.println("Air algorithm warning: Bosch specifies BSEC 3.2 or newer for BME690; verify installed binary and matching configuration.");
    }

    if (SAMPLE_RATE == BSEC_SAMPLE_RATE_ULP)
    {
      bme[i].setTemperatureOffset(TEMP_OFFSET_ULP);
    }
    else if (SAMPLE_RATE == BSEC_SAMPLE_RATE_LP)
    {
      bme[i].setTemperatureOffset(TEMP_OFFSET_LP);
    }

    if (!bme[i].updateSubscription(BSEC_OUTPUTS, BSEC_OUTPUT_COUNT, SAMPLE_RATE))
    {
      bmeSubscriptionStatus[i] = (int)bme[i].status;
      if (bme[i].status == BSEC_W_SU_SAMPLERATEMISMATCH)
        Serial.println("Air configuration warning: requested 3-second sampling does not match the loaded profile. A matching BME690 profile is needed; readings may be delayed.");
      Serial.printf("Air sensor %u (pin %u): measurement setup needs attention. Diagnostic codes: %d / %d\n",
                    i+1, BME_CS[i], (int)bme[i].status, (int)bme[i].sensor.status);
      // The library returns false for positive warnings too. Like the working
      // example, still attach the callback and run when there is only a warning.
      if (bme[i].status <= BSEC_OK || bme[i].sensor.status < BME68X_OK) {
        Serial.println("Measurement setup failed; this air sensor is unavailable.");
        bmeReady[i] = false;
    bmeSubscriptionStatus[i] = 0;
        continue;
      }
      Serial.println("Measurement setup reported a warning; this air sensor will still be read.");
    }
    // BSEC calls this function in SensorShield.ino when outputs are ready.
    bme[i].attachCallback(newBsecData);
    bmeReady[i] = true;
    activeBmeCount++;
    Serial.printf("Air sensor %u (pin %u): ready; waiting for its first reading\n", i+1, BME_CS[i]);
  }

  Serial.printf("Air sensors: %u of %u ready to measure\n", activeBmeCount, BME_COUNT);

  // Launch only after BME initialization, so both drivers share the SPI lock.
  setDisplaySensor(8, "gathering");
  refreshDisplay();
  bool particleTaskStarted = setupParticleSensor(sensorSpiMutex);
  if (!particleTaskStarted) setDisplaySensor(8, "failed");
  return activeBmeCount > 0 || particleTaskStarted;
}

uint8_t getActiveBmeCount() {
  return activeBmeCount;
}

void readAirSensors()
{

  for (uint8_t i = 0; i < BME_COUNT; i++)
  {
    if (!bmeReady[i])
    {
      continue;
    }

    // The BSEC callback runs inside run(), so this index identifies its sensor.
    currentBmeSensor = i;
    if (xSemaphoreTake(sensorSpiMutex, pdMS_TO_TICKS(20)) != pdTRUE) continue;
    bool success = bme[i].run();
    xSemaphoreGive(sensorSpiMutex);
    if (success)
    {
      continue;
    }

    // Report errors without rebooting the INA3221, charger and particle sensor.
    if (millis() - lastBmeErrorMs[i] >= 5000) {
      Serial.printf("Air sensor %u: reading unsuccessful; retrying\n", i+1);
      checkBsecStatus(bme[i]);
      lastBmeErrorMs[i] = millis();
    }
  }

}

String getAirSensorStatus()
{
  bool bmv080Ready = isParticleSensorReady();
  if (activeBmeCount == BME_COUNT && bmv080Ready)
  {
    return "Healthy";
  }

  if (activeBmeCount > 0 || bmv080Ready)
  {
    return "Partial";
  }

  return "Bad";
}

void checkBsecStatus(Bsec2 sensor)
{
  if (sensor.status < BSEC_OK)
  {
    Serial.println("BSEC error code: " + String(sensor.status));
  }
  else if (sensor.status > BSEC_OK)
  {
    Serial.println("BSEC warning code: " + String(sensor.status));
  }

  if (sensor.sensor.status < BME68X_OK)
  {
    Serial.println("BME68X error code: " + String(sensor.sensor.status));
  }
  else if (sensor.sensor.status > BME68X_OK)
  {
    Serial.println("BME68X warning code: " + String(sensor.sensor.status));
  }
}

bool isBmeSensorReady(uint8_t index) {
  return index < BME_COUNT && bmeReady[index];
}

void stopAirSensors() {
  for (uint8_t i = 0; i < BME_COUNT; ++i) {
    if (bmeReady[i]) bme[i].sensor.setOpMode(BME68X_SLEEP_MODE);
    digitalWrite(BME_CS[i], HIGH);
  }
}
void addAirSensorHealth(JsonObject sample) {
  JsonArray health = sample["bmeHealth"].to<JsonArray>();
  for (uint8_t i = 0; i < BME_COUNT; ++i) {
    JsonObject h = health.add<JsonObject>();
    h["index"] = i; h["cs"] = BME_CS[i]; h["initialized"] = bmeReady[i];
    h["subscriptionStatus"] = bmeSubscriptionStatus[i];
    h["configurationMatched"] = bmeSubscriptionStatus[i] == BSEC_OK;
    h["bsecStatus"] = (int)bme[i].status; h["sensorStatus"] = (int)bme[i].sensor.status;
  }
}
