#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/ParticleSensor.cpp"
#include "ParticleSensor.h"
#include "board_pins.h"
#include <7Semi_BMV080.h>
#include <SPI.h>
#include <freertos/task.h>

BMV080_7Semi bmv;
SemaphoreHandle_t particleSpiMutex = nullptr;

// The task writes these values while the main loop reads them.
// Only hold this short lock while copying data, never during SPI or JSON work.
portMUX_TYPE particleDataLock = portMUX_INITIALIZER_UNLOCKED;
bmv080_output_t latestParticleData = {};
bool particleReady = false;
bool particleTaskAlive = false;
bool particleStopRequested = false;
bool hasParticleSample = false;
uint32_t particleSampleMs = 0;
const char *particleState = "Not started";

void setParticleState(const char *message) {
  portENTER_CRITICAL(&particleDataLock);
  particleState = message; // Only string literals are passed here.
  portEXIT_CRITICAL(&particleDataLock);
  Serial.printf("Particle sensor: %s\n", message);
}

void bmvTask(void *parameter) {
  (void)parameter;
  setParticleState("Initializing...");

  // Match the working example: CS=1, SCK=2, MISO=42, MOSI=41, 1 MHz.
  xSemaphoreTake(particleSpiMutex, portMAX_DELAY);
  bool initialized = bmv.begin(SPI, BMV080_CS, 1000000, SPI_SCK, SPI_MISO, SPI_MOSI);
  bool started = false;
  if (initialized) {
    started = bmv.startContinuous();
  }
  xSemaphoreGive(particleSpiMutex);

  if (!initialized || !started) {
    setParticleState(initialized ? "Start failed" : "Initialization failed");
    portENTER_CRITICAL(&particleDataLock);
    particleTaskAlive = false;
    portEXIT_CRITICAL(&particleDataLock);
    vTaskDelete(nullptr);
    return;
  }

  portENTER_CRITICAL(&particleDataLock);
  particleReady = true;
  portEXIT_CRITICAL(&particleDataLock);
  setParticleState("ready to measure");

  bmv080_output_t data = {};
  bool printedFirstReading = false;
  while (true) {
    portENTER_CRITICAL(&particleDataLock);
    bool stopRequested = particleStopRequested;
    portEXIT_CRITICAL(&particleDataLock);
    if (stopRequested) {
      xSemaphoreTake(particleSpiMutex, portMAX_DELAY);
      bool stopped = bmv.stop();
      if (stopped) bmv.end();
      xSemaphoreGive(particleSpiMutex);
      if (stopped) {
        portENTER_CRITICAL(&particleDataLock);
        particleTaskAlive = false;
        portEXIT_CRITICAL(&particleDataLock);
        vTaskDelete(nullptr);
        return;
      }
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    // One sensor driver at a time may use this shared SPI bus.
    xSemaphoreTake(particleSpiMutex, portMAX_DELAY);
    bool newData = bmv.getSensorData(data);
    xSemaphoreGive(particleSpiMutex);

    if (newData) {
      portENTER_CRITICAL(&particleDataLock);
      latestParticleData = data;
      hasParticleSample = true;
      particleSampleMs = millis();
      portEXIT_CRITICAL(&particleDataLock);
      if (!printedFirstReading) {
        printParticleReadings();
        printedFirstReading = true;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

bool setupParticleSensor(SemaphoreHandle_t spiMutex) {
  if (!spiMutex) {
    setParticleState("SPI mutex unavailable");
    return false;
  }
  particleSpiMutex = spiMutex;
  // ESP32 specifies task stack size in bytes: 65536 = 64 KB, as in the working test.
  particleTaskAlive = true;
  BaseType_t result = xTaskCreatePinnedToCore(
      bmvTask, "BMV080", 65536, nullptr, 1, nullptr, 1);
  if (result != pdPASS) {
    particleTaskAlive = false;
    setParticleState("Insufficient internal RAM for task");
    return false;
  }
  return true; // Task created; it reports initialization success/failure separately.
}

bool isParticleSensorReady() {
  portENTER_CRITICAL(&particleDataLock);
  bool ready = particleReady;
  portEXIT_CRITICAL(&particleDataLock);
  return ready;
}

void addParticlesToPayload(JsonObject sample) {
  bmv080_output_t data = {};
  portENTER_CRITICAL(&particleDataLock);
  bool available = hasParticleSample;
  const char *state = particleState;
  uint32_t sampleMs = particleSampleMs;
  if (available) data = latestParticleData;
  portEXIT_CRITICAL(&particleDataLock);
  sample["particleAvailable"] = available;
  sample["particleState"] = state;
  sample["particleValid"] = available && !data.is_obstructed && !data.is_outside_measurement_range;
  if (!available) {
    sample["pmObs"] = nullptr; sample["pmOor"] = nullptr; sample["particleAgeMs"] = nullptr;
    sample["pm1"] = nullptr; sample["pm2_5"] = nullptr; sample["pm10"] = nullptr;
    return;
  }

  sample["particleAgeMs"] = millis()-sampleMs;
  sample["pm1"] = data.pm1_mass_concentration;
  sample["pm2_5"] = data.pm2_5_mass_concentration;
  sample["pm10"] = data.pm10_mass_concentration;
  sample["pmObs"] = data.is_obstructed;
  sample["pmOor"] = data.is_outside_measurement_range;
}

void printParticleReadings() {
  bmv080_output_t data = {};
  portENTER_CRITICAL(&particleDataLock);
  const char *state = particleState;
  bool available = hasParticleSample;
  uint32_t sampleMs = particleSampleMs;
  if (available) data = latestParticleData;
  portEXIT_CRITICAL(&particleDataLock);
  if (!available) {
    Serial.printf("Particle sensor: %s; no readings yet\n", state);
    return;
  }
  Serial.printf("Particle sensor (%s): PM1 %.2f, PM2.5 %.2f, PM10 %.2f micrograms/m3; blocked=%d, outside range=%d, reading age=%lu ms\n",
                state, data.pm1_mass_concentration, data.pm2_5_mass_concentration,
                data.pm10_mass_concentration, data.is_obstructed,
                data.is_outside_measurement_range, (unsigned long)(millis()-sampleMs));
}

bool hasParticleReading() {
  portENTER_CRITICAL(&particleDataLock);
  bool available = hasParticleSample;
  portEXIT_CRITICAL(&particleDataLock);
  return available;
}

bool stopParticleSensor() {
  portENTER_CRITICAL(&particleDataLock);
  particleStopRequested = true;
  portEXIT_CRITICAL(&particleDataLock);
  uint32_t start = millis();
  do {
    portENTER_CRITICAL(&particleDataLock);
    bool running = particleTaskAlive;
    portEXIT_CRITICAL(&particleDataLock);
    if (!running) return true;
    delay(10);
  } while (millis() - start < 5000);
  return false;
}

// Read from the main loop; the particle worker never writes to the OLED/I2C bus.
bool particleSensorFailed() {
  portENTER_CRITICAL(&particleDataLock);
  bool failed = !particleTaskAlive && !hasParticleSample && !particleStopRequested;
  portEXIT_CRITICAL(&particleDataLock);
  return failed;
}
