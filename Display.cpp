#include "Display.h"
#include "board_pins.h"

// Follows ssd1306_128x64_i2c: clear, set text style, print, display.
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

const uint8_t SCREEN_WIDTH = 128;
const uint8_t SCREEN_HEIGHT = 64;
const int8_t OLED_RESET = -1;
// Fixed address confirmed by the working OLED test.
const uint8_t OLED_ADDRESS = 0x3C;

// Same four-argument constructor as the working OLED test.
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayReady = false;

const char *screenState[] = {"working", "not started", "disconnected", "not started", "not started", "not sent"};
const char *sensorNames[] = {"A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8", "PM", "PW", "BT"};
const char *sensorState[11] = {"not started", "not started", "not started", "not started", "not started", "not started", "not started", "not started", "not started", "not started", "not started"};
bool sensorPage = false, screenDirty = true;
uint32_t lastScreenMs = 0;

void setDisplayStatus(DisplayField field, const char *value) {
  if (strcmp(screenState[field], value) == 0) return;
  screenState[field] = value;
  screenDirty = true;
  refreshDisplay();
}

void setDisplaySensor(uint8_t index, const char *value) {
  if (index >= 11 || strcmp(sensorState[index], value) == 0) return;
  sensorState[index] = value;
  screenDirty = true;
}

// A warning means data arrived but needs checking, not that collection failed.
void finishDisplaySensors() {
  uint8_t received = 0;
  bool warning = false;
  for (const char *state : sensorState) {
    bool check = strcmp(state, "check") == 0;
    if (strcmp(state, "done") == 0 || check) ++received;
    warning |= check;
  }
  setDisplayStatus(SENSOR_STATUS, received == 0 ? "failed" :
                   received < 11 ? "partial" : warning ? "done (check)" : "done");
}

void showSensorPage() { sensorPage = true; screenDirty = true; refreshDisplay(); }

void displayLine(const char *key, const char *value) {
  display.print(key); display.print(":"); display.println(value);
}

// 8 lines of 21 characters at the library's standard 6x8 font size.
// Rotate sensor rows every 2 seconds, without delays or a background I2C task.
void refreshDisplay() {
  if (!displayReady) return;
  uint32_t now = millis();
  if (!screenDirty && (!sensorPage || now - lastScreenMs < 250)) return;
  lastScreenMs = now; screenDirty = false;
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  if (!sensorPage) {
    display.println(F("ODRADE STATUS"));
    displayLine("Init", screenState[INIT_STATUS]);
    displayLine("GPS", screenState[GPS_STATUS]);
    displayLine("Modem", screenState[MODEM_STATUS]);
    displayLine("Sensors", screenState[SENSOR_STATUS]);
    displayLine("SD", screenState[SD_STATUS]);
    displayLine("Payload", screenState[PAYLOAD_STATUS]);
    display.println(F("Wait = still working"));
  } else {
    uint8_t selected[11], count = 0;
    for (uint8_t i = 0; i < 11; ++i)
      if (!strcmp(sensorState[i], "failed") || !strcmp(sensorState[i], "check")) selected[count++] = i;
    bool needsHelp = count > 0;
    if (!needsHelp) for (uint8_t i = 0; i < 11; ++i) selected[count++] = i;
    display.println(needsHelp ? F("CHECK THESE SENSORS") : F("ODRADE SENSORS"));
    displayLine("GPS", screenState[GPS_STATUS]);
    displayLine("Modem", screenState[MODEM_STATUS]);
    displayLine("Sensors", screenState[SENSOR_STATUS]);
    uint8_t page = (now / 2000) % ((count + 2) / 3);
    for (uint8_t row = 0; row < 3; ++row) {
      uint8_t n = page * 3 + row;
      if (n < count) displayLine(sensorNames[selected[n]], sensorState[selected[n]]);
      else display.println();
    }
    displayLine("Payload", screenState[PAYLOAD_STATUS]);
  }
  display.display();
}

bool setupDisplay() {
  displayReady = false;
  // Keep the project's existing SDA39/SCL40 bus; do not restart Wire.
  Wire.setClock(100000);
  delay(500); // OLED power-up settling, matching the successful test.
  bool found = false;
  for (uint8_t attempt = 1; attempt <= 3; ++attempt) {
    Serial.printf("Screen: I2C scan %u/3 on SDA%u/SCL%u.\n", attempt, I2C_SDA, I2C_SCL);
    uint8_t count = 0;
    for (uint8_t address = 1; address < 127; ++address) {
      Wire.beginTransmission(address);
      if (Wire.endTransmission() == 0) {
        Serial.printf("  I2C device: 0x%02X\n", address);
        ++count;
        if (address == OLED_ADDRESS) found = true;
      }
    }
    Serial.printf("Screen: %u I2C devices; OLED 0x3C %s.\n", count, found ? "found" : "not found");
    if (attempt < 3) delay(200);
  }
  if (!found) {
    Serial.println("Screen: OLED 0x3C missing. Check power and wiring; measurements will continue.");
    return false;
  }
  // Exact begin arguments from the working test. Wire is already initialized.
  displayReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS, true, false);
  if (!displayReady) {
    Serial.println("Screen: could not start; readings will still be saved.");
    return false;
  }

  screenDirty = true;
  refreshDisplay();
  Serial.printf("Screen: SSD1306 128x64 initialized at 0x%02X; startup text sent.\n", OLED_ADDRESS);
  return true;
}

bool isDisplayReady() { return displayReady; }

void stopDisplay() { if (displayReady) display.ssd1306_command(SSD1306_DISPLAYOFF); }
