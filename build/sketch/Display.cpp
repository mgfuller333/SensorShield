#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/Display.cpp"
#include "Display.h"

// Follows ssd1306_128x64_i2c: clear, set text style, print, display.
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

const uint8_t SCREEN_WIDTH = 128;
const uint8_t SCREEN_HEIGHT = 64;
const int8_t OLED_RESET = -1;
const uint8_t OLED_ADDRESS = 0x3C;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayReady = false;

bool setupDisplay() {
  Wire.beginTransmission(OLED_ADDRESS);
  if (Wire.endTransmission() != 0) {
    Serial.println("Screen: not detected; readings will still be saved.");
    return false;
  }
  displayReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  if (!displayReady) {
    Serial.println("Screen: could not start; readings will still be saved.");
    return false;
  }

  display.clearDisplay();
  display.display();
  Serial.println("Screen: ready.");
  return true;
}

void showDisplay(const DeviceStatus &status) {
  if (!displayReady) {
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.print(F("Status: "));
  display.println(status.setup);
  display.print(F("Modem: "));
  display.println(status.modem);
  display.print(F("GPS: "));
  display.println(status.gps);
  display.print(F("Power: "));
  display.println(status.power);
  display.print(F("Air: "));
  display.println(status.air);
  display.print(F("BMS: "));
  display.println(status.bms);

  display.display();
}

bool isDisplayReady() { return displayReady; }

void stopDisplay() { if (displayReady) display.ssd1306_command(SSD1306_DISPLAYOFF); }
