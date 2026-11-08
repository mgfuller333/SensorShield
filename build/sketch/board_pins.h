#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/board_pins.h"
#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include <Arduino.h>

#define WALTER_3V3_EN 0 // Active LOW, matching the working INA3221 test.

const uint8_t I2C_SDA = 39;
const uint8_t I2C_SCL = 40;

const uint8_t SPI_SCK = 2;
const uint8_t SPI_MISO = 42;
const uint8_t SPI_MOSI = 41;

const uint8_t BME_COUNT = 8;
const uint8_t BME_CS[BME_COUNT] = {4, 6, 7, 15, 16, 17, 18, 8};

const uint8_t BMV080_CS = 1;
const uint8_t SD_CS = 38; // uSD_CS in the supplied Walter/socket schematics.

#endif // BOARD_PINS_H
