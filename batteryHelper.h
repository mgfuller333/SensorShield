#include <Arduino.h>
#include <Adafruit_INA219.h>

// I2C pins (for the expander controlling chip‐select)
#define PowerLevel_SDA_PIN 6
#define PowerLevel_SCL_PIN 7

struct PowerData
{
    float busVoltage;   // V
    float shuntVoltage; // mV
    float current_mA;   // mA
    float power_mW;     // mW
};

// Holds readings for battery and solar panel sensors
struct DualPowerData
{
    PowerData battery;
    PowerData solar;
};

class BatteryHelper
{
public:
    // addrBattery = battery sensor address, addrSolar = solar panel sensor address
    BatteryHelper(uint8_t addrBattery = 0x41, uint8_t addrSolar = 0x40);

    // Initialize both INA219 sensors
    bool begin();

    // Read both sensors in one call
    DualPowerData readAllSensors();

private:
    Adafruit_INA219 inaBattery;
    Adafruit_INA219 inaSolar;
};