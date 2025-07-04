#include "batteryHelper.h"
#include <Wire.h>

BatteryHelper::BatteryHelper(uint8_t addrBattery, uint8_t addrSolar)
    : inaBattery(addrBattery),
      inaSolar(addrSolar)
{
}

bool BatteryHelper::begin()
{
    Serial.println("Initializing BatteryHelper...");

    // Static flag to ensure Wire1 is only initialized once
    static bool wireInitialized = false;
    if (!wireInitialized)
    {
        Wire1.begin(PowerLevel_SDA_PIN, PowerLevel_SCL_PIN, 100000); // e.g., GPIO 6, 7
        wireInitialized = true;
        Serial.println("Wire1 initialized.");
    }
    else
    {
        Serial.println("Wire1 already initialized.");
    }

    // Initialize INA219 sensors
    Serial.println("Initializing INA219 sensors...");
    if (!inaBattery.begin(&Wire1))
    {
        Serial.println("Failed to initialize INA219 for battery.");
        return false;
    }
    Serial.println("INA219 for battery initialized.");

    if (!inaSolar.begin(&Wire1))
    {
        Serial.println("Failed to initialize INA219 for solar panel.");
        return false;
    }
    Serial.println("INA219 for solar panel initialized.");

    // Calibrate sensors
    Serial.println("Calibrating INA219 sensors...");
    inaBattery.setCalibration_32V_1A();
    Serial.println("Battery sensor calibrated for 32V, 1A.");
    inaSolar.setCalibration_32V_1A();
    Serial.println("Solar panel sensor calibrated for 32V, 1A.");

    // Disable power save
    Serial.println("Disabling power save mode for sensors...");
    inaBattery.powerSave(false);
    inaSolar.powerSave(false);
    Serial.println("Sensors power save disabled.");

    Serial.println("BatteryHelper initialization complete.");
    return true;
}

DualPowerData BatteryHelper::readAllSensors()
{
    DualPowerData data;

    // Read battery sensor data
    data.battery.busVoltage = inaBattery.getBusVoltage_V();
    data.battery.shuntVoltage = inaBattery.getShuntVoltage_mV();
    data.battery.current_mA = inaBattery.getCurrent_mA();
    data.battery.power_mW = inaBattery.getPower_mW();

    // Serial.println("Battery Sensor Data:");
    // Serial.printf("  Bus Voltage:   %.2f V\n", data.battery.busVoltage);
    // Serial.printf("  Shunt Voltage: %.2f mV\n", data.battery.shuntVoltage);
    // Serial.printf("  Current:       %.2f mA\n", data.battery.current_mA);
    // Serial.printf("  Power:         %.2f mW\n\n", data.battery.power_mW);

    // Read solar panel sensor data
    data.solarPanel.busVoltage = inaSolar.getBusVoltage_V();
    data.solarPanel.shuntVoltage = inaSolar.getShuntVoltage_mV();
    data.solarPanel.current_mA = inaSolar.getCurrent_mA();
    data.solarPanel.power_mW = inaSolar.getPower_mW();

    // Serial.println("Solar Panel Sensor Data:");
    // Serial.printf("  Bus Voltage:   %.2f V\n", data.solarPanel.busVoltage);
    // Serial.printf("  Shunt Voltage: %.2f mV\n", data.solarPanel.shuntVoltage);
    // Serial.printf("  Current:       %.2f mA\n", data.solarPanel.current_mA);
    // Serial.printf("  Power:         %.2f mW\n", data.solarPanel.power_mW);

    // Optional: re-enable power save after reading
    inaBattery.powerSave(true);
    inaSolar.powerSave(true);

    return data;
}
