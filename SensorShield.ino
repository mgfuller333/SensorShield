#include <Wire.h>
#include <SPI.h>
#include "cellularHelper.h"
#include "gpsHelper.h"
#include "bsec2.h" // Bosch BSEC2 library for BME688
#include "bme68x_datalogger.h"
#include "bsec_datalogger.h"
#include "label_provider.h"
#include "led_controller.h"
#include "batteryHelper.h"
#include "utils.h"
#include "sensor_manager.h"
#include "commMux.h" // Communication multiplexer for SPI/I2C
#include <ArduinoJson.h>
#include <inttypes.h>
#include <WalterModem.h>
#include <esp_mac.h>
#include <Adafruit_LTR329_LTR303.h>
#include <HardwareSerial.h>
#include <Adafruit_MPL3115A2.h>
#include <Wire.h>

#include <esp_sleep.h>
#include "esp_wifi.h"
#include "esp_bt.h"

// e.g. A0 on many ESP32 boards is ADC1_CHANNEL_6

#define LED_PIN 42 // Use a free IO pin for LED (example IO2 or any other free)

// --- these retain GPS Values across deep sleep! ---
// RTC_DATA_ATTR double savedLatitude = 0.0;
// RTC_DATA_ATTR double savedLongitude = 0.0;
// RTC_DATA_ATTR double savedAltitude = 0.0;
// RTC_DATA_ATTR uint8_t savedSatCount = 0;
// RTC_DATA_ATTR WalterModemGNSSFix _lastGNSSFix;

//--INA Comm Pin Definitions--//
constexpr uint8_t CHAN_INA = 0; // same as whichever port the INA219s are wired to

//--End of INA Comm Pin Definitions--//

/**
 * @brief The serial interface to talk to the modem.
 */
#define ModemSerial Serial2

//------Start of BME Initialization-------------//

/* Macros used */
/*! FILE_DATA_READ_SIZE determines the size of the data to be read from the file */
#define FILE_DATA_READ_SIZE UINT16_C(400)

/*!
 * @brief : This function is called by the BSEC library when a new output is available
 *
 * @param[in] input 	: BME68X data
 * @param[in] outputs	: BSEC output data
 */
void bsecCallBack(const bme68x_data input, const bsecOutputs outputs);

BatteryHelper powerReader; // defaults: 0x40 for battery, 0x41 for solar panel

Bsec2 bsec; // BSEC2 instance for the BME688

// Communication multiplexer configuration for the sensor (use sensor index 0)
commMux commConfig;

// Memory for the BSEC instance (size defined in bsec2.h)
uint8_t bsecInstanceMem[BSEC_INSTANCE_SIZE];

/* Helper functions declarations */
/**
   @brief : This function toggles the led when a fault was detected
*/
void errLeds(void);

/**
   @brief : This function is called by the BSEC library when a new output is available
   @param[in] input     : BME68X sensor data before processing
   @param[in] outputs   : Processed BSEC BSEC output data
   @param[in] bsec      : Instance of BSEC2 calling the callback
*/
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);

/* Create an array of objects of the class Bsec2 */
Bsec2 envSensor[NUM_OF_SENS];
commMux communicationSetup[NUM_OF_SENS];
uint8_t bsecMemBlock[NUM_OF_SENS][BSEC_INSTANCE_SIZE];
uint8_t sensor = 0;

/*!
 * @brief : This function handles sensor manager and BME68X datalogger configuration
 *
 * @param[in] bmeExtension : reference to the bmeconfig file extension
 *
 * @return  Application return code
 */
demo_ret_code configure_sensor_logging(const String &bme_extension);

/*!
 * @brief : This function handles the read configuration File
 *
 * @param[in] configFile	: reference to the config File Name
 *
 * @return  Application return code
 */
demo_ret_code config_file_read(const String &config_file);

/*!
 * @brief : This function handles BSEC datalogger configuration
 *
 * @param[in] aiConfigFile		: reference to the AI configuration file extension
 *
 * @param[in] bsecConfigFile	: reference to the BSEC configuration string file extension
 *
 * @param[inout] bsecConfigStr	: pointer to the BSEC configuration string
 *
 * @param[in] sensorNum			: selected sensor number
 *
 * @return  Application return code
 */
demo_ret_code configure_bsec_logging(const String &ai_config_file, const String &bsec_config_file,
                                     uint8_t bsec_config_str[BSEC_MAX_PROPERTY_BLOB_SIZE], uint8_t sensor_num);

/*!
 * @brief : This function handles collecting, sending sensor raw data via ble and data logging
 *
 * @param[in] sensNum :	sensor number to collect and log the data
 *
 * @return  a Bosch return code
 */
demo_ret_code log_sensor_data(uint8_t sens_num);

/*!
 * @brief : This function receive the system time (in unix epoch time format) from serial port and update into RTC time
 *
 * @return  void
 */
void read_sys_time();

uint8_t bsec_config[BSEC_MAX_PROPERTY_BLOB_SIZE];
char config_file_data[FILE_DATA_READ_SIZE];
Bsec2 bsec2[NUM_OF_SENS];
// bleController  			bleCtlr(bleMessageReceived);
labelProvider labelPvr;
ledController ledCtlr;
sensorManager sensorMgr;
bme68xDataLogger bme68xDlog;
bsecDataLogger bsecDlog;
demo_ret_code ret_code;
uint8_t selected_sensor;
String bme68x_conf_file_name, bsec_conf_file_name, ai_conf_file_name, config_file_name, label_file_name;
demo_app_mode app_mode, current_app_mode;
gas_label label;
bool is_bme68x_conf_available, is_bsec_conf_available, is_ai_conf_available, is_label_info_available;
commMux comm[NUM_OF_SENS];
uint8_t bsec_mem_block[NUM_OF_SENS][BSEC_INSTANCE_SIZE];

uint8_t sensor_num;
uint32_t ground_truth;
static uint8_t buff_count = 0;
static bsecDataLogger::sensor_io_data buff[NUM_OF_SENS];

uint32_t received_sys_time = 0;

//--End of BME Initialization--//

/**
 * @brief Set up the system.
 *
 * This function will set up the system and initialize the modem.
 *
 * @return None.
 */

void setup()
{

  Serial.begin(115200);
  delay(300);

  // if(!modem.configGNSS()) {
  //   Serial.print("Could not configure the GNSS subsystem\r\n");
  //   delay(1000);
  //   ESP.restart();
  //   return;
  // }

  // Set up the LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Start LED off
  digitalWrite(LED_PIN, HIGH); // Turn LED ON

  //------End of Modem Initialization------//

  //---------BME 688 Setup-----------------//
  /* Desired subscription list of BSEC2 outputs */
  bsecSensor sensorList[] = {
      BSEC_OUTPUT_IAQ,
      BSEC_OUTPUT_RAW_TEMPERATURE,
      BSEC_OUTPUT_RAW_PRESSURE,
      BSEC_OUTPUT_RAW_HUMIDITY,
      BSEC_OUTPUT_RAW_GAS,
      BSEC_OUTPUT_STABILIZATION_STATUS,
      BSEC_OUTPUT_RUN_IN_STATUS,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
      BSEC_OUTPUT_STATIC_IAQ,
      BSEC_OUTPUT_CO2_EQUIVALENT,
      BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
      BSEC_OUTPUT_GAS_PERCENTAGE,
      BSEC_OUTPUT_COMPENSATED_GAS};

  // Initialize I2C bus for the expander (commMux)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Initialize SPI bus with custom pin assignments
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  Serial.println("Wire and SPI set");

  delay(50);

  commMuxBegin(Wire, SPI);

  Serial.println("Wire and SPI initialized");
  delay(100);

  while (!Serial)
    delay(10);

  for (uint8_t i = 0; i < NUM_OF_SENS; i++)
  {
    /* Sets the Communication interface for the sensors */
    Serial.println("Setting up sensor:");
    Serial.println(i);

    communicationSetup[i] = commMuxSetConfig(Wire, SPI, i, communicationSetup[i]);

    //   /* Assigning a chunk of memory block to the bsecInstance */
    envSensor[i].allocateMemory(bsecMemBlock[i]);
    Serial.println("allocating memory:");
    Serial.println(i);

    //   /* Initialize the library and interfaces */
    if (!envSensor[i].begin(BME68X_SPI_INTF, commMuxRead, commMuxWrite, commMuxDelay, &communicationSetup[i]))
    {
      Serial.println("Failed to init comms to sensor outputs");
    }

    Serial.println("beginning sensor comm:");
    Serial.println(i);
    /*
     *	The default offset provided has been determined by testing the sensor in LP and ULP mode on application board 3.0
     *	Please update the offset value after testing this on your product
     */
    if (SAMPLE_RATE == BSEC_SAMPLE_RATE_ULP)
    {
      envSensor[i].setTemperatureOffset(TEMP_OFFSET_ULP);
    }
    else if (SAMPLE_RATE == BSEC_SAMPLE_RATE_LP)
    {
      envSensor[i].setTemperatureOffset(TEMP_OFFSET_LP);
    }

    //     /* Subscribe to the desired BSEC2 outputs */
    if (!envSensor[i].updateSubscription(sensorList, ARRAY_LEN(sensorList), SAMPLE_RATE))
    {
      Serial.println("Failed to sub to sensor outputs");
    }

    Serial.println("subscribing to sensor:");
    Serial.println(i);

    //     /* Whenever new data is available call the newDataCallback function */
    envSensor[i].attachCallback(newDataCallback);

    Serial.println("adding callback to sensor:");
    Serial.println(i);
  }

  Serial.println("BSEC library version " +
                 String(envSensor[0].version.major) + "." + String(envSensor[0].version.minor) + "." + String(envSensor[0].version.major_bugfix) + "." + String(envSensor[0].version.minor_bugfix));

  app_mode = DEMO_RECORDING_MODE;
  // current_app_mode = DEMO_RECORDING_MODE;

  label = BSEC_NO_CLASS;

  /* Setting default sensor as sensor 0 to collect data */
  selected_sensor = NUM_BME68X_UNITS;

  // Initialize the battery helper
  // communicationSetup[CHAN_INA] = commMuxSetConfig(Wire, SPI, CHAN_INA, communicationSetup[CHAN_INA]);
  if (!powerReader.begin())
  {
    Serial.println(F("ERROR: INA219 init failed. Check wiring & addresses."));
    while (1)
      delay(10);
  }
  Serial.println(F("batteryHelper: sensors ready."));

  /* Initializes the SD and RTC module */
  Serial.println("init SD and RTC...");
  ret_code = utils::begin();
  Serial.println("SD and RTC init complete");

  if (ret_code >= EDK_OK)
  {
    is_bme68x_conf_available = utils::get_file_with_extension(bme68x_conf_file_name, BME68X_CONFIG_FILE_EXT);

    /* checks the availability of BME board configuration file */
    Serial.println("is_bme68x_conf_available...");
    if (is_bme68x_conf_available)
    {

      if (configure_sensor_logging(bme68x_conf_file_name) < EDK_OK)
      {
        Serial.println("EDK_SENSOR_INITIALIZATION_FAILED");
        ret_code = EDK_SENSOR_INITIALIZATION_FAILED;
      }
    }
    else
    {
      Serial.println("EDK_SENSOR_CONFIG_MISSING_ERROR");
      ret_code = EDK_SENSOR_CONFIG_MISSING_ERROR;
    }
  }

  if (ret_code < EDK_OK)
  {
    Serial.println("ret_code Good: Setting to DEMO_IDLE_MODE");
    current_app_mode = DEMO_IDLE_MODE;
    app_mode = DEMO_IDLE_MODE;
  }
  delay(5000);
  /* Modem initialization */
  if (modem.begin(&Serial2))
  {
    Serial.println("Modem initialization OK");

    Serial.println("Configured low power mode");
    // Wait 5 seconds
    digitalWrite(LED_PIN, LOW); // Turn LED OFF
    return;
  }
  else
  {
    Serial.println("Modem initialization ERROR");
    return;
  }
}

void loop()
{
  Serial.println("Entering loop...");
  /* Transmit the packet */

  //-----End Fetch Call-----//
  //-------Start GPS-------//

  /* Check clock and assistance data, update if required */

  // WalterModemGNSSFix gpsData = getGPSFix();

  // Save the fix to RTC memory
  // _lastGNSSFix = gpsData;
  // savedLatitude = gpsData.latitude;
  // savedLongitude = gpsData.longitude;
  // savedAltitude = gpsData.height;
  // savedSatCount = gpsData.satCount;

  // -------End GPS-------//

  // ledCtlr.update(ret_code);
  Serial.println("test1");
  WalterModemRsp rsp = {};
  Serial.println("test2");
  read_sys_time();
  Serial.println("test3");

  // Create a JSON document with the expected format as a single object
  DynamicJsonDocument jsonDoc(768);
  Serial.println("test4");                                    // Adjust size as needed
  jsonDoc["device"] = "ca108a0d-01be-4487-9866-94d5887a801b"; // Add device ID
  // jsonDoc["Timestamp"] = utils::get_rtc().now().timestamp() + "+00:00"; // Add global timestamp

  // Format location as "(lat,lon)" string
  Serial.println("test5");
  char locationStr[40];
  Serial.println("test6");
  // snprintf(locationStr, sizeof(locationStr), "(%.6f,%.6f)", savedLatitude, savedLongitude);

  // Insert formatted location string into JSON
  // jsonDoc["location"] = locationStr;
  // jsonDoc["altitude"] = savedAltitude;
  // jsonDoc["satelliteCount"] = savedSatCount;

  // grab battery voltage
  // Add battery readings
  Serial.println("test7");
  communicationSetup[0] = commMuxSetConfig(Wire, SPI, 0, communicationSetup[0]);
  Serial.println("test8");
  DualPowerData data = powerReader.readAllSensors();
  Serial.println("test  9");
  // jsonDoc["Battery BusVoltage"] = data.battery.busVoltage;
  // jsonDoc["Battery ShuntVoltage"] = data.battery.shuntVoltage;
  // jsonDoc["Battery Current_mA"] = data.battery.current_mA;
  // jsonDoc["Battery Power_mW"] = data.battery.power_mW;

  // // Add solar readings
  // jsonDoc["Solar BusVoltage"] = data.solarPanel.busVoltage;
  // jsonDoc["Solar ShuntVoltage"] = data.solarPanel.shuntVoltage;
  // jsonDoc["Solar Current_mA"] = data.solarPanel.current_mA;
  // jsonDoc["Solar Power_mW"] = data.solarPanel.power_mW;

  if (!envSensor[0].run())
  {
    Serial.println("Failed to run sensor");
    return;
  }
  Serial.println("sensor 0 running");

  // Retrieve sensor outputs
  const bsecOutputs *outputs = envSensor[0].getOutputs();
  if (outputs)
  {
    Serial.println("outputs true");
    for (uint8_t i = 0; i < outputs->nOutputs; i++)
    {
      switch (outputs->output[i].sensor_id)
      {
      case BSEC_OUTPUT_RAW_PRESSURE:
        jsonDoc["pressure"] = (float)outputs->output[i].signal;
        Serial.printf("Pressure: %.2f hPa\n", (float)outputs->output[i].signal);
        // jsonDoc["RawPressure"] = (float)outputs->output[i].signal;
        break;
      case BSEC_OUTPUT_RAW_TEMPERATURE:
        // jsonDoc["temperature"] = (float)outputs->output[i].signal;
        Serial.printf("Temperature: %.2f °C\n", (float)outputs->output[i].signal);
        // jsonDoc["RawTemperature"] = (float)outputs->output[i].signal;
        break;
      case BSEC_OUTPUT_RAW_HUMIDITY:
        // jsonDoc["humidity"] = (float)outputs->output[i].signal;
        Serial.printf("Humidity: %.2f %%\n", (float)outputs->output[i].signal);
        // jsonDoc["RawHumidity"] = (float)outputs->output[i].signal;
        break;
      case BSEC_OUTPUT_RAW_GAS:
        // jsonDoc["RawGasResistance"] = (float)outputs->output[i].signal;
        Serial.printf("Gas Resistance: %.2f Ohms\n", (float)outputs->output[i].signal);
        break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
        jsonDoc["SensorCompensatedTemperature"] = (float)outputs->output[i].signal;
        break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
        jsonDoc["SensorCompensatedHumidity"] = (float)outputs->output[i].signal;
        break;
      case BSEC_OUTPUT_COMPENSATED_GAS:
        jsonDoc["SensorCompensatedGasResistance"] = (float)outputs->output[i].signal;
        break;
      case BSEC_OUTPUT_IAQ:
        jsonDoc["IAQ"] = (float)outputs->output[i].signal;
        jsonDoc["IAQAccuracy"] = (int)outputs->output[i].accuracy;
        break;
      case BSEC_OUTPUT_STATIC_IAQ:
        jsonDoc["StaticIAQ"] = (float)outputs->output[i].signal;
        break;
      case BSEC_OUTPUT_CO2_EQUIVALENT:
        jsonDoc["CO2Equivalents"] = (float)outputs->output[i].signal;
        break;
      case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
        jsonDoc["BVOCEquivalents"] = (float)outputs->output[i].signal;
        break;
      case BSEC_OUTPUT_STABILIZATION_STATUS:
        // jsonDoc["StabilizationTimeStatus"] = (int)outputs->output[i].signal;
        break;
      case BSEC_OUTPUT_RUN_IN_STATUS:
        // jsonDoc["RunInStatus"] = (int)outputs->output[i].signal;
        break;
      // case BSEC_OUTPUT_GAS_PERCENTAGE:
      //   jsonDoc["GasPercentage"] = (float)outputs->output[i].signal;
      //   break;
      // case BSEC_OUTPUT_GAS_ESTIMATE_1:
      //   jsonDoc["GasScanResult1"] = (float)outputs->output[i].signal;
      //   break;
      // case BSEC_OUTPUT_GAS_ESTIMATE_2:
      //   jsonDoc["GasScanResult2"] = (float)outputs->output[i].signal;
      //   break;
      // case BSEC_OUTPUT_GAS_ESTIMATE_3:
      //   jsonDoc["GasScanResult3"] = (float)outputs->output[i].signal;
      //   break;
      // case BSEC_OUTPUT_GAS_ESTIMATE_4:
      //   jsonDoc["GasScanResult4"] = (float)outputs->output[i].signal;
      //   break;
      case BSEC_OUTPUT_RAW_GAS_INDEX:
        jsonDoc["RawGasIndex"] = (int)outputs->output[i].signal;
        break;
      // case BSEC_OUTPUT_REGRESSION_ESTIMATE_1:
      //   jsonDoc["RegressionEstimate1"] = (float)outputs->output[i].signal;
      //   break;
      // case BSEC_OUTPUT_REGRESSION_ESTIMATE_2:
      //   jsonDoc["RegressionEstimate2"] = (float)outputs->output[i].signal;
      //   break;
      // case BSEC_OUTPUT_REGRESSION_ESTIMATE_3:
      //   jsonDoc["RegressionEstimate3"] = (float)outputs->output[i].signal;
      //   break;
      // case BSEC_OUTPUT_REGRESSION_ESTIMATE_4:
      //   jsonDoc["RegressionEstimate4"] = (float)outputs->output[i].signal;
      //   break;
      default:
        Serial.printf("Unknown sensor output ID: %d\n", outputs->output[i].sensor_id);
        break;
      }
    }
  }
  if (!lteConnect())
  {
    Serial.printf("failed and restarting....");
    ESP.restart();
  }
  else
  {
    configureLowPowerModem();
    Serial.println("Configured Lower Power mode...");
  }

  Serial.println("Trying to send data...");

  char jsonBuffer[512];
  serializeJson(jsonDoc, jsonBuffer);
  if (jsonDoc.containsKey("SensorCompensatedTemperature") && !jsonDoc["SensorCompensatedTemperature"].isNull() && jsonDoc["SensorCompensatedTemperature"].as<float>() != 0.0)
  {
    Serial.printf("sending...");
    if (sendSensorData(jsonBuffer))
    {
      Serial.printf("sent and waiting....");
      digitalWrite(LED_PIN, HIGH); // Turn LED ON
      delay(5);                    // Wait 5 seconds
      digitalWrite(LED_PIN, LOW);  // Turn LED OFF
      digitalWrite(LED_PIN, HIGH); // Turn LED ON
      delay(5);                    // Wait 5 seconds
      digitalWrite(LED_PIN, LOW);  // Turn LED OFF
      digitalWrite(LED_PIN, HIGH); // Turn LED ON
      delay(4000);                 // Wait 5 seconds
      digitalWrite(LED_PIN, LOW);  // Turn LED OFF
      if (!lteDisconnect())
      {
        Serial.printf("LTE disconneting.... and waiting...");
        ESP.restart();
      }
      delay(1000); // 5 minutes
    }
    else
    {
      Serial.println("Failed to send sensor data");
    }
    Serial.println("Going to deep sleep...");

    disableWifiAndBluetooth();
    enterLightSleep(800);
  }
  else
  {
    Serial.println("No valid SensorCompensatedTemperature data - skipping send");
  }
}
demo_ret_code log_sensor_data(uint8_t sens_num)
{

  demo_ret_code ret;
  bme68x_data *sensor_data[3];

  /* Returns the selected sensor address */
  bme68x_sensor *sensor = sensorMgr.get_sensor(sens_num);

  /* Retrieves the selected sensor data */
  ret = sensorMgr.collect_data(sens_num, sensor_data);

  if (ret < EDK_OK)
  {
    StaticJsonDocument<1024> jsonDoc;
    JsonObject errorObj = jsonDoc.createNestedObject("errorCode");

    errorObj["sensor_number"] = sens_num;
    // errorObj["error_code"] = bleController::SENSOR_READ_ERROR;

    // bleCtlr.send_notification(jsonDoc);
  }
  else
  {

    for (const auto data : sensor_data)
    {

      if (data != nullptr)
      {
        bme68x_data raw_data;
        raw_data.temperature = data->temperature;
        raw_data.pressure = (data->pressure * 0.01f);
        raw_data.humidity = data->humidity;
        raw_data.gas_resistance = data->gas_resistance;
        raw_data.gas_index = data->gas_index;

        /* sends the sensor raw data via ble */
        // ble_notify_bme68x_data(raw_data, sens_num);
        /* Writes the sensor data to the current log file */
        ret = bme68xDlog.write_sensor_data(&sens_num, &sensor->id, &sensor->mode, data,
                                           &sensor->scan_cycle_index, label, ret);

        /* Increments scanCycleIndex by one, if cycle completes */
        if (raw_data.gas_index == sensor->heater_profile.length - 1)
        {
          sensor->scan_cycle_index += 1;

          if (sensor->scan_cycle_index > sensor->heater_profile.nb_repetitions)
          {
            sensor->scan_cycle_index = 1;
          }
        }
      }
    }
  }
  return ret;
}

bme68x_data bsecCallBack(uint8_t sens_num)
{
  demo_ret_code ret;
  bme68x_data *sensor_data[3];
  bme68x_data raw_data = {}; // Initialize to prevent undefined behavior

  /* Returns the selected sensor address */
  bme68x_sensor *sensor = sensorMgr.get_sensor(sens_num);

  /* Retrieves the selected sensor data */
  ret = sensorMgr.collect_data(sens_num, sensor_data);

  if (ret < EDK_OK)
  {
    StaticJsonDocument<1024> jsonDoc;
    JsonObject errorObj = jsonDoc.createNestedObject("errorCode");

    errorObj["sensor_number"] = sens_num;
    // errorObj["error_code"] = bleController::SENSOR_READ_ERROR;

    // bleCtlr.send_notification(jsonDoc);
  }
  else
  {
    for (const auto data : sensor_data)
    {
      if (data != nullptr)
      {

        raw_data.temperature = data->temperature;
        raw_data.pressure = (data->pressure * 0.01f);
        raw_data.humidity = data->humidity;
        raw_data.gas_resistance = data->gas_resistance;
        raw_data.gas_index = data->gas_index;

        /* sends the sensor raw data via ble */
        // ble_notify_bme68x_data(raw_data, sens_num);
        /* Writes the sensor data to the current log file */
        ret = bme68xDlog.write_sensor_data(&sens_num, &sensor->id, &sensor->mode, data,
                                           &sensor->scan_cycle_index, label, ret);

        /* Increments scanCycleIndex by one, if cycle completes */
        if (raw_data.gas_index == sensor->heater_profile.length - 1)
        {
          sensor->scan_cycle_index += 1;

          if (sensor->scan_cycle_index > sensor->heater_profile.nb_repetitions)
          {
            sensor->scan_cycle_index = 1;
          }
        }
      }
    }
  }
  Serial.print("Raw data temp: ");
  Serial.println(raw_data.temperature);
  Serial.print("Raw data %Rh: ");
  Serial.println(raw_data.humidity);
  Serial.print("Raw data press: ");
  Serial.println(raw_data.pressure);
  Serial.print("Raw data gass: ");
  Serial.println(raw_data.gas_resistance);
  return raw_data; // Now returns raw_data correctly
}

void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec)
{
  if (!outputs.nOutputs)
  {
    return;
  }

  // Serial.println("BSEC outputs:\n\tSensor num = " + String(sensor));
  // Serial.println("\tTime stamp = " + String((int) (outputs.output[0].time_stamp / INT64_C(1000000))));
  for (uint8_t i = 0; i < outputs.nOutputs; i++)
  {
    const bsecData output = outputs.output[i];
    switch (output.sensor_id)
    {
    case BSEC_OUTPUT_IAQ:
      // Serial.println("\tIAQ = " + String(output.signal));
      // Serial.println("\tIAQ accuracy = " + String((int) output.accuracy));
      break;
    case BSEC_OUTPUT_RAW_TEMPERATURE:
      // Serial.println("\tTemperature = " + String(output.signal));
      break;
    case BSEC_OUTPUT_RAW_PRESSURE:
      // Serial.println("\tPressure = " + String(output.signal));
      break;
    case BSEC_OUTPUT_RAW_HUMIDITY:
      // Serial.println("\tHumidity = " + String(output.signal));
      break;
    case BSEC_OUTPUT_RAW_GAS:
      // Serial.println("\tGas resistance = " + String(output.signal));
      break;
    case BSEC_OUTPUT_STABILIZATION_STATUS:
      // Serial.println("\tStabilization status = " + String(output.signal));
      break;
    case BSEC_OUTPUT_RUN_IN_STATUS:
      // Serial.println("\tRun in status = " + String(output.signal));
      break;
    case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
      // Serial.println("\tCompensated temperature = " + String(output.signal));
      break;
    case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
      // Serial.println("\tCompensated humidity = " + String(output.signal));
      break;
    case BSEC_OUTPUT_STATIC_IAQ:
      // Serial.println("\tStatic IAQ = " + String(output.signal));
      break;
    case BSEC_OUTPUT_CO2_EQUIVALENT:
      // Serial.println("\tCO2 Equivalent = " + String(output.signal));
      break;
    case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
      // Serial.println("\tbVOC equivalent = " + String(output.signal));
      break;
    case BSEC_OUTPUT_GAS_PERCENTAGE:
      // Serial.println("\tGas percentage = " + String(output.signal));
      break;
    case BSEC_OUTPUT_COMPENSATED_GAS:
      // Serial.println("\tCompensated gas = " + String(output.signal));
      break;

    default:
      break;
    }
  }
}

demo_ret_code configure_sensor_logging(const String &bme_config_file)
{
  demo_ret_code ret = sensorMgr.begin(bme_config_file);

  if (ret >= EDK_OK)
  {
    ret = bme68xDlog.begin(bme_config_file);
  }
  return ret;
}

demo_ret_code configure_bsec_logging(const String &ai_config_file, const String &bsec_config_file,
                                     uint8_t bsec_config_str[BSEC_MAX_PROPERTY_BLOB_SIZE], uint8_t sensor_num)
{
  memset(bsec_config_str, 0, BSEC_MAX_PROPERTY_BLOB_SIZE);
  demo_ret_code ret = bsecDlog.begin(ai_config_file, bsec2[0].version, sensor_num);

  if (ret >= EDK_OK)
  {
    ret = utils::get_bsec_config(bsec_config_file, bsec_config_str);
  }
  return ret;
}

demo_ret_code config_file_read(const String &config_file)
{
  demo_ret_code ret;
  static bool first_time = true;
  StaticJsonDocument<1024> jsonDoc;

  while ((ret = utils::read_file(config_file, FILE_DATA_READ_SIZE, config_file_data)) == EDK_OK)
  {
    if (first_time)
    {
      if (config_file == BME68X_LABEL_INFO_FILE_EXT)
      {
        // jsonDoc["getlabelinfo"] = bleController::CMD_VALID;
      }
      else
      {
        // jsonDoc["readconfig"] = bleController::CMD_VALID;
      }
      first_time = false;
    }

    jsonDoc.clear();               // Clear any existing data
    jsonDoc.add(config_file_data); // Add the new data (make sure it's in the right format)

    // bleCtlr.send_notification(jsonDoc);  // Send the JSON document (uncomment to use)
  }
  first_time = true;
  return ret;
}

void read_sys_time()
{
  while (Serial.available())
  {
    String rx_msg;

    rx_msg = Serial.readStringUntil('\n');
    received_sys_time = rx_msg.toInt();

    utils::get_rtc().adjust(DateTime(received_sys_time));
  }
}

JsonObject serialPrintEnvSensorJson(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec)
{
  StaticJsonDocument<1024> jsonDoc;
  JsonObject sensorObj = jsonDoc.createNestedObject("sensor_data");

  if (!outputs.nOutputs)
  {
    sensorObj["status"] = "No output data available";
  }
  else
  {
    sensorObj["timestamp"] = (int)(outputs.output[0].time_stamp / INT64_C(1000000));
    JsonArray sensorDataArray = sensorObj.createNestedArray("data");

    for (uint8_t i = 0; i < outputs.nOutputs; i++)
    {
      const bsecData output = outputs.output[i];
      JsonObject dataObj = sensorDataArray.createNestedObject();

      switch (output.sensor_id)
      {
      case BSEC_OUTPUT_CO2_EQUIVALENT:
        dataObj["type"] = "CO2 Equivalent";
        dataObj["value"] = output.signal;
        break;
      case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
        dataObj["type"] = "bVOC Equivalent";
        dataObj["value"] = output.signal;
        break;
      case BSEC_OUTPUT_GAS_PERCENTAGE:
        dataObj["type"] = "Gas Percentage";
        dataObj["value"] = output.signal;
        break;
      case BSEC_OUTPUT_COMPENSATED_GAS:
        dataObj["type"] = "Compensated Gas";
        dataObj["value"] = output.signal;
        break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
        dataObj["type"] = "Temperature";
        dataObj["value"] = output.signal;
        break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
        dataObj["type"] = "Humidity";
        dataObj["value"] = output.signal;
        break;
      case BSEC_OUTPUT_RAW_PRESSURE:
        dataObj["type"] = "Pressure";
        dataObj["value"] = output.signal;
        break;
      default:
        continue; // Skip any other sensor outputs
      }

      dataObj["accuracy"] = output.accuracy;
    }
  }

  /* Print JSON to Serial */
  Serial.println();
  serializeJsonPretty(jsonDoc, Serial);
  Serial.println();

  return sensorObj; // Return the JSON object
}

void disableWifiAndBluetooth()
{
  esp_wifi_stop();
  esp_bt_controller_disable();
}

void configureLowPowerModem()
{
  // Enable Power Saving Mode (PSM)
  // T3412 = "00100001" (1 hour TAU), T3324 = "00000101" (10s active time)
  // 30s TAU, 10s Active time
  modem.configPSM(WALTER_MODEM_PSM_ENABLE, "00011110", "00000101");
}

void enterLightSleep(uint32_t seconds)
{
  // ESP32 timer wakeup
  esp_sleep_enable_timer_wakeup(seconds * 1000000ULL);

  // Walter library manages RTS line before/after sleep
  modem.sleep(seconds, true); // true = light sleep
}