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
#include <ESP32Time.h>
#include <esp_sleep.h>
#include "esp_wifi.h"
#include "esp_bt.h"
#include "SparkFun_BMV080_Arduino_Library.h"
// e.g. A0 on many // ESP32Time rtc;
ESP32Time rtc(-6 * 3600); // offset in seconds GMT+1ESP32 boards is ADC1_CHANNEL_6

#define LED_PIN 42 // Use a free IO pin for LED (example IO2 or any other free)
constexpr size_t BUFFER_SIZE = 20;
// --- these retain GPS Values across deep sleep! ---
RTC_DATA_ATTR double savedLatitude = 0.0;
RTC_DATA_ATTR double savedLongitude = 0.0;
RTC_DATA_ATTR double savedAltitude = 0.0;
RTC_DATA_ATTR uint8_t savedSatCount = 0;
RTC_DATA_ATTR WalterModemGNSSFix _lastGNSSFix;
sensorManager sensorMgr;
// // ESP32Time rtc;
// ESP32Time rtc(-6 * 3600); // offset in seconds GMT+1

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

WalterModemGNSSFix gpsData;

DynamicJsonDocument payloadDoc(8192); // 16 KB on heap

// WalterModemRsp rsp = {};
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
  delay(5000);

  // Set up the LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Start LED off
  digitalWrite(LED_PIN, HIGH); // Turn LED ON
  delay(1000);                 // Wait 100 ms
  digitalWrite(LED_PIN, LOW);  // Start LED off

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

  commMuxBegin(Wire, SPI);

  Serial.println("Wire and SPI initialized");
  delay(100);

  for (uint8_t i = 0; i < NUM_OF_SENS; i++)
  {
    /* Sets the Communication interface for the sensors */
    Serial.println("Setting up sensor:");
    Serial.println(i);
    // Wait 5 ms

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

  if (!powerReader.begin())
  {
    Serial.println(F("ERROR: INA219 init failed. Check wiring & addresses."));
    delay(10);
  }
  Serial.println(F("batteryHelper: sensors ready."));

  /* Initializes the SD and RTC module */
  Serial.println("init SD ...");
  ret_code = utils::begin();
  Serial.println("SD init complete");

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

  if (!sensorMgr.initialize_pm_sensor())
  {
    Serial.println("Failed to initialize PM sensor");
    return;
  }
  else
  {
    Serial.println("PM sensor initialized successfully");
  }

  /* Modem initialization */
  if (modemSetup())
  {
    Serial.println("Modem initialization OK");

    delay(2000);
  }
  else
  {

    Serial.println("Modem initialization ERROR");

    return;
  }

  //-------Start GPS-------//

  /* Check clock and assistance data, update if required */
  Serial.println("Initialize GPS");
  WalterModemGNSSFix gpsData = getGPSFix();

  //-------Start GPS-------//

  // Save the fix to RTC memory
  _lastGNSSFix = gpsData;
  savedLatitude = gpsData.latitude;
  savedLongitude = gpsData.longitude;
  savedAltitude = gpsData.height;
  savedSatCount = gpsData.satCount;
  Serial.printf("  retrieved Longitude: %.6f\n", gpsData.longitude);
  Serial.printf("  retrieved Satcount: %d\n", gpsData.satCount);
}

void loop()
{

  int sampleCount = 0; // Initialize sample count
  // StaticJsonDocument<768> sampleDoc;
  // Initialize or clear the JSON document
  payloadDoc.clear();
  JsonArray sensors = payloadDoc.to<JsonArray>();

  // JsonArray sensors = root.createNestedObject("sensors");

  Serial.println("Entering loop...");

  // Collect samples until buffer is full
  while (sampleCount < BUFFER_SIZE)
  {

    Serial.println("enable modem response...");

    // read_sys_time();

    // Create a JSON document with the expected format as a single object

    JsonObject sensorObj;

    // Grab battery and solar readings
    Serial.println("Configuring communication for INA219...");
    communicationSetup[0] = commMuxSetConfig(Wire, SPI, 0, communicationSetup[0]);
    Serial.println("Reading battery and solar sensors...");
    DualPowerData data = powerReader.readAllSensors();
    Serial.println("Initializing runInStatus variable...");
    uint8_t runInStatus = -1;
    Serial.println("Reading PM sensor data...");
    SparkFunBMV080 pmData = sensorMgr.get_pm_data();
    Serial.println("PM sensor data read complete.");

    // Serial.printf("sensor obj Timestamp: %s\n", timestamp);

    // Add to JSON sensor object

    // Retrieve sensor outputs
    sensorObj = sensors.createNestedObject();
    for (uint8_t i = 0; i < NUM_OF_SENS; i++)
    {
      if (!envSensor[i].run())
      {
        Serial.printf("Sensor %d failed to run\n", i);

        continue;
      }

      Serial.printf("Sensor %d is to running\n", i);
    }

    for (uint8_t i = 0; i < NUM_OF_SENS; i++)
    {

      const bsecOutputs *outputs = envSensor[i].getOutputs();
      Serial.printf("trying to grab outputs from sensor: %d ", i);

      if (outputs)
      {
        Serial.print("outputs true for sensor: ");
        Serial.println(i);
        for (uint8_t t = 0; t < outputs->nOutputs; t++)

        {
          switch (outputs->output[t].sensor_id)
          {
          case BSEC_OUTPUT_RUN_IN_STATUS:
            runInStatus = outputs->output[t].signal;
            Serial.print("runInStatus");
            Serial.print(runInStatus);
            break;

          case BSEC_OUTPUT_RAW_PRESSURE:
            sensorObj[String("pres_") + String(0)] = (float)outputs->output[t].signal;
            // Serial.println(String("pres_") + String(i));

            // Serial.printf("pres_%d: %.2f hPa\n", i, sensorObj[String("pres_") + String(i)].as<float>());
            // Serial.printf("Pressure: %.2f hPa\n", (float)outputs->output[t].signal);
            break;
          case BSEC_OUTPUT_RAW_TEMPERATURE:
            // sensorObj[String("temp_") + String(i)] = (float)outputs->output[i].signal;
            // Serial.printf("Temperature: %.2f °C\n", (float)outputs->output[i].signal);
            break;
          case BSEC_OUTPUT_RAW_HUMIDITY:
            // sensorObj[String("hum_") + String(i)] = (float)outputs->output[i].signal;
            // Serial.printf("Humidity: %.2f %%\n", (float)outputs->output[i].signal);
            break;
          case BSEC_OUTPUT_RAW_GAS:
            sensorObj[String("gasR_") + String(i)] = (float)outputs->output[t].signal;
            // Serial.printf("Gas Resistance: %.2f Ohms\n", (float)outputs->output[t].signal);
            break;
          case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
            sensorObj[String("temp_") + String(i)] = (float)outputs->output[t].signal;
            break;
          case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
            sensorObj[String("hum_") + String(i)] = (float)outputs->output[t].signal;
            break;
          case BSEC_OUTPUT_COMPENSATED_GAS:
            sensorObj[String("com_gas") + String(i)] = (float)outputs->output[t].signal;
            break;
          case BSEC_OUTPUT_IAQ:
            sensorObj[String("iaq_") + String(i)] = (float)outputs->output[t].signal;
            sensorObj[String("iaqAcc_") + String(i)] = (float)outputs->output[t].accuracy;
            break;
          case BSEC_OUTPUT_STATIC_IAQ:
            // sensorObj[String("staq_") + String(i)] = (float)outputs->output[i].signal;
            break;
          case BSEC_OUTPUT_CO2_EQUIVALENT:
            sensorObj[String("co2_") + String(i)] = (float)outputs->output[t].signal;
            break;
          case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
            sensorObj[String("bvoc_") + String(i)] = (float)outputs->output[t].signal;
            break;
          case BSEC_OUTPUT_STABILIZATION_STATUS:
            // sensorObj[String("stabStat")] = (int)outputs->output[i].signal;
            break;
          case BSEC_OUTPUT_RAW_GAS_INDEX:
            // sensorObj[String("rgasIdx_") + String(i)] = (int)outputs->output[i].signal;
            break;
          default:
            // Serial.printf("Unknown sensor output ID: %d\n", outputs->output[t].sensor_id);
            break;
          }
        }
      }
      else
      {
        Serial.println("No valid data - skipping sample");
        // sensors.clear();
        // sensorObj.clear(); // Clear the sensor object
        delay(1000); // Brief delay before retrying
        continue;    // Skip to next iteration of while loop
      }

      delay(30);
      Serial.println("checking if data is populated...");
    }

    Serial.println("Data Populated");
    if (runInStatus > -1)
    {

      sensorObj["device"] = "ca108a0d-01be-4487-9866-94d5887a801b";
      // Format timestamp with milliseconds (e.g., "2025-06-24T05:19:45.123Z")
      char timestamp[40];

      // Get current time as formatted string (no ms)
      String baseTime = rtc.getTime("%Y-%m-%dT%H:%M:%S");

      // Get milliseconds from RTC
      unsigned long ms = rtc.getMillis();

      // Format timestamp with ms precision
      snprintf(timestamp, sizeof(timestamp), "%s.%03luZ", baseTime.c_str(), ms);

      // Assign to JSON object
      sensorObj["ts"] = timestamp;

      // Add sensor values to the JSON object
      char locationStr[50];
      snprintf(locationStr, sizeof(locationStr),
               "POINT(%.6f %.6f)",
               savedLongitude, // longitude first!
               savedLatitude); // then latitude
      sensorObj["loc"] = locationStr;
      sensorObj["alt"] = savedAltitude;
      sensorObj["satCnt"] = savedSatCount;

      sensorObj["pm10"] = pmData.PM10();
      sensorObj["pm2_5"] = pmData.PM25();
      sensorObj["pm1"] = pmData.PM1();

      sensorObj["batV"] = data.battery.busVoltage;
      sensorObj["batSV"] = data.battery.shuntVoltage;
      sensorObj["batC"] = data.battery.current_mA;
      sensorObj["batP"] = data.battery.power_mW;

      sensorObj["solV"] = data.solarPanel.busVoltage;
      sensorObj["solSV"] = data.solarPanel.shuntVoltage;
      sensorObj["solC"] = data.solarPanel.current_mA;
      sensorObj["solP"] = data.solarPanel.power_mW;

      sensorObj["bID"] = sampleCount; // Assign a unique ID for each sample

      sampleCount++;
      Serial.printf("sample count: %d\n", sampleCount);
      // Validate sample before adding to array
      Serial.println("Sensor Object:");
      serializeJson(sensorObj, Serial);
    }
  }
  Serial.println("Buffer full. Preparing to send data...");
  Serial.println("Sensor Array:");
  serializeJson(sensors, Serial);
  Serial.println();

  // Print the payloadArray to Serial

  // Buffer is full, send the data
  Serial.println("Buffer full. Sending 60-sample batch...");

  JsonObject testObj;
  lteConnect();

  for (int i = 0; i < BUFFER_SIZE; ++i)
  {
    Serial.println(String("sending parts... ") + String(i));
    char jsonBuffer[2000];
    testObj = sensors[i];
    serializeJson(testObj, jsonBuffer);
    Serial.println("serialize json parts...");
    if (sendSensorData(jsonBuffer))
    {
      Serial.println("sending data to cloud...");
      digitalWrite(LED_PIN, LOW); // Turn LED OFF
      delay(50);
      digitalWrite(LED_PIN, HIGH); // Turn LED ON
      delay(50);                   // Wait 5 ms
      digitalWrite(LED_PIN, LOW);  // Turn LED OFF
      delay(50);
      digitalWrite(LED_PIN, HIGH); // Turn LED ON
      delay(50);                   // Wait 5 ms
      digitalWrite(LED_PIN, LOW);  // Turn LED OFF
      delay(50);
      digitalWrite(LED_PIN, HIGH); // Turn LED ON
      delay(50);                   // Wait 4 seconds
      digitalWrite(LED_PIN, LOW);  // Turn LED OFF
    }
    else
    {
      Serial.println("Send failed.");
    }
  }

  // // Reset buffer and sample count
  sensors.clear();
  sampleCount = 0;

  delay(60000); // 10 min delay between batches
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
  Serial.println("trying to read time...");
  while (Serial.available())
  {
    String rx_msg;

    rx_msg = Serial.readStringUntil('\n');
    received_sys_time = rx_msg.toInt();

    utils::get_rtc().adjust(DateTime(received_sys_time));
  }

  Serial.println("read time...");
  return;
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

    JsonArray sensorDataArray = sensorObj.createNestedArray("sensor_data_array");

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