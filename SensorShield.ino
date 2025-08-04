#include <Wire.h>
#include <SPI.h>
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
#include "SparkFun_BMV080_Arduino_Library.h"
#include <ArduinoJson.h>    // <-- new
#include "cellularHelper.h" // <-- for lteConnect() + sendSensorData()
#include "PMS.h"
#define BUFFER_SIZE 4

// Define UART1 pins (example: TX = GPIO17, RX = GPIO18)

const int PMS_RX = 9;  // sensor TX → ESP32 RX
const int PMS_TX = 10; // sensor RX ← ESP32 TX (must be an output-capable pin)

// Use hardware UART2
PMS pms(Serial1); // Use Serial1 for PMS sensor
PMS::DATA pmData;

uint8_t sampleCount = 0;
uint8_t payloadCounter = 0;
DynamicJsonDocument payloadDoc(8192); // 16 KB on heap
JsonArray sensors = payloadDoc.to<JsonArray>();

// adjust these to your needs
constexpr uint8_t NUM_SENSORS = 8;

static uint8_t sensorStepCount = 0; // how many sensor callbacks in current sample

static JsonObject currentSampleObj; // points into `sensors`

#define LED_PIN 42 // Use a free IO pin for LED (example IO2 or any other free)

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

/**
   @brief : This function checks the BSEC status, prints the respective error code. Halts in case of error
   @param[in] bsec  : Bsec2 class object
*/
void checkBsecStatus(Bsec2 bsec);

BatteryHelper powerReader; // defaults: 0x40 for battery, 0x41 for solar panel

Bsec2 bsec; // BSEC2 instance for the BME688
Bsec2 envSensor[NUM_OF_SENS];
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
 * @brief : This function handles collecting, sending sensor raw data via ble and data logging
 *
 * @param[in] sensNum :	sensor number to collect and log the data
 *
 * @return  a Bosch return code
 */
demo_ret_code log_sensor_data(uint8_t sens_num);

void configureUltraLowPowerModem();
/*!
 * @brief : This function receive the system time (in unix epoch time format) from serial port and update into RTC time
 *
 * @return  void
 */
void read_sys_time();
bool pmsWakeStatus = false;
uint8_t bsec_config[BSEC_MAX_PROPERTY_BLOB_SIZE];
char config_file_data[FILE_DATA_READ_SIZE];
Bsec2 bsec2[NUM_OF_SENS];
// bleController  			bleCtlr(bleMessageReceived);
labelProvider labelPvr;
ledController ledCtlr;

bme68xDataLogger bme68xDlog;
bsecDataLogger bsecDlog;
demo_ret_code ret_code;

String bme68x_conf_file_name, bsec_conf_file_name, ai_conf_file_name, config_file_name, label_file_name;

gas_label label;
bool is_bme68x_conf_available, is_bsec_conf_available, is_ai_conf_available, is_label_info_available;

uint8_t bsec_mem_block[NUM_OF_SENS][BSEC_INSTANCE_SIZE];

commMux communicationSetup[NUM_OF_SENS];

uint8_t sensor_num;
uint32_t ground_truth;
static uint8_t buff_count = 0;
static bsecDataLogger::sensor_io_data buff[NUM_OF_SENS];

String deviceSetupStatus = "---";
String modemStatus = "---";
String gpsFixStatus = "---";
String payloadStatus = "---";
String packetsSent = "---";
String powerSensorStatus = "---";
String sd_clock_status = "---";
String BMEStatus = "---";

uint32_t received_sys_time = 0;

WalterModemGNSSFix gpsData;

//--End of BME Initialization--//

/**
 * @brief Set up the system.
 *
 * This function will set up the system and initialize the modem.
 *
 * @return None.
 */
int packetsSenTotal = 0;
void setup()
{
  deviceSetupStatus = "setup";
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, PMS_TX, PMS_RX); // Serial1 for PMS sensor, TX: GPIO10, RX: GPIO9
                                                   // Initialize I2C bus for the expander (commMux)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Initialize SPI bus with custom pin assignments
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  Serial.println("Wire and SPI set");

  commMuxBegin(Wire, SPI); //  Switch to passive mode
  if (!sensorMgr.initializeOLED())
  {
    Serial.println("OLED initialization failed");
  }
  else
  {
    Serial.println("OLED initialized successfully");
  }

  Serial.println("setting plantower to passive mode");
  pms.passiveMode();
  Serial.println("Waking up, wait 30 seconds for stable readings...");
  pms.wakeUp();
  pmsWakeStatus = true;

  sensorMgr.printDeviceStatus(
      deviceSetupStatus,
      modemStatus,
      gpsFixStatus,
      payloadStatus,
      packetsSent,
      sd_clock_status,
      powerSensorStatus,
      BMEStatus);
  delay(4000); // Wait for a second to stabilize the system
  // 1 Blink Set up the LED pin
  pinMode(LED_PIN, OUTPUT);

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
    envSensor[i].allocateMemory(bsec_mem_block[i]);

    Serial.println("allocating memory:");
    Serial.println(i);

    //   /* Initialize the library and interfaces */
    if (!envSensor[i].begin(BME68X_SPI_INTF, commMuxRead, commMuxWrite, commMuxDelay, &communicationSetup[i]))
    {
      Serial.println("Failed to init comms to sensor outputs");
      checkBsecStatus(envSensor[i]);

      BMEStatus = String(i) + "-Bad";
    }
    else
    {
      BMEStatus = String(i) + "-Good";
    }

    Serial.println("beginning sensor comm:");
    Serial.println(i);

    sensorMgr.printDeviceStatus(
        deviceSetupStatus,
        modemStatus,
        gpsFixStatus,
        payloadStatus,
        packetsSent,
        sd_clock_status,
        powerSensorStatus,
        BMEStatus);

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

  // Initialize PMS5003 sensor

  //  End of PMS5003 sensor initialization

  if (!powerReader.begin())
  {
    powerSensorStatus = "unhealthy";
    Serial.println("ERROR: INA219 init failed. Check wiring & addresses.");

    delay(10);
  }
  else
  {
    powerSensorStatus = "Healthy";
    Serial.println("batteryHelper: sensors ready.");
  }

  sensorMgr.printDeviceStatus(
      deviceSetupStatus,
      modemStatus,
      gpsFixStatus,
      payloadStatus,
      packetsSent,
      sd_clock_status,
      powerSensorStatus,
      BMEStatus);

  /* Initializes the SD and RTC module */
  Serial.println("init SD ...");
  ret_code = utils::begin();
  Serial.println("SD init complete");
  if (ret_code == EDK_OK)
  {
    sd_clock_status = "Healthy";
    sensorMgr.printDeviceStatus(
        deviceSetupStatus,
        modemStatus,
        gpsFixStatus,
        payloadStatus,
        packetsSent,
        sd_clock_status,
        powerSensorStatus,
        BMEStatus);
  }
  else
  {
    sd_clock_status = "Unhealthy";
    sensorMgr.printDeviceStatus(
        deviceSetupStatus,
        modemStatus,
        gpsFixStatus,
        payloadStatus,
        packetsSent,
        sd_clock_status,
        powerSensorStatus,
        BMEStatus

    );
  }

  // Adjust Time

  // read_sys_time();

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
  }

  /* Modem initialization */
  if (modemSetup())
  {
    Serial.println("Modem initialization OK");
    modemStatus = "Healthy";
    sensorMgr.printDeviceStatus(
        deviceSetupStatus,
        modemStatus,
        gpsFixStatus,
        payloadStatus,
        packetsSent,
        sd_clock_status,
        powerSensorStatus,
        BMEStatus

    );
    // disableWifiAndBluetooth();
  }
  else
  {

    Serial.println("Modem initialization ERROR");
    modemStatus = "Bad";
    sensorMgr.printDeviceStatus(
        deviceSetupStatus,
        modemStatus,
        gpsFixStatus,
        payloadStatus,
        packetsSent,
        sd_clock_status,
        powerSensorStatus,
        BMEStatus

    );
    return;
  }

  //-------Start GPS-------//

  /* Check clock and assistance data, update if required */
  Serial.println("Initialize GPS");

  gpsFixStatus = "trying...";

  sensorMgr.printDeviceStatus(
      deviceSetupStatus,
      modemStatus,
      gpsFixStatus,
      payloadStatus,
      packetsSent,
      sd_clock_status,
      powerSensorStatus,
      BMEStatus

  );

  WalterModemGNSSFix gpsData = getGPSFix();
  //  5 - Blinks

  //-------Start GPS-------//

  // // Save the fix to RTC memory
  _lastGNSSFix = gpsData;
  savedLatitude = gpsData.latitude;
  savedLongitude = gpsData.longitude;
  savedAltitude = gpsData.height;
  savedSatCount = gpsData.satCount;

  Serial.printf("  retrieved Longitude: %.6f\n", gpsData.longitude);
  Serial.printf("  retrieved Satcount: %d\n", gpsData.satCount);

  gpsFixStatus = "bad";
  if (gpsData.satCount > 1)
  {
    modemStatus = "Not Connected";
    gpsFixStatus = "Healthy";
    sensorMgr.printDeviceStatus(
        deviceSetupStatus,
        modemStatus,
        gpsFixStatus,
        payloadStatus,
        packetsSent,
        sd_clock_status,
        powerSensorStatus,
        BMEStatus

    );
  }
  else
  {
    gpsFixStatus = "Bad";
    sensorMgr.printDeviceStatus(
        deviceSetupStatus,
        modemStatus,
        gpsFixStatus,
        payloadStatus,
        packetsSent,
        sd_clock_status,
        powerSensorStatus,
        BMEStatus

    );
  }

  deviceSetupStatus = "Good";

  sensorMgr.printDeviceStatus(
      deviceSetupStatus,
      modemStatus,
      gpsFixStatus,
      payloadStatus,
      packetsSent,
      sd_clock_status,
      powerSensorStatus,
      BMEStatus

  );
}

void loop()
{

  deviceSetupStatus = "In Loop...";

  bool isConnected = lteConnected();
  if (!pmsWakeStatus)
  {
    Serial.println("pms is sleep, waking up sensor...");
    pms.wakeUp();         // Ensure PMS is awake
    pmsWakeStatus = true; // Set wake status to true
    Serial.println("pms is awake and waiting for readings...");
    for (int i = 0; i < 30; i++)
    {
      Serial.print(".");
      delay(1000);
    }
  }

  // Wait 100 ms

  if (!isConnected)
  {
    Serial.println("Modem not connected, attempting to connect...");
    if (!lteConnect())
    {

      Serial.println("Failed to connect to LTE network.");
      modemStatus = "Failed to reconnect";

      ESP.restart();
    }
    else
    {
      modemStatus = "good";
    }
  }

  for (sensor = 0; sensor < NUM_OF_SENS; sensor++)
  {

    if (!envSensor[sensor].run())
    {

      checkBsecStatus(envSensor[sensor]);
      Serial.printf("Sensor %d failed to run\n", sensor);
    }
    // else
    // {
    //   Serial.printf("Sensor %d is running\n", sensor);
    // }
  }
  // Send data in batches of 4
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

    DynamicJsonDocument jsonDoc(8192); // 16 KB on heap
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
    return;

  // On first sensor of a new time-slice, create a fresh object in the array
  if (sensorStepCount == 0)
  {
    currentSampleObj = sensors.createNestedObject();
  }
  deviceSetupStatus = "Data...";
  // Temporary JsonObject to collect BSEC outputs
  DynamicJsonDocument tempDoc(4096); // 4 KB on heap
  JsonObject tempObj = tempDoc.to<JsonObject>();

  uint8_t runInStatus = -1;

  // Collect BSEC outputs in tempObj
  for (uint8_t i = 0; i < outputs.nOutputs; i++)
  {
    const bsecData &o = outputs.output[i];
    String sensorIdx = String(sensorStepCount);
    switch (o.sensor_id)
    {
    case BSEC_OUTPUT_IAQ:
      tempObj[String("iaq_") + sensorIdx] = o.signal;
      tempObj[String("iaqAcc_") + sensorIdx] = o.accuracy;
      break;
    case BSEC_OUTPUT_RAW_TEMPERATURE:
      tempObj[String("temp_") + sensorIdx] = o.signal;
      break;
    case BSEC_OUTPUT_RAW_PRESSURE:
      if (sensorStepCount == 0)
        tempObj[String("pres_") + sensorIdx] = o.signal;
      break;
    case BSEC_OUTPUT_RAW_HUMIDITY:
      tempObj[String("hum_") + sensorIdx] = o.signal;
      break;
    case BSEC_OUTPUT_RAW_GAS:
      tempObj[String("gasR_") + sensorIdx] = o.signal;
      break;
    case BSEC_OUTPUT_CO2_EQUIVALENT:
      tempObj[String("co2_") + sensorIdx] = o.signal;
      break;
    case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
      tempObj[String("bvoc_") + sensorIdx] = o.signal;
      break;
    case BSEC_OUTPUT_COMPENSATED_GAS:
      tempObj[String("com_gas") + sensorIdx] = o.signal;
      break;
    default:
      break;
    }
    if (o.sensor_id == BSEC_OUTPUT_RUN_IN_STATUS)
      runInStatus = o.signal;
  }

  // If runInStatus != 1, add collected data to currentSampleObj
  if (runInStatus != 1)
  {

    // Copy tempObj to currentSampleObj
    for (JsonPair kv : tempObj)
    {
      currentSampleObj[kv.key()] = kv.value();
    }

    // Add power data
    Serial.println("Configuring communication for INA219...");
    Serial.println("Reading battery and solar sensors...");
    DualPowerData powerData = powerReader.readAllSensors();
    currentSampleObj["batV"] = powerData.battery.busVoltage;
    currentSampleObj["batSV"] = powerData.battery.shuntVoltage;
    currentSampleObj["batC"] = powerData.battery.current_mA;
    currentSampleObj["batP"] = powerData.battery.power_mW;
    currentSampleObj["solV"] = powerData.solar.busVoltage;
    currentSampleObj["solSV"] = powerData.solar.shuntVoltage;
    currentSampleObj["solC"] = powerData.solar.current_mA;
    currentSampleObj["solP"] = powerData.solar.power_mW;

    // Add metadata
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char deviceId[13];
    snprintf(deviceId, sizeof(deviceId), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    currentSampleObj["device"] = deviceId;
    DateTime now = utils::get_rtc().now();
    unsigned long ms = millis() % 1000; // current millis within this second

    // Format timestamp with milliseconds: "YYYY-MM-DDTHH:MM:SS.mmmZ"
    char isoTimestamp[25]; // 24 chars + null terminator
    snprintf(isoTimestamp, sizeof(isoTimestamp),
             "%04d-%02d-%02dT%02d:%02d:%02d.%03luZ",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second(),
             ms);

    // Print the timestamp to Serial
    Serial.print("Payload UTC time: ");
    Serial.println(isoTimestamp);

    // Assign to JSON object
    currentSampleObj["ts"] = isoTimestamp;
    char wkt[50];
    snprintf(wkt, sizeof(wkt), "POINT(%f %f)", savedLongitude, savedLatitude);
    currentSampleObj["loc"] = wkt;
    currentSampleObj["alt"] = savedAltitude;
    currentSampleObj["satCnt"] = savedSatCount;
    currentSampleObj["bID"] = sampleCount;

    sensorStepCount++;
    Serial.print("Sensor count: ");
    Serial.println(sensorStepCount);
    Serial.print("Sample count: ");
    Serial.println(sampleCount);
  }

  // Once all sensors for this time-slice are processed
  if (sensorStepCount >= NUM_SENSORS)
  {
    sampleCount++;
    sensorStepCount = 0; // Reset for next sample

    // If buffer full, serialize & clear everything
    if (sampleCount >= BUFFER_SIZE)
    {
      // Add PMS sensor data
      Serial.println("Send PM read request...");
      pms.requestRead();
      delay(10);
      Serial.println("Wait max. 1 second for read...");
      if (pms.readUntil(pmData))
      {
        Serial.print("PM 1.0 (ug/m3): ");
        Serial.println(pmData.PM_AE_UG_1_0);
        Serial.print("PM 2.5 (ug/m3): ");
        Serial.println(pmData.PM_AE_UG_2_5);
        Serial.print("PM 10.0 (ug/m3): ");
        Serial.println(pmData.PM_AE_UG_10_0);
        sensors[0]["pm10"] = pmData.PM_AE_UG_10_0;
        sensors[0]["pm2_5"] = pmData.PM_AE_UG_2_5;
        sensors[0]["pm1"] = pmData.PM_AE_UG_1_0;
      }
      else
      {
        Serial.println("PMS read failed");
        sensors[0]["pm10"] = nullptr;
        sensors[0]["pm2_5"] = nullptr;
        sensors[0]["pm1"] = nullptr;
      }

      char jsonBuffer[6000];
      Serial.println("=== SENDING BATCH ===");
      serializeJsonPretty(sensors, Serial);
      serializeJson(sensors, jsonBuffer);
      Serial.println();

      payloadStatus = "" + String(sampleCount);
      sensorMgr.printDeviceStatus(
          deviceSetupStatus,
          modemStatus,
          gpsFixStatus,
          payloadStatus,
          packetsSent,
          sd_clock_status,
          powerSensorStatus,
          BMEStatus

      );

      for (int i = 0; i < BUFFER_SIZE; ++i)
      {

        JsonObject testObj;

        Serial.println(String("sending parts... ") + String(i));
        char jsonBuffer[12000];
        testObj = sensors[i];
        serializeJson(testObj, jsonBuffer);
        Serial.println("serialize json parts...");
        if (sendSensorData(jsonBuffer))
        {
          Serial.println("sending data to cloud...");
          payloadStatus = "0";
          payloadCounter += 1;
          packetsSent = "" + String(payloadCounter);
          sensorMgr.printDeviceStatus(
              deviceSetupStatus,
              modemStatus,
              gpsFixStatus,
              payloadStatus,
              packetsSent,
              sd_clock_status,
              powerSensorStatus,
              BMEStatus);
        }
        else
        {
          Serial.println("Send failed.");
        }
      }

      // TODO: Send payloadDoc via modem or BLE

      // Clear for next round
      sensors.clear();
      payloadDoc.clear();
      sensors = payloadDoc.to<JsonArray>();
      sampleCount = 0;
      // Serial.println("disconnecting from LTE...");
      //  if (lteDisconnect())
      // {
      Serial.println("entering light sleep for 5 seconds...");
      delay(10000); // Wait 5 seconds before going to sleep
                    // modem.sleep(10, true); // true = light sleep
                    // }
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

void disableWifiAndBluetooth()
{
  esp_wifi_stop();
  esp_bt_controller_disable();
}

void configureUltraLowPowerModem()
{
  // Enable Power Saving Mode (PSM)
  // T3412 = "00100001" (1 hour TAU), T3324 = "00000101" (10s active time)
  // 30s TAU, 10s Active time
  modem.configPSM(WALTER_MODEM_PSM_ENABLE, "00011110", "00000101");
}

void checkBsecStatus(Bsec2 bsec)
{
  if (bsec.status < BSEC_OK)
  {
    Serial.println("BSEC error code : " + String(bsec.status));
    // errLeds(); /* Halt in case of failure */
  }
  else if (bsec.status > BSEC_OK)
  {
    Serial.println("BSEC warning code : " + String(bsec.status));
  }

  if (bsec.sensor.status < BME68X_OK)
  {
    Serial.println("BME68X error code : " + String(bsec.sensor.status));
    // errLeds(); /* Halt in case of failure */
  }
  else if (bsec.sensor.status > BME68X_OK)
  {
    Serial.println("BME68X warning code : " + String(bsec.sensor.status));
  }
}
