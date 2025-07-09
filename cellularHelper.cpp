// config.cpp
#include "cellularHelper.h"
#include <esp_mac.h>
#include <WalterModem.h>
#define LED_PIN 42 // Use a free IO pin for LED (example IO2 or any other free)
#include <Wire.h>
// Global modem and sensor objects
WalterModem modem;
WalterModemRsp rsp = {};

uint8_t incomingBuf[256] = {0};

/**
 * @brief MQTT client and message prefix based on mac address
 */
char macString[32];

/**
 * @brief The buffer used to format the outgoing JSON messages in.
 */
uint8_t outBuf[JSON_BUFSIZE] = {0};

/**
 * @brief Buffer for incoming HTTP response
 */
uint8_t inBuf[256] = {0};

uint8_t dataBuf[PACKET_SIZE] = {0};
/**
 * @brief Connect to the LTE network.
 *
 * This function will connect the modem to the LTE network. This function will
 * block until the modem is attached.
 *
 * @return True on success, false on error.
 */

bool lteConnected()
{
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
}

bool waitForNetwork()
{
  /* Wait for the network to become available */
  Serial.println("waiting for the network...");
  int timeout = 0;
  while (!lteConnected())
  {
    delay(100);
    timeout += 100;
    if (timeout > 300000)
      return false;
  }
  Serial.println("Connected to the network");

  if (modem.mqttConfig("Sharingan1", "sireenUser", "sireenPass333", 0))
  {
    Serial.println("MQTT configuration succeeded");
  }
  else
  {
    Serial.println("Error: MQTT configuration failed");
    ESP.restart();
    return false;
  }

  Serial.println("attempting mqtt connection....");

  if (modem.mqttConnect("broker.hivemq.com", 1883))
  {
    Serial.println("MQTT connection succeeded");
  }
  else
  {

    Serial.println("Error: MQTT connection failed");
    ESP.restart();
    return false;
  }

  Serial.printf("Attempting MQTT...");

  return true;
}

/**
 * @brief This function tries to connect the modem to the cellular network.
 * @return true if the connection attempt is successful, else false.
 */
bool lteConnect()
{
  if (modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF))
  {
    Serial.println("Successfully set operational state to NO RF");
  }
  else
  {
    Serial.println("Error: Could not set operational state to NO RF");
    return false;
  }

  /* Create PDP context */
  if (modem.definePDPContext())
  {
    Serial.println("Created PDP context");
  }
  else
  {
    Serial.println("Error: Could not create PDP context");
    return false;
  }

  /* Set the operational state to full */
  if (modem.setOpState(WALTER_MODEM_OPSTATE_FULL))
  {
    Serial.println("Successfully set operational state to FULL");
  }
  else
  {
    Serial.println("Error: Could not set operational state to FULL");

    return false;
  }

  /* Set the network operator selection to automatic */
  if (modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC))
  {
    Serial.println("Network selection mode to was set to automatic");
  }
  else
  {
    Serial.println(
        "Error: Could not set the network selection mode to automatic");
    return false;
  }

  /* Set the network operator selection to automatic */
  if (waitForNetwork())
  {
    Serial.println("Connected to network");
    return true;
  }
  else
  {
    Serial.println("could not connect to network");
    return false;
  }

  return false;
}

/**
 * @brief Disconnect from the LTE network.
 *
 * This function will disconnect the modem from the LTE network and block until
 * the network is actually disconnected. After the network is disconnected the
 * GNSS subsystem can be used.
 *
 * @return True on success, false on error.
 */
bool lteDisconnect()
{
  /* Set the operational state to minimum */
  if (!modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF))
  {
    Serial.print("Could not set operational state to No_RF\r\n");
    return false;
  }

  /* Wait for the network to become available */
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while (regState != WALTER_MODEM_NETWORK_REG_NOT_SEARCHING)
  {
    delay(100);
    regState = modem.getNetworkRegState();
  }

  Serial.print("Disconnected from the network\r\n");
  return true;
}

bool modemSetup()
{

  //------Start of Modem Initialization------//
  Serial.println("Walter SensorShield v0.0.1 with BME688 (SPI via commMux)");

  // Retrieve and print the MAC address (stored in first 6 bytes of dataBuf)
  esp_read_mac(dataBuf, ESP_MAC_WIFI_STA);
  Serial.printf("Walter's MAC is: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                dataBuf[0],
                dataBuf[1],
                dataBuf[2],
                dataBuf[3],
                dataBuf[4],
                dataBuf[5]);

  // Initialize the modem on Serial2 (or your chosen hardware serial)
  if (modem.begin(&Serial2))
  {
    Serial.println("Modem began... OK");
  }
  else
  {
    Serial.println("Modem initialization ERROR");
    return false;
  }

  if (modem.checkComm())
  {
    Serial.print("Modem communication is ok\r\n");
  }
  else
  {
    Serial.print("Modem communication error\r\n");
    return false;
  }

  if (modem.tlsConfigProfile(TLS_PROFILE))
  {
    Serial.print("Successfully configured the TLS profile\r\n");
  }
  else
  {
    Serial.print("Failed to configure TLS profile\r\n");
  }

  /* Configure http profile for a simple test */
  if (modem.httpConfigProfile(HTTP_PROFILE, "api.datacake.co", 443, 1))
  {
    Serial.println("Successfully configured the http profile");
    return true;
  }
  else
  {
    Serial.println("Error: Failed to configure HTTP profile");
    return false;
  }

  if (modem.mqttSubscribe("SireenTest"))
  {
    Serial.println("MQTT subscribed to topic 'SireenTest'");
  }
  else
  {
    Serial.println("Error: MQTT subscribe failed");
    ESP.restart();
    return false;
  }

  // if (modem.unlockSIM())
  // {
  //   Serial.print("Successfully unlocked SIM card\r\n");
  // }
  // else
  // {
  //   Serial.print("Could not unlock SIM card\r\n");
  //   return false;
  // }

  // if (modem.getSIMCardID(&rsp))
  // {
  //   Serial.print("SIM card identity:\r\n");
  //   Serial.printf(" -ICCID: %s\r\n", rsp.data.simCardID.iccid);
  //   Serial.printf(" -eUICCID: %s\r\n", rsp.data.simCardID.euiccid);
  // }
  // else
  // {
  //   Serial.print("Could not getSIMCardID\r\n");
  //   return false;
  // }

  // if (modem.getSIMCardIMSI(&rsp))
  // {
  //   Serial.printf("Active IMSI: %s\r\n", rsp.data.imsi);
  // }
  // else
  // {
  //   Serial.print("Could not getSIMCardIMSI\r\n");
  //   return false;
  // }
}

/**
 * @brief Sends environmental sensor data via HTTP POST.
 *
 * This function formats sensor data into a JSON payload and sends it to a
 * specified HTTP endpoint using the modem. It handles HTTP send and receive
 * operations, including retries if the response is not immediately available.
 *
 * @param jsonPayload  JsonPaylod.
 *
 * @return True if the data was successfully sent and a response was received,
 *         False otherwise.
 */
bool sendSensorData(String dataPayload)
{
  // Format the JSON payload

  Serial.printf("Outgoing JSON: %s\n", dataPayload.c_str());

  // HTTP variables
  static short httpReceiveAttemptsLeft = 0;
  static char ctbuf[32];

  static char outgoingMsg[65536];
  /* other public mqtt broker with web client: mqtthq.com */
  /* configure the mqtt client */

  if (modem.mqttPublish("SireenTest",
                        (uint8_t *)dataPayload.c_str(), // Pass jsonString directly
                        dataPayload.length()            // Size of the JSON data
                        ))
  {
    Serial.printf("published  on topic 'SireenTest'\r\n");
  }
  else
  {
    {
      Serial.print("MQTT publish failed\r\n");
      return false;
    }
  }

  delay(1000);

  // while (modem.mqttDidRing("SireenTest", incomingBuf,
  //                          sizeof(incomingBuf), &rsp))
  // {
  //   Serial.printf("incoming: qos=%d msgid=%d len=%d:\r\n",
  //                 rsp.data.mqttResponse.qos,
  //                 rsp.data.mqttResponse.messageId,
  //                 rsp.data.mqttResponse.length);
  //   for (int i = 0; i < rsp.data.mqttResponse.length; i++)
  //   {
  //     Serial.printf("'%c' 0x%02x\r\n", incomingBuf[i], incomingBuf[i]);
  //   }
  // }

  // Attempt to send HTTP POST request

  // if (!httpReceiveAttemptsLeft)
  // {

  //   if (modem.httpSend(
  //           HTTP_PROFILE,
  //           "/integrations/api/b2f660a9-c735-463b-ba46-1396a60d4319/",
  //           (uint8_t *)dataPayload.c_str(), // Pass jsonString directly
  //           dataPayload.length(),           // Size of the JSON data
  //           WALTER_MODEM_HTTP_SEND_CMD_POST,
  //           WALTER_MODEM_HTTP_POST_PARAM_JSON,
  //           ctbuf,
  //           sizeof(ctbuf)))
  //   {
  //     Serial.println("HTTP query performed");
  //     httpReceiveAttemptsLeft = MAX_HTTP_ATTEMPTS;
  //     return true;
  //   }
  //   else
  //   {
  //     Serial.println("HTTP query failed");
  //     delay(1000);
  //     return false;
  //   }

  //   // confirmation
  //   digitalWrite(LED_PIN, LOW); // Turn LED OFF
  //   delay(50);
  //   digitalWrite(LED_PIN, HIGH); // Turn LED ON
  //   delay(50);                   // Wait 5 ms
  //   digitalWrite(LED_PIN, LOW);  // Turn LED OFF
  //   delay(50);
  //   digitalWrite(LED_PIN, HIGH); // Turn LED ON
  //   delay(50);                   // Wait 5 ms
  //   digitalWrite(LED_PIN, LOW);  // Turn LED OFF
  //   delay(50);
  //   digitalWrite(LED_PIN, HIGH); // Turn LED ON
  //   delay(50);                   // Wait 4 seconds
  //   digitalWrite(LED_PIN, LOW);  // Turn LED OFF

  //   delay(1000);

  //   return true;
  // }
  // else
  // {
  //   httpReceiveAttemptsLeft--;

  //   if (modem.httpDidRing(HTTP_PROFILE, inBuf, sizeof(inBuf), &rsp))
  //   {
  //     httpReceiveAttemptsLeft = 1;

  //     Serial.printf("HTTP status code: %d\n", rsp.data.httpResponse.httpStatus);
  //     Serial.printf("Content type: %s\n", ctbuf);
  //     Serial.printf("Response: [%s]\n", inBuf);
  //     return true;
  //   }
  //   else
  //   {
  //     Serial.println("HTTP response not yet received");
  //   }
  // }

  // delay(1000);

  return true;
}

bool configureLowPowerModem()
{
  // Enable Power Saving Mode (T3412 = TAU, T3324 = Active Time)
  // TAU = "00100001" → 1 hour
  // Active Time = "00000101" → 10s
  Serial.println("Configuring Power Saving Mode (PSM)...");
  if (!modem.configPSM(WALTER_MODEM_PSM_ENABLE, "00100001", "00000101"))
  {
    Serial.println("Error: Failed to configure PSM.");
    return false;
  }
  Serial.println("PSM configured successfully.");

  Serial.println("Configuring eDRX...");
  if (!modem.configEDRX(WALTER_MODEM_EDRX_ENABLE, "1001", "0000"))
  {
    Serial.println("Error: Failed to configure eDRX.");
    return false;
  }
  Serial.println("eDRX configured successfully.");

  Serial.println("Low power modem configuration complete.");
  return true;
}