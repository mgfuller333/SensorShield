
#ifndef CELLULARHELPER_H
#define CELLULARHELPER_H

#include "cellularHelper.h"
#include <esp_mac.h>
#include <WalterModem.h>
#include <Arduino.h>

// Global modem and sensor objects
extern WalterModem modem;
extern WalterModemRsp rsp;

/**
 * @brief The radio access technology to use - LTEM or NBIOT.
 */
#define RADIO_TECHNOLOGY WALTER_MODEM_RAT_LTEM

/**
 * @brief The size in bytes of a minimal sensor + GNSS + cell info packet.
 */
#define PACKET_SIZE 29
/**
 * @brief The size of the outgoing JSON buffer.
 */
#define JSON_BUFSIZE 128
#define HTTP_PROFILE 1
#define TLS_PROFILE 1

/**
 * @brief Buffers for incoming and outgoing data
 */
extern uint8_t incomingBuf[256];
extern uint8_t outBuf[128];
extern uint8_t inBuf[256];
extern uint8_t dataBuf[PACKET_SIZE];

/**
 * @brief Maximum retries before reset
 */
#define MAX_HTTP_ATTEMPTS 3

/**
 * @brief Connect to the LTE network.
 *
 * This function will connect the modem to the LTE network. This function will
 * block until the modem is attached.
 *
 * @return True on success, false on error.
 */
bool lteConnect();

/**
 * @brief Connect to the LTE network.
 *
 * This function will return the connection status of the LTE network.
 *
 * @return True on success, false on error.
 */

bool lteConnected();

/**
 * @brief Disconnect from the LTE network.
 *
 * This function will disconnect the modem from the LTE network and block until
 * the network is actually disconnected. After the network is disconnected the
 * GNSS subsystem can be used.
 *
 * @return True on success, false on error.
 */
bool lteDisconnect();

/**
 * @brief Initialize the modem and perform basic setup.
 *
 * This function initializes the modem, retrieves SIM and device information,
 * and prepares the modem for network communication.
 *
 * @return True on success, false on error.
 */
bool modemSetup();

/**
 * @brief Configures Low Power Settings.
 *
 * This function prepares the modem for low power operation by configuring
 *
 * @return True on success, false on error.
 */
bool configureLowPowerModem();

bool sendSensorData(String jsonPayload);

#endif // CONFIG_H