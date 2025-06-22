#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/ConnectivityTest/SensorShield/gpsHelper.h"
// config.h
#ifndef GPSHELPER_H
#define GPSHELPER_H

#include <esp_mac.h>
#include <WalterModem.h>
#include "cellularHelper.h"
#include <Arduino.h>

// Global modem and sensor objects
extern WalterModem modem;
extern WalterModemRsp rsp;

/**
 * @brief All fixes with a confidence below this number are considered ok.
 */
#define MAX_GNSS_CONFIDENCE 100.0

/**
 * @brief Flag used to signal when a fix is received.
 */
extern volatile bool fixRcvd;

/**
 * @brief The last received GNSS fix.
 */
extern WalterModemGNSSFix posFix;

/**
 * @brief Check the assistance data in the modem response.
 *
 * This function checks the availability of assistance data in the modem's
 * response. This function also sets a flag if any of the assistance databases
 * should be updated.
 *
 * @param rsp The modem response to check.
 * @param updateAlmanac Pointer to the flag to set when the almanac should be
 * updated.
 * @param updateEphemeris Pointer to the flag to set when ephemeris should be
 * updated.
 *
 * @return None.
 */
void checkAssistanceData(
    WalterModemRsp *rsp,
    bool *updateAlmanac = nullptr,
    bool *updateEphemeris = nullptr);

/**
 * @brief This function will update GNSS assistance data when needed.
 *
 * This function will check if the current real-time ephemeris data is good
 * enough to get a fast GNSS fix. If not, the function will attach to the LTE
 * network to download newer assistance data.
 *
 * @return True on success, false on error.
 */
bool updateGNSSAssistance();

/**
 * @brief This function is called when a fix attempt finishes.
 *
 * This function is called by Walter's modem library as soon as a fix attempt
 * has finished. It should be as short as possible as it is called within the
 * modem data thread.
 *
 * @param fix The fix data.
 * @param args Optional arguments, a NULL pointer in this case.
 *
 * @return None.
 */
void fixHandler(const WalterModemGNSSFix *fix, void *args);

/**
 * @brief Retrieve the GPS fix.
 *
 * This function attempts to get a GNSS fix and returns the fix data.
 *
 * @return The GNSS fix data.
 */
WalterModemGNSSFix getGPSFix();

#endif // CONFIG_H
