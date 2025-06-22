#include <esp_mac.h>
#include <WalterModem.h>
#include "cellularHelper.h"
#include "utils.h"

// RTC_DATA_ATTR uint32_t lastGPSFixTime = 0;
#define MAX_GNSS_CONFIDENCE 100.0

volatile bool fixRcvd = false;
WalterModemGNSSFix posFix = {};

void fixHandler(const WalterModemGNSSFix *fix, void *args)
{
    memcpy(&posFix, fix, sizeof(WalterModemGNSSFix));
    fixRcvd = true;
}

void checkAssistanceData(
    WalterModemRsp *rsp,
    bool *updateAlmanac = NULL,
    bool *updateEphemeris = NULL)
{
    if (updateAlmanac != NULL)
    {
        *updateAlmanac = false;
    }

    if (updateEphemeris != NULL)
    {
        *updateEphemeris = false;
    }

    Serial.print("Almanac data is ");
    if (rsp->data.gnssAssistance.almanac.available)
    {
        Serial.printf("available and should be updated within %ds\r\n",
                      rsp->data.gnssAssistance.almanac.timeToUpdate);
        if (updateAlmanac != NULL)
        {
            *updateAlmanac = rsp->data.gnssAssistance.almanac.timeToUpdate <= 0;
        }
    }
    else
    {
        Serial.print("not available.\r\n");
        if (updateAlmanac != NULL)
        {
            *updateAlmanac = true;
        }
    }

    Serial.print("Real-time ephemeris data is ");
    if (rsp->data.gnssAssistance.realtimeEphemeris.available)
    {
        Serial.printf("available and should be updated within %ds\r\n",
                      rsp->data.gnssAssistance.realtimeEphemeris.timeToUpdate);
        if (updateEphemeris != NULL)
        {
            *updateEphemeris = rsp->data.gnssAssistance.realtimeEphemeris.timeToUpdate <= 0;
        }
    }
    else
    {
        Serial.print("not available.\r\n");
        if (updateEphemeris != NULL)
        {
            *updateEphemeris = true;
        }
    }
}

bool updateGNSSAssistance()
{
    bool lteConnected = false;
    WalterModemRsp rsp = {};

    lteDisconnect();

    /* Even with valid assistance data the system clock could be invalid */
    modem.gnssGetUTCTime(&rsp);

    if (rsp.data.clock.epochTime <= 4)
    {
        /* The system clock is invalid, connect to LTE network to sync time */
        if (!lteConnect())
        {
            Serial.print("Could not connect to LTE network\r\n");
            return false;
        }

        lteConnected = true;

        /*
         * Wait for the modem to synchronize time with the LTE network, try 5 times
         * with a delay of 500ms.
         */
        for (int i = 0; i < 5; ++i)
        {
            modem.gnssGetUTCTime(&rsp);

            if (rsp.data.clock.epochTime > 4)
            {
                Serial.printf("Synchronized clock with network: %" PRIi64 "\r\n",
                              rsp.data.clock.epochTime);
                break;
            }
            else if (i == 4)
            {
                Serial.print("Could not sync time with network\r\n");
                return false;
            }

            delay(500);
        }
    }

    /* Check the availability of assistance data */
    if (!modem.gnssGetAssistanceStatus(&rsp) ||
        rsp.type != WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA)
    {
        Serial.print("Could not request GNSS assistance status\r\n");
        return false;
    }

    bool updateAlmanac = false;
    bool updateEphemeris = false;
    checkAssistanceData(&rsp, &updateAlmanac, &updateEphemeris);

    if (!(updateAlmanac || updateEphemeris))
    {
        if (lteConnected)
        {
            if (!lteDisconnect())
            {
                Serial.print("Could not disconnect from the LTE network\r\n");
                return false;
            }
        }

        return true;
    }

    if (!lteConnected)
    {
        if (!lteConnect())
        {
            Serial.print("Could not connect to LTE network\r\n");
            return false;
        }
    }

    if (updateAlmanac)
    {
        if (!modem.gnssUpdateAssistance(WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC))
        {
            Serial.print("Could not update almanac data\r\n");
            return false;
        }
    }

    if (updateEphemeris)
    {
        if (!modem.gnssUpdateAssistance(
                WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS))
        {
            Serial.print("Could not update real-time ephemeris data\r\n");
            return false;
        }
    }

    if (!modem.gnssGetAssistanceStatus(&rsp) ||
        rsp.type != WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA)
    {
        Serial.print("Could not request GNSS assistance status\r\n");
        return false;
    }

    checkAssistanceData(&rsp, &updateAlmanac, &updateEphemeris);

    if (!lteDisconnect())
    {
        Serial.print("Could not disconnect from the LTE network\r\n");
        return false;
    }

    return true;
}

WalterModemGNSSFix getGPSFix()
{
    // Ensure GNSS assistance data is up-to-date
    if (!updateGNSSAssistance())
    {
        Serial.println("Assistance update failed");
    }

    // Reset flag and set fix handler
    fixRcvd = false;
    modem.gnssSetEventHandler(fixHandler);

    Serial.println("Attempting GNSS fix...");

    // Try up to 5 times to get a good fix
    for (int attempt = 0; attempt < 5; ++attempt)
    {
        if (!modem.gnssPerformAction())
        {
            Serial.println("GNSS fix request failed");
            continue;
        }

        Serial.print("Waiting for fix");
        for (int waitCount = 0; waitCount < 300 && !fixRcvd; ++waitCount)
        {
            delay(500);
            Serial.print(".");
        }
        Serial.println();

        if (fixRcvd && posFix.estimatedConfidence <= MAX_GNSS_CONFIDENCE)
        {
            break;
        }
    }

    // Evaluate result
    if (!fixRcvd || posFix.estimatedConfidence > MAX_GNSS_CONFIDENCE)
    {
        Serial.println("Failed to get a valid GNSS fix");
        posFix.latitude = 0.0;
        posFix.longitude = 0.0;
        posFix.satCount = 0xFF;
    }
    else
    {
        uint8_t strongSats = 0;
        for (int i = 0; i < posFix.satCount; ++i)
        {
            if (posFix.sats[i].signalStrength >= 30)
            {
                strongSats++;
            }
        }

        Serial.printf("GNSS fix:");
        Serial.printf("  Confidence: %.2f\n", posFix.estimatedConfidence);
        Serial.printf("  Latitude: %.6f\n", posFix.latitude);
        Serial.printf("  Longitude: %.6f\n", posFix.longitude);
        Serial.printf("  Satcount: %d\n", posFix.satCount);
        Serial.printf("  Good sats (>=30): %d\n", strongSats);
    }

    return posFix;
}
