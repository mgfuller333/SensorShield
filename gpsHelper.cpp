#include <esp_mac.h>
#include <WalterModem.h>
#include "cellularHelper.h"
#include "utils.h"
#include <Wire.h>

RTC_DATA_ATTR uint32_t lastGPSFixTime = 0;
#define MAX_GNSS_CONFIDENCE 1000.0
#define LED_PIN 42 // Use a free IO pin for LED (example IO2 or any other free)
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
            // 4 blinks
            digitalWrite(LED_PIN, LOW); // Start LED off
            delay(10);
            digitalWrite(LED_PIN, HIGH); // Turn LED ON
            delay(10);                   // Wait 5 ms
            digitalWrite(LED_PIN, LOW);  // Turn LED OFF
            delay(10);
            digitalWrite(LED_PIN, HIGH); // Turn LED ON
            delay(10);                   // Wait 5 ms
            digitalWrite(LED_PIN, LOW);  // Turn LED OFF
            delay(10);
            digitalWrite(LED_PIN, HIGH); // Turn LED ON
            delay(10);                   // Wait 5 ms
            digitalWrite(LED_PIN, LOW);  // Turn LED OFF
            delay(10);
            digitalWrite(LED_PIN, HIGH); // Turn LED ON
            delay(10);
            digitalWrite(LED_PIN, LOW); // Turn LED OFF

            Serial.print("Could not connect to LTE network\r\n");
            return false;
        }

        lteConnected = true;

        /*
         * Wait for the modem to synchronize time with the LTE network, try 5 times
         * with a delay of 500ms.
         */
        // 3 blinks
        digitalWrite(LED_PIN, HIGH); // Turn LED ON
        delay(500);                  // Wait 5 ms
        digitalWrite(LED_PIN, LOW);  // Turn LED OFF
        delay(500);
        digitalWrite(LED_PIN, HIGH); // Turn LED ON
        delay(500);                  // Wait 5 ms
        digitalWrite(LED_PIN, LOW);  // Turn LED OFF
        delay(500);
        digitalWrite(LED_PIN, HIGH); // Turn LED ON
        delay(500);
        digitalWrite(LED_PIN, LOW); // Turn LED OFF

        for (int i = 0; i < 5; ++i)
        {
            if (!modem.gnssGetUTCTime(&rsp))
            {
                Serial.println("GNSS UTC time request failed");
                delay(500);
                continue;
            }

            int64_t epochSec = -1;

            Serial.println("GNSS UTC time request succeeded");
            if (rsp.data.clock.epochTime > 4)
            {
                int64_t epochSec = rsp.data.clock.epochTime;

                Serial.print("Epoch Time...");
                Serial.println(epochSec);

                // if (!utils::get_rtc().isrunning())
                // {
                //     Serial.println("RTC is not initialized or running!");
                //     return false;
                // }

                // Serial.print("clock is running...");

                Serial.println("GNSS UTC time adjusting...");

                utils::get_rtc().adjust(DateTime((uint32_t)epochSec));

                Serial.println("GNSS UTC time adjusted");

                Serial.println("GNSS UTC time now...");
                DateTime now = utils::get_rtc().now();
                Serial.println("GNSS UTC time now grabbed");
                // Create a buffer for the timestamp
                char buf[20];
                // Format timestamp as YYYY-MM-DD hh:mm:ss (ISO 8601 format)
                sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d",
                        now.year(), now.month(), now.day(),
                        now.hour(), now.minute(), now.second());

                // Print the timestamp to Serial
                Serial.print("Current UTC time: ");
                Serial.println(buf);
            }

            delay(500);
        }
    }
    else
    {
        // Clock is already valid, update RTC
        // Serial.printf("Clock already synchronized: %" PRIi64 "\r\n", rsp.data.clock.epochTime);
        // utils::get_rtc().adjust(DateTime((uint32_t)rsp.data.clock.epochTime));
        // Serial.println("RTC updated with LTE time");
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

    if (!modem.gnssConfig())
    {
        Serial.println("Could not configure the GNSS subsystem");

        ESP.restart();
    }
    Serial.println("Configured the GNSS subsystem");

    modem.gnssSetEventHandler(fixHandler);

    // 4 blinks
    digitalWrite(LED_PIN, HIGH); // Turn LED ON
    delay(500);                  // Wait 5 ms
    digitalWrite(LED_PIN, LOW);  // Turn LED OFF
    delay(500);
    digitalWrite(LED_PIN, HIGH); // Turn LED ON
    delay(500);                  // Wait 5 ms
    digitalWrite(LED_PIN, LOW);  // Turn LED OFF
    delay(500);
    digitalWrite(LED_PIN, HIGH); // Turn LED ON
    delay(500);
    digitalWrite(LED_PIN, LOW); // Turn LED OFF
    delay(500);
    digitalWrite(LED_PIN, HIGH); // Turn LED ON
    delay(500);
    digitalWrite(LED_PIN, LOW); // Turn LED OFF

    // Serial.println("Set Fix Event Handler");
    if (!updateGNSSAssistance())
    {
        // Serial.println("Assistance update failed");
    }
    // Serial.println("Updated GNSS Assistance");

    // Serial.println("Attempting GNSS fix...");
    // 4 - Blinks
    digitalWrite(LED_PIN, HIGH);

    // Try up to 5 times to get a good fix
    // for (int i = 0; i < 5; ++i)
    for (int i = 0; i < 1; ++i)
    {
        fixRcvd = false;
        if (!modem.gnssPerformAction())
        {
            // Serial.print("Could not request GNSS fix\r\n");
            delay(1000);
            ESP.restart();
        }
        // Serial.print("Started GNSS fix\r\n");

        int j = 0;
        while (!fixRcvd)
        {
            Serial.print(".");
            if (j >= 300)
            {
                Serial.println("");
                Serial.println("Timed out while waiting for GNSS fix");
                delay(1000);
                break;
            }
            j++;
            delay(500);
        }

        Serial.println("");

        if (posFix.estimatedConfidence <= MAX_GNSS_CONFIDENCE)
        {
            break;
        }
    }

    uint8_t abovedBTreshold = 0;
    for (int i = 0; i < posFix.satCount; ++i)
    {
        if (posFix.sats[i].signalStrength >= 30)
        {
            abovedBTreshold += 1;
        }
    }

    // Evaluate result
    uint8_t strongSats = 0;
    for (int i = 0; i < posFix.satCount; ++i)
    {
        if (posFix.sats[i].signalStrength >= 30)
        {
            strongSats++;
        }
    }
    digitalWrite(LED_PIN, LOW);

    Serial.printf(" Strong Sats: %.2f\n", strongSats);

    // if (strongSats < 5)
    // {
    //     Serial.println("Failed to get a valid GNSS fix due to insufficient strong satellites.");
    //     posFix.latitude = 0.0;
    //     posFix.longitude = 0.0;
    //     posFix.height = 0.0;
    //     posFix.satCount = 0xFF;
    //     return posFix;
    // }

    Serial.printf("GNSS fix:\n");
    Serial.printf("  Confidence: %.2f\n", posFix.estimatedConfidence);
    Serial.printf("  Latitude: %.6f\n", posFix.latitude);
    Serial.printf("  Longitude: %.6f\n", posFix.longitude);
    Serial.printf("  Satcount: %d\n", posFix.satCount);
    Serial.printf("  Good sats (>=30): %d\n", strongSats);

    return posFix;
}
