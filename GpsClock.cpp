#include "GpsClock.h"
#include "LoggerConfig.h"
#include "QueueRules.h"

#include <Preferences.h>
#include <esp_timer.h>
#include <sys/time.h>
#include <time.h>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <inttypes.h>

namespace {
constexpr int64_t MIN_VALID_EPOCH = 1577836800LL;  // 2020-01-01 UTC
constexpr int64_t MAX_VALID_EPOCH = 4102444800LL;  // 2100-01-01 UTC (exclusive)

bool validEpoch(int64_t epoch) {
  return epoch >= MIN_VALID_EPOCH && epoch < MAX_VALID_EPOCH;
}
}  // namespace

// Retained only across ESP32 deep sleep; never assume RTC has survived power loss.
RTC_DATA_ATTR bool clockValid = false;

// Preserve the exact NVS layout used by earlier firmware releases.
struct Location {
  double latitude = 0, longitude = 0, altitude = 0;
  int64_t fixTime = 0;
  uint32_t satellites = 0;
  uint32_t crc = 0;
};

Location location;
bool locationUpdatedThisWake = false;

void setupGpsClock() {
  locationUpdatedThisWake = false;

  // RTC_DATA_ATTR validity must also agree with the actual system clock.
  if (clockValid && !validEpoch(static_cast<int64_t>(time(nullptr)))) {
    clockValid = false;
  }
  Serial.println(clockValid ? "Time: retained ESP32 clock available."
                            : "Time: awaiting GNSS/modem UTC synchronization.");

  location = {};
  Preferences p;
  if (!p.begin("odrade-nav", true)) {
    Serial.println("Location: could not open saved navigation data.");
    return;
  }

  if (p.getBytesLength("location") == sizeof(location)) {
    if (p.getBytes("location", &location, sizeof(location)) != sizeof(location) ||
        location.crc != dataCrc(&location, offsetof(Location, crc)) ||
        !std::isfinite(location.latitude) ||
        !std::isfinite(location.longitude) ||
        !std::isfinite(location.altitude) ||
        fabs(location.latitude) > 90 || fabs(location.longitude) > 180 ||
        !validEpoch(location.fixTime)) {
      location = {};
      Serial.println("Location: discarded invalid saved position.");
    }
  }
  p.end();
}

void setGpsTime(int64_t epoch, int64_t receivedUs) {
  if (!validEpoch(epoch)) {
    Serial.printf("Time: rejected invalid UTC epoch %" PRId64 ".\n", epoch);
    return;
  }

  const int64_t currentUs = esp_timer_get_time();
  if (receivedUs < 0 || receivedUs > currentUs) {
    Serial.println("Time: rejected invalid receipt time; not updating ESP32 clock.");
    return;
  }

  // Account for the time spent processing a modem response/event.
  const int64_t nowUs = epoch * 1000000LL + (currentUs - receivedUs);
  timeval tv = {};
  tv.tv_sec = static_cast<time_t>(nowUs / 1000000LL);
  tv.tv_usec = static_cast<suseconds_t>(nowUs % 1000000LL);
  if (settimeofday(&tv, nullptr) != 0) {
    Serial.println("Time: could not synchronize the ESP32 clock.");
    return;
  }
  clockValid = true;
  char iso[25];
  if (formatGpsTimestamp(iso, sizeof(iso)))
    Serial.printf("Time: ESP32 synchronized to %s (UTC, BigQuery TIMESTAMP compatible).\n", iso);
}

bool formatGpsTimestamp(char *buffer, size_t size) {
  if (!buffer || size == 0) return false;
  buffer[0] = '\0';
  if (!clockValid) return false;

  const time_t now = time(nullptr);
  if (!validEpoch(static_cast<int64_t>(now))) return false;

  struct tm utc = {};
  return gmtime_r(&now, &utc) != nullptr &&
         strftime(buffer, size, "%Y-%m-%dT%H:%M:%SZ", &utc) > 0;
}

bool locationRefreshDue() {
  if (GPS_REFRESH_SECONDS == 0 || !clockValid || !validEpoch(location.fixTime))
    return true;
  const int64_t now = static_cast<int64_t>(time(nullptr));
  return !validEpoch(now) || now < location.fixTime ||
         now - location.fixTime >= GPS_REFRESH_SECONDS;
}

void saveLocation(double latitude, double longitude, double altitude,
                  uint8_t satellites, int64_t epoch) {
  if (!std::isfinite(latitude) || !std::isfinite(longitude) ||
      !std::isfinite(altitude) || fabs(latitude) > 90 ||
      fabs(longitude) > 180 || !validEpoch(epoch)) {
    Serial.println("Location: rejected invalid GNSS fix.");
    return;
  }

  locationUpdatedThisWake = true;
  location = {};
  location.latitude = latitude;
  location.longitude = longitude;
  location.altitude = altitude;
  location.satellites = satellites;
  location.fixTime = epoch;
  location.crc = dataCrc(&location, offsetof(Location, crc));

  Preferences p;
  if (p.begin("odrade-nav", false)) {
    if (p.putBytes("location", &location, sizeof(location)) != sizeof(location))
      Serial.println("Location: could not save position for the next power cycle.");
    p.end();
  } else {
    Serial.println("Location: could not open storage to save position.");
  }
  // Location acquisition and clock synchronization are independent.
}

void addLocationMetadata(JsonObject sample) {
  char timestamp[25] = {};
  const bool valid = formatGpsTimestamp(timestamp, sizeof(timestamp));
  const int64_t now = valid ? static_cast<int64_t>(time(nullptr)) : 0;

  sample["timestampValid"] = valid;
  if (valid) {
    sample["ts"] = timestamp;
    sample["epoch"] = now;
  } else {
    sample["ts"] = nullptr;
    sample["epoch"] = nullptr;
    Serial.println("Time: no synchronized clock; storing ts=null (not fabricated).");
  }
  sample["timestampSource"] = valid ? "GNSS/modem UTC; ESP32 retained clock" : "unknown";

  // Keep prior behavior: a stored location can be reused only while location
  // caching is enabled; an always-refresh device needs a fix this wake.
  const bool haveLocation = validEpoch(location.fixTime) &&
                            (GPS_REFRESH_SECONDS != 0 || locationUpdatedThisWake);
  const bool fresh = haveLocation && valid &&
                     (locationUpdatedThisWake || !locationRefreshDue());
  sample["locationValid"] = haveLocation;
  sample["locationFresh"] = fresh;

  if (haveLocation) {
    char point[64];
    snprintf(point, sizeof(point), "POINT(%.6f %.6f)",
             location.longitude, location.latitude);
    sample["loc"] = point;
    sample["alt"] = location.altitude;
    sample["satCnt"] = location.satellites;
    sample["locationFixEpoch"] = location.fixTime;
    sample["locationAgeSeconds"] = valid ? now - location.fixTime : 0;
    if (!valid) sample["locationAgeSeconds"] = nullptr;
  } else {
    sample["loc"] = nullptr;
    sample["alt"] = nullptr;
    sample["satCnt"] = 0;
    sample["locationFixEpoch"] = nullptr;
    sample["locationAgeSeconds"] = nullptr;
  }

  JsonDocument navigation;
  for (const char *key : {"ts", "loc", "alt", "satCnt"})
    navigation[key] = sample[key];
  Serial.print("Payload GPS fields: ");
  serializeJson(navigation, Serial);
  Serial.println();
}
