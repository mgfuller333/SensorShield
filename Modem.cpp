#include "Display.h"
// Adapted flow: Walter examples/positioning/positioning.ino and examples/https/https.ino.
// Uses the installed WalterModem 1.6.1 APIs; see README.md for example links.
#include "Modem.h"
#include "GpsClock.h"
#include "LoggerConfig.h"
#include "ServerCertificate.h"
#include <atomic>
#include <cmath>
#include <esp_sleep.h>
#include <esp_timer.h>
#include <time.h>

// Configure the real server and its trust policy before deployment.
const char *HTTP_HOST = "gemini-ai-chatbot-swart-tau.vercel.app";
const char *HTTP_PATH = "/api/data";
// Keep the connection sequence close to Walter's examples/https/https.ino.
// TLS profile 1 is reserved for BlueCherry in the library example.
const uint8_t HTTP_PROFILE = 1, TLS_PROFILE = 2;
const uint8_t PDP_CONTEXT = 1;
const char *APN = "soracom.io";
const char *APN_USERNAME = "sora";
const char *APN_PASSWORD = "sora";
const uint32_t NETWORK_TIMEOUT_MS = 180000;
WalterModem modem;
bool modemStarted = false;
bool gnssPending = false;
// Remember verified radio standby across timer wakes. A cold boot clears this.
RTC_DATA_ATTR bool radioInStandby = false;
String modemStatus = "Radio off";
portMUX_TYPE fixLock = portMUX_INITIALIZER_UNLOCKED;
WMGNSSFixEvent latestGnssFix = {};
bool gnssFixReceived = false;
int64_t gnssFixReceivedUs = 0;
std::atomic<int> gnssAssistanceReceived{-1};
bool lteConnected();
bool lteConnect();
bool lteDisconnect();

String getModemStatus() { return modemStatus; }
bool httpsEndpointConfigured() { return HTTP_HOST[0] && strcmp(HTTP_HOST, "YOUR_API_HOST") != 0; }

const char *networkDescription(WalterModemNetworkRegState state)
{
  switch (state)
  {
  case WALTER_MODEM_NETWORK_REG_REGISTERED_HOME:
    return "connection successful - using the SIM provider's network";
  case WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING:
    return "connection successful - using a partner network (roaming)";
  case WALTER_MODEM_NETWORK_REG_SEARCHING:
    return "searching for a mobile network";
  case WALTER_MODEM_NETWORK_REG_NOT_SEARCHING:
    return "not connected; network search is not running";
  case WALTER_MODEM_NETWORK_REG_DENIED:
    return "connection refused by the network; check SIM activation and service";
  case WALTER_MODEM_NETWORK_REG_UNKNOWN:
    return "connection status temporarily unavailable; waiting for an update";
  default:
    return "waiting for a recognized network status";
  }
}

void myNetworkEventHandler(WMNetworkEventType event, const WMNetworkEventData *data, void *)
{
  if (event == WALTER_MODEM_NETWORK_EVENT_REG_STATE_CHANGE)
    Serial.printf("Connection: %s.\n", networkDescription(data->cereg.state));
}

// The callback only copies the notification. Read the body in sendStoredPayload().
WMHTTPEventData httpReply = {};
bool httpReplyReady = false;
std::atomic<int> httpConnectionError{0};
void printHttpsError(int code)
{
  if (code == 77)
    Serial.printf("HTTPS: modem cannot load the CA certificate (CURL 77). Check certificate slot %u and TLS profile %u.\n",
                  TLS_CA_CERTIFICATE_SLOT, TLS_PROFILE);
  else
    Serial.printf("HTTPS: connection failed (CURL %d). Readings remain saved.\n", code);
}

void myHTTPEventHandler(WMHTTPEventType event, const WMHTTPEventData *data, void *)
{
  if (data->profile_id != HTTP_PROFILE)
    return;
  if (event == WALTER_MODEM_HTTP_EVENT_RING)
  {
    portENTER_CRITICAL(&fixLock);
    httpReply = *data;
    httpReplyReady = true;
    portEXIT_CRITICAL(&fixLock);
  }
  else if (event == WALTER_MODEM_HTTP_EVENT_CONNECTED ||
           event == WALTER_MODEM_HTTP_EVENT_CONNECTION_CLOSED)
  {
    if (data->rc != 0)
    {
      httpConnectionError = data->rc;
      printHttpsError(data->rc);
    }
    else if (event == WALTER_MODEM_HTTP_EVENT_CONNECTED)
      Serial.println("HTTPS: secure connection to the server established.");
    else
      Serial.println("HTTPS: connection to the server closed.");
  }
}

bool setupModem()
{
  if (modemStarted)
    return true;
  Serial.printf("[Modem %lu ms] Starting Walter modem: begin().\n", (unsigned long)millis());
  if (!modem.begin(&Serial2, 0))
  {
    modemStatus = "Unavailable";
    Serial.println("Connection: modem initialization failed.");
    setDisplayStatus(MODEM_STATUS, "failed");
    return false;
  }
  Serial.println("Connection: modem initialized. Checking communication...");
  if (!modem.checkComm())
  {
    modemStatus = "Unavailable";
    Serial.println("Connection: modem did not answer the communication check.");
    setDisplayStatus(MODEM_STATUS, "failed");
    return false;
  }
  modem.setNetworkEventHandler(myNetworkEventHandler);
  modem.setHTTPEventHandler(myHTTPEventHandler);
  // Match the working Soracom positioning setup: select LTE-M explicitly.
  WalterModemRsp radio = {};
  if (!modem.getRAT(&radio)) {
    Serial.println("Connection: could not read the radio technology; modem setup failed.");
    setDisplayStatus(MODEM_STATUS, "failed");
    return false;
  }
  if (radio.data.rat != WALTER_MODEM_RAT_LTEM) {
    Serial.println("Connection: selecting LTE-M for Soracom.");
    if (!modem.setRAT(WALTER_MODEM_RAT_LTEM)) {
      Serial.println("Connection: could not select LTE-M.");
      setDisplayStatus(MODEM_STATUS, "failed");
      return false;
    }
  }
  Serial.println("Connection: LTE-M radio technology selected.");
  modemStarted = true;
  modemStatus = "Ready";
  Serial.println("Connection: modem communication OK.");
  return true;
}

// The callback runs on the modem task. Copy under a lock; process it below.
void myGNSSEventHandler(WMGNSSEventType event, const WMGNSSEventData *data, void *)
{
  if (!data) return;
  if (event == WALTER_MODEM_GNSS_EVENT_ASSISTANCE) {
    gnssAssistanceReceived = int(data->assistance);
    return;
  }
  if (event != WALTER_MODEM_GNSS_EVENT_FIX) return;
  portENTER_CRITICAL(&fixLock);
  latestGnssFix = data->gnssfix;
  gnssFixReceivedUs = esp_timer_get_time();
  gnssFixReceived = true;
  portEXIT_CRITICAL(&fixLock);
}

// Follow Walter's positioning example: validate GPS time, then refresh assistance.
void printGpsCommandError(const char *step, const WalterModemRsp &rsp) {
  int cme = rsp.type == WALTER_MODEM_RSP_DATA_TYPE_CME_ERROR ? int(rsp.data.cmeError) : -1;
  Serial.printf("Location: %s failed. Modem result=%d, CME=%d (-1 means no detailed error).\n",
                step, int(rsp.result), cme);
}

bool validateGNSSClock() {
  WalterModemRsp rsp = {};
  Serial.println("Location: checking the Walter GNSS/modem UTC clock.");

  // A valid modem UTC time is useful even BEFORE a position fix. Previously
  // this function returned true without synchronizing the ESP32 clock,
  // leaving sensor records with ts=null until an actual GNSS fix arrived.
  auto synchronizeFromModem = [&]() -> bool {
    rsp = {};
    const bool received = modem.gnssGetUTCTime(&rsp);
    const int64_t receivedUs = esp_timer_get_time();
    const int64_t epoch = static_cast<int64_t>(rsp.data.clock.epochTime);
    if (!received || epoch < 1577836800LL || epoch >= 4102444800LL)
      return false;

    setGpsTime(epoch, receivedUs);
    char iso[25];
    if (!formatGpsTimestamp(iso, sizeof(iso))) {
      Serial.println("Location: modem UTC was returned but ESP32 clock synchronization failed.");
      return false;
    }
    Serial.printf("Location: synchronized measurement clock from modem UTC: %s.\n", iso);
    return true;
  };

  if (synchronizeFromModem()) return true;

  Serial.println("Location: modem UTC not yet available. Trying LTE for network time.");
  if (!lteConnected() && !lteConnect()) {
    Serial.println("Location: LTE unavailable; no new modem UTC time acquired.");
    return false;
  }

  for (int i = 0; i < 5; ++i) {
    delay(2000);
    if (synchronizeFromModem()) return true;
    Serial.printf("Location: waiting for modem UTC (%d/5); network time may not be supported.\n", i + 1);
  }
  Serial.println("Location: modem UTC unavailable; a GNSS fix may still provide time later.");
  return false;
}

void updateGNSSAssistance() {
  WalterModemRsp rsp = {};
  if (!modem.gnssGetAssistanceStatus(&rsp) ||
      rsp.type != WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA) {
    Serial.println("Location: assistance status unavailable; trying GPS without a download.");
    return;
  }
  for (auto type : {WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC,
                    WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS}) {
    auto info = rsp.data.gnssAssistance[type];
    if (info.available && info.timeToUpdate > 0) {
      Serial.println("Location: satellite assistance is already current.");
      continue;
    }
    if (!lteConnected() && !lteConnect()) return;
    const char *name = type == WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC ? "almanac" : "ephemeris";
    Serial.printf("Location: downloading satellite %s assistance.\n", name);
    gnssAssistanceReceived = -1;
    WalterModemRsp download = {};
    if (!modem.gnssUpdateAssistance(type, &download)) {
      printGpsCommandError("satellite assistance download", download);
      continue;
    }
    uint32_t started = millis(), progress = started;
    while (gnssAssistanceReceived != int(type) && millis() - started < 60000) {
      if (millis() - progress >= 5000) {
        Serial.printf("Location: waiting for %s assistance (%lu seconds).\n", name,
                      (unsigned long)((millis() - started) / 1000));
        progress = millis();
      }
      refreshDisplay();
    delay(50);
    }
    Serial.println(gnssAssistanceReceived == int(type) ?
                   "Location: satellite assistance updated." :
                   "Location: assistance wait expired; continuing with available GPS data.");
  }
  WalterModemRsp updated = {};
  if (modem.gnssGetAssistanceStatus(&updated) &&
      updated.type == WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA) {
    Serial.printf("Location: assistance check complete; almanac %s, ephemeris %s.\n",
      updated.data.gnssAssistance[WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC].available ? "available" : "unavailable",
      updated.data.gnssAssistance[WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS].available ? "available" : "unavailable");
  } else {
    Serial.println("Location: could not recheck satellite assistance; attempting a fix with available data.");
  }
}

// Print in the main loop, outside the callback/lock, so UART output does not
// block Walter's modem event task. These are the library-decoded fix fields.
void printGnssFix(const WMGNSSFixEvent &fix) {
  const char *status = "unrecognized status";
  switch (fix.status) {
    case WALTER_MODEM_GNSS_FIX_STATUS_READY: status = "result ready"; break;
    case WALTER_MODEM_GNSS_FIX_STATUS_STOPPED_BY_USER: status = "search stopped by a stop command"; break;
    case WALTER_MODEM_GNSS_FIX_STATUS_NO_RTC: status = "GPS clock unavailable"; break;
    case WALTER_MODEM_GNSS_FIX_STATUS_LTE_CONCURRENCY: status = "GPS blocked by simultaneous LTE activity"; break;
  }
  Serial.println("\n----- GPS FIX EVENT (decoded by Walter library) -----");
  Serial.printf("Status: %s (code %d); fix ID: %u\n", status, int(fix.status), fix.fixId);
  Serial.printf("GPS timestamp: %lld Unix seconds; time to fix: %lu ms\n",
                (long long)fix.timestamp, (unsigned long)fix.timeToFix);
  if (fix.timestamp >= 1577836800LL && fix.timestamp < 4102444800LL) {
    time_t seconds = time_t(fix.timestamp);
    struct tm utc;
    char iso[25];
    if (gmtime_r(&seconds, &utc) && strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%SZ", &utc))
      Serial.printf("Reported fix time in BigQuery UTC format: %s\n", iso);
  }
  Serial.printf("Reported coordinates as WKT: POINT(%.6f %.6f)\n",
                fix.longitude, fix.latitude);
  Serial.printf("Latitude: %.9f; longitude: %.9f; altitude: %.3f m\n",
                fix.latitude, fix.longitude, fix.height);
  Serial.printf("Estimated horizontal uncertainty: %.3f m (reported only; not filtered)\n",
                fix.estimatedConfidence);
  Serial.printf("Velocity north/east/down: %.3f / %.3f / %.3f m/s\n",
                fix.northSpeed, fix.eastSpeed, fix.downSpeed);
  Serial.printf("Satellite entries reported: %u (this does not mean a valid position)\n", fix.satCount);
  size_t capacity = sizeof(fix.sats) / sizeof(fix.sats[0]);
  for (size_t i = 0; i < fix.satCount && i < capacity; ++i) {
    Serial.printf("  Satellite %u: signal %u dB-Hz (%s)\n",
                  fix.sats[i].satNo, fix.sats[i].signalStrength,
                  fix.sats[i].signalStrength >= 30 ? "strong by example threshold" : "below example threshold");
  }
  if (fix.satCount > capacity) Serial.println("Satellite count exceeds library array capacity; output truncated safely.");
  if (fix.status != WALTER_MODEM_GNSS_FIX_STATUS_READY)
    Serial.println("Rejected: modem did not report a ready result.");
  if (!fix.satCount) Serial.println("Rejected: no satellite entries reported.");
  if (!std::isfinite(fix.latitude) || !std::isfinite(fix.longitude) ||
      !std::isfinite(fix.height) || fabs(fix.latitude) > 90 || fabs(fix.longitude) > 180)
    Serial.println("Rejected: coordinates or altitude are invalid.");
  if (fix.timestamp < 1577836800LL || fix.timestamp >= 4102444800LL)
    Serial.println("Rejected: GPS timestamp is outside the accepted 2020-2099 range.");
  Serial.println("----- END GPS FIX EVENT -----");
}

// Positioning example: configure GNSS, request a fix, wait for its callback.
// Up to five attempts share a three-minute search budget; failed mobile fixes stay null.
bool attemptGNSSFix()
{
  if (!modemStarted)
    return false;
  radioInStandby = false;
  modem.setGNSSEventHandler(myGNSSEventHandler);
  if (!modem.gnssConfig()) {
    Serial.println("Location: GPS configuration failed.");
    return false;
  }
  if (!validateGNSSClock()) return false;
  updateGNSSAssistance();
  // Match positioning.ino: MINIMUM and wait for network disconnection.
  // NO_RF is the LTE configuration step, not the handoff used by the GPS example.
  Serial.println("Location: releasing the cellular radio before GPS (MINIMUM).");
  if (!lteDisconnect()) {
    Serial.println("Location: LTE disconnection was not confirmed; GPS search has not started.");
    return false;
  }
  Serial.println("Location: cellular connection stopped. Requesting a GPS fix.");
  portENTER_CRITICAL(&fixLock);
  gnssFixReceived = false;
  portEXIT_CRITICAL(&fixLock);
  WalterModemRsp startReply = {};
  if (!modem.gnssPerformAction(WALTER_MODEM_GNSS_ACTION_GET_SINGLE_FIX, &startReply))
  {
    printGpsCommandError("starting GPS fix (satellite search did not begin)", startReply);
    return false;
  }
  gnssPending = true;
  Serial.printf("Location: searching for satellites (up to %lu seconds). Place the GPS antenna with a clear view of the sky.\n",
                (unsigned long)(GPS_TIMEOUT_MS / 1000));
  uint32_t start = millis();
  uint32_t lastProgress = start;
  uint8_t attempt = 1;
  Serial.println("Location: GPS attempt 1 of 5 started.");
  while (millis() - start < GPS_TIMEOUT_MS)
  {
    WMGNSSFixEvent fix;
    int64_t receivedUs = 0;
    portENTER_CRITICAL(&fixLock);
    bool available = gnssFixReceived;
    if (available)
    {
      fix = latestGnssFix;
      receivedUs = gnssFixReceivedUs;
      gnssFixReceived = false;
    }
    portEXIT_CRITICAL(&fixLock);
    if (available)
    {
      gnssPending = false;
      // Clock and position are independent: a rejected position must not discard
      // an otherwise plausible GPS timestamp. Account for callback processing delay.
      if (fix.status == WALTER_MODEM_GNSS_FIX_STATUS_READY &&
          fix.timestamp >= 1577836800LL && fix.timestamp < 4102444800LL) {
        setGpsTime(fix.timestamp, receivedUs);
        char utc[25];
        if (formatGpsTimestamp(utc, sizeof(utc)))
          Serial.printf("Time: GPS clock synchronized to %s; payload ts uses UTC.\n", utc);
      }
      printGnssFix(fix); // Print every result before deciding whether to accept it.
      uint8_t goodSatellites = 0;
      for (size_t i = 0; i < fix.satCount && i < sizeof(fix.sats) / sizeof(fix.sats[0]); ++i)
        if (fix.sats[i].signalStrength >= 30) ++goodSatellites;
      Serial.printf("Location: GPS result received; %u satellites, %u with strong signals, estimated uncertainty %.1f m.\n",
                    fix.satCount, goodSatellites, fix.estimatedConfidence);
      if (fix.status == WALTER_MODEM_GNSS_FIX_STATUS_READY && fix.satCount &&
          std::isfinite(fix.latitude) && std::isfinite(fix.longitude) &&
          std::isfinite(fix.height) && fabs(fix.latitude) <= 90 &&
          fabs(fix.longitude) <= 180 && fix.timestamp >= 1577836800LL &&
          fix.timestamp < 4102444800LL)
      {
        saveLocation(fix.latitude, fix.longitude, fix.height, fix.satCount, fix.timestamp);
        Serial.printf("Location: GPS coordinates received. Latitude %.6f, longitude %.6f, altitude %.1f m; %u satellites, estimated uncertainty %.1f m.\n",
                      fix.latitude, fix.longitude, fix.height, fix.satCount, fix.estimatedConfidence);
        Serial.println("Location: valid GNSS position saved; ESP32 timestamp is populated if time synchronization succeeded.");
        return true;
      }
      Serial.println("Location: this GPS attempt did not provide a usable position/time.");
      // Like the example, request another fix after an unsuccessful result.
      // All attempts share this cycle's three-minute search budget.
      if (millis() - start >= GPS_TIMEOUT_MS || attempt >= 5) break;
      ++attempt;
      Serial.printf("Location: requesting GPS attempt %u of 5 (%lu seconds elapsed).\n",
                    attempt, (unsigned long)((millis() - start) / 1000));
      startReply = {};
      if (!modem.gnssPerformAction(WALTER_MODEM_GNSS_ACTION_GET_SINGLE_FIX, &startReply)) {
        printGpsCommandError("requesting another GPS fix", startReply);
        break;
      }
      gnssPending = true;
    }
    if (millis() - lastProgress >= 5000) {
      Serial.printf("Location: still searching for satellites (%lu seconds elapsed).\n",
                    (unsigned long)((millis() - start) / 1000));
      lastProgress = millis();
    }
    refreshDisplay();
    delay(50);
  }
  if (gnssPending && modem.gnssPerformAction(WALTER_MODEM_GNSS_ACTION_CANCEL))
    gnssPending = false;
  Serial.println("Location: no new fix. Mobile measurements report an unknown location instead of an old position.");
  return false;
}

bool refreshLocationIfDue()
{
  if (!GPS_FIX_ENABLED)
  {
    Serial.println("Location: GPS fixes disabled; no new GNSS time acquisition requested. A cold boot may yield ts=null.");
    return true;
  }
  if (!locationRefreshDue())
    return true;

  setDisplayStatus(GPS_STATUS, "gathering");
  bool gotFix = attemptGNSSFix();
  setDisplayStatus(GPS_STATUS, gotFix ? "fix done" : "failed");
  if (!gotFix)
    Serial.println("Location: continuing measurement without a new GPS fix.");
  // Clock/assistance preparation can fail while LTE is still on. Put it back
  // in standby before sensor acquisition, and cancel any unfinished GPS search.
  if (gnssPending || !radioInStandby) {
    Serial.println("Location: finishing GPS preparation and returning LTE to standby before sensors.");
    if (!shutdownModem()) return false;
  }
  Serial.println("Location: GPS phase finished; cellular radio is in standby for sensor collection.");
  return true;
}

// 1.6.1 still skips setPDPAuthParams when the previous mode is NONE.
// Its sendCmd now waits for OK, so no custom reply handler is needed.
bool setupSoracomAuthentication()
{
  // Use the example API, then explicitly apply CHAP because 1.6.1 can return
  // success without sending CGAUTH when a fresh context has auth NONE.
  if (!modem.setPDPAuthParams(WALTER_MODEM_PDP_AUTH_PROTO_CHAP,
                            APN_USERNAME, APN_PASSWORD, PDP_CONTEXT)) return false;
  char command[96];
  snprintf(command, sizeof(command), "AT+CGAUTH=%u,2,\"%s\",\"%s\"",
           PDP_CONTEXT, APN_USERNAME, APN_PASSWORD);
  WalterModemRsp reply = {};
  if (!modem.sendCmd(command, "OK", &reply))
  {
    Serial.printf("SIM: authentication setup failed (result %d).\n", (int)reply.result);
    return false;
  }
  Serial.println("SIM: Soracom CHAP authentication accepted.");
  return true;
}

// Use the library callback for this slow command so the monitor stays informative.
std::atomic<int> radioCommandResult{-1};
std::atomic<int> radioCmeError{-1};
void radioEnabled(const WalterModemRsp *reply, void *)
{
  if (reply->type == WALTER_MODEM_RSP_DATA_TYPE_CME_ERROR)
    radioCmeError = (int)reply->data.cmeError;
  radioCommandResult = (int)reply->result;
}

bool enableCellularRadio()
{
  radioCommandResult = -1;
  radioCmeError = -1;
  uint32_t start = millis(), lastProgress = start;
  if (!modem.setOpState(WALTER_MODEM_OPSTATE_FULL, nullptr, radioEnabled))
    return false;
  // Wait for library completion before issuing another command or sleeping.
  while (radioCommandResult == -1)
  {
    if (millis() - lastProgress >= 5000)
    {
      Serial.printf("Connection: waiting for the cellular radio to turn on (%lu seconds).\n",
                    (unsigned long)((millis() - start) / 1000));
      lastProgress = millis();
    }
    refreshDisplay();
    delay(50);
  }
  if (radioCommandResult == WALTER_MODEM_STATE_OK)
    Serial.printf("Connection: cellular radio is on (%lu ms). Ready to search for a network.\n",
                  (unsigned long)(millis() - start));
  else
  {
    Serial.printf("Connection: cellular radio could not turn on (modem result %d, %lu ms).\n",
                  radioCommandResult.load(), (unsigned long)(millis() - start));
    if (radioCmeError >= 0)
      Serial.printf("Technical detail: modem reported CME error %d.\n", radioCmeError.load());
  }
  return radioCommandResult == WALTER_MODEM_STATE_OK;
}

// Same three cellular helpers as Walter's HTTPS and positioning examples.
bool lteConnected()
{
  WalterModemNetworkRegState state = modem.getNetworkRegState();
  return state == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
         state == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING;
}

bool waitForNetwork()
{
  Serial.println("Connection: searching for a mobile network (up to 3 minutes).");
  uint32_t start = millis(), lastProgress = start;
  while (!lteConnected())
  {
    if (millis() - start >= NETWORK_TIMEOUT_MS)
    {
      Serial.println("Connection: network search timed out. Check SIM activation, antenna and coverage.");
      return false;
    }
    if (millis() - lastProgress >= 10000)
    {
      Serial.printf("Connection: %s (%lu seconds elapsed).\n",
                    networkDescription(modem.getNetworkRegState()),
                    (unsigned long)((millis() - start) / 1000));
      lastProgress = millis();
    }
    refreshDisplay();
    delay(500);
  }
  Serial.println("Connection successful: connected to the mobile network. Preparing internet access.");
  return true;
}

// From the working sketch: allow SIM detection after NO_RF, then check READY.
bool waitForSIM()
{
  Serial.println("Connection: allowing two seconds for SIM detection.");
  delay(2000);
  uint32_t start = millis();
  do
  {
    WalterModemRsp reply = {};
    if (!modem.getSIMState(&reply))
    {
      Serial.printf("SIM: status request failed (result %d).\n", (int)reply.result);
      return false;
    }
    if (reply.type == WALTER_MODEM_RSP_DATA_TYPE_SIM_STATE &&
        reply.data.simState == WALTER_MODEM_SIM_STATE_READY)
    {
      Serial.println("SIM: ready.");
      return true;
    }
    if (reply.type != WALTER_MODEM_RSP_DATA_TYPE_SIM_STATE)
      Serial.println("SIM: modem did not provide a readable SIM status.");
    else if (reply.data.simState == WALTER_MODEM_SIM_STATE_PIN_REQUIRED)
      Serial.println("SIM: locked; a SIM PIN is required before connecting.");
    else if (reply.data.simState == WALTER_MODEM_SIM_STATE_PUK_REQUIRED)
      Serial.println("SIM: blocked; contact the SIM provider for the PUK unlock code.");
    else
      Serial.println("SIM: not ready; the SIM may require an unlock code from its provider.");
    refreshDisplay();
    delay(500);
  } while (millis() - start < 10000);
  Serial.println("SIM: readiness check timed out. Check SIM seating or PIN requirements.");
  return false;
}

bool connectMobileData();
bool lteConnect()
{
  setDisplayStatus(MODEM_STATUS, "searching");
  bool connected = connectMobileData();
  setDisplayStatus(MODEM_STATUS, connected ? "connected" : "failed");
  return connected;
}

bool connectMobileData()
{
  radioInStandby = false;
  Serial.println("Connection: 1/5 - turning off radio before configuration (NO_RF).");
  if (!modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF))
  {
    Serial.println("Connection: radio setup failed. Readings remain saved for retry.");
    return false;
  }
  if (!waitForSIM())
    return false;
  Serial.println("Connection: 2/5 - setting Soracom APN: soracom.io.");
  if (!modem.definePDPContext(PDP_CONTEXT, APN, nullptr, nullptr, nullptr, WALTER_MODEM_PDP_TYPE_IP))
  {
    Serial.println("Connection: Soracom APN setup failed. Readings remain saved for retry.");
    return false;
  }
  Serial.println("Connection: 3/5 - applying Soracom authentication.");
  if (!setupSoracomAuthentication())
  {
    Serial.println("Connection: Soracom authentication setup failed. Readings remain saved for retry.");
    return false;
  }
  Serial.println("Connection: 4/5 - enabling cellular radio (FULL).");
  WalterModemRsp reply = {};
  if (!enableCellularRadio())
  {
    Serial.println("Connection: radio startup failed. Readings remain saved.");
    return false;
  }
  Serial.println("Connection: 5/5 - selecting the mobile network automatically.");
  if (!modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC))
  {
    Serial.println("Connection: automatic network selection failed. Readings remain saved for retry.");
    return false;
  }

  if (!waitForNetwork())
    return false;

  // The older working sketch explicitly activated and attached the data context.
  Serial.println("Internet: enabling the SIM data connection (PDP context).");
  if (!modem.setPDPContextActive(true, PDP_CONTEXT))
  {
    Serial.println("Connection: data context activation failed.");
    return false;
  }
  Serial.println("Internet: connecting to the mobile data service.");
  if (!modem.setNetworkAttachmentState(true))
  {
    Serial.println("Connection: data service attachment failed.");
    return false;
  }
  if (!modem.getPDPAddress(&reply, nullptr, nullptr, PDP_CONTEXT) ||
      reply.type != WALTER_MODEM_RSP_DATA_TYPE_PDP_ADDR)
  {
    Serial.println("Connection: could not retrieve the assigned IP address.");
    return false;
  }
  const char *ip = reply.data.pdpAddressList.pdpAddress;
  if (!ip || !ip[0] || strcmp(ip, "0.0.0.0") == 0)
  {
    Serial.println("Connection: no usable IPv4 address; HTTPS postponed.");
    return false;
  }
  Serial.printf("Internet ready: the network assigned this device an address (IP %s).\n", ip);
  return true;
}

bool setupTLSProfile()
{
  // Like Walter's HTTPS example, install the server CA before configuring TLS.
  // This CA matches odrade.vercel.app, rather than the example's httpbin server.
  Serial.println("HTTPS: preparing the certificate used to verify the server identity.");
  if (!modem.tlsWriteCredential(false, TLS_CA_CERTIFICATE_SLOT, SERVER_ROOT_CA))
  {
    Serial.println("Connection: server certificate installation failed. Readings remain saved for retry.");
    return false;
  }
  // Same read command used by WalterModem's internal credential-presence check.
  // This checks readability, not the certificate fingerprint or server chain.
  char command[48];
  snprintf(command, sizeof(command), "AT+SQNSNVR=\"certificate\",%u", TLS_CA_CERTIFICATE_SLOT);
  Serial.printf("HTTPS: checking the saved server certificate can be read (CA slot %u).\n", TLS_CA_CERTIFICATE_SLOT);
  WalterModemRsp reply = {};
  if (!modem.sendCmd(command, "+SQNSNVR: ", &reply))
  {
    Serial.printf("HTTPS: certificate readback failed (modem result %d). Upload stopped; readings remain saved.\n",
                  (int)reply.result);
    return false;
  }
  Serial.println("HTTPS: saved certificate is readable. Setting up secure communication...");
  if (!modem.tlsConfigProfile(TLS_PROFILE, WALTER_MODEM_TLS_VALIDATION_URL_AND_CA,
                              WALTER_MODEM_TLS_VERSION_12, TLS_CA_CERTIFICATE_SLOT))
  {
    Serial.println("Connection: TLS security setup failed. Readings remain saved for retry.");
    return false;
  }
  Serial.printf("HTTPS: security settings accepted; the server identity will be checked (TLS profile %u, CA slot %u, TLS 1.2).\n",
                TLS_PROFILE, TLS_CA_CERTIFICATE_SLOT);
  return true;
}

// Explain the server's answer before looking at the response body size.
void printHttpFailure(int status)
{
  if (status == 404)
    Serial.printf("Upload failed: the server could not find this API address (HTTP 404). Check that POST https://%s%s exists in the deployed web app.\n",
                  HTTP_HOST, HTTP_PATH);
  else if (status == 401 || status == 403)
    Serial.printf("Upload failed: the server requires authorization or refused access (HTTP %d). Check API credentials and permissions.\n", status);
  else if (status == 405)
    Serial.println("Upload failed: this API address does not accept POST requests (HTTP 405). Check the web app route.");
  else if (status >= 500)
    Serial.printf("Upload failed: the server reported a problem (HTTP %d). Check the web app logs.\n", status);
  else if (status >= 300 && status < 400)
    Serial.printf("Upload stopped: the server requested a different address (HTTP %d). Check the configured API URL.\n", status);
  else
    Serial.printf("Upload failed: the server did not accept the request (HTTP %d). Check the API requirements.\n", status);
  Serial.println("Readings remain saved on the SD card; nothing has been marked as uploaded.");
}

bool sendStoredPayload(const String &body, int &httpStatus, String &response)
{
  httpStatus = 0;
  response = "";
  if (!httpsEndpointConfigured() || !modemStarted || gnssPending)
    return false;
  if (PRINT_UPLOAD_JSON)
  {
    Serial.printf("Upload preview: %u bytes of JSON for https://%s%s\n",
                  (unsigned)body.length(), HTTP_HOST, HTTP_PATH);
    Serial.println("----- BEGIN UPLOAD JSON -----");
    Serial.print(body); // The exact string passed to httpSend, including queued records.
    Serial.println();
    Serial.println("----- END UPLOAD JSON -----");
    Serial.flush();
  }
  if (!lteConnect())
  {
    Serial.println("Connection: mobile network unavailable. Saved readings will be retried later.");
    return false;
  }
  if (!setupTLSProfile())
    return false;
  Serial.printf("Upload: destination https://%s%s (HTTPS POST).\n", HTTP_HOST, HTTP_PATH);
  if (!modem.httpConfigProfile(HTTP_PROFILE, HTTP_HOST, 443, TLS_PROFILE))
  {
    Serial.println("Connection: HTTPS server setup failed. Readings remain saved.");
    return false;
  }
  portENTER_CRITICAL(&fixLock);
  httpReplyReady = false;
  portEXIT_CRITICAL(&fixLock);
  httpConnectionError = 0;
  char contentType[64] = {};
  Serial.println("Upload: sending saved readings; waiting for the server to confirm storage.");
  if (!modem.httpSend(HTTP_PROFILE, HTTP_PATH, (uint8_t *)body.c_str(), body.length(),
                      WALTER_MODEM_HTTP_SEND_CMD_POST, WALTER_MODEM_HTTP_POST_PARAM_JSON,
                      contentType, sizeof(contentType)))
  {
    Serial.println("Upload: the modem could not send this batch. Readings remain saved.");
    return false;
  }
  uint32_t start = millis();
  while (millis() - start < 60000)
  {
    WMHTTPEventData received = {};
    portENTER_CRITICAL(&fixLock);
    bool ready = httpReplyReady;
    if (ready)
      received = httpReply;
    portEXIT_CRITICAL(&fixLock);
    if (ready)
    {
      uint8_t buffer[1024] = {};
      size_t length = received.data_len;
      httpStatus = received.status;
      if (httpStatus == 0)
      {
        Serial.println("HTTPS: no HTTP response received (status 0). Connection/TLS failed; readings remain saved.");
        // Do not call httpReceive for an empty failure notification.
        return false;
      }
      if (httpStatus < 200 || httpStatus >= 300)
      {
        printHttpFailure(httpStatus);
        return false;
      }
      Serial.printf("HTTPS successful: the server replied (HTTP %d, %u bytes). Checking its saved-readings confirmation...\n",
                    httpStatus, (unsigned)length);
      if (length == 0 || length >= sizeof(buffer))
      {
        Serial.println("Upload not confirmed: the server reply is empty or exceeds the confirmation buffer. Readings remain saved.");
        return false;
      }
      // 1.6.1 requires the exact ring length, not the buffer capacity.
      if (!modem.httpReceive(HTTP_PROFILE, buffer, length) || memchr(buffer, 0, length))
      {
        Serial.println("Upload: could not read a valid acknowledgment. Readings remain saved.");
        return false;
      }
      buffer[length] = 0;
      response = (const char *)buffer;
      return true; // DataLogger checks HTTP success and the exact acknowledged range.
    }
    if (httpConnectionError != 0)
    {
      printHttpsError(httpConnectionError.load());
      return false;
    }
    refreshDisplay();
    delay(250);
  }
  Serial.println("Upload: server response timed out. Readings remain on the SD card.");
  return false;
}

// Walter's positioning example disconnects with MINIMUM, not AT+SQNSSHDN.
// Read it back so the caller only sleeps after verified radio standby.
bool lteDisconnect()
{
  WalterModemRsp reply = {};
  Serial.printf("[Modem %lu ms] Disabling cellular radio (MINIMUM).\n", (unsigned long)millis());
  if (!modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM, &reply))
  {
    Serial.printf("Connection: could not disable radio (modem result %d).\n", (int)reply.result);
    return false;
  }
  Serial.println("Connection: radio standby requested; waiting for LTE disconnection.");
  // positioning.ino waits for NOT_SEARCHING before requesting GPS.
  // Keep that order, but bound the wait so a missing event cannot hang the board.
  uint32_t disconnectStarted = millis(), progress = disconnectStarted;
  while (modem.getNetworkRegState() != WALTER_MODEM_NETWORK_REG_NOT_SEARCHING) {
    if (millis() - disconnectStarted >= 30000) {
      Serial.println("Connection: LTE disconnection was not confirmed within 30 seconds.");
      return false;
    }
    if (millis() - progress >= 5000) {
      Serial.printf("Connection: waiting for LTE to disconnect (%lu seconds elapsed).\n",
                    (unsigned long)((millis() - disconnectStarted) / 1000));
      progress = millis();
    }
    refreshDisplay();
    delay(100);
  }
  Serial.println("Connection: LTE disconnected. Verifying radio standby.");
  if (!modem.getOpState(&reply) || reply.type != WALTER_MODEM_RSP_DATA_TYPE_OPSTATE)
  {
    Serial.printf("Connection: state check failed (modem result %d).\n", (int)reply.result);
    return false;
  }
  if (reply.data.opState != WALTER_MODEM_OPSTATE_MINIMUM)
  {
    Serial.printf("Connection: the modem has not entered radio standby (reported mode %d).\n", (int)reply.data.opState);
    return false;
  }
  radioInStandby = true;
  modemStatus = "Standby";
  setDisplayStatus(MODEM_STATUS, "disconnected");
  Serial.println("Connection: cellular radio standby confirmed.");
  return true;
}

// One synchronous command, as in Walter's examples. MINIMUM disables the radio;
// it does not remove modem power. The caller sleeps even if the command fails.
bool shutdownModem()
{
  if (!modemStarted) {
    Serial.println("Modem: not initialized; no standby command sent.");
    return false;
  }
  if (gnssPending) {
    if (modem.gnssPerformAction(WALTER_MODEM_GNSS_ACTION_CANCEL))
      gnssPending = false;
    else
      Serial.println("Location: GPS stop command failed.");
  }
  Serial.println("Modem: turning off cellular radio (MINIMUM).");
  bool accepted = modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM);
  if (accepted) {
    radioInStandby = true;
    modemStatus = "Standby";
    Serial.println("Modem: radio standby command accepted.");
  }
  return accepted;
}
