#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/README.md"
# Odrade SensorShield

One Arduino sketch (`SensorShield.ino`) coordinates the existing sensor drivers,
SD storage, Walter HTTPS, and 15-minute deep-sleep cycles. There are no JavaScript
upload scripts. The latest implementation replaces continuous eight-sample RAM
batching with **one persistent record per wake**.

## What the user sees

After a two-second pause on manual starts for opening Serial (9600 baud), messages describe
starting, measuring, saving, connection attempts, server confirmation, and resting.
Each air sensor prints its first reading. The particle sensor prints once and
then works silently. Detailed payload JSON is optional (`PRINT_PAYLOAD_JSON`).
The installed BSEC library has its own diagnostic prints; these can still appear
between the application's plain-language messages. ESP32 core debug output also
depends on the Arduino board's Core Debug Level setting. The project VS Code
configuration now uses `DebugLevel=none`; select Core Debug Level: None in Arduino
IDE as well if it has a separate saved board configuration.

The current power-pin sequence and its settling delays remain unchanged; scheduled timer wakes skip the two-second Serial-opening delay. No ten-second
health-check phase was added. Measurement ends when all eight BME sensors and the
BMV have a reading, or after 30 seconds. A timeout saves a clearly marked partial
record instead of inventing missing readings. Sensor failures do not erase data.

## Hardware and files

| Item | Configuration |
| --- | --- |
| External sensor power | GPIO0, active LOW |
| Shared SPI | SCK2, MISO42, MOSI41 |
| SD card | CS38, confirmed by the supplied schematics; 10 MHz SD transfers for board testing |
| BMV080 | CS1, 1 MHz, existing 64 KB worker task |
| BME sensors | Existing CS list in `board_pins.h`, eight independent BSEC instances |
| INA3221 | I2C39/40, addresses 0x40 and 0x41 |
| Internal modem reset | GPIO45, as defined by WalterModem; not an external sensor pin |

| Source | Responsibility |
| --- | --- |
| `SensorShield.ino` | Wake/measure/save/upload/sleep sequence and BSEC callback |
| `LoggerConfig.h` | Timing, batch limits, SD speed and debug output |
| `DataLogger.cpp/.h` | Readable open/save/upload/close flow |
| `LogStorage.cpp/.h` | SD recovery, checksums, indexes and archive cleanup |
| `QueueRules.h` | CRC and strict server acknowledgment validation |
| `Modem.cpp/.h` | Existing Walter LTE/HTTPS calls, occasional GNSS and confirmed shutdown |
| `GpsClock.cpp/.h` | GNSS-derived UTC, retained ESP32 clock, last-known location |
| `AirSensors`, `ParticleSensor`, `PowerSensors`, `Charger`, `Display` | Existing device drivers and lifecycle helpers |

SD access occurs before sensor tasks start or after the BMV task and BME acquisition have stopped. The
BME/BMV SPI mutex remains in place. The card is never automatically formatted.

## Reading the logger code

Start with `DataLogger.cpp`. `beginDataLogger()` opens the card, and
`saveSample()` follows four steps: build a JSON line, append it, save its file
position, and remember the next sequence number. `prepareUpload()` reads the
oldest waiting records; `acknowledgeUpload()` records the server confirmation.
`closeDataLogger()` releases the card before sleep.

The file operations use SdFat's normal `SdFs`, `FsFile`, `open()`, `write()`,
`sync()` and `close()` calls. Compare them with the library's
[ReadWrite example](https://github.com/greiman/SdFat/blob/master/examples/examplesV1/ReadWrite/ReadWrite.ino).
Directory recovery uses the `openNext()` loop from its
[OpenNext example](https://github.com/greiman/SdFat/blob/master/examples/OpenNext/OpenNext.ino).
Odrade checks write results and calls `sync()` before advancing the queue.

`LogStorage.cpp` contains the additional recovery work that a basic SD example
does not provide. A checkpoint remembers upload progress; an index remembers
where each reading starts in a file; a checksum detects damaged data. These
remain necessary for interrupted writes, quick boots and safe cleanup. Existing
SD files keep the same format. Timing and batch settings remain in `LoggerConfig.h`.

## Durable queue

- `/odrade/samples.jsonl`: current append-only log. Each line wraps the existing
  JSON payload with `seq`, UTC `day` (or zero when unknown), and a CRC32.
- `/odrade/samples.idx`: fixed-size checksummed sequence/offset/length entries.
- `/odrade/day_<firstSeq>.jsonl` and `.idx`: closed daily archives. Without valid
  UTC, an undated archive closes after 288 readings; it is not assigned a false date.
- `/odrade/upload_state.json` and `/odrade/upload_state.bak`: alternating checkpoint
  slots. Both contain `version`, `generation`, `lastAckedSeq`, `nextSeq`, retry/wake
  counters, and CRC32. **The valid slot with the highest generation is current**;
  the `.bak` slot may be newer than `.json`.

Writes are ordered: append record and sync; append index and sync; commit and
verify the alternate checkpoint. Boot validates both checkpoint slots, each
segment's indexed boundaries, and any unindexed tail. It rebuilds a missing or
broken index only for that segment. Normal boot reads boundary records rather
than scanning all samples. Upload reads and validates every selected record.
An unfinished final line is truncated; a complete corrupted record, sequence gap,
or unrecoverable checkpoint stops the queue for inspection rather than discarding
unacknowledged records. SD-card controller/FAT corruption under physical power
loss still needs hardware testing; checksums and sync cannot guarantee media integrity.

Sequence numbers remain monotonic within the recovered card log. Keep the card
with its device. Replacing or erasing a card is a new queue and requires a server-side
provisioning/reset procedure to avoid reusing `(device, seq)` identities.

## Upload policy

MVP flow: measure, save to SD, attempt one upload, then sleep until the next
15-minute cycle. Every successful save triggers an upload attempt when the endpoint
is configured. There is no retry backoff or minimum-record threshold. Low, unavailable or fault-reporting
battery telemetry does not block transmission. The web app receives the measured
values, `chargerValid`, `batteryReadingSuspect`, and `bmsF0`/`bmsF1` fault registers.
Unavailable readings remain null with validity flags rather than invented zeros.

One oldest-first batch is attempted per wake, up to eight records or 48,000 bytes.
A failed upload leaves readings on SD for the next measurement cycle. Old retry
fields remain in the checkpoint format so existing cards can still be read, but
no longer delay uploads. A valid server acknowledgment is still required before
marking records uploaded.

`Modem.cpp` sends to `https://odrade.vercel.app/api/data`. The API must implement
the acknowledgment contract in `Upload_API.md`. The firmware installs GTS Root R1
from `ServerCertificate.h` into application
certificate slot 12 for `odrade.vercel.app`; HTTPS validates both hostname and CA.
Review the CA when changing hosts. Soracom uses APN `soracom.io`, username `sora`,
password `sora`, and PAP authentication. The connection follows the Walter HTTPS
example using WalterModem 1.6.1 HTTP events and `httpReceive()`.
A checked `AT+CGAUTH` command through Walter works around that version's
authentication setter returning early when its previous mode is NONE.
Network registration stops waiting as soon as connected, with a 3-minute maximum.
Use a working SIM/APN and valid modem time for TLS. The firmware never starts LTE
until the current measurement is saved and an upload is attempted (except initialization needed to put the radio in standby).

HTTP success alone does not acknowledge records. Only an exact device-scoped,
durable contiguous range matching the attempted batch advances `lastAckedSeq`.
Lost responses cause retries; the server must deduplicate by device and sequence.

When free space drops below 1 MiB, remove the oldest **closed, fully acknowledged**
archives until enough space is available. Current-day and unacknowledged data are
protected. If nothing safe can be removed, report storage full and do not claim
that a new reading was saved. The next cycle tries again.

## Location, validity and sleep

GNSS is attempted only on eligible upload wakes when time/location is unavailable
or the last fix is at least 24 hours old. It has a 60-second acquisition limit.
Location, fix time and satellites are checksummed in ESP32 Preferences. The
GNSS-derived ESP32 clock continues through deep sleep; after complete power loss,
time is unknown until a new GNSS fix. A retained location is marked with its age
and freshness. The first records may legitimately contain `ts: null`.

Records preserve BSEC accuracy, BSEC/sensor error codes, per-sensor reading flags,
particle obstruction/range flags, charger faults and timestamp/location validity.
Unavailable measurements are null or absent, not valid zeros. BSEC instances
restart each wake; do not treat low-accuracy IAQ/CO2-equivalent/VOC estimates as
calibrated measurements. Five-minute cold-start operation needs on-device sensor
warm-up and accuracy evaluation.

Before sleep, the BMV task confirms stop, BME devices enter sleep mode, charger
telemetry ADC and screen stop, SD closes, buses end, and the external sensor rail
is disabled and held off. Charging itself remains enabled. The modem enters
minimum functionality using `setOpState(MINIMUM)` and reads that state back.
Sleep proceeds after radio standby is verified, not a full modem power-off.
The first cold boot initializes the modem even if needed only to disable its radio.
A sample-only timer wake leaves a previously verified standby radio untouched.

WalterModem 1.6.1 defaults to 180,000 ms per command attempt and up to three
attempts. FULL uses the library's completion callback so the monitor reports elapsed
waiting time every five seconds. Other library calls remain blocking. Network
registration has a separate three-minute deadline after FULL completes.
The firmware does not issue overlapping commands or sleep with a pending FULL command.
This dependency setting was not modified outside the project. Validate modem failure
latency as part of the power budget before deployment.

Wake-to-wake target is 15 minutes, subtracting awake time. If a cycle overruns,
the next wake is scheduled one second after shutdown. The serial USB disconnect
on deep sleep is expected; it reconnects on wake.

The shutdown event follows the Sequans-family command documented in the
[RYZ024 AT command manual, section 15.10](https://www.renesas.com/en/document/mah/ryz024-modules-command-users-manual?language=en).
Confirm that the installed GM02SP firmware emits this event before deploying;
there is intentionally no unsafe "sleep anyway" timeout. ESP32 pin retention and
sleep use [Espressif's GPIO/sleep APIs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/sleep_modes.html).

## Verification

`python3 tests/run_host_tests.py` compiles the production queue and modem code
against simulated SD/modem adapters. Set `ARDUINOJSON_INCLUDE` if ArduinoJson is
installed elsewhere. Tests cover batching, exact ACK validation, acknowledgment
through sequence 103, HTTP failure/timeout, torn writes and checkpoints, damaged
indexes, bounded startup reads, retention, and shutdown confirmation.

These tests simulate file-level power cuts, not FAT/controller behavior. Before
field use, follow the on-device checks in `Quick_Start_Guide.md`: actual power
cuts, SD remount, server idempotency, GNSS/TLS, 15-minute timing, sensor validity,
and measured sleep current. Firmware compilation is not hardware qualification.

## Startup troubleshooting

Startup messages include elapsed milliseconds and the ESP32 reset reason. The
reset reason is repeated after the power-settling wait so it is easier to catch
when USB reconnects. It describes the last processor reset, not every USB event.
SD startup errors include SdFat's hexadecimal error and detail codes; mounted-card
folder/recovery failures are reported separately.

INA readings remain available for diagnosis, with per-channel `ReadingSuspect`
flags in the payload. Warnings mark non-finite/negative bus voltage, current near
+/-3276 mA (the measurement limit with the configured 0.05-ohm shunts), and
unexpectedly high voltage. The regulated 5 V and 3.3 V channels use 5.5 V and
3.6 V warning limits; other channels use 26 V. These are plausibility checks,
not proof of a faulty supply or a substitute for measurement with a meter.
The existing battery warning also identifies readings outside the expected
2-cell range. BSEC sampling and configuration are unchanged.

## SD startup recovery

The card is still initialized before BME/BMV SPI traffic, as required by
[Espressif's shared-SPI guidance](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sdspi_share.html).
SdFat already performs the low-speed initialization clocks and CMD0 sequence.
Error `0x01` means CMD0/card reset failed; it is not a filesystem-format diagnosis.
See [SdFat's error definitions](https://github.com/greiman/SdFat/blob/2.1.2/src/SdCard/SdCardInfo.h).

Card communication failures get up to three startup attempts, separated by
250 ms. Successful cards get no added wait. SD transfers now use
10 MHz for board testing; this does not change SdFat's initial probing speed or
provide electrical soft-start. SdFat's cleanup stops SPI, so each attempt restores
SCK2/MISO42/MOSI41 and deselects all sensor chips before using CS38.

If startup communication still fails, one final mount attempt occurs after the
particle worker and BME acquisition stop, before saving the current measurement.
A successful retry recovers the queue and saves that measurement normally. A
failed retry reports that the measurement was not saved. Filesystem, checkpoint,
queue corruption and uncertain write failures are not bypassed by this retry.

Test with the card inserted before applying power. Live insertion is not a
supported operating procedure: an uninitialized card can interfere with other
SPI devices even while deselected. A successful run with late insertion does not
prove that startup supply loading is safe. The socket shares sensor power; CS38
selects communication and cannot separately switch the card's supply. Walter's
[datasheet](https://quickspot.io/datasheet/walter_datasheet.pdf) specifies a maximum
250 mA load on its external 3.3 V output. If enabling that shared rail still causes
a reset, the voltage/current behavior needs hardware verification; retrying SPI
cannot correct an overloaded supply.

Validation: `python3 tests/run_host_tests.py` includes transient/absent-card startup,
a successful later retry and save, restored SPI pins, and preservation of damaged
checkpoints, in addition to the power-loss and upload-acknowledgment tests.

## GPS time and location

`GPS_FIX_ENABLED` in `LoggerConfig.h` is `true`. When a fix is due, GPS runs
before sensor collection so the current saved reading includes the new location
and GPS-synchronized timestamp. `GPS_REFRESH_SECONDS = 0` requests a fresh fix on every measurement cycle for
mobile operation. A failed fix retains the last known position internally but reports null location
in this cycle's payload, so the web app does not display an old point as current. Place the GPS antenna
with a clear sky view. Search progress prints every five seconds. The fix search is
limited to three minutes (LTE/time/assistance preparation and modem commands add their own time); a cold
start can take longer and may need another cycle. No fix means cached location
or null, never invented coordinates/time. Set the switch false to disable GPS.

`SD_STARTUP_DELAY_MS` is 500 ms before the first SD attempt. Retries remain as a
fallback. This delay is adjustable and does not guarantee a card will start on
its first attempt.

Sample-only timer wakes leave an already shut-down modem off. A fresh power-on
initializes it once to confirm shutdown before sleep. Uploads still require at
least one queued reading and retry eligibility;
server acknowledgment and TLS checks remain enabled.

## Timing the 15-minute cycle

Every successful measurement is eligible for an upload immediately; the logger
no longer waits for three samples. Normally one new record is sent per wake.
An older backlog may be included in the same batch, oldest first. There is no upload backoff; verified radio standby still precedes sleep.

SD startup now prints the start and elapsed time for card initialization,
filesystem mounting, queue recovery, free-space checking, and checkpoint saving.
The sketch also reports sensor collection, saving, upload-attempt and total awake
time. These timings identify the actual slow stage before further optimization;
they do not impose new timeouts on library calls. Normal sleep subtracts awake
time from 900 seconds to target 15-minute wake-to-wake sampling.

### Reading the GPS and cellular code

`Modem.cpp` follows the [Walter positioning example](https://github.com/QuickSpot/walter-arduino/blob/main/examples/positioning/positioning.ino)
and [Walter HTTPS example](https://github.com/QuickSpot/walter-arduino/blob/main/examples/https/https.ino).

- `myGNSSEventHandler()` copies the GPS fix when it arrives.
- `attemptGNSSFix()` configures GPS, requests a fix, waits for the callback, and saves a valid position and timestamp.
- `refreshLocationIfDue()` adds Odrade's GPS enable switch and cached-location policy.
- `lteConnected()`, `waitForNetwork()`, and `lteConnect()` check registration and connect using Soracom.
- `setupTLSProfile()` installs the server CA and configures TLS; `sendStoredPayload()` configures HTTP and posts the saved JSON.

The main functions use direct library calls and ordinary `if` statements, like
the examples. The project targets WalterModem 1.6.1: GPS uses `WMGNSSFixEvent`
with `setGNSSEventHandler()`, and HTTPS uses `setHTTPEventHandler()` followed by
`httpReceive()` with exactly the length reported in the ring notification.
The callback copies the notification; the main flow reads and validates the reply.

Lessons from the older working cellular sketch: wait two seconds after NO_RF,
check SIM READY, configure Soracom, enable FULL, select a network, activate the
PDP context, attach data service, and require an IP before HTTPS. SIM polling has
a ten-second window, but each individual library command can take longer.
The authentication setter still skips the initial NONE mode in the installed
1.6.1 source; a checked `sendCmd()` applies PAP credentials without modifying the
library. Network events and FULL completion/CME results are printed for diagnosis.
CME -1 means no numeric CME error was returned, not a modem error code.

GPS is enabled before measurement. Odrade keeps its existing
three-minute search window. Like the installed positioning example, it validates
the GPS clock (using LTE time if needed), refreshes missing/expired almanac and
ephemeris assistance, turns off LTE, and requests a fix. Unusable fix results are
retried within the same search window. Assistance events have bounded 60-second
waits. LTE setup, downloads, and command response times are additional preparation
time; the three minutes cover satellite search. Rejected start commands print
modem result/CME diagnostics rather than pretending the search started. No demo
UDP uploads or restart loop are used.
Timeouts, timestamp checks, verified HTTPS acknowledgments, and confirmed modem
shutdown are application requirements retained alongside the example structure.

### Diagnosing HTTPS certificate errors

Before posting, SensorShield writes the endpoint CA into slot 12, then requests
that slot with `AT+SQNSNVR` and requires a certificate response. This confirms
that an entry is readable, not that its bytes or fingerprint were compared.
The log reports which TLS profile and CA slot the modem accepted. Certificate
and hostname validation remain enabled. The installed public API does not expose
structured profile/certificate contents for full readback comparison.

CURL 77 is reported as a CA loading problem. HTTP status 0 is a connection/TLS
failure notification, not a response from the API; no body is fetched for it.
Failed requests never acknowledge queued readings. A successful credential write
and readback alone do not prove that the server handshake will succeed.

### Walter 1.6.1 GPS clock response

`gnssGetUTCTime()` populates `rsp.data.clock.epochTime` without setting the
`CLOCK` response tag in this library version. The firmware therefore checks
command success and a plausible UTC timestamp, matching the positioning example's
use of the timestamp. Requiring the tag incorrectly rejects valid GPS time.
Serial reports the returned Unix seconds and clock retry results. A failed clock
check means satellite search has not started, not that satellites were searched
for three minutes without a fix. Installed libraries are unchanged.

### LTE/GPS handoff order

Adapted from the supplied Walter 1.6.1 positioning example:
1. Start Walter communication and configure GNSS.
2. Validate GPS time; connect LTE if time synchronization is needed.
3. Refresh missing or expired satellite assistance while LTE is available.
4. Set MINIMUM, wait for network NOT_SEARCHING, and verify radio standby.
5. Request GPS, with up to five attempts sharing a 180-second search window;
   print elapsed search time every five seconds.
6. Collect sensors and save the sample with the new fix (or null mobile location).
7. Reconnect LTE using NO_RF → Soracom PDP/auth → FULL → automatic registration;
   activate data, POST HTTPS, verify acknowledgment, then return to MINIMUM for sleep.

Unlike the unbounded example, disconnection waits at most 30 seconds. If it cannot
be confirmed, GPS does not start. Preparation/command waits are additional to the
satellite-search budget. The library and sensor acquisition code are unchanged.

### Adaptation of the working Soracom plan-US positioning sketch

Modem startup now explicitly selects LTE-M. PDP context 1 explicitly uses IPv4
and Soracom CHAP (`soracom.io`, `sora`, `sora`). The library authentication API is
called, followed by a checked CHAP `AT+CGAUTH` command because installed 1.6.1
returns early for a fresh context with auth NONE. No installed library is edited.
Assistance event waits allow 60 seconds and status is queried again afterward.
GPS results print satellite counts and estimated uncertainty. GPS preparation
failure also triggers radio standby cleanup before sensor acquisition where
possible; a failed cleanup is reported and active GPS blocks LTE uploads.
The sensor sketch still measures after GPS, saves locally, then reconnects LTE
for its existing verified HTTPS POST. It does not send to the example UDP server.
The earlier requested 180-second total satellite-search budget remains, rather
than five separate 120-second waits from the standalone demonstration.
