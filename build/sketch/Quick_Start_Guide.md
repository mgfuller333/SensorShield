#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/Quick_Start_Guide.md"
# Odrade bring-up

1. Insert a compatible FAT/exFAT SD card. The firmware uses SdFat on CS38,
   SCK2/MISO42/MOSI41. It never formats the card automatically.
2. Compile `SensorShield.ino` with the existing ESP32-S3 settings and installed
   libraries. Keep exactly one `.ino`; other implementation files are `.cpp/.h`.
3. Open Serial at 115200 baud. Manual starts pause for two seconds; timer wakes skip
   that pause. Startup reports each stage and SD operation duration.
4. Confirm all eight air sensors and the particle sensor produce data. A partial
   measurement identifies missing sensors instead of reporting them as valid zeros.
5. Look for `Saved: reading ... is safely on the SD card`. The configured endpoint receives an upload attempt after each saved sample when
   retry checks permit. GPS requests a fresh fix before every measurement cycle for mobile operation.
6. Expect `Connection: modem shutdown confirmed`, then `Resting: ...`. USB Serial
   disconnecting during deep sleep is normal. Wake-to-wake target is 15 minutes.
   If the shutdown event is absent, sleep is deliberately postponed.

The first cold boot starts the modem once to shut it down. Subsequent sample-only
wakes leave it off. GPIO45 is the existing internal modem reset from WalterModem;
GPIO38 is the SD select from your schematics. Other existing pin assignments remain.

## Building and flashing from VS Code

Open the `SensorShield` folder and close Serial Monitor in all windows before
flashing. Open the Command Palette, choose **Tasks: Run Task**, then
**SensorShield: Build and Upload**. This task compiles once and passes those
binaries directly to Arduino CLI for uploading, bypassing the extension's
build-status checks. The terminal shows when compilation ends and flashing begins.

**SensorShield: Build only** compiles without flashing. **SensorShield: Upload
built firmware** flashes that build without compiling again; it stops if the
project source or board settings have changed. Board and USB port settings come
from `.vscode/arduino.json`. Select the current port there if reconnecting the
board gives it a different port name. Build files are in `build/direct-upload`.

## Before enabling uploads

- Confirm the hostname in `Modem.cpp` (`odrade.vercel.app`); path is `/api/data`.
- Implement the exact durable acknowledgment in `Upload_API.md`, including
  idempotent replay. An empty 200/204 response is not enough.
- Provision the correct server CA in modem certificate slot 0, or change
  `TLS_CA_CERTIFICATE_SLOT`. HTTPS validates CA and hostname.
- Verify the SIM/APN and modem time. Battery readings and diagnostic flags are
  uploaded even when low, unavailable or faulty so the web app can report them.
- Adjust sampling, batch size, SD speed and debugging in `LoggerConfig.h`.

## On-device acceptance checks

- Three ordinary 15-minute wakes create three sequential SD records. With a
  valid network and successful acknowledgments, each wake sends its one
  new record. Failed uploads stay queued for a later eligible wake. GPS stays off.
- A matching acknowledgment of 101–103 advances the current checkpoint to 103.
  The highest valid generation across `.json` and `.bak` is current.
- A 500 response, wrong device/range, missing body or timeout must leave the
  acknowledgment unchanged. Check that the next allowed wake retries the records.
- Cut power after a save, during upload, and during checkpoint updates. Confirm
  saved IDs survive, no acknowledged range is invented, and replays are deduplicated.
- Remove an index on a test card: its segment should rebuild. Corrupt a complete
  record: storage should report an error and preserve it for inspection.
- Fill a test card below 1 MiB free. Only fully uploaded closed archives may be
  removed. If every archive contains pending records, expect `Storage: full`.
- Verify SD closure and modem shutdown confirmation precede deep sleep. Measure
  actual board current; library calls alone do not establish a power budget.
- Check UTC/location metadata after complete power loss and after a daily fix.
  Before a new fix, current time is unknown. Cached location remains identifiable
  by its original timestamp and freshness flag.

Host regression tests: `python3 tests/run_host_tests.py`. They simulate file
operations, acknowledgments and modem events; they do not replace these hardware checks.

The BSEC library and ESP32 core may print technical diagnostic lines independently
of application messages. `PRINT_PAYLOAD_JSON` enables full per-record JSON for debugging.
