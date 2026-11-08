#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/Upload_API.md"
# Odrade upload contract (version 1)

Firmware destination: `https://odrade.vercel.app/api/rawData`.

**The shared web app's current `/api/rawData` POST is a read/query endpoint.**
Both `hereUp` and `1_2/nextjs-admin-dashboard` accept `{keys, start, end}`, run
`SELECT`, and return chart points. They do not insert readings or return the
acknowledgment below. The supplied Vercel `rows` log is query output, not a
successful device upload. Changing the URL or table schema alone is insufficient.
The existing route needs a separate ingestion branch for an array of reading objects
(or a dedicated ingestion URL configured in the firmware). Keep the chart query
branch working. No web app source or deployed service was modified here.

## Compact upload rows

Each POST is a JSON array of flat readings, using exactly the field list in
`Upload_allowed_keys.json`. This matches the smaller user-provided example, with
one additional field: `seq`. The server uses `(device, seq)` to identify duplicate
uploads and to confirm which records it saved.

```json
[
  {"device":"206EF1E28EE4","seq":101,"iaq_0":50,"temp_0":23.5,"ts":null},
  {"device":"206EF1E28EE4","seq":102,"iaq_0":50,"temp_0":23.6,"ts":null}
]
```

This abbreviated example shows the structure. Actual rows include all five BME
fields (`iaq`, `co2`, `bvoc`, `temp`, `hum`) for sensors 0–7; voltage/current for
solar, USB-C, 12 V, 5 V, 3.3 V and external rails; `pm1`, `pm2_5`, `pm10`, `pmObs`;
`batMv`, `sysMv`, `ibatMa`; and `device`, `ts`, `loc`, `alt`, `satCnt`, `seq`.
Missing selected fields remain JSON null. Numbers and booleans retain their types.
The JSON is serialized without indentation, and Serial prints the exact body.

Detailed health reports, IAQ accuracy, pressure, gas resistance, calculated power,
extra clock metadata and SD checksums are no longer transmitted. They remain in
the full SD record. This also applies when uploading older queued records; the
stored files are not rewritten. The compact cloud data therefore cannot by itself
show all accuracy/fault information; use the full SD record for those diagnostics.

`BigQuery_schema.json` still describes the full collected data; unused nullable
columns can remain in the table. `Upload_allowed_keys.json` is the current HTTP
row allowlist. No new columns are required beyond the previously provided schema.

Voltage is in volts and rail current in milliamps. `batMv`/`sysMv` are millivolts,
`ibatMa` is signed milliamps, PM is micrograms/m3, and temperature is raw Celsius.
BSEC subscriptions are unchanged. GPS is enabled before measurement when due.
If no GPS fix or valid retained clock is available,
`ts` is null; do not invent a measurement timestamp or location.

## Storage acknowledgment

Derive `firstSeq` from the first row's `seq` and `lastSeq` from the last row's
`seq`. Require one device and contiguous sequence IDs. Version 1 below describes
the acknowledgment protocol; it is no longer repeated in each uploaded row.

After durably committing the entire contiguous batch, the API must respond with
HTTP 2xx and this JSON body (numeric sequence IDs, not strings):

```json
{
  "version": 1,
  "device": "206EF1E28EE4",
  "firstSeq": 101,
  "lastSeq": 102,
  "durable": true
}
```

This is the API acknowledgment/callback, returned in the POST response. The
firmware does not expose a public inbound callback server.

The device rejects missing/wrong device IDs, mismatched ranges, missing/false
`durable`, malformed/truncated/oversized bodies, trailing data and non-2xx HTTP
statuses. It only accepts a range beginning at `lastAckedSeq + 1`. Partial ACKs
are deliberately not accepted: retry the same records until the full attempted
range is confirmed. Keep the response below 1023 bytes; the example is under 150.

The backend must:

1. Validate device identity according to the deployed API's authentication policy.
2. Validate contiguous record IDs and matching device IDs.
3. Commit the full batch durably before acknowledging it.
4. Deduplicate by `(device, seq)`. For identical previously committed records,
   return the same acknowledgment. Reject conflicting content for an existing ID.
5. Return the exact range requested, including on replay after a lost response.

The local acknowledgment checkpoint commits only after the body passes these
checks. No HTTP response, timeout, or power loss before checkpoint commit may
remove the local records. A crash after the server commit but before the local
checkpoint can produce a duplicate POST, which is why server idempotency is required.

## Viewing the actual request

`PRINT_UPLOAD_JSON = true` in `LoggerConfig.h` prints the exact serialized HTTP
body between `BEGIN UPLOAD JSON` and `END UPLOAD JSON` before LTE connection.
Copy just the JSON, without Serial Monitor timestamps or the marker lines.
This includes older queued records, not only the current measurement. Each array element contains the sensor keys and its queue metadata directly.
Set the option false after API testing to avoid serial output time and power.
The JSON is intentionally unchanged rather than parsed into a second large copy.

Your table's old temperature/humidity descriptions say compensated, but the
current BSEC subscriptions are RAW_TEMPERATURE and RAW_HUMIDITY. The proposed
schema documents the actual readings; the algorithm remains unchanged.

## GPS conversions

A ready GPS event with a plausible Unix timestamp now synchronizes the measurement
clock independently of position quality. For example, 1794084884 converts to
`2026-11-07T20:54:44Z`. `ts` is the UTC time at measurement serialization, which
can be later than the original fix event. The library exposes integer seconds;
no fractional GPS precision is invented. The main-loop delay since callback receipt
is included when setting the ESP32 clock.

Accepted coordinates become WKT `POINT(longitude latitude)` for GEOGRAPHY.
Altitude remains a numeric value in meters; satellite count remains numeric.
Position uncertainty is printed for diagnosis but does not filter uploads.
A 20,000,000 m uncertainty result is sent using the same WKT conversion as any
other ready result. Basic coordinate/time format checks remain. A missing or
unformattable result may still produce a non-null `ts` with null `loc` and `alt`. This timestamp check validates range/status, not independent UTC accuracy.
Serial prints `Payload GPS fields:` with the exact navigation JSON for the sample.
No BigQuery schema changes are required.
