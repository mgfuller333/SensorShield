#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/tests/README.md"
Run `python3 tests/test_gps_payload.py` from this project.

The test compiles the current GPS clock/metadata implementation, GPS result
acceptance block, SD record serializer/checksum validator, and upload preparation
function with the installed ArduinoJson headers. Only platform services (clock,
NVS, serial and indexed SD reads) are simulated. Arduino String operations used
by the record serializer are adapted to their standard C++ equivalents.

It verifies a simulated fix reaches a newly created sensor payload and survives
SD serialization and upload, with ISO-8601 UTC `ts`, longitude-first WKT `loc`,
numeric altitude and satellite count. It includes the user's 20,000,000-meter
uncertainty example, which must not be rejected based on uncertainty.

It also reproduces eight older records with null GPS fields preceding a fresh
record. The first upload contains those eight old records; after their simulated
acknowledgment, the next upload contains the fresh GPS-equipped record. Existing
records are never rewritten with a newer position/time.

This does not test satellite reception, Walter's UART parser, physical SD writes,
or a live server acknowledgment. Serial's `Payload GPS fields:` line describes
the current measurement; an upload may contain earlier queued measurements.
