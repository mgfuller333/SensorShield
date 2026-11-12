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

Charger regression: `python3 tests/test_charger.py` runs the actual charger
configuration, CE hold, and sleep shutdown functions with simulated GPIO/I2C.
It checks the supplied 7.90 V / 2.00 A settings, readback/write failure handling,
LOW output configuration before releasing the wake hold, and preservation of
charging settings while stopping only the ADC. It does not measure physical CE
voltage or battery current.

On the board, verify GPIO10/CE remains near 0 V during sleep and wake, then
connect USB while asleep and check battery charging current. Repeat with solar
input independently. The working example's MPPT-off setting is retained; source
availability and charger protections still control whether charging occurs.
GPIO hold cannot guarantee a LOW during a cold reset or loss of ESP32 power.
The supplied example identifies a REGN pull-up above the ESP32 GPIO voltage
rating; open-drain output and sleep hold do not correct that PCB interface.

### Sensor supply after deep sleep

Run `python3 tests/test_sensor_power.py`. It runs the production
`enableSensorPower()` helper with mocked GPIO holds through cold boot and three
sleep/wake cycles, and checks that setup enables power after SD initialization and before I2C startup. This
tests software sequencing only; measure the actual 3.3 V rail on the board.
