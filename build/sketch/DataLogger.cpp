#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/DataLogger.cpp"
#include "DataLogger.h"
#include "LogStorage.h"
#include "LoggerConfig.h"
#include "QueueRules.h"
#include "board_pins.h"
#include <SPI.h>

// Uses SdFat's begin/open/write/sync/close pattern. Recovery and archive
// bookkeeping live in LogStorage.cpp so the normal flow stays easy to follow.
// SD access happens before sensor tasks start or after they have stopped.
bool cardStartupRetryPending = false;
// RAM only: older SD records are never selected after a restart.
uint64_t currentWakeSeq = 0;
bool currentUploadPrepared = false;

// Call only before sensor tasks start or after they have stopped.
// SdFat end() also stops SPI, so restore our board's pins before EVERY attempt.
void prepareCardBus() {
  for (uint8_t cs : BME_CS) { pinMode(cs, OUTPUT); digitalWrite(cs, HIGH); }
  pinMode(BMV080_CS, OUTPUT); digitalWrite(BMV080_CS, HIGH);
  pinMode(SD_CS, OUTPUT); digitalWrite(SD_CS, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
}

// Start/end messages show where a slow card is spending time.
void printStorageTiming(const char *step, uint32_t startedMs, bool success) {
  Serial.printf("[Storage %lu ms] %s: %s (%lu ms).\n", (unsigned long)millis(),
                step, success ? "done" : "failed", (unsigned long)(millis() - startedMs));
}

bool startStorage(uint8_t attempts) {
  if (storageReady) return true;
  cardStartupRetryPending = false;
  bool mounted = false;
  for (uint8_t attempt = 1; attempt <= attempts; ++attempt) {
    prepareCardBus();
    Serial.printf("Storage: checking SD card (attempt %u/%u).\n", attempt, attempts);
    // SdFat supplies the startup clocks, selects CS38 and sends CMD0 to enter
    // SPI mode. Keep this before any BME/BMV communication on the shared bus.
    uint32_t cardStartedMs = millis();
    bool spiReady = logCard.cardBegin(SdSpiConfig(SD_CS, SHARED_SPI, SD_SPI_CLOCK_HZ, &SPI));
    printStorageTiming("Card initialization", cardStartedMs, spiReady);
    if (spiReady) {
      Serial.println("Storage: SD card initialized in SPI mode (CS38, SCK2, MISO42, MOSI41).");
      Serial.println("Storage: opening the card filesystem...");
      uint32_t mountStartedMs = millis();
      mounted = logCard.volumeBegin();
      printStorageTiming("Filesystem mount", mountStartedMs, mounted);
      if (mounted) break;
      Serial.printf("Storage: SPI mode is ready, but the filesystem could not be opened. Error=0x%02X, detail=0x%02X.\n",
                    logCard.sdErrorCode(), logCard.sdErrorData());
      Serial.println("Storage: check the card's partition and FAT32/exFAT filesystem; files have not been changed.");
      logCard.end();
      prepareCardBus();
      return false;
    }

    uint8_t error = logCard.sdErrorCode();
    Serial.printf("Storage: SD startup failed at %lu ms. SdFat error=0x%02X, detail=0x%02X.\n",
                  (unsigned long)millis(), error, logCard.sdErrorData());
    if (error == SD_CARD_ERROR_CMD0)
      Serial.println("Storage: card did not enter SPI mode (CMD0 failed); no files were opened.");
    else if (error)
      Serial.println("Storage: card communication failed; check power, contacts and SPI wiring.");
    else
      Serial.println("Storage: card did not mount; check its partition and FAT32/exFAT formatting.");

    logCard.end();
    prepareCardBus(); // Keep the shared bus usable even if every attempt fails.
    if (!error) return false; // A filesystem problem is not a power-up retry.
    if (attempt < attempts) delay(SD_RETRY_DELAY_MS);
  }
  if (!mounted) {
    cardStartupRetryPending = true;
    if (attempts > 1) Serial.println("Storage: card still unavailable; one final check will run after measurements stop.");
    return false;
  }
  if (!logCard.exists("/odrade") && !logCard.mkdir("/odrade")) {
    Serial.println("Storage: card mounted, but the logging folder could not be created.");
    return false;
  }
  Serial.println("Storage: validating saved readings and upload progress...");
  uint32_t stepStartedMs = millis();
  bool recovered = recoverStorage();
  printStorageTiming("Queue recovery", stepStartedMs, recovered);
  if (!recovered) {
    Serial.println("Storage: card mounted, but saved log/checkpoint recovery failed; files preserved.");
    return false;
  }
  Serial.println("Storage: checking free space (the card may need an allocation-table scan)...");
  stepStartedMs = millis();
  bool roomAvailable = makeStorageRoom();
  printStorageTiming("Free-space check", stepStartedMs, roomAvailable);
  if (!roomAvailable) return false;

  checkpoint.wakeCount++;
  Serial.println("Storage: saving upload progress...");
  stepStartedMs = millis();
  bool progressSaved = commitState(checkpoint);
  printStorageTiming("Checkpoint save", stepStartedMs, progressSaved);
  if (!progressSaved) return false;
  storageReady = true;
  Serial.println("Storage: ready. Older readings stay on SD; only this wake's reading will upload.");
  return true;
}

bool beginDataLogger() {
  return startStorage(SD_STARTUP_ATTEMPTS);
}

bool retryDataLoggerAfterMeasurements() {
  if (storageReady || !cardStartupRetryPending) return storageReady;
  Serial.println("Storage: sensors have stopped; checking the SD card once more before saving.");
  bool ready = startStorage(1);
  cardStartupRetryPending = false; // Never loop or retry an uncertain write/checkpoint.
  if (!ready) Serial.println("Storage: final check failed; this reading cannot be saved.");
  return ready;
}

bool saveSample(JsonObject sample) {
  currentWakeSeq = 0;
  currentUploadPrepared = false;
  if (!storageReady) return false;

  // 1. Wrap the existing sensor payload with a sequence number and checksum.
  uint64_t seq = checkpoint.nextSeq;
  int64_t day = 0;
  if (sample["timestampValid"].as<bool>()) day = sample["epoch"].as<int64_t>() / 86400;
  String line;
  if (!buildRecord(sample, seq, day, line) || !prepareDailyLog(day)) return false;

  // 2. Append one line. sync() finishes the write before we record progress.
  FsFile file = logCard.open(ACTIVE_LOG, O_WRONLY | O_CREAT | O_APPEND);
  if (!file) return false;
  uint64_t offset = file.fileSize();
  bool saved = file.write(line.c_str(), line.length()) == line.length();
  if (saved) saved = file.write('\n') == 1;
  if (saved) saved = file.sync();
  file.close();
  if (!saved) {
    storageReady = false;
    return false;
  }

  // 3. Save the line's position so uploads can find it quickly.
  FsFile index = logCard.open(ACTIVE_INDEX, O_WRONLY | O_CREAT | O_APPEND);
  bool indexed = index && addIndex(index, seq, offset, line);
  index.close();
  if (!indexed) {
    storageReady = false;
    return false;
  }
  Segment *active = findActiveLog();
  if (!active->first) active->first = seq;
  active->last = seq;
  active->count++;
  active->end = offset + line.length() + 1;
  active->day = day;

  // 4. Only advance the sequence after the reading and index are saved.
  Checkpoint next = checkpoint;
  next.nextSeq++;
  if (!commitState(next)) {
    storageReady = false;
    return false;
  }
  currentWakeSeq = seq;
  Serial.printf("Saved: reading %llu is on SD and ready for one upload attempt.\n", seq);
  return true;
}

bool prepareUpload(const String &device, String &body, uint64_t &first, uint64_t &last) {
  if (!storageReady || !currentWakeSeq || currentUploadPrepared) return false;
  currentUploadPrepared = true; // One attempt per wake, including failed HTTPS.
  first = currentWakeSeq;
  last = first - 1;
  JsonDocument batch;
  // Upload only the compact field list; keep full diagnostics on the SD card.
  const char *uploadKeys[] = {
      "iaq_0", "co2_0", "bvoc_0", "temp_0", "hum_0",
      "iaq_1", "co2_1", "bvoc_1", "temp_1", "hum_1",
      "iaq_2", "co2_2", "bvoc_2", "temp_2", "hum_2",
      "iaq_3", "co2_3", "bvoc_3", "temp_3", "hum_3",
      "iaq_4", "co2_4", "bvoc_4", "temp_4", "hum_4",
      "iaq_5", "co2_5", "bvoc_5", "temp_5", "hum_5",
      "iaq_6", "co2_6", "bvoc_6", "temp_6", "hum_6",
      "iaq_7", "co2_7", "bvoc_7", "temp_7", "hum_7",
      "device", "solV", "solC", "v5V", "v5C", "usbcV",
      "usbcC", "v3_3V", "v3_3C", "v12V", "v12C", "extV",
      "extC", "pm1", "pm2_5", "pm10", "pmObs", "batMv",
      "sysMv", "ibatMa", "ts", "loc", "alt", "satCnt",
  };
  JsonArray records = batch.to<JsonArray>();
  for (Segment &archive : segments) {
    if (!archive.count || archive.last < first) continue;
    uint64_t n = first > archive.first ? first - archive.first : 0;
    for (; n < archive.count && records.size() < 1; ++n) {
      String line;
      RecordIndex entry;
      if (!readIndexed(archive, n, line, entry) || entry.seq != last+1) return false;
      JsonDocument record;
      if (deserializeJson(record, line) || !record["payload"]["device"].is<const char *>() ||
          strcmp(record["payload"]["device"], device.c_str()) != 0) {
        Serial.println("Upload paused: the SD record belongs to another device or is unreadable.");
        return false;
      }
      JsonObject row = records.add<JsonObject>();
      JsonObject sample = record["payload"];
      for (const char *key : uploadKeys) row[key] = sample[key];
      // The server uses device + seq to avoid duplicates and acknowledge storage.
      row["seq"] = entry.seq;
      if (batch.overflowed()) return false;
      if (measureJson(batch) > UPLOAD_MAX_BYTES) {
        records.remove(records.size() - 1);
        break;
      }
      last = entry.seq;
    }
    if (records.size() >= 1 || (n < archive.count)) break;
  }
  if (last < first) return false;
  if (batch.overflowed()) return false;
  size_t expectedBytes = measureJson(batch);
  body = "";
  return serializeJson(batch, body) == expectedBytes && body.length() == expectedBytes &&
         expectedBytes <= UPLOAD_MAX_BYTES;
}

bool acknowledgeUpload(const String &device, uint64_t first, uint64_t last,
                       int httpStatus, const String &response) {
  if (!storageReady || !currentUploadPrepared || !currentWakeSeq ||
      first != currentWakeSeq || last != first || last >= checkpoint.nextSeq ||
      !validUploadAck(device.c_str(), first, last, first - 1, httpStatus, response.c_str())) {
    Serial.println("Upload: server confirmation was missing or did not match. The reading stays on SD and will not be retried.");
    return false;
  }
  // Keep an exact receipt per successful reading. Never mark skipped records sent.
  String receiptPath = "/odrade/sent_" + String((unsigned long long)first) + ".json";
  FsFile receipt = logCard.open(receiptPath.c_str(), O_WRONLY | O_CREAT | O_TRUNC);
  bool receiptSaved = receipt && receipt.write(response.c_str(), response.length()) == response.length();
  if (receiptSaved) receiptSaved = receipt.sync();
  receipt.close();
  if (!receiptSaved) {
    Serial.println("Upload: server confirmed storage, but its receipt could not be saved on SD.");
    return false;
  }
  currentWakeSeq = 0;
  // Preserve the contiguous checkpoint for safe archive cleanup. A gap stays
  // protected even when later records have individual sent receipts.
  if (first != checkpoint.lastAckedSeq + 1) {
    Serial.printf("Upload confirmed: reading %llu saved by the server; receipt saved on SD.\n", first);
    return true;
  }
  Checkpoint next = checkpoint;
  next.lastAckedSeq = last;
  next.failures = 0;
  next.retryWake = 0;
  if (!commitState(next)) { storageReady = false; return false; }
  Serial.printf("Upload confirmed: the server saved readings %llu through %llu.\n", first, last);
  return true;
}

bool closeDataLogger() {
  // All writes are synced and files closed before reaching here.
  logCard.end();
  storageReady = false;
  cardStartupRetryPending = false;
  return true;
}
