#pragma once
// Internal bookkeeping for DataLogger.cpp. Start with that file when learning
// the logger; these helpers handle interrupted writes and fast queue recovery.
#include <Arduino.h>
#include <ArduinoJson.h>
#include <SdFat.h>
#include <vector>

extern SdFs logCard;
extern bool storageReady;
extern const char *ACTIVE_LOG;
extern const char *ACTIVE_INDEX;

// The checkpoint remembers which readings the server has accepted.
struct Checkpoint {
  uint64_t generation = 0, lastAckedSeq = 0, nextSeq = 1;
  // Legacy retry fields remain only for reading existing SD checkpoints.
  // The MVP never uses them to postpone an upload.
  uint64_t wakeCount = 0, retryWake = 0;
  uint32_t failures = 0;
};
extern Checkpoint checkpoint;
// A fixed-size bookmark for a JSON line. packed keeps its on-card size stable.
struct __attribute__((packed)) RecordIndex {
  uint64_t seq, offset;
  uint32_t length, lineCrc, entryCrc;
};
// One active or archived log and its index.
struct Segment {
  String log, index;
  uint64_t first = 0, last = 0, end = 0, count = 0;
  int64_t day = 0;
};
extern std::vector<Segment> segments;

// Each index entry points to one JSON line, avoiding a full SD scan at boot.
// Keep these structures and the saved format compatible with existing cards.
bool recoverStorage();
bool commitState(Checkpoint next);
bool makeStorageRoom();
Segment *findActiveLog();
bool prepareDailyLog(int64_t day);
bool buildRecord(JsonObject sample, uint64_t seq, int64_t day, String &line);
bool addIndex(FsFile &index, uint64_t seq, uint64_t offset, const String &line);
bool readIndexed(Segment &segment, uint64_t number, String &line, RecordIndex &entry);
