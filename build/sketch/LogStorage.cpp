#line 1 "/Users/morganfuller/Documents/Sireen/Sharingan/CoralGablesOdradePCBRev1/SensorShield/LogStorage.cpp"
#include "LogStorage.h"
#include "LoggerConfig.h"
#include "QueueRules.h"
#include <algorithm>

// SD access happens before sensor tasks start or after they have stopped.
SdFs logCard;
bool storageReady = false;
const char *STATE = "/odrade/upload_state.json";
const char *BACKUP = "/odrade/upload_state.bak";
const char *ACTIVE_LOG = "/odrade/samples.jsonl";
const char *ACTIVE_INDEX = "/odrade/samples.idx";
const size_t MAX_RECORD_BYTES = 16000;

Checkpoint checkpoint;
std::vector<Segment> segments;

bool writeAll(FsFile &file, const String &text) {
  return file.write(text.c_str(), text.length()) == text.length() && file.sync();
}

String stateText(const Checkpoint &s) {
  JsonDocument d;
  d["version"] = 1;
  d["generation"] = s.generation;
  d["lastAckedSeq"] = s.lastAckedSeq;
  d["nextSeq"] = s.nextSeq;
  d["wakeCount"] = s.wakeCount;
  d["retryWake"] = s.retryWake;
  d["failures"] = s.failures;
  String text;
  serializeJson(d, text);
  return text;
}

bool readState(const char *path, Checkpoint &s) {
  FsFile f = logCard.open(path, O_RDONLY);
  if (!f || f.fileSize() > 2048) return false;
  JsonDocument d;
  auto error = deserializeJson(d, f);
  f.close();
  if (error || d["version"] != 1 || !d["nextSeq"].is<uint64_t>() ||
      !d["lastAckedSeq"].is<uint64_t>() || !d["generation"].is<uint64_t>() ||
      !d["wakeCount"].is<uint64_t>() || !d["retryWake"].is<uint64_t>() ||
      !d["failures"].is<uint32_t>() || !d["crc32"].is<uint32_t>()) return false;
  s = {d["generation"], d["lastAckedSeq"], d["nextSeq"],
       d["wakeCount"], d["retryWake"], d["failures"]};
  String text = stateText(s);
  return s.nextSeq > s.lastAckedSeq &&
         d["crc32"].as<uint32_t>() == dataCrc(text.c_str(), text.length());
}

// Alternate two checkpoint slots. Never truncate the newest valid generation.
// A torn write leaves the other slot available; boot selects the newest CRC-valid slot.
bool commitState(Checkpoint next) {
  next.generation = checkpoint.generation + 1;
  Checkpoint primary;
  bool primaryCurrent = readState(STATE, primary) && primary.generation == checkpoint.generation;
  const char *target = primaryCurrent ? BACKUP : STATE;
  String text = stateText(next);
  uint32_t crc = dataCrc(text.c_str(), text.length());
  text.remove(text.length() - 1);
  text += ",\"crc32\":" + String(crc) + "}";
  FsFile f = logCard.open(target, O_WRONLY | O_CREAT | O_TRUNC);
  if (!f || !writeAll(f, text)) return false;
  f.close();
  Checkpoint verify;
  if (!readState(target, verify) || verify.generation != next.generation) return false;
  checkpoint = next;
  return true;
}

bool parseRecord(const String &line, uint64_t &seq, int64_t &day, JsonDocument &doc) {
  int crcAt = line.lastIndexOf(",\"crc32\":");
  if (crcAt < 0 || deserializeJson(doc, line) || !doc["seq"].is<uint64_t>() ||
      !doc["day"].is<int64_t>() || !doc["payload"].is<JsonObject>() ||
      !doc["crc32"].is<uint32_t>()) return false;
  seq = doc["seq"];
  day = doc["day"];
  return seq > 0 && doc["crc32"].as<uint32_t>() == dataCrc(line.c_str(), crcAt);
}

bool readLine(FsFile &f, String &line) {
  line = "";
  while (f.available()) {
    int c = f.read();
    if (c == '\n') return true;
    if (c < 0 || line.length() >= MAX_RECORD_BYTES) return false;
    line += char(c);
  }
  return false;
}

bool readIndexed(Segment &s, uint64_t number, String &line, RecordIndex &entry) {
  FsFile index = logCard.open(s.index.c_str(), O_RDONLY);
  if (!index || !index.seekSet(number * sizeof(entry)) ||
      index.read(&entry, sizeof(entry)) != sizeof(entry)) return false;
  index.close();
  if (entry.entryCrc != dataCrc(&entry, offsetof(RecordIndex, entryCrc)) ||
      !entry.length || entry.length > MAX_RECORD_BYTES + 1) return false;
  FsFile f = logCard.open(s.log.c_str(), O_RDONLY);
  if (!f || !f.seekSet(entry.offset) || !readLine(f, line)) return false;
  bool valid = f.curPosition() == entry.offset + entry.length &&
      dataCrc(line.c_str(), line.length()) == entry.lineCrc;
  f.close();
  JsonDocument doc;
  uint64_t seq; int64_t day;
  return valid && parseRecord(line, seq, day, doc) && seq == entry.seq;
}

bool addIndex(FsFile &index, uint64_t seq, uint64_t offset, const String &line) {
  RecordIndex entry = {seq, offset, uint32_t(line.length() + 1),
                      dataCrc(line.c_str(), line.length()), 0};
  entry.entryCrc = dataCrc(&entry, offsetof(RecordIndex, entryCrc));
  return index.write(&entry, sizeof(entry)) == sizeof(entry) && index.sync();
}

// Normal boot checks the first/last indexed records, then scans only an
// unindexed tail. A missing/damaged index is rebuilt from its own segment.
bool recoverSegment(Segment &s) {
  FsFile f = logCard.open(s.log.c_str(), O_RDWR);
  if (!f) return false;
  FsFile idx = logCard.open(s.index.c_str(), O_RDWR | O_CREAT);
  if (!idx) return false;
  uint64_t count = idx.fileSize() / sizeof(RecordIndex);
  bool indexed = idx.fileSize() % sizeof(RecordIndex) == 0;
  idx.close();
  String line; RecordIndex first, last;
  if (count && indexed && readIndexed(s, 0, line, first) &&
      readIndexed(s, count - 1, line, last) && last.seq - first.seq + 1 == count) {
    s.first = first.seq; s.last = last.seq; s.count = count;
    s.end = last.offset + last.length;
    JsonDocument d; uint64_t seq;
    if (!parseRecord(line, seq, s.day, d)) return false;
  } else {
    s.first = s.last = s.count = s.end = 0;
  }
  if (s.count && f.fileSize() == s.end) {
    f.close();
    return true; // Normal boot: no index rewrite or archive scan is needed.
  }
  idx = logCard.open(s.index.c_str(), O_RDWR | O_CREAT);
  if (!idx || !idx.truncate(s.count * sizeof(RecordIndex)) || !idx.seekEnd()) return false;
  if (!f.seekSet(s.end)) return false;
  while (f.curPosition() < f.fileSize()) {
    uint64_t offset = f.curPosition();
    bool completeLine = readLine(f, line);
    JsonDocument doc; uint64_t seq; int64_t day;
    if (!completeLine) {
      // Only an unfinished final line can be discarded. It was never saved.
      if (f.curPosition() != f.fileSize() || !f.truncate(offset) || !f.sync()) return false;
      break;
    }
    // A complete but corrupt record is not silently thrown away.
    if (!parseRecord(line, seq, day, doc) || (s.last && seq != s.last + 1)) return false;
    if (!addIndex(idx, seq, offset, line)) return false;
    if (!s.first) s.first = seq;
    s.last = seq; s.day = day; s.count++; s.end = f.curPosition();
  }
  bool ok = idx.sync();
  idx.close(); f.close();
  return ok;
}

// SdFat OpenNext example pattern: open directory, openNext, close each file.
bool recoverStorage() {
  Checkpoint best; bool found = false;
  for (const char *path : {STATE, BACKUP}) {
    Checkpoint candidate;
    if (readState(path, candidate) && (!found || candidate.generation > best.generation)) {
      best = candidate; found = true;
    }
  }
  if (found) checkpoint = best;
  segments.clear();
  FsFile dir = logCard.open("/odrade", O_RDONLY), entry;
  if (!dir) return false;
  while (entry.openNext(&dir, O_RDONLY)) {
    char name[96]; entry.getName(name, sizeof(name));
    String n(name);
    bool wanted = n == "samples.jsonl" || (n.startsWith("day_") && n.endsWith(".jsonl"));
    entry.close();
    if (!wanted) continue;
    Segment s; s.log = "/odrade/" + n; s.index = s.log.substring(0, s.log.length()-6) + ".idx";
    if (!recoverSegment(s)) { dir.close(); Serial.println("Storage: damaged record detected; queue preserved for recovery."); return false; }
    segments.push_back(s);
  }
  dir.close();
  std::sort(segments.begin(), segments.end(), [](const Segment &a, const Segment &b) {
    return (a.first ? a.first : UINT64_MAX) < (b.first ? b.first : UINT64_MAX);
  });
  uint64_t maximum = 0, expected = 0;
  for (auto &s : segments) {
    if (!s.count) continue;
    if (expected && s.first != expected) return false;
    expected = s.last + 1; maximum = s.last;
  }
  if (!found && (maximum || logCard.exists(STATE) || logCard.exists(BACKUP))) {
    Serial.println("Storage: upload checkpoint cannot be recovered. Saved readings have been left untouched.");
    return false;
  }
  if (found && (checkpoint.lastAckedSeq > maximum || checkpoint.nextSeq > maximum + 1)) return false;
  // Deleted archive prefixes must already be acknowledged.
  for (auto &s : segments) if (s.count) {
    if (s.first > checkpoint.lastAckedSeq + 1) return false;
    break;
  }
  checkpoint.nextSeq = maximum + 1;
  return true;
}

uint64_t freeStorageBytes() {
  uint32_t clusters = logCard.freeClusterCount();
  if (clusters == UINT32_MAX) return 0;
  return uint64_t(clusters) * logCard.bytesPerCluster();
}

bool makeStorageRoom() {
  if (freeStorageBytes() >= SD_MIN_FREE_BYTES) return true;
  for (auto it = segments.begin(); it != segments.end();) {
    if (it->log != ACTIVE_LOG && it->count && it->last <= checkpoint.lastAckedSeq) {
      if (!logCard.remove(it->log.c_str())) return false;
      logCard.remove(it->index.c_str()); // An orphan index never contains sample data.
      Serial.println("Storage: removed the oldest fully uploaded archive to free space.");
      it = segments.erase(it);
      if (freeStorageBytes() >= SD_MIN_FREE_BYTES) return true;
    } else ++it;
  }
  Serial.println("Storage: full. Unsent readings are protected; no new reading can be saved.");
  return false;
}

Segment *findActiveLog() {
  for (Segment &segment : segments) {
    if (segment.log == ACTIVE_LOG) return &segment;
  }
  return nullptr;
}

bool prepareDailyLog(int64_t day) {
  Segment *active = findActiveLog();
  // Rotate by UTC day; without time, cap each undated archive at 288 readings.
  if (active && active->count && (active->day != day || (!day && active->count >= 288))) {
    String base = "/odrade/day_" + String((unsigned long long)active->first);
    if (!logCard.rename(active->log.c_str(), (base + ".jsonl").c_str())) return false;
    logCard.rename(active->index.c_str(), (base + ".idx").c_str());
    active->log = base + ".jsonl"; active->index = base + ".idx";
  }
  if (!makeStorageRoom()) return false;
  // Cleanup may invalidate vector pointers.
  active = findActiveLog();
  if (!active) {
    Segment s; s.log = ACTIVE_LOG; s.index = ACTIVE_INDEX;
    // An index left over from interrupted rotation must not belong to this log.
    if (!logCard.exists(ACTIVE_LOG)) logCard.remove(ACTIVE_INDEX);
    segments.push_back(s);
  }
  return true;
}

bool buildRecord(JsonObject sample, uint64_t seq, int64_t day, String &line) {
  if (sample.isNull() || !sample["device"].is<const char *>()) return false;
  String payload;
  size_t expectedPayloadBytes = measureJson(sample);
  if (serializeJson(sample, payload) != expectedPayloadBytes || payload.length() != expectedPayloadBytes) return false;
  line = "{\"seq\":" + String((unsigned long long)seq) + ",\"day\":" + String((long long)day) + ",\"payload\":" + payload;
  uint32_t crc = dataCrc(line.c_str(), line.length());
  line += ",\"crc32\":" + String(crc) + "}";
  if (line.length() > MAX_RECORD_BYTES) return false;
  { // Refuse an incomplete record if an allocation failed while building it.
    JsonDocument verify; uint64_t verifiedSeq; int64_t verifiedDay;
    if (!parseRecord(line, verifiedSeq, verifiedDay, verify) || verifiedSeq != seq || verifiedDay != day) return false;
  }
  return true;
}
