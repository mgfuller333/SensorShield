#pragma once
#include <ArduinoJson.h>
#include <stdint.h>
#include <stddef.h>

inline uint32_t dataCrc(const void *data, size_t length) {
  uint32_t crc = 0xffffffffU;
  const uint8_t *p = static_cast<const uint8_t *>(data);
  while (length--) {
    crc ^= *p++;
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
  }
  return ~crc;
}

// Reject truncated JSON and any non-whitespace bytes after the JSON object.
inline bool singleJsonObject(const char *text) {
  if (!text) return false;
  while (*text == ' ' || *text == '\r' || *text == '\n' || *text == '\t') ++text;
  if (*text != '{') return false;
  int depth = 0; bool quoted = false, escaped = false;
  for (; *text; ++text) {
    char c = *text;
    if (quoted) {
      if (escaped) escaped = false;
      else if (c == '\\') escaped = true;
      else if (c == '"') quoted = false;
    } else if (c == '"') quoted = true;
    else if (c == '{' || c == '[') ++depth;
    else if (c == '}' || c == ']') {
      if (--depth == 0) {
        ++text;
        while (*text == ' ' || *text == '\r' || *text == '\n' || *text == '\t') ++text;
        return *text == 0;
      }
      if (depth < 0) return false;
    }
  }
  return false;
}

// Only an exact, device-scoped, durable acknowledgement advances the queue.
inline bool validUploadAck(const char *device, uint64_t first, uint64_t last,
                           uint64_t lastAcked, int status, const char *body) {
  if (status < 200 || status >= 300 || first != lastAcked + 1 || last < first) return false;
  if (!singleJsonObject(body)) return false;
  JsonDocument ack;
  if (deserializeJson(ack, body)) return false;
  return ack["version"].is<unsigned>() && ack["version"].as<unsigned>() == 1 &&
         ack["device"].is<const char *>() && strcmp(ack["device"], device) == 0 &&
         ack["durable"].is<bool>() && ack["durable"].as<bool>() &&
         ack["firstSeq"].is<uint64_t>() && ack["firstSeq"].as<uint64_t>() == first &&
         ack["lastSeq"].is<uint64_t>() && ack["lastSeq"].as<uint64_t>() == last;
}
