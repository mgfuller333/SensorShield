#!/usr/bin/env python3
"""Run production GPS/JSON functions on the host; mock clock, NVS and SD I/O.
Requires c++ and the installed ArduinoJson library (or ARDUINOJSON_INCLUDE).
No board, modem, system clock changes, or network access required.
"""
import os
import re
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

def function(file, name):
    text = (ROOT / file).read_text()
    start = text.index(name)
    opening = text.index('{', start)
    # These functions contain balanced braces in their string literals too.
    depth = 1
    end = opening + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]

clock = '\n'.join(line for line in (ROOT / 'GpsClock.cpp').read_text().splitlines()
                  if not line.startswith('#include'))
modem = (ROOT / 'Modem.cpp').read_text()
start = modem.index('      if (fix.status == WALTER_MODEM_GNSS_FIX_STATUS_READY &&', modem.index('if (available)'))
end = modem.index('      Serial.println("Location: this GPS attempt', start)
accept = 'bool receiveFix(const WMGNSSFixEvent &fix, int64_t receivedUs) {\n' + modem[start:end] + '\nreturn false;}'
parse = function('LogStorage.cpp', 'bool parseRecord(').replace('line.lastIndexOf(', 'lastIndexOf(line, ')
record = function('LogStorage.cpp', 'bool buildRecord(')
for expr in ['(unsigned long long)seq', '(long long)day', 'crc']:
    record = record.replace('String(' + expr + ')', 'std::to_string(' + expr + ')')
pre = r'''
#include <ArduinoJson.h>
#include <string>
#include <vector>
#include <cassert>
#include <iostream>
#include <cmath>
#include <ctime>
#include <sys/time.h>
#include <cstddef>
#include <cstdio>
#include <inttypes.h>
#include "QueueRules.h"
using String = std::string;
#define RTC_DATA_ATTR
struct SerialMock {
 void println(const char* = "") {} void print(const char*) {}
 template<class... T> void printf(const char*, T...) {}
 size_t write(uint8_t) {return 1;}
 size_t write(const uint8_t*, size_t n) {return n;}
} Serial;
struct Preferences {
 bool begin(const char*,bool) {return true;} void end() {}
 size_t getBytesLength(const char*) {return 0;}
 size_t getBytes(const char*,void*,size_t) {return 0;}
 size_t putBytes(const char*,const void*,size_t n) {return n;}
};
int64_t timerUs=1000000; time_t epochNow=0;
int64_t esp_timer_get_time(){return timerUs;}
time_t fakeTime(time_t*){return epochNow;}
int fakeSettimeofday(const timeval* tv, const void*){epochNow=tv->tv_sec;return 0;}
#define time fakeTime
#define settimeofday fakeSettimeofday
const unsigned GPS_REFRESH_SECONDS=0;
bool formatGpsTimestamp(char*,size_t);
struct WMGNSSFixEvent {
 int status=0; int64_t timestamp=1794084884;
 double latitude=33.685504,longitude=-126.888, height=1854.210;
 double estimatedConfidence=20000000;
 unsigned satCount=12;
 struct {unsigned signalStrength=0;} sats[32];
};
const int WALTER_MODEM_GNSS_FIX_STATUS_READY=0;
void printGnssFix(const WMGNSSFixEvent&){}
bool storageReady=true;
struct {uint64_t lastAckedSeq=0;} checkpoint;
struct Segment {uint64_t count,first,last;};
struct RecordIndex {uint64_t seq;};
std::vector<Segment> segments;
std::vector<String> lines;
const size_t UPLOAD_MAX_RECORDS=8,UPLOAD_MAX_BYTES=48000,MAX_RECORD_BYTES=16000;
uint64_t pendingRecords(){return lines.size()-checkpoint.lastAckedSeq;}
bool readIndexed(Segment&,uint64_t n,String &line,RecordIndex &entry){line=lines.at(n);entry.seq=n+1;return true;}
int lastIndexOf(const String &s,const char* text){auto p=s.rfind(text);return p==String::npos?-1:int(p);}
'''
post = r'''
int main(){
 setupGpsClock();
 JsonDocument missing; auto before=missing.to<JsonObject>(); before["device"]="ABC";
 addLocationMetadata(before); assert(before["ts"].isNull() && before["loc"].isNull());
 // Reproduce an existing backlog of eight records with no fix.
 for(unsigned seq=1;seq<=8;++seq){String line;assert(buildRecord(before,seq,0,line));lines.push_back(line);}
 WMGNSSFixEvent fix; assert(receiveFix(fix,timerUs));
 // Sensor acquisition creates its document AFTER the GPS fix, as in setup().
 JsonDocument sample; auto row=sample.to<JsonObject>(); row["device"]="ABC"; row["temp_0"]=24.5;
 epochNow+=4; timerUs+=4000000; addLocationMetadata(row);
 assert(row["ts"]=="2026-11-07T20:54:48Z");
 assert(row["loc"]=="POINT(-126.888000 33.685504)");
 assert(std::abs(row["alt"].as<double>()-1854.210)<0.000001); assert(row["satCnt"]==12);
 String line; assert(buildRecord(row,9,epochNow/86400,line)); lines.push_back(line);
 segments={{9,1,9}}; auto original=lines;
 String body; uint64_t first,last; assert(prepareUpload("ABC",body,first,last));
 JsonDocument uploaded; assert(!deserializeJson(uploaded,body));
 assert(first==1 && last==8 && uploaded.size()==8);
 assert(uploaded[0]["ts"].isNull() && uploaded[7]["loc"].isNull());
 // Simulate server acknowledging the first eight: now the fresh sample uploads.
 checkpoint.lastAckedSeq=8; assert(prepareUpload("ABC",body,first,last));
 assert(!deserializeJson(uploaded,body)); assert(first==9 && last==9);
 for(const char* key:{"ts","loc","satCnt"}) assert(uploaded[0][key]==row[key]);
 assert(std::abs(uploaded[0]["alt"].as<double>()-row["alt"].as<double>())<0.001);
 assert(lines==original); std::cout<<"Fresh upload: "<<body<<"\n";
 // A later wake with no fix must not pretend to have a new mobile location.
 setupGpsClock(); JsonDocument next; addLocationMetadata(next.to<JsonObject>());
 assert(next["loc"].isNull()); assert(!next["ts"].isNull());
 std::cout<<"PASS: fix acceptance (including 20,000,000 m uncertainty), UTC/WKT conversion, sensor document creation, SD record/checksum, upload projection, old backlog, and next-wake missing fix.\n";
}
'''
# Follow the real queue limits and GPS refresh policy instead of duplicating them.
config = (ROOT / 'LoggerConfig.h').read_text()
for name in ['GPS_REFRESH_SECONDS', 'UPLOAD_MAX_RECORDS', 'UPLOAD_MAX_BYTES']:
    value = re.search(r'\b' + name + r'\s*=\s*(\d+)', config).group(1)
    pre = re.sub(r'\b' + name + r'=\d+', name + '=' + value, pre)

include = os.environ.get('ARDUINOJSON_INCLUDE', str(Path.home() / 'Documents/Arduino/libraries/ArduinoJson/src'))
with tempfile.TemporaryDirectory(prefix='gps-payload-') as tmp:
    source=Path(tmp)/'test.cpp'; binary=Path(tmp)/'test'
    source.write_text(pre+clock+accept+parse+record+function('DataLogger.cpp','bool prepareUpload(')+post)
    subprocess.run(['c++','-std=c++17','-I'+include,'-I'+str(ROOT),str(source),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
