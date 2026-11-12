#!/usr/bin/env python3
"""Run the production OLED page renderer against an in-memory text display."""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
source=(root/'Display.cpp').read_text()
renderer=source[source.index('const char *screenState'):source.index('bool setupDisplay()')]
header=(root/'Display.h').read_text().replace('#pragma once','').replace('#include <Arduino.h>','')
mock=r'''
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <cassert>
#include <iostream>
#define F(x) x
const int SSD1306_WHITE=1;
uint32_t clockMs=0;
uint32_t millis(){return clockMs;}
struct Screen {
 std::vector<std::string> rows; std::string line;
 void clearDisplay(){rows.clear();line.clear();}
 void setTextSize(int){} void setTextColor(int){} void setCursor(int,int){}
 void print(const char *s){line+=s;}
 void println(const char *s=""){line+=s;rows.push_back(line);line.clear();}
 void display(){assert(rows.size()<=8);for(auto &r:rows)assert(r.size()<=21);}
 bool contains(const char *s){for(auto &r:rows)if(r.find(s)!=std::string::npos)return true;return false;}
} display;
bool displayReady=true;
'''
test=r'''
int main(){
 refreshDisplay();assert(display.contains("Init:working"));
 assert(display.contains("GPS:not started"));assert(display.contains("Payload:not sent"));
 setDisplayStatus(GPS_STATUS,"fix done");setDisplayStatus(MODEM_STATUS,"connected");
 setDisplayStatus(SENSOR_STATUS,"gathering");showSensorPage();
 for(int i=0;i<11;i++)setDisplaySensor(i,"done");
 finishDisplaySensors();assert(display.contains("Sensors:done"));
 setDisplaySensor(9,"check");finishDisplaySensors();
 assert(display.contains("Sensors:done (check)"));
 setDisplaySensor(8,"failed");finishDisplaySensors();
 assert(display.contains("Sensors:partial"));refreshDisplay();
 assert(display.contains("PM:failed") && display.contains("PW:check"));
 assert(!display.contains("A1:done"));assert(display.contains("GPS:fix done"));
 assert(display.contains("Modem:connected"));
 for(int i=0;i<8;i++)setDisplaySensor(i,"failed");
 for(clockMs=0;clockMs<10000;clockMs+=250)refreshDisplay();
 setDisplayStatus(PAYLOAD_STATUS,"sent/no proof");assert(display.contains("Payload:sent/no proof"));
 setDisplayStatus(PAYLOAD_STATUS,"sent + saved");assert(display.contains("Payload:sent + saved"));
 for(int i=0;i<11;i++)setDisplaySensor(i,"failed");
 finishDisplaySensors();assert(display.contains("Sensors:failed"));
 displayReady=false;setDisplayStatus(MODEM_STATUS,"failed");
 std::cout<<"PASS: 21-column/8-line pages, GPS/modem persistence, failure priority, rotating failures and distinct upload results.\n";
}
'''
with tempfile.TemporaryDirectory(prefix='oled-status-') as tmp:
    cpp=Path(tmp)/'test.cpp';binary=Path(tmp)/'test'
    cpp.write_text(mock+header+renderer+test)
    subprocess.run(['c++','-std=c++17',str(cpp),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
