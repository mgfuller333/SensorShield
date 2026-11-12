#!/usr/bin/env python3
"""Host regression for charger configuration and CE hold; no hardware I/O."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / 'Charger.cpp').read_text()

def function(name):
    start = source.index('bool ' + name + '() {')
    end = source.index('\n}', start) + 2
    return source[start:end]

pre = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <map>
#include <vector>
using gpio_num_t=int;
const int LOW=0,OUTPUT_OPEN_DRAIN=3,ESP_OK=0;
std::vector<std::string> calls;
bool held=false,deepHold=false,failHold=false;
int pinModeValue=0,pinLevel=1;
void pinMode(int pin,int mode){assert(pin==10);pinModeValue=mode;calls.push_back("mode");}
void digitalWrite(int pin,int level){assert(pin==10 && level==LOW);pinLevel=level;calls.push_back("low");}
int gpio_hold_dis(int pin){assert(pin==10 && pinModeValue==OUTPUT_OPEN_DRAIN && pinLevel==LOW);held=false;calls.push_back("release");return 0;}
int gpio_hold_en(int pin){assert(pin==10);held=!failHold;calls.push_back("hold");return failHold?-1:0;}
void gpio_deep_sleep_hold_en(){deepHold=true;calls.push_back("deepHold");}
struct { void println(const char*){} } Serial;
std::map<std::string,double> values;
std::string failMethod;
bool incorrectVoltage=false;
struct MockCharger {
 bool set(const char* name,double value){calls.push_back(name);if(failMethod==name)return false;values[name]=value;return true;}
'''
methods = sorted(set(re.findall(r'charger\.(set\w+)\(', function('configureCharger'))))
for method in methods:
    pre += f' bool {method}(double v){{return set("{method}",v);}}\n'
for getter, setter in [('getChargeEnable','setChargeEnable'),('getCellCount','setCellCount'),('getChargeLimitV','setChargeLimitV'),('getChargeLimitA','setChargeLimitA'),('getInputLimitA','setInputLimitA'),('getMinSystemV','setMinSystemV'),('getWDT','setWDT')]:
    suffix = '+(incorrectVoltage?1:0)' if getter=='getChargeLimitV' else ''
    pre += f' float {getter}(){{return values["{setter}"]{suffix};}}\n'
pre += '''} charger;
bool chargerReady=true,failRead=false,failWrite=false;
uint8_t adcValue=0x40;
bool readChargerRegisters(uint8_t,uint8_t* data,uint8_t){*data=adcValue;return !failRead;}
bool writeChargerRegister(uint8_t,uint8_t data){if(failWrite)return false;adcValue=data;return true;}
'''
# Symbol values are irrelevant to the mock, but keep distinct enumerators.
for i, name in enumerate(sorted(set(re.findall(r'\bBQ25798_\w+', source)))):
    pre += f'const int {name}={i};\n'
pre += re.search(r'const uint8_t BQ_CE_PIN[^;]+;', (ROOT/'board_pins.h').read_text())[0]+'\n'
pre += '\n'.join(re.findall(r'const float \w+ = [^;]+;', source))+'\n'
post = r'''
int main(){
 assert(holdChargerEnableLow());assert(held && deepHold && pinLevel==LOW);
 assert((calls==std::vector<std::string>{"mode","low","release","hold","deepHold"}));
 // Wake: output registers reset while the pad is still held. Configure before release.
 pinModeValue=0;pinLevel=1;assert(holdChargerEnableLow());assert(held && pinLevel==LOW);
 assert(configureCharger());
 assert(values["setChargeLimitV"]==double(7.90f));
 assert(values["setChargeLimitA"]==2 && values["setInputLimitA"]==2);
 assert(values["setMinSystemV"]==5 && values["setCellCount"]==BQ25798_CELL_COUNT_2S);
 assert(values["setChargeEnable"]==1 && values["setWDT"]==BQ25798_WDT_DISABLE);
 assert(values["setMPPTenable"]==0 && values["setOTGenable"]==0);
 assert(values["setFastChargeTimerEnable"]==1 && (adcValue & 0xC0)==0x80);
 auto beforeSleep=values;auto adcBeforeSleep=adcValue;
 assert(stopChargerTelemetry());assert(values==beforeSleep && held && deepHold && pinLevel==LOW);
 assert(adcValue==(adcBeforeSleep & ~0x80));
 failRead=true;assert(!stopChargerTelemetry());failRead=false;
 failHold=true;assert(!stopChargerTelemetry());failHold=false;
 // Configuration writes/readback failures must not enable charging.
 failMethod="setChargeLimitV";assert(!configureCharger());assert(values["setChargeEnable"]==0);
 failMethod="";incorrectVoltage=true;assert(!configureCharger());assert(values["setChargeEnable"]==0);
 incorrectVoltage=false;failRead=true;assert(!configureCharger());assert(values["setChargeEnable"]==0);
 std::cout<<"PASS: working-example settings, configuration failures, CE LOW hold/release order, wake restoration, and ADC-only sleep shutdown.\n";
}
'''
# Also check lifecycle placement: /CE precedes Serial, charger setup precedes GPS.
sketch=(ROOT/'SensorShield.ino').read_text()
setup=sketch[sketch.index('void setup()'):sketch.index('void loop()')]
assert setup.index('holdChargerEnableLow()') < setup.index('Serial.begin(')
assert setup.index('setupCharger()') < setup.index('refreshLocationIfDue()')
assert setup.count('setupCharger()')==1
with tempfile.TemporaryDirectory(prefix='charger-test-') as temp:
    cpp=Path(temp)/'test.cpp';binary=Path(temp)/'test'
    cpp.write_text(pre+'\n'+function('holdChargerEnableLow')+'\n'+function('configureCharger')+'\n'+function('stopChargerTelemetry')+post)
    subprocess.run(['c++','-std=c++17',str(cpp),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
