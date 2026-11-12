#!/usr/bin/env python3
"""Exercise production power-enable code with GPIO hold behavior modeled on host."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
sketch = (root / 'SensorShield.ino').read_text()
start = sketch.index('bool enableSensorPower()')
end = sketch.index('\nvoid setup()', start)
helper = sketch[start:end]
setup = sketch[sketch.index('void setup()'):sketch.index('void loop()')]
assert setup.index('beginDataLogger()') < setup.index('enableSensorPower()')
assert setup.index('enableSensorPower()') < setup.index('Wire.begin(')
source = r'''
#include <cassert>
#include <iostream>
using gpio_num_t = int;
const int WALTER_3V3_EN=0, OUTPUT=1, LOW=0, HIGH=1, ESP_OK=0;
bool held=false, output=false, chargerHeld=true;
int latch=HIGH, pad=HIGH;
void pinMode(int pin,int mode){assert(pin==0 && mode==OUTPUT);output=true;}
void digitalWrite(int pin,int level){assert(pin==0);latch=level;if(!held)pad=level;}
int gpio_hold_dis(int pin){assert(pin==0);assert(output && latch==LOW);held=false;pad=latch;return ESP_OK;}
'''+helper+r'''
int main(){
 assert(enableSensorPower()); assert(output && pad==LOW && !held);
 for(int wake=0;wake<3;wake++){
   // Firmware shuts the supply off and holds it through deep sleep.
   digitalWrite(0,HIGH);held=true;
   // Wake resets the output configuration, but the pad retains HIGH.
   output=false;latch=LOW;assert(pad==HIGH);
   assert(enableSensorPower());assert(output && pad==LOW && !held);
   assert(chargerHeld);
 }
 std::cout<<"PASS: cold boot and three simulated wakes enable GPIO0 after SD initialization.\n";
}
'''
with tempfile.TemporaryDirectory(prefix='sensor-power-') as tmp:
    cpp=Path(tmp)/'test.cpp';binary=Path(tmp)/'test'
    cpp.write_text(source)
    subprocess.run(['c++','-std=c++17',str(cpp),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
