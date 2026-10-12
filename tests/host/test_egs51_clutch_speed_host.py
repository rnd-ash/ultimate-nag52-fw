"""Forward clutch-speed stages, reciprocal shifts and production ratio adapter."""
from pathlib import Path
import unittest
from owner_port_test import compile_run
from owner_port_test import method

ROOT=Path(__file__).resolve().parents[2]


class ClutchSpeedTests(unittest.TestCase):
    def test_stages_direction_boundaries_and_custom_ratios(self):
        compile_run(r'''
#include "models/egs51_clutch_speed.h"
#include <cassert>
#include <limits>
using namespace Egs51ClutchSpeed;
int main() {
 auto calc=[](unsigned shift,unsigned n2,unsigned n3,unsigned turbine,unsigned output) {
  return calculate(shift,n2,n3,turbine,output,2408,1486,1000);
 };
 auto s=calc(2,0,149,149,100);
 assert(s.applying==1); // Previous combined float expression truncated 0.4 RPM to zero.
 s=calc(2,1001,999,1000,1000);
 assert(s.applying==-487 && s.releasing==2269 && s.rear_sun==0);
 s=calc(3,1001,999,1000,1000);
 assert(s.applying==-3 && s.releasing==1002 && s.rear_sun==1002);
 for(unsigned up=1;up<=4;++up) {
  auto a=calc(up,1001,999,1000,1000),b=calc(up+4,1001,999,1000,1000);
  assert(a.applying==b.releasing && a.releasing==b.applying);
  assert(a.rear_sun==b.rear_sun);
 }
 s=calculate(2,1000,1000,1000,1000,2200,1400,1000);
 assert(s.applying==-400 && s.releasing==2100); // Configured mechanical ratios.
 s=calculate(3,1000,1000,1000,1000,2200,1400,1000);
 assert(s.applying==0 && s.releasing==1000);
 s=calc(3,0,0,0,60000);
 assert(s.applying==INT16_MIN && s.releasing==INT16_MAX && s.rear_sun==INT16_MAX);
 assert(calc(1,UINT16_MAX,0,0,0).applying==INT16_MAX);
 assert(calc(2,0,UINT16_MAX,0,0).applying==INT16_MAX);
 assert(calc(3,0,0,0,UINT16_MAX).applying==INT16_MAX);
 assert(calc(4,0,0,UINT16_MAX,0).rear_sun==INT16_MAX);
 assert(calculate(2,0,0,0,0,1400,1400,1000).applying==INT16_MAX);
 assert(calculate(3,0,0,0,0,2200,1000,1000).applying==INT16_MAX);
 assert(calc(0,0,0,0,0).applying==0);
 assert(ratio_word(1.486f)==1486 && ratio_word(65.535f)==65535);
 assert(ratio_word(0)==0 && ratio_word(-1)==0);
 assert(ratio_word(std::numeric_limits<float>::infinity())==0);
 assert(ratio_word(std::numeric_limits<float>::quiet_NaN())==0);
 // All mechanical ratio words survive the existing float configuration adapter.
 for(unsigned r=1;r<=65535;++r) assert(ratio_word(float(r)/1000)==r);
}
''')

    def test_actual_model_wiring(self):
        compile_run(r'''
#include "models/egs51_clutch_speed.h"
#include <cassert>
struct ShiftClutchData { int16_t on_clutch_speed,off_clutch_speed,rear_sun_speed; };
struct SpeedSensors { uint16_t n2,n3,turbine,output; };
struct GearRatioInfo { float ratio; };
enum class GearChange { _2_3=2,_3_2=6 };
constexpr int RAT_2_IDX=1,RAT_3_IDX=2,RAT_4_IDX=3;
namespace ClutchSpeedModel {
 ShiftClutchData get_shifting_clutch_speeds(SpeedSensors,GearChange,const GearRatioInfo*);
}
'''+method('src/models/clutch_speed.cpp',
          'ShiftClutchData ClutchSpeedModel::get_shifting_clutch_speeds(')+r'''
int main() {
 GearRatioInfo ratios[4]={{3.932f},{2.408f},{1.486f},{1.0f}};
 auto s=ClutchSpeedModel::get_shifting_clutch_speeds({1001,999,1000,1000},GearChange::_2_3,ratios);
 assert(s.on_clutch_speed==-487 && s.off_clutch_speed==2269);
 ratios[1].ratio=2.2f; ratios[2].ratio=1.4f;
 s=ClutchSpeedModel::get_shifting_clutch_speeds({1000,1000,1000,1000},GearChange::_3_2,ratios);
 assert(s.on_clutch_speed==2100 && s.off_clutch_speed==-400);
}
''')
