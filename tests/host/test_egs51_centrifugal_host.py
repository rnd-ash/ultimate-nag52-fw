"""Staged centrifugal pressure arithmetic and actual clutch selection."""
import unittest
from owner_port_test import compile_run
from owner_port_test import method


class CentrifugalTests(unittest.TestCase):
    def test_stages_and_wide_boundaries(self):
        compile_run(r'''
#include "models/egs51_centrifugal.h"
#include <cassert>
using Egs51Centrifugal::pressure;
int main() {
 assert(pressure(2000,75,889,60,6117)==55);
 assert(pressure(2000,75,889,60,1635)==206);
 assert(pressure(99,75,889,60,6117)==0);
 assert(pressure(100,75,889,60,6117)==0);
 assert(pressure(1000,1,889,60,6117)==14);
 assert(pressure(5000,255,889,60,6117)==300);
 assert(pressure(5000,300,889,60,6117)==pressure(5000,255,889,60,6117));
 assert(pressure(5000,-1,889,60,6117)==pressure(5000,0,889,60,6117));
 assert(pressure(5000,100,10,20,6117)==0); // Density cannot become negative.
 assert(pressure(5000,0,889,60,0)==0); // Disabled clutch factor.
 // Independent wide equation, well beyond the OEM word-wrap domain.
 assert(pressure(60000,0,889,60,6117)==52319);
 assert(pressure(65535,0,65535,0,1)==28146207726ULL);
 for(unsigned speed=0; speed<8000; ++speed) {
  auto expected=(((uint64_t(speed)*speed/1000)*844)/6117)/10;
  assert(pressure(speed,75,889,60,6117)==expected);
 }
}
''')

    def test_actual_pressure_manager_clutch_selection(self):
        source = r'''
#include "models/egs51_centrifugal.h"
#include <cassert>
enum class Clutch { K1,K2,K3,B1,B2,B3 };
struct { uint16_t atf_density_minus_50c=889,atf_density_drop_per_c=60;
 uint16_t atf_density_centrifugal_force_factor[3]={100,6117,1635}; } mech;
auto MECH_PTR=&mech;
struct Sensor { int atf_temp=25; } sensor;
struct PressureManager {
 Sensor* sensor_data=&sensor;
 float calculate_centrifugal_force_for_clutch(Clutch,uint16_t,uint16_t);
};
'''+method('src/pressure_manager.cpp',
          'float PressureManager::calculate_centrifugal_force_for_clutch(')+r'''
int main() {
 PressureManager pm;
 assert(pm.calculate_centrifugal_force_for_clutch(Clutch::K2,2000,1000)==55);
 assert(pm.calculate_centrifugal_force_for_clutch(Clutch::K3,1000,2000)==206);
 for(auto c:{Clutch::K1,Clutch::B1,Clutch::B2,Clutch::B3})
  assert(pm.calculate_centrifugal_force_for_clutch(c,2000,2000)==0);
 mech.atf_density_centrifugal_force_factor[1]=40000; // Actual log calibration.
 assert(pm.calculate_centrifugal_force_for_clutch(Clutch::K2,2000,1000)==8);
 mech.atf_density_centrifugal_force_factor[1]=0;
 assert(pm.calculate_centrifugal_force_for_clutch(Clutch::K2,2000,1000)==0);
 sensor.atf_temp=150;
 assert(pm.calculate_centrifugal_force_for_clutch(Clutch::K3,1000,2000)==188);
}
'''
        compile_run('#include <initializer_list>\n'+source)
