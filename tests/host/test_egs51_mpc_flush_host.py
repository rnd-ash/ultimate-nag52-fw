"""OEM flush boundaries and the production pressure-manager filter bypass."""
from pathlib import Path
import unittest
from owner_port_test import compile_run
from owner_port_test import method

ROOT = Path(__file__).resolve().parents[2]


class MpcFlushTests(unittest.TestCase):
    def test_word_timers_and_qualification(self):
        compile_run(r'''
#include "models/egs51_mpc_flush.h"
#include <cassert>
int main() {
 Egs51MpcFlush::State s;
 auto step=[&](unsigned raw,unsigned previous,int temp,unsigned rest=30000) {
  return s.update(true,raw,previous,100,temp,90,rest,50);
 };
 assert(!step(0,101,90)); // Pressure has not settled.
 assert(!step(0,100,89)); // Temperature below threshold.
 assert(!step(1,100,90)); // Loaded clutch, even at low demand.
 assert(!step(0,100,90,0)); // Disabled calibration.
 assert(step(0,100,90)); assert(s.remaining==50);
 for(int i=0;i<49;++i) { s.tick(); assert(step(0,0,90)); }
 s.tick(); assert(!step(0,0,90)); assert(s.remaining==30000);
 for(int i=0;i<29999;++i) { s.tick(); assert(!step(0,100,90)); }
 s.tick(); assert(step(0,100,90)); // Full word rest interval, no truncation.
 assert(!step(1,100,90)); assert(s.remaining==0);
 assert(step(0,100,90)); // Requalify after demand reset.
 assert(!s.update(false,0,0,100,90,90,30000,50));
 assert(s.remaining==0);
 assert(s.update(true,0,0,100,90,90,65535,0)); // Zero flush duration.
 assert(!s.update(true,0,0,100,90,90,65535,0));
 assert(s.remaining==65535); s.tick(); assert(s.remaining==65534);
}
''')

    def test_actual_pressure_manager(self):
        source = r'''
#include "models/egs51_mpc_flush.h"
#include <algorithm>
#include <cassert>
#include <cstdlib>
#define MAX(a,b) std::max(a,b)
enum class GearboxGear { Neutral, First, Reverse_First, Second };
enum class Clutch { K1 };
enum class CoefficientTy { Static };
struct { uint8_t strongest_loaded_clutch_idx[2]={0,0};
 uint16_t release_spring_pressure[6]={0}; } mech;
auto MECH_PTR=&mech;
struct { uint16_t extra_p_not_shifting=0,p_multi_1=1000,p_multi_other=1000;
 uint16_t lp_reg_spring_pressure=0,min_mpc_pressure=100;
 uint8_t mpc_flush_temp_threshold=90,filter_factor=1;
 uint16_t mpc_no_flush_time=30000,mpc_flush_time=2; } hydraulic;
auto HYDR_PTR=&hydraulic;
struct Sensor { int input_torque=0,atf_temp=40; } sensor;
uint8_t gear_to_idx_lookup(GearboxGear g) { return g==GearboxGear::Neutral?0:1; }
unsigned filter_calls=0;
uint16_t first_order_filter(uint8_t w,uint16_t target,uint16_t old) {
 ++filter_calls; return (old*w+target)/(w+1);
}
struct PressureManager {
 Sensor* sensor_data=&sensor;
 Egs51MpcFlush::State mpc_flush;
 uint16_t target_modulating_pressure=100,clutch_pressure=0;
 uint16_t get_max_solenoid_pressure() { return 14000; }
 float p_clutch_with_coef(GearboxGear,Clutch,int,CoefficientTy) { return clutch_pressure; }
 uint16_t find_working_mpc_pressure(GearboxGear,bool);
};
'''+method('src/pressure_manager.cpp',
          'uint16_t PressureManager::find_working_mpc_pressure(')+r'''
int main() {
 PressureManager pm;
 pm.target_modulating_pressure=200;
 assert(pm.find_working_mpc_pressure(GearboxGear::Neutral,true)==150);
 assert(filter_calls==1 && !pm.mpc_flush.flushing);
 pm.target_modulating_pressure=100;
 assert(pm.find_working_mpc_pressure(GearboxGear::Neutral,true)==0);
 assert(filter_calls==1 && pm.mpc_flush.remaining==2);
 pm.target_modulating_pressure=0;
 assert(pm.find_working_mpc_pressure(GearboxGear::Neutral,true)==0);
 assert(pm.find_working_mpc_pressure(GearboxGear::Neutral,true)==100);
 assert(pm.mpc_flush.remaining==30000);
 pm.clutch_pressure=500;
 assert(pm.find_working_mpc_pressure(GearboxGear::First,true)==500);
 assert(!pm.mpc_flush.flushing && pm.mpc_flush.remaining==0);
 pm.target_modulating_pressure=500;
 assert(pm.find_working_mpc_pressure(GearboxGear::Neutral,false)==300);
 assert(filter_calls==2); // Existing normal decrease filtering remains.
 pm.clutch_pressure=15000;
 assert(pm.find_working_mpc_pressure(GearboxGear::Second,true)==14000);
 sensor.atf_temp=39; pm.target_modulating_pressure=100;
 assert(pm.find_working_mpc_pressure(GearboxGear::Neutral,true)==100);
}
'''
        compile_run(source)
