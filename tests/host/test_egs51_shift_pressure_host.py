"""Shift pressure arithmetic boundaries and all production conversion callers."""
import unittest
from owner_port_test import compile_run
from owner_port_test import method


class ShiftPressureTests(unittest.TestCase):
    def test_pressure_boundaries(self):
        compile_run(r'''
#include "models/egs51_shift_pressure.h"
#include <cassert>
using namespace Egs51ShiftPressure;
int main() {
 assert(available(14000,1000,1500)==19500);
 assert(available(1000,1001,1500)==0);
 assert(available(1000,1000,1500)==0);
 assert(available(65535,0,65535)==65535);
 assert(available(14000,1000,0)==0);
 assert(solenoid(-200,14000,1000,1500)==1000);
 assert(solenoid(1,14000,1000,1500)==1000);
 assert(solenoid(3,14000,1000,1500)==1002);
 assert(solenoid(19500,14000,1000,1500)==14000);
 assert(solenoid(INT32_MAX,14000,1000,1500)==14000);
 assert(solenoid(100,14000,1000,0)==14000);
 assert(solenoid(100,1000,1001,1500)==1000);
 assert(solenoid(100,0,1000,1500)==0);
 for(unsigned gain: {1u,37u,1000u,1500u,65535u}) {
  unsigned previous=0;
  for(int request=-1;request<=65535;++request) {
   unsigned result=solenoid(request,14000,1000,gain);
   assert(result>=previous && result>=1000 && result<=14000);
   previous=result;
  }
 }
}
'''.replace('#include <cassert>', '#include <cassert>\n#include <initializer_list>'))

    def test_production_callers(self):
        source=r'''
#include "models/egs51_shift_pressure.h"
#include <cassert>
constexpr unsigned SHIFT_ARRAY_LEN=8;
struct { uint16_t shift_reg_spring_pressure=1000;
 uint16_t shift_spc_gain[8]={1500,1500,1500,1500,1500,1500,1500,1500};
} hydraulic;
auto HYDR_PTR=&hydraulic;
struct PressureManager {
 uint16_t get_max_solenoid_pressure() { return 14000; }
 uint16_t get_max_shift_pressure(uint8_t);
};
struct ShiftHelpers {
 static uint16_t correct_shift_shift_pressure(PressureManager*,int16_t,uint8_t);
};
struct Adapt { int get_adapt_spc_offset(uint8_t) { return 300; } } adapt;
struct Sid { struct { uint8_t map_idx=0; } inf;
 Adapt* adaptation_mgr=nullptr; } data;
struct ShiftingAlgorithm {
 PressureManager* pm; Sid* sid=&data;
 uint8_t adapt_p_map_idx() { return 0; }
 uint16_t correct_shift_shift_pressure(int);
};
'''
        source += method('src/pressure_manager.cpp', 'uint16_t PressureManager::get_max_shift_pressure(')
        source += method('src/shifting_algo/shifting_algo_helpers.cpp', 'uint16_t ShiftHelpers::correct_shift_shift_pressure(')
        source += method('src/shifting_algo/s_algo.cpp', 'uint16_t ShiftingAlgorithm::correct_shift_shift_pressure(')
        source += r'''
int main() {
 PressureManager pm; ShiftingAlgorithm algo{&pm};
 assert(pm.get_max_shift_pressure(0)==19500);
 assert(pm.get_max_shift_pressure(8)==0);
 assert(pm.get_max_shift_pressure(255)==0);
 assert(ShiftHelpers::correct_shift_shift_pressure(&pm,1500,0)==2000);
 assert(algo.correct_shift_shift_pressure(1500)==2000);
 data.adaptation_mgr=&adapt;
 assert(algo.correct_shift_shift_pressure(1500)==2200);
 assert(algo.correct_shift_shift_pressure(-400)==1000);
 assert(algo.correct_shift_shift_pressure(20000)==14000);
 data.inf.map_idx=255;
 assert(algo.correct_shift_shift_pressure(1500)==14000);
 assert(ShiftHelpers::correct_shift_shift_pressure(&pm,1500,255)==14000);
 data.inf.map_idx=0; hydraulic.shift_spc_gain[0]=0;
 assert(pm.get_max_shift_pressure(0)==0);
 assert(algo.correct_shift_shift_pressure(1500)==14000);
 assert(ShiftHelpers::correct_shift_shift_pressure(&pm,1500,0)==14000);
 hydraulic.shift_reg_spring_pressure=15000;
 assert(pm.get_max_shift_pressure(1)==0);
 assert(ShiftHelpers::correct_shift_shift_pressure(&pm,1500,1)==14000);
}
'''
        compile_run(source)
