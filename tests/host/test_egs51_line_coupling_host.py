"""OEM 1->2 working-pressure coupling and production inlet-correction wiring."""
import unittest
from owner_port_test import compile_run
from owner_port_test import method


class LineCouplingTests(unittest.TestCase):
    def test_qualification_rounding_and_saturation(self):
        compile_run(r'''
#include "models/egs51_line_coupling.h"
#include "models/egs51_inlet_pressure.h"
#include <cassert>
using Egs51LineCoupling::working_pressure;
int main() {
 assert(working_pressure(5750,true,2000,37)==5010);
 assert(working_pressure(5750,false,2000,37)==5750);
 assert(working_pressure(5750,true,2000,0)==5750);
 assert(working_pressure(1000,true,2702,37)==1);
 assert(working_pressure(1000,true,2703,37)==0);
 assert(working_pressure(1000,true,14000,37)==0);
 assert(working_pressure(0,true,0,37)==0);
 assert(working_pressure(1000,true,65535,65535)==0);
 assert(working_pressure(UINT32_MAX,true,65535,65535)==
        UINT32_MAX-(uint64_t(65535)*65535/100));
 for(unsigned spc=0;spc<65536;++spc) {
  unsigned reduction=spc*37/100;
  unsigned expected=reduction>=5000?0:5000-reduction;
  assert(working_pressure(5000,true,spc,37)==expected);
 }
}
''')

    def test_actual_pressure_manager(self):
        source=r'''
#include "models/egs51_line_coupling.h"
#include "models/egs51_inlet_pressure.h"
#include <cassert>
#include <algorithm>
enum class GearboxGear { First,Reverse_First,Second };
enum class GearChange { _IDLE,_1_2,_2_1,_2_3,_3_2,_3_4,_4_3,_4_5,_5_4 };
enum class InterpType { Linear };
float interpolate_float(float x,float y0,float y1,float x0,float x1,InterpType) {
 if(x<=x0) return y0;
 if(x>=x1) return y1;
 return y0+(y1-y0)*(x-x0)/(x1-x0);
}
struct {
 uint16_t p_multi_1=1000,p_multi_other=1000;
 uint16_t extra_pressure_adder_r1_1=1500,extra_pressure_adder_other_gears=1000;
 uint16_t extra_pressure_pump_speed_min=1000,extra_pressure_pump_speed_max=4000;
 uint16_t lp_reg_spring_pressure=1000;
 uint16_t inlet_pressure_output_min=1000,inlet_pressure_output_max=2000;
 uint16_t inlet_pressure_input_min=0,inlet_pressure_input_max=10000;
 uint16_t shift_pressure_addr_percent=30,inlet_pressure_offset=1000;
 uint16_t shift_pressure_factor_percent=37;
} hydraulic;
auto HYDR_PTR=&hydraulic;
struct Sensor { uint16_t engine_rpm=2500; } sensor;
struct PressureManager {
 Sensor* sensor_data=&sensor;
 uint16_t target_modulating_pressure=4000,target_shift_pressure=2000;
 uint16_t calculated_working_pressure=0,calculated_inlet_pressure=0;
 uint16_t get_max_solenoid_pressure() { return 14000; }
 uint16_t calc_current_linear_sol(uint16_t,GearboxGear,GearChange);
};
'''+method('src/pressure_manager.cpp',
          'uint16_t PressureManager::calc_current_linear_sol(')+r'''
int main() {
 PressureManager pm;
 assert(pm.calc_current_linear_sol(1000,GearboxGear::First,GearChange::_1_2)==1028);
 assert(pm.calculated_working_pressure==5010 && pm.calculated_inlet_pressure==1501);
 pm.calc_current_linear_sol(2000,GearboxGear::First,GearChange::_1_2);
 assert(pm.calculated_working_pressure==5010); // SPC and MPC share the same estimate.
 for(auto shift:{GearChange::_2_3,GearChange::_3_2,GearChange::_3_4,
                 GearChange::_4_3,GearChange::_4_5,GearChange::_5_4}) {
  pm.calc_current_linear_sol(1000,GearboxGear::Second,shift);
  assert(pm.calculated_working_pressure==5500);
 }
 pm.calc_current_linear_sol(1000,GearboxGear::Second,GearChange::_2_1);
 assert(pm.calculated_working_pressure==5750); // No reverse-direction coupling.
 pm.calc_current_linear_sol(1000,GearboxGear::First,GearChange::_IDLE);
 assert(pm.calculated_working_pressure==5000);
 hydraulic.shift_pressure_factor_percent=0;
 pm.calc_current_linear_sol(1000,GearboxGear::First,GearChange::_1_2);
 assert(pm.calculated_working_pressure==5750);
 hydraulic.shift_pressure_factor_percent=37;
 pm.target_shift_pressure=14000;
 assert(pm.calc_current_linear_sol(14000,GearboxGear::First,GearChange::_1_2)==14000);
 assert(pm.target_shift_pressure==14000); // Final pressure-matching request retained.
 pm.target_shift_pressure=20000;
 assert(pm.calc_current_linear_sol(500,GearboxGear::First,GearChange::_1_2)==545);
 assert(pm.calculated_working_pressure==0 && pm.calculated_inlet_pressure==1000);
 hydraulic.inlet_pressure_offset=32768;
 assert(pm.calc_current_linear_sol(500,GearboxGear::First,GearChange::_1_2)==500);
 hydraulic.inlet_pressure_offset=1000;
 hydraulic.shift_pressure_addr_percent=65535;
 assert(pm.calc_current_linear_sol(500,GearboxGear::First,GearChange::_1_2)==14000);
}
'''
        compile_run(source)
