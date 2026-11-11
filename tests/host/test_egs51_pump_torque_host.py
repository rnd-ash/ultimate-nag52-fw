"""Staged pump curve, overflow/fault boundaries and actual production wiring."""
import unittest
from owner_port_test import compile_run
from owner_port_test import method


class PumpTorqueTests(unittest.TestCase):
    def test_curve_rounding_limits_and_faults(self):
        compile_run(r'''
#include "models/egs51_pump_torque.h"
#include <cassert>
using Egs51PumpTorque::calculate;
int main() {
 uint16_t axis[2]={0,3},curve[2]={10000,9998};
 assert(calculate(6000,6,axis,curve,2,5000)==3600);
 assert(calculate(6000,6,axis,curve,2,400)==400);
 assert(calculate(6000,6,axis,curve,2,0)==0);
 uint16_t large_axis[2]={0,2500},large_curve[2]={65535,65535};
 assert(calculate(60000,0,large_axis,large_curve,2,400)==400); // No signed overflow.
 assert(calculate(65534,0,large_axis,large_curve,2,65535)==INT16_MAX-1);
 assert(calculate(1,65534,large_axis,large_curve,2,400)==0); // Wide ratio clamps.
 assert(calculate(0,0,axis,curve,2,400)==INT16_MAX);
 assert(calculate(UINT16_MAX,0,axis,curve,2,400)==INT16_MAX);
 assert(calculate(INT16_MAX,0,axis,curve,2,400)==INT16_MAX);
 assert(calculate(1000,UINT16_MAX,axis,curve,2,400)==INT16_MAX);
 assert(calculate(1000,INT16_MAX,axis,curve,2,400)==INT16_MAX);
 assert(calculate(1000,0,nullptr,nullptr,0,400)==INT16_MAX);
 axis[1]=0;
 assert(calculate(1000,0,axis,curve,2,400)==INT16_MAX); // Duplicate axis.
 axis[0]=100;axis[1]=99;
 assert(calculate(1000,0,axis,curve,2,400)==INT16_MAX); // Reversed axis.
}
''')

    def test_actual_model_logged_map_and_cap(self):
        compile_run(r'''
#include "models/egs51_pump_torque.h"
#include <cassert>
struct {
 uint16_t pump_map_x[11]={0,104,381,575,674,825,883,1000,1056,1500,2500};
 uint16_t pump_map_z[11]={4211,4276,4252,4187,3902,3106,2724,0,797,3252,10406};
} calibration;
auto TCC_CFG_PTR=&calibration;
struct { uint16_t engine_drag_torque=400; } VEHICLE_CONFIG;
namespace InputTorqueModel { int16_t get_pump_torque(uint16_t,uint16_t); }
'''+method('src/models/input_torque.cpp',
          'int16_t InputTorqueModel::get_pump_torque(')+r'''
int main() {
 using InputTorqueModel::get_pump_torque;
 assert(get_pump_torque(1000,0)==42);
 assert(get_pump_torque(1000,1000)==0);
 assert(get_pump_torque(2400,0)==242);
 assert(get_pump_torque(4500,0)==400);
 VEHICLE_CONFIG.engine_drag_torque=100;
 assert(get_pump_torque(2400,0)==100); // Existing configured cap is retained.
 assert(get_pump_torque(0,0)==INT16_MAX);
 assert(get_pump_torque(UINT16_MAX,0)==INT16_MAX);
 assert(get_pump_torque(1000,UINT16_MAX)==INT16_MAX);
 calibration.pump_map_x[5]=calibration.pump_map_x[4];
 assert(get_pump_torque(1000,0)==INT16_MAX);
}
''')
