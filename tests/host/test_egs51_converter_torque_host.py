"""Converter percentage staging, signed scaling and actual model wiring."""
import unittest
from owner_port_test import compile_run
from owner_port_test import method


class ConverterTorqueTests(unittest.TestCase):
    def test_curve_signed_torque_and_bounds(self):
        compile_run(r'''
#include "models/egs51_converter_torque.h"
#include <cassert>
using namespace Egs51ConverterTorque;
int main() {
 assert(percentage(0,0,917,179,100)==179);
 assert(percentage(500,0,917,179,100)==136); // Descending truncated magnitude.
 assert(percentage(917,0,917,179,100)==100);
 assert(percentage(1000000,0,917,179,100)==100); // Wide ratio, no wrap.
 assert(percentage(500,0,1000,100,200)==150);
 assert(percentage(100,100,100,150,100)==150); // No division at degenerate axis.
 assert(scale(100,136)==136 && scale(-100,136)==-136);
 assert(scale(-1,136)==-1 && scale(1,136)==1); // Truncate toward zero.
 assert(scale(20000,179)==INT16_MAX-1);
 assert(scale(INT16_MIN,179)==INT16_MIN);
 assert(scale(INT16_MAX,100)==INT16_MAX); // Sentinel is preserved.
 assert(scale(INT16_MIN,65535)==INT16_MIN);
 assert(scale(INT16_MAX-1,65535)==INT16_MAX-1);
 assert(scale(-100,0)==0);
 assert(unavailable_speed(UINT16_MAX) && unavailable_speed(INT16_MAX));
}
''')

    def test_actual_model_and_configured_curve(self):
        source=r'''
#include "models/egs51_converter_torque.h"
#include <cassert>
#include <cmath>
#include <initializer_list>
struct { uint16_t multiplier_map_x[2]={0,917};
 uint16_t multiplier_map_z[2]={179,100}; } calibration;
auto TCC_CFG_PTR=&calibration;
namespace InputTorqueModel {
 int16_t get_input_torque(uint16_t,uint16_t,int16_t);
 float get_input_torque_factor(uint16_t,uint16_t);
}
'''
        for signature in ('static uint16_t converter_percentage(',
                          'int16_t InputTorqueModel::get_input_torque(',
                          'float InputTorqueModel::get_input_torque_factor('):
            source+=method('src/models/input_torque.cpp',signature)+'\n'
        compile_run(source+r'''
int main() {
 using namespace InputTorqueModel;
 assert(get_input_torque(1000,500,100)==136);
 assert(get_input_torque(1000,500,-100)==-136);
 assert(std::fabs(get_input_torque_factor(1000,500)-1.36f)<0.00001f);
 assert(get_input_torque(1000,1000,100)==100);
 assert(get_input_torque(0,500,-100)==-100);
 assert(get_input_torque_factor(0,500)==1.0f);
 assert(get_input_torque(1000,500,INT16_MAX)==INT16_MAX);
 for(auto missing:{uint16_t(INT16_MAX),uint16_t(UINT16_MAX)}) {
  assert(get_input_torque(missing,500,100)==100);
  assert(get_input_torque(1000,missing,100)==100);
  assert(get_input_torque_factor(missing,500)==1.0f);
  assert(get_input_torque_factor(1000,missing)==1.0f);
 }
 assert(get_input_torque(1,65534,100)==100); // Wide speed ratio selects final point.
 assert(get_input_torque(1000,0,20000)==INT16_MAX-1);
 calibration.multiplier_map_x[1]=1000;
 calibration.multiplier_map_z[0]=200;
 assert(get_input_torque(1000,500,100)==150); // Custom calibration retained.
}
''')
