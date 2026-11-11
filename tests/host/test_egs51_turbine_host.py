"""Turbine arithmetic, calibrated TCUIO wiring and unavailable boundaries."""
from pathlib import Path
import unittest
from owner_port_test import compile_run
from owner_port_test import block

ROOT = Path(__file__).resolve().parents[2]


class TurbineTests(unittest.TestCase):
    def test_rounding_missing_and_wide_intermediates(self):
        compile_run(r'''
#include "models/egs51_turbine.h"
#include <cassert>
using Egs51Turbine::speed;
int main() {
 // Independent original-ROM C449 examples.
 assert(speed(1000,2000,3932,2408)==367);
 assert(speed(1001,999,3932,2408)==1002);
 assert(speed(100,101,3932,2408)==100);
 assert(speed(5000,1000,3932,2408)==7532);
 assert(speed(0,500,3932,2408)==0);
 assert(speed(2408,0,3932,2408)==3932);
 // Equal shaft speeds must be an exact identity, including high inputs.
 for(unsigned n=0;n<65535;++n) assert(speed(n,n,3932,2408)==n);
 assert(speed(2200,0,3600,2200)==3600); // Configured ratios, not fixed ROM.
 assert(speed(65535,0,3932,2408)==UINT16_MAX);
 assert(speed(0,65535,3932,2408)==UINT16_MAX);
 assert(speed(100,100,3932,0)==UINT16_MAX);
 assert(speed(100,100,0,2408)==UINT16_MAX);
 assert(speed(100,100,2408,2408)==UINT16_MAX);
 assert(speed(60000,0,3932,2408)==UINT16_MAX); // Never wrap to a low RPM.
 assert(speed(65534,0,65535,1)==UINT16_MAX);
 assert(speed(65534,65534,65535,1)==65534); // Wide sum still fits uint32_t.
 assert(speed(0,65534,65535,1)==0);
}
''')

    def test_actual_io_setter_and_calculation(self):
        source = (ROOT / 'src/tcu_io/tcu_io.cpp').read_text()
        methods = '\n'.join(block(source, marker) for marker in (
            'void TCUIO::set_2_1_ratios(', 'uint16_t TCUIO::calc_turbine_rpm('))
        compile_run(r'''
#include "models/egs51_turbine.h"
#include <cassert>
static uint16_t turbine_first_ratio=3932,turbine_second_ratio=2408;
namespace TCUIO {
 void set_2_1_ratios(uint16_t,uint16_t);
 uint16_t calc_turbine_rpm(uint16_t,uint16_t);
}
'''+methods+r'''
int main() {
 assert(TCUIO::calc_turbine_rpm(2408,0)==3932);
 TCUIO::set_2_1_ratios(3600,2200);
 assert(TCUIO::calc_turbine_rpm(2200,0)==3600);
 TCUIO::set_2_1_ratios(0,0);
 assert(TCUIO::calc_turbine_rpm(1000,1000)==UINT16_MAX);
}
''')
