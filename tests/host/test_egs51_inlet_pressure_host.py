"""Inlet interpolation stages and OEM signed-positive offset guard."""
import unittest
from owner_port_test import compile_run


class InletPressureTests(unittest.TestCase):
    def test_interpolation_clamps_and_descending_rounding(self):
        compile_run(r'''
#include "models/egs51_inlet_pressure.h"
#include <cassert>
using Egs51InletPressure::inlet;
int main() {
 assert(inlet(0,3180,8820,2690,8330)==2690);
 assert(inlet(3180,3180,8820,2690,8330)==2690);
 assert(inlet(5000,3180,8820,2690,8330)==4510);
 assert(inlet(8820,3180,8820,2690,8330)==8330);
 assert(inlet(UINT32_MAX,3180,8820,2690,8330)==8330);
 assert(inlet(1,0,3,10,0)==7); // Subtract truncated magnitude, not float result.
 assert(inlet(2,0,3,0,10)==6);
 assert(inlet(0,0,0,10,20)==10); // Degenerate axes never divide by zero.
 assert(inlet(1,0,0,10,20)==20);
 for(unsigned w=3180;w<=8820;++w)
  assert(inlet(w,3180,8820,2690,8330)==w-490);
}
''')

    def test_offset_boundary_gain_stages_and_saturation(self):
        compile_run(r'''
#include "models/egs51_inlet_pressure.h"
#include <cassert>
using Egs51InletPressure::corrected;
int main() {
 assert(corrected(1000,2690,8330,30,1000,14000)==1338);
 assert(corrected(1000,2690,8330,30,31767,14000)==6537); // Offset sum 32767.
 assert(corrected(1001,2690,8330,30,31767,14000)==1001); // Sum 32768 suppresses.
 assert(corrected(1000,2690,8330,30,65535,14000)==1000); // No offset word wrap.
 assert(corrected(2690,2690,8330,30,1000,14000)==14000);
 assert(corrected(14000,2690,8330,30,1000,14000)==14000);
 assert(corrected(1000,2690,8330,0,1000,14000)==1000);
 assert(corrected(1000,5000,2690,30,1000,14000)==1000); // No negative gap gain.
 assert(corrected(1000,5000,65535,65535,1000,14000)==14000); // Wide/capped.
 assert(corrected(1000,5000,8330,30,1000,500)==500);
}
''')
