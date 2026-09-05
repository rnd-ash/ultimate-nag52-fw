#ifndef TCU_IO_LOGIC_H
#define TCU_IO_LOGIC_H

#include <stdint.h>

uint16_t tcuio_calc_turbine_rpm_safe(uint16_t n2, uint16_t n3, float ratio_2_1);

#endif