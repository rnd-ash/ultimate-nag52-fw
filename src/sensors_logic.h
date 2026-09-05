#ifndef SENSORS_LOGIC_H
#define SENSORS_LOGIC_H

#include <stdint.h>

bool sensors_try_calc_atf_resistance(int adc_voltage_mv, uint16_t r2_resistance_ohm, int* out_resistance_ohm);

#endif