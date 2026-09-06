#ifndef PRESSURE_MANAGER_LOGIC_H
#define PRESSURE_MANAGER_LOGIC_H

#include <stdint.h>

uint16_t pressure_manager_calc_max_shift_pressure(uint16_t max_solenoid_pressure, uint16_t shift_reg_spring_pressure, uint16_t shift_spc_gain);

#endif