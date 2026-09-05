#include "pressure_manager_logic.h"

#include <limits.h>

uint16_t pressure_manager_calc_max_shift_pressure(uint16_t max_solenoid_pressure, uint16_t shift_reg_spring_pressure, uint16_t shift_spc_gain) {
    if (max_solenoid_pressure <= shift_reg_spring_pressure) {
        return 0u;
    }

    uint32_t diff = (uint32_t)max_solenoid_pressure - (uint32_t)shift_reg_spring_pressure;
    uint32_t scaled = (diff * (uint32_t)shift_spc_gain) / 1000u;
    if (scaled > UINT16_MAX) {
        return UINT16_MAX;
    }
    return (uint16_t)scaled;
}