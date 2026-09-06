#include "sensors_logic.h"

bool sensors_try_calc_atf_resistance(int adc_voltage_mv, uint16_t r2_resistance_ohm, int* out_resistance_ohm) {
    if (out_resistance_ohm == nullptr) {
        return false;
    }
    int denom = 3300 - adc_voltage_mv;
    if (adc_voltage_mv < 0 || denom <= 0) {
        return false;
    }
    *out_resistance_ohm = (adc_voltage_mv * r2_resistance_ohm) / denom;
    return true;
}