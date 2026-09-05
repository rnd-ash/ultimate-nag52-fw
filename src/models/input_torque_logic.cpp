#include "input_torque_logic.h"

#include <limits.h>

int32_t input_torque_engine_pow2_div1000(uint16_t engine_rpm) {
    uint64_t sq = (uint64_t)engine_rpm * (uint64_t)engine_rpm;
    return (int32_t)(sq / 1000u);
}

uint16_t input_torque_ratio_x1000(uint16_t input_rpm, uint16_t engine_rpm) {
    if (engine_rpm == 0u) {
        return UINT16_MAX;
    }
    uint32_t ratio = ((uint32_t)input_rpm * 1000u) / (uint32_t)engine_rpm;
    if (ratio > UINT16_MAX) {
        return UINT16_MAX;
    }
    return (uint16_t)ratio;
}

int16_t input_torque_scaled_clamped(int16_t static_torque, float multi) {
    float scaled = (float)static_torque * multi;
    if (scaled > (float)INT16_MAX) {
        return INT16_MAX;
    }
    if (scaled < (float)INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)scaled;
}