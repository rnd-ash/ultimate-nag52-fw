#pragma once
#include <stdint.h>

namespace Egs51ConverterTorque {
// Bank0 12BE/1412: subtract a truncated magnitude for descending curves.
inline uint16_t percentage(uint32_t ratio, uint16_t x0, uint16_t x1,
                           uint16_t z0, uint16_t z1) {
    if (ratio <= x0) return z0;
    if (ratio >= x1 || x1 <= x0) return z1;
    const uint32_t distance = ratio-x0;
    const uint32_t span = x1-x0;
    return z1 >= z0 ? z0 + uint32_t(z1-z0)*distance/span :
        z0 - uint32_t(z0-z1)*distance/span;
}
// CF6A scales qualified torque by the whole percentage, truncating toward zero.
// Reserve INT16_MAX for unavailable; keep wide signed intermediates.
inline int16_t scale(int16_t torque, uint16_t percent) {
    if (torque == INT16_MAX) return INT16_MAX;
    const int32_t result = int32_t(torque)*percent/100;
    return result >= INT16_MAX ? INT16_MAX-1 :
        result < INT16_MIN ? INT16_MIN : int16_t(result);
}
inline bool unavailable_speed(uint16_t rpm) {
    // Retain the legacy signed sentinel and accept the uint16_t CAN sentinel.
    return rpm == INT16_MAX || rpm == UINT16_MAX;
}
}
