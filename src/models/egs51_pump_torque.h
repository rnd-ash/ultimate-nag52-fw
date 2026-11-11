#pragma once
#include "egs51_converter_torque.h"

namespace Egs51PumpTorque {
inline int16_t calculate(uint16_t engine, uint16_t turbine,
                         const uint16_t* axis, const uint16_t* values,
                         unsigned count, uint16_t cap) {
    if (engine == 0 || Egs51ConverterTorque::unavailable_speed(engine) ||
        Egs51ConverterTorque::unavailable_speed(turbine) || count < 2)
        return INT16_MAX;
    for (unsigned i=1; i<count; ++i)
        if (axis[i] <= axis[i-1]) return INT16_MAX;
    const uint32_t ratio = uint32_t(turbine) * 1000 / engine;
    unsigned hi = 1;
    while (hi+1 < count && ratio > axis[hi]) ++hi;
    const uint16_t coefficient = Egs51ConverterTorque::percentage(
        ratio, axis[hi-1], axis[hi], values[hi-1], values[hi]);
    // Bank0 CA65: square/1000, coefficient product/10000, then /10.
    // Wide intermediates avoid OEM word wrapping and signed product overflow.
    const uint32_t squared = uint32_t(engine) * engine / 1000;
    uint64_t torque = uint64_t(coefficient) * squared / 10000 / 10;
    if (torque > cap) torque = cap;
    if (torque >= INT16_MAX) torque = INT16_MAX-1;
    return int16_t(torque);
}
}
