#pragma once
#include <stdint.h>

namespace Egs51InletPressure {
// Bank1 D8BD: interpolate by a truncated unsigned magnitude.
inline uint16_t inlet(uint32_t working, uint16_t x0, uint16_t x1,
                      uint16_t y0, uint16_t y1) {
    if (working <= x0) return y0;
    if (working >= x1 || x1 <= x0) return y1;
    const uint32_t distance = working-x0, span = x1-x0;
    return y1 >= y0 ? y0 + uint32_t(y1-y0)*distance/span :
        y0 - uint32_t(y0-y1)*distance/span;
}
// D905..D97B: inlet gap gain, positive offset guard, then correction.
// Keep products wide and cap output instead of copying OEM word wrapping.
inline uint16_t corrected(uint16_t requested, uint16_t inlet_pressure,
                          uint16_t inlet_ceiling, uint16_t percent,
                          uint16_t offset, uint16_t maximum) {
    if (requested >= inlet_pressure) return maximum;
    const uint32_t sum = uint32_t(requested) + offset;
    uint64_t output = requested;
    if (sum < 0x8000 && inlet_ceiling > inlet_pressure) {
        const uint32_t gain = uint32_t(percent) * (inlet_ceiling-inlet_pressure) / 1000;
        output += uint64_t(gain) * sum / 1000;
    }
    return output > maximum ? maximum : uint16_t(output);
}
}
