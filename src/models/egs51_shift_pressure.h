#pragma once
#include <stdint.h>

namespace Egs51ShiftPressure {
// Bank1 2BB2: available clutch pressure after the regulator spring and gain.
// Keep the OEM /1000 stage, but bound unsupported underflow/word wrapping.
inline uint16_t available(uint16_t maximum, uint16_t spring, uint16_t gain) {
    if (maximum <= spring) return 0;
    const uint32_t pressure = uint32_t(maximum-spring) * gain / 1000;
    return pressure > UINT16_MAX ? UINT16_MAX : uint16_t(pressure);
}

// Bank1 7093: signed-positive clutch demand, ceiling, inverse gain, spring.
// Adaptation is supplied by the caller in native pressure units. Invalid gain
// cannot be inverted: retain the maximum solenoid demand instead of dividing.
inline uint16_t solenoid(int32_t requested, uint16_t maximum,
                         uint16_t spring, uint16_t gain) {
    if (!gain || spring >= maximum) return maximum;
    const uint16_t ceiling = available(maximum, spring, gain);
    const uint32_t clutch = requested <= 0 ? 0 :
        requested >= ceiling ? ceiling : uint32_t(requested);
    const uint32_t pressure = clutch * 1000 / gain + spring;
    return pressure > maximum ? maximum : uint16_t(pressure);
}
}
