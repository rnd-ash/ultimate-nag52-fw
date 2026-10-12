#pragma once
#include <stdint.h>

namespace Egs51Centrifugal {
// Bank1 43C5: density correction, squared speed, clutch factor, then /10.
// Preserve stage truncation but keep intermediates wide instead of OEM wrapping.
inline uint64_t pressure(uint16_t speed, int encoded_temperature,
                         uint16_t density_at_minus_50, uint16_t density_drop,
                         uint16_t clutch_factor) {
    if (clutch_factor == 0) return 0;
    // The OEM temperature operand is a byte (degrees C + 50).
    const unsigned temperature = encoded_temperature < 0 ? 0 :
        encoded_temperature > 255 ? 255 : unsigned(encoded_temperature);
    const uint32_t drop = uint32_t(density_drop) * temperature / 100;
    if (drop >= density_at_minus_50) return 0; // Do not underflow density.
    const uint32_t density = density_at_minus_50 - drop;
    const uint32_t squared = uint32_t(speed) * speed / 1000;
    return (uint64_t(squared) * density / clutch_factor) / 10;
}
}
