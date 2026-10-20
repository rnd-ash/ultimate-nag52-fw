#pragma once
#include <cstdint>

namespace Egs51Turbine {
// A0215451432 bank0 C449: truncate each ratio term before subtraction.
// Use NAG52's configured first/second ratios rather than fixed ROM constants.
inline uint16_t speed(uint16_t n2, uint16_t n3, uint16_t first, uint16_t second) {
    if (n2 == UINT16_MAX || n3 == UINT16_MAX || second == 0 || first <= second) {
        return UINT16_MAX;
    }
    const uint32_t scaled_n2 = static_cast<uint32_t>(first) * n2 / second;
    const uint32_t scaled_n3 = static_cast<uint32_t>(first) * n3 / second;
    const uint32_t sum = scaled_n2 + n3;
    const uint32_t result = sum > scaled_n3 ? sum - scaled_n3 : 0;
    // NAG52 represents unavailable with UINT16_MAX. Keep wide intermediates
    // and reject an unrepresentable result rather than wrapping to a low RPM.
    // This boundary policy differs from OEM 16-bit intermediate wrapping.
    return result < UINT16_MAX ? static_cast<uint16_t>(result) : UINT16_MAX;
}
}
