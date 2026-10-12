#pragma once
#include <cstdint>

namespace Egs51Pedal {
// A0215451432 bank0 BC4A: FF02 replaces the raw pedal on X71 & 0x28
// or X70 bit 0. X1CA bit 0 retains the fault independently of the value.
constexpr uint8_t SUBSTITUTE = 64;
struct Sample {
    uint8_t pedal;
    bool valid;
};
inline Sample qualify(uint8_t raw, bool fault = false) {
    // NAG52 CAN uses 0..250, with 0xFF unavailable. Reserved values are
    // unavailable too; this domain check is an adapter policy, not ROM logic.
    const bool valid = !fault && raw <= 250;
    return {valid ? raw : SUBSTITUTE, valid};
}
}
