#pragma once
#include <cstdint>

namespace Egs51EngineRpm {
// A0215451432 BEC3: FEFD substitute and FFAE engine-active threshold.
constexpr uint16_t SUBSTITUTE_RPM = 750;
constexpr uint16_t ACTIVE_RPM = 450;
struct Sample {
    uint16_t rpm;
    bool valid;
    bool active;
};
inline Sample qualify(uint16_t raw, bool fault = false) {
    const bool valid = !fault && raw != UINT16_MAX;
    const uint16_t rpm = valid ? raw : SUBSTITUTE_RPM;
    // OEM's activity bit uses the effective value, independently of its fault
    // flag. The NAG52 adapter must also check validity before enabling control.
    return {rpm, valid, rpm >= ACTIVE_RPM};
}
}
