#pragma once
#include <stdint.h>

// Bank1 5480: idle MPC flush supervision. Durations are calibration counts;
// the caller owns their timebase, independently of the OEM timer service.
namespace Egs51MpcFlush {
struct State {
    bool flushing = false;
    uint16_t remaining = 0;

    void tick() {
        if (remaining != 0) --remaining;
    }

    bool update(bool enabled, uint16_t raw_target, uint16_t previous_pressure,
                uint16_t minimum, int temperature, uint8_t threshold,
                uint16_t rest_time, uint16_t flush_time) {
        if (!enabled || raw_target != 0 || previous_pressure > minimum ||
            temperature < threshold || rest_time == 0) {
            flushing = false;
            remaining = 0;
        } else if (remaining == 0) {
            flushing = !flushing;
            remaining = flushing ? flush_time : rest_time;
        }
        return flushing;
    }
};
}
