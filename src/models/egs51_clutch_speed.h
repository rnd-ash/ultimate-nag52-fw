#pragma once
#include <stdint.h>
#include <math.h>

namespace Egs51ClutchSpeed {
struct Speeds { int16_t applying, releasing, rear_sun; };
inline int16_t bounded(int64_t value) {
    return value > INT16_MAX ? INT16_MAX : value < INT16_MIN ? INT16_MIN : int16_t(value);
}
// GearRatioInfo is initialized from mechanical calibration words / 1000.
inline uint16_t ratio_word(float ratio) {
    if (!isfinite(ratio) || ratio <= 0 || ratio > 65.535f) return 0;
    return uint16_t(double(ratio) * 1000 + 0.5);
}
// Bank0 C6C6 forward-shift branch. Keep each unsigned division separate,
// then subtract in a wide signed type; avoid OEM word wrapping.
inline Speeds calculate(unsigned shift, uint16_t n2, uint16_t n3,
                        uint16_t turbine, uint16_t output,
                        uint16_t second, uint16_t third, uint16_t fourth) {
    if (shift < 1 || shift > 8) return {0,0,0};
    const unsigned pair = (shift - 1) % 4;
    if (n3 == UINT16_MAX || (pair == 0 && n2 == UINT16_MAX) ||
        (pair == 3 && (n2 == UINT16_MAX || turbine == UINT16_MAX)) ||
        (pair != 0 && output == UINT16_MAX) ||
        (pair == 1 && (third == 0 || second <= third)) ||
        (pair >= 2 && (fourth == 0 || third <= fourth)))
        return {INT16_MAX,INT16_MAX,INT16_MAX};
    int64_t a = 0, b = 0, rear = 0;
    if (pair == 0 || pair == 3) {
        a = int64_t(n2) - n3;
        b = n3;
        if (pair == 3)
            rear = int64_t(uint64_t(third) * output / (third-fourth)) -
                int64_t(uint64_t(1000) * turbine / (third-fourth));
    } else if (pair == 1) {
        a = int64_t(n3) - int64_t(uint64_t(third) * output / 1000);
        const uint64_t scaled_output = uint64_t(third) * output / (second-third);
        b = int64_t(uint64_t(second) * scaled_output / 1000) -
            int64_t(uint64_t(third) * n3 / (second-third));
    } else {
        b = int64_t(uint64_t(third) * output / (third-fourth)) -
            int64_t(uint64_t(1000) * n3 / (third-fourth));
        rear = b;
        a = int64_t(n3) - b;
    }
    const bool a_applies = (pair == 0 || pair == 3) ? (shift == 1 || shift == 8) : shift <= 4;
    return {bounded(a_applies ? a : b), bounded(a_applies ? b : a), bounded(rear)};
}
}
