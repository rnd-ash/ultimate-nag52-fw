#pragma once
#include <cstdint>

namespace Egs51Torque {
// Engine response outlasts TORQUE_REQ_EN. Do not learn that response as a
// driver/static offset during a brief request gap. Only the offset is held:
// current driver demand and engine limits are still applied on every read.
class DemandCorrection {
public:
    // Covers the observed ~80–100 ms request gaps with a bounded recovery
    // allowance. This is not an OEM calibration or a guarantee of engine settle.
    static constexpr uint32_t RECOVERY_MS = 500;

    int16_t update(uint32_t now_ms, bool request, int16_t demand,
                   int16_t actual, int16_t maximum) {
        if (request) {
            last_request_ms = now_ms;
            recovering = true;
        } else if (recovering && uint32_t(now_ms - last_request_ms) >= RECOVERY_MS) {
            recovering = false;
        }
        if (!request && !recovering) {
            delta = int(demand) - actual;
        }
        int result = demand;
        if (request) {
            result = int(demand) - delta;
            if (result < actual) { result = actual; }
        }
        const int ceiling = maximum > 0 ? maximum : 0;
        if (result < 0) { result = 0; }
        if (result > ceiling) { result = ceiling; }
        return result;
    }

private:
    int delta = 0;
    uint32_t last_request_ms = 0;
    bool recovering = false;
};

}
