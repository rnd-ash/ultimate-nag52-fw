#include "can_egs51_logic.h"

#include <limits.h>

namespace {

int32_t round_to_i32_local(float value) {
    if (value >= (float)INT32_MAX) {
        return INT32_MAX;
    }
    if (value <= (float)INT32_MIN) {
        return INT32_MIN;
    }
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) : (int32_t)(value - 0.5f);
}

uint8_t round_to_u8_sat_local(float value) {
    if (value <= 0.0f) {
        return 0u;
    }
    if (value >= 255.0f) {
        return 255u;
    }
    return (uint8_t)(value + 0.5f);
}

} // namespace

uint8_t egs51_torque_request_to_raw(float amount_nm) {
    float scaled = amount_nm / 3.0f;
    if (scaled < 0.0f) {
        return 0;
    }
    if (scaled > 253.0f) {
        // 0xFE is reserved as "request inactive" in GS_218.
        return 0xFD;
    }
    return round_to_u8_sat_local(scaled);
}

Egs51FreezeResult egs51_apply_freeze_logic(bool freeze, int16_t driver_converted, int16_t static_converted, int16_t req_static_torque_delta) {
    Egs51FreezeResult result = {
        .driver_converted = driver_converted,
        .req_static_torque_delta = req_static_torque_delta,
    };

    if (freeze) {
        int16_t frozen = driver_converted - req_static_torque_delta;
        if (frozen < static_converted) {
            frozen = static_converted;
        }
        result.driver_converted = frozen;
    } else {
        result.req_static_torque_delta = driver_converted - static_converted;
    }
    return result;
}

uint8_t egs51_tcc_multiplier_to_raw(float multi) {
    float raw = multi * 100.0f;
    if (raw < 100.0f) {
        raw = 100.0f;
    }
    if (raw > 254.0f) {
        raw = 254.0f;
    }
    return round_to_u8_sat_local(raw);
}

uint16_t egs51_decode_wheel_speed_or_sna(uint16_t wheel_raw) {
    // ESP51 uses 0x3FFF as wheel-speed signal-not-available sentinel.
    if (wheel_raw == 0x3FFF) {
        return UINT16_MAX;
    }
    return wheel_raw;
}

bool egs51_infer_engine_limp(bool temp_kl, bool uehitz, bool diag_kl) {
    // Conservative fallback heuristic until a direct limp-status source is decoded.
    // Avoid using DIAG lamp alone to prevent false limp classification from generic MIL events.
    return uehitz || (temp_kl && diag_kl);
}

int16_t egs51_apply_max_torque_factor(int16_t base_max_nm, uint8_t max_trq_factor_raw) {
    // 0xFF means value unavailable in this protocol family.
    if (base_max_nm == INT16_MAX || max_trq_factor_raw == UINT8_MAX) {
        return base_max_nm;
    }

    // Same style as other families: factor uses approx 1/128 scaling.
    float factor = (float)max_trq_factor_raw * 0.0078f;
    int32_t scaled = round_to_i32_local((float)base_max_nm * factor);
    if (scaled > INT16_MAX) {
        return INT16_MAX;
    }
    if (scaled < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)scaled;
}
