#include "can_egs_scaling_logic.h"

uint8_t egs52_encode_turbine_torque_loss(uint16_t loss_nm) {
    uint32_t scaled = (uint32_t)loss_nm * 4u;
    if (scaled > 0xFEu) {
        scaled = 0xFEu;
    }
    return (uint8_t)scaled;
}

uint16_t egs52_encode_wheel_torque_multi_factor(float ratio) {
    // Negative or NaN ratios are implausible for this signal.
    if (!(ratio >= 0.0f)) {
        return 0x7FFu;
    }

    float scaled_ratio = ratio * 0.05f;
    if (scaled_ratio > 2046.0f) {
        scaled_ratio = 2046.0f;
    }

    return (uint16_t)scaled_ratio;
}

uint16_t egs53_encode_wheel_torque_multi_factor(float ratio) {
    // Negative or NaN ratios are implausible for this signal.
    if (!(ratio >= 0.0f)) {
        return 0u;
    }

    float scaled_ratio = ratio * 100.0f;
    if (scaled_ratio > 16383.0f) {
        scaled_ratio = 16383.0f;
    }

    return (uint16_t)scaled_ratio;
}