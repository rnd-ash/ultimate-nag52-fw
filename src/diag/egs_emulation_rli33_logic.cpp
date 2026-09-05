#include "egs_emulation_rli33_logic.h"

static uint16_t flip_u16(uint16_t x) {
    return ((x & 0x00FFu) << 8) | ((x & 0xFF00u) >> 8);
}

Egs51Rli33DerivedData egs51_build_rli33_derived(
    uint16_t corrected_spc_pressure,
    uint16_t corrected_mpc_pressure,
    uint16_t spc_target_current,
    uint16_t spc_actual_current,
    uint16_t mpc_target_current,
    uint16_t mpc_actual_current,
    uint16_t tcc_pwm_raw,
    uint16_t y3_pwm_raw,
    uint16_t y5_pwm_raw,
    uint16_t y4_pwm_raw
) {
    Egs51Rli33DerivedData ret = {
        .valve_flag = 0,
        .shift_valve_state = 0,
        .spc_pressure = flip_u16(corrected_spc_pressure),
        .mpc_pressure = flip_u16(corrected_mpc_pressure),
        .spc_target_current = spc_target_current,
        .spc_actual_current = flip_u16(spc_actual_current),
        .mpc_target_current = mpc_target_current,
        .mpc_actual_current = flip_u16(mpc_actual_current),
        .tcc_pwm_255 = 0,
    };

    uint16_t clamped_tcc = tcc_pwm_raw;
    if (clamped_tcc > 4095) {
        clamped_tcc = 4095;
    }
    ret.tcc_pwm_255 = (uint8_t)(clamped_tcc >> 4);

    bool _1245 = y3_pwm_raw > 10;
    bool _23 = y5_pwm_raw > 10;
    bool _34 = y4_pwm_raw > 10;

    if (_1245 && !_23 && !_34) {
        ret.shift_valve_state = 1;
    } else if (_23 && !_1245 && !_34) {
        ret.shift_valve_state = 2;
    } else if (_34 && !_1245 && !_23) {
        ret.shift_valve_state = 4;
    } else if (_1245 && _23 && !_34) {
        ret.shift_valve_state = 3;
    } else if (_1245 && _34 && !_23) {
        ret.shift_valve_state = 5;
    } else if (_23 && _34 && !_1245) {
        ret.shift_valve_state = 6;
    } else if (_1245 && _23 && _34) {
        ret.shift_valve_state = 7;
    } else {
        ret.shift_valve_state = 0;
    }

    ret.valve_flag = (_1245 || _23 || _34) ? 1 : 0;
    return ret;
}