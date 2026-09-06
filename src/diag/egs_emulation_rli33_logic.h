#ifndef EGS_EMULATION_RLI33_LOGIC_H
#define EGS_EMULATION_RLI33_LOGIC_H

#include <stdint.h>

struct Egs51Rli33DerivedData {
    uint8_t valve_flag;
    uint8_t shift_valve_state;
    uint16_t spc_pressure;
    uint16_t mpc_pressure;
    uint16_t spc_target_current;
    uint16_t spc_actual_current;
    uint16_t mpc_target_current;
    uint16_t mpc_actual_current;
    uint8_t tcc_pwm_255;
};

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
);

#endif