#include "egs_emulation.h"
#include "egs_emulation_logic.h"
#include "egs_emulation_rli33_logic.h"
#include "tcu_io/tcu_io.hpp"
#include "../pressure_manager.h"
#include "clock.hpp"

inline uint16_t flip_uint16_t(uint16_t x) {
    return egs51_flip_uint16_t(x);
}

RLI_30_DATA get_rli_30(EgsBaseCan* can_layer) {
    RLI_30_DATA ret = {};
    memset(&ret, 0x00, sizeof(RLI_30_DATA));
    return ret;
}

RLI_31_DATA get_rli_31(EgsBaseCan* can_layer) {
    RLI_31_DATA ret = {};
    if (can_layer == nullptr) {
        memset(&ret, 0x00, sizeof(RLI_31_DATA));
        return ret;
    }
    uint16_t n2 = TCUIO::n2_rpm();
    uint16_t n3 = TCUIO::n3_rpm();
    Egs51Rli31DerivedData derived = egs51_build_rli31_derived(
        n2,
        n3,
        can_layer->get_engine_rpm(300),
        WheelSpeed::raw_2x_u16(can_layer->get_front_left_wheel(300)),
        WheelSpeed::raw_2x_u16(can_layer->get_front_right_wheel(300)),
        WheelSpeed::raw_2x_u16(can_layer->get_rear_left_wheel(300)),
        WheelSpeed::raw_2x_u16(can_layer->get_rear_right_wheel(300))
    );

    ret.n2_pulse_count = derived.n2_pulse_count;
    ret.n3_pulse_count = derived.n3_pulse_count;
    ret.engine_speed = derived.engine_speed;
    ret.front_left_wheel_speed = derived.front_left_wheel_speed;
    ret.front_right_wheel_speed = derived.front_right_wheel_speed;
    ret.rear_left_wheel_speed = derived.rear_left_wheel_speed;
    ret.rear_right_wheel_speed = derived.rear_right_wheel_speed;

    return ret;
}

RLI_32_DATA get_rli_32(EgsBaseCan* can_layer) {
    RLI_32_DATA ret = {};
    memset(&ret, 0x00, sizeof(RLI_32_DATA));
    return ret;
}

RLI_33_DATA get_rli_33(EgsBaseCan* can_layer) {
    RLI_33_DATA ret = {};
    memset(&ret, 0x00, sizeof(RLI_33_DATA));

    // This RLI is readable in the default diagnostic session, including on a TCU
    // that failed POST - where pressure_manager and the solenoids are all still
    // null. get_rli_31 above already guards its inputs; this one did not.
    if (nullptr == pressure_manager ||
        nullptr == sol_spc || nullptr == sol_mpc || nullptr == sol_tcc ||
        nullptr == sol_y3 || nullptr == sol_y4 || nullptr == sol_y5) {
        return ret;
    }

    Egs51Rli33DerivedData derived = egs51_build_rli33_derived(
        pressure_manager->get_corrected_spc_pressure(),
        pressure_manager->get_corrected_modulating_pressure(),
        sol_spc->get_current_target(),
        sol_spc->get_current(),
        sol_mpc->get_current_target(),
        sol_mpc->get_current(),
        sol_tcc->get_pwm_raw(),
        sol_y3->get_pwm_raw(),
        sol_y5->get_pwm_raw(),
        sol_y4->get_pwm_raw()
    );

    ret.valve_flag = derived.valve_flag;
    ret.shift_valve_state = (ShiftValveStatus)derived.shift_valve_state;
    ret.spc_pressure = derived.spc_pressure;
    ret.mpc_pressure = derived.mpc_pressure;
    ret.spc_target_current = derived.spc_target_current;
    ret.spc_actual_current = derived.spc_actual_current;
    ret.mpc_target_current = derived.mpc_target_current;
    ret.mpc_actual_current = derived.mpc_actual_current;
    ret.tcc_pwm_255 = derived.tcc_pwm_255;
    return ret;
}