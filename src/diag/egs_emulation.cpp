#include "egs_emulation.h"
#include "tcu_io/tcu_io.hpp"
#include "../pressure_manager.h"
#include "clock.hpp"

inline uint16_t flip_uint16_t(uint16_t x) {
   return ((x & 0xff) << 8) | ((x & 0xff00) >> 8); 
}

RLI_30_DATA get_rli_30(EgsBaseCan* can_layer) {
    RLI_30_DATA ret = {};
    memset(&ret, 0x00, sizeof(RLI_30_DATA));
    return ret;
}

RLI_31_DATA get_rli_31(EgsBaseCan* can_layer) {
    RLI_31_DATA ret = {};
    uint16_t n2 = TCUIO::n2_rpm();
    uint16_t n3 = TCUIO::n2_rpm();
    ret.n2_pulse_count = flip_uint16_t(n2);
    ret.n3_pulse_count = flip_uint16_t(n3);
    //ret.input_rpm = flip_uint16_t(turbine.value);

    uint16_t wd = can_layer->get_front_left_wheel(300);
    if (UINT16_MAX != wd) {
        ret.front_left_wheel_speed = 0xFFFF;
    } else {
        ret.front_left_wheel_speed = flip_uint16_t(wd / 2.0);
    }

    wd = can_layer->get_front_right_wheel(300);
    if (UINT16_MAX != wd) {
        ret.front_right_wheel_speed = 0xFFFF;
    } else {
        ret.front_right_wheel_speed = flip_uint16_t(wd / 2.0);
    }

    wd = can_layer->get_rear_left_wheel(300);
    if (UINT16_MAX != wd) {
        ret.rear_left_wheel_speed = 0xFFFF;
    } else {
        ret.rear_left_wheel_speed = flip_uint16_t(wd / 2.0);
    }

    wd = can_layer->get_rear_right_wheel(300);
    if (UINT16_MAX != wd) {
        ret.rear_right_wheel_speed = 0xFFFF;
    } else {
        ret.rear_right_wheel_speed = flip_uint16_t(wd / 2.0);
    }

    ret.engine_speed = flip_uint16_t(can_layer->get_engine_rpm(300));

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

    ret.mpc_pressure = flip_uint16_t(pressure_manager->get_corrected_spc_pressure());
    ret.spc_pressure = flip_uint16_t(pressure_manager->get_corrected_modulating_pressure());
    ret.mpc_target_current = sol_mpc->get_current_target();
    ret.spc_target_current = sol_spc->get_current_target();
    ret.mpc_actual_current = flip_uint16_t(sol_mpc->get_current());
    ret.spc_actual_current = flip_uint16_t(sol_spc->get_current());
    ret.tcc_pwm_255 = (uint8_t)(sol_tcc->get_pwm_raw() >> 4);

    bool _1245 = sol_y3->get_pwm_raw() > 10;
    bool _23 = sol_y5->get_pwm_raw() > 10;
    bool _34 = sol_y4->get_pwm_raw() > 10;

    if (_1245 && !_23 && !_34) {
        ret.shift_valve_state = ShiftValveStatus::_1245;
    } else if (_23 && !_1245 && !_34) {
        ret.shift_valve_state = ShiftValveStatus::_23;
    } else if (_34 && !_1245 && !_23) {
        ret.shift_valve_state = ShiftValveStatus::_34;
    } else if (_1245 && _23 && !_34) {
        ret.shift_valve_state = ShiftValveStatus::_1245_and_23;
    } else if (_1245 && _34 && !_23) {
        ret.shift_valve_state = ShiftValveStatus::_1245_and_34;
    } else if (_23 && _34 && !_1245) {
        ret.shift_valve_state = ShiftValveStatus::_23_and_34;
    } else if (_1245 && _23 && _34) {
        ret.shift_valve_state = ShiftValveStatus::_1245_and_23_and_34;
    } else {
        ret.shift_valve_state = ShiftValveStatus::No;
    }
    ret.valve_flag = _1245 || _23 || _34;
    return ret;
}