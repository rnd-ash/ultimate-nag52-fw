#include "s_algo.h"
#include "egs_calibration/calibration_structs.h"
#include "nvs/eeprom_config.h"

float ShiftHelpers::calcualte_abs_engine_inertia(uint8_t shift_idx, uint16_t engine_rpm, uint16_t input_rpm) {
    float min_factor = 1.0 / ((float)(MECH_PTR->intertia_factor[shift_idx])/1000.0);
    float turbine_factor = (float)input_rpm / (float)engine_rpm;
    float engine_inertia = (float)(VEHICLE_CONFIG.engine_drag_torque)/10.0;
    float pump_inertia = MECH_PTR->intertia_torque[shift_idx];
    float ret = interpolate_float(turbine_factor, pump_inertia, engine_inertia, min_factor, 1, InterpType::Linear);
    return abs(ret);
}

uint16_t ShiftHelpers::ms_till_target_on_rpm(int target, int d_on_clutch, int rpm_on_clutch) {
    uint16_t ret = 0;
    if (rpm_on_clutch > target && d_on_clutch < 0) {
        // Assume 20ms EGS cycles
        int rpm_left = rpm_on_clutch - target;
        float r = (float)rpm_left / (float)abs(d_on_clutch);
        ret = r * 20.0;
    } else if (d_on_clutch >= 0) {
        ret = UINT16_MAX;
    }
    return ret;
}

uint16_t ShiftHelpers::calc_output_mod_pressure(uint8_t shift_idx, uint16_t p_shift, uint16_t p_mod, uint16_t hydr_max) {
    float shift = HYDR_PTR->overlap_circuit_factor_spc[shift_idx] * p_shift;
    shift /= 1000.0;

    float mod = HYDR_PTR->overlap_circuit_factor_mpc[shift_idx] * p_mod;
    mod /= 1000.0;

    float ret = shift + mod + HYDR_PTR->overlap_circuit_spring_pressure[shift_idx];

    if (ret > hydr_max) {
        ret = hydr_max;
    }
    return ret;
}

// EGS func 0xd7272
float ShiftHelpers::get_shift_intertia(uint8_t shift_idx) {
    float r = (float)(MECH_PTR->intertia_torque[shift_idx]) + (float)(VEHICLE_CONFIG.engine_drag_torque/10);
    return r;
}


// RELEASE_CAL->field_0x5a
const float factors[8] = {0.9, 0.9, 0.85, 0.7, 1.0, 1.0, 1.0, 1.0};

uint16_t ShiftHelpers::get_threshold_rpm(uint32_t shift_flags, uint8_t shift_idx, uint16_t abs_trq, uint8_t ramp_cycles) {
    // EGS func 0xd728CC
    /*
    int bvar1 = 6;
    float inertia = ShiftHelpers::get_shift_intertia(shift_idx);
    float res = ((abs_trq*5) * (ramp_cycles+(2*bvar1))) / inertia;
    res *= factors[shift_idx];
    res *= (MECH_PTR->intertia_factor[shift_idx]/1000.0);
    if (res < 130) {
        res = 130;
    }
    return (uint16_t)res;
    */

    // EGS func 0xd732C
    if ((shift_flags & SHIFT_FLAG_COAST) != 0) {
        return 25;
    } else {
        int bvar1 = 3;
        float inertia = ShiftHelpers::get_shift_intertia(shift_idx);
        float res = ((abs_trq*10) * (ramp_cycles+(2*bvar1))) / inertia;
        res *= factors[shift_idx];
        res *= (MECH_PTR->turbine_drag[shift_idx]/10.0);
        if (res < 130) {
            res = 130;
        }
        return (uint16_t)res;
    }
}