#include "s_algo.h"
#include "egs_calibration/calibration_structs.h"
#include "nvs/eeprom_config.h"
#include "tcu_scaling.h"
#include <math.h>

// The mechanical calibration arrays are 8 entries wide (one per forward shift).
// shift_idx normally comes from fwd_gearchange_egs_map_lookup_idx(), which is
// bounded 0-7, but these are public helpers so the bound is enforced here too.
#define MECH_SHIFT_IDX_COUNT (8u)

float ShiftHelpers::calcualte_abs_engine_inertia(uint8_t shift_idx, uint16_t engine_rpm, uint16_t input_rpm) {
    if (nullptr == MECH_PTR || shift_idx >= MECH_SHIFT_IDX_COUNT) {
        return 0.0f;
    }
    // intertia_factor is calibration data, so it cannot be assumed non-zero.
    const float inertia_factor = (float)(MECH_PTR->intertia_factor[shift_idx]) / 1000.0f;
    if (!(inertia_factor > 0.0f)) { // Also catches NaN
        return 0.0f;
    }
    float min_factor = 1.0f / inertia_factor;
    // A stalled or unreported engine gives engine_rpm == 0. Dividing by it
    // yields inf, which then poisons the interpolation below.
    float turbine_factor = (0u != engine_rpm) ? ((float)input_rpm / (float)engine_rpm) : 0.0f;
    float engine_inertia = (float)(VEHICLE_CONFIG.engine_drag_torque) / 10.0f;
    float pump_inertia = MECH_PTR->intertia_torque[shift_idx];
    float ret = interpolate_float(turbine_factor, pump_inertia, engine_inertia, min_factor, 1.0f, InterpType::Linear);
    return fabsf(ret);
}

float ShiftHelpers::get_shift_intertia(uint8_t shift_idx) {
    if (nullptr == MECH_PTR || shift_idx >= MECH_SHIFT_IDX_COUNT) {
        return 0.0f;
    }
    // NOTE: engine_drag_torque is scaled by 10 and must be divided as a float.
    // This previously used integer division (engine_drag_torque/10), which
    // silently truncated - and disagreed with calcualte_abs_engine_inertia
    // above, which does the same conversion correctly.
    float r = (float)(MECH_PTR->intertia_torque[shift_idx]) + ((float)VEHICLE_CONFIG.engine_drag_torque / 10.0f);
    return r;
}

void ShiftHelpers::calc_shift_flags(ShiftInterfaceData* sid, SensorData* sd) {
    sid->shift_flags = 0;
    if (sd->pedal_pos < Pedal::percent(6.0f)) { // 15 raw. NOTE: the old comment here claimed ~10%, which is wrong - 15/250 is 6%
        sid->shift_flags |= SHIFT_FLAG_COAST;
        if (sid->change == GearChange::_5_4 || sid->change == GearChange::_4_3) {
            sid->shift_flags &= ~SHIFT_FLAG_COAST;
            sid->shift_flags |= SHIFT_FLAG_COAST_54_43;
        }
        if (sid->change == GearChange::_1_2 || sid->change == GearChange::_3_2) {
            sid->shift_flags |= SHIFT_FLAG_COAST_32_21;
        }
    }
    if (sd->input_rpm < 400) {
        sid->shift_flags |= SHIFT_FLAG_STATIONARY;
    }
}