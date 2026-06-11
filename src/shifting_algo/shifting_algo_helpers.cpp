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

float ShiftHelpers::get_shift_intertia(uint8_t shift_idx) {
    float r = (float)(MECH_PTR->intertia_torque[shift_idx]) + (float)(VEHICLE_CONFIG.engine_drag_torque/10);
    return r;
}

void ShiftHelpers::calc_shift_flags(ShiftInterfaceData* sid, SensorData* sd, bool bleed_phase) {
    // Set to 0 at the start of the shift, so we just keep OR'ing it
    if ((uint8_t)sid->targ_g < (uint8_t)sid->curr_g) {
        // Downshift detected
        if ((sid->shift_flags & SHIFT_FLAG_COAST_54_43) != 0) {
            // Check if pedal has jumped, and only then clear the 54_43 coast flag
            if (sd->pedal_pos > 30) {
                sid->shift_flags &= ~(SHIFT_FLAG_COAST | SHIFT_FLAG_COAST_54_43);
            }
        }
        if (sd->pedal_pos < 15 && ((sid->shift_flags & SHIFT_FLAG_COAST) == 0) && (((sid->shift_flags & SHIFT_FLAG_COAST_54_43) == 0) && bleed_phase)) {
            if (sid->change == GearChange::_5_4 || sid->change == GearChange::_4_3) {
                sid->shift_flags &= ~SHIFT_FLAG_COAST;
                sid->shift_flags |= SHIFT_FLAG_COAST_54_43;
            } else {
                sid->shift_flags |= SHIFT_FLAG_COAST;
                sid->shift_flags &= ~SHIFT_FLAG_COAST_54_43;
            }
        }
    }
    // 3-2/2-1 is always evaluated
    if ((sid->shift_flags & SHIFT_FLAG_COAST) != 0 && (sid->change == GearChange::_2_1 || sid->change == GearChange::_3_2)) {
        sid->shift_flags |= SHIFT_FLAG_COAST_32_21;
    }

    if (sd->input_rpm < 400) {
        sid->shift_flags |= SHIFT_FLAG_STATIONARY;
    }
}