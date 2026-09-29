#include "s_algo.h"
#include "egs_calibration/calibration_structs.h"
#include "nvs/eeprom_config.h"
#include "common_structs_ops.h"

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

uint8_t ShiftHelpers::cycles_crossover_overlap(GearChange change, uint16_t abs_input_torque, uint16_t input_rpm) {
    uint8_t interp_min = CRS_CURRENT_SETTINGS.overlap_cycles_low_trq;
    uint8_t interp_max = CRS_CURRENT_SETTINGS.overlap_cycles_high_trq;
    if (change == GearChange::_1_2) {
        interp_min += CRS_CURRENT_SETTINGS.overlap_cycles_low_trq_adder_1_2;
        interp_max += CRS_CURRENT_SETTINGS.overlap_cycles_high_trq_adder_1_2;
    }
    int min_trq = VEHICLE_CONFIG.engine_drag_torque/5.0; // 2x drag torque real
    int max_trq = VEHICLE_CONFIG.engine_drag_torque; // 10x drag torque real
    uint8_t ret = interpolate_float(abs_input_torque, interp_min,interp_max, min_trq, max_trq, InterpType::Linear);
    ret += interpolate_float(input_rpm, &CRS_CURRENT_SETTINGS.overlap_cycles_adder_rpm, InterpType::Linear);
    return ret;
}

uint8_t ShiftHelpers::cycles_crossover_overlap2(GearChange change, uint16_t abs_input_torque, uint16_t input_rpm) {
    uint8_t interp_min = CRS_CURRENT_SETTINGS.sync_cycles_low_trq;
    uint8_t interp_max = CRS_CURRENT_SETTINGS.sync_cycles_high_trq;
    if (change == GearChange::_1_2) {
        interp_min += CRS_CURRENT_SETTINGS.sync_cycles_low_trq_adder_1_2;
        interp_max += CRS_CURRENT_SETTINGS.sync_cycles_high_trq_adder_1_2;
    }
    int min_trq = VEHICLE_CONFIG.engine_drag_torque/5.0; // 2x drag torque real
    int max_trq = VEHICLE_CONFIG.engine_drag_torque; // 10x drag torque real
    uint8_t ret = interpolate_float(abs_input_torque, interp_min,interp_max, min_trq, max_trq, InterpType::Linear);
    ret += interpolate_float(input_rpm, &CRS_CURRENT_SETTINGS.sync_cycles_adder_rpm, InterpType::Linear);
    return ret;
}

uint16_t ShiftHelpers::total_time_crossover_shift(PressureManager* pm, GearChange change, uint16_t abs_input_torque, uint16_t input_rpm) {
    Clutch applying = get_clutch_to_apply(change);
    PrefillData info = pm->make_fill_data(applying);

    return 5 + // Bleed phase time
        info.fill_cycles + // High fill time
        3 + // Cycles to low filling P
        5 + // Cycles held at low filling P
    (uint16_t)ShiftHelpers::cycles_crossover_overlap(change, abs_input_torque, input_rpm) +
    (uint16_t)ShiftHelpers::cycles_crossover_overlap2(change, abs_input_torque, input_rpm);
}

uint16_t ShiftHelpers::correct_shift_shift_pressure(PressureManager* pm, int16_t pressure, uint8_t map_idx) {
    // TODO - Move max_p to global constant so it can be referred in other functions
    uint16_t max_p = pm->get_max_shift_pressure(map_idx);
    if (pressure <= 0) {
        pressure = 0;
    } else if (pressure >= max_p) {
        pressure = max_p;
    }
    // P*1000 as shift_spc_gain is *1000
    return (uint16_t)(((pressure * 1000) / HYDR_PTR->shift_spc_gain[map_idx]) + HYDR_PTR->shift_reg_spring_pressure);
}