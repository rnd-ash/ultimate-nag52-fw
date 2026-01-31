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

void ShiftHelpers::calc_shift_flags(ShiftInterfaceData* sid, SensorData* sd) {
    sid->shift_flags = 0;
    if (sd->converted_torque < -ShiftHelpers::get_shift_intertia(sid->inf.map_idx)/2) {
        sid->shift_flags |= SHIFT_FLAG_COAST;
        if ((sid->targ_g < sid->curr_g) && (sid->targ_g == GearboxGear::Third || sid->targ_g == GearboxGear::Fourth)) {
            sid->shift_flags &= ~SHIFT_FLAG_COAST;
            sid->shift_flags |= SHIFT_FLAG_COAST_54_43;
        }
        if (sid->change == GearChange::_1_2 || sid->change == GearChange::_3_2) {
            sid->shift_flags |= SHIFT_FLAG_COAST_AND_FREEWHEELING;
        }
    }
}