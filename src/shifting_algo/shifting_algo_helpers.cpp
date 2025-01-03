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