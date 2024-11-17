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

TorqueRequestModel ShiftHelpers::trq_req_init_model(uint16_t ramp_down_ms, uint16_t ramp_up_ms) {
    TorqueRequestModel mdl;
    memset(&mdl, 0x00, sizeof(TorqueRequestModel));
    mdl.ramp_up_ms = ramp_up_ms;
    mdl.ramp_down_ms = ramp_down_ms;
    return mdl;
}