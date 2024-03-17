#include "input_torque.hpp"
#include "tcu_maths.h"
#include "egs_calibration/calibration_structs.h"

int16_t InputTorqueModel::get_input_torque(EgsBaseCan* can_hal, SensorData* measures) {
    int16_t ret = 0;
    if (measures->static_torque == INT16_MAX) {
        ret = INT16_MAX;
    } else if (measures->engine_rpm == 0 || measures->engine_rpm == INT16_MAX) {
        ret = measures->static_torque;
    } else {
        int motor_torque = measures->static_torque;
        uint8_t ac_loss = can_hal->get_ac_torque_loss(500);
        if (ac_loss != UINT8_MAX) {
            motor_torque -= ac_loss;
        }
        float multi = InputTorqueModel::get_input_torque_factor(measures->engine_rpm, measures->input_rpm);
        ret = motor_torque * multi;
    }
    return ret;
}

float InputTorqueModel::get_input_torque_factor(uint16_t engine, uint16_t input) {
    float rpm_multi = ((float)input) / ((float)engine);

    //Interpolate map, but faster, since its just 4 values, no need to allocate a whole map for this
    float x1 = (float)(TCC_CFG_PTR->multiplier_map_x[0]) / 1000.0;
    float x2 = (float)(TCC_CFG_PTR->multiplier_map_x[1]) / 1000.0;
    float z1 = (float)(TCC_CFG_PTR->multiplier_map_z[0]) / 100.0;
    float z2 = (float)(TCC_CFG_PTR->multiplier_map_z[1]) / 100.0;
    return interpolate_float(rpm_multi, z1, z2, x1, x2, InterpType::Linear);
}