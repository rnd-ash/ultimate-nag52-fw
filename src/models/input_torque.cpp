#include "input_torque.hpp"
#include "tcu_maths.h"

// 290R TCC (E55 M113K) (Test)
// 0    919
// 175  100
// 1.75x at 0, 1.00x at 0.919

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
        float rpm_multi = ((float)measures->input_rpm) / ((float)measures->engine_rpm);
        float multi = interpolate_float(rpm_multi, 1.75, 1.00, 0, 0.919, InterpType::Linear);
        ret = motor_torque * multi;
    }
    return ret;
}