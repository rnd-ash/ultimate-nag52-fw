#include "input_torque.hpp"
#include "tcu_maths.h"

InputTorqueModel::InputTorqueModel() {

}


void InputTorqueModel::update(EgsBaseCan* can_hal, SensorData* measures, bool is_fwd_gear) {
    if (measures->static_torque == INT16_MAX) {
        measures->input_torque = INT16_MAX;
    } else {
        float multi;
        int16_t input_torque = measures->static_torque;
        uint8_t ac_loss = can_hal->get_ac_torque_loss(measures->current_timestamp_ms, 500);
        if (ac_loss != UINT8_MAX) {
            input_torque -= ac_loss;
        }
        if (is_fwd_gear) {
            // Now calculate torque loss (Function of TCC slip)
            if (measures->input_rpm > 0 && measures->input_rpm > measures->engine_rpm) {
                if (measures->input_rpm - measures->engine_rpm > 25) {
                    multi = ((float)measures->input_rpm/(float)measures->engine_rpm);
                    this->last_tcc_loss = input_torque * multi;
                    input_torque -= this->last_tcc_loss;
                }
            }
            egs_can_hal->set_turbine_torque_loss(MIN(0, (float)this->last_tcc_loss));

            // Now calculate input torque multiplication
            if (measures->input_rpm < MULTI_MULTIPLIER_END_RPM && input_torque > 0) {
                float multi = scale_number(measures->input_rpm, INPUT_MULTIPLIER_STOPPED, 1.0, 0, MULTI_MULTIPLIER_END_RPM);
                input_torque *= multi;
            }
        }
        measures->input_torque = input_torque;
    }
    
    // Set the measured input torque
}