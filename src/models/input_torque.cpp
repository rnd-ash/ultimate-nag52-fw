#include "input_torque.hpp"
#include "tcu_maths.h"

InputTorqueModel::InputTorqueModel() {

}


void InputTorqueModel::update(EgsBaseCan* can_hal, SensorData* measures, bool is_fwd_gear) {
    if (measures->static_torque == INT16_MAX) {
        measures->input_torque = INT16_MAX;
    } else {
        int motor_torque = measures->static_torque;
        int16_t input_torque = motor_torque;
        uint8_t ac_loss = can_hal->get_ac_torque_loss(measures->current_timestamp_ms, 500);
        if (ac_loss != UINT8_MAX) {
            motor_torque -= ac_loss;
        }
        if (is_fwd_gear) {
            // Engine RPM = static torque
            // So torque ratio = fraction of static torque
            // REMEMBER: Tcc multiplies torque when it is lower than engine RPM
#define MAX_MULTIPLICATION 2 // 250%
            egs_can_hal->set_turbine_torque_loss(0); // For now

            // engine RPM / 2.5 - 2.5x
            // engine RPM - 1.0
            float multi = scale_number(measures->input_rpm, MAX_MULTIPLICATION, 1.0, (float)measures->engine_rpm/2, (float)measures->engine_rpm);
            //ESP_LOGI("_","MULTI %.2f - %d %d", multi, motor_torque, (int)(motor_torque * multi));
            input_torque = motor_torque * multi;


            /*
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
            */
        }
        measures->input_torque = MAX(input_torque, motor_torque);
    }
    
    // Set the measured input torque
}