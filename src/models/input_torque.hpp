#ifndef INPUT_TORQUE_H
#define INPUT_TORQUE_H

#include <stdint.h>
#include <canbus/can_hal.h>
#include <common_structs.h>

namespace InputTorqueModel {
    int16_t get_input_torque(uint16_t engine_rpm, uint16_t input_rpm, int16_t static_torque);
    int16_t get_pump_torque(uint16_t engine_rpm, uint16_t input_rpm);
    float get_input_torque_factor(uint16_t engine, uint16_t input);
};

#endif