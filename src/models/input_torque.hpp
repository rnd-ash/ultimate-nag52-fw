#ifndef __INPUT_TORQUE_H_
#define __INPUT_TORQUE_H_

#include <stdint.h>
#include <canbus/can_hal.h>
#include <common_structs.h>

namespace InputTorqueModel {
    int16_t get_input_torque(uint16_t engine_rpm, uint16_t input_rpm, int16_t static_torque, uint8_t ac_torque);
    float get_input_torque_factor(uint16_t engine, uint16_t input);
};

#endif