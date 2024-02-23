#ifndef __INPUT_TORQUE_H_
#define __INPUT_TORQUE_H_

#include <stdint.h>
#include <canbus/can_hal.h>
#include <common_structs.h>

namespace InputTorqueModel {
    int16_t get_input_torque(EgsBaseCan* can_hal, SensorData* measures);
};

#endif