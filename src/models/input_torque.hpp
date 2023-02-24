#ifndef __INPUT_TORQUE_H_
#define __INPUT_TORQUE_H_

#include <stdint.h>
#include <canbus/can_hal.h>
#include <common_structs.h>

// Linear interp values for loading of the torque converter. Input shaft
#define INPUT_MULTIPLIER_STOPPED 2.0 // At 0RPM, the input torque will be 2x the engines output torque
#define MULTI_MULTIPLIER_END_RPM 1000 // At 1000RPM+, the input torque will be 1x the engines output torque

class InputTorqueModel {
public:
    InputTorqueModel();
    void update(EgsBaseCan* can_hal, SensorData* measures, bool is_fwd_gear);
private:
    float last_tcc_loss = 0;
};

#endif