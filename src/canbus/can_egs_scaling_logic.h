#ifndef CAN_EGS_SCALING_LOGIC_H
#define CAN_EGS_SCALING_LOGIC_H

#include <stdint.h>

uint8_t egs52_encode_turbine_torque_loss(uint16_t loss_nm);
uint16_t egs52_encode_wheel_torque_multi_factor(float ratio);
uint16_t egs53_encode_wheel_torque_multi_factor(float ratio);

#endif