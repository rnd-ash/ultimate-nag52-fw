#ifndef INPUT_TORQUE_LOGIC_H
#define INPUT_TORQUE_LOGIC_H

#include <stdint.h>

int32_t input_torque_engine_pow2_div1000(uint16_t engine_rpm);
uint16_t input_torque_ratio_x1000(uint16_t input_rpm, uint16_t engine_rpm);
int16_t input_torque_scaled_clamped(int16_t static_torque, float multi);

#endif