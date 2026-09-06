#ifndef EGS_EMULATION_LOGIC_H
#define EGS_EMULATION_LOGIC_H

#include <stdint.h>

struct Egs51Rli31DerivedData {
    uint16_t n2_pulse_count;
    uint16_t n3_pulse_count;
    uint16_t engine_speed;
    uint16_t front_left_wheel_speed;
    uint16_t front_right_wheel_speed;
    uint16_t rear_left_wheel_speed;
    uint16_t rear_right_wheel_speed;
};

uint16_t egs51_flip_uint16_t(uint16_t x);
Egs51Rli31DerivedData egs51_build_rli31_derived(uint16_t n2, uint16_t n3, uint16_t engine_rpm, uint16_t front_left_wheel, uint16_t front_right_wheel, uint16_t rear_left_wheel, uint16_t rear_right_wheel);

#endif // EGS_EMULATION_LOGIC_H
