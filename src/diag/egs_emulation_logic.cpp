#include "egs_emulation_logic.h"

#include <limits.h>

uint16_t egs51_flip_uint16_t(uint16_t x) {
    return ((x & 0xff) << 8) | ((x & 0xff00) >> 8);
}

static uint16_t encode_wheel_speed(uint16_t wheel_speed) {
    if (UINT16_MAX == wheel_speed) {
        return 0xFFFF;
    }
    return egs51_flip_uint16_t((uint16_t)(wheel_speed / 2));
}

Egs51Rli31DerivedData egs51_build_rli31_derived(uint16_t n2, uint16_t n3, uint16_t engine_rpm, uint16_t front_left_wheel, uint16_t front_right_wheel, uint16_t rear_left_wheel, uint16_t rear_right_wheel) {
    Egs51Rli31DerivedData derived = {};
    derived.n2_pulse_count = egs51_flip_uint16_t(n2);
    derived.n3_pulse_count = egs51_flip_uint16_t(n3);
    derived.engine_speed = egs51_flip_uint16_t(engine_rpm);
    derived.front_left_wheel_speed = encode_wheel_speed(front_left_wheel);
    derived.front_right_wheel_speed = encode_wheel_speed(front_right_wheel);
    derived.rear_left_wheel_speed = encode_wheel_speed(rear_left_wheel);
    derived.rear_right_wheel_speed = encode_wheel_speed(rear_right_wheel);
    return derived;
}
