#ifndef COMMON_STRUCTS_H__
#define COMMON_STRUCTS_H__

#include <stdint.h>

typedef struct {
    int input_rpm;
    int engine_rpm;
    int output_rpm;
    uint16_t voltage;
    uint8_t pedal_pos;
    int atf_temp;
    int static_torque;
    int max_torque;
    int min_torque;
} SensorData;


enum class ProfileGearChange {
    ONE_TWO, // 1-2
    TWO_THREE, // 2-3
    THREE_FOUR, // 3-4
    FOUR_FIVE, // 4-5
    FIVE_FOUR, // 5-4
    FOUR_THREE, // 4-3
    THREE_TWO, // 3-2
    TWO_ONE // 2-1
};

typedef struct {
    uint16_t spc_perc;
    uint16_t mpc_perc;
    uint16_t targ_ms;
} ShiftData;

#endif