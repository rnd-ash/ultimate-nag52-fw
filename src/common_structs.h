#ifndef COMMON_STRUCTS_H__
#define COMMON_STRUCTS_H__

#include <stdint.h>

typedef int16_t pressure_map[11];

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
    int tcc_slip_rpm;
    uint64_t last_shift_time; // In ms
    uint64_t current_timestamp_ms;
    bool is_braking;
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
    uint16_t spc_pwm;
    uint16_t mpc_pwm;
    uint16_t targ_ms;
    float shift_firmness;
} ShiftData;

const ShiftData DEFAULT_SHIFT_DATA = { .spc_pwm = 100, .mpc_pwm = 100, .targ_ms = 500, .shift_firmness = 1.0};

typedef struct {
    bool shifted; // Did the car change gears or not??
    bool valid_measurement; // Valid measurement sample complete
    int time_ms; // Time taken to shift
    int avg_d_rpm; // Average delta RPM
    int max_d_rpm; // Max delta RPM
    int min_d_rpm; // Min delta RPM
} ShiftResponse;

#endif