#ifndef COMMON_STRUCTS_H__
#define COMMON_STRUCTS_H__

#include <stdint.h>
#include "solenoids/solenoids.h"

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
    TWO_ONE, // 2-1
};

typedef struct {
    uint16_t initial_spc_pwm;
    uint16_t mpc_pwm;
    uint16_t targ_ms;
    // Valid range = 1 - 10 (Auto clamped if value is outside this range) - Higher = firmer shift
    float shift_firmness;
    // Valid range = 1 - 10 (Auto clamped if value is outside this range) - Higher = faster shift
    float shift_speed;
    Solenoid* shift_solenoid;
    uint8_t targ_g;
    uint8_t curr_g;
} ShiftData;

#pragma GCC diagnostic ignored "-Wmissing-field-initializers" // This is ALWAYS correctly initialized in pressure_manager.cpp
const ShiftData DEFAULT_SHIFT_DATA = { .initial_spc_pwm = 100, .mpc_pwm = 100, .targ_ms = 500, .shift_firmness = 1.0, .shift_speed = 5.0};

typedef struct {
    bool shifted; // Did the car change gears or not??
    bool aborted; // Was an abort shift made during the change?
    bool valid_measurement; // Valid measurement sample complete
    int time_ms; // Time taken to shift
    int avg_d_rpm; // Average delta RPM
    int max_d_rpm; // Max delta RPM
    int min_d_rpm; // Min delta RPM
} ShiftResponse;

typedef struct {
    uint16_t target_shift_time_ms;
    // Valid range = 1 - 10 (Auto clamped if value is outside this range) - Higher = firmer shift
    float shift_firmness;
    // Valid range =  1 - 10 (Auto clamped if value is outside this range) - Higher = faster shift
    float shift_speed;
} ShiftCharacteristics;

#endif