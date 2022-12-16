#ifndef EGS_EMULATION_H
#define EGS_EMULATION_H

/// Data structures for EGS52 emulation in DAS

#include <stdint.h>
#include "canbus/can_hal.h"

// #define RLI_30 0x30
// #define RLI_31 0x31 // 7
static const uint16_t RLI_31 = 0x31;
// #define RLI_35 0x35 // 16

enum class RecognisedGear : uint8_t {
    Inactive = 0,
    D1 = 1,
    D2 = 2,
    D3 = 3,
    D4 = 4,
    D5 = 5,
    R = 6,
    R2 = 7,
    WrongGear = 23,
    Calculating = 88,
    SNV = 255
};

enum class TccState : uint8_t {
    Open = 0,
    OpenSlipping = 1,
    SlippingOpen = 2,
    Slipping = 3,
    SlippingLocked = 4,
    LockedSlipping = 5,
    Locked = 6
};

enum class ShiftValveStatus : uint8_t {
    No = 0,
    _1245 = 1,
    _23 = 2,
    _1245_and_23 = 3,
    _34 = 4,
    _1245_and_34 = 5,
    _23_and_34 = 6,
    _1245_and_23_and_34 = 7
};

typedef struct {
    uint16_t tcc_delta_speed; // 16..31
    uint16_t tcc_speed; // 32-47
    uint16_t tcc_pressure; // 48-63
    TccState tcc_status: 8; // 64-71 
    uint8_t ewm_position; // 72-79
    uint8_t program;// 80-87
    RecognisedGear gear_recognised: 8; // 88-95
    uint8_t actual_gear: 4; // 96..99
    uint8_t target_gear: 4; // 100..103
    uint8_t atf_temp: 8; // 104..111
    uint16_t motor_torque: 16; // 112..127
    uint16_t converter_torque: 16; // 128..143
    uint16_t output_speed; // 144-159
    bool kickdown: 1; // 160
    bool alf: 1; // 161
    bool start_lockout_reason: 1; // 162
    bool rp_lock: 1; // 163
    bool starter_relay: 1; // 164
    bool solenoid_1245: 1; // 165
    bool solenoid_23: 1; // 166
    bool solenoid_34: 1; // 167
    bool ewm_prg_btn: 1; // 168
    bool ewm_plus: 1; // 169
    bool ewm_minus: 1; // 170
    bool not_switching_bit: 1; // 171
    bool gear_protection: 1; // 172
    bool tcc_open_request: 1; // 173
    uint8_t padding_2: 2; // 174..175
    bool downshift: 1; // 176
    bool upshift: 1; // 177
    bool dyn0_amr_egs: 1; // 178
    bool dyn1_amr_egs: 1; // 179
    bool release_circuit: 1; // 180
    bool wheel_plus_btn: 1; // 181
    bool wheel_minus_btn: 1; // 182
    bool converter_clutch_en: 1; // 183
    bool error_current: 1; // 184
    bool emergency_mode: 1; // 185
    bool asr_active: 1; // 186
    bool transmission_protection_ack: 1; // 187
    bool bang_start: 1; // 188
    bool min_trq_egs: 1; // 189
    bool circuit_break: 1; // 190
    bool double_circuit: 1; // 191
    uint16_t padding_9: 9; // 192..200
    bool converter_ok: 1; // 201


} __attribute__ ((packed)) RLI_30_DATA;

typedef struct {
    uint16_t n2_pulse_count;
    uint16_t n3_pulse_count;
    uint16_t input_rpm;
    uint16_t engine_speed;
    uint16_t front_left_wheel_speed;
    uint16_t front_right_wheel_speed;
    uint16_t rear_left_wheel_speed;
    uint16_t rear_right_wheel_speed;
    uint16_t vehicle_speed_rear_wheels; //km/h
    uint16_t vehicle_speed_front_wheels;
} __attribute__ ((packed)) RLI_31_DATA;

typedef struct {
    uint8_t pedal_percent; // 16..23
    uint16_t delta_rpm_upshift; // 24..39
    uint16_t delta_rpm_downshift; // 40..55
    uint8_t pedal_delta_percent; // 56..63
    uint16_t pitch; // 64..79
    uint8_t driving_staus_ident; // 80..87
    uint8_t switching_warming_engine; // 88..95
    uint8_t low_request_gear_range_restrict; // 96..103
    uint8_t max_request_gear_range_restrict; // 104..111

} __attribute__ ((packed)) RLI_32_DATA;

typedef struct {
    uint8_t valve_flag; // 16..23
    ShiftValveStatus shift_valve_state; //24..31
    uint16_t spc_pressure; // 32..47
    uint16_t mpc_pressure; // 48..63
    uint16_t spc_target_current; // 64..79
    uint16_t spc_actual_current; // 80..95
    uint16_t mpc_target_current; // 96..111
    uint16_t mpc_actual_current; // 112..127
    uint8_t tcc_pwm_255; // 128..135
} __attribute__ ((packed)) RLI_33_DATA;

RLI_33_DATA get_rli_33(EgsBaseCan* can_layer);
RLI_32_DATA get_rli_32(EgsBaseCan* can_layer);
RLI_31_DATA get_rli_31(EgsBaseCan* can_layer);
RLI_30_DATA get_rli_30(EgsBaseCan* can_layer);

#endif

