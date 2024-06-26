#ifndef __MODULE_SETTINGS_H
#define __MODULE_SETTINGS_H

#include <stdint.h>
#include <tcu_maths.h>
#include <esp_err.h>

// TCC Settings

// Torque converter setting
typedef struct {
    // Enable adaptation for all gears
    bool adapt_enable;
    // Enable torque converter in D1
    bool enable_d1;
    // Enable torque converter in D2
    bool enable_d2;
    // Enable torque converter in D3
    bool enable_d3;
    // Enable torque converter in D4
    bool enable_d4;
    // Enable torque converter in D5
    bool enable_d5;
    // When adapting, this is the time between checks to see how
    // much additional or less pressure should be applied to the converter.
    // Making this interval too quick can result in over adapting!
    //
    // UNIT: milliseconds
    uint16_t adapt_test_interval_ms;
    // The minimum output shaft speed for Sailing mode to occur.
    // In sailing mode, when the accelerator input is 0%, the torque
    // converter will fully unlock, in order to acheive the maximum
    // possible coasting distance, for better fuel economy. Set this to 0
    // to disable sailing mode. Note. This feature is inspired by the 9G tronic
    //
    // UNIT: RPM
    uint16_t sailing_mode_active_rpm;
    // The minimum output shaft speed before the torque converter is forced
    // to lockup regardless of pedal input. This is done to avoid slipping
    // at high RPM, which causes massive heat buildup.
    //
    // UNIT: RPM
    uint16_t force_lock_min_output_rpm;
    // Open the converter to slipping (If locked) if the engine requests it
    // This is used on the M113K platform when the supercharger clutch
    // is about to engage, so that the shock of the supercharger coming on does
    // not cause too much discomfort
    bool react_on_engine_slip_request;
    // Open the converter fully, if the engine requests it.
    // This is usually used under very heavy load under low RPM
    bool react_on_engine_open_request;
} __attribute__ ((packed)) TCC_MODULE_SETTINGS;

const TCC_MODULE_SETTINGS TCC_DEFAULT_SETTINGS = {
    .adapt_enable = true,
    .enable_d1 = false,
    .enable_d2 = true,
    .enable_d3 = true,
    .enable_d4 = true,
    .enable_d5 = true,
    .adapt_test_interval_ms = 1000,
    .sailing_mode_active_rpm = 500,
    .force_lock_min_output_rpm = 2000,
    .react_on_engine_slip_request = true,
    .react_on_engine_open_request = true
};

// Solenoid subsystem settings
typedef struct {
    // Minimum battery voltage before performing 
    // the solenoid boot up test on TCU start
    //
    // UNIT: mV
    uint16_t min_batt_power_on_test;
    // If a solenoid is reading more than this current during the
    // boot test, then it is assumed faulty
    //
    // UNIT: mA
    uint16_t current_threshold_error;
    // Solenoid reference voltage. DO NOT TOUCH THIS. It is intended 
    // for debugging ONLY!
    //
    // UNIT: mV
    uint16_t cc_vref_solenoid;
    // The temperature coefficient of the solenoid wiring and coils.
    // DO NOT TOUCH THIS. It is intended for debugging ONLY!
    float cc_temp_coefficient_wires;
    // MPC and SPC solenoids reference resistance at cc_reference_temp
    float cc_reference_resistance;
    // MPC and SPC solenoids resistance reference temperature
    float cc_reference_temp;
    
} __attribute__ ((packed)) SOL_MODULE_SETTINGS;

const SOL_MODULE_SETTINGS SOL_DEFAULT_SETTINGS = {
    .min_batt_power_on_test = 11000,
    .current_threshold_error = 500,
    .cc_vref_solenoid = 12000,
    .cc_temp_coefficient_wires = 0.393,
    .cc_reference_resistance = 5.3,
    .cc_reference_temp = 25,
    //.cc_max_adjust_per_step = 2,
};

// Shift program basic settings
typedef struct {
    // Minimum end RPM for an upshift. Setting this too high
    // might block shifting
    // UNIT: RPM
    uint16_t min_upshift_end_rpm;
    // DEBUG - Show an 'F' marker in the gear display when the TCU
    // detects a flare condition
    bool f_shown_if_flare;
    // DEBUG - Show '^' or 'v' in the gear display when the shift
    // thread is active
    bool debug_show_up_down_arrows_in_r;
    // Torque factor reduction based on input torque
    LinearInterpSetting torque_reduction_factor_input_torque;
    // Torque factor reduction based on shift speed
    LinearInterpSetting torque_reduction_factor_shift_speed;
    // When not moving, this is the time the gearbox holds the overlap
    // phase and assumes the shift completes successfully.
    // UNIT: milliseconds
    uint16_t stationary_shift_hold_time;
    // Shift timeout when pulling
    // UNIT: milliseconds
    uint16_t shift_timeout_pulling;
    // Shift timeout when coasting
    // UNIT: milliseconds
    uint16_t shift_timeout_coasting;
    // When garage shifting, this is the maximum time to wait
    // for engine to drop its RPM when we ask it to before performing
    // the garage shift
    // UNIT: milliseconds
    uint16_t garage_shift_max_timeout_engine;
    // Enable torque request for the 1-2 upshift
    bool trq_req_1_2_enable;
    // Enable torque request for the 2-3 upshift
    bool trq_req_2_3_enable;
    // Enable torque request for the 3-4 upshift
    bool trq_req_3_4_enable;
    // Enable torque request for the 4-5 upshift
    bool trq_req_4_5_enable;
    // Enable torque request for the 5-4 downshift
    bool trq_req_5_4_enable;
    // Enable torque request for the 4-3 downshift
    bool trq_req_4_3_enable;
    // Enable torque request for the 3-2 downshift
    bool trq_req_3_2_enable;
    // Enable torque request for the 2-1 downshift
    bool trq_req_2_1_enable;
} __attribute__ ((packed)) SBS_MODULE_SETTINGS;

const SBS_MODULE_SETTINGS SBS_DEFAULT_SETTINGS = {
    .min_upshift_end_rpm = 1000,
    .f_shown_if_flare = false,
    .debug_show_up_down_arrows_in_r = false,
    .torque_reduction_factor_input_torque = {
        .new_min = 0.3,
        .new_max = 0.2,
        .raw_min = 100,
        .raw_max = 400,
    },
    .torque_reduction_factor_shift_speed = {
        .new_min = 1.3,
        .new_max = 1.0,
        .raw_min = 100,
        .raw_max = 1000,
    },
    .stationary_shift_hold_time = 1000,
    .shift_timeout_pulling = 3000,
    .shift_timeout_coasting = 5000,
    .garage_shift_max_timeout_engine = 1000,
    .trq_req_1_2_enable = true,
    .trq_req_2_3_enable = true,
    .trq_req_3_4_enable = true,
    .trq_req_4_5_enable = true,
    .trq_req_5_4_enable = true,
    .trq_req_4_3_enable = true,
    .trq_req_3_2_enable = true,
    .trq_req_2_1_enable = true,
};

// Pressure manager settings
typedef struct {
    // Time before shift solenoids are reduced PWM.
    // Setting this too low can result in the shift circuit
    // not activating!
    //
    // UNIT: milliseconds
    uint16_t shift_solenoid_pwm_reduction_time;
} __attribute__ ((packed)) PRM_MODULE_SETTINGS;



const PRM_MODULE_SETTINGS PRM_DEFAULT_SETTINGS = {
    .shift_solenoid_pwm_reduction_time = 1000,
};

// Adaptation settings
typedef struct {
    // Minimum transmission oil temperature for adaptation
    //
    // UNIT: degrees C
    int16_t min_atf_temp;
    // Maximum transmission oil temperature for adaptation
    //
    // UNIT: degrees C
    int16_t max_atf_temp;
    // Minimum input speed for adaptation
    //
    // UNIT: RPM
    uint16_t min_input_rpm;
    // Maximum input speed for adaptation
    //
    // UNIT: RPM
    uint16_t max_input_rpm;
    // Adapt allowed for the K1 clutch
    bool prefill_adapt_k1;
    // Adapt allowed for the K2 clutch
    bool prefill_adapt_k2;
    // Adapt allowed for the K3 clutch
    bool prefill_adapt_k3;
    // Adapt allowed for the B1 brake
    bool prefill_adapt_b1;
    // Adapt allowed for the B1 brake
    bool prefill_adapt_b2;
    // The max pressure delta (+/-) allowed for any adaptation cell
    //
    // UNIT: mBar
    uint16_t prefill_max_pressure_delta;
    // The max time delta (+/-) allowed for any adaptation cell
    //
    // UNIT: milliseconds
    uint16_t prefill_max_time_delta;
    
} __attribute__ ((packed)) ADP_MODULE_SETTINGS;

const ADP_MODULE_SETTINGS ADP_DEFAULT_SETTINGS = {
    .min_atf_temp = 60,
    .max_atf_temp = 110,
    .min_input_rpm = 1000,
    .max_input_rpm = 3000,
    .prefill_adapt_k1 = true,
    .prefill_adapt_k2 = true,
    .prefill_adapt_k3 = true,
    .prefill_adapt_b1 = true,
    .prefill_adapt_b2 = true,
    .prefill_max_pressure_delta = 200,
    .prefill_max_time_delta = 200,
};

enum EwmSelectorType: uint8_t {
    None = 0,
    Button = 1,
    Switch = 2
};

enum SelectableGearboxProfile : uint8_t {
    // Standard mode
    Standard = 0,
    // Comfort mode
    Comfort = 1,
    // Agility mode
    Agility = 2,
    /// Manual mode - REQUIRES TIPTRONIC OR PADDLES TO FUNCTION
    Manual = 3,
    /// Manual mode - REQUIRES TIPTRONIC OR PADDLES TO FUNCTION
    Race = 4
};

// Shifter settings
typedef struct {
    // TRRS shifter (Wired to the TCU) has a profile selector?
    bool trrs_has_profile_selector;
    // The type of profile selection available on the CAN EWM
    // shifter
    EwmSelectorType ewm_selector_type;
    // When using a switch profile selector. This is the profile
    // to use when in the top position
    SelectableGearboxProfile switch_profile_idx_top;
    // When using a switch profile selector. This is the profile
    // to use when in the bottom position
    SelectableGearboxProfile switch_profile_idx_bottom;
    // When using the SLR profile selector, this is the profile
    // when the profile selector is in the left position
    SelectableGearboxProfile slr_profile_idx_left;
    // When using the SLR profile selector, this is the profile
    // when the profile selector is in the center position
    SelectableGearboxProfile slr_profile_idx_center;
    // When using the SLR profile selector, this is the profile
    // when the profile selector is in the right position
    SelectableGearboxProfile slr_profile_idx_right;
} __attribute__ ((packed)) ETS_MODULE_SETTINGS;

const ETS_MODULE_SETTINGS ETS_DEFAULT_SETTINGS = {
    .trrs_has_profile_selector = true,
    .ewm_selector_type = EwmSelectorType::Button,
    .switch_profile_idx_top = SelectableGearboxProfile::Comfort,
    .switch_profile_idx_bottom = SelectableGearboxProfile::Standard,
    .slr_profile_idx_left = SelectableGearboxProfile::Comfort,
    .slr_profile_idx_center = SelectableGearboxProfile::Standard,
    .slr_profile_idx_right = SelectableGearboxProfile::Manual
};

// module settings
extern TCC_MODULE_SETTINGS TCC_CURRENT_SETTINGS;
extern SOL_MODULE_SETTINGS SOL_CURRENT_SETTINGS;
extern SBS_MODULE_SETTINGS SBS_CURRENT_SETTINGS;
extern PRM_MODULE_SETTINGS PRM_CURRENT_SETTINGS;
extern ADP_MODULE_SETTINGS ADP_CURRENT_SETTINGS;
extern ETS_MODULE_SETTINGS ETS_CURRENT_SETTINGS;

// Setting IDx
#define TCC_MODULE_SETTINGS_SCN_ID 0x01
#define SOL_MODULE_SETTINGS_SCN_ID 0x02
#define SBS_MODULE_SETTINGS_SCN_ID 0x03
#define PRM_MODULE_SETTINGS_SCN_ID 0x05
#define ADP_MODULE_SETTINGS_SCN_ID 0x06
#define ETS_MODULE_SETTINGS_SCN_ID 0x07

namespace ModuleConfiguration {
    esp_err_t load_all_settings();
    esp_err_t reset_settings(uint8_t idx);
    esp_err_t write_settings(uint8_t module_id, uint16_t buffer_len, uint8_t* buffer);
    esp_err_t read_settings(uint8_t module_id, uint16_t* buffer_len, uint8_t** buffer);
}

#endif