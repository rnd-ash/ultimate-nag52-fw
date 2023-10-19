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
    // The pressure that is applied to the torque converter
    // simply to start closing the distance between the clutch pack,
    // without actually biting the clutch.
    //
    // UNIT: mBar
    uint16_t prefill_pressure;
    // The minimum input shaft speed before the torque converter can
    // being to slip or close
    //
    // UNIT: RPM
    uint16_t min_locking_rpm;
    // When adapting, this is the time between checks to see how
    // much additional or less pressure should be applied to the converter.
    // Making this interval too quick can result in over adapting!
    //
    // UNIT: milliseconds
    uint16_t adapt_test_interval_ms;
    // The stall speed of the torque converter. At this speed,
    // no matter the state of the clutch, the fluid in the converter
    // will trigger a soft lockup of the impellor and turbine.
    //
    // UNIT: RPM
    uint16_t tcc_stall_speed;
    // The minimum torque limit for adaptation
    //
    // UNIT: Nm
    uint16_t min_torque_adapt;
    // The maximum torque limit for adaptation
    //
    // UNIT: Nm
    uint16_t max_torque_adapt;
    // The minimum engine speed for prefill to start
    //
    // UNIT: RPM
    uint16_t prefill_min_engine_rpm;
    // Maximum slip allowed when at maximum adapting torque
    // UNIT: RPM
    uint16_t max_slip_max_adapt_trq;
    // Minimum slip allowed when at maximum adapting torque
    // UNIT: RPM
    uint16_t min_slip_max_adapt_trq;
    // Maximum slip allowed when at minimum adapting torque
    // UNIT: RPM
    uint16_t max_slip_min_adapt_trq;
    // Minimum slip allowed when at minimum adapting torque
    // UNIT: RPM
    uint16_t min_slip_min_adapt_trq;
    // If the adaptation algorithm detects it needs to increase or 
    // decrease pressure. This is how much of a change it will perform
    // in 1 adaptation cycle.
    //
    // UNIT: mBar
    uint8_t pressure_increase_step;

    uint8_t adapt_pressure_step;
    // Pressure multiplier based on output speed
    LinearInterpSetting pressure_multiplier_output_rpm;
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
    // The maximum pedal input when the output speed is below 
    // force_lock_min_output_rpm to trigger a lock of the torque converter
    //
    // UNIT: %
    uint8_t locking_pedal_pos_max;
} __attribute__ ((packed)) TCC_MODULE_SETTINGS;

const TCC_MODULE_SETTINGS TCC_DEFAULT_SETTINGS = {
    .adapt_enable = true,
    .enable_d1 = false,
    .enable_d2 = true,
    .enable_d3 = true,
    .enable_d4 = true,
    .enable_d5 = true,
    .prefill_pressure = 50,
    .min_locking_rpm = 1100,
    .adapt_test_interval_ms = 1000,
    .tcc_stall_speed = 2500,
    .min_torque_adapt = 30,
    .max_torque_adapt = 200,
    .prefill_min_engine_rpm = 1000,
    .max_slip_max_adapt_trq = 200,
    .min_slip_max_adapt_trq = 100,
    .max_slip_min_adapt_trq = 60,
    .min_slip_min_adapt_trq = 30,
    .pressure_increase_step = 10,
    .adapt_pressure_step = 10,
    .pressure_multiplier_output_rpm = {
        .new_min = 1.00,
        .new_max = 1.25,
        .raw_min = 1500,
        .raw_max = 2500,
    },
    .sailing_mode_active_rpm = 500,
    .force_lock_min_output_rpm = 2500,
    .locking_pedal_pos_max = 15
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
    // DEBUG
    float smooth_shifting_spc_multi_too_slow;
    // DEBUG
    float smooth_shifting_spc_multi_too_fast;
    // Maximum torque reduction this far into the shift when upshifting
    // UNIT: %
    uint16_t upshift_trq_max_reduction_at;
    // Maximum torque reduction this far into the shift when downshifting
    // UNIT: %
    uint16_t downshift_trq_max_reduction_at;
    // SPC ramp multiplier based on target shift speed
    LinearInterpSetting spc_multi_overlap_shift_speed;
    // SPC multiplier when at 0 gearbox torque
    float spc_multi_overlap_zero_trq;
    // SPC multiplier when at maximum gearbox torque
    float spc_multi_overlap_max_trq;
    // When garage shifting, this is the maximum time to wait
    // for engine to drop its RPM when we ask it to before performing
    // the garage shift
    // UNIT: milliseconds
    uint16_t garage_shift_max_timeout_engine;
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
    .smooth_shifting_spc_multi_too_slow = 2.0,
    .smooth_shifting_spc_multi_too_fast = 0.5,
    .upshift_trq_max_reduction_at = 50,
    .downshift_trq_max_reduction_at = 100,
    .spc_multi_overlap_shift_speed = {
        .new_min = 1.0,
        .new_max = 5.0,
        .raw_min = 500,
        .raw_max = 100,
    },
    .spc_multi_overlap_zero_trq = 2.0,
    .spc_multi_overlap_max_trq = 30.0,
    .garage_shift_max_timeout_engine = 1000,
};

/// Hydralic valve body settings
typedef struct {
    // Pressure multiplier in 1st gear
    // This compensates for the fact that Shift pressure
    // is acting on the main pressure regulator in 1st gear
    float multiplier_in_1st_gear;
    float multiplier_all_gears;
    // Main pressure regulator spring pressure
    uint16_t lp_regulator_force_mbar;
    // Shift pressure regulator spring pressure
    uint16_t shift_regulator_force_mbar;
    // Shift circuit shift pressure multiplier for 1-2/2-1
    float shift_circuit_factor_1_2;
    // Solenoid inlet pressure offset vs working pressure (1st gear)
    uint16_t inlet_pressure_offset_mbar_first_gear;
    // Solenoid inlet pressure offset vs working pressure
    uint16_t inlet_pressure_offset_mbar_other_gears;
    // Inlet pressure correction algorithm pump speed minimum
    uint16_t pressure_correction_pump_speed_max;
    // Inlet pressure correction algorithm pump speed maximum
    uint16_t pressure_correction_pump_speed_min;
    // Working pressure vs inlet pressure correction
    LinearInterpSetting working_pressure_compensation;
    // Factor for shift pressure when the K1 clutch is engaged
    float k1_engaged_factor;
    // Minimum MPC pressure
    uint16_t minimum_mpc_pressure;
} __attribute__ ((packed)) VBY_SETTINGS;

// Hydralic configuration
// It is unknown at this moment if the settings here
// simply attain to small/large NAG, but it does NOT
// correlate with brown/blue solenoids.
typedef struct {
    // Valve body settings type 1
    // Type 1 valve bodys have solenoid pressure
    // from 0-7700mBar
    VBY_SETTINGS type0;
    // Valve body settings type 2
    // Type 2 valve bodys have solenoid pressure
    // from 0-9700mBar
    VBY_SETTINGS type1;
} __attribute__ ((packed)) HYD_MODULE_SETTINGS;

const HYD_MODULE_SETTINGS HYD_DEFAULT_SETTINGS = {
    .type0 = {
        .multiplier_in_1st_gear = 2.320,
        .multiplier_all_gears = 1.689,
        .lp_regulator_force_mbar = 1828,
        .shift_regulator_force_mbar = 601,
        .shift_circuit_factor_1_2 = 1.993,
        .inlet_pressure_offset_mbar_first_gear = 1500,
        .inlet_pressure_offset_mbar_other_gears = 1000,
        .pressure_correction_pump_speed_max = 1000,
        .pressure_correction_pump_speed_min = 4000,
        .working_pressure_compensation = {
            .new_min = 2690,
            .new_max = 8330,
            .raw_min = 3180,
            .raw_max = 8820,
        },
        .k1_engaged_factor = 1.993,
        .minimum_mpc_pressure = 500
    },
    .type1 = {
        .multiplier_in_1st_gear = 1.954,
        .multiplier_all_gears = 1.567,
        .lp_regulator_force_mbar = 1926,
        .shift_regulator_force_mbar = 601,
        .shift_circuit_factor_1_2 = 1.993,
        .inlet_pressure_offset_mbar_first_gear = 1500,
        .inlet_pressure_offset_mbar_other_gears = 1000,
        .pressure_correction_pump_speed_max = 1000,
        .pressure_correction_pump_speed_min = 4000,
        .working_pressure_compensation = {
            .new_min = 4000,
            .new_max = 10000,
            .raw_min = 4000,
            .raw_max = 10000,
        },
        .k1_engaged_factor = 1.993,
        .minimum_mpc_pressure = 1500
    }
};


typedef struct {
    // Maximum input torque the gearbox can withstand
    // UNIT: Nm
    uint16_t max_torque;
    // Gear 1 ratio
    float ratio_1;
    // Gear 2 ratio
    float ratio_2;
    // Gear 3 ratio
    float ratio_3;
    // Gear 4 ratio
    float ratio_4;
    // Gear 5 ratio
    float ratio_5;
    // Gear R1 ratio
    float ratio_r1;
    // Gear R2 ratio
    float ratio_r2;
    // Power loss in 1st gear
    // UNIT: %
    uint8_t power_loss_1;
    // Power loss in 2nd gear
    // UNIT: %
    uint8_t power_loss_2;
    // Power loss in 3rd gear
    // UNIT: %
    uint8_t power_loss_3;
    // Power loss in 4rd gear
    // UNIT: %
    uint8_t power_loss_4;
    // Power loss in 5th gear
    // UNIT: %
    uint8_t power_loss_5;
    // Power loss in R1 gear
    // UNIT: %
    uint8_t power_loss_r1;
    // Power loss in R2 gear
    // UNIT: %
    uint8_t power_loss_r2;
} __attribute__ ((packed)) NAG_SETTINGS;

// Gearbox configuration
typedef struct {
    // Maximum allowed drift from the target ratio in 1st gear.
    //
    // UNIT: %
    uint8_t max_drift_1;
    // Maximum allowed drift from the target ratio in 2nd gear.
    //
    // UNIT: %
    uint8_t max_drift_2;
    // Maximum allowed drift from the target ratio in 3rd gear.
    //
    // UNIT: %
    uint8_t max_drift_3;
    // Maximum allowed drift from the target ratio in 4tg gear.
    //
    // UNIT: %
    uint8_t max_drift_4;
    // Maximum allowed drift from the target ratio in 5th gear.
    //
    // UNIT: %
    uint8_t max_drift_5;
    // Maximum allowed drift from the target ratio in R1 gear.
    //
    // UNIT: %
    uint8_t max_drift_r1;
    // Maximum allowed drift from the target ratio in R2 gear.
    //
    // UNIT: %
    uint8_t max_drift_r2;
    // Small NAG settings
    NAG_SETTINGS small_nag;
    // Large NAG settings
    NAG_SETTINGS large_nag;
} __attribute__ ((packed)) NAG_MODULE_SETTINGS;

const NAG_MODULE_SETTINGS NAG_DEFAULT_SETTINGS = {
    .max_drift_1 = 10,
    .max_drift_2 = 10,
    .max_drift_3 = 9,
    .max_drift_4 = 8,
    .max_drift_5 = 5,
    .max_drift_r1 = 10,
    .max_drift_r2 = 10,
    .small_nag = {
        .max_torque = 330,
        .ratio_1 = 3.951,
        .ratio_2 = 2.423,
        .ratio_3 = 1.486,
        .ratio_4 = 1.000,
        .ratio_5 = 0.833,
        .ratio_r1 = -3.147,
        .ratio_r2 = -1.930,
        .power_loss_1 = 10,
        .power_loss_2 = 10,
        .power_loss_3 = 10,
        .power_loss_4 = 10,
        .power_loss_5 = 10,
        .power_loss_r1 = 10,
        .power_loss_r2 = 10,
    },
    .large_nag = {
        .max_torque = 580,
        .ratio_1 = 3.595,
        .ratio_2 = 2.186,
        .ratio_3 = 1.405,
        .ratio_4 = 1.000,
        .ratio_5 = 0.831,
        .ratio_r1 = -3.167,
        .ratio_r2 = -1.926,
        .power_loss_1 = 10,
        .power_loss_2 = 10,
        .power_loss_3 = 10,
        .power_loss_4 = 10,
        .power_loss_5 = 10,
        .power_loss_r1 = 10,
        .power_loss_r2 = 10,
    },
};

// Pressure manager settings
typedef struct {
    // Maximum Shift pressure with SPC solenoid off
    // UNIT: mBar
    uint16_t max_spc_pressure;
    // Maximum Modulating pressure with MPC solenoid off
    // UNIT: mBar
    uint16_t max_mpc_pressure;
    // Maximum line pressure
    // UNIT: mBar
    uint16_t max_line_pressure;
    // Engine RPM multiplier on line pressure
    LinearInterpSetting engine_rpm_pressure_multi;
    // K1 clutch factor for 1-2 and 2-1 shifting
    float k1_pressure_multi;
    // Time before shift solenoids are reduced PWM.
    // Setting this too low can result in the shift circuit
    // not activating!
    //
    // UNIT: milliseconds
    uint16_t shift_solenoid_pwm_reduction_time;
} __attribute__ ((packed)) PRM_MODULE_SETTINGS;



const PRM_MODULE_SETTINGS PRM_DEFAULT_SETTINGS = {
    .max_spc_pressure = 7000,
    .max_mpc_pressure = 7000,
    .max_line_pressure = 15000,
    .engine_rpm_pressure_multi = {
        .new_min = 1.0,
        .new_max = 1.5,
        .raw_min = 1000,
        .raw_max = 6000,
    },
    .k1_pressure_multi = 1.9,
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

enum AutoProfile: uint8_t {
    Sport = 0,
    Comfort = 1,
    Agility = 2,
    Winter = 3
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
    AutoProfile profile_idx_top;
    // When using a switch profile selector. This is the profile
    // to use when in the bottom position
    AutoProfile profile_idx_buttom;
    // If you have a CAN based EWM shifter (+/- markings on it),
    // but have a profile W/S switch rather than a profile button,
    // you can enable this option to force cycling of ALL profiles,
    // including manual ones, by toggling the switch.
    //
    // It is suggested to only enable this if you have a profile
    // display on the instrument cluster, so you know if the car
    // is in manual mode or not.
    bool ewm_shifter_switch_cycles_profiles;
} __attribute__ ((packed)) ETS_MODULE_SETTINGS;

const ETS_MODULE_SETTINGS ETS_DEFAULT_SETTINGS = {
    .trrs_has_profile_selector = true,
    .ewm_selector_type = EwmSelectorType::Button,
    .profile_idx_top = AutoProfile::Comfort,
    .profile_idx_buttom = AutoProfile::Sport,
    .ewm_shifter_switch_cycles_profiles = false,
};


// Hydralic calibration
enum HydralicCalibration: uint8_t {
    // 0-7700 mBar
    HydralicSet0 = 0,
    // 0-9700 mBar
    HydralicSet1 = 1
};

enum NagClutchCalibrationLookup: uint8_t {
    // Set 0 (Small NAG). Max torque 420/500Nm
    ClutchSet0 = 0,
    // Set 1 (Small NAG). Max torque 300/370Nm
    ClutchSet1 = 1,
    // Set 2 (Large NAG). Max torque 460/580Nm
    ClutchSet2 = 2,
    // Set 3 (Large NAG). Max torque 460/580Nm
    ClutchSet3 = 3,
    // Set 4 (Large NAG). Max torque 580/730Nm
    ClutchSet4 = 4,
};

enum NagSpringCalibrationLookup: uint8_t {
    // Set 0 (Default)
    SpringSet0 = 0,
    // Set 1 (Stronger K3 release spring)
    SpringSet1 = 1,
};

// EGS52 calibration lookup
typedef struct {
    // Hydralic calibration
    HydralicCalibration hydralic_set;
    // Clutch friction capabilities calibration
    NagClutchCalibrationLookup clutch_friction_set;
    // Clutch release spring calibration
    NagSpringCalibrationLookup clutch_release_spring_set;
} __attribute__ ((packed)) CAL_MODULE_SETTINGS;

const CAL_MODULE_SETTINGS CAL_DEFAULT_SETTINGS = {
    .hydralic_set = HydralicCalibration::HydralicSet0,
    .clutch_friction_set = NagClutchCalibrationLookup::ClutchSet0,
    .clutch_release_spring_set = NagSpringCalibrationLookup::SpringSet0
};

// module settings
extern TCC_MODULE_SETTINGS TCC_CURRENT_SETTINGS;
extern SOL_MODULE_SETTINGS SOL_CURRENT_SETTINGS;
extern SBS_MODULE_SETTINGS SBS_CURRENT_SETTINGS;
extern NAG_MODULE_SETTINGS NAG_CURRENT_SETTINGS;
extern PRM_MODULE_SETTINGS PRM_CURRENT_SETTINGS;
extern ADP_MODULE_SETTINGS ADP_CURRENT_SETTINGS;
extern ETS_MODULE_SETTINGS ETS_CURRENT_SETTINGS;
extern HYD_MODULE_SETTINGS HYD_CURRENT_SETTINGS;
extern CAL_MODULE_SETTINGS CAL_CURRENT_SETTINGS;

// Setting IDx
#define TCC_MODULE_SETTINGS_SCN_ID 0x01
#define SOL_MODULE_SETTINGS_SCN_ID 0x02
#define SBS_MODULE_SETTINGS_SCN_ID 0x03
#define NAG_MODULE_SETTINGS_SCN_ID 0x04
#define PRM_MODULE_SETTINGS_SCN_ID 0x05
#define ADP_MODULE_SETTINGS_SCN_ID 0x06
#define ETS_MODULE_SETTINGS_SCN_ID 0x07
#define HYD_MODULE_SETTINGS_SCN_ID 0x08
#define CAL_MODULE_SETTINGS_SCN_ID 0x09

namespace ModuleConfiguration {
    esp_err_t load_all_settings();
    esp_err_t reset_settings(uint8_t idx);
    esp_err_t write_settings(uint8_t module_id, uint16_t buffer_len, uint8_t* buffer);
    esp_err_t read_settings(uint8_t module_id, uint16_t* buffer_len, uint8_t** buffer);
}

#endif