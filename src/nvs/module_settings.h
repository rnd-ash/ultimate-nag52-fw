#ifndef __MODULE_SETTINGS_H
#define __MODULE_SETTINGS_H

#include <stdint.h>
#include <tcu_maths.h>
#include <esp_err.h>

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
    // Temperature threshold for adapting. Below this value,
    // adaptation does not occur due to lag time of the ATF pressure
    int16_t temp_threshold_adapt;
    // Open the converter to slipping (If locked) if the engine requests it
    // This is used on the M113K platform when the supercharger clutch
    // is about to engage, so that the shock of the supercharger coming on does
    // not cause too much discomfort
    bool react_on_engine_slip_request;
    // Open the converter fully, if the engine requests it.
    // This is usually used under very heavy load under low RPM
    bool react_on_engine_open_request;
    // If the TCC should open for release upshifts (Coasting upshifts)
    bool open_release_upshift;
    // If the TCC should open for release downshifts (Load downshifts)
    bool open_release_downshift;
    // If the TCC should open for crossover upshifts (Load upshifts)
    bool open_crossover_upshift;
    // If the TCC should open for crossover downshifts (Coasting downshifts)
    bool open_crossover_downshift;
    // Scaling for TCC output pressure based on temperature. 
    //
    // When cold, the ATF is thicker, thus a higher pressure can be commanded
    // with the same TCC solenoid PWM. This scaling is meant to mitigate this
    // effect.
    //
    // To disable this scaling, set output_min and output_max to 1.0
    LinearInterpSetting tcc_temp_multiplier;
} __attribute__ ((packed)) TCC_MODULE_SETTINGS;

const TCC_MODULE_SETTINGS TCC_DEFAULT_SETTINGS = {
    .adapt_enable = true,
    .enable_d1 = true,
    .enable_d2 = true,
    .enable_d3 = true,
    .enable_d4 = true,
    .enable_d5 = true,
    .adapt_test_interval_ms = 100,
    .temp_threshold_adapt = 70,
    .react_on_engine_slip_request = true,
    .react_on_engine_open_request = true,
    .open_release_upshift = true,
    .open_release_downshift = false,
    .open_crossover_upshift = false,
    .open_crossover_downshift = true,
    .tcc_temp_multiplier = LinearInterpSetting {
        .new_min = 0.6,
        .new_max = 1.0,
        .raw_min = -10,
        .raw_max = 70
    }
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
    // DEBUG - Show an 'F' marker in the gear display when the TCU
    // detects a flare condition
    bool f_shown_if_flare;
    // DEBUG - Show '^' or 'v' in the gear display when the shift
    // thread is active
    bool debug_show_up_down_arrows_in_r;
    // Torque threshold before harsher crossover shift algorithm is
    // used for upshifting 1-2. The absolute torque threshold is this value
    // multiplied by engine drag torque specified in TCU Configuration
    float crossover_trq_thres_1_2;
    // Torque threshold before harsher crossover shift algorithm is
    // used for upshifting 1-2. The absolute torque threshold is this value
    // multiplied by engine drag torque specified in TCU Configuration
    float crossover_trq_thres_2_3;
    // Torque threshold before harsher crossover shift algorithm is
    // used for upshifting 2-3. The absolute torque threshold is this value
    // multiplied by engine drag torque specified in TCU Configuration
    float crossover_trq_thres_3_4;
    // Torque threshold before harsher crossover shift algorithm is
    // used for upshifting 4-5. The absolute torque threshold is this value
    // multiplied by engine drag torque specified in TCU Configuration
    float crossover_trq_thres_4_5;
} __attribute__ ((packed)) SBS_MODULE_SETTINGS;

const SBS_MODULE_SETTINGS SBS_DEFAULT_SETTINGS = {
    .f_shown_if_flare = false,
    .debug_show_up_down_arrows_in_r = false,
    .crossover_trq_thres_1_2 = 2.0,
    .crossover_trq_thres_2_3 = 2.0,
    .crossover_trq_thres_3_4 = 2.0,
    .crossover_trq_thres_4_5 = 2.0,
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

// Release shift settings
typedef struct {
    // Past this output shaft RPM, torque requests
    // will not be activated when upshifting
    uint16_t output_rpm_disable_trq_req;
    // Below this RPM, a clutch will be considered 'stationary'
    // which triggers the clutch syncronization phases
    uint16_t clutch_stationary_rpm;
    // Clutch inertia control PID algorithm 'P' value (upshifts)
    int16_t pid_p_val_upshift;
    // Clutch inertia control PID algorithm 'I' value (downshifts)
    int16_t pid_i_val_upshift;
    // Clutch inertia control PID algorithm 'P' value (upshifts)
    int16_t pid_p_val_downshift;
    // Clutch inertia control PID algorithm 'I' value (downshifts)
    int16_t pid_i_val_downshift;
    // Mapping of pedal position to off clutch torque ramp release speed
    //
    // 'raw' values are pedal position (0-250 = 0-100%), 'new' values
    // are the output, in Nm/20ms reduction
    LinearInterpSetting torque_loss_speed_pedal_pos;
    // SPC ramp speed in mBar/20ms
    uint8_t spc_ramp_speed;
    // SPC ramp multiplier in 'Manual' mode
    float spc_ramp_multi_m;
    // SPC ramp multiplier in 'Race' mode
    float spc_ramp_multi_r;
    // SPC offset based on input RPM. If your shifts are sliggish at higher
    // RPMs, then you can increase the output pressure here
    LinearInterpSetting adder_spc_rpm;
    // SPC offset based on pedal position for Race mode. Pedal is from 0-250
    LinearInterpSetting adder_spc_pedal_r;
    // SPC offset based on pedal position for Manual mode. Pedal is from 0-250
    LinearInterpSetting adder_spc_pedal_m;
    // SPC offset based on pedal position for all other modes. Pedal is from 0-250
    LinearInterpSetting adder_spc_pedal;
} __attribute__ ((packed)) REL_MODULE_SETTINGS;

const REL_MODULE_SETTINGS REL_DEFAULT_SETTINGS = {
    .output_rpm_disable_trq_req = 1500,
    .clutch_stationary_rpm = 130,
    .pid_p_val_upshift = -150,
    .pid_i_val_upshift = -5,
    .pid_p_val_downshift = 200,
    .pid_i_val_downshift = 5,
    .torque_loss_speed_pedal_pos = {
        .new_min = 0.1,
        .new_max = 0.5,
        .raw_min = 10,
        .raw_max = 250,
    },
    .spc_ramp_speed = 8,
    .spc_ramp_multi_m = 1.5,
    .spc_ramp_multi_r = 2.0,
    .adder_spc_rpm = {
        .new_min = 0,
        .new_max = 100,
        .raw_min = 2500,
        .raw_max = 5000,
    },
    .adder_spc_pedal_r = {
        .new_min = 0,
        .new_max = 500,
        .raw_min = 10,
        .raw_max = 250,
    },
    .adder_spc_pedal_m = {
        .new_min = 0,
        .new_max = 250,
        .raw_min = 10,
        .raw_max = 250,
    },
    .adder_spc_pedal = {
        .new_min = 0,
        .new_max = 100,
        .raw_min = 10,
        .raw_max = 250,
    }
};

// Garage shift settings
typedef struct {
    // Number of 20ms cycles before garage shift times out
    // and the TCU tries again
    uint16_t timeout_cycles;
    // Prefilling time for B2 clutch (For N to D shift)
    // 'raw' values are the ATF Temperature (In Celcius), 'new' values
    // are the number of 20ms cycles for prefilling (so 20 would be 400ms)
    LinearInterpSetting prefill_time_b2;
    // Prefilling time for B3 clutch (For N to R shift)
    // 'raw' values are the ATF Temperature (In Celcius), 'new' values
    // are the number of 20ms cycles for prefilling (so 20 would be 400ms)
    LinearInterpSetting prefill_time_b3;

    // Apply ramp for B2 clutch
    // 'raw' values are the ATF Temperature (In Celcius), 'new' values
    // are the pressure added to B2 every 20ms until it engages
    LinearInterpSetting p_ramp_b2;
    // Apply ramp for B3 clutch
    // 'raw' values are the ATF Temperature (In Celcius), 'new' values
    // are the pressure added to B2 every 20ms until it engages
    LinearInterpSetting p_ramp_b3;
    // Modulating pressure adder factor of shift pressure for N to D shift
    // (mod = working + (mod_mul_b2*spc))
    float mod_mul_b2;
    // Modulating pressure adder factor of shift pressure for N to R shift
    // (mod = working + (mod_mul_b2*spc))
    float mod_mul_b3;
} __attribute__ ((packed)) GAR_MODULE_SETTINGS;

const GAR_MODULE_SETTINGS GAR_DEFAULT_SETTINGS = {
    .timeout_cycles = 250,
    .prefill_time_b2 = {
        .new_min = 15,
        .new_max = 4,
        .raw_min = -10,
        .raw_max = 80,
    },
    .prefill_time_b3 = {
        .new_min = 15,
        .new_max = 4,
        .raw_min = -10,
        .raw_max = 80,
    },
    .p_ramp_b2 = {
        .new_min = 20,
        .new_max = 7,
        .raw_min = -10,
        .raw_max = 80,
    },
    .p_ramp_b3 = {
        .new_min = 20,
        .new_max = 7,
        .raw_min = -10,
        .raw_max = 80,
    },
    .mod_mul_b2 = 0.25,
    .mod_mul_b3 = 0.25
};

// Crossover shift settings
typedef struct {
    // Below this RPM, a clutch will be considered 'stationary'
    // which triggers the clutch syncronization phases
    uint16_t clutch_stationary_rpm;
    // Number of 20ms cycles for the overlap phase when at low torque (<= 2x Drag torque)
    uint8_t overlap_cycles_low_trq;
    // Number of 20ms cycles for the overlap phase when at high torque (>= 10x Drag torque)
    uint8_t overlap_cycles_high_trq;
    // Adder to overlap_cycles_low_trq for 1-2 at low torque
    uint8_t overlap_cycles_low_trq_adder_1_2;
    // Adder to overlap_cycles_high_trq for 1-2 at low torque
    uint8_t overlap_cycles_high_trq_adder_1_2;
    // Adder to overlap_cycles based on RPM. Increasing the output minimum can help
    // with harsh shifting at lower RPMs
    LinearInterpSetting overlap_cycles_adder_rpm;
    // Multiplier for the overlap cycles based on target shift speed
    LinearInterpSetting overlap_multi_shift_speed;
    // Torque adder factor for upshifting in normal profiles
    float adder_trq_multi_normal_up;
    // Torque adder factor for upshifting in manual profile
    float adder_trq_multi_manual_up;
    // Torque adder factor for upshifting in race profile
    float adder_trq_multi_race_up;
    // Torque adder factor for downshifting in normal profiles
    float adder_trq_multi_normal_dn;
    // Torque adder factor for downshifting in manual profile
    float adder_trq_multi_manual_dn;
    // Torque adder factor for downshifting in race profile
    float adder_trq_multi_race_dn;
    // Number of 20ms cycles for the torque sync phase when at low torque (<= 2x Drag torque)
    uint8_t sync_cycles_low_trq;
    // Number of 20ms cycles for the torque sync phase when at high torque (>= 10x Drag torque)
    uint8_t sync_cycles_high_trq;
    // Adder to sync_cycles_low_trq for 1-2 at low torque
    uint8_t sync_cycles_low_trq_adder_1_2;
    // Adder to sync_cycles_high_trq for 1-2 at low torque
    uint8_t sync_cycles_high_trq_adder_1_2;
    // Adder to sync_cycles based on RPM. Increasing the output minimum can help
    // with harsh shifting at lower RPMs
    LinearInterpSetting sync_cycles_adder_rpm;
    // Multiplier for the output sync cycles based on target shift speed
    LinearInterpSetting sync_multi_shift_speed;
    // Torque sync torque adder ramp value based on shift speed (Output is in Nm/20ms)
    LinearInterpSetting sync_trq_adder_speed;

    // Torque request multiplier based on pedal position (100% = 250)
    LinearInterpSetting trq_req_multi_pedal_pos;
    // Torque request multiplier based on input RPM
    LinearInterpSetting trq_req_multi_input_rpm;
} __attribute__ ((packed)) CRS_MODULE_SETTINGS;

const CRS_MODULE_SETTINGS CRS_DEFAULT_SETTINGS = {
    .clutch_stationary_rpm = 130,
    .overlap_cycles_low_trq = 10,
    .overlap_cycles_high_trq = 6,
    .overlap_cycles_low_trq_adder_1_2 = 3,
    .overlap_cycles_high_trq_adder_1_2 = 2,
    .overlap_cycles_adder_rpm = LinearInterpSetting {
        .new_min = 3,
        .new_max = 1,
        .raw_min = 1000,
        .raw_max = 4000
    },
    .overlap_multi_shift_speed = LinearInterpSetting {
        .new_min = 0.5,
        .new_max = 1.0,
        .raw_min = 100,
        .raw_max = 750
    },

    .adder_trq_multi_normal_up = 1.0,
    .adder_trq_multi_manual_up = 1.5,
    .adder_trq_multi_race_up = 2.0,
    .adder_trq_multi_normal_dn = 1.0,
    .adder_trq_multi_manual_dn = 1.5,
    .adder_trq_multi_race_dn = 2.0,

    .sync_cycles_low_trq = 10,
    .sync_cycles_high_trq = 7,
    .sync_cycles_low_trq_adder_1_2 = 2,
    .sync_cycles_high_trq_adder_1_2 = 1,
    .sync_cycles_adder_rpm = LinearInterpSetting {
        .new_min = 0,
        .new_max = 2,
        .raw_min = 1000,
        .raw_max = 4000
    },
    .sync_multi_shift_speed = LinearInterpSetting {
        .new_min = 0.5,
        .new_max = 1.0,
        .raw_min = 100,
        .raw_max = 750
    },
    .sync_trq_adder_speed = LinearInterpSetting {
        .new_min = 0.25,
        .new_max = 1.0,
        .raw_min = 1000,
        .raw_max = 100
    },

    .trq_req_multi_pedal_pos = LinearInterpSetting {
        .new_min = 0,
        .new_max = 0.5,
        .raw_min = 25,
        .raw_max = 250
    },
    .trq_req_multi_input_rpm = LinearInterpSetting {
        .new_min = 1.0,
        .new_max = 1.5,
        .raw_min = 1500,
        .raw_max = 6000
    },
};

// module settings
extern TCC_MODULE_SETTINGS TCC_CURRENT_SETTINGS;
extern SOL_MODULE_SETTINGS SOL_CURRENT_SETTINGS;
extern SBS_MODULE_SETTINGS SBS_CURRENT_SETTINGS;
extern PRM_MODULE_SETTINGS PRM_CURRENT_SETTINGS;
extern ADP_MODULE_SETTINGS ADP_CURRENT_SETTINGS;
extern ETS_MODULE_SETTINGS ETS_CURRENT_SETTINGS;
extern REL_MODULE_SETTINGS REL_CURRENT_SETTINGS;
extern GAR_MODULE_SETTINGS GAR_CURRENT_SETTINGS;
extern CRS_MODULE_SETTINGS CRS_CURRENT_SETTINGS;

// Setting IDx
#define TCC_MODULE_SETTINGS_SCN_ID 0x01
#define SOL_MODULE_SETTINGS_SCN_ID 0x02
#define SBS_MODULE_SETTINGS_SCN_ID 0x03
#define PRM_MODULE_SETTINGS_SCN_ID 0x05
#define ADP_MODULE_SETTINGS_SCN_ID 0x06
#define ETS_MODULE_SETTINGS_SCN_ID 0x07
#define REL_MODULE_SETTINGS_SCN_ID 0x08
#define GAR_MODULE_SETTINGS_SCN_ID 0x09
#define CRS_MODULE_SETTINGS_SCN_ID 0x0A

namespace ModuleConfiguration {
    esp_err_t load_all_settings();
    esp_err_t reset_settings(uint8_t idx);
    esp_err_t write_settings(uint8_t module_id, uint16_t buffer_len, uint8_t* buffer);
    esp_err_t read_settings(uint8_t module_id, uint16_t* buffer_len, uint8_t** buffer);
}

#endif