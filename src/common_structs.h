/** @file */
#ifndef COMMON_STRUCTS_H
#define COMMON_STRUCTS_H

#include <stdint.h>
#include "solenoids/solenoids.h"
#include "canbus/can_hal.h"

typedef int16_t pressure_map[11];
typedef float rpm_modifier_map[9];

template<typename T, uint8_t MAX_SIZE> struct MovingAverage {
    T readings[MAX_SIZE];
    uint8_t sample_id;
    uint64_t sum;

    MovingAverage(void) {
        this->sample_id = 0;
        this->sum = 0;
        memset(this->readings, 0x00, sizeof(this->readings));
    }

    void add_to_sample(T reading) {
        this->sum -= this->readings[this->sample_id];
        this->readings[this->sample_id] = reading;
        this->sum += reading;
        this->sample_id = (this->sample_id+1) % MAX_SIZE;
    }

    T get_average(void) {
        return (float)sum / (float)MAX_SIZE;
    }
};

/**
 * @brief Gearbox sensor data
 * This structure gets passed between a lot of the gearbox
 * code and is used for decision making about pressures and gear
 * change behaviour
 * 
 */
struct SensorData{
    /// Gearbox input RPM
    uint16_t input_rpm;
    /// Engine RPM
    uint16_t engine_rpm;
    /// Output shaft RPM
    uint16_t output_rpm;
    /// Accelerator pedal position. 0-255
    uint8_t pedal_pos;
    /// Transmission oil temperature in Celcius
    int16_t atf_temp;
    /// Current 'static' torque of the engine in Nm
    int16_t static_torque;
    /// Engine torque limit maximum in Nm
    int16_t max_torque;
    /// Engine torque limit minimum in Nm
    int16_t min_torque;
    /// Driver requested torque
    int16_t driver_requested_torque;
    /// Torque converter slip RPM (Calculated)
    int16_t tcc_slip_rpm;
    /// Last time the gearbox changed gear (in milliseconds)
    uint64_t last_shift_time;
    /// Current clock time in milliseconds
    uint64_t current_timestamp_ms;
    /// Is the brake pedal depressed?
    bool is_braking;
    /// Delta in output RPM, used for calculating if car is accelerating or slowing down
    int16_t d_output_rpm;
    /// Current gearbox ratio
    float gear_ratio;
    WheelData rr_wheel;
    WheelData rl_wheel;
    WheelData fr_wheel;
    WheelData fl_wheel;
};

struct OutputData {
    float torque_req_amount;
    TorqueRequest torque_req_type;
};

/**
 * @brief A gearchange that a AbstractProfile can request
 * 
 */
enum class ProfileGearChange {
    /// Gear 1 to gear 2
    ONE_TWO,
    /// Gear 2 to gear 3
    TWO_THREE,
    /// Gear 3 to gear 4
    THREE_FOUR,
    /// Gear 4 to gear 5
    FOUR_FIVE,
    /// Gear 5 down to gear 4
    FIVE_FOUR,
    /// Gear 4 down to gear 3
    FOUR_THREE,
    /// Gear 3 down to gear 2
    THREE_TWO,
    /// Gear 2 down to gear 1
    TWO_ONE,
};

struct ShiftPhase{
    /// Ramp down from current pressure to new pressure
    uint16_t ramp_time;
    /// Hold time at requested pressure
    uint16_t hold_time;
    /// Requested pressure for shift pressure
    /// This value is interpreted differently based on the value of `spc_offset_mode`.
    /// 
    /// If `spc_offset_mode` is true, then spc_pressure is interpreted as an offset to the value of `mpc_pressure`.
    ///
    /// If `spc_offset_mode` is false, then spc_pressure is interpreted as a fixed static value. 
    int16_t spc_pressure;
    /// Requested pressure for modulating (Working) pressure
    /// This value is interpreted differently based on the value of `mpc_offset_mode`.
    /// 
    /// If `mpc_offset_mode` is true, then mpc_pressure is interpreted as an offset to the current working pressure.
    /// The working pressure is calculated dynamically based on the current application state of the clutches in the gearbox
    /// and the load on the input shaft of the gearbox
    ///
    /// If `mpc_offset_mode` is false, then mpc_pressure is interpreted as a fixed static value. 
    int16_t mpc_pressure;
    /// If true, spc_pressure is interpreted as an offset to mpc_pressure
    bool spc_offset_mode;
    /// If true, mpc_pressure is interpreted as an offset to working pressure
    bool mpc_offset_mode;
};

/**
 * @brief Shift data request structure
 * 
 */
struct ShiftData{
    Solenoid* shift_solenoid;
    float torque_down_amount;
    uint8_t targ_g;
    uint8_t curr_g;
 
    /** 
     * Bleed phase
     * Shift solenoid has not opened yet
     * This reduces the line pressure of the SPC line so that
     * there is not a spike in pressure when Shift solenoid opens
     */
    ShiftPhase bleed_data;

    /** 
     * Fill phase
     * Clutches are moved into 0 tolerance position
     * This phase uses adaptation data
     * 
     * AFTER THIS PHASE, SHIFT CANNOT BE ABORTED!
     */
    ShiftPhase fill_data;

    /** 
     * Shift torque phase 
     * Just before overlap, engine torque is reduced to aid shifting
     */
    ShiftPhase torque_data;

    /** 
     * Shift overlap phase
     * New clutches are moved into place and old once released
     * SPC now controls new clutches, MPC is relaxed.
     * 
     * Duration and pressure during the overlap phase
     * affects shift feeling
     * 
     * Duration: Longer = slower gear change
     * SPC Ramp: Bigger = firmer shift
     */
    ShiftPhase overlap_data;

    /** 
     * Shift complete max pressure phase 
     * New clutches are locked into place
     */
    ShiftPhase max_pressure_data;
};

struct GearRatioLimit{
    float max;
    float min;
};


typedef const float FwdRatios[7];

struct GearboxConfiguration{
    uint16_t max_torque;
    const GearRatioLimit* bounds;
    const float* ratios; // 1-5 and R1+R2
} ;

struct PressureMgrData{
    pressure_map spc_1_2;
    pressure_map mpc_1_2;

    pressure_map spc_2_3;
    pressure_map mpc_2_3;

    pressure_map spc_3_4;
    pressure_map mpc_3_4;

    pressure_map spc_4_5;
    pressure_map mpc_4_5;

    pressure_map spc_5_4;
    pressure_map mpc_5_4;

    pressure_map spc_4_3;
    pressure_map mpc_4_3;

    pressure_map spc_3_2;
    pressure_map mpc_3_2;

    pressure_map spc_2_1;
    pressure_map mpc_2_1;

    pressure_map working_mpc_p;
    pressure_map working_mpc_r;
    pressure_map working_mpc_1;
    pressure_map working_mpc_2;
    pressure_map working_mpc_3;
    pressure_map working_mpc_4;
    pressure_map working_mpc_5;

    rpm_modifier_map ramp_speed_multiplier;
} ;

struct ShiftCharacteristics{
    uint16_t target_shift_time;
} ;

struct TccLockupBounds{
    int max_slip_rpm;
    int min_slip_rpm;
};

#define SR_REPORT_INTERVAL 50
#define MAX_POINTS_PER_SR_ARRAY 6000/SR_REPORT_INTERVAL // Every 50ms
struct  __attribute__ ((packed)) ShiftReport{
    int16_t atf_temp_c;
    // Target << 4 | current
    uint8_t targ_curr;
    uint8_t profile;
    uint16_t requested_torque;
    uint8_t interval_points;
    uint16_t report_array_len;
    uint16_t engine_rpm[MAX_POINTS_PER_SR_ARRAY];
    uint16_t input_rpm[MAX_POINTS_PER_SR_ARRAY];
    uint16_t output_rpm[MAX_POINTS_PER_SR_ARRAY];
    int16_t engine_torque[MAX_POINTS_PER_SR_ARRAY];
    uint16_t total_ms;
    uint16_t initial_mpc_pressure; // used to calculate with the ramps what MPC is doing
    ShiftPhase bleed_data;
    ShiftPhase fill_data;
    ShiftPhase torque_data;
    ShiftPhase overlap_data;
    ShiftPhase max_pressure_data;
    uint16_t transition_start;
    uint16_t transition_end;

    // Flags for reporting shift errors
    uint16_t flare_timestamp;
    uint8_t shift_timeout;
} ;

static const int SD_Size = sizeof(ShiftReport);

#endif