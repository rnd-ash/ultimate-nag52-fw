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

enum class ShiftStage {
    Bleed = 1,
    Fill = 2,
    Torque = 3,
    Overlap = 4,
    MaxPressure = 5
};

enum class Clutch {
    K1 = 1,
    K2 = 2,
    K3 = 3,
    B1 = 4,
    B2 = 5,
    B3 = 6 // Reverse ONLY
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
    // Input shaft torque
    int16_t input_torque;
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

struct ShiftPhaseTiming {
    /// Ramp down from current pressure to new pressure
    uint16_t ramp_time;
    /// Hold time at requested pressure
    uint16_t hold_time;
};

/**
 * Shift circuit
*/
enum class ShiftCircuit {
    None = 0,
    sc_1_2 = 1 << 0,
    sc_2_3 = 1 << 1,
    sc_3_4 = 1 << 2
};

/**
 * @brief Shift data request structure
 * 
 */
struct ShiftData{
    ShiftCircuit shift_circuit;
    uint8_t targ_g;
    uint8_t curr_g;
 
    /** 
     * Bleed phase
     * Shift solenoid has not opened yet
     * This reduces the line pressure of the SPC line so that
     * there is not a spike in pressure when Shift solenoid opens
     */
    ShiftPhaseTiming bleed_data;
    uint16_t bleed_pressure;
};

struct GearRatioInfo {
    float ratio_max_drift;
    float ratio;
    float ratio_min_drift;
    float power_loss;
};

struct GearboxConfiguration{
    uint16_t max_torque;
    GearRatioInfo bounds[7];
};

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

struct   __attribute__ ((packed)) ShiftReportSegment {
    int16_t static_torque;
    int16_t driver_torque;
    int16_t egs_req_torque;
    uint16_t engine_rpm;
    uint16_t input_rpm;
    uint16_t output_rpm;
    uint16_t mpc_pressure;
    uint16_t spc_pressure;
    uint16_t timestamp;
};

#define MAX_OVERLAP_REPORTS 20
struct  __attribute__ ((packed)) ShiftReport {
    // Metadata
    int16_t atf_temp_c;
    ProfileGearChange change;
    uint8_t profile;
    uint8_t shift_status; // 0 = fail, 1 = OK
    uint8_t overlap_reading_size;
    uint16_t target_shift_speed;
    ShiftReportSegment start_reading;
    ShiftReportSegment bleed_reading;
    ShiftReportSegment prefill_reading;
    ShiftReportSegment overlap_readings[MAX_OVERLAP_REPORTS]; // Every 250ms (5 seconds max for load shifts), Every 500ms (10 seconds max for coasting)
    ShiftReportSegment end_reading;
    uint16_t detect_shift_start_ts;
    uint16_t detect_shift_end_ts;
    uint16_t detect_flare_ts;
};

static const int SD_Size = sizeof(ShiftReport);

#endif