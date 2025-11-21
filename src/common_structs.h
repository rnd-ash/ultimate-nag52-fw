/** @file */
#ifndef COMMON_STRUCTS_H
#define COMMON_STRUCTS_H

#include <stdint.h>
#include "solenoids/solenoids.h"
#include "canbus/can_defines.h"
#include "firstorder_average.h"

typedef int16_t pressure_map[11];
typedef float rpm_modifier_map[9];

enum class Clutch {
    K1 = 0,
    K2 = 1,
    K3 = 2,
    B1 = 3,
    B2 = 4,
    B3 = 5
};


struct ShiftClutchData {
    int16_t on_clutch_speed;
    int16_t off_clutch_speed;
    int16_t rear_sun_speed;
};

struct ShiftAlgoFeedback {
    uint8_t active;
    uint8_t shift_phase;
    uint8_t subphase_shift;
    uint8_t subphase_mod;
    uint16_t sync_rpm;
    int16_t pid_torque;
    int16_t adder_torque;
    uint16_t p_on;
    uint16_t p_off;
    int16_t s_off;
    int16_t s_on;
} __attribute__ ((packed));

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
    const FirstOrderAverage* pedal_smoothed;
    // in %/sec
    FirstOrderAverage* pedal_delta;
    /// Transmission oil temperature in Celcius
    int16_t atf_temp;
    // Input shaft torque
    int16_t input_torque;
    int16_t converted_torque;
    int16_t converted_driver_torque;
    int16_t indicated_torque;
    /// Engine torque limit maximum in Nm
    int16_t max_torque;
    /// Engine torque limit minimum in Nm
    int16_t min_torque;
    /// Last time the gearbox changed gear (in milliseconds)
    uint32_t last_shift_time;
    /// Current gearbox ratio
    float gear_ratio;
    /// Target gearbox ratio
    float targ_gear_ratio;
    float tcc_trq_multiplier;
    bool kickdown_pressed;
    bool brake_pressed;
};

struct OutputData {
    float torque_req_amount;
    TorqueRequestControlType ctrl_type;
    TorqueRequestBounds bounds;
};

/**
 * @brief A gearchange that a AbstractProfile can request
 * Compatible with OEM EGS data type
 */
enum class GearChange {
    /// Gear 1 to gear 2
    _IDLE = 0,
    _1_2 = 1,
    _2_3 = 2,
    _3_4 = 3,
    _4_5 = 4,
    _2_1 = 5,
    _3_2 = 6,
    _4_3 = 7,
    _5_4 = 8,
};

/**
 * Shifting circuit definition for the 722.6
 * 
 * A shifting circuit acts like a Flip-Flop circuit. Activating the circuit causes
 * the toggle between 2 clutches on the gearbox (Thus completing a gear change). Once the circuit
 * is disengaged, the clutch state is untouched until the solenoid activates again, causing a toggle
 * again.
*/
enum class ShiftCircuit {
    /**
     * @brief No shift circuit (placeholder)
     */
    None = 0,

    /**
     * @brief 1/2 and 4/5 shift circuit. This controls the toggling between the B1 and K1 clutches
     */
    sc_1_2 = 1 << 0,

    /**
     * @brief 2/3 shift circuit. This controls the toggling between the K2 and K3 clutches
     */
    sc_2_3 = 1 << 1,

    /**
     * @brief 3/4 shift circuit. This controls the toggling between the B2 and K3 clutches
     */
    sc_3_4 = 1 << 2,

    /**
     * @brief 4/5 shift circuit. This controls the toggling between the B1 and K1 clutches
     */
    sc_4_5 = 1 << 3,
};

/**
 * @brief Shift data request structure
 * 
 */
struct CircuitInfo{
    uint8_t map_idx;
    ShiftCircuit shift_circuit;
    uint8_t targ_g;
    uint8_t curr_g;
    /// 1000x real value
    int pressure_multi_spc_int;
    /// 1000x real value
    int pressure_multi_mpc_int;
    int mpc_pressure_spring_reduction;
    float centrifugal_factor_off_clutch;
};

#define RAT_1_IDX 0
#define RAT_2_IDX 1
#define RAT_3_IDX 2
#define RAT_4_IDX 3
#define RAT_5_IDX 4
#define RAT_R1_IDX 5
#define RAT_R2_IDX 6

struct GearRatioInfo {
    float ratio_max_drift;
    float ratio;
    float ratio_min_drift;
};

struct GearboxConfiguration{
    uint16_t max_torque;
    /**
     * At index
     * 0 - 1st
     * 1 - 2nd
     * 2 - 3rd
     * 3 - 4th
     * 4 - 5th
     * 5 - R1
     * 6 - R2
    */
    GearRatioInfo bounds[7];
};

struct ShiftCharacteristics{
    uint16_t target_shift_time;
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
    GearChange change;
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