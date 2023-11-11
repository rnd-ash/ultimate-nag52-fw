/** @file */
#ifndef COMMON_STRUCTS_H
#define COMMON_STRUCTS_H

#include <stdint.h>
#include "solenoids/solenoids.h"
#include "canbus/can_defines.h"

typedef int16_t pressure_map[11];
typedef float rpm_modifier_map[9];

/**
 * @brief Shifting state
 */
enum class ShiftStage {
    /**
     * @brief Hydralic bleed phase.
     * In this phase, Shift pressure fluid is bled in order to not over pressurise the overlap valve
     * The shift solenoid is not active
    */
    Bleed = 1,
    /**
     * @brief Clutch filling phase.
     * In this phase, the shift solenoid is engaged, and Shift pressure is ramped in order to clear
     * the tolorance between the engaging clutch pack, without actually applying it.
     */
    Fill = 2,
    /**
     * @brief Clutch overlap phase.
     * In this phase, the engaging clutch's pressure is raised, whilst the disengaging clutch's pressure
     * is reduced, causing (Via the overlap hydralic valve), the gearbox to transition clutch packs.
     */
    Overlap = 3,
    /**
     * @brief Shift pressure is increased to its maximum via a ramp in order to lock the engaging clutch in place.
     * Then, the shift solenoid turns off, completing the gear change.
     */
    MaxPressure = 4
};

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

struct ShiftClutchVelocity {
    int16_t on_clutch_vel;
    int16_t off_clutch_vel;
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
    /// Last time the gearbox changed gear (in milliseconds)
    uint32_t last_shift_time;
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
    TorqueRequestControlType ctrl_type;
    TorqueRequestBounds bounds;
};

/**
 * @brief A gearchange that a AbstractProfile can request
 * 
 */
enum class ProfileGearChange {
    /// Gear 1 to gear 2
    ONE_TWO = 0,
    /// Gear 2 to gear 3
    TWO_THREE = 1,
    /// Gear 3 to gear 4
    THREE_FOUR = 2,
    /// Gear 4 to gear 5
    FOUR_FIVE = 3,
    /// Gear 5 down to gear 4
    FIVE_FOUR = 4,
    /// Gear 4 down to gear 3
    FOUR_THREE = 5,
    /// Gear 3 down to gear 2
    THREE_TWO = 6,
    /// Gear 2 down to gear 1
    TWO_ONE = 7,
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
struct ShiftData{
    ShiftCircuit shift_circuit;
    uint8_t targ_g;
    uint8_t curr_g;
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
    float power_loss;
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