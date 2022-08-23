/** @file */
#ifndef COMMON_STRUCTS_H__
#define COMMON_STRUCTS_H__

#include <stdint.h>
#include "solenoids/solenoids.h"

typedef int16_t pressure_map[11];
typedef float rpm_modifier_map[9];

template<typename T, uint8_t MAX_SIZE> struct MovingAverage {
    T readings[MAX_SIZE];
    uint8_t sample_id;
    uint64_t sum;

    MovingAverage() {
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

    T get_average() {
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
typedef struct {
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
} SensorData;

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

typedef struct {
    /// Ramp down from current pressure to new pressure
    uint16_t ramp_time;
    /// Hold time at requested pressure
    uint16_t hold_time;
    /// request pressure in mBar for SPC
    uint16_t spc_pressure;
    /// How much to modify MPC pressure by in mBar
    int16_t mpc_pressure;
} ShiftPhase;

/**
 * @brief Shift data request structure
 * 
 */
typedef struct {
    Solenoid* shift_solenoid;
    float torque_down_amount;
    uint8_t targ_g;
    uint8_t curr_g;
 
    /** 
     * Phase 1 hold
     * Shift solenoid has not opened yet
     * This reduces the line pressure of the SPC line so that
     * there is not a spike in pressure when Shift solenoid opens
     */
    ShiftPhase hold1_data;

    /** 
     * Phase 2 hold 
     * Shift solenoid opens, PWM instantly jumps rather than ramps
     * This bleeds air out of the lines
     */

    ShiftPhase hold2_data;

    /** 
     * Phase 3 hold 
     * Removing any play in the new clutches to engage
     */

    ShiftPhase hold3_data;

    /** 
     * Shift torque phase 
     * New clutches start to spin to take the torque of the old ones
     */
    ShiftPhase torque_data;

    /** 
     * Shift overlap phase
     * New clutches are moved into place and old once released
     */

    ShiftPhase overlap_data;

    /** 
     * Shift complete max pressure phase 
     * New clutches are locked into place
     */

    ShiftPhase max_pressure_data;
} ShiftData;

typedef struct {
    float max;
    float min;
} GearRatioLimit;

typedef const GearRatioLimit GearboxRatioBounds[7];
typedef const float FwdRatios[7];

typedef struct {
    uint16_t max_torque;
    const GearRatioLimit* bounds;
    const float* ratios; // 1-5 and R1+R2
} GearboxConfiguration;


typedef struct {
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
} PressureMgrData;

typedef struct {
    // Target delta RPM per step (Each step ~= 40ms)
    uint16_t target_d_rpm;
    // Valid range =  1 - 10 (Auto clamped if value is outside this range) - Higher = faster shift
    float shift_speed;
} ShiftCharacteristics;

typedef struct {
    int max_slip_rpm;
    int min_slip_rpm;
} TccLockupBounds;

#define SR_REPORT_INTERVAL 50
#define MAX_POINTS_PER_SR_ARRAY 6000/SR_REPORT_INTERVAL // Every 50ms
typedef struct {
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
    ShiftPhase hold1_data;
    ShiftPhase hold2_data;
    ShiftPhase hold3_data;
    ShiftPhase torque_data;
    ShiftPhase overlap_data;
    ShiftPhase max_pressure_data;
    uint16_t transition_start;
    uint16_t transition_end;

    // Flags for reporting shift errors
    uint16_t flare_timestamp;
    uint8_t shift_timeout;
}  __attribute__ ((packed)) ShiftReport;

const int SD_Size = sizeof(ShiftReport);

#endif