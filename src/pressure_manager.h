#ifndef __PRESSURE_MANAGER_H_
#define __PRESSURE_MANAGER_H_

#include <common_structs.h>
#include "tcm_maths.h"
#include "profiles.h"
#include "adaptation/adapt_map.h"
#include <gearbox_config.h>
#include "nvs/eeprom_config.h"

#define CLUTCH_NO_MOVE 0
#define CLUTCH_APPLY 1
#define CLUTCH_RELEASE 2

// For each of the tables:
//                                           K1,   K2,   K3,   B1,   B2,   B3
const uint16_t clutch_torque_factor_pn[6] {7003,    0,    0, 4765,    0,    0}; // N/P 11769

const uint16_t clutch_torque_factor_d1[6] {   0,    0, 4331, 3004, 3194,    0}; // D1  10529
const uint16_t clutch_torque_factor_d2[6] {2708,    0, 2656,    0, 1959,    0}; // D2  7323
const uint16_t clutch_torque_factor_d3[6] {   0, 1652,    0,    0, 1329,    0}; // D3  2981
const uint16_t clutch_torque_factor_d4[6] {1403, 2507, 1376,    0,    0,    0}; // D4  5286
const uint16_t clutch_torque_factor_d5[6] {   0, 2089, 1146,  795,    0,    0}; // D5  4030

const uint16_t clutch_torque_factor_r1[6] {   0,    0, 4331, 3004,    0, 4204}; // R1
const uint16_t clutch_torque_factor_r2[6] {   0,    0, 2656,    0,    0, 2579}; // R2

/**
 *                               K  K  K  B  B  B
 *                               1  2  3  1  2  3
 */
// Garage shifting
const bool clutch_move_pn_d1[6] {2, 0, 1, 0, 1, 0}; // N  -> D1
const bool clutch_move_pn_d2[6] {0, 0, 1, 2, 1, 0}; // N  -> D2
const bool clutch_move_d1_pn[6] {1, 0, 2, 0, 2, 0}; // D1 -> N
const bool clutch_move_d2_pn[6] {0, 0, 2, 1, 2, 0}; // D2 -> N
const bool clutch_move_pn_r1[6] {2, 0, 1, 0, 0, 1}; // N  -> R1
const bool clutch_move_pn_r2[6] {2, 0, 1, 2, 0, 1}; // N  -> R2
const bool clutch_move_r1_pn[6] {1, 0, 2, 0, 0, 2}; // R1 -> N
const bool clutch_move_r2_pn[6] {1, 0, 2, 1, 0, 2}; // R2 -> N

// Fwd shifting (Up)
const bool clutch_move_d1_d2[6] {1, 0, 0, 2, 0, 0}; // D1 -> D2
const bool clutch_move_d2_d3[6] {2, 1, 2, 0, 0, 0}; // D2 -> D3
const bool clutch_move_d3_d4[6] {1, 0, 1, 0, 2, 0}; // D3 -> D4
const bool clutch_move_d4_d5[6] {2, 0, 0, 1, 0, 0}; // D4 -> D5

// Fwd shifting (Down)
const bool clutch_move_d2_d1[6] {2, 0, 0, 1, 0, 0}; // D2 -> D1
const bool clutch_move_d3_d2[6] {1, 2, 1, 0, 0, 0}; // D3 -> D2
const bool clutch_move_d4_d3[6] {2, 0, 2, 0, 1, 0}; // D4 -> D3
const bool clutch_move_d5_d4[6] {1, 0, 0, 2, 0, 0}; // D5 -> D4

// Shift phase IDs

#define SHIFT_PHASE_HOLD_1 1
#define SHIFT_PHASE_HOLD_2 2
#define SHIFT_PHASE_HOLD_3 3
#define SHIFT_PHASE_TORQUE 4
#define SHIFT_PHASE_OVERLAP 5
#define SHIFT_PHASE_MAX_P 6

// 0 -> 100% rated torque (In mbar)
const pressure_map working_norm_pressure_p = { 500,  600, 700,   800,  900, 1000, 1100, 1200, 1300, 1400, 1500}; // Park or N
const pressure_map working_norm_pressure_r = { 500,  750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000}; // R1 or R2
const pressure_map working_norm_pressure_1 = { 500,  750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000}; // 1
const pressure_map working_norm_pressure_2 = { 500,  750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000}; // 2
const pressure_map working_norm_pressure_3 = { 500,  750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000}; // 3
const pressure_map working_norm_pressure_4 = { 500,  750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000}; // 4
const pressure_map working_norm_pressure_5 = { 500,  750, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000}; // 5

typedef void (*P_RAMP_FUNC)(float, float);

class PressureManager {

public:

    void set_target_mpc_pressure(uint16_t targ);
    void set_target_spc_pressure(uint16_t targ);
    void set_target_tcc_pressure(uint16_t targ);
    uint16_t get_targ_mpc_pressure();
    uint16_t get_targ_spc_pressure();
    uint16_t get_targ_tcc_pressure();
    uint16_t get_targ_spc_current();
    uint16_t get_targ_mpc_current();
    void disable_spc();

    PressureManager(SensorData* sensor_ptr, uint16_t max_torque);

    /**
     * @brief Get the shift data object for the requested shift
     * 
     * @param shift_firmness Firmness of the shift (higher = firmer shift)
     * @param shift_speed Speed of the shift (higher = faster shift)
     * @return ShiftData 
     */
    ShiftData get_shift_data(SensorData* sensors, ProfileGearChange shift_request, ShiftCharacteristics chars, int max_rated_torque, uint16_t curr_mpc);

    void perform_adaptation(SensorData* sensors, ProfileGearChange change, ShiftReport* response, bool is_valid_rpt) {
        if (this->adapt_map != nullptr) { 
            this->adapt_map->perform_adaptation(sensors, response, change, is_valid_rpt, this->gb_max_torque);
        }
    }

    void save() {
        if (this->adapt_map != nullptr) { 
            this->adapt_map->save(); 
        }
    }

    uint16_t find_working_mpc_pressure(GearboxGear curr_g, SensorData* sensors, int max_rated_torque);

    float get_tcc_temp_multiplier(int atf_temp);
private:
     /**
     * Returns the estimated PWM to send to either SPC or MPC solenoid
     * Based on the requested pressure that is needed withint either pressure rail.
     */
    uint16_t get_p_solenoid_current(uint16_t request_mbar, bool is_spc);

    /**
     * Returns the estimated PWM to send to the TCC solenoid
     * based on the requested pressure that is needed
     */
    uint16_t get_tcc_solenoid_pwm_duty(uint16_t request_mbar);

    SensorData* sensor_data;
    AdaptationMap* adapt_map;

    uint16_t req_tcc_pressure;
    uint16_t req_spc_pressure;
    uint16_t req_mpc_pressure;
    uint16_t req_current_spc;
    uint16_t req_current_mpc;
    TcmMap* pressure_pwm_map;
    TcmMap* tcc_pwm_map;
    TcmMap* hold2_time_map;
    uint16_t gb_max_torque;

    void make_hold3_data(ShiftPhase* dest, ShiftPhase* prev, ShiftCharacteristics chars, ProfileGearChange change, uint16_t curr_mpc);
    void make_torque_data(ShiftPhase* dest, ShiftPhase* prev, ShiftCharacteristics chars, ProfileGearChange change, uint16_t curr_mpc);
    void make_overlap_data(ShiftPhase* dest, ShiftPhase* prev, ShiftCharacteristics chars, ProfileGearChange change, uint16_t curr_mpc);

};

#endif