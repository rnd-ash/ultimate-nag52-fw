#ifndef PRESSURE_MANAGER_H
#define PRESSURE_MANAGER_H

#include <common_structs.h>
#include "tcu_maths.h"
#include "profiles.h"
#include "adaptation/shift_adaptation.h"
#include "nvs/eeprom_config.h"
#include "stored_map.h"

enum class Clutch {
    K1 = 1,
    K2 = 2,
    K3 = 3,
    B1 = 4,
    B2 = 5,
    B3 = 6 // Reverse ONLY
};

typedef struct {
    uint16_t fill_time;
    uint16_t fill_pressure;
} PrefillData;

typedef struct {
    uint16_t hold_time;
    uint16_t ramp_time;
} PressureStageTiming;

class PressureManager {

public:

    /**
     * @brief Set the target MPC pressure (Modulating pressure)
     * 
     * @param targ Target MPC pressure to achieve in mBar
     */
    void set_target_mpc_pressure(uint16_t targ);

    /**
     * @brief Set the target SPC pressure (Shift pressure)
     * 
     * @param targ Target SPC pressure to achieve in mBar
     */
    void set_target_spc_pressure(uint16_t targ);

    /**
     * @brief Set the target TCC pressure (Torque converter)
     * 
     * @param targ Target TCC pressure to achieve in mBar
     */
    void set_target_tcc_pressure(uint16_t targ);
    uint16_t get_targ_mpc_pressure(void);
    uint16_t get_targ_spc_pressure(void);
    uint16_t get_targ_tcc_pressure(void);
    uint16_t get_targ_spc_current(void);
    uint16_t get_targ_mpc_current(void);
    void disable_spc(void);

    PressureManager(SensorData* sensor_ptr, uint16_t max_torque);

    /**
     * @brief Get the shift data object for the requested gear change
     * 
     * @param shift_request Which gear shift is being requested
     * @param chars Shift requested characteristics
     * @param curr_mpc Current MPC working pressure at the time of shift
     * @return ShiftData 
     */
    ShiftData get_basic_shift_data(GearboxConfiguration* cfg, ProfileGearChange shift_request, ShiftCharacteristics chars);

    /**
     * @brief Reset adaptation data
     * 
     * @return true if adaptation reset was OK
     * @return false if adaptation reset failed
     */
    esp_err_t diag_reset_adaptation(void) {
        bool result = false;
        if (this->pressure_adapt_system != nullptr) { 
            this->pressure_adapt_system->reset();
            result = true;
        }
        return result;
    }

    /**
     * @brief Called after every gear change to try and better improve future shifts
     * 
     * @param prefill_sensors Sensor data from just before the fill stage
     * @param change Gear change executed
     * @param response The report of the gear change
     * @param is_valid_rpt If the response is valid or not (Invalid would be due to a shift at stantstill)
     */
    void perform_adaptation(SensorData* prefill_sensors, ProfileGearChange change, ShiftReport* response, bool is_valid_rpt) {
        if (this->pressure_adapt_system != nullptr) { 
            //this->pressure_adapt_system->perform_adaptation(prefill_sensors, response, change, is_valid_rpt, this->gb_max_torque);
        }
    }

    /**
     * @brief Save adaptation data to NVS EEPROM
     * 
     */
    void save(void) {
        if (this->pressure_adapt_system != nullptr) { 
            this->pressure_adapt_system->save(); 
        }
    }

    uint16_t find_working_mpc_pressure(GearboxGear curr_g);
    
    float get_tcc_temp_multiplier(int atf_temp);

    PrefillData make_fill_data(ProfileGearChange change);
    //void make_torque_and_overlap_data(ShiftPhase* dest_torque, ShiftPhase* dest_overlap, ShiftPhase* prev, ShiftCharacteristics chars, ProfileGearChange change, uint16_t curr_mpc);
    //void make_max_p_data(ShiftPhase* dest, ShiftPhase* prev, ShiftCharacteristics chars, ProfileGearChange change, uint16_t curr_mpc);
    PressureStageTiming get_max_pressure_timing();
    Clutch get_clutch_to_release(ProfileGearChange change);
    Clutch get_clutch_to_apply(ProfileGearChange change);
    StoredMap* get_pcs_map(void);
    StoredMap* get_tcc_pwm_map(void);
    StoredMap* get_working_map(void);
    StoredMap* get_fill_time_map(void);
    StoredMap* get_fill_pressure_map(void);
    uint16_t get_max_rated_torque(void) {
        return this->gb_max_torque;
    }

    uint16_t get_mpc_hold_adder(Clutch to_apply);

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
    ShiftAdaptationSystem* pressure_adapt_system;
    uint16_t req_tcc_pressure;
    uint16_t req_spc_pressure;
    uint16_t req_mpc_pressure;
    uint16_t req_current_spc;
    uint16_t req_current_mpc;
    StoredMap* pressure_pwm_map;
    StoredMap* tcc_pwm_map;
    StoredMap* mpc_working_pressure;
    StoredMap* hold2_time_map;
    StoredMap* hold2_pressure_map;
    uint16_t gb_max_torque;
};

extern PressureManager* pressure_manager;

#endif