#ifndef PRESSURE_MANAGER_H
#define PRESSURE_MANAGER_H

#include <common_structs.h>
#include "tcu_maths.h"
#include "profiles.h"
#include "adaptation/shift_adaptation.h"
#include "nvs/eeprom_config.h"
#include "stored_map.h"

typedef struct {
    uint16_t fill_time;
    uint16_t fill_pressure_on_clutch;
    uint16_t fill_pressure_off_clutch;
} PrefillData;

typedef struct {
    uint16_t hold_time;
    uint16_t ramp_time;
} PressureStageTiming;

class PressureManager {

public:

    [[noreturn]]
    static void start_pm_internal(void *_this) {
        static_cast<PressureManager*>(_this)->controller_loop();
    }

    void set_shift_circuit(ShiftCircuit ss, bool enable);
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
     * @brief Set the both target SPC pressure (Shift pressure) and MPC pressure (Modulating pressure)
     * 
     * @param mpc Target MPC pressure to achieve in mBar
     * @param spc Target SPC pressure to achieve in mBar
     * 
     * This function is faster to execute than calling `set_target_mpc_pressure` and `set_target_spc_pressure`
     */
    void set_target_spc_and_mpc_pressure(uint16_t mpc, uint16_t spc);

    /**
     * @brief Set the target TCC pressure (Torque converter)
     * 
     * @param targ Target TCC pressure to achieve in mBar
     */
    void set_target_tcc_pressure(uint16_t targ);

    /**
     * @brief Sets the target for working line pressure (In gears)
    */
   void set_target_line_pressure(uint16_t targ);

    uint16_t get_targ_line_pressure(void);
    uint16_t get_targ_mpc_clutch_pressure(void) const;
    uint16_t get_targ_spc_clutch_pressure(void) const;
    uint16_t get_targ_mpc_solenoid_pressure(void) const;
    uint16_t get_targ_spc_solenoid_pressure(void) const;
    uint16_t get_targ_tcc_pressure(void) const;
    uint8_t get_active_shift_circuits(void) const;

    /**
     * Force SPC solenoid to turn off
    */
    void set_spc_p_max();

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

    uint16_t find_working_mpc_pressure(GearboxGear curr_g);
    
    float get_tcc_temp_multiplier(int atf_temp);

    PrefillData make_fill_data(ProfileGearChange change);
    PressureStageTiming get_max_pressure_timing();
    StoredMap* get_pcs_map(void);
    StoredMap* get_tcc_pwm_map(void);
    StoredMap* get_working_map(void);
    StoredMap* get_fill_time_map(void);
    StoredMap* get_fill_pressure_map(void);
private:

    void controller_loop();

     /**
     * Returns the estimated PWM to send to either SPC or MPC solenoid
     * Based on the requested pressure that is needed withint either pressure rail.
     */
    uint16_t get_p_solenoid_current(uint16_t request_mbar) const;

    /**
     * Returns the estimated PWM to send to the TCC solenoid
     * based on the requested pressure that is needed
     */
    uint16_t get_tcc_solenoid_pwm_duty(uint16_t request_mbar) const;

    SensorData* sensor_data;
    // At the clutch
    uint16_t req_tcc_clutch_pressure;
    // At the engaging clutch (When shifting)
    uint16_t req_spc_clutch_pressure;
    // At the releasing clutch (When shifting)
    uint16_t req_mpc_clutch_pressure;
    // For all clutches
    uint16_t req_line_pressure;

    // At SPC solenoid
    uint16_t commanded_spc_pressure;
    // At MPC solenoid
    uint16_t commanded_mpc_pressure;
    // Shift circuit currently open
    ShiftCircuit currently_open_circuit;

    StoredMap* pressure_pwm_map;
    StoredMap* tcc_pwm_map;
    StoredMap* mpc_working_pressure;
    StoredMap* hold2_time_map;
    StoredMap* hold2_pressure_map;
    uint16_t gb_max_torque;
    uint64_t ss_1_2_open_time = 0;
    uint64_t ss_2_3_open_time = 0;
    uint64_t ss_3_4_open_time = 0;
};

extern PressureManager* pressure_manager;

#endif