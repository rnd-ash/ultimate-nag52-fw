#ifndef PRESSURE_MANAGER_H
#define PRESSURE_MANAGER_H

#include <common_structs.h>
#include "tcu_maths.h"
#include "profiles.h"
#include "adaptation/shift_adaptation.h"
#include "nvs/eeprom_config.h"
#include "stored_map.h"
#include "sensors.h"
#include "nvs/module_settings.h"
#include "lookuptable.h"

typedef struct {
    uint16_t fill_time;
    uint16_t fill_pressure_on_clutch;
    uint16_t low_fill_pressure_on_clutch;
    uint16_t fill_pressure_off_clutch;
} PrefillData;

typedef struct {
    uint16_t hold_time;
    uint16_t ramp_time;
} PressureStageTiming;

class PressureManager {

public:

    /**
     * @brief Toggle the state of a shift solenoid
     * @param ss Shifting hydralic circuit to engage
     * @param enable Set circuit state to on
     */
    void set_shift_circuit(ShiftCircuit ss, bool enable);

    /**
     * @brief Set the target modulating pressure of the gearbox. 
     * This pressure affects the actively engaged clutches in any gear, 
     * as well as hydralic elements in the valve body
     * 
     * @param targ Target Working pressure to achieve in mBar
     */
    void set_target_modulating_pressure(uint16_t targ);

    /**
     * @brief Set the target Shift pressure clutch pressure.
     * Via means of the overlap valve, when a shift command solenoid is engaged,
     * this pressure is sent to the engaging clutch in the gearbox
     * 
     * @param targ Target shift pressure to achieve in mBar
     */
    void set_target_shift_pressure(uint16_t targ);

    /**
     * @brief Set the target TCC pressure (Torque converter)
     * 
     * @param targ Target TCC pressure to achieve in mBar
     */
    void set_target_tcc_pressure(uint16_t targ);

    uint16_t get_max_solenoid_pressure();

    uint16_t get_spring_pressure(Clutch c);

    uint16_t get_calc_line_pressure(void) const;
    uint16_t get_calc_inlet_pressure(void) const;
    uint16_t get_input_shift_pressure(void) const;
    uint16_t get_input_modulating_pressure(void) const;
    uint16_t get_corrected_spc_pressure(void) const;
    uint16_t get_corrected_modulating_pressure(void) const;
    uint16_t get_targ_tcc_pressure(void) const;

    uint8_t get_active_shift_circuits(void) const;

    /**
     * Force SPC solenoid to turn off
    */
    void set_spc_p_max(void);

    PressureManager(SensorData* sensor_ptr, uint16_t max_torque);

    /**
     * @brief Get the shift data object for the requested gear change
     * 
     * @param shift_request Which gear shift is being requested
     * @param chars Shift requested characteristics
     * @param curr_mpc Current MPC working pressure at the time of shift
     * @return ShiftData 
     */
    void notify_shift_end();
    ShiftData get_basic_shift_data(GearboxConfiguration* cfg, ProfileGearChange shift_request, ShiftCharacteristics chars);
    uint16_t find_working_mpc_pressure(GearboxGear curr_g);
    uint16_t find_working_pressure_for_clutch(GearboxGear gear, Clutch clutch, uint16_t abs_torque_nm, bool clamp_to_min_mpc = true);
    void update_pressures(GearboxGear current_gear);

    PrefillData make_fill_data(ProfileGearChange change);
    PressureStageTiming get_max_pressure_timing();
    StoredMap* get_tcc_pwm_map(void);
    StoredMap* get_fill_time_map(void);
    StoredMap* get_fill_pressure_map(void);
    uint16_t get_shift_regulator_pressure(void);

    const VBY_SETTINGS* vby_settings() { return this->valve_body_settings; }
private:

    uint16_t calc_working_pressure(GearboxGear current_gear, uint16_t in_mpc, uint16_t in_spc);
    uint16_t calc_input_pressure(uint16_t working_pressure);
    float calc_inlet_factor(uint16_t inlet_pressure);

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

    uint8_t shift_circuit_flag = 0;

    SensorData* sensor_data;
    
    // Shift pressure
    uint16_t target_shift_pressure = 0;
    // Modulating pressure
    uint16_t target_modulating_pressure = 0;
    // TCC pressure
    uint16_t target_tcc_pressure = 0;
    uint16_t corrected_spc_pressure = 0;
    uint16_t corrected_mpc_pressure = 0;

    uint16_t calculated_working_pressure = 0;
    uint16_t calculated_inlet_pressure = 0;

    // Shift circuit currently open
    ShiftCircuit currently_open_circuit;
    const int16_t* clutch_friction_coefficient_map;
    const int16_t* clutch_spring_release_map;
    const uint8_t* heaviest_loaded_clutch_idx_map;
    LookupMap* pressure_pwm_map;
    StoredMap* tcc_pwm_map;
    StoredMap* fill_time_map;
    StoredMap* fill_pressure_map;
    StoredMap* fill_low_pressure_map;
    uint16_t gb_max_torque;
    uint8_t c_gear = 0;
    uint8_t t_gear = 0;
    uint16_t solenoid_max_pressure = 0;

    bool init_ss_recovery = false;
    uint64_t last_ss_on_time = 0;

    VBY_SETTINGS* valve_body_settings;
};

extern PressureManager* pressure_manager;

#endif