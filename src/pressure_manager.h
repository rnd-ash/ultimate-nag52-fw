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
} PrefillData;

typedef struct {
    uint16_t hold_time;
    uint16_t ramp_time;
} PressureStageTiming;

struct ShiftPressures {
    // At the applying clutch
    float on_clutch;
    // At the releasing clutch
    float off_clutch;
    // Pressure on the modulating side of the overlap slider
    float overlap_mod;
    // Pressure on the shift side of the overlap slider
    float overlap_shift;
    // At the shift solenoid
    float shift_sol_req;
    // At the modulating solenoid
    float mod_sol_req;
};

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
    CircuitInfo get_basic_shift_data(GearboxConfiguration* cfg, ProfileGearChange shift_request, ShiftCharacteristics chars);
    uint16_t find_working_mpc_pressure(GearboxGear curr_g);
    uint16_t find_working_pressure_for_clutch(GearboxGear gear, Clutch clutch, uint16_t abs_torque_nm, bool clamp_to_min_mpc = true);
    uint16_t calc_max_torque_for_clutch(GearboxGear gear, Clutch clutch, uint16_t pressure);
    void update_pressures(GearboxGear current_gear);

    PrefillData make_fill_data(ProfileGearChange change);
    PressureStageTiming get_max_pressure_timing();
    StoredMap* get_tcc_pwm_map(void);
    StoredMap* get_fill_time_map(void);
    StoredMap* get_fill_pressure_map(void);
    uint16_t get_shift_regulator_pressure(void);

    float calculate_centrifugal_force_for_clutch(Clutch clutch, uint16_t input, uint16_t rear_sun);

    void register_shift_pressure_data(ShiftPressures* p) {
        this->ptr_shift_pressures = p;
    }

    ShiftPressures get_shift_pressures_now() {
        ShiftPressures ret{};
        if (nullptr == this->ptr_shift_pressures) {
            memset(&ret, 0x00, sizeof(ShiftPressures));
        } else {
            memcpy(&ret, this->ptr_shift_pressures, sizeof(ShiftPressures));
        }
        return ret;
    }
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
    ShiftPressures* ptr_shift_pressures = nullptr;
};

extern PressureManager* pressure_manager;

#endif